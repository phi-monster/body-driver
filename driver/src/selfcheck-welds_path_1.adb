with Selfmap.Graph;
with Readings;
with Ada.Exceptions;
separate (Selfcheck)
procedure Welds_Path_1 is
   use Ada.Numerics.Long_Elementary_Functions;
   use type Selfmap.Group_Role;
   use type Readings.Eye_Verdict;
   --  路 1 的焊点(大并行.md §5 路 1):每条写清"错了会是什么病",带一颗牙(去掉那一改就红)
   function Vec (A : Bytes.Int_Vectors.Vector) return Ints is (A);
   function Same (A, B : Ints) return Boolean is
     (Natural (A.Length) = Natural (B.Length) and then (for all I in 0 .. Natural (A.Length) - 1 => A (I) = B (I)));
   function Show (V : Ints) return String is
      R : Unbounded_String;
   begin
      for I in 0 .. Natural (V.Length) - 1 loop
         Append (R, (if I > 0 then "," else "") & Codec.Img (V (I)));
      end loop;
      return "[" & To_String (R) & "]";
   end Show;
   --  一张只填了身体图那几个旧字段的 Body_Map(别的字段不影响这几问)
   function Map_Of (Arms, Cams : Natural; Jaws, On_Arm : Ints) return Selfmap.Body_Map is
      M : Selfmap.Body_Map;
   begin
      M.Arms := Arms; M.N_Cams := Cams; M.Per_Arm := Chan.Per_Arm; M.Channels := Arms * Chan.Per_Arm;
      M.Jaws := Jaws; M.Cam_On_Arm := On_Arm;
      return M;
   end Map_Of;
   Empty : constant Ints := Int_Vectors.Empty_Vector;

   --  ══ I1 认组的假身体:锁步里一只假手真跑 Selfmap.Measure_Idle + Jointboot.Find_Arms,主线程当身体 ══
   --  身体 = 几组数(state.<名字>;收命令的另有 action.<名字> 回声)+ 几只眼。眼长在哪几组上:那几组一动,整幅画面跟着挪(按读数位移 × 增益,
   --  亚像素也算:纹理是解析的);看得见哪几组带动的一块:画面底下一条 12 × 10 的小块,只跟着那一组挪。读数瞬时到命令(夹在范围里)
   Fw : constant := 64;
   Fh : constant := 48;
   type Fake_Group is record
      Name : Unbounded_String;
      Command : Boolean := True;      --  收命令;False = 只是读数(观测里没有回声)
      Echo : Boolean := True;         --  观测里另有 action.<名字>(上一条命令)
      V0 : Floats;                    --  起点读数
      Lo, Hi : Long_Float := 0.0;     --  推得到的范围(Lo >= Hi = 不限)
      Frozen : Boolean := False;      --  推不动:读数、身体都不跟
      Lie_Neg : Boolean := False;     --  往起点以下推:读数照命令走,身体没动(画面不变)
      Follows : Integer := -1;        --  只是读数:第 0 个数跟着第几组的位移变
      Gain : Long_Float := 400.0;     --  读数位移一个单位,画面挪几像素
   end record;
   package FG_Vectors is new Ada.Containers.Vectors (Natural, Fake_Group);
   type Fake_Cam is record
      Ego : Ints;                     --  长在这几组上
      Parts : Ints;                   --  看得见这几组各自带动的一块
   end record;
   package FC_Vectors is new Ada.Containers.Vectors (Natural, Fake_Cam);
   type Fake_Result is record
      Ok : Boolean := False;
      Arms : Jointboot.Arm_Vectors.Vector;
      World : Integer := -1;
      Map : Selfmap.Body_Map;
      Lay, Lay0 : Layout.Body_Layout;  --  量完的布局 / 按形状认的旧布局(牙用)
      Beats : Natural := 0;
      Pushed : Strs;                   --  收到过和读数不一样的命令的名字
      Last : Msgpack.Doc;
      Last_Obs : Integer := -1;
   end record;
   function G_Of (Name : String; N : Natural; V : Long_Float; Command : Boolean := True; Lo, Hi : Long_Float := 0.0;
                  Gain : Long_Float := 400.0) return Fake_Group is
      G : Fake_Group;
   begin
      G.Name := To_Unbounded_String (Name); G.Command := Command; G.Echo := Command; G.Lo := Lo; G.Hi := Hi; G.Gain := Gain;
      for I in 1 .. N loop
         G.V0.Append (V);
      end loop;
      return G;
   end G_Of;
   --  身体报的位姿那一组(只是读数,第 0 个数跟着第 Follows 组走;四元数 w = 1,按形状认得出是位姿)
   function Pose_Of_Group (Name : String; Follows : Natural) return Fake_Group is
      G : Fake_Group := G_Of (Name, 0, 0.0, Command => False);
      P : constant array (0 .. 6) of Long_Float := [0.3, 0.1, 0.2, 1.0, 0.0, 0.0, 0.0];
   begin
      for X of P loop
         G.V0.Append (X);
      end loop;
      G.Follows := Follows;
      return G;
   end Pose_Of_Group;
   function Cam_Of (Ego, Parts : Ints) return Fake_Cam is (Ego => Ego, Parts => Parts);

   function Run_Fake (Gs : FG_Vectors.Vector; Cs : FC_Vectors.Vector) return Fake_Result is
      Ng : constant Natural := Natural (Gs.Length);
      type Vals is array (0 .. Ng - 1) of Floats;
      V, Phys, Sent : Vals;
      R : Fake_Result;
      Lk : Plug.Link;
      Fr0 : Plug.Frame;
      Mp : Selfmap.Body_Map;
      Ok_F : Boolean := False;
      Arms_F : Jointboot.Arm_Vectors.Vector;
      World_F : Integer := -1;
      Cap : constant := 4000;   --  拍数上限(次数,防万一:到了就让插头说线断了,假手照样收得了尾)
      task type Probe_Hand;
      task body Probe_Hand is
         Fr : Plug.Frame := Fr0;
         Okm : Boolean;
      begin
         Lockstep.Begin_Hand (0);
         Mp.N_Cams := Natural (Cs.Length);
         Selfmap.Measure_Idle (Lk, Fr, Mp, Okm);
         if Okm then
            Jointboot.Find_Arms (Lk, Fr, Mp, Arms_F, World_F, Ok_F);
         end if;
         Lockstep.Done;
      exception
         when E : others =>
            Put_Line ("  假手出错:" & Ada.Exceptions.Exception_Information (E));
            Lockstep.Done;
      end Probe_Hand;
      function Disp (G : Natural) return Long_Float is
         S : Long_Float := 0.0;
      begin
         for I in 0 .. Natural (Phys (G).Length) - 1 loop
            S := S + (Phys (G) (I) - Gs (G).V0 (I));
         end loop;
         return S;
      end Disp;
      function World_Px (U, W : Long_Float) return Long_Float is
        (128.0 + 50.0 * Sin (0.9 * U + 0.4 * W) + 35.0 * Sin (0.37 * U - 1.1 * W) + 20.0 * Sin (1.7 * U + 0.2 * W));
      function Part_Px (U, W : Long_Float) return Long_Float is (128.0 + 80.0 * Sin (1.3 * U) * Cos (0.9 * W));
      procedure Put_Nums (S : in out Buf; X : Floats) is
      begin
         Msgpack.Put_Array (S, Natural (X.Length));
         for Y of X loop
            Msgpack.Put_Float (S, Y);
         end loop;
      end Put_Nums;
      procedure Build (S : in out Buf) is
         Nc : constant Natural := Natural (Cs.Length);
         N_Action : Natural := 0;
      begin
         S.Clear;
         for G of Gs loop
            if G.Command and then G.Echo then
               N_Action := N_Action + 1;
            end if;
         end loop;
         Msgpack.Put_Map (S, 1);
         Msgpack.Put_Str (S, "obs");
         Msgpack.Put_Map (S, (if N_Action > 0 then 3 else 2));
         Msgpack.Put_Str (S, "vision"); Msgpack.Put_Map (S, Nc);
         for C in 0 .. Nc - 1 loop
            declare
               Ox : Long_Float := 0.0;
               Px : Buf;
            begin
               for E of Cs (C).Ego loop
                  Ox := Ox + Gs (Natural (E)).Gain * Disp (Natural (E));
               end loop;
               for Y in 0 .. Fh - 1 loop
                  for X in 0 .. Fw - 1 loop
                     declare
                        Val : Long_Float := World_Px (Long_Float (X) - Ox, Long_Float (Y) - 0.5 * Ox);
                     begin
                        for K in 0 .. Natural (Cs (C).Parts.Length) - 1 loop
                           declare
                              Pg : constant Natural := Natural (Cs (C).Parts (K));
                              X0 : constant Natural := 2 + 15 * K;
                           begin
                              if X >= X0 and then X < X0 + 12 and then Y >= Fh - 12 and then Y < Fh - 2 then
                                 Val := Part_Px (Long_Float (X) - Gs (Pg).Gain * Disp (Pg), Long_Float (Y));
                              end if;
                           end;
                        end loop;
                        declare
                           use type Interfaces.Unsigned_32;
                           --  相机噪声:每拍每个像素 −1 / 0 / +1 个灰阶(按像素和拍号散列;静止时量得出地板,极小的一推看不见)
                           Hs : constant Interfaces.Unsigned_32 := (Interfaces.Unsigned_32 (X) * 73856093) xor (Interfaces.Unsigned_32 (Y) * 19349663)
                             xor (Interfaces.Unsigned_32 (Lk.Seq mod 65536) * 83492791) xor (Interfaces.Unsigned_32 (C) * 2654435761);
                           Nz : constant Long_Float := Long_Float (Integer ((Hs / 7) mod 3) - 1);
                           B : constant U8 := U8 (Long_Float'Max (0.0, Long_Float'Min (255.0, Long_Float'Rounding (Val + Nz))));
                        begin
                           Px.Append (B); Px.Append (B); Px.Append (B);
                        end;
                     end;
                  end loop;
               end loop;
               --  同 RoboDojo:画面旁边另挂一个 shape(相机自己的数,不是身体的一组读数)
               Msgpack.Put_Str (S, "cam_" & Codec.Img (C)); Msgpack.Put_Map (S, 2);
               Msgpack.Put_Str (S, "color"); Msgpack.Put_Map (S, 4);
               Msgpack.Put_Str (S, "nd"); Msgpack.Put_Bool (S, True);
               Msgpack.Put_Str (S, "type"); Msgpack.Put_Str (S, "|u1");
               Msgpack.Put_Str (S, "shape"); Msgpack.Put_Array (S, 3); Msgpack.Put_Int (S, Fh); Msgpack.Put_Int (S, Fw); Msgpack.Put_Int (S, 3);
               Msgpack.Put_Str (S, "data"); Msgpack.Put_Bin (S, Px, 0, Natural (Px.Length));
               Msgpack.Put_Str (S, "shape"); Msgpack.Put_Array (S, 3); Msgpack.Put_Int (S, Fh); Msgpack.Put_Int (S, Fw); Msgpack.Put_Int (S, 3);
            end;
         end loop;
         Msgpack.Put_Str (S, "state"); Msgpack.Put_Map (S, Ng);
         for G in 0 .. Ng - 1 loop
            Msgpack.Put_Str (S, To_String (Gs (G).Name)); Put_Nums (S, V (G));
         end loop;
         if N_Action > 0 then
            Msgpack.Put_Str (S, "action"); Msgpack.Put_Map (S, N_Action);
            for G in 0 .. Ng - 1 loop
               if Gs (G).Command and then Gs (G).Echo then
                  Msgpack.Put_Str (S, To_String (Gs (G).Name)); Put_Nums (S, Sent (G));
               end if;
            end loop;
         end if;
      end Build;
      procedure Apply (G : Natural; Q : Floats) is
      begin
         if not Gs (G).Command then
            return;
         end if;
         Sent (G) := Q;
         if Gs (G).Frozen then
            return;
         end if;
         for I in 0 .. Natural'Min (Natural (Q.Length), Natural (V (G).Length)) - 1 loop
            declare
               X : Long_Float := Q (I);
            begin
               if Gs (G).Lo < Gs (G).Hi then
                  X := Long_Float'Max (Gs (G).Lo, Long_Float'Min (Gs (G).Hi, X));
               end if;
               V (G).Replace_Element (I, X);
               Phys (G).Replace_Element (I, (if Gs (G).Lie_Neg and then X < Gs (G).V0 (I) then Gs (G).V0 (I) else X));
            end;
         end loop;
      end Apply;
      procedure Settle_Followers is
      begin
         for G in 0 .. Ng - 1 loop
            if Gs (G).Follows >= 0 and then not V (G).Is_Empty then
               V (G).Replace_Element (0, Gs (G).V0 (0) + Disp (Natural (Gs (G).Follows)));
               Phys (G) := V (G);
            end if;
         end loop;
      end Settle_Followers;
      S : Buf;
      D : Msgpack.Doc;
   begin
      for G in 0 .. Ng - 1 loop
         V (G) := Gs (G).V0; Phys (G) := Gs (G).V0; Sent (G) := Gs (G).V0;
      end loop;
      Build (S);
      if not Msgpack.Decode (S, D) then
         return R;
      end if;
      Lk.Last := D; Lk.Last_Obs := Msgpack.Key (D, 0, "obs");
      Layout.Recognise (D, Lk.Last_Obs, Lk.Lay);
      R.Lay0 := Lk.Lay;
      Layout.Probe_Mode (Lk.Lay);
      Plug.Frame_Of (Lk, Fr0);
      Lockstep.Clear;
      Plug.Lock_Begin;
      declare
         Hd : Probe_Hand;
      begin
         Lockstep.Start (0, Hd'Identity);
         loop
            Lockstep.Run (0);
            exit when Lockstep.Finished (0);
            R.Beats := R.Beats + 1;
            declare
               Mg : constant Plug.Cmd := Plug.Lock_Merged;
            begin
               for K in 0 .. Natural'Min (Natural (Mg.Groups.Length), Natural (Mg.Qs.Length)) - 1 loop
                  if Mg.Groups (K) >= 0 and then Mg.Groups (K) < Natural (Lk.Lay.Joints.Length) then
                     declare
                        Nm : constant String := Layout.Last_Seg (Lk.Lay.Joints (Natural (Mg.Groups (K))));
                        Q : constant Floats := Mg.Qs (K);
                     begin
                        for G in 0 .. Ng - 1 loop
                           if To_String (Gs (G).Name) = Nm then
                              if not R.Pushed.Contains (Nm)
                                and then (for some I in 0 .. Natural'Min (Natural (Q.Length), Natural (V (G).Length)) - 1 => Q (I) /= V (G) (I))
                              then
                                 R.Pushed.Append (Nm);
                              end if;
                              Apply (G, Q);
                           end if;
                        end loop;
                     end;
                  end if;
               end loop;
            end;
            Plug.Lock_Begin;   --  这一拍的命令用过就清(布局换了以后旧下标不许再按新布局读)
            Settle_Followers;
            Build (S);
            if Msgpack.Decode (S, D) then
               Lk.Last := D; Lk.Last_Obs := Msgpack.Key (D, 0, "obs");
            end if;
            Lk.Seq := Lk.Seq + 1;
            declare
               Fr : Plug.Frame;
            begin
               Plug.Frame_Of (Lk, Fr);
               Plug.Lock_Feed (Fr, Ok => R.Beats < Cap);
            end;
         end loop;
      end;
      Plug.Lock_End;
      Lockstep.Clear;
      R.Ok := Ok_F; R.Arms := Arms_F; R.World := World_F; R.Map := Mp; R.Lay := Lk.Lay; R.Last := Lk.Last; R.Last_Obs := Lk.Last_Obs;
      return R;
   end Run_Fake;
   --  量出来的身体图里路径是 Path 的那一组(-1 = 没有)
   function Grp (R : Fake_Result; Path : String) return Integer is
   begin
      for G in 0 .. Natural (R.Map.Groups.Length) - 1 loop
         if To_String (R.Map.Groups (G).Name) = Path then
            return Integer (G);
         end if;
      end loop;
      return -1;
   end Grp;
   function Role_Of (R : Fake_Result; Path : String) return Selfmap.Group_Role is
     (if Grp (R, Path) >= 0 then R.Map.Groups (Natural (Grp (R, Path))).Role else Selfmap.Unprobed);
   function Paths_Img (P : Layout.Paths) return String is
      T : Unbounded_String;
   begin
      for I in 0 .. Natural (P.Length) - 1 loop
         Append (T, (if I > 0 then "," else "") & Layout.Joined (P (I)));
      end loop;
      return "[" & To_String (T) & "]";
   end Paths_Img;
   function Role_Img (R : Selfmap.Group_Role) return String is (Selfmap.Group_Role'Image (R));
begin
   --  ── I1 身体图的通用问法(Selfmap.Graph):别路只问这几句,不按下标算通道号、不认"一条臂一只眼 / 至少一个合拢通道 / 只有一只不动的眼" ──
   --  ① x5 的样子(V1B78 身体文件里的字段:两条臂、三台相机、每条臂一个合拢通道、第 1 / 2 台是两只腕眼)
   --     ⇒ 答得和今天的字段一模一样,开机报告那一行逐字对。
   --  病:这几问和字段答得不一样 ⇒ 别路照着写的新代码在 x5 上悄悄换了通道 / 眼。
   --  牙:开机报告那一行照旧读字段(Integer'Image (Cam_On_Arm))⇒ 这一行对不上
   declare
      M : constant Selfmap.Body_Map := Map_Of (2, 3, Vec ([1, 1]), Vec ([1, 2]));
      Want_Say : constant String :=
        "身体图(通用问法):2 条臂 · 第 1 条臂:位姿通道 0,1,2,3,4,5、合拢通道 1 个、长在它上面的眼(相机号)1"
        & " · 第 2 条臂:位姿通道 6,7,8,9,10,11、合拢通道 1 个、长在它上面的眼(相机号)2"
        & " · 不长在任何臂上的眼(相机号)0 · 扛着全身走的组 没有";
      Ok_Arms : constant Boolean := Selfmap.Graph.Arm_Count (M) = 2;
      Ok_Pose : constant Boolean := Same (Selfmap.Graph.Pose_Channels (M, 0), Vec ([0, 1, 2, 3, 4, 5]))
                                    and then Same (Selfmap.Graph.Pose_Channels (M, 1), Vec ([6, 7, 8, 9, 10, 11]));
      Ok_Close : constant Boolean := Selfmap.Graph.Closing_Count (M, 0) = 1 and then Selfmap.Graph.Closing_Count (M, 1) = 1;
      Ok_Eyes : constant Boolean := Same (Selfmap.Graph.Eyes_On (M, 0), Vec ([1])) and then Same (Selfmap.Graph.Eyes_On (M, 1), Vec ([2]))
                                    and then Same (Selfmap.Graph.Eyes_Off_Arms (M), Vec ([0]));
      Ok_Carry : constant Boolean := Selfmap.Graph.Carrying_Groups (M).Is_Empty;
      Ok_Say : constant Boolean := Selfmap.Graph.Say (M) = Want_Say;
   begin
      Check (Ok_Arms and then Ok_Pose and then Ok_Close and then Ok_Eyes and then Ok_Carry and then Ok_Say,
             "身体图通用问法 · x5 的样子(两条臂、三台相机、各一只腕眼、各一个合拢通道):几条臂 " & Boolean'Image (Ok_Arms)
             & " · 位姿通道 " & Show (Selfmap.Graph.Pose_Channels (M, 0)) & Show (Selfmap.Graph.Pose_Channels (M, 1))
             & " · 合拢通道 " & Codec.Img (Selfmap.Graph.Closing_Count (M, 0)) & "/" & Codec.Img (Selfmap.Graph.Closing_Count (M, 1))
             & " · 长在臂上的眼 " & Show (Selfmap.Graph.Eyes_On (M, 0)) & Show (Selfmap.Graph.Eyes_On (M, 1))
             & " · 不长在臂上的眼 " & Show (Selfmap.Graph.Eyes_Off_Arms (M))
             & " · 扛着全身的组 " & Show (Selfmap.Graph.Carrying_Groups (M)) & " · 开机报告一行逐字对 " & Boolean'Image (Ok_Say)
             & (if Ok_Say then "" else "(念出来的:" & Selfmap.Graph.Say (M) & ")"));
   end;
   --  ② 三条臂、五台相机;第 2 条臂没有眼、也没有合拢通道,第 3 条臂五个合拢通道、眼是第 1 台;
   --     第 0、2、4 台不长在任何臂上;问第 4 条臂(没有)⇒ 空。
   --  病:第 3 条臂的通道号按两条臂算错;没有合拢通道的臂被当成有一个(DR1 / DR2 09-28:对着不存在的通道合空、清单里列出不存在的手指);
   --  几只不动的眼只认一只(世界相机 = 一只);问一条不存在的臂拿到下一段通道号(越界读 Amp)。
   --  牙:合拢通道按"至少一个"答(Selfmap.Measure 原来的 Max (1, …))⇒ 第 2 条臂 1 个;不长在臂上的眼只认一只 ⇒ [0];
   --     位姿通道不查"有没有这条臂" ⇒ 第 4 条臂 18..23
   declare
      M : constant Selfmap.Body_Map := Map_Of (3, 5, Vec ([1, 0, 5]), Vec ([3, -1, 1]));
      Ok_Pose : constant Boolean := Same (Selfmap.Graph.Pose_Channels (M, 2), Vec ([12, 13, 14, 15, 16, 17]))
                                    and then Selfmap.Graph.Pose_Channels (M, 3).Is_Empty;
      Ok_Close : constant Boolean := Selfmap.Graph.Closing_Count (M, 0) = 1 and then Selfmap.Graph.Closing_Count (M, 1) = 0
                                     and then Selfmap.Graph.Closing_Count (M, 2) = 5 and then Selfmap.Graph.Closing_Count (M, 3) = 0;
      Ok_Eyes : constant Boolean := Same (Selfmap.Graph.Eyes_On (M, 0), Vec ([3])) and then Selfmap.Graph.Eyes_On (M, 1).Is_Empty
                                    and then Same (Selfmap.Graph.Eyes_On (M, 2), Vec ([1])) and then Selfmap.Graph.Eyes_On (M, 3).Is_Empty;
      Ok_Off : constant Boolean := Same (Selfmap.Graph.Eyes_Off_Arms (M), Vec ([0, 2, 4]));
      Say : constant String := Selfmap.Graph.Say (M);
      Ok_Say : constant Boolean :=
        Ada.Strings.Fixed.Index (Say, "第 2 条臂:位姿通道 6,7,8,9,10,11、合拢通道 0 个、长在它上面的眼(相机号)没有") > 0
        and then Ada.Strings.Fixed.Index (Say, "第 3 条臂:位姿通道 12,13,14,15,16,17、合拢通道 5 个、长在它上面的眼(相机号)1") > 0
        and then Ada.Strings.Fixed.Index (Say, "不长在任何臂上的眼(相机号)0,2,4") > 0;
   begin
      Check (Ok_Pose and then Ok_Close and then Ok_Eyes and then Ok_Off and then Ok_Say,
             "身体图通用问法 · 三条臂、五台相机、第 2 条臂没眼没合拢通道:第 3 条臂位姿通道 " & Show (Selfmap.Graph.Pose_Channels (M, 2))
             & "、问第 4 条臂 " & Show (Selfmap.Graph.Pose_Channels (M, 3)) & "(该空)"
             & " · 合拢通道 " & Codec.Img (Selfmap.Graph.Closing_Count (M, 0)) & "/" & Codec.Img (Selfmap.Graph.Closing_Count (M, 1)) & "/"
             & Codec.Img (Selfmap.Graph.Closing_Count (M, 2)) & "(该 1/0/5)"
             & " · 长在臂上的眼 " & Show (Selfmap.Graph.Eyes_On (M, 0)) & Show (Selfmap.Graph.Eyes_On (M, 1)) & Show (Selfmap.Graph.Eyes_On (M, 2))
             & " · 不长在任何臂上的眼 " & Show (Selfmap.Graph.Eyes_Off_Arms (M)) & "(该 [0,2,4])"
             & " · 开机报告念全了 " & Boolean'Image (Ok_Say));
   end;
   --  ③ 身体文件没记每条臂几个合拢通道(09-30 以前写的:H4、DR2 那两份就没有 jaws)⇒ 每条臂 0 个;Act.Any_Fingers 问的就是它(Jaws_Of → Closing_Count),
   --  握区表里就算留着一只量过的手,也不说"我有手指"(开机照实说要重量)。
   --  病:没记就当 1 个(09-30 以前 Jaws_Of 的 `else 1`)⇒ 五指手装回来只剩第 0 号、没有抓握的身体被说成有手指。牙:没记按 1 个答 ⇒ 1/1、有手指
   declare
      C : Act.Context;
      Hd : Zone.Hand;
      Zv : Zone.Hand_Zone;
      Zero_Each, No_Fingers : Boolean;
   begin
      C.Map := Map_Of (2, 3, Empty, Vec ([1, 2]));
      Zv.Valid := True;
      Hd.Arm := 0; Hd.K := 0;
      Hd.Zones.Append (Zv);
      C.Hands.Append (Hd);
      Zero_Each := Selfmap.Graph.Closing_Count (C.Map, 0) = 0 and then Selfmap.Graph.Closing_Count (C.Map, 1) = 0;
      No_Fingers := not Act.Any_Fingers (C);
      Check (Zero_Each and then No_Fingers,
             "身体图通用问法 · 身体文件没记合拢通道:每条臂 " & Codec.Img (Selfmap.Graph.Closing_Count (C.Map, 0)) & "/"
             & Codec.Img (Selfmap.Graph.Closing_Count (C.Map, 1)) & " 个(该 0/0,不猜 1)· 握区表里留着一只量过的手也不说有手指 " & Boolean'Image (No_Fingers));
   end;
   --  ══ I1 认读数(Readings.Verdict):推一组读数一下,这只眼里"整幅在动"还是"只一块在动" ══
   --  合成的 64 × 48 画面(解析纹理,亚像素挪动也算):① 眼长在推的那组上 ⇒ 整幅挪 0.3 像素 ⇒ 整幅;② 只底下一块挪 ⇒ 一块;③ 没变 ⇒ 没看见;
   --  ④ 世界里左上那一片是白墙(没纹理),整幅挪 ⇒ 整幅;⑤ 腕眼右下那一象限被跟着眼不动的自己的手占满(人形 H4 第 1 只手那样),整幅挪 ⇒ 整幅;
   --  ⑥ 腕眼看的是稀疏的世界(两道竖线),整幅挪 0.3 像素只变一成的像素;头顶眼里那条胳膊一大块挪 2 像素,变了四分之一的像素
   --     ⇒ 腕眼整幅、头顶眼一块。
   --  病:眼认错 ⇒ 这条臂量不了运动学 / 合拢通道挂错臂;自己的手占了一个象限的腕眼推多大都判不出整幅 ⇒ 这条臂被当成零件。
   --  牙:⑤ 原来按象限判(每个象限都要过半才算整幅、有一个象限一格不变就算一块)⇒ 判成一块;
   --     ⑥ 原来的判法"变了的像素比例比第二名多一倍"⇒ 认成头顶眼长在这条臂上
   declare
      function World_At (U, W : Long_Float) return Long_Float is
        (128.0 + 50.0 * Sin (0.9 * U + 0.4 * W) + 35.0 * Sin (0.37 * U - 1.1 * W) + 20.0 * Sin (1.7 * U + 0.2 * W));
      type Scene is (Textured, Wall, Own_Hand, Sparse, Head_Arm);
      --  Ox = 整幅挪多少(像素);Dx = 底下那一块 / 头顶眼里胳膊那一块挪多少
      function Img (Sc : Scene; Ox, Dx : Long_Float) return Buf is
         B : Buf;
      begin
         for Y in 0 .. Fh - 1 loop
            for X in 0 .. Fw - 1 loop
               declare
                  U : constant Long_Float := Long_Float (X) - Ox;
                  W : constant Long_Float := Long_Float (Y) - 0.5 * Ox;
                  Val : Long_Float := World_At (U, W);
               begin
                  case Sc is
                     when Textured =>
                        if X >= 4 and then X < 16 and then Y >= Fh - 12 and then Y < Fh - 2 then
                           Val := 128.0 + 80.0 * Sin (1.3 * (Long_Float (X) - Dx)) * Cos (0.9 * Long_Float (Y));
                        end if;
                     when Wall =>
                        if U < Long_Float (Fw / 2) and then W < Long_Float (Fh / 2) then
                           Val := 200.0;   --  世界里的白墙:跟着整幅一起挪(墙边也挪)
                        end if;
                     when Own_Hand =>
                        if X >= Fw / 2 and then Y >= Fh / 2 then
                           Val := 128.0 + 80.0 * Sin (1.3 * Long_Float (X)) * Cos (0.9 * Long_Float (Y));   --  跟着眼不动的自己的手
                        end if;
                     when Sparse =>
                        declare
                           Du : constant Long_Float := abs (U - 32.0 * Long_Float'Rounding (U / 32.0));
                        begin
                           Val := 128.0 + 80.0 * Long_Float'Max (0.0, 1.0 - Du);   --  每 32 像素一道竖线,别处一片平
                        end;
                     when Head_Arm =>
                        if X >= 16 and then X < 48 and then Y >= 12 and then Y < 36 then
                           Val := World_At (Long_Float (X) - Dx, Long_Float (Y)) - 30.0;   --  胳膊:纹理和世界差不多,整块挪
                        end if;
                  end case;
                  B.Append (U8 (Long_Float'Max (0.0, Long_Float'Min (255.0, Long_Float'Rounding (Val)))));
               end;
            end loop;
         end loop;
         return B;
      end Img;
      Fl0 : constant Buf := Img (Textured, 0.0, 0.0);
      Fl : constant Picture.Floor_Map := Picture.Null_Floor (Fl0, Fl0, Fw, Fh, Picture.Min_Pixels (Fw, Fh));
      function V_Of (Sc : Scene; Ox, Dx : Long_Float) return Readings.Eye_Verdict is
         A : constant Buf := Img (Sc, 0.0, 0.0);
         B : constant Buf := Img (Sc, Ox, Dx);
      begin
         return Readings.Verdict (A, B, B, A, Fl, Fw, Fh);
      end V_Of;
      function Frac_Of (Sc : Scene; Ox, Dx : Long_Float) return Long_Float is
         A : constant Buf := Img (Sc, 0.0, 0.0);
         B : constant Buf := Img (Sc, Ox, Dx);
      begin
         return Picture.Fraction (Picture.Either (Picture.Moved (A, B, Fl), Picture.Moved (B, A, Fl)));
      end Frac_Of;
      V_Ego : constant Readings.Eye_Verdict := V_Of (Textured, 0.3, 0.0);
      V_Pat : constant Readings.Eye_Verdict := V_Of (Textured, 0.0, 0.3);
      V_None : constant Readings.Eye_Verdict := V_Of (Textured, 0.0, 0.0);
      V_Wall : constant Readings.Eye_Verdict := V_Of (Wall, 0.3, 0.0);
      V_Hand : constant Readings.Eye_Verdict := V_Of (Own_Hand, 0.3, 0.0);
      V_Sparse : constant Readings.Eye_Verdict := V_Of (Sparse, 0.3, 0.0);
      V_Head : constant Readings.Eye_Verdict := V_Of (Head_Arm, 0.0, 2.0);
      Fr_Wrist : constant Long_Float := Frac_Of (Sparse, 0.3, 0.0);
      Fr_Head : constant Long_Float := Frac_Of (Head_Arm, 0.0, 2.0);
      Old_Picks_Head : constant Boolean := Fr_Head >= 2.0 * Fr_Wrist;
      --  牙 ⑤:原来按象限判(同一份够强的格、变了的格)
      Old_Hand_Whole : Boolean := True;
      Old_Hand_Part : Boolean := False;
      Vh : Readings.Eye_Verdict;
      St, Ch : Readings.Quad_Counts;
   begin
      declare
         A : constant Buf := Img (Own_Hand, 0.0, 0.0);
         B : constant Buf := Img (Own_Hand, 0.3, 0.0);
      begin
         Readings.Verdict_Of (A, B, B, A, Fl, Fw, Fh, Vh, St, Ch);
      end;
      for Q in Readings.Quad_Counts'Range loop
         if St (Q) = 0 or else Ch (Q) < St (Q) - Ch (Q) then
            Old_Hand_Whole := False;
         end if;
         if St (Q) > 0 and then Ch (Q) = 0 then
            Old_Hand_Part := True;
         end if;
      end loop;
      Check (V_Ego = Readings.Whole and then V_Pat = Readings.Part and then V_None = Readings.Nothing and then V_Wall = Readings.Whole
             and then V_Hand = Readings.Whole and then V_Sparse = Readings.Whole and then V_Head = Readings.Part
             and then not Old_Hand_Whole and then Old_Hand_Part and then Old_Picks_Head,
             "认读数 · 眼里整幅 / 一块:整幅挪 0.3 像素 ⇒ " & Readings.Image (V_Ego) & " · 只底下一块挪 ⇒ " & Readings.Image (V_Pat)
             & " · 没变 ⇒ " & Readings.Image (V_None) & " · 世界里一面白墙、整幅挪 ⇒ " & Readings.Image (V_Wall)
             & " · 自己的手占了右下象限、整幅挪 ⇒ " & Readings.Image (V_Hand) & "(够强的格变了的:左上 " & Codec.Img (Ch (0)) & "/" & Codec.Img (St (0))
             & " 右上 " & Codec.Img (Ch (1)) & "/" & Codec.Img (St (1)) & " 左下 " & Codec.Img (Ch (2)) & "/" & Codec.Img (St (2))
             & " 右下 " & Codec.Img (Ch (3)) & "/" & Codec.Img (St (3)) & ")"
             & " · 稀疏的世界整幅挪 ⇒ " & Readings.Image (V_Sparse) & "、头顶眼里胳膊一大块挪 ⇒ " & Readings.Image (V_Head)
             & " · 牙:原来按象限判 ⇒ 自己的手那一推" & (if Old_Hand_Part then "判成一块" elsif Old_Hand_Whole then "(也判整幅,牙没咬上)" else "分不出")
             & ";原来按变了的比例(腕眼 " & Codec.Fmt (100.0 * Fr_Wrist, 1) & "%、头顶眼 " & Codec.Fmt (100.0 * Fr_Head, 1) & "%)比第二名多一倍 ⇒ "
             & (if Old_Picks_Head then "认成头顶眼长在这条臂上" else "(没认错,牙没咬上)"));
   end;
   --  ══ I1 布局:身体报的每一组数都是一组(不按"几个数、值在哪");画面旁边挂的 shape 不是身体的数;同名的另一组 = 命令的回声 ══
   --  一具没有夹爪、读数是度的身体(state.arm 6 个 30°、state.ee_pose、action.arm;一台相机,画面旁边挂着 shape)
   --  ⇒ 两组读数 + 一组回声,shape 不算;命令组 = arm 那一对;Missing = ""(开得了机)。
   --  病:没有夹爪不开机(DR1 / DR2 只好给无人机装一个假夹爪);读数是度(> 7)就认不出关节、拒绝开机;相机的 shape 被当成一组命令发回去(RoboDojo 不认的键直接报错)。
   --  牙:原来的 Missing(没认出关节 / 没认出夹爪 / 有分不开的读数 ⇒ 拒绝)这时说"没认出末端位姿也没认出关节角"或"没认出夹爪开度"
   declare
      Gs : FG_Vectors.Vector;
      Cs : FC_Vectors.Vector;
      S : Buf;
      D : Msgpack.Doc;
      Lay : Layout.Body_Layout;
   begin
      S.Clear;
      Msgpack.Put_Map (S, 1);
      Msgpack.Put_Str (S, "obs"); Msgpack.Put_Map (S, 3);
      Msgpack.Put_Str (S, "vision"); Msgpack.Put_Map (S, 1);
      Msgpack.Put_Str (S, "cam_0"); Msgpack.Put_Map (S, 2);
      Msgpack.Put_Str (S, "color"); Msgpack.Put_Map (S, 4);
      Msgpack.Put_Str (S, "nd"); Msgpack.Put_Bool (S, True);
      Msgpack.Put_Str (S, "type"); Msgpack.Put_Str (S, "|u1");
      Msgpack.Put_Str (S, "shape"); Msgpack.Put_Array (S, 3); Msgpack.Put_Int (S, 2); Msgpack.Put_Int (S, 2); Msgpack.Put_Int (S, 3);
      declare
         Px : Buf;
      begin
         for I in 1 .. 12 loop
            Px.Append (100);
         end loop;
         Msgpack.Put_Str (S, "data"); Msgpack.Put_Bin (S, Px, 0, 12);
      end;
      Msgpack.Put_Str (S, "shape"); Msgpack.Put_Array (S, 3); Msgpack.Put_Int (S, 2); Msgpack.Put_Int (S, 2); Msgpack.Put_Int (S, 3);
      Msgpack.Put_Str (S, "state"); Msgpack.Put_Map (S, 2);
      Msgpack.Put_Str (S, "arm"); Msgpack.Put_Array (S, 6);
      for I in 1 .. 6 loop
         Msgpack.Put_Float (S, 30.0);
      end loop;
      Msgpack.Put_Str (S, "ee_pose"); Msgpack.Put_Array (S, 7);
      for X of Floats'([0.3, 0.1, 0.2, 1.0, 0.0, 0.0, 0.0]) loop
         Msgpack.Put_Float (S, X);
      end loop;
      Msgpack.Put_Str (S, "action"); Msgpack.Put_Map (S, 1);
      Msgpack.Put_Str (S, "arm"); Msgpack.Put_Array (S, 6);
      for I in 1 .. 6 loop
         Msgpack.Put_Float (S, 30.0);
      end loop;
      Check (Msgpack.Decode (S, D), "布局 · 没夹爪、读数是度的那一帧解得开");
      Layout.Recognise (D, Msgpack.Key (D, 0, "obs"), Lay);
      declare
         Cg : constant Ints := Layout.Command_Groups (Lay);
         Old_Missing : constant String :=
           (if Lay.EE.Is_Empty and then Lay.Joints.Is_Empty then "没认出末端位姿也没认出关节角"
            elsif Lay.Jaw.Is_Empty then "没认出夹爪开度"
            elsif not Lay.Ambiguous.Is_Empty then "有形状分不开的读数,拒绝硬认" else "");
      begin
         Check (Paths_Img (Lay.Groups) = "[state.arm,state.ee_pose,action.arm]" and then Natural (Lay.Twin.Length) = 3
                and then Lay.Twin (0) = 2 and then Lay.Twin (1) = -1 and then Lay.Twin (2) = 0
                and then Same (Cg, Vec ([0, 2])) and then Layout.Missing (Lay) = "" and then Old_Missing /= "",
                "布局 · 每一组数都是一组:" & Paths_Img (Lay.Groups) & "(相机旁边的 shape 不算)· 同名的另一组 " & Show (Lay.Twin)
                & " · 命令组 " & Show (Cg) & " · 缺什么:" & (if Layout.Missing (Lay) = "" then "不缺,开得了机" else Layout.Missing (Lay))
                & " · 牙:原来的写法 ⇒ " & (if Old_Missing = "" then "(也开得了,牙没咬上)" else Old_Missing & ",拒绝开机"));
      end;
      pragma Unreferenced (Gs, Cs);
   end;

   --  ══ I1 认组(Jointboot.Find_Arms):假身体真跑开机第一步 ══
   --  ① x5 的样子:两条臂(各 6 个数,弧度,有回声)、两只夹爪(各 1 个数,开机贴着上头 1.0)、两组身体报的位姿(只是读数,跟着各自那条臂);
   --     头顶眼看得见两条臂和两只夹爪各一块,两只腕眼各长在一条臂上、各看得见自己那只夹爪。
   --  ⇒ 两条臂、眼 = 第 1 / 2 台、各一个合拢通道、不动的眼 = 第 0 台;量出来的布局(Joints / Jaw)和原来按形状认的一字不差;
   --     位姿那两组是读数(推臂时跟着变),从来没被推过;夹爪往正推不动(贴着头)⇒ 下一次往负推,认得出。
   --  病:x5 换了认法以后通道、眼对不上(V1 那条线会断)。牙:夹爪不换方向(只往正推)⇒ 推不动 ⇒ 认不成合拢通道,第 1 / 2 条臂各 0 个
   declare
      Gs : FG_Vectors.Vector;
      Cs : FC_Vectors.Vector;
      R : Fake_Result;
   begin
      Gs.Append (G_Of ("left_arm_joint_state", 6, 0.0));
      Gs.Append (G_Of ("right_arm_joint_state", 6, 0.0));
      Gs.Append (G_Of ("left_ee_joint_state", 1, 1.0, Lo => 0.0, Hi => 1.0));
      Gs.Append (G_Of ("right_ee_joint_state", 1, 1.0, Lo => 0.0, Hi => 1.0));
      Gs.Append (Pose_Of_Group ("left_ee_pose", 0));
      Gs.Append (Pose_Of_Group ("right_ee_pose", 1));
      Cs.Append (Cam_Of (Empty, Vec ([0, 1, 2, 3])));
      Cs.Append (Cam_Of (Vec ([0]), Vec ([2])));
      Cs.Append (Cam_Of (Vec ([1]), Vec ([3])));
      R := Run_Fake (Gs, Cs);
      declare
         Eyes_Ok : constant Boolean := Natural (R.Arms.Length) = 2 and then R.Arms (0).Eye = 1 and then R.Arms (1).Eye = 2;
         Lay_Ok : constant Boolean := Paths_Img (R.Lay.Joints) = Paths_Img (R.Lay0.Joints) and then Paths_Img (R.Lay.Jaw) = Paths_Img (R.Lay0.Jaw)
                                      and then Same (R.Lay.Closing_N, Vec ([1, 1])) and then R.Lay.N_Arms = 2;
         Roles_Ok : constant Boolean := Role_Of (R, "state.left_arm_joint_state") = Selfmap.Arm and then Role_Of (R, "state.right_arm_joint_state") = Selfmap.Arm
                                        and then Role_Of (R, "state.left_ee_joint_state") = Selfmap.Closing and then Role_Of (R, "state.right_ee_joint_state") = Selfmap.Closing
                                        and then Role_Of (R, "state.left_ee_pose") = Selfmap.Reading and then Role_Of (R, "action.left_arm_joint_state") = Selfmap.Reading;
         Pose_Follows : constant Boolean := Grp (R, "state.left_ee_pose") >= 0
                                            and then R.Map.Groups (Natural (Grp (R, "state.left_ee_pose"))).Follows = Grp (R, "state.left_arm_joint_state");
         Not_Pushed : constant Boolean := not R.Pushed.Contains ("left_ee_pose") and then not R.Pushed.Contains ("right_ee_pose");
         Closing_Arm : constant Integer := (if Grp (R, "state.right_ee_joint_state") >= 0 then R.Map.Groups (Natural (Grp (R, "state.right_ee_joint_state"))).Arm else -9);
         Fr : Plug.Frame;
         Lk : Plug.Link;
      begin
         Lk.Lay := R.Lay; Lk.Last := R.Last; Lk.Last_Obs := R.Last_Obs;
         Plug.Frame_Of (Lk, Fr);
         Check (R.Ok and then Eyes_Ok and then Lay_Ok and then Roles_Ok and then Pose_Follows and then Not_Pushed and then R.World = 0 and then Closing_Arm = 1
                and then Selfmap.Jaw_Count (Fr, 0) = 1 and then Selfmap.Jaw_Count (Fr, 1) = 1 and then Plug.Arms (Lk) = 2,
                "认组 · x5 的样子(" & Codec.Img (R.Beats) & " 拍):" & Codec.Img (Natural (R.Arms.Length)) & " 条臂、眼 "
                & (if Natural (R.Arms.Length) = 2 then Codec.Img (Natural (R.Arms (0).Eye)) & "/" & Codec.Img (Natural (R.Arms (1).Eye)) else "?")
                & "、不动的眼 " & Integer'Image (R.World) & " · 合拢通道 " & Show (R.Lay.Closing_N) & "(第 2 只夹爪挂在第" & Integer'Image (Closing_Arm + 1) & " 条臂)"
                & " · 布局和原来按形状认的一样 " & Boolean'Image (Lay_Ok) & "(Joints " & Paths_Img (R.Lay.Joints) & " · Jaw " & Paths_Img (R.Lay.Jaw) & ")"
                & " · 各组是什么 " & Boolean'Image (Roles_Ok) & " · 位姿跟着臂、没被推过 " & Boolean'Image (Pose_Follows and then Not_Pushed)
                & " · 抓握读数每条臂 " & Codec.Img (Selfmap.Jaw_Count (Fr, 0)) & "/" & Codec.Img (Selfmap.Jaw_Count (Fr, 1)) & " 个");
      end;
      --  量出来的布局发命令(Plug.Act → Act_Raw):第 1 条臂给关节目标 + 抓握 0.3 ⇒ 一条动作里四个命令键(两条臂、两只夹爪)都在,
      --  左臂发目标、右臂照读数保持、左夹爪发 0.3、右夹爪照读数;不发身体报的位姿
      declare
         Lk : Plug.Link;
         C : Plug.Cmd;
         Dh : Msgpack.Doc;
         Sent_Ok : Boolean;
         Q : Floats;
         function Nums (Nm : String) return Floats is
           (if Msgpack.Key (Dh, 0, Nm) >= 0 then Msgpack.Numbers (Dh, Msgpack.Key (Dh, 0, Nm)) else F64_Vectors.Empty_Vector);
      begin
         Lk.Lay := R.Lay; Lk.Last := R.Last; Lk.Last_Obs := R.Last_Obs;
         for I in 1 .. 6 loop
            Q.Append (0.2);
         end loop;
         C.Kind := Plug.Joint; C.Arm := 0; C.Group := 0; C.Q := Q; C.Jaw.Append (0.3);
         Sent_Ok := Plug.Act (Lk, C) and then Msgpack.Decode (Lk.Pending, Dh);
         Check (Sent_Ok and then Natural (Nums ("left_arm_joint_state").Length) = 6 and then Nums ("left_arm_joint_state") (0) = 0.2
                and then Natural (Nums ("right_arm_joint_state").Length) = 6 and then Nums ("right_arm_joint_state") (0) = 0.0
                and then Natural (Nums ("left_ee_joint_state").Length) = 1 and then Nums ("left_ee_joint_state") (0) = 0.3
                and then Natural (Nums ("right_ee_joint_state").Length) = 1 and then Nums ("right_ee_joint_state") (0) = 1.0
                and then Msgpack.Key (Dh, 0, "left_ee_pose") < 0,
                "认组 · 量出来的布局发一条关节命令:四个命令键都在(左臂发目标、右臂照读数、左夹爪 0.3、右夹爪照读数 1.0),不发身体报的位姿");
      end;
   end;
   --  ② 没有夹爪(无人机测试台不报抓握):一组 6 个数扛着身上那只眼,头顶眼看得见它 ⇒ 一条臂、0 个合拢通道,开得了机(Ok);
   --     抓握读数那一格是空的(不编)。病:没有夹爪不开机(DR1 / DR2 只好装假夹爪)。牙:原来的 Missing ⇒ "没认出夹爪开度"
   declare
      Gs : FG_Vectors.Vector;
      Cs : FC_Vectors.Vector;
      R : Fake_Result;
      Fr : Plug.Frame;
      Lk : Plug.Link;
   begin
      Gs.Append (G_Of ("arm_joint_state", 6, 0.0));
      Gs.Append (Pose_Of_Group ("ee_pose", 0));
      Cs.Append (Cam_Of (Empty, Vec ([0])));
      Cs.Append (Cam_Of (Vec ([0]), Empty));
      R := Run_Fake (Gs, Cs);
      Lk.Lay := R.Lay; Lk.Last := R.Last; Lk.Last_Obs := R.Last_Obs;
      Plug.Frame_Of (Lk, Fr);
      Check (R.Ok and then Natural (R.Arms.Length) = 1 and then R.Arms (0).Eye = 1 and then Same (R.Lay.Closing_N, Vec ([0]))
             and then Selfmap.Jaw_Count (Fr, 0) = 0 and then R.Lay0.Jaw.Is_Empty,
             "认组 · 没有夹爪:" & Codec.Img (Natural (R.Arms.Length)) & " 条臂、合拢通道 " & Show (R.Lay.Closing_N) & "、抓握读数 "
             & Codec.Img (Selfmap.Jaw_Count (Fr, 0)) & " 个 ⇒ 开得了机 " & Boolean'Image (R.Ok)
             & " · 牙:原来的写法 ⇒ " & (if R.Lay0.Jaw.Is_Empty then "没认出夹爪开度,拒绝开机" else "(认出了夹爪,牙没咬上)"));
   end;
   --  ③ 读数是度:一条臂 6 个数从 30°起,一度挪 7 像素(一弧度 400 像素)⇒ 起点那一推(协议起点)看不见,翻几倍就看得见,认得出臂。
   --  病:读数当弧度认("±2π 以内才是关节"),报度的身体开不了机。牙:原来按形状认 ⇒ 这组不算关节,也没有位姿 ⇒ 拒绝开机
   declare
      Gs : FG_Vectors.Vector;
      Cs : FC_Vectors.Vector;
      R : Fake_Result;
   begin
      Gs.Append (G_Of ("arm_joint_state", 6, 30.0, Gain => 7.0));
      Cs.Append (Cam_Of (Empty, Vec ([0])));
      Cs.Append (Cam_Of (Vec ([0]), Empty));
      R := Run_Fake (Gs, Cs);
      Check (R.Ok and then Natural (R.Arms.Length) = 1 and then R.Arms (0).Eye = 1 and then R.Arms (0).Probe > 1.0e-4 and then R.Lay0.Joints.Is_Empty,
             "认组 · 读数是度:认出 " & Codec.Img (Natural (R.Arms.Length)) & " 条臂(每个数推 " & (if R.Arms.Is_Empty then "?" else Codec.Fmt (R.Arms (0).Probe, 4))
             & " 度才看得见)· 牙:原来按形状认 ⇒ " & (if R.Lay0.Joints.Is_Empty then "这组不算关节,拒绝开机" else "(认成了关节,牙没咬上)"));
   end;
   --  ④ 传感器:一组 6 个数只是读数(没有回声:不是命令),推臂时跟着变;另一组 3 个数一直不变 ⇒ 都认成读数,
   --     一次都没被推过(推一个对方不认的键,RoboDojo 直接报错)。病:按形状认,6 个小数就当关节组推。
   --  牙:原来按形状认 ⇒ imu 那一组进了关节组(原来的 Find_Arms 每个关节组都推)
   declare
      Gs : FG_Vectors.Vector;
      Cs : FC_Vectors.Vector;
      R : Fake_Result;
      Old_Has_Imu : Boolean := False;
   begin
      Gs.Append (G_Of ("left_arm_joint_state", 6, 0.0));
      Gs.Append (G_Of ("imu", 6, 0.0, Command => False));
      Gs.Append (G_Of ("temperature", 3, 20.0, Command => False));
      Gs.Append (G_Of ("left_ee_joint_state", 1, 0.5, Lo => 0.0, Hi => 1.0));
      declare
         Imu : Fake_Group := Gs (1);
      begin
         Imu.Follows := 0;
         Gs.Replace_Element (1, Imu);
      end;
      Cs.Append (Cam_Of (Empty, Vec ([0, 3])));
      Cs.Append (Cam_Of (Vec ([0]), Vec ([3])));
      R := Run_Fake (Gs, Cs);
      for P of R.Lay0.Joints loop
         Old_Has_Imu := Old_Has_Imu or else Layout.Last_Seg (P) = "imu";
      end loop;
      Check (R.Ok and then Role_Of (R, "state.imu") = Selfmap.Reading and then Role_Of (R, "state.temperature") = Selfmap.Reading
             and then R.Map.Groups (Natural (Grp (R, "state.imu"))).Follows = Grp (R, "state.left_arm_joint_state")
             and then R.Map.Groups (Natural (Grp (R, "state.temperature"))).Follows = -1
             and then not R.Pushed.Contains ("imu") and then not R.Pushed.Contains ("temperature") and then Old_Has_Imu,
             "认组 · 传感器:imu " & Role_Img (Role_Of (R, "state.imu")) & "(推臂时跟着变)、temperature " & Role_Img (Role_Of (R, "state.temperature"))
             & "(推哪一组都不变)· 推过它们没有 " & Boolean'Image (R.Pushed.Contains ("imu") or else R.Pushed.Contains ("temperature"))
             & " · 牙:原来按形状认 ⇒ " & (if Old_Has_Imu then "imu 进了关节组、要被推" else "(没进,牙没咬上)"));
   end;
   --  ⑤ 扛着全身的那组:base 一动,两只眼都整幅在动;一条臂扛着腕眼 ⇒ base = 扛着全身(身体图 Carrying_Groups 答它),臂照认;
   --     不动的眼:没有(两只眼都被 base 带着走)。病:扛着全身的那组被当成一条没眼的臂、头顶眼被当成不动的眼(世界跟着身体走)。
   --  牙:原来挑不动的眼只躲开臂上的眼 ⇒ 挑头顶眼(第 0 台)
   declare
      Gs : FG_Vectors.Vector;
      Cs : FC_Vectors.Vector;
      R : Fake_Result;
      Old_World : Integer := -1;
   begin
      Gs.Append (G_Of ("base", 3, 0.0));
      Gs.Append (G_Of ("arm", 6, 0.0));
      Cs.Append (Cam_Of (Vec ([0]), Vec ([1])));
      Cs.Append (Cam_Of (Vec ([0, 1]), Empty));
      R := Run_Fake (Gs, Cs);
      for C in 0 .. 1 loop
         declare
            On_Arm : Boolean := False;
         begin
            for A of R.Arms loop
               On_Arm := On_Arm or else A.Eye = C;
            end loop;
            if not On_Arm and then Old_World < 0 then
               Old_World := C;
            end if;
         end;
      end loop;
      Check (R.Ok and then Role_Of (R, "state.base") = Selfmap.Carrying and then Same (Selfmap.Graph.Carrying_Groups (R.Map), Vec ([Grp (R, "state.base")]))
             and then Natural (R.Arms.Length) = 1 and then R.Arms (0).Eye = 1 and then R.World = -1 and then Old_World = 0,
             "认组 · 扛着全身:base " & Role_Img (Role_Of (R, "state.base")) & "、身体图里扛着全身的组 " & Show (Selfmap.Graph.Carrying_Groups (R.Map))
             & "、臂 " & Codec.Img (Natural (R.Arms.Length)) & " 条、不动的眼 " & Integer'Image (R.World) & "(该没有)"
             & " · 牙:原来只躲开臂上的眼 ⇒ 不动的眼 = " & Integer'Image (Old_World));
   end;
   --  ⑥ 三条臂,只有第 1 条有夹爪(还都报位姿):三条臂、眼 1 / 2 / 3、合拢通道 1 / 0 / 0;插头说 3 条臂。
   --  病:按"位姿键数和抓握键数取小"数臂 ⇒ 少一条臂。牙:原来的 Plug.Arms(位姿 3 组、抓握 2 组(连回声))⇒ 2 条
   declare
      Gs : FG_Vectors.Vector;
      Cs : FC_Vectors.Vector;
      R : Fake_Result;
      Lk, Lo : Plug.Link;
   begin
      Gs.Append (G_Of ("a1", 6, 0.0));
      Gs.Append (G_Of ("a2", 6, 0.0));
      Gs.Append (G_Of ("a3", 6, 0.0));
      Gs.Append (G_Of ("g1", 1, 0.5, Lo => 0.0, Hi => 1.0));
      Gs.Append (Pose_Of_Group ("p1", 0));
      Gs.Append (Pose_Of_Group ("p2", 1));
      Gs.Append (Pose_Of_Group ("p3", 2));
      Cs.Append (Cam_Of (Empty, Vec ([0, 1, 2])));
      Cs.Append (Cam_Of (Vec ([0]), Vec ([3])));
      Cs.Append (Cam_Of (Vec ([1]), Empty));
      Cs.Append (Cam_Of (Vec ([2]), Empty));
      R := Run_Fake (Gs, Cs);
      Lk.Lay := R.Lay;
      Lo.Lay := R.Lay0;
      Check (R.Ok and then Natural (R.Arms.Length) = 3 and then R.Arms (0).Eye = 1 and then R.Arms (1).Eye = 2 and then R.Arms (2).Eye = 3
             and then Same (R.Lay.Closing_N, Vec ([1, 0, 0])) and then Plug.Arms (Lk) = 3 and then R.World = 0 and then Plug.Arms (Lo) = 2,
             "认组 · 三条臂、只第 1 条有夹爪:" & Codec.Img (Natural (R.Arms.Length)) & " 条臂、合拢通道 " & Show (R.Lay.Closing_N)
             & "、插头说 " & Codec.Img (Plug.Arms (Lk)) & " 条 · 牙:原来按位姿键数和抓握键数取小 ⇒ " & Codec.Img (Plug.Arms (Lo)) & " 条");
   end;
   --  ⑦ 接入契约三条:一组推得动、哪只眼里都看不见(哑巴)⇒ 第 2 条;一组推了读数不跟 ⇒ 第 1 条;
   --     一组往正推画面里那一块动了,往负推读数说走到了、画面却一个像素都没变 ⇒ 第 3 条(没动却不说)。
   --     哑巴和推不动的两组进 Holds(照读数保持,命令里照样带齐),不当合拢通道。
   --  病:哑巴零件 / 推不动的通道被当成夹爪去合空(DR1 / DR2 那只假夹爪),说谎的通道没人发现。
   --  牙:原来按形状认 ⇒ 这三组(1 个 [0,1] 的数)都进了抓握组
   declare
      Gs : FG_Vectors.Vector;
      Cs : FC_Vectors.Vector;
      R : Fake_Result;
      Old_Jaws : Natural := 0;
   begin
      Gs.Append (G_Of ("arm", 6, 0.0));
      Gs.Append (G_Of ("mute", 1, 0.5, Lo => 0.0, Hi => 1.0));
      Gs.Append (G_Of ("frozen", 1, 0.5, Lo => 0.0, Hi => 1.0));
      Gs.Append (G_Of ("liar", 1, 0.5, Lo => 0.0, Hi => 1.0));
      declare
         Fz : Fake_Group := Gs (2);
         Li : Fake_Group := Gs (3);
      begin
         Fz.Frozen := True; Li.Lie_Neg := True;
         Gs.Replace_Element (2, Fz); Gs.Replace_Element (3, Li);
      end;
      Cs.Append (Cam_Of (Empty, Vec ([0, 3])));
      Cs.Append (Cam_Of (Vec ([0]), Vec ([3])));
      R := Run_Fake (Gs, Cs);
      for P of R.Lay0.Jaw loop
         if Layout.Joined (P) in "state.mute" | "state.frozen" | "state.liar" then
            Old_Jaws := Old_Jaws + 1;
         end if;
      end loop;
      Check (R.Ok and then Role_Of (R, "state.mute") = Selfmap.Mute and then Role_Of (R, "state.frozen") = Selfmap.Not_Following
             and then Role_Of (R, "state.liar") = Selfmap.Closing and then R.Map.Groups (Natural (Grp (R, "state.liar"))).Lied
             and then not R.Map.Groups (Natural (Grp (R, "state.arm"))).Lied
             and then Paths_Img (R.Lay.Holds) = "[state.mute,state.frozen]" and then Old_Jaws = 3,
             "认组 · 接入契约:mute " & Role_Img (Role_Of (R, "state.mute")) & "(第 2 条)· frozen " & Role_Img (Role_Of (R, "state.frozen"))
             & "(第 1 条)· liar " & Role_Img (Role_Of (R, "state.liar")) & "、说谎 "
             & Boolean'Image (Grp (R, "state.liar") >= 0 and then R.Map.Groups (Natural (Grp (R, "state.liar"))).Lied) & "(第 3 条)"
             & " · 照读数保持的 " & Paths_Img (R.Lay.Holds) & " · 牙:原来按形状认 ⇒ 三组里 " & Codec.Img (Old_Jaws) & " 组进了抓握组");
   end;
   --  ⑧ 一条臂两组合拢通道(人形每根手指一组):两根手指都在腕眼里 ⇒ 这条臂 2 组合拢通道;命令里这条臂的抓握目标 [0.1, 0.9]
   --     ⇒ 第一根手指发 0.1、第二根发 0.9(按每组几个数切)。病:一条臂只认一个抓握组(下标 = 臂),第二根手指永远照读数、第一根收到两个数。
   --  牙:原来的写法(抓握组的下标就是臂)⇒ 第一根收到 [0.1, 0.9] 两个数、第二根不是这条臂的
   declare
      Gs : FG_Vectors.Vector;
      Cs : FC_Vectors.Vector;
      R : Fake_Result;
      Lk : Plug.Link;
      C : Plug.Cmd;
      Dh : Msgpack.Doc;
      Sent_Ok : Boolean;
      function Nums (Nm : String) return Floats is
        (if Msgpack.Key (Dh, 0, Nm) >= 0 then Msgpack.Numbers (Dh, Msgpack.Key (Dh, 0, Nm)) else F64_Vectors.Empty_Vector);
   begin
      Gs.Append (G_Of ("arm", 6, 0.0));
      Gs.Append (G_Of ("f1", 1, 0.5, Lo => 0.0, Hi => 1.0));
      Gs.Append (G_Of ("f2", 1, 0.5, Lo => 0.0, Hi => 1.0));
      Cs.Append (Cam_Of (Empty, Vec ([0, 1, 2])));
      Cs.Append (Cam_Of (Vec ([0]), Vec ([1, 2])));
      R := Run_Fake (Gs, Cs);
      Lk.Lay := R.Lay; Lk.Last := R.Last; Lk.Last_Obs := R.Last_Obs;
      C.Kind := Plug.Joint; C.Arm := 0; C.Group := 0; C.Q := Msgpack.Numbers (R.Last, Layout.Find (R.Last, R.Last_Obs, R.Lay.Joints (0)));
      C.Jaw.Append (0.1); C.Jaw.Append (0.9);
      Sent_Ok := Plug.Act (Lk, C) and then Msgpack.Decode (Lk.Pending, Dh);
      Check (R.Ok and then Same (R.Lay.Closing_N, Vec ([2])) and then Sent_Ok
             and then Natural (Nums ("f1").Length) = 1 and then Nums ("f1") (0) = 0.1 and then Natural (Nums ("f2").Length) = 1 and then Nums ("f2") (0) = 0.9,
             "认组 · 一条臂两组合拢通道:" & Show (R.Lay.Closing_N) & " · 抓握目标 [0.1, 0.9] ⇒ f1 "
             & (if Nums ("f1").Is_Empty then "没发" else Codec.Fmt (Nums ("f1") (0), 2)) & "、f2 " & (if Nums ("f2").Is_Empty then "没发" else Codec.Fmt (Nums ("f2") (0), 2))
             & " · 牙:原来抓握组的下标就是臂 ⇒ 第二根手指(下标 1)不是第 1 条臂的、第一根收到两个数");
   end;
   --  ══ 开机扫描时身前不一定是空的(P8A 10-01:右臂扫第 2 个关节时撞上柜子把手,第 2 个关节一格就冲到 −1.237、第 1 个关节被顶偏 0.3715,
   --  卡在那儿;之后每一段都从卡住的地方出发,22 格里 8 格是卡着的,运动学没量成)══
   --  锁步里一只假手真跑 Jointboot.Sweep_All(配点仪器不在:每格存不成图,只看格子和读数),主线程当身体 —— 6 个关节,第 2 个关节往负过了
   --  −0.06(把手在那儿)就挂住:那一格第 2 个关节冲到 −1.2369、第 1 个关节被顶偏 +0.3715,之后整组回起点、只回第 2 个关节都挪不动。
   --  ① 挂得住也松得开(别的关节停在原处、先把第 1 个关节挪回去就松开):碰上的那一格不进格子;一根一根往回挪退回了起点,
   --     后面的关节(第 3–5 个)、几个关节一起动的格照扫,没有一格是挂着的;第 2 个关节往负留下两格干净的(−0.03、−0.06)。
   --  ② 挂住了就松不开:碰上的那一格不进格子,退不回 ⇒ 这只手不再扫(后面的关节一格都没有),没有一格是挂着的。
   --  病:碰上东西的那一格和之后卡着的格子进了运动学(P8A 第 2 只手"没量成");一直往挡着的东西里压。
   --  牙:原来的写法 —— 碰上了照样先把这一格留下(Keep 在判停之前)、下一段直接从卡住的地方出发 ⇒ ① 里留下挂着的格子
   declare
      type Hook_Run is record
         Frames : Kinem.Frame_Vectors.Vector;
         Hooked_Kept, Late_Joints, Neg2, Multi : Natural := 0;
         Beats : Natural := 0;
      end record;
      function Run_Sweep (Releasable : Boolean) return Hook_Run is
         R : Hook_Run;
         Lk : Plug.Link;
         Mp : Selfmap.Body_Map;
         Fr0 : Plug.Frame;
         Arms : Jointboot.Arm_Vectors.Vector;
         Ds : Jointboot.Sweep_Vectors.Vector;
         Css : Jointboot.Corr_Set_Vectors.Vector;
         Q : Floats := F64_Vectors.To_Vector (0.0, 6);
         Hooked : Boolean := False;
         Cap : constant := 3000;   --  拍数上限(次数,防万一)
         Img : Plug.Cam;
         task type Sweep_Hand;
         task body Sweep_Hand is
            Fr : Plug.Frame := Fr0;
         begin
            Lockstep.Begin_Hand (0);
            Jointboot.Sweep_All (Lk, Fr, Mp, Arms, "127.0.0.1", 1, "", Ds, Css);
            Lockstep.Done;
         exception
            when E : others =>
               Put_Line ("  假手出错:" & Ada.Exceptions.Exception_Information (E));
               Lockstep.Done;
         end Sweep_Hand;
         procedure Apply (T : Floats) is
         begin
            if Hooked then
               --  挂住了:只有别的关节停在原处(第 2 个关节就停在此刻)、先把第 1 个关节挪回起点,才松得开
               if Releasable and then T (1) <= 0.0 and then abs (T (2) - Q (2)) < 0.01 then
                  Hooked := False;
                  Q.Replace_Element (1, T (1));
               end if;
               return;
            end if;
            Q := T;
            if T (2) < -0.06 then
               Hooked := True;
               Q.Replace_Element (2, -1.2369); Q.Replace_Element (1, T (1) + 0.3715);
            end if;
         end Apply;
         function Frame_Now return Plug.Frame is
            Fr : Plug.Frame;
         begin
            Fr.Joints.Append (Q);
            Fr.Cams.Append (Img); Fr.Cams.Append (Img);
            Fr.Seq := Lk.Seq;
            return Fr;
         end Frame_Now;
      begin
         Img.W := Fw; Img.H := Fh;
         for I in 1 .. Fw * Fh loop
            Img.Gray.Append (U8 (100 + I mod 50));
            for K in 1 .. 3 loop
               Img.RGB.Append (U8 (100 + I mod 50));
            end loop;
         end loop;
         declare
            Ai : Jointboot.Arm_Info;
         begin
            Ai.Group := 0; Ai.Eye := 1; Ai.Probe := 1.0e-4; Ai.Frac.Append (0.0); Ai.Frac.Append (0.5);
            Arms.Append (Ai);
         end;
         Mp.N_Cams := 2; Mp.Settle := 2;
         for C in 0 .. 1 loop
            Mp.Floors.Append (Picture.Null_Floor (Img.Gray, Img.Gray, Fw, Fh, Picture.Min_Pixels (Fw, Fh)));
            Mp.Pic_Floor.Append (0);
         end loop;
         Fr0 := Frame_Now;
         Lockstep.Clear;
         Plug.Lock_Begin;
         declare
            Hd : Sweep_Hand;
         begin
            Lockstep.Start (0, Hd'Identity);
            loop
               Lockstep.Run (0);
               exit when Lockstep.Finished (0);
               R.Beats := R.Beats + 1;
               declare
                  Mg : constant Plug.Cmd := Plug.Lock_Merged;
               begin
                  for K in 0 .. Natural'Min (Natural (Mg.Groups.Length), Natural (Mg.Qs.Length)) - 1 loop
                     if Mg.Groups (K) = 0 and then Natural (Mg.Qs (K).Length) = 6 then
                        Apply (Mg.Qs (K));
                     end if;
                  end loop;
               end;
               Plug.Lock_Begin;
               Lk.Seq := Lk.Seq + 1;
               Plug.Lock_Feed (Frame_Now, Ok => R.Beats < Cap);
            end loop;
         end;
         Plug.Lock_End;
         Lockstep.Clear;
         if not Ds.Is_Empty then
            R.Frames := Ds (0).Frames;
         end if;
         for Fi of R.Frames loop
            if abs Fi.Q (2) > 0.5 or else abs Fi.Q (1) > 0.2 then
               R.Hooked_Kept := R.Hooked_Kept + 1;
            end if;
            if Fi.Joint >= 3 then
               R.Late_Joints := R.Late_Joints + 1;
            end if;
            if Fi.Joint = 2 and then Fi.Q (2) < 0.0 then
               R.Neg2 := R.Neg2 + 1;
            end if;
            if Fi.Joint < 0 then
               R.Multi := R.Multi + 1;
            end if;
         end loop;
         return R;
      end Run_Sweep;
      Ra : constant Hook_Run := Run_Sweep (Releasable => True);
      Rb : constant Hook_Run := Run_Sweep (Releasable => False);
   begin
      Check (Ra.Hooked_Kept = 0 and then Ra.Late_Joints > 0 and then Ra.Neg2 = 2 and then Ra.Multi > 1
             and then Rb.Hooked_Kept = 0 and then Rb.Late_Joints = 0 and then not Rb.Frames.Is_Empty,
             "扫描碰上东西(P8A 那样挂在把手上):① 松得开 ⇒ " & Codec.Img (Natural (Ra.Frames.Length)) & " 格(" & Codec.Img (Ra.Beats) & " 拍)里挂着的 "
             & Codec.Img (Ra.Hooked_Kept) & " 格(要 0)、第 2 个关节往负干净的 " & Codec.Img (Ra.Neg2) & " 格(要 2)、退回以后第 3–5 个关节照扫 "
             & Codec.Img (Ra.Late_Joints) & " 格、几个关节一起动的 " & Codec.Img (Ra.Multi) & " 格 · ② 松不开 ⇒ " & Codec.Img (Natural (Rb.Frames.Length))
             & " 格里挂着的 " & Codec.Img (Rb.Hooked_Kept) & " 格(要 0)、卡住以后不再扫(第 3–5 个关节 " & Codec.Img (Rb.Late_Joints) & " 格)"
             & " · 牙:原来碰上了照样留下那一格、下一段从卡住的地方出发 ⇒ 挂着的格子进运动学(P8A 22 格里 8 格)");
   end;
end Welds_Path_1;
