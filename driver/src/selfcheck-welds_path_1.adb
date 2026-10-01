with Selfmap.Graph;
with Readings;
with Links;
with Stats;
with Geom;
with Bodyfile;
with Schema;
with Ada.Exceptions;
with Sinew;
with Table;
with Plan;
with Episode;
with World;
with Memory;
with Runtime;
with Probe;
separate (Selfcheck)
procedure Welds_Path_1 is
   use Ada.Numerics.Long_Elementary_Functions;
   use type Selfmap.Group_Role;
   use type Selfmap.Walk_End;
   use type Runtime.Yield;
   use type Probe.Next_Step;
   use type Readings.Eye_Verdict;
   use type Readings.View_Says;
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
      White : Boolean := False;       --  看的是一面白墙(世界没纹理,只有噪声;那几块照样有纹理)
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
   function Cam_Of (Ego, Parts : Ints; White : Boolean := False) return Fake_Cam is (Ego => Ego, Parts => Parts, White => White);

   function Run_Fake (Gs : FG_Vectors.Vector; Cs : FC_Vectors.Vector; Prior_Arms : Strs := Str_Vectors.Empty_Vector;
                      Prior_Eyes : Ints := Int_Vectors.Empty_Vector) return Fake_Result is
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
                        Val : Long_Float := (if Cs (C).White then 230.0 else World_Px (Long_Float (X) - Ox, Long_Float (Y) - 0.5 * Ox));
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
      Lk.Lay.Prior_Arms := Prior_Arms; Lk.Lay.Prior_Eyes := Prior_Eyes;   --  上一回存的(白墙那一条焊点用)
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
   --  ══ 每一节的形状(Links,I7):不动的眼看着臂一个关节一个关节转,跟着哪一节动的点就是那一节的表面点 ══
   --  合成的两条 6 关节臂(同运动学那条焊点的几何;第 2 条挪开 0.6),放进世界时转过、缩放过(S = 1.5);一只不动的眼离 1.2 米斜着看。
   --  每一节 4 个表面点(离各自的轴有一段),墙上 70 个不动的背景点;扫描:每个关节往两边各两格(两条臂同一拍扫同一个关节)、
   --  再 3 格几个关节一起动;每个点每一格的像素 = 真的投影 + 0.15 像素以内的噪声(配点噪声从这批配点自己量)。
   --  ① 三角:每条臂每一节的点都收回来、认对臂和节(48 个点,认错 0 个),离真值都在 Stats.Z 倍自报的不确定度以内;背景点一个都不收。
   --  ② 净空:场景里放一个点,离第 1 条臂第 5 节最近那一点 0.05 ⇒ Clear_Of 说最近 0.05、是第 5 节、是场景点;
   --     第 1 条臂朝它平移 ⇒ 走 0.05 减去 Stats.Z 倍不确定度就进带子;背着它走 ⇒ 碰不上;没量过的臂 ⇒ 说不出(Known = False)。
   --  ③ 自己:不动的眼里,两条臂的表面点落的像素是"自己",墙上一个背景点那一格不是。
   --  病:没有每一节的形状 ⇒ 走一步不知道胳膊会不会撞上东西、眼把自己的胳膊框成一件东西(路 7 那一回)。
   --  牙:每一格按整条臂的位姿(所有关节,不截到那一节)搬视线 ⇒ 近端那几节的点在远端关节转的那几格里被搬错、交不上,收不回来
   --  (10-01 改一行跑过:48 个点只收回第 5 节那 8 个,红;造假数据的真值是焊点自己算的,不借 Links)
   declare
      use Geom;
      Wax : constant array (0 .. 5) of V3 := [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]];
      Pax : constant array (0 .. 5) of V3 := [[0.0, 0.0, 0.05], [0.0, 0.0, 0.12], [0.25, 0.0, 0.12], [0.45, 0.0, 0.16], [0.5, 0.0, 0.16], [0.55, 0.0, 0.16]];
      C0 : constant V3 := [0.6, 0.0, 0.22];
      Md : Kinem.Model;
      Pls : Links.Placement_Vectors.Vector;
      G : Cam_Geo;
      Truth : Links.Link_Pt_Vectors.Vector;
      Cells : Links.Cell_Vectors.Vector;
      Tracks : Links.Track_Vectors.Vector;
      Pts : Links.Link_Pt_Vectors.Vector;
      Sd_Px : Long_Float;
      N_Seed : Natural := 0;
      function Noise return Long_Float is
      begin
         N_Seed := N_Seed + 1;
         return 0.15 * Sin (Long_Float (N_Seed) * 12.9898) ;   --  确定的伪随机,±0.15 像素
      end Noise;
      function V3_Add (A, B : V3) return V3 is ([A (0) + B (0), A (1) + B (1), A (2) + B (2)]);
      function V3_Sub (A, B : V3) return V3 is ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]);
      function Cross3 (A, B : V3) return V3 is ([A (1) * B (2) - A (2) * B (1), A (2) * B (0) - A (0) * B (2), A (0) * B (1) - A (1) * B (0)]);
      function Unit3 (A : V3) return V3 is ([A (0) / Norm (A), A (1) / Norm (A), A (2) / Norm (A)]);
      --  第 A 条臂第 Lk 节、读数 Q 时,参照系里的点 P 在世界里在哪
      --  真值自己算(不借被考的那一份 Links.World_Of —— 借了的话,它算错的那一种动法造出来的假数据跟着一起错,牙咬不住):
      --  第 Lk 节只跟着前 Lk + 1 个关节动(更远的关节放回参照读数 0),再按 X_世界 = Rw · (S · Ra · X + Ta − O) 放进世界
      function Where (A, Lk : Natural; Q : Floats; P : V3) return V3 is
         Pl : constant Links.Placement := Pls (A);
         Qt : Floats := Q;
         R : M3;
         T : V3;
      begin
         for J in Lk + 1 .. Natural (Qt.Length) - 1 loop
            Qt.Replace_Element (J, 0.0);
         end loop;
         Kinem.FK (Pl.Model, Qt, R, T);
         declare
            Xm : constant V3 := V3_Add (Ap (R, P), T);
            Xr : constant V3 := Ap (Pl.Ra, Xm);
         begin
            return Ap (Pl.Rw, V3_Sub (V3_Add ([Pl.S * Xr (0), Pl.S * Xr (1), Pl.S * Xr (2)], Pl.Ta), Pl.O));
         end;
      end Where;
      Zero6 : constant Floats := F64_Vectors.To_Vector (0.0, 6);
   begin
      Md.N := 6; Md.F := 400.0; Md.Cx := 320.0; Md.Cy := 240.0; Md.Valid := True; Md.Q0 := Zero6;
      for I in 0 .. 5 loop
         Md.Ax (I).W := Wax (I);
         Md.Ax (I).P := V3_Sub (Pax (I), C0);
      end loop;
      Pls.Append (Links.Placement'(Model => Md, S => 1.5, Ra => Rodrigues ([0.0, 0.0, 0.3]), Ta => [0.1, -0.2, 0.05],
                                   Rw => Rodrigues ([0.2, 0.0, 0.0]), O => [0.05, 0.05, 0.0], Valid => True, Group => 1, Eye => -1));
      Pls.Append (Links.Placement'(Model => Md, S => 1.5, Ra => Rodrigues ([0.0, 0.0, 0.3]), Ta => [0.1, 0.4, 0.05],
                                   Rw => Rodrigues ([0.2, 0.0, 0.0]), O => [0.05, 0.05, 0.0], Valid => True, Group => 0, Eye => -1));
      --  每一节 4 个点:那一节的轴上一点挪开 ±3 cm、±2 cm(离轴有一段,转的时候挪得出来)
      for A in 0 .. 1 loop
         for Lk in 0 .. 5 loop
            for K in 0 .. 3 loop
               Truth.Append (Links.Link_Pt'(Arm => A, Link => Lk,
                                            P => V3_Add (V3_Sub (Pax (Lk), C0), [(if K mod 2 = 0 then 0.03 else -0.03), (if K < 2 then 0.02 else -0.02), 0.04]),
                                            others => <>));
            end loop;
         end loop;
      end loop;
      --  不动的眼:看着两条臂的中间,离 1.2(世界单位)
      declare
         Center : constant V3 := [0.5 * (Where (0, 0, Zero6, [0.0, 0.0, 0.0]) (0) + Where (1, 0, Zero6, [0.0, 0.0, 0.0]) (0)),
                                  0.5 * (Where (0, 0, Zero6, [0.0, 0.0, 0.0]) (1) + Where (1, 0, Zero6, [0.0, 0.0, 0.0]) (1)),
                                  0.5 * (Where (0, 0, Zero6, [0.0, 0.0, 0.0]) (2) + Where (1, 0, Zero6, [0.0, 0.0, 0.0]) (2))];
         Pos : constant V3 := V3_Add (Center, [0.4, -0.5, 1.0]);
         Zc : constant V3 := Unit3 (V3_Sub (Pos, Center));            --  相机 z 朝后(往前看是 −z)
         Xc : constant V3 := Unit3 (Cross3 ([0.0, 0.0, 1.0], Zc));
         Yc : constant V3 := Cross3 (Zc, Xc);
      begin
         G.Valid := True; G.Fixed := True; G.F := 400.0; G.Cx := 320.0; G.Cy := 240.0; G.Pos := Pos;
         for I in 0 .. 2 loop
            G.R_Ce (I, 0) := Xc (I); G.R_Ce (I, 1) := Yc (I); G.R_Ce (I, 2) := Zc (I);
         end loop;
      end;
      --  扫描的格:每个关节两边各两格(两条臂同一拍扫同一个关节),再 3 格几个关节一起动
      for J in 0 .. 5 loop
         for V of Floats'([0.1, 0.2, -0.1, -0.2]) loop
            declare
               C : Links.Cell;
               Q : Floats := Zero6;
            begin
               Q.Replace_Element (J, V);
               C.Qs.Append (Q); C.Qs.Append (Q);
               C.Joints.Append (J); C.Joints.Append (J);
               Cells.Append (C);
            end;
         end loop;
      end loop;
      for K in 1 .. 3 loop
         declare
            C : Links.Cell;
            Q : Floats := Zero6;
         begin
            for J in 0 .. 5 loop
               Q.Replace_Element (J, 0.05 * Long_Float (K) * (if (J + K) mod 2 = 0 then 1.0 else -1.0));
            end loop;
            C.Qs.Append (Q); C.Qs.Append (Q);
            C.Joints.Append (-1); C.Joints.Append (-1);
            Cells.Append (C);
         end;
      end loop;
      --  每个表面点、每个背景点的一串像素
      for T of Truth loop
         declare
            Tr : Links.Track;
            U, V : Long_Float;
            Front : Boolean;
         begin
            Project_Fixed (G, Where (T.Arm, T.Link, Zero6, T.P), U, V, Front);
            Tr.U0 := U; Tr.V0 := V;
            for C of Cells loop
               Project_Fixed (G, Where (T.Arm, T.Link, C.Qs (T.Arm), T.P), U, V, Front);
               Tr.U.Append (U + Noise); Tr.V.Append (V + Noise);
            end loop;
            Tracks.Append (Tr);
         end;
      end loop;
      for K in 0 .. 69 loop
         declare
            Tr : Links.Track;
            Wall : constant V3 := V3_Add (V3_Sub (G.Pos, [3.0 * G.R_Ce (0, 2), 3.0 * G.R_Ce (1, 2), 3.0 * G.R_Ce (2, 2)]),
                                          [0.08 * Long_Float (K mod 10 - 5) * G.R_Ce (0, 0) + 0.08 * Long_Float (K / 10 - 3) * G.R_Ce (0, 1),
                                           0.08 * Long_Float (K mod 10 - 5) * G.R_Ce (1, 0) + 0.08 * Long_Float (K / 10 - 3) * G.R_Ce (1, 1),
                                           0.08 * Long_Float (K mod 10 - 5) * G.R_Ce (2, 0) + 0.08 * Long_Float (K / 10 - 3) * G.R_Ce (2, 1)]);
            U, V : Long_Float;
            Front : Boolean;
         begin
            Project_Fixed (G, Wall, U, V, Front);
            Tr.U0 := U; Tr.V0 := V;
            for C of Cells loop
               Tr.U.Append (U + Noise); Tr.V.Append (V + Noise);
            end loop;
            Tracks.Append (Tr);
         end;
      end loop;
      Links.Triangulate (Pls, G, 640, Cells, Tracks, Pts, Sd_Px);
      declare
         Wrong, Far : Natural := 0;
         Worst_Z : Long_Float := 0.0;
         Per_Link : array (0 .. 1, 0 .. 5) of Natural := [others => [others => 0]];
         Sc_Ok, Free_Ok, Self_Ok : Boolean := False;
         C1 : Links.Clearance;
         T_Toward, T_Away : Long_Float := 0.0;
         True_Min : Long_Float := Long_Float'Last;
         Readings_Ok : Boolean := False;
         T_Solo : Long_Float := 0.0;
         True_Link : Integer := -1;
         K_Toward, K_Away, K_None : Boolean := False;
      begin
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               P : constant Links.Link_Pt := Pts (I);
               --  这一点是哪一个真值点(参照系里最近的那个,同一条臂)
               Best : Natural := 0;
               Bd : Long_Float := Long_Float'Last;
            begin
               for J in 0 .. Natural (Truth.Length) - 1 loop
                  if Truth (J).Arm = P.Arm and then Norm (V3_Sub (Truth (J).P, P.P)) < Bd then
                     Bd := Norm (V3_Sub (Truth (J).P, P.P)); Best := J;
                  end if;
               end loop;
               if Truth (Best).Link /= P.Link then
                  Wrong := Wrong + 1;
               end if;
               Per_Link (P.Arm, P.Link) := Per_Link (P.Arm, P.Link) + 1;
               declare
                  Sd : constant Long_Float := Sqrt (P.Cov (0, 0) + P.Cov (1, 1) + P.Cov (2, 2));
               begin
                  if Bd > Stats.Z * Sd then
                     Far := Far + 1;
                  end if;
                  Worst_Z := Long_Float'Max (Worst_Z, Bd / Sd);
               end;
            end;
         end loop;
         --  ② 净空
         Links.Install (Pls, Pts);
         declare
            Qs : Plug.Floats_Vectors.Vector;
            Near : constant V3 := Where (0, 5, Zero6, Truth (5 * 4).P);   --  第 1 条臂第 5 节的第 0 个点
            Away_Dir : V3;
            Scene : Scene_Pt_Vectors.Vector;
            Toward : V3;
         begin
            Qs.Append (Zero6); Qs.Append (Zero6);
            --  离它 0.05、在背着另一条臂的那一边
            Toward := Unit3 (V3_Sub (Near, Where (1, 5, Zero6, Truth (24 + 20).P)));
            Scene.Append (Scene_Pt'(Pw => V3_Add (Near, [0.05 * Toward (0), 0.05 * Toward (1), 0.05 * Toward (2)]), others => <>));
            C1 := Links.Clear_Of (0, Qs, Scene);
            --  真的最近:这个场景点离第 1 条臂每一个真表面点(此刻)最近多远、是哪一节
            for T of Truth loop
               if T.Arm = 0 and then Norm (V3_Sub (Where (0, T.Link, Zero6, T.P), Scene (0).Pw)) < True_Min then
                  True_Min := Norm (V3_Sub (Where (0, T.Link, Zero6, T.P), Scene (0).Pw)); True_Link := Integer (T.Link);
               end if;
            end loop;
            Sc_Ok := C1.Valid and then C1.To_Scene and then C1.Link = True_Link and then abs (C1.Dist - True_Min) <= Stats.Z * C1.Sd;
            T_Toward := Links.Free_Along (0, Qs, Toward, Scene, K_Toward);
            Away_Dir := [-Toward (0), -Toward (1), -Toward (2)];
            T_Away := Links.Free_Along (0, Qs, Away_Dir, Scene, K_Away);
            declare
               Tn : Long_Float;
               Solo : Links.Link_Pt_Vectors.Vector;
               K_Solo : Boolean;
            begin
               --  背着场景点走,那一边是第 2 条臂 ⇒ 碰上的是它(比朝场景点走远);第 2 条臂的点撤掉 ⇒ 碰不上
               for P of Pts loop
                  if P.Arm = 0 then
                     Solo.Append (P);
                  end if;
               end loop;
               Links.Install (Pls, Solo);
               T_Solo := Links.Free_Along (0, Qs, Away_Dir, Scene, K_Solo);
               Links.Install (Pls, Links.Link_Pt_Vectors.Empty_Vector);   --  量过的点一个都没有
               Tn := Links.Free_Along (0, Qs, Toward, Scene, K_None);
               Free_Ok := K_Toward and then T_Toward <= 0.05 and then T_Toward > 0.0 and then K_Away and then T_Away > T_Toward
                          and then T_Away < Long_Float'Last and then K_Solo and then T_Solo = Long_Float'Last
                          and then not K_None and then Tn = Long_Float'Last;
            end;
            Links.Install (Pls, Pts);
            --  ③ 自己
            declare
               Mk : constant Bools := Links.Self_Mask (G, [others => 0.0], 640, 480, Qs);
               U, V, Ub, Vb : Long_Float;
               Front : Boolean;
               Far_Px : Long_Float := -1.0;
            begin
               Project_Fixed (G, Where (0, 3, Zero6, Truth (3 * 4).P), U, V, Front);
               --  墙上离所有表面点的投影最远的那个背景点
               Ub := 0.0; Vb := 0.0;
               for K in Natural (Truth.Length) .. Natural (Tracks.Length) - 1 loop
                  declare
                     Mn : Long_Float := Long_Float'Last;
                  begin
                     for J in 0 .. Natural (Truth.Length) - 1 loop
                        Mn := Long_Float'Min (Mn, Sqrt ((Tracks (K).U0 - Tracks (J).U0) ** 2 + (Tracks (K).V0 - Tracks (J).V0) ** 2));
                     end loop;
                     if Mn > Far_Px and then Tracks (K).U0 >= 0.0 and then Tracks (K).U0 < 640.0 and then Tracks (K).V0 >= 0.0 and then Tracks (K).V0 < 480.0 then
                        Far_Px := Mn; Ub := Tracks (K).U0; Vb := Tracks (K).V0;
                     end if;
                  end;
               end loop;
               Self_Ok := Front and then U >= 0.0 and then U < 640.0 and then V >= 0.0 and then V < 480.0
                          and then Mk (Natural (Long_Float'Floor (V)) * 640 + Natural (Long_Float'Floor (U)))
                          and then Far_Px > 0.0 and then not Mk (Natural (Long_Float'Floor (Vb)) * 640 + Natural (Long_Float'Floor (Ub)));
               --  干活时一帧里直接问(路 7 用):读数按装上时记下的组号取 —— 第 1 条臂的读数排在第 1 组、第 2 条臂的在第 0 组时也取对;
               --  同一帧问出来的"自己"和按臂给读数问出来的一样。牙:按下标取(第 A 条臂 = 第 A 组)⇒ 两条臂的读数对调
               declare
                  Fr : Plug.Frame;
                  Qa : Floats := Zero6;
                  Rn : Plug.Floats_Vectors.Vector;
                  Mk2 : Bools;
                  Same_Mask : Boolean := True;
               begin
                  Qa.Replace_Element (1, 0.05);
                  Fr.Joints.Append (Qa); Fr.Joints.Append (Zero6);   --  第 1 条臂装上时记的是第 1 组、第 2 条臂是第 0 组(Qa 在第 0 组 = 第 2 条臂的)
                  Rn := Links.Readings_Now (Fr);
                  Readings_Ok := Natural (Rn.Length) = 2 and then Rn (0) (1) = 0.0 and then Rn (1) (1) = 0.05;
                  Fr.Joints.Clear; Fr.Joints.Append (Zero6); Fr.Joints.Append (Zero6);
                  Mk2 := Links.Self_Mask_Now (Fr, 0, G, 640, 480);
                  for I in 0 .. Natural (Mk.Length) - 1 loop
                     if Mk (I) /= Mk2 (I) then
                        Same_Mask := False;
                     end if;
                  end loop;
                  Self_Ok := Self_Ok and then Same_Mask;
               end;
            end;
         end;
         Check (Natural (Pts.Length) = Natural (Truth.Length) and then Wrong = 0 and then Far = 0
                and then (for all A in 0 .. 1 => (for all Lk in 0 .. 5 => Per_Link (A, Lk) = 4))
                and then Sc_Ok and then Free_Ok and then Self_Ok and then Readings_Ok,
                "每一节的形状 · 三角:" & Codec.Img (Natural (Pts.Length)) & " / " & Codec.Img (Natural (Truth.Length)) & " 个表面点收回来(背景 70 个点一个没收)、"
                & "认错节 " & Codec.Img (Wrong) & " 个、离真值超过 Stats.Z 倍自报不确定度的 " & Codec.Img (Far) & " 个(最多 " & Codec.Fmt (Worst_Z, 2) & " 倍)"
                & "、配点噪声自己量出 " & Codec.Fmt (Sd_Px, 3) & " px(加的 ±0.15)"
                & " · 净空:最近 " & Codec.Fmt (C1.Dist, 4) & " ± " & Codec.Fmt (C1.Sd, 4) & "(真的 " & Codec.Fmt (True_Min, 4) & ")、第" & Integer'Image (C1.Link) & " 节、"
                & (if C1.To_Scene then "场景点" else "别的臂") & " · 朝它走 " & Codec.Fmt (T_Toward, 4) & " 就进带子、背着走 "
                & (if T_Away = Long_Float'Last then "碰不上" else Codec.Fmt (T_Away, 4) & " 碰上第 2 条臂")
                & "(第 2 条臂撤掉 ⇒ " & (if T_Solo = Long_Float'Last then "碰不上" else Codec.Fmt (T_Solo, 4)) & ")"
                & "、没量过的臂说不出 " & Boolean'Image (not K_None)
                & " · 自己:第 1 条臂第 3 节的点那一格是自己、墙上那一点不是、一帧里直接问的一样 " & Boolean'Image (Self_Ok)
                & " · 读数按装上时的组号取 " & Boolean'Image (Readings_Ok) & "(牙:按下标取 ⇒ 两条臂对调)"
                & " · 牙:每一格按整条臂的位姿搬视线(不截到那一节)⇒ 近端几节的点交不上、收不回来");
      end;
   end;
   --  ══ 身体文件里的静止噪声按量法版本认(路 4 查出来的,10-01):H4 / H7 两份文件的 ee_noise 0.0121 是胳膊还在慢慢挪的时候量的 ══
   --  ① 这一版写的文件(带着量法版本)装回 ⇒ 0.0121 照装(同一版量的信);② 同一份去掉版本(= 09-30 以前写的,H4 / H7 那样)⇒ 装回时记成"不信"、
   --  开机报告说要重量;③ 和这一回量的(0.0004)合:不信的那份 ⇒ 只用这一回量的;信的那份 ⇒ 照旧只放大不缩小(0.0121)。
   --  病:老量法量大了的噪声永远留着(取历来最大),新的"挡没挡"在人形上判不出短步被挡。牙:原来的合法(一律取历来最大)⇒ ③ 里不信的那份也留 0.0121
   declare
      use Ada.Text_IO;
      M1, M2, M3, Fresh, Mg_Old, Mg_New : Selfmap.Body_Map;
      H1, H2 : Zone.Hand_Vectors.Vector;
      T1, T2 : Act.Effect_Vectors.Vector;
      S1, S2 : Schema.Map;
      Note2, Note3 : Unbounded_String;
      Got2, Got3 : Boolean;
      Path : constant String := "/tmp/bd_selfcheck_noise.json";
      Old_Path : constant String := "/tmp/bd_selfcheck_noise_old.json";
      Text : Unbounded_String;
      Rep, Kp : Natural;
      Old_Max : Long_Float;
   begin
      M1.Arms := 1; M1.N_Cams := 1; M1.Per_Arm := Chan.Per_Arm; M1.Channels := Chan.Per_Arm;
      for Ch in 0 .. Chan.Per_Arm - 1 loop
         M1.Amp.Append (0.0065); M1.Delivered.Append (0.005); M1.Seen.Append (True);
         M1.Amp_Hist.Append (F64_Vectors.To_Vector (0.0065, 1)); M1.Deliv_Hist.Append (F64_Vectors.To_Vector (0.005, 1));
      end loop;
      M1.Cam_On_Arm.Append (0); M1.Jaws.Append (1);
      M1.EE_Noise := 0.0121; M1.Rot_Noise := 0.0002; M1.Jaw_Noise := 0.001;
      Bodyfile.Save (Path, "selfcheck", M1, H1, T1, S1);
      Got2 := Bodyfile.Load (Path, "selfcheck", M2, H2, T2, S2, Note2);
      --  去掉量法版本 = 09-30 以前写的文件
      declare
         F : File_Type;
         Tag : constant String := """noise_ver"":" & Codec.Img (Selfmap.Idle_Ver) & ",";
         P : Natural;
      begin
         Open (F, In_File, Path);
         while not End_Of_File (F) loop
            Append (Text, Get_Line (F));
         end loop;
         Close (F);
         P := Index (Text, Tag);
         if P > 0 then
            Delete (Text, P, P + Tag'Length - 1);
         end if;
         Create (F, Out_File, Old_Path);
         Put (F, To_String (Text));
         Close (F);
      end;
      Got3 := Bodyfile.Load (Old_Path, "selfcheck", M3, H2, T2, S2, Note3);
      Fresh := M1;
      Fresh.EE_Noise := 0.0004; Fresh.Rot_Noise := 0.0001; Fresh.Jaw_Noise := 0.0005;
      Bodyfile.Merge (M3, Fresh, Mg_Old, Rep, Kp);
      Bodyfile.Merge (M2, Fresh, Mg_New, Rep, Kp);
      Old_Max := Long_Float'Max (0.0121, Fresh.EE_Noise);   --  牙:原来的合法
      Check (Got2 and then M2.EE_Noise = 0.0121 and then Got3 and then M3.EE_Noise < 0.0 and then Index (Note3, "老量法") > 0
             and then Mg_Old.EE_Noise = 0.0004 and then Mg_Old.Rot_Noise = 0.0001 and then Mg_Old.Jaw_Noise = 0.0005
             and then Mg_New.EE_Noise = 0.0121 and then Old_Max = 0.0121,
             "身体文件的静止噪声按量法版本认:这一版写的装回 " & Codec.Fmt (M2.EE_Noise, 4) & " · 去掉版本的(09-30 以前写的)装回 "
             & (if M3.EE_Noise < 0.0 then "不信" else Codec.Fmt (M3.EE_Noise, 4)) & "(开机报告:" & To_String (Note3) & ")"
             & " · 和这一回量的 0.0004 合:不信的那份 ⇒ " & Codec.Fmt (Mg_Old.EE_Noise, 4) & "、信的那份 ⇒ " & Codec.Fmt (Mg_New.EE_Noise, 4)
             & " · 牙:原来一律取历来最大 ⇒ 不信的那份也留 " & Codec.Fmt (Old_Max, 4));
   end;
   --  ══ "上" = 板的法向(路 5 查出,10-01):没碰过面、板已经拟合出了面时,Up_Dir 原来给协议的 +z;接触集"它躺的面"用板法向 ⇒ 同一个"上"两种量法 ══
   --  协议的 +z 歪 10°、板是平的(板法向 = 真的竖直):没碰过面 ⇒ Up_Dir = 板法向;碰过一张面 ⇒ 以碰到的为准。
   --  病:"抬多高"、挑空地、接触集朝哪抬按两个不同的"上"算,差 10° 就差出去。牙:原来的 Up_Dir 没碰过面就给 +z(差 10°)
   declare
      C : Act.Context;
      Ten : constant Long_Float := Ada.Numerics.Pi / 18.0;
      Board : constant Geom.V3 := Geom.Ap (Geom.Rodrigues ([Ten, 0.0, 0.0]), [0.0, 0.0, 1.0]);
      Touch : constant Geom.V3 := Geom.Ap (Geom.Rodrigues ([0.0, Ten / 2.0, 0.0]), [0.0, 0.0, 1.0]);
      U1, U2 : Geom.V3;
      function Ang (A, B : Geom.V3) return Long_Float is
        (Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, A (0) * B (0) + A (1) * B (1) + A (2) * B (2)))));
      Old_Up : constant Geom.V3 := [0.0, 0.0, 1.0];   --  牙:原来没碰过面时给的
   begin
      C.Board_Plane := True; C.Board_N := Board; C.Touch_Valid := False;
      U1 := Act.Up_Dir (C);
      C.Touch_Valid := True; C.Touch_N := Touch;
      U2 := Act.Up_Dir (C);
      --  比方向用差的长度(arccos 在 1 附近只准到 1e-8 弧度那么粗)
      Check (Geom.Norm ([U1 (0) - Board (0), U1 (1) - Board (1), U1 (2) - Board (2)]) < 1.0e-12
             and then Geom.Norm ([U2 (0) - Touch (0), U2 (1) - Touch (1), U2 (2) - Touch (2)]) < 1.0e-12 and then Ang (Old_Up, Board) > 0.17,
             "「上」= 板的法向:协议的 +z 歪 10°、板是平的 ⇒ 没碰过面时 Up_Dir 离板法向 " & Codec.Fmt (Ang (U1, Board) * 180.0 / Ada.Numerics.Pi, 3)
             & "°、碰过一张面以后离碰到的面 " & Codec.Fmt (Ang (U2, Touch) * 180.0 / Ada.Numerics.Pi, 3) & "° · 牙:原来没碰过面就给 +z,离板法向 "
             & Codec.Fmt (Ang (Old_Up, Board) * 180.0 / Ada.Numerics.Pi, 1) & "°");
   end;

   --  ══ 白桌白墙(路 8 P8I,10-01):看不出 ≠ 动了 ══
   --  ① 一张有纹理的画面 ⇒ 看得出;一面白墙(只有 ±1 灰阶的噪声)⇒ 看不出;纹理只挤在一个象限(白墙前一小块)⇒ 看不出。
   --  ② 核对前半段:两张都看得出、配上的点够、没挪 ⇒ 没动;配上的点够、挪了 ⇒ 动了;有一张看不出 / 一个点都配不上 ⇒ 看不出(不当成动了)。
   --  ③ 开机认组:腕眼对着白墙(推臂时它分不出整幅挪没挪),头顶眼看得见那条臂 ——
   --     存着上一回的前半段(这一组是臂、长着第 1 台眼)⇒ 照存的认成臂、眼 = 第 1 台;什么都没存 ⇒ 照实是"一块零件",不硬认。
   --  病:配不上一个点被记成"位移中位 −1 px ⇒ 动了",整份从零量,白墙又量不出,开不了机(P8I);身体没变、只是换了一间屋子也这样。
   --  牙:原来的判法(Same_View:配上的点不到 10 个就"不是没动")⇒ 白墙那一对判成动了
   declare
      function Img (Kind : Natural) return Buf is
         B : Buf;
      begin
         for Y in 0 .. Fh - 1 loop
            for X in 0 .. Fw - 1 loop
               declare
                  Nz : constant Long_Float := Long_Float ((X * X * 31 + Y * 17 + X * Y * 7) mod 3 - 1);   --  ±1 灰阶的噪声
                  Val : constant Long_Float :=
                    (case Kind is
                        when 0 => 128.0 + 50.0 * Sin (0.9 * Long_Float (X) + 0.4 * Long_Float (Y)) + 35.0 * Sin (0.37 * Long_Float (X) - 1.1 * Long_Float (Y)),
                        when 1 => 230.0 + Nz,
                        when others => (if X < Fw / 2 and then Y < Fh / 2 then 128.0 + 80.0 * Sin (1.3 * Long_Float (X)) else 230.0 + Nz));
               begin
                  B.Append (U8 (Long_Float'Max (0.0, Long_Float'Min (255.0, Long_Float'Rounding (Val)))));
               end;
            end loop;
         end loop;
         return B;
      end Img;
      Tex : constant Buf := Img (0);
      Wall : constant Buf := Img (1);
      Corner : constant Buf := Img (2);
      --  静止地板:白墙那张自己和"下一拍"比 —— 下一拍的噪声换一个样、一样大(同一台相机前后两拍的噪声和相邻两个像素的噪声一样大)
      Wall2 : Buf;
      Fl : Picture.Floor_Map;
      J_Tex, J_Wall, J_Corner : Boolean;
      Few, Many_Same, Many_Moved : Floats;
      V_Wall, V_Few, V_Same, V_Moved : Readings.View_Says;
      Old_Wall : Boolean;
   begin
      for Y in 0 .. Fh - 1 loop
         for X in 0 .. Fw - 1 loop
            Wall2.Append (U8 (230 + (X * 13 + Y * Y * 29 + X * Y * 3 + 1) mod 3 - 1));   --  和上一拍不相干的另一份 ±1 噪声
         end loop;
      end loop;
      Fl := Picture.Null_Floor (Wall, Wall2, Fw, Fh, Picture.Min_Pixels (Fw, Fh));
      J_Tex := Readings.Can_Judge (Tex, Fl, Fw, Fh);
      J_Wall := Readings.Can_Judge (Wall, Fl, Fw, Fh);
      J_Corner := Readings.Can_Judge (Corner, Fl, Fw, Fh);
      for I in 1 .. 3 loop
         Few.Append (0.1);
      end loop;
      for I in 1 .. 40 loop
         Many_Same.Append (0.2); Many_Moved.Append (4.0);
      end loop;
      V_Wall := Readings.View_Verdict (J_Wall, J_Tex, 0, 10, Jointboot.Same_View (Floats'(F64_Vectors.Empty_Vector)));
      V_Few := Readings.View_Verdict (J_Tex, J_Tex, Natural (Few.Length), 10, Jointboot.Same_View (Few));
      V_Same := Readings.View_Verdict (J_Tex, J_Tex, Natural (Many_Same.Length), 10, Jointboot.Same_View (Many_Same));
      V_Moved := Readings.View_Verdict (J_Tex, J_Tex, Natural (Many_Moved.Length), 10, Jointboot.Same_View (Many_Moved));
      Old_Wall := not Jointboot.Same_View (Floats'(F64_Vectors.Empty_Vector));   --  牙:原来只问 Same_View ⇒ 白墙那一对 "不是没动" = 动了
      Check (J_Tex and then not J_Wall and then not J_Corner
             and then V_Wall = Readings.Unseen and then V_Few = Readings.Unseen and then V_Same = Readings.Same and then V_Moved = Readings.Moved and then Old_Wall,
             "白桌白墙 · 看不出 ≠ 动了:有纹理的画面看得出 " & Boolean'Image (J_Tex) & "、白墙 " & Boolean'Image (J_Wall) & "、纹理只挤在一个象限 " & Boolean'Image (J_Corner)
             & " · 核对:白墙那一对 ⇒ " & Readings.Image (V_Wall) & "、配上 3 个点 ⇒ " & Readings.Image (V_Few) & "、40 个没挪 ⇒ " & Readings.Image (V_Same)
             & "、40 个挪了 4 px ⇒ " & Readings.Image (V_Moved) & " · 牙:原来只问 Same_View ⇒ 白墙那一对" & (if Old_Wall then "判成动了" else "(没判成动了,牙没咬上)"));
   end;
   declare
      Gs : FG_Vectors.Vector;
      Cs : FC_Vectors.Vector;
      R_Prior, R_None : Fake_Result;
      Names : Strs;
      Eyes_P : Ints;
   begin
      Gs.Append (G_Of ("arm_joint_state", 6, 0.0));
      Cs.Append (Cam_Of (Empty, Vec ([0])));                  --  头顶眼:有纹理,看得见那条臂
      Cs.Append (Cam_Of (Vec ([0]), Empty, White => True));   --  腕眼:对着白墙
      Names.Append ("arm_joint_state"); Eyes_P.Append (1);
      R_Prior := Run_Fake (Gs, Cs, Names, Eyes_P);
      R_None := Run_Fake (Gs, Cs);
      Check (R_Prior.Ok and then Natural (R_Prior.Arms.Length) = 1 and then R_Prior.Arms (0).Eye = 1
             and then not R_None.Ok and then Role_Of (R_None, "state.arm_joint_state") = Selfmap.Piece,
             "白桌白墙 · 开机认组:腕眼对着白墙 —— 存着上一回的(这一组是臂、长着第 1 台眼)⇒ " & Codec.Img (Natural (R_Prior.Arms.Length)) & " 条臂、眼 "
             & (if R_Prior.Arms.Is_Empty then "?" else Integer'Image (R_Prior.Arms (0).Eye)) & " · 什么都没存 ⇒ "
             & Role_Img (Role_Of (R_None, "state.arm_joint_state")) & "(不硬认,开不了机就照实说)");
   end;
   --  ══ 眼长在谁身上按开机认组量的答(P8A 10-01:第 2 只手扫描撞柜子、运动学没量成,身体图只收量成的手上的眼 ⇒ 第 2 台相机被说成
   --  "不跟着我动的眼",清单告诉脑 "through the eye that does not move with me (camera index 2)")══
   --  两条臂认组时各认出一只眼(第 1、2 台),第 2 条臂没装上(身体图的旧字段 Cam_On_Arm 只有第 1 台)⇒ 不长在身上的眼只有第 0 台。
   --  牙:按旧字段答 ⇒ 第 0、2 台
   declare
      M : Selfmap.Body_Map := Map_Of (1, 3, Vec ([1]), Vec ([1]));
      G1, G2 : Selfmap.Group_Info;
      Old_Off : Ints;
   begin
      G1.Role := Selfmap.Arm; G1.Arm := 0; G1.Eyes.Append (1);
      G2.Role := Selfmap.Arm; G2.Arm := 1; G2.Eyes.Append (2);
      M.Groups.Append (G1); M.Groups.Append (G2);
      for Cm in 0 .. M.N_Cams - 1 loop
         if not M.Cam_On_Arm.Contains (Cm) then
            Old_Off.Append (Cm);
         end if;
      end loop;
      Check (Same (Selfmap.Graph.Eyes_Off_Arms (M), Vec ([0])) and then Same (Old_Off, Vec ([0, 2])),
             "眼长在谁身上 · 第 2 条臂没装上:不长在身上的眼 " & Show (Selfmap.Graph.Eyes_Off_Arms (M)) & "(该 [0];第 2 台长在没量成的那条臂上)"
             & " · 牙:按旧字段 Cam_On_Arm 答 ⇒ " & Show (Old_Off));
   end;
   --  ══ 整个我(大并行 §2 第 2 条):推一下它,我身上量得到的每一样都跟着动的那一组 ══
   --  ① 无人机测试台的样子:一条臂(龙门吊 6 个数,扛着机身那只眼)+ 一组哑巴(假抓握)⇒ 整个我 = 那条臂;
   --  ② x5 的样子:两条臂 ⇒ 没有(哪一条都不带着另一条);③ 有扛着全身的那组 ⇒ 就是它(不是臂);
   --  ④ 一条臂 + 一块长在哪儿量不出的零件 ⇒ 没有(那一块不一定跟着它动)。
   --  病:无人机说不出"整个我",脑说 me 绑不上(P8MD:键盘上只剩结局词)。牙:只认扛着全身的那组 ⇒ ① 也说没有
   declare
      function G (Role : Selfmap.Group_Role; Arm : Integer := -1) return Selfmap.Group_Info is
         Gi : Selfmap.Group_Info;
      begin
         Gi.Role := Role; Gi.Arm := Arm;
         return Gi;
      end G;
      M1, M2, M3, M4 : Selfmap.Body_Map;
      Only_Carrying : Integer := -1;   --  牙:只认扛着全身的那组
   begin
      M1.Groups.Append (G (Selfmap.Arm, 0)); M1.Groups.Append (G (Selfmap.Reading)); M1.Groups.Append (G (Selfmap.Mute));
      M2.Groups.Append (G (Selfmap.Arm, 0)); M2.Groups.Append (G (Selfmap.Arm, 1)); M2.Groups.Append (G (Selfmap.Closing, 0)); M2.Groups.Append (G (Selfmap.Closing, 1));
      M3.Groups.Append (G (Selfmap.Carrying)); M3.Groups.Append (G (Selfmap.Arm, 0));
      M4.Groups.Append (G (Selfmap.Arm, 0)); M4.Groups.Append (G (Selfmap.Piece));
      for I in 0 .. Natural (M1.Groups.Length) - 1 loop
         if M1.Groups (I).Role = Selfmap.Carrying then
            Only_Carrying := Integer (I);
         end if;
      end loop;
      Check (Selfmap.Graph.Whole_Group (M1) = 0 and then Selfmap.Graph.Whole_Arm (M1) = 0 and then Selfmap.Graph.Whole_Group (M2) = -1
             and then Selfmap.Graph.Whole_Group (M3) = 0 and then Selfmap.Graph.Whole_Arm (M3) = -1 and then Selfmap.Graph.Whole_Group (M4) = -1
             and then Only_Carrying = -1,
             "整个我:无人机的样子 ⇒ 第" & Integer'Image (Selfmap.Graph.Whole_Group (M1)) & " 组(第" & Integer'Image (Selfmap.Graph.Whole_Arm (M1)) & " 条臂)"
             & " · x5 的样子 ⇒" & Integer'Image (Selfmap.Graph.Whole_Group (M2)) & " · 有扛着全身的组 ⇒ 第" & Integer'Image (Selfmap.Graph.Whole_Group (M3))
             & " 组 · 一条臂 + 一块量不出长在哪的零件 ⇒" & Integer'Image (Selfmap.Graph.Whole_Group (M4))
             & " · 牙:只认扛着全身的那组 ⇒ 无人机说" & (if Only_Carrying < 0 then "没有" else "有"));
   end;

   --  ══ me = 整个我(大并行 §2 第 2 条、§5 路 1;10-01 主代理授权 act.ads / act.adb / act-round.adb 那几行):清单里"整个我"那条臂的零件
   --  带 Whole(Selfmap.Graph.Whole_Piece,Build_Listing 置),me 只收它(Act.Role_Wants 按整件问);绑不上时按开机量到的说为什么
   --  (Selfmap.Graph.Why_No_Me,act-round 的 Why_No_Role 念它)。三种身体,都是假身体真跑开机第一步(Find_Arms 一组一组推着认):
   --  ① 无人机那种:一组 6 个数扛着身上那只眼(头顶眼看得见它),没有夹爪 ⇒ 整个我 = 那条臂,me 绑得上;再按绑上的那条臂走点
   --     (位姿空间的假无人机:命令晚 1 拍起效,每拍走还差的 0.89 —— x5 量的延迟基线,读数抖一丝;先按开机的量法 Selfmap.Measure 量它,
   --     再 Selfmap.Walk_To 走到一个平移 (0.20, −0.10, −0.15)、转 (0, 0, 0.3) 的点)⇒ 落到指定处(平移、转动都差不到一档);
   --  ② x5 那种:两条臂互不带着 ⇒ me 绑不上,照实说"量出 2 条臂,哪一条都不带着别的";
   --  ③ 会走的人形那种:一组扛着全身(三只眼都跟着它整幅动)+ 两条臂各带腕眼和手 ⇒ 整个我 = 那一组;它不是一条臂 ⇒ me 绑不上,
   --     照实说是哪一组、为什么(我只会按臂上的零件走)。
   --  病:P8MD 无人机键盘上角色那一格是空的 —— me 恒为 False,绑不上的原因按手指数猜("没手指 ⇒ me 没接上""有手指 ⇒ 要点名到零件")。
   --  牙:① 按种类问(原来的 Role_Wants (R, Kind))⇒ me 一块都绑不上 ⇒ 没有臂可走、不落点;② 原来按手指数说的那句不提臂;
   --     ③ 认不出扛着全身的那组(当成读数)⇒ 说成"量出 2 条臂,哪一条都不带着别的"
   declare
      function Bound_Arm (M : Selfmap.Body_Map; By_Kind : Boolean) return Integer is
      begin
         for A in 0 .. Selfmap.Graph.Arm_Count (M) - 1 loop
            declare
               It : Act.Item;
            begin
               It.Kind := Act.Piece; It.Arm := A; It.Located := True;
               It.Whole := Selfmap.Graph.Whole_Piece (M, A);
               if (if By_Kind then Act.Role_Wants (Sinew.Rl_Me, It.Kind) else Act.Role_Wants (Sinew.Rl_Me, It)) then
                  return Integer (A);
               end if;
            end;
         end loop;
         return -1;
      end Bound_Arm;
      function Has (S, Part : String) return Boolean is (Ada.Strings.Fixed.Index (S, Part) > 0);
      --  原来那句(按手指数;牙 ②)
      function Old_Why (N_Finger, N_Grip : Natural) return String is
        ((if N_Finger + N_Grip = 0 then "me 是【整个我】:我身上没量出手指和爪心,本该就是它 —— 可这一版还不会按整个机身走(me 没接上)"
          else "me 是【整个我】,只有推一下整幅画面跟着变、身上又分不出零件的机体才有它;我身上量得出 " & Codec.Img (N_Finger) & " 瓣手指、"
               & Codec.Img (N_Grip) & " 组爪心,所以要点名到零件:写 grasper"));

      --  ① 的走点:位姿空间的假无人机(一组读数 = 机身的位姿),主线程当身体,手的任务里跑驱动真的 Selfmap.Measure + Walk_To
      procedure Walk_Drone (Arm : Integer; Off : Table.Vec; Landed : out Boolean; Miss_T, Miss_R : out Long_Float;
                            Why : out Selfmap.Walk_End; Steps : out Natural; Beats : out Natural) is
         Tn : constant Long_Float := 0.005;    --  一步看得见的那一档(平移;同路 4 那具假身体)
         Tr : constant Long_Float := 0.0025;   --  转动那一档
         Alpha : constant Long_Float := 0.89;  --  每拍走还差的(x5 量的:命令发出下一拍走 89%)
         Start : constant Plug.Arm_Pose := [0.0, -0.2, 0.9, 1.0, 0.0, 0.0, 0.0];
         Goal : constant Plug.Arm_Pose := Chan.Compose (Start, Off);
         X, Y, Last_T : Plug.Arm_Pose := Start;
         Due : Integer := -1;          --  排着的那条命令哪一拍起效(-1 = 没有)
         Due_T : Plug.Arm_Pose := Start;
         Beat : Natural := 0;
         Lk : Plug.Link;
         Pic : Plug.Cam;
         Mw : Selfmap.Body_Map;
         Mw_Ok : Boolean := False;
         function To_Q (P : Plug.Arm_Pose) return Floats is
            Q : Floats;
         begin
            for V of P loop
               Q.Append (V);
            end loop;
            return Q;
         end To_Q;
         function To_Pose (Q : Floats) return Plug.Arm_Pose is
            P : Plug.Arm_Pose := [others => 0.0];
         begin
            for I in P'Range loop
               if I < Natural (Q.Length) then
                  P (I) := Q (I);
               end if;
            end loop;
            return P;
         end To_Pose;
         function Rot_Len (D : Table.Vec) return Long_Float is (Sqrt (D (3) ** 2 + D (4) ** 2 + D (5) ** 2));
         procedure Fake_Cmd (C : in out Plug.Cmd; Ok : out Boolean) is
         begin
            C.Kind := Plug.Joint; C.Group := Integer (C.Arm); C.Q := To_Q (C.Pose);
            Ok := True;
         end Fake_Cmd;
         function Frame_Now return Plug.Frame is
            Ff : Plug.Frame;
            N : constant Long_Float := 1.0e-6 * Long_Float ((Beat * 7) mod 5 - 2) / 2.0;
            V : Table.Vec := Table.Zero_Vec;
            P : Plug.Arm_Pose;
         begin
            V (0) := N; V (1) := -N; V (2) := N; V (5) := N;
            P := Chan.Compose (X, V);
            Ff.EE.Append (P); Ff.Joints.Append (To_Q (P)); Ff.Cams.Append (Pic); Ff.Seq := Beat;
            return Ff;
         end Frame_Now;
         procedure Advance is
            Mg : constant Plug.Cmd := Plug.Lock_Merged;
            V : Table.Vec;
            D : Table.Vec;
         begin
            for K in 0 .. Natural'Min (Natural (Mg.Groups.Length), Natural (Mg.Qs.Length)) - 1 loop
               if Mg.Groups (K) = 0 then
                  declare
                     T : constant Plug.Arm_Pose := To_Pose (Mg.Qs (K));
                  begin
                     if (for some I in T'Range => abs (T (I) - Last_T (I)) > 1.0e-9) then
                        Due := Beat + 1; Due_T := T; Last_T := T;
                     end if;
                  end;
               end if;
            end loop;
            if Due >= 0 and then Beat >= Due then
               Y := Due_T; Due := -1;
            end if;
            D := Chan.Delivered (X, Y);
            V := Table.Zero_Vec;
            for I in 0 .. Chan.Per_Arm - 1 loop
               V (I) := Alpha * D (I);
            end loop;
            X := Chan.Compose (X, V);
         end Advance;
         task type Hand;
         task body Hand is
            Fr : Plug.Frame := Frame_Now;
         begin
            Lockstep.Begin_Hand (0);
            begin
               declare
                  St : Floats;
                  Step_Px : Plug.Floats_Vectors.Vector;
                  Eyes : Ints;
               begin
                  St.Append (Tn); St.Append (Tr);
                  Step_Px.Append (St); Eyes.Append (0);
                  Selfmap.Measure (Lk, Fr, Mw, Mw_Ok, Step_Px, Eyes => Eyes, World => 0);
               end;
               if Mw_Ok and then Arm >= 0 then
                  declare
                     Wk : Selfmap.Walk;
                     Lm : Selfmap.Limits;
                     Went, Turned : Long_Float;
                  begin
                     Lm.Reach := True;
                     --  最多 50 步:自检自己的保险(驱动里走到一个定了的目标不设步数,出口是 Gained)
                     Selfmap.Walk_To (Lk, Mw, (Arm => Natural (Arm), Goal => Goal, Jaw => <>), Lm, Tn, Tr, 50, Fr, Wk, Went, Turned, Steps, Why);
                  end;
               end if;
            exception
               when E : others =>
                  Put_Line ("  🔴 路 1 假无人机的手出错:" & Ada.Exceptions.Exception_Information (E));
                  Fails := Fails + 1;
            end;
            Lockstep.Done;
         end Hand;
      begin
         Why := Selfmap.Lost_Link; Steps := 0;
         Pic.W := 8; Pic.H := 8;
         for I in 0 .. Pic.W * Pic.H - 1 loop
            Pic.Gray.Append (U8 (100));
         end loop;
         Plug.Set_Hooks (null, Fake_Cmd'Unrestricted_Access);
         Plug.Set_Reach (null); Plug.Set_Limit (null);
         Lockstep.Clear;
         Plug.Lock_Begin;
         declare
            H : Hand;
         begin
            Lockstep.Start (0, H'Identity);
            loop
               Lockstep.Run (0);
               exit when Lockstep.Finished (0);
               Beat := Beat + 1;
               Advance;
               declare
                  Ff : constant Plug.Frame := Frame_Now;
               begin
                  Lk.Seq := Beat;
                  Plug.Note_Beat (Lk, Ff);
                  Plug.Lock_Feed (Ff, Ok => Beat <= 20_000);   --  自检自己的保险:走不完就断线(手照样收得了尾),不挂住自检
               end;
            end loop;
         end;
         Plug.Lock_End;
         Lockstep.Clear;
         Plug.Set_Hooks (null, null);
         Miss_T := Table.Norm (Chan.Delivered (X, Goal), Chan.Pos_Channels);
         Miss_R := Rot_Len (Chan.Delivered (X, Goal));
         Beats := Beat;
         Landed := Mw_Ok and then Why = Selfmap.Arrived and then Miss_T <= Tn and then Miss_R <= Tr;
      end Walk_Drone;

      Rd, Rx, Rh : Fake_Result;
      Md, Mx, Mh, Mh_Old : Selfmap.Body_Map;
   begin
      --  ① 无人机那种
      declare
         Gs : FG_Vectors.Vector;
         Cs : FC_Vectors.Vector;
      begin
         Gs.Append (G_Of ("arm_joint_state", 6, 0.0));
         Gs.Append (Pose_Of_Group ("ee_pose", 0));
         Cs.Append (Cam_Of (Empty, Vec ([0])));
         Cs.Append (Cam_Of (Vec ([0]), Empty));
         Rd := Run_Fake (Gs, Cs);
      end;
      --  ② x5 那种
      declare
         Gs : FG_Vectors.Vector;
         Cs : FC_Vectors.Vector;
      begin
         Gs.Append (G_Of ("left_arm_joint_state", 6, 0.0));
         Gs.Append (G_Of ("right_arm_joint_state", 6, 0.0));
         Gs.Append (G_Of ("left_ee_joint_state", 1, 1.0, Lo => 0.0, Hi => 1.0));
         Gs.Append (G_Of ("right_ee_joint_state", 1, 1.0, Lo => 0.0, Hi => 1.0));
         Cs.Append (Cam_Of (Empty, Vec ([0, 1, 2, 3])));
         Cs.Append (Cam_Of (Vec ([0]), Vec ([2])));
         Cs.Append (Cam_Of (Vec ([1]), Vec ([3])));
         Rx := Run_Fake (Gs, Cs);
      end;
      --  ③ 会走的人形那种:base 扛着全身(头上那只眼长在它上面,两只腕眼也跟着它走),两条臂各带腕眼,两只手各一个合拢通道
      declare
         Gs : FG_Vectors.Vector;
         Cs : FC_Vectors.Vector;
      begin
         Gs.Append (G_Of ("base", 3, 0.0));
         Gs.Append (G_Of ("left_arm", 6, 0.0));
         Gs.Append (G_Of ("right_arm", 6, 0.0));
         Gs.Append (G_Of ("left_hand", 1, 1.0, Lo => 0.0, Hi => 1.0));
         Gs.Append (G_Of ("right_hand", 1, 1.0, Lo => 0.0, Hi => 1.0));
         Cs.Append (Cam_Of (Vec ([0]), Vec ([1, 2, 3, 4])));
         Cs.Append (Cam_Of (Vec ([0, 1]), Vec ([3])));
         Cs.Append (Cam_Of (Vec ([0, 2]), Vec ([4])));
         Rh := Run_Fake (Gs, Cs);
      end;
      --  开机时几条臂 = 量出来的布局的(Selfmap.Measure 按 Plug.Arms 填 M.Arms;Find_Arms 只填 Groups)
      Md := Rd.Map; Md.Arms := Rd.Lay.N_Arms;
      Mx := Rx.Map; Mx.Arms := Rx.Lay.N_Arms;
      Mh := Rh.Map; Mh.Arms := Rh.Lay.N_Arms;
      Mh_Old := Mh;
      for I in 0 .. Natural (Mh_Old.Groups.Length) - 1 loop
         if Mh_Old.Groups (I).Role = Selfmap.Carrying then
            declare
               Gi : Selfmap.Group_Info := Mh_Old.Groups (I);
            begin
               Gi.Role := Selfmap.Reading;
               Mh_Old.Groups.Replace_Element (I, Gi);
            end;
         end if;
      end loop;
      declare
         Me_D : constant Integer := Bound_Arm (Md, By_Kind => False);
         Me_D_Old : constant Integer := Bound_Arm (Md, By_Kind => True);
         Me_X : constant Integer := Bound_Arm (Mx, By_Kind => False);
         Me_H : constant Integer := Bound_Arm (Mh, By_Kind => False);
         Why_D : constant String := Selfmap.Graph.Why_No_Me (Md);
         Why_X : constant String := Selfmap.Graph.Why_No_Me (Mx);
         Why_H : constant String := Selfmap.Graph.Why_No_Me (Mh);
         Why_H_Old : constant String := Selfmap.Graph.Why_No_Me (Mh_Old);
         Old_X : constant String := Old_Why (2, 2);   --  x5:两只手各一组爪心、各一瓣以上的手指(按原来那句的数法,数多少不改它不提臂)
         Hg : constant Integer := Grp (Rh, "state.base");
         Off : Table.Vec := Table.Zero_Vec;
         Landed : Boolean := False;
         Miss_T, Miss_R : Long_Float := Long_Float'Last;
         Why_W : Selfmap.Walk_End := Selfmap.Lost_Link;
         Steps_W, Beats_W : Natural := 0;
      begin
         Off (0) := 0.20; Off (1) := -0.10; Off (2) := -0.15; Off (5) := 0.3;
         if Me_D >= 0 then
            Walk_Drone (Me_D, Off, Landed, Miss_T, Miss_R, Why_W, Steps_W, Beats_W);
         end if;
         Check (Rd.Ok and then Me_D = 0 and then Why_D = "" and then Landed and then Me_D_Old = -1,
                "me · 无人机那种:整个我 = 第" & Integer'Image (Selfmap.Graph.Whole_Group (Md)) & " 组(第" & Integer'Image (Me_D + 1) & " 条臂),me 绑到它"
                & " · 走点:" & Selfmap.Walk_End'Image (Why_W) & "、" & Codec.Img (Steps_W) & " 步 " & Codec.Img (Beats_W) & " 拍,离指定处 平移 "
                & Codec.Fmt (Miss_T, 5) & "、转 " & Codec.Fmt (Miss_R, 5) & "(一档 0.005 / 0.0025)"
                & " · 牙:按种类问 ⇒ " & (if Me_D_Old < 0 then "me 绑不上、没有臂可走、不落点" else "(绑上了,牙没咬上)"));
         Check (Rx.Ok and then Me_X = -1 and then Has (Why_X, "2 条臂") and then Has (Why_X, "不带着") and then not Has (Old_X, "条臂"),
                "me · x5 那种:绑不上,照实说「" & Why_X & "」 · 牙:原来那句「" & Old_X & "」" & (if Has (Old_X, "条臂") then "(提了臂,牙没咬上)" else "不提臂"));
         Check (Rh.Ok and then Hg >= 0 and then Selfmap.Graph.Whole_Group (Mh) = Hg and then Selfmap.Graph.Whole_Arm (Mh) = -1 and then Me_H = -1
                and then Has (Why_H, "第 " & Codec.Img (Natural'Max (0, Hg)) & " 组") and then Has (Why_H, "扛着全身")
                and then Has (Why_H_Old, "2 条臂") and then not Has (Why_H_Old, "第 " & Codec.Img (Natural'Max (0, Hg)) & " 组"),
                "me · 会走的人形那种:整个我 = 第" & Integer'Image (Selfmap.Graph.Whole_Group (Mh)) & " 组(base,"
                & Role_Img (Role_Of (Rh, "state.base")) & "),不是一条臂 ⇒ me 绑不上,照实说「" & Why_H & "」"
                & " · 牙:认不出扛着全身的那组 ⇒ 「" & Why_H_Old & "」");
      end;
   end;

   --  ══ 一集和一集之间不带经验(路 7 查出,10-01):对方复位、开新的一集 ⇒ Episode.Begin_New 清掉上一集的世界、脑那一段程序、这一集的选择,身体留着 ══
   --  上一集留下:remember 记下的一个地方、上次认出名字的那只眼、跑到一半的一段程序(跑完了第一条)、"这一集已经自己换过一次眼睛了"、
   --  几何逼近记的观测;身体那一侧:身体图里的臂数、碰过的面(标成上一集的)。
   --  病:上一集 remember 的地方、认名字的眼带进下一集;下一轮接着跑上一集那段程序的下一条;第一集换过一次眼以后每一集都不许换。
   --  牙:原来 body_driver 里那一串(只清世界、框、碰的面、墙)⇒ 地方、认名字的眼、换过眼都还在,程序还在:执行器再走一步交出的是上一集那段的第二条
   declare
      procedure Dirty (C : in out Act.Context) is
         Pl : Act.Place;
         B : Plan.Bind_Entry;
         Gi : Selfmap.Group_Info;
      begin
         Pl.Name := To_Unbounded_String ("spot"); Pl.Cam := 1; Pl.Cu := 0.4; Pl.Cv := 0.6;
         C.Places.Append (Pl);
         C.Name_Cam := 1;
         B.Key := To_Unbounded_String ("ball"); B.Item := 3;
         C.Binds.Append (B);
         --  上一集那段程序:跑完了第一条(执行器走过一步),下一步该交出第二条
         C.Prog := Sinew.Parse ("say first" & ASCII.LF & "say second" & ASCII.LF & "done");
         C.Have_Prog := True;
         declare
            What : Runtime.Yield;
            I : Sinew.Instr;
         begin
            Runtime.Advance (C.Prog, C.M, What, I);
         end;
         C.Refused := To_Unbounded_String ("line 2: no");
         C.Eye_Chosen := True;
         C.Last_Prog := To_Unbounded_String ("do grasper close ball until stuck");
         C.Geo_Obs.Append (Geom.Obs'(others => <>));
         C.Map.Arms := 2;
         Gi.Role := Selfmap.Arm;
         C.Map.Groups.Append (Gi);
         C.Touch_Valid := True; C.Touch_N := [0.0, 0.0, 1.0]; C.Touch_Fresh := True;
      end Dirty;
      --  牙:原来 body_driver 干活循环里复位那一串(10-01 以前),一字不改
      procedure Old_Reset (C : in out Act.Context) is
      begin
         World.Reset_All (C.Wld);
         Memory.Clear (C.Mem);
         C.Recent := Null_Unbounded_String;
         C.Cam := C.Map.World_Cam;
         C.Boxed.Clear;
         C.Touch_Fresh := False; C.Bumps.Clear; C.Fingers_Aimed := False; C.Geo_Pw_Valid := False; C.Geo_Pw_Met := False; C.Geo_At_Above := False;
         C.Sil_Valid := False; C.Held_Set_Valid := False; C.Walls.Clear; C.No_Reach_Arm := -1;
         Act.Init_Tracks (C);
      end Old_Reset;
      Cn, Co : Act.Context;
      Old_Next : Unbounded_String;   --  牙那一份:执行器再走一步交出什么
   begin
      Dirty (Cn); Dirty (Co);
      Episode.Begin_New (Cn);
      Old_Reset (Co);
      declare
         What : Runtime.Yield;
         I : Sinew.Instr;
      begin
         if Co.Have_Prog then
            Runtime.Advance (Co.Prog, Co.M, What, I);
            Old_Next := To_Unbounded_String (Runtime.Yield'Image (What) & (if What = Runtime.Y_Say then " " & To_String (I.Text) else ""));
         end if;
      end;
      Check (Cn.Places.Is_Empty and then Cn.Name_Cam = -1 and then not Cn.Eye_Chosen and then Cn.Geo_Obs.Is_Empty
             and then not Cn.Have_Prog and then Cn.Prog.Code.Is_Empty and then Cn.M.PC = 0 and then Cn.Binds.Is_Empty
             and then Cn.Refused = Null_Unbounded_String and then Cn.Last_Prog = Null_Unbounded_String
             and then Cn.Map.Arms = 2 and then Natural (Cn.Map.Groups.Length) = 1 and then Cn.Touch_Valid and then not Cn.Touch_Fresh
             and then not Co.Places.Is_Empty and then Co.Name_Cam = 1 and then Co.Eye_Chosen and then Co.Have_Prog
             and then To_String (Old_Next) = "Y_SAY second",
             "新的一集 · 复位以后:记下的地方 " & Codec.Img (Natural (Cn.Places.Length)) & " 个、认名字的眼" & Integer'Image (Cn.Name_Cam)
             & "、程序 " & (if Cn.Have_Prog then "还在" else "清了") & "(" & Codec.Img (Natural (Cn.Prog.Code.Length)) & " 条、绑定 "
             & Codec.Img (Natural (Cn.Binds.Length)) & ",下一轮问脑要新的)"
             & "、换过眼 " & Boolean'Image (Cn.Eye_Chosen) & "、几何逼近的观测 " & Codec.Img (Natural (Cn.Geo_Obs.Length)) & " 笔"
             & " · 身体留着(臂 " & Codec.Img (Cn.Map.Arms) & "、碰过的面 " & Boolean'Image (Cn.Touch_Valid)
             & "、标成上一集的 " & Boolean'Image (not Cn.Touch_Fresh) & ")"
             & " · 牙:原来那一串 ⇒ 地方 " & Codec.Img (Natural (Co.Places.Length)) & " 个、认名字的眼" & Integer'Image (Co.Name_Cam)
             & "、换过眼 " & Boolean'Image (Co.Eye_Chosen) & "、程序还在,执行器下一步交出「" & To_String (Old_Next) & "」(上一集那段的第二条)");
   end;

   --  ══ 探针那一推看没看见(Probe;Act.Probe_Effects 用它,10-01 主代理批):命令实到了、画面里那一点一个像素都没挪(按驱动的跟法)⇒ 不算量到 ══
   --  假身体(只有画面这一边):推 Amp,那一点在画面里真挪 K × Amp 像素;驱动的跟法(Act.Retrack 跟手上那一块)流不到半个像素就判跟丢、放回原处。
   --  V1B79 落盘重放(驱动自己的光流,10-01):通道 6 一推实到 0.0127,手在头顶眼里真挪 0.67–0.87 px(OpenCV LK / 相位相关),
   --  驱动的光流只看到 0.28–0.37 px ⇒ 跟丢、放回原处 ⇒ 原来照样记成"这一列是零"、还信它。A 照它取(一推 0.3 px),B 更小(一推 0.2 px);
   --  地板按那一炮静止两帧重跟量的(0.010 / 0.013 px)。
   --  病:没挪的那一推当成"量到了零",响应表里这一列是假的零,解算以为这根通道不动手。
   --  牙:① 原来的判法(命令实到超过读数噪声就算)⇒ 第一推就收、记成零;② 新判法配原来的"加倍了一点没多跑"(拿跟丢的 0 比)⇒ B 两推都跟丢就判成零、不再加倍
   declare
      Amp0 : constant Long_Float := 0.0127;
      Cap : constant Long_Float := 8.0 * Amp0;
      Lost_Px : constant Long_Float := 0.5;            --  Retrack:手动了、这儿流不到半个像素 ⇒ 跟丢(它那一行的数)
      W : constant Long_Float := 640.0;
      Static : Floats;                                 --  静止两帧上重跟的挪动平方(画幅²)
      Floor : Long_Float;
      type Rule is (New_Rule, Old_Seen, Old_Next);
      --  一路加倍到看见(或上限 / 判成零):Pushes = 推了几推,Col = 记进表的那一列(像素 / 单位),Got = 收下了
      procedure Ladder (K_Px : Long_Float; R : Rule; Pushes : out Natural; Col : out Long_Float; Got : out Boolean) is
         Amp : Long_Float := Amp0;
         Last : Long_Float := -1.0;
      begin
         Pushes := 0; Col := 0.0; Got := False;
         loop
            declare
               Px : constant Long_Float := K_Px * Amp;
               Lost : constant Boolean := Px < Lost_Px;
               Ran : constant Long_Float := (if Lost then 0.0 else Px / W);   --  跟丢 ⇒ 放回原处
               Meas : constant Long_Float := (if Lost then -1.0 else Ran);
               Seen : constant Boolean := (if R = Old_Seen then True else Probe.Moved (False, Lost, Ran, Floor));
               Nx : Probe.Next_Step;
            begin
               Pushes := Pushes + 1;
               if R = Old_Next and then not Seen then
                  --  原来的:幅度 × 2 过上限 ⇒ 不用;加倍以后(连跟丢的 0 一起比)一点没多 ⇒ 零
                  Nx := (if Amp * 2.0 > Cap then Probe.At_Cap elsif Last >= 0.0 and then Ran <= Last then Probe.Is_Zero else Probe.Double_It);
                  Last := Ran;
               else
                  Nx := Probe.Next (Seen, Meas, Last, Amp, Cap);
                  Last := Meas;
               end if;
               case Nx is
                  when Probe.Take_It =>
                     Col := Ran * W / Amp; Got := True;
                     return;
                  when Probe.At_Cap | Probe.Is_Zero =>
                     return;
                  when Probe.Double_It =>
                     Amp := 2.0 * Amp;
               end case;
            end;
         end loop;
      end Ladder;
      Ka : constant Long_Float := 0.3 / Amp0;
      Kb : constant Long_Float := 0.2 / Amp0;
      Pa, Pb, Pa_Old, Pb_Old : Natural;
      Ca, Cb, Ca_Old, Cb_Old : Long_Float;
      Ga, Gb, Ga_Old, Gb_Old : Boolean;
   begin
      Static.Append ((0.010 / W) ** 2); Static.Append ((0.013 / W) ** 2);
      Floor := Probe.Floor_Of (Probe.Track_Sigma (Static));
      Ladder (Ka, New_Rule, Pa, Ca, Ga);
      Ladder (Kb, New_Rule, Pb, Cb, Gb);
      Ladder (Ka, Old_Seen, Pa_Old, Ca_Old, Ga_Old);
      Ladder (Kb, Old_Next, Pb_Old, Cb_Old, Gb_Old);
      Check (Ga and then Pa = 2 and then abs (Ca - Ka) <= 1.0e-9 * Ka and then Gb and then Pb = 3 and then abs (Cb - Kb) <= 1.0e-9 * Kb
             and then not Probe.Moved (False, True, 0.0, Floor) and then not Probe.Moved (True, False, 1.0, Floor)
             and then Probe.Next (False, 0.002, 0.002, 2.0 * Amp0, Cap) = Probe.Is_Zero
             and then Probe.Next (False, -1.0, 0.002, 2.0 * Amp0, Cap) = Probe.Double_It
             and then Ga_Old and then Pa_Old = 1 and then Ca_Old = 0.0 and then not Gb_Old and then Pb_Old = 2,
             "探针 · 命令实到了、画面里那一点按驱动的跟法没挪(流不到半像素 ⇒ 跟丢)不算量到:A(一推 0.3 px)第 " & Codec.Img (Pa)
             & " 推看见、记 " & Codec.Fmt (Ca, 2) & " px/单位(真 " & Codec.Fmt (Ka, 2) & ") · B(一推 0.2 px)第 " & Codec.Img (Pb) & " 推看见、记 "
             & Codec.Fmt (Cb, 2) & "(真 " & Codec.Fmt (Kb, 2) & ") · 地板 = 静止两帧重跟的 Stats.Z 倍 = " & Codec.Fmt (Floor * W, 4) & " px"
             & " · 长在这只眼上的手的点不算看见 · 两推都量出来、加倍一点没多 ⇒ 零;有一推跟丢 ⇒ 接着加倍"
             & " · 牙:① 原来只问命令实到 ⇒ A 第 " & Codec.Img (Pa_Old) & " 推就收、这一列记成 " & Codec.Fmt (Ca_Old, 2)
             & ";② 拿跟丢的 0 比「没多跑」⇒ B 第 " & Codec.Img (Pb_Old) & " 推判成零、" & (if Gb_Old then "(收下了,牙没咬上)" else "没收下"));
   end;

   --  ══ 走一段关节直线,身体的表面挪多远(Links.Max_Shift,10-01 路 4 要的:保守推进每一步走多远)══
   --  合成的两节臂(两根竖着的转轴,第 1 根在第 0 根外 L1;第 0 节、第 1 节各 5 个表面点),参照那一刻第 1 节往回折着(折 Fold);
   --  沿关节直线走过去,每个点真走的路长(细分 4000 段量,按自己写的平面运动学算,不用 Links 的)≤ 上界 ≤ 2 倍。两种走法:
   --  ① 两个关节同向(L1 0.20、L2 0.40、折 2.8,q0 −2.0、q1 −2.8:第 1 节一路伸直,离第 0 根轴越来越远);② 反向(L1 0.30、L2 0.25、折 2.5,q0 +0.4、q1 −2.5)。
   --  没量过形状的臂:说不出(Known = False、Long_Float'Last)。
   --  病:上界比真挪的小 ⇒ 路 4 按它放的步子撞上;大得离谱 ⇒ 步子小到走不动。
   --  牙:只按参照那一刻点离轴多远算(不沿链往外加)⇒ ① 第 1 节伸直以后离第 0 根轴的距离没算上,上界比真挪的小
   declare
      function Rz (A : Long_Float; X : Geom.V3) return Geom.V3 is
        ([Cos (A) * X (0) - Sin (A) * X (1), Sin (A) * X (0) + Cos (A) * X (1), X (2)]);
      type Case_Rec is record
         L1, L2, Fold, Q0e, Q1e : Long_Float;
      end record;
      type Case_Arr is array (1 .. 2) of Case_Rec;
      Cases : constant Case_Arr := [(0.20, 0.40, 2.8, -2.0, -2.8), (0.30, 0.25, 2.5, 0.4, -2.5)];
      Ok_All : Boolean := True;
      Txt : Unbounded_String;
      Naive_Bites : Boolean := False;
   begin
      for Cs of Cases loop
         declare
            Md : Kinem.Model;
            Pl : Links.Placement;
            Pls : Links.Placement_Vectors.Vector;
            Pts : Links.Link_Pt_Vectors.Vector;
            A1 : constant Geom.V3 := [Cs.L1, 0.0, 0.0];
            Q0, Q1 : Floats;
            Known, Known_None : Boolean;
            Bound, None_Bound : Long_Float;
            True_Max : Long_Float := 0.0;
            Naive : Long_Float;
            R0_Ref, R1 : Long_Float := 0.0;
         begin
            Md.Valid := True; Md.N := 2;
            Md.Ax (0).W := [0.0, 0.0, 1.0]; Md.Ax (0).P := [0.0, 0.0, 0.0];
            Md.Ax (1).W := [0.0, 0.0, 1.0]; Md.Ax (1).P := A1;
            Md.Q0.Append (0.0); Md.Q0.Append (0.0);
            Pl.Model := Md; Pl.Valid := True;
            Pls.Append (Pl);
            for K in 0 .. 4 loop
               declare
                  R : constant Long_Float := 0.2 + 0.2 * Long_Float (K);
                  P0, P1 : Links.Link_Pt;
               begin
                  P0.Arm := 0; P0.Link := 0; P0.P := [R * Cs.L1, 0.0, 0.0]; P0.Views := 2;
                  P1.Arm := 0; P1.Link := 1; P1.P := [Cs.L1 + R * Cs.L2 * Cos (Cs.Fold), R * Cs.L2 * Sin (Cs.Fold), 0.0]; P1.Views := 2;
                  Pts.Append (P0); Pts.Append (P1);
               end;
            end loop;
            Links.Install (Pls, Pts);
            Q0.Append (0.0); Q0.Append (0.0);
            Q1.Append (Cs.Q0e); Q1.Append (Cs.Q1e);
            Bound := Links.Max_Shift (0, Q0, Q1, Known);
            --  真挪的:每个点沿关节直线细分走一遍,路长求和(平面运动学自己算)
            for P of Pts loop
               declare
                  Prev : Geom.V3 := P.P;
                  L : Long_Float := 0.0;
                  Nst : constant := 4000;   --  细分段数(自检自己的分辨率)
               begin
                  for I in 1 .. Nst loop
                     declare
                        T : constant Long_Float := Long_Float (I) / Long_Float (Nst);
                        X : constant Geom.V3 :=
                          (if P.Link = 0 then Rz (Cs.Q0e * T, P.P)
                           else Rz (Cs.Q0e * T, [A1 (0) + Rz (Cs.Q1e * T, [P.P (0) - A1 (0), P.P (1) - A1 (1), P.P (2) - A1 (2)]) (0),
                                                 A1 (1) + Rz (Cs.Q1e * T, [P.P (0) - A1 (0), P.P (1) - A1 (1), P.P (2) - A1 (2)]) (1),
                                                 A1 (2) + Rz (Cs.Q1e * T, [P.P (0) - A1 (0), P.P (1) - A1 (1), P.P (2) - A1 (2)]) (2)]));
                     begin
                        L := L + Geom.Norm ([X (0) - Prev (0), X (1) - Prev (1), X (2) - Prev (2)]);
                        Prev := X;
                     end;
                  end loop;
                  True_Max := Long_Float'Max (True_Max, L);
               end;
            end loop;
            --  牙:只按参照那一刻点离轴多远(第 0 根轴:所有点到原点;第 1 根轴:第 1 节的点到 A1)
            for P of Pts loop
               R0_Ref := Long_Float'Max (R0_Ref, Geom.Norm (P.P));
               if P.Link = 1 then
                  R1 := Long_Float'Max (R1, Geom.Norm ([P.P (0) - A1 (0), P.P (1) - A1 (1), P.P (2) - A1 (2)]));
               end if;
            end loop;
            Naive := abs Cs.Q0e * R0_Ref + abs Cs.Q1e * R1;
            Naive_Bites := Naive_Bites or else Naive < True_Max;
            --  没量过形状的臂(第 1 条:装上的只有第 0 条)
            None_Bound := Links.Max_Shift (1, Q0, Q1, Known_None);
            Ok_All := Ok_All and then Known and then Bound >= True_Max and then Bound <= 2.0 * True_Max
              and then not Known_None and then None_Bound = Long_Float'Last;
            Append (Txt, " · 真挪最多 " & Codec.Fmt (True_Max, 3) & "、上界 " & Codec.Fmt (Bound, 3) & "(" & Codec.Fmt (Bound / True_Max, 2)
                    & " 倍)、只按参照那一刻 " & Codec.Fmt (Naive, 3));
         end;
      end loop;
      Links.Install (Links.Placement_Vectors.Empty_Vector, Links.Link_Pt_Vectors.Empty_Vector);   --  别的焊点看到的是空的
      Check (Ok_All and then Naive_Bites,
             "表面挪多远(Max_Shift)· 同向 / 反向两种走法" & To_String (Txt) & " · 没量过形状的臂说不出"
             & " · 牙:只按参照那一刻点离轴多远 ⇒ " & (if Naive_Bites then "同向那一种比真挪的小" else "(不比真挪的小,牙没咬上)"));
   end;
end Welds_Path_1;
