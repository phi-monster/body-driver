with Readings;
with Stats;
separate (Jointboot)
--  ① 认身体(大并行 I1,路 1,10-01 改):身体报的每一组数都当一组通道,不按"几个数、值在哪"认。每个命令的名字(Layout.Command_Groups:
--  对方把命令回给我们看的那些名字;一个都不回 ⇒ 每一组)推一下:读数跟不跟、哪只眼里整幅在动(眼长在它上面)、哪只眼里只一块在动、
--  一点都没变。推法同原来:从极小起(起点 = 协议起点和 Stats.Z 倍静止噪声里大的那个),走得出来又看得见为止、每次翻一倍;
--  哪个数这一下没跟上,下一次往反方向推(贴着尽头的那个数往正推不动,往负推得动)。
--  认出来的:
--    臂 = 有眼整幅跟着它动、不是每只眼都动;扛着全身 = 每只眼都整幅在动(两只眼以上);
--    合拢通道 = 只一块在动、那一块在某条臂自己的眼里(看得最多的那条);零件 = 只一块在动、哪条臂的眼里都没有;
--    哑巴 = 读数跟着走、哪只眼里都没变(接入契约第 2 条不满足);推不动 = 推到最大读数也不跟(第 1 条);
--    说谎 = 一个方向推了画面变,反方向同样大的一推读数说走到了、画面却一个像素都没变(第 3 条:没动却不说)。
--  同名的几组里,到了以后离命令差得多的那组是读数,另一组是命令的回声(回声一点不差地等于命令)。别的读数(身体报的位姿……)
--  记下推哪一组时它跟着变。
--  认完把布局换成量出来的(Layout.Set_Measured):Joints = 各臂的读数组 + 它们的回声,Jaw = 各臂的合拢通道(按臂接起来)+ 回声,
--  Holds = 别的命令组(发命令时照读数保持)。x5:两条臂、各一个合拢通道,和原来按形状认的一模一样。
--  世界相机 = 不长在哪条臂上、不被扛着走的那几台里,推臂时变得最少的那台(都长在身上 ⇒ -1)
procedure Find_Arms (L : in out Plug.Link; F : in out Plug.Frame; M : in out Selfmap.Body_Map;
                     Arms : out Arm_Vectors.Vector; World_Cam : out Integer; Ok : out Boolean) is
   use type Selfmap.Group_Role;
   use type Readings.Eye_Verdict;
   Nc : constant Natural := Natural (F.Cams.Length);
   Ngr : constant Natural := Natural (L.Lay.Groups.Length);
   Cmd : constant Ints := Layout.Command_Groups (L.Lay);
   Nk : constant Natural := Natural'Min (Natural (Cmd.Length), Natural (F.Joints.Length));
   function Name_Of (K : Natural) return String is (Layout.Last_Seg (L.Lay.Groups (Natural (Cmd (K)))));
   function Path_Of (K : Natural) return String is (Layout.Joined (L.Lay.Groups (Natural (Cmd (K)))));
   type Probe_Info is record
      Rep : Integer := -1;                          --  同名的第一个命令组(推它 = 推这个名字)
      Reading : Integer := -1;                      --  同名几组里哪一组是读数(只在 Rep 上填;另几组是回声)
      Role : Selfmap.Group_Role := Selfmap.Unprobed;
      Arm : Integer := -1;                          --  臂:第几条;合拢通道:哪条臂的
      Amp, Got : Long_Float := 0.0;                 --  认出来那一推多大、读数实到多少(跟得最少的那个数)
      Frac : Floats;                                --  那一推每台相机变了多少画面(比例)
      Verd : Ints;                                  --  那一推每台相机的判法(Readings.Eye_Verdict'Pos)
      Eyes, Seen : Ints;                            --  整幅跟着动的眼 / 只看见一块动的眼
      Stuck : Ints;                                 --  推到最大两个方向都没跟上的那几个数
      Ever_Seen : Boolean := False;                 --  推的时候哪只眼里看见过动(读数却没跟上:照实说)
      Lied : Boolean := False;
      Lie_Note : Ada.Strings.Unbounded.Unbounded_String;
      Followers : Ints;                             --  推它时跟着变的别的组(Groups 的下标;同名的不算)
   end record;
   --  每个命令组一格(数组,不是容器:下面到处按下标读写它的字段,容器的引用计数在条件表达式里会漏放 —— 10-01 自检里收尾时报"正被引用")
   P : array (0 .. Natural'Max (1, Nk) - 1) of Probe_Info;
   Link_Ok : Boolean := True;

   --  第 R 组(命令下标)此刻离起点 Start、目标 Start + Dir × Amp 哪个近:近目标 = 跟上了这个数(两个假设的预测里取近的那个 = 等噪声下的最大似然)
   function Followed (Now, Start : Floats; I : Natural; Dir, Amp : Long_Float) return Boolean is
     (I < Natural (Now.Length) and then I < Natural (Start.Length)
      and then abs (Now (I) - (Start (I) + Dir * Amp)) < abs (Now (I) - Start (I)));

   --  推一下(命令下标 K 的名字):每个数往 Dir 那一边推 Amp,到了再读一帧,推回来。交出推之前 / 到了 / 再读 / 推回来四拍的画面和读数
   procedure Push (K : Natural; Q0 : Floats; Dir : Floats; Amp : Long_Float;
                   F0, F1, F1b, F2 : out Plug.Cam_Vectors.Vector; J0, J1 : out Plug.Floats_Vectors.Vector;
                   G0, G1 : out Plug.Floats_Vectors.Vector) is
      Tgt : Floats := Q0;
      S0 : Natural;
      Okg : Boolean;
   begin
      F0 := F.Cams; J0 := F.Joints; G0 := F.Groups;
      F1.Clear; F1b.Clear; F2.Clear; J1.Clear; G1.Clear;
      for I in 0 .. Natural (Tgt.Length) - 1 loop
         Tgt.Replace_Element (I, Q0 (I) + Dir (I) * Amp);
      end loop;
      S0 := L.Seq;   --  从发命令这一拍数起,读数几拍才停住(Settle,开机前半段就量)
      Go_Group (L, F, M, 0, K, Tgt, Amp * Third, Okg);
      if not Okg then
         Link_Ok := False;
         return;
      end if;
      F1 := F.Cams; J1 := F.Joints; G1 := F.Groups;
      --  到了再读一帧:画面比读数晚一拍(V1B58),走完那一帧和再读的这一帧都是到了以后的画面
      if not Plug.Sense (L, F) then
         Link_Ok := False;
         return;
      end if;
      F1b := F.Cams;
      M.Settle := Natural'Max (M.Settle, Selfmap.Settle_Since (L, S0, M.Joint_Noise));
      S0 := L.Seq;
      Go_Group (L, F, M, 0, K, Q0, Amp * Third, Okg);
      if not Okg then
         Link_Ok := False;
         return;
      end if;
      M.Settle := Natural'Max (M.Settle, Selfmap.Settle_Since (L, S0, M.Joint_Noise));
      F2 := F.Cams;
   end Push;

   function Four (A, B, C, D : Plug.Cam) return Boolean is
     (Plug.Has_Picture (A) and then Plug.Has_Picture (B) and then Plug.Has_Picture (C) and then Plug.Has_Picture (D)
      and then A.W = D.W and then A.H = D.H and then B.W = D.W and then C.W = D.W and then B.H = D.H and then C.H = D.H);

   --  这一组(命令下标 K)是名字 Name_Of (K) 的第一组吗
   function First_Of_Name (K : Natural) return Integer is
   begin
      for K2 in 0 .. K loop
         if Name_Of (K2) = Name_Of (K) then
            return Integer (K2);
         end if;
      end loop;
      return Integer (K);
   end First_Of_Name;

   procedure Probe (K : Natural; Pi : in out Probe_Info) is
      Q0 : constant Floats := F.Joints (K);
      N : constant Natural := Natural (Q0.Length);
      Dir : Floats := F64_Vectors.To_Vector (1.0, Ada.Containers.Count_Type (N));
      Ever : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (N));
      Amp : Long_Float := Long_Float'Max (Start_Amp, Stats.Z * M.Joint_Noise);
      F0, F1, F1b, F2 : Plug.Cam_Vectors.Vector;
      J0, J1, G0, G1 : Plug.Floats_Vectors.Vector;
      Decided : Boolean := False;
      Part_Once : Boolean := False;   --  上一推已经是"只一块在动"(看得见的头一推是刚过地板的那一推,要再推大一倍照那一推判)
   begin
      for Try in 0 .. Max_Doublings loop
         Push (K, Q0, Dir, Amp, F0, F1, F1b, F2, J0, J1, G0, G1);
         exit when not Link_Ok;
         declare
            R : Natural := K;   --  同名的几组里哪一组是读数:到了以后离命令差得多的那组(回声一点不差)
            Dev_R : Long_Float := -1.0;
            Any_Follow : Boolean := False;
            Got : Long_Float := Long_Float'Last;
            Fr : Floats;
            Vd : Ints;
            W_Set, P_Set : Ints;
            Unsure : Boolean := False;   --  有一只眼看见动了、分不出整幅还是一块(Readings.Undecided):推大一点再看,不先下结论
            Any_Seen : Boolean := False;
         begin
            for K2 in 0 .. Nk - 1 loop
               if Name_Of (K2) = Name_Of (K) and then K2 < Natural (J1.Length) and then Natural (J1 (K2).Length) = N then
                  declare
                     Dv : Long_Float := 0.0;
                  begin
                     for I in 0 .. N - 1 loop
                        Dv := Long_Float'Max (Dv, abs (J1 (K2) (I) - (Q0 (I) + Dir (I) * Amp)));
                     end loop;
                     if Dv > Dev_R then
                        Dev_R := Dv; R := K2;
                     end if;
                  end;
               end if;
            end loop;
            for I in 0 .. N - 1 loop
               if R < Natural (J0.Length) and then Followed (J1 (R), J0 (R), I, Dir (I), Amp) then
                  Any_Follow := True;
                  Ever.Replace_Element (I, True);
                  Got := Long_Float'Min (Got, Dir (I) * (J1 (R) (I) - J0 (R) (I)));
               end if;
            end loop;
            for C in 0 .. Nc - 1 loop
               if C < Natural (F2.Length) and then C < Natural (M.Floors.Length) and then Four (F0 (C), F1 (C), F1b (C), F2 (C)) then
                  declare
                     Fl : Picture.Floor_Map renames M.Floors (C);
                     Comps : constant Picture.Regions :=
                       Picture.Seen_Twice (F0 (C).Gray, F1 (C).Gray, F1b (C).Gray, F2 (C).Gray, Fl, F2 (C).W, F2 (C).H);
                     V : constant Readings.Eye_Verdict := Readings.Verdict (F0 (C).Gray, F1 (C).Gray, F1b (C).Gray, F2 (C).Gray, Fl, F2 (C).W, F2 (C).H);
                  begin
                     Fr.Append (Picture.Fraction (Picture.Either (Picture.Moved (F0 (C).Gray, F1 (C).Gray, Fl), Picture.Moved (F1 (C).Gray, F2 (C).Gray, Fl))));
                     Vd.Append (Readings.Eye_Verdict'Pos (V));
                     if not Comps.Is_Empty then
                        Any_Seen := True;
                        if V = Readings.Whole then
                           W_Set.Append (C);
                        elsif V = Readings.Part or else (V = Readings.Undecided and then Try = Max_Doublings) then
                           P_Set.Append (C);   --  推到最大还分不出:当它只看见一块(不说这只眼长在它上面)
                        elsif V = Readings.Undecided then
                           Unsure := True;
                        end if;
                     end if;
                  end;
               else
                  --  这台相机这四拍里有一拍没画面(插头留的空位):这一次看不出它动没动,不算它
                  Fr.Append (0.0);
                  Vd.Append (Readings.Eye_Verdict'Pos (Readings.Nothing));
               end if;
            end loop;
            Pi.Ever_Seen := Pi.Ever_Seen or else Any_Seen;
            if Any_Follow and then Any_Seen and then not Unsure and then not (W_Set.Is_Empty and then P_Set.Is_Empty)
              and then (not W_Set.Is_Empty or else Part_Once or else Try = Max_Doublings)
            then
               Decided := True;
               Pi.Reading := Integer (R); Pi.Amp := Amp; Pi.Got := Got; Pi.Frac := Fr; Pi.Verd := Vd; Pi.Eyes := W_Set; Pi.Seen := P_Set;
               Pi.Role := (if Nc >= 2 and then Natural (W_Set.Length) = Nc then Selfmap.Carrying
                           elsif not W_Set.Is_Empty then Selfmap.Arm
                           else Selfmap.Piece);
               --  推它时跟着变的别的组(同名的不算):超过 Stats.Z 倍静止噪声(统计门;仿真噪声是 0 ⇒ 变了就算)
               for G in 0 .. Natural'Min (Natural (G0.Length), Natural (G1.Length)) - 1 loop
                  if Layout.Last_Seg (L.Lay.Groups (G)) /= Name_Of (K) and then Natural (G0 (G).Length) = Natural (G1 (G).Length) then
                     declare
                        Mx : Long_Float := 0.0;
                     begin
                        for I in 0 .. Natural (G0 (G).Length) - 1 loop
                           Mx := Long_Float'Max (Mx, abs (G1 (G) (I) - G0 (G) (I)));
                        end loop;
                        if Mx > Stats.Z * M.Joint_Noise and then Mx > 0.0 then
                           Pi.Followers.Append (G);
                        end if;
                     end;
                  end if;
               end loop;
               exit;
            end if;
            --  只一块在动:看得见的头一推是刚过地板的那一推 —— 整幅挪得太少时只有最强的几道边过了地板,看着也像一块
            --  (人形 H4 第 1 只手 0.0001 那一推腕眼里刚过半)⇒ 再推大一倍,照那一推判(真是一块:世界推多大都不挪;整幅:变了的格跟着多起来)
            if Any_Follow and then Any_Seen and then not Unsure and then W_Set.Is_Empty and then not P_Set.Is_Empty then
               Part_Once := True;
            end if;
            --  没跟上的那几个数下一次往反方向推
            for I in 0 .. N - 1 loop
               if not (R < Natural (J0.Length) and then Followed (J1 (R), J0 (R), I, Dir (I), Amp)) then
                  Dir.Replace_Element (I, -Dir (I));
               end if;
            end loop;
         end;
         Amp := Amp * Grow;
      end loop;
      if not Link_Ok then
         return;
      end if;
      for I in 0 .. N - 1 loop
         if not Ever (I) then
            Pi.Stuck.Append (I);
         end if;
      end loop;
      if not Decided then
         Pi.Amp := Amp / Grow;
         Pi.Role := (if Natural (Pi.Stuck.Length) < N then Selfmap.Mute else Selfmap.Not_Following);
         return;
      end if;
      --  第 3 条(没动却不说):反方向同样大的一推 —— 读数说走到了,每只眼里却一个像素都没变(两次比较都超过地板的一个都没有)⇒ 说谎
      declare
         Back_Dir : Floats := Dir;
         Fb0, Fb1, Fb1b, Fb2 : Plug.Cam_Vectors.Vector;
         Jb0, Jb1, Gb0, Gb1 : Plug.Floats_Vectors.Vector;
         Rr : constant Natural := Natural (Pi.Reading);
         Follow_Back : Boolean := False;
         Any_Px : Boolean := False;
      begin
         for I in 0 .. N - 1 loop
            Back_Dir.Replace_Element (I, -Dir (I));
         end loop;
         Push (K, Q0, Back_Dir, Pi.Amp, Fb0, Fb1, Fb1b, Fb2, Jb0, Jb1, Gb0, Gb1);
         if not Link_Ok then
            return;
         end if;
         for I in 0 .. N - 1 loop
            if Rr < Natural (Jb0.Length) and then Followed (Jb1 (Rr), Jb0 (Rr), I, Back_Dir (I), Pi.Amp) then
               Follow_Back := True;
            end if;
         end loop;
         for C in 0 .. Nc - 1 loop
            if C < Natural (Fb2.Length) and then C < Natural (M.Floors.Length) and then Four (Fb0 (C), Fb1 (C), Fb1b (C), Fb2 (C)) then
               declare
                  Mv : constant Bools := Picture.Both (Picture.Moved (Fb0 (C).Gray, Fb1 (C).Gray, M.Floors (C)), Picture.Moved (Fb1b (C).Gray, Fb2 (C).Gray, M.Floors (C)));
               begin
                  for B of Mv loop
                     Any_Px := Any_Px or else B;
                  end loop;
               end;
            end if;
         end loop;
         if Follow_Back and then not Any_Px then
            Pi.Lied := True;
            Pi.Lie_Note := Ada.Strings.Unbounded.To_Unbounded_String
              ("往" & (if Back_Dir (0) > 0.0 then "正" else "负") & "推 " & Codec.Fmt (Pi.Amp, 4) & ":读数说走到了,每只眼里一个像素都没变");
         end if;
      end;
   end Probe;

   function Img_List (V : Ints) return String is
      R : Ada.Strings.Unbounded.Unbounded_String;
   begin
      for I in 0 .. Natural (V.Length) - 1 loop
         Ada.Strings.Unbounded.Append (R, (if I > 0 then "、" else "") & Codec.Img (Natural (V (I))));
      end loop;
      return Ada.Strings.Unbounded.To_String (R);
   end Img_List;
   function Role_Word (R : Selfmap.Group_Role) return String is
     (case R is
         when Selfmap.Arm => "臂",
         when Selfmap.Closing => "合拢通道",
         when Selfmap.Carrying => "扛着全身",
         when Selfmap.Piece => "零件(长在哪儿量不出)",
         when Selfmap.Mute => "哑巴(读数跟、画面不变)",
         when Selfmap.Not_Following => "推不动",
         when Selfmap.Reading => "读数",
         when Selfmap.Unprobed => "没量");

   Arm_Ks : Ints;                     --  第 A 条臂 = 命令下标 Arm_Ks (A)
begin
   Arms.Clear;
   World_Cam := -1;
   Ok := False;
   M.Groups.Clear;
   if Nk = 0 or else Nc = 0 then
      Say ("身体不报一组能推的数或没有相机 ⇒ 认不了身体");
      return;
   end if;
   for K in 0 .. Nk - 1 loop
      P (K) := Probe_Info'(others => <>);
      P (K).Rep := First_Of_Name (K);
   end loop;
   --  ── 逐个名字推一下 ──
   for K in 0 .. Nk - 1 loop
      if P (K).Rep = Integer (K) then
         if F.Joints (K).Is_Empty then
            P (K).Role := Selfmap.Not_Following;
            Say ("第" & Codec.Img (Natural (Cmd (K))) & " 组(" & Path_Of (K) & "):这一拍没读数 ⇒ 推不了");
         else
            declare
               Pi : Probe_Info := P (K);
            begin
               Probe (K, Pi);
               P (K) := Pi;
            end;
            if not Link_Ok then
               Say ("认组时线断了");
               return;
            end if;
         end if;
      end if;
   end loop;
   --  ── 臂的顺序 = 推的顺序;合拢通道挂到它那一块在其眼里变得最多的那条臂 ──
   for K in 0 .. Nk - 1 loop
      if P (K).Rep = Integer (K) and then P (K).Role = Selfmap.Arm then
         P (K).Arm := Integer (Arm_Ks.Length);
         Arm_Ks.Append (K);
      end if;
   end loop;
   for K in 0 .. Nk - 1 loop
      if P (K).Rep = Integer (K) and then P (K).Role = Selfmap.Piece then
         declare
            Best : Integer := -1;
            Bv : Long_Float := -1.0;
         begin
            for A in 0 .. Natural (Arm_Ks.Length) - 1 loop
               for E of P (Natural (Arm_Ks (A))).Eyes loop
                  for S of P (K).Seen loop
                     if S = E and then Natural (E) < Natural (P (K).Frac.Length) and then P (K).Frac (Natural (E)) > Bv then
                        Bv := P (K).Frac (Natural (E)); Best := Integer (A);
                     end if;
                  end loop;
               end loop;
            end loop;
            if Best >= 0 then
               P (K).Role := Selfmap.Closing;
               P (K).Arm := Best;
            end if;
         end;
      end if;
   end loop;
   --  ── 布局换成量出来的 ──
   declare
      Joints, Jaw, Holds : Layout.Paths;
      Closing_First, Closing_N, Jaw_Len : Ints;
      Echo_Of_Arm : array (0 .. Natural'Max (1, Natural (Arm_Ks.Length)) - 1) of Ints;
      Jaw_Echo : Layout.Paths;
      Jaw_Echo_Len : Ints;
      function Is_Reading (K : Natural) return Boolean is (P (Natural (P (K).Rep)).Reading = Integer (K));
   begin
      for A in 0 .. Natural (Arm_Ks.Length) - 1 loop
         Joints.Append (L.Lay.Groups (Natural (Cmd (Natural (P (Natural (Arm_Ks (A))).Reading)))));
      end loop;
      for A in 0 .. Natural (Arm_Ks.Length) - 1 loop
         for K in 0 .. Nk - 1 loop
            if P (K).Rep = Arm_Ks (A) and then not Is_Reading (K) then
               Echo_Of_Arm (A).Append (Natural (Joints.Length));
               Joints.Append (L.Lay.Groups (Natural (Cmd (K))));
            end if;
         end loop;
      end loop;
      for A in 0 .. Natural (Arm_Ks.Length) - 1 loop
         Closing_First.Append (Natural (Jaw.Length));
         for K in 0 .. Nk - 1 loop
            if P (K).Rep = Integer (K) and then P (K).Role = Selfmap.Closing and then P (K).Arm = Integer (A) then
               declare
                  Rk : constant Natural := Natural (P (K).Reading);
               begin
                  Jaw.Append (L.Lay.Groups (Natural (Cmd (Rk))));
                  Jaw_Len.Append (Natural (F.Joints (Rk).Length));
                  for K2 in 0 .. Nk - 1 loop
                     if P (K2).Rep = Integer (K) and then K2 /= Rk then
                        Jaw_Echo.Append (L.Lay.Groups (Natural (Cmd (K2))));
                        Jaw_Echo_Len.Append (Natural (F.Joints (K2).Length));
                     end if;
                  end loop;
               end;
            end if;
         end loop;
         Closing_N.Append (Natural (Jaw.Length) - Natural (Closing_First (A)));
      end loop;
      for I in 0 .. Natural (Jaw_Echo.Length) - 1 loop
         Jaw.Append (Jaw_Echo (I)); Jaw_Len.Append (Jaw_Echo_Len (I));
      end loop;
      for K in 0 .. Nk - 1 loop
         if P (K).Rep = Integer (K) and then P (K).Role not in Selfmap.Arm | Selfmap.Closing then
            Holds.Append (L.Lay.Groups (Natural (Cmd (if P (K).Reading >= 0 then Natural (P (K).Reading) else K))));
         end if;
      end loop;
      --  ── 身体图:每一组(Groups 的下标)量出来是什么 ──
      for G in 0 .. Ngr - 1 loop
         declare
            Gi : Selfmap.Group_Info;
            Kc : Integer := -1;
         begin
            Gi.Name := Ada.Strings.Unbounded.To_Unbounded_String (Layout.Joined (L.Lay.Groups (G)));
            Gi.N_Values := (if G < Natural (F.Groups.Length) then Natural (F.Groups (G).Length) else 0);
            for K in 0 .. Nk - 1 loop
               if Natural (Cmd (K)) = G then
                  Kc := Integer (K);
               end if;
            end loop;
            if Kc >= 0 then
               declare
                  Rp : constant Probe_Info := P (Natural (P (Natural (Kc)).Rep));
                  Rd : constant Integer := (if Rp.Reading >= 0 then Rp.Reading else P (Natural (Kc)).Rep);
               begin
                  if Rd = Kc then
                     Gi.Role := Rp.Role; Gi.Arm := Rp.Arm; Gi.Eyes := Rp.Eyes; Gi.Seen_In := Rp.Seen; Gi.Probe := Rp.Amp; Gi.Delivered := Rp.Got;
                     Gi.Lied := Rp.Lied;
                     for K2 in 0 .. Nk - 1 loop
                        if P (K2).Rep = P (Natural (Kc)).Rep and then Integer (K2) /= Kc and then Gi.Twin < 0 then
                           Gi.Twin := Integer (Cmd (K2));
                        end if;
                     end loop;
                  else
                     Gi.Role := Selfmap.Reading; Gi.Twin := Integer (Cmd (Natural (Rd))); Gi.Follows := Integer (Cmd (Natural (Rd)));
                  end if;
               end;
            else
               Gi.Role := Selfmap.Reading;
               Gi.Twin := (if G < Natural (L.Lay.Twin.Length) then L.Lay.Twin (G) else -1);
               for K in 0 .. Nk - 1 loop
                  if P (K).Rep = Integer (K) and then Gi.Follows < 0 then
                     for Fw of P (K).Followers loop
                        if Natural (Fw) = G and then P (K).Reading >= 0 then
                           Gi.Follows := Integer (Cmd (Natural (P (K).Reading)));
                        end if;
                     end loop;
                  end if;
               end loop;
            end if;
            M.Groups.Append (Gi);
         end;
      end loop;
      Layout.Set_Measured (L.Lay, Joints, Jaw, Holds, Closing_First, Closing_N, Jaw_Len, Natural (Arm_Ks.Length));
      --  布局换了:按新布局再收一帧(F.Joints / F.Jaw 的下标从此是量出来的那一套)
      if not Plug.Sense (L, F) then
         Say ("认完组以后取不到画面");
         return;
      end if;
      for A in 0 .. Natural (Arm_Ks.Length) - 1 loop
         declare
            Pa : constant Probe_Info := P (Natural (Arm_Ks (A)));
            Info : Arm_Info;
            Bv : Long_Float := -1.0;
         begin
            Info.Group := A;
            Info.Frac := Pa.Frac;
            Info.Probe := Pa.Amp;
            Info.Echoes := Echo_Of_Arm (A);
            for E of Pa.Eyes loop   --  几只眼都整幅跟着它动:变得最多的那只当它的眼(别的记在身体图里)
               if Natural (E) < Natural (Pa.Frac.Length) and then Pa.Frac (Natural (E)) > Bv then
                  Bv := Pa.Frac (Natural (E)); Info.Eye := Integer (E);
               end if;
            end loop;
            Arms.Append (Info);
         end;
      end loop;
   end;
   --  ── 开机报告(兼接入检查:哪一组不满足三条接入契约里的哪一条)──
   for G in 0 .. Ngr - 1 loop
      declare
         Gi : constant Selfmap.Group_Info := M.Groups (G);
         Kc : Integer := -1;
      begin
         for K in 0 .. Nk - 1 loop
            if Natural (Cmd (K)) = G then
               Kc := Integer (K);
            end if;
         end loop;
         declare
            Pr : constant Probe_Info := (if Kc >= 0 then P (Natural (P (Natural (Kc)).Rep)) else Probe_Info'(others => <>));
            Head : constant String := "第" & Codec.Img (G) & " 组(" & Ada.Strings.Unbounded.To_String (Gi.Name) & "," & Codec.Img (Gi.N_Values) & " 个数):";
         begin
            case Gi.Role is
               when Selfmap.Arm | Selfmap.Carrying | Selfmap.Closing | Selfmap.Piece =>
                  Say (Head & Role_Word (Gi.Role)
                       & (if Gi.Role = Selfmap.Arm then " 第" & Codec.Img (Natural (Gi.Arm) + 1) & " 条"
                          elsif Gi.Role = Selfmap.Closing then "(第" & Codec.Img (Natural (Gi.Arm) + 1) & " 条臂的)" else "")
                       & " · 每个数推 " & Codec.Fmt (Gi.Probe, 4) & " 就看得见(实到 " & Codec.Fmt (Gi.Delivered, 4) & ")"
                       & (if Gi.Eyes.Is_Empty then "" else " · 整幅跟着动的眼:第 " & Img_List (Gi.Eyes) & " 台")
                       & (if Gi.Seen_In.Is_Empty then "" else " · 只看见一块动的眼:第 " & Img_List (Gi.Seen_In) & " 台")
                       & (if Gi.Twin >= 0 then " · 第" & Codec.Img (Natural (Gi.Twin)) & " 组是它的回声" else ""));
               when Selfmap.Mute =>
                  Say (Head & "🔴 接入契约第 2 条不满足:推到 " & Codec.Fmt (Pr.Amp, 4) & " 读数跟着走,哪只眼里都没变(哑巴零件:加眼睛或镜子,代码修不了)");
               when Selfmap.Not_Following =>
                  Say (Head & "🔴 接入契约第 1 条不满足:两个方向推到 " & Codec.Fmt (Pr.Amp, 4) & " 读数都不跟(推不动)"
                       & (if Pr.Ever_Seen then ";推的时候画面却变过 —— 读数没报它动" else ""));
               when Selfmap.Reading =>
                  Say (Head & "读数" & (if Gi.Follows >= 0 and then Gi.Twin = Gi.Follows then "(第" & Codec.Img (Natural (Gi.Follows)) & " 组命令的回声)"
                                      elsif Gi.Follows >= 0 then "(推第" & Codec.Img (Natural (Gi.Follows)) & " 组时跟着变)" else "(推哪一组都不变)"));
               when Selfmap.Unprobed =>
                  Say (Head & "没量");
            end case;
            if Gi.Role not in Selfmap.Reading | Selfmap.Unprobed and then not Pr.Stuck.Is_Empty and then Gi.Role /= Selfmap.Not_Following then
               Say ("  🔴 接入契约第 1 条:第" & Codec.Img (G) & " 组里第 " & Img_List (Pr.Stuck) & " 个数两个方向都推不动");
            end if;
            if Gi.Lied then
               Say ("  🔴 接入契约第 3 条不满足(没动却不说):第" & Codec.Img (G) & " 组 " & Ada.Strings.Unbounded.To_String (Pr.Lie_Note));
            end if;
         end;
      end;
   end loop;
   --  世界相机 = 不长在哪条臂上、不被扛着走的那几台里,推臂时变得最少的那台(每台都长在身上 ⇒ 没有,-1;DIY 身体可以没有不动的眼)
   declare
      Bv : Long_Float := Long_Float'Last;
   begin
      for C in 0 .. Nc - 1 loop
         declare
            Mx : Long_Float := 0.0;
            On_Body : Boolean := False;
         begin
            for A of Arms loop
               if C < Natural (A.Frac.Length) then
                  Mx := Long_Float'Max (Mx, A.Frac (C));
               end if;
            end loop;
            for G of M.Groups loop
               if G.Role in Selfmap.Arm | Selfmap.Carrying then
                  for E of G.Eyes loop
                     On_Body := On_Body or else Natural (E) = C;
                  end loop;
               end if;
            end loop;
            if not On_Body and then Mx < Bv then
               Bv := Mx; World_Cam := C;
            end if;
         end;
      end loop;
   end;
   Say ("认组完:" & Codec.Img (Natural (Arms.Length)) & " 条臂" & (if Arms.Is_Empty then "" else "(合拢通道各 ")
        & (if Arms.Is_Empty then "" else Img_List (L.Lay.Closing_N) & " 组)")
        & " · 不动的眼:" & (if World_Cam >= 0 then "第" & Codec.Img (Natural (World_Cam)) & " 台" else "没有"));
   Ok := not Arms.Is_Empty;
end Find_Arms;
