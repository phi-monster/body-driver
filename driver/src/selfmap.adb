with Ada.Text_IO; use Ada.Text_IO;
with Codec;
with Chan;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
package body Selfmap is
   Start_Amp : constant Long_Float := 1.0e-4;   --  探针协议的起点(极小,翻倍到走得出来又看得见为止;起点多小不影响结果),无量纲协议
   Max_Doublings : constant := 12;              --  次数,无量纲

   function Jaw_Index (F : Plug.Frame; Arm : Natural) return Natural is
     (if Natural (F.Jaw.Length) > Arm then Arm else 0);

   function Jaw_Count (F : Plug.Frame; Arm : Natural) return Natural is
     (if F.Jaw.Is_Empty then 0 else Natural (F.Jaw (Jaw_Index (F, Arm)).Length));

   function Jaw_All (F : Plug.Frame; Arm : Natural) return Floats is
     (if F.Jaw.Is_Empty then F64_Vectors.Empty_Vector else F.Jaw (Jaw_Index (F, Arm)));

   function Jaw_Of (F : Plug.Frame; Arm : Natural; K : Natural := 0) return Long_Float is
     (if F.Jaw.Is_Empty or else K >= Jaw_Count (F, Arm) then 1.0
      else F.Jaw (Jaw_Index (F, Arm)) (K));

   procedure Idle (L : in out Plug.Link; F : in out Plug.Frame; N : Natural; Ok : out Boolean) is
   begin
      Ok := True;
      for I in 1 .. N loop
         if not Plug.Sense (L, F) then
            Ok := False;
            return;
         end if;
      end loop;
   end Idle;

   function Picture_Still (M : Body_Map; Before, After : Plug.Cam; Cam : Natural) return Boolean is
      --  静止 = 超过各自噪声地板的像素凑不成一团(最少像素数的几倍,倍数无量纲;去噪闪烁是撒开的单点)
      Mv : constant Bools := Picture.Moved (Before.Gray, After.Gray, M.Floors (Cam));
      Cnt : Natural := 0;
   begin
      for B of Mv loop
         if B then
            Cnt := Cnt + 1;
         end if;
      end loop;
      return Cnt <= 4 * Picture.Min_Pixels (Before.W, Before.H);
   end Picture_Still;

   function Pictures_Still (M : Body_Map; Before, After : Plug.Cam_Vectors.Vector) return Boolean is
   begin
      for C in 0 .. Natural'Min (Natural (Before.Length), Natural (After.Length)) - 1 loop
         if C < Natural (M.Floors.Length) and then not Picture_Still (M, Before (C), After (C), C) then
            return False;
         end if;
      end loop;
      return True;
   end Pictures_Still;

   procedure Wait_Still (L : in out Plug.Link; M : Body_Map; F : in out Plug.Frame; Max : Natural; Used : out Natural; Ok : out Boolean) is
      Prev : Plug.Cam_Vectors.Vector := F.Cams;
      Still : Natural := 0;
   begin
      Used := 0;
      Ok := True;
      for I in 1 .. Max loop
         if not Plug.Sense (L, F) then
            Ok := False;
            return;
         end if;
         Used := I;
         if Pictures_Still (M, Prev, F.Cams) then
            Still := Still + 1;
         else
            Still := 0;
         end if;
         Prev := F.Cams;
         exit when Still >= 2;
      end loop;
   end Wait_Still;

   procedure Go (L : in out Plug.Link; M : Body_Map; Arm : Natural; Target : Plug.Arm_Pose; Jaw : Floats;
                 F : in out Plug.Frame; Delivered : out Table.Vec; Frames : out Natural; Ok : out Boolean; Quick : Boolean := False;
                 Watch : Watcher := null; Joints : Floats := F64_Vectors.Empty_Vector; Group : Integer := -1;
                 Groups : Ints := Int_Vectors.Empty_Vector; Qs : Plug.Floats_Vectors.Vector := Plug.Floats_Vectors.Empty_Vector;
                 Tol : Long_Float := 0.0) is
      C : Plug.Cmd;
      P0 : constant Plug.Arm_Pose := (if Arm < Natural (F.EE.Length) then F.EE (Arm) else [others => 0.0]);
      Prev : Plug.Arm_Pose := P0;
      Still : Natural := 0;
      Send : Boolean := True;
      Halted : Boolean := False;
      --  关节目标:看哪几组读数、各自的目标
      W_G : Ints;
      W_Q : Plug.Floats_Vectors.Vector;
      Prev_All : Plug.Floats_Vectors.Vector;
      Is_Joint : constant Boolean := Group >= 0 or else not Groups.Is_Empty;
      Arrived : Natural := 0;
      Still_Frac : constant := 0.01;   --  百分之一(比例,见下)
   begin
      Delivered := Table.Zero_Vec;
      Frames := 0;
      C.Kind := Plug.Ee; C.Arm := Arm; C.Pose := Target; C.Jaw := Jaw;
      if not Groups.Is_Empty then
         C.Kind := Plug.Joint; C.Groups := Groups; C.Qs := Qs;
         W_G := Groups; W_Q := Qs;
      elsif Group >= 0 then
         C.Kind := Plug.Joint; C.Q := Joints; C.Group := Group;
         W_G.Append (Group); W_Q.Append (Joints);
      end if;
      for G of W_G loop
         Prev_All.Append (if G >= 0 and then G < Natural (F.Joints.Length) then F.Joints (Natural (G)) else F64_Vectors.Empty_Vector);
      end loop;
      loop
         if Send then
            Ok := Plug.Act (L, C);
            if not Ok then
               return;
            end if;
            Send := False;
         end if;
         if not Plug.Sense (L, F) then
            Ok := False;
            return;
         end if;
         Frames := Frames + 1;
         --  关节目标:"停稳"看这几组关节读数(不看位姿:只报关节的身体没有位姿读数)。
         --  到了目标附近(差 ≤ Tol,调用方按这一格的步子定)再有一拍不动 ⇒ 到了;没到目标就等连着两拍不动(被顶住 / 到头)
         --  (5 分钟一炮,2026-09-26:原来每格都等"连着两拍不动 + 量出来的稳定拍数",V1B3 扫描一格 9 拍)
         if Is_Joint then
            declare
               Moved, Miss : Long_Float := 0.0;
               --  一拍挪不到"到了"那个范围的百分之一 = 停了(比例;动作做完以后读数还会有极小的抖动,空闲时量的噪声是 0 ⇒ 不能拿它当"不动"的门,
               --  V1B4 2026-09-26:每格都等满 14 拍)
               Still_Gate : constant Long_Float := Long_Float'Max (M.Joint_Noise, Tol * Still_Frac);
            begin
               for Gi in 0 .. Natural (W_G.Length) - 1 loop
                  declare
                     G : constant Integer := W_G (Gi);
                  begin
                     if G >= 0 and then G < Natural (F.Joints.Length) and then Natural (Prev_All (Gi).Length) = Natural (F.Joints (Natural (G)).Length) then
                        for K in 0 .. Natural (Prev_All (Gi).Length) - 1 loop
                           Moved := Long_Float'Max (Moved, abs (F.Joints (Natural (G)) (K) - Prev_All (Gi) (K)));
                           if K < Natural (W_Q (Gi).Length) then
                              Miss := Long_Float'Max (Miss, abs (F.Joints (Natural (G)) (K) - W_Q (Gi) (K)));
                           end if;
                        end loop;
                        Prev_All.Replace_Element (Gi, F.Joints (Natural (G)));
                     end if;
                  end;
               end loop;
               Still := (if Moved <= Still_Gate then Still + 1 else 0);
               Arrived := (if Tol > 0.0 and then Miss <= Tol then Arrived + 1 else 0);
               --  连着两拍都到了目标附近 = 到了;没到目标就等连着两拍不动(被顶住 / 到头)
               exit when Arrived >= 2 or else (Still >= 2 and then Frames >= M.Settle)
                 or else Frames >= 12 + M.Settle or else (Quick and then Frames >= M.Settle);
            end;
         else
         exit when Arm >= Natural (F.EE.Length);
         --  途中每一拍看一眼:出事就把目标改成"停在此刻的位姿",同一条发命令的路再发一次
         if Watch /= null and then not Halted and then Watch (F) then
            Halted := True;
            C.Pose := F.EE (Arm);

            Send := True;
            Still := 0;
         end if;
         declare
            D : constant Table.Vec := Chan.Delivered (Prev, F.EE (Arm));
            Moved_P : constant Long_Float := Table.Norm (D, 3);
            Rv : constant Long_Float := D (3) ** 2 + D (4) ** 2 + D (5) ** 2;
         begin
            if Moved_P <= M.EE_Noise and then Rv <= M.Rot_Noise * M.Rot_Noise then
               Still := Still + 1;
            else
               Still := 0;
            end if;
            Prev := F.EE (Arm);
         end;
         exit when (Still >= 2 and then Frames >= M.Settle) or else Frames >= 12 + M.Settle or else (Quick and then Frames >= M.Settle);
         end if;
      end loop;
      if Arm < Natural (F.EE.Length) then
         Delivered := Chan.Delivered (P0, F.EE (Arm));
      end if;
   end Go;

   procedure Verify (L : in out Plug.Link; M : Body_Map; F : in out Plug.Frame; Ok_Body, Ok_Link : out Boolean; Note : out String_Note) is
      use Ada.Strings.Unbounded;
      T : Unbounded_String;
   begin
      Ok_Body := True; Ok_Link := True;
      for A in 0 .. M.Arms - 1 loop
         declare
            K : constant Natural := 0;          --  第一个平移通道
            Ch : constant Natural := A * Chan.Per_Arm + K;
            P0 : constant Plug.Arm_Pose := F.EE (A);
            F0 : constant Plug.Cam_Vectors.Vector := F.Cams;
            A_Cmd : Table.Vec := Table.Zero_Vec;
            Deliv, Back : Table.Vec;
            Frames : Natural;
            Ok2 : Boolean;
            Jaw0 : Floats;
            Visible : Boolean := False;
         begin
            if Ch >= Natural (M.Amp.Length) or else not M.Seen (Ch) then
               Append (T, "第" & Natural'Image (A + 1) & " 只手没有可核的通道;");
               Ok_Body := False;
            else
               Jaw0.Append (Jaw_Of (F, A));
               A_Cmd (K) := M.Amp (Ch);
               Go (L, M, A, Chan.Compose (P0, A_Cmd), Jaw0, F, Deliv, Frames, Ok2);
               if not Ok2 then
                  Ok_Link := False;
                  return;
               end if;
               for C in 0 .. Natural (F.Cams.Length) - 1 loop
                  if C < Natural (M.Floors.Length) then
                     declare
                        Mv : constant Bools := Picture.Moved (F0 (C).Gray, F.Cams (C).Gray, M.Floors (C));
                        Comps : constant Picture.Regions := Picture.Components (Mv, F.Cams (C).W, F.Cams (C).H, Picture.Min_Pixels (F.Cams (C).W, F.Cams (C).H));
                     begin
                        if not Comps.Is_Empty then
                           Visible := True;
                        end if;
                     end;
                  end if;
               end loop;
               Go (L, M, A, P0, Jaw0, F, Back, Frames, Ok2);
               if not Ok2 then
                  Ok_Link := False;
                  return;
               end if;
               declare
                  Expect : constant Long_Float := M.Delivered (Ch);
                  Got : constant Long_Float := Deliv (K);
               begin
                  --  差一半以内(比例,无量纲)算同一具身体
                  if abs (Got - Expect) <= 0.5 * abs Expect + M.EE_Noise and then Visible then
                     Append (T, "第" & Natural'Image (A + 1) & " 只手:命令 " & Codec.Fmt (M.Amp (Ch), 4) & " 实到 " & Codec.Fmt (Got, 4) & "(存的 " & Codec.Fmt (Expect, 4) & ")对得上;");
                  else
                     Append (T, "第" & Natural'Image (A + 1) & " 只手:实到 " & Codec.Fmt (Got, 4) & " 和存的 " & Codec.Fmt (Expect, 4) & " 对不上" & (if Visible then "" else "(画面里也没看见)") & ";");
                     Ok_Body := False;
                  end if;
               end;
            end if;
         end;
      end loop;
      Note.Text := T;
   end Verify;

   procedure Measure_Idle (L : in out Plug.Link; F : in out Plug.Frame; M : in out Body_Map; Ok : out Boolean) is
      N_Cams : constant Natural := Natural (F.Cams.Length);
      Arms : constant Natural := Natural'Min (Natural (F.EE.Length), M.Arms);
   begin
      Ok := True;
      --  ① 什么都不做时读数抖多少、画面抖多少(静止对)
      declare
         Prev_EE : Plug.Pose_Vectors.Vector := F.EE;
         Prev_Jaw : Plug.Floats_Vectors.Vector := F.Jaw;
         Prev_Q : Plug.Floats_Vectors.Vector := F.Joints;
         Prev_Gray : Plug.Cam_Vectors.Vector := F.Cams;
      begin
         for K in 1 .. 4 loop
            if not Plug.Sense (L, F) then
               Ok := False;
               return;
            end if;
            for A in 0 .. Arms - 1 loop
               declare
                  D : constant Table.Vec := Chan.Delivered (Prev_EE (A), F.EE (A));
               begin
                  M.EE_Noise := Long_Float'Max (M.EE_Noise, Table.Norm (D, 3));
                  M.Rot_Noise := Long_Float'Max (M.Rot_Noise, Sqrt (D (3) ** 2 + D (4) ** 2 + D (5) ** 2));
               end;
            end loop;
            for J in 0 .. Natural (F.Jaw.Length) - 1 loop
               if J < Natural (Prev_Jaw.Length) then
                  for K in 0 .. Natural'Min (Natural (F.Jaw (J).Length), Natural (Prev_Jaw (J).Length)) - 1 loop
                     M.Jaw_Noise := Long_Float'Max (M.Jaw_Noise, abs (F.Jaw (J) (K) - Prev_Jaw (J) (K)));
                  end loop;
               end if;
            end loop;
            for G in 0 .. Natural'Min (Natural (F.Joints.Length), Natural (Prev_Q.Length)) - 1 loop
               for K2 in 0 .. Natural'Min (Natural (F.Joints (G).Length), Natural (Prev_Q (G).Length)) - 1 loop
                  M.Joint_Noise := Long_Float'Max (M.Joint_Noise, abs (F.Joints (G) (K2) - Prev_Q (G) (K2)));
               end loop;
            end loop;
            Prev_EE := F.EE; Prev_Jaw := F.Jaw; Prev_Q := F.Joints;
            if K < 4 then
               Prev_Gray := F.Cams;
            end if;
         end loop;
         M.Floors.Clear; M.Pic_Floor.Clear;
         for C in 0 .. N_Cams - 1 loop
            declare
               Cw : constant Natural := F.Cams (C).W;
               Ch : constant Natural := F.Cams (C).H;
            begin
               M.Floors.Append (Picture.Null_Floor (Prev_Gray (C).Gray, F.Cams (C).Gray, Cw, Ch, Picture.Min_Pixels (Cw, Ch)));
               M.Pic_Floor.Append (Picture.Max_Diff (Prev_Gray (C).Gray, F.Cams (C).Gray));
            end;
         end loop;
      end;
   end Measure_Idle;

   procedure Measure (L : in out Plug.Link; F : in out Plug.Frame; M : out Body_Map; Ok : out Boolean;
                      Eyes : Ints := Int_Vectors.Empty_Vector; World : Integer := -1) is
      N_Cams : constant Natural := Natural (F.Cams.Length);
      Arms : constant Natural := Natural (F.EE.Length);
   begin
      M := (others => <>);
      M.Arms := Arms; M.N_Cams := N_Cams; M.Per_Arm := Chan.Per_Arm; M.Channels := Arms * Chan.Per_Arm;
      M.Jaws.Clear;
      for A in 0 .. Arms - 1 loop
         M.Jaws.Append (Integer (Natural'Max (1, Jaw_Count (F, A))));
      end loop;
      Ok := False;
      if Arms = 0 or else N_Cams = 0 then
         Put_Line ("[身] 没有末端位姿或没有相机,量不了身体");
         return;
      end if;
      declare
         Ok2 : Boolean;
      begin
         Measure_Idle (L, F, M, Ok2);
         if not Ok2 then
            return;
         end if;
      end;
      Put_Line ("[身] 静止噪声:本体位置 " & Codec.Fmt (M.EE_Noise, 5) & " m · 姿态 " & Codec.Fmt (M.Rot_Noise, 5) &
                " rad · 抓握读数 " & Codec.Fmt (M.Jaw_Noise, 4) & " · 各相机灰度地板 " &
                (if M.Pic_Floor.Is_Empty then "-" else Codec.Img (M.Pic_Floor (0))));
      --  ② 逐通道推一下再推回来
      M.Parts := Part_Vectors.To_Vector ((others => <>), Ada.Containers.Count_Type (M.Channels * N_Cams));
      M.Cam_Frac := Zeros (Arms * N_Cams);
      M.Amp := Zeros (M.Channels); M.Delivered := Zeros (M.Channels);
      M.Seen := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (M.Channels));
      M.Cam_On_Arm := Int_Vectors.To_Vector (-1, Ada.Containers.Count_Type (Arms));
      declare
         Last_Trans, Last_Rot : Long_Float := 0.0;
      begin
      for A in 0 .. Arms - 1 loop
         for K in 0 .. Chan.Per_Arm - 1 loop
            declare
               Ch : constant Natural := A * Chan.Per_Arm + K;
               P0 : constant Plug.Arm_Pose := F.EE (A);
               F0 : constant Plug.Cam_Vectors.Vector := F.Cams;
               Noise : constant Long_Float := (if K < 3 then M.EE_Noise else M.Rot_Noise);
               --  起点:同类通道(平移/转动)上一次被接受的幅度的一半(协议:从已知能走的档往下试一档),没有就从极小起
               Amp : Long_Float := Long_Float'Max (Start_Amp, Long_Float'Max (4.0 * Noise, (if K < 3 then Last_Trans else Last_Rot) * 0.5));
               Accepted : Boolean := False;
               Jaw0 : Floats;
            begin
               Jaw0.Append (Jaw_Of (F, A));
               for Try in 0 .. Max_Doublings loop
                  declare
                     A_Cmd : Table.Vec := Table.Zero_Vec;
                     Deliv, Back : Table.Vec;
                     Frames : Natural;
                     Ok2 : Boolean;
                     Frames_Back : Natural;
                     Got : Long_Float;
                     F1 : Plug.Cam_Vectors.Vector;
                     Visible : Boolean := False;
                  begin
                     A_Cmd (K) := Amp;
                     Go (L, M, A, Chan.Compose (P0, A_Cmd), Jaw0, F, Deliv, Frames, Ok2);
                     if not Ok2 then
                        return;
                     end if;
                     M.Settle := Natural'Max (M.Settle, Natural'Min (Frames, 6));
                     Got := Deliv (K);
                     F1 := F.Cams;
                     Go (L, M, A, P0, Jaw0, F, Back, Frames_Back, Ok2);
                     if not Ok2 then
                        return;
                     end if;
                     for C in 0 .. N_Cams - 1 loop
                        declare
                           Fl : Picture.Floor_Map renames M.Floors (C);
                           M1 : constant Bools := Picture.Moved (F0 (C).Gray, F1 (C).Gray, Fl);
                           M2 : constant Bools := Picture.Moved (F1 (C).Gray, F.Cams (C).Gray, Fl);
                           Here : constant Bools := Picture.Both (M1, M2);
                           Cw : constant Natural := F.Cams (C).W;
                           Ch2 : constant Natural := F.Cams (C).H;
                           Comps : constant Picture.Regions := Picture.Components (Here, Cw, Ch2, Picture.Min_Pixels (Cw, Ch2));
                           Fr : constant Long_Float := Picture.Fraction (Picture.Either (M1, M2));
                        begin
                           if not Comps.Is_Empty then
                              Visible := True;
                              declare
                                 Pt : Part;
                                 Big : constant Picture.Region := Comps (0);
                              begin
                                 Pt.Valid := True;
                                 Pt.X0 := Big.X0; Pt.Y0 := Big.Y0; Pt.X1 := Big.X1; Pt.Y1 := Big.Y1;
                                 Pt.Cu := Big.Cu; Pt.Cv := Big.Cv; Pt.Count := Big.Count;
                                 Pt.Frac := Long_Float (Big.Count) / Long_Float (Cw * Ch2);
                                 if abs Got >= Amp * 0.5 then
                                    M.Parts.Replace_Element (Ch * N_Cams + C, Pt);
                                 end if;
                              end;
                           end if;
                           if abs Got >= Amp * 0.5 then
                              M.Cam_Frac.Replace_Element (A * N_Cams + C, Long_Float'Max (M.Cam_Frac (A * N_Cams + C), Fr));
                           end if;
                        end;
                     end loop;
                     if abs Got >= Amp * 0.5 and then Visible then
                        Accepted := True;
                        M.Amp.Replace_Element (Ch, Amp);
                        M.Delivered.Replace_Element (Ch, Got);
                        M.Seen.Replace_Element (Ch, True);
                        if K < 3 then
                           Last_Trans := Amp;
                        else
                           Last_Rot := Amp;
                        end if;
                        Put_Line ("[身]   通道" & Natural'Image (Ch) & "(第" & Natural'Image (A + 1) & " 只手第" & Natural'Image (K) &
                                  " 轴):命令 " & Codec.Fmt (Amp, 4) & " 实到 " & Codec.Fmt (Got, 4) & " · " & Natural'Image (Frames) & " 拍稳 · 画面里看见了");
                        exit;
                     end if;
                     M.Amp.Replace_Element (Ch, Amp);
                     M.Delivered.Replace_Element (Ch, Got);
                     Amp := Amp * 2.0;
                  end;
               end loop;
               if not Accepted then
                  Put_Line ("[身]   通道" & Natural'Image (Ch) & ":探到 " & Codec.Fmt (M.Amp (Ch), 4) & " 仍走不出来或看不见(实到 " & Codec.Fmt (M.Delivered (Ch), 4) & ")");
               end if;
            end;
         end loop;
      end loop;
      end;
      --  ③ 哪台相机长在哪只手上:这只手一动它整幅都变,而且比第二名多一倍(倍数,无量纲);世界相机 = 变得最少的。
      --  开机前半段已经认过(Eyes 不空)⇒ 照用
      if not Eyes.Is_Empty then
         for A in 0 .. Natural'Min (Arms, Natural (Eyes.Length)) - 1 loop
            M.Cam_On_Arm.Replace_Element (A, Eyes (A));
         end loop;
         M.World_Cam := (if World >= 0 then Natural (World) else 0);
         Ok := True;
         return;
      end if;
      for A in 0 .. Arms - 1 loop
         declare
            Best : Integer := -1;
            Bv, Second : Long_Float := 0.0;
         begin
            for C in 0 .. N_Cams - 1 loop
               declare
                  V : constant Long_Float := M.Cam_Frac (A * N_Cams + C);
               begin
                  if V > Bv then
                     Second := Bv; Bv := V; Best := C;
                  elsif V > Second then
                     Second := V;
                  end if;
               end;
            end loop;
            if Best >= 0 and then Bv > 0.0 and then Bv >= 2.0 * Second then
               M.Cam_On_Arm.Replace_Element (A, Best);
               Put_Line ("[身] 第" & Natural'Image (A + 1) & " 只手一动,第" & Integer'Image (Best) & " 台相机变了 " &
                         Codec.Fmt (Bv * 100.0, 0) & "% 的画面 ⇒ 它长在这只手上");
            end if;
         end;
      end loop;
      declare
         Best : Natural := 0;
         Bv : Long_Float := 1.0e9;
      begin
         for C in 0 .. N_Cams - 1 loop
            declare
               Mx : Long_Float := 0.0;
            begin
               for A in 0 .. Arms - 1 loop
                  Mx := Long_Float'Max (Mx, M.Cam_Frac (A * N_Cams + C));
               end loop;
               if Mx < Bv then
                  Bv := Mx; Best := C;
               end if;
            end;
         end loop;
         M.World_Cam := Best;
         Put_Line ("[身] 世界相机 = 第" & Natural'Image (Best) & " 台(手动时它变得最少)");
      end;
      Ok := True;
   end Measure;
end Selfmap;
