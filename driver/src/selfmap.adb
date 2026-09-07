with Ada.Text_IO; use Ada.Text_IO;
with Codec;
with Chan;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
package body Selfmap is
   Start_Amp : constant Long_Float := 1.0e-4;   --  探针协议的起点(极小,翻倍到走得出来又看得见为止;起点多小不影响结果),无量纲协议
   Max_Doublings : constant := 12;              --  次数,无量纲

   function Jaw_Index (F : Plug.Frame; Arm : Natural) return Natural is
     (if Natural (F.Jaw.Length) > Arm then Arm else 0);

   function Jaw_Of (F : Plug.Frame; Arm : Natural) return Long_Float is
     (if F.Jaw.Is_Empty then 1.0 else F.Jaw (Jaw_Index (F, Arm)));

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

   function Pictures_Still (M : Body_Map; Before, After : Plug.Cam_Vectors.Vector) return Boolean is
   begin
      for C in 0 .. Natural'Min (Natural (Before.Length), Natural (After.Length)) - 1 loop
         if C < Natural (M.Floors.Length) then
            declare
               --  静止 = 超过各自噪声地板的像素凑不成一团(最少像素数的几倍,倍数无量纲;去噪闪烁是撒开的单点)
               Mv : constant Bools := Picture.Moved (Before (C).Gray, After (C).Gray, M.Floors (C));
               Cnt : Natural := 0;
            begin
               for B of Mv loop
                  if B then
                     Cnt := Cnt + 1;
                  end if;
               end loop;
               if Cnt > 4 * Picture.Min_Pixels (Before (C).W, Before (C).H) then
                  return False;
               end if;
            end;
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
                 Watch : Watcher := null) is
      C : Plug.Cmd;
      P0 : constant Plug.Arm_Pose := (if Arm < Natural (F.EE.Length) then F.EE (Arm) else [others => 0.0]);
      Prev : Plug.Arm_Pose := P0;
      Still : Natural := 0;
      Send : Boolean := True;
      Halted : Boolean := False;
   begin
      Delivered := Table.Zero_Vec;
      Frames := 0;
      C.Kind := Plug.Ee; C.Arm := Arm; C.Pose := Target; C.Jaw := Jaw;
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

   procedure Measure (L : in out Plug.Link; F : in out Plug.Frame; M : out Body_Map; Ok : out Boolean) is
      N_Cams : constant Natural := Natural (F.Cams.Length);
      Arms : constant Natural := Natural (F.EE.Length);
   begin
      M := (others => <>);
      M.Arms := Arms; M.N_Cams := N_Cams; M.Per_Arm := Chan.Per_Arm; M.Channels := Arms * Chan.Per_Arm;
      Ok := False;
      if Arms = 0 or else N_Cams = 0 then
         Put_Line ("[身] 没有末端位姿或没有相机,量不了身体");
         return;
      end if;
      --  ① 什么都不做时读数抖多少、画面抖多少(静止对)
      declare
         Prev_EE : Plug.Pose_Vectors.Vector := F.EE;
         Prev_Jaw : Floats := F.Jaw;
         Prev_Gray : Plug.Cam_Vectors.Vector := F.Cams;
      begin
         for K in 1 .. 4 loop
            if not Plug.Sense (L, F) then
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
                  M.Jaw_Noise := Long_Float'Max (M.Jaw_Noise, abs (F.Jaw (J) - Prev_Jaw (J)));
               end if;
            end loop;
            Prev_EE := F.EE; Prev_Jaw := F.Jaw;
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
      --  ③ 哪台相机长在哪只手上:这只手一动它整幅都变,而且比第二名多一倍(倍数,无量纲);世界相机 = 变得最少的
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
