with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Codec;
with Ada.Containers;
package body Zone is
   function Is_Self (Z : Hand_Zone; R : Picture.Region; W, Hh : Natural) return Boolean is
      function In_Box (X0, Y0, X1, Y1 : Natural) return Boolean is
         Gw : constant Natural := (X1 - X0) / 2;
         Gh : constant Natural := (Y1 - Y0) / 2;
         Cx : constant Natural := Natural (R.Cu * Long_Float (W));
         Cy : constant Natural := Natural (R.Cv * Long_Float (Hh));
      begin
         return Cx + Gw >= X0 and then Cx <= X1 + Gw and then Cy + Gh >= Y0 and then Cy <= Y1 + Gh;
      end In_Box;
   begin
      if not Z.Valid then
         return False;
      end if;
      if Z.A.Valid and then In_Box (Z.A.X0, Z.A.Y0, Z.A.X1, Z.A.Y1) then
         return True;
      end if;
      if Z.B.Valid and then In_Box (Z.B.X0, Z.B.Y0, Z.B.X1, Z.B.Y1) then
         return True;
      end if;
      return False;
   end Is_Self;

   procedure Measure (L : in out Plug.Link; M : Selfmap.Body_Map; Arm : Natural; F : in out Plug.Frame; H : out Hand; Ok : out Boolean) is
      N_Cams : constant Natural := Natural (F.Cams.Length);
      F0 : Plug.Cam_Vectors.Vector;
      Swept : array (0 .. Natural'Max (0, N_Cams - 1)) of Bools;
      J0 : constant Long_Float := Selfmap.Jaw_Of (F, Arm);
      Pose : constant Plug.Arm_Pose := (if Arm < Natural (F.EE.Length) then F.EE (Arm) else [others => 0.0]);
      Prev_J : Long_Float := J0;
      Prev_Cams : Plug.Cam_Vectors.Vector;
      Still : Natural := 0;
      Target : Floats;
      Closed_Frame : Plug.Cam_Vectors.Vector;
   begin
      H := (others => <>);
      H.Arm := Arm;
      H.Open_Reading := J0;
      Ok := False;
      if N_Cams = 0 or else Arm >= Natural (F.EE.Length) then
         return;
      end if;
      for C in 0 .. N_Cams - 1 loop
         Swept (C) := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (F.Cams (C).W * F.Cams (C).H));
      end loop;
      Target.Append (0.0);
      --  抓握读数只是命令的回声(这台机器如此;真机也未必是关节)⇒ "合到停住"只认画面:每台相机连着两拍不变
      declare
         Used : Natural;
         Ok2 : Boolean;
      begin
         Selfmap.Wait_Still (L, M, F, 30, Used, Ok2);
         if not Ok2 then
            return;
         end if;
         Put_Line ("[身] 第" & Natural'Image (Arm + 1) & " 只手合空一次(先等画面静止:" & Natural'Image (Used) & " 拍;读数从 " & Codec.Fmt (J0, 3) & " 起)…");
      end;
      F0 := F.Cams;
      Prev_Cams := F.Cams;
      for Step in 1 .. 40 loop
         declare
            C : Plug.Cmd;
         begin
            C.Kind := Plug.Ee; C.Arm := Arm; C.Pose := Pose; C.Jaw := Target;
            if not Plug.Act (L, C) or else not Plug.Sense (L, F) then
               return;
            end if;
         end;
         H.Close_Steps := Step;
         for C in 0 .. N_Cams - 1 loop
            declare
               Mv : constant Bools := Picture.Moved (F0 (C).Gray, F.Cams (C).Gray, M.Floors (C));
            begin
               Swept (C) := Picture.Either (Swept (C), Mv);
            end;
         end loop;
         declare
            J : constant Long_Float := Selfmap.Jaw_Of (F, Arm);
         begin
            if abs (J - Prev_J) <= M.Jaw_Noise and then Selfmap.Pictures_Still (M, Prev_Cams, F.Cams) then
               Still := Still + 1;
            else
               Still := 0;
            end if;
            Prev_J := J;
            Prev_Cams := F.Cams;
         end;
         exit when Still >= 2 and then Step >= 3;
      end loop;
      H.Empty_Close := Selfmap.Jaw_Of (F, Arm);
      Closed_Frame := F.Cams;
      Put_Line ("[身]   合到停住:读数 " & Codec.Fmt (H.Empty_Close, 3) & "(" & Natural'Image (H.Close_Steps) & " 拍)");
      --  张回去
      Target.Replace_Element (0, J0);
      Still := 0;
      Prev_Cams := F.Cams;
      for Step in 1 .. 40 loop
         declare
            C : Plug.Cmd;
         begin
            C.Kind := Plug.Ee; C.Arm := Arm; C.Pose := Pose; C.Jaw := Target;
            if not Plug.Act (L, C) or else not Plug.Sense (L, F) then
               return;
            end if;
         end;
         if abs (Selfmap.Jaw_Of (F, Arm) - J0) <= Long_Float'Max (M.Jaw_Noise, 1.0e-3) and then Selfmap.Pictures_Still (M, Prev_Cams, F.Cams) then
            Still := Still + 1;
         else
            Still := 0;
         end if;
         Prev_Cams := F.Cams;
         exit when Still >= 2 and then Step >= 3;
      end loop;
      --  每台相机:扫过的像素 → 块 → 瓣
      for C in 0 .. N_Cams - 1 loop
         declare
            Cw : constant Natural := F.Cams (C).W;
            Ch : constant Natural := F.Cams (C).H;
            Comps : constant Picture.Regions := Picture.Components (Swept (C), Cw, Ch, Picture.Min_Pixels (Cw, Ch));
            Z : Hand_Zone;
            procedure Fill (Lb : in out Lobe; R : Picture.Region) is
            begin
               Lb.Valid := True; Lb.X0 := R.X0; Lb.Y0 := R.Y0; Lb.X1 := R.X1; Lb.Y1 := R.Y1;
               Lb.Cu := R.Cu; Lb.Cv := R.Cv; Lb.Count := R.Count;
            end Fill;
         begin
            Z.Fingers := Swept (C);
            --  落图自证:扫过的像素(白)+ 合拢后的灰度图 —— 手指认错了要一眼看得见
            if Codec.Env ("BL_DUMP") /= "" then
               declare
                  Mk : Buf := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (Cw * Ch));
               begin
                  for I in 0 .. Cw * Ch - 1 loop
                     if Swept (C).Element (I) then
                        Mk.Replace_Element (I, 255);
                     end if;
                  end loop;
                  Codec.Write_PGM (Codec.Env ("BL_DUMP") & "/zone_arm" & Codec.Img (Arm + 1) & "_cam" & Codec.Img (C) & "_swept.pgm", Mk, Cw, Ch);
                  Codec.Write_PGM (Codec.Env ("BL_DUMP") & "/zone_arm" & Codec.Img (Arm + 1) & "_cam" & Codec.Img (C) & "_closed.pgm", Closed_Frame (C).Gray, Cw, Ch);
                  Codec.Write_PGM (Codec.Env ("BL_DUMP") & "/zone_arm" & Codec.Img (Arm + 1) & "_cam" & Codec.Img (C) & "_open.pgm", F0 (C).Gray, Cw, Ch);
               end;
            end if;
            if not Comps.Is_Empty then
               Fill (Z.A, Comps (0));
               Z.N_Lobes := 1;
               --  第二瓣:至少有第一瓣四分之一的像素(比例,无量纲),且两瓣框不互相包含
               if Natural (Comps.Length) >= 2 and then Comps (1).Count * 4 >= Comps (0).Count then
                  declare
                     R0 : constant Picture.Region := Comps (0);
                     R1 : constant Picture.Region := Comps (1);
                     Overlap_X : constant Boolean := R1.X0 <= R0.X1 and then R0.X0 <= R1.X1;
                     Overlap_Y : constant Boolean := R1.Y0 <= R0.Y1 and then R0.Y0 <= R1.Y1;
                  begin
                     if not (Overlap_X and then Overlap_Y) then
                        Fill (Z.B, R1);
                        Z.N_Lobes := 2;
                     end if;
                  end;
               end if;
               if Z.N_Lobes = 2 then
                  declare
                     Du : constant Long_Float := Z.B.Cu - Z.A.Cu;
                     Dv : constant Long_Float := Z.B.Cv - Z.A.Cv;
                     Ln : constant Long_Float := Sqrt (Du * Du + Dv * Dv);
                  begin
                     Z.Cu := 0.5 * (Z.A.Cu + Z.B.Cu);
                     Z.Cv := 0.5 * (Z.A.Cv + Z.B.Cv);
                     Z.Span := Ln;
                     if Ln > 1.0e-9 then
                        Z.Au := Du / Ln; Z.Av := Dv / Ln;
                     end if;
                     if abs Du * Long_Float (Cw) >= abs Dv * Long_Float (Ch) then
                        Z.X0 := Natural'Min (Z.A.X1, Z.B.X1); Z.X1 := Natural'Max (Z.A.X0, Z.B.X0);
                        Z.Y0 := Natural'Min (Z.A.Y0, Z.B.Y0); Z.Y1 := Natural'Max (Z.A.Y1, Z.B.Y1);
                     else
                        Z.Y0 := Natural'Min (Z.A.Y1, Z.B.Y1); Z.Y1 := Natural'Max (Z.A.Y0, Z.B.Y0);
                        Z.X0 := Natural'Min (Z.A.X0, Z.B.X0); Z.X1 := Natural'Max (Z.A.X1, Z.B.X1);
                     end if;
                     if Z.X0 > Z.X1 then
                        declare
                           T : constant Natural := Z.X0;
                        begin
                           Z.X0 := Z.X1; Z.X1 := T;
                        end;
                     end if;
                     if Z.Y0 > Z.Y1 then
                        declare
                           T : constant Natural := Z.Y0;
                        begin
                           Z.Y0 := Z.Y1; Z.Y1 := T;
                        end;
                     end if;
                  end;
               else
                  Z.Cu := Z.A.Cu; Z.Cv := Z.A.Cv;
                  Z.Au := Comps (0).Au; Z.Av := Comps (0).Av;
                  Z.Span := Long_Float (Natural'Max (Z.A.X1 - Z.A.X0, Z.A.Y1 - Z.A.Y0)) / Long_Float (Cw);
                  Z.X0 := Z.A.X0; Z.Y0 := Z.A.Y0; Z.X1 := Z.A.X1; Z.Y1 := Z.A.Y1;
               end if;
               if Closed_Frame (C).Has_Depth then
                  Z.Depth := Picture.Region_Depth (Closed_Frame (C).Depth, Cw, Ch, Swept (C), 0.5);
               else
                  Z.Depth := Picture.Region_Depth (F.Cams (C).Depth, Cw, Ch, Swept (C), 0.5);
               end if;
               Z.Valid := True;
               Put_Line ("[身]   第" & Natural'Image (C) & " 台相机里:" & Natural'Image (Z.N_Lobes) & " 瓣 · 区心 (" &
                         Codec.Fmt (Z.Cu, 3) & "," & Codec.Fmt (Z.Cv, 3) & ") · 张幅 " & Codec.Fmt (Z.Span, 3) &
                         " 画幅 · 手指深 " & Codec.Fmt (Z.Depth, 3) & " m · 扫过 " & Codec.Fmt (Picture.Fraction (Swept (C)) * 100.0, 2) & "% 画面");
            else
               Put_Line ("[身]   第" & Natural'Image (C) & " 台相机里看不见这只手合拢");
            end if;
            H.Zones.Append (Z);
         end;
      end loop;
      Ok := True;
   end Measure;
end Zone;
