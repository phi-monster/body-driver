--  离线重解开机标定(2026-09-25):拿炮里落盘的观测(BL_DUMP 下的 geo_cam<k>_obs.txt / head_obs.txt)原样跑 Fit_Rig / Fit_Fixed_Rig,
--  打出焦距、朝向、偏移和各自的 ±、残差、踢掉几笔、没解出来时的原因。改拟合不用再开一小时的炮。
--  用法:geoexam wrist geo_cam1_obs.txt | geoexam head head_obs.txt
with Ada.Command_Line;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Strings.Fixed;
with Geom;
with Plug;
with Codec;
procedure Geoexam is
   package V3s renames Geom.V3_Vectors;
   --  按空格切一行
   function Field (S : String; K : Positive) return String is
      I : Natural := S'First;
      N : Natural := 0;
   begin
      loop
         while I <= S'Last and then S (I) = ' ' loop
            I := I + 1;
         end loop;
         exit when I > S'Last;
         declare
            J : Natural := I;
         begin
            while J <= S'Last and then S (J) /= ' ' loop
               J := J + 1;
            end loop;
            N := N + 1;
            if N = K then
               return S (I .. J - 1);
            end if;
            I := J;
         end;
      end loop;
      return "";
   end Field;
   function Num (S : String; K : Positive) return Long_Float is (Long_Float'Value (Field (S, K)));
   function Pose_Of (S : String; From : Positive) return Plug.Arm_Pose is
      P : Plug.Arm_Pose;
   begin
      for I in 0 .. 6 loop
         P (I) := Num (S, From + I);
      end loop;
      return P;
   end Pose_Of;
   Mode : constant String := (if Ada.Command_Line.Argument_Count >= 1 then Ada.Command_Line.Argument (1) else "");
   Path : constant String := (if Ada.Command_Line.Argument_Count >= 2 then Ada.Command_Line.Argument (2) else "");
   Fi : File_Type;
   G : Geom.Cam_Geo;
begin
   if Mode = "" or else Path = "" then
      Put_Line ("用法:geoexam wrist geo_camK_obs.txt | geoexam head head_obs.txt");
      return;
   end if;
   Open (Fi, In_File, Path);
   declare
      Head : constant String := Get_Line (Fi);
   begin
      G.Cx := Num (Head, 3); G.Cy := Num (Head, 4); G.F := Num (Head, 5);
      Put_Line ("画幅 " & Field (Head, 1) & "x" & Field (Head, 2) & " · 主点 (" & Codec.Fmt (G.Cx, 1) & "," & Codec.Fmt (G.Cy, 1) & ") · 焦距"
                & (if G.F > 0.0 then "给的 " & Codec.Fmt (G.F, 1) else "没给,一起解"));
      if Mode = "wrist" then
         declare
            Obs : Geom.Obs_Pt_Vectors.Vector;
            N_Pts : constant Natural := Natural (Num (Head, 6));
            Ok : Boolean;
            Used : Natural;
         begin
            while not End_Of_File (Fi) loop
               declare
                  L : constant String := Get_Line (Fi);
               begin
                  if Ada.Strings.Fixed.Trim (L, Ada.Strings.Both) /= "" then
                     Obs.Append (Geom.Obs_Pt'(Pt => Natural (Num (L, 1)), Pose => Pose_Of (L, 4), U => Num (L, 2), V => Num (L, 3)));
                  end if;
               end;
            end loop;
            Put_Line (Codec.Img (Natural (Obs.Length)) & " 笔观测 · " & Codec.Img (N_Pts) & " 个候选");
            Geom.Fit_Rig (G, Obs, N_Pts, Ok, Used);
            if Ok then
               Put_Line ("解出来:" & Codec.Img (Used) & " 点进了解,踢掉 " & Codec.Img (G.Dropped) & " 笔 · 残差 " & Codec.Fmt (G.Rms, 2) & " px · 焦距 "
                         & Codec.Fmt (G.F, 1) & " ± " & Codec.Fmt (G.F_Sd, 1) & " px · 朝向 ± " & Codec.Fmt (G.Rot_Sd, 4) & " rad · 偏移 (" & Codec.Fmt (G.Off (0), 3) & ","
                         & Codec.Fmt (G.Off (1), 3) & "," & Codec.Fmt (G.Off (2), 3) & ") ± " & Codec.Fmt (G.Off_Sd, 3) & " m");
            else
               Put_Line ("解不出来:" & To_String (Geom.Why));
            end if;
         end;
      else
         declare
            Obs : Geom.Obs_Pt_Vectors.Vector;
            Ray_O, Ray_D, Tip_H : V3s.Vector;
            Arms : constant Natural := Natural (Num (Head, 6));
            Ok : Boolean;
         begin
            for A in 0 .. Arms - 1 loop
               Ray_O.Append (Geom.V3'[0.0, 0.0, 0.0]); Ray_D.Append (Geom.V3'[0.0, 0.0, 0.0]);
            end loop;
            while not End_Of_File (Fi) loop
               declare
                  L : constant String := Get_Line (Fi);
               begin
                  if Field (L, 1) = "ray" then
                     declare
                        A : constant Natural := Natural (Num (L, 2));
                     begin
                        Ray_O.Replace_Element (A, Geom.V3'[Num (L, 3), Num (L, 4), Num (L, 5)]);
                        Ray_D.Replace_Element (A, Geom.V3'[Num (L, 6), Num (L, 7), Num (L, 8)]);
                     end;
                  elsif Field (L, 1) = "obs" then
                     Obs.Append (Geom.Obs_Pt'(Pt => Natural (Num (L, 2)), Pose => Pose_Of (L, 5), U => Num (L, 3), V => Num (L, 4)));
                  end if;
               end;
            end loop;
            Put_Line (Codec.Img (Natural (Obs.Length)) & " 笔指尖观测 · " & Codec.Img (Arms) & " 条臂");
            for A in 0 .. Arms - 1 loop
               Put_Line ("  臂 " & Codec.Img (A) & " 视线起点 (" & Codec.Fmt (Ray_O (A) (0), 3) & "," & Codec.Fmt (Ray_O (A) (1), 3) & "," & Codec.Fmt (Ray_O (A) (2), 3)
                         & ") 方向 (" & Codec.Fmt (Ray_D (A) (0), 3) & "," & Codec.Fmt (Ray_D (A) (1), 3) & "," & Codec.Fmt (Ray_D (A) (2), 3) & ")");
            end loop;
            Geom.Fit_Fixed_Rig (G, Obs, Ray_O, Ray_D, Tip_H, Ok);
            if Ok then
               Put_Line ("解出来:踢掉 " & Codec.Img (G.Dropped) & " 笔 · 残差 " & Codec.Fmt (G.Rms, 2) & " px · 它在 (" & Codec.Fmt (G.Pos (0), 3) & "," & Codec.Fmt (G.Pos (1), 3) & ","
                         & Codec.Fmt (G.Pos (2), 3) & ") ± " & Codec.Fmt (G.Pos_Sd, 3) & " m · 焦距 " & Codec.Fmt (G.F, 1) & " ± " & Codec.Fmt (G.F_Sd, 1) & " px · 朝向 ± "
                         & Codec.Fmt (G.Rot_Sd, 4) & " rad");
               for A in 0 .. Natural (Tip_H.Length) - 1 loop
                  Put_Line ("  臂 " & Codec.Img (A) & " 指尖(手系)(" & Codec.Fmt (Tip_H (A) (0), 3) & "," & Codec.Fmt (Tip_H (A) (1), 3) & "," & Codec.Fmt (Tip_H (A) (2), 3) & ")");
               end loop;
            else
               Put_Line ("解不出来:" & To_String (Geom.Why));
            end if;
         end;
      end if;
   end;
   Close (Fi);
end Geoexam;
