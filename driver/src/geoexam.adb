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
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
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
      Put_Line ("用法:geoexam wrist geo_camK_obs.txt | geoexam head head_obs.txt [geo_cam1_obs.txt geo_cam2_obs.txt …]");
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
                     Obs.Append (Geom.Obs_Pt'(Pt => Natural (Num (L, 1)), Pose => Pose_Of (L, 4), U => Num (L, 2), V => Num (L, 3), Seq => 0));
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
                  elsif Field (L, 1) = "tip" then
                     --  炮里这只眼没量好 ⇒ 视线是零向量;后面的参数给了这只眼的观测文件(geo_camK_obs.txt)就在这儿离线解它、重建视线
                     declare
                        A : constant Natural := Natural (Num (L, 2));
                        Kc : constant Integer := Integer (Num (L, 3));
                        Tu : constant Long_Float := Num (L, 4);
                        Tv : constant Long_Float := Num (L, 5);
                     begin
                        if Kc >= 0 and then Tu >= 0.0 and then Geom.Norm (Ray_D (A)) = 0.0 then
                           for K in 3 .. Ada.Command_Line.Argument_Count loop
                              declare
                                 Wf : constant String := Ada.Command_Line.Argument (K);
                                 Tag : constant String := "geo_cam" & Codec.Img (Kc) & "_obs";
                              begin
                                 if Ada.Strings.Fixed.Index (Wf, Tag) > 0 then
                                    declare
                                       Wi : File_Type;
                                       Gw : Geom.Cam_Geo;
                                       Wobs : Geom.Obs_Pt_Vectors.Vector;
                                       Wok : Boolean;
                                       Wused : Natural;
                                    begin
                                       Open (Wi, In_File, Wf);
                                       declare
                                          Wh : constant String := Get_Line (Wi);
                                          Npts : constant Natural := Natural (Num (Wh, 6));
                                       begin
                                          Gw.Cx := Num (Wh, 3); Gw.Cy := Num (Wh, 4); Gw.F := Num (Wh, 5);
                                          while not End_Of_File (Wi) loop
                                             declare
                                                Wl : constant String := Get_Line (Wi);
                                             begin
                                                if Ada.Strings.Fixed.Trim (Wl, Ada.Strings.Both) /= "" then
                                                   Wobs.Append (Geom.Obs_Pt'(Pt => Natural (Num (Wl, 1)), Pose => Pose_Of (Wl, 4), U => Num (Wl, 2), V => Num (Wl, 3), Seq => 0));
                                                end if;
                                             end;
                                          end loop;
                                          Close (Wi);
                                          Geom.Fit_Rig (Gw, Wobs, Npts, Wok, Wused);
                                       end;
                                       if Wok then
                                          declare
                                             Dc : Geom.V3 := [(Tu - Gw.Cx) / Gw.F, -(Tv - Gw.Cy) / Gw.F, -1.0];
                                             Nn : constant Long_Float := Geom.Norm (Dc);
                                          begin
                                             for I in 0 .. 2 loop
                                                Dc (I) := Dc (I) / Nn;
                                             end loop;
                                             Ray_D.Replace_Element (A, Geom.Ap (Gw.R_Ce, Dc));
                                             Ray_O.Replace_Element (A, Gw.Off);
                                             Put_Line ("  臂 " & Codec.Img (A) & " 的眼(第 " & Codec.Img (Kc) & " 台)离线解了:焦距 " & Codec.Fmt (Gw.F, 1) & " ± " & Codec.Fmt (Gw.F_Sd, 1)
                                                       & ",视线按指尖像素 (" & Codec.Fmt (Tu, 0) & "," & Codec.Fmt (Tv, 0) & ") 重建");
                                          end;
                                       else
                                          Put_Line ("  臂 " & Codec.Img (A) & " 的眼离线也解不出:" & To_String (Geom.Why));
                                       end if;
                                    end;
                                 end if;
                              end;
                           end loop;
                        end if;
                     end;
                  elsif Field (L, 1) = "obs" then
                     Obs.Append (Geom.Obs_Pt'(Pt => Natural (Num (L, 2)), Pose => Pose_Of (L, 5), U => Num (L, 3), V => Num (L, 4),
                                              Seq => (if Field (L, 12) /= "" then Natural (Num (L, 12)) else 0)));
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
               --  每一笔:解出来的指尖投回不动的眼,和记下的像素差多少(哪一笔坏了一眼看出来;G1S 2026-09-25:5 笔落在空桌面上的标记把解拖到 0.5 m 外)
               for Ob of Obs loop
                  if Ob.Pt < Natural (Tip_H.Length) then
                     declare
                        Th : constant Geom.V3 := Tip_H (Ob.Pt);
                        Tw : constant Geom.V3 := Geom.Ap (Geom.Quat_To_R (Ob.Pose), Th);
                        Pw : constant Geom.V3 := [Ob.Pose (0) + Tw (0), Ob.Pose (1) + Tw (1), Ob.Pose (2) + Tw (2)];
                        U, V : Long_Float;
                        Front : Boolean;
                     begin
                        Geom.Project_Fixed (G, Pw, U, V, Front);
                        Put_Line ("    臂 " & Codec.Img (Ob.Pt) & " 帧 " & Codec.Img (Ob.Seq) & " 记 (" & Codec.Fmt (Ob.U, 1) & "," & Codec.Fmt (Ob.V, 1) & ") 投 ("
                                  & Codec.Fmt (U, 1) & "," & Codec.Fmt (V, 1) & ") 差 " & Codec.Fmt (Sqrt ((U - Ob.U) ** 2 + (V - Ob.V) ** 2), 1) & " px"
                                  & (if Front then "" else " 在相机后面"));
                     end;
                  end if;
               end loop;
            else
               Put_Line ("解不出来:" & To_String (Geom.Why));
            end if;
         end;
      end if;
   end;
   Close (Fi);
end Geoexam;
