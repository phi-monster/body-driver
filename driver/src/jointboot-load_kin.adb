with Links;
separate (Jointboot)
procedure Load_Kin (Path : String; K : out Kin_Store; Ok : out Boolean; Note : out Unbounded_String) is
   use Ada.Text_IO;
   Fi : File_Type;
   function Fields (S : String) return Strs is
      R : Strs;
      I : Natural := S'First;
   begin
      while I <= S'Last loop
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
            R.Append (S (I .. J - 1));
            I := J;
         end;
      end loop;
      return R;
   end Fields;
   function V (T : Strs; I : Natural) return Long_Float is (Long_Float'Value (T (I)));
   function M3_At (T : Strs; I : Natural) return Geom.M3 is
     ([[V (T, I), V (T, I + 1), V (T, I + 2)], [V (T, I + 3), V (T, I + 4), V (T, I + 5)], [V (T, I + 6), V (T, I + 7), V (T, I + 8)]]);
   function V3_At (T : Strs; I : Natural) return Geom.V3 is ([V (T, I), V (T, I + 1), V (T, I + 2)]);
   function Arm_Of (T : Strs) return Natural is (Natural'Value (T (1)));
   Version_Ok : Boolean := False;
   Pts : Links.Link_Pt_Vectors.Vector;   --  每一节的表面点(读完交给 Links;放进世界的那一份装回核对过以后由开机给)
begin
   K := (others => <>); Ok := False; Note := Null_Unbounded_String;
   Open (Fi, In_File, Path);
   while not End_Of_File (Fi) loop
      declare
         T : constant Strs := Fields (Get_Line (Fi));
         Tag : constant String := (if T.Is_Empty then "" else T (0));
      begin
         if Tag = "kin" then
            Version_Ok := Natural (T.Length) >= 2 and then T (1) = "5";
         elsif Tag = "key" and then Natural (T.Length) >= 2 then
            K.Key := To_Unbounded_String (T (1));
         elsif Tag = "world_cam" then
            K.World_Cam := Integer'Value (T (1));
         elsif Tag = "rw" then
            K.Rw := M3_At (T, 1);
         elsif Tag = "o" then
            K.O := V3_At (T, 1);
         elsif Tag = "plane" then
            K.Plane_Pt := V3_At (T, 1); K.Plane_N := V3_At (T, 4); K.Plane_Rms := V (T, 7);
         elsif Tag = "fixed" and then Natural (T.Length) < 41 then   --  整份相机几何 = 标签 + 40 个字段(格式);少了 = 不是这一版
            Version_Ok := False;
         elsif Tag = "fixed" then
            K.Fixed_Eye := Geom.Cam_Geo'(Valid => T (1) = "1", F => V (T, 2), Cx => V (T, 3), Cy => V (T, 4), K1 => V (T, 5), K2 => V (T, 6), K1_Sd => V (T, 7),
                                        F_Meas => V (T, 8), F_Prior => V (T, 9), F_Prior_Sd => V (T, 10), R_Ce => M3_At (T, 11), Off => V3_At (T, 20),
                                        Rms => V (T, 23), F_Sd => V (T, 24), Rot_Sd => V (T, 25), Off_Sd => V (T, 26), Pos_Sd => V (T, 27),
                                        Dropped => Natural'Value (T (28)), Tip_Valid => T (29) = "1", Tip_Touch => T (30) = "1", Tip => V3_At (T, 31),
                                        Gap => V (T, 34), Stride => V (T, 35), Stride_Rot => V (T, 36), Fixed => T (37) = "1", Pos => V3_At (T, 38),
                                        Lobes => Geom.Lobe_Geo_Vectors.Empty_Vector, Tip_Sd => 0.0);   --  不动的眼没有手指:这两样恒为空
         elsif Tag = "arm" then
            declare
               A : constant Natural := Arm_Of (T);
               W : Arm_World;
               D : Sweep_Data;
            begin
               while Natural (K.Worlds.Length) <= A loop
                  K.Worlds.Append (Arm_World'(others => <>)); K.Ds.Append (Sweep_Data'(others => <>)); K.Eyes.Append (-1);
               end loop;
               W := K.Worlds (A); D := K.Ds (A);
               W.Group := Natural'Value (T (2));
               K.Eyes.Replace_Element (A, Integer'Value (T (3)));
               W.Valid := T (4) = "1";
               W.Model.N := Natural'Value (T (5));
               W.Model.F := V (T, 6); W.Model.Cx := V (T, 7); W.Model.Cy := V (T, 8); W.S := V (T, 9);
               W.Model.Valid := W.Valid; W.Sweep := A;
               D.W := Natural'Value (T (10)); D.H := Natural'Value (T (11));
               W.Eye_W := D.W;
               K.Worlds.Replace_Element (A, W); K.Ds.Replace_Element (A, D);
            end;
         elsif Tag = "q0" or else Tag = "lo" or else Tag = "hi" or else Tag = "axis" or else Tag = "ra" or else Tag = "ta" or else Tag = "frame"
           or else Tag = "glo" or else Tag = "ghi" or else Tag = "slo" or else Tag = "shi"
         then
            declare
               A : constant Natural := Arm_Of (T);
               W : Arm_World := K.Worlds (A);
               D : Sweep_Data := K.Ds (A);
            begin
               if Tag = "q0" then
                  W.Model.Q0.Clear;
                  for I in 2 .. Natural (T.Length) - 1 loop
                     W.Model.Q0.Append (V (T, I));
                  end loop;
               elsif Tag = "glo" or else Tag = "ghi" or else Tag = "slo" or else Tag = "shi" then
                  declare
                     Lst : Floats;
                  begin
                     for I in 2 .. Natural (T.Length) - 1 loop
                        Lst.Append (V (T, I));
                     end loop;
                     if Tag = "glo" then
                        W.Got_Lo := Lst;
                     elsif Tag = "ghi" then
                        W.Got_Hi := Lst;
                     elsif Tag = "slo" then
                        W.Step_Lo := Lst;
                     else
                        W.Step_Hi := Lst;
                     end if;
                  end;
               elsif Tag = "lo" or else Tag = "hi" then
                  declare
                     Lst : Floats;
                  begin
                     for I in 2 .. Natural (T.Length) - 1 loop
                        Lst.Append (if T (I) = "none" then (if Tag = "lo" then Long_Float'First else Long_Float'Last) else V (T, I));
                     end loop;
                     if Tag = "lo" then
                        W.Lo := Lst;
                     else
                        W.Hi := Lst;
                     end if;
                  end;
               elsif Tag = "axis" then
                  declare
                     J : constant Natural := Natural'Value (T (2));
                  begin
                     W.Model.Ax (J).W := V3_At (T, 3); W.Model.Ax (J).P := V3_At (T, 6);
                     if Natural (T.Length) < 10 or else (T (9) /= "turn" and then T (9) /= "slide") then   --  axis 臂 轴 W P 转/走(格式)
                        Version_Ok := False;
                     else
                        W.Model.Ax (J).Slide := T (9) = "slide";
                     end if;
                  end;
               elsif Tag = "ra" then
                  W.Ra := M3_At (T, 2);
               elsif Tag = "ta" then
                  W.Ta := V3_At (T, 2);
               else
                  declare
                     Fr : Kinem.Frame_Info;
                  begin
                     Fr.Joint := Integer'Value (T (2));
                     for I in 3 .. Natural (T.Length) - 1 loop
                        Fr.Q.Append (V (T, I));
                     end loop;
                     D.Frames.Append (Fr);
                  end;
               end if;
               K.Worlds.Replace_Element (A, W); K.Ds.Replace_Element (A, D);
            end;
         elsif Tag = "board" then
            K.Board.Append (Geom.Scene_Pt'(Pw => V3_At (T, 1), Cov => M3_At (T, 4), U => V (T, 13), V => V (T, 14), Sh => V (T, 15),
                                           Views => Natural'Value (T (16))));
         elsif Tag = "link" then
            declare
               P : Links.Link_Pt;
            begin
               if Links.From_Fields (T, 1, P) then
                  Pts.Append (P);
               else
                  Version_Ok := False;   --  一行读不全 = 不是这一版
               end if;
            end;
         end if;
      end;
   end loop;
   Close (Fi);
   if not Version_Ok then
      Note := To_Unbounded_String ("格式是旧版(" & Path & ";存的量不全:kin 1 没存不动的眼的像素残差,kin 2 没存每根轴是转是走,kin 3 没存关节到过的范围,"
                                   & "kin 4 没存每一节的表面点)");
      return;
   end if;
   if K.Worlds.Is_Empty or else Length (K.Key) = 0 then
      Note := To_Unbounded_String ("文件不全(" & Path & ")");
      return;
   end if;
   --  核对用的图
   for A in 0 .. Natural (K.Worlds.Length) - 1 loop
      declare
         D : Sweep_Data := K.Ds (A);
         Im : Plug.Cam;
         Okb : Boolean := False;
      begin
         if Ada.Directories.Exists (Path & "_arm" & Codec.Img (A) & ".bmp") then
            Codec.Read_BMP (Path & "_arm" & Codec.Img (A) & ".bmp", Im.RGB, Im.W, Im.H, Okb);
         end if;
         if not Okb then
            Note := To_Unbounded_String ("第" & Codec.Img (A + 1) & " 只手核对用的图读不了");
            return;
         end if;
         D.Imgs.Append (Im);
         K.Ds.Replace_Element (A, D);
      end;
   end loop;
   if K.World_Cam >= 0 then
      declare
         D : Sweep_Data := K.Ds (0);
         Okb : Boolean := False;
      begin
         if Ada.Directories.Exists (Path & "_world.bmp") then
            Codec.Read_BMP (Path & "_world.bmp", D.World_Img.RGB, D.World_Img.W, D.World_Img.H, Okb);
         end if;
         if not Okb then
            Note := To_Unbounded_String ("不动的眼核对用的图读不了");
            return;
         end if;
         K.Ds.Replace_Element (0, D);
      end;
   end if;
   Ok := True;
   Links.Set_Points (Pts);
   Note := To_Unbounded_String (Codec.Img (Natural (K.Worlds.Length)) & " 只手的运动学和世界、不动的眼(第" & Integer'Image (K.World_Cam) & " 台)、板 "
                                & Codec.Img (Natural (K.Board.Length)) & " 个点、每一节的表面点 " & Codec.Img (Natural (Pts.Length)) & " 个");
exception
   when others =>
      if Is_Open (Fi) then
         Close (Fi);
      end if;
      Ok := False;
      Note := To_Unbounded_String ("文件读不了(" & Path & ")");
end Load_Kin;
