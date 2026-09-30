separate (Jointboot)
procedure Save_Kin (Path : String; K : Kin_Store; Images : Boolean := True) is
   use Ada.Text_IO;
   Fo : File_Type;
   procedure Put_M3 (M : Geom.M3) is
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            Put (Fo, " " & F9 (M (I, J)));
         end loop;
      end loop;
   end Put_M3;
   procedure Put_V3 (V : Geom.V3) is
   begin
      Put (Fo, " " & F9 (V (0)) & " " & F9 (V (1)) & " " & F9 (V (2)));
   end Put_V3;
begin
   Create (Fo, Out_File, Path);
   Put_Line (Fo, "kin 4");   --  格式版本:4 = 每个关节到过的范围和往外一步(到过的范围,09-29);3 = 每根轴记着是转还是走(09-27 无人机);2 = 不动的眼整份相机几何;更旧的读到 ⇒ 从零量
   Put_Line (Fo, "key " & To_String (K.Key));
   Put_Line (Fo, "world_cam " & Codec.Img (K.World_Cam));   --  09-30:原来 Integer'Image 在 −1 时写成 "world_cam-1",读回来标签认不出
   Put (Fo, "rw"); Put_M3 (K.Rw); New_Line (Fo);
   Put (Fo, "o"); Put_V3 (K.O); New_Line (Fo);
   Put (Fo, "plane"); Put_V3 (K.Plane_Pt); Put_V3 (K.Plane_N); Put_Line (Fo, " " & F9 (K.Plane_Rms));
   --  不动的眼:整份相机几何按记录的次序一个字段不落(读回是不带 others 的整份聚合,记录加了字段那边编译不过)。
   --  09-27 V1B35 / V1B38:原来只存焦距、主点、位置、朝向,没存像素残差 ⇒ 装回后每轮核对的门 = 3 × 0 px,板上一个点都对不上,核对瞎了还报"没挪、没挡"
   declare
      G : Geom.Cam_Geo renames K.Fixed_Eye;
      function B (X : Boolean) return String is (if X then "1" else "0");
   begin
      Put (Fo, "fixed " & B (G.Valid) & " " & F9 (G.F) & " " & F9 (G.Cx) & " " & F9 (G.Cy) & " " & F9 (G.K1) & " " & F9 (G.K2) & " " & F9 (G.K1_Sd)
           & " " & F9 (G.F_Meas) & " " & F9 (G.F_Prior) & " " & F9 (G.F_Prior_Sd));
      Put_M3 (G.R_Ce); Put_V3 (G.Off);
      Put (Fo, " " & F9 (G.Rms) & " " & F9 (G.F_Sd) & " " & F9 (G.Rot_Sd) & " " & F9 (G.Off_Sd) & " " & F9 (G.Pos_Sd) & " " & Codec.Img (G.Dropped)
           & " " & B (G.Tip_Valid) & " " & B (G.Tip_Touch));
      Put_V3 (G.Tip);
      Put (Fo, " " & F9 (G.Gap) & " " & F9 (G.Stride) & " " & F9 (G.Stride_Rot) & " " & B (G.Fixed));
      Put_V3 (G.Pos); New_Line (Fo);
   end;
   for A in 0 .. Natural (K.Worlds.Length) - 1 loop
      declare
         W : constant Arm_World := K.Worlds (A);
         D : constant Sweep_Data := K.Ds (A);
      begin
         Put_Line (Fo, "arm " & Codec.Img (A) & " " & Codec.Img (W.Group) & " " & Integer'Image (K.Eyes (A)) & " " & (if W.Valid then "1" else "0") & " "
                   & Codec.Img (W.Model.N) & " " & F9 (W.Model.F) & " " & F9 (W.Model.Cx) & " " & F9 (W.Model.Cy) & " " & F9 (W.S) & " "
                   & Codec.Img (D.W) & " " & Codec.Img (D.H));
         Put (Fo, "q0 " & Codec.Img (A));
         for X of W.Model.Q0 loop
            Put (Fo, " " & F9 (X));
         end loop;
         New_Line (Fo);
         for J in 0 .. W.Model.N - 1 loop
            Put (Fo, "axis " & Codec.Img (A) & " " & Codec.Img (J)); Put_V3 (W.Model.Ax (J).W); Put_V3 (W.Model.Ax (J).P);
            Put_Line (Fo, " " & Kind_Word (W.Model.Ax (J)));
         end loop;
         Put (Fo, "ra " & Codec.Img (A)); Put_M3 (W.Ra); New_Line (Fo);
         Put (Fo, "ta " & Codec.Img (A)); Put_V3 (W.Ta); New_Line (Fo);
         Put (Fo, "lo " & Codec.Img (A));
         for X of W.Lo loop
            Put (Fo, " " & Lim (X));
         end loop;
         New_Line (Fo);
         Put (Fo, "hi " & Codec.Img (A));
         for X of W.Hi loop
            Put (Fo, " " & Lim (X));
         end loop;
         New_Line (Fo);
         --  到过的范围:到过的范围(两头)、往外一步(两边)
         declare
            procedure Row (Tag : String; V : Floats) is
            begin
               Put (Fo, Tag & " " & Codec.Img (A));
               for X of V loop
                  Put (Fo, " " & F9 (X));
               end loop;
               New_Line (Fo);
            end Row;
         begin
            Row ("glo", W.Got_Lo); Row ("ghi", W.Got_Hi); Row ("slo", W.Step_Lo); Row ("shi", W.Step_Hi);
         end;
         for Fr of D.Frames loop
            Put (Fo, "frame " & Codec.Img (A) & " " & Integer'Image (Fr.Joint));
            for X of Fr.Q loop
               Put (Fo, " " & F9 (X));
            end loop;
            New_Line (Fo);
         end loop;
         if Images and then not D.Imgs.Is_Empty then
            Codec.Write_BMP (Path & "_arm" & Codec.Img (A) & ".bmp", D.Imgs (0).RGB, D.Imgs (0).W, D.Imgs (0).H);
         end if;
      end;
   end loop;
   for P of K.Board loop
      Put (Fo, "board"); Put_V3 (P.Pw); Put_M3 (P.Cov);
      Put_Line (Fo, " " & F9 (P.U) & " " & F9 (P.V) & " " & F9 (P.Sh) & " " & Codec.Img (P.Views));
   end loop;
   Close (Fo);
   if Images and then K.World_Cam >= 0 and then not K.Ds.Is_Empty and then K.Ds (0).World_Img.W > 0 then
      Codec.Write_BMP (Path & "_world.bmp", K.Ds (0).World_Img.RGB, K.Ds (0).World_Img.W, K.Ds (0).World_Img.H);
   end if;
end Save_Kin;
