--  离线解不动的眼(09-30 V1B69:开机"不长在手上的那只眼放不进世界",同一台机器、同样的配点数 V1B68 解出焦距 297.5):
--  读一炮落盘的 look/cam_obs.txt(每一笔:哪只手、它的第几格、第几个三角点、不动的眼里的像素、往返差、这个点在那只手自己系里的坐标和协方差),
--  第 0 只手的点就是世界系;别的手按 align_arm<k>.txt 的长度倍数、转动、平移搬进世界(同 Jointboot.To_World / Cov_World;
--  落盘的是一起精修以后的那一份,和当时配进来时差一点)。照 Jointboot.Fit_Cam 的样子搭板:配点噪声 = 往返差的中位、主点在画幅正中、焦距一起解,
--  跑驱动同一份 Geom.Fit_Fixed_Board,打出解没解出、进解几个点、残差、焦距、位置。
--  用法:fixedexam <cam_obs.txt> <画幅宽> <画幅高> [align_arm1.txt …];环境变量 FIT_ARM = k ⇒ 只拿第 k 只手的点解,再按解出来的眼把每只手的点投回去各报残差
--  (两只手各自在世界里对不对得上这只眼)
with Ada.Command_Line; use Ada.Command_Line;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Bytes; use Bytes;
with Codec;
with Geom; use Geom;
procedure Fixedexam is
   package Sorting is new F64_Vectors.Generic_Sorting;
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
   type Arm_World is record
      S : Long_Float := 1.0;
      Ra : M3 := [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
      Ta : V3 := [0.0, 0.0, 0.0];
   end record;
   package World_Vectors is new Ada.Containers.Vectors (Natural, Arm_World);
   Worlds : World_Vectors.Vector;
   Scene, All_Pts : Scene_Pt_Vectors.Vector;
   Arm_Of : Nat_Vectors.Vector;   --  All_Pts 里每个点是哪只手的
   Es : Floats;
   Fit_Arm : constant Integer := (if Codec.Env ("FIT_ARM") = "" then -1 else Integer'Value (Codec.Env ("FIT_ARM")));
   F : File_Type;
   W, H : Natural;
begin
   if Argument_Count < 3 then
      Put_Line ("用法:fixedexam <cam_obs.txt> <画幅宽> <画幅高> [align_arm1.txt …]");
      return;
   end if;
   W := Natural'Value (Argument (2));
   H := Natural'Value (Argument (3));
   Worlds.Append (Arm_World'(others => <>));
   for A in 4 .. Argument_Count loop
      Open (F, In_File, Argument (A));
      declare
         T : constant Strs := Fields (Get_Line (F));
         Wd : Arm_World;
      begin
         --  S s R r00 … r22 T t0 t1 t2
         Wd.S := Long_Float'Value (T (1));
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Wd.Ra (I, J) := Long_Float'Value (T (3 + 3 * I + J));
            end loop;
         end loop;
         for I in 0 .. 2 loop
            Wd.Ta (I) := Long_Float'Value (T (13 + I));
         end loop;
         Worlds.Append (Wd);
      end;
      Close (F);
   end loop;
   Open (F, In_File, Argument (1));
   while not End_Of_File (F) loop
      declare
         T : constant Strs := Fields (Get_Line (F));
         Pa : constant Natural := Natural'Value (T (0));
         X : constant V3 := [Long_Float'Value (T (6)), Long_Float'Value (T (7)), Long_Float'Value (T (8))];
         C : M3;
      begin
         C (0, 0) := Long_Float'Value (T (9)); C (0, 1) := Long_Float'Value (T (10)); C (0, 2) := Long_Float'Value (T (11));
         C (1, 1) := Long_Float'Value (T (12)); C (1, 2) := Long_Float'Value (T (13)); C (2, 2) := Long_Float'Value (T (14));
         C (1, 0) := C (0, 1); C (2, 0) := C (0, 2); C (2, 1) := C (1, 2);
         if Pa < Natural (Worlds.Length) then
            declare
               Wd : constant Arm_World := Worlds (Pa);
               Rt : constant V3 := Ap (Wd.Ra, X);
               Rc : constant M3 := Mul (Mul (Wd.Ra, C), Tr (Wd.Ra));
               Cw : M3;
            begin
               for I in 0 .. 2 loop
                  for J in 0 .. 2 loop
                     Cw (I, J) := Wd.S * Wd.S * Rc (I, J);
                  end loop;
               end loop;
               declare
                  Sp : constant Scene_Pt := (Pw => [Wd.S * Rt (0) + Wd.Ta (0), Wd.S * Rt (1) + Wd.Ta (1), Wd.S * Rt (2) + Wd.Ta (2)], Cov => Cw,
                                             U => Long_Float'Value (T (3)), V => Long_Float'Value (T (4)), Sh => 0.0, Views => 2);
               begin
                  All_Pts.Append (Sp);
                  Arm_Of.Append (Pa);
                  if Fit_Arm < 0 or else Pa = Natural (Fit_Arm) then
                     Scene.Append (Sp);
                     Es.Append (Long_Float'Value (T (5)));
                  end if;
               end;
            end;
         end if;
      end;
   end loop;
   Close (F);
   if Scene.Is_Empty then
      Put_Line ("板上一个点都没有");
      return;
   end if;
   declare
      Sorted : Floats := Es;
      Sh : Long_Float;
      G : Cam_Geo := No_Geo;
      Rp : Fixed_Report;
      Ok : Boolean;
   begin
      Sorting.Sort (Sorted);
      Sh := Sorted (Natural (Sorted.Length) / 2);   --  同 Jointboot.Fit_Cam:这只眼里配点的噪声 = 往返差的中位
      for I in 0 .. Natural (Scene.Length) - 1 loop
         declare
            P : Scene_Pt := Scene (I);
         begin
            P.Sh := Sh;
            Scene.Replace_Element (I, P);
         end;
      end loop;
      G.Cx := Long_Float (W) / 2.0; G.Cy := Long_Float (H) / 2.0; G.F := 0.0;   --  主点在正中、焦距一起解(同 Fit_Cam)
      Fit_Fixed_Board (G, Scene, Rp, Ok);
      Put_Line ((if Ok then "解出" else "解不出") & " · 板 " & Codec.Img (Rp.Scene_N) & " 个点、进解 " & Codec.Img (Rp.Scene_Used) & " · 残差 " & Codec.Fmt (Rp.Scene_Rms, 3)
                & " px · 焦距 " & Codec.Fmt (G.F, 2) & " · 位置 (" & Codec.Fmt (G.Pos (0), 3) & ", " & Codec.Fmt (G.Pos (1), 3) & ", " & Codec.Fmt (G.Pos (2), 3)
                & ") · 配点噪声 " & Codec.Fmt (Sh, 3) & " px" & (if Ok then "" else " · 为什么:" & Ada.Strings.Unbounded.To_String (Geom.Why)));
      if Ok then
         for A in 0 .. Natural (Worlds.Length) - 1 loop
            declare
               Ds : Floats;
               Su, Sv : Long_Float := 0.0;
               N : Natural := 0;
            begin
               for I in 0 .. Natural (All_Pts.Length) - 1 loop
                  if Arm_Of (I) = A then
                     declare
                        U, V : Long_Float;
                        Front : Boolean;
                     begin
                        Project_Fixed (G, All_Pts (I).Pw, U, V, Front);
                        if Front then
                           Ds.Append (Norm ([U - All_Pts (I).U, V - All_Pts (I).V, 0.0]));
                           Su := Su + (U - All_Pts (I).U); Sv := Sv + (V - All_Pts (I).V); N := N + 1;
                        end if;
                     end;
                  end if;
               end loop;
               if N > 0 then
                  Sorting.Sort (Ds);
                  Put_Line ("  第" & Natural'Image (A) & " 只手的点投回这只眼:" & Codec.Img (N) & " 个 · 残差中位 " & Codec.Fmt (Ds (N / 2), 2) & " px、九成 "
                            & Codec.Fmt (Ds (N * 9 / 10), 2) & " px · 平均偏 (" & Codec.Fmt (Su / Long_Float (N), 2) & ", " & Codec.Fmt (Sv / Long_Float (N), 2) & ") px");
               end if;
            end;
         end loop;
      end if;
   end;
end Fixedexam;
