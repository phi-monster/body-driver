--  离线回放运动学(2026-09-26,V1b 5 分钟一炮):拿一炮落盘的扫描格子(look/sweep.txt:每行 图名 臂 关节 方向 第几格 拍数 | 各组读数 … || 真值)
--  和配点(look/corrs_arm<k>.txt:每行 I J Ua Va Ub Vb)原样跑驱动那一份 Kinem.Fit,打出报告、把模型写成 kinem_arm<k>.txt(格式同驱动落盘)。
--  改解法不用再开一炮;真值不进解,只在打分脚本里用。这只手的读数是哪一组:这只手各格之间变了的第一组(同驱动认手的判法)。
--  用法:kinexam <look 目录> <臂号> <输出目录>
with Ada.Command_Line; use Ada.Command_Line;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Strings.Fixed;
with Ada.Containers.Vectors;
with Bytes; use Bytes;
with Codec;
with Kinem;
procedure Kinexam is
   Dir : constant String := Argument (1);
   Arm : constant Natural := Natural'Value (Argument (2));
   Out_Dir : constant String := Argument (3);
   --  一行按空格切开
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
   Frames : Kinem.Frame_Vectors.Vector;
   Cs : Kinem.Corr_Vectors.Vector;
   package Group_Vectors is new Ada.Containers.Vectors (Natural, Floats, F64_Vectors."=");
   All_Groups : array (0 .. 4095) of Group_Vectors.Vector;   --  每一格的各组读数(格子数上限,次数)
   Jf, Kf : array (0 .. 4095) of Integer := [others => 0];
   Nf : Natural := 0;
   G : Integer := -1;
   Fi : File_Type;
   M : Kinem.Model;
   Rep : Kinem.Fit_Report;
   Ok : Boolean;
begin
   Open (Fi, In_File, Dir & "/sweep.txt");
   while not End_Of_File (Fi) loop
      declare
         Line : constant String := Get_Line (Fi);
         P2 : constant Natural := Ada.Strings.Fixed.Index (Line, "||");
      begin
         if P2 > 0 then
            declare
               Left : constant String := Line (Line'First .. P2 - 1);
               First_Bar : constant Natural := Ada.Strings.Fixed.Index (Left, "|");
               H : constant Strs := Fields (Left (Left'First .. First_Bar - 1));
            begin
               if Natural'Value (H (1)) = Arm and then Nf <= All_Groups'Last then
                  Jf (Nf) := Integer'Value (H (2)); Kf (Nf) := Integer'Value (H (4));
                  declare
                     Rest : constant String := Left (First_Bar + 1 .. Left'Last);
                     Start : Natural := Rest'First;
                  begin
                     loop
                        declare
                           Bar : constant Natural := Ada.Strings.Fixed.Index (Rest (Start .. Rest'Last), "|");
                           Seg : constant String := (if Bar > 0 then Rest (Start .. Bar - 1) else Rest (Start .. Rest'Last));
                           Q : Floats;
                        begin
                           for X of Fields (Seg) loop
                              Q.Append (Long_Float'Value (X));
                           end loop;
                           All_Groups (Nf).Append (Q);
                           exit when Bar = 0;
                           Start := Bar + 1;
                        end;
                     end loop;
                  end;
                  Nf := Nf + 1;
               end if;
            end;
         end if;
      end;
   end loop;
   Close (Fi);
   --  这只手的读数组:各格之间变了的第一组
   for Gi in 0 .. Natural (All_Groups (0).Length) - 1 loop
      if G < 0 then
         for Fr in 1 .. Nf - 1 loop
            if Gi < Natural (All_Groups (Fr).Length) and then not F64_Vectors."=" (All_Groups (Fr) (Gi), All_Groups (0) (Gi)) then
               G := Gi;
               exit;
            end if;
         end loop;
      end if;
   end loop;
   if G < 0 then
      Put_Line ("这只手各格的读数都一样,认不出是哪组");
      return;
   end if;
   for Fr in 0 .. Nf - 1 loop
      Frames.Append (Kinem.Frame_Info'(Q => All_Groups (Fr) (G), Joint => (if Kf (Fr) = 0 or else Jf (Fr) = 99 then -1 else Jf (Fr))));
   end loop;
   Open (Fi, In_File, Dir & "/corrs_arm" & Codec.Img (Arm) & ".txt");
   while not End_Of_File (Fi) loop
      declare
         F : constant Strs := Fields (Get_Line (Fi));
      begin
         if Natural (F.Length) >= 6 then
            Cs.Append (Kinem.Corr'(I => Natural'Value (F (0)), J => Natural'Value (F (1)), Ua => Long_Float'Value (F (2)), Va => Long_Float'Value (F (3)),
                                   Ub => Long_Float'Value (F (4)), Vb => Long_Float'Value (F (5))));
         end if;
      end;
   end loop;
   Close (Fi);
   Put_Line ("臂" & Natural'Image (Arm) & ":" & Codec.Img (Nf) & " 格(读数第" & Integer'Image (G) & " 组)· 配点 " & Codec.Img (Natural (Cs.Length)));
   Kinem.Fit (Frames, 0, Cs, 320.0, 240.0, 640.0, M, Rep, Ok);
   declare
      T : Unbounded_String;
   begin
      for J in 0 .. Natural (Rep.Joint_Med.Length) - 1 loop
         Append (T, " " & Codec.Fmt (Rep.Joint_Med (J), 3));
      end loop;
      Put_Line ((if Ok then "量成" else "没量成") & " · 每根轴单独的残差中位(像素):" & To_String (T));
      T := Null_Unbounded_String;
      for X of Rep.Rho loop
         Append (T, " " & Codec.Fmt (X, 3));
      end loop;
      Put_Line ("焦距 起步 " & Codec.Fmt (Rep.F_Start, 1) & " → " & Codec.Fmt (Rep.F, 1) & " · 一起解的残差中位 " & Codec.Fmt (Rep.Med_Px, 3) & " px、九成 "
                & Codec.Fmt (Rep.P90_Px, 3) & " px · 内点 " & Codec.Img (Rep.N_Used) & " · 各轴比例(以第" & Codec.Img (Rep.Ref_Joint) & " 根为 1):" & To_String (T));
   end;
   if Ok then
      declare
         Fo : File_Type;
      begin
         Create (Fo, Out_File, Out_Dir & "/kinem_arm" & Codec.Img (Arm) & ".txt");
         Put_Line (Fo, "arm " & Codec.Img (Arm) & " n " & Codec.Img (M.N) & " f " & Codec.Fmt (M.F, 6) & " cx " & Codec.Fmt (M.Cx, 3) & " cy " & Codec.Fmt (M.Cy, 3));
         Put (Fo, "q0");
         for X of M.Q0 loop
            Put (Fo, " " & Codec.Fmt (X, 9));
         end loop;
         New_Line (Fo);
         for J in 0 .. M.N - 1 loop
            Put_Line (Fo, "axis " & Codec.Img (J) & " " & Codec.Fmt (M.Ax (J).W (0), 9) & " " & Codec.Fmt (M.Ax (J).W (1), 9) & " " & Codec.Fmt (M.Ax (J).W (2), 9) & " "
                      & Codec.Fmt (M.Ax (J).P (0), 9) & " " & Codec.Fmt (M.Ax (J).P (1), 9) & " " & Codec.Fmt (M.Ax (J).P (2), 9));
         end loop;
         Close (Fo);
      end;
   end if;
end Kinexam;
