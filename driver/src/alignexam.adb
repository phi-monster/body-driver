--  离线回放两只手对齐(2026-09-26,V1B11:第 2 只手对到第 1 只手的系,长度倍数 0.73、残差中位 0.96 单位 —— 两只手的模型单位按真值只差 0.8%)。
--  拿一炮落盘的扫描格(look/sweep.txt)、画面(sweep_*.bmp)、运动学(kinem_arm<k>.txt)、配点(corrs_arm<k>.txt)
--  原样跑驱动那一份 Jointboot.Align(要配点仪器在线),打出报告,落盘 align_arm<k>.txt / world.txt 到输出目录。真值不进解,只在打分脚本里用。
--  不动的眼的画面 = look/world_cam.bmp(扫描起点那一刻;没有这个文件 = 这具身体没有不长在手上的眼)。
--  用法:alignexam <look 目录> <输出目录> <仪器主机> <仪器端口> [第 0 只手的读数组号 第 1 只手的 …](不给 = 第 k 只手用第 k 组)
with Ada.Command_Line; use Ada.Command_Line;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Strings.Fixed;
with Ada.Directories;
with Bytes; use Bytes;
with Codec;
with Kinem;
with Plug;
with Geom;
with Jointboot;
with Instrument;
procedure Alignexam is
   Dir : constant String := Argument (1);
   Out_Dir : constant String := Argument (2);
   Host : constant String := Argument (3);
   Port : constant Natural := Natural'Value (Argument (4));
   Max_Arms : constant := 8;   --  最多几只手(次数)
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
   Ds : Jointboot.Sweep_Vectors.Vector;
   Worlds : Jointboot.Arm_World_Vectors.Vector;
   Css : Jointboot.Corr_Set_Vectors.Vector;
   Rw : Geom.M3;
   O : Geom.V3;
   Ok : Boolean;
   Fixed_Eye : Geom.Cam_Geo;
   N_Arms : Natural := 0;
   function Group_Of (A : Natural) return Natural is (if Argument_Count >= 5 + A then Natural'Value (Argument (5 + A)) else A);
begin
   --  这一炮有几只手:kinem_arm<k>.txt 连着有几个
   while N_Arms < Max_Arms and then Ada.Directories.Exists (Dir & "/kinem_arm" & Codec.Img (N_Arms) & ".txt") loop
      N_Arms := N_Arms + 1;
   end loop;
   for A in 0 .. N_Arms - 1 loop
      Ds.Append (Jointboot.Sweep_Data'(others => <>));
      Css.Append (Kinem.Corr_Vectors.Empty_Vector);
      Worlds.Append (Jointboot.Arm_World'(others => <>));
   end loop;
   --  扫描格 + 画面
   declare
      Fi : File_Type;
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
                  A : constant Natural := Natural'Value (H (1));
                  Groups : Plug.Floats_Vectors.Vector;
               begin
                  if A < N_Arms then
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
                              Groups.Append (Q);
                              exit when Bar = 0;
                              Start := Bar + 1;
                           end;
                        end loop;
                     end;
                     declare
                        Rgb : Buf;
                        W, Hh : Natural;
                        Okb : Boolean;
                        C : Plug.Cam;
                        J : constant Integer := Integer'Value (H (2));
                        K : constant Natural := Natural'Value (H (4));
                     begin
                        Codec.Read_BMP (Dir & "/" & H (0), Rgb, W, Hh, Okb);
                        if Okb and then Group_Of (A) < Natural (Groups.Length) then
                           C.W := W; C.H := Hh; C.RGB := Rgb;
                           Ds (A).Frames.Append (Kinem.Frame_Info'(Q => Groups (Group_Of (A)), Joint => (if K = 0 or else J = 99 then -1 else J)));
                           Ds (A).Imgs.Append (C);
                           Ds (A).W := W; Ds (A).H := Hh;
                        end if;
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
      Close (Fi);
   end;
   for A in 0 .. N_Arms - 1 loop
      --  运动学
      declare
         Fi : File_Type;
         M : Kinem.Model;
      begin
         Open (Fi, In_File, Dir & "/kinem_arm" & Codec.Img (A) & ".txt");
         while not End_Of_File (Fi) loop
            declare
               F : constant Strs := Fields (Get_Line (Fi));
            begin
               if Natural (F.Length) >= 10 and then F (0) = "arm" then
                  M.N := Natural'Value (F (3)); M.F := Long_Float'Value (F (5)); M.Cx := Long_Float'Value (F (7)); M.Cy := Long_Float'Value (F (9));
               elsif Natural (F.Length) >= 2 and then F (0) = "q0" then
                  for I in 1 .. Natural (F.Length) - 1 loop
                     M.Q0.Append (Long_Float'Value (F (I)));
                  end loop;
               elsif Natural (F.Length) >= 8 and then F (0) = "axis" then
                  declare
                     J : constant Natural := Natural'Value (F (1));
                  begin
                     M.Ax (J).W := [Long_Float'Value (F (2)), Long_Float'Value (F (3)), Long_Float'Value (F (4))];
                     M.Ax (J).P := [Long_Float'Value (F (5)), Long_Float'Value (F (6)), Long_Float'Value (F (7))];
                  end;
               end if;
            end;
         end loop;
         Close (Fi);
         M.Valid := True;
         Worlds (A).Model := M; Worlds (A).Group := Group_Of (A); Worlds (A).Valid := True;
      end;
      --  配点
      declare
         Fi : File_Type;
      begin
         Open (Fi, In_File, Dir & "/corrs_arm" & Codec.Img (A) & ".txt");
         while not End_Of_File (Fi) loop
            declare
               F : constant Strs := Fields (Get_Line (Fi));
            begin
               if Natural (F.Length) >= 6 then
                  Css (A).Append (Kinem.Corr'(I => Natural'Value (F (0)), J => Natural'Value (F (1)), Ua => Long_Float'Value (F (2)), Va => Long_Float'Value (F (3)),
                                              Ub => Long_Float'Value (F (4)), Vb => Long_Float'Value (F (5))));
               end if;
            end;
         end loop;
         Close (Fi);
      end;
      Put_Line ("手" & Natural'Image (A) & ":" & Codec.Img (Natural (Ds (A).Frames.Length)) & " 格(读数第" & Natural'Image (Group_Of (A)) & " 组)· 配点 "
                & Codec.Img (Natural (Css (A).Length)) & " · 焦距 " & Codec.Fmt (Worlds (A).Model.F, 1));
   end loop;
   --  每一格、不动的眼的画面存到配点仪器那边(驱动扫描时就是这么存的,对齐按编号配)
   declare
      Err : Unbounded_String;
      Id : Integer;
      Wi : Plug.Cam;
      Okb : Boolean;
      Has_World : constant Boolean := Ada.Directories.Exists (Dir & "/world_cam.bmp");
   begin
      if Has_World then
         Codec.Read_BMP (Dir & "/world_cam.bmp", Wi.RGB, Wi.W, Wi.H, Okb);
         if Okb then
            Instrument.Frame_Put (Host, Port, Wi.RGB, Wi.W, Wi.H, Id, Err);
            for A in 0 .. N_Arms - 1 loop
               Ds (A).World_Img := Wi; Ds (A).World_Id := Id;
            end loop;
         end if;
      end if;
      for A in 0 .. N_Arms - 1 loop
         for C of Ds (A).Imgs loop
            Instrument.Frame_Put (Host, Port, C.RGB, C.W, C.H, Id, Err);
            Ds (A).Ids.Append (Id);
         end loop;
      end loop;
      Put_Line ("不动的眼:" & (if Has_World then "有(world_cam.bmp)" else "没有"));
   end;
   declare
      Board : Geom.Scene_Pt_Vectors.Vector;
      Pp, Pn : Geom.V3;
      Pr : Long_Float;
   begin
      Jointboot.Align (Ds, Worlds, Css, Host, Port, Rw, O, Ok, Fixed_Eye, Board, Pp, Pn, Pr, Dump => Out_Dir,
                       Pin_Fixed_F => (if Codec.Env ("ALIGNEXAM_PIN_F") /= "" then Long_Float'Value (Codec.Env ("ALIGNEXAM_PIN_F")) else 0.0));   --  对照实验:钉住不动的眼的焦距
      --  交给开机后半段的板落盘(每行:世界系 x y z、离桌面多高、沿法向的不确定度、几只眼看见;单位 = 世界单位),离线看开机碰桌面挑的落点附近有什么
      declare
         Fo : File_Type;
      begin
         Create (Fo, Out_File, Out_Dir & "/board.txt");
         Put_Line (Fo, "# 桌面点 " & Codec.Fmt (Pp (0), 5) & " " & Codec.Fmt (Pp (1), 5) & " " & Codec.Fmt (Pp (2), 5) & " 法向 "
                   & Codec.Fmt (Pn (0), 5) & " " & Codec.Fmt (Pn (1), 5) & " " & Codec.Fmt (Pn (2), 5) & " 离散 " & Codec.Fmt (Pr, 5));
         for S of Board loop
            declare
               H : constant Long_Float := (S.Pw (0) - Pp (0)) * Pn (0) + (S.Pw (1) - Pp (1)) * Pn (1) + (S.Pw (2) - Pp (2)) * Pn (2);
               Cn : constant Geom.V3 := Geom.Ap (S.Cov, Pn);
            begin
               Put_Line (Fo, Codec.Fmt (S.Pw (0), 5) & " " & Codec.Fmt (S.Pw (1), 5) & " " & Codec.Fmt (S.Pw (2), 5) & " " & Codec.Fmt (H, 5) & " "
                         & Codec.Fmt (Cn (0) * Pn (0) + Cn (1) * Pn (1) + Cn (2) * Pn (2), 8) & " " & Codec.Img (S.Views));
            end;
         end loop;
         Close (Fo);
      end;
   end;
   Put_Line (if Ok then "对齐做完(报告见上面 [身] 那几行;点对落盘在输出目录)" else "对齐没做成");
end Alignexam;
