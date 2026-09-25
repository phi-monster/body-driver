--  离线跑标定板(2026-09-25):拿一炮落盘的腕眼各停灰度图(look/geo_cam<k>_stop<i>.pgm,位姿在 geo_cam<k>_obs.txt)、
--  同一刻的不动的眼的帧(vid/f<n>_c<m>.pgm:vid/poses.txt 里那只手的位姿和这一停一样、画面已经静止的那一帧)、那一炮解好的腕眼几何(<身体文件>.geo.json),
--  走驱动同一段代码:Act.Geo_Board(仪器配点 → Geom.Build_Board → 板上的点躺的面)再 Geom.Fit_Fixed_Rig(按板解不动的眼),打出结果。
--  灰度图复制成三通道交给仪器(驱动里是彩色图)。平移停 = 朝向和同组其它停差不到 0.02 rad 的那一组里最大的那组(转动停各自朝向不同)。
--  用法:boardexam <look 目录> <vid 目录> <几何文件> <仪器 host> <仪器 port> [不动的眼的相机号,默认 0]
with Ada.Command_Line; use Ada.Command_Line;
with Ada.Containers.Vectors;
with Ada.Containers;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Streams.Stream_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Strings.Fixed;
with Bytes; use Bytes;
with Geom;
with Plug;
with Codec;
with Act;
procedure Boardexam is
   --  读 P5 灰度图(同 zoneexam)
   procedure Read_PGM (Path : String; G : out Buf; W, H : out Natural) is
      package SIO renames Ada.Streams.Stream_IO;
      Fi : SIO.File_Type;
      S : SIO.Stream_Access;
      C : Character;
      function Next_Num return Natural is
         N : Natural := 0;
         Got : Boolean := False;
      begin
         loop
            Character'Read (S, C);
            if C = '#' then
               while C /= ASCII.LF loop
                  Character'Read (S, C);
               end loop;
            elsif C in '0' .. '9' then
               N := N * 10 + (Character'Pos (C) - Character'Pos ('0'));   --  十进制(纯数学)
               Got := True;
            elsif Got then
               return N;
            end if;
         end loop;
      end Next_Num;
   begin
      SIO.Open (Fi, SIO.In_File, Path);
      S := SIO.Stream (Fi);
      Character'Read (S, C);
      Character'Read (S, C);   --  "P5"
      W := Next_Num;
      H := Next_Num;
      declare
         Mx : constant Natural := Next_Num;
         pragma Unreferenced (Mx);
      begin
         null;
      end;
      G := U8_Vectors.Empty_Vector;
      G.Reserve_Capacity (Ada.Containers.Count_Type (W * H));
      for I in 1 .. W * H loop
         Character'Read (S, C);
         G.Append (U8 (Character'Pos (C)));
      end loop;
      SIO.Close (Fi);
   end Read_PGM;
   function To_RGB (G : Buf) return Buf is
      R : Buf;
   begin
      R.Reserve_Capacity (Ada.Containers.Count_Type (3 * Natural (G.Length)));
      for B of G loop
         R.Append (B); R.Append (B); R.Append (B);
      end loop;
      return R;
   end To_RGB;
   --  第 K 个空格分开的字段(没有 ⇒ "")
   function Field (S : String; K : Positive) return String is
      N : Natural := 0;
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
   function Same (P, Q : Plug.Arm_Pose) return Boolean is
      Tol : constant Long_Float := 2.0e-6;   --  落盘位姿是 6–7 位小数(格式的舍入,协议)
   begin
      for I in 0 .. 6 loop
         if abs (P (I) - Q (I)) > Tol then
            return False;
         end if;
      end loop;
      return True;
   end Same;
   type Pose_Pair is array (0 .. 1) of Plug.Arm_Pose;   --  两只手的位姿
   type Vid_Rec is record
      Vid : Natural := 0;
      P : Pose_Pair;
   end record;
   package Vid_Vectors is new Ada.Containers.Vectors (Natural, Vid_Rec);
   Vids : Vid_Vectors.Vector;
   C : Act.Context;
   Head_Cam : Natural := 0;
   Same_Turn : constant Long_Float := 0.02;   --  平移停之间朝向几乎不变(弧度;转动停差一整档,这是认"没转"的协议)
begin
   if Argument_Count < 5 then
      Put_Line ("用法:boardexam <look 目录> <vid 目录> <几何文件> <仪器 host> <仪器 port> [不动的眼的相机号]");
      return;
   end if;
   if Argument_Count >= 6 then
      Head_Cam := Natural'Value (Argument (6));
   end if;
   declare
      Note : String (1 .. 160);
   begin
      Geom.Load (Argument (3), C.Geo, 3, Note);   --  三台相机:不动的眼 + 两只腕眼(这两具身体的落盘布局)
      Put_Line (Ada.Strings.Fixed.Trim (Note, Ada.Strings.Both));
   end;
   C.Inst_Host := To_Unbounded_String (Argument (4));
   C.Inst_Port := Natural'Value (Argument (5));
   declare
      Fi : File_Type;
   begin
      Open (Fi, In_File, Argument (2) & "/poses.txt");
      while not End_Of_File (Fi) loop
         declare
            L : constant String := Get_Line (Fi);
         begin
            if Field (L, 2) /= "-1" and then Field (L, 16) /= "" then
               Vids.Append (Vid_Rec'(Vid => Natural'Value (Field (L, 2)), P => [Pose_Of (L, 3), Pose_Of (L, 10)]));
            end if;
         end;
      end loop;
      Close (Fi);
   end;
   Put_Line ("vid 帧 " & Codec.Img (Natural (Vids.Length)) & " 个(带位姿)");
   for K in 1 .. 2 loop
      declare
         Stops : Plug.Pose_Vectors.Vector;
         Fi : File_Type;
         Best_I, Best_N : Natural := 0;
         From : Natural := 0;   --  vid 里往后找(停是按时间顺序的)
      begin
         Open (Fi, In_File, Argument (1) & "/geo_cam" & Codec.Img (K) & "_obs.txt");
         Skip_Line (Fi);
         while not End_Of_File (Fi) loop
            declare
               L : constant String := Get_Line (Fi);
            begin
               if Field (L, 10) /= "" then
                  declare
                     P : constant Plug.Arm_Pose := Pose_Of (L, 4);
                  begin
                     if Stops.Is_Empty or else not Same (Stops.Last_Element, P) then
                        Stops.Append (P);
                     end if;
                  end;
               end if;
            end;
         end loop;
         Close (Fi);
         for I in 0 .. Natural (Stops.Length) - 1 loop
            declare
               N : Natural := 0;
            begin
               for J in 0 .. Natural (Stops.Length) - 1 loop
                  if Geom.Angle_Between (Stops (I), Stops (J)) < Same_Turn then
                     N := N + 1;
                  end if;
               end loop;
               if N > Best_N then
                  Best_N := N; Best_I := I;
               end if;
            end;
         end loop;
         Put_Line ("第" & Codec.Img (K) & " 台腕眼:" & Codec.Img (Natural (Stops.Length)) & " 停,平移停 " & Codec.Img (Best_N) & " 个");
         for I in 0 .. Natural (Stops.Length) - 1 loop
            if Geom.Angle_Between (Stops (Best_I), Stops (I)) < Same_Turn then
               declare
                  Arm : Integer := -1;
                  Hit : Integer := -1;
                  Run : Natural := 0;
               begin
                  --  vid 里第一段位姿和这一停一样的帧:取这一段的第 3 帧(画面比位姿晚一帧,静止之后再取),这一段不到 3 帧就取最后一帧
                  for V in From .. Natural (Vids.Length) - 1 loop
                     declare
                        A : Integer := -1;
                     begin
                        if Same (Vids (V).P (0), Stops (I)) then
                           A := 0;
                        elsif Same (Vids (V).P (1), Stops (I)) then
                           A := 1;
                        end if;
                        if A >= 0 then
                           Arm := A; Hit := V; Run := Run + 1;
                           exit when Run >= 3;   --  次数
                        elsif Run > 0 then
                           exit;
                        end if;
                     end;
                  end loop;
                  if Hit < 0 then
                     Put_Line ("  第 " & Codec.Img (I) & " 停:vid 里找不到这个位姿 ⇒ 不用");
                  else
                     From := Natural (Hit) + 1;
                     declare
                        Wg, Hg : Buf;
                        W, H, Hw, Hh : Natural;
                        Vn : constant String := Codec.Pad6 (Vids (Natural (Hit)).Vid);
                     begin
                        Read_PGM (Argument (1) & "/geo_cam" & Codec.Img (K) & "_stop" & Codec.Img (I) & ".pgm", Wg, W, H);
                        Read_PGM (Argument (2) & "/f" & Vn & "_c" & Codec.Img (Head_Cam) & ".pgm", Hg, Hw, Hh);
                        C.Board_Stops.Append (Act.Board_Stop'(Cam => K, Arm => Natural (Arm), Seg => 0, Pose => Stops (I), W => W, H => H, RGB => To_RGB (Wg),
                                                              Hw => Hw, Hh => Hh, Head => To_RGB (Hg)));
                        Put_Line ("  第 " & Codec.Img (I) & " 停 ↔ vid f" & Vn & "(第" & Codec.Img (Arm + 1) & " 只手)");
                     end;
                  end if;
               end;
            end if;
         end loop;
      end;
   end loop;
   Act.Geo_Board (C);
   declare
      G : Geom.Cam_Geo;
      No_Obs : Geom.Obs_Pt_Vectors.Vector;
      Ro, Rd : Geom.V3_Vectors.Vector;
      Ok_K : Geom.Nat_Vectors.Vector;
      Tips : Geom.Tip_Class_Vectors.Vector;
      Rep : Geom.Fixed_Report;
      Ok : Boolean;
      Hw : constant Natural := (if C.Board_Stops.Is_Empty then 640 else C.Board_Stops.First_Element.Hw);   --  没有停就按常见画幅(只为打印)
      Hh : constant Natural := (if C.Board_Stops.Is_Empty then 480 else C.Board_Stops.First_Element.Hh);
   begin
      G.F := 0.0; G.Cx := Long_Float (Hw) / 2.0; G.Cy := Long_Float (Hh) / 2.0;
      Geom.Fit_Fixed_Rig (G, No_Obs, C.Board, Ro, Rd, Ok_K, Tips, Rep, Ok);
      if Ok then
         Put_Line ("不动的眼按板解:" & Codec.Img (Rep.Scene_Used) & "/" & Codec.Img (Rep.Scene_N) & " 个点 · 残差 " & Codec.Fmt (Rep.Scene_Rms, 2) & " px · 踢掉 " & Codec.Img (G.Dropped)
                   & " · 它在 (" & Codec.Fmt (G.Pos (0), 4) & "," & Codec.Fmt (G.Pos (1), 4) & "," & Codec.Fmt (G.Pos (2), 4) & ") ± " & Codec.Fmt (G.Pos_Sd, 4) & " m · 焦距 "
                   & Codec.Fmt (G.F, 1) & " ± " & Codec.Fmt (G.F_Sd, 1) & " px · 朝向 ± " & Codec.Fmt (G.Rot_Sd, 4) & " rad");
      else
         Put_Line ("不动的眼按板解不出:" & To_String (Geom.Why));
      end if;
      if C.Board_Plane then
         Put_Line ("板上的点躺的面:过 (" & Codec.Fmt (C.Board_Pt (0), 4) & "," & Codec.Fmt (C.Board_Pt (1), 4) & "," & Codec.Fmt (C.Board_Pt (2), 4) & "),法向 ("
                   & Codec.Fmt (C.Board_N (0), 4) & "," & Codec.Fmt (C.Board_N (1), 4) & "," & Codec.Fmt (C.Board_N (2), 4) & ")");
      end if;
   end;
end Boardexam;
