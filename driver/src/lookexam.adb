--  离线看"压之前先看底下"(09-30):拿一次开机录下的两帧腕眼灰度图(vid/fNNNNNN_cK.pgm)和驱动按关节读数算的那两拍位姿
--  (vid/fk_poses.txt,世界系、模型单位),跑驱动同一段 Act.Seen_Above_Of(问 Kinem 那张格点、配点仪器往返配),
--  打出配上 / 交成 / 比面高出的点数,交成的点按离面多高分几档数一数,写一张标了点的彩图(PPM):
--  红 = 判成比面高出,绿 = 交成、没高出,蓝 = 配上了、两帧对不上(或交不成),不标 = 没配上。
--  面按驱动开机量的:世界 z = 0、法向 +z,离散给出(日志"桌面离散")。手指像素这里不挑出去(全问:长在眼上的交不出远近,判不成高出面)。
--  灰度图按三个通道一样当彩图发(驱动发的是彩图)。
--  用法:lookexam run_dir seq0 seq1 cam arm geo.json plane_rms host port [out.ppm [tip_u tip_v R step_px]]
--  给了压的那一瓣尖的像素、落点圈半径 R(世界单位)、铺点间距(像素):按驱动同一个 Act.Look_Points 在落点圈里密铺(落点 = 第一帧里那一瓣的视线交面),
--  另报落点圈里判成高出面的点
with Ada.Command_Line;
with Ada.Containers;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Streams.Stream_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Bytes; use Bytes;
with Codec;
with Kinem;
with Geom;
with Plug;
with Act;
with Instrument;
with Stats;
procedure Lookexam is
   package SIO renames Ada.Streams.Stream_IO;
   --  读 P5 灰度图:头部三个数(宽、高、最大值)之后是原始字节
   procedure Read_PGM (Path : String; G : out Buf; W, H : out Natural) is
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
   function Rgb (G : Buf) return Buf is
      R : Buf;
   begin
      for P of G loop
         R.Append (P); R.Append (P); R.Append (P);
      end loop;
      return R;
   end Rgb;
   function Pad (N : Natural) return String is
      S : constant String := Codec.Img (N);
   begin
      return [1 .. 6 - S'Length => '0'] & S;   --  帧号六位(录像文件名的格式)
   end Pad;
   --  fk_poses.txt:每行 = 帧号、-1、每只手 7 个数(位置 3 + 四元数 4)
   function Pose_At (Path : String; Seq, Arm : Natural; Ok : out Boolean) return Plug.Arm_Pose is
      Fi : File_Type;
      P : Plug.Arm_Pose := [others => 0.0];
   begin
      Ok := False;
      Open (Fi, In_File, Path);
      while not End_Of_File (Fi) loop
         declare
            Ln : constant String := Get_Line (Fi);
            F : Strs;
            Cur : Unbounded_String;
         begin
            for Ch of Ln loop
               if Ch = ' ' then
                  if Length (Cur) > 0 then
                     F.Append (To_String (Cur)); Cur := Null_Unbounded_String;
                  end if;
               else
                  Append (Cur, Ch);
               end if;
            end loop;
            if Length (Cur) > 0 then
               F.Append (To_String (Cur));
            end if;
            if Natural (F.Length) >= 2 + Plug.Arm_Pose'Length * (Arm + 1) and then Natural'Value (F (0)) = Seq then
               for I in Plug.Arm_Pose'Range loop
                  P (I) := Long_Float'Value (F (2 + Plug.Arm_Pose'Length * Arm + I));
               end loop;
               Ok := True;
               exit;
            end if;
         end;
      end loop;
      Close (Fi);
      return P;
   end Pose_At;
begin
   if Ada.Command_Line.Argument_Count < 9 then
      Put_Line ("用法:lookexam run_dir seq0 seq1 cam arm geo.json plane_rms host port [out.ppm]");
      return;
   end if;
   declare
      Run : constant String := Ada.Command_Line.Argument (1);
      S0 : constant Natural := Natural'Value (Ada.Command_Line.Argument (2));
      S1 : constant Natural := Natural'Value (Ada.Command_Line.Argument (3));
      Cam : constant Natural := Natural'Value (Ada.Command_Line.Argument (4));
      Arm : constant Natural := Natural'Value (Ada.Command_Line.Argument (5));
      Host : constant String := Ada.Command_Line.Argument (8);
      Port : constant Natural := Natural'Value (Ada.Command_Line.Argument (9));
      G0, G1 : Buf;
      W, H, W1, H1 : Natural;
      Gs : Geom.Geo_Vectors.Vector;
      Note : String (1 .. 200);
      Ok0, Ok1 : Boolean;
      P0, P1 : Plug.Arm_Pose;
      C : Act.Context;
      Q, M : Instrument.Match_Vectors.Vector;
      Err : Unbounded_String;
      Qu, Qv, Mu, Mv, Bu, Bv : Floats;
      Above : Geom.Scene_Pt_Vectors.Vector;
      Matched, Tri : Natural;
      Sig : Long_Float;
   begin
      Read_PGM (Run & "/vid/f" & Pad (S0) & "_c" & Codec.Img (Cam) & ".pgm", G0, W, H);
      Read_PGM (Run & "/vid/f" & Pad (S1) & "_c" & Codec.Img (Cam) & ".pgm", G1, W1, H1);
      if W /= W1 or else H /= H1 then
         Put_Line ("两帧大小不一样");
         return;
      end if;
      Geom.Load (Ada.Command_Line.Argument (6), Gs, Cam + 1, Note);
      if Natural (Gs.Length) <= Cam or else not Gs (Cam).Valid then
         Put_Line ("几何文件里没有第" & Codec.Img (Cam) & " 台相机(" & Note & ")");
         return;
      end if;
      P0 := Pose_At (Run & "/vid/fk_poses.txt", S0, Arm, Ok0);
      P1 := Pose_At (Run & "/vid/fk_poses.txt", S1, Arm, Ok1);
      if not (Ok0 and then Ok1) then
         Put_Line ("fk_poses.txt 里没有这两拍");
         return;
      end if;
      C.Board_Plane := True; C.Board_Pt := [0.0, 0.0, 0.0]; C.Board_N := [0.0, 0.0, 1.0];
      C.Board_Rms := Long_Float'Value (Ada.Command_Line.Argument (7));
      declare
         Spot : Geom.V3 := [0.0, 0.0, 0.0];
         Rr : Long_Float := 0.0;
         Step : Long_Float := 0.0;
         No_Fingers : Bools;
         Ends : Geom.V3_Vectors.Vector;
      begin
         if Ada.Command_Line.Argument_Count >= 14 then
            declare
               Hok : Boolean;
            begin
               Spot := Geom.Hit_Plane (Geom.Cam_Pos (Gs (Cam), P0), Geom.Ray (Gs (Cam), P0, Long_Float'Value (Ada.Command_Line.Argument (11)),
                                       Long_Float'Value (Ada.Command_Line.Argument (12))), C.Board_Pt, C.Board_N, Hok);
               Rr := (if Hok then Long_Float'Value (Ada.Command_Line.Argument (13)) else 0.0);
               Step := Long_Float'Value (Ada.Command_Line.Argument (14));
               Put_Line ("落点 (" & Codec.Fmt (Spot (0), 3) & ", " & Codec.Fmt (Spot (1), 3) & ") · 落点圈半径 " & Codec.Fmt (Rr, 3) & " · 铺点间距 " & Codec.Fmt (Step, 1) & " px");
            end;
         end if;
         Q := Act.Look_Points (C, Gs (Cam), P0, W, H, No_Fingers, Spot, Ends, Rr, Step);
         if Rr > 0.0 then
            declare
               Above2 : Geom.Scene_Pt_Vectors.Vector;
               M2 : Instrument.Match_Vectors.Vector;
               Err2 : Unbounded_String;
               Qu2, Qv2, Mu2, Mv2, Bu2, Bv2 : Floats;
               Mt2, Tr2 : Natural;
               Sg2 : Long_Float;
               N_In, Hit : Natural := 0;
            begin
               M2 := Instrument.Match (Host, Port, Rgb (G0), W, H, Rgb (G1), W, H, Q, Err2, Back => True);
               if Natural (M2.Length) = Natural (Q.Length) then
                  for I in 0 .. Natural (Q.Length) - 1 loop
                     Qu2.Append (Q (I).U); Qv2.Append (Q (I).V); Mu2.Append (M2 (I).U); Mv2.Append (M2 (I).V); Bu2.Append (M2 (I).Bu); Bv2.Append (M2 (I).Bv);
                  end loop;
                  Act.Seen_Above_Of (C, Gs (Cam), P0, P1, W, H, Qu2, Qv2, Mu2, Mv2, Bu2, Bv2, Above2, Mt2, Tr2, Sg2);
                  N_In := Natural (Q.Length) - Kinem.Gx * Kinem.Gy;
                  for A of Above2 loop
                     if Sqrt ((A.Pw (0) - Spot (0)) ** 2 + (A.Pw (1) - Spot (1)) ** 2) <= Rr then
                        Hit := Hit + 1;
                        Put_Line ("  落点圈里高出面 " & Codec.Fmt (A.Pw (2), 3) & " @ (" & Codec.Fmt (A.Pw (0), 3) & ", " & Codec.Fmt (A.Pw (1), 3) & ") 离落点 "
                                  & Codec.Fmt (Sqrt ((A.Pw (0) - Spot (0)) ** 2 + (A.Pw (1) - Spot (1)) ** 2), 3));
                     end if;
                  end loop;
                  Put_Line ("密铺 " & Codec.Img (N_In) & " 个 · 一共配上 " & Codec.Img (Mt2) & "、交成 " & Codec.Img (Tr2) & " · 落点圈里高出面的 " & Codec.Img (Hit) & " 个"
                            & (if Hit > 0 then " ⇒ 这一处被挡" else " ⇒ 这一处照样空"));
               end if;
            end;
         end if;
      end;
      M := Instrument.Match (Host, Port, Rgb (G0), W, H, Rgb (G1), W, H, Q, Err, Back => True);
      if Natural (M.Length) /= Natural (Q.Length) then
         Put_Line ("仪器没配成:" & To_String (Err));
         return;
      end if;
      for I in 0 .. Natural (Q.Length) - 1 loop
         Qu.Append (Q (I).U); Qv.Append (Q (I).V); Mu.Append (M (I).U); Mv.Append (M (I).V); Bu.Append (M (I).Bu); Bv.Append (M (I).Bv);
      end loop;
      Act.Seen_Above_Of (C, Gs (Cam), P0, P1, W, H, Qu, Qv, Mu, Mv, Bu, Bv, Above, Matched, Tri, Sig);
      declare
         E0 : constant Geom.V3 := Geom.Cam_Pos (Gs (Cam), P0);
         E1 : constant Geom.V3 := Geom.Cam_Pos (Gs (Cam), P1);
      begin
         Put_Line ("眼 " & Codec.Fmt (E0 (0), 3) & " " & Codec.Fmt (E0 (1), 3) & " " & Codec.Fmt (E0 (2), 3) & " → " & Codec.Fmt (E1 (0), 3) & " " & Codec.Fmt (E1 (1), 3)
                   & " " & Codec.Fmt (E1 (2), 3) & "(挪 " & Codec.Fmt (Geom.Norm ([E1 (0) - E0 (0), E1 (1) - E0 (1), E1 (2) - E0 (2)]), 3) & ")");
      end;
      Put_Line ("问 " & Codec.Img (Natural (Q.Length)) & " · 配上 " & Codec.Img (Matched) & " · 交成且两帧对得上 " & Codec.Img (Tri) & " · 配点噪声 " & Codec.Fmt (Sig, 3)
                & " px · 比面高出 " & Codec.Img (Natural (Above.Length)));
      --  交成的点离面多高(诊断:驱动同一套交法,按高度分档数;档宽 = 面离散的 Z 倍)
      declare
         Tol : constant Long_Float := Stats.Z * C.Board_Rms;
         Bins : array (-3 .. 20) of Natural := [others => 0];
         Col : array (0 .. Natural (Q.Length) - 1) of Character := [others => ' '];
      begin
         for I in 0 .. Natural (Q.Length) - 1 loop
            if M (I).U >= 0.0 and then M (I).V >= 0.0 and then M (I).U < Long_Float (W) and then M (I).V < Long_Float (H)
              and then Geom.Round_Trip_Ok (Q (I).U, Q (I).V, M (I).Bu, M (I).Bv)
            then
               declare
                  Rays : Geom.Sight_Vectors.Vector;
                  Okm, F0, F1 : Boolean;
                  Spread, U0, V0, U1, V1 : Long_Float;
                  X : Geom.V3;
               begin
                  Col (I) := 'b';
                  Rays.Append (Geom.Sight'(O => Geom.Cam_Pos (Gs (Cam), P0), D => Geom.Ray (Gs (Cam), P0, Q (I).U, Q (I).V)));
                  Rays.Append (Geom.Sight'(O => Geom.Cam_Pos (Gs (Cam), P1), D => Geom.Ray (Gs (Cam), P1, M (I).U, M (I).V)));
                  X := Geom.Meet (Rays, Okm, Spread);
                  if Okm then
                     Geom.Project (Gs (Cam), P0, X, U0, V0, F0);
                     Geom.Project (Gs (Cam), P1, X, U1, V1, F1);
                     if F0 and then F1 and then Sqrt ((U0 - Q (I).U) ** 2 + (V0 - Q (I).V) ** 2 + (U1 - M (I).U) ** 2 + (V1 - M (I).V) ** 2) <= Stats.Z * Sig then
                        Col (I) := 'g';
                        declare
                           Bn : constant Integer := Integer (Long_Float'Floor (X (2) / Tol));
                        begin
                           Bins (Integer'Max (Bins'First, Integer'Min (Bins'Last, Bn))) := Bins (Integer'Max (Bins'First, Integer'Min (Bins'Last, Bn))) + 1;
                        end;
                     end if;
                  end if;
               end;
            end if;
         end loop;
         for A of Above loop
            declare
               U0, V0 : Long_Float;
               F0 : Boolean;
               Best : Natural := 0;
               Bd : Long_Float := Long_Float'Last;
            begin
               Geom.Project (Gs (Cam), P0, A.Pw, U0, V0, F0);
               for I in 0 .. Natural (Q.Length) - 1 loop
                  if (Q (I).U - U0) ** 2 + (Q (I).V - V0) ** 2 < Bd then
                     Bd := (Q (I).U - U0) ** 2 + (Q (I).V - V0) ** 2; Best := I;
                  end if;
               end loop;
               Col (Best) := 'r';
            end;
         end loop;
         Put ("交成的点离面(档 = " & Codec.Fmt (Tol, 4) & ",下限 → 个数):");
         for B in Bins'Range loop
            if Bins (B) > 0 then
               Put (" " & Codec.Fmt (Long_Float (B) * Tol, 3) & "→" & Codec.Img (Bins (B)));
            end if;
         end loop;
         New_Line;
         for A of Above loop
            declare
               Cn : constant Geom.V3 := Geom.Ap (A.Cov, [0.0, 0.0, 1.0]);
            begin
               Put_Line ("  高出面 " & Codec.Fmt (A.Pw (2), 3) & " ± " & Codec.Fmt (Sqrt (Long_Float'Max (0.0, Cn (2))), 3) & " @ (" & Codec.Fmt (A.Pw (0), 3) & ", "
                         & Codec.Fmt (A.Pw (1), 3) & ")");
            end;
         end loop;
         if Ada.Command_Line.Argument_Count >= 10 then
            declare
               Fo : SIO.File_Type;
               S : SIO.Stream_Access;
               Img : Buf := Rgb (G0);
               procedure Dot (U, V : Long_Float; R, Gg, B : U8) is
               begin
                  for Dy in -2 .. 2 loop
                     for Dx in -2 .. 2 loop
                        declare
                           X : constant Integer := Integer (U) + Dx;
                           Y : constant Integer := Integer (V) + Dy;
                        begin
                           if X >= 0 and then Y >= 0 and then X < W and then Y < H then
                              Img.Replace_Element (3 * (Y * W + X), R); Img.Replace_Element (3 * (Y * W + X) + 1, Gg); Img.Replace_Element (3 * (Y * W + X) + 2, B);
                           end if;
                        end;
                     end loop;
                  end loop;
               end Dot;
               Hdr : constant String := "P6" & ASCII.LF & Codec.Img (W) & " " & Codec.Img (H) & ASCII.LF & "255" & ASCII.LF;
            begin
               for I in 0 .. Natural (Q.Length) - 1 loop
                  case Col (I) is
                     when 'r' => Dot (Q (I).U, Q (I).V, 255, 0, 0);
                     when 'g' => Dot (Q (I).U, Q (I).V, 0, 255, 0);
                     when 'b' => Dot (Q (I).U, Q (I).V, 0, 0, 255);
                     when others => null;
                  end case;
               end loop;
               SIO.Create (Fo, SIO.Out_File, Ada.Command_Line.Argument (10));
               S := SIO.Stream (Fo);
               String'Write (S, Hdr);
               for P of Img loop
                  Character'Write (S, Character'Val (P));
               end loop;
               SIO.Close (Fo);
            end;
         end if;
      end;
   end;
end Lookexam;
