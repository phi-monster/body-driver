--  离线重放接触集(大并行路 5,10-01):一炮落盘的一帧 + 那一拍的关节读数 + 这一炮装的身体文件,照驱动的路把接触集的输入重做一遍,
--  再跑驱动同一份 Act.Plan_Contact,打出它挑中的那一组。改接触集不用再开一炮(同 kinexam / lookexam 的用法)。
--  驱动的路,一段一段:
--   ① 身体:<身体>.json(按文件里存的钥匙装)、.geo.json(每台眼的几何、每一瓣的尖)、.kin.txt(运动学 + 板)⇒ Jointboot.Install ⇒ Pose_Hook
--     按那一拍的关节读数(vid/joints.txt)算腕眼位姿,同驱动当场;板的面 = 它躺的面(同 Note_Support 有板时:碰到的面就是板的面);
--   ② 东西是哪些像素:配点仪器分割(框 + 框里一个点,同 Seg_In_Box);
--   ③ 顶面点:轮廓像素隔几个取一个(同 Take_Silhouette:取到两千个上下)、各发一条视线落到它躺的面上
--     (碰过面 ⇒ 面过碰过的那一点、厚度当零,同 Plane_Point);
--     采样间距 = Contact.Gen.Sampling_Gap;预期误差 = 这只眼的像素残差 ÷ 焦距 × 眼到面的距离(同 Take_Silhouette);
--     我自己手指的像素这里不剔(仪器按框里那一点分出来的是那一块东西,不含手指;照实说);
--   ④ Act.Plan_Contact:够不够得着 = Plug.Reach(①装上的运动学);这件东西的摩擦没量过;旁边的东西没有(这一集还没被顶住过)。
--  身体文件没存每一瓣的尖(09-29 以前的)⇒ 可以给另一份同一只手的几何文件借尖:
--  按两份各自量的"眼到指尖中点"的距离之比换单位(比值照实印出来)。
--  打出:挑中那一组的接触点和法向、要的摩擦(按量到的法向 / 按误差最坏)、每单位重量的法向力之和、进场方向、合拢方向,
--  以及合拢方向和它顶面轮廓长轴的夹角(轮廓的主轴,按轮廓点在面里的协方差)。
--  用法:contactexam run_dir body.json seq cam arm name host port x0 y0 x1 y1 u v [lobes_geo.json]
with Ada.Command_Line; use Ada.Command_Line;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Streams.Stream_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Strings.Fixed;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Bytes; use Bytes;
with Codec;
with Json;
with Geom;
with Plug;
with Act;
with Bodyfile;
with Jointboot;
with Instrument;
with Contact;
with Contact.Gen;
with Contact.Search;
with Contact.Surface;
procedure Contactexam is
   package SIO renames Ada.Streams.Stream_IO;
   function Sub (A, B : Geom.V3) return Geom.V3 is ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]);
   function Add (A, B : Geom.V3) return Geom.V3 is ([A (0) + B (0), A (1) + B (1), A (2) + B (2)]);
   function Scl (K : Long_Float; A : Geom.V3) return Geom.V3 is ([K * A (0), K * A (1), K * A (2)]);
   function Read_All (Path : String) return String is
      Fi : File_Type;
      R : Unbounded_String;
   begin
      Open (Fi, In_File, Path);
      while not End_Of_File (Fi) loop
         Append (R, Get_Line (Fi));
         Append (R, ASCII.LF);
      end loop;
      Close (Fi);
      return To_String (R);
   end Read_All;
   --  P5 灰度图:头部三个数(宽、高、最大值)之后是原始字节
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
               N := N * 10 + (Character'Pos (C) - Character'Pos ('0'));
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
      Character'Read (S, C);
      W := Next_Num;
      H := Next_Num;
      declare
         Mx : constant Natural := Next_Num;
         pragma Unreferenced (Mx);
      begin
         null;
      end;
      G := U8_Vectors.Empty_Vector;
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
   function Pad6 (N : Natural) return String is
      S : constant String := Codec.Img (N);
   begin
      return [1 .. 6 - S'Length => '0'] & S;
   end Pad6;
   --  按空格切开
   function Fields (S : String) return Strs is
      R : Strs;
      I : Natural := S'First;
   begin
      while I <= S'Last loop
         while I <= S'Last and then (S (I) = ' ' or else S (I) = ASCII.HT) loop
            I := I + 1;
         end loop;
         exit when I > S'Last;
         declare
            J : Natural := I;
         begin
            while J <= S'Last and then S (J) /= ' ' and then S (J) /= ASCII.HT loop
               J := J + 1;
            end loop;
            R.Append (S (I .. J - 1));
            I := J;
         end;
      end loop;
      return R;
   end Fields;
   --  vid/joints.txt 的一行:帧号 画面号 | 第 0 组关节读数 | 第 1 组 | …(每组几个数由身体定)
   procedure Joints_At (Path : String; Seq : Natural; Js : out Plug.Floats_Vectors.Vector; Img : out Integer; Ok : out Boolean) is
      Fi : File_Type;
   begin
      Js.Clear; Img := -1; Ok := False;
      Open (Fi, In_File, Path);
      while not End_Of_File (Fi) and then not Ok loop
         declare
            Ln : constant String := Get_Line (Fi);
            Bar : Natural := Ada.Strings.Fixed.Index (Ln, "|");
         begin
            if Bar > 0 then
               declare
                  Hd : constant Strs := Fields (Ln (Ln'First .. Bar - 1));
               begin
                  if Natural (Hd.Length) >= 2 and then Natural'Value (Hd (0)) = Seq then
                     Img := Integer'Value (Hd (1));
                     while Bar > 0 loop
                        declare
                           Nx : constant Natural := Ada.Strings.Fixed.Index (Ln (Bar + 1 .. Ln'Last), "|");
                           Last : constant Natural := (if Nx > 0 then Nx - 1 else Ln'Last);
                           Fs : constant Strs := Fields (Ln (Bar + 1 .. Last));
                           G : Floats;
                        begin
                           for F of Fs loop
                              G.Append (Long_Float'Value (F));
                           end loop;
                           Js.Append (G);
                           Bar := Nx;
                        end;
                     end loop;
                     Ok := True;
                  end if;
               end;
            end if;
         end;
      end loop;
      Close (Fi);
   end Joints_At;
   function V (X : Geom.V3) return String is
     ("(" & Codec.Fmt (X (0), 3) & ", " & Codec.Fmt (X (1), 3) & ", " & Codec.Fmt (X (2), 3) & ")");
begin
   if Argument_Count < 14 then
      Put_Line ("用法:contactexam run_dir body.json seq cam arm name host port x0 y0 x1 y1 u v [lobes_geo.json]");
      return;
   end if;
   declare
      Run : constant String := Argument (1);
      Body_Path : constant String := Argument (2);
      Seq : constant Natural := Natural'Value (Argument (3));
      Cam : constant Natural := Natural'Value (Argument (4));
      Arm : constant Natural := Natural'Value (Argument (5));
      Name : constant Unbounded_String := To_Unbounded_String (Argument (6));
      Host : constant String := Argument (7);
      Port : constant Natural := Natural'Value (Argument (8));
      X0 : constant Integer := Integer'Value (Argument (9));
      Y0 : constant Integer := Integer'Value (Argument (10));
      X1 : constant Integer := Integer'Value (Argument (11));
      Y1 : constant Integer := Integer'Value (Argument (12));
      Pu : constant Long_Float := Long_Float'Value (Argument (13));
      Pv : constant Long_Float := Long_Float'Value (Argument (14));
      C : Act.Context;
      F : Plug.Frame;
      D : Json.Doc;
      Jerr, Note : Unbounded_String;
      Gnote : String (1 .. 200);
      K : Jointboot.Kin_Store;
      Kok, Jok : Boolean;
      Img : Integer;
      Gray : Buf;
      W, H : Natural;
   begin
      --  ① 身体
      if not Json.Parse (Read_All (Body_Path), D, Jerr) then
         Put_Line ("身体文件读不成:" & To_String (Jerr));
         return;
      end if;
      if not Bodyfile.Load (Body_Path, Json.Text (D, Json.Get (D, 0, "key")), C.Map, C.Hands, C.Tables, C.Sch, Note) then
         Put_Line ("身体文件装不上:" & To_String (Note));
         return;
      end if;
      Geom.Load (Body_Path & ".geo.json", C.Geo, C.Map.N_Cams, Gnote);
      Jointboot.Load_Kin (Body_Path & ".kin.txt", K, Kok, Note);
      if not Kok then
         Put_Line ("运动学装不上:" & To_String (Note));
         return;
      end if;
      Jointboot.Install (K.Worlds, K.Rw, K.O);
      C.Board_Plane := True; C.Board_Pt := K.Plane_Pt; C.Board_N := K.Plane_N; C.Board_Rms := K.Plane_Rms;
      C.Touch_Pt := K.Plane_Pt; C.Touch_N := K.Plane_N; C.Touch_Valid := True; C.Touch_Fresh := True;
      Put_Line ("身体:" & Codec.Img (C.Map.Arms) & " 只手、" & Codec.Img (C.Map.N_Cams) & " 台眼;它躺的面 = 板 过 " & V (K.Plane_Pt) & " 法向 " & V (K.Plane_N)
                & ";面内离散 " & Codec.Fmt (K.Plane_Rms, 4));
      if Cam >= Natural (C.Geo.Length) or else not C.Geo (Cam).Valid then
         Put_Line ("几何文件里第" & Codec.Img (Cam) & " 台眼没量过");
         return;
      end if;
      --  别的几何文件借尖(照实说换算)
      if Argument_Count >= 15 then
         declare
            Gs2 : Geom.Geo_Vectors.Vector;
            G : Geom.Cam_Geo := C.Geo (Cam);
         begin
            Geom.Load (Argument (15), Gs2, C.Map.N_Cams, Gnote);
            if Cam < Natural (Gs2.Length) and then not Gs2 (Cam).Lobes.Is_Empty and then Geom.Norm (Gs2 (Cam).Tip) > 0.0 then
               declare
                  S : constant Long_Float := Geom.Norm (G.Tip) / Geom.Norm (Gs2 (Cam).Tip);
               begin
                  G.Lobes.Clear;
                  for Lg of Gs2 (Cam).Lobes loop
                     G.Lobes.Append (Geom.Lobe_Geo'(Tip => [S * Lg.Tip (0), S * Lg.Tip (1), S * Lg.Tip (2)], Wide => S * Lg.Wide, Thin => S * Lg.Thin));
                  end loop;
                  G.Tip_Sd := S * Gs2 (Cam).Tip_Sd;
                  C.Geo.Replace_Element (Cam, G);
                  Put_Line ("每一瓣的尖从 " & Argument (15) & " 借:换单位按两份各自量的眼到指尖中点 " & Codec.Fmt (Geom.Norm (G.Tip), 4) & " / "
                            & Codec.Fmt (Geom.Norm (Gs2 (Cam).Tip), 4) & " = " & Codec.Fmt (S, 4) & "(两份的张口之比 "
                            & Codec.Fmt ((if Gs2 (Cam).Gap > 0.0 then G.Gap / Gs2 (Cam).Gap else 0.0), 4) & ")");
               end;
            else
               Put_Line ("借尖的那份几何文件第" & Codec.Img (Cam) & " 台眼也没有每一瓣的尖");
            end if;
         end;
      end if;
      Put_Line ("第" & Codec.Img (Cam) & " 台眼:" & Codec.Img (Natural (C.Geo (Cam).Lobes.Length)) & " 瓣;张口 " & Codec.Fmt (C.Geo (Cam).Gap, 4)
                & ";尖的误差 " & Codec.Fmt (C.Geo (Cam).Tip_Sd, 4));
      for Lg of C.Geo (Cam).Lobes loop
         Put_Line ("  瓣:尖 " & V (Lg.Tip) & " · 宽 " & Codec.Fmt (Lg.Wide, 4) & " · 厚 " & Codec.Fmt (Lg.Thin, 4));
      end loop;
      --  这一拍的关节读数 ⇒ 位姿(驱动的 Pose_Hook)
      Joints_At (Run & "/vid/joints.txt", Seq, F.Joints, Img, Jok);
      if not Jok or else Img < 0 then
         Put_Line ("vid/joints.txt 里没有第" & Codec.Img (Seq) & " 拍(或那一拍没有画面)");
         return;
      end if;
      F.Seq := Seq;
      Jointboot.Pose_Hook (F);
      declare
         Eye_Arm : Integer := -1;
      begin
         for A in 0 .. C.Map.Arms - 1 loop
            if A < Natural (C.Map.Cam_On_Arm.Length) and then C.Map.Cam_On_Arm (A) = Integer (Cam) then
               Eye_Arm := Integer (A);
            end if;
         end loop;
         if Eye_Arm < 0 or else Natural (Eye_Arm) >= Natural (F.EE.Length) then
            Put_Line ("第" & Codec.Img (Cam) & " 台眼不长在哪只手上(这个工具只重放腕眼取的轮廓)");
            return;
         end if;
         Read_PGM (Run & "/vid/f" & Pad6 (Natural (Img)) & "_c" & Codec.Img (Cam) & ".pgm", Gray, W, H);
         declare
            G : constant Geom.Cam_Geo := C.Geo (Cam);
            P : constant Plug.Arm_Pose := F.EE (Natural (Eye_Arm));
            Mask : Bools;
            Area : Natural;
            Score : Long_Float;
            Sok : Boolean;
            Err : Unbounded_String;
            Pts : Instrument.Seg_Pt_Vectors.Vector;
            Rays : Geom.Sight_Vectors.Vector;
            Top : Contact.V3_Vectors.Vector;
            Dropped : Natural;
            N_Px : Natural := 0;
            Stride : Positive := 1;
            Keep : constant := 2000;   --  同 Take_Silhouette(它的那个数,这里照抄才是重放)
            Eye_O : constant Geom.V3 := Geom.Cam_Pos (G, P);
         begin
            Put_Line ("第" & Codec.Img (Seq) & " 拍(第" & Codec.Img (Natural (Img)) & " 帧):眼在 " & V (Eye_O));
            Pts.Append (Instrument.Seg_Pt'(U => Pu, V => Pv, On => True));
            Instrument.Segment (Host, Port, Rgb (Gray), W, H, X0, Y0, X1, Y1, Pts, Mask, Area, Score, Sok, Err);
            if not Sok or else Area = 0 then
               Put_Line ("仪器没分出来:" & To_String (Err));
               return;
            end if;
            for I in 0 .. H - 1 loop
               for J in 0 .. W - 1 loop
                  if Mask (I * W + J) then
                     N_Px := N_Px + 1;
                  end if;
               end loop;
            end loop;
            Stride := Positive'Max (1, Positive (Long_Float'Ceiling (Sqrt (Long_Float (N_Px) / Long_Float (Keep)))));
            for I in 0 .. H - 1 loop
               if I mod Stride = 0 then
                  for J in 0 .. W - 1 loop
                     if J mod Stride = 0 and then Mask (I * W + J) then
                        Rays.Append (Geom.Sight'(O => Eye_O, D => Geom.Ray (G, P, Long_Float (J), Long_Float (I))));
                     end if;
                  end loop;
               end if;
            end loop;
            Contact.Surface.On_Plane (Rays, C.Touch_Pt, C.Touch_N, Top, Dropped);
            Put_Line ("仪器分出 " & Codec.Img (N_Px) & " 个像素(分数 " & Codec.Fmt (Score, 3) & ")· 隔 " & Codec.Img (Stride) & " 个取一个 ⇒ "
                      & Codec.Img (Natural (Top.Length)) & " 个顶面点(" & Codec.Img (Dropped) & " 条视线落不到面上)");
            if Natural (Top.Length) = 0 then
               return;
            end if;
            declare
               Pitch : constant Long_Float := Contact.Gen.Sampling_Gap (Top);
               Sp0 : constant Geom.V3 := Top.First_Element;
               Dist : constant Long_Float := Geom.Norm (Sub (Sp0, Eye_O));
            begin
               C.Sil_Pts := Top; C.Sil_Valid := True; C.Sil_Name := Name; C.Sil_Cam := Integer (Cam); C.Sil_N := C.Touch_N; C.Sil_Pitch := Pitch;
               C.Sil_Err := (if G.F > 0.0 then G.Rms * Dist / G.F else Dist);
               C.Sil_P0 := Sp0; C.Sil_Rays := Rays;
               Put_Line ("采样间距 " & Codec.Fmt (Pitch, 4) & " · 预期误差 " & Codec.Fmt (C.Sil_Err, 5) & " · 眼到面 " & Codec.Fmt (Dist, 3));
            end;
         end;
      end;
      --  ④ 驱动的接触集
      declare
         Pick : Contact.Search.Candidate;
         Ok : Boolean;
         G : constant Geom.Cam_Geo := C.Geo (Cam);
         Mx, My, Sxx, Sxy, Syy : Long_Float := 0.0;
         N : constant Geom.V3 := C.Touch_N;
         Seed : constant Geom.V3 := (if abs N (0) < abs N (1) then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);
         Oe : Boolean;
         E1 : constant Geom.V3 := Contact.Unit (Contact.Cross (N, Seed), Oe);
         E2 : constant Geom.V3 := Contact.Cross (N, E1);
         Np : constant Long_Float := Long_Float (C.Sil_Pts.Length);
      begin
         Act.Plan_Contact (C, F, Arm, Cam, Name, Pick, Note, Ok);
         Put_Line ("接触集:" & To_String (Note));
         if not Ok then
            return;
         end if;
         --  轮廓在面里的主轴
         for Q of C.Sil_Pts loop
            Mx := Mx + Contact.Dot (Q, E1) / Np; My := My + Contact.Dot (Q, E2) / Np;
         end loop;
         for Q of C.Sil_Pts loop
            declare
               A : constant Long_Float := Contact.Dot (Q, E1) - Mx;
               B : constant Long_Float := Contact.Dot (Q, E2) - My;
            begin
               Sxx := Sxx + A * A / Np; Sxy := Sxy + A * B / Np; Syy := Syy + B * B / Np;
            end;
         end loop;
         declare
            Th : constant Long_Float := 0.5 * Arctan (2.0 * Sxy, Sxx - Syy);
            Ax : constant Geom.V3 := [Cos (Th) * E1 (0) + Sin (Th) * E2 (0), Cos (Th) * E1 (1) + Sin (Th) * E2 (1), Cos (Th) * E1 (2) + Sin (Th) * E2 (2)];
            L1 : constant Long_Float := 0.5 * (Sxx + Syy) + Sqrt (0.25 * (Sxx - Syy) ** 2 + Sxy ** 2);
            L2 : constant Long_Float := 0.5 * (Sxx + Syy) - Sqrt (0.25 * (Sxx - Syy) ** 2 + Sxy ** 2);
            Oj : Boolean;
            Jaw_Eye : constant Geom.V3 := (if Natural (G.Lobes.Length) >= 2 then Contact.Unit (Sub (G.Lobes (1).Tip, G.Lobes (0).Tip), Oj) else [0.0, 0.0, 0.0]);
            Jw : constant Geom.V3 := Geom.Ap (Pick.R, Jaw_Eye);
            Jp : constant Geom.V3 := Sub (Jw, Scl (Contact.Dot (Jw, N), N));
            Ojp : Boolean;
            Jpu : constant Geom.V3 := Contact.Unit (Jp, Ojp);
            Ang : constant Long_Float := Arccos (Long_Float'Min (1.0, abs Contact.Dot (Jpu, Ax)));
         begin
            Put_Line ("顶面轮廓:" & Codec.Img (Natural (Np)) & " 个点,重心 " & V (Add (Scl (Mx, E1), Scl (My, E2))) & "(面内)· 长轴 " & V (Ax) & " · 长短两轴的散布 "
                      & Codec.Fmt (Sqrt (Long_Float'Max (0.0, L1)), 4) & " / " & Codec.Fmt (Sqrt (Long_Float'Max (0.0, L2)), 4));
            Put_Line ("挑中的那一组:眼到 " & V (Pick.T) & " · 进场 " & V (Pick.Approach) & "(离竖直 "
                      & Codec.Fmt (Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, -Contact.Dot (Pick.Approach, N)))), 3) & " rad)· 合拢方向 " & V (Jw)
                      & " · 它和轮廓长轴的夹角 " & Codec.Fmt (Ang, 3) & " rad(" & Codec.Fmt (Ang * 180.0 / Ada.Numerics.Pi, 1) & "°)");
            for T of Pick.Touches loop
               Put_Line ("  接触 " & V (T.P) & " · 法向(手往里推的方向)" & V (T.N) & " · 能拧的半径 " & Codec.Fmt (T.Twist_R, 4));
            end loop;
            Put_Line ("  两处相距 " & Codec.Fmt (Pick.Width, 4) & " · 中点离重心(水平)" & Codec.Fmt (Pick.Com_Off, 4) & " · 下去之前先合 " & Codec.Fmt (Pick.Pre, 4)
                      & " · 要的摩擦 " & Codec.Fmt (Pick.Mu_Nom, 4) & " / 最坏 " & Codec.Fmt (Pick.Mu_Worst, 4) & " · 每单位重量的法向力之和 " & Codec.Fmt (Pick.Squeeze, 4));
            Put_Line ("  (长度单位:这具身体的世界单位;一个指尖长 = " & Codec.Fmt (Act.Hand_Len (C), 4) & ")");
         end;
      end;
   end;
end Contactexam;
