with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Text_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Codec;
with Instrument;
with Picture;
with Stats;
package body Links is

   function Add (A, B : V3) return V3 is ([A (0) + B (0), A (1) + B (1), A (2) + B (2)]);
   function Sub (A, B : V3) return V3 is ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]);
   function Scl (A : V3; S : Long_Float) return V3 is ([A (0) * S, A (1) * S, A (2) * S]);
   function Dot (A, B : V3) return Long_Float is (A (0) * B (0) + A (1) * B (1) + A (2) * B (2));
   function ApT (A : M3; X : V3) return V3 is
     ([A (0, 0) * X (0) + A (1, 0) * X (1) + A (2, 0) * X (2),
       A (0, 1) * X (0) + A (1, 1) * X (1) + A (2, 1) * X (2),
       A (0, 2) * X (0) + A (1, 2) * X (1) + A (2, 2) * X (2)]);
   function Unit (A : V3) return V3 is
      N : constant Long_Float := Norm (A);
   begin
      return (if N > 0.0 then Scl (A, 1.0 / N) else A);
   end Unit;
   --  R · C · Rᵀ(协方差换系)
   function Turn_Cov (R : M3; C : M3) return M3 is (Mul (Mul (R, C), Tr (R)));
   function Scale_Cov (C : M3; S : Long_Float) return M3 is
      Out_C : M3 := C;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            Out_C (I, J) := C (I, J) * S * S;
         end loop;
      end loop;
      return Out_C;
   end Scale_Cov;
   function Along (C : M3; U : V3) return Long_Float is (Dot (U, [C (0, 0) * U (0) + C (0, 1) * U (1) + C (0, 2) * U (2),
                                                                  C (1, 0) * U (0) + C (1, 1) * U (1) + C (1, 2) * U (2),
                                                                  C (2, 0) * U (0) + C (2, 1) * U (1) + C (2, 2) * U (2)]));
   function Trace (C : M3) return Long_Float is (C (0, 0) + C (1, 1) + C (2, 2));

   --  世界 ↔ 这条臂的参照系(X_世界 = Rw · (S · Ra · X + Ta − O))
   function To_Model (Pl : Placement; Xw : V3) return V3 is
     (Scl (ApT (Pl.Ra, Add (Sub (ApT (Pl.Rw, Xw), Pl.Ta), Pl.O)), 1.0 / Pl.S));
   function Dir_To_Model (Pl : Placement; Dw : V3) return V3 is (ApT (Pl.Ra, ApT (Pl.Rw, Dw)));
   function To_World (Pl : Placement; Xm : V3) return V3 is (Ap (Pl.Rw, Sub (Add (Scl (Ap (Pl.Ra, Xm), Pl.S), Pl.Ta), Pl.O)));

   --  第 Link 节的位姿:只算到第 Link 个关节(更远的关节放在参照读数上 —— 它们转了这一节不动)
   procedure FK_To (M : Kinem.Model; Q : Floats; Link : Natural; R : out M3; T : out V3) is
      Qt : Floats := Q;
   begin
      for J in Link + 1 .. Natural (Qt.Length) - 1 loop
         if J < Natural (M.Q0.Length) then
            Qt.Replace_Element (J, M.Q0 (J));
         end if;
      end loop;
      Kinem.FK (M, Qt, R, T);
   end FK_To;

   function Median (V : in out Floats) return Long_Float is (Picture.Quantile (V, 0.5));
   --  最后一次三角:挪过、交得出(在眼前面)的格点有几个(进第二遍之前),背景那一份配点噪声多少(Measure 的报告印)
   Last_Moved : Natural := 0;
   Last_Static : Long_Float := 0.0;

   function Track_Noise (Tracks : Track_Vectors.Vector; N_Cells : Natural) return Long_Float is
      Per_Cell : Floats;
   begin
      for K in 0 .. N_Cells - 1 loop
         declare
            D : Floats;
         begin
            for Tr of Tracks loop
               if K < Natural (Tr.U.Length) and then Tr.U (K) >= 0.0 then
                  D.Append (Sqrt ((Tr.U (K) - Tr.U0) ** 2 + (Tr.V (K) - Tr.V0) ** 2));
               end if;
            end loop;
            --  这一格里大半格点是不动的背景:挪了多少的中位 ÷ 瑞利中位 = 每轴 σ(二维正态误差的长度服从瑞利分布)
            if Natural (D.Length) > 1 then
               Per_Cell.Append (Median (D) / Stats.Rayleigh_Median);
            end if;
         end;
      end loop;
      return (if Per_Cell.Is_Empty then 0.0 else Median (Per_Cell));
   end Track_Noise;

   procedure Triangulate (Pls : Placement_Vectors.Vector; G : Cam_Geo; W : Natural; Cells : Cell_Vectors.Vector; Tracks : Track_Vectors.Vector;
                          Pts : out Link_Pt_Vectors.Vector; Sd_Used : out Long_Float) is
      --  挪没挪:不动的背景那一份配点噪声(大半格点是背景;它们配得最准)
      Sig_Static : constant Long_Float := Track_Noise (Tracks, Natural (Cells.Length));
      Step_Px : constant Long_Float := Long_Float (W) / Long_Float (Kinem.Gx);   --  相邻两个格点隔几像素(采样密度)
      --  一个格点在一条臂的一种假设下交出来的点(第一遍:每个格点留离各条视线最近的那个假设)
      type Cand is record
         Arm, Link : Natural := 0;
         P : V3 := [0.0, 0.0, 0.0];
         Rays : Sight_Vectors.Vector;
         Res_Px : Floats;                       --  每条视线离交点多远(折成像素:角度 × 焦距)
         Worst_Px : Long_Float := Long_Float'Last;
         Range_M : Long_Float := 0.0;
      end record;
      package Cand_Vectors is new Ada.Containers.Vectors (Natural, Cand);
      Best_Of : Cand_Vectors.Vector;
      Sig : Long_Float;
   begin
      Pts := Link_Pt_Vectors.Empty_Vector;
      Sd_Used := 0.0;
      if not G.Valid or else G.F <= 0.0 or else Sig_Static <= 0.0 then
         return;
      end if;
      --  ── 第一遍:每个挪过的格点、每条臂的假设各交一次,留离各条视线最近的那个 ──
      for Tr of Tracks loop
         declare
            Ok0 : Boolean;
            D0w : constant V3 := Ray_Fixed (G, Tr.U0, Tr.V0, Ok0);
            Best : Cand;
            Found : Boolean := False;
            function Moved (K : Natural) return Boolean is
              (K < Natural (Tr.U.Length) and then Tr.U (K) >= 0.0
               and then Sqrt ((Tr.U (K) - Tr.U0) ** 2 + (Tr.V (K) - Tr.V0) ** 2) > Stats.Z * Sig_Static);
         begin
            if Ok0 then
               for A in 0 .. Natural (Pls.Length) - 1 loop
                  if Pls (A).Valid then
                     declare
                        Pl : constant Placement := Pls (A);
                        L_Max : Integer := -1;
                     begin
                        --  一转它就动的关节里最远的那一个(只看这条臂那一拍在单独扫的那个关节)
                        for K in 0 .. Natural (Cells.Length) - 1 loop
                           if A < Natural (Cells (K).Joints.Length) and then Cells (K).Joints (A) >= 0 and then Moved (K) then
                              L_Max := Integer'Max (L_Max, Cells (K).Joints (A));
                           end if;
                        end loop;
                        if L_Max >= 0 and then L_Max < Pl.Model.N then
                           declare
                              C : Cand;
                              Om : constant V3 := To_Model (Pl, G.Pos);
                              Okm : Boolean;
                              Spread : Long_Float;
                              Ahead : Boolean := True;
                           begin
                              C.Arm := A; C.Link := Natural (L_Max);
                              C.Rays.Append (Sight'(O => Om, D => Unit (Dir_To_Model (Pl, D0w))));
                              for K in 0 .. Natural (Cells.Length) - 1 loop
                                 if K < Natural (Tr.U.Length) and then Tr.U (K) >= 0.0 and then A < Natural (Cells (K).Qs.Length)
                                   and then Natural (Cells (K).Qs (A).Length) = Pl.Model.N
                                 then
                                    declare
                                       Okk : Boolean;
                                       Dkw : constant V3 := Ray_Fixed (G, Tr.U (K), Tr.V (K), Okk);
                                       R : M3;
                                       T : V3;
                                    begin
                                       if Okk then
                                          --  这一格这一节的位姿 X_k = R · X + T ⇒ 这一格的视线搬回起点那一刻:起点 Rᵀ (Om − T)、方向 Rᵀ D
                                          FK_To (Pl.Model, Cells (K).Qs (A), Natural (L_Max), R, T);
                                          C.Rays.Append (Sight'(O => ApT (R, Sub (Om, T)), D => Unit (ApT (R, Dir_To_Model (Pl, Dkw)))));
                                       end if;
                                    end;
                                 end if;
                              end loop;
                              C.P := Meet (C.Rays, Okm, Spread);
                              if Okm then
                                 C.Worst_Px := 0.0;
                                 for Ry of C.Rays loop
                                    declare
                                       Wv : constant V3 := Sub (C.P, Ry.O);
                                       Tt : constant Long_Float := Dot (Wv, Ry.D);
                                    begin
                                       if Tt <= 0.0 then
                                          Ahead := False;
                                       else
                                          --  这条视线离交点多远,折成像素(角度 × 焦距)
                                          C.Res_Px.Append (G.F * Norm (Sub (Wv, Scl (Ry.D, Tt))) / Tt);
                                          C.Worst_Px := Long_Float'Max (C.Worst_Px, C.Res_Px.Last_Element);
                                       end if;
                                    end;
                                 end loop;
                                 C.Range_M := Norm (Sub (C.P, Om));
                                 if Ahead and then C.Worst_Px < Best.Worst_Px then
                                    Best := C; Found := True;
                                 end if;
                              end if;
                           end;
                        end if;
                     end;
                  end if;
               end loop;
            end if;
            if Found then
               Best_Of.Append (Best);
            end if;
         end;
      end loop;
      --  ── 跟着身体动的点配得没有背景准(配的是一块在转的东西),运动学自己也有误差:这一份噪声从这批点自己量 ——
      --  每个格点最好的那个假设下,各条视线离交点多远(像素)的中位 ÷ 瑞利中位 = 每轴 σ;配点仪器配背景的那一份是它的下限 ──
      declare
         All_Res : Floats;
      begin
         for C of Best_Of loop
            for R of C.Res_Px loop
               All_Res.Append (R);
            end loop;
         end loop;
         Sig := (if Natural (All_Res.Length) > 1 then Long_Float'Max (Sig_Static, Median (All_Res) / Stats.Rayleigh_Median) else Sig_Static);
      end;
      Sd_Used := Sig;
      --  ── 第二遍:离每条视线都在 Stats.Z 倍噪声以内、远近定得住(沿起点那条视线的不确定度不比远近本身大)才收 ──
      declare
         Ang : constant Long_Float := Sig / G.F;   --  一条视线的角度噪声(弧度)
      begin
         for C of Best_Of loop
            if C.Worst_Px <= Stats.Z * Sig then
               declare
                  Sds : Floats;
                  Okc : Boolean;
                  Cv : M3;
               begin
                  for I in 1 .. Natural (C.Rays.Length) loop
                     Sds.Append (Ang);
                  end loop;
                  Cv := Meet_Cov (C.Rays, Sds, C.P, Okc);
                  if Okc and then Sqrt (Along (Cv, C.Rays (0).D)) < C.Range_M then
                     Pts.Append (Link_Pt'(Arm => C.Arm, Link => C.Link, P => C.P, Cov => Cv, Views => Natural (C.Rays.Length),
                                          Spacing => C.Range_M * Step_Px / G.F));
                  end if;
               end;
            end if;
         end loop;
      end;
      Last_Moved := Natural (Best_Of.Length);
      Last_Static := Sig_Static;
   end Triangulate;

   --  ── 开机扫描攒下的 ──
   Sw_World_Id : Integer := -1;
   Sw_W, Sw_H : Natural := 0;
   Sw_Ids : Ints;
   Sw_Cells : Cell_Vectors.Vector;
   Sw_Tracks : Track_Vectors.Vector;

   procedure Sweep_Begin (World_Id : Integer; W, H : Natural) is
   begin
      Sw_World_Id := World_Id; Sw_W := W; Sw_H := H;
      Sw_Ids.Clear; Sw_Cells.Clear; Sw_Tracks.Clear;
   end Sweep_Begin;

   function Sweep_On return Boolean is (Sw_World_Id >= 0 and then Sw_W > 0 and then Sw_H > 0);

   function Last_Seq return Integer is (if Sw_Cells.Is_Empty then -1 else Integer (Sw_Cells.Last_Element.Seq));

   procedure Sweep_Cell (Id : Integer; C : Cell) is
   begin
      Sw_Ids.Append (Id); Sw_Cells.Append (C);
   end Sweep_Cell;

   procedure Sweep_Set_Joint (Arm : Natural; Joint : Integer) is
   begin
      if not Sw_Cells.Is_Empty then
         declare
            C : Cell := Sw_Cells.Last_Element;
         begin
            while Natural (C.Joints.Length) <= Arm loop
               C.Joints.Append (-1);
            end loop;
            C.Joints.Replace_Element (Arm, Joint);
            Sw_Cells.Replace_Element (Natural (Sw_Cells.Length) - 1, C);
         end;
      end if;
   end Sweep_Set_Joint;

   procedure Sweep_Repair (Seq_Of : access function (Seq : Natural) return Plug.Floats_Vectors.Vector; Lag : Integer; Groups : Ints) is
   begin
      for I in 0 .. Natural (Sw_Cells.Length) - 1 loop
         declare
            C : Cell := Sw_Cells (I);
            Qa : constant Plug.Floats_Vectors.Vector := Seq_Of (Natural (Integer'Max (0, Integer (C.Seq) - Lag)));
         begin
            for A in 0 .. Natural'Min (Natural (Groups.Length), Natural (C.Qs.Length)) - 1 loop
               if Groups (A) >= 0 and then Natural (Groups (A)) < Natural (Qa.Length) then
                  C.Qs.Replace_Element (A, Qa (Natural (Groups (A))));
               end if;
            end loop;
            Sw_Cells.Replace_Element (I, C);
         end;
      end loop;
   end Sweep_Repair;

   procedure Sweep_Match (Host : String; Port : Natural; Note : out Unbounded_String) is
      Q : Instrument.Match_Vectors.Vector;
      Pairs, Got : Natural := 0;
   begin
      Note := Null_Unbounded_String;
      Sw_Tracks.Clear;
      if not Sweep_On then
         return;
      end if;
      for Gyy in 0 .. Kinem.Gy - 1 loop
         for Gxx in 0 .. Kinem.Gx - 1 loop
            Q.Append (Instrument.Match_Pt'(U => Kinem.Grid_U (Gxx, Sw_W), V => Kinem.Grid_V (Gyy, Sw_H), others => <>));
            Sw_Tracks.Append (Track'(U0 => Kinem.Grid_U (Gxx, Sw_W), V0 => Kinem.Grid_V (Gyy, Sw_H), U => F64_Vectors.Empty_Vector, V => F64_Vectors.Empty_Vector));
         end loop;
      end loop;
      for K in 0 .. Natural (Sw_Cells.Length) - 1 loop
         declare
            Err : Unbounded_String;
            R : constant Instrument.Match_Vectors.Vector :=
              (if Sw_Ids (K) >= 0 then Instrument.Match_Ids (Host, Port, Natural (Sw_World_Id), Natural (Sw_Ids (K)), Q, Err, Coarse => True, Back => True)
               else Instrument.Match_Vectors.Empty_Vector);
            Full : constant Boolean := Natural (R.Length) = Natural (Q.Length);
         begin
            if Full then
               Pairs := Pairs + 1;
            end if;
            for G in 0 .. Natural (Sw_Tracks.Length) - 1 loop
               declare
                  Tr : Track := Sw_Tracks (G);
                  --  往返 1 px 以内、落在画面里才算配上(同扫描配点)
                  Hit : constant Boolean := Full and then R (G).Bu >= 0.0 and then R (G).U >= 0.0 and then R (G).U < Long_Float (Sw_W)
                    and then R (G).V >= 0.0 and then R (G).V < Long_Float (Sw_H)
                    and then Norm ([R (G).Bu - Q (G).U, R (G).Bv - Q (G).V, 0.0]) < Trip_Px;
               begin
                  Tr.U.Append (if Hit then R (G).U else -1.0);
                  Tr.V.Append (if Hit then R (G).V else -1.0);
                  if Hit then
                     Got := Got + 1;
                  end if;
                  Sw_Tracks.Replace_Element (G, Tr);
               end;
            end loop;
         end;
      end loop;
      Note := To_Unbounded_String ("不动的眼:起点那一张的 " & Codec.Img (Natural (Q.Length)) & " 个格点配进 " & Codec.Img (Pairs) & " / "
                                   & Codec.Img (Natural (Sw_Cells.Length)) & " 格(共配上 " & Codec.Img (Got) & " 笔)");
   end Sweep_Match;

   function Sweep_Cells return Cell_Vectors.Vector is (Sw_Cells);
   function Sweep_Tracks return Track_Vectors.Vector is (Sw_Tracks);

   --  格式:第一行 "sweep 起点编号 画幅宽 画幅高 格数 格点数";每一格一行 "cell 帧号 臂数 [扫的关节 读数个数 读数…]×臂数";
   --  每个格点一行 "track u0 v0 [u v]×格数"
   procedure Dump_Sweep (Path : String) is
      use Ada.Text_IO;
      Fo : File_Type;
   begin
      Create (Fo, Out_File, Path);
      Put_Line (Fo, "sweep " & Integer'Image (Sw_World_Id) & " " & Codec.Img (Sw_W) & " " & Codec.Img (Sw_H) & " " & Codec.Img (Natural (Sw_Cells.Length))
                & " " & Codec.Img (Natural (Sw_Tracks.Length)));
      for C of Sw_Cells loop
         Put (Fo, "cell " & Codec.Img (C.Seq) & " " & Codec.Img (Natural (C.Qs.Length)));
         for A in 0 .. Natural (C.Qs.Length) - 1 loop
            Put (Fo, " " & Integer'Image (if A < Natural (C.Joints.Length) then C.Joints (A) else -1) & " " & Codec.Img (Natural (C.Qs (A).Length)));
            for X of C.Qs (A) loop
               Put (Fo, " " & Codec.Fmt (X, 9));
            end loop;
         end loop;
         New_Line (Fo);
      end loop;
      for T of Sw_Tracks loop
         Put (Fo, "track " & Codec.Fmt (T.U0, 3) & " " & Codec.Fmt (T.V0, 3));
         for K in 0 .. Natural (T.U.Length) - 1 loop
            Put (Fo, " " & Codec.Fmt (T.U (K), 3) & " " & Codec.Fmt (T.V (K), 3));
         end loop;
         New_Line (Fo);
      end loop;
      Close (Fo);
   exception
      when others =>
         if Is_Open (Fo) then
            Close (Fo);
         end if;
   end Dump_Sweep;


   --  ── 装上的 ──
   In_Pls : Placement_Vectors.Vector;
   In_Pts : Link_Pt_Vectors.Vector;

   procedure Install (Pls : Placement_Vectors.Vector; Pts : Link_Pt_Vectors.Vector) is
   begin
      In_Pls := Pls; In_Pts := Pts;
   end Install;

   procedure Set_Points (Pts : Link_Pt_Vectors.Vector) is
   begin
      In_Pts := Pts;
   end Set_Points;

   procedure Place (Pls : Placement_Vectors.Vector) is
   begin
      In_Pls := Pls;
   end Place;

   function Points return Link_Pt_Vectors.Vector is (In_Pts);

   function Line_Of (P : Link_Pt) return String is
      R : Unbounded_String := To_Unbounded_String (Codec.Img (P.Arm) & " " & Codec.Img (P.Link));
   begin
      for I in 0 .. 2 loop
         Append (R, " " & Codec.Fmt (P.P (I), 9));
      end loop;
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            Append (R, " " & Codec.Fmt (P.Cov (I, J), 12));
         end loop;
      end loop;
      Append (R, " " & Codec.Img (P.Views) & " " & Codec.Fmt (P.Spacing, 9));
      return To_String (R);
   end Line_Of;

   function From_Fields (F : Bytes.Strs; First : Natural; P : out Link_Pt) return Boolean is
      --  臂 节 + 位置 3 + 协方差 9 + 视线数 + 采样间距(格式)
      N_Fields : constant := 16;
      function V (I : Natural) return Long_Float is (Long_Float'Value (F (First + I)));
   begin
      P := (others => <>);
      if Natural (F.Length) < First + N_Fields then
         return False;
      end if;
      P.Arm := Natural'Value (F (First)); P.Link := Natural'Value (F (First + 1));
      P.P := [V (2), V (3), V (4)];
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            P.Cov (I, J) := V (5 + 3 * I + J);
         end loop;
      end loop;
      P.Views := Natural'Value (F (First + 14)); P.Spacing := V (15);
      return True;
   exception
      when others =>
         return False;
   end From_Fields;

   procedure Measure (Pls : Placement_Vectors.Vector; G : Cam_Geo; Note : out Unbounded_String) is
      Pts : Link_Pt_Vectors.Vector;
      Sd : Long_Float;
   begin
      Note := Null_Unbounded_String;
      if not Sweep_On or else Sw_Tracks.Is_Empty or else not G.Valid then
         Install (Pls, Pts);
         Note := To_Unbounded_String ("[身] 📐 每一节的形状:" & (if not G.Valid then "没有解出来的不动的眼" else "扫描时不动的眼没配上点")
                                      & " ⇒ 量不了(净空说不出、画面里哪些是自己说不出)" & ASCII.LF);
         return;
      end if;
      Triangulate (Pls, G, Sw_W, Sw_Cells, Sw_Tracks, Pts, Sd);
      Install (Pls, Pts);
      Append (Note, "[身] 📐 每一节的形状(不动的眼看着各臂一个关节一个关节转,跟着哪一节动的点就是那一节的表面点):挪过、交得出的格点 "
              & Codec.Img (Last_Moved) & " 个,收下 " & Codec.Img (Natural (Pts.Length)) & " 个 · 配点噪声:背景 " & Codec.Fmt (Last_Static, 3)
              & " px、跟着身体动的点 " & Codec.Fmt (Sd, 3) & " px(按它定门)");
      for A in 0 .. Natural (Pls.Length) - 1 loop
         if Pls (A).Valid then
            Append (Note, " · 第" & Codec.Img (A + 1) & " 条臂");
            for Lk in 0 .. Pls (A).Model.N - 1 loop
               declare
                  N : Natural := 0;
                  S : Floats;
               begin
                  for P of Pts loop
                     if P.Arm = A and then P.Link = Lk then
                        N := N + 1;
                        S.Append (Pls (A).S * Sqrt (Trace (P.Cov)));
                     end if;
                  end loop;
                  Append (Note, " 第" & Codec.Img (Lk) & " 节 " & Codec.Img (N) & " 点" & (if N > 0 then "(±" & Codec.Fmt (Median (S), 4) & ")" else ""));
               end;
            end loop;
         end if;
      end loop;
      Append (Note, ASCII.LF);
   end Measure;

   procedure World_Of (Pl : Placement; Q : Floats; Pt : Link_Pt; X : out V3; Cov : out M3) is
      R : M3;
      T : V3;
   begin
      FK_To (Pl.Model, Q, Pt.Link, R, T);
      X := To_World (Pl, Add (Ap (R, Pt.P), T));
      --  参照系 → 此刻这一节 → 世界:协方差跟着转,长度按 S 缩放
      Cov := Scale_Cov (Turn_Cov (Mul (Pl.Rw, Mul (Pl.Ra, R)), Pt.Cov), Pl.S);
   end World_Of;

   --  此刻的障碍(场景点 + 别的臂的表面点),世界系
   type Obstacle is record
      X : V3;
      Cov : M3;
      Arm : Integer := -1;   --  -1 = 场景点
   end record;
   package Obstacle_Vectors is new Ada.Containers.Vectors (Natural, Obstacle);
   function Obstacles (Arm : Natural; Qs : Plug.Floats_Vectors.Vector; Scene : Scene_Pt_Vectors.Vector) return Obstacle_Vectors.Vector is
      R : Obstacle_Vectors.Vector;
   begin
      for S of Scene loop
         R.Append (Obstacle'(X => S.Pw, Cov => S.Cov, Arm => -1));
      end loop;
      for P of In_Pts loop
         if P.Arm /= Arm and then P.Arm < Natural (In_Pls.Length) and then P.Arm < Natural (Qs.Length) and then In_Pls (P.Arm).Valid then
            declare
               X : V3;
               C : M3;
            begin
               World_Of (In_Pls (P.Arm), Qs (P.Arm), P, X, C);
               R.Append (Obstacle'(X => X, Cov => C, Arm => Integer (P.Arm)));
            end;
         end if;
      end loop;
      return R;
   end Obstacles;

   function Clear_Of (Arm : Natural; Qs : Plug.Floats_Vectors.Vector; Scene : Scene_Pt_Vectors.Vector) return Clearance is
      C : Clearance;
      Best : Long_Float := Long_Float'Last;
   begin
      if Arm >= Natural (In_Pls.Length) or else Arm >= Natural (Qs.Length) or else not In_Pls (Arm).Valid then
         return C;
      end if;
      declare
         Obs : constant Obstacle_Vectors.Vector := Obstacles (Arm, Qs, Scene);
      begin
         for P of In_Pts loop
            if P.Arm = Arm then
               C.Valid := True;
               declare
                  X : V3;
                  Cx : M3;
               begin
                  World_Of (In_Pls (Arm), Qs (Arm), P, X, Cx);
                  for Ob of Obs loop
                     declare
                        Dv : constant V3 := Sub (X, Ob.X);
                        D : constant Long_Float := Norm (Dv);
                        U : constant V3 := Unit (Dv);
                        Sd : constant Long_Float := Sqrt (Long_Float'Max (0.0, Along (Cx, U) + Along (Ob.Cov, U)));
                     begin
                        if D - Stats.Z * Sd < Best then
                           Best := D - Stats.Z * Sd;
                           C.Dist := D; C.Sd := Sd; C.To_Scene := Ob.Arm < 0; C.Other_Arm := Ob.Arm; C.Link := Integer (P.Link);
                        end if;
                     end;
                  end loop;
               end;
            end if;
         end loop;
      end;
      return C;
   end Clear_Of;

   function Free_Along (Arm : Natural; Qs : Plug.Floats_Vectors.Vector; Dir : V3; Scene : Scene_Pt_Vectors.Vector; Known : out Boolean) return Long_Float is
      Best : Long_Float := Long_Float'Last;
      D_U : constant V3 := Unit (Dir);
   begin
      Known := False;
      if Arm >= Natural (In_Pls.Length) or else Arm >= Natural (Qs.Length) or else not In_Pls (Arm).Valid then
         return Best;
      end if;
      declare
         Obs : constant Obstacle_Vectors.Vector := Obstacles (Arm, Qs, Scene);
      begin
         for P of In_Pts loop
            if P.Arm = Arm then
               Known := True;
               declare
                  X : V3;
                  Cx : M3;
               begin
                  World_Of (In_Pls (Arm), Qs (Arm), P, X, Cx);
                  for Ob of Obs loop
                     declare
                        --  带子的半径 = Stats.Z 倍这一对的不确定度(各方向都按最不准的那个总量算:迹的平方根不小于任何一个方向的标准差)
                        Rb : constant Long_Float := Stats.Z * Sqrt (Long_Float'Max (0.0, Trace (Cx) + Trace (Ob.Cov)));
                        Wv : constant V3 := Sub (Ob.X, X);
                        Tc : constant Long_Float := Dot (Wv, D_U);
                        D2 : constant Long_Float := Dot (Wv, Wv) - Tc * Tc;
                     begin
                        if Norm (Wv) <= Rb then
                           Best := 0.0;   --  已经在带子里
                        elsif Tc > 0.0 and then D2 < Rb * Rb then
                           Best := Long_Float'Min (Best, Tc - Sqrt (Rb * Rb - D2));
                        end if;
                     end;
                  end loop;
               end;
            end if;
         end loop;
      end;
      return Best;
   end Free_Along;

   function Readings_Now (F : Plug.Frame) return Plug.Floats_Vectors.Vector is
      R : Plug.Floats_Vectors.Vector;
   begin
      for Pl of In_Pls loop
         R.Append (if Pl.Group < Natural (F.Joints.Length) then F.Joints (Pl.Group) else F64_Vectors.Empty_Vector);
      end loop;
      return R;
   end Readings_Now;

   function Self_Mask_Now (F : Plug.Frame; Cam : Natural; Geo : Cam_Geo; W, H : Natural) return Bools is
     (Self_Mask_At (Readings_Now (F), Cam, Geo, W, H));

   function Reference_Readings return Plug.Floats_Vectors.Vector is
      R : Plug.Floats_Vectors.Vector;
   begin
      for Pl of In_Pls loop
         R.Append (Pl.Model.Q0);
      end loop;
      return R;
   end Reference_Readings;

   function Has_Shape (Arm : Natural) return Boolean is (for some P of In_Pts => P.Arm = Arm);

   function Self_Mask_At (Qs : Plug.Floats_Vectors.Vector; Cam : Natural; Geo : Cam_Geo; W, H : Natural) return Bools is
      Pose : Plug.Arm_Pose := [others => 0.0];
      Placed : Boolean := Geo.Fixed;
   begin
      if not Geo.Fixed then
         for A in 0 .. Natural (In_Pls.Length) - 1 loop
            if In_Pls (A).Valid and then In_Pls (A).Eye = Integer (Cam) and then A < Natural (Qs.Length)
              and then Natural (Qs (A).Length) = In_Pls (A).Model.N
            then
               declare
                  Pl : constant Placement := In_Pls (A);
                  R : M3;
                  T : V3;
               begin
                  --  这只眼此刻的位姿 = 这条臂此刻的运动学放进世界(同装上以后插头算手的位姿:眼就是手,不转不偏)
                  Kinem.FK (Pl.Model, Qs (A), R, T);
                  Pose := Kinem.To_Pose (Mul (Pl.Rw, Mul (Pl.Ra, R)), To_World (Pl, T));
                  Placed := True;
               end;
            end if;
         end loop;
      end if;
      if not Placed then
         return Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (W * H));
      end if;
      return Self_Mask (Geo, Pose, W, H, Qs);
   end Self_Mask_At;

   function Self_Mask (Geo : Cam_Geo; Pose : Plug.Arm_Pose; W, H : Natural; Qs : Plug.Floats_Vectors.Vector) return Bools is
      M : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (W * H));
   begin
      if W = 0 or else H = 0 or else Geo.F <= 0.0 then
         return M;
      end if;
      for P of In_Pts loop
         if P.Arm < Natural (In_Pls.Length) and then P.Arm < Natural (Qs.Length) and then In_Pls (P.Arm).Valid then
            declare
               X : V3;
               C : M3;
               U, V : Long_Float;
               Front : Boolean;
               Eye : constant V3 := (if Geo.Fixed then Geo.Pos else Cam_Pos (Geo, Pose));
            begin
               World_Of (In_Pls (P.Arm), Qs (P.Arm), P, X, C);
               if Geo.Fixed then
                  Project_Fixed (Geo, X, U, V, Front);
               else
                  Project (Geo, Pose, X, U, V, Front);
               end if;
               if Front then
                  declare
                     Dist : constant Long_Float := Norm (Sub (X, Eye));
                     --  这个点代表它周围一个采样间距那么大一片(世界单位 = 这条臂的模型单位 × S):在这只眼里画它一半那么大的圆
                     Rpx : constant Long_Float := (if Dist > 0.0 then 0.5 * In_Pls (P.Arm).S * P.Spacing * Geo.F / Dist else 0.0);
                     Ui : constant Integer := Integer (Long_Float'Floor (U));
                     Vi : constant Integer := Integer (Long_Float'Floor (V));
                     X0 : constant Integer := Integer (Long_Float'Floor (U - Rpx));
                     X1 : constant Integer := Integer (Long_Float'Floor (U + Rpx));
                     Y0 : constant Integer := Integer (Long_Float'Floor (V - Rpx));
                     Y1 : constant Integer := Integer (Long_Float'Floor (V + Rpx));
                  begin
                     for Y in Integer'Max (0, Y0) .. Integer'Min (H - 1, Y1) loop
                        for Xx in Integer'Max (0, X0) .. Integer'Min (W - 1, X1) loop
                           --  这个点落在的那一格,和离它不到半个采样间距的那些格
                           if (Xx = Ui and then Y = Vi) or else (Long_Float (Xx) - U) ** 2 + (Long_Float (Y) - V) ** 2 <= Rpx * Rpx then
                              M.Replace_Element (Natural (Y) * W + Natural (Xx), True);
                           end if;
                        end loop;
                     end loop;
                  end;
               end if;
            end;
         end if;
      end loop;
      return M;
   end Self_Mask;

   function Max_Shift (Arm : Natural; Q0, Q1 : Floats; Known : out Boolean) return Long_Float is
      Best : Long_Float := 0.0;
   begin
      Known := False;
      if Arm >= Natural (In_Pls.Length) or else not In_Pls (Arm).Valid or else not Has_Shape (Arm) then
         return Long_Float'Last;
      end if;
      declare
         Pl : constant Placement := In_Pls (Arm);
         M : Kinem.Model renames Pl.Model;
         N : constant Natural := M.N;
         function Q_Of (V : Floats; J : Natural) return Long_Float is
           (if J < Natural (V.Length) then V (J) elsif J < Natural (M.Q0.Length) then M.Q0 (J) else 0.0);
         function Ref (J : Natural) return Long_Float is (if J < Natural (M.Q0.Length) then M.Q0 (J) else 0.0);
         --  这一段关节直线上第 J 个关节离参照读数最远多远(直线 ⇒ 两头之一)
         function Off_Ref (J : Natural) return Long_Float is
           (Long_Float'Max (abs (Q_Of (Q0, J) - Ref (J)), abs (Q_Of (Q1, J) - Ref (J))));
         --  第 K 根转轴上那一点:这根轴往外那几节(Link ≥ K)表面点的形心在轴上的垂足;一个点都没有 ⇒ 轴上存的那一点
         function Anchor (K : Natural) return V3 is
            Sum : V3 := [0.0, 0.0, 0.0];
            Cnt : Natural := 0;
            A : constant Kinem.Axis := M.Ax (K);
            Wu : constant V3 := Unit (A.W);
         begin
            for P of In_Pts loop
               if P.Arm = Arm and then P.Link >= K then
                  Sum := Add (Sum, P.P); Cnt := Cnt + 1;
               end if;
            end loop;
            if Cnt = 0 then
               return A.P;
            end if;
            declare
               C : constant V3 := Scl (Sum, 1.0 / Long_Float (Cnt));
            begin
               return Add (A.P, Scl (Wu, Dot (Sub (C, A.P), Wu)));
            end;
         end Anchor;
         type V3_Arr is array (0 .. Natural'Max (1, N) - 1) of V3;
         Anc : V3_Arr;
      begin
         for K in 0 .. N - 1 loop
            Anc (K) := (if M.Ax (K).Slide then [0.0, 0.0, 0.0] else Anchor (K));
         end loop;
         for J in 0 .. N - 1 loop
            declare
               Dq : constant Long_Float := abs (Q_Of (Q1, J) - Q_Of (Q0, J));
               Rj : Long_Float := 0.0;   --  转的关节:往外那几节表面点离它的轴最远多远(走到哪都成立的上界)
            begin
               if Dq > 0.0 then
                  if M.Ax (J).Slide then
                     Best := Best + Dq * Norm (M.Ax (J).W);
                  else
                     for P of In_Pts loop
                        if P.Arm = Arm and then P.Link >= J and then P.Link < N then
                           declare
                              D : Long_Float := 0.0;
                              Prev : V3 := Anc (J);
                           begin
                              for K in J + 1 .. P.Link loop
                                 if M.Ax (K).Slide then
                                    D := D + Norm (M.Ax (K).W) * Off_Ref (K);   --  走的关节:最多走出去这么远
                                 else
                                    D := D + Norm (Sub (Anc (K), Prev));        --  相邻两根转轴上的两点:同一节上,长度不变
                                    Prev := Anc (K);
                                 end if;
                              end loop;
                              D := D + Norm (Sub (P.P, Prev));                   --  点到它那一节最后一根转轴上那一点:同一节上,不变
                              Rj := Long_Float'Max (Rj, D);
                           end;
                        end if;
                     end loop;
                     Best := Best + Dq * Rj;
                  end if;
               end if;
            end;
         end loop;
         Known := True;
         return Pl.S * Best;   --  模型单位 → 世界单位
      end;
   end Max_Shift;
end Links;
