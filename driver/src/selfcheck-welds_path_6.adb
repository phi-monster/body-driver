with Held;
with Seek;
with Linkage;
with Strokes;
with Stats;
separate (Selfcheck)
procedure Welds_Path_6 is
   --  路 6 的焊点(大并行.md §5 路 6):每条写清"错了会是什么病",带一颗牙(去掉那一改就红)
   use Ada.Numerics.Long_Elementary_Functions;
   use type Linkage.Role;
   use type Linkage.Axis_Status;
   use type Linkage.Follow_End;
   use type Held.Ride;
   use type Seek.Seek_End;
   use type Strokes.Strokes_End;
   subtype V3 is Geom.V3;
   subtype M3 is Geom.M3;
   package FR renames Ada.Numerics.Float_Random;
   Gen : FR.Generator;
   Z : constant Long_Float := Stats.Z;
   Deg : constant Long_Float := Ada.Numerics.Pi / 180.0;
   function Add (A, B : V3) return V3 is ([A (0) + B (0), A (1) + B (1), A (2) + B (2)]);
   function Sub (A, B : V3) return V3 is ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]);
   function Scl (A : V3; S : Long_Float) return V3 is ([A (0) * S, A (1) * S, A (2) * S]);
   function Dot (A, B : V3) return Long_Float is (A (0) * B (0) + A (1) * B (1) + A (2) * B (2));
   function Unit (A : V3) return V3 is (Scl (A, 1.0 / Geom.Norm (A)));
   function Rot (Ax : V3; Ang : Long_Float) return M3 is (Geom.Rodrigues (Scl (Unit (Ax), Ang)));
   function Gauss return Long_Float is
      U1 : constant Long_Float := Long_Float'Max (Long_Float (FR.Random (Gen)), 1.0e-300);
      U2 : constant Long_Float := Long_Float (FR.Random (Gen));
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gauss;
   function Uni (A, B : Long_Float) return Long_Float is (A + (B - A) * Long_Float (FR.Random (Gen)));
   --  点到一条直线(过 P、单位方向 W)的距离
   function Line_Dist (X, P, W : V3) return Long_Float is
      D : constant V3 := Sub (X, P);
   begin
      return Geom.Norm (Sub (D, Scl (W, Dot (D, W))));
   end Line_Dist;

   --  合成的场景:模型坐标 ↦ 世界 = 转 Sr、放大 Us 倍、平移 St(不认单位和尺度:世界里的数没有一个是"米")。
   --  每个点的噪声:横着 Sa、沿"视线"Vd(模型系)Sd(两眼交点那样,沿视线的不准是横着的几倍),都是模型单位;
   --  给 Linkage 的协方差比真的小 Under 倍(标准差;1 = 给得准)
   type Scene is record
      Sr : M3 := Geom.Identity;
      St : V3 := [others => 0.0];
      Us : Long_Float := 1.0;
      Vd : V3 := [0.0, 0.0, 1.0];
      Sa, Sd : Long_Float := 0.0;
      Under : Long_Float := 1.0;
   end record;
   function To_W (Sc : Scene; P : V3) return V3 is (Add (Scl (Geom.Ap (Sc.Sr, P), Sc.Us), Sc.St));
   function Dir_W (Sc : Scene; D : V3) return V3 is (Geom.Ap (Sc.Sr, D));
   --  真的噪声的一个因子 L(L Lᵀ = 真的协方差,世界单位):σa (I − v vᵀ) + σd v vᵀ,v = 视线在世界里
   function True_L (Sc : Scene) return M3 is
      V : constant V3 := Dir_W (Sc, Unit (Sc.Vd));
      L : M3;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            L (I, J) := Sc.Us * (Sc.Sa * ((if I = J then 1.0 else 0.0) - V (I) * V (J)) + Sc.Sd * V (I) * V (J));
         end loop;
      end loop;
      return L;
   end True_L;
   function Given_Cov (Sc : Scene) return M3 is
      L : constant M3 := True_L (Sc);
      C : M3 := Geom.Mul (L, Geom.Tr (L));
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            C (I, J) := C (I, J) / Sc.Under ** 2;
         end loop;
      end loop;
      return C;
   end Given_Cov;
   --  一笔观测:模型坐标的真位置 ⇒ 世界位置 + 真的噪声;协方差给 Given_Cov
   function Look (Sc : Scene; P : V3; Seen : Boolean) return Linkage.Obs is
      L : constant M3 := True_L (Sc);
      E : constant V3 := [Gauss, Gauss, Gauss];
   begin
      if not Seen then
         return (Seen => False, others => <>);
      end if;
      return (Seen => True, X => Add (To_W (Sc, P), Geom.Ap (L, E)), Cov => Given_Cov (Sc));
   end Look;

   --  模型坐标里的刚体运动
   type Motion is record
      R : M3 := Geom.Identity;
      T : V3 := [others => 0.0];
   end record;
   function Apply (M : Motion; P : V3) return V3 is (Add (Geom.Ap (M.R, P), M.T));
   Still : constant Motion := (others => <>);
   --  绕过 P0、方向 Ax 的轴转 Ang
   function Hinge (Ax, P0 : V3; Ang : Long_Float) return Motion is
      R : constant M3 := Rot (Ax, Ang);
   begin
      return (R => R, T => Sub (P0, Geom.Ap (R, P0)));
   end Hinge;

   Max_K : constant := 8;
   type Motions is array (0 .. Max_K - 1) of Motion;
   --  一条轨迹:模型坐标的一个点 P,第 F 帧它在 Base (F) ∘ Rel (F) 之后的地方;Drop = 每一帧看不见的机会
   procedure Add_Track (Tr : in out Linkage.Track_Vectors.Vector; Sc : Scene; Kf : Natural; Base, Rel : Motions; P : V3; Drop : Long_Float) is
      O : Linkage.Obs_Vectors.Vector;
   begin
      for F in 0 .. Kf - 1 loop
         O.Append (Look (Sc, Apply (Base (F), Apply (Rel (F), P)), Long_Float (FR.Random (Gen)) >= Drop));
      end loop;
      Tr.Append (O);
   end Add_Track;
   --  混进来的点:每一帧在一个方盒子里随便一处(跟错了的点)
   procedure Add_Junk (Tr : in out Linkage.Track_Vectors.Vector; Sc : Scene; Kf : Natural; Half : Long_Float) is
      O : Linkage.Obs_Vectors.Vector;
   begin
      for F in 0 .. Kf - 1 loop
         O.Append (Look (Sc, [Uni (-Half, Half), Uni (-Half, Half), Uni (-Half, Half)], True));
      end loop;
      Tr.Append (O);
   end Add_Junk;

   --  归块对不对:每一块按成员里的多数认它是哪一块真的;Wrong = 成员里不是那一块的;Junk_In = 混进来的点被归进了哪一块;
   --  Amb_Far = 被说成"两块都说得通"、但它真的相对挪得比两块的门还远(不该分不清)的点;
   --  Real_Lost = 真点被说成"哪块都说不通"的;Lost_Bound = 门自己的虚警该有的上限:n 个真点每个 Q(Z) 的机会被自己那一块的门挡在外面
   --  (Chance_Bound;再多就不是运气,是门或者噪声倍数量错了)。被挡在外面的点离另一块近,就被那一块收走 = 归错:归错和说不通是同一件事,合起来算
   type Tally is record
      Labels : Bytes.Ints;
      Wrong, Junk_In, Junk_Unexplained, Amb, Amb_Far, Real_Lost, Lost_Bound : Natural := 0;
   end record;
   Tail_Z : constant Long_Float := 0.0013498980316301;   --  Q(3):一维正态单侧 Z = 3 倍的尾巴(自检里的查表值,门那一条焊点另外按积分核过)
   --  二维的误差(轴的方向两个角、轴的位置垂直于轴两个数)跟它最不准那个方向的一倍标准差比:同一个置信度是二维卡方的门开方,
   --  不是 Z(各向一样不准时 |e| > Z σ 的机会是 e^{−Z²/2} = 1.1%,比一维的 0.135% 大 8 倍;09-30 离线 40 组种子三块那一场 1 次 3.3 倍)
   Z2 : constant Long_Float := Sqrt (Linkage.Gate (2));
   --  n 个真点里,门凭运气挡在外面的上限:n Q(Z) + Z √(n Q(Z)) 向上取整
   function Chance_Bound (N : Natural) return Natural is
     (Natural (Long_Float'Ceiling (Long_Float (N) * Tail_Z + Z * Sqrt (Long_Float (N) * Tail_Z))));
   function Count_Roles (Rep : Linkage.Report; Truth : Bytes.Ints; Rel_Move : Bytes.Floats; Amb_Bound : Long_Float) return Tally is
      T : Tally;
   begin
      for P of Rep.Pieces loop
         declare
            Votes : array (-1 .. 9) of Natural := [others => 0];
            Best : Integer := -1;
         begin
            for I of P.Members loop
               Votes (Truth (I)) := Votes (Truth (I)) + 1;
            end loop;
            for L in 0 .. 9 loop
               if Votes (L) > (if Best < 0 then 0 else Votes (Best)) then
                  Best := L;
               end if;
            end loop;
            T.Labels.Append (Best);
            for I of P.Members loop
               --  "两块都说得通"的也留在块里拟合(块是成团地归出来的),它归哪一块不算对错,另由 Amb_Far 核它是不是真在两块交界处
               if Truth (I) /= Best and then Rep.Roles (I) = Linkage.Member then
                  T.Wrong := T.Wrong + 1;
               end if;
            end loop;
         end;
      end loop;
      for I in 0 .. Natural (Truth.Length) - 1 loop
         if Truth (I) < 0 then
            if Rep.Roles (I) = Linkage.Member then
               T.Junk_In := T.Junk_In + 1;
            elsif Rep.Roles (I) = Linkage.Unexplained then
               T.Junk_Unexplained := T.Junk_Unexplained + 1;
            end if;
         elsif Rep.Roles (I) = Linkage.Ambiguous then
            T.Amb := T.Amb + 1;
            if Rel_Move (I) > Amb_Bound then
               T.Amb_Far := T.Amb_Far + 1;
            end if;
         elsif Rep.Roles (I) = Linkage.Unexplained then
            T.Real_Lost := T.Real_Lost + 1;
         end if;
      end loop;
      declare
         Real : Natural := 0;
      begin
         for L of Truth loop
            if L >= 0 then
               Real := Real + 1;
            end if;
         end loop;
         T.Lost_Bound := Chance_Bound (Real);
      end;
      return T;
   end Count_Roles;
   function Img_Ints (V : Bytes.Ints) return String is
      S : Unbounded_String;
   begin
      for X of V loop
         Append (S, (if Length (S) > 0 then "," else "") & Codec.Img (X));
      end loop;
      return To_String (S);
   end Img_Ints;
   function Sizes (Rep : Linkage.Report) return String is
      S : Unbounded_String;
   begin
      for P of Rep.Pieces loop
         Append (S, (if Length (S) > 0 then "/" else "") & Codec.Img (Natural (P.Members.Length)));
      end loop;
      return To_String (S);
   end Sizes;

   --  卡方 k 个自由度的一个样本:k 是偶数 ⇒ −2 Σ ln U(k/2 个);奇数 ⇒ 再加一个正态的平方
   function Chi2 (K : Positive) return Long_Float is
      S : Long_Float := 0.0;
   begin
      for I in 1 .. K / 2 loop
         S := S - 2.0 * Log (Long_Float'Max (Long_Float (FR.Random (Gen)), 1.0e-300));
      end loop;
      if K mod 2 = 1 then
         S := S + Gauss ** 2;
      end if;
      return S;
   end Chi2;

   --  ── 两块东西的一次合成 ──
   --  底座的点 Pa、动的那一块的点 Pb(模型坐标,参照 = 第 0 帧的样子);第 F 帧:底座 Base (F),动的那块先相对底座 Rel (F) 再跟底座走;
   --  混进 Junk 个坏点;每一笔 Drop 的机会看不见。真轴(模型坐标,底座上):Ax_W 方向、过 Ax_P;Qt (F) = 第 F 帧的真关节量(转:弧度;走:模型单位)。
   --  Want:该认出转轴 / 滑轴 / 定不下。Tooth:这一条的牙是哪一种老办法(见各条)
   type Expect is (Rot_Axis, Slide_Axis, Cannot_Tell);
   type Tooth_Kind is (Moved_Or_Not, Curve_Nonzero, Flat_Means_Slide);
   procedure Two_Piece (Name : String; Sc : Scene; Kf : Positive; Seed : Integer; Pa, Pb : Geom.V3_Vectors.Vector; Base, Rel : Motions;
                        Junk : Natural; Drop : Long_Float; Want : Expect; Ax_W, Ax_P : V3; Qt : Bytes.Floats; Tooth : Tooth_Kind) is
      Tracks : Linkage.Track_Vectors.Vector;
      Truth : Bytes.Ints;
      Rel_Move : Bytes.Floats;
      Rep : Linkage.Report;
      Id : constant Motions := [others => Still];
      Naive_Still, Naive_Moved : Natural := 0;
      procedure Side (Pts : Geom.V3_Vectors.Vector; R : Motions; Label : Natural) is
      begin
         for P of Pts loop
            declare
               Mx : Long_Float := 0.0;
            begin
               Add_Track (Tracks, Sc, Kf, Base, R, P, Drop);
               Truth.Append (Label);
               for F in 0 .. Kf - 1 loop
                  Mx := Long_Float'Max (Mx, Sc.Us * Geom.Norm (Sub (Apply (Rel (F), P), P)));
               end loop;
               Rel_Move.Append (Mx);
               --  "动没动"分块那一种老办法:第一帧到最后一帧真挪得不到 Z 倍噪声的算没动
               if Sc.Us * Geom.Norm (Sub (Apply (Base (Kf - 1), Apply (R (Kf - 1), P)), P)) <= Z * Sc.Us * Sc.Sd then
                  Naive_Still := Naive_Still + 1;
               else
                  Naive_Moved := Naive_Moved + 1;
               end if;
            end;
         end loop;
      end Side;
   begin
      FR.Reset (Gen, Seed);
      Side (Pa, Id, 0);
      Side (Pb, Rel, 1);
      for J in 1 .. Junk loop
         Add_Junk (Tracks, Sc, Kf, 0.35);
         Truth.Append (-1);
         Rel_Move.Append (0.0);
      end loop;
      Linkage.Fit (Tracks, Rep);
      Put_Line ("    " & Name & ":" & Linkage.Say (Rep));
      declare
         --  两块都说得通的点,真的相对挪得不会超过两道门的宽(门按量出来的噪声倍数开:给的 × Sigma,真的 × Sigma / Under)
         Amb_Bound : constant Long_Float := 2.0 * Sqrt (Linkage.Gate (3 * (Kf - 1))) * Sc.Us * Sc.Sd * Rep.Sigma / Sc.Under;
         T : constant Tally := Count_Roles (Rep, Truth, Rel_Move, Amb_Bound);
         Two : constant Boolean := Natural (Rep.Pieces.Length) = 2 and then Natural (Rep.Joints.Length) = 1;
         Sig_Ok : constant Boolean := Rep.Sigma / Sc.Under > 0.8 and then Rep.Sigma / Sc.Under < 1.25;
      begin
         Check (Two and then T.Wrong + T.Real_Lost <= T.Lost_Bound and then T.Junk_In = 0 and then T.Junk_Unexplained = Junk and then T.Amb_Far = 0
                and then Sig_Ok,
                Name & "·归块:" & Codec.Img (Natural (Rep.Pieces.Length)) & " 块(" & Sizes (Rep) & ",真的 " & Img_Ints (T.Labels)
                & ")· 归错 " & Codec.Img (T.Wrong) & " + 真点说不通 " & Codec.Img (T.Real_Lost) & "(门的虚警上限 " & Codec.Img (T.Lost_Bound) & ")"
                & " · 坏点进块 " & Codec.Img (T.Junk_In) & "、说不通 " & Codec.Img (T.Junk_Unexplained) & "/" & Codec.Img (Junk)
                & " · 分不清 " & Codec.Img (T.Amb) & "(离轴太远还分不清 " & Codec.Img (T.Amb_Far)
                & ")· 噪声倍数 " & Codec.Fmt (Rep.Sigma, 3) & "(真的是给的 " & Codec.Fmt (Sc.Under, 1) & " 倍)");
         if Two then
            declare
               J : constant Linkage.Joint := Rep.Joints (0);
               Base_Is_A : constant Boolean := T.Labels (J.A) = 0;
               R : constant Natural := Rep.Ref;
               Wt : constant V3 := Dir_W (Sc, Geom.Ap (Base (R).R, Ax_W));
               Pt : constant V3 := To_W (Sc, Apply (Base (R), Ax_P));
               Sgn : constant Long_Float := (if Dot (J.Ax.W, Wt) >= 0.0 then 1.0 else -1.0) * (if Base_Is_A then 1.0 else -1.0);
               Ang : constant Long_Float := Arccos (Long_Float'Min (1.0, abs Dot (J.Ax.W, Wt)));
               Q_Worst : Long_Float := 0.0;
               Q_Ok : Boolean := True;
               Unit_Q : constant Long_Float := (if Want = Slide_Axis then Sc.Us else 1.0);
            begin
               if J.Status = Linkage.Found then
                  for F in 0 .. Kf - 1 loop
                     if J.Q_Sd (F) < Long_Float'Last then
                        declare
                           E : constant Long_Float := abs (Sgn * J.Q (F) - Unit_Q * (Qt (F) - Qt (R)));
                        begin
                           Q_Worst := Long_Float'Max (Q_Worst, E / Long_Float'Max (J.Q_Sd (F), Long_Float'Model_Small));
                           Q_Ok := Q_Ok and then E <= Z * J.Q_Sd (F);
                        end;
                     end if;
                  end loop;
               end if;
               case Want is
                  when Rot_Axis =>
                     declare
                        Pd : constant Long_Float := Line_Dist (J.Ax.P, Pt, Wt);
                     begin
                        Check (J.Status = Linkage.Found and then not J.Ax.Slide and then Ang <= Z2 * J.W_Sd and then Pd <= Z2 * J.P_Sd and then Q_Ok
                               and then (Tooth /= Moved_Or_Not or else Naive_Still = 0),
                               Name & "·转轴:" & J.Status'Image & (if J.Ax.Slide then "(滑)" else "(转)") & " · 方向差 " & Codec.Fmt (Ang / Deg, 3)
                               & "°(自报 ±" & Codec.Fmt (J.W_Sd / Deg, 3) & "°)· 轴离真轴 " & Codec.Fmt (Pd, 4) & "(自报 ±" & Codec.Fmt (J.P_Sd, 4)
                               & ")· 转角差最多 " & Codec.Fmt (Q_Worst, 2) & " 倍自报 · 曲率 " & Codec.Fmt (J.Kappa, 3) & " ± " & Codec.Fmt (J.Kappa_Sd, 3)
                               & (if Tooth = Moved_Or_Not then " · 牙:按动没动分,没动 " & Codec.Img (Naive_Still) & " / 动了 " & Codec.Img (Naive_Moved)
                                  & "(底座也在动 ⇒ 分不出两块)" else ""));
                     end;
                  when Slide_Axis =>
                     Check (J.Status = Linkage.Found and then J.Ax.Slide and then Ang <= Z2 * J.W_Sd and then Q_Ok
                            and then (Tooth /= Curve_Nonzero or else J.Kappa /= 0.0),
                            Name & "·滑轴:" & J.Status'Image & (if J.Ax.Slide then "(滑)" else "(转)") & " · 方向差 " & Codec.Fmt (Ang / Deg, 3)
                            & "°(自报 ±" & Codec.Fmt (J.W_Sd / Deg, 3) & "°)· 走的量差最多 " & Codec.Fmt (Q_Worst, 2) & " 倍自报 · 曲率 "
                            & Codec.Fmt (J.Kappa, 4) & " ± " & Codec.Fmt (J.Kappa_Sd, 4) & "(要是转轴,离 C 至少 " & Codec.Fmt (J.Min_Radius, 2)
                            & ",这件东西只铺到 " & Codec.Fmt (J.Reach, 2) & ")"
                            & (if Tooth = Curve_Nonzero then " · 牙:按'拟合出来的曲率不是 0 就是转'判 ⇒ 转" else ""));
                  when Cannot_Tell =>
                     declare
                        True_R : constant Long_Float := Line_Dist (J.C, Pt, Wt);   --  C 离真轴多远
                        Naive_Slide : constant Boolean := not (abs J.Kappa > Z * J.Kappa_Sd);
                     begin
                        Check (J.Status = Linkage.Undecided and then J.Min_Radius <= True_R and then (Tooth /= Flat_Means_Slide or else Naive_Slide),
                               Name & "·定不下:" & J.Status'Image & " · 曲率 " & Codec.Fmt (J.Kappa, 3) & " ± " & Codec.Fmt (J.Kappa_Sd, 3)
                               & " · 说'要是转轴离 C 至少 " & Codec.Fmt (J.Min_Radius, 3) & "',真轴离 C " & Codec.Fmt (True_R, 3)
                               & " · 这件东西铺到 " & Codec.Fmt (J.Reach, 3)
                               & (if Tooth = Flat_Means_Slide then " · 牙:按'弯得不显著就是滑'判 ⇒ " & (if Naive_Slide then "滑(错)" else "转") else ""));
                     end;
               end case;
            end;
         end if;
      end;
   end Two_Piece;
   --  平面上一片格点(模型坐标):X0 起每 Dx 一个共 Nx 个,Y0 起每 Dy 一个共 Ny 个,z = Z0
   function Grid (X0, Dx : Long_Float; Nx : Positive; Y0, Dy : Long_Float; Ny : Positive; Z0 : Long_Float) return Geom.V3_Vectors.Vector is
      V : Geom.V3_Vectors.Vector;
   begin
      for I in 0 .. Nx - 1 loop
         for J in 0 .. Ny - 1 loop
            V.Append (V3'[X0 + Dx * Long_Float (I), Y0 + Dy * Long_Float (J), Z0]);
         end loop;
      end loop;
      return V;
   end Grid;
begin
   Put_Line ("── 路 6:部件和轴、顺着它让的方向走 ──");

   --  🔴 同一个置信度的门(Linkage.Gate,Wilson–Hilferty):ν 维白化残差的平方和过门的机会,要和一维单侧 Z 倍一样少见(Q(Z) = 0.135%)。
   --  错了会是什么病:门按"每一维 Z 倍"拼(ν Z²)⇒ 三维时 50 个坏点里 49 个放进来;门一律 Z²(不管几维)⇒ 三维时 2.9% 的好点、
   --  五帧(12 维)时 43% 的好点被当成跟错了。准的尾巴:χ²_1 = 2Q(√x)、χ²_2 = e^{−x/2}、再往上 S_{ν+2} = S_ν + (x/2)^{ν/2} e^{−x/2} / Γ(ν/2 + 1),
   --  Q 按正态密度数值积分(Simpson,离线)。牙:ν Z² 和 Z² 两种拼法同一组 ν 算出来的尾巴,差出几个数量级
   declare
      function Phi (X : Long_Float) return Long_Float is (Exp (-0.5 * X * X) / Sqrt (2.0 * Ada.Numerics.Pi));
      function Q_Tail (Z0 : Long_Float) return Long_Float is
         Hi : constant Long_Float := Z0 + 14.0;
         M : constant := 20000;
         H : constant Long_Float := (Hi - Z0) / Long_Float (M);
         S : Long_Float := Phi (Z0) + Phi (Hi);
      begin
         for I in 1 .. M - 1 loop
            S := S + (if I mod 2 = 1 then 4.0 else 2.0) * Phi (Z0 + Long_Float (I) * H);
         end loop;
         return S * H / 3.0;
      end Q_Tail;
      function Chi_Tail (Nu : Positive; X : Long_Float) return Long_Float is
         S : Long_Float := (if Nu mod 2 = 1 then 2.0 * Q_Tail (Sqrt (X)) else Exp (-0.5 * X));
         Nu0 : constant Natural := (if Nu mod 2 = 1 then 1 else 2);
         Gm : Long_Float := (if Nu0 = 1 then Sqrt (Ada.Numerics.Pi) else 1.0);   --  Γ(ν0 / 2)
         V : Natural := Nu0;
      begin
         while V < Nu loop
            Gm := Gm * (Long_Float (V) / 2.0);                     --  Γ(v/2 + 1) = (v/2) Γ(v/2)
            S := S + (0.5 * X) ** (Long_Float (V) / 2.0) * Exp (-0.5 * X) / Gm;
            V := V + 2;
         end loop;
         return S;
      end Chi_Tail;
      Target : constant Long_Float := Q_Tail (Z);
      Worst : Long_Float := 1.0;
      Old_A, Old_B : Long_Float := 1.0;
   begin
      for Nu in 1 .. 12 loop
         declare
            R : constant Long_Float := Chi_Tail (Nu, Linkage.Gate (Nu)) / Target;
         begin
            if abs Log (R) > abs Log (Worst) then
               Worst := R;
            end if;
         end;
      end loop;
      Old_A := Chi_Tail (3, 3.0 * Z * Z) / Target;
      Old_B := Chi_Tail (12, Z * Z) / Target;
      Check (Worst > 0.8 and then Worst < 1.25 and then Old_A < 0.01 and then Old_B > 100.0,
             "门·ν 维卡方的尾巴 = 一维单侧 Z 倍的尾巴 " & Codec.Fmt (100.0 * Target, 3) & "%:ν = 1..12 最差差 " & Codec.Fmt (Worst, 3)
             & " 倍 · 牙:每维 Z 倍拼(ν = 3)⇒ " & Codec.Fmt (Old_A, 5) & " 倍,一律 Z²(ν = 12)⇒ " & Codec.Fmt (Old_B, 0) & " 倍");
   end;


   --  🔴 噪声是量的,门要跟着宽(Linkage.Gate_F):白化残差平方和 ÷ 量出来的噪声² 服从 ν F(ν, m)(m = 量噪声的有效自由度),
   --  过 Gate_F (ν, m) 的机会要和一维单侧 Z 倍一样少见(Q(Z) = 0.135%)。蒙特卡洛:每组 (ν, m) 抽 20 万个 ν (χ²_ν/ν)/(χ²_m/m)。
   --  错了会是什么病:量噪声用的样本少、量小了一成,门按卡方开 ⇒ 真点成批地被挡在门外(09-30 离线:小块那一场噪声量成 0.81,两个真点说不通)。
   --  牙:同样的样本按卡方的门 Gate (ν) 判,m 小时过门的多出几倍
   declare
      Draws : constant := 200_000;
      Worst, Old_Worst : Long_Float := 1.0;
      type Pair is record
         Nu, M : Positive;
      end record;
      Cases : constant array (1 .. 3) of Pair := [(3, 20), (6, 60), (9, 150)];
   begin
      FR.Reset (Gen, 41);
      for C of Cases loop
         declare
            G_F : constant Long_Float := Linkage.Gate_F (C.Nu, Long_Float (C.M));
            G_C : constant Long_Float := Linkage.Gate (C.Nu);
            Hit_F, Hit_C : Natural := 0;
         begin
            for D in 1 .. Draws loop
               declare
                  X : constant Long_Float := Chi2 (C.Nu) / (Chi2 (C.M) / Long_Float (C.M));
               begin
                  if X > G_F then
                     Hit_F := Hit_F + 1;
                  end if;
                  if X > G_C then
                     Hit_C := Hit_C + 1;
                  end if;
               end;
            end loop;
            declare
               R_F : constant Long_Float := Long_Float (Hit_F) / Long_Float (Draws) / Tail_Z;
               R_C : constant Long_Float := Long_Float (Hit_C) / Long_Float (Draws) / Tail_Z;
            begin
               if abs Log (Long_Float'Max (R_F, 1.0e-9)) > abs Log (Worst) then
                  Worst := R_F;
               end if;
               Old_Worst := Long_Float'Max (Old_Worst, R_C);
            end;
         end;
      end loop;
      Check (Worst > 0.75 and then Worst < 1.33 and then Old_Worst > 2.0 and then Linkage.Gate_F (3, Long_Float'Last) = Linkage.Gate (3),
             "门·噪声是量的:ν F(ν, m) 过 Gate_F 的机会 / Q(Z),(3,20)(6,60)(9,150)最差 " & Codec.Fmt (Worst, 3) & " 倍 · m 无穷时就是 Gate"
             & " · 牙:按卡方的门判,最多 " & Codec.Fmt (Old_Worst, 2) & " 倍");
   end;

   --  🔴 Z_Of 是 Gate_F 反过来(Linkage.Z_Of;两块合不合、先合哪一对按它比):门上的平方和换回去正好是 Z 倍,门内一点的不到 Z、门外一点的过 Z;
   --  m 无穷时就是 Wilson–Hilferty。错了会是什么病:"不显著才合"按 Z_Of 判,别处的门按 Gate_F 开 —— 两个对不上,同一对块按一个该合、
   --  按另一个不该合。牙:不管噪声是量的、按卡方换(m 当无穷),m 小时门上的就不是 Z 倍
   declare
      type Pair is record
         Nu : Positive;
         M : Long_Float;
      end record;
      Cases : constant array (1 .. 4) of Pair := [(3, 20.0), (6, 60.0), (12, 150.0), (24, Long_Float'Last)];
      Worst, Old_Worst : Long_Float := 0.0;
      Side_Ok : Boolean := True;
   begin
      for C of Cases loop
         declare
            G : constant Long_Float := Linkage.Gate_F (C.Nu, C.M);
         begin
            Worst := Long_Float'Max (Worst, abs (Linkage.Z_Of (G, C.Nu, C.M) - Z));
            Old_Worst := Long_Float'Max (Old_Worst, abs (Linkage.Z_Of (G, C.Nu, Long_Float'Last) - Z));
            Side_Ok := Side_Ok and then Linkage.Z_Of (0.9 * G, C.Nu, C.M) < Z and then Linkage.Z_Of (1.1 * G, C.Nu, C.M) > Z;
         end;
      end loop;
      Check (Worst < Sqrt (Long_Float'Model_Epsilon) and then Side_Ok and then Old_Worst > 0.1,
             "门·Z_Of 是 Gate_F 反过来:(3,20)(6,60)(12,150)(24,∞)门上换回去差 Z 最多 " & Codec.Fmt (Worst, 12) & " · 门内一成 < Z、门外一成 > Z "
             & Side_Ok'Image & " · 牙:按卡方换,门上差 Z 最多 " & Codec.Fmt (Old_Worst, 3));
   end;

   --  🔴 铰链(Linkage.Fit):底座 48 个点、门 48 个点,门绕底座上的一根轴转 0 / 8 / 16 / 24 / 32°,同时整件东西被挪着走(转 2°/帧 + 平移);
   --  混进 10 个跟错了的点;每一笔一成的机会看不见;噪声沿"视线"是横着的 4 倍;整个场景转到一个斜的方向、放大 7.3 倍。
   --  要:两块、没有一个点归错、10 个坏点都说"哪块都说不通"、"两块都说得通"的只在轴附近;
   --  转轴的方向、位置在自报的不确定度的二维门(Z2 倍)以内,每一帧的转角在 Z 倍以内;量出来的噪声倍数 ≈ 给的协方差差的那个倍数。
   --  错了会是什么病:按"动没动"分块 —— 整件东西在被挪着走,底座和门都动了 ⇒ 分不出两块;坏点硬塞进哪一块 ⇒ 轴被拽歪;
   --  不确定度只按一帧的观测算(形状当成准的)⇒ 自报的比真的小,轴出了自报的范围;信给的协方差不量 ⇒ 给小了 4 倍时门窄 4 倍,
   --  一块碎成几块、轴的不确定度小报 4 倍。牙:按"动没动"分,96 个点全在"动了"那一组(第二条同一套、给的协方差小 4 倍)
   declare
      Kf : constant := 5;
      Base, Door : Motions := [others => Still];
      Qt : Bytes.Floats;
   begin
      for F in 0 .. Kf - 1 loop
         Qt.Append (8.0 * Deg * Long_Float (F));
         Base (F) := (R => Rot ([1.0, 1.0, 0.5], 2.0 * Deg * Long_Float (F)), T => [0.01 * Long_Float (F), -0.005 * Long_Float (F), 0.003 * Long_Float (F)]);
         Door (F) := Hinge ([0.0, 1.0, 0.0], [0.0, 0.0, 0.0], Qt (F));
      end loop;
      for Under in 0 .. 1 loop
         Two_Piece ((if Under = 0 then "铰链" else "铰链(给的协方差小 4 倍)"),
                    (Sr => Rot ([0.3, -0.5, 0.8], 0.7), St => [1.2, -0.4, 2.5], Us => 7.3, Vd => [0.2, 0.3, 0.93], Sa => 0.002, Sd => 0.008,
                     Under => (if Under = 0 then 1.0 else 4.0)),
                    Kf, 61 + Under, Grid (-0.30, 0.05, 6, -0.175, 0.05, 8, 0.0), Grid (0.05, 0.05, 6, -0.175, 0.05, 8, 0.0), Base, Door,
                    10, 0.1, Rot_Axis, [0.0, 1.0, 0.0], [0.0, 0.0, 0.0], Qt, Moved_Or_Not);
      end loop;
   end;

   --  🔴 滑轨(Linkage.Fit):柜子(顶面 15 个点 + 侧面 6 个点)被挪着走(1.5°/帧 + 平移),抽屉脸 12 个点沿柜子里的一根滑轴走 0 / 4 / 8 / 12 / 16 cm 那么多
   --  (模型单位 0.04 一档);混进 8 个坏点;单位是 0.37 倍(换一套单位也一样)。
   --  要:两块、归对,认成滑轴,方向在自报的二维门(Z2 倍)以内、每一帧走了多少在 Z 倍以内。
   --  错了会是什么病:照身体关节那样"按转、按走各解一次,残差小的那样"—— 身体关节的转角是读数给的,两样参数一样多;东西的转角要和轴一起解,
   --  转的那一样总能把轴放到很远去装成走,残差永远不比走的大 ⇒ 每一根滑轴都认成转轴(离得很远的)。
   --  牙:按"拟合出来的曲率不是 0 就是转"判 ⇒ 转(真的曲率是 0,拟合出来的总不是 0)
   declare
      Kf : constant := 5;
      Base, Drawer : Motions := [others => Still];
      Qt : Bytes.Floats;
      Cab : Geom.V3_Vectors.Vector := Grid (-0.2, 0.1, 5, -0.2, 0.1, 3, 0.1);
      Front : Geom.V3_Vectors.Vector;
   begin
      for P of Grid (-0.2, 0.0, 1, -0.2, 0.1, 3, -0.1) loop
         Cab.Append (P);
      end loop;
      for P of Grid (-0.2, 0.0, 1, -0.2, 0.1, 3, 0.0) loop
         Cab.Append (P);
      end loop;
      for I in 0 .. 3 loop
         for J in 0 .. 2 loop
            Front.Append (V3'[-0.15 + 0.1 * Long_Float (I), 0.02, -0.08 + 0.05 * Long_Float (J)]);
         end loop;
      end loop;
      for F in 0 .. Kf - 1 loop
         Qt.Append (0.04 * Long_Float (F));
         Base (F) := (R => Rot ([0.2, 1.0, 0.3], 1.5 * Deg * Long_Float (F)), T => [0.004 * Long_Float (F), 0.002 * Long_Float (F), -0.003 * Long_Float (F)]);
         Drawer (F) := (R => Geom.Identity, T => [0.0, Qt (F), 0.0]);
      end loop;
      Two_Piece ("滑轨", (Sr => Rot ([-0.6, 0.2, 0.4], 1.9), St => [-0.3, 0.8, 0.1], Us => 0.37, Vd => [0.1, 0.9, 0.4], Sa => 0.002, Sd => 0.006,
                           Under => 1.0),
                 Kf, 71, Cab, Front, Base, Drawer, 8, 0.1, Slide_Axis, [0.0, 1.0, 0.0], [0.0, 0.0, 0.0], Qt, Curve_Nonzero);
   end;

   --  🔴 动得太少 ⇒ 定不下(Linkage.Fit):门上只跟住了离轴 2 远、0.04 见方的一小块(9 个点),门只转了 0 / 0.15 / 0.3°;
   --  底座 48 个点铺到轴的另一侧 2 远;混进 6 个坏点。这一小块挪了 9 倍噪声,而"整件东西一起转"说不通它(那样底座远端也要挪 9 倍噪声)
   --  ⇒ 看得出是两块;可它自己转了多少只有噪声的四分之一那么清楚:是绕东西上的一根轴转、还是沿一根滑轴走,分不出。
   --  (底座只铺到轴边 0.3 时,"整件东西绕门轴转 0.3°"同样说得通,合成一块的代价不比两块大 ⇒ 归成一块、说"只见过它整块动"也是照实 ——
   --  09-30 离线 20 组种子里 2 次;这一条要测的是"分得出两块、定不下轴",几何放在分得清的那一边)
   --  要:两块、归对、坏点都说不通;轴说"定不下",并且照实说的"要是转轴,离 C 至少多远"不比真轴离 C 的远。
   --  同一块、同样的点,门转到 0 / 20 / 40° ⇒ 定成转轴,方向、位置、转角都在自报以内(第二条)。
   --  错了会是什么病:"弯得不显著就是滑" ⇒ 把一扇门说成抽屉,执行时沿直线拽门。牙:按"弯得不显著就是滑"判 ⇒ 滑
   declare
      Kf : constant := 3;
      Door_S, Door_L : Motions := [others => Still];
      Qs, Ql : Bytes.Floats;
      Base : constant Motions := [others => Still];
      Sc : constant Scene := (Sr => Rot ([0.5, 0.5, -0.2], 0.4), St => [0.3, 0.2, 1.1], Us => 1.0, Vd => [0.93, 0.3, 0.2], Sa => 0.001, Sd => 0.003,
                              Under => 1.0);
   begin
      for F in 0 .. Kf - 1 loop
         Qs.Append (0.15 * Deg * Long_Float (F));
         Ql.Append (20.0 * Deg * Long_Float (F));
         Door_S (F) := Hinge ([0.0, 1.0, 0.0], [0.0, 0.0, 0.0], Qs (F));
         Door_L (F) := Hinge ([0.0, 1.0, 0.0], [0.0, 0.0, 0.0], Ql (F));
      end loop;
      Two_Piece ("转得太少", Sc, Kf, 81, Grid (-2.0, 0.35, 6, -0.175, 0.05, 8, 0.0), Grid (1.98, 0.02, 3, -0.02, 0.02, 3, 0.0), Base, Door_S,
                 6, 0.0, Cannot_Tell, [0.0, 1.0, 0.0], [0.0, 0.0, 0.0], Qs, Flat_Means_Slide);
      Two_Piece ("同一小块转得多", Sc, Kf, 82, Grid (-2.0, 0.35, 6, -0.175, 0.05, 8, 0.0), Grid (1.98, 0.02, 3, -0.02, 0.02, 3, 0.0), Base, Door_L,
                 6, 0.0, Rot_Axis, [0.0, 1.0, 0.0], [0.0, 0.0, 0.0], Ql, Flat_Means_Slide);
   end;

   --  🔴 三块(Linkage.Fit;不认零件个数):柜子(侧面 24 个点)被挪着走,一扇门(24 个点)绕柜子上的竖轴转 0 / 10 / 20 / 30°,
   --  一只抽屉(18 个点)沿柜子里的一根滑轴同时拉出 0 / 0.06 / 0.03 / 0.09(进进出出,和门不同步);混进 8 个坏点。
   --  要:三块、归对;柜子–门 = 转轴、柜子–抽屉 = 滑轴(方向、位置、关节量都在自报以内);门–抽屉 = 一根轴说不清(门转、抽屉走,
   --  它俩之间的相对运动是一根每一帧都换了地方的转轴。两个一起匀速动时这根虚的轴几乎不挪 —— 那时"一根轴"说得通,是对的)。
   --  错了会是什么病:只会两块 / 只在点最多的那一块和别的之间找轴 ⇒ 抽屉被说成门的一部分,
   --  或者门和抽屉之间凭空量出一根"轴"。牙:门–抽屉那一对,一根轴比各走各的多出来的卡方过门(说明它俩真的不是一根轴连着的)
   declare
      Kf : constant := 4;
      Sc : constant Scene := (Sr => Rot ([0.4, 0.1, -0.9], 2.3), St => [0.0, 1.5, -0.7], Us => 2.2, Vd => [0.5, -0.6, 0.6], Sa => 0.0015, Sd => 0.004,
                              Under => 1.0);
      Base, Door, Drawer : Motions := [others => Still];
      Id : constant Motions := [others => Still];
      Tracks : Linkage.Track_Vectors.Vector;
      Truth : Bytes.Ints;
      Rel_Move : Bytes.Floats;
      Rep : Linkage.Report;
      Door_Ax : constant V3 := [0.0, 0.0, 1.0];
      Door_P : constant V3 := [0.2, 0.0, 0.0];
      Drawer_W : constant V3 := [0.0, -1.0, 0.0];
      procedure Side (Pts : Geom.V3_Vectors.Vector; R : Motions; Label : Natural) is
      begin
         for P of Pts loop
            Add_Track (Tracks, Sc, Kf, Base, R, P, 0.05);
            Truth.Append (Label);
            Rel_Move.Append (0.0);
         end loop;
      end Side;
      function Of_Label (T : Tally; L : Natural) return Integer is
      begin
         for I in 0 .. Natural (T.Labels.Length) - 1 loop
            if T.Labels (I) = L then
               return I;
            end if;
         end loop;
         return -1;
      end Of_Label;
   begin
      FR.Reset (Gen, 131);
      for F in 0 .. Kf - 1 loop
         Base (F) := (R => Rot ([0.3, 0.2, 1.0], 1.5 * Deg * Long_Float (F)), T => [0.003 * Long_Float (F), 0.004 * Long_Float (F), 0.001 * Long_Float (F)]);
         Door (F) := Hinge (Door_Ax, Door_P, 10.0 * Deg * Long_Float (F));
         Drawer (F) := (R => Geom.Identity, T => Scl (Drawer_W, 0.03 * Long_Float ((F * 2) mod 5)));   --  0 / 0.06 / 0.03 / 0.09
      end loop;
      Side (Grid (-0.2, 0.0, 1, 0.0, 0.1, 4, -0.15), Id, 0);
      Side (Grid (-0.2, 0.0, 1, 0.0, 0.1, 4, 0.15), Id, 0);
      Side (Grid (-0.2, 0.08, 6, 0.3, 0.0, 1, 0.15), Id, 0);
      Side (Grid (-0.2, 0.08, 6, 0.3, 0.0, 1, -0.15), Id, 0);
      Side (Grid (0.21, 0.06, 6, -0.02, 0.0, 1, 0.1), Door, 1);      --  门:柜子前面,轴在 x = 0.2 竖着
      Side (Grid (0.21, 0.06, 6, -0.02, 0.0, 1, -0.1), Door, 1);
      Side (Grid (0.21, 0.06, 6, -0.02, 0.0, 1, 0.0), Door, 1);
      Side (Grid (0.21, 0.06, 6, -0.02, 0.0, 1, 0.05), Door, 1);
      Side (Grid (-0.15, 0.06, 6, -0.03, 0.0, 1, -0.3), Drawer, 2);  --  抽屉:柜子下面一格,脸朝 −y
      Side (Grid (-0.15, 0.06, 6, -0.03, 0.0, 1, -0.25), Drawer, 2);
      Side (Grid (-0.15, 0.06, 6, -0.03, 0.0, 1, -0.35), Drawer, 2);
      for J in 1 .. 8 loop
         Add_Junk (Tracks, Sc, Kf, 0.45);
         Truth.Append (-1);
         Rel_Move.Append (0.0);
      end loop;
      Linkage.Fit (Tracks, Rep);
      Put_Line ("    三块:" & Linkage.Say (Rep));
      declare
         T : constant Tally := Count_Roles (Rep, Truth, Rel_Move, Long_Float'Last);
         Three : constant Boolean := Natural (Rep.Pieces.Length) = 3 and then Natural (Rep.Joints.Length) = 3;
         Pc, Pd, Pw : Integer;
         Ok_Door, Ok_Drawer, Ok_Pair : Boolean := False;
         Msg : Unbounded_String;
         R : constant Natural := Rep.Ref;
      begin
         if Three then
            Pc := Of_Label (T, 0);
            Pd := Of_Label (T, 1);
            Pw := Of_Label (T, 2);
            for J of Rep.Joints loop
               declare
                  Has_C : constant Boolean := Integer (J.A) = Pc or else Integer (J.B) = Pc;
                  Has_D : constant Boolean := Integer (J.A) = Pd or else Integer (J.B) = Pd;
                  Has_W : constant Boolean := Integer (J.A) = Pw or else Integer (J.B) = Pw;
               begin
                  if Has_C and then Has_D then
                     declare
                        Wt : constant V3 := Dir_W (Sc, Geom.Ap (Base (R).R, Door_Ax));
                        Pt : constant V3 := To_W (Sc, Apply (Base (R), Door_P));
                        Ang : constant Long_Float := Arccos (Long_Float'Min (1.0, abs Dot (J.Ax.W, Wt)));
                        Pdist : constant Long_Float := Line_Dist (J.Ax.P, Pt, Wt);
                     begin
                        Ok_Door := J.Status = Linkage.Found and then not J.Ax.Slide and then Ang <= Z2 * J.W_Sd and then Pdist <= Z2 * J.P_Sd;
                        Append (Msg, " · 柜–门 " & J.Status'Image & (if J.Ax.Slide then "(滑)" else "(转)") & " 方向差 " & Codec.Fmt (Ang / Deg, 2)
                                & "°(±" & Codec.Fmt (J.W_Sd / Deg, 2) & ")轴差 " & Codec.Fmt (Pdist, 4) & "(±" & Codec.Fmt (J.P_Sd, 4) & ")");
                     end;
                  elsif Has_C and then Has_W then
                     declare
                        Wt : constant V3 := Dir_W (Sc, Geom.Ap (Base (R).R, Drawer_W));
                        Ang : constant Long_Float := Arccos (Long_Float'Min (1.0, abs Dot (J.Ax.W, Wt)));
                     begin
                        Ok_Drawer := J.Status = Linkage.Found and then J.Ax.Slide and then Ang <= Z2 * J.W_Sd;
                        Append (Msg, " · 柜–抽屉 " & J.Status'Image & (if J.Ax.Slide then "(滑)" else "(转)") & " 方向差 " & Codec.Fmt (Ang / Deg, 2)
                                & "°(±" & Codec.Fmt (J.W_Sd / Deg, 2) & ")");
                     end;
                  elsif Has_D and then Has_W then
                     Ok_Pair := J.Status = Linkage.Not_One_Axis and then J.Chi > J.Chi_Gate;
                     Append (Msg, " · 门–抽屉 " & J.Status'Image & " 卡方 " & Codec.Fmt (J.Chi, 1) & "(门 " & Codec.Fmt (J.Chi_Gate, 1) & ",自由度 "
                             & Codec.Img (J.Dof) & ")");
                  end if;
               end;
            end loop;
         end if;
         Check (Three and then T.Wrong + T.Real_Lost <= T.Lost_Bound and then T.Junk_In = 0 and then Ok_Door and then Ok_Drawer and then Ok_Pair,
                "三块:" & Codec.Img (Natural (Rep.Pieces.Length)) & " 块(" & Sizes (Rep) & ",真的 " & Img_Ints (T.Labels) & ")· 归错 "
                & Codec.Img (T.Wrong) & " · 坏点进块 " & Codec.Img (T.Junk_In) & " · 真点说不通 " & Codec.Img (T.Real_Lost) & "(上限 "
                & Codec.Img (T.Lost_Bound) & ")· 噪声倍数 " & Codec.Fmt (Rep.Sigma, 3) & To_String (Msg));
      end;
   end;

   --  🔴 没见过它动 / 只见过它整块动 ⇒ 当一整块(Linkage.Fit):一只盒子表面 40 个点。① 三帧都不动 ⇒ 一块、Moved = False、没有轴;
   --  ② 绕穿过它自己的一根轴整块转 0 / 12 / 24° ⇒ 一块、Moved = True、没有轴;两次都混进 5 个坏点,都说不通。
   --  错了会是什么病:按"挪没挪"分块 —— 整块转时轴附近的点几乎不挪,被分成另一块,凭空多出一根轴。
   --  牙:按"挪没挪"分,② 里没挪的 / 挪了的两组都不空
   declare
      Kf : constant := 3;
      Box : Geom.V3_Vectors.Vector;
      Sc : constant Scene := (Sr => Rot ([0.1, -0.7, 0.3], 1.2), St => [0.5, 0.5, 0.5], Us => 3.0, Vd => [0.3, -0.2, 0.93], Sa => 0.002, Sd => 0.005,
                              Under => 1.0);
   begin
      for P of Grid (-0.1, 0.05, 5, -0.1, 0.05, 5, 0.08) loop
         Box.Append (P);
      end loop;
      for P of Grid (-0.1, 0.05, 5, -0.12, 0.0, 1, -0.06) loop
         Box.Append (P);
      end loop;
      for I in 0 .. 9 loop
         Box.Append (V3'[0.12, -0.1 + 0.02 * Long_Float (I), -0.02 + 0.01 * Long_Float (I mod 3)]);
      end loop;
      for Moving in Boolean loop
         declare
            Spin : Motions := [others => Still];
            Tracks : Linkage.Track_Vectors.Vector;
            Rep : Linkage.Report;
            Unex : Natural := 0;
            N_Still, N_Moved : Natural := 0;
         begin
            FR.Reset (Gen, (if Moving then 92 else 91));
            for F in 0 .. Kf - 1 loop
               if Moving then
                  Spin (F) := Hinge ([0.0, 0.0, 1.0], [0.0, 0.0, 0.0], 12.0 * Deg * Long_Float (F));
               end if;
            end loop;
            for P of Box loop
               Add_Track (Tracks, Sc, Kf, Spin, [others => Still], P, 0.0);
               if Sc.Us * Geom.Norm (Sub (Apply (Spin (Kf - 1), P), P)) <= Z * Sc.Us * Sc.Sd then
                  N_Still := N_Still + 1;
               else
                  N_Moved := N_Moved + 1;
               end if;
            end loop;
            for J in 1 .. 5 loop
               Add_Junk (Tracks, Sc, Kf, 0.35);
            end loop;
            Linkage.Fit (Tracks, Rep);
            Put_Line ("    " & (if Moving then "整块转" else "不动") & ":" & Linkage.Say (Rep));
            for I in Natural (Box.Length) .. Natural (Tracks.Length) - 1 loop
               if Rep.Roles (I) = Linkage.Unexplained then
                  Unex := Unex + 1;
               end if;
            end loop;
            Check (Natural (Rep.Pieces.Length) = 1 and then Rep.Joints.Is_Empty and then Rep.Moved = Moving
                   and then Natural (Rep.Pieces (0).Members.Length) + Chance_Bound (Natural (Box.Length)) >= Natural (Box.Length) and then Unex = 5
                   and then (not Moving or else (N_Still > 0 and then N_Moved > 0)),
                   (if Moving then "整块转 ⇒ 当一整块" else "没见过它动 ⇒ 当一整块") & ":" & Codec.Img (Natural (Rep.Pieces.Length)) & " 块("
                   & Sizes (Rep) & " / " & Codec.Img (Natural (Box.Length)) & ")· 轴 " & Codec.Img (Natural (Rep.Joints.Length)) & " 根 · 动过 "
                   & Rep.Moved'Image & " · 坏点说不通 " & Codec.Img (Unex) & "/5"
                   & (if Moving then " · 牙:按挪没挪分,没挪 " & Codec.Img (N_Still) & " / 挪了 " & Codec.Img (N_Moved) & "(两组都不空)" else ""));
         end;
      end loop;
   end;

   --  🔴 同一件东西上隔得远的两小团点 ⇒ 一块(Linkage.Fit 里两块合不合):两小团各 3 × 3 个点(间距 0.05,一团只有 0.1 宽),相隔 4;
   --  一起转 0 / 6 / 12 / 18° + 平移,混进 5 个坏点。一团自己的位姿转得定不准,外推到 4 以外差出十来倍噪声 —— 一块一块往外拿时一定拿成两块;
   --  两块合成一块比各自一块多出来的卡方不显著(同一个刚体)⇒ 合起来。要:一块、18 个点都归进去、没有轴、坏点都说不通。
   --  错了会是什么病:不合 ⇒ 同一件东西报成两块、中间凭空一根"没动过"的轴;两块对同一批点说法一样时还会全成"分不清"、两块一起被拿掉
   --  (09-30 离线铰链:底座被拆成两块,底座 45 个真点全说不通)。牙:拿掉合并(Assign 每一轮开头的 Merge_Same)⇒ 两块
   declare
      Kf : constant := 4;
      Pts : Geom.V3_Vectors.Vector;
      Sc : constant Scene := (Sr => Rot ([0.4, 0.2, -0.9], 0.7), St => [-0.3, 0.8, 0.2], Us => 2.0, Vd => [0.1, 0.3, 0.95], Sa => 0.002, Sd => 0.005,
                              Under => 1.0);
      Spin : Motions := [others => Still];
      Tracks : Linkage.Track_Vectors.Vector;
      Rep : Linkage.Report;
      Unex : Natural := 0;
   begin
      FR.Reset (Gen, 151);
      for P of Grid (-2.05, 0.05, 3, -0.05, 0.05, 3, 0.0) loop
         Pts.Append (P);
      end loop;
      for P of Grid (1.95, 0.05, 3, -0.05, 0.05, 3, 0.0) loop
         Pts.Append (P);
      end loop;
      for F in 0 .. Kf - 1 loop
         Spin (F) := (R => Rot ([0.2, 0.3, 1.0], 6.0 * Deg * Long_Float (F)), T => [0.05 * Long_Float (F), -0.03 * Long_Float (F), 0.02 * Long_Float (F)]);
      end loop;
      for P of Pts loop
         Add_Track (Tracks, Sc, Kf, Spin, [others => Still], P, 0.0);
      end loop;
      for J in 1 .. 5 loop
         Add_Junk (Tracks, Sc, Kf, 2.5);
      end loop;
      Linkage.Fit (Tracks, Rep);
      Put_Line ("    隔得远的两小团:" & Linkage.Say (Rep));
      for I in Natural (Pts.Length) .. Natural (Tracks.Length) - 1 loop
         if Rep.Roles (I) = Linkage.Unexplained then
            Unex := Unex + 1;
         end if;
      end loop;
      Check (Natural (Rep.Pieces.Length) = 1 and then Rep.Joints.Is_Empty and then Rep.Moved
             and then Natural (Rep.Pieces (0).Members.Length) + Chance_Bound (Natural (Pts.Length)) >= Natural (Pts.Length) and then Unex = 5,
             "隔得远的两小团 ⇒ 一块:" & Codec.Img (Natural (Rep.Pieces.Length)) & " 块(" & Sizes (Rep) & " / " & Codec.Img (Natural (Pts.Length))
             & ")· 轴 " & Codec.Img (Natural (Rep.Joints.Length)) & " 根 · 动过 " & Rep.Moved'Image & " · 坏点说不通 " & Codec.Img (Unex) & "/5");
   end;

   --  🔴 点比噪声密 ⇒ 照样找得到起步(Linkage.Fit 的种子):一块 6 × 6 个点的板(间距 0.05),视线方向的噪声 0.0125(间距只有它的 4 倍),
   --  转 0 / 5 / 10 / 15° + 平移。种子要的是"挨着的一小团、显著铺开":只配两个最近的点不够 —— 两点挨着、第三点再远,三角形的宽也超不过
   --  那两点的间距,这里一个也过不了"显著铺开"(间距² / 2 < 门 × 噪声²);一条条加最近的点,加到五个(十字)就过了。
   --  要:一块、36 个点都归进去。错了会是什么病:一个起步都没有 ⇒ 一块都归不出、整块东西全"说不通"
   --  (09-30 离线铰链:噪声倍数起步量大了三成,96 个真点一个起步都没有,只剩坏点配远处两个真点的那几个)。
   --  牙:种子只配最近的两个点(老的三点起步)⇒ 0 块
   declare
      Kf : constant := 4;
      Sc : constant Scene := (Sr => Rot ([-0.5, 0.4, 0.3], 0.9), St => [0.2, -0.6, 1.1], Us => 4.0, Vd => [0.05, -0.1, 0.99], Sa => 0.002, Sd => 0.0125,
                              Under => 1.0);
      Plate : constant Geom.V3_Vectors.Vector := Grid (-0.125, 0.05, 6, -0.125, 0.05, 6, 0.0);
      Spin : Motions := [others => Still];
      Tracks : Linkage.Track_Vectors.Vector;
      Rep : Linkage.Report;
   begin
      FR.Reset (Gen, 161);
      for F in 0 .. Kf - 1 loop
         Spin (F) := (R => Rot ([0.3, -0.2, 1.0], 5.0 * Deg * Long_Float (F)), T => [0.02 * Long_Float (F), 0.01 * Long_Float (F), -0.015 * Long_Float (F)]);
      end loop;
      for P of Plate loop
         Add_Track (Tracks, Sc, Kf, Spin, [others => Still], P, 0.0);
      end loop;
      Linkage.Fit (Tracks, Rep);
      Put_Line ("    点比噪声密:" & Linkage.Say (Rep));
      Check (Natural (Rep.Pieces.Length) = 1 and then Rep.Moved
             and then Natural (Rep.Pieces (0).Members.Length) + Chance_Bound (Natural (Plate.Length)) >= Natural (Plate.Length),
             "点比噪声密 ⇒ 照样找得到起步:" & Codec.Img (Natural (Rep.Pieces.Length)) & " 块(" & Sizes (Rep) & " / " & Codec.Img (Natural (Plate.Length))
             & ")· 动过 " & Rep.Moved'Image & " · 噪声倍数 " & Codec.Fmt (Rep.Sigma, 3));
   end;

   --  🔴 顺着它让的方向走(Linkage.Follow):手拿着一扇门的把手(离轴 0.8,门能开到 100°),脑要把手往"和门开的方向差 50°、偏向离轴那一侧"挪,
   --  到门开 35° 为止。手是位置控制的:命令的一步里垂直于门能走的那一截被门顶着(顶着的那一截 > 0.9 步 ⇒ 把手从手里滑出去),
   --  沿门能走的那一截门跟着转。眼量把手挪了多少,噪声 0.002;手靠得住的最小一步 0.03(Light_Len 按这两个定一步多长)。
   --  要:门开到 35°、把手一次都没滑出去。错了会是什么病:一直按脑说的方向直着推 ⇒ 门越开,推的方向越顶着门(顶着的那一截越来越大),
   --  14° 左右把手就滑出去了。牙:同一扇门一直沿 Want 推
   declare
      Rho : constant Long_Float := 0.8;
      Phi : Long_Float := 0.0;
      Phi_Max : constant Long_Float := 100.0 * Deg;
      Slipped : Boolean := False;
      Sig_T : constant Long_Float := 0.002;
      Sig_H : constant Long_Float := 0.0001;
      Ct : constant M3 := [[Sig_T ** 2, 0.0, 0.0], [0.0, Sig_T ** 2, 0.0], [0.0, 0.0, Sig_T ** 2]];
      Ch : constant M3 := [[Sig_H ** 2, 0.0, 0.0], [0.0, Sig_H ** 2, 0.0], [0.0, 0.0, Sig_H ** 2]];
      Len : constant Long_Float := Linkage.Light_Len (0.03, Ct);
      Grip : constant Long_Float := 0.9 * Len;
      Max_Off : Long_Float := 0.0;
      function Pos (A : Long_Float) return V3 is ([Rho * Cos (A), Rho * Sin (A), 0.0]);
      procedure Door_Step (Dir : V3; L : Long_Float; R : out Linkage.Step_Report) is
         D : constant V3 := Scl (Dir, L);
         T : constant V3 := [-Sin (Phi), Cos (Phi), 0.0];
         N : constant V3 := [Cos (Phi), Sin (Phi), 0.0];
         Off : constant Long_Float := Sqrt (Dot (D, N) ** 2 + D (2) ** 2);   --  顶着门的那一截
         Real : V3 := [others => 0.0];
      begin
         R.Ok := True;
         R.Hand_Cov := Ch;
         R.Thing_Cov := Ct;
         if Slipped or else Off > Grip then
            Slipped := True;
            R.Hand := Add (D, [Sig_H * Gauss, Sig_H * Gauss, Sig_H * Gauss]);
            R.Thing := [Sig_T * Gauss, Sig_T * Gauss, Sig_T * Gauss];
            R.Blocked := False;
            return;
         end if;
         Max_Off := Long_Float'Max (Max_Off, Off);
         declare
            New_Phi : constant Long_Float := Long_Float'Max (0.0, Long_Float'Min (Phi_Max, Phi + Dot (D, T) / Rho));
         begin
            Real := Sub (Pos (New_Phi), Pos (Phi));
            Phi := New_Phi;
         end;
         R.Hand := Add (Real, [Sig_H * Gauss, Sig_H * Gauss, Sig_H * Gauss]);
         R.Thing := Add (Real, [Sig_T * Gauss, Sig_T * Gauss, Sig_T * Gauss]);
         R.Blocked := Selfmap.Blocked (L - Dot (Real, Dir), 0.0, 0.0, Selfmap.Free_Base, L, Sig_H);
      end Door_Step;
      function Door_Goal return Boolean is (Phi >= 35.0 * Deg);
      Want : constant V3 := [Sin (50.0 * Deg), Cos (50.0 * Deg), 0.0];
      Fw : Linkage.Follow_Report;
      Phi_F, Off_F : Long_Float;
      Slip_F : Boolean;
      Naive_Phi : Long_Float;
      Naive_Slip : Boolean;
   begin
      FR.Reset (Gen, 101);
      Linkage.Follow (Want, Len, 400, Door_Step'Access, Door_Goal'Access, Fw);
      Phi_F := Phi;
      Slip_F := Slipped;
      Off_F := Max_Off;
      Phi := 0.0;
      Slipped := False;
      for I in 1 .. 400 loop
         declare
            R : Linkage.Step_Report;
         begin
            Door_Step (Unit (Want), Len, R);
         end;
         exit when Slipped or else Door_Goal;
      end loop;
      Naive_Phi := Phi;
      Naive_Slip := Slipped;
      Check (Fw.How = Linkage.Arrived and then Phi_F >= 35.0 * Deg and then not Slip_F and then Naive_Slip,
             "顺着让的方向走·门:" & Fw.How'Image & " · " & Codec.Img (Fw.Steps) & " 步(一步 " & Codec.Fmt (Len, 4) & ")门开到 "
             & Codec.Fmt (Phi_F / Deg, 1) & "° · 滑出去 " & Slip_F'Image & " · 顶着门最多 " & Codec.Fmt (Off_F / Len, 2) & " 步(握得住 0.9 步)"
             & " · 牙:一直沿 Want 推 ⇒ 门开到 " & Codec.Fmt (Naive_Phi / Deg, 1) & "° 滑出去 " & Naive_Slip'Image);
   end;

   --  🔴 哪边都不让 ⇒ 照实说试过哪几个方向(Linkage.Follow):① 锁死的东西:沿 Want、垂直于它的 ±E1、±E2 各试一小步都被挡住、都没挪
   --  ⇒ Stuck,Tried 正好是这 5 个方向(Want 本身 + 两对互相反向、互相垂直、都垂直于 Want 的单位向量),Yields 空;
   --  ② 只能沿一根和 Want 垂直的滑轴走的东西:沿 Want 推不动,试到沿滑轴那一对时它挪了,可挪的方向和 Want 垂直 ⇒ 不跟着走,
   --  Stuck,Yields 里记着它往哪让过(沿滑轴的两个方向)。
   --  错了会是什么病:第一个方向推不动就说"推不动"(没试别的方向)⇒ 脑听到的"我试过的"只有一个方向,不知道它其实能沿别的方向让开;
   --  或者看它挪了就跟着走 ⇒ 沿一个和脑要的无关的方向把东西拖走。牙:只试 Want 一个方向 ⇒ Tried 只有 1 个、Yields 空(代码里拿掉试方向跑过)
   declare
      Sig_T : constant Long_Float := 0.002;
      Ct : constant M3 := [[Sig_T ** 2, 0.0, 0.0], [0.0, Sig_T ** 2, 0.0], [0.0, 0.0, Sig_T ** 2]];
      Ch : constant M3 := [[1.0e-8, 0.0, 0.0], [0.0, 1.0e-8, 0.0], [0.0, 0.0, 1.0e-8]];
      Len : constant Long_Float := Linkage.Light_Len (0.03, Ct);
      Want : constant V3 := Unit ([1.0, 0.3, 0.2]);
      Rail : constant V3 := Unit ([-0.3, 1.0, 0.0]);   --  和 Want 垂直:Want · Rail = −0.3 + 0.3 + 0 = 0
      Mode : Natural := 0;   --  0 = 锁死,1 = 沿 Rail 滑
      Pos_S, Far_S : Long_Float := 0.0;   --  沿 Rail 滑到哪、离起点最远多远
      procedure Stuck_Step (Dir : V3; L : Long_Float; R : out Linkage.Step_Report) is
         Real : constant V3 := (if Mode = 0 then [0.0, 0.0, 0.0] else Scl (Rail, L * Dot (Dir, Rail)));
      begin
         Pos_S := Pos_S + Dot (Real, Rail);
         Far_S := Long_Float'Max (Far_S, abs Pos_S);
         R.Ok := True;
         R.Hand_Cov := Ch;
         R.Thing_Cov := Ct;
         R.Hand := Real;
         R.Thing := Add (Real, [Sig_T * Gauss, Sig_T * Gauss, Sig_T * Gauss]);
         R.Blocked := Selfmap.Blocked (L - Dot (Real, Dir), 0.0, 0.0, Selfmap.Free_Base, L, 0.0);
      end Stuck_Step;
      Fw0, Fw1 : Linkage.Follow_Report;
      function Probes_Ok (F : Linkage.Follow_Report) return Boolean is
      begin
         if Natural (F.Tried.Length) /= 5 then
            return False;
         end if;
         declare
            T0 : constant V3 := F.Tried (0);
            T1 : constant V3 := F.Tried (1);
            T2 : constant V3 := F.Tried (2);
            T3 : constant V3 := F.Tried (3);
            T4 : constant V3 := F.Tried (4);
            Eps : constant Long_Float := 1.0e-9;
         begin
            return abs (Dot (T0, Want) - 1.0) < Eps and then abs Dot (T1, Want) < Eps and then abs Dot (T3, Want) < Eps
              and then abs Dot (T1, T3) < Eps and then Geom.Norm (Add (T1, T2)) < Eps and then Geom.Norm (Add (T3, T4)) < Eps
              and then abs (Geom.Norm (T1) - 1.0) < Eps and then abs (Geom.Norm (T3) - 1.0) < Eps;
         end;
      end Probes_Ok;
      Along_Rail : Boolean := True;
   begin
      FR.Reset (Gen, 111);
      Mode := 0;
      Linkage.Follow (Want, Len, 50, Stuck_Step'Access, null, Fw0);
      Mode := 1;
      Linkage.Follow (Want, Len, 50, Stuck_Step'Access, null, Fw1);
      --  它让过的方向要沿着滑轴:沿滑轴试的那一步它挪一整步 Len,量到的方向不准 ≈ √2 σ / Len(垂直于它的两个分量),Z 倍以内
      for Y of Fw1.Yields loop
         Along_Rail := Along_Rail and then abs Dot (Y, Rail) > Cos (Z * Sqrt (2.0) * Sig_T / Len);
      end loop;
      Check (Fw0.How = Linkage.Stuck and then Fw0.Steps = 5 and then Probes_Ok (Fw0) and then Fw0.Yields.Is_Empty
             and then Fw1.How = Linkage.Stuck and then Probes_Ok (Fw1) and then not Fw1.Yields.Is_Empty and then Along_Rail
             and then Far_S <= Len * (1.0 + 1.0e-9),
             "哪边都不让 ⇒ 照实说:锁死的 " & Fw0.How'Image & "(试了 " & Codec.Img (Natural (Fw0.Tried.Length)) & " 个方向、" & Codec.Img (Fw0.Steps)
             & " 步,让过 " & Codec.Img (Natural (Fw0.Yields.Length)) & " 次)· 只能垂直于 Want 滑的 " & Fw1.How'Image & "(试了 "
             & Codec.Img (Natural (Fw1.Tried.Length)) & " 个,它沿滑轴让过 " & Codec.Img (Natural (Fw1.Yields.Length)) & " 次,离起点最远 "
             & Codec.Fmt (Far_S / Len, 2) & " 步 —— 没跟着拖走)");
   end;

   --  🔴 顺着一根没对准的滑轴走到头(Linkage.Follow):抽屉的滑轴和 Want 差 20°,能拉出 0.2;脑没说到哪为止(一直走)。
   --  要:抽屉拉到头(0.2)、到头以后 5 个方向都试过 ⇒ Stuck,到头以后花的步数不超过"被挡住的那一步 + 每个试的方向一步、再各跟一步"(1 + 2 × 5)。
   --  错了会是什么病:把"挪了"不按噪声判(量到的位移不是 0 就算挪了)⇒ 到头以后每一步的噪声都被当成"它往这边让了",跟着噪声乱走,
   --  永远说不出"到头了",步数耗光;"顺了"按每一段自己算(不和走到过的最远处比)⇒ 一个试的方向把它往回挪一点、下一个又推回去,
   --  算成新的进展,试过的方向清掉,到头以后来回打转上百步。牙:代码里把这两处各拿掉跑过 ⇒ Out_Of_Steps / 到头以后 150 步
   declare
      Sig_T : constant Long_Float := 0.002;
      Ct : constant M3 := [[Sig_T ** 2, 0.0, 0.0], [0.0, Sig_T ** 2, 0.0], [0.0, 0.0, Sig_T ** 2]];
      Ch : constant M3 := [[1.0e-8, 0.0, 0.0], [0.0, 1.0e-8, 0.0], [0.0, 0.0, 1.0e-8]];
      Len : constant Long_Float := Linkage.Light_Len (0.03, Ct);
      Want : constant V3 := [1.0, 0.0, 0.0];
      Rail : constant V3 := [Cos (20.0 * Deg), Sin (20.0 * Deg), 0.0];
      Travel : constant Long_Float := 0.2;
      S : Long_Float := 0.0;
      Max_S : Long_Float := 0.0;
      Steps, End_Step : Natural := 0;
      procedure Rail_Step (Dir : V3; L : Long_Float; R : out Linkage.Step_Report) is
         New_S : constant Long_Float := Long_Float'Max (0.0, Long_Float'Min (Travel, S + L * Dot (Dir, Rail)));
         Real : constant V3 := Scl (Rail, New_S - S);
      begin
         Steps := Steps + 1;
         if New_S >= Travel and then End_Step = 0 then
            End_Step := Steps;
         end if;
         S := New_S;
         Max_S := Long_Float'Max (Max_S, S);
         R.Ok := True;
         R.Hand_Cov := Ch;
         R.Thing_Cov := Ct;
         R.Hand := Real;
         R.Thing := Add (Real, [Sig_T * Gauss, Sig_T * Gauss, Sig_T * Gauss]);
         R.Blocked := Selfmap.Blocked (L - Dot (Real, Dir), 0.0, 0.0, Selfmap.Free_Base, L, 0.0);
      end Rail_Step;
      Fw : Linkage.Follow_Report;
   begin
      FR.Reset (Gen, 121);
      Linkage.Follow (Want, Len, 300, Rail_Step'Access, null, Fw);
      Check (Fw.How = Linkage.Stuck and then abs (Max_S - Travel) < 1.0e-12 and then Natural (Fw.Tried.Length) >= 5
             and then End_Step > 0 and then Fw.Steps - End_Step <= 1 + 2 * 5,
             "顺着没对准的滑轴走到头:" & Fw.How'Image & " · 抽屉最远到 " & Codec.Fmt (Max_S, 4) & "(头在 " & Codec.Fmt (Travel, 2) & ")· "
             & Codec.Img (Fw.Steps) & " 步(第 " & Codec.Img (End_Step) & " 步到头)· 沿 Want 挪了 " & Codec.Fmt (Fw.Along, 4) & " · 到头以后试了 "
             & Codec.Img (Natural (Fw.Tried.Length)) & " 个方向");
   end;

   --  🔴 拿着的东西跟着手走(Held.Take / Shape_Cov / In_World / Check_Slip,§2 第 20 条):手拿着一件东西(15 个点,在手的系里离手 0.06–0.20),
   --  手转 0 / 10 / 20 / 30 / 40° 同时抬着走;桌上 8 个点不动;混进 4 个跟错了的点。手的位姿是按关节读数算的,不准 0.5°、0.003
   --  (每一帧真的手都按这个不准偏一点);眼的噪声横着 0.001、沿视线 0.003。
   --  要:① 15 个点跟着手走、8 个留在世界里、4 个哪种都不是,没有一个归错;它在手的系里的形状在自报的不准以内;
   --     ② 手没动的那一场:一个都不说"拿着"(两种说法预测到同一处,照实说判不了);
   --     ③ 再看一眼:没滑 ⇒ 不说滑了;整件东西在手里转了 8°、挪了 0.003(每个点单看都在门里:手此刻的不准每个点都摊着)⇒ 滑了。
   --  错了会是什么病:手的不准不算进"长在手上"那一说 ⇒ 拿得好好的点被说成"自己在动";不看两种说法隔不隔得开 ⇒ 手没动时桌上的点也说成拿着;
   --  一个个点比滑没滑 ⇒ 整件东西一起转了一点,一个点都不过门,说没滑;手此刻的不准不算 ⇒ 没滑也说滑了。
   --  牙:① 手的不准当 0 ⇒ 跟着手走的少一大截;③ 一个个点比 ⇒ 转了 8° 一个都不过门;手此刻的不准当 0 ⇒ 没滑的那一眼说滑了;
   --  ② 拿掉"隔不开就判不了"那一条 ⇒ 手没动时 23 个点说成拿着(离线跑过,见报告)
   declare
      Kf : constant := 5;
      Vd : constant V3 := Unit ([0.2, 0.3, 0.93]);
      Le, Ce : M3;
      Rot_Sd : constant Long_Float := 0.5 * Deg;
      Pos_Sd : constant Long_Float := 0.003;
      Ch : Held.Cov6 := Held.Zero6;
      Obj : constant Geom.V3_Vectors.Vector := Grid (0.06, 0.035, 5, -0.03, 0.03, 3, -0.04);
      Tab : constant Geom.V3_Vectors.Vector := Grid (0.0, 0.1, 4, -0.1, 0.2, 2, 0.0);
      No : constant Natural := Natural (Obj.Length);
      Nt : constant Natural := Natural (Tab.Length);
      Nj : constant := 4;
      R0 : constant M3 := Rot ([0.3, 1.0, 0.2], 0.4);
      T0 : constant V3 := [0.1, 0.0, 0.08];
      function Noise return V3 is (Geom.Ap (Le, [Gauss, Gauss, Gauss]));
      function Sum3 (A, B : M3) return M3 is
         C : M3;
      begin
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               C (I, J) := A (I, J) + B (I, J);
            end loop;
         end loop;
         return C;
      end Sum3;
      --  手第 F 帧:驱动以为的(Est)、真的(Truth:按手的不准偏一点)
      type Pair is record
         Est, Truth : Held.Hand_Pose;
      end record;
      function Hand_At (F : Natural; Moving : Boolean) return Pair is
         Ang : constant Long_Float := (if Moving then 10.0 * Deg * Long_Float (F) else 0.0);
         Est : constant Held.Hand_Pose :=
           (Ok => True, R => Geom.Mul (Rot ([0.5, -0.2, 1.0], Ang), R0),
            T => Add (T0, (if Moving then Scl ([0.03, -0.02, 0.05], Long_Float (F)) else [0.0, 0.0, 0.0])), Cov => Ch);
         Om : constant V3 := [Rot_Sd * Gauss, Rot_Sd * Gauss, Rot_Sd * Gauss];
         Ta : constant V3 := [Pos_Sd * Gauss, Pos_Sd * Gauss, Pos_Sd * Gauss];
      begin
         return (Est => Est, Truth => (Ok => True, R => Geom.Mul (Geom.Rodrigues (Om), Est.R), T => Add (Est.T, Ta), Cov => Ch));
      end Hand_At;
      procedure Scene_Tracks (Moving : Boolean; Hands : out Held.Hand_Vectors.Vector; Tracks : out Linkage.Track_Vectors.Vector) is
         Ps : array (0 .. Kf - 1) of Pair;
      begin
         Hands.Clear;
         Tracks.Clear;
         for F in 0 .. Kf - 1 loop
            Ps (F) := Hand_At (F, Moving);
            Hands.Append (Ps (F).Est);
         end loop;
         for P of Obj loop
            declare
               O : Linkage.Obs_Vectors.Vector;
            begin
               for F in 0 .. Kf - 1 loop
                  O.Append (Linkage.Obs'(Seen => True, X => Add (Add (Geom.Ap (Ps (F).Truth.R, P), Ps (F).Truth.T), Noise), Cov => Ce));
               end loop;
               Tracks.Append (O);
            end;
         end loop;
         for P of Tab loop
            declare
               O : Linkage.Obs_Vectors.Vector;
            begin
               for F in 0 .. Kf - 1 loop
                  O.Append (Linkage.Obs'(Seen => True, X => Add (P, Noise), Cov => Ce));
               end loop;
               Tracks.Append (O);
            end;
         end loop;
         for J in 1 .. Nj loop
            declare
               O : Linkage.Obs_Vectors.Vector;
            begin
               for F in 0 .. Kf - 1 loop
                  O.Append (Linkage.Obs'(Seen => True, X => [Uni (-0.2, 0.4), Uni (-0.3, 0.3), Uni (0.0, 0.4)], Cov => Ce));
               end loop;
               Tracks.Append (O);
            end;
         end loop;
      end Scene_Tracks;
      function Obj_Rides (Rep : Held.Take_Report) return Natural is
         C : Natural := 0;
      begin
         for I in 0 .. No - 1 loop
            if Rep.Roles (I) = Held.Rides then
               C := C + 1;
            end if;
         end loop;
         return C;
      end Obj_Rides;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            Le (I, J) := 0.001 * ((if I = J then 1.0 else 0.0) - Vd (I) * Vd (J)) + 0.003 * Vd (I) * Vd (J);
         end loop;
      end loop;
      Ce := Geom.Mul (Le, Geom.Tr (Le));
      for D in 0 .. 2 loop
         Ch (D, D) := Rot_Sd ** 2;
         Ch (D + 3, D + 3) := Pos_Sd ** 2;
      end loop;
      FR.Reset (Gen, 171);
      declare
         Hands : Held.Hand_Vectors.Vector;
         Tracks : Linkage.Track_Vectors.Vector;
         P, P0 : Held.Part;
         Rep, Rep0 : Held.Take_Report;
         Tab_World, Junk_Neither, Wrong, Shape_Out : Natural := 0;
      begin
         Scene_Tracks (True, Hands, Tracks);
         Held.Take (Hands, Tracks, 0, 1.0, Long_Float'Last, P, Rep);
         Put_Line ("    手在动:" & Held.Say (Rep));
         for I in No .. Natural (Tracks.Length) - 1 loop
            if I < No + Nt and then Rep.Roles (I) = Held.World then
               Tab_World := Tab_World + 1;
            elsif I >= No + Nt and then Rep.Roles (I) = Held.Neither then
               Junk_Neither := Junk_Neither + 1;
            end if;
            if Rep.Roles (I) = Held.Rides then
               Wrong := Wrong + 1;
            end if;
         end loop;
         for K in 0 .. Natural (P.Tracks.Length) - 1 loop
            if P.Tracks (K) < No and then Linkage.Mahal (Sub (P.Pts (K), Obj (P.Tracks (K))), Held.Shape_Cov (P, K)) > Linkage.Gate (3) then
               Shape_Out := Shape_Out + 1;
            end if;
         end loop;
         --  牙:手的位姿当成准的(不准 = 0)
         declare
            Hands0 : Held.Hand_Vectors.Vector := Hands;
         begin
            for H of Hands0 loop
               H.Cov := Held.Zero6;
            end loop;
            Held.Take (Hands0, Tracks, 0, 1.0, Long_Float'Last, P0, Rep0);
         end;
         Check (Obj_Rides (Rep) + Chance_Bound (No) >= No and then Tab_World + Chance_Bound (Nt) >= Nt and then Junk_Neither = Nj and then Wrong = 0
                and then Shape_Out <= Chance_Bound (No) and then Obj_Rides (Rep0) + Chance_Bound (No) < No,
                "拿着的东西·认跟着手走的:" & Codec.Img (Obj_Rides (Rep)) & " / " & Codec.Img (No) & " 跟着手走 · 桌上 " & Codec.Img (Tab_World) & " / "
                & Codec.Img (Nt) & " 留在世界里 · 坏点 " & Codec.Img (Junk_Neither) & " / " & Codec.Img (Nj) & " 哪种都不是 · 归错 " & Codec.Img (Wrong)
                & " · 形状出了自报的不准 " & Codec.Img (Shape_Out) & " 个 · 牙:手的不准当 0 ⇒ 跟着手走的只剩 " & Codec.Img (Obj_Rides (Rep0)));
         --  ③ 再看一眼(手又挪了一步):没滑 / 在手里绕自己转了 8°、挪了 0.003
         declare
            Pr : constant Pair := Hand_At (Kf, True);
            Now_A, Now_B : Linkage.Obs_Vectors.Vector;
            Slip_R : constant M3 := Rot ([0.6, -0.5, 0.4], 8.0 * Deg);
            Slip_T : constant V3 := [0.003, -0.001, 0.0];
            Cen : V3 := [0.0, 0.0, 0.0];
            Sa, Sb, Sa0 : Held.Slip_Report;
            Per_Point_Out : Natural := 0;
         begin
            for Q of Obj loop
               Cen := Add (Cen, Scl (Q, 1.0 / Long_Float (No)));
            end loop;
            for K in 0 .. Natural (P.Tracks.Length) - 1 loop
               declare
                  S_True : constant V3 := Obj (P.Tracks (K));
                  S_Slip : constant V3 := Add (Add (Geom.Ap (Slip_R, Sub (S_True, Cen)), Cen), Slip_T);
               begin
                  Now_A.Append (Linkage.Obs'(Seen => True, X => Add (Add (Geom.Ap (Pr.Truth.R, S_True), Pr.Truth.T), Noise), Cov => Ce));
                  Now_B.Append (Linkage.Obs'(Seen => True, X => Add (Add (Geom.Ap (Pr.Truth.R, S_Slip), Pr.Truth.T), Noise), Cov => Ce));
               end;
            end loop;
            Held.Check_Slip (P, Pr.Est, Now_A, Sa);
            Held.Check_Slip (P, Pr.Est, Now_B, Sb);
            Put_Line ("    没滑那一眼:" & Held.Say (Sa));
            Put_Line ("    滑了那一眼:" & Held.Say (Sb));
            declare
               E0 : Held.Hand_Pose := Pr.Est;
            begin
               E0.Cov := Held.Zero6;
               Held.Check_Slip (P, E0, Now_A, Sa0);
            end;
            declare
               Wv : constant Held.World_View := Held.In_World (P, Pr.Est);
            begin
               for K in 0 .. Natural (Wv.Pts.Length) - 1 loop
                  if Linkage.Mahal (Sub (Now_B (K).X, Wv.Pts (K)), Sum3 (Wv.Covs (K), Ce)) > Linkage.Gate (3) then
                     Per_Point_Out := Per_Point_Out + 1;
                  end if;
               end loop;
            end;
            Check (not Sa.Slipped and then Sa.Dof = 6 and then Sb.Slipped and then Sa0.Slipped and then Per_Point_Out = 0,
                   "拿着的东西·滑没滑:没滑的那一眼 " & Codec.Fmt (Sa.Chi, 1) & "(门 " & Codec.Fmt (Sa.Gate, 1) & ")不说滑 · 转了 8° 那一眼 "
                   & Codec.Fmt (Sb.Chi, 1) & " 说滑了 · 牙:一个个点比,转了 8° 过门的 " & Codec.Img (Per_Point_Out) & " 个;手此刻的不准当 0 ⇒ 没滑的那一眼 "
                   & Codec.Fmt (Sa0.Chi, 1) & (if Sa0.Slipped then " 说滑了" else " 没说滑"));
         end;
      end;
      --  ② 手没动
      declare
         Hands : Held.Hand_Vectors.Vector;
         Tracks : Linkage.Track_Vectors.Vector;
         P : Held.Part;
         Rep : Held.Take_Report;
      begin
         Scene_Tracks (False, Hands, Tracks);
         Held.Take (Hands, Tracks, 0, 1.0, Long_Float'Last, P, Rep);
         Put_Line ("    手没动:" & Held.Say (Rep));
         Check (Rep.N_Rides = 0 and then Rep.N_Unknown = No + Nt + Nj,
                "拿着的东西·手没动 ⇒ 判不了:跟着手走 " & Codec.Img (Rep.N_Rides) & " · 判不了 " & Codec.Img (Rep.N_Unknown) & " / "
                & Codec.Img (No + Nt + Nj));
      end;
   end;

   --  🔴 对准、插进去(Seek.Run,§2 第 21 条):一个孔沿一根斜的轴往下,缝 0.0005(插进去的东西横着偏不到这么多就进得去)、深 0.01;
   --  眼量的孔口位置横着不准 0.0015、沿轴 0.001 —— 眼的不准是缝的 3 倍;手靠得住的最小一步 0.0002(每一步走完量它在哪,读数的噪声是它的三分之一)。
   --  真的孔口离估计横着沿垂直于轴的两根轴各偏半倍不准(正好在"一步一倍不准"那种粗格子的四个格点正中间)、沿轴深了半倍。
   --  要:进去了;送的次数不超过"比找到的那一格更可能的格点"那么多;沿轴送进去的够深。
   --  错了会是什么病:只在估计的那一处直着送 ⇒ 顶在孔边上进不去;横着找的步子按眼的不准定(一步 = 一倍不准)⇒ 格子比缝粗,
   --  正好把孔跳过去,不准那一片都试完了也没找到;孔真在不准那一片外面 ⇒ 不在外面瞎找,照实说没找到。
   --  牙:不准当 0(只在估计那一处送)⇒ 进不去;格子按一倍不准 ⇒ 不准那一片都试完了没找到;孔在 4 倍不准外 ⇒ 一片都试过、照实说没找到
   declare
      A : constant V3 := Unit ([0.3, -0.2, -0.93]);
      E1, E2 : V3;
      Sd_L : constant Long_Float := 0.0015;
      Sd_A : constant Long_Float := 0.001;
      Gap : constant Long_Float := 0.0005;
      Depth : constant Long_Float := 0.01;
      Depth_Sd : constant Long_Float := 0.0005;
      Res : constant Long_Float := 0.0002;
      Start : constant V3 := [0.1, 0.2, 0.3];
      Est : constant V3 := Add (Start, Scl (A, 0.004));
      Cov : M3;
      Hole : V3;
      Tip : V3;
      --  孔口在 Hole、沿 A 往里;a = 沿轴进了多深(孔口那一面 = 0),l = 横着离孔的轴多远。
      --  空着的地方:还没到那一面(a < 0),或者在孔里(0 ≤ a ≤ Depth、l ≤ Gap)
      function Free (P : V3) return Boolean is
         D : constant V3 := Sub (P, Hole);
         Ax : constant Long_Float := Dot (D, A);
         L : constant Long_Float := Geom.Norm (Sub (D, Scl (A, Ax)));
      begin
         return Ax < 0.0 or else (Ax <= Depth and then L <= Gap);
      end Free;
      --  身体走一步:沿 Dir 一小截一小截地走(最小一步的十分之一),碰到不空的地方就停;走完量它在哪(读数带噪声,最小一步的三分之一)
      procedure Sim_Move (Dir : V3; Len : Long_Float; R : out Seek.Step_Report) is
         Sub_L : constant Long_Float := Res / 10.0;
         Done : Long_Float := 0.0;
      begin
         while Done < Len loop
            declare
               Nx : constant Long_Float := Long_Float'Min (Len, Done + Sub_L);
            begin
               exit when not Free (Add (Tip, Scl (Dir, Nx - Done)));
               Tip := Add (Tip, Scl (Dir, Nx - Done));
               Done := Nx;
            end;
         end loop;
         R := (Ok => True, At_Now => Add (Tip, [Res / 3.0 * Gauss, Res / 3.0 * Gauss, Res / 3.0 * Gauss]), Blocked => Done < Len - Res);
      end Sim_Move;
      procedure Place (Lat1, Lat2, Ax_Off : Long_Float) is
      begin
         Hole := Add (Est, Add (Add (Scl (E1, Lat1 * Sd_L), Scl (E2, Lat2 * Sd_L)), Scl (A, Ax_Off * Sd_A)));
         Tip := Start;
      end Place;
      Zero : constant M3 := [others => [others => 0.0]];
      R_Ok, R_Naive, R_Coarse, R_Far : Seek.Report;
      Bound : Natural := 0;
   begin
      --  垂直于轴的那一对轴(和 Seek 里同一个取法:辅助方向取 A 分量绝对值最小的那根坐标轴),好把真的孔口放在两种格子的格点之间
      declare
         Aux : V3 := [others => 0.0];
         Low : Natural := 0;
      begin
         for C in 1 .. 2 loop
            if abs A (C) < abs A (Low) then
               Low := C;
            end if;
         end loop;
         Aux (Low) := 1.0;
         E1 := Unit (Contact.Cross (A, Aux));
         E2 := Contact.Cross (A, E1);
      end;
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            Cov (I, J) := Sd_L ** 2 * (E1 (I) * E1 (J) + E2 (I) * E2 (J)) + Sd_A ** 2 * A (I) * A (J);
         end loop;
      end loop;
      FR.Reset (Gen, 181);
      Place (0.5, 0.5, 0.5);
      Seek.Run (Start, Est, Cov, A, Depth, Depth_Sd, Gap, Res, Sim_Move'Access, R_Ok);
      Put_Line ("    插进去:" & Seek.Say (R_Ok));
      --  比找到的那一格更可能的格点有几个(同一套格子、同一个排法)
      declare
         H : constant Long_Float := R_Ok.Spacing;
         M_Hit : constant Long_Float := R_Ok.Mahal_Max;
         N1 : constant Integer := Integer (Long_Float'Floor (4.0 * Sd_L / H));
      begin
         for I in -N1 .. N1 loop
            for J in -N1 .. N1 loop
               if Sqrt ((H * Long_Float (I)) ** 2 + (H * Long_Float (J)) ** 2) / Sd_L <= M_Hit then
                  Bound := Bound + 1;
               end if;
            end loop;
         end loop;
      end;
      Place (0.5, 0.5, 0.5);
      Seek.Run (Start, Est, Zero, A, Depth, Depth_Sd, Gap, Res, Sim_Move'Access, R_Naive);
      Place (0.5, 0.5, 0.5);
      Seek.Run (Start, Est, Cov, A, Depth, Depth_Sd, Sd_L / Sqrt (2.0), Res, Sim_Move'Access, R_Coarse);
      Place (4.0, 0.0, 0.5);
      Seek.Run (Start, Est, Cov, A, Depth, Depth_Sd, Gap, Res, Sim_Move'Access, R_Far);
      Put_Line ("    孔在 4 倍不准外:" & Seek.Say (R_Far));
      Check (R_Ok.How = Seek.Reached and then R_Ok.Tries <= Bound and then R_Ok.Went_In >= Depth
             and then R_Naive.How /= Seek.Reached and then R_Coarse.How = Seek.Not_Found
             and then R_Far.How = Seek.Not_Found and then R_Far.Tries = R_Far.Candidates,
             "对准插进去:" & R_Ok.How'Image & " · 送了 " & Codec.Img (R_Ok.Tries) & " 次(更可能的格点 " & Codec.Img (Bound) & " 个,一片 "
             & Codec.Img (R_Ok.Candidates) & " 处,格子 " & Codec.Fmt (R_Ok.Spacing, 5) & ")· 送进去 " & Codec.Fmt (R_Ok.Went_In, 4)
             & " · 牙:不准当 0 ⇒ " & R_Naive.How'Image & ";格子按一倍不准 ⇒ " & R_Coarse.How'Image & "(试了 " & Codec.Img (R_Coarse.Tries)
             & " 处);孔在 4 倍外 ⇒ " & R_Far.How'Image & "(" & Codec.Img (R_Far.Tries) & " / " & Codec.Img (R_Far.Candidates) & ")");
   end;

   --  🔴 一次走不完的(Strokes.Run,§2 第 22 条):手腕量到的范围 −170° … +170°,最小一步 1°(读数的噪声 0.2°)。
   --  ① 从 −100° 起拧一颗要转 900° 才拧紧的螺丝,脑说"到转不动为止";② 从 −100° 起转一把钥匙 450°(没有转不动的地方);
   --  ③ 从 +100° 起往回转 500°(反着走);④ 倒手拉一根长东西 0.8(手够得着的那一截 0.1 … 0.4,最小一步 0.002 —— 同一段代码,不认单位);
   --  ⑤ 同 ④ 但再握没握上。
   --  要:① 转不动了、螺丝真转了 900°(差不过两个最小一步)、松开再握 2 回;② ③ ④ 走够了,真走的和要的差不过两个最小一步;
   --  ⑤ 照实说再握没握上,走了第一下那么多。
   --  错了会是什么病:手腕转到头就停 ⇒ 螺丝只拧了 270°、说不出"拧紧了";到头被挡住当成"拧紧了" ⇒ 同上;转回去只回一点 ⇒ 来回折腾很多回。
   --  牙:不松开重握(一下走到范围的头)⇒ 螺丝只转了 270°
   declare
      Res : Long_Float := Deg;
      Noise : Long_Float := 0.2 * Deg;
      Wrist : Long_Float := 0.0;              --  手腕(或手)此刻真的在哪
      Turned : Long_Float := 0.0;             --  那件东西真被转了(挪了)多少
      Limit : Long_Float := Long_Float'Last;  --  转到这么多就转不动了(Long_Float'Last = 没有)
      Lo : Long_Float := -170.0 * Deg;
      Hi : Long_Float := 170.0 * Deg;
      Grip_Ok : Boolean := True;
      procedure Sim_Stroke (D : Long_Float; R : out Strokes.Step_Report) is
         Can : constant Long_Float := (if D >= 0.0 then Long_Float'Min (D, Limit - Turned) else Long_Float'Max (D, -Limit - Turned));
         Got : constant Long_Float := Long_Float'Max (Lo - Wrist, Long_Float'Min (Hi - Wrist, Can));
      begin
         Wrist := Wrist + Got;
         Turned := Turned + Got;
         R := (Ok => True, At_Now => Wrist + Noise * Gauss, Blocked => abs (D - Got) > Res);
      end Sim_Stroke;
      procedure Sim_Free (D : Long_Float; R : out Strokes.Step_Report) is
         Got : constant Long_Float := Long_Float'Max (Lo - Wrist, Long_Float'Min (Hi - Wrist, D));
      begin
         Wrist := Wrist + Got;
         R := (Ok => True, At_Now => Wrist + Noise * Gauss, Blocked => abs (D - Got) > Res);
      end Sim_Free;
      procedure Sim_Release (Ok : out Boolean) is
      begin
         Ok := True;
      end Sim_Release;
      procedure Sim_Regrasp (Ok : out Boolean) is
      begin
         Ok := Grip_Ok;
      end Sim_Regrasp;
      procedure Go (Want, Start, Lim : Long_Float; Grip : Boolean; Rep : out Strokes.Report; True_Done : out Long_Float) is
      begin
         Wrist := Start;
         Turned := 0.0;
         Limit := Lim;
         Grip_Ok := Grip;
         Strokes.Run (Want, Lo, Hi, Wrist + Noise * Gauss, Res, Natural'Last, Sim_Stroke'Access, Sim_Release'Access, Sim_Free'Access,
                     Sim_Regrasp'Access, Rep);
         True_Done := Turned;
      end Go;
      R1, R2, R3, R4, R5 : Strokes.Report;
      D1, D2, D3, D4, D5, Naive : Long_Float;
      Res_Rot : constant Long_Float := Deg;
   begin
      FR.Reset (Gen, 191);
      Go (Long_Float'Last, -100.0 * Deg, 900.0 * Deg, True, R1, D1);
      Put_Line ("    拧到转不动:" & Strokes.Say (R1) & " · 螺丝真转了 " & Codec.Fmt (D1 / Deg, 1) & "°");
      --  牙:一下走到范围的头,不松开重握
      Wrist := -100.0 * Deg;
      Turned := 0.0;
      Limit := 900.0 * Deg;
      declare
         R : Strokes.Step_Report;
      begin
         Sim_Stroke (Hi - Wrist, R);
      end;
      Naive := Turned;
      Go (450.0 * Deg, -100.0 * Deg, Long_Float'Last, True, R2, D2);
      Go (-500.0 * Deg, 100.0 * Deg, Long_Float'Last, True, R3, D3);
      Put_Line ("    转 450° / 往回 500°:" & Strokes.Say (R2) & " / " & Strokes.Say (R3));
      Res := 0.002;
      Noise := 0.0004;
      Lo := 0.1;
      Hi := 0.4;
      Go (0.8, 0.1, Long_Float'Last, True, R4, D4);
      Go (0.8, 0.1, Long_Float'Last, False, R5, D5);
      Put_Line ("    倒手拉 0.8:" & Strokes.Say (R4) & " / 再握不上:" & Strokes.Say (R5));
      Check (R1.How = Strokes.Tight and then abs (D1 - 900.0 * Deg) <= 2.0 * Res_Rot and then R1.Regrasps = 2
             and then R2.How = Strokes.Reached and then abs (D2 - 450.0 * Deg) <= 2.0 * Res_Rot
             and then R3.How = Strokes.Reached and then abs (D3 + 500.0 * Deg) <= 2.0 * Res_Rot
             and then R4.How = Strokes.Reached and then abs (D4 - 0.8) <= 2.0 * Res and then R4.Regrasps = 2
             and then R5.How = Strokes.Regrasp_Failed and then abs (D5 - 0.3) <= 2.0 * Res
             and then Naive < 900.0 * Deg - 2.0 * Res_Rot,
             "一次走不完的:拧到转不动 " & R1.How'Image & " 真转了 " & Codec.Fmt (D1 / Deg, 1) & "°、再握 " & Codec.Img (R1.Regrasps) & " 回 · 转 450° ⇒ "
             & Codec.Fmt (D2 / Deg, 1) & "° · 往回 500° ⇒ " & Codec.Fmt (D3 / Deg, 1) & "° · 倒手拉 0.8 ⇒ " & Codec.Fmt (D4, 4) & "、再握 "
             & Codec.Img (R4.Regrasps) & " 回 · 再握不上 ⇒ " & R5.How'Image & " 走了 " & Codec.Fmt (D5, 4)
             & " · 牙:不松开重握 ⇒ 螺丝只转了 " & Codec.Fmt (Naive / Deg, 1) & "°");
   end;

   --  🔴 接到执行上的那一层(Linkage.Follow_Held,Change_Held_Qty 那一下"手里的东西沿要的方向变一个单位"换成它):同一扇门(把手离轴 0.8,
   --  握得住顶着的 0.9 步),脑要把手沿"和门开的方向差 50°"挪 0.15(门开到二十几度才挪得够)。调用方只给"走一下":手实到多少、
   --  东西挪了多少(眼没另外跟着 ⇒ 就是手实到的)、挡没挡(Selfmap.Blocked);一步多长按这只手看得见的一档 0.03、读数抖 0.002 定。
   --  要:到了、沿要的方向挪够 0.15、把手没滑出去。错了会是什么病:今天的 Change_Held_Qty 直着走一个单位(顶住了就沿着顶住的那一面再走一步)
   --  ⇒ 门越开推得越顶着门,把手滑出去。牙:一直直着沿要的方向走 ⇒ 没挪够就滑出去了
   declare
      Rho : constant Long_Float := 0.8;
      Phi : Long_Float := 0.0;
      Phi_Max : constant Long_Float := 100.0 * Deg;
      Slipped : Boolean := False;
      Noise : constant Long_Float := 0.002;
      Floor : constant Long_Float := 0.03;
      Unit_L : constant Long_Float := 0.15;
      Want : constant V3 := [Sin (50.0 * Deg), Cos (50.0 * Deg), 0.0];
      Wu : constant V3 := Unit (Want);
      V2 : constant Long_Float := 2.0 * Noise ** 2;
      Grip : constant Long_Float := 0.9 * Linkage.Light_Len (Floor, [[V2, 0.0, 0.0], [0.0, V2, 0.0], [0.0, 0.0, V2]]);
      Walked : Long_Float := 0.0;   --  把手真沿 Want 挪了多少
      function Pos (A : Long_Float) return V3 is ([Rho * Cos (A), Rho * Sin (A), 0.0]);
      procedure Door_Move (D : V3; Got_Hand, Got_Thing : out V3; Blocked, Ok : out Boolean) is
         N : constant V3 := [Cos (Phi), Sin (Phi), 0.0];
         T : constant V3 := [-Sin (Phi), Cos (Phi), 0.0];
         Off : constant Long_Float := Sqrt (Dot (D, N) ** 2 + D (2) ** 2);   --  顶着门的那一截
         Real : V3 := [others => 0.0];
         L : constant Long_Float := Geom.Norm (D);
      begin
         Ok := True;
         if Slipped or else Off > Grip then
            Slipped := True;
            Got_Hand := Add (D, [Noise * Gauss, Noise * Gauss, Noise * Gauss]);
            Got_Thing := Got_Hand;   --  眼没另外跟着把手:只知道手到了哪
            Blocked := False;
            return;
         end if;
         declare
            New_Phi : constant Long_Float := Long_Float'Max (0.0, Long_Float'Min (Phi_Max, Phi + Dot (D, T) / Rho));
         begin
            Real := Sub (Pos (New_Phi), Pos (Phi));
            Phi := New_Phi;
         end;
         Walked := Walked + Dot (Real, Wu);
         Got_Hand := Add (Real, [Noise * Gauss, Noise * Gauss, Noise * Gauss]);
         Got_Thing := Got_Hand;
         Blocked := L > 0.0 and then Selfmap.Blocked (L - Dot (Real, Scl (D, 1.0 / L)), 0.0, 0.0, Selfmap.Free_Base, L, Noise);
      end Door_Move;
      Fw : Linkage.Follow_Report;
      Walked_F, Phi_F : Long_Float;
      Slip_F : Boolean;
      Naive_Walked : Long_Float;
      Naive_Slip : Boolean;
   begin
      FR.Reset (Gen, 201);
      Linkage.Follow_Held (Want, Unit_L, Floor, Noise, 400, Door_Move'Access, Fw);
      Walked_F := Walked;
      Phi_F := Phi;
      Slip_F := Slipped;
      --  牙:直着沿 Want 一下一下走(同 Light_Len 那一步),走够 Unit 或者滑出去为止
      Phi := 0.0;
      Slipped := False;
      Walked := 0.0;
      for I in 1 .. 400 loop
         declare
            Gh, Gt : V3;
            Bl, Ok : Boolean;
         begin
            Door_Move (Scl (Wu, Grip / 0.9), Gh, Gt, Bl, Ok);
         end;
         exit when Slipped or else Walked >= Unit_L;
      end loop;
      Naive_Walked := Walked;
      Naive_Slip := Slipped;
      Check (Fw.How = Linkage.Arrived and then Walked_F >= Unit_L - Linkage.Light_Len (Floor, [[V2, 0.0, 0.0], [0.0, V2, 0.0], [0.0, 0.0, V2]])
             and then not Slip_F and then Naive_Slip and then Naive_Walked < Unit_L,
             "接到执行上·手里的东西沿要的方向变一个单位:" & Fw.How'Image & " · " & Codec.Img (Fw.Steps) & " 步 · 沿 Want 真挪了 " & Codec.Fmt (Walked_F, 4)
             & "(要 " & Codec.Fmt (Unit_L, 2) & ")· 门开到 " & Codec.Fmt (Phi_F / Deg, 1) & "° · 滑出去 " & Slip_F'Image
             & " · 牙:直着走 ⇒ 挪了 " & Codec.Fmt (Naive_Walked, 4) & " 就滑出去 " & Naive_Slip'Image);
   end;

   --  🔴 两块之间的轴 ⇒ 接触集的第③格(Linkage.Motion_Of,§2 第 19 条"同时两套接触"里动的那一块):一件东西被挪过(A 此刻的位姿转了 30°、平移了),
   --  它上面一块绕 A 上的一根轴转(或沿 A 上的一根滑轴走)。要:按此刻的位姿把轴搬到世界里,B 上一点按这个旋量搬过去,和真的搬过去的一致
   --  (差在数值分辨率以内);轴没定下来(Undecided)⇒ 不给旋量(Ok = False)。
   --  错了会是什么病:轴照参照帧那一刻的世界系用 ⇒ 东西一被挪过,"扣扳机"那一段绕着空中一根不在枪上的轴转。
   --  牙:不按 A 此刻的位姿搬轴 ⇒ 差出一大截
   declare
      Pa : constant Linkage.Pose := (Ok => True, R => Rot ([0.2, -0.4, 1.0], 30.0 * Deg), T => [0.3, -0.1, 0.05]);
      Jt : Linkage.Joint;
      Js : Linkage.Joint;
      Ju : Linkage.Joint;
      X_Ref : constant V3 := [0.12, 0.05, -0.02];   --  B 上一点,参照帧那一刻(A 的系 = 参照帧的世界)
      Q : constant Long_Float := 25.0 * Deg;
      Ok_T, Ok_S, Ok_U : Boolean;
      Tw_T, Tw_S, Tw_U : Contact.Twist;
      Err_T, Err_S, Naive_Err : Long_Float;
   begin
      Jt.Status := Linkage.Found;
      Jt.Ax := (W => Unit ([0.3, 1.0, 0.1]), P => [0.05, 0.0, 0.01], Slide => False);
      Js := Jt;
      Js.Ax := (W => Unit ([1.0, 0.2, -0.3]), P => [0.0, 0.0, 0.0], Slide => True);
      Ju := Jt;
      Ju.Status := Linkage.Undecided;
      Tw_T := Linkage.Motion_Of (Jt, Pa, Q, Ok_T);
      Tw_S := Linkage.Motion_Of (Js, Pa, 0.04, Ok_S);
      Tw_U := Linkage.Motion_Of (Ju, Pa, Q, Ok_U);
      declare
         X_Now : constant V3 := Add (Geom.Ap (Pa.R, X_Ref), Pa.T);
         --  真的:在 A 的系里绕轴转 Q,再按 A 此刻的位姿搬到世界
         Rq : constant M3 := Rot (Jt.Ax.W, Q);
         True_T : constant V3 := Add (Geom.Ap (Pa.R, Add (Geom.Ap (Rq, Sub (X_Ref, Jt.Ax.P)), Jt.Ax.P)), Pa.T);
         True_S : constant V3 := Add (Geom.Ap (Pa.R, Add (X_Ref, Scl (Js.Ax.W, 0.04))), Pa.T);
         Naive : Contact.Twist;
         Okn : Boolean;
      begin
         Err_T := Geom.Norm (Sub (Contact.Apply (Tw_T, X_Now), True_T));
         Err_S := Geom.Norm (Sub (Contact.Apply (Tw_S, X_Now), True_S));
         Naive := Contact.Rotation (Jt.Ax.W, Q, Jt.Ax.P, Okn);
         Naive_Err := Geom.Norm (Sub (Contact.Apply (Naive, X_Now), True_T));
      end;
      Check (Ok_T and then Ok_S and then not Ok_U and then Err_T < Sqrt (Long_Float'Model_Epsilon) and then Err_S < Sqrt (Long_Float'Model_Epsilon)
             and then Naive_Err > 0.01,
             "两块之间的轴 ⇒ 接触集的旋量:转轴搬过去差 " & Codec.Fmt (Err_T, 12) & " · 滑轴 " & Codec.Fmt (Err_S, 12) & " · 定不下 ⇒ 不给 "
             & Boolean'Image (not Ok_U) & " · 牙:轴不按 A 此刻的位姿搬 ⇒ 差 " & Codec.Fmt (Naive_Err, 4));
   end;
end Welds_Path_6;
