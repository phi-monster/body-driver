with Linkage;
with Stats;
separate (Selfcheck)
procedure Welds_Path_6 is
   --  路 6 的焊点(大并行.md §5 路 6):每条写清"错了会是什么病",带一颗牙(去掉那一改就红)
   use Ada.Numerics.Long_Elementary_Functions;
   use type Linkage.Role;
   use type Linkage.Axis_Status;
   use type Linkage.Follow_End;
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
end Welds_Path_6;
