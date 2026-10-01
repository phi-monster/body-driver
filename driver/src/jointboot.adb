with Ada.Text_IO;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Containers;
with Ada.Calendar;
with Codec;
with Picture;
with Table;
with Instrument;
with Layout;
with Ada.Directories;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Interfaces;
with Stats;
package body Jointboot is

   procedure Say (S : String) is
   begin
      Ada.Text_IO.Put_Line ("[身] 📐 " & S);
   end Say;

   --  不动的眼这个像素去不去得了畸变(落在镜头模型够不到的地方 ⇒ Geom 交零向量):去不了的那一笔不进对齐(09-30,E 组)
   function Ray_Usable (G : Geom.Cam_Geo; U, V : Long_Float) return Boolean is
      Ok : Boolean;
      D : constant Geom.V3 := Geom.Ray_Fixed (G, U, V, Ok);
      pragma Unreferenced (D);
   begin
      return Ok;
   end Ray_Usable;

   --  落盘时一根轴是转还是走(格式里的一个词)
   function Kind_Word (A : Kinem.Axis) return String is (if A.Slide then "slide" else "turn");

   function Multi_Up (Cell, Joint : Natural) return Boolean is
      use type Interfaces.Unsigned_32;
      X : Interfaces.Unsigned_32 := Interfaces.Unsigned_32 (Cell) and Interfaces.Unsigned_32 (Joint + 1);
      Odd : Boolean := False;
   begin
      while X /= 0 loop
         Odd := not Odd;
         X := X and (X - 1);   --  去掉最低的那一个 1:几轮去完 = 同为 1 的有几位
      end loop;
      return not Odd;
   end Multi_Up;

   Start_Amp : constant Long_Float := 1.0e-4;   --  探针协议的起点(同 Selfmap:极小,翻倍到走得出来又看得见为止;无量纲协议)
   Max_Doublings : constant := 12;              --  次数
   Grow : constant := 2.0;                      --  探针每次翻一倍(次数:同 Selfmap 的探针协议)
   Third : constant := 1.0 / 3.0;               --  三分之一(比例:"没转到命令的三分之一 = 被顶住",到没到目标用同一个比例)
   Ramp : constant := 4.0;                      --  扫描下一格最多放大四倍(次数;每个方向只停 3 格,头一格之后两格就要转开)

   --  一组关节读数一起挪到 Q(关节目标走唯一那条挪手的路 Selfmap.Go,停稳看这组读数)
   procedure Go_Group (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; A, G : Natural; Q : Floats; Tol : Long_Float; Ok : out Boolean) is
      Dl : Table.Vec;
      Fr : Natural;
   begin
      Selfmap.Go (L, M, A, [others => 0.0], F64_Vectors.Empty_Vector, F, Dl, Fr, Ok, Joints => Q, Group => G, Tol => Tol);
   end Go_Group;

   procedure Find_Arms (L : in out Plug.Link; F : in out Plug.Frame; M : in out Selfmap.Body_Map;
                        Arms : out Arm_Vectors.Vector; World_Cam : out Integer; Ok : out Boolean) is separate;

   Gx : constant := Kinem.Gx;   --  问格点的那张格子(全仓一份,见 Kinem.Grid_U / Grid_V)
   Gy : constant := Kinem.Gy;

   procedure Sweep_All (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; Arms : Arm_Vectors.Vector;
                        Host : String; Port : Natural; Dump : String; Ds : out Sweep_Vectors.Vector; Css : out Corr_Set_Vectors.Vector;
                        World_Cam : Integer := -1) is separate;

   --  相机系里的单位视线(同 Geom 的约定:-z 朝前、+y 朝上)



   procedure Fit_Arm (A : Natural; D : Sweep_Data; Cs : Kinem.Corr_Vectors.Vector; Dump : String; M : out Kinem.Model; Ok : out Boolean;
                      Note : out Unbounded_String) is separate;

   --  这只手扫描里三角出来的点(参照眼系,模型单位):起点那一格上的格点配进很多格(轨迹),按运动学在每一格里的像素一起解它的远近(多视图三角,
   --  Kinem.Track_Points;09-27 V1B32 改:原来按"起点 ↔ 某一格"一对三角,每一对自己的位姿误差让整片点成块偏 1–3.6 mm,不动的眼按它们定位偏 4 mm)。
   --  只收:至少 3 格看见(起点 + 另外两格)、重投影残差中位 < 3 px(协议)、配点差 1 像素时远近误差不到眼到它距离的二十分之一(比例,同原来"两条视线夹角 ≥ 20 / 焦距")。
   --  协方差:垂直视线 r·σ/f,沿视线 = 那一维解的方差(σ = Track_Points 量的:重投影残差垂直于对极线那一分量的 Mad_Sigma × 中位)
   type Tri_Pt is record
      X : Geom.V3 := [0.0, 0.0, 0.0];
      Cov : Geom.M3 := [others => [others => 0.0]];
      Fr : Natural := 0;                    --  看见它的格子里离起点那格的眼最远的一格(对齐拿它当"世界里的一只眼")
      U0, V0 : Long_Float := 0.0;           --  在起点那格里的像素
   end record;
   package Tri_Vectors is new Ada.Containers.Vectors (Natural, Tri_Pt);
   procedure Tri_Pts (M : Kinem.Model; D : Sweep_Data; Cs : Kinem.Corr_Vectors.Vector; Max_Pts : Natural; P : out Tri_Vectors.Vector; Sig_Px : out Long_Float) is
      use Geom;
      Tp : Kinem.Track_Pt_Vectors.Vector;
   begin
      P.Clear; Sig_Px := 0.0;
      Kinem.Track_Points (M, D.Frames, Cs, Only_I => 0, Min_Views => 3, Tracks => Tp, Sig_Px => Sig_Px);   --  起点那一格出发、至少 3 格(次数)
      if Sig_Px <= 0.0 then
         return;
      end if;
      for T of Tp loop
         exit when Natural (P.Length) >= Max_Pts;
         declare
            X : constant V3 := T.X;
            Rr : constant Long_Float := Norm (X);
            Sd : constant Long_Float := Sqrt (T.Var_Along);   --  沿视线(模型单位)
         begin
            if T.Med_Px < 3.0 and then Rr > 0.0 and then Sd / Sig_Px < 0.05 * Rr and then X (2) < 0.0 then   --  3 px(协议);二十分之一(比例,见上);在参照眼前面(-z 朝前)
               declare
                  Dd : constant V3 := [X (0) / Rr, X (1) / Rr, X (2) / Rr];
                  Sp : constant Long_Float := Rr * Sig_Px / M.F;   --  垂直视线
                  Cv : M3;
               begin
                  for I in 0 .. 2 loop
                     for J in 0 .. 2 loop
                        Cv (I, J) := Sp * Sp * ((if I = J then 1.0 else 0.0) - Dd (I) * Dd (J)) + Sd * Sd * Dd (I) * Dd (J);
                     end loop;
                  end loop;
                  P.Append (Tri_Pt'(X => X, Cov => Cv, Fr => T.Far, U0 => T.U, V0 => T.V));
               end;
            end if;
         end;
      end loop;
   end Tri_Pts;

   procedure Until_Settled (Rounds : out Natural; Verdict : out Settle_Verdict) is
      package Set_Vectors is new Ada.Containers.Vectors (Natural, Bools, Bool_Vectors."=");
      Seen : Set_Vectors.Vector;   --  解过的每一组(按先后)
      U : Bools;
      Enough : Boolean;
   begin
      Rounds := 0;
      loop
         Pick (U, Enough);
         if not Enough then
            Verdict := Too_Few;
            return;
         elsif not Seen.Is_Empty and then Bool_Vectors."=" (U, Seen.Last_Element) then
            Verdict := Settled;
            return;
         elsif Seen.Contains (U) then
            Verdict := Cycled;
            return;
         elsif Rounds >= Natural (U.Length) + 1 then
            Verdict := Capped;
            return;
         end if;
         Seen.Append (U);
         Solve (U);
         Rounds := Rounds + 1;
      end loop;
   end Until_Settled;

   --  ── 把几只手、几只不长在手上的眼放进同一个世界(一种办法,所有身体一样;代码里不问"有没有头顶眼")──
   --  世界 = 第一只手的参照眼系;它扫描的每一格(有三角点的)都是世界里的一只眼,位姿按它的运动学。
   --  还没放进世界的:每一只不长在手上的眼(解它的位姿 + 焦距)、每一只别的手(解它整个系的位置、朝向、长度倍数)。
   --  每一轮,每个没放进的都和世界里每一只还没配过的眼配一次(用它最像那只眼的一格,按整体特征 Instrument.Describe 挑),配点 + 往返 1 px 核对 ⇒ 共同看见的点:
   --    · 一只眼:世界点(世界里的手三角出的点)在它里面的像素 ⇒ 按板解它(Geom.Fit_Fixed_Board);
   --    · 一只手:它自己三角出、落在它自己桌面上的点,在世界里那只眼里的像素 ⇒ 那只眼的视线交世界的桌面 ⇒ 同一个点在两个系里 ⇒ 相似变换(抗野点)。
   --  这一轮内点最多的那个放进世界(它的格子 / 它这只眼随即也是世界里的眼),再下一轮;谁都放不进 ⇒ 停,放不进的如实说。
   --  全部放完以后所有放进世界的一起精修一遍(Joint_Refine):先放进去的也被后面的证据修正。
   --  (V1B11 2026-09-26 量:两只腕眼起点那格一点不重叠,可按整体特征挑的两手之间最像的 12 对往返配上 34–63%、随机 12 对平均 7%;
   --  头顶眼和第一只手最像的 8 格里 6 格配得上 29–36%)
   Trip_Px : constant := Geom.Trip_Px;   --  往返 1 px 内的才算(绝对门 —— 按中位数倍数定的门在乱配占多数时跟着放宽,V1B11)
   Min_Inl : constant := 10;    --  至少 10 个内点才放进世界(次数)

   --  ── 一条配点的残差和它的噪声分量(10-01;原来 Align 里的 Proj_W 按这个算,K = 1 时一模一样)──
   --  点 X(它那只手自己系、协方差 Cov)按放法 (S, R, T) 放进世界,投进世界里那只眼 Cg:像素残差;三角的不确定度拆两份 ——
   --  远近那一维(沿 X 自己的方向:点都是从它那只手的参照眼三角出来的)投进画面是沿对极线的一条(A 方向、方差 Vd),横向那两维投进来是 P11 / P12 / P22
   type Ob_Part is record
      Valid : Boolean := False;              --  点在那只眼前面、那只眼有焦距
      E0, E1 : Long_Float := 0.0;            --  像素残差(投影 − 配上的)
      A0, A1 : Long_Float := 0.0;            --  沿对极线的单位方向(这个点沿它自己的视线挪,在这只眼里往哪边走)
      P11, P12, P22 : Long_Float := 0.0;     --  横向那份投进画面(像素²)
      Vd : Long_Float := 0.0;                --  远近那份(它自报的)投进来的方差(像素²,沿 A)
   end record;
   Behind_W : constant := 1.0e3;             --  点在那只眼后面:白化残差记成远大于任何一条真残差的罚(无量纲哨兵,同原来 Proj_W)
   function Part_Of (Cg : Geom.Cam_Geo; X : Geom.V3; Cov : Geom.M3; S : Long_Float; R : Geom.M3; T : Geom.V3; U, V : Long_Float) return Ob_Part is
      use Geom;
      P : Ob_Part;
      Rx : constant V3 := Ap (R, X);
      Xw : constant V3 := [S * Rx (0) + T (0), S * Rx (1) + T (1), S * Rx (2) + T (2)];
      Rt : constant M3 := Tr (Cg.R_Ce);
      Pc : constant V3 := Ap (Rt, [Xw (0) - Cg.Pos (0), Xw (1) - Cg.Pos (1), Xw (2) - Cg.Pos (2)]);
      Z : constant Long_Float := -Pc (2);   --  -z 朝前(同 Geom 的约定)
      Nx : constant Long_Float := Norm (X);
   begin
      if Z <= 0.0 or else Cg.F <= 0.0 then
         return P;
      end if;
      declare
         Du : constant V3 := [Cg.F / Z, 0.0, Cg.F * Pc (0) / (Z * Z)];
         Dv : constant V3 := [0.0, -Cg.F / Z, -Cg.F * Pc (1) / (Z * Z)];
         Ju, Jv : V3 := [0.0, 0.0, 0.0];     --  像素对世界点的导数(两行)
         Dd : constant V3 := (if Nx > 0.0 then [X (0) / Nx, X (1) / Nx, X (2) / Nx] else [0.0, 0.0, 0.0]);
         Dw : constant V3 := Ap (R, Dd);
         Sd2 : Long_Float := 0.0;
         Cl, Cw : M3;
         G0, G1, Ng : Long_Float := 0.0;
         function Q (A : V3; C : M3; B : V3) return Long_Float is
            Sm : Long_Float := 0.0;
         begin
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Sm := Sm + A (I) * C (I, J) * B (J);
               end loop;
            end loop;
            return Sm;
         end Q;
      begin
         for K in 0 .. 2 loop
            Ju (K) := Du (0) * Rt (0, K) + Du (1) * Rt (1, K) + Du (2) * Rt (2, K);
            Jv (K) := Dv (0) * Rt (0, K) + Dv (1) * Rt (1, K) + Dv (2) * Rt (2, K);
         end loop;
         Sd2 := Q (Dd, Cov, Dd);
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Cl (I, J) := Cov (I, J) - Sd2 * Dd (I) * Dd (J);
            end loop;
         end loop;
         declare
            Rc : constant M3 := Mul (Mul (R, Cl), Tr (R));
         begin
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Cw (I, J) := S * S * Rc (I, J);
               end loop;
            end loop;
         end;
         P.P11 := Q (Ju, Cw, Ju); P.P12 := Q (Ju, Cw, Jv); P.P22 := Q (Jv, Cw, Jv);
         G0 := S * (Ju (0) * Dw (0) + Ju (1) * Dw (1) + Ju (2) * Dw (2));
         G1 := S * (Jv (0) * Dw (0) + Jv (1) * Dw (1) + Jv (2) * Dw (2));
         Ng := Sqrt (G0 * G0 + G1 * G1);
         if Ng > 0.0 then
            P.A0 := G0 / Ng; P.A1 := G1 / Ng;
         else
            P.A0 := 1.0; P.A1 := 0.0;   --  这一眼里远近挪它不动:方向随便取一个,Vd = 0(单位向量,纯数学)
         end if;
         P.Vd := Sd2 * Ng * Ng;
         P.E0 := Cg.F * Pc (0) / Z + Cg.Cx - U;
         P.E1 := -Cg.F * Pc (1) / Z + Cg.Cy - V;
         P.Valid := True;
      end;
      return P;
   end Part_Of;
   --  白化:这条配点的协方差 = Sm² I(配点噪声)+ 横向那份 + K² × 远近那份(K = 远近真的不准是自报的几倍),按它的 Cholesky 把像素里的一个向量 (V0, V1)
   --  换成以标准差为单位的两条(W1, W2);Area = √det(这条配点每一白化单位面积占多少像素面积,混合似然用)
   procedure Whiten_Vec (P : Ob_Part; Sm, K, V0, V1 : Long_Float; W1, W2, Area : out Long_Float) is
      A11 : constant Long_Float := Sm * Sm + P.P11 + K * K * P.Vd * P.A0 * P.A0;
      A12 : constant Long_Float := P.P12 + K * K * P.Vd * P.A0 * P.A1;
      A22 : constant Long_Float := Sm * Sm + P.P22 + K * K * P.Vd * P.A1 * P.A1;
      L11 : constant Long_Float := Sqrt (Long_Float'Max (A11, 1.0e-18));   --  数值保护(无量纲,同原来 Proj_W)
      L21 : constant Long_Float := A12 / L11;
      L22 : constant Long_Float := Sqrt (Long_Float'Max (A22 - L21 * L21, 1.0e-18));   --  同上
   begin
      W1 := V0 / L11;
      W2 := (V1 - L21 * W1) / L22;
      Area := L11 * L22;
   end Whiten_Vec;
   --  这条配点的两条白化残差(点在那只眼后面 ⇒ 两条都记成 Behind_W)
   procedure Whiten (P : Ob_Part; Sm, K : Long_Float; E1, E2, Area : out Long_Float) is
   begin
      if not P.Valid then
         E1 := Behind_W; E2 := Behind_W; Area := 1.0;
         return;
      end if;
      Whiten_Vec (P, Sm, K, P.E0, P.E1, E1, E2, Area);
   end Whiten;
   --  放法的 7 个数:转动向量(弧度)、平移(世界单位)、长度倍数的对数(同原来 Place_Arm / Joint_Refine 的参数化)
   procedure Unpack7 (X : Kinem.Vec; Sx : out Long_Float; Rx : out Geom.M3; Tx : out Geom.V3) is
   begin
      Rx := Geom.Rodrigues ([X (X'First), X (X'First + 1), X (X'First + 2)]);
      Tx := [X (X'First + 3), X (X'First + 4), X (X'First + 5)];
      Sx := Exp (X (X'First + 6));
   end Unpack7;
   function Pack7 (Sx : Long_Float; Rx : Geom.M3; Tx : Geom.V3) return Kinem.Vec is
      Rv : constant Geom.V3 := Geom.Rot_Vec (Rx);
   begin
      return [Rv (0), Rv (1), Rv (2), Tx (0), Tx (1), Tx (2), Log (Sx)];
   end Pack7;
   function Cross3 (A, B : Geom.V3) return Geom.V3 is ([A (1) * B (2) - A (2) * B (1), A (2) * B (0) - A (0) * B (2), A (0) * B (1) - A (1) * B (0)]);
   --  它的桌面对上世界的桌面:两个倾角(除以倾角的不确定度)、一个高度(除以高度的不确定度)(同原来 Align 的 Plane_Res)
   procedure Tie_Res (Tie : Plane_Tie; Sx : Long_Float; Rx : Geom.M3; Tx : Geom.V3; R1, R2, R3 : out Long_Float) is
      use Geom;
      N0 : constant V3 := Tie.N0;
      D0w : constant Long_Float := Tie.P0 (0) * N0 (0) + Tie.P0 (1) * N0 (1) + Tie.P0 (2) * N0 (2);
      Nw : constant V3 := Ap (Rx, Tie.Nb);
      Tilt : constant V3 := Cross3 (Nw, N0);
      E1, E2 : V3;
      Rt : constant V3 := Ap (Rx, Tie.Cb);
      Sig_N : constant Long_Float := Long_Float'Max (1.0e-12, Tie.Sn);   --  数值保护(无量纲)
      Sig_P : constant Long_Float := Long_Float'Max (1.0e-12, Tie.Sd);   --  同上
   begin
      E1 := Cross3 (N0, (if abs N0 (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]));   --  垂直于世界法向的一对方向(纯数学,无量纲)
      E1 := [E1 (0) / Norm (E1), E1 (1) / Norm (E1), E1 (2) / Norm (E1)];
      E2 := Cross3 (N0, E1);
      R1 := (Tilt (0) * E1 (0) + Tilt (1) * E1 (1) + Tilt (2) * E1 (2)) / Sig_N;
      R2 := (Tilt (0) * E2 (0) + Tilt (1) * E2 (1) + Tilt (2) * E2 (2)) / Sig_N;
      R3 := (N0 (0) * (Sx * Rt (0) + Tx (0)) + N0 (1) * (Sx * Rt (1) + Tx (1)) + N0 (2) * (Sx * Rt (2) + Tx (2)) - D0w) / Sig_P;
   end Tie_Res;
   --  它干活的地方(它的桌面中心)放进世界:S · R · Cb + T
   function Work_Of (Tie : Plane_Tie; Sx : Long_Float; Rx : Geom.M3; Tx : Geom.V3) return Geom.V3 is
      Rc : constant Geom.V3 := Geom.Ap (Rx, Tie.Cb);
   begin
      return [Sx * Rc (0) + Tx (0), Sx * Rc (1) + Tx (1), Sx * Rc (2) + Tx (2)];
   end Work_Of;
   function Hand_Part (O : Hand_Ob; Sx : Long_Float; Rx : Geom.M3; Tx : Geom.V3) return Ob_Part is (Part_Of (O.Cam, O.X, O.Cov, Sx, Rx, Tx, O.U, O.V));
   --  沿 / 垂直对极线两个分量(Ea、Ep)和横向那份在这两个方向上的方差(Va、Vp);远近那份只沿 A(方差分量用)
   procedure Split (P : Ob_Part; Ea, Ep, Va, Vp : out Long_Float) is
   begin
      Ea := P.E0 * P.A0 + P.E1 * P.A1;
      Ep := -P.E0 * P.A1 + P.E1 * P.A0;
      Va := P.A0 * P.A0 * P.P11 + 2.0 * P.A0 * P.A1 * P.P12 + P.A1 * P.A1 * P.P22;
      Vp := P.A1 * P.A1 * P.P11 - 2.0 * P.A0 * P.A1 * P.P12 + P.A0 * P.A0 * P.P22;
   end Split;

   function Median_Of (X : Floats) return Long_Float is
      package Sorting is new F64_Vectors.Generic_Sorting;
      S : Floats := X;
   begin
      if S.Is_Empty then
         return 0.0;
      end if;
      Sorting.Sort (S);
      return S (Natural (S.Length) / 2);
   end Median_Of;

   --  ── 门里还是乱配:混合模型自己的分界(10-01)──
   --  每条配点要么是门里的(白化残差二维单位正态,密度 e^(−|w|²/2) / 2π),要么是乱配(均匀落在那只眼的画面上:每白化单位面积的密度
   --  = 这条配点的像素面积 √det ÷ 画幅面积 = Dens);门里的占比 γ 按 EM 解到不再变(EM 单调;保险 = 条数 + 1 遍)。
   --  W2 = 每条的 |w|²;Cost = 负对数似然 −Σ ln(γ e^(−|w|²/2) / 2π + (1 − γ) Dens)
   procedure Mix_Em (W2, Dens : Floats; Gamma, Cost : out Long_Float) is
      N : constant Natural := Natural (W2.Length);
      Pin : Floats;
   begin
      Gamma := 0.5;   --  EM 的起点(取一半;不影响收到哪儿)
      Cost := 0.0;
      for I in 0 .. N - 1 loop
         Pin.Append (Exp (-0.5 * W2 (I)) / (2.0 * Ada.Numerics.Pi));   --  二维单位正态的密度(纯数学)
      end loop;
      for It in 0 .. N loop
         declare
            Sum : Long_Float := 0.0;
            Gn : Long_Float;
         begin
            for I in 0 .. N - 1 loop
               declare
                  D : constant Long_Float := Gamma * Pin (I) + (1.0 - Gamma) * Dens (I);
               begin
                  if D > 0.0 then
                     Sum := Sum + Gamma * Pin (I) / D;
                  end if;
               end;
            end loop;
            Gn := Sum / Long_Float (Natural'Max (1, N));
            exit when Gn = Gamma;
            Gamma := Gn;
         end;
      end loop;
      for I in 0 .. N - 1 loop
         Cost := Cost - Log (Long_Float'Max (Gamma * Pin (I) + (1.0 - Gamma) * Dens (I), Long_Float'Model_Small));
      end loop;
   end Mix_Em;
   --  门里 = 这一条"像门里的"的似然比"像乱配的"大:|w|² < 2 ln(γ / ((1 − γ) · 2π · Dens))。
   --  (原来门 = max(Z, Z × 中位),假定所有配点同一个尺度;两只腕眼隔 0.6 m 看近处桌面,近点沿对极线的真误差比按"一只手一个远近放大"给它的大,
   --  P3B 在真值处就有 17 个里的好几个卡在 3.4σ、门 3.3σ ⇒ 砍掉,远点成片那一拉把第 2 只手拖偏 32 mm;乱配差几十上百 σ,照样挑掉)
   procedure Mix_Gate (W2, Dens : Floats; Inl : out Bools; Gamma : out Long_Float) is
      Cost : Long_Float;
   begin
      Mix_Em (W2, Dens, Gamma, Cost);
      Inl.Clear;
      for I in 0 .. Natural (W2.Length) - 1 loop
         if Gamma >= 1.0 then
            Inl.Append (True);
         elsif Gamma <= 0.0 or else Dens (I) <= 0.0 then
            Inl.Append (Gamma > 0.0 and then Dens (I) <= 0.0);
         else
            Inl.Append (W2 (I) < 2.0 * Log (Gamma / ((1.0 - Gamma) * 2.0 * Ada.Numerics.Pi * Dens (I))));   --  两个似然相等的分界(纯数学)
         end if;
      end loop;
   end Mix_Gate;
   --  乱配在白化单位里的密度:这条配点的像素面积 ÷ 那只眼的画幅面积(画幅没给 ⇒ 按 1 像素算,只防除零)
   function Dens_Of (Area : Long_Float; W, H : Natural) return Long_Float is
     (Area / Long_Float'Max (1.0, Long_Float (W) * Long_Float (H)));

   --  尺度方程 Mad_Sigma × 中位 |R_i| / √(S² + V_i) = 1 的解(左边随 S 单调降 ⇒ 二分,到区间不再缩为止);S = 0 时左边已经 ≤ 1 ⇒ 0
   function Scale_Root (R, V : Floats; Extra : Floats) return Long_Float is
      function F (S : Long_Float) return Long_Float is
         Q : Floats;
      begin
         for I in 0 .. Natural (R.Length) - 1 loop
            Q.Append (abs R (I) / Sqrt (Long_Float'Max (S * S * Extra (I) + V (I), Long_Float'Model_Small)));
         end loop;
         return Kinem.Mad_Sigma * Median_Of (Q);
      end F;
      Lo : Long_Float := 0.0;
      Hi : Long_Float := 0.0;
   begin
      if R.Is_Empty or else F (0.0) <= 1.0 then
         return 0.0;
      end if;
      --  上界:S 大到每一条 |R| / (S √Extra) 都不到 1 / Mad_Sigma ⇒ 中位 × Mad_Sigma ≤ 1
      for I in 0 .. Natural (R.Length) - 1 loop
         if Extra (I) > 0.0 then
            Hi := Long_Float'Max (Hi, Kinem.Mad_Sigma * abs R (I) / Sqrt (Extra (I)));
         end if;
      end loop;
      if Hi <= 0.0 then
         return 0.0;
      end if;
      while F (Hi) > 1.0 loop   --  Extra = 0 的那几条不随 S 变:上界不够就翻倍(翻不到 ⇒ 这几条压过中位,S 取多大都不够,取到溢出为止)
         exit when Hi > Long_Float'Last - Hi;   --  再翻倍就溢出
         Hi := 2.0 * Hi;
      end loop;
      loop
         declare
            Mid : constant Long_Float := 0.5 * (Lo + Hi);
         begin
            exit when Mid <= Lo or else Mid >= Hi;
            if F (Mid) > 1.0 then
               Lo := Mid;
            else
               Hi := Mid;
            end if;
         end;
      end loop;
      return Hi;
   end Scale_Root;

   procedure Noise_Of (Ep, Vp, Ea, Va, Vd : Floats; Grp : Ints; Live : Bools; N_Grp : Natural; Sig0 : Long_Float; Sm : out Floats; K : out Long_Float) is
      N : constant Natural := Natural (Ep.Length);
   begin
      Sm := Filled (N_Grp, Sig0);
      K := 1.0;
      for G in 0 .. N_Grp - 1 loop
         declare
            R, V, One : Floats;
         begin
            for I in 0 .. N - 1 loop
               if Live (I) and then Grp (I) = G then
                  R.Append (Ep (I)); V.Append (Vp (I)); One.Append (1.0);
               end if;
            end loop;
            if not R.Is_Empty then
               Sm.Replace_Element (G, Scale_Root (R, V, One));
            end if;
         end;
      end loop;
      declare
         R, V, D : Floats;
      begin
         for I in 0 .. N - 1 loop
            if Live (I) and then Vd (I) > 0.0 then
               R.Append (Ea (I)); V.Append (Sm (Natural (Grp (I))) ** 2 + Va (I)); D.Append (Vd (I));
            end if;
         end loop;
         if not R.Is_Empty then
            K := Scale_Root (R, V, D);
         end if;
      end;
   end Noise_Of;

   function Peak_Of (T, St : Floats) return Integer is
      Best : Long_Float := -1.0;
      Bi : Integer := -1;
   begin
      for C in 0 .. Natural (T.Length) - 1 loop
         declare
            D : Long_Float := 0.0;
         begin
            for J in 0 .. Natural (T.Length) - 1 loop
               if St (J) > 0.0 then
                  D := D + Exp (-0.5 * ((T (C) - T (J)) / St (J)) ** 2) / St (J);   --  正态核(指数里的 ½ 是正态密度本身的,纯数学)
               end if;
            end loop;
            if D > Best then
               Best := D; Bi := C;
            end if;
         end;
      end loop;
      return Bi;
   end Peak_Of;

   --  N × N 求逆(列主元高斯-约当);奇异 ⇒ Ok = False
   type Mat is array (Natural range <>, Natural range <>) of Long_Float;
   procedure Invert (A : in out Mat; Ok : out Boolean) is
      N : constant Natural := A'Length (1);
      B : Mat (0 .. N - 1, 0 .. 2 * N - 1) := [others => [others => 0.0]];
   begin
      Ok := False;
      for I in 0 .. N - 1 loop
         for J in 0 .. N - 1 loop
            B (I, J) := A (A'First (1) + I, A'First (2) + J);
         end loop;
         B (I, N + I) := 1.0;
      end loop;
      for C in 0 .. N - 1 loop
         declare
            Pv : Natural := C;
         begin
            for I in C + 1 .. N - 1 loop
               if abs B (I, C) > abs B (Pv, C) then
                  Pv := I;
               end if;
            end loop;
            if B (Pv, C) = 0.0 then
               return;
            end if;
            if Pv /= C then
               for J in 0 .. 2 * N - 1 loop
                  declare
                     Tmp : constant Long_Float := B (C, J);
                  begin
                     B (C, J) := B (Pv, J); B (Pv, J) := Tmp;
                  end;
               end loop;
            end if;
            declare
               D : constant Long_Float := B (C, C);
            begin
               for J in 0 .. 2 * N - 1 loop
                  B (C, J) := B (C, J) / D;
               end loop;
            end;
            for I in 0 .. N - 1 loop
               if I /= C and then B (I, C) /= 0.0 then
                  declare
                     F : constant Long_Float := B (I, C);
                  begin
                     for J in 0 .. 2 * N - 1 loop
                        B (I, J) := B (I, J) - F * B (C, J);
                     end loop;
                  end;
               end if;
            end loop;
         end;
      end loop;
      for I in 0 .. N - 1 loop
         for J in 0 .. N - 1 loop
            A (A'First (1) + I, A'First (2) + J) := B (I, N + J);
         end loop;
      end loop;
      Ok := True;
   end Invert;
   --  3 × 3 对称阵的特征分解(循环 Jacobi,转到非对角全为 0 或不再变小):Ev 升序,V 的列 = 对应的单位特征向量
   procedure Eig3 (A : Geom.M3; Ev : out Geom.V3; V : out Geom.M3) is
      M : Geom.M3 := A;
      Prev : Long_Float := Long_Float'Last;
   begin
      V := Geom.Identity;
      loop
         declare
            Off : constant Long_Float := M (0, 1) ** 2 + M (0, 2) ** 2 + M (1, 2) ** 2;
         begin
            exit when Off = 0.0 or else Off >= Prev;
            Prev := Off;
         end;
         for P in 0 .. 1 loop
            for Q in P + 1 .. 2 loop
               if M (P, Q) /= 0.0 then
                  declare
                     Th : constant Long_Float := 0.5 * Arctan (2.0 * M (P, Q), M (Q, Q) - M (P, P));   --  转掉 (P, Q) 那一格的角(二倍角公式,纯数学)
                     C : constant Long_Float := Cos (Th);
                     S : constant Long_Float := Sin (Th);
                     Mn : Geom.M3 := M;
                     Vn : Geom.M3 := V;
                  begin
                     for K in 0 .. 2 loop
                        Mn (K, P) := C * M (K, P) - S * M (K, Q);
                        Mn (K, Q) := S * M (K, P) + C * M (K, Q);
                     end loop;
                     M := Mn;
                     for K in 0 .. 2 loop
                        Mn (P, K) := C * M (P, K) - S * M (Q, K);
                        Mn (Q, K) := S * M (P, K) + C * M (Q, K);
                     end loop;
                     M := Mn;
                     for K in 0 .. 2 loop
                        Vn (K, P) := C * V (K, P) - S * V (K, Q);
                        Vn (K, Q) := S * V (K, P) + C * V (K, Q);
                     end loop;
                     V := Vn;
                  end;
               end if;
            end loop;
         end loop;
      end loop;
      Ev := [M (0, 0), M (1, 1), M (2, 2)];
      --  升序排(交换列)
      for I in 0 .. 1 loop
         for J in I + 1 .. 2 loop
            if Ev (J) < Ev (I) then
               declare
                  Te : constant Long_Float := Ev (I);
               begin
                  Ev (I) := Ev (J); Ev (J) := Te;
                  for K in 0 .. 2 loop
                     declare
                        Tv : constant Long_Float := V (K, I);
                     begin
                        V (K, I) := V (K, J); V (K, J) := Tv;
                     end;
                  end loop;
               end;
            end if;
         end loop;
      end loop;
   end Eig3;

   procedure Place_Hand (Obs : Hand_Ob_Vectors.Vector; Tie : Plane_Tie; Sig0 : Long_Float; P : in out Hand_Place; Ok : out Boolean) is separate;
   procedure Hand_Sd (Obs : Hand_Ob_Vectors.Vector; Tie : Plane_Tie; P : in out Hand_Place) is separate;

   function Eye_List (N_Cams : Natural; Eyes, Groups : Ints; Valid : Bools; World_Cam, Ref : Integer; Fixed_Placed : Boolean) return Eye_Vectors.Vector is
      L : Eye_Vectors.Vector := Eye_Vectors.To_Vector ((others => <>), Ada.Containers.Count_Type (N_Cams));
      Installed : Natural := 0;   --  装上以后的第几只手(同 Install:Valid 的按先后数)
   begin
      for A in 0 .. Natural (Eyes.Length) - 1 loop
         declare
            Ok_A : constant Boolean := A < Natural (Valid.Length) and then Valid (A);
         begin
            if Eyes (A) >= 0 and then Natural (Eyes (A)) < N_Cams then
               L.Replace_Element (Natural (Eyes (A)), Eye_Info'(Kind => On_Group, Group => (if A < Natural (Groups.Length) then Groups (A) else -1),
                                                               Arm => (if Ok_A then Integer (Installed) else -1), Placed => Ok_A, World => A = Ref and then Ok_A));
            end if;
            if Ok_A then
               Installed := Installed + 1;
            end if;
         end;
      end loop;
      if World_Cam >= 0 and then Natural (World_Cam) < N_Cams and then L (Natural (World_Cam)).Kind = Unclear then
         L.Replace_Element (Natural (World_Cam), Eye_Info'(Kind => Still, Group => -1, Arm => -1, Placed => Fixed_Placed, World => False));
      end if;
      return L;
   end Eye_List;

   function Eye_Say (L : Eye_Vectors.Vector) return String is
      R : Unbounded_String;
   begin
      for C in 0 .. Natural (L.Length) - 1 loop
         declare
            E : constant Eye_Info := L (C);
         begin
            Append (R, (if C > 0 then ";" else "") & "第" & Codec.Img (C) & " 台相机 ");
            case E.Kind is
               when On_Group =>
                  Append (R, "长在第" & Codec.Img (Natural'Max (0, E.Group)) & " 组读数上"
                          & (if E.Arm >= 0 then "(装上了,第" & Codec.Img (Natural (E.Arm) + 1) & " 只手" & (if E.World then ",世界就是它的系" else "") & ")"
                             else "(那只手这回没量成 / 放不进世界 ⇒ 用不了;它照样跟着那组读数动,不是不动的眼)"));
               when Still =>
                  Append (R, "推哪组读数都不动(不动的眼," & (if E.Placed then "放进世界了)" else "没放进世界)"));
               when Unclear =>
                  Append (R, "量不清长在谁身上(推哪组都没认出它跟着动,也不是最不动的那只)");
            end case;
         end;
      end loop;
      return To_String (R);
   end Eye_Say;

   function World_Arm (Tilt_Sd : Floats) return Integer is
   begin
      for A in 0 .. Natural (Tilt_Sd.Length) - 1 loop
         if Tilt_Sd (A) >= 0.0 then
            return A;
         end if;
      end loop;
      return -1;
   end World_Arm;

   procedure Align (Ds : Sweep_Vectors.Vector; Worlds : in out Arm_World_Vectors.Vector; Css : Corr_Set_Vectors.Vector;
                    Host : String; Port : Natural; Rw : out Geom.M3; O : out Geom.V3; Ok : out Boolean; Fixed_Eye : out Geom.Cam_Geo;
                    Board : out Geom.Scene_Pt_Vectors.Vector; Plane_Pt, Plane_N : out Geom.V3; Plane_Rms : out Long_Float; Dump : String := "";
                    Pin_Fixed_F : Long_Float := 0.0;
                    Eyes : Ints := Int_Vectors.Empty_Vector; World_Cam : Integer := -1; N_Cams : Natural := 0) is separate;

   --  ── 到过的范围(09-29,owner:"已知范围,越用越大"):到过的范围、往外一步、尽头 ──

   procedure Set_Ranges (D : Sweep_Data; W : in out Arm_World) is
   begin
      W.Lo.Clear; W.Hi.Clear; W.Got_Lo.Clear; W.Got_Hi.Clear; W.Step_Lo.Clear; W.Step_Hi.Clear;
      W.Eye_W := D.W;
      if D.Frames.Is_Empty then
         return;
      end if;
      for J in 0 .. Natural (D.Frames (0).Q.Length) - 1 loop
         declare
            Lo : Long_Float := Long_Float'Last;
            Hi : Long_Float := Long_Float'First;
            Lim_Lo : constant Boolean := J < Natural (D.Has_Lo.Length) and then D.Has_Lo (J);
            Lim_Hi : constant Boolean := J < Natural (D.Has_Hi.Length) and then D.Has_Hi (J);
         begin
            for Fr of D.Frames loop
               if J < Natural (Fr.Q.Length) then
                  Lo := Long_Float'Min (Lo, Fr.Q (J)); Hi := Long_Float'Max (Hi, Fr.Q (J));
               end if;
            end loop;
            --  记下的尽头:扫描时这一边是"关节到头"停的,以扫到的最远那一格为界;走满格数 / 碰上东西了停的这一边没量到头,不设界
            --  (V1B18 2026-09-27:问"够不够得着"时只按扫到过的范围解,碰桌面前转手转到 0.32 弧度就解不出更远的了 ⇒ 问够不够得着只按尽头)
            W.Lo.Append (if Lim_Lo then Lo else Long_Float'First);
            W.Hi.Append (if Lim_Hi then Hi else Long_Float'Last);
            W.Got_Lo.Append (Lo); W.Got_Hi.Append (Hi);
            W.Step_Lo.Append (if J < Natural (D.Step_Lo.Length) then D.Step_Lo (J) else 0.0);
            W.Step_Hi.Append (if J < Natural (D.Step_Hi.Length) then D.Step_Hi (J) else 0.0);
         end;
      end loop;
   end Set_Ranges;

   procedure Cmd_Bounds (W : Arm_World; Lo, Hi : out Floats) is
      N : constant Natural := Natural'Min (Natural (W.Lo.Length), Natural (W.Hi.Length));
   begin
      Lo.Clear; Hi.Clear;
      for J in 0 .. N - 1 loop
         declare
            Has : constant Boolean := J < Natural (W.Got_Lo.Length) and then J < Natural (W.Got_Hi.Length)
              and then J < Natural (W.Step_Lo.Length) and then J < Natural (W.Step_Hi.Length);
         begin
            Lo.Append (if Has then Long_Float'Max (W.Lo (J), W.Got_Lo (J) - W.Step_Lo (J)) else W.Lo (J));
            Hi.Append (if Has then Long_Float'Min (W.Hi (J), W.Got_Hi (J) + W.Step_Hi (J)) else W.Hi (J));
         end;
      end loop;
   end Cmd_Bounds;

   function Judge_End (Q_Cmd, Q_At, Q_Now, Got_Lo, Got_Hi, Step_Lo, Step_Hi : Floats; Tol : Long_Float; J : out Integer; Hi_Side : out Boolean) return End_Verdict is separate;

   function End_Passed (Q, Lo, Hi : Floats; Tol : Long_Float; J : out Integer; Hi_Side : out Boolean) return Boolean is
   begin
      J := -1; Hi_Side := False;
      for K in 0 .. Natural'Min (Natural (Q.Length), Natural'Min (Natural (Lo.Length), Natural (Hi.Length))) - 1 loop
         if Hi (K) /= Long_Float'Last and then Q (K) > Hi (K) + Tol then
            J := K; Hi_Side := True;
            return True;
         elsif Lo (K) /= Long_Float'First and then Q (K) < Lo (K) - Tol then
            J := K; Hi_Side := False;
            return True;
         end if;
      end loop;
      return False;
   end End_Passed;

   --  判"到了 / 停在界上"的最小一档:这只手那只眼里画面挪不到 1 像素的转角(Kinem.Clean_Tol,同扫描收格子);画幅不知道 ⇒ 0(只认正好相等,宁可不记)
   function Tol_Of (W : Arm_World) return Long_Float is (if W.Eye_W > 0 then Kinem.Clean_Tol (Long_Float (W.Eye_W)) else 0.0);

   --  ── 装上以后插头用的状态 ──
   St_Worlds : Arm_World_Vectors.Vector;
   St_Rw : Geom.M3 := Geom.Identity;
   St_O : Geom.V3 := [0.0, 0.0, 0.0];
   St_Last : Plug.Floats_Vectors.Vector;   --  每只手最近一帧的关节读数(反解从这儿起)
   St_Noise : Long_Float := 0.0;           --  开机量的关节读数噪声(判"停下了"的下限)
   Still_Frac : constant := 0.01;          --  停下了 = 一拍挪的不到这条命令的百分之一(比例;同 Selfmap.Go 判关节目标停了)
   --  每只手上一条位姿命令:要不要等它停下核有没有碰到尽头、反解是不是被"往外一步"卡住了
   type Pend_State is record
      Live : Boolean := False;             --  有一条要到范围外的命令还没核
      Q_Cmd, Q_At, Glo, Ghi : Floats;      --  它的关节目标、发命令时的读数、发命令时到过的范围
      Still : Natural := 0;                --  停下了几拍
      Held_Back : Boolean := False;        --  上一条命令有关节被夹到"到过的范围 + 往外一步"(没发到解出来的那一处)
      Cmd_Glo, Cmd_Ghi : Floats;           --  发上一条命令时到过的范围(之后长了,重解才会往前走)
   end record;
   package Pend_Vectors is new Ada.Containers.Vectors (Natural, Pend_State);
   St_Pend : Pend_Vectors.Vector;
   --  身体文件(Remember_Kin):干活时范围长了 / 尽头变了写回
   St_Kin_Path : Unbounded_String;
   St_Kin : Kin_Store;
   St_Kin_Idx : Ints;                      --  St_Worlds 第 i 只手是 St_Kin.Worlds 的第几只
   St_Saved_Glo, St_Saved_Ghi : Plug.Floats_Vectors.Vector;   --  上回写文件时每只手到过的范围

   procedure Remember_Kin (Path : String; K : Kin_Store) is
   begin
      St_Kin_Path := To_Unbounded_String (Path);
      St_Kin := K;
   end Remember_Kin;

   procedure Install (Worlds : Arm_World_Vectors.Vector; Rw : Geom.M3; O : Geom.V3; Joint_Noise : Long_Float := 0.0) is
   begin
      St_Worlds.Clear; St_Last.Clear; St_Pend.Clear; St_Kin_Idx.Clear; St_Saved_Glo.Clear; St_Saved_Ghi.Clear;
      for I in 0 .. Natural (Worlds.Length) - 1 loop
         if Worlds (I).Valid then
            St_Worlds.Append (Worlds (I));
            St_Last.Append (Worlds (I).Model.Q0);
            St_Pend.Append (Pend_State'(others => <>));
            St_Kin_Idx.Append (I);
            St_Saved_Glo.Append (Worlds (I).Got_Lo); St_Saved_Ghi.Append (Worlds (I).Got_Hi);
         end if;
      end loop;
      St_Rw := Rw; St_O := O; St_Noise := Joint_Noise;
      Plug.Set_Hooks (Pose_Hook'Access, Cmd_Hook'Access);
      Plug.Set_Reach (Reach_Hook'Access);
      Plug.Set_Limit (Held_Back'Access);
      Say ("装上:从此每一帧手的位姿 = 按关节读数算出的腕眼位姿(" & Codec.Img (Natural (St_Worlds.Length)) & " 只手),位姿命令 = 按记下的尽头解出关节目标、"
           & "每个关节夹到到过的范围往外一步里发(到过的范围;问够不够得着只按尽头)");
   end Install;

   --  上一条被截住了没有;截住了,从那一条以来到过的范围长了没有(不止一档)—— 手还没动(命令要隔一两拍才起效)⇒ Held,截住的状态留着;
   --  手一动、范围一长 ⇒ Held_Grown,重解重发。手停在真的尽头 / 碰上东西 ⇒ 范围不再长 ⇒ 一直 Held,照常停下、核尽头
   function Held_Back (Arm : Natural) return Plug.Limit_State is
   begin
      if Arm >= Natural (St_Pend.Length) or else Arm >= Natural (St_Worlds.Length) or else not St_Pend (Arm).Held_Back then
         return Plug.Free;
      end if;
      declare
         P : constant Pend_State := St_Pend (Arm);
         W : constant Arm_World := St_Worlds (Arm);
         Tol : constant Long_Float := Tol_Of (W);
      begin
         for J in 0 .. Natural'Min (Natural (W.Got_Hi.Length), Natural'Min (Natural (P.Cmd_Ghi.Length), Natural (P.Cmd_Glo.Length))) - 1 loop
            if W.Got_Hi (J) > P.Cmd_Ghi (J) + Tol or else W.Got_Lo (J) < P.Cmd_Glo (J) - Tol then
               return Plug.Held_Grown;
            end if;
         end loop;
         return Plug.Held;
      end;
   end Held_Back;

   --  到过的范围长了一步以上、或者尽头变了 ⇒ 写回身体文件(只写字)
   procedure Save_If_Grown (Changed_End : Boolean) is separate;

   --  这一帧第 A 只手的读数:到过的范围并进来;越过记下的尽头 ⇒ 删掉那个尽头;有一条要到范围外的命令 ⇒ 等它停下(连着两拍一拍挪不到这条命令的百分之一)
   --  核有没有碰到尽头(Judge_End):正好一个关节没走到一半、别的都到了 ⇒ 这一头到了,记下;别的关节没到 / 被顶偏 ⇒ 碰上东西了,不记
   procedure Track (A : Natural; Prev, Qn : Floats) is separate;

   procedure Pose_Hook (F : in out Plug.Frame) is
      use Geom;
   begin
      F.EE.Clear;
      for A in 0 .. Natural (St_Worlds.Length) - 1 loop
         if St_Worlds (A).Group < Natural (F.Joints.Length) then
            Track (A, St_Last (A), F.Joints (St_Worlds (A).Group));
         end if;
         declare
            W : Arm_World renames St_Worlds (A);
            Rr : M3;
            Tt : V3;
         begin
            if W.Group < Natural (F.Joints.Length) then
               St_Last.Replace_Element (A, F.Joints (W.Group));
               Kinem.FK (W.Model, F.Joints (W.Group), Rr, Tt);
               declare
                  R0 : constant M3 := Mul (W.Ra, Rr);
                  Rt : constant V3 := Ap (W.Ra, Tt);
                  T0 : constant V3 := [W.S * Rt (0) + W.Ta (0) - St_O (0), W.S * Rt (1) + W.Ta (1) - St_O (1), W.S * Rt (2) + W.Ta (2) - St_O (2)];
               begin
                  F.EE.Append (Kinem.To_Pose (Mul (St_Rw, R0), Ap (St_Rw, T0)));
               end;
            else
               F.EE.Append (Plug.Arm_Pose'[others => 0.0]);
            end if;
         end;
      end loop;
   end Pose_Hook;

   --  世界里的一个腕眼位姿 ⇒ 第 A 只手的关节目标(位姿命令、开机自检、问够不够得着都走这一条)。
   --  每个关节夹到"到过的范围 + 往外一步"里(Cmd_Bounds);Clamped = 有关节被夹住了
   procedure Clamp_Cmd (W : Arm_World; Q : in out Floats; Clamped : out Boolean) is
      Lo, Hi : Floats;
   begin
      Clamped := False;
      Cmd_Bounds (W, Lo, Hi);
      for J in 0 .. Natural'Min (Natural (Q.Length), Natural'Min (Natural (Lo.Length), Natural (Hi.Length))) - 1 loop
         if Q (J) > Hi (J) then
            Q.Replace_Element (J, Hi (J)); Clamped := True;
         elsif Q (J) < Lo (J) then
            Q.Replace_Element (J, Lo (J)); Clamped := True;
         end if;
      end loop;
   end Clamp_Cmd;

   --  先只按记下的尽头解出这个位姿要的关节(Pe / Re = 解完还差多少:位置按第一只手的模型单位 = 世界的单位,朝向按弧度)。
   --  For_Command = 真要发出去(到过的范围):每个关节再夹到"到过的范围 + 往外一步"里 —— 每个关节直接朝那个解走、一条命令最多走出到过的地方一步;
   --  Clamped = 有关节被夹住了(这一条到不了那个解,手走过去、范围长了再往前)。
   --  (09-29 离线接真 Go 查出来的:原来直接在"到过的范围 + 一步"里反解,被夹住的那个关节差的那点由别的关节凑 ——
   --  只转第 4 个关节到 0.8,第 0、2 个关节中途被拉出去 0.23 弧度再转回来;到不了时停在一个扭着的姿势)
   procedure Pose_To_Q (A : Natural; Pose : Plug.Arm_Pose; For_Command : Boolean; Q : out Floats; Pe, Re : out Long_Float; Clamped : out Boolean) is
      use Geom;
      W : Arm_World renames St_Worlds (A);
      Rt_W : constant M3 := Quat_To_R (Pose);
      Tt_W : constant V3 := [Pose (0), Pose (1), Pose (2)];
      --  世界 → 第一只手参照眼系 → 这只手参照眼系
      R0 : constant M3 := Mul (Tr (St_Rw), Rt_W);
      T0w : constant V3 := Ap (Tr (St_Rw), Tt_W);
      T0 : constant V3 := [T0w (0) + St_O (0), T0w (1) + St_O (1), T0w (2) + St_O (2)];
      Ra_T : constant M3 := Tr (W.Ra);
      Ra_Arm : constant M3 := Mul (Ra_T, R0);
      Ta_D : constant V3 := Ap (Ra_T, [T0 (0) - W.Ta (0), T0 (1) - W.Ta (1), T0 (2) - W.Ta (2)]);
      Ta_Arm : constant V3 := [Ta_D (0) / W.S, Ta_D (1) / W.S, Ta_D (2) / W.S];
   begin
      Clamped := False;
      Kinem.IK (W.Model, Ra_Arm, Ta_Arm, St_Last (A), W.Lo, W.Hi, Q, Pe, Re);
      Pe := Pe * W.S;   --  换成第一只手的模型单位(= 世界的单位)
      if For_Command then
         Clamp_Cmd (W, Q, Clamped);
      end if;
   end Pose_To_Q;

   procedure Reach_Hook (Arm : Natural; Pose : Plug.Arm_Pose; Pos_Err, Rot_Err : out Long_Float) is
      Q : Floats;
      Cl : Boolean;
   begin
      Pos_Err := Long_Float'Last; Rot_Err := Long_Float'Last;
      if Arm < Natural (St_Worlds.Length) then
         Pose_To_Q (Arm, Pose, False, Q, Pos_Err, Rot_Err, Cl);
      end if;
   end Reach_Hook;

   --  一条要发出去的位姿命令解成了 Q(Full = 只按记下的尽头解出来的那一个,Q = 夹到"到过的范围 + 往外一步"里以后真发的):
   --  记下要不要等它停下核尽头(有关节的目标出了到过的范围)、有没有被夹住(重不重发看之后范围长没长,见 Held_Back)
   procedure Note_Command (A : Natural; Q, Full : Floats; Clamped : Boolean) is separate;

   procedure Cmd_Hook (C : in out Plug.Cmd; Ok : out Boolean) is
      Q, Full : Floats;
      Pe, Re : Long_Float;
      Cl : Boolean;
   begin
      Ok := False;
      if C.Arm >= Natural (St_Worlds.Length) then
         return;
      end if;
      Pose_To_Q (C.Arm, C.Pose, False, Full, Pe, Re, Cl);
      Q := Full;
      Clamp_Cmd (St_Worlds (C.Arm), Q, Cl);
      Note_Command (C.Arm, Q, Full, Cl);
      C.Kind := Plug.Joint;
      C.Group := St_Worlds (C.Arm).Group;
      C.Q := Q;
      Ok := True;
   end Cmd_Hook;
   N_Check : constant := 3;   --  每只手走几处(次数)
   procedure Self_Check (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; Ds : Sweep_Vectors.Vector; Dump : String) is separate;

   --  ── ⑦ 存 / 装回 ──
   function Kin_Key (L : Plug.Link; F : Plug.Frame) return String is
      R : Unbounded_String;
   begin
      Append (R, "cams=");
      for C of F.Cams loop
         Append (R, Codec.Img (C.W) & "x" & Codec.Img (C.H) & ",");
      end loop;
      Append (R, ";groups=");
      for G of F.Joints loop
         Append (R, Codec.Img (Natural (G.Length)) & ",");
      end loop;
      Append (R, ";jaws=" & Codec.Img (Natural (F.Jaw.Length)) & ";joints=");
      for P of L.Lay.Joints loop
         Append (R, Layout.Last_Seg (P) & ",");
      end loop;
      declare
         S : String := To_String (R);
      begin
         for I in S'Range loop
            if S (I) = ' ' then
               S (I) := '_';   --  钥匙在文件里是一行里的一个词
            end if;
         end loop;
         return S;
      end;
   end Kin_Key;

   function F9 (X : Long_Float) return String is (Codec.Fmt (X, 9));
   function Lim (X : Long_Float) return String is
     (if X = Long_Float'First or else X = Long_Float'Last then "none" else F9 (X));   --  没量到头的界记成 none

   procedure Save_Kin (Path : String; K : Kin_Store; Images : Boolean := True) is separate;

   procedure Load_Kin (Path : String; K : out Kin_Store; Ok : out Boolean; Note : out Unbounded_String) is separate;

   function Same_View (Disp : Floats) return Boolean is
      package Sorting is new F64_Vectors.Generic_Sorting;
      D : Floats := Disp;
   begin
      if Natural (D.Length) < Min_Inl then
         return False;
      end if;
      Sorting.Sort (D);
      return D (Natural (D.Length) / 2) < Trip_Px;
   end Same_View;

   --  一对图(存的 → 此刻)问格点、往返 1 px 内的留下 ⇒ 每个留下的点挪了多少像素
   procedure View_Shift (Host : String; Port : Natural; A_Img, B_Img : Plug.Cam; Disp : out Floats; Err : out Unbounded_String) is
      Ia, Ib : Integer;
      Q : Instrument.Match_Vectors.Vector;
   begin
      Disp.Clear;
      Instrument.Frame_Put (Host, Port, A_Img.RGB, A_Img.W, A_Img.H, Ia, Err);
      Instrument.Frame_Put (Host, Port, B_Img.RGB, B_Img.W, B_Img.H, Ib, Err);
      if Ia < 0 or else Ib < 0 then
         return;
      end if;
      for Gyy in 0 .. Gy - 1 loop
         for Gxx in 0 .. Gx - 1 loop
            Q.Append (Instrument.Match_Pt'(U => Kinem.Grid_U (Gxx, A_Img.W), V => Kinem.Grid_V (Gyy, A_Img.H), others => <>));
         end loop;
      end loop;
      declare
         R : constant Instrument.Match_Vectors.Vector := Instrument.Match_Ids (Host, Port, Natural (Ia), Natural (Ib), Q, Err, Coarse => True, Back => True);
      begin
         if Natural (R.Length) = Natural (Q.Length) then
            for G in 0 .. Natural (Q.Length) - 1 loop
               if R (G).Bu >= 0.0 and then R (G).U >= 0.0 and then Geom.Norm ([R (G).Bu - Q (G).U, R (G).Bv - Q (G).V, 0.0]) < Trip_Px then
                  Disp.Append (Geom.Norm ([R (G).U - Q (G).U, R (G).V - Q (G).V, 0.0]));
               end if;
            end loop;
         end if;
      end;
   end View_Shift;

   procedure Check_Kin (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; K : Kin_Store; Host : String; Port : Natural;
                        Ok : out Boolean; Note : out Unbounded_String) is separate;

   procedure Dump_Kin (Dump : String; K : Kin_Store) is separate;
end Jointboot;
