with Ada.Unchecked_Deallocation;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Numerics.Float_Random;
with Ada.Text_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Json;
with Codec;
with Stats;
package body Geom is

   function Quat_To_R (P : Plug.Arm_Pose) return M3 is
      --  读数不一定精确归一(四位小数就够让矩阵不正交):先归一
      Nq : constant Long_Float := Long_Float'Max (1.0e-12, Sqrt (P (3) ** 2 + P (4) ** 2 + P (5) ** 2 + P (6) ** 2));
      W : constant Long_Float := P (3) / Nq; X : constant Long_Float := P (4) / Nq;
      Y : constant Long_Float := P (5) / Nq; Z : constant Long_Float := P (6) / Nq;
   begin
      return [[1.0 - 2.0 * (Y * Y + Z * Z), 2.0 * (X * Y - Z * W), 2.0 * (X * Z + Y * W)],
              [2.0 * (X * Y + Z * W), 1.0 - 2.0 * (X * X + Z * Z), 2.0 * (Y * Z - X * W)],
              [2.0 * (X * Z - Y * W), 2.0 * (Y * Z + X * W), 1.0 - 2.0 * (X * X + Y * Y)]];
   end Quat_To_R;

   function Mul (A, B : M3) return M3 is
      R : M3 := [others => [others => 0.0]];
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            for K in 0 .. 2 loop
               R (I, J) := R (I, J) + A (I, K) * B (K, J);
            end loop;
         end loop;
      end loop;
      return R;
   end Mul;

   function Tr (A : M3) return M3 is
      R : M3;
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            R (I, J) := A (J, I);
         end loop;
      end loop;
      return R;
   end Tr;

   function Ap (A : M3; X : V3) return V3 is
      R : V3 := [others => 0.0];
   begin
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            R (I) := R (I) + A (I, J) * X (J);
         end loop;
      end loop;
      return R;
   end Ap;

   function Norm (X : V3) return Long_Float is (Sqrt (X (0) ** 2 + X (1) ** 2 + X (2) ** 2));

   function Rodrigues (R : V3) return M3 is
      Th : constant Long_Float := Norm (R);
      K : V3;
      Kx : M3;
      Res : M3 := Identity;
   begin
      if Th < 1.0e-12 then
         return Identity;
      end if;
      K := [R (0) / Th, R (1) / Th, R (2) / Th];
      Kx := [[0.0, -K (2), K (1)], [K (2), 0.0, -K (0)], [-K (1), K (0), 0.0]];
      declare
         Kx2 : constant M3 := Mul (Kx, Kx);
         S : constant Long_Float := Sin (Th);
         Cc : constant Long_Float := 1.0 - Cos (Th);
      begin
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Res (I, J) := Res (I, J) + S * Kx (I, J) + Cc * Kx2 (I, J);
            end loop;
         end loop;
      end;
      return Res;
   end Rodrigues;

   function Rot_Vec (A : M3) return V3 is
      C : Long_Float := (A (0, 0) + A (1, 1) + A (2, 2) - 1.0) / 2.0;
      Th : Long_Float;
      S : Long_Float;
   begin
      C := Long_Float'Max (-1.0, Long_Float'Min (1.0, C));
      Th := Arccos (C);
      if Th < 1.0e-9 then
         return [0.0, 0.0, 0.0];
      end if;
      S := 2.0 * Sin (Th);
      return [(A (2, 1) - A (1, 2)) / S * Th, (A (0, 2) - A (2, 0)) / S * Th, (A (1, 0) - A (0, 1)) / S * Th];
   end Rot_Vec;

   function Cam_R (G : Cam_Geo; P : Plug.Arm_Pose) return M3 is (Mul (Quat_To_R (P), G.R_Ce));
   function Cam_Pos (G : Cam_Geo; P : Plug.Arm_Pose) return V3 is
      Ow : constant V3 := Ap (Quat_To_R (P), G.Off);
   begin
      return [P (0) + Ow (0), P (1) + Ow (1), P (2) + Ow (2)];
   end Cam_Pos;

   --  径向畸变(归一化平面,r² = x² + y²):畸变后 = 理想 × (1 + K1 r² + K2 r⁴)
   procedure Distort (G : Cam_Geo; X, Y : Long_Float; Xd, Yd : out Long_Float) is
      R2 : constant Long_Float := X * X + Y * Y;
      D : constant Long_Float := 1.0 + G.K1 * R2 + G.K2 * R2 * R2;
   begin
      Xd := X * D; Yd := Y * D;
   end Distort;

   --  反过来:从畸变后的点回到理想的点(09-30 改成一维牛顿法)。畸变只改离主点多远、不改方位 ⇒ 只解一个方程 r·(1 + K1 r² + K2 r⁴) = r畸
   --  (r畸 = 畸变后的点离主点多远,归一化平面)。只在单调的那一段里解:r 从 0 往外,畸变后的半径先跟着涨,到折回半径 r*
   --  (导数 1 + 3K1 r² + 5K2 r⁴ 第一次到 0)处最大,再往外反而变小 —— r* 以外的方向和 r* 以内的某个方向落在同一个像素上,
   --  镜头模型在那儿已经不成立。
   --  r畸 比 r* 处能到的还大 ⇒ 没有哪条视线落在这个像素上 ⇒ Ok = False(调用方照实说"这一点去不了畸变")。有根时根夹在 [0, r*] 里
   --  (一直单调时夹在 [0, r畸 ÷ 导数的最小值] 里):牛顿一步走出夹住的区间就改成对半分,每一步都收窄区间,做到 r 不再变
   --  (或区间只剩相邻两个浮点数)为止;遍数上限只当保险 = 对半分把任何有限区间收到一个浮点数最多要的步数,碰到就 Ok = False,照实报。
   --  原来的不动点迭代 x = x畸 ÷ (1 + K1 r² + K2 r⁴) 在 r* 处斜率正好是 1:靠近折回半径时 50 次停在半路,超出能到的最大半径时发一个
   --  折到主点另一边的"解",都不报(K1 = −0.35:r = 0.975 差 0.0115,r畸 = 0.66 解成 2.39)
   procedure Undistort (G : Cam_Geo; Xd, Yd : Long_Float; X, Y : out Long_Float; Ok : out Boolean) is separate;

   function Cam_Dir (G : Cam_Geo; U, V : Long_Float; Ok : out Boolean) return V3 is
      X, Y : Long_Float;
   begin
      Undistort (G, (U - G.Cx) / G.F, -(V - G.Cy) / G.F, X, Y, Ok);
      if not Ok then
         return [0.0, 0.0, 0.0];   --  没有视线
      end if;
      declare
         N : constant Long_Float := Sqrt (X * X + Y * Y + 1.0);
      begin
         return [X / N, Y / N, -1.0 / N];
      end;
   end Cam_Dir;

   procedure Cam_Pixel (G : Cam_Geo; Pc : V3; U, V : out Long_Float; In_Front : out Boolean) is
      Z : constant Long_Float := -Pc (2);
      Xd, Yd : Long_Float;
   begin
      In_Front := Z > 1.0e-6;
      if not In_Front then
         U := 0.0; V := 0.0;
         return;
      end if;
      Distort (G, Pc (0) / Z, Pc (1) / Z, Xd, Yd);
      U := G.Cx + G.F * Xd;
      V := G.Cy - G.F * Yd;
   end Cam_Pixel;

   function Ray (G : Cam_Geo; P : Plug.Arm_Pose; U, V : Long_Float; Ok : out Boolean) return V3 is
      Dc : constant V3 := Cam_Dir (G, U, V, Ok);
      Dw : V3 := Ap (Cam_R (G, P), Dc);
      N : constant Long_Float := Norm (Dw);
   begin
      if not Ok then
         return Dc;   --  零向量:没有视线
      end if;
      for I in 0 .. 2 loop
         Dw (I) := Dw (I) / N;
      end loop;
      return Dw;
   end Ray;
   function Ray (G : Cam_Geo; P : Plug.Arm_Pose; U, V : Long_Float) return V3 is
      Ok : Boolean;
   begin
      return Ray (G, P, U, V, Ok);
   end Ray;

   --  3×3 线性方程组,列主元消元
   function Solve3 (A : M3; B : V3) return V3 is separate;

   function To_Cam (G : Cam_Geo; P : Plug.Arm_Pose; Pw : V3) return V3 is
      Cp : constant V3 := Cam_Pos (G, P);
      D : constant V3 := [Pw (0) - Cp (0), Pw (1) - Cp (1), Pw (2) - Cp (2)];
   begin
      return Ap (Tr (Cam_R (G, P)), D);
   end To_Cam;

   procedure Project (G : Cam_Geo; P : Plug.Arm_Pose; Pw : V3; U, V : out Long_Float; In_Front : out Boolean) is
   begin
      Cam_Pixel (G, To_Cam (G, P, Pw), U, V, In_Front);
   end Project;

   --  ── 量朝向 ──
   --  ── Levenberg–Marquardt 精修(数值雅可比、正规方程高斯消元)──:参数个数由调用方定(6 = 朝向 + 点/位置;7 = 再加焦距)。
   --  Resid 把每个观测的两个像素差填进 Fill;Steps 是各参数的差分步(弧度 / 米 / 像素,极小量)。
   --  阻尼升降的两个倍数不是门槛、不影响身体动不动,只管这次拟合怎么迭代(参数向量 Param_Vec 在 geom.ads)
   --  雅可比和残差放在堆上:标定板一起解时 2 万多条残差 × 20 多个未知数,摆在 8 MB 的栈上不稳(一份雅可比就 4 MB)
   type Big_Mat is array (Natural range <>, Natural range <>) of Long_Float;
   type Big_Mat_Ptr is access Big_Mat;
   procedure Free_Mat is new Ada.Unchecked_Deallocation (Big_Mat, Big_Mat_Ptr);
   type Big_Vec_Ptr is access Param_Vec;
   procedure Free_Vec is new Ada.Unchecked_Deallocation (Param_Vec, Big_Vec_Ptr);
   function New_Vec (N : Natural) return Big_Vec_Ptr is
      V : constant Big_Vec_Ptr := new Param_Vec (0 .. Integer (N) - 1);
   begin
      for I in V'Range loop
         V (I) := 0.0;
      end loop;
      return V;
   end New_Vec;
   procedure LM_Refine (P : in out Param_Vec; N_Obs : Natural; Steps : Param_Vec; Iters : Positive;
                        Resid : access procedure (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float));
                        Cur : in out Long_Float) is separate;

   --  ── 解完之后每个参数的不确定度 ──:在解处再算一次数值雅可比 J,σ² = 残差平方和 ÷ (方程数 − 未知数),协方差 = σ² (JᵀJ)⁻¹,
   --  Sd = 对角线开方(和参数同单位)。"解不出"从此按它判:不确定度比量本身还大 = 方程分不开这个量,而不是拍一个阈值。
   --  方程数只数真的那几条(09-30):每一槽两条(u、v),Prior = 最后一槽是焦距先验,只有一条 —— 它的第二条恒为 0,原来也算进自由度,
   --  σ² 偏小(多算 1)。真方程不比未知数多 ⇒ 全是 Long_Float'Last;没有信息的参数 = Long_Float'Last
   procedure Param_Sd (P : Param_Vec; N_Obs : Natural; Prior : Boolean; Steps : Param_Vec;
                       Resid : access procedure (P : Param_Vec; R : out Long_Float; Fill : access procedure (I : Natural; Du, Dv : Long_Float));
                       Sd : out Param_Vec) is separate;

   function Pointing_Lost (G : Cam_Geo; F, Rot_Sd : Long_Float) return Boolean is (F * Rot_Sd >= Sqrt (G.Cx ** 2 + G.Cy ** 2));

   function Normal_Tail (X : Long_Float) return Long_Float is
   begin
      if X = 0.0 then
         return 0.5;
      elsif X < 0.0 then
         return 1.0 - Normal_Tail (-X);
      end if;
      --  Q(x) = φ(x) / (x + 1 / (x + 2 / (x + 3 / (x + …))))(改过的 Lentz 法,做到这一节不再改变结果)
      declare
         Tiny : constant Long_Float := Long_Float'Model_Small;
         F : Long_Float := X;
         C : Long_Float := X;
         D : Long_Float := 0.0;
         K : Natural := 0;
         Cap : constant Natural := Long_Float'Machine_Mantissa * Long_Float'Machine_Mantissa;   --  只当保险(收敛到浮点精度要的节数远比它少)
      begin
         loop
            K := K + 1;
            D := X + Long_Float (K) * D;
            D := (if D = 0.0 then 1.0 / Tiny else 1.0 / D);
            C := X + Long_Float (K) / (if C = 0.0 then Tiny else C);
            declare
               Delta_K : constant Long_Float := C * D;
            begin
               F := F * Delta_K;
               exit when abs (Delta_K - 1.0) <= Long_Float'Epsilon or else K >= Cap;
            end;
         end loop;
         return Exp (-0.5 * X * X) / Sqrt (2.0 * Ada.Numerics.Pi) / F;
      end;
   end Normal_Tail;

   function Two_More_Significant (Rss0, Rss1 : Long_Float; D2 : Natural) return Boolean is
   begin
      if D2 = 0 or else Rss1 <= 0.0 then
         return Rss0 > Rss1;   --  没有剩下的自由度 / 一点残差都没有:只能比大小
      end if;
      declare
         Alpha : constant Long_Float := Normal_Tail (Stats.Z);
         Fv : constant Long_Float := ((Rss0 - Rss1) / 2.0) / (Rss1 / Long_Float (D2));
         Fc : constant Long_Float := Long_Float (D2) / 2.0 * (Alpha ** (-2.0 / Long_Float (D2)) - 1.0);
      begin
         return Fv > Fc;
      end;
   end Two_More_Significant;

   --  前 K 个数的中位数(拷一份排序;标定的观测最多几百笔)
   function Median (Xs : Param_Vec; K : Natural) return Long_Float is
      A : Param_Vec (0 .. Natural'Max (0, K - 1)) := [others => 0.0];
   begin
      if K = 0 then
         return 0.0;
      end if;
      for I in 0 .. K - 1 loop
         A (I) := Xs (Xs'First + I);
      end loop;
      for I in 1 .. K - 1 loop   --  插入排序
         declare
            X : constant Long_Float := A (I);
            J : Integer := I - 1;
         begin
            while J >= 0 and then A (J) > X loop
               A (J + 1) := A (J); J := J - 1;
            end loop;
            A (J + 1) := X;
         end;
      end loop;
      return A (K / 2);
   end Median;

   --  离群重挑的一遍(Fit_Rig、Fit_Fixed_Board 同一套,09-30):门 = 上一遍进解那些的残差中位 × 3 —— 二维残差按瑞利分布,3 倍中位 = 3.53σ,
   --  一笔好的被踢的机会 0.2%(统计门);每一笔都按这道门重判(先前踢掉的也算,踢错的能回来)
   procedure Reselect (Rs : Param_Vec; Skip : in out Flags; Changed : out Boolean; Kept : out Natural) is
      Ks : Param_Vec (0 .. Rs'Length - 1) := [others => 0.0];
      Nk : Natural := 0;
      Med : Long_Float;
   begin
      for I in Rs'Range loop
         if not Skip (I) then
            Ks (Nk) := Rs (I); Nk := Nk + 1;
         end if;
      end loop;
      Med := Median (Ks, Nk);
      Changed := False; Kept := Nk;
      if Med <= 0.0 then
         return;   --  中位是 0:没有尺度,不挑
      end if;
      Kept := 0;
      for I in Rs'Range loop
         declare
            Out_Now : constant Boolean := Rs (I) > 3.0 * Med;
         begin
            Changed := Changed or else Out_Now /= Skip (I);
            Skip (I) := Out_Now;
            if not Out_Now then
               Kept := Kept + 1;
            end if;
         end;
      end loop;
   end Reselect;

   procedure Reselect_Loop (Errs : access procedure (Rs : out Param_Vec); Solve : access procedure (Skip : Flags);
                            Skip : in out Flags; Kept, Rounds : out Natural; How : out Reselect_End) is
      Rs : Param_Vec (Skip'Range);
      Changed : Boolean;
   begin
      Rounds := 0;
      loop
         Errs (Rs);
         Reselect (Rs, Skip, Changed, Kept);
         if not Changed then
            How := Settled;
            return;
         end if;
         if 2 * Kept < Skip'Length then   --  进解的不到一半:中位数的崩溃点(数学)
            How := Broken;
            return;
         end if;
         if Rounds >= Skip'Length then   --  保险:重解的遍数到了笔数还在变
            How := Stuck;
            return;
         end if;
         Solve (Skip);
         Rounds := Rounds + 1;
      end loop;
   end Reselect_Loop;

   --  ── 不动的眼 ──
   function Ray_Fixed (G : Cam_Geo; U, V : Long_Float; Ok : out Boolean) return V3 is
      Dc : constant V3 := Cam_Dir (G, U, V, Ok);
      Dw : V3 := Ap (G.R_Ce, Dc);
      N : constant Long_Float := Norm (Dw);
   begin
      if not Ok then
         return Dc;   --  零向量:没有视线
      end if;
      for I in 0 .. 2 loop
         Dw (I) := Dw (I) / N;
      end loop;
      return Dw;
   end Ray_Fixed;
   function Ray_Fixed (G : Cam_Geo; U, V : Long_Float) return V3 is
      Ok : Boolean;
   begin
      return Ray_Fixed (G, U, V, Ok);
   end Ray_Fixed;

   procedure Project_Fixed (G : Cam_Geo; Pw : V3; U, V : out Long_Float; In_Front : out Boolean) is
   begin
      Cam_Pixel (G, Ap (Tr (G.R_Ce), [Pw (0) - G.Pos (0), Pw (1) - G.Pos (1), Pw (2) - G.Pos (2)]), U, V, In_Front);
   end Project_Fixed;

   function Hit_Plane (Origin, Dir, P0, N : V3; Ok : out Boolean) return V3 is
      Den : constant Long_Float := Dir (0) * N (0) + Dir (1) * N (1) + Dir (2) * N (2);
      Num : constant Long_Float := (P0 (0) - Origin (0)) * N (0) + (P0 (1) - Origin (1)) * N (1) + (P0 (2) - Origin (2)) * N (2);
   begin
      Ok := False;
      if abs Den < 1.0e-12 then
         return Origin;
      end if;
      declare
         T : constant Long_Float := Num / Den;
      begin
         if T <= 0.0 then
            return Origin;
         end if;
         Ok := True;
         return [Origin (0) + T * Dir (0), Origin (1) + T * Dir (1), Origin (2) + T * Dir (2)];
      end;
   end Hit_Plane;

   --  给定朝向,相机位置有闭式最小二乘解:每条视线都该穿过它看见的那个点 ⇒ Σ(I − ddᵀ)(P − c) = 0
   function Pos_For (R : M3; G : Cam_Geo; O : Mark_Vectors.Vector) return V3 is
      A : M3 := [others => [others => 0.0]];
      B : V3 := [others => 0.0];
      Gt : Cam_Geo := G;
   begin
      Gt.R_Ce := R;
      for Ob of O loop
         declare
            Seen : Boolean;
            D : constant V3 := Ray_Fixed (Gt, Ob.U, Ob.V, Seen);
         begin
            if Seen then   --  去不了畸变的那一笔没有视线,不进
               for I in 0 .. 2 loop
                  for J in 0 .. 2 loop
                     declare
                        Pm : constant Long_Float := (if I = J then 1.0 else 0.0) - D (I) * D (J);
                     begin
                        A (I, J) := A (I, J) + Pm;
                        B (I) := B (I) + Pm * Ob.Pw (J);
                     end;
                  end loop;
               end loop;
            end if;
         end;
      end loop;
      return Solve3 (A, B);
   end Pos_For;

   procedure Fit_Fixed (G : in out Cam_Geo; O : Mark_Vectors.Vector; Ok : out Boolean) is separate;

   --  板上一个点在不动的眼里的每轴像素方差:配点噪声² + 三角的协方差投进这只眼(雅可比 ∂(u,v)/∂Pw;两轴取平均,系数无量纲)
   function Scene_Var (G : Cam_Geo; S : Scene_Pt) return Long_Float is
      Rt : constant M3 := Tr (G.R_Ce);
      Pc : constant V3 := Ap (Rt, [S.Pw (0) - G.Pos (0), S.Pw (1) - G.Pos (1), S.Pw (2) - G.Pos (2)]);
      Z : constant Long_Float := -Pc (2);
      Ju, Jv : V3 := [others => 0.0];
      function Quad (X : V3) return Long_Float is
         Sum : Long_Float := 0.0;
      begin
         for I in 0 .. 2 loop
            for K in 0 .. 2 loop
               Sum := Sum + X (I) * S.Cov (I, K) * X (K);
            end loop;
         end loop;
         return Sum;
      end Quad;
   begin
      if Z <= 0.0 or else G.F <= 0.0 then
         return S.Sh * S.Sh;
      end if;
      declare
         Du : constant V3 := [G.F / Z, 0.0, G.F * Pc (0) / (Z * Z)];
         Dv : constant V3 := [0.0, -G.F / Z, -G.F * Pc (1) / (Z * Z)];
      begin
         for K in 0 .. 2 loop
            Ju (K) := Du (0) * Rt (0, K) + Du (1) * Rt (1, K) + Du (2) * Rt (2, K);
            Jv (K) := Dv (0) * Rt (0, K) + Dv (1) * Rt (1, K) + Dv (2) * Rt (2, K);
         end loop;
      end;
      return S.Sh * S.Sh + 0.5 * (Quad (Ju) + Quad (Jv));
   end Scene_Var;

   procedure Fit_Fixed_Board (G : in out Cam_Geo; Scene : Scene_Pt_Vectors.Vector; Rep : in out Fixed_Report; Ok : out Boolean; Start_Here : Boolean := False) is separate;

   --  画面的几块:0 左半、1 右半、2 上半、3 下半、4 左上、5 右上、6 左下、7 右下(按像素;分界 = 主点,驱动的约定里主点就是画幅中心)
   function In_Region (R : Natural; U, V, Cx, Cy : Long_Float) return Boolean is
     (case R is
         when 0 => U < Cx,
         when 1 => U >= Cx,
         when 2 => V < Cy,
         when 3 => V >= Cy,
         when 4 => U < Cx and then V < Cy,
         when 5 => U >= Cx and then V < Cy,
         when 6 => U < Cx and then V >= Cy,
         when others => U >= Cx and then V >= Cy);
   function Region_Name (R : Natural) return String is
     (case R is
         when 0 => "左半边", when 1 => "右半边", when 2 => "上半边", when 3 => "下半边",
         when 4 => "左上那四分之一", when 5 => "右上那四分之一", when 6 => "左下那四分之一", when others => "右下那四分之一");
   --  点按位姿 Pg 投进画面(画幅 = 两倍主点):落在哪几块
   procedure Add_Regions (Pg : Cam_Geo; U, V : Long_Float; Reg : in out Region_Counts) is
   begin
      if U >= 0.0 and then V >= 0.0 and then U < 2.0 * Pg.Cx and then V < 2.0 * Pg.Cy then
         for R in Reg'Range loop
            if In_Region (R, U, V, Pg.Cx, Pg.Cy) then
               Reg (R) := Reg (R) + 1;
            end if;
         end loop;
      end if;
   end Add_Regions;
   function Round_Trip_Ok (Qu, Qv, Bu, Bv : Long_Float) return Boolean is
     (Bu >= 0.0 and then Bv >= 0.0 and then Sqrt ((Bu - Qu) ** 2 + (Bv - Qv) ** 2) < Trip_Px);
   function Board_Rms (G : Cam_Geo; Pts : Scene_Pt_Vectors.Vector; Gate : Long_Float) return Long_Float is
      Es : Param_Vec (0 .. Natural'Max (1, Natural (Pts.Length)) - 1) := [others => 0.0];
      Ne : Natural := 0;
      --  二维高斯误差离原点的距离服从瑞利分布:均方根 σ√2 ÷ 中位 σ√(2 ln 2) = 1/√ln2(统计换算;09-30 以前写成 1.2,差 0.1%)
      Rms_Per_Median : constant Long_Float := 1.0 / Sqrt (Log (2.0));
   begin
      for P of Pts loop
         if P.U >= 0.0 and then P.V >= 0.0 then
            declare
               U, V : Long_Float;
               Front : Boolean;
            begin
               Project_Fixed (G, P.Pw, U, V, Front);
               if Front and then Sqrt ((U - P.U) ** 2 + (V - P.V) ** 2) <= Gate then
                  Es (Ne) := Sqrt ((U - P.U) ** 2 + (V - P.V) ** 2);
                  Ne := Ne + 1;
               end if;
            end;
         end if;
      end loop;
      return (if Ne > 0 then Rms_Per_Median * Median (Es, Ne) else 0.0);   --  误差中位 → 均方根
   end Board_Rms;

   procedure Check_Fixed (G : in out Cam_Geo; Scene : Scene_Pt_Vectors.Vector; Now : Scene_Pt_Vectors.Vector; Best : in out Fixed_Best; Rep : out Fixed_Check;
                          Turn_Sd : Long_Float := 0.0; Base_Now : Integer := -1) is separate;

   function Tips_On_Plane (G : Cam_Geo; Views : Board_View_Vectors.Vector; P0, N : V3; Sd_Plane : Long_Float) return Plane_Tip_Vectors.Vector is
      R : Plane_Tip_Vectors.Vector;
   begin
      for Vw of Views loop
         declare
            O : constant V3 := Cam_Pos (G, Vw.Pose);
            Seen : Boolean;
            D : constant V3 := Ray (G, Vw.Pose, Vw.U, Vw.V, Seen);
            Dn : constant Long_Float := D (0) * N (0) + D (1) * N (1) + D (2) * N (2);
            H : constant Long_Float := (P0 (0) - O (0)) * N (0) + (P0 (1) - O (1)) * N (1) + (P0 (2) - O (2)) * N (2);
            T : Plane_Tip;
         begin
            if Seen and then Dn /= 0.0 and then H / Dn > 0.0 then   --  有视线(去得了畸变)、朝着面、交在眼前
               T.S := H / Dn;
               T.Sd := Sd_Plane / abs Dn;
               T.Pw := [O (0) + T.S * D (0), O (1) + T.S * D (1), O (2) + T.S * D (2)];
               T.Ok := True;
            end if;
            R.Append (T);
         end;
      end loop;
      return R;
   end Tips_On_Plane;

   function Press_Of (G : Cam_Geo; P : Plug.Arm_Pose; P0, N : V3) return Press_Eq is
      R : constant M3 := Cam_R (G, P);
      T : constant V3 := Cam_Pos (G, P);
      E : Press_Eq;
   begin
      E.A := Ap (Tr (R), N);
      E.B := (P0 (0) - T (0)) * N (0) + (P0 (1) - T (1)) * N (1) + (P0 (2) - T (2)) * N (2);
      return E;
   end Press_Of;

   function Fit_Presses (Eqs : Press_Eq_Vectors.Vector; Gate : Long_Float; View : Finger_View) return Press_Fit is separate;
   function Fit_On_Ray (Eqs : Press_Eq_Vectors.Vector; D : V3; Gate : Long_Float) return Press_Fit is separate;

   function Tilt_Dir (D : V3; Tilt, Azim : Long_Float) return V3 is
      function Dot (A, B : V3) return Long_Float is (A (0) * B (0) + A (1) * B (1) + A (2) * B (2));
      X : constant V3 := [1.0, 0.0, 0.0];
      Y : constant V3 := [0.0, 1.0, 0.0];
      Xp : V3 := [X (0) - Dot (X, D) * D (0), X (1) - Dot (X, D) * D (1), X (2) - Dot (X, D) * D (2)];
   begin
      if Norm (Xp) < 1.0e-9 then   --  数值保护:视线正好沿着眼的 x 轴(画面里看不到)⇒ 从 y 轴起量
         Xp := [Y (0) - Dot (Y, D) * D (0), Y (1) - Dot (Y, D) * D (1), Y (2) - Dot (Y, D) * D (2)];
      end if;
      declare
         L : constant Long_Float := Norm (Xp);
         E1 : constant V3 := [Xp (0) / L, Xp (1) / L, Xp (2) / L];
         E2 : constant V3 := [D (1) * E1 (2) - D (2) * E1 (1), D (2) * E1 (0) - D (0) * E1 (2), D (0) * E1 (1) - D (1) * E1 (0)];
         C : constant Long_Float := Cos (Tilt);
         S : constant Long_Float := Sin (Tilt);
      begin
         return [C * D (0) + S * (Cos (Azim) * E1 (0) + Sin (Azim) * E2 (0)),
                 C * D (1) + S * (Cos (Azim) * E1 (1) + Sin (Azim) * E2 (1)),
                 C * D (2) + S * (Cos (Azim) * E1 (2) + Sin (Azim) * E2 (2))];
      end;
   end Tilt_Dir;

   function Azim_Of (D, U : V3) return Long_Float is
      function Dot (A, B : V3) return Long_Float is (A (0) * B (0) + A (1) * B (1) + A (2) * B (2));
      X : constant V3 := [1.0, 0.0, 0.0];
      Y : constant V3 := [0.0, 1.0, 0.0];
      Xp : V3 := [X (0) - Dot (X, D) * D (0), X (1) - Dot (X, D) * D (1), X (2) - Dot (X, D) * D (2)];
   begin
      if Norm (Xp) < 1.0e-9 then   --  同 Tilt_Dir
         Xp := [Y (0) - Dot (Y, D) * D (0), Y (1) - Dot (Y, D) * D (1), Y (2) - Dot (Y, D) * D (2)];
      end if;
      declare
         L : constant Long_Float := Norm (Xp);
         E1 : constant V3 := [Xp (0) / L, Xp (1) / L, Xp (2) / L];
         E2 : constant V3 := [D (1) * E1 (2) - D (2) * E1 (1), D (2) * E1 (0) - D (0) * E1 (2), D (0) * E1 (1) - D (1) * E1 (0)];
      begin
         return Arctan (Dot (U, E2), Dot (U, E1));
      end;
   end Azim_Of;

   function Tilt_Angle (D : V3_Vectors.Vector; K : Natural; Single : Long_Float) return Long_Float is
      Beta : Long_Float := Long_Float'Last;
   begin
      for J in 0 .. Natural (D.Length) - 1 loop
         if J /= K then
            Beta := Long_Float'Min (Beta, Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, D (K) (0) * D (J) (0) + D (K) (1) * D (J) (1) + D (K) (2) * D (J) (2)))));
         end if;
      end loop;
      return (if Beta = Long_Float'Last then Single else (1.0 / 3.0) * Beta);
   end Tilt_Angle;

   function Ray_Owner (X : V3; D : V3_Vectors.Vector) return Natural is
      Best : Natural := Natural'Last;
      Bd : Long_Float := Long_Float'Last;
   begin
      for J in 0 .. Natural (D.Length) - 1 loop
         declare
            T : constant Long_Float := X (0) * D (J) (0) + X (1) * D (J) (1) + X (2) * D (J) (2);
            Off : constant Long_Float := Norm ([X (0) - T * D (J) (0), X (1) - T * D (J) (1), X (2) - T * D (J) (2)]);
         begin
            if T > 0.0 and then Off < Bd then
               Bd := Off; Best := J;
            end if;
         end;
      end loop;
      return Best;
   end Ray_Owner;

   function Turn_To (Fwd, Down : V3) return V3 is
      Cr : constant V3 := [Fwd (1) * Down (2) - Fwd (2) * Down (1), Fwd (2) * Down (0) - Fwd (0) * Down (2), Fwd (0) * Down (1) - Fwd (1) * Down (0)];
      Sn : constant Long_Float := Norm (Cr);
      Ang : constant Long_Float := Arctan (Sn, Fwd (0) * Down (0) + Fwd (1) * Down (1) + Fwd (2) * Down (2));
      --  正好反向(叉积为零)时随便取一根和它垂直的轴(同 Geo_Turn)
      Az : constant V3 := [Fwd (1), -Fwd (0), 0.0];
      Ax : constant V3 := [0.0, Fwd (2), -Fwd (1)];
      Alt : constant V3 := (if Norm (Az) >= Norm (Ax) then Az else Ax);
      Aln : constant Long_Float := Norm (Alt);
      Axis : constant V3 := (if Sn > 1.0e-9 then [Cr (0) / Sn, Cr (1) / Sn, Cr (2) / Sn]
                             elsif Aln > 1.0e-9 then [Alt (0) / Aln, Alt (1) / Aln, Alt (2) / Aln] else [0.0, 0.0, 1.0]);
   begin
      return [Axis (0) * Ang, Axis (1) * Ang, Axis (2) * Ang];
   end Turn_To;

   --  几条视线的信息矩阵 A = Σ (I − d dᵀ) / (σ_I t_I)²(协方差 = A⁻¹),先按 A 的迹缩到 1 附近(Solve3 的奇异门是绝对数):An = A / Sc。
   --  Sds 的条数和视线对不上、不到两条、有哪条是 0、交点在某只眼背后 ⇒ Ok = False
   procedure Meet_Info (Rays : Sight_Vectors.Vector; Sds : Bytes.Floats; P : V3; An : out M3; Sc : out Long_Float; Ok : out Boolean) is
      A : M3 := [others => [others => 0.0]];
   begin
      An := A; Sc := 0.0; Ok := False;
      if Natural (Sds.Length) /= Natural (Rays.Length) or else Natural (Rays.Length) < 2 then
         return;
      end if;
      for K in 0 .. Natural (Rays.Length) - 1 loop
         declare
            R : constant Sight := Rays (K);
            T : constant Long_Float := (P (0) - R.O (0)) * R.D (0) + (P (1) - R.O (1)) * R.D (1) + (P (2) - R.O (2)) * R.D (2);
            S : constant Long_Float := Sds (K) * T;   --  这条视线在交点处垂直方向的位置噪声
         begin
            if Sds (K) <= 0.0 or else T <= 0.0 then
               return;
            end if;
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  A (I, J) := A (I, J) + ((if I = J then 1.0 else 0.0) - R.D (I) * R.D (J)) / (S * S);
               end loop;
            end loop;
         end;
      end loop;
      Sc := (A (0, 0) + A (1, 1) + A (2, 2)) / 3.0;
      if Sc <= 0.0 then
         return;
      end if;
      for I in 0 .. 2 loop
         for J in 0 .. 2 loop
            An (I, J) := A (I, J) / Sc;
         end loop;
      end loop;
      Ok := True;
   end Meet_Info;

   function Meet_Sd (Rays : Sight_Vectors.Vector; Sds : Bytes.Floats; P, U : V3) return Long_Float is
      An : M3;
      Sc : Long_Float;
      Ok : Boolean;
      --  沿 U 的方差 = Uᵀ A⁻¹ U = U · X,X 解 A X = U(奇异 ⇒ Solve3 交零向量 ⇒ 这个方向量不出)
      Un : constant Long_Float := Norm (U);
      Uu : constant V3 := (if Un > 0.0 then [U (0) / Un, U (1) / Un, U (2) / Un] else U);
      X : V3;
      Var : Long_Float;
   begin
      Meet_Info (Rays, Sds, P, An, Sc, Ok);
      if not Ok then
         return Long_Float'Last;
      end if;
      X := Solve3 (An, Uu);
      Var := (Uu (0) * X (0) + Uu (1) * X (1) + Uu (2) * X (2)) / Sc;
      if Un <= 0.0 or else Var <= 0.0 then
         return Long_Float'Last;
      end if;
      return Sqrt (Var);
   end Meet_Sd;

   function Meet_Cov (Rays : Sight_Vectors.Vector; Sds : Bytes.Floats; P : V3; Ok : out Boolean) return M3 is
      An : M3;
      Sc : Long_Float;
      Cv : M3 := [others => [others => 0.0]];
   begin
      Meet_Info (Rays, Sds, P, An, Sc, Ok);
      if not Ok then
         return Cv;
      end if;
      --  A⁻¹ 按列解:A x = e_j(奇异 ⇒ Solve3 交零向量 ⇒ 那一列是零、对角线不正 ⇒ 量不出)
      for J in 0 .. 2 loop
         declare
            E : V3 := [0.0, 0.0, 0.0];
            X : V3;
         begin
            E (J) := 1.0;
            X := Solve3 (An, E);
            for I in 0 .. 2 loop
               Cv (I, J) := X (I) / Sc;
            end loop;
         end;
      end loop;
      Ok := Cv (0, 0) > 0.0 and then Cv (1, 1) > 0.0 and then Cv (2, 2) > 0.0;
      return Cv;
   end Meet_Cov;

   function Meet (Rays : Sight_Vectors.Vector; Ok : out Boolean; Spread : out Long_Float) return V3 is separate;

   --  ── 存 / 读(自己的小文件,一行一台相机,坏一行不毁整份)──
   procedure Save (Path : String; Gs : Geo_Vectors.Vector) is separate;

   procedure Load (Path : String; Gs : in out Geo_Vectors.Vector; N_Cams : Natural; Note : out String) is separate;
end Geom;
