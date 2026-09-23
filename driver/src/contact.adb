with Ada.Numerics; use Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
package body Contact is

   function Nat_Img (N : Natural) return String is (Ada.Strings.Fixed.Trim (Natural'Image (N), Ada.Strings.Left));

   function Dot (A, B : V3) return Long_Float is (A (0) * B (0) + A (1) * B (1) + A (2) * B (2));

   function Cross (A, B : V3) return V3 is
     ([A (1) * B (2) - A (2) * B (1), A (2) * B (0) - A (0) * B (2), A (0) * B (1) - A (1) * B (0)]);

   function Unit (A : V3; Ok : out Boolean) return V3 is
      N : constant Long_Float := Norm (A);
   begin
      if N'Valid and then N > 1.0e-12 then
         Ok := True;
         return [A (0) / N, A (1) / N, A (2) / N];
      end if;
      Ok := False;
      return [others => 0.0];
   end Unit;

   function Is_Dir (A : V3) return Boolean is
      N : constant Long_Float := Norm (A);
   begin
      return N'Valid and then N > 1.0e-12;
   end Is_Dir;

   function Admits (K : Cone; Dir : V3) return Boolean is
      Oa, Od : Boolean;
      A : constant V3 := Unit (K.Axis, Oa);
      D : constant V3 := Unit (Dir, Od);
   begin
      if not (Oa and Od) then
         return False;
      end if;
      return Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, Dot (A, D)))) <= K.Half_Angle + 1.0e-9;
   end Admits;

   function Still (Pivot : V3) return Twist is (Lin => [others => 0.0], Ang => [others => 0.0], Pivot => Pivot);
   function Slide (Lin : V3) return Twist is (Lin => Lin, Ang => [others => 0.0], Pivot => [others => 0.0]);

   function Turn (Axis : V3; Rad : Long_Float; Pivot : V3; Ok : out Boolean) return Twist is
      A : constant V3 := Unit (Axis, Ok);
   begin
      return (Lin => [others => 0.0], Ang => [A (0) * Rad, A (1) * Rad, A (2) * Rad], Pivot => Pivot);
   end Turn;

   function Angle (T : Twist) return Long_Float is (Norm (T.Ang));
   function Moving (T : Twist) return Boolean is (Norm (T.Lin) > 1.0e-9 or else Angle (T) > 1.0e-9);

   function Apply (T : Twist; P : V3) return V3 is
      Th : constant Long_Float := Angle (T);
      R : V3 := P;
   begin
      if Th >= 1.0e-12 then
         declare
            K : constant V3 := [T.Ang (0) / Th, T.Ang (1) / Th, T.Ang (2) / Th];
            Q : constant V3 := [P (0) - T.Pivot (0), P (1) - T.Pivot (1), P (2) - T.Pivot (2)];
            C : constant Long_Float := Cos (Th);
            Sn : constant Long_Float := Sin (Th);
            Kq : constant V3 := Cross (K, Q);
            Kd : constant Long_Float := Dot (K, Q);
         begin
            for I in 0 .. 2 loop
               R (I) := T.Pivot (I) + Q (I) * C + Kq (I) * Sn + K (I) * Kd * (1.0 - C);
            end loop;
         end;
      end if;
      return [R (0) + T.Lin (0), R (1) + T.Lin (1), R (2) + T.Lin (2)];
   end Apply;

   function Img (G : Gap) return String is
   begin
      case G.Kind is
         when Fine => return "fine";
         when No_Points => return "NoPoints";
         when Bad_Normal => return "BadNormal(" & Nat_Img (G.Index) & ")";
         when Bad_Cone => return "BadCone(" & Nat_Img (G.Index) & ")";
         when Cannot_Drive => return "CannotDrive";
         when Motion_Still => return "MotionStill";
         when No_Pivot => return "NoPivot";
         when Bad_Tolerance => return "BadTolerance(" & Nat_Img (G.Index) & ")";
      end case;
   end Img;

   function Check (S : Set; Must_Move : Boolean) return Gap is
   begin
      if S.Points.Is_Empty then
         return (No_Points, 0);
      end if;
      for I in 0 .. Natural (S.Points.Length) - 1 loop
         declare
            P : constant Point := S.Points (I);
         begin
            if not Is_Dir (P.Normal) then
               return (Bad_Normal, I);
            end if;
            if not Is_Dir (P.Push.Axis) or else not P.Push.Half_Angle'Valid
              or else P.Push.Half_Angle < 0.0 or else P.Push.Half_Angle > Pi
            then
               return (Bad_Cone, I);
            end if;
            if not P.Tol_M'Valid or else P.Tol_M <= 0.0 then
               return (Bad_Tolerance, I);
            end if;
         end;
      end loop;
      if Must_Move and then not Moving (S.Motion) then
         return (Motion_Still, 0);
      end if;
      if Angle (S.Motion) > 1.0e-9 then
         --  绕轴转必须说清绕哪一点。这里只查它是不是一个数;"填得对不对"是几何层的事
         for I in 0 .. 2 loop
            if not S.Motion.Pivot (I)'Valid then
               return (No_Pivot, 0);
            end if;
         end loop;
      end if;
      --  自洽性:所有接触【加在一起】,能不能产生③要的那个力旋量
      if Moving (S.Motion) and then not Can_Drive (S) then
         return (Cannot_Drive, 0);
      end if;
      return (Fine, 0);
   end Check;

   type W6 is array (0 .. 5) of Long_Float;
   package W6_Vectors is new Ada.Containers.Vectors (Natural, W6);

   --  Wd 在不在 Gen 张成的凸锥里 —— 非负最小二乘,加速投影梯度(FISTA),零依赖。
   function In_Cone (Gen_In : W6_Vectors.Vector; Wd : W6) return Boolean is
      Gen : W6_Vectors.Vector;
   begin
      --  先把每条棱归一化:凸锥对每条棱各自的正倍数不变,答案不变;但不做就会算错 ——
      --  力矩那三列除以特征长度,L 小到 1.2 cm 时棱的模差 ~80 倍,步长被最大的那条锁死,几千步收敛不到
      --  ⇒ 一条可行的接触集被判成不可行(实测:两指捏着勺子往下插,明明做得到,被判 CannotDrive)。
      for G of Gen_In loop
         declare
            M : Long_Float := 0.0;
         begin
            for D in 0 .. 5 loop
               M := M + G (D) * G (D);
            end loop;
            M := Sqrt (M);
            if M > 1.0e-12 then
               Gen.Append (W6'([G (0) / M, G (1) / M, G (2) / M, G (3) / M, G (4) / M, G (5) / M]));
            end if;
         end;
      end loop;
      if Gen.Is_Empty then
         return False;
      end if;
      declare
         N : constant Natural := Natural (Gen.Length);
         type Coefs is array (0 .. N - 1) of Long_Float;
         A : Coefs := [others => 0.0];
         Y : Coefs := [others => 0.0];
         Nx : Coefs := [others => 0.0];
         Lip : Long_Float := 0.0;
         Step : Long_Float;
         T : Long_Float := 1.0;
         Last : Long_Float := Long_Float'Last;
         Res : Long_Float;
         R : W6;
         --  残差 = Σ a_j g_j − wd,和它的模
         procedure Residual (Cf : Coefs; R : out W6; Rn : out Long_Float) is
         begin
            R := [others => 0.0];
            for J in Cf'Range loop
               for D in 0 .. 5 loop
                  R (D) := R (D) + Cf (J) * Gen (J) (D);
               end loop;
            end loop;
            Rn := 0.0;
            for D in 0 .. 5 loop
               R (D) := R (D) - Wd (D);
               Rn := Rn + R (D) * R (D);
            end loop;
            Rn := Sqrt (Rn);
         end Residual;
      begin
         --  步长取 1/L(L = 最大特征值上界,用 Frobenius 范数代替,保守但稳)
         for G of Gen loop
            for D in 0 .. 5 loop
               Lip := Lip + G (D) * G (D);
            end loop;
         end loop;
         Step := (if Lip > 1.0e-12 then 1.0 / Lip else 1.0);
         --  加速(FISTA)+ 一条真的停机判据。上一版定步长跑满 4000 步就下结论,会在还没算完的时候回答"做不到" ——
         --  而"做不到"是这条链上最重的一句话(2026-08-16 实测:薄板的三指/五指绕 x/y 转在 4000 步下判死,放到 20 万步全部可行:
         --  那几格根本不是物理上做不到,是判据没算完就下了结论,而且它长得和真结论一模一样)。
         --  停机看"还在不在进步",不看跑了多少步;进步停了才允许说"做不到"。步数上限 20000 只是保险(次数,无量纲)。
         for It in 1 .. 20000 loop
            Residual (Y, R, Res);
            for J in 0 .. N - 1 loop
               declare
                  Grad : Long_Float := 0.0;
               begin
                  for D in 0 .. 5 loop
                     Grad := Grad + Gen (J) (D) * R (D);
                  end loop;
                  Nx (J) := Long_Float'Max (0.0, Y (J) - Step * Grad);
               end;
            end loop;
            declare
               T2 : constant Long_Float := 0.5 * (1.0 + Sqrt (1.0 + 4.0 * T * T));
               W : constant Long_Float := (T - 1.0) / T2;
            begin
               for J in 0 .. N - 1 loop
                  Y (J) := Nx (J) + W * (Nx (J) - A (J));
               end loop;
               A := Nx;
               T := T2;
            end;
            Residual (A, R, Res);
            --  残差小于 1e-7(无量纲:棱都归一化过,Wd 也是单位向量)⇒ 在锥里
            if Res < 1.0e-7 then
               return True;
            end if;
            --  一步的进步不到上一次残差的 1e-14 倍(无量纲的相对量)⇒ 掉不动了
            if abs (Last - Res) < 1.0e-14 * Long_Float'Max (Last, 1.0) then
               exit;
            end if;
            Last := Res;
         end loop;
         Residual (A, R, Res);
         return Res < 1.0e-4;
      end;
   end In_Cone;

   function Can_Drive (S : Set) return Boolean is
      Np : constant Natural := Natural (S.Points.Length);
      Kf : constant Long_Float := Long_Float (Np);
      Ref : V3 := [others => 0.0];
      L : Long_Float := 0.0;
      Wd : W6;
      Nd : Long_Float := 0.0;
      Gen : W6_Vectors.Vector;
      --  锥上取 8 条棱 + 轴心(采样次数,无量纲),各生成一条力旋量
      Edges : constant := 8;
      procedure Add (F : V3; R : V3) is
         Tq : constant V3 := Cross (R, F);
      begin
         Gen.Append (W6'([F (0), F (1), F (2), Tq (0) / L, Tq (1) / L, Tq (2) / L]));
      end Add;
      procedure Add_Torque (Ax : V3) is
      begin
         Gen.Append (W6'([0.0, 0.0, 0.0, Ax (0) / L, Ax (1) / L, Ax (2) / L]));
         Gen.Append (W6'([0.0, 0.0, 0.0, -Ax (0) / L, -Ax (1) / L, -Ax (2) / L]));
      end Add_Torque;
   begin
      if Np = 0 then
         return False;
      end if;
      --  参考点取接触点质心 —— 物体质心的代理(质心是世界属性,要学;几何能给的只有这个)。
      --  不是第③格的 Pivot:Pivot 说的是"转轴在哪",不是"反力作用在哪";拿 Pivot 当参考点,"绕一个偏在下面的支点转"会被写成纯力矩的需求,
      --  两指对夹根本给不出(实测:握着绕下方 10 cm 的支点转被判 CannotDrive,而这件事天天在做)。
      for P of S.Points loop
         for I in 0 .. 2 loop
            Ref (I) := Ref (I) + P.Pos (I) / Kf;
         end loop;
      end loop;
      --  特征长度 L = 各接触到参考点的平均距离:力与力矩单位不同,用它配平。它是这个问题里真实存在的长度,不是调出来的常数
      for P of S.Points loop
         L := L + Norm ([P.Pos (0) - Ref (0), P.Pos (1) - Ref (1), P.Pos (2) - Ref (2)]) / Kf;
      end loop;
      if L <= 1.0e-9 then
         L := 1.0;
      end if;
      --  想要的力旋量方向:力 ∝ 参考点自己的速度(平移 + 绕支点转带出来的那一份),力矩 ∝ 角速度。
      --  绕支点转会把质心也甩出去 —— 那一份必须算进来,不算就是要求纯力矩。
      declare
         M : constant Twist := S.Motion;
         Rp : constant V3 := [Ref (0) - M.Pivot (0), Ref (1) - M.Pivot (1), Ref (2) - M.Pivot (2)];
         Vs : constant V3 := Cross (M.Ang, Rp);
      begin
         Wd := [M.Lin (0) + Vs (0), M.Lin (1) + Vs (1), M.Lin (2) + Vs (2), M.Ang (0) * L, M.Ang (1) * L, M.Ang (2) * L];
      end;
      for D in 0 .. 5 loop
         Nd := Nd + Wd (D) * Wd (D);
      end loop;
      Nd := Sqrt (Nd);
      if Nd < 1.0e-12 then
         return True;
      end if;
      for D in 0 .. 5 loop
         Wd (D) := Wd (D) / Nd;
      end loop;
      for P of S.Points loop
         declare
            Ok1, Ok2 : Boolean;
            N : constant V3 := Unit (P.Push.Axis, Ok1);
            --  挑一条不和锥轴平行的种子轴(0.9 是无量纲的比较:沿哪个坐标轴的分量小于它,就拿那个坐标轴当种子)
            Seed : constant V3 := (if abs N (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);
            T1 : constant V3 := Unit (Cross (N, Seed), Ok2);
            T2 : constant V3 := Cross (N, T1);
            C : constant Long_Float := Cos (P.Push.Half_Angle);
            Sn : constant Long_Float := Sin (P.Push.Half_Angle);
            R : constant V3 := [P.Pos (0) - Ref (0), P.Pos (1) - Ref (1), P.Pos (2) - Ref (2)];
         begin
            if not Ok1 or else not Ok2 then
               return False;
            end if;
            for K in 0 .. Edges loop
               declare
                  Phi : constant Long_Float := 2.0 * Pi * Long_Float (K) / Long_Float (Edges);
                  Cp : constant Long_Float := Cos (Phi);
                  Sp : constant Long_Float := Sin (Phi);
                  F : V3;
               begin
                  if K = Edges then
                     F := N;
                  else
                     for I in 0 .. 2 loop
                        F (I) := C * N (I) + Sn * (Cp * T1 (I) + Sp * T2 (I));
                     end loop;
                  end if;
                  Add (F, R);
                  --  能拉的接触(真空/磁/胶):沿法向反着也使得上劲。把锥镜像过来。
                  if P.Pull then
                     if K = Edges then
                        Add ([-N (0), -N (1), -N (2)], R);
                     else
                        for I in 0 .. 2 loop
                           F (I) := -C * N (I) + Sn * (Cp * T1 (I) + Sp * T2 (I));
                        end loop;
                        Add (F, R);
                     end if;
                  end if;
               end;
            end loop;
            --  抗剥离的接触(吸盘/胶垫):还能绕切向两根轴传力矩,四个方向都给
            if P.Peel then
               Add_Torque (T1);
               Add_Torque (T2);
            end if;
            --  面接触:还能绕自己的法向拧,两个方向都给 —— 扭矩可正可负
            if P.Torsion then
               Add_Torque (N);
            end if;
         end;
      end loop;
      return In_Cone (Gen, Wd);
   end Can_Drive;

   --  ── 一串 · 并存 · 过渡 ──

   function One_Of (S : Set) return Move is
      M : Move;
      N : Node;
   begin
      N.Kind := One;
      N.S := S;
      M.Nodes.Append (N);
      M.Root := 0;
      return M;
   end One_Of;

   function Chain (Kind : Move_Kind; Items : Move_Vectors.Vector) return Move is
      M : Move;
      Parent : Node;
   begin
      Parent.Kind := Kind;
      for It of Items loop
         declare
            Off : constant Natural := Natural (M.Nodes.Length);
         begin
            for N of It.Nodes loop
               declare
                  N2 : Node := N;
               begin
                  for J in 0 .. Natural (N2.Items.Length) - 1 loop
                     declare
                        Cj : constant Natural := N2.Items (J);
                     begin
                        N2.Items.Replace_Element (J, Cj + Off);
                     end;
                  end loop;
                  M.Nodes.Append (N2);
               end;
            end loop;
            Parent.Items.Append (It.Root + Off);
         end;
      end loop;
      M.Nodes.Append (Parent);
      M.Root := Natural (M.Nodes.Length) - 1;
      return M;
   end Chain;

   function Clear_Of (Keep_Out : V3_Vectors.Vector; By_M : Long_Float; From : V3_Vectors.Vector) return Move is
      M : Move;
      N : Node;
   begin
      N.Kind := Clear;
      N.Keep_Out := Keep_Out;
      N.By_M := By_M;
      N.From := From;
      M.Nodes.Append (N);
      M.Root := 0;
      return M;
   end Clear_Of;

   --  这个接触集里【手】那几个点在哪(世界接触不算 —— 手够不到桌子底下那条边)
   function Hand_At (S : Set) return V3_Vectors.Vector is
      V : V3_Vectors.Vector;
   begin
      for P of S.Points loop
         if P.By.Kind = Hand then
            V.Append (P.Pos);
         end if;
      end loop;
      return V;
   end Hand_At;

   function Moves_At (M : Move; Id : Natural) return Boolean is
      N : constant Node := M.Nodes (Id);
   begin
      case N.Kind is
         when One =>
            return Moving (N.S.Motion);
         when In_Turn | Keep | Meanwhile =>
            for C of N.Items loop
               if Moves_At (M, C) then
                  return True;
               end if;
            end loop;
            return False;
         when Clear =>
            return False;   --  躲开是手在动,不是物体在动
      end case;
   end Moves_At;

   function Moves (M : Move) return Boolean is (Moves_At (M, M.Root));

   function Start_At (M : Move; Id : Natural) return V3_Vectors.Vector is
      N : constant Node := M.Nodes (Id);
   begin
      case N.Kind is
         when One =>
            return Hand_At (N.S);
         when In_Turn | Keep | Meanwhile =>
            if N.Items.Is_Empty then
               return V3_Vectors.Empty_Vector;
            end if;
            return Start_At (M, N.Items.First_Element);
         when Clear =>
            return N.From;
      end case;
   end Start_At;

   function End_At (M : Move; Id : Natural) return V3_Vectors.Vector is
      N : constant Node := M.Nodes (Id);
   begin
      case N.Kind is
         when One =>
            declare
               V : V3_Vectors.Vector := Hand_At (N.S);
            begin
               for J in 0 .. Natural (V.Length) - 1 loop
                  declare
                     Pj : constant V3 := V (J);
                  begin
                     V.Replace_Element (J, Apply (N.S.Motion, Pj));
                  end;
               end loop;
               return V;
            end;
         when In_Turn | Keep =>
            if N.Items.Is_Empty then
               return V3_Vectors.Empty_Vector;
            end if;
            return End_At (M, N.Items.Last_Element);
         when Clear =>
            return N.From;   --  躲完手在哪由执行层算;这一层只说"别碰那儿"
         when Meanwhile =>
            if N.Items.Is_Empty then
               return V3_Vectors.Empty_Vector;
            end if;
            return End_At (M, N.Items.First_Element);   --  并存:末了停在维持的那一段上
      end case;
   end End_At;

   function Start_Points (M : Move) return V3_Vectors.Vector is (Start_At (M, M.Root));
   function End_Points (M : Move) return V3_Vectors.Vector is (End_At (M, M.Root));

   procedure Flatten_At (M : Move; Id : Natural; Into : in out Nat_Vectors.Vector) is
      N : constant Node := M.Nodes (Id);
   begin
      case N.Kind is
         when One =>
            Into.Append (Id);
         when In_Turn | Keep | Meanwhile =>
            for C of N.Items loop
               Flatten_At (M, C, Into);
            end loop;
         when Clear =>
            null;   --  零接触点 ⇒ 摊平出来什么都没有,而那正是它的定义
      end case;
   end Flatten_At;

   function Flatten (M : Move) return Nat_Vectors.Vector is
      V : Nat_Vectors.Vector;
   begin
      Flatten_At (M, M.Root, V);
      return V;
   end Flatten;

   function Walk (M : Move; Id : Natural; Must_Move : Boolean; Path : in out Nat_Vectors.Vector) return Many_Gap is
      N : constant Node := M.Nodes (Id);
      function Here (K : Many_Kind; Seg : Natural := 0; Off : Long_Float := 0.0; G : Gap := (Fine, 0)) return Many_Gap is
        ((Kind => K, Path => Path, G => G, Seg => Seg, Off_M => Off));
      R : Many_Gap;
   begin
      case N.Kind is
         when One =>
            declare
               G : constant Gap := Check (N.S, Must_Move);
            begin
               return (if G.Kind = Fine then Here (Fine) else Here (Inside, G => G));
            end;
         when In_Turn | Keep =>
            if N.Items.Is_Empty then
               return Here (Empty);
            end if;
            for I in 0 .. Natural (N.Items.Length) - 1 loop
               Path.Append (I);
               R := Walk (M, N.Items (I), Must_Move, Path);
               Path.Delete_Last;
               if R.Kind /= Fine then
                  return R;
               end if;
            end loop;
            if N.Kind = Keep then
               --  接续条件:说了不松手,下一段就必须从上一段末了那些点接着走
               for I in 1 .. Natural (N.Items.Length) - 1 loop
                  declare
                     Prev : constant V3_Vectors.Vector := End_At (M, N.Items (I - 1));
                     Next : constant V3_Vectors.Vector := Start_At (M, N.Items (I));
                     Worst : Long_Float := 0.0;
                     Tol : Long_Float := Long_Float'Last;
                     Ids : Nat_Vectors.Vector;
                  begin
                     if Natural (Prev.Length) /= Natural (Next.Length) then
                        return Here (Keep_Changes_Point_Count, Seg => I);
                     end if;
                     for J in 0 .. Natural (Prev.Length) - 1 loop
                        declare
                           A : constant V3 := Prev (J);
                           B : constant V3 := Next (J);
                        begin
                           Worst := Long_Float'Max (Worst, Norm ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]));
                        end;
                     end loop;
                     --  门槛用下一段自己声明的最严容差 —— 不另拍一个常数
                     Flatten_At (M, N.Items (I), Ids);
                     for Sid of Ids loop
                        for P of M.Nodes (Sid).S.Points loop
                           Tol := Long_Float'Min (Tol, P.Tol_M);
                        end loop;
                     end loop;
                     if Worst > Tol then
                        return Here (Keep_Breaks_Contact, Seg => I, Off => Worst);
                     end if;
                  end;
               end loop;
            end if;
            return Here (Fine);
         when Clear =>
            if N.Keep_Out.Is_Empty then
               return Here (No_Keep_Out);
            end if;
            if not N.By_M'Valid or else N.By_M <= 0.0 or else N.From.Is_Empty then
               return Here (Bad_Clearance);
            end if;
            return Here (Fine);
         when Meanwhile =>
            if N.Items.Is_Empty then
               return Here (Empty);
            end if;
            if Natural (N.Items.Length) = 1 then
               return Here (Nothing_To_Pair_With);
            end if;
            for I in 0 .. Natural (N.Items.Length) - 1 loop
               Path.Append (I);
               --  第一段是维持的那一个:它必须不动
               if I = 0 and then Moves_At (M, N.Items (0)) then
                  R := Here (Holder_Moves);
               else
                  R := Walk (M, N.Items (I), (if I = 0 then False else Must_Move), Path);
               end if;
               Path.Delete_Last;
               if R.Kind /= Fine then
                  return R;
               end if;
            end loop;
            return Here (Fine);
      end case;
   end Walk;

   function Check (M : Move; Must_Move : Boolean) return Many_Gap is
      Path : Nat_Vectors.Vector;
   begin
      return Walk (M, M.Root, Must_Move, Path);
   end Check;

   function Path_Img (P : Nat_Vectors.Vector) return String is
      U : Unbounded_String;
   begin
      for I of P loop
         if Length (U) > 0 then
            Append (U, "/");
         end if;
         Append (U, Nat_Img (I));
      end loop;
      return "[" & To_String (U) & "]";
   end Path_Img;

   function Img (M : Many_Gap) return String is
   begin
      case M.Kind is
         when Fine => return "fine";
         when Inside => return "At" & Path_Img (M.Path) & " " & Img (M.G);
         when Empty => return "Empty" & Path_Img (M.Path);
         when Holder_Moves => return "HolderMoves" & Path_Img (M.Path);
         when Nothing_To_Pair_With => return "NothingToPairWith" & Path_Img (M.Path);
         when Keep_Breaks_Contact => return "KeepBreaksContact" & Path_Img (M.Path) & " seg " & Nat_Img (M.Seg) & " off" & Long_Float'Image (M.Off_M);
         when Keep_Changes_Point_Count => return "KeepChangesPointCount" & Path_Img (M.Path) & " seg " & Nat_Img (M.Seg);
         when No_Keep_Out => return "NoKeepOut" & Path_Img (M.Path);
         when Bad_Clearance => return "BadClearance" & Path_Img (M.Path);
      end case;
   end Img;

end Contact;
