with Ada.Numerics; use Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Containers.Ordered_Maps;
with Ada.Containers.Generic_Array_Sort;
package body Contact.Wrench is

   --  摩擦锥线性化的棱数(次数):第一条对准 Align 在接触面上的那一份,那个方向上是准的,别的方向内接、偏保守
   Edges : constant := 8;
   --  算作零(无量纲:和它相比的量都按自己的量级归过一)
   Tiny : constant := 1.0e-12;

   function Sub (A, B : V3) return V3 is ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]);
   function Add (A, B : V3) return V3 is ([A (0) + B (0), A (1) + B (1), A (2) + B (2)]);
   function Scl (K : Long_Float; A : V3) return V3 is ([K * A (0), K * A (1), K * A (2)]);

   procedure Min_Sum (A : Real_Array; Rows, Cols : Positive; B : Real_Array; Obj : out Long_Float; Ok : out Boolean; Cost : Real_Array := No_Cost) is
      --  表:第 0 行 = 检验数,第 1 .. Rows 行 = 约束;列 0 .. Cols − 1 = 变量,Cols .. Cols + Rows − 1 = 人工变量,最后一列 = 右端
      Nc : constant Natural := Cols + Rows;
      type Tab is array (0 .. Rows, 0 .. Nc) of Long_Float;
      T : Tab := [others => [others => 0.0]];
      Bas : array (1 .. Rows) of Natural;
      Eps : constant := 1.0e-10;          --  算作零(无量纲:行都按量级归一)
      Max_Pivots : constant Natural := 50 * (Rows + Cols);   --  防万一的步数上限(次数;Bland 规则本身不会循环)
      Scale : Long_Float := 0.0;
      --  第 J 个变量的成本(没给成本 = 每个都算 1:求各棱上的力总和最小)
      function C_Of (J : Natural) return Long_Float is (if Cost'Length = 0 then 1.0 else Cost (Cost'First + J));
      procedure Pivot (R, E : Natural) is
         P : constant Long_Float := T (R, E);
      begin
         for J in 0 .. Nc loop
            T (R, J) := T (R, J) / P;
         end loop;
         for I in 0 .. Rows loop
            if I /= R and then T (I, E) /= 0.0 then
               declare
                  F : constant Long_Float := T (I, E);
               begin
                  for J in 0 .. Nc loop
                     T (I, J) := T (I, J) - F * T (R, J);
                  end loop;
               end;
            end if;
         end loop;
         Bas (R) := E;
      end Pivot;
      --  按当前检验数一直换基,直到没有能让目标变小的列(Bland:最小下标的负检验数进、比值最小里基下标最小的出)。Allow = 能进基的列的上界
      procedure Run (Allow : Natural; Done : out Boolean) is
         N_Piv : Natural := 0;
      begin
         Done := True;
         loop
            declare
               E : Integer := -1;
               R : Integer := -1;
               Best : Long_Float := Long_Float'Last;
            begin
               for J in 0 .. Allow - 1 loop
                  if T (0, J) < -Eps then
                     E := J;
                     exit;
                  end if;
               end loop;
               exit when E < 0;
               for I in 1 .. Rows loop
                  if T (I, E) > Eps then
                     declare
                        Q : constant Long_Float := T (I, Nc) / T (I, E);
                     begin
                        if Q < Best - Eps or else (abs (Q - Best) <= Eps and then R > 0 and then Bas (I) < Bas (R)) then
                           Best := Q; R := I;
                        end if;
                     end;
                  end if;
               end loop;
               if R < 0 then
                  Done := False;   --  无界(成本都 ≥ 0,目标有下界 0,不该出现)
                  return;
               end if;
               Pivot (R, E);
               N_Piv := N_Piv + 1;
               if N_Piv > Max_Pivots then
                  Done := False;
                  return;
               end if;
            end;
         end loop;
      end Run;
      Done : Boolean;
   begin
      Obj := No_Way; Ok := False;
      if Cost'Length /= 0 and then Cost'Length /= Cols then
         return;   --  成本和变量数对不上
      end if;
      --  每一行按它自己的量级归一(不改解,只让门槛 Eps 对每一行都一样有意义)
      for I in 0 .. Rows - 1 loop
         declare
            M : Long_Float := abs B (B'First + I);
         begin
            for J in 0 .. Cols - 1 loop
               M := Long_Float'Max (M, abs A (A'First + I * Cols + J));
            end loop;
            if M = 0.0 then
               M := 1.0;
            end if;
            declare
               Sg : constant Long_Float := (if B (B'First + I) < 0.0 then -1.0 else 1.0);   --  右端翻成非负
            begin
               for J in 0 .. Cols - 1 loop
                  T (I + 1, J) := Sg * A (A'First + I * Cols + J) / M;
               end loop;
               T (I + 1, Cols + I) := 1.0;
               T (I + 1, Nc) := Sg * B (B'First + I) / M;
            end;
         end;
         Bas (I + 1) := Cols + I;
      end loop;
      --  第一阶段:最小化人工变量之和
      for J in 0 .. Nc loop
         T (0, J) := 0.0;
      end loop;
      for I in 1 .. Rows loop
         for J in 0 .. Cols - 1 loop
            T (0, J) := T (0, J) - T (I, J);
         end loop;
         T (0, Nc) := T (0, Nc) - T (I, Nc);
         Scale := Scale + T (I, Nc);
      end loop;
      Run (Nc, Done);
      if not Done or else -T (0, Nc) > 1.0e-9 * Long_Float'Max (1.0, Scale) then
         return;   --  做不到
      end if;
      --  还在基里的人工变量(值为零)换出去;换不出去的那一行是多余的,留着(它在变量列上全是零)
      for I in 1 .. Rows loop
         if Bas (I) >= Cols then
            for J in 0 .. Cols - 1 loop
               if abs T (I, J) > Eps then
                  Pivot (I, J);
                  exit;
               end if;
            end loop;
         end if;
      end loop;
      --  第二阶段:目标 = Σ 成本 · x(人工变量成本 0、不许再进基)
      for J in 0 .. Nc loop
         T (0, J) := (if J < Cols then C_Of (J) else 0.0);
      end loop;
      T (0, Nc) := 0.0;
      for I in 1 .. Rows loop
         if Bas (I) < Cols and then C_Of (Bas (I)) /= 0.0 then
            declare
               Cb : constant Long_Float := C_Of (Bas (I));
            begin
               for J in 0 .. Nc loop
                  T (0, J) := T (0, J) - Cb * T (I, J);
               end loop;
            end;
         end if;
      end loop;
      Run (Cols, Done);
      if not Done then
         return;
      end if;
      Obj := -T (0, Nc);
      Ok := True;
   end Min_Sum;

   --  面内的一组底(E1, E2 ⊥ U)。挑一条不和 U 平行的种子轴:x、y 里离 U 更远的那根(纯几何)
   procedure Plane_Basis (U : V3; E1, E2 : out V3) is
      Ax : constant V3 := (if abs U (0) <= abs U (1) then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);
      Ok : Boolean;
      pragma Warnings (Off, Ok);   --  U 是单位向量、Ax 是离它更远的那根坐标轴 ⇒ 叉乘不会是零
   begin
      E1 := Unit (Cross (U, Ax), Ok);
      E2 := Cross (U, E1);
   end Plane_Basis;

   function Footprint (Pts : V3_Vectors.Vector; P0, Up : V3; Pitch : Long_Float) return Surface is
      type Cell is record
         I, J : Integer;
      end record;
      function "<" (A, B : Cell) return Boolean is (A.I < B.I or else (A.I = B.I and then A.J < B.J));
      type Acc is record
         S : V3 := [others => 0.0];
         K : Natural := 0;
      end record;
      package Cell_Maps is new Ada.Containers.Ordered_Maps (Cell, Acc);
      Mp : Cell_Maps.Map;
      S : Surface;
      Ok : Boolean;
      U : constant V3 := Unit (Up, Ok);
      E1, E2 : V3;
   begin
      if not Ok or else not Pitch'Valid or else Pitch <= 0.0 or else Pts.Is_Empty then
         return No_Surface;
      end if;
      Plane_Basis (U, E1, E2);
      for P of Pts loop
         declare
            H : constant Long_Float := Dot (Sub (P, P0), U);
            On : constant V3 := Sub (P, Scl (H, U));   --  投到面上
            C : constant Cell := (I => Integer (Long_Float'Floor (Dot (On, E1) / Pitch)), J => Integer (Long_Float'Floor (Dot (On, E2) / Pitch)));
            Cu : constant Cell_Maps.Cursor := Mp.Find (C);
         begin
            if Cell_Maps.Has_Element (Cu) then
               declare
                  E : Acc := Cell_Maps.Element (Cu);
               begin
                  E.S := Add (E.S, On); E.K := E.K + 1;
                  Mp.Replace_Element (Cu, E);
               end;
            else
               Mp.Insert (C, Acc'(S => On, K => 1));
            end if;
         end;
      end loop;
      S.Present := True; S.Up := U; S.Pitch := Pitch;
      for E of Mp loop
         S.Foot.Append (Scl (1.0 / Long_Float (E.K), E.S));   --  格里取平均
      end loop;
      return S;
   end Footprint;

   --  面内一串点(按 E1, E2 的坐标)的凸包顶点下标(单调链;共线的中间点不要;一个点 / 两个点照原样)
   type Idx_Array is array (Natural range <>) of Natural;
   type Xy is record
      X, Y : Long_Float;
      K : Natural;
   end record;
   type Xy_Array is array (Natural range <>) of Xy;
   function "<" (A, B : Xy) return Boolean is (A.X < B.X or else (A.X = B.X and then A.Y < B.Y));
   procedure Sort_Xy is new Ada.Containers.Generic_Array_Sort (Natural, Xy, Xy_Array);
   function Hull (P : Xy_Array) return Idx_Array is
      S : Xy_Array := P;
      H : Xy_Array (0 .. 2 * P'Length) := [others => (X => 0.0, Y => 0.0, K => 0)];
      N : Natural := 0;
      function Side_Of (O, A, B : Xy) return Long_Float is ((A.X - O.X) * (B.Y - O.Y) - (A.Y - O.Y) * (B.X - O.X));
   begin
      if P'Length <= 2 then
         declare
            R : Idx_Array (0 .. P'Length - 1);
         begin
            for I in 0 .. P'Length - 1 loop
               R (I) := P (P'First + I).K;
            end loop;
            return R;
         end;
      end if;
      Sort_Xy (S);
      for I in S'Range loop   --  下半圈
         while N >= 2 and then Side_Of (H (N - 2), H (N - 1), S (I)) <= 0.0 loop
            N := N - 1;
         end loop;
         H (N) := S (I); N := N + 1;
      end loop;
      declare
         Lo : constant Natural := N + 1;
      begin
         for I in reverse S'First .. S'Last - 1 loop   --  上半圈
            while N >= Lo and then Side_Of (H (N - 2), H (N - 1), S (I)) <= 0.0 loop
               N := N - 1;
            end loop;
            H (N) := S (I); N := N + 1;
         end loop;
      end;
      N := N - 1;   --  最后一个就是第一个
      declare
         R : Idx_Array (0 .. Natural'Max (N, 1) - 1);
      begin
         for I in R'Range loop
            R (I) := H (I).K;
         end loop;
         return R;
      end;
   end Hull;

   subtype W6 is Real_Array (0 .. 5);
   package W6_Vectors is new Ada.Containers.Vectors (Natural, W6);

   function Least (Ts : Touch_Vectors.Vector; Ref, F, Mo, Align : V3; Sup : Surface; M : Twist; Mu_Hand, Mu_Surf : Long_Float; Why : out Why_Kind) return Long_Float is
      Nt : constant Natural := Natural (Ts.Length);
      Len : Long_Float := 0.0;   --  特征长度:各手接触离参考点的平均距离(力矩那三行除以它,和力那三行同一个量级;解不随它变)
      Hand : W6_Vectors.Vector;   --  手的生成元(每一份单位法向力产生的力旋量),成本 1
      Surf : W6_Vectors.Vector;   --  面的生成元,成本 0
      procedure Put (To : in out W6_Vectors.Vector; Fv, Tq : V3) is
      begin
         To.Append (W6'[Fv (0), Fv (1), Fv (2), Tq (0), Tq (1), Tq (2)]);
      end Put;
   begin
      Why := Fine;
      if not Mu_Hand'Valid or else Mu_Hand < 0.0 or else not Mu_Surf'Valid or else Mu_Surf < 0.0 then
         Why := Unbalanced;
         return No_Way;
      end if;
      for T of Ts loop
         Len := Len + Norm (Sub (T.P, Ref)) / Long_Float (Nt);
      end loop;
      if Len <= Tiny then
         Len := 1.0;
      end if;
      --  手:每处接触的摩擦锥(Edges 条棱)+ 能拧的正反两个方向
      for T of Ts loop
         declare
            On, Ot : Boolean;
            N : constant V3 := Unit (T.N, On);
            R : constant V3 := Sub (T.P, Ref);
            Fn : constant Long_Float := Dot (Align, N);
            Ft : constant V3 := Sub (Align, Scl (Fn, N));
            Seed : constant V3 := (if abs N (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);   --  0.9:挑一条不和法向平行的种子轴(无量纲)
            T1a : constant V3 := Unit (Ft, Ot);
            T1 : constant V3 := (if Ot then T1a else Unit (Cross (N, Seed), On));
            T2 : constant V3 := Cross (N, T1);
         begin
            if not On then
               Why := Unbalanced;
               return No_Way;
            end if;
            for K in 0 .. Edges - 1 loop
               declare
                  Th : constant Long_Float := 2.0 * Pi * Long_Float (K) / Long_Float (Edges);
                  Fv : V3;
               begin
                  for I in 0 .. 2 loop
                     Fv (I) := N (I) + Mu_Hand * (Cos (Th) * T1 (I) + Sin (Th) * T2 (I));
                  end loop;
                  Put (Hand, Fv, Cross (R, Fv));
               end;
            end loop;
            if T.Twist_R > 0.0 then
               declare
                  Tq : constant V3 := Cross (R, N);
                  Tw : constant Long_Float := Mu_Hand * T.Twist_R;
               begin
                  Put (Hand, N, Add (Tq, Scl (Tw, N)));
                  Put (Hand, N, Sub (Tq, Scl (Tw, N)));
               end;
            end if;
         end;
      end loop;
      --  面:按要的旋量看那一片每一点往哪走
      if Sup.Present and then not Sup.Foot.Is_Empty then
         declare
            Ou : Boolean;
            U : constant V3 := Unit (Sup.Up, Ou);
            Np : constant Natural := Natural (Sup.Foot.Length);
            type V3_Arr is array (0 .. Np - 1) of V3;
            P, Tv : V3_Arr;
            Nv : array (0 .. Np - 1) of Long_Float;
            Wm : constant Long_Float := Norm (M.Ang);
            Vm : constant Long_Float := Norm (M.Lin);
            Rmax : Long_Float := 0.0;
            Tol : Long_Float;
            E1, E2 : V3;
            Stay : Natural := 0;
            Slide : Natural := 0;
         begin
            if not Ou then
               Why := Unbalanced;
               return No_Way;
            end if;
            Plane_Basis (U, E1, E2);
            for I in 0 .. Np - 1 loop
               P (I) := Sup.Foot (I);
               Rmax := Long_Float'Max (Rmax, Norm (Sub (P (I), M.Pivot)));
            end loop;
            --  分辨率:点的位置只准到一个采样间距,转动时一个间距里各点的速度差 = 角速度 × 间距;另加算作零的那一份(数值)
            Tol := Wm * Sup.Pitch + Tiny * (Vm + Wm * Rmax);
            for I in 0 .. Np - 1 loop
               declare
                  Uv : constant V3 := Add (M.Lin, Cross (M.Ang, Sub (P (I), M.Pivot)));
               begin
                  Nv (I) := Dot (Uv, U);
                  Tv (I) := Sub (Uv, Scl (Nv (I), U));
                  if Nv (I) < -Tol then
                     Why := Surface_In_Way;   --  这一点要往面里去:面挡着
                     return No_Way;
                  end if;
                  if abs Nv (I) <= Tol then
                     Stay := Stay + 1;
                     if Norm (Tv (I)) > Tol then
                        Slide := Slide + 1;
                     end if;
                  end if;
               end;
            end loop;
            if Stay > 0 then
               declare
                  Pts : Xy_Array (0 .. Stay - 1);
                  K : Natural := 0;
                  Ff, Mf : V3 := [others => 0.0];   --  每一份法向压力带出来的滑动摩擦(按均匀分摊到贴着的每一点)
               begin
                  for I in 0 .. Np - 1 loop
                     if abs Nv (I) <= Tol then
                        Pts (K) := (X => Dot (P (I), E1), Y => Dot (P (I), E2), K => I);
                        K := K + 1;
                        if Norm (Tv (I)) > Tol then
                           declare
                              Ok : Boolean;
                              Tu : constant V3 := Unit (Tv (I), Ok);
                              Fi : constant V3 := Scl (-Mu_Surf / Long_Float (Stay), Tu);
                           begin
                              Ff := Add (Ff, Fi);
                              Mf := Add (Mf, Cross (Sub (P (I), Ref), Fi));
                           end;
                        end if;
                     end if;
                  end loop;
                  for H of Hull (Pts) loop
                     declare
                        R : constant V3 := Sub (P (H), Ref);
                     begin
                        if Slide > 0 then
                           --  压在这一点的法向压力 + 它带出来的滑动摩擦(方向跟各点滑的方向相反,不能帮着推)
                           Put (Surf, Add (U, Ff), Add (Cross (R, U), Mf));
                        else
                           --  没有一点在滑(翻的那条边、它不动):摩擦在锥里
                           for Ke in 0 .. Edges - 1 loop
                              declare
                                 Th : constant Long_Float := 2.0 * Pi * Long_Float (Ke) / Long_Float (Edges);
                                 Fv : constant V3 := Add (U, Scl (Mu_Surf, Add (Scl (Cos (Th), E1), Scl (Sin (Th), E2))));
                              begin
                                 Put (Surf, Fv, Cross (R, Fv));
                              end;
                           end loop;
                        end if;
                     end;
                  end loop;
               end;
            end if;
         end;
      end if;
      declare
         Nh : constant Natural := Natural (Hand.Length);
         Ns : constant Natural := Natural (Surf.Length);
         Cols : constant Natural := Nh + Ns;
      begin
         if Cols = 0 then
            if Norm (F) <= Tiny and then Norm (Mo) <= Tiny then
               return 0.0;
            end if;
            Why := Unbalanced;
            return No_Way;
         end if;
         declare
            A : Real_Array (0 .. 6 * Cols - 1) := [others => 0.0];
            B : constant Real_Array (0 .. 5) := [F (0), F (1), F (2), Mo (0) / Len, Mo (1) / Len, Mo (2) / Len];
            Cost : Real_Array (0 .. Cols - 1) := [others => 0.0];
            Obj : Long_Float;
            Ok : Boolean;
            procedure Col (J : Natural; G : W6) is
            begin
               for R in 0 .. 2 loop
                  A (R * Cols + J) := G (R);
                  A ((3 + R) * Cols + J) := G (3 + R) / Len;
               end loop;
            end Col;
         begin
            for J in 0 .. Nh - 1 loop
               Col (J, Hand (J));
               Cost (J) := 1.0;   --  手的一份单位法向力
            end loop;
            for J in 0 .. Ns - 1 loop
               Col (Nh + J, Surf (J));
            end loop;
            Min_Sum (A, 6, Cols, B, Obj, Ok, Cost);
            if not Ok then
               Why := Unbalanced;
               return No_Way;
            end if;
            return Obj;
         end;
      end;
   end Least;

   function Need (Ts : Touch_Vectors.Vector; Com, Up : V3; Sup : Surface; M : Twist; Mu_Hand, Mu_Surf : Long_Float; Why : out Why_Kind) return Long_Float is
      Ok : Boolean;
      U : constant V3 := Unit (Up, Ok);
   begin
      if not Ok then
         Why := Unbalanced;
         return No_Way;
      end if;
      --  配平重力:这几处接触(连同面)要一起产生朝上的单位重量,绕重心的力矩为零;第一条棱对准"上"
      return Least (Ts, Com, U, [0.0, 0.0, 0.0], U, Sup, M, Mu_Hand, Mu_Surf, Why);
   end Need;

   function Squeeze (Ts : Touch_Vectors.Vector; L : Load; Mu : Long_Float) return Long_Float is
      Why : Why_Kind;
   begin
      if Ts.Is_Empty then
         return No_Way;
      end if;
      return Least (Ts, L.C, L.F, L.M, L.F, No_Surface, Still (L.C), Mu, 0.0, Why);
   end Squeeze;

   function Mu_Need (Ts : Touch_Vectors.Vector; L : Load) return Long_Float is
      Lo : Long_Float := 0.0;
      Hi : Long_Float := 1.0 / 64.0;   --  从 1/64 起翻倍(次数的起点)
   begin
      if Squeeze (Ts, L, 0.0) < No_Way then
         return 0.0;   --  不靠摩擦也做得到(比如东西被夹在一个朝上的 V 里)
      end if;
      while Squeeze (Ts, L, Hi) = No_Way loop
         Lo := Hi;
         Hi := 2.0 * Hi;
         if Hi > Mu_Top then
            return No_Way;
         end if;
      end loop;
      for It in 1 .. 30 loop   --  二分 30 次(次数:区间缩到原来的 1e-9)
         declare
            Mid : constant Long_Float := 0.5 * (Lo + Hi);
         begin
            if Squeeze (Ts, L, Mid) < No_Way then
               Hi := Mid;
            else
               Lo := Mid;
            end if;
         end;
      end loop;
      return Hi;
   end Mu_Need;

end Contact.Wrench;
