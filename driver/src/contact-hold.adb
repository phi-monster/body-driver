with Ada.Numerics; use Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
package body Contact.Hold is

   --  摩擦锥线性化的棱数(次数):8 条,其中第一条对准"要的那个力"在接触面上的方向,主方向上是准的,别的方向内接、偏保守
   Edges : constant := 8;

   procedure Min_Sum (A : Real_Array; Rows, Cols : Positive; B : Real_Array; Obj : out Long_Float; Ok : out Boolean) is
      --  表:第 0 行 = 检验数,第 1 .. Rows 行 = 约束;列 0 .. Cols − 1 = 变量,Cols .. Cols + Rows − 1 = 人工变量,最后一列 = 右端
      Nc : constant Natural := Cols + Rows;
      type Tab is array (0 .. Rows, 0 .. Nc) of Long_Float;
      T : Tab := [others => [others => 0.0]];
      Bas : array (1 .. Rows) of Natural;
      Eps : constant := 1.0e-10;          --  算作零(无量纲:行都按量级归一过)
      Max_Pivots : constant Natural := 50 * (Rows + Cols);   --  防万一的步数上限(次数;Bland 规则本身不会循环)
      Scale : Long_Float := 0.0;
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
                  Done := False;   --  无界(目标是 Σ x ≥ 0,不该出现)
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
      --  第二阶段:目标 = Σ x(每个变量系数 1,人工变量 0、不许再进基)
      for J in 0 .. Nc loop
         T (0, J) := (if J < Cols then 1.0 else 0.0);
      end loop;
      T (0, Nc) := 0.0;
      for I in 1 .. Rows loop
         if Bas (I) < Cols then
            for J in 0 .. Nc loop
               T (0, J) := T (0, J) - T (I, J);
            end loop;
         end if;
      end loop;
      Run (Cols, Done);
      if not Done then
         return;
      end if;
      Obj := -T (0, Nc);
      Ok := True;
   end Min_Sum;

   function Squeeze (Ts : Touch_Vectors.Vector; L : Load; Mu : Long_Float) return Long_Float is
      Nt : constant Natural := Natural (Ts.Length);
      Per : Natural := 0;
      Len : Long_Float := 0.0;   --  特征长度:各接触离重心的平均距离(力矩那三行除以它,和力那三行同一个量级)
   begin
      if Nt = 0 or else not Mu'Valid or else Mu < 0.0 then
         return No_Way;
      end if;
      for T of Ts loop
         Per := Per + Edges + (if T.Twist_R > 0.0 then 2 else 0);
         Len := Len + Norm ([T.P (0) - L.C (0), T.P (1) - L.C (1), T.P (2) - L.C (2)]) / Long_Float (Nt);
      end loop;
      if Len <= 1.0e-12 then
         Len := 1.0;
      end if;
      declare
         A : Real_Array (0 .. 6 * Per - 1) := [others => 0.0];
         B : constant Real_Array (0 .. 5) := [L.F (0), L.F (1), L.F (2), L.M (0) / Len, L.M (1) / Len, L.M (2) / Len];
         Col : Natural := 0;
         Obj : Long_Float;
         Ok : Boolean;
         procedure Put (F, Tq : V3) is
         begin
            for R in 0 .. 2 loop
               A (R * Per + Col) := F (R);
               A ((3 + R) * Per + Col) := Tq (R) / Len;
            end loop;
            Col := Col + 1;
         end Put;
      begin
         for T of Ts loop
            declare
               On, Ot : Boolean;
               N : constant V3 := Unit (T.N, On);
               R : constant V3 := [T.P (0) - L.C (0), T.P (1) - L.C (1), T.P (2) - L.C (2)];
               --  第一条棱对准要的那个力在这个接触面上的方向;没有(要的力正好沿法向)就任取一条垂直的(0.9 是无量纲的比较:挑一条不和法向平行的种子轴)
               Fn : constant Long_Float := Dot (L.F, N);
               Ft : constant V3 := [L.F (0) - Fn * N (0), L.F (1) - Fn * N (1), L.F (2) - Fn * N (2)];
               Seed : constant V3 := (if abs N (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);
               T1a : constant V3 := Unit (Ft, Ot);
               T1 : constant V3 := (if Ot then T1a else Unit (Cross (N, Seed), On));
               T2 : constant V3 := Cross (N, T1);
            begin
               if not On then
                  return No_Way;
               end if;
               for K in 0 .. Edges - 1 loop
                  declare
                     Th : constant Long_Float := 2.0 * Pi * Long_Float (K) / Long_Float (Edges);
                     F : V3;
                  begin
                     for I in 0 .. 2 loop
                        F (I) := N (I) + Mu * (Cos (Th) * T1 (I) + Sin (Th) * T2 (I));
                     end loop;
                     Put (F, Cross (R, F));
                  end;
               end loop;
               if T.Twist_R > 0.0 then
                  declare
                     Tq : constant V3 := Cross (R, N);
                     Tw : constant Long_Float := Mu * T.Twist_R;
                  begin
                     Put (N, [Tq (0) + Tw * N (0), Tq (1) + Tw * N (1), Tq (2) + Tw * N (2)]);
                     Put (N, [Tq (0) - Tw * N (0), Tq (1) - Tw * N (1), Tq (2) - Tw * N (2)]);
                  end;
               end if;
            end;
         end loop;
         Min_Sum (A, 6, Per, B, Obj, Ok);
         return (if Ok then Obj else No_Way);
      end;
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

end Contact.Hold;
