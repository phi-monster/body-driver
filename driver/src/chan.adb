with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
package body Chan is
   function Quat_Mul (A, B : Pose) return Pose is
      R : Pose := [others => 0.0];
      W1 : constant Long_Float := A (3); X1 : constant Long_Float := A (4); Y1 : constant Long_Float := A (5); Z1 : constant Long_Float := A (6);
      W2 : constant Long_Float := B (3); X2 : constant Long_Float := B (4); Y2 : constant Long_Float := B (5); Z2 : constant Long_Float := B (6);
   begin
      R (3) := W1 * W2 - X1 * X2 - Y1 * Y2 - Z1 * Z2;
      R (4) := W1 * X2 + X1 * W2 + Y1 * Z2 - Z1 * Y2;
      R (5) := W1 * Y2 - X1 * Z2 + Y1 * W2 + Z1 * X2;
      R (6) := W1 * Z2 + X1 * Y2 - Y1 * X2 + Z1 * W2;
      return R;
   end Quat_Mul;

   function Compose (P0 : Pose; A : Table.Vec; Offset : Natural := 0) return Pose is
      P : Pose := P0;
      Rx : constant Long_Float := A (Offset + 3);
      Ry : constant Long_Float := A (Offset + 4);
      Rz : constant Long_Float := A (Offset + 5);
      Ang : constant Long_Float := Sqrt (Rx * Rx + Ry * Ry + Rz * Rz);
   begin
      P (0) := P0 (0) + A (Offset);
      P (1) := P0 (1) + A (Offset + 1);
      P (2) := P0 (2) + A (Offset + 2);
      if Ang > 1.0e-12 then
         declare
            Dq : Pose := [others => 0.0];
            S : constant Long_Float := Sin (Ang / 2.0) / Ang;
            Q : Pose;
            N : Long_Float;
         begin
            Dq (3) := Cos (Ang / 2.0);
            Dq (4) := Rx * S; Dq (5) := Ry * S; Dq (6) := Rz * S;
            Q := Quat_Mul (Dq, P0);      --  世界轴小转动 ⇒ 左乘
            N := Sqrt (Q (3) * Q (3) + Q (4) * Q (4) + Q (5) * Q (5) + Q (6) * Q (6));
            if N > 1.0e-12 then
               for K in 3 .. 6 loop
                  P (K) := Q (K) / N;
               end loop;
            end if;
         end;
      end if;
      return P;
   end Compose;

   function Rot_Vec (Q0, Q1 : Pose) return Table.Vec3 is
      Conj : Pose := Q0;
      D : Pose;
      V : Table.Vec3 := Table.Zero3;
      W, S, Ang : Long_Float;
   begin
      for K in 4 .. 6 loop
         Conj (K) := -Q0 (K);
      end loop;
      D := Quat_Mul (Q1, Conj);     --  Q1 = D ⊗ Q0 ⇒ D = Q1 ⊗ Q0⁻¹
      W := D (3);
      if W < 0.0 then
         W := -W;
         for K in 4 .. 6 loop
            D (K) := -D (K);
         end loop;
      end if;
      S := Sqrt (D (4) * D (4) + D (5) * D (5) + D (6) * D (6));
      if S < 1.0e-12 then
         return V;
      end if;
      Ang := 2.0 * Arctan (S, Long_Float'Min (1.0, W));
      V (0) := D (4) / S * Ang; V (1) := D (5) / S * Ang; V (2) := D (6) / S * Ang;
      return V;
   end Rot_Vec;

   function Delivered (P0, P1 : Pose) return Table.Vec is
      A : Table.Vec := Table.Zero_Vec;
      R : constant Table.Vec3 := Rot_Vec (P0, P1);
   begin
      A (0) := P1 (0) - P0 (0);
      A (1) := P1 (1) - P0 (1);
      A (2) := P1 (2) - P0 (2);
      A (3) := R (0); A (4) := R (1); A (5) := R (2);
      return A;
   end Delivered;
end Chan;
