with Ada.Numerics.Long_Elementary_Functions;

package body Driver.Numerics is

   use Ada.Numerics.Long_Elementary_Functions;

   function Cross (A, B : Vec3) return Vec3 is
     [A (2) * B (3) - A (3) * B (2),
      A (3) * B (1) - A (1) * B (3),
      A (1) * B (2) - A (2) * B (1)];

   --  Where the length overflows the float (a component beyond the square root of its largest), the vector over
   --  its largest component first: V over an infinite length is nought, no direction, and a fit's axis tilted past
   --  the float's range went to nought so and ended the estimates (n17s2 shadow2a, 10-27). Every other vector as
   --  before, to the bit.
   function Unit (V : Vec3) return Vec3 is
      Length : constant Real := abs V;
   begin
      if Length <= Real'Last then
         return V / Length;
      end if;
      declare
         Largest : constant Real := Real'Max (abs V (1), Real'Max (abs V (2), abs V (3)));
         W       : constant Vec3 := [V (1) / Largest, V (2) / Largest, V (3) / Largest];
      begin
         return W / abs W;
      end;
   end Unit;

   --  Each sum is accumulated term by term from zero, as the general operators
   --  accumulate it, so that a multiply-add the compiler fuses is fused the
   --  same way in both.
   function Times (M : Mat3; V : Vec3) return Vec3 is
      R : Vec3;
   begin
      for I in 1 .. 3 loop
         declare
            S : Real := 0.0;
         begin
            for K in 1 .. 3 loop
               S := S + M (I, K) * V (K);
            end loop;
            R (I) := S;
         end;
      end loop;
      return R;
   end Times;

   function Transposed_Times (M : Mat3; V : Vec3) return Vec3 is
      R : Vec3;
   begin
      for I in 1 .. 3 loop
         declare
            S : Real := 0.0;
         begin
            for K in 1 .. 3 loop
               S := S + M (K, I) * V (K);
            end loop;
            R (I) := S;
         end;
      end loop;
      return R;
   end Transposed_Times;

   function Times (A, B : Mat3) return Mat3 is
      R : Mat3;
   begin
      for I in 1 .. 3 loop
         for J in 1 .. 3 loop
            declare
               S : Real := 0.0;
            begin
               for K in 1 .. 3 loop
                  S := S + A (I, K) * B (K, J);
               end loop;
               R (I, J) := S;
            end;
         end loop;
      end loop;
      return R;
   end Times;

   function Transposed (M : Mat3) return Mat3 is
     [[M (1, 1), M (2, 1), M (3, 1)],
      [M (1, 2), M (2, 2), M (3, 2)],
      [M (1, 3), M (2, 3), M (3, 3)]];

   function Plus (A, B : Vec3) return Vec3 is [A (1) + B (1), A (2) + B (2), A (3) + B (3)];

   function Scaled (S : Real; V : Vec3) return Vec3 is [S * V (1), S * V (2), S * V (3)];

   function Minus (A, B : Vec3) return Vec3 is [A (1) - B (1), A (2) - B (2), A (3) - B (3)];

   function Dot (A, B : Vec3) return Real is
      S : Real := 0.0;
   begin
      for K in 1 .. 3 loop
         S := S + A (K) * B (K);
      end loop;
      return S;
   end Dot;

   function Outer (A, B : Vec3) return Mat3 is
     [for I in 1 .. 3 => [for J in 1 .. 3 => A (I) * B (J)]];

   function Skew (V : Vec3) return Mat3 is
     [[0.0, -V (3), V (2)],
      [V (3), 0.0, -V (1)],
      [-V (2), V (1), 0.0]];

   --  Written out, as Times is: the general operators' sums each a result on the secondary stack, and the arm fit
   --  composes hundreds of thousands of turns a second (Eye_At: of the main task's samples of Panda 1a's last fit of
   --  round p314s2, replayed on the Mac, 10-25, a sixth were the general operators under this).
   function Exp (Rotation_Vector : Vec3) return Mat3 is
      Theta : constant Real := Sqrt (Dot (Rotation_Vector, Rotation_Vector));
      K     : constant Mat3 := Skew (Rotation_Vector);
      Turn  : Mat3 := Identity3;
   begin
      if Theta = 0.0 then
         for I in 1 .. 3 loop
            for J in 1 .. 3 loop
               Turn (I, J) := Turn (I, J) + K (I, J);
            end loop;
         end loop;
         return Turn;
      end if;
      --  sin and 1 - cos lose no precision until theta is far below 1e-8,
      --  where the first-order term above is exact to double precision.
      declare
         A  : constant Real := Sin (Theta) / Theta;
         B  : constant Real := (1.0 - Cos (Theta)) / (Theta * Theta);
         KK : constant Mat3 := Times (K, K);
      begin
         for I in 1 .. 3 loop
            for J in 1 .. 3 loop
               Turn (I, J) := (Turn (I, J) + A * K (I, J)) + B * KK (I, J);
            end loop;
         end loop;
      end;
      return Turn;
   end Exp;

   function Log (R : Mat3) return Vec3 is
      Trace : constant Real := R (1, 1) + R (2, 2) + R (3, 3);
      C     : constant Real := Real'Max (-1.0, Real'Min (1.0, (Trace - 1.0) / 2.0));
      W     : constant Vec3 := [R (3, 2) - R (2, 3), R (1, 3) - R (3, 1), R (2, 1) - R (1, 2)];
      S     : constant Real := abs W / 2.0;
      Theta : constant Real := Arctan (S, C);
   begin
      if S > 0.0 and then C > -0.5 then
         --  Away from pi the antisymmetric part fixes the axis well.
         return (Theta / (2.0 * S)) * W;
      elsif S = 0.0 and then C > 0.0 then
         return Zero3;
      end if;
      --  Near pi the axis comes from the symmetric part: R + I = 2 a a^T
      --  (1 - cos) + 2 cos I; take the best-conditioned column.
      declare
         B    : constant Mat3 := (R + Transpose (R)) / 2.0 - C * Identity3;
         Best : Integer := 1;
         Axis : Vec3;
      begin
         for I in 2 .. 3 loop
            if B (I, I) > B (Best, Best) then
               Best := I;
            end if;
         end loop;
         Axis := [B (1, Best), B (2, Best), B (3, Best)];
         Axis := Unit (Axis);
         if Axis * W < 0.0 then
            Axis := -Axis;
         end if;
         return Theta * Axis;
      end;
   end Log;

   procedure Symmetric_Eigensystem (M : Mat3; Values : out Vec3; Vectors : out Mat3) is
   begin
      Eigensystem ((M + Transpose (M)) / 2.0, Values, Vectors);
   end Symmetric_Eigensystem;

   function Orthonormalize (R : Mat3) return Mat3 is
      --  For R = U S V^T the nearest rotation is U diag (1, 1, det (U V^T)) V^T.
      --  V comes from the eigensystem of R^T R, in descending order; the first
      --  two left vectors from R V, the third as their cross product, which
      --  also holds when R has rank two (points in a plane: R v3 is zero).
      --  With u3 = u1 x u2, det (U V^T) times u3 v3^T is det (V) u3 v3^T
      --  whichever sign the true third left vector has.
      Values : Vec3;
      V      : Mat3;
      U1, U2 : Vec3;
      function Column (J : Positive) return Vec3 is ([V (1, J), V (2, J), V (3, J)]);
   begin
      Symmetric_Eigensystem (Transpose (R) * R, Values, V);
      U1 := Unit (R * Column (1));
      U2 := R * Column (2);
      U2 := Unit (U2 - Real'(U2 * U1) * U1);
      return Outer (U1, Column (1)) + Outer (U2, Column (2)) + Determinant (V) * Outer (Cross (U1, U2), Column (3));
   end Orthonormalize;

   function To_Matrix (Q : Quaternion) return Mat3 is
      N : constant Real := Sqrt (Q.W * Q.W + Q.X * Q.X + Q.Y * Q.Y + Q.Z * Q.Z);
      W : constant Real := Q.W / N;
      X : constant Real := Q.X / N;
      Y : constant Real := Q.Y / N;
      Z : constant Real := Q.Z / N;
   begin
      return
        [[1.0 - 2.0 * (Y * Y + Z * Z), 2.0 * (X * Y - W * Z), 2.0 * (X * Z + W * Y)],
         [2.0 * (X * Y + W * Z), 1.0 - 2.0 * (X * X + Z * Z), 2.0 * (Y * Z - W * X)],
         [2.0 * (X * Z - W * Y), 2.0 * (Y * Z + W * X), 1.0 - 2.0 * (X * X + Y * Y)]];
   end To_Matrix;

   function To_Quaternion (R : Mat3) return Quaternion is
      Trace : constant Real := R (1, 1) + R (2, 2) + R (3, 3);
      S     : Real;
   begin
      --  Shepperd's method: divide by the largest of the four candidates.
      if Trace > R (1, 1) and then Trace > R (2, 2) and then Trace > R (3, 3) then
         S := 2.0 * Sqrt (1.0 + Trace);
         return (W => S / 4.0, X => (R (3, 2) - R (2, 3)) / S,
                 Y => (R (1, 3) - R (3, 1)) / S, Z => (R (2, 1) - R (1, 2)) / S);
      elsif R (1, 1) > R (2, 2) and then R (1, 1) > R (3, 3) then
         S := 2.0 * Sqrt (1.0 + R (1, 1) - R (2, 2) - R (3, 3));
         return (W => (R (3, 2) - R (2, 3)) / S, X => S / 4.0,
                 Y => (R (1, 2) + R (2, 1)) / S, Z => (R (1, 3) + R (3, 1)) / S);
      elsif R (2, 2) > R (3, 3) then
         S := 2.0 * Sqrt (1.0 + R (2, 2) - R (1, 1) - R (3, 3));
         return (W => (R (1, 3) - R (3, 1)) / S, X => (R (1, 2) + R (2, 1)) / S,
                 Y => S / 4.0, Z => (R (2, 3) + R (3, 2)) / S);
      else
         S := 2.0 * Sqrt (1.0 + R (3, 3) - R (1, 1) - R (2, 2));
         return (W => (R (2, 1) - R (1, 2)) / S, X => (R (1, 3) + R (3, 1)) / S,
                 Y => (R (2, 3) + R (3, 2)) / S, Z => S / 4.0);
      end if;
   end To_Quaternion;

   function "*" (A, B : Rigid) return Rigid is
     (Rotation => Times (A.Rotation, B.Rotation), Translation => Plus (Times (A.Rotation, B.Translation), A.Translation));

   function "*" (T : Rigid; P : Vec3) return Vec3 is (Plus (Times (T.Rotation, P), T.Translation));

   function Inverse (T : Rigid) return Rigid is
      Rt : constant Mat3 := Transposed (T.Rotation);
      Tt : constant Vec3 := Times (Rt, T.Translation);
   begin
      return (Rotation => Rt, Translation => [-Tt (1), -Tt (2), -Tt (3)]);
   end Inverse;

   function Spiral_Rotation (K, N : Positive) return Mat3 is
      Phi : constant Real := Sqrt (2.0);
      Psi : Real := 1.5;
   begin
      for Step in 1 .. Real'Machine_Mantissa loop
         declare
            Miss  : constant Real := Psi ** 4 - Psi - 4.0;
            Slope : constant Real := 4.0 * Psi ** 3 - 1.0;
         begin
            exit when Miss = 0.0 or else abs (Miss / Slope) <= Real'Epsilon * Psi;
            Psi := Psi - Miss / Slope;
         end;
      end loop;
      declare
         S : constant Real := Real (K) - 0.5;
         T : constant Real := S / Real (N);
         D : constant Real := 2.0 * Ada.Numerics.Pi * S;
         A : constant Real := Sqrt (T);
         B : constant Real := Sqrt (1.0 - T);
      begin
         return To_Matrix ((W => A * Sin (D / Phi), X => A * Cos (D / Phi), Y => B * Sin (D / Psi), Z => B * Cos (D / Psi)));
      end;
   end Spiral_Rotation;

end Driver.Numerics;
