with Ada.Numerics;
with Driver.Bytes;
with Driver.Images;
with Driver.Numerics.Dense;
with Driver.Stats;
with Driver.Tests;
with Driver.Uncertain;

package body Driver.Core_Tests is

   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use type Driver.Bytes.Byte;
   use type Driver.Bytes.Offset;

   Pi : constant := Ada.Numerics.Pi;

   function Max_Abs (A : Mat3) return Real is
      M : Real := 0.0;
   begin
      for I in 1 .. 3 loop
         for J in 1 .. 3 loop
            M := Real'Max (M, abs A (I, J));
         end loop;
      end loop;
      return M;
   end Max_Abs;

   procedure Rotation_Round_Trip is
      Samples : constant array (Positive range <>) of Vec3 :=
        [[0.3, -0.2, 0.9], [1.0e-9, 0.0, 0.0], [0.0, Pi - 1.0e-6, 0.0], [Pi, 0.0, 0.0],
         [-1.2, 2.0, 0.4], [0.0, 0.0, 0.0]];
   begin
      for W of Samples loop
         declare
            R    : constant Mat3 := Exp (W);
            Back : constant Mat3 := Exp (Log (R));
         begin
            Check (Max_Abs (R * Transpose (R) - Identity3) < 1.0e-12, "Exp is not orthonormal");
            Check (Max_Abs (Back - R) < 1.0e-9, "Exp (Log (R)) differs from R");
            Check (Angle (R) <= Pi + 1.0e-12, "Log angle outside [0, pi]");
         end;
      end loop;
   end Rotation_Round_Trip;

   procedure Quaternion_Round_Trip is
      R : constant Mat3 := Exp ([0.4, -1.9, 0.7]);
   begin
      Check (Max_Abs (To_Matrix (To_Quaternion (R)) - R) < 1.0e-12, "quaternion round trip");
      Check (Max_Abs (To_Matrix (To_Quaternion (Exp ([Pi, 0.0, 0.0]))) - Exp ([Pi, 0.0, 0.0])) < 1.0e-12,
             "quaternion round trip at pi");
   end Quaternion_Round_Trip;

   procedure Rigid_Inverse is
      T : constant Rigid := (Rotation => Exp ([0.2, 0.5, -0.3]), Translation => [1.0, -2.0, 0.5]);
      P : constant Vec3 := [0.3, 0.4, -0.7];
   begin
      Check (abs (Inverse (T) * (T * P) - P) < 1.0e-12, "inverse does not undo");
      Check (abs ((T * Inverse (T)) * P - P) < 1.0e-12, "T * inverse is not the identity");
   end Rigid_Inverse;

   procedure Least_Squares_Exact is
      A : constant Real_Matrix := [[1.0, 0.0], [1.0, 1.0], [1.0, 2.0], [1.0, 3.0]];
      B : constant Real_Vector := [2.0, 5.0, 8.0, 11.0];
      X : Real_Vector (1 .. 2);
      Ok : Boolean;
   begin
      Driver.Numerics.Dense.Least_Squares (A, B, X, Ok);
      Check (Ok, "full-rank system reported deficient");
      Check_Close (X (1), 2.0, 1.0e-12, "intercept");
      Check_Close (X (2), 3.0, 1.0e-12, "slope");
      Driver.Numerics.Dense.Least_Squares ([[1.0, 2.0], [2.0, 4.0], [3.0, 6.0]], [1.0, 2.0, 3.0], X, Ok);
      Check (not Ok, "rank-deficient system reported full rank");
   end Least_Squares_Exact;

   procedure Cholesky_Solve is
      A : constant Real_Matrix := [[4.0, 2.0, 0.4], [2.0, 5.0, 1.0], [0.4, 1.0, 3.0]];
      B : constant Real_Vector := [1.0, -2.0, 0.5];
      L : Real_Matrix (1 .. 3, 1 .. 3);
      Ok : Boolean;
   begin
      Driver.Numerics.Dense.Cholesky (A, L, Ok);
      Check (Ok, "positive-definite matrix rejected");
      Check (abs (A * Driver.Numerics.Dense.Cholesky_Solve (L, B) - B) < 1.0e-12, "solution does not satisfy A x = b");
      declare
         L2 : Real_Matrix (1 .. 2, 1 .. 2);
      begin
         Driver.Numerics.Dense.Cholesky ([[1.0, 2.0], [2.0, 1.0]], L2, Ok);
         Check (not Ok, "indefinite matrix accepted");
      end;
   end Cholesky_Solve;

   procedure Robust_Statistics is
      X : constant Real_Array := [1.0, 2.0, 3.0, 4.0, 1000.0];
   begin
      Check_Close (Driver.Stats.Median (X), 3.0, 0.0, "median");
      Check (Driver.Stats.Robust_Sigma (X) < 2.0, "one outlier moves the robust sigma");
      Check_Close (Driver.Stats.Correlation ([1.0, 2.0, 3.0], [2.0, 4.0, 6.0]), 1.0, 1.0e-12, "perfect correlation");
      declare
         L : constant Driver.Stats.Line := Driver.Stats.Fit_Line ([0.0, 1.0, 2.0, 3.0], [1.0, 3.0, 5.0, 7.0]);
      begin
         Check_Close (L.Slope.Value, 2.0, 1.0e-12, "line slope");
         Check_Close (L.Intercept.Value, 1.0, 1.0e-12, "line intercept");
      end;
   end Robust_Statistics;

   procedure Significance is
      use Driver.Uncertain;
   begin
      Check (Significant (3.1, 1.0), "3.1 sigma not significant");
      Check (not Significant (2.9, 1.0), "2.9 sigma significant");
      Check (not Significant (1.0e9, Real'Last), "unknown sigma made something significant");
      Check (not Significant (Estimate'(10.0, 2.0), Estimate'(5.0, 2.0)), "5 apart with combined sigma 2.83");
      Check (Significant (Estimate'(10.0, 1.0), Estimate'(5.0, 1.0)), "5 apart with combined sigma 1.41");
   end Significance;

   procedure Buffer_Growth is
      B : Driver.Bytes.Buffer;
   begin
      for I in 1 .. 1000 loop
         B.Append (Driver.Bytes.Byte (I mod 256));
      end loop;
      B.Append ("abc");
      Check (B.Length = 1003, "length after appends");
      Check (B.Element (1000) = Driver.Bytes.Byte (1000 mod 256), "element kept across growth");
      declare
         C : constant Driver.Bytes.Buffer := B;
      begin
         B.Clear;
         Check (C.Length = 1003, "a copy shares storage with its source");
      end;
   end Buffer_Growth;

   procedure Image_Access is
      use Driver.Bytes;
      Data : Byte_Array (1 .. 3 * 4 * 2) := [others => 0];
   begin
      Data (3 * (1 * 4 + 2) + 1) := 200;   --  red of column 2, row 1
      declare
         I : constant Driver.Images.Image := Driver.Images.Create (4, 2, Data);
         M : Driver.Images.Mask := Driver.Images.Create (4, 2);
      begin
         Check (Driver.Images.Red (I, 2, 1) = 200, "pixel addressed by column and row");
         Check (Driver.Images.Red (I, 1, 2 - 1) = 0, "neighbouring pixel");
         Driver.Images.Include (M, 3, 1);
         Check (Driver.Images.Contains (M, 3, 1) and then Driver.Images.Count (M) = 1, "mask membership");
      end;
   end Image_Access;

   procedure Register is
   begin
      Driver.Tests.Register ("core.rotation", "Exp and Log disagree near 0 or pi", Rotation_Round_Trip'Access);
      Driver.Tests.Register ("core.quaternion", "quaternion conversion loses a rotation", Quaternion_Round_Trip'Access);
      Driver.Tests.Register ("core.rigid", "a rigid inverse does not undo the transform", Rigid_Inverse'Access);
      Driver.Tests.Register ("core.least_squares", "QR least squares wrong or blind to rank loss",
                             Least_Squares_Exact'Access);
      Driver.Tests.Register ("core.cholesky", "Cholesky accepts indefinite input or solves wrongly",
                             Cholesky_Solve'Access);
      Driver.Tests.Register ("core.stats", "robust statistics moved by a single outlier", Robust_Statistics'Access);
      Driver.Tests.Register ("core.significance", "the one significance rule misjudges a difference",
                             Significance'Access);
      Driver.Tests.Register ("core.buffer", "a byte buffer loses data when it grows or copies",
                             Buffer_Growth'Access);
      Driver.Tests.Register ("core.image", "pixels are addressed by the wrong column or row", Image_Access'Access);
   end Register;

end Driver.Core_Tests;
