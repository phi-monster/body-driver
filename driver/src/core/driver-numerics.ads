--  Three-dimensional geometry: vectors, rotations and rigid transforms.
--
--  Vectors and matrices are constrained subtypes of the standard real arrays,
--  so every operator of Ada.Numerics.Long_Real_Arrays applies to them ("abs"
--  of a vector is its Euclidean norm, "*" of two vectors their dot product).
--  Frames follow the usual convention: a Rigid T maps a point expressed in
--  its child frame to the parent frame, T * P = R P + t.

with Ada.Numerics.Long_Real_Arrays;

package Driver.Numerics with Pure is

   package Arrays renames Ada.Numerics.Long_Real_Arrays;
   use Arrays;

   subtype Vec3 is Real_Vector (1 .. 3);
   subtype Mat3 is Real_Matrix (1 .. 3, 1 .. 3);

   Zero3     : constant Vec3 := [0.0, 0.0, 0.0];
   Identity3 : constant Mat3 := [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

   function Cross (A, B : Vec3) return Vec3;

   --  The products of three-vectors and three-by-three matrices written out:
   --  the same sums in the same order as the general operators of
   --  Long_Real_Arrays, without their array machinery and the secondary stack
   --  each result takes there (the arm fit computes tens of millions of them).
   function Times (M : Mat3; V : Vec3) return Vec3 with Inline;
   --  M V.
   function Transposed_Times (M : Mat3; V : Vec3) return Vec3 with Inline;
   --  The transpose of M times V.
   function Times (A, B : Mat3) return Mat3 with Inline;
   --  A B.
   function Transposed (M : Mat3) return Mat3 with Inline;
   function Plus (A, B : Vec3) return Vec3 with Inline;
   function Minus (A, B : Vec3) return Vec3 with Inline;
   function Dot (A, B : Vec3) return Real with Inline;

   function Unit (V : Vec3) return Vec3
     with Pre => abs V > 0.0;

   function Outer (A, B : Vec3) return Mat3;

   function Skew (V : Vec3) return Mat3;
   --  The matrix of the cross product: Skew (A) * B = Cross (A, B).

   function Exp (Rotation_Vector : Vec3) return Mat3;
   --  The rotation by |w| radians about w / |w| (Rodrigues).

   function Log (R : Mat3) return Vec3;
   --  The rotation vector of R, with its angle in [0, pi].

   function Angle (R : Mat3) return Real is (abs Log (R));

   procedure Symmetric_Eigensystem (M : Mat3; Values : out Vec3; Vectors : out Mat3);
   --  The eigenvalues of the symmetric part of M in descending order, with
   --  their unit eigenvectors as the columns of Vectors. Matrices that are
   --  symmetric in exact arithmetic often are not after rounding, which the
   --  standard Eigensystem does not accept.

   function Orthonormalize (R : Mat3) return Mat3;
   --  The rotation nearest to R in the Frobenius norm (Kabsch's, with the
   --  determinant's sign kept): unique when R has rank two or more, so it
   --  also turns a cross-covariance of points in a plane into their rotation.

   type Quaternion is record
      W, X, Y, Z : Real := 0.0;
   end record;
   --  A unit quaternion w + xi + yj + zk; q and -q are the same rotation.

   function To_Matrix (Q : Quaternion) return Mat3;
   function To_Quaternion (R : Mat3) return Quaternion;

   type Rigid is record
      Rotation    : Mat3 := Identity3;
      Translation : Vec3 := Zero3;
   end record;

   Identity : constant Rigid :=
     (Rotation    => [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
      Translation => [0.0, 0.0, 0.0]);

   function "*" (A, B : Rigid) return Rigid;
   --  (A * B) * P = A * (B * P).

   function "*" (T : Rigid; P : Vec3) return Vec3;

   function Inverse (T : Rigid) return Rigid;

   function Rotate (T : Rigid; V : Vec3) return Vec3 is (Times (T.Rotation, V));
   --  A direction carried by T: rotation only.

end Driver.Numerics;
