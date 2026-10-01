with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;

package body Driver.Action.Snapshots.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   Pi : constant := Ada.Numerics.Pi;

   function At_Pose (P : Vec3) return Rigid is ((Rotation => Identity3, Translation => P));

   function One (P : Part) return Model is
      M : Model;
   begin
      M.Parts.Append (P);
      return M;
   end One;

   function Bar (Length, Width, Height : Real) return Model is
     (One ((Kind => Block, Pose => At_Pose ([0.0, 0.0, Height / 2.0]),
            Sizes => [Length / 2.0, Width / 2.0, Height / 2.0])));

   function Block (X, Y, Z : Real) return Model is (Bar (X, Y, Z));

   function Upright_Cylinder (Radius, Height : Real) return Model is
     (One ((Kind => Cylinder, Pose => At_Pose ([0.0, 0.0, Height / 2.0]), Sizes => [Radius, 0.0, Height / 2.0])));

   function Scissors (Length, Blade_Width, Thickness : Real) return Model is
      M     : Model;
      Cross : constant Real := 0.15;   --  half the angle between the blades, radians
      Ring  : constant Real := Blade_Width * 1.5;
   begin
      for Sign in -1 .. 1 loop
         if Sign /= 0 then
            declare
               R : constant Mat3 := Exp ([0.0, 0.0, Real (Sign) * Cross]);
            begin
               --  A blade from the pivot to the tip, and a ring of four bars
               --  behind the pivot for the handle.
               M.Parts.Append (Part'(Kind => Block,
                                Pose => (Rotation => R, Translation => R * [Length / 4.0, 0.0, Thickness / 2.0]),
                                Sizes => [Length / 4.0, Blade_Width / 2.0, Thickness / 2.0]));
               declare
                  Centre : constant Vec3 := R * [-Length / 4.0, Real (Sign) * Ring / 2.0, Thickness / 2.0];
                  W      : constant Real := Blade_Width / 2.0;
                  procedure Side (Offset, Sizes : Vec3) is
                  begin
                     M.Parts.Append (Part'(Kind => Block, Pose => (Rotation => R, Translation => Centre + R * Offset),
                                           Sizes => Sizes));
                  end Side;
               begin
                  Side ([Ring, 0.0, 0.0], [W, Ring, Thickness / 2.0]);
                  Side ([-Ring, 0.0, 0.0], [W, Ring, Thickness / 2.0]);
                  Side ([0.0, Ring, 0.0], [Ring, W, Thickness / 2.0]);
                  Side ([0.0, -Ring, 0.0], [Ring, W, Thickness / 2.0]);
               end;
            end;
         end if;
      end loop;
      return M;
   end Scissors;

   function Cup (Radius, Wall, Height : Real) return Model is
      M : Model;
   begin
      M.Parts.Append (Part'(Kind => Tube, Pose => At_Pose ([0.0, 0.0, Height / 2.0]),
                       Sizes => [Radius, Radius - Wall, Height / 2.0]));
      M.Parts.Append (Part'(Kind => Cylinder, Pose => At_Pose ([0.0, 0.0, Wall / 2.0]),
                       Sizes => [Radius - Wall, 0.0, Wall / 2.0]));
      --  The handle: an upright loop of three bars off the +x side.
      M.Parts.Append (Part'(Kind => Block, Pose => At_Pose ([Radius + Height / 4.0, 0.0, Height * 0.75]),
                       Sizes => [Height / 4.0, Wall, Wall]));
      M.Parts.Append (Part'(Kind => Block, Pose => At_Pose ([Radius + Height / 4.0, 0.0, Height * 0.25]),
                       Sizes => [Height / 4.0, Wall, Wall]));
      M.Parts.Append (Part'(Kind => Block, Pose => At_Pose ([Radius + Height / 2.0, 0.0, Height / 2.0]),
                       Sizes => [Wall, Wall, Height / 4.0 + Wall]));
      return M;
   end Cup;

   function Inside_Part (P : Part; Q : Vec3; Margin : Real) return Boolean is
      L : constant Vec3 := Inverse (P.Pose) * Q;
      S : constant Vec3 := P.Sizes;
   begin
      case P.Kind is
         when Block =>
            return abs L (1) < S (1) - Margin and then abs L (2) < S (2) - Margin and then abs L (3) < S (3) - Margin;
         when Cylinder =>
            return Sqrt (L (1) ** 2 + L (2) ** 2) < S (1) - Margin and then abs L (3) < S (3) - Margin;
         when Tube =>
            declare
               R : constant Real := Sqrt (L (1) ** 2 + L (2) ** 2);
            begin
               return R < S (1) - Margin and then R > S (2) + Margin and then abs L (3) < S (3) - Margin;
            end;
      end case;
   end Inside_Part;

   function Inside (M : Model; P : Vec3) return Boolean is
     (for some X of M.Parts => Inside_Part (X, P, 0.0));

   function Volume (P : Part) return Real is
     (case P.Kind is
         when Block    => 8.0 * P.Sizes (1) * P.Sizes (2) * P.Sizes (3),
         when Cylinder => 2.0 * Pi * P.Sizes (1) ** 2 * P.Sizes (3),
         when Tube     => 2.0 * Pi * (P.Sizes (1) ** 2 - P.Sizes (2) ** 2) * P.Sizes (3));

   function Centre (M : Model) return Vec3 is
      Sum : Vec3 := Zero3;
      V   : Real := 0.0;
   begin
      for P of M.Parts loop
         Sum := Sum + Volume (P) * P.Pose.Translation;
         V := V + Volume (P);
      end loop;
      return Sum / V;
   end Centre;

   function Steps (Span, Pitch : Real) return Positive is (Positive'Max (1, Natural (Real'Ceiling (Span / Pitch))));

   --  The faces of one part, in the part's frame.
   procedure Faces (P : Part; Pitch : Real; Into : in out Sample_Vectors.Vector) is
      S : constant Vec3 := P.Sizes;
      procedure Add (Q, N : Vec3) is
      begin
         Into.Append (Sample'(Point => Q, Normal => N));
      end Add;
      procedure Ring_Of (Radius, Z : Real; Up : Real; Outward : Real) is
         N_Around : constant Positive := Steps (2.0 * Pi * Radius, Pitch);
      begin
         for K in 0 .. N_Around - 1 loop
            declare
               A : constant Real := 2.0 * Pi * Real (K) / Real (N_Around);
            begin
               Add ([Radius * Cos (A), Radius * Sin (A), Z],
                    (if Up /= 0.0 then [0.0, 0.0, Up] else [Outward * Cos (A), Outward * Sin (A), 0.0]));
            end;
         end loop;
      end Ring_Of;
      procedure Disc (Inner, Outer, Z, Up : Real) is
         N_Radial : constant Positive := Steps (Outer - Inner, Pitch);
      begin
         for K in 0 .. N_Radial loop
            declare
               R : constant Real := Inner + (Outer - Inner) * Real (K) / Real (N_Radial);
            begin
               if R > 0.0 then
                  Ring_Of (R, Z, Up, 0.0);
               else
                  Add ([0.0, 0.0, Z], [0.0, 0.0, Up]);
               end if;
            end;
         end loop;
      end Disc;
      procedure Wall (Radius, Outward : Real) is
         N_Up : constant Positive := Steps (2.0 * S (3), Pitch);
      begin
         for K in 0 .. N_Up loop
            Ring_Of (Radius, -S (3) + 2.0 * S (3) * Real (K) / Real (N_Up), 0.0, Outward);
         end loop;
      end Wall;
   begin
      case P.Kind is
         when Block =>
            for Axis in 1 .. 3 loop
               declare
                  A1 : constant Positive := (if Axis = 1 then 2 else 1);
                  A2 : constant Positive := (if Axis = 3 then 2 else 3);
                  N1 : constant Positive := Steps (2.0 * S (A1), Pitch);
                  N2 : constant Positive := Steps (2.0 * S (A2), Pitch);
               begin
                  for Sign in -1 .. 1 loop
                     if Sign /= 0 then
                        for I in 0 .. N1 loop
                           for J in 0 .. N2 loop
                              declare
                                 Q : Vec3 := Zero3;
                                 N : Vec3 := Zero3;
                              begin
                                 Q (Axis) := Real (Sign) * S (Axis);
                                 Q (A1) := -S (A1) + 2.0 * S (A1) * Real (I) / Real (N1);
                                 Q (A2) := -S (A2) + 2.0 * S (A2) * Real (J) / Real (N2);
                                 N (Axis) := Real (Sign);
                                 Add (Q, N);
                              end;
                           end loop;
                        end loop;
                     end if;
                  end loop;
               end;
            end loop;
         when Cylinder =>
            Wall (S (1), 1.0);
            Disc (0.0, S (1), S (3), 1.0);
            Disc (0.0, S (1), -S (3), -1.0);
         when Tube =>
            Wall (S (1), 1.0);
            Wall (S (2), -1.0);
            Disc (S (2), S (1), S (3), 1.0);
            Disc (S (2), S (1), -S (3), -1.0);
      end case;
   end Faces;

   function Thing_Of
     (Id      : Thing_Id;
      M       : Model;
      Place   : Rigid;
      Pitch   : Real;
      Sigma   : Real;
      Support : Surface_Id'Base) return Thing_State
   is
      T : Thing_State;
   begin
      T.Id := Id;
      T.Sigma := Sigma;
      --  A normal estimated from neighbours one pitch apart, each Sigma off.
      T.Normal_Sigma := Sigma / Pitch;
      T.Pitch := Pitch;
      T.Support := Support;
      T.Seen := True;
      T.Height := (Value => 0.0, Sigma => Sigma);
      T.Centre := (Mean => Place * Centre (M), Covariance => (Sigma * Sigma) * Identity3);
      for P of M.Parts loop
         declare
            Local : Sample_Vectors.Vector;
         begin
            Faces (P, Pitch, Local);
            for L of Local loop
               declare
                  Q : constant Vec3 := P.Pose * L.Point;
                  N : constant Vec3 := Rotate (P.Pose, L.Normal);
               begin
                  if not (for some Other of M.Parts => Other /= P and then Inside_Part (Other, Q, Pitch / 2.0))
                    and then not (Q (3) <= Pitch / 2.0 and then N (3) < 0.0)
                  then
                     T.Samples.Append (Sample'(Point => Place * Q, Normal => Rotate (Place, N)));
                  end if;
               end;
            end loop;
         end;
      end loop;
      return T;
   end Thing_Of;

   function Floor (Id : Surface_Id; Place : Rigid; Sigma : Real) return Surface_State is
     ((Id     => Id,
       Point  => (Mean => Place.Translation, Covariance => (Sigma * Sigma) * Identity3),
       Normal => (Unit_Vector => Rotate (Place, [0.0, 0.0, 1.0]), Sigma => Sigma)));

   function Hand_Of (Arm : Arm_Id; Hand : Hand_Id; Depth, Sigma : Real) return Hand_State is
     ((Id => Hand, Arm => Arm, Lobes => Lobe_Vectors.Empty_Vector, Depth => (Value => Depth, Sigma => Sigma),
       Fraction => (Value => 0.0, Sigma => Sigma)));

   function Gripper (Arm : Arm_Id; Hand : Hand_Id; Opening, Width, Thickness, Depth, Sigma : Real)
     return Hand_State
   is
      H : Hand_State := Hand_Of (Arm, Hand, Depth, Sigma);
   begin
      for Sign in -1 .. 1 loop
         if Sign /= 0 then
            H.Lobes.Append (Lobe_State'(Open_Tip   => [Real (Sign) * (Opening + Thickness) / 2.0, 0.0, Depth],
                             Closed_Tip => [Real (Sign) * Thickness / 2.0, 0.0, Depth],
                             Tip_Sigma  => Sigma, Width => Width, Thickness => Thickness));
         end if;
      end loop;
      return H;
   end Gripper;

   function Five_Lobes (Arm : Arm_Id; Hand : Hand_Id; Radius, Width, Thickness, Depth, Sigma : Real)
     return Hand_State
   is
      H : Hand_State := Hand_Of (Arm, Hand, Depth, Sigma);
   begin
      for K in 0 .. 4 loop
         declare
            A : constant Real := 2.0 * Pi * Real (K) / 5.0;
            D : constant Vec3 := [Cos (A), Sin (A), 0.0];
         begin
            H.Lobes.Append (Lobe_State'(Open_Tip   => (Radius + Thickness / 2.0) * D + [0.0, 0.0, Depth],
                             Closed_Tip => (Width + Thickness / 2.0) * D + [0.0, 0.0, Depth],
                             Tip_Sigma  => Sigma, Width => Width, Thickness => Thickness));
         end;
      end loop;
      return H;
   end Five_Lobes;

   function Arm_Of (Id : Arm_Id; Tool : Rigid; Sigma : Real) return Arm_State is
     ((Id          => Id,
       Tool        => (Pose => Tool, Position_Covariance => (Sigma * Sigma) * Identity3,
                       Rotation_Covariance => (Sigma * Sigma) * Identity3),
       Step        => (Value => 3.0 * Sigma, Sigma => Sigma),
       Turn_Step   => (Value => 3.0 * Sigma, Sigma => Sigma),
       Lag         => (Value => 2.0, Sigma => 0.5),
       Rate        => (Value => 0.5, Sigma => 0.05),
       Surface     => Sample_Vectors.Empty_Vector,
       Carries_Eye => True,
       Carries_All => False));

   function Plate_Arm (Id : Arm_Id; Tool : Rigid; Radius, Pitch, Sigma : Real) return Arm_State is
      A : Arm_State := Arm_Of (Id, Tool, Sigma);
      N : constant Positive := Steps (Radius, Pitch);
   begin
      for K in 0 .. N loop
         declare
            R      : constant Real := Radius * Real (K) / Real (N);
            Around : constant Positive := Steps (2.0 * Pi * R, Pitch);
         begin
            for J in 0 .. Around - 1 loop
               declare
                  B : constant Real := 2.0 * Pi * Real (J) / Real (Around);
               begin
                  A.Surface.Append (Sample'(Point => [R * Cos (B), R * Sin (B), 0.0], Normal => [0.0, 0.0, 1.0]));
               end;
            end loop;
         end;
      end loop;
      return A;
   end Plate_Arm;

end Driver.Action.Snapshots.Tests;
