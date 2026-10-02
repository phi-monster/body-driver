with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Tests;
with Driver.Uncertain;

package body Driver.Action.Contact.Wrench.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;

   --  A box of half sizes A, B, C standing on a surface through the origin
   --  with normal Up, centred above the origin. Everything is expressed in a
   --  frame turned by Frame, so the same physics must come out whatever way
   --  the world is turned.
   type Box is record
      A, B, C : Real;
      Frame   : Rigid := Identity;
      Pitch   : Real;
   end record;

   function World (X : Box; P : Vec3) return Vec3 is (X.Frame * P);
   function Dir (X : Box; V : Vec3) return Vec3 is (Rotate (X.Frame, V));

   function Up (X : Box) return Vec3 is (Dir (X, [0.0, 0.0, 1.0]));
   function Centre (X : Box) return Vec3 is (World (X, [0.0, 0.0, X.C]));

   function Bottom (X : Box) return Footing is
      Points : Point_Vectors.Vector;
      Nx : constant Natural := Natural (Real'Floor (2.0 * X.A / X.Pitch));
      Ny : constant Natural := Natural (Real'Floor (2.0 * X.B / X.Pitch));
   begin
      for I in 0 .. Nx loop
         for J in 0 .. Ny loop
            Points.Append (World (X, [-X.A + Real (I) * 2.0 * X.A / Real (Nx),
                                      -X.B + Real (J) * 2.0 * X.B / Real (Ny), 0.0]));
         end loop;
      end loop;
      return Footing_Of (Points, World (X, Zero3), Up (X), X.Pitch);
   end Bottom;

   function Side_Pair (X : Box) return Touch_Vectors.Vector is
      T : Touch_Vectors.Vector;
   begin
      T.Append (Touch'(Point => World (X, [-X.A, 0.0, X.C]), Inward => Dir (X, [1.0, 0.0, 0.0]), Patch => 0.0,
                 Tension => False));
      T.Append (Touch'(Point => World (X, [X.A, 0.0, X.C]), Inward => Dir (X, [-1.0, 0.0, 0.0]), Patch => 0.0,
                 Tension => False));
      return T;
   end Side_Pair;

   Upright : constant Box := (A => 0.03, B => 0.02, C => 0.05, Frame => Identity, Pitch => 0.005);
   Turned  : constant Box :=
     (A => 0.03, B => 0.02, C => 0.05, Pitch => 0.005,
      Frame => (Rotation => Exp ([0.7, -1.9, 0.4]), Translation => [3.0, -1.0, 2.0]));

   type Box_Array is array (Positive range <>) of Box;
   Both_Frames : constant Box_Array := [Upright, Turned];

   procedure Squeeze_Off_Surface is
   begin
      for X of Both_Frames loop
         declare
            Off : constant Twist := Slide (Up (X));
            Ok  : constant Answer := Need (Side_Pair (X), Bottom (X), Off, Centre (X), Up (X), 0.5);
            No  : constant Answer := Need (Side_Pair (X), Bottom (X), Off, Centre (X), Up (X), 0.0);
         begin
            --  Two opposed touches each carry half the weight by friction:
            --  normal forces of 1 / (2 mu) each, 1 / mu in all. The polygon's
            --  edge need not point straight along the weight once the frame
            --  turns, hence the polygon's own error as tolerance.
            Check (Ok.Why = None, "an opposed pair with friction cannot raise a box off its surface");
            Check_Close (Ok.Force, 2.0, 2.0 * Driver.Conventions.Unchanged_Fraction, "least normal force of a squeeze");
            Check (No.Why = Unbalanced, "a frictionless squeeze is said to raise a box");
         end;
      end loop;
   end Squeeze_Off_Surface;

   procedure Slide_Along_Surface is
      Mu : constant Real := 0.5;
      --  Pushed from behind at its centre's height, the surface's friction
      --  mu L opposes the slide and the touch's own friction f bears part of
      --  the weight, L = 1 - f, N = mu L. On a flat box f reaches mu N, so
      --  N = mu / (1 + mu^2). On a tall one the surface's friction would tip
      --  it over the front edge unless f stays below (a - c mu) / (2 a - c mu),
      --  so N = mu a / (2 a - c mu).
      Flat_Box : constant Box := (Turned with delta C => 0.01);
      Tall_Box : constant Box := Turned;
      function Behind (X : Box) return Answer is
         T : Touch_Vectors.Vector;
      begin
         T.Append (Touch'(Point => World (X, [-X.A, 0.0, X.C]), Inward => Dir (X, [1.0, 0.0, 0.0]),
                          Patch => 0.0, Tension => False));
         return Need (T, Bottom (X), Slide (Dir (X, [1.0, 0.0, 0.0])), Centre (X), Up (X), Mu);
      end Behind;
      Flat_Answer : constant Answer := Behind (Flat_Box);
      Tall_Answer : constant Answer := Behind (Tall_Box);
      Tolerance   : constant Real := 2.0 * Driver.Conventions.Unchanged_Fraction * Mu;
   begin
      Check (Flat_Answer.Why = None and then Tall_Answer.Why = None,
             "one touch behind a box cannot slide it along its surface");
      Check_Close (Flat_Answer.Force, Mu / (1.0 + Mu * Mu), Tolerance, "least force to slide a flat box");
      Check_Close (Tall_Answer.Force, Mu * Tall_Box.A / (2.0 * Tall_Box.A - Tall_Box.C * Mu), Tolerance,
                   "least force to slide a tall box without tipping it");
   end Slide_Along_Surface;

   procedure Surface_In_The_Way is
      A : constant Answer :=
        Need (Side_Pair (Turned), Bottom (Turned), Slide (-Up (Turned)), Centre (Turned), Up (Turned), 0.5);
   begin
      Check (A.Why = Footing_In_Way, "moving a thing into the surface it lies on is reported " & A.Why'Image);
   end Surface_In_The_Way;

   procedure Spin_In_Place is
      --  Spinning in place, the surface's friction has no net force, only a
      --  moment: one touch at the side presses the box sideways with nothing
      --  to balance it, while an opposed pair balances its own normal forces
      --  and makes the moment by friction.
      X    : constant Box := Turned;
      Spin : constant Twist := Rotation (Up (X), 1.0, Centre (X));
      One  : Touch_Vectors.Vector;
   begin
      One.Append (Side_Pair (X).First_Element);
      Check (Need (Side_Pair (X), Bottom (X), Spin, Centre (X), Up (X), 0.5).Why = None,
             "an opposed pair with friction cannot rotate a box about its up");
      Check (Need (One, Bottom (X), Spin, Centre (X), Up (X), 0.5).Why = Unbalanced,
             "a single side touch is said to rotate a box in place, against the surface's friction");
   end Spin_In_Place;

   procedure Least_Friction_Of_A_Wedge is
      --  Touches pressing inward and down at Alpha below the horizontal: off
      --  the surface they need friction of at least tan Alpha.
      X     : constant Box := Turned;
      Alpha : constant Real := 0.3;
      Pair  : Touch_Vectors.Vector;
      Sigma : constant Real := 0.001;
   begin
      Pair.Append (Touch'(Point => World (X, [-X.A, 0.0, X.C]), Inward => Dir (X, [Cos (Alpha), 0.0, -Sin (Alpha)]),
                    Patch => 0.0, Tension => False));
      Pair.Append (Touch'(Point => World (X, [X.A, 0.0, X.C]), Inward => Dir (X, [-Cos (Alpha), 0.0, -Sin (Alpha)]),
                    Patch => 0.0, Tension => False));
      declare
         Mu : constant Real := Least_Friction (Pair, Bottom (X), Slide (Up (X)), Centre (X), Up (X), Sigma);
      begin
         --  The answer is at most one resolution of friction angle above the
         --  truth, and never below it (the polygon is inscribed).
         Check (Need (Pair, Bottom (X), Slide (Up (X)), Centre (X), Up (X), Mu).Why = None,
                "the least friction returned is not enough for the motion");
         Check (Mu >= Tan (Alpha), "least friction found below the true threshold");
         Check (Arctan (Mu) - Alpha <= Sigma + Driver.Conventions.Unchanged_Fraction,
                "least friction more than one resolution above tan alpha: " & Mu'Image);
      end;
   end Least_Friction_Of_A_Wedge;

   procedure Suction_From_Above is
      X   : constant Box := Turned;
      Cup : Touch_Vectors.Vector;
      Pad : Touch_Vectors.Vector;
   begin
      Cup.Append (Touch'(Point => World (X, [0.0, 0.0, 2.0 * X.C]), Inward => Dir (X, [0.0, 0.0, -1.0]), Patch => 0.01,
                   Tension => True));
      Pad.Append (Touch'(Point => World (X, [0.0, 0.0, 2.0 * X.C]), Inward => Dir (X, [0.0, 0.0, -1.0]), Patch => 0.01,
                   Tension => False));
      Check (Need (Cup, Bottom (X), Slide (Up (X)), Centre (X), Up (X), 0.5).Why = None,
             "a touch that transmits tension cannot raise a box from its top");
      Check (Need (Pad, Bottom (X), Slide (Up (X)), Centre (X), Up (X), 0.5).Why = Unbalanced,
             "a touch that can only press is said to raise a box from its top");
   end Suction_From_Above;

   procedure Rests_With_Margin is
      use Driver.Uncertain;
      X     : constant Box := Turned;
      Sigma : constant Real := 0.002;
      Cov   : constant Mat3 := (Sigma * Sigma) * Identity3;
      function At_X (Offset : Real) return Point_Estimate is
        ((Mean => World (X, [Offset, 0.0, X.C]), Covariance => Cov));
      Middle   : constant Rest_Answer := Rests (Bottom (X), At_X (0.0), Up (X), 0.0);
      Near_Rim : constant Rest_Answer := Rests (Bottom (X), At_X (X.A - Sigma), Up (X), 0.0);
      Beyond   : constant Rest_Answer := Rests (Bottom (X), At_X (X.A + Sigma), Up (X), 0.0);
   begin
      Check (Middle.Rests, "a box centred on its foot is said not to rest");
      Check_Close (Middle.Margin, X.B, 1.0e-9, "margin to the nearest edge of the foot");
      Check (not Near_Rim.Rests, "a centre one sigma inside the rim is trusted to rest");
      Check (Near_Rim.Margin > 0.0, "a centre inside the foot has a negative margin");
      Check (not Beyond.Rests and then Beyond.Margin < 0.0, "a centre beyond the foot is said to rest");
   end Rests_With_Margin;

   procedure Register is
   begin
      Driver.Tests.Register ("action.wrench.squeeze", "an opposed pair is judged without friction, or by the frame",
                             Squeeze_Off_Surface'Access);
      Driver.Tests.Register ("action.wrench.slide", "sliding friction of the surface is left out of a slide along it",
                             Slide_Along_Surface'Access);
      Driver.Tests.Register ("action.wrench.in_way", "a motion into the supporting surface is reported as possible",
                             Surface_In_The_Way'Access);
      Driver.Tests.Register ("action.wrench.spin", "the surface's friction moment is missing from a rotation about up",
                             Spin_In_Place'Access);
      Driver.Tests.Register ("action.wrench.least_friction", "the least friction of a contact set is misjudged",
                             Least_Friction_Of_A_Wedge'Access);
      Driver.Tests.Register ("action.wrench.tension", "a touch that cannot transmit tension is used to draw a thing",
                             Suction_From_Above'Access);
      Driver.Tests.Register ("action.wrench.rests", "a thing is let go where its centre may lie outside its foot",
                             Rests_With_Margin'Access);
   end Register;

end Driver.Action.Contact.Wrench.Tests;
