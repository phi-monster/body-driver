with Ada.Numerics;
with Ada.Numerics.Float_Random;
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

   procedure Degenerate_Balance_Ends is
      --  A program the search on a bar met: two opposed touches at its bottom
      --  edge, their normals turned the worst way the measurement allows,
      --  lifting it. The moment rows have nothing on their right-hand side,
      --  and taking the most improving column for every pivot goes round a
      --  cycle there for ever. It must end, at the force Bland's rule alone
      --  finds.
      T : Touch_Vectors.Vector;
      A : Answer;
   begin
      T.Append (Touch'(Point => [5.13694762170866E-02, 3.25757104785479E-02, 0.0],
                       Inward => [3.72025551942260E-01, -8.79923176281257E-01, 2.95520206661340E-01],
                       Patch => 7.5E-03, Tension => False));
      T.Append (Touch'(Point => [5.91578430632596E-02, 1.41544905984902E-02, 0.0],
                       Inward => [-3.72025551942260E-01, 8.79923176281257E-01, -2.95520206661340E-01],
                       Patch => 7.5E-03, Tension => False));
      A := Need (T, No_Footing, Slide ([0.0, 0.0, 1.0]), [0.0, 0.0, 0.01], [0.0, 0.0, 1.0], 3.16836573328104E-01);
      Check (A.Why = None, "a lift the touches can make is judged " & A.Why'Image);
      Check_Close (A.Force, 1.59810185373185E+03, 1.0E-6, "the least force of a degenerate balance");
   end Degenerate_Balance_Ends;

   procedure Noise_Does_Not_Decide is
      --  Three touches round an upright cylinder, at three heights and a
      --  third of a turn about, that are to lift it off the table it stands
      --  on: a balance they cannot make. Their program is degenerate, its
      --  right-hand sides at a vertex are the rounding noise of the
      --  eliminations, and the rows Bland's rule must choose among by their
      --  basic columns were chosen among by that noise: with every number of
      --  the touches moved by a few units of its last place, one balance in
      --  ten went round a cycle for ever, as one balance of the search of a
      --  symmetric hand did on x86-64, where the same numbers differ in the
      --  last place. Whatever the last bits, every balance ends, and says the
      --  same.
      Up    : constant Vec3 := [0.0, 0.0, 1.0];
      Where : constant array (1 .. 3) of Vec3 :=
        [[0.02, 0.0, 0.03], [-0.0070920977408507092, 0.018700324853708296, 0.065],
         [-0.011361294934623119, -0.016459677317873126, 0.0]];
      Along : constant array (1 .. 3) of Vec3 :=
        [[-1.0, 0.0, 0.0], [0.35460488704253545, -0.93501624268541483, 0.0],
         [0.56806474673115592, 0.82298386589365635, 0.0]];
      Patch : constant Real := 0.006;
      Mu    : constant Real := 0.91947909715639886;
      Radius : constant Real := 0.02;
      Foot_Points : constant := 25;
      Unit  : constant Real := Real'Epsilon;
      Runs  : constant := 100;
      Generator : Ada.Numerics.Float_Random.Generator;
      Foot  : Point_Vectors.Vector;
      Wrong : Natural := 0;   --  balances that were not unbalanced
      Noise : Real_Array (1 .. 20);
   begin
      for K in 0 .. Foot_Points - 1 loop
         declare
            Angle : constant Real := 2.0 * Ada.Numerics.Pi * Real (K) / Real (Foot_Points);
         begin
            Foot.Append (Vec3'[Radius * Cos (Angle), Radius * Sin (Angle), 0.0]);
         end;
      end loop;
      Ada.Numerics.Float_Random.Reset (Generator, 1);
      for Scale of Real_Array'[1.0, 4.0, 16.0] loop
         for Run in 1 .. Runs loop
            --  The noise is drawn in this order, whatever evaluates first.
            for K in Noise'Range loop
               Noise (K) := 2.0 * Real (Ada.Numerics.Float_Random.Random (Generator)) - 1.0;
            end loop;
            declare
               function Moved (X : Real; K : Positive) return Real is (X * (1.0 + Scale * Unit * Noise (K)));
               T : Touch_Vectors.Vector;
            begin
               for I in Where'Range loop
                  T.Append (Touch'(Point   => [Moved (Where (I) (1), 6 * I - 5), Moved (Where (I) (2), 6 * I - 4),
                                               Moved (Where (I) (3), 6 * I - 3)],
                                   Inward  => [Moved (Along (I) (1), 6 * I - 2), Moved (Along (I) (2), 6 * I - 1),
                                               Moved (Along (I) (3), 6 * I)],
                                   Patch   => Moved (Patch, 19),
                                   Tension => False));
               end loop;
               if Need (T, Footing_Of (Foot, Zero3, Up, 0.005), Slide (Up), [0.0, 0.0, 0.04], Up, Moved (Mu, 20)).Why
                 /= Unbalanced
               then
                  Wrong := Wrong + 1;
               end if;
            end;
         end loop;
      end loop;
      Check (Wrong = 0, Wrong'Image & " balances of touches the last bits of whose numbers were moved did not say "
             & "unbalanced");
   end Noise_Does_Not_Decide;

   procedure Register is
   begin
      Driver.Tests.Register ("action.wrench.noise", "the rounding noise of a symmetric contact set's program decides "
                             & "which row leaves, and the balance goes round a cycle for ever or says another thing",
                             Noise_Does_Not_Decide'Access);
      Driver.Tests.Register ("action.wrench.cycle", "a pivot that does not move the solution is taken by the most "
                             & "improving column, which goes round a cycle for ever", Degenerate_Balance_Ends'Access);
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
