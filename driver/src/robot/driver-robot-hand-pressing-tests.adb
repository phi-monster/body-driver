with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Clock;
with Driver.Geometry;
with Driver.Log;
with Driver.Robot.Hand.Aims;
with Driver.Robot.Kinematics.Fit;
with Driver.Tests;

package body Driver.Robot.Hand.Pressing.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use type Driver.Robot.Motion.Plan_Status;

   package Fit renames Driver.Robot.Kinematics.Fit;
   package Motion renames Driver.Robot.Motion;

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      --  The generator returns [0, 1] with 1 included: U1 is drawn on (0, 1].
      U1 : Real;
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      loop
         U1 := Real (Ada.Numerics.Float_Random.Random (Gen));
         exit when U1 > 0.0;
      end loop;
      return Sqrt (-2.0 * Ada.Numerics.Long_Elementary_Functions.Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   ---------------------------------------------------------------------------
   --  A body of two arms, each of six joints carrying an eye, each with a
   --  two-lobe hand. The arms' fits are exact (the truth of the rig) with
   --  their uncertainty stated: what is under test is which frame the hand
   --  works in, not how well an arm is fitted. The second arm stands in the
   --  world, far from the first and turned, in its own unit; the first arm's
   --  frame is the world.

   Joint_Count : constant := 6;

   --  Three slides along the reference eye's axes, then three turns about
   --  them through the eye: the tool's place is the slides' readings, its
   --  turn the turns'.
   Truth : constant Fit.Joint_Array (1 .. Joint_Count) :=
     [(W => [1.0, 0.0, 0.0], P => Zero3, C => 1.0, Slide => True),
      (W => [0.0, 1.0, 0.0], P => Zero3, C => 1.0, Slide => True),
      (W => [0.0, 0.0, 1.0], P => Zero3, C => 1.0, Slide => True),
      (W => [1.0, 0.0, 0.0], P => Zero3, C => 1.0, Slide => False),
      (W => [0.0, 1.0, 0.0], P => Zero3, C => 1.0, Slide => False),
      (W => [0.0, 0.0, 1.0], P => Zero3, C => 1.0, Slide => False)];

   --  Each arm's table in its own frame, its normal towards the eye (the
   --  arms' tables differ in the frames they are seen in).
   Table_Normal : constant array (1 .. 2) of Vec3 := [Unit ([0.0, -0.6, -0.8]), Unit ([0.12, -0.35, -1.0])];
   Table_Offset : constant array (1 .. 2) of Real := [-0.20, -0.22];   --  Normal * X on the table
   Table_Sigma  : constant Real := 5.0e-4;   --  its offset's, at its centre
   Tilt_Sigma   : constant Real := 3.0e-4;   --  and its tilt's

   function Table_Of (A : Positive) return Driver.Geometry.Plane_Estimate is
      N : constant Vec3 := Table_Normal (A);
      T : constant Vec3 := Driver.Robot.Hand.Aims.Any_Across (N);
   begin
      return (Centre       => (Table_Offset (A) / N (3)) * [0.0, 0.0, 1.0],
              Normal       => N,
              Tangent_1    => T,
              Tangent_2    => Cross (N, T),
              Offset_Sigma => Table_Sigma,
              Tilt_11      => Tilt_Sigma ** 2,
              Tilt_12      => 0.0,
              Tilt_22      => Tilt_Sigma ** 2,
              Points       => 100,
              Scatter      => 1.0);
   end Table_Of;

   --  The hand of either arm, in its tool frame: two lobes, apart when open
   --  and nearly touching when closed, ahead of the eye at the tool's origin.
   type Tip_Table is array (1 .. 2, Opening) of Vec3;
   Tips_True : constant Tip_Table :=
     [1 => [Open => [0.04, 0.03, 0.13], Closed_Empty => [0.012, 0.03, 0.135]],
      2 => [Open => [-0.04, 0.03, 0.13], Closed_Empty => [-0.012, 0.03, 0.135]]];

   Contact_Sigma : constant Real := 2.0e-4;   --  how far a press lands off the table
   Closer_Noise  : constant Real := 1.0e-3;

   --  Where the second arm stands in the world as the body has it.
   type Standing is (Placed, Misplaced, Unplaced);
   --  Misplaced: turned by a hundredth of a radian, a unit away and five per
   --  cent too large, far beyond what its placement says it is sure of.

   Placement_2 : constant Rigid := (Rotation => Exp ([0.4, -0.8, 0.3]), Translation => [3.0, -1.0, 0.5]);
   Unit_2      : constant Real := 1.4;

   function Fitted_Arm (A : Positive; How : Standing) return Arm_Fit is
      R     : Arm_Fit;
      Terms : constant Natural := Fit.Terms (Joint_Count);
   begin
      R.Fitted := True;
      for J of Truth loop
         R.Joints.Append (Joint_Fit'(W => J.W, P => J.P, C => J.C, Slide => J.Slide));
         R.Reference.Append (0.0);
      end loop;
      R.Lens := (Fx => 400.0, Fy => 400.0, Cx => 320.0, Cy => 240.0, K1 => 0.0, K2 => 0.0);
      R.Sigma_Px := 0.1;
      --  Every term of the fit uncertain by a few parts in ten thousand.
      for Row in 1 .. Terms loop
         for Column in 1 .. Terms loop
            R.Covariance.Append (if Row /= Column then 0.0 elsif Row <= Fit.Lens_Terms then 1.0e-8 else 9.0e-8);
         end loop;
      end loop;
      R.Table := Table_Of (A);
      R.Placed := A = 1 or else How /= Unplaced;
      if A = 1 then
         for I in 1 .. 36 loop
            R.Placement_Covariance.Append (0.0);
         end loop;
      else
         R.Placement := Placement_2;
         R.Scale := Unit_2;
         R.Scale_Sigma := 1.0e-2;
         for Row in 1 .. 6 loop
            for Column in 1 .. 6 loop
               R.Placement_Covariance.Append (if Row /= Column then 0.0 elsif Row <= 3 then 1.0e-6 else 1.0e-4);
            end loop;
         end loop;
         if How = Misplaced then
            R.Placement := (Rotation    => Exp ([0.0, 0.0, 0.01]) * Placement_2.Rotation,
                            Translation => Placement_2.Translation + [1.0, 0.0, 0.0]);
            R.Scale := 1.05 * Unit_2;
         end if;
      end if;
      return R;
   end Fitted_Arm;

   --  Groups 1 and 2 are the arms, 3 and 4 their closers; eye 1 rides on arm
   --  1 and eye 2 on arm 2.
   procedure Build (M : in out Model; How : Standing) is
   begin
      for G in 1 .. 4 loop
         M.Groups.Append
           (Group_Stream'(Size => (if G <= 2 then Joint_Count else 1), Commandable => True, others => <>));
         M.Graph.Roles.Append (if G <= 2 then Arm else Closer);
         M.Graph.Arm_Of.Append (if G mod 2 = 1 then 1 else 2);
      end loop;
      for A in 1 .. 2 loop
         M.Graph.Arms.Append (Group_Id (A));
         M.Graph.Mounts.Append (Mount'(Kind => Arm_Carried, Arm => Arm_Id (A)));
         M.Kinematics.Append (Arm_Evidence'(Arm => Arm_Id (A), Group => Group_Id (A), Eye => Eye_Id (A),
                                            Result => Fitted_Arm (A, How), others => <>));
      end loop;
      for G in 1 .. 4 loop
         for C in 1 .. (if G <= 2 then Joint_Count else 1) loop
            M.Noise.Append (if G <= 2 then 1.0e-6 else Closer_Noise);
            M.Noise_Freedom.Append (100);
         end loop;
      end loop;
   end Build;

   function Observed (Beat : Natural; First, Second : Real_Array; Closer : Real) return Observation is
      O : Observation;
   begin
      O.Beat := Driver.Clock.Beat (Beat);
      O.Readings.Append (First);
      O.Readings.Append (Second);
      O.Readings.Append (Real_Array'[1 => 0.0]);
      O.Readings.Append (Real_Array'[1 => Closer]);
      return O;
   end Observed;

   ---------------------------------------------------------------------------
   --  A press as the arm makes it: the tool's six readings with the lobe's
   --  tip landing on the table, the lobe that leads.

   type Press_Shape is record
      Lobe         : Positive;
      Which        : Opening;
      Tilt         : Real;   --  from pressing straight along the lobe's line of sight
      Azimuth      : Real;   --  towards which side
      Along, Across : Real;   --  where on the table, from its centre
   end record;

   --  The readings at which the tool stands Away_By above the contact, and
   --  at it. False when another lobe would have touched first.
   procedure Contact
     (A : Positive; P : Press_Shape; Away_By : Real; Start, Rest : out Real_Array; Leads : out Boolean)
   is
      N      : constant Vec3 := Table_Normal (A);
      Down   : constant Vec3 := -N;
      X      : constant Vec3 := Tips_True (P.Lobe, P.Which);
      R0     : constant Mat3 := Driver.Robot.Hand.Aims.Turned_About (Identity, Zero3, Unit (X), Down).Rotation;
      Side   : constant Vec3 := Driver.Robot.Hand.Aims.Any_Across (Down);
      R      : constant Mat3 := Exp (P.Tilt * (Cos (P.Azimuth) * Side + Sin (P.Azimuth) * Cross (Down, Side))) * R0;
      Table  : constant Driver.Geometry.Plane_Estimate := Table_Of (A);
      Lands  : constant Vec3 :=
        Table.Centre + P.Along * Table.Tangent_1 + P.Across * Table.Tangent_2 + Contact_Sigma * Gaussian * N;
      Place  : constant Vec3 := Lands - R * X;
      Other  : constant Vec3 := R * Tips_True (3 - P.Lobe, P.Which) + Place;
   begin
      --  The Euler angles of R = Rx Ry Rz.
      Rest := [Place (1), Place (2), Place (3), Arctan (-R (2, 3), R (3, 3)), Arcsin (R (1, 3)),
               Arctan (-R (1, 2), R (1, 1))];
      Start := Rest;
      for K in 1 .. 3 loop
         Start (K) := Rest (K) - Away_By * Down (K);
      end loop;
      Leads := N * Other - Table_Offset (A) > 0.0;
   end Contact;

   --  The hand of arm A given to H as its sweep would leave it: the lobes'
   --  tips seen along lines of sight from the eye at both openings.
   procedure Give_Hand (H : in out Hands; A : Positive) is
      Rows : Sight_Rows (1 .. 2);
   begin
      for L in 1 .. 2 loop
         for W in Opening loop
            Rows (L) (W) :=
              (Known => True,
               Pixel => (U => 0.0, V => 0.0),
               Ray   => (Origin    => (Mean => Zero3, Covariance => [others => [others => 0.0]]),
                         Direction => (Unit_Vector => Unit (Tips_True (L, W)), Sigma => 2.5e-4)));
         end loop;
      end loop;
      Adopt (H, Group_Id (2 + A), Arm_Id (A), Eye_Id (A), [1 => 1.0], [1 => 0.0], Rows);
   end Give_Hand;

   Pushed_Back : constant Real := 0.04;   --  where a press starts, above the table

   --  Presses of every lobe at both openings, fed to the hand beat by beat as
   --  the body would judge them: still above the table, blocked, still on it.
   --  Made is how many were made, First_Tip how many had been when the first
   --  lobe's tip at the open opening was first known (0: never).
   procedure Press_Everything
     (H : in out Hands; M : Model; A : Positive; Made : out Natural; First_Tip : out Natural)
   is
      Beat : Natural := 0;
      Id   : constant Hand_Id := 1;
   begin
      Made := 0;
      First_Tip := 0;
      Ada.Numerics.Float_Random.Reset (Gen, 41);
      for L in 1 .. 2 loop
         for W in Opening loop
            for K in 0 .. 11 loop
               declare
                  P      : constant Press_Shape :=
                    (Lobe    => L,
                     Which   => W,
                     Tilt    => 0.25 * Real (K mod 3),
                     Azimuth => Ada.Numerics.Pi * Real (K mod 4) / 2.0,
                     Along   => -0.03 + 0.012 * Real (K),
                     Across  => 0.02 + 0.01 * Real (K mod 5));
                  Start, Rest : Real_Array (1 .. Joint_Count);
                  Leads  : Boolean;
                  Closer : constant Real := (if W = Open then 1.0 else 0.0);
                  Idle   : constant Real_Array (1 .. Joint_Count) := [others => 0.0];
                  function At_Beat (Q : Real_Array) return Observation is
                    (if A = 1 then Observed (Beat, Q, Idle, Closer) else Observed (Beat, Idle, Q, Closer));
               begin
                  Contact (A, P, Pushed_Back, Start, Rest, Leads);
                  if Leads then
                     Beat := Beat + 1;
                     Press_Beat (H, Id, M, At_Beat (Start), Is_Blocked => False, Is_Still => True);
                     Beat := Beat + 1;
                     Press_Beat (H, Id, M, At_Beat (Rest), Is_Blocked => True, Is_Still => False);
                     Beat := Beat + 1;
                     Press_Beat (H, Id, M, At_Beat (Rest), Is_Blocked => False, Is_Still => True);
                     Made := Made + 1;
                     if First_Tip = 0 and then Known (Tip_In_Tool (H, Id, 1, Open)) then
                        First_Tip := Made;
                     end if;
                  end if;
               end;
            end loop;
         end loop;
      end loop;
   end Press_Everything;

   function Mahalanobis (T : Point_Estimate; At_Truth : Vec3) return Real is
      D : constant Vec3 := T.Mean - At_Truth;
   begin
      return Sqrt (D * (Inverse (T.Covariance) * D));
   end Mahalanobis;

   function Trace (S : Mat3) return Real is (S (1, 1) + S (2, 2) + S (3, 3));

   ---------------------------------------------------------------------------

   --  The second arm's hand presses its lobes at both openings on the table
   --  its own eye saw, while the arm stands in the world as the body has it
   --  placed, misplaced far beyond its placement's sigma, or not placed at
   --  all. Its tips are a hand's own, in its tool frame and the arm's unit:
   --  each within Z of its sigma of the truth, whatever the placement, and
   --  the same tips to the last digit for the same presses. Placed in the
   --  world the old way, the tips came out of the world's lengths where the
   --  hand is in the arm's: off by the unit's difference; unplaced, none. The
   --  table the arm's own eye saw is the presses' prior: the first lobe's tip
   --  is fixed by its first few presses, where a table nothing measured takes
   --  as many as the hand has tips and the surface has unknowns.
   procedure Tips_Free_Of_Placement is
      Z     : constant Real := Threshold (Vector_Gate (3));
      First : array (1 .. 2, Opening) of Point_Estimate;
   begin
      for How in Standing loop
         declare
            M     : Model;
            H     : Hands;
            Made  : Natural;
            Early : Natural;
            Worst : Real := 0.0;   --  the most any tip is off the truth, in units of its own sigma
            Wide  : Real := 0.0;   --  the widest tip's sigma
         begin
            Build (M, How);
            Give_Hand (H, 2);
            Press_Everything (H, M, 2, Made, Early);
            Check (Made >= 40, "the second arm's hand was pressed only" & Made'Image & " times, standing " & How'Image);
            Check (Early in 1 .. 4, "the first lobe's tip was known after" & Early'Image
                   & " presses, not after its first few, the arm standing " & How'Image);
            for L in 1 .. 2 loop
               for W in Opening loop
                  declare
                     T    : constant Point_Estimate := Tip_In_Tool (H, 1, L, W);
                     What : constant String := "lobe" & L'Image & " at " & W'Image & ", the arm standing " & How'Image;
                  begin
                     Check (Known (T), What & ": no tip");
                     if Known (T) then
                        Worst := Real'Max (Worst, Mahalanobis (T, Tips_True (L, W)));
                        Wide := Real'Max (Wide, Sqrt (Trace (T.Covariance)));
                        Check (Mahalanobis (T, Tips_True (L, W)) <= Z,
                               What & ": off by" & Real'Image (abs (T.Mean - Tips_True (L, W))) & ", which is"
                               & Real'Image (Mahalanobis (T, Tips_True (L, W))) & " of its sigma");
                        --  Within its sigma means something: that sigma is finer than the
                        --  error of taking the tip in the world's unit instead of the arm's.
                        Check (Z * Sqrt (Trace (T.Covariance)) < (Unit_2 - 1.0) * abs Tips_True (L, W),
                               What & ": measured only to" & Real'Image (Sqrt (Trace (T.Covariance))));
                        if How = Placed then
                           First (L, W) := T;
                        else
                           Check (T.Mean = First (L, W).Mean and then T.Covariance = First (L, W).Covariance,
                                  What & ": differs from where the arm stands placed, by"
                                  & Real'Image (abs (T.Mean - First (L, W).Mean)));
                        end if;
                     end if;
                  end;
               end loop;
            end loop;
            Driver.Log.Line (Driver.Log.Robot, "press test: the arm standing " & How'Image & "," & Made'Image
                             & " presses, the first tip known after" & Early'Image & ", the tips off by at most"
                             & Real'Image (Worst) & " of their own sigma (Z" & Real'Image (Z) & "), the widest sigma"
                             & Real'Image (Wide));
         end;
      end loop;
   end Tips_Free_Of_Placement;

   --  Aimed and lowered in the arm's own frame: the way down is into the table
   --  its own eye saw (not the first arm's), the plans exist with the arm not
   --  placed, and arrive where they were aimed; the same readings whatever the
   --  placement. How far the tip is above the table is the same, the tool and
   --  the table in one frame.
   procedure Press_Planned_In_The_Arms_Frame is
      Idle      : constant Real_Array (1 .. Joint_Count) := [others => 0.0];
      Here      : constant Real_Array (1 .. Joint_Count) := [0.01, -0.02, 0.0, 0.05, -0.04, 0.03];
      Along     : constant Vec3 := Unit (Tips_True (1, Open));
      Tip       : constant Point_Estimate := (Mean => Tips_True (1, Open), Covariance => 1.0e-8 * Identity3);
      First     : Aimed;
      First_Low : Real_Array (1 .. Joint_Count) := [others => 0.0];
      First_Gap : Estimate;
      Lowered_By : constant Real := 0.02;
   begin
      for How in Standing loop
         declare
            M     : Model;
            O     : constant Observation := Observed (1, Idle, Here, 0.0);
            Got   : Aimed;
            Tool  : Rigid;
            Plan  : Motion.Plan;
            Gap_Is : Estimate;
            Expected : Real;
         begin
            Build (M, How);
            Tool := Tool_In_Arm (M, 2, O).Pose;
            Aim (M, 2, 2, O, Along, Got);
            Check (Got.Ok, "an aim in the second arm's frame is refused, the arm standing " & How'Image);
            Check (Got.Into = -Table_Normal (2),
                   "the way down is not into the second arm's own table, the arm standing " & How'Image);
            Check (abs (Got.Above.Rotation * Along - Got.Into) < 1.0e-9,
                   "the aimed tool does not point the lobe's line of sight down, the arm standing " & How'Image);
            Check (abs (Got.Above.Translation - Tool.Translation) < 1.0e-9,
                   "the aimed tool left its eye, the arm standing " & How'Image);
            Check (Got.Turn > 0.0
                   and then abs (Got.Turn - Angle (Transpose (Tool.Rotation) * Got.Above.Rotation)) < 1.0e-12,
                   "the aim says it turns the tool by" & Got.Turn'Image & " rad, not as far as it turns it, the arm "
                   & "standing " & How'Image);
            Check (Motion.Status (Got.Plan) = Motion.Planned,
                   "the aim is not planned with the arm standing " & How'Image & ": " & Motion.Why (Got.Plan));
            if Motion.Status (Got.Plan) = Motion.Planned then
               declare
                  Ends : constant Real_Array := Motion.Last_Readings (Got.Plan);
                  There : constant Rigid := Tool_In_Arm (M, 2, Observed (2, Idle, Ends, 0.0)).Pose;
               begin
                  Check (abs (There.Translation - Got.Above.Translation) < 1.0e-4
                         and then Angle (Transpose (There.Rotation) * Got.Above.Rotation) < 1.0e-4,
                         "the aim's plan ends off the aim, the arm standing " & How'Image);
                  Plan := Lowered (M, 2, O, Got.Into, Lowered_By);
                  Check (Motion.Status (Plan) = Motion.Planned,
                         "a lowering is not planned with the arm standing " & How'Image & ": " & Motion.Why (Plan));
                  if Motion.Status (Plan) = Motion.Planned then
                     declare
                        Lowest : constant Real_Array := Motion.Last_Readings (Plan);
                        Low    : constant Rigid := Tool_In_Arm (M, 2, Observed (2, Idle, Lowest, 0.0)).Pose;
                     begin
                        Check (abs (Low.Translation - (Tool.Translation + Lowered_By * Got.Into)) < 1.0e-4,
                               "the lowering ends off its mark, the arm standing " & How'Image);
                        if How = Placed then
                           First_Low := Lowest;
                        else
                           Check (abs (Real_Vector (Lowest) - Real_Vector (First_Low)) < 1.0e-9,
                                  "the lowering reaches other readings with the arm standing " & How'Image);
                        end if;
                     end;
                  end if;
               end;
            end if;
            Gap_Is := Gap (M, 2, O, Tip, Table_Of (2), Got.Into);
            Expected := Table_Normal (2) * (Tool * Tip.Mean) - Table_Offset (2);
            Check (Known (Gap_Is) and then abs (Gap_Is.Value - Expected) < 1.0e-9,
                   "the tip is" & Real'Image (Gap_Is.Value) & " above the table, not" & Real'Image (Expected)
                   & ", the arm standing " & How'Image);
            if How = Placed then
               First := Got;
               First_Gap := Gap_Is;
            else
               Check (Got.Into = First.Into and then Got.Above = First.Above,
                      "the aim differs from where the arm stands placed, the arm standing " & How'Image);
               Check (Gap_Is = First_Gap, "the tip's gap differs from where the arm stands placed, the arm standing "
                      & How'Image);
            end if;
         end;
      end loop;
      --  An arm whose eye found no table has no way down, and an arm not
      --  fitted no tool: nothing is aimed.
      declare
         M     : Model;
         O     : constant Observation := Observed (1, Idle, Here, 0.0);
         Got   : Aimed;
      begin
         Build (M, Placed);
         M.Kinematics (2).Result.Table := (others => <>);
         Aim (M, 2, 2, O, Along, Got);
         Check (not Got.Ok, "a press is aimed down with no table found");
         M.Kinematics (2).Result.Table := Table_Of (2);
         M.Kinematics (2).Result.Fitted := False;
         Aim (M, 2, 2, O, Along, Got);
         Check (not Got.Ok, "a press is aimed by an arm that is not fitted");
      end;
   end Press_Planned_In_The_Arms_Frame;

   --  The arm is fitted again as it moves, and its frame and unit move with
   --  the fit: here the unit grows by a tenth between the presses (the same
   --  arm, its slides' readings making 1.1 of what they made, its table 1.1
   --  away). The presses made in the old unit were kept with the arm's
   --  readings and take their poses in the new one, so the hand comes out as
   --  the same hand in the new unit, each tip within Z of its sigma. Kept as
   --  poses they disagree with the table by a tenth of its distance, and the
   --  tips come out 12 sigma off.
   procedure Presses_Follow_The_Arm_Fitted_Again is
      Z     : constant Real := Threshold (Vector_Gate (3));
      Grown : constant Real := 1.1;
      Idle  : constant Real_Array (1 .. Joint_Count) := [others => 0.0];
      Here  : constant Real_Array (1 .. Joint_Count) := [0.01, -0.02, 0.0, 0.05, -0.04, 0.03];
      M     : Model;
      H     : Hands;
      Made, Early : Natural;
      Worst : Real := 0.0;   --  the most any tip is off the new unit's, in units of its own sigma
      Wide  : Real := 0.0;   --  the widest tip's sigma
   begin
      Build (M, Unplaced);
      Give_Hand (H, 2);
      Press_Everything (H, M, 2, Made, Early);
      for J in 1 .. 3 loop
         M.Kinematics (2).Result.Joints (J).C := Grown;
      end loop;
      M.Kinematics (2).Result.Table.Centre := Grown * M.Kinematics (2).Result.Table.Centre;
      M.Kinematics (2).Result.Table.Offset_Sigma := Grown * M.Kinematics (2).Result.Table.Offset_Sigma;
      --  The next beat of the arm, still: the hand sees the table is not the one it was fitted with.
      Press_Beat (H, 1, M, Observed (1, Idle, Here, 1.0), Is_Blocked => False, Is_Still => True);
      for L in 1 .. 2 loop
         for W in Opening loop
            declare
               T    : constant Point_Estimate := Tip_In_Tool (H, 1, L, W);
               What : constant String := "lobe" & L'Image & " at " & W'Image;
            begin
               Check (Known (T), What & " has no tip once the arm is fitted again");
               if Known (T) then
                  Worst := Real'Max (Worst, Mahalanobis (T, Grown * Tips_True (L, W)));
                  Wide := Real'Max (Wide, Sqrt (Trace (T.Covariance)));
                  Check (Mahalanobis (T, Grown * Tips_True (L, W)) <= Z,
                         What & " is" & Real'Image (abs (T.Mean - Grown * Tips_True (L, W)))
                         & " off the tip in the new unit, which is"
                         & Real'Image (Mahalanobis (T, Grown * Tips_True (L, W))) & " of its sigma");
                  --  Within its sigma means something: the sigma is finer than the unit's change,
                  --  and the tip is not the old unit's.
                  Check (Z * Sqrt (Trace (T.Covariance)) < (Grown - 1.0) * abs Tips_True (L, W),
                         What & " is measured only to" & Real'Image (Sqrt (Trace (T.Covariance))));
                  Check (Mahalanobis (T, Tips_True (L, W)) > Z, What & " is still the tip in the old unit");
               end if;
            end;
         end loop;
      end loop;
      Check (Made >= 40 and then Early > 0, "the hand was not pressed before the arm was fitted again");
      Driver.Log.Line (Driver.Log.Robot, "refit test:" & Made'Image & " presses, then the unit grown by" & Real'Image (Grown - 1.0)
                       & "; the tips off the new unit's by at most" & Real'Image (Worst) & " of their own sigma (Z"
                       & Real'Image (Z) & "), the widest sigma" & Real'Image (Wide));
   end Presses_Follow_The_Arm_Fitted_Again;

   --  The tip in the world, of the second arm placed: the placement turns it
   --  and its unit scales the tip's offset from the tool as it scales the
   --  tool's place; not placed, it is not known.
   procedure Tip_Taken_Into_The_World is
      Idle : constant Real_Array (1 .. Joint_Count) := [others => 0.0];
      Here : constant Real_Array (1 .. Joint_Count) := [0.01, -0.02, 0.0, 0.05, -0.04, 0.03];
      O    : constant Observation := Observed (1, Idle, Here, 1.0);
   begin
      for How in Standing loop
         declare
            M     : Model;
            H     : Hands;
            Made  : Natural;
            Early : Natural;
         begin
            Build (M, How);
            Give_Hand (H, 2);
            Press_Everything (H, M, 2, Made, Early);
            Check (Early > 0, "the hand's first lobe has no tip after" & Made'Image & " presses");
            declare
               R    : Arm_Fit renames M.Kinematics (2).Result;
               Pose : constant Rigid := Tool_In_Arm (M, 2, O).Pose;
               T    : constant Point_Estimate := Tip_In_Tool (H, 1, 1, Open);
               --  The tip in the arm's frame, then its unit, the placement's turn, and its place.
               Seen : constant Vec3 := Pose * T.Mean;
               Want : constant Vec3 := R.Placement.Rotation * (R.Scale * Seen) + R.Placement.Translation;
               Got  : constant Point_Estimate := Tip (H, M, 1, 1, Open, O);
            begin
               if How = Unplaced then
                  Check (not Known (Got), "a tip is known in a world its arm is not placed in");
               else
                  Check (Known (Got) and then abs (Got.Mean - Want) < 1.0e-9,
                         "the tip in the world is off by" & Real'Image (abs (Got.Mean - Want))
                         & " from the arm's placement of it, the arm standing " & How'Image);
               end if;
            end;
         end;
      end loop;
   end Tip_Taken_Into_The_World;

   --  Where the least push is read. The arm's frame is the eye at its
   --  reference readings, where the tool's place is known exactly, and A15's
   --  first press stood there before its aim: the least push read there was
   --  too small to print and the descent doubled from it for 34 pushes of nothing. A
   --  press lowers the tool from where the aim leaves it, which is where the
   --  push is to be told from the tool's noise.
   procedure Least_Push_Is_Where_The_Aim_Leaves_The_Tool is
      Idle  : constant Real_Array (1 .. Joint_Count) := [others => 0.0];
      Along : constant Vec3 := Unit (Tips_True (1, Open));
      M     : Model;
      Got   : Aimed;
   begin
      Build (M, Placed);
      Aim (M, 1, 1, Observed (1, Idle, Idle, 0.0), Along, Got);
      Check (Got.Ok and then Motion.Status (Got.Plan) = Motion.Planned, "the aim from the reference readings is not planned");
      Check (Least_Push (M, 1, Observed (1, Idle, Idle, 0.0)) = 0.0,
             "the tool's place at the arm's reference readings is not known exactly");
      Check (Got.Least > 0.0 and then Got.Least < Real'Last,
             "the least push where the aim leaves the tool is" & Got.Least'Image & ", not above zero");
      Check (Got.Least = Least_Push (M, 1, Observed (1, Motion.Last_Readings (Got.Plan), Idle, 0.0)),
             "the least push is not where the aim's plan ends");
   end Least_Push_Is_Where_The_Aim_Leaves_The_Tool;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.pressing.tips",
                             "a hand's presses measure its tips in the world's frame, which the arm's placement "
                             & "moves, or not at all with the arm not placed", Tips_Free_Of_Placement'Access);
      Driver.Tests.Register ("hand.pressing.plan",
                             "a press is aimed down into another arm's table, or planned in the world where an arm "
                             & "not placed in it cannot", Press_Planned_In_The_Arms_Frame'Access);
      Driver.Tests.Register ("hand.pressing.refit",
                             "presses kept as poses of an arm's frame stay in it when the arm is fitted again and its "
                             & "unit moves", Presses_Follow_The_Arm_Fitted_Again'Access);
      Driver.Tests.Register ("hand.pressing.world",
                             "a tip is taken into the world without the arm's unit, or known of an arm not placed "
                             & "there", Tip_Taken_Into_The_World'Access);
      Driver.Tests.Register ("hand.pressing.least",
                             "the least push of a press is read where the aim leaves the tool, not at the arm's "
                             & "reference readings where the tool's place is exact", Least_Push_Is_Where_The_Aim_Leaves_The_Tool'Access);
   end Register;

end Driver.Robot.Hand.Pressing.Tests;
