with Driver.Action.Snapshots.Tests;
with Driver.Tests;

package body Driver.Action.Goals.Tests is

   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use Driver.Action.Snapshots.Tests;

   package Contact renames Driver.Action.Contact;

   Pitch : constant Real := 0.005;
   Sigma : constant Real := 0.0005;

   --  The table is tilted against gravity, so a direction taken from the
   --  wrong one of the two shows; and the whole world is turned.
   World : constant Rigid := (Rotation => Exp ([0.9, -1.3, 2.2]), Translation => [2.0, 1.0, -0.5]);
   Table : constant Rigid := World * (Rotation => Exp ([0.2, 0.0, 0.0]), Translation => Zero3);

   function Table_Up return Vec3 is (Rotate (Table, [0.0, 0.0, 1.0]));
   function Table_X return Vec3 is (Rotate (Table, [1.0, 0.0, 0.0]));

   function On_Table (X, Y, About, Lift : Real) return Rigid is
     (Table * (Rotation => Exp ([0.0, 0.0, About]), Translation => [X, Y, Lift]));

   --  A still eye on the near side of the table, looking across it and down:
   --  its image columns grow along the table's x.
   function Still_Eye return Eye_State is
     ((Pose   => (Pose => Table * (Rotation => Exp ([-2.16, 0.0, 0.0]), Translation => [0.0, -0.6, 0.4]),
                  Position_Covariance => (Sigma * Sigma) * Identity3,
                  Rotation_Covariance => (Sigma * Sigma) * Identity3),
       On_Arm => 0));

   function Base_Scene return Snapshot is
      S : Snapshot;
   begin
      S.Up := (Unit_Vector => Rotate (World, [0.0, 0.0, 1.0]), Sigma => Sigma);
      S.Surfaces.Append (Floor (1, Table, Sigma));
      S.Arms.Append (Arm_Of (1, Table * (Rotation => Identity3, Translation => [0.0, 0.0, 0.3]), Sigma));
      S.Eyes.Append (Still_Eye);
      return S;
   end Base_Scene;

   procedure Add (S : in out Snapshot; Id : Thing_Id; M : Model; Place : Rigid; Support : Surface_Id'Base := 1) is
      T : Thing_State := Thing_Of (Id, M, Place, Pitch, Sigma, Support);
   begin
      T.Best_Eye := 1;
      S.Things.Append (T);
   end Add;

   function Toward (S : Snapshot; A, B : Thing_Id; R : Pair_Relation) return Answer is
      Ax : Real;
      L  : constant Vec3 := Long_Axis (S, A, Ax);
   begin
      return Toward (S, Item_Of (S, A), Item_Of (S, B), R, L, Ax);
   end Toward;

   procedure Height_Follows_Its_Support is
      S : Snapshot := Base_Scene;
   begin
      Add (S, 1, Bar (0.2, 0.02, 0.02), On_Table (0.0, 0.0, 0.4, 0.0));
      declare
         A : constant Answer := Twist_Of (S, 1, Height, Increase => True);
         D : constant Answer := Twist_Of (S, 1, Height, Increase => False);
      begin
         Check (A.Ok and then D.Ok, "height cannot be changed on a measured table");
         Check_Close (A.Motion.Linear * Table_Up, 1.0, 1.0e-9, "up is not away from the table it lies on");
         Check_Close (D.Motion.Linear * Table_Up, -1.0, 1.0e-9, "down is not toward the table it lies on");
         Check (abs A.Motion.Angular = 0.0, "a change of height turns the thing");
         Check (not Known (A.Gap), "a change of height is given an end of its own");
      end;
      S.Things (1).Support := 0;
      Check_Close (Twist_Of (S, 1, Height, Increase => True).Motion.Linear * S.Up.Unit_Vector, 1.0, 1.0e-9,
                   "with no support, up is not gravity's up");
      S.Things (1).Support := 1;
      S.Up := (others => <>);
      Check (not Changeable (S) (Height), "height is offered with no gravity measured");
   end Height_Follows_Its_Support;

   procedure Heading_Turns_The_Long_Axis is
      S : Snapshot := Base_Scene;
      Axis_Sigma : Real;
   begin
      Add (S, 1, Bar (0.2, 0.02, 0.02), On_Table (0.0, 0.0, 0.4, 0.0));
      Add (S, 2, Upright_Cylinder (0.03, 0.08), On_Table (0.2, 0.1, 0.0, 0.0));
      declare
         A : constant Vec3 := Long_Axis (S, 1, Axis_Sigma);
         Length_Way : constant Vec3 := Rotate (On_Table (0.0, 0.0, 0.4, 0.0), [1.0, 0.0, 0.0]);
         T : constant Answer := Twist_Of (S, 1, Heading, Increase => True);
      begin
         Check (abs (A * Length_Way) > 0.999, "the long axis of a bar is not along its length");
         Check (Axis_Sigma < 0.01, "a bar's long axis is not known well");
         Check (T.Ok, "a bar's heading cannot be changed");
         Check (abs (Unit (T.Motion.Angular) - Table_Up) < 1.0e-9,
                "heading up does not turn counter-clockwise about the table's up");
         Check (abs (Contact.Velocity (T.Motion, Thing (S, 1).Centre.Mean)) < 1.0e-9,
                "a change of heading moves the thing's centre");
      end;
      declare
         Round : constant Vec3 := Long_Axis (S, 2, Axis_Sigma);
      begin
         Check (not (abs Round > 0.0) and then Axis_Sigma = Real'Last, "a round footprint is given a long axis");
      end;
      Check (not Twist_Of (S, 2, Heading, Increase => True).Ok, "a round thing's heading is changed");
   end Heading_Turns_The_Long_Axis;

   procedure Tilt_Leans_Away_From_The_Still_Eye is
      S : Snapshot := Base_Scene;
   begin
      Add (S, 1, Block (0.04, 0.04, 0.1), On_Table (0.05, 0.05, 0.3, 0.0));
      declare
         T    : constant Answer := Twist_Of (S, 1, Tilt, Increase => True);
         Top  : constant Vec3 := Thing (S, 1).Centre.Mean + 0.05 * Table_Up;
         Away : constant Vec3 := Top - S.Eyes (1).Pose.Pose.Translation;
      begin
         Check (T.Ok, "tilt cannot be changed with a still eye");
         Check (Contact.Velocity (T.Motion, Top) * (Away - Real'(Away * Table_Up) * Table_Up) > 0.0,
                "tilt up does not lean the top away from the still eye");
         Check (abs (T.Motion.Angular * Table_Up) < 1.0e-9, "tilt turns about the up");
      end;
      S.Eyes (1).On_Arm := 1;
      Check (not Twist_Of (S, 1, Tilt, Increase => True).Ok, "tilt is changed with only an eye that rides an arm");
      Check (not Changeable (S) (Tilt), "tilt is offered with no still eye");
   end Tilt_Leans_Away_From_The_Still_Eye;

   procedure As_The_Still_Eye_Sees_Them is
      S : Snapshot := Base_Scene;
   begin
      --  Thing 1 is nearer the eye than thing 2, and left of it.
      Add (S, 1, Block (0.04, 0.04, 0.04), On_Table (0.0, 0.0, 0.0, 0.0));
      Add (S, 2, Block (0.06, 0.06, 0.06), On_Table (0.2, 0.1, 0.0, 0.0));
      declare
         Eye : constant Vec3 := S.Eyes (1).Pose.Pose.Translation;
         From_Eye : constant Vec3 := Thing (S, 1).Centre.Mean - Eye;
         F : constant Answer := Toward (S, 1, 2, Farther);
         R : constant Answer := Toward (S, 1, 2, Right);
      begin
         Check (Toward (S, 1, 2, Nearer).Done, "a thing nearer the eye than the other is not said to be nearer");
         Check (F.Ok and then not F.Done and then F.Motion.Linear * From_Eye > 0.0,
                "farther does not go away from the eye");
         Check (abs (F.Motion.Linear * Table_Up) < 1.0e-9, "farther leaves the table it lies on");
         Check (Toward (S, 1, 2, Left).Done, "a thing left of the other in the image is not said to be left");
         Check (R.Ok and then not R.Done and then R.Motion.Linear * Table_X > 0.0,
                "right does not go toward the image's right");
         Check (abs (R.Motion.Linear * Table_Up) < 1.0e-9, "right leaves the table it lies on");
      end;
      S.Eyes (1).On_Arm := 1;
      Check (not Toward (S, 1, 2, Nearer).Ok, "nearer is judged by an eye that rides an arm");
   end As_The_Still_Eye_Sees_Them;

   --  A direction taken from the image is never exactly level: a thing a little
   --  off the eye's middle column is carried along the image's columns, which
   --  there rise a little off the table. A thing that lies on the table goes
   --  along it unless the way leaves it by more than that direction and the
   --  table's normal, between them, can tell from level; the footing of a
   --  thing that is lifted, however little, bears nothing.
   procedure A_Slide_Stays_On_The_Table_Within_What_Is_Known is
      S : Snapshot := Base_Scene;
   begin
      Add (S, 1, Block (0.04, 0.04, 0.04), On_Table (0.002, 0.0, 0.0, 0.0));
      Add (S, 2, Block (0.06, 0.06, 0.06), On_Table (0.2, 0.1, 0.0, 0.0));
      declare
         R : constant Answer := Toward (S, 1, 2, Right);
      begin
         Check (R.Ok and then not R.Done and then R.Motion.Linear * Table_X > 0.0,
                "right does not go toward the image's right");
         Check (abs (R.Motion.Linear * Table_Up) < 1.0e-9,
                "right takes a thing a little off the middle column up off the table it lies on");
      end;
      --  Far off the middle column the columns do leave the table: that is a way up, and it is taken.
      S.Things.Delete_First;
      Add (S, 1, Block (0.04, 0.04, 0.04), On_Table (0.15, 0.0, 0.0, 0.0));
      declare
         R : constant Answer := Toward (S, 1, 2, Right);
      begin
         Check (R.Ok and then R.Motion.Linear * Table_Up > 1.0e-4,
                "right on a thing far off the middle column does not go up with the columns");
      end;
   end A_Slide_Stays_On_The_Table_Within_What_Is_Known;

   procedure Onto_Goes_Up_Over_And_Down is
      S : Snapshot := Base_Scene;
   begin
      Add (S, 1, Block (0.04, 0.04, 0.04), On_Table (0.0, 0.0, 0.0, 0.0));
      Add (S, 2, Block (0.06, 0.06, 0.06), On_Table (0.2, 0.1, 0.0, 0.0));
      declare
         O : constant Answer := Toward (S, 1, 2, Onto);
      begin
         Check (O.Ok and then abs (O.Motion.Linear - Table_Up) < 1.0e-9,
                "onto a taller thing beside it does not go up first");
      end;
      --  Raised clear of the other's top: across to it.
      S.Things.Delete_First;
      Add (S, 1, Block (0.04, 0.04, 0.04), On_Table (0.0, 0.0, 0.0, 0.1), Support => 0);
      declare
         O : constant Answer := Toward (S, 1, 2, Onto);
      begin
         Check (O.Ok and then not O.Done and then abs (O.Motion.Linear * Table_Up) < 1.0e-9,
                "onto from above the other's top does not go across to it");
      end;
      --  Straight above it: down.
      S.Things.Delete_Last;
      Add (S, 1, Block (0.04, 0.04, 0.04), On_Table (0.2, 0.1, 0.0, 0.1), Support => 0);
      declare
         D : constant Answer := Toward (S, 1, 2, Onto);
      begin
         Check (D.Ok and then abs (D.Motion.Linear + Table_Up) < 1.0e-9, "onto from straight above does not go down");
         Check (Toward (S, 1, 2, Above).Done, "a thing straight over the other is not above it");
      end;
      --  Resting on its top: done.
      S.Things.Delete_Last;
      Add (S, 1, Block (0.04, 0.04, 0.04), On_Table (0.2, 0.1, 0.0, 0.06), Support => 0);
      Check (Toward (S, 1, 2, Onto).Done, "a thing resting on the other's top is not on it");
      Check (not Toward (S, 1, 2, Off).Done, "a thing resting on the other's top is said to be off it");
   end Onto_Goes_Up_Over_And_Down;

   procedure Never_Into_Its_Surface is
      S : Snapshot := Base_Scene;
   begin
      --  The other one straight under the table, as an eye's ghost might put
      --  it, on a level table.
      S.Up := (Unit_Vector => Table_Up, Sigma => Sigma);
      Add (S, 1, Block (0.04, 0.04, 0.04), On_Table (0.0, -0.1, 0.0, 0.0));
      Add (S, 2, Block (0.04, 0.04, 0.04), On_Table (0.0, -0.1, 0.0, -0.3), Support => 0);
      Check (not Toward (S, 1, 2, Below).Ok, "below a thing under the table goes into the table");
   end Never_Into_Its_Surface;

   procedure Facing_Turns_The_Nearer_End is
      S : Snapshot := Base_Scene;
   begin
      Add (S, 1, Bar (0.2, 0.02, 0.02), On_Table (0.0, 0.0, 0.0, 0.0));
      Add (S, 2, Block (0.04, 0.04, 0.04), On_Table (-0.3, 0.2, 0.0, 0.0));
      declare
         F  : constant Answer := Toward (S, 1, 2, Facing);
         Ax : Real;
         A0 : constant Vec3 := Long_Axis (S, 1, Ax);
         To : constant Vec3 := Thing (S, 2).Centre.Mean - Thing (S, 1).Centre.Mean;
         A  : constant Vec3 := (if A0 * To < 0.0 then -A0 else A0);
         After : constant Vec3 := Exp (0.01 * Unit (F.Motion.Angular)) * A;
      begin
         Check (F.Ok and then not F.Done, "a bar pointing away is said to face the other");
         Check (Unit (After) * Unit (To) > Unit (A) * Unit (To), "facing turns the bar's end away from the other");
         Check (F.Gap.Value > 0.0 and then Known (F.Gap), "facing has no measured angle left to turn");
      end;
      S.Things.Delete_Last;
      Add (S, 2, Block (0.04, 0.04, 0.04), On_Table (-0.3, 0.0, 0.0, 0.0));
      Check (Toward (S, 1, 2, Facing).Done, "a bar pointing at the other is not said to face it");
   end Facing_Turns_The_Nearer_End;

   procedure Register is
   begin
      Register ("action.goals.height", "height is changed along gravity on a tilted table, or with no up measured",
                Height_Follows_Its_Support'Access);
      Register ("action.goals.heading", "heading turns about the wrong axis, or a round thing gets one",
                Heading_Turns_The_Long_Axis'Access);
      Register ("action.goals.tilt", "tilt leans the thing toward the eye, or is offered without a still eye",
                Tilt_Leans_Away_From_The_Still_Eye'Access);
      Register ("action.goals.eye", "nearer, farther, left and right are judged other than as the still eye sees",
                As_The_Still_Eye_Sees_Them'Access);
      Register ("action.goals.level", "a thing a little off the eye's middle column is taken up off the table it lies on",
                A_Slide_Stays_On_The_Table_Within_What_Is_Known'Access);
      Register ("action.goals.onto","onto knocks into the other, or does not come down on it",
                Onto_Goes_Up_Over_And_Down'Access);
      Register ("action.goals.surface", "a thing is moved into the surface it lies on", Never_Into_Its_Surface'Access);
      Register ("action.goals.facing", "facing turns the far end, or a thing already facing is turned",
                Facing_Turns_The_Nearer_End'Access);
   end Register;

end Driver.Action.Goals.Tests;
