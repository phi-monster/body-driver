with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Text_IO;
with Driver.Action.Snapshots.Tests;
with Driver.Clock;
with Driver.Log;
with Driver.Tests;

package body Driver.Action.Contact.Search.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;
   use Driver.Action.Snapshots.Tests;

   Pi : constant := Ada.Numerics.Pi;

   Pitch : constant Real := 0.005;
   Sigma : constant Real := 0.0005;

   Upright : constant Rigid := Identity;
   Turned  : constant Rigid := (Rotation => Exp ([0.9, -1.3, 2.2]), Translation => [2.0, 1.0, -0.5]);

   type Rigid_Array is array (Positive range <>) of Rigid;
   Both_Frames : constant Rigid_Array := [Upright, Turned];

   --  A world turned by Place: the floor, one thing on it (its own frame
   --  turned About its up and set At on the floor), and one arm above it
   --  pointing its tool down at it.
   type Scene is record
      S     : Snapshot;
      Place : Rigid;
      Thing_Frame : Rigid;
   end record;

   function Up_Of (X : Scene) return Vec3 is (Rotate (X.Place, [0.0, 0.0, 1.0]));

   function Make (M : Model; Place : Rigid; About : Real; Hands : Natural; Plate : Boolean := False) return Scene is
      X : Scene;
      F : constant Rigid := Place * (Rotation => Exp ([0.0, 0.0, About]), Translation => Zero3);
      Down : constant Rigid := Place * (Rotation => Exp ([Pi, 0.0, 0.0]), Translation => [0.01, -0.02, 0.2]);
   begin
      X.Place := Place;
      X.Thing_Frame := F;
      X.S.Up := (Unit_Vector => Up_Of (X), Sigma => Sigma);
      X.S.Surfaces.Append (Floor (1, Place, Sigma));
      X.S.Things.Append (Thing_Of (1, M, F, Pitch, Sigma, 1));
      if Plate then
         X.S.Arms.Append (Plate_Arm (1, Down, 0.02, Pitch, Sigma));
      else
         X.S.Arms.Append (Arm_Of (1, Down, Sigma));
      end if;
      if Hands = 2 then
         X.S.Hands.Append (Gripper (1, 1, 0.08, 0.015, 0.01, 0.04, Sigma));
      elsif Hands = 5 then
         X.S.Hands.Append (Five_Lobes (1, 1, 0.06, 0.012, 0.01, 0.05, Sigma));
      end if;
      return X;
   end Make;

   function Anywhere (Tool : Rigid) return Boolean is (abs Tool.Translation >= 0.0);
   function Nowhere (Tool : Rigid) return Boolean is (abs Tool.Translation < 0.0);
   function Free_Anywhere (Tool : Rigid; Closers : Real_Vectors.Vector) return Boolean is
     (abs Tool.Translation >= 0.0 and then Natural (Closers.Length) >= 0);

   procedure Search
     (X : Scene; Motion : Twist; Best : out Candidate; Found : out Boolean; Tried : out Account;
      Friction : Friction_Bounds := (others => <>); Reach_None : Boolean := False)
   is
      Beside : Point_Vectors.Vector;
      Start  : constant Duration := Driver.Clock.Seconds;
   begin
      Find (Shape_Of (X.S, 1), Beside, Effector_Of (X.S, 1), Motion, Up_Of (X), Friction,
            (if Reach_None then Nowhere'Access else Anywhere'Access), Free_Anywhere'Access, Best, Found, Tried);
      Ada.Text_IO.Put_Line ("      search: " & Driver.Log.Image (Real (Driver.Clock.Seconds - Start), 3) & " s, "
                            & Driver.Log.Image (Tried.Placements) & " placements, "
                            & Driver.Log.Image (Tried.Distinct) & " contact sets");
   end Search;

   --  The unit direction between the first two touches.
   function Across (C : Candidate) return Vec3 is (Unit (C.Touches (2).Point - C.Touches (1).Point));

   procedure Bar_Closes_Across_Its_Width is
   begin
      for Place of Both_Frames loop
         declare
            X : constant Scene := Make (Bar (0.2, 0.02, 0.02), Place, 0.4, 2);
            Long : constant Vec3 := Rotate (X.Thing_Frame, [1.0, 0.0, 0.0]);
            Best : Candidate;
            Found : Boolean;
            Tried : Account;
         begin
            Search (X, Slide (Up_Of (X)), Best, Found, Tried);
            Check (Found, "no contact set raises a bar a gripper spans: " & Say (Tried));
            if Found then
               Check (Natural (Best.Touches.Length) = 2, "a two-lobe set has" & Best.Touches.Length'Image & " touches");
               Check (abs (Across (Best) * Long) < 0.1, "the lobes close along the bar's length, not across it");
               Check (abs (Best.Touches (1).Inward * Best.Touches (2).Inward + 1.0) < 0.1,
                      "the two touches do not oppose each other");
               for T of Best.Touches loop
                  Check (T.Inward * (Shape_Of (X.S, 1).Centre.Mean - T.Point) > 0.0,
                         "a touch presses outward, away from the thing");
               end loop;
               Check (Best.Force < Real'Last, "the chosen set needs no finite force");
               --  Two opposed touches across the bar's middle, with friction
               --  one resolution of the normals' angle above what they need,
               --  squeeze it with no more than about the inverse of that
               --  resolution times its weight; at the very friction they need
               --  the squeeze has no bound.
               declare
                  Resolution : constant Real := Shape_Of (X.S, 1).Normal_Sigma;
               begin
                  Check (Best.Force <= 1.0 / Sin (Resolution),
                         "a gripper lifting a bar across its middle squeezes it with" & Best.Force'Image
                         & " times its weight: it was chosen at the very friction it needs");
               end;
            end if;
         end;
      end loop;
   end Bar_Closes_Across_Its_Width;

   procedure Scissors_Close_Across_A_Part is
      X     : constant Scene := Make (Scissors (0.18, 0.016, 0.006), Turned, 1.1, 2);
      Best  : Candidate;
      Found : Boolean;
      Tried : Account;
   begin
      Search (X, Slide (Up_Of (X)), Best, Found, Tried);
      Check (Found, "no contact set raises flat scissors: " & Say (Tried));
      if Found then
         --  Thin flat parts are taken by their edges, level with the table,
         --  the two touches pressing against each other: never from above,
         --  where the lobes would land on the blades and close on nothing.
         Check (abs (Across (Best) * Up_Of (X)) < 0.2, "the lobes close up and down on flat scissors");
         Check (Best.Touches (1).Inward * Best.Touches (2).Inward < -0.9,
                "the two touches on the scissors do not press against each other");
      end if;
   end Scissors_Close_Across_A_Part;

   --  The pose where the last straight stretch begins is reached by the
   --  arm's travel, so a candidate whose pose there the travel cannot reach
   --  is passed over for one it can. Here the travel keeps the hand above
   --  the table by the hand's own depth, which only a hand that comes down
   --  onto the scissors is where its last stretch begins.
   procedure Travel_Reaches_The_Start is
      X     : constant Scene := Make (Scissors (0.18, 0.016, 0.006), Turned, 2.4, 2);
      Depth : constant Real := Effector_Of (X.S, 1).Depth;
      function Above_The_Table (Tool : Rigid; Closers : Real_Vectors.Vector) return Boolean is
        (Natural (Closers.Length) >= 0
         and then Real'((Tool.Translation - X.Place.Translation) * Up_Of (X)) >= Depth);
      Beside : Point_Vectors.Vector;
      Best   : Candidate;
      Found  : Boolean;
      Tried  : Account;
   begin
      Find (Shape_Of (X.S, 1), Beside, Effector_Of (X.S, 1), Slide (Up_Of (X)), Up_Of (X), (others => <>),
            Anywhere'Access, Above_The_Table'Access, Best, Found, Tried);
      Check (Found, "no contact set raises flat scissors from where the travel can bring the hand: " & Say (Tried));
      if Found then
         Check (Above_The_Table (Best.Hover, Best.Before),
                "the last straight stretch begins where the arm's travel cannot bring the hand");
      end if;
   end Travel_Reaches_The_Start;

   procedure Cylinder_Opposite_Sides is
      X     : constant Scene := Make (Upright_Cylinder (0.025, 0.08), Turned, 0.0, 2);
      Axis  : constant Vec3 := Up_Of (X);
      Best  : Candidate;
      Found : Boolean;
      Tried : Account;
   begin
      Search (X, Slide (Axis), Best, Found, Tried);
      Check (Found, "no contact set raises an upright cylinder: " & Say (Tried));
      if Found then
         declare
            Mid : constant Vec3 := (Best.Touches (1).Point + Best.Touches (2).Point) / 2.0;
            Off : constant Vec3 := Mid - X.Thing_Frame.Translation;
         begin
            Check (abs (Off - Real'(Off * Axis) * Axis) < 2.0 * Pitch,
                   "the touches on a cylinder are not on opposite sides of its axis");
         end;
      end if;
   end Cylinder_Opposite_Sides;

   procedure Cup_Is_Raised is
      X     : constant Scene := Make (Cup (0.03, 0.004, 0.08), Turned, 0.7, 2);
      Best  : Candidate;
      Found : Boolean;
      Tried : Account;
   begin
      Search (X, Slide (Up_Of (X)), Best, Found, Tried);
      Check (Found, "no contact set raises a cup: " & Say (Tried));
   end Cup_Is_Raised;

   procedure Five_Lobes_Wrap_A_Cylinder is
      X     : constant Scene := Make (Upright_Cylinder (0.02, 0.08), Turned, 0.0, 5);
      Best  : Candidate;
      Found : Boolean;
      Tried : Account;
   begin
      Search (X, Slide (Up_Of (X)), Best, Found, Tried);
      Check (Found, "no contact set of a five-lobe hand raises a cylinder: " & Say (Tried));
      if Found then
         Check (Natural (Best.Touches.Length) >= 3, "a five-lobe hand closes on a cylinder with"
                & Best.Touches.Length'Image & " touches");
      end if;
   end Five_Lobes_Wrap_A_Cylinder;

   --  Five alike lobes evenly around the hand's axis repeat after a fifth
   --  of a turn, two alike opposed lobes after half a turn; one lobe made
   --  unlike the others, wider or moved by far more than its sigma, breaks
   --  the repetition.
   procedure Symmetry_Is_Measured is
      Five : constant Scene := Make (Upright_Cylinder (0.02, 0.08), Upright, 0.0, 5);
      Two  : constant Scene := Make (Bar (0.2, 0.02, 0.02), Upright, 0.0, 2);
   begin
      Check (Effector_Of (Five.S, 1).Repeats = 5, "five alike lobes around the axis are not seen to repeat"
             & Effector_Of (Five.S, 1).Repeats'Image);
      Check (Effector_Of (Two.S, 1).Repeats = 2, "two alike opposed lobes are not seen to repeat"
             & Effector_Of (Two.S, 1).Repeats'Image);
      for Change in 1 .. 2 loop
         declare
            X : Scene := Five;
            H : Hand_State := X.S.Hands (1);
            L : Lobe_State := H.Lobes (3);
         begin
            if Change = 1 then
               L.Width := L.Width + 20.0 * Sigma;
            else
               L.Open_Tip := L.Open_Tip + [0.0, 0.0, 20.0 * Sigma];
            end if;
            H.Lobes.Replace_Element (3, L);
            X.S.Hands.Replace_Element (1, H);
            Check (Effector_Of (X.S, 1).Repeats = 1, "a hand with one lobe unlike the others is taken to repeat"
                   & Effector_Of (X.S, 1).Repeats'Image & " times (change" & Change'Image & ")");
         end;
      end loop;
   end Symmetry_Is_Measured;

   --  A turn of the hand its symmetry allows makes the same touches with
   --  other lobes, so it is tried when the first pose cannot be reached:
   --  here only poses whose first lobe stays clear of the thing all the way
   --  from open to closed can be, which no pose that pins that lobe is.
   procedure Symmetric_Pose_Reached is
      X      : constant Scene := Make (Upright_Cylinder (0.02, 0.08), Upright, 0.0, 5);
      Radius : constant Real := 0.02;
      Height : constant Real := 0.08;
      --  The five-lobe hand of Make: its first lobe's tip open and closed.
      Open   : constant Vec3 := [0.065, 0.0, 0.05];
      Closed : constant Vec3 := [0.017, 0.0, 0.05];
      Clear  : constant Real := 0.016;   --  half its width and its thickness
      function First_Lobe_Clear (Tool : Rigid) return Boolean is
         Back : constant Rigid := Inverse (X.Thing_Frame);
         Steps : constant := 100;
      begin
         for K in 0 .. Steps loop
            declare
               P : constant Vec3 := Back * (Tool * (Open + (Real (K) / Real (Steps)) * (Closed - Open)));
               Radial : constant Real := Real'Max (0.0, Sqrt (P (1) ** 2 + P (2) ** 2) - Radius);
               Axial  : constant Real := Real'Max (0.0, Real'Max (-P (3), P (3) - Height));
            begin
               if Sqrt (Radial ** 2 + Axial ** 2) <= Clear then
                  return False;
               end if;
            end;
         end loop;
         return True;
      end First_Lobe_Clear;
      Beside : Point_Vectors.Vector;
      Best   : Candidate;
      Found  : Boolean;
      Tried  : Account;
   begin
      Find (Shape_Of (X.S, 1), Beside, Effector_Of (X.S, 1), Slide (Up_Of (X)), Up_Of (X), (others => <>),
            First_Lobe_Clear'Access, Free_Anywhere'Access, Best, Found, Tried);
      Check (Found, "a five-lobe hand whose first lobe must stay clear finds no turn of itself that touches: "
             & Say (Tried));
      if Found then
         Check (First_Lobe_Clear (Best.Tool) and then First_Lobe_Clear (Best.Hover),
                "the chosen pose brings the first lobe onto the thing");
      end if;
   end Symmetric_Pose_Reached;

   procedure Plate_Slides_A_Bar_Through_Its_Middle is
      X     : constant Scene := Make (Bar (0.2, 0.02, 0.02), Turned, 0.3, 0, Plate => True);
      Side  : constant Vec3 := Rotate (X.Thing_Frame, [0.0, 1.0, 0.0]);
      Long  : constant Vec3 := Rotate (X.Thing_Frame, [1.0, 0.0, 0.0]);
      Best  : Candidate;
      Found : Boolean;
      Tried : Account;
   begin
      Search (X, Slide (Side), Best, Found, Tried);
      Check (Found, "a flat end cannot slide a bar sideways: " & Say (Tried));
      if Found then
         Check (Natural (Best.Touches.Length) = 1,
                "a body with no lobes made" & Best.Touches.Length'Image & " touches");
         Check (Best.Touches (1).Inward * Side > 0.9, "the touch does not press the bar the way it is to go");
         Check (abs ((Best.Touches (1).Point - X.Thing_Frame.Translation) * Long) <= Pitch,
                "a bar slid sideways is touched away from its middle, which would rotate it");
      end if;
   end Plate_Slides_A_Bar_Through_Its_Middle;

   procedure Bar_Spun_About_Up is
      X     : constant Scene := Make (Bar (0.2, 0.02, 0.02), Turned, 0.4, 2);
      Best  : Candidate;
      Found : Boolean;
      Tried : Account;
   begin
      Search (X, Rotation (Up_Of (X), 1.0, Shape_Of (X.S, 1).Centre.Mean), Best, Found, Tried);
      Check (Found, "no contact set rotates a bar about its up: " & Say (Tried));
   end Bar_Spun_About_Up;

   procedure Surface_In_Way_Refused is
      X     : constant Scene := Make (Bar (0.2, 0.02, 0.02), Turned, 0.4, 2);
      Best  : Candidate;
      Found : Boolean;
      Tried : Account;
   begin
      Search (X, Slide (-Up_Of (X)), Best, Found, Tried);
      Check (not Found and then Tried.Surface_In_Way, "a motion into the table is not refused as such");
   end Surface_In_Way_Refused;

   procedure Failed_Friction_Not_Taken_Again is
      X      : constant Scene := Make (Bar (0.2, 0.02, 0.02), Turned, 0.4, 2);
      First  : Candidate;
      Again  : Candidate;
      Found  : Boolean;
      Tried  : Account;
   begin
      Search (X, Slide (Up_Of (X)), First, Found, Tried);
      Check (Found, "no first choice to fail with");
      if Found then
         Search (X, Slide (Up_Of (X)), Again, Found, Tried, Friction => (Low => 0.0, High => First.Mu_Worst));
         Check (not Found or else Again.Mu_Worst < First.Mu_Worst,
                "a contact set needing the friction that failed is chosen again");
      end if;
   end Failed_Friction_Not_Taken_Again;

   procedure Unreachable_Refused_With_What_Was_Tried is
      X     : constant Scene := Make (Bar (0.2, 0.02, 0.02), Turned, 0.4, 2);
      Best  : Candidate;
      Found : Boolean;
      Tried : Account;
   begin
      Search (X, Slide (Up_Of (X)), Best, Found, Tried, Reach_None => True);
      Check (not Found, "a contact set nothing can reach is chosen");
      Check (Tried.Unreachable > 0 and then Tried.Placements > 0, "a refusal does not count what it tried");
   end Unreachable_Refused_With_What_Was_Tried;

   procedure Register is
   begin
      Driver.Tests.Register ("action.search.bar", "a gripper closes along a bar instead of across it, or is chosen "
                             & "at the very friction it needs and squeezes without bound",
                             Bar_Closes_Across_Its_Width'Access);
      Driver.Tests.Register ("action.search.scissors", "flat scissors are taken across their gap or from above",
                             Scissors_Close_Across_A_Part'Access);
      Driver.Tests.Register ("action.search.travel", "the last straight stretch begins where the arm's travel cannot "
                             & "bring the hand", Travel_Reaches_The_Start'Access);
      Driver.Tests.Register ("action.search.cylinder", "the touches on a cylinder do not face each other",
                             Cylinder_Opposite_Sides'Access);
      Driver.Tests.Register ("action.search.cup", "no contact set is found on a cup", Cup_Is_Raised'Access);
      Driver.Tests.Register ("action.search.five_lobes", "the search assumes two lobes",
                             Five_Lobes_Wrap_A_Cylinder'Access);
      Driver.Tests.Register ("action.search.symmetry", "a hand is taken to repeat under a turn when one lobe is "
                             & "unlike the others, or alike lobes are not seen to repeat",
                             Symmetry_Is_Measured'Access);
      Driver.Tests.Register ("action.search.symmetric_pose", "a contact set is refused when only another turn of a "
                             & "symmetric hand can reach it", Symmetric_Pose_Reached'Access);
      Driver.Tests.Register ("action.search.plate", "a body without lobes is touched where it would rotate the thing",
                             Plate_Slides_A_Bar_Through_Its_Middle'Access);
      Driver.Tests.Register ("action.search.spin", "no contact set rotates a thing about its up",
                             Bar_Spun_About_Up'Access);
      Driver.Tests.Register ("action.search.in_way", "a motion into the table is searched for",
                             Surface_In_Way_Refused'Access);
      Driver.Tests.Register ("action.search.bound", "a contact set that failed for lack of friction is retried",
                             Failed_Friction_Not_Taken_Again'Access);
      Driver.Tests.Register ("action.search.unreachable", "an unreachable choice is made or refused without an account",
                             Unreachable_Refused_With_What_Was_Tried'Access);
   end Register;

end Driver.Action.Contact.Search.Tests;
