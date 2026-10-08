--  contact_matrix
--
--  Can the contact set be found? For every shape, every hand and every motion
--  the search (Driver.Action.Contact.Search) is asked, on the self tests'
--  simulated scene with nothing in the way and every pose reachable, for the
--  contact set that makes the motion, and the physical check
--  (Driver.Action.Contact.Wrench) judges it: where the hand touches, how many
--  touches, the friction they need at the worst the measurement allows, and
--  the least normal force. Nothing about an arm enters here: this is the
--  question of what the hand can hold or push, before the arm has to bring it
--  there (action_matrix asks that).
--
--  Shapes: bar, block, cylinder, scissors, cup. Hands: two lobes, five lobes,
--  a plate. Motions: lifted off the table, pushed along its long way, pushed
--  across it, turned about up, and lowered while held in the air (the hold
--  that puts it down or stacks it).

with Ada.Calendar;
with Ada.Command_Line;
with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Ada.Strings.Maps.Constants;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Action.Contact;
with Driver.Action.Contact.Search;
with Driver.Action.Snapshots;
with Driver.Action.Snapshots.Tests;
with Driver.Log;
with Driver.Numerics;

procedure Contact_Matrix is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Driver.Action.Contact;
   use Driver.Action.Snapshots;
   use Driver.Action.Snapshots.Tests;

   subtype Real is Driver.Real;

   package Search renames Driver.Action.Contact.Search;

   Pi : constant := Ada.Numerics.Pi;

   function Img (X : Real; D : Positive := 3) return String is (Driver.Log.Image (X, D));
   function Lower (S : String) return String is
     (Ada.Strings.Fixed.Translate (S, Ada.Strings.Maps.Constants.Lower_Case_Map));

   Scale : constant Real := (if Ada.Command_Line.Argument_Count >= 3 then Real'Value (Ada.Command_Line.Argument (3)) else 1.0);
   Pitch : constant Real := 0.005 * Scale;
   Sigma : constant Real := 0.0005 * Scale;
   Arm_Sigma : constant Real := (if Ada.Command_Line.Argument_Count >= 1 then Real'Value (Ada.Command_Line.Argument (1)) else Sigma);
   Filter    : constant String := (if Ada.Command_Line.Argument_Count >= 2 then Ada.Command_Line.Argument (2) else "");
   Tilt      : constant Real := (if Ada.Command_Line.Argument_Count >= 4 then Real'Value (Ada.Command_Line.Argument (4)) else 0.0);
   --  Where the table is in the world: its frame, turned about an arbitrary axis by Tilt and moved off the origin.
   Offset    : constant Real := (if Ada.Command_Line.Argument_Count >= 5 then Real'Value (Ada.Command_Line.Argument (5)) else 0.0);   --  the thing is seen off where it is, in sigmas
   World_Of  : constant Rigid := (Rotation => Exp (Tilt * Unit ([0.3, -0.8, 0.5])), Translation => [1.5 * Scale, -2.0 * Scale, 0.7 * Scale]);

   type Shape_Kind is (Bar, Block, Cylinder, Scissors, Cup);
   type Hand_Kind is (Two_Lobes, Five_Lobes, Plate);
   type Motion_Kind is (Up, Along, Across, Turn, Lowered);

   function Model_Of (K : Shape_Kind) return Model is
     (case K is
         when Bar      => Bar (0.2 * Scale, 0.02 * Scale, 0.02 * Scale),
         when Block    => Block (0.04 * Scale, 0.04 * Scale, 0.04 * Scale),
         when Cylinder => Upright_Cylinder (0.025 * Scale, 0.08 * Scale),
         when Scissors => Scissors (0.18 * Scale, 0.016 * Scale, 0.006 * Scale),
         when Cup      => Cup (0.03 * Scale, 0.004 * Scale, 0.08 * Scale));

   --  The thing lies at the origin of the table's frame (z up), heading About.
   function Motion_Of (M : Motion_Kind; Centre : Vec3; About : Real) return Twist is
      R    : constant Mat3 := World_Of.Rotation;
      Long : constant Vec3 := R * Vec3'[Cos (About), Sin (About), 0.0];
      Left : constant Vec3 := R * Vec3'[-Sin (About), Cos (About), 0.0];
      Flat : constant Vec3 := R * Vec3'[0.0, 0.0, 1.0];   --  the table's up
   begin
      return (case M is
                 when Up      => Slide (Flat),
                 when Along   => Slide (Long),
                 when Across  => Slide (Left),
                 when Turn    => Rotation (Flat, 1.0, Centre),
                 when Lowered => Slide (-Flat));
   end Motion_Of;

   function Anywhere (Tool : Rigid) return Boolean is (abs Tool.Translation >= 0.0);
   function Free_Anywhere (Tool : Rigid; Closers : Search.Real_Vectors.Vector) return Boolean is
     (abs Tool.Translation >= 0.0 and then Natural (Closers.Length) >= 0);

   Found_Count, Not_Found : Natural := 0;
   Missing : Ada.Strings.Unbounded.Unbounded_String;

   procedure Ask (K : Shape_Kind; Hand : Hand_Kind; M : Motion_Kind; About : Real) is
      Floor_Place : constant Rigid := World_Of;
      Height      : constant Real := (if M = Lowered then 0.05 * Scale else 0.0);
      Frame       : constant Rigid := World_Of * Rigid'(Rotation => Exp ([0.0, 0.0, About]), Translation => [0.6 * Offset * Sigma, -0.7 * Offset * Sigma, Height + 0.8 * Offset * Sigma]);
      S           : Snapshot;
      Beside      : Point_Vectors.Vector;
      Best        : Search.Candidate;
      Found       : Boolean;
      Tried       : Search.Account;
      Began       : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      Down        : constant Rigid := World_Of * Rigid'
        (Rotation => Exp ([Pi, 0.0, 0.0]), Translation => [0.01 * Scale, -0.02 * Scale, 0.2 * Scale + Height]);
      Name        : constant String := Lower (Shape_Kind'Image (K)) & (if About /= 0.0 then " across" else "")
        & " / " & Lower (Hand_Kind'Image (Hand)) & " / " & Lower (Motion_Kind'Image (M));
   begin
      if Filter'Length > 0 and then Ada.Strings.Fixed.Index (Name, Filter) = 0 then
         return;
      end if;
      S.Up := (Unit_Vector => World_Of.Rotation * Vec3'[0.0, 0.0, 1.0], Sigma => Sigma);
      S.Surfaces.Append (Floor (1, Floor_Place, Sigma));
      S.Things.Append (Thing_Of (1, Model_Of (K), Frame, Pitch, Sigma, (if M = Lowered then 0 else 1)));
      if Hand = Plate then
         S.Arms.Append (Plate_Arm (1, Down, 0.02 * Scale, Pitch, Arm_Sigma));
      else
         S.Arms.Append (Arm_Of (1, Down, Arm_Sigma));
         if Hand = Two_Lobes then
            S.Hands.Append (Gripper (1, 1, 0.08 * Scale, 0.015 * Scale, 0.01 * Scale, 0.04 * Scale, Sigma));
         else
            S.Hands.Append (Five_Lobes (1, 1, 0.06 * Scale, 0.012 * Scale, 0.01 * Scale, 0.05 * Scale, Sigma));
         end if;
      end if;
      declare
         Centre : constant Vec3 := S.Things.First_Element.Centre.Mean;
         E      : constant Search.Effector := Search.Effector_Of (S, 1);
      begin
         if Filter'Length > 0 then
            for K in 1 .. Natural'Min (3, Natural (E.Surface.Length)) loop
               Ada.Text_IO.Put_Line ("DBG surface" & K'Image & " point " & Img (E.Surface (K).Point (1)) & "," & Img (E.Surface (K).Point (2)) & ","
                                     & Img (E.Surface (K).Point (3)) & " normal " & Img (E.Surface (K).Normal (1)) & "," & Img (E.Surface (K).Normal (2)) & ","
                                     & Img (E.Surface (K).Normal (3)));
            end loop;
            Ada.Text_IO.Put_Line ("DBG effector along=" & Img (E.Along (1)) & "," & Img (E.Along (2)) & "," & Img (E.Along (3))
                                  & " depth=" & Img (E.Depth) & " band=" & Img (E.Band) & " surface" & E.Surface.Length'Image
                                  & " sigma=" & Img (E.Sigma));
         end if;
         Search.Find (Search.Shape_Of (S, 1), Beside, Search.Effector_Of (S, 1), Motion_Of (M, Centre, About),
                      World_Of.Rotation * Vec3'[0.0, 0.0, 1.0], (others => <>), Anywhere'Access, Free_Anywhere'Access, Best, Found, Tried);
      end;
      declare
         Seconds : constant Duration := Ada.Calendar."-" (Ada.Calendar.Clock, Began);
      begin
         if Found then
            Found_Count := Found_Count + 1;
            Ada.Text_IO.Put_Line (Name & ": FOUND, " & Natural'Image (Natural (Best.Touches.Length)) & " touches, friction "
                                  & Img (Best.Mu_Worst) & " at worst, force " & Img (Best.Force, 2) & " weights ["
                                  & Img (Real (Seconds), 1) & " s]");
         else
            Not_Found := Not_Found + 1;
            Ada.Strings.Unbounded.Append (Missing, "  " & Name & ASCII.LF);
            Ada.Text_IO.Put_Line (Name & ": NONE [" & Img (Real (Seconds), 1) & " s] " & Search.Say (Tried));
         end if;
      end;
      Ada.Text_IO.Flush;
   end Ask;

begin
   for K in Shape_Kind loop
      for Hand in Hand_Kind loop
         if Hand /= Five_Lobes or else K in Block | Cylinder | Cup then
            for M in Motion_Kind loop
               Ask (K, Hand, M, 0.0);
            end loop;
         end if;
      end loop;
   end loop;
   Ada.Text_IO.Put_Line ("found" & Found_Count'Image & ", none" & Not_Found'Image);
   if Ada.Strings.Unbounded.Length (Missing) > 0 then
      Ada.Text_IO.Put_Line ("none for:");
      Ada.Text_IO.Put (Ada.Strings.Unbounded.To_String (Missing));
   end if;
end Contact_Matrix;
