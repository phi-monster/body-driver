with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Ada.Text_IO;
with Driver.Json;

package body Fake_World is

   use Ada.Numerics.Long_Elementary_Functions;

   Pi : constant := Ada.Numerics.Pi;

   --  How far a hand may go into what it pressed, as a share of the push
   --  that was blocked (the lab's robot: a fraction of a millimetre).
   Give_Share : constant := 0.02;
   Give_Depth : constant := 0.003;   --  and no deeper than this into what blocks it: V1B21's finger sank 6.6 mm pushed and
                                     --  rested 2.5 mm in; a give that went on forever walked a blocked closer half its
                                     --  range in 400 beats (S4)

   --  Reading units of an open closer pressed shut, per metre of descent the
   --  surface blocked.
   Closer_Yield : constant := 1.0;

   --  The easing-back after a let-go: the first beat's overshoot, in
   --  radians, and the shake that follows, every third beat, for a while.
   Bounce_Over  : constant := 0.02;
   Bounce_Shake : constant := 1.0e-4;
   Bounce_Beats : constant := 30;

   --  The creep of a hand resting on the table, radians a beat.
   Creep_Step : constant := 1.0e-6;

   --  The gears' friction, as a share of the hardest gravity pull on any joint of the arm: a joint pulled less keeps
   --  its slack where the motor left it.
   Play_Friction : constant := 0.05;

   --  The jitter of a reading after a push, in the last bits, and the drift
   --  of every reading, radians a beat.
   Jitter_Share : constant := 1.0e-16;
   Drift_Rate   : constant := 1.2e-12;
   Jitter_Beats : constant := 5;

   --  How near a finger must stand to a thing to touch it: the give's depth, the most a press goes in.
   Grip_Margin : constant := Give_Depth;

   type Corner_Array is array (1 .. 8) of Vec3;

   --  Every random draw is its own statement: Ada leaves the order of the
   --  calls within one expression to the compiler, and the lab's two machines
   --  order them differently, so a body drawn in one expression would differ
   --  between them.
   function Gaussian (W : in out World) return Real;
   function Moved_To (X, Y, Z : Real) return Rigid;
   function About (Axis : Vec3; Angle : Real) return Mat3;
   function Img (X : Real) return String;
   function Trim (N : Integer) return String;
   procedure Add_Link (W : in out World; L : Link; Index : out Positive);
   procedure Add_Box
     (W : in out World; Name : String; Half : Vec3; Local : Rigid; Mat : Material; Link_Index, Arm : Natural;
      Finger : Boolean := False);
   procedure Add_Plane (W : in out World; Name : String; Pose : Rigid; Mat : Material);
   function Look (Position, Forward_Dir, Up : Vec3) return Rigid;
   procedure Add_Camera (W : in out World; Name : String; Link_Index, Arm : Natural; Mount : Rigid;
                         Width, Height : Positive; Focal_Share : Real);
   procedure Build_Arm (W : in out World; A : Positive; Side : Real; Width, Height : Positive; Kind : Hand_Kind;
                        With_Eye, Eye_Up : Boolean);
   procedure Add_Things (W : in out World);
   function Motion (L : Link; Q : Real) return Rigid;
   function Corners (S : Solid) return Corner_Array;
   function Inside (P : Vec3; S : Solid) return Boolean;
   function Overlap (A, B : Solid) return Boolean;
   function Penetrates (W : World; A : Positive) return Boolean;
   function Deepest (W : World; A : Positive) return Real;
   function Lowest (W : World; A : Positive) return Real;
   procedure Mark_Touching (W : in out World; A : Positive);
   function Droop (L : Link; Tool : Vec3) return Real;
   procedure Move_Arm (W : in out World; A : Positive);
   procedure Report (W : in out World);

   function Random (W : in out World) return Real is
     (Real (Ada.Numerics.Float_Random.Random (W.Rng)));

   function Random (W : in out World; Low, High : Real) return Real is
     (Low + (High - Low) * Random (W));

   function Gaussian (W : in out World) return Real is
      U1 : constant Real := Real'Max (1.0e-12, Random (W));
      U2 : constant Real := Random (W);
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Pi * U2);
   end Gaussian;

   function Moved_To (X, Y, Z : Real) return Rigid is
     (Rotation => Identity3, Translation => [X, Y, Z]);

   function About (Axis : Vec3; Angle : Real) return Mat3 is
     (Exp ([Axis (1) * Angle, Axis (2) * Angle, Axis (3) * Angle]));

   function Img (X : Real) return String is (Driver.Json.Number_Image (X));

   function Trim (N : Integer) return String is (Ada.Strings.Fixed.Trim (N'Image, Ada.Strings.Left));

   ---------------------------------------------------------------------------
   --  Building

   procedure Add_Link (W : in out World; L : Link; Index : out Positive) is
   begin
      W.Links.Append (L);
      Index := W.Links.Last_Index;
   end Add_Link;

   procedure Add_Box
     (W : in out World; Name : String; Half : Vec3; Local : Rigid; Mat : Material; Link_Index, Arm : Natural;
      Finger : Boolean := False) is
   begin
      W.Solids.Append
        (Solid'(Name => To_Unbounded_String (Name), Kind => Box, Half => Half, Local => Local, Pose => Local,
                Mat => Mat, Link => Link_Index, Arm => Arm, Finger => Finger, others => <>));
   end Add_Box;

   procedure Add_Plane (W : in out World; Name : String; Pose : Rigid; Mat : Material) is
   begin
      W.Solids.Append
        (Solid'(Name => To_Unbounded_String (Name), Kind => Plane, Half => [0.0, 0.0, 0.0], Local => Pose,
                Pose => Pose, Mat => Mat, Link => 0, Arm => 0, Finger => False, others => <>));
   end Add_Plane;

   --  A camera frame (x right, y down, z forward) at Position that looks
   --  along Forward_Dir with Up as near its top as it can be.
   function Look (Position, Forward_Dir, Up : Vec3) return Rigid is
      Z : constant Vec3 := Unit (Forward_Dir);
      X : constant Vec3 := Unit (Cross (Z, Up));
      Y : constant Vec3 := Cross (Z, X);
   begin
      return (Rotation    => [[X (1), Y (1), Z (1)], [X (2), Y (2), Z (2)], [X (3), Y (3), Z (3)]],
              Translation => Position);
   end Look;

   procedure Add_Camera (W : in out World; Name : String; Link_Index, Arm : Natural; Mount : Rigid;
                         Width, Height : Positive; Focal_Share : Real) is
      C      : Camera;
      Spread : constant Real := Random (W, 0.92, 1.08);
      Aspect : constant Real := Random (W, 0.995, 1.005);
      Dx     : constant Real := Random (W, -3.0, 3.0);
      Dy     : constant Real := Random (W, -3.0, 3.0);
   begin
      C.Name := To_Unbounded_String (Name);
      C.Link := Link_Index;
      C.Arm := Arm;
      C.Mount := Mount;
      C.Width := Width;
      C.Height := Height;
      C.Fx := Focal_Share * Real (Width) * Spread;
      C.Fy := C.Fx * Aspect;
      C.Cx := Real (Width) / 2.0 + Dx;
      C.Cy := Real (Height) / 2.0 + Dy;
      C.Shown := new Real_Array (1 .. 3 * Width * Height);
      W.Cameras.Append (C);
   end Add_Camera;

   procedure Build_Arm (W : in out World; A : Positive; Side : Real; Width, Height : Positive; Kind : Hand_Kind;
                        With_Eye, Eye_Up : Boolean) is
      Base_X : constant Real := Side * Random (W, 0.25, 0.35);
      Base_Y : constant Real := Random (W, -0.52, -0.42);
      --  The levers run from a desktop arm's (an upper arm and a forearm of ten centimetres, a wrist of two or
      --  three, the eye a few centimetres off the wrist's axes: the LeKiwi's SO-101 is of that size) to a
      --  table-top arm's (thirty): ranges that take in the small arms, never one arm's numbers.
      L0 : constant Real := Random (W, 0.04, 0.08);
      L1 : constant Real := Random (W, 0.02, 0.06);
      L2 : constant Real := Random (W, 0.10, 0.30);
      L3 : constant Real := Random (W, 0.10, 0.30);
      L4 : constant Real := Random (W, 0.03, 0.10);
      L5 : constant Real := Random (W, 0.02, 0.07);
      L6 : constant Real := Random (W, 0.02, 0.06);
      Finger_Length : constant Real := Random (W, 0.04, 0.07);
      Open_Width    : constant Real := Random (W, 0.06, 0.10);
      --  The reading unit: radians, degrees, milliradians, or a servo bus's ticks (a power of two a turn, 1024
      --  to 16384), which come in whole numbers and are sent as integers.
      Unit_Draw : constant Real := Random (W);
      Ticks     : constant Real := 2.0 ** (10 + Integer (Real'Floor (5.0 * Random (W))));
      Arm_Unit  : constant Real :=
        (if Unit_Draw < 0.3 then 1.0 elsif Unit_Draw < 0.5 then Pi / 180.0 elsif Unit_Draw < 0.7 then 0.001
         else 2.0 * Pi / Ticks);
      In_Ticks  : constant Boolean := Unit_Draw >= 0.7;
      --  A servo's tolerance about its target, in radians: an ask within it moves nothing, a move lands short of
      --  its target by up to it (the kiwi's servos land 2 to 5 ticks off and ignore asks below their dead band,
      --  R5 and R6 on 10-14). Up to half a degree; one joint in three has none (a controller that reaches its
      --  target).
      Dead_Rad  : constant Real := (if Random (W) < 1.0 / 3.0 then 0.0 else Random (W, 0.0, 0.5 * Pi / 180.0));
      Tau       : constant Real := Random (W, 0.8, 1.5);
      --  The wrist eye sits behind the palm and above it, as a wrist camera does, and looks forward and a little
      --  down: the fingertips stand in the lower third of its picture and the table fills the rest. It used to sit
      --  over the finger roots looking steeply down them, and the fingers filled its picture from top to bottom, so
      --  that the closer's sweep moved most of its cells and the body took the closer for the group carrying the eye.
      --  A small arm's eye sits a few centimetres from the wrist, and its fingers may fill its view: the driver
      --  must tell the closer from the arm there too.
      Eye_Back  : constant Real := Random (W, 0.02, 0.05);
      Eye_Above : constant Real := Random (W, 0.03, 0.13);
      Tilt      : constant Real := Random (W, 0.55, 0.7);
      --  The arm's own surfaces have texture: screws, cables, edges and prints at the scale of millimetres to
      --  centimetres, more or less contrasted; a white table or a white room leaves the arm as the only texture an
      --  eye on it, or on the other arm, has.
      Grey  : constant Material :=
        (Kind => Noise, Albedo => [0.26, 0.26, 0.29], Contrast => Random (W, 0.1, 0.5), Scale => Random (W, 100.0, 400.0),
         Gloss => 0.2);
      Black : constant Material :=
        (Kind => Flat, Albedo => [0.04, 0.04, 0.04], Contrast => 0.0, Scale => 1.0, Gloss => 0.6);
      Prefix : constant String := "arm" & Trim (A) & "/";
      Base, J1, J2, J3, J4, J5, J6, F1, F2, T1 : Positive;
      Rest : constant array (1 .. 6) of Real := [0.0, -0.3, -0.3, -0.6, 0.0, 0.0];   --  radians
      Low  : constant array (1 .. 6) of Real := [-2.9, -1.6, -2.0, -1.8, -3.0, -1.8];
      High : constant array (1 .. 6) of Real := [2.9, 1.6, 1.5, 1.8, 3.0, 1.8];
      Sag  : array (1 .. 6) of Real;
      Speed : array (1 .. 6) of Real;
      --  The servos' gears have slack: the link turns that much while the servo, whose reading the robot reports,
      --  stands still, and gravity takes it up (the kiwi's arm_a elbow: about 40 ticks of 4096, 10-14). Up to a
      --  degree a joint and one joint of the arm up to four. Drawn, with the jaw's sizes, from a generator of the
      --  arm's own, so that every other draw of the body stays as it was.
      Arm_Draws : Ada.Numerics.Float_Random.Generator;
      function Arm_Random return Real is (Real (Ada.Numerics.Float_Random.Random (Arm_Draws)));
      Play_Rad  : array (1 .. 6) of Real := [others => 0.0];

      function Joint (Name : String; Parent : Positive; Origin : Rigid; Axis : Vec3; N : Positive) return Link is
        (Name => To_Unbounded_String (Prefix & Name), Parent => Parent, Arm => A, Kind => Revolute, Origin => Origin,
         Axis => Axis, Unit => Arm_Unit, Q => Rest (N) / Arm_Unit, Target => Rest (N) / Arm_Unit,
         Lo => Low (N) / Arm_Unit, Hi => High (N) / Arm_Unit, Tau => Tau, Droop_Gain => Sag (N), Rate => Speed (N),
         Dead => Dead_Rad / Arm_Unit, Play => Play_Rad (N) / Arm_Unit, others => <>);
      --  A servo-driven closer reads its ticks too: a quarter turn of the servo from closed to open.
      Closer_Hi   : constant Real := (if In_Ticks then Ticks / 4.0 else 1.0);
      Closer_Unit : constant Real := (Open_Width / 2.0) / Closer_Hi;
      Closer_Dead : constant Real := (if In_Ticks then Dead_Rad / (2.0 * Pi / Ticks) else 0.0);
      G : Group;
   begin
      for N in Sag'Range loop
         Sag (N) := Random (W, 0.0005, 0.002);
      end loop;
      Ada.Numerics.Float_Random.Reset (Arm_Draws, Integer (W.Seed) * 41 + A);
      for N in Play_Rad'Range loop
         Play_Rad (N) := Arm_Random * Pi / 180.0;
      end loop;
      Play_Rad (1 + Natural'Min (5, Integer (Real'Floor (Arm_Random * 6.0)))) := (2.0 + 2.0 * Arm_Random) * Pi / 180.0;
      --  How fast a joint can turn: a real arm's controller limits it to about one to two and a half radians a
      --  second, 0.08 to 0.25 rad a beat at ten beats a second. Without the limit a joint followed a target of
      --  half a radian within three beats, the wrist eye's picture jumped by far more than a flow cell between two
      --  frames, and the lock-in's gains collapsed under the sweep (R4: the visible steps grew three hundredfold).
      for N in Speed'Range loop
         Speed (N) := Random (W, 0.08, 0.25);
      end loop;
      Add_Link (W, (Name => To_Unbounded_String (Prefix & "base"), Parent => 0, Arm => 0, Kind => Fixed,
                    Origin => Moved_To (Base_X, Base_Y, W.Table_Z), others => <>), Base);
      Add_Box (W, Prefix & "base", [0.045, 0.045, L0 / 2.0], Moved_To (0.0, 0.0, L0 / 2.0), Grey, Base, 0);
      Add_Link (W, Joint ("link1", Base, Moved_To (0.0, 0.0, L0), [0.0, 0.0, 1.0], 1), J1);
      Add_Box (W, Prefix & "link1", [0.03, 0.03, L1 / 2.0], Moved_To (0.0, 0.0, L1 / 2.0), Grey, J1, A);
      Add_Link (W, Joint ("link2", J1, Moved_To (0.0, 0.0, L1), [1.0, 0.0, 0.0], 2), J2);
      Add_Box (W, Prefix & "link2", [0.025, 0.025, L2 / 2.0], Moved_To (0.0, 0.0, L2 / 2.0), Grey, J2, A);
      Add_Link (W, Joint ("link3", J2, Moved_To (0.0, 0.0, L2), [1.0, 0.0, 0.0], 3), J3);
      Add_Box (W, Prefix & "link3", [0.025, L3 / 2.0, 0.025], Moved_To (0.0, L3 / 2.0, 0.0), Grey, J3, A);
      Add_Link (W, Joint ("link4", J3, Moved_To (0.0, L3, 0.0), [1.0, 0.0, 0.0], 4), J4);
      Add_Box (W, Prefix & "link4", [0.022, L4 / 2.0, 0.022], Moved_To (0.0, L4 / 2.0, 0.0), Grey, J4, A);
      Add_Link (W, Joint ("link5", J4, Moved_To (0.0, L4, 0.0), [0.0, 1.0, 0.0], 5), J5);
      Add_Box (W, Prefix & "link5", [0.02, L5 / 2.0, 0.02], Moved_To (0.0, L5 / 2.0, 0.0), Grey, J5, A);
      Add_Link (W, Joint ("tool", J5, Moved_To (0.0, L5, 0.0), [1.0, 0.0, 0.0], 6), J6);
      Add_Box (W, Prefix & "palm", [0.05, 0.012, 0.014], Moved_To (0.0, L6 + 0.012, 0.0), Grey, J6, A);
      W.Tools.Append (J6);
      W.Kinds.Append (Kind);
      W.Hand_Reach.Append (Open_Width + Finger_Length);
      G.Name := To_Unbounded_String ("arm" & Trim (A) & "_joints");
      G.Commandable := True;
      G.Integers := In_Ticks;
      declare
         Arm_Joints : constant array (1 .. 6) of Positive := [J1, J2, J3, J4, J5, J6];
      begin
         for J of Arm_Joints loop
            G.Joints.Append (J);
            G.Echoed.Append (W.Links (J).Target);
         end loop;
      end;
      W.Groups.Append (G);
      case Kind is
         when Pair | Pair_And_Thumb =>
            --  Two plates sliding apart along the tool's x, one joint read (the
            --  share of the way open, 1 open, 0 closed) and its mirror; with a
            --  thumb, a plate that comes down on them from above on a second
            --  channel of the same group.
            Add_Link (W, (Name => To_Unbounded_String (Prefix & "finger1"), Parent => J6, Arm => A, Kind => Prismatic,
                          Origin => Moved_To (0.0, L6 + 0.024, 0.0), Axis => [1.0, 0.0, 0.0], Unit => Closer_Unit,
                          Q => Closer_Hi, Target => Closer_Hi, Lo => 0.0, Hi => Closer_Hi, Tau => Tau, Dead => Closer_Dead,
                          others => <>), F1);
            Add_Box (W, Prefix & "finger1", [0.005, Finger_Length / 2.0, 0.01],
                     Moved_To (0.005, Finger_Length / 2.0, 0.0), Black, F1, A, Finger => True);
            Add_Link (W, (Name => To_Unbounded_String (Prefix & "finger2"), Parent => J6, Arm => A, Kind => Prismatic,
                          Origin => Moved_To (0.0, L6 + 0.024, 0.0), Axis => [-1.0, 0.0, 0.0], Unit => Closer_Unit,
                          Q => Closer_Hi, Target => Closer_Hi, Lo => 0.0, Hi => Closer_Hi, Tau => Tau, Mirror => F1,
                          others => <>), F2);
            Add_Box (W, Prefix & "finger2", [0.005, Finger_Length / 2.0, 0.01],
                     Moved_To (-0.005, Finger_Length / 2.0, 0.0), Black, F2, A, Finger => True);
            W.Closers.Append (F1);
            G := (Name => To_Unbounded_String ("arm" & Trim (A) & "_closer"), Commandable => True, Integers => In_Ticks,
                  others => <>);
            G.Joints.Append (F1);
            G.Echoed.Append (Closer_Hi);
            if Kind = Pair_And_Thumb then
               Add_Link (W, (Name => To_Unbounded_String (Prefix & "thumb"), Parent => J6, Arm => A, Kind => Prismatic,
                             Origin => Moved_To (0.0, L6 + 0.024, 0.005), Axis => [0.0, 0.0, 1.0], Unit => 0.04,
                             Q => 1.0, Target => 1.0, Lo => 0.0, Hi => 1.0, Tau => Tau, others => <>), T1);
               Add_Box (W, Prefix & "thumb", [0.015, Finger_Length / 2.0, 0.005],
                        Moved_To (0.0, Finger_Length / 2.0, 0.005), Black, T1, A, Finger => True);
               G.Joints.Append (T1);
               G.Echoed.Append (1.0);
            end if;
            W.Groups.Append (G);
         when Hook =>
            --  No closer: one prong slanting down from the palm, which presses and hooks and never closes.
            Add_Link (W, (Name => To_Unbounded_String (Prefix & "hook"), Parent => J6, Arm => A, Kind => Fixed,
                          Origin => (Rotation => About ([1.0, 0.0, 0.0], -1.0), Translation => [0.0, L6 + 0.024, 0.0]),
                          others => <>), F1);
            Add_Box (W, Prefix & "hook", [0.008, Finger_Length / 2.0, 0.006], Moved_To (0.0, Finger_Length / 2.0, 0.0),
                     Black, F1, A, Finger => True);
            W.Closers.Append (0);
         when Fingers =>
            --  Five fingers side by side, each turning about the tool's x to
            --  curl down, each read as a group of its own (as a humanoid's
            --  hand reports them): 0 straight, 1 curled.
            W.Closers.Append (0);
            for F in 1 .. 5 loop
               declare
                  Fl : Positive;
               begin
                  Add_Link (W, (Name => To_Unbounded_String (Prefix & "finger" & Trim (F)), Parent => J6, Arm => A,
                                Kind => Revolute, Origin => Moved_To (Real (F - 3) * 0.02, L6 + 0.024, 0.0),
                                Axis => [-1.0, 0.0, 0.0], Unit => 1.5, Q => 0.0, Target => 0.0, Lo => 0.0, Hi => 1.0,
                                Tau => Tau, others => <>), Fl);
                  Add_Box (W, Prefix & "finger" & Trim (F), [0.007, Finger_Length / 2.0, 0.006],
                           Moved_To (0.0, Finger_Length / 2.0, 0.0), Black, Fl, A, Finger => True);
                  G := (Name => To_Unbounded_String ("arm" & Trim (A) & "_finger" & Trim (F)), Commandable => True,
                        others => <>);
                  G.Joints.Append (Fl);
                  G.Echoed.Append (0.0);
                  W.Groups.Append (G);
               end;
            end loop;
         when Jaw =>
            --  One jaw swung onto a jaw that does not move, as a desktop arm's gripper is (the LeKiwi's SO-101,
            --  10-15). The fixed jaw stands on the palm along the hand's way, its inner face on the hand's middle;
            --  the moving jaw turns about an upright axis through the root of its own inner face, so that closed (its
            --  low end) it lies flat on the fixed jaw, and open its tip stands the swing's chord away. The servo's stop
            --  at closed is the fixed jaw itself. Both jaws are black and ribbed across, as the kiwi's are.
            declare
               Jaw_Length : constant Real := 0.05 + 0.03 * Arm_Random;
               Jaw_Thick  : constant Real := 0.008 + 0.006 * Arm_Random;
               Jaw_Height : constant Real := 0.012 + 0.010 * Arm_Random;
               --  Radians from closed to open: the kiwi's moving jaw opens about a third of a radian to half of one
               --  (its wrist frames, 10-14); the fake's jaw opened 0.7 to 1.1 and swung out of its eye's picture.
               Swing      : constant Real := 0.35 + 0.35 * Arm_Random;
               --  Dark grey with its ribs showing, as the kiwi's show in its wrist frames (10-14); at 0.05 and a third to
               --  seven tenths of contrast the jaws rendered black, nothing on them for a matcher to follow (a05).
               Ribbed     : constant Material :=
                 (Kind => Stripes, Albedo => [0.10, 0.10, 0.10], Contrast => 0.6 + 0.35 * Arm_Random,
                  Scale => 200.0 + 200.0 * Arm_Random, Gloss => 0.4);
               Jaw_Hi     : constant Real := (if In_Ticks then Swing / (2.0 * Pi / Ticks) else 1.0);
               Fixed_Jaw  : Positive;
            begin
               Add_Link (W, (Name => To_Unbounded_String (Prefix & "jaw_fixed"), Parent => J6, Arm => A, Kind => Fixed,
                             Origin => Moved_To (0.0, L6 + 0.024, 0.0), others => <>), Fixed_Jaw);
               Add_Box (W, Prefix & "jaw_fixed", [Jaw_Thick / 2.0, Jaw_Length / 2.0, Jaw_Height / 2.0],
                        Moved_To (-Jaw_Thick / 2.0, Jaw_Length / 2.0, 0.0), Ribbed, Fixed_Jaw, A, Finger => True);
               Add_Link (W, (Name => To_Unbounded_String (Prefix & "jaw"), Parent => J6, Arm => A, Kind => Revolute,
                             Origin => Moved_To (0.0, L6 + 0.024, 0.0), Axis => [0.0, 0.0, -1.0], Unit => Swing / Jaw_Hi,
                             Q => Jaw_Hi, Target => Jaw_Hi, Lo => 0.0, Hi => Jaw_Hi, Tau => Tau, Dead => Closer_Dead,
                             others => <>), F1);
               Add_Box (W, Prefix & "jaw", [Jaw_Thick / 2.0, Jaw_Length / 2.0, Jaw_Height / 2.0],
                        Moved_To (Jaw_Thick / 2.0, Jaw_Length / 2.0, 0.0), Ribbed, F1, A, Finger => True);
               W.Closers.Append (F1);
               W.Hand_Reach.Replace_Element (A, Jaw_Length * (1.0 + Sin (Swing)));
               G := (Name => To_Unbounded_String ("arm" & Trim (A) & "_closer"), Commandable => True,
                     Integers => In_Ticks, others => <>);
               G.Joints.Append (F1);
               G.Echoed.Append (Jaw_Hi);
               W.Groups.Append (G);
            end;
         when Drawn =>
            raise Program_Error with "a hand to build must be drawn first";
      end case;
      --  The wrist eye: behind the palm and above it, looking forward and a
      --  little down, so that the fingertips stand in the lower third of its
      --  picture and the table beyond them fills the rest; or ahead at the wall.
      if With_Eye then
         declare
            --  A desktop gripper's camera sits on the gripper over the roots of its jaws, the palm behind it, and
            --  looks down along them: they come into the bottom of its picture from its border and fill two fifths of
            --  it, open and closed (the kiwi's wrist eyes, 10-14; its lens is a wide one, 0.45 of the picture's width
            --  here, where every other eye has 0.62). Placed there with the narrower lens and the jaw swung 0.7 to 1.1
            --  rad, the open jaw left the picture (a03 on 10-16: its tip a sliver on the picture's bottom row, 41 mm
            --  off the truth); placed behind the roots and over the palm, the palm's top filled the picture's bottom
            --  and the jaws stood on it, not on the border, as they do not on the kiwi (a04 to a07 on 10-16).
            Near  : constant Boolean := Kind = Jaw;
            Ahead : constant Real := (if Near then 0.024 + 0.005 else -Eye_Back);
            Above : constant Real := (if Near then 0.024 + 0.015 * Arm_Random else Eye_Above);
            Pitch : constant Real := (if Eye_Up then -0.3 elsif Near then 0.6 + 0.3 * Arm_Random else Tilt);
         begin
            Add_Camera (W, "eye_arm" & Trim (A), J6, A,
                        Look ([0.0, L6 + Ahead, Above], [0.0, Cos (Pitch), -Sin (Pitch)], [0.0, 0.0, 1.0]),
                        Width, Height, (if Near then 0.45 else 0.62));
         end;
      end if;
   end Build_Arm;

   --  Things on the table: a flat slab of fine stripes in front of the first
   --  arm (the keyboard the lab's table had), and a few boxes.
   procedure Add_Things (W : in out World) is
      --  Nothing is placed within a hand's reach of its tool, over the table: the boot starts with the hands
      --  free, and nothing stands between the fingers for a closer's sweep to push about (the first box run had a
      --  box under arm 1's hand, and the closer's pushes moved most of its eye's picture).
      function Under_A_Hand (S : Solid) return Boolean is
      begin
         for A in 1 .. W.Arms loop
            declare
               T  : constant Vec3 := W.Links (W.Tools (A)).Pose.Translation;
               Dx : constant Real := S.Pose.Translation (1) - T (1);
               Dy : constant Real := S.Pose.Translation (2) - T (2);
            begin
               if Sqrt (Dx ** 2 + Dy ** 2) < W.Hand_Reach (A) + Real'Max (S.Half (1), S.Half (2)) then
                  return True;
               end if;
            end;
         end loop;
         return False;
      end Under_A_Hand;
      Keys  : constant Material :=
        (Kind => Stripes, Albedo => [0.15, 0.15, 0.17], Contrast => 0.6, Scale => 125.0, Gloss => 0.1);
      Base1 : constant Vec3 := W.Links (1).Origin.Translation;
      Yaw   : constant Real := Random (W, -0.3, 0.3);
      Dx    : constant Real := Random (W, 0.08, 0.16) * (if Base1 (1) < 0.0 then 1.0 else -1.0);
      Dy    : constant Real := Random (W, 0.30, 0.40);
   begin
      Add_Box (W, "slab", [0.15, 0.055, 0.006],
               (Rotation    => About ([0.0, 0.0, 1.0], Yaw),
                Translation => [Base1 (1) + Dx, Base1 (2) + Dy, W.Table_Z + 0.006]),
               Keys, 0, 0);
      for N in 1 .. 5 loop
         declare
            Hx   : constant Real := Random (W, 0.02, 0.08);
            Hy   : constant Real := Random (W, 0.02, 0.08);
            Hz   : constant Real := Random (W, 0.01, 0.06);
            Pick : constant Real := Random (W);
            R    : constant Real := Random (W, 0.1, 0.9);
            Gr   : constant Real := Random (W, 0.1, 0.9);
            B    : constant Real := Random (W, 0.1, 0.9);
            Con  : constant Real := Random (W, 0.3, 0.7);
            Sc   : constant Real := Random (W, 30.0, 150.0);
            Gl   : constant Real := Random (W, 0.0, 0.3);
            Mat  : Material;
            Placed : Boolean := False;
         begin
            if Pick < 0.4 then
               Mat := (Kind => Noise, Albedo => [R, Gr, B], Contrast => Con, Scale => Sc, Gloss => 0.1);
            elsif Pick < 0.7 then
               Mat := (Kind => Flat, Albedo => [R, Gr, B], Contrast => 0.0, Scale => 1.0, Gloss => Gl);
            else
               Mat := (Kind => Stripes, Albedo => [R, Gr, B], Contrast => Con, Scale => Sc, Gloss => 0.1);
            end if;
            --  A thing is placed where no part of the body stands at rest: the
            --  boot starts with the hands free, as the lab's layouts have them.
            for Try in 1 .. 20 loop
               exit when Placed;
               declare
                  X    : constant Real := Random (W, -0.5, 0.5);
                  Y    : constant Real := Random (W, -0.15, 0.45);
                  Turn : constant Real := Random (W, 0.0, Pi);
                  Pose : constant Rigid :=
                    (Rotation => About ([0.0, 0.0, 1.0], Turn), Translation => [X, Y, W.Table_Z + Hz]);
                  S    : constant Solid :=
                    (Name => To_Unbounded_String ("thing" & Trim (N)), Kind => Box, Half => [Hx, Hy, Hz], Local => Pose,
                     Pose => Pose, Mat => Mat, Link => 0, Arm => 0, Finger => False, others => <>);
               begin
                  if not (for some O of W.Solids => O.Link > 0 and then Overlap (S, O)) and then not Under_A_Hand (S) then
                     W.Solids.Append (S);
                     Placed := True;
                  end if;
               end;
            end loop;
         end;
      end loop;
      --  A black box within reach of each arm that ends in a jaw, just past where its hand rests: the thing the
      --  person asks for (the kiwi's mouse, 10-15), narrow enough across for the jaw to take.
      for A in 1 .. W.Arms loop
         if W.Kinds (A) = Jaw then
            declare
               Tool   : constant Vec3 := W.Links (W.Tools (A)).Pose.Translation;
               Half   : constant Vec3 := [0.02, 0.04, 0.016];
               Mat    : constant Material :=
                 (Kind => Noise, Albedo => [0.06, 0.06, 0.07], Contrast => 0.25, Scale => 150.0, Gloss => 0.3);
               Placed : Boolean := False;
            begin
               for Try in 1 .. 20 loop
                  exit when Placed;
                  declare
                     Beyond : constant Real := Random (W, 0.02, 0.08);
                     Out_By : constant Real := W.Hand_Reach (A) + Half (2) + Beyond;
                     Side   : constant Real := Random (W, -0.06, 0.06);
                     Turn   : constant Real := Random (W, -0.4, 0.4);
                     Pose   : constant Rigid :=
                       (Rotation    => About ([0.0, 0.0, 1.0], Turn),
                        Translation => [Tool (1) + Side, Tool (2) + Out_By, W.Table_Z + Half (3)]);
                     S      : constant Solid :=
                       (Name => To_Unbounded_String ("target" & Trim (A)), Kind => Box, Half => Half, Local => Pose,
                        Pose => Pose, Mat => Mat, Link => 0, Arm => 0, Finger => False, others => <>);
                  begin
                     if not (for some O of W.Solids => O.Kind = Box and then Overlap (S, O)) and then not Under_A_Hand (S)
                     then
                        W.Solids.Append (S);
                        Placed := True;
                     end if;
                  end;
               end loop;
            end;
         end if;
      end loop;
   end Add_Things;

   procedure Build (W : in out World; Seed : Natural; Width, Height : Positive; T : Template) is
      Wood : Material :=
        (Kind => Grain, Albedo => [0.62, 0.50, 0.38], Contrast => 0.0, Scale => 60.0,
         Gloss => 0.05);
      Wall : Material :=
        (Kind => Noise, Albedo => [0.75, 0.74, 0.70], Contrast => 0.0, Scale => 8.0, Gloss => 0.0);
      Dark : constant Material :=
        (Kind => Flat, Albedo => [0.2, 0.2, 0.2], Contrast => 0.0, Scale => 1.0, Gloss => 0.0);
   begin
      Ada.Numerics.Float_Random.Reset (W.Rng, Integer (Seed));
      W.Seed := Seed;
      W.Table_Z := Random (W, 0.70, 0.84);
      --  How much texture the table and the walls have is drawn too, down to none: a white table, a white room
      --  (the owner's: the environment is given, and the arm's own surfaces are then the only texture).
      Wood.Contrast := (if T.Plain_Table then 0.0 else Random (W, 0.0, 0.3));
      Wall.Contrast := Random (W, 0.0, 0.35);
      --  A white room: the table and the walls near white, faint broad veins their only texture (the kiwi's room,
      --  10-14), from the same draws as their texture would have been.
      if T.White_Room then
         Wood := (Kind => Noise, Albedo => [0.88, 0.88, 0.86], Contrast => Wood.Contrast / 6.0, Scale => 6.0, Gloss => 0.1);
         Wall := (Kind => Noise, Albedo => [0.90, 0.90, 0.89], Contrast => Wall.Contrast / 10.0, Scale => 4.0,
                  Gloss => 0.0);
      end if;
      Add_Plane (W, "table", Moved_To (0.0, 0.0, W.Table_Z), Wood);
      Add_Plane (W, "floor", Moved_To (0.0, 0.0, 0.0), Dark);
      Add_Plane (W, "wall", (Rotation => About ([1.0, 0.0, 0.0], Pi / 2.0), Translation => [0.0, 1.3, 0.8]), Wall);
      Add_Plane (W, "wall_left", (Rotation => About ([0.0, 1.0, 0.0], Pi / 2.0), Translation => [-1.6, 0.0, 0.8]),
                 Wall);
      Add_Plane (W, "wall_right", (Rotation => About ([0.0, 1.0, 0.0], -Pi / 2.0), Translation => [1.6, 0.0, 0.8]),
                 Wall);
      W.Arms := 2;
      declare
         Kinds : Hand_Kinds := T.Hands;
         Draws : Ada.Numerics.Float_Random.Generator;
      begin
         --  The hands have a generator of their own: forcing one leaves every other draw of the body as it was.
         Ada.Numerics.Float_Random.Reset (Draws, Integer (Seed) * 31 + 7);
         for A in Kinds'Range loop
            declare
               Pick : constant Real := Real (Ada.Numerics.Float_Random.Random (Draws));
            begin
               if Kinds (A) = Drawn then
                  Kinds (A) := (if Pick < 0.35 then Pair elsif Pick < 0.55 then Jaw elsif Pick < 0.7 then Hook
                                elsif Pick < 0.85 then Pair_And_Thumb else Fingers);
               end if;
            end;
         end loop;
         Build_Arm (W, 1, -1.0, Width, Height, Kinds (1), T.Eyeless /= 1, T.Eye_Up = 1);
         Build_Arm (W, 2, 1.0, Width, Height, Kinds (2), T.Eyeless /= 2, T.Eye_Up = 2);
      end;
      --  An arm whose bus is dead stands at rest where it was built, seen and never heard from.
      if T.Dead_Arm > 0 then
         for G of W.Groups loop
            if Ada.Strings.Fixed.Index (To_String (G.Name), "arm" & Trim (T.Dead_Arm) & "_") = 1 then
               G.Silent := True;
            end if;
         end loop;
      end if;
      --  The fixed eye above and behind the arms, looking down at the table between them.
      declare
         Px  : constant Real := Random (W, -0.05, 0.05);
         Py  : constant Real := Random (W, -0.5, -0.4);
         Dz  : constant Real := Random (W, 0.5, 0.65);
         Pz  : constant Real := W.Table_Z + Dz;
         Ay  : constant Real := Random (W, 0.0, 0.15);
         Pos : constant Vec3 := [Px, Py, Pz];
         Aim : constant Vec3 := [0.0, Ay, W.Table_Z];
      begin
         Add_Camera (W, "eye_fixed", 0, 0, Look (Pos, Minus (Aim, Pos), [0.0, 0.0, 1.0]), Width, Height, 0.45);
      end;
      Forward (W);
      --  Two constants the robot reports beside its readings, as the lab's
      --  simulator does (a rate and an index).
      declare
         G : Group;
      begin
         G := (Name => To_Unbounded_String ("frequency"), Scalar => True, others => <>);
         G.Values.Append (10.0);
         W.Groups.Append (G);
         G := (Name => To_Unbounded_String ("env_idx"), Scalar => True, others => <>);
         G.Values.Append (0.0);
         W.Groups.Append (G);
      end;
      Forward (W);
      --  The body rests with its hands clear of the table: the elbow is raised
      --  until the lowest point of the arm stands well above it, and the
      --  echoes of the commands start at the rest positions.
      for A in 1 .. W.Arms loop
         declare
            Elbow : constant Positive := W.Tools (A) - 3;
         begin
            for Lift in 1 .. 80 loop
               exit when Lowest (W, A) >= W.Table_Z + 0.04;
               W.Links (Elbow).Q := W.Links (Elbow).Q + 0.02 / W.Links (Elbow).Unit;
               W.Links (Elbow).Target := W.Links (Elbow).Q;
               Forward (W);
            end loop;
         end;
      end loop;
      --  The things go on the table once the body rests as it will boot: none within a hand's reach.
      Add_Things (W);
      for G of W.Groups loop
         for I in 1 .. Natural (G.Joints.Length) loop
            G.Echoed (I) := W.Links (G.Joints (I)).Target;
         end loop;
      end loop;
      Report (W);
   end Build;

   ---------------------------------------------------------------------------
   --  Poses

   function Motion (L : Link; Q : Real) return Rigid is
   begin
      case L.Kind is
         when Fixed =>
            return Identity;
         when Revolute =>
            return (Rotation => About (L.Axis, Q * L.Unit), Translation => Zero3);
         when Prismatic =>
            return (Rotation => Identity3,
                    Translation => [L.Axis (1) * Q * L.Unit, L.Axis (2) * Q * L.Unit, L.Axis (3) * Q * L.Unit]);
      end case;
   end Motion;

   procedure Forward (W : in out World) is
   begin
      for I in W.Links.First_Index .. W.Links.Last_Index loop
         declare
            L      : Link renames W.Links (I);
            Parent : constant Rigid := (if L.Parent = 0 then Identity else W.Links (L.Parent).Pose);
            --  The link stands where its servo reads less the slack gravity takes up.
            Q      : constant Real := (if L.Mirror = 0 then L.Q - L.Play_Offset else W.Links (L.Mirror).Q);
         begin
            L.Pose := Parent * L.Origin * Motion (L, Q);
         end;
      end loop;
      for S of W.Solids loop
         if S.Link > 0 then
            S.Pose := W.Links (S.Link).Pose * S.Local;
         elsif S.Held_By > 0 then
            S.Pose := W.Links (W.Tools (S.Held_By)).Pose * S.Held_Rel;
         end if;
      end loop;
      for C of W.Cameras loop
         C.Pose := (if C.Link = 0 then C.Mount else W.Links (C.Link).Pose * C.Mount);
      end loop;
   end Forward;

   ---------------------------------------------------------------------------
   --  Contacts

   function Corners (S : Solid) return Corner_Array is
      R : Corner_Array;
      K : Positive := 1;
   begin
      for X in 0 .. 1 loop
         for Y in 0 .. 1 loop
            for Z in 0 .. 1 loop
               R (K) := S.Pose * Vec3'[(if X = 0 then -S.Half (1) else S.Half (1)),
                                       (if Y = 0 then -S.Half (2) else S.Half (2)),
                                       (if Z = 0 then -S.Half (3) else S.Half (3))];
               K := K + 1;
            end loop;
         end loop;
      end loop;
      return R;
   end Corners;

   function Inside (P : Vec3; S : Solid) return Boolean is
      Q : constant Vec3 := Inverse (S.Pose) * P;
   begin
      return abs Q (1) < S.Half (1) and then abs Q (2) < S.Half (2) and then abs Q (3) < S.Half (3);
   end Inside;

   function Overlap (A, B : Solid) return Boolean is
   begin
      return (for some P of Corners (A) => Inside (P, B)) or else (for some P of Corners (B) => Inside (P, A));
   end Overlap;

   --  The link arm A hangs from: its first link's parent (the base, a link of no arm). The arm's solids are
   --  never in contact with the mount's: its first link's box stands on the base's box face to face by
   --  construction, and a corner on a face is inside it or not by the rounding of one pose product. Seed 3 on
   --  10-14: arm 2 "penetrated" its own base by a hair at every beat, every step of its joints was cut to the
   --  Give quirk's share, and a closer asked a step crept to it over 440 beats while arm 1, the same body
   --  mirrored, answered in five.
   function Mount_Of (W : World; A : Positive) return Natural is
   begin
      for L of W.Links loop
         if L.Arm = A then
            return L.Parent;
         end if;
      end loop;
      return 0;
   end Mount_Of;

   --  Whether any of arm A's solids goes into the table or into a thing of the world (not its own mount).
   function Penetrates (W : World; A : Positive) return Boolean is
      Mount : constant Natural := Mount_Of (W, A);
   begin
      for S of W.Solids loop
         if S.Arm = A then
            for P of Corners (S) loop
               if P (3) < W.Table_Z - 1.0e-7 then
                  return True;
               end if;
            end loop;
            for O of W.Solids loop
               if O.Arm = 0 and then O.Kind = Box and then O.Link /= Mount and then O.Held_By /= A and then Overlap (S, O)
               then
                  return True;
               end if;
            end loop;
         end if;
      end loop;
      --  What the arm holds goes with it, into the table or not.
      for O of W.Solids loop
         if O.Held_By = A then
            for P of Corners (O) loop
               if P (3) < W.Table_Z - 1.0e-7 then
                  return True;
               end if;
            end loop;
         end if;
      end loop;
      return False;
   end Penetrates;

   --  How far any of arm A's solids goes into the table or into a thing of the world: the deepest corner, metres;
   --  zero when none does.
   function Deepest (W : World; A : Positive) return Real is
      Mount : constant Natural := Mount_Of (W, A);
      D     : Real := 0.0;
   begin
      for S of W.Solids loop
         if S.Arm = A then
            for P of Corners (S) loop
               D := Real'Max (D, W.Table_Z - P (3));
               for O of W.Solids loop
                  if O.Arm = 0 and then O.Kind = Box and then O.Link /= Mount and then O.Held_By /= A then
                     declare
                        Q : constant Vec3 := Inverse (O.Pose) * P;
                     begin
                        if abs Q (1) < O.Half (1) and then abs Q (2) < O.Half (2) and then abs Q (3) < O.Half (3) then
                           D := Real'Max (D, Real'Min (O.Half (1) - abs Q (1),
                                                     Real'Min (O.Half (2) - abs Q (2), O.Half (3) - abs Q (3))));
                        end if;
                     end;
                  end if;
               end loop;
            end loop;
         end if;
      end loop;
      return D;
   end Deepest;

   --  The lowest point of arm A's solids in the world.
   function Lowest (W : World; A : Positive) return Real is
      Z : Real := Real'Last;
   begin
      for S of W.Solids loop
         if S.Arm = A then
            for P of Corners (S) loop
               Z := Real'Min (Z, P (3));
            end loop;
         end if;
      end loop;
      return Z;
   end Lowest;

   procedure Mark_Touching (W : in out World; A : Positive) is
   begin
      for L of W.Links loop
         if L.Arm = A then
            L.Touching := False;
         end if;
      end loop;
      for S of W.Solids loop
         if S.Arm = A then
            declare
               Hit : Boolean := (for some P of Corners (S) => P (3) < W.Table_Z + 1.0e-7);
            begin
               if not Hit then
                  for O of W.Solids loop
                     if O.Arm = 0 and then O.Kind = Box and then O.Held_By /= A and then Overlap (S, O) then
                        Hit := True;
                     end if;
                  end loop;
               end if;
               if Hit then
                  W.Links (S.Link).Touching := True;
               end if;
            end;
         end if;
      end loop;
   end Mark_Touching;

   ---------------------------------------------------------------------------
   --  Motion

   procedure Command (W : in out World; G : Positive; Values : Real_Array) is
      Grp : Group renames W.Groups (G);
   begin
      if not Grp.Commandable or else Grp.Silent or else Values'Length /= Natural (Grp.Joints.Length) then
         return;
      end if;
      for I in 1 .. Natural (Grp.Joints.Length) loop
         Grp.Echoed (I) := Values (Values'First + I - 1);
         W.Pending.Append
           (Due_Target'(Due => W.Beat + 1 + W.Quirk.Delay_Beats, Joint => Grp.Joints (I),
                        Value => Values (Values'First + I - 1)));
      end loop;
   end Command;

   --  The sag of a loaded revolute joint, in reading units: its gain times the
   --  moment of a unit weight at the arm's tool about its axis.
   function Droop (L : Link; Tool : Vec3) return Real is
      Origin : constant Vec3 := L.Pose.Translation;
      Axis   : constant Vec3 := Rotate (L.Pose, L.Axis);
      R      : constant Vec3 := Minus (Tool, Origin);
      Moment : constant Vec3 := Cross (R, [0.0, 0.0, -1.0]);
   begin
      if L.Kind /= Revolute then
         return 0.0;
      end if;
      return L.Droop_Gain * Dot (Moment, Axis) / L.Unit;
   end Droop;

   procedure Move_Arm (W : in out World; A : Positive) is
      Old          : array (W.Links.First_Index .. W.Links.Last_Index) of Real;
      Wanted       : array (W.Links.First_Index .. W.Links.Last_Index) of Real;
      Was_Touching : Boolean := False;
      Tool_Before  : constant Vec3 := W.Links (W.Tools (A)).Pose.Translation;
      Sunk         : constant Real := Deepest (W, A);   --  how far the arm already gave into what blocks it
      Closer       : constant Natural := W.Closers (A);
      Z_Wanted     : Real;
      Lo, Hi       : Real := 0.0;

      procedure Place (Share : Real);
      procedure Place (Share : Real) is
      begin
         for I in Old'Range loop
            if W.Links (I).Arm = A then
               W.Links (I).Q := Old (I) + Share * (Wanted (I) - Old (I));
            end if;
         end loop;
         Forward (W);
      end Place;
   begin
      --  Gravity takes up a joint's slack on the side it pulls the tool's load to when it pulls harder than the
      --  gears' friction holds, a twentieth of the hardest pull on any joint of the arm; a joint pulled less (a
      --  wrist roll balanced on its axis) stays where the motor last pushed it in its slack. Snapping to the side of
      --  the load's sign, a balanced wrist roll flipped its whole play at every command of a ten-millionth of a
      --  unit and turned the wrist eye's view by 0.8 degrees, and the boot saw it as the body moving (10-15).
      declare
         Hardest : Real := 0.0;
      begin
         if W.Quirk.Play then
            for L of W.Links loop
               if L.Arm = A and then L.Kind = Revolute and then L.Play > 0.0 then
                  Hardest := Real'Max (Hardest, abs Droop ((L with delta Droop_Gain => 1.0), Tool_Before));
               end if;
            end loop;
         end if;
         for I in W.Links.First_Index .. W.Links.Last_Index loop
            declare
               L : Link renames W.Links (I);
            begin
               if L.Arm = A and then L.Kind = Revolute and then L.Play > 0.0 and then W.Quirk.Play then
                  declare
                     Load : constant Real := Droop ((L with delta Droop_Gain => 1.0), Tool_Before);
                  begin
                     if abs Load > Play_Friction * Hardest then
                        L.Play_Offset := (if Load > 0.0 then L.Play / 2.0 else -L.Play / 2.0);
                     end if;
                  end;
               end if;
            end;
         end loop;
      end;
      for I in Old'Range loop
         Old (I) := W.Links (I).Q;
         Wanted (I) := W.Links (I).Q;
         if W.Links (I).Arm = A and then W.Links (I).Touching then
            Was_Touching := True;
         end if;
      end loop;
      --  Each joint follows its target, short of it by its sag, and no further than its ends.
      for I in Old'Range loop
         declare
            L : Link renames W.Links (I);
         begin
            if L.Arm = A and then L.Kind /= Fixed and then L.Mirror = 0 then
               declare
                  Sag   : constant Real := (if W.Quirk.Droop then Droop (L, Tool_Before) else 0.0);
                  T     : constant Real := L.Target - Sag;
                  Share : constant Real := 1.0 - Exp (-1.0 / L.Tau);
                  From  : constant Real := L.Q - L.Bounce;
                  --  A servo's tolerance (Dead): within it of its target the joint does not move at all, and a
                  --  move aims short of the target by its landing, on the side it comes from.
                  Dead_Here : constant Real := (if W.Quirk.Dead then L.Dead else 0.0);
                  Landing   : constant Real := (if W.Quirk.Dead then L.Landing else 0.0);
                  Within : constant Boolean := abs (T - From) <= Dead_Here;
                  Aim   : constant Real := (if Within then From elsif T > From then T - Landing else T + Landing);
                  --  The first-order step, no faster than the joint turns.
                  Step  : constant Real :=
                    (if L.Rate > 0.0 and then abs ((Aim - From) * Share) > L.Rate / L.Unit
                     then (if Aim > From then L.Rate / L.Unit else -L.Rate / L.Unit)
                     else (Aim - From) * Share);
                  Q     : constant Real := From + Step;
               begin
                  Wanted (I) := Real'Max (L.Lo, Real'Min (L.Hi, Q));
               end;
            end if;
         end;
      end loop;
      --  Where the hand meets the table or a thing, the arm stops: the
      --  largest share of the way that goes into nothing, then the give.
      Place (1.0);
      Z_Wanted := W.Links (W.Tools (A)).Pose.Translation (3);
      if Penetrates (W, A) then
         Hi := 1.0;
         for Halving in 1 .. 12 loop
            declare
               Mid : constant Real := (Lo + Hi) / 2.0;
            begin
               Place (Mid);
               if Penetrates (W, A) then
                  Hi := Mid;
               else
                  Lo := Mid;
               end if;
            end;
         end loop;
         Place (Lo + (if W.Quirk.Give and then Sunk < Give_Depth then Give_Share * (1.0 - Lo) else 0.0));
         --  An open closer pressed on the surface is pressed shut by the descent the surface blocked.
         if W.Quirk.Soft_Closer and then Closer > 0 then
            declare
               Blocked    : constant Real := W.Links (W.Tools (A)).Pose.Translation (3) - Z_Wanted;
               Finger_Hit : Boolean := False;
            begin
               for S of W.Solids loop
                  if S.Arm = A and then S.Finger and then (for some P of Corners (S) => P (3) < W.Table_Z + 1.0e-4) then
                     Finger_Hit := True;
                  end if;
               end loop;
               if Finger_Hit and then Blocked > 0.0 then
                  declare
                     C : Link renames W.Links (Closer);
                  begin
                     C.Q := Real'Max (C.Lo, C.Q - Closer_Yield * Blocked / C.Unit);
                  end;
                  Forward (W);
               end if;
            end;
         end if;
         Mark_Touching (W, A);
      else
         for I in Old'Range loop
            if W.Links (I).Arm = A then
               W.Links (I).Touching := False;
            end if;
         end loop;
      end if;
      --  Let go of a press: the arm eases back past where it was asked, then shakes a while.
      declare
         Now_Touching : constant Boolean := (for some L of W.Links => L.Arm = A and then L.Touching);
      begin
         for I in Old'Range loop
            if W.Links (I).Arm = A and then W.Links (I).Kind = Revolute then
               declare
                  Shake : Real := 0.0;
                  Way   : constant Real := (if Wanted (I) >= Old (I) then 1.0 else -1.0);
                  Age   : Natural := W.Links (I).Bounce_Age;
               begin
                  if W.Quirk.Bounce and then Was_Touching and then not Now_Touching then
                     Age := 0;
                  end if;
                  if Age < Bounce_Beats then
                     Shake := (if Age = 0 then Bounce_Over * Way
                               elsif (Age / 3) mod 2 = 0 then Bounce_Shake else -Bounce_Shake) / W.Links (I).Unit;
                     Age := Age + 1;
                  end if;
                  --  A hand on the table never quite stands still.
                  if W.Quirk.Creep and then Now_Touching then
                     declare
                        Draw : constant Real := Gaussian (W);
                     begin
                        Shake := Shake + Creep_Step * Draw / W.Links (I).Unit;
                     end;
                  end if;
                  declare
                     L : Link renames W.Links (I);
                  begin
                     L.Q := L.Q - L.Bounce + Shake;
                     L.Bounce := Shake;
                     L.Bounce_Age := Age;
                  end;
               end;
            end if;
         end loop;
      end;
      --  The motor turns within the slack before it takes the link along: the link stands where it stood until the
      --  motor meets the slack's other end.
      if W.Quirk.Play then
         for I in Old'Range loop
            declare
               L : Link renames W.Links (I);
            begin
               if L.Arm = A and then L.Kind = Revolute and then L.Play > 0.0 then
                  L.Play_Offset :=
                    Real'Max (-L.Play / 2.0, Real'Min (L.Play / 2.0, L.Play_Offset + (L.Q - Old (I))));
               end if;
            end;
         end loop;
         Forward (W);
      end if;
      for I in Old'Range loop
         if W.Links (I).Arm = A and then abs (W.Links (I).Q - Old (I)) > 1.0e-15 * Real'Max (1.0, abs Old (I)) then
            W.Links (I).Moved_Beat := W.Beat;
         end if;
      end loop;
   end Move_Arm;


   ---------------------------------------------------------------------------
   --  Holding

   --  A box grown by Margin on every side.
   function Grown (S : Solid; Margin : Real) return Solid is
     ((S with delta Half => [S.Half (1) + Margin, S.Half (2) + Margin, S.Half (3) + Margin]));

   --  Two of arm A's fingers touch the thing S from opposite sides: each within the grip margin of it, their middles
   --  on either side of its middle.
   function Pinched (W : World; A : Positive; S : Solid) return Boolean is
      package Offsets is new Ada.Containers.Vectors (Positive, Vec3, Driver.Numerics.Arrays."=");
      Touching : Offsets.Vector;
   begin
      for F of W.Solids loop
         if F.Arm = A and then F.Finger and then Overlap (Grown (F, Grip_Margin), S) then
            Touching.Append (Minus (F.Pose.Translation, S.Pose.Translation));
         end if;
      end loop;
      for I in Touching.First_Index .. Touching.Last_Index loop
         for J in I + 1 .. Touching.Last_Index loop
            if Dot (Touching (I), Touching (J)) < 0.0 then
               return True;
            end if;
         end loop;
      end loop;
      return False;
   end Pinched;

   --  A thing pinched by arm A's fingers while its closer is asked further shut than it stands goes with the hand,
   --  rigidly, from where it was when it was taken; once the fingers no longer pinch it (the closer opened), it is
   --  let go and comes down onto the table where it is, turned only about the upright as it was. Nothing slips:
   --  a pinch holds whatever it holds.
   procedure Grip (W : in out World; A : Positive) is
      C : constant Natural := W.Closers (A);
   begin
      if C = 0 then
         return;
      end if;
      declare
         Closer   : Link renames W.Links (C);
         Pressing : constant Boolean := Closer.Target < Closer.Q - Closer.Dead;
         Tool     : constant Rigid := W.Links (W.Tools (A)).Pose;
      begin
         for S of W.Solids loop
            if S.Link = 0 and then S.Arm = 0 and then S.Kind = Box then
               if S.Held_By = A then
                  if not Pinched (W, A, S) then
                     declare
                        R   : constant Mat3 := S.Pose.Rotation;
                        Yaw : constant Real := Arctan (R (2, 1), R (1, 1));
                     begin
                        S.Held_By := 0;
                        S.Pose := (Rotation => About ([0.0, 0.0, 1.0], Yaw),
                                   Translation => [S.Pose.Translation (1), S.Pose.Translation (2), W.Table_Z + S.Half (3)]);
                        S.Local := S.Pose;
                        Ada.Text_IO.Put_Line ("fake_robot: beat" & W.Beat'Image & ": arm" & A'Image & " let go of "
                                              & To_String (S.Name));
                     end;
                  end if;
               elsif S.Held_By = 0 and then Pressing and then Pinched (W, A, S) then
                  S.Held_By := A;
                  S.Held_Rel := Inverse (Tool) * S.Pose;
                  Ada.Text_IO.Put_Line ("fake_robot: beat" & W.Beat'Image & ": arm" & A'Image & " holds "
                                        & To_String (S.Name) & " at " & Img (S.Pose.Translation (3) - W.Table_Z - S.Half (3))
                                        & " m above its rest");
               end if;
            end if;
         end loop;
      end;
   end Grip;
   procedure Step (W : in out World) is
   begin
      W.Beat := W.Beat + 1;
      --  The delay lets through what is due.
      declare
         Kept : Due_Vectors.Vector;
      begin
         for D of W.Pending loop
            if D.Due <= W.Beat then
               W.Links (D.Joint).Target := D.Value;
               --  Where this move lands short of its target: drawn with the target, as a servo's tolerance has it.
               W.Links (D.Joint).Landing := Random (W, 0.0, W.Links (D.Joint).Dead);
            else
               Kept.Append (D);
            end if;
         end loop;
         W.Pending := Kept;
      end;
      for A in 1 .. W.Arms loop
         Move_Arm (W, A);
      end loop;
      Forward (W);
      for A in 1 .. W.Arms loop
         Grip (W, A);
      end loop;
      Forward (W);
      Report (W);
   end Step;

   ---------------------------------------------------------------------------
   --  What the robot reports

   --  Draws this beat's readings of every group, noise and all, once.
   procedure Report (W : in out World) is
   begin
      for G in W.Groups.First_Index .. W.Groups.Last_Index loop
         W.Groups (G).Reported.Clear;
         for V of W.Groups (G).Values loop
            W.Groups (G).Reported.Append (V);
         end loop;
         for I in 1 .. Natural (W.Groups (G).Joints.Length) loop
            declare
               J     : constant Positive := W.Groups (G).Joints (I);
               Unit  : constant Real := W.Links (J).Unit;
               Moved : constant Boolean := W.Beat - W.Links (J).Moved_Beat <= Jitter_Beats;
               Q     : Real := W.Links (J).Q;
            begin
               if W.Quirk.Zero_Noise and then Moved then
                  declare
                     Draw : constant Real := Random (W, -1.0, 1.0);
                  begin
                     Q := Q + Jitter_Share * Real'Max (1.0, abs Q) * Draw;
                  end;
               end if;
               if W.Quirk.Drift then
                  Q := Q + Drift_Rate * Real (W.Beat) / Unit;
               end if;
               if W.Quirk.Reading_Noise > 0.0 then
                  declare
                     Draw : constant Real := Gaussian (W);
                  begin
                     Q := Q + W.Quirk.Reading_Noise * Draw;
                  end;
               end if;
               --  A servo bus reads whole ticks: the jitter and the drift vanish under the rounding, and a resting
               --  reading repeats exactly.
               if W.Groups (G).Integers then
                  Q := Real'Rounding (Q);
               end if;
               W.Groups (G).Reported.Append (Q);
            end;
         end loop;
      end loop;
   end Report;

   function Readings (W : World; G : Positive) return Real_Array is
      R : Real_Array (1 .. Natural (W.Groups (G).Reported.Length));
   begin
      for I in R'Range loop
         R (I) := W.Groups (G).Reported (I);
      end loop;
      return R;
   end Readings;

   function Echo (W : World; G : Positive) return Real_Array is
      R : Real_Array (1 .. Natural (W.Groups (G).Echoed.Length));
   begin
      for I in R'Range loop
         R (I) := W.Groups (G).Echoed (I);
      end loop;
      return R;
   end Echo;

   ---------------------------------------------------------------------------

   function Describe (W : World) return String is
      S : Unbounded_String;
      procedure Put (Text : String);
      procedure Put (Text : String) is
      begin
         Append (S, Text & ASCII.LF);
      end Put;
      function V (X : Vec3) return String is ("(" & Img (X (1)) & ", " & Img (X (2)) & ", " & Img (X (3)) & ")");
   begin
      Put ("seed" & W.Seed'Image & ": table top at z = " & Img (W.Table_Z) & " m," & W.Arms'Image & " arms");
      for A in 1 .. W.Arms loop
         Put ("arm" & A'Image & " ends in " & Hand_Kind'Image (W.Kinds (A))
              & (if (for some C of W.Cameras => C.Arm = A) then ", with an eye of its own" else ", with no eye of its own"));
      end loop;
      for I in W.Links.First_Index .. W.Links.Last_Index loop
         declare
            L : Link renames W.Links (I);
         begin
            Put ("link " & To_String (L.Name) & ": " & L.Kind'Image & " on "
                 & (if L.Parent = 0 then "the world" else To_String (W.Links (L.Parent).Name))
                 & " at " & V (L.Origin.Translation) & " axis " & V (L.Axis)
                 & (if L.Kind = Fixed then ""
                    else ", " & Img (L.Unit) & " per reading unit, ends " & Img (L.Lo) & " .. " & Img (L.Hi)
                         & ", rest " & Img (L.Q) & ", tau " & Img (L.Tau) & " beats")
                 & (if L.Mirror > 0 then ", mirrors " & To_String (W.Links (L.Mirror).Name) else ""));
         end;
      end loop;
      for G of W.Groups loop
         Put ("group " & To_String (G.Name) & ":"
              & Natural'Image (Natural (G.Joints.Length) + Natural (G.Values.Length))
              & " channels" & (if G.Commandable then ", commandable" else ""));
      end loop;
      for C of W.Cameras loop
         Put ("camera " & To_String (C.Name) & " on "
              & (if C.Link = 0 then "the world" else To_String (W.Links (C.Link).Name)) & ": " & Trim (C.Width) & " x "
              & Trim (C.Height) & ", fx " & Img (C.Fx) & " fy " & Img (C.Fy) & " cx " & Img (C.Cx) & " cy " & Img (C.Cy)
              & ", mounted at " & V (C.Mount.Translation));
      end loop;
      for O of W.Solids loop
         if O.Link = 0 and then O.Kind = Box then
            Put ("thing " & To_String (O.Name) & ": half sizes " & V (O.Half) & " at " & V (O.Pose.Translation)
                 & ", " & O.Mat.Kind'Image);
         end if;
      end loop;
      return To_String (S);
   end Describe;

end Fake_World;
