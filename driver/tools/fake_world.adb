with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Driver.Json;

package body Fake_World is

   use Ada.Numerics.Long_Elementary_Functions;

   Pi : constant := Ada.Numerics.Pi;

   --  How far a hand may go into what it pressed, as a share of the push
   --  that was blocked (the lab's robot: a fraction of a millimetre).
   Give_Share : constant := 0.02;

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

   --  The jitter of a reading after a push, in the last bits, and the drift
   --  of every reading, radians a beat.
   Jitter_Share : constant := 1.0e-16;
   Drift_Rate   : constant := 1.2e-12;
   Jitter_Beats : constant := 5;

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
   procedure Build_Arm (W : in out World; A : Positive; Side : Real; Width, Height : Positive);
   procedure Add_Things (W : in out World);
   function Motion (L : Link; Q : Real) return Rigid;
   function Corners (S : Solid) return Corner_Array;
   function Inside (P : Vec3; S : Solid) return Boolean;
   function Overlap (A, B : Solid) return Boolean;
   function Penetrates (W : World; A : Positive) return Boolean;
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
                Mat => Mat, Link => Link_Index, Arm => Arm, Finger => Finger));
   end Add_Box;

   procedure Add_Plane (W : in out World; Name : String; Pose : Rigid; Mat : Material) is
   begin
      W.Solids.Append
        (Solid'(Name => To_Unbounded_String (Name), Kind => Plane, Half => [0.0, 0.0, 0.0], Local => Pose,
                Pose => Pose, Mat => Mat, Link => 0, Arm => 0, Finger => False));
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

   procedure Build_Arm (W : in out World; A : Positive; Side : Real; Width, Height : Positive) is
      Base_X : constant Real := Side * Random (W, 0.25, 0.35);
      Base_Y : constant Real := Random (W, -0.52, -0.42);
      L0 : constant Real := Random (W, 0.05, 0.08);
      L1 : constant Real := Random (W, 0.03, 0.06);
      L2 : constant Real := Random (W, 0.22, 0.30);
      L3 : constant Real := Random (W, 0.22, 0.30);
      L4 : constant Real := Random (W, 0.06, 0.10);
      L5 : constant Real := Random (W, 0.04, 0.07);
      L6 : constant Real := Random (W, 0.03, 0.06);
      Finger_Length : constant Real := Random (W, 0.05, 0.07);
      Open_Width    : constant Real := Random (W, 0.07, 0.10);
      Unit_Draw : constant Real := Random (W);
      Arm_Unit  : constant Real := (if Unit_Draw < 0.4 then 1.0 elsif Unit_Draw < 0.7 then Pi / 180.0 else 0.001);
      Tau       : constant Real := Random (W, 0.8, 1.5);
      Tilt      : constant Real := Random (W, 0.75, 0.95);
      Grey  : constant Material :=
        (Kind => Flat, Albedo => [0.26, 0.26, 0.29], Contrast => 0.0, Scale => 1.0, Gloss => 0.2);
      Black : constant Material :=
        (Kind => Flat, Albedo => [0.04, 0.04, 0.04], Contrast => 0.0, Scale => 1.0, Gloss => 0.6);
      Prefix : constant String := "arm" & Trim (A) & "/";
      Base, J1, J2, J3, J4, J5, J6, F1, F2 : Positive;
      Rest : constant array (1 .. 6) of Real := [0.0, -0.3, -0.3, -0.6, 0.0, 0.0];   --  radians
      Low  : constant array (1 .. 6) of Real := [-2.9, -1.6, -2.0, -1.8, -3.0, -1.8];
      High : constant array (1 .. 6) of Real := [2.9, 1.6, 1.5, 1.8, 3.0, 1.8];
      Sag  : array (1 .. 6) of Real;

      function Joint (Name : String; Parent : Positive; Origin : Rigid; Axis : Vec3; N : Positive) return Link is
        (Name => To_Unbounded_String (Prefix & Name), Parent => Parent, Arm => A, Kind => Revolute, Origin => Origin,
         Axis => Axis, Unit => Arm_Unit, Q => Rest (N) / Arm_Unit, Target => Rest (N) / Arm_Unit,
         Lo => Low (N) / Arm_Unit, Hi => High (N) / Arm_Unit, Tau => Tau, Droop_Gain => Sag (N), others => <>);
      G : Group;
   begin
      for N in Sag'Range loop
         Sag (N) := Random (W, 0.0005, 0.002);
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
      --  The fingers: two plates sliding apart along the tool's x, one joint
      --  read (the share of the way open, 1 open, 0 closed) and its mirror.
      Add_Link (W, (Name => To_Unbounded_String (Prefix & "finger1"), Parent => J6, Arm => A, Kind => Prismatic,
                    Origin => Moved_To (0.0, L6 + 0.024, 0.0), Axis => [1.0, 0.0, 0.0], Unit => Open_Width / 2.0,
                    Q => 1.0, Target => 1.0, Lo => 0.0, Hi => 1.0, Tau => Tau, others => <>), F1);
      Add_Box (W, Prefix & "finger1", [0.005, Finger_Length / 2.0, 0.01],
               Moved_To (0.005, Finger_Length / 2.0, 0.0), Black, F1, A, Finger => True);
      Add_Link (W, (Name => To_Unbounded_String (Prefix & "finger2"), Parent => J6, Arm => A, Kind => Prismatic,
                    Origin => Moved_To (0.0, L6 + 0.024, 0.0), Axis => [-1.0, 0.0, 0.0], Unit => Open_Width / 2.0,
                    Q => 1.0, Target => 1.0, Lo => 0.0, Hi => 1.0, Tau => Tau, Mirror => F1, others => <>), F2);
      Add_Box (W, Prefix & "finger2", [0.005, Finger_Length / 2.0, 0.01],
               Moved_To (-0.005, Finger_Length / 2.0, 0.0), Black, F2, A, Finger => True);
      W.Closers.Append (F1);
      --  The wrist eye: above the root of the fingers, looking steeply down
      --  along them, so that they stand in the lower part of its picture and
      --  the table beyond them in the upper part.
      Add_Camera (W, "eye_arm" & Trim (A), J6, A,
                  Look ([0.0, L6 + 0.03, 0.055], [0.0, Cos (Tilt), -Sin (Tilt)], [0.0, 0.0, 1.0]), Width, Height, 0.62);
      G.Name := To_Unbounded_String ("arm" & Trim (A) & "_joints");
      G.Commandable := True;
      declare
         Arm_Joints : constant array (1 .. 6) of Positive := [J1, J2, J3, J4, J5, J6];
      begin
         for J of Arm_Joints loop
            G.Joints.Append (J);
            G.Echoed.Append (W.Links (J).Target);
         end loop;
      end;
      W.Groups.Append (G);
      G := (Name => To_Unbounded_String ("arm" & Trim (A) & "_closer"), Commandable => True, others => <>);
      G.Joints.Append (F1);
      G.Echoed.Append (1.0);
      W.Groups.Append (G);
   end Build_Arm;

   --  Things on the table: a flat slab of fine stripes in front of the first
   --  arm (the keyboard the lab's table had), and a few boxes.
   procedure Add_Things (W : in out World) is
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
                     Pose => Pose, Mat => Mat, Link => 0, Arm => 0, Finger => False);
               begin
                  if not (for some O of W.Solids => O.Link > 0 and then Overlap (S, O)) then
                     W.Solids.Append (S);
                     Placed := True;
                  end if;
               end;
            end loop;
         end;
      end loop;
   end Add_Things;

   procedure Build (W : in out World; Seed : Natural; Width, Height : Positive) is
      Wood : constant Material :=
        (Kind => Grain, Albedo => [0.62, 0.50, 0.38], Contrast => 0.3, Scale => 60.0, Gloss => 0.05);
      Wall : constant Material :=
        (Kind => Noise, Albedo => [0.75, 0.74, 0.70], Contrast => 0.15, Scale => 3.0, Gloss => 0.0);
      Dark : constant Material :=
        (Kind => Flat, Albedo => [0.2, 0.2, 0.2], Contrast => 0.0, Scale => 1.0, Gloss => 0.0);
   begin
      Ada.Numerics.Float_Random.Reset (W.Rng, Integer (Seed));
      W.Seed := Seed;
      W.Table_Z := Random (W, 0.70, 0.84);
      Add_Plane (W, "table", Moved_To (0.0, 0.0, W.Table_Z), Wood);
      Add_Plane (W, "floor", Moved_To (0.0, 0.0, 0.0), Dark);
      Add_Plane (W, "wall", (Rotation => About ([1.0, 0.0, 0.0], Pi / 2.0), Translation => [0.0, 1.3, 0.8]), Wall);
      Add_Plane (W, "wall_left", (Rotation => About ([0.0, 1.0, 0.0], Pi / 2.0), Translation => [-1.6, 0.0, 0.8]),
                 Wall);
      Add_Plane (W, "wall_right", (Rotation => About ([0.0, 1.0, 0.0], -Pi / 2.0), Translation => [1.6, 0.0, 0.8]),
                 Wall);
      W.Arms := 2;
      Build_Arm (W, 1, -1.0, Width, Height);
      Build_Arm (W, 2, 1.0, Width, Height);
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
      Add_Things (W);
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
            Q      : constant Real := (if L.Mirror = 0 then L.Q else W.Links (L.Mirror).Q);
         begin
            L.Pose := Parent * L.Origin * Motion (L, Q);
         end;
      end loop;
      for S of W.Solids loop
         if S.Link > 0 then
            S.Pose := W.Links (S.Link).Pose * S.Local;
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

   --  Whether any of arm A's solids goes into the table or into a thing of the world.
   function Penetrates (W : World; A : Positive) return Boolean is
   begin
      for S of W.Solids loop
         if S.Arm = A then
            for P of Corners (S) loop
               if P (3) < W.Table_Z - 1.0e-7 then
                  return True;
               end if;
            end loop;
            for O of W.Solids loop
               if O.Arm = 0 and then O.Kind = Box and then Overlap (S, O) then
                  return True;
               end if;
            end loop;
         end if;
      end loop;
      return False;
   end Penetrates;

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
                     if O.Arm = 0 and then O.Kind = Box and then Overlap (S, O) then
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
      if not Grp.Commandable or else Values'Length /= Natural (Grp.Joints.Length) then
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
                  Q     : constant Real := From + (T - From) * Share;
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
         Place (Lo + (if W.Quirk.Give then Give_Share * (1.0 - Lo) else 0.0));
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
      for I in Old'Range loop
         if W.Links (I).Arm = A and then abs (W.Links (I).Q - Old (I)) > 1.0e-15 * Real'Max (1.0, abs Old (I)) then
            W.Links (I).Moved_Beat := W.Beat;
         end if;
      end loop;
   end Move_Arm;

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
