with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Bytes;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Pixels;
with Driver.Tests;

package body Driver.Robot.Hand.Lobes.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Images;
   use Driver.Tests;

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
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   W : constant := 160;
   H : constant := 120;
   Noise_Px : constant Real := 0.2;   --  the matcher's own noise on still pixels
   Lobe_Floor : constant := 20 * 80 * 98 / 100;   --  a finger of 20 x 80 pixels, of which 2 % may be lost

   --  A finger is a band of columns from row Top down to the bottom border,
   --  moved sideways by Shift columns between the two views.
   type Finger is record
      Left, Right, Top : Natural;
      Shift            : Integer;
   end record;

   type Finger_Array is array (Positive range <>) of Finger;

   function Inside (F : Finger; C, R : Natural; Moved : Boolean) return Boolean is
     (R >= F.Top and then Integer (C) >= Integer (F.Left) + (if Moved then F.Shift else 0)
      and then Integer (C) <= Integer (F.Right) + (if Moved then F.Shift else 0));

   --  What the matcher reports for every pixel of one view against the other:
   --  a finger pixel lands shifted and comes back; a pixel a finger covers in
   --  the other view lands anywhere and does not come back; a still pixel
   --  lands on itself, all with the matcher's noise.
   function Matches (Fingers : Finger_Array; From_Moved : Boolean) return Correspondence_Array is
      Result : Correspondence_Array (1 .. W * H);
      K : Natural := 0;
   begin
      for R in 0 .. H - 1 loop
         for C in 0 .. W - 1 loop
            declare
               P : constant Pixel := (U => Real (C) + 0.5, V => Real (R) + 0.5);
               Shift : Integer := 0;
               On_Finger, Covered : Boolean := False;
            begin
               for F of Fingers loop
                  if Inside (F, C, R, From_Moved) then
                     On_Finger := True;
                     Shift := (if From_Moved then -F.Shift else F.Shift);
                  elsif Inside (F, C, R, not From_Moved) then
                     Covered := True;
                  end if;
               end loop;
               K := K + 1;
               if On_Finger then
                  Result (K) := (From => P, To => (U => P.U + Real (Shift) + Noise_Px * Gaussian,
                                                   V => P.V + Noise_Px * Gaussian),
                                 Back => (U => P.U + Noise_Px * Gaussian, V => P.V + Noise_Px * Gaussian),
                                 Matched => True);
               elsif Covered then
                  Result (K) := (From => P, To => (U => P.U + 7.0, V => P.V - 9.0),
                                 Back => (U => P.U + 13.0, V => P.V + 4.0), Matched => True);
               else
                  Result (K) := (From => P, To => (U => P.U + Noise_Px * Gaussian, V => P.V + Noise_Px * Gaussian),
                                 Back => (U => P.U + Noise_Px * Gaussian, V => P.V + Noise_Px * Gaussian),
                                 Matched => True);
               end if;
            end;
         end loop;
      end loop;
      return Result;
   end Matches;

   --  The lobes found between two views, the first view's correspondences
   --  drawn before the second's. They come from one generator, and the order
   --  in which a call's arguments are evaluated is the compiler's own: x86-64
   --  evaluates the last first, arm64 the first, so one seed gave the two
   --  targets different noise, and a count of pixels that stays within a
   --  finger's size by chance on one was one over on the other.
   function Found
     (Fingers    : Finger_Array;
      Here_Moved : Boolean;
      Noise      : Matcher_Noise;
      Still      : Mask) return Lobe_Vectors.Vector
   is
      Here  : constant Correspondence_Array := Matches (Fingers, Here_Moved);
      There : constant Correspondence_Array := Matches (Fingers, not Here_Moved);
   begin
      return Find (Here, There, Noise, Still, W, H);
   end Found;

   function Still_Sample (Fingers : Finger_Array) return Correspondence_Array is
      --  Pixels away from every finger, in both views.
      All_Matches : constant Correspondence_Array := Matches (Fingers, False);
      Result : Correspondence_Array (All_Matches'Range);
      K : Natural := 0;
   begin
      for M of All_Matches loop
         declare
            C : constant Natural := Natural (Real'Floor (M.From.U));
            R : constant Natural := Natural (Real'Floor (M.From.V));
            Near : Boolean := False;
         begin
            for F of Fingers loop
               Near := Near or else Inside (F, C, R, False) or else Inside (F, C, R, True);
            end loop;
            if not Near then
               K := K + 1;
               Result (K) := M;
            end if;
         end;
      end loop;
      return Result (1 .. K);
   end Still_Sample;

   procedure Two_Fingers_Close is
      Fingers : constant Finger_Array :=
        [(Left => 10, Right => 29, Top => 40, Shift => 45), (Left => 130, Right => 149, Top => 40, Shift => -45)];
      Noise : Matcher_Noise;
      Lobes : Lobe_Vectors.Vector;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 1);
      Noise := Noise_Of (Still_Sample (Fingers));
      Check_Close (Noise.Displacement.Sigma, Noise_Px, 0.05, "matcher noise from still pixels");
      Lobes := Found (Fingers, False, Noise, Create (W, H));
      Check (Natural (Lobes.Length) = 2, "two closing fingers gave" & Natural'Image (Natural (Lobes.Length)) & " lobes");
      if Natural (Lobes.Length) = 2 then
         for L of Lobes loop
            --  A lobe loses the few pixels whose round trip fails by chance
            --  (0.27 %) or whose match lands a pixel off at its edge.
            Check (L.Count_Here >= Lobe_Floor and then L.Count_Here <= 20 * 80
                   and then L.Count_There >= Lobe_Floor and then L.Count_There <= 20 * 80,
                   "a lobe's pixels are not its finger's:" & Natural'Image (L.Count_Here) & Natural'Image (L.Count_There));
            --  The tip is the row farthest from the bottom border it comes in from.
            Check (L.Tip_Known_Here and then abs (L.Tip_Here.V - 40.5) < 1.0e-9,
                   "the open tip is not the finger's top:" & Real'Image (L.Tip_Here.V));
            Check (L.Tip_Known_There and then abs (L.Tip_There.V - 40.5) < 1.0e-9,
                   "the closed tip is not the finger's top:" & Real'Image (L.Tip_There.V));
         end loop;
      end if;
      Check (Direction (Lobes, Create (W, H), Noise) = Towards_There,
             "closing fingers judged " & Closing'Image (Direction (Lobes, Create (W, H), Noise)));
      --  Seen from the closed end, the open end is where they part.
      Lobes := Found (Fingers, True, Noise, Create (W, H));
      Check (Direction (Lobes, Create (W, H), Noise) = Towards_Here,
             "opening fingers judged " & Closing'Image (Direction (Lobes, Create (W, H), Noise)));
   end Two_Fingers_Close;

   procedure Chance_Pixel_Is_No_Tip is
      --  A still pixel just above a finger's top passes the single test by
      --  chance (its displacement between the single and the family
      --  thresholds) and joins the finger's pixels; it must not be the tip.
      Fingers : constant Finger_Array :=
        [(Left => 10, Right => 29, Top => 40, Shift => 45), (Left => 130, Right => 149, Top => 40, Shift => -45)];
      Forward  : Correspondence_Array (1 .. W * H);
      Backward : Correspondence_Array (1 .. W * H);
      Noise    : Matcher_Noise;
      Lobes    : Lobe_Vectors.Vector;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 6);
      Noise := Noise_Of (Still_Sample (Fingers));
      Forward := Matches (Fingers, False);
      Backward := Matches (Fingers, True);
      declare
         Single : constant Real := Threshold (Noise.Displacement_Gate);
         Family : constant Real :=
           Threshold (Vector_Gate (2, Noise.Displacement.Degrees_Of_Freedom, Tests => 2 * W * H));
         Step   : constant Real := Noise.Displacement.Sigma * (Single + Family) / 2.0;
         --  Row 39, column 20: right above the first finger's top in the first view.
         K      : constant Positive := 39 * W + 20 + 1;
      begin
         Check (Family > Single, "the family threshold is not above the single one");
         Forward (K).To := (U => Forward (K).From.U + Step, V => Forward (K).From.V);
         Forward (K).Back := Forward (K).From;
      end;
      Lobes := Find (Forward, Backward, Noise, Create (W, H), W, H);
      Check (Natural (Lobes.Length) = 2, "two fingers gave" & Natural'Image (Natural (Lobes.Length)) & " lobes");
      for L of Lobes loop
         Check (L.Tip_Known_Here and then abs (L.Tip_Here.V - 40.5) < 1.0e-9,
                "a pixel that moved by chance became the tip:" & Real'Image (L.Tip_Here.V));
      end loop;
   end Chance_Pixel_Is_No_Tip;

   procedure Touching_At_The_Closed_End is
      --  Closed, the two fingers touch and form one patch; each of its
      --  pixels still belongs to the finger it came from.
      Fingers : constant Finger_Array :=
        [(Left => 10, Right => 29, Top => 40, Shift => 50), (Left => 120, Right => 139, Top => 40, Shift => -40)];
      Noise : Matcher_Noise;
      Lobes : Lobe_Vectors.Vector;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 2);
      Noise := Noise_Of (Still_Sample (Fingers));
      Lobes := Found (Fingers, False, Noise, Create (W, H));
      Check (Natural (Lobes.Length) = 2, "touching closed fingers gave" & Natural'Image (Natural (Lobes.Length)) & " lobes");
      for L of Lobes loop
         Check (L.Count_There >= Lobe_Floor and then L.Count_There <= 20 * 80,
                "a closed lobe took the other finger's pixels or lost its own:" & Natural'Image (L.Count_There));
         Check (abs (L.Centre_There.U - L.Centre_Here.U) > 39.0, "a closed lobe's centre did not move with its finger");
      end loop;
      --  Seen from the touching end the same two lobes come out.
      Lobes := Found (Fingers, True, Noise, Create (W, H));
      Check (Natural (Lobes.Length) = 2, "from the touching end:" & Natural'Image (Natural (Lobes.Length)) & " lobes");
   end Touching_At_The_Closed_End;

   procedure One_Finger_Against_A_Still_One is
      --  A gripper with one moving finger; the other is part of the hand's
      --  still pixels, which the moving finger closes towards.
      Fingers : constant Finger_Array := [1 => (Left => 20, Right => 39, Top => 50, Shift => 60)];
      Still_Finger : Mask := Create (W, H);
      Noise : Matcher_Noise;
      Lobes : Lobe_Vectors.Vector;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 3);
      for R in 50 .. H - 1 loop
         for C in 125 .. 144 loop
            Include (Still_Finger, C, R);
         end loop;
      end loop;
      Noise := Noise_Of (Still_Sample (Fingers));
      Lobes := Found (Fingers, False, Noise, Still_Finger);
      Check (Natural (Lobes.Length) = 1, "one moving finger gave" & Natural'Image (Natural (Lobes.Length)) & " lobes");
      Check (Direction (Lobes, Still_Finger, Noise) = Towards_There, "closing against the still finger not seen");
      Check (Direction (Lobes, Create (W, H), Noise) = Undecided, "one lobe judged closing with nothing to close on");
   end One_Finger_Against_A_Still_One;

   procedure Floating_Patch_Has_No_Tip is
      --  A moving patch attached to nothing (a shadow, a thing pushed along)
      --  is found, but has no tip.
      Fingers : constant Finger_Array := [1 => (Left => 20, Right => 39, Top => 30, Shift => 30)];
      Noise : Matcher_Noise;
      Lobes : Lobe_Vectors.Vector;
      Forward, Backward : Correspondence_Array (1 .. W * H);
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 4);
      Noise := Noise_Of (Still_Sample (Fingers));
      Forward := Matches (Fingers, False);
      Backward := Matches (Fingers, True);
      --  Cut the patch off from the bottom border: its lowest rows stay still.
      for K in Forward'Range loop
         if Forward (K).From.V > Real (H - 10) then
            Forward (K).To := Forward (K).From;
            Forward (K).Back := Forward (K).From;
            Backward (K).To := Backward (K).From;
            Backward (K).Back := Backward (K).From;
         end if;
      end loop;
      Lobes := Find (Forward, Backward, Noise, Create (W, H), W, H);
      Check (Natural (Lobes.Length) = 1, "a floating patch gave" & Natural'Image (Natural (Lobes.Length)) & " lobes");
      if Natural (Lobes.Length) = 1 then
         Check (not Lobes (1).Tip_Known_Here, "a patch attached to nothing was given a tip");
      end if;
   end Floating_Patch_Has_No_Tip;

   procedure Nothing_Moves is
      --  Two views of a still scene: the matcher's noise alone. Dozens of
      --  pixels pass the single test in each view, and some land on one
      --  another; none of that may make a lobe more often than one family
      --  test alarms.
      Trials  : constant := 100;
      Nominal : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
      None    : constant Finger_Array (1 .. 0) := [others => (0, 0, 0, 0)];
      With_Lobes, Candidates : Natural := 0;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 5);
      for T in 1 .. Trials loop
         declare
            Forward  : constant Correspondence_Array := Matches (None, False);
            Backward : constant Correspondence_Array := Matches (None, True);
            Noise    : constant Matcher_Noise := Noise_Of (Forward);   --  every pixel is still
            Found    : constant Lobe_Vectors.Vector := Find (Forward, Backward, Noise, Create (W, H), W, H);
         begin
            Candidates := Candidates + Count (Moving (Forward, Noise, W, H)) + Count (Moving (Backward, Noise, W, H));
            With_Lobes := With_Lobes + Boolean'Pos (not Found.Is_Empty);
         end;
      end loop;
      --  The single test still passes its share of still pixels ...
      Check (Real (Candidates) > Nominal * Real (Trials * 2 * W * H) / 2.0,
             "too few still pixels pass the single test:" & Natural'Image (Candidates));
      --  ... but a lobe appears at most as often as one test alarms.
      Check (Real (With_Lobes) <= Nominal * Real (Trials) + Driver.Conventions.Z * Sqrt (Nominal * Real (Trials)),
             "a still scene gave lobes in" & Natural'Image (With_Lobes) & " of" & Natural'Image (Trials) & " trials");
   end Nothing_Moves;

   --  Lobes from the change between two views and the poses of the arm
   --  (From_Change): the eye of an arm, fingers fixed in its picture that the
   --  closer moves sideways, and a textured world that slides across the
   --  picture as the arm moves. Every finger is smooth, which no matcher
   --  follows; the table under them is not.
   Finger_Level : constant := 20;

   function World (C, R : Integer) return Natural is (60 + (C * C * 7 + R * R * 13 + C * R * 5) mod 140);

   type Look is record
      Glossy   : Boolean := False;   --  the fingers reflect: their grey varies with the pose and the place
      Flat     : Boolean := False;   --  a patch of the world is one grey wherever the arm is
      Still    : Boolean := False;   --  a third finger the closer does not move
      Floating : Boolean := False;   --  the moving finger is cut off from the bottom border
   end record;

   function Seen
     (Fingers : Finger_Array; Moved : Boolean; Pose : Natural; Looks : Look) return Driver.Images.Image
   is
      use Driver.Bytes;
      use type Driver.Bytes.Offset;
      Data : Byte_Array (1 .. 3 * W * H);
   begin
      for Row in 0 .. H - 1 loop
         for Column in 0 .. W - 1 loop
            declare
               On_Finger : Boolean := False;
               Level     : Natural;
            begin
               for F of Fingers loop
                  On_Finger := On_Finger or else (Inside (F, Column, Row, Moved) and then not (Looks.Floating and then Row > H - 10));
               end loop;
               On_Finger := On_Finger or else (Looks.Still and then Column in 125 .. 144 and then Row >= 50);
               if On_Finger then
                  Level := Finger_Level + (if Looks.Glossy then (Column * 7 + Row * 3 + Pose * 11) mod 25 else 0);
               elsif Looks.Flat and then Column in 60 .. 74 and then Row >= 60 then
                  Level := 90;
               else
                  Level := World (Column + 5 * Pose, Row + 3 * Pose);
               end if;
               Data (Offset (3 * (Row * W + Column)) + 1) := Byte (Level);
               Data (Offset (3 * (Row * W + Column)) + 2) := Byte (Level);
               Data (Offset (3 * (Row * W + Column)) + 3) := Byte (Level);
            end;
         end loop;
      end loop;
      return Driver.Images.Create (W, H, Data);
   end Seen;

   function View_At (Fingers : Finger_Array; Moved : Boolean; Poses : Positive; Looks : Look; Fixed : Boolean := True)
     return Driver.Pixels.View
   is
      V : Driver.Pixels.View := Driver.Pixels.Empty (W, H);
   begin
      for P in 0 .. Poses - 1 loop
         Driver.Pixels.Add (V, Seen (Fingers, Moved, (if Fixed then 0 else P), Looks));
      end loop;
      return V;
   end View_At;

   --  The two ends seen from one pose (two frames each), the poses of the arm
   --  at the anchored end, and what From_Change makes of the change.
   function Place
     (Fingers          : Finger_Array;
      Anchored_At_Here : Boolean;
      Looks            : Look := (others => False);
      Poses            : Positive := 12;
      Attached         : Mask := Create (W, H);
      Moving_Poses     : Boolean := True) return Located
   is
      Here     : constant Driver.Pixels.View := View_At (Fingers, False, 2, Looks);
      There    : constant Driver.Pixels.View := View_At (Fingers, True, 2, Looks);
      Anchor   : constant Driver.Pixels.View :=
        View_At (Fingers, not Anchored_At_Here, (if Moving_Poses then Poses else 2), Looks, Fixed => not Moving_Poses);
      Compared : constant Driver.Pixels.Comparison := Driver.Pixels.Compare (Here, There);
   begin
      return From_Change (Compared.Changed, Here, There, Anchor, Anchored_At_Here, Compared.Spread, Attached);
   end Place;

   Two : constant Finger_Array :=
     [(Left => 10, Right => 29, Top => 40, Shift => 45), (Left => 130, Right => 149, Top => 40, Shift => -45)];

   procedure Check_Two (Found : Located; Looks : String) is
   begin
      Check (Found.How = Placed and then Natural (Found.Lobes.Length) = 2,
             Looks & ": two fingers gave" & Natural'Image (Natural (Found.Lobes.Length)) & " lobes, " & Placing'Image (Found.How));
      for L of Found.Lobes loop
         --  Each finger is 20 columns by 80 rows at each end, none of them lost.
         Check (L.Count_Here = 20 * 80 and then L.Count_There = 20 * 80,
                Looks & ": a lobe's pixels are not its finger's:" & Natural'Image (L.Count_Here)
                & Natural'Image (L.Count_There) & " (given" & Found.Here'Image & Found.There'Image & ", neither" & Found.Unassigned'Image
                & ", seeds" & Found.Seeds'Image & ", rounds" & Found.Rounds'Image & ")");
         Check (L.Tip_Known_Here and then abs (L.Tip_Here.V - 40.5) < 1.0e-9
                and then L.Tip_Known_There and then abs (L.Tip_There.V - 40.5) < 1.0e-9,
                Looks & ": a tip is not the finger's top:" & Real'Image (L.Tip_Here.V) & Real'Image (L.Tip_There.V));
         Check (L.Bordered_Here and then L.Bordered_There, Looks & ": a lobe is not seen coming in from the border");
      end loop;
      Check (Direction (Found.Lobes, Create (W, H)) = Towards_There,
             Looks & ": closing fingers judged " & Closing'Image (Direction (Found.Lobes, Create (W, H))));
   end Check_Two;

   procedure Change_Two_Fingers is
   begin
      for Anchored_Here in Boolean loop
         declare
            Found : constant Located := Place (Two, Anchored_Here);
         begin
            Check_Two (Found, (if Anchored_Here then "anchored at the first view" else "anchored at the other"));
            Check (Found.Unassigned = 0, "a pixel that changed was left to neither end:" & Natural'Image (Found.Unassigned));
         end;
      end loop;
   end Change_Two_Fingers;

   procedure Change_Glossy_Fingers is
      --  The fingers reflect: their grey varies with the pose and the place, by
      --  more than the table's pixels do in places.
      Found : constant Located := Place (Two, True, (Glossy => True, others => False));
   begin
      Check_Two (Found, "glossy");
   end Change_Glossy_Fingers;

   procedure Change_Over_Flat_World is
      --  Where the first finger arrives at the other end, the world is one grey
      --  whatever the pose: the poses call it the robot's, but the grey is not
      --  the fingers'.
      Found : constant Located := Place (Two, True, (Flat => True, others => False));
   begin
      Check_Two (Found, "flat world under a finger");
   end Change_Over_Flat_World;

   procedure Change_Touching is
      --  Closed, the two fingers touch and form one patch; each of its pixels
      --  still belongs to the finger it came from, as far as the nearer finger
      --  tells.
      Touching : constant Finger_Array :=
        [(Left => 10, Right => 29, Top => 40, Shift => 50), (Left => 120, Right => 139, Top => 40, Shift => -40)];
   begin
      for Anchored_Here in Boolean loop
         declare
            Found : constant Located := Place (Touching, Anchored_Here);
         begin
            Check (Found.How = Placed and then Natural (Found.Lobes.Length) = 2,
                   "touching fingers gave" & Natural'Image (Natural (Found.Lobes.Length)) & " lobes, "
                   & Placing'Image (Found.How));
            for L of Found.Lobes loop
               Check (L.Count_There >= 15 * 80 and then L.Count_There <= 25 * 80,
                      "a closed lobe took the other finger's pixels or lost its own:" & Natural'Image (L.Count_There));
            end loop;
         end;
      end loop;
   end Change_Touching;

   procedure Change_One_Against_Still is
      --  A gripper with one moving finger; the other is part of the hand's
      --  still pixels, which the moving finger closes towards.
      One   : constant Finger_Array := [1 => (Left => 20, Right => 39, Top => 50, Shift => 60)];
      Still_Finger : Mask := Create (W, H);
      Looks : constant Look := (Still => True, others => False);
   begin
      for R in 50 .. H - 1 loop
         for C in 125 .. 144 loop
            Include (Still_Finger, C, R);
         end loop;
      end loop;
      declare
         Found : constant Located := Place (One, True, Looks, Attached => Still_Finger);
      begin
         Check (Found.How = Placed and then Natural (Found.Lobes.Length) = 1,
                "one moving finger gave" & Natural'Image (Natural (Found.Lobes.Length)) & " lobes");
         Check (Direction (Found.Lobes, Still_Finger) = Towards_There, "closing against the still finger not seen");
         Check (Direction (Found.Lobes, Create (W, H)) = Undecided, "one lobe judged closing with nothing to close on");
      end;
   end Change_One_Against_Still;

   procedure Change_Floating is
      --  A moving patch attached to nothing (a shadow, a thing pushed along) is
      --  not a part of the robot: it is no lobe.
      One   : constant Finger_Array := [1 => (Left => 20, Right => 39, Top => 30, Shift => 30)];
      Found : constant Located := Place (One, True, (Floating => True, others => False));
   begin
      Check (Found.Lobes.Is_Empty and then Found.How = One_Sided,
             "a patch attached to nothing gave" & Natural'Image (Natural (Found.Lobes.Length)) & " lobes, "
             & Placing'Image (Found.How));
   end Change_Floating;

   procedure Change_Without_Poses is
      --  The arm did not move between the frames kept: what stays put in the
      --  picture is everything, and the poses tell nothing.
      Found : constant Located := Place (Two, True, Moving_Poses => False);
   begin
      Check (Found.How = Unseparated and then Found.Lobes.Is_Empty,
             "poses that did not move the eye placed lobes: " & Placing'Image (Found.How));
   end Change_Without_Poses;

   --  A block of columns C0 .. C1, rows R0 .. R1, in a mask.
   procedure Block (M : in out Mask; C0, C1, R0, R1 : Natural) is
   begin
      for R in R0 .. R1 loop
         for C in C0 .. C1 loop
            Include (M, C, R);
         end loop;
      end loop;
   end Block;

   procedure Fragments_Are_Not_Lobes is
      --  The pixels given to each end by the mixture: two fingers at each,
      --  and at each end a small block on the bottom border, which the
      --  mixture misplaced. A14's low end held parts of 29 219 and 19 664
      --  pixels (its two fingers) and of 400, 334, 149 and a few smaller (on
      --  the border, from keyboard keys), its high end parts of 19 689 and
      --  11 680 and of 832, 670, 300 ...; every part attached to the border
      --  was a lobe, the block of 400 pixels nearest to the open finger took
      --  its place, and the finger it belonged to was left without a partner.
      --  The parts' own sizes part the fingers from the fragments, whatever
      --  the mixture's doubt is (here none).
      Here_Set  : Mask := Create (W, H);
      There_Set : Mask := Create (W, H);
      Here_Parts, There_Parts : Natural;
   begin
      Block (Here_Set, 55, 74, 40, H - 1);     --  the first finger, closed
      Block (Here_Set, 85, 104, 40, H - 1);    --  the second
      Block (Here_Set, 150, 153, H - 6, H - 1);   --  24 pixels, near where the second finger opens
      Block (There_Set, 10, 29, 40, H - 1);    --  the first, open
      Block (There_Set, 130, 149, 40, H - 1);  --  the second
      Block (There_Set, 154, 158, H - 5, H - 1);  --  25 pixels, beside it
      declare
         Lobes : constant Lobe_Vectors.Vector :=
           Lobes_Of_Sets (Here_Set, There_Set, Create (W, H), 0.0, Here_Parts, There_Parts);
      begin
         Check (Natural (Lobes.Length) = 2 and then Here_Parts = 2 and then There_Parts = 2,
                "blocks a sixtieth of a finger's size were lobes or parts:" & Natural'Image (Natural (Lobes.Length))
                & " lobes, parts" & Natural'Image (Here_Parts) & Natural'Image (There_Parts));
         if Natural (Lobes.Length) = 2 then
            for L of Lobes loop
               Check (L.Count_Here = 20 * 80 and then L.Count_There = 20 * 80,
                      "a lobe holds pixels that are not its finger's:" & Natural'Image (L.Count_Here) & Natural'Image (L.Count_There));
            end loop;
            --  The first finger's two places are the first lobe's.
            Check (Lobes (1).Centre_Here.U < 80.0 and then Lobes (1).Centre_There.U < 40.0
                   and then Lobes (2).Centre_Here.U > 80.0 and then Lobes (2).Centre_There.U > 100.0,
                   "a finger's open place was given to the other finger's closed one");
         end if;
      end;
   end Fragments_Are_Not_Lobes;

   procedure Doubt_Is_Not_A_Lobe is
      --  Two small fingers at each end of 30 and 40 pixels, in a mixture whose
      --  doubt is 35 pixels: a part of no more than that could be made of
      --  nothing but the pixels it was wrong about. The same sets with no
      --  doubt have their two lobes.
      Here_Set  : Mask := Create (W, H);
      There_Set : Mask := Create (W, H);
      Here_Parts, There_Parts : Natural;
   begin
      Block (Here_Set, 55, 59, H - 6, H - 1);      --  25 pixels
      Block (Here_Set, 100, 105, H - 7, H - 1);    --  42 pixels
      Block (There_Set, 10, 14, H - 6, H - 1);
      Block (There_Set, 140, 145, H - 7, H - 1);
      declare
         With_None : constant Lobe_Vectors.Vector :=
           Lobes_Of_Sets (Here_Set, There_Set, Create (W, H), 0.0, Here_Parts, There_Parts);
      begin
         Check (Natural (With_None.Length) = 2, "two small fingers were not two lobes with no doubt:"
                & Natural'Image (Natural (With_None.Length)));
      end;
      declare
         Doubting : constant Lobe_Vectors.Vector :=
           Lobes_Of_Sets (Here_Set, There_Set, Create (W, H), 35.0, Here_Parts, There_Parts);
      begin
         Check (Here_Parts = 1 and then There_Parts = 1,
                "parts of 35 pixels or fewer counted, with a doubt of 35:" & Natural'Image (Here_Parts)
                & Natural'Image (There_Parts));
         Check (Natural (Doubting.Length) = 1, "a part no larger than the doubt was a lobe:"
                & Natural'Image (Natural (Doubting.Length)));
      end;
   end Doubt_Is_Not_A_Lobe;

   procedure Tip_Is_The_Reach_Not_The_Strip is
      --  A wedge from the left border rising to an apex 118 pixels in, and a
      --  strip of four pixels' height along the bottom of the picture that
      --  hangs on the wedge and runs 143 pixels from the border without
      --  touching the bottom border: the farthest pixel along the paths through
      --  the lobe is the strip's end (A14's open end: 160.8 against the
      --  apex's 127.9; here 143 against 128); the farthest along its reach is
      --  the apex.
      Here_Set  : Mask := Create (W, H);
      There_Set : Mask := Create (W, H);
      Parts_Here, Parts_There : Natural;
   begin
      --  The wedge: rows 70 .. 119 at the left border, rising to a point at
      --  column 118, row 20; its top edge climbs 50 rows and its bottom edge 99.
      for C in 0 .. 118 loop
         for R in 70 - 50 * C / 118 .. 119 - 99 * C / 118 loop
            Include (There_Set, C, R);
         end loop;
      end loop;
      Block (There_Set, 0, 143, H - 7, H - 4);   --  the strip, four rows, from the left border to column 143
      Block (Here_Set, 60, 80, 40, H - 1);   --  the finger closed, from the bottom border
      declare
         Lobes : constant Lobe_Vectors.Vector :=
           Lobes_Of_Sets (Here_Set, There_Set, Create (W, H), 0.0, Parts_Here, Parts_There);
      begin
         Check (Natural (Lobes.Length) = 1, "the wedge and its strip were not one lobe:" & Natural'Image (Natural (Lobes.Length)));
         if Natural (Lobes.Length) = 1 then
            Check (Lobes (1).Tip_Known_There and then Lobes (1).Tip_There.V < 50.0 and then Lobes (1).Tip_There.U > 100.0,
                   "the open tip is not the wedge's apex: (" & Real'Image (Lobes (1).Tip_There.U) & ","
                   & Real'Image (Lobes (1).Tip_There.V) & ")");
         end if;
      end;
   end Tip_Is_The_Reach_Not_The_Strip;


   procedure Parts_Are_Not_Turned_For_Ever is
      --  Sixteen separate parts of changed pixels, each with a brightness of
      --  its own at each end, and seeds right in most of their pixels. A part
      --  is turned when the rest explain it better the other way round, and
      --  turning one moves what the others are explained by, so a pass can
      --  turn parts that the next turns back. A pass that turns no fewer
      --  parts than the one before has not come closer, and the rounds must
      --  end there, not at the most a label can cross the picture in (A14's
      --  stage of 62 570 changed pixels turned four to nine parts at every
      --  pass for 1 120 rounds, fourteen seconds, and left 38 % of its pixels
      --  to neither; here eight of twelve scenes ran to that most).
      Grid  : constant := 4;
      Block : constant := 12;
      Pitch : constant := Block + 2;
      Side  : constant := Grid * Pitch;
      Cells : constant := Grid * Grid;
      Total : constant := Cells * Block * Block;
      Span  : constant Real := 40.0;
      Anchored_Low, Other_Low : array (1 .. Cells) of Real;
      Is_Robot : array (1 .. Cells) of Boolean;
      Where    : Places (1 .. Total);
      Seeds    : Flags (1 .. Total);
      Anchored : Driver.Real_Array (1 .. Total);
      Other    : Driver.Real_Array (1 .. Total);
      Given    : Kinds (1 .. Total);
      Rounds   : Natural;
      Doubt    : Real;
      Next     : Natural;

      function Uniform return Real is (Real (Ada.Numerics.Float_Random.Random (Gen)));
   begin
      for Seed in 2 .. 8 loop
         Ada.Numerics.Float_Random.Reset (Gen, Seed);
         for C in 1 .. Cells loop
            Anchored_Low (C) := Real'Rounding ((255.0 - Span) * Uniform);
            Other_Low (C) := Real'Rounding ((255.0 - Span) * Uniform);
            Is_Robot (C) := Uniform < 0.5;
         end loop;
         Next := 0;
         for Cell_Row in 0 .. Grid - 1 loop
            for Cell_Column in 0 .. Grid - 1 loop
               for R in 0 .. Block - 1 loop
                  for Column in 0 .. Block - 1 loop
                     Next := Next + 1;
                     Where (Next) := (Cell_Row * Pitch + R) * Side + Cell_Column * Pitch + Column;
                     Anchored (Next) := Real'Rounding (Anchored_Low (Cell_Row * Grid + Cell_Column + 1) + Span * Uniform);
                     Other (Next) := Real'Rounding (Other_Low (Cell_Row * Grid + Cell_Column + 1) + Span * Uniform);
                     Seeds (Next) := (if Uniform < 0.2 then not Is_Robot (Cell_Row * Grid + Cell_Column + 1)
                                      else Is_Robot (Cell_Row * Grid + Cell_Column + 1));
                  end loop;
               end loop;
            end loop;
         end loop;
         Tell_Ends (Side, Side, Where, Seeds, Anchored, Other, Given, Rounds, Doubt);
         Check (Rounds < 2 * Side,
                "scene" & Integer'Image (Seed) & ": the mixture ran" & Natural'Image (Rounds)
                & " rounds, to the most it is allowed");
         Check (Doubt >= 0.0 and then Doubt <= Real (Total) / 2.0,
                "scene" & Integer'Image (Seed) & ": the doubt is not a count of pixels:" & Real'Image (Doubt));
      end loop;
   end Parts_Are_Not_Turned_For_Ever;

   procedure Neighbours_Decide_What_Brightness_Cannot is
      --  Two regions, the robot at the anchored end on the left and at the
      --  other end on the right, in each of which some pixels are dark at
      --  both ends: their brightnesses tell nothing, and only their neighbours
      --  can, a pixel at a round from the clear ones around them. The seeds
      --  are right but for a few of the clear pixels, and a coin on the rest.
      --  The rounds must go on while the doubt, the expected error, still
      --  moves (A14's final ends: 12 962 of 89 294 pixels left to neither when
      --  the rounds ended with the labels, 3 840 when they end with the
      --  doubt), and must end when it stops moving or comes back to what it was
      --  two rounds before, not at the most a label can cross the picture in:
      --  two pixels whose neighbours each tell them to take the other's label
      --  swap it every round for ever (scene 2: 200 rounds, 21 with the rule).
      type Layout is record
         Seed  : Integer;
         Blank : Real;   --  the share of the pixels dark at both ends
         Wrong : Real;   --  the share of the clear pixels whose seed is wrong
      end record;
      Layouts : constant array (1 .. 3) of Layout := [(1, 0.25, 0.05), (2, 0.25, 0.05), (3, 0.40, 0.10)];
      Side     : constant := 100;
      Total    : constant := Side * Side;
      Where    : Places (1 .. Total);
      Seeds    : Flags (1 .. Total);
      Anchored : Driver.Real_Array (1 .. Total);
      Other    : Driver.Real_Array (1 .. Total);
      Given    : Kinds (1 .. Total);
      Blank    : Flags (1 .. Total);
      On_Left  : Flags (1 .. Total);
      Rounds   : Natural;
      Doubt    : Real;
      Right, Undecided : Natural;

      function Uniform return Real is (Real (Ada.Numerics.Float_Random.Random (Gen)));
      function Draw (Low, High : Real) return Real is (Real'Rounding (Low + (High - Low) * Uniform));
   begin
      for Which in Layouts'Range loop
         Ada.Numerics.Float_Random.Reset (Gen, Layouts (Which).Seed);
         for Row in 0 .. Side - 1 loop
            for Column in 0 .. Side - 1 loop
               declare
                  J     : constant Positive := Row * Side + Column + 1;
                  Empty : constant Boolean := Uniform < Layouts (Which).Blank;
               begin
                  Where (J) := J - 1;
                  Blank (J) := Empty;
                  On_Left (J) := Column < Side / 2;
                  if Empty then
                     Anchored (J) := Draw (20.0, 60.0);
                     Other (J) := Draw (20.0, 60.0);
                     Seeds (J) := Uniform < 0.5;
                  elsif On_Left (J) then
                     Anchored (J) := Draw (20.0, 45.0);
                     Other (J) := Draw (80.0, 250.0);
                     Seeds (J) := not (Uniform < Layouts (Which).Wrong);
                  else
                     Anchored (J) := Draw (80.0, 250.0);
                     Other (J) := Draw (20.0, 45.0);
                     Seeds (J) := Uniform < Layouts (Which).Wrong;
                  end if;
               end;
            end loop;
         end loop;
         Tell_Ends (Side, Side, Where, Seeds, Anchored, Other, Given, Rounds, Doubt);
         Right := 0;
         Undecided := 0;
         for J in 1 .. Total loop
            if Blank (J) then
               case Given (J) is
                  when Anchored_End =>
                     Right := Right + Boolean'Pos (On_Left (J));
                  when Other_End =>
                     Right := Right + Boolean'Pos (not On_Left (J));
                  when Neither =>
                     Undecided := Undecided + 1;
               end case;
            end if;
         end loop;
         Check (Rounds < 2 * Side,
                "scene" & Integer'Image (Which) & ": the mixture ran" & Natural'Image (Rounds)
                & " rounds, to the most it is allowed");
         Check (Undecided < Right,
                "scene" & Integer'Image (Which) & ": of the pixels dark at both ends" & Natural'Image (Right)
                & " went to the right end and" & Natural'Image (Undecided) & " to neither");
      end loop;
   end Neighbours_Decide_What_Brightness_Cannot;

   procedure Change_Nothing is
      Here  : constant Driver.Pixels.View := View_At (Two, False, 2, (others => False));
      Found : constant Located :=
        From_Change (Create (W, H), Here, Here, View_At (Two, False, 12, (others => False), Fixed => False), True, 0.3,
                     Create (W, H));
   begin
      Check (Found.How = Nothing_Changed and then Found.Lobes.Is_Empty, "no change placed lobes");
   end Change_Nothing;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.lobes.change", "two smooth closing fingers are not two lobes with their tips and "
                             & "direction, found from the change and the poses", Change_Two_Fingers'Access);
      Driver.Tests.Register ("hand.lobes.glossy", "fingers that reflect, and so vary with the arm's poses, are not "
                             & "found", Change_Glossy_Fingers'Access);
      Driver.Tests.Register ("hand.lobes.flat", "a flat world where a finger arrives is given to the robot's end",
                             Change_Over_Flat_World'Access);
      Driver.Tests.Register ("hand.lobes.closed", "fingers touching at the closed end are not two lobes from either "
                             & "end", Change_Touching'Access);
      Driver.Tests.Register ("hand.lobes.against", "a finger closing against a still one is not seen closing, from "
                             & "the change", Change_One_Against_Still'Access);
      Driver.Tests.Register ("hand.lobes.adrift", "a patch that changed and is attached to nothing is made a lobe",
                             Change_Floating'Access);
      Driver.Tests.Register ("hand.lobes.poses", "poses that did not move the eye place lobes",
                             Change_Without_Poses'Access);
      Driver.Tests.Register ("hand.lobes.nothing", "no change places lobes", Change_Nothing'Access);
      Driver.Tests.Register ("hand.lobes.cycle", "parts turned back and forth are not turned for ever",
                             Parts_Are_Not_Turned_For_Ever'Access);
      Driver.Tests.Register ("hand.lobes.blank", "pixels dark at both ends are told by their neighbours, and the rounds end",
                             Neighbours_Decide_What_Brightness_Cannot'Access);
      Driver.Tests.Register ("hand.lobes.fragments", "fragments of a sixtieth of a finger's size are lobes",
                             Fragments_Are_Not_Lobes'Access);
      Driver.Tests.Register ("hand.lobes.doubt", "parts no larger than the mixture's doubt are lobes",
                             Doubt_Is_Not_A_Lobe'Access);
      Driver.Tests.Register ("hand.lobes.reach", "a strip hanging on a wedge is its tip, not the wedge's apex",
                             Tip_Is_The_Reach_Not_The_Strip'Access);
      Driver.Tests.Register ("hand.lobes.still", "the matcher's noise on a still scene makes lobes",
                             Nothing_Moves'Access);
      Driver.Tests.Register ("hand.lobes.two", "two closing fingers are not two lobes with their tips and direction",
                             Two_Fingers_Close'Access);
      Driver.Tests.Register ("hand.lobes.tip_seed", "a still pixel that passed the single test by chance becomes a tip",
                             Chance_Pixel_Is_No_Tip'Access);
      Driver.Tests.Register ("hand.lobes.touching", "fingers touching at the closed end merge into one lobe",
                             Touching_At_The_Closed_End'Access);
      Driver.Tests.Register ("hand.lobes.one", "a finger closing against a still one is not seen closing",
                             One_Finger_Against_A_Still_One'Access);
      Driver.Tests.Register ("hand.lobes.floating", "a moving patch attached to nothing is given a tip",
                             Floating_Patch_Has_No_Tip'Access);
   end Register;

end Driver.Robot.Hand.Lobes.Tests;
