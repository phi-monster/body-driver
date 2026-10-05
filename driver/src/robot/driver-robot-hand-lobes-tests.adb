with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Distributions;
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

   procedure Register is
   begin
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
