with Ada.Calendar;
with Ada.Containers.Vectors;
with Ada.Exceptions;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Driver.Images;
with Driver.Log;
with Driver.Tests;

package body Driver.Robot.Hand.Shape.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;
   use Driver.Tests;

   package Lobes renames Driver.Robot.Hand.Lobes;
   subtype Pixel is Driver.Images.Pixel;

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

   Z            : constant Real := Threshold (Scalar_Gate);
   Matcher_Px   : constant Real := 0.2;      --  the matcher's own noise, pixels
   Press_Sigma  : constant Real := 2.0e-4;   --  a pressed tip's, metres along each axis

   --  The rig: an eye riding on the arm above and between the lobes, looking
   --  forward and down along them as a wrist camera does, every lobe, the
   --  palm and any still jaw a box, and a table far below. The tool frame
   --  has x along the lobes, y to the left and z up.

   type Box is record
      Centre : Vec3 := Zero3;
      Axes   : Mat3 := Identity3;   --  its own axes as columns, tool frame
      Half   : Vec3 := Zero3;       --  half its size along each
   end record;

   type Motion is record
      Turn  : Mat3 := Identity3;
      Pivot : Vec3 := Zero3;
      Shift : Vec3 := Zero3;
   end record;
   --  Open to closed: X' = Turn (X - Pivot) + Pivot + Shift.

   function Apply (M : Motion; X : Vec3) return Vec3 is (M.Turn * (X - M.Pivot) + M.Pivot + M.Shift);
   function Undo (M : Motion; X : Vec3) return Vec3 is (Transpose (M.Turn) * (X - M.Pivot - M.Shift) + M.Pivot);

   function Moved (B : Box; M : Motion) return Box is
     ((Centre => Apply (M, B.Centre), Axes => M.Turn * B.Axes, Half => B.Half));

   type Box_Array is array (Positive range <>) of Box;
   type Motion_Array is array (Positive range <>) of Motion;

   Max_Lobes : constant := 2;

   type Rig is record
      Width, Height : Positive := 320;
      Focal         : Real := 250.0;
      Eye           : Vec3 := [0.0, 0.0, 0.04];
      Camera        : Mat3 := Identity3;   --  image right, image down, forward, as columns
      Lobe_Count    : Positive := 2;
      Open_Lobes    : Box_Array (1 .. Max_Lobes);
      Motions       : Motion_Array (1 .. Max_Lobes);
      Still_Count   : Natural := 1;
      Still         : Box_Array (1 .. 2);
      Table_Z       : Real := -0.08;
      Bend_Lobe     : Natural := 0;        --  a lobe whose part beyond Bend_From does not move with the rest
      Bend_From     : Real := 0.0;         --  along x
      Bend_By       : Real := 0.0;         --  pixels, in the closed view, down the image
   end record;

   function Looking (Forward : Vec3) return Mat3 is
      Z_C : constant Vec3 := Unit (Forward);
      X_C : constant Vec3 := Unit (Cross (Z_C, [0.0, 0.0, 1.0]));
      Y_C : constant Vec3 := Cross (Z_C, X_C);
   begin
      return [[X_C (1), Y_C (1), Z_C (1)], [X_C (2), Y_C (2), Z_C (2)], [X_C (3), Y_C (3), Z_C (3)]];
   end Looking;
   --  Image right is forward crossed with up, image down completes the frame.

   function Hit_Box (B : Box; From, Dir : Vec3; T : out Real) return Boolean is
      O     : constant Vec3 := Transpose (B.Axes) * (From - B.Centre);
      D     : constant Vec3 := Transpose (B.Axes) * Dir;
      Enter : Real := Real'First;
      Leave : Real := Real'Last;
   begin
      T := Real'Last;
      for K in 1 .. 3 loop
         if D (K) = 0.0 then
            if abs O (K) > B.Half (K) then
               return False;
            end if;
         else
            declare
               A : constant Real := (-B.Half (K) - O (K)) / D (K);
               C : constant Real := (B.Half (K) - O (K)) / D (K);
            begin
               Enter := Real'Max (Enter, Real'Min (A, C));
               Leave := Real'Min (Leave, Real'Max (A, C));
            end;
         end if;
      end loop;
      if Enter > Leave or else Enter <= 0.0 then
         return False;
      end if;
      T := Enter;
      return True;
   end Hit_Box;

   type Hit_Kind is (Nothing, Lobe_Hit, Still_Hit, Table_Hit);

   type Hit is record
      Kind     : Hit_Kind := Nothing;
      Lobe     : Natural := 0;
      Distance : Real := Real'Last;
      Point    : Vec3 := Zero3;
   end record;

   function Ray_Of (R : Rig; Px : Pixel) return Vec3 is
     (Unit (R.Camera * [(Px.U - Real (R.Width) / 2.0) / R.Focal, (Px.V - Real (R.Height) / 2.0) / R.Focal, 1.0]));

   function Cast (R : Rig; Dir : Vec3; Closed : Boolean) return Hit is
      Best : Hit;
      T    : Real;
   begin
      for L in 1 .. R.Lobe_Count loop
         if Hit_Box ((if Closed then Moved (R.Open_Lobes (L), R.Motions (L)) else R.Open_Lobes (L)), R.Eye, Dir, T)
           and then T < Best.Distance
         then
            Best := (Lobe_Hit, L, T, R.Eye + T * Dir);
         end if;
      end loop;
      for S in 1 .. R.Still_Count loop
         if Hit_Box (R.Still (S), R.Eye, Dir, T) and then T < Best.Distance then
            Best := (Still_Hit, 0, T, R.Eye + T * Dir);
         end if;
      end loop;
      if Dir (3) < 0.0 then
         T := (R.Table_Z - R.Eye (3)) / Dir (3);
         if T < Best.Distance then
            Best := (Table_Hit, 0, T, R.Eye + T * Dir);
         end if;
      end if;
      return Best;
   end Cast;

   function Project (R : Rig; X : Vec3; Px : out Pixel) return Boolean is
      C : constant Vec3 := Transpose (R.Camera) * (X - R.Eye);
   begin
      Px := (U => 0.0, V => 0.0);
      if C (3) <= 0.0 then
         return False;
      end if;
      Px := (U => Real (R.Width) / 2.0 + R.Focal * C (1) / C (3), V => Real (R.Height) / 2.0 + R.Focal * C (2) / C (3));
      return Px.U >= 0.0 and then Px.U < Real (R.Width) and then Px.V >= 0.0 and then Px.V < Real (R.Height);
   end Project;

   --  Where a point of a lobe is at the other end, the bent part off by its bend.
   function Elsewhere (R : Rig; L : Positive; X : Vec3; To_Closed : Boolean) return Vec3 is
     (if To_Closed then Apply (R.Motions (L), X) else Undo (R.Motions (L), X));

   type Correspondences_Access is access Lobes.Correspondence_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Lobes.Correspondence_Array, Correspondences_Access);

   --  The matcher's answers for every pixel of one view against the other: a
   --  lobe's point lands where the lobe's motion takes it and comes back,
   --  when it is seen there; a point the other view does not show lands
   --  anywhere and does not come back; a still point lands on itself; all
   --  with the matcher's noise.
   function Answers (R : Rig; From_Closed : Boolean) return Correspondences_Access is
      Result : constant Correspondences_Access := new Lobes.Correspondence_Array (1 .. R.Width * R.Height);
      K      : Natural := 0;
   begin
      for Row in 0 .. R.Height - 1 loop
         for Column in 0 .. R.Width - 1 loop
            declare
               P       : constant Pixel := (U => Real (Column) + 0.5, V => Real (Row) + 0.5);
               Here    : constant Hit := Cast (R, Ray_Of (R, P), From_Closed);
               Lost    : constant Lobes.Correspondence :=
                 (From => P, To => (U => P.U + 7.0, V => P.V - 9.0), Back => (U => P.U + 13.0, V => P.V + 4.0),
                  Matched => True);
               Q       : Pixel;
               Seen_As : Hit;
            begin
               K := K + 1;
               if Here.Kind = Lobe_Hit then
                  declare
                     There      : constant Vec3 := Elsewhere (R, Here.Lobe, Here.Point, To_Closed => not From_Closed);
                     Open_Point : constant Vec3 := (if From_Closed then There else Here.Point);
                  begin
                     if Project (R, There, Q) then
                        Seen_As := Cast (R, Ray_Of (R, Q), not From_Closed);
                     end if;
                     if Project (R, There, Q) and then Seen_As.Kind = Lobe_Hit and then Seen_As.Lobe = Here.Lobe
                       and then abs (Seen_As.Point - There) <= 1.0e-9
                     then
                        if R.Bend_Lobe = Here.Lobe and then Open_Point (1) > R.Bend_From then
                           Q.V := Q.V + (if From_Closed then -R.Bend_By else R.Bend_By);
                        end if;
                        Result (K) := (From    => P,
                                       To      => (U => Q.U + Matcher_Px * Gaussian, V => Q.V + Matcher_Px * Gaussian),
                                       Back    => (U => P.U + Matcher_Px * Gaussian, V => P.V + Matcher_Px * Gaussian),
                                       Matched => True);
                     else
                        Result (K) := Lost;
                     end if;
                  end;
               elsif Cast (R, Ray_Of (R, P), not From_Closed).Kind = Lobe_Hit then
                  Result (K) := Lost;
               else
                  Result (K) := (From    => P,
                                 To      => (U => P.U + Matcher_Px * Gaussian, V => P.V + Matcher_Px * Gaussian),
                                 Back    => (U => P.U + Matcher_Px * Gaussian, V => P.V + Matcher_Px * Gaussian),
                                 Matched => True);
               end if;
            end;
         end loop;
      end loop;
      return Result;
   end Answers;

   package Vec_Vectors is new Ada.Containers.Vectors (Positive, Vec3);

   --  What the eye sees of a lobe at the open end, sampled four times
   --  finer than its pixels across and down: the surface the measurement
   --  is of, to within a quarter of a pixel.
   function Seen_Surface (R : Rig; L : Positive) return Vec_Vectors.Vector is
      Fine   : constant := 4;
      Result : Vec_Vectors.Vector;
   begin
      for Row in 0 .. Fine * R.Height - 1 loop
         for Column in 0 .. Fine * R.Width - 1 loop
            declare
               H : constant Hit :=
                 Cast (R, Ray_Of (R, (U => (Real (Column) + 0.5) / Real (Fine), V => (Real (Row) + 0.5) / Real (Fine))),
                       Closed => False);
            begin
               if H.Kind = Lobe_Hit and then H.Lobe = L then
                  Result.Append (H.Point);
               end if;
            end;
         end loop;
      end loop;
      return Result;
   end Seen_Surface;

   --  The extent of points along a direction.
   procedure Span (Points : Vec_Vectors.Vector; D : Vec3; Low, High : out Real) is
   begin
      Low := Real'Last;
      High := Real'First;
      for X of Points loop
         Low := Real'Min (Low, D * X);
         High := Real'Max (High, D * X);
      end loop;
   end Span;

   type Tip_Pair is array (Opening) of Vec3;
   type Tip_Pairs is array (1 .. Max_Lobes) of Tip_Pair;
   type Shape_Pairs is array (1 .. Max_Lobes) of Lobe_Shape;
   type Mask_Pairs is array (1 .. Max_Lobes) of Driver.Images.Mask;

   type Outcome is record
      Found  : Natural := 0;          --  lobes found, matched to the rig's
      Shapes : Shape_Pairs;
      Tips   : Tip_Pairs;             --  where the tips truly are
      Masks  : Mask_Pairs;            --  each lobe's pixels at the open end
      Size   : Hand_Size (Max_Lobes);
      Problem : Unbounded_String;
   end record;

   type Tip_Change is access procedure (L : Positive; Tips : in out Tip_Array);

   --  The hand's chain on the rig: the matcher's answers, the lobes from them,
   --  each lobe's shape from its moves through the eye's lines of sight, the
   --  tips the presses would give with their noise, and the sizes.
   function Run (R : Rig; Seed : Integer; Change : Tip_Change := null) return Outcome is
      Result   : Outcome;
      Forward  : Correspondences_Access;
      Backward : Correspondences_Access;
      Still    : Correspondences_Access;
      Count    : Natural := 0;
      Attached : Driver.Images.Mask := Driver.Images.Create (R.Width, R.Height);
      function Line (Px : Pixel) return Vec3 is (Ray_Of (R, Px));
   begin
      Ada.Numerics.Float_Random.Reset (Gen, Seed);
      Forward := Answers (R, From_Closed => False);
      Backward := Answers (R, From_Closed => True);
      --  The still pixels: in both views neither a lobe nor covered by one.
      --  What a lobe is attached to is the robot's own pixels that did not
      --  change, as Hand.Collect has them (the eye's self mask less the
      --  pixels that changed between the ends): the still parts, and the
      --  part of a lobe by its hinge that moves by less than a pixel.
      Still := new Lobes.Correspondence_Array (1 .. 2 * R.Width * R.Height);
      for K in Forward'Range loop
         declare
            P      : constant Pixel := Forward (K).From;
            Dir    : constant Vec3 := Ray_Of (R, P);
            Open   : constant Hit := Cast (R, Dir, Closed => False);
            Closed : constant Hit := Cast (R, Dir, Closed => True);
            Q      : Pixel;
         begin
            if Open.Kind /= Lobe_Hit and then Closed.Kind /= Lobe_Hit then
               Still (Count + 1) := Forward (K);
               Still (Count + 2) := Backward (K);
               Count := Count + 2;
            end if;
            if (Open.Kind = Still_Hit and then Closed.Kind = Still_Hit)
              or else (Open.Kind = Lobe_Hit and then Closed.Kind = Lobe_Hit and then Open.Lobe = Closed.Lobe
                       and then Project (R, Apply (R.Motions (Open.Lobe), Open.Point), Q)
                       and then abs (Q.U - P.U) < 1.0 and then abs (Q.V - P.V) < 1.0)
            then
               Driver.Images.Include (Attached, Natural (Real'Floor (P.U)), Natural (Real'Floor (P.V)));
            end if;
         end;
      end loop;
      declare
         Noise : constant Lobes.Matcher_Noise := Lobes.Noise_Of (Still (1 .. Count));
         Found : constant Lobes.Lobe_Vectors.Vector :=
           Lobes.Find (Forward.all, Backward.all, Noise, Attached, R.Width, R.Height);
         Fits  : Lobe_Shape_Array (1 .. R.Lobe_Count);
         Tips  : Tip_Array (1 .. R.Lobe_Count, Opening);
         Have  : array (1 .. R.Lobe_Count) of Boolean := [others => False];
      begin
         Free (Forward);
         Free (Backward);
         Free (Still);
         for F of Found loop
            declare
               At_Open : constant Hit := Cast (R, Ray_Of (R, F.Tip_Here), Closed => False);
               At_Shut : constant Hit := Cast (R, Ray_Of (R, F.Tip_There), Closed => True);
            begin
               if F.Tip_Known_Here and then F.Tip_Known_There and then At_Open.Kind = Lobe_Hit
                 and then At_Shut.Kind = Lobe_Hit and then At_Open.Lobe = At_Shut.Lobe
                 and then not Have (At_Open.Lobe) and then F.Tip_Move_Here > 0 and then F.Tip_Move_There > 0
               then
                  declare
                     L     : constant Positive := At_Open.Lobe;
                     Moves : Lobes.Move_Array renames F.Moves_Here.Constant_Reference.Element.all;
                     Seen  : constant Sighting_Array :=
                       From_Moves (Moves, F.Moves_There.Constant_Reference.Element (F.Tip_Move_There), Line'Access,
                                   Noise.Displacement.Sigma);
                  begin
                     Have (L) := True;
                     Result.Found := Result.Found + 1;
                     Result.Masks (L) := F.Here;
                     Fits (L) := Fit (R.Eye, Seen, F.Tip_Move_Here, Seen'Last, F.Bordered_Here);
                     Result.Shapes (L) := Fits (L);
                     Result.Tips (L) := [Open => At_Open.Point, Closed_Empty => At_Shut.Point];
                     for O in Opening loop
                        Tips (L, O) := (Mean       => Result.Tips (L) (O)
                                                      + Press_Sigma * Vec3'[Gaussian, Gaussian, Gaussian],
                                        Covariance => (Press_Sigma ** 2) * Identity3);
                     end loop;
                     if Change /= null then
                        Change (L, Tips);
                     end if;
                  end;
               end if;
            end;
         end loop;
         if Result.Found /= R.Lobe_Count then
            Result.Problem := To_Unbounded_String
              (Natural'Image (Natural (Found.Length)) & " lobes found," & Result.Found'Image & " of them the rig's");
            return Result;
         end if;
         declare
            Sized : constant Hand_Size := Measure (Fits, Tips);
         begin
            Result.Size.Sizes (1 .. R.Lobe_Count) := Sized.Sizes;
            Result.Size.Depth := Sized.Depth;
            Result.Size.Axis := Sized.Axis;
            Result.Size.Why := Sized.Why;
         end;
      end;
      return Result;
   end Run;

   function Image (E : Estimate) return String is
     (Driver.Log.Image (E.Value, 5) & " +- " & Driver.Log.Image (E.Sigma, 5));

   procedure Check_Within (Name : String; Measured : Estimate; Truth : Real; Or_Unmeasured : Boolean := False) is
   begin
      Check (Or_Unmeasured or else Known (Measured), Name & " is not measured");
      if Known (Measured) then
         Check (abs (Measured.Value - Truth) <= Z * Measured.Sigma,
                Name & " " & Image (Measured) & " is off the truth " & Driver.Log.Image (Truth, 5) & " by "
                & Driver.Log.Image (abs (Measured.Value - Truth) / Measured.Sigma, 2) & " sigma");
         --  An honest sigma is no use when it is as large as what it measures.
         Check (Measured.Sigma < abs Truth / 10.0,
                Name & " " & Image (Measured) & " says too little of a size of " & Driver.Log.Image (Truth, 5));
      end if;
   end Check_Within;

   --  Every size against the same definitions on the rig's exact surface and
   --  motion, worked out here another way: a lobe's closing direction is
   --  the way its open tip's point of it moves as it closes; its width the
   --  least extent of what the eye sees of it across that direction, found
   --  by turning a line through every direction of the plane across it, a
   --  tenth of a degree apart and then finer about the least; the axis
   --  along each lobe, across both, toward its tip, averaged over the
   --  lobes; and the depth from the tips' middle along it to the root
   --  nearest them.
   procedure Check_Against_Truth (Name : String; R : Rig; O : Outcome; Or_Unmeasured : Boolean := False) is
      Points  : array (1 .. R.Lobe_Count) of Vec_Vectors.Vector;
      Closing : array (1 .. R.Lobe_Count) of Vec3;
      Along   : array (1 .. R.Lobe_Count) of Vec3;
      Widths  : array (1 .. R.Lobe_Count) of Real;
      Sum     : Vec3 := Zero3;

      procedure Least_Across (L : Positive) is
         F      : constant Vec3 := Closing (L);
         E1     : constant Vec3 := Unit (Cross (F, (if abs F (3) < 0.9 then [0.0, 0.0, 1.0] else [1.0, 0.0, 0.0])));
         E2     : constant Vec3 := Cross (F, E1);
         Best   : Real := Real'Last;
         Best_A : Real := 0.0;
         procedure Try (A : Real) is
            D         : constant Vec3 := Cos (A) * E1 + Sin (A) * E2;
            Low, High : Real;
         begin
            Span (Points (L), D, Low, High);
            if High - Low < Best then
               Best := High - Low;
               Best_A := A;
            end if;
         end Try;
         Coarse : constant := 1800;   --  a tenth of a degree over half a turn
      begin
         for K in 0 .. Coarse - 1 loop
            Try (Ada.Numerics.Pi * Real (K) / Real (Coarse));
         end loop;
         declare
            Around : constant Real := Best_A;
            Pitch  : constant Real := Ada.Numerics.Pi / Real (Coarse);
         begin
            for K in -100 .. 100 loop
               Try (Around + Pitch * Real (K) / 100.0);
            end loop;
         end;
         Widths (L) := Best;
         declare
            Across : constant Vec3 := Cos (Best_A) * E1 + Sin (Best_A) * E2;
            Middle : Vec3 := Zero3;
         begin
            for X of Points (L) loop
               Middle := Middle + X;
            end loop;
            Middle := (1.0 / Real (Points (L).Length)) * Middle;
            Along (L) := Cross (F, Across);
            if Real'(Along (L) * (O.Tips (L) (Open) - Middle)) < 0.0 then
               Along (L) := -Along (L);
            end if;
         end;
      end Least_Across;
   begin
      for L in 1 .. R.Lobe_Count loop
         Points (L) := Seen_Surface (R, L);
         Closing (L) := Unit (Apply (R.Motions (L), O.Tips (L) (Open)) - O.Tips (L) (Open));
         Least_Across (L);
         Sum := Sum + Along (L);
      end loop;
      declare
         Axis    : constant Vec3 := Unit (Sum);
         Tip_Sum : Real := 0.0;
         Deepest : Real := Real'First;
      begin
         for L in 1 .. R.Lobe_Count loop
            declare
               F         : constant Vec3 := Closing (L);
               Low, High : Real;
            begin
               Check_Within (Name & ": lobe" & L'Image & " width", O.Size.Sizes (L).Width, Widths (L), Or_Unmeasured);
               Span (Points (L), F, Low, High);
               Check_Within (Name & ": lobe" & L'Image & " thickness", O.Size.Sizes (L).Thickness, High - Low,
                             Or_Unmeasured);
               Check ((Or_Unmeasured and then not Known (O.Size.Sizes (L).Face))
                      or else (Known (O.Size.Sizes (L).Face)
                               and then abs (O.Size.Sizes (L).Face.Value - (High - F * O.Tips (L) (Open)))
                                        <= Z * O.Size.Sizes (L).Face.Sigma),
                      Name & ": lobe" & L'Image & " face " & Image (O.Size.Sizes (L).Face) & " is not "
                      & Driver.Log.Image (High - F * O.Tips (L) (Open), 5));
               Span (Points (L), Axis, Low, High);
               Deepest := Real'Max (Deepest, Low);
               Tip_Sum := Tip_Sum + Axis * O.Tips (L) (Open);
            end;
         end loop;
         Check_Within (Name & ": depth", O.Size.Depth, Tip_Sum / Real (R.Lobe_Count) - Deepest, Or_Unmeasured);
         Check ((Or_Unmeasured and then O.Size.Axis.Sigma = Real'Last)
                or else (O.Size.Axis.Sigma < Real'Last
                         and then Arcsin (Real'Min (1.0, abs Cross (O.Size.Axis.Unit_Vector, Axis)))
                                  <= Z * O.Size.Axis.Sigma),
                Name & ": the axis is off by" & Real'Image (abs Cross (O.Size.Axis.Unit_Vector, Axis))
                & " radians, its sigma" & Real'Image (O.Size.Axis.Sigma));
         --  What is not measured says why.
         Check (Known (O.Size.Depth) or else Length (O.Size.Why) > 0, Name & ": an unmeasured depth does not say why");
      end;
   end Check_Against_Truth;

   --  Two parallel lobes, closing straight toward each other by 25 mm
   --  each: 20 mm wide, 10 mm thick, 60 mm out of a palm the eye sees.
   function Parallel return Rig is
      R : Rig;
   begin
      R.Camera := Looking ([1.0, 0.0, -0.6]);
      R.Open_Lobes := [(Centre => [0.08, 0.035, 0.0], Axes => Identity3, Half => [0.03, 0.005, 0.01]),
                       (Centre => [0.08, -0.035, 0.0], Axes => Identity3, Half => [0.03, 0.005, 0.01])];
      R.Motions := [(Shift => [0.0, -0.025, 0.0], others => <>), (Shift => [0.0, 0.025, 0.0], others => <>)];
      R.Still (1) := (Centre => [0.025, 0.0, -0.0075], Axes => Identity3, Half => [0.025, 0.05, 0.0075]);
      return R;
   end Parallel;

   procedure Parallel_Lobes is
      R : constant Rig := Parallel;
      O : constant Outcome := Run (R, Seed => 7);
   begin
      Check (O.Found = 2, "the parallel lobes were not found: " & To_String (O.Problem));
      if O.Found = 2 then
         for L in 1 .. 2 loop
            Check (Fitted (O.Shapes (L)), "lobe" & L'Image & " not fitted: " & Why (O.Shapes (L)));
            Check (not Fitted (O.Shapes (L)) or else Angle (O.Shapes (L).Rotation) <= 0.01,
                   "a lobe that only shifts was fitted with a turn of"
                   & Real'Image (Angle (O.Shapes (L).Rotation)) & " radians");
         end loop;
         Check_Against_Truth ("parallel", R, O);
         --  The rig's boxes themselves: every side of them the measurement
         --  needs is in the eye's view, so their sizes come back.
         for L in 1 .. 2 loop
            Check_Within ("parallel: box" & L'Image & " width", O.Size.Sizes (L).Width, 0.02);
            Check_Within ("parallel: box" & L'Image & " thickness", O.Size.Sizes (L).Thickness, 0.01);
         end loop;
         --  The lobes run out of the picture's sides by their roots: the
         --  depth is what is seen of them, and says so.
         Check ((Index (O.Size.Why, "at least") > 0) = (O.Shapes (1).Bordered or else O.Shapes (2).Bordered),
                "a hand says its depth is at least as measured exactly when a lobe runs out of the picture: "
                & To_String (O.Size.Why));
      end if;
   end Parallel_Lobes;

   --  One jaw turning 20 degrees about a hinge at its root against a still
   --  one, as a pivoting gripper does.
   procedure Turning_Jaw is
      R     : Rig;
      Hinge : constant Vec3 := [0.05, 0.005, 0.0];
      Turn  : constant Mat3 := Exp ([0.0, 0.0, -20.0 * Ada.Numerics.Pi / 180.0]);
   begin
      R.Camera := Looking ([1.0, 0.0, -0.6]);
      R.Lobe_Count := 1;
      --  Closed, the jaw lies along x beside the still one; open, it is
      --  turned out about the hinge.
      R.Open_Lobes (1) := Moved ((Centre => [0.08, 0.01, 0.0], Axes => Identity3, Half => [0.03, 0.005, 0.01]),
                                 (Turn => Transpose (Turn), Pivot => Hinge, Shift => Zero3));
      R.Motions (1) := (Turn => Turn, Pivot => Hinge, Shift => Zero3);
      R.Still_Count := 2;
      R.Still (1) := (Centre => [0.025, 0.0, -0.0075], Axes => Identity3, Half => [0.025, 0.05, 0.0075]);
      R.Still (2) := (Centre => [0.08, -0.01, 0.0], Axes => Identity3, Half => [0.03, 0.005, 0.01]);
      declare
         O : constant Outcome := Run (R, Seed => 11);
      begin
         Check (O.Found = 1, "the turning jaw was not found: " & To_String (O.Problem));
         if O.Found = 1 then
            --  Seen mostly on one face, its two views fit, from the start of
            --  a pure shift, a wrong valley: a turn about another axis, the
            --  shift reversed, its sightings scattering twice their noise.
            --  Its sizes then do not stand out of their uncertainty, and the
            --  hand says they are not measured: never sizes off their sigma.
            Check_Against_Truth ("turning", R, O, Or_Unmeasured => True);
         end if;
      end;
   end Turning_Jaw;

   --  What is not measured says so, and why.
   procedure Unpressed is
      R : constant Rig := Parallel;
      O : constant Outcome := Run (R, Seed => 7);
   begin
      if O.Found /= 2 then
         Check (False, "the parallel lobes were not found: " & To_String (O.Problem));
         return;
      end if;
      declare
         No_Tips : constant Tip_Array (1 .. 2, Opening) := [others => [others => (others => <>)]];
         Sized   : constant Hand_Size := Measure (Lobe_Shape_Array'(O.Shapes (1), O.Shapes (2)), No_Tips);
      begin
         for L in 1 .. 2 loop
            Check (not Known (Sized.Sizes (L).Width) and then not Known (Sized.Sizes (L).Thickness)
                   and then not Known (Sized.Sizes (L).Face),
                   "a lobe with no tips pressed has a size: width " & Image (Sized.Sizes (L).Width));
         end loop;
         Check (not Known (Sized.Depth) and then Sized.Axis.Sigma = Real'Last,
                "a hand with no tips pressed has a depth " & Image (Sized.Depth));
         Check (Index (Sized.Why, "not pressed") > 0, "an unpressed hand does not say why: " & To_String (Sized.Why));
      end;
   end Unpressed;

   procedure Displace_Closed_Tip (L : Positive; Tips : in out Tip_Array) is
   begin
      --  The closed press of lobe 1 touched something 20 sigma in front of
      --  the lobe, along its line of sight.
      if L = 1 then
         Tips (L, Closed_Empty).Mean := Tips (L, Closed_Empty).Mean - 20.0 * Press_Sigma * Unit ([1.0, 0.0, -0.6]);
      end if;
   end Displace_Closed_Tip;

   procedure Tips_Disagree is
      R : constant Rig := Parallel;
      O : constant Outcome := Run (R, Seed => 7, Change => Displace_Closed_Tip'Access);
   begin
      Check (O.Found = 2, "the parallel lobes were not found: " & To_String (O.Problem));
      if O.Found = 2 then
         Check (not Known (O.Size.Sizes (1).Width) and then not Known (O.Size.Depth),
                "a lobe whose tips disagree on its scale is measured: width " & Image (O.Size.Sizes (1).Width));
         Check (Index (O.Size.Why, "disagree") > 0, "the disagreement is not said: " & To_String (O.Size.Why));
         Check (Known (O.Size.Sizes (2).Width), "the other lobe is not measured: " & To_String (O.Size.Why));
      end if;
   end Tips_Disagree;

   procedure Bent_Tip is
      --  The part of lobe 1 beyond 90 mm, its tip with it, lands three
      --  pixels off where the rest of the lobe's motion puts it, down the
      --  image: across the lines its points slide along with their depths
      --  (a lobe moving sideways to the eye slides them across the image),
      --  where two views can tell it. Along them it would only look deeper.
      R : Rig := Parallel;
   begin
      R.Bend_Lobe := 1;
      R.Bend_From := 0.09;
      R.Bend_By := 3.0;
      declare
         O : constant Outcome := Run (R, Seed => 7);
      begin
         Check (O.Found = 2, "the parallel lobes were not found: " & To_String (O.Problem));
         if O.Found = 2 then
            Check (not Known (O.Size.Sizes (1).Width) and then not Known (O.Size.Sizes (1).Thickness)
                   and then not Known (O.Size.Depth),
                   "a lobe whose tip does not move with the rest is measured: width "
                   & Image (O.Size.Sizes (1).Width) & " (" & To_String (O.Size.Why) & ")");
            Check (Index (O.Size.Why, "lobe 1:") > 0, "the bent lobe does not say why: " & To_String (O.Size.Why));
            Check (Known (O.Size.Sizes (2).Width), "the straight lobe beside it is not measured: "
                   & To_String (O.Size.Why));
         end if;
      end;
   end Bent_Tip;

   procedure In_A_Task is
      --  The fit runs in the estimators, which also run in the decider's
      --  task with GNAT's default stack: a VGA eye that each lobe fills
      --  tens of thousands of pixels of.
      R        : Rig := Parallel;
      Done     : Boolean := False with Atomic;
      Fitted_N : Natural := 0;
      Failure  : Unbounded_String;
      Took     : Duration := 0.0;
   begin
      R.Width := 640;
      R.Height := 480;
      R.Focal := 500.0;
      declare
         task Decider;
         task body Decider is
            Start : constant Ada.Calendar.Time := Ada.Calendar.Clock;
            O     : constant Outcome := Run (R, Seed => 3);
            use type Ada.Calendar.Time;
         begin
            Took := Ada.Calendar.Clock - Start;
            for L in 1 .. O.Found loop
               Fitted_N := Fitted_N + Boolean'Pos (Fitted (O.Shapes (L)));
            end loop;
            Done := True;
         exception
            when E : others =>
               Failure := To_Unbounded_String (Ada.Exceptions.Exception_Information (E));
         end Decider;
      begin
         null;
      end;
      Check (Done, "the shape's fit failed in a task with the default stack: " & To_String (Failure));
      Check (not Done or else Fitted_N = 2, "two lobes in a VGA eye gave" & Fitted_N'Image & " fitted shapes");
      Driver.Log.Line (Driver.Log.Robot, "hand.shape.task: the rig and two VGA lobes' shapes took"
                       & Duration'Image (Took) & " s");
   end In_A_Task;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.shape.parallel",
                             "two lobes of known size closing straight do not come back within their sigma",
                             Parallel_Lobes'Access);
      Driver.Tests.Register ("hand.shape.turn", "a jaw turning about a hinge does not come back within its sigma",
                             Turning_Jaw'Access);
      Driver.Tests.Register ("hand.shape.unpressed", "a hand whose tips are not pressed reports sizes",
                             Unpressed'Access);
      Driver.Tests.Register ("hand.shape.disagree", "a lobe whose two tips disagree on its scale reports sizes",
                             Tips_Disagree'Access);
      Driver.Tests.Register ("hand.shape.bent", "a lobe whose tip does not move with the rest of it reports sizes",
                             Bent_Tip'Access);
      Driver.Tests.Register ("hand.shape.task", "the shape's fit fails in a task with the default stack",
                             In_A_Task'Access);
   end Register;

end Driver.Robot.Hand.Shape.Tests;
