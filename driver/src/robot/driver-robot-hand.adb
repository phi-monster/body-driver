with Ada.Containers.Vectors;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Log;
with Driver.Robot.Channels;
with Driver.Robot.Hand.Frames;
with Driver.Robot.Hand.Lobes;
with Driver.Robot.Hand.Lowering;
with Driver.Robot.Hand.Presses;
with Driver.Robot.Hand.Shape;
with Driver.Robot.Hand.Slide;
with Driver.Robot.Hand.Sweep;
with Driver.Robot.Hand.Tips;
with Driver.Robot.Hand.Views;
with Driver.Robot.Kinematics;
with Driver.Robot.Steps;
with Driver.Robot.Stillness;

package body Driver.Robot.Hand is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;
   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;
   use type Driver.Robot.Hand.Sweep.Progress;
   use type Driver.Robot.Hand.Lowering.Verdict;

   package Sweeps renames Driver.Robot.Hand.Sweep;

   --  One closer group as one eye sees it.
   type Pair is record
      Group    : Group_Id;
      Eye      : Eye_Id;
      Arm      : Arm_Id;
      Sweep    : Sweeps.State;
   end record;

   package Pair_Vectors is new Ada.Containers.Vectors (Positive, Pair);

   type Patch_Array is array (Opening) of Driver.Robot.Hand.Slide.Patch;

   --  How a lobe's finger closes in its eye's picture: the unit way from where
   --  its tip is open to where it is closed, and how far it goes.
   type Closing_Way is record
      Known : Boolean := False;
      U, V  : Real := 0.0;
      Reach : Real := 0.0;
   end record;

   type Lobe_Record is record
      Channel : Positive;
      Sights  : Sight_Array;
      Patches : Patch_Array;   --  what its tip looked like at each opening, in the views of the sweep
      Way     : Closing_Way;
      Shape   : Driver.Robot.Hand.Shape.Lobe_Shape;   --  its surface as its own eye saw it, up to scale
      Size    : Driver.Robot.Hand.Shape.Lobe_Size;    --  what the presses so far make of it
   end record;

   package Lobe_Record_Vectors is new Ada.Containers.Vectors (Positive, Lobe_Record);

   type Reading_Array is array (Opening) of Driver.Robot.Hand.Views.Reading_Holders.Holder;

   type Hand_Record is record
      Group    : Group_Id;
      Arm      : Arm_Id;
      Eye      : Eye_Id;
      Readings : Reading_Array;
      Lobes    : Lobe_Record_Vectors.Vector;
      Watch    : Driver.Robot.Hand.Presses.Watcher;   --  the arm's presses, from the stream
      Book     : Driver.Robot.Hand.Tips.Book;         --  the presses kept and the tips they measure
      Lowering : Driver.Robot.Hand.Lowering.Track;    --  the descent under way, judged by where the hand went
      Judged   : Natural := 0;                        --  the beat the latest push judged for lowering began
      Stalled  : Natural := 0;                        --  the beat the latest push judged stalled began
      Stalls   : Natural := 0;                        --  how many pushes were judged stalled
      Depth    : Estimate;
      Axis     : Direction_Estimate;
      Unsized  : Unbounded_String;                    --  what of its sizes is not measured, and why
   end record;

   package Hand_Vectors is new Ada.Containers.Vectors (Hand_Id, Hand_Record);

   type Hand_Data is record
      Pairs : Pair_Vectors.Vector;
      Found : Hand_Vectors.Vector;
   end record;

   procedure Free is new Ada.Unchecked_Deallocation (Hand_Data, Hand_Data_Access);

   overriding procedure Finalize (H : in out Hands) is
   begin
      Free (H.Data);
   end Finalize;

   function Image (P : Driver.Images.Pixel) return String is
     ("(" & Driver.Log.Image (P.U, 1) & ", " & Driver.Log.Image (P.V, 1) & ")");

   --  Every other group's readings in group order, and their noises in the
   --  same order: what must not change while one view of a closer is taken.

   function Rest_Of (O : Observation; G : Group_Id) return Real_Array is
      Total : Natural := 0;
   begin
      for Other in O.Readings.First_Index .. O.Readings.Last_Index loop
         if Other /= G then
            Total := Total + O.Readings.Element (Other)'Length;
         end if;
      end loop;
      declare
         R : Real_Array (1 .. Total);
         K : Natural := 0;
      begin
         for Other in O.Readings.First_Index .. O.Readings.Last_Index loop
            if Other /= G then
               for V of O.Readings (Other) loop
                  K := K + 1;
                  R (K) := V;
               end loop;
            end if;
         end loop;
         return R;
      end;
   end Rest_Of;

   function Rest_Moved (M : Model; G : Group_Id; Before, After : Real_Array) return Boolean is
      --  Every other group's readings, as Rest_Of lays them out, through the
      --  body's own one test of motion: a channel an eye watches moves only
      --  by a step that eye can see. Laid out otherwise, something came or
      --  went: that is a change.
      K : Natural := 0;
   begin
      if Before'Length /= After'Length then
         return True;
      end if;
      for Other in 1 .. Group_Id'Base (Group_Count (M)) loop
         if Other /= G then
            declare
               N : constant Natural := Group_Size (M, Other);
            begin
               if K + N > Before'Length then
                  return True;
               end if;
               declare
                  D : constant Real_Array (1 .. N) :=
                    [for C in 1 .. N => After (After'First + K + C - 1) - Before (Before'First + K + C - 1)];
               begin
                  if Driver.Robot.Channels.Visible (M, Other, D) then
                     return True;
                  end if;
               end;
               K := K + N;
            end;
         end if;
      end loop;
      return K /= Before'Length;
   end Rest_Moved;

   function All_Read (O : Observation) return Boolean is
     (for all G in O.Readings.First_Index .. O.Readings.Last_Index => Driver.Observations.Has_Reading (O, G));

   function Has_Pair (D : Hand_Data; G : Group_Id; E : Eye_Id) return Boolean is
     (for some P of D.Pairs => P.Group = G and then P.Eye = E);

   procedure Find_Pairs (D : in out Hand_Data; M : Model; O : Observation);
   --  Every closer group with the eyes on its own arm, once the body has
   --  measured them; by the body's roles as they are now, which the boot
   --  re-reads: a pair whose group is no longer a closer, or no longer of that
   --  arm, goes, and so does a hand found from it. An eye that rides on
   --  another arm or on nothing is not a pair: what the eye shows of the robot
   --  itself as the arm moves (Driver.Robot.Hand.Selfsight) is the arm-carried
   --  eye's.

   function Still_A_Pair (P : Pair; M : Model) return Boolean is
     (Role (M, P.Group) = Closer and then Closer_Arm (M, P.Group) = P.Arm
      and then Eye_Mount (M, P.Eye).Kind = Arm_Carried and then Eye_Mount (M, P.Eye).Arm = P.Arm);

   procedure Find_Pairs (D : in out Hand_Data; M : Model; O : Observation) is
   begin
      declare
         I : Positive := 1;
      begin
         while I <= Natural (D.Pairs.Length) loop
            if Still_A_Pair (D.Pairs (I), M) then
               I := I + 1;
            else
               Driver.Log.Line (Driver.Log.Robot, "hand: group" & D.Pairs (I).Group'Image & " is no longer a closer"
                                & " of arm" & D.Pairs (I).Arm'Image & " watched in eye" & D.Pairs (I).Eye'Image
                                & "; its sweep there is dropped");
               D.Pairs.Delete (I);
            end if;
         end loop;
      end;
      declare
         I : Hand_Id := 1;
      begin
         while I <= D.Found.Last_Index loop
            if Role (M, D.Found (I).Group) = Closer and then Closer_Arm (M, D.Found (I).Group) = D.Found (I).Arm then
               I := I + 1;
            else
               Driver.Log.Line (Driver.Log.Robot, "hand: group" & D.Found (I).Group'Image & " is no longer a closer"
                                & " of arm" & D.Found (I).Arm'Image & "; the hand found from it is dropped");
               D.Found.Delete (I);
            end if;
         end loop;
      end;
      for G in 1 .. Group_Id'Base (Group_Count (M)) loop
         if Role (M, G) = Closer and then Closer_Arm (M, G) > 0 and then Driver.Observations.Has_Reading (O, G) then
            for E in 1 .. Eye_Id'Base (Eye_Count (M)) loop
               declare
                  Arm      : constant Arm_Id := Closer_Arm (M, G);
                  Mount_Of : constant Mount := Eye_Mount (M, E);
                  Own      : constant Boolean := Mount_Of.Kind = Arm_Carried and then Mount_Of.Arm = Arm;
               begin
                  if Own and then Driver.Observations.Has_Image (O, E) and then not Has_Pair (D, G, E) then
                     declare
                        Channels : constant Positive := O.Readings.Element (G)'Length;
                        Noise    : Real_Array (1 .. Channels);
                     begin
                        for C in Noise'Range loop
                           Noise (C) := Reading_Noise (M, G, C);
                        end loop;
                        D.Pairs.Append
                          (Pair'(Group => G,
                            Eye   => E,
                            Arm   => Arm,
                            Sweep => Sweeps.Start (Driver.Images.Width (O.Images (E)), Driver.Images.Height (O.Images (E)),
                                                   Channels, Noise)));
                        Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " on arm" & Arm'Image
                                         & " is watched in eye" & E'Image & ", its own");
                     end;
                  end if;
               end;
            end loop;
         end if;
      end loop;
   end Find_Pairs;

   procedure Feed (P : in out Pair; M : Model; O : Observation);

   procedure Feed (P : in out Pair; M : Model; O : Observation) is
      Complete : constant Boolean := All_Read (O) and then Driver.Observations.Has_Image (O, P.Eye);
      function Moved (Before, After : Real_Array) return Boolean is (Rest_Moved (M, P.Group, Before, After));
   begin
      --  A view of the pair's eye needs that eye's picture to have stopped
      --  changing (the one stop rule, Driver.Robot.Stillness): its validity
      --  is its eye's, not the far arm's. Whether the rest of the body moved
      --  between two of its readings is the body's one test of motion
      --  (Rest_Moved), not the views' own: by its readings' noise alone, A12's
      --  arms, creeping 1.2e-12 rad a beat for an hour, moved at every beat,
      --  every beat started the view again, and none ever had two frames.
      Sweeps.Observe (P.Sweep, O, Still => Driver.Robot.Stillness.Eye_Settled (M, P.Eye) and then Complete,
                      Closer => O.Readings (P.Group), Rest => Rest_Of (O, P.Group), Image => O.Images (P.Eye),
                      Rest_Moved => Moved'Access);
   end Feed;

   function Sizes_Text (R : Hand_Record) return String;
   --  The hand's sizes as measured so far, for the log.

   function Sizes_Text (R : Hand_Record) return String is
      Text : Unbounded_String;
      function Image (E : Estimate) return String is
        (if Known (E) then Driver.Log.Image (E.Value, 4) & " +- " & Driver.Log.Image (E.Sigma, 4) else "unmeasured");
   begin
      for L in R.Lobes.First_Index .. R.Lobes.Last_Index loop
         Append (Text, "lobe" & L'Image & " width " & Image (R.Lobes (L).Size.Width)
                 & ", thickness " & Image (R.Lobes (L).Size.Thickness)
                 & ", face " & Image (R.Lobes (L).Size.Face) & " ahead of its tip"
                 & (if Driver.Robot.Hand.Shape.Fitted (R.Lobes (L).Shape)
                    then " (" & Natural'Image (Driver.Robot.Hand.Shape.Kept (R.Lobes (L).Shape)) & " points, scatter "
                         & Driver.Log.Image (Driver.Robot.Hand.Shape.Scatter (R.Lobes (L).Shape), 3) & ")"
                    else "")
                 & "; ");
      end loop;
      Append (Text, "depth " & Image (R.Depth));
      if Length (R.Unsized) > 0 then
         Append (Text, "; " & To_String (R.Unsized));
      end if;
      return To_String (Text);
   end Sizes_Text;

   procedure Size_Up (R : in out Hand_Record; Id : Hand_Id);
   --  The hand's sizes again, from its lobes' shapes and the tips its
   --  presses measure now.

   procedure Size_Up (R : in out Hand_Record; Id : Hand_Id) is
      package Shapes renames Driver.Robot.Hand.Shape;
      N      : constant Natural := Natural (R.Lobes.Length);
      Fits   : Shapes.Lobe_Shape_Array (1 .. N);
      Tipped : Shapes.Tip_Array (1 .. N, Opening);
      Unmeasured : Point_Estimate;
   begin
      if N = 0 then
         return;
      end if;
      --  A size rests on tips two presses have landed on: a provisional tip
      --  is a bound, and what is taken from it would be taken for measured.
      for L in 1 .. N loop
         Fits (L) := R.Lobes (L).Shape;
         for O in Opening loop
            Tipped (L, O) := (if Driver.Robot.Hand.Tips.Confirmed (R.Book, L, O)
                              then Driver.Robot.Hand.Tips.Tip (R.Book, L, O) else Unmeasured);
         end loop;
      end loop;
      declare
         Sized : constant Shapes.Hand_Size := Shapes.Measure (Fits, Tipped);
      begin
         for L in 1 .. N loop
            R.Lobes (L).Size := Sized.Sizes (L);
         end loop;
         R.Depth := Sized.Depth;
         R.Axis := Sized.Axis;
         R.Unsized := Sized.Why;
      end;
      Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & " sizes: " & Sizes_Text (R));
   end Size_Up;

   procedure Keep (D : in out Hand_Data; Made : in out Hand_Record);
   --  The hand found from a closer group, among the hands. A hand measured
   --  again keeps its presses; they are given to the new lobes by number, or
   --  forgotten when the number changed.

   procedure Keep (D : in out Hand_Data; Made : in out Hand_Record) is
      Table : Driver.Robot.Hand.Tips.Sight_Table (1 .. Natural (Made.Lobes.Length));
   begin
      for L in Table'Range loop
         for Which in Opening loop
            Table (L) (Which) := (Known  => Made.Lobes (L).Sights (Which).Known,
                                  Ray    => Made.Lobes (L).Sights (Which).Ray,
                                  Spread => Made.Lobes (L).Sights (Which).Spread,
                                  Travel => Made.Lobes (L).Way.Reach * Made.Lobes (L).Sights (Which).Pitch,
                                  Pitch  => Made.Lobes (L).Sights (Which).Pitch);
         end loop;
      end loop;
      for Id in D.Found.First_Index .. D.Found.Last_Index loop
         if D.Found (Id).Group = Made.Group then
            Made.Watch := D.Found (Id).Watch;
            Made.Book := D.Found (Id).Book;
            Driver.Robot.Hand.Tips.Set_Sights (Made.Book, Table);
            Size_Up (Made, Id);
            D.Found.Replace_Element (Id, Made);
            return;
         end if;
      end loop;
      Driver.Robot.Hand.Tips.Set_Sights (Made.Book, Table);
      Size_Up (Made, D.Found.Last_Index + 1);
      D.Found.Append (Made);
   end Keep;

   procedure Drop (D : in out Hand_Data; Group : Group_Id);
   --  The hand found from a closer group goes, when its ends were measured
   --  again and give no lobes: a measurement of older ends no longer stands.

   procedure Drop (D : in out Hand_Data; Group : Group_Id) is
   begin
      for Id in D.Found.First_Index .. D.Found.Last_Index loop
         if D.Found (Id).Group = Group then
            Driver.Log.Line (Driver.Log.Robot, "hand: the hand of closer group" & Group'Image
                             & " is dropped: its ends were measured again and gave no lobes");
            D.Found.Delete (Id);
            return;
         end if;
      end loop;
   end Drop;

   procedure Rebuild (D : in out Hand_Data; P : Pair; M : Model);
   --  The hand of a closer group from what its own eye measured: every
   --  measured channel's lobes with their tips at both openings.

   procedure Rebuild (D : in out Hand_Data; P : Pair; M : Model) is
      Made : Hand_Record := (Group => P.Group, Arm => P.Arm, Eye => P.Eye, others => <>);

      function Seen_Tip
        (V      : Driver.Robot.Hand.Views.View;
         Known  : Boolean;
         Px     : Driver.Images.Pixel;
         Region : Driver.Images.Mask;
         Centre : Driver.Images.Pixel) return Sight
      is
      begin
         if not Known then
            return (Known => False, Pixel => Px, Ray => <>, Spread => 0.0, Pitch => 0.0);
         end if;
         declare
            Line  : constant Ray_Estimate := Eye_Ray (M, P.Eye, Px);
            Next  : constant Ray_Estimate := Eye_Ray (M, P.Eye, (U => Px.U + 1.0, V => Px.V));
            --  The angle one pixel spans at the tip.
            Pitch : constant Real :=
              (if Line.Direction.Sigma < Real'Last and then Next.Direction.Sigma < Real'Last
               then Arctan (abs Cross (Line.Direction.Unit_Vector, Next.Direction.Unit_Vector),
                            Line.Direction.Unit_Vector * Next.Direction.Unit_Vector)
               else 0.0);
         begin
            return (Known  => True,
                    Pixel  => Px,
                    Ray    => Driver.Robot.Hand.Frames.Into (Eye_In_Tool (M, P.Eye, V.Seen), Line),
                    Spread => Pitch * Driver.Robot.Hand.Lobes.Tip_Spread
                                        (Driver.Robot.Hand.Lobes.Tip_Cap (Region, Centre, Px), Px),
                    Pitch  => Pitch);
         end;
      end Seen_Tip;

      --  The way the finger closes in the picture: from its open tip to its closed one.
      function Closing_Way_Of (From, To : Driver.Images.Pixel) return Closing_Way is
         Reach : constant Real := Sqrt ((To.U - From.U) ** 2 + (To.V - From.V) ** 2);
      begin
         if Reach = 0.0 then
            return (others => <>);
         end if;
         return (Known => True, U => (To.U - From.U) / Reach, V => (To.V - From.V) / Reach, Reach => Reach);
      end Closing_Way_Of;

      --  What a lobe's tip looked like in a view where it stood still: the points of its edge in the
      --  region at its tip.
      function Patch_At
        (Of_View : Driver.Robot.Hand.Views.View;
         Lobe    : Driver.Images.Mask;
         Centre  : Driver.Images.Pixel;
         Tip     : Driver.Images.Pixel;
         Way     : Closing_Way) return Driver.Robot.Hand.Slide.Patch
      is
         Cap : constant Driver.Images.Mask := Driver.Robot.Hand.Lobes.Tip_Cap (Lobe, Centre, Tip);
      begin
         return Driver.Robot.Hand.Slide.Take (Lobe, Cap, Of_View.Frames, Way.U, Way.V, Way.Reach);
      end Patch_At;
   begin
      for C in 1 .. Sweeps.Channels (P.Sweep) loop
         if Sweeps.Status (P.Sweep, C) = Sweeps.Measured and then Sweeps.Closing_Known (P.Sweep, C) then
            declare
               Low  : constant Driver.Robot.Hand.Views.View := Sweeps.Low_End (P.Sweep, C);
               High : constant Driver.Robot.Hand.Views.View := Sweeps.High_End (P.Sweep, C);
               Closed_High : constant Boolean := Sweeps.Closed_End_Is_High (P.Sweep, C);
            begin
               for L of Sweeps.Lobes_Of (P.Sweep, C) loop
                  declare
                     At_Low  : constant Sight := Seen_Tip (Low, L.Tip_Known_Here, L.Tip_Here, L.Here, L.Centre_Here);
                     At_High : constant Sight := Seen_Tip (High, L.Tip_Known_There, L.Tip_There, L.There, L.Centre_There);
                     Way     : constant Closing_Way :=
                       (if L.Tip_Known_Here and then L.Tip_Known_There
                        then Closing_Way_Of (From => (if Closed_High then L.Tip_Here else L.Tip_There),
                                             To   => (if Closed_High then L.Tip_There else L.Tip_Here))
                        else (others => <>));
                  begin
                     Made.Lobes.Append
                       (Lobe_Record'(Channel => C,
                                     Sights  => [Open         => (if Closed_High then At_Low else At_High),
                                                 Closed_Empty => (if Closed_High then At_High else At_Low)],
                                     Patches =>
                                       [Open         => (if Way.Known
                                                         then Patch_At ((if Closed_High then Low else High),
                                                                        (if Closed_High then L.Here else L.There),
                                                                        (if Closed_High then L.Centre_Here
                                                                         else L.Centre_There),
                                                                        (if Closed_High then L.Tip_Here else L.Tip_There),
                                                                        Way)
                                                         else Driver.Robot.Hand.Slide.Empty),
                                        Closed_Empty => (if Way.Known
                                                         then Patch_At ((if Closed_High then High else Low),
                                                                        (if Closed_High then L.There else L.Here),
                                                                        (if Closed_High then L.Centre_There
                                                                         else L.Centre_Here),
                                                                        (if Closed_High then L.Tip_There else L.Tip_Here),
                                                                        Way)
                                                         else Driver.Robot.Hand.Slide.Empty)],
                                     Way     => Way,
                                     Shape   => Driver.Robot.Hand.Shape.Unfitted
                                       ("its pixels are not matched between the ends: the hand asks no instrument"),
                                     Size    => <>));
                  end;
               end loop;
               --  The group's readings at each opening: this channel at its end,
               --  the others as they were held while it was swept.
               for Which in Opening loop
                  declare
                     At_End : constant Real_Array :=
                       (if (Which = Open) = Closed_High then Low.Closer.Element else High.Closer.Element);
                  begin
                     if Made.Readings (Which).Is_Empty then
                        Made.Readings (Which) := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (At_End);
                     else
                        declare
                           Merged : Real_Array := Made.Readings (Which).Element;
                        begin
                           Merged (Merged'First + C - 1) := At_End (At_End'First + C - 1);
                           Made.Readings (Which) := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (Merged);
                        end;
                     end if;
                  end;
               end loop;
            end;
         end if;
      end loop;
      if not Made.Lobes.Is_Empty then
         Keep (D, Made);
      else
         Drop (D, P.Group);
      end if;
   end Rebuild;

   procedure Adopt
     (H         : in out Hands;
      Group     : Group_Id;
      Arm       : Arm_Id;
      Eye       : Eye_Id;
      Open_At   : Real_Array;
      Closed_At : Real_Array;
      Lobes     : Sight_Rows)
   is
      Made : Hand_Record := (Group => Group, Arm => Arm, Eye => Eye, others => <>);
   begin
      if H.Data = null then
         H.Data := new Hand_Data;
      end if;
      Made.Readings := [Open         => Driver.Robot.Hand.Views.Reading_Holders.To_Holder (Open_At),
                        Closed_Empty => Driver.Robot.Hand.Views.Reading_Holders.To_Holder (Closed_At)];
      for Row of Lobes loop
         Made.Lobes.Append
           (Lobe_Record'(Channel => 1,
                         Sights  => Row,
                         Patches => [others => Driver.Robot.Hand.Slide.Empty],
                         Way     => <>,
                         Shape   => Driver.Robot.Hand.Shape.Unfitted ("the hand was given, not swept"),
                         Size    => <>));
      end loop;
      if not Made.Lobes.Is_Empty then
         Keep (H.Data.all, Made);
      end if;
   end Adopt;

   procedure Report (P : in out Pair; D : in out Hand_Data; M : Model);
   --  Says what became of every channel whose ends are new since it last
   --  said, and makes the hand of the measured ones again: a hand found from
   --  older ends does not stand when the ends are new and give no lobes.

   procedure Report (P : in out Pair; D : in out Hand_Data; M : Model) is
      Said : Boolean := False;
   begin
      for C in 1 .. Sweeps.Channels (P.Sweep) loop
         if Sweeps.Unannounced (P.Sweep, C) then
            Sweeps.Announce (P.Sweep, C);
            Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & P.Group'Image & " channel" & C'Image & " in eye"
                             & P.Eye'Image & ": " & Sweeps.Account (P.Sweep, C));
            if Sweeps.Status (P.Sweep, C) = Sweeps.Measured then
               declare
                  Number : Natural := 0;
               begin
                  for L of Sweeps.Lobes_Of (P.Sweep, C) loop
                     Number := Number + 1;
                     Driver.Log.Line
                       (Driver.Log.Robot, "hand: closer group" & P.Group'Image & " channel" & C'Image & " in eye"
                        & P.Eye'Image & ": lobe" & Number'Image & ":" & L.Count_Here'Image & " pixels at the low reading, "
                        & (if L.Tip_Known_Here then "tip " & Image (L.Tip_Here) else "no tip")
                        & (if L.Bordered_Here then " from the border" else "") & ";" & L.Count_There'Image
                        & " at the high reading, " & (if L.Tip_Known_There then "tip " & Image (L.Tip_There) else "no tip")
                        & (if L.Bordered_There then " from the border" else ""));
                  end loop;
               end;
            end if;
            Said := True;
         end if;
      end loop;
      if Said then
         Rebuild (D, P, M);
      end if;
   end Report;

   function Opening_Of (R : Hand_Record; M : Model; Readings : Real_Array; Which : out Opening) return Boolean;
   --  The opening the closer was at, when its readings equal those of one
   --  opening within their noise and not those of the other.

   function Opening_Of (R : Hand_Record; M : Model; Readings : Real_Array; Which : out Opening) return Boolean is
      function At_It (O : Opening) return Boolean is
         Measured : constant Real_Array := R.Readings (O).Element;
      begin
         if Measured'Length /= Readings'Length then
            return False;
         end if;
         --  Each reading and the one measured at the opening carry the
         --  channel's noise, so their difference carries it twice over.
         return (for all C in 1 .. Readings'Length =>
                   not Significant (Readings (Readings'First + C - 1) - Measured (Measured'First + C - 1),
                                    Sqrt (2.0) * Reading_Noise (M, R.Group, C)));
      end At_It;
   begin
      Which := Open;
      if R.Readings (Open).Is_Empty or else R.Readings (Closed_Empty).Is_Empty then
         return False;
      end if;
      if At_It (Open) and then not At_It (Closed_Empty) then
         Which := Open;
         return True;
      elsif At_It (Closed_Empty) and then not At_It (Open) then
         Which := Closed_Empty;
         return True;
      end if;
      return False;
   end Opening_Of;

   function Tips_Said (R : Hand_Record; Which : Opening) return String;
   --  What the presses make of each lobe's tip at an opening: how far along
   --  its line of sight in the arm's unit, and whether a second press from
   --  another pose has landed on it (confirmed) or not (provisional).

   function Tips_Said (R : Hand_Record; Which : Opening) return String is
      Text : Unbounded_String;
   begin
      for L in 1 .. Natural (R.Lobes.Length) loop
         declare
            Reach  : constant Estimate := Driver.Robot.Hand.Tips.Distance (R.Book, L, Which);
            Across : constant Real_Array := Driver.Robot.Hand.Tips.Across (R.Book, L, Which);
         begin
            Append (Text, (if L > 1 then ", " else "") & "lobe" & L'Image & " "
                    & (if not Known (Reach) then "none"
                       else (if Driver.Robot.Hand.Tips.Confirmed (R.Book, L, Which) then "confirmed " else "provisional ")
                            & Driver.Log.Image (Reach.Value, 4) & " +- " & Driver.Log.Image (Reach.Sigma, 4)
                            & " along its sight, on" & Driver.Robot.Hand.Tips.Agreeing (R.Book, L, Which)'Image
                            & " presses, its tip region" & Driver.Log.Image (Reach.Value * R.Lobes (L).Sights (Which).Spread, 3)
                            & " wide across it, left" & Driver.Log.Image (Across (1), 3) & " and" & Driver.Log.Image (Across (2), 3)
                            & " along the axes the tilts of the presses told most and least ("
                            & (if Driver.Robot.Hand.Tips.Tested (R.Book, L, Which) then "tested across the sight"
                               else "not tested across the sight") & ")"));
         end;
      end loop;
      return To_String (Text);
   end Tips_Said;

   function Measured_Slides
     (R : Hand_Record; Id : Hand_Id; O : Observation; Which : Opening) return Driver.Robot.Hand.Tips.Slid_Row;
   --  How far each lobe's finger stands from where the closer's reading puts
   --  it, in the picture of the beat of a press made at that opening: the
   --  pixels of its tip as the sweep saw them, found again under the press
   --  (Driver.Robot.Hand.Slide). Said in the log for every lobe, measured or not.

   function Measured_Slides
     (R : Hand_Record; Id : Hand_Id; O : Observation; Which : Opening) return Driver.Robot.Hand.Tips.Slid_Row
   is
      Result : Driver.Robot.Hand.Tips.Slid_Row (1 .. Natural (R.Lobes.Length)) := [others => <>];
      Text   : Unbounded_String;
   begin
      if O.Images.Is_Empty or else R.Eye > O.Images.Last_Index or else not Driver.Observations.Has_Image (O, R.Eye) then
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": no picture at the press: no slide measured");
         return Result;
      end if;
      declare
         Under : constant Driver.Robot.Hand.Slide.Picture := Driver.Robot.Hand.Slide.See (O.Images (R.Eye));
      begin
         for L in Result'Range loop
            declare
               Lobe  : constant Lobe_Record := R.Lobes (L);
               Shift : Driver.Robot.Hand.Slide.Shift;
            begin
               Append (Text, (if L > 1 then "; " else "") & "lobe" & L'Image & " ");
               if not Lobe.Way.Known or else Driver.Robot.Hand.Slide.Points (Lobe.Patches (Which)) = 0 then
                  Append (Text, "has no edge to look for");
               else
                  Driver.Robot.Hand.Slide.Measure (Lobe.Patches (Which), Under, Shift);
                  if Shift.Known then
                     Result (L) := (Known          => True,
                                    Pixels         => Shift.By,
                                    Pixels_Sigma   => Shift.Sigma,
                                    Fraction       => Shift.By / Lobe.Way.Reach,
                                    Fraction_Sigma => Shift.Sigma / Lobe.Way.Reach);
                     Append (Text, "slid" & Driver.Log.Image (Shift.By, 2) & " +- " & Driver.Log.Image (Shift.Sigma, 2)
                             & " pixels, " & Driver.Log.Image (100.0 * Shift.By / Lobe.Way.Reach, 2) & " % of the"
                             & Driver.Log.Image (Lobe.Way.Reach, 1) & " it closes");
                  else
                     Append (Text, "not found at" & Driver.Log.Image (Shift.By, 2) & " pixels");
                  end if;
                  Append (Text, " (the edge steps by" & Driver.Log.Image (Shift.Peak, 1) & " levels there,"
                          & Driver.Log.Image (Shift.Still, 1) & " where it stood, the others by" & Driver.Log.Image (Shift.Typical, 1)
                          & " about their middle)");
               end if;
            end;
         end loop;
      end;
      Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": under the press at the " & Opening'Image (Which)
                       & " opening, at beat" & O.Beat'Image & ", the fingers stand from where the reading puts them: "
                       & To_String (Text));
      return Result;
   end Measured_Slides;

   procedure Judge_Push (R : in out Hand_Record; Id : Hand_Id; M : Model; O : Observation);
   --  Judges the arm's latest push, once it has ended at rest, by where its
   --  readings put the points of the hand (Driver.Robot.Hand.Lowering): the
   --  tool's origin and the tips measured at the opening the closer stands at.
   --  A push the arm followed and the hand did not go down with is a stall:
   --  the hand lies on what it met, as a blocked push says too (A17's third
   --  press, 4,600 beats on the table). Read from the stream alone, so that a
   --  replay gives the verdicts of the run it replays.

   procedure Judge_Push (R : in out Hand_Record; Id : Hand_Id; M : Model; O : Observation) is
      Arm_Is : constant Group_Id := Arm_Group (M, R.Arm);
   begin
      if Driver.Robot.Steps.Episodes (M, Arm_Is) = 0 then
         return;
      end if;
      declare
         E : constant Episode := Driver.Robot.Steps.Latest (M, Arm_Is);
      begin
         if not (E.Ended and then E.Settled and then E.Rested) or else E.Start = R.Judged then
            return;
         end if;
         R.Judged := E.Start;
         if E.Start < 1 or else not Driver.Robot.Channels.Has_Reading (M, Arm_Is, E.Start - 1)
           or else not Driver.Robot.Channels.Has_Reading (M, Arm_Is, E.End_At)
           or else not Driver.Robot.Channels.Has_Target (M, Arm_Is, E.Start)
         then
            return;
         end if;
         declare
            Size   : constant Natural := Group_Size (M, Arm_Is);
            Up     : constant Direction_Estimate := Up_In_Arm (M, R.Arm);
            Before : constant Real_Array :=
              [for C in 1 .. Size => Driver.Robot.Channels.Reading (M, Arm_Is, E.Start - 1, C)];
            Asked  : constant Real_Array := [for C in 1 .. Size => Driver.Robot.Channels.Target (M, Arm_Is, E.Start, C)];
            After  : constant Real_Array := [for C in 1 .. Size => Driver.Robot.Channels.Reading (M, Arm_Is, E.End_At, C)];

            function Seen_At (Arm : Real_Array) return Observation is
               Then_Read : Observation;
            begin
               Then_Read.Readings := O.Readings;
               Then_Read.Readings.Replace_Element (Arm_Is, Arm);
               return Then_Read;
            end Seen_At;

            function Norm (V : Real_Array) return Real is
               Sum : Real := 0.0;
            begin
               for X of V loop
                  Sum := Sum + X * X;
               end loop;
               return Sqrt (Sum);
            end Norm;

            --  What the readings did: how long the push asked, and how far short of its target they stopped, the
            --  whole vector and not only the part along the ask.
            Ask_Vec   : constant Real_Array := [for C in 1 .. Size => Asked (C) - Before (C)];
            Short_Vec : constant Real_Array := [for C in 1 .. Size => Asked (C) - After (C)];
            Joints    : constant Driver.Robot.Hand.Lowering.Joint_Push :=
              (Asked  => Norm (Ask_Vec) > 0.0 and then Driver.Robot.Channels.Visible (M, Arm_Is, Ask_Vec),
               Length => Norm (Ask_Vec),
               Short  => Norm (Short_Vec),
               Seen   => Driver.Robot.Channels.Visible (M, Arm_Is, Short_Vec));

            From   : constant Pose_Estimate := Tool_In_Arm (M, R.Arm, Seen_At (Before));
            Target : constant Pose_Estimate := Tool_In_Arm (M, R.Arm, Seen_At (Asked));
            To     : constant Pose_Estimate := Tool_In_Arm (M, R.Arm, Seen_At (After));
         begin
            if Up.Sigma = Real'Last or else From.Position_Covariance (1, 1) = Real'Last
              or else Target.Position_Covariance (1, 1) = Real'Last or else To.Position_Covariance (1, 1) = Real'Last
            then
               return;
            end if;
            declare
               Closer : constant Natural := Group_Size (M, R.Group);
               Lobes  : constant Natural := Natural (R.Lobes.Length);
               Where  : Driver.Robot.Hand.Lowering.Points (1 .. 1 + Lobes) := [others => Zero3];
               Whose  : array (1 .. 1 + Lobes) of Natural := [others => 0];   --  the lobe a point is the tip of
               Count  : Natural := 1;
               Which  : Opening;
               Said   : Driver.Robot.Hand.Lowering.Judgment;
            begin
               if Driver.Robot.Channels.Has_Reading (M, R.Group, E.End_At)
                 and then Opening_Of (R, M, [for C in 1 .. Closer => Driver.Robot.Channels.Reading (M, R.Group, E.End_At, C)],
                                      Which)
               then
                  for L in 1 .. Lobes loop
                     declare
                        Tip : constant Point_Estimate := Driver.Robot.Hand.Tips.Tip (R.Book, L, Which);
                     begin
                        if Known (Tip) then
                           Count := Count + 1;
                           Where (Count) := Tip.Mean;
                           Whose (Count) := L;
                        end if;
                     end;
                  end loop;
               end if;
               Driver.Robot.Hand.Lowering.Judge
                 (R.Lowering, From, Target, To, -Up.Unit_Vector, Where (1 .. Count), Joints, Said);
               if Said.Result = Driver.Robot.Hand.Lowering.Stalled then
                  R.Stalled := E.Start;
                  R.Stalls := R.Stalls + 1;
                  declare
                     Pushes  : constant String := Driver.Robot.Hand.Lowering.Pushes (R.Lowering)'Image;
                     Said_By : Unbounded_String;
                  begin
                     if Said.Joint_Stalled then
                        Append (Said_By, " its readings stopped short of the push's target by "
                                & Driver.Log.Image (100.0 * Said.Joint_Share, 1) & " % of its length, where the" & Pushes
                                & " pushes of this descent before it stopped short by at most "
                                & Driver.Log.Image (100.0 * Said.Joint_Free, 3) & " %;");
                     end if;
                     if Said.Point_Stalled then
                        Append (Said_By, " asked to take " & (if Said.Point = 1 then "the tool's origin" else "the tip of lobe"
                                                              & Whose (Said.Point)'Image)
                                & " down" & Driver.Log.Image (Said.Asked, 4) & ", it went down" & Driver.Log.Image (Said.Went, 4)
                                & ", short by " & Driver.Log.Image (100.0 * Said.Share, 1) & " % of the ask, where the pushes"
                                & " before it fell short by at most " & Driver.Log.Image (100.0 * Said.Free, 3) & " %;");
                     end if;
                     Driver.Log.Line
                       (Driver.Log.Robot,
                        "hand" & Id'Image & ": the arm followed the push that began at beat" & E.Start'Image
                        & " and the hand did not go down with it:" & To_String (Said_By) & " it has stopped lowering the hand");
                  end;
               end if;
            end;
         end;
      end;
   end Judge_Push;

   procedure Watch
     (R          : in out Hand_Record;
      Id         : Hand_Id;
      M          : Model;
      O          : Observation;
      Is_Blocked : Boolean;
      Is_Still   : Boolean;
      Is_Pushing : Boolean);
   --  Follows the hand's arm for presses and keeps every press made at one
   --  of the hand's openings. Presses are found in the arm's own frame, the
   --  one its fit gives the tool pose (Tool_In_Arm) and the table its eye saw
   --  (Table_In_Arm) in: they need neither where the arm stands in the world
   --  nor the arm's unit, and the surface the presses fit is that table
   --  corrected by them. The arm is fitted again as it moves, and its frame
   --  and unit with it: when its table is not the one the presses were fitted
   --  with, they take their poses again from the arm's readings they kept.

   procedure Watch
     (R          : in out Hand_Record;
      Id         : Hand_Id;
      M          : Model;
      O          : Observation;
      Is_Blocked : Boolean;
      Is_Still   : Boolean;
      Is_Pushing : Boolean)
   is
      Found : Boolean;
      Moved : Boolean;
      Press : Driver.Robot.Hand.Presses.Event;
      Which : Opening;

      --  The arm's latest push is one judged stalled.
      function Stalled_Now return Boolean is
        (R.Stalled /= 0 and then Driver.Robot.Steps.Episodes (M, Arm_Group (M, R.Arm)) > 0
         and then Driver.Robot.Steps.Latest (M, Arm_Group (M, R.Arm)).Start = R.Stalled);

      function Pose_Of (Arm : Real_Array) return Pose_Estimate is
         Then_Read : Observation;
      begin
         Then_Read.Readings := O.Readings;
         Then_Read.Readings.Replace_Element (Arm_Group (M, R.Arm), Arm);
         return Tool_In_Arm (M, R.Arm, Then_Read);
      end Pose_Of;
   begin
      Driver.Robot.Hand.Tips.Set_Frame (R.Book, Table_In_Arm (M, R.Arm), Pose_Of'Access, Moved);
      if Moved and then Driver.Robot.Hand.Tips.Pressed (R.Book) > 0 then
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": its arm was fitted again; the"
                          & Driver.Robot.Hand.Tips.Pressed (R.Book)'Image & " presses kept take their poses from the new fit");
         Size_Up (R, Id);
      end if;
      Judge_Push (R, Id, M, O);
      if not Driver.Observations.Has_Reading (O, R.Group) then
         return;
      end if;
      --  A push that stalled is a block as far as the press is concerned: the verdict stands until the next push begins.
      Driver.Robot.Hand.Presses.Observe (R.Watch, O.Beat, Is_Blocked or else Stalled_Now, Is_Pushing, Is_Still,
                                         Tool_In_Arm (M, R.Arm, O),
                                         O.Readings.Element (Arm_Group (M, R.Arm)), O.Readings.Element (R.Group),
                                         Found, Press,
                                         Retargeted => Driver.Robot.Channels.Target_Changed (M, Arm_Group (M, R.Arm), Natural (O.Beat)));
      if not Found then
         return;
      end if;
      if Opening_Of (R, M, Press.Closer.Element, Which) then
         Driver.Robot.Hand.Tips.Add (R.Book, Press, Which, Measured_Slides (R, Id, O, Which));
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": a press at the " & Opening'Image (Which)
                          & " opening, at beat" & Press.Beat'Image & "," & Driver.Robot.Hand.Tips.Pressed (R.Book)'Image
                          & " kept; "
                          & (if Driver.Robot.Hand.Tips.Latest_Agrees (R.Book)
                             then "a tip rests on it"
                             else "no tip rests on it (it stopped short of the table, or no tip is fixed)")
                          & "; the tips at this opening: " & Tips_Said (R, Which));
         Size_Up (R, Id);
      else
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image
                          & ": a press with the closer at neither measured opening is not used");
      end if;
   end Watch;

   procedure Press_Beat
     (H          : in out Hands;
      Id         : Hand_Id;
      M          : Model;
      O          : Observation;
      Is_Blocked : Boolean;
      Is_Still   : Boolean;
      Is_Pushing : Boolean := False)
   is
      R : Hand_Record := H.Data.Found (Id);
   begin
      Watch (R, Id, M, O, Is_Blocked, Is_Still, Is_Pushing);
      H.Data.Found.Replace_Element (Id, R);
   end Press_Beat;

   function Arm_At_Rest (M : Model; A : Arm_Id; O : Observation) return Boolean is
     (Natural (A) <= Arm_Count (M)
      and then Driver.Robot.Stillness.Group_Still (M, Arm_Group (M, A), Natural (O.Beat)));
   --  The arm's own readings are at rest at the beat. The body's eyes are not
   --  asked: the pose of a press is a function of the readings alone, and the
   --  pictures settle after the arm does.

   function Longest_Push (M : Model; A : Arm_Id) return Natural is
      Longest : Natural := 0;
   begin
      if Natural (A) <= Arm_Count (M) then
         for E of M.Groups (Arm_Group (M, A)).Episodes loop
            if E.Ended and then E.End_At >= E.Start then
               Longest := Natural'Max (Longest, E.End_At - E.Start + 1);
            end if;
         end loop;
      end if;
      return Longest;
   end Longest_Push;

   function Asks_A_Translation (M : Model; A : Arm_Id; From, Target : Real_Array) return Boolean is
      Arm_Is : constant Group_Id := Arm_Group (M, A);

      function Pose_At (Arm : Real_Array) return Pose_Estimate is
         O : Observation;
      begin
         for G in Group_Id'First .. Arm_Is loop
            O.Readings.Append ((if G = Arm_Is then Arm else Real_Array'(1 .. 0 => 0.0)));
         end loop;
         return Tool_In_Arm (M, A, O);
      end Pose_At;

      Began : constant Pose_Estimate := Pose_At (From);
      Goal  : constant Pose_Estimate := Pose_At (Target);
   begin
      --  Nothing is told of a push whose poses are not known: it is left to the watcher, as every push was.
      return Began.Position_Covariance (1, 1) = Real'Last or else Goal.Position_Covariance (1, 1) = Real'Last
        or else Driver.Numerics.Angle (Transpose (Began.Pose.Rotation) * Goal.Pose.Rotation)
                <= Driver.Conventions.Z * Driver.Robot.Kinematics.Angle_Sigma (M, A);
   end Asks_A_Translation;

   function Latest_Push_Lowers (M : Model; A : Arm_Id) return Boolean is
      Arm_Is : constant Group_Id := Arm_Group (M, A);
   begin
      if Driver.Robot.Steps.Episodes (M, Arm_Is) = 0 then
         return True;
      end if;
      declare
         E : constant Episode := Driver.Robot.Steps.Latest (M, Arm_Is);
      begin
         if E.Start < 1 or else not Driver.Robot.Channels.Has_Reading (M, Arm_Is, E.Start - 1)
           or else not Driver.Robot.Channels.Has_Target (M, Arm_Is, E.Start)
         then
            return True;
         end if;
         declare
            Size : constant Natural := Group_Size (M, Arm_Is);
         begin
            return Asks_A_Translation
              (M, A,
               From   => [for C in 1 .. Size => Driver.Robot.Channels.Reading (M, Arm_Is, E.Start - 1, C)],
               Target => [for C in 1 .. Size => Driver.Robot.Channels.Target (M, Arm_Is, E.Start, C)]);
         end;
      end;
   end Latest_Push_Lowers;
   --  The arm's latest push, under way or ended, asked the tool a translation. A block is a press only when the push
   --  that was blocked lowered the hand: A27's first press of hand 2 was an aim the arm itself stopped (its third
   --  joint at +0.07, asked -0.05, the eye 12.39 above the table), the stream's watcher took the rest after it for a
   --  press, and a tip 12.39 from the eye was fitted to it, then confirmed by two presses stopped in the air as well.

   function Arm_Pushed (M : Model; A : Arm_Id) return Boolean is
     (Natural (A) <= Arm_Count (M)
      and then Driver.Robot.Steps.Episodes (M, Arm_Group (M, A)) > 0
      and then not Driver.Robot.Steps.Latest (M, Arm_Group (M, A)).Ended);
   --  The arm's latest push has begun and not ended.

   procedure Observe (H : in out Hands; M : Model; O : Observation; Sent : Driver.Commands.Command) is
      pragma Unreferenced (Sent);
   begin
      if H.Data = null then
         H.Data := new Hand_Data;
      end if;
      Find_Pairs (H.Data.all, M, O);
      for I in H.Data.Pairs.First_Index .. H.Data.Pairs.Last_Index loop
         declare
            P : Pair := H.Data.Pairs (I);
         begin
            Feed (P, M, O);
            Report (P, H.Data.all, M);
            H.Data.Pairs.Replace_Element (I, P);
         end;
      end loop;
      for Id in H.Data.Found.First_Index .. H.Data.Found.Last_Index loop
         Press_Beat (H, Id, M, O,
                     Is_Blocked => Blocked (M, H.Data.Found (Id).Arm, O)
                                   and then Latest_Push_Lowers (M, H.Data.Found (Id).Arm),
                     Is_Still   => Arm_At_Rest (M, H.Data.Found (Id).Arm, O),
                     Is_Pushing => Arm_Pushed (M, H.Data.Found (Id).Arm));
      end loop;
   end Observe;

   procedure Free_Closer
     (Arrived    : not null access function return Boolean;
      Reading    : not null access function return Real;
      Moved      : not null access function (Before, After : Real) return Boolean;
      Ask        : not null access procedure;
      Raise_Hand : not null access procedure (First : Boolean; Raised : out Boolean);
      Raises     : out Natural)
   is
      Before : Real;
      Done   : Boolean;
   begin
      Raises := 0;
      Ask.all;
      while not Arrived.all loop
         Before := Reading.all;
         Raise_Hand (Raises = 0, Done);
         exit when not Done;
         Raises := Raises + 1;
         Ask.all;
         exit when not Arrived.all and then not Moved (Before, Reading.all);
      end loop;
   end Free_Closer;

   procedure Gather_Poses
     (Wanted       : Positive;
      Wait_At_Most : Positive;
      Poses        : not null access function return Natural;
      Raise_Eye    : not null access procedure (First : Boolean; Raised : out Boolean);
      Raises       : out Natural;
      Reached      : out Boolean)
   is
      Have   : Natural;
      Raised : Boolean;
   begin
      Raises := 0;
      loop
         Have := Poses.all;
         exit when Have >= Wanted;
         Raise_Eye (Raises = 0, Raised);
         exit when not Raised;
         Raises := Raises + 1;
         --  The new pose's frame is kept once the eye's picture has rested from the move: asked a beat later each time.
         declare
            Asked : Natural := 0;
         begin
            while Asked < Wait_At_Most and then Poses.all <= Have loop
               Asked := Asked + 1;
            end loop;
         end;
      end loop;
      Reached := Poses.all >= Wanted;
   end Gather_Poses;

   procedure Sweep_Way
     (Way          : Real;
      Step         : Real;
      Seen_By      : Real;
      Wait_At_Most : Positive;
      Push         : not null access procedure (Offset : Real; Followed : out Boolean);
      Shows        : not null access function return Showing;
      Pushes       : out Natural;
      Unseen       : out Natural;
      Longest_Wait : out Natural;
      Formed       : out Boolean;
      Answered     : out Boolean)
   is
      Offset   : Real := Step;
      Followed : Boolean;
      Shown    : Boolean := False;   --  some push of this way has shown the views something
      Seen     : Showing;
   begin
      Pushes := 0;
      Unseen := 0;
      Longest_Wait := 0;
      Formed := True;
      Answered := False;
      loop
         Push (Way * Offset, Followed);
         Pushes := Pushes + 1;
         if Pushes = 1 then
            Answered := Followed;
         end if;
         exit when not Followed;
         --  Each asking is a beat later; the view forms once the eye rests,
         --  within as long as the caller measured that to take.
         declare
            Asked : Natural := 0;
         begin
            loop
               Seen := Shows.all;
               Asked := Asked + 1;
               exit when Seen /= Not_Yet or else Asked >= Wait_At_Most;
            end loop;
            Longest_Wait := Natural'Max (Longest_Wait, Asked);
         end;
         if Seen = Not_Yet then
            Formed := False;
            exit;
         end if;
         if Seen = Something_New then
            Shown := True;
         else
            Unseen := Unseen + Boolean'Pos (not Shown);
            --  Nothing new: past the end once something was shown, stuck
            --  once a push that moves the view by a pixel has not.
            exit when Shown or else Offset >= Seen_By;
         end if;
         Offset := 2.0 * Offset;
      end loop;
   end Sweep_Way;

   function In_Free_Air (Stopped : Heights) return Boolean is
     (Known (Stopped.Tip) and then Stopped.Tip.Value > Driver.Conventions.Z * Stopped.Tip.Sigma);

   procedure Descend
     (Above : not null access function return Heights;
      Least : Real;
      Lower : not null access procedure (By : Real; Reached : out Boolean);
      Steps : out Descent_Steps)
   is
      Fast    : Real := Least;   --  the next step of the fast part, doubling
      Past    : Real := 0.0;     --  the last step past the band, doubling
      Reached : Boolean;
      Allowed : Natural := Natural'Last;   --  the steps the schedule takes to cover the eye's room: no more are made
      First   : Boolean := True;

      --  How many steps doubling from Step takes to cover Over.
      function Doublings (Over, Step : Real) return Natural is
        (Natural (Real'Ceiling (Ada.Numerics.Long_Elementary_Functions.Log (Over / Step + 1.0, 2.0))));
   begin
      Steps := (others => <>);
      loop
         declare
            type Kind is (Doubling, Banded, Blind);
            Seen : constant Heights := Above.all;
            G    : Estimate renames Seen.Tip;
            By   : Real := Least;
            How  : Kind := Blind;
         begin
            --  The arm followed the step before and the hand did not go down with it: the hand lies on what it met,
            --  and the descent is the press, a bound or a contact, not spent.
            if Seen.Stalled then
               Steps.Stalled := True;
               exit;
            end if;
            if First then
               First := False;
               if Known (Seen.Eye) then
                  declare
                     Room : constant Real :=
                       Seen.Eye.Value - Threshold (Scalar_Gate (Seen.Eye.Degrees_Of_Freedom)) * Seen.Eye.Sigma;
                  begin
                     if Room >= Least then
                        --  Doubling from Least covers the room, the last step cut to it and the one
                        --  that finds none left; a prediction adds its band and the doubling past it.
                        Allowed := Doublings (Room, Least) + 2;
                        if Known (G) then
                           Allowed := Allowed + 2 * Natural (Real'Ceiling (Threshold (Scalar_Gate (G.Degrees_Of_Freedom)))) + 1
                             + Doublings (Room, Real'Max (G.Sigma, Least));
                        end if;
                     end if;
                  end;
               end if;
            end if;
            --  Steps that were reached and went on without the tool meeting anything are not lowering it:
            --  they end, on the schedule, not on the clock (A17: 6000 beats of steps the hand slid under).
            if Total (Steps) >= Allowed then
               Steps.Spent := True;
               exit;
            end if;
            if Known (G) then
               declare
                  Spread : constant Real := Threshold (Scalar_Gate (G.Degrees_Of_Freedom)) * G.Sigma;
                  --  How far the tip may go before it is within Z sigma of the
                  --  predicted contact.
                  Room : constant Real := G.Value - Spread;
               begin
                  if Room >= Least then
                     --  The prediction says where the contact is: the first push goes to Z sigma above it, in
                     --  one (the steps used to double from Least over it, thirteen pushes of ten beats to cover
                     --  A22's 4.5 units, and a stop of the arm's own in the air showed at the sixth of them).
                     By := Room;
                     How := Doubling;
                  elsif G.Value + Spread > 0.0 then
                     By := Real'Max (G.Sigma, Least);
                     How := Banded;
                  else
                     --  The band is behind and nothing was met: the contact is not
                     --  where it was predicted, as when the press that fixed the tip
                     --  was stopped above the surface by something under it, and the
                     --  prediction is only a bound. Steps of the band's size would
                     --  creep to the table (A17: 6000 beats, then the hand slid
                     --  along it, each small push reached): they double again.
                     if Past = 0.0 then
                        Driver.Log.Line
                          (Driver.Log.Robot,
                           "hand: the tip has gone past the contact its presses predict and the band of its sigma"
                           & " about it, and met nothing (it is predicted" & Driver.Log.Image (G.Value, 4)
                           & " above it now): the steps double again");
                     end if;
                     Past := Real'Max (2.0 * Past, Real'Max (G.Sigma, Least));
                     By := Past;
                  end if;
               end;
            else
               --  Nothing predicts the contact: doubling until a step is
               --  not reached, the overshoot as it comes.
               By := Fast;
               Fast := 2.0 * Fast;
            end if;
            --  The eye stays above the surface, whatever the tip does.
            if Known (Seen.Eye) then
               declare
                  Room : constant Real :=
                    Seen.Eye.Value - Threshold (Scalar_Gate (Seen.Eye.Degrees_Of_Freedom)) * Seen.Eye.Sigma;
               begin
                  if Room < Least then
                     Steps.Spent := True;
                     exit;
                  elsif By > Room then
                     By := Room;
                     Steps.Capped := Steps.Capped + 1;
                  end if;
               end;
            end if;
            case How is
               when Doubling => Steps.Fast := Steps.Fast + 1;
               when Banded   => Steps.Band := Steps.Band + 1;
               when Blind    => Steps.Blind := Steps.Blind + 1;
            end case;
            Lower (By, Reached);
            exit when not Reached;
         end;
      end loop;
   end Descend;

   function Sweepable (H : Hands; M : Model; G : Group_Id) return Boolean is
     (H.Data /= null
      and then (for some P of H.Data.Pairs => P.Group = G and then Still_A_Pair (P, M)));

   procedure Measure (H : in out Hands; M : in out Model) is separate;
   --  The decider (driver-robot-hand-measure.adb).

   function Found (H : Hands; Id : Hand_Id) return Hand_Record is (H.Data.Found (Id));

   function Hand_Count (H : Hands) return Natural is (if H.Data = null then 0 else Natural (H.Data.Found.Length));

   function Exists (H : Hands; Id : Hand_Id) return Boolean is (H.Data /= null and then Id <= H.Data.Found.Last_Index);
   function Closer_Group (H : Hands; Id : Hand_Id) return Group_Id is (Found (H, Id).Group);
   function Arm_Of (H : Hands; Id : Hand_Id) return Arm_Id is (Found (H, Id).Arm);
   function Lobe_Count (H : Hands; Id : Hand_Id) return Positive is (Natural (Found (H, Id).Lobes.Length));
   function Presses_Kept (H : Hands; Id : Hand_Id) return Natural is (Driver.Robot.Hand.Tips.Pressed (Found (H, Id).Book));
   function Own_Eye (H : Hands; Id : Hand_Id) return Eye_Id is (Found (H, Id).Eye);

   function Tip_Pixel (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Driver.Images.Pixel is
     (Found (H, Id).Lobes (Lobe).Sights (At_Opening).Pixel);

   function Tip_Sight (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Ray_Estimate is
     (Found (H, Id).Lobes (Lobe).Sights (At_Opening).Ray);

   function Closer_Reading (H : Hands; Id : Hand_Id; At_Opening : Opening) return Real_Array is
     (Found (H, Id).Readings (At_Opening).Element);

   function Lobe_Width (H : Hands; Id : Hand_Id; Lobe : Positive) return Estimate is
     (Found (H, Id).Lobes (Lobe).Size.Width);

   function Lobe_Thickness (H : Hands; Id : Hand_Id; Lobe : Positive) return Estimate is
     (Found (H, Id).Lobes (Lobe).Size.Thickness);

   function Lobe_Face (H : Hands; Id : Hand_Id; Lobe : Positive) return Estimate is
     (Found (H, Id).Lobes (Lobe).Size.Face);

   function Grip_Depth (H : Hands; Id : Hand_Id) return Estimate is (Found (H, Id).Depth);

   function Grip_Axis (H : Hands; Id : Hand_Id) return Direction_Estimate is (Found (H, Id).Axis);

   --  Where along its line of sight a tip is comes only from presses, and
   --  none is measured yet: these report an unknown estimate.

   function Tip_In_Tool
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded)
     return Point_Estimate
   is
   begin
      return Driver.Robot.Hand.Tips.Tip (Found (H, Id).Book, Lobe, At_Opening, Kind);
   end Tip_In_Tool;

   function Tip_Beat
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Driver.Clock.Beat is
     (Driver.Robot.Hand.Tips.Beat (Found (H, Id).Book, Lobe, At_Opening, Kind));

   function Slides (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Slide_Readings is
      Kept : constant Driver.Robot.Hand.Tips.Press_Slides :=
        Driver.Robot.Hand.Tips.Slides_Of (Found (H, Id).Book, Lobe, At_Opening);
   begin
      return [for K of Kept => (Beat           => K.Beat,
                                Contact        => K.Contact,
                                Tip_Rests      => K.Agrees,
                                Known          => K.Slid.Known,
                                Pixels         => K.Slid.Pixels,
                                Pixels_Sigma   => K.Slid.Pixels_Sigma,
                                Fraction       => K.Slid.Fraction,
                                Fraction_Sigma => K.Slid.Fraction_Sigma)];
   end Slides;

   function Tip_Confirmed
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Boolean is
     (Driver.Robot.Hand.Tips.Confirmed (Found (H, Id).Book, Lobe, At_Opening, Kind));

   function Tip_Tested
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Boolean is
     (Driver.Robot.Hand.Tips.Tested (Found (H, Id).Book, Lobe, At_Opening, Kind));

   function Tip_Across
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Real_Array is
     (Driver.Robot.Hand.Tips.Across (Found (H, Id).Book, Lobe, At_Opening, Kind));

   function Press_Direction (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening)
     return Direction_Estimate is
     (Driver.Robot.Hand.Tips.Direction (Found (H, Id).Book, Lobe, At_Opening));

   function In_World (H : Hands; M : Model; Id : Hand_Id; O : Observation; Tip : Point_Estimate) return Point_Estimate is
     (Driver.Robot.Hand.Frames.Into_World
        (Tool_Pose (M, Arm_Of (H, Id), O), Tool_In_Arm (M, Arm_Of (H, Id), O), Arm_Unit (M, Arm_Of (H, Id)), Tip));
   --  A point of the hand's tool frame, which is in the arm's unit, in the world.

   function Tip (H : Hands; M : Model; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; O : Observation)
     return Point_Estimate is
     (In_World (H, M, Id, O, Tip_In_Tool (H, Id, Lobe, At_Opening)));

   function Tip_Now_In_Tool (H : Hands; Id : Hand_Id; Lobe : Positive; O : Observation) return Point_Estimate;
   --  The tip at the closer reading of O, on the straight path between the
   --  two measured ends in the proportion the lobe's channel is closed.

   function Tip_Now_In_Tool (H : Hands; Id : Hand_Id; Lobe : Positive; O : Observation) return Point_Estimate is
      R       : constant Hand_Record := Found (H, Id);
      C       : constant Positive := R.Lobes (Lobe).Channel;
      Opened  : constant Point_Estimate := Tip_In_Tool (H, Id, Lobe, Open);
      Shut    : constant Point_Estimate := Tip_In_Tool (H, Id, Lobe, Closed_Empty);
      Unknown_Tip : Point_Estimate;
   begin
      if not Driver.Observations.Has_Reading (O, R.Group) or else not Known (Opened) or else not Known (Shut) then
         return Unknown_Tip;
      end if;
      declare
         Now     : constant Real_Array := O.Readings.Element (R.Group);
         From    : constant Real_Array := R.Readings (Open).Element;
         To      : constant Real_Array := R.Readings (Closed_Empty).Element;
         Reading : constant Real := Now (Now'First + C - 1);
         Start   : constant Real := From (From'First + C - 1);
         --  The ends were measured at significantly different readings.
         F       : constant Real := (Reading - Start) / (To (To'First + C - 1) - Start);
      begin
         return (Mean       => Opened.Mean + F * (Shut.Mean - Opened.Mean),
                 Covariance => ((1.0 - F) ** 2) * Opened.Covariance + (F ** 2) * Shut.Covariance);
      end;
   end Tip_Now_In_Tool;

   function Tip_Now (H : Hands; M : Model; Id : Hand_Id; Lobe : Positive; O : Observation) return Point_Estimate is
     (In_World (H, M, Id, O, Tip_Now_In_Tool (H, Id, Lobe, O)));

   function Grip_Centre (H : Hands; M : Model; Id : Hand_Id; O : Observation) return Point_Estimate is
      Sum   : Vec3 := Zero3;
      Total : Mat3 := [others => [others => 0.0]];
      Count : constant Positive := Lobe_Count (H, Id);
   begin
      for L in 1 .. Count loop
         declare
            T : constant Point_Estimate := Tip_Now_In_Tool (H, Id, L, O);
         begin
            if not Known (T) then
               return In_World (H, M, Id, O, T);
            end if;
            Sum := Sum + T.Mean;
            Total := Total + T.Covariance;
         end;
      end loop;
      --  The lobes' tips at this opening, averaged; each tip was fitted from
      --  its own presses.
      return In_World (H, M, Id, O, (Mean => (1.0 / Real (Count)) * Sum, Covariance => (1.0 / Real (Count) ** 2) * Total));
   end Grip_Centre;

   function Describe (H : Hands) return String is
      Text : Unbounded_String;
   begin
      --  Every closer of an arm's own eye that no hand is made of says why not,
      --  channel by channel.
      if H.Data /= null then
         for P of H.Data.Pairs loop
            if not (for some R of H.Data.Found => R.Group = P.Group) then
               Append (Text, "closer group" & P.Group'Image & " on arm" & P.Arm'Image & " in eye" & P.Eye'Image
                       & ", its own: no hand;");
               for C in 1 .. Sweeps.Channels (P.Sweep) loop
                  Append (Text, " channel" & C'Image & ": " & Sweeps.Account (P.Sweep, C) & ";");
               end loop;
               Append (Text, ASCII.LF);
            end if;
         end loop;
      end if;
      for Id in 1 .. Hand_Id'Base (Hand_Count (H)) loop
         declare
            R      : constant Hand_Record := Found (H, Id);
            Known_Tips, Checked_Tips, Sought : Natural := 0;
         begin
            Append (Text, "hand" & Id'Image & ": closer group" & R.Group'Image & " on arm" & R.Arm'Image & ", eye"
                    & R.Eye'Image & "," & R.Lobes.Length'Image & " lobes;");
            for L in R.Lobes.First_Index .. R.Lobes.Last_Index loop
               Append (Text, " channel" & R.Lobes (L).Channel'Image & " tip open " & Image (R.Lobes (L).Sights (Open).Pixel)
                       & " closed " & Image (R.Lobes (L).Sights (Closed_Empty).Pixel) & ";");
               for Which in Opening loop
                  if R.Lobes (L).Sights (Which).Known then
                     Sought := Sought + 1;
                     Known_Tips := Known_Tips + Boolean'Pos (Known (Driver.Robot.Hand.Tips.Tip (R.Book, L, Which)));
                     Checked_Tips := Checked_Tips + Boolean'Pos (Driver.Robot.Hand.Tips.Confirmed (R.Book, L, Which));
                  end if;
               end loop;
            end loop;
            Append (Text, " presses:" & Driver.Robot.Hand.Tips.Pressed (R.Book)'Image & " kept, tips measured:"
                    & Known_Tips'Image & " of" & Sought'Image & " (" & Natural'Image (Checked_Tips)
                    & " confirmed by a second press); sizes: " & Sizes_Text (R) & ASCII.LF);
         end;
      end loop;
      return To_String (Text);
   end Describe;

end Driver.Robot.Hand;
