with Ada.Containers.Indefinite_Holders;
with Ada.Containers.Vectors;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Driver.Instrument;
with Driver.Log;
with Driver.Robot.Channels;
with Driver.Robot.Hand.Frames;
with Driver.Robot.Hand.Lobes;
with Driver.Robot.Hand.Presses;
with Driver.Robot.Hand.Shape;
with Driver.Robot.Hand.Sweep;
with Driver.Robot.Hand.Tips;
with Driver.Robot.Hand.Views;
with Driver.Robot.Stillness;
with Driver.Services;

package body Driver.Robot.Hand is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;
   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;
   use type Driver.Robot.Hand.Sweep.Progress;

   package Sweeps renames Driver.Robot.Hand.Sweep;
   package Point_Holders is new Ada.Containers.Indefinite_Holders (Driver.Instrument.Point_Array, Driver.Instrument."=");

   --  The correspondences asked for between one channel's two ends.
   type Request is record
      Out_Now  : Boolean := False;
      Forward  : Driver.Services.Ticket;   --  low end to high end
      Backward : Driver.Services.Ticket;   --  high end to low end
      Points   : Point_Holders.Holder;
      Attached : Driver.Images.Mask;       --  the robot's own pixels when it was asked
   end record;

   type Request_Array is array (Positive range <>) of Request;
   package Request_Holders is new Ada.Containers.Indefinite_Holders (Request_Array);

   --  One closer group as one eye sees it.
   type Pair is record
      Group    : Group_Id;
      Eye      : Eye_Id;
      Arm      : Arm_Id;
      Own      : Boolean := False;   --  the eye rides on the closer's arm
      Sweep    : Sweeps.State;
      Requests : Request_Holders.Holder;
   end record;

   package Pair_Vectors is new Ada.Containers.Vectors (Positive, Pair);

   type Lobe_Record is record
      Channel : Positive;
      Sights  : Sight_Array;
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

   --  The matcher's answers are sized by pixels and live on the heap: the
   --  estimates also run in the decider's task, whose stack is small.
   type Answers_Access is access Driver.Instrument.Answer_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Instrument.Answer_Array, Answers_Access);

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
   --  Every closer group with the eyes it moves a patch in, its own arm's
   --  eyes among them, once the body has measured them; by the body's roles
   --  as they are now, which the boot re-reads: a pair whose group is no
   --  longer a closer, or no longer of that arm, goes, and so does a hand
   --  found from it.

   function Still_A_Pair (P : Pair; M : Model) return Boolean is
     (Role (M, P.Group) = Closer and then Closer_Arm (M, P.Group) = P.Arm
      and then (if P.Own then Eye_Mount (M, P.Eye).Kind = Arm_Carried and then Eye_Mount (M, P.Eye).Arm = P.Arm
                else Response (M, P.Group, P.Eye) = Patch));

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
                  if (Own or else Response (M, G, E) = Patch) and then Driver.Observations.Has_Image (O, E)
                    and then not Has_Pair (D, G, E)
                  then
                     declare
                        Channels : constant Positive := O.Readings.Element (G)'Length;
                        Noise    : Real_Array (1 .. Channels);
                     begin
                        for C in Noise'Range loop
                           Noise (C) := Reading_Noise (M, G, C);
                        end loop;
                        D.Pairs.Append
                          (Pair'(Group    => G,
                            Eye      => E,
                            Arm      => Arm,
                            Own      => Own,
                            Sweep    => Sweeps.Start (Driver.Images.Width (O.Images (E)), Driver.Images.Height (O.Images (E)),
                                                      Channels, Noise),
                            Requests => Request_Holders.To_Holder ([1 .. Channels => (others => <>)])));
                        Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " on arm" & Arm'Image
                                         & " is watched in eye" & E'Image & (if Own then ", its own" else ""));
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

   procedure Ask (P : in out Pair; M : Model; O : Observation);
   --  Asks the instrument for the correspondences between the ends of every
   --  channel whose ends are new, both ways round with round trips.

   procedure Ask (P : in out Pair; M : Model; O : Observation) is
   begin
      for C in 1 .. Sweeps.Channels (P.Sweep) loop
         if Sweeps.Wants_Correspondences (P.Sweep, C) then
            declare
               Points : constant Driver.Instrument.Point_Array := Sweeps.Query_Points (P.Sweep, C);
               Low    : constant Driver.Instrument.Source := (Stored => False, Image => Sweeps.Low_End (P.Sweep, C).Last);
               High   : constant Driver.Instrument.Source := (Stored => False, Image => Sweeps.High_End (P.Sweep, C).Last);
               Requests : Request_Array := P.Requests.Element;
            begin
               Requests (C) := (Out_Now  => True,
                                Forward  => Driver.Instrument.Submit_Match (Low, High, Points, True, O.Beat),
                                Backward => Driver.Instrument.Submit_Match (High, Low, Points, True, O.Beat),
                                Points   => Point_Holders.To_Holder (Points),
                                Attached => Self_Mask (M, P.Eye, O));
               P.Requests := Request_Holders.To_Holder (Requests);
               Sweeps.Asked (P.Sweep, C);
               Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & P.Group'Image & " channel" & C'Image
                                & " seen at both ends in eye" & P.Eye'Image & "; asking where"
                                & Points'Length'Image & " pixels went");
            end;
         end if;
      end loop;
   end Ask;

   function Shape_Of
     (M           : Model;
      Eye         : Eye_Id;
      L           : Driver.Robot.Hand.Lobes.Lobe;
      Open_View   : Driver.Robot.Hand.Views.View;
      Closed_High : Boolean;
      Noise       : Driver.Robot.Hand.Lobes.Matcher_Noise) return Driver.Robot.Hand.Shape.Lobe_Shape;
   --  A lobe's shape from every pixel of it at the open end and where the
   --  matcher put it at the closed end, and from its closed tip and where
   --  that matched back to: their lines of sight taken into the tool frame
   --  through the eye's mount as it was when the open end was seen.

   function Shape_Of
     (M           : Model;
      Eye         : Eye_Id;
      L           : Driver.Robot.Hand.Lobes.Lobe;
      Open_View   : Driver.Robot.Hand.Views.View;
      Closed_High : Boolean;
      Noise       : Driver.Robot.Hand.Lobes.Matcher_Noise) return Driver.Robot.Hand.Shape.Lobe_Shape
   is
      package Shapes renames Driver.Robot.Hand.Shape;
      package Lobes renames Driver.Robot.Hand.Lobes;
      Mount    : constant Pose_Estimate := Eye_In_Tool (M, Eye, Open_View.Seen);
      Opened   : constant Lobes.Move_Holders.Holder := (if Closed_High then L.Moves_Here else L.Moves_There);
      Shut     : constant Lobes.Move_Holders.Holder := (if Closed_High then L.Moves_There else L.Moves_Here);
      Open_Tip : constant Natural := (if Closed_High then L.Tip_Move_Here else L.Tip_Move_There);
      Shut_Tip : constant Natural := (if Closed_High then L.Tip_Move_There else L.Tip_Move_Here);
      Bordered : constant Boolean := (if Closed_High then L.Bordered_Here else L.Bordered_There);

      function Line (Px : Driver.Images.Pixel) return Vec3 is
        (Mount.Pose.Rotation * Eye_Ray (M, Eye, Px).Direction.Unit_Vector);
   begin
      if Mount.Position_Covariance (1, 1) = Real'Last then
         return Shapes.Unfitted ("its eye's mount was not measured when it was swept");
      elsif not Known (Noise.Displacement) then
         return Shapes.Unfitted ("the matcher's own noise between its ends is not known");
      elsif Open_Tip = 0 or else Shut_Tip = 0 or else Opened.Is_Empty or else Shut.Is_Empty then
         return Shapes.Unfitted ("its tip is not seen at both ends");
      elsif Eye_Ray (M, Eye, Opened.Constant_Reference.Element (Open_Tip).From).Direction.Sigma = Real'Last then
         return Shapes.Unfitted ("its eye's lens was not measured when it was swept");
      end if;
      declare
         Moves : Lobes.Move_Array renames Opened.Constant_Reference.Element.all;
         --  Sized by pixels: a function result, off the stack.
         Seen  : constant Shapes.Sighting_Array :=
           Shapes.From_Moves (Moves, Shut.Constant_Reference.Element (Shut_Tip), Line'Access, Noise.Displacement.Sigma);
      begin
         return Shapes.Fit (Mount.Pose.Translation, Seen, Open_Tip - Moves'First + 1, Seen'Last, Bordered);
      end;
   end Shape_Of;

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
   begin
      if N = 0 then
         return;
      end if;
      for L in 1 .. N loop
         Fits (L) := R.Lobes (L).Shape;
         for O in Opening loop
            Tipped (L, O) := Driver.Robot.Hand.Tips.Tip (R.Book, L, O);
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
            Table (L) (Which) := (Known => Made.Lobes (L).Sights (Which).Known,
                                  Ray   => Made.Lobes (L).Sights (Which).Ray);
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

   procedure Rebuild (D : in out Hand_Data; P : Pair; M : Model);
   --  The hand of a closer group from what its own eye measured: every
   --  measured channel's lobes with their tips at both openings.

   procedure Rebuild (D : in out Hand_Data; P : Pair; M : Model) is
      Made : Hand_Record := (Group => P.Group, Arm => P.Arm, Eye => P.Eye, others => <>);

      function Seen_Tip (V : Driver.Robot.Hand.Views.View; Known : Boolean; Px : Driver.Images.Pixel) return Sight is
        (if Known then (Known => True, Pixel => Px, Ray => Driver.Robot.Hand.Frames.Into (Eye_In_Tool (M, P.Eye, V.Seen), Eye_Ray (M, P.Eye, Px)))
         else (Known => False, Pixel => Px, Ray => <>));
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
                     At_Low  : constant Sight := Seen_Tip (Low, L.Tip_Known_Here, L.Tip_Here);
                     At_High : constant Sight := Seen_Tip (High, L.Tip_Known_There, L.Tip_There);
                  begin
                     Made.Lobes.Append
                       (Lobe_Record'(Channel => C,
                                     Sights  => [Open         => (if Closed_High then At_Low else At_High),
                                                 Closed_Empty => (if Closed_High then At_High else At_Low)],
                                     Shape   => Shape_Of (M, P.Eye, L, (if Closed_High then Low else High), Closed_High,
                                                          Sweeps.Noise_Of (P.Sweep, C)),
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
                         Shape   => Driver.Robot.Hand.Shape.Unfitted ("the hand was given, not swept"),
                         Size    => <>));
      end loop;
      if not Made.Lobes.Is_Empty then
         Keep (H.Data.all, Made);
      end if;
   end Adopt;

   procedure Collect (P : in out Pair; D : in out Hand_Data; M : Model);
   --  Reads the replies that are in and finds the lobes from them.

   procedure Collect (P : in out Pair; D : in out Hand_Data; M : Model) is
      Measured_Now : Boolean := False;
   begin
      for C in 1 .. Sweeps.Channels (P.Sweep) loop
         declare
            Asked : constant Request := P.Requests.Element (C);
         begin
            if Asked.Out_Now and then Driver.Services.Ready (Asked.Forward) and then Driver.Services.Ready (Asked.Backward)
            then
               declare
                  Points   : constant Driver.Instrument.Point_Array := Asked.Points.Element;
                  Forward  : Answers_Access := new Driver.Instrument.Answer_Array (Points'Range);
                  Backward : Answers_Access := new Driver.Instrument.Answer_Array (Points'Range);
                  Ok_Forward, Ok_Backward : Boolean;
                  Why_Forward, Why_Backward : Unbounded_String;
                  Requests : Request_Array := P.Requests.Element;
                  Ahead    : constant Driver.Services.Reply := Driver.Services.Collect (Asked.Forward);
                  Behind   : constant Driver.Services.Reply := Driver.Services.Collect (Asked.Backward);
               begin
                  Driver.Instrument.Read_Match (Ahead, True, Forward.all, Ok_Forward, Why_Forward);
                  Driver.Instrument.Read_Match (Behind, True, Backward.all, Ok_Backward, Why_Backward);
                  Requests (C).Out_Now := False;
                  P.Requests := Request_Holders.To_Holder (Requests);
                  if Ok_Forward and then Ok_Backward then
                     Sweeps.Answer (P.Sweep, C, Points, Forward.all, Backward.all, Asked.Attached);
                     Driver.Log.Line
                       (Driver.Log.Robot, "hand: closer group" & P.Group'Image & " channel" & C'Image & " in eye"
                        & P.Eye'Image & ": "
                        & (if Sweeps.Status (P.Sweep, C) = Sweeps.Measured
                           then Natural (Sweeps.Lobes_Of (P.Sweep, C).Length)'Image & " lobes, "
                                & (if not Sweeps.Closing_Known (P.Sweep, C) then "closing direction not significant"
                                   elsif Sweeps.Closed_End_Is_High (P.Sweep, C) then "closed at the high reading"
                                   else "closed at the low reading")
                           else "nothing moves between its ends"));
                     Measured_Now := Measured_Now or else Sweeps.Status (P.Sweep, C) = Sweeps.Measured;
                  else
                     declare
                        Why     : constant String := To_String (if Ok_Forward then Why_Backward else Why_Forward);
                        Lasting : constant Boolean := Ahead.Lasting or else Behind.Lasting;
                     begin
                        Sweeps.Refuse (P.Sweep, C, Lasting, Why);
                        Driver.Log.Line (Driver.Log.Robot, "hand: the instrument did not answer for closer group"
                                         & P.Group'Image & " channel" & C'Image & ": " & Why
                                         & (if Lasting then "; it never can, so nothing more is asked" else ""));
                     end;
                  end if;
                  Free (Forward);
                  Free (Backward);
               end;
            end if;
         end;
      end loop;
      if Measured_Now and then P.Own then
         Rebuild (D, P, M);
      end if;
   end Collect;

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

   procedure Watch
     (R          : in out Hand_Record;
      Id         : Hand_Id;
      M          : Model;
      O          : Observation;
      Is_Blocked : Boolean;
      Is_Still   : Boolean);
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
      Is_Still   : Boolean)
   is
      Found : Boolean;
      Moved : Boolean;
      Press : Driver.Robot.Hand.Presses.Event;
      Which : Opening;

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
      if not Driver.Observations.Has_Reading (O, R.Group) then
         return;
      end if;
      Driver.Robot.Hand.Presses.Observe (R.Watch, O.Beat, Is_Blocked, Is_Still, Tool_In_Arm (M, R.Arm, O),
                                         O.Readings.Element (Arm_Group (M, R.Arm)), O.Readings.Element (R.Group),
                                         Found, Press);
      if not Found then
         return;
      end if;
      if Opening_Of (R, M, Press.Closer.Element, Which) then
         Driver.Robot.Hand.Tips.Add (R.Book, Press, Which);
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": a press at the " & Opening'Image (Which)
                          & " opening, " & Driver.Robot.Hand.Tips.Pressed (R.Book)'Image & " kept; it "
                          & (if Driver.Robot.Hand.Tips.Latest_Agrees (R.Book) then "agrees" else "does not agree")
                          & " with the others");
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
      Is_Still   : Boolean)
   is
      R : Hand_Record := H.Data.Found (Id);
   begin
      Watch (R, Id, M, O, Is_Blocked, Is_Still);
      H.Data.Found.Replace_Element (Id, R);
   end Press_Beat;

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
            Ask (P, M, O);
            Collect (P, H.Data.all, M);
            H.Data.Pairs.Replace_Element (I, P);
         end;
      end loop;
      for Id in H.Data.Found.First_Index .. H.Data.Found.Last_Index loop
         Press_Beat (H, Id, M, O, Blocked (M, H.Data.Found (Id).Arm, O), Still (M));
      end loop;
   end Observe;

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

   procedure Descend
     (Gap   : not null access function return Estimate;
      Least : Real;
      Lower : not null access procedure (By : Real; Reached : out Boolean);
      Steps : out Descent_Steps)
   is
      Fast    : Real := Least;   --  the next step of the fast part, doubling
      Reached : Boolean;
   begin
      Steps := (others => 0);
      loop
         declare
            G  : constant Estimate := Gap.all;
            By : Real := Least;
         begin
            if Known (G) then
               declare
                  --  How far the tip may go before it is within Z sigma of the
                  --  predicted contact.
                  Room : constant Real := G.Value - Threshold (Scalar_Gate (G.Degrees_Of_Freedom)) * G.Sigma;
               begin
                  if Room >= Least then
                     By := Real'Min (Fast, Room);
                     Fast := 2.0 * Fast;
                     Steps.Fast := Steps.Fast + 1;
                  else
                     By := Real'Max (G.Sigma, Least);
                     Steps.Band := Steps.Band + 1;
                  end if;
               end;
            else
               --  Nothing predicts the contact: doubling until a step is
               --  not reached, the overshoot as it comes.
               By := Fast;
               Fast := 2.0 * Fast;
               Steps.Blind := Steps.Blind + 1;
            end if;
            Lower (By, Reached);
            exit when not Reached;
         end;
      end loop;
   end Descend;

   function Sweepable (H : Hands; M : Model; G : Group_Id) return Boolean is
     (H.Data /= null
      and then (for some P of H.Data.Pairs => P.Group = G and then P.Own and then Still_A_Pair (P, M)));

   procedure Measure (H : in out Hands; M : in out Model) is separate;
   --  The decider (driver-robot-hand-measure.adb).

   function Found (H : Hands; Id : Hand_Id) return Hand_Record is (H.Data.Found (Id));

   function Hand_Count (H : Hands) return Natural is (if H.Data = null then 0 else Natural (H.Data.Found.Length));
   function Closer_Group (H : Hands; Id : Hand_Id) return Group_Id is (Found (H, Id).Group);
   function Arm_Of (H : Hands; Id : Hand_Id) return Arm_Id is (Found (H, Id).Arm);
   function Lobe_Count (H : Hands; Id : Hand_Id) return Positive is (Natural (Found (H, Id).Lobes.Length));
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

   function Tip_In_Tool (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Point_Estimate is
     (Driver.Robot.Hand.Tips.Tip (Found (H, Id).Book, Lobe, At_Opening));

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
      --  A closer the instrument can never answer for is not measured, and says why.
      if H.Data /= null then
         for P of H.Data.Pairs loop
            if Sweeps.Refusal (P.Sweep) /= "" then
               Append (Text, "closer group" & P.Group'Image & " in eye" & P.Eye'Image & ": not measured, the instrument"
                       & " can never answer (" & Sweeps.Refusal (P.Sweep) & ")" & ASCII.LF);
            end if;
         end loop;
      end if;
      for Id in 1 .. Hand_Id'Base (Hand_Count (H)) loop
         declare
            R : constant Hand_Record := Found (H, Id);
         begin
            Append (Text, "hand" & Id'Image & ": closer group" & R.Group'Image & " on arm" & R.Arm'Image & ", eye"
                    & R.Eye'Image & "," & R.Lobes.Length'Image & " lobes;");
            for L of R.Lobes loop
               Append (Text, " channel" & L.Channel'Image & " tip open " & Image (L.Sights (Open).Pixel)
                       & " closed " & Image (L.Sights (Closed_Empty).Pixel) & ";");
            end loop;
            Append (Text, " sizes: " & Sizes_Text (R) & ASCII.LF);
         end;
      end loop;
      return To_String (Text);
   end Describe;

end Driver.Robot.Hand;
