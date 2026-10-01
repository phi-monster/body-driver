with Ada.Containers.Indefinite_Holders;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Driver.Instrument;
with Driver.Log;
with Driver.Robot.Hand.Frames;
with Driver.Robot.Hand.Sweep;
with Driver.Robot.Hand.Views;
with Driver.Services;

package body Driver.Robot.Hand is

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

   --  A lobe's tip at one opening: its pixel in the own eye and its line of
   --  sight in the tool frame.
   type Sight is record
      Known : Boolean := False;
      Pixel : Driver.Images.Pixel;
      Ray   : Ray_Estimate;
   end record;

   type Sight_Array is array (Opening) of Sight;

   type Lobe_Record is record
      Channel : Positive;
      Sights  : Sight_Array;
   end record;

   package Lobe_Record_Vectors is new Ada.Containers.Vectors (Positive, Lobe_Record);

   type Reading_Array is array (Opening) of Driver.Robot.Hand.Views.Reading_Holders.Holder;

   type Hand_Record is record
      Group    : Group_Id;
      Arm      : Arm_Id;
      Eye      : Eye_Id;
      Readings : Reading_Array;
      Lobes    : Lobe_Record_Vectors.Vector;
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

   function Rest_Noise (M : Model; O : Observation; G : Group_Id) return Real_Array is
      R : Real_Array (1 .. Rest_Of (O, G)'Length);
      K : Natural := 0;
   begin
      for Other in O.Readings.First_Index .. O.Readings.Last_Index loop
         if Other /= G then
            for C in 1 .. O.Readings.Element (Other)'Length loop
               K := K + 1;
               R (K) := Reading_Noise (M, Other, C);
            end loop;
         end if;
      end loop;
      return R;
   end Rest_Noise;

   function All_Read (O : Observation) return Boolean is
     (for all G in O.Readings.First_Index .. O.Readings.Last_Index => Driver.Observations.Has_Reading (O, G));

   function Has_Pair (D : Hand_Data; G : Group_Id; E : Eye_Id) return Boolean is
     (for some P of D.Pairs => P.Group = G and then P.Eye = E);

   procedure Find_Pairs (D : in out Hand_Data; M : Model; O : Observation);
   --  Every closer group with the eyes it moves a patch in, its own arm's
   --  eyes among them, once the body has measured them.

   procedure Find_Pairs (D : in out Hand_Data; M : Model; O : Observation) is
   begin
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
                                                      Channels, Noise, Rest_Noise (M, O, G)),
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
   begin
      Sweeps.Observe (P.Sweep, O, Still => Still (M) and then Complete,
                      Closer => O.Readings (P.Group), Rest => Rest_Of (O, P.Group), Image => O.Images (P.Eye));
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
                                                 Closed_Empty => (if Closed_High then At_High else At_Low)]));
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
      if Made.Lobes.Is_Empty then
         return;
      end if;
      for Id in D.Found.First_Index .. D.Found.Last_Index loop
         if D.Found (Id).Group = P.Group then
            D.Found.Replace_Element (Id, Made);
            return;
         end if;
      end loop;
      D.Found.Append (Made);
   end Rebuild;

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
                  Forward  : Driver.Instrument.Answer_Array (Points'Range);
                  Backward : Driver.Instrument.Answer_Array (Points'Range);
                  Ok_Forward, Ok_Backward : Boolean;
                  Why_Forward, Why_Backward : Unbounded_String;
                  Requests : Request_Array := P.Requests.Element;
               begin
                  Driver.Instrument.Read_Match
                    (Driver.Services.Collect (Asked.Forward), True, Forward, Ok_Forward, Why_Forward);
                  Driver.Instrument.Read_Match
                    (Driver.Services.Collect (Asked.Backward), True, Backward, Ok_Backward, Why_Backward);
                  Requests (C).Out_Now := False;
                  P.Requests := Request_Holders.To_Holder (Requests);
                  if Ok_Forward and then Ok_Backward then
                     Sweeps.Answer (P.Sweep, C, Points, Forward, Backward, Asked.Attached);
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
                     Sweeps.Refuse (P.Sweep, C);
                     Driver.Log.Line (Driver.Log.Robot, "hand: the instrument did not answer for closer group"
                                      & P.Group'Image & " channel" & C'Image & ": "
                                      & To_String (if Ok_Forward then Why_Backward else Why_Forward));
                  end if;
               end;
            end if;
         end;
      end loop;
      if Measured_Now and then P.Own then
         Rebuild (D, P, M);
      end if;
   end Collect;

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
   end Observe;

   procedure Measure (H : in out Hands; M : in out Model) is
      pragma Unreferenced (H, M);
   begin
      Driver.Log.Line (Driver.Log.Robot, "hand: the sweeps and presses that measure the hands are not built yet");
   end Measure;

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

   --  Where along its line of sight a tip is comes only from presses, and
   --  none is measured yet: these report an unknown estimate.

   function Tip_In_Tool (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Point_Estimate is
      pragma Unreferenced (H, Id, Lobe, At_Opening);
      Unmeasured : Point_Estimate;
   begin
      return Unmeasured;
   end Tip_In_Tool;

   function Press_Direction (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening)
     return Direction_Estimate
   is
      pragma Unreferenced (H, Id, Lobe, At_Opening);
      Unmeasured : Direction_Estimate;
   begin
      return Unmeasured;
   end Press_Direction;

   function Tip (H : Hands; M : Model; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; O : Observation)
     return Point_Estimate is
     (Driver.Robot.Hand.Frames.Into (Tool_Pose (M, Arm_Of (H, Id), O), Tip_In_Tool (H, Id, Lobe, At_Opening)));

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
     (Driver.Robot.Hand.Frames.Into (Tool_Pose (M, Arm_Of (H, Id), O), Tip_Now_In_Tool (H, Id, Lobe, O)));

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
               return Driver.Robot.Hand.Frames.Into (Tool_Pose (M, Arm_Of (H, Id), O), T);
            end if;
            Sum := Sum + T.Mean;
            Total := Total + T.Covariance;
         end;
      end loop;
      --  The lobes' tips at this opening, averaged; each tip was fitted from
      --  its own presses.
      return Driver.Robot.Hand.Frames.Into
        (Tool_Pose (M, Arm_Of (H, Id), O),
         (Mean => (1.0 / Real (Count)) * Sum, Covariance => (1.0 / Real (Count) ** 2) * Total));
   end Grip_Centre;

   function Describe (H : Hands) return String is
      Text : Unbounded_String;
   begin
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
            Append (Text, ASCII.LF);
         end;
      end loop;
      return To_String (Text);
   end Describe;

end Driver.Robot.Hand;
