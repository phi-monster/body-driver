with Driver.Log;
with Driver.Pixels;

package body Driver.Robot.Hand.Sweep is

   use Driver.Images;
   use type Driver.Clock.Beat;
   use type Driver.Robot.Hand.Lobes.Closing;
   use type Driver.Robot.Hand.Lobes.Placing;

   function Start (Width, Height : Positive; Channels : Positive; Closer_Noise : Real_Array) return State is
     ((Width       => Width,
       Height      => Height,
       Views       => Driver.Robot.Hand.Views.Start (Width, Height, Closer_Noise),
       Memory      => Driver.Robot.Hand.Selfsight.Start (Width, Height, Closer_Noise),
       Per_Channel => Channel_Holders.To_Holder ([1 .. Channels => (others => <>)])));

   function Channels (S : State) return Positive is (S.Per_Channel.Element'Length);

   procedure Set_Closer_Noise (S : in out State; Closer_Noise : Real_Array) is
   begin
      Driver.Robot.Hand.Views.Set_Closer_Noise (S.Views, Closer_Noise);
      Driver.Robot.Hand.Selfsight.Set_Key_Noise (S.Memory, Closer_Noise);
   end Set_Closer_Noise;

   function Status (S : State; Channel : Positive) return Progress is
     (S.Per_Channel.Constant_Reference.Element (Channel).Status);

   function Has_Ends (S : State; Channel : Positive) return Boolean is
     (Driver.Robot.Hand.Views.Has_Ends (S.Views, Channel));

   function Poses_Doubled (S : State; Channel : Positive) return Boolean is
      Was  : constant Channel_Holders.Constant_Reference_Type := S.Per_Channel.Constant_Reference;
      Low  : constant Natural :=
        Driver.Robot.Hand.Selfsight.Poses (S.Memory, Driver.Robot.Hand.Views.Low_Closer (S.Views, Channel));
      High : constant Natural :=
        Driver.Robot.Hand.Selfsight.Poses (S.Memory, Driver.Robot.Hand.Views.High_Closer (S.Views, Channel));
   begin
      return (Low >= 2 and then Low >= 2 * Was.Element (Channel).Poses_Low)
        or else (High >= 2 and then High >= 2 * Was.Element (Channel).Poses_High);
   end Poses_Doubled;
   --  The arm has moved the eye, at the readings of an end, through twice the
   --  poses the last placing had (two at the least): a deviation over more
   --  poses is a better one.

   function Ends_Moved (S : State; Channel : Positive) return Boolean is
      Now : constant Channel_Holders.Constant_Reference_Type := S.Per_Channel.Constant_Reference;
   begin
      return Driver.Robot.Hand.Views.Has_Ends (S.Views, Channel)
        and then (not Now.Element (Channel).Analysed
                  or else Driver.Robot.Hand.Views.Low_Beat (S.Views, Channel) /= Now.Element (Channel).Low_From
                  or else Driver.Robot.Hand.Views.High_Beat (S.Views, Channel) /= Now.Element (Channel).High_From
                  or else ((Now.Element (Channel).Status = Unlocated or else Now.Element (Channel).Status = Unplaced)
                           and then Poses_Doubled (S, Channel)));
   end Ends_Moved;

   procedure Renew
     (S       : in out State;
      Channel : Positive;
      Motion  : access procedure (Channel : Positive; Column, Row : Natural; U, V : out Real));
   --  New ends: what changed between them decides whether anything is to be
   --  placed; a measurement of older ends no longer stands.

   procedure Renew
     (S       : in out State;
      Channel : Positive;
      Motion  : access procedure (Channel : Positive; Column, Row : Natural; U, V : out Real))
   is
      --  How the picture moves per unit of this channel's reading, from the low end's reading towards the high one's.
      procedure Channel_Motion (Column, Row : Natural; U, V : out Real) is
      begin
         Motion (Channel, Column, Row, U, V);
      end Channel_Motion;
      Low      : constant Driver.Robot.Hand.Views.View := Driver.Robot.Hand.Views.Low_End (S.Views, Channel);
      High     : constant Driver.Robot.Hand.Views.View := Driver.Robot.Hand.Views.High_End (S.Views, Channel);
      Compared : constant Driver.Pixels.Comparison := Driver.Pixels.Compare (Low.Frames, High.Frames);
      Per_Channel : Channel_Array := S.Per_Channel.Element;
      Now      : Channel_State;
      --  The lobes' parts are attached to the picture's border alone; the robot's own pixels that did not change
      --  are found by the placing (Located.Still), and a single lobe closes against them.
      Nothing  : constant Mask := Create (0, 0);
   begin
      Now.Low_From := Low.From;
      Now.High_From := High.From;
      Now.Analysed := True;
      Now.Announced := False;
      Now.Poses_Low := Driver.Robot.Hand.Selfsight.Poses (S.Memory, Low.Closer.Element);
      Now.Poses_High := Driver.Robot.Hand.Selfsight.Poses (S.Memory, High.Closer.Element);
      Now.Low_At := Driver.Robot.Hand.Views.Reading (Low, Channel);
      Now.High_At := Driver.Robot.Hand.Views.Reading (High, Channel);
      Now.Changed := Count (Compared.Changed);
      Now.Spread := Compared.Spread;
      Now.Beyond := Compared.Beyond;
      if not Compared.Trusted then
         Now.Status := Everything_Moves;
      elsif Now.Changed = 0 then
         Now.Status := Nothing_Moves;
      elsif Now.Poses_Low < 2 and then Now.Poses_High < 2 then
         Now.Status := Unlocated;
      else
         declare
            At_Low : constant Boolean := Now.Poses_Low >= Now.Poses_High;
            Anchor : constant Driver.Pixels.View :=
              Driver.Robot.Hand.Selfsight.Anchor_For (S.Memory, (if At_Low then Low.Closer.Element else High.Closer.Element));
         begin
            Now.Located := Driver.Robot.Hand.Lobes.From_Change
              (Compared.Changed, Low.Frames, High.Frames, Anchor, At_Low, Compared.Spread, Nothing,
               Motion => (if Motion = null then null else Channel_Motion'Access));
            Now.Closing := Driver.Robot.Hand.Lobes.Direction (Now.Located.Lobes, Now.Located.Still);
            Now.Change := Driver.Robot.Hand.Lobes.Closing_Change (Now.Located.Lobes, Now.Located.Still);
            Now.Status := (if Now.Located.How = Driver.Robot.Hand.Lobes.Placed then Measured else Unplaced);
         end;
      end if;
      Per_Channel (Channel) := Now;
      S.Per_Channel := Channel_Holders.To_Holder (Per_Channel);
   end Renew;

   function Overlap (A, B : Mask) return Natural;
   --  How many pixels are in both.

   function Overlap (A, B : Mask) return Natural is
      N : Natural := 0;
   begin
      if Width (A) /= Width (B) or else Height (A) /= Height (B) then
         return 0;
      end if;
      for Row in 0 .. Height (A) - 1 loop
         for Column in 0 .. Width (A) - 1 loop
            if Contains (A, Column, Row) and then Contains (B, Column, Row) then
               N := N + 1;
            end if;
         end loop;
      end loop;
      return N;
   end Overlap;

   procedure Renew_Stops
     (S       : in out State;
      Channel : Positive;
      Motion  : access procedure (Channel : Positive; Column, Row : Natural; U, V : out Real));
   --  The lobes at every stop of the channel's travel, placed as the ends' were (between the anchored end and
   --  the stop's view), each given to the lobe of the ends' placing whose pixels at the anchored end it most
   --  shares. Asked once the ends' lobes are placed, and again whenever the views have more stops.

   procedure Renew_Stops
     (S       : in out State;
      Channel : Positive;
      Motion  : access procedure (Channel : Positive; Column, Row : Natural; U, V : out Real))
   is
      Per_Channel : Channel_Array := S.Per_Channel.Element;
      Now         : Channel_State renames Per_Channel (Channel);
      Low         : constant Driver.Robot.Hand.Views.View := Driver.Robot.Hand.Views.Low_End (S.Views, Channel);
      High        : constant Driver.Robot.Hand.Views.View := Driver.Robot.Hand.Views.High_End (S.Views, Channel);
      At_Low      : constant Boolean := Now.Poses_Low >= Now.Poses_High;   --  the anchored end, as the ends' placing had it
      Anchored    : constant Driver.Robot.Hand.Views.View := (if At_Low then Low else High);
      Anchor      : constant Driver.Pixels.View :=
        Driver.Robot.Hand.Selfsight.Anchor_For (S.Memory, Anchored.Closer.Element);
      N           : constant Natural := Natural (Now.Located.Lobes.Length);
      Count       : constant Natural := Driver.Robot.Hand.Views.Stops (S.Views, Channel);
      Nothing     : constant Mask := Create (0, 0);
      --  How the picture moves per unit of the channel's reading, from the anchored end towards a stop.
      procedure Stop_Motion (Column, Row : Natural; U, V : out Real) is
      begin
         Motion (Channel, Column, Row, U, V);
         if not At_Low then
            U := -U;
            V := -V;
         end if;
      end Stop_Motion;
   begin
      Now.Stops.Clear;
      Now.Stops_Seen := Count;
      for K in 1 .. Count loop
         declare
            V        : constant Driver.Robot.Hand.Views.View := Driver.Robot.Hand.Views.Stop (S.Views, Channel, K);
            Compared : constant Driver.Pixels.Comparison := Driver.Pixels.Compare (Anchored.Frames, V.Frames);
            Tips     : Stop_Tips (1 .. N);
         begin
            if Compared.Trusted and then Driver.Images.Count (Compared.Changed) > 0 then
               declare
                  Placed : constant Driver.Robot.Hand.Lobes.Located := Driver.Robot.Hand.Lobes.From_Change
                    (Compared.Changed, Anchored.Frames, V.Frames, Anchor, True, Compared.Spread, Nothing,
                     Motion => (if Motion = null then null else Stop_Motion'Access));
               begin
                  if Placed.How = Driver.Robot.Hand.Lobes.Placed then
                     for L in 1 .. N loop
                        declare
                           Mine : constant Mask :=
                             (if At_Low then Now.Located.Lobes (L).Here else Now.Located.Lobes (L).There);
                           Best : Natural := 0;
                           Most : Natural := 0;
                        begin
                           for J in Placed.Lobes.First_Index .. Placed.Lobes.Last_Index loop
                              declare
                                 Shared : constant Natural := Overlap (Mine, Placed.Lobes (J).Here);
                              begin
                                 if Shared > Most then
                                    Best := J;
                                    Most := Shared;
                                 end if;
                              end;
                           end loop;
                           if Best > 0 and then Placed.Lobes (Best).Tip_Known_There then
                              Tips (L) := (Known  => True,
                                           Tip    => Placed.Lobes (Best).Tip_There,
                                           Centre => Placed.Lobes (Best).Centre_There,
                                           Region => Placed.Lobes (Best).There);
                           end if;
                        end;
                     end loop;
                  end if;
               end;
            end if;
            Now.Stops.Append
              (Stop_State'(Reading => Driver.Robot.Hand.Views.Reading (V, Channel), From => V.From,
                           Tips    => Tip_Holders.To_Holder (Tips)));
         end;
      end loop;
      S.Per_Channel := Channel_Holders.To_Holder (Per_Channel);
   end Renew_Stops;

   procedure Set_Status (S : in out State; Channel : Positive; To : Progress);

   procedure Set_Status (S : in out State; Channel : Positive; To : Progress) is
      Per_Channel : Channel_Array := S.Per_Channel.Element;
   begin
      Per_Channel (Channel).Status := To;
      S.Per_Channel := Channel_Holders.To_Holder (Per_Channel);
   end Set_Status;

   procedure Observe
     (S          : in out State;
      Seen       : Observation;
      Still      : Boolean;
      Closer     : Real_Array;
      Rest       : Real_Array;
      Image      : Driver.Images.Image;
      Rest_Moved : not null access function (Before, After : Real_Array) return Boolean;
      Eye_Moved  : not null access function (Before, After : Real_Array) return Boolean;
      Motion     : access procedure (Channel : Positive; Column, Row : Natural; U, V : out Real) := null)
   is
   begin
      Driver.Robot.Hand.Views.Observe (S.Views, Seen, Still, Closer, Rest, Image, Rest_Moved);
      Driver.Robot.Hand.Selfsight.Observe (S.Memory, Closer, Rest, Still, Image, Eye_Moved);
      for C in 1 .. Channels (S) loop
         if Ends_Moved (S, C) then
            Renew (S, C, Motion);
            if Status (S, C) = Measured then
               Renew_Stops (S, C, Motion);
            end if;
         elsif Status (S, C) = Measured
           --  Only while the ends are seen: a reading that moved starts an end's view again, and until it has two
           --  frames there is no end to place a stop between (Stops is 0 then, not what was seen) — R3 on the Pi,
           --  10-13: hand b's lobes placed, its ends started again after a push, and Renew_Stops asked for the low
           --  end that was not there; the driver died at beat 6166 on Low_End's precondition.
           and then Driver.Robot.Hand.Views.Has_Ends (S.Views, C)
           and then Driver.Robot.Hand.Views.Stops (S.Views, C) /= S.Per_Channel.Constant_Reference.Element (C).Stops_Seen
         then
            Renew_Stops (S, C, Motion);
         elsif Status (S, C) = Waiting and then Driver.Robot.Hand.Views.Unseen_Travel (S.Views, C) then
            Set_Status (S, C, Nothing_Moves);
         end if;
      end loop;
   end Observe;

   function Would_Extend
     (S          : State;
      Channel    : Positive;
      Rest_Moved : not null access function (Before, After : Real_Array) return Boolean) return Boolean
   is (Driver.Robot.Hand.Views.Would_Extend (S.Views, Channel, Rest_Moved));

   function Gathered (S : State) return Boolean is (Driver.Robot.Hand.Views.Gathered (S.Views));

   function Poses (S : State; Key : Real_Array) return Natural is (Driver.Robot.Hand.Selfsight.Poses (S.Memory, Key));

   function Low_End (S : State; Channel : Positive) return Driver.Robot.Hand.Views.View is
     (Driver.Robot.Hand.Views.Low_End (S.Views, Channel));

   function High_End (S : State; Channel : Positive) return Driver.Robot.Hand.Views.View is
     (Driver.Robot.Hand.Views.High_End (S.Views, Channel));

   function Lobes_Of (S : State; Channel : Positive) return Driver.Robot.Hand.Lobes.Lobe_Vectors.Vector is
     (S.Per_Channel.Constant_Reference.Element (Channel).Located.Lobes);

   function Located_Of (S : State; Channel : Positive) return Driver.Robot.Hand.Lobes.Located is
     (S.Per_Channel.Constant_Reference.Element (Channel).Located);

   function Stops (S : State; Channel : Positive) return Natural is
     (Natural (S.Per_Channel.Constant_Reference.Element (Channel).Stops.Length));

   function Stop_Reading (S : State; Channel : Positive; K : Positive) return Real is
     (S.Per_Channel.Constant_Reference.Element (Channel).Stops (K).Reading);

   function Stop_View (S : State; Channel : Positive; K : Positive) return Views.View is
      From : constant Driver.Clock.Beat := S.Per_Channel.Constant_Reference.Element (Channel).Stops (K).From;
   begin
      --  The views' stops may have grown since the placing: the view is the one that began at that beat.
      for J in 1 .. Driver.Robot.Hand.Views.Stops (S.Views, Channel) loop
         if Driver.Robot.Hand.Views.Stop (S.Views, Channel, J).From = From then
            return Driver.Robot.Hand.Views.Stop (S.Views, Channel, J);
         end if;
      end loop;
      raise Program_Error with "a stop's view is gone";
   end Stop_View;

   function Tips_At_Stop (S : State; Channel : Positive; K : Positive) return Stop_Tips is
     (S.Per_Channel.Constant_Reference.Element (Channel).Stops (K).Tips.Element);

   function Closed_End_Is_High (S : State; Channel : Positive) return Boolean is
     (S.Per_Channel.Constant_Reference.Element (Channel).Closing = Driver.Robot.Hand.Lobes.Towards_There);

   function Closing_Known (S : State; Channel : Positive) return Boolean is
     (S.Per_Channel.Constant_Reference.Element (Channel).Closing /= Driver.Robot.Hand.Lobes.Undecided);

   function Unannounced (S : State; Channel : Positive) return Boolean is
     (not S.Per_Channel.Constant_Reference.Element (Channel).Announced);

   procedure Announce (S : in out State; Channel : Positive) is
      Per_Channel : Channel_Array := S.Per_Channel.Element;
   begin
      Per_Channel (Channel).Announced := True;
      S.Per_Channel := Channel_Holders.To_Holder (Per_Channel);
   end Announce;

   function Account (S : State; Channel : Positive) return String is
      Here : constant Channel_State := S.Per_Channel.Constant_Reference.Element (Channel);
      Lobes_Text : constant String := Natural'Image (Natural (Here.Located.Lobes.Length)) & " lobes";
      Pixels_Text : constant String :=
        Natural'Image (Here.Changed) & " pixels changed, beyond " & Driver.Log.Image (Here.Beyond, 1)
        & " levels of a spread of " & Driver.Log.Image (Here.Spread, 2);
   begin
      case Here.Status is
         when Waiting =>
            return "its two ends were not both seen still";
         when Nothing_Moves =>
            return "nothing in this eye moves between its ends";
         when Everything_Moves =>
            return "half of this eye's picture or more changes between its ends, so what moved cannot be told from what did not";
         when Unlocated =>
            return "the arm has not moved the eye against its surroundings at either end's readings (seen from"
              & Here.Poses_Low'Image & " poses at the low reading, " & Driver.Log.Image (Here.Low_At, 6) & ","
              & Here.Poses_High'Image & " at the high one, " & Driver.Log.Image (Here.High_At, 6)
              & "; two are needed): " & Pixels_Text;
         when Unplaced =>
            return (case Here.Located.How is
                       when Driver.Robot.Hand.Lobes.Unseparated =>
                          "the " & Pixels_Text & ", do not fall in two groups by how much they vary over the arm's poses"
                          & " (the poses seen from:" & Here.Poses_Low'Image & " at the low reading,"
                          & Here.Poses_High'Image & " at the high one)",
                       when Driver.Robot.Hand.Lobes.One_Sided =>
                          "the " & Pixels_Text & "; of them" & Here.Located.Here'Image & " went to the low end,"
                          & Here.Located.There'Image & " to the high end," & Here.Located.Unassigned'Image
                          & " to neither; the parts they made that are attached to the picture's border and larger than the"
                          & " doubt of " & Driver.Log.Image (Here.Located.Doubt, 0) & " pixels are" & Here.Located.Parts_Here'Image
                          & " at the low end and" & Here.Located.Parts_There'Image & " at the high end",
                       when others =>
                          "the changed pixels could not be placed");
         when Measured =>
            return Lobes_Text & ", "
              & (if Here.Closing = Driver.Robot.Hand.Lobes.Towards_There then "closed at the high reading"
                 elsif Here.Closing = Driver.Robot.Hand.Lobes.Towards_Here then "closed at the low reading"
                 elsif Known (Here.Change)
                 then "closing direction not significant: their distances changed by "
                      & Driver.Log.Image (Here.Change.Value, 3) & " +- " & Driver.Log.Image (Here.Change.Sigma, 3)
                      & " pixels between the ends"
                 else "closing direction not known: there is nothing to compare their distances with ("
                      & Driver.Images.Count (Here.Located.Still)'Image & " of the robot's pixels did not change)")
              & " (" & Pixels_Text & ";" & Here.Located.Here'Image & " given to the low end,"
              & Here.Located.There'Image & " to the high end," & Here.Located.Unassigned'Image & " to neither, of doubt "
              & Driver.Log.Image (Here.Located.Doubt, 0) & " pixels, after" & Here.Located.Rounds'Image & " rounds; its parts:"
              & Here.Located.Parts_Here'Image & " at the low end," & Here.Located.Parts_There'Image & " at the high end"
              & "; the closer's own motion goes with" & Here.Located.With_Motion'Image & " of their pixels and against"
              & Here.Located.Against_Motion'Image
              & (if Here.Located.Turned
                 then ": the robot's look gave each end the other's pixels, and the closer's own motion turned them"
                 else "")
              & ")";
      end case;
   end Account;

end Driver.Robot.Hand.Sweep;
