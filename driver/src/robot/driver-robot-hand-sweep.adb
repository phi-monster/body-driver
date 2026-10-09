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

   procedure Renew (S : in out State; Channel : Positive);
   --  New ends: what changed between them decides whether anything is to be
   --  placed; a measurement of older ends no longer stands.

   procedure Renew (S : in out State; Channel : Positive) is
      Low      : constant Driver.Robot.Hand.Views.View := Driver.Robot.Hand.Views.Low_End (S.Views, Channel);
      High     : constant Driver.Robot.Hand.Views.View := Driver.Robot.Hand.Views.High_End (S.Views, Channel);
      Compared : constant Driver.Pixels.Comparison := Driver.Pixels.Compare (Low.Frames, High.Frames);
      Per_Channel : Channel_Array := S.Per_Channel.Element;
      Now      : Channel_State;
      Nothing  : constant Mask := Create (0, 0);   --  the robot's own pixels that did not change: not yet known
   begin
      Now.Low_From := Low.From;
      Now.High_From := High.From;
      Now.Analysed := True;
      Now.Announced := False;
      Now.Poses_Low := Driver.Robot.Hand.Selfsight.Poses (S.Memory, Low.Closer.Element);
      Now.Poses_High := Driver.Robot.Hand.Selfsight.Poses (S.Memory, High.Closer.Element);
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
              (Compared.Changed, Low.Frames, High.Frames, Anchor, At_Low, Compared.Spread, Nothing);
            Now.Closing := Driver.Robot.Hand.Lobes.Direction (Now.Located.Lobes, Nothing);
            Now.Change := Driver.Robot.Hand.Lobes.Closing_Change (Now.Located.Lobes, Nothing);
            Now.Status := (if Now.Located.How = Driver.Robot.Hand.Lobes.Placed then Measured else Unplaced);
         end;
      end if;
      Per_Channel (Channel) := Now;
      S.Per_Channel := Channel_Holders.To_Holder (Per_Channel);
   end Renew;

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
      Eye_Moved  : not null access function (Before, After : Real_Array) return Boolean)
   is
   begin
      Driver.Robot.Hand.Views.Observe (S.Views, Seen, Still, Closer, Rest, Image, Rest_Moved);
      Driver.Robot.Hand.Selfsight.Observe (S.Memory, Closer, Rest, Still, Image, Eye_Moved);
      for C in 1 .. Channels (S) loop
         if Ends_Moved (S, C) then
            Renew (S, C);
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
              & Here.Poses_Low'Image & " poses at the low reading,"
              & Here.Poses_High'Image & " at the high one; two are needed): " & Pixels_Text;
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
                 else "closing direction not known: there is nothing to compare their distances with")
              & " (" & Pixels_Text & ";" & Here.Located.Here'Image & " given to the low end,"
              & Here.Located.There'Image & " to the high end," & Here.Located.Unassigned'Image & " to neither, of doubt "
              & Driver.Log.Image (Here.Located.Doubt, 0) & " pixels, after" & Here.Located.Rounds'Image & " rounds; its parts:"
              & Here.Located.Parts_Here'Image & " at the low end," & Here.Located.Parts_There'Image & " at the high end)";
      end case;
   end Account;

end Driver.Robot.Hand.Sweep;
