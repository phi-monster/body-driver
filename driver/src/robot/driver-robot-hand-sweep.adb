with Driver.Pixels;

package body Driver.Robot.Hand.Sweep is

   use Driver.Images;
   use type Driver.Clock.Beat;
   use type Driver.Robot.Hand.Lobes.Closing;

   function Start (Width, Height : Positive; Channels : Positive; Closer_Noise, Rest_Noise : Real_Array) return State is
     ((Width       => Width,
       Height      => Height,
       Views       => Driver.Robot.Hand.Views.Start (Width, Height, Closer_Noise, Rest_Noise),
       Per_Channel => Channel_Holders.To_Holder ([1 .. Channels => (others => <>)])));

   function Channels (S : State) return Positive is (S.Per_Channel.Element'Length);

   function Ends_Moved (S : State; Channel : Positive) return Boolean is
      Now : constant Channel_Holders.Constant_Reference_Type := S.Per_Channel.Constant_Reference;
   begin
      return Driver.Robot.Hand.Views.Has_Ends (S.Views, Channel)
        and then (Now.Element (Channel).Changed.Is_Empty
                  or else Driver.Robot.Hand.Views.Low_Beat (S.Views, Channel) /= Now.Element (Channel).Low_From
                  or else Driver.Robot.Hand.Views.High_Beat (S.Views, Channel) /= Now.Element (Channel).High_From);
   end Ends_Moved;

   procedure Renew (S : in out State; Channel : Positive);
   --  New ends: what changed between them decides whether anything is to
   --  be asked; a measurement of older ends no longer stands.

   procedure Renew (S : in out State; Channel : Positive) is
      Low     : constant Driver.Robot.Hand.Views.View := Driver.Robot.Hand.Views.Low_End (S.Views, Channel);
      High    : constant Driver.Robot.Hand.Views.View := Driver.Robot.Hand.Views.High_End (S.Views, Channel);
      Changed : constant Mask := Driver.Pixels.Changed (Low.Frames, High.Frames);
      Per_Channel : Channel_Array := S.Per_Channel.Element;
   begin
      Per_Channel (Channel) :=
        (Status    => (if Count (Changed) = 0 then Nothing_Moves else Waiting),
         Low_From  => Low.From,
         High_From => High.From,
         Changed   => Change_Holders.To_Holder (Changed),
         Lobes     => Driver.Robot.Hand.Lobes.Lobe_Vectors.Empty_Vector,
         Closing   => Driver.Robot.Hand.Lobes.Undecided);
      S.Per_Channel := Channel_Holders.To_Holder (Per_Channel);
   end Renew;

   procedure Set_Status (S : in out State; Channel : Positive; To : Progress);

   procedure Observe
     (S      : in out State;
      Seen   : Observation;
      Still  : Boolean;
      Closer : Real_Array;
      Rest   : Real_Array;
      Image  : Driver.Images.Image)
   is
   begin
      Driver.Robot.Hand.Views.Observe (S.Views, Seen, Still, Closer, Rest, Image);
      for C in 1 .. Channels (S) loop
         if Ends_Moved (S, C) then
            Renew (S, C);
         elsif Status (S, C) = Waiting and then Driver.Robot.Hand.Views.Unseen_Travel (S.Views, C) then
            Set_Status (S, C, Nothing_Moves);
         end if;
      end loop;
   end Observe;

   function Status (S : State; Channel : Positive) return Progress is
     (S.Per_Channel.Constant_Reference.Element (Channel).Status);

   function Would_Extend (S : State; Channel : Positive) return Boolean is
     (Driver.Robot.Hand.Views.Would_Extend (S.Views, Channel));

   function Wants_Correspondences (S : State; Channel : Positive) return Boolean is
     (Status (S, Channel) = Waiting and then not S.Per_Channel.Constant_Reference.Element (Channel).Changed.Is_Empty
      and then Driver.Robot.Hand.Views.Has_Ends (S.Views, Channel));

   function Low_End (S : State; Channel : Positive) return Driver.Robot.Hand.Views.View is
     (Driver.Robot.Hand.Views.Low_End (S.Views, Channel));

   function High_End (S : State; Channel : Positive) return Driver.Robot.Hand.Views.View is
     (Driver.Robot.Hand.Views.High_End (S.Views, Channel));

   function Changed_Of (S : State; Channel : Positive) return Mask is
     (S.Per_Channel.Constant_Reference.Element (Channel).Changed.Element);

   function Query_Points (S : State; Channel : Positive) return Driver.Instrument.Point_Array is
      Changed : constant Mask := Changed_Of (S, Channel);
      C0, R0  : Natural := Natural'Last;
      C1, R1  : Natural := 0;
   begin
      for R in 0 .. Height (Changed) - 1 loop
         for C in 0 .. Width (Changed) - 1 loop
            if Contains (Changed, C, R) then
               C0 := Natural'Min (C0, C);
               C1 := Natural'Max (C1, C);
               R0 := Natural'Min (R0, R);
               R1 := Natural'Max (R1, R);
            end if;
         end loop;
      end loop;
      declare
         Points : Driver.Instrument.Point_Array (1 .. (C1 - C0 + 1) * (R1 - R0 + 1));
         K : Natural := 0;
      begin
         for R in R0 .. R1 loop
            for C in C0 .. C1 loop
               K := K + 1;
               --  The pixel's centre.
               Points (K) := (U => Real (C) + 0.5, V => Real (R) + 0.5);
            end loop;
         end loop;
         return Points;
      end;
   end Query_Points;

   procedure Set_Status (S : in out State; Channel : Positive; To : Progress) is
      Per_Channel : Channel_Array := S.Per_Channel.Element;
   begin
      Per_Channel (Channel).Status := To;
      S.Per_Channel := Channel_Holders.To_Holder (Per_Channel);
   end Set_Status;

   procedure Asked (S : in out State; Channel : Positive) is
   begin
      Set_Status (S, Channel, Requested);
   end Asked;

   procedure Refuse (S : in out State; Channel : Positive) is
   begin
      Set_Status (S, Channel, Unanswered);
   end Refuse;

   procedure Answer
     (S        : in out State;
      Channel  : Positive;
      Points   : Driver.Instrument.Point_Array;
      Forward  : Driver.Instrument.Answer_Array;
      Backward : Driver.Instrument.Answer_Array;
      Attached : Driver.Images.Mask)
   is
      subtype Correspondence_Array is Driver.Robot.Hand.Lobes.Correspondence_Array;
      subtype Matcher_Noise is Driver.Robot.Hand.Lobes.Matcher_Noise;
      Changed  : constant Mask := Changed_Of (S, Channel);
      Ahead    : Correspondence_Array (Points'Range);
      Behind   : Correspondence_Array (Points'Range);
      Still_Count : Natural := 0;
   begin
      for K in Points'Range loop
         declare
            F : constant Driver.Instrument.Answer := Forward (Forward'First + K - Points'First);
            B : constant Driver.Instrument.Answer := Backward (Backward'First + K - Points'First);
         begin
            Ahead (K) := (From => Points (K), To => F.To, Back => F.Back, Matched => F.Found);
            Behind (K) := (From => Points (K), To => B.To, Back => B.Back, Matched => B.Found);
            if not Contains (Changed, Natural (Real'Floor (Points (K).U)), Natural (Real'Floor (Points (K).V))) then
               Still_Count := Still_Count + 1;
            end if;
         end;
      end loop;
      declare
         --  Pixels that did not change between the ends, both ways round:
         --  what the matcher does with pixels that did not move.
         Still : Correspondence_Array (1 .. 2 * Still_Count);
         K     : Natural := 0;
         Attached_Still : Mask := Create (S.Width, S.Height);
      begin
         for I in Points'Range loop
            if not Contains (Changed, Natural (Real'Floor (Points (I).U)), Natural (Real'Floor (Points (I).V))) then
               K := K + 1;
               Still (K) := Ahead (I);
               K := K + 1;
               Still (K) := Behind (I);
            end if;
         end loop;
         --  The robot's own pixels a lobe can be attached to are those that
         --  did not move with the closer.
         if Width (Attached) = S.Width and then Height (Attached) = S.Height then
            for R in 0 .. S.Height - 1 loop
               for C in 0 .. S.Width - 1 loop
                  if Contains (Attached, C, R) and then not Contains (Changed, C, R) then
                     Include (Attached_Still, C, R);
                  end if;
               end loop;
            end loop;
         end if;
         declare
            Noise : constant Matcher_Noise := Driver.Robot.Hand.Lobes.Noise_Of (Still);
            Found : constant Driver.Robot.Hand.Lobes.Lobe_Vectors.Vector :=
              Driver.Robot.Hand.Lobes.Find (Ahead, Behind, Noise, Attached_Still, S.Width, S.Height);
            Per_Channel : Channel_Array := S.Per_Channel.Element;
         begin
            Per_Channel (Channel).Lobes := Found;
            Per_Channel (Channel).Closing := Driver.Robot.Hand.Lobes.Direction (Found, Attached_Still, Noise);
            Per_Channel (Channel).Status := (if Found.Is_Empty then Nothing_Moves else Measured);
            S.Per_Channel := Channel_Holders.To_Holder (Per_Channel);
         end;
      end;
   end Answer;

   function Lobes_Of (S : State; Channel : Positive) return Driver.Robot.Hand.Lobes.Lobe_Vectors.Vector is
     (S.Per_Channel.Constant_Reference.Element (Channel).Lobes);

   function Closed_End_Is_High (S : State; Channel : Positive) return Boolean is
     (S.Per_Channel.Constant_Reference.Element (Channel).Closing = Driver.Robot.Hand.Lobes.Towards_There);

   function Closing_Known (S : State; Channel : Positive) return Boolean is
     (S.Per_Channel.Constant_Reference.Element (Channel).Closing /= Driver.Robot.Hand.Lobes.Undecided);

end Driver.Robot.Hand.Sweep;
