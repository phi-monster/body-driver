package body Driver.Beats is

   Current      : aliased Driver.Observations.Observation;
   Current_Sent : Driver.Commands.Command := Driver.Commands.Hold;

   protected Episodes is
      procedure Advance;
      function Count return Natural;
   private
      N : Natural := 0;
   end Episodes;

   protected body Episodes is
      procedure Advance is
      begin
         N := N + 1;
      end Advance;

      function Count return Natural is (N);
   end Episodes;

   protected Channel is
      entry Take (Beat : out Driver.Clock.Beat);
      procedure Put (Beat : Driver.Clock.Beat; Taken : out Boolean);
      procedure Reply (C : Driver.Commands.Command);
      entry Collect (C : out Driver.Commands.Command);
   private
      Offered  : Boolean := False;
      Current  : Driver.Clock.Beat := 0;
      Replied  : Boolean := False;
      Answer   : Driver.Commands.Command := Driver.Commands.Hold;
   end Channel;

   protected body Channel is

      entry Take (Beat : out Driver.Clock.Beat) when Offered is
      begin
         Beat := Current;
         Offered := False;
      end Take;

      procedure Put (Beat : Driver.Clock.Beat; Taken : out Boolean) is
      begin
         --  A beat is taken only when the decider is already queued on Take.
         Taken := Take'Count > 0;
         if Taken then
            Current := Beat;
            Offered := True;
            Replied := False;
         end if;
      end Put;

      procedure Reply (C : Driver.Commands.Command) is
      begin
         Answer := C;
         Replied := True;
      end Reply;

      entry Collect (C : out Driver.Commands.Command) when Replied is
      begin
         C := Answer;
         Replied := False;
      end Collect;

   end Channel;

   procedure Next (Beat : out Driver.Clock.Beat) is
   begin
      Channel.Take (Beat);
   end Next;

   procedure Send (C : Driver.Commands.Command) is
   begin
      Channel.Reply (C);
   end Send;

   procedure Offer
     (Beat         : Driver.Clock.Beat;
      O            : Driver.Observations.Observation;
      Sent_Before  : Driver.Commands.Command;
      Decider_Took : out Boolean) is
   begin
      --  Safe without a lock: the main loop only offers while no decider is
      --  between Next and Send, the only window in which Latest may be read.
      Current := O;
      Current_Sent := Sent_Before;
      Channel.Put (Beat, Decider_Took);
   end Offer;

   function Latest return Observation_View is (Current'Access);

   function Last_Sent return Driver.Commands.Command is (Current_Sent);

   procedure Await (C : out Driver.Commands.Command) is
   begin
      Channel.Collect (C);
   end Await;

   procedure New_Episode is
   begin
      Episodes.Advance;
   end New_Episode;

   function Episode return Natural is (Episodes.Count);

end Driver.Beats;
