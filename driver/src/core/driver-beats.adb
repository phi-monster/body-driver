with Ada.Containers.Vectors;
with Ada.Exceptions;
with Ada.Strings.Unbounded;
with Ada.Task_Attributes;

package body Driver.Beats is

   Current      : aliased Driver.Observations.Observation;
   Current_Sent : Driver.Commands.Command := Driver.Commands.Hold;

   --  The lane a task decides for: 0 for the decider itself, 1 and up for the
   --  lanes At_Once runs.
   package Lane_Of is new Ada.Task_Attributes (Natural, 0);

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

   --  A lane's own door to the beats: it waits at Take until the channel
   --  opens the door with a beat, at its turn.
   protected type Gate is
      entry Take (Beat : out Driver.Clock.Beat);
      procedure Open (Beat : Driver.Clock.Beat);
      function Waiting return Boolean;
   private
      Opened : Boolean := False;
      Given  : Driver.Clock.Beat := 0;
   end Gate;

   protected body Gate is
      entry Take (Beat : out Driver.Clock.Beat) when Opened is
      begin
         Beat := Given;
         Opened := False;
      end Take;

      procedure Open (Beat : Driver.Clock.Beat) is
      begin
         Given := Beat;
         Opened := True;
      end Open;

      function Waiting return Boolean is (Take'Count > 0);
   end Gate;

   type Gate_Access is access all Gate;
   package Gate_Vectors is new Ada.Containers.Vectors (Positive, Gate_Access);
   package Flag_Vectors is new Ada.Containers.Vectors (Positive, Boolean);

   --  The doors of the lanes that run now, for a lane's Next to find its own:
   --  set before any lane starts, cleared once every lane has ended.
   Doors : Gate_Vectors.Vector;

   protected Channel is
      entry Take (Beat : out Driver.Clock.Beat);
      procedure Put (Beat : Driver.Clock.Beat; Taken : out Boolean);
      procedure Reply (C : Driver.Commands.Command);
      procedure Reply_If_Held (C : Driver.Commands.Command);
      function Is_Held return Boolean;
      procedure Lane_Reply (Lane : Positive; C : Driver.Commands.Command; Clash : out Boolean);
      procedure Lane_Reply_If_Held (Lane : Positive; C : Driver.Commands.Command);
      function Lane_Held (Lane : Positive) return Boolean;
      procedure Join (Gates : Gate_Vectors.Vector);
      procedure Leave;
      entry Collect (C : out Driver.Commands.Command);
   private
      Offered  : Boolean := False;
      Current  : Driver.Clock.Beat := 0;
      Replied  : Boolean := False;
      Held     : Boolean := False;   --  a beat taken by the decider and not yet answered
      Answer   : Driver.Commands.Command := Driver.Commands.Hold;
      Lanes    : Gate_Vectors.Vector;   --  empty but while At_Once runs
      Serving  : Flag_Vectors.Vector;   --  per lane: it takes the beat offered last
      Turn     : Natural := 0;          --  the lane whose window is open; 0 for none
   end Channel;

   protected body Channel is

      entry Take (Beat : out Driver.Clock.Beat) when Offered is
      begin
         Beat := Current;
         Offered := False;
         Held := True;
      end Take;

      --  The first lane after From that takes the beat offered last; 0 for none.
      function Served_After (From : Natural) return Natural is
      begin
         for L in From + 1 .. Natural (Serving.Length) loop
            if Serving (L) then
               return L;
            end if;
         end loop;
         return 0;
      end Served_After;

      procedure Put (Beat : Driver.Clock.Beat; Taken : out Boolean) is
      begin
         if Lanes.Is_Empty then
            --  A beat is taken only when the decider is already queued on Take.
            Taken := Take'Count > 0;
            if Taken then
               Current := Beat;
               Offered := True;
               Replied := False;
            end if;
         else
            --  The lanes waiting at their doors take it, one after another.
            Serving.Clear;
            for G of Lanes loop
               Serving.Append (G.Waiting);
            end loop;
            Current := Beat;
            Answer := Driver.Commands.Hold;
            Replied := False;
            Turn := Served_After (0);
            Taken := Turn > 0;
            if Taken then
               Lanes (Turn).Open (Beat);
            end if;
         end if;
      end Put;

      procedure Reply (C : Driver.Commands.Command) is
      begin
         Answer := C;
         Replied := True;
         Held := False;
      end Reply;

      function Is_Held return Boolean is (Held);

      procedure Reply_If_Held (C : Driver.Commands.Command) is
      begin
         if Held then
            Reply (C);
         end if;
      end Reply_If_Held;

      --  The lane's targets join the beat's reply, and the next lane that takes
      --  the beat has it; after the last one the reply is ready. A lane that
      --  answers a beat it does not hold changes nothing.
      procedure Lane_Reply (Lane : Positive; C : Driver.Commands.Command; Clash : out Boolean) is
      begin
         Clash := False;
         if Turn /= Lane then
            return;
         end if;
         Driver.Commands.Merge (Answer, C, Clash);
         Turn := Served_After (Lane);
         if Turn > 0 then
            Lanes (Turn).Open (Current);
         else
            Replied := True;
         end if;
      end Lane_Reply;

      procedure Lane_Reply_If_Held (Lane : Positive; C : Driver.Commands.Command) is
         Clash : Boolean;
      begin
         if Turn = Lane then
            Lane_Reply (Lane, C, Clash);
         end if;
      end Lane_Reply_If_Held;

      function Lane_Held (Lane : Positive) return Boolean is (Turn = Lane);

      procedure Join (Gates : Gate_Vectors.Vector) is
      begin
         Lanes := Gates;
         Serving.Clear;
         Turn := 0;
      end Join;

      procedure Leave is
      begin
         Lanes.Clear;
         Serving.Clear;
         Turn := 0;
      end Leave;

      entry Collect (C : out Driver.Commands.Command) when Replied is
      begin
         C := Answer;
         Replied := False;
      end Collect;

   end Channel;

   function Lane return Natural is (Lane_Of.Value);

   procedure Next (Beat : out Driver.Clock.Beat) is
      L : constant Natural := Lane_Of.Value;
   begin
      if L = 0 then
         Channel.Take (Beat);
      else
         Doors (L).Take (Beat);
      end if;
   end Next;

   procedure Send (C : Driver.Commands.Command) is
      L     : constant Natural := Lane_Of.Value;
      Clash : Boolean;
   begin
      if L = 0 then
         Channel.Reply (C);
      else
         Channel.Lane_Reply (L, C, Clash);
         if Clash then
            raise Program_Error with "lane" & L'Image & " targeted a group another lane targets in the same beat";
         end if;
      end if;
   end Send;

   procedure Release is
      L : constant Natural := Lane_Of.Value;
   begin
      if L = 0 then
         Channel.Reply_If_Held (Driver.Commands.Hold);
      else
         Channel.Lane_Reply_If_Held (L, Driver.Commands.Hold);
      end if;
   end Release;

   function Held return Boolean is
     (if Lane_Of.Value = 0 then Channel.Is_Held else Channel.Lane_Held (Lane_Of.Value));

   procedure At_Once (Lanes : Positive) is
      Own : array (1 .. Lanes) of aliased Gate;

      protected Failure is
         procedure Keep (E : Ada.Exceptions.Exception_Occurrence);
         procedure Raise_Kept;
      private
         Kept : Ada.Exceptions.Exception_Occurrence;
         Has  : Boolean := False;
      end Failure;

      protected body Failure is
         procedure Keep (E : Ada.Exceptions.Exception_Occurrence) is
         begin
            if not Has then
               Ada.Exceptions.Save_Occurrence (Kept, E);
               Has := True;
            end if;
         end Keep;

         procedure Raise_Kept is
         begin
            if Has then
               Ada.Exceptions.Reraise_Occurrence (Kept);
            end if;
         end Raise_Kept;
      end Failure;

      task type Runner is
         entry Start (Lane : Positive);
      end Runner;

      task body Runner is
         Mine : Positive := 1;
      begin
         accept Start (Lane : Positive) do
            Mine := Lane;
         end Start;
         Lane_Of.Set_Value (Mine);
         Work (Mine);
         Release;
      exception
         when E : others =>
            Release;   --  the main loop waits for an answer to a beat this lane holds
            Failure.Keep (E);
      end Runner;
   begin
      if Lane_Of.Value /= 0 then
         raise Program_Error with "lanes are run by the decider itself, not by a lane";
      end if;
      declare
         Gates : Gate_Vectors.Vector;
      begin
         for G of Own loop
            Gates.Append (G'Unchecked_Access);
         end loop;
         Doors := Gates;
         Channel.Join (Gates);
      end;
      declare
         Runners : array (1 .. Lanes) of Runner;
      begin
         for I in Runners'Range loop
            Runners (I).Start (I);
         end loop;
      end;   --  every lane has ended here
      Channel.Leave;
      Doors.Clear;
      Failure.Raise_Kept;
   end At_Once;

   --  Estimates a decider asked for and the main loop has adopted, counted:
   --  an adoption the decider has not yet waited for is not lost, and one
   --  nobody asked for (the main loop's own) wakes nobody.
   protected Estimates is
      procedure Ask;
      procedure Adopt;
      entry Wait;
   private
      Asked, Adopted : Natural := 0;
   end Estimates;

   protected body Estimates is
      procedure Ask is
      begin
         Asked := Asked + 1;
      end Ask;

      procedure Adopt is
      begin
         Adopted := Asked;
      end Adopt;

      entry Wait when Adopted >= Asked is
      begin
         null;
      end Wait;
   end Estimates;

   procedure Wait_For_Estimates is
      Beat : Driver.Clock.Beat;
   begin
      Estimates.Ask;
      Send (Driver.Commands.Hold);
      Estimates.Wait;
      Next (Beat);
   end Wait_For_Estimates;

   procedure Estimates_Adopted is
   begin
      Estimates.Adopt;
   end Estimates_Adopted;

   procedure Within_A_Beat (During : not null access procedure) is
      Beat : Driver.Clock.Beat;
   begin
      Next (Beat);
      During.all;
      Send (Driver.Commands.Hold);
   exception
      when others =>
         Release;   --  the main loop waits for an answer to the beat it gave
         raise;
   end Within_A_Beat;

   procedure Offer
     (Beat         : Driver.Clock.Beat;
      O            : Driver.Observations.Observation;
      Sent_Before  : Driver.Commands.Command;
      Decider_Took : out Boolean) is
   begin
      --  Safe without a lock: the main loop only offers while no decider or
      --  lane is between Next and Send, the only window in which Latest may be
      --  read.
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

   protected Person is
      procedure Hear (Words : String);
      function Latest return String;
      function Changes return Natural;
   private
      Text  : Ada.Strings.Unbounded.Unbounded_String;
      Count : Natural := 0;
   end Person;

   protected body Person is
      procedure Hear (Words : String) is
      begin
         if Words'Length > 0 and then Words /= Ada.Strings.Unbounded.To_String (Text) then
            Text := Ada.Strings.Unbounded.To_Unbounded_String (Words);
            Count := Count + 1;
         end if;
      end Hear;

      function Latest return String is (Ada.Strings.Unbounded.To_String (Text));
      function Changes return Natural is (Count);
   end Person;

   procedure Hear (Words : String) is
   begin
      Person.Hear (Words);
   end Hear;

   function Latest_Words return String is (Person.Latest);
   function Words_Heard return Natural is (Person.Changes);

end Driver.Beats;
