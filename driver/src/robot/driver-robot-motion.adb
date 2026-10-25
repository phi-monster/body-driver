with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Ada.Text_IO;
with Ada.Unchecked_Deallocation;
with Driver.Bytes;
with Driver.Beats;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Clock;
with Driver.Log;
with Driver.Recording;
with Driver.Observations;
with Driver.Robot.Channels;
with Driver.Robot.Flow;
with Driver.Robot.Steps;
with Driver.Robot.Stillness;
with Driver.Robot.Lockin;
with Driver.Robot.Kinematics;
with Driver.Stats;
with Driver.Uncertain;

package body Driver.Robot.Motion is

   use Ada.Numerics.Long_Elementary_Functions;

   type Count_Array is array (Positive range <>) of Natural;

   --  The longest wait for the body to settle so far (Settle): a wait twice as long is told in the log as it goes, and at
   --  every doubling after, which group or eye it waits on. A wait that never ended left nothing in the log at all
   --  (h406 on the big machine, 10-22: from beat 20461 to an estimate at 43146, and from 46635 to the run's end).
   Longest_Wait : Natural := 0;

   --  Each eye's drift a beat with nothing pushed, as the latest settle that sent nothing back saw it (Settle_Body):
   --  the slope of its picture's change from its anchor over the second half of the beats the readings stood. A body
   --  that creeps or turns on by itself (ALOHA's arms, pressed together from the start, 0.02 mm a beat; the G1 after
   --  a push turning on 0.003 to 0.016 degrees a beat, the world's truth, d98y and d117, 10-24) leaves its pictures
   --  away by that much a beat of a push, sent back or not.
   Local_Rates : Real_Vectors.Vector;

   --  Sent back and settled: a joint sent back comes back, and its eyes' pictures with it, to where the push began,
   --  however it shook or crept on the way (round 1, 10-23: arms of the SO-101, the Panda and ALOHA, creeping under
   --  their quantized readings, drifted, and drifted again once sent back); a body that a speed carried stops
   --  where it got to, and its pictures stay away. Come back: nearer to the push's first frame than halfway from what
   --  rest changes a picture by (Stillness.Eye_Rest_Change) to how far the push had taken it (Pushed, each eye's change
   --  from where the push began as it was sent back, from 1). Told only by an eye the push took further than rest takes
   --  a picture (Z of its scale beyond, as a drift away from an anchor is told), and a speed only where no such eye came
   --  back: nothing a speed moved returns when it is sent back, and a joint's push that leaves something moved (a box its
   --  jaw knocked, a view another body part moved meanwhile) is seen to stay by the eyes that saw that alone. ALOHA's jaw
   --  was a speed by its overhead eye, pushed 1.14 from where it began against rest's 0.62, and 0.99 after, while its
   --  wrist eye came back from 24.69 to 0.78 (seed 1, paced round p69a, 10-23). Considered: the eyes judged, and on
   --  return those that stayed away where the push kept the body going; Showed: some eye the push took beyond rest.
   procedure Judge_Return
     (M          : Model;
      Pushed     : Real_Array;
      Considered : in out Channel_Flags;
      Kept_Going : out Boolean;
      Showed     : out Boolean)
   is
      Text     : Unbounded_String;
      Any_Back : Boolean := False;   --  some eye the push took beyond rest came back
   begin
      Kept_Going := False;
      Showed := False;
      for E in 1 .. Eye_Count (M) loop
         if Considered (Considered'First + E - 1) then
            declare
               Now_Away : constant Real := Driver.Robot.Stillness.Eye_From_Origin (M, Eye_Id (E));
               Rest     : constant Real := Driver.Robot.Stillness.Eye_Rest_Change (M, Eye_Id (E));
               Was      : constant Real := Pushed (Pushed'First + E - 1);
               --  Further than rest takes a picture: beyond what rest changes it by, Z of its scale.
               --  Rest over as many beats as the push took: what the body drifts a beat by itself, as it was last left
               --  (Local_Rates), on top of rest's change.
               Span     : constant Natural := M.Beats - Natural'Min (M.Beats, Driver.Robot.Stillness.Eye_Origin_At (M, Eye_Id (E)));
               Base     : constant Real :=
                 (if Rest < Real'Last then Rest + (if E <= Natural (Local_Rates.Length) then Local_Rates (E - 1) else 0.0) * Real (Span)
                  else Rest);
               Bound    : constant Real := Base + Driver.Conventions.Z * Driver.Robot.Stillness.Eye_Rest_Scale (M, Eye_Id (E));
               Beyond   : constant Boolean :=
                 Now_Away < Real'Last and then Was < Real'Last and then Rest < Real'Last and then Was > Bound;
               --  Stayed away: nearer to how far the push took it than to rest, and itself further than rest takes a
               --  picture. Nearer the halfway mark alone, the G1's hands were speeds by their eyes' 2.6 and 3.0 after
               --  pushes of 2.0 and 2.4 against rest's 1.5 (seed 1, paced round p79b, 10-23).
               Stayed   : constant Boolean := Beyond and then Now_Away > Real'Max ((Base + Was) / 2.0, Bound);
            begin
               Considered (Considered'First + E - 1) := Stayed;
               Kept_Going := Kept_Going or else Stayed;
               Showed := Showed or else Beyond;
               Any_Back := Any_Back or else (Beyond and then not Stayed);
               Append (Text, " eye" & E'Image & " " & Driver.Log.Image (Now_Away, 2) & " from where the push began"
                       & " (pushed " & Driver.Log.Image (Was, 2) & ", rest " & Driver.Log.Image (Rest, 2)
                       & (if Base > Rest and then Base < Real'Last then " and drift " & Driver.Log.Image (Base - Rest, 2) else "")
                       & (if not Beyond then "): no further than rest takes it;"
                          elsif Stayed then "): stayed away;" else "): came back;"));
            end;
         end if;
      end loop;
      Kept_Going := Kept_Going and then not Any_Back;
      if not Kept_Going then
         Considered := [others => False];
      end if;
      Driver.Log.Line (Driver.Log.Robot, "motion: sent back and settled:" & To_String (Text)
                       & (if Kept_Going then " the push kept the body going" else " no speed"));
   end Judge_Return;

   procedure Settle_Body
     (M            : in out Model;
      Back         : Driver.Commands.Command;
      Use_Back     : Boolean;
      Beats_Waited : out Natural;
      Kept_Going   : out Boolean;
      Kept_Eyes    : out Channel_Flags;
      Came_Back    : out Boolean)
   is
      B : Driver.Clock.Beat;
      --  What the wait was for, for the log: per group the beats its readings still moved, per eye the beats its
      --  picture had not settled (R5 on the Pi, 10-13: the rounds' settling between pushes grew to 24 s a push).
      Groups : constant Natural := Group_Count (M);
      Eyes   : constant Natural := Eye_Count (M);
      Group_Beats : array (1 .. Groups) of Natural := [others => 0];
      Eye_Beats   : array (1 .. Eyes) of Natural := [others => 0];

      --  A group whose readings still move but go nowhere is as settled as it will be, held where it was sent: it
      --  goes nowhere as a push that moves about does (Steps.Follow), when it has come neither nearer its target nor
      --  farther from where it stood as the wait began, by a step the one test of motion sees and by more than its
      --  own chatter makes, for as long as it took it to come as near or as far as it did, nor for less than the
      --  body's delay. A body may go round a cycle of poses where it was sent (s31, servo seed 7 of the fake robot,
      --  10-17: its hand pressed on the table, arm 1 and its closer went round four poses a beat apart from beat
      --  6409, and the boot held here five thousand beats with nothing in its log), or jitter about where it stands:
      --  h401, h404 and h408 (humanoids of the default fake robot on the big machine, 10-22), a hand resting on the
      --  table, its joints a millionth of a radian either way at every beat against the 1e-12 their readings' noise
      --  had measured with nothing touched, waited here 27800, 52000 and 33000 beats. Each beat of a jitter was
      --  motion, and no cycle repeats a jitter. A cycle comes no nearer and goes no farther either, after its first
      --  round.
      From        : array (1 .. Groups) of Real_Vectors.Vector;   --  the readings as the wait began, once read
      From_Beat   : array (1 .. Groups) of Natural := [others => 0];
      Nearest     : array (1 .. Groups) of Extreme;
      Farthest    : array (1 .. Groups) of Extreme;
      Progress_At : array (1 .. Groups) of Natural := [others => 0];   --  the beat of the wait it last came nearer
                                                                     --  or went farther at
      Delay_Beats : Natural;
      Delay_Known : Boolean;

      --  The distance of the group's readings at Beat from its target in effect and from where the wait began
      --  (less the creep of the beats between, Channels.Drift), and the change of each channel from there.
      procedure Distances (G : Group_Id; Beat : Natural; Near, Far : out Real; To_Target, Away : out Real_Array) is
         Aimed : constant Boolean := Is_Commandable (M, G) and then Driver.Robot.Channels.Has_Target (M, G, Beat);
      begin
         Near := 0.0;
         Far := 0.0;
         for C in Away'Range loop
            Away (C) := Driver.Robot.Channels.Reading (M, G, Beat, C) - From (Positive (G)) (C - 1)
              - Driver.Robot.Channels.Drift (M, G, C) * Real (Beat - From_Beat (Positive (G)));
            To_Target (C) := (if Aimed then Driver.Robot.Channels.Target (M, G, Beat, C)
                                             - Driver.Robot.Channels.Reading (M, G, Beat, C) else 0.0);
            Near := Near + To_Target (C) ** 2;
            Far := Far + Away (C) ** 2;
         end loop;
         Near := Sqrt (Near);
         Far := Sqrt (Far);
      end Distances;

      --  Follows the group through the beat just read, as Steps.Follow a push.
      procedure Follow (G : Group_Id) is
         Beat : constant Natural := M.Beats - 1;
         Size : constant Natural := Group_Size (M, G);
         I    : constant Positive := Positive (G);
      begin
         if not Driver.Robot.Channels.Has_Reading (M, G, Beat) then
            return;
         end if;
         if From (I).Is_Empty then
            for C in 1 .. Size loop
               From (I).Append (Driver.Robot.Channels.Reading (M, G, Beat, C));
            end loop;
            From_Beat (I) := Beat;
            declare
               Near, Far : Real;
               To_Target, Away : Real_Array (1 .. Size);
            begin
               Distances (G, Beat, Near, Far, To_Target, Away);
               Nearest (I) := (Value => Near, others => <>);
               Farthest (I) := (Value => 0.0, others => <>);
               Driver.Robot.Steps.Note (Nearest (I), Near, Back => False);
               Driver.Robot.Steps.Note (Farthest (I), Far, Back => False);
            end;
            Progress_At (I) := Beats_Waited;
            return;
         end if;
         declare
            Near, Far : Real;
            To_Target, Away : Real_Array (1 .. Size);
            Steps : Real_Array (1 .. Size);
            Stepped : Boolean := False;
            --  The change along the given direction, D scaled to a length, as the one test of motion sees it: the
            --  channels' visible steps made once a beat, at the first that needs them.
            function Seen (D : Real_Array; Length, Norm : Real) return Boolean is
            begin
               if not Stepped then
                  Steps := Driver.Robot.Channels.Steps_Of (M, G);
                  Stepped := True;
               end if;
               return Norm > 0.0
                 and then Driver.Robot.Channels.Visible (M, G, [for C in D'Range => D (C) * Length / Norm], Steps);
            end Seen;
         begin
            Distances (G, Beat, Near, Far, To_Target, Away);
            declare
               N : Extreme renames Nearest (I);
               F : Extreme renames Farthest (I);
               --  Back the way it had come since the beat before, by a step the one test of motion sees: farther from
               --  its target, or nearer to where the wait began.
               Back_Near : constant Boolean :=
                 N.Followed > 0 and then Near > N.Last and then Seen (To_Target, Near - N.Last, Near);
               Back_Far  : constant Boolean :=
                 F.Followed > 0 and then Far < F.Last and then Seen (Away, F.Last - Far, Far);
               Nearer    : constant Boolean := Near < N.Value and then Seen (To_Target, N.Value - Near, Near)
                 and then Driver.Robot.Steps.Beyond_Chatter (N, N.Value - Near);
               Farther   : constant Boolean := Far > F.Value and then Seen (Away, Far - F.Value, Far)
                 and then Driver.Robot.Steps.Beyond_Chatter (F, Far - F.Value);
            begin
               if Nearer then
                  N := (Value => Near, others => <>);
               end if;
               if Farther then
                  F := (Value => Far, others => <>);
               end if;
               Driver.Robot.Steps.Note (N, Near, Back => not Nearer and then Back_Near);
               Driver.Robot.Steps.Note (F, Far, Back => not Farther and then Back_Far);
               if Nearer or else Farther then
                  Progress_At (I) := Beats_Waited;
               end if;
            end;
         end;
      end Follow;

      function Nowhere (G : Group_Id) return Boolean is
        (not From (Positive (G)).Is_Empty
         and then Beats_Waited - Progress_At (Positive (G)) > Natural'Max (Progress_At (Positive (G)), Delay_Beats));

      Still_Now   : array (1 .. Groups) of Boolean := [others => False];
      Ended_There : array (1 .. Groups) of Boolean := [others => False];   --  the group ended the wait going nowhere
      Tell_At     : Positive := 2 * Natural'Max (1, Longest_Wait);   --  the wait's length at which it is told next

      --  A push that keeps the body going (Use_Back): the beat of the wait the readings last began to stand at, the
      --  slowest eye's lag (a picture that trails its readings has not drifted on before it has passed), whether
      --  Back was sent, and each eye's change from the frame the push began from (Stillness.Set_Origin) then.
      Stood_Since   : Integer := -1;
      Lag           : Natural := 0;
      Sent_Back     : Boolean := False;
      Anchored      : Boolean := False;   --  every eye's anchor was taken as the readings stood (Stillness.Set_Anchor)
      Stood_Before  : Natural := 0;       --  the beats the readings stood, past the lag, before the drift was told
      Pushed_Away   : Real_Array (1 .. Eyes) := [others => Real'Last];
      --  Each eye's farthest change from its anchor, and the beat of the wait it was reached at: an eye whose picture went
      --  no farther from where it was as the readings stood, for as long again as it took to get there, the body's delay,
      --  and the longest any wait of the body took before, is as settled as it will be, as a group going nowhere is
      --  (Nowhere); the last, so that a picture this body settles in time is waited for as before: a push that began
      --  from one not yet settled is no row of the lock-in. The G1 held an arm out, its balance swaying that arm's eye,
      --  and waited here 505 beats for it to settle, its longest wait before 75 (seed 1, paced round p75a, 10-23).
      Eye_Far       : array (1 .. Eyes) of Real := [others => 0.0];
      Eye_Far_At    : array (1 .. Eyes) of Natural := [others => 0];
      Ended_Nowhere : Boolean := False;   --  some eye ended the wait going no farther, unstilled (Eye_Nowhere)
      Aways         : array (1 .. Eyes) of Real_Vectors.Vector;   --  each eye's change from its anchor, a beat each
      function Eye_Nowhere (E : Positive) return Boolean is
        (Anchored
         and then Beats_Waited - Eye_Far_At (E) > Natural'Max (Natural'Max (Eye_Far_At (E), Delay_Beats), Longest_Wait));

      --  The eye's picture drifts: beat on beat (Stillness.Eye_Drift_Run), or away from its anchor (Eye_Away_Run), two
      --  beats running, the least that tells a lasting drift from one beat's.
      function Drifted (E : Eye_Id) return Boolean is
        (Driver.Robot.Stillness.Eye_Drift_Run (M, E) >= 2
         or else (Anchored and then Driver.Robot.Stillness.Eye_Away_Run (M, E) >= 2));

      procedure Tell is
         Text : Unbounded_String;
      begin
         for G in 1 .. Groups loop
            if not Still_Now (G) then
               Append (Text, " group" & G'Image & (if Nowhere (Group_Id (G)) then " going nowhere" else " moving")
                       & " (farther or nearer last at" & Progress_At (G)'Image & ")");
            end if;
         end loop;
         for E in 1 .. Eyes loop
            if not Driver.Robot.Stillness.Eye_Settled (M, Eye_Id (E)) then
               Append (Text, " eye" & E'Image & " unsettled");
            end if;
         end loop;
         Driver.Log.Line (Driver.Log.Robot, "motion: still waiting after" & Beats_Waited'Image & " beats:" & To_String (Text));
      end Tell;
   begin
      Beats_Waited := 0;
      Kept_Going := False;
      Kept_Eyes := [others => False];
      Came_Back := False;
      for E in 1 .. Eyes loop
         Lag := Natural'Max (Lag, Natural'Max (0, Image_Lag (M, Eye_Id (E))));
      end loop;
      Driver.Robot.Steps.Body_Delay (M, Delay_Beats, Delay_Known);
      loop
         Driver.Beats.Next (B);
         if M.Beats > 0 then
            for G in 1 .. Groups loop
               Still_Now (G) := Driver.Robot.Stillness.Group_Still (M, Group_Id (G), M.Beats - 1);
               Follow (Group_Id (G));
            end loop;
         end if;
         if Anchored then
            for E in 1 .. Eyes loop
               Aways (E).Append (Driver.Robot.Stillness.Eye_From_Anchor (M, Eye_Id (E)));
            end loop;
         end if;
         if Anchored then
            for E in 1 .. Eyes loop
               if Driver.Robot.Stillness.Eye_From_Anchor (M, Eye_Id (E)) > Eye_Far (E) then
                  Eye_Far (E) := Driver.Robot.Stillness.Eye_From_Anchor (M, Eye_Id (E));
                  Eye_Far_At (E) := Beats_Waited;
               end if;
            end loop;
         end if;
         declare
            --  An eye settles as its watch tells (Stillness.Eye_Settled). A group going nowhere restarts that watch at
            --  every beat it jitters back into motion (the body beginning to move, by the one test of motion), and the
            --  watch of an eye that sees nothing of the jitter then never ends: h406 on the big machine (10-22) waited
            --  from beat 46635 to the run's end, and another humanoid 54 beats, its group going nowhere from the third
            --  and eye 1 unsettled all 54. While a group still moves and every such group goes nowhere, an eye whose
            --  picture shows nothing new against its still view (Stillness.Eye_Still) is as settled as it will be.
            Jittering : constant Boolean := (for some G in 1 .. Groups => not Still_Now (G));
            Done : constant Boolean :=
              M.Beats > 0 and then (for all G in 1 .. Groups => Still_Now (G) or else Nowhere (Group_Id (G)))
              and then (for all E in 1 .. Eyes => Driver.Robot.Stillness.Eye_Settled (M, Eye_Id (E))
                          or else (Jittering and then Driver.Robot.Stillness.Eye_Still (M, Eye_Id (E)))
                          or else Eye_Nowhere (E));
            --  Every commanded reading stands: a group no command reaches (a legged robot's legs, read only) moves as
            --  the body goes, and is not waited for here.
            Standing : constant Boolean :=
              M.Beats > 0
              and then (for all G in 1 .. Groups =>
                          not Is_Commandable (M, Group_Id (G)) or else Still_Now (G) or else Nowhere (Group_Id (G)));
            Drifting : Boolean;
         begin
            if not Done and then M.Beats > 0 then
               for G in 1 .. Groups loop
                  if not Still_Now (G) then
                     Group_Beats (G) := Group_Beats (G) + 1;
                  end if;
               end loop;
               for E in 1 .. Eyes loop
                  if not Driver.Robot.Stillness.Eye_Settled (M, Eye_Id (E)) then
                     Eye_Beats (E) := Eye_Beats (E) + 1;
                  end if;
               end loop;
            end if;
            if Done then
               for G in 1 .. Groups loop
                  Ended_There (G) := not Still_Now (G);
               end loop;
               Ended_Nowhere :=
                 (for some E in 1 .. Eyes =>
                    not Driver.Robot.Stillness.Eye_Settled (M, Eye_Id (E))
                    and then not (Jittering and then Driver.Robot.Stillness.Eye_Still (M, Eye_Id (E))));
            end if;
            if not Standing then
               Stood_Since := -1;
            elsif Stood_Since < 0 then
               Stood_Since := Beats_Waited;
            end if;
            --  Each eye's picture as the readings stood, past the slowest eye's lag: a drift too slow for one beat to
            --  show goes on away from it (Stillness.Eye_Away_Run; round 1, 10-23: Stretch's base at its first level,
            --  3 mm a second, drifted no beat's change past its noise).
            if Anchored and then not Standing then
               for E in 1 .. Eyes loop
                  Driver.Robot.Stillness.Drop_Anchor (M, Eye_Id (E));
               end loop;
               Anchored := False;
            elsif not Anchored and then Standing and then Beats_Waited - Stood_Since > Lag then
               for E in 1 .. Eyes loop
                  Driver.Robot.Stillness.Set_Anchor (M, Eye_Id (E));
                  Eye_Far (E) := 0.0;
                  Eye_Far_At (E) := Beats_Waited;
                  Aways (E).Clear;
               end loop;
               Anchored := True;
            end if;
            Drifting := Use_Back and then Standing and then Beats_Waited - Stood_Since > Lag
              and then (for some E in 1 .. Eyes => Drifted (Eye_Id (E)));
            if Drifting and then not Done and then not Sent_Back then
               Sent_Back := True;
               Stood_Before := Beats_Waited - Stood_Since;
               for E in 1 .. Eyes loop
                  Pushed_Away (E) := Driver.Robot.Stillness.Eye_From_Origin (M, Eye_Id (E));
               end loop;
               Stood_Since := -1;
               declare
                  Which : Unbounded_String;
               begin
                  for E in 1 .. Eyes loop
                     Kept_Eyes (Kept_Eyes'First + E - 1) := Drifted (Eye_Id (E));
                     if Kept_Eyes (Kept_Eyes'First + E - 1) then
                        Append (Which, " eye" & E'Image & " (beat on beat"
                                & Driver.Robot.Stillness.Eye_Drift_Run (M, Eye_Id (E))'Image & ", away from its anchor"
                                & Driver.Robot.Stillness.Eye_Away_Run (M, Eye_Id (E))'Image & ")");
                     end if;
                  end loop;
                  Driver.Log.Line (Driver.Log.Robot, "motion: every commanded reading stands, after" & Stood_Before'Image
                                   & " beats" & To_String (Which) & " drift on; sent back to its hold");
               end;
               Driver.Beats.Send (Back);
            else
               Driver.Beats.Send (Driver.Commands.Hold);
            end if;
            exit when Done;
         end;
         Beats_Waited := Beats_Waited + 1;
         if Beats_Waited >= Tell_At then
            Tell;
            Tell_At := 2 * Tell_At;
         end if;
      end loop;
      --  The longest wait the body took to settle, from waits whose every eye stilled: one an eye ended going no
      --  farther (Eye_Nowhere) waited at least the longest before, and kept so it lengthened every wait after it.
      --  The G1, its balance swaying an eye on for a hundred beats after one push, then waited 114, 159 and 153
      --  beats a push and was unfinished at the cap (seed 3, paced round p128a, 10-24).
      if not Ended_Nowhere then
         Longest_Wait := Natural'Max (Longest_Wait, Beats_Waited);
      end if;
      if not Use_Back then
         for E in 1 .. Eyes loop
            declare
               N : constant Natural := Natural (Aways (E).Length);
               H : constant Natural := N / 2;
               Rate : Real := 0.0;
            begin
               if N >= 2 and then N - 1 > H then
                  Rate := Real'Max (0.0, (Aways (E) (N - 1) - Aways (E) (H)) / Real (N - 1 - H));
               end if;
               while Natural (Local_Rates.Length) < E loop
                  Local_Rates.Append (0.0);
               end loop;
               Local_Rates.Replace_Element (E - 1, Rate);
            end;
         end loop;
      end if;
      if Beats_Waited > 0 then
         declare
            Text : Unbounded_String;
         begin
            for G in 1 .. Groups loop
               if Group_Beats (G) > 0 then
                  Append (Text, " group" & G'Image & " moving" & Group_Beats (G)'Image
                          & (if Ended_There (G) then " (going nowhere since" & Progress_At (G)'Image & ")" else ""));
               end if;
            end loop;
            for E in 1 .. Eyes loop
               if Eye_Beats (E) > 0 then
                  Append (Text, " eye" & E'Image & " unsettled" & Eye_Beats (E)'Image);
               end if;
            end loop;
            Driver.Log.Line (Driver.Log.Robot, "motion: settled after" & Beats_Waited'Image & " beats:" & To_String (Text));
         end;
      end if;
      if Anchored then
         for E in 1 .. Eyes loop
            Driver.Robot.Stillness.Drop_Anchor (M, Eye_Id (E));
         end loop;
      end if;
      --  Weighed by every eye, not only those that drifted on as it was sent back: an eye the push took beyond rest and
      --  that came back tells it was no speed, whether or not it drifted then. Weighed by the drifting eyes alone, the
      --  G1's right arm was a speed: its own wrist eye, pushed 15.33 against rest's 1.25, came back to 1.63 the first
      --  time and had stopped as the arm did the second, while its head's and other wrist's eyes, swaying on as the
      --  body stood, stayed away (rec315 g12a, 10-25); taken for a speed, the arm was pushed no more as a joint, and
      --  its wrist eye rode its hand.
      if Sent_Back then
         declare
            Showed : Boolean;
         begin
            Kept_Eyes := [others => True];
            Judge_Return (M, Pushed_Away, Kept_Eyes, Kept_Going, Showed);
         end;
      end if;
      Came_Back := Sent_Back and then not Kept_Going;
   end Settle_Body;

   function Eyes_From_Origin (M : Model) return Real_Array is
     ([for E in 1 .. Eye_Count (M) => Driver.Robot.Stillness.Eye_From_Origin (M, Eye_Id (E))]);

   procedure Judge_Sent_Back
     (M            : in out Model;
      Pushed       : Real_Array;
      Beats_Waited : out Natural;
      Kept_Going   : out Boolean;
      Kept_Eyes    : out Channel_Flags;
      Came_Back    : out Boolean)
   is
      Considered : Channel_Flags (1 .. Eye_Count (M)) := [others => True];
      Showed     : Boolean;
   begin
      Settle (M, Beats_Waited);
      Judge_Return (M, Pushed, Considered, Kept_Going, Showed);
      Kept_Eyes := Considered;
      Came_Back := Showed and then not Kept_Going;
   end Judge_Sent_Back;

   procedure Mark_Origins (M : in out Model) is
   begin
      Driver.Recording.Write_Shared (Driver.Recording.Model_Noted, Driver.Bytes.To_Bytes ("origins"));
      for E in 1 .. Eye_Count (M) loop
         Driver.Robot.Stillness.Set_Origin (M, Eye_Id (E));
      end loop;
   end Mark_Origins;

   function Reach_Ratio (M : Model) return Real is
      Gate  : constant Driver.Uncertain.Gate := Driver.Uncertain.Vector_Gate (2);
      Ratio : Real := 1.0;
      Most  : Real := 0.0;       --  the median displacement of the moved cells of the eye with the largest share moved
      Share : Real := 0.0;       --  that share, of the eye's textured cells
      Reach : Real := 0.0;       --  half a cell of that eye, in pixels
      Beyond : Boolean := False;  --  some eye has more of its moved cells beyond reach than within it
   begin
      for E in 1 .. Eye_Count (M) loop
         declare
            S : Eye_Stream renames M.Eyes (Eye_Id (E));
            N : constant Natural := Cells (S.Grid);
            Latest : constant Luma_Access := Latest_Frame (S);
         begin
            if S.Has_Origin and then Latest /= null and then S.Origin /= null
              and then S.Origin'Length = Latest'Length and then N > 0
              and then Natural (S.Luma_Variance.Length) >= N
            then
               declare
                  type Real_Access is access Real_Array;
                  type Flag_Access is access Flow.Flag_Array;
                  procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
                  procedure Free is new Ada.Unchecked_Deallocation (Flow.Flag_Array, Flag_Access);
                  Du       : Real_Access := new Real_Array (1 .. N);
                  Dv       : Real_Access := new Real_Array (1 .. N);
                  Cond     : Real_Access := new Real_Array (1 .. N);
                  Noise    : Real_Access := new Real_Array (1 .. N);
                  Resolved : Flag_Access := new Flow.Flag_Array (1 .. N);
                  Shifts   : Real_Access := new Real_Array (1 .. N);
                  Far, Within, Textured : Natural := 0;
                  X0, X1, Y0, Y1 : Natural;
               begin
                  for C in 1 .. N loop
                     Noise (C) := S.Luma_Variance (C - 1) * Rest_Factor_Of (S, C - 1) ** 2;
                  end loop;
                  Flow.Displacements (S.Grid, S.Origin.all, Latest.all, Noise.all, Du.all, Dv.all, Cond.all,
                                      Resolved.all);
                  for C in 1 .. N loop
                     if Cond (C) > 0.0 and then (C - 1 >= Natural (S.Textured.Length) or else S.Textured (C - 1)) then
                        Textured := Textured + 1;
                        if not Resolved (C) then
                           Far := Far + 1;
                        elsif Driver.Uncertain.Significant
                            (Gate, Sqrt (Du (C) ** 2 + Dv (C) ** 2),
                             Rest_Factor_Of (S, C - 1) * Flow.Noise_Floor (Cond (C), S.Luma_Variance (C - 1)))
                        then
                           Within := Within + 1;
                           Shifts (Within) := Sqrt (Du (C) ** 2 + Dv (C) ** 2);
                        end if;
                     end if;
                  end loop;
                  Flow.Bounds (S.Grid, 1, X0, X1, Y0, Y1);
                  if Far > Within then
                     Beyond := True;
                  elsif Within > 0 and then Real (Far + Within) / Real (Textured) > Share then
                     --  The eye the push moved most broadly: an eye that sees a few cells move at rest (RB-Y1's wrist
                     --  eyes, a hundred of their 244) shows them moved by a pixel whatever was pushed, and its median
                     --  is noise's: RB-Y1's head, its own camera's 86 moved cells by 0.07 px, was scaled by a wrist
                     --  eye's 47 at 1.27 px and left its camera short of the flow's reach, its torso the camera's
                     --  own group by it (seed 1, d117, 10-24).
                     Share := Real (Far + Within) / Real (Textured);
                     Most := Driver.Stats.Median (Shifts (1 .. Within));
                     Reach := Real (Natural'Min (X1 - X0, Y1 - Y0)) / 2.0;
                  end if;
                  Free (Du);
                  Free (Dv);
                  Free (Cond);
                  Free (Noise);
                  Free (Resolved);
                  Free (Shifts);
               end;
            end if;
         end;
      end loop;
      if not Beyond and then Most > 0.0 and then Reach > Most then
         Ratio := Reach / Most;
      end if;
      return Ratio;
   end Reach_Ratio;

   procedure Note_Stayed (M : in out Model; G : Group_Id; Way : Real) is
      Gate : constant Driver.Uncertain.Gate := Driver.Uncertain.Vector_Gate (2);
   begin
      Driver.Recording.Write_Shared
        (Driver.Recording.Model_Noted, Driver.Bytes.To_Bytes ("stayed" & G'Image & (if Way > 0.0 then " 1" else " -1")));
      for E in 1 .. Eye_Count (M) loop
         declare
            S : Eye_Stream renames M.Eyes (Eye_Id (E));
            N : constant Natural := Cells (S.Grid);
            Latest : constant Luma_Access := Latest_Frame (S);
         begin
            if S.Has_Origin and then Latest /= null and then S.Origin /= null
              and then S.Origin'Length = Latest'Length and then N > 0
              and then Natural (S.Luma_Variance.Length) >= N
            then
               declare
                  --  Sized by cells: on the heap, as Measure_Displacement has them.
                  type Real_Access is access Real_Array;
                  type Flag_Access is access Flow.Flag_Array;
                  procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
                  procedure Free is new Ada.Unchecked_Deallocation (Flow.Flag_Array, Flag_Access);
                  Du       : Real_Access := new Real_Array (1 .. N);
                  Dv       : Real_Access := new Real_Array (1 .. N);
                  Cond     : Real_Access := new Real_Array (1 .. N);
                  Noise    : Real_Access := new Real_Array (1 .. N);
                  Resolved : Flag_Access := new Flow.Flag_Array (1 .. N);
                  Moved, Textured, Far : Natural := 0;
                  Shifts   : Real_Access := new Real_Array (1 .. N);   --  the moved cells' displacements, in pixels
                  Resolved_Moved : Natural := 0;
                  --  The cells that did not move by more than each can tell alone: their displacements pooled by inverse
                  --  variance, in each picture direction, and how many they are.
                  Pu, Wu, Pv, Wv : Real := 0.0;
                  Faint    : Natural := 0;
                  Pulled   : Boolean := False;
               begin
                  for C in 1 .. N loop
                     Noise (C) := S.Luma_Variance (C - 1) * Rest_Factor_Of (S, C - 1) ** 2;
                  end loop;
                  Flow.Displacements (S.Grid, S.Origin.all, Latest.all, Noise.all, Du.all, Dv.all, Cond.all,
                                      Resolved.all);
                  for C in 1 .. N loop
                     if Cond (C) > 0.0 and then (C - 1 >= Natural (S.Textured.Length) or else S.Textured (C - 1)) then
                        Textured := Textured + 1;
                        if not Resolved (C) then
                           Moved := Moved + 1;
                           Far := Far + 1;
                        else
                           declare
                              Sd : constant Real :=
                                Rest_Factor_Of (S, C - 1) * Flow.Noise_Floor (Cond (C), S.Luma_Variance (C - 1));
                           begin
                              if Driver.Uncertain.Significant (Gate, Sqrt (Du (C) ** 2 + Dv (C) ** 2), Sd) then
                                 Moved := Moved + 1;
                                 Resolved_Moved := Resolved_Moved + 1;
                                 Shifts (Resolved_Moved) := Sqrt (Du (C) ** 2 + Dv (C) ** 2);
                              elsif Sd > 0.0 and then Sd < Real'Last then
                                 Pu := Pu + Du (C) / Sd ** 2;
                                 Wu := Wu + 1.0 / Sd ** 2;
                                 Pv := Pv + Dv (C) / Sd ** 2;
                                 Wv := Wv + 1.0 / Sd ** 2;
                                 Faint := Faint + 1;
                              end if;
                           end;
                        end if;
                     end if;
                  end loop;
                  --  The cells that did not move alone, pulled together the same way (a chi square of two degrees, one
                  --  pooled displacement per picture direction, as the lock-in pools a push's gains: Rest_Pull): a far
                  --  part of a carried eye's view, moved by less than any one cell can tell, moved with the rest, and the
                  --  push left the view moved, not unmoved (Stayed_Pulled, Graph). Read cell by cell alone, the rig's arm
                  --  whose eye shows a near object move twenty times as much as the rest left 12 of its 48 cells moved,
                  --  and carried nothing (robot.boot.step, 10-24); a view shaken that came back pools to nothing. Not
                  --  counted as moved cells: Stretch's gripper, its push bending its arm, left its wrist eye's view
                  --  pulled so, as its arm did, and took the eye from its arm (seed 1, paced round p177ca, 10-24).
                  Pulled := Faint > 0 and then Wu > 0.0 and then Wv > 0.0
                    and then Driver.Distributions.Chi_Square_Deviate (Pu ** 2 / Wu + Pv ** 2 / Wv, 2) > Driver.Conventions.Z;
                  while Natural (S.Stayed_Pulled.Length) < Natural (G) loop
                     S.Stayed_Pulled.Append (0);
                  end loop;
                  if Pulled then
                     S.Stayed_Pulled.Replace_Element (Natural (G) - 1, S.Stayed_Pulled (Natural (G) - 1) + 1);
                  end if;
                  if Textured > 0 then
                     while Natural (S.Stayed_Pushes.Length) < Natural (G) loop
                        S.Stayed_Pushes.Append (0);
                        S.Stayed_Far.Append (0.0);
                        S.Stayed_Moved.Append (0);
                        S.Stayed_Cells.Append (0);
                        S.Stayed_Far_Cells.Append (0);
                        S.Stayed_Best_Moved.Append (0);
                        S.Stayed_Best_Cells.Append (0);
                     end loop;
                     S.Stayed_Pushes.Replace_Element (Natural (G) - 1, S.Stayed_Pushes (Natural (G) - 1) + 1);
                     S.Stayed_Far.Replace_Element (Natural (G) - 1, S.Stayed_Far (Natural (G) - 1) + Real (Far) / Real (Textured));
                     S.Stayed_Moved.Replace_Element (Natural (G) - 1, S.Stayed_Moved (Natural (G) - 1) + Moved);
                     S.Stayed_Cells.Replace_Element (Natural (G) - 1, S.Stayed_Cells (Natural (G) - 1) + Textured);
                     S.Stayed_Far_Cells.Replace_Element (Natural (G) - 1, S.Stayed_Far_Cells (Natural (G) - 1) + Far);
                     if S.Stayed_Best_Cells (Natural (G) - 1) = 0
                       or else Real (Moved) / Real (Textured)
                               > Real (S.Stayed_Best_Moved (Natural (G) - 1)) / Real (S.Stayed_Best_Cells (Natural (G) - 1))
                     then
                        S.Stayed_Best_Moved.Replace_Element (Natural (G) - 1, Moved);
                        S.Stayed_Best_Cells.Replace_Element (Natural (G) - 1, Textured);
                     end if;
                  end if;
                  Driver.Log.Line (Driver.Log.Robot, "motion: group" & G'Image & "'s push "
                                   & (if Way > 0.0 then "upwards" else "downwards") & " left eye" & E'Image
                                   & " changed:" & Moved'Image & " of" & Textured'Image
                                   & " textured cells moved and stayed," & Far'Image & " of them too far to resolve, the"
                                   & " others by a median of "
                                   & (if Resolved_Moved > 0 then Driver.Log.Image (Driver.Stats.Median (Shifts (1 .. Resolved_Moved)), 2)
                                      else "-") & " px");
                  Free (Du);
                  Free (Dv);
                  Free (Cond);
                  Free (Noise);
                  Free (Resolved);
                  Free (Shifts);
               end;
            end if;
         end;
      end loop;
   end Note_Stayed;

   procedure Settle (M : in out Model; Beats_Waited : out Natural) is
      Kept, Back : Boolean;
      Eyes       : Channel_Flags (1 .. Eye_Count (M));
   begin
      Settle_Body (M, Driver.Commands.Hold, False, Beats_Waited, Kept, Eyes, Back);
   end Settle;

   procedure Settle
     (M            : in out Model;
      Back         : Driver.Commands.Command;
      Beats_Waited : out Natural;
      Kept_Going   : out Boolean;
      Kept_Eyes    : out Channel_Flags;
      Came_Back    : out Boolean) is
   begin
      Settle_Body (M, Back, True, Beats_Waited, Kept_Going, Kept_Eyes, Came_Back);
   end Settle;

   procedure Judge
     (M       : Model;
      Targets : Driver.Commands.Command;
      Before  : Count_Array;
      Waited  : Natural;
      Report  : out Step_Report)
   is
      Least : Estimate := (Value => 1.0, Sigma => 0.0, Degrees_Of_Freedom => 0);
      Moved_Some : Boolean := True;
      Blocked_Any : Boolean := False;
   begin
      Report := (others => <>);
      Report.Beats := Waited;
      for G in Before'Range loop
         if Is_Commandable (M, Group_Id (G)) and then Driver.Commands.Has_Target (Targets, Group_Id (G))
           and then Steps.Episodes (M, Group_Id (G)) > Before (G)
         then
            declare
               E : constant Episode := Steps.Latest (M, Group_Id (G));
            begin
               Append (Report.Detail, "group" & G'Image & ": "
                       & (if Known (E.Delivered)
                          then "delivered " & Driver.Log.Image (E.Delivered.Value, 3) & " of "
                          else "delivery not judged, asked ")
                       & Driver.Log.Image (E.Length, 4)
                       & (if E.Blocked then ", blocked" else "")
                       & (if E.Rested then "" else ", given up still moving") & "; ");
               Report.At_Rest := Report.At_Rest and then E.Rested;
               if E.Delivered.Value < Least.Value then
                  Least := E.Delivered;
               end if;
               if E.Blocked then
                  Blocked_Any := True;
                  --  Pushing did not move it at all along the ask.
                  Moved_Some := Moved_Some
                    and then E.Delivered.Value > 0.0
                    and then Driver.Uncertain.Significant (E.Delivered.Value, E.Delivered.Sigma,
                                                           E.Delivered.Degrees_Of_Freedom);
               end if;
            end;
         end if;
      end loop;
      Report.Delivered := Least;
      Report.Outcome := (if not Blocked_Any then Reached elsif Moved_Some then Short else Blocked);
   end Judge;

   --  One sending (Sent) of a step whose targets, as readings, are Targets; waited for and judged (Step). With
   --  Hold_At, the targets are what the targeted groups are held at from now on (Group_Stream.Held): a step's own
   --  targets, not a sending again of them. Where a channel reads its target when the sending has ended, the
   --  command less the reading is the servo's steady error there, kept for the next step's first sending
   --  (Group_Stream.Landing).
   --  The longest a step's push took to end so far (One_Step): a wait twice as long is told in the log as it goes, and at
   --  every doubling after, with each pushed group's push, as Settle tells its own. A fixed eye's capture on the big
   --  machine (k201, a kiwi from its swept body, 10-22) sent a step at beat 2434 that ended at 15289, and another that
   --  had not ended at 39441, with nothing in the log.
   Longest_Step : Natural := 0;

   procedure One_Step
     (M       : in out Model;
      Sent    : Driver.Commands.Command;
      Targets : Driver.Commands.Command;
      Report  : out Step_Report;
      Stop    : access function return Boolean := null;
      Hold_At : Boolean := False;
      Land    : Boolean := False)
   is
      B : Driver.Clock.Beat;
   begin
      Report := (others => <>);
      --  The models are read only between Next and Send.
      Driver.Beats.Next (B);
      declare
         Groups : constant Natural := Group_Count (M);
         --  How many pushes each group had before this step; a push of the
         --  step is the one after them.
         Before : Count_Array (1 .. Groups) := [others => 0];
         Waited : Natural := 0;
         Tell_At : Positive := 2 * Natural'Max (1, Longest_Step);   --  the push's wait at which it is told next
         Started : constant Natural := M.Beats;

         function Targeted (G : Group_Id) return Boolean is
           (Is_Commandable (M, G) and then Driver.Commands.Has_Target (Targets, G));

         Stopped : Boolean := False;   --  Stop held, and every targeted group was held where it was read

         --  Every targeted group held at what it reads at this beat.
         function Held_Here return Driver.Commands.Command is
            C : Driver.Commands.Command := Driver.Commands.Hold;
            O : constant Driver.Beats.Observation_View := Driver.Beats.Latest;
         begin
            for G in 1 .. Groups loop
               if Targeted (Group_Id (G)) and then Driver.Observations.Has_Reading (O.all, Group_Id (G)) then
                  Driver.Commands.Set_Target (C, Group_Id (G), O.Readings.Element (Group_Id (G)));
               end if;
            end loop;
            return C;
         end Held_Here;

         --  Every push this step started is over.
         function Done return Boolean is
         begin
            for G in 1 .. Groups loop
               if Targeted (Group_Id (G)) and then Steps.Episodes (M, Group_Id (G)) > Before (G)
                 and then not Steps.Latest (M, Group_Id (G)).Ended
               then
                  return False;
               end if;
            end loop;
            return True;
         end Done;
      begin
         for G in 1 .. Groups loop
            Before (G) := Steps.Episodes (M, Group_Id (G));
            if Hold_At and then Targeted (Group_Id (G)) then
               declare
                  T : constant Real_Array := Driver.Commands.Target (Targets, Group_Id (G));
               begin
                  M.Groups (Group_Id (G)).Held.Clear;
                  for V of T loop
                     M.Groups (Group_Id (G)).Held.Append (V);
                  end loop;
               end;
            end if;
         end loop;
         M.Stepping := True;
         --  Noted for a replay (Driver.Recording, kind N): no keyframe is taken while a step is under way
         --  (Kinematics.Observe), and a replay that did not know it took keyframes the run had not (RB-Y1 seed 3,
         --  paced round p205s2r replayed, 10-24: its arm 3 first fitted 1100 beats earlier from other keyframes, its
         --  focal 183 px where the run's was 173).
         Driver.Recording.Write_Shared (Driver.Recording.Model_Noted, Driver.Bytes.To_Bytes ("stepping 1"));
         Driver.Beats.Send (Sent);
         --  The push starts at the beat whose observation was taken under the
         --  new targets; it is over when the step tracker has seen it end.
         loop
            Driver.Beats.Next (B);
            Waited := Waited + 1;
            if Waited >= Tell_At then
               declare
                  Text : Unbounded_String;
               begin
                  for G in 1 .. Groups loop
                     if Driver.Commands.Has_Target (Sent, Group_Id (G)) and then Steps.Episodes (M, Group_Id (G)) > 0 then
                        declare
                           E : constant Episode := Steps.Latest (M, Group_Id (G));
                        begin
                           Append (Text, " group" & G'Image & (if E.Ended then " ended" elsif E.Moved then " moving" else " not yet moved")
                                   & " from" & E.Start'Image & ", farther or nearer last at" & E.Progress_At'Image
                                   & ", nearest" & Real'Image (E.Nearest.Value) & ", farthest"
                                   & Real'Image (E.Farthest.Value) & ";");
                        end;
                     end if;
                  end loop;
                  Driver.Log.Line (Driver.Log.Robot, "motion: a step still under way after" & Waited'Image & " beats:"
                                   & To_String (Text));
               end;
               Tell_At := 2 * Tell_At;
            end if;
            declare
               Finished : constant Boolean := Done;
               Halt     : constant Boolean := not Finished and then not Stopped and then Stop /= null and then Stop.all;
            begin
               if Finished then
                  Judge (M, Targets, Before, Waited, Report);
                  Report.Started := Started;
                  Report.Stopped := Stopped;
                  --  The channels that read their target: what was sent for them, less what they read, is the
                  --  servo's steady error there (Group_Stream.Landing); learned from positionings only.
                  if Land and then not Stopped then
                     declare
                        O : constant Driver.Beats.Observation_View := Driver.Beats.Latest;
                     begin
                        for G in 1 .. Groups loop
                           if Targeted (Group_Id (G)) and then Driver.Observations.Has_Reading (O.all, Group_Id (G)) then
                              declare
                                 T : constant Real_Array := Driver.Commands.Target (Targets, Group_Id (G));
                                 C : constant Real_Array := Driver.Commands.Target (Sent, Group_Id (G));
                                 R : constant Real_Array := O.Readings.Element (Group_Id (G));
                                 L : Real_Vectors.Vector renames M.Groups (Group_Id (G)).Landing;
                              begin
                                 while Natural (L.Length) < M.Groups (Group_Id (G)).Size loop
                                    L.Append (0.0);
                                 end loop;
                                 for K in 1 .. Natural'Min (T'Length, Natural'Min (C'Length, R'Length)) loop
                                    declare
                                       Noise : constant Real := Channels.Noise (M, Group_Id (G), K);
                                       Error : constant Real := T (T'First + K - 1) - R (R'First + K - 1);
                                    begin
                                       if Noise < Real'Last and then Noise > 0.0
                                         and then not Driver.Uncertain.Significant
                                                        (Error, Noise * Ada.Numerics.Long_Elementary_Functions.Sqrt (2.0))
                                       then
                                          L.Replace_Element (K - 1, C (C'First + K - 1) - R (R'First + K - 1));
                                       end if;
                                    end;
                                 end loop;
                              end;
                           end if;
                        end loop;
                     end;
                  end if;
               end if;
               if Halt then
                  Stopped := True;
                  Driver.Beats.Send (Held_Here);
               else
                  Driver.Beats.Send (Driver.Commands.Hold);
               end if;
               exit when Finished;
            end;
         end loop;
         M.Stepping := False;
         Driver.Recording.Write_Shared (Driver.Recording.Model_Noted, Driver.Bytes.To_Bytes ("stepping 0"));
         Longest_Step := Natural'Max (Longest_Step, Waited);
      end;
   end One_Step;

   procedure Step
     (M       : in out Model;
      Targets : Driver.Commands.Command;
      Report  : out Step_Report;
      Stop    : access function return Boolean := null;
      Land    : Boolean := False)
   is
      Groups : constant Natural := Group_Count (M);
      Worst  : Real := Real'Last;   --  the largest error of the last landing, in deviations of what the readings tell

      --  The first sending: every targeted channel's target plus the servo's steady error where the group last
      --  landed on a target (Group_Stream.Landing), so that it lands on this one at the first sending where the
      --  error is the same (R5 on the Pi, 10-14: 4 to 17 units, nearly the same at every landing; each sending
      --  again is a step's wait).
      function With_Landing return Driver.Commands.Command is
         C : Driver.Commands.Command := Targets;
      begin
         for G in 1 .. Groups loop
            if Is_Commandable (M, Group_Id (G)) and then Driver.Commands.Has_Target (Targets, Group_Id (G))
              and then Natural (M.Groups (Group_Id (G)).Landing.Length) = M.Groups (Group_Id (G)).Size
            then
               declare
                  T : Real_Array := Driver.Commands.Target (Targets, Group_Id (G));
               begin
                  for K in T'Range loop
                     T (K) := T (K) + M.Groups (Group_Id (G)).Landing (K - T'First);
                  end loop;
                  Driver.Commands.Set_Target (C, Group_Id (G), T);
               end;
            end if;
         end loop;
         return C;
      end With_Landing;
      --  Where every group's readings stood when the step was sent: no sending again asks a channel farther than
      --  the step itself asked it from there. The bound used to be the last push's length over all its channels,
      --  and a step that brought the elbow back 157 units while asking the wrist 19.6 let the wrist's sendings
      --  again ask 27, 46 and 70 units of a joint that was not following, until it stood 161 units off (R6's
      --  thirteenth start, 10-14: the camera dropped out of its view for the rest of the sweep).
      Began : array (1 .. Groups) of Real_Vectors.Vector;
      procedure Read_Began is
         O : constant Driver.Beats.Observation_View := Driver.Beats.Latest;
      begin
         for G in 1 .. Groups loop
            Began (G).Clear;
            if Driver.Observations.Has_Reading (O.all, Group_Id (G)) then
               for V of O.Readings.Element (Group_Id (G)) loop
                  Began (G).Append (V);
               end loop;
            end if;
         end loop;
      end Read_Began;
   begin
      if not Land then
         One_Step (M, Targets, Targets, Report, Stop, Hold_At => True);
         return;
      end if;
      Read_Began;
      One_Step (M, With_Landing, Targets, Report, Stop, Hold_At => True, Land => True);
      --  Landing where it was sent: a servo that holds a load with a proportional controller settles a few
      --  units short of its goal, the same few units at the same pose (the kiwi's arms, R4 on the Pi, 10-13:
      --  back from every push 4 to 7 units off the hold, so that no keyframe of the recognition turned one joint
      --  alone and the fit had no joint's own keyframe). Where a targeted channel reads off its target by more
      --  than the noise of a difference of two readings tells, the target is sent again moved by that error, as
      --  long as each landing comes nearer than the one before: what does not come nearer is not the servo's
      --  steady error (a joint on what stops it), and what was reached stands. The error is asked over the share
      --  of the last sending the group delivered (R5 on the Pi: the servos deliver about a third of a small
      --  change, and an error asked as it is took eight sendings to close; each sending is a step's wait).
      loop
         exit when Report.Outcome /= Reached or else Report.Stopped;
         declare
            O       : constant Driver.Beats.Observation_View := Driver.Beats.Latest;
            Nudged  : Driver.Commands.Command := Driver.Commands.Hold;
            Largest : Real := 0.0;
            Any     : Boolean := False;
            Off_By  : Ada.Strings.Unbounded.Unbounded_String;   --  the errors, per group and channel, for the log
         begin
            for G in 1 .. Groups loop
               if Is_Commandable (M, Group_Id (G)) and then Driver.Commands.Has_Target (Targets, Group_Id (G))
                 and then Driver.Observations.Has_Reading (O.all, Group_Id (G))
               then
                  declare
                     Target  : constant Real_Array := Driver.Commands.Target (Targets, Group_Id (G));
                     Reading : constant Real_Array := O.Readings.Element (Group_Id (G));
                     Goal    : Real_Array := Target;
                     Off     : Boolean := False;
                     --  What the group delivered of the last sending: a servo that moves a fraction of a small
                     --  change asked of it (the kiwi's, R5 on the Pi, 10-13: 0.33 to 0.38 of 6 to 27 units) is
                     --  asked for the error over that fraction, so that the fraction it delivers is the error.
                     --  All of it when the fraction is not measured, or is not below one.
                     Share : Real := 1.0;
                     --  How far the last sending asked the group to move, in all: no sending again asks a channel
                     --  farther than that (R5 on the Pi, 10-13: a push of 314 units stopped 525 short against
                     --  the head camera, was judged reached, and the sending again asked the error over a share
                     --  of 0.16, then of 0.01: 3727 and 216445 units, the arm thrown against what stopped it).
                     Farthest : Real := Real'Last;
                     Stopped  : Boolean := False;   --  the last sending ended against something: no sending again
                     Errors : Ada.Strings.Unbounded.Unbounded_String;
                     --  The least turn of channel C that any arm of the group counts (Kinematics.Counted_Turn): a
                     --  landing of an arm's joint off its target by less than that is no turn of it to the fit, which
                     --  takes a keyframe's joints from their readings and counts a joint as turned only from that turn,
                     --  so it changes nothing the sweep's keyframes are for and is not sent again (the eye's visible
                     --  step alone let it be where the lock-in tells sub-pixel motion: a25, 10-16, a simulated arm in
                     --  noise-free pictures, sent 172 landings again in its second pass, 663 of its 2186 beats, each
                     --  error under a twentieth of a unit, where the matcher's steps were 0.04 to 0.28 units and the
                     --  lock-in's 0.001 to 0.009); a group several arms share (a humanoid's torso is the joints of
                     --  every arm) lands a joint within none of their turns.
                     function Least_Counted (C : Positive) return Real is
                        Result : Real := 0.0;
                     begin
                        for Other in 1 .. Arm_Count (M) loop
                           if Natural (Arm_Group (M, Arm_Id (Other))) = G then
                              declare
                                 T : constant Real := Kinematics.Counted_Turn (M, Arm_Id (Other), C);
                              begin
                                 if T > 0.0 and then (Result = 0.0 or else T < Result) then
                                    Result := T;
                                 end if;
                              end;
                           end if;
                        end loop;
                        return Result;
                     end Least_Counted;
                     --  A landing the body cannot tell from its target is where it was sent: its error as a change
                     --  of the readings is no motion by the body's one test (Channels.Visible), an eye seeing none
                     --  of it. Judged against the readings' noise alone, a simulator's arm that repeats its readings
                     --  to the last digit was off by millions of deviations at every landing, 0.00001 units no eye
                     --  saw, and 309 steps of b01's boot (10-16) were sent again, seven beats each: two fifths of
                     --  its sweep.
                     Seen : constant Boolean :=
                       Target'Length /= Group_Size (M, Group_Id (G)) or else Reading'Length /= Target'Length
                       or else Channels.Visible
                                 (M, Group_Id (G),
                                  [for C in 1 .. Target'Length =>
                                     Target (Target'First + C - 1) - Reading (Reading'First + C - 1)]);
                  begin
                     if Steps.Episodes (M, Group_Id (G)) > 0 then
                        declare
                           E : constant Episode := Steps.Latest (M, Group_Id (G));
                           D : constant Estimate := E.Delivered;
                        begin
                           Stopped := E.Blocked;
                           Farthest := E.Length;
                           if Known (D) and then D.Value > 0.0 and then D.Value < 1.0
                             and then Driver.Uncertain.Significant (D.Value, D.Sigma, D.Degrees_Of_Freedom)
                           then
                              Share := D.Value;
                           end if;
                        end;
                     end if;
                     for C in 1 .. (if Seen then Natural'Min (Target'Length, Reading'Length) else 0) loop
                        declare
                           Noise : constant Real := Channels.Noise (M, Group_Id (G), C);
                           Error : constant Real := Target (Target'First + C - 1) - Reading (Reading'First + C - 1);
                           --  An error within the channel's dead band (Steps.Dead_Band) is not sent again: the body
                           --  stands still for an ask within its dead band, and sending
                           --  it bounced the kiwi's joints two or three units either side of their targets, a step
                           --  sent again two to four times each, 340 times in a boot whose sweep then held still
                           --  for no keyframe (R6's eighth start, 10-14).
                           Least : constant Real := Steps.Dead_Band (M, Group_Id (G), C);
                           Counted : constant Real := Least_Counted (C);
                        begin
                           if not Stopped and then Noise < Real'Last and then Noise > 0.0
                             and then Driver.Uncertain.Significant (Error, Noise * Sqrt (2.0))
                             and then (Least = Real'Last or else abs Error > Least)
                             and then abs Error >= Counted
                           then
                              Off := True;
                              Largest := Real'Max (Largest, abs Error / (Noise * Sqrt (2.0)));
                              declare
                                 Asked : constant Real := Error / Share;
                                 --  This channel's own ask in the step, where its readings were read when it was sent;
                                 --  the last push's length over every channel otherwise.
                                 Own   : constant Real :=
                                   (if C - 1 < Natural (Began (G).Length)
                                    then abs (Target (Target'First + C - 1) - Began (G) (C - 1)) else Farthest);
                                 Most  : constant Real := Real'Min (abs Asked, Real'Min (Own, Farthest));
                              begin
                                 Goal (Goal'First + C - 1) :=
                                   Target (Target'First + C - 1) + (if Asked < 0.0 then -Most else Most);
                              end;
                              Append (Errors, " c" & C'Image & " " & Driver.Log.Image (Error, 1));
                           end if;
                        end;
                     end loop;
                     if Off then
                        Any := True;
                        Driver.Commands.Set_Target (Nudged, Group_Id (G), Goal);
                        Append (Off_By, " group" & G'Image & ":" & To_String (Errors) & " over the share "
                                & Driver.Log.Image (Share, 2) & ";");
                     end if;
                  end;
               end if;
            end loop;
            exit when not Any or else Largest >= Worst;
            Worst := Largest;
            declare
               Again : Step_Report;
            begin
               One_Step (M, Nudged, Targets, Again, Stop, Land => True);
               Driver.Log.Line (Driver.Log.Robot, "motion: a step sent again by its landing's error of "
                                & Driver.Log.Image (Largest, 1) & " deviations of what the readings tell (" & To_String (Off_By)
                                & "), landed in" & Again.Beats'Image & " beats: " & To_String (Again.Detail));
               Report.Beats := Report.Beats + Again.Beats;
               Report.At_Rest := Again.At_Rest;
               Report.Stopped := Again.Stopped;
               Append (Report.Detail, "sent again by the landing's error (" & Driver.Log.Image (Largest, 1)
                       & " deviations): " & To_String (Again.Detail));
               --  A sending again that meets something changes nothing of what the step reached: the joint stands
               --  where what stops it leaves it.
               exit when Again.Outcome /= Reached;
               --  Nor does one the body did not execute: no group it asked delivered a significant share of the ask.
               --  Each landing came nearer than the one before by four of 1.7e9 deviations, the arm creeping on what
               --  held it, and the step was sent again every six beats for 1700 beats, delivering 0.000 of 2.02 units
               --  each time (f15m on 10-15, readings exact to 1e-9 units).
               declare
                  Executed : Boolean := False;
               begin
                  for G in 1 .. Groups loop
                     if Driver.Commands.Has_Target (Nudged, Group_Id (G)) and then Steps.Episodes (M, Group_Id (G)) > 0 then
                        declare
                           D : constant Estimate := Steps.Latest (M, Group_Id (G)).Delivered;
                        begin
                           if Known (D) and then D.Value > 0.0
                             and then Driver.Uncertain.Significant (D.Value, D.Sigma, D.Degrees_Of_Freedom)
                           then
                              Executed := True;
                           end if;
                        end;
                     end if;
                  end loop;
                  exit when not Executed;
               end;
            end;
         end;
      end loop;
   end Step;

   procedure Note_Stopped (M : in out Model; G : Group_Id; Noted : out Boolean) is
   begin
      Noted := False;
      if Steps.Episodes (M, G) = 0 then
         return;
      end if;
      declare
         E : constant Episode := Steps.Latest (M, G);
         C : constant Natural := Steps.Limiter (M, G, E);
      begin
         if C = 0 then
            return;
         end if;
         declare
            S     : Group_Stream renames M.Groups (G);
            Start : constant Real := Channels.Reading (M, G, E.Start - 1, C);
            Stop  : constant Real := Channels.Reading (M, G, E.End_At, C);
            Asked : constant Real := Channels.Target (M, G, E.Start, C);
            Up    : constant Boolean := Asked > Start;
            At_C  : constant Natural := C - 1;
            Pose  : Real_Vectors.Vector;

            --  How close two readings of channel I must be to be one reading to this test. Two stops of a channel
            --  that are two independent contacts fall anywhere in the channel's travel, and meet within a distance
            --  D of one another with the probability 2 D over the travel; D is the distance at which that is no more
            --  than the false alarm rate of Z, and no less than Z deviations of the difference of two readings (the
            --  readings' noise, which is the whole of it where the readings are as coarse as a real robot's). The
            --  controller leaves a stop a few thousandths of a radian from one push to the next (A22 and A27's third
            --  joint: -0.0359, -0.0368, -0.0359), far above the noise of an exact simulator.
            function Tolerance (I : Natural) return Real is
              (if Channels.Noise (M, G, I + 1) < Real'Last
               then Real'Max (Driver.Conventions.Z * Sqrt (2.0) * Channels.Noise (M, G, I + 1),
                              Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z)
                              * (S.High_Seen (I) - S.Low_Seen (I)) / 2.0)
               else Real'Last);
            function Agree (I : Natural; A, B : Real) return Boolean is
              (Tolerance (I) < Real'Last and then abs (A - B) <= Tolerance (I));
            function Apart (I : Natural; A, B : Real) return Boolean is
              (Tolerance (I) < Real'Last and then abs (A - B) > Tolerance (I));

            --  A stop made at another pose of the group: some other channel stood somewhere else.
            function Another_Pose (Other : Real_Vectors.Vector) return Boolean is
              (Natural (Other.Length) = S.Size
               and then (for some I in 0 .. S.Size - 1 => I /= At_C and then Apart (I, Other (I), Pose (I))));

            Where : constant String :=
              "body: group" & G'Image & " channel" & C'Image & " stopped at " & Driver.Log.Image (Stop, 6) & " asked "
              & Driver.Log.Image (Asked, 6) & " from " & Driver.Log.Image (Start, 6) & ", the push that began at beat"
              & E.Start'Image;
            Way : constant String := (if Up then "upwards" else "downwards");
         begin
            for I in 0 .. S.Size - 1 loop
               Pose.Append (Channels.Reading (M, G, E.End_At, I + 1));
            end loop;
            --  The vectors are laid out for the group's channels the first time a stop is noted.
            if Natural (S.Has_Stopped_Low.Length) /= S.Size then
               S.Has_Stopped_Low.Clear;
               S.Has_Stopped_High.Clear;
               S.Stopped_Low.Clear;
               S.Stopped_High.Clear;
               for I in 1 .. S.Size loop
                  S.Has_Stopped_Low.Append (False);
                  S.Has_Stopped_High.Append (False);
                  S.Stopped_Low.Append (0.0);
                  S.Stopped_High.Append (0.0);
               end loop;
            end if;
            if (if Up then S.Has_Stopped_High (At_C) else S.Has_Stopped_Low (At_C)) then
               --  The end the channel has explains it; a stop beyond it is read as a reading, which widens the end, and
               --  said, so that a far end hit after a near false one is not a quiet success in the log.
               declare
                  Found_At : constant Real := (if Up then S.Stopped_High (At_C) else S.Stopped_Low (At_C));
               begin
                  if (if Up then Stop > Found_At else Stop < Found_At) and then Apart (At_C, Stop, Found_At) then
                     Driver.Log.Line (Driver.Log.Robot, Where & ": beyond the end found at " & Driver.Log.Image (Found_At, 6)
                                      & ": the reading widens it");
                  else
                     Driver.Log.Line (Driver.Log.Robot, Where & ": at the end the channel has");
                  end if;
               end;
            else
               declare
                  Found  : Boolean := False;   --  the same reading found again from another pose
                  Twice  : Boolean := False;   --  the same reading from the same pose: the same contact
                  Value  : Real := Stop;
                  Before : Real := Stop;
               begin
                  for Old of S.Stops loop
                     if Old.Channel = C and then Old.Up = Up and then Agree (At_C, Old.Value, Stop) then
                        if Another_Pose (Old.Pose) then
                           Found := True;
                           Before := Old.Value;
                           Value := (if Up then Real'Max (Value, Old.Value) else Real'Min (Value, Old.Value));
                        else
                           Twice := True;
                        end if;
                     end if;
                  end loop;
                  if Found then
                     declare
                        K : Natural := S.Stops.First_Index;
                     begin
                        while K <= S.Stops.Last_Index loop
                           if S.Stops (K).Channel = C and then S.Stops (K).Up = Up
                             and then Agree (At_C, S.Stops (K).Value, Stop)
                           then
                              S.Stops.Delete (K);
                           else
                              K := K + 1;
                           end if;
                        end loop;
                     end;
                     if Up then
                        S.Has_Stopped_High.Replace_Element (At_C, True);
                        S.Stopped_High.Replace_Element (At_C, Value);
                     else
                        S.Has_Stopped_Low.Replace_Element (At_C, True);
                        S.Stopped_Low.Replace_Element (At_C, Value);
                     end if;
                     Noted := True;
                     Driver.Log.Line (Driver.Log.Robot, Where & ": found again from another pose (" & Driver.Log.Image (Before, 6)
                                      & " before): an end of the channel " & Way);
                  elsif Twice then
                     Driver.Log.Line (Driver.Log.Robot, Where & ": the same reading from the same pose as before, the same contact");
                  elsif Channels.Noise (M, G, C) < Real'Last then
                     S.Stops.Append (Stop_Record'(Channel => C, Up => Up, Value => Stop, Pose => Pose));
                     Driver.Log.Line (Driver.Log.Robot, Where & ": noted; it is an end of the channel " & Way
                                      & " when the channel stops at this reading from another pose");
                  end if;
               end;
            end if;
         end;
      end;
   end Note_Stopped;

   procedure Note_Stopped (M : in out Model; G : Group_Id) is
      Ignored : Boolean;
   begin
      Note_Stopped (M, G, Ignored);
   end Note_Stopped;

   function Least_Turn (M : Model; A : Arm_Id; Channel : Positive) return Real is
      Step     : constant Real := Kinematics.Step_Of (M, A, Channel);
      Turn     : constant Real := Kinematics.Counted_Turn (M, A, Channel);
      Noise    : constant Real := Channels.Noise (M, Arm_Group (M, A), Channel);
      Executed : constant Real := Steps.Dead_Band (M, Arm_Group (M, A), Channel);
      --  The turn the fit counts, and what the readings' noise can take off
      --  a turn asked of exactly it (a keyframe counts as a joint's own, and
      --  is kept at all, from the step on, and a turn of the step itself
      --  lands below it half the time: fake body V6, arm 1, the first joint's
      --  turns of its step, 38 units, left no keyframe through eleven rounds,
      --  and the fit said the joint had none of its own).
      Counted  : constant Real :=
        (if Noise < Real'Last then Turn + Driver.Conventions.Z * Sqrt (2.0) * Noise else Turn);
   begin
      if Step <= 0.0 then
         return 0.0;
      end if;
      --  And the channel's dead band (Steps.Dead_Band) on top: a turn asked within a
      --  servo's dead band moves nothing, and a keyframe needs the joint to have turned (the kiwi's sweep, R5
      --  on 10-14: first levels of 2.4 units, every one called blocked where it stood); and a turn that does
      --  execute lands short by up to that dead band, so a level of the counted step alone lands within the
      --  step of the pose it left as often as not and leaves no keyframe (R6's eleventh start, 10-14: first
      --  levels of 4.9 units at a dead band of 4.9, the first pass of arm_a without one keyframe, its pushes
      --  blocked or too small to count). And the fit counts a turn from the reference, itself a landing within
      --  the dead band of where it was sent: the level is the counted turn plus what the two landings can lose,
      --  the dead band each. s07's cells (10-16), from levels of the counted step plus one band, landed 16 units
      --  from the reference where the fit counted from 19.6 (a band of 9.8), five joints of six counted none,
      --  no keyframe moved two joints at once, and the arm was not fitted.
      return (if Executed < Real'Last then Counted + 2.0 * Executed else Counted);
   end Least_Turn;

   function Sweep_Start (M : Model; A : Arm_Id; Channel : Positive) return Real is
      Step  : constant Real := Kinematics.Step_Of (M, A, Channel);
      Least : constant Real := Least_Turn (M, A, Channel);
   begin
      --  The first level is the largest turn the aligner follows by itself,
      --  half a patch of view (Kinematics.Follow_Step), and never less than
      --  the least turn a keyframe counts. The ladder doubles from there: its
      --  next level is about a patch, the one after it is followed only by a
      --  fitted arm's prediction. From the least turn, a doubling a round and
      --  a fit a round, the x5's eye (no pixel noise, a step of 1e-4 reading
      --  units) climbed four levels in 1100 beats to turns of 0.0013 units,
      --  and every fit of them was undetermined (B0a_1, 10-13). Before any
      --  keyframe has turned the joint alone, how far the view moves per unit
      --  is not measured (Kinematics.Pixels_Per_Unit) and the first level is
      --  the least turn: its keyframe, predicted from nothing, is what
      --  measures it, and the ladder climbs from there.
      return (if Step > 0.0 then Real'Max (Least, Kinematics.Follow_Step (M, A, Channel)) else Step);
   end Sweep_Start;

   procedure Hold_For_Keyframe (M : in out Model; A : Arm_Id) is
      B      : Driver.Clock.Beat;
      Done   : Boolean;
      Cause  : Ada.Strings.Unbounded.Unbounded_String;   --  what stands in the way now
      Detail : Ada.Strings.Unbounded.Unbounded_String;
      Said   : Ada.Strings.Unbounded.Unbounded_String;   --  every cause the log has said stood in the way

      function Carries_An_Eye return Boolean is
        (for some E in 1 .. Eye_Count (M) =>
           Eye_Mount (M, Eye_Id (E)).Kind = Arm_Carried and then Eye_Mount (M, Eye_Id (E)).Arm = A);
   begin
      loop
         Driver.Beats.Next (B);
         Done := not Carries_An_Eye or else (M.Beats > 0 and then Kinematics.Held_Still (M, A, M.Beats - 1));
         --  A wait says what it waits for, each cause once: the drone's first sweep held for a keyframe 2300 beats
         --  and more with nothing in the log (m1020_d5, 10-20).
         if not Done then
            Kinematics.Still_Blocked_By (M, A, Cause, Detail);
            if Ada.Strings.Unbounded.Index (Said, Ada.Strings.Unbounded.To_String (Cause) & "|") = 0 then
               Ada.Strings.Unbounded.Append (Said, Ada.Strings.Unbounded.To_String (Cause) & "|");
               Driver.Log.Line (Driver.Log.Robot, "motion: arm" & A'Image & " is not held still for a keyframe:"
                                & Ada.Strings.Unbounded.To_String (Detail));
            end if;
         end if;
         Driver.Beats.Send (Driver.Commands.Hold);
         exit when Done;
      end loop;
   end Hold_For_Keyframe;

   procedure Hold_For_Twin (M : in out Model; A : Arm_Id) is
      B      : Driver.Clock.Beat;
      Done   : Boolean;
      Cause  : Ada.Strings.Unbounded.Unbounded_String;   --  what stands in the way now
      Detail : Ada.Strings.Unbounded.Unbounded_String;
      Said   : Ada.Strings.Unbounded.Unbounded_String;   --  every cause the log has said stood in the way

      function Carries_An_Eye return Boolean is
        (for some E in 1 .. Eye_Count (M) =>
           Eye_Mount (M, Eye_Id (E)).Kind = Arm_Carried and then Eye_Mount (M, Eye_Id (E)).Arm = A);
   begin
      loop
         Driver.Beats.Next (B);
         Done := not Carries_An_Eye or else Kinematics.Twin_Answered (M, A);
         --  A wait says what it waits for, each time that changes: the twin waited 2200 to 2800 beats on the drone and
         --  more than 7000 on the humanoid (m1020_h7, 10-20) with nothing in the log.
         if not Done then
            Kinematics.Twin_Blocked_By (M, A, Cause, Detail);
            if Ada.Strings.Unbounded.Index (Said, Ada.Strings.Unbounded.To_String (Cause) & "|") = 0 then
               Ada.Strings.Unbounded.Append (Said, Ada.Strings.Unbounded.To_String (Cause) & "|");
               Driver.Log.Line (Driver.Log.Robot, "motion: arm" & A'Image & "'s reference has no still twin yet: "
                                & Ada.Strings.Unbounded.To_String (Detail));
            end if;
         end if;
         Driver.Beats.Send (Driver.Commands.Hold);
         exit when Done;
      end loop;
   end Hold_For_Twin;

   procedure Hold (M : in out Model; Beats : Positive) is
      pragma Unreferenced (M);
      B : Driver.Clock.Beat;
   begin
      for K in 1 .. Beats loop
         Driver.Beats.Next (B);
         Driver.Beats.Send (Driver.Commands.Hold);
      end loop;
   end Hold;

   procedure Hold_And_Estimate (M : in out Model) is
      B : Driver.Clock.Beat;
   begin
      Driver.Beats.Next (B);
      Estimate_Now (M);
      Driver.Beats.Send (Driver.Commands.Hold);
   end Hold_And_Estimate;

   function Hold_Of (M : Model; G : Group_Id; Channel : Positive) return Real is
      Last : constant Natural := (if M.Beats > 0 then M.Beats - 1 else 0);
   begin
      --  A channel that keeps the body going while held is held at its rest command: a speed held anywhere else
      --  moves the body on (round 0, 10-23: held at its last reading, the Go2 walked 34 to 39 m, the drone flew up to
      --  365 m away, the G1 walked 7 m and fell).
      if Keeps_Going (M, G, Channel) then
         return Rest_Command (M, G, Channel);
      end if;
      --  The pose a step last sent the group to, as a reading (Group_Stream.Held): the command that holds it there
      --  may stand off it by the servo's steady error (Step sends a step again by its landing's error), and a hold
      --  read from the command would creep by that error at every step. Before any step, the last target (a group
      --  never commanded is held at its first reading) or the reading.
      if Natural (M.Groups (G).Held.Length) = M.Groups (G).Size and then Channel <= M.Groups (G).Size then
         return M.Groups (G).Held (Channel - 1);
      end if;
      return (if Channels.Has_Target (M, G, Last) then Channels.Target (M, G, Last, Channel)
              else Channels.Reading (M, G, Last, Channel));
   end Hold_Of;

   function Ln (X : Real) return Real renames Ada.Numerics.Long_Elementary_Functions.Log;

   package Real_IO is new Ada.Text_IO.Float_IO (Real);

   --  A number in the log with its magnitude: probe sizes span many decades (as Boot writes them).
   function Scientific (X : Real) return String is
      S : String (1 .. 32);
   begin
      Real_IO.Put (S, X, Aft => 2, Exp => 3);
      return Ada.Strings.Fixed.Trim (S, Ada.Strings.Both);
   end Scientific;

   --  Readings for a log line, each to the six places the group lines give them (Driver.Robot.Group_Watch).
   function Readings_Image (Q : Real_Array) return String is
     (if Q'Length = 0 then "" else Readings_Image (Q (Q'First .. Q'Last - 1)) & " " & Driver.Log.Image (Q (Q'Last), 6));

   --  The probes may try as many levels as a float has bits of precision.
   Levels : constant Positive := Real'Machine_Mantissa;

   function Tail return Real is (Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z));

   --  Some eye's image moved at a beat from From on, and the chance that a look at that many verdicts finds one as
   --  rare at rest by noise alone: the verdict the rarest at rest of those looked at is as rare as its share of the
   --  rest's counts that reach it (Lockin.Rest_Share: at most (k + 1) / (n + 1) of n counts at rest, k of them as
   --  high, whatever their distribution), and any of the look's verdicts is that rare by chance with a chance at most
   --  that share times their number. A look used to charge every verdict the chance of a count at rest passing the
   --  level a verdict asks (Rest_Count_Above of Rest_Count_Beats), however far above it the count of a beat stood: the
   --  G1's looks of two beats over three eyes, each of which saw its arm move, were charged 0.636 each, and confirming
   --  the level took 22 moves back and forth (round 4a, 10-23: 128 of its 1200 beats for one group).
   procedure Look (M : Model; From : Natural; Seen : out Boolean; Chance : out Real) is
      Verdicts : Natural := 0;
      Rarest   : Real := 1.0;
   begin
      Seen := False;
      for E in 1 .. Eye_Count (M) loop
         for B in From .. M.Beats - 1 loop
            Verdicts := Verdicts + 1;
            Rarest := Real'Min (Rarest, Lockin.Rest_Share (M, Eye_Id (E), B));
            if Lockin.Moved (M, Eye_Id (E), B) then
               Seen := True;
            end if;
         end loop;
      end loop;
      --  The chance that at least one of the verdicts looked at is that rare by chance: one less the chance that none
      --  is, the verdicts independent. Summed instead, a look of twenty beats over three eyes with rest counts of
      --  53 beats "had a chance" of 1.1, above one, and no run of moves could make such a
      --  level certain: the ladder doubled past every level an eye saw, to 56 radians and the arm's ends (seed 2
      --  of the fake robot, 10-14: levels 10 to 21 all seen, none confirmed).
      Chance := 1.0 - Ada.Numerics.Long_Elementary_Functions.Exp (-Real (Verdicts) * Rarest);
   end Look;

   --  The smallest change of the channel's reading that tells from its noise
   --  (a change is the difference of two readings, the noise taken as measured:
   --  Channels.Significant_Change), and no less than one step of the reading
   --  (Channels.Step_Unit); without jitter, the latter.
   function Smallest_Step (M : Model; G : Group_Id; Channel : Positive) return Real is
      Sigma : constant Real := Reading_Noise (M, G, Channel);
      Wire  : constant Real := Channels.Step_Unit (M, G, M.Beats - 1, Channel);
   begin
      if Sigma > 0.0 and then Sigma < Real'Last then
         return Real'Max
           (Wire, Driver.Uncertain.Threshold (Driver.Uncertain.Scalar_Gate) * Sigma * Sqrt (2.0));
      end if;
      return Wire;
   end Smallest_Step;

   function Group_Least_Step (M : Model; G : Group_Id) return Real is
      Least : Real := 0.0;
   begin
      if Natural (G) <= Group_Count (M) and then M.Beats > 0 and then Channels.Has_Reading (M, G, M.Beats - 1) then
         for C in 1 .. Group_Size (M, G) loop
            Least := Real'Max (Least, Smallest_Step (M, G, C));
         end loop;
      end if;
      return Least;
   end Group_Least_Step;

   type Flag_Array is array (Positive range <>) of Boolean;

   --  Moves the listed channels to Base plus the offsets and looks for as
   --  long as a response takes to show in the readings and then in the
   --  images: whether some eye saw motion, the chance that a look at so many
   --  verdicts finds one by noise, and each reading once the readings are
   --  still. Unlooked is the first beat no look has judged: the looks tile
   --  the probe, so a response that shows later than expected falls in the
   --  next one.
   procedure Move_And_Look
     (M        : in out Model;
      Listed   : Channel_Refs;
      Usable   : Flag_Array;
      Base     : Driver.Commands.Command;
      Offsets  : Real_Array;
      Unlooked : in out Natural;
      Seen     : out Boolean;
      Chance   : out Real;
      Now      : out Real_Array;
      Moving   : out Flag_Array;
      Read_At  : out Natural)
   is
      N      : constant Natural := Listed'Length;
      B      : Driver.Clock.Beat;
      C      : Driver.Commands.Command := Base;
      From   : Natural;
      Looked : Natural := 0;
      --  How many pushes each listed group had before this move; a push of the move is one after them.
      Before : Count_Array (1 .. N) := [others => 0];

      --  Some listed group's push of this move was answered (its readings moved) and is still under way (Steps
      --  follows it to its end: its readings still, or plainly going nowhere). A push nothing answered is not
      --  waited for: before the body's delay is measured such a push waits as many beats as the stream had
      --  before it (Steps: no wait is longer than all the waiting so far), and the ladder's first levels, one
      --  tick each under a servo's deadband, stood 99, 200, 400 and 750 beats in a row (R3 on the Pi, 10-13:
      --  the "rest" that took a minute); the next level cuts such a push short, as it always did.
      --  A push under way: begun and not ended, whether or not its readings have moved yet. An eye sees the move
      --  a beat before the readings do (its lag, the servo's delay), and a look that ends at that beat read the
      --  readings where they stood: the level counted as not executed, and the ladder doubled its ask twenty-four
      --  times, the arm swinging between its range's ends (R6's fourth start on the kiwi, 10-14, arm_b's first
      --  joint, once the eye's threshold for motion came down).
      --  A push nothing answered and no eye saw is not waited for: before the body's delay is measured, Steps
      --  gives such a push up only after as many beats as the stream has had, and the first probe's levels below
      --  the dead band would each wait that long (R6's fifth start, 10-14: 500 beats into the first probe).
      --  And a push whose readings have not moved is waited for while an eye still sees motion at the latest beat:
      --  a look's sighting is one verdict of the many a look takes, each a false alarm by chance (False_Alarm), and
      --  one sighting in a look made a push be waited for as long as Steps gives an unanswered one. s43, s46 and s47
      --  (servo seed 6 of the fake robot, 10-17) all waited from beat 79 to 118 and from 167 to 206 after looks whose
      --  chance of a false alarm was 0.19 and 0.99, at levels of 2.45 and 4.9 units that no channel went along.
      Looked_To : Natural := 0;   --  the last beat a look of this move judged
      Seen_Any : Boolean := False;
      function Seen_Now return Boolean is
        (M.Beats > 0 and then (for some E in 1 .. Eye_Count (M) => Lockin.Moved (M, Eye_Id (E), M.Beats - 1)));
      function Still_Pushing return Boolean is
        (for some I in 1 .. N =>
           Usable (I) and then Steps.Episodes (M, Listed (I).Group) > Before (I)
           and then not Steps.Latest (M, Listed (I).Group).Ended
           and then ((Seen_Any and then Seen_Now) or else Steps.Latest (M, Listed (I).Group).Moved));

      --  Each listed channel's reading at the last beat, and whether its group's readings still move there.
      procedure Read_Ends is
      begin
         Read_At := Natural'Max (1, M.Beats) - 1;
         for I in 1 .. N loop
            Now (I) := 0.0;
            Moving (I) := False;
            if Usable (I) and then Channels.Has_Reading (M, Listed (I).Group, M.Beats - 1) then
               Now (I) := Channels.Reading (M, Listed (I).Group, M.Beats - 1, Listed (I).Channel);
               Moving (I) := Channels.Moving (M, Listed (I).Group, M.Beats - 1);
            end if;
         end loop;
      end Read_Ends;
   begin
      Read_At := 0;
      for I in 1 .. N loop
         if Usable (I) then
            declare
               G : constant Group_Id := Listed (I).Group;
               T : Real_Array := Driver.Commands.Target (C, G);
               K : constant Positive := T'First + Listed (I).Channel - 1;
            begin
               T (K) := Driver.Commands.Target (Base, G) (K) + Offsets (I);
               Driver.Commands.Set_Target (C, G, T);
            end;
         end if;
      end loop;
      Driver.Beats.Next (B);
      From := Unlooked;
      for I in 1 .. N loop
         if Usable (I) then
            Before (I) := Steps.Episodes (M, Listed (I).Group);
         end if;
      end loop;
      Driver.Beats.Send (C);
      loop
         Driver.Beats.Next (B);
         Looked := Looked + 1;
         declare
            Wait : Natural := 1;
            Delay_Beats : Natural := 0;
            Delay_Known : Boolean;
            Earliest : Natural := Natural'Last;
            Lag : Natural := 0;
            Done : Boolean;
         begin
            --  How long the readings may take to answer: the body's delay where any push of it was answered
            --  (Steps.Body_Delay: a command travels one path to every group). Before one was, this move's own
            --  answer where its readings have moved, else as long as Steps gives an unanswered push before any
            --  was answered (First_Push): read a beat after it was sent, a body that answers a beat later shows
            --  nothing yet, the level is judged not executed, the next level is sent into the answer of this
            --  one, and no push is ever clean enough to measure the delay from (seed 2 of the fake robot, 10-14:
            --  every channel "stopped following" at the third level, the boot ended with nothing seen; seed 1's
            --  ladder doubled 33 times before an eye saw).
            Steps.Body_Delay (M, Delay_Beats, Delay_Known);
            --  The earliest beat a listed group's readings can answer: the least of their own measured delays
            --  (a closer answering in five beats beside arms answering in one made every look of every channel
            --  together six beats long, and its chance of a false alarm that much larger, seed 2 of the fake
            --  robot, 10-14); the wait above is the longest.
            for I in 1 .. N loop
               if Usable (I) and then M.Groups (Listed (I).Group).Delay_Known then
                  Earliest := Natural'Min (Earliest, M.Groups (Listed (I).Group).Delay_Beats);
               end if;
            end loop;
            if Earliest = Natural'Last then
               Earliest := 0;
            end if;
            if not Delay_Known then
               declare
                  Own : Natural := 0;
                  Any : Boolean := False;
               begin
                  for I in 1 .. N loop
                     if Usable (I) and then Steps.Episodes (M, Listed (I).Group) > Before (I)
                       and then Steps.Latest (M, Listed (I).Group).Moved
                     then
                        Own := Natural'Max (Own, Steps.Latest (M, Listed (I).Group).Moved_At
                                                 - Steps.Latest (M, Listed (I).Group).Start);
                        Any := True;
                     end if;
                  end loop;
                  Delay_Beats := (if Any then Own else Steps.First_Push (M));
               end;
            end if;
            for E in 1 .. Eye_Count (M) loop
               Lag := Natural'Max (Lag, Natural'Max (0, Image_Lag (M, Eye_Id (E))));
            end loop;
            Wait := Wait + Delay_Beats + Lag;
            --  Once the readings had time to answer, every beat is looked at,
            --  and the look ends at the first one some eye saw move, or when
            --  the slowest eye's lag has passed: a lag measured long (an
            --  eye that hardly sees the group, its fit taking the alias a push
            --  away: A62's eye 3 at seven beats for one) costs its wait only
            --  where nothing is seen. The chance of a false alarm is summed
            --  over every beat looked at, whenever the look ends (Look).
            Done := Looked >= Wait;
            --  Before the body's delay is known, every beat is looked at from the first: an eye may show the move
            --  before the readings do, and the wait above is then only how long an unanswered push is given.
            if not Done and then (not Delay_Known or else Looked > Earliest) then
               Look (M, From, Seen, Chance);
               Done := Seen;
            end if;
            if Done then
               Look (M, From, Seen, Chance);
               Looked_To := M.Beats - 1;
               Seen_Any := Seen;
               Read_Ends;
            end if;
            Driver.Beats.Send (Driver.Commands.Hold);
            exit when Done;
         end;
      end loop;
      --  A channel whose push is still under way when the look is over has not shown where the level takes it:
      --  judged while it was still travelling, every level went further than the one before, and the ladder
      --  doubled its ask up to 2 ** 18 units of a servo that travels for seconds to a far target, swinging the arm
      --  from end to end at each level (R2 on the Pi, 10-13: arm b's joint 1, whose own eye did not see it move).
      --  The push is waited for to its end as Steps judges it (the readings still, or plainly going nowhere: a
      --  joint that creeps towards its target for good is not waited for for good), held meanwhile, and the
      --  readings are read then. What an eye sees meanwhile falls in the next look, as any response that shows
      --  later than expected does.
      --  A joint that travels slowly, less in a beat than its readings jitter, is under way too: Steps counts a
      --  push answered once a reading has left where the push started by more than its noise explains, not
      --  only when it moves from one beat to the next (the kiwi's elbow, read as going nowhere at every level,
      --  was probed to 8.8e16 units and called dead, R6's seventh and ninth starts, 10-14). Holding until the
      --  whole body was still instead (the tenth start) caught it too, and cost the recognition two minutes of
      --  eyes settling after levels nothing had answered.
      while Still_Pushing loop
         Driver.Beats.Next (B);
         Read_Ends;
         --  While its push is under way, what an eye sees is this move's answer, later than the wait above: a look
         --  that has not seen it yet goes on over these beats, its chance with them. The next move's look begins
         --  where this push ended. Charged with the tail before it, each look of a level's confirmation covered some
         --  twenty beats (690 beats over 31 moves), its chance of a false alarm 0.73, and confirming the level took
         --  those 31 moves (b07, seed 8, 10-17).
         if not Seen then
            Look (M, From, Seen, Chance);
            Looked_To := M.Beats - 1;
            Seen_Any := Seen;
         end if;
         Driver.Beats.Send (Driver.Commands.Hold);
      end loop;
      Unlooked := M.Beats;
      Driver.Log.Line (Driver.Log.Robot, "probe: the move's look judged beats" & From'Image & " to" & Looked_To'Image
                       & (if Seen then ", seen" else ", nothing seen") & ", its chance of a false alarm "
                       & Driver.Log.Image (Chance, 3) & "; its push ended at beat" & Natural'Image (M.Beats - 1));
   end Move_And_Look;

   --  A probe level took the channel further along its ask than any smaller
   --  offset did, by Advance, as far as anything can tell: by a step that is
   --  both one an eye watching the channel can see (when one does) and
   --  significant against the readings' noise, the body's one test of motion
   --  (Channels.Visible) for one channel's advance. A visible step alone is
   --  no evidence (a lock-in can fit 1e-17 to a creeping group), nor is a
   --  noise that is not measured. (Not the fraction of the offset
   --  delivered: a reading held a constant hair off its target delivers a
   --  fraction that shrinks towards one as the offset grows, which exact
   --  readings call significant.)
   function Further
     (M : Model; Ref : Channel_Ref; Advance, Noise : Real) return Boolean
   is
      V : constant Estimate := Visible_Step (M, Ref.Group, Ref.Channel);
   begin
      return Advance > 0.0 and then (not Known (V) or else Advance >= V.Value)
        and then Driver.Uncertain.Significant (Advance, Noise * Sqrt (2.0));
   end Further;

   --  The body executed a level: the reading went along the ask, significantly against its own noise. The eye's
   --  visible step is not asked here: it is the lock-in's, measured from the few pushes before, and asked of an
   --  executed turn it made arm_a's second joint, moved 19.6 units at every level, "not further" at every one,
   --  and the probe doubled nineteen times (R6's fifth start, 10-14).
   function Executed (Advance, Noise : Real) return Boolean is
     (Advance > 0.0 and then Driver.Uncertain.Significant (Advance, Noise * Sqrt (2.0)));

   --  A level some eye saw, confirmed by moves back and forth by the same
   --  offsets, each of them seen and each executed by the body, until the
   --  chance that every look of the run, the level's first, saw a false
   --  alarm (the product of their own chances) is below Z's tail over every
   --  level: a long look has a large chance, a short one a small one, and a
   --  count set by the level's look alone asked 6455 moves of s18 (servo seed
   --  4, 10-17) whose own looks each had 0.134, where five do. Executed as
   --  the level's own look asks (Probe_Together): some listed reading went the way the move asked,
   --  from where the move before left it, significantly against its own
   --  noise (Executed). A move the body did not make is no evidence however
   --  its look went: on the servo fake the level of 4.9 units took one
   --  channel out of its dead band once and none again, the eye "saw" each
   --  move after it with a chance of a false alarm near one, and the run that
   --  chance asked was some 270 moves, 10800 beats (s02 and s04 on 10-16).
   procedure Confirm
     (M         : in out Model;
      Listed    : Channel_Refs;
      Usable    : Flag_Array;
      Base      : Driver.Commands.Command;
      Offsets   : Real_Array;
      Noise     : Real_Array;
      After     : Real_Array;   --  the readings the level's own move left
      Unlooked  : in out Natural;
      Chance    : Real;
      Confirmed : out Boolean;
      Group     : Natural := 0)   --  the one group the listed channels are of, or 0
   is
      --  The run is rare enough when the chance that every look in it was a false alarm, the product of their own
      --  chances (the level's look first), is below Z's tail over every level a probe may try. Its verdict takes in the
      --  lock-in's chance for the Group, once the run is over (Probe_Together).
      Rare      : constant Real := Ln (Tail / Real (Levels));
      Run_Log   : Real := (if Chance > 0.0 then Ln (Chance) else Real'First);
      Run       : Natural := 1;
      At_Amount : Boolean := True;
      Seen      : Boolean;
      Again     : Real;
      Now       : Real_Array (1 .. Listed'Length);
      Before    : Real_Array (1 .. Listed'Length) := After;
      Moving    : Flag_Array (1 .. Listed'Length);
      Read_At   : Natural;
      Made      : Boolean;
   begin
      while Run_Log > Rare loop
         At_Amount := not At_Amount;
         declare
            Back : Real_Array (1 .. Listed'Length);
         begin
            for I in Back'Range loop
               Back (I) := (if At_Amount then Offsets (Offsets'First + I - 1) else 0.0);
            end loop;
            Move_And_Look (M, Listed, Usable, Base, Back, Unlooked, Seen, Again, Now, Moving, Read_At);
         end;
         exit when not Seen;
         Made := False;
         for I in 1 .. Listed'Length loop
            if Usable (I) and then Offsets (Offsets'First + I - 1) /= 0.0 then
               declare
                  --  The way this move asked the reading to go: along the offset when out to it, against it
                  --  when back.
                  Way : constant Real := (if Offsets (Offsets'First + I - 1) > 0.0 then 1.0 else -1.0)
                                         * (if At_Amount then 1.0 else -1.0);
               begin
                  Made := Made or else Executed (Way * (Now (I) - Before (I)), Noise (I));
               end;
            end if;
         end loop;
         exit when not Made;
         --  A look whose chance is not below one tells nothing, and no run of them becomes rare.
         exit when Again >= 1.0;
         Run_Log := (if Again > 0.0 then Run_Log + Ln (Again) else Real'First);
         Before := Now;
         Run := Run + 1;
      end loop;
      declare
         Looks  : constant Real := Run_Log;
         Prior  : Real := 1.0;   --  the lock-in's chance
         procedure Read_Prior is
         begin
            Prior := Lockin.Moving_Chance (M, Group_Id (Group));
         end Read_Prior;
      begin
         if Group > 0 then
            Hold_And_Estimate (M);
            Driver.Beats.Within_A_Beat (Read_Prior'Access);
            Run_Log := (if Run_Log = Real'First or else Prior <= 0.0 then Real'First else Run_Log + Ln (Prior));
         end if;
         Confirmed := Run_Log <= Rare;
         Driver.Log.Line (Driver.Log.Robot, "probe: the level's look and" & Natural'Image (Run - 1) & " moves back and forth"
                          & " after it, each seen and made, had together a chance of e^"
                          & Driver.Log.Image ((if Looks = Real'First then -Real'Last else Looks), 2)
                          & " of false alarms only"
                          & (if Group > 0 then ", the lock-in's " & Scientific (Prior) else "")
                          & ", against e^" & Driver.Log.Image (Rare, 2) & ": "
                          & (if Confirmed then "confirmed" else "not confirmed"));
      end;
   end Confirm;

   --  Where every listed channel is held and reads, read in a held beat:
   --  which channels can take part, their readings, their noise (measured
   --  again first when some channel's is not), the hold every listed group is
   --  moved from, and the first step when First is 0. The estimates can come
   --  back beats after they were asked (Estimate_Now returns in the beat they
   --  are in the model), so nothing is read before them: the beat the probe
   --  begins at, the readings it starts from, the holds and the first step are
   --  all those of the beat they return in.
   procedure Begin_Probe
     (M        : in out Model;
      Listed   : Channel_Refs;
      First    : Real;
      Usable   : out Flag_Array;
      Start    : out Real_Array;
      Noise    : out Real_Array;
      Base     : out Driver.Commands.Command;
      Amount   : out Real;
      Unlooked : out Natural;
      Start_At : out Natural;
      Creep    : out Real_Array;
      Weights  : out Real_Array)
   is
      B : Driver.Clock.Beat;

      --  Whether the listed channel can take part: its group takes commands,
      --  has the channel and has a reading at the last beat.
      function Takes_Part (I : Positive) return Boolean is
        (Natural (Listed (I).Group) <= Group_Count (M) and then Is_Commandable (M, Listed (I).Group)
         and then Listed (I).Channel <= Group_Size (M, Listed (I).Group)
         and then Channels.Has_Reading (M, Listed (I).Group, M.Beats - 1));
   begin
      Usable := [others => False];
      Start := [others => 0.0];
      Start_At := 0;
      Creep := [others => 0.0];
      Noise := [others => 0.0];
      Weights := [others => 1.0];
      Base := Driver.Commands.Hold;
      Amount := First;
      Driver.Beats.Next (B);
      --  A reading whose noise is unmeasured tells nothing of whether it
      --  followed (Further). What the model had when it last measured was
      --  not enough to find a rest for its group, and the beats since may
      --  be: the noise is measured again from every one of them, before
      --  anything is read.
      if M.Beats > 0
        and then (for some I in Listed'Range =>
                    Takes_Part (I) and then Reading_Noise (M, Listed (I).Group, Listed (I).Channel) >= Real'Last)
      then
         Estimate_Now (M);
      end if;
      Unlooked := M.Beats;
      if M.Beats > 0 then
         for I in Listed'Range loop
            if Takes_Part (I) then
               declare
                  G : constant Group_Id := Listed (I).Group;
                  C : constant Positive := Listed (I).Channel;
               begin
                  Usable (I) := True;
                  Start (I) := Channels.Reading (M, G, M.Beats - 1, C);
                  Start_At := M.Beats - 1;
                  Creep (I) := Channels.Drift (M, G, C);
                  Noise (I) := Reading_Noise (M, G, C);
                  if not Driver.Commands.Has_Target (Base, G) then
                     declare
                        H : Real_Array (1 .. Group_Size (M, G));
                     begin
                        for K in H'Range loop
                           H (K) := Hold_Of (M, G, K);
                        end loop;
                        Driver.Commands.Set_Target (Base, G, H);
                     end;
                  end if;
                  if First = 0.0 then
                     --  The smallest step every listed reading can tell.
                     Amount := Real'Max (Amount, Smallest_Step (M, G, C));
                  end if;
               end;
            end if;
         end loop;
         --  Each listed channel from its own group's least step (Group_Least_Step), of the largest any group has: a
         --  group's readings tell its own steps, and one amount for all, the largest step any reading tells, took the
         --  quietest joints as far as the noisiest reading needs. RB-Y1's wheels, read as counts over a beat at eight
         --  times, set 0.546 units for every channel, and its torso, read at an eighth, turned 4.4 radians' worth into
         --  its limit and folded the robot to the floor (seed 3, paced rounds p89 to p99, 10-23 and 10-24). (Experiment
         --  bd113.)
         if First = 0.0 and then Amount > 0.0 then
            for I in Listed'Range loop
               if Usable (I) then
                  Weights (I) := Group_Least_Step (M, Listed (I).Group) / Amount;
               end if;
            end loop;
         end if;
      end if;
      Driver.Beats.Send (Driver.Commands.Hold);
   end Begin_Probe;

   procedure Probe_Together
     (M         : in out Model;
      Channels  : Channel_Refs;
      Direction : Real;
      First     : Real;
      Report    : out Probe_Report;
      Other_Way : Real := 0.0)
   is
      Executes  : Boolean := False;   --  some listed reading went along some amount this way
      Empty_Run : Natural := 0;       --  levels running that moved no reading further and showed nothing
      Sign      : constant Real := (if Direction > 0.0 then 1.0 else -1.0);
      N         : constant Natural := Channels'Length;
      Listed    : constant Channel_Refs (1 .. N) := Channels;
      Usable    : Flag_Array (1 .. N);
      Base      : Driver.Commands.Command;            --  every listed group at its hold
      Start     : Real_Array (1 .. N);   --  each reading when the probe began
      Noise     : Real_Array (1 .. N);
      Following : Flag_Array (1 .. N);
      Delivered : Flag_Array (1 .. N) := [others => False];   --  some level moved it along the ask
      Did       : Flag_Array (1 .. N) := [others => False];   --  this level's ask was executed (Executed)
      Farthest  : Real_Array (1 .. N) := [others => 0.0];   --  the farthest a level took it along the ask
      Kept      : Real_Array (1 .. N) := [others => 0.0];   --  the offset where it last followed
      Amount    : Real;
      Unlooked  : Natural;
      Start_At  : Natural;                --  the beat the probe's readings were read at
      Creep     : Real_Array (1 .. N);    --  each reading's steady change a beat at rest (Channels.Drift)
      Farthest_At : Count_Array (1 .. N);   --  the beat each one's farthest reading was read at
      Weights   : Real_Array (1 .. N);    --  each one's share of the amount (Begin_Probe)
      Began     : Real;                   --  the amount the ladder began from
      Last_Executed : Real := 0.0;        --  the largest level some listed reading went along at
   begin
      Report := (others => <>);
      Begin_Probe (M, Listed, First, Usable, Start, Noise, Base, Amount, Unlooked, Start_At, Creep, Weights);
      Began := Amount;
      --  A group's own probe: every eye's picture as it began, for what the level seen moved (Reach_Ratio).
      if First /= 0.0 then
         Mark_Origins (M);
      end if;
      Farthest_At := [others => Start_At];
      Following := Usable;
      if Amount <= 0.0 or else (for all U of Usable => not U) then
         return;
      end if;
      --  Where the body begins to execute, first: the amount climbs one doubling more at each level no listed reading
      --  went along, and once one did, it is halved back between the last that none did and the least that some did
      --  until the two lie within a doubling; the ladder then starts from the least that some did. Every level no
      --  reading answers is waited for as long as no push of the body had ever been answered (Steps.First_Push), and
      --  the ladder doubled through the body's dead band one such wait at a time: b13 (seed 10 of the fake robot,
      --  readings in milliradians, 10-17) asked 4.2e-7 units first, its readings first went along at 1.76, twenty-two
      --  levels of 45 beats each, a thousand of the recognition's 2059 beats. An amount is judged from the hold: after
      --  a level some reading went along, the body is sent back to it before the next.
      declare
         Least  : constant Real := Amount;
         Most   : constant Real := Least * 2.0 ** Levels;   --  as far as the ladder itself would double
         Lo, Hi : Real := 0.0;   --  the largest amount no reading went along, the least some did
         Stride : Natural := 0;  --  the doublings the amount climbed at its last level
         Back   : Step_Report;
      begin
         loop
            declare
               Ask     : constant Real := (if Hi = 0.0 then Amount else Sqrt (Lo * Hi));
               Offsets : Real_Array (1 .. N);
               Seen    : Boolean;
               Chance  : Real;
               Now     : Real_Array (1 .. N);
               Moving  : Flag_Array (1 .. N);
               Now_At  : Natural;
               Went    : Natural := 0;
            begin
               for I in 1 .. N loop
                  Offsets (I) := Sign * Ask * Weights (I);
               end loop;
               Move_And_Look (M, Listed, Usable, Base, Offsets, Unlooked, Seen, Chance, Now, Moving, Now_At);
               for I in 1 .. N loop
                  if Usable (I)
                    and then Executed (Sign * (Now (I) - Start (I)) - Sign * Creep (I) * Real (Now_At - Start_At),
                                       Noise (I))
                  then
                     Went := Went + 1;
                  end if;
               end loop;
               Driver.Log.Line (Driver.Log.Robot, "probe: where the body begins to execute, every channel together at "
                                & Scientific (Ask) & " units, read at beat" & Now_At'Image & ":" & Went'Image
                                & " channels went along their ask");
               if Went > 0 then
                  Hi := Ask;
                  --  Back to the hold, and its readings, before the next amount is judged from them.
                  Step (M, Base, Back);
                  Start_At := M.Beats - 1;
                  for I in 1 .. N loop
                     if Usable (I) then
                        Start (I) := Driver.Robot.Channels.Reading (M, Listed (I).Group, Start_At, Listed (I).Channel);
                     end if;
                  end loop;
               else
                  Lo := Ask;
                  if Hi = 0.0 then
                     Stride := Stride + 1;
                     Amount := Real'Min (Ask * 2.0 ** Stride, Most);
                  end if;
               end if;
               exit when Hi > 0.0 and then (Lo = 0.0 or else Hi <= 2.0 * Lo);
               exit when Hi = 0.0 and then Lo >= Most;
               --  A limit is one-sided: a way that moved no reading up to the amount the other way first moved one
               --  at is at its end.
               exit when Hi = 0.0 and then Other_Way > 0.0 and then Lo >= Other_Way;
            end;
         end loop;
         Executes := Hi > 0.0;
         if Executes then
            Amount := Hi;
            Report.Answered := Hi;
         end if;
         Farthest_At := [others => Start_At];
      end;
      --  Nothing it was asked moved a listed reading: no level can be seen executed (below), and none is climbed. RB-Y1's
      --  jaw at its end was asked shut from 0.017 to 1.5e14 units and then up the ladder's levels, 190 of its 1200
      --  beats (round 4c, 10-23).
      if not Executes then
         Driver.Log.Line (Driver.Log.Robot, "probe: every channel together moved no reading along its ask, up to "
                          & Scientific (Amount) & " units: no level is climbed");
         Step (M, Base, Report.Last);
         return;
      end if;
      for Level in 1 .. Levels loop
         Report.Steps := Level;
         declare
            Offsets : Real_Array (1 .. N);
            Executed_None : Boolean := False;   --  no listed reading went further along its ask at this level
            Seen    : Boolean;
            Chance  : Real;
            Now     : Real_Array (1 .. N);
            Moving  : Flag_Array (1 .. N);
            Now_At  : Natural;
         begin
            for I in 1 .. N loop
               Offsets (I) := (if Following (I) then Sign * Amount * Weights (I) else Kept (I));
            end loop;
            Move_And_Look (M, Listed, Usable, Base, Offsets, Unlooked, Seen, Chance, Now, Moving, Now_At);
            --  Which channels still follow: whether asking further took each
            --  further than any smaller offset did, less the creep of the beats between (Channels.Drift).
            Did := [others => False];
            for I in 1 .. N loop
               if Usable (I) and then Following (I) then
                  declare
                     Excursion : constant Real := Sign * (Now (I) - Start (I));
                     Advance   : constant Real :=
                       Excursion - Farthest (I)
                       - Sign * Creep (I) * Real (Now_At - Natural'Min (Now_At, Farthest_At (I)));
                     Went      : constant Boolean := Further (M, Listed (I), Advance, Noise (I));
                  begin
                     Did (I) := Executed (Advance, Noise (I));
                     if Delivered (I) and then not Went and then not Moving (I) then
                        --  Its own end: held where it last followed.
                        Following (I) := False;
                     else
                        Kept (I) := Sign * Amount * Weights (I);
                        if Went then
                           Delivered (I) := True;
                           Farthest (I) := Excursion;
                           Farthest_At (I) := Now_At;
                        end if;
                     end if;
                  end;
               end if;
            end loop;
            declare
               Went_Along : Natural := 0;
            begin
               for I in 1 .. N loop
                  if Usable (I) and then Did (I) then
                     Went_Along := Went_Along + 1;
                  end if;
               end loop;
               Driver.Log.Line (Driver.Log.Robot, "probe: level" & Level'Image & " of every channel together, "
                                & Scientific (Amount) & " units, read at beat" & Now_At'Image & ": seen " & Seen'Image
                                & " (the look's chance of a false alarm " & Driver.Log.Image (Chance, 3) & "), "
                                & Went_Along'Image & " channels went along their ask");
               Executed_None := Went_Along = 0;
               if Went_Along > 0 then
                  Last_Executed := Amount;
               end if;
            end;
            --  A level is seen only when the body executed it: some channel's reading went along its ask. A
            --  camera on an arm whose servo hums against its dead band shakes, and an eye sees motion where no
            --  reading moved (R6 on the kiwi, 10-14: every channel together "first seen" at 4.9 units while no
            --  reading changed, and the recognition ran on 4.9-unit asks the servos ignored, each waited for as
            --  long as the stream was old).
            if Seen and then (for some I in 1 .. N => Usable (I) and then (Delivered (I) or else Did (I))) then
               declare
                  Confirmed : Boolean := False;
                  --  What this level moved, as it stands, before the moves back and forth (Reach_Ratio).
                  Level_Reach : constant Real := (if First /= 0.0 then Reach_Ratio (M) else 1.0);
                  --  The run of moves back and forth (Confirm) goes on as its looks ask, and its verdict, for one group's
                  --  channels, takes in the lock-in's evidence of the group moving some eye over every push, the run's among
                  --  them (Lockin.Moving_Chance): another chance that chance alone made what was seen, multiplied in as the
                  --  looks' are. The Go2's walk and the drone's flight, their pictures answering beats after their readings,
                  --  broke their runs at the first move and the boot stopped, while the lock-in had them moving 96 and 150 of
                  --  their eyes' 255 cells (seed 1 of each, paced round p69a, 10-23). Not to shorten the run: its moves are
                  --  the rows the verdicts are judged from, and runs the lock-in ended at no move left the Panda's closer
                  --  inert and its base camera on its arm, and Stretch's base and the Shadow's hand taken for arms (paced
                  --  rounds p70 to p72); a group the lock-in measured nothing of (its pushes beyond the flow's reach, or its
                  --  pictures never settled, as Stretch's base and the G1's groups) adds nothing. The channels of several
                  --  groups, pushed at the same beats (the first ladder, of every channel), leave the lock-in no way to tell
                  --  one group's part from another's.
                  One_Group : constant Boolean :=
                    (for all I in 1 .. N => Natural (Listed (I).Group) = Natural (Listed (Listed'First).Group));
               begin
                  Confirm (M, Listed, Usable, Base, Offsets, Noise, Now, Unlooked, Chance, Confirmed,
                           Group => (if One_Group then Natural (Listed (Listed'First).Group) else 0));
                  if Confirmed then
                     Report.Seen := True;
                     Report.Excursion := Amount;
                     Report.Factor := Amount / Began;
                     Report.Reach := Level_Reach;
                  else
                     Driver.Log.Line (Driver.Log.Robot, "probe: level" & Level'Image & " of every channel together, "
                                      & Scientific (Amount) & " units, was seen by an eye but not confirmed (the look's"
                                      & " chance of a false alarm " & Driver.Log.Image (Chance, 3) & ")");
                  end if;
               end;
            elsif Seen then
               Driver.Log.Line (Driver.Log.Robot, "probe: level" & Level'Image & " of every channel together, "
                                & Scientific (Amount) & " units, was seen by an eye while no reading went along its ask");
            end if;
            exit when Report.Seen or else (for all I in 1 .. N => not (Usable (I) and then Following (I)));
            --  Two levels running past the first that no reading went further along and no eye saw, the first being where
            --  the body was found to execute: doubling the ask moves the body no further (a speed at its most, a joint at
            --  its end), and no level beyond can be seen executed. The Go2's walk, asked down from 0.52 units, went along
            --  at no level and showed nothing, and the ladder doubled it 50 times, to 1e9 units, a level every two beats
            --  (seed 2, paced round p62a, 10-23). Two, the least that tells the body's having stopped from one level's miss:
            --  ended at one level an eye saw (the drone's readings, its flight controller's wandering estimates, went no
            --  further while its camera saw it fly, seeds 1 and 2, paced round p65a), or at one that showed nothing (the
            --  drone hovering, its camera swaying 163 cells a beat at rest, missed its second level, seed 1, p66a), the
            --  ladder ended with nothing confirmed and the boot stopped.
            Empty_Run := (if Executed_None and then not Seen and then Level > 1 then Empty_Run + 1 else 0);
            if Empty_Run >= 2 then
               Driver.Log.Line (Driver.Log.Robot, "probe: level" & Level'Image & " moved no reading further along its ask:"
                                & " the ladder ends");
               exit;
            end if;
            Amount := 2.0 * Amount;
         end;
      end loop;
      --  A group's own ladder no look saw at any level its readings executed: the lock-in, regressing every eye's
      --  cells on the group's readings over all of the ladder's pushes (Lockin.Moving_Chance), may tell it moved
      --  some eye where no look of a few beats could. Its chance below the rarity a level's confirmation asks (Z's
      --  tail over every level a probe may try, as Confirm's): seen, at the largest level its readings executed.
      --  The G1's hand, its seven fingers pushed together, showed nothing in looks of five beats at any of eight
      --  levels, its pictures swaying from its balance, and was probed a channel at a time, 50 beats each, two fingers
      --  seen, out of beats (seed 1, paced rounds p116a and p116b, 10-24).
      if not Report.Seen and then First /= 0.0 and then Last_Executed > 0.0
        and then (for all I in 1 .. N => Natural (Listed (I).Group) = Natural (Listed (Listed'First).Group))
      then
         declare
            Prior : Real := 1.0;
            procedure Read_Prior is
            begin
               Prior := Lockin.Moving_Chance (M, Listed (Listed'First).Group);
            end Read_Prior;
         begin
            Hold_And_Estimate (M);
            Driver.Beats.Within_A_Beat (Read_Prior'Access);
            Report.Seen := Prior < Tail / Real (Levels);
            Driver.Log.Line (Driver.Log.Robot, "probe: no level of the group was seen by a look; the lock-in's chance "
                             & Scientific (Prior) & " against " & Scientific (Tail / Real (Levels)) & ": "
                             & (if Report.Seen then "seen at " & Scientific (Last_Executed) & " units"
                                else "not seen"));
            if Report.Seen then
               Report.Excursion := Last_Executed;
               Report.Factor := Last_Executed / Began;
            end if;
         end;
      end if;
      --  Back to the hold.
      Step (M, Base, Report.Last);
   end Probe_Together;

   procedure Probe_Both_Ways
     (M      : in out Model;
      Ref    : Channel_Ref;
      First  : Real;
      Report : out Two_Way_Report)
   is
      Listed    : constant Channel_Refs (1 .. 1) := [1 => Ref];
      Usable    : Flag_Array (1 .. 1);
      Base      : Driver.Commands.Command;
      Start     : Real_Array (1 .. 1);
      Noise     : Real_Array (1 .. 1);
      Amount    : Real;
      Unlooked  : Natural;
      Signs     : constant array (Sense) of Real := [Increasing => 1.0, Decreasing => -1.0];
      Other     : constant array (Sense) of Sense := [Increasing => Decreasing, Decreasing => Increasing];
      Open      : Sense_Flags := [others => True];
      Delivered : Sense_Flags := [others => False];
      Did       : Sense_Flags := [others => False];   --  the way's latest level was executed (Executed)
      Answered  : Sense_Counts := [others => 0];   --  the level at which it first followed that way
      Farthest  : array (Sense) of Real := [others => 0.0];
      Start_At  : Natural;                         --  the beat the probe's reading was read at
      Creep     : Real_Array (1 .. 1);             --  its steady change a beat at rest (Channels.Drift)
      Weights   : Real_Array (1 .. 1);             --  its share of the amount: all of it, alone
      Farthest_At : array (Sense) of Natural;      --  the beat each way's farthest reading was read at
      Last_Offset : Real := 0.0;                   --  the latest level's push and its way: held until the next level
      Last_Way    : Sense := Increasing;           --  begins, where a speed shows itself (Settle)
   begin
      Report := (others => <>);
      Begin_Probe (M, Listed, First, Usable, Start, Noise, Base, Amount, Unlooked, Start_At, Creep, Weights);
      Farthest_At := [others => Start_At];
      if not Usable (1) then
         return;
      end if;
      loop
         declare
            Way   : Sense := Increasing;
            Found : Boolean := False;
         begin
            --  The next way: the one asked at the lower level, the increasing
            --  one first; the decreasing one only while the increasing one has
            --  delivered nothing, or once it has ended.
            for S in Sense loop
               if Open (S) and then Report.Levels (S) < Levels
                 and then (S = Increasing or else not Open (Increasing) or else not Delivered (Increasing))
                 and then (not Found or else Report.Levels (S) < Report.Levels (Way))
               then
                  Way := S;
                  Found := True;
               end if;
            end loop;
            exit when not Found;
            Report.Levels (Way) := Report.Levels (Way) + 1;
            declare
               Offset  : constant Real := Amount * 2.0 ** (Report.Levels (Way) - 1);
               Offsets : constant Real_Array (1 .. 1) := [1 => Signs (Way) * Offset];
               Seen    : Boolean;
               Chance  : Real;
               Now     : Real_Array (1 .. 1);
               Moving  : Flag_Array (1 .. 1);
               Now_At  : Natural;
               Waited  : Natural;
            begin
               --  Each push from a body at rest and every picture settled: a sighting of this channel is confirmed by
               --  the lock-in (below), whose regression counts a push only from a picture at rest, and an eye whose
               --  picture still settled from the move before when the push began has no row of it. s49 and s50 (servo
               --  seed 6 of the fake robot, 10-17 and 10-18): the closer's pushes began a beat after the probe's
               --  together levels ended, while eye 3 (on the arm those levels swung) still settled, none of them was a
               --  row of eye 3, the closer's column never had a beat there, and the closer, which only eye 3 sees, was
               --  left unclassified. The waits after looks that saw nothing real gave those pictures their rest before.
               --  And a push that keeps the body going while held (a speed: a wheel, a walk, a flight) is sent back to the
               --  hold once every reading stands and the pictures drift on (round 0, 10-23: the Go2 waited here 6800
               --  beats, walking on, its picture never settling).
               declare
                  Kept, Came    : Boolean := False;
                  Kept_Eyes     : Channel_Flags (1 .. Eye_Count (M));
               begin
                  --  A channel whose held push came back once is a joint: no hold to tell it from a speed (round 1,
                  --  10-23: ALOHA's sagging arms were held and sent back some twenty times a boot).
                  if Comes_Back (M, Ref.Group, Ref.Channel) then
                     Settle (M, Waited);
                  else
                     Settle (M, Base, Waited, Kept, Kept_Eyes, Came);
                     --  The first level's settle holds the push before this channel's own (the last level of every channel of
                     --  its group together, sent back to the hold): what the pictures did then tells nothing of this
                     --  channel. Taken for its own, the jaws of the SO-101 and of ALOHA's second arm were speeds, the
                     --  together ladder's last level having moved their readings no further: a box the jaw knocked, and
                     --  the first arm's eye, drifted on as every reading stood and stayed away (seeds 3 and 1, paced round
                     --  p68a, 10-23).
                     if not (Last_Offset > 0.0) and then (Kept or else Came) then
                        Driver.Log.Line (Driver.Log.Robot, "probe: group" & Ref.Group'Image & " channel" & Ref.Channel'Image
                                         & ": the pictures " & (if Kept then "stayed away" else "came back")
                                         & " after a push before its own: not told of it");
                        Kept := False;
                        Came := False;
                     end if;
                     if Came then
                        Note_Comes_Back (M, Ref.Group, Ref.Channel);
                     end if;
                  end if;
                  if Kept then
                     Note_Keeps_Going (M, Ref.Group, Ref.Channel, Kept_Eyes);
                     Report.Keeps_Going := True;
                     Driver.Log.Line (Driver.Log.Robot, "probe: group" & Ref.Group'Image & " channel" & Ref.Channel'Image
                                      & " keeps the body going while held: a speed, sent back to its hold after"
                                      & " every push of it");
                     --  The last level's push, held, drifted the pictures on and stopped once sent back: seen there; this
                     --  level is not pushed.
                     if Last_Offset > 0.0 then
                        Report.Seen := True;
                        Report.Excursion := Last_Offset;
                        Report.Seen_Sense := Last_Way;
                        if Report.First_Seen = 0.0 or else Last_Offset < Report.First_Seen then
                           Report.First_Seen := Last_Offset;
                        end if;
                        Report.Levels (Way) := Report.Levels (Way) - 1;
                     end if;
                  end if;
               end;
               exit when Report.Seen;
               --  The level's look begins with its push: the beats since the last look ended (this settle, and the
               --  estimate a sighting before it was confirmed by) showed the pushes before it settling and the eyes at
               --  rest, and every one of them was a verdict that could only be a false alarm of this level. h401 (seed
               --  401 of the default fake robot on the big machine, 10-22, a humanoid of 30 channels): each level of
               --  each channel looked at the 436 to 484 beats since the last, its chance of a false alarm 1.000, was
               --  "seen", and was confirmed by an estimate of 17 to 20 s that saw nothing; 4 channels in 28209 beats.
               Mark_Origins (M);
               Unlooked := M.Beats;
               Move_And_Look (M, Listed, Usable, Base, Offsets, Unlooked, Seen, Chance, Now, Moving, Now_At);
               Last_Offset := Offset;
               Last_Way := Way;
               declare
                  Excursion : constant Real := Signs (Way) * (Now (1) - Start (1));
                  --  Less the creep of the beats since the farthest reading (Channels.Drift): a creeping reading went
                  --  further at every level the way it creeps, and the jaw held open at its end was asked open,
                  --  never shut, to 7e13 units (f16f on 10-15).
                  Advance   : constant Real :=
                    Excursion - Farthest (Way)
                    - Signs (Way) * Creep (1) * Real (Now_At - Natural'Min (Now_At, Farthest_At (Way)));
                  Went      : constant Boolean := Further (M, Ref, Advance, Noise (1));
               begin
                  Did (Way) := Executed (Advance, Noise (1));
                  if Delivered (Way) and then not Went and then not Moving (1) then
                     --  Its own end.
                     Open (Way) := False;
                  elsif Went then
                     if not Delivered (Way) then
                        Answered (Way) := Report.Levels (Way);
                        Report.Answered := (if Report.Answered = 0.0 then Offset else Real'Min (Report.Answered, Offset));
                     end if;
                     Delivered (Way) := True;
                     Farthest (Way) := Excursion;
                     Farthest_At (Way) := Now_At;
                  end if;
               end;
               --  Seen only at a level the body executed that way (as Probe_Together: a humming servo shakes
               --  its camera without moving its reading). A sighting is confirmed by the body's one test of
               --  motion: the estimates are computed once, and the lock-in's regression on every push so far
               --  must see this channel move some eye (Lockin.Seen: its gain over the cells that respond stands
               --  Z of its spread above zero). It used to be confirmed by a run of moves back and forth each seen
               --  by an eye's per-beat verdict, as many as that verdict's chance of a false alarm asked
               --  (Confirm): 6 to 22 moves of 8 to 14 beats for every channel of every group, 340 beats a
               --  channel on the fake robot against 40 on the kiwi, and the "probes with confirmations" of the
               --  kiwi's recognition (10-14). A channel probed alone has a column of its own in the regression,
               --  so the lock-in can tell it from the rest (the levels of every channel together cannot be told
               --  apart that way, and that ladder keeps its runs).
               if Keeps_Going (M, Ref.Group, Ref.Channel) then
                  --  A speed: held until the pictures drift on (seen, and sent back by the settle) or settle (unseen at
                  --  this level, and sent back): a look of a few beats saw nothing of a walk the body starts some beats
                  --  after its readings answer (the Go2 in round 1, 10-23: its looks two to three beats long).
                  declare
                     Kept, Came : Boolean;
                     Kept_Eyes  : Channel_Flags (1 .. Eye_Count (M));
                  begin
                     Settle (M, Base, Waited, Kept, Kept_Eyes, Came);
                     if Kept then
                        Note_Keeps_Going (M, Ref.Group, Ref.Channel, Kept_Eyes);
                        Report.Seen := True;
                        Report.Excursion := Offset;
                        Report.Seen_Sense := Way;
                        if Report.First_Seen = 0.0 or else Offset < Report.First_Seen then
                           Report.First_Seen := Offset;
                        end if;
                     else
                        Step (M, Base, Report.Last);
                     end if;
                  end;
               elsif Seen and then (Delivered (Way) or else Did (Way)) then
                  if Report.First_Seen = 0.0 or else Offset < Report.First_Seen then
                     Report.First_Seen := Offset;
                  end if;
                  declare
                     Confirmed : Boolean := False;
                     --  And the step is one an eye's cells can tell (Lockin.Shows_Step): a channel seen at a level that
                     --  moves the picture by a hundredth of a pixel is seen by the regression's pooled gain and by no
                     --  push a decision could rest on (a drone's motors, d20 on 10-20: levels of 0.05 to 3 thousandths of a
                     --  metre, its own camera shown nothing at them, the group left a part with no arm to sweep).
                     procedure Read_Seen is
                        Step : Real_Array (1 .. Group_Size (M, Ref.Group)) := [others => 0.0];
                     begin
                        Step (Ref.Channel) := Offset;
                        Confirmed := (for some E in 1 .. Eye_Count (M) => Lockin.Seen (M, Eye_Id (E), Ref.Group, Ref.Channel))
                          and then (for some E in 1 .. Eye_Count (M) => Lockin.Shows_Step (M, Eye_Id (E), Ref.Group, Step));
                     end Read_Seen;
                  begin
                     Hold_And_Estimate (M);
                     Driver.Beats.Within_A_Beat (Read_Seen'Access);
                     if Confirmed then
                        Report.Seen := True;
                        Report.Excursion := Offset;
                        Report.Seen_Sense := Way;
                     else
                        Driver.Log.Line (Driver.Log.Robot, "probe: group" & Ref.Group'Image & " channel" & Ref.Channel'Image
                                         & " level" & Report.Levels (Way)'Image & " " & Way'Image & ", " & Scientific (Offset)
                                         & " units, was seen by an eye (chance of a false alarm "
                                         & Driver.Log.Image (Chance, 3) & ") but the lock-in sees no eye moved by it, or by a step its cells tell");
                     end if;
                  end;
               end if;
               exit when Report.Seen;
               --  A limit is one-sided: a way that delivered nothing up to the
               --  level at which the other one answered is at its end.
               for S in Sense loop
                  if Open (S) and then not Delivered (S) and then Delivered (Other (S))
                    and then Report.Levels (S) >= Answered (Other (S))
                  then
                     Open (S) := False;
                     Report.At_End (S) := True;
                  end if;
               end loop;
            end;
         end;
      end loop;
      --  The last level's push, held: one that keeps the body going drifts the pictures while every commanded reading
      --  stands, and is sent back (a channel seen at its first level was never held past it: Stretch's base and some
      --  of the drone's motors in round 1, 10-23).
      if not Keeps_Going (M, Ref.Group, Ref.Channel) and then not Comes_Back (M, Ref.Group, Ref.Channel) then
         declare
            Waited     : Natural;
            Kept, Came : Boolean;
            Kept_Eyes  : Channel_Flags (1 .. Eye_Count (M));
         begin
            Settle (M, Base, Waited, Kept, Kept_Eyes, Came);
            if Came then
               Note_Comes_Back (M, Ref.Group, Ref.Channel);
            end if;
            --  What the last level's push left, held and neither sent back nor come back (Note_Stayed), as a group's
            --  recognition notes it: the graph tells an eye's own group and a closer's eye by it (Graph.Best_Of), and a
            --  group recognized one channel at a time had none. RB-Y1's left finger, probed so, went to the arm whose
            --  wrist's eye showed it in 43 cells against its own wrist's 42 (seed 1, paced rounds p89c and p96b, 10-24).
            if not Kept and then not Came and then Last_Offset > 0.0 then
               Note_Stayed (M, Ref.Group, Signs (Last_Way));
            end if;
            if Kept then
               Note_Keeps_Going (M, Ref.Group, Ref.Channel, Kept_Eyes);
               Report.Keeps_Going := True;
               Driver.Log.Line (Driver.Log.Robot, "probe: group" & Ref.Group'Image & " channel" & Ref.Channel'Image
                                & " keeps the body going while held: a speed, sent back to its hold after every push"
                                & " of it");
               if not Report.Seen and then Last_Offset > 0.0 then
                  Report.Seen := True;
                  Report.Excursion := Last_Offset;
                  Report.Seen_Sense := Last_Way;
                  if Report.First_Seen = 0.0 or else Last_Offset < Report.First_Seen then
                     Report.First_Seen := Last_Offset;
                  end if;
               end if;
            end if;
         end;
      end if;
      --  Neither way followed at any level either way: a deadband would have
      --  yielded to a level that large, so the channel is dead or
      --  disconnected, where the noise tells a following from none.
      Report.Blind := Noise (1) >= Real'Last;
      Report.Dead := not Report.Seen and then not Report.Blind
        and then not Delivered (Increasing) and then not Delivered (Decreasing);
      --  Back to the hold.
      Step (M, Base, Report.Last);
   end Probe_Both_Ways;

   procedure Probe (M : in out Model; G : Group_Id; Channel : Positive; Direction : Real; Report : out Probe_Report) is
   begin
      Probe_Together (M, [1 => (Group => G, Channel => Channel)], Direction, 0.0, Report);
   end Probe;

   procedure Gather_Rest (M : in out Model; Probes : Natural) is
      B      : Driver.Clock.Beat;
      Wanted : Natural := 0;
   begin
      Driver.Beats.Next (B);
      declare
         Eyes   : constant Natural := Eye_Count (M);
         Rest   : Natural := Natural'Last;   --  the fewest counts at rest an eye has
         Window : Natural := 2;              --  the verdicts one move looks at, per eye
      begin
         for E in 1 .. Eyes loop
            Rest := Natural'Min (Rest, (if M.Eyes (Eye_Id (E)).Rest_Counts_Known
                                        then M.Eyes (Eye_Id (E)).Rest_Count_Beats else 0));
            Window := Natural'Max (Window, 2 + Natural'Max (0, Image_Lag (M, Eye_Id (E))));
         end loop;
         if Eyes > 0 and then Probes > 0 then
            declare
               Looked : constant Real := Real (Eyes * Window);
               Log_Levels : constant Real := Ln (Real (Levels) / Tail);
               --  The beats held still to have N counts at rest, plus every
               --  probe's run of moves at that chance of a false alarm.
               function Cost (N : Natural) return Real is
                 (if Real (N + 1) <= Looked then Real'Last
                  else Real (N - Rest) + Real (Probes * Window) * Log_Levels / Ln (Real (N + 1) / Looked));
               N : Natural := Rest;
            begin
               while Cost (N) = Real'Last or else Cost (N + 1) < Cost (N) loop
                  N := N + 1;
               end loop;
               Wanted := N - Rest;
            end;
         end if;
      end;
      Driver.Beats.Send (Driver.Commands.Hold);
      if Wanted > 0 then
         Hold (M, Wanted);
         Driver.Beats.Next (B);
         Estimate_Now (M);
         Driver.Beats.Send (Driver.Commands.Hold);
      end if;
   end Gather_Rest;

   function Refused (State : Plan_Status; Why : String) return Plan is
     ((State => State, Reason => To_Unbounded_String (Why), others => <>));

   function Stray
     (M : Model; A : Arm_Id; From, To : Rigid; Q0, Q1 : Real_Array; Lever : Real := 0.0; Position_Only : Boolean := False;
      Free_About : Vec3 := [0.0, 0.0, 0.0])
      return Real
   is
      use Driver.Numerics.Arrays;
      Worst : Real := 0.0;
   begin
      for K in 1 .. 3 loop
         declare
            Share : constant Real := Real (K) / 4.0;
            Here  : constant Real_Array (Q0'Range) := [for I in Q0'Range => Q0 (I) + Share * (Q1 (I) - Q0 (I))];
            Is_At : constant Rigid := Kinematics.Eye_In_Reference (M, A, Here);
            Want  : constant Rigid :=
              (Rotation    => From.Rotation
                                * Driver.Numerics.Exp (Share * Driver.Numerics.Log (Transpose (From.Rotation) * To.Rotation)),
               Translation => (1.0 - Share) * From.Translation + Share * To.Translation);
         begin
            Worst := Real'Max (Worst, abs (Is_At.Translation - Want.Translation));
            if Position_Only then
               null;
            elsif abs Free_About > 0.0 then
               Worst := Real'Max
                 (Worst, abs Driver.Numerics.Cross (Is_At.Rotation * Free_About, Want.Rotation * Free_About)
                         / (abs Free_About) ** 2 * Lever);
            else
               Worst := Real'Max (Worst, Driver.Numerics.Angle (Transpose (Is_At.Rotation) * Want.Rotation) * Lever);
            end if;
         end;
      end loop;
      return Worst;
   end Stray;

   --  Whether a path passes an end a channel has shown (Driver.Robot.End_Of), by more than the end's sigma and the
   --  waypoint's (the fit's own floor, Floor, as a segment the solver closes is within it); says which and where.
   function Passes_An_End (M : Model; G : Group_Id; P : Plan; Floor : Real; Why : out Unbounded_String) return Boolean is
   begin
      for W of P.Waypoints loop
         for C in W'Range loop
            for Way in Sense loop
               declare
                  Stop : constant Estimate := End_Of (M, G, C, Way);
                  Past : constant Real := (if Way = Increasing then W (C) - Stop.Value else Stop.Value - W (C));
               begin
                  --  Past the end, not short of it by as much (Significant is two-sided).
                  if Known (Stop) and then Past > 0.0
                    and then Driver.Uncertain.Significant (Past, Sqrt (Stop.Sigma ** 2 + Floor ** 2))
                  then
                     Why := To_Unbounded_String
                       ("channel" & C'Image & " of group" & G'Image & " would pass its end, "
                        & (if Way = Increasing then "upwards" else "downwards") & " at " & Driver.Log.Image (Stop.Value, 6)
                        & " where its stops were found, going to " & Driver.Log.Image (W (C), 6));
                     return True;
                  end if;
               end;
            end loop;
         end loop;
      end loop;
      return False;
   end Passes_An_End;

   function Plan_Reach (M : Model; A : Arm_Id; O : Observation; Goal : Pose_Goal;
                        Clearance : Real := Real'Last; Lever : Real := 0.0) return Plan is
      use Driver.Numerics.Arrays;
      Placement : Rigid;
      Scale     : Real;
      Known     : Boolean;
   begin
      if Natural (A) > Arm_Count (M) or else not Kinematics.Fitted (M, A) then
         return Refused (Unmeasured, "the arm's kinematics are not measured yet");
      end if;
      Kinematics.In_World (M, A, Placement, Scale, Known);
      if not Known then
         return Refused (Unmeasured, "the arm is not measured into the world yet");
      end if;
      --  X_world = Placement * (Scale * X_arm).
      declare
         Back   : constant Mat3 := Transpose (Placement.Rotation);
         Result : Plan :=
           Plan_Reach_In_Arm
             (M, A, O,
              (Pose          => (Rotation    => Back * Goal.Pose.Rotation,
                                 Translation => (1.0 / Scale) * (Back * (Goal.Pose.Translation - Placement.Translation))),
               Position_Only => Goal.Position_Only,
               Free_About    => Goal.Free_About),
              Clearance => (if Clearance < Real'Last then Clearance / Scale else Real'Last),
              Lever     => Lever / Scale);
      begin
         Result.Bow := Scale * Result.Bow;   --  the bow left, in the goal's unit
         return Result;
      end;
   end Plan_Reach;

   function Plan_Reach_In_Arm (M : Model; A : Arm_Id; O : Observation; Goal : Pose_Goal;
                               Clearance : Real := Real'Last; Lever : Real := 0.0;
                               Goal_Turn_Sigma : Real := 0.0) return Plan is
      use Driver.Numerics.Arrays;
   begin
      if Natural (A) > Arm_Count (M) or else not Kinematics.Fitted (M, A) then
         return Refused (Unmeasured, "the arm's kinematics are not measured yet");
      end if;
      declare
         G    : constant Group_Id := Arm_Group (M, A);
         Size : constant Natural := Group_Size (M, G);
      begin
         if Size = 0 or else Natural (G) > Natural (O.Readings.Length) or else O.Readings.Element (G)'Length /= Size then
            return Refused (Unmeasured, "the arm's readings are missing at that beat");
         end if;
         declare
            --  The path is the fitted model's, within the readings the arm has
            --  moved through or beyond them: a joint's end is met where it is,
            --  as the step that meets it ends Blocked or Short.
            Sigma : constant Real := Kinematics.Angle_Sigma (M, A);
            Start : constant Real_Array (1 .. Size) := O.Readings.Element (G);
            From  : constant Rigid := Tool_In_Arm (M, A, O).Pose;
            Result : Plan := (State => Planned, Reason => Null_Unbounded_String, Group => G, others => <>);
            Failed : Boolean := False;
            Worst_Position, Worst_Turn : Real := 0.0;
            Failed_From : Real_Array (1 .. Size) := Start;   --  the readings the segment that failed was solved from
            Failed_At   : Real := 0.0;                       --  and how far along the path it began

            --  The pose a fraction of the way from the start to the goal:
            --  straight in position, about one axis in turn.
            function Along (S : Real) return Rigid is
              ((Rotation    => From.Rotation
                                 * Driver.Numerics.Exp (S * Driver.Numerics.Log (Transpose (From.Rotation)
                                                                                   * Goal.Pose.Rotation)),
                Translation => (1.0 - S) * From.Translation + S * Goal.Pose.Translation));

            --  How far the whole path goes, in position and in turn: a segment
            --  of it shorter than the fit can tell apart (Sigma) is not cut
            --  again, for what it would learn is nothing.
            Total : constant Real :=
              Real'Max (abs (Goal.Pose.Translation - From.Translation),
                        Driver.Numerics.Angle (Transpose (From.Rotation) * Goal.Pose.Rotation));

            function Can_Cut (A_Of, B_Of : Real; Depth : Natural) return Boolean is
              (Depth < Real'Machine_Mantissa and then (B_Of - A_Of) * Total > Sigma);

            --  Solves the path from Fraction A (readings Q) to B; a segment the
            --  solver cannot close from where the last one ended is halved,
            --  and so is one it closes along a bow above the clearance, while
            --  it is longer than the fit can tell apart, at most as many times
            --  as a float has bits. A bow that is left is noted.
            procedure Reach (A_Of, B_Of : Real; Q : in out Real_Array; Depth : Natural) is
               Next : Real_Array (1 .. Size);
               Position_Off, Turn_Off : Real;
            begin
               Result.Solves := Result.Solves + 1;
               --  A goal whose turn is free leaves it free along the way too: held at the start's on every segment
               --  but the last, a raise of the hand's own eye near the wrist's singularity (conditioning 1e-6) was
               --  refused for a turn 2.26e-3 rad off at a third of the way, and no hand was made (f16f on 10-15).
               Kinematics.Solve_Pose (M, A, Q, Along (B_Of), Goal.Position_Only, Next, Position_Off, Turn_Off, Goal.Free_About);
               if not Driver.Uncertain.Significant (Position_Off, Sigma)
                 and then not Driver.Uncertain.Significant (Turn_Off, Sqrt (Sigma ** 2 + Goal_Turn_Sigma ** 2))
               then
                  declare
                     Bow : constant Real :=
                       (if Clearance < Real'Last
                        then Stray (M, A, Along (A_Of), Along (B_Of), Q, Next, Lever, Goal.Position_Only, Goal.Free_About)
                        else 0.0);
                  begin
                     if Bow > Clearance and then Can_Cut (A_Of, B_Of, Depth) then
                        declare
                           Kept     : constant Natural := Natural (Result.Waypoints.Length);
                           Kept_Bow : constant Real := Result.Bow;
                        begin
                           Reach (A_Of, (A_Of + B_Of) / 2.0, Q, Depth + 1);
                           if not Failed then
                              Reach ((A_Of + B_Of) / 2.0, B_Of, Q, Depth + 1);
                           end if;
                           if Failed then
                              --  The straight path cannot be followed all the way (it crosses what
                              --  the arm cannot reach): the joints' own line, which does, is taken,
                              --  with the bow it has.
                              Failed := False;
                              Result.Waypoints.Delete_Last
                                (Ada.Containers.Count_Type (Natural (Result.Waypoints.Length) - Kept));
                              Result.Waypoints.Append (Next);
                              Result.Bow := Real'Max (Kept_Bow, Bow);
                              Q := Next;
                           end if;
                        end;
                     else
                        Result.Waypoints.Append (Next);
                        if Bow > Clearance then
                           Result.Bow := Real'Max (Result.Bow, Bow);
                        end if;
                        Q := Next;
                     end if;
                  end;
               elsif Can_Cut (A_Of, B_Of, Depth) then
                  Reach (A_Of, (A_Of + B_Of) / 2.0, Q, Depth + 1);
                  if not Failed then
                     Reach ((A_Of + B_Of) / 2.0, B_Of, Q, Depth + 1);
                  end if;
               else
                  Failed := True;
                  Worst_Position := Position_Off;
                  Worst_Turn := Turn_Off;
                  Failed_From := Q;
                  Failed_At := A_Of;
               end if;
            end Reach;

            Q : Real_Array (1 .. Size) := Start;
         begin
            if Sigma = Real'Last then
               return Refused (Unmeasured, "the arm's fit has no uncertainty");
            end if;
            Reach (0.0, 1.0, Q, 0);
            if Failed then
               return (State => Unreachable,
                       Reason => To_Unbounded_String ("the arm's fitted model leaves the goal"
                                                      & Real'Image (Worst_Position) & " model units and"
                                                      & Real'Image (Worst_Turn) & " rad away, from "
                                                      & Driver.Log.Image (Failed_At, 4) & " of the way, at readings"
                                                      & Readings_Image (Failed_From) & " where the arm's conditioning is "
                                                      & Driver.Log.Image (Kinematics.Conditioning (M, A, Failed_From), 6)),
                       Solves => Result.Solves, others => <>);
            end if;
            declare
               Why : Unbounded_String;
            begin
               if Passes_An_End (M, G, Result, Sigma, Why) then
                  return (State => Unreachable, Reason => Why, Solves => Result.Solves, others => <>);
               end if;
            end;
            return Result;
         end;
      end;
   end Plan_Reach_In_Arm;

   function Status (P : Plan) return Plan_Status is (P.State);
   function Why (P : Plan) return String is (To_String (P.Reason));
   function Last_Readings (P : Plan) return Real_Array is (P.Waypoints.Last_Element);
   function Waypoint_Count (P : Plan) return Natural is (Natural (P.Waypoints.Length));
   function Waypoint (P : Plan; K : Positive) return Real_Array is (P.Waypoints (K));
   function Solve_Count (P : Plan) return Natural is (P.Solves);
   function Worst_Bow (P : Plan) return Real is (P.Bow);

   procedure Follow
     (M      : in out Model;
      P      : Plan;
      Report : out Step_Report;
      Stop   : access function return Boolean := null)
   is
   begin
      Report := (Outcome => Reached, others => <>);
      for K in P.Waypoints.First_Index .. P.Waypoints.Last_Index loop
         declare
            C : Driver.Commands.Command;
         begin
            Driver.Commands.Set_Target (C, P.Group, P.Waypoints (K));
            Step (M, C, Report, Stop);
         end;
         --  Which waypoint of how many the path ended at, and how: the step's own account of its channels.
         if Report.Outcome /= Reached and then not Report.Stopped then
            Driver.Log.Line (Driver.Log.Robot, "motion: a path of" & Natural'Image (Natural (P.Waypoints.Length))
                             & " waypoints ended at waypoint" & Natural'Image (K - P.Waypoints.First_Index + 1) & ", "
                             & Report.Outcome'Image & " after" & Report.Beats'Image & " beats: "
                             & To_String (Report.Detail));
         end if;
         exit when Report.Outcome /= Reached or else Report.Stopped;
      end loop;
   end Follow;

end Driver.Robot.Motion;
