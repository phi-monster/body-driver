with Ada.Strings.Unbounded;
with Driver.Tests;

package body Driver.Twins.Tests is

   use Ada.Strings.Unbounded;
   use Driver.Tests;

   --  One command key of one channel, its first group at "a/k" and its second at "b/k".
   function Pair_Layout return Driver.Observations.Layout is
      L : Driver.Observations.Layout;
   begin
      L.Groups.Append
        (Driver.Observations.Group_Info'(Path        => To_Unbounded_String ("a/k"),
                                         Keys        => To_Unbounded_String ("a" & Driver.Observations.Key_Separator & "k"),
                                         Size        => 1,
                                         Command_Key => To_Unbounded_String ("k"),
                                         Echo_Path   => To_Unbounded_String ("b/k"),
                                         Echo_Keys   => To_Unbounded_String ("b" & Driver.Observations.Key_Separator & "k")));
      return L;
   end Pair_Layout;

   function Beat_Of (First, Second : Real) return Driver.Observations.Observation is
      O : Driver.Observations.Observation;
   begin
      O.Readings.Append (Real_Array'[1 => First]);
      O.Echoes.Append (Real_Array'[1 => Second]);
      O.Turned.Append (False);
      return O;
   end Beat_Of;

   function Sent_Of (Value : Real) return Driver.Commands.Command is
      C : Driver.Commands.Command;
   begin
      Driver.Commands.Set_Target (C, 1, [1 => Value]);
      return C;
   end Sent_Of;

   type Beat_Kind is record
      First, Second, Sent : Real;
   end record;
   type Stream_Kind is array (Positive range <>) of Beat_Kind;

   --  Runs the stream; the readings the driver would take at each beat, and the beats Turned was set at.
   procedure Run (Stream : Stream_Kind; Readings : out Stream_Kind; Turns : out Natural; Last_Turn : out Natural;
                  W : out Watch) is
      L : constant Driver.Observations.Layout := Pair_Layout;
   begin
      Turns := 0;
      Last_Turn := 0;
      for B in Stream'Range loop
         declare
            O : Driver.Observations.Observation := Beat_Of (Stream (B).First, Stream (B).Second);
         begin
            Order (W, L, O, Sent_Of (Stream (B).Sent));
            Readings (B) := (First => O.Readings (1) (1), Second => O.Echoes (1) (1), Sent => Stream (B).Sent);
            if O.Turned (1) then
               Turns := Turns + 1;
               Last_Turn := B;
            end if;
         end;
      end loop;
   end Run;

   procedure Echo_First is
      --  A92: the echo came first in the observation. The command holds at 0.5; the body, read second, stands at
      --  0.49 and jitters in its last places; the command then steps to 0.6, the echo with it at once, the body
      --  after. The body is the reading from the beat it moved while the command held.
      Stream : constant Stream_Kind :=
        [(0.5, 0.49, 0.5), (0.5, 0.49, 0.5), (0.5, 0.490000000000001, 0.5), (0.5, 0.490000000000001, 0.5),
         (0.6, 0.55, 0.6), (0.6, 0.59, 0.6)];
      Got  : Stream_Kind (Stream'Range);
      Turns, At_Beat : Natural;
      W    : Watch;
   begin
      Run (Stream, Got, Turns, At_Beat, W);
      Check (Told_Apart (W, 1) and then Reading_Is_Echo_Path (W, 1), "the echo given first was not found to be the echo");
      Check (Turns = 1 and then At_Beat = 3, "the pair was turned" & Turns'Image & " times, last at beat" & At_Beat'Image
             & ", not once at beat 3, where the body moved while the command held");
      Check (Got (6).First = 0.59 and then Got (6).Second = 0.6, "after the turn the readings are not the body's");
   end Echo_First;

   procedure Reading_First is
      --  The order the layout read: the reading first, the echo second; nothing is turned.
      Stream : constant Stream_Kind :=
        [(0.49, 0.5, 0.5), (0.490000000000001, 0.5, 0.5), (0.55, 0.6, 0.6), (0.59, 0.6, 0.6)];
      Got  : Stream_Kind (Stream'Range);
      Turns, At_Beat : Natural;
      W    : Watch;
   begin
      Run (Stream, Got, Turns, At_Beat, W);
      Check (Told_Apart (W, 1) and then not Reading_Is_Echo_Path (W, 1), "the reading given first was taken for the echo");
      Check (Turns = 0, "a pair in the layout's order was turned");
      Check (Got (4).First = 0.59, "the readings are not the body's");
   end Reading_First;

   procedure Reading_A_Beat_Late is
      --  A closer whose reading is the command it received the beat before (RoboDojo's: the gripper's state is the
      --  previous control): both stand while the command holds; when the command steps the echo steps with it and
      --  the reading a beat later. The reading stood at the beat the command changed: no echo.
      Stream : constant Stream_Kind := [(1.0, 1.0, 1.0), (1.0, 1.0, 1.0), (0.0, 1.0, 0.0), (0.0, 0.0, 0.0)];
      Got  : Stream_Kind (Stream'Range);
      Turns, At_Beat : Natural;
      W    : Watch;
   begin
      Run (Stream, Got, Turns, At_Beat, W);
      Check (Told_Apart (W, 1) and then Reading_Is_Echo_Path (W, 1),
             "the reading a beat late, given second, was not found to be the reading");
      Check (Turns = 1 and then At_Beat = 3, "the pair was not turned at the beat the command stepped");
   end Reading_A_Beat_Late;

   procedure Two_Readings is
      --  Two groups that both move while the command holds: neither is an echo, nothing is told apart, and the
      --  first stands for the reading.
      Stream : constant Stream_Kind := [(0.1, 0.2, 0.5), (0.11, 0.21, 0.5), (0.12, 0.22, 0.5)];
      Got  : Stream_Kind (Stream'Range);
      Turns, At_Beat : Natural;
      W    : Watch;
   begin
      Run (Stream, Got, Turns, At_Beat, W);
      Check (not Told_Apart (W, 1) and then Turns = 0, "two readings were told apart as a reading and an echo");
      Check (Got (3).First = 0.12, "the first group does not stand for the reading");
   end Two_Readings;

   procedure Register is
   begin
      Driver.Tests.Register ("twins.echo_first", "an echo given before the reading is read for the joints, or the pair "
                             & "is turned at another beat than the one that told it", Echo_First'Access);
      Driver.Tests.Register ("twins.reading_first", "a reading given first is taken for the echo", Reading_First'Access);
      Driver.Tests.Register ("twins.late", "a reading that is the command of the beat before is taken for the echo",
                             Reading_A_Beat_Late'Access);
      Driver.Tests.Register ("twins.neither", "two groups that both move are told apart as a reading and an echo",
                             Two_Readings'Access);
   end Register;

end Driver.Twins.Tests;
