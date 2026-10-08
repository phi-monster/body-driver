with Ada.Strings.Fixed;
with Driver.Beats;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Images;
with Driver.Observations;
with Driver.Recording;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.Tests;
with Driver.World;
with GNAT.OS_Lib;

package body Driver.Action.Plants.Live.Tests is

   use Driver.Tests;
   use Driver.Uncertain;
   use type Driver.Recording.Record_Kind;
   use type GNAT.OS_Lib.File_Descriptor;
   use type GNAT.OS_Lib.String_Access;

   protected type Flag is
      procedure Raise_It;
      function Is_Up return Boolean;
   private
      Up : Boolean := False;
   end Flag;

   protected body Flag is
      procedure Raise_It is
      begin
         Up := True;
      end Raise_It;

      function Is_Up return Boolean is (Up);
   end Flag;

   --  The main loop played by the test over a body that has measured
   --  nothing: one group of two readings that takes commands, one eye
   --  showing a grey picture. Every beat the robot reaches what it was sent.
   type Main_Loop is limited record
      M     : aliased Driver.Robot.Model;
      H     : aliased Driver.Robot.Hand.Hands;
      S     : aliased Driver.World.Scene;
      Sent  : Driver.Commands.Command;
      Now   : Real_Array (1 .. 2) := [0.0, 0.0];
      Moved : Boolean := False;   --  some reply carried a command
      Next  : Natural := 0;       --  the beat to make next
   end record;

   Side  : constant := 8;
   Bytes : constant := 3 * Side * Side;
   Grey  : constant Driver.Bytes.Byte_Array (1 .. Bytes) := [others => 128];

   function Observation_Of (L : Main_Loop) return Driver.Observations.Observation is
      O : Driver.Observations.Observation;
   begin
      O.Beat := Driver.Clock.Beat (L.Next);
      O.Images.Append (Driver.Images.Create (Side, Side, Grey));
      O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
      O.Readings.Append (L.Now);
      O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
      return O;
   end Observation_Of;

   --  Every estimator observes the beat, with the command in effect.
   procedure Observe (L : in out Main_Loop; O : Driver.Observations.Observation) is
   begin
      if L.Next = 0 then
         Driver.Commands.Set_Target (L.Sent, 1, L.Now);
      end if;
      Driver.Robot.Observe (L.M, O, L.Sent);
      Driver.Robot.Hand.Observe (L.H, L.M, O, L.Sent);
      Driver.World.Observe (L.S, L.M, L.H, O, L.Sent);
   end Observe;

   --  The beat is offered until the decider is waiting for it, or has
   --  finished; the reply is taken. A record of each kind goes into the
   --  recording, when there is one, as the main loop writes the messages it
   --  receives and sends: the decider's windows lie between the two.
   procedure Offer_Beat (L : in out Main_Loop; O : Driver.Observations.Observation; Done : Flag; Took : out Boolean) is
      Pending : Driver.Commands.Command;
   begin
      Took := False;
      Driver.Recording.Write_Shared (Driver.Recording.Robot_Message, Driver.Bytes.To_Bytes ("beat"));
      loop
         Driver.Beats.Offer (O.Beat, O, L.Sent, Took);
         exit when Took or else Done.Is_Up;
         delay 0.0;
      end loop;
      if Took then
         Driver.Beats.Await (Pending);
         L.Moved := L.Moved or else not Driver.Commands.Is_Hold (Pending);
         if Driver.Commands.Has_Target (Pending, 1) then
            Driver.Commands.Set_Target (L.Sent, 1, Driver.Commands.Target (Pending, 1));
            L.Now := Driver.Commands.Target (L.Sent, 1);
         end if;
      end if;
      Driver.Recording.Write_Shared (Driver.Recording.Driver_Message, Driver.Bytes.To_Bytes ("reply"));
      L.Next := L.Next + 1;
   end Offer_Beat;

   procedure Beat (L : in out Main_Loop; Done : Flag; Took : out Boolean) is
      O : constant Driver.Observations.Observation := Observation_Of (L);
   begin
      Observe (L, O);
      Offer_Beat (L, O, Done, Took);
   end Beat;

   type Run_Report is record
      Finished : Boolean := False;   --  the decider's script ended
      Raised   : Boolean := False;   --  by raising
      Moved    : Boolean := False;   --  some beat's reply carried a command
   end record;

   --  The script runs as the decider over a live plant on the lower layers
   --  while the main loop feeds it beats, at most Bound of them.
   procedure Run (Work : not null access procedure (P : in out Live); Bound : Positive; R : out Run_Report) is
      L    : aliased Main_Loop;
      P    : aliased Live (L.M'Access, L.H'Access, L.S'Access);
      Done : Flag;
      Died : Flag;
      Took : Boolean;

      task Decider;
      task body Decider is
      begin
         Work (P);
         Done.Raise_It;
      exception
         when others =>
            Driver.Beats.Release;
            Died.Raise_It;
            Done.Raise_It;
      end Decider;
   begin
      for B in 1 .. Bound loop
         exit when Done.Is_Up;
         Beat (L, Done, Took);
         exit when not Took;
      end loop;
      if not Done.Is_Up then
         abort Decider;
      end if;
      R := (Finished => Done.Is_Up, Raised => Died.Is_Up, Moved => L.Moved);
   end Run;

   type Run_Kind is (Change_Height, Touch_With_Grasper);

   function Want_For (Kind : Run_Kind) return Want is
      Settles : Ending_Set := [others => False];
   begin
      Settles (Settled) := True;
      case Kind is
         when Change_Height =>
            return (Kind => Change, Until_Endings => Settles, Max_Steps => 0, Eye => Any_Eye, Anyway => False,
                    Thing => 1, Quantity => 1, Increase => True);
         when Touch_With_Grasper =>
            declare
               W : Want (Interval);
            begin
               W.Until_Endings := Settles;
               W.Constraints.Append (Constraint'(Subject  => (Kind => Role_Operand, The_Role => Grasper),
                                                 Relation => Touching,
                                                 Object   => (Kind => Thing_Operand, Thing => 1),
                                                 Step     => Unspecified, Strength => Unspecified, Must => False));
               return W;
            end;
      end case;
   end Want_For;

   procedure Run_On_Unmeasured_Body (Kind : Run_Kind; Final : out Ending; Tried : out Unbounded_String;
                                     Check_Ok : out Boolean; Finished_Run : out Boolean; Moved : out Boolean)
   is
      Wanted : constant Want := Want_For (Kind);
      Got    : Result;
      Gated  : Flag;
      Passed : Flag;

      procedure Script (P : in out Live) is
         C : Context (P.Robot, P.Hands, P.Scene);
         procedure Gate is
            V : constant Verdict := Check (C, Wanted);
         begin
            Gated.Raise_It;
            if V.Ok then
               Passed.Raise_It;
            end if;
         end Gate;
      begin
         Driver.Beats.Within_A_Beat (Gate'Access);
         Execute (C, Wanted, Got);
      end Script;

      R : Run_Report;
   begin
      Run (Script'Access, 400, R);
      Moved := R.Moved;
      Finished_Run := R.Finished and then not R.Raised;
      Final := Got.Final;
      Tried := Got.Tried;
      Check_Ok := Gated.Is_Up and then Passed.Is_Up;
   end Run_On_Unmeasured_Body;

   procedure Unmeasured_Body_Refuses is
   begin
      for Kind in Run_Kind loop
         declare
            Final : Ending;
            Tried : Unbounded_String;
            Ok, Done, Moved : Boolean;
         begin
            Run_On_Unmeasured_Body (Kind, Final, Tried, Ok, Done, Moved);
            Check (not Moved, "the action layer sent targets to a body that has measured nothing ("
                   & Run_Kind'Image (Kind) & ")");
            Check (Done, "the action layer does not finish on a body that has measured nothing ("
                   & Run_Kind'Image (Kind) & ")");
            Check (not Ok, "the gates pass a want on a body that has measured nothing (" & Run_Kind'Image (Kind) & ")");
            Check (Final = Refused and then Length (Tried) > 0,
                   "a want on a body that has measured nothing is not refused with what was tried ("
                   & Run_Kind'Image (Kind) & "): " & Ending'Image (Final));
         end;
      end loop;
   end Unmeasured_Body_Refuses;

   --  What the models answer is asked inside a beat's window, where the main
   --  loop holds them still. Asked outside one, a reach, a view and a
   --  prediction are refused; and what takes a beat of its own is refused
   --  inside one, where it would wait for ever for the beat the window holds.
   Reach_Refused, View_Refused, Prediction_Refused : Flag;
   Reach_Said, View_Said, Prediction_Said          : Flag;   --  answered outside a window
   Reach_Answered, View_Answered                   : Flag;   --  answered inside one
   Look_Refused, Move_Refused, Learn_Refused       : Flag;
   Window_Refused, Window_Reusable                 : Flag;

   procedure Nothing is null;
   procedure Used (Answer : Boolean) is null;

   procedure Windows_Script (P : in out Live) is
      Goal : constant Arm_Goal := (Arm => 1, Tool => Identity, Position_Only => False, others => <>);
      Spot : constant Vec3 := [0.0, 0.0, 1.0];

      procedure Ask is
      begin
         if P.Reach (Goal).Status = Unmeasured then
            Reach_Answered.Raise_It;
         end if;
         Used (P.In_View (Spot));
         View_Answered.Raise_It;
      end Ask;

      procedure Look_Inside is
         S : Snapshot;
      begin
         P.Look (S);
      end Look_Inside;

      procedure Move_Inside is
         R : Report;
      begin
         P.Move ((Arms => Arm_Goal_Vectors.Empty_Vector, Closers => Closer_Goal_Vectors.Empty_Vector, Settle => False),
                 R);
      end Move_Inside;

      procedure Learn_Inside is
      begin
         P.Learn ((Kind => Touched_At, Thing => 1, Point => (others => <>)));
      end Learn_Inside;

      procedure Nest is
      begin
         P.Within (Nothing'Access);
      end Nest;
   begin
      begin
         Used (P.Reach (Goal).Status = Reachable);
         Reach_Said.Raise_It;
      exception
         when Program_Error => Reach_Refused.Raise_It;
      end;
      begin
         Used (P.In_View (Spot));
         View_Said.Raise_It;
      exception
         when Program_Error => View_Refused.Raise_It;
      end;
      begin
         Used (Known (P.Predicted (1, 0)));
         Prediction_Said.Raise_It;
      exception
         when Program_Error => Prediction_Refused.Raise_It;
      end;
      P.Within (Ask'Access);
      begin
         P.Within (Look_Inside'Access);
      exception
         when Program_Error => Look_Refused.Raise_It;
      end;
      begin
         P.Within (Move_Inside'Access);
      exception
         when Program_Error => Move_Refused.Raise_It;
      end;
      begin
         P.Within (Learn_Inside'Access);
      exception
         when Program_Error => Learn_Refused.Raise_It;
      end;
      begin
         P.Within (Nest'Access);
      exception
         when Program_Error => Window_Refused.Raise_It;
      end;
      --  A refusal inside a window ends that window and leaves the next one.
      P.Within (Nothing'Access);
      Window_Reusable.Raise_It;
   end Windows_Script;

   procedure Windows_Are_Kept is
      R : Run_Report;
   begin
      Run (Windows_Script'Access, 400, R);
      Check (R.Finished and then not R.Raised, "the decider did not get through its windows");
      Check (Reach_Refused.Is_Up and then not Reach_Said.Is_Up,
             "a reach was answered outside a beat's window, where the main loop changes the models");
      Check (View_Refused.Is_Up and then not View_Said.Is_Up, "a view was answered outside a beat's window");
      Check (Prediction_Refused.Is_Up and then not Prediction_Said.Is_Up,
             "a prediction was answered outside a beat's window");
      Check (Reach_Answered.Is_Up and then View_Answered.Is_Up, "a reach or a view was not answered inside a window");
      Check (Look_Refused.Is_Up, "a look was taken inside a window, which holds the beat it waits for");
      Check (Move_Refused.Is_Up, "a move was taken inside a window");
      Check (Learn_Refused.Is_Up, "a lesson was taken inside a window");
      Check (Window_Refused.Is_Up, "a window was opened inside a window");
      Check (Window_Reusable.Is_Up, "a window refused inside a window left the plant without the next one");
   end Windows_Are_Kept;

   --  A decider's write into the world goes into the recording where the
   --  replay applies it: inside the beat that carries it, after the message
   --  that brought the beat in and before the reply. A lesson written outside
   --  a window lands wherever the main loop happens to be, between two beats
   --  here, while the main loop writes the world itself.
   procedure Lesson_Takes_The_Window is
      Waits : constant Duration := 0.2;
      L        : aliased Main_Loop;
      P        : aliased Live (L.M'Access, L.H'Access, L.S'Access);
      Done     : Flag;
      Learned  : Flag;
      Thing    : Driver.World.Thing_Id := 1;
      Region   : Driver.Images.Mask := Driver.Images.Create (Side, Side);
      First    : constant Driver.Observations.Observation := Observation_Of (L);
      Took     : Boolean;
      FD       : GNAT.OS_Lib.File_Descriptor;
      Name     : GNAT.OS_Lib.String_Access;
      Order    : Unbounded_String;   --  the kinds of the recording, one letter each
      Gone     : Boolean;

      task Decider is
         entry Go;
      end Decider;

      task body Decider is
      begin
         accept Go;
         P.Learn ((Kind  => Touched_At, Thing => Thing,
                   Point => (Mean => [0.1, 0.2, 0.3], Covariance => [others => [others => 0.0]])));
         Learned.Raise_It;
         Done.Raise_It;
      exception
         when others =>
            Driver.Beats.Release;
            Done.Raise_It;
      end Decider;
   begin
      GNAT.OS_Lib.Create_Temp_File (FD, Name);
      Check (FD /= GNAT.OS_Lib.Invalid_FD and then Name /= null, "no scratch file for the recording");
      if Name = null then
         abort Decider;
         return;
      end if;
      GNAT.OS_Lib.Close (FD);
      for Row in 2 .. 4 loop
         for Column in 2 .. 4 loop
            Driver.Images.Include (Region, Column, Row);
         end loop;
      end loop;
      Observe (L, First);
      Driver.World.Adopt (L.S, L.M, 1, First, Region, Thing);
      Driver.Recording.Start_Shared (Name.all);
      Decider.Go;
      --  The main loop makes no beat: a lesson that needs none is learned by now.
      delay Waits;
      Check (not Learned.Is_Up, "a lesson was written without a beat's window, whatever the main loop was doing");
      Offer_Beat (L, First, Done, Took);
      for B in 1 .. 100 loop
         exit when Done.Is_Up or else not Took;
         Beat (L, Done, Took);
      end loop;
      Driver.Recording.Stop_Shared;
      declare
         R       : Driver.Recording.Reader;
         Opened  : Boolean;
         More    : Boolean := True;
         Kind    : Driver.Recording.Record_Kind;
         Ns      : Long_Long_Integer;
         Payload : Driver.Bytes.Buffer;
      begin
         Driver.Recording.Open (R, Name.all, Opened);
         Check (Opened, "the recording cannot be opened");
         while Opened and then More loop
            Driver.Recording.Next (R, Kind, Ns, Payload, More);
            if More then
               Append (Order, (case Kind is
                                  when Driver.Recording.Robot_Message  => 'R',
                                  when Driver.Recording.Driver_Message => 'D',
                                  when Driver.Recording.World_Written  => 'W',
                                  when others                          => '?'));
            end if;
         end loop;
         if Opened then
            Driver.Recording.Close (R);
         end if;
      end;
      Check (Learned.Is_Up, "the lesson was not learned");
      Check (Ada.Strings.Fixed.Count (To_String (Order), "W") = 1,
             "the lesson is not in the recording once: " & To_String (Order));
      Check (Ada.Strings.Fixed.Index (To_String (Order), "RWD") > 0,
             "the lesson is not between the message that brought a beat and its reply: " & To_String (Order));
      GNAT.OS_Lib.Delete_File (Name.all, Gone);
      GNAT.OS_Lib.Free (Name);
   end Lesson_Takes_The_Window;

   procedure Register is
   begin
      Register ("action.live.unmeasured", "the action layer over the real lower layers deadlocks with the main loop, "
                & "raises, or moves on a body that has measured nothing", Unmeasured_Body_Refuses'Access);
      Register ("action.live.windows", "the live plant answers a reach, a view or a prediction outside a beat's "
                & "window, where the main loop changes the models, or takes a beat of its own inside one",
                Windows_Are_Kept'Access);
      Register ("action.live.lesson", "a lesson is written into the world outside a beat's window, so the recording "
                & "holds it where the replay does not apply it", Lesson_Takes_The_Window'Access);
   end Register;

end Driver.Action.Plants.Live.Tests;
