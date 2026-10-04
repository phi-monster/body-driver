with Driver.Beats;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Images;
with Driver.Observations;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.Tests;
with Driver.World;

package body Driver.Action.Plants.Live.Tests is

   use Driver.Tests;

   --  A body that has measured nothing yet: one group of two readings that
   --  takes commands, one eye showing a grey picture. Every beat the robot
   --  reaches what it was sent.
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
                                     Check_Ok : out Boolean; Finished_Run : out Boolean; Beats : out Natural;
                                     Moved : out Boolean)
   is
      M : aliased Driver.Robot.Model;
      H : aliased Driver.Robot.Hand.Hands;
      S : aliased Driver.World.Scene;
      Wanted : constant Want := Want_For (Kind);
      Got : Result;
      Finished : Boolean := False with Atomic;
      Verdict_Ok : Boolean := True with Atomic;

      task Decider;
      task body Decider is
         C : Context (M'Access, H'Access, S'Access);
         procedure Gate is
            V : constant Verdict := Check (C, Wanted);
         begin
            Verdict_Ok := V.Ok;
         end Gate;
      begin
         Driver.Beats.Within_A_Beat (Gate'Access);
         Execute (C, Wanted, Got);
         Finished := True;
      exception
         when others =>
            Driver.Beats.Release;
            Finished := True;
      end Decider;

      Side  : constant := 8;
      Bytes : constant := 3 * Side * Side;
      Grey  : constant Driver.Bytes.Byte_Array (1 .. Bytes) := [others => 128];
      Sent : Driver.Commands.Command;
      Now  : Real_Array (1 .. 2) := [0.0, 0.0];
      Bound : constant := 400;
   begin
      Beats := 0;
      Moved := False;
      for B in 0 .. Bound loop
         exit when Finished;
         declare
            O       : Driver.Observations.Observation;
            Took    : Boolean := False;
            Pending : Driver.Commands.Command;
         begin
            O.Beat := Driver.Clock.Beat (B);
            O.Images.Append (Driver.Images.Create (8, 8, Grey));
            O.Depth.Append (Real_Array'(1 .. 0 => 0.0));
            O.Readings.Append (Now);
            O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
            if B = 0 then
               Driver.Commands.Set_Target (Sent, 1, Now);
            end if;
            Driver.Robot.Observe (M, O, Sent);
            Driver.Robot.Hand.Observe (H, M, O, Sent);
            Driver.World.Observe (S, M, H, O, Sent);
            loop
               Driver.Beats.Offer (O.Beat, O, Sent, Took);
               exit when Took or else Finished;
               delay 0.0;
            end loop;
            exit when not Took;
            Driver.Beats.Await (Pending);
            Moved := Moved or else not Driver.Commands.Is_Hold (Pending);
            if Driver.Commands.Has_Target (Pending, 1) then
               Driver.Commands.Set_Target (Sent, 1, Driver.Commands.Target (Pending, 1));
               Now := Driver.Commands.Target (Sent, 1);
            end if;
            Beats := B + 1;
         end;
      end loop;
      if not Finished then
         abort Decider;
      end if;
      Finished_Run := Finished;
      Final := Got.Final;
      Tried := Got.Tried;
      Check_Ok := Verdict_Ok;
   end Run_On_Unmeasured_Body;

   procedure Unmeasured_Body_Refuses is
   begin
      for Kind in Run_Kind loop
         declare
            Final : Ending;
            Tried : Unbounded_String;
            Ok, Done, Moved : Boolean;
            Beats : Natural;
         begin
            Run_On_Unmeasured_Body (Kind, Final, Tried, Ok, Done, Beats, Moved);
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

   procedure Register is
   begin
      Register ("action.live.unmeasured", "the action layer over the real lower layers deadlocks with the main loop, "
                & "raises, or moves on a body that has measured nothing", Unmeasured_Body_Refuses'Access);
   end Register;

end Driver.Action.Plants.Live.Tests;
