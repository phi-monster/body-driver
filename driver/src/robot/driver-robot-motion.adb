with Ada.Numerics.Long_Elementary_Functions;
with Driver.Beats;
with Driver.Clock;
with Driver.Log;
with Driver.Robot.Channels;
with Driver.Robot.Steps;
with Driver.Robot.Lockin;

package body Driver.Robot.Motion is

   use Ada.Numerics.Long_Elementary_Functions;


   type Count_Array is array (Positive range <>) of Natural;

   procedure Settle (M : in out Model; Beats_Waited : out Natural) is
      B : Driver.Clock.Beat;
   begin
      Beats_Waited := 0;
      loop
         Driver.Beats.Next (B);
         declare
            Done : constant Boolean := Still (M);
         begin
            Driver.Beats.Send (Driver.Commands.Hold);
            exit when Done;
         end;
         Beats_Waited := Beats_Waited + 1;
      end loop;
   end Settle;

   --  Some eye's image moved, as the lock-in can tell, at a beat after Beat.
   function Seen_Since (M : Model; Beat : Natural) return Boolean is
   begin
      for E in 1 .. Eye_Count (M) loop
         for B in Beat + 1 .. M.Beats - 1 loop
            if Lockin.Moved (M, Eye_Id (E), B) then
               return True;
            end if;
         end loop;
      end loop;
      return False;
   end Seen_Since;

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
               Append (Report.Detail, "group" & G'Image & ": delivered "
                       & Driver.Log.Image (E.Delivered.Value, 3) & " of " & Driver.Log.Image (E.Length, 4)
                       & (if E.Blocked then ", blocked" else "") & "; ");
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

   procedure Step (M : in out Model; Targets : Driver.Commands.Command; Report : out Step_Report) is
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
         Started : constant Natural := M.Beats;

         function Targeted (G : Group_Id) return Boolean is
           (Is_Commandable (M, G) and then Driver.Commands.Has_Target (Targets, G));

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
         end loop;
         Driver.Beats.Send (Targets);
         --  The push starts at the beat whose observation was taken under the
         --  new targets; it is over when the step tracker has seen it end.
         loop
            Driver.Beats.Next (B);
            Waited := Waited + 1;
            declare
               Finished : constant Boolean := Done;
            begin
               if Finished then
                  Judge (M, Targets, Before, Waited, Report);
                  Report.Started := Started;
               end if;
               Driver.Beats.Send (Driver.Commands.Hold);
               exit when Finished;
            end;
         end loop;
      end;
   end Step;

   procedure Hold (M : in out Model; Beats : Positive) is
      pragma Unreferenced (M);
      B : Driver.Clock.Beat;
   begin
      for K in 1 .. Beats loop
         Driver.Beats.Next (B);
         Driver.Beats.Send (Driver.Commands.Hold);
      end loop;
   end Hold;

   procedure Probe (M : in out Model; G : Group_Id; Channel : Positive; Direction : Real; Report : out Probe_Report) is
      B         : Driver.Clock.Beat;
      Sign      : constant Real := (if Direction > 0.0 then 1.0 else -1.0);
      Size      : Natural := 0;
      Step_Size : Real := 0.0;
   begin
      Report := (others => <>);
      --  Where the group reads, and the first step, read in a held beat.
      Driver.Beats.Next (B);
      if Is_Commandable (M, G) and then Channel <= Group_Size (M, G)
        and then Natural (G) <= Natural (Driver.Beats.Latest.Readings.Length)
        and then Driver.Beats.Latest.Readings.Element (G)'Length = Group_Size (M, G)
      then
         Size := Group_Size (M, G);
      end if;
      declare
         Start : Real_Array (1 .. Size) := [others => 0.0];
         Offset : Real := 0.0;
      begin
         if Size > 0 then
            Start := Driver.Beats.Latest.Readings.Element (G);
            declare
               Sigma : constant Real := Reading_Noise (M, G, Channel);
               Scale : Real := 0.0;
            begin
               if Sigma > 0.0 and then Sigma < Real'Last then
                  --  The smallest change of the reading that tells from its
                  --  noise: a change is the difference of two readings.
                  Step_Size := Driver.Uncertain.Threshold
                    (Driver.Uncertain.Scalar_Gate (Channels.Noise_Freedom (M, G, Channel))) * Sigma * Sqrt (2.0);
               else
                  --  No jitter to scale by: the resolution of the readings'
                  --  size, or of one reading unit when every reading is zero.
                  for V of Start loop
                     Scale := Real'Max (Scale, abs V);
                  end loop;
                  Step_Size := Real'Model_Epsilon * (if Scale > 0.0 then Scale else 1.0);
               end if;
            end;
         end if;
         Driver.Beats.Send (Driver.Commands.Hold);
         if Size = 0 then
            return;
         end if;
         --  As many doublings as a float has bits of precision take a step
         --  from the resolution of a reading past the reading's whole size: a
         --  joint is blocked long before, and a channel still unseen then
         --  moves nothing any eye sees.
         for Doubling in 1 .. Real'Machine_Mantissa loop
            declare
               Target : Real_Array := Start;
               C      : Driver.Commands.Command;
               Looked : Natural := 0;
               Lag    : Integer := 0;
            begin
               Offset := Offset + Sign * Step_Size;
               Target (Channel) := Start (Channel) + Offset;
               Driver.Commands.Set_Target (C, G, Target);
               Step (M, C, Report.Last);
               Report.Steps := Doubling;
               Report.Excursion := abs Offset;
               --  The eyes show the step a lag later: look in held beats until
               --  it has had time to show.
               loop
                  Driver.Beats.Next (B);
                  for E in 1 .. Eye_Count (M) loop
                     Lag := Integer'Max (Lag, Image_Lag (M, Eye_Id (E)));
                  end loop;
                  Report.Seen := Seen_Since (M, Report.Last.Started);
                  Driver.Beats.Send (Driver.Commands.Hold);
                  Looked := Looked + 1;
                  exit when Report.Seen or else Looked >= Lag;
               end loop;
               exit when Report.Seen or else Report.Last.Outcome /= Reached;
               Step_Size := 2.0 * Step_Size;
            end;
         end loop;
      end;
   end Probe;

   function Plan_Reach (M : Model; A : Arm_Id; O : Observation; Goal : Pose_Goal) return Plan is
      pragma Unreferenced (M, A, O, Goal);
   begin
      return (State => Unmeasured, Reason => To_Unbounded_String ("the arm's kinematics are not measured yet"));
   end Plan_Reach;

   function Status (P : Plan) return Plan_Status is (P.State);
   function Why (P : Plan) return String is (To_String (P.Reason));

   procedure Follow (M : in out Model; P : Plan; Report : out Step_Report) is
      pragma Unreferenced (M, P);
   begin
      --  Only a planned path is followed (the precondition); none can be
      --  planned before the kinematics are measured.
      Report := (others => <>);
   end Follow;

end Driver.Robot.Motion;
