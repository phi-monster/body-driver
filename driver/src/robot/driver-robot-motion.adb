with Driver.Beats;
with Driver.Clock;
with Driver.Log;
with Driver.Robot.Steps;

package body Driver.Robot.Motion is


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
      B      : Driver.Clock.Beat;
      Groups : constant Natural := Group_Count (M);
      --  How many pushes each group had before this step; a push of the step
      --  is the one after them.
      Before : Count_Array (1 .. Groups) := [others => 0];
      Waited : Natural := 0;

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
      Report := (others => <>);
      Driver.Beats.Next (B);
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
            end if;
            Driver.Beats.Send (Driver.Commands.Hold);
            exit when Finished;
         end;
      end loop;
   end Step;

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
