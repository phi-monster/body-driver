--  body_driver --listen PORT [--eye HOST:PORT] [--inst HOST:PORT] [--body FILE]
--
--  The main loop owns the connection to the robot: every beat it reads one
--  observation, runs every estimator, offers the beat to the decider task
--  (boot, then rounds with the brain) and replies with the decider's command,
--  or Hold when the decider is busy. The loop is being built with the core
--  transport (Driver.Wire, Driver.Msgpack, Driver.Replies).

with Ada.Command_Line;
with Driver.Log;

procedure Body_Driver is
begin
   Driver.Log.Line (Driver.Log.Core, "the main loop is not built yet");
   Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
end Body_Driver;
