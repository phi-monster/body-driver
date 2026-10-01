--  replay RECORDING [--body FILE] [--out ESTIMATES.json]
--
--  Feeds a recording (harness/record/wire_proxy.py format) through every
--  estimator, exactly as the main loop would, and writes what was estimated
--  for scoring against simulator truth. Being built with the core transport.

with Ada.Command_Line;
with Driver.Log;

procedure Replay is
begin
   Driver.Log.Line (Driver.Log.Core, "replay is not built yet");
   Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
end Replay;
