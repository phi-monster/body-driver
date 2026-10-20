--  driver-robot-beacon_replay DUMP [FX FY CX CY]: the fit of an eye fixed in the world to the flicks of an arm's closer
--  (Kinematics.Beacon.Fit_Eye) made again from a dump of its sightings, which the driver writes with BD_DUMP_FITS naming
--  a folder. The fit is made on the first K sightings for K growing to all of them, as the boot would have had them
--  when it had that many, so that how many flicks the standard needs shows. With the true lens (the fake body's
--  camera line) each line says how far the fitted lens is from it.
procedure Driver.Robot.Beacon_Replay;
