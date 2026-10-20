--  The inputs of a fixed eye's fit to the points it tracked on the arms (Hand_Eye.Fit_Eye), written to a file and read
--  back, for a fit studied alone in seconds (tools/driver-robot-hand_eye_replay) instead of in a boot.

package Driver.Robot.Kinematics.Hand_Eye.Dumps is

   procedure Dump (Arms : Watched_Array; Width, Height : Positive; Name : String);
   --  With BD_DUMP_FITS naming a folder, the arms' inputs in a file handeye_NAME_N.txt there, N counting the dumps of
   --  the run, every real to its last bit; nothing without the variable.

   type Loaded (Count : Natural) is record
      Width, Height : Positive := 1;
      Arms          : Watched_Array (1 .. Count);
   end record;

   function Load (Path : String) return Loaded;
   --  A file Dump wrote. Raises Ada.IO_Exceptions.Data_Error on a file it did not write.

end Driver.Robot.Kinematics.Hand_Eye.Dumps;
