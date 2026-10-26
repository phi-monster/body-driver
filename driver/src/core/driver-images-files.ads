--  Pictures in files: the portable pixmap (binary PPM, P6, eight bits a channel), for pictures studied away from the
--  run that took them (a walk's stills, Driver.Robot.Kinematics.Base; a recording's frames, tools/replay).

package Driver.Images.Files is

   procedure Write_PPM (Name : String; I : Image);
   --  Writes the picture to the file Name, made anew.

   function Read_PPM (Name : String) return Image;
   --  The picture the file Name holds; Constraint_Error for a file that is no binary PPM of eight-bit channels.

end Driver.Images.Files;
