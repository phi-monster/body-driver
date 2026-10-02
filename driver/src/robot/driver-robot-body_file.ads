--  The body file: everything the body measured about itself, as JSON.
--
--  Every quantity is stored with the version of the method that measured
--  it, so a reload keeps what is still valid and re-measures only what a
--  changed method would measure differently. The file also carries a key,
--  the shape of what the robot reports (group sizes, which groups take
--  commands, image sizes): a file whose key differs from the body that
--  connected belongs to another body and is not reloaded.

private package Driver.Robot.Body_File is

   --  Method versions; each goes up when the code that measures it changes.
   Noise_Method  : constant := 1;
   Lag_Method    : constant := 2;
   Lockin_Method : constant := 2;
   Graph_Method  : constant := 1;

   function Text (M : Model) return String;
   --  The whole file.

   procedure Write (M : Model; Path : String; Ok : out Boolean);
   --  Writes Text to Path; Ok is False when the file cannot be written.

end Driver.Robot.Body_File;
