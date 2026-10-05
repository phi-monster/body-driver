--  The body file: everything the body measured about itself, as JSON.
--
--  Every quantity is stored with the version of the method that measured
--  it, so a reload keeps what is still valid and re-measures only what a
--  changed method would measure differently. A quantity is kept only when
--  its inputs are kept too: the lags and the step responses rest on the
--  readings' noise, the lock-in on the noise and the lags, the graph on the
--  lock-in, the kinematics on the graph and the lock-in. The file also
--  carries a key, the shape of what the robot reports (group sizes, which
--  groups take commands, image sizes): a file whose key differs from the
--  body that connected belongs to another body and is not reloaded.

with Ada.Strings.Unbounded;

private package Driver.Robot.Body_File is

   --  Method versions; each goes up when the code that measures it changes.
   Noise_Method      : constant := 1;
   Travel_Method     : constant := 1;
   Steps_Method      : constant := 1;
   Lag_Method        : constant := 2;
   Lockin_Method     : constant := 2;
   Graph_Method      : constant := 1;
   Kinematics_Method : constant := 4;

   function Text (M : Model) return String;
   --  The whole file.

   procedure Write (M : Model; Path : String; Ok : out Boolean);
   --  Writes Text to Path; Ok is False when the file cannot be written.

   procedure Read
     (M    : in out Model;
      Text : String;
      Ok   : out Boolean;
      Why  : out Ada.Strings.Unbounded.Unbounded_String);
   --  Restores from Text, as Text wrote it, every quantity whose method is
   --  the code's and whose inputs are restored too, and marks it reloaded
   --  (Driver.Robot.Reloaded). A model without groups takes the key's groups
   --  and eyes; one with them must match the key. Ok is False when Text is
   --  not a body file or its key is not this body's; nothing is restored then.

end Driver.Robot.Body_File;
