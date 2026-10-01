package body Driver.Robot.Boot is

   --  Path A replaces this placeholder.

   procedure Run (M : in out Model; H : in out Driver.Robot.Hand.Hands; Body_File : String; Ok : out Boolean) is
      pragma Unreferenced (M, H, Body_File);
   begin
      Ok := False;
   end Run;

end Driver.Robot.Boot;
