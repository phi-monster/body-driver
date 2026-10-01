with Ada.Real_Time;

package body Driver.Clock is

   use Ada.Real_Time;

   Start : constant Time := Ada.Real_Time.Clock;

   function Seconds return Duration is (To_Duration (Ada.Real_Time.Clock - Start));

end Driver.Clock;
