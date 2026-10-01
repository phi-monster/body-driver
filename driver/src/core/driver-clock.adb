with Ada.Real_Time;

package body Driver.Clock is

   use Ada.Real_Time;

   Start : constant Time := Ada.Real_Time.Clock;

   function Seconds return Duration is (To_Duration (Ada.Real_Time.Clock - Start));

   function Nanoseconds_Of (S : Duration) return Long_Long_Integer is (Long_Long_Integer (Real (S) * 1.0e9));

end Driver.Clock;
