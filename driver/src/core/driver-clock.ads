--  Time: beats counted from the robot's observations, and a monotonic clock.

package Driver.Clock is

   type Beat is new Natural;
   --  One beat is one observation from the robot; beat 0 is the first one
   --  after the driver started. Episodes do not restart the count.

   function Seconds return Duration;
   --  Monotonic time since the driver started.

   function Nanoseconds_Of (S : Duration) return Long_Long_Integer;
   --  S in whole nanoseconds, exact for a hundred days. (S times 10 ** 9
   --  overflows Duration beyond nine seconds.)

   function Nanoseconds return Long_Long_Integer is (Nanoseconds_Of (Seconds));
   --  The same clock in whole nanoseconds.

end Driver.Clock;
