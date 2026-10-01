--  Time: beats counted from the robot's observations, and a monotonic clock.

package Driver.Clock is

   type Beat is new Natural;
   --  One beat is one observation from the robot; beat 0 is the first one
   --  after the driver started. Episodes do not restart the count.

   function Seconds return Duration;
   --  Monotonic time since the driver started.

end Driver.Clock;
