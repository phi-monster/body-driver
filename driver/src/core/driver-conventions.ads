--  The conventions the whole driver shares.
--
--  Every other number in the driver is measured on the body or the scene, or
--  follows from mathematics. A gate rejects any other hand-picked constant, so
--  a new convention has to be added here, with its reason, or not at all.

package Driver.Conventions with Pure is

   Z : constant := 3.0;
   --  A difference is significant when it exceeds Z standard deviations of
   --  its own measured noise. For a Gaussian error a single test then raises
   --  a false alarm about 0.3 % of the time, while effects a few times larger
   --  than the noise are still detected.

   Unchanged_Fraction : constant := 0.01;
   --  An iterative estimate has stopped changing when one more iteration
   --  moves it by less than this fraction of its own size. Below this the
   --  remaining change is smaller than any uncertainty the driver reports.

end Driver.Conventions;
