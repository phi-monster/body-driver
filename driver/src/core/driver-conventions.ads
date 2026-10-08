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

   Initial_Damping : constant := 1.0;
   --  A Levenberg-Marquardt fit that has a flat valley to cross starts with its damping as large as the curvature
   --  itself: every diagonal entry of the normal equations grows by that fraction of itself, so a first step is at
   --  most half the Gauss-Newton step along every axis the data curve, and the damping halves with every step that
   --  lowers the cost. From a damping of the float's epsilon (a Gauss-Newton step, damped by nothing) the arm fit's
   --  first stages crossed the valley between the focal lengths, the centre and the distortion in jumps, and the
   --  track refinement after them started from a lens that was off. Of eight refits of A17's arm with the keyframes
   --  of its hand's presses (80 to 134 keyframes), seven ended undetermined (a noise of 1.6 pixels, the normal
   --  equations singular) or fitted with a noise of 0.3 to 1.6 pixels, the centre up to 8 pixels off and, once, the
   --  focal lengths at 376 and 361 pixels; from this damping all eight gave a noise of 0.12 pixels and a focal
   --  length of 397.4, where the camera's is 397.04.

   Unchanged_Fraction : constant := 0.01;
   --  An iterative estimate has stopped changing when one more iteration
   --  moves it by less than this fraction of its own size. Below this the
   --  remaining change is smaller than any uncertainty the driver reports.

end Driver.Conventions;
