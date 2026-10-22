--  Floor, and rounding to the nearest whole number with halves away from zero, as the processor does them. GNAT's
--  Real'Floor and Real'Rounding go through its run time's software (System.Fat_Gen); in the camera's MJPEG and the
--  renderer's textures they were a third of the fake robot's time (the lab's Mac, 10-22, seed 108: 2151 of 6200
--  samples). Both are exact, so the values are the attributes' to the bit.

with Driver;

package Fake_Rounding is

   function Floor (X : Driver.Real) return Driver.Real;
   pragma Import (Intrinsic, Floor, "__builtin_floor");

   function Round (X : Driver.Real) return Driver.Real;
   pragma Import (Intrinsic, Round, "__builtin_round");

end Fake_Rounding;
