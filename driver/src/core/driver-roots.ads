--  The square root the processor takes, inline. IEEE 754 defines the square root correctly rounded, so this is the
--  same to the bit as Ada.Numerics.Long_Elementary_Functions.Sqrt, which reaches the C library's after a call and two
--  tests: that call was 2 to 3 % of a hall boot's driver in the build the rounds run (gprof, 10-28). A negative
--  argument gives a NaN here, where the library raises Argument_Error.
package Driver.Roots with Pure is

   function Sqrt (X : Long_Float) return Long_Float
     with Import, Convention => Intrinsic, External_Name => "__builtin_sqrt";

end Driver.Roots;
