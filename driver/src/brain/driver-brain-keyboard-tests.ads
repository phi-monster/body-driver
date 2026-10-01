--  Self test of the keyboard (LANGUAGE.md 17.1), with a matcher for the GBNF
--  subset the driver writes, so the tests can ask what the decoder lets
--  through.

package Driver.Brain.Keyboard.Tests is

   procedure Register;

   function Accepts (Grammar, Text : String) return Boolean;
   --  Whether Text is a sentence of Grammar's root rule. Other tests use it
   --  to check that what they expect the brain to write can be typed.

end Driver.Brain.Keyboard.Tests;
