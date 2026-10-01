--  The body driver lets a robot measure its own body from joint readings,
--  commands and images, and lets a brain that only speaks the body language
--  (LANGUAGE.md) drive it.
--
--  Layers, each depending only on the layers below it:
--
--    1. Driver.*          core: protocol, numerics, statistics, recording
--    2. Driver.Robot      the body, measured at boot and kept up to date
--    3. Driver.World      things, surfaces and the remembered scene
--    4. Driver.Action     from a wanted change to commands and an ending word
--    5. Driver.Brain      rounds with the brain, names, program execution
--
--  Every quantity the driver uses about a body or a scene is measured and
--  carries its uncertainty; Driver.Conventions holds the only chosen numbers.

package Driver with Pure is

   subtype Real is Long_Float;

   type Real_Array is array (Positive range <>) of Real;

   type Natural_Array is array (Positive range <>) of Natural;

end Driver;
