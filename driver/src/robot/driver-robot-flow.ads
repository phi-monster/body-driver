--  Where an image moves between two frames.
--
--  The image is divided into a grid of cells; for every cell the content's
--  translation from one frame to the next is estimated by Lucas-Kanade
--  (Gauss-Newton steps on the translation that best explains the second
--  frame by the first, with the first frame's gradient, until a step no
--  longer changes it), together with the smaller eigenvalue of the cell's
--  gradient tensor, which says how well that translation is determined.
--  Iterating makes the displacement exact for small motions instead of
--  biased by the linearization, so it is linear in small motions of the
--  body, which is what lets a regression on the readings tell which group
--  moves which part of the image (Driver.Robot.Lockin); motions far beyond
--  a cell give no consistent displacement and are rejected there. A change
--  of light without motion (a moving shadow's interior, a renderer
--  converging) explains little as a translation, unlike a plain brightness
--  difference.

private package Driver.Robot.Flow is

   function Grid_Of (Width, Height : Natural) return Cell_Grid;
   --  As many cells across as the square root of the width, as many down as
   --  the square root of the height: a cell is about as many pixels wide as
   --  there are cells across, the geometric middle between a pixel and the
   --  whole image.

   procedure Bounds (G : Cell_Grid; Cell : Positive; X0, X1, Y0, Y1 : out Natural)
     with Pre => Cell <= Cells (G);
   --  The cell covers pixel columns X0 .. X1 - 1 and rows Y0 .. Y1 - 1;
   --  cells are numbered row by row from the top-left one.

   procedure Displacements
     (G             : Cell_Grid;
      Before, After : Real_Array;
      Luma_Variance : Real_Array;
      Du, Dv        : out Real_Array;
      Condition     : out Real_Array)
     with Pre => Before'Length = G.Width * G.Height and then After'Length = Before'Length
                 and then Luma_Variance'Length = Cells (G)
                 and then Du'Length = Cells (G) and then Dv'Length = Cells (G)
                 and then Condition'Length = Cells (G);
   --  Per cell, the translation (pixels, +U right and +V down) that moves the
   --  content of Before to After, and the smaller eigenvalue of the cell's
   --  gradient tensor. A cell without texture in two directions gets zero
   --  displacement and zero condition. Luma_Variance is, per cell, the
   --  variance of a resting pixel's luma (Noise_Floor); the iteration stops
   --  once a step is below what that noise lets the cell resolve.

   function Noise_Floor (Condition, Luma_Variance : Real) return Real;
   --  The standard deviation of a displacement of a cell with that condition
   --  measured between two frames whose pixels carry that luma variance
   --  each: the variance propagated through the cell's gradient tensor along
   --  its weakest direction. Unknown (Real'Last) for a cell without texture.

end Driver.Robot.Flow;
