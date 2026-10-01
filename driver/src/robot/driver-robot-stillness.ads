--  The one stillness judgment.
--
--  A group is still at a beat when its readings did not change since the
--  beat before, significantly against their own noise and tested together
--  as one vector (Driver.Robot.Channels).
--
--  An eye is still when its latest frame shows no change against the
--  frames it has seen since its last change (its settled view): each
--  pixel's luma is tested against that view's mean, with the pixel's noise
--  as measured over the eye's noise view (Driver.Pixels: per-pixel variance
--  over still frames, never below the 8-bit quantization floor) and the
--  degrees of freedom it rests on, and the eye has changed when the changed
--  pixels significantly outnumber the false alarms that per-pixel test
--  raises on its own. The noise view is the longest still run the eye has
--  had, so a pixel's noise is never judged from the very frames it is being
--  compared with once a change has been seen; it starts as the first two
--  frames, which are not judged (a single frame cannot say how noisy a
--  camera is, and judging a frame against the floor alone would see change
--  in every noisy pixel and never come to rest). Nothing assumes the scene
--  is at rest at any time: a run that holds motion the test could not yet
--  see makes the moving pixels' noise large, which only makes the test
--  blind there, never alarm.
--
--  Before anything is measured nothing is still: a group whose noise is not
--  measured yet, or an eye not yet judged, makes the body not still.
--
--  The body is still when every group and every eye is.

private package Driver.Robot.Stillness is

   procedure Judge_Eye (S : in out Eye_Stream; Frame : Driver.Images.Image; Luma : Real_Array)
     with Pre => not Driver.Images.Is_Empty (Frame)
                 and then Luma'Length = Driver.Images.Width (Frame) * Driver.Images.Height (Frame);
   --  One beat of an eye that has a frame, and the frame's luma (Driver.Images.Luma):
   --  judges it and updates its views.

   procedure Measure_Luma_Noise (S : in out Eye_Stream)
     with Pre => S.Has_Settled;
   --  Per cell of the eye's grid, the median over its pixels of their luma
   --  variance over the eye's noise view (which Driver.Pixels never takes
   --  below the 8-bit quantization): how noisy a resting pixel of the cell
   --  is. Left as it was when the noise view is not of the grid's size.

   function Group_Still (M : Model; G : Group_Id; Beat : Natural) return Boolean;
   --  False while the group's noise is not measured or a reading is missing.

   function Eye_Still (M : Model; E : Eye_Id) return Boolean;
   --  At the latest beat; False until the eye has been judged.

   function All_Still (M : Model) return Boolean;
   --  Every group and every eye still at the latest beat.

end Driver.Robot.Stillness;
