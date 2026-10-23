--  What each group's motion does to what each eye sees.
--
--  For every cell of an eye, the displacement from one beat to the next is
--  regressed on the reading change of every channel of every commandable
--  group, read the measured image lag earlier: for small motions the
--  displacement is linear in the reading changes (the image Jacobian), so
--  one regression separates groups that moved at different times, gives
--  no weight to a group's tiny incidental motion (a reaction to another
--  group), and lets a swaying body or a running mouse only add noise. A
--  group moves a cell when its block of the Jacobian is significant.
--
--  A group whose push moves the whole image of an eye carries that eye;
--  one that moves a patch is seen by it. Whole means a significant majority
--  of the cells that can show a displacement show it (an eye mostly sees
--  the world, not the body parts that ride with it); a patch is a
--  significant minority; anything in between is undecided. A cell shows it
--  when it responds, or, when its own noise is too much for it to tell a motion
--  as large as the responding cells show, in the share that the cells like it
--  show it together (Cells_Shown). So a view partly too faint for its cells
--  to tell a push is still moved whole by it; a patch of the best-measured
--  cells is not made the whole picture by the faint cells around it, which
--  together show nothing, nor by the few of them that show a great deal (the
--  fingers of a hand in view); and a push too small for the faint cells to
--  show it even together leaves the verdict undecided, since the share is
--  known only within its error. That a group moves anything at all is itself
--  tested: the responding cells must outnumber the false alarms the per-cell
--  test makes on its own.

private package Driver.Robot.Lockin is

   type Noisy_Cell is record
      Energy   : Real;       --  the motion the group shows in the cell: its Wald statistic less its degrees of freedom,
                             --  times its variance (zero in mean when the group moves nothing there)
      Variance : Real;       --  of the cell's displacement noise
      Freedom  : Positive;   --  degrees of freedom of the cell's test of the group
      Responds : Boolean;    --  the test is significant
   end record;

   type Noisy_Cell_Array is array (Positive range <>) of Noisy_Cell;

   type Shown is record
      Least, Most : Natural;
   end record;

   function Cells_Disagree (Showing, Silent : Natural) return Boolean;
   --  Whether the cells that can tell a motion of the typical energy, Showing of them showing it and Silent of them not,
   --  disagree that a picture moves whole: they are enough to say it (even all of them showing it would be a significant
   --  majority) and a significant majority of them do not show it. Too few of them to say anything do not disagree.

   function Cells_Shown (Pool : Noisy_Cell_Array; Typical : Real; Able_Disagree : Boolean; Quiet : Boolean := False) return Shown
     with Pre  => Pool'Length > 0 and then Typical > 0.0 and then (for all C of Pool => C.Variance > 0.0),
          Post => Cells_Shown'Result.Least <= Cells_Shown'Result.Most and then Cells_Shown'Result.Most <= Pool'Length;
   --  Of cells whose noise is too much for them to tell a motion of the energy Typical, how many show a motion of that
   --  energy, at least and at most (Z errors either side of the estimate). A cell's energy estimates the motion it
   --  shows without bias whatever its noise, with the variance of a non-central chi-square of the typical energy
   --  (twice its degrees of freedom times its variance squared, and four times its variance times the energy), so
   --  their mean over the typical energy is the share of them that show it if each that does shows the typical energy.
   --  Cells far above it, that respond with an energy beyond the typical by Z of their own spread, show it, and are
   --  counted so, when the cells that can tell do not agree that the picture moves whole (Able_Disagree: they are enough
   --  to say it, and a significant majority of them do not show it), and they carry more than half the pool's energy:
   --  the mean is then theirs and says nothing of the others (the fingers of a hand in view, a few cells among hundreds,
   --  show 10 ** 4 to 10 ** 5 times what the rest of the picture does: A29's closer, at 8136 beats, had 37 of 371,
   --  99.8 % of the energy, a mean 560 times the typical energy, 75 of its 154 cells that can tell showing it, and was
   --  called an arm), and the others are read without them. A whole picture's nearer parts, a few cells that show
   --  more, stay in the mean when the cells that can tell agree (A31's first arm, at 252 beats: 14 cells carried
   --  three quarters of the energy and 103 of its 108 cells that can tell showed the motion).

   function Still_A_Patch (Responding, Textured : Natural) return Boolean;
   --  Whether an eye with that many of its textured cells responding to a group is a patch by the verdict's
   --  own rule: the silent cells are a significant majority. The boot asks it of a patch's responding count
   --  doubled: a patch one more doubling of the push cannot make anything else is not pushed on (R5 on the
   --  Pi, 10-13: a gripper's own patch in its wrist eye grew 17, 22, 46 of 525 with every doubling, as a jaw
   --  opening wider does, and the rounds went to 64 times the amounts for it).

   function Judge (Responding, Textured, Least, Most : Natural; Able_Disagree : Boolean) return Eye_Response;
   --  What an eye makes of a group from how many of its Textured cells Responding to it and how many show its motion,
   --  Least at least and Most at most: nothing unless the responding cells outnumber the false alarms the per-cell test
   --  makes on its own; whole when a significant majority show it and the cells that can tell do not disagree; a patch
   --  when a significant majority do not; else undecided.

   procedure Measure (M : in out Model);
   --  Re-measures every group's effect on every eye, and each cell's
   --  displacement noise at rest, from the whole stream. Uses the image
   --  lags (Driver.Robot.Lag) and the channel noise (Driver.Robot.Channels).

   procedure Measure_Rest_Noise (M : in out Model);
   --  For every eye, how much noisier its cells' displacements are at rest
   --  than their quantization floor says: over the beats where the eye was
   --  judged still in both frames (Driver.Robot.Stillness, which never looks
   --  at displacements), each textured cell's displacement over its floor;
   --  its squared length is a chi square of two degrees in those units, so
   --  the factor is the square root of its median over the median of that
   --  chi square. Never below one.

   procedure Count_Moved (S : Eye_Stream; Beat : Natural; Count, Tested : out Natural);
   function Moved (M : Model; E : Eye_Id; Beat : Natural) return Boolean;
   --  The eye's image moved at that beat as the lock-in can tell: more of its
   --  textured cells resolved a displacement beyond their noise at rest (their
   --  floor times the eye's rest factor, Measure_Rest_Noise), or
   --  moved too far to be resolved, than the rest's beats do: significantly
   --  above the rest counts' median by their robust scatter (Rest_Count_Level),
   --  or, before the rest is measured, than that per-cell test alarms on by
   --  chance. The step that makes an eye move this way is one the lock-in
   --  can measure; a change the stillness judgment sees (a pixel's rounding
   --  flipping) can be far smaller.

   function Rest_Share (M : Model; E : Eye_Id; Beat : Natural) return Real;
   --  The chance that a beat of the eye at rest counts as many moved cells as this beat did, or more (Count_Moved):
   --  of the n counts it had at rest (Rest_Counts), k reached this one, and a count at rest reaches it with chance at
   --  most (k + 1) / (n + 1), whatever their distribution. Before those are measured, the per-cell test's own level;
   --  one where the eye tested no cell at the beat.

   function Cell_Noise (M : Model; E : Eye_Id) return Real;
   --  How finely one cell of the eye tells a displacement: the median over its
   --  textured cells of the noise of their displacements about the
   --  regression on the pushes, in pixels; Real'Last before it is measured.

   procedure Total_Gains (S : in out Eye_Stream);
   --  Each regressor's gains summed over the eye's cells, and their variances, in the cells' order: what Gain_Of gives,
   --  made once whenever the gains are. Summed at every asking, a channel's visible step summed every cell of every
   --  eye, and the one test of motion asks for every channel's at every beat of every group (a humanoid's boot on the
   --  Mac, 10-22: 6 in a hundred of the driver's handling of a beat).

   procedure Gain_Of
     (M : Model; E : Eye_Id; G : Group_Id; Channel : Positive; Gain, Spread : out Real; Measured : out Boolean);
   --  The channel's gain in the eye summed over its cells (Measure: each
   --  cell's displacement per reading unit in units of the cell's noise,
   --  squared, less the variance of estimating it), and the variance of that
   --  sum. Measured is False, and both zero, when the eye keeps no column for
   --  the channel.

   function Seen (Gain, Spread : Real) return Boolean;
   function Seen (M : Model; E : Eye_Id; G : Group_Id; Channel : Positive) return Boolean;
   --  The eye sees the channel move it at all: the summed gain stands Z of
   --  its spread above zero. A channel that moves nothing the eye sees still
   --  gets a coefficient in every cell that responds to its group, noise
   --  about zero with a magnitude, and from that a shift (the median
   --  magnitude) and a visible step (Z over the root of the summed gain,
   --  positive by chance) that mean nothing: fake body seed 5, arm 1's third
   --  channel, which the probe drove to its end unseen, had a keyframe step
   --  of 1.2 units from a shift of 0.05 px, while the step the eye could see
   --  was 137; its return at 16 units a beat gave a keyframe every beat, 262
   --  of them 96 px from the reference, which aligned nothing. False before
   --  the eye is measured.

   function Shift (M : Model; E : Eye_Id; G : Group_Id; Channel : Positive) return Real;
   --  How many pixels the eye's image moves per reading unit of the channel:
   --  the median over the cells that respond to its group; zero when none
   --  does or it was not measured.
   --  Zero too for a channel the eye does not see (Seen): its coefficients
   --  are noise.

   function Shows_Step (M : Model; E : Eye_Id; G : Group_Id; Step : Real_Array) return Boolean;
   --  A step of the group's readings moves the eye's image by what its cells
   --  tell from their noise: some channel's step times the pixels the image
   --  moves per unit of it (Shift) is significant against one cell's noise
   --  (Cell_Noise). What the step does to the picture, not only that the
   --  picture changed: A72 and A73's renderer drifted every eye's picture
   --  after any move, a probe saw arm 2 move at a few millionths of a radian,
   --  and steps that small moved its eye against nothing. False until both
   --  are measured.

   procedure Dump_Cells (M : Model; Tag : String);
   --  For a recognition studied after its run: with BD_DUMP_CELLS naming a folder, every eye's grid, its textured
   --  cells, and for each of its regressors (a group's channel) the cells that respond to it and which way each moved
   --  per reading unit, in BD_DUMP_CELLS/cells_TAG.txt. Nothing when BD_DUMP_CELLS is not set.

end Driver.Robot.Lockin;
