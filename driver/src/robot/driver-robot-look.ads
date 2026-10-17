--  An eye the body carries looks at the scene from places apart. The world places what an eye shows by itself from
--  pictures of the scene seen from elsewhere (Driver.World.Sight); at the start of an episode the eye may have seen
--  the scene as it now is from where it stands alone (a new episode may have set the scene anew, and an arm started
--  from a body file has swept nothing), and then nothing it shows can be placed.
--
--  The arm turns the one joint that carries its eye furthest across its line of sight for what that turn moves its
--  picture (the parallax at the table it gains a pixel of the picture's motion, by the fit and by what the keyframes
--  measured of the view's motion per unit), one way and then the other: by what moves the picture a quarter of its
--  width, so that the middle half of what the eye shows now stays in each view (the sweep's together-cells move the
--  picture by half of it); halved while the body does not reach it, down to what moves the picture by Z of the
--  aligner's error. A turn of one joint asks no solver for a pose the arm may not reach: s59 (servo seed 6, 10-18)
--  asked arcs about the point it looked at and was given poses on another branch, joints 140 degrees away, which the
--  body did not move to. At each place the eye is held still, where the world keeps its picture as a view
--  (Driver.World.Observe at still beats), and the joint is brought back. Two places: with the eye's own line of sight,
--  the three a point needs (Driver.Robot.Surfaces.Place).

package Driver.Robot.Look is

   procedure Around (M : in out Model; A : Arm_Id; Taken : out Natural);
   --  The fitted arm A's eye looks from places apart and comes back, as above; Taken is how many places it was held
   --  at. Called by a decider, outside a beat's window; nothing when the arm is not fitted, carries no eye, or its
   --  eye's middle line of sight does not meet the arm's table in front.

end Driver.Robot.Look;
