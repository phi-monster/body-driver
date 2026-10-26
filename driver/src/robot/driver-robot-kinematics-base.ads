--  The body under an arm. An arm's base rides on a body that need not hold still while the arm moves: a humanoid on
--  its walking program leans as the arm's weight moves (the hall's G1, 10-26: fitted on the truth's own sightings its
--  arms' focal lengths came out 16 to 18 % long, and 0 to 1.3 % with the torso's motion taken out), a wheeled base
--  turns. An eye the body carries and no arm (its head camera) sees that motion; where it sits on the arm's eye is
--  told by a walk of the body with the arm held (Driver.Rig), and each keyframe's sightings are taken where the arm's
--  eye would have seen them had its base held still, before every stage of the arm's fit. Taken out of the fit's last
--  stage alone, the motion left the stages before it in a wrong basin (g11a from a start that had only the turn taken
--  out ended 1.5 % long at four times the residual, g12b went from right to 21 % long), and taking out the turn alone
--  left g11a 14 % long: the base's step at the eye matters as much as its turn.
--
--  The walk: before an arm's sweep takes its reference keyframe the boot pushes what carries the body each way
--  (Begin_Walk, Take_Still), the pictures still after each push. Once the reference keyframe is taken, the stills of
--  both eyes are aligned to it (Driver.Alignment.Keyframes: the view's common motion) and adjusted together: the body
--  eye's frame on the arm's eye, both lenses and every point's inverse depth, in the walk's own scale. At each
--  keyframe the body eye's picture is aligned to its reference; the body eye's motion since the reference keyframe
--  (Driver.Rig.Locate on its points) carried onto the arm's eye by that frame is the base's, the arm eye's pose as
--  seen at the keyframe (Locate on its points) less that motion is where a still base would have put it, and each
--  sighting is shifted by the difference of where the two poses see its point.

package Driver.Robot.Kinematics.Base is

   function Body_Eye_Of (M : Model; A : Arm_Id) return Eye_Id'Base;
   --  The eye arm A's walk is seen by: an eye the carrier moves whole (Response) and arm A does not carry, which stands
   --  still on the body while arm A is swept (the others are held): one the carrier carries and no arm
   --  (Carrier_Carried) before one another arm carries (Stretch's head swept, the wrist eye of its arm held still on
   --  its base), and of those the one with the most textured cells; 0 where none is, or nothing carries every eye
   --  (Carrier_Group).

   procedure Begin_Walk (M : in out Model; A : Arm_Id);
   --  The boot's word that the body's walk under arm A begins: its stills are taken from now on (Take_Still) until the
   --  arm's sweep begins (Kinematics.Begin_Anew), and earlier ones are dropped. Recorded as a note ("walk A").

   procedure Take_Still (M : in out Model; A : Arm_Id);
   --  The boot's word that the next beat's pictures are a still of the walk. Recorded as a note ("still A"); a replay
   --  applies both notes where the run made them (Driver.Robot.Replay_Note).

   procedure Observe (M : Model; R : in out Arm_Evidence; O : Observation);
   --  One beat, for an arm's evidence: a still taken where one is due.

   procedure Align (M : Model; R : in out Arm_Evidence);
   --  After the arm's keyframes are aligned (Align_Arm): the walk's stills aligned and calibrated once the reference
   --  keyframe is taken (again once the arm is fitted, where they told nothing before it), and the body eye's picture
   --  at every keyframe aligned so far.

   procedure Shift_Sightings (M : Model; R : in out Arm_Evidence);
   --  Every aligned keyframe's base motion and the shift of its sightings (Arm_Evidence.Base), where the walk was
   --  calibrated; none where it was not.

   function Shift_U (R : Arm_Evidence; Frame : Positive; Point : Natural) return Real;
   function Shift_V (R : Arm_Evidence; Frame : Positive; Point : Natural) return Real;
   --  What is added to keyframe Frame's answer of the query point Point (from 0) before the fit; nought where unknown.

end Driver.Robot.Kinematics.Base;
