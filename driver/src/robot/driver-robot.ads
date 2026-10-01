--  Layer 2: the robot's body, measured from its own readings, commands and
--  images.
--
--  Nothing here is declared by a user. Boot (Driver.Robot.Boot) measures the
--  body from zero, or reloads what an earlier boot measured and re-measures
--  what changed; Observe then refines the estimates every beat. Every
--  quantity carries its uncertainty.
--
--  Frames and units. The world frame is fixed to the scene; how boot anchors
--  it is documented in Driver.Robot.Boot. Lengths are in the body's own unit,
--  fixed at boot; no layer may assume metres or degrees, and nothing may
--  assume that +z is up (use Up). Camera frames have z along the optical
--  axis, x along +U and y along +V of the image (Driver.Images).
--
--  Estimation is separate from decision: Observe is fed every beat with the
--  observation and the command in effect, whoever chose it, so the same
--  estimates follow from a recording of any driver. The heavier estimates
--  (roles, kinematics) are recomputed by Observe whenever the evidence
--  behind them has doubled, and a decider may ask for them at once with
--  Estimate_Now.
--
--  Ownership: path A owns this layer except Driver.Robot.Hand (path B).
--  Upper layers use only what this specification and Driver.Robot.Motion and
--  Driver.Robot.Hand export.

with Driver.Commands;
with Driver.Images;
with Driver.Numerics;
with Driver.Observations;
with Driver.Uncertain;

package Driver.Robot is

   use Driver.Numerics;
   use Driver.Uncertain;

   subtype Group_Id is Driver.Observations.Group_Id;
   subtype Eye_Id is Driver.Observations.Camera_Id;
   subtype Observation is Driver.Observations.Observation;

   type Group_Role is (Unclassified, Arm, Carrier, Closer, Part, Sensor, Inert);
   --  Measured by pushing each commandable group (docs/body-protocol.md, 3):
   --    Arm      moving it moves the whole image of the eyes it carries, not of all
   --    Carrier  moving it moves the whole image of every eye (base, torso, drone)
   --    Closer   moving it moves a patch inside an arm's own eye (fingers, gripper)
   --    Part     moving it moves a patch outside every arm's eye
   --    Sensor   its readings change though it is not commanded
   --    Inert    nothing it is pushed to changes anything any eye sees, or,
   --             not commandable, its readings never change
   --  With a single eye nothing tells carrying the whole body apart from
   --  carrying that eye, so a group moving it is an Arm.

   type Arm_Id is new Positive;

   type Mount_Kind is (Unmeasured, World_Fixed, Arm_Carried, Carrier_Carried);

   type Mount (Kind : Mount_Kind := Unmeasured) is record
      case Kind is
         when Arm_Carried => Arm : Arm_Id;
         when others      => null;
      end case;
   end record;

   type Model is tagged limited private;

   procedure Observe (M : in out Model; O : Observation; Sent : Driver.Commands.Command);
   --  One beat: the observation, and Sent, the last command sent to the
   --  robot before it arrived (the one in effect while it was captured;
   --  holds included). Estimators only; never sends anything.

   procedure Estimate_Now (M : in out Model);
   --  Recomputes the heavier estimates from everything observed so far.
   --  Deciders call it between Driver.Beats.Next and Send; it can take
   --  seconds, during which the robot holds.

   function Booted (M : Model) return Boolean;

   function Role (M : Model; G : Group_Id) return Group_Role;

   function Arm_Count (M : Model) return Natural;
   function Arm_Group (M : Model; A : Arm_Id) return Group_Id
     with Pre => Natural (A) <= Arm_Count (M);

   function Eye_Count (M : Model) return Natural;
   function Eye_Mount (M : Model; E : Eye_Id) return Mount;

   function Eye_Pose (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate;
   --  The camera frame in the world frame at the beat of O.

   procedure Project
     (M       : Model;
      E       : Eye_Id;
      O       : Observation;
      Point   : Vec3;
      Px      : out Driver.Images.Pixel;
      Visible : out Boolean);
   --  Where a world point appears in the eye; Visible is False when it falls
   --  behind the eye or outside the image.

   function Ray (M : Model; E : Eye_Id; O : Observation; Px : Driver.Images.Pixel) return Ray_Estimate;
   --  The line of sight through a pixel, in the world frame.

   function Up (M : Model) return Direction_Estimate;
   --  Away from gravity, in the world frame, as measured.

   function Tool_Pose (M : Model; A : Arm_Id; O : Observation) return Pose_Estimate;
   --  The last link of the arm in the world frame at the beat of O.

   function Eye_In_Tool (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate
     with Pre => Eye_Mount (M, E).Kind = Arm_Carried;
   --  The eye's frame in its arm's tool frame at the beat of O, carrying only
   --  the uncertainty of what lies between them (the mount, and any joints
   --  between the last link and the eye), so a measurement made in the eye is
   --  taken into the tool frame without counting the arm's kinematics twice.
   --  Constant when the eye rides on the last link.

   function Blocked (M : Model; A : Arm_Id; O : Observation) return Boolean;
   --  At the beat of O the arm was commanded further than it went, by more
   --  than its free motion falls short: the estimators' view of the judgment
   --  Driver.Robot.Motion.Step reports, so a replay sees the blocked beats of
   --  the run it replays.

   function Self_Mask (M : Model; E : Eye_Id; O : Observation) return Driver.Images.Mask;
   --  The pixels of the eye that show the robot itself at the beat of O.

   function Clearance (M : Model; Point : Vec3; O : Observation) return Estimate;
   --  The distance from a world point to the nearest surface of the body.

   function Still (M : Model) return Boolean;
   --  The one stillness judgment: at the latest beat no group and no eye
   --  changes significantly against its own measured noise.

   --  What the body reported and how each group behaves.

   function Group_Count (M : Model) return Natural;
   function Group_Size (M : Model; G : Group_Id) return Natural;
   function Is_Commandable (M : Model; G : Group_Id) return Boolean;
   --  A command in effect has carried a target for it.

   function Reading_Noise (M : Model; G : Group_Id; Channel : Positive) return Real;
   --  The standard deviation of the channel's reading at rest, in reading
   --  units; zero for a reading that repeats exactly.

   function Visible_Step (M : Model; G : Group_Id; Channel : Positive) return Estimate;
   --  The smallest change of the channel's command whose effect the eyes that
   --  see it tell from their own noise, in reading units: where a probe or a
   --  sweep of the channel starts.

   type Eye_Response is (Unmeasured, Nothing, Patch, Undecided, Whole);
   --  What pushing a group does to what an eye sees: nothing, a patch of the
   --  image moves, or the whole image moves (the eye rides on the group).

   function Response (M : Model; G : Group_Id; E : Eye_Id) return Eye_Response;

   function Image_Lag (M : Model; E : Eye_Id) return Integer;
   --  How many beats the eye's images trail the readings they belong to:
   --  the image of beat B shows the body as read at beat B - Image_Lag.

   function Closer_Arm (M : Model; G : Group_Id) return Arm_Id'Base;
   --  For a Closer, the arm in whose eye it moves; 0 otherwise.

   function Carrier_Group (M : Model) return Group_Id'Base;
   --  The group that carries every eye, or 0 when there is none.

   function Contract_Breach (M : Model; G : Group_Id) return Natural;
   --  The clause of the porting contract (docs/body-protocol.md, 3) the
   --  group breaks: 1 its reading does not follow its command, 2 nothing
   --  any eye sees changes when it moves, 3 it reports moving where an eye
   --  sees nothing move although the opposite push showed; 0 none.

   function Describe (M : Model) return String;
   --  The measured body in a few lines, for the log.

private

   type Model is tagged limited record
      Is_Booted : Boolean := False;
   end record;

end Driver.Robot;
