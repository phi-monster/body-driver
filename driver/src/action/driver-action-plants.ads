--  The body and the world as the action layer acts on them: what it can
--  read, ask and do, and nothing else.
--
--  Look reads the latest estimates; Reach and Predicted answer questions
--  without moving anything; Move is the only way anything moves, one step for
--  any number of arms and closers together; Learn hands back what an outcome
--  showed about a thing. Execute drives the live plant over the lower layers
--  (Driver.Action.Plants.Live); the self test drives a simulated one. Every
--  operation is a decider operation: it may exchange beats with the robot.

with Ada.Containers.Vectors;
with Driver.Action.Snapshots;
with Driver.Numerics;
with Driver.Uncertain;

package Driver.Action.Plants is

   use Driver.Numerics;
   use Driver.Action.Snapshots;

   type Reach_Status is (Reachable, Unreachable, Unmeasured);

   type Reach_Answer is record
      Status : Reach_Status := Unmeasured;
      Why    : Unbounded_String;   --  for Unreachable: which limit, joint or distance
   end record;

   type Arm_Goal is record
      Arm           : Arm_Id;
      Tool          : Rigid;
      Position_Only : Boolean := False;   --  the tool's orientation is free
   end record;

   package Arm_Goal_Vectors is new Ada.Containers.Vectors (Positive, Arm_Goal);

   type Closer_Goal is record
      Hand     : Hand_Id;
      Fraction : Real;   --  0 open, 1 closed on nothing
   end record;

   package Closer_Goal_Vectors is new Ada.Containers.Vectors (Positive, Closer_Goal);

   type Order is record
      Arms    : Arm_Goal_Vectors.Vector;
      Closers : Closer_Goal_Vectors.Vector;
      Settle  : Boolean := True;   --  wait until the body settles; False: one beat, for timing
   end record;
   --  Everything in one order moves together, in the same beats.

   type Step_Outcome is (Reached, Blocked, Short, Refused);
   --  Reached  arrived within its noise
   --  Blocked  stopped short and pushing further does not move it
   --  Short    moved but delivered only part of the step
   --  Refused  could not be commanded (unreachable, unmeasured)

   type Arm_Result is record
      Arm       : Arm_Id;
      Outcome   : Step_Outcome := Refused;
      Delivered : Driver.Uncertain.Estimate;   --  share of the commanded displacement
      Why       : Unbounded_String;
   end record;

   package Arm_Result_Vectors is new Ada.Containers.Vectors (Positive, Arm_Result);

   type Closer_Result is record
      Hand    : Hand_Id;
      Outcome : Step_Outcome := Refused;
   end record;

   package Closer_Result_Vectors is new Ada.Containers.Vectors (Positive, Closer_Result);

   type Report is record
      Arms    : Arm_Result_Vectors.Vector;
      Closers : Closer_Result_Vectors.Vector;
      Beats   : Natural := 0;
   end record;

   type Lesson_Kind is (Friction_Learned, Touched_At);

   type Lesson (Kind : Lesson_Kind := Friction_Learned) is record
      Thing : Thing_Id;
      case Kind is
         when Friction_Learned => Bounds : Friction_Bounds;
         when Touched_At       => Point  : Driver.Uncertain.Point_Estimate;
      end case;
   end record;

   type Plant is limited interface;

   procedure Look (P : in out Plant; S : out Snapshot) is abstract;
   --  The estimates at the latest beat.

   function Reach (P : Plant; Goal : Arm_Goal) return Reach_Answer is abstract;
   --  Whether the arm can be brought there from where it is now, solved
   --  along the way; the same solver Move uses. Moves nothing.

   procedure Move (P : in out Plant; O : Order; R : out Report) is abstract;
   --  One step of every arm and closer in O, together.

   function Predicted (P : Plant; T : Thing_Id; Beats : Natural) return Driver.Uncertain.Point_Estimate is abstract;
   --  Where the thing's centre will be that many beats from now, from its
   --  measured motion; a thing not moving stays where it is.

   procedure Learn (P : in out Plant; L : Lesson) is abstract;
   --  Records what an outcome showed about a thing, for every later look.

   function Episode_Over (P : Plant) return Boolean is abstract;
   --  No more beats will come in this episode.

   function In_View (P : Plant; Point : Vec3) return Boolean is abstract;
   --  Some eye that rides on no arm would see the point, as the body is now.

end Driver.Action.Plants;
