--  Layer 4: from what the brain wants to commands, and back to one ending
--  word (LANGUAGE.md 4, 5 and 17).
--
--  A want is either an interval of simultaneous constraints that runs until
--  one of the named endings, or a change of one measured quantity of one
--  thing (the quantity keyboard). Execute is the only way the upper layer
--  moves the body; it plans from the measured body and scene, moves through
--  Driver.Robot.Motion, checks after every step that the scene moved as
--  wanted, and ends with one ending and a plain account of what moved.
--
--  Ownership: path C.

with Ada.Containers.Indefinite_Vectors;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.World;

package Driver.Action is

   use Ada.Strings.Unbounded;

   type Ending is (Arrived, Touched, Stuck, Slipped, Lost, Free, Settled, Stalled, Timeout, Refused);
   type Ending_Set is array (Ending) of Boolean;

   type Role is (Me, Grasper, Pusher);

   type Relation is
     (Touching, Above, Below, Left, Right, Nearer, Farther, Onto, Off, Into, Facing, Clear, Still,
      Press, Close, Open);

   type Size is (Unspecified, Small, Medium, Large);
   --  An upper bound on one step, priced in the channel's own probe size.

   type Effort is (Unspecified, Light, Firm, Hard);

   type Operand_Kind is (Nothing, Role_Operand, Thing_Operand, Place_Operand);

   type Operand (Kind : Operand_Kind := Nothing) is record
      case Kind is
         when Role_Operand  => The_Role : Role;
         when Thing_Operand => Thing    : Driver.World.Thing_Id;
         when Place_Operand => Place    : Driver.World.Place_Id;
         when Nothing       => null;
      end case;
   end record;

   type Constraint is record
      Subject  : Operand;
      Relation : Driver.Action.Relation;
      Object   : Operand;
      Step     : Size := Unspecified;
      Strength : Effort := Unspecified;
      Must     : Boolean := False;
   end record;

   package Constraint_Vectors is new Ada.Containers.Vectors (Positive, Constraint);

   type Eye_Choice is (Any_Eye, Still_Eye, Moving_Eye);

   type Want_Kind is (Interval, Change);

   type Want (Kind : Want_Kind := Interval) is record
      Until_Endings : Ending_Set := [others => False];
      Max_Steps     : Natural := 0;          --  0: no step limit was written
      Eye           : Eye_Choice := Any_Eye;
      Anyway        : Boolean := False;
      case Kind is
         when Interval =>
            Constraints : Constraint_Vectors.Vector;
         when Change =>
            Thing    : Driver.World.Thing_Id;
            Quantity : Positive;             --  index into Quantities
            Increase : Boolean;              --  up, as opposed to down
      end case;
   end record;

   type Context is limited record
      Robot : not null access Driver.Robot.Model;
      Hands : not null access Driver.Robot.Hand.Hands;
      Scene : not null access Driver.World.Scene;
   end record;

   package Name_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);

   function Quantities (C : Context) return Name_Vectors.Vector;
   --  The quantities of things this body can measure and change now, by
   --  their keyboard word (for example "height").

   function Can_Bind (C : Context; R : Role) return Boolean;
   --  The role is bound to a measured part of this body now.

   type Verdict (Ok : Boolean := True) is record
      case Ok is
         when True  => null;
         when False =>
            Why         : Unbounded_String;   --  plain words for the brain
            Alternative : Unbounded_String;   --  a line it can write instead, if any
      end case;
   end record;

   function Check (C : Context; W : Want) return Verdict;
   --  The gates before any motor moves (LANGUAGE.md 9): can it be said with
   --  this body, are its quantities proven, and does a dry run on the
   --  measured response succeed. Milliseconds; moves nothing.

   type Result is record
      Final   : Ending := Refused;
      Tried   : Unbounded_String;   --  for Refused: what was tried (never empty)
      Account : Unbounded_String;   --  what moved and by how much, for the next round
   end record;

   procedure Execute (C : in out Context; W : Want; R : out Result);
   --  Decider: runs the want to one ending.

end Driver.Action;
