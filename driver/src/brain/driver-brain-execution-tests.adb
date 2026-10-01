with Ada.Containers.Vectors;
with Ada.Strings.Fixed;
with Driver.Brain.Parser;
with Driver.Tests;

package body Driver.Brain.Execution.Tests is

   use Driver.Brain.Programs;
   use Driver.Tests;
   use type Driver.Action.Ending;
   use type Driver.Action.Want_Kind;
   use type Driver.Action.Operand_Kind;
   use type Driver.World.Place_Id;
   use type Driver.Brain.Wants.Build_Result;

   LF : constant String := [ASCII.LF];

   package Ending_Vectors is new Ada.Containers.Vectors (Positive, Driver.Action.Ending);
   package Want_Vectors is new Ada.Containers.Vectors (Positive, Driver.Action.Want, Driver.Action."=");

   --  A body that ends its stretches as scripted and records what it was given.
   type Fake is new Performer with record
      Script      : Ending_Vectors.Vector;   --  the endings of the stretches, in order
      Done_Wants  : Want_Vectors.Vector;
      Spoken      : Unbounded_String;
      Refuse_All  : Boolean := False;
      Stop_After  : Natural := Natural'Last;  --  statements before Interrupted turns True
      Asked       : Natural := 0;
      Places      : Natural := 0;
   end record;

   overriding function Quantities (P : Fake) return Driver.Brain.Keyboard.Word_Vectors.Vector;
   overriding function Check (P : Fake; W : Driver.Action.Want) return Driver.Action.Verdict;
   overriding procedure Execute (P : in out Fake; W : Driver.Action.Want; R : out Driver.Action.Result);
   overriding procedure Place_Of
     (P     : in out Fake;
      Who   : Driver.Action.Operand;
      Place : out Driver.World.Place_Id;
      Ok    : out Boolean;
      Why   : out Unbounded_String);
   overriding procedure Say (P : in out Fake; Sentence : String);
   overriding function Interrupted (P : Fake) return Boolean;

   overriding function Quantities (P : Fake) return Driver.Brain.Keyboard.Word_Vectors.Vector is
      pragma Unreferenced (P);
      R : Driver.Brain.Keyboard.Word_Vectors.Vector;
   begin
      R.Append ("height");
      return R;
   end Quantities;

   overriding function Check (P : Fake; W : Driver.Action.Want) return Driver.Action.Verdict is
     (if P.Refuse_All then (Ok => False, Why => To_Unbounded_String ("out of reach"),
                             Alternative => To_Unbounded_String ("come closer first"))
      else (Ok => True));

   overriding procedure Execute (P : in out Fake; W : Driver.Action.Want; R : out Driver.Action.Result) is
   begin
      P.Done_Wants.Append (W);
      R := (Final => (if P.Script.Is_Empty then Driver.Action.Settled else P.Script.First_Element),
            Tried => Null_Unbounded_String, Account => To_Unbounded_String ("moved"));
      if not P.Script.Is_Empty then
         P.Script.Delete_First;
      end if;
   end Execute;

   overriding procedure Place_Of
     (P     : in out Fake;
      Who   : Driver.Action.Operand;
      Place : out Driver.World.Place_Id;
      Ok    : out Boolean;
      Why   : out Unbounded_String)
   is
      pragma Unreferenced (Who);
   begin
      P.Places := P.Places + 1;
      Place := Driver.World.Place_Id (P.Places);
      Ok := True;
      Why := Null_Unbounded_String;
   end Place_Of;

   overriding procedure Say (P : in out Fake; Sentence : String) is
   begin
      Append (P.Spoken, Sentence & ";");
   end Say;

   overriding function Interrupted (P : Fake) return Boolean is (P.Asked >= P.Stop_After);

   procedure Go
     (Text   : String;
      Doer   : in out Fake;
      Report : out Run_Report;
      Last   : in out Driver.Brain.Termination.Last_Endings;
      Bound  : Driver.Brain.Wants.Binding_Maps.Map := Driver.Brain.Wants.Binding_Maps.Empty_Map)
   is
      P     : Program;
      Ok    : Boolean;
      Why   : Refusal;
      B     : Driver.Brain.Wants.Binding_Maps.Map := Bound;
      Names : Driver.Brain.Names.Table;
   begin
      Driver.Brain.Parser.Parse (Text, P, Ok, Why);
      Check (Ok, "the test program does not parse: " & To_String (Why.Why));
      Run (P, B, Names, Doer, Last, Report);
   end Go;

   function Ball return Driver.Brain.Wants.Binding_Maps.Map is
      M : Driver.Brain.Wants.Binding_Maps.Map;
   begin
      M.Include ("ball", (Kind => Driver.Brain.Names.To_Thing, Thing => 3, others => <>));
      M.Include ("cup", (Kind => Driver.Brain.Names.To_Thing, Thing => 4, others => <>));
      return M;
   end Ball;

   function Spoken (D : Fake; S : String) return Boolean is (Ada.Strings.Fixed.Index (To_String (D.Spoken), S) > 0);

   procedure In_Order is
      D : Fake;
      R : Run_Report;
      L : Driver.Brain.Termination.Last_Endings := Driver.Brain.Termination.No_Stretch_Yet;
   begin
      Go ("do grasper touching ball until touched" & LF & "say hi" & LF & "do grasper open until settled", D, R, L, Ball);
      Check (Natural (D.Done_Wants.Length) = 2 and then Natural (R.Events.Length) = 3 and then R.How = Finished
             and then R.Moved and then Spoken (D, "hi;"), "two stretches and a sentence, in order");
      Check (D.Done_Wants (1).Constraints (1).Object.Kind = Driver.Action.Thing_Operand
             and then Integer (D.Done_Wants (1).Constraints (1).Object.Thing) = 3, "the name became its thing");
   end In_Order;

   procedure Try_Branches is
      D : Fake;
      R : Run_Report;
      L : Driver.Brain.Termination.Last_Endings := Driver.Brain.Termination.No_Stretch_Yet;
   begin
      D.Script.Append (Driver.Action.Stuck);
      Go ("try:" & LF & "do grasper touching ball until touched" & LF & "do grasper touching cup until touched" & LF
          & "or:" & LF & "say fallback" & LF & "end" & LF & "say after", D, R, L, Ball);
      Check (Natural (D.Done_Wants.Length) = 1 and then Spoken (D, "fallback;after;"),
             "a stuck stretch abandons the attempt for the or lines, then the program goes on");
      D := (others => <>);
      D.Script.Append (Driver.Action.Slipped);
      Go ("to a:" & LF & "do grasper touching ball until touched" & LF & "do grasper touching cup until touched" & LF
          & "end" & LF & "try:" & LF & "run a" & LF & "or:" & LF & "say caught" & LF & "end", D, R, L, Ball);
      Check (Natural (D.Done_Wants.Length) = 1 and then Spoken (D, "caught;"),
             "a failure inside a called behaviour reaches the try that called it");
      D := (others => <>);
      D.Refuse_All := True;
      Go ("try:" & LF & "do grasper touching ball until touched" & LF & "or:" & LF & "say refused" & LF & "end", D, R, L,
          Ball);
      Check (D.Done_Wants.Is_Empty and then Spoken (D, "refused;") and then not R.Moved
             and then R.Events (1).Ending = Driver.Action.Refused,
             "a stretch the body refuses just before moving does not move, ends refused and fails the attempt");
   end Try_Branches;

   procedure Loops is
      D : Fake;
      R : Run_Report;
      L : Driver.Brain.Termination.Last_Endings := Driver.Brain.Termination.No_Stretch_Yet;
   begin
      D.Script.Append (Driver.Action.Timeout);
      D.Script.Append (Driver.Action.Timeout);
      D.Script.Append (Driver.Action.Touched);
      D.Script.Append (Driver.Action.Settled);
      Go ("repeat until touched:" & LF & "do grasper nearer ball until touched or 3 steps" & LF & "end", D, R, L, Ball);
      Check (Natural (D.Done_Wants.Length) = 3, "repeat until stops after the pass that ended touched");
      D := (others => <>);
      Go ("repeat 3 times:" & LF & "do grasper still until settled" & LF & "end", D, R, L, Ball);
      Check (Natural (D.Done_Wants.Length) = 3, "repeat 3 times runs three passes");
   end Loops;

   procedure Last_Ending is
      D : Fake;
      R : Run_Report;
      L : Driver.Brain.Termination.Last_Endings := Driver.Brain.Termination.Exactly (Driver.Action.Slipped);
   begin
      Go ("if slipped:" & LF & "say it fell" & LF & "else:" & LF & "say it held" & LF & "end", D, R, L);
      Check (Spoken (D, "it fell;") and then not Spoken (D, "held"),
             "if reads how the last stretch ended, also when it ran in the program before");
      D.Script.Append (Driver.Action.Touched);
      Go ("do grasper touching ball until touched", D, R, L, Ball);
      Check (L.Endings (Driver.Action.Touched) and then not L.Endings (Driver.Action.Slipped),
             "the last ending is passed on to the next program");
   end Last_Ending;

   procedure Stops is
      D : Fake;
      R : Run_Report;
      L : Driver.Brain.Termination.Last_Endings := Driver.Brain.Termination.No_Stretch_Yet;
   begin
      Go ("say a" & LF & "done" & LF & "say b", D, R, L);
      Check (R.How = Said_Done and then Spoken (D, "a;") and then not Spoken (D, "b;"), "nothing runs after done");
      D := (others => <>);
      D.Stop_After := 0;
      Go ("say a", D, R, L);
      Check (R.How = Stopped and then Length (D.Spoken) = 0, "an interrupted run stops before the next statement");
   end Stops;

   procedure Places is
      D : Fake;
      R : Run_Report;
      L : Driver.Brain.Termination.Last_Endings := Driver.Brain.Termination.No_Stretch_Yet;
   begin
      Go ("remember where grasper is as home" & LF & "do grasper touching Home until touched", D, R, L);
      Check (Natural (D.Done_Wants.Length) = 1
             and then D.Done_Wants (1).Constraints (1).Object.Kind = Driver.Action.Place_Operand
             and then D.Done_Wants (1).Constraints (1).Object.Place = 1,
             "a place remembered in the program is the place a later stretch goes to, by its letters");
      D := (others => <>);
      Go ("do grasper touching home until touched" & LF & "remember where grasper is as home", D, R, L);
      Check (D.Done_Wants.Is_Empty and then R.Events (1).Ending = Driver.Action.Refused,
             "a place used before it is remembered is refused when that stretch comes, without moving");
   end Places;

   procedure Building is
      P   : Program;
      Ok  : Boolean;
      Why : Refusal;
      W   : Driver.Action.Want;
      Res : Driver.Brain.Wants.Build_Result;
      Q   : Driver.Brain.Keyboard.Word_Vectors.Vector;

      procedure Build_First (Text : String) is
      begin
         Driver.Brain.Parser.Parse (Text, P, Ok, Why);
         Driver.Brain.Wants.Build (P.Statements (P.Blocks (Top).First_Element), Ball, Q, W, Res, Why);
      end Build_First;
   begin
      Q.Append ("height");
      Q.Append ("heading");
      Build_First ("do ball heading down until settled");
      Check (Res = Driver.Brain.Wants.Built and then W.Kind = Driver.Action.Change and then W.Quantity = 2
             and then not W.Increase and then Integer (W.Thing) = 3,
             "the quantity sentence is a change of the thing's quantity, by the quantity's place in the list");
      Build_First ("do ball tilt up until settled");
      Check (Res = Driver.Brain.Wants.Refused and then Ada.Strings.Fixed.Index (To_String (Why.Instead), "height") > 0,
             "a quantity not measured this round is refused with the ones that are");
      Build_First ("do grasper height up until settled");
      Check (Res = Driver.Brain.Wants.Refused, "a quantity is said of a thing, not of a part of me");
      Build_First ("do grasper touching the plate until touched");
      Check (Res = Driver.Brain.Wants.Later, "a name no binding knows (a place remembered later) waits until it runs");
      Build_First ("do the plate height up until settled");
      Check (Res = Driver.Brain.Wants.Refused, "a place has no quantity");
   end Building;

   procedure Register is
   begin
      Register ("brain.execution.order", "statements run out of order, or a name reaches the body unbound",
                In_Order'Access);
      Register ("brain.execution.try", "a failure inside try does not reach its or lines", Try_Branches'Access);
      Register ("brain.execution.loops", "a loop runs too few or too many passes", Loops'Access);
      Register ("brain.execution.last", "if reads something other than how the last stretch ended", Last_Ending'Access);
      Register ("brain.execution.stops", "lines run after done, or after the run was interrupted", Stops'Access);
      Register ("brain.execution.places", "a remembered place is lost, or used before it exists", Places'Access);
      Register ("brain.wants.build", "a stretch reaches the body as the wrong want", Building'Access);
   end Register;

end Driver.Brain.Execution.Tests;
