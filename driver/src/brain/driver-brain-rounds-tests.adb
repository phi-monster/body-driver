with Ada.Containers.Indefinite_Vectors;
with Ada.Containers.Vectors;
with Ada.Strings.Fixed;
with Driver.Action;
with Driver.Bytes;
with Driver.Robot;
with Driver.Tests;
with Driver.World;

package body Driver.Brain.Rounds.Tests is

   use Driver.Tests;
   use type Driver.Action.Want_Kind;
   use type Driver.Observations.Camera_Id;

   LF : constant String := [ASCII.LF];

   package Text_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);

   function Has (S, Part : String) return Boolean is (Ada.Strings.Fixed.Index (S, Part) > 0);

   function Picture return Driver.Images.Image is
      Data : constant Driver.Bytes.Byte_Array (1 .. 12) := [others => 100];
   begin
      return Driver.Images.Create (2, 2, Data);
   end Picture;

   function Quantity_Keys return Driver.Brain.Keyboard.Keyboard is
      Q, M : Driver.Brain.Keyboard.Word_Vectors.Vector;
   begin
      Q.Append ("height");
      M.Append ("how high it is above what it rests on");
      return Driver.Brain.Keyboard.Choose (Q, M, [Driver.Action.Grasper => True, others => False], [others => True],
                                           False, [others => False]);
   end Quantity_Keys;

   --  A scene with two eyes, the first on arm 1, the second fixed; the
   --  episode ends after Looks_Left looks.
   type Scene is new Surroundings with record
      Looks_Left : Natural := Natural'Last;
   end record;

   overriding procedure Look (S : in out Scene; Named : Driver.Brain.Names.Table; Now : out Snapshot);
   overriding function Episode_Over (S : Scene) return Boolean;

   overriding procedure Look (S : in out Scene; Named : Driver.Brain.Names.Table; Now : out Snapshot) is
      pragma Unreferenced (Named);
   begin
      if S.Looks_Left > 0 then
         S.Looks_Left := S.Looks_Left - 1;
      end if;
      Now := (others => <>);
      Now.Images.Append (Picture);
      Now.Images.Append (Picture);
      Now.Eyes.Append (Driver.Brain.Round.Eye_Facts'(Eye => 1, Mount => (Kind => Driver.Robot.Arm_Carried, Arm => 1)));
      Now.Eyes.Append (Driver.Brain.Round.Eye_Facts'(Eye => 2, Mount => (Kind => Driver.Robot.World_Fixed)));
      Now.Keys := Quantity_Keys;
   end Look;

   overriding function Episode_Over (S : Scene) return Boolean is (S.Looks_Left = 0);

   --  A brain that answers from a script and keeps every prompt.
   type Scripted is new Thinker with record
      Programs : Text_Vectors.Vector;
      Prompts  : Text_Vectors.Vector;
   end record;

   overriding function Write_Program
     (T       : in out Scripted;
      Picture : Driver.Images.Image;
      Prompt  : String;
      Keys    : Driver.Brain.Keyboard.Keyboard) return Driver.Brain.Service.Answer;

   overriding function Write_Program
     (T       : in out Scripted;
      Picture : Driver.Images.Image;
      Prompt  : String;
      Keys    : Driver.Brain.Keyboard.Keyboard) return Driver.Brain.Service.Answer
   is
      pragma Unreferenced (Picture, Keys);
      Text : constant String := (if T.Programs.Is_Empty then "done" & LF else T.Programs.First_Element);
   begin
      T.Prompts.Append (Prompt);
      if not T.Programs.Is_Empty then
         T.Programs.Delete_First;
      end if;
      return (How => Driver.Brain.Service.Ended, Program => To_Unbounded_String (Text), others => <>);
   end Write_Program;

   --  Eyes that box the scissors as thing 7, in eye 1.
   type Eyes_Fixture is new Driver.Brain.Names.Senses with null record;

   overriding function Eyes (S : Eyes_Fixture) return Driver.Brain.Names.Eye_Vectors.Vector;
   overriding function Sees (S : Eyes_Fixture; T : Driver.Brain.Names.Thing_Id; E : Driver.Brain.Names.Eye_Id)
                             return Boolean;
   overriding procedure Ask_Where
     (S      : in out Eyes_Fixture;
      E      : Driver.Brain.Names.Eye_Id;
      Name   : String;
      Answer : out Driver.Brain.Names.Pointing;
      Where  : out Driver.Brain.Names.Box;
      Why    : out Unbounded_String);
   overriding procedure Identify
     (S     : in out Eyes_Fixture;
      E     : Driver.Brain.Names.Eye_Id;
      Where : Driver.Brain.Names.Box;
      Found : out Driver.Brain.Names.Patch;
      T     : out Driver.Brain.Names.Thing_Id;
      Why   : out Unbounded_String);

   overriding function Eyes (S : Eyes_Fixture) return Driver.Brain.Names.Eye_Vectors.Vector is
      pragma Unreferenced (S);
      R : Driver.Brain.Names.Eye_Vectors.Vector;
   begin
      R.Append (1);
      R.Append (2);
      return R;
   end Eyes;

   overriding function Sees (S : Eyes_Fixture; T : Driver.Brain.Names.Thing_Id; E : Driver.Brain.Names.Eye_Id)
                             return Boolean is (False);

   overriding procedure Ask_Where
     (S      : in out Eyes_Fixture;
      E      : Driver.Brain.Names.Eye_Id;
      Name   : String;
      Answer : out Driver.Brain.Names.Pointing;
      Where  : out Driver.Brain.Names.Box;
      Why    : out Unbounded_String)
   is
      pragma Unreferenced (S);
   begin
      Answer := (if E = 1 and then Name = "scissors" then Driver.Brain.Names.Boxed else Driver.Brain.Names.Not_Here);
      Where := (others => <>);
      Why := Null_Unbounded_String;
   end Ask_Where;

   overriding procedure Identify
     (S     : in out Eyes_Fixture;
      E     : Driver.Brain.Names.Eye_Id;
      Where : Driver.Brain.Names.Box;
      Found : out Driver.Brain.Names.Patch;
      T     : out Driver.Brain.Names.Thing_Id;
      Why   : out Unbounded_String)
   is
      pragma Unreferenced (S, E, Where);
   begin
      Found := Driver.Brain.Names.A_Thing;
      T := 7;
      Why := Null_Unbounded_String;
   end Identify;

   package Want_Vectors is new Ada.Containers.Vectors (Positive, Driver.Action.Want, Driver.Action."=");

   --  A body that ends every stretch settled and records it.
   type Body_Fixture is new Driver.Brain.Execution.Performer with record
      Done_Wants : Want_Vectors.Vector;
   end record;

   overriding function Quantities (P : Body_Fixture) return Driver.Brain.Keyboard.Word_Vectors.Vector;
   overriding function Check (P : Body_Fixture; W : Driver.Action.Want) return Driver.Action.Verdict;
   overriding procedure Execute (P : in out Body_Fixture; W : Driver.Action.Want; R : out Driver.Action.Result);
   overriding procedure Place_Of
     (P     : in out Body_Fixture;
      Who   : Driver.Action.Operand;
      Place : out Driver.World.Place_Id;
      Ok    : out Boolean;
      Why   : out Unbounded_String);
   overriding procedure Say (P : in out Body_Fixture; Sentence : String);
   overriding function Interrupted (P : Body_Fixture) return Boolean;

   overriding function Quantities (P : Body_Fixture) return Driver.Brain.Keyboard.Word_Vectors.Vector is
     (Quantity_Keys.Quantities);
   overriding function Check (P : Body_Fixture; W : Driver.Action.Want) return Driver.Action.Verdict is
     ((Ok => True));

   overriding procedure Execute (P : in out Body_Fixture; W : Driver.Action.Want; R : out Driver.Action.Result) is
   begin
      P.Done_Wants.Append (W);
      R := (Final => Driver.Action.Settled, Tried => Null_Unbounded_String,
            Account => To_Unbounded_String ("it rose by 0.2 of my length unit"));
   end Execute;

   overriding procedure Place_Of
     (P     : in out Body_Fixture;
      Who   : Driver.Action.Operand;
      Place : out Driver.World.Place_Id;
      Ok    : out Boolean;
      Why   : out Unbounded_String)
   is
      pragma Unreferenced (P, Who);
   begin
      Place := 1;
      Ok := True;
      Why := Null_Unbounded_String;
   end Place_Of;

   overriding procedure Say (P : in out Body_Fixture; Sentence : String) is null;
   overriding function Interrupted (P : Body_Fixture) return Boolean is (False);

   procedure Episode (Programs : Text_Vectors.Vector; Brain : out Scripted; Doer : out Body_Fixture;
                      Looks : Natural := Natural'Last) is
      Around : Scene := (Looks_Left => Looks);
      Eyes   : Eyes_Fixture;
   begin
      Brain := (Programs => Programs, Prompts => <>);
      Doer := (Done_Wants => <>);
      Run ("Pick up the scissors.", Around, Brain, Eyes, Doer);
   end Episode;

   function Script (A, B, C : String := "") return Text_Vectors.Vector is
      R : Text_Vectors.Vector;
   begin
      for S of Text_Vectors.Vector'([A, B, C]) loop
         if S'Length > 0 then
            R.Append (S);
         end if;
      end loop;
      return R;
   end Script;

   procedure Lift_And_Done is
      Brain : Scripted;
      Doer  : Body_Fixture;
   begin
      Episode (Script ("do scissors height up until settled" & LF & "say I lifted it" & LF,
                       "done" & LF), Brain, Doer);
      Check (Natural (Doer.Done_Wants.Length) = 1 and then Doer.Done_Wants (1).Kind = Driver.Action.Change
             and then Integer (Doer.Done_Wants (1).Thing) = 7 and then Doer.Done_Wants (1).Increase,
             "the name became the eye's thing and the stretch reached the body as a change of its height");
      Check (Natural (Brain.Prompts.Length) = 2, "done ends the episode's rounds");
      Check (Has (Brain.Prompts (1), Driver.Brain.Round.First_Round) and then Has (Brain.Prompts (1), "Pick up the scissors")
             and then Has (Brain.Prompts (1), "<change>"), "the first prompt: no history, the task, the keyboard");
      Check (Has (Brain.Prompts (2), "line 1: do scissors height up until settled -- ended settled: it rose")
             and then Has (Brain.Prompts (2), "you said: I lifted it"),
             "the next round tells how every stretch ended and what the body said");
   end Lift_And_Done;

   procedure Refused_Then_Told is
      Brain : Scripted;
      Doer  : Body_Fixture;
   begin
      Episode (Script ("do grasper height up until settled" & LF, "done" & LF), Brain, Doer);
      Check (Doer.Done_Wants.Is_Empty, "a refused program moves nothing");
      Check (Has (Brain.Prompts (2), "I refused your last program before anything moved: line 1")
             and then Has (Brain.Prompts (2), "said of a thing you see"),
             "the next round says which line was refused and why");
   end Refused_Then_Told;

   procedure Eye_Switch is
      Brain : Scripted;
      Doer  : Body_Fixture;
   begin
      Episode (Script ("say look = 1" & LF, "done" & LF), Brain, Doer);
      Check (Has (Brain.Prompts (1), "what my eye 2 sees now; it is fixed in the scene")
             and then Has (Brain.Prompts (1), "Below it, from left to right: eye 1 (carried by arm 1)"),
             "the first round looks through the eye fixed in the scene, the others shown below");
      Check (Has (Brain.Prompts (2), "what my eye 1 sees now"), "say look = 1 switches the next round's eye");
   end Eye_Switch;

   procedure Same_Again is
      Brain : Scripted;
      Doer  : Body_Fixture;
   begin
      Episode (Script ("say I wait" & LF, "say I wait" & LF, "done" & LF), Brain, Doer);
      Check (Has (Brain.Prompts (3), "You wrote the same program as before, and the one before moved nothing"),
             "the same program after one that moved nothing is said to be the same");
   end Same_Again;

   procedure Episode_Ends is
      Brain : Scripted;
      Doer  : Body_Fixture;
   begin
      Episode (Script ("do scissors height up until settled" & LF), Brain, Doer, Looks => 1);
      Check (Doer.Done_Wants.Is_Empty, "a program written while the episode ended does not run");
   end Episode_Ends;

   procedure Register is
   begin
      Register ("brain.rounds.lift", "a round loses the name, the change, or the account of how it ended",
                Lift_And_Done'Access);
      Register ("brain.rounds.refused", "a refused program moves the body, or the brain is not told why",
                Refused_Then_Told'Access);
      Register ("brain.rounds.look", "the brain looks through the wrong eye", Eye_Switch'Access);
      Register ("brain.rounds.same", "the brain is not told it repeats a program that moved nothing", Same_Again'Access);
      Register ("brain.rounds.episode", "a program written for an episode that ended still runs", Episode_Ends'Access);
   end Register;

end Driver.Brain.Rounds.Tests;
