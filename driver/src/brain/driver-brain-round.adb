with Ada.Characters.Handling;
with Ada.Strings.Fixed;
with Driver.Brain.Words;
with Driver.Log;
with Driver.Observations;

package body Driver.Brain.Round is

   use type Driver.Robot.Mount_Kind;
   use type Driver.Brain.Execution.Run_End;

   NL : constant String := [ASCII.LF];

   function Eye_Name (E : Driver.Brain.Names.Eye_Id) return String is
     ("eye " & Ada.Strings.Fixed.Trim (Driver.Brain.Names.Eye_Id'Image (E), Ada.Strings.Both));

   function Mount_Text (M : Driver.Robot.Mount) return String is
     (case M.Kind is
         when Driver.Robot.Unmeasured      => "its mounting is not measured yet",
         when Driver.Robot.World_Fixed     => "fixed in the scene",
         when Driver.Robot.Arm_Carried     =>
            "carried by arm " & Ada.Strings.Fixed.Trim (Driver.Robot.Arm_Id'Image (M.Arm), Ada.Strings.Both),
         when Driver.Robot.Carrier_Carried => "carried by the base that moves the whole body");

   function Mount_Of (F : Facts; E : Driver.Brain.Names.Eye_Id) return String is
   begin
      for X of F.Eyes loop
         if Driver.Observations."=" (X.Eye, E) then
            return Mount_Text (X.Mount);
         end if;
      end loop;
      return Mount_Text ((Kind => Driver.Robot.Unmeasured));
   end Mount_Of;

   function Eye_List (V : Driver.Brain.Names.Eye_Vectors.Vector) return String is
      R : Unbounded_String;
   begin
      for E of V loop
         Append (R, (if Length (R) > 0 then ", " else "") & Eye_Name (E));
      end loop;
      return To_String (R);
   end Eye_List;

   function Thing_Line (T : Thing_Facts) return String is
     ("- """ & To_String (T.Name) & """: "
      & (if T.Seen_By.Is_Empty then "no eye sees it now" else Eye_List (T.Seen_By) & " "
         & (if T.Seen_By.First_Index = T.Seen_By.Last_Index then "sees" else "see") & " it now")
      & (if Driver.Uncertain.Known (T.Height)
         then "; " & Driver.Log.Image (T.Height.Value) & " +- " & Driver.Log.Image (T.Height.Sigma)
              & " above the surface it rests on, in my own length unit"
         else "")
      & (if T.Held then "; in my grasp" else ""));

   function Prompt (F : Facts) return String is
      S : Unbounded_String;

      procedure Put (Line : String) is
      begin
         Append (S, Line & NL);
      end Put;
   begin
      Put ("You are the brain of a robot body, and I am that body. You move me only by writing programs in my language."
           & " I check a whole program against what I have measured about myself before anything moves, run it, and ask"
           & " you again when it is over, with new pictures and an account of what happened.");
      Put ("");
      Put ("THE PICTURE");
      Put ("The large picture is what my " & Eye_Name (F.View) & " sees now; it is " & Mount_Of (F, F.View) & ".");
      if not F.Strip.Is_Empty then
         declare
            Under : Unbounded_String;
         begin
            for E of F.Strip loop
               Append (Under, (if Length (Under) > 0 then ", " else "") & Eye_Name (E) & " (" & Mount_Of (F, E) & ")");
            end loop;
            Put ("Below it, from left to right: " & To_String (Under) & ".");
         end;
      end if;
      Put ("");
      Put ("THINGS YOU HAVE NAMED");
      if F.Things.Is_Empty then
         Put ("None yet. Name a thing in your own words; I ask my eyes where it is.");
      else
         for T of F.Things loop
            Put (Thing_Line (T));
         end loop;
      end if;
      Put ("");
      Put ("WHAT HAPPENED");
      Put (To_String (F.Happened));
      Put ("");
      Put ("YOUR TASK");
      Put (To_String (F.Instruction));
      Put ("");
      Put ("THE LANGUAGE THIS ROUND (the only text the decoder lets you type)");
      Append (S, To_String (F.Sheet));
      Put ("");
      Put ("Write the program now, one statement per line.");
      return To_String (S);
   end Prompt;

   function Happened_Text (Report : Driver.Brain.Execution.Run_Report; Cut : String) return String is
      S : Unbounded_String;

      procedure Put (Line : String) is
      begin
         Append (S, (if Length (S) > 0 then NL else "") & Line);
      end Put;
   begin
      if Cut'Length > 0 then
         Put ("Your last answer was cut short (" & Cut & "); the lines before the cut ran.");
      end if;
      for E of Report.Events loop
         declare
            Line : constant String := "line " & Driver.Log.Image (E.Line) & ": ";
         begin
            case E.Kind is
               when Driver.Brain.Execution.Stretch =>
                  Put (Line & To_String (E.Text) & " -- ended " & Driver.Brain.Words.Word (E.Ending)
                       & (if Length (E.Account) > 0 then ": " & To_String (E.Account) else ""));
               when Driver.Brain.Execution.Spoken =>
                  Put (Line & "you said: " & To_String (E.Account));
               when Driver.Brain.Execution.Remembered | Driver.Brain.Execution.Not_Remembered =>
                  Put (Line & To_String (E.Account));
            end case;
         end;
      end loop;
      if Report.How = Driver.Brain.Execution.Said_Done and then not Report.Done_Seen then
         Put ("line " & Driver.Log.Image (Report.Done_Line)
              & ": done came before you saw how the lines above it ended, so I ask you again.");
      end if;
      if Report.Events.Is_Empty then
         Put ("Your last program had nothing to run.");
      end if;
      if Report.How = Driver.Brain.Execution.Stopped then
         Put ("The program was interrupted.");
      elsif not Report.Moved then
         Put ("Nothing of me moved.");
      end if;
      return To_String (S);
   end Happened_Text;

   function Refusal_Text (R : Driver.Brain.Programs.Refusal) return String is
     ("I refused your last program before anything moved"
      & (if R.Line > 0 then ": line " & Driver.Log.Image (R.Line) else "") & ": " & To_String (R.Why)
      & (if Length (R.Instead) > 0 then ". Instead: " & To_String (R.Instead) else "") & ".");

   function Look_At (Sentence : String) return Natural is
      Look    : constant String := "look";
      S       : constant String := Ada.Characters.Handling.To_Lower (Sentence);
      Word_At : constant Natural := Ada.Strings.Fixed.Index (S, Look);
      I       : Natural;
   begin
      if Word_At = 0 then
         return 0;
      end if;
      I := Word_At + Look'Length;
      while I <= S'Last and then S (I) = ' ' loop
         I := I + 1;
      end loop;
      if I > S'Last or else S (I) /= '=' then
         return 0;
      end if;
      I := I + 1;
      while I <= S'Last and then S (I) = ' ' loop
         I := I + 1;
      end loop;
      declare
         First : constant Positive := I;
      begin
         while I <= S'Last and then S (I) in '0' .. '9' loop
            I := I + 1;
         end loop;
         return (if I = First then 0 else Natural'Value (S (First .. I - 1)));
      exception
         when Constraint_Error =>
            return 0;
      end;
   end Look_At;

end Driver.Brain.Round;
