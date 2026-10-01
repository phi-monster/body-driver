with Driver.Action;
with Driver.Brain.Parser;
with Driver.Brain.Pictures;
with Driver.Brain.Programs;
with Driver.Brain.Termination;
with Driver.Brain.Wants;
with Driver.Log;
with Driver.Robot;
with Driver.World;

package body Driver.Brain.Rounds is

   use Driver.Brain.Programs;
   use type Driver.Brain.Service.Reading_End;
   use type Driver.Brain.Execution.Run_End;
   use type Driver.Brain.Execution.Event_Kind;
   use type Driver.Robot.Mount_Kind;
   use type Driver.Brain.Names.Binding_Kind;
   use type Driver.Brain.Wants.Build_Result;
   use type Driver.Observations.Camera_Id;

   subtype Eye_Id is Driver.Brain.Names.Eye_Id;

   procedure Say (Text : String) is
   begin
      Driver.Log.Line (Driver.Log.Brain, Text);
   end Say;

   --  A heading, then every line of Text on a log line of its own.
   procedure Say_Lines (Heading, Text : String) is
      First : Positive := Text'First;
   begin
      Say (Heading);
      for I in Text'Range loop
         if Text (I) = ASCII.LF then
            Say ("  | " & Text (First .. I - 1));
            First := I + 1;
         end if;
      end loop;
      if First <= Text'Last then
         Say ("  | " & Text (First .. Text'Last));
      end if;
   end Say_Lines;

   function Same (A, B : String) return Boolean is (Driver.Brain.Names.Same_Name (A, B));

   function Has_Picture (Now : Snapshot; E : Eye_Id) return Boolean is
     (E in Now.Images.First_Index .. Now.Images.Last_Index and then not Driver.Images.Is_Empty (Now.Images (E)));

   --  The eye a task starts in: the first one fixed in the scene, else the
   --  first with a picture (LANGUAGE.md 17.5: the brain switches with look).
   function First_View (Now : Snapshot) return Eye_Id'Base is
   begin
      for F of Now.Eyes loop
         if F.Mount.Kind = Driver.Robot.World_Fixed and then Has_Picture (Now, F.Eye) then
            return F.Eye;
         end if;
      end loop;
      for E in Now.Images.First_Index .. Now.Images.Last_Index loop
         if Has_Picture (Now, E) then
            return E;
         end if;
      end loop;
      return 0;
   end First_View;

   procedure Run
     (Instruction : String;
      Around      : in out Surroundings'Class;
      Brain       : in out Thinker'Class;
      Eyes        : in out Driver.Brain.Names.Senses'Class;
      Doer        : in out Driver.Brain.Execution.Performer'Class)
   is
      Table          : Driver.Brain.Names.Table;
      Last           : Driver.Brain.Termination.Last_Endings := Driver.Brain.Termination.No_Stretch_Yet;
      Chosen         : Eye_Id'Base := 0;   --  the eye the brain asked to look through
      Happened       : Unbounded_String := To_Unbounded_String (Driver.Brain.Round.First_Round);
      Task_Words     : Unbounded_String := To_Unbounded_String (Instruction);
      Previous       : Unbounded_String;
      Previous_Moved : Boolean := True;
      Round_Number   : Natural := 0;
      Finished       : Boolean := False;

      --  One answer: read, bound, checked and run, or refused before anything
      --  moves. Happened tells the next round which.
      procedure Answer (A : Driver.Brain.Service.Answer; Now : Snapshot; View : Eye_Id) is
         Text : constant String := To_String (A.Program);
         P    : Program;
         Ok   : Boolean;
         Why  : Refusal;

         procedure Refuse (R : Refusal) is
         begin
            Happened := To_Unbounded_String (Driver.Brain.Round.Refusal_Text (R));
            Say_Lines ("refused before anything moved:", To_String (Happened));
         end Refuse;
      begin
         Say_Lines ("the program:", Text);
         if Text'Length = 0 then
            Happened := To_Unbounded_String
              (if A.How = Driver.Brain.Service.Ran_Away
               then "Your last answer ran away before its first line was finished (" & To_String (A.Why)
                    & "), so none of it could run."
               else "Your last answer had no line to run.");
            return;
         end if;
         Driver.Brain.Parser.Parse (Text, P, Ok, Why);
         if not Ok then
            Refuse (Why);
            return;
         end if;
         Why := Driver.Brain.Termination.Check (P, Last, Same'Access);
         if Why.Line > 0 then
            Refuse (Why);
            return;
         end if;
         declare
            Bound : Driver.Brain.Wants.Binding_Maps.Map;
            Glue  : constant Driver.Brain.Keyboard.Word_Vectors.Vector := Driver.Brain.Keyboard.Name_Words (Now.Keys);
            Names : constant Driver.Brain.Keyboard.Word_Vectors.Vector := Driver.Brain.Wants.Names_Of (P);
         begin
            for N of Names loop
               declare
                  B : Driver.Brain.Names.Binding;
               begin
                  Driver.Brain.Names.Bind (Table, Eyes, View, N, Glue, B);
                  Bound.Include (N, B);
               end;
            end loop;
            for N of Names loop
               declare
                  B : Driver.Brain.Names.Binding := Bound.Element (N);
               begin
                  Driver.Brain.Names.Bind_Again (Table, Glue, N, B);
                  Bound.Include (N, B);
                  Say ("""" & N & """: "
                       & (case B.Kind is
                            when Driver.Brain.Names.To_Thing => "thing" & Driver.World.Thing_Id'Image (B.Thing),
                            when Driver.Brain.Names.To_Place => "place" & Driver.World.Place_Id'Image (B.Place),
                            when Driver.Brain.Names.Unbound  => "not bound")
                       & " (" & To_String (B.Account) & ")");
               end;
            end loop;
            for S of P.Statements loop
               if S.Kind = Interval then
                  declare
                     W      : Driver.Action.Want;
                     Result : Driver.Brain.Wants.Build_Result;
                  begin
                     Driver.Brain.Wants.Build (S, Bound, Doer.Quantities, W, Result, Why);
                     if Result = Driver.Brain.Wants.Refused then
                        Refuse (Why);
                        return;
                     elsif Result = Driver.Brain.Wants.Built then
                        declare
                           V : constant Driver.Action.Verdict := Doer.Check (W);
                        begin
                           if not V.Ok then
                              Refuse (Driver.Brain.Programs.Refused (S.Line, To_String (V.Why),
                                                                     To_String (V.Alternative)));
                              return;
                           end if;
                        end;
                     end if;
                  end;
               end if;
            end loop;
            declare
               Report : Driver.Brain.Execution.Run_Report;
               Again  : constant Boolean := Text = To_String (Previous) and then not Previous_Moved;
            begin
               Driver.Brain.Execution.Run (P, Bound, Table, Doer, Last, Report);
               Happened := To_Unbounded_String
                 ((if Again then "You wrote the same program as before, and the one before moved nothing of me. "
                   else "")
                  & Driver.Brain.Round.Happened_Text
                      (Report, (if A.How = Driver.Brain.Service.Ran_Away then To_String (A.Why) else "")));
               if Report.How /= Driver.Brain.Execution.Said_Done then
                  Say_Lines ("what happened:", To_String (Happened));
               end if;
               Previous := A.Program;
               Previous_Moved := Report.Moved;
               for E of Report.Events loop
                  if E.Kind = Driver.Brain.Execution.Spoken then
                     declare
                        K : constant Natural := Driver.Brain.Round.Look_At (To_String (E.Account));
                     begin
                        if K > 0 and then Has_Picture (Now, Eye_Id (K)) then
                           Chosen := Eye_Id (K);
                           Say ("the brain looks through eye" & K'Image & " from the next round on");
                        end if;
                     end;
                  end if;
               end loop;
               Finished := Report.How = Driver.Brain.Execution.Said_Done;
            end;
         end;
      end Answer;

   begin
      while not Around.Episode_Over and then not Finished loop
         declare
            Now : Snapshot;
         begin
            Around.Look (Table, Now);
            if Length (Now.Instruction) > 0 and then Now.Instruction /= Task_Words then
               Task_Words := Now.Instruction;
               Say ("new words from the person: " & To_String (Task_Words));
            end if;
            declare
               View : constant Eye_Id'Base :=
                 (if Chosen > 0 and then Has_Picture (Now, Chosen) then Chosen else First_View (Now));
            begin
               if View = 0 then
                  Say ("no eye has a picture this beat; looking again");
               else
                  declare
                     Strip : Driver.Brain.Names.Eye_Vectors.Vector;

                     function Image_Of (E : Eye_Id) return Driver.Images.Image is (Now.Images (E));
                  begin
                     for E in Now.Images.First_Index .. Now.Images.Last_Index loop
                        if E /= View and then Has_Picture (Now, E) then
                           Strip.Append (E);
                        end if;
                     end loop;
                     Round_Number := Round_Number + 1;
                     Say ("round" & Round_Number'Image & ": the brain looks through eye" & View'Image & ", "
                          & Driver.Brain.Keyboard.Image (Now.Keys.Keys) & " keyboard");
                     declare
                        A : constant Driver.Brain.Service.Answer :=
                          Brain.Write_Program
                            (Driver.Brain.Pictures.Compose (Now.Images (View), Strip, Image_Of'Access),
                             Driver.Brain.Round.Prompt
                               ((View        => View,
                                 Strip       => Strip,
                                 Eyes        => Now.Eyes,
                                 Things      => Now.Things,
                                 Happened    => Happened,
                                 Instruction => Task_Words,
                                 Sheet       => To_Unbounded_String (Driver.Brain.Keyboard.Sheet (Now.Keys)))),
                             Now.Keys);
                     begin
                        if Around.Episode_Over then
                           Say ("the episode ended while the brain was writing");
                        elsif A.How = Driver.Brain.Service.Failed then
                           Happened := "Your last answer could not be read: " & A.Why & ".";
                           Say ("no program: " & To_String (A.Why));
                        else
                           Answer (A, Now, View);
                        end if;
                     end;
                  end;
               end if;
            end;
         end;
      end loop;
      if Finished then
         Say ("the brain said done after" & Round_Number'Image & " rounds");
      end if;
   end Run;

end Driver.Brain.Rounds;
