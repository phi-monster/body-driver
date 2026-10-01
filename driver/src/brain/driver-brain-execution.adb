with Driver.Brain.Words;

package body Driver.Brain.Execution is

   use Driver.Brain.Programs;
   use type Driver.Action.Ending;
   use type Driver.Brain.Wants.Build_Result;

   type Frame_Kind is (Plain, Times, Until_Loop, Attempt);
   --  Plain: the lines of a branch, a behaviour or the program; the others
   --  are loops and try attempts, which act when their lines run out.

   type Frame is record
      Kind   : Frame_Kind := Plain;
      Block  : Block_Index := Top;
      Next   : Positive := Positive'First;   --  the position of the next line in the block
      Header : Statement_Id := Statement_Id'First;
      Left   : Natural := 0;                 --  Times: passes still to run after this one
   end record;

   package Frame_Vectors is new Ada.Containers.Vectors (Positive, Frame);

   procedure Run
     (P      : Program;
      Bound  : in out Driver.Brain.Wants.Binding_Maps.Map;
      Names  : in out Driver.Brain.Names.Table;
      Doer   : in out Performer'Class;
      Last   : in out Driver.Brain.Termination.Last_Endings;
      Report : out Run_Report)
   is
      Stack  : Frame_Vectors.Vector;
      Unseen : Boolean := False;
      --  A stretch ran, outside every try, whose ending no if or repeat until
      --  has read since: a done after it cannot be the brain's judgment of it.

      procedure Push (Kind : Frame_Kind; Block : Block_Index; Header : Statement_Id; Left : Natural := 0) is
      begin
         Stack.Append (Frame'(Kind => Kind, Block => Block, Next => Positive'First, Header => Header, Left => Left));
      end Push;

      function Definition (Called : String) return Block_Id is
      begin
         for S of P.Statements loop
            if S.Kind = Define and then Driver.Brain.Names.Same_Name (To_String (S.Behaviour), Called) then
               return S.Definition;
            end if;
         end loop;
         return No_Block;
      end Definition;

      procedure Note (S : Statement; Kind : Event_Kind; Ending : Driver.Action.Ending; Account : String) is
      begin
         Report.Events.Append (Event'(Kind => Kind, Line => S.Line, Text => S.Text, Ending => Ending,
                                      Account => To_Unbounded_String (Account)));
      end Note;

      --  A stretch ended with E. Inside a try, a failure abandons the attempt
      --  and everything it called, for the or lines.
      procedure Ended (E : Driver.Action.Ending) is
      begin
         Last := Driver.Brain.Termination.Exactly (E);
         if not (for some F of Stack => F.Kind = Attempt) then
            Unseen := True;
         end if;
         if not Driver.Brain.Words.Is_Failure (E) then
            return;
         end if;
         for I in reverse Stack.First_Index .. Stack.Last_Index loop
            if Stack (I).Kind = Attempt then
               declare
                  Header : constant Statement_Id := Stack (I).Header;
               begin
                  while Stack.Last_Index >= I loop
                     Stack.Delete_Last;
                  end loop;
                  if P.Statements (Header).Alternative /= No_Block then
                     Push (Plain, P.Statements (Header).Alternative, Header);
                  end if;
               end;
               return;
            end if;
         end loop;
      end Ended;

      procedure Run_Stretch (S : Statement) is
         W      : Driver.Action.Want;
         Result : Driver.Brain.Wants.Build_Result;
         Why    : Refusal;
      begin
         Driver.Brain.Wants.Build (S, Bound, Doer.Quantities, W, Result, Why);
         case Result is
            when Driver.Brain.Wants.Refused =>
               Note (S, Stretch, Driver.Action.Refused, To_String (Why.Why)
                     & (if Length (Why.Instead) > 0 then "; " & To_String (Why.Instead) else ""));
               Ended (Driver.Action.Refused);
            when Driver.Brain.Wants.Later =>
               Note (S, Stretch, Driver.Action.Refused,
                     "it names a place this program remembers further on, which is not remembered yet");
               Ended (Driver.Action.Refused);
            when Driver.Brain.Wants.Built =>
               --  Checked again just before it moves: the world may have changed
               --  since the program was read.
               declare
                  V : constant Driver.Action.Verdict := Doer.Check (W);
               begin
                  if not V.Ok then
                     Note (S, Stretch, Driver.Action.Refused, To_String (V.Why)
                           & (if Length (V.Alternative) > 0 then "; " & To_String (V.Alternative) else ""));
                     Ended (Driver.Action.Refused);
                     return;
                  end if;
               end;
               declare
                  R : Driver.Action.Result;
               begin
                  Doer.Execute (W, R);
                  Report.Moved := True;
                  Note (S, Stretch, R.Final, To_String (R.Account)
                        & (if R.Final = Driver.Action.Refused and then Length (R.Tried) > 0
                           then " (tried: " & To_String (R.Tried) & ")" else ""));
                  Ended (R.Final);
               end;
         end case;
      end Run_Stretch;

      procedure Run_Remember (S : Statement) is
         Who   : Driver.Action.Operand := (Kind => Driver.Action.Nothing);
         Place : Driver.World.Place_Id;
         Ok    : Boolean;
         Why   : Unbounded_String;
      begin
         case S.Who.Kind is
            when Role_Noun =>
               Who := (Kind => Driver.Action.Role_Operand, The_Role => S.Who.Role);
            when Name_Noun =>
               declare
                  B : constant Driver.Brain.Names.Binding := Driver.Brain.Wants.Find (Bound, To_String (S.Who.Name));
               begin
                  case B.Kind is
                     when Driver.Brain.Names.To_Thing =>
                        Who := (Kind => Driver.Action.Thing_Operand, Thing => B.Thing);
                     when Driver.Brain.Names.To_Place =>
                        Who := (Kind => Driver.Action.Place_Operand, Place => B.Place);
                     when Driver.Brain.Names.Unbound =>
                        Note (S, Not_Remembered, Driver.Action.Refused, "I could not tell what """
                              & To_String (S.Who.Name) & """ is, so I remembered nothing; I went on with the rest");
                        return;
                  end case;
               end;
            when Nothing =>
               null;
         end case;
         Doer.Place_Of (Who, Place, Ok, Why);
         if Ok then
            Names.Name_Place (To_String (S.Place), Place);
            Bound.Include (To_String (S.Place), (Kind => Driver.Brain.Names.To_Place, Place => Place, others => <>));
            Note (S, Remembered, Driver.Action.Refused, "remembered as """ & To_String (S.Place) & """");
         else
            Note (S, Not_Remembered, Driver.Action.Refused,
                  "I could not remember it (" & To_String (Why) & "); I went on with the rest");
         end if;
      end Run_Remember;

   begin
      Report := (others => <>);
      Push (Plain, Top, Statement_Id'First);
      while not Stack.Is_Empty loop
         declare
            F     : Frame := Stack.Last_Element;
            Lines : constant Id_Vectors.Vector := P.Blocks (F.Block);
         begin
            if F.Next > Lines.Last_Index then
               case F.Kind is
                  when Plain | Attempt =>
                     Stack.Delete_Last;
                  when Times =>
                     if F.Left > 0 then
                        Stack.Replace_Element (Stack.Last_Index, (F with delta Left => F.Left - 1,
                                                                             Next => Positive'First));
                     else
                        Stack.Delete_Last;
                     end if;
                  when Until_Loop =>
                     Unseen := False;
                     if Last.Endings (P.Statements (F.Header).Exit_Ending) then
                        Stack.Delete_Last;
                     else
                        Stack.Replace_Element (Stack.Last_Index, (F with delta Next => Positive'First));
                     end if;
               end case;
            elsif Doer.Interrupted then
               Report.How := Stopped;
               return;
            else
               declare
                  Id : constant Statement_Id := Lines (F.Next);
                  S  : constant Statement := P.Statements (Id);
               begin
                  F.Next := F.Next + 1;
                  Stack.Replace_Element (Stack.Last_Index, F);
                  case S.Kind is
                     when Interval =>
                        Run_Stretch (S);
                     when Repeat_Times =>
                        if S.Count > 0 then
                           Push (Times, S.Times_Body, Id, S.Count - 1);
                        end if;
                     when Repeat_Until =>
                        Push (Until_Loop, S.Until_Body, Id);
                     when If_Ending =>
                        Unseen := False;
                        if Last.Endings (S.Test) then
                           Push (Plain, S.Then_Block, Id);
                        elsif S.Else_Block /= No_Block then
                           Push (Plain, S.Else_Block, Id);
                        end if;
                     when Try_Or =>
                        Push (Attempt, S.Attempt, Id);
                     when Define =>
                        null;
                     when Run =>
                        Push (Plain, Definition (To_String (S.Called)), Id);
                     when Remember =>
                        Run_Remember (S);
                     when Say =>
                        Doer.Say (To_String (S.Sentence));
                        Note (S, Spoken, Driver.Action.Refused, To_String (S.Sentence));
                     when Done =>
                        Report.How := Said_Done;
                        Report.Done_Line := S.Line;
                        Report.Done_Seen := not Unseen;
                        return;
                  end case;
               end;
            end if;
         end;
      end loop;
      Report.How := Finished;
   end Run;

end Driver.Brain.Execution;
