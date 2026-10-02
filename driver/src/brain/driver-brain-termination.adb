with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Driver.Brain.Words;

package body Driver.Brain.Termination is

   use Ada.Strings.Unbounded;
   use Driver.Brain.Programs;
   use Driver.Brain.Words;

   subtype Ending is Driver.Action.Ending;
   use type Driver.Action.Ending;

   function Exactly (E : Ending) return Last_Endings is ((Endings => [for X in Ending => X = E], None => False));

   Empty : constant Last_Endings := (Endings => [others => False], None => False);

   function Any_Ending (Inside_Try : Boolean; Failing : Boolean) return Last_Endings is
     ((Endings => [for E in Ending => (if Inside_Try then Is_Failure (E) = Failing else not Failing)],
       None => False));
   --  What a stretch may end with: inside a try its failures leave for the
   --  try's alternative, so they are told apart from the rest.

   function "or" (A, B : Last_Endings) return Last_Endings is
     ((Endings => [for E in Ending => A.Endings (E) or else B.Endings (E)], None => A.None or else B.None));

   function Is_Empty (A : Last_Endings) return Boolean is (not A.None and then (for all E in Ending => not A.Endings (E)));

   function Within (A, B : Last_Endings) return Boolean is
     ((not A.None or else B.None) and then (for all E in Ending => not A.Endings (E) or else B.Endings (E)));

   function Only (A : Last_Endings; E : Ending) return Last_Endings is
     ((Endings => [for X in Ending => X = E and then A.Endings (X)], None => False));

   function Except (A : Last_Endings; E : Ending) return Last_Endings is
     ((Endings => [for X in Ending => X /= E and then A.Endings (X)], None => A.None));

   function Listed (A : Last_Endings) return String is
      R : Unbounded_String;
   begin
      if A.None then
         R := To_Unbounded_String ("nothing (no stretch has run)");
      end if;
      for E in Ending loop
         if A.Endings (E) then
            Append (R, (if Length (R) > 0 then " or " else "") & Word (E));
         end if;
      end loop;
      return To_String (R);
   end Listed;

   type Flow is record
      Normal   : Last_Endings := Empty;   --  the last endings when control leaves the normal way
      Escapes  : Last_Endings := Empty;   --  failures that leave for an enclosing try
      Finished : Boolean := False;        --  some path reaches done
   end record;

   function Leaves (F : Flow) return Boolean is
     (not Is_Empty (F.Normal) or else not Is_Empty (F.Escapes) or else F.Finished);

   --  What a call of a behaviour does, by behaviour, input and whether a try
   --  encloses the call; refined from "never leaves" until it stops changing.
   type Summary is record
      Definition : Statement_Id;
      Input      : Last_Endings;
      Inside_Try : Boolean;
      Result     : Flow;
      Active     : Boolean := False;   --  being analysed: a call to it now is recursion
   end record;

   package Summary_Vectors is new Ada.Containers.Vectors (Positive, Summary);

   function Check
     (P      : Program;
      Before : Last_Endings;
      Same   : not null access function (A, B : String) return Boolean) return Refusal
   is
      Summaries : Summary_Vectors.Vector;
      Changed   : Boolean;
      Final     : Boolean := False;   --  errors are recorded only once the summaries are settled
      Error     : Refusal;

      procedure Fail (S : Statement; Why, Instead : String) is
      begin
         if Final and then Error.Line = 0 then
            Error := Refused (S.Line, Why, Instead);
         end if;
      end Fail;

      function Definition_Of (Called : String) return Natural is
         Found : Natural := 0;
      begin
         for I in P.Statements.First_Index .. P.Statements.Last_Index loop
            if P.Statements (I).Kind = Define and then Same (To_String (P.Statements (I).Behaviour), Called) then
               Found := Natural (I);
            end if;
         end loop;
         return Found;
      end Definition_Of;

      function Block_Flow (B : Block_Id; Input : Last_Endings; Inside_Try : Boolean) return Flow;

      function Call_Flow (D : Statement_Id; Input : Last_Endings; Inside_Try : Boolean) return Flow is
         At_Summary : Natural := 0;
      begin
         for I in Summaries.First_Index .. Summaries.Last_Index loop
            if Summaries (I).Definition = D and then Summaries (I).Input = Input
              and then Summaries (I).Inside_Try = Inside_Try
            then
               At_Summary := I;
            end if;
         end loop;
         if At_Summary = 0 then
            Summaries.Append (Summary'(Definition => D, Input => Input, Inside_Try => Inside_Try, Result => <>,
                                       Active => False));
            At_Summary := Summaries.Last_Index;
            Changed := True;
         end if;
         if Summaries (At_Summary).Active then
            return Summaries (At_Summary).Result;
         end if;
         Summaries (At_Summary).Active := True;
         declare
            F : constant Flow := Block_Flow (P.Statements (D).Definition, Input, Inside_Try);
         begin
            Summaries (At_Summary).Active := False;
            if F /= Summaries (At_Summary).Result then
               Summaries (At_Summary).Result := F;
               Changed := True;
            end if;
            return F;
         end;
      end Call_Flow;

      function Statement_Flow (S : Statement; Input : Last_Endings; Inside_Try : Boolean) return Flow is
         F : Flow;
      begin
         case S.Kind is
            when Interval =>
               F.Normal := Any_Ending (Inside_Try, Failing => False);
               F.Escapes := (if Inside_Try then Any_Ending (Inside_Try, Failing => True) else Empty);
            when Say | Remember | Define =>
               F.Normal := Input;
            when Done =>
               F.Finished := True;
            when If_Ending =>
               declare
                  Yes : constant Last_Endings := Only (Input, S.Test);
                  No  : constant Last_Endings := Except (Input, S.Test);
                  T   : constant Flow := (if Is_Empty (Yes) then (others => <>) else Block_Flow (S.Then_Block, Yes,
                                                                                                 Inside_Try));
                  E   : constant Flow := (if Is_Empty (No) then (others => <>)
                                          elsif S.Else_Block = No_Block then (Normal => No, others => <>)
                                          else Block_Flow (S.Else_Block, No, Inside_Try));
               begin
                  F := (Normal => T.Normal or E.Normal, Escapes => T.Escapes or E.Escapes,
                        Finished => T.Finished or else E.Finished);
               end;
            when Try_Or =>
               declare
                  A : constant Flow := Block_Flow (S.Attempt, Input, Inside_Try => True);
               begin
                  if S.Alternative = No_Block then
                     F := (Normal => A.Normal or A.Escapes, Escapes => Empty, Finished => A.Finished);
                  elsif Is_Empty (A.Escapes) then
                     F := (Normal => A.Normal, Escapes => Empty, Finished => A.Finished);
                  else
                     declare
                        Alt : constant Flow := Block_Flow (S.Alternative, A.Escapes, Inside_Try);
                     begin
                        F := (Normal => A.Normal or Alt.Normal, Escapes => Alt.Escapes,
                              Finished => A.Finished or else Alt.Finished);
                     end;
                  end if;
               end;
            when Repeat_Times =>
               if S.Count = 0 then
                  F.Normal := Input;
               else
                  declare
                     Into : Last_Endings := Input;
                     Seen : Last_Endings := Empty;
                  begin
                     for Pass in 1 .. S.Count loop
                        declare
                           G : constant Flow := Block_Flow (S.Times_Body, Into, Inside_Try);
                        begin
                           F.Escapes := F.Escapes or G.Escapes;
                           F.Finished := F.Finished or else G.Finished;
                           exit when Within (G.Normal, Seen);
                           Seen := Seen or G.Normal;
                           Into := G.Normal;
                        end;
                     end loop;
                     F.Normal := Seen;
                  end;
               end if;
            when Repeat_Until =>
               declare
                  Into : Last_Endings := Input;
                  Seen : Last_Endings := Empty;
               begin
                  loop
                     declare
                        G    : constant Flow := Block_Flow (S.Until_Body, Into, Inside_Try);
                        Next : constant Last_Endings := Except (G.Normal, S.Exit_Ending);
                     begin
                        F.Normal := F.Normal or Only (G.Normal, S.Exit_Ending);
                        F.Escapes := F.Escapes or G.Escapes;
                        F.Finished := F.Finished or else G.Finished;
                        exit when Within (Next, Seen);
                        Seen := Seen or Next;
                        Into := Next;
                     end;
                  end loop;
                  if not Leaves (F) then
                     Fail (S, "this loop can never end: on its way no stretch runs, so the last stretch's ending stays "
                           & Listed (Seen) & " and never becomes " & Word (S.Exit_Ending),
                           "put a stretch inside the loop, or write repeat <n> times");
                  end if;
               end;
            when Run =>
               declare
                  D : constant Natural := Definition_Of (To_String (S.Called));
               begin
                  if D = 0 then
                     Fail (S, "no behaviour is called """ & To_String (S.Called) & """",
                           "define it first with to " & To_String (S.Called) & ": <lines> end");
                  else
                     F := Call_Flow (Statement_Id (D), Input, Inside_Try);
                     if not Leaves (F) then
                        Fail (S, """" & To_String (S.Called) & """ can never end: on every path it calls itself again"
                              & " before anything can end it", "repeat its lines <n> times instead");
                     end if;
                  end if;
               end;
         end case;
         return F;
      end Statement_Flow;

      function Block_Flow (B : Block_Id; Input : Last_Endings; Inside_Try : Boolean) return Flow is
         F : Flow := (Normal => Input, others => <>);
      begin
         for Id of P.Blocks (B) loop
            exit when Is_Empty (F.Normal);
            declare
               G : constant Flow := Statement_Flow (P.Statements (Id), F.Normal, Inside_Try);
            begin
               F := (Normal => G.Normal, Escapes => F.Escapes or G.Escapes, Finished => F.Finished or else G.Finished);
            end;
         end loop;
         return F;
      end Block_Flow;

      Ignored : Flow;
   begin
      --  Two behaviours under one name would make run ambiguous.
      for I in P.Statements.First_Index .. P.Statements.Last_Index loop
         if P.Statements (I).Kind = Define
           and then Definition_Of (To_String (P.Statements (I).Behaviour)) /= Natural (I)
         then
            return Refused (P.Statements (I).Line, "another behaviour is already called """
                            & To_String (P.Statements (I).Behaviour) & """", "give each behaviour its own name");
         end if;
      end loop;
      loop
         Changed := False;
         Ignored := Block_Flow (Top, Before, Inside_Try => False);
         exit when not Changed;
      end loop;
      Final := True;
      Ignored := Block_Flow (Top, Before, Inside_Try => False);
      return Error;
   end Check;

end Driver.Brain.Termination;
