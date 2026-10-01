with Ada.Strings.Unbounded;
with Driver.Brain.Words;

package body Driver.Brain.Wants is

   use Ada.Strings.Unbounded;
   use Driver.Brain.Programs;
   use type Driver.Action.Ending;
   use type Driver.Brain.Names.Binding_Kind;

   function Find (Bound : Binding_Maps.Map; Name : String) return Driver.Brain.Names.Binding is
   begin
      if Bound.Contains (Name) then
         return Bound.Element (Name);
      end if;
      for C in Bound.Iterate loop
         if Driver.Brain.Names.Same_Name (Binding_Maps.Key (C), Name) then
            return Binding_Maps.Element (C);
         end if;
      end loop;
      return (others => <>);
   end Find;

   function Known (Bound : Binding_Maps.Map; Name : String) return Boolean is
     (Bound.Contains (Name) or else (for some C in Bound.Iterate => Driver.Brain.Names.Same_Name (Binding_Maps.Key (C),
                                                                                                    Name)));

   function Remembers (P : Program; Name : String) return Boolean is
     (for some S of P.Statements => S.Kind = Remember and then Driver.Brain.Names.Same_Name (To_String (S.Place), Name));

   function Names_Of (P : Program) return Driver.Brain.Keyboard.Word_Vectors.Vector is
      R : Driver.Brain.Keyboard.Word_Vectors.Vector;

      procedure Add (N : Noun) is
      begin
         if N.Kind = Name_Noun and then not Remembers (P, To_String (N.Name)) and then not R.Contains (To_String (N.Name))
         then
            R.Append (To_String (N.Name));
         end if;
      end Add;
   begin
      for S of P.Statements loop
         case S.Kind is
            when Interval =>
               for C of S.Constraints loop
                  Add (C.Subject);
                  Add (C.Object);
               end loop;
            when Remember =>
               Add (S.Who);
            when others =>
               null;
         end case;
      end loop;
      return R;
   end Names_Of;

   procedure Build
     (S          : Statement;
      Bound      : Binding_Maps.Map;
      Quantities : Driver.Brain.Keyboard.Word_Vectors.Vector;
      W          : out Driver.Action.Want;
      Result     : out Build_Result;
      Why        : out Refusal)
   is
      Endings : constant Driver.Action.Ending_Set := [for E in Driver.Action.Ending => E = S.Until_Ending];

      procedure Refuse (Reason, Instead : String) is
      begin
         if Result /= Refused then
            Result := Refused;
            Why := Refused (S.Line, Reason, Instead);
         end if;
      end Refuse;

      function Operand_Of (N : Noun) return Driver.Action.Operand is
      begin
         case N.Kind is
            when Nothing =>
               return (Kind => Driver.Action.Nothing);
            when Role_Noun =>
               return (Kind => Driver.Action.Role_Operand, The_Role => N.Role);
            when Name_Noun =>
               if not Known (Bound, To_String (N.Name)) then
                  if Result = Built then
                     Result := Later;
                  end if;
                  return (Kind => Driver.Action.Nothing);
               end if;
               declare
                  B : constant Driver.Brain.Names.Binding := Find (Bound, To_String (N.Name));
               begin
                  case B.Kind is
                     when Driver.Brain.Names.To_Thing =>
                        return (Kind => Driver.Action.Thing_Operand, Thing => B.Thing);
                     when Driver.Brain.Names.To_Place =>
                        return (Kind => Driver.Action.Place_Operand, Place => B.Place);
                     when Driver.Brain.Names.Unbound =>
                        Refuse ("I cannot tell what """ & To_String (N.Name) & """ is: " & To_String (B.Account),
                                "name it by what you see, in other words");
                        return (Kind => Driver.Action.Nothing);
                  end case;
               end;
         end case;
      end Operand_Of;
   begin
      Result := Built;
      Why := (others => <>);
      W := (Kind => Driver.Action.Interval, others => <>);
      --  The parser makes a quantity change the stretch's only constraint.
      if S.Constraints.First_Element.Kind = Quantity_Constraint then
         declare
            C : constant Programs.Constraint := S.Constraints.First_Element;
            Q : Natural := 0;
         begin
            for I in Quantities.First_Index .. Quantities.Last_Index loop
               if Quantities (I) = To_String (C.Quantity) then
                  Q := I;
               end if;
            end loop;
            if Q = 0 then
               declare
                  Listed : Unbounded_String;
               begin
                  for Word of Quantities loop
                     Append (Listed, (if Length (Listed) > 0 then ", " else "") & Word);
                  end loop;
                  Refuse ("""" & To_String (C.Quantity) & """ is not a quantity I can measure and change now",
                          (if Length (Listed) = 0 then "none can be changed this round"
                           else "the quantities this round: " & To_String (Listed)));
                  return;
               end;
            end if;
            if C.Subject.Kind /= Name_Noun then
               Refuse (To_String (C.Quantity) & " " & (if C.Increase then Driver.Brain.Words.Up_Word
                                                         else Driver.Brain.Words.Down_Word)
                       & " is said of a thing you see, not of a part of me",
                       "write the name of the thing before " & To_String (C.Quantity));
               return;
            end if;
            declare
               O : constant Driver.Action.Operand := Operand_Of (C.Subject);
            begin
               case O.Kind is
                  when Driver.Action.Thing_Operand =>
                     W := (Kind          => Driver.Action.Change,
                           Thing         => O.Thing,
                           Quantity      => Q,
                           Increase      => C.Increase,
                           Until_Endings => Endings,
                           Max_Steps     => S.Max_Steps,
                           Eye           => S.Eye,
                           Anyway        => S.Anyway);
                  when Driver.Action.Place_Operand =>
                     Refuse ("""" & To_String (C.Subject.Name) & """ is a place you had me remember, and a place has no "
                             & To_String (C.Quantity), "");
                  when others =>
                     null;   --  refused or later, as Operand_Of said
               end case;
               if Result = Later then
                  Refuse ("""" & To_String (C.Subject.Name) & """ is a place this program remembers, and a place has no "
                          & To_String (C.Quantity), "");
               end if;
            end;
         end;
         return;
      end if;
      W.Until_Endings := Endings;
      W.Max_Steps := S.Max_Steps;
      W.Eye := S.Eye;
      W.Anyway := S.Anyway;
      for C of S.Constraints loop
         W.Constraints.Append (Driver.Action.Constraint'(Subject  => Operand_Of (C.Subject),
                                                         Relation => C.Relation,
                                                         Object   => Operand_Of (C.Object),
                                                         Step     => C.Step,
                                                         Strength => C.Strength,
                                                         Must     => C.Must));
      end loop;
   end Build;

end Driver.Brain.Wants;
