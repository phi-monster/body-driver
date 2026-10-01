package body Driver.Brain.Names is

   use type Driver.World.Thing_Id;
   use type Driver.Observations.Camera_Id;

   function Letters (W : String) return String is
      R : String (1 .. W'Length);
      N : Natural := 0;
   begin
      for C of W loop
         if C in 'a' .. 'z' then
            N := N + 1;
            R (N) := C;
         elsif C in 'A' .. 'Z' then
            N := N + 1;
            R (N) := Character'Val (Character'Pos (C) - Character'Pos ('A') + Character'Pos ('a'));
         end if;
      end loop;
      return R (1 .. N);
   end Letters;

   function Same_Name (A, B : String) return Boolean is
      La : constant String := Letters (A);
   begin
      return La'Length > 0 and then La = Letters (B);
   end Same_Name;

   --  Cut (K) is True when the first K letters of L (from the end: the last
   --  K) can be cut entirely into words of Glue; the empty run always can.
   type Cut_Array is array (Natural range <>) of Boolean;

   function Cuts (L : String; Glue : Word_List; From_End : Boolean) return Cut_Array is
      Cut : Cut_Array (0 .. L'Length) := [0 => True, others => False];
   begin
      for K in 1 .. L'Length loop
         for W of Glue loop
            if not Cut (K) and then W'Length in 1 .. K and then Cut (K - W'Length) then
               declare
                  Start : constant Positive := (if From_End then L'Last - K + 1 else L'First + K - W'Length);
               begin
                  Cut (K) := L (Start .. Start + W'Length - 1) = W;
               end;
            end if;
         end loop;
      end loop;
      return Cut;
   end Cuts;

   function Has_Core (W : String; Glue : Word_List) return Boolean is
      L : constant String := Letters (W);
   begin
      return L'Length > 0 and then not Cuts (L, Glue, From_End => False) (L'Length);
   end Has_Core;

   function Same_Core (A, B : String; Glue : Word_List) return Boolean is
      La : constant String := Letters (A);
      Lb : constant String := Letters (B);
      Pa : constant Cut_Array := Cuts (La, Glue, From_End => False);
      Sa : constant Cut_Array := Cuts (La, Glue, From_End => True);
      Pb : constant Cut_Array := Cuts (Lb, Glue, From_End => False);
      Sb : constant Cut_Array := Cuts (Lb, Glue, From_End => True);
   begin
      if not Has_Core (A, Glue) or else not Has_Core (B, Glue) then
         return False;
      end if;
      --  A's core: what is left after I glued letters in front and J behind;
      --  B must hold the same core with glue on either side of it.
      for I in 0 .. La'Length - 1 loop
         if Pa (I) then
            for J in 0 .. La'Length - I - 1 loop
               if Sa (J) then
                  declare
                     Core : constant String := La (La'First + I .. La'Last - J);
                  begin
                     for P in 0 .. Lb'Length - Core'Length loop
                        if Pb (P) and then Sb (Lb'Length - P - Core'Length)
                          and then Lb (Lb'First + P .. Lb'First + P + Core'Length - 1) = Core
                        then
                           return True;
                        end if;
                     end loop;
                  end;
               end if;
            end loop;
         end if;
      end loop;
      return False;
   end Same_Core;

   procedure Name_Place (N : in out Table; Name : String; P : Place_Id) is
   begin
      for I in N.Places.First_Index .. N.Places.Last_Index loop
         if Same_Name (To_String (N.Places (I).Name), Name) then
            N.Places.Replace_Element (I, (Name => To_Unbounded_String (Name), Place => P));
            return;
         end if;
      end loop;
      N.Places.Append (Place_Name'(Name => To_Unbounded_String (Name), Place => P));
   end Name_Place;

   function Name_Of (N : Table; T : Thing_Id) return String is
   begin
      for E of N.Things loop
         if E.Thing = T then
            return To_String (E.Name);
         end if;
      end loop;
      return "";
   end Name_Of;

   function Named_Count (N : Table) return Natural is (Natural (N.Things.Length));

   function Named_Thing (N : Table; Index : Positive) return Thing_Id is (N.Things (Index).Thing);

   --  From now on T goes by Name.
   procedure Call (N : in out Table; T : Thing_Id; Name : String) is
   begin
      for I in N.Things.First_Index .. N.Things.Last_Index loop
         if N.Things (I).Thing = T then
            N.Things.Replace_Element (I, (Name => To_Unbounded_String (Name), Thing => T));
            return;
         end if;
      end loop;
      N.Things.Append (Named'(Name => To_Unbounded_String (Name), Thing => T));
   end Call;

   function Known_Names (N : Table) return String is
      R : Unbounded_String;
   begin
      for E of N.Things loop
         Append (R, (if Length (R) > 0 then ", " else "") & """" & To_String (E.Name) & """");
      end loop;
      return (if Length (R) = 0 then "you have named nothing yet" else "the names you gave: " & To_String (R));
   end Known_Names;

   --  Steps 3 and 4: the letters alone, against every name given so far.
   procedure By_Letters (N : Table; Glue : Word_List; Name : String; Result : in out Binding; Asked : String) is
      Picks : Named_Vectors.Vector;
   begin
      for Exact in reverse Boolean loop
         exit when not Picks.Is_Empty;
         for E of N.Things loop
            if (if Exact then Same_Name (To_String (E.Name), Name) else Same_Core (Name, To_String (E.Name), Glue))
            then
               Picks.Append (E);
            end if;
         end loop;
      end loop;
      if Picks.Is_Empty then
         Result := (Kind => Unbound, Account => To_Unbounded_String
                      (Asked & (if Asked'Length > 0 then "; " else "") & "by its letters """ & Name
                       & """ is none of the things you named (" & Known_Names (N) & ")"), others => <>);
      elsif Picks.First_Index = Picks.Last_Index then
         Result := (Kind => To_Thing, Thing => Picks.First_Element.Thing, Account => To_Unbounded_String
                      (Asked & (if Asked'Length > 0 then "; " else "") & "by its letters it is what you called """
                       & To_String (Picks.First_Element.Name) & """"), others => <>);
      else
         declare
            Listed : Unbounded_String;
         begin
            for E of Picks loop
               Append (Listed, (if Length (Listed) > 0 then " and " else "") & """" & To_String (E.Name) & """");
            end loop;
            Result := (Kind => Unbound, Account => To_Unbounded_String
                         (Asked & (if Asked'Length > 0 then "; " else "") & "by its letters it could be " & To_String (Listed)
                          & ", and I do not guess which"), others => <>);
         end;
      end if;
   end By_Letters;

   procedure Bind
     (N      : in out Table;
      S      : in out Senses'Class;
      View   : Eye_Id;
      Name   : String;
      Glue   : Word_List;
      Result : out Binding)
   is
      Asked : Unbounded_String;

      procedure Said (What : String) is
      begin
         Append (Asked, (if Length (Asked) > 0 then "; " else "") & What);
      end Said;

      function Eye (E : Eye_Id) return String is ("eye" & Eye_Id'Image (E));
   begin
      Result := (others => <>);
      for P of N.Places loop
         if Same_Name (To_String (P.Name), Name) then
            Result := (Kind => To_Place, Place => P.Place, Account => To_Unbounded_String
                         ("the place you had me remember as """ & To_String (P.Name) & """"), others => <>);
            return;
         end if;
      end loop;
      if not Has_Core (Name, Glue) then
         Result := (Kind => Unbound, Account => To_Unbounded_String
                      ("""" & Name & """ is made only of words of the language, so it names nothing"), others => <>);
         return;
      end if;
      for E of N.Things loop
         if Same_Name (To_String (E.Name), Name) and then S.Sees (E.Thing, View) then
            Result := (Kind => To_Thing, Thing => E.Thing, Account => To_Unbounded_String
                         ("what you named before; " & Eye (View) & " sees it"), others => <>);
            return;
         end if;
      end loop;
      declare
         Order : Eye_Vectors.Vector;
      begin
         Order.Append (View);
         for E of S.Eyes loop
            if E /= View then
               Order.Append (E);
            end if;
         end loop;
         for E of Order loop
            declare
               Answer : Pointing;
               Where  : Box;
               Why    : Unbounded_String;
            begin
               S.Ask_Where (E, Name, Answer, Where, Why);
               case Answer is
                  when No_Answer =>
                     Said ("I could not ask " & Eye (E) & " where it is (" & To_String (Why) & ")");
                     exit;
                  when Not_Here =>
                     Said (Eye (E) & " cannot point it out");
                  when Boxed =>
                     declare
                        Found : Patch;
                        T     : Thing_Id;
                        What  : Unbounded_String;
                     begin
                        S.Identify (E, Where, Found, T, What);
                        case Found is
                           when A_Thing =>
                              declare
                                 Before : constant String := Name_Of (N, T);
                              begin
                                 Call (N, T, Name);
                                 Result := (Kind => To_Thing, Thing => T, Account => To_Unbounded_String
                                              (Eye (E) & " boxed it"
                                               & (if Before'Length > 0 and then Before /= Name
                                                  then ", on the same pixels as what you called """ & Before
                                                       & """, which now goes by your new words"
                                                  else "")), others => <>);
                                 return;
                              end;
                           when Part_Of_Me =>
                              Said (Eye (E) & " boxed a part of me" & (if Length (What) > 0 then " (" & To_String (What)
                                                                         & ")" else ""));
                           when No_Patch =>
                              Said ("in the box " & Eye (E) & " gave, nothing stands apart from its surroundings");
                        end case;
                     end;
               end case;
            end;
         end loop;
      end;
      By_Letters (N, Glue, Name, Result, To_String (Asked));
   end Bind;

   procedure Bind_Again (N : Table; Glue : Word_List; Name : String; Result : in out Binding) is
   begin
      if Result.Kind = Unbound and then Has_Core (Name, Glue) then
         declare
            Was : constant String := To_String (Result.Account);
            Now : Binding := Result;
         begin
            By_Letters (N, Glue, Name, Now, "");
            if Now.Kind = To_Thing then
               Result := (Kind => To_Thing, Thing => Now.Thing, Account => To_Unbounded_String
                            (Was & "; once the whole program was read, " & To_String (Now.Account)), others => <>);
            end if;
         end;
      end if;
   end Bind_Again;

end Driver.Brain.Names;
