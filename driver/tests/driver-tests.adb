with Ada.Containers.Indefinite_Vectors;
with Ada.Exceptions;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Log;

package body Driver.Tests is

   use Ada.Strings.Unbounded;

   type Entry_Type is record
      Name, Guards : Unbounded_String;
      Run          : Procedure_Access;
   end record;

   package Entry_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, Entry_Type);

   Registry : Entry_Vectors.Vector;
   Failures : Natural := 0;
   First_Failure : Unbounded_String;

   procedure Register (Name : String; Guards : String; Run : not null Procedure_Access) is
   begin
      Registry.Append (Entry_Type'(To_Unbounded_String (Name), To_Unbounded_String (Guards), Run));
   end Register;

   procedure Check (Condition : Boolean; What : String) is
   begin
      if not Condition then
         if Failures = 0 then
            First_Failure := To_Unbounded_String (What);
         end if;
         Failures := Failures + 1;
      end if;
   end Check;

   procedure Check_Close (Actual, Expected, Tolerance : Real; What : String) is
   begin
      Check (abs (Actual - Expected) <= Tolerance,
             What & ": got " & Driver.Log.Image (Actual, 9) & ", expected " & Driver.Log.Image (Expected, 9)
             & " within " & Driver.Log.Image (Tolerance, 9));
   end Check_Close;

   function Run_All (Filter : String := "") return Natural is
      Failed : Natural := 0;
      Passed : Natural := 0;
   begin
      for E of Registry loop
         declare
            Name : constant String := To_String (E.Name);
         begin
            if Name'Length >= Filter'Length and then Name (Name'First .. Name'First + Filter'Length - 1) = Filter then
               Failures := 0;
               First_Failure := Null_Unbounded_String;
               begin
                  E.Run.all;
               exception
                  when X : others =>
                     Check (False, "raised " & Ada.Exceptions.Exception_Information (X));
               end;
               if Failures = 0 then
                  Passed := Passed + 1;
                  Ada.Text_IO.Put_Line ("pass  " & Name);
               else
                  Failed := Failed + 1;
                  Ada.Text_IO.Put_Line ("FAIL  " & Name & "  (guards: " & To_String (E.Guards) & ")");
                  Ada.Text_IO.Put_Line ("      " & To_String (First_Failure)
                                        & (if Failures > 1 then "  (+" & Driver.Log.Image (Failures - 1) & " more)"
                                           else ""));
               end if;
            end if;
         end;
      end loop;
      Ada.Text_IO.Put_Line (Driver.Log.Image (Passed) & " passed, " & Driver.Log.Image (Failed) & " failed");
      return Failed;
   end Run_All;

end Driver.Tests;
