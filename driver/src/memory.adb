with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
package body Memory is
   procedure Set (M : in out Store; Name, Value : String) is
   begin
      for I in 0 .. Natural (M.Names.Length) - 1 loop
         if M.Names (I) = Name then
            M.Values.Replace_Element (I, Value);
            return;
         end if;
      end loop;
      M.Names.Append (Name);
      M.Values.Append (Value);
   end Set;

   function Get (M : Store; Name : String) return String is
   begin
      for I in 0 .. Natural (M.Names.Length) - 1 loop
         if M.Names (I) = Name then
            return M.Values (I);
         end if;
      end loop;
      return "";
   end Get;

   function Text (M : Store) return String is
      R : Unbounded_String;
   begin
      for I in 0 .. Natural (M.Names.Length) - 1 loop
         if M.Values (I) /= "" then
            Append (R, "- " & M.Names (I) & ": " & M.Values (I) & ASCII.LF);
         end if;
      end loop;
      return To_String (R);
   end Text;

   procedure Clear (M : in out Store) is
   begin
      M.Names.Clear;
      M.Values.Clear;
   end Clear;
end Memory;
