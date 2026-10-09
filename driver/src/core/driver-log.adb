with Ada.Characters.Handling;
with Ada.Strings.Fixed;
with Ada.Text_IO;

package body Driver.Log is

   protected Output is
      procedure Write (Text : String);
   end Output;

   protected body Output is
      procedure Write (Text : String) is
      begin
         Ada.Text_IO.Put_Line (Text);
         Ada.Text_IO.Flush;
      end Write;
   end Output;

   Stamped    : Boolean := False with Atomic;
   Stamp_Beat : Natural := 0 with Atomic;

   procedure Stamp (Beat : Natural) is
   begin
      Stamp_Beat := Beat;
      Stamped := True;
   end Stamp;

   procedure Line (T : Topic; Text : String) is
      Head : constant String := "[" & Ada.Characters.Handling.To_Lower (Topic'Image (T)) & "] " & Text;
   begin
      if Stamped then
         Output.Write ("@" & Image (Stamp_Beat) & " " & Head);
      else
         Output.Write (Head);
      end if;
   end Line;

   function Image (X : Real; Digits_After_Point : Natural := 3) return String is
      package IO is new Ada.Text_IO.Float_IO (Real);
      Buffer : String (1 .. 64);
   begin
      if X /= X or else abs X > Real'Last then
         return "nan";
      end if;
      IO.Put (Buffer, X, Aft => Digits_After_Point, Exp => 0);
      return Ada.Strings.Fixed.Trim (Buffer, Ada.Strings.Both);
   exception
      when Ada.Text_IO.Layout_Error =>
         IO.Put (Buffer, X, Aft => Digits_After_Point, Exp => 3);
         return Ada.Strings.Fixed.Trim (Buffer, Ada.Strings.Both);
   end Image;

   function Image (N : Integer) return String is
     (Ada.Strings.Fixed.Trim (Integer'Image (N), Ada.Strings.Both));

end Driver.Log;
