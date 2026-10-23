with Ada.Characters.Handling;
with Ada.Strings.Fixed;
with Ada.Text_IO;
with Ada.Task_Attributes;
with GNAT.Traceback.Symbolic;

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

   --  The beat of the last robot, action or brain line, the longest stretch without one so far, and the silence at
   --  which the decider's place is next told (Check_Silence).
   protected Silence is
      procedure Said (Beat : Natural);
      procedure Due (Beat : Natural; Tell : out Boolean; Quiet : out Natural);
   private
      Last_Said : Natural := 0;
      Longest   : Natural := 0;
      Tell_At   : Natural := 0;
   end Silence;

   protected body Silence is
      procedure Said (Beat : Natural) is
      begin
         if Beat > Last_Said then
            Longest := Natural'Max (Longest, Beat - Last_Said);
         end if;
         Last_Said := Natural'Max (Last_Said, Beat);
         Tell_At := 2 * Natural'Max (1, Longest);
      end Said;

      procedure Due (Beat : Natural; Tell : out Boolean; Quiet : out Natural) is
      begin
         Quiet := (if Beat > Last_Said then Beat - Last_Said else 0);
         Tell := Tell_At > 0 and then Quiet >= Tell_At;
         if Tell then
            Tell_At := 2 * Quiet;
         end if;
      end Due;
   end Silence;

   --  Whether the calling task decides: it takes beats (Driver.Beats.Next calls Check_Silence). Only its own lines end
   --  a silence: the estimates' lines, written apart from the main loop, ended k204se's twice while its decider wrote
   --  nothing for 13 000 beats (10-23).
   package Deciding is new Ada.Task_Attributes (Boolean, False);

   procedure Line (T : Topic; Text : String) is
      Head : constant String := "[" & Ada.Characters.Handling.To_Lower (Topic'Image (T)) & "] " & Text;
   begin
      if T in Robot | Action | Brain and then Deciding.Value then
         Silence.Said (Stamp_Beat);
      end if;
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

   procedure Check_Silence is
      Tell  : Boolean;
      Quiet : Natural;
   begin
      Deciding.Set_Value (True);
      Silence.Due (Stamp_Beat, Tell, Quiet);
      if Tell then
         declare
            Chain : GNAT.Traceback.Tracebacks_Array (1 .. 64);
            Count : Natural;
         begin
            GNAT.Traceback.Call_Chain (Chain, Count);
            declare
               Where : String := GNAT.Traceback.Symbolic.Symbolic_Traceback (Chain (1 .. Count));
            begin
               for C of Where loop
                  if C = ASCII.LF then
                     C := '|';
                  end if;
               end loop;
               Line (Core, "the decider has written nothing for" & Quiet'Image & " beats; it waits at: " & Where);
            end;
         end;
      end if;
   end Check_Silence;

   function Image (N : Integer) return String is
     (Ada.Strings.Fixed.Trim (Integer'Image (N), Ada.Strings.Both));

end Driver.Log;
