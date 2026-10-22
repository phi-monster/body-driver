--  The workers' cost a call: Parallel_For over INDICES indices, CALLS times, each index WORK steps of a multiply and
--  an add; prints the mean wall time of a call in microseconds, and the same work done on the caller alone. A call's
--  cost less its work's share of the processors is what the pool takes to hand the work out and gather it back.
--
--    pool_bench INDICES CALLS WORK      (BD_WORKERS sets the workers, as for the driver)

with Ada.Calendar;
with Ada.Command_Line;
with Ada.Text_IO;
with Driver;
with Driver.Parallel_For;

procedure Pool_Bench is
   use type Ada.Calendar.Time;
   subtype Real is Driver.Real;
   Indices : constant Positive := Positive'Value (Ada.Command_Line.Argument (1));
   Calls   : constant Positive := Positive'Value (Ada.Command_Line.Argument (2));
   Steps   : constant Natural := Natural'Value (Ada.Command_Line.Argument (3));
   type Sink_Array is array (1 .. Indices) of Real;
   Sink : Sink_Array := [others => 0.0];

   procedure Work (I : Positive) is
      S : Real := Real (I);
   begin
      for K in 1 .. Steps loop
         S := S * 0.5 + Real (K);
      end loop;
      Sink (I) := S;
   end Work;
   procedure Every is new Driver.Parallel_For (Work);

   Start : Ada.Calendar.Time := Ada.Calendar.Clock;
   Pooled, Alone : Duration;
begin
   for C in 1 .. Calls loop
      Every (1, Indices);
   end loop;
   Pooled := Ada.Calendar.Clock - Start;
   Start := Ada.Calendar.Clock;
   for C in 1 .. Calls loop
      for I in 1 .. Indices loop
         Work (I);
      end loop;
   end loop;
   Alone := Ada.Calendar.Clock - Start;
   Ada.Text_IO.Put_Line
     ("indices" & Indices'Image & ", work" & Steps'Image & " steps an index: a call"
      & Integer'Image (Integer (Real (Pooled) * 1.0e6 / Real (Calls))) & " us pooled,"
      & Integer'Image (Integer (Real (Alone) * 1.0e6 / Real (Calls))) & " us alone; checksum"
      & Integer'Image (Integer (Sink (Indices)) mod 1000));
end Pool_Bench;
