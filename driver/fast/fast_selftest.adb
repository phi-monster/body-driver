--  fast_selftest.adb -- the fast face's conformance run.
--
--  🔴 THE RULE THIS FILE EXISTS TO ENFORCE:
--      a guard that has never failed has never been tested, and a test that has never failed is
--      indistinguishable in the output from a test that does not exist.
--
--  So every case below is a case the guard MUST refuse.  If any of them is admitted, this program
--  exits non-zero and the build is broken.  There is deliberately no "expected pass" case that
--  could carry the whole suite on its own.
--
--  This project has paid for this rule repeatedly: a two-state occlusion control whose second
--  clause made "0 > 0" report PASS; a docstring promising a --ref positive control that the
--  argument parser did not implement; a watchdog whose counter was broken so its "no new episodes"
--  predicate was permanently true and it deleted a healthy leg's 15 episodes.  In each case the
--  log said the guard was fine.

pragma SPARK_Mode (Off);   --  the harness does I/O; the package under test stays SPARK_Mode On

with Ada.Command_Line;
with Ada.Text_IO; use Ada.Text_IO;
with Body_Layer_Fast; use Body_Layer_Fast;

procedure Fast_Selftest is

   Failures : Natural := 0;

   procedure Check (Name : String; Condition : Boolean);
   function Envelope return Limit_Array;

   procedure Check (Name : String; Condition : Boolean) is
   begin
      if Condition then
         Put_Line ("  ok   " & Name);
      else
         Put_Line ("  FAIL " & Name);
         Failures := Failures + 1;
      end if;
   end Check;

   function Envelope return Limit_Array is
      L : constant Limit_Array := [others => (Lo => -1.0, Hi => 1.0)];
   begin
      return L;
   end Envelope;

   Good : constant Joint_Array := [others => 0.5];
   Bad  : Joint_Array := [others => 0.5];

begin
   Put_Line ("[fast_selftest] every case below MUST be refused");

   ------------------------------------------------------------------ 1
   declare
      S   : State := Initial;
      Out_C : Joint_Array;
      Ok  : Boolean;
   begin
      --  A fresh state must refuse.  The default has to be "refuse", because a layer whose
      --  default is "permit" permits whenever somebody forgets to configure it.
      Admit (S, Good, 0.0, 10, Out_C, Ok);
      Check ("fresh state refuses", not Ok and then Reason (S) = Not_Installed);
   end;

   ------------------------------------------------------------------ 2
   declare
      S     : State := Initial;
      Out_C : Joint_Array;
      Ok    : Boolean;
   begin
      Install_Limits (S, Envelope, 6, Good, 20.0, 100, 1_000);
      Bad := Good;
      Bad (3) := 1.5;   --  outside the installed envelope
      Admit (S, Bad, 0.0, 1_010, Out_C, Ok);
      Check ("out-of-range halts (does NOT clamp)",
             not Ok and then Reason (S) = Limit_Violation);
      --  and the safe hold must be INSIDE the envelope, not zeros-as-a-move
      Check ("hold stays inside envelope",
             Out_C (3) >= Lo_Of (S, 3) and then Out_C (3) <= Hi_Of (S, 3));
   end;

   ------------------------------------------------------------------ 3
   declare
      S     : State := Initial;
      Out_C : Joint_Array;
      Ok    : Boolean;
   begin
      Install_Limits (S, Envelope, 6, Good, 20.0, 100, 1_000);
      Admit (S, Good, 25.0, 1_010, Out_C, Ok);   --  above the installed cap
      Check ("force above cap halts", not Ok and then Reason (S) = Force_Exceeded);
   end;

   ------------------------------------------------------------------ 4
   declare
      S     : State := Initial;
      Out_C : Joint_Array;
      Ok    : Boolean;
   begin
      Install_Limits (S, Envelope, 6, Good, 20.0, 100, 1_000);
      Tick (S, 1_500);   --  deadline was 100 ms; 500 ms of silence
      Check ("watchdog expires on silence", Halted (S) and then Reason (S) = Watchdog_Expired);
      --  and a perfectly good command must NOT walk it out of the halt
      Admit (S, Good, 0.0, 1_510, Out_C, Ok);
      Check ("good command does not clear a latch", not Ok and then Halted (S));
   end;

   ------------------------------------------------------------------ 5
   declare
      S  : State := Initial;
      Ok : Boolean;
   begin
      Install_Limits (S, Envelope, 6, Good, 20.0, 100, 1_000);
      Stop (S);
      Clear (S, Watchdog_Expired, 1_100, Ok);   --  wrong witness
      Check ("clear with wrong witness refuses", not Ok and then Halted (S));
      Clear (S, External_Stop, 1_100, Ok);      --  right witness
      Check ("clear with right witness releases", Ok and then not Halted (S));
   end;

   ------------------------------------------------------------------ 6
   declare
      S     : State := Initial;
      Out_C : Joint_Array;
      Ok    : Boolean;
   begin
      --  The one admitted case, kept LAST and kept small: without it a suite that refuses
      --  everything would also pass, and "refuses everything" is not a body layer either.
      Install_Limits (S, Envelope, 6, Good, 20.0, 100, 1_000);
      Admit (S, Good, 1.0, 1_050, Out_C, Ok);
      Check ("an in-envelope command IS admitted", Ok and then Out_C (1) = Good (1));
   end;

   New_Line;
   if Failures = 0 then
      Put_Line ("[fast_selftest] PASS -- every guard fired on the input that must make it fire");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Success);
   else
      Put_Line ("[fast_selftest] FAIL --" & Natural'Image (Failures) & " guard(s) did not fire");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   end if;
end Fast_Selftest;
