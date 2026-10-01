--  The ending of an interval, judged after every step.
--
--  Constant memory and pure comparisons: every fact of a step is measured,
--  and every quantity goes through the one test of significance against its
--  own measured noise (Driver.Uncertain); nothing here counts steps toward a
--  verdict except the brain's own step limit. An ending fires when what it
--  names is measured to have happened. Arrived is never judged here (only the
--  brain can tell a want is met) and Refused never either (the body refuses
--  before it moves). A wanted ending that holds wins over every other; a
--  fact that makes going on pointless ends the interval even when nobody
--  waited for it, and the brain reads which one it was.

with Driver.Uncertain;

package Driver.Action.Monitor
  with SPARK_Mode
is

   use Driver.Uncertain;

   type Facts is record
      Commanded    : Boolean := False;   --  the body was told to move in this step
      Blocked      : Boolean := False;   --  it stopped short, and pushing further does not move it
      Exhausted    : Boolean := False;   --  the want needs motion and the body can make no more toward it
      Touch        : Boolean := False;   --  it met what it moved toward
      Height_Gain  : Estimate;           --  the thing's height above where it began; Unknown without one
      Left_Behind  : Estimate;           --  how far a carried thing fell behind its hand; Unknown when none is
      Closed_Short : Estimate;           --  how far a holding closer is from closed on nothing; Unknown when none
      Seen         : Boolean := True;    --  the thing it follows is seen by some eye now
      Followable   : Boolean := True;    --  where that thing is is still known (seen, remembered or held)
      Still        : Boolean := False;   --  nothing in body or scene changes against its own noise
      Gap          : Estimate;           --  how far the want still is; Unknown when it has no end point
      Owed         : Real := 0.0;        --  how much the motion delivered in this step should have closed Gap
      Out_Of_Beats : Boolean := False;   --  the episode has no more beats
   end record;

   type Watch is private;
   --  What carries over from one step to the next: the steps taken, and the
   --  gap where progress was last confirmed with what has been owed since.

   function Start return Watch;

   procedure Step (W : in out Watch; F : Facts);
   --  Takes in one step.

   function Steps (W : Watch) return Natural;

   function Stalled (W : Watch) return Boolean;
   --  Since the last confirmed progress the gap closed significantly less
   --  than the delivered motion owed, before it closed significantly at all:
   --  of "closes as owed" and "does not close", the evidence told it from
   --  the first one first.

   function Holds (W : Watch; F : Facts; E : Ending; Max_Steps : Natural) return Boolean;
   --  Whether what the ending names is measured to have happened.

   function Fired (W : Watch; F : Facts; Wanted : Ending_Set; Max_Steps : Natural) return Boolean;
   --  Whether the interval ends after this step: a wanted ending holds, or
   --  going on is pointless (stuck, slipped, lost, stalled, out of steps).

   function Ending_Of (W : Watch; F : Facts; Wanted : Ending_Set; Max_Steps : Natural) return Ending
     with Pre => Fired (W, F, Wanted, Max_Steps);
   --  The first wanted ending that holds, in the order of Ending; otherwise
   --  the first fact, in that order, that made going on pointless. A body
   --  that can make no more progress is stuck, unless waiting for the scene
   --  to settle was wanted: then it waits.

private

   type Watch is record
      Count   : Natural := 0;
      Mark    : Estimate;            --  the gap at the last confirmed progress
      Owed    : Real := 0.0;         --  closing owed since the mark
      Stalled : Boolean := False;
   end record;

end Driver.Action.Monitor;
