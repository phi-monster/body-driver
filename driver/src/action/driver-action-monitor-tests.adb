with Driver.Tests;

package body Driver.Action.Monitor.Tests is

   use Driver.Tests;

   None : constant Ending_Set := [others => False];

   function Only (E : Ending) return Ending_Set is
      S : Ending_Set := None;
   begin
      S (E) := True;
      return S;
   end Only;

   function Ends (W : Watch; F : Facts; Wanted : Ending_Set; Max_Steps : Natural := 0) return String is
     (if Fired (W, F, Wanted, Max_Steps) then Ending'Image (Ending_Of (W, F, Wanted, Max_Steps)) else "GOES ON");

   procedure Settled_Only_At_Rest is
      W : Watch := Start;
      Moving_Still : constant Facts := (Commanded => True, Still => True, others => <>);
      At_Rest      : constant Facts := (Commanded => False, Still => True, others => <>);
      Changing     : constant Facts := (Commanded => False, Still => False, others => <>);
   begin
      Step (W, Moving_Still);
      Check (Ends (W, Moving_Still, Only (Settled)) = "GOES ON",
             "a still picture while the body is told to move is called settled");
      Step (W, Changing);
      Check (Ends (W, Changing, Only (Settled)) = "GOES ON", "a changing scene is called settled");
      Step (W, At_Rest);
      Check (Ends (W, At_Rest, Only (Settled)) = "SETTLED", "the scene at rest with nothing commanded is not settled");
   end Settled_Only_At_Rest;

   procedure Stuck_Ends_Unasked is
      W : Watch := Start;
      F : constant Facts := (Commanded => True, Blocked => True, others => <>);
   begin
      Step (W, F);
      Check (Ends (W, F, Only (Touched)) = "STUCK", "a blocked body goes on toward a touch it cannot make");
      Check (Ends (W, F, Only (Stuck)) = "STUCK", "a wanted stuck is not reported");
   end Stuck_Ends_Unasked;

   procedure Wanted_Wins is
      W : Watch := Start;
      F : constant Facts := (Commanded => True, Blocked => True, Touch => True, others => <>);
      Both : Ending_Set := None;
   begin
      Step (W, F);
      Check (Ends (W, F, Only (Touched)) = "TOUCHED", "a wanted touch is reported as the block that came with it");
      Both (Touched) := True;
      Both (Stuck) := True;
      Check (Ends (W, F, Both) = "TOUCHED", "of two wanted endings the later in order is reported");
      Check (Ends (W, (F with delta Blocked => False), Only (Settled)) = "GOES ON",
             "an unwanted touch that does not block ends the interval");
   end Wanted_Wins;

   --  A gap of sigma 0.01 at every look; each step delivers motion owing
   --  Owed of closing.
   procedure Run_Gap (Closing, Owed : Real; Steps_Run : Natural; W : out Watch; Last : out Facts) is
      Gap : Real := 1.0;
   begin
      W := Start;
      Last := (others => <>);
      for I in 1 .. Steps_Run loop
         Last := (Commanded => True, Gap => (Value => Gap, Sigma => 0.01, Degrees_Of_Freedom => 0), Owed => Owed,
                  others => <>);
         Step (W, Last);
         exit when Fired (W, Last, Only (Touched), 0);
         Gap := Gap - Closing;
      end loop;
   end Run_Gap;

   procedure Stalled_By_Owed_Progress is
      W : Watch;
      F : Facts;
   begin
      --  Moving but the gap does not shrink: owed 0.01 a step against a
      --  difference sigma of 0.014, so a few steps owe significantly.
      Run_Gap (0.0, 0.01, 40, W, F);
      Check (Ends (W, F, Only (Touched)) = "STALLED", "a body moving without closing the gap goes on");
      Check (Steps (W) < 40, "a stall is called only at the step limit");
      --  The gap shrinks at what is owed: never stalled.
      Run_Gap (0.01, 0.01, 40, W, F);
      Check (not Stalled (W), "a body closing the gap as owed is called stalled");
      --  Slower than owed, nearer the owed than none: progress.
      Run_Gap (0.006, 0.01, 40, W, F);
      Check (not Stalled (W), "a body closing the gap at most of the owed rate is called stalled");
      --  Shrinking, but nearer none than the owed: a stall.
      Run_Gap (0.003, 0.01, 40, W, F);
      Check (Stalled (W), "a body closing a small part of what it owes is not called stalled");
      --  Nothing delivered: nothing owed, so no stall (that is stuck's business).
      Run_Gap (0.0, 0.0, 40, W, F);
      Check (not Stalled (W), "a body that is not moving is called stalled");
   end Stalled_By_Owed_Progress;

   procedure Free_When_Height_Rises is
      W : Watch := Start;
      Low  : constant Facts := (Commanded => True, Height_Gain => (Value => 0.002, Sigma => 0.001,
                                Degrees_Of_Freedom => 0), others => <>);
      High : constant Facts := (Low with delta Height_Gain => (Value => 0.004, Sigma => 0.001, Degrees_Of_Freedom => 0));
      Down : constant Facts := (Low with delta Height_Gain => (Value => -0.004, Sigma => 0.001, Degrees_Of_Freedom => 0));
   begin
      Step (W, Low);
      Check (Ends (W, Low, Only (Free)) = "GOES ON", "a rise within its noise is called free");
      Check (Ends (W, High, Only (Free)) = "FREE", "a significant rise is not called free");
      Check (Ends (W, Down, Only (Free)) = "GOES ON", "a thing pushed down is called free");
      Check (Ends (W, High, Only (Settled)) = "GOES ON", "an unwanted free ends the interval");
   end Free_When_Height_Rises;

   procedure Slipped_When_Gone is
      W : Watch := Start;
      Holding : constant Facts := (Commanded => True, Closed_Short => (Value => 0.3, Sigma => 0.01,
                                   Degrees_Of_Freedom => 0), others => <>);
      Shut    : constant Facts := (Holding with delta Closed_Short => (Value => 0.01, Sigma => 0.01,
                                   Degrees_Of_Freedom => 0));
      Behind  : constant Facts := (Holding with delta Left_Behind => (Value => 0.05, Sigma => 0.01,
                                   Degrees_Of_Freedom => 0));
   begin
      Step (W, Holding);
      Check (Ends (W, Holding, Only (Free)) = "GOES ON", "a closer holding something is called slipped");
      Check (Ends (W, Shut, Only (Free)) = "SLIPPED", "a closer closed on nothing goes on carrying");
      Check (Ends (W, Behind, Only (Free)) = "SLIPPED", "a thing left behind by the hand goes on being carried");
   end Slipped_When_Gone;

   procedure Lost_Only_When_Unknown is
      W : Watch := Start;
      Hidden : constant Facts := (Commanded => True, Seen => False, Followable => True, others => <>);
      Gone   : constant Facts := (Hidden with delta Followable => False);
   begin
      Step (W, Hidden);
      Check (Ends (W, Hidden, Only (Lost)) = "LOST", "a wanted lost is not reported when no eye sees it");
      Check (Ends (W, Hidden, Only (Touched)) = "GOES ON", "a thing hidden but still known stops the motion");
      Check (Ends (W, Gone, Only (Touched)) = "LOST", "a thing nobody knows the place of is still followed");
   end Lost_Only_When_Unknown;

   procedure Exhausted_Waits_Or_Sticks is
      W : Watch := Start;
      Done_Moving : constant Facts := (Commanded => False, Exhausted => True, Still => False, others => <>);
      Rested      : constant Facts := (Done_Moving with delta Still => True);
   begin
      Step (W, Done_Moving);
      Check (Ends (W, Done_Moving, Only (Settled)) = "GOES ON", "a body that can do no more stops waiting to settle");
      Check (Ends (W, Rested, Only (Settled)) = "SETTLED", "a body that can do no more never settles");
      Check (Ends (W, Done_Moving, Only (Touched)) = "STUCK",
             "a body that can do no more toward a touch waits for it");
   end Exhausted_Waits_Or_Sticks;

   procedure Timeout_At_The_Limit is
      W : Watch := Start;
      F : constant Facts := (Commanded => True, others => <>);
   begin
      for I in 1 .. 3 loop
         Step (W, F);
      end loop;
      Check (Ends (W, F, Only (Touched), 4) = "GOES ON", "an interval ends before its step limit");
      Step (W, F);
      Check (Ends (W, F, Only (Touched), 4) = "TIMEOUT", "an interval runs past its step limit");
      Check (Ends (W, F, Only (Touched), 0) = "GOES ON", "an interval with no step limit is cut short");
      Check (Ends (W, (F with delta Out_Of_Beats => True), Only (Touched), 0) = "TIMEOUT",
             "an interval goes on after the episode's last beat");
   end Timeout_At_The_Limit;

   procedure Register is
   begin
      Register ("action.monitor.settled", "a body that is told to move, or a scene still changing, is called settled",
                Settled_Only_At_Rest'Access);
      Register ("action.monitor.stuck", "a blocked body goes on pushing until its step limit",
                Stuck_Ends_Unasked'Access);
      Register ("action.monitor.wanted", "a fact nobody waited for hides the wanted ending that came with it",
                Wanted_Wins'Access);
      Register ("action.monitor.stalled", "a stall is called by counting steps instead of by owed progress",
                Stalled_By_Owed_Progress'Access);
      Register ("action.monitor.free", "a thing is called free on a rise within its own noise",
                Free_When_Height_Rises'Access);
      Register ("action.monitor.slipped", "a thing gone from the hand is carried on", Slipped_When_Gone'Access);
      Register ("action.monitor.lost", "a thing hidden for a moment ends the interval", Lost_Only_When_Unknown'Access);
      Register ("action.monitor.exhausted", "a body that can do no more waits forever or is called settled at once",
                Exhausted_Waits_Or_Sticks'Access);
      Register ("action.monitor.timeout", "the step limit is off by one or ignored", Timeout_At_The_Limit'Access);
   end Register;

end Driver.Action.Monitor.Tests;
