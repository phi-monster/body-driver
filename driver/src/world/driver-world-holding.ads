--  A thing a hand holds goes where the hand's arm goes. The hand closed on it
--  and its closer stopped short of closed on nothing, as nothing but what is
--  between its lobes stops it; while the closer stays short the thing is
--  carried rigidly with the arm's tool, from where it was when the hand took
--  it. The closer closing on nothing says the thing has left the hand, and its
--  opening that the hand let it go: either ends the hold, and the thing stays
--  where it was carried to last. Nothing here asks what the thing is or how the
--  hand is made: a closer's fraction and a tool's pose.
--
--  A body that sees a thing only from the arm that takes it (the kiwi's wrist
--  eyes, 10-15) knows so where the thing is once it is lifted: those eyes placed
--  its points from the arm's sweep while it lay still (Driver.Robot.Surfaces),
--  and no eye places it again where it goes.

with Driver.World.Estimates;
with Driver.World.Points;

private package Driver.World.Holding is

   function Still_Held (At_Hold, Now : Estimate) return Boolean;
   --  A closer that stood at At_Hold when it took the thing (a fraction, 0
   --  open and 1 closed on nothing) still holds it at Now: short of closed by
   --  more than its noise tells from closed, and not opened from where it held
   --  by more than the noise of the two tells from staying (a squeeze that
   --  closes it further holds on). False when either is unknown.

   procedure Carry
     (Points   : in out Driver.World.Points.Placed_Vectors.Vector;
      Normals  : in out Driver.World.Estimates.Normal_Vectors.Vector;
      From, To : Pose_Estimate);
   --  Each point and normal moved as the tool moved from From to To: means
   --  turned and shifted, covariances turned, and each point's covariance
   --  grown by the two tool poses' own, their positions and their turns at the
   --  point's lever from the tool, since the hand's measured motion carries it
   --  no better than it is measured.

end Driver.World.Holding;
