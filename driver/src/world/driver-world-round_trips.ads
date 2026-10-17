--  How far a matcher errs, measured on its own round trips: each point it was asked about, matched from one picture
--  into another and back, comes back by a right match's error or, matched wrong, anywhere. The error is measured on
--  the matches themselves, whatever moved between the pictures, since a point the matcher found again comes back
--  whatever moved it (Driver.World.Tracking finds a thing again this way, the driver's aligner the matcher).

package Driver.World.Round_Trips is

   procedure Matcher_Error (Trips : Real_Array; Area : Real; Sigma : out Real; Right : out Real)
     with Pre => Trips'Length mod 2 = 0 and then Area > 0.0;
   --  The matcher's error from round trips (each two coordinates, from where
   --  a pixel was to where its match matched back to). A right match comes
   --  back by a centred Gaussian error; a wrong one lands anywhere in the
   --  image, of that Area. The two are told apart by the mixture's maximum
   --  likelihood (climbed from every doubling rank of the round trips'
   --  lengths until it stops growing), not by a robust
   --  spread of them all: when the second picture does not show most of what
   --  is asked, the wrong ones are most of them, and their spread is no
   --  matcher's error. The fit is then made again within its own gate, the
   --  wrong ones taken as spread over that window, until the window holds
   --  what it held: a matcher lost near what it was asked comes back near
   --  it, and over the whole image those look like a wide Gaussian that
   --  swallows the right ones. Sigma is one coordinate of a right match's
   --  round trip; Right, how many of the round trips the fit takes for right
   --  ones.

private

   procedure Fit_Mixture
     (Squared    : Real_Array;
      Dimensions : Positive;
      Measure    : Real;
      Sigma      : out Real;
      Right      : out Real;
      Passes     : in out Natural);
   --  One fit of that mixture, over the whole range: its maximum likelihood,
   --  climbed from every doubling rank of the errors' lengths.

   function Ball (Dimensions : Positive; Radius : Real) return Real;
   --  The measure of the errors in that many dimensions no longer than Radius.

   procedure Mixture
     (Squared    : Real_Array;
      Dimensions : Positive;
      Measure    : Real;
      Sigma      : out Real;
      Right      : out Real;
      Passes     : in out Natural);
   --  The fit Matcher_Error makes, of errors given by their squared lengths in
   --  that many dimensions, the wrong ones spread over a range of that
   --  Measure; Passes counts the times it went over them.

end Driver.World.Round_Trips;
