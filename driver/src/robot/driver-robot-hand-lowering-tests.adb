with Driver.Tests;

package body Driver.Robot.Hand.Lowering.Tests is

   use Driver.Numerics.Arrays;
   use Driver.Tests;

   --  The way down is -z. The tool stands upright at a height with its tip 0.3
   --  below its origin, or turned about x by an angle (the tip then stands
   --  0.3 cos of it below, and 0.3 sin of it across).
   Into  : constant Vec3 := [0.0, 0.0, -1.0];
   Tip   : constant Vec3 := [0.0, 0.0, -0.3];
   Both  : constant Points := [[0.0, 0.0, 0.0], Tip];
   Alone : constant Points := [1 => [0.0, 0.0, 0.0]];
   Least : constant Real := 0.005;

   function Stand (X, Y, Z : Real; Turn : Real := 0.0) return Pose_Estimate is
     ((Pose                => (Rotation => Exp ([Turn, 0.0, 0.0]), Translation => [X, Y, Z]),
       Position_Covariance => 1.0e-8 * Identity3,
       Rotation_Covariance => 1.0e-8 * Identity3));

   --  One push from a height, asked down by Ask and delivered down by Got, the tool's place across kept.
   function Push
     (T      : in out Track;
      Height : Real;
      Ask    : Real;
      Got    : Real;
      Where  : Points := Both) return Judgment
   is
      Said : Judgment;
   begin
      Judge (T, Stand (0.0, 0.0, Height), Stand (0.0, 0.0, Height - Ask), Stand (0.0, 0.0, Height - Got), Into, Where,
             Least, Said);
      return Said;
   end Push;

   --  Free pushes of the sizes a descent makes, each falling short by the same share of its size.
   procedure Free_Pushes (T : in out Track; Share : Real; Count : Positive; Height : in out Real) is
      Ask : Real := Least;
   begin
      for K in 1 .. Count loop
         declare
            Said : constant Judgment := Push (T, Height, Ask, Ask * (1.0 - Share));
         begin
            Check (Said.Result /= Stalled and then Said.Result /= Not_Asked,
                   "a free push of" & Ask'Image & " that fell short by a share" & Share'Image & " was judged"
                   & Said.Result'Image);
         end;
         Height := Height - Ask * (1.0 - Share);
         Ask := Real'Min (2.0 * Ask, 0.2);
      end loop;
   end Free_Pushes;

   procedure Hand_Slides is
      --  A17's third press: free pushes take the hand down, then the finger meets the table and the pushes are
      --  followed by the arm while the hand slides along it. The first push that takes nothing down is a stall,
      --  not after it a thousand pushes later; the free pushes before it were not.
      T      : Track;
      Height : Real := 1.0;
      Said   : Judgment;
   begin
      Free_Pushes (T, 0.0, 6, Height);
      Check (Pushes (T) = 6, "the free pushes counted were" & Pushes (T)'Image & ", not 6");
      Judge (T, Stand (0.0, 0.0, Height), Stand (0.0, 0.0, Height - 0.05), Stand (0.04, 0.0, Height), Into, Both, Least,
             Said);
      Check (Said.Result = Stalled, "a push that took the hand along the table was judged " & Said.Result'Image);
      Check (abs (Said.Asked - 0.05) < 1.0e-12 and then abs Said.Went < 1.0e-12 and then abs (Said.Share - 1.0) < 1.0e-9,
             "the stall was said as asked" & Said.Asked'Image & ", went" & Said.Went'Image & ", share" & Said.Share'Image);
      Check (Pushes (T) = 6, "a stalled push was counted among the free ones");
   end Hand_Slides;

   procedure Hand_Turns_On_Its_Finger is
      --  The finger stands on the table and the hand turns about it as the arm pushes: the origin still goes
      --  down, by 95 per cent of the ask, as free pushes fall short by 3 per cent, and the tip, which the turn
      --  of 0.4 raises by 0.3 (1 - cos 0.4), has stopped. Only the tip tells.
      T      : Track;
      Height : Real := 1.0;
      Ask    : constant Real := 0.05;
      Said   : Judgment;
   begin
      Free_Pushes (T, 0.03, 6, Height);
      declare
         Origin : Track := T;
         Both_Points : Track := T;
         Turned : constant Pose_Estimate := Stand (0.0, 0.0, Height - 0.95 * Ask, Turn => 0.4);
      begin
         Judge (Origin, Stand (0.0, 0.0, Height), Stand (0.0, 0.0, Height - Ask), Turned, Into, Alone, Least, Said);
         Check (Said.Result = Lowered, "the origin of a hand that turned was judged " & Said.Result'Image & ", share"
                & Said.Share'Image & " against the free pushes'" & Said.Free'Image);
         Judge (Both_Points, Stand (0.0, 0.0, Height), Stand (0.0, 0.0, Height - Ask), Turned, Into, Both, Least, Said);
         Check (Said.Result = Stalled and then Said.Point = 2,
                "the tip of a hand that turned on its finger was judged " & Said.Result'Image & " at point"
                & Said.Point'Image & ", went" & Said.Went'Image & " of" & Said.Asked'Image);
      end;
   end Hand_Turns_On_Its_Finger;

   procedure Sagging_Pushes is
      --  A body whose free pushes all fall short by 4 per cent of their size, from the least to the largest a
      --  descent asks: none of them stalls however long they go on, a push that delivers 90 per cent (a share
      --  below Z times 4 per cent) is not one either, and a push that delivers 60 per cent is.
      T      : Track;
      Height : Real := 1.0;
   begin
      Free_Pushes (T, 0.04, 14, Height);
      declare
         Mild   : Track := T;
         Heavy  : Track := T;
         Said   : constant Judgment := Push (Mild, Height, 0.1, 0.09);
         Stopped : constant Judgment := Push (Heavy, Height, 0.1, 0.06);
      begin
         Check (Said.Result = Lowered, "a push that fell short by a tenth, where free pushes fall short by 4 per cent, was "
                & Said.Result'Image);
         Check (Stopped.Result = Stalled, "a push that fell short by four tenths, where free pushes fall short by 4 per"
                & " cent, was " & Stopped.Result'Image);
      end;
   end Sagging_Pushes;

   procedure Early_Stalls is
      --  A track forgets nothing it is told and judges nothing from fewer than three pushes: a stall among
      --  the first is taken for what free pushes do.
      T : Track;
   begin
      for K in 1 .. 3 loop
         declare
            Said : constant Judgment := Push (T, 1.0, 0.05, 0.0);
         begin
            Check (Said.Result = Too_Few, "push" & K'Image & " of a descent that stalls from the first was "
                   & Said.Result'Image);
         end;
      end loop;
   end Early_Stalls;

   procedure Retreat_Forgets is
      --  Pushes that ask nothing down end a descent: a retreat, then the next descent compared with itself.
      T      : Track;
      Height : Real := 1.0;
      Said   : Judgment;
   begin
      Free_Pushes (T, 0.0, 5, Height);
      Said := Push (T, Height, -0.2, -0.2);
      Check (Said.Result = Not_Asked and then Pushes (T) = 0,
             "a retreat was judged " & Said.Result'Image & " and left" & Pushes (T)'Image & " pushes counted");
      Said := Push (T, 1.0, 0.05, 0.0);
      Check (Said.Result = Too_Few, "a stall at the start of the next descent was judged " & Said.Result'Image);
   end Retreat_Forgets;

   procedure Hairs_Are_Not_Stalls is
      --  Free pushes that fall short by nothing a float tells, then a push whose shortfall is a hair beside the
      --  tool's noise: a large share of a small push and not a stall, since no move of the tool that small
      --  is told from its noise. A push of less than the noise is not judged at all, and does not forget.
      T      : Track;
      Height : Real := 1.0;
      Said   : Judgment;
   begin
      Free_Pushes (T, 0.0, 6, Height);
      Said := Push (T, Height, 0.01, 0.01 - 1.0e-7);
      Check (Said.Result = Lowered, "a hair's shortfall was judged " & Said.Result'Image);
      Said := Push (T, Height, 0.5 * Least, 0.0);
      Check (Said.Result = Not_Asked and then Pushes (T) = 7, "a push of less than the noise was judged "
             & Said.Result'Image & " and left" & Pushes (T)'Image & " pushes counted");
   end Hairs_Are_Not_Stalls;

   procedure Rising_Is_A_Stall is
      --  A point that goes up under a push that asked it down has stopped going down, and more than by the
      --  push's size.
      T      : Track;
      Height : Real := 1.0;
      Said   : Judgment;
   begin
      Free_Pushes (T, 0.0, 6, Height);
      Said := Push (T, Height, 0.05, -0.02, Alone);
      Check (Said.Result = Stalled and then Said.Share > 1.0, "a point that rose was judged " & Said.Result'Image
             & ", share" & Said.Share'Image);
   end Rising_Is_A_Stall;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.lowering.stall", "a hand that stops going down while the arm follows is not "
                             & "stalled at the first push that takes it nowhere", Hand_Slides'Access);
      Driver.Tests.Register ("hand.lowering.turn", "a hand turning on its finger, its origin still going down, is not "
                             & "stalled at the tip", Hand_Turns_On_Its_Finger'Access);
      Driver.Tests.Register ("hand.lowering.sag", "a body whose free pushes fall short by a share is stalled by them, "
                             & "or a push that delivers a part is not", Sagging_Pushes'Access);
      Driver.Tests.Register ("hand.lowering.early", "a stall among the first pushes is judged against nothing",
                             Early_Stalls'Access);
      Driver.Tests.Register ("hand.lowering.retreat", "a retreat does not end a descent's comparison with itself",
                             Retreat_Forgets'Access);
      Driver.Tests.Register ("hand.lowering.hair", "a shortfall below the tool's noise is a stall, or a push below it "
                             & "is judged", Hairs_Are_Not_Stalls'Access);
      Driver.Tests.Register ("hand.lowering.rise", "a point that goes up under a push down is not stalled",
                             Rising_Is_A_Stall'Access);
   end Register;

end Driver.Robot.Hand.Lowering.Tests;
