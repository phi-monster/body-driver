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

   --  What the one test of motion sees of a push of these tests: a move of this much or more.
   Seen_From : constant Real := 0.005;

   --  What the readings did in A17's pushes (beats 7044 and 9009): a free push of 5.8 mrad stopped 1.8 microradians
   --  short of its target, the crawl's push of 11.6 mrad 3.4 mrad short of it, 29 per cent of its length.
   Free_Joints  : constant Joint_Push := (Asked => True, Length => 5.8e-3, Short => 1.8e-6, Seen => False);
   Crawl_Joints : constant Joint_Push := (Asked => True, Length => 1.16e-2, Short => 3.4e-3, Seen => True);

   --  Readings not given: those of a tool that goes straight down, whose length is negative here.
   Straight_Down : constant Joint_Push := (Asked => False, Length => -1.0, Short => 0.0, Seen => False);

   function Stand (X, Y, Z : Real; Turn : Real := 0.0; Pitch : Real := 0.0; Yaw : Real := 0.0) return Pose_Estimate is
     ((Pose                => (Rotation => Exp ([Turn, Pitch, Yaw]), Translation => [X, Y, Z]),
       Position_Covariance => 1.0e-8 * Identity3,
       Rotation_Covariance => 1.0e-8 * Identity3));

   --  The readings of a push that asked Ask and delivered Got along the way down, as the joints of a tool that
   --  goes straight down would show it: what it asked is seen from Seen_From on, and so is how far it fell short.
   function Straight (Ask, Got : Real) return Joint_Push is
     ((Asked => Ask >= Seen_From, Length => Ask, Short => abs (Ask - Got), Seen => abs (Ask - Got) >= Seen_From));

   --  One push from a height, asked down by Ask and delivered down by Got, the tool's place across kept.
   --  The readings are those of a tool going straight down unless they are given.
   function Push
     (T      : in out Track;
      Height : Real;
      Ask    : Real;
      Got    : Real;
      Where  : Points := Both;
      Joints : Joint_Push := Straight_Down) return Judgment
   is
      Said : Judgment;
   begin
      Judge (T, Stand (0.0, 0.0, Height), Stand (0.0, 0.0, Height - Ask), Stand (0.0, 0.0, Height - Got), Into, Where,
             (if Joints.Length < 0.0 then Straight (Ask, Got) else Joints), Said);
      return Said;
   end Push;

   --  Free pushes of the sizes a descent makes, each falling short by the same share of its size.
   procedure Free_Pushes
     (T      : in out Track;
      Share  : Real;
      Count  : Positive;
      Height : in out Real;
      Joints : Joint_Push := Straight_Down)
   is
      Ask : Real := Seen_From;
   begin
      for K in 1 .. Count loop
         declare
            Said : constant Judgment := Push (T, Height, Ask, Ask * (1.0 - Share), Both, Joints);
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
      Judge (T, Stand (0.0, 0.0, Height), Stand (0.0, 0.0, Height - 0.05), Stand (0.04, 0.0, Height), Into, Both,
             Straight (0.05, 0.0), Said);
      Check (Said.Result = Stalled, "a push that took the hand along the table was judged " & Said.Result'Image);
      Check (abs (Said.Asked - 0.05) < 1.0e-12 and then abs Said.Went < 1.0e-12 and then abs (Said.Share - 1.0) < 1.0e-9,
             "the stall was said as asked" & Said.Asked'Image & ", went" & Said.Went'Image & ", share" & Said.Share'Image);
      Check (Said.Point_Stalled and then Said.Joint_Stalled, "the stall was said by the points:" & Said.Point_Stalled'Image
             & " and by the readings:" & Said.Joint_Stalled'Image);
      Check (Pushes (T) = 6, "a stalled push was counted among the free ones");
   end Hand_Slides;

   procedure Hand_Turns_On_Its_Finger is
      --  The finger stands on the table and the hand turns about it as the arm pushes: the origin still goes
      --  down, by 95 per cent of the ask, as free pushes fall short by 3 per cent, and the tip, which the turn
      --  of 0.4 raises by 0.3 (1 - cos 0.4), has stopped. The readings stopped short by 5 per cent, which free
      --  pushes do not by more than 9 per cent. Only the tip tells.
      T      : Track;
      Height : Real := 1.0;
      Ask    : constant Real := 0.05;
      Said   : Judgment;
      Joints : constant Joint_Push := (Asked => True, Length => Ask, Short => 0.05 * Ask, Seen => True);
   begin
      Free_Pushes (T, 0.03, 6, Height);
      declare
         Origin : Track := T;
         Both_Points : Track := T;
         Turned : constant Pose_Estimate := Stand (0.0, 0.0, Height - 0.95 * Ask, Turn => 0.4);
      begin
         Judge (Origin, Stand (0.0, 0.0, Height), Stand (0.0, 0.0, Height - Ask), Turned, Into, Alone, Joints, Said);
         Check (Said.Result = Lowered, "the origin of a hand that turned was judged " & Said.Result'Image & ", share"
                & Said.Share'Image & " against the free pushes'" & Said.Free'Image);
         Judge (Both_Points, Stand (0.0, 0.0, Height), Stand (0.0, 0.0, Height - Ask), Turned, Into, Both, Joints, Said);
         Check (Said.Result = Stalled and then Said.Point = 2 and then Said.Point_Stalled and then not Said.Joint_Stalled,
                "the tip of a hand that turned on its finger was judged " & Said.Result'Image & " at point"
                & Said.Point'Image & ", went" & Said.Went'Image & " of" & Said.Asked'Image);
      end;
   end Hand_Turns_On_Its_Finger;

   procedure Tool_Turns_About_Its_Origin is
      --  A push that turns the tool about its origin asks the origin nothing down and a point at its side
      --  0.3 across a part of the turn (0.3 sin of it). Free pushes of that kind are delivered, and a share
      --  of the origin's nothing is no number: the pushes before must not be left with one that no push
      --  of the descent could be compared with, and a push that is not delivered is a stall at the side.
      Pivot  : constant Points := [[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]];
      T      : Track;
      Said   : Judgment;
      Free_Turn : constant Joint_Push := (Asked => True, Length => 0.06, Short => 1.0e-6, Seen => False);
      Stuck_Turn : constant Joint_Push := (Asked => True, Length => 0.06, Short => 0.06, Seen => True);
   begin
      for K in 1 .. 6 loop
         declare
            From : constant Real := 0.02 * Real (K);
            To   : constant Real := From + 0.2;
         begin
            --  The origin is asked 2e-15 down (the turn is about it) and delivered 3.4e-6 up: A17's aim.
            Judge (T, Stand (0.0, 0.0, 1.0, Pitch => From), Stand (0.0, 0.0, 1.0 - 2.0e-15, Pitch => To),
                   Stand (0.0, 0.0, 1.0 + 3.4e-6, Pitch => To), Into, Pivot, Free_Turn, Said);
            Check (Said.Result = Too_Few or else Said.Result = Lowered, "a free turn about the origin was judged "
                   & Said.Result'Image & ", share" & Said.Share'Image);
         end;
      end loop;
      Judge (T, Stand (0.0, 0.0, 1.0, Pitch => 0.2), Stand (0.0, 0.0, 1.0, Pitch => 0.4), Stand (0.0, 0.0, 1.0, Pitch => 0.2),
             Into, Pivot, Stuck_Turn, Said);
      Check (Said.Result = Stalled and then Said.Point = 2 and then Said.Free < 1.0e-3,
             "a turn that was not delivered was judged " & Said.Result'Image & " at point" & Said.Point'Image
             & " against free shares up to" & Said.Free'Image);
   end Tool_Turns_About_Its_Origin;

   procedure Hand_Deflected is
      --  The readings tell what the hand's points do not (a hand with no tip measured yet, its origin going on
      --  down): free pushes stop short of their targets by 0.03 per cent of their length, then a push stops
      --  short of its target by 29 per cent of it, a motion the one test of motion sees, while the tool goes
      --  down as asked. It is a stall at once, by the readings; the same shortfall below what that test sees
      --  is not; and with the points stalled too (A17's crawl) both say so.
      T      : Track;
      Height : Real := 1.0;
   begin
      Free_Pushes (T, 0.0, 6, Height, Free_Joints);
      declare
         Deflected : Track := T;
         Unseen    : Track := T;
         Both_Say  : Track := T;
         Said      : constant Judgment := Push (Deflected, Height, 0.05, 0.05, Alone, Crawl_Joints);
         Hidden    : constant Judgment :=
           Push (Unseen, Height, 0.05, 0.05, Alone, (Asked => True, Length => 1.16e-2, Short => 3.4e-3, Seen => False));
         Crawl     : constant Judgment := Push (Both_Say, Height, 0.05, 0.0, Both, Crawl_Joints);
      begin
         Check (Said.Result = Stalled and then Said.Joint_Stalled and then not Said.Point_Stalled,
                "a push deflected across its ask, the tool going down as asked, was judged " & Said.Result'Image
                & ", by the readings:" & Said.Joint_Stalled'Image & ", by the points:" & Said.Point_Stalled'Image);
         Check (abs (Said.Joint_Share - 3.4e-3 / 1.16e-2) < 1.0e-12 and then Said.Joint_Free < 1.0e-3,
                "the readings' share was said as" & Said.Joint_Share'Image & " against" & Said.Joint_Free'Image);
         Check (Hidden.Result = Lowered, "a deflection the one test of motion does not see was judged " & Hidden.Result'Image);
         Check (Crawl.Result = Stalled and then Crawl.Joint_Stalled and then Crawl.Point_Stalled,
                "the crawl was judged " & Crawl.Result'Image & ", by the readings:" & Crawl.Joint_Stalled'Image
                & ", by the points:" & Crawl.Point_Stalled'Image);
      end;
   end Hand_Deflected;

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
      --  Free pushes that fall short by nothing a float tells, then a push whose shortfall is a hair beside what
      --  the one test of motion sees: a large share of a small push and not a stall, since no move that small is
      --  told. A push of less than that test sees is not judged at all, and does not forget.
      T      : Track;
      Height : Real := 1.0;
      Said   : Judgment;
   begin
      Free_Pushes (T, 0.0, 6, Height);
      Said := Push (T, Height, 0.01, 0.01 - 1.0e-7);
      Check (Said.Result = Lowered, "a hair's shortfall was judged " & Said.Result'Image);
      Said := Push (T, Height, 0.5 * Seen_From, 0.0);
      Check (Said.Result = Not_Asked and then Pushes (T) = 7, "a push of less than the test sees was judged "
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

   procedure Aim_About_The_Way_Down is
      --  A22's aims (press 2 and press 5): the hand turned about the way down asks every point of it nothing
      --  down, 2e-16 by the rounding of the turn, and the readings delivered 4.7e-8 up, which is no share of
      --  nothing. The aim counts as a push with the share it has, and the descent after it is compared with
      --  itself all the same: free pushes, then one that takes the hand nowhere is a stall by the points.
      Side   : constant Points := [[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]];
      Turned : constant Joint_Push := (Asked => True, Length => 1.48, Short => 1.0e-6, Seen => False);
      T      : Track;
      Height : Real := 1.0;
      Said   : Judgment;
   begin
      Judge (T, Stand (0.0, 0.0, 1.0), Stand (0.0, 0.0, 1.0 - 2.0e-16, Yaw => 1.0),
             Stand (0.0, 0.0, 1.0 + 4.7e-8, Yaw => 1.0), Into, Side, Turned, Said);
      Check (Said.Result = Too_Few and then Said.Share < 1.0e-6,
             "an aim about the way down was judged " & Said.Result'Image & ", share" & Said.Share'Image);
      Free_Pushes (T, 0.0, 6, Height);
      Said := Push (T, Height, 0.05, 0.0);
      Check (Said.Result = Stalled and then Said.Point_Stalled and then Said.Free < 1.0e-6,
             "a push that took the hand nowhere after an aim about the way down was judged " & Said.Result'Image
             & ", by the points:" & Said.Point_Stalled'Image & ", against free shares up to" & Said.Free'Image);
   end Aim_About_The_Way_Down;

   procedure Let_Go_Turning is
      --  A22's let-go after press 5: it raised one side of the hand by 0.05 and lowered the other by 0.01, the
      --  readings falling short of the target by what free pushes do not. It asks the hand, the mean of its
      --  points, nothing down, so the descent before it is forgotten and it is not judged; and a point asked
      --  up that went further up, in a push that does ask the hand down, has stopped nothing.
      Side   : constant Points := [[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]];
      Spread : constant Points := [[0.0, 0.0, 0.0], [0.3, 0.0, 0.0], [-0.3, 0.0, 0.0]];
      Held   : constant Joint_Push := (Asked => True, Length => 0.1, Short => 0.0, Seen => True);
      T      : Track;
      Height : Real := 1.0;
      Said   : Judgment;
   begin
      Free_Pushes (T, 0.0, 6, Height);
      Judge (T, Stand (0.0, 0.0, Height), Stand (0.0, 0.0, Height + 0.05, Pitch => 0.2),
             Stand (0.0, 0.0, Height + 0.139, Pitch => 0.5182), Into, Side, (Asked => True, Length => 0.1, Short => 0.03,
                                                                              Seen => True), Said);
      Check (Said.Result = Not_Asked and then Pushes (T) = 0,
             "a let-go that raised the hand was judged " & Said.Result'Image & " and left" & Pushes (T)'Image
             & " pushes counted");
      Free_Pushes (T, 0.0, 6, Height);
      --  Asked down by 0.05 at the middle and 0.11 at one side, which the turn raises by 0.01 at the other; the
      --  hand turned further than asked (by 30 degrees, the sine of which is a half), so that side rose by 0.1
      --  and the other went down by 0.2.
      Judge (T, Stand (0.0, 0.0, Height), Stand (0.0, 0.0, Height - 0.05, Pitch => 0.2),
             Stand (0.0, 0.0, Height - 0.05, Pitch => 0.5235987755982988), Into, Spread, Held, Said);
      Check (Said.Result = Lowered and then not Said.Point_Stalled,
             "a point asked up that went further up was judged " & Said.Result'Image & ", by the points:"
             & Said.Point_Stalled'Image & ", point" & Said.Point'Image & ", share" & Said.Share'Image);
   end Let_Go_Turning;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.lowering.yaw", "an aim about the way down, which asks the hand nothing down, leaves "
                             & "the descent after it with a share no push can be compared with", Aim_About_The_Way_Down'Access);
      Driver.Tests.Register ("hand.lowering.letgo", "a let-go that raises one side as it lowers the other is judged "
                             & "as a descent, or a point asked up that went further up is stalled", Let_Go_Turning'Access);
      Driver.Tests.Register ("hand.lowering.stall", "a hand that stops going down while the arm follows is not "
                             & "stalled at the first push that takes it nowhere", Hand_Slides'Access);
      Driver.Tests.Register ("hand.lowering.turn", "a hand turning on its finger, its origin still going down, is not "
                             & "stalled at the tip", Hand_Turns_On_Its_Finger'Access);
      Driver.Tests.Register ("hand.lowering.pivot", "a turn about the tool's origin leaves the descent a share no push can be "
                             & "compared with, or is not judged by the point it lowers", Tool_Turns_About_Its_Origin'Access);
      Driver.Tests.Register ("hand.lowering.across", "a push deflected across its ask, the hand's points going down as "
                             & "asked, is not a stall by the readings, or one the test of motion does not see is",
                             Hand_Deflected'Access);
      Driver.Tests.Register ("hand.lowering.sag", "a body whose free pushes fall short by a share is stalled by them, "
                             & "or a push that delivers a part is not", Sagging_Pushes'Access);
      Driver.Tests.Register ("hand.lowering.early", "a stall among the first pushes is judged against nothing",
                             Early_Stalls'Access);
      Driver.Tests.Register ("hand.lowering.retreat", "a retreat does not end a descent's comparison with itself",
                             Retreat_Forgets'Access);
      Driver.Tests.Register ("hand.lowering.hair", "a shortfall below what the test of motion sees is a stall, or a "
                             & "push below it is judged", Hairs_Are_Not_Stalls'Access);
      Driver.Tests.Register ("hand.lowering.rise", "a point that goes up under a push down is not stalled",
                             Rising_Is_A_Stall'Access);
   end Register;

end Driver.Robot.Hand.Lowering.Tests;
