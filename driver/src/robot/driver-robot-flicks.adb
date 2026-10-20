with Ada.Containers.Vectors;
with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Driver.Beats;
with Driver.Commands;
with Driver.Log;
with Driver.Robot.Channels;
with Driver.Robot.Kinematics;
with Driver.Robot.Kinematics.Beacon;
with Driver.Robot.Kinematics.Placement;
with Driver.Robot.Motion;
with Driver.Uncertain;

package body Driver.Robot.Flicks is

   use type Driver.Robot.Motion.Step_Outcome;
   use type Driver.Observations.Camera_Id;
   use type Driver.Observations.Group_Id;
   use type Driver.Numerics.Arrays.Real_Vector;

   function Sqrt (X : Real) return Real renames Ada.Numerics.Long_Elementary_Functions.Sqrt;

   type Pose_Readings is array (Positive range <>, Positive range <>) of Real;
   type Pose_Access is access Pose_Readings;
   procedure Free is new Ada.Unchecked_Deallocation (Pose_Readings, Pose_Access);

   package Place_Lists is new Ada.Containers.Vectors (Positive, Driver.Numerics.Vec3, Driver.Numerics.Arrays."=");

   procedure Place (M : in out Model; Arm : Arm_Id) is
      World  : Natural := 0;
      Closer : Group_Id'Base := 0;
      Arm_G  : Group_Id'Base := 0;
      Size   : Natural := 0;
      Ready  : Boolean := False;
      --  What the model gives in one beat (Look), per channel from nought: the arm's reference reading and the range
      --  its joints were read over, the offset at which a revolute joint repeats its poses (a half turn; none for a
      --  slide), the readings' noise, and the closer's two ends: the lowest and the highest reading it was seen at.
      Ref, Low, High, Half_Turn, Noise : Real_Vectors.Vector;
      Chain                            : Count_Vectors.Vector;
      Closer_Low, Closer_High          : Real_Vectors.Vector;

      --  The first arm's closer is read from the graph (the group that moves a patch inside the arm's eye) and its two
      --  ends from the range its readings were seen over, not from the hand's measure: the flicks come before the
      --  hands, as step three comes before step four, and a flick needs two readings of the closer and nothing of what
      --  its lobes are. (The restarts from a body file took the hand's measure first, 5400 beats of 8000, for two
      --  readings the body file has.)
      procedure Look is
      begin
         for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
            if M.Kinematics (K).Arm = Arm and then M.Kinematics (K).Result.Fitted then
               World := K;
            end if;
         end loop;
         for G in 1 .. Group_Count (M) loop
            if Closer = 0 and then Closer_Arm (M, Group_Id (G)) = Arm then
               Closer := Group_Id (G);
            end if;
         end loop;
         if World = 0 or else Closer = 0 then
            return;
         end if;
         declare
            R : Arm_Fit renames M.Kinematics (World).Result;
            N : constant Natural := Group_Size (M, M.Kinematics (World).Group);
         begin
            if N = 0 or else Group_Size (M, Closer) = 0 or else Natural (R.Reference.Length) /= N
              or else Natural (R.Joints.Length) /= N or else Natural (R.Chain.Length) /= N
              or else Natural (M.Groups (M.Kinematics (World).Group).Low_Seen.Length) /= N
              or else Natural (M.Groups (M.Kinematics (World).Group).High_Seen.Length) /= N
              or else Natural (M.Groups (Closer).Low_Seen.Length) /= Group_Size (M, Closer)
              or else Natural (M.Groups (Closer).High_Seen.Length) /= Group_Size (M, Closer)
            then
               return;
            end if;
            Arm_G := M.Kinematics (World).Group;
            Size := N;
            Chain := R.Chain;
            for C in 1 .. Size loop
               Ref.Append (R.Reference (C - 1));
               Low.Append (M.Groups (Arm_G).Low_Seen (C - 1));
               High.Append (M.Groups (Arm_G).High_Seen (C - 1));
               Noise.Append (Channels.Noise (M, Arm_G, C));
               Half_Turn.Append (if R.Joints (C).Slide or else R.Joints (C).C = 0.0 then Real'Last
                                 else Ada.Numerics.Pi / abs R.Joints (C).C);
            end loop;
            for C in 0 .. Group_Size (M, Closer) - 1 loop
               Closer_Low.Append (M.Groups (Closer).Low_Seen (C));
               Closer_High.Append (M.Groups (Closer).High_Seen (C));
            end loop;
            Ready := True;
         end;
      end Look;
   begin
      Driver.Beats.Within_A_Beat (Look'Access);
      if not Ready then
         return;
      end if;
      declare
         Ref_A   : constant Real_Array := [for C in 1 .. Size => Ref (C - 1)];
         Noise_A : constant Real_Array := [for C in 1 .. Size => Noise (C - 1)];
         Half_A  : constant Real_Array := [for C in 1 .. Size => Half_Turn (C - 1)];
         --  How far each channel goes from the reference, down and up: the range it was read over, then, once Reach has
         --  been along it, as far as it went.
         Down, Up : Real_Array (1 .. Size) := [others => 0.0];
         Got      : Real_Array (1 .. Size) := [others => 0.0];   --  the arm's readings (Read_Got)
         Report   : Driver.Robot.Motion.Step_Report;
         --  Where the closer rests at its two ends, begun at the ends of the range it was read over and, after each
         --  move, the reading it came to rest at.
         High_At  : Real_Array := [for C in 1 .. Natural (Closer_High.Length) => Closer_High (C - 1)];
         Low_At   : Real_Array := [for C in 1 .. Natural (Closer_Low.Length) => Closer_Low (C - 1)];
         Here     : Real_Array (High_At'Range) := [others => 0.0];   --  where the closer stood, and
         Now      : Real_Array (High_At'Range) := [others => 0.0];   --  stands
         Closer_Report : Driver.Robot.Motion.Step_Report;
         Poses    : Natural := 0;
         Readings : Pose_Access;
         Places   : Place_Lists.Vector;
         Flicked  : Real_Vectors.Vector;   --  the readings of the poses the closer was flicked at, Size to a pose
         Flicks_Seen : Natural := 0;       --  the flicks the eyes fixed in the world have seen, read in a held beat

         procedure Go_To
           (G : Group_Id; Target : Real_Array; Report : out Driver.Robot.Motion.Step_Report; Land : Boolean := False)
         is
            C : Driver.Commands.Command;
         begin
            Driver.Commands.Set_Target (C, G, Target);
            Driver.Robot.Motion.Step (M, C, Report, Land => Land);
         end Go_To;

         procedure Read_Got is
         begin
            for C in Got'Range loop
               Got (C) := Channels.Reading (M, Arm_G, (if M.Beats > 0 then M.Beats - 1 else 0), C);
            end loop;
         end Read_Got;

         procedure Read_Now is
         begin
            for C in Now'Range loop
               Now (C) := Channels.Reading (M, Closer, (if M.Beats > 0 then M.Beats - 1 else 0), C);
            end loop;
         end Read_Now;

         procedure Read_Seen is
         begin
            Flicks_Seen := Natural (M.Flicks.Length);
         end Read_Seen;

         --  Whether the arm already stands, within the readings' noise, where it was flicked at once.
         function Flicked_Here return Boolean is
         begin
            for P in 0 .. Natural (Flicked.Length) / Size - 1 loop
               if (for all C in 1 .. Size =>
                     Noise_A (C) < Real'Last
                     and then not Driver.Uncertain.Significant
                                    (Got (C) - Flicked (P * Size + C - 1), Noise_A (C) * Sqrt (2.0)))
               then
                  return True;
               end if;
            end loop;
            return False;
         end Flicked_Here;

         --  The squared distance of two readings of the closer.
         function Apart (A, B : Real_Array) return Real is
            Sum : Real := 0.0;
         begin
            for C in A'Range loop
               Sum := Sum + (A (C) - B (C)) ** 2;
            end loop;
            return Sum;
         end Apart;

         --  The channel goes farther along Way (-1 or 1) than it did if it went by more than two readings of it tell.
         function Farther (C : Positive; Gain : Real) return Boolean is
           (Gain > 0.0
            and then (Noise_A (C) = Real'Last or else Driver.Uncertain.Significant (Gain, Noise_A (C) * Sqrt (2.0))));

         --  How far the channel goes along Way from the reference: the arm is sent the channel's range, then twice
         --  that, and twice again, and each time stands where it came to; the last that took it farther is its extent.
         --  The push ends against the joint's stop (or the table), which is the extent: that is where the joint goes.
         --  A revolute joint is not sent past a half turn, which is every pose it has. The arm comes back to the
         --  reference after.
         procedure Reach (C : Positive; Way : Real) is
            Level  : Real := (if Way < 0.0 then Down (C) else Up (C));
            Went   : Real := 0.0;
            Target : Real_Array (1 .. Size);
         begin
            if Level <= 0.0 then
               return;
            end if;
            loop
               Target := Ref_A;
               Target (C) := Ref_A (C) + Way * Level;
               Go_To (Arm_G, Target, Report);
               Driver.Beats.Within_A_Beat (Read_Got'Access);
               declare
                  Along : constant Real := Way * (Got (C) - Ref_A (C));
               begin
                  exit when not Farther (C, Along - Went);
                  Went := Along;
               end;
               exit when Level >= Half_A (C);
               Level := 2.0 * Level;
            end loop;
            if Way < 0.0 then
               Down (C) := Went;
            else
               Up (C) := Went;
            end if;
            Go_To (Arm_G, Ref_A, Report);
         end Reach;

         --  The poses: the reference; each joint of the chain alone, half way to each extent and at it; every two
         --  joints of the chain half way to their extents together.
         Row : Natural := 1;
         procedure Fill (J, K : Natural; Dj, Dk : Real) is
         begin
            Row := Row + 1;
            for C in 1 .. Size loop
               Readings (Row, C) := Ref_A (C);
            end loop;
            if J > 0 then
               Readings (Row, J) := Ref_A (J) + Dj;
            end if;
            if K > 0 then
               Readings (Row, K) := Ref_A (K) + Dk;
            end if;
         end Fill;

         function Extent (C : Positive; Way : Real) return Real is (if Way < 0.0 then Down (C) else Up (C));

         procedure Build is
            Ways  : array (1 .. Size) of Natural := [others => 0];   --  how many ways a joint has to go: 0 to 2
            Total : Natural := 1;
         begin
            for K in Chain.First_Index .. Chain.Last_Index loop
               if Chain (K) <= Size then
                  Ways (Chain (K)) :=
                    (if Down (Chain (K)) > 0.0 then 1 else 0) + (if Up (Chain (K)) > 0.0 then 1 else 0);
               end if;
            end loop;
            for J in 1 .. Size loop
               Total := Total + 2 * Ways (J);
               for K in J + 1 .. Size loop
                  Total := Total + Ways (J) * Ways (K);
               end loop;
            end loop;
            Poses := Total;
            Readings := new Pose_Readings (1 .. Total, 1 .. Size);
            for C in 1 .. Size loop
               Readings (1, C) := Ref_A (C);
            end loop;
            for J in 1 .. Size loop
               for Way of Real_Array'[-1.0, 1.0] loop
                  if Extent (J, Way) > 0.0 then
                     for Share of Real_Array'[0.5, 1.0] loop
                        Fill (J, 0, Way * Share * Extent (J, Way), 0.0);
                     end loop;
                  end if;
               end loop;
            end loop;
            for J in 1 .. Size loop
               for K in J + 1 .. Size loop
                  for Wj of Real_Array'[-1.0, 1.0] loop
                     for Wk of Real_Array'[-1.0, 1.0] loop
                        if Extent (J, Wj) > 0.0 and then Extent (K, Wk) > 0.0 then
                           Fill (J, K, Wj * 0.5 * Extent (J, Wj), Wk * 0.5 * Extent (K, Wk));
                        end if;
                     end loop;
                  end loop;
               end loop;
            end loop;
            for F in 1 .. Poses loop
               Places.Append (Driver.Robot.Kinematics.Eye_In_Reference
                                (M, Arm, [for C in 1 .. Size => Readings (F, C)]).Translation);
            end loop;
         end Build;

         --  What the flicks have yet to place. Of the first arm, every fixed eye that saw it move is placed, or has no
         --  more than the beacon's terms of the flicks. Of another arm, the arm itself in the world: through an eye
         --  that is placed and sees it, with no fewer than the placement's least sightings of its flicks.
         Unplaced, Short : Boolean := False;
         procedure Read_Eyes is
            In_World : Boolean := False;
            Placement : Rigid;
            Unit      : Real;
         begin
            Unplaced := False;
            Short := False;
            Driver.Robot.Kinematics.In_World (M, Arm, Placement, Unit, In_World);
            for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
               --  An eye fixed in the world that the lock-in saw respond to the arm is to be placed, whether it has
               --  seen a flick yet or not; the flicks go on while one is not (k18, 10-20: with none seen yet, the poses
               --  ended at the second).
               if Eye_Mount (M, E).Kind = World_Fixed and then Driver.Robot.Kinematics.Sees (M, E, Arm_G) then
                  declare
                     Seen : Natural := 0;
                  begin
                     for X of M.Flicks loop
                        if X.Eye = E and then X.Arm = Arm then
                           Seen := Seen + 1;
                        end if;
                     end loop;
                     if Arm = 1 then
                        if not Driver.Robot.Kinematics.Fixed_Known (M, E) then
                           Unplaced := True;
                           Short := Short or else Seen <= Driver.Robot.Kinematics.Beacon.Terms;
                        end if;
                     elsif Driver.Robot.Kinematics.Fixed_Known (M, E) and then not In_World then
                        Unplaced := True;
                        Short := Short or else Seen < Driver.Robot.Kinematics.Placement.Least_Sightings;
                     end if;
                  end;
               end if;
            end loop;
         end Read_Eyes;

         --  The closer to the end of its range it is not at, the arm held still at the pose it reached: the eyes fixed
         --  in the world have two still views at the reading it leaves before it moves, and two at the reading it
         --  comes to, which make the flick (Kinematics.Watch_Flicks). Each move is made once, not landed, and the
         --  reading the closer came to rest at is kept for the next: a closer sent past its stop pins against it, and
         --  sent again to land on the reading it pushed against it, the arm shakes under the push for the beats a
         --  flick needs it still (k17, 10-20: no flick in 1400 beats). Sent to the reading it rested at, it arrives
         --  softly. And a move is made when the eyes' views are gathered, not when the arm is still: the eye fixed in
         --  the world settles after the arm's own, and a closer moved a beat after the arm stopped had no view before
         --  it (k20, 10-20: 73 poses, no flick). Which end it goes to is read from where it stands, not kept as a
         --  state: a closer whose fingers stand on the table at a pose does not shut there, and shuts of itself when
         --  the arm lifts them, so that the end it stands at is not the one the last move was for. The reading it came
         --  to is kept as the rest of an end when it stands nearer that end than the other, not when the step says it
         --  reached: a step that moved nothing was judged reached (k30, 10-20, the third pose), and the high reading
         --  was kept as the low one, and the closer sent to the reading it stood at for the seventy poses after: two
         --  flicks in 73 poses (k28: the same by a step judged blocked).
         procedure Move_Closer is
            To_High : Boolean;
            Came    : Boolean;
         begin
            Driver.Robot.Motion.Hold_For_Keyframe (M, Arm);
            Driver.Robot.Motion.Hold_For_Flick (M, Arm);
            Driver.Beats.Within_A_Beat (Read_Now'Access);
            Here := Now;
            To_High := Apart (Here, Low_At) < Apart (Here, High_At);
            Driver.Log.Line (Driver.Log.Robot, "boot: the closer at" & Driver.Log.Image (Here (Here'First), 1)
                             & " goes to its " & (if To_High then "high" else "low") & " reading,"
                             & Driver.Log.Image
                                 ((if To_High then High_At (High_At'First) else Low_At (Low_At'First)), 1)
                             & " (the high one" & Driver.Log.Image (High_At (High_At'First), 1) & ", the low one"
                             & Driver.Log.Image (Low_At (Low_At'First), 1) & ")");
            Go_To (Closer, (if To_High then High_At else Low_At), Closer_Report);
            Driver.Robot.Motion.Hold_For_Keyframe (M, Arm);
            Driver.Robot.Motion.Hold_For_Flick (M, Arm);
            Driver.Beats.Within_A_Beat (Read_Now'Access);
            Came := (if To_High then Apart (Now, High_At) < Apart (Now, Low_At)
                     else Apart (Now, Low_At) < Apart (Now, High_At));
            if Came then
               if To_High then
                  High_At := Now;
               else
                  Low_At := Now;
               end if;
            end if;
            Driver.Log.Line (Driver.Log.Robot, "boot: the closer's step " & Closer_Report.Outcome'Image
                             & ", it stands at"
                             & Driver.Log.Image (Now (Now'First), 1)
                             & (if Came then "" else ", not at the end it was sent to"));
         end Move_Closer;

         Estimated_Short : Boolean := True;
         Count           : Natural := 0;
         Next            : Positive := 1;   --  the reference first
      begin
         --  Nothing to do when no fixed eye that sees the arm lacks a place: a body begun from its file keeps the eyes
         --  it placed.
         Driver.Beats.Within_A_Beat (Read_Eyes'Access);
         if not Unplaced then
            return;
         end if;
         --  Where the joints go. A joint is sent along the range it was read over and past it while it goes farther,
         --  then back; poses made of that range alone told nothing of the lens (k31, 10-20: 41 flicks over 7 by 3 by
         --  8 cm of arm at 90 cm).
         for K in Chain.First_Index .. Chain.Last_Index loop
            if Chain (K) <= Size then
               Down (Chain (K)) := Real'Max (0.0, Ref_A (Chain (K)) - Low (Chain (K) - 1));
               Up (Chain (K)) := Real'Max (0.0, High (Chain (K) - 1) - Ref_A (Chain (K)));
            end if;
         end loop;
         for K in Chain.First_Index .. Chain.Last_Index loop
            if Chain (K) <= Size then
               Reach (Chain (K), -1.0);
               Reach (Chain (K), 1.0);
            end if;
         end loop;
         declare
            Text : Ada.Strings.Unbounded.Unbounded_String;
         begin
            for K in Chain.First_Index .. Chain.Last_Index loop
               if Chain (K) <= Size then
                  Ada.Strings.Unbounded.Append
                    (Text, " c" & Natural'Image (Chain (K)) & " -" & Driver.Log.Image (Down (Chain (K)), 1)
                     & " +" & Driver.Log.Image (Up (Chain (K)), 1) & ";");
               end if;
            end loop;
            Driver.Log.Line (Driver.Log.Robot, "boot: arm" & Arm'Image & "'s joints go from their reference as far as:"
                             & Ada.Strings.Unbounded.To_String (Text));
         end;
         Driver.Beats.Within_A_Beat (Build'Access);
         declare
            Used    : array (1 .. Poses) of Boolean := [others => False];
            Nearest : array (1 .. Poses) of Real := [others => Real'Last];
            Relaxed : Driver.Robot.Motion.Step_Report;
         begin
            Driver.Log.Line (Driver.Log.Robot, "boot: arm" & Arm'Image & "'s closer is flicked at the"
                             & Poses'Image & " poses of its excursions, the farthest first");
            --  Once to each reading before the first pose, for the readings the closer rests at: no eye is asked to see
            --  these.
            Move_Closer;
            Move_Closer;
            loop
               Used (Next) := True;
               Count := Count + 1;
               declare
                  Pose    : constant Real_Array := [for C in 1 .. Size => Readings (Next, C)];
                  Flicked_Now : Boolean := False;
               begin
                  --  The arm is not landed on the pose: a flick records the readings the arm stands at, and the fit
                  --  takes the eye there; landing a pose beyond what the body reaches took a hundred beats of resends
                  --  (k17, 10-20). A pose it comes to rest short of is flicked where it stands, unless it has been
                  --  flicked there. The hand is not turned to show its jaw: the poses turn the joints alone and in
                  --  pairs.
                  Go_To (Arm_G, Pose, Report);
                  Driver.Beats.Within_A_Beat (Read_Got'Access);
                  --  An arm that stopped short of a pose it was sent to is sent to where it stands: held at the target
                  --  it stopped against, it would push on it for as long as the flick takes.
                  if Report.Outcome /= Driver.Robot.Motion.Reached then
                     Go_To (Arm_G, Got, Relaxed);
                     Driver.Beats.Within_A_Beat (Read_Got'Access);
                  end if;
                  if Report.At_Rest and then not Flicked_Here then
                     Move_Closer;
                     for C in Got'Range loop
                        Flicked.Append (Got (C));
                     end loop;
                     Flicked_Now := True;
                  end if;
                  --  What the pose gave, for the log: where the arm stood, whether the closer was flicked and how many
                  --  flicks the eyes have seen in all.
                  Driver.Beats.Within_A_Beat (Read_Seen'Access);
                  Driver.Log.Line (Driver.Log.Robot, "boot: flick pose" & Count'Image & " of" & Poses'Image
                                   & ": the arm "
                                   & (if Report.Outcome = Driver.Robot.Motion.Reached then "reached it"
                                      elsif Report.At_Rest then "stopped short of it" else "did not come to rest")
                                   & (if Flicked_Now then ", the closer was flicked"
                                      else ", the closer was not flicked")
                                   & ", the eyes have seen" & Flicks_Seen'Image & " flicks in all");
               end;
               Driver.Beats.Within_A_Beat (Read_Eyes'Access);
               if Unplaced and then not Short and then Estimated_Short then
                  Driver.Robot.Motion.Hold_And_Estimate (M);
                  Driver.Beats.Within_A_Beat (Read_Eyes'Access);
                  Estimated_Short := Short;
               end if;
               exit when Count > 1 and then not Unplaced;
               --  The next pose: the one farthest from every pose taken.
               for F in 1 .. Poses loop
                  Nearest (F) := Real'Min (Nearest (F), abs (Places (F) - Places (Next)));
               end loop;
               declare
                  Far : Natural := 0;
               begin
                  for F in 1 .. Poses loop
                     if not Used (F) and then (Far = 0 or else Nearest (F) > Nearest (Far)) then
                        Far := F;
                     end if;
                  end loop;
                  exit when Far = 0;
                  Next := Far;
               end;
               if not Short then
                  Estimated_Short := True;
               end if;
            end loop;
         end;
         Go_To (Arm_G, Ref_A, Report, Land => True);
         Driver.Robot.Motion.Hold_And_Estimate (M);
         Driver.Log.Line (Driver.Log.Robot, "boot: the closer was flicked at"
                          & Natural'Image (Natural (Flicked.Length) / Size) & " of the" & Poses'Image & " poses");
         if Readings /= null then
            Free (Readings);
         end if;
      end;
   end Place;

end Driver.Robot.Flicks;
