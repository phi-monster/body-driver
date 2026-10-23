with Ada.Numerics.Long_Elementary_Functions;
with Ada.Containers.Generic_Array_Sort;
with Ada.Containers.Vectors;
with Ada.Unchecked_Deallocation;
with Driver.Robot.Channels;
with Driver.Stats;
with Driver.Log;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Robot.Flow;
with Driver.Robot.Regression;
with Driver.Robot.Steps;

package body Driver.Robot.Lag is

   use Ada.Numerics.Long_Elementary_Functions;
   use type Driver.Observations.Group_Id;

   --  Everything sized by beats lives on the heap: the estimates also run in
   --  the decider's task, whose stack is small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   type Index_Array is array (Positive range <>) of Positive;
   type Index_Access is access Index_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Index_Array, Index_Access);

   type Flags is array (Positive range <>) of Boolean;

   --  A series over the beats of the stream (beat B stored at B + 1), with
   --  gaps where it is not available.
   type Series (Length : Natural) is record
      Value : Real_Array (1 .. Length) := [others => 0.0];
      Have  : Flags (1 .. Length) := [others => False];
   end record;

   type Series_Access is access Series;
   procedure Free is new Ada.Unchecked_Deallocation (Series, Series_Access);

   --  Replaces the available values by their ranks among themselves, ties
   --  sharing the average rank.
   procedure Rank (S : in out Series) is
      N : Natural := 0;
   begin
      for B of S.Have loop
         if B then
            N := N + 1;
         end if;
      end loop;
      if N = 0 then
         return;
      end if;
      declare
         Index : Index_Access := new Index_Array (1 .. N);
         K     : Natural := 0;
         Ranks : Real_Access := new Real_Array (1 .. N);

         procedure Sift (Start, Stop : Natural) is
            Root  : Natural := Start;
            Child : Natural;
            T     : Positive;
         begin
            loop
               Child := 2 * Root;
               exit when Child > Stop;
               if Child < Stop and then S.Value (Index (Child)) < S.Value (Index (Child + 1)) then
                  Child := Child + 1;
               end if;
               exit when not (S.Value (Index (Root)) < S.Value (Index (Child)));
               T := Index (Root);
               Index (Root) := Index (Child);
               Index (Child) := T;
               Root := Child;
            end loop;
         end Sift;
      begin
         for I in S.Have'Range loop
            if S.Have (I) then
               K := K + 1;
               Index (K) := I;
            end if;
         end loop;
         --  Heap sort of the indexes by value.
         for Start in reverse 1 .. N / 2 loop
            Sift (Start, N);
         end loop;
         for Stop in reverse 2 .. N loop
            declare
               T : constant Positive := Index (1);
            begin
               Index (1) := Index (Stop);
               Index (Stop) := T;
            end;
            Sift (1, Stop - 1);
         end loop;
         declare
            I : Positive := 1;
         begin
            while I <= N loop
               declare
                  J : Positive := I;
               begin
                  while J < N and then S.Value (Index (J + 1)) = S.Value (Index (I)) loop
                     J := J + 1;
                  end loop;
                  for L in I .. J loop
                     Ranks (L) := Real (I + J) / 2.0;
                  end loop;
                  I := J + 1;
               end;
            end loop;
         end;
         for L in 1 .. N loop
            S.Value (Index (L)) := Ranks (L);
         end loop;
         Free (Index);
         Free (Ranks);
      end;
   end Rank;

   --  The first difference of a series, in place, available where both
   --  terms are: from the last term back, so each term is differenced
   --  against the one before it while that one still holds its value.
   procedure Difference (S : in out Series) is
   begin
      for I in reverse S.Value'First + 1 .. S.Value'Last loop
         if S.Have (I) and then S.Have (I - 1) then
            S.Value (I) := S.Value (I) - S.Value (I - 1);
         else
            S.Value (I) := 0.0;
            S.Have (I) := False;
         end if;
      end loop;
      if S.Length > 0 then
         S.Value (S.Value'First) := 0.0;
         S.Have (S.Have'First) := False;
      end if;
   end Difference;

   --  The series takes more than one value: some available value differs from the first available one
   --  (one pass; the pass over every pair it used to be took the square of the stream's length when
   --  nothing varied, see Measure).
   function Varies (S : Series) return Boolean is
      First : Natural := 0;
   begin
      for I in S.Value'Range loop
         if S.Have (I) then
            if First = 0 then
               First := I;
            elsif S.Value (I) /= S.Value (First) then
               return True;
            end if;
         end if;
      end loop;
      return False;
   end Varies;

   --  How far a lag can be told at all: a push's response is told from the
   --  next push's only when it shows before that one starts, so the lag is
   --  identifiable up to the median stretch between one burst of pushes and
   --  the next (pushes that each cut the last one short, a ramp, are one
   --  burst; bursts of groups that start on the same beat are one start).
   --  Without two bursts, by the same reasoning, up to the median stretch
   --  between one push and the next within the burst: a single burst of pushes
   --  at a steady rhythm aligns with itself shifted by any number of pushes
   --  (A56: the bursts of the first probe, a push every seven beats, gave eye
   --  2 a lag of 35 beats and eye 3 one of 31, and every probe after waited
   --  that long to look). Without two pushes, every shift the stream allows.
   package Natural_Vectors is new Ada.Containers.Vectors (Positive, Natural);
   package Sorting is new Natural_Vectors.Generic_Sorting;

   --  Where the bursts of pushes start (a push that cut the one before it short is of its burst), and where any push
   --  starts, each in order.
   procedure Starts_Of (M : Model; Starts, Pushes : out Natural_Vectors.Vector) is
   begin
      Starts.Clear;
      Pushes.Clear;
      for S of M.Groups loop
         if S.Commandable then
            declare
               Cut : Boolean := False;
            begin
               for E of S.Episodes loop
                  if not Cut then
                     Starts.Append (E.Start);
                  end if;
                  Pushes.Append (E.Start);
                  Cut := E.Ended and then not E.Settled;
               end loop;
            end;
         end if;
      end loop;
      Sorting.Sort (Starts);
      Sorting.Sort (Pushes);
   end Starts_Of;

   function Identifiable (M : Model; Last : Natural) return Natural is
      Starts : Natural_Vectors.Vector;   --  where a burst starts
      Pushes : Natural_Vectors.Vector;   --  where any push starts
   begin
      Starts_Of (M, Starts, Pushes);
      declare
         --  The median stretch between distinct starts of a list, 0 for none.
         function Median_Gap (List : Natural_Vectors.Vector) return Natural is
            Gaps : Real_Access := new Real_Array (1 .. Natural'Max (1, Natural (List.Length)));
            K    : Natural := 0;
         begin
            for I in List.First_Index + 1 .. List.Last_Index loop
               if List (I) > List (I - 1) then
                  K := K + 1;
                  Gaps (K) := Real (List (I) - List (I - 1));
               end if;
            end loop;
            return Result : constant Natural := (if K = 0 then 0 else Natural (Driver.Stats.Median (Gaps (1 .. K)))) do
               Free (Gaps);
            end return;
         end Median_Gap;
         Between_Bursts : constant Natural := Median_Gap (Starts);
         Between_Pushes : constant Natural := Median_Gap (Pushes);
      begin
         return Natural'Min (Last, (if Between_Bursts > 0 then Between_Bursts
                                    elsif Between_Pushes > 0 then Between_Pushes
                                    else Last));
      end;
   end Identifiable;

   function Longest (M : Model) return Natural is
     (if M.Beats > 0 then Identifiable (M, M.Beats - 1) else 0);

   type Natural_Array is array (Natural range <>) of Natural;
   type Natural_Access is access Natural_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Natural_Array, Natural_Access);

   --  A shift's fit is made from sums over its rows alone (Regression.Explained_Nonnegative on its sums), and every
   --  term of them is a whole number: ranks are halves, so twice a rank is whole, and the motion's are taken about
   --  their mean, the speeds' about the rank of no change. Summed in integers, every shift's sums are exact however
   --  long the stream, and a group whose speed was the same at every row of a shift has nothing about its mean,
   --  exactly, as the rows' own fit had.
   subtype Wide is Long_Long_Long_Integer;
   type Wide_Array is array (Positive range <>) of Wide;
   type Wide_Matrix is array (Positive range <>, Positive range <>) of Wide;
   type Wide_Prefix is array (Natural range <>) of Wide;
   type Wide_Prefix_Access is access Wide_Prefix;
   procedure Free is new Ada.Unchecked_Deallocation (Wide_Prefix, Wide_Prefix_Access);
   type Whole_Array is array (Positive range <>) of Integer;
   type Whole_Access is access Whole_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Whole_Array, Whole_Access);

   --  The sums of one shift's rows: their count; the motion's (Y) and its squares'; and per varying group its speed's
   --  (X), its products with the motion's and with the other groups' speeds (the upper triangle).
   type Sums (K : Natural) is record
      Rows   : Wide := 0;
      Y, YY  : Wide := 0;
      X, XY  : Wide_Array (1 .. K) := [others => 0];
      XX     : Wide_Matrix (1 .. K, 1 .. K) := [others => [others => 0]];
   end record;

   --  The first place of a list in increasing order whose entry is at least Value, one past its end for none.
   function First_At_Least (List : Index_Array; Value : Integer) return Positive is
      Lo : Positive := List'First;
      Hi : Integer := List'Last;
   begin
      while Lo <= Hi loop
         declare
            Mid : constant Positive := Lo + (Hi - Lo) / 2;
         begin
            if List (Mid) < Value then
               Lo := Mid + 1;
            else
               Hi := Mid - 1;
            end if;
         end;
      end loop;
      return Lo;
   end First_At_Least;

   --  A stretch of the stream, from a burst's start to the next's (series places, beat B at B + 1).
   type Span is record
      First, Last : Positive;
   end record;
   type Span_Array is array (Positive range <>) of Span;
   type Span_Access is access Span_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Span_Array, Span_Access);

   function Longer (A, B : Span) return Boolean is (A.Last - A.First > B.Last - B.First);
   procedure Sort_Longest_First is new Ada.Containers.Generic_Array_Sort (Positive, Span, Span_Array, Longer);

   procedure Measure (M : in out Model) is
      Last : constant Integer := M.Beats - 1;
      --  A shift up to the median stretch between bursts (Identifiable) is told over every beat, as it always was. A
      --  longer one lines most pushes' responses up with the next burst's pushes, and is told only where a response
      --  shows before the next burst starts: the eye's motion at a beat is set against the pushes the shift back only
      --  when no burst started between the two beats (both in one window, from a burst's start to the next's), and
      --  such shifts are tried as long as the longest window. Capped at the median stretch, the shifts stopped short
      --  of an eye ten beats behind its readings where the first probe's levels came three beats apart: its lag was
      --  taken at one beat, the eye's motion went with the wrong pushes, and the arm it rides was never swept with it
      --  (robot.boot.reread, 10-17). Told within windows at every shift instead, a picture that flickers for beats
      --  after each move (robot.boot.settling) fitted shifts of two and three beats better than its own one: its
      --  trails ran into the next windows, which only the shortest shifts were told over.
      Starts, Pushes : Natural_Vectors.Vector;
      Every  : constant Natural := (if M.Beats > 0 then Identifiable (M, M.Beats - 1) else 0);
      Bound  : Natural := Every;
      --  A picture shows a push no sooner than the push began, and the readings show it no later than the body's delay
      --  (Steps.Body_Delay: the longest wait of any answered push from its start to its readings' first motion): the
      --  eye can lead the readings by that delay at most, and a shift further back lines the eye's response to one push
      --  up with the next push of a regular train. The Shadow's eye 1, its body answering within 0 beats, was taken
      --  to lead its readings by 15 beats, chance 4.6e-5 of 45 shifts, the moves back and forth of its probe 15 to 20
      --  beats apart: the probe of its hand looked at the wrong beats, saw nothing at any level, and the hand was
      --  probed one channel at a time to the boot's cap (seed 2, paced round p98b, 10-24); the Panda's base camera,
      --  at 2 beats by chance 2e-18, went to -10 by 1.5e-55 as its arm was probed (seed 3, round p97a). Before any push
      --  was answered, none.
      Delay_Beats : Natural;
      Delay_Known : Boolean;
      --  An eye whose lag is known keeps it until the stream has doubled since the lags were last all measured, as the
      --  estimates themselves are redone (Observe); one whose lag is not known is measured at every estimate, as it
      --  was. A lag is a constant of the eye and the evidence for it grows with the pushes: told again over every beat
      --  at every estimate a decider asked for, the known lags took 6 to 10 s of each late in a sweep and 90 s at beat
      --  33040 (s41, servo seed 6 of the fake robot, 10-17, replayed: 31521 shifts up to 15763 beats). Measured only
      --  as the stream doubled, every eye's, an arm's noisy eye kept the lag its first pushes gave it, the lock-in
      --  read it fixed in the world, and its arm was never swept with it (robot.boot.undecided).
      Due       : constant Boolean := M.Lag_Evidence = 0 or else M.Beats >= 2 * M.Lag_Evidence;
      Was_Lags  : constant Lag_Vectors.Vector := M.Lags;
      Was_Known : constant Eye_Flag_Vectors.Vector := M.Lag_Known;
      function Kept (E : Eye_Id) return Boolean is
        (not Due and then Natural (E) <= Natural (Was_Known.Last_Index) and then Was_Known (E));
   begin
      --  Every eye kept: nothing to measure, and the lags stand as they are.
      if not M.Eyes.Is_Empty and then (for all E in M.Eyes.First_Index .. M.Eyes.Last_Index => Kept (E)) then
         return;
      end if;
      Driver.Robot.Steps.Body_Delay (M, Delay_Beats, Delay_Known);
      Starts_Of (M, Starts, Pushes);
      for I in Starts.First_Index .. Starts.Last_Index loop
         --  Bursts of groups that start on one beat are one start: no window between them.
         Bound := Natural (Integer'Max
           (Bound, (if I < Starts.Last_Index then Starts (I + 1) - 1 else Integer'Max (Last, Starts (I))) - Starts (I)));
      end loop;
      M.Lags.Clear;
      M.Lag_Known.Clear;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         M.Lags.Append ((if Kept (E) then Was_Lags (E) else 0));
         M.Lag_Known.Append (Kept (E));
      end loop;
      if Due then
         M.Lag_Evidence := M.Beats;
      end if;
      if Last < 1 then
         return;
      end if;
      declare
         Speeds : array (M.Groups.First_Index .. M.Groups.Last_Index) of Series_Access := [others => null];
         --  The first beat each group's speed did not change at (0 for none): its rank there is that of every beat
         --  between pushes, the most of the stream.
         Still_At : array (M.Groups.First_Index .. M.Groups.Last_Index) of Natural := [others => 0];
      begin
         for G in Speeds'Range loop
            Speeds (G) := new Series (M.Beats);
            if M.Groups (G).Commandable then
               for B in 1 .. Last loop
                  if Channels.Has_Reading (M, G, B) and then Channels.Has_Reading (M, G, B - 1) then
                     declare
                        Sum : Real := 0.0;
                     begin
                        for C in 1 .. M.Groups (G).Size loop
                           Sum := Sum + Channels.Change (M, G, B, C) ** 2;
                        end loop;
                        Speeds (G).Value (B + 1) := (if Channels.Pushed (M, G, B) then Sqrt (Sum) else 0.0);
                        Speeds (G).Have (B + 1) := True;
                     end;
                  end if;
               end loop;
               Difference (Speeds (G).all);
               for I in Speeds (G).Value'Range loop
                  if Speeds (G).Have (I) and then Speeds (G).Value (I) = 0.0 then
                     Still_At (G) := I;
                     exit;
                  end if;
               end loop;
               Rank (Speeds (G).all);
            end if;
         end loop;
         declare
         --  The groups whose speed ranks vary, found once: they are the same at every shift and for
         --  every eye. Found inside Fit_Tail, as they were, a stream of a few thousand beats held at
         --  rest, with no push yet so that every shift the stream allows was tried, spent the whole
         --  estimate here (B0F1, a random body sagging under its hold: the lag measurement took 1.8 s
         --  at 384 beats, 13 at 768, 140 at 1536 and 731 at 2651, and the boot waited on it for
         --  good; the profile of its replay put the time in this scan).
         Varying : array (1 .. Natural (M.Groups.Length)) of Group_Id;
         K       : Natural := 0;
      begin
         for G in Speeds'Range loop
            if M.Groups (G).Commandable and then Varies (Speeds (G).all) then
               K := K + 1;
               Varying (K) := G;
            end if;
         end loop;
         --  No group's speed varied: no shift can be told from another (the fits gave every shift the same chance),
         --  so none is tried and every lag measured now stays unmeasured, as before.
         if K > 0 then
            declare
               --  The beats every varying group has a speed at (a row needs them all), and the groups' speeds there,
               --  twice their ranks less twice the rank of no change (the mean where none is), nought elsewhere.
               Pushes_Read : Flags (1 .. M.Beats) := [others => True];
               Xs      : array (1 .. K) of Whole_Access := [others => null];
               Moving  : Index_Access;   --  the beats some varying group's speed is not that of no change at
               Unread  : Index_Access;   --  the beats some varying group has no speed at
               Windows : Span_Access;    --  longest first
               Moved, Gaps, Count : Natural := 0;
            begin
               for J in 1 .. K loop
                  for I in Pushes_Read'Range loop
                     Pushes_Read (I) := Pushes_Read (I) and then Speeds (Varying (J)).Have (I);
                  end loop;
               end loop;
               for J in 1 .. K loop
                  declare
                     S      : Series renames Speeds (Varying (J)).all;
                     Have   : Natural := 0;
                     Origin : Integer;
                  begin
                     for B of S.Have loop
                        if B then
                           Have := Have + 1;
                        end if;
                     end loop;
                     Origin := (if Still_At (Varying (J)) > 0 then Integer (2.0 * S.Value (Still_At (Varying (J))))
                                else Have + 1);
                     Xs (J) := new Whole_Array'(1 .. M.Beats => 0);
                     for I in Pushes_Read'Range loop
                        if Pushes_Read (I) then
                           Xs (J) (I) := Integer (2.0 * S.Value (I)) - Origin;
                        end if;
                     end loop;
                  end;
               end loop;
               for I in Pushes_Read'Range loop
                  if not Pushes_Read (I) then
                     Gaps := Gaps + 1;
                  elsif (for some J in 1 .. K => Xs (J) (I) /= 0) then
                     Moved := Moved + 1;
                  end if;
               end loop;
               Moving := new Index_Array (1 .. Moved);
               Unread := new Index_Array (1 .. Gaps);
               Moved := 0;
               Gaps := 0;
               for I in Pushes_Read'Range loop
                  if not Pushes_Read (I) then
                     Gaps := Gaps + 1;
                     Unread (Gaps) := I;
                  elsif (for some J in 1 .. K => Xs (J) (I) /= 0) then
                     Moved := Moved + 1;
                     Moving (Moved) := I;
                  end if;
               end loop;
               --  The windows from one burst's start to the next's, as beats: the first from the stream's start, the
               --  last to its end; bursts that start on one beat open one window.
               declare
                  Lo : Integer := 0;
               begin
                  Windows := new Span_Array (1 .. Natural (Starts.Length) + 1);
                  for W in 0 .. Natural (Starts.Length) loop
                     declare
                        Hi : constant Integer :=
                          (if W < Natural (Starts.Length) then Starts (Starts.First_Index + W) - 1 else Last);
                     begin
                        if Hi >= Lo then
                           Count := Count + 1;
                           Windows (Count) := (First => Lo + 1, Last => Hi + 1);
                        end if;
                        Lo := Integer'Max (Lo, Hi + 1);
                     end;
                  end loop;
                  Sort_Longest_First (Windows (1 .. Count));
               end;
               for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
                  declare
                     S      : Eye_Stream renames M.Eyes (E);
                     N      : constant Natural := Cells (S.Grid);
                     Motion : Series_Access := new Series (M.Beats);
                     Best   : Real := 1.0;     --  the smallest chance of a fit this good by noise
                     Lag    : Integer := 0;
                     Tests  : Natural := 0;
                     Beyond : Real := 1.0;     --  the smallest at a shift longer than Every, told within windows
                     Beyond_Lag : Integer := 0;
                  begin
                     if N > 0 and then not Kept (E) then
                        for B in 1 .. Natural'Min (Last, Natural (S.Measured.Length) - 1) loop
                           if S.Measured (B) then
                              declare
                                 Sum : Real := 0.0;
                              begin
                                 --  A cell whose content moved too far to be resolved moved at
                                 --  least as far as a resolved displacement can go: half its
                                 --  width and half its height.
                                 for C in 0 .. N - 1 loop
                                    if S.Resolved (B * N + C) then
                                       Sum := Sum + Sqrt (S.Du (B * N + C) ** 2 + S.Dv (B * N + C) ** 2);
                                    elsif S.Condition (B * N + C) > 0.0 then
                                       declare
                                          X0, X1, Y0, Y1 : Natural;
                                       begin
                                          Flow.Bounds (S.Grid, C + 1, X0, X1, Y0, Y1);
                                          Sum := Sum + Sqrt ((Real (X1 - X0) / 2.0) ** 2 + (Real (Y1 - Y0) / 2.0) ** 2);
                                       end;
                                    end if;
                                 end loop;
                                 Motion.Value (B + 1) := Sum / Real (N);
                                 Motion.Have (B + 1) := True;
                              end;
                           end if;
                        end loop;
                        Difference (Motion.all);
                        Rank (Motion.all);
                        declare
                           --  The eye's motion, twice its ranks less their mean (the ranks of the beats it has run
                           --  from one, so their mean is half of one more than their count), nought where it has
                           --  none; and the sums from the stream's start of its count, of it and of its squares.
                           Y   : Whole_Access := new Whole_Array'(1 .. M.Beats => 0);
                           Pn  : Natural_Access := new Natural_Array (0 .. M.Beats);
                           Py  : Wide_Prefix_Access := new Wide_Prefix (0 .. M.Beats);
                           Pyy : Wide_Prefix_Access := new Wide_Prefix (0 .. M.Beats);
                           Have : Natural := 0;

                           --  Adds the rows of a shift within one stretch of the stream (series places First ..
                           --  Through): every pair of a push place and the motion place the shift later, both within
                           --  it, where the motion and every varying group's speed are had. The motion's sums come
                           --  from its sums from the start, less the places some group has no speed at; the speeds'
                           --  from the places some group's speed moved at alone (nought everywhere else).
                           procedure Add (Into : in out Sums; First, Through : Positive; Shift : Integer) is
                              Lo : constant Integer := First + Integer'Max (0, -Shift);   --  the push places
                              Hi : constant Integer := Through - Integer'Max (0, Shift);
                           begin
                              if Lo > Hi then
                                 return;
                              end if;
                              Into.Rows := Into.Rows + Wide (Pn (Hi + Shift) - Pn (Lo + Shift - 1));
                              Into.Y := Into.Y + (Py (Hi + Shift) - Py (Lo + Shift - 1));
                              Into.YY := Into.YY + (Pyy (Hi + Shift) - Pyy (Lo + Shift - 1));
                              for Q in First_At_Least (Unread.all, Lo) .. Unread'Last loop
                                 exit when Unread (Q) > Hi;
                                 declare
                                    I : constant Positive := Unread (Q) + Shift;
                                 begin
                                    if Motion.Have (I) then
                                       Into.Rows := Into.Rows - 1;
                                       Into.Y := Into.Y - Wide (Y (I));
                                       Into.YY := Into.YY - Wide (Y (I)) * Wide (Y (I));
                                    end if;
                                 end;
                              end loop;
                              for Q in First_At_Least (Moving.all, Lo) .. Moving'Last loop
                                 exit when Moving (Q) > Hi;
                                 declare
                                    P : constant Positive := Moving (Q);
                                    I : constant Positive := P + Shift;
                                 begin
                                    if Motion.Have (I) then
                                       for J in 1 .. K loop
                                          declare
                                             XJ : constant Wide := Wide (Xs (J) (P));
                                          begin
                                             if XJ /= 0 then
                                                Into.X (J) := Into.X (J) + XJ;
                                                Into.XY (J) := Into.XY (J) + XJ * Wide (Y (I));
                                                for L in J .. K loop
                                                   Into.XX (J, L) := Into.XX (J, L) + XJ * Wide (Xs (L) (P));
                                                end loop;
                                             end if;
                                          end;
                                       end loop;
                                    end if;
                                 end;
                              end loop;
                           end Add;

                           --  The chance that noise fits the eye's motion at a shift as well as the pushes do: the
                           --  eye's differenced motion ranks regressed on every varying group's differenced speed ranks
                           --  at once, each of which can only add motion, against the F distribution of that fit; its
                           --  sums about their means are the rows' count times them, exactly.
                           function Tail_Of (Of_Rows : Sums) return Real is
                              Rows  : constant Natural := Natural (Of_Rows.Rows);
                              Gram  : Driver.Numerics.Arrays.Real_Matrix (1 .. K, 1 .. K);
                              Cross : Driver.Numerics.Arrays.Real_Vector (1 .. K);
                              Total : constant Real := Real (Of_Rows.Rows * Of_Rows.YY - Of_Rows.Y * Of_Rows.Y);
                              Used  : Natural;
                           begin
                              for J in 1 .. K loop
                                 Cross (J) := Real (Of_Rows.Rows * Of_Rows.XY (J) - Of_Rows.X (J) * Of_Rows.Y);
                                 for L in 1 .. K loop
                                    Gram (J, L) := Real (Of_Rows.Rows * Of_Rows.XX (Integer'Min (J, L), Integer'Max (J, L))
                                                         - Of_Rows.X (J) * Of_Rows.X (L));
                                 end loop;
                              end loop;
                              declare
                                 R2 : constant Real := Regression.Explained_Nonnegative (Gram, Cross, Total, Used);
                              begin
                                 if Used = 0 or else R2 >= 1.0 then
                                    return (if Used = 0 then 1.0 else 0.0);
                                 end if;
                                 return Driver.Distributions.F_Upper_Tail
                                   ((R2 / Real (Used)) / ((1.0 - R2) / Real (Rows - Used - 1)), Used, Rows - Used - 1);
                              end;
                           end Tail_Of;
                        begin
                           for B of Motion.Have loop
                              if B then
                                 Have := Have + 1;
                              end if;
                           end loop;
                           Pn (0) := 0;
                           Py (0) := 0;
                           Pyy (0) := 0;
                           for I in 1 .. M.Beats loop
                              if Motion.Have (I) then
                                 Y (I) := Integer (2.0 * Motion.Value (I)) - (Have + 1);
                              end if;
                              Pn (I) := Pn (I - 1) + (if Motion.Have (I) then 1 else 0);
                              Py (I) := Py (I - 1) + Wide (Y (I));
                              Pyy (I) := Pyy (I - 1) + Wide (Y (I)) * Wide (Y (I));
                           end loop;
                           for Shift in -Natural'Min (Bound, (if Delay_Known then Delay_Beats else 0)) .. Bound loop
                              declare
                                 Of_Rows : Sums (K);
                              begin
                                 if abs Shift <= Every then
                                    Add (Of_Rows, 1, M.Beats, Shift);
                                 else
                                    for W of Windows (1 .. Count) loop
                                       exit when W.Last - W.First < abs Shift;
                                       Add (Of_Rows, W.First, W.Last, Shift);
                                    end loop;
                                 end if;
                                 --  A shift has rows enough for its fit.
                                 if Of_Rows.Rows > Wide (K + 1) then
                                    declare
                                       Tail : constant Real := Tail_Of (Of_Rows);
                                    begin
                                       Tests := Tests + 1;
                                       if abs Shift <= Every then
                                          if Tail < Best then
                                             Best := Tail;
                                             Lag := Shift;
                                          end if;
                                       elsif Tail < Beyond then
                                          Beyond := Tail;
                                          Beyond_Lag := Shift;
                                       end if;
                                    end;
                                 end if;
                              end;
                           end loop;
                           Free (Y);
                           Free (Pn);
                           Free (Py);
                           Free (Pyy);
                        end;
                        --  The best of many shifts is one of a family: the lag is
                        --  measured only when it stands out of all of them.
                        --  A longer shift is told over fewer beats, the windows where the shorter ones' trails do not
                        --  reach: it displaces the best of the shorter ones only when its fit is the less likely by
                        --  noise by Z's tail. The aliased one beat of robot.boot.reread's eye 2 had a chance of 1e-18,
                        --  its true ten beats 1e-270; robot.boot.settling's flickering eye 1 had 2.4e-8 at its true
                        --  one beat and 2.0e-8 at two.
                        if Tests > 0 then
                           Driver.Log.Line (Driver.Log.Robot, "lag: eye" & E'Image & " best at" & Lag'Image
                                            & " beats of the" & Every'Image & " told over every beat, chance"
                                            & Real'Image (Best) & "; beyond, at" & Beyond_Lag'Image & ", chance"
                                            & Real'Image (Beyond) & "; of" & Tests'Image & " shifts up to"
                                            & Bound'Image);
                        end if;
                        if Beyond < Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z) * Best then
                           Best := Beyond;
                           Lag := Beyond_Lag;
                        end if;
                        if Tests > 0
                          and then Best < Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z)
                                          / Real (Tests)
                        then
                           M.Lags.Replace_Element (E, Lag);
                           M.Lag_Known.Replace_Element (E, True);
                        end if;
                     end if;
                     Free (Motion);
                  end;
               end loop;
               for J in 1 .. K loop
                  Free (Xs (J));
               end loop;
               Free (Moving);
               Free (Unread);
               Free (Windows);
            end;
         end if;
      end;
         for G in Speeds'Range loop
            Free (Speeds (G));
         end loop;
      end;
   end Measure;

end Driver.Robot.Lag;
