with Ada.Containers;
with Ada.Environment_Variables;
with Ada.Strings.Fixed;
with Ada.Text_IO;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Clock;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Log;
with Driver.Shared_For;
with Driver.Robot.Channels;
with Driver.Robot.Kinematics;
with Driver.Robot.Flow;
with Driver.Robot.Lag;
with Driver.Robot.Regression;
with Driver.Stats;
with Driver.Uncertain;

package body Driver.Robot.Lockin is

   --  Everything sized by beats, cells or pixels lives on the heap: the
   --  estimates also run in the decider's task, whose stack is small.
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   type Natural_Array is array (Positive range <>) of Natural;
   type Natural_Access is access Natural_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Natural_Array, Natural_Access);
   type Matrix_Access is access Driver.Numerics.Arrays.Real_Matrix;
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Numerics.Arrays.Real_Matrix, Matrix_Access);
   type Start_Access is access Regression.Start;
   procedure Free is new Ada.Unchecked_Deallocation (Regression.Start, Start_Access);
   type Flags is array (Positive range <>) of Boolean;
   type Flags_Access is access Flags;
   procedure Free is new Ada.Unchecked_Deallocation (Flags, Flags_Access);
   type Flag_Grid is array (Positive range <>, Driver.Observations.Group_Id range <>) of Boolean;
   type Flag_Grid_Access is access Flag_Grid;
   procedure Free is new Ada.Unchecked_Deallocation (Flag_Grid, Flag_Grid_Access);
   type Real_Grid is array (Positive range <>, Driver.Observations.Group_Id range <>) of Real;
   type Real_Grid_Access is access Real_Grid;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Grid, Real_Grid_Access);
   type Natural_Grid is array (Positive range <>, Driver.Observations.Group_Id range <>) of Natural;
   type Natural_Grid_Access is access Natural_Grid;
   procedure Free is new Ada.Unchecked_Deallocation (Natural_Grid, Natural_Grid_Access);
   type Noisy_Cell_Access is access Noisy_Cell_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Noisy_Cell_Array, Noisy_Cell_Access);

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;

   package Count_Sorting is new Count_Vectors.Generic_Sorting;

   --  An eye mostly sees the world: a whole image moves when a significant majority of what can move does. The same
   --  half is how much of the energy of the cells too noisy to tell the cells far above the rest must carry to be what
   --  moves (Cells_Shown).
   Half : constant Real := 0.5;

   Whole_Rounding : constant Real := 1.0 / 12.0;
   --  The variance of a uniform rounding to a whole number: of a count of cells.

   --  A regressor: one channel of one commandable group.
   type Column is record
      Group   : Group_Id;
      Channel : Positive;
   end record;

   type Column_Array is array (Positive range <>) of Column;

   function Median_Shift (S : Eye_Stream; Kept, Column, Count : Positive) return Real is
      Values : Real_Access := new Real_Array (1 .. Count);
      K      : Natural := 0;
   begin
      for Cell in 0 .. Natural (S.Shifts.Length) / Kept - 1 loop
         if S.Shifts (Cell * Kept + Column - 1) > 0.0 then
            K := K + 1;
            Values (K) := S.Shifts (Cell * Kept + Column - 1);
         end if;
      end loop;
      return Result : constant Real := Driver.Stats.Median (Values.all) do
         Free (Values);
      end return;
   end Median_Shift;

   function Cells_Disagree (Showing, Silent : Natural) return Boolean is
     (Regression.Count_Significant (Showing + Silent, Showing + Silent, Half)
      and then not Regression.Count_Significant (Showing, Showing + Silent, Half));

   function Cells_Shown (Pool : Noisy_Cell_Array; Typical : Real; Able_Disagree : Boolean; Quiet : Boolean := False)
     return Shown is
      Z : constant Real := Driver.Conventions.Z;

      --  The variance of a cell's energy at the typical energy: Var ** 2 times that of its statistic.
      function Variance_Of (C : Noisy_Cell) return Real is
        (2.0 * Real (C.Freedom) * C.Variance ** 2 + 4.0 * C.Variance * Typical);

      function Far_Above (C : Noisy_Cell) return Boolean is
        (C.Responds and then C.Energy > Typical + Z * Sqrt (Variance_Of (C)));

      Far, Rest : Natural := 0;
      Far_Energy, Energy : Real := 0.0;
      Weights, Weighted  : Real := 0.0;
      Sum_Energy, Sum_Variance : Real := 0.0;   --  for the log line
   begin
      for C of Pool loop
         Sum_Energy := Sum_Energy + C.Energy;
         Sum_Variance := Sum_Variance + C.Variance;
         Energy := Energy + Real'Max (0.0, C.Energy);
         if Far_Above (C) then
            Far := Far + 1;
            Far_Energy := Far_Energy + C.Energy;
         end if;
      end loop;
      declare
         Theirs : constant Boolean := Able_Disagree and then Far > 0 and then Far_Energy >= Half * Energy;
         Counted : constant Natural := (if Theirs then Far else 0);
      begin
         for C of Pool loop
            if not (Theirs and then Far_Above (C)) then
               Rest := Rest + 1;
               Weights := Weights + 1.0 / Variance_Of (C);
               Weighted := Weighted + C.Energy / Variance_Of (C);
            end if;
         end loop;
         if Rest = 0 then
            return (Least => Counted, Most => Counted);
         end if;
         declare
            Mean  : constant Real := Weighted / Weights;
            Sigma : constant Real := 1.0 / Sqrt (Weights);
            Low   : constant Real := Real'Max (0.0, Real'Min (1.0, (Mean - Z * Sigma) / Typical));
            High  : constant Real := Real'Max (0.0, Real'Min (1.0, (Mean + Z * Sigma) / Typical));
         begin
            if not Quiet then
            Driver.Log.Line (Driver.Log.Robot, "lock-in: the" & Pool'Length'Image & " cells too noisy to tell show together "
                             & Driver.Log.Image (Mean, 4) & " +- " & Driver.Log.Image (Sigma, 4) & " of the typical energy "
                             & Driver.Log.Image (Typical, 4) & " (their energies sum to " & Driver.Log.Image (Sum_Energy, 4)
                             & ", their variances to " & Driver.Log.Image (Sum_Variance, 4) & ";" & Far'Image & " far above)");
            end if;
            --  Toward the whole view only when together they show as much as the cells that can tell do: a share
            --  of the typical energy below it is a part of them moving with the others or all of them moving a
            --  little, and of the second a group shows a whole view it does not ride: a closer's push that shakes its
            --  own wrist's camera, Stretch's and ALOHA's jaws called arms by it (round 4a, 10-23: 86 to 123 of 255
            --  cells responding, the rest pulled along by 2 to 8 sigmas), and an arm that fills half a fixed eye's
            --  view, the Panda's base camera called ridden and its arm probed channel by channel (146 of 255).
            return (Least => Counted + (if Mean + Z * Sigma >= Typical then Natural (Real'Floor (Low * Real (Rest))) else 0),
                    Most  => Counted + Natural (Real'Ceiling (High * Real (Rest))));
         end;
      end;
   end Cells_Shown;

   --  How many of an eye's textured cells show group G's motion, at least and
   --  at most, which is what a whole image moving takes (Measure): the cells
   --  whose displacement follows the group, and the cells too noisy to tell by
   --  themselves in the share that they show it together.
   --
   --  A silent cell says nothing against the whole image moving when its own
   --  noise would not let it show the motion. The cells that did respond
   --  measure the motion's energy, the squared displacement the group caused
   --  summed over the regression's rows: a cell's Wald statistic less its
   --  degrees of freedom, times its variance, estimates it without bias. A
   --  cell can tell a motion of the median of those energies when the
   --  statistic it would have, less Z of that statistic's own spread, reaches
   --  the critical value of its test; the energy over its variance that takes
   --  is Needed. Whether a cell can tell depends on its noise alone, not on
   --  whether it responded, so the energies of all the cells that cannot say,
   --  whatever those cells did, how much of the motion they show: Cells_Shown,
   --  as a share of the median. A picture that moves only where it is best
   --  measured gains nothing by this, however many cells are too noisy to
   --  tell: together they show no energy, and the few of them that show a
   --  great deal are what moves, not the others. The cells that can tell are
   --  the witnesses of the whole: they are enough to say it when even all of
   --  them showing it would be a significant majority, and then a significant
   --  majority of them must show it (Disagree says they do not).
   procedure Cells_Showing
     (G          : Group_Id;
      Textured   : Flags;
      Responding : Flag_Grid;
      Energy     : Real_Grid;
      Dof        : Natural_Grid;
      Noise      : Real_Vectors.Vector;
      Least, Most : out Natural;
      Disagree    : out Boolean;
      Resting     : out Real;
      Quiet       : Boolean := False)
   is
      Z        : constant Real := Driver.Conventions.Z;
      Tail     : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Z);
      Count    : Natural := 0;
      Largest  : Natural := 0;
      Energies : Real_Access := new Real_Array (1 .. Textured'Length);

      --  The energy over the variance at which a statistic of that many degrees
      --  of freedom, which is a non-central chi-square of mean K + Lambda and
      --  variance 2 (K + 2 Lambda) for Lambda that ratio, clears the critical
      --  value Q of its test by Z of its spread: the root of
      --  (K + Lambda - Q) ** 2 = Z ** 2 * 2 (K + 2 Lambda) above Q - K.
      function Needed_For (Freedom : Positive) return Real is
         K : constant Real := Real (Freedom);
         Q : constant Real := Driver.Distributions.Chi_Square_Quantile (Tail, Freedom);
      begin
         return (Q - K + 2.0 * Z ** 2) + Z * Sqrt (2.0 * (2.0 * Q - K) + 4.0 * Z ** 2);
      end Needed_For;
   begin
      for Cell in Textured'Range loop
         if Textured (Cell) then
            Largest := Natural'Max (Largest, Dof (Cell, G));
            if Responding (Cell, G) then
               Count := Count + 1;
               Energies (Count) := Energy (Cell, G);
            end if;
         end if;
      end loop;
      Least := Count;
      Most := Count;
      Disagree := False;
      --  The cells that did not respond, together: each one's energy is an unbiased estimate of the motion it
      --  shows (its statistic less its degrees of freedom, in its noise), whatever it is, and under no motion
      --  has the variance of a chi-square: twice its degrees of freedom times its noise to the fourth.
      Resting := 0.0;
      declare
         Weights, Weighted : Real := 0.0;
      begin
         for Cell in Textured'Range loop
            if Textured (Cell) and then not Responding (Cell, G) and then Dof (Cell, G) > 0
              and then Noise (Cell - 1) > 0.0 and then Noise (Cell - 1) < Real'Last
            then
               declare
                  Weight : constant Real := 1.0 / (2.0 * Real (Dof (Cell, G)) * Noise (Cell - 1) ** 4);
               begin
                  if Weight < Real'Last then
                     Weights := Weights + Weight;
                     Weighted := Weighted + Weight * Energy (Cell, G);
                  end if;
               end;
            end if;
         end loop;
         if Weights > 0.0 then
            Resting := Weighted / Sqrt (Weights);
         end if;
      end;
      if Count > 0 and then Largest > 0 then
         declare
            Typical   : constant Real := Driver.Stats.Median (Energies (1 .. Count));
            Needed    : Real_Array (1 .. Largest) := [others => 0.0];   --  by degrees of freedom, found when first asked
            Able      : Natural := 0;   --  cells that could tell and responded
            Silent    : Natural := 0;   --  cells that could tell and did not
            Too_Noisy : Natural := 0;   --  cells that could not tell
            Pool      : Noisy_Cell_Access := new Noisy_Cell_Array (1 .. Textured'Length);
         begin
            for Cell in Textured'Range loop
               if Textured (Cell) and then Dof (Cell, G) > 0 and then Typical > 0.0 then
                  declare
                     Freedom : constant Positive := Dof (Cell, G);
                     Var     : constant Real := Noise (Cell - 1) ** 2;
                  begin
                     if Needed (Freedom) = 0.0 then
                        Needed (Freedom) := Needed_For (Freedom);
                     end if;
                     if Needed (Freedom) * Var <= Typical then
                        if Responding (Cell, G) then
                           Able := Able + 1;
                        else
                           Silent := Silent + 1;
                        end if;
                     else
                        Too_Noisy := Too_Noisy + 1;
                        Pool (Too_Noisy) :=
                          (Energy => Energy (Cell, G), Variance => Var, Freedom => Freedom,
                           Responds => Responding (Cell, G));
                     end if;
                  end;
               end if;
            end loop;
            Disagree := Cells_Disagree (Able, Silent);
            if Too_Noisy > 0 then
               declare
                  Show : constant Shown := Cells_Shown (Pool (1 .. Too_Noisy), Typical, Disagree, Quiet);
               begin
                  Least := Natural'Max (Count, Able + Show.Least);
                  Most := Natural'Max (Count, Able + Show.Most);
               end;
            end if;
            Free (Pool);
         end;
      end if;
      Free (Energies);
   end Cells_Showing;

   function Still_A_Patch (Responding, Textured : Natural) return Boolean is
     (Responding < Textured and then Regression.Count_Significant (Textured - Responding, Textured, Half));

   function Judge (Responding, Textured, Least, Most : Natural; Able_Disagree : Boolean) return Eye_Response is
      --  The per-cell test's own false-alarm rate.
      P0 : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
   begin
      if not Regression.Count_Significant (Responding, Textured, P0) then
         return Nothing;
      elsif not Able_Disagree and then Regression.Count_Significant (Least, Textured, Half) then
         return Whole;
      elsif Regression.Count_Significant (Textured - Most, Textured, Half) then
         return Patch;
      else
         return Undecided;
      end if;
   end Judge;

   procedure Measure (M : in out Model) is
      Groups : constant Natural := Natural (M.Groups.Length);
      Eyes   : constant Natural := Natural (M.Eyes.Length);
      All_Columns : Natural := 0;

      --  The beat at which the push of a group that is under way at a beat began, for every group and beat.
      Starts : Natural_Access := new Natural_Array (1 .. Groups * M.Beats);

      function Push_Start (G : Group_Id; Beat : Natural) return Natural is
        (Starts ((Natural (G) - 1) * M.Beats + Beat + 1));

      --  Whether each commandable group moved at each beat, by the one test of motion at the steps below: asked once for
      --  every eye's settling tail, marks and rows. Asked by each of them for each eye, it was asked nine times a beat
      --  of a humanoid's stream with three eyes, every estimate.
      Moving_At : Flags_Access;

      function Group_Moved (G : Group_Id; Beat : Natural) return Boolean is
        (Moving_At ((Natural (G) - 1) * M.Beats + Beat + 1));

      --  What the measure before this one knew of every group in every eye, for telling a held group's motion that
      --  no cell of an eye can see (Usable): its verdict, every channel's pixels per reading unit, the eye's cell noise.
      Total_Channels : Natural := 0;
      Channel_Of     : array (1 .. Groups) of Natural := [others => 0];   --  the channels of the groups before it
      Was_Verdict    : array (1 .. Groups * Eyes) of Eye_Response := [others => Unmeasured];
      Was_Noise      : array (1 .. Eyes) of Real := [others => Real'Last];
      Was_Shift      : Real_Access;
      Step_Table     : Real_Access;
      function Steps_Of (G : Group_Id) return Real_Array is
        (Step_Table (Channel_Of (Natural (G)) + 1 .. Channel_Of (Natural (G)) + M.Groups (G).Size));
   begin
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         Channel_Of (Natural (G)) := Total_Channels;
         Total_Channels := Total_Channels + M.Groups (G).Size;
      end loop;
      Was_Shift := new Real_Array'(1 .. Eyes * Total_Channels => 0.0);
      --  Every channel's visible step as the body has it when the measure begins, before any eye's gains, which
      --  it rests on, are cleared and made again: the one test of motion is asked at every beat of every group
      --  below, and the step summed every cell of every eye at each asking.
      Step_Table := new Real_Array'(1 .. Total_Channels => Real'Last);
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         declare
            Steps : constant Real_Array := Channels.Steps_Of (M, G);
         begin
            for C in Steps'Range loop
               Step_Table (Channel_Of (Natural (G)) + C) := Steps (C);
            end loop;
         end;
      end loop;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         Was_Noise (Natural (E)) := Cell_Noise (M, E);
         for G in M.Groups.First_Index .. M.Groups.Last_Index loop
            declare
               K : constant Natural := (Natural (G) - 1) * Eyes + Natural (E);   --  as Graph.Effect has it
            begin
               Was_Verdict (K) := (if K <= Natural (M.Graph.Effects.Length) then M.Graph.Effects (K).Verdict else Unmeasured);
            end;
            for C in 1 .. M.Groups (G).Size loop
               Was_Shift ((Natural (E) - 1) * Total_Channels + Channel_Of (Natural (G)) + C) := Shift (M, E, G, C);
            end loop;
         end loop;
      end loop;
      M.Graph.Effects.Clear;
      M.Graph.Effects.Append (Eye_Effect'(others => <>), Ada.Containers.Count_Type (Groups * Eyes));
      M.Graph.Channels.Clear;
      M.Graph.Channels.Append (Channel_Effect_Vectors.Empty_Vector, Ada.Containers.Count_Type (Eyes));
      for S of M.Groups loop
         if S.Commandable then
            All_Columns := All_Columns + S.Size;
         end if;
      end loop;
      if All_Columns = 0 or else M.Beats < 2 then
         Free (Starts);
         Free (Was_Shift);
         Free (Step_Table);
         return;
      end if;
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         declare
            Began : Natural := 0;
         begin
            for B in 0 .. M.Beats - 1 loop
               if M.Groups (G).Commandable and then Channels.Pushed (M, G, B) then
                  if B = 0 or else not Channels.Pushed (M, G, B - 1) then
                     Began := B;
                  end if;
                  Starts ((Natural (G) - 1) * M.Beats + B + 1) := Began;
               else
                  Starts ((Natural (G) - 1) * M.Beats + B + 1) := 0;
               end if;
            end loop;
         end;
      end loop;
      Moving_At := new Flags'(1 .. Groups * M.Beats => False);
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         if M.Groups (G).Commandable then
            for B in 0 .. M.Beats - 1 loop
               Moving_At ((Natural (G) - 1) * M.Beats + B + 1) := Channels.Moving (M, G, B, Steps_Of (G));
            end loop;
         end if;
      end loop;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            S     : Eye_Stream renames M.Eyes (E);
            N     : constant Natural := Cells (S.Grid);
            Lag   : constant Integer := (if E <= M.Lags.Last_Index then M.Lags (E) else 0);
            --  Where the eye's time goes, for the log: the marks of the usable beats, the rows assembled, the
            --  cells fitted (timed below), and what follows the fit.
            Eye_Began   : constant Duration := Driver.Clock.Seconds;
            Marks_Took, Rows_Took : Duration := 0.0;
            Rows  : Natural := 0;

            --  How long the eye's picture goes on settling after the body stops moving: from the last beat any
            --  commandable group moved or was pushed to the first beat after it at which the picture had settled
            --  (the stillness watch), each such stretch of the stream, and of them the bound Z robust deviations above
            --  their median. The picture of a renderer that accumulates its frames drifts for beats after the
            --  readings stop, and carries the tail of a motion into the beats after it. Not the longest stretch: one
            --  stretch where the picture never settled (a hand resting on the table, creeping) made it 1317 beats,
            --  every push of the hand's closer was then within it of another group's, its column left the regression,
            --  and the closer lost its role and the arm its hand (f15t on 10-15; f15f's was 1893).
            function Settling_Tail return Natural is
               Stretches : Real_Vectors.Vector;
               Last      : Natural := 0;   --  the last beat a group moved or was pushed
               Waiting   : Boolean := False;
            begin
               for B in 0 .. Natural'Min (M.Beats, Natural (S.Settled_At.Length)) - 1 loop
                  if (for some G in M.Groups.First_Index .. M.Groups.Last_Index =>
                        M.Groups (G).Commandable and then (Channels.Pushed (M, G, B) or else Group_Moved (G, B)))
                  then
                     Last := B;
                     Waiting := True;
                  elsif Waiting and then S.Settled_At (B) then
                     Stretches.Append (Real (B - Last));
                     Waiting := False;
                  end if;
               end loop;
               if Stretches.Is_Empty then
                  return 0;
               end if;
               declare
                  Values : Real_Array (1 .. Natural (Stretches.Length));
               begin
                  for I in Values'Range loop
                     Values (I) := Stretches (Stretches.First_Index + I - 1);
                  end loop;
                  return Natural (Real'Ceiling (Driver.Stats.Median (Values)
                                                 + Driver.Conventions.Z * Driver.Stats.Robust_Sigma (Values)));
               end;
            end Settling_Tail;
            Tail : constant Natural := Settling_Tail;

            --  A beat of the eye can be explained when its displacement was
            --  measured, every commandable group's readings exist for the
            --  beat it shows and the one before, at most one group was being
            --  pushed then, no other group's reading moved but as the push's
            --  own reaction (a push that coincides with another group's motion
            --  is no reference for either: their effects cannot be told apart,
            --  and a group that moves with no push under way, one that ended or
            --  was given up while it kept moving, moves its picture all the
            --  same; a held group that stood until the push began and, once it is
            --  under way, moves by less than any cell of this eye can see
            --  (Unseen) moves nothing of its picture, and the regression gives
            --  a held group's motion no weight: A94's left closer shook its arm
            --  at every beat of its pushes, none was kept, and no eye ever
            --  measured it; an arm that moves with the gripper by as much as its
            --  own pushes still leaves the beat out, A29's), and the push, if
            --  there is one, began from a picture that had settled,
            --  at the beat before it began: the tail of an earlier motion is
            --  not the push's effect. (That beat, not the one that shows the
            --  push's first reading: the watch restarts at the lag the eye
            --  had when the beat was taken, which is not yet measured early.)
            --  A held group's step at a reading beat moves this eye's picture by less than one cell tells: the eye
            --  showed nothing of the group, or no channel's step times its pixels per reading unit is significant
            --  against the cells' noise (Shows_Step, as the measure before this one had them). Not known of a group
            --  this eye has not measured.
            function Unseen (G : Group_Id; Beat : Natural) return Boolean is
               V : constant Eye_Response := Was_Verdict ((Natural (G) - 1) * Eyes + Natural (E));
            begin
               if V = Nothing then
                  return True;
               elsif V = Unmeasured or else Was_Noise (Natural (E)) = Real'Last then
                  return False;
               end if;
               for C in 1 .. M.Groups (G).Size loop
                  declare
                     Per_Unit : constant Real :=
                       Was_Shift ((Natural (E) - 1) * Total_Channels + Channel_Of (Natural (G)) + C);
                  begin
                     if Per_Unit > 0.0
                       and then Driver.Uncertain.Significant (Channels.Change (M, G, Beat, C) * Per_Unit, Was_Noise (Natural (E)))
                     then
                        return False;
                     end if;
                  end;
               end loop;
               return True;
            end Unseen;

            --  For every commandable group, the last beat up to each at which it was pushed, at which it moved, and
            --  at which it moved by what this eye can see (not Unseen), each as the beat plus one and zero for none
            --  yet. What a usable beat's push must be free of, in the window from the settling tail before it
            --  through the beat, is then asked in one step each, not beat by beat over the window: that scan was
            --  all of the lock-in's time on the fake body, growing with the square of the beats (17 s at 1056
            --  beats, the cells' fits 0.1 s of it).
            Last_Pushed      : Natural_Access := new Natural_Array'(1 .. Groups * M.Beats => 0);
            Last_Moved       : Natural_Access := new Natural_Array'(1 .. Groups * M.Beats => 0);
            Last_Seen_Moving : Natural_Access := new Natural_Array'(1 .. Groups * M.Beats => 0);

            function Mark (Marks : Natural_Access; G : Group_Id; B : Natural) return Natural is
              (Marks ((Natural (G) - 1) * M.Beats + B + 1));

            --  A group's reading change as a cause of what the eye saw at a beat: while it is pushed, and while, held, it
            --  moves by what this eye can see of a group the eye has measured. A held group that moves as another is pushed
            --  (a hand that swings as its arm moves, an arm that sways as its hand does) moves the picture with it, and
            --  is credited with that, rather than the beat left out: the Shadow's arm and hand, each moving at the other's
            --  pushes, left its lock-in no row in a whole boot (seed 3, paced rounds p69 to p79, 10-23: 535 pushed beats,
            --  none a row). Only a group with pushes of its own up to the beat, which give its column its own rows, and only
            --  where the eye can see its motion: a held group that moves a little because another one moved, its tiny
            --  motion a scaled copy of the push's and no push of its own to tell them apart, would take the push's credit
            --  and leave its own still pushes outliers. Asked further, of a group this eye had measured, the Shadow's
            --  arm and hand, never measured, had no row still (paced round p80a). Nor a speed's (Keeps_Going): what its
            --  readings change moves is the body's speed, not its place, and Stretch's wheels, their readings shaken as its
            --  arm moved, took the arm's motion for theirs, and the base was pushed again and again to the boot's end (seed
            --  2, paced round p82b); a speed moving while another group is pushed leaves the beat out (Kind_Of).
            function Own_Pushes (G : Group_Id; Beat : Natural) return Boolean is (Mark (Last_Pushed, G, Beat) > 0);
            function Cause_Change (G : Group_Id; Beat : Natural; Channel : Positive) return Real is
              (if Channels.Pushed (M, G, Beat) then Channels.Change (M, G, Beat, Channel)
               elsif Own_Pushes (G, Beat) and then not Keeps_Going (M, G) and then Group_Moved (G, Beat)
                 and then not Unseen (G, Beat)
               then Channels.Change (M, G, Beat, Channel)
               else 0.0);


            --  Whether the eye's picture moved, at each beat, within what the flow resolves: a cell resolves a
            --  displacement of less than half its width, and where the cells that could not resolve theirs are a
            --  significant majority of those with texture, the picture moved beyond the flow's reach, and the
            --  few cells that did resolve one are not to be believed (a flow that tracks a texture aliases on its
            --  lookalikes a period away). Such a beat is no evidence for the regression, which measures the
            --  small-motion Jacobian: on the fake body the sweep's pushes, half a radian in three beats, put most of
            --  the picture beyond a cell every beat, the gains collapsed under them and the visible steps grew
            --  three hundredfold (R4, the box). Indexed by the beat plus one.
            Reached : Flags_Access := new Flags'(1 .. Natural'Max (1, M.Beats) => True);

            procedure Mark_Reach is
               --  The cells that resolve their displacement at most measured beats: a cell too faint to resolve
               --  any says nothing of how far the picture moved at one (an eye whose faint cells are a majority
               --  would have every push beyond reach, and show nothing of a motion it sees whole).
               Usual    : Flags_Access := new Flags'(1 .. N => False);
               Resolved : Natural_Access := new Natural_Array'(1 .. N => 0);
               Measured : Natural := 0;
            begin
               for B in 0 .. M.Beats - 1 loop
                  if B < Natural (S.Measured.Length) and then S.Measured (B) and then Natural (S.Condition.Length) >= (B + 1) * N then
                     Measured := Measured + 1;
                     for C in 0 .. N - 1 loop
                        if S.Condition (B * N + C) > 0.0 and then S.Resolved (B * N + C) then
                           Resolved (C + 1) := Resolved (C + 1) + 1;
                        end if;
                     end loop;
                  end if;
               end loop;
               for C in 1 .. N loop
                  Usual (C) := 2 * Resolved (C) > Measured;
               end loop;
               for B in 0 .. M.Beats - 1 loop
                  if B < Natural (S.Measured.Length) and then S.Measured (B) and then Natural (S.Condition.Length) >= (B + 1) * N then
                     declare
                        Textured, Solved : Natural := 0;
                     begin
                        for C in 0 .. N - 1 loop
                           if Usual (C + 1) and then S.Condition (B * N + C) > 0.0 then
                              Textured := Textured + 1;
                              if S.Resolved (B * N + C) then
                                 Solved := Solved + 1;
                              end if;
                           end if;
                        end loop;
                        Reached (B + 1) := not Cells_Disagree (Showing => Solved, Silent => Textured - Solved);
                     end;
                  end if;
               end loop;
               Free (Usual);
               Free (Resolved);
            end Mark_Reach;

            procedure Mark_Motions is
            begin
               for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                  if M.Groups (G).Commandable then
                     declare
                        Pushed, Moved, Seen : Natural := 0;
                        Here : Natural := (Natural (G) - 1) * M.Beats;
                     begin
                        for B in 0 .. M.Beats - 1 loop
                           if Channels.Pushed (M, G, B) then
                              Pushed := B + 1;
                           end if;
                           if Group_Moved (G, B) then
                              Moved := B + 1;
                              if not Unseen (G, B) then
                                 Seen := B + 1;
                              end if;
                           end if;
                           Here := Here + 1;
                           Last_Pushed (Here) := Pushed;
                           Last_Moved (Here) := Moved;
                           Last_Seen_Moving (Here) := Seen;
                        end loop;
                     end;
                  end if;
               end loop;
            end Mark_Motions;

            --  What a beat is to the regression: a row, or why not (Usable), for the log as well.
            type Row_Kind is (Row, No_Picture, Not_Reached, No_Readings, Two_Pushed, Held_Moved, Unsettled_Before,
                              Another_Pushed, Another_Moved_Before, Another_Moving, Unsettled);
            function Kind_Of (B : Natural) return Row_Kind is
               R      : constant Integer := B - Lag;
               Pushes : Natural := 0;
               Pusher : Group_Id := M.Groups.First_Index;
               Held_Moving : Boolean := False;   --  a group not pushed moved at the beat
            begin
               if B >= Natural (S.Measured.Length) or else not S.Measured (B) or else R < 1 or else R >= M.Beats then
                  return No_Picture;
               elsif not Reached (B + 1) then
                  return Not_Reached;
               end if;
               for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                  if M.Groups (G).Commandable then
                     if not Channels.Has_Reading (M, G, R) or else not Channels.Has_Reading (M, G, R - 1) then
                        return No_Readings;
                     elsif Channels.Pushed (M, G, R) then
                        Pushes := Pushes + 1;
                        Pusher := G;
                     elsif Group_Moved (G, R) then
                        Held_Moving := True;
                     end if;
                  end if;
               end loop;
               if Pushes > 1 then
                  return Two_Pushed;
               elsif Pushes = 0 and then Held_Moving then
                  return Held_Moved;
               elsif Pushes = 1 then
                  declare
                     Began  : constant Natural := Push_Start (Pusher, Natural (R));
                     Before : constant Integer := Integer (Began) - 1;
                  begin
                     --  Settled before the push, or moving no more than the eye does at rest (Moved: no more of its
                     --  cells than its resting beats move): a body that sways as it stands never settles by the
                     --  picture's rule, and the G1's right arm, pushed at 19 beats, had 2 rows in its own wrist eye
                     --  (8 unsettled before), its eye undecided, and the wrist eye rode its hand (rec315 g12a, 10-25:
                     --  its head eye moving 252 of 255 cells at rest).
                     if Before < 0 or else Before >= Natural (S.Settled_At.Length)
                       or else not (S.Settled_At (Before) or else not Moved (M, E, Before))
                     then
                        return Unsettled_Before;
                     end if;
                     --  Nor was another group pushed at any beat of the push or
                     --  within the eye's settling tail before it, nor did it move
                     --  in that tail: its picture goes on changing for beats after
                     --  its readings stop, and the lock-in would credit that to
                     --  this push (A62 and A63,
                     --  the arms swept at once: arm 2 took eye 2, arm 1's, as a
                     --  whole; with the push's own beats only, A79 to A85 still
                     --  gave arm 2 eye 2 WHOLE after their sweeps, A85's with 240
                     --  of 525 cells, and A85's boot ended in a breach).
                     declare
                        Low : constant Natural := Natural (Integer'Max (0, Integer (Began) - Tail));
                     begin
                        for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                           if G /= Pusher and then M.Groups (G).Commandable then
                              if Mark (Last_Pushed, G, Natural (R)) > Low then
                                 return Another_Pushed;
                              elsif Began > Low and then Mark (Last_Moved, G, Began - 1) > Low then
                                 return Another_Moved_Before;
                              elsif Mark (Last_Seen_Moving, G, Natural (R)) > Began
                                and then (not Own_Pushes (G, Natural (R)) or else Keeps_Going (M, G))
                              then
                                 return Another_Moving;
                              end if;
                           end if;
                        end loop;
                     end;
                     return Row;
                  end;
               end if;
               --  A beat that no group pushes is at rest only once the eye's picture has settled since the body last
               --  moved (the stillness watch): the picture goes on changing after the readings stop, and a row of
               --  motion with no push is noise in the regression. The sweep's pushes left the arm's own eye settling
               --  for up to 31 beats after each, the cells' noise grew with it, and the eye read as showing a patch
               --  (f15j on 10-15). Asked beat by beat, not as the stream's longest settling: one stretch of a hand
               --  creeping on the table made that 1893 beats (f15f), and would have left out every rest after it.
               return (if B < Natural (S.Settled_At.Length) and then S.Settled_At (B) then Row else Unsettled);
            end Kind_Of;

            function Usable (B : Natural) return Boolean is (Kind_Of (B) = Row);
         begin
            S.Settling_Tail := Tail;
            S.Noise.Clear;
            S.Textured.Clear;
            S.Kept_Groups.Clear;
            S.Kept_Channels.Clear;
            S.Gains.Clear;
            S.Gain_Variances.Clear;
            S.Gain_Totals.Clear;
            S.Spread_Totals.Clear;
            S.Shifts.Clear;
            S.Shift_U.Clear;
            S.Shift_V.Clear;
            S.Col_Responds.Clear;
            if N > 0 then
               Mark_Motions;
               Mark_Reach;
               for B in 1 .. M.Beats - 1 loop
                  if Usable (B) then
                     Rows := Rows + 1;
                  end if;
               end loop;
               Marks_Took := Driver.Clock.Seconds - Eye_Began;
            end if;
            --  How many beats each group was pushed at and how many of them are rows (Usable: one group pushed, the eye
            --  settled before it, no other group pushed or moving in its tail): the Shadow's arm, recognized after its hand,
            --  had no column in either eye (round 4a, 10-23). Told before the regression is asked, which too few rows do not
            --  get to (the Shadow's seed 3, no regression in a whole boot, paced rounds p69 to p78).
            for G in M.Groups.First_Index .. M.Groups.Last_Index loop
               if M.Groups (G).Commandable then
                  declare
                     Pushed_Beats : Natural := 0;
                     Kinds        : array (Row_Kind) of Natural := [others => 0];
                     Why          : Ada.Strings.Unbounded.Unbounded_String;
                  begin
                     for B in 1 .. M.Beats - 1 loop
                        declare
                           R : constant Integer := B - Lag;
                        begin
                           if R >= 1 and then Channels.Pushed (M, G, R) then
                              Pushed_Beats := Pushed_Beats + 1;
                              Kinds (Kind_Of (B)) := Kinds (Kind_Of (B)) + 1;
                           end if;
                        end;
                     end loop;
                     for K in Row_Kind range No_Picture .. Row_Kind'Last loop
                        if Kinds (K) > 0 then
                           Ada.Strings.Unbounded.Append (Why, ", " & K'Image & Kinds (K)'Image);
                        end if;
                     end loop;
                     if Pushed_Beats > 0 then
                        Driver.Log.Line (Driver.Log.Robot, "lock-in: eye" & E'Image & ": group" & G'Image
                                         & " pushed at" & Pushed_Beats'Image & " beats," & Kinds (Row)'Image
                                         & " of them rows" & Ada.Strings.Unbounded.To_String (Why));
                     end if;
                  end;
               end if;
            end loop;
            --  A regression needs more observations than coefficients: the channels pushed over its rows, and the mean
            --  (below, Kept). Not every commandable channel, pushed or not: asked for the Shadow's 26 rows and more, the
            --  lock-in measured nothing in a whole boot of its arm's and its hand's pushes, every group unmeasured in both
            --  eyes and the hand unclassified, nor in one of RB-Y1's (seed 3 of each, paced round p69a, 10-23).
            if Rows > 0 then
               declare
                  Beat_Of : Natural_Access := new Natural_Array (1 .. Rows);
                  Cols    : Column_Array (1 .. All_Columns);
                  Kept    : Natural := 0;
               begin
                  declare
                     R : Natural := 0;
                  begin
                     for B in 1 .. M.Beats - 1 loop
                        if Usable (B) then
                           R := R + 1;
                           Beat_Of (R) := B;
                        end if;
                     end loop;
                  end;
                  --  Channels that never changed over these beats tell nothing.
                  for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                     if M.Groups (G).Commandable then
                        for C in 1 .. M.Groups (G).Size loop
                           declare
                              Moved : Boolean := False;
                           begin
                              for R in 1 .. Rows loop
                                 if Cause_Change (G, Beat_Of (R) - Lag, C) /= 0.0 then
                                    Moved := True;
                                    exit;
                                 end if;
                              end loop;
                              if Moved then
                                 Kept := Kept + 1;
                                 Cols (Kept) := (Group => G, Channel => C);
                              end if;
                           end;
                        end loop;
                     end if;
                  end loop;
                  --  More rows than the coefficients they can tell apart (the rank of the regressors' X'X, the mean's
                  --  among them, Regression.Gram_Rank): the channels of a group always pushed together are one coefficient
                  --  however many they are. Counted one a channel, the Shadow's arm and hand asked for 28 rows, and the
                  --  lock-in measured nothing in a whole boot (seed 3, paced rounds p69a and p72a, 10-23).
                  if Kept > 0 then
                     declare
                        A : Real_Matrix (1 .. Kept + 1, 1 .. Kept + 1) := [others => [others => 0.0]];
                        X : Real_Array (1 .. Kept + 1);
                     begin
                        for R in 1 .. Rows loop
                           X (1) := 1.0;
                           for K in 1 .. Kept loop
                              X (K + 1) := Cause_Change (Cols (K).Group, Beat_Of (R) - Lag, Cols (K).Channel);
                           end loop;
                           for I in X'Range loop
                              for J in X'Range loop
                                 A (I, J) := A (I, J) + X (I) * X (J);
                              end loop;
                           end loop;
                        end loop;
                        if Rows <= Regression.Gram_Rank (A) then
                           Kept := 0;
                        end if;
                     end;
                  end if;
                  for K in 1 .. Kept loop
                     S.Kept_Groups.Append (Natural (Cols (K).Group));
                     S.Kept_Channels.Append (Cols (K).Channel);
                  end loop;
                  if Kept > 0 then
                     declare
                        X : Matrix_Access := new Real_Matrix (1 .. Rows, 1 .. Kept + 1);
                        --  The first pass of every cell that resolved every row, at equal weights: theirs is X's
                        --  alone (Regression.Start_Of), made once X is.
                        All_Rows : Start_Access := null;
                        Responding : Flag_Grid_Access :=
                          new Flag_Grid'[1 .. N => [M.Groups.First_Index .. M.Groups.Last_Index => False]];
                        --  What each cell's block of each group's columns measured: the energy of the
                        --  motion it saw (Cells_Showing) and the degrees of freedom of that test.
                        Energy : Real_Grid_Access :=
                          new Real_Grid'[1 .. N => [M.Groups.First_Index .. M.Groups.Last_Index => 0.0]];
                        Dof : Natural_Grid_Access :=
                          new Natural_Grid'[1 .. N => [M.Groups.First_Index .. M.Groups.Last_Index => 0]];
                        --  The same for each column alone (the grids' second index is the column, not a group).
                        Col_Responding : Flag_Grid_Access :=
                          new Flag_Grid'[1 .. N => [Group_Id (1) .. Group_Id (Kept) => False]];
                        Col_Energy : Real_Grid_Access :=
                          new Real_Grid'[1 .. N => [Group_Id (1) .. Group_Id (Kept) => 0.0]];
                        Col_Dof : Natural_Grid_Access :=
                          new Natural_Grid'[1 .. N => [Group_Id (1) .. Group_Id (Kept) => 0]];
                        Textured : Flags_Access := new Flags'[1 .. N => False];
                        --  Each cell's results, cell by cell: its noise, and for each regressor its gain, the gain's
                        --  variance and its shift (nothing, where it does not respond). The cells are fitted at once
                        --  and write only their own; the eye's vectors get them in order afterwards.
                        Noise_Of  : Real_Access := new Real_Array'(1 .. N => Real'Last);
                        --  What each cell's two fits cost, for the log: their reweighting passes, and whether one
                        --  stopped short of converging.
                        Passes_Of : Natural_Access := new Natural_Array'(1 .. N => 0);
                        Stuck_Of  : Flags_Access := new Flags'[1 .. N => False];
                        Gain_Of   : Real_Access := new Real_Array'(1 .. N * Kept => 0.0);
                        Spread_Of : Real_Access := new Real_Array'(1 .. N * Kept => 0.0);
                        Shift_Of  : Real_Access := new Real_Array'(1 .. N * Kept => 0.0);
                        --  Every textured cell's signed coefficients for each regressor, and their variances
                        --  (zero where the cell has none): the cells that do not respond to a group are pooled
                        --  on them (Rest_Pull).
                        Beta_U_Of : Real_Access := new Real_Array'(1 .. N * Kept => 0.0);
                        Beta_V_Of : Real_Access := new Real_Array'(1 .. N * Kept => 0.0);
                        Var_U_Of  : Real_Access := new Real_Array'(1 .. N * Kept => 0.0);
                        Var_V_Of  : Real_Access := new Real_Array'(1 .. N * Kept => 0.0);
                        --  One cell: its displacements regressed on the pushes, over
                        --  the beats where the cell resolved one, and for every
                        --  group whether its block responds.
                        procedure Fit_Resolved (Cell : Positive; Here : Positive) is
                           Xc     : Matrix_Access := new Real_Matrix (1 .. Here, 1 .. Kept + 1);
                           U      : Real_Access := new Real_Array (1 .. Here);
                           V      : Real_Access := new Real_Array (1 .. Here);
                           Floors : Real_Access := new Real_Array (1 .. Here);
                           K      : Natural := 0;
                        begin
                           for R in 1 .. Rows loop
                              if S.Resolved (Beat_Of (R) * N + Cell - 1) then
                                 K := K + 1;
                                 for J in 1 .. Kept + 1 loop
                                    Xc (K, J) := X (R, J);
                                 end loop;
                                 U (K) := S.Du (Beat_Of (R) * N + Cell - 1);
                                 V (K) := S.Dv (Beat_Of (R) * N + Cell - 1);
                                 Floors (K) := Flow.Noise_Floor (S.Condition (Beat_Of (R) * N + Cell - 1),
                                                                 S.Luma_Variance (Cell - 1));
                              end if;
                           end loop;
                           if Cell = 1 or else Cell = N or else Cell = N / 2 then
                              Driver.Log.Line (Driver.Log.Robot, "lock-in: eye" & E'Image & " cell" & Cell'Image & " regresses"
                                               & K'Image & " of" & Rows'Image & " rows (resolved there), floor "
                                               & Driver.Log.Image (Driver.Stats.Median (Floors.all), 5));
                           end if;
                           declare
                              Floor : constant Real := Driver.Stats.Median (Floors.all);
                              --  A cell that resolved every row regresses on X itself, whose first pass is made once
                              --  for all of them (All_Rows); a cell's two fits share their first pass either way.
                              Mine   : Start_Access :=
                                (if Here = Rows then null else new Regression.Start'(Regression.Start_Of (Xc.all)));
                              Pass_1 : constant Start_Access := (if Mine = null then All_Rows else Mine);
                              Fu    : constant Regression.Fit := Regression.Solve (Xc.all, U.all, Floor, Pass_1.all);
                              Fv    : constant Regression.Fit := Regression.Solve (Xc.all, V.all, Floor, Pass_1.all);
                              --  Each fit's covariance, made once for all its tests below (Regression.Covariance_Of).
                              Cov_U : constant Real_Matrix := Regression.Covariance_Of (Fu);
                              Cov_V : constant Real_Matrix := Regression.Covariance_Of (Fv);
                           begin
                              Noise_Of (Cell) := Sqrt ((Fu.Scale ** 2 + Fv.Scale ** 2) / 2.0);
                              Passes_Of (Cell) := Fu.Passes + Fv.Passes;
                              Stuck_Of (Cell) := not (Fu.Converged and then Fv.Converged);
                              for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                                 declare
                                    First : Natural := 0;
                                    Last  : Natural := 0;
                                 begin
                                    for K in 1 .. Kept loop
                                       if Cols (K).Group = G then
                                          if First = 0 then
                                             First := K + 1;
                                          end if;
                                          Last := K + 1;
                                       end if;
                                    end loop;
                                    if First > 0 then
                                       declare
                                          Su, Sv : Real;
                                          Ku, Kv : Natural;
                                       begin
                                          Regression.Test_Block (Fu, Cov_U, First, Last, Su, Ku);
                                          Regression.Test_Block (Fv, Cov_V, First, Last, Sv, Kv);
                                          Responding (Cell, G) :=
                                            Ku + Kv > 0
                                            and then Driver.Distributions.Chi_Square_Deviate (Su + Sv, Ku + Kv)
                                                       > Driver.Conventions.Z;
                                          --  A statistic exceeds its degrees of freedom by the
                                          --  motion's energy over the variance, on average.
                                          Energy (Cell, G) := (Su - Real (Ku)) * Fu.Scale ** 2 + (Sv - Real (Kv)) * Fv.Scale ** 2;
                                          Dof (Cell, G) := Ku + Kv;
                                       end;
                                    end if;
                                 end;
                              end loop;
                              --  Each column alone: whether this cell's displacement follows that channel, and the
                              --  energy and degrees of freedom of the test.
                              for K in 1 .. Kept loop
                                 declare
                                    Su, Sv : Real;
                                    Ku, Kv : Natural;
                                    Col    : constant Group_Id := Group_Id (K);
                                 begin
                                    Regression.Test_Block (Fu, Cov_U, K + 1, K + 1, Su, Ku);
                                    Regression.Test_Block (Fv, Cov_V, K + 1, K + 1, Sv, Kv);
                                    Col_Responding (Cell, Col) :=
                                      Ku + Kv > 0
                                      and then Driver.Distributions.Chi_Square_Deviate (Su + Sv, Ku + Kv) > Driver.Conventions.Z;
                                    Col_Energy (Cell, Col) := (Su - Real (Ku)) * Fu.Scale ** 2 + (Sv - Real (Kv)) * Fv.Scale ** 2;
                                    Col_Dof (Cell, Col) := Ku + Kv;
                                 end;
                              end loop;
                              --  Each regressor's gain in this cell: its displacement per
                              --  reading unit in units of the cell's noise, squared, less
                              --  what estimating it adds on average (its own variance in
                              --  those units), where the cell responds to its group.
                              declare
                                 Var_U : constant Real_Array := Regression.Coefficient_Variances (Fu, Cov_U);
                                 Var_V : constant Real_Array := Regression.Coefficient_Variances (Fv, Cov_V);
                              begin
                                 for K in 1 .. Kept loop
                                    if Var_U (K + 1) > 0.0 and then Var_V (K + 1) > 0.0
                                      and then Var_U (K + 1) < Real'Last and then Var_V (K + 1) < Real'Last
                                    then
                                       Beta_U_Of ((Cell - 1) * Kept + K) := Fu.Beta (K + 1);
                                       Beta_V_Of ((Cell - 1) * Kept + K) := Fv.Beta (K + 1);
                                       Var_U_Of ((Cell - 1) * Kept + K) := Var_U (K + 1);
                                       Var_V_Of ((Cell - 1) * Kept + K) := Var_V (K + 1);
                                    end if;
                                    if Responding (Cell, Cols (K).Group) and then Fu.Scale > 0.0 and then Fv.Scale > 0.0 then
                                       declare
                                          Bu : constant Real := Fu.Beta (K + 1) / Fu.Scale;
                                          Bv : constant Real := Fv.Beta (K + 1) / Fv.Scale;
                                          Gu : constant Real := Var_U (K + 1) / Fu.Scale ** 2;
                                          Gv : constant Real := Var_V (K + 1) / Fv.Scale ** 2;
                                       begin
                                          Gain_Of ((Cell - 1) * Kept + K) := Bu ** 2 + Bv ** 2 - Gu - Gv;
                                          Spread_Of ((Cell - 1) * Kept + K) := 4.0 * (Gu * Bu ** 2 + Gv * Bv ** 2);
                                          Shift_Of ((Cell - 1) * Kept + K) := Sqrt (Fu.Beta (K + 1) ** 2 + Fv.Beta (K + 1) ** 2);
                                       end;
                                    end if;
                                 end loop;
                              end;
                              Free (Mine);
                           end;
                           Free (Xc);
                           Free (U);
                           Free (V);
                           Free (Floors);
                        end Fit_Resolved;

                        procedure Fit_Cell (Cell : Positive) is
                           Cond : Real_Access := new Real_Array (1 .. Rows);
                           Here : Natural := 0;
                        begin
                           for R in 1 .. Rows loop
                              Cond (R) := S.Condition (Beat_Of (R) * N + Cell - 1);
                              if S.Resolved (Beat_Of (R) * N + Cell - 1) then
                                 Here := Here + 1;
                              end if;
                           end loop;
                           --  A cell can show a displacement when it has texture in two
                           --  directions, and it is tested when it resolved more
                           --  displacements than the fit has coefficients.
                           Textured (Cell) := Driver.Stats.Median (Cond.all) > 0.0 and then Here > Kept + 1;
                           Free (Cond);
                           if Textured (Cell) then
                              Fit_Resolved (Cell, Here);
                           end if;
                        end Fit_Cell;

                        --  The cells are fitted at once, each processor taking the next cell nobody has: a cell's
                        --  fit takes as many rounds as its weights need to settle, and some take many more.
                        procedure Fit_Cells is new Driver.Shared_For (Fit_Cell);
                     begin
                        for R in 1 .. Rows loop
                           X (R, 1) := 1.0;
                           for K in 1 .. Kept loop
                              X (R, K + 1) := Cause_Change (Cols (K).Group, Beat_Of (R) - Lag, Cols (K).Channel);
                           end loop;
                        end loop;
                        All_Rows := new Regression.Start'(Regression.Start_Of (X.all));
                        declare
                           Motion_Rows : Natural := 0;
                        begin
                           for R in 1 .. Rows loop
                              if (for some K in 1 .. Kept => X (R, K + 1) /= 0.0) then
                                 Motion_Rows := Motion_Rows + 1;
                              end if;
                           end loop;
                           Driver.Log.Line (Driver.Log.Robot, "lock-in: eye" & E'Image & ":" & Motion_Rows'Image & " of" & Rows'Image
                                            & " rows carry a push; the rows' beats run" & (if Rows > 0 then Beat_Of (1)'Image else "")
                                            & " to" & (if Rows > 0 then Beat_Of (Rows)'Image else "") & ", lag" & Lag'Image);
                        end;
                        declare
                           Started : constant Duration := Driver.Clock.Seconds;
                           Fitted, Passes, Most, Stuck : Natural := 0;
                        begin
                           Rows_Took := Started - Eye_Began - Marks_Took;
                           Fit_Cells (1, N);
                           for Cell in 1 .. N loop
                              if Textured (Cell) then
                                 Fitted := Fitted + 1;
                                 Passes := Passes + Passes_Of (Cell);
                                 Most := Natural'Max (Most, Passes_Of (Cell));
                                 Stuck := Stuck + Boolean'Pos (Stuck_Of (Cell));
                              end if;
                           end loop;
                           Driver.Log.Line
                             (Driver.Log.Robot, "lock-in: eye" & E'Image & ":" & Rows'Image & " rows," & Kept'Image
                              & " columns, settling tail" & Tail'Image & " beats," & Fitted'Image & " of" & N'Image
                              & " cells fitted in "
                              & Driver.Log.Image (Real (Driver.Clock.Seconds - Started), 1) & " s (the marks took "
                              & Driver.Log.Image (Real (Marks_Took), 1) & " s, the rows "
                              & Driver.Log.Image (Real (Rows_Took), 1) & " s before them), with" & Passes'Image
                              & " reweighting passes in all," & Most'Image & " the most in a cell," & Stuck'Image
                              & " cells with a fit that did not converge");
                        end;
                        Free (Passes_Of);
                        Free (Stuck_Of);
                        for Cell in 1 .. N loop
                           S.Noise.Append (Noise_Of (Cell));
                        end loop;
                        for I in 1 .. N * Kept loop
                           S.Gains.Append (Gain_Of (I));
                           S.Gain_Variances.Append (Spread_Of (I));
                           S.Shifts.Append (Shift_Of (I));
                           --  Which way it moved, where it responds (Shift_Of is nought where it does not).
                           S.Shift_U.Append (if Shift_Of (I) > 0.0 then Beta_U_Of (I) else 0.0);
                           S.Shift_V.Append (if Shift_Of (I) > 0.0 then Beta_V_Of (I) else 0.0);
                           S.Col_Responds.Append (Col_Responding ((I - 1) / Kept + 1, Group_Id ((I - 1) mod Kept + 1)));
                        end loop;
                        Total_Gains (S);
                        Free (Noise_Of);
                        Free (Gain_Of);
                        Free (Spread_Of);
                        Free (Shift_Of);
                        for Cell in 1 .. N loop
                           S.Textured.Append (Textured (Cell));
                        end loop;
                        declare
                           T : Natural := 0;
                        begin
                           for Cell in 1 .. N loop
                              if Textured (Cell) then
                                 T := T + 1;
                              end if;
                           end loop;
                           for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                              declare
                                 Tested : Boolean := False;
                                 Count  : Natural := 0;
                                 Effect : Eye_Effect;
                              begin
                                 for K in 1 .. Kept loop
                                    Tested := Tested or else Cols (K).Group = G;
                                 end loop;
                                 if Tested and then T > 0 then
                                    for Cell in 1 .. N loop
                                       if Textured (Cell) and then Responding (Cell, G) then
                                          Count := Count + 1;
                                       end if;
                                    end loop;
                                    declare
                                       F    : constant Real := Real (Count) / Real (T);
                                       --  The cells that show the motion, at least and at most:
                                       --  those that respond, and the share of the cells too noisy
                                       --  to tell that show it together.
                                       Least, Most : Natural;
                                       Disagree    : Boolean;
                                       Resting     : Real;
                                    begin
                                       Cells_Showing (G, Textured.all, Responding.all, Energy.all, Dof.all, S.Noise,
                                                      Least, Most, Disagree, Resting);
                                       Effect.Resting := Resting;
                                       --  The cells that did not respond, pulled together along the push: for each
                                       --  of the group's regressors, their signed coefficients pooled by inverse
                                       --  variance (a mean gain over its standard error, in each picture direction,
                                       --  a chi-square of two degrees of freedom), the most significant regressor's.
                                       --  A far part of the view moved with the push by less than any one cell can
                                       --  tell shows as a consistent pull (every cell's gain the same sign); a base
                                       --  shaken by the push moves the cells either way and pools to nothing. Their
                                       --  pooled ENERGY (Resting) grows with any motion whatever moves them, which is
                                       --  why it was dropped from the boot's rounds (R5 on the Pi, 10-13: the head
                                       --  eye on a shaken base was pushed to 128 times the amounts).
                                       Effect.Rest_Pull := 0.0;
                                       for K in 1 .. Kept loop
                                          if Cols (K).Group = G then
                                             declare
                                                Pu, Wu, Pv, Wv : Real := 0.0;
                                             begin
                                                for Cell in 1 .. N loop
                                                   if Textured (Cell) and then not Responding (Cell, G)
                                                     and then Var_U_Of ((Cell - 1) * Kept + K) > 0.0
                                                     and then Var_V_Of ((Cell - 1) * Kept + K) > 0.0
                                                   then
                                                      Pu := Pu + Beta_U_Of ((Cell - 1) * Kept + K) / Var_U_Of ((Cell - 1) * Kept + K);
                                                      Wu := Wu + 1.0 / Var_U_Of ((Cell - 1) * Kept + K);
                                                      Pv := Pv + Beta_V_Of ((Cell - 1) * Kept + K) / Var_V_Of ((Cell - 1) * Kept + K);
                                                      Wv := Wv + 1.0 / Var_V_Of ((Cell - 1) * Kept + K);
                                                   end if;
                                                end loop;
                                                if Wu > 0.0 and then Wv > 0.0 then
                                                   Effect.Rest_Pull :=
                                                     Real'Max (Effect.Rest_Pull,
                                                               Driver.Distributions.Chi_Square_Deviate
                                                                 (Pu ** 2 / Wu + Pv ** 2 / Wv, 2));
                                                end if;
                                             end;
                                          end if;
                                       end loop;
                                       Effect.Responding := Count;
                                       Effect.Textured := T;
                                       Effect.Fraction :=
                                         (Value => F, Sigma => Sqrt (F * (1.0 - F) / Real (T)), Degrees_Of_Freedom => 0);
                                       Effect.Verdict := Judge (Count, T, Least, Most, Disagree);
                                       Effect.Could_Be_Whole := Regression.Count_Significant (Most, T, Half);
                                       Driver.Log.Line (Driver.Log.Robot, "lock-in: eye" & E'Image & " group" & G'Image & ":"
                                                        & Count'Image & " of" & T'Image & " textured cells respond, "
                                                        & Least'Image & " to" & Most'Image & " show the motion"
                                                        & (if Disagree then " and the cells that can tell disagree" else "")
                                                        & ": " & Effect.Verdict'Image
                                                        --  And how the cells that do not respond pull along with it,
                                                        --  pooled: the Shadow's hand filled 192 of its wrist eye's 247
                                                        --  textured cells and was judged to move the view whole, and
                                                        --  the line said nothing of the other 55 (n9s2, 10-26).
                                                        & " (the rest pull along by "
                                                        & Driver.Log.Image (Effect.Rest_Pull, 2) & ")");
                                    end;
                                 end if;
                                 M.Graph.Effects.Replace_Element ((Natural (G) - 1) * Eyes + Natural (E), Effect);
                              end;
                           end loop;
                           --  A cell that shows another group's motion and not this group's, where that other group
                           --  moves a patch of this eye's view, is a thing carried along with the eye when this group
                           --  moves it: the hand before a wrist eye, which the arm's push leaves where it stands in the
                           --  picture while its closer moves it, or an arm before a head eye on a base. It witnesses
                           --  nothing against this group moving the whole view. A group judged otherwise with such
                           --  cells among its witnesses is judged again without them: made whole, or left as it was.
                           --  The six-value groups of the race's line 1 (10-23, tables and kiwis, 626 of them): the
                           --  arms whose own eye was left undecided had 52 to 78% of its textured cells respond, and
                           --  the cells that can tell disagreed; those arms were taken for parts and their eyes for
                           --  fixed ones. The patches are the first judgement's, so no group's order among them counts.
                           declare
                              Patch_Of : array (M.Groups.First_Index .. M.Groups.Last_Index) of Boolean :=
                                [others => False];
                           begin
                              for H in Patch_Of'Range loop
                                 Patch_Of (H) := M.Graph.Effects ((Natural (H) - 1) * Eyes + Natural (E)).Verdict = Patch;
                              end loop;
                              for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                                 declare
                                    Effect : Eye_Effect := M.Graph.Effects ((Natural (G) - 1) * Eyes + Natural (E));
                                 begin
                                    if Effect.Verdict in Undecided | Patch and then Effect.Textured > 0 then
                                       declare
                                          Own         : Flags_Access := new Flags'(Textured.all);
                                          Left_Out    : Natural := 0;
                                          Own_Count   : Natural := 0;
                                          Count       : Natural := 0;
                                          Least, Most : Natural;
                                          Disagree    : Boolean;
                                          Resting     : Real;
                                       begin
                                          for Cell in 1 .. N loop
                                             if Own (Cell) and then not Responding (Cell, G)
                                               and then (for some H in Patch_Of'Range =>
                                                           H /= G and then Patch_Of (H) and then Responding (Cell, H))
                                             then
                                                Own (Cell) := False;
                                                Left_Out := Left_Out + 1;
                                             end if;
                                             if Own (Cell) then
                                                Own_Count := Own_Count + 1;
                                                Count := Count + Boolean'Pos (Responding (Cell, G));
                                             end if;
                                          end loop;
                                          if Left_Out > 0 and then Own_Count > 0 then
                                             Cells_Showing (G, Own.all, Responding.all, Energy.all, Dof.all, S.Noise,
                                                            Least, Most, Disagree, Resting, Quiet => True);
                                             if Judge (Count, Own_Count, Least, Most, Disagree) = Whole then
                                                declare
                                                   F : constant Real := Real (Count) / Real (Own_Count);
                                                begin
                                                   Effect.Verdict := Whole;
                                                   Effect.Responding := Count;
                                                   Effect.Textured := Own_Count;
                                                   Effect.Fraction :=
                                                     (Value => F, Sigma => Sqrt (F * (1.0 - F) / Real (Own_Count)),
                                                      Degrees_Of_Freedom => 0);
                                                   Effect.Could_Be_Whole := True;
                                                end;
                                                M.Graph.Effects.Replace_Element
                                                  ((Natural (G) - 1) * Eyes + Natural (E), Effect);
                                                Driver.Log.Line
                                                  (Driver.Log.Robot, "lock-in: eye" & E'Image & " group" & G'Image
                                                   & ": WHOLE without the" & Left_Out'Image & " cells that move with the"
                                                   & " patches of other groups alone:" & Count'Image & " of"
                                                   & Own_Count'Image & " textured cells respond, " & Least'Image & " to"
                                                   & Most'Image & " show the motion");
                                             end if;
                                          end if;
                                          Free (Own);
                                       end;
                                    end if;
                                 end;
                              end loop;
                           end;
                           --  And of each channel alone, as of a group (the same tests of the column's cells).
                           for K in 1 .. Kept loop
                              declare
                                 Col    : constant Group_Id := Group_Id (K);
                                 Count  : Natural := 0;
                                 Effect : Eye_Effect;
                              begin
                                 if T > 0 then
                                    for Cell in 1 .. N loop
                                       if Textured (Cell) and then Col_Responding (Cell, Col) then
                                          Count := Count + 1;
                                       end if;
                                    end loop;
                                    declare
                                       F           : constant Real := Real (Count) / Real (T);
                                       Least, Most : Natural;
                                       Disagree    : Boolean;
                                       Resting     : Real;
                                    begin
                                       Cells_Showing (Col, Textured.all, Col_Responding.all, Col_Energy.all, Col_Dof.all, S.Noise,
                                                      Least, Most, Disagree, Resting, Quiet => True);
                                       Effect.Resting := Resting;
                                       Effect.Responding := Count;
                                       Effect.Textured := T;
                                       Effect.Fraction :=
                                         (Value => F, Sigma => Sqrt (F * (1.0 - F) / Real (T)), Degrees_Of_Freedom => 0);
                                       Effect.Verdict := Judge (Count, T, Least, Most, Disagree);
                                       Effect.Could_Be_Whole := Regression.Count_Significant (Most, T, Half);
                                    end;
                                 end if;
                                 M.Graph.Channels (E).Append
                                   (Channel_Effect'(Group => Cols (K).Group, Channel => Cols (K).Channel, Effect => Effect));
                              end;
                           end loop;
                           declare
                              Whole_List : Ada.Strings.Unbounded.Unbounded_String;
                           begin
                              for Item of M.Graph.Channels (E) loop
                                 if Item.Effect.Verdict = Whole then
                                    Ada.Strings.Unbounded.Append
                                      (Whole_List, " g" & Ada.Strings.Fixed.Trim (Item.Group'Image, Ada.Strings.Left) & "c"
                                       & Ada.Strings.Fixed.Trim (Item.Channel'Image, Ada.Strings.Left));
                                 end if;
                              end loop;
                              Driver.Log.Line (Driver.Log.Robot, "lock-in: eye" & E'Image & ": of the" & Kept'Image
                                               & " channels kept, the view moves whole with"
                                               & (if Ada.Strings.Unbounded.Length (Whole_List) = 0 then " none"
                                                  else Ada.Strings.Unbounded.To_String (Whole_List)));
                           end;
                        end;
                        Free (X);
                        Free (All_Rows);
                        Free (Responding);
                        Free (Energy);
                        Free (Dof);
                        Free (Col_Responding);
                        Free (Col_Energy);
                        Free (Col_Dof);
                        Free (Textured);
                        Free (Beta_U_Of);
                        Free (Beta_V_Of);
                        Free (Var_U_Of);
                        Free (Var_V_Of);
                     end;
                  end if;
                  Free (Beat_Of);
               end;
            end if;
            --  Beyond the flow's reach: how likely rest alone put as many of a group's pushed beats' pictures farther than
            --  the cells resolve (Reached) as there were (Far_Chance), at rest's rate of such beats where no group was pushed
            --  or moved, by the rule of succession (one beat of each kind added: never zero before it is seen). The regression
            --  measures small motions and leaves such beats out: Stretch's base, a speed whose pushes swung its wrist
            --  eye's view beyond reach at 16 of its 24 beats, was in no row of either eye, and its ladder found it moving
            --  no eye up to 7 units and probed its channels one at a time to the boot's end (seed 3, paced round p70a,
            --  10-23).
            declare
               Rest_Beats, Rest_Far : Natural := 0;
               function Pictured (B : Natural) return Boolean is
                 (B - Lag >= 1 and then B - Lag < M.Beats and then B < Natural (S.Measured.Length) and then S.Measured (B));
            begin
               for B in 1 .. M.Beats - 1 loop
                  if Pictured (B)
                    and then (for all G in M.Groups.First_Index .. M.Groups.Last_Index =>
                                not M.Groups (G).Commandable
                                or else not (Channels.Pushed (M, G, B - Lag) or else Group_Moved (G, B - Lag)))
                  then
                     Rest_Beats := Rest_Beats + 1;
                     if not Reached (B + 1) then
                        Rest_Far := Rest_Far + 1;
                     end if;
                  end if;
               end loop;
               for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                  if M.Groups (G).Commandable then
                     declare
                        Push_Beats, Push_Far : Natural := 0;
                        Index : constant Positive := (Natural (G) - 1) * Eyes + Natural (E);
                        Effect : Eye_Effect := M.Graph.Effects (Index);
                     begin
                        for B in 1 .. M.Beats - 1 loop
                           if Pictured (B) and then Channels.Pushed (M, G, B - Lag) then
                              Push_Beats := Push_Beats + 1;
                              if not Reached (B + 1) then
                                 Push_Far := Push_Far + 1;
                              end if;
                           end if;
                        end loop;
                        Effect.Far_Chance :=
                          (if Push_Beats > 0
                           then Regression.Count_Tail (Push_Far, Push_Beats, Real (Rest_Far + 1) / Real (Rest_Beats + 2))
                           else 1.0);
                        M.Graph.Effects.Replace_Element (Index, Effect);
                        if Regression.Count_Significant
                             (Push_Far, Push_Beats, Real (Rest_Far + 1) / Real (Rest_Beats + 2))
                        then
                           Driver.Log.Line (Driver.Log.Robot, "lock-in: eye" & E'Image & ": group" & G'Image
                                            & " moved the picture beyond the flow's reach at" & Push_Far'Image & " of its"
                                            & Push_Beats'Image & " pushed beats, rest at" & Rest_Far'Image & " of"
                                            & Rest_Beats'Image);
                        end if;
                     end;
                  end if;
               end loop;
            end;
            Free (Last_Pushed);
            Free (Last_Moved);
            Free (Last_Seen_Moving);
            Free (Reached);
         end;
      end loop;
      Free (Starts);
      Free (Moving_At);
      Free (Was_Shift);
      Free (Step_Table);
   end Measure;

   procedure Measure_Rest_Noise (M : in out Model) is
      Half_Chi : constant Real := Driver.Distributions.Chi_Square_Quantile (0.5, 2);
   begin
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            S     : Eye_Stream renames M.Eyes (E);
            N     : constant Natural := Cells (S.Grid);
            Beats : constant Natural := Natural'Min (M.Beats, Natural (S.Measured.Length));
            --  An eye whose lag is not measured may show a push as late as any
            --  lag the stream can tell; and its picture goes on settling for
            --  its settling tail after the body stops (Measure).
            Lag   : constant Natural :=
              (if E <= M.Lag_Known.Last_Index and then M.Lag_Known (E)
               then Natural (Integer'Max (0, M.Lags (E)))
               else Driver.Robot.Lag.Longest (M));
            Quiet : constant Natural := Lag + S.Settling_Tail;
            --  Per beat: nothing commandable was pushed within Quiet beats before it, through it, and the eye
            --  measured a displacement at it.
            Rest_At : Flags_Access := new Flags'(1 .. Natural'Max (1, Beats) => False);
            Rest    : Natural := 0;
         begin
            if N > 0 and then Natural (S.Luma_Variance.Length) >= N and then Natural (S.Condition.Length) >= N * Beats then
               for B in 1 .. Beats - 1 loop
                  if S.Measured (B) then
                     Rest_At (B) := True;
                     for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                        for K in Integer'Max (0, B - Quiet) .. B loop
                           if M.Groups (G).Commandable and then Channels.Pushed (M, G, K) then
                              Rest_At (B) := False;
                           end if;
                        end loop;
                     end loop;
                     if Rest_At (B) then
                        Rest := Rest + 1;
                     end if;
                  end if;
               end loop;
            end if;
            if Rest > 0 then
               declare
                  --  One ratio per resting beat of a cell: on the heap, a long stream has thousands.
                  Ratios  : Real_Access := new Real_Array (1 .. Rest);
                  Counts  : Real_Access := new Real_Array (1 .. Rest);
                  Of_Cell : Real_Access := new Real_Array'(1 .. N => 0.0);   --  0: the cell resolved nothing at rest
                  Typical : Real_Access := new Real_Array (1 .. N);
                  Known   : Natural := 0;
               begin
                  --  A cell's noise at rest is what it showed at rest: its displacements over the floor its
                  --  pixels' variance allows, the median of their squares against the median of a chi-square of
                  --  two degrees. Per cell, not one factor for the eye: a camera that quantizes its pictures
                  --  (JPEG) leaves a cell's flat pixels unchanged from frame to frame and moves its textured ones
                  --  by whole steps, so the pixels' variance (the median over the cell's pixels) says little of
                  --  what a textured cell does at rest, and one factor for the whole eye is the flat cells' (R2's
                  --  wrist eye on the Pi, 10-13: a hundred of 525 cells read as moved at every resting beat, the
                  --  most at rest came to 290, and no push of its own arm was ever seen).
                  for C in 0 .. N - 1 loop
                     declare
                        K : Natural := 0;
                     begin
                        for B in 1 .. Beats - 1 loop
                           if Rest_At (B) and then S.Condition (B * N + C) > 0.0 then
                              K := K + 1;
                              Ratios (K) := (S.Du (B * N + C) ** 2 + S.Dv (B * N + C) ** 2)
                                / Flow.Noise_Floor (S.Condition (B * N + C), S.Luma_Variance (C)) ** 2;
                           end if;
                        end loop;
                        if K > 0 then
                           Of_Cell (C + 1) := Real'Max (1.0, Sqrt (Driver.Stats.Median (Ratios (1 .. K)) / Half_Chi));
                           Known := Known + 1;
                           Typical (Known) := Of_Cell (C + 1);
                        end if;
                     end;
                  end loop;
                  if Known > 0 then
                     S.Rest_Factor := Driver.Stats.Median (Typical (1 .. Known));
                  end if;
                  S.Rest_Factors.Clear;
                  for C in 1 .. N loop
                     S.Rest_Factors.Append (if Of_Cell (C) > 0.0 then Of_Cell (C) else S.Rest_Factor);
                  end loop;
                  --  How many cells an eye counts as moved when nothing is pushed (its own fingers' jitter, the
                  --  scene, the rest of the body's noise): the most at any resting beat of the latest stationary
                  --  stretch of those counts, by the readings' own rule of rest (Stats.Latest_Regime): a camera
                  --  whose exposure was still settling when the stream began, or a resting beat something walked
                  --  through, goes with the half of the stretch it is in, instead of setting the most for good.
                  declare
                     K : Natural := 0;
                     Count, Tested : Natural;
                  begin
                     for B in 1 .. Beats - 1 loop
                        if Rest_At (B) then
                           K := K + 1;
                           Count_Moved (S, B, Count, Tested);
                           Counts (K) := Real (Count);
                        end if;
                     end loop;
                     declare
                        --  A count is a whole number: no scatter finer than the rounding to one. Counts mostly zero
                        --  left both halves' median absolute deviations at zero, any difference of their means
                        --  was significant, and the stretch was halved down to its last 4 and 7 beats (f15q_play on
                        --  10-15).
                        Start : constant Positive :=
                          Driver.Stats.Latest_Regime (Counts (1 .. K), Floor => Sqrt (Whole_Rounding));
                        Most  : Natural := 0;
                     begin
                        for I in Start .. K loop
                           Most := Natural'Max (Most, Natural (Counts (I)));
                        end loop;
                        S.Rest_Count_Max := Most;
                        S.Rest_Count_Beats := K - Start + 1;
                        S.Rest_Counts_Known := S.Rest_Count_Beats > 1;
                        S.Rest_Counts.Clear;
                        for I in Start .. K loop
                           S.Rest_Counts.Append (Natural (Counts (I)));
                        end loop;
                        Count_Sorting.Sort (S.Rest_Counts);
                        --  Motion is a count significantly above what the rest produces: the rest counts' median
                        --  plus Z of their robust scatter. The most of them is one beat's: a frame that flickered
                        --  once among 116 at rest put it at 273 of 525 cells for the kiwi's wrist eye, and a jaw
                        --  opening 1250 units moved 130 to 156 cells, unseen (R6, 10-14).
                        --  And no scatter below the root of the counts' mean: a count of cells each moved at rest by
                        --  chance is a sum of rare independent alarms, whose variance is its mean at the least.
                        --  Counts mostly zero leave their median absolute deviation at zero and the level at the
                        --  median, and a count of one or two at rest was motion (the fake's eyes looking at little
                        --  texture, f15q_play on 10-15: the boot's probe saw a push of 1.9e-7 units).
                        if S.Rest_Counts_Known then
                           declare
                              Sum : Real := 0.0;
                           begin
                              for I in Start .. K loop
                                 Sum := Sum + Counts (I);
                              end loop;
                              S.Rest_Count_Level :=
                                Driver.Stats.Median (Counts (Start .. K))
                                + Driver.Conventions.Z
                                  * Real'Max (Driver.Stats.Robust_Sigma (Counts (Start .. K)),
                                              Sqrt (Sum / Real (K - Start + 1)));
                              S.Rest_Count_Above := 0;
                              for I in Start .. K loop
                                 if Counts (I) > S.Rest_Count_Level then
                                    S.Rest_Count_Above := S.Rest_Count_Above + 1;
                                 end if;
                              end loop;
                           end;
                        end if;
                     end;
                  end;
                  Free (Ratios);
                  Free (Counts);
                  Free (Of_Cell);
                  Free (Typical);
               end;
            end if;
            Free (Rest_At);
         end;
      end loop;
   end Measure_Rest_Noise;

   --  How many textured cells moved beyond their noise at rest, or too far
   --  to be resolved, at the beat; how many were tested.
   procedure Count_Moved (S : Eye_Stream; Beat : Natural; Count, Tested : out Natural) is
      N    : constant Natural := Cells (S.Grid);
      Gate : constant Driver.Uncertain.Gate := Driver.Uncertain.Vector_Gate (2);
   begin
      Count := 0;
      Tested := 0;
      if N = 0 or else Beat >= Natural (S.Measured.Length) or else not S.Measured (Beat)
        or else Natural (S.Luma_Variance.Length) < N
      then
         return;
      end if;
      for C in 0 .. N - 1 loop
         declare
            Cond : constant Real := S.Condition (Beat * N + C);
         begin
            if Cond > 0.0 then
               Tested := Tested + 1;
               if not S.Resolved (Beat * N + C)
                 or else Driver.Uncertain.Significant
                   (Gate, Sqrt (S.Du (Beat * N + C) ** 2 + S.Dv (Beat * N + C) ** 2),
                    --  The cell's displacement noise at rest, as its own resting beats measured it.
                    Rest_Factor_Of (S, C) * Flow.Noise_Floor (Cond, S.Luma_Variance (C)))
               then
                  Count := Count + 1;
               end if;
            end if;
         end;
      end loop;
   end Count_Moved;

   function Moved (M : Model; E : Eye_Id; Beat : Natural) return Boolean is
      S : Eye_Stream renames M.Eyes (E);
      Count, Tested : Natural;
   begin
      Count_Moved (S, Beat, Count, Tested);
      if Tested = 0 then
         return False;
      end if;
      --  More than the counts at the beats nobody pushed (its own fingers' jitter, the scene) reach but rarely
      --  (Rest_Count_Level). Before those are measured, against the false alarms of the per-cell test alone.
      return (if S.Rest_Counts_Known
              then Real (Count) > S.Rest_Count_Level
              else Regression.Count_Significant
                     (Count, Tested, Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z)));
   end Moved;

   function Mostly (Count, Cells : Natural) return Boolean is (Regression.Count_Significant (Count, Cells, Half));

   function Rest_Share (M : Model; E : Eye_Id; Beat : Natural) return Real is
      S : Eye_Stream renames M.Eyes (E);
      Count, Tested : Natural;
      Reached : Natural := 0;
   begin
      Count_Moved (S, Beat, Count, Tested);
      if Tested = 0 then
         return 1.0;
      end if;
      if not S.Rest_Counts_Known or else S.Rest_Counts.Is_Empty then
         return Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
      end if;
      --  Least first: those from the first that reaches the count on.
      for I in reverse S.Rest_Counts.First_Index .. S.Rest_Counts.Last_Index loop
         exit when S.Rest_Counts (I) < Count;
         Reached := Reached + 1;
      end loop;
      return Real (Reached + 1) / Real (Natural (S.Rest_Counts.Length) + 1);
   end Rest_Share;

   function Cell_Noise (M : Model; E : Eye_Id) return Real is
      S     : Eye_Stream renames M.Eyes (E);
      Count : Natural := 0;
   begin
      for X of S.Noise loop
         if X < Real'Last then
            Count := Count + 1;
         end if;
      end loop;
      if Count = 0 then
         return Real'Last;
      end if;
      declare
         Values : Real_Access := new Real_Array (1 .. Count);
         K      : Natural := 0;
      begin
         for X of S.Noise loop
            if X < Real'Last then
               K := K + 1;
               Values (K) := X;
            end if;
         end loop;
         return Result : constant Real := Driver.Stats.Median (Values.all) do
            Free (Values);
         end return;
      end;
   end Cell_Noise;

   --  The arm that carries the eye moves its whole view, and its keyframes measure how far a unit of each channel moves
   --  it in the eye they are seen in (Kinematics.Pixels_Per_Unit), where the lock-in's flow, a beat at a time, reads
   --  that view hardly moving: s33 (servo seed 7 of the fake robot, 10-17) at beat 3217 had 0.0011 px a unit of the
   --  arm's first channel in its own eye from the lock-in, 0.31 from the keyframes; no raise of the eye within the
   --  arm's reach showed, its closer was seen from no second pose, and its hand was never made. The keyframes'
   --  measure, once they have one; the lock-in's before, and for every other group and eye.
   function Moving_Chance (M : Model; G : Group_Id) return Real is
      P0    : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
      Eyes  : constant Natural := Natural (M.Eyes.Length);
      Least : Real := 1.0;
      Tests : Natural := 0;
   begin
      for E in 1 .. Eyes loop
         if (Natural (G) - 1) * Eyes + E <= Natural (M.Graph.Effects.Length) then
            declare
               F : constant Eye_Effect := M.Graph.Effects ((Natural (G) - 1) * Eyes + E);
            begin
               if F.Verdict /= Unmeasured and then F.Textured > 0 then
                  Least := Real'Min (Least, Regression.Count_Tail (F.Responding, F.Textured, P0));
               end if;
               Least := Real'Min (Least, F.Far_Chance);
               Tests := Tests + 2;
               --  And each channel's gain summed over the eye's cells (Seen's test, on the channel's own column): a
               --  channel probed alone is confirmed by it (Motion.Probe_Both_Ways), and with the cells counted alone a
               --  group of RB-Y1's jaw's one channel was seen moved whole by no level and probed again one channel at a
               --  time, 216 beats, to be seen at the first (seed 1, paced rounds p83 and p84, 10-23).
               for C in 1 .. M.Groups (G).Size loop
                  declare
                     Gain, Spread : Real;
                     Measured     : Boolean;
                  begin
                     Gain_Of (M, Eye_Id (E), G, C, Gain, Spread, Measured);
                     if Measured and then Spread > 0.0 then
                        Tests := Tests + 1;
                        if Gain > 0.0 then
                           Least := Real'Min (Least, Driver.Distributions.Gaussian_Two_Sided_Tail (Gain / Sqrt (Spread)) / 2.0);
                        end if;
                     end if;
                  end;
               end loop;
            end;
         end if;
      end loop;
      return Real'Min (1.0, Least * Real (Natural'Max (1, Tests)));
   end Moving_Chance;

   function Shows_Step (M : Model; E : Eye_Id; G : Group_Id; Step : Real_Array) return Boolean is
      Noise   : constant Real := Cell_Noise (M, E);
      Mount_E : constant Mount := Eye_Mount (M, E);
      Carrier : constant Arm_Id'Base :=
        (if Mount_E.Kind = Arm_Carried and then Natural (Mount_E.Arm) <= Arm_Count (M)
           and then Arm_Group (M, Mount_E.Arm) = G and then Driver.Robot.Kinematics.Eye_Of (M, Mount_E.Arm) = E
         then Mount_E.Arm else 0);
   begin
      if Noise = Real'Last then
         return False;
      end if;
      for C in Step'Range loop
         declare
            Kept     : constant Real :=
              (if Carrier > 0 then Driver.Robot.Kinematics.Pixels_Per_Unit (M, Carrier, C - Step'First + 1) else 0.0);
            Per_Unit : constant Real := (if Kept > 0.0 then Kept else Shift (M, E, G, C - Step'First + 1));
         begin
            if Per_Unit > 0.0 and then Driver.Uncertain.Significant (Step (C) * Per_Unit, Noise) then
               return True;
            end if;
         end;
      end loop;
      return False;
   end Shows_Step;

   function Column_Of (S : Eye_Stream; G : Group_Id; Channel : Positive) return Natural is
      Kept : constant Natural := Natural (S.Kept_Groups.Length);
   begin
      for K in 0 .. Kept - 1 loop
         if S.Kept_Groups (K) = Natural (G) and then S.Kept_Channels (K) = Channel then
            return K + 1;
         end if;
      end loop;
      return 0;
   end Column_Of;

   procedure Total_Gains (S : in out Eye_Stream) is
      Kept : constant Natural := Natural (S.Kept_Groups.Length);
   begin
      S.Gain_Totals.Clear;
      S.Spread_Totals.Clear;
      for Column in 1 .. Kept loop
         declare
            Gain, Spread : Real := 0.0;
         begin
            for Cell in 0 .. Natural (S.Gains.Length) / Kept - 1 loop
               Gain := Gain + S.Gains (Cell * Kept + Column - 1);
               Spread := Spread + S.Gain_Variances (Cell * Kept + Column - 1);
            end loop;
            S.Gain_Totals.Append (Gain);
            S.Spread_Totals.Append (Spread);
         end;
      end loop;
   end Total_Gains;

   procedure Gain_Of
     (M : Model; E : Eye_Id; G : Group_Id; Channel : Positive; Gain, Spread : out Real; Measured : out Boolean)
   is
      S      : Eye_Stream renames M.Eyes (E);
      Kept   : constant Natural := Natural (S.Kept_Groups.Length);
      Column : constant Natural := Column_Of (S, G, Channel);
   begin
      Gain := 0.0;
      Spread := 0.0;
      Measured := Column > 0;
      if Measured then
         if Natural (S.Gain_Totals.Length) = Kept then
            Gain := S.Gain_Totals (Column - 1);
            Spread := S.Spread_Totals (Column - 1);
         else
            for Cell in 0 .. Natural (S.Gains.Length) / Kept - 1 loop
               Gain := Gain + S.Gains (Cell * Kept + Column - 1);
               Spread := Spread + S.Gain_Variances (Cell * Kept + Column - 1);
            end loop;
         end if;
      end if;
   end Gain_Of;

   function Seen (Gain, Spread : Real) return Boolean is
     (Gain > Driver.Conventions.Z * Sqrt (Real'Max (0.0, Spread)));

   function Seen (M : Model; E : Eye_Id; G : Group_Id; Channel : Positive) return Boolean is
      Gain, Spread : Real;
      Measured     : Boolean;
   begin
      Gain_Of (M, E, G, Channel, Gain, Spread, Measured);
      return Measured and then Seen (Gain, Spread);
   end Seen;

   function Shift (M : Model; E : Eye_Id; G : Group_Id; Channel : Positive) return Real is
      S      : Eye_Stream renames M.Eyes (E);
      Kept   : constant Natural := Natural (S.Kept_Groups.Length);
      Column : Natural := 0;
      Count  : Natural := 0;
   begin
      for K in 0 .. Kept - 1 loop
         if S.Kept_Groups (K) = Natural (G) and then S.Kept_Channels (K) = Channel then
            Column := K + 1;
         end if;
      end loop;
      if Column = 0 then
         return 0.0;
      end if;
      if not Seen (M, E, G, Channel) then
         return 0.0;
      end if;
      for Cell in 0 .. Natural (S.Shifts.Length) / Kept - 1 loop
         if S.Shifts (Cell * Kept + Column - 1) > 0.0 then
            Count := Count + 1;
         end if;
      end loop;
      return (if Count = 0 then 0.0 else Median_Shift (S, Kept, Column, Count));
   end Shift;

   procedure Dump_Cells (M : Model; Tag : String) is
      use Ada.Text_IO;
      File : File_Type;
      function Img (N : Integer) return String is (Ada.Strings.Fixed.Trim (N'Image, Ada.Strings.Left));
   begin
      if not Ada.Environment_Variables.Exists ("BD_DUMP_CELLS") then
         return;
      end if;
      Create (File, Out_File, Ada.Environment_Variables.Value ("BD_DUMP_CELLS") & "/cells_" & Tag & ".txt");
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            S    : Eye_Stream renames M.Eyes (E);
            N    : constant Natural := Cells (S.Grid);
            Kept : constant Natural := Natural (S.Kept_Groups.Length);
            Line : Ada.Strings.Unbounded.Unbounded_String;
         begin
            Put_Line (File, "eye " & Img (Natural (E)) & " width " & Img (S.Grid.Width) & " height " & Img (S.Grid.Height)
                      & " columns " & Img (S.Grid.Columns) & " rows " & Img (S.Grid.Rows) & " kept " & Img (Kept));
            for Cell in 0 .. Natural'Min (N, Natural (S.Textured.Length)) - 1 loop
               Ada.Strings.Unbounded.Append (Line, (if S.Textured (Cell) then "1" else "0"));
            end loop;
            Put_Line (File, "textured " & Ada.Strings.Unbounded.To_String (Line));
            if Natural (S.Shift_U.Length) = N * Kept and then Natural (S.Shift_V.Length) = N * Kept
              and then Natural (S.Col_Responds.Length) = N * Kept
            then
               --  Each group's cells as the lock-in's test of its block finds them responding (a shift where it does),
               --  and its channels' shifts summed: what the cell does when the group moves whole.
               for G in M.Groups.First_Index .. M.Groups.Last_Index loop
                  if (for some K in 1 .. Kept => S.Kept_Groups (S.Kept_Groups.First_Index + K - 1) = Natural (G)) then
                     Line := Ada.Strings.Unbounded.Null_Unbounded_String;
                     for Cell in 0 .. N - 1 loop
                        Ada.Strings.Unbounded.Append
                          (Line, (if (for some K in 1 .. Kept =>
                                        S.Kept_Groups (S.Kept_Groups.First_Index + K - 1) = Natural (G)
                                        and then S.Shifts (Cell * Kept + K - 1) > 0.0)
                                  then "1" else "0"));
                     end loop;
                     Put_Line (File, "responds " & Img (Natural (G)) & " " & Ada.Strings.Unbounded.To_String (Line));
                     for Cell in 0 .. N - 1 loop
                        declare
                           U, V : Real := 0.0;
                        begin
                           for K in 1 .. Kept loop
                              if S.Kept_Groups (S.Kept_Groups.First_Index + K - 1) = Natural (G) then
                                 U := U + S.Shift_U (Cell * Kept + K - 1);
                                 V := V + S.Shift_V (Cell * Kept + K - 1);
                              end if;
                           end loop;
                           if U /= 0.0 or else V /= 0.0 then
                              Put_Line (File, "whole " & Img (Natural (G)) & " " & Img (Cell) & " " & Driver.Log.Image (U, 6)
                                        & " " & Driver.Log.Image (V, 6));
                           end if;
                        end;
                     end loop;
                  end if;
               end loop;
               for K in 1 .. Kept loop
                  Put_Line (File, "column " & Img (K) & " group " & Img (S.Kept_Groups (S.Kept_Groups.First_Index + K - 1))
                            & " channel " & Img (S.Kept_Channels (S.Kept_Channels.First_Index + K - 1)));
                  for Cell in 0 .. N - 1 loop
                     if S.Col_Responds (Cell * Kept + K - 1) then
                        Put_Line (File, "cell " & Img (Cell) & " " & Driver.Log.Image (S.Shift_U (Cell * Kept + K - 1), 6)
                                  & " " & Driver.Log.Image (S.Shift_V (Cell * Kept + K - 1), 6)
                                  & (if S.Shifts (Cell * Kept + K - 1) > 0.0 then " group" else " alone"));
                     end if;
                  end loop;
               end loop;
            end if;
         end;
      end loop;
      Close (File);
   end Dump_Cells;

end Driver.Robot.Lockin;
