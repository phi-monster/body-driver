with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Driver.Beats;
with Driver.Commands;
with Driver.Conventions;
with Driver.Images;
with Driver.Log;
with Driver.Observations;
with Driver.Pixels;
with Driver.Robot.Channels;
with Driver.Robot.Kinematics;
with Driver.Robot.Kinematics.Fit;
with Driver.Robot.Kinematics.Flow_Eye;
with Driver.Robot.Motion;

package body Driver.Robot.Flows is

   package Flow renames Driver.Robot.Kinematics.Flow_Eye;
   package KFit renames Driver.Robot.Kinematics.Fit;
   use type Driver.Observations.Camera_Id;
   use type Driver.Observations.Group_Id;
   use type Driver.Robot.Motion.Step_Outcome;
   use type Flow.Luma_Access;

   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Flow.Luma_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Flow.Pose_View, Flow.Pose_Access);

   --  One pose as an eye saw it: the arm's readings and every joint's turn either way (chain places, from the
   --  reference), the noise of a difference of two still means, the still picture and the turned ones (luma).
   type Pose_Capture (N : Natural) is record
      Readings, Steps : Real_Array (1 .. N);
      Noise           : Real := 0.0;
      Still           : Flow.Luma_Access;
      Minus, Plus     : Flow.Luma_Array (1 .. N);
   end record;

   type Capture_Access is access Pose_Capture;
   procedure Free is new Ada.Unchecked_Deallocation (Pose_Capture, Capture_Access);
   package Capture_Vectors is new Ada.Containers.Vectors (Positive, Capture_Access);
   package View_Vectors is new Ada.Containers.Vectors (Positive, Driver.Pixels.View, Driver.Pixels."=");

   type Joint_Array_Access is access KFit.Joint_Array;
   procedure Free is new Ada.Unchecked_Deallocation (KFit.Joint_Array, Joint_Array_Access);
   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);

   --  A fitted arm the fixed eyes watch: its group, its channels along its chain from the base, per channel its
   --  reference reading and the range its readings were seen over, its joints as its fit has them along the chain, the
   --  turn each way per chain place (found at its first pose), and how many of its poses are taken.
   type Arm_Info is record
      Arm    : Arm_Id := 1;
      Group  : Group_Id'Base := 0;
      N      : Natural := 0;
      Chain  : Count_Vectors.Vector;
      Ref, Low, High : Real_Vectors.Vector;
      Joints : Joint_Array_Access;
      Steps  : Real_Access;
      Taken  : Natural := 0;
   end record;

   package Arm_Info_Vectors is new Ada.Containers.Vectors (Positive, Arm_Info);

   --  The K-th point of the radical-inverse sequence in base B (van der Corput): the K-th pose's place along a
   --  channel's range, each channel its own prime base (Halton), so that the poses cover the range seen evenly however
   --  many there are.
   function Radical_Inverse (K : Natural; B : Positive) return Real is
      Result : Real := 0.0;
      Scale  : Real := 1.0 / Real (B);
      N      : Natural := K;
   begin
      while N > 0 loop
         Result := Result + Real (N mod B) * Scale;
         N := N / B;
         Scale := Scale / Real (B);
      end loop;
      return Result;
   end Radical_Inverse;

   function Prime (I : Positive) return Positive is
      Count : Natural := 0;
      N     : Positive := 1;
   begin
      loop
         N := N + 1;
         if (for all D in 2 .. N - 1 => N mod D /= 0 or else D * D > N) then
            Count := Count + 1;
            exit when Count = I;
         end if;
      end loop;
      return N;
   end Prime;

   type Arm_Id_Array is array (Positive range <>) of Arm_Id;

   --  The flow's fit stored as the fixed eye's place (Fixed_Fit): its lens (one focal length both ways, the principal
   --  point, no distortion), its camera in the frame of every arm it saw (Arms, in the fit's order), and the covariance
   --  of the lens's six terms as an arm's fit has them (the logarithms of the two focal lengths, the principal point,
   --  the two distortion terms, held) and of every arm's turn about the camera's own axes and centre; the world arm's
   --  pose (the first arm's) and its twelve terms' covariance as the fixed eye's users read them, the eye known only
   --  when it saw that arm. Its spread the place it leaves the arms' points in its picture uncertain by.
   procedure Store (M : in out Model; E : Eye_Id; Arms : Arm_Id_Array; Result : Flow.Report) is
      Slots : constant Natural := Arms'Length;
      Terms : constant Natural := KFit.Lens_Terms + 6 * Slots;
      Flow_Lens : constant Natural := Flow.Terms_For (Slots) - 6 * Slots;   --  the flow's own lens terms
      --  The flow's term behind each of the stored ones (nought: held, its rows and columns nought): both focal
      --  lengths' logarithms its one, the principal point its, then the arms' six each in the same order.
      function From (P : Positive) return Natural is
        (if P <= KFit.Lens_Terms then (case P is when 1 | 2 => 1, when 3 => 2, when 4 => 3, when others => 0)
         else Flow_Lens + (P - KFit.Lens_Terms));
      function Joint_At (P, Q : Positive) return Real is
        (if From (P) > 0 and then From (Q) > 0 then Result.Covariance (From (P), From (Q)) else 0.0);
      --  The world arm's place among the arms (nought: the eye did not see it).
      World : Natural := 0;
      --  A term of the world arm's twelve among the stored ones.
      function Own (P : Positive) return Positive is
        (if P <= KFit.Lens_Terms then P else P + 6 * (World - 1));
   begin
      for S in Arms'Range loop
         if Arms (S) = 1 then
            World := S - Arms'First + 1;
         end if;
      end loop;
      while M.Fixed_Eyes.Last_Index < E loop
         M.Fixed_Eyes.Append (Fixed_Fit'(others => <>));
      end loop;
      declare
         F : Fixed_Fit renames M.Fixed_Eyes (E);
      begin
         F := (others => <>);
         F.Lens := (Fx => Result.Lens.Fx, Fy => Result.Lens.Fy, Cx => Result.Lens.Cx, Cy => Result.Lens.Cy,
                    K1 | K2 => 0.0);
         if World > 0 then
            F.Known := True;
            F.Arm := 1;
            F.Pose := Result.Poses (World);
            for P in 1 .. KFit.Lens_Terms + 6 loop
               for Q in 1 .. KFit.Lens_Terms + 6 loop
                  F.Covariance.Append (Joint_At (Own (P), Own (Q)));
               end loop;
            end loop;
         else
            F.Why := Ada.Strings.Unbounded.To_Unbounded_String ("it did not see the first arm, the world's");
         end if;
         for S in 1 .. Slots loop
            F.Arms.Append (Natural (Arms (Arms'First + S - 1)));
            F.Poses.Append (Result.Poses (S));
         end loop;
         for P in 1 .. Terms loop
            for Q in 1 .. Terms loop
               F.Joint.Append (Joint_At (P, Q));
            end loop;
         end loop;
         F.Used := Result.Used;
         F.Offered := Result.Offered;
         F.Sigma_Px := Result.Spread;
         F.Judged := True;
      end;
   end Store;

   procedure Not_Placed (M : in out Model; E : Eye_Id; Why : String) is
   begin
      while M.Fixed_Eyes.Last_Index < E loop
         M.Fixed_Eyes.Append (Fixed_Fit'(others => <>));
      end loop;
      M.Fixed_Eyes (E).Known := False;
      M.Fixed_Eyes (E).Judged := True;
      M.Fixed_Eyes (E).Why := Ada.Strings.Unbounded.To_Unbounded_String (Why);
   end Not_Placed;

   function Means_Of (V : Driver.Pixels.View) return Flow.Luma_Access is
      Result : constant Flow.Luma_Access :=
        new Real_Array (1 .. Driver.Pixels.Width (V) * Driver.Pixels.Height (V));
   begin
      Driver.Pixels.Means (V, Result.all);
      return Result;
   end Means_Of;

   procedure Place (M : in out Model) is
      Arms  : Arm_Info_Vectors.Vector;
      Eyes  : Count_Vectors.Vector;   --  the fixed eyes that see a fitted arm and lack a place

      procedure Look is
      begin
         for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
            declare
               R    : Arm_Fit renames M.Kinematics (K).Result;
               G    : constant Group_Id'Base := M.Kinematics (K).Group;
               Size : constant Natural := (if G > 0 then Group_Size (M, G) else 0);
            begin
               if R.Fitted and then Size > 0 and then Natural (R.Reference.Length) = Size
                 and then Natural (R.Joints.Length) = Size and then Natural (R.Chain.Length) = Size
                 and then Natural (M.Groups (G).Low_Seen.Length) = Size
                 and then Natural (M.Groups (G).High_Seen.Length) = Size
               then
                  declare
                     Info : Arm_Info;
                  begin
                     Info.Arm := M.Kinematics (K).Arm;
                     Info.Group := G;
                     Info.N := Size;
                     Info.Chain := R.Chain;
                     for C in 1 .. Size loop
                        Info.Ref.Append (R.Reference (C - 1));
                        Info.Low.Append (M.Groups (G).Low_Seen (C - 1));
                        Info.High.Append (M.Groups (G).High_Seen (C - 1));
                     end loop;
                     Info.Joints := new KFit.Joint_Array (1 .. Size);
                     Info.Steps := new Real_Array'(1 .. Size => 0.0);
                     for J in 1 .. Size loop
                        declare
                           F : constant Joint_Fit := R.Joints (R.Chain (J - 1) - 1);
                        begin
                           Info.Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
                        end;
                     end loop;
                     Arms.Append (Info);
                  end;
               end if;
            end;
         end loop;
         for E in 1 .. Eye_Count (M) loop
            --  An eye judged before (a body file's) is not captured again, told or not.
            if Eye_Mount (M, Eye_Id (E)).Kind = World_Fixed
              and then (Eye_Id (E) > M.Fixed_Eyes.Last_Index or else not M.Fixed_Eyes (Eye_Id (E)).Judged)
              and then (for some A of Arms => Driver.Robot.Kinematics.Sees (M, Eye_Id (E), A.Group))
            then
               Eyes.Append (E);
            end if;
         end loop;
      end Look;
   begin
      Driver.Beats.Within_A_Beat (Look'Access);
      if Eyes.Is_Empty or else Arms.Is_Empty then
         for A of Arms loop
            Free (A.Joints);
            Free (A.Steps);
         end loop;
         return;
      end if;
      declare
         Count_E : constant Positive := Natural (Eyes.Length);
         Count_A : constant Positive := Natural (Arms.Length);
         --  Per eye and arm: whether the eye sees the arm, and the poses of the arm the eye saw.
         Sees_It  : array (1 .. Count_E, 1 .. Count_A) of Boolean := [others => [others => False]];
         Captures : array (1 .. Count_E, 1 .. Count_A) of Capture_Vectors.Vector;
         Report   : Driver.Robot.Motion.Step_Report;
         Waited   : Natural;
         Last_Spread : Real_Array (1 .. Count_E) := [others => Real'Last];
         Last_Why    : array (1 .. Count_E) of Ada.Strings.Unbounded.Unbounded_String;
         Told        : array (1 .. Count_E) of Boolean := [others => False];   --  the flow told the eye's place
         Settled     : Boolean := False;   --  every eye placed or a doubling of the poses left none surer

         function Eye_Of (K : Positive) return Eye_Id is (Eye_Id (Eyes (K - 1)));

         --  Up to Up_To poses of arm A, the arm turning one joint at a time either way at each, every eye that sees
         --  it taking still views.
         procedure Capture_Arm (A : Positive; Up_To : Positive) is
            Info : Arm_Info renames Arms (A);
            N    : constant Positive := Info.N;

            function Channel (Place : Positive) return Positive is (Info.Chain (Place - 1));

            procedure Go_To (Target : Real_Array) is
               C : Driver.Commands.Command;
            begin
               Driver.Commands.Set_Target (C, Info.Group, Target);
               Driver.Robot.Motion.Step (M, C, Report, Land => True);
            end Go_To;

            --  The body still and every eye that sees the arm with two frames or more since its picture last changed:
            --  every eye's view (those that do not see the arm too, unused).
            procedure Take (Views : out View_Vectors.Vector) is
            begin
               Driver.Robot.Motion.Settle (M, Waited);
               while (for some K in 1 .. Count_E =>
                        Sees_It (K, A) and then Driver.Pixels.Frames (M.Eyes (Eye_Of (K)).Settled) < 2)
               loop
                  Driver.Robot.Motion.Hold (M, 1);
               end loop;
               Views.Clear;
               for K in 1 .. Count_E loop
                  Views.Append (M.Eyes (Eye_Of (K)).Settled);
               end loop;
            end Take;

            function Readings_Now return Real_Array is
               Result : Real_Array (1 .. N);
               Last   : constant Natural := M.Beats - 1;
            begin
               for J in 1 .. N loop
                  Result (J) :=
                    Driver.Robot.Channels.Reading (M, Info.Group, Last, Channel (J)) - Info.Ref (Channel (J) - 1);
               end loop;
               return Result;
            end Readings_Now;

            --  The K-th pose's targets (the reference the first, then the radical-inverse points of the range seen).
            function Pose_Targets (K : Positive) return Real_Array is
               Result : Real_Array (1 .. N);
            begin
               for C in 1 .. N loop
                  Result (C) := (if K = 1 then Info.Ref (C - 1)
                                 else Info.Low (C - 1)
                                      + Radical_Inverse (K - 1, Prime (C)) * (Info.High (C - 1) - Info.Low (C - 1)));
               end loop;
               return Result;
            end Pose_Targets;

            --  Every joint's turn either way: the least turn the arm's fit counts (Motion.Least_Turn) doubled until an
            --  eye that sees the arm sees the picture change with it (Pixels.Compare), then the turn that measures the
            --  rates best by what that change showed (Flow_Eye.Best_Turn: the motion that balances the central
            --  difference's bias against the noise), the least over the eyes and no less than the turn seen; none when
            --  no turn inside the range seen shows.
            procedure Find_Steps (Pose : Real_Array; Still : View_Vectors.Vector) is
            begin
               for J in 1 .. N loop
                  declare
                     C     : constant Positive := Channel (J);
                     Turn  : Real := Driver.Robot.Motion.Least_Turn (M, Info.Arm, C);
                     Range_Seen : constant Real := Info.High (C - 1) - Info.Low (C - 1);
                     Views : View_Vectors.Vector;
                  begin
                     while Turn > 0.0 and then 2.0 * Turn <= Range_Seen loop
                        declare
                           Target : Real_Array := Pose;
                           Shows  : Boolean := False;
                        begin
                           Target (C) := Pose (C) + Turn;
                           Go_To (Target);
                           Take (Views);
                           for K in 1 .. Count_E loop
                              if Sees_It (K, A) then
                                 declare
                                    Comparison : constant Driver.Pixels.Comparison :=
                                      Driver.Pixels.Compare (Still (K), Views (K));
                                 begin
                                    Shows := Shows
                                      or else (Comparison.Trusted
                                               and then Driver.Images.Count (Comparison.Changed) > 0);
                                 end;
                              end if;
                           end loop;
                           if Shows then
                              --  The best turn by the pixels this turn moved, taken and judged again until it no
                              --  longer grows by more than what is no change: the pixels a turn shows moving are
                              --  those of its own size (the fake, 10-21: 0.0040, 0.0048 and 0.0057 rad from trials
                              --  of 0.001, 0.002 and 0.004), and the turn used is the one whose pixels chose it.
                              loop
                                 declare
                                    Best : Real := Real'Last;
                                 begin
                                    for K in 1 .. Count_E loop
                                       if Sees_It (K, A) then
                                          declare
                                             Before : Flow.Luma_Access := Means_Of (Still (K));
                                             After  : Flow.Luma_Access := Means_Of (Views (K));
                                          begin
                                             Best := Real'Min
                                               (Best,
                                                Flow.Best_Turn
                                                  (Before.all, After.all, Turn,
                                                   Driver.Pixels.Compare (Still (K), Views (K)).Spread,
                                                   Driver.Pixels.Width (Views (K)), Driver.Pixels.Height (Views (K))));
                                             Free (Before);
                                             Free (After);
                                          end;
                                       end if;
                                    end loop;
                                    exit when not (Best < Real'Last
                                                   and then Best > (1.0 + Driver.Conventions.Unchanged_Fraction) * Turn
                                                   and then 2.0 * Best <= Range_Seen);
                                    Turn := Best;
                                    Target (C) := Pose (C) + Turn;
                                    Go_To (Target);
                                    Take (Views);
                                 end;
                              end loop;
                              exit;
                           end if;
                           Turn := 2.0 * Turn;
                        end;
                     end loop;
                     Info.Steps (J) := (if Turn > 0.0 and then 2.0 * Turn <= Range_Seen then Turn else 0.0);
                  end;
               end loop;
               Go_To (Pose);
            end Find_Steps;

            procedure Capture (K : Positive) is
               Pose     : constant Real_Array := Pose_Targets (K);
               Still    : View_Vectors.Vector;
               Readings : Real_Array (1 .. N);
               Minus, Plus : array (1 .. Count_E) of Flow.Luma_Array (1 .. N);
               Achieved : Real_Array (1 .. N) := [others => 0.0];
               Spreads  : array (1 .. Count_E) of Real_Array (1 .. N) := [others => [others => 0.0]];
            begin
               Go_To (Pose);
               if Report.Outcome /= Driver.Robot.Motion.Reached then
                  return;
               end if;
               Take (Still);
               if K = 1 then
                  Find_Steps (Pose, Still);
                  Take (Still);
               end if;
               Readings := Readings_Now;
               for J in 1 .. N loop
                  declare
                     C       : constant Positive := Channel (J);
                     Low_V, High_V : View_Vectors.Vector;
                     Low_R, High_R : Real_Array (1 .. N);
                     Target  : Real_Array := Pose;
                  begin
                     if Info.Steps (J) > 0.0 then
                        Target (C) := Pose (C) - Info.Steps (J);
                        Go_To (Target);
                        Take (Low_V);
                        Low_R := Readings_Now;
                        Target (C) := Pose (C) + Info.Steps (J);
                        Go_To (Target);
                        Take (High_V);
                        High_R := Readings_Now;
                        Achieved (J) := (High_R (J) - Low_R (J)) / 2.0;
                        for E in 1 .. Count_E loop
                           if Sees_It (E, A) then
                              Minus (E) (J) := Means_Of (Low_V (E));
                              Plus (E) (J) := Means_Of (High_V (E));
                              Spreads (E) (J) := Driver.Pixels.Compare (Low_V (E), High_V (E)).Spread;
                           end if;
                        end loop;
                     else
                        for E in 1 .. Count_E loop
                           if Sees_It (E, A) then
                              Minus (E) (J) := Means_Of (Still (E));
                              Plus (E) (J) := Means_Of (Still (E));
                           end if;
                        end loop;
                     end if;
                  end;
               end loop;
               Go_To (Pose);
               for E in 1 .. Count_E loop
                  if Sees_It (E, A) then
                     declare
                        Picture : Flow.Luma_Access := Means_Of (Still (E));
                        Noise   : Real := 0.0;
                        Counted : Natural := 0;
                        Sorted  : Real_Array (1 .. N);
                     begin
                        --  The noise of a difference of two still means: the median of what the joints' comparisons
                        --  found.
                        for J in 1 .. N loop
                           if Spreads (E) (J) > 0.0 then
                              Counted := Counted + 1;
                              Sorted (Counted) := Spreads (E) (J);
                           end if;
                        end loop;
                        for I in 2 .. Counted loop
                           for D in reverse 2 .. I loop
                              exit when Sorted (D - 1) <= Sorted (D);
                              declare
                                 Swap : constant Real := Sorted (D);
                              begin
                                 Sorted (D) := Sorted (D - 1);
                                 Sorted (D - 1) := Swap;
                              end;
                           end loop;
                        end loop;
                        if Counted > 0 then
                           Noise := Sorted ((Counted + 1) / 2);
                        end if;
                        if Noise > 0.0 then
                           Captures (E, A).Append
                             (new Pose_Capture'(N => N, Readings => Readings, Steps => Achieved, Noise => Noise,
                                                Still => Picture, Minus => Minus (E), Plus => Plus (E)));
                        else
                           Free (Picture);
                           for J in 1 .. N loop
                              if Minus (E) (J) /= null then
                                 Free (Minus (E) (J));
                              end if;
                              if Plus (E) (J) /= null then
                                 Free (Plus (E) (J));
                              end if;
                           end loop;
                        end if;
                     end;
                  end if;
               end loop;
            end Capture;
         begin
            while Info.Taken < Up_To loop
               Info.Taken := Info.Taken + 1;
               Capture (Info.Taken);
            end loop;
            Go_To (Pose_Targets (1));
         end Capture_Arm;

         --  Every eye not yet placed fitted to every arm it saw two poses of or more, through its one lens: stored when
         --  it is told (in the frame of the first such arm); not when a doubling of the poses left it no surer.
         procedure Fit_All is
            More : Boolean := False;
         begin
            for E in 1 .. Count_E loop
               if not Told (E) then
                  declare
                     Used : Natural := 0;
                     Which : array (1 .. Count_A) of Natural := [others => 0];
                  begin
                     for A in 1 .. Count_A loop
                        if Sees_It (E, A) and then Natural (Captures (E, A).Length) >= 2 then
                           Used := Used + 1;
                           Which (Used) := A;
                        end if;
                     end loop;
                     if Used > 0 then
                        declare
                           Width  : constant Positive := Driver.Pixels.Width (M.Eyes (Eye_Of (E)).Settled);
                           Height : constant Positive := Driver.Pixels.Height (M.Eyes (Eye_Of (E)).Settled);
                           type Views_Access is access Flow.Pose_Array;
                           procedure Free is new Ada.Unchecked_Deallocation (Flow.Pose_Array, Views_Access);
                           Views  : array (1 .. Used) of Views_Access;
                           In_Arms : Flow.Arm_Flow_Array (1 .. Used);
                           Result : Flow.Report (Arms => Used, Count => Flow.Terms_For (Used));
                        begin
                           for U in 1 .. Used loop
                              declare
                                 A  : constant Positive := Which (U);
                                 P  : constant Positive := Natural (Captures (E, A).Length);
                                 --  The background: the arm's poses' stills' per-pixel median (Flow_Eye.Evidence).
                                 Background : Real_Array (1 .. Width * Height);
                                 Column     : Real_Array (1 .. P);
                              begin
                                 for K in Background'Range loop
                                    for V in 1 .. P loop
                                       Column (V) := Captures (E, A) (V).Still (K);
                                    end loop;
                                    for I in 2 .. P loop
                                       for D in reverse 2 .. I loop
                                          exit when Column (D - 1) <= Column (D);
                                          declare
                                             Swap : constant Real := Column (D);
                                          begin
                                             Column (D) := Column (D - 1);
                                             Column (D - 1) := Swap;
                                          end;
                                       end loop;
                                    end loop;
                                    Background (K) := (if P mod 2 = 1 then Column ((P + 1) / 2)
                                                       else (Column (P / 2) + Column (P / 2 + 1)) / 2.0);
                                 end loop;
                                 Views (U) := new Flow.Pose_Array (1 .. P);
                                 for V in 1 .. P loop
                                    declare
                                       C : Pose_Capture renames Captures (E, A) (V).all;
                                    begin
                                       Views (U) (V) := Flow.Evidence (C.Readings, C.Still.all, Background, C.Noise,
                                                                       C.Minus, C.Plus, C.Steps, Width, Height);
                                    end;
                                 end loop;
                                 In_Arms (U) := (Poses  => Views (U).all'Unchecked_Access,
                                                 Joints => Arms (A).Joints.all'Unchecked_Access);
                              end;
                           end loop;
                           Flow.Fit_Eye (In_Arms, Width, Height, Result);
                           Driver.Log.Line (Driver.Log.Robot, "flows: fixed eye" & Natural'Image (Eyes (E - 1))
                                            & " from" & Natural'Image (Used) & " arms: "
                                            & (if Result.Determined then "told" else "untold") & ", focal "
                                            & Driver.Log.Image (Result.Lens.Fx, 1) & ", spread "
                                            & Driver.Log.Image (Result.Spread, 2) & " px"
                                            & (if Result.Determined then ""
                                               else ": " & Ada.Strings.Unbounded.To_String (Result.Why)));
                           if Result.Determined then
                              Store (M, Eye_Of (E), [for U in 1 .. Used => Arms (Which (U)).Arm], Result);
                              Told (E) := True;
                           else
                              Last_Why (E) := Result.Why;
                              if Result.Spread < Last_Spread (E) then
                                 More := True;
                                 Last_Spread (E) := Result.Spread;
                              end if;
                           end if;
                           for U in 1 .. Used loop
                              for V in Views (U)'Range loop
                                 Free (Views (U) (V));
                              end loop;
                              Free (Views (U));
                           end loop;
                        end;
                     end if;
                  end;
               end if;
            end loop;
            Settled := not More;
         end Fit_All;
      begin
         for E in 1 .. Count_E loop
            for A in 1 .. Count_A loop
               Sees_It (E, A) := Driver.Robot.Kinematics.Sees (M, Eye_Of (E), Arms (A).Group);
            end loop;
         end loop;
         --  The poses double, from two, until every eye is told or a doubling leaves none surer.
         declare
            Wanted : Positive := 2;
         begin
            loop
               for A in 1 .. Count_A loop
                  if (for some E in 1 .. Count_E => Sees_It (E, A)
                                                      and then not Told (E))
                  then
                     Capture_Arm (A, Wanted);
                  end if;
               end loop;
               Fit_All;
               exit when Settled;
               Wanted := 2 * Wanted;
            end loop;
         end;
         for E in 1 .. Count_E loop
            if not Told (E) then
               Not_Placed (M, Eye_Of (E), Ada.Strings.Unbounded.To_String (Last_Why (E)));
            end if;
            for A in 1 .. Count_A loop
               for V of Captures (E, A) loop
                  declare
                     P : Capture_Access := V;
                  begin
                     Free (P.Still);
                     for J in P.Minus'Range loop
                        if P.Minus (J) /= null then
                           Free (P.Minus (J));
                        end if;
                        if P.Plus (J) /= null then
                           Free (P.Plus (J));
                        end if;
                     end loop;
                     Free (P);
                  end;
               end loop;
            end loop;
         end loop;
      end;
      for A of Arms loop
         Free (A.Joints);
         Free (A.Steps);
      end loop;
   end Place;

end Driver.Robot.Flows;
