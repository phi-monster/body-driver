with Ada.Containers.Vectors;
with Ada.Directories;
with Ada.Environment_Variables;
with Ada.Long_Float_Text_IO;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Ada.Unchecked_Deallocation;
with Driver.Geometry;
with Driver.Numerics.Dense;
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
   procedure Free is new Ada.Unchecked_Deallocation (Flow.Pixel_Flags, Flow.Pixel_Flags_Access);

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

   package Wrist_Vectors is new Ada.Containers.Vectors (Positive, Flow.Wrist_Picture, Flow."=");

   --  A fitted arm the fixed eyes watch: its group, its channels along its chain from the base, per channel its
   --  reference reading and the range its readings were seen over, its joints as its fit has them along the chain and
   --  the relative standard deviation of each one's unit (none when the fit states no covariance), the turn each way
   --  per chain place (found at its first pose), and how many of its poses are taken.
   type Arm_Info is record
      Arm    : Arm_Id := 1;
      Group  : Group_Id'Base := 0;
      N      : Natural := 0;
      Chain  : Count_Vectors.Vector;
      Ref, Low, High : Real_Vectors.Vector;
      Joints : Joint_Array_Access;
      Unit_Sigma : Real_Access;
      Steps  : Real_Access;
      Taken  : Natural := 0;
      --  How many of its poses (the first ones) the arm's own eye has a picture of: the scene's stills run ahead of
      --  the flow poses along the same poses, and a pose stilled is not pictured again when the flows reach it.
      Stilled : Natural := 0;
      --  The arm's own eye, its fit's place among the model's, and that eye's pictures at the arm's poses with where
      --  it stood in the arm's reference frame (Flow_Eye.Refine_By_Scene: the world it saw places the fixed eyes).
      Eye     : Eye_Id'Base := 0;
      Fitting : Natural := 0;
      Wrists  : Wrist_Vectors.Vector;
      --  The arm's readings at each of those pictures, by channel, picture after picture: where the eye stood by any
      --  fit of the arm (Variants_Of).
      Wrist_Readings : Real_Vectors.Vector;
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

   --  An arm's fit's variants for placing a fixed eye (Flow_Eye.Arm_Variant), with the arrays they point at.
   type Rigid_Array_Access is access Flow.Rigid_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Flow.Rigid_Array, Rigid_Array_Access);
   type Joint_Access_Array is array (Positive range <>) of Joint_Array_Access;
   type Rigid_Access_Array is array (Positive range <>) of Rigid_Array_Access;
   type Variant_Array_Access is access Flow.Arm_Variant_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Flow.Arm_Variant_Array, Variant_Array_Access);
   type Variant_Set (Count : Natural) is record
      Variants : Variant_Array_Access;
      Joints   : Joint_Access_Array (1 .. Count);
      Eyes     : Rigid_Access_Array (1 .. Count);
   end record;
   type Variant_Set_Access is access Variant_Set;
   procedure Free_Set is new Ada.Unchecked_Deallocation (Variant_Set, Variant_Set_Access);

   procedure Free (S : in out Variant_Set_Access) is
   begin
      if S /= null then
         for J of S.Joints loop
            Free (J);
         end loop;
         for E of S.Eyes loop
            Free (E);
         end loop;
         Free (S.Variants);
         Free_Set (S);
      end if;
   end Free;

   --  The arm's fit moved along its covariance (Flow_Eye.Arm_Variant): per axis of the covariance of its terms (its
   --  eye's lens and its joints, in Kinematics.Fit's order) a standard deviation either way, the lesser end first: its
   --  joints and its lens moved by it (Fit.Move_Term), its table as the terms move it (Arm_Fit.Table_Response), and its
   --  eye's frame at each picture the eye took at the arm's poses placed by the moved joints; then per axis of what
   --  the table's points leave its plane uncertain by alone (Arm_Fit.Table_Own), the table alone moved so. None when
   --  the fit states no covariance of its terms or its table no response to them.
   function Variants_Of (R : Arm_Fit; Info : Arm_Info) return Variant_Set_Access is
      use Driver.Numerics.Arrays;
      N        : constant Natural := Natural (R.Joints.Length);
      Terms    : constant Natural := KFit.Terms (N);
      Pictures : constant Natural := Natural (Info.Wrists.Length);
   begin
      if N = 0 or else N /= Info.N or else Natural (R.Chain.Length) /= N or else Natural (R.Reference.Length) /= N
        or else Natural (R.Covariance.Length) /= Terms ** 2 or else Natural (R.Table_Response.Length) /= 3 * Terms
        or else not Driver.Geometry.Known (R.Table) or else Natural (Info.Wrist_Readings.Length) /= N * Pictures
      then
         return null;
      end if;
      declare
         Cov         : Real_Matrix (1 .. Terms, 1 .. Terms);
         Values      : Real_Vector (1 .. Terms);
         Vectors     : Real_Matrix (1 .. Terms, 1 .. Terms);
         Own_Values  : Real_Vector (1 .. 3);
         Own_Vectors : Real_Matrix (1 .. 3, 1 .. 3);
         Chain       : constant KFit.Chain_Order :=
           [for K in 1 .. N => Positive (R.Chain (R.Chain.First_Index + K - 1))];
         Joints0     : constant KFit.Joint_Array :=
           [for J in 1 .. N =>
              (W => R.Joints (R.Joints.First_Index + J - 1).W, P => R.Joints (R.Joints.First_Index + J - 1).P,
               C => R.Joints (R.Joints.First_Index + J - 1).C, Slide => R.Joints (R.Joints.First_Index + J - 1).Slide)];
         Lens0       : constant KFit.Lens :=
           (Fx => R.Lens.Fx, Fy => R.Lens.Fy, Cx => R.Lens.Cx, Cy => R.Lens.Cy, K1 => R.Lens.K1, K2 => R.Lens.K2);
         --  The table as its eye sees it (Fit.Sight_Plane: the points X with A * X = 1).
         A0          : constant Vec3 :=
           (1.0 / Driver.Numerics.Dot (R.Table.Normal, R.Table.Centre)) * R.Table.Normal;
         Count       : Natural := 0;
         Set         : Variant_Set_Access;
         K           : Natural := 0;

         --  The variant of these joints (by their readings' places), lens and table.
         procedure Add (Joints : KFit.Joint_Array; Lens : KFit.Lens; A : Vec3) is
            Along : constant Joint_Array_Access := new KFit.Joint_Array (1 .. N);
            Eyes  : constant Rigid_Array_Access := new Flow.Rigid_Array (1 .. Pictures);
         begin
            for J in 1 .. N loop
               Along (J) := Joints (Chain (J));
            end loop;
            for P in 1 .. Pictures loop
               declare
                  Change : Real_Array (1 .. N);
               begin
                  for J in 1 .. N loop
                     Change (J) := Info.Wrist_Readings (Info.Wrist_Readings.First_Index + N * (P - 1) + J - 1)
                       - R.Reference (R.Reference.First_Index + J - 1);
                  end loop;
                  Eyes (P) := KFit.Eye_At (Joints, Chain, Change);
               end;
            end loop;
            K := K + 1;
            Set.Joints (K) := Along;
            Set.Eyes (K) := Eyes;
            Set.Variants (K) :=
              (Joints => Along.all'Unchecked_Access, Eyes => Eyes.all'Unchecked_Access, Lens => Lens,
               Table  => KFit.Plane_Estimate_Of ((Found => True, A => A, Covariance => R.Table_Own,
                                                  Points => R.Table.Points, Scatter => R.Table.Scatter),
                                                 R.Table_Own));
         end Add;
      begin
         for P in 1 .. Terms loop
            for Q in 1 .. Terms loop
               Cov (P, Q) := R.Covariance (R.Covariance.First_Index + (P - 1) * Terms + Q - 1);
            end loop;
         end loop;
         Driver.Numerics.Dense.Small_Eigensystem (Cov, Values, Vectors);
         Driver.Numerics.Dense.Small_Eigensystem (R.Table_Own, Own_Values, Own_Vectors);
         for V of Values loop
            Count := Count + Boolean'Pos (V > 0.0);
         end loop;
         for V of Own_Values loop
            Count := Count + Boolean'Pos (V > 0.0);
         end loop;
         Set := new Variant_Set (2 * Count);
         Set.Variants := new Flow.Arm_Variant_Array (1 .. 2 * Count);
         for I in 1 .. Terms loop
            if Values (I) > 0.0 then
               for E in 1 .. 2 loop
                  declare
                     By     : constant Real := (if E = 1 then -1.0 else 1.0)
                       * Ada.Numerics.Long_Elementary_Functions.Sqrt (Values (I));
                     Joints : KFit.Joint_Array := Joints0;
                     Lens   : KFit.Lens := Lens0;
                     A      : Vec3 := A0;
                  begin
                     for T in 1 .. Terms loop
                        if Vectors (T, I) /= 0.0 then
                           KFit.Move_Term (Joints, Lens, T, By * Vectors (T, I));
                        end if;
                        for D in 1 .. 3 loop
                           A (D) := A (D) + By * Vectors (T, I)
                             * R.Table_Response (R.Table_Response.First_Index + (D - 1) * Terms + T - 1);
                        end loop;
                     end loop;
                     Add (Joints, Lens, A);
                  end;
               end loop;
            end if;
         end loop;
         for I in 1 .. 3 loop
            if Own_Values (I) > 0.0 then
               for E in 1 .. 2 loop
                  declare
                     By : constant Real := (if E = 1 then -1.0 else 1.0)
                       * Ada.Numerics.Long_Elementary_Functions.Sqrt (Own_Values (I));
                     A  : Vec3 := A0;
                  begin
                     for D in 1 .. 3 loop
                        A (D) := A (D) + By * Own_Vectors (D, I);
                     end loop;
                     Add (Joints0, Lens0, A);
                  end;
               end loop;
            end if;
         end loop;
         return Set;
      end;
   end Variants_Of;

   --  The flow's fit stored as the fixed eye's place (Fixed_Fit): its lens (its focal lengths across and down, the
   --  principal point, no distortion), its camera in the frame of every arm it saw (Arms, in the fit's order), and the
   --  covariance of the lens's six terms as an arm's fit has them (the logarithms of the two focal lengths, the
   --  principal point, the two distortion terms, held) and of every arm's turn about the camera's own axes and centre;
   --  the world arm's pose (the first arm's) and its twelve terms' covariance as the fixed eye's users read them, the
   --  eye known only when it saw that arm. Its spread the place it leaves the arms' points in its picture uncertain by.
   procedure Store (M : in out Model; E : Eye_Id; Arms : Arm_Id_Array; Result : Flow.Report) is
      Slots : constant Natural := Arms'Length;
      Terms : constant Natural := KFit.Lens_Terms + 6 * Slots;
      Flow_Lens : constant Natural := Flow.Terms_For (Slots) - 6 * Slots;   --  the flow's own lens terms
      --  The flow's term behind each of the stored ones (nought: held, its rows and columns nought): both focal
      --  lengths' logarithms its one, the principal point its, then the arms' six each in the same order; and the
      --  term added to it (nought: none), the aspect's logarithm to the second focal length's.
      function From (P : Positive) return Natural is
        (if P <= KFit.Lens_Terms then (case P is when 1 | 2 => 1, when 3 => 2, when 4 => 3, when others => 0)
         else Flow_Lens + (P - KFit.Lens_Terms));
      function Also (P : Positive) return Natural is (if P = 2 then 4 else 0);
      function Cov (P, Q : Natural) return Real is
        (if P > 0 and then Q > 0 then Result.Covariance (P, Q) else 0.0);
      function Joint_At (P, Q : Positive) return Real is
        (if From (P) > 0 and then From (Q) > 0
         then Cov (From (P), From (Q)) + Cov (Also (P), From (Q)) + Cov (From (P), Also (Q)) + Cov (Also (P), Also (Q))
         else 0.0);
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

   --  The longest wait for the eyes' still views in a capture so far (Place.Take).
   Longest_Take : Natural := 0;

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
                     Info.Eye := Driver.Robot.Kinematics.Eye_Of (M, Info.Arm);
                     Info.Fitting := K;
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
                           --  The chain names its joints by their readings' places from 1, as the fit keeps its
                           --  joints (Kinematics.Chain_Of): taken one before, the first joint was the joints' index 0
                           --  and every other the joint before it. On the big machine's first round (10-22, 40
                           --  default-option fakes of every class), all seven cold boots that reached the placing
                           --  with a fitted arm failed there on the range check (d508, seed 508, among them).
                           F : constant Joint_Fit := R.Joints (R.Chain (J - 1));
                        begin
                           Info.Joints (J) := (W => F.W, P => F.P, C => F.C, Slide => F.Slide);
                        end;
                     end loop;
                     --  Each unit's relative deviation: the variance of its scale term in the fit's covariance.
                     if Natural (R.Covariance.Length) = KFit.Terms (Size) ** 2 then
                        Info.Unit_Sigma := new Real_Array (1 .. Size);
                        for J in 1 .. Size loop
                           declare
                              T : constant Positive := KFit.Term_Of (R.Chain (J - 1), KFit.Scale);
                              V : constant Real :=
                                R.Covariance (R.Covariance.First_Index + (T - 1) * KFit.Terms (Size) + T - 1);
                           begin
                              Info.Unit_Sigma (J) :=
                                (if V > 0.0 and then Info.Joints (J).C /= 0.0
                                 then Ada.Numerics.Long_Elementary_Functions.Sqrt (V) / abs Info.Joints (J).C else 0.0);
                           end;
                        end loop;
                     end if;
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
            Free (A.Unit_Sigma);
            Free (A.Steps);
            for W of A.Wrists loop
               Free (W.Luma);
               Free (W.World);
            end loop;
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
         --  Per eye, the directions the world's places told at its last fit with these flow poses (Report.Places_Told),
         --  and whether some eye's grew with the last doubling of the stills: none is judged before a doubling of the
         --  stills leaves the places telling no more, so that the flows tell only what the places cannot.
         Last_Places : array (1 .. Count_E) of Natural := [others => 0];
         Grown       : Boolean := False;

         function Eye_Of (K : Positive) return Eye_Id is (Eye_Id (Eyes (K - 1)));

         --  Up to Up_To poses of arm A, the arm turning one joint at a time either way at each, every eye that sees
         --  it taking still views; then the arm's own eye's stills of the scene at the poses after them, up to
         --  Stills_To (the arm still at each: one move, no turns).
         procedure Capture_Arm (A : Positive; Up_To : Positive; Stills_To : Natural) is
            Info : Arm_Info renames Arms (A);
            N    : constant Positive := Info.N;

            function Channel (Place : Positive) return Positive is (Info.Chain (Place - 1));

            procedure Go_To (Target : Real_Array) is
               C : Driver.Commands.Command;
            begin
               Driver.Commands.Set_Target (C, Info.Group, Target);
               Driver.Robot.Motion.Step (M, C, Report, Land => True);
            end Go_To;

            --  The body still and every eye that sees the arm, and the arm's own, with two frames or more since its
            --  picture last changed: every fixed eye's view (those that do not see the arm too, unused).
            --  A wait for the eyes' still views twice as long as the longest before it is told in the log, and at every
            --  doubling after, with each eye's view: k201 (a kiwi from its swept body on the big machine, 10-22) waited
            --  here from beat 2434 to 15289 and again from 15289 to its run's end, with nothing in the log.
            procedure Take (Views : out View_Vectors.Vector) is
               Held_For : Natural := 0;
               Tell_At  : Positive := 2 * Natural'Max (1, Longest_Take);
               procedure Tell is
                  Text : Ada.Strings.Unbounded.Unbounded_String;
                  procedure Describe (E : Eye_Id; What : String) is
                     S : Eye_Stream renames M.Eyes (E);
                  begin
                     Ada.Strings.Unbounded.Append
                       (Text, " eye" & E'Image & What & ", still view of" & Driver.Pixels.Frames (S.Settled)'Image
                              & " frames" & (if S.Is_Still then ", still" else ", changing")
                              & (if S.Watch_Done then ", settled" else ", settling") & ";");
                  end Describe;
               begin
                  for K in 1 .. Count_E loop
                     Describe (Eye_Of (K), (if Sees_It (K, A) then " sees the arm" else ""));
                  end loop;
                  if Info.Eye > 0 then
                     Describe (Info.Eye, " (the arm's own)");
                  end if;
                  Driver.Log.Line (Driver.Log.Robot, "flows: the still views still awaited after" & Held_For'Image
                                   & " beats:" & Ada.Strings.Unbounded.To_String (Text));
               end Tell;
               procedure Hold_One is
               begin
                  Driver.Robot.Motion.Hold (M, 1);
                  Held_For := Held_For + 1;
               end Hold_One;
            begin
               Driver.Robot.Motion.Settle (M, Waited);
               while (for some K in 1 .. Count_E =>
                        Sees_It (K, A) and then Driver.Pixels.Frames (M.Eyes (Eye_Of (K)).Settled) < 2)
                 or else (Info.Eye > 0 and then Driver.Pixels.Frames (M.Eyes (Info.Eye).Settled) < 2)
               loop
                  Hold_One;
                  if Held_For >= Tell_At then
                     Tell;
                     Tell_At := 2 * Tell_At;
                  end if;
               end loop;
               Longest_Take := Natural'Max (Longest_Take, Held_For);
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
                              --  Taken either way as well (21d1834f), the turns ran down below what the joints do:
                              --  Best_Turn's best follows the turn it is shown (n21s2, 10-27, SO-101 seed 3b's second
                              --  joint: at 21.3 reading units it asked for 8.3, at 0.42 for 0.38, the true median
                              --  motion of its links' pixels 1.8 and 0.04 px), and the hall's pictures of seed 3b's
                              --  first three joints changed a twentieth to a tenth of what renders of their turns do.
                              --  Every trial is in the log.
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
                                    Driver.Log.Line
                                      (Driver.Log.Robot, "flows: arm" & Info.Arm'Image & " joint" & J'Image
                                       & " turn " & Driver.Log.Image (Turn, 6) & " reading units: best "
                                       & (if Best < Real'Last then Driver.Log.Image (Best, 6) else "none"));
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

            --  The arm's own eye's picture as the arm stands, and where that eye stands in the arm's reference frame.
            procedure Wrist_Here is
            begin
               declare
                  V        : constant Driver.Pixels.View := M.Eyes (Info.Eye).Settled;
                  Last     : constant Natural := M.Beats - 1;
                  Absolute : Real_Array (1 .. N);
               begin
                  for C in 1 .. N loop
                     Absolute (C) := Driver.Robot.Channels.Reading (M, Info.Group, Last, C);
                  end loop;
                  --  Where the joints stand, not their readings: each less where its gears' slack stands since the fitted
                  --  arm's reference (Kinematics.Slack_Parts); the eye's pose and its readings for the flows are the
                  --  joints'. Taken at its readings, the fake's seed 20 (10-22) stood its base turn up to four degrees
                  --  off where it was.
                  declare
                     Parts : constant Real_Array := Driver.Robot.Kinematics.Slack_Parts (M, Info.Arm, Last);
                  begin
                     if Parts'Length = N then
                        for C in 1 .. N loop
                           Absolute (C) := Absolute (C) - Parts (C);
                        end loop;
                     end if;
                  end;
                  declare
                     W    : constant Positive := Driver.Pixels.Width (V);
                     H    : constant Positive := Driver.Pixels.Height (V);
                     Mask : constant Flow.Pixel_Flags_Access := new Flow.Pixel_Flags (1 .. W * H);
                  begin
                     for Y in 0 .. H - 1 loop
                        for X in 0 .. W - 1 loop
                           Mask (Y * W + X + 1) :=
                             Driver.Robot.Kinematics.Sees_World (M, Info.Arm, Real (X) + 0.5, Real (Y) + 0.5);
                        end loop;
                     end loop;
                     Info.Wrists.Append
                       (Flow.Wrist_Picture'(Luma   => Means_Of (V), Width => W, Height => H,
                                            Eye    =>
                                              Driver.Robot.Kinematics.Eye_In_Reference (M, Info.Arm, Absolute),
                                            World  => Mask));
                     for C in 1 .. N loop
                        Info.Wrist_Readings.Append (Absolute (C));
                     end loop;
                  end;
               end;
            end Wrist_Here;

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
               --  The arm's own eye's picture at the pose, unless a still of the scene took it there before.
               if Info.Eye > 0 and then K > Info.Stilled then
                  Wrist_Here;
                  Info.Stilled := K;
               end if;
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
            if Info.Eye > 0 then
               for K in Natural'Max (Info.Stilled, Info.Taken) + 1 .. Stills_To loop
                  Go_To (Pose_Targets (K));
                  exit when Report.Outcome /= Driver.Robot.Motion.Reached;
                  declare
                     Views : View_Vectors.Vector;
                  begin
                     Take (Views);
                  end;
                  Wrist_Here;
                  Info.Stilled := K;
               end loop;
            end if;
            Go_To (Pose_Targets (1));
         end Capture_Arm;

         --  Every eye not yet placed fitted to every arm it saw two poses of or more, through its one lens: stored when
         --  it is told (in the frame of the first such arm); not when a doubling of the poses left it no surer.
         --  For a fixed eye's flow fit studied alone (tools/flow_replay): with BD_DUMP_FITS naming a folder (as the arms'
         --  fits' inputs are, Kinematics.Fit), its inputs in a folder of their own, flow_EYE_BEAT: index.txt (the
         --  picture's size; per arm its joints along its chain as its fit has them, each one's unit's relative sigma,
         --  and per pose its readings, its steps and its noise) and per pose its still luma and each joint's turned ones
         --  either way, row after row, as raw reals (POSE_still.bin, POSE_minus_J.bin, POSE_plus_J.bin, under arm_U).
         procedure Dump_Flow (E : Positive; Used : Positive; Which : KFit.Count_Array; Width, Height : Positive) is
            use Ada.Text_IO;
            function Trim (S : String) return String is (Ada.Strings.Fixed.Trim (S, Ada.Strings.Both));
            Folder : constant String := Ada.Environment_Variables.Value ("BD_DUMP_FITS") & "/flow_"
              & Trim (Natural'Image (Eyes (E - 1))) & "_" & Trim (M.Beats'Image);
            Index  : File_Type;
            procedure Put_Real (X : Real) is
            begin
               Put (Index, " ");
               Ada.Long_Float_Text_IO.Put (Index, X, Fore => 1, Aft => Real'Digits + 1, Exp => 1);
            end Put_Real;
            procedure Write (Name : String; Luma : Real_Array) is
               F : Ada.Streams.Stream_IO.File_Type;
            begin
               Ada.Streams.Stream_IO.Create (F, Ada.Streams.Stream_IO.Out_File, Folder & "/" & Name);
               Real_Array'Write (Ada.Streams.Stream_IO.Stream (F), Luma);
               Ada.Streams.Stream_IO.Close (F);
            end Write;
         begin
            Ada.Directories.Create_Path (Folder);
            Create (Index, Out_File, Folder & "/index.txt");
            Put_Line (Index, "flow eye" & Natural'Image (Eyes (E - 1)) & " beat" & M.Beats'Image & " width"
                      & Width'Image & " height" & Height'Image & " arms" & Used'Image);
            for U in 1 .. Used loop
               declare
                  Info : Arm_Info renames Arms (Which (U));
                  P    : constant Natural := Natural (Captures (E, Which (U)).Length);
               begin
                  Put_Line (Index, "arm" & Info.Arm'Image & " joints" & Info.N'Image & " poses" & P'Image);
                  for J in 1 .. Info.N loop
                     Put (Index, "joint");
                     for X of Info.Joints (J).W loop
                        Put_Real (X);
                     end loop;
                     for X of Info.Joints (J).P loop
                        Put_Real (X);
                     end loop;
                     Put_Real (Info.Joints (J).C);
                     Put (Index, Natural'Image (Boolean'Pos (Info.Joints (J).Slide)));
                     Put_Real (if Info.Unit_Sigma = null then 0.0 else Info.Unit_Sigma (J));
                     New_Line (Index);
                  end loop;
                  for V in 1 .. P loop
                     declare
                        C : Pose_Capture renames Captures (E, Which (U)) (V).all;
                        Stem : constant String := "arm_" & Trim (U'Image) & "_pose_" & Trim (V'Image);
                     begin
                        Put (Index, "pose" & V'Image & " noise");
                        Put_Real (C.Noise);
                        New_Line (Index);
                        Put (Index, "readings");
                        for X of C.Readings loop
                           Put_Real (X);
                        end loop;
                        New_Line (Index);
                        Put (Index, "steps");
                        for X of C.Steps loop
                           Put_Real (X);
                        end loop;
                        New_Line (Index);
                        Write (Stem & "_still.bin", C.Still.all);
                        for J in 1 .. C.N loop
                           Write (Stem & "_minus_" & Trim (J'Image) & ".bin", C.Minus (J).all);
                           Write (Stem & "_plus_" & Trim (J'Image) & ".bin", C.Plus (J).all);
                        end loop;
                     end;
                  end loop;
               end;
            end loop;
            Close (Index);
         end Dump_Flow;

         --  May_Grow: the stills may double yet, and an eye whose places told more than at the stills before waits for
         --  them.
         procedure Fit_All (May_Grow : Boolean) is
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
                                 In_Arms (U) :=
                                   (Poses      => Views (U).all'Unchecked_Access,
                                    Joints     => Arms (A).Joints.all'Unchecked_Access,
                                    Unit_Sigma =>
                                      (if Arms (A).Unit_Sigma = null then null
                                       else Arms (A).Unit_Sigma.all'Unchecked_Access));
                              end;
                           end loop;
                           if Ada.Environment_Variables.Exists ("BD_DUMP_FITS") then
                              Dump_Flow (E, Used, [for U in 1 .. Used => Which (U)], Width, Height);
                           end if;
                           Flow.Fit_Eye (In_Arms, Width, Height, Result);
                           --  Placed again by the world the arms' own eyes saw at their poses (their tables' points
                           --  in this eye's picture at the first arm's first pose).
                           declare
                              type Pictures_Access is access Flow.Wrist_Picture_Array;
                              procedure Free is new Ada.Unchecked_Deallocation
                                (Flow.Wrist_Picture_Array, Pictures_Access);
                              Pictures : array (1 .. Used) of Pictures_Access;
                              Worlds   : Flow.Arm_World_Array (1 .. Used);
                              Variants : array (1 .. Used) of Variant_Set_Access;
                           begin
                              for U in 1 .. Used loop
                                 declare
                                    Info : Arm_Info renames Arms (Which (U));
                                    R    : Arm_Fit renames M.Kinematics (Info.Fitting).Result;
                                 begin
                                    Pictures (U) := new Flow.Wrist_Picture_Array (1 .. Natural (Info.Wrists.Length));
                                    for P in Pictures (U)'Range loop
                                       Pictures (U) (P) := Info.Wrists (P);
                                    end loop;
                                    Variants (U) := Variants_Of (R, Info);
                                    Worlds (U) :=
                                      (Pictures => Pictures (U).all'Unchecked_Access,
                                       Lens     => (Fx => R.Lens.Fx, Fy => R.Lens.Fy, Cx => R.Lens.Cx, Cy => R.Lens.Cy,
                                                    K1 => R.Lens.K1, K2 => R.Lens.K2),
                                       Table    => R.Table,
                                       Variants =>
                                         (if Variants (U) = null then null
                                          else Variants (U).Variants.all'Unchecked_Access));
                                 end;
                              end loop;
                              Flow.Refine_By_Scene
                                (In_Arms, Worlds, Captures (E, Which (1)) (1).Still.all, Width, Height, Result);
                              for U in 1 .. Used loop
                                 Free (Pictures (U));
                                 Free (Variants (U));
                              end loop;
                           end;
                           Driver.Log.Line (Driver.Log.Robot, "flows: fixed eye" & Natural'Image (Eyes (E - 1))
                                            & " from" & Natural'Image (Used) & " arms: "
                                            & (if Result.Determined then "told" else "untold") & ", focal "
                                            & Driver.Log.Image (Result.Lens.Fx, 1) & ", spread "
                                            & Driver.Log.Image (Result.Spread, 2) & " px"
                                            & ", the places tell" & Natural'Image (Result.Places_Told) & " directions"
                                            & (if May_Grow and then Result.Places_Told > Last_Places (E)
                                               then " (more than the stills before: more stills first)" else "")
                                            & (if Result.Determined then ""
                                               else ": " & Ada.Strings.Unbounded.To_String (Result.Why)));
                           if May_Grow and then Result.Places_Told > Last_Places (E) then
                              Grown := True;
                              Last_Why (E) := Result.Why;
                           elsif Result.Determined then
                              Store (M, Eye_Of (E), [for U in 1 .. Used => Arms (Which (U)).Arm], Result);
                              Told (E) := True;
                           else
                              Last_Why (E) := Result.Why;
                              if Result.Spread < Last_Spread (E) then
                                 More := True;
                                 Last_Spread (E) := Result.Spread;
                              end if;
                           end if;
                           Last_Places (E) := Result.Places_Told;
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
         --  The poses double, from two, until every eye is told or a doubling leaves none surer; with each, the arms'
         --  own eyes' stills of the scene, from as many, double until a doubling leaves the places telling no more or
         --  the stills would outnumber the flows' own pictures (a still and two a joint at every pose, of the arm with
         --  the fewest joints): the places check the flows with no more pictures than the flows take.
         declare
            Wanted : Positive := 2;
            Stills : Positive := 2;
         begin
            loop
               for A in 1 .. Count_A loop
                  if (for some E in 1 .. Count_E => Sees_It (E, A)
                                                      and then not Told (E))
                  then
                     Capture_Arm (A, Wanted, Stills);
                  end if;
               end loop;
               declare
                  Pictures : Positive := Positive'Last;   --  the flows' own, of the arm with the fewest
               begin
                  for A in 1 .. Count_A loop
                     if (for some E in 1 .. Count_E => Sees_It (E, A) and then not Told (E)) then
                        Pictures := Positive'Min (Pictures, Wanted * (2 * Arms (A).N + 1));
                     end if;
                  end loop;
                  Grown := False;
                  Fit_All (May_Grow => 2 * Stills <= Pictures);
               end;
               exit when Settled and then not Grown;
               if Grown then
                  Stills := 2 * Stills;
               else
                  Wanted := 2 * Wanted;
                  Stills := Positive'Max (Stills, Wanted);
                  Last_Places := [others => 0];
               end if;
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
         Free (A.Unit_Sigma);
         Free (A.Steps);
         for W of A.Wrists loop
            Free (W.Luma);
            Free (W.World);
         end loop;
      end loop;
   end Place;

end Driver.Robot.Flows;
