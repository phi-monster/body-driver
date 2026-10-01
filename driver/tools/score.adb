--  score ESTIMATES RECORDING [PROBE]
--
--  Scores a replay's estimates against simulator truth, whatever model made
--  them. ESTIMATES is what `replay RECORDING --estimates FILE` wrote; the
--  robot-reported end-effector poses inside RECORDING are the truth for the
--  arms (the driver never reads them); PROBE, a recording of the same rig with
--  intrinsic matrices in the observation, is the truth for the lines of sight.
--
--  Arms: the estimated poses T_est relate to the reported poses T_true by an
--  unknown similarity S (world frames and units differ) and a constant offset
--  X (the frame the driver chose on the last link): T_true = S T_est X. Both
--  are fitted on every other distinct pose; errors are reported on the rest.
--  Eyes carried by an arm are matched the same way. Lines of sight are
--  compared, in each eye's frame, with the rays of the true intrinsics, the
--  principal point taken at the image centre (the render is a symmetric
--  frustum and the matrix's principal-point convention is not documented).

with Ada.Command_Line;
with Ada.Containers.Vectors;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Bytes;
with Driver.Clock;
with Driver.Conventions;
with Driver.Json;
with Driver.Log;
with Driver.Msgpack;
with Driver.Numerics.Dense;
with Driver.Observations;
with Driver.Protocol;
with Driver.Recording;
with Driver.Stats;

procedure Score is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Driver.Log;
   use type Driver.Recording.Record_Kind;
   use type Driver.Json.Node;

   subtype Real is Driver.Real;
   subtype Real_Array is Driver.Real_Array;

   Degrees_Per_Radian : constant := 180.0 / Ada.Numerics.Pi;
   Millimetres_Per_Metre : constant := 1000.0;

   package Pose_Vectors is new Ada.Containers.Vectors (Positive, Rigid);

   type Beat_Poses is record
      Truth : Pose_Vectors.Vector;   --  reported end-effector poses, metres
      Tools : Pose_Vectors.Vector;   --  estimated tool poses, driver units
      Eyes  : Pose_Vectors.Vector;   --  estimated eye poses, driver units
      Has_Truth, Has_Estimate : Boolean := False;
   end record;

   package Beat_Vectors is new Ada.Containers.Vectors (Natural, Beat_Poses);
   Beats : Beat_Vectors.Vector;

   procedure Ensure (B : Natural) is
   begin
      while Beats.Is_Empty or else Beats.Last_Index < B loop
         Beats.Append (Beat_Poses'(others => <>));
      end loop;
   end Ensure;

   type Ray_Sample is record
      U, V : Real;
      D    : Vec3;
   end record;

   package Ray_Vectors is new Ada.Containers.Vectors (Positive, Ray_Sample);
   package Grid_Vectors is new Ada.Containers.Vectors (Positive, Ray_Vectors.Vector, Ray_Vectors."=");
   Rays : Grid_Vectors.Vector;

   type Size is record
      Width, Height : Natural := 0;
   end record;

   package Size_Vectors is new Ada.Containers.Vectors (Positive, Size);
   Sizes : Size_Vectors.Vector;

   function From_Matrix (Doc : Driver.Json.Document; N : Driver.Json.Node) return Rigid is
      use Driver.Json;
      M : Rigid;
   begin
      for I in 1 .. 3 loop
         for J in 1 .. 3 loop
            M.Rotation (I, J) := Number (Doc, Element (Doc, N, 4 * (I - 1) + J));
         end loop;
         M.Translation (I) := Number (Doc, Element (Doc, N, 4 * (I - 1) + 4));
      end loop;
      return M;
   end From_Matrix;

   procedure Read_Estimates (Path : String) is
      use Driver.Json;
      F   : Ada.Text_IO.File_Type;
      Doc : Document;
      Ok  : Boolean;
      Why : Unbounded_String;
   begin
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (F) loop
         Parse (Ada.Text_IO.Get_Line (F), Doc, Ok, Why);
         if Ok and then Lookup (Doc, Root (Doc), "beat") /= No_Node then
            declare
               B     : constant Natural := Natural (Number (Doc, Lookup (Doc, Root (Doc), "beat")));
               Tools : constant Node := Lookup (Doc, Root (Doc), "tools");
               Eyes  : constant Node := Lookup (Doc, Root (Doc), "eyes");
            begin
               Ensure (B);
               for I in 1 .. Count (Doc, Tools) loop
                  Beats (B).Tools.Append (From_Matrix (Doc, Element (Doc, Tools, I)));
               end loop;
               for I in 1 .. Count (Doc, Eyes) loop
                  Beats (B).Eyes.Append (From_Matrix (Doc, Element (Doc, Eyes, I)));
               end loop;
               Beats (B).Has_Estimate := True;
            end;
         elsif Ok and then Lookup (Doc, Root (Doc), "rays") /= No_Node then
            declare
               All_Rays : constant Node := Lookup (Doc, Root (Doc), "rays");
            begin
               for E in 1 .. Count (Doc, All_Rays) loop
                  declare
                     Grid : constant Node := Element (Doc, All_Rays, E);
                     G    : Ray_Vectors.Vector;
                  begin
                     for I in 1 .. Count (Doc, Grid) loop
                        declare
                           S : constant Node := Element (Doc, Grid, I);
                        begin
                           G.Append (Ray_Sample'(U => Number (Doc, Element (Doc, S, 1)),
                                                 V => Number (Doc, Element (Doc, S, 2)),
                                      D => [Number (Doc, Element (Doc, S, 3)), Number (Doc, Element (Doc, S, 4)),
                                            Number (Doc, Element (Doc, S, 5))]));
                        end;
                     end loop;
                     Rays.Append (G);
                  end;
               end loop;
            end;
         end if;
      end loop;
      Ada.Text_IO.Close (F);
   end Read_Estimates;

   function Ends_With (S, Tail : String) return Boolean is
     (S'Length >= Tail'Length and then S (S'Last - Tail'Length + 1 .. S'Last) = Tail);

   function Quaternion_Pose (X : Real_Array) return Rigid is
     (Rotation    => To_Matrix ((W => X (X'First + 3), X => X (X'First + 4), Y => X (X'First + 5),
                                 Z => X (X'First + 6))),
      Translation => [X (X'First), X (X'First + 1), X (X'First + 2)]);

   --  Calls Process for every observation of a recording, with its layout.
   generic
      with procedure Process (Doc : Driver.Msgpack.Document; Obs : Driver.Msgpack.Node;
                              Layout : Driver.Observations.Layout; Beat : Natural; Stop : out Boolean);
   procedure Each_Observation (Path : String);

   procedure Each_Observation (Path : String) is
      R      : Driver.Recording.Reader;
      Kind   : Driver.Recording.Record_Kind;
      Ns     : Long_Long_Integer;
      Data   : Driver.Bytes.Buffer;
      More   : Boolean;
      Known  : Boolean := False;
      Stop   : Boolean := False;
      Layout : Driver.Observations.Layout;
      Beat   : Natural := 0;

      procedure Robot_Message (Message : Driver.Bytes.Byte_Array) is
         Req : Driver.Protocol.Request;
         Ok  : Boolean;
      begin
         Driver.Protocol.Decode (Message, Req, Ok);
         if not Ok or else not Driver.Protocol.Has_Observation (Req) then
            return;
         end if;
         if not Known then
            Driver.Observations.Recognize (Req.Doc, Req.Observation, Layout, Known);
         end if;
         if Known then
            Process (Req.Doc, Req.Observation, Layout, Beat, Stop);
            Beat := Beat + 1;
         end if;
      end Robot_Message;
   begin
      Driver.Recording.Open (R, Path, More);
      while More and then not Stop loop
         Driver.Recording.Next (R, Kind, Ns, Data, More);
         if More and then Kind = Driver.Recording.Robot_Message then
            Data.Query (Robot_Message'Access);
         end if;
      end loop;
      Driver.Recording.Close (R);
   end Each_Observation;

   procedure Take_Truth (Doc : Driver.Msgpack.Document; Obs : Driver.Msgpack.Node;
                         Layout : Driver.Observations.Layout; Beat : Natural; Stop : out Boolean) is
      O : Driver.Observations.Observation;
   begin
      Stop := False;
      if Sizes.Is_Empty then
         for C of Layout.Cameras loop
            Sizes.Append (Size'(Width => C.Width, Height => C.Height));
         end loop;
      end if;
      Driver.Observations.Parse (Doc, Obs, Layout, Driver.Clock.Beat (Beat), O);
      Ensure (Beat);
      for G in Layout.Groups.First_Index .. Layout.Groups.Last_Index loop
         if Ends_With (To_String (Layout.Groups (G).Path), "ee_pose") and then O.Readings.Element (G)'Length = 7 then
            Beats (Beat).Truth.Append (Quaternion_Pose (O.Readings.Element (G)));
         end if;
      end loop;
      Beats (Beat).Has_Truth := not Beats (Beat).Truth.Is_Empty;
   end Take_Truth;

   procedure Read_Truth is new Each_Observation (Take_Truth);

   package Matrix_Vectors is new Ada.Containers.Vectors (Positive, Mat3);
   Intrinsics : Matrix_Vectors.Vector;

   procedure Take_Intrinsics (Doc : Driver.Msgpack.Document; Obs : Driver.Msgpack.Node;
                              Layout : Driver.Observations.Layout; Beat : Natural; Stop : out Boolean) is
      pragma Unreferenced (Layout, Beat);
      use Driver.Msgpack;

      procedure Walk (N : Node) is
      begin
         for I in 1 .. Count (Doc, N) loop
            declare
               V : constant Node := Value (Doc, N, I);
            begin
               if Text (Doc, Key (Doc, N, I)) = "intrinsic_matrix" and then Is_Numeric (Doc, V)
                 and then Numbers (Doc, V)'Length = 9
               then
                  declare
                     K : constant Real_Array := Numbers (Doc, V);
                     M : Mat3;
                  begin
                     for A in 1 .. 3 loop
                        for B in 1 .. 3 loop
                           M (A, B) := K (K'First + 3 * (A - 1) + B - 1);
                        end loop;
                     end loop;
                     Intrinsics.Append (M);
                  end;
               elsif Kind_Of (Doc, V) = Map_Value and then not Is_Ndarray (Doc, V) then
                  Walk (V);
               end if;
            end;
         end loop;
      end Walk;
   begin
      Walk (Obs);
      Stop := True;
   end Take_Intrinsics;

   procedure Read_Intrinsics is new Each_Observation (Take_Intrinsics);

   --  The fit: x = (log s, rotation vector of S, translation of S, rotation
   --  vector of X, translation of X). Each pose contributes its position
   --  error and its rotation error times the spread of the true positions,
   --  so both residuals are lengths and neither unit dominates.

   subtype Parameters is Real_Vector (1 .. 13);

   function Similarity (X : Parameters; T : Rigid) return Rigid is
      S  : constant Real := Exp (X (1));
      Rs : constant Mat3 := Driver.Numerics.Exp (Vec3'(X (2), X (3), X (4)));
      Ts : constant Vec3 := [X (5), X (6), X (7)];
      Rx : constant Mat3 := Driver.Numerics.Exp (Vec3'(X (8), X (9), X (10)));
      Tx : constant Vec3 := [X (11), X (12), X (13)];
   begin
      return (Rotation => Rs * T.Rotation * Rx, Translation => S * (Rs * (T.Rotation * Tx + T.Translation)) + Ts);
   end Similarity;

   type Pair is record
      Est, Truth : Rigid;
   end record;

   package Pair_Vectors is new Ada.Containers.Vectors (Positive, Pair);

   function Residuals (X : Parameters; Pairs : Pair_Vectors.Vector; Length : Real) return Real_Vector is
      R : Real_Vector (1 .. 6 * Natural (Pairs.Length));
   begin
      for I in 1 .. Natural (Pairs.Length) loop
         declare
            M : constant Rigid := Similarity (X, Pairs (I).Est);
         begin
            R (6 * I - 5 .. 6 * I - 3) := M.Translation - Pairs (I).Truth.Translation;
            R (6 * I - 2 .. 6 * I) := Length * Log (Transpose (Pairs (I).Truth.Rotation) * M.Rotation);
         end;
      end loop;
      return R;
   end Residuals;

   function Cost (X : Parameters; Pairs : Pair_Vectors.Vector; Length : Real) return Real is
      R : constant Real_Vector := Residuals (X, Pairs, Length);
   begin
      return R * R;
   end Cost;

   --  Gauss-Newton with backtracking, until a step lowers the cost by less
   --  than the unchanged fraction of it. The caps only bound pathological
   --  inputs; on scoring data the fit converges in a handful of steps.
   procedure Fit (Pairs : Pair_Vectors.Vector; Length : Real; X : in out Parameters) is
      Iteration_Cap : constant := 200;
      Halving_Cap   : constant := 52;   --  a binary64 mantissa: the step is then below rounding
   begin
      for Iteration in 1 .. Iteration_Cap loop
         declare
            R0 : constant Real_Vector := Residuals (X, Pairs, Length);
            C0 : constant Real := R0 * R0;
            J  : Real_Matrix (R0'Range, Parameters'Range);
            Step : Parameters;
            Full_Rank : Boolean;
            Moved : Boolean := False;
         begin
            for K in Parameters'Range loop
               declare
                  H  : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs X (K));
                  Xp : Parameters := X;
                  Xm : Parameters := X;
               begin
                  Xp (K) := X (K) + H;
                  Xm (K) := X (K) - H;
                  declare
                     D : constant Real_Vector := Residuals (Xp, Pairs, Length) - Residuals (Xm, Pairs, Length);
                  begin
                     for I in D'Range loop
                        J (I, K) := D (I) / (2.0 * H);
                     end loop;
                  end;
               end;
            end loop;
            Driver.Numerics.Dense.Least_Squares (J, -R0, Step, Full_Rank);
            exit when not Full_Rank;
            for Try in 1 .. Halving_Cap loop
               declare
                  C1 : constant Real := Cost (X + Step, Pairs, Length);
               begin
                  if C1 < C0 then
                     Moved := C0 - C1 > Driver.Conventions.Unchanged_Fraction * C0;
                     X := X + Step;
                     exit;
                  end if;
               end;
               Step := Step / 2.0;
            end loop;
            exit when not Moved;
         end;
      end loop;
   end Fit;

   --  The 24 rotations of the cube: starting orientations spread evenly over
   --  every axis permutation and sign, so the fit cannot be trapped by a bad start.
   function Cube_Rotation (Index : Positive) return Mat3 is
      Count : Natural := 0;
   begin
      for A in 1 .. 3 loop
         for Positive_A in Boolean loop
            for B in 1 .. 3 loop
               for Positive_B in Boolean loop
                  if B /= A then
                     Count := Count + 1;
                     if Count = Index then
                        declare
                           X : Vec3 := Zero3;
                           Y : Vec3 := Zero3;
                        begin
                           X (A) := (if Positive_A then 1.0 else -1.0);
                           Y (B) := (if Positive_B then 1.0 else -1.0);
                           declare
                              Z : constant Vec3 := Cross (X, Y);
                           begin
                              return [[X (1), Y (1), Z (1)], [X (2), Y (2), Z (2)], [X (3), Y (3), Z (3)]];
                           end;
                        end;
                     end if;
                  end if;
               end loop;
            end loop;
         end loop;
      end loop;
      return Identity3;
   end Cube_Rotation;

   function Spread (Pairs : Pair_Vectors.Vector; Of_Truth : Boolean) return Real is
      Mean : Vec3 := Zero3;
      Sum  : Real := 0.0;
      N    : constant Real := Real (Pairs.Length);
   begin
      for P of Pairs loop
         Mean := Mean + (if Of_Truth then P.Truth.Translation else P.Est.Translation) / N;
      end loop;
      for P of Pairs loop
         declare
            D : constant Vec3 := (if Of_Truth then P.Truth.Translation else P.Est.Translation) - Mean;
         begin
            Sum := Sum + D * D;
         end;
      end loop;
      return Sqrt (Sum / N);
   end Spread;

   function Best_Fit (Pairs : Pair_Vectors.Vector) return Parameters is
      Length    : constant Real := Spread (Pairs, True);
      Best      : Parameters := [others => 0.0];
      Best_Cost : Real := Real'Last;
   begin
      for Start in 1 .. 24 loop
         declare
            X : Parameters := [others => 0.0];
         begin
            X (1) := Log (Spread (Pairs, True) / Spread (Pairs, False));
            X (2 .. 4) := Log (Cube_Rotation (Start));
            Fit (Pairs, Length, X);
            if Cost (X, Pairs, Length) < Best_Cost then
               Best_Cost := Cost (X, Pairs, Length);
               Best := X;
            end if;
         end;
      end loop;
      return Best;
   end Best_Fit;

   function Largest (X : Real_Array) return Real is
      M : Real := X (X'First);
   begin
      for V of X loop
         M := Real'Max (M, V);
      end loop;
      return M;
   end Largest;

   procedure Score_Arms is
      Arms : Natural := 0;
   begin
      for B of Beats loop
         if B.Has_Truth and then B.Has_Estimate then
            Arms := Natural (B.Truth.Length);
            exit;
         end if;
      end loop;
      if Arms = 0 then
         Ada.Text_IO.Put_Line ("no beat has both truth and estimates");
      end if;
      for Arm in 1 .. Arms loop
         Ada.Text_IO.Put_Line ("true arm" & Arm'Image & ":");
         for Of_Eyes in Boolean loop
            for Candidate in Positive loop
               declare
                  Train, Test : Pair_Vectors.Vector;
                  Last_Position : Vec3 := [Real'Last, 0.0, 0.0];
                  To_Train : Boolean := True;
                  Exists   : Boolean := False;
               begin
                  for P of Beats loop
                     if P.Has_Truth and then P.Has_Estimate and then Natural (P.Truth.Length) >= Arm
                       and then Candidate <= Natural (if Of_Eyes then P.Eyes.Length else P.Tools.Length)
                     then
                        Exists := True;
                        --  Beats at which the true pose did not move are one pose.
                        if abs (P.Truth (Arm).Translation - Last_Position) > 0.0 then
                           Last_Position := P.Truth (Arm).Translation;
                           declare
                              Q : constant Pair :=
                                (Est => (if Of_Eyes then P.Eyes (Candidate) else P.Tools (Candidate)),
                                 Truth => P.Truth (Arm));
                           begin
                              if To_Train then
                                 Train.Append (Q);
                              else
                                 Test.Append (Q);
                              end if;
                           end;
                           To_Train := not To_Train;
                        end if;
                     end if;
                  end loop;
                  exit when not Exists;
                  if Natural (Train.Length) >= 5 and then not Test.Is_Empty then
                     declare
                        X : constant Parameters := Best_Fit (Train);
                        Position, Rotation : Real_Array (1 .. Natural (Test.Length));
                     begin
                        for I in Position'Range loop
                           declare
                              M : constant Rigid := Similarity (X, Test (I).Est);
                           begin
                              Position (I) := Millimetres_Per_Metre * abs (M.Translation - Test (I).Truth.Translation);
                              Rotation (I) := Degrees_Per_Radian * Angle (Transpose (Test (I).Truth.Rotation) * M.Rotation);
                           end;
                        end loop;
                        Ada.Text_IO.Put_Line
                          ("  " & (if Of_Eyes then "eye" else "tool") & Candidate'Image & ":" & Test.Length'Image
                           & " test poses, position median " & Image (Driver.Stats.Median (Position), 2)
                           & " mm, largest " & Image (Largest (Position), 2) & " mm; rotation median "
                           & Image (Driver.Stats.Median (Rotation), 3) & " deg, largest "
                           & Image (Largest (Rotation), 3) & " deg; scale " & Image (Exp (X (1)), 6) & " m per unit");
                     end;
                  end if;
               end;
            end loop;
         end loop;
      end loop;
   end Score_Arms;

   procedure Score_Rays is
   begin
      for E in 1 .. Natural'Min (Natural (Rays.Length), Natural'Min (Natural (Intrinsics.Length),
                                                                     Natural (Sizes.Length))) loop
         declare
            K : constant Mat3 := Intrinsics (E);
            W : constant Real := Real (Sizes (E).Width);
            H : constant Real := Real (Sizes (E).Height);
            Errors : Real_Array (1 .. Natural (Rays (E).Length));
         begin
            for I in Errors'Range loop
               declare
                  S : constant Ray_Sample := Rays (E) (I);
                  T : constant Vec3 := Unit ([(S.U - W / 2.0) / K (1, 1), (S.V - H / 2.0) / K (2, 2), 1.0]);
                  D : constant Vec3 := Unit (S.D);
               begin
                  Errors (I) := Degrees_Per_Radian * Arctan (abs Cross (D, T), D * T);
               end;
            end loop;
            Ada.Text_IO.Put_Line
              ("eye" & E'Image & ": lines of sight median " & Image (Driver.Stats.Median (Errors), 4) & " deg, largest "
               & Image (Largest (Errors), 4) & " deg (about " & Image (Driver.Stats.Median (Errors) / Degrees_Per_Radian
                                                                         * K (1, 1), 2) & " px)");
         end;
      end loop;
   end Score_Rays;

begin
   if Ada.Command_Line.Argument_Count < 2 then
      Line (Core, "usage: score ESTIMATES RECORDING [PROBE]");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   Read_Truth (Ada.Command_Line.Argument (2));
   Read_Estimates (Ada.Command_Line.Argument (1));
   Score_Arms;
   if Ada.Command_Line.Argument_Count >= 3 then
      Read_Intrinsics (Ada.Command_Line.Argument (3));
      Score_Rays;
   end if;
end Score;
