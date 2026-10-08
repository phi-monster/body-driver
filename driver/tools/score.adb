--  score ESTIMATES RECORDING TRUTH
--
--  Scores a replay's estimates against the simulator's truth, whatever model
--  made them. ESTIMATES is what `replay RECORDING --estimates FILE` wrote;
--  TRUTH is the file the simulator wrote beside the run
--  (harness/robodojo_truth), which the driver never sees. Every truth line
--  repeats the state values of its observation and is paired with the
--  recorded observation whose readings are the same, so truth lines of
--  observations that were never sent cannot shift the pairing.
--
--  Arms: an estimated tool pose T_est relates to the true pose T_true of some
--  link by an unknown similarity S (world frames and units differ) and a
--  constant offset X (the frame the driver chose on its last link):
--  T_true = S T_est X. Every true link that turns like the tool between the
--  estimate's own successive distinct poses is tried (the angle of R_i^T R_j
--  does not depend on S or X; a link that moves while the tool stands, a finger
--  of the other hand, is not compared on the beats the tool stood); the fit is
--  made on every other distinct pose of the estimate and the errors reported on
--  the rest, with how far apart those poses lie, for the link that fits best
--  (links rigid with one another fit alike). The driver has one world, so
--  there is one S: the first tool fitted finds it with its X, and every later
--  tool is fitted in that world with only its own X.
--
--  Eyes: an eye frame is defined (z along the optical axis, x and y along +U
--  and +V), so an estimated eye pose is compared with the true optical frame
--  through the one S, with no offset of its own; its lines of sight with the
--  rays of the true intrinsics, or of the rig's F-theta lens.
--
--  Hands: each estimated tip is taken into the true tool link by its arm's S
--  and X and compared, at the beat whose closer reading is nearest the tip's,
--  with the support point of a finger's collision mesh along the estimated
--  press direction; the fingers are the links below the tool link in the
--  robot's joint tree through a joint that moves, and lobes are given to fingers by the smallest total distance. The
--  finger's farthest vertex along the tool's approach (from the tool link
--  towards the fingers) is reported beside it.
--
--  score --check RECORDING TRUTH CLOSER LINK... checks the scorer itself: it
--  makes estimates from the truth (tools at the given links, eyes at the
--  cameras, rays from the lenses, one hand on the first link, read by the
--  CLOSER group) seen through a known similarity and offset, and scores them;
--  a correct scorer reports the scale it was given and no error.
--
--  score --project RECORDING TRUTH BEAT EYE prints, for every link and every
--  object the truth puts inside that eye's image at that beat, its pixel:
--  drawn on the frame (driver/tools/frame), it checks the truth's camera
--  model against the images.

with Ada.Command_Line;
with Ada.Containers.Indefinite_Holders;
with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Containers.Indefinite_Vectors;
with Ada.Containers.Vectors;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Streams.Stream_IO;
with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with Interfaces;
with Driver.Bytes;
with Driver.Clock;
with Driver.Conventions;
with Driver.Json;
with Driver.Log;
with Driver.Numerics.Dense;
with Driver.Observations;
with Driver.Protocol;
with Driver.Recording;
with Driver.Stats;
with Driver.Uncertain;

procedure Score is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Driver.Log;
   use type Driver.Recording.Record_Kind;
   use type Driver.Json.Node;
   use type Driver.Json.Kind;
   use type Driver.Real_Array;

   subtype Real is Driver.Real;
   subtype Real_Array is Driver.Real_Array;

   Degrees_Per_Radian    : constant := 180.0 / Ada.Numerics.Pi;
   Millimetres_Per_Metre : constant := 1000.0;

   package Pose_Vectors is new Ada.Containers.Vectors (Positive, Rigid);
   package Pose_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Rigid);
   package Value_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Real_Array);
   package Name_Vectors is new Ada.Containers.Indefinite_Vectors (Positive, String);
   package Real_Holders is new Ada.Containers.Indefinite_Holders (Real_Array);

   ---------------------------------------------------------------------------
   --  Truth

   type Truth_Line is record
      Links   : Pose_Maps.Map;    --  link name -> world pose, metres
      Cameras : Pose_Maps.Map;    --  camera name -> optical frame
      Objects : Pose_Maps.Map;    --  object instance -> root pose
      State   : Value_Maps.Map;   --  state key -> the observation's own values
   end record;

   package Truth_Vectors is new Ada.Containers.Vectors (Positive, Truth_Line);
   Truth : Truth_Vectors.Vector;

   type Lens is record
      Has_K        : Boolean := False;
      K            : Mat3 := Identity3;
      Width, Height : Natural := 0;
      Coefficients : Real_Holders.Holder;   --  F-theta: angle = sum of c (i) r ** (i - 1), r in pixels
      Max_Field    : Real := 0.0;           --  F-theta: the full field of view, degrees
   end record;

   package Lens_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Lens);
   Lenses : Lens_Maps.Map;

   Store     : Unbounded_String;
   package Key_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, String);
   Link_Keys : Key_Maps.Map;   --  link name -> geometry file in the store

   type Joint is record
      Parent, Child, Kind : Unbounded_String;
   end record;

   package Joint_Vectors is new Ada.Containers.Vectors (Positive, Joint);
   Joints : Joint_Vectors.Vector;   --  every robot's joint tree

   function Quaternion_Pose (X : Real_Array) return Rigid is
     (Rotation    => To_Matrix ((W => X (X'First + 3), X => X (X'First + 4), Y => X (X'First + 5),
                                 Z => X (X'First + 6))),
      Translation => [X (X'First), X (X'First + 1), X (X'First + 2)]);

   function Numbers_Of (Doc : Driver.Json.Document; N : Driver.Json.Node) return Real_Array is
      R : Real_Array (1 .. Driver.Json.Count (Doc, N));
   begin
      for I in R'Range loop
         R (I) := Driver.Json.Number (Doc, Driver.Json.Element (Doc, N, I));
      end loop;
      return R;
   end Numbers_Of;

   procedure Read_Truth (Path : String) is
      use Driver.Json;
      F   : Ada.Text_IO.File_Type;
      Doc : Document;
      Ok  : Boolean;
      Why : Unbounded_String;
   begin
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (F) loop
         Parse (Ada.Text_IO.Get_Line (F), Doc, Ok, Why);
         if Ok and then Text (Doc, Lookup (Doc, Root (Doc), "kind")) = "geometry" then
            declare
               Links : constant Node := Lookup (Doc, Root (Doc), "links");
            begin
               Store := To_Unbounded_String (Text (Doc, Lookup (Doc, Root (Doc), "store")));
               declare
                  All_Joints : constant Node := Lookup (Doc, Root (Doc), "joints");
               begin
                  for I in 1 .. Count (Doc, All_Joints) loop
                     Joints.Append
                       (Joint'(Parent => To_Unbounded_String (Text (Doc, Lookup (Doc, Element (Doc, All_Joints, I), "parent"))),
                               Child  => To_Unbounded_String (Text (Doc, Lookup (Doc, Element (Doc, All_Joints, I), "child"))),
                               Kind   => To_Unbounded_String (Text (Doc, Lookup (Doc, Element (Doc, All_Joints, I), "type")))));
                  end loop;
               end;
               for I in 1 .. Count (Doc, Links) loop
                  if Kind_Of (Doc, Member_Value (Doc, Links, I)) = String_Value then
                     Link_Keys.Include (Member_Name (Doc, Links, I), Text (Doc, Member_Value (Doc, Links, I)));
                  end if;
               end loop;
            end;
         elsif Ok and then Text (Doc, Lookup (Doc, Root (Doc), "kind")) = "observation" then
            declare
               L       : Truth_Line;
               Links   : constant Node := Lookup (Doc, Root (Doc), "links");
               Objects : constant Node := Lookup (Doc, Root (Doc), "objects");
               Cameras : constant Node := Lookup (Doc, Root (Doc), "cameras");
               State   : constant Node := Lookup (Doc, Root (Doc), "state");
            begin
               for I in 1 .. Count (Doc, Links) loop
                  L.Links.Include (Member_Name (Doc, Links, I), Quaternion_Pose (Numbers_Of (Doc, Member_Value (Doc, Links, I))));
               end loop;
               for I in 1 .. Count (Doc, Objects) loop
                  if Lookup (Doc, Member_Value (Doc, Objects, I), "pose") /= No_Node then
                     L.Objects.Include (Member_Name (Doc, Objects, I),
                                        Quaternion_Pose (Numbers_Of (Doc, Lookup (Doc, Member_Value (Doc, Objects, I), "pose"))));
                  end if;
               end loop;
               for I in 1 .. Count (Doc, State) loop
                  L.State.Include (Member_Name (Doc, State, I), Numbers_Of (Doc, Member_Value (Doc, State, I)));
               end loop;
               for I in 1 .. Count (Doc, Cameras) loop
                  declare
                     Name : constant String := Member_Name (Doc, Cameras, I);
                     C    : constant Node := Member_Value (Doc, Cameras, I);
                  begin
                     L.Cameras.Include (Name, Quaternion_Pose (Numbers_Of (Doc, Lookup (Doc, C, "pose"))));
                     if not Lenses.Contains (Name) then
                        declare
                           E          : Lens;
                           K          : constant Node := Lookup (Doc, C, "K");
                           Resolution : constant Real_Array := Numbers_Of (Doc, Lookup (Doc, C, "resolution"));
                           Ftheta     : constant Node := Lookup (Doc, C, "lens");
                        begin
                           E.Width := Natural (Resolution (1));
                           E.Height := Natural (Resolution (2));
                           E.Has_K := Kind_Of (Doc, K) = Array_Value;
                           if E.Has_K then
                              for A in 1 .. 3 loop
                                 for B in 1 .. 3 loop
                                    E.K (A, B) := Number (Doc, Element (Doc, Element (Doc, K, A), B));
                                 end loop;
                              end loop;
                           elsif Kind_Of (Doc, Ftheta) = Object_Value then
                              E.Coefficients := Real_Holders.To_Holder
                                (Numbers_Of (Doc, Lookup (Doc, Ftheta, "coefficients")));
                              E.Max_Field := Number (Doc, Lookup (Doc, Ftheta, "max_fov"));
                           end if;
                           Lenses.Include (Name, E);
                        end;
                     end if;
                  end;
               end loop;
               Truth.Append (L);
            end;
         end if;
      end loop;
      Ada.Text_IO.Close (F);
   end Read_Truth;

   ---------------------------------------------------------------------------
   --  The recording: per beat, the readings that pair it with its truth line

   type Recorded_Beat is record
      Readings : Value_Maps.Map;   --  group path -> values
      Line     : Natural := 0;     --  the truth line paired with it, 0 for none
   end record;

   package Beat_Vectors is new Ada.Containers.Vectors (Natural, Recorded_Beat);
   Recorded : Beat_Vectors.Vector;
   Camera_Paths : Name_Vectors.Vector;   --  the layout's camera paths, in eye order

   procedure Read_Recording (Path : String) is
      R      : Driver.Recording.Reader;
      Kind   : Driver.Recording.Record_Kind;
      Ns     : Long_Long_Integer;
      Data   : Driver.Bytes.Buffer;
      More   : Boolean;
      Known  : Boolean := False;
      Layout : Driver.Observations.Layout;

      procedure Robot_Message (Message : Driver.Bytes.Byte_Array) is
         Req : Driver.Protocol.Request;
         Ok  : Boolean;
         O   : Driver.Observations.Observation;
         B   : Recorded_Beat;
      begin
         Driver.Protocol.Decode (Message, Req, Ok);
         if not Ok or else not Driver.Protocol.Has_Observation (Req) then
            return;
         end if;
         if not Known then
            Driver.Observations.Recognize (Req.Doc, Req.Observation, Layout, Known);
            if Known then
               for C of Layout.Cameras loop
                  Camera_Paths.Append (To_String (C.Path));
               end loop;
            end if;
         end if;
         if Known then
            Driver.Observations.Parse (Req.Doc, Req.Observation, Layout, Driver.Clock.Beat (Recorded.Length), O);
            for G in Layout.Groups.First_Index .. Layout.Groups.Last_Index loop
               B.Readings.Include (To_String (Layout.Groups (G).Path), O.Readings.Element (G));
            end loop;
            Recorded.Append (B);
         end if;
      end Robot_Message;
   begin
      Driver.Recording.Open (R, Path, More);
      while More loop
         Driver.Recording.Next (R, Kind, Ns, Data, More);
         if More and then Kind = Driver.Recording.Robot_Message then
            Data.Query (Robot_Message'Access);
         end if;
      end loop;
      Driver.Recording.Close (R);
   end Read_Recording;

   function Same_State (B : Recorded_Beat; L : Truth_Line) return Boolean is
      Common : Natural := 0;
   begin
      for C in L.State.Iterate loop
         declare
            Key : constant String := "state/" & Value_Maps.Key (C);
         begin
            if B.Readings.Contains (Key) then
               if B.Readings (Key) /= Value_Maps.Element (C) then
                  return False;
               end if;
               Common := Common + 1;
            end if;
         end;
      end loop;
      return Common > 0;
   end Same_State;

   procedure Pair_Beats is
      Next   : Positive := 1;   --  the first truth line not yet paired
      Paired : Natural := 0;
   begin
      for B in Recorded.First_Index .. Recorded.Last_Index loop
         for L in Next .. Truth.Last_Index loop
            if Same_State (Recorded (B), Truth (L)) then
               Recorded (B).Line := L;
               Next := L + 1;
               Paired := Paired + 1;
               exit;
            end if;
         end loop;
      end loop;
      Ada.Text_IO.Put_Line (Natural'Image (Paired) & " of" & Natural'Image (Natural (Recorded.Length))
                            & " recorded observations paired with" & Natural'Image (Natural (Truth.Length))
                            & " truth lines");
   end Pair_Beats;

   ---------------------------------------------------------------------------
   --  Estimates

   type Estimated_Beat is record
      Tools, Eyes : Pose_Vectors.Vector;   --  driver units
   end record;

   package Estimate_Maps is new Ada.Containers.Indefinite_Ordered_Maps (Natural, Estimated_Beat);
   Estimated : Estimate_Maps.Map;   --  by beat

   type Ray_Sample is record
      U, V : Real;
      D    : Vec3;
   end record;

   package Ray_Vectors is new Ada.Containers.Vectors (Positive, Ray_Sample);
   package Grid_Vectors is new Ada.Containers.Vectors (Positive, Ray_Vectors.Vector, Ray_Vectors."=");
   Rays : Grid_Vectors.Vector;

   type Tip_Estimate is record
      Tip, Press : Vec3;
      Reading    : Real_Holders.Holder;
      Confirmed  : Boolean := False;   --  a second press from a distinct pose landed on it
      Has_Spread : Boolean := False;   --  the driver stated a covariance (estimates before it did not)
      Covariance : Mat3 := [others => [others => 0.0]];
   end record;

   type Lobe_Estimate is record
      Open, Closed : Tip_Estimate;
   end record;

   package Lobe_Vectors is new Ada.Containers.Vectors (Positive, Lobe_Estimate);

   type Hand_Estimate is record
      Arm    : Positive := 1;
      Closer : Unbounded_String;
      Lobes  : Lobe_Vectors.Vector;
   end record;

   package Hand_Vectors is new Ada.Containers.Vectors (Positive, Hand_Estimate);
   Hands : Hand_Vectors.Vector;

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

   function Vector_Of (Doc : Driver.Json.Document; N : Driver.Json.Node) return Vec3 is
     ([Driver.Json.Number (Doc, Driver.Json.Element (Doc, N, 1)), Driver.Json.Number (Doc, Driver.Json.Element (Doc, N, 2)),
       Driver.Json.Number (Doc, Driver.Json.Element (Doc, N, 3))]);

   procedure Read_Estimates (Path : String) is
      use Driver.Json;
      F   : Ada.Text_IO.File_Type;
      Doc : Document;
      Ok  : Boolean;
      Why : Unbounded_String;

      function Tip_Of (N : Node) return Tip_Estimate is
         T : Tip_Estimate :=
           (Tip     => Vector_Of (Doc, Lookup (Doc, N, "tip")),
            Press   => Vector_Of (Doc, Lookup (Doc, N, "press")),
            Reading => Real_Holders.To_Holder (Numbers_Of (Doc, Lookup (Doc, N, "reading"))),
            others  => <>);
         Confirmed  : constant Node := Lookup (Doc, N, "confirmed");
         Covariance : constant Node := Lookup (Doc, N, "covariance");
      begin
         T.Confirmed := Confirmed /= No_Node and then Kind_Of (Doc, Confirmed) = Boolean_Value
                        and then Is_True (Doc, Confirmed);
         if Covariance /= No_Node and then Kind_Of (Doc, Covariance) = Array_Value then
            T.Has_Spread := True;
            for I in 1 .. 3 loop
               declare
                  Row : constant Vec3 := Vector_Of (Doc, Element (Doc, Covariance, I));
               begin
                  for J in 1 .. 3 loop
                     T.Covariance (I, J) := Row (J);
                  end loop;
               end;
            end loop;
         end if;
         return T;
      end Tip_Of;
   begin
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (F) loop
         Parse (Ada.Text_IO.Get_Line (F), Doc, Ok, Why);
         if Ok and then Lookup (Doc, Root (Doc), "beat") /= No_Node then
            declare
               B     : Estimated_Beat;
               Tools : constant Node := Lookup (Doc, Root (Doc), "tools");
               Eyes  : constant Node := Lookup (Doc, Root (Doc), "eyes");
            begin
               for I in 1 .. Count (Doc, Tools) loop
                  B.Tools.Append (From_Matrix (Doc, Element (Doc, Tools, I)));
               end loop;
               for I in 1 .. Count (Doc, Eyes) loop
                  B.Eyes.Append (From_Matrix (Doc, Element (Doc, Eyes, I)));
               end loop;
               Estimated.Include (Natural (Number (Doc, Lookup (Doc, Root (Doc), "beat"))), B);
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
                                                 D => [Number (Doc, Element (Doc, S, 3)),
                                                       Number (Doc, Element (Doc, S, 4)),
                                                       Number (Doc, Element (Doc, S, 5))]));
                        end;
                     end loop;
                     Rays.Append (G);
                  end;
               end loop;
            end;
         elsif Ok and then Lookup (Doc, Root (Doc), "hands") /= No_Node then
            declare
               All_Hands : constant Node := Lookup (Doc, Root (Doc), "hands");
            begin
               for I in 1 .. Count (Doc, All_Hands) loop
                  declare
                     N : constant Node := Element (Doc, All_Hands, I);
                     H : Hand_Estimate;
                     L : constant Node := Lookup (Doc, N, "lobes");
                  begin
                     H.Arm := Positive (Number (Doc, Lookup (Doc, N, "arm")));
                     H.Closer := To_Unbounded_String (Text (Doc, Lookup (Doc, N, "closer")));
                     for J in 1 .. Count (Doc, L) loop
                        H.Lobes.Append (Lobe_Estimate'(Open   => Tip_Of (Lookup (Doc, Element (Doc, L, J), "open")),
                                         Closed => Tip_Of (Lookup (Doc, Element (Doc, L, J), "closed"))));
                     end loop;
                     Hands.Append (H);
                  end;
               end loop;
            end;
         end if;
      end loop;
      Ada.Text_IO.Close (F);
   end Read_Estimates;

   ---------------------------------------------------------------------------
   --  The fit: x = (log s, rotation vector of S, translation of S, rotation
   --  vector of X, translation of X). Each pose contributes its position
   --  error and its rotation error times the spread of the true positions,
   --  so both residuals are lengths and neither unit dominates.

   subtype Parameters is Real_Vector (1 .. 13);

   function Scale_Of (X : Parameters) return Real is (Exp (X (1)));
   function S_Rotation (X : Parameters) return Mat3 is (Driver.Numerics.Exp (Vec3'(X (2), X (3), X (4))));
   function S_Translation (X : Parameters) return Vec3 is ([X (5), X (6), X (7)]);
   function X_Rotation (X : Parameters) return Mat3 is (Driver.Numerics.Exp (Vec3'(X (8), X (9), X (10))));
   function X_Translation (X : Parameters) return Vec3 is ([X (11), X (12), X (13)]);

   function Similarity (X : Parameters; T : Rigid) return Rigid is
     (Rotation    => S_Rotation (X) * T.Rotation * X_Rotation (X),
      Translation => Scale_Of (X) * (S_Rotation (X) * (T.Rotation * X_Translation (X) + T.Translation))
                     + S_Translation (X));

   function World (X : Parameters; T : Rigid) return Rigid is
     --  S alone: a pose of the driver's world in the true world, no offset.
     (Rotation    => S_Rotation (X) * T.Rotation,
      Translation => Scale_Of (X) * (S_Rotation (X) * T.Translation) + S_Translation (X));

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
   --  inputs; on scoring data the fit converges in a handful of steps. Only
   --  X (First .. Parameters'Last) moves: a tool fitted in a world already
   --  fitted keeps that world (First = Offset_First).
   Offset_First : constant Positive := Parameters'Last - 5;

   procedure Fit (Pairs : Pair_Vectors.Vector; Length : Real; X : in out Parameters;
                  First : Positive := Parameters'First) is
      Iteration_Cap : constant := 200;
      Halving_Cap   : constant := 52;   --  a binary64 mantissa: the step is then below rounding
      Free          : constant Positive := Parameters'Last - First + 1;
   begin
      for Iteration in 1 .. Iteration_Cap loop
         declare
            R0 : constant Real_Vector := Residuals (X, Pairs, Length);
            C0 : constant Real := R0 * R0;
            J  : Real_Matrix (R0'Range, 1 .. Free);
            Step : Real_Vector (1 .. Free);
            Full_Rank : Boolean;
            Moved : Boolean := False;
         begin
            for C in 1 .. Free loop
               declare
                  K  : constant Positive := First + C - 1;
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
                        J (I, C) := D (I) / (2.0 * H);
                     end loop;
                  end;
               end;
            end loop;
            Driver.Numerics.Dense.Least_Squares (J, -R0, Step, Full_Rank);
            exit when not Full_Rank;
            for Try in 1 .. Halving_Cap loop
               declare
                  Next : Parameters := X;
               begin
                  Next (First .. Parameters'Last) := X (First .. Parameters'Last) + Step;
                  declare
                     C1 : constant Real := Cost (Next, Pairs, Length);
                  begin
                     if C1 < C0 then
                        Moved := C0 - C1 > Driver.Conventions.Unchanged_Fraction * C0;
                        X := Next;
                        exit;
                     end if;
                  end;
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

   --  The driver has one world, so one similarity S maps it into the true
   --  world for every tool and eye. The first tool fitted finds S with its
   --  offset X (Has_World False); every later one is fitted in that world,
   --  only its own X free (World, Length: the first fit's parameters and the
   --  length its rotation residuals were weighed by), so an arm placed wrong
   --  in the one world shows as that arm's error, not as a world of its own.
   function Best_Fit (Pairs : Pair_Vectors.Vector; Has_World : Boolean; World : Parameters; Length : Real)
                      return Parameters is
      Best      : Parameters := [others => 0.0];
      Best_Cost : Real := Real'Last;
   begin
      for Start in 1 .. 24 loop
         declare
            X : Parameters := [others => 0.0];
         begin
            if Has_World then
               X (1 .. Offset_First - 1) := World (1 .. Offset_First - 1);
               X (Offset_First .. Offset_First + 2) := Log (Cube_Rotation (Start));
               Fit (Pairs, Length, X, Offset_First);
            else
               X (1) := Log (Spread (Pairs, True) / Spread (Pairs, False));
               X (2 .. 4) := Log (Cube_Rotation (Start));
               Fit (Pairs, Length, X);
            end if;
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

   ---------------------------------------------------------------------------
   --  Arms

   type Arm_Fit is record
      Found : Boolean := False;
      Link  : Unbounded_String;
      X     : Parameters := [others => 0.0];
      --  The links are ranked by the root mean square of the test positions'
      --  errors, not their median: a link rigid with the estimate at every
      --  test pose beats one that is so at most of them (A17: a finger that
      --  slid under its presses fitted with the median as well as the wrist,
      --  and its slides bent the one world, the wrist eye 30.7 mm off).
      Rank : Real := Real'Last;
      --  The fit's own largest test errors (metres, radians): a relative pose
      --  that moves no more than these cannot be told from rigid.
      Largest_Position, Largest_Turn : Real := 0.0;
   end record;

   package Arm_Fit_Vectors is new Ada.Containers.Vectors (Positive, Arm_Fit);
   Arm_Fits : Arm_Fit_Vectors.Vector;

   --  The one world (Best_Fit), set by the first tool fitted.
   Has_World    : Boolean := False;
   World_X      : Parameters := [others => 0.0];
   World_Length : Real := 0.0;

   function Parent_Of (Link : String) return String is
      --  The link the joint tree hangs Link from; "" for a root.
   begin
      for J of Joints loop
         if To_String (J.Child) = Link then
            return To_String (J.Parent);
         end if;
      end loop;
      return "";
   end Parent_Of;

   function Rigid_With (Tool : Positive; Upper, Lower : String; Position_Bound, Turn_Bound : Real) return Boolean is
      --  Whether Lower's pose in Upper's frame moves, over the beats the tool
      --  has estimates, by no more than the bounds.
      First : Rigid;
      Have  : Boolean := False;
   begin
      for B in Recorded.First_Index .. Recorded.Last_Index loop
         if Recorded (B).Line > 0 and then Estimated.Contains (B)
           and then Natural (Estimated (B).Tools.Length) >= Tool
           and then Truth (Recorded (B).Line).Links.Contains (Upper)
           and then Truth (Recorded (B).Line).Links.Contains (Lower)
         then
            declare
               L : constant Truth_Line := Truth (Recorded (B).Line);
               R : constant Rigid := Inverse (L.Links (Upper)) * L.Links (Lower);
            begin
               if not Have then
                  First := R;
                  Have := True;
               elsif abs (R.Translation - First.Translation) > Position_Bound
                 or else Angle (Transpose (First.Rotation) * R.Rotation) > Turn_Bound
               then
                  return False;
               end if;
            end;
         end if;
      end loop;
      return Have;
   end Rigid_With;

   procedure Split_Pairs (Tool : Positive; Link : String; Train, Test : out Pair_Vectors.Vector) is
      --  Beats at which the estimate did not move are one pose; distinct
      --  poses alternate between fitting and testing. The poses are the
      --  estimate's, as in the ranking: on the truth's own, a link that moved
      --  while the tool stood gave a standing estimate pairs enough to be
      --  fitted to it (A16's arm 2 on its own fingers, 3.5e8 m per unit).
      Last_Position : Vec3 := [Real'Last, 0.0, 0.0];
      To_Train      : Boolean := True;
   begin
      Train.Clear;
      Test.Clear;
      for B in Recorded.First_Index .. Recorded.Last_Index loop
         if Recorded (B).Line > 0 and then Estimated.Contains (B)
           and then Natural (Estimated (B).Tools.Length) >= Tool
           and then Truth (Recorded (B).Line).Links.Contains (Link)
         then
            declare
               T : constant Rigid := Truth (Recorded (B).Line).Links (Link);
               E : constant Rigid := Estimated (B).Tools (Tool);
            begin
               if abs (E.Translation - Last_Position) > 0.0 then
                  Last_Position := E.Translation;
                  if To_Train then
                     Train.Append (Pair'(Est => E, Truth => T));
                  else
                     Test.Append (Pair'(Est => E, Truth => T));
                  end if;
                  To_Train := not To_Train;
               end if;
            end;
         end if;
      end loop;
   end Split_Pairs;

   type Turn_Mismatch is record
      Median, Spread : Real := Real'Last;
   end record;

   function Rotation_Mismatch (Tool : Positive; Link : String) return Turn_Mismatch is
      --  How much the tool turns between the estimate's successive distinct
      --  poses is the same angle whatever the world frame and the offset on
      --  the link (S and X cancel in R_i^T R_j), so the tool's true link turns
      --  as much as the estimate does: a cheap ranking of the links before any
      --  fit. The poses are the estimate's: on the truth's own, a link that
      --  moved while the tool stood (A16: the other hand's fingers, turn 0
      --  against 0) tied at no mismatch and no scatter and pushed the tool's
      --  link out of the candidates.
      package Real_Vectors is new Ada.Containers.Vectors (Positive, Real);
      Differences   : Real_Vectors.Vector;
      Last_Position : Vec3 := [Real'Last, 0.0, 0.0];
      Previous_Est, Previous_True : Rigid;
      Have : Boolean := False;
   begin
      for B in Recorded.First_Index .. Recorded.Last_Index loop
         if Recorded (B).Line > 0 and then Estimated.Contains (B)
           and then Natural (Estimated (B).Tools.Length) >= Tool
           and then Truth (Recorded (B).Line).Links.Contains (Link)
         then
            declare
               T : constant Rigid := Truth (Recorded (B).Line).Links (Link);
               E : constant Rigid := Estimated (B).Tools (Tool);
            begin
               if abs (E.Translation - Last_Position) > 0.0 then
                  Last_Position := E.Translation;
                  if Have then
                     Differences.Append (abs (Angle (Transpose (Previous_Est.Rotation) * E.Rotation)
                                              - Angle (Transpose (Previous_True.Rotation) * T.Rotation)));
                  end if;
                  Previous_Est := E;
                  Previous_True := T;
                  Have := True;
               end if;
            end;
         end if;
      end loop;
      if Natural (Differences.Length) < 2 then
         return (others => <>);
      end if;
      declare
         D : Real_Array (1 .. Natural (Differences.Length));
      begin
         for I in D'Range loop
            D (I) := Differences (I);
         end loop;
         return (Median => Driver.Stats.Median (D), Spread => Driver.Stats.Robust_Sigma (D));
      end;
   end Rotation_Mismatch;

   procedure Score_Arms is
      Tools : Natural := 0;
      Links : Name_Vectors.Vector;
      World_Tool : Positive := 1;   --  the tool whose fit set the one world
   begin
      for E of Estimated loop
         Tools := Natural'Max (Tools, Natural (E.Tools.Length));
      end loop;
      if Tools = 0 or else Truth.Is_Empty then
         Ada.Text_IO.Put_Line ("no estimated tool poses to score");
         return;
      end if;
      for C in Truth (Truth.First_Index).Links.Iterate loop
         Links.Append (Pose_Maps.Key (C));
      end loop;
      for Tool in 1 .. Tools loop
         declare
            Best : Arm_Fit;
            Best_Test : Pair_Vectors.Vector;
            Best_Length : Real := 0.0;
            Least : Turn_Mismatch;
            Turns : array (1 .. Natural (Links.Length)) of Turn_Mismatch;
         begin
            --  Only links that turn like the tool, within Z of the best one's
            --  own scatter, are worth a fit.
            for I in Turns'Range loop
               Turns (I) := Rotation_Mismatch (Tool, Links (I));
               if Turns (I).Median < Least.Median then
                  Least := Turns (I);
               end if;
            end loop;
            for I in Turns'Range loop
               declare
                  Link        : constant String := Links (I);
                  Train, Test : Pair_Vectors.Vector;
               begin
                  --  No link to rank when the estimate never moved (A16's arm 2 stood after its boot).
                  if Least.Median < Real'Last
                    and then Turns (I).Median <= Least.Median + Driver.Conventions.Z * Least.Spread
                  then
                     Split_Pairs (Tool, Link, Train, Test);
                  end if;
                  --  A world of its own needs both the estimate and the link to
                  --  have moved: the scale is the ratio of their spreads.
                  if Natural (Train.Length) >= 5 and then not Test.Is_Empty
                    and then (Has_World or else (Spread (Train, True) > 0.0 and then Spread (Train, False) > 0.0))
                  then
                     declare
                        X : constant Parameters :=
                          Best_Fit (Train, Has_World, World_X, (if Has_World then World_Length else Spread (Train, True)));
                        Squares : Real := 0.0;
                     begin
                        for P of Test loop
                           declare
                              D : constant Vec3 := Similarity (X, P.Est).Translation - P.Truth.Translation;
                           begin
                              Squares := Squares + D * D;
                           end;
                        end loop;
                        if Sqrt (Squares / Real (Test.Length)) < Best.Rank then
                           Best := (Found => True, Link => To_Unbounded_String (Link), X => X,
                                    Rank => Sqrt (Squares / Real (Test.Length)), others => <>);
                           Best_Test := Test;
                           Best_Length := Spread (Train, True);
                        end if;
                     end;
                  end if;
               end;
            end loop;
            if not Best.Found then
               Ada.Text_IO.Put_Line ("tool" & Tool'Image & ": too few distinct poses to fit (the estimate moves too little)");
            else
               declare
                  Position, Rotation : Real_Array (1 .. Natural (Best_Test.Length));
                  Travel : constant Real := Millimetres_Per_Metre * Spread (Best_Test, True);
               begin
                  for I in Position'Range loop
                     declare
                        M : constant Rigid := Similarity (Best.X, Best_Test (I).Est);
                     begin
                        Position (I) := Millimetres_Per_Metre * abs (M.Translation - Best_Test (I).Truth.Translation);
                        Rotation (I) := Degrees_Per_Radian * Angle (Transpose (Best_Test (I).Truth.Rotation) * M.Rotation);
                     end;
                  end loop;
                  --  How far the test poses lie apart is said with their errors:
                  --  an error is only as telling as the motion it was found over.
                  --  A link that moves no more than the largest error has not
                  --  shown which link carries the tool (A16's arm 2 stood: every
                  --  standing link of it fitted alike), so it carries neither
                  --  the world nor a hand.
                  if Travel <= Largest (Position) then
                     Best.Found := False;
                     Ada.Text_IO.Put_Line
                       ("tool" & Tool'Image & ": its true link moves no more over the" & Best_Test.Length'Image
                        & " test poses (" & Image (Travel, 3) & " mm) than its largest error ("
                        & Image (Largest (Position), 3) & " mm), so which link carries it is not known; its pose"
                        & " is scored as its eye's");
                  else
                     Best.Largest_Position := Largest (Position) / Millimetres_Per_Metre;
                     Best.Largest_Turn := Largest (Rotation) / Degrees_Per_Radian;
                     if not Has_World then
                        Has_World := True;
                        World_X := Best.X;
                        World_Length := Best_Length;
                        World_Tool := Tool;
                     end if;
                     Ada.Text_IO.Put_Line
                       ("tool" & Tool'Image & " (true link " & To_String (Best.Link) & "):" & Best_Test.Length'Image
                        & " test poses spread over " & Image (Travel, 1)
                        & " mm, position median " & Image (Driver.Stats.Median (Position), 2) & " mm, largest "
                        & Image (Largest (Position), 2) & " mm; rotation median "
                        & Image (Driver.Stats.Median (Rotation), 3) & " deg, largest " & Image (Largest (Rotation), 3)
                        & " deg; scale " & Image (Scale_Of (Best.X), 6) & " m per unit"
                        & (if World_Tool = Tool then "" else ", in the world of tool" & World_Tool'Image));
                  end if;
               end;
            end if;
            Arm_Fits.Append (Best);
         end;
      end loop;
   end Score_Arms;

   ---------------------------------------------------------------------------
   --  Eyes

   function Camera_Name (Eye : Positive) return String is
      --  The truth camera whose name is a component of the eye's layout path.
      Path : constant String := "/" & Camera_Paths (Eye) & "/";
   begin
      for C in Lenses.Iterate loop
         if Ada.Strings.Fixed.Index (Path, "/" & Lens_Maps.Key (C) & "/") > 0 then
            return Lens_Maps.Key (C);
         end if;
      end loop;
      return "";
   end Camera_Name;

   procedure Score_Eyes is
      --  Every fitted tool carries the one world (Best_Fit): the first does.
      World_Fit : Natural := 0;
   begin
      for I in reverse 1 .. Natural (Arm_Fits.Length) loop
         if Arm_Fits (I).Found then
            World_Fit := I;
         end if;
      end loop;
      if World_Fit = 0 then
         Ada.Text_IO.Put_Line ("eyes: no arm fit to carry the world frame");
         return;
      end if;
      for Eye in 1 .. Natural (Camera_Paths.Length) loop
         declare
            Name     : constant String := Camera_Name (Eye);
            Position : Real_Array (1 .. Natural (Recorded.Length));
            Rotation : Real_Array (1 .. Natural (Recorded.Length));
            N        : Natural := 0;
         begin
            if Name /= "" then
               for B in Recorded.First_Index .. Recorded.Last_Index loop
                  if Recorded (B).Line > 0 and then Estimated.Contains (B)
                    and then Natural (Estimated (B).Eyes.Length) >= Eye
                    and then Truth (Recorded (B).Line).Cameras.Contains (Name)
                  then
                     declare
                        M : constant Rigid := World (Arm_Fits (World_Fit).X, Estimated (B).Eyes (Eye));
                        T : constant Rigid := Truth (Recorded (B).Line).Cameras (Name);
                     begin
                        N := N + 1;
                        Position (N) := Millimetres_Per_Metre * abs (M.Translation - T.Translation);
                        Rotation (N) := Degrees_Per_Radian * Angle (Transpose (T.Rotation) * M.Rotation);
                     end;
                  end if;
               end loop;
            end if;
            if N = 0 then
               Ada.Text_IO.Put_Line ("eye" & Eye'Image & ": no beat with both truth and an estimate");
            else
               Ada.Text_IO.Put_Line
                 ("eye" & Eye'Image & " (" & Name & "):" & N'Image & " beats, position median "
                  & Image (Driver.Stats.Median (Position (1 .. N)), 2) & " mm, largest "
                  & Image (Largest (Position (1 .. N)), 2) & " mm; rotation median "
                  & Image (Driver.Stats.Median (Rotation (1 .. N)), 3) & " deg, largest "
                  & Image (Largest (Rotation (1 .. N)), 3) & " deg");
            end if;
         end;
      end loop;
   end Score_Eyes;

   function True_Ray (L : Lens; U, V : Real) return Vec3 is
   begin
      if L.Has_K then
         return Unit ([(U - L.K (1, 3)) / L.K (1, 1), (V - L.K (2, 3)) / L.K (2, 2), 1.0]);
      end if;
      declare
         --  F-theta about the image centre (the rig puts its optical centre there).
         C  : constant Real_Array := L.Coefficients.Element;
         Du : constant Real := U - Real (L.Width) / 2.0;
         Dv : constant Real := V - Real (L.Height) / 2.0;
         R  : constant Real := Sqrt (Du * Du + Dv * Dv);
         Theta : Real := 0.0;
      begin
         for I in reverse C'Range loop
            Theta := Theta * R + C (I);
         end loop;
         if R = 0.0 then
            return [0.0, 0.0, 1.0];
         end if;
         return [Sin (Theta) * Du / R, Sin (Theta) * Dv / R, Cos (Theta)];
      end;
   end True_Ray;

   procedure Score_Rays is
   begin
      for E in 1 .. Natural'Min (Natural (Rays.Length), Natural (Camera_Paths.Length)) loop
         declare
            Name   : constant String := Camera_Name (E);
            Errors : Real_Array (1 .. Natural (Rays (E).Length));
         begin
            if Name = "" or else Errors'Length = 0
              or else (not Lenses (Name).Has_K and then Lenses (Name).Coefficients.Is_Empty)
            then
               Ada.Text_IO.Put_Line ("eye" & E'Image & ": no true lens");
            elsif (for some S of Rays (E) => abs S.D = 0.0) then
               --  An eye whose lens the driver has not measured writes no line of sight.
               Ada.Text_IO.Put_Line ("eye" & E'Image & " (" & Name & "): no lines of sight estimated");
            else
               for I in Errors'Range loop
                  declare
                     S : constant Ray_Sample := Rays (E) (I);
                     T : constant Vec3 := True_Ray (Lenses (Name), S.U, S.V);
                     D : constant Vec3 := Unit (S.D);
                  begin
                     Errors (I) := Degrees_Per_Radian * Arctan (abs Cross (D, T), D * T);
                  end;
               end loop;
               Ada.Text_IO.Put_Line
                 ("eye" & E'Image & " (" & Name & "): lines of sight median " & Image (Driver.Stats.Median (Errors), 4)
                  & " deg, largest " & Image (Largest (Errors), 4) & " deg");
            end if;
         end;
      end loop;
   end Score_Rays;

   ---------------------------------------------------------------------------
   --  Hands

   package Vertex_Vectors is new Ada.Containers.Vectors (Positive, Vec3);

   procedure Append_Vertices (Path : String; Points : in out Vertex_Vectors.Vector) is
      --  A raw little-endian float32 file of x y z triples (the truth store's
      --  vertex files), read a block at a time: meshes run to millions of vertices.
      use Ada.Streams.Stream_IO;
      use type Interfaces.Unsigned_32;
      use type Ada.Streams.Stream_Element_Offset;
      function To_Float is new Ada.Unchecked_Conversion (Interfaces.Unsigned_32, Interfaces.IEEE_Float_32);
      Vertex_Bytes : constant := 12;   --  three float32
      Block_Vertices : constant := 4096;
      F     : File_Type;
      Block : Driver.Bytes.Byte_Array (1 .. Vertex_Bytes * Block_Vertices);
      Last  : Ada.Streams.Stream_Element_Offset;

      function Float_At (B : Ada.Streams.Stream_Element_Offset) return Real is
        (Real (To_Float (Interfaces.Unsigned_32 (Block (B))
                         or Interfaces.Shift_Left (Interfaces.Unsigned_32 (Block (B + 1)), 8)
                         or Interfaces.Shift_Left (Interfaces.Unsigned_32 (Block (B + 2)), 16)
                         or Interfaces.Shift_Left (Interfaces.Unsigned_32 (Block (B + 3)), 24))));
   begin
      Open (F, In_File, Path);
      while not End_Of_File (F) loop
         Read (F, Block, Last);
         for V in 0 .. Last / Vertex_Bytes - 1 loop
            Points.Append (Vec3'(Float_At (V * Vertex_Bytes + 1), Float_At (V * Vertex_Bytes + 5),
                                 Float_At (V * Vertex_Bytes + 9)));
         end loop;
      end loop;
      Close (F);
   end Append_Vertices;

   package Mesh_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Vertex_Vectors.Vector, "<", Vertex_Vectors."=");
   Meshes_Read : Mesh_Maps.Map;   --  every stored geometry is read once

   function Collision_Points (Key : String) return Vertex_Vectors.Vector is
      --  Every collision vertex of a stored geometry, in its link's frame.
      use Driver.Json;
      F      : Ada.Text_IO.File_Type;
      Doc    : Document;
      Ok     : Boolean;
      Why    : Unbounded_String;
      Points : Vertex_Vectors.Vector;
   begin
      if Meshes_Read.Contains (Key) then
         return Meshes_Read (Key);
      end if;
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, To_String (Store) & "/" & Key & ".json");
      Parse (Ada.Text_IO.Get_Line (F), Doc, Ok, Why);
      Ada.Text_IO.Close (F);
      if Ok then
         declare
            Meshes : constant Node := Lookup (Doc, Root (Doc), "meshes");
         begin
            for I in 1 .. Count (Doc, Meshes) loop
               if Is_True (Doc, Lookup (Doc, Element (Doc, Meshes, I), "collision")) then
                  Append_Vertices (To_String (Store) & "/" & Text (Doc, Lookup (Doc, Element (Doc, Meshes, I), "points")),
                                   Points);
               end if;
            end loop;
         end;
      end if;
      Meshes_Read.Include (Key, Points);
      return Points;
   end Collision_Points;

   function Support (Points : Vertex_Vectors.Vector; In_Tool : Rigid; Direction : Vec3) return Vec3 is
      --  The vertex that leads along Direction, all in the tool link's frame.
      Best       : Vec3 := Zero3;
      Best_Along : Real := Real'First;
   begin
      for V of Points loop
         declare
            P : constant Vec3 := In_Tool * V;
         begin
            if P * Direction > Best_Along then
               Best_Along := P * Direction;
               Best := P;
            end if;
         end;
      end loop;
      return Best;
   end Support;

   function Centroid (Points : Vertex_Vectors.Vector; In_Tool : Rigid) return Vec3 is
      Sum : Vec3 := Zero3;
   begin
      for V of Points loop
         Sum := Sum + In_Tool * V;
      end loop;
      return Sum / Real'Max (1.0, Real (Points.Length));
   end Centroid;

   function Is_Finger (Name, Tool_Link : String) return Boolean is
      --  A finger hangs below the tool link in the robot's joint tree with at
      --  least one joint between them that moves: links fixed to the tool
      --  link (a camera, a flange) are part of the tool, not fingers.
      function Below (Link : String; Through_Moving : Boolean) return Boolean is
      begin
         for J of Joints loop
            if To_String (J.Parent) = Link then
               declare
                  Moving : constant Boolean := Through_Moving or else To_String (J.Kind) /= "PhysicsFixedJoint";
               begin
                  if To_String (J.Child) = Name then
                     return Moving;
                  elsif Below (To_String (J.Child), Moving) then
                     return True;
                  end if;
               end;
            end if;
         end loop;
         return False;
      end Below;
   begin
      return Below (Tool_Link, False);
   end Is_Finger;

   function Nearest_Beat (Closer : String; Reading : Real_Array) return Natural is
      --  The paired beat whose closer reading is nearest the given one.
      Best      : Natural := Natural'Last;
      Best_Dist : Real := Real'Last;
   begin
      for B in Recorded.First_Index .. Recorded.Last_Index loop
         if Recorded (B).Line > 0 and then Recorded (B).Readings.Contains (Closer) then
            declare
               Got : constant Real_Array := Value_Maps.Element (Recorded (B).Readings, Closer);
               Sum : Real := 0.0;
            begin
               if Got'Length = Reading'Length then
                  for I in Got'Range loop
                     Sum := Sum + (Got (I) - Reading (Reading'First + I - Got'First)) ** 2;
                  end loop;
                  if Sum < Best_Dist then
                     Best_Dist := Sum;
                     Best := B;
                  end if;
               end if;
            end;
         end if;
      end loop;
      return Best;
   end Nearest_Beat;

   procedure Score_Hands is

      function Fingers_Below (Tool_Link : String) return Name_Vectors.Vector is
         --  The fingers: links below the tool link in the joint tree, through a joint that moves.
         Prefix : constant String := Tool_Link (Tool_Link'First .. Ada.Strings.Fixed.Index (Tool_Link, "/"));
         Found  : Name_Vectors.Vector;
      begin
         for C in Link_Keys.Iterate loop
            declare
               Name : constant String := Key_Maps.Key (C);
            begin
               if Name /= Tool_Link and then Name'Length > Prefix'Length
                 and then Name (Name'First .. Name'First + Prefix'Length - 1) = Prefix
                 and then Is_Finger (Name, Tool_Link)
               then
                  Found.Append (Name);
               end if;
            end;
         end loop;
         return Found;
      end Fingers_Below;

      function Hand_Fit (Arm : Positive) return Arm_Fit is
         --  Links rigid with one another while the tool was scored fit alike, so
         --  the arm's fit may have taken one the fingers do not hang from (A17's
         --  closer stood: a finger, or the camera fixed to the wrist). The hand
         --  is scored from the nearest link up the tree that has fingers and
         --  stayed rigid with the fitted one, its offset fitted again in the one
         --  world.
         Fit  : Arm_Fit := Arm_Fits (Arm);
         Link : Unbounded_String := Fit.Link;
      begin
         while Fingers_Below (To_String (Link)).Is_Empty loop
            declare
               Up : constant String := Parent_Of (To_String (Link));
            begin
               exit when Up = ""
                 or else not Rigid_With (Arm, Up, To_String (Fit.Link), Fit.Largest_Position, Fit.Largest_Turn);
               Link := To_Unbounded_String (Up);
            end;
         end loop;
         if Link /= Fit.Link and then not Fingers_Below (To_String (Link)).Is_Empty then
            declare
               Train, Test : Pair_Vectors.Vector;
            begin
               Split_Pairs (Arm, To_String (Link), Train, Test);
               if Natural (Train.Length) >= 5 then
                  Fit.X := Best_Fit (Train, True, World_X, World_Length);
                  Fit.Link := Link;
               end if;
            end;
         end if;
         return Fit;
      end Hand_Fit;

   begin
      for H of Hands loop
         if H.Arm > Natural (Arm_Fits.Length) or else not Arm_Fits (H.Arm).Found then
            Ada.Text_IO.Put_Line ("hand of arm" & H.Arm'Image & ": the arm has no fit to take its tips into");
            goto Next_Hand;
         end if;
         declare
            Fit       : constant Arm_Fit := Hand_Fit (H.Arm);
            Tool_Link : constant String := To_String (Fit.Link);
            Fingers   : constant Name_Vectors.Vector := Fingers_Below (Tool_Link);
         begin
            Ada.Text_IO.Put_Line ("hand of arm" & H.Arm'Image & " (tool link " & Tool_Link & ", fingers"
                                  & Fingers.Length'Image & ", lobes" & H.Lobes.Length'Image & "):");
            for At_Open in reverse Boolean loop
               declare
                  type Tip_Truth is record
                     Support, Farthest : Vec3;
                  end record;
                  type Truth_Grid is array (1 .. Natural (H.Lobes.Length), 1 .. Natural (Fingers.Length)) of Tip_Truth;
                  Grid      : Truth_Grid;
                  Estimates : array (1 .. Natural (H.Lobes.Length)) of Vec3;
                  --  A tip no press has measured at this opening (its press direction zero) is not scored.
                  Measured  : array (1 .. Natural (H.Lobes.Length)) of Boolean := [others => True];
                  --  Each tip's press direction in the true link frame, the
                  --  standard deviation the driver states for the tip along it
                  --  (metres; Real'Last when it states none), and whether a
                  --  second press confirmed the tip.
                  Presses   : array (1 .. Natural (H.Lobes.Length)) of Vec3;
                  Sigmas    : array (1 .. Natural (H.Lobes.Length)) of Real := [others => Real'Last];
                  Confirmed : array (1 .. Natural (H.Lobes.Length)) of Boolean := [others => False];
                  Usable    : Boolean := not Fingers.Is_Empty;
               begin
                  for Lobe in Estimates'Range loop
                     declare
                        E : constant Tip_Estimate :=
                          (if At_Open then H.Lobes (Lobe).Open else H.Lobes (Lobe).Closed);
                        B : constant Natural := Nearest_Beat (To_String (H.Closer), E.Reading.Element);
                        --  Into the true link frame: p = s Rx^T (p_D - tx), d = Rx^T d_D.
                        Rx : constant Mat3 := X_Rotation (Fit.X);
                        Press : constant Vec3 := (if abs E.Press > 0.0 then Unit (Transpose (Rx) * E.Press) else Zero3);
                     begin
                        Presses (Lobe) := Press;
                        Confirmed (Lobe) := E.Confirmed;
                        if E.Has_Spread and then abs E.Press > 0.0 then
                           --  The map into the true link frame turns and scales: the
                           --  spread along the turned press is s times the stated one.
                           Sigmas (Lobe) := Scale_Of (Fit.X) * Driver.Uncertain.Sigma_Along (E.Covariance, Unit (E.Press));
                        end if;
                        Estimates (Lobe) := Scale_Of (Fit.X) * (Transpose (Rx) * (E.Tip - X_Translation (Fit.X)));
                        if abs E.Press = 0.0 then
                           Measured (Lobe) := False;
                        elsif B = Natural'Last then
                           Usable := False;
                        else
                           for F in 1 .. Natural (Fingers.Length) loop
                              declare
                                 L        : constant Truth_Line := Truth (Recorded (B).Line);
                                 In_Tool  : constant Rigid := Inverse (L.Links (Tool_Link)) * L.Links (Fingers (F));
                                 Points   : constant Vertex_Vectors.Vector := Collision_Points (Link_Keys (Fingers (F)));
                              begin
                                 Grid (Lobe, F) := (Support  => Support (Points, In_Tool, Press),
                                                    Farthest => Support (Points, In_Tool, Unit (Centroid (Points, In_Tool))));
                              end;
                           end loop;
                        end if;
                     end;
                  end loop;
                  if not Usable then
                     Ada.Text_IO.Put_Line ("  " & (if At_Open then "open" else "closed") & ": no finger or no beat"
                                           & " with that closer reading");
                  else
                     --  Lobes to fingers by the smallest total distance, over every assignment.
                     declare
                        Lobes : constant Natural := Estimates'Length;
                        type Assignment is array (1 .. Lobes) of Natural;
                        Best_Total : Real := Real'Last;
                        Best_Of, Current : Assignment := [others => 0];

                        procedure Assign (Lobe : Positive; Total : Real) is
                        begin
                           if Lobe > Lobes then
                              if Total < Best_Total then
                                 Best_Total := Total;
                                 Best_Of := Current;
                              end if;
                              return;
                           end if;
                           if not Measured (Lobe) then
                              Current (Lobe) := 0;
                              Assign (Lobe + 1, Total);
                              return;
                           end if;
                           for F in 1 .. Natural (Fingers.Length) loop
                              if (for all K in 1 .. Lobe - 1 => Current (K) /= F) then
                                 Current (Lobe) := F;
                                 Assign (Lobe + 1, Total + abs (Estimates (Lobe) - Grid (Lobe, F).Support));
                              end if;
                           end loop;
                        end Assign;
                     begin
                        Assign (1, 0.0);
                        for Lobe in 1 .. Lobes loop
                           if not Measured (Lobe) then
                              Ada.Text_IO.Put_Line ("  " & (if At_Open then "open" else "closed") & " lobe" & Lobe'Image
                                                    & ": no press has measured its tip");
                           elsif Best_Of (Lobe) = 0 then
                              Ada.Text_IO.Put_Line ("  lobe" & Lobe'Image & ": more lobes than fingers");
                           else
                              declare
                                 --  The press measures the tip along its own direction: the
                                 --  error there, against the spread the driver states there.
                                 Along : constant Real := (Estimates (Lobe) - Grid (Lobe, Best_Of (Lobe)).Support) * Presses (Lobe);
                              begin
                                 Ada.Text_IO.Put_Line
                                   ("  " & (if At_Open then "open" else "closed") & " lobe" & Lobe'Image & " ("
                                    & Fingers (Best_Of (Lobe)) & ", "
                                    & (if Confirmed (Lobe) then "confirmed" else "provisional") & "): "
                                    & Image (Millimetres_Per_Metre * abs (Estimates (Lobe) - Grid (Lobe, Best_Of (Lobe)).Support), 2)
                                    & " mm from the support point along its press ("
                                    & Image (Millimetres_Per_Metre * Along, 2) & " mm along the press"
                                    & (if Sigmas (Lobe) < Real'Last and then Sigmas (Lobe) > 0.0
                                       then ", " & Image (Along / Sigmas (Lobe), 2) & " of its stated sigma "
                                            & Image (Millimetres_Per_Metre * Sigmas (Lobe), 2) & " mm"
                                       else ", no sigma stated")
                                    & "), "
                                    & Image (Millimetres_Per_Metre * abs (Estimates (Lobe) - Grid (Lobe, Best_Of (Lobe)).Farthest), 2)
                                    & " mm from the farthest vertex along the approach");
                              end;
                           end if;
                        end loop;
                     end;
                  end if;
               end;
            end loop;
         end;
         <<Next_Hand>>
      end loop;
   end Score_Hands;

   ---------------------------------------------------------------------------
   --  The scorer's own check: estimates made from the truth itself and seen
   --  through a known similarity and offset. A correct scorer finds the links
   --  they came from (or links rigid with them), the scale it was given, and
   --  no error.

   procedure Synthesize (Closer : String; Tool_Links : Name_Vectors.Vector) is
      Known : Parameters := [others => 0.0];
      Rs, Rx : Mat3;
      S      : Real;
      Ts, Tx : Vec3;
      Grid_Columns : constant := 15;
      Grid_Rows    : constant := 11;
   begin
      Known (1) := Log (0.137_931);
      Known (2 .. 4) := [0.1, 0.7, -0.4];
      Known (5 .. 7) := [0.5, -0.3, 0.2];
      Known (8 .. 10) := [0.3, -0.2, 0.5];
      Known (11 .. 13) := [0.01, 0.02, -0.03];
      S := Scale_Of (Known);
      Rs := S_Rotation (Known);
      Ts := S_Translation (Known);
      Rx := X_Rotation (Known);
      Tx := X_Translation (Known);
      for B in Recorded.First_Index .. Recorded.Last_Index loop
         if Recorded (B).Line > 0 then
            declare
               L : constant Truth_Line := Truth (Recorded (B).Line);
               E : Estimated_Beat;
            begin
               for Link of Tool_Links loop
                  declare
                     T : constant Rigid := L.Links (Link);
                     R : constant Mat3 := Transpose (Rs) * T.Rotation * Transpose (Rx);
                  begin
                     E.Tools.Append (Rigid'(Rotation    => R,
                                      Translation => Transpose (Rs) * (T.Translation - Ts) / S - R * Tx));
                  end;
               end loop;
               for Eye in 1 .. Natural (Camera_Paths.Length) loop
                  declare
                     T : constant Rigid := L.Cameras (Camera_Name (Eye));
                  begin
                     E.Eyes.Append (Rigid'(Rotation => Transpose (Rs) * T.Rotation,
                                     Translation => Transpose (Rs) * (T.Translation - Ts) / S));
                  end;
               end loop;
               Estimated.Include (B, E);
            end;
         end if;
      end loop;
      for Eye in 1 .. Natural (Camera_Paths.Length) loop
         declare
            Lens_Of : constant Lens := Lenses (Camera_Name (Eye));
            G       : Ray_Vectors.Vector;
         begin
            for Gu in 1 .. Grid_Columns loop
               for Gv in 1 .. Grid_Rows loop
                  declare
                     U : constant Real := Real (Lens_Of.Width) * Real (Gu) / Real (Grid_Columns + 1);
                     V : constant Real := Real (Lens_Of.Height) * Real (Gv) / Real (Grid_Rows + 1);
                  begin
                     G.Append (Ray_Sample'(U => U, V => V, D => True_Ray (Lens_Of, U, V)));
                  end;
               end loop;
            end loop;
            Rays.Append (G);
         end;
      end loop;
      --  One hand on the first tool link: every link that moves against it is
      --  a lobe, measured at the beats where the closer reads least and most.
      declare
         Tool_Link : constant String := Tool_Links (1);
         Prefix    : constant String := Tool_Link (Tool_Link'First .. Ada.Strings.Fixed.Index (Tool_Link, "/"));
         H         : Hand_Estimate := (Arm => 1, Closer => To_Unbounded_String (Closer), Lobes => <>);
         Low, High : Natural := Natural'Last;
      begin
         for B in Recorded.First_Index .. Recorded.Last_Index loop
            if Recorded (B).Line > 0 and then Recorded (B).Readings.Contains (Closer) then
               declare
                  R : constant Real := Value_Maps.Element (Recorded (B).Readings, Closer) (1);
               begin
                  if Low = Natural'Last or else R < Value_Maps.Element (Recorded (Low).Readings, Closer) (1) then
                     Low := B;
                  end if;
                  if High = Natural'Last or else R > Value_Maps.Element (Recorded (High).Readings, Closer) (1) then
                     High := B;
                  end if;
               end;
            end if;
         end loop;
         for C in Link_Keys.Iterate loop
            declare
               Name : constant String := Key_Maps.Key (C);
               function At_Beat (B : Natural) return Tip_Estimate is
                  L        : constant Truth_Line := Truth (Recorded (B).Line);
                  In_Tool  : constant Rigid := Inverse (L.Links (Tool_Link)) * L.Links (Name);
                  Points   : constant Vertex_Vectors.Vector := Collision_Points (Key_Maps.Element (C));
               begin
                  declare
                     Press : constant Vec3 := Unit (Centroid (Points, In_Tool));
                     Tip   : constant Vec3 := Support (Points, In_Tool, Press);
                  begin
                     --  Back into the driver's tool frame: p_D = Rx p_L / s + tx.
                     return (Tip     => Rx * Tip / S + Tx,
                             Press   => Rx * Press,
                             Reading => Real_Holders.To_Holder (Value_Maps.Element (Recorded (B).Readings, Closer)),
                             Confirmed => True, others => <>);
                  end;
               end At_Beat;
            begin
               if Name /= Tool_Link and then Name'Length > Prefix'Length
                 and then Name (Name'First .. Name'First + Prefix'Length - 1) = Prefix
                 and then Low /= Natural'Last and then Is_Finger (Name, Tool_Link)
               then
                  H.Lobes.Append (Lobe_Estimate'(Open => At_Beat (High), Closed => At_Beat (Low)));
               end if;
            end;
         end loop;
         Hands.Append (H);
         Ada.Text_IO.Put_Line ("check: estimates made from the truth at scale " & Image (S, 6) & " m per unit,"
                               & Natural'Image (Natural (Tool_Links.Length)) & " tool links,"
                               & Natural'Image (Natural (H.Lobes.Length)) & " lobes on the first");
      end;
   end Synthesize;


   ---------------------------------------------------------------------------
   --  Where the truth puts each link in an eye: checks the truth's camera
   --  model against the images themselves (draw the pixels on the frame).

   procedure True_Pixel (L : Lens; In_Eye : Vec3; U, V : out Real; Visible : out Boolean) is
   begin
      Visible := In_Eye (3) > 0.0;
      U := 0.0;
      V := 0.0;
      if L.Has_K then
         if Visible then
            U := L.K (1, 1) * In_Eye (1) / In_Eye (3) + L.K (1, 3);
            V := L.K (2, 2) * In_Eye (2) / In_Eye (3) + L.K (2, 3);
         end if;
         return;
      end if;
      declare
         --  F-theta: invert angle (r) by bisection; it rises with r over the image.
         C     : constant Real_Array := L.Coefficients.Element;
         Rho   : constant Real := Sqrt (In_Eye (1) ** 2 + In_Eye (2) ** 2);
         Theta : constant Real := Arctan (Rho, In_Eye (3));
         Lo    : Real := 0.0;
         Hi    : Real := Real (Natural'Max (L.Width, L.Height));
         function Angle_At (R : Real) return Real is
            A : Real := 0.0;
         begin
            for I in reverse C'Range loop
               A := A * R + C (I);
            end loop;
            return A;
         end Angle_At;
      begin
         Visible := Theta <= L.Max_Field / 2.0 / Degrees_Per_Radian and then Angle_At (Hi) >= Theta;
         if not Visible or else Rho = 0.0 then
            U := Real (L.Width) / 2.0;
            V := Real (L.Height) / 2.0;
            return;
         end if;
         for Step in 1 .. Real'Machine_Mantissa loop
            if Angle_At ((Lo + Hi) / 2.0) < Theta then
               Lo := (Lo + Hi) / 2.0;
            else
               Hi := (Lo + Hi) / 2.0;
            end if;
         end loop;
         U := Real (L.Width) / 2.0 + Lo * In_Eye (1) / Rho;
         V := Real (L.Height) / 2.0 + Lo * In_Eye (2) / Rho;
      end;
   end True_Pixel;

   procedure Project (Beat : Natural; Eye : Positive) is
      Name : constant String := Camera_Name (Eye);
   begin
      if Name = "" or else Beat > Recorded.Last_Index or else Recorded (Beat).Line = 0 then
         Ada.Text_IO.Put_Line ("no truth for that eye at that beat");
         return;
      end if;
      declare
         L   : constant Truth_Line := Truth (Recorded (Beat).Line);
         Cam : constant Rigid := L.Cameras (Name);
      begin
         for C in L.Links.Iterate loop
            declare
               P : constant Vec3 := Inverse (Cam) * Pose_Maps.Element (C).Translation;
               U, V : Real;
               Visible : Boolean;
            begin
               True_Pixel (Lenses (Name), P, U, V, Visible);
               if Visible and then U in 0.0 .. Real (Lenses (Name).Width) and then V in 0.0 .. Real (Lenses (Name).Height)
               then
                  Ada.Text_IO.Put_Line (Pose_Maps.Key (C) & " " & Image (U, 1) & " " & Image (V, 1));
               end if;
            end;
         end loop;
         for C in L.Objects.Iterate loop
            declare
               P : constant Vec3 := Inverse (Cam) * Pose_Maps.Element (C).Translation;
               U, V : Real;
               Visible : Boolean;
            begin
               True_Pixel (Lenses (Name), P, U, V, Visible);
               if Visible and then U in 0.0 .. Real (Lenses (Name).Width) and then V in 0.0 .. Real (Lenses (Name).Height)
               then
                  Ada.Text_IO.Put_Line ("object:" & Pose_Maps.Key (C) & " " & Image (U, 1) & " " & Image (V, 1));
               end if;
            end;
         end loop;
      end;
   end Project;


   use Ada.Command_Line;

begin
   if Argument_Count = 5 and then Argument (1) = "--project" then
      Read_Truth (Argument (3));
      Read_Recording (Argument (2));
      Pair_Beats;
      Project (Natural'Value (Argument (4)), Positive'Value (Argument (5)));
      return;
   end if;
   if Argument_Count >= 5 and then Argument (1) = "--check" then
      Read_Truth (Argument (3));
      Read_Recording (Argument (2));
      Pair_Beats;
      declare
         Links : Name_Vectors.Vector;
      begin
         for I in 5 .. Argument_Count loop
            Links.Append (Argument (I));
         end loop;
         Synthesize (Argument (4), Links);
      end;
   elsif Argument_Count = 3 then
      Read_Truth (Argument (3));
      Read_Recording (Argument (2));
      Pair_Beats;
      Read_Estimates (Argument (1));
   else
      Line (Core, "usage: score ESTIMATES RECORDING TRUTH, or score --check RECORDING TRUTH CLOSER LINK...");
      Set_Exit_Status (Failure);
      return;
   end if;
   Score_Arms;
   Score_Eyes;
   Score_Rays;
   Score_Hands;
end Score;
