with Ada.Characters.Handling;
with Ada.Containers;
with Ada.Text_IO;
with GNAT.OS_Lib;
with Driver.Json;
with Driver.Log;
with Driver.Robot.Flow;
with Driver.Robot.Graph;
with Driver.Robot.Kinematics;
with Driver.Robot.Lockin;

package body Driver.Robot.Body_File is

   use Ada.Strings.Unbounded;
   use type Driver.Json.Node;
   use type Driver.Json.Kind;
   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;

   LF : constant Character := ASCII.LF;

   ---------------------------------------------------------------------------
   --  Writing

   function Num (X : Real) return String renames Driver.Json.Number_Image;

   function Int (N : Integer) return String renames Driver.Log.Image;

   function Word (Image : String) return String is
     (Driver.Json.Quote (Ada.Characters.Handling.To_Lower (Image)));

   function Flag (B : Boolean) return String is (if B then "true" else "false");

   function Separator (First : Boolean) return String is (if First then "" else ", ");

   function Reals (V : Real_Vectors.Vector) return String is
      T : Unbounded_String := To_Unbounded_String ("[");
   begin
      for I in V.First_Index .. V.Last_Index loop
         Append (T, Separator (I = V.First_Index) & Num (V (I)));
      end loop;
      Append (T, "]");
      return To_String (T);
   end Reals;

   --  Per channel of an arm, the standard deviation of how far the view moves per unit as its keyframes
   --  measured it (Kinematics.Measure_Shifts), 0 where it is not measured, and which are.
   function Shift_Sigmas_Of (R : Arm_Evidence) return Real_Vectors.Vector is
      V : Real_Vectors.Vector;
   begin
      for S of R.Shift_Sigmas loop
         V.Append (if S < Real'Last then S else 0.0);
      end loop;
      return V;
   end Shift_Sigmas_Of;

   function Shifts_Known_Of (R : Arm_Evidence) return Flag_Vectors.Vector is
      V : Flag_Vectors.Vector;
   begin
      for S of R.Shift_Sigmas loop
         V.Append (S < Real'Last);
      end loop;
      return V;
   end Shifts_Known_Of;

   --  Every channel's creep (Model.Drift), one for each noise: 0 where it was not measured.
   function Drift_Of (M : Model) return Real_Vectors.Vector is
      V : Real_Vectors.Vector;
   begin
      for K in 0 .. Natural (M.Noise.Length) - 1 loop
         V.Append (if K < Natural (M.Drift.Length) then M.Drift (K) else 0.0);
      end loop;
      return V;
   end Drift_Of;

   --  Per channel of a group, the least ask a push moved it for (Group_Stream.Executed), 0 where none did yet,
   --  and which did.
   function Executed_Of (S : Group_Stream) return Real_Vectors.Vector is
      V : Real_Vectors.Vector;
   begin
      for X of S.Executed loop
         V.Append (if X < Real'Last then X else 0.0);
      end loop;
      return V;
   end Executed_Of;

   function Executed_Known_Of (S : Group_Stream) return Flag_Vectors.Vector is
      V : Flag_Vectors.Vector;
   begin
      for X of S.Executed loop
         V.Append (X < Real'Last);
      end loop;
      return V;
   end Executed_Known_Of;

   function Counts (V : Count_Vectors.Vector) return String is
      T : Unbounded_String := To_Unbounded_String ("[");
   begin
      for I in V.First_Index .. V.Last_Index loop
         Append (T, Separator (I = V.First_Index) & Int (V (I)));
      end loop;
      Append (T, "]");
      return To_String (T);
   end Counts;

   function Flags (V : Flag_Vectors.Vector) return String is
      T : Unbounded_String := To_Unbounded_String ("[");
   begin
      for I in V.First_Index .. V.Last_Index loop
         Append (T, Separator (I = V.First_Index) & Flag (V (I)));
      end loop;
      Append (T, "]");
      return To_String (T);
   end Flags;

   function Stops (V : Stop_Vectors.Vector) return String is
      T : Unbounded_String := To_Unbounded_String ("[");
   begin
      for I in V.First_Index .. V.Last_Index loop
         Append (T, Separator (I = V.First_Index) & "{""channel"": " & Int (V (I).Channel) & ", ""up"": " & Flag (V (I).Up)
                 & ", ""value"": " & Num (V (I).Value) & ", ""pose"": " & Reals (V (I).Pose) & "}");
      end loop;
      Append (T, "]");
      return To_String (T);
   end Stops;

   function Three (X : Vec3) return String is ("[" & Num (X (1)) & ", " & Num (X (2)) & ", " & Num (X (3)) & "]");

   function Nine (R : Mat3) return String is
     ("[" & Num (R (1, 1)) & ", " & Num (R (1, 2)) & ", " & Num (R (1, 3)) & ", "
      & Num (R (2, 1)) & ", " & Num (R (2, 2)) & ", " & Num (R (2, 3)) & ", "
      & Num (R (3, 1)) & ", " & Num (R (3, 2)) & ", " & Num (R (3, 3)) & "]");

   --  Poses as a list of their rotations and centres.
   function Poses (V : Rigid_Vectors.Vector) return String is
      T : Unbounded_String := To_Unbounded_String ("[");
   begin
      for I in V.First_Index .. V.Last_Index loop
         Append (T, (if I > V.First_Index then ", " else "") & "{""rotation"": " & Nine (V (I).Rotation)
                 & ", ""centre"": " & Three (V (I).Translation) & "}");
      end loop;
      Append (T, "]");
      return To_String (T);
   end Poses;

   function Text (M : Model) return String is
      T : Unbounded_String;

      procedure Add (S : String) is
      begin
         Append (T, S);
      end Add;
   begin
      --  The key: the shape of what the robot reports.
      Add ("{""key"": {""groups"": [");
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         Add (Separator (G = M.Groups.First_Index) & "{""size"": " & Int (M.Groups (G).Size)
              & ", ""commandable"": " & Flag (M.Groups (G).Commandable) & "}");
      end loop;
      Add ("], ""eyes"": [");
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         Add (Separator (E = M.Eyes.First_Index) & "{""width"": " & Int (M.Eyes (E).Grid.Width)
              & ", ""height"": " & Int (M.Eyes (E).Grid.Height) & "}");
      end loop;
      Add ("]}," & LF & " ""beats"": " & Int (M.Beats) & "," & LF);

      --  The readings' noise, every channel of every group in group order.
      Add (" ""noise"": {""method"": " & Int (Noise_Method) & ", ""sigma"": " & Reals (M.Noise)
           & ", ""freedom"": " & Counts (M.Noise_Freedom) & ", ""drift"": " & Reals (Drift_Of (M)) & "}," & LF);

      --  The readings every channel has moved through.
      Add (" ""travel"": {""method"": " & Int (Travel_Method) & ", ""groups"": [");
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         Add (Separator (G = M.Groups.First_Index) & "{""low"": " & Reals (M.Groups (G).Low_Seen)
              & ", ""high"": " & Reals (M.Groups (G).High_Seen) & "}");
      end loop;
      Add ("]}," & LF);

      --  The ends the arm showed by stopping (Driver.Robot.Motion.Note_Stopped): per channel where it ended, the furthest
      --  down and up, and whether it did, and the stops not yet found again from another pose, each with the pose it was
      --  made from. A group none of whose channels showed an end has empty vectors.
      Add (" ""ends"": {""method"": " & Int (Ends_Method) & ", ""groups"": [");
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         Add (Separator (G = M.Groups.First_Index) & "{""low"": " & Reals (M.Groups (G).Stopped_Low)
              & ", ""has_low"": " & Flags (M.Groups (G).Has_Stopped_Low)
              & ", ""high"": " & Reals (M.Groups (G).Stopped_High)
              & ", ""has_high"": " & Flags (M.Groups (G).Has_Stopped_High)
              & ", ""stops"": " & Stops (M.Groups (G).Stops) & "}");
      end loop;
      Add ("]}," & LF);

      --  The step responses: the longest wait for an answer, every free
      --  push's shortfall, and per channel the least ask a push moved it for.
      Add (" ""steps"": {""method"": " & Int (Steps_Method) & ", ""groups"": [");
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         Add (Separator (G = M.Groups.First_Index) & "{""delay"": " & Int (M.Groups (G).Delay_Beats)
              & ", ""delay_known"": " & Flag (M.Groups (G).Delay_Known)
              & ", ""free_shortfalls"": " & Reals (M.Groups (G).Free_Shortfalls)
              & ", ""pause"": " & Int (M.Groups (G).Pause)
              & ", ""executed"": " & Reals (Executed_Of (M.Groups (G)))
              & ", ""executed_known"": " & Flags (Executed_Known_Of (M.Groups (G)))
              & ", ""held_off"": " & Reals (M.Groups (G).Held_Off)
              & ", ""free_short_most"": " & Reals (M.Groups (G).Free_Short_Most)
              & ", ""free_short_count"": " & Counts (M.Groups (G).Free_Short_Count) & "}");
      end loop;
      Add ("]}," & LF);

      --  The image lags.
      Add (" ""lags"": {""method"": " & Int (Lag_Method) & ", ""eyes"": [");
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         Add (Separator (E = M.Eyes.First_Index) & "{""value"": " & Int (Image_Lag (M, E))
              & ", ""known"": " & Flag (Lag_Known (M, E))
              & ", ""settling"": " & Int (Longest_Settling (M, E)) & "}");
      end loop;
      Add ("]}," & LF);

      --  The lock-in: per eye, its cells' rest noise, what every channel's
      --  push does to every cell, and each group's effect on the eye.
      Add (" ""responses"": {""method"": " & Int (Lockin_Method) & ", ""eyes"": [");
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            S : Eye_Stream renames M.Eyes (E);
         begin
            Add (Separator (E = M.Eyes.First_Index) & LF & "  {""rest_factor"": " & Num (S.Rest_Factor)
                 & ", ""rest_counts_known"": " & Flag (S.Rest_Counts_Known)
                 & ", ""rest_count_max"": " & Int (S.Rest_Count_Max)
                 & ", ""rest_count_level"": " & Num (S.Rest_Count_Level)
                 & ", ""rest_count_beats"": " & Int (S.Rest_Count_Beats)
                 & ", ""rest_count_above"": " & Int (S.Rest_Count_Above)
                 & ", ""rest_counts"": " & Counts (S.Rest_Counts)
                 & ", ""noise"": " & Reals (S.Noise)
                 & ", ""textured"": " & Flags (S.Textured)
                 & ", ""kept_groups"": " & Counts (S.Kept_Groups)
                 & ", ""kept_channels"": " & Counts (S.Kept_Channels)
                 & ", ""gains"": " & Reals (S.Gains)
                 & ", ""gain_variances"": " & Reals (S.Gain_Variances)
                 & ", ""shifts"": " & Reals (S.Shifts)
                 & ", ""effects"": [");
            for G in M.Groups.First_Index .. M.Groups.Last_Index loop
               declare
                  F : constant Eye_Effect := Graph.Effect (M, G, E);
               begin
                  Add (Separator (G = M.Groups.First_Index) & "{""verdict"": " & Word (Eye_Response'Image (F.Verdict))
                       & ", ""responding"": " & Int (F.Responding) & ", ""textured"": " & Int (F.Textured)
                       & ", ""fraction"": " & Num (F.Fraction.Value) & ", ""sigma"": " & Num (F.Fraction.Sigma)
                       & ", ""freedom"": " & Int (F.Fraction.Degrees_Of_Freedom)
                       & ", ""could_be_whole"": " & Int ((if F.Could_Be_Whole then 1 else 0)) & "}");
               end;
            end loop;
            Add ("]}");
         end;
      end loop;
      Add ("]}," & LF);

      --  The graph: every group's role, the arms, the carrier, every eye's mount.
      Add (" ""graph"": {""method"": " & Int (Graph_Method) & ", ""groups"": [");
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         Add (Separator (G = M.Groups.First_Index) & "{""role"": " & Word (Group_Role'Image (Role (M, G)))
              & ", ""arm"": " & Int (if G <= M.Graph.Arm_Of.Last_Index then Integer (M.Graph.Arm_Of (G)) else 0)
              & ", ""breach"": " & Int (Contract_Breach (M, G))
              & ", ""seen_at"": " & Reals (M.Groups (G).Seen_At) & "}");
      end loop;
      Add ("], ""arms"": [");
      for A in M.Graph.Arms.First_Index .. M.Graph.Arms.Last_Index loop
         Add (Separator (A = M.Graph.Arms.First_Index) & Int (Integer (M.Graph.Arms (A))));
      end loop;
      Add ("], ""arm_eyes"": [");
      for A in M.Graph.Arms.First_Index .. M.Graph.Arms.Last_Index loop
         Add (Separator (A = M.Graph.Arms.First_Index)
              & Int (if A <= M.Graph.Arm_Eye.Last_Index then Integer (M.Graph.Arm_Eye (A)) else 0));
      end loop;
      Add ("], ""carrier"": " & Int (Integer (M.Graph.Carrier)) & ", ""mounts"": [");
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            Mt : constant Mount := Eye_Mount (M, E);
         begin
            Add (Separator (E = M.Eyes.First_Index) & "{""kind"": " & Word (Mount_Kind'Image (Mt.Kind))
                 & ", ""arm"": " & Int (if Mt.Kind = Arm_Carried then Integer (Mt.Arm) else 0) & "}");
         end;
      end loop;
      Add ("]}," & LF);

      --  The kinematics: every arm's fit with its lens and covariance, and
      --  the table its eye found, in its own frame.
      Add (" ""kinematics"": {""method"": " & Int (Kinematics_Method) & ", ""arms"": [");
      for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
         declare
            R : Arm_Evidence renames M.Kinematics (K);
            F : Arm_Fit renames R.Result;
         begin
            Add (Separator (K = M.Kinematics.First_Index) & LF & "  {""arm"": " & Int (Integer (R.Arm))
                 & ", ""group"": " & Int (Integer (R.Group)) & ", ""eye"": " & Int (Integer (R.Eye))
                 & ", ""shifts"": " & Reals (R.Shifts) & ", ""shift_sigmas"": " & Reals (Shift_Sigmas_Of (R))
                 & ", ""shifts_known"": " & Flags (Shifts_Known_Of (R)) & ", ""shift_points"": " & Counts (R.Shift_Points)
                 & ", ""shifts_seen"": " & Flags (R.Shift_Seen) & ", ""shift_told"": " & Counts (R.Shift_Told)
                 & ", ""match_noise"": " & Num (Kinematics.Match_Noise (M, R.Arm))
                 & ", ""fitted"": " & Flag (F.Fitted) & ", ""reference"": " & Reals (F.Reference)
                 & ", ""chain"": " & Counts (F.Chain) & ", ""joints"": [");
            for J in F.Joints.First_Index .. F.Joints.Last_Index loop
               Add (Separator (J = F.Joints.First_Index) & "{""w"": " & Three (F.Joints (J).W)
                    & ", ""p"": " & Three (F.Joints (J).P) & ", ""c"": " & Num (F.Joints (J).C)
                    & ", ""slide"": " & Flag (F.Joints (J).Slide) & "}");
            end loop;
            Add ("], ""lens"": {""fx"": " & Num (F.Lens.Fx) & ", ""fy"": " & Num (F.Lens.Fy)
                 & ", ""cx"": " & Num (F.Lens.Cx) & ", ""cy"": " & Num (F.Lens.Cy)
                 & ", ""k1"": " & Num (F.Lens.K1) & ", ""k2"": " & Num (F.Lens.K2)
                 & "}, ""used"": " & Int (F.Used) & ", ""median_px"": " & Num (F.Median_Px)
                 & ", ""sigma_px"": " & Num (F.Sigma_Px) & ", ""matches"": " & Int (F.Matches)
                 & ", ""why"": " & Driver.Json.Quote (To_String (F.Why))
                 & ", ""covariance"": " & Reals (F.Covariance)
                 & ", ""unit_changes"": " & Reals (F.Unit_Changes)
                 & ", ""table"": {""centre"": " & Three (F.Table.Centre) & ", ""normal"": " & Three (F.Table.Normal)
                 & ", ""tangent"": " & Three (F.Table.Tangent_1) & ", ""offset_sigma"": " & Num (F.Table.Offset_Sigma)
                 & ", ""tilt"": [" & Num (F.Table.Tilt_11) & ", " & Num (F.Table.Tilt_12) & ", "
                 & Num (F.Table.Tilt_22) & "], ""points"": " & Int (F.Table.Points)
                 & ", ""scatter"": " & Num (F.Table.Scatter) & ", ""response"": " & Reals (F.Table_Response)
                 & ", ""own"": " & Nine (F.Table_Own) & "}"
                 & ", ""placed"": " & Flag (F.Placed)
                 & ", ""placement"": {""rotation"": " & Nine (F.Placement.Rotation)
                 & ", ""centre"": " & Three (F.Placement.Translation)
                 & ", ""scale"": " & Num (F.Scale) & ", ""scale_sigma"": " & Num (F.Scale_Sigma)
                 & ", ""covariance"": " & Reals (F.Placement_Covariance)
                 & ", ""px"": " & Num (F.Placed_Px) & ", ""points"": " & Int (F.Placed_Points)
                 & ", ""through"": " & Int (F.Placed_Through) & "}}");
         end;
      end loop;
      --  The eyes that stand still in the world: the lens and place of each, with the covariance of both.
      Add ("], ""fixed"": [");
      declare
         First : Boolean := True;
      begin
         for E in M.Fixed_Eyes.First_Index .. M.Fixed_Eyes.Last_Index loop
            declare
               F : Fixed_Fit renames M.Fixed_Eyes (E);
            begin
               if F.Judged then
                  Add (Separator (First) & LF & "  {""eye"": " & Int (Integer (E)) & ", ""known"": " & Flag (F.Known)
                       & ", ""arm"": " & Int (Integer (F.Arm)) & ", ""lens"": {""fx"": " & Num (F.Lens.Fx)
                       & ", ""fy"": " & Num (F.Lens.Fy) & ", ""cx"": " & Num (F.Lens.Cx)
                       & ", ""cy"": " & Num (F.Lens.Cy)
                       & ", ""k1"": " & Num (F.Lens.K1) & ", ""k2"": " & Num (F.Lens.K2) & "}"
                       & ", ""rotation"": " & Nine (F.Pose.Rotation) & ", ""centre"": " & Three (F.Pose.Translation)
                       & ", ""covariance"": " & Reals (F.Covariance) & ", ""used"": " & Int (F.Used)
                       & ", ""offered"": " & Int (F.Offered) & ", ""sigma_px"": " & Num (F.Sigma_Px)
                       & ", ""arms"": " & Counts (F.Arms) & ", ""poses"": " & Poses (F.Poses)
                       & ", ""joint"": " & Reals (F.Joint)
                       & ", ""why"": " & Driver.Json.Quote (To_String (F.Why)) & "}");
                  First := False;
               end if;
            end;
         end loop;
      end;
      Add ("]}}" & LF);
      return To_String (T);
   end Text;

   procedure Write (M : Model; Path : String; Ok : out Boolean) is
      Part : constant String := Path & ".part";
      F    : Ada.Text_IO.File_Type;
   begin
      Ada.Text_IO.Create (F, Ada.Text_IO.Out_File, Part);
      Ada.Text_IO.Put (F, Text (M));
      Ada.Text_IO.Close (F);
      GNAT.OS_Lib.Rename_File (Part, Path, Ok);
   exception
      when Ada.Text_IO.Name_Error | Ada.Text_IO.Use_Error =>
         Ok := False;
   end Write;

   ---------------------------------------------------------------------------
   --  Reading

   procedure Read
     (M    : in out Model;
      Text : String;
      Ok   : out Boolean;
      Why  : out Unbounded_String)
   is
      use Driver.Json;
      Doc    : Document;
      Parsed : Boolean;
      Top    : Node;

      function Field (N : Node; Name : String) return Node is (Lookup (Doc, N, Name));
      function Size (N : Node) return Natural is (Count (Doc, N));
      function Item (N : Node; I : Positive) return Node is (Element (Doc, N, I));
      function Is_Number (N : Node) return Boolean is (Kind_Of (Doc, N) = Number_Value);
      function Value (N : Node) return Real is (Number (Doc, N));
      function Whole (N : Node) return Integer is (if Is_Number (N) then Integer (Number (Doc, N)) else -1);
      function Whole (N : Node; Name : String) return Integer is (Whole (Field (N, Name)));
      function Truth (N : Node; Name : String) return Boolean is (Is_True (Doc, Field (N, Name)));
      function Method_Is (N : Node; Version : Integer) return Boolean is (Whole (N, "method") = Version);

      function Reals_Of (N : Node) return Real_Vectors.Vector is
         V : Real_Vectors.Vector;
      begin
         for I in 1 .. Size (N) loop
            V.Append (Value (Item (N, I)));
         end loop;
         return V;
      end Reals_Of;

      function Counts_Of (N : Node) return Count_Vectors.Vector is
         V : Count_Vectors.Vector;
      begin
         for I in 1 .. Size (N) loop
            V.Append (Natural'Max (0, Whole (Item (N, I))));
         end loop;
         return V;
      end Counts_Of;

      function Flags_Of (N : Node) return Flag_Vectors.Vector is
         V : Flag_Vectors.Vector;
      begin
         for I in 1 .. Size (N) loop
            V.Append (Is_True (Doc, Item (N, I)));
         end loop;
         return V;
      end Flags_Of;

      function Three_Of (N : Node) return Vec3 is
        ([Value (Item (N, 1)), Value (Item (N, 2)), Value (Item (N, 3))]);

      function Nine_Of (N : Node) return Mat3 is
        ([[Value (Item (N, 1)), Value (Item (N, 2)), Value (Item (N, 3))],
          [Value (Item (N, 4)), Value (Item (N, 5)), Value (Item (N, 6))],
          [Value (Item (N, 7)), Value (Item (N, 8)), Value (Item (N, 9))]]);

      Restored : Stored_Flags := [others => False];
   begin
      Ok := False;
      Why := Null_Unbounded_String;
      Parse (Text, Doc, Parsed, Why);
      if not Parsed then
         Why := "not a body file: " & Why;
         return;
      end if;
      Top := Root (Doc);
      declare
         Key    : constant Node := Field (Top, "key");
         Groups : constant Node := Field (Key, "groups");
         Eyes   : constant Node := Field (Key, "eyes");
      begin
         if Kind_Of (Doc, Groups) /= Array_Value or else Kind_Of (Doc, Eyes) /= Array_Value then
            Why := To_Unbounded_String ("not a body file: it has no key");
            return;
         end if;
         if M.Groups.Is_Empty and then M.Eyes.Is_Empty then
            --  A model that has seen nothing yet takes the file's body.
            for I in 1 .. Size (Groups) loop
               M.Groups.Append (Group_Stream'(Size        => Natural'Max (0, Whole (Item (Groups, I), "size")),
                                              Commandable => Truth (Item (Groups, I), "commandable"),
                                              others      => <>));
            end loop;
            for I in 1 .. Size (Eyes) loop
               M.Eyes.Append (Eye_Stream'(Grid   => Flow.Grid_Of (Natural'Max (0, Whole (Item (Eyes, I), "width")),
                                                                  Natural'Max (0, Whole (Item (Eyes, I), "height"))),
                                          others => <>));
            end loop;
         else
            declare
               Same : Boolean := Size (Groups) = Natural (M.Groups.Length) and then Size (Eyes) = Natural (M.Eyes.Length);
            begin
               if Same then
                  for I in 1 .. Size (Groups) loop
                     Same := Same and then Whole (Item (Groups, I), "size") = M.Groups (Group_Id (I)).Size
                       and then Truth (Item (Groups, I), "commandable") = M.Groups (Group_Id (I)).Commandable;
                  end loop;
                  for I in 1 .. Size (Eyes) loop
                     Same := Same and then Whole (Item (Eyes, I), "width") = M.Eyes (Eye_Id (I)).Grid.Width
                       and then Whole (Item (Eyes, I), "height") = M.Eyes (Eye_Id (I)).Grid.Height;
                  end loop;
               end if;
               if not Same then
                  Why := To_Unbounded_String ("the file is another body's: its key does not match what the robot reports");
                  return;
               end if;
            end;
         end if;
      end;

      declare
         Group_Count : constant Natural := Natural (M.Groups.Length);
         Eye_Count   : constant Natural := Natural (M.Eyes.Length);
         Channels    : Natural := 0;
      begin
         for S of M.Groups loop
            Channels := Channels + S.Size;
         end loop;

         --  The readings' noise.
         declare
            N : constant Node := Field (Top, "noise");
         begin
            if Method_Is (N, Noise_Method) and then Size (Field (N, "sigma")) = Channels
              and then Size (Field (N, "freedom")) = Channels and then Size (Field (N, "drift")) = Channels
            then
               M.Noise := Reals_Of (Field (N, "sigma"));
               M.Noise_Freedom := Counts_Of (Field (N, "freedom"));
               M.Drift := Reals_Of (Field (N, "drift"));
               Restored (Stored_Noise) := True;
            end if;
         end;

         --  The readings every channel has moved through.
         declare
            N  : constant Node := Field (Top, "travel");
            Gs : constant Node := Field (N, "groups");
         begin
            if Method_Is (N, Travel_Method) and then Size (Gs) = Group_Count then
               for I in 1 .. Group_Count loop
                  declare
                     S    : Group_Stream renames M.Groups (Group_Id (I));
                     Low  : constant Real_Vectors.Vector := Reals_Of (Field (Item (Gs, I), "low"));
                     High : constant Real_Vectors.Vector := Reals_Of (Field (Item (Gs, I), "high"));
                  begin
                     if Natural (Low.Length) = S.Size and then Natural (High.Length) = S.Size then
                        if Natural (S.Low_Seen.Length) = S.Size and then Natural (S.High_Seen.Length) = S.Size then
                           for C in 0 .. S.Size - 1 loop
                              S.Low_Seen.Replace_Element (C, Real'Min (S.Low_Seen (C), Low (C)));
                              S.High_Seen.Replace_Element (C, Real'Max (S.High_Seen (C), High (C)));
                           end loop;
                        else
                           S.Low_Seen := Low;
                           S.High_Seen := High;
                        end if;
                     end if;
                  end;
               end loop;
               Restored (Stored_Travel) := True;
            end if;
         end;

         --  The ends the arm showed by stopping, on the noise (an end's sigma is its channel's).
         declare
            N  : constant Node := Field (Top, "ends");
            Gs : constant Node := Field (N, "groups");
         begin
            if Restored (Stored_Noise) and then Method_Is (N, Ends_Method) and then Size (Gs) = Group_Count then
               for I in 1 .. Group_Count loop
                  declare
                     S        : Group_Stream renames M.Groups (Group_Id (I));
                     Low      : constant Real_Vectors.Vector := Reals_Of (Field (Item (Gs, I), "low"));
                     High     : constant Real_Vectors.Vector := Reals_Of (Field (Item (Gs, I), "high"));
                     Has_Low  : constant Flag_Vectors.Vector := Flags_Of (Field (Item (Gs, I), "has_low"));
                     Has_High : constant Flag_Vectors.Vector := Flags_Of (Field (Item (Gs, I), "has_high"));
                  begin
                     if Natural (Low.Length) = Natural (Has_Low.Length) and then Natural (High.Length) = Natural (Has_High.Length)
                       and then Natural (Low.Length) = Natural (High.Length)
                       and then (Low.Is_Empty or else Natural (Low.Length) = S.Size)
                     then
                        --  What the model already holds stands beside what the file brings: the furthest stop of
                        --  each sense.
                        if Natural (S.Has_Stopped_Low.Length) /= Natural (Low.Length) then
                           S.Stopped_Low := Low;
                           S.Stopped_High := High;
                           S.Has_Stopped_Low := Has_Low;
                           S.Has_Stopped_High := Has_High;
                        else
                           for C in Low.First_Index .. Low.Last_Index loop
                              if Has_Low (C) and then (not S.Has_Stopped_Low (C) or else Low (C) < S.Stopped_Low (C)) then
                                 S.Has_Stopped_Low.Replace_Element (C, True);
                                 S.Stopped_Low.Replace_Element (C, Low (C));
                              end if;
                              if Has_High (C) and then (not S.Has_Stopped_High (C) or else High (C) > S.Stopped_High (C)) then
                                 S.Has_Stopped_High.Replace_Element (C, True);
                                 S.Stopped_High.Replace_Element (C, High (C));
                              end if;
                           end loop;
                        end if;
                        --  The stops not yet found again: the file's join the model's, each with the pose it was made from.
                        declare
                           Stops_Node : constant Node := Field (Item (Gs, I), "stops");
                        begin
                           for K in 1 .. Size (Stops_Node) loop
                              declare
                                 Record_Node : constant Node := Item (Stops_Node, K);
                                 Pose        : constant Real_Vectors.Vector := Reals_Of (Field (Record_Node, "pose"));
                                 Channel     : constant Integer := Whole (Record_Node, "channel");
                              begin
                                 if Channel >= 1 and then Channel <= S.Size and then Natural (Pose.Length) = S.Size then
                                    S.Stops.Append
                                      (Stop_Record'(Channel => Channel, Up => Is_True (Doc, Field (Record_Node, "up")),
                                                     Value => Value (Field (Record_Node, "value")), Pose => Pose));
                                 end if;
                              end;
                           end loop;
                        end;
                     end if;
                  end;
               end loop;
               Restored (Stored_Ends) := True;
            end if;
         end;

         --  The step responses, on the noise.
         declare
            N  : constant Node := Field (Top, "steps");
            Gs : constant Node := Field (N, "groups");
         begin
            if Restored (Stored_Noise) and then Method_Is (N, Steps_Method) and then Size (Gs) = Group_Count then
               for I in 1 .. Group_Count loop
                  declare
                     S : Group_Stream renames M.Groups (Group_Id (I));
                  begin
                     S.Delay_Beats := Natural'Max (0, Whole (Item (Gs, I), "delay"));
                     S.Delay_Known := Truth (Item (Gs, I), "delay_known");
                     S.Free_Shortfalls := Reals_Of (Field (Item (Gs, I), "free_shortfalls"));
                     --  A body file written before pauses were measured has none: the group pauses as it is seen to.
                     S.Pause := Natural'Max (0, Whole (Item (Gs, I), "pause"));
                     declare
                        Ex    : constant Real_Vectors.Vector := Reals_Of (Field (Item (Gs, I), "executed"));
                        Known : constant Flag_Vectors.Vector := Flags_Of (Field (Item (Gs, I), "executed_known"));
                     begin
                        S.Executed.Clear;
                        for J in Ex.First_Index .. Ex.Last_Index loop
                           S.Executed.Append (if J <= Known.Last_Index and then Known (J) then Ex (J) else Real'Last);
                        end loop;
                     end;
                     --  A body file written before the dead bands were measured where pushes came to rest has none:
                     --  the band is the least executed ask until a push comes to rest (Steps.Dead_Band).
                     S.Held_Off := Reals_Of (Field (Item (Gs, I), "held_off"));
                     --  And one written before the free pushes were kept channel by channel has none: a channel is judged
                     --  short once three free pushes asked it again (Steps.Finish).
                     S.Free_Short_Most := Reals_Of (Field (Item (Gs, I), "free_short_most"));
                     S.Free_Short_Count := Counts_Of (Field (Item (Gs, I), "free_short_count"));
                  end;
               end loop;
               Restored (Stored_Steps) := True;
            end if;
         end;

         --  The image lags, on the noise.
         declare
            N  : constant Node := Field (Top, "lags");
            Es : constant Node := Field (N, "eyes");
         begin
            if Restored (Stored_Noise) and then Method_Is (N, Lag_Method) and then Size (Es) = Eye_Count then
               M.Lags.Clear;
               M.Lag_Known.Clear;
               M.Settling_Kept.Clear;
               for I in 1 .. Eye_Count loop
                  M.Lags.Append (Whole (Item (Es, I), "value"));
                  M.Lag_Known.Append (Truth (Item (Es, I), "known"));
                  M.Settling_Kept.Append (Natural'Max (0, Whole (Item (Es, I), "settling")));
               end loop;
               Restored (Stored_Lags) := True;
            end if;
         end;

         --  The lock-in and the effects, on the noise and the lags.
         declare
            N  : constant Node := Field (Top, "responses");
            Es : constant Node := Field (N, "eyes");
         begin
            if Restored (Stored_Noise) and then Restored (Stored_Lags) and then Method_Is (N, Lockin_Method)
              and then Size (Es) = Eye_Count
            then
               M.Graph.Effects.Clear;
               M.Graph.Effects.Append (Eye_Effect'(others => <>), Ada.Containers.Count_Type (Group_Count * Eye_Count));
               for I in 1 .. Eye_Count loop
                  declare
                     X : constant Node := Item (Es, I);
                     S : Eye_Stream renames M.Eyes (Eye_Id (I));
                     F : constant Node := Field (X, "effects");
                  begin
                     S.Rest_Factor := Value (Field (X, "rest_factor"));
                     S.Rest_Counts_Known := Truth (X, "rest_counts_known");
                     S.Rest_Count_Max := Natural'Max (0, Whole (X, "rest_count_max"));
                     S.Rest_Count_Level := Value (Field (X, "rest_count_level"));
                     S.Rest_Count_Beats := Natural'Max (0, Whole (X, "rest_count_beats"));
                     S.Rest_Count_Above := Natural'Max (0, Whole (X, "rest_count_above"));
                     S.Rest_Counts := Counts_Of (Field (X, "rest_counts"));
                     S.Noise := Reals_Of (Field (X, "noise"));
                     S.Textured := Flags_Of (Field (X, "textured"));
                     S.Kept_Groups := Counts_Of (Field (X, "kept_groups"));
                     S.Kept_Channels := Counts_Of (Field (X, "kept_channels"));
                     S.Gains := Reals_Of (Field (X, "gains"));
                     S.Gain_Variances := Reals_Of (Field (X, "gain_variances"));
                     S.Shifts := Reals_Of (Field (X, "shifts"));
                     Lockin.Total_Gains (S);
                     for G in 1 .. Natural'Min (Size (F), Group_Count) loop
                        declare
                           Y : constant Node := Item (F, G);
                        begin
                           M.Graph.Effects.Replace_Element
                             ((G - 1) * Eye_Count + I,
                              (Verdict    => Eye_Response'Value (Driver.Json.Text (Doc, Field (Y, "verdict"))),
                               Responding => Natural'Max (0, Whole (Y, "responding")),
                               Textured   => Natural'Max (0, Whole (Y, "textured")),
                               Fraction   => (Value              => Value (Field (Y, "fraction")),
                                              Sigma              => Value (Field (Y, "sigma")),
                                              Degrees_Of_Freedom => Natural'Max (0, Whole (Y, "freedom"))),
                               Resting    => 0.0,
                               Rest_Pull  => 0.0,
                               --  Absent from a file written before it: as then, the eye may ride on the group.
                               Could_Be_Whole => Whole (Y, "could_be_whole") /= 0));
                        end;
                     end loop;
                  end;
               end loop;
               Restored (Stored_Responses) := True;
            end if;
         end;

         --  The graph, on the lock-in.
         declare
            N  : constant Node := Field (Top, "graph");
            Gs : constant Node := Field (N, "groups");
            As : constant Node := Field (N, "arms");
            Ms : constant Node := Field (N, "mounts");
         begin
            if Restored (Stored_Responses) and then Method_Is (N, Graph_Method) and then Size (Gs) = Group_Count
              and then Size (Ms) = Eye_Count
            then
               M.Graph.Roles.Clear;
               M.Graph.Arm_Of.Clear;
               M.Graph.Breach.Clear;
               for I in 1 .. Group_Count loop
                  M.Graph.Roles.Append (Group_Role'Value (Driver.Json.Text (Doc, Field (Item (Gs, I), "role"))));
                  M.Graph.Arm_Of.Append (Arm_Id'Base (Natural'Max (0, Whole (Item (Gs, I), "arm"))));
                  M.Graph.Breach.Append (Natural'Max (0, Whole (Item (Gs, I), "breach")));
                  M.Groups (Group_Id (I)).Seen_At := Reals_Of (Field (Item (Gs, I), "seen_at"));
               end loop;
               M.Graph.Arms.Clear;
               M.Graph.Arm_Eye.Clear;
               for I in 1 .. Size (As) loop
                  M.Graph.Arms.Append (Group_Id (Whole (Item (As, I))));
                  --  The eye an arm of a split group is for; none in a body file of before the split (and for every arm
                  --  of a whole group).
                  M.Graph.Arm_Eye.Append
                    (if Size (Field (N, "arm_eyes")) = Size (As) then Eye_Id'Base (Natural'Max (0, Whole (Item (Field (N, "arm_eyes"), I))))
                     else 0);
               end loop;
               M.Graph.Carrier := Group_Id'Base (Natural'Max (0, Whole (N, "carrier")));
               M.Graph.Mounts.Clear;
               for I in 1 .. Eye_Count loop
                  declare
                     Kind : constant Mount_Kind := Mount_Kind'Value (Driver.Json.Text (Doc, Field (Item (Ms, I), "kind")));
                  begin
                     M.Graph.Mounts.Append
                       (Mount'(case Kind is
                           when Arm_Carried      => (Kind => Arm_Carried, Arm => Arm_Id (Whole (Item (Ms, I), "arm"))),
                           when World_Fixed      => (Kind => World_Fixed),
                           when Carrier_Carried  => (Kind => Carrier_Carried),
                           when Unmeasured       => (Kind => Unmeasured)));
                  end;
               end loop;
               Restored (Stored_Graph) := True;
            end if;
         end;

         --  The kinematics, on the graph and the lock-in.
         declare
            N  : constant Node := Field (Top, "kinematics");
            As : constant Node := Field (N, "arms");
         begin
            if Restored (Stored_Graph) and then Method_Is (N, Kinematics_Method) then
               M.Kinematics.Clear;
               for I in 1 .. Size (As) loop
                  declare
                     X : constant Node := Item (As, I);
                     R : Arm_Evidence := (Arm   => Arm_Id'Base (Natural'Max (0, Whole (X, "arm"))),
                                          Group => Group_Id'Base (Natural'Max (0, Whole (X, "group"))),
                                          Eye   => Eye_Id'Base (Natural'Max (0, Whole (X, "eye"))),
                                          others => <>);
                     Js : constant Node := Field (X, "joints");
                     L  : constant Node := Field (X, "lens");
                  begin
                     --  What its keyframes measured of how far the view moves per unit (Kinematics.Measure_Shifts),
                     --  with its standard deviation, as it was.
                     R.Shifts := Reals_Of (Field (X, "shifts"));
                     R.Shift_Points := Counts_Of (Field (X, "shift_points"));
                     R.Shift_Seen := Flags_Of (Field (X, "shifts_seen"));
                     R.Shift_Told := Counts_Of (Field (X, "shift_told"));
                     --  How far the aligner errs on its eye's pictures, as its keyframes measured it; absent from a
                     --  file written before it was kept: not measured.
                     R.Scatter := (if Is_Number (Field (X, "match_noise")) then Value (Field (X, "match_noise")) else 0.0);
                     declare
                        Sigmas : constant Real_Vectors.Vector := Reals_Of (Field (X, "shift_sigmas"));
                        Known  : constant Flag_Vectors.Vector := Flags_Of (Field (X, "shifts_known"));
                     begin
                        for J in Sigmas.First_Index .. Sigmas.Last_Index loop
                           R.Shift_Sigmas.Append (if J <= Known.Last_Index and then Known (J) then Sigmas (J) else Real'Last);
                        end loop;
                     end;
                     R.Result.Fitted := Truth (X, "fitted");
                     R.Result.Reference := Reals_Of (Field (X, "reference"));
                     R.Result.Chain := Counts_Of (Field (X, "chain"));
                     for J in 1 .. Size (Js) loop
                        R.Result.Joints.Append
                          (Joint_Fit'(W     => Three_Of (Field (Item (Js, J), "w")),
                            P     => Three_Of (Field (Item (Js, J), "p")),
                            C     => Value (Field (Item (Js, J), "c")),
                            Slide => Truth (Item (Js, J), "slide"), others => <>));
                     end loop;
                     R.Result.Lens := (Fx => Value (Field (L, "fx")), Fy => Value (Field (L, "fy")),
                                       Cx => Value (Field (L, "cx")), Cy => Value (Field (L, "cy")),
                                       K1 => Value (Field (L, "k1")), K2 => Value (Field (L, "k2")));
                     R.Result.Used := Natural'Max (0, Whole (X, "used"));
                     R.Result.Median_Px := Value (Field (X, "median_px"));
                     R.Result.Sigma_Px := Value (Field (X, "sigma_px"));
                     R.Result.Matches := Natural'Max (0, Whole (X, "matches"));
                     R.Result.Why := To_Unbounded_String (Driver.Json.Text (Doc, Field (X, "why")));
                     R.Result.Covariance := Reals_Of (Field (X, "covariance"));
                     R.Result.Unit_Changes := Reals_Of (Field (X, "unit_changes"));
                     declare
                        T : constant Node := Field (X, "table");
                        P : constant Node := Field (X, "placement");
                        Tilt : constant Node := Field (T, "tilt");
                     begin
                        R.Result.Table.Centre := Three_Of (Field (T, "centre"));
                        R.Result.Table.Normal := Three_Of (Field (T, "normal"));
                        R.Result.Table.Tangent_1 := Three_Of (Field (T, "tangent"));
                        R.Result.Table.Tangent_2 := Cross (R.Result.Table.Normal, R.Result.Table.Tangent_1);
                        R.Result.Table.Offset_Sigma := Value (Field (T, "offset_sigma"));
                        R.Result.Table.Tilt_11 := Value (Item (Tilt, 1));
                        R.Result.Table.Tilt_12 := Value (Item (Tilt, 2));
                        R.Result.Table.Tilt_22 := Value (Item (Tilt, 3));
                        R.Result.Table.Points := Natural'Max (0, Whole (T, "points"));
                        R.Result.Table.Scatter := Value (Field (T, "scatter"));
                        R.Result.Table_Response := Reals_Of (Field (T, "response"));
                        declare
                           Own : constant Real_Vectors.Vector := Reals_Of (Field (T, "own"));
                        begin
                           if Natural (Own.Length) = 9 then
                              R.Result.Table_Own :=
                                [for I in 1 .. 3 => [for J in 1 .. 3 => Own (Own.First_Index + 3 * (I - 1) + J - 1)]];
                           end if;
                        end;
                        R.Result.Placed := Truth (X, "placed");
                        R.Result.Placement := (Rotation    => Nine_Of (Field (P, "rotation")),
                                               Translation => Three_Of (Field (P, "centre")));
                        R.Result.Scale := Value (Field (P, "scale"));
                        R.Result.Scale_Sigma := Value (Field (P, "scale_sigma"));
                        R.Result.Placement_Covariance := Reals_Of (Field (P, "covariance"));
                        R.Result.Placed_Px := Value (Field (P, "px"));
                        R.Result.Placed_Points := Natural'Max (0, Whole (P, "points"));
                        R.Result.Placed_Through := Natural'Max (0, Whole (P, "through"));
                     end;
                     M.Kinematics.Append (R);
                  end;
               end loop;
               --  The eyes that stand still in the world.
               M.Fixed_Eyes.Clear;
               declare
                  Fs : constant Node := Field (N, "fixed");
               begin
                  for I in 1 .. Size (Fs) loop
                     declare
                        X : constant Node := Item (Fs, I);
                        E : constant Integer := Whole (X, "eye");
                        L : constant Node := Field (X, "lens");
                     begin
                        if E >= 1 then
                           while Integer (M.Fixed_Eyes.Last_Index) < E loop
                              M.Fixed_Eyes.Append (Fixed_Fit'(others => <>));
                           end loop;
                           declare
                              F : Fixed_Fit renames M.Fixed_Eyes (Eye_Id (E));
                           begin
                              F.Known := Truth (X, "known");
                              F.Arm := Arm_Id'Base (Natural'Max (0, Whole (X, "arm")));
                              F.Lens := (Fx => Value (Field (L, "fx")), Fy => Value (Field (L, "fy")),
                                         Cx => Value (Field (L, "cx")), Cy => Value (Field (L, "cy")),
                                         K1 => Value (Field (L, "k1")), K2 => Value (Field (L, "k2")));
                              F.Pose := (Rotation    => Nine_Of (Field (X, "rotation")),
                                         Translation => Three_Of (Field (X, "centre")));
                              F.Covariance := Reals_Of (Field (X, "covariance"));
                              F.Used := Natural'Max (0, Whole (X, "used"));
                              F.Offered := Natural'Max (0, Whole (X, "offered"));
                              F.Sigma_Px := Value (Field (X, "sigma_px"));
                              F.Why := To_Unbounded_String (Driver.Json.Text (Doc, Field (X, "why")));
                              F.Judged := True;
                              F.Arms := Counts_Of (Field (X, "arms"));
                              F.Joint := Reals_Of (Field (X, "joint"));
                              declare
                                 Ps : constant Node := Field (X, "poses");
                              begin
                                 for P in 1 .. Size (Ps) loop
                                    F.Poses.Append
                                      (Driver.Numerics.Rigid'
                                         (Rotation    => Nine_Of (Field (Item (Ps, P), "rotation")),
                                          Translation => Three_Of (Field (Item (Ps, P), "centre"))));
                                 end loop;
                              end;
                           end;
                        end if;
                     end;
                  end loop;
               end;
               --  What is reloaded stands and is not measured again, so the fits
               --  are kept only when every arm that carries an eye is fitted in
               --  them: a boot that failed during the arms' sweeps wrote the arms
               --  it had, and an arm left unfitted here would never be swept.
               declare
                  All_Fitted : Boolean := True;
               begin
                  for E in M.Graph.Mounts.First_Index .. M.Graph.Mounts.Last_Index loop
                     if M.Graph.Mounts (E).Kind = Arm_Carried then
                        declare
                           Found : Boolean := False;
                        begin
                           for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
                              Found := Found or else (M.Kinematics (K).Arm = M.Graph.Mounts (E).Arm
                                                      and then M.Kinematics (K).Result.Fitted);
                           end loop;
                           All_Fitted := All_Fitted and then Found;
                        end;
                     end if;
                  end loop;
                  for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
                     if (for some S of M.Kinematics (K).Shift_Sigmas => S < Real'Last) then
                        Restored (Stored_Shifts) := True;
                     end if;
                  end loop;
                  if All_Fitted then
                     Restored (Stored_Kinematics) := True;
                  else
                     --  The fits go; what the arms' keyframes measured of how far the view moves per unit of
                     --  each joint stays: the recognition's keyframes measured it, the sweep's first levels are
                     --  sized by it, and a boot that reloaded the graph without it began its sweep from turns
                     --  of 2.4 units, which the servos did not execute (R5 on the Pi, 10-14).
                     for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
                        declare
                           R : Arm_Evidence renames M.Kinematics (K);
                        begin
                           R := (Arm => R.Arm, Group => R.Group, Eye => R.Eye, Shifts => R.Shifts,
                                 Shift_Sigmas => R.Shift_Sigmas, Shift_Points => R.Shift_Points,
                                 Shift_Seen => R.Shift_Seen, Shift_Told => R.Shift_Told, others => <>);
                        end;
                     end loop;
                     M.Fixed_Eyes.Clear;
                  end if;
               end;
            end if;
         end;
      end;

      M.From_File := Restored;
      declare
         Kept, Again : Unbounded_String;
         function Name (Q : Stored) return String is
           (case Q is
               when Stored_Noise      => "the readings' noise",
               when Stored_Travel     => "the readings' travel",
               when Stored_Ends       => "the ends the arm showed by stopping",
               when Stored_Steps      => "the step responses",
               when Stored_Lags       => "the image lags",
               when Stored_Responses  => "the eyes' responses",
               when Stored_Graph      => "the graph",
               when Stored_Shifts     => "how far the view moves per unit of each joint",
               when Stored_Kinematics => "the kinematics");
      begin
         for Q in Stored loop
            if Restored (Q) then
               Append (Kept, (if Length (Kept) > 0 then ", " else "") & Name (Q));
            else
               Append (Again, (if Length (Again) > 0 then ", " else "") & Name (Q));
            end if;
         end loop;
         Why := "reloaded " & (if Length (Kept) > 0 then Kept else To_Unbounded_String ("nothing"))
           & "; to measure again: " & (if Length (Again) > 0 then Again else To_Unbounded_String ("nothing"));
      end;
      Ok := True;
   exception
      when Constraint_Error =>
         Why := To_Unbounded_String ("not a body file this driver wrote: a value is out of range");
         M.From_File := [others => False];
         Ok := False;
   end Read;

end Driver.Robot.Body_File;
