--  align_study roma RUN_DIR LAG [-v | REQUEST...]
--  align_study run RUN_DIR LAG FIRST LAST SIGMA_FACTOR ERROR...
--  align_study report [-field] ROWS_FILE...
--  align_study chain RUN_DIR LAG FIRST LAST
--
--  Offline comparison of Driver.Alignment with the recorded answers of the instrument (RoMa), both against
--  the truth: truth_warp's projection of each query point into the second picture.
--
--  RUN_DIR holds what match_extract and truth_warp wrote of a run (the pictures as req_NNNN_a.ppm and
--  req_NNNN_b.ppm, the questions and the instrument's answers, the truth of each question as
--  req_NNNN.truthLAG).
--
--  roma: for the instrument's answers, over the points the truth sees in both pictures on a smooth surface
--  (status V): how many lie within a quarter, a half, one, two and five pixels of the truth, the bias (mean and
--  median error), the spread (the standard deviation of the errors within three pixels, and the robust one by
--  the median absolute deviation), and the same for the answers whose round trip came back within a pixel.
--
--  run: for every question FIRST to LAST and every ERROR (pixels), the aligner is asked for each point the
--  truth places: the prediction is the truth carried through a rigid error of the second eye's pose (a rotation
--  and a shift drawn once per question and scaled so that the points' predicted places are ERROR pixels off, in
--  the root mean square), with the covariance that error implies (SIGMA_FACTOR times it: 1 is a prediction
--  that knows how wrong it is) and the linear part the same pose error makes of the surface's own. One row per
--  point and error on standard output.
--
--  report: the rows of any runs, summarized by error, by how far the points moved, by the precision the answers
--  claim, and by what the truth says of them; with -field the smooth field the aligner and the instrument both
--  differ from the truth by (fitted on the odd points of each question and applied to the even, and the reverse)
--  taken out of both.
--
--  chain: the arm's own keyframes as the boot takes them (beats.txt gives the arm's readings, the questions of one
--  reference in the order they were asked), each predicted from the earlier ones alone before it is aligned, as the
--  driver would have to before the arm is fitted: the earlier keyframes whose change of readings is a multiple of
--  this one's give a displacement field (a quadratic of the place, fitted to their found points), scaled by the
--  multiple (a quadratic in the amount through two), with the scatter of the field and the relative error the
--  last prediction showed as its covariance; a change of readings with no earlier multiple is predicted not to
--  have moved, within as far as a unit of change has moved the points at most. One row per point (the prediction
--  error column is 1 for all) and one "pred" line per keyframe on standard output: how far the points moved, how
--  far the prediction was off, and how often its window held the truth. FIRST to LAST select the references whose
--  first question lies between them.

with Ada.Calendar;
with Ada.Command_Line;
with Ada.Containers.Indefinite_Vectors;
with Ada.Containers.Vectors;
with Ada.Directories;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Alignment;
with Driver.Alignment.Keyframes;
with Driver.Bytes;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Images;
with Driver.Numerics;
with Driver.Numerics.Dense;
with Driver.Stats;
with Driver.Uncertain;

procedure Align_Study is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use type Ada.Streams.Stream_Element_Offset;
   use type Ada.Streams.Stream_Element;
   use type Ada.Containers.Count_Type;
   use type Driver.Real_Array;

   subtype Real is Driver.Real;
   subtype Real_Array is Driver.Real_Array;

   function Img (N : Integer) return String is (Ada.Strings.Fixed.Trim (Integer'Image (N), Ada.Strings.Left));

   function Padded (N : Natural) return String is
      S : constant String := Img (N);
   begin
      return (1 .. 4 - S'Length => '0') & S;
   end Padded;

   function Fixed (X : Real; Aft : Natural := 3) return String is
      Rounded : constant Real := Real'Rounding (abs X * 10.0 ** Aft) / 10.0 ** Aft;
      Whole   : constant Natural := Natural (Real'Floor (Rounded));
      Frac    : constant Natural := Natural (Real'Rounding ((Rounded - Real (Whole)) * 10.0 ** Aft));
      Digits_Text : constant String := Img (Frac);
   begin
      if Aft = 0 then
         return (if X < 0.0 then "-" else "") & Img (Whole);
      end if;
      return (if X < 0.0 and then Rounded > 0.0 then "-" else "") & Img (Whole) & "."
        & (1 .. Aft - Digits_Text'Length => '0') & Digits_Text;
   end Fixed;

   function Percent (Part, Whole : Natural) return String is
     (if Whole = 0 then "   - " else Fixed (100.0 * Real (Part) / Real (Whole), 1));

   ---------------------------------------------------------------------------
   --  Reading

   type Word_Array is array (Positive range <>) of Unbounded_String;

   function Words (Line : String) return Word_Array is
      Result : Word_Array (1 .. Line'Length);
      Count  : Natural := 0;
      I      : Natural := Line'First;
   begin
      while I <= Line'Last loop
         if Line (I) = ' ' then
            I := I + 1;
         else
            declare
               J : Natural := I;
            begin
               while J <= Line'Last and then Line (J) /= ' ' loop
                  J := J + 1;
               end loop;
               Count := Count + 1;
               Result (Count) := To_Unbounded_String (Line (I .. J - 1));
               I := J;
            end;
         end if;
      end loop;
      return Result (1 .. Count);
   end Words;

   function Value (W : Unbounded_String) return Real is (Real'Value (To_String (W)));

   type Lens is record
      Fx, Cx, Cy : Real := 0.0;
   end record;

   type Motion is record
      Present : Boolean := False;
      Rotation : Mat3 := Identity3;
      Shift    : Vec3 := Zero3;
   end record;

   type Point is record
      U, V             : Real := 0.0;
      Roma_U, Roma_V   : Real := -1.0;
      Certainty        : Real := 0.0;
      Back_U, Back_V   : Real := -1.0;
      Status           : Character := 'U';
      Part             : Natural := 0;
      True_U, True_V   : Real := -1.0;
      Surface          : Vec3 := Zero3;     --  in the first camera's frame
      Normal           : Vec3 := [0.0, 0.0, 1.0];
      Jxx, Jxy, Jyx, Jyy : Real := 0.0;
   end record;

   package Point_Vectors is new Ada.Containers.Vectors (Positive, Point);

   type Motion_Array is array (0 .. 63) of Motion;

   type Question is record
      Number           : Natural := 0;
      A_Camera, A_Beat : Natural := 0;
      B_Camera, B_Beat : Natural := 0;
      Points           : Point_Vectors.Vector;
      Paired           : Boolean := False;
      Lens_A, Lens_B   : Lens;
      Motions          : Motion_Array;
   end record;

   function Camera_Of (S : String) return Natural is
      At_Camera : constant Natural := Ada.Strings.Fixed.Index (S, "camera");
      At_Beat   : constant Natural := Ada.Strings.Fixed.Index (S, " beat");
   begin
      return Natural'Value (S (At_Camera + 6 .. At_Beat - 1));
   end Camera_Of;

   function Beat_Of (S : String) return Natural is
      At_Beat : constant Natural := Ada.Strings.Fixed.Index (S, " beat");
   begin
      return Natural'Value (S (At_Beat + 5 .. S'Last));
   end Beat_Of;

   type Question_Access is access Question;

   function Read_Question (Dir : String; Number : Natural; Lag : String) return Question_Access is
      F : Ada.Text_IO.File_Type;
      Q : constant Question_Access := new Question;
   begin
      Q.Number := Number;
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Dir & "/req_" & Padded (Number) & ".txt");
      declare
         Head  : constant String := Ada.Text_IO.Get_Line (F);
         A     : constant String := Ada.Text_IO.Get_Line (F);
         B     : constant String := Ada.Text_IO.Get_Line (F);
         Sizes : constant String := Ada.Text_IO.Get_Line (F);
         pragma Unreferenced (Head, Sizes);
      begin
         Q.A_Camera := Camera_Of (A);
         Q.A_Beat := Beat_Of (A);
         Q.B_Camera := Camera_Of (B);
         Q.B_Beat := Beat_Of (B);
      end;
      while not Ada.Text_IO.End_Of_File (F) loop
         declare
            Line : constant String := Ada.Text_IO.Get_Line (F);
         begin
            if Line'Length > 2 and then Line (Line'First) = 'p' then
               declare
                  Bar_1 : constant Natural := Ada.Strings.Fixed.Index (Line, " | ");
                  Bar_2 : constant Natural :=
                    (if Bar_1 = 0 then 0 else Ada.Strings.Fixed.Index (Line (Bar_1 + 3 .. Line'Last), " | "));
                  P     : Point;
                   Mine  : constant Word_Array :=
                     Words (Line (Line'First + 1 .. (if Bar_1 > 0 then Bar_1 - 1 else Line'Last)));
               begin
                  P.U := Value (Mine (1));
                  P.V := Value (Mine (2));
                  if Bar_1 > 0 then
                     declare
                        Answer : constant Word_Array :=
                          Words (Line (Bar_1 + 3 .. (if Bar_2 > 0 then Bar_2 - 1 else Line'Last)));
                     begin
                        if Answer'Length >= 3 then
                           P.Roma_U := Value (Answer (1));
                           P.Roma_V := Value (Answer (2));
                           P.Certainty := Value (Answer (3));
                        end if;
                     end;
                  end if;
                  if Bar_2 > 0 then
                     declare
                        Back : constant Word_Array := Words (Line (Bar_2 + 3 .. Line'Last));
                     begin
                        if Back'Length >= 2 then
                           P.Back_U := Value (Back (1));
                           P.Back_V := Value (Back (2));
                        end if;
                     end;
                  end if;
                  Q.Points.Append (P);
               end;
            end if;
         end;
      end loop;
      Ada.Text_IO.Close (F);
      --  the truth
      declare
         Path : constant String := Dir & "/req_" & Padded (Number) & ".truth" & Lag;
      begin
         if Ada.Directories.Exists (Path) then
            Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
            while not Ada.Text_IO.End_Of_File (F) loop
               declare
                  Line : constant String := Ada.Text_IO.Get_Line (F);
               begin
                  if Line'Length > 5 and then Line (Line'First .. Line'First + 4) = "point" then
                     declare
                        W : constant Word_Array := Words (Line);
                        K : constant Positive := Positive'Value (To_String (W (2)));
                     begin
                        Q.Paired := True;
                        if K <= Natural (Q.Points.Length) then
                           declare
                              P : Point := Q.Points (K);
                           begin
                              P.Status := To_String (W (3)) (1);
                              P.Part := Natural'Value (To_String (W (4)));
                              P.True_U := Value (W (5));
                              P.True_V := Value (W (6));
                              P.Surface := [Value (W (8)), Value (W (9)), Value (W (10))];
                              P.Normal := [Value (W (12)), Value (W (13)), Value (W (14))];
                              P.Jxx := Value (W (16));
                              P.Jxy := Value (W (17));
                              P.Jyx := Value (W (18));
                              P.Jyy := Value (W (19));
                              Q.Points.Replace_Element (K, P);
                           end;
                        end if;
                     end;
                  elsif Line'Length > 6 and then Line (Line'First .. Line'First + 5) = "lens_a" then
                     declare
                        W : constant Word_Array := Words (Line);
                     begin
                        Q.Lens_A := (Fx => Value (W (2)), Cx => Value (W (3)), Cy => Value (W (4)));
                        Q.Lens_B := (Fx => Value (W (6)), Cx => Value (W (7)), Cy => Value (W (8)));
                     end;
                  elsif Line'Length > 6 and then Line (Line'First .. Line'First + 5) = "motion" then
                     declare
                        W : constant Word_Array := Words (Line);
                        P : constant Natural := Natural'Value (To_String (W (2)));
                     begin
                        if P <= Q.Motions'Last then
                           Q.Motions (P).Present := True;
                           for A in 1 .. 3 loop
                              for B in 1 .. 3 loop
                                 Q.Motions (P).Rotation (A, B) := Value (W (2 + 3 * (A - 1) + B));
                              end loop;
                           end loop;
                           Q.Motions (P).Shift := [Value (W (12)), Value (W (13)), Value (W (14))];
                        end if;
                     end;
                  end if;
               end;
            end loop;
            Ada.Text_IO.Close (F);
         end if;
      end;
      return Q;
   end Read_Question;

   function Read_Ppm (Path : String) return Driver.Images.Image is
      use Ada.Streams.Stream_IO;
      type Bytes_Access is access Driver.Bytes.Byte_Array;
      F    : File_Type;
      Data : Bytes_Access;
      Last : Ada.Streams.Stream_Element_Offset;
   begin
      Open (F, In_File, Path);
      Data := new Driver.Bytes.Byte_Array (1 .. Ada.Streams.Stream_Element_Offset (Size (F)));
      Read (F, Data.all, Last);
      Close (F);
      declare
         Position : Ada.Streams.Stream_Element_Offset := 1;
         Fields   : array (1 .. 3) of Natural;

         function Next_Number return Natural is
            N : Natural := 0;
         begin
            while Data (Position) = 32 or else Data (Position) = 10 loop
               Position := Position + 1;
            end loop;
            while Data (Position) >= 48 and then Data (Position) <= 57 loop
               N := N * 10 + Natural (Data (Position)) - 48;
               Position := Position + 1;
            end loop;
            return N;
         end Next_Number;
      begin
         Position := 3;   --  after "P6"
         for K in Fields'Range loop
            Fields (K) := Next_Number;
         end loop;
         Position := Position + 1;   --  the one white space after the largest value
         return Driver.Images.Create (Fields (1), Fields (2), Data (Position .. Last));
      end;
   end Read_Ppm;

   ---------------------------------------------------------------------------
   --  Statistics of a set of errors

   package Real_Vectors is new Ada.Containers.Vectors (Positive, Real);
   package Index_Vectors is new Ada.Containers.Vectors (Positive, Positive);

   type Sample is record
      Du, Dv : Real_Vectors.Vector;
   end record;

   procedure Add (S : in out Sample; Du, Dv : Real) is
   begin
      S.Du.Append (Du);
      S.Dv.Append (Dv);
   end Add;

   function To_Array (V : Real_Vectors.Vector) return Real_Array is
      R : Real_Array (1 .. Natural (V.Length));
   begin
      for I in R'Range loop
         R (I) := V (I);
      end loop;
      return R;
   end To_Array;

   function Mean_Of (X : Real_Array) return Real is
      Sum : Real := 0.0;
   begin
      for V of X loop
         Sum := Sum + V;
      end loop;
      return Sum / Real (X'Length);
   end Mean_Of;

   function Std_Of (X : Real_Array) return Real is
      M   : constant Real := Mean_Of (X);
      Sum : Real := 0.0;
   begin
      for V of X loop
         Sum := Sum + (V - M) ** 2;
      end loop;
      return Sqrt (Sum / Real (Integer'Max (1, X'Length - 1)));
   end Std_Of;

   --  One line of what a set of errors says: how many within a quarter, a half, one, two and five pixels,
   --  and (of those within three) the bias and the spread.
   procedure Report (Name : String; S : Sample; Of_All : Natural := 0) is
      N : constant Natural := Natural (S.Du.Length);
      Whole : constant Natural := (if Of_All > 0 then Of_All else N);
      Within : array (1 .. 5) of Natural := [others => 0];
      Limits : constant array (1 .. 5) of Real := [0.25, 0.5, 1.0, 2.0, 5.0];
      Inner_Du, Inner_Dv : Real_Vectors.Vector;
   begin
      if N = 0 then
         Ada.Text_IO.Put_Line (Name & ": no answers");
         return;
      end if;
      for I in 1 .. N loop
         declare
            E : constant Real := Sqrt (S.Du (I) ** 2 + S.Dv (I) ** 2);
         begin
            for K in Limits'Range loop
               if E <= Limits (K) then
                  Within (K) := Within (K) + 1;
               end if;
            end loop;
            if E <= 3.0 then
               Inner_Du.Append (S.Du (I));
               Inner_Dv.Append (S.Dv (I));
            end if;
         end;
      end loop;
      declare
         Du : constant Real_Array := To_Array (S.Du);
         Dv : constant Real_Array := To_Array (S.Dv);
         Iu : constant Real_Array := To_Array (Inner_Du);
         Iv : constant Real_Array := To_Array (Inner_Dv);
      begin
         Ada.Text_IO.Put
            (Name & ":" & Natural'Image (N) & " answers of" & Natural'Image (Whole)
             & "; within 0.25/0.5/1/2/5 px of the answers: "
            & Percent (Within (1), N) & " /" & Percent (Within (2), N) & " /" & Percent (Within (3), N) & " /"
             & Percent (Within (4), N) & " /" & Percent (Within (5), N) & " %; of all points within 1 px: "
             & Percent (Within (3), Whole) & " %");
         if Iu'Length > 1 then
            Ada.Text_IO.Put_Line
              ("; bias u " & Fixed (Mean_Of (Iu), 3) & " v " & Fixed (Mean_Of (Iv), 3)
               & " std u " & Fixed (Std_Of (Iu), 3) & " v " & Fixed (Std_Of (Iv), 3)
                & " robust u " & Fixed (Driver.Stats.Robust_Sigma (Du), 3)
                & " v " & Fixed (Driver.Stats.Robust_Sigma (Dv), 3));
         else
            Ada.Text_IO.New_Line;
         end if;
      end;
   end Report;

   ---------------------------------------------------------------------------
   --  The instrument's answers against the truth

   procedure Roma_Mode is
      Dir : constant String := Ada.Command_Line.Argument (2);
      Lag : constant String := Ada.Command_Line.Argument (3);
      Verbose : constant Boolean := Ada.Command_Line.Argument_Count >= 4 and then Ada.Command_Line.Argument (4) = "-v";
      Table_Only : constant Boolean :=
        Ada.Command_Line.Argument_Count >= 4 and then Ada.Command_Line.Argument (4) = "-table";
      All_V, Round_Trip_V, Same_Eye_V, Cross_Eye_V : Sample;
      Counts : array (Character range 'A' .. 'Z') of Natural := [others => 0];
      Missing : Natural := 0;
      Not_In_View_Total, Not_In_View_Rejected : Natural := 0;
      Number : Natural := 1;
   begin
      while Ada.Directories.Exists (Dir & "/req_" & Padded (Number) & ".txt") loop
         if Ada.Command_Line.Argument_Count < 4 or else Verbose or else Table_Only
           or else (for some I in 4 .. Ada.Command_Line.Argument_Count => Ada.Command_Line.Argument (I) = Img (Number))
         then
            declare
               Q : constant Question_Access := Read_Question (Dir, Number, Lag);
            begin
               if not Q.Paired then
                  Missing := Missing + 1;
               end if;
               if Verbose then
                  declare
                     Seen, Near : Natural := 0;
                     Moved      : Real := 0.0;
                  begin
                     for P of Q.Points loop
                        if P.Status = 'V' then
                           Seen := Seen + 1;
                           Moved := Moved + Sqrt ((P.True_U - P.U) ** 2 + (P.True_V - P.V) ** 2);
                            if P.Roma_U >= 0.0
                              and then Sqrt ((P.Roma_U - P.True_U) ** 2 + (P.Roma_V - P.True_V) ** 2) <= 1.0
                            then
                              Near := Near + 1;
                           end if;
                        end if;
                     end loop;
                     Ada.Text_IO.Put_Line
                        ("request " & Img (Number) & ": eye" & Natural'Image (Q.A_Camera)
                         & " beat" & Natural'Image (Q.A_Beat)
                         & " to eye" & Natural'Image (Q.B_Camera) & " beat" & Natural'Image (Q.B_Beat)
                         & ":" & Natural'Image (Seen)
                         & " seen, they moved " & (if Seen > 0 then Fixed (Moved / Real (Seen), 2) else "-")
                         & " px on average, RoMa within 1 px"
                        & Natural'Image (Near));
                  end;
               end if;
               for P of Q.Points loop
                  Counts (P.Status) := Counts (P.Status) + 1;
                   if P.Status = 'V' and then P.Roma_U >= 0.0
                     and then (not Table_Only
                               or else (P.Part = 0
                                        and then Sqrt ((P.True_U - P.U) ** 2 + (P.True_V - P.V) ** 2) > 1.5))
                   then
                     declare
                        Du : constant Real := P.Roma_U - P.True_U;
                        Dv : constant Real := P.Roma_V - P.True_V;
                        Trip : constant Real := Sqrt ((P.Back_U - P.U) ** 2 + (P.Back_V - P.V) ** 2);
                     begin
                        Add (All_V, Du, Dv);
                        if P.Back_U >= 0.0 and then Trip <= 1.0 then
                           Add (Round_Trip_V, Du, Dv);
                        end if;
                        if Q.A_Camera = Q.B_Camera then
                           Add (Same_Eye_V, Du, Dv);
                        else
                           Add (Cross_Eye_V, Du, Dv);
                        end if;
                     end;
                  elsif P.Status = 'O' or else P.Status = 'X' then
                     Not_In_View_Total := Not_In_View_Total + 1;
                     declare
                        Trip : constant Real :=
                          (if P.Back_U >= 0.0 then Sqrt ((P.Back_U - P.U) ** 2 + (P.Back_V - P.V) ** 2) else Real'Last);
                     begin
                        if Trip > 1.0 then
                           Not_In_View_Rejected := Not_In_View_Rejected + 1;
                        end if;
                     end;
                  end if;
               end loop;
            end;
         end if;
         Number := Number + 1;
      end loop;
      Ada.Text_IO.Put_Line ("lag " & Lag & ": questions without a truth: " & Img (Missing));
      Ada.Text_IO.Put ("statuses:");
      for C in Counts'Range loop
         if Counts (C) > 0 then
            Ada.Text_IO.Put (" " & C & "=" & Img (Counts (C)));
         end if;
      end loop;
      Ada.Text_IO.New_Line;
      Report ("RoMa, every point seen in both", All_V);
      Report ("RoMa, same eye", Same_Eye_V);
      Report ("RoMa, two eyes", Cross_Eye_V);
      Report ("RoMa, round trip within 1 px", Round_Trip_V);
      Ada.Text_IO.Put_Line ("not in view (truth O or X):" & Natural'Image (Not_In_View_Total)
                            & ", of which RoMa's round trip rejected"
                            & Natural'Image (Not_In_View_Rejected));
   end Roma_Mode;

   ---------------------------------------------------------------------------
   --  The aligner against the truth, from a prediction with a known error

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      U1 : constant Real := 1.0 - Real (Ada.Numerics.Float_Random.Random (Gen));
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   function Project (L : Lens; X : Vec3) return Real_Array is
     ([L.Fx * X (1) / X (3) + L.Cx, L.Fx * X (2) / X (3) + L.Cy]);

   --  Where the second camera, its pose off by the rigid error (Omega, Shift), sees what the first camera sees at
   --  (U, V) on the surface of a point: the same surface point, carried by its part's motion.
   function Predicted
     (Q : Question; P : Point; Du, Dv : Real; Omega, Shift : Vec3) return Real_Array
   is
      Ray : constant Vec3 := [(P.U + Du - Q.Lens_A.Cx) / Q.Lens_A.Fx, (P.V + Dv - Q.Lens_A.Cy) / Q.Lens_A.Fx, 1.0];
      D   : constant Real := P.Normal * P.Surface;
      Den : constant Real := P.Normal * Ray;
      X   : constant Vec3 := (if abs Den > 1.0E-12 then (D / Den) * Ray else Ray);
      M   : constant Motion := Q.Motions (P.Part);
      Moved : constant Vec3 := M.Rotation * X + M.Shift;
      Off   : constant Vec3 := Exp (Omega) * Moved + Shift;
   begin
      return Project (Q.Lens_B, Off);
   end Predicted;

   function Median_Of (X : Real_Array) return Real is
     (if X'Length = 0 then 0.0 else Driver.Stats.Median (X));

   use type Driver.Alignment.Verdict;

   --  The text of a value that is only there when the answer is.
   function Fixed_If (There : Boolean; X : Real; Aft : Natural) return String is
     (if There then Fixed (X, Aft) else "0");

   --  The derivative of the predicted place by the place in the first picture, by central differences of half a
   --  pixel.
   function Predicted_Linear (Q : Question; P : Point; Rotation, Shift : Vec3) return Driver.Alignment.Linear_Part is
      Pu : constant Real_Array := Predicted (Q, P, 0.5, 0.0, Rotation, Shift);
      Mu : constant Real_Array := Predicted (Q, P, -0.5, 0.0, Rotation, Shift);
      Pv : constant Real_Array := Predicted (Q, P, 0.0, 0.5, Rotation, Shift);
      Mv : constant Real_Array := Predicted (Q, P, 0.0, -0.5, Rotation, Shift);
   begin
      return (UU => Pu (1) - Mu (1), UV => Pv (1) - Mv (1), VU => Pu (2) - Mu (2), VV => Pv (2) - Mv (2));
   end Predicted_Linear;

   --  The rigid error of a question's second eye, drawn once per question: a rotation of about a radian and a shift
   --  of about the median depth of the points, which move the points about alike.
   procedure Draw_Pose_Error (Q : Question; Rotation, Shift : out Vec3) is
      Depths : Real_Vectors.Vector;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, Q.Number * 7919);
      Rotation := [Gaussian, Gaussian, Gaussian];
      Shift := [Gaussian, Gaussian, Gaussian];
      for P of Q.Points loop
         if P.Status = 'V' then
            Depths.Append (P.Surface (3));
         end if;
      end loop;
      Shift := Median_Of (To_Array (Depths)) * Shift;
   end Draw_Pose_Error;

   --  How far the points the truth sees move under a rigid error, in the root mean square.
   function Moved_Rms (Q : Question; Rotation, Shift : Vec3) return Real is
      Sum   : Real := 0.0;
      Count : Natural := 0;
   begin
      for P of Q.Points loop
         if P.Status = 'V' then
            declare
               Moved : constant Real_Array := Predicted (Q, P, 0.0, 0.0, Rotation, Shift);
               Still : constant Real_Array := Predicted (Q, P, 0.0, 0.0, Zero3, Zero3);
            begin
               Sum := Sum + (Moved (1) - Still (1)) ** 2 + (Moved (2) - Still (2)) ** 2;
               Count := Count + 1;
            end;
         end if;
      end loop;
      return (if Count = 0 then 0.0 else Sqrt (Sum / Real (Count)));
   end Moved_Rms;

   --  How far the prediction's linear part is off the truth's, over the points the truth sees: the root mean
   --  square of the entries' differences.
   function Linear_Error (Q : Question; Rotation, Shift : Vec3) return Real is
      Sum   : Real := 0.0;
      Count : Natural := 0;
   begin
      for P of Q.Points loop
         if P.Status = 'V' then
            declare
               L : constant Driver.Alignment.Linear_Part := Predicted_Linear (Q, P, Rotation, Shift);
            begin
               Sum := Sum + (L.UU - P.Jxx) ** 2 + (L.UV - P.Jxy) ** 2 + (L.VU - P.Jyx) ** 2 + (L.VV - P.Jyy) ** 2;
               Count := Count + 4;
            end;
         end if;
      end loop;
      return Sqrt (Sum / Real (Integer'Max (1, Count)));
   end Linear_Error;

   --  The pictures of a question, as pyramids.
   type Pictures is record
      First, Second : Driver.Alignment.Pyramid;
   end record;

   --  What one asked-for point of a question is, as the rows say it. The columns: the request, the point, the
   --  truth's status of it, whether both pictures are of one eye, how far it moved, the prediction's error, the
   --  aligner's verdict and reason, its error against the truth (u, v), its standard deviations (u, v) and
   --  correlation, the instrument's error (u, v), the instrument's round trip (or -1) and whether it answered,
   --  the point's place in the first picture, what the fit left unexplained, the patch's own spread, and the
   --  answer's degrees of freedom.
   procedure Print_Row (Q : Question; K : Positive; A : Driver.Alignment.Answer; Error : Real; Agrees : Integer := -1) is
      P        : constant Point := Q.Points (K);
      Found    : constant Boolean := A.Verdict = Driver.Alignment.Found;
      Has_Roma : constant Boolean := P.Roma_U >= 0.0;
      Trip     : constant Real :=
        (if P.Back_U >= 0.0 then Sqrt ((P.Back_U - P.U) ** 2 + (P.Back_V - P.V) ** 2) else -1.0);
   begin
      Ada.Text_IO.Put_Line
        ("row " & Img (Q.Number) & Natural'Image (K) & " " & P.Status & " "
         & (if Q.A_Camera = Q.B_Camera then "1" else "0")
         & " " & Fixed (Sqrt ((P.True_U - P.U) ** 2 + (P.True_V - P.V) ** 2), 3) & " " & Fixed (Error, 3)
         & " " & (case A.Verdict is
                     when Driver.Alignment.Found => "F",
                     when Driver.Alignment.Not_Found => "N",
                     when Driver.Alignment.Not_In_View => "V")
         & " " & Driver.Alignment.Reason'Image (A.Because)
         & " " & Fixed_If (Found, A.To.U - P.True_U, 4) & " " & Fixed_If (Found, A.To.V - P.True_V, 4)
         & " " & Fixed_If (Found, Sqrt (Real'Max (0.0, A.Cov.UU)), 4)
         & " " & Fixed_If (Found, Sqrt (Real'Max (0.0, A.Cov.VV)), 4)
         & " " & Fixed (A.Correlation, 4)
         & " " & Fixed_If (Has_Roma, P.Roma_U - P.True_U, 4) & " " & Fixed_If (Has_Roma, P.Roma_V - P.True_V, 4)
         & " " & Fixed (Trip, 3) & " " & (if Has_Roma then "1" else "0")
         & " " & Fixed (P.U, 2) & " " & Fixed (P.V, 2)
         & " " & Fixed (A.Fit_Rms, 3) & " " & Fixed (A.Patch_Rms, 3) & " " & Img (A.Degrees_Of_Freedom)
         & (if Agrees >= 0 then " " & Img (Agrees) else ""));
   end Print_Row;

   procedure Ask_Point
     (Q : Question; K : Positive; Seen : Pictures; Error, Variance, Linear_Sigma : Real; Rotation, Shift : Vec3)
   is
      P     : constant Point := Q.Points (K);
      Pred  : constant Real_Array := Predicted (Q, P, 0.0, 0.0, Rotation, Shift);
      Query : constant Driver.Alignment.Prediction :=
        (From => (U => P.U, V => P.V), To => (U => Pred (1), V => Pred (2)),
         Linear => Predicted_Linear (Q, P, Rotation, Shift), Cov => (UU => Variance, UV => 0.0, VV => Variance),
         Linear_Sigma => Linear_Sigma);
   begin
      Print_Row (Q, K, Driver.Alignment.Align (Seen.First, Seen.Second, Query), Error);
   end Ask_Point;

   --  Asks the aligner every point of one question at every error.
   procedure Ask_Question
     (Q : Question; Dir : String; Factor : Real; Errors : Real_Array; Only : Natural; Asked : in out Natural)
   is
      Seen : constant Pictures :=
        (First => Driver.Alignment.Pyramid_Of (Read_Ppm (Dir & "/req_" & Padded (Q.Number) & "_a.ppm")),
         Second => Driver.Alignment.Pyramid_Of (Read_Ppm (Dir & "/req_" & Padded (Q.Number) & "_b.ppm")));
      Rotation, Shift : Vec3;
   begin
      Draw_Pose_Error (Q, Rotation, Shift);
      --  The unit error: a radian of rotation and a median depth of shift move the points about alike.
      declare
         Unit : constant Real := Moved_Rms (Q, Rotation, Shift);
      begin
         if Unit = 0.0 then
            return;
         end if;
         for Error of Errors loop
            declare
               Scale        : constant Real := Error / Unit;
               Rot          : constant Vec3 := Scale * Rotation;
               Sh           : constant Vec3 := Scale * Shift;
               Linear_Sigma : constant Real := Real'Max (1.0E-3, Linear_Error (Q, Rot, Sh));
               Variance     : constant Real := Factor ** 2 * Error ** 2 / 2.0;   --  per axis
            begin
               for K in 1 .. Natural (Q.Points.Length) loop
                  if Q.Points (K).Status /= 'U' and then Q.Points (K).Status /= 'E'
                    and then (Only = 0 or else K = Only)
                  then
                     Asked := Asked + 1;
                     Ask_Point (Q, K, Seen, Error, Variance, Linear_Sigma, Rot, Sh);
                  end if;
               end loop;
            end;
         end loop;
      end;
   end Ask_Question;

   procedure Run_Mode (Probe : Boolean := False) is
      Dir     : constant String := Ada.Command_Line.Argument (2);
      Lag     : constant String := Ada.Command_Line.Argument (3);
      First   : constant Natural := Natural'Value (Ada.Command_Line.Argument (4));
      Last    : constant Natural := (if Probe then First else Natural'Value (Ada.Command_Line.Argument (5)));
      Only    : constant Natural := (if Probe then Natural'Value (Ada.Command_Line.Argument (5)) else 0);
      Factor  : constant Real := Real'Value (Ada.Command_Line.Argument (6));
      Errors  : Real_Array (1 .. Ada.Command_Line.Argument_Count - 6);
      Asked   : Natural := 0;
      use type Ada.Calendar.Time;
      Started : constant Ada.Calendar.Time := Ada.Calendar.Clock;
   begin
      for K in Errors'Range loop
         Errors (K) := Real'Value (Ada.Command_Line.Argument (6 + K));
      end loop;
      for Number in First .. Last loop
         exit when not Ada.Directories.Exists (Dir & "/req_" & Padded (Number) & ".txt");
         declare
            Q : constant Question_Access := Read_Question (Dir, Number, Lag);
         begin
            if Q.Paired and then Ada.Directories.Exists (Dir & "/req_" & Padded (Number) & "_a.ppm") then
               Ask_Question (Q.all, Dir, Factor, Errors, Only, Asked);
            end if;
         end;
      end loop;
      Ada.Text_IO.Put_Line
        (Ada.Text_IO.Standard_Error,
         Img (Asked) & " alignments in"
         & Real'Image (Real (Ada.Calendar.Clock - Started)) & " s of wall time");
   end Run_Mode;

   ---------------------------------------------------------------------------
   --  The rows, summarized

   type Row is record
      Request, Point : Natural := 0;
      Status         : Character := 'U';
      Same_Eye       : Boolean := False;
      Moved, Error   : Real := 0.0;
      Verdict        : Character := 'N';
      Why            : Unbounded_String;
      Eu, Ev, Su, Sv, Corr : Real := 0.0;
      Roma_Eu, Roma_Ev, Trip : Real := 0.0;
      Has_Roma       : Boolean := False;
      U, V           : Real := 0.0;
      Fit_Rms, Patch_Rms : Real := 0.0;
      Dof            : Natural := 0;
   end record;

   package Row_Vectors is new Ada.Containers.Vectors (Positive, Row);
   Rows : Row_Vectors.Vector;

   procedure Read_Rows (Path : String) is
      F : Ada.Text_IO.File_Type;
   begin
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (F) loop
         declare
            Line : constant String := Ada.Text_IO.Get_Line (F);
         begin
            if Line'Length > 4 and then Line (Line'First .. Line'First + 3) = "row " then
               declare
                  W : constant Word_Array := Words (Line);
                  R : Row;
               begin
                  R.Request := Natural'Value (To_String (W (2)));
                  R.Point := Natural'Value (To_String (W (3)));
                  R.Status := To_String (W (4)) (1);
                  R.Same_Eye := To_String (W (5)) = "1";
                  R.Moved := Value (W (6));
                  R.Error := Value (W (7));
                  R.Verdict := To_String (W (8)) (1);
                  R.Why := W (9);
                  R.Eu := Value (W (10));
                  R.Ev := Value (W (11));
                  R.Su := Value (W (12));
                  R.Sv := Value (W (13));
                  R.Corr := Value (W (14));
                  R.Roma_Eu := Value (W (15));
                  R.Roma_Ev := Value (W (16));
                  R.Trip := Value (W (17));
                  R.Has_Roma := To_String (W (18)) = "1";
                  R.U := Value (W (19));
                  R.V := Value (W (20));
                  R.Fit_Rms := Value (W (21));
                  R.Patch_Rms := Value (W (22));
                  R.Dof := Natural'Value (To_String (W (23)));
                  Rows.Append (R);
               end;
            end if;
         end;
      end loop;
      Ada.Text_IO.Close (F);
   end Read_Rows;

   --  How far the points moved, in classes: a pixel or less, up to four, sixteen, sixty-four, and beyond.
   function Class_Of (Moved : Real) return Positive is
     (if Moved <= 1.0 then 1 elsif Moved <= 4.0 then 2 elsif Moved <= 16.0 then 3 elsif Moved <= 64.0 then 4 else 5);

   function Class_Name (Class : Positive) return String is
      (case Class is
          when 1 => "<= 1 px", when 2 => "1 - 4 px", when 3 => "4 - 16 px", when 4 => "16 - 64 px",
          when others => "> 64 px");

   --  How many standard deviations a difference must be to be significant for a covariance that rests on that many
   --  other fits: the driver's Z at the Gaussian, the Student t at the same tail probability otherwise.
   function Gate_Of (Dof : Natural) return Real is
     (if Dof = 0 then Driver.Conventions.Z
       else Driver.Distributions.Student_T_Quantile
         (Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z), Dof));

   type Eyes is (Same, Cross);

   procedure Summarize
     (Name : String; Error : Real; Which : Eyes; Class_Wanted : Boolean; Class : Natural)
   is
      Seen, Found, Refused, Out_Of_View, Within_1, Within_Z, Axes, False_Answers, False_Beyond_Z : Natural := 0;
      Roma_Within_1, Roma_Rt_Answers, Roma_Rt_Within : Natural := 0;
      Both_Within, Both_Total : Natural := 0;
      Found_Within_1_Roma_Bad, Roma_Ok_Found_Bad : Natural := 0;
      Eu, Ev : Real_Vectors.Vector;
      Normalized : Real := 0.0;
      Terms : Natural := 0;
   begin
      for R of Rows loop
         if R.Status = 'V' and then abs (R.Error - Error) < 1.0E-6 and then (R.Same_Eye = (Which = Same))
           and then (not Class_Wanted or else Class_Of (R.Moved) = Class)
         then
            Seen := Seen + 1;
            declare
               Dist : constant Real := Sqrt (R.Eu ** 2 + R.Ev ** 2);
               Roma_Dist : constant Real := Sqrt (R.Roma_Eu ** 2 + R.Roma_Ev ** 2);
               Roma_Good : constant Boolean := R.Has_Roma and then Roma_Dist <= 1.0;
            begin
               if R.Has_Roma then
                  if Roma_Dist <= 1.0 then
                     Roma_Within_1 := Roma_Within_1 + 1;
                  end if;
                  if R.Trip >= 0.0 and then R.Trip <= 1.0 then
                     Roma_Rt_Answers := Roma_Rt_Answers + 1;
                     if Roma_Dist <= 1.0 then
                        Roma_Rt_Within := Roma_Rt_Within + 1;
                     end if;
                  end if;
               end if;
               case R.Verdict is
                  when 'F' =>
                     Found := Found + 1;
                     Eu.Append (R.Eu);
                     Ev.Append (R.Ev);
                     if Dist <= 1.0 then
                        Within_1 := Within_1 + 1;
                     else
                        False_Answers := False_Answers + 1;
                     end if;
                     if R.Su > 0.0 and then R.Sv > 0.0 then
                        Normalized := Normalized + (R.Eu / R.Su) ** 2 + (R.Ev / R.Sv) ** 2;
                        Terms := Terms + 2;
                        Axes := Axes + 2;
                        if abs R.Eu <= Gate_Of (R.Dof) * R.Su then
                           Within_Z := Within_Z + 1;
                        end if;
                        if abs R.Ev <= Gate_Of (R.Dof) * R.Sv then
                           Within_Z := Within_Z + 1;
                        end if;
                        if abs R.Eu > Gate_Of (R.Dof) * R.Su or else abs R.Ev > Gate_Of (R.Dof) * R.Sv then
                           False_Beyond_Z := False_Beyond_Z + 1;
                        end if;
                     end if;
                     Both_Total := Both_Total + 1;
                     if Roma_Good then
                        Both_Within := Both_Within + 1;
                     end if;
                     if Dist > 1.0 and then Roma_Good then
                        Roma_Ok_Found_Bad := Roma_Ok_Found_Bad + 1;
                     end if;
                     if Dist <= 1.0 and then R.Has_Roma and then not Roma_Good then
                        Found_Within_1_Roma_Bad := Found_Within_1_Roma_Bad + 1;
                     end if;
                  when 'V' =>
                     Out_Of_View := Out_Of_View + 1;
                  when others =>
                     Refused := Refused + 1;
               end case;
            end;
         end if;
      end loop;
      if Seen = 0 then
         return;
      end if;
      Ada.Text_IO.Put
        (Name & " error" & Fixed (Error, 1) & ": points" & Natural'Image (Seen) & "; aligner found "
         & Percent (Found, Seen) & " % (refused "
         & Percent (Refused, Seen) & " %, called out of view " & Percent (Out_Of_View, Seen)
         & " %), within 1 px of all "
         & Percent (Within_1, Seen) & " % / of found " & Percent (Within_1, Found) & " %, found beyond 1 px "
         & Percent (False_Answers, Found)
         & " % of found");
      if Terms > 0 then
         Ada.Text_IO.Put
           (", errors within Z sigma on" & Percent (Within_Z, Axes) & " % of axes, beyond on "
            & Percent (False_Beyond_Z, Found) & " % of found, mean (e/s)^2 "
            & Fixed (Normalized / Real (Terms), 2));
      end if;
      if Eu.Length > 1 then
         declare
            Du : constant Real_Array := To_Array (Eu);
            Dv : constant Real_Array := To_Array (Ev);
            Inner_U, Inner_V : Real_Vectors.Vector;
         begin
            for I in Du'Range loop
               if Sqrt (Du (I) ** 2 + Dv (I) ** 2) <= 3.0 then
                  Inner_U.Append (Du (I));
                  Inner_V.Append (Dv (I));
               end if;
            end loop;
            if Inner_U.Length > 1 then
               Ada.Text_IO.Put
                 ("; within 3 px: bias" & Real'Image (Real'Rounding (Mean_Of (To_Array (Inner_U)) * 1000.0) / 1000.0)
                  & Real'Image (Real'Rounding (Mean_Of (To_Array (Inner_V)) * 1000.0) / 1000.0)
                  & " std " & Fixed (Std_Of (To_Array (Inner_U)), 3) & " " & Fixed (Std_Of (To_Array (Inner_V)), 3)
                  & " robust " & Fixed (Driver.Stats.Robust_Sigma (Du), 3) & " "
                  & Fixed (Driver.Stats.Robust_Sigma (Dv), 3));
            end if;
         end;
      end if;
      Ada.Text_IO.New_Line;
      Ada.Text_IO.Put_Line
        ("      RoMa on the same points: within 1 px of all " & Percent (Roma_Within_1, Seen)
         & " %; round trip within 1 px on "
         & Percent (Roma_Rt_Answers, Seen) & " %, of which within 1 px " & Percent (Roma_Rt_Within, Roma_Rt_Answers)
         & " %; of the aligner's answers RoMa is within 1 px on "
         & Percent (Both_Within, Both_Total) & " %");
      if Roma_Ok_Found_Bad + Found_Within_1_Roma_Bad > 0 then
         Ada.Text_IO.Put_Line
           ("      aligner found beyond 1 px where RoMa is within it:" & Natural'Image (Roma_Ok_Found_Bad)
            & "; aligner within 1 px where RoMa is not:" & Natural'Image (Found_Within_1_Roma_Bad));
      end if;
   end Summarize;

   ---------------------------------------------------------------------------
   --  The error the two matchers share: a smooth field of each question

   --  What the aligner and the instrument both do, against the truth's projection, that the truth does not hold:
   --  a field over the picture that grows with the radius and the size of the motion (the render's own geometry
   --  differing from the pinhole of the truth). It is fitted to the instrument's errors, a quadratic of the
   --  place per axis, on the points of one parity of their number in the question, three robust passes, and
   --  taken out of the errors of the other parity (of both matchers), then the parities are swapped, so no
   --  point is ever corrected by a field that saw its own answer of the instrument.
   type Field is record
      U, V  : Real_Array (1 .. 6) := [others => 0.0];
      Valid : Boolean := False;
   end record;

   Fields : array (0 .. 999, 0 .. 1) of Field;

   function Basis (U, V : Real) return Real_Array is
      X : constant Real := (U - 320.0) / 320.0;
      Y : constant Real := (V - 240.0) / 240.0;
   begin
      return [1.0, X, Y, X * X, X * Y, Y * Y];
   end Basis;

   function Evaluate (C : Real_Array; U, V : Real) return Real is
      B   : constant Real_Array := Basis (U, V);
      Sum : Real := 0.0;
   begin
      for K in B'Range loop
         Sum := Sum + C (K) * B (K);
      end loop;
      return Sum;
   end Evaluate;

   procedure Fit_Field (Request, Parity : Natural; First_Error : Real) is
      Chosen : Index_Vectors.Vector;   --  the rows used, as their numbers in Rows
      Keep   : array (1 .. Natural (Rows.Length)) of Boolean := [others => False];
      Used   : Natural := 0;
      F      : Field;
   begin
      for I in 1 .. Natural (Rows.Length) loop
         declare
            R : Row renames Rows (I);
         begin
            if R.Request = Request and then R.Status = 'V' and then R.Same_Eye and then R.Has_Roma
              and then R.Trip >= 0.0 and then R.Trip <= 1.0 and then abs (R.Error - First_Error) < 1.0E-6
              and then R.Point mod 2 = Parity and then Sqrt (R.Roma_Eu ** 2 + R.Roma_Ev ** 2) < 3.0
            then
               Chosen.Append (I);
               Keep (I) := True;
               Used := Used + 1;
            end if;
         end;
      end loop;
      if Used < 40 then
         return;
      end if;
      for Pass in 1 .. 3 loop
         declare
            Count : Natural := 0;
         begin
            for Item of Chosen loop
               if Keep (Item) then
                  Count := Count + 1;
               end if;
            end loop;
            exit when Count < 40;
            declare
               A  : Real_Matrix (1 .. Count, 1 .. 6);
               Bu, Bv : Real_Vector (1 .. Count);
               Xu, Xv : Real_Vector (1 .. 6);
               Full_U, Full_V : Boolean;
               K  : Natural := 0;
            begin
               for Item of Chosen loop
                  if Keep (Item) then
                     K := K + 1;
                     declare
                        R : Row renames Rows (Item);
                        Row_Basis : constant Real_Array := Basis (R.U, R.V);
                     begin
                        for J in 1 .. 6 loop
                           A (K, J) := Row_Basis (J);
                        end loop;
                        Bu (K) := R.Roma_Eu;
                        Bv (K) := R.Roma_Ev;
                     end;
                  end if;
               end loop;
               Driver.Numerics.Dense.Least_Squares (A, Bu, Xu, Full_U);
               Driver.Numerics.Dense.Least_Squares (A, Bv, Xv, Full_V);
               exit when not (Full_U and then Full_V);
               for J in 1 .. 6 loop
                  F.U (J) := Xu (J);
                  F.V (J) := Xv (J);
               end loop;
               F.Valid := True;
            end;
            --  Who stays for the next pass: the answers within Z robust sigmas of the field.
            declare
               Ru, Rv : Real_Vectors.Vector;
            begin
               for Item of Chosen loop
                  declare
                     R : Row renames Rows (Item);
                  begin
                     Ru.Append (R.Roma_Eu - Evaluate (F.U, R.U, R.V));
                     Rv.Append (R.Roma_Ev - Evaluate (F.V, R.U, R.V));
                  end;
               end loop;
               declare
                  Su : constant Real := Driver.Stats.Robust_Sigma (To_Array (Ru));
                  Sv : constant Real := Driver.Stats.Robust_Sigma (To_Array (Rv));
                  Index : Natural := 0;
               begin
                  for Item of Chosen loop
                     Index := Index + 1;
                     Keep (Item) :=
                       abs Ru (Index) <= Driver.Conventions.Z * Su and then abs Rv (Index) <= Driver.Conventions.Z * Sv;
                  end loop;
               end;
            end;
         end;
      end loop;
      Fields (Request, Parity) := F;
   end Fit_Field;

   procedure Take_Out_Fields (First_Error : Real) is
      Largest : Natural := 0;
   begin
      for R of Rows loop
         Largest := Natural'Max (Largest, R.Request);
      end loop;
      for Request in 0 .. Natural'Min (Largest, 999) loop
         Fit_Field (Request, 0, First_Error);
         Fit_Field (Request, 1, First_Error);
      end loop;
      for I in 1 .. Natural (Rows.Length) loop
         declare
            R : Row := Rows (I);
            F : constant Field := Fields (Natural'Min (R.Request, 999), 1 - R.Point mod 2);
         begin
            if F.Valid then
               R.Eu := R.Eu - Evaluate (F.U, R.U, R.V);
               R.Ev := R.Ev - Evaluate (F.V, R.U, R.V);
               R.Roma_Eu := R.Roma_Eu - Evaluate (F.U, R.U, R.V);
               R.Roma_Ev := R.Roma_Ev - Evaluate (F.V, R.U, R.V);
               Rows.Replace_Element (I, R);
            end if;
         end;
      end loop;
   end Take_Out_Fields;

   --  What a consumer that weighs the answers by their stated precision gets: of the answers that claim to know
   --  the place to within S pixels (the larger of the two axes' standard deviations), how many there are and how
   --  many of them lie more than a pixel, or more than their own gate, from the truth.
   procedure Operating_Curve (Error : Real) is
      Limits : constant array (1 .. 7) of Real := [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, Real'Last];
      Seen   : Natural := 0;
   begin
      for R of Rows loop
         if R.Status = 'V' and then R.Same_Eye and then abs (R.Error - Error) < 1.0E-6 then
            Seen := Seen + 1;
         end if;
      end loop;
      if Seen = 0 then
         return;
      end if;
      Ada.Text_IO.Put_Line ("error" & Fixed (Error, 1) & ": points" & Natural'Image (Seen));
      for Limit of Limits loop
         declare
            Answers, Beyond_1, Beyond_2, Beyond_Gate : Natural := 0;
         begin
            for R of Rows loop
               if R.Status = 'V' and then R.Same_Eye and then R.Verdict = 'F' and then abs (R.Error - Error) < 1.0E-6
                 and then R.Su > 0.0 and then R.Sv > 0.0 and then Real'Max (R.Su, R.Sv) <= Limit
               then
                  Answers := Answers + 1;
                  if Sqrt (R.Eu ** 2 + R.Ev ** 2) > 1.0 then
                     Beyond_1 := Beyond_1 + 1;
                  end if;
                  if Sqrt (R.Eu ** 2 + R.Ev ** 2) > 2.0 then
                     Beyond_2 := Beyond_2 + 1;
                  end if;
                  if abs R.Eu > Gate_Of (R.Dof) * R.Su or else abs R.Ev > Gate_Of (R.Dof) * R.Sv then
                     Beyond_Gate := Beyond_Gate + 1;
                  end if;
               end if;
            end loop;
            Ada.Text_IO.Put_Line
              ("  sigma <= " & (if Limit = Real'Last then "any" else Fixed (Limit, 1) & " px")
               & ": answers " & Percent (Answers, Seen) & " % of the points; of them beyond 1 px "
               & Percent (Beyond_1, Answers) & " %, beyond 2 px " & Percent (Beyond_2, Answers)
               & " %, beyond their own gate " & Percent (Beyond_Gate, Answers) & " %");
         end;
      end loop;
   end Operating_Curve;

   --  Why the points the truth sees in both pictures were not found, over all the errors asked.
   procedure Refusals (Same_Eye : Boolean) is
      type Count_Array is array (Driver.Alignment.Reason) of Natural;
      Counts : Count_Array := [others => 0];
   begin
      Ada.Text_IO.Put_Line
        ("--- why points were refused (truth status V, " & (if Same_Eye then "same eye)" else "two eyes)"));
      for R of Rows loop
         if R.Status = 'V' and then R.Same_Eye = Same_Eye and then R.Verdict /= 'F' then
            declare
               Why : constant Driver.Alignment.Reason := Driver.Alignment.Reason'Value (To_String (R.Why));
            begin
               Counts (Why) := Counts (Why) + 1;
            end;
         end if;
      end loop;
      for Why in Count_Array'Range loop
         if Counts (Why) > 0 then
            Ada.Text_IO.Put_Line ("  " & Driver.Alignment.Reason'Image (Why) & Natural'Image (Counts (Why)));
         end if;
      end loop;
   end Refusals;

   ---------------------------------------------------------------------------
   --  The keyframes of one reference, each predicted from the earlier ones and then aligned

   package Reading_Vectors is new Ada.Containers.Indefinite_Vectors (Natural, Real_Array);
   Left_Arm, Right_Arm : Reading_Vectors.Vector;   --  by beat

   --  The joint readings of both arms at every beat of the run (beats.txt).
   procedure Read_Readings (Path : String) is
      F : Ada.Text_IO.File_Type;
   begin
      Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
      while not Ada.Text_IO.End_Of_File (F) loop
         declare
            W    : constant Word_Array := Words (Ada.Text_IO.Get_Line (F));
            Beat : constant Natural := (if W'Length > 1 then Natural'Value (To_String (W (2))) else 0);
            Mode : Natural := 0;   --  0 none, 1 left arm, 2 right arm
            Left, Right : Real_Vectors.Vector;
         begin
            for K in 3 .. W'Last loop
               declare
                  T : constant String := To_String (W (K));
               begin
                  if T = "|" then
                     Mode := 0;
                  elsif T = "state/left_arm_joint_state" then
                     Mode := 1;
                  elsif T = "state/right_arm_joint_state" then
                     Mode := 2;
                  elsif Mode = 1 then
                     Left.Append (Real'Value (T));
                  elsif Mode = 2 then
                     Right.Append (Real'Value (T));
                  end if;
               end;
            end loop;
            while Natural (Left_Arm.Length) <= Beat loop
               Left_Arm.Append (Real_Array'(1 .. 0 => 0.0));
               Right_Arm.Append (Real_Array'(1 .. 0 => 0.0));
            end loop;
            Left_Arm.Replace_Element (Beat, To_Array (Left));
            Right_Arm.Replace_Element (Beat, To_Array (Right));
         end;
      end loop;
      Ada.Text_IO.Close (F);
   end Read_Readings;

   type Keyframe_Question is record
      Number             : Natural := 0;
      Camera             : Natural := 0;
      From_Beat, To_Beat : Natural := 0;
      Q                  : Question_Access;
   end record;

   package Keyframe_Vectors is new Ada.Containers.Vectors (Positive, Keyframe_Question);

   function Readings_At (Camera, Beat : Natural) return Real_Array is
     (if Camera = 2 then Left_Arm (Beat) else Right_Arm (Beat));

   function Change_Of (K : Keyframe_Question) return Real_Array is
      A : constant Real_Array := Readings_At (K.Camera, K.From_Beat);
      B : constant Real_Array := Readings_At (K.Camera, K.To_Beat);
      D : Real_Array (A'Range);
   begin
      for C in D'Range loop
         D (C) := B (C) - A (C);
      end loop;
      return D;
   end Change_Of;

   function Dot (A, B : Real_Array) return Real is
      Sum : Real := 0.0;
   begin
      for K in A'Range loop
         Sum := Sum + A (K) * B (K);
      end loop;
      return Sum;
   end Dot;

   Quantization : constant Real := 1.0 / 12.0;

   --  The noise of one reading of each channel: the robust scale of the readings' beat-to-beat differences over the
   --  whole stream (the arm is still at most beats, so that scale is its readings' jitter), over the root of two.
   function Reading_Noises (Camera : Natural) return Real_Array is
      Channels : constant Natural := Readings_At (Camera, 0)'Length;
      Result   : Real_Array (1 .. Channels) := [others => 0.0];
   begin
      for C in Result'Range loop
         declare
            Steps : Real_Vectors.Vector;
         begin
            for Beat in 1 .. Natural (Left_Arm.Length) - 1 loop
               Steps.Append (Readings_At (Camera, Beat) (C) - Readings_At (Camera, Beat - 1) (C));
            end loop;
            Result (C) := Driver.Stats.Robust_Sigma (To_Array (Steps)) / Sqrt (2.0);
         end;
      end loop;
      return Result;
   end Reading_Noises;

   --  The pixels a unit of each channel moved the view: the median, over the keyframes that turned that channel
   --  most, of how far their points moved (the truth's, over the points on a smooth surface) over the change, those
   --  that moved by more than a pixel grid rounds to. What the lock-in measures before the arm is swept.
   function Shifts_Of (Group : Keyframe_Vectors.Vector) return Real_Array is
      Channels    : constant Natural := Change_Of (Group.First_Element)'Length;
      Per_Channel : array (1 .. Channels) of Real_Vectors.Vector;
      Result      : Real_Array (1 .. Channels) := [others => 0.0];
   begin
      for Now of Group loop
         declare
            Change : constant Real_Array := Change_Of (Now);
            Big    : Positive := Change'First;
            Moves  : Real_Vectors.Vector;
         begin
            for K in Change'Range loop
               if abs Change (K) > abs Change (Big) then
                  Big := K;
               end if;
            end loop;
            for P of Now.Q.Points loop
               if P.Status = 'V' then
                  Moves.Append (Sqrt ((P.True_U - P.U) ** 2 + (P.True_V - P.V) ** 2));
               end if;
            end loop;
            --  A keyframe whose points moved by less than a pixel grid's own rounding says nothing of the shift.
            if not Moves.Is_Empty and then Driver.Stats.Median (To_Array (Moves)) >= Sqrt (Quantization) then
               Per_Channel (Big).Append (Driver.Stats.Median (To_Array (Moves)) / abs Change (Big));
            end if;
         end;
      end loop;
      for K in Result'Range loop
         if not Per_Channel (K).Is_Empty then
            Result (K) := Driver.Stats.Median (To_Array (Per_Channel (K)));
         end if;
      end loop;
      return Result;
   end Shifts_Of;

   procedure Chain_Group (Dir : String; Group : Keyframe_Vectors.Vector) is
      Window : constant Real := Driver.Uncertain.Threshold (Driver.Uncertain.Vector_Gate (2));
      First  : Question renames Group.First_Element.Q.all;
      Noises : constant Real_Array := Reading_Noises (Group.First_Element.Camera);
      Shifts : constant Real_Array := Shifts_Of (Group);
      Points : Driver.Alignment.Keyframes.Point_Array (1 .. Natural (First.Points.Length));
      Chain  : Driver.Alignment.Keyframes.Chain;
   begin
      for K in Noises'Range loop
         Ada.Text_IO.Put_Line
           (Ada.Text_IO.Standard_Error, "camera" & Natural'Image (Group.First_Element.Camera) & " channel" & Natural'Image (K)
            & ": reading noise " & Fixed (Noises (K), 9) & ", shift " & Fixed (Shifts (K), 3) & " px per unit");
      end loop;
      for I in Points'Range loop
         Points (I) := (U => First.Points (I).U, V => First.Points (I).V);
      end loop;
      Driver.Alignment.Keyframes.Begin_Reference
        (Chain, Read_Ppm (Dir & "/req_" & Padded (Group.First_Element.Number) & "_a.ppm"), Points);
      for Now of Group loop
         declare
            Q       : Question renames Now.Q.all;
            Change  : constant Real_Array := Change_Of (Now);
            Own     : constant Real := Sqrt (Dot (Change, Change));
            Given   : Driver.Alignment.Keyframes.Prediction_Array (Points'Range);
            Answers : Driver.Alignment.Keyframes.Answer_Array (Points'Range);
            Agree   : Driver.Alignment.Keyframes.Flag_Array (Points'Range);
            From    : Driver.Alignment.Keyframes.Source;
            Points_Here, Reached, Covered, Found : Natural := 0;
            Sum_Identity, Sum_Predicted : Real := 0.0;
            Sigmas, Errors : Real_Vectors.Vector;
         begin
            Driver.Alignment.Keyframes.Predict
              (Chain, Change, Noises, Shifts, Driver.Alignment.Keyframes.No_Offer, Given, From);
            Driver.Alignment.Keyframes.Add
              (Chain, Read_Ppm (Dir & "/req_" & Padded (Q.Number) & "_b.ppm"), Change, Given, Answers, Agree);
            for I in Points'Range loop
               declare
                  P : Point renames Q.Points (I);
               begin
                  Print_Row (Q, I, Answers (I), 1.0, (if Agree (I) then 1 else 0));
                  if Answers (I).Verdict = Driver.Alignment.Found then
                     Found := Found + 1;
                  end if;
                  if P.Status = 'V' then
                     declare
                        Variance : constant Real := Given (I).Cov.UU;
                        Off      : constant Real :=
                          Sqrt ((P.True_U - Given (I).To.U) ** 2 + (P.True_V - Given (I).To.V) ** 2);
                     begin
                        Points_Here := Points_Here + 1;
                        Sum_Identity := Sum_Identity + (P.True_U - P.U) ** 2 + (P.True_V - P.V) ** 2;
                        Sum_Predicted := Sum_Predicted + Off ** 2;
                        Sigmas.Append (Sqrt (Variance));
                        Errors.Append (Off);
                        if Off <= Window * Sqrt (Variance + Quantization) then
                           Covered := Covered + 1;
                        end if;
                        if Off <= 1.0 then
                           Reached := Reached + 1;
                        end if;
                     end;
                  end if;
               end;
            end loop;
            if Points_Here > 0 then
               Ada.Text_IO.Put_Line
                 ("pred request" & Natural'Image (Q.Number) & " camera" & Natural'Image (Now.Camera)
                  & " readings change " & Fixed (Own, 5) & ": "
                  & Driver.Alignment.Keyframes.Source'Image (From)
                  & "; points" & Natural'Image (Points_Here) & ", identity error "
                  & Fixed (Sqrt (Sum_Identity / Real (Points_Here)), 3) & " px, prediction error rms "
                  & Fixed (Sqrt (Sum_Predicted / Real (Points_Here)), 3) & " median "
                  & Fixed (Driver.Stats.Median (To_Array (Errors)), 3) & " px, within 1 px "
                  & Percent (Reached, Points_Here) & " %, median stated sigma "
                  & Fixed (Driver.Stats.Median (To_Array (Sigmas)), 3) & " px, truth inside the window "
                  & Percent (Covered, Points_Here) & " %, found" & Natural'Image (Found));
            end if;
         end;
      end loop;
   end Chain_Group;

   procedure Chain_Mode is
      Dir : constant String := Ada.Command_Line.Argument (2);
      Lag : constant String := Ada.Command_Line.Argument (3);
      First : constant Natural := Natural'Value (Ada.Command_Line.Argument (4));
      Last  : constant Natural := Natural'Value (Ada.Command_Line.Argument (5));
      All_Questions : Keyframe_Vectors.Vector;
      Started : constant Ada.Calendar.Time := Ada.Calendar.Clock;
      use type Ada.Calendar.Time;
   begin
      Read_Readings (Dir & "/beats.txt");
      declare
         Number : Natural := 1;
      begin
         while Ada.Directories.Exists (Dir & "/req_" & Padded (Number) & ".txt") loop
            declare
               Q : constant Question_Access := Read_Question (Dir, Number, Lag);
            begin
               if Q.Paired and then Q.A_Camera = Q.B_Camera and then Q.A_Camera in 2 .. 3 then
                  All_Questions.Append
                    (Keyframe_Question'(Number => Number, Camera => Q.A_Camera, From_Beat => Q.A_Beat,
                                        To_Beat => Q.B_Beat, Q => Q));
               end if;
            end;
            Number := Number + 1;
         end loop;
      end;
      --  The keyframes of one reference, in the order the arm took them.
      for Start in 1 .. Natural (All_Questions.Length) loop
         declare
            Ref   : constant Keyframe_Question := All_Questions (Start);
            Group : Keyframe_Vectors.Vector;
         begin
            if (for all J in 1 .. Start - 1 =>
                  All_Questions (J).From_Beat /= Ref.From_Beat or else All_Questions (J).Camera /= Ref.Camera)
              and then Ref.Number >= First and then Ref.Number <= Last
            then
               for Other of All_Questions loop
                  if Other.Camera = Ref.Camera and then Other.From_Beat = Ref.From_Beat then
                     Group.Append (Other);
                  end if;
               end loop;
               Chain_Group (Dir, Group);
            end if;
         end;
      end loop;
      Ada.Text_IO.Put_Line
        (Ada.Text_IO.Standard_Error,
         "chained in" & Real'Image (Real (Ada.Calendar.Clock - Started)) & " s of wall time");
   end Chain_Mode;

   procedure Report_Mode is
      Levels : Real_Vectors.Vector;
      Calibrate : constant Boolean :=
        Ada.Command_Line.Argument_Count >= 2 and then Ada.Command_Line.Argument (2) = "-field";
   begin
      for K in (if Calibrate then 3 else 2) .. Ada.Command_Line.Argument_Count loop
         Read_Rows (Ada.Command_Line.Argument (K));
      end loop;
      if Calibrate and then not Rows.Is_Empty then
         Take_Out_Fields (Rows.First_Element.Error);
         Ada.Text_IO.Put_Line
           ("the shared field taken out, fitted on the odd points and applied to the even ones and the reverse");
      end if;
      Ada.Text_IO.Put_Line (Img (Natural (Rows.Length)) & " rows");
      for R of Rows loop
         declare
            Known : Boolean := False;
         begin
            for L of Levels loop
               if abs (L - R.Error) < 1.0E-6 then
                  Known := True;
               end if;
            end loop;
            if not Known then
               Levels.Append (R.Error);
            end if;
         end;
      end loop;
      Ada.Text_IO.Put_Line
        ("--- by prediction error, same eye (the truth sees the point in both pictures on a smooth surface)");
      for E of Levels loop
         Summarize ("all", E, Same, False, 0);
      end loop;
      Ada.Text_IO.Put_Line ("--- the answers by the precision they claim, same eye");
      for E of Levels loop
         Operating_Curve (E);
      end loop;
      Ada.Text_IO.Put_Line ("--- by how far the points moved, same eye");
      for E of Levels loop
         for C in 1 .. 5 loop
            Summarize (Class_Name (C), E, Same, True, C);
         end loop;
      end loop;
      Ada.Text_IO.Put_Line ("--- two eyes");
      for E of Levels loop
         Summarize ("two eyes", E, Cross, False, 0);
      end loop;
      Ada.Text_IO.Put_Line
        ("--- points the truth does not see in the second picture (status O behind something, X outside it)");
      for E of Levels loop
         declare
            Total, Found_Count, Out_Count, Refused_Count : Natural := 0;
            Roma_Rejected : Natural := 0;
         begin
            for R of Rows loop
               if (R.Status = 'O' or else R.Status = 'X') and then abs (R.Error - E) < 1.0E-6 then
                  Total := Total + 1;
                  case R.Verdict is
                     when 'F' => Found_Count := Found_Count + 1;
                     when 'V' => Out_Count := Out_Count + 1;
                     when others => Refused_Count := Refused_Count + 1;
                  end case;
                  if not R.Has_Roma or else R.Trip < 0.0 or else R.Trip > 1.0 then
                     Roma_Rejected := Roma_Rejected + 1;
                  end if;
               end if;
            end loop;
            if Total > 0 then
               Ada.Text_IO.Put_Line
                  ("error" & Fixed (E, 1) & ": points" & Natural'Image (Total) & "; aligner said out of view "
                   & Percent (Out_Count, Total)
                  & " %, not found " & Percent (Refused_Count, Total) & " %, FOUND " & Percent (Found_Count, Total)
                  & " %; RoMa's round trip rejected " & Percent (Roma_Rejected, Total) & " %");
            end if;
         end;
      end loop;
      Refusals (Same_Eye => True);
      Refusals (Same_Eye => False);
   end Report_Mode;

   ---------------------------------------------------------------------------
   --  distinct: how distinctive a point's patch is in the whole of the other picture, when its warp is known.

   type Real_Access is access Real_Array;

   --  The luma of a picture at a level (halved that many times by averaging two by two).
   procedure Level_Luma
     (I : Driver.Images.Image; Level : Natural; Pixels : out Real_Access; Width, Height : out Natural)
   is
      W    : Natural := Driver.Images.Width (I);
      H    : Natural := Driver.Images.Height (I);
      Cur  : Real_Access := new Real_Array (1 .. W * H);
   begin
      Driver.Images.Luma (I, Cur.all);
      for Step in 1 .. Level loop
         declare
            NW : constant Natural := W / 2;
            NH : constant Natural := H / 2;
            Next : constant Real_Access := new Real_Array (1 .. NW * NH);
         begin
            for Y in 0 .. NH - 1 loop
               for X in 0 .. NW - 1 loop
                  Next (1 + Y * NW + X) :=
                    (Cur (1 + (2 * Y) * W + 2 * X) + Cur (1 + (2 * Y) * W + 2 * X + 1)
                     + Cur (1 + (2 * Y + 1) * W + 2 * X) + Cur (1 + (2 * Y + 1) * W + 2 * X + 1)) / 4.0;
               end loop;
            end loop;
            Cur := Next;
            W := NW;
            H := NH;
         end;
      end loop;
      Pixels := Cur;
      Width := W;
      Height := H;
   end Level_Luma;

   function Bilinear (P : Real_Array; W, H : Natural; X, Y : Real) return Real is
      --  X and Y in pixel units where the centre of the first pixel is (0.5, 0.5).
      Fx : constant Real := X - 0.5;
      Fy : constant Real := Y - 0.5;
      X0 : constant Integer := Integer (Real'Floor (Fx));
      Y0 : constant Integer := Integer (Real'Floor (Fy));
      Ax : constant Real := Fx - Real (X0);
      Ay : constant Real := Fy - Real (Y0);
      function At_Pixel (Cx, Cy : Integer) return Real is
        (P (1 + Integer'Max (0, Integer'Min (H - 1, Cy)) * W + Integer'Max (0, Integer'Min (W - 1, Cx))));
   begin
      return (1.0 - Ay) * ((1.0 - Ax) * At_Pixel (X0, Y0) + Ax * At_Pixel (X0 + 1, Y0))
        + Ay * ((1.0 - Ax) * At_Pixel (X0, Y0 + 1) + Ax * At_Pixel (X0 + 1, Y0 + 1));
   end Bilinear;

   procedure Distinct_Mode is
      Dir    : constant String := Ada.Command_Line.Argument (2);
      Lag    : constant String := Ada.Command_Line.Argument (3);
      Number : constant Natural := Natural'Value (Ada.Command_Line.Argument (4));
      Q      : constant Question_Access := Read_Question (Dir, Number, Lag);
      Image_A : constant Driver.Images.Image := Read_Ppm (Dir & "/req_" & Padded (Number) & "_a.ppm");
      Image_B : constant Driver.Images.Image := Read_Ppm (Dir & "/req_" & Padded (Number) & "_b.ppm");
      Half   : constant Natural := 8;   --  a patch of seventeen pixels a side, as a level of the aligner's carries
      Tried, Top1, Top5, Seen : Natural := 0;
      Ratios : Real_Vectors.Vector;
   begin
      for I in 1 .. Natural (Q.Points.Length) loop
         declare
            P : Point renames Q.Points (I);
            Det : constant Real := P.Jxx * P.Jyy - P.Jxy * P.Jyx;
         begin
            if P.Status = 'V' and then Det > 1.0E-6 then
               Seen := Seen + 1;
               declare
                  Scale : constant Real := Sqrt (Det);
                  --  The level of the first picture whose pixels are the second's, or the second's that are the first's.
                  A_Level : constant Natural :=
                    (if Scale < 1.0 then Natural (Real'Max (0.0, Real'Rounding (Log (1.0 / Scale, 2.0)))) else 0);
                  B_Level : constant Natural :=
                    (if Scale > 1.0 then Natural (Real'Max (0.0, Real'Rounding (Log (Scale, 2.0)))) else 0);
                  A_Px, B_Px : Real_Access;
                  Aw, Ah, Bw, Bh : Natural;
                  Ratio : constant Real := Real (2 ** A_Level) / Real (2 ** B_Level);
                  --  The second picture's pixel offset (at its level) to the first's offset (at its level):
                  --  d_A = Ratio * Inverse (J) * d_B / ...
                  Ia : constant Real := P.Jyy / Det;
                  Ib : constant Real := -P.Jxy / Det;
                  Ic : constant Real := -P.Jyx / Det;
                  Id : constant Real := P.Jxx / Det;
                  Side : constant Positive := 2 * Half + 1;
                  Template : Real_Array (1 .. Side * Side);
                  Mean_T, Var_T : Real := 0.0;
                  Best, Truth_Rho : Real := -2.0;
                  Best_X, Best_Y : Integer := 0;
                  Truth_X : constant Integer := Integer (Real'Floor (P.True_U / Real (2 ** B_Level)));
                  Truth_Y : constant Integer := Integer (Real'Floor (P.True_V / Real (2 ** B_Level)));
                  type Peak is record
                     X, Y : Integer;
                     Rho  : Real;
                  end record;
                  Peaks : array (1 .. 5) of Peak := [others => (0, 0, -2.0)];
               begin
                  Level_Luma (Image_A, A_Level, A_Px, Aw, Ah);
                  Level_Luma (Image_B, B_Level, B_Px, Bw, Bh);
                  --  The first picture's patch resampled through the known warp, at the second's level.
                  for Dy in -Half .. Half loop
                     for Dx in -Half .. Half loop
                        declare
                           Bx : constant Real := Real (Dx);
                           By : constant Real := Real (Dy);
                           --  offset in the first picture's full-size pixels: Inverse (J) * (offset in the second's full-size pixels)
                           Fu : constant Real := Real (2 ** B_Level) * (Ia * Bx + Ib * By);
                           Fv : constant Real := Real (2 ** B_Level) * (Ic * Bx + Id * By);
                           Sx : constant Real := (P.U + Fu) / Real (2 ** A_Level);
                           Sy : constant Real := (P.V + Fv) / Real (2 ** A_Level);
                           T  : constant Real := Bilinear (A_Px.all, Aw, Ah, Sx, Sy);
                        begin
                           Template ((Dy + Half) * Side + Dx + Half + 1) := T;
                           Mean_T := Mean_T + T;
                        end;
                     end loop;
                  end loop;
                  Mean_T := Mean_T / Real (Side * Side);
                  for K in Template'Range loop
                     Template (K) := Template (K) - Mean_T;
                     Var_T := Var_T + Template (K) ** 2;
                  end loop;
                  if Var_T > 1.0E-9 then
                     Tried := Tried + 1;
                     for Cy in Half .. Bh - 1 - Half loop
                        for Cx in Half .. Bw - 1 - Half loop
                           declare
                              Sum_S, Sum_SS, Sum_ST : Real := 0.0;
                              K : Natural := 0;
                           begin
                              for Dy in -Half .. Half loop
                                 for Dx in -Half .. Half loop
                                    K := K + 1;
                                    declare
                                       S : constant Real := B_Px (1 + (Cy + Dy) * Bw + Cx + Dx);
                                    begin
                                       Sum_S := Sum_S + S;
                                       Sum_SS := Sum_SS + S * S;
                                       Sum_ST := Sum_ST + S * Template (K);
                                    end;
                                 end loop;
                              end loop;
                              declare
                                 N   : constant Real := Real (Side * Side);
                                 Vss : constant Real := Sum_SS - Sum_S ** 2 / N;
                                 Rho : constant Real := (if Vss > 1.0E-9 then Sum_ST / Sqrt (Vss * Var_T) else -2.0);
                              begin
                                 if abs (Cx - Truth_X) <= 1 and then abs (Cy - Truth_Y) <= 1 then
                                    Truth_Rho := Real'Max (Truth_Rho, Rho);
                                 end if;
                                 if Rho > Best then
                                    Best := Rho;
                                    Best_X := Cx;
                                    Best_Y := Cy;
                                 end if;
                                 --  the five best places at least four pixels apart
                                 declare
                                    Slot : Natural := 0;
                                 begin
                                    for S_K in Peaks'Range loop
                                       if abs (Peaks (S_K).X - Cx) <= 3 and then abs (Peaks (S_K).Y - Cy) <= 3 then
                                          Slot := S_K;
                                       end if;
                                    end loop;
                                    if Slot > 0 then
                                       if Rho > Peaks (Slot).Rho then
                                          Peaks (Slot) := (Cx, Cy, Rho);
                                       end if;
                                    else
                                       declare
                                          Lowest : Positive := 1;
                                       begin
                                          for S_K in Peaks'Range loop
                                             if Peaks (S_K).Rho < Peaks (Lowest).Rho then
                                                Lowest := S_K;
                                             end if;
                                          end loop;
                                          if Rho > Peaks (Lowest).Rho then
                                             Peaks (Lowest) := (Cx, Cy, Rho);
                                          end if;
                                       end;
                                    end if;
                                 end;
                              end;
                           end;
                        end loop;
                     end loop;
                     if abs (Best_X - Truth_X) <= 1 and then abs (Best_Y - Truth_Y) <= 1 then
                        Top1 := Top1 + 1;
                     end if;
                     if (for some Item of Peaks => abs (Item.X - Truth_X) <= 1 and then abs (Item.Y - Truth_Y) <= 1) then
                        Top5 := Top5 + 1;
                     end if;
                     Ratios.Append (Truth_Rho - Best);
                     Ada.Text_IO.Put_Line
                       ("point" & Natural'Image (I) & " scale " & Fixed (Scale, 3) & " levels" & Natural'Image (A_Level)
                        & Natural'Image (B_Level) & " truth rho " & Fixed (Truth_Rho, 3) & " best " & Fixed (Best, 3)
                        & (if abs (Best_X - Truth_X) <= 1 and then abs (Best_Y - Truth_Y) <= 1 then " TOP1" else ""));
                  end if;
                  pragma Unreferenced (Ratio);
               end;
            end if;
         end;
      end loop;
      Ada.Text_IO.Put_Line
        ("request" & Natural'Image (Number) & ": points the truth sees" & Natural'Image (Seen) & ", textured"
         & Natural'Image (Tried) & ", the whole picture's best place is the truth's" & Natural'Image (Top1) & " ("
         & Percent (Top1, Natural'Max (1, Tried)) & " %), among its five best" & Natural'Image (Top5) & " ("
         & Percent (Top5, Natural'Max (1, Tried)) & " %)");
   end Distinct_Mode;

begin
   if Ada.Command_Line.Argument_Count < 2 then
      Ada.Text_IO.Put_Line ("usage: align_study roma|run|report ...");
      return;
   end if;
   if Ada.Command_Line.Argument (1) = "roma" then
      Roma_Mode;
   elsif Ada.Command_Line.Argument (1) = "run" then
      Run_Mode;
   elsif Ada.Command_Line.Argument (1) = "probe" then
      Run_Mode (Probe => True);
   elsif Ada.Command_Line.Argument (1) = "chain" then
      Chain_Mode;
   elsif Ada.Command_Line.Argument (1) = "distinct" then
      Distinct_Mode;
   else
      Report_Mode;
   end if;
end Align_Study;
