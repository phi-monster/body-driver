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
   procedure Print_Row (Q : Question; K : Positive; A : Driver.Alignment.Answer; Error : Real) is
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
         & " " & Fixed (A.Fit_Rms, 3) & " " & Fixed (A.Patch_Rms, 3) & " " & Img (A.Degrees_Of_Freedom));
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

   --  Where a reference point went in a keyframe, as the aligner gave it.
   type Kept_Match is record
      Found : Boolean := False;
      U, V  : Real := 0.0;
      Sigma : Real := 0.0;   --  the larger of the answer's standard deviations
   end record;

   package Kept_Vectors is new Ada.Containers.Vectors (Positive, Kept_Match);

   --  The displacements of a keyframe's points as a smooth field: a quadratic of the place, per axis.
   type Smooth is record
      U, V  : Real_Array (1 .. 6) := [others => 0.0];
      Valid : Boolean := False;
      Sigma : Real := 0.0;   --  the robust scale of what the field leaves
      Typical : Real := 0.0;   --  the median displacement of the points it was fitted to
   end record;

   function Gradient (F : Smooth; U, V : Real) return Driver.Alignment.Linear_Part is
      X : constant Real := (U - 320.0) / 320.0;
      Y : constant Real := (V - 240.0) / 240.0;
      Du_U : constant Real := (F.U (2) + 2.0 * F.U (4) * X + F.U (5) * Y) / 320.0;
      Du_V : constant Real := (F.U (3) + F.U (5) * X + 2.0 * F.U (6) * Y) / 240.0;
      Dv_U : constant Real := (F.V (2) + 2.0 * F.V (4) * X + F.V (5) * Y) / 320.0;
      Dv_V : constant Real := (F.V (3) + F.V (5) * X + 2.0 * F.V (6) * Y) / 240.0;
   begin
      return (UU => 1.0 + Du_U, UV => Du_V, VU => Dv_U, VV => 1.0 + Dv_V);
   end Gradient;

   function Fit_Smooth (Q : Question; Matches : Kept_Vectors.Vector) return Smooth is
      Result : Smooth;
      Chosen : Index_Vectors.Vector;
      Keep   : array (1 .. Natural (Matches.Length)) of Boolean := [others => False];
   begin
      for I in 1 .. Natural (Matches.Length) loop
         if Matches (I).Found then
            Chosen.Append (I);
            Keep (I) := True;
         end if;
      end loop;
      if Natural (Chosen.Length) < 40 then
         return Result;
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
                        P : Point renames Q.Points (Item);
                        Row_Basis : constant Real_Array := Basis (P.U, P.V);
                     begin
                        for J in 1 .. 6 loop
                           A (K, J) := Row_Basis (J);
                        end loop;
                        Bu (K) := Matches (Item).U - P.U;
                        Bv (K) := Matches (Item).V - P.V;
                     end;
                  end if;
               end loop;
               Driver.Numerics.Dense.Least_Squares (A, Bu, Xu, Full_U);
               Driver.Numerics.Dense.Least_Squares (A, Bv, Xv, Full_V);
               exit when not (Full_U and then Full_V);
               for J in 1 .. 6 loop
                  Result.U (J) := Xu (J);
                  Result.V (J) := Xv (J);
               end loop;
               Result.Valid := True;
            end;
            declare
               Ru, Rv : Real_Vectors.Vector;
            begin
               for Item of Chosen loop
                  declare
                     P : Point renames Q.Points (Item);
                  begin
                     Ru.Append (Matches (Item).U - P.U - Evaluate (Result.U, P.U, P.V));
                     Rv.Append (Matches (Item).V - P.V - Evaluate (Result.V, P.U, P.V));
                  end;
               end loop;
               declare
                  Su : constant Real := Driver.Stats.Robust_Sigma (To_Array (Ru));
                  Sv : constant Real := Driver.Stats.Robust_Sigma (To_Array (Rv));
                  Index : Natural := 0;
               begin
                  Result.Sigma := Sqrt ((Su ** 2 + Sv ** 2) / 2.0);
                  for Item of Chosen loop
                     Index := Index + 1;
                     Keep (Item) := abs Ru (Index) <= Driver.Conventions.Z * Su
                       and then abs Rv (Index) <= Driver.Conventions.Z * Sv;
                  end loop;
               end;
            end;
         end;
      end loop;
      return Result;
   end Fit_Smooth;

   --  A keyframe already aligned: the readings' change from the reference, where the points went, the smooth
   --  field of that, and how far the prediction that found them was off, relatively.
   type Done_Keyframe is record
      Change    : Real_Vectors.Vector;
      Amount    : Real := 0.0;   --  the length of the change
      Matches   : Kept_Vectors.Vector;
      Field     : Smooth;
      Rel_Error : Real := 0.0;   --  the relative error its own prediction showed, beyond its stated sigma
   end record;

   package Done_Vectors is new Ada.Containers.Vectors (Positive, Done_Keyframe);

   Quantization : constant Real := 1.0 / 12.0;

   Collinear : constant Real := 0.05;
   --  A keyframe is a multiple of an earlier one when its change of readings lies within this fraction of its own
   --  length of a multiple of that one's (a study value, not the driver's).

   --  How far along the direction of Change the earlier keyframe lies (signed), and whether it lies along it at all.
   procedure Along (Before : Done_Keyframe; Change : Real_Array; Amount : out Real; Is_Along : out Boolean) is
      Own  : constant Real := Dot (Change, Change);
      Past : constant Real_Array := To_Array (Before.Change);
      Left : Real_Array (Change'Range);
      Unit : constant Real := Dot (Past, Change) / Own;
   begin
      Amount := Dot (Past, Change) / Sqrt (Own);
      for C in Left'Range loop
         Left (C) := Past (C) - Unit * Change (C);
      end loop;
      Is_Along :=
        Before.Field.Valid and then abs Amount > 1.0E-6 and then Dot (Left, Left) <= Collinear ** 2 * Dot (Past, Past);
   end Along;

   type Predicted_Point is record
      Du, Dv, Sigma : Real := 0.0;
      Warp          : Driver.Alignment.Linear_Part := Driver.Alignment.Identity_Part;
   end record;

   package Predicted_Vectors is new Ada.Containers.Vectors (Positive, Predicted_Point);

   --  What the fields of one or two earlier keyframes, at the amounts A1 and A2 along the direction, say of a
   --  keyframe at the amount Target: a field that is linear in the amount (one earlier keyframe) or quadratic in
   --  it (two), its uncertainty the earlier fields' scatter carried by the same weights, and the relative error
   --  the earlier predictions showed.
   procedure Extrapolate
     (Q : Question; First, Second : Done_Keyframe; A1, A2, Target : Real; Two : Boolean; Relative : Real;
      Into : out Predicted_Vectors.Vector)
   is
      W1 : constant Real := (if Two then Target * (A2 - Target) / (A1 * (A2 - A1)) else Target / A1);
      W2 : constant Real := (if Two then Target * (Target - A1) / (A2 * (A2 - A1)) else 0.0);
      Noise : constant Real := Sqrt ((W1 * First.Field.Sigma) ** 2 + (W2 * Second.Field.Sigma) ** 2);
   begin
      Into.Clear;
      for P of Q.Points loop
         declare
            Item : Predicted_Point;
         begin
            if P.Status /= 'U' and then P.Status /= 'E' then
               declare
                  G1 : constant Driver.Alignment.Linear_Part := Gradient (First.Field, P.U, P.V);
                  G2 : constant Driver.Alignment.Linear_Part := Gradient (Second.Field, P.U, P.V);
               begin
                  Item.Du := W1 * Evaluate (First.Field.U, P.U, P.V) + W2 * Evaluate (Second.Field.U, P.U, P.V);
                  Item.Dv := W1 * Evaluate (First.Field.V, P.U, P.V) + W2 * Evaluate (Second.Field.V, P.U, P.V);
                  Item.Warp :=
                    (UU => 1.0 + W1 * (G1.UU - 1.0) + W2 * (G2.UU - 1.0), UV => W1 * G1.UV + W2 * G2.UV,
                     VU => W1 * G1.VU + W2 * G2.VU, VV => 1.0 + W1 * (G1.VV - 1.0) + W2 * (G2.VV - 1.0));
                  Item.Sigma := Sqrt (Noise ** 2 + (Relative * Sqrt (Item.Du ** 2 + Item.Dv ** 2)) ** 2);
               end;
            end if;
            Into.Append (Item);
         end;
      end loop;
   end Extrapolate;

   function Median_Sigma (V : Predicted_Vectors.Vector) return Real is
      S : Real_Vectors.Vector;
   begin
      for Item of V loop
         S.Append (Item.Sigma);
      end loop;
      return (if S.Is_Empty then 0.0 else Driver.Stats.Median (To_Array (S)));
   end Median_Sigma;

   procedure Chain_Group (Dir : String; Group : Keyframe_Vectors.Vector) is
      Done : Done_Vectors.Vector;
      Window : constant Real := Driver.Uncertain.Threshold (Driver.Uncertain.Vector_Gate (2));
   begin
      for K in 1 .. Natural (Group.Length) loop
         declare
            Now    : Keyframe_Question renames Group (K);
            Q      : Question renames Now.Q.all;
            Seen   : constant Pictures :=
              (First => Driver.Alignment.Pyramid_Of (Read_Ppm (Dir & "/req_" & Padded (Q.Number) & "_a.ppm")),
               Second => Driver.Alignment.Pyramid_Of (Read_Ppm (Dir & "/req_" & Padded (Q.Number) & "_b.ppm")));
            Change : constant Real_Array := Change_Of (Now);
            Own    : constant Real := Sqrt (Dot (Change, Change));
            Guess  : Predicted_Vectors.Vector;
            How    : Unbounded_String := To_Unbounded_String ("a new direction");
            Used_Relative : Real := 0.0;
            Predicted_Here : Boolean := False;
            Kept   : Kept_Vectors.Vector;
            Sensitivity : Real := 0.0;   --  how far a unit of change of readings moved the points, at most so far
            Points, Reached, Covered, Found : Natural := 0;
            Sum_Identity, Sum_Predicted : Real := 0.0;
            Sum_Excess, Sum_Shown : Real := 0.0;
            Sigmas, Errors : Real_Vectors.Vector;
         begin
            if Own >= 1.0E-6 then
               --  The earlier keyframes along this direction and where they lie on it; the best way to predict from
               --  them is the one with the least uncertainty.
               declare
                  Count   : constant Natural := Natural (Done.Length);
                  Amounts : Real_Array (1 .. Count) := [others => 0.0];
                  Usable  : array (1 .. Count) of Boolean := [others => False];
                  Best    : Real := Real'Last;

                  procedure Consider (J1, J2 : Natural; Two : Boolean) is
                     Try : Predicted_Vectors.Vector;
                     Relative : constant Real :=
                       Real'Max (Done (J1).Rel_Error, (if Two then Done (J2).Rel_Error else 0.0));
                  begin
                     Extrapolate (Q, Done (J1), Done (J2), Amounts (J1), Amounts (J2), Own, Two, Relative, Try);
                     if Median_Sigma (Try) < Best then
                        Best := Median_Sigma (Try);
                        Guess := Try;
                        Used_Relative := Relative;
                        Predicted_Here := True;
                        How := To_Unbounded_String
                          ((if Two then "order 2 from requests" & Natural'Image (Group (J1).Number) & " and"
                                          & Natural'Image (Group (J2).Number)
                            else "order 1 from request" & Natural'Image (Group (J1).Number)));
                     end if;
                  end Consider;
               begin
                  for J in 1 .. Count loop
                     Along (Done (J), Change, Amounts (J), Usable (J));
                     if Done (J).Field.Valid and then Done (J).Amount > 1.0E-6 then
                        Sensitivity := Real'Max (Sensitivity, Done (J).Field.Typical / Done (J).Amount);
                     end if;
                  end loop;
                  for J1 in 1 .. Count loop
                     if Usable (J1) then
                        Consider (J1, J1, False);
                        for J2 in 1 .. Count loop
                           if J2 /= J1 and then Usable (J2) and then abs (Amounts (J1) - Amounts (J2)) > 1.0E-6 then
                              Consider (J1, J2, True);
                           end if;
                        end loop;
                     end if;
                  end loop;
               end;
            else
               How := To_Unbounded_String ("same pose");
            end if;
            for I in 1 .. Natural (Q.Points.Length) loop
               declare
                  P : Point renames Q.Points (I);
                  Match : Kept_Match;
               begin
                  if P.Status /= 'U' and then P.Status /= 'E' then
                     declare
                        Item : Predicted_Point;
                     begin
                        if Predicted_Here then
                           Item := Guess (I);
                        elsif Own >= 1.0E-6 then
                           --  A change of readings no earlier keyframe holds a multiple of: the points may have
                           --  moved as far as a unit of change moved them at most, so far.
                           Item.Sigma := (if Sensitivity > 0.0 then Own * Sensitivity else 1.0);
                        end if;
                        declare
                           Variance : constant Real := Item.Sigma ** 2;
                           Query : constant Driver.Alignment.Prediction :=
                             (From => (U => P.U, V => P.V), To => (U => P.U + Item.Du, V => P.V + Item.Dv),
                              Linear => Item.Warp, Cov => (UU => Variance, UV => 0.0, VV => Variance), others => <>);
                           A : constant Driver.Alignment.Answer :=
                             Driver.Alignment.Align (Seen.First, Seen.Second, Query);
                        begin
                           Print_Row (Q, I, A, 1.0);
                           if A.Verdict = Driver.Alignment.Found then
                              Match := (Found => True, U => A.To.U, V => A.To.V,
                                        Sigma => Sqrt (Real'Max (A.Cov.UU, A.Cov.VV)));
                              Found := Found + 1;
                              Sum_Excess := Sum_Excess
                                + ((A.To.U - P.U - Item.Du) ** 2 + (A.To.V - P.V - Item.Dv) ** 2) / 2.0
                                - Variance - Match.Sigma ** 2;
                              Sum_Shown := Sum_Shown + (Item.Du ** 2 + Item.Dv ** 2) / 2.0;
                           end if;
                           if P.Status = 'V' then
                              declare
                                 Off : constant Real :=
                                   Sqrt ((P.True_U - P.U - Item.Du) ** 2 + (P.True_V - P.V - Item.Dv) ** 2);
                              begin
                                 Points := Points + 1;
                                 Sum_Identity := Sum_Identity + (P.True_U - P.U) ** 2 + (P.True_V - P.V) ** 2;
                                 Sum_Predicted := Sum_Predicted + Off ** 2;
                                 Sigmas.Append (Item.Sigma);
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
                     end;
                  end if;
                  Kept.Append (Match);
               end;
            end loop;
            if Points > 0 then
               Ada.Text_IO.Put_Line
                 ("pred request" & Natural'Image (Q.Number) & " camera" & Natural'Image (Now.Camera)
                  & " readings change " & Fixed (Own, 5) & ": " & To_String (How)
                  & "; points" & Natural'Image (Points) & ", identity error "
                  & Fixed (Sqrt (Sum_Identity / Real (Points)), 3) & " px, prediction error rms "
                  & Fixed (Sqrt (Sum_Predicted / Real (Points)), 3) & " median "
                  & Fixed (Driver.Stats.Median (To_Array (Errors)), 3) & " px, within 1 px "
                  & Percent (Reached, Points) & " %, median stated sigma "
                  & Fixed (Driver.Stats.Median (To_Array (Sigmas)), 3) & " px, truth inside the window "
                  & Percent (Covered, Points) & " %, found" & Natural'Image (Found));
            end if;
            declare
               Next   : Done_Keyframe;
               Shifts : Real_Vectors.Vector;
            begin
               for X of Change loop
                  Next.Change.Append (X);
               end loop;
               Next.Amount := Own;
               Next.Matches := Kept;
               for I in 1 .. Natural (Kept.Length) loop
                  if Kept (I).Found then
                     Shifts.Append (Sqrt ((Kept (I).U - Q.Points (I).U) ** 2 + (Kept (I).V - Q.Points (I).V) ** 2));
                  end if;
               end loop;
               Next.Field := Fit_Smooth (Q, Kept);
               Next.Field.Typical := (if Shifts.Is_Empty then 0.0 else Driver.Stats.Median (To_Array (Shifts)));
               Next.Rel_Error :=
                 (if Predicted_Here and then Sum_Shown > 0.0
                  then Sqrt (Used_Relative ** 2 + Real'Max (0.0, Sum_Excess) / Sum_Shown) else 0.0);
               Done.Append (Next);
            end;
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
   else
      Report_Mode;
   end if;
end Align_Study;
