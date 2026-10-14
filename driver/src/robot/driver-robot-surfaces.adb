with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Robot.Kinematics;

package body Driver.Robot.Surfaces is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Driver.Alignment.Verdict;
   use type Driver.Observations.Camera_Id;
   use type Driver.Observations.Group_Id;

   --  A difference of two places, judged against the sum of their covariances: significant when its chi square on
   --  two degrees of freedom is beyond Z (the driver's one gate, as the lock-in judges its cells).
   function Apart (Du, Dv : Real; C : Driver.Alignment.Place_Covariance) return Boolean is
      Det : constant Real := C.UU * C.VV - C.UV * C.UV;
   begin
      if Det <= 0.0 then
         return True;
      end if;
      declare
         Q : constant Real := (C.VV * Du * Du - 2.0 * C.UV * Du * Dv + C.UU * Dv * Dv) / Det;
      begin
         return Driver.Distributions.Chi_Square_Deviate (Q, 2) > Driver.Conventions.Z;
      end;
   end Apart;

   function Plus (A, B : Driver.Alignment.Place_Covariance) return Driver.Alignment.Place_Covariance is
     ((UU => A.UU + B.UU, UV => A.UV + B.UV, VV => A.VV + B.VV));

   --  The inverse of a warp's linear part: the derivative of the first picture's place by the second's.
   function Inverse (L : Driver.Alignment.Linear_Part; Ok : out Boolean) return Driver.Alignment.Linear_Part is
      Det : constant Real := L.UU * L.VV - L.UV * L.VU;
   begin
      Ok := Det /= 0.0;
      if not Ok then
         return Driver.Alignment.Identity_Part;
      end if;
      return (UU => L.VV / Det, UV => -L.UV / Det, VU => -L.VU / Det, VV => L.UU / Det);
   end Inverse;

   procedure Place
     (Now      : View;
      Elsewhere : View_Array;
      Queries  : Pixel_Array;
      Columns  : Positive;
      Table    : Driver.Geometry.Plane_Estimate;
      Ray      : not null Lens_Ray;
      Project  : not null Lens_Project;
      Noise    : Real;
      Result   : out Placed_Array)
   is
      Z      : constant Real := Driver.Conventions.Z;
      Centre : constant Vec3 := Now.Pose.Translation;

      --  Where a point of the frame lands in a view's picture.
      procedure Into (V : View; P : Vec3; U, W : out Real; In_Front : out Boolean) is
      begin
         Project (Inverse (V.Pose) * P, U, W, In_Front);
      end Into;

      --  The line of sight through a pixel of a view, in the frame.
      function Sight (V : View; U, W : Real) return Vec3 is (Times (V.Pose.Rotation, Ray (U, W)));

      --  The angle a pixel spans along a line of sight: the eye's resolution there.
      function Pixel_Angle (U, W : Real) return Real is
        (Arccos (Real'Min (1.0, Dot (Ray (U, W), Ray (U + 1.0, W)))));

      --  How far along a line of sight from the eye at C it meets the table; 0 when it does not, in front.
      function To_Table (C, D : Vec3) return Real is
         Facing : constant Real := Dot (Table.Normal, D);
      begin
         if Facing = 0.0 then
            return 0.0;
         end if;
         declare
            S : constant Real := Dot (Table.Normal, Table.Centre - C) / Facing;
         begin
            return (if S > 0.0 then S else 0.0);
         end;
      end To_Table;

      Width  : constant Natural := Driver.Alignment.Level_Width (Now.Picture, 0);
      Height : constant Natural := Driver.Alignment.Level_Height (Now.Picture, 0);
      function Inside (U, W : Real) return Boolean is
        (U >= 0.0 and then W >= 0.0 and then U <= Real (Width) and then W <= Real (Height));

      --  The views used: the farthest from where the eye stands now among those whose picture holds the queries'
      --  middle at the table, and the one standing farthest off the line between those two.
      First, Second : Natural := 0;
   begin
      for R of Result loop
         R := (others => <>);
      end loop;
      if Elsewhere'Length = 0 or else not Driver.Geometry.Known (Table) or else Noise <= 0.0 then
         return;
      end if;
      declare
         Sum   : Vec3 := Zero3;
         Count : Natural := 0;
         Middle_U, Middle_V : Real;
         Best  : Real := 0.0;
         Off   : Real := 0.0;
      begin
         for Q of Queries loop
            if Q.U >= 0.0 then
               Sum := Sum + [Q.U, Q.V, 0.0];
               Count := Count + 1;
            end if;
         end loop;
         if Count = 0 then
            return;
         end if;
         Middle_U := Sum (1) / Real (Count);
         Middle_V := Sum (2) / Real (Count);
         declare
            D : constant Vec3 := Sight (Now, Middle_U, Middle_V);
            S : constant Real := To_Table (Centre, D);
            X : constant Vec3 := Centre + S * D;
         begin
            if S <= 0.0 then
               return;
            end if;
            for K in Elsewhere'Range loop
               declare
                  U, W     : Real;
                  In_Front : Boolean;
                  Baseline : constant Real := abs (Elsewhere (K).Pose.Translation - Centre);
               begin
                  Into (Elsewhere (K), X, U, W, In_Front);
                  if In_Front and then Inside (U, W) and then Baseline > Best then
                     Best := Baseline;
                     First := K;
                  end if;
               end;
            end loop;
            if First = 0 then
               return;
            end if;
            for K in Elsewhere'Range loop
               if K /= First then
                  declare
                     U, W     : Real;
                     In_Front : Boolean;
                     Line     : constant Vec3 := Elsewhere (First).Pose.Translation - Centre;
                     Here     : constant Vec3 := Elsewhere (K).Pose.Translation - Centre;
                     Away     : constant Real := (if abs Line > 0.0 then abs Cross (Here, Line) / abs Line else 0.0);
                  begin
                     Into (Elsewhere (K), X, U, W, In_Front);
                     if In_Front and then Inside (U, W) and then Away > Off then
                        Off := Away;
                        Second := K;
                     end if;
                  end;
               end if;
            end loop;
         end;
      end;
      for I in Queries'Range loop
         if Queries (I).U >= 0.0 then
            declare
               Q        : constant Driver.Images.Pixel := Queries (I);
               D_Now    : constant Vec3 := Sight (Now, Q.U, Q.V);
               Far      : constant Real := To_Table (Centre, D_Now);
               --  The lines of sight that found the point: the eye now and each view that found it.
               Origins  : array (1 .. 3) of Vec3 := [others => Zero3];
               Ways     : array (1 .. 3) of Vec3 := [others => Zero3];
               Angles   : array (1 .. 3) of Real := [others => 0.0];
               Lines    : Natural := 1;
            begin
               Origins (1) := Centre;
               Ways (1) := D_Now;
               Angles (1) := Noise * Pixel_Angle (Q.U, Q.V);
               if Far > 0.0 then
                  for Pick in 1 .. 2 loop
                     declare
                        K : constant Natural := (if Pick = 1 then First else Second);
                     begin
                        if K > 0 then
                           declare
                              V : View renames Elsewhere (K);
                              --  The segment the point can lie on in the view: from the table towards the eye now,
                              --  clipped where it leaves the picture or comes to the eye.
                              Uf, Vf, Un, Vn : Real;
                              Front_F, Front_N : Boolean;
                              Near : Real := Far;
                           begin
                              Into (V, Centre + Far * D_Now, Uf, Vf, Front_F);
                              if Front_F and then Inside (Uf, Vf) then
                                 --  Halve the depth while the place stays in the picture, in front: the nearest depth
                                 --  the view still sees, within the halvings a float can tell.
                                 for Halving in 1 .. Real'Machine_Mantissa loop
                                    declare
                                       U2, V2 : Real;
                                       F2     : Boolean;
                                    begin
                                       Into (V, Centre + (Near / 2.0) * D_Now, U2, V2, F2);
                                       exit when not F2 or else not Inside (U2, V2);
                                       Near := Near / 2.0;
                                    end;
                                 end loop;
                                 Into (V, Centre + Near * D_Now, Un, Vn, Front_N);
                                 declare
                                    Mid_U  : constant Real := (Uf + Un) / 2.0;
                                    Mid_V  : constant Real := (Vf + Vn) / 2.0;
                                    Half   : constant Real := Sqrt ((Uf - Un) ** 2 + (Vf - Vn) ** 2) / 2.0;
                                    Gate   : constant Real := Driver.Alignment.Window_Radius (1.0);
                                    Along  : constant Real := Half / Gate;
                                    Across : constant Real := Noise;
                                    Cu     : constant Real := (if Half > 0.0 then (Uf - Un) / (2.0 * Half) else 1.0);
                                    Cv     : constant Real := (if Half > 0.0 then (Vf - Vn) / (2.0 * Half) else 0.0);
                                    Wide   : constant Real := Real'Max (Along, Across);
                                    Cov    : constant Driver.Alignment.Place_Covariance :=
                                      (UU => Wide ** 2 * Cu * Cu + Across ** 2 * Cv * Cv,
                                       UV => (Wide ** 2 - Across ** 2) * Cu * Cv,
                                       VV => Wide ** 2 * Cv * Cv + Across ** 2 * Cu * Cu);
                                    --  The warp: how the view turns the surroundings of the point at the segment's
                                    --  middle depth, its entries uncertain by how much it differs at the two ends.
                                    Mid_S  : constant Real := (Far + Near) / 2.0;
                                    function Warp_At (S : Real) return Driver.Alignment.Linear_Part is
                                       U0, V0, U1, V1, U2, V2 : Real;
                                       F0, F1, F2 : Boolean;
                                       Dist : constant Real := S;
                                    begin
                                       Into (V, Centre + Dist * D_Now, U0, V0, F0);
                                       Into (V, Centre + Dist * Sight (Now, Q.U + 1.0, Q.V), U1, V1, F1);
                                       Into (V, Centre + Dist * Sight (Now, Q.U, Q.V + 1.0), U2, V2, F2);
                                       return (UU => U1 - U0, UV => U2 - U0, VU => V1 - V0, VV => V2 - V0);
                                    end Warp_At;
                                    Lin_Mid : constant Driver.Alignment.Linear_Part := Warp_At (Mid_S);
                                    Lin_Far : constant Driver.Alignment.Linear_Part := Warp_At (Far);
                                    Lin_Near : constant Driver.Alignment.Linear_Part := Warp_At (Near);
                                    Lin_Sd  : constant Real :=
                                      Real'Max (Real'Max (abs (Lin_Far.UU - Lin_Near.UU), abs (Lin_Far.UV - Lin_Near.UV)),
                                                Real'Max (abs (Lin_Far.VU - Lin_Near.VU), abs (Lin_Far.VV - Lin_Near.VV)));
                                    Ask : constant Driver.Alignment.Prediction :=
                                      (From => Q, To => (U => Mid_U, V => Mid_V), Linear => Lin_Mid, Cov => Cov,
                                       Linear_Sigma => Real'Max (Lin_Sd, Real'Model_Epsilon));
                                    Got : constant Driver.Alignment.Answer :=
                                      Driver.Alignment.Align (Now.Picture, V.Picture, Ask);
                                 begin
                                    if Got.Verdict = Driver.Alignment.Found then
                                       declare
                                          Ok   : Boolean;
                                          Back_Linear : constant Driver.Alignment.Linear_Part := Inverse (Got.Linear, Ok);
                                          Back_Cov    : constant Driver.Alignment.Place_Covariance :=
                                            Plus (Got.Cov, (UU => Noise ** 2, UV => 0.0, VV => Noise ** 2));
                                          Back : constant Driver.Alignment.Answer :=
                                            (if Ok
                                             then Driver.Alignment.Align
                                                    (V.Picture, Now.Picture,
                                                     (From => Got.To, To => Q, Linear => Back_Linear,
                                                      Cov => Back_Cov, Linear_Sigma => Lin_Sd + Real'Model_Epsilon),
                                                     Widen => False)
                                             else Driver.Alignment.Not_Asked_Answer);
                                       begin
                                          if Back.Verdict = Driver.Alignment.Found
                                            and then not Apart (Back.To.U - Q.U, Back.To.V - Q.V, Plus (Back.Cov, Back_Cov))
                                          then
                                             Lines := Lines + 1;
                                             Origins (Lines) := V.Pose.Translation;
                                             Ways (Lines) := Sight (V, Got.To.U, Got.To.V);
                                             Angles (Lines) :=
                                               Sqrt (Real'Max (Got.Cov.UU, Got.Cov.VV) + Noise ** 2)
                                               * Pixel_Angle (Got.To.U, Got.To.V);
                                          end if;
                                       end;
                                    end if;
                                 end;
                              end if;
                           end;
                        end if;
                     end;
                  end loop;
               end if;
               if Lines >= 2 then
                  --  The lines meet: X minimises the sum of every line's squared distance to it over that distance's
                  --  variance (its angle's sigma times its range), the ranges taken from the unweighted meeting.
                  declare
                     function Meet (Weighted : Boolean; Guess : Vec3) return Vec3 is
                        A : Mat3 := [others => [others => 0.0]];
                        B : Vec3 := Zero3;
                     begin
                        for L in 1 .. Lines loop
                           declare
                              P : constant Mat3 := Identity3 - Outer (Ways (L), Ways (L));
                              R : constant Real := Real'Max (abs (Guess - Origins (L)), Real'Model_Small);
                              W : constant Real := (if Weighted then 1.0 / (Angles (L) * R) ** 2 else 1.0);
                           begin
                              A := A + W * P;
                              B := B + W * Times (P, Origins (L));
                           end;
                        end loop;
                        return Solve (A, B);
                     end Meet;
                     Guess : constant Vec3 := Meet (False, Centre);
                     X     : constant Vec3 := Meet (True, Guess);
                     A     : Mat3 := [others => [others => 0.0]];
                     Ahead : Boolean := True;
                     --  How far the lines pass from where they meet, each over its own uncertainty there: a chi square
                     --  on two degrees of freedom a line, less the three the point takes. Lines that do not meet within
                     --  their uncertainty were not all of one point: a match slid along a contour where two surfaces
                     --  at different depths meet in one patch, or took a repeat of the texture.
                     Miss  : Real := 0.0;
                  begin
                     for L in 1 .. Lines loop
                        declare
                           P : constant Mat3 := Identity3 - Outer (Ways (L), Ways (L));
                           R : constant Real := Real'Max (abs (X - Origins (L)), Real'Model_Small);
                           Off_Line : constant Vec3 := Times (P, X - Origins (L));
                        begin
                           A := A + (1.0 / (Angles (L) * R) ** 2) * P;
                           Ahead := Ahead and then Dot (Ways (L), X - Origins (L)) > 0.0;
                           Miss := Miss + Dot (Off_Line, Off_Line) / (Angles (L) * R) ** 2;
                        end;
                     end loop;
                     if Ahead
                       and then Driver.Distributions.Chi_Square_Deviate (Miss, 2 * Lines - 3) <= Z
                     then
                        declare
                           Cov    : constant Mat3 := Inverse (A);
                           Height : constant Real := Driver.Geometry.Height (Table, X);
                           Sigma  : constant Real :=
                             Sqrt (Real'Max (0.0, Dot (Table.Normal, Times (Cov, Table.Normal)))
                                   + Driver.Geometry.Height_Sigma (Table, X) ** 2);
                        begin
                           --  On the table within its own uncertainty, or below it: the table, not what stands on it.
                           if Height > Z * Sigma then
                              Result (I) := (Found => True, Point => (Mean => X, Covariance => Cov), Normal => Zero3);
                           end if;
                        end;
                     end if;
                  end;
               end if;
            end;
         end if;
      end loop;
      --  Normals from the grid's neighbours.
      for I in Queries'Range loop
         if Result (I).Found then
            declare
               K      : constant Natural := I - Queries'First;
               Column : constant Natural := K mod Columns;
               function At_Offset (Dc, Dr : Integer) return Natural is
                  C : constant Integer := Column + Dc;
                  J : constant Integer := I + Dr * Columns + Dc;
               begin
                  if C < 0 or else C >= Columns or else J < Queries'First or else J > Queries'Last then
                     return 0;
                  end if;
                  return (if Result (J).Found then J else 0);
               end At_Offset;
               L : constant Natural := At_Offset (-1, 0);
               R : constant Natural := At_Offset (1, 0);
               U : constant Natural := At_Offset (0, -1);
               D : constant Natural := At_Offset (0, 1);
            begin
               if L > 0 and then R > 0 and then U > 0 and then D > 0 then
                  declare
                     N : constant Vec3 :=
                       Cross (Result (R).Point.Mean - Result (L).Point.Mean, Result (D).Point.Mean - Result (U).Point.Mean);
                  begin
                     if abs N > 0.0 then
                        Result (I).Normal :=
                          (if Dot (N, Centre - Result (I).Point.Mean) >= 0.0 then Unit (N) else -Unit (N));
                     end if;
                  end;
               end if;
            end;
         end if;
      end loop;
   end Place;

   function In_Region (M : Model; E : Eye_Id; O : Observation; Region : Driver.Images.Mask)
     return Surface_Point_Array
   is
      Empty : Surface_Point_Array (1 .. 0);
   begin
      if Natural (E) > Eye_Count (M) or else Eye_Mount (M, E).Kind /= Arm_Carried then
         return Empty;
      end if;
      declare
         A : constant Arm_Id := Eye_Mount (M, E).Arm;
         G : constant Group_Id := Arm_Group (M, A);
         Evidence : Natural := 0;
         Placement : Rigid;
         Scale     : Real;
         Placed_In : Boolean;
      begin
         if not Kinematics.Fitted (M, A) or else not Driver.Geometry.Known (Kinematics.Table (M, A)) then
            return Empty;
         end if;
         for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
            if M.Kinematics (K).Arm = A and then M.Kinematics (K).Eye = E then
               Evidence := K;
            end if;
         end loop;
         Kinematics.In_World (M, A, Placement, Scale, Placed_In);
         if Evidence = 0 or else not Placed_In or else G > O.Readings.Last_Index
           or else O.Readings.Element (G)'Length /= Group_Size (M, G)
           or else Natural (E) > Natural (O.Images.Length)
           or else Driver.Images.Width (O.Images (E)) /= Driver.Images.Width (Region)
           or else Driver.Images.Height (O.Images (E)) /= Driver.Images.Height (Region)
           or else Driver.Images.Width (Region) = 0
         then
            return Empty;
         end if;
         declare
            Ev      : Arm_Evidence renames M.Kinematics (Evidence);
            W       : constant Natural := Driver.Images.Width (Region);
            H       : constant Natural := Driver.Images.Height (Region);
            Step    : constant Positive := Positive'Max (1, Driver.Alignment.Half_Patch (W, H));
            Columns : constant Positive := Positive'Max (1, (W + Step - 1) / Step);
            Rows    : constant Positive := Positive'Max (1, (H + Step - 1) / Step);
            Self    : constant Driver.Images.Mask := Self_Mask (M, E, O);
            Queries : Pixel_Array (1 .. Columns * Rows);
            Asked   : Natural := 0;
            Now     : constant View := (Pose    => Kinematics.Eye_In_Reference (M, A, O.Readings.Element (G)),
                                        Picture => Driver.Alignment.Pyramid_Of (O.Images (E)));
            Views   : View_Array (1 .. Natural (Ev.Frames.Length));
            function Readings_Of (F : Keyframe) return Real_Array is
              ([for C in 1 .. Natural (F.Readings.Length) => F.Readings (C - 1)]);
            function Lens_Ray_Of (U, V : Real) return Vec3 is (Kinematics.Ray_In_Eye (M, A, U, V));
            procedure Lens_Project_Of (P : Vec3; U, V : out Real; In_Front : out Boolean) is
            begin
               Kinematics.Project_In_Eye (M, A, P, U, V, In_Front);
            end Lens_Project_Of;
         begin
            for R in 0 .. Rows - 1 loop
               for C in 0 .. Columns - 1 loop
                  declare
                     U : constant Natural := C * Step + Step / 2;
                     V : constant Natural := R * Step + Step / 2;
                     K : constant Positive := R * Columns + C + 1;
                  begin
                     if U < W and then V < H and then Driver.Images.Contains (Region, U, V)
                       and then not (Driver.Images.Width (Self) = W and then Driver.Images.Contains (Self, U, V))
                     then
                        Queries (K) := (U => Real (U) + 0.5, V => Real (V) + 0.5);
                        Asked := Asked + 1;
                     else
                        Queries (K) := (U => -1.0, V => -1.0);
                     end if;
                  end;
               end loop;
            end loop;
            if Asked = 0 or else Views'Length = 0 then
               return Empty;
            end if;
            for K in Views'Range loop
               Views (K) := (Pose    => Kinematics.Eye_In_Reference
                                          (M, A, Readings_Of (Ev.Frames (Ev.Frames.First_Index + K - 1))),
                             Picture => Driver.Alignment.Pyramid_Of (Ev.Frames (Ev.Frames.First_Index + K - 1).Image));
            end loop;
            declare
               Got : Placed_Array (Queries'Range);
               N   : Natural := 0;
            begin
               Place (Now, Views, Queries, Columns, Kinematics.Table (M, A), Lens_Ray_Of'Unrestricted_Access,
                      Lens_Project_Of'Unrestricted_Access, Kinematics.Match_Noise (M, A), Got);
               for P of Got loop
                  if P.Found then
                     N := N + 1;
                  end if;
               end loop;
               return R : Surface_Point_Array (1 .. N) do
                  N := 0;
                  for K in Got'Range loop
                     if Got (K).Found then
                        N := N + 1;
                        declare
                           Turn : constant Mat3 := Placement.Rotation;
                        begin
                           R (N) := (Point  => (Mean       => Placement * (Scale * Got (K).Point.Mean),
                                                Covariance => (Scale ** 2) * Times (Times (Turn, Got (K).Point.Covariance),
                                                                                   Transposed (Turn))),
                                     Normal => Times (Turn, Got (K).Normal),
                                     Pixel  => Queries (K));
                        end;
                     end if;
                  end loop;
               end return;
            end;
         end;
      end;
   end In_Region;

end Driver.Robot.Surfaces;
