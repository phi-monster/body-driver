with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Distributions;

package body Driver.Robot.Surfaces is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Driver.Alignment.Verdict;

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
     (Now       : View;
      Elsewhere : View_Array;
      Queries   : Pixel_Array;
      Columns   : Positive;
      Table     : Driver.Geometry.Plane_Estimate;
      Ray       : not null Lens_Ray;
      Project   : not null Lens_Project;
      Result    : out Placed_Array)
   is
      Z      : constant Real := Driver.Conventions.Z;
      Centre : constant Vec3 := Now.Pose.Translation;

      --  Where a point of the frame lands in a view's picture.
      procedure Into (V : View; P : Vec3; U, W : out Real; In_Front : out Boolean) is
      begin
         Project (V.Lens, Inverse (V.Pose) * P, U, W, In_Front);
      end Into;

      --  The line of sight through a pixel of a view, in the frame.
      function Sight (V : View; U, W : Real) return Vec3 is (Times (V.Pose.Rotation, Ray (V.Lens, U, W)));

      --  The angle a pixel spans along a line of sight: the eye's resolution there.
      function Pixel_Angle (V : View; U, W : Real) return Real is
        (Arccos (Real'Min (1.0, Dot (Ray (V.Lens, U, W), Ray (V.Lens, U + 1.0, W)))));

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

      function Inside (V : View; U, W : Real) return Boolean is
        (U >= 0.0 and then W >= 0.0 and then U <= Real (Driver.Alignment.Level_Width (V.Picture, 0))
         and then W <= Real (Driver.Alignment.Level_Height (V.Picture, 0)));
   begin
      for R of Result loop
         R := (others => <>);
      end loop;
      if Elsewhere'Length = 0 or else not Driver.Geometry.Known (Table) or else Now.Noise <= 0.0 then
         return;
      end if;
      for I in Queries'Range loop
         if Queries (I).U >= 0.0 then
            declare
               Q     : constant Driver.Images.Pixel := Queries (I);
               D_Now : constant Vec3 := Sight (Now, Q.U, Q.V);
               Far   : constant Real := To_Table (Centre, D_Now);
               X_Far : constant Vec3 := Centre + Far * D_Now;
               --  The views that see where the line of sight meets the table, the one standing farthest from the eye
               --  now first: the wider a view's baseline, the better it tells the depth.
               Order : array (1 .. Elsewhere'Length) of Natural := [others => 0];
               Count : Natural := 0;
               --  The lines of sight that found the point: the eye now's first, then each view's that found it.
               Origins : array (1 .. Elsewhere'Length + 1) of Vec3 := [others => Zero3];
               Ways    : array (1 .. Elsewhere'Length + 1) of Vec3 := [others => Zero3];
               Angles  : array (1 .. Elsewhere'Length + 1) of Real := [others => 0.0];
               Lines   : Natural := 1;
               Done    : Boolean := False;

               --  Whether the lines but Drop (0: none) meet, and where, with what uncertainty: X minimises the sum of
               --  every line's squared distance to it over that distance's variance (its angle's sigma times its
               --  range), the ranges taken from the unweighted meeting; the lines must pass within their uncertainty
               --  of it (a chi square on two degrees of freedom a line, less the three the point takes) and see it
               --  ahead. Lines that do not meet were not all of one point: a match slid along a contour where two
               --  surfaces at different depths meet in one patch, or took a repeat of the texture along the segment.
               procedure Meet_All (Drop : Natural; X : out Vec3; Cov : out Mat3; Ok : out Boolean) is
                  function Meet (Weighted : Boolean; Guess : Vec3) return Vec3 is
                     A : Mat3 := [others => [others => 0.0]];
                     B : Vec3 := Zero3;
                  begin
                     for L in 1 .. Lines loop
                        if L /= Drop then
                           declare
                              P : constant Mat3 := Identity3 - Outer (Ways (L), Ways (L));
                              R : constant Real := Real'Max (abs (Guess - Origins (L)), Real'Model_Small);
                              W : constant Real := (if Weighted then 1.0 / (Angles (L) * R) ** 2 else 1.0);
                           begin
                              A := A + W * P;
                              B := B + W * Times (P, Origins (L));
                           end;
                        end if;
                     end loop;
                     return Solve (A, B);
                  end Meet;
                  Used  : constant Natural := Lines - (if Drop > 0 then 1 else 0);
                  A     : Mat3 := [others => [others => 0.0]];
                  Ahead : Boolean := True;
                  Miss  : Real := 0.0;
               begin
                  X := Meet (True, Meet (False, Centre));
                  for L in 1 .. Lines loop
                     if L /= Drop then
                        declare
                           P : constant Mat3 := Identity3 - Outer (Ways (L), Ways (L));
                           R : constant Real := Real'Max (abs (X - Origins (L)), Real'Model_Small);
                           Off_Line : constant Vec3 := Times (P, X - Origins (L));
                        begin
                           A := A + (1.0 / (Angles (L) * R) ** 2) * P;
                           Ahead := Ahead and then Dot (Ways (L), X - Origins (L)) > 0.0;
                           Miss := Miss + Dot (Off_Line, Off_Line) / (Angles (L) * R) ** 2;
                        end;
                     end if;
                  end loop;
                  Ok := Ahead and then Driver.Distributions.Chi_Square_Deviate (Miss, 2 * Used - 3) <= Z;
                  Cov := (if Ok then Inverse (A) else [others => [others => 0.0]]);
               end Meet_All;

               --  How far along the eye now's line of sight the point is uncertain, by the covariance it was met with.
               function Depth_Sigma (Cov : Mat3) return Real is (Sqrt (Real'Max (0.0, Dot (D_Now, Times (Cov, D_Now)))));

               --  How many of the views' lines but Drop's tell the depth by themselves about as well as all of them
               --  together: the eye now's line and that one alone meet with a depth no more uncertain than Z times the
               --  meeting's. A depth only one view tells is that view's alone, and a wrong match of it (a repeat of the
               --  texture along its segment) meets the other lines within their wider uncertainty: a view standing a
               --  centimetre from the eye now tells the depth a few times worse than one standing seven, and passed
               --  a wrong one by 1.6 cm (the scene test, before this). A second view that tells it as well can
               --  refute it.
               function Telling (Drop : Natural; X : Vec3; Cov : Mat3) return Natural is
                  Whole : constant Real := Depth_Sigma (Cov);
                  Count : Natural := 0;
                  function Weighted (L : Positive) return Mat3 is
                    ((1.0 / (Angles (L) * Real'Max (abs (X - Origins (L)), Real'Model_Small)) ** 2)
                     * (Identity3 - Outer (Ways (L), Ways (L))));
               begin
                  for L in 2 .. Lines loop
                     if L /= Drop then
                        declare
                           A   : constant Mat3 := Weighted (1) + Weighted (L);
                           Det : constant Real := Determinant (A);
                        begin
                           if Det > 0.0 and then Depth_Sigma (Inverse (A)) <= Z * Whole then
                              Count := Count + 1;
                           end if;
                        end;
                     end if;
                  end loop;
                  return Count;
               end Telling;

               --  The point, from three lines that meet or more, two views of them telling its depth: every line found,
               --  or every line but one when one of them did not meet the rest (the eye now's own line is never the one
               --  left out). Until two views tell the depth, more views are searched.
               procedure Try is
                  X    : Vec3;
                  Cov  : Mat3;
                  Ok   : Boolean;
                  Drop : Natural := 0;
               begin
                  Meet_All (0, X, Cov, Ok);
                  if not Ok and then Lines >= 4 then
                     for D in 2 .. Lines loop
                        Meet_All (D, X, Cov, Ok);
                        Drop := D;
                        exit when Ok;
                     end loop;
                  end if;
                  if Ok and then Telling ((if Ok then Drop else 0), X, Cov) >= 2 then
                     Done := True;
                     declare
                        Height : constant Real := Driver.Geometry.Height (Table, X);
                        Sigma  : constant Real :=
                          Sqrt (Real'Max (0.0, Dot (Table.Normal, Times (Cov, Table.Normal)))
                                + Driver.Geometry.Height_Sigma (Table, X) ** 2);
                     begin
                        --  Above the table beyond its own uncertainty: what stands on it. Within it: the table there.
                        --  Below it beyond that: no point the line of sight could end at.
                        if Height > Z * Sigma then
                           Result (I) := (Found => True, Point => (Mean => X, Covariance => Cov), Normal => Zero3,
                                          On_Table => False);
                        elsif Height >= -Z * Sigma then
                           Result (I) := (Found => True, Point => (Mean => X, Covariance => Cov), Normal => Zero3,
                                          On_Table => True);
                        end if;
                     end;
                  end if;
               end Try;

               --  The view's place of the point at depth S along the eye now's line of sight.
               procedure At_Depth (V : View; S : Real; U, W : out Real; In_Front : out Boolean) is
               begin
                  Into (V, Centre + S * D_Now, U, W, In_Front);
               end At_Depth;

               --  How the view turns the surroundings of the point at depth S: the derivative of its place there by
               --  the place in the eye now's picture.
               function Warp_At (V : View; S : Real) return Driver.Alignment.Linear_Part is
                  U0, V0, U1, V1, U2, V2 : Real;
                  F0, F1, F2 : Boolean;
               begin
                  Into (V, Centre + S * D_Now, U0, V0, F0);
                  Into (V, Centre + S * Sight (Now, Q.U + 1.0, Q.V), U1, V1, F1);
                  Into (V, Centre + S * Sight (Now, Q.U, Q.V + 1.0), U2, V2, F2);
                  return (UU => U1 - U0, UV => U2 - U0, VU => V1 - V0, VV => V2 - V0);
               end Warp_At;

               --  A search of the view: its answer, kept when its place, aligned back into the eye now's picture,
               --  comes to the query again (a texture repeated along the way can be taken for the point once, not in
               --  both directions).
               function Search (V : View; Ask : Driver.Alignment.Prediction; Got : out Driver.Alignment.Answer)
                 return Boolean
               is
                  Ok : Boolean;
               begin
                  Got := Driver.Alignment.Align_At_Scale (Now.Picture, V.Picture, Ask);
                  if Got.Verdict /= Driver.Alignment.Found then
                     return False;
                  end if;
                  declare
                     Back_Linear : constant Driver.Alignment.Linear_Part := Inverse (Got.Linear, Ok);
                     Back_Cov    : constant Driver.Alignment.Place_Covariance :=
                       Plus (Got.Cov, (UU => Now.Noise ** 2, UV => 0.0, VV => Now.Noise ** 2));
                  begin
                     if not Ok then
                        return False;
                     end if;
                     declare
                        Back : constant Driver.Alignment.Answer :=
                          Driver.Alignment.Align_At_Scale
                            (V.Picture, Now.Picture,
                             (From => Got.To, To => Q, Linear => Back_Linear, Cov => Back_Cov,
                              Linear_Sigma => Ask.Linear_Sigma), Widen => False);
                     begin
                        return Back.Verdict = Driver.Alignment.Found
                          and then not Apart (Back.To.U - Q.U, Back.To.V - Q.V, Plus (Back.Cov, Back_Cov));
                     end;
                  end;
               end Search;

               procedure Add_Line (V : View; Got : Driver.Alignment.Answer) is
               begin
                  Lines := Lines + 1;
                  Origins (Lines) := V.Pose.Translation;
                  Ways (Lines) := Sight (V, Got.To.U, Got.To.V);
                  Angles (Lines) :=
                    Sqrt (Real'Max (Got.Cov.UU, Got.Cov.VV) + V.Noise ** 2) * Pixel_Angle (V, Got.To.U, Got.To.V);
               end Add_Line;

               --  A place the point may be at, asked of every view but Scanned (0: none) where it would be: there,
               --  within its uncertainty carried into the view's picture and the view's own error. Each answer kept is a
               --  line more, until the point is told (Try).
               procedure Verify (Scanned : Natural; X : Vec3; Cov : Mat3) is
               begin
                  for Other in 1 .. Count loop
                     exit when Done;
                     if Order (Other) /= Scanned then
                        declare
                           V      : View renames Elsewhere (Order (Other));
                           U0, W0 : Real;
                           F0     : Boolean;
                        begin
                           Into (V, X, U0, W0, F0);
                           if F0 and then Inside (V, U0, W0) then
                              declare
                                 --  The derivative of the view's place by the point, a column per axis of the frame.
                                 J  : array (1 .. 2, 1 .. 3) of Real := [others => [others => 0.0]];
                                 Pc : Driver.Alignment.Place_Covariance := (UU => V.Noise ** 2, UV => 0.0, VV => V.Noise ** 2);
                                 --  A step of the point small against where it lies: its own uncertainty.
                                 Step : constant Real := Real'Max (Sqrt (Real'Max (0.0, Cov (1, 1) + Cov (2, 2) + Cov (3, 3))),
                                                                   Real'Model_Epsilon * abs (X - Centre));
                                 Got  : Driver.Alignment.Answer;
                              begin
                                 for A in 1 .. 3 loop
                                    declare
                                       E      : Vec3 := Zero3;
                                       U1, W1 : Real;
                                       F1     : Boolean;
                                    begin
                                       E (A) := Step;
                                       Into (V, X + E, U1, W1, F1);
                                       J (1, A) := (U1 - U0) / Step;
                                       J (2, A) := (W1 - W0) / Step;
                                    end;
                                 end loop;
                                 for A in 1 .. 3 loop
                                    for B in 1 .. 3 loop
                                       Pc.UU := Pc.UU + J (1, A) * Cov (A, B) * J (1, B);
                                       Pc.UV := Pc.UV + J (1, A) * Cov (A, B) * J (2, B);
                                       Pc.VV := Pc.VV + J (2, A) * Cov (A, B) * J (2, B);
                                    end loop;
                                 end loop;
                                 declare
                                    S_Here : constant Real := Dot (X - Centre, D_Now);
                                 begin
                                    if Search (V, (From => Q, To => (U => U0, V => W0), Linear => Warp_At (V, S_Here),
                                                   Cov => Pc, Linear_Sigma => Real'Model_Epsilon), Got)
                                    then
                                       Add_Line (V, Got);
                                       Try;
                                    end if;
                                 end;
                              end;
                           end if;
                        end;
                     end if;
                  end loop;
               end Verify;

               --  A view scans its segment of the eye now's line of sight, from the nearest depth it sees towards the
               --  table: windows the aligner searches where the pictures are finest (Finest_Radius), each a window's
               --  width on along the segment from the one before, so that the segment is searched whole and no answer
               --  is a coarser size's guess; each answer a candidate the other views are asked about (Verify). Nearest
               --  first: a pixel shows the nearest surface along its line of sight, and where its patch takes in the
               --  edge of a thing and what lies behind it, the views can bear out both; scanned from the table, the
               --  scene test's box top's corners were placed on the table behind it, 8.7 cm off.
               procedure Scan (K : Positive) is
                  V      : View renames Elsewhere (K);
                  Radius : constant Real := Driver.Alignment.Finest_Radius
                    (Driver.Alignment.Level_Width (V.Picture, 0), Driver.Alignment.Level_Height (V.Picture, 0));
                  Along  : constant Real := Driver.Alignment.Sigma_Of_Window (Radius);
                  Near   : Real := Far;
                  S      : Real;
                  function Seen (D : Real) return Boolean is
                     U2, V2 : Real;
                     F2     : Boolean;
                  begin
                     At_Depth (V, D, U2, V2, F2);
                     return F2 and then Inside (V, U2, V2);
                  end Seen;
               begin
                  if not Seen (Far) or else Radius <= 0.0 then
                     return;
                  end if;
                  --  The nearest depth the view still sees: halved from the table while it stays seen, then the depth
                  --  between the last seen and the first not seen halved again, within the halvings a float can tell.
                  for Halving in 1 .. Real'Machine_Mantissa loop
                     exit when not Seen (Near / 2.0);
                     Near := Near / 2.0;
                  end loop;
                  declare
                     Unseen : Real := Near / 2.0;
                  begin
                     for Halving in 1 .. Real'Machine_Mantissa loop
                        declare
                           Between : constant Real := (Unseen + Near) / 2.0;
                        begin
                           exit when Between <= Unseen or else Between >= Near;
                           if Seen (Between) then
                              Near := Between;
                           else
                              Unseen := Between;
                           end if;
                        end;
                     end loop;
                  end;
                  S := Near;
                  loop
                     exit when Done;
                     declare
                        U, W     : Real;
                        F        : Boolean;
                        Next     : Real := Far;   --  the depth of the next window's middle
                        Un, Wn   : Real;
                        Fn       : Boolean;
                     begin
                        At_Depth (V, S, U, W, F);
                        At_Depth (V, Far, Un, Wn, Fn);
                        declare
                           Run : constant Real := Sqrt ((Un - U) ** 2 + (Wn - W) ** 2);
                           Cu  : constant Real := (if Run > 0.0 then (Un - U) / Run else 1.0);
                           Cv  : constant Real := (if Run > 0.0 then (Wn - W) / Run else 0.0);
                           Across : constant Real :=
                             Sqrt (V.Noise ** 2 + (Now.Noise * Pixel_Angle (Now, Q.U, Q.V) / Pixel_Angle (V, U, W)) ** 2);
                           Cov : constant Driver.Alignment.Place_Covariance :=
                             (UU => Along ** 2 * Cu * Cu + Across ** 2 * Cv * Cv,
                              UV => (Along ** 2 - Across ** 2) * Cu * Cv,
                              VV => Along ** 2 * Cv * Cv + Across ** 2 * Cu * Cu);
                           Got : Driver.Alignment.Answer;
                        begin
                           if Search (V, (From => Q, To => (U => U, V => W), Linear => Warp_At (V, S), Cov => Cov,
                                          Linear_Sigma => Real'Model_Epsilon), Got)
                           then
                              Lines := 1;
                              Add_Line (V, Got);
                              declare
                                 X  : Vec3;
                                 C  : Mat3;
                                 Ok : Boolean;
                              begin
                                 Meet_All (0, X, C, Ok);
                                 if Ok then
                                    Verify (K, X, C);
                                 end if;
                              end;
                           end if;
                           exit when Done or else Run <= Real'Model_Small or else S >= Far;
                           --  The next window's middle: a window's width on along the segment, or its end at the table.
                           if Run > 2.0 * Radius then
                              declare
                                 Lo : Real := S;
                                 Hi : Real := Far;
                              begin
                                 for Halving in 1 .. Real'Machine_Mantissa loop
                                    declare
                                       Mid    : constant Real := (Lo + Hi) / 2.0;
                                       Um, Wm : Real;
                                       Fm     : Boolean;
                                    begin
                                       exit when Mid <= Lo or else Mid >= Hi;
                                       At_Depth (V, Mid, Um, Wm, Fm);
                                       if Sqrt ((Um - U) ** 2 + (Wm - W) ** 2) > 2.0 * Radius then
                                          Hi := Mid;
                                       else
                                          Lo := Mid;
                                       end if;
                                    end;
                                 end loop;
                                 Next := Lo;
                              end;
                           end if;
                           exit when Next <= S;
                           S := Next;
                        end;
                     end;
                  end loop;
               end Scan;
            begin
               Origins (1) := Centre;
               Ways (1) := D_Now;
               Angles (1) := Now.Noise * Pixel_Angle (Now, Q.U, Q.V);
               if Far > 0.0 then
                  for K in Elsewhere'Range loop
                     declare
                        U, W     : Real;
                        In_Front : Boolean;
                        Baseline : constant Real := abs (Elsewhere (K).Pose.Translation - Centre);
                     begin
                        Into (Elsewhere (K), X_Far, U, W, In_Front);
                        if In_Front and then Inside (Elsewhere (K), U, W) and then Baseline > 0.0 then
                           declare
                              At_Place : Positive := Count + 1;
                           begin
                              while At_Place > 1
                                and then abs (Elsewhere (Order (At_Place - 1)).Pose.Translation - Centre) < Baseline
                              loop
                                 Order (At_Place) := Order (At_Place - 1);
                                 At_Place := At_Place - 1;
                              end loop;
                              Order (At_Place) := K;
                              Count := Count + 1;
                           end;
                        end if;
                     end;
                  end loop;
               end if;
               --  The depths of the neighbours placed already, before it on its row and in the row above, asked first: a
               --  surface goes on from one pixel of the grid to the next but where it ends, and a place the views bear
               --  out there needs no scan. Each is asked where its point lies along this query's line of sight, as
               --  uncertain as it was placed; where the surface ends, the views do not bear it out.
               --  The nearer first: the pixel shows the nearest surface along its line of sight.
               declare
                  Left  : constant Natural := (if (I - Queries'First) mod Columns > 0 then I - 1 else 0);
                  Above : constant Natural := (if I - Queries'First >= Columns then I - Columns else 0);
                  function Depth_Of (N : Natural) return Real is
                    (if N > 0 and then Result (N).Found then Dot (Result (N).Point.Mean - Centre, D_Now) else 0.0);
                  First_Asked : constant Natural :=
                    (if Depth_Of (Above) > 0.0 and then (Depth_Of (Left) <= 0.0 or else Depth_Of (Above) < Depth_Of (Left))
                     then Above else Left);
                  Then_Asked  : constant Natural := (if First_Asked = Left then Above else Left);
               begin
                  for Neighbour of Natural_Array'[First_Asked, Then_Asked] loop
                     exit when Done or else Far <= 0.0;
                     if Depth_Of (Neighbour) > 0.0 then
                        Lines := 1;
                        Verify (0, Centre + Depth_Of (Neighbour) * D_Now, Result (Neighbour).Point.Covariance);
                     end if;
                  end loop;
               end;
               --  Every view in turn scans its segment, the widest baseline first, until the point is told.
               for Pick in 1 .. Count loop
                  exit when Done;
                  Scan (Order (Pick));
               end loop;
            end;
         end if;
      end loop;
      --  A point whose patch takes in two surfaces is placed by neither: where the depths of its neighbours along its
      --  row or its column do not go on through it as a surface does (their second difference with it is beyond Z of
      --  its own uncertainty), the patch spans an edge of a thing or what it hides, and the views bear it out at one
      --  depth or the other, its place slid along the edge: the scene test's box's upright edges were placed a
      --  millimetre off its sides. A surface that only leans goes on through it. The neighbours either side of it, or
      --  where it has none on one side, the two beyond it on the other: the last row a surface shows before what it
      --  hides has neighbours on one side only. (The world test's table's far edge against the black beyond it is
      --  placed 2 to 4 cm above the table, a centimetre uncertain at its grazing view: within that, its depths go on
      --  through, and only 1 of its 13 points is left out.)
      declare
         Edge : array (Result'Range) of Boolean := [others => False];
      begin
         for I in Result'Range loop
            if Result (I).Found then
               declare
                  K      : constant Natural := I - Result'First;
                  Column : constant Natural := K mod Columns;
                  Way    : constant Vec3 := Unit (Result (I).Point.Mean - Now.Pose.Translation);
                  function Depth (J : Positive) return Real is (Dot (Result (J).Point.Mean - Now.Pose.Translation, Way));
                  function Sigma (J : Positive) return Real is
                    (Sqrt (Real'Max (0.0, Dot (Way, Times (Result (J).Point.Covariance, Way)))));
                  function Placed (J : Integer) return Boolean is
                    (J >= Result'First and then J <= Result'Last and then Result (J).Found);
                  --  The second difference of three placed points in a line, I one of them, against Z of its spread.
                  procedure Through (A, B, C : Integer) is
                  begin
                     if Placed (A) and then Placed (B) and then Placed (C) then
                        declare
                           Second : constant Real := Depth (A) - 2.0 * Depth (B) + Depth (C);
                           Spread : constant Real := Sqrt (Sigma (A) ** 2 + 4.0 * Sigma (B) ** 2 + Sigma (C) ** 2);
                        begin
                           if abs Second > Driver.Conventions.Z * Spread then
                              Edge (I) := True;
                           end if;
                        end;
                     end if;
                  end Through;
                  --  Along one way of the grid (Step a query apart, Before and After whether there are queries that way).
                  procedure Along (Step : Positive; Before, After, Two_Before, Two_After : Boolean) is
                  begin
                     if Before and then After and then Placed (I - Step) and then Placed (I + Step) then
                        Through (I - Step, I, I + Step);
                     elsif Two_After and then not Placed (I - Step) then
                        Through (I, I + Step, I + 2 * Step);
                     elsif Two_Before and then not Placed (I + Step) then
                        Through (I - 2 * Step, I - Step, I);
                     end if;
                  end Along;
               begin
                  Along (1, Column > 0, Column < Columns - 1, Column > 1, Column < Columns - 2);
                  Along (Columns, I - Columns >= Result'First, I + Columns <= Result'Last,
                         I - 2 * Columns >= Result'First, I + 2 * Columns <= Result'Last);
               end;
            end if;
         end loop;
         for I in Result'Range loop
            if Edge (I) then
               Result (I) := (others => <>);
            end if;
         end loop;
      end;
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

end Driver.Robot.Surfaces;
