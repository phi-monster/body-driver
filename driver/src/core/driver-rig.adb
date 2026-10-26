with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Numerics.Dense;
with Driver.Stats;

package body Driver.Rig is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   type Real_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   type Matrix_Access is access Real_Matrix;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Matrix, Matrix_Access);
   type Vec3_Array is array (Positive range <>) of Vec3;

   function Norm (V : Vec3) return Real is (Sqrt (Dot (V, V)));

   function Huber (Z_Value : Real) return Real is
     (if abs Z_Value <= Driver.Conventions.Z then 1.0 else Driver.Conventions.Z / abs Z_Value);

   function Huber_Cost (Z_Value : Real) return Real is
     (if abs Z_Value <= Driver.Conventions.Z then Z_Value * Z_Value / 2.0
      else Driver.Conventions.Z * abs Z_Value - Driver.Conventions.Z ** 2 / 2.0);

   function Ray (L : Lens; P : Place) return Vec3 is ([(P.U - L.Cx) / L.Focal, (P.V - L.Cy) / L.Focal, 1.0]);

   --  The residuals' own scale, in the sigmas the answers state: what their median absolute value implies for Gaussian
   --  errors (Driver.Stats.Sigma_Of_Deviation), no less than one (the answers' own). What the model leaves beside the
   --  aligner's errors (the walk's inverse depths, a sway between two eyes' pictures of one moment) widens it, and
   --  Huber's cut at Z of it then keeps the points that agree as far as the points agree: at Z of the stated sigmas
   --  alone, a pose placed from inverse depths 5 % off left every point beyond the cut, and none was placed
   --  (robot.base.shift's noisy case, 10-27).
   function Scale_Of (R : Real_Array) return Real is
      A : Real_Array (R'Range);
   begin
      if R'Length = 0 then
         return 1.0;
      end if;
      for I in R'Range loop
         A (I) := abs R (I);
      end loop;
      return Real'Max (1.0, Driver.Stats.Sigma_Of_Deviation (Driver.Stats.Median (A)));
   end Scale_Of;

   procedure Project (L : Lens; Q : Vec3; P : out Place; Ahead : out Boolean) is
   begin
      Ahead := Q (3) > 0.0;
      if Ahead then
         P := (U => L.Focal * Q (1) / Q (3) + L.Cx, V => L.Focal * Q (2) / Q (3) + L.Cy);
      else
         P := (U => L.Cx, V => L.Cy);
      end if;
   end Project;

   --  The residual, over its sigma, of a sighting behind the eye: as far off as the picture's diagonal (a picture whose
   --  centre is its middle), a residual every place in the picture is nearer than; bounded, so that a step that puts a
   --  point behind costs that much and no more.
   function Far (L : Lens) return Real is (2.0 * Sqrt (L.Cx ** 2 + L.Cy ** 2));

   function Pose (Turn, Place_At : Vec3) return Rigid is ((Rotation => Exp (Turn), Translation => Place_At));

   --  Where the eye that moved by Motion sees the point along R at Inverse, and how far off Seen_At that is, over
   --  Sigma, per axis.
   procedure Miss
     (L : Lens; Motion : Rigid; R : Vec3; Inverse : Real; Seen_At : Place; Sigma : Real; Du, Dv : out Real)
   is
      P     : Place;
      Ahead : Boolean;
   begin
      Project (L, Seen (Motion, R, Inverse), P, Ahead);
      if Ahead then
         Du := (P.U - Seen_At.U) / Sigma;
         Dv := (P.V - Seen_At.V) / Sigma;
      else
         Du := Far (L) / Sigma;
         Dv := Far (L) / Sigma;
      end if;
   end Miss;

   ---------------------------------------------------------------------------
   --  Locate

   procedure Locate
     (L         : Lens;
      Rays      : Driver.Numerics.Arrays.Real_Matrix;
      Seen_At   : Place_Array;
      Sigmas    : Real_Array;
      Start     : Rigid;
      Result    : out Locate_Result)
   is
      N      : constant Natural := Seen_At'Length;
      X      : Real_Array (1 .. 6);
      R0, Rn : Real_Access := new Real_Array (1 .. 2 * N);
      Lambda : Real := Driver.Conventions.Initial_Damping;
      Cost0  : Real;
      Scale  : Real := 1.0;   --  the residuals' own (Scale_Of), at the iteration's start

      function Pose_Of (Y : Real_Array) return Rigid is (Pose ([Y (1), Y (2), Y (3)], [Y (4), Y (5), Y (6)]));

      procedure Evaluate (Y : Real_Array; R : out Real_Array) is
         T : constant Rigid := Pose_Of (Y);
      begin
         for K in 1 .. N loop
            declare
               Row : constant Natural := Rays'First (1) + K - 1;
               C   : constant Natural := Rays'First (2);
            begin
               Miss (L, T, [Rays (Row, C), Rays (Row, C + 1), Rays (Row, C + 2)], Rays (Row, C + 3),
                     Seen_At (Seen_At'First + K - 1), Sigmas (Sigmas'First + K - 1), R (2 * K - 1), R (2 * K));
            end;
         end loop;
      end Evaluate;

      function Cost (R : Real_Array) return Real is
         S : Real := 0.0;
      begin
         for V of R loop
            S := S + Huber_Cost (V / Scale);
         end loop;
         return S;
      end Cost;

      A : Real_Matrix (1 .. 6, 1 .. 6);
      G : Real_Vector (1 .. 6);
      J : Matrix_Access := new Real_Matrix (1 .. 2 * N, 1 .. 6);
   begin
      Result := (others => <>);
      X (1 .. 3) := Real_Array (Log (Start.Rotation));
      X (4 .. 6) := Real_Array (Start.Translation);
      Evaluate (X, R0.all);
      loop
         Scale := Scale_Of (R0.all);
         Cost0 := Cost (R0.all);
         --  Forward differences at the square root of the float's resolution relative to each parameter's size.
         for P in 1 .. 6 loop
            declare
               Xp : Real_Array := X;
               H  : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs X (P));
            begin
               Xp (P) := Xp (P) + H;
               Evaluate (Xp, Rn.all);
               for I in 1 .. 2 * N loop
                  J (I, P) := (Rn (I) - R0 (I)) / H;
               end loop;
            end;
         end loop;
         A := [others => [others => 0.0]];
         G := [others => 0.0];
         for I in 1 .. 2 * N loop
            declare
               W : constant Real := Huber (R0 (I) / Scale);
            begin
               for P in 1 .. 6 loop
                  G (P) := G (P) - W * J (I, P) * R0 (I);
                  for Q in 1 .. 6 loop
                     A (P, Q) := A (P, Q) + W * J (I, P) * J (I, Q);
                  end loop;
               end loop;
            end;
         end loop;
         declare
            Improved, Lowered : Boolean := False;
         begin
            loop
               declare
                  D     : Real_Matrix := A;
                  Lf    : Real_Matrix (1 .. 6, 1 .. 6);
                  Pd    : Boolean;
                  Moves : Boolean := False;
               begin
                  for P in 1 .. 6 loop
                     D (P, P) := A (P, P) * (1.0 + Lambda) + Lambda * Real'Model_Small;
                  end loop;
                  Driver.Numerics.Dense.Cholesky (D, Lf, Pd);
                  if Pd then
                     declare
                        Step : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, G);
                        Xn   : Real_Array := X;
                     begin
                        for P in 1 .. 6 loop
                           Xn (P) := X (P) + Step (P);
                           Moves := Moves or else Xn (P) /= X (P);
                        end loop;
                        if Moves then
                           Evaluate (Xn, Rn.all);
                           if Cost (Rn.all) < Cost0 then
                              Improved := Cost0 - Cost (Rn.all) > Driver.Conventions.Unchanged_Fraction * Cost0;
                              X := Xn;
                              R0.all := Rn.all;
                              Cost0 := Cost (Rn.all);
                              Lambda := Lambda / 2.0;
                              Lowered := True;
                           end if;
                        end if;
                     end;
                  else
                     Moves := True;
                  end if;
                  exit when Lowered or else not Moves or else Lambda >= Real'Last / 2.0;
                  Lambda := 2.0 * Lambda;
               end;
            end loop;
            exit when not Improved;
         end;
      end loop;
      --  The covariance: the inverse of the weighted normal matrix at the end, times the residuals' scale over their
      --  degrees of freedom (the weighted squares' mean, as a robust fit's own sigma).
      declare
         Squares : Real := 0.0;
         Within  : Natural := 0;
         Larger  : Real_Access := new Real_Array (1 .. Natural'Max (1, N));
      begin
         Scale := Scale_Of (R0.all);
         A := [others => [others => 0.0]];
         for I in 1 .. 2 * N loop
            declare
               W : constant Real := Huber (R0 (I) / Scale);
            begin
               Squares := Squares + W * R0 (I) ** 2;
               for P in 1 .. 6 loop
                  for Q in 1 .. 6 loop
                     A (P, Q) := A (P, Q) + W * J (I, P) * J (I, Q);
                  end loop;
               end loop;
            end;
         end loop;
         for K in 1 .. N loop
            Larger (K) := Real'Max (abs R0 (2 * K - 1), abs R0 (2 * K));
            if Larger (K) <= Driver.Conventions.Z * Scale then
               Within := Within + 1;
            end if;
         end loop;
         Result.Pose := Pose_Of (X);
         Result.Scale := Scale;
         Result.Used := N;
         Result.Kept := Within;
         Result.Misfit := (if N > 0 then Driver.Stats.Median (Larger (1 .. N)) else 0.0);
         declare
            Lf : Real_Matrix (1 .. 6, 1 .. 6);
            Pd : Boolean;
         begin
            Driver.Numerics.Dense.Cholesky (A, Lf, Pd);
            if Pd and then 2 * N > 6 then
               for P in 1 .. 6 loop
                  declare
                     E : Real_Vector (1 .. 6) := [others => 0.0];
                  begin
                     E (P) := 1.0;
                     E := Driver.Numerics.Dense.Cholesky_Solve (Lf, E);
                     for Q in 1 .. 6 loop
                        Result.Covariance (Q, P) := E (Q) * Squares / Real (2 * N - 6);
                     end loop;
                  end;
               end loop;
            end if;
            --  Six points kept and more (within Z of the residuals' own scale): three are the fewest that place an eye,
            --  and three more leave as many residuals over to judge it by.
            Result.Ok := Pd and then Within >= 6;
         end;
         Free (Larger);
      end;
      Free (R0);
      Free (Rn);
      Free (J);
   end Locate;

   ---------------------------------------------------------------------------
   --  Triangulate

   procedure Triangulate
     (L : Lens; R : Vec3; Poses : Rigid_Array; Seen_At : Place_Array; Sigmas : Real_Array; Start : Real;
      Inverse_Depth : out Real; Ok : out Boolean)
   is
      N     : constant Natural := Poses'Length;
      Rho   : Real := Start;
      Res   : Real_Array (1 .. 2 * Natural'Max (1, N));
      Scale : Real := 1.0;
      procedure Evaluate (X : Real; Out_R : out Real_Array) is
      begin
         for K in 1 .. N loop
            Miss (L, Poses (Poses'First + K - 1), R, X, Seen_At (Seen_At'First + K - 1),
                  Sigmas (Sigmas'First + K - 1), Out_R (2 * K - 1), Out_R (2 * K));
         end loop;
      end Evaluate;
      A : Real := 0.0;
   begin
      Inverse_Depth := Start;
      Ok := False;
      if N = 0 then
         return;
      end if;
      for Iteration in 1 .. Real'Machine_Mantissa loop
         declare
            H      : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs Rho);
            Up     : Real_Array (1 .. 2 * N);
            G      : Real := 0.0;
            Before : constant Real := Rho;
         begin
            Evaluate (Rho, Res (1 .. 2 * N));
            Evaluate (Rho + H, Up);
            Scale := Scale_Of (Res (1 .. 2 * N));
            A := 0.0;
            for I in 1 .. 2 * N loop
               declare
                  W : constant Real := Huber (Res (I) / Scale);
                  D : constant Real := (Up (I) - Res (I)) / H;
               begin
                  A := A + W * D * D;
                  G := G - W * D * Res (I);
               end;
            end loop;
            exit when not (A > 0.0);
            Rho := Real'Max (0.0, Rho + G / A);
            exit when abs (Rho - Before)
                      <= Driver.Conventions.Unchanged_Fraction * Real'Max (abs Before, Real'Model_Small);
         end;
      end loop;
      Inverse_Depth := Rho;
      Ok := A > 0.0;
   end Triangulate;

   ---------------------------------------------------------------------------
   --  Calibrate

   procedure Calibrate
     (First_Points  : Place_Array;
      Second_Points : Place_Array;
      Sightings     : Sighting_Array;
      Stills        : Positive;
      First_Start   : Lens;
      Second_Centre : Place;
      Result        : out Calibration)
   is
      N1 : constant Natural := First_Points'Length;
      N2 : constant Natural := Second_Points'Length;
      Nd : constant Natural := N1 + N2;
      Ns : constant Natural := Sightings'Length;
      S  : constant Positive := Stills;
      Np : constant Positive := 6 * S + 8;   --  each still's turn and place, the second eye's on the first, two focals

      --  The parameters: Turn and Place of the first eye at each still, the second eye's on the first, the focal
      --  lengths' logarithms, and every point's inverse depth (the first eye's, then the second's).
      type State is record
         Turn, Place_At : Vec3_Array (1 .. S);
         X_Turn, X_Place : Vec3;
         Log_F1, Log_F2  : Real;
         Inverse         : Real_Access;
      end record;

      Sigma : Real_Access := new Real_Array (1 .. Natural'Max (1, Ns));   --  per sighting, floored below
      Depth_Of : array (1 .. Natural'Max (1, Ns)) of Positive := [others => 1];   --  per sighting, its inverse depth

      function First_Point (K : Positive) return Place is (First_Points (First_Points'First + K - 1));
      function Second_Point (K : Positive) return Place is (Second_Points (Second_Points'First + K - 1));
      function Sg (K : Positive) return Sighting is (Sightings (Sightings'First + K - 1));

      function Lens_1 (St : State) return Lens is
        ((Focal => Exp (St.Log_F1), Cx => First_Start.Cx, Cy => First_Start.Cy));
      function Lens_2 (St : State) return Lens is
        ((Focal => Exp (St.Log_F2), Cx => Second_Centre.U, Cy => Second_Centre.V));

      --  The eyes' motions at every still: the first's pose, and the second's in its own reference frame
      --  (X^-1 T X: the second eye rides the first by X).
      procedure Motions (St : State; First, Second : out Rigid_Array) is
         X  : constant Rigid := Pose (St.X_Turn, St.X_Place);
         Xi : constant Rigid := Inverse (X);
      begin
         for Q in 1 .. S loop
            First (Q) := Pose (St.Turn (Q), St.Place_At (Q));
            Second (Q) := Xi * First (Q) * X;
         end loop;
      end Motions;

      procedure Residual
        (St : State; First, Second : Rigid_Array; K : Positive; Du, Dv : out Real) is
         Z : constant Sighting := Sg (K);
      begin
         if Z.Eye = 1 then
            Miss (Lens_1 (St), First (Z.Still), Ray (Lens_1 (St), First_Point (Z.Point)),
                  St.Inverse (Depth_Of (K)), (Z.U, Z.V), Sigma (K), Du, Dv);
         else
            Miss (Lens_2 (St), Second (Z.Still), Ray (Lens_2 (St), Second_Point (Z.Point)),
                  St.Inverse (Depth_Of (K)), (Z.U, Z.V), Sigma (K), Du, Dv);
         end if;
      end Residual;

      procedure Evaluate (St : State; R : out Real_Array) is
         First, Second : Rigid_Array (1 .. S);
      begin
         Motions (St, First, Second);
         for K in 1 .. Ns loop
            Residual (St, First, Second, K, R (2 * K - 1), R (2 * K));
         end loop;
      end Evaluate;

      Scale : Real := 1.0;   --  the residuals' own (Scale_Of), at the adjustment's iteration's start

      function Cost (R : Real_Array) return Real is
         C : Real := 0.0;
      begin
         for V of R loop
            C := C + Huber_Cost (V / Scale);
         end loop;
         return C;
      end Cost;

      function Copy (St : State) return State is
        ((Turn => St.Turn, Place_At => St.Place_At, X_Turn => St.X_Turn, X_Place => St.X_Place, Log_F1 => St.Log_F1,
          Log_F2 => St.Log_F2, Inverse => new Real_Array'(St.Inverse.all)));

      procedure Release (St : in out State) is
      begin
         Free (St.Inverse);
      end Release;

      --  The scale is the pictures' own (the package's header): every place and every inverse depth together, the
      --  median of the first eye's placed points' inverse depths one. It changes no residual.
      procedure Gauge (St : in out State) is
         Count : Natural := 0;
         Them  : Real_Access := new Real_Array (1 .. Natural'Max (1, N1));
      begin
         for I in 1 .. N1 loop
            if St.Inverse (I) > 0.0 then
               Count := Count + 1;
               Them (Count) := St.Inverse (I);
            end if;
         end loop;
         if Count > 0 then
            declare
               C : constant Real := Driver.Stats.Median (Them (1 .. Count));
            begin
               if C > 0.0 then
                  for I in 1 .. Nd loop
                     St.Inverse (I) := St.Inverse (I) / C;
                  end loop;
                  for Q in 1 .. S loop
                     St.Place_At (Q) := C * St.Place_At (Q);
                  end loop;
                  St.X_Place := C * St.X_Place;
               end if;
            end;
         end if;
         Free (Them);
      end Gauge;

      --  The parameter P of the state (1 .. Np), and a state with it moved by H.
      function Get (St : State; P : Positive) return Real is
        (if P <= 6 * S
         then (if (P - 1) mod 6 < 3 then St.Turn ((P - 1) / 6 + 1) ((P - 1) mod 6 + 1)
               else St.Place_At ((P - 1) / 6 + 1) ((P - 1) mod 6 - 2))
         elsif P <= 6 * S + 3 then St.X_Turn (P - 6 * S)
         elsif P <= 6 * S + 6 then St.X_Place (P - 6 * S - 3)
         elsif P = 6 * S + 7 then St.Log_F1
         else St.Log_F2);

      procedure Set (St : in out State; P : Positive; Value : Real) is
      begin
         if P <= 6 * S then
            if (P - 1) mod 6 < 3 then
               St.Turn ((P - 1) / 6 + 1) ((P - 1) mod 6 + 1) := Value;
            else
               St.Place_At ((P - 1) / 6 + 1) ((P - 1) mod 6 - 2) := Value;
            end if;
         elsif P <= 6 * S + 3 then
            St.X_Turn (P - 6 * S) := Value;
         elsif P <= 6 * S + 6 then
            St.X_Place (P - 6 * S - 3) := Value;
         elsif P = 6 * S + 7 then
            St.Log_F1 := Value;
         else
            St.Log_F2 := Value;
         end if;
      end Set;

      --  Whether parameter P moves sighting K at all: a still's pose moves that still's sightings, the second eye's
      --  place and its focal length the second eye's, the first focal length the first eye's.
      function Moves (P : Positive; K : Positive) return Boolean is
        (if P <= 6 * S then Sg (K).Still = (P - 1) / 6 + 1
         elsif P <= 6 * S + 6 then Sg (K).Eye = 2
         elsif P = 6 * S + 7 then Sg (K).Eye = 1
         else Sg (K).Eye = 2);

      --  The bundle adjustment from St (Levenberg and Marquardt, each inverse depth taken out of the normal equations),
      --  to where a step lowers the cost by less than the unchanged fraction of it or no damping lowers it at all.
      --  Fixed_F2: the second focal length held (the turns' start, before the second eye's place is known).
      procedure Adjust (St : in out State; Final_Cost : out Real; Normal : out Real_Matrix; Squares : out Real;
                        Residuals : out Real_Array) is
         R0     : Real_Access := new Real_Array (1 .. 2 * Ns);
         Rn     : Real_Access := new Real_Array (1 .. 2 * Ns);
         Jp     : Matrix_Access := new Real_Matrix (1 .. 2 * Ns, 1 .. Np);
         Jd     : Real_Access := new Real_Array (1 .. 2 * Ns);
         Hpd    : Matrix_Access := new Real_Matrix (1 .. Np, 1 .. Natural'Max (1, Nd));
         Hdd    : Real_Access := new Real_Array (1 .. Natural'Max (1, Nd));
         Bd     : Real_Access := new Real_Array (1 .. Natural'Max (1, Nd));
         Lambda : Real := Driver.Conventions.Initial_Damping;
         Cost0  : Real;
         Hpp    : Real_Matrix (1 .. Np, 1 .. Np);
         Bp     : Real_Vector (1 .. Np);
      begin
         Evaluate (St, R0.all);
         loop
            Scale := Scale_Of (R0.all);
            Cost0 := Cost (R0.all);
            --  The Jacobian: each pose term's column over the sightings it moves; every inverse depth's at once
            --  (a sighting moves with its own point's alone).
            Jp.all := [others => [others => 0.0]];
            for P in 1 .. Np loop
               declare
                  Sp : State := Copy (St);
                  H  : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs Get (St, P));
                  First, Second : Rigid_Array (1 .. S);
               begin
                  Set (Sp, P, Get (St, P) + H);
                  Motions (Sp, First, Second);
                  for K in 1 .. Ns loop
                     if Moves (P, K) then
                        declare
                           Du, Dv : Real;
                        begin
                           Residual (Sp, First, Second, K, Du, Dv);
                           Jp (2 * K - 1, P) := (Du - R0 (2 * K - 1)) / H;
                           Jp (2 * K, P) := (Dv - R0 (2 * K)) / H;
                        end;
                     end if;
                  end loop;
                  Release (Sp);
               end;
            end loop;
            declare
               Sp : State := Copy (St);
               First, Second : Rigid_Array (1 .. S);
               H  : Real_Array (1 .. Natural'Max (1, Nd));
            begin
               for I in 1 .. Nd loop
                  H (I) := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs St.Inverse (I));
                  Sp.Inverse (I) := St.Inverse (I) + H (I);
               end loop;
               Motions (Sp, First, Second);
               for K in 1 .. Ns loop
                  declare
                     Du, Dv : Real;
                  begin
                     Residual (Sp, First, Second, K, Du, Dv);
                     Jd (2 * K - 1) := (Du - R0 (2 * K - 1)) / H (Depth_Of (K));
                     Jd (2 * K) := (Dv - R0 (2 * K)) / H (Depth_Of (K));
                  end;
               end loop;
               Release (Sp);
            end;
            --  The normal equations, Huber's weights at Z.
            Hpp := [others => [others => 0.0]];
            Bp := [others => 0.0];
            Hpd.all := [others => [others => 0.0]];
            Hdd.all := [others => 0.0];
            Bd.all := [others => 0.0];
            for I in 1 .. 2 * Ns loop
               declare
                  W : constant Real := Huber (R0 (I) / Scale);
                  D : constant Positive := Depth_Of ((I + 1) / 2);
               begin
                  for P in 1 .. Np loop
                     if Jp (I, P) /= 0.0 then
                        Bp (P) := Bp (P) + W * Jp (I, P) * R0 (I);
                        Hpd (P, D) := Hpd (P, D) + W * Jp (I, P) * Jd (I);
                        for Q in P .. Np loop
                           Hpp (P, Q) := Hpp (P, Q) + W * Jp (I, P) * Jp (I, Q);
                        end loop;
                     end if;
                  end loop;
                  Hdd (D) := Hdd (D) + W * Jd (I) ** 2;
                  Bd (D) := Bd (D) + W * Jd (I) * R0 (I);
               end;
            end loop;
            for P in 1 .. Np loop
               for Q in 1 .. P - 1 loop
                  Hpp (P, Q) := Hpp (Q, P);
               end loop;
            end loop;
            declare
               Improved, Lowered : Boolean := False;
            begin
               loop
                  declare
                     Sm    : Real_Matrix (1 .. Np, 1 .. Np) := Hpp;
                     Bs    : Real_Vector (1 .. Np) := Bp;
                     Lf    : Real_Matrix (1 .. Np, 1 .. Np);
                     Pd    : Boolean;
                     Moved : Boolean := False;
                  begin
                     for P in 1 .. Np loop
                        Sm (P, P) := Hpp (P, P) * (1.0 + Lambda) + Lambda * Real'Model_Small;
                     end loop;
                     for D in 1 .. Nd loop
                        declare
                           Hd : constant Real := Hdd (D) * (1.0 + Lambda) + Lambda * Real'Model_Small;
                        begin
                           for P in 1 .. Np loop
                              if Hpd (P, D) /= 0.0 then
                                 Bs (P) := Bs (P) - Hpd (P, D) * Bd (D) / Hd;
                                 for Q in 1 .. Np loop
                                    Sm (P, Q) := Sm (P, Q) - Hpd (P, D) * Hpd (Q, D) / Hd;
                                 end loop;
                              end if;
                           end loop;
                        end;
                     end loop;
                     Driver.Numerics.Dense.Cholesky (Sm, Lf, Pd);
                     if Pd then
                        declare
                           Dp : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, -Bs);
                           Sn : State := Copy (St);
                        begin
                           for P in 1 .. Np loop
                              Set (Sn, P, Get (St, P) + Dp (P));
                              Moved := Moved or else Get (Sn, P) /= Get (St, P);
                           end loop;
                           for D in 1 .. Nd loop
                              declare
                                 Hd   : constant Real := Hdd (D) * (1.0 + Lambda) + Lambda * Real'Model_Small;
                                 Step : Real := -Bd (D);
                              begin
                                 for P in 1 .. Np loop
                                    Step := Step - Hpd (P, D) * Dp (P);
                                 end loop;
                                 --  A point at no finite depth is the inverse depth nought, and none lies behind the
                                 --  reference eye that saw it: a step past nought puts it at nought.
                                 Sn.Inverse (D) := Real'Max (0.0, St.Inverse (D) + Step / Hd);
                                 Moved := Moved or else Sn.Inverse (D) /= St.Inverse (D);
                              end;
                           end loop;
                           if Moved then
                              Evaluate (Sn, Rn.all);
                              if Cost (Rn.all) < Cost0 then
                                 Improved := Cost0 - Cost (Rn.all) > Driver.Conventions.Unchanged_Fraction * Cost0;
                                 Gauge (Sn);
                                 Release (St);
                                 St := Sn;
                                 Evaluate (St, R0.all);
                                 Cost0 := Cost (R0.all);
                                 Lambda := Lambda / 2.0;
                                 Lowered := True;
                              else
                                 Release (Sn);
                              end if;
                           else
                              Release (Sn);
                           end if;
                        end;
                     else
                        Moved := True;
                     end if;
                     exit when Lowered or else not Moved or else Lambda >= Real'Last / 2.0;
                     Lambda := 2.0 * Lambda;
                  end;
               end loop;
               exit when not Improved;
            end;
         end loop;
         --  The normal matrix with the inverse depths taken out, undamped, at the end: the pose terms' information.
         Normal := Hpp;
         Squares := 0.0;
         for D in 1 .. Nd loop
            if Hdd (D) > 0.0 then
               for P in 1 .. Np loop
                  for Q in 1 .. Np loop
                     Normal (P, Q) := Normal (P, Q) - Hpd (P, D) * Hpd (Q, D) / Hdd (D);
                  end loop;
               end loop;
            end if;
         end loop;
         Scale := Scale_Of (R0.all);
         for I in 1 .. 2 * Ns loop
            Squares := Squares + Huber (R0 (I) / Scale) * R0 (I) ** 2;
         end loop;
         Residuals := R0.all;
         Final_Cost := Cost0;
         Free (R0);
         Free (Rn);
         Free (Jp);
         Free (Jd);
         Free (Hpd);
         Free (Hdd);
         Free (Bd);
      end Adjust;

      --  An eye's turn at one still, a turn alone (each point's inverse depth nought: a view whose points lie far),
      --  from its sightings at that still: Gauss and Newton with Huber's weight at Z, from no turn. The angle's own
      --  standard deviation in Angle_Sigma.
      procedure Turn_Alone
        (L : Lens; Eye : Positive; Still : Positive; Turn : out Vec3; Angle_Sigma : out Real; Count : out Natural;
         Final : out Real)
      is
         W0     : Vec3 := Zero3;
         Lambda : Real := Driver.Conventions.Initial_Damping;
         function Point_Of (K : Positive) return Place is
           (if Sg (K).Eye = 1 then First_Point (Sg (K).Point) else Second_Point (Sg (K).Point));
         function Cost_At (W : Vec3) return Real is
            C : Real := 0.0;
            Du, Dv : Real;
         begin
            for K in 1 .. Ns loop
               if Sg (K).Eye = Eye and then Sg (K).Still = Still then
                  Miss (L, Pose (W, Zero3), Ray (L, Point_Of (K)), 0.0, (Sg (K).U, Sg (K).V), Sigma (K), Du, Dv);
                  C := C + Huber_Cost (Du) + Huber_Cost (Dv);
               end if;
            end loop;
            return C;
         end Cost_At;
         C0 : Real;
         A  : Mat3;
         G  : Vec3;
      begin
         Count := 0;
         for K in 1 .. Ns loop
            if Sg (K).Eye = Eye and then Sg (K).Still = Still then
               Count := Count + 1;
            end if;
         end loop;
         Turn := Zero3;
         Angle_Sigma := Real'Last;
         Final := 0.0;
         if Count < 3 then
            return;
         end if;
         C0 := Cost_At (W0);
         loop
            A := [others => [others => 0.0]];
            G := Zero3;
            for K in 1 .. Ns loop
               if Sg (K).Eye = Eye and then Sg (K).Still = Still then
                  declare
                     Du, Dv : Real;
                     Ju, Jv : Vec3;
                  begin
                     Miss (L, Pose (W0, Zero3), Ray (L, Point_Of (K)), 0.0, (Sg (K).U, Sg (K).V), Sigma (K), Du, Dv);
                     for P in 1 .. 3 loop
                        declare
                           Wp : Vec3 := W0;
                           H  : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs W0 (P));
                           Eu, Ev : Real;
                        begin
                           Wp (P) := Wp (P) + H;
                           Miss (L, Pose (Wp, Zero3), Ray (L, Point_Of (K)), 0.0, (Sg (K).U, Sg (K).V), Sigma (K),
                                 Eu, Ev);
                           Ju (P) := (Eu - Du) / H;
                           Jv (P) := (Ev - Dv) / H;
                        end;
                     end loop;
                     for P in 1 .. 3 loop
                        G (P) := G (P) - Huber (Du) * Ju (P) * Du - Huber (Dv) * Jv (P) * Dv;
                        for Q in 1 .. 3 loop
                           A (P, Q) := A (P, Q) + Huber (Du) * Ju (P) * Ju (Q) + Huber (Dv) * Jv (P) * Jv (Q);
                        end loop;
                     end loop;
                  end;
               end if;
            end loop;
            declare
               Improved, Lowered : Boolean := False;
            begin
               loop
                  declare
                     D : Real_Matrix (1 .. 3, 1 .. 3);
                     Lf : Real_Matrix (1 .. 3, 1 .. 3);
                     Pd : Boolean;
                     Moved : Boolean := False;
                  begin
                     for P in 1 .. 3 loop
                        for Q in 1 .. 3 loop
                           D (P, Q) := A (P, Q);
                        end loop;
                        D (P, P) := A (P, P) * (1.0 + Lambda) + Lambda * Real'Model_Small;
                     end loop;
                     Driver.Numerics.Dense.Cholesky (D, Lf, Pd);
                     if Pd then
                        declare
                           Step : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, Real_Vector (G));
                           Wn   : constant Vec3 := W0 + Step;
                        begin
                           Moved := Wn /= W0;
                           if Moved and then Cost_At (Wn) < C0 then
                              Improved := C0 - Cost_At (Wn) > Driver.Conventions.Unchanged_Fraction * C0;
                              W0 := Wn;
                              C0 := Cost_At (Wn);
                              Lambda := Lambda / 2.0;
                              Lowered := True;
                           end if;
                        end;
                     else
                        Moved := True;
                     end if;
                     exit when Lowered or else not Moved or else Lambda >= Real'Last / 2.0;
                     Lambda := 2.0 * Lambda;
                  end;
               end loop;
               exit when not Improved;
            end;
         end loop;
         Turn := W0;
         Final := C0;
         --  The angle's sigma: along the turn's own axis, from the normal matrix (unit weight: the residuals are in
         --  their sigmas).
         declare
            Ax : constant Vec3 := (if Norm (W0) > 0.0 then Unit (W0) else [1.0, 0.0, 0.0]);
            Lf : Real_Matrix (1 .. 3, 1 .. 3);
            Pd : Boolean;
            D  : Real_Matrix (1 .. 3, 1 .. 3);
         begin
            for P in 1 .. 3 loop
               for Q in 1 .. 3 loop
                  D (P, Q) := A (P, Q);
               end loop;
            end loop;
            Driver.Numerics.Dense.Cholesky (D, Lf, Pd);
            if Pd then
               declare
                  E : constant Real_Vector := Driver.Numerics.Dense.Cholesky_Solve (Lf, Real_Vector (Ax));
               begin
                  Angle_Sigma := Sqrt (Real'Max (0.0, Dot (Ax, Vec3 (E))));
               end;
            end if;
         end;
      end Turn_Alone;

      --  An eye's direction of motion at one still given its turn and lens: the line every point's two rays span
      --  holds it (the plane of the reference ray and the turned ray of its sighting), the least eigenvector of the
      --  sum of those planes' normals' outer products, each over its sigma; its sense the one that puts most points
      --  ahead. Its length such that the median inverse depth of the points it places is one.
      procedure Direction
        (L : Lens; Eye : Positive; Still : Positive; Turn : Vec3; Step : out Vec3; Count : out Natural)
      is
         Rm : constant Mat3 := Exp (Turn);
         M  : Mat3 := [others => [others => 0.0]];
         function Point_Of (K : Positive) return Place is
           (if Sg (K).Eye = 1 then First_Point (Sg (K).Point) else Second_Point (Sg (K).Point));
         Values  : Vec3;
         Vectors : Mat3;
      begin
         Count := 0;
         Step := Zero3;
         for K in 1 .. Ns loop
            if Sg (K).Eye = Eye and then Sg (K).Still = Still then
               declare
                  R  : constant Vec3 := Ray (L, Point_Of (K));
                  Qr : constant Vec3 := Times (Rm, Ray (L, (Sg (K).U, Sg (K).V)));
                  Nv : constant Vec3 := Cross (R, Qr);
               begin
                  if Norm (Nv) > 0.0 then
                     M := M + Outer (Nv, Nv) / (Sigma (K) ** 2 * Dot (Nv, Nv));
                     Count := Count + 1;
                  end if;
               end;
            end if;
         end loop;
         if Count < 3 then
            Count := 0;
            return;
         end if;
         Symmetric_Eigensystem (M, Values, Vectors);
         declare
            T      : Vec3 := [Vectors (1, 3), Vectors (2, 3), Vectors (3, 3)];
            Inv    : Real_Access := new Real_Array (1 .. Count);
            Placed : Natural := 0;
            Ahead  : Integer := 0;
         begin
            --  Each point's inverse depth along its reference ray, for a step of length one: P = z r and P - t lies
            --  along the turned ray q, so (t x q) . (r x q) = z |r x q|^2.
            for Pass in 1 .. 2 loop
               Placed := 0;
               Ahead := 0;
               for K in 1 .. Ns loop
                  if Sg (K).Eye = Eye and then Sg (K).Still = Still then
                     declare
                        R  : constant Vec3 := Ray (L, Point_Of (K));
                        Qr : constant Vec3 := Times (Rm, Ray (L, (Sg (K).U, Sg (K).V)));
                        Rq : constant Vec3 := Cross (R, Qr);
                        Zd : constant Real :=
                          (if Dot (Rq, Rq) > 0.0 then Dot (Cross (T, Qr), Rq) / Dot (Rq, Rq) else 0.0);
                     begin
                        Ahead := Ahead + (if Zd > 0.0 then 1 elsif Zd < 0.0 then -1 else 0);
                        if Zd > 0.0 then
                           Placed := Placed + 1;
                           Inv (Placed) := 1.0 / Zd;
                        end if;
                     end;
                  end if;
               end loop;
               exit when Ahead >= 0;
               T := -T;
            end loop;
            if Placed > 0 then
               Step := Driver.Stats.Median (Inv (1 .. Placed)) * T;
            end if;
            Free (Inv);
         end;
      end Direction;

      --  Every point's inverse depth given the motions, alone: Gauss and Newton on the one number, from the median of
      --  its eye's (one), over its own sightings; nought where none places it ahead.
      procedure Place_Points (St : in out State) is
         First, Second : Rigid_Array (1 .. S);
      begin
         Motions (St, First, Second);
         for D in 1 .. Nd loop
            declare
               Rho : Real := 1.0;
               Count : Natural := 0;
            begin
               for K in 1 .. Ns loop
                  if Depth_Of (K) = D then
                     Count := Count + 1;
                  end if;
               end loop;
               if Count > 0 then
                  for Iteration in 1 .. Real'Machine_Mantissa loop
                     declare
                        A, G : Real := 0.0;
                        Before : constant Real := Rho;
                     begin
                        for K in 1 .. Ns loop
                           if Depth_Of (K) = D then
                              declare
                                 Du, Dv, Eu, Ev : Real;
                                 H : constant Real := Sqrt (Real'Model_Epsilon) * Real'Max (1.0, abs Rho);
                                 Save : constant Real := St.Inverse (D);
                              begin
                                 St.Inverse (D) := Rho;
                                 Residual (St, First, Second, K, Du, Dv);
                                 St.Inverse (D) := Rho + H;
                                 Residual (St, First, Second, K, Eu, Ev);
                                 St.Inverse (D) := Save;
                                 A := A + Huber (Du) * ((Eu - Du) / H) ** 2 + Huber (Dv) * ((Ev - Dv) / H) ** 2;
                                 G := G - Huber (Du) * (Eu - Du) / H * Du - Huber (Dv) * (Ev - Dv) / H * Dv;
                              end;
                           end if;
                        end loop;
                        exit when A <= 0.0;
                        Rho := Real'Max (0.0, Rho + G / A);
                        exit when Rho = Before;
                     end;
                  end loop;
                  St.Inverse (D) := Rho;
               end if;
            end;
         end loop;
      end Place_Points;

      Best      : State;
      Best_Normal : Real_Matrix (1 .. Np, 1 .. Np) := [others => [others => 0.0]];
      Best_Squares : Real := 0.0;
      Best_Residuals : Real_Access := new Real_Array (1 .. 2 * Ns);
      Have_Best : Boolean := False;
   begin
      Result.Ok := False;
      Result.Cost := 0.0;
      if Ns = 0 then
         Free (Sigma);
         Free (Best_Residuals);
         return;
      end if;
      --  The sigmas: as the aligner stated them, a sighting that states none at the median of those that do.
      declare
         Stated : Real_Access := new Real_Array (1 .. Ns);
         Count  : Natural := 0;
         Middle : Real := 0.0;
      begin
         for K in 1 .. Ns loop
            if Sg (K).Sigma > 0.0 then
               Count := Count + 1;
               Stated (Count) := Sg (K).Sigma;
            end if;
            Depth_Of (K) := (if Sg (K).Eye = 1 then Sg (K).Point else N1 + Sg (K).Point);
         end loop;
         if Count > 0 then
            Middle := Driver.Stats.Median (Stated (1 .. Count));
         end if;
         for K in 1 .. Ns loop
            Sigma (K) := (if Sg (K).Sigma > 0.0 then Sg (K).Sigma else Middle);
         end loop;
         Free (Stated);
         if Middle <= 0.0 then
            Free (Sigma);
            Free (Best_Residuals);
            return;
         end if;
      end;
      --  The starts.
      declare
         --  The first focal length where none is given: the one at which the first eye's turn alone explains its
         --  sightings at the still it turned most at best (a turn bends its points' paths by the focal length, the more
         --  the farther from the centre), searched as the arm's fit searches its own (Kinematics.Fit's stage 1): from
         --  half the picture's width (seen at half a right angle), an octave about it, the bracket widened by its own
         --  width on the side whose end costs less until an inner point costs less than both ends, then narrowed by
         --  golden sections until it is the unchanged fraction wide in the logarithm. Nought where no still's turn is
         --  told, or no bracket closes before the float's end: nothing tells the focal length.
         function First_Focal return Real is
            Golden : constant Real := (Sqrt (5.0) - 1.0) / 2.0;
            Start  : constant Real := First_Start.Cx;
            Most   : Real := 0.0;
            Which  : Natural := 0;
            function Cost_At (Ln_F : Real) return Real is
               T     : Vec3;
               A_Sig : Real;
               Cnt   : Natural;
               Spent : Real;
            begin
               Turn_Alone ((Focal => Exp (Ln_F), Cx => First_Start.Cx, Cy => First_Start.Cy), 1, Which, T, A_Sig, Cnt,
                           Spent);
               return Spent;
            end Cost_At;
            A, B, C : Real;   --  logarithms: A < C < B, the cost at C below both ends' once bracketed
            Fa, Fb, Fc : Real;
         begin
            if First_Start.Focal > 0.0 then
               return First_Start.Focal;
            end if;
            for Q in 1 .. S loop
               declare
                  T     : Vec3;
                  A_Sig : Real;
                  Cnt   : Natural;
                  Spent : Real;
               begin
                  Turn_Alone ((Focal => Start, Cx => First_Start.Cx, Cy => First_Start.Cy), 1, Q, T, A_Sig, Cnt, Spent);
                  if A_Sig < Real'Last and then Norm (T) > Driver.Conventions.Z * A_Sig and then Norm (T) > Most then
                     Most := Norm (T);
                     Which := Q;
                  end if;
               end;
            end loop;
            if Which = 0 then
               return 0.0;
            end if;
            A := Log (Start) - Log (2.0) / 2.0;
            B := Log (Start) + Log (2.0) / 2.0;
            Fa := Cost_At (A);
            Fb := Cost_At (B);
            loop
               C := B - Golden * (B - A);
               Fc := Cost_At (C);
               exit when Fc <= Fa and then Fc <= Fb;
               if B - A >= Log (Real'Last) / 2.0 then
                  return 0.0;
               end if;
               if Fa <= Fb then
                  A := A - (B - A);
                  Fa := Cost_At (A);
               else
                  B := B + (B - A);
                  Fb := Cost_At (B);
               end if;
            end loop;
            while B - A > Driver.Conventions.Unchanged_Fraction loop
               declare
                  D  : constant Real :=
                    (if B - C > C - A then C + (1.0 - Golden) * (B - C) else C - (1.0 - Golden) * (C - A));
                  Fd : constant Real := Cost_At (D);
               begin
                  if Fd < Fc then
                     if D > C then
                        A := C;
                     else
                        B := C;
                     end if;
                     C := D;
                     Fc := Fd;
                  elsif D > C then
                     B := D;
                  else
                     A := D;
                  end if;
               end;
            end loop;
            return Exp (C);
         end First_Focal;
         Focal_1 : constant Real := First_Focal;
         L1 : constant Lens := (Focal => Focal_1, Cx => First_Start.Cx, Cy => First_Start.Cy);
         Spent : Real;
         Turn1, Turn2 : Vec3_Array (1 .. S) := [others => Zero3];
         Sig1, Sig2   : Real_Array (1 .. S) := [others => Real'Last];
         F2 : Real := Focal_1;   --  the second focal length, from the first's
         Step1, Step2 : Vec3_Array (1 .. S) := [others => Zero3];
         Rx : Mat3 := Identity3;
         Tx : Vec3 := Zero3;
         Count : Natural;
         function L2 return Lens is ((Focal => F2, Cx => Second_Centre.U, Cy => Second_Centre.V));
         --  A turn the sightings tell: its angle beyond Z of its own sigma.
         function Told (Turn : Vec3; Sigma_Of : Real) return Boolean is
           (Sigma_Of < Real'Last and then Norm (Turn) > Driver.Conventions.Z * Sigma_Of);
      begin
         if Focal_1 <= 0.0 then
            Free (Sigma);
            Free (Best_Residuals);
            return;
         end if;
         for Q in 1 .. S loop
            Turn_Alone (L1, 1, Q, Turn1 (Q), Sig1 (Q), Count, Spent);
         end loop;
         --  The second eye's focal length: the one that turns it as far as the first eye turned at the stills both
         --  tell a turn at (a view's turn in pixels is its angle times the focal length, near the centre): the angle
         --  its turn alone takes at one focal length, scaled, again from there, as long as it comes nearer.
         for Pass in 1 .. Real'Machine_Mantissa loop
            for Q in 1 .. S loop
               Turn_Alone (L2, 2, Q, Turn2 (Q), Sig2 (Q), Count, Spent);
            end loop;
            declare
               Num, Den : Real := 0.0;
            begin
               for Q in 1 .. S loop
                  if Told (Turn1 (Q), Sig1 (Q)) and then Told (Turn2 (Q), Sig2 (Q)) then
                     Num := Num + Norm (Turn1 (Q)) * Norm (Turn2 (Q));
                     Den := Den + Norm (Turn1 (Q)) ** 2;
                  end if;
               end loop;
               exit when Num <= 0.0 or else Den <= 0.0;
               declare
                  Ratio : constant Real := Num / Den;   --  the second eye's angles over the first's
                  Was   : constant Real := F2;
               begin
                  F2 := F2 * Ratio;
                  exit when abs Log (Ratio) <= Driver.Conventions.Unchanged_Fraction or else F2 = Was;
               end;
            end;
         end loop;
         for Q in 1 .. S loop
            Turn_Alone (L2, 2, Q, Turn2 (Q), Sig2 (Q), Count, Spent);
            Direction (L1, 1, Q, Turn1 (Q), Step1 (Q), Count);
            Direction (L2, 2, Q, Turn2 (Q), Step2 (Q), Count);
         end loop;
         --  The second eye's frame on the first from every still's motion of both eyes, linear (Andreff, Horaud and
         --  Espiau, 2001): the first eye moved by (R1, t1) carries the second by X^-1 (R1, t1) X, so R1 Y = Y R2 and
         --  (R1 - I) Tx - Y t2 = -t1, where Y is the second eye's turn on the first times the ratio of the two eyes'
         --  scales (each eye's steps are in its own: Direction), linear in Y's nine entries and Tx: least squares over
         --  every still, the turn the rotation nearest to Y (Kabsch's). The steps tell what turns about one axis
         --  alone leave free (a body that only turns about the vertical and steps on the floor). None where the
         --  stills do not determine it.
         declare
            Rows : constant Natural := 12 * S;
            Am   : Real_Matrix (1 .. Rows, 1 .. 12) := [others => [others => 0.0]];
            Bv   : Real_Vector (1 .. Rows) := [others => 0.0];
            Xv   : Real_Vector (1 .. 12);
            Full : Boolean;
            --  The entry of Y at row I and column J, column by column: Y's columns stacked.
            function At_Y (I, J : Positive) return Positive is ((J - 1) * 3 + I);
         begin
            for Q in 1 .. S loop
               declare
                  R1 : constant Mat3 := Exp (Turn1 (Q));
                  R2 : constant Mat3 := Exp (Turn2 (Q));
                  Base : constant Natural := 12 * (Q - 1);
               begin
                  --  R1 Y - Y R2 = 0, entry (I, J): sum over K of R1 (I, K) Y (K, J) - Y (I, K) R2 (K, J).
                  for I in 1 .. 3 loop
                     for J in 1 .. 3 loop
                        for K in 1 .. 3 loop
                           Am (Base + At_Y (I, J), At_Y (K, J)) := Am (Base + At_Y (I, J), At_Y (K, J)) + R1 (I, K);
                           Am (Base + At_Y (I, J), At_Y (I, K)) := Am (Base + At_Y (I, J), At_Y (I, K)) - R2 (K, J);
                        end loop;
                     end loop;
                  end loop;
                  --  (R1 - I) Tx - Y t2 = -t1, entry I.
                  for I in 1 .. 3 loop
                     for C in 1 .. 3 loop
                        Am (Base + 9 + I, 9 + C) := R1 (I, C) - (if I = C then 1.0 else 0.0);
                        Am (Base + 9 + I, At_Y (I, C)) := -Step2 (Q) (C);
                     end loop;
                     Bv (Base + 9 + I) := -Step1 (Q) (I);
                  end loop;
               end;
            end loop;
            Driver.Numerics.Dense.Least_Squares (Am, Bv, Xv, Full);
            if Full then
               declare
                  Y : constant Mat3 := [for I in 1 .. 3 => [for J in 1 .. 3 => Xv (At_Y (I, J))]];
               begin
                  Rx := Orthonormalize (Y);
                  Tx := [Xv (10), Xv (11), Xv (12)];
               end;
            else
               Free (Sigma);
               Free (Best_Residuals);
               return;
            end if;
         end;
         --  The adjustment from there.
         declare
            St    : State;
            Fc    : Real;
            Nm    : Real_Matrix (1 .. Np, 1 .. Np);
            Sq    : Real;
         begin
            St.Turn := Turn1;
            St.Place_At := Step1;
            St.X_Turn := Log (Rx);
            St.X_Place := Tx;
            St.Log_F1 := Log (L1.Focal);
            St.Log_F2 := Log (F2);
            St.Inverse := new Real_Array'(1 .. Natural'Max (1, Nd) => 1.0);
            Place_Points (St);
            Gauge (St);
            Adjust (St, Fc, Nm, Sq, Best_Residuals.all);
            Result.Cost := Fc;
            Best := St;
            Best_Normal := Nm;
            Best_Squares := Sq;
            Have_Best := True;
         end;
      end;
      if Have_Best then
         declare
            First, Second : Rigid_Array (1 .. S);
            Larger : Real_Access := new Real_Array (1 .. Ns);
            Kept   : Natural := 0;
            Final_Scale : constant Real := Scale_Of (Best_Residuals.all);
         begin
            Motions (Best, First, Second);
            Result.Second_On_First := Pose (Best.X_Turn, Best.X_Place);
            Result.First_Lens := Lens_1 (Best);
            Result.Second_Lens := Lens_2 (Best);
            Result.Poses := First;
            Result.First_Placed := [others => 0];
            Result.Second_Placed := [others => 0];
            for K in 1 .. Ns loop
               Larger (K) := Real'Max (abs Best_Residuals (2 * K - 1), abs Best_Residuals (2 * K));
               if Larger (K) <= Driver.Conventions.Z * Final_Scale then
                  Kept := Kept + 1;
                  if Sg (K).Eye = 1 then
                     Result.First_Placed (Sg (K).Point) := Result.First_Placed (Sg (K).Point) + 1;
                  else
                     Result.Second_Placed (Sg (K).Point) := Result.Second_Placed (Sg (K).Point) + 1;
                  end if;
               end if;
            end loop;
            for I in 1 .. N1 loop
               Result.First_Inverse (I) := (if Result.First_Placed (I) > 0 then Best.Inverse (I) else 0.0);
            end loop;
            for J in 1 .. N2 loop
               Result.Second_Inverse (J) := (if Result.Second_Placed (J) > 0 then Best.Inverse (N1 + J) else 0.0);
            end loop;
            Result.Kept := Kept;
            Result.Used := Ns;
            Result.Misfit := Driver.Stats.Median (Larger (1 .. Ns));
            Result.Scale := Final_Scale;
            --  The second eye's turn's covariance: the pose terms' normal matrix (the inverse depths taken out)
            --  inverted on the subspace the sightings determine (its eigenvalues above the float's resolution of
            --  the largest; the scale, which no residual moves, is not), the turn's block, times the residuals'
            --  weighted squares over their degrees of freedom.
            declare
               Values  : Real_Vector (1 .. Np);
               Vectors : Real_Matrix (1 .. Np, 1 .. Np);
               Cov     : Mat3 := [others => [others => 0.0]];
               Dof     : constant Integer := 2 * Ns - Np - Nd;
               Largest : Real := 0.0;
               V3      : Vec3;
               M3      : Mat3;
            begin
               Driver.Numerics.Dense.Small_Eigensystem (Best_Normal, Values, Vectors);
               for P in 1 .. Np loop
                  Largest := Real'Max (Largest, Values (P));
               end loop;
               for P in 1 .. Np loop
                  if Values (P) > Real'Model_Epsilon * Largest * Real (Np) then
                     for A in 1 .. 3 loop
                        for B in 1 .. 3 loop
                           Cov (A, B) := Cov (A, B) + Vectors (6 * S + A, P) * Vectors (6 * S + B, P) / Values (P);
                        end loop;
                     end loop;
                  end if;
               end loop;
               if Dof > 0 then
                  Cov := (Best_Squares / Real (Dof)) * Cov;
               end if;
               Symmetric_Eigensystem (Cov, V3, M3);
               Result.Turn_Sigma := Sqrt (Real'Max (0.0, V3 (1)));
            end;
            Result.Ok := Result.Misfit <= Driver.Conventions.Z and then Result.Turn_Sigma < Real'Last;
            Free (Larger);
         end;
         Release (Best);
      end if;
      Free (Sigma);
      Free (Best_Residuals);
   end Calibrate;

end Driver.Rig;
