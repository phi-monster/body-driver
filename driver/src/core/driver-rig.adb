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
   type Flag_Array is array (Positive range <>) of Boolean;

   function Norm (V : Vec3) return Real is (Sqrt (Dot (V, V)));

   --  A number, and not past the float's end (a step from degenerate stills, a covariance of a term nothing tells):
   --  what the standard eigensystem refuses as not symmetric and a unit vector's precondition as of no length.
   function Finite (X : Real) return Boolean is (abs X <= Real'Last);

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
            Result.Ok := Pd and then Within >= 6 and then (for all V of X => Finite (V));
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
      Inverse_Depth : out Real; Variance : out Real; Ok : out Boolean)
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
      Variance := Real'Last;
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
      --  The one number's variance: the residuals are in their sigmas, widened by their own scale (Huber's weights
      --  at the last step's place).
      if Ok then
         Variance := Scale ** 2 / A;
      end if;
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
      --  Per sighting: its eye's stills adjusted alone could not place its still (Refine_Eye), and it is in no
      --  adjustment: its residuals nought.
      Dropped : array (1 .. Natural'Max (1, Ns)) of Boolean := [others => False];

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
         if Dropped (K) then
            Du := 0.0;
            Dv := 0.0;
         elsif Z.Eye = 1 then
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

      --  The residuals' own scale (Scale_Of) over the sightings still in the adjustment (not Dropped).
      function Live_Scale (R : Real_Array) return Real is
         Count : Natural := 0;
      begin
         for K in 1 .. Ns loop
            if not Dropped (K) then
               Count := Count + 2;
            end if;
         end loop;
         declare
            A : Real_Array (1 .. Count);
            J : Natural := 0;
         begin
            for K in 1 .. Ns loop
               if not Dropped (K) then
                  A (J + 1) := R (R'First + 2 * K - 2);
                  A (J + 2) := R (R'First + 2 * K - 1);
                  J := J + 2;
               end if;
            end loop;
            return Scale_Of (A);
         end;
      end Live_Scale;

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
            Scale := Live_Scale (R0.all);
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
         Scale := Live_Scale (R0.all);
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
         if Count < 3 or else (for some I in 1 .. 3 => (for some J in 1 .. 3 => not Finite (M (I, J)))) then
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

      --  One eye's stills adjusted alone, its lens held: every point of the eye placed from the stills' poses
      --  (Triangulate), then every still's pose from the points (Locate), in turn, the scale such that the placed
      --  points' median inverse depth is one, until no still's pose moves by more than the unchanged fraction of how
      --  far it lies from the reference. A still whose sightings of the eye do not place it (Locate not Ok) is that
      --  eye's no more (Live false); with Apply its sightings of the eye are dropped from the adjustments (Dropped).
      --  Cost: the Huber cost of every sighting of the eye in its sigmas at the end, a point placed nowhere taken at
      --  no finite depth and a still left out at its last pose. The poses begin from a turn alone and the direction
      --  given it (Turn_Alone, Direction), which hold for far points: a step of the G1's walk, a tenth of a metre
      --  with the table half a metre off, bent a turn alone by degrees, and the eyes' frames on each other begun from
      --  such turns went wrong by 100 to 180 degrees from matches the truth had right (walkdump.py, seeds 1 and 3,
      --  10-27).
      procedure Refine_Eye
        (Eye : Positive; L : Lens; Turns, Steps : in out Vec3_Array; Live : out Flag_Array; Apply : Boolean;
         Cost : out Real)
      is
         Npts : constant Natural := (if Eye = 1 then N1 else N2);
         Inv  : Real_Access := new Real_Array (1 .. Natural'Max (1, Npts));
         Have : array (1 .. Natural'Max (1, Npts)) of Boolean := [others => False];
         --  The eye's sightings by point and by still: their indices, each list's start in its order.
         Mine_Count : Natural := 0;
         function Point_Of (P : Positive) return Place is (if Eye = 1 then First_Point (P) else Second_Point (P));
         function Mine (K : Positive) return Boolean is (Sg (K).Eye = Eye and then not Dropped (K));
      begin
         Live := [others => True];
         Inv.all := [others => 1.0];
         for K in 1 .. Ns loop
            if Mine (K) then
               Mine_Count := Mine_Count + 1;
            end if;
         end loop;
         declare
            By_Point, By_Still : Driver.Natural_Array (1 .. Natural'Max (1, Mine_Count));
            Point_From : Driver.Natural_Array (1 .. Npts + 1) := [others => 0];
            Still_From : Driver.Natural_Array (1 .. S + 1) := [others => 0];
            Fill       : Natural;
         begin
            --  Counting sorts: the sightings of point P are By_Point (Point_From (P) .. Point_From (P + 1) - 1).
            for K in 1 .. Ns loop
               if Mine (K) then
                  Point_From (Sg (K).Point + 1) := Point_From (Sg (K).Point + 1) + 1;
                  Still_From (Sg (K).Still + 1) := Still_From (Sg (K).Still + 1) + 1;
               end if;
            end loop;
            Point_From (1) := 1;
            for P in 2 .. Npts + 1 loop
               Point_From (P) := Point_From (P) + Point_From (P - 1);
            end loop;
            Still_From (1) := 1;
            for Q in 2 .. S + 1 loop
               Still_From (Q) := Still_From (Q) + Still_From (Q - 1);
            end loop;
            declare
               Next_P : Driver.Natural_Array := Point_From;
               Next_Q : Driver.Natural_Array := Still_From;
            begin
               for K in 1 .. Ns loop
                  if Mine (K) then
                     By_Point (Next_P (Sg (K).Point)) := K;
                     Next_P (Sg (K).Point) := Next_P (Sg (K).Point) + 1;
                     By_Still (Next_Q (Sg (K).Still)) := K;
                     Next_Q (Sg (K).Still) := Next_Q (Sg (K).Still) + 1;
                  end if;
               end loop;
            end;
            --  Rounds: the poses and points adjusted in turn until they hold, then every still judged as the whole
            --  adjustment is (its median sighting within Z of its sigma, Locate's own scale widening with what its
            --  sightings leave: a still whose matches lay 14 px off the truth in the median, the first of the G1's
            --  longest walk, was placed with its step the wrong way round); a round again without the stills judged
            --  out, until none is. Judged before the poses hold, every still of the walks was out by its first
            --  points' depths.
            for Round in 1 .. S loop
               for Iteration in 1 .. Real'Machine_Mantissa loop
                  --  The points from the poses.
                  for P in 1 .. Npts loop
                     Fill := 0;
                     for J in Point_From (P) .. Point_From (P + 1) - 1 loop
                        if Live (Sg (By_Point (J)).Still) then
                           Fill := Fill + 1;
                        end if;
                     end loop;
                     declare
                        Poses   : Rigid_Array (1 .. Fill);
                        Seen_At : Place_Array (1 .. Fill);
                        Sigmas  : Real_Array (1 .. Fill);
                        C       : Natural := 0;
                        Var     : Real;
                        Ok      : Boolean := False;
                     begin
                        for J in Point_From (P) .. Point_From (P + 1) - 1 loop
                           declare
                              K : constant Positive := By_Point (J);
                           begin
                              if Live (Sg (K).Still) then
                                 C := C + 1;
                                 Poses (C) := Pose (Turns (Sg (K).Still), Steps (Sg (K).Still));
                                 Seen_At (C) := (Sg (K).U, Sg (K).V);
                                 Sigmas (C) := Sigma (K);
                              end if;
                           end;
                        end loop;
                        if Fill > 0 then
                           Triangulate (L, Ray (L, Point_Of (P)), Poses, Seen_At, Sigmas,
                                        (if Have (P) then Inv (P) else 1.0), Inv (P), Var, Ok);
                        end if;
                        Have (P) := Ok and then Inv (P) > 0.0;
                     end;
                  end loop;
                  --  The scale: the placed points' median inverse depth one.
                  declare
                     Them  : Real_Access := new Real_Array (1 .. Natural'Max (1, Npts));
                     Count : Natural := 0;
                  begin
                     for P in 1 .. Npts loop
                        if Have (P) then
                           Count := Count + 1;
                           Them (Count) := Inv (P);
                        end if;
                     end loop;
                     if Count > 0 then
                        declare
                           C : constant Real := Driver.Stats.Median (Them (1 .. Count));
                        begin
                           if C > 0.0 then
                              for P in 1 .. Npts loop
                                 Inv (P) := Inv (P) / C;
                              end loop;
                              for Q in 1 .. S loop
                                 Steps (Q) := C * Steps (Q);
                              end loop;
                           end if;
                        end;
                     end if;
                     Free (Them);
                  end;
                  --  The poses from the points.
                  declare
                     Moved : Boolean := False;
                  begin
                     for Q in 1 .. S loop
                        if Live (Q) then
                           Fill := 0;
                           for J in Still_From (Q) .. Still_From (Q + 1) - 1 loop
                              if Have (Sg (By_Still (J)).Point) then
                                 Fill := Fill + 1;
                              end if;
                           end loop;
                           declare
                              Rays    : Real_Matrix (1 .. Fill, 1 .. 4);
                              Seen_At : Place_Array (1 .. Fill);
                              Sigmas  : Real_Array (1 .. Fill);
                              C       : Natural := 0;
                              Found   : Locate_Result;
                           begin
                              for J in Still_From (Q) .. Still_From (Q + 1) - 1 loop
                                 declare
                                    K : constant Positive := By_Still (J);
                                 begin
                                    if Have (Sg (K).Point) then
                                       C := C + 1;
                                       declare
                                          Rr : constant Vec3 := Ray (L, Point_Of (Sg (K).Point));
                                       begin
                                          Rays (C, 1) := Rr (1);
                                          Rays (C, 2) := Rr (2);
                                          Rays (C, 3) := Rr (3);
                                          Rays (C, 4) := Inv (Sg (K).Point);
                                       end;
                                       Seen_At (C) := (Sg (K).U, Sg (K).V);
                                       Sigmas (C) := Sigma (K);
                                    end if;
                                 end;
                              end loop;
                              if Fill > 0 then
                                 Locate (L, Rays, Seen_At, Sigmas, Pose (Turns (Q), Steps (Q)), Found);
                              end if;
                              if Found.Ok then
                                 declare
                                    T  : constant Vec3 := Log (Found.Pose.Rotation);
                                    D  : constant Real :=
                                      Norm (Minus (T, Turns (Q))) + Norm (Minus (Found.Pose.Translation, Steps (Q)));
                                    Sz : constant Real := Real'Max (Norm (T) + Norm (Found.Pose.Translation),
                                                                    Real'Model_Small);
                                 begin
                                    if D > Driver.Conventions.Unchanged_Fraction * Sz then
                                       Moved := True;
                                    end if;
                                    Turns (Q) := T;
                                    Steps (Q) := Found.Pose.Translation;
                                 end;
                              else
                                 Live (Q) := False;
                                 Moved := True;
                              end if;
                           end;
                        end if;
                     end loop;
                     exit when not Moved;
                  end;
               end loop;
               declare
                  Judged_Out : Boolean := False;
               begin
                  for Q in 1 .. S loop
                     if Live (Q) then
                        declare
                           Larger : Real_Access :=
                             new Real_Array (1 .. Natural'Max (1, Still_From (Q + 1) - Still_From (Q)));
                           C      : Natural := 0;
                        begin
                           for J in Still_From (Q) .. Still_From (Q + 1) - 1 loop
                              declare
                                 K      : constant Positive := By_Still (J);
                                 Du, Dv : Real;
                              begin
                                 if Have (Sg (K).Point) then
                                    Miss (L, Pose (Turns (Q), Steps (Q)), Ray (L, Point_Of (Sg (K).Point)),
                                          Inv (Sg (K).Point), (Sg (K).U, Sg (K).V), Sigma (K), Du, Dv);
                                    C := C + 1;
                                    Larger (C) := Real'Max (abs Du, abs Dv);
                                 end if;
                              end;
                           end loop;
                           if C = 0 or else Driver.Stats.Median (Larger (1 .. C)) > Driver.Conventions.Z then
                              Live (Q) := False;
                              Judged_Out := True;
                           end if;
                           Free (Larger);
                        end;
                     end if;
                  end loop;
                  exit when not Judged_Out;
               end;
            end loop;
            --  The cost at the end, over every sighting of the eye.
            Cost := 0.0;
            for J in 1 .. Mine_Count loop
               declare
                  K      : constant Positive := By_Point (J);
                  Du, Dv : Real;
               begin
                  Miss (L, Pose (Turns (Sg (K).Still), Steps (Sg (K).Still)), Ray (L, Point_Of (Sg (K).Point)),
                        (if Have (Sg (K).Point) then Inv (Sg (K).Point) else 0.0), (Sg (K).U, Sg (K).V), Sigma (K),
                        Du, Dv);
                  Cost := Cost + Huber_Cost (Du) + Huber_Cost (Dv);
               end;
            end loop;
         end;
         if Apply then
            for K in 1 .. Ns loop
               if Sg (K).Eye = Eye and then not Live (Sg (K).Still) then
                  Dropped (K) := True;
               end if;
            end loop;
         end if;
         Free (Inv);
      end Refine_Eye;

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

      Best      : State :=
        (Turn => [others => Zero3], Place_At => [others => Zero3], X_Turn => Zero3, X_Place => Zero3, Log_F1 => 0.0,
         Log_F2 => 0.0, Inverse => null);
      Ambiguous_Cost : Real := Real'Last;   --  how much more the other start's adjustment cost
      Ambiguous_Turn : Real := 0.0;         --  and how far its second eye's turn lay from the kept one's
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
         --  An eye's cost at a lens: its stills begun from a turn alone and the direction given it at that lens, then
         --  adjusted alone (Refine_Eye), its sightings' Huber cost at the end.
         function Eye_Cost (Eye : Positive; L : Lens) return Real is
            Turns, Steps : Vec3_Array (1 .. S) := [others => Zero3];
            Live  : Flag_Array (1 .. S);
            Sig, Spent, Cost : Real;
            Cnt   : Natural;
         begin
            for Q in 1 .. S loop
               Turn_Alone (L, Eye, Q, Turns (Q), Sig, Cnt, Spent);
               Direction (L, Eye, Q, Turns (Q), Steps (Q), Cnt);
            end loop;
            Refine_Eye (Eye, L, Turns, Steps, Live, Apply => False, Cost => Cost);
            return Cost;
         end Eye_Cost;

         --  An eye's focal length where none is given: the one at which its stills adjusted alone explain its
         --  sightings best (Eye_Cost), searched as the arm's fit searches its own (Kinematics.Fit's stage 1): from half
         --  the picture's width (seen at half a right angle), an octave about it, the bracket widened by its own width
         --  on the side whose end costs less until an inner point costs less than both ends, then narrowed by golden
         --  sections until it is the unchanged fraction wide in the logarithm. Nought where no bracket closes before
         --  the float's end: nothing tells the focal length. A turn alone at the still that turned most, the search's
         --  cost before, put the G1's wrist eye at 291 px for its 157, and the second eye's taken from the two eyes'
         --  turns went to nought (walkdump.py seed 1, 10-27): its walk stepped a tenth of a metre and turned half a
         --  degree.
         function Focal_Of (Eye : Positive; Cx, Cy : Real) return Real is
            Golden : constant Real := (Sqrt (5.0) - 1.0) / 2.0;
            Start  : constant Real := Cx;
            function Cost_At (Ln_F : Real) return Real is
              (Eye_Cost (Eye, (Focal => Exp (Ln_F), Cx => Cx, Cy => Cy)));
            A, B, C : Real;   --  logarithms: A < C < B, the cost at C below both ends' once bracketed
            Fa, Fb, Fc : Real;
         begin
            if Start <= 0.0 then
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
         end Focal_Of;
         Focal_1 : constant Real :=
           (if First_Start.Focal > 0.0 then First_Start.Focal else Focal_Of (1, First_Start.Cx, First_Start.Cy));
         L1 : constant Lens := (Focal => Focal_1, Cx => First_Start.Cx, Cy => First_Start.Cy);
         F2 : constant Real := (if Focal_1 > 0.0 then Focal_Of (2, Second_Centre.U, Second_Centre.V) else 0.0);
         L2 : constant Lens := (Focal => F2, Cx => Second_Centre.U, Cy => Second_Centre.V);
         Spent : Real;
         Turn1, Turn2 : Vec3_Array (1 .. S) := [others => Zero3];
         Sig1, Sig2   : Real_Array (1 .. S) := [others => Real'Last];
         Step1, Step2 : Vec3_Array (1 .. S) := [others => Zero3];
         Rx : Mat3 := Identity3;
         Tx : Vec3 := Zero3;
         Count : Natural;
         Live1, Live2 : Flag_Array (1 .. S);   --  the stills each eye's own adjustment places (Refine_Eye)
         Cost1, Cost2 : Real;
      begin
         if Focal_1 <= 0.0 or else F2 <= 0.0 then
            Free (Sigma);
            Free (Best_Residuals);
            return;
         end if;
         --  Each eye's stills adjusted alone at its lens, from a turn alone and the direction given it.
         for Q in 1 .. S loop
            Turn_Alone (L1, 1, Q, Turn1 (Q), Sig1 (Q), Count, Spent);
            Turn_Alone (L2, 2, Q, Turn2 (Q), Sig2 (Q), Count, Spent);
            Direction (L1, 1, Q, Turn1 (Q), Step1 (Q), Count);
            Direction (L2, 2, Q, Turn2 (Q), Step2 (Q), Count);
         end loop;
         Refine_Eye (1, L1, Turn1, Step1, Live1, Apply => True, Cost => Cost1);
         Refine_Eye (2, L2, Turn2, Step2, Live2, Apply => True, Cost => Cost2);
         --  The second eye's frame on the first from the stills' motions of both eyes, linear (Andreff, Horaud and
         --  Espiau, 2001): the first eye moved by (R1, t1) carries the second by X^-1 (R1, t1) X, so R1 Y = Y R2 and
         --  (R1 - I) Tx - Y t2 = -t1, where Y is the second eye's turn on the first times the ratio of the two eyes'
         --  scales (each eye's steps are in its own), linear in Y's nine entries and Tx: least squares over the stills
         --  taken, the turn the rotation nearest to Y (Kabsch's). The steps tell what turns about one axis alone leave
         --  free (a body that only turns about the vertical and steps on the floor).
         --  The stills taken are those the most of which one frame explains: the frame of every pair of stills both
         --  eyes' own adjustments place (Refine_Eye), and of all of them, each tried on every such still; a still
         --  agrees with a frame where both its equations miss by no more than their own sizes (the turns' by the
         --  larger of R1 - I and R2 - I, the steps' by the larger step); the frame most agree with, the fewest misses
         --  between equals, is solved again from the stills that agree, and the rest are left out with both eyes'
         --  sightings of them. One frame explaining the walk's every still is what the eyes riding one body means: the
         --  G1's longest walk had its first still's wrist eye matched 14 px off the truth in a field that one motion
         --  of that eye explains (the view moved 31 px; the matcher locked onto another of the floor's squares), its
         --  step the wrong way round for the head eye's, and the frame with it in lay 106 to 165 degrees from the
         --  truth, the joint adjustment fitting it better than the truth's (walkdump.py, 10-27). None where no frame is
         --  told.
         declare
            Both : constant Flag_Array (1 .. S) := [for Q in 1 .. S => Live1 (Q) and then Live2 (Q)];
            --  The entry of Y at row I and column J, column by column: Y's columns stacked.
            function At_Y (I, J : Positive) return Positive is ((J - 1) * 3 + I);
            procedure Solve (Use_Still : Flag_Array; Y : out Mat3; T : out Vec3; Ok : out Boolean) is
               Rows : constant Natural := 12 * S;
               Am   : Real_Matrix (1 .. Rows, 1 .. 12) := [others => [others => 0.0]];
               Bv   : Real_Vector (1 .. Rows) := [others => 0.0];
               Xv   : Real_Vector (1 .. 12);
               Full : Boolean;
            begin
               for Q in 1 .. S loop
                  if Use_Still (Q) then
                     declare
                        R1 : constant Mat3 := Exp (Turn1 (Q));
                        R2 : constant Mat3 := Exp (Turn2 (Q));
                        Base : constant Natural := 12 * (Q - 1);
                     begin
                        --  R1 Y - Y R2 = 0, entry (I, J): sum over K of R1 (I, K) Y (K, J) - Y (I, K) R2 (K, J).
                        for I in 1 .. 3 loop
                           for J in 1 .. 3 loop
                              for K in 1 .. 3 loop
                                 Am (Base + At_Y (I, J), At_Y (K, J)) :=
                                   Am (Base + At_Y (I, J), At_Y (K, J)) + R1 (I, K);
                                 Am (Base + At_Y (I, J), At_Y (I, K)) :=
                                   Am (Base + At_Y (I, J), At_Y (I, K)) - R2 (K, J);
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
                  end if;
               end loop;
               Driver.Numerics.Dense.Least_Squares (Am, Bv, Xv, Full);
               Ok := Full and then (for all V of Xv => Finite (V));
               Y := [for I in 1 .. 3 => [for J in 1 .. 3 => (if Ok then Xv (At_Y (I, J)) else 0.0)]];
               T := (if Ok then [Xv (10), Xv (11), Xv (12)] else Zero3);
            end Solve;
            --  How far still Q's two equations miss under a frame, each over its own size; the larger.
            function Miss_Of (Q : Positive; Y : Mat3; T : Vec3) return Real is
               R1  : constant Mat3 := Exp (Turn1 (Q));
               R2  : constant Mat3 := Exp (Turn2 (Q));
               Yt2 : constant Vec3 := Times (Y, Step2 (Q));
               Step_Miss : constant Vec3 := Plus (Minus (Minus (Times (R1, T), T), Yt2), Step1 (Q));
               Step_Size : constant Real := Real'Max (Real'Max (Norm (Step1 (Q)), Norm (Yt2)), Real'Model_Small);
               function Frobenius (A : Mat3) return Real is
                 (Sqrt (A (1, 1) ** 2 + A (1, 2) ** 2 + A (1, 3) ** 2 + A (2, 1) ** 2 + A (2, 2) ** 2 + A (2, 3) ** 2
                        + A (3, 1) ** 2 + A (3, 2) ** 2 + A (3, 3) ** 2));
               Turn_Miss : constant Real := Frobenius (Times (R1, Y) - Times (Y, R2));
               Turn_Size : constant Real :=
                 Real'Max (Frobenius (Y) * Real'Max (Frobenius (R1 - Identity3), Frobenius (R2 - Identity3)),
                           Real'Model_Small);
            begin
               return Real'Max (Turn_Miss / Turn_Size, Norm (Step_Miss) / Step_Size);
            end Miss_Of;
            Best_Count : Natural := 0;
            Best_Miss  : Real := Real'Last;
            Best_Y     : Mat3 := Identity3;
            Best_T     : Vec3 := Zero3;
            Use_Still  : Flag_Array (1 .. S) := [others => False];
            Solved     : Boolean := False;
            --  A frame tried: how many stills agree with it, and their misses summed.
            procedure Try (Y : Mat3; T : Vec3) is
               Count : Natural := 0;
               Sum   : Real := 0.0;
            begin
               for Q in 1 .. S loop
                  if Both (Q) and then Miss_Of (Q, Y, T) <= 1.0 then
                     Count := Count + 1;
                     Sum := Sum + Miss_Of (Q, Y, T);
                  end if;
               end loop;
               if Count > Best_Count or else (Count = Best_Count and then Sum < Best_Miss) then
                  Best_Count := Count;
                  Best_Miss := Sum;
                  Best_Y := Y;
                  Best_T := T;
               end if;
            end Try;
         begin
            declare
               Y  : Mat3;
               T  : Vec3;
               Ok : Boolean;
            begin
               Solve (Both, Y, T, Ok);
               if Ok then
                  Try (Y, T);
               end if;
               for Q1 in 1 .. S loop
                  for Q2 in Q1 + 1 .. S loop
                     if Both (Q1) and then Both (Q2) then
                        Solve ([for Q in 1 .. S => Q = Q1 or else Q = Q2], Y, T, Ok);
                        if Ok then
                           Try (Y, T);
                        end if;
                     end if;
                  end loop;
               end loop;
            end;
            if Best_Count >= 2 then
               Use_Still := [for Q in 1 .. S => Both (Q) and then Miss_Of (Q, Best_Y, Best_T) <= 1.0];
               declare
                  Y  : Mat3;
                  T  : Vec3;
                  Ok : Boolean;
               begin
                  Solve (Use_Still, Y, T, Ok);
                  if Ok then
                     Rx := Orthonormalize (Y);
                     Tx := T;
                     Solved := True;
                  end if;
               end;
               for K in 1 .. Ns loop
                  if Both (Sg (K).Still) and then not Use_Still (Sg (K).Still) then
                     Dropped (K) := True;
                  end if;
               end loop;
            end if;
            if not Solved then
               Free (Sigma);
               Free (Best_Residuals);
               return;
            end if;
         end;
         --  The adjustment from there, and from there turned half a turn about the axis the first eye's stills turned
         --  about (the principal axis of their turns), the second eye's place with it: where the stills turn about one
         --  axis and step little, the steps alone tell the second eye's turn about that axis, and by their signs; the
         --  G1's walks adjusted from the linear start alone came out 156 to 177 degrees from the truth at both focal
         --  lengths right (walkdump.py, its longest walk and its slowest, 10-27). The one whose adjustment costs the
         --  less is kept. Where the other costs no more than a half Z squared more (a likelihood's ratio no gate at Z
         --  tells from one) and its second eye's turn lies beyond Z of the kept one's sigma from it, the stills tell
         --  both alike: not Ok (Ambiguous).
         declare
            Axis : Vec3 := [1.0, 0.0, 0.0];
            Other_Turn : Vec3 := Zero3;
            Other_Cost : Real := Real'Last;
         begin
            declare
               M3 : Mat3 := [others => [others => 0.0]];
               V3 : Vec3;
               E3 : Mat3;
            begin
               for Q in 1 .. S loop
                  if Live1 (Q) then
                     M3 := M3 + Outer (Turn1 (Q), Turn1 (Q));
                  end if;
               end loop;
               Symmetric_Eigensystem (M3, V3, E3);
               if V3 (1) > 0.0 then
                  Axis := [E3 (1, 1), E3 (2, 1), E3 (3, 1)];
               end if;
            end;
            for Turned in Boolean loop
               declare
                  St    : State;
                  Fc    : Real;
                  Nm    : Real_Matrix (1 .. Np, 1 .. Np);
                  Sq    : Real;
                  Res   : Real_Access := new Real_Array (1 .. 2 * Ns);
                  Half  : constant Mat3 := (if Turned then Exp (Ada.Numerics.Pi * Axis) else Identity3);
               begin
                  St.Turn := Turn1;
                  St.Place_At := Step1;
                  St.X_Turn := Log (Times (Half, Rx));
                  St.X_Place := Times (Half, Tx);
                  St.Log_F1 := Log (L1.Focal);
                  St.Log_F2 := Log (F2);
                  St.Inverse := new Real_Array'(1 .. Natural'Max (1, Nd) => 1.0);
                  Place_Points (St);
                  Gauge (St);
                  Adjust (St, Fc, Nm, Sq, Res.all);
                  if not Have_Best or else Fc < Result.Cost then
                     if Have_Best then
                        Other_Turn := Best.X_Turn;
                        Other_Cost := Result.Cost;
                        Release (Best);
                     end if;
                     Result.Cost := Fc;
                     Best := St;
                     Best_Normal := Nm;
                     Best_Squares := Sq;
                     Best_Residuals.all := Res.all;
                     Have_Best := True;
                  else
                     Other_Turn := St.X_Turn;
                     Other_Cost := Fc;
                     Release (St);
                  end if;
                  Free (Res);
               end;
            end loop;
            if Have_Best then
               Ambiguous_Cost := Other_Cost - Result.Cost;
               Ambiguous_Turn := Angle (Times (Transposed (Exp (Best.X_Turn)), Exp (Other_Turn)));
            end if;
         end;
      end;
      if Have_Best then
         declare
            First, Second : Rigid_Array (1 .. S);
            Larger : Real_Access := new Real_Array (1 .. Ns);
            Used_Larger : Real_Access := new Real_Array (1 .. Ns);   --  of those no eye's own adjustment dropped
            Kept   : Natural := 0;
            Used   : Natural := 0;
            Final_Scale : constant Real := Live_Scale (Best_Residuals.all);
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
               if not Dropped (K) then
                  Used := Used + 1;
                  Used_Larger (Used) := Larger (K);
               end if;
               if not Dropped (K) and then Larger (K) <= Driver.Conventions.Z * Final_Scale then
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
            Result.Used := Used;
            Result.Misfit := (if Used > 0 then Driver.Stats.Median (Used_Larger (1 .. Used)) else Real'Last);
            Result.Scale := Final_Scale;
            --  The second eye's turn's covariance: the pose terms' normal matrix (the inverse depths taken out)
            --  inverted on the subspace the sightings determine (its eigenvalues above the float's resolution of
            --  the largest; the scale, which no residual moves, is not), the turn's block, times the residuals'
            --  weighted squares over their degrees of freedom.
            declare
               Values  : Real_Vector (1 .. Np);
               Vectors : Real_Matrix (1 .. Np, 1 .. Np);
               Cov     : Mat3 := [others => [others => 0.0]];
               Dof     : constant Integer := 2 * Used - Np - Nd;
               Largest : Real := 0.0;
               V3      : Vec3;
               M3      : Mat3;
            begin
               Driver.Numerics.Dense.Small_Eigensystem (Best_Normal, Values, Vectors);
               for P in 1 .. Np loop
                  Largest := Real'Max (Largest, Values (P));
               end loop;
               Result.Covariance := [others => [others => 0.0]];
               for P in 1 .. Np loop
                  if Values (P) > Real'Model_Epsilon * Largest * Real (Np) then
                     for A in 1 .. 8 loop
                        for B in 1 .. 8 loop
                           Result.Covariance (A, B) := Result.Covariance (A, B)
                             + Vectors (6 * S + A, P) * Vectors (6 * S + B, P) / Values (P);
                        end loop;
                     end loop;
                  end if;
               end loop;
               if Dof > 0 then
                  for A in 1 .. 8 loop
                     for B in 1 .. 8 loop
                        Result.Covariance (A, B) := (Best_Squares / Real (Dof)) * Result.Covariance (A, B);
                     end loop;
                  end loop;
               end if;
               for A in 1 .. 3 loop
                  for B in 1 .. 3 loop
                     Cov (A, B) := Result.Covariance (A, B);
                  end loop;
               end loop;
               if (for all I in 1 .. 3 => (for all J in 1 .. 3 => Finite (Cov (I, J)))) then
                  Symmetric_Eigensystem (Cov, V3, M3);
                  Result.Turn_Sigma := Sqrt (Real'Max (0.0, V3 (1)));
               else
                  Result.Turn_Sigma := Real'Last;
               end if;
            end;
            Result.Ok := Result.Misfit <= Driver.Conventions.Z and then Result.Turn_Sigma < Real'Last
              and then not (Ambiguous_Cost <= Driver.Conventions.Z ** 2 / 2.0
                            and then Ambiguous_Turn > Driver.Conventions.Z * Result.Turn_Sigma)
              and then (for all I in 1 .. 3 => Finite (Best.X_Turn (I)) and then Finite (Best.X_Place (I)));
            Free (Larger);
            Free (Used_Larger);
         end;
         Release (Best);
      end if;
      Free (Sigma);
      Free (Best_Residuals);
   end Calibrate;

end Driver.Rig;
