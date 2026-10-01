with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;

package body Driver.Action.Goals is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use type Arm_Id;
   use type Surface_Id;

   package Contact renames Driver.Action.Contact;

   Pi : constant := Ada.Numerics.Pi;

   function Word (Q : Quantity) return String is
     (case Q is when Height => "height", when Heading => "heading", when Tilt => "tilt");

   function Gravity (S : Snapshot) return Vec3 is
     (if S.Up.Sigma < Real'Last and then abs S.Up.Unit_Vector > 0.0 then Unit (S.Up.Unit_Vector) else Zero3);

   function Still_Eye (S : Snapshot; E : Natural) return Boolean is
     (E in 1 .. Natural (S.Eyes.Length) and then S.Eyes (E).On_Arm = 0 and then Known (Position (S.Eyes (E).Pose)));

   function Changeable (S : Snapshot) return Quantity_Set is
      Up_Known : constant Boolean := abs Gravity (S) > 0.0;
      Arms     : constant Boolean := not S.Arms.Is_Empty;
   begin
      return [Height => Up_Known and then Arms,
              Heading => Up_Known and then Arms,
              Tilt => Up_Known and then Arms and then (for some E in 1 .. Natural (S.Eyes.Length) => Still_Eye (S, E))];
   end Changeable;

   function Up_Of (S : Snapshot; T : Thing_Id) return Vec3 is
      X : constant Thing_State := Thing (S, T);
   begin
      if X.Support /= 0 and then Has_Surface (S, X.Support) then
         declare
            N : constant Vec3 := Surface (S, X.Support).Normal.Unit_Vector;
         begin
            if abs N > 0.0 then
               return Unit (N);
            end if;
         end;
      end if;
      return Gravity (S);
   end Up_Of;

   function Long_Axis (S : Snapshot; T : Thing_Id; Sigma : out Real) return Vec3 is
      X  : constant Thing_State := Thing (S, T);
      N  : constant Vec3 := Up_Of (S, T);
      E1, E2 : Vec3;
      C  : Vec3 := Zero3;
      Count : Natural := 0;
      Sxx, Sxy, Syy : Real := 0.0;
   begin
      Sigma := Real'Last;
      if not (abs N > 0.0) or else X.Samples.Is_Empty or else X.Sigma = Real'Last then
         return Zero3;
      end if;
      Contact.Plane_Basis (N, E1, E2);
      for Q of X.Samples loop
         C := C + Q.Point;
         Count := Count + 1;
      end loop;
      C := C / Real (Count);
      for Q of X.Samples loop
         declare
            A : constant Real := (Q.Point - C) * E1;
            B : constant Real := (Q.Point - C) * E2;
         begin
            Sxx := Sxx + A * A / Real (Count);
            Sxy := Sxy + A * B / Real (Count);
            Syy := Syy + B * B / Real (Count);
         end;
      end loop;
      declare
         Half_Gap : constant Real := Sqrt (((Sxx - Syy) / 2.0) ** 2 + Sxy ** 2);   --  half of l1 - l2
         Larger   : constant Real := (Sxx + Syy) / 2.0 + Half_Gap;
         Theta    : constant Real := Arctan (2.0 * Sxy, Sxx - Syy) / 2.0;
      begin
         if Half_Gap = 0.0 then
            return Zero3;
         end if;
         --  The principal axis of n points each Sigma off turns by about
         --  Sigma sqrt (l1 / n) / (l1 - l2); a quarter turn must stand out.
         Sigma := X.Sigma * Sqrt (Larger / Real (Count)) / (2.0 * Half_Gap);
         if not Significant (Pi / 2.0, Sigma) then
            Sigma := Real'Last;
            return Zero3;
         end if;
         return Cos (Theta) * E1 + Sin (Theta) * E2;
      end;
   end Long_Axis;

   function Missing (What : String) return Answer is
     ((Ok => False, Motion => Contact.Still (Zero3), Gap => Unknown, Done => False,
       Why => To_Unbounded_String ("I have not measured " & What)));

   function Moving (Motion : Contact.Twist; Gap : Estimate; Why : String := "") return Answer is
     ((Ok => True, Motion => Motion, Gap => Gap, Done => False, Why => To_Unbounded_String (Why)));

   function Twist_Of (S : Snapshot; T : Thing_Id; Q : Quantity; Increase : Boolean) return Answer is
      X    : constant Thing_State := Thing (S, T);
      N    : constant Vec3 := Up_Of (S, T);
      Sign : constant Real := (if Increase then 1.0 else -1.0);
   begin
      if not (abs N > 0.0) then
         return Missing ("which way is up from it");
      elsif not Known (X.Centre) then
         return Missing ("where it is");
      end if;
      case Q is
         when Height =>
            return Moving (Contact.Slide (Sign * N), Unknown);
         when Heading =>
            declare
               Sigma : Real;
               A     : constant Vec3 := Long_Axis (S, T, Sigma);
            begin
               if not (abs A > 0.0) then
                  return Missing ("which way it is long: it is as long one way as another");
               end if;
               return Moving (Contact.Rotation (Sign * N, 1.0, X.Centre.Mean), Unknown);
            end;
         when Tilt =>
            if not Still_Eye (S, X.Best_Eye) then
               return Missing ("it from an eye that stays still while my arms move");
            end if;
            declare
               D : constant Vec3 := X.Centre.Mean - S.Eyes (X.Best_Eye).Pose.Pose.Translation;
               H : constant Vec3 := D - Real'(D * N) * N;
            begin
               if not Significant (Vector_Gate (2), abs H, X.Sigma) then
                  return Missing ("which way it leans: the still eye looks straight down its up");
               end if;
               --  About n x h the top, along n, moves along h: away from the eye.
               return Moving (Contact.Rotation (Sign * Cross (N, Unit (H)), 1.0, X.Centre.Mean), Unknown);
            end;
      end case;
   end Twist_Of;

   function Item_Of (S : Snapshot; T : Thing_Id) return Item is
      X : constant Thing_State := Thing (S, T);
   begin
      return (Centre  => X.Centre,
              Samples => X.Samples,
              Sigma   => X.Sigma,
              Up      => (if X.Support /= 0 then Up_Of (S, T) else Zero3),
              Eye     => X.Best_Eye);
   end Item_Of;

   function Point_Item (P : Point_Estimate) return Item is
     ((Centre  => P,
       Samples => Sample_Vectors.Empty_Vector,
       Sigma   => Sqrt (Real'Max (P.Covariance (1, 1), Real'Max (P.Covariance (2, 2), P.Covariance (3, 3)))),
       Up      => Zero3,
       Eye     => 0));

   --  The item's highest or lowest extent along N.
   function Extent (X : Item; N : Vec3; Highest : Boolean) return Real is
      E : Real := (if Highest then Real'First else Real'Last);
   begin
      if X.Samples.Is_Empty then
         return X.Centre.Mean * N;
      end if;
      for Q of X.Samples loop
         E := (if Highest then Real'Max (E, Q.Point * N) else Real'Min (E, Q.Point * N));
      end loop;
      return E;
   end Extent;

   function Toward (S : Snapshot; Subject, Object : Item; R : Pair_Relation; Long : Vec3 := Zero3;
                    Long_Sigma : Real := Real'Last) return Answer
   is
      Sigma : constant Real := Sqrt (Subject.Sigma ** 2 + Object.Sigma ** 2);
      Apart : constant Vec3 := Object.Centre.Mean - Subject.Centre.Mean;
      N_O   : constant Vec3 := (if abs Object.Up > 0.0 then Object.Up else Gravity (S));
      Eye   : constant Natural := (if Still_Eye (S, Object.Eye) then Object.Eye else Subject.Eye);

      function Holds (Why : String) return Answer is
        ((Ok => True, Motion => Contact.Still (Subject.Centre.Mean), Gap => (Value => 0.0, Sigma => Sigma,
          Degrees_Of_Freedom => 0), Done => True, Why => To_Unbounded_String (Why)));

      function Gap_Of (Value : Real) return Estimate is ((Value => Value, Sigma => Sigma, Degrees_Of_Freedom => 0));

      --  A subject lying on a surface goes along it: the part of D into the
      --  surface is dropped, unless D, known to Angle_Sigma, goes straight in.
      function Along (D : Vec3; Angle_Sigma : Real; Gap : Estimate; Why : String := "") return Answer is
         U : constant Vec3 := Subject.Up;
      begin
         if abs U > 0.0 and then Real'(D * U) < 0.0 then
            declare
               F : constant Vec3 := D - Real'(D * U) * U;
            begin
               if not (abs F > 0.0) or else not Significant (Arccos (Real'Min (1.0, -Real'(D * U))), Angle_Sigma)
               then
                  return (Ok => False, Motion => Contact.Still (Subject.Centre.Mean), Gap => Gap, Done => False,
                          Why => To_Unbounded_String ("the only way to do that goes into the surface it lies on"));
               end if;
               return Moving (Contact.Slide (Unit (F)), Gap, "it lies on a surface, so it goes along it");
            end;
         end if;
         return Moving (Contact.Slide (Unit (D)), Gap, Why);
      end Along;

      --  Over or under the object along N_O, clear of its top or bottom.
      function Over (Above_It, Then_Down : Boolean) return Answer is
         Flat  : constant Vec3 := Apart - Real'(Apart * N_O) * N_O;
         Level : constant Boolean := Significant (Vector_Gate (2), abs Flat, Sigma);
         Clear : constant Real :=
           (if Above_It then Extent (Subject, N_O, Highest => False) - Extent (Object, N_O, Highest => True)
            else Extent (Object, N_O, Highest => False) - Extent (Subject, N_O, Highest => True));
         Is_Clear : constant Boolean := Clear > 0.0 and then Significant (Clear, Sigma);
         Way   : constant Vec3 := (if Above_It then N_O else -N_O);
      begin
         if Level then
            if Is_Clear then
               return Moving (Contact.Slide (Unit (Flat)), Gap_Of (abs Flat + (if Then_Down then Clear else 0.0)),
                              "it is clear of the other, so it goes across to it");
            end if;
            return Along (Way, Sigma / abs Flat, Gap_Of (abs Flat - Clear),
                          "it is not clear of the other yet, so it goes " & (if Above_It then "up" else "down")
                          & " first");
         elsif Is_Clear then
            if Then_Down then
               return Moving (Contact.Slide (-Way), Gap_Of (Clear), "it is straight over the other, so it goes onto it");
            end if;
            return Holds ("it is straight " & (if Above_It then "over" else "under") & " the other");
         elsif Then_Down and then not Significant (Clear, Sigma) then
            return Holds ("it is on the other's top as near as I can tell");
         end if;
         return Along (Way, Sigma / Real'Max (abs Apart, Sigma), Gap_Of (-Clear));
      end Over;

   begin
      if not Known (Subject.Centre) or else not Known (Object.Centre) then
         return Missing ("where the two of them are");
      end if;
      case R is
         when Nearer | Farther =>
            if not Still_Eye (S, Eye) then
               return Missing ("which of them is nearer: no eye that stays still sees them");
            end if;
            declare
               C : constant Vec3 := S.Eyes (Eye).Pose.Pose.Translation;
               D_S : constant Real := abs (Subject.Centre.Mean - C);
               D_O : constant Real := abs (Object.Centre.Mean - C);
               G   : constant Real := (if R = Nearer then D_S - D_O else D_O - D_S);
            begin
               if G < 0.0 and then Significant (G, Sigma) then
                  return Holds ("it is " & (if R = Nearer then "nearer" else "farther") & " already");
               end if;
               return Along ((if R = Nearer then -1.0 else 1.0) * Unit (Subject.Centre.Mean - C), Sigma / D_S,
                             Gap_Of (G));
            end;
         when Left | Right =>
            if not Still_Eye (S, Eye) then
               return Missing ("left and right: no eye that stays still sees them");
            end if;
            declare
               P   : constant Rigid := S.Eyes (Eye).Pose.Pose;
               Q_S : constant Vec3 := Transpose (P.Rotation) * (Subject.Centre.Mean - P.Translation);
               Q_O : constant Vec3 := Transpose (P.Rotation) * (Object.Centre.Mean - P.Translation);
            begin
               if not (Q_S (3) > 0.0 and then Q_O (3) > 0.0) then
                  return Missing ("left and right: they are not both in front of the still eye");
               end if;
               declare
                  U_S : constant Real := Q_S (1) / Q_S (3);
                  U_O : constant Real := Q_O (1) / Q_O (3);
                  --  The image column grows along (x - u z) / z of the eye.
                  Grow : constant Vec3 := P.Rotation * [1.0, 0.0, -U_S];
                  G    : constant Real := (if R = Right then U_O - U_S else U_S - U_O) * Q_S (3);
               begin
                  if G < 0.0 and then Significant (G, Sigma) then
                     return Holds ("it is " & (if R = Right then "right" else "left") & " of it already");
                  end if;
                  return Along ((if R = Right then 1.0 else -1.0) * Unit (Grow),
                                Sqrt (Real'Max (S.Eyes (Eye).Pose.Rotation_Covariance (1, 1),
                                      Real'Max (S.Eyes (Eye).Pose.Rotation_Covariance (2, 2),
                                                S.Eyes (Eye).Pose.Rotation_Covariance (3, 3)))),
                                Gap_Of (G));
               end;
            end;
         when Above =>
            return (if abs N_O > 0.0 then Over (True, False) else Missing ("which way is up"));
         when Below =>
            return (if abs N_O > 0.0 then Over (False, False) else Missing ("which way is up"));
         when Onto =>
            return (if abs N_O > 0.0 then Over (True, True) else Missing ("which way is up"));
         when Off =>
            if not (abs N_O > 0.0) then
               return Missing ("which way is up");
            end if;
            declare
               Clear : constant Real := Extent (Subject, N_O, Highest => False) - Extent (Object, N_O, Highest => True);
            begin
               if Clear > 0.0 and then Significant (Clear, Sigma) then
                  return Holds ("it is clear of the other's top");
               end if;
               return Moving (Contact.Slide (N_O), Gap_Of (-Clear));
            end;
         when Facing =>
            declare
               Axis : constant Vec3 := (if abs Subject.Up > 0.0 then Subject.Up else Gravity (S));
            begin
               if not (abs Long > 0.0) then
                  return Missing ("which way it is long: it is as long one way as another");
               elsif not (abs Axis > 0.0) then
                  return Missing ("which way is up");
               end if;
               declare
                  Flat : constant Vec3 := Apart - Real'(Apart * Axis) * Axis;
               begin
                  if not Significant (Vector_Gate (2), abs Flat, Sigma) then
                     return Missing ("the other apart from it");
                  end if;
                  declare
                     To  : constant Vec3 := Unit (Flat);
                     --  Either end of the long axis may point at it: the nearer.
                     A   : constant Vec3 := (if Long * To < 0.0 then -Long else Long);
                     Phi : constant Real := Arctan (Cross (A, To) * Axis, A * To);
                  begin
                     if not Significant (Phi, Long_Sigma) then
                        return Holds ("its long way points at the other as near as I can tell");
                     end if;
                     return (Ok => True, Done => False, Why => Null_Unbounded_String,
                             Motion => Contact.Rotation ((if Phi > 0.0 then Axis else -Axis), 1.0, Subject.Centre.Mean),
                             Gap => (Value => abs Phi, Sigma => Long_Sigma, Degrees_Of_Freedom => 0));
                  end;
               end;
            end;
      end case;
   end Toward;

end Driver.Action.Goals;
