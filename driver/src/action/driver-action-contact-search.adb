with Ada.Containers.Indefinite_Ordered_Maps;
with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Action.Contact.Wrench;
with Driver.Conventions;
with Driver.Log;

package body Driver.Action.Contact.Search is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Uncertain;

   package Physics renames Driver.Action.Contact.Wrench;

   Pi        : constant := Ada.Numerics.Pi;
   Z         : constant Real := Driver.Conventions.Z;
   Round_Off : constant Real := Sqrt (Real'Model_Epsilon);

   use type Driver.Robot.Arm_Id;
   use type Driver.World.Surface_Id;

   function Largest_Sigma (Covariance : Mat3) return Real is
     (if Covariance (1, 1) >= Real'Last then Real'Last
      else Sqrt (Real'Max (Covariance (1, 1), Real'Max (Covariance (2, 2), Covariance (3, 3)))));

   --  The least rotation taking the direction From onto To.
   function Align (From, To : Vec3) return Mat3 is
      F : constant Vec3 := Unit (From);
      T : constant Vec3 := Unit (To);
      A : constant Vec3 := Cross (F, T);
      C : constant Real := F * T;
   begin
      if abs A > Round_Off then
         return Exp (Arctan (abs A, C) * Unit (A));
      elsif C > 0.0 then
         return Identity3;
      end if;
      declare
         E1, E2 : Vec3;
      begin
         Plane_Basis (F, E1, E2);
         return Exp (Pi * E1);
      end;
   end Align;

   function Moves (P : Pad) return Boolean is (abs (P.Closed - P.Open) > 0.0);

   function Effector_Of (S : Snapshot; A : Arm_Id) return Effector is
      Arm_S   : constant Arm_State := Arm (S, A);
      E       : Effector := (Arm => A, Tool => Arm_S.Tool.Pose, others => <>);
      Tip_Sum : Vec3 := Zero3;
      Tips    : Natural := 0;
      Depth   : Real := Real'Last;
      Tip_Sd  : Real := 0.0;
      Unknown : Boolean := False;
   begin
      E.Surface := Arm_S.Surface;
      for H of S.Hands loop
         if H.Arm = A then
            E.Closers.Append (Closer_Info'(Hand => H.Id,
                                           Now  => (if Known (H.Fraction) then H.Fraction.Value else 0.0)));
            if Known (H.Depth) then
               Depth := Real'Min (Depth, H.Depth.Value);
            else
               Unknown := True;
            end if;
            declare
               C      : constant Positive := Natural (E.Closers.Length);
               Meet   : Vec3 := Zero3;
               Moving : Natural := 0;
               function Moves (L : Lobe_State) return Boolean is
                 (Significant (abs (L.Closed_Tip - L.Open_Tip), Sqrt (2.0) * L.Tip_Sigma));
            begin
               --  A lobe that does not move faces the point where the moving
               --  ones meet when closed on nothing.
               for L of H.Lobes loop
                  if Moves (L) then
                     Meet := Meet + L.Closed_Tip;
                     Moving := Moving + 1;
                  end if;
               end loop;
               for L of H.Lobes loop
                  Tip_Sum := Tip_Sum + L.Open_Tip;
                  Tips := Tips + 1;
                  Tip_Sd := Real'Max (Tip_Sd, L.Tip_Sigma);
                  declare
                     Toward : constant Vec3 := (if Moving > 0 then Meet / Real (Moving) - L.Open_Tip else Zero3);
                  begin
                     if Moves (L) or else abs Toward > 0.0 then
                        declare
                           Facing : constant Vec3 :=
                             (if Moves (L) then Unit (L.Closed_Tip - L.Open_Tip) else Unit (Toward));
                           Ahead  : constant Vec3 := (L.Thickness / 2.0) * Facing;
                        begin
                           E.Pads.Append (Pad'(Closer => C, Open => L.Open_Tip + Ahead,
                                               Closed => (if Moves (L) then L.Closed_Tip else L.Open_Tip) + Ahead,
                                               Facing => Facing, Half_Width => L.Width / 2.0,
                                               Thickness => L.Thickness));
                        end;
                     end if;
                  end;
               end loop;
            end;
         end if;
      end loop;
      if Tips > 0 and then abs Tip_Sum > 0.0 then
         E.Along := Unit (Tip_Sum);
      end if;
      for H of S.Hands loop
         if H.Arm = A and then Known (H.Fraction) then
            for L of H.Lobes loop
               E.Ends.Append (Sample'(Point  => L.Open_Tip + H.Fraction.Value * (L.Closed_Tip - L.Open_Tip),
                                      Normal => E.Along));
            end loop;
         end if;
      end loop;
      E.Depth := (if Unknown or else Depth = Real'Last then 0.0 else Depth);
      declare
         Lever  : constant Real := (if Tips > 0 then abs (Tip_Sum / Real (Tips)) else 0.0);
         Pos_Sd : constant Real := Largest_Sigma (Arm_S.Tool.Position_Covariance);
         Rot_Sd : constant Real := Largest_Sigma (Arm_S.Tool.Rotation_Covariance);
      begin
         if Pos_Sd < Real'Last and then Rot_Sd < Real'Last and then Tip_Sd < Real'Last then
            E.Sigma := Sqrt (Pos_Sd ** 2 + (Lever * Rot_Sd) ** 2 + Tip_Sd ** 2);
         end if;
      end;
      if Natural (E.Pads.Length) < 2 then
         E.Why_Not := To_Unbounded_String ("this arm has fewer than two measured lobes that close together");
      elsif Unknown or else Depth = Real'Last then
         E.Why_Not := To_Unbounded_String ("how far a thing may go into this hand is not measured");
      elsif E.Sigma = Real'Last then
         E.Why_Not := To_Unbounded_String ("where this hand's lobes are is not measured with its uncertainty");
      elsif not (for some P of E.Pads => Moves (P)) then
         E.Why_Not := To_Unbounded_String ("none of this hand's lobes moves");
      else
         E.Closes := True;
      end if;
      return E;
   end Effector_Of;

   function Shape_Of (S : Snapshot; T : Thing_Id) return Shape is
      X : constant Thing_State := Thing (S, T);
      R : Shape := (Samples => X.Samples, Sigma => X.Sigma, Normal_Sigma => X.Normal_Sigma, Pitch => X.Pitch,
                    Centre => X.Centre, Base => No_Footing, others => <>);
   begin
      if X.Support /= 0 and then Has_Surface (S, X.Support) then
         declare
            F      : constant Surface_State := Surface (S, X.Support);
            Up     : constant Vec3 := F.Normal.Unit_Vector;
            Points : Point_Vectors.Vector;
         begin
            if abs Up > 0.0 and then Known (F.Point) then
               R.Floor_Point := F.Point.Mean;
               R.Floor_Up := Unit (Up);
               R.Floor_Sigma := Sigma_Along (F.Point.Covariance, R.Floor_Up);
               for Q of X.Samples loop
                  Points.Append (Q.Point);
               end loop;
               --  The foot is what lies on that surface, within a pitch and
               --  the combined uncertainty of the thing and the surface.
               R.Base := Footing_Of (Points, R.Floor_Point, R.Floor_Up,
                                     Real'Max (X.Pitch, Z * Sqrt (X.Sigma ** 2 + R.Floor_Sigma ** 2)));
            end if;
         end;
      end if;
      return R;
   end Shape_Of;

   function Say (A : Account) return String is
      function N (X : Natural) return String is (Driver.Log.Image (X));
   begin
      return "I looked at " & N (A.Placements) & " placements: " & N (A.Into_Material)
        & " would bring a lobe down on it, " & N (A.Too_Deep) & " would take it deeper into my hand than it goes, "
        & N (A.Through_Surface) & " would pass through the surface it lies on, " & N (A.Through_Others)
        & " through something beside it, " & N (A.Close_On_Air) & " would close on nothing; of "
        & N (A.Distinct) & " different contact sets "
        & (if A.Surface_In_Way then "none can do it, because that way goes into the surface it lies on"
           else N (A.Cannot_Balance) & " cannot make it move that way, " & N (A.Over_Bound)
                & " need friction it has already failed to give me, " & N (A.Unreachable)
                & " have no reachable pose with a clear way in");
   end Say;

   ------------------------------------------------------------------------
   --  The search proper, in two passes. The first finds every contact set
   --  with each face taken as its inscribed disc about the lobe's end, which
   --  does not change when the hand turns about a pinned face's axis if the
   --  other faces lie on that axis; the physics ranks those sets. The second
   --  realizes the best set: whole faces, the way in and the reach, in the
   --  order of least turning of the hand. Inner loops run over plain arrays:
   --  element access through a container costs a controlled reference.

   type Vec3_Array is array (Positive range <>) of Vec3;

   type Pin_Record is record
      Pad_Index, Sample : Positive;
      Psi               : Real := 0.0;     --  turn about the sample's normal
      Free_Turn         : Boolean := False;   --  the set is the same at every turn
   end record;

   package Pin_Vectors is new Ada.Containers.Vectors (Positive, Pin_Record);

   package Index_Vectors is new Ada.Containers.Vectors (Positive, Positive);

   type Group is record
      Touches    : Touch_Vectors.Vector;
      Pads       : Index_Vectors.Vector;   --  which pad makes each touch
      Pins       : Pin_Vectors.Vector;
      Mu_Nominal : Real := Physics.No_Way;
      Mu_Worst   : Real := -1.0;            --  negative: not computed yet
      Force      : Real := Physics.No_Way;  --  normals as measured, at the reference friction
      Worst      : Real := -1.0;            --  negative: not computed yet
      Single     : Boolean := False;        --  one touch: its placement is fitted afterwards
      Sample     : Natural := 0;            --  for a single touch, which sample
   end record;
   --  One contact set and the pins that make it.

   package Group_Maps is new Ada.Containers.Indefinite_Ordered_Maps (String, Group);

   type Footprint is (Disc, Whole_Face);
   --  Disc: a face's inscribed disc about the lobe's end. Whole_Face: the face
   --  back along the lobe as deep as a thing may go into the hand.

   procedure Find
     (Thing     : Shape;
      Beside    : Point_Vectors.Vector;
      E         : Effector;
      Motion    : Twist;
      Up        : Vec3;
      Friction  : Friction_Bounds;
      Reachable : not null access function (Tool : Rigid) return Boolean;
      Best      : out Candidate;
      Found     : out Boolean;
      Tried     : out Account)
   is
      N_Samples : constant Natural := Natural (Thing.Samples.Length);
      N_Pads    : constant Natural := Natural (E.Pads.Length);
      N_Closers : constant Natural := Natural (E.Closers.Length);
      Pts   : Vec3_Array (1 .. N_Samples);
      Nrm   : Vec3_Array (1 .. N_Samples);
      Has_N : array (1 .. N_Samples) of Boolean;
      Obs   : Vec3_Array (1 .. Natural (Beside.Length));
      Low_Corner, High_Corner : Vec3 := Zero3;
      Res   : constant Real := Thing.Pitch;
      Sig   : constant Real := (if E.Sigma < Real'Last and then Thing.Sigma < Real'Last
                                then Sqrt (E.Sigma ** 2 + Thing.Sigma ** 2) else Real'Last);
      Clear : constant Real := (if Sig < Real'Last then Z * Sig + Res else Real'Last);
      --  How far a face stays from where the thing may be, on the way in.
      Groups : Group_Maps.Map;
      Now_R  : constant Mat3 := E.Tool.Rotation;
      U      : constant Vec3 := Unit (Up);

      subtype Closer_Index is Positive range 1 .. Positive'Max (1, N_Closers);
      type Fractions is array (Closer_Index) of Real;
      type Pad_Hits is array (1 .. N_Pads) of Natural;

      function To_Vector (F : Fractions) return Real_Vectors.Vector is
         V : Real_Vectors.Vector;
      begin
         for C in 1 .. N_Closers loop
            V.Append (F (C));
         end loop;
         return V;
      end To_Vector;

      function Key_Of (T : Touch_Vectors.Vector; P : Index_Vectors.Vector) return String is
         K : Unbounded_String;
      begin
         for J in 1 .. Natural (T.Length) loop
            Append (K, Integer'Image (P (J)));
            for C in 1 .. 3 loop
               Append (K, Integer'Image (Integer (Real'Floor (T (J).Point (C) / Res))));
            end loop;
            Append (K, ";");
         end loop;
         return To_String (K);
      end Key_Of;

      --  The fraction at which a face, at A0 with its closer open and moving
      --  V per unit fraction, first meets a sample that faces it within its
      --  footprint; Real'Last when none does before the closer is shut.
      procedure First_Meeting
        (A0, V, Facing, Back : Vec3; Half_Width : Real; Face : Footprint; S : out Real; Hit : out Natural)
      is
         Speed : constant Real := V * Facing;
      begin
         S := Real'Last;
         Hit := 0;
         if Speed <= 0.0 then
            return;
         end if;
         for M in 1 .. N_Samples loop
            if Has_N (M) and then Nrm (M) * Facing < 0.0 then
               declare
                  Sq : constant Real := ((Pts (M) - A0) * Facing) / Speed;
               begin
                  if Sq >= 0.0 and then Sq <= 1.0 and then Sq < S then
                     declare
                        D    : constant Vec3 := Pts (M) - (A0 + Sq * V);
                        Flat : constant Vec3 := D - Real'(D * Facing) * Facing;
                        L    : constant Real := D * Back;
                        Fits : constant Boolean :=
                          (case Face is
                              when Disc       => abs Flat <= Half_Width,
                              when Whole_Face => abs (Flat - L * Back) <= Half_Width and then L >= -Res
                                                 and then L <= E.Depth);
                     begin
                        if Fits then
                           S := Sq;
                           Hit := M;
                        end if;
                     end;
                  end if;
               end;
            end if;
         end loop;
      end First_Meeting;

      type Outcome is (Touching, On_Air, Not_First);

      --  Pad K's face pinned on sample I, the hand turned R. Along the path
      --  relative to the pinned face, the closer's other faces give the
      --  fraction where one of them meets the thing too; the hand is placed
      --  there and the closing is then followed as it really happens, the
      --  hand fixed and every closer moving its own faces until one of its
      --  faces meets the thing.
      procedure Contacts
        (K, I : Positive; R : Mat3; Face : Footprint;
         X : out Vec3; Stops : out Fractions; Hits : out Pad_Hits; Result : out Outcome)
      is
         PK     : constant Pad := E.Pads (K);
         Back   : constant Vec3 := -(R * E.Along);
         S_Star : Real := Real'Last;
         Meet   : array (1 .. N_Pads) of Real;
      begin
         Hits := [others => 0];
         Stops := [others => 0.0];
         X := Zero3;
         for J in 1 .. N_Pads loop
            if J /= K and then E.Pads (J).Closer = PK.Closer then
               declare
                  PJ  : constant Pad := E.Pads (J);
                  S   : Real;
                  Hit : Natural;
               begin
                  First_Meeting (Pts (I) + R * (PJ.Open - PK.Open), R * ((PJ.Closed - PJ.Open) - (PK.Closed - PK.Open)),
                                 R * PJ.Facing, Back, PJ.Half_Width, Face, S, Hit);
                  S_Star := Real'Min (S_Star, S);
               end;
            end if;
         end loop;
         if S_Star = Real'Last then
            Result := On_Air;
            return;
         end if;
         X := Pts (I) - R * (PK.Open + S_Star * (PK.Closed - PK.Open));
         for J in 1 .. N_Pads loop
            First_Meeting (X + R * E.Pads (J).Open, R * (E.Pads (J).Closed - E.Pads (J).Open), R * E.Pads (J).Facing,
                           Back, E.Pads (J).Half_Width, Face, Meet (J), Hits (J));
         end loop;
         --  Each closer stops where its first face meets the thing; one that
         --  meets nothing keeps its present fraction.
         for C in 1 .. N_Closers loop
            declare
               Stop : Real := Real'Last;
            begin
               for J in 1 .. N_Pads loop
                  if E.Pads (J).Closer = C then
                     Stop := Real'Min (Stop, Meet (J));
                  end if;
               end loop;
               Stops (C) := (if Stop = Real'Last then E.Closers (C).Now else Stop);
            end;
         end loop;
         --  The pinned face must be the first to meet its sample, at the
         --  fraction the pin was solved for.
         if Hits (K) = 0 or else abs (Pts (Hits (K)) - Pts (I)) > Res
           or else abs (Meet (K) - S_Star) * abs (PK.Closed - PK.Open) > Res
         then
            Result := Not_First;
            return;
         end if;
         Hits (K) := I;
         declare
            Count : Natural := 0;
         begin
            for J in 1 .. N_Pads loop
               if Hits (J) /= 0 and then Moves (E.Pads (J))
                 and then abs (Meet (J) - Stops (E.Pads (J).Closer)) * abs (E.Pads (J).Closed - E.Pads (J).Open) <= Res
               then
                  Count := Count + 1;
               else
                  Hits (J) := 0;
               end if;
            end loop;
            Result := (if Count >= 2 then Touching else On_Air);
         end;
      end Contacts;

      procedure Touches_Of (Hits : Pad_Hits; T : out Touch_Vectors.Vector; P : out Index_Vectors.Vector) is
      begin
         T.Clear;
         P.Clear;
         for J in Hits'Range loop
            if Hits (J) /= 0 then
               T.Append (Touch'(Point => Pts (Hits (J)), Inward => -Nrm (Hits (J)), Patch => E.Pads (J).Half_Width,
                                Tension => False));
               P.Append (J);
            end if;
         end loop;
      end Touches_Of;

      function Pin_Rotation (K, I : Positive; Psi : Real) return Mat3 is
        (Exp (Psi * Nrm (I)) * Align (Now_R * E.Pads (K).Facing, -Nrm (I)) * Now_R);

      --  The farthest any other face's centre, or Extra beyond it, comes from
      --  the axis along pad K's facing through its face: the lever a turn
      --  about that axis acts on.
      function Lever (K : Positive; Extra : Real) return Real is
         PK : constant Pad := E.Pads (K);
         L  : Real := 0.0;
      begin
         for J in 1 .. N_Pads loop
            if J /= K then
               for F in 0 .. 1 loop
                  for G in 0 .. 1 loop
                     declare
                        P    : constant Pad := E.Pads (J);
                        D    : constant Vec3 :=
                          (P.Open + Real (F) * (P.Closed - P.Open)) - (PK.Open + Real (G) * (PK.Closed - PK.Open));
                        Perp : constant Vec3 := D - Real'(D * PK.Facing) * PK.Facing;
                     begin
                        L := Real'Max (L, abs Perp + (if Extra > 0.0 then P.Half_Width + Extra else 0.0));
                     end;
                  end loop;
               end loop;
            end if;
         end loop;
         return L;
      end Lever;

      function Steps_For (L : Real) return Positive is
        (if L <= Res then 1 else Positive (Real'Ceiling (2.0 * Pi * L / Res)));

      --  First pass: every contact set the closers make.
      procedure Enumerate is
      begin
         for K in 1 .. N_Pads loop
            if Moves (E.Pads (K)) then
               declare
                  Steps : constant Positive := Steps_For (Lever (K, 0.0));
               begin
                  for I in 1 .. N_Samples loop
                     if Has_N (I) then
                        for Step in 0 .. Steps - 1 loop
                           declare
                              Psi    : constant Real := 2.0 * Pi * Real (Step) / Real (Steps);
                              X      : Vec3;
                              Stops  : Fractions;
                              Hits   : Pad_Hits;
                              Result : Outcome;
                           begin
                              Tried.Placements := Tried.Placements + 1;
                              Contacts (K, I, Pin_Rotation (K, I, Psi), Disc, X, Stops, Hits, Result);
                              case Result is
                                 when On_Air    => Tried.Close_On_Air := Tried.Close_On_Air + 1;
                                 when Not_First => Tried.Into_Material := Tried.Into_Material + 1;
                                 when Touching  =>
                                    declare
                                       T   : Touch_Vectors.Vector;
                                       P   : Index_Vectors.Vector;
                                       Pin : constant Pin_Record :=
                                         (Pad_Index => K, Sample => I, Psi => Psi, Free_Turn => Steps = 1);
                                    begin
                                       Touches_Of (Hits, T, P);
                                       declare
                                          Key : constant String := Key_Of (T, P);
                                          Pos : constant Group_Maps.Cursor := Groups.Find (Key);
                                       begin
                                          if Group_Maps.Has_Element (Pos) then
                                             declare
                                                G : Group := Group_Maps.Element (Pos);
                                             begin
                                                G.Pins.Append (Pin);
                                                Groups.Replace_Element (Pos, G);
                                             end;
                                          else
                                             declare
                                                G : Group := (Touches => T, Pads => P, others => <>);
                                             begin
                                                G.Pins.Append (Pin);
                                                Groups.Insert (Key, G);
                                             end;
                                          end if;
                                       end;
                                    end;
                              end case;
                           end;
                        end loop;
                     end if;
                  end loop;
               end;
            end if;
         end loop;
      end Enumerate;

      --  A pad's box at fraction F with the tool at (R, X): its face, the way
      --  it faces, across it, and back along the lobe toward the hand.
      type Box is record
         Face, Facing, Side, Back      : Vec3;
         Half_Width, Thickness, Length : Real;
      end record;

      function Box_Of (P : Pad; R : Mat3; X : Vec3; F, Extra : Real) return Box is
         Facing : constant Vec3 := R * P.Facing;
         Back   : constant Vec3 := -(R * E.Along);
      begin
         return (Face => X + R * (P.Open + F * (P.Closed - P.Open)), Facing => Facing,
                 Side => Unit (Cross (Back, Facing)), Back => Back,
                 Half_Width => P.Half_Width, Thickness => P.Thickness, Length => E.Depth + Extra);
      end Box_Of;

      --  Inside the box grown by Margin on every side: behind the face by its
      --  thickness, across by half its width, back along the lobe by Length.
      function Inside (B : Box; Q : Vec3; Margin : Real) return Boolean is
         D : constant Vec3 := Q - B.Face;
         A : constant Real := D * B.Facing;
         C : constant Real := D * B.Side;
         L : constant Real := D * B.Back;
      begin
         return A <= Margin and then A >= -B.Thickness - Margin and then abs C <= B.Half_Width + Margin
           and then L >= -Margin and then L <= B.Length + Margin;
      end Inside;

      function Below_Floor (P : Vec3) return Boolean is
        (abs Thing.Floor_Up > 0.0 and then (P - Thing.Floor_Point) * Thing.Floor_Up < -Z * Thing.Floor_Sigma);

      function Below_Floor (B : Box) return Boolean is
      begin
         for I in 0 .. 1 loop
            for J in -1 .. 1 loop
               for K in 0 .. 1 loop
                  if J /= 0 and then Below_Floor (B.Face - Real (I) * B.Thickness * B.Facing
                                                  + Real (J) * B.Half_Width * B.Side + Real (K) * B.Length * B.Back)
                  then
                     return True;
                  end if;
               end loop;
            end loop;
         end loop;
         return False;
      end Below_Floor;

      type Verdict is (Fits, Into_Material, Too_Deep, Through_Surface, Through_Others);

      --  The hard conditions of one placement: closers at Before on the way
      --  in, the hand coming in along its lobes from where every lobe's end
      --  has left the thing behind, Z sigma further.
      function Judge (R : Mat3; X : Vec3; Before, At_Touch : Fractions; Travel : out Real) return Verdict is
         Back  : constant Vec3 := -(R * E.Along);
         Reach : Real := 0.0;
         Mid   : Vec3 := Zero3;
         Span  : Real := 0.0;
      begin
         for P of E.Pads loop
            declare
               B : constant Box := Box_Of (P, R, X, Before (P.Closer), 0.0);
            begin
               for M in 1 .. N_Samples loop
                  Reach := Real'Max (Reach, (Pts (M) - B.Face) * Back);
               end loop;
            end;
            Mid := Mid + Box_Of (P, R, X, At_Touch (P.Closer), 0.0).Face / Real (N_Pads);
         end loop;
         Travel := Reach + Z * Sig;
         for P of E.Pads loop
            declare
               B : constant Box := Box_Of (P, R, X, Before (P.Closer), Travel);
            begin
               if Below_Floor (B) then
                  return Through_Surface;
               end if;
               for M in 1 .. N_Samples loop
                  if Inside (B, Pts (M), Z * Sig) then
                     return Into_Material;
                  end if;
               end loop;
               for Q of Obs loop
                  if Inside (B, Q, Z * Sig) then
                     return Through_Others;
                  end if;
               end loop;
            end;
         end loop;
         --  Nothing of the thing goes further between the lobes than the
         --  hand's depth.
         for P of E.Pads loop
            declare
               F : constant Vec3 := Box_Of (P, R, X, At_Touch (P.Closer), 0.0).Face - Mid;
            begin
               Span := Real'Max (Span, abs (F - Real'(F * Back) * Back) + P.Half_Width);
            end;
         end loop;
         for M in 1 .. N_Samples loop
            declare
               D     : constant Vec3 := Pts (M) - Mid;
               Along : constant Real := D * Back;
            begin
               if Along > E.Depth and then abs (D - Along * Back) <= Span then
                  return Too_Deep;
               end if;
            end;
         end loop;
         return Fits;
      end Judge;

      function Turn_From_Now (R : Mat3) return Real is (Angle (R * Transpose (Now_R)));

      --  Second pass for a closing set: each pin of it, at its own turn or,
      --  when the set does not depend on the turn, at every turn in steps of
      --  one pitch at the whole faces' lever, least turning first; whole
      --  faces must make the same touches, the way in must be clear and both
      --  poses reachable.
      procedure Realize (G : Group; Out_C : out Candidate; Ok : out Boolean) is
      begin
         Ok := False;
         Out_C := (others => <>);
         for Pin of G.Pins loop
            declare
               Steps : constant Positive :=
                 (if Pin.Free_Turn then Steps_For (Lever (Pin.Pad_Index, E.Depth)) else 1);
            begin
               for Step in 0 .. Steps - 1 loop
                  declare
                     --  0, +1, -1, +2, -2, ... steps: the least turn first.
                     Half  : constant Natural := (Step + 1) / 2;
                     Sign  : constant Real := (if Step mod 2 = 1 then 1.0 else -1.0);
                     Psi   : constant Real :=
                       (if Pin.Free_Turn then Sign * 2.0 * Pi * Real (Half) / Real (Steps) else Pin.Psi);
                     R     : constant Mat3 := Pin_Rotation (Pin.Pad_Index, Pin.Sample, Psi);
                     X     : Vec3;
                     Stops : Fractions;
                     Hits  : Pad_Hits;
                     Result : Outcome;
                  begin
                     Contacts (Pin.Pad_Index, Pin.Sample, R, Whole_Face, X, Stops, Hits, Result);
                     if Result = Touching
                       and then (for all J in 1 .. Natural (G.Pads.Length) =>
                                   Hits (G.Pads (J)) /= 0 and then abs (Pts (Hits (G.Pads (J))) - G.Touches (J).Point) <= Res)
                     then
                        declare
                           Before : Fractions;
                           Travel : Real;
                           V      : Verdict;
                        begin
                           --  On the way in each closer that touches is open far
                           --  enough that its slowest face stays Clear of where it
                           --  will touch.
                           for C in 1 .. N_Closers loop
                              declare
                                 Slow : Real := Real'Last;
                              begin
                                 for P of E.Pads loop
                                    if P.Closer = C and then Moves (P) then
                                       Slow := Real'Min (Slow, abs (P.Closed - P.Open));
                                    end if;
                                 end loop;
                                 Before (C) := (if Slow = Real'Last or else Stops (C) = E.Closers (C).Now then Stops (C)
                                                else Real'Max (0.0, Stops (C) - Clear / Slow));
                              end;
                           end loop;
                           V := Judge (R, X, Before, Stops, Travel);
                           case V is
                              when Fits =>
                                 declare
                                    Tool  : constant Rigid := (Rotation => R, Translation => X);
                                    Hover : constant Rigid := (Rotation => R, Translation => X - Travel * (R * E.Along));
                                 begin
                                    if Reachable (Tool) and then Reachable (Hover) then
                                       Out_C := (Tool => Tool, Hover => Hover, Before => To_Vector (Before),
                                                 At_Touch => To_Vector (Stops), Touches => G.Touches,
                                                 Mu_Nominal => G.Mu_Nominal, Mu_Worst => G.Mu_Worst,
                                                 Force => G.Worst);
                                       Ok := True;
                                       return;
                                    end if;
                                 end;
                              when Into_Material   => Tried.Into_Material := Tried.Into_Material + 1;
                              when Too_Deep        => Tried.Too_Deep := Tried.Too_Deep + 1;
                              when Through_Surface => Tried.Through_Surface := Tried.Through_Surface + 1;
                              when Through_Others  => Tried.Through_Others := Tried.Through_Others + 1;
                           end case;
                        end;
                     end if;
                  end;
               end loop;
            end;
         end loop;
      end Realize;

      --  A part of the body (a lobe's end or the arm's surface, tool frame)
      --  brought onto sample I against its normal, the closers as they are:
      --  the first turn about that normal, least first, at which no other
      --  part of the body is inside the thing, near another thing or below
      --  the floor, and both poses are reachable.
      procedure Fit_Single (G : Group; Out_C : out Candidate; Ok : out Boolean) is
         I     : constant Positive := G.Sample;
         N     : constant Vec3 := Nrm (I);
         Parts : Sample_Vectors.Vector;
         --  Behind the nearest sample by more than the uncertainty: in the
         --  material. Outside the thing's box grown by that much: clear.
         function Penetrates (W : Vec3) return Boolean is
            Nearest : Natural := 0;
            Best_D  : Real := Real'Last;
         begin
            for C in 1 .. 3 loop
               if W (C) < Low_Corner (C) - Z * Sig or else W (C) > High_Corner (C) + Z * Sig then
                  return False;
               end if;
            end loop;
            for M in 1 .. N_Samples loop
               if Has_N (M) and then abs (Pts (M) - W) < Best_D then
                  Best_D := abs (Pts (M) - W);
                  Nearest := M;
               end if;
            end loop;
            return Nearest /= 0 and then (W - Pts (Nearest)) * Nrm (Nearest) < -Z * Sig;
         end Penetrates;
      begin
         Ok := False;
         Out_C := (others => <>);
         for Q of E.Ends loop
            Parts.Append (Q);
         end loop;
         for Q of E.Surface loop
            Parts.Append (Q);
         end loop;
         for Part of Parts loop
            if abs Part.Normal > 0.0 then
               declare
                  M_Part : constant Vec3 := Unit (Part.Normal);
                  R0     : constant Mat3 := Align (Now_R * M_Part, -N) * Now_R;
                  Extent : Real := Res;
               begin
                  for Other of Parts loop
                     declare
                        D : constant Vec3 := Other.Point - Part.Point;
                     begin
                        Extent := Real'Max (Extent, abs (D - Real'(D * M_Part) * M_Part));
                     end;
                  end loop;
                  declare
                     Steps : constant Positive := Steps_For (Extent);
                  begin
                     for Step in 0 .. Steps - 1 loop
                        declare
                           Half  : constant Natural := (Step + 1) / 2;
                           Sign  : constant Real := (if Step mod 2 = 1 then 1.0 else -1.0);
                           R     : constant Mat3 := Exp ((Sign * 2.0 * Pi * Real (Half) / Real (Steps)) * N) * R0;
                           X     : constant Vec3 := Pts (I) - R * Part.Point;
                           Clean : Boolean := True;
                        begin
                           for Other of Parts loop
                              declare
                                 W : constant Vec3 := X + R * Other.Point;
                              begin
                                 if Below_Floor (W) or else Penetrates (W)
                                   or else (for some Q of Obs => abs (Q - W) <= Z * Sig)
                                 then
                                    Clean := False;
                                 end if;
                              end;
                              exit when not Clean;
                           end loop;
                           if Clean then
                              declare
                                 Tool  : constant Rigid := (Rotation => R, Translation => X);
                                 Hover : constant Rigid := (Rotation => R, Translation => X + (Clear + Z * Sig) * N);
                              begin
                                 if Reachable (Tool) and then Reachable (Hover) then
                                    Out_C := (Tool => Tool, Hover => Hover, Before => Real_Vectors.Empty_Vector,
                                              At_Touch => Real_Vectors.Empty_Vector, Touches => G.Touches,
                                              Mu_Nominal => G.Mu_Nominal, Mu_Worst => G.Mu_Worst, Force => G.Worst);
                                    for C of E.Closers loop
                                       Out_C.Before.Append (C.Now);
                                       Out_C.At_Touch.Append (C.Now);
                                    end loop;
                                    Ok := True;
                                    return;
                                 end if;
                              end;
                           end if;
                        end;
                     end loop;
                  end;
               end;
            end if;
         end loop;
      end Fit_Single;

      --  The touches with every inward normal turned Z sigma about two axes
      --  across the set, all the same way: the worst of these is a case the
      --  measurement cannot exclude.
      type Touch_Set_Array is array (1 .. 4) of Touch_Vectors.Vector;

      function Tilted (T : Touch_Vectors.Vector) return Touch_Set_Array is
         Axis   : constant Vec3 :=
           (if Natural (T.Length) >= 2 and then abs (T (2).Point - T (1).Point) > 0.0
            then Unit (T (2).Point - T (1).Point) else Unit (T (1).Inward));
         Theta  : constant Real := (if Thing.Normal_Sigma < Real'Last then Z * Thing.Normal_Sigma else 0.0);
         A1, A2 : Vec3;
         Sets   : Touch_Set_Array;
      begin
         Plane_Basis (Axis, A1, A2);
         for V in Sets'Range loop
            declare
               By  : constant Vec3 :=
                 (case V is when 1 => Theta * A1, when 2 => -Theta * A1, when 3 => Theta * A2, when 4 => -Theta * A2);
               Rot : constant Mat3 := Exp (By);
            begin
               for X of T loop
                  Sets (V).Append (Touch'(Point => X.Point, Inward => Rot * X.Inward, Patch => X.Patch,
                                          Tension => X.Tension));
               end loop;
            end;
         end loop;
         return Sets;
      end Tilted;

      Resolution : constant Real := (if Thing.Normal_Sigma < Real'Last then Thing.Normal_Sigma else 0.0);

      function Least (T : Touch_Vectors.Vector; Below : Real) return Real is
        (Physics.Least_Friction (T, Thing.Base, Motion, Thing.Centre.Mean, U, Resolution, Below));

      --  The least friction with the normals tilted the worst way, from the
      --  nominal one already found; No_Way when it is not below Below.
      function Worst_Mu (G : Group; Below : Real) return Real is
         M : Real := G.Mu_Nominal;
      begin
         for T of Tilted (G.Touches) loop
            exit when M >= Below;
            M := Real'Max (M, Least (T, Below));
         end loop;
         return (if M < Below then M else Physics.No_Way);
      end Worst_Mu;

      --  How nearly two of its touches press against each other: the least
      --  cosine between two inward normals; a single touch comes after all.
      function Opposition (G : Group) return Real is
         C : Real := Real'Last;
      begin
         for A in 1 .. Natural (G.Touches.Length) loop
            for B in A + 1 .. Natural (G.Touches.Length) loop
               C := Real'Min (C, G.Touches (A).Inward * G.Touches (B).Inward);
            end loop;
         end loop;
         return C;
      end Opposition;

      function Worst_Force (G : Group; Mu : Real) return Real is
         F : Real := Physics.Need (G.Touches, Thing.Base, Motion, Thing.Centre.Mean, U, Mu).Force;
      begin
         for T of Tilted (G.Touches) loop
            F := Real'Max (F, Physics.Need (T, Thing.Base, Motion, Thing.Centre.Mean, U, Mu).Force);
         end loop;
         return F;
      end Worst_Force;

   begin
      Best := (others => <>);
      Found := False;
      Tried := (others => <>);
      if N_Samples = 0 or else not (Res > 0.0) or else Sig = Real'Last or else not Known (Thing.Centre) then
         return;
      end if;
      Low_Corner := Thing.Samples (1).Point;
      High_Corner := Low_Corner;
      for M in 1 .. N_Samples loop
         Pts (M) := Thing.Samples (M).Point;
         Has_N (M) := abs Thing.Samples (M).Normal > 0.0;
         Nrm (M) := (if Has_N (M) then Unit (Thing.Samples (M).Normal) else Zero3);
         for C in 1 .. 3 loop
            Low_Corner (C) := Real'Min (Low_Corner (C), Pts (M) (C));
            High_Corner (C) := Real'Max (High_Corner (C), Pts (M) (C));
         end loop;
      end loop;
      for M in Obs'Range loop
         Obs (M) := Beside (M);
      end loop;
      if E.Closes then
         Enumerate;
      end if;
      --  Single touches: one group per sample; the physics needs only the
      --  touch, and the placement is fitted once the touch is ranked.
      for I in 1 .. N_Samples loop
         if Has_N (I) then
            declare
               T : Touch_Vectors.Vector;
               P : Index_Vectors.Vector;
               G : Group;
            begin
               T.Append (Touch'(Point => Pts (I), Inward => -Nrm (I), Patch => 0.0, Tension => False));
               G := (Touches => T, Pads => P, Single => True, Sample => I, others => <>);
               Groups.Include ("single" & Integer'Image (I), G);
            end;
         end if;
      end loop;
      Tried.Distinct := Natural (Groups.Length);
      declare
         type Name_Array is array (Positive range <>) of Unbounded_String;
         Names     : Name_Array (1 .. Natural (Groups.Length));
         K         : Natural := 0;
         Reference : Real := Physics.No_Way;

         function At_Name (I : Positive) return Group is (Groups (To_String (Names (I))));

         procedure Store (I : Positive; G : Group) is
         begin
            Groups.Replace (To_String (Names (I)), G);
         end Store;

         --  Insertion sort on keys computed once; equal keys keep their order.
         procedure Sort_By (Key : not null access function (G : Group) return Real) is
            Keys : array (Names'Range) of Real;
         begin
            for I in Names'Range loop
               Keys (I) := Key (At_Name (I));
            end loop;
            for I in Names'First + 1 .. Names'Last loop
               declare
                  N : constant Unbounded_String := Names (I);
                  V : constant Real := Keys (I);
                  J : Natural := I;
               begin
                  while J > Names'First and then Keys (J - 1) > V loop
                     Names (J) := Names (J - 1);
                     Keys (J) := Keys (J - 1);
                     J := J - 1;
                  end loop;
                  Names (J) := N;
                  Keys (J) := V;
               end;
            end loop;
         end Sort_By;

         function Nominal_Force (G : Group) return Real is (G.Force);
      begin
         declare
            use type Physics.Obstacle;
            None : Touch_Vectors.Vector;
         begin
            if Physics.Need (None, Thing.Base, Motion, Thing.Centre.Mean, U, 0.0).Why = Physics.Footing_In_Way then
               Tried.Surface_In_Way := True;
               return;
            end if;
         end;
         for C in Groups.Iterate loop
            K := K + 1;
            Names (K) := To_Unbounded_String (Group_Maps.Key (C));
         end loop;
         --  The least worst-case friction of any set. A set that cannot do it
         --  at the least found so far cannot lower it, which one program
         --  shows; only the others are searched, below that bound. Sets whose
         --  touches oppose most come first, which only makes the bound fall
         --  sooner.
         Sort_By (Opposition'Access);
         for I in Names'Range loop
            declare
               G : Group := At_Name (I);
            begin
               if Reference = Physics.No_Way
                 or else Physics.Need (G.Touches, Thing.Base, Motion, Thing.Centre.Mean, U, Reference).Force
                         < Physics.No_Way
               then
                  G.Mu_Nominal := Least (G.Touches, Reference);
                  if G.Mu_Nominal < Reference then
                     declare
                        W : constant Real := Worst_Mu (G, Reference);
                     begin
                        if W < Physics.No_Way then
                           G.Mu_Worst := W;
                           Reference := W;
                        end if;
                     end;
                  end if;
                  Store (I, G);
               end if;
            end;
         end loop;
         if Reference = Physics.No_Way then
            Tried.Cannot_Balance := Tried.Distinct;
            return;
         end if;
         --  The thing is taken to give the least friction under which it can
         --  be done at all, or what it is known to give if that is more.
         Reference := Real'Max (Reference, Friction.Low);
         Tried.Reference_Mu := Reference;
         for I in Names'Range loop
            declare
               G : Group := At_Name (I);
            begin
               G.Force := Physics.Need (G.Touches, Thing.Base, Motion, Thing.Centre.Mean, U, Reference).Force;
               if G.Force = Physics.No_Way then
                  Tried.Cannot_Balance := Tried.Cannot_Balance + 1;
               end if;
               Store (I, G);
            end;
         end loop;
         Sort_By (Nominal_Force'Access);
         declare
            Done : array (Names'Range) of Boolean := [others => False];
            Next : Positive := Names'First;   --  the first set whose worst case is not computed
         begin
            loop
               declare
                  Pick : Natural := 0;
               begin
                  --  Worst cases are computed in order of the nominal force
                  --  until no later set could beat the best one found.
                  loop
                     Pick := 0;
                     for I in Names'First .. Next - 1 loop
                        if not Done (I) and then At_Name (I).Worst < Physics.No_Way
                          and then (Pick = 0 or else At_Name (I).Worst < At_Name (Pick).Worst)
                        then
                           Pick := I;
                        end if;
                     end loop;
                     exit when Next > Names'Last or else At_Name (Next).Force = Physics.No_Way
                       or else (Pick /= 0 and then At_Name (Pick).Worst <= At_Name (Next).Force);
                     declare
                        G : Group := At_Name (Next);
                     begin
                        if G.Mu_Worst < 0.0 then
                           G.Mu_Worst := Worst_Mu (G);
                        end if;
                        G.Worst := Worst_Force (G, Reference);
                        if G.Worst = Physics.No_Way then
                           Tried.Cannot_Balance := Tried.Cannot_Balance + 1;
                        end if;
                        Store (Next, G);
                     end;
                     Next := Next + 1;
                  end loop;
                  exit when Pick = 0;
                  Done (Pick) := True;
                  declare
                     G  : constant Group := At_Name (Pick);
                     Ok : Boolean := False;
                  begin
                     if G.Mu_Worst >= Friction.High then
                        Tried.Over_Bound := Tried.Over_Bound + 1;
                     else
                        if G.Single then
                           Fit_Single (G, Best, Ok);
                        else
                           Realize (G, Best, Ok);
                        end if;
                        if Ok then
                           Found := True;
                           return;
                        end if;
                        Tried.Unreachable := Tried.Unreachable + 1;
                     end if;
                  end;
               end;
            end loop;
         end;
      end;
   end Find;

end Driver.Action.Contact.Search;
