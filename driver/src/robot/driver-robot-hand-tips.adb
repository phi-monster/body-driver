with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;

package body Driver.Robot.Hand.Tips is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   --  Everything sized by presses lives on the heap: the estimates also run
   --  in the decider's task, whose stack is small.
   type Index_Array is array (Positive range <>) of Positive;
   type Index_Access is access Index_Array;
   type Presses_Access is access Driver.Robot.Hand.Touch.Press_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Index_Array, Index_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Driver.Robot.Hand.Touch.Press_Array, Presses_Access);

   function Lobes (B : Book) return Natural is (if B.Sights.Is_Empty then 0 else B.Sights.Element'Length);

   function Pressed (B : Book) return Natural is (Natural (B.Kept.Length));

   function Sights_Of (B : Book) return Sight_Table is (B.Sights.Element);

   --  The known lines of sight in order, lobe by lobe, open before closed;
   --  the fit numbers them so.
   function Index_Of (T : Sight_Table; Lobe : Positive; At_Opening : Opening) return Natural is
      N : Natural := 0;
   begin
      for L in T'Range loop
         for O in Opening loop
            if T (L) (O).Known then
               N := N + 1;
               if L = Lobe and then O = At_Opening then
                  return N;
               end if;
            end if;
         end loop;
      end loop;
      return 0;
   end Index_Of;

   function Known_Sights (T : Sight_Table) return Driver.Robot.Hand.Touch.Sight_Array is
      Count : Natural := 0;
   begin
      for Row of T loop
         for S of Row loop
            Count := Count + Boolean'Pos (S.Known);
         end loop;
      end loop;
      declare
         Result : Driver.Robot.Hand.Touch.Sight_Array (1 .. Count);
         N      : Natural := 0;
      begin
         for Row of T loop
            for S of Row loop
               if S.Known then
                  N := N + 1;
                  --  The prior of the angles across the line: the eye's and the lobe's tip region's, together.
                  Result (N) := S.Ray;
                  Result (N).Direction.Sigma := Sqrt (S.Ray.Direction.Sigma ** 2 + S.Spread ** 2);
               end if;
            end loop;
         end loop;
         return Result;
      end;
   end Known_Sights;

   function Fit_Ok (B : Book) return Boolean is (not B.Fitted.Is_Empty and then B.Fitted.Element.Ok);

   function Surface (B : Book) return Driver.Geometry.Plane_Estimate is
      Unmeasured : Driver.Geometry.Plane_Estimate;
   begin
      return (if Fit_Ok (B) then B.Fitted.Element.Planes (1) else Unmeasured);
   end Surface;

   --  The index of the tip among the fit's tips of a kind, when that fit has it: the loaded fit for every tip a
   --  press rests on, the free fit for those the presses whose slide is measured rest on; 0 otherwise.
   function Fitted_Index (B : Book; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind) return Natural is
   begin
      if B.Sights.Is_Empty or else Lobe > Lobes (B) or else not Fit_Ok (B) then
         return 0;
      end if;
      declare
         Index : constant Natural := Index_Of (Sights_Of (B), Lobe, At_Opening);
      begin
         if Index = 0 then
            return 0;
         end if;
         case Kind is
            when Loaded =>
               return (if B.Fitted.Element.Tips (Index).Ok then Index else 0);
            when Free =>
               return (if not B.Free.Is_Empty and then B.Free.Element.Ok and then B.Free.Element.Tips (Index).Ok
                       then Index else 0);
         end case;
      end;
   end Fitted_Index;

   function Fitted_Tip (B : Book; Index : Positive; Kind : Tip_Kind) return Driver.Robot.Hand.Touch.Tip_Fit is
     (if Kind = Loaded then B.Fitted.Element.Tips (Index) else B.Free.Element.Tips (Index));

   function Tip (B : Book; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Point_Estimate is
      Unmeasured : Point_Estimate;
      Index      : constant Natural := Fitted_Index (B, Lobe, At_Opening, Kind);
   begin
      if Index = 0 then
         return Unmeasured;
      end if;
      --  The contact is somewhere in the lobe's tip region, which across the line of sight the eye does not see the
      --  depth of: the fit took that region's spread for the prior of the tip's angles across the line, and leaves
      --  it as far as the presses' tilts do not tell.
      return Fitted_Tip (B, Index, Kind).Tip;
   end Tip;

   function Beat (B : Book; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Driver.Clock.Beat is
      Result : Driver.Clock.Beat := 0;
      Lowest : Real := Real'Last;
   begin
      if not Known (Tip (B, Lobe, At_Opening, Kind)) then
         return 0;
      end if;
      for K of B.Kept loop
         declare
            Rests : constant Boolean := (if Kind = Loaded then K.Agrees else K.Agrees_Free);
            Hit   : constant Real := (if Kind = Loaded then K.Hit else K.Hit_Free);
         begin
            if Rests and then K.Lobe = Lobe and then K.Opening = At_Opening and then Hit < Lowest then
               Lowest := Hit;
               Result := K.Event.Beat;
            end if;
         end;
      end loop;
      return Result;
   end Beat;

   function Tested (B : Book; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Boolean is
      Index : constant Natural := Fitted_Index (B, Lobe, At_Opening, Kind);
   begin
      return Index > 0 and then Fitted_Tip (B, Index, Kind).Tested;
   end Tested;

   function Across (B : Book; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Real_Array is
      T : constant Point_Estimate := Tip (B, Lobe, At_Opening, Kind);
   begin
      if not Known (T) then
         return [0.0, 0.0];
      end if;
      declare
         U    : constant Vec3 := Sights_Of (B) (Lobe) (At_Opening).Ray.Direction.Unit_Vector;
         Axis : constant Vec3 :=
           (if abs U (1) <= abs U (2) and then abs U (1) <= abs U (3) then [1.0, 0.0, 0.0]
            elsif abs U (2) <= abs U (3) then [0.0, 1.0, 0.0] else [0.0, 0.0, 1.0]);
         E1   : constant Vec3 := Unit (Cross (U, Axis));
         E2   : constant Vec3 := Cross (U, E1);
         A    : constant Real := E1 * (T.Covariance * E1);
         C    : constant Real := E2 * (T.Covariance * E2);
         Off  : constant Real := E1 * (T.Covariance * E2);
         Mid  : constant Real := (A + C) / 2.0;
         Gap  : constant Real := Sqrt (((A - C) / 2.0) ** 2 + Off ** 2);
      begin
         return [Sqrt (Real'Max (Mid - Gap, 0.0)), Sqrt (Mid + Gap)];
      end;
   end Across;

   function Confirmed (B : Book; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Boolean is
      Index : constant Natural := Fitted_Index (B, Lobe, At_Opening, Kind);
   begin
      return Index > 0 and then Fitted_Tip (B, Index, Kind).Confirmed;
   end Confirmed;

   function Distance (B : Book; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Estimate is
      Index : constant Natural := Fitted_Index (B, Lobe, At_Opening, Kind);
   begin
      return (if Index > 0 then Fitted_Tip (B, Index, Kind).Distance else Unknown);
   end Distance;

   function Into_Surface (B : Book; K : Kept) return Vec3 is
     (-(Transpose (K.Event.Tool.Pose.Rotation) * Surface (B).Normal));
   --  The direction into the fitted surface at a press, tool frame.

   function Agreeing (B : Book; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Natural is
      N : Natural := 0;
   begin
      for K of B.Kept loop
         if (if Kind = Loaded then K.Agrees else K.Agrees_Free) and then K.Lobe = Lobe and then K.Opening = At_Opening then
            N := N + 1;
         end if;
      end loop;
      return N;
   end Agreeing;

   function Latest_Agrees (B : Book) return Boolean is
     (not B.Kept.Is_Empty and then B.Kept.Last_Element.Agrees);

   function Direction (B : Book; Lobe : Positive; At_Opening : Opening) return Direction_Estimate is
      Unmeasured : Direction_Estimate;
      Sum   : Vec3 := Zero3;
      Count : constant Natural := Agreeing (B, Lobe, At_Opening);
   begin
      if Count = 0 or else not Driver.Geometry.Known (Surface (B)) then
         return Unmeasured;
      end if;
      for K of B.Kept loop
         if K.Agrees and then K.Lobe = Lobe and then K.Opening = At_Opening then
            Sum := Sum + Into_Surface (B, K);
         end if;
      end loop;
      declare
         Mean   : constant Vec3 := Unit (Sum);
         Spread : Real := 0.0;
      begin
         --  Each press's angle from the mean: one number for both directions
         --  across it, so half the squared length of the part across.
         for K of B.Kept loop
            if K.Agrees and then K.Lobe = Lobe and then K.Opening = At_Opening then
               declare
                  Across : constant Vec3 := Cross (Into_Surface (B, K), Mean);
               begin
                  Spread := Spread + (Across * Across) / 2.0;
               end;
            end if;
         end loop;
         return (Unit_Vector => Mean, Sigma => Sqrt (Spread / Real (Count)));
      end;
   end Direction;

   function Prior_Of (B : Book) return Driver.Robot.Hand.Touch.Surface_Prior is
     (if Driver.Geometry.Known (B.Table) then (Measured => True, Plane => B.Table) else (Measured => False));
   --  The surface as measured before the presses, when it was.

   procedure Refit_Free (B : in out Book; T : Sight_Table);
   --  Fits the presses whose slide is measured again, each taken with the tip it had slid to, for the free finger.

   procedure Refit (B : in out Book);
   --  Fits every press given to a lobe whose line of sight at its opening is known.

   procedure Refit_Free (B : in out Book; T : Sight_Table) is
      --  The travel of a lobe between its two openings, tool frame: the closed tip less the open one, from the tips
      --  of a fit of a kind. A finger slides along it.
      function Travel_Of (Lobe : Positive; From : Tip_Kind) return Point_Estimate is
         Unmeasured : Point_Estimate;
         Open_Tip   : constant Point_Estimate := Tip (B, Lobe, Open, From);
         Closed_Tip : constant Point_Estimate := Tip (B, Lobe, Closed_Empty, From);
      begin
         if not Known (Open_Tip) or else not Known (Closed_Tip) then
            return Unmeasured;
         end if;
         return (Mean => Closed_Tip.Mean - Open_Tip.Mean, Covariance => Closed_Tip.Covariance + Open_Tip.Covariance);
      end Travel_Of;

      --  Fits the free finger, the travel of a lobe taken from the tips of the fit of a kind.
      procedure Fit_Once (From : Tip_Kind) is
         Travels  : array (T'Range) of Point_Estimate;
         Count    : Natural := 0;
         function Eligible (K : Kept) return Boolean is
           (K.Lobe > 0 and then Index_Of (T, K.Lobe, K.Opening) > 0 and then Known (Travels (K.Lobe))
            and then K.Lobe <= Natural (K.Slides.Length) and then K.Slides (K.Lobe).Known);
      begin
         for L in T'Range loop
            Travels (L) := Travel_Of (L, From);
         end loop;
         for K of B.Kept loop
            if Eligible (K) then
               Count := Count + 1;
            end if;
         end loop;
         B.Free := Fit_Holders.Empty_Holder;
         for I in B.Kept.First_Index .. B.Kept.Last_Index loop
            declare
               K : Kept := B.Kept (I);
            begin
               K.Agrees_Free := False;
               K.Hit_Free := 0.0;
               B.Kept.Replace_Element (I, K);
            end;
         end loop;
         if Count = 0 then
            return;
         end if;
         declare
            Presses : Presses_Access := new Driver.Robot.Hand.Touch.Press_Array (1 .. Count);
            Of_Kept : Index_Access := new Index_Array (1 .. Count);
            N       : Natural := 0;
         begin
            for I in B.Kept.First_Index .. B.Kept.Last_Index loop
               declare
                  K : constant Kept := B.Kept (I);
               begin
                  if Eligible (K) then
                     declare
                        Slipped : constant Slid := K.Slides (K.Lobe);
                        Travel : constant Point_Estimate := Travels (K.Lobe);
                     begin
                        N := N + 1;
                        --  Inward is towards the closed opening: the finger stood Fraction of the travel from where it
                        --  stands free, and how well that is known is the fraction's and the travel's.
                        Presses (N) :=
                          (Tool             => K.Event.Tool,
                           Sight            => Index_Of (T, K.Lobe, K.Opening),
                           Surface          => 1,
                           Slide            => Slipped.Fraction * Travel.Mean,
                           Slide_Covariance => Slipped.Fraction_Sigma ** 2 * Outer (Travel.Mean, Travel.Mean)
                                               + Slipped.Fraction ** 2 * Travel.Covariance,
                           Slide_Angle      => 0.0);   --  a vector tells it
                        Of_Kept (N) := I;
                     end;
                  end if;
               end;
            end loop;
            declare
               F : constant Driver.Robot.Hand.Touch.Fit_Result :=
                 Driver.Robot.Hand.Touch.Fit (Presses.all, Known_Sights (T), [1 => Prior_Of (B)]);
            begin
               B.Free := Fit_Holders.To_Holder (F);
               if F.Ok then
                  for P in 1 .. Count loop
                     declare
                        K : Kept := B.Kept (Of_Kept (P));
                     begin
                        K.Agrees_Free := F.Agrees (P);
                        K.Hit_Free := F.Hits (P);
                        B.Kept.Replace_Element (Of_Kept (P), K);
                     end;
                  end loop;
               end if;
            end;
            Free (Presses);
            Free (Of_Kept);
         end;
      end Fit_Once;
   begin
      B.Free := Fit_Holders.Empty_Holder;
      if not Fit_Ok (B) then
         return;
      end if;
      Fit_Once (Loaded);
      --  The travel the slides are shares of is the free finger's, which differs from the loaded fingers' by
      --  the difference of their slides: once more, from the free tips.
      if not B.Free.Is_Empty and then B.Free.Element.Ok then
         Fit_Once (Free);
      end if;
   end Refit_Free;

   --  How far the lobe's finger stood from where its free pixel puts it under a press, as an angle seen from the
   --  eye: what was measured, with its sigma, and no more than the finger's whole travel; the whole travel when the
   --  press could not say (the finger was not found where it was looked for, or has no edge to look for). A press
   --  with no slides at all is one made where nothing was measured: nothing is taken to have slid.
   function Slid_Angle (K : Kept; Sight : Sight_Of) return Real is
   begin
      if K.Lobe > Natural (K.Slides.Length) then
         return 0.0;
      end if;
      declare
         S : constant Slid := K.Slides (K.Lobe);
      begin
         if not S.Known or else S.Pixels_Sigma >= Real'Last then
            return Sight.Travel;
         end if;
         return Real'Min (Sight.Travel, Sight.Pitch * Sqrt (S.Pixels ** 2 + S.Pixels_Sigma ** 2));
      end;
   end Slid_Angle;

   procedure Refit (B : in out Book) is
      T     : constant Sight_Table := Sights_Of (B);
      Count : Natural := 0;
   begin
      for K of B.Kept loop
         if K.Lobe > 0 and then Index_Of (T, K.Lobe, K.Opening) > 0 then
            Count := Count + 1;
         end if;
      end loop;
      declare
         Presses : Presses_Access := new Driver.Robot.Hand.Touch.Press_Array (1 .. Count);
         Of_Kept : Index_Access := new Index_Array (1 .. Count);
         N       : Natural := 0;
      begin
         for I in B.Kept.First_Index .. B.Kept.Last_Index loop
            declare
               K : constant Kept := B.Kept (I);
            begin
               if K.Lobe > 0 and then Index_Of (T, K.Lobe, K.Opening) > 0 then
                  N := N + 1;
                  Presses (N) := (Tool => K.Event.Tool, Sight => Index_Of (T, K.Lobe, K.Opening), Surface => 1,
                                  Slide_Angle => Slid_Angle (K, T (K.Lobe) (K.Opening)), others => <>);
                  Of_Kept (N) := I;
               end if;
            end;
         end loop;
         declare
            F : constant Driver.Robot.Hand.Touch.Fit_Result :=
              Driver.Robot.Hand.Touch.Fit (Presses.all, Known_Sights (T), [1 => Prior_Of (B)]);
         begin
            B.Fitted := Fit_Holders.To_Holder (F);
            for I in B.Kept.First_Index .. B.Kept.Last_Index loop
               declare
                  K : Kept := B.Kept (I);
               begin
                  K.Agrees := False;
                  K.Hit := 0.0;
                  K.Agrees_Free := False;
                  K.Hit_Free := 0.0;
                  B.Kept.Replace_Element (I, K);
               end;
            end loop;
            if F.Ok then
               for P in 1 .. Count loop
                  declare
                     K : Kept := B.Kept (Of_Kept (P));
                  begin
                     K.Agrees := F.Agrees (P);
                     K.Hit := F.Hits (P);
                     B.Kept.Replace_Element (Of_Kept (P), K);
                  end;
               end loop;
            end if;
         end;
         Free (Presses);
         Free (Of_Kept);
      end;
      Refit_Free (B, T);
   end Refit;

   function Leading (B : Book; At_Opening : Opening; Into : Vec3) return Natural is
      --  The lobe whose tip is foremost along Into, when every lobe seen at
      --  this opening has a fitted tip; else the lobe whose line of sight lies
      --  closest to Into.
      T       : constant Sight_Table := Sights_Of (B);
      By_Tips : Boolean := True;
      Best    : Natural := 0;
      Score   : Real := Real'First;
   begin
      for L in T'Range loop
         if T (L) (At_Opening).Known and then not Known (Tip (B, L, At_Opening)) then
            By_Tips := False;
         end if;
      end loop;
      for L in T'Range loop
         if T (L) (At_Opening).Known then
            declare
               Ahead : constant Real :=
                 (if By_Tips then Tip (B, L, At_Opening).Mean * Into
                  else T (L) (At_Opening).Ray.Direction.Unit_Vector * Into);
            begin
               if Ahead > Score then
                  Score := Ahead;
                  Best := L;
               end if;
            end;
         end if;
      end loop;
      return Best;
   end Leading;

   procedure Give_Out (B : in out Book; Changed : out Boolean);
   --  Gives every press to the lobe that leads into the surface, along the
   --  fitted surface's normal when there is one, else along the way the tool
   --  was pressing; a press with neither waits.

   procedure Give_Out (B : in out Book; Changed : out Boolean) is
      Fitted_Surface : constant Boolean := Driver.Geometry.Known (Surface (B));
   begin
      Changed := False;
      for I in B.Kept.First_Index .. B.Kept.Last_Index loop
         declare
            K    : Kept := B.Kept (I);
            Lobe : Natural := K.Lobe;
         begin
            if Fitted_Surface then
               Lobe := Leading (B, K.Opening, Into_Surface (B, K));
            elsif K.Event.Approach.Sigma < Real'Last then
               Lobe := Leading (B, K.Opening, K.Event.Approach.Unit_Vector);
            end if;
            if Lobe /= K.Lobe then
               K.Lobe := Lobe;
               B.Kept.Replace_Element (I, K);
               Changed := True;
            end if;
         end;
      end loop;
   end Give_Out;

   procedure Settle (B : in out Book);
   --  Gives out and refits until no press changes lobe; each round can move
   --  every press, so as many rounds as presses bound it.

   procedure Settle (B : in out Book) is
      Changed : Boolean := True;
      Rounds  : Natural := 0;
   begin
      while Changed and then Rounds < Natural (B.Kept.Length) loop
         Give_Out (B, Changed);
         Refit (B);
         Rounds := Rounds + 1;
      end loop;
   end Settle;

   procedure Set_Sights (B : in out Book; Sights : Sight_Table) is
   begin
      if Lobes (B) /= Sights'Length then
         B.Kept.Clear;
      end if;
      B.Sights := Table_Holders.To_Holder (Sights);
      B.Fitted := Fit_Holders.Empty_Holder;
      Settle (B);
   end Set_Sights;

   procedure Set_Frame
     (B       : in out Book;
      Surface : Driver.Geometry.Plane_Estimate;
      Pose_Of : not null access function (Arm : Real_Array) return Pose_Estimate;
      Moved   : out Boolean)
   is
      use type Driver.Geometry.Plane_Estimate;
   begin
      Moved := Surface /= B.Table;
      if not Moved then
         return;
      end if;
      B.Table := Surface;
      for I in B.Kept.First_Index .. B.Kept.Last_Index loop
         declare
            K : Kept := B.Kept (I);
         begin
            if not K.Event.Arm.Is_Empty then
               declare
                  Pose : constant Pose_Estimate := Pose_Of (K.Event.Arm.Element);
               begin
                  if Pose.Position_Covariance (1, 1) < Real'Last and then Pose.Rotation_Covariance (1, 1) < Real'Last then
                     K.Event.Tool := Pose;
                     B.Kept.Replace_Element (I, K);
                  end if;
               end;
            end if;
         end;
      end loop;
      Settle (B);
   end Set_Frame;

   procedure Add
     (B          : in out Book;
      Press      : Driver.Robot.Hand.Presses.Event;
      At_Opening : Opening;
      Slides     : Slid_Row := No_Slides)
   is
      Row : Slid_Vectors.Vector;
   begin
      for S of Slides loop
         Row.Append (S);
      end loop;
      B.Kept.Append (Kept'(Event => Press, Opening => At_Opening, Lobe => 0, Agrees => False, Hit => 0.0,
                           Agrees_Free => False, Hit_Free => 0.0, Slides => Row));
      Settle (B);
   end Add;

   function Slides_Of (B : Book; Lobe : Positive; At_Opening : Opening) return Press_Slides is
      Total : Natural := 0;
   begin
      for K of B.Kept loop
         if K.Opening = At_Opening then
            Total := Total + 1;
         end if;
      end loop;
      declare
         Result : Press_Slides (1 .. Total);
         Next   : Natural := 0;
      begin
         for K of B.Kept loop
            if K.Opening = At_Opening then
               Next := Next + 1;
               Result (Next) :=
                 (Beat    => K.Event.Beat,
                  Contact => K.Lobe = Lobe,
                  Agrees  => K.Agrees and then K.Lobe = Lobe,
                  Slid    => (if Lobe <= Natural (K.Slides.Length) then K.Slides (Lobe) else Slid'(others => <>)));
            end if;
         end loop;
         return Result;
      end;
   end Slides_Of;

end Driver.Robot.Hand.Tips;
