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
                  Result (N) := S.Ray;
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

   function Tip (B : Book; Lobe : Positive; At_Opening : Opening) return Point_Estimate is
      Unmeasured : Point_Estimate;
   begin
      if B.Sights.Is_Empty or else Lobe > Lobes (B) or else not Fit_Ok (B) then
         return Unmeasured;
      end if;
      declare
         Index : constant Natural := Index_Of (Sights_Of (B), Lobe, At_Opening);
      begin
         if Index = 0 or else not B.Fitted.Element.Tips (Index).Ok then
            return Unmeasured;
         end if;
         return B.Fitted.Element.Tips (Index).Tip;
      end;
   end Tip;

   function Confirmed (B : Book; Lobe : Positive; At_Opening : Opening) return Boolean is
   begin
      if B.Sights.Is_Empty or else Lobe > Lobes (B) or else not Fit_Ok (B) then
         return False;
      end if;
      declare
         Index : constant Natural := Index_Of (Sights_Of (B), Lobe, At_Opening);
      begin
         return Index > 0 and then B.Fitted.Element.Tips (Index).Ok and then B.Fitted.Element.Tips (Index).Confirmed;
      end;
   end Confirmed;

   function Distance (B : Book; Lobe : Positive; At_Opening : Opening) return Estimate is
   begin
      if B.Sights.Is_Empty or else Lobe > Lobes (B) or else not Fit_Ok (B) then
         return Unknown;
      end if;
      declare
         Index : constant Natural := Index_Of (Sights_Of (B), Lobe, At_Opening);
      begin
         return (if Index > 0 and then B.Fitted.Element.Tips (Index).Ok
                 then B.Fitted.Element.Tips (Index).Distance else Unknown);
      end;
   end Distance;

   function Into_Surface (B : Book; K : Kept) return Vec3 is
     (-(Transpose (K.Event.Tool.Pose.Rotation) * Surface (B).Normal));
   --  The direction into the fitted surface at a press, tool frame.

   function Agreeing (B : Book; Lobe : Positive; At_Opening : Opening) return Natural is
      N : Natural := 0;
   begin
      for K of B.Kept loop
         if K.Agrees and then K.Lobe = Lobe and then K.Opening = At_Opening then
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

   procedure Refit (B : in out Book);
   --  Fits every press given to a lobe whose line of sight at its opening is known.

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
                  Presses (N) := (Tool => K.Event.Tool, Sight => Index_Of (T, K.Lobe, K.Opening), Surface => 1);
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
                  B.Kept.Replace_Element (I, K);
               end;
            end loop;
            if F.Ok then
               for P in 1 .. Count loop
                  declare
                     K : Kept := B.Kept (Of_Kept (P));
                  begin
                     K.Agrees := F.Agrees (P);
                     B.Kept.Replace_Element (Of_Kept (P), K);
                  end;
               end loop;
            end if;
         end;
         Free (Presses);
         Free (Of_Kept);
      end;
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

   procedure Add (B : in out Book; Press : Driver.Robot.Hand.Presses.Event; At_Opening : Opening) is
   begin
      B.Kept.Append (Kept'(Event => Press, Opening => At_Opening, Lobe => 0, Agrees => False));
      Settle (B);
   end Add;

end Driver.Robot.Hand.Tips;
