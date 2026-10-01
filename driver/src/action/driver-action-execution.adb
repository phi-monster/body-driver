with Ada.Numerics.Long_Elementary_Functions;
with Driver.Action.Contact;
with Driver.Action.Contact.Search;
with Driver.Action.Contact.Wrench;
with Driver.Action.Goals;
with Driver.Action.Monitor;
with Driver.Action.Snapshots;
with Driver.Conventions;
with Driver.Log;
with Driver.Numerics;
with Driver.Uncertain;

package body Driver.Action.Execution is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Driver.Uncertain;
   use Driver.Action.Snapshots;
   use Driver.Action.Plants;
   use type Arm_Id;
   use type Hand_Id;
   use type Thing_Id;
   use type Surface_Id;
   use type Driver.Action.Goals.Quantity;

   package Contact renames Driver.Action.Contact;
   package Search renames Driver.Action.Contact.Search;
   package Wrench renames Driver.Action.Contact.Wrench;

   Z : constant Real := Driver.Conventions.Z;

   function Img (X : Real) return String is (Driver.Log.Image (X, 3));
   function Img (N : Integer) return String is (Driver.Log.Image (N));

   type State is record
      S       : Snapshot;
      Account : Unbounded_String;
      Tried   : Unbounded_String;
   end record;
   --  What a run has seen last, what it did, and what it tried that failed.

   procedure Say (X : in out State; Line : String) is
   begin
      if Length (X.Account) > 0 then
         Append (X.Account, "; ");
      end if;
      Append (X.Account, Line);
   end Say;

   procedure Note_Tried (X : in out State; Line : String) is
   begin
      if Length (X.Tried) > 0 then
         Append (X.Tried, "; ");
      end if;
      Append (X.Tried, Line);
   end Note_Tried;

   procedure Look (P : in out Plant'Class; X : in out State) is
   begin
      P.Look (X.S);
   end Look;

   function Gravity (S : Snapshot) return Vec3 is
     (if S.Up.Sigma < Real'Last and then abs S.Up.Unit_Vector > 0.0 then Unit (S.Up.Unit_Vector) else Zero3);

   function Largest_Sigma (C : Mat3) return Real is
     (Sqrt (Real'Max (C (1, 1), Real'Max (C (2, 2), C (3, 3)))));

   --  The tool pose after a share S of a unit twist: the tool turns about the
   --  twist's axis with whatever it holds, then shifts.
   function Moved (G : Contact.Twist; S : Real; Tool : Rigid) return Rigid is
      R : constant Mat3 := Exp (S * G.Angular);
   begin
      return (Rotation => R * Tool.Rotation, Translation => R * (Tool.Translation - G.Pivot) + G.Pivot + S * G.Linear);
   end Moved;

   function One_Arm (A : Arm_Id; Tool : Rigid) return Order is
     ((Arms     => Arm_Goal_Vectors.To_Vector ((Arm => A, Tool => Tool, Position_Only => False), 1),
       Closers  => Closer_Goal_Vectors.Empty_Vector,
       Settle   => True));

   --  The smallest step the arm delivers distinguishably, for a twist: a
   --  length for a shift, an angle for a turn.
   function Resolution (A : Arm_State; G : Contact.Twist) return Real is
     (if abs G.Angular > 0.0 then A.Turn_Step.Value else A.Step.Value);

   --  The tool is at Goal as far as the arm can tell: nearer than its
   --  smallest step, or not significantly away given its own uncertainty.
   function At_Goal (A : Arm_State; Goal : Rigid) return Boolean is
      Off  : constant Real := abs (Goal.Translation - A.Tool.Pose.Translation);
      Turn : constant Real := Angle (Transpose (A.Tool.Pose.Rotation) * Goal.Rotation);
   begin
      return (Off <= A.Step.Value
              or else not Significant (Position (A.Tool), (Mean => Goal.Translation,
                                                           Covariance => [others => [others => 0.0]])))
        and then (Turn <= A.Turn_Step.Value
                  or else not Significant (Vector_Gate (3), Turn, Largest_Sigma (A.Tool.Rotation_Covariance)));
   end At_Goal;

   --  Brings the arm's tool to Goal, sending it again from wherever it got to
   --  until it is there as far as it can tell, the plant says it is, or a
   --  send moves it by nothing it can tell from noise.
   procedure Go (P : in out Plant'Class; X : in out State; A : Arm_Id; Goal : Rigid; Outcome : out Step_Outcome;
                 Why : out Unbounded_String)
   is
   begin
      Why := Null_Unbounded_String;
      loop
         Look (P, X);
         declare
            R : Report;
         begin
            if At_Goal (Arm (X.S, A), Goal) then
               Outcome := Reached;
               return;
            elsif P.Episode_Over then
               Outcome := Short;
               return;
            end if;
            P.Move (One_Arm (A, Goal), R);
            declare
               Res : constant Arm_Result := R.Arms.First_Element;
            begin
               case Res.Outcome is
                  when Refused =>
                     Outcome := Refused;
                     Why := Res.Why;
                     Look (P, X);
                     return;
                  when Blocked =>
                     Outcome := Blocked;
                     Look (P, X);
                     return;
                  when Reached =>
                     Outcome := Reached;
                     Look (P, X);
                     return;
                  when Short =>
                     if not (Res.Delivered.Value > 0.0
                             and then Significant (Res.Delivered.Value, Res.Delivered.Sigma,
                                                   Res.Delivered.Degrees_Of_Freedom))
                     then
                        Look (P, X);
                        Outcome := (if At_Goal (Arm (X.S, A), Goal) then Reached else Blocked);
                        return;
                     end if;
               end case;
            end;
         end;
      end loop;
   end Go;

   procedure Set_Closer (P : in out Plant'Class; X : in out State; H : Hand_Id; Fraction : Real;
                         Outcome : out Step_Outcome)
   is
      R : Report;
   begin
      P.Move ((Arms    => Arm_Goal_Vectors.Empty_Vector,
               Closers => Closer_Goal_Vectors.To_Vector ((Hand => H, Fraction => Fraction), 1),
               Settle  => True), R);
      Outcome := (if R.Closers.Is_Empty then Refused else R.Closers.First_Element.Outcome);
      Look (P, X);
   end Set_Closer;

   --  The measured parts of the arm that can meet something, in the world
   --  with the tool at Tool: the lobes' faces and backs back to the depth of
   --  the hand, their ends, and the arm's own surface.
   function Body_Points (E : Search.Effector; Tool : Rigid; Spacing : Real) return Contact.Point_Vectors.Vector is
      Pts : Contact.Point_Vectors.Vector;
   begin
      for Pd of E.Pads loop
         declare
            F  : constant Real := E.Closers (Pd.Closer).Now;
            C  : constant Vec3 := Pd.Open + F * (Pd.Closed - Pd.Open);
            Across : constant Vec3 := Cross (Pd.Facing, E.Along);
            Wd : constant Vec3 := (if abs Across > 0.0 then Unit (Across) else Zero3);
            NW : constant Positive := Positive'Max (1, Natural (Real'Ceiling (2.0 * Pd.Half_Width / Spacing)));
            NL : constant Positive := Positive'Max (1, Natural (Real'Ceiling (E.Depth / Spacing)));
         begin
            for I in 0 .. NW loop
               for J in 0 .. NL loop
                  declare
                     Q : constant Vec3 := C + (2.0 * Real (I) / Real (NW) - 1.0) * Pd.Half_Width * Wd
                                            - (E.Depth * Real (J) / Real (NL)) * E.Along;
                  begin
                     Pts.Append (Tool * Q);
                     Pts.Append (Tool * (Q - Pd.Thickness * Pd.Facing));
                  end;
               end loop;
            end loop;
         end;
      end loop;
      for En of E.Ends loop
         Pts.Append (Tool * En.Point);
      end loop;
      for Sf of E.Surface loop
         Pts.Append (Tool * Sf.Point);
      end loop;
      return Pts;
   end Body_Points;

   --  The least distance from any of the points to the surfaces and the
   --  things' samples (Except and Held left out), less the margin each needs:
   --  Z of the two sigmas together, and half a pitch between samples.
   function Least_Gap (S : Snapshot; Points : Contact.Point_Vectors.Vector; Sigma : Real; Except, Held : Thing_Id'Base)
     return Real
   is
      Least : Real := Real'Last;
   begin
      for F of S.Surfaces loop
         if F.Of_Thing = 0 and then abs F.Normal.Unit_Vector > 0.0 then
            declare
               N      : constant Vec3 := Unit (F.Normal.Unit_Vector);
               Margin : constant Real := Z * Sqrt (Sigma ** 2 + Largest_Sigma (F.Point.Covariance) ** 2);
            begin
               for Q of Points loop
                  Least := Real'Min (Least, Real'((Q - F.Point.Mean) * N) - Margin);
               end loop;
            end;
         end if;
      end loop;
      for T of S.Things loop
         if T.Id /= Except and then T.Id /= Held then
            declare
               Margin : constant Real := Z * Sqrt (Sigma ** 2 + T.Sigma ** 2) + T.Pitch / 2.0;
            begin
               for Q of Points loop
                  for Smp of T.Samples loop
                     Least := Real'Min (Least, abs (Q - Smp.Point) - Margin);
                  end loop;
               end loop;
            end;
         end if;
      end loop;
      return Least;
   end Least_Gap;

   --  Whether the arm's measured parts, and what it holds, moving straight
   --  from one tool pose to another, keep clear of the surfaces and of every
   --  other thing; coming no nearer than it was at the start always is.
   function Clear_Way (S : Snapshot; E : Search.Effector; From, To : Rigid; Except, Held : Thing_Id'Base)
     return Boolean
   is
      Spacing : Real := Real'Last;
      Lever   : Real := 0.0;
      Carried : Contact.Point_Vectors.Vector;
   begin
      for T of S.Things loop
         if T.Pitch > 0.0 then
            Spacing := Real'Min (Spacing, T.Pitch);
         end if;
      end loop;
      if Spacing = Real'Last then
         return True;
      end if;
      if Held /= 0 and then Has_Thing (S, Held) then
         for Smp of Thing (S, Held).Samples loop
            Carried.Append (Smp.Point);
         end loop;
      end if;
      declare
         function At_Pose (Tool : Rigid) return Contact.Point_Vectors.Vector is
            Pts   : Contact.Point_Vectors.Vector := Body_Points (E, Tool, Spacing);
            Carry : constant Rigid := Tool * Inverse (From);
         begin
            for C of Carried loop
               Pts.Append (Carry * C);
            end loop;
            return Pts;
         end At_Pose;
         Start_Gap : constant Real := Least_Gap (S, At_Pose (From), E.Sigma, Except, Held);
      begin
         for P of At_Pose (From) loop
            Lever := Real'Max (Lever, abs (P - From.Translation));
         end loop;
         declare
            Shift  : constant Real := abs (To.Translation - From.Translation);
            Turn   : constant Real := Angle (Transpose (From.Rotation) * To.Rotation);
            Pieces : constant Positive := Positive'Max (1, Natural (Real'Ceiling ((Shift + Turn * Lever) / Spacing)));
         begin
            for I in 1 .. Pieces loop
               declare
                  Share : constant Real := Real (I) / Real (Pieces);
                  Pose  : constant Rigid :=
                    (Rotation    => From.Rotation
                                      * Exp (Share * Driver.Numerics.Log (Transpose (From.Rotation) * To.Rotation)),
                     Translation => From.Translation + Share * (To.Translation - From.Translation));
                  Gap   : constant Real := Least_Gap (S, At_Pose (Pose), E.Sigma, Except, Held);
               begin
                  if Gap < 0.0 and then Gap < Start_Gap then
                     return False;
                  end if;
               end;
            end loop;
         end;
      end;
      return True;
   end Clear_Way;

   --  Takes the arm to Goal by the lowest clear way: straight when that is
   --  clear, else over the top, raised by its own reach doubled until the
   --  way over is clear; planned again from wherever each step got it.
   procedure Travel (P : in out Plant'Class; X : in out State; A : Arm_Id; Goal : Rigid; Except, Held : Thing_Id'Base;
                     Outcome : out Step_Outcome; Why : out Unbounded_String)
   is
   begin
      Why := Null_Unbounded_String;
      loop
         declare
            E    : constant Search.Effector := Search.Effector_Of (X.S, A);
            From : constant Rigid := E.Tool;
            Now  : constant Arm_State := Arm (X.S, A);
            Up   : constant Vec3 := Gravity (X.S);
         begin
            if At_Goal (Now, Goal) then
               Outcome := Reached;
               return;
            end if;
            if not (abs Up > 0.0) or else Clear_Way (X.S, E, From, Goal, Except, Held) then
               Go (P, X, A, Goal, Outcome, Why);
               return;
            end if;
            declare
               Lift : Real := E.Depth + E.Sigma;
               Via_1, Via_2 : Rigid;
               Found : Boolean := False;
            begin
               loop
                  Via_1 := (Rotation => From.Rotation, Translation => From.Translation + Lift * Up);
                  Via_2 := (Rotation => Goal.Rotation, Translation => Goal.Translation + Lift * Up);
                  exit when P.Reach ((Arm => A, Tool => Via_1, Position_Only => False)).Status /= Reachable
                    or else P.Reach ((Arm => A, Tool => Via_2, Position_Only => False)).Status /= Reachable;
                  if Clear_Way (X.S, E, From, Via_1, Except, Held)
                    and then Clear_Way (X.S, E, Via_1, Via_2, Except, Held)
                    and then Clear_Way (X.S, E, Via_2, Goal, Except, Held)
                  then
                     Found := True;
                     exit;
                  end if;
                  Lift := 2.0 * Lift;
               end loop;
               if not Found then
                  Outcome := Refused;
                  Why := To_Unbounded_String ("no clear way there: straight is blocked and every way over the top "
                                              & "up to " & Img (Lift) & " high is out of reach");
                  return;
               end if;
               --  The first leg not done yet.
               declare
                  Raised : constant Boolean :=
                    Real'((From.Translation - Via_1.Translation) * Up) >= -Now.Step.Value;
                  Over   : constant Boolean :=
                    abs (From.Translation - Via_2.Translation) <= Now.Step.Value;
                  Leg    : constant Rigid := (if Over then Goal elsif Raised then Via_2 else Via_1);
               begin
                  Go (P, X, A, Leg, Outcome, Why);
                  if Outcome in Refused | Blocked then
                     return;
                  end if;
               end;
            end;
         end;
      end loop;
   end Travel;

   --  Everything measured near the thing but itself and what holds it.
   function Beside_Of (S : Snapshot; T : Thing_Id) return Contact.Point_Vectors.Vector is
      B : Contact.Point_Vectors.Vector;
   begin
      for O of S.Things loop
         if O.Id /= T then
            for Smp of O.Samples loop
               B.Append (Smp.Point);
            end loop;
         end if;
      end loop;
      return B;
   end Beside_Of;

   function Busy (S : Snapshot; A : Arm_Id; T : Thing_Id) return Boolean is
     (for some O of S.Things => O.Id /= T and then O.Held_By /= 0 and then Has_Hand (S, O.Held_By)
                                and then Hand (S, O.Held_By).Arm = A);
   --  The arm holds something else.

   type Grip is record
      Arm     : Arm_Id := Arm_Id'First;
      Hands   : Search.Closer_Vectors.Vector;   --  the closers that close on it
      Closing : Boolean := False;               --  it is held between lobes, not touched by one part
      Searched : Boolean := False;              --  Chosen is the contact set this run picked
      Chosen  : Search.Candidate;
   end record;

   --  Brings a part of the body into the contact set the search picks for
   --  Motion, over every arm free to do it; on a closing set, closes on the
   --  thing. A closing that finds nothing between the lobes is tried again
   --  from a new look only if it changed something.
   procedure Acquire (P : in out Plant'Class; X : in out State; T : Thing_Id; Motion : Contact.Twist; G : out Grip;
                      Ok : out Boolean)
   is
   begin
      Ok := False;
      loop
         declare
            Best  : Search.Candidate;
            Force : Real := Real'Last;
            Arm_Of_Best : Arm_Id := Arm_Id'First;
            Before_Centre : constant Point_Estimate := Thing (X.S, T).Centre;
         begin
            for A of X.S.Arms loop
               if not Busy (X.S, A.Id, T) then
                  declare
                     E     : constant Search.Effector := Search.Effector_Of (X.S, A.Id);
                     Arm_Id_Now : constant Arm_Id := A.Id;
                     function Can_Reach (Tool : Rigid) return Boolean is
                       (P.Reach ((Arm => Arm_Id_Now, Tool => Tool, Position_Only => False)).Status = Reachable);
                     C     : Search.Candidate;
                     Found : Boolean;
                     Acc   : Search.Account;
                  begin
                     Search.Find (Search.Shape_Of (X.S, T), Beside_Of (X.S, T), E, Motion, Gravity (X.S),
                                  Thing (X.S, T).Friction, Can_Reach'Access, C, Found, Acc);
                     if Found and then C.Force < Force then
                        Best := C;
                        Force := C.Force;
                        Arm_Of_Best := A.Id;
                     elsif not Found then
                        Note_Tried (X, "arm " & Img (Integer (A.Id)) & ": " & Search.Say (Acc));
                     end if;
                  end;
               else
                  Note_Tried (X, "arm " & Img (Integer (A.Id)) & " holds something else");
               end if;
            end loop;
            if Force = Real'Last then
               return;
            end if;
            declare
               E   : constant Search.Effector := Search.Effector_Of (X.S, Arm_Of_Best);
               Out_Come : Step_Outcome;
               Why : Unbounded_String;
            begin
               G := (Arm => Arm_Of_Best, Hands => E.Closers, Closing => Natural (Best.Touches.Length) > 1, Searched => True,
                     Chosen => Best);
               Say (X, "I meet it with arm " & Img (Integer (Arm_Of_Best)) & " at "
                    & Img (Natural (Best.Touches.Length)) & " touch" & (if Natural (Best.Touches.Length) > 1 then "es" else "")
                    & ", needing friction " & Img (Best.Mu_Worst) & " at most");
               if G.Closing then
                  for K in 1 .. Natural (E.Closers.Length) loop
                     Set_Closer (P, X, E.Closers (K).Hand, Best.Before (K), Out_Come);
                  end loop;
               end if;
               Travel (P, X, Arm_Of_Best, Best.Hover, 0, 0, Out_Come, Why);
               if Out_Come in Refused | Blocked then
                  Note_Tried (X, "going to where the last straight stretch begins: "
                              & (if Out_Come = Refused then To_String (Why) else "something stopped the arm"));
                  return;
               end if;
               Go (P, X, Arm_Of_Best, Best.Tool, Out_Come, Why);
               if Out_Come = Refused then
                  Note_Tried (X, "coming in to touch it: " & To_String (Why));
                  return;
               end if;
               if not G.Closing then
                  Ok := True;
                  return;
               end if;
               for K in 1 .. Natural (E.Closers.Length) loop
                  Set_Closer (P, X, E.Closers (K).Hand, 1.0, Out_Come);
               end loop;
               --  Held: the closers stopped short of closed on nothing.
               declare
                  Holding : Boolean := True;
               begin
                  for C of E.Closers loop
                     declare
                        F : constant Estimate := Hand (X.S, C.Hand).Fraction;
                     begin
                        if not (Known (F) and then Significant (1.0 - F.Value, F.Sigma, F.Degrees_Of_Freedom)) then
                           Holding := False;
                        end if;
                     end;
                  end loop;
                  if Holding then
                     Say (X, "my lobes closed on it");
                     Ok := True;
                     return;
                  end if;
               end;
               Note_Tried (X, "closing arm " & Img (Integer (Arm_Of_Best)) & "'s lobes on it: they closed on nothing");
               for C of E.Closers loop
                  Set_Closer (P, X, C.Hand, 0.0, Out_Come);
               end loop;
               if not Significant (Before_Centre, Thing (X.S, T).Centre) then
                  Note_Tried (X, "it did not move, so the same try would end the same way");
                  return;
               end if;
               Say (X, "it moved while I closed, so I look again and choose anew");
            end;
         end;
      end loop;
   end Acquire;

   --  How far along the unit twist the thing can go before it meets a
   --  surface or another thing, and the uncertainty of that distance.
   procedure Contact_Ahead (S : Snapshot; T : Thing_Id; G : Contact.Twist; Ahead, Band : out Real) is
      X : constant Thing_State := Thing (S, T);
   begin
      Ahead := Real'Last;
      Band := 0.0;
      for Smp of X.Samples loop
         declare
            V  : constant Vec3 := Contact.Velocity (G, Smp.Point);
            VV : constant Real := V * V;
         begin
            if VV > 0.0 then
               for F of S.Surfaces loop
                  if F.Of_Thing = 0 and then abs F.Normal.Unit_Vector > 0.0 then
                     declare
                        N  : constant Vec3 := Unit (F.Normal.Unit_Vector);
                        In_Rate : constant Real := -Real'(V * N);
                        H  : constant Real := (Smp.Point - F.Point.Mean) * N;
                        B  : constant Real := Z * Sqrt (X.Sigma ** 2 + Largest_Sigma (F.Point.Covariance) ** 2);
                     begin
                        if In_Rate > 0.0 and then H > -B and then H / In_Rate < Ahead then
                           Ahead := Real'Max (0.0, H) / In_Rate;
                           Band := B / In_Rate;
                        end if;
                     end;
                  end if;
               end loop;
               for O of S.Things loop
                  if O.Id /= T then
                     declare
                        B : constant Real := Z * Sqrt (X.Sigma ** 2 + O.Sigma ** 2);
                        Reach_Of : constant Real := O.Pitch / 2.0 + B;
                     begin
                        for Q of O.Samples loop
                           declare
                              D : constant Vec3 := Q.Point - Smp.Point;
                              Along : constant Real := Real'(D * V) / VV;
                           begin
                              if Along > 0.0 and then Along < Ahead
                                and then abs (D - Along * V) <= Reach_Of
                              then
                                 Ahead := Along;
                                 Band := B / Sqrt (VV);
                              end if;
                           end;
                        end loop;
                     end;
                  end if;
               end loop;
            end if;
         end;
      end loop;
   end Contact_Ahead;

   --  The thing's own lowest layer over what bears it, as its footing.
   function Base_Over_Support (S : Snapshot; T : Thing_Id) return Contact.Footing is
      X    : constant Thing_State := Thing (S, T);
      N    : constant Vec3 := Goals.Up_Of (S, T);
      Low  : Real := Real'Last;
      Pts  : Contact.Point_Vectors.Vector;
      Under : constant Thing_Id'Base :=
        (if X.Support /= 0 and then Has_Surface (S, X.Support) then Surface (S, X.Support).Of_Thing else 0);
      Band : constant Real := Real'Max (X.Pitch, Z * X.Sigma);
   begin
      if not (abs N > 0.0) then
         return Contact.No_Footing;
      end if;
      for Smp of X.Samples loop
         Low := Real'Min (Low, Smp.Point * N);
      end loop;
      for Smp of X.Samples loop
         if Smp.Point * N - Low <= Band then
            if Under = 0 or else not Has_Thing (S, Under) then
               Pts.Append (Smp.Point);
            else
               declare
                  Below : constant Thing_State := Thing (S, Under);
                  Over  : Boolean := False;
               begin
                  for Q of Below.Samples loop
                     if Q.Normal * N > 0.0 then
                        declare
                           D : constant Vec3 := Q.Point - Smp.Point;
                        begin
                           if abs (D - Real'(D * N) * N) <= Below.Pitch then
                              Over := True;
                              exit;
                           end if;
                        end;
                     end if;
                  end loop;
                  if Over then
                     Pts.Append (Smp.Point);
                  end if;
               end;
            end if;
         end if;
      end loop;
      return Contact.Footing_Of (Pts, Low * N, N, X.Pitch);
   end Base_Over_Support;

   --  Moves the thing, held or touched as G says, step by step along what
   --  Next_Goal wants of it, until the monitor names an ending.
   procedure Carry (P : in out Plant'Class; X : in out State; T : Thing_Id; G : Grip;
                    Next_Goal : not null access function (S : Snapshot) return Goals.Answer;
                    Wanted : Ending_Set; Max_Steps : Natural; Final : out Ending)
   is
      Watch  : Monitor.Watch := Monitor.Start;
      Start  : constant Point_Estimate := Thing (X.S, T).Centre;
      Up0    : constant Vec3 := Goals.Up_Of (X.S, T);
      Supported : constant Boolean := Thing (X.S, T).Support /= 0;
      Travelled : Real := 0.0;
   begin
      loop
         declare
            Goal    : constant Goals.Answer := Next_Goal (X.S);
            Now     : constant Arm_State := Arm (X.S, G.Arm);
            Before  : constant Thing_State := Thing (X.S, T);
            F       : Monitor.Facts;
            Step    : Real := 0.0;
            Ahead, Band : Real := Real'Last;
            Res     : Arm_Result;
         begin
            if Goal.Ok and then not Goal.Done then
               Contact_Ahead (X.S, T, Goal.Motion, Ahead, Band);
               declare
                  Fine  : constant Real := Resolution (Now, Goal.Motion);
                  Limit : Real := Real'Last;
                  function Fits (S : Real) return Boolean is
                    (P.Reach ((Arm => G.Arm, Tool => Moved (Goal.Motion, S, Now.Tool.Pose), Position_Only => False))
                       .Status = Reachable
                     and then P.In_View (Contact.Apply (Contact.Scaled (Goal.Motion, S), Before.Centre.Mean)));
               begin
                  if Ahead < Real'Last then
                     Limit := Real'Max (Fine, Ahead - Band);
                  end if;
                  if Known (Goal.Gap) then
                     Limit := Real'Min (Limit, Real'Max (Fine, Goal.Gap.Value));
                  end if;
                  if Wanted (Free) and then Supported then
                     --  The rise is judged against its own noise: go just far enough for that.
                     Limit := Real'Min (Limit, Real'Max (Fine, Z * Sqrt (2.0) * Largest_Sigma (Start.Covariance)
                                                          - Real'((Before.Centre.Mean - Start.Mean) * Up0)));
                  end if;
                  if Limit < Real'Last and then Fits (Limit) then
                     Step := Limit;
                  else
                     --  Doubled from the smallest step while it fits, then halved back down to it.
                     declare
                        Low  : Real := 0.0;
                        High : Real := Fine;
                     begin
                        while High < Limit and then Fits (High) loop
                           Low := High;
                           High := 2.0 * High;
                        end loop;
                        High := Real'Min (High, Limit);
                        while High - Low > Fine loop
                           if Fits ((Low + High) / 2.0) then
                              Low := (Low + High) / 2.0;
                           else
                              High := (Low + High) / 2.0;
                           end if;
                        end loop;
                        Step := Low;
                     end;
                  end if;
                  if Step < Fine then
                     Step := 0.0;
                  end if;
               end;
            end if;
            F.Commanded := Step > 0.0;
            F.Exhausted := Goal.Ok and then not Goal.Done and then Step = 0.0;
            if F.Commanded then
               declare
                  R : Report;
               begin
                  P.Move (One_Arm (G.Arm, Moved (Goal.Motion, Step, Now.Tool.Pose)), R);
                  Res := R.Arms.First_Element;
               end;
            end if;
            Look (P, X);
            declare
               After_Tool : constant Rigid := Arm (X.S, G.Arm).Tool.Pose;
               Carried    : constant Rigid := After_Tool * Inverse (Now.Tool.Pose);
               Delivered  : constant Real := (if F.Commanded and then Known (Res.Delivered)
                                              then Real'Max (0.0, Res.Delivered.Value) * Step else 0.0);
            begin
               if F.Commanded then
                  Travelled := Travelled + Delivered;
               end if;
               F.Blocked := F.Commanded and then Res.Outcome = Blocked and then Delivered < Ahead - Band;
               F.Touch := F.Commanded and then Res.Outcome = Blocked and then not F.Blocked;
               if Res.Outcome = Refused and then F.Commanded then
                  F.Blocked := True;
                  Say (X, "the arm refused the step: " & To_String (Res.Why));
               end if;
               if Has_Thing (X.S, T) then
                  declare
                     Now_T : constant Thing_State := Thing (X.S, T);
                  begin
                     F.Seen := Now_T.Seen;
                     F.Followable := Known (Now_T.Centre);
                     F.Height_Gain := (Value => Real'((Now_T.Centre.Mean - Start.Mean) * Up0),
                                       Sigma => Sqrt (2.0) * Largest_Sigma (Start.Covariance), Degrees_Of_Freedom => 0);
                     if F.Commanded then
                        --  Where it was, carried by the hand's measured motion:
                        --  the two hand readings and the lever of their turn add
                        --  their uncertainty to that of where it was.
                        declare
                           Lever : constant Real := abs (Before.Centre.Mean - Now.Tool.Pose.Translation);
                           Turns : constant Real := Largest_Sigma (Now.Tool.Rotation_Covariance) ** 2
                             + Largest_Sigma (Arm (X.S, G.Arm).Tool.Rotation_Covariance) ** 2;
                        begin
                           F.Carried_To := (Mean       => Carried * Before.Centre.Mean,
                                            Covariance => Before.Centre.Covariance + Now.Tool.Position_Covariance
                                              + Arm (X.S, G.Arm).Tool.Position_Covariance
                                              + (Lever ** 2 * Turns) * Identity3);
                        end;
                        F.Carried_At := Now_T.Centre;
                     end if;
                  end;
               else
                  F.Seen := False;
                  F.Followable := False;
               end if;
               if G.Closing then
                  declare
                     Fr : constant Estimate := Hand (X.S, G.Hands.First_Element.Hand).Fraction;
                  begin
                     F.Closed_Short := (Value => 1.0 - Fr.Value, Sigma => Fr.Sigma,
                                        Degrees_Of_Freedom => Fr.Degrees_Of_Freedom);
                  end;
               end if;
               F.Still := X.S.Still;
               F.Gap := (if Goal.Ok then Goal.Gap else Unknown);
               F.Owed := Delivered;
               F.Out_Of_Beats := P.Episode_Over;
            end;
            Monitor.Step (Watch, F);
            if Monitor.Fired (Watch, F, Wanted, Max_Steps) then
               Final := Monitor.Ending_Of (Watch, F, Wanted, Max_Steps);
               if Has_Thing (X.S, T) then
                  declare
                     Moved_By : constant Vec3 := Thing (X.S, T).Centre.Mean - Start.Mean;
                  begin
                     Say (X, "it moved by " & Img (abs Moved_By) & ", " & Img (Real'(Moved_By * Up0))
                          & " of it up, in " & Img (Monitor.Steps (Watch)) & " steps");
                  end;
               end if;
               if F.Exhausted and then Final in Stuck | Settled then
                  Say (X, "I could take it no further: a step of the smallest size would leave my reach or my view");
               end if;
               if not Goal.Ok then
                  Say (X, To_String (Goal.Why));
               end if;
               return;
            end if;
         end;
      end loop;
   end Carry;

   --  A thing brought down onto a surface is let go when it rests there:
   --  the closers open, the hand backs out the way it came, and a look
   --  confirms that it stayed.
   procedure Put_Down (P : in out Plant'Class; X : in out State; T : Thing_Id; G : Grip) is
      Thing_Now : constant Thing_State := Thing (X.S, T);
      Rest : constant Wrench.Rest_Answer :=
        Wrench.Rests (Base_Over_Support (X.S, T), Thing_Now.Centre, Goals.Up_Of (X.S, T), Thing_Now.Friction.Low);
      Out_Come : Step_Outcome;
      Why : Unbounded_String;
   begin
      if not G.Closing then
         return;
      elsif not Rest.Rests then
         Say (X, "I keep holding it: it would not rest there, its centre is not over its footing by "
              & Img (Rest.Margin));
         return;
      end if;
      for C of G.Hands loop
         Set_Closer (P, X, C.Hand, 0.0, Out_Come);
      end loop;
      declare
         E    : constant Search.Effector := Search.Effector_Of (X.S, G.Arm);
         Back : constant Rigid :=
           (Rotation => E.Tool.Rotation, Translation => E.Tool.Translation - E.Depth * Rotate (E.Tool, E.Along));
      begin
         Go (P, X, G.Arm, Back, Out_Come, Why);
      end;
      if Has_Thing (X.S, T) and then Thing (X.S, T).Held_By = 0
        and then not Significant (Thing_Now.Centre, Thing (X.S, T).Centre)
      then
         Say (X, "I let go of it and it stayed where I put it");
      elsif Has_Thing (X.S, T) then
         Say (X, "I let go of it and it moved by " & Img (abs (Thing (X.S, T).Centre.Mean - Thing_Now.Centre.Mean)));
      end if;
   end Put_Down;

   --  The quantities this body can change now, in the keyboard's order.
   function Quantity_Of (S : Snapshot; Index : Positive; Q : out Goals.Quantity) return Boolean is
      Can  : constant Goals.Quantity_Set := Goals.Changeable (S);
      Seen : Natural := 0;
   begin
      for K in Goals.Quantity loop
         if Can (K) then
            Seen := Seen + 1;
            if Seen = Index then
               Q := K;
               return True;
            end if;
         end if;
      end loop;
      return False;
   end Quantity_Of;

   procedure Run_Change (P : in out Plant'Class; X : in out State; W : Want; Final : out Ending) is
      T : constant Thing_Id := W.Thing;
      Q : Goals.Quantity;
   begin
      Final := Refused;
      if not Has_Thing (X.S, T) then
         Note_Tried (X, "I looked for it among the " & Img (Natural (X.S.Things.Length))
                     & " things I measure now and it is not one of them");
         return;
      elsif not Quantity_Of (X.S, W.Quantity, Q) then
         Note_Tried (X, "this body cannot measure and change that quantity now");
         return;
      end if;
      declare
         First : constant Goals.Answer := Goals.Twist_Of (X.S, T, Q, W.Increase);
         Hold  : Grip;
         Ok    : Boolean;
         function Next_Goal (S : Snapshot) return Goals.Answer is
           (if Has_Thing (S, T) then Goals.Twist_Of (S, T, Q, W.Increase)
            else (Ok => False, Why => To_Unbounded_String ("I no longer see it"), others => <>));
      begin
         if not First.Ok then
            Note_Tried (X, To_String (First.Why));
            return;
         end if;
         if Thing (X.S, T).Held_By /= 0 and then Has_Hand (X.S, Thing (X.S, T).Held_By) then
            declare
               H : constant Hand_State := Hand (X.S, Thing (X.S, T).Held_By);
            begin
               Hold := (Arm => H.Arm, Hands => Search.Effector_Of (X.S, H.Arm).Closers, Closing => True, Searched => False,
                        Chosen => <>);
               Say (X, "I hold it already");
            end;
         else
            Acquire (P, X, T, First.Motion, Hold, Ok);
            if not Ok then
               Note_Tried (X, "no part of me could take it that way");
               return;
            end if;
         end if;
         Carry (P, X, T, Hold, Next_Goal'Access, W.Until_Endings, W.Max_Steps, Final);
         if Final = Touched and then Q = Goals.Height and then not W.Increase then
            Put_Down (P, X, T, Hold);
         elsif Final = Slipped and then Hold.Closing and then Hold.Searched then
            --  The grip needed Mu_Nominal as measured and did not hold:
            --  the thing's friction is below it.
            declare
               B : Friction_Bounds := Thing (X.S, T).Friction;
            begin
               B.High := Real'Min (B.High, Hold.Chosen.Mu_Nominal);
               P.Learn ((Kind => Friction_Learned, Thing => T, Bounds => B));
               Say (X, "it slipped out of a grip that needed friction " & Img (Hold.Chosen.Mu_Nominal)
                    & ", so I take its friction to be less than that from now on");
            end;
         end if;
      end;
   end Run_Change;

   procedure Execute (P : in out Driver.Action.Plants.Plant'Class; W : Want; R : out Result) is
      X : State;
   begin
      R := (Final => Refused, Tried => Null_Unbounded_String, Account => Null_Unbounded_String);
      Look (P, X);
      case W.Kind is
         when Change =>
            Run_Change (P, X, W, R.Final);
         when Interval =>
            Note_Tried (X, "intervals of constraints are not built yet");
      end case;
      R.Account := X.Account;
      R.Tried := X.Tried;
   end Execute;

end Driver.Action.Execution;
