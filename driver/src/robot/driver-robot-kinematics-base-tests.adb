with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Rig;
with Driver.Stats;
with Driver.Tests;

package body Driver.Robot.Kinematics.Base.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;

   function Radians (Deg : Real) return Real is (Deg * Ada.Numerics.Pi / 180.0);

   --  A floor 0.7 below the arm's eye at its reference, looked at 35 degrees down; the body's eye 0.25 above it and
   --  0.1 aside, looking 50 degrees down and 20 inwards (its frame on the arm's eye, Rig_X, as the walk told it). At a
   --  keyframe the arm turned its eye 12 degrees and moved it 0.15, and the base under it leaned 1.5 degrees about a
   --  line 0.9 below the arm's eye: each eye moved by that lean, the body's eye with nothing else. Every point of
   --  either eye's reference picture on the floor every 20 pixels, its inverse depth known (the walk's), its
   --  answer where the eye saw it. Shifted, each answer must be where the arm's eye would have seen its point had the
   --  base held still, to a thousandth of a pixel.
   --  Park and Miller's minimal standard generator and Box and Muller's normal draws, for the noisy case.
   type Generator is record
      State : Long_Long_Integer := 1;
   end record;

   function Uniform (G : in out Generator) return Real is
   begin
      G.State := (G.State * 48_271) mod 2_147_483_647;
      return Real (G.State) / 2_147_483_647.0;
   end Uniform;

   function Gaussian (G : in out Generator) return Real is
      U1 : constant Real := Real'Max (Uniform (G), Real'Model_Small);
      U2 : constant Real := Uniform (G);
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   --  The scene below, its answers with Noise px of error, the walk's inverse depths off by a relative Depth_Error
   --  each, the walk's frame of the body's eye on the arm's turned by Turn_Error degrees: how far, at most (Worst) and
   --  in the median (Middle), each sighting's shift stands from the shift that takes a still base's place.
   procedure Shift_Errors
     (Noise, Depth_Error, Turn_Error : Real; Worst, Middle, Lean_Size : out Real; Checked : out Natural;
      Within_Z : out Real; Moved : out Boolean; Lean_Factor : Real := 1.0);
   --  Lean_Size: the median of how far the base's lean moved the sightings, the shift's own size. Within_Z: the share
   --  of the sightings whose shift lies within Z of the sigma it states (Shift_Variance) from the shift that takes a
   --  still base's place, the walk's frame's covariance its turn's error; Moved: the keyframe's base was judged to
   --  have moved. Lean_Factor: the lean's size, nought for a base that held still.

   procedure Leaned_Base_Taken_Out is
      Worst, Middle, Lean, Within : Real;
      Checked : Natural;
      Moved   : Boolean;
   begin
      Shift_Errors (0.0, 0.0, 0.0, Worst, Middle, Lean, Checked, Within, Moved);
      Check (Checked > 20, "only" & Checked'Image & " sightings to judge");
      Check (Worst < 1.0e-3, "a shifted sighting stands" & Worst'Image & " px from where a still base puts it");
      --  As the walk and the aligner leave them: a fifth of a pixel of noise on every answer, the inverse depths 5 %
      --  off, the body's eye's frame on the arm's half a degree off (the walk's calibration did 0.43 to 0.66, 10-26).
      --  The lean taken out to a twentieth of itself in the median, Z twentieths at most: the hall's G1's whole lean
      --  put its arms' focal lengths 16 to 18 % long (10-26), so what a twentieth of it leaves stays within a
      --  ninth of step 2's 2 %.
      Shift_Errors (0.2, 0.05, 0.5, Worst, Middle, Lean, Checked, Within, Moved);
      Check (Middle < Lean / 20.0 and then Worst < Driver.Conventions.Z * Lean / 20.0,
             "with the answers' noise, the walk's depths and turn off, a shift errs by" & Middle'Image
             & " px in the median and" & Worst'Image & " at most, of a lean of" & Lean'Image & " px");
      --  And says how far: within Z of the sigma it states, as a Gaussian two-dimensional error is but at the
      --  e ** (-Z ** 2 / 2) of them, here at the most a tenth.
      Check (Within >= 0.9, "the shifts' errors lie within Z of the sigmas they state at" & Within'Image
             & " of the sightings");
      --  A walk that told nothing (its frame half a turn off, stated as known) under a base that held still: no
      --  sighting moved (RB-Y1, round n15s2, 10-27: such a walk moved its arm's sightings a median 98 px).
      Shift_Errors (0.2, 0.05, 90.0, Worst, Middle, Lean, Checked, Within, Moved, Lean_Factor => 0.0);
      Check (not Moved and then Worst < Driver.Conventions.Z * 0.2,
             "a still base under a walk 90 degrees off moved its sightings by" & Worst'Image & " px at most");
   end Leaned_Base_Taken_Out;

   procedure Shift_Errors
     (Noise, Depth_Error, Turn_Error : Real; Worst, Middle, Lean_Size : out Real; Checked : out Natural;
      Within_Z : out Real; Moved : out Boolean; Lean_Factor : Real := 1.0) is
      Width  : constant Real := 320.0;
      Height : constant Real := 240.0;
      L_Arm  : constant Driver.Rig.Lens := (Focal => 157.0, Cx => Width / 2.0, Cy => Height / 2.0);
      L_Body : constant Driver.Rig.Lens := (Focal => 140.0, Cx => Width / 2.0, Cy => Height / 2.0);
      --  An eye looking along its heading H pitched down by P (x right, y down, z ahead), in a world z up.
      function Looking (Heading, Pitch : Real; At_Place : Vec3) return Rigid is
         Ahead : constant Vec3 := [Cos (Pitch) * Cos (Heading), Cos (Pitch) * Sin (Heading), -Sin (Pitch)];
         Right : constant Vec3 := [Sin (Heading), -Cos (Heading), 0.0];
         Down  : constant Vec3 := Cross (Ahead, Right);
      begin
         return (Rotation => [[Right (1), Down (1), Ahead (1)], [Right (2), Down (2), Ahead (2)],
                              [Right (3), Down (3), Ahead (3)]],
                 Translation => At_Place);
      end Looking;
      World_Arm  : constant Rigid := Looking (0.0, Radians (35.0), [0.0, 0.0, 0.7]);
      World_Body : constant Rigid := Looking (Radians (-20.0), Radians (50.0), [0.05, -0.1, 0.95]);
      --  Everything in the arm's eye's reference frame, as Kinematics.Base has it.
      X       : constant Rigid := Inverse (World_Arm) * World_Body;
      Floor_Z : constant Real := 0.0;
      --  The arm's own motion of its eye (in that frame), and the base's lean about a line below the arm's eye.
      Arm_Move : constant Rigid := (Rotation => Exp ([Radians (4.0), Radians (12.0), 0.0]),
                                    Translation => [0.15, -0.02, 0.05]);
      Pivot    : constant Vec3 := Transposed_Times (World_Arm.Rotation, [0.0, 0.0, -0.9]);
      Lean_R   : constant Mat3 := Exp ([Lean_Factor * Radians (1.5), 0.0, Lean_Factor * Radians (0.5)]);
      Lean     : constant Rigid := (Rotation => Lean_R, Translation => Minus (Pivot, Times (Lean_R, Pivot)));
      E_Still  : constant Rigid := Arm_Move;
      E_Seen   : constant Rigid := Lean * Arm_Move;
      B_Seen   : constant Rigid := Inverse (X) * Lean * X;   --  the body's eye's motion in its own reference frame
      R        : Arm_Evidence;
      G        : Generator;
      Errors, Leans : Real_Vectors.Vector;
      Inside : Natural := 0;   --  the sightings whose shift lies within Z of its stated sigma from the truth's
      --  The world point a reference pixel of an eye (in the arm's eye's reference frame, Frame) looks at on the floor.
      function On_Floor (Frame : Rigid; L : Driver.Rig.Lens; P : Driver.Rig.Place; Inv : out Real) return Boolean is
         Ray   : constant Vec3 := Driver.Rig.Ray (L, P);
         World : constant Rigid := World_Arm * Frame;
         D     : constant Vec3 := Times (World.Rotation, Ray);
      begin
         if D (3) >= 0.0 then
            return False;
         end if;
         Inv := 1.0 / ((Floor_Z - World.Translation (3)) / D (3));
         return 1.0 / Inv < 6.0;
      end On_Floor;
      procedure Add_Points (Arm : Boolean) is
         V : Real := 10.0;
      begin
         while V < Height loop
            declare
               U : Real := 10.0;
            begin
               while U < Width loop
                  declare
                     Inv : Real;
                  begin
                     if On_Floor ((if Arm then Identity else X), (if Arm then L_Arm else L_Body), (U, V), Inv) then
                        if Arm then
                           R.Query_U.Append (U);
                           R.Query_V.Append (V);
                           R.Rig_Arm_Inverse.Append (Inv);
                        else
                           R.Body_Query_U.Append (U);
                           R.Body_Query_V.Append (V);
                           R.Rig_Body_Inverse.Append (Inv);
                        end if;
                     end if;
                  end;
                  U := U + 20.0;
               end loop;
            end;
            V := V + 20.0;
         end loop;
      end Add_Points;
      --  The answers of an eye that moved by Motion (in its reference frame) for its points.
      procedure Answers (L : Driver.Rig.Lens; Motion : Rigid; U0, V0, Inverse_Depths : Real_Vectors.Vector;
                         Set : out Match_Set) is
      begin
         Set := (Frame => 2, Followed => True, others => <>);
         for I in 0 .. Natural (U0.Length) - 1 loop
            declare
               P     : Driver.Rig.Place;
               Ahead : Boolean;
            begin
               Driver.Rig.Project (L, Driver.Rig.Seen (Motion, Driver.Rig.Ray (L, (U0 (I), V0 (I))),
                                                       Inverse_Depths (I)), P, Ahead);
               Set.To_U.Append (P.U + Noise * Gaussian (G));
               Set.To_V.Append (P.V + Noise * Gaussian (G));
               Set.Sigma.Append (Real'Max (Noise, 0.1));
               Set.Freedom.Append (0);
               Set.Back_U.Append (U0 (I));
               Set.Back_V.Append (V0 (I));
               Set.Found.Append (Ahead and then P.U in 0.0 .. Width and then P.V in 0.0 .. Height);
               Set.Answered.Append (Ahead);
            end;
         end loop;
      end Answers;
   begin
      Worst := 0.0;
      Middle := 0.0;
      Lean_Size := 0.0;
      Checked := 0;
      Within_Z := 0.0;
      Moved := False;
      Add_Points (Arm => True);
      Add_Points (Arm => False);
      R.Rig_Ok := True;
      R.Rig_X := X;
      R.Rig_Arm_Lens := L_Arm;
      R.Rig_Body_Lens := L_Body;
      R.Aligned := 2;
      R.Frames.Append (Keyframe'(Beat => 1, others => <>));
      R.Frames.Append (Keyframe'(Beat => 2, others => <>));
      R.Frames (1).Readings.Append (0.0);
      R.Frames (2).Readings.Append (1.0);
      declare
         Arm_Set, Body_Set : Match_Set;
         M : Model;
      begin
         Answers (L_Arm, E_Seen, R.Query_U, R.Query_V, R.Rig_Arm_Inverse, Arm_Set);
         Answers (L_Body, B_Seen, R.Body_Query_U, R.Body_Query_V, R.Rig_Body_Inverse, Body_Set);
         R.Matches.Append (Arm_Set);
         R.Body_Matches.Append (Body_Set);
         --  The truth's depths kept to judge by; the walk's as off as asked.
         declare
            True_Inverse : constant Real_Vectors.Vector := R.Rig_Arm_Inverse;
         begin
            for I in 0 .. Natural (R.Rig_Arm_Inverse.Length) - 1 loop
               R.Rig_Arm_Inverse.Replace_Element (I, R.Rig_Arm_Inverse (I) * (1.0 + Depth_Error * Gaussian (G)));
            end loop;
            for J in 0 .. Natural (R.Rig_Body_Inverse.Length) - 1 loop
               R.Rig_Body_Inverse.Replace_Element (J, R.Rig_Body_Inverse (J) * (1.0 + Depth_Error * Gaussian (G)));
            end loop;
            R.Rig_X := (Rotation => Times (Exp (Scaled (Radians (Turn_Error), Unit ([0.3, -0.8, 0.5]))), X.Rotation),
                        Translation => X.Translation);
            --  The walk's frame's covariance its turn's error on each term of its turn.
            for T in 1 .. 3 loop
               R.Rig_Cov (T, T) := Radians (Turn_Error) ** 2;
            end loop;
            Shift_Sightings (M, R);
            R.Rig_Arm_Inverse := True_Inverse;
         end;
         Check (Natural (R.Base.Length) = 2 and then R.Base (2).Known, "the keyframe's base motion is not known");
         if Natural (R.Base.Length) = 2 and then R.Base (2).Known then
            Moved := R.Base (2).Moved;
            for I in 0 .. Natural (R.Query_U.Length) - 1 loop
               if Arm_Set.Found (I) then
                  declare
                     P     : Driver.Rig.Place;
                     Ahead : Boolean;
                     Q     : Driver.Rig.Place;
                     Ahead_Q : Boolean;
                     Ray   : constant Vec3 := Driver.Rig.Ray (L_Arm, (R.Query_U (I), R.Query_V (I)));
                  begin
                     Driver.Rig.Project (L_Arm, Driver.Rig.Seen (E_Still, Ray, R.Rig_Arm_Inverse (I)), P, Ahead);
                     Driver.Rig.Project (L_Arm, Driver.Rig.Seen (E_Seen, Ray, R.Rig_Arm_Inverse (I)), Q, Ahead_Q);
                     if Ahead and then Ahead_Q then
                        declare
                           E : constant Real := Sqrt ((Shift_U (R, 2, I) - (P.U - Q.U)) ** 2
                                                      + (Shift_V (R, 2, I) - (P.V - Q.V)) ** 2);
                        begin
                           Worst := Real'Max (Worst, E);
                           Errors.Append (E);
                           if E <= Driver.Conventions.Z * Sqrt (Shift_Variance (R, 2, I)) then
                              Inside := Inside + 1;
                           end if;
                           Leans.Append (Sqrt ((P.U - Q.U) ** 2 + (P.V - Q.V) ** 2));
                           Checked := Checked + 1;
                        end;
                     end if;
                  end;
               end if;
            end loop;
         end if;
      end;
      if not Errors.Is_Empty then
         declare
            A, B : Real_Array (1 .. Natural (Errors.Length));
         begin
            for K in A'Range loop
               A (K) := Errors (K - 1);
               B (K) := Leans (K - 1);
            end loop;
            Middle := Driver.Stats.Median (A);
            Lean_Size := Driver.Stats.Median (B);
            Within_Z := Real (Inside) / Real (A'Length);
         end;
      end if;
   end Shift_Errors;

   procedure Register is
   begin
      Driver.Tests.Register ("robot.base.shift", "a keyframe's sightings are not taken where a base that held still "
                             & "would have put them, from its body's eye's view of the base's lean, nor say how far "
                             & "off they may be; or a still base's are moved by a walk that told nothing",
                             Leaned_Base_Taken_Out'Access);
   end Register;

end Driver.Robot.Kinematics.Base.Tests;
