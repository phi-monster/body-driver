with Ada.Numerics;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Tests;

package body Driver.Rig.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      U1 : constant Real := 1.0 - Real (Ada.Numerics.Float_Random.Random (Gen));
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   function Degrees (Radians : Real) return Real is (Radians * 180.0 / Ada.Numerics.Pi);
   function Radians (Deg : Real) return Real is (Deg * Ada.Numerics.Pi / 180.0);

   --  A walk of a body carrying two eyes over a floor, as the G1's head camera and wrist camera saw it (10-26): the
   --  first eye 0.7 above the floor looking 35 degrees down, the second 0.45 above it and 0.1 aside, looking 50
   --  degrees down and 20 degrees inwards; the body turned 15 degrees about the vertical and back, stepped 0.2 forward
   --  and back, 0.15 aside and back, each still off where the walk meant by a sway of half a degree and a centimetre.
   --  Every point of either eye's reference picture on the floor, every 20 pixels, a tenth of a pixel of noise; lenses
   --  of 157 and 140 px, the first's started 20 % long.
   procedure Walk_Calibrates is
      Width  : constant Real := 320.0;
      Height : constant Real := 240.0;
      L1 : constant Lens := (Focal => 157.0, Cx => Width / 2.0, Cy => Height / 2.0);
      L2 : constant Lens := (Focal => 140.0, Cx => Width / 2.0, Cy => Height / 2.0);
      --  The world: z up, the floor z = 0. A camera looking along its heading H, pitched down by P: its frame's
      --  columns (x right, y down, z ahead) in the world.
      function Looking (Heading, Pitch : Real; At_Place : Vec3) return Rigid is
         Ahead : constant Vec3 := [Cos (Pitch) * Cos (Heading), Cos (Pitch) * Sin (Heading), -Sin (Pitch)];
         Right : constant Vec3 := [Sin (Heading), -Cos (Heading), 0.0];
         Down  : constant Vec3 := Cross (Ahead, Right);
      begin
         return (Rotation => [[Right (1), Down (1), Ahead (1)], [Right (2), Down (2), Ahead (2)],
                              [Right (3), Down (3), Ahead (3)]],
                 Translation => At_Place);
      end Looking;
      Body_Frame : constant Rigid := Identity;   --  the body's frame in the world at the reference
      Eye1 : constant Rigid := Looking (0.0, Radians (35.0), [0.0, 0.0, 0.7]);
      Eye2 : constant Rigid := Looking (Radians (-20.0), Radians (50.0), [0.05, -0.1, 0.45]);
      --  The body's moves (in the world): a turn about the vertical through the body's origin and a step.
      Stills : constant := 6;
      type Move is record
         Turn : Real;   --  degrees about z
         Step : Vec3;
      end record;
      Moves : constant array (1 .. Stills) of Move :=
        [(15.0, [0.0, 0.0, 0.0]), (0.0, [0.0, 0.0, 0.0]), (0.0, [0.2, 0.0, 0.0]), (0.0, [0.0, 0.0, 0.0]),
         (0.0, [0.0, 0.15, 0.0]), (0.0, [0.0, 0.0, 0.0])];
      Bodies : array (1 .. Stills) of Rigid;
      Points1, Points2 : Place_Array (1 .. 200);
      N1, N2 : Natural := 0;
      Sightings : Sighting_Array (1 .. 4000);
      Ns : Natural := 0;
      --  The same, but the first eye's sightings of still Bad matched as if the body had stepped the other way: a
      --  field one motion of that eye explains, the wrong one (the matcher locked onto another of the floor's squares,
      --  as the G1's wrist eye did at its longest walk's first still, walkdump.py, 10-27).
      Bad : constant := 3;
      Corrupt : Sighting_Array (1 .. 4000);
      Nc : Natural := 0;
      --  Where a point of an eye's reference picture lies on the floor (world).
      function On_Floor (E : Rigid; L : Lens; P : Place; X : out Vec3) return Boolean is
         D : constant Vec3 := Times (E.Rotation, Ray (L, P));
      begin
         if D (3) >= 0.0 then
            return False;
         end if;
         X := E.Translation + (-E.Translation (3) / D (3)) * D;
         return abs (X - E.Translation) < 6.0;
      end On_Floor;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 7);
      for Q in 1 .. Stills loop
         declare
            Sway : constant Vec3 := [Radians (0.5) * Gaussian, Radians (0.5) * Gaussian, Radians (0.5) * Gaussian];
         begin
            Bodies (Q) := (Rotation => Times (Exp ([0.0, 0.0, Radians (Moves (Q).Turn)]), Exp (Sway)),
                           Translation => Moves (Q).Step + [0.01 * Gaussian, 0.01 * Gaussian, 0.0]);
         end;
      end loop;
      for Eye in 1 .. 2 loop
         declare
            E : constant Rigid := (if Eye = 1 then Eye1 else Eye2);
            L : constant Lens := (if Eye = 1 then L1 else L2);
            V : Real := 10.0;
         begin
            while V < Height loop
               declare
                  U : Real := 10.0;
               begin
                  while U < Width loop
                     declare
                        X : Vec3;
                     begin
                        if On_Floor (Body_Frame * E, L, (U, V), X) then
                           if Eye = 1 then
                              N1 := N1 + 1;
                              Points1 (N1) := (U, V);
                           else
                              N2 := N2 + 1;
                              Points2 (N2) := (U, V);
                           end if;
                           for Q in 1 .. Stills loop
                              declare
                                 Now : constant Rigid := Bodies (Q) * E;
                                 P   : Place;
                                 Ahead : Boolean;
                              begin
                                 Project (L, Transposed_Times (Now.Rotation, X - Now.Translation), P, Ahead);
                                 if Ahead and then P.U in 0.0 .. Width and then P.V in 0.0 .. Height then
                                    Ns := Ns + 1;
                                    Sightings (Ns) := (Eye => Eye, Point => (if Eye = 1 then N1 else N2), Still => Q,
                                                       U => P.U + 0.1 * Gaussian, V => P.V + 0.1 * Gaussian,
                                                       Sigma => 0.1);
                                    if Eye /= 1 or else Q /= Bad then
                                       Nc := Nc + 1;
                                       Corrupt (Nc) := Sightings (Ns);
                                    end if;
                                 end if;
                                 if Eye = 1 and then Q = Bad then
                                    declare
                                       Wrong : constant Rigid :=
                                         (Rotation => Bodies (Q).Rotation, Translation => -Bodies (Q).Translation) * E;
                                       Pw : Place;
                                       Aw : Boolean;
                                    begin
                                       Project (L, Transposed_Times (Wrong.Rotation, X - Wrong.Translation), Pw, Aw);
                                       if Aw and then Pw.U in 0.0 .. Width and then Pw.V in 0.0 .. Height then
                                          Nc := Nc + 1;
                                          Corrupt (Nc) := (Eye => 1, Point => N1, Still => Q,
                                                           U => Pw.U + 0.1 * Gaussian, V => Pw.V + 0.1 * Gaussian,
                                                           Sigma => 0.1);
                                       end if;
                                    end;
                                 end if;
                              end;
                           end loop;
                        end if;
                     end;
                     U := U + 20.0;
                  end loop;
               end;
               V := V + 20.0;
            end loop;
         end;
      end loop;
      declare
         C : Calibration (Stills, N1, N2);
         True_X : constant Rigid := Inverse (Eye1) * Eye2;
      begin
         Calibrate (Points1 (1 .. N1), Points2 (1 .. N2), Sightings (1 .. Ns), Stills,
                    (Focal => 1.2 * L1.Focal, Cx => L1.Cx, Cy => L1.Cy), (L2.Cx, L2.Cy), C);
         Check (C.Ok, "the walk's stills calibrate nothing:" & Ns'Image & " sightings, misfit" & C.Misfit'Image);
         Check (Degrees (Angle (Times (Transposed (True_X.Rotation), C.Second_On_First.Rotation))) < 0.5,
                "the second eye's turn on the first is"
                & Real'Image (Degrees (Angle (Times (Transposed (True_X.Rotation), C.Second_On_First.Rotation))))
                & " degrees off");
         Check (abs (C.First_Lens.Focal / L1.Focal - 1.0) < 0.02,
                "the first focal length is" & C.First_Lens.Focal'Image & " for" & L1.Focal'Image);
         Check (abs (C.Second_Lens.Focal / L2.Focal - 1.0) < 0.02,
                "the second focal length is" & C.Second_Lens.Focal'Image & " for" & L2.Focal'Image);
         Check (C.Turn_Sigma < Radians (0.5), "the turn's sigma states" & Real'Image (Degrees (C.Turn_Sigma)) & " deg");
      end;
      --  And with no first focal length to start from.
      declare
         C : Calibration (Stills, N1, N2);
         True_X : constant Rigid := Inverse (Eye1) * Eye2;
      begin
         Calibrate (Points1 (1 .. N1), Points2 (1 .. N2), Sightings (1 .. Ns), Stills,
                    (Focal => 0.0, Cx => L1.Cx, Cy => L1.Cy), (L2.Cx, L2.Cy), C);
         Check (C.Ok and then Degrees (Angle (Times (Transposed (True_X.Rotation), C.Second_On_First.Rotation))) < 0.5
                and then abs (C.First_Lens.Focal / L1.Focal - 1.0) < 0.02
                and then abs (C.Second_Lens.Focal / L2.Focal - 1.0) < 0.02,
                "with no focal length to start from, the second eye's turn on the first is"
                & Real'Image (Degrees (Angle (Times (Transposed (True_X.Rotation), C.Second_On_First.Rotation))))
                & " degrees off, the lenses" & C.First_Lens.Focal'Image & C.Second_Lens.Focal'Image);
      end;
      --  And with one still's first-eye matches the wrong motion's: that still left out, the frame as before.
      declare
         C : Calibration (Stills, N1, N2);
         True_X : constant Rigid := Inverse (Eye1) * Eye2;
      begin
         Calibrate (Points1 (1 .. N1), Points2 (1 .. N2), Corrupt (1 .. Nc), Stills,
                    (Focal => 1.2 * L1.Focal, Cx => L1.Cx, Cy => L1.Cy), (L2.Cx, L2.Cy), C);
         Check (C.Ok and then Degrees (Angle (Times (Transposed (True_X.Rotation), C.Second_On_First.Rotation))) < 0.5
                and then abs (C.First_Lens.Focal / L1.Focal - 1.0) < 0.02
                and then abs (C.Second_Lens.Focal / L2.Focal - 1.0) < 0.02,
                "with still" & Bad'Image & "'s first-eye matches the body stepped the other way, the second eye's turn"
                & " on the first is"
                & Real'Image (Degrees (Angle (Times (Transposed (True_X.Rotation), C.Second_On_First.Rotation))))
                & " degrees off, the lenses" & C.First_Lens.Focal'Image & C.Second_Lens.Focal'Image & ", Ok "
                & C.Ok'Image);
      end;
   end Walk_Calibrates;

   --  An eye 0.5 from a scene of points at depths 1 to 3 placed from 40 of them with a tenth of a pixel of noise, from
   --  a start turned 10 degrees and 0.1 off.
   procedure Locate_Places is
      L    : constant Lens := (Focal => 200.0, Cx => 160.0, Cy => 120.0);
      Truth : constant Rigid := (Rotation => Exp ([Radians (8.0), Radians (-12.0), Radians (5.0)]),
                                 Translation => [0.3, -0.2, 0.4]);
      Count : constant := 40;
      Rays  : Real_Matrix (1 .. Count, 1 .. 4);
      Seen_At : Place_Array (1 .. Count);
      Sigmas  : constant Real_Array (1 .. Count) := [others => 0.1];
      R : Locate_Result;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 11);
      for K in 1 .. Count loop
         declare
            Pixel : constant Place := (U => 40.0 + 240.0 * Real (Ada.Numerics.Float_Random.Random (Gen)),
                                       V => 30.0 + 180.0 * Real (Ada.Numerics.Float_Random.Random (Gen)));
            Ray_At : constant Vec3 := Ray (L, Pixel);
            Inverse_Depth : constant Real := 1.0 / (1.0 + 2.0 * Real (Ada.Numerics.Float_Random.Random (Gen)));
            P : Place;
            Ahead : Boolean;
         begin
            Rays (K, 1) := Ray_At (1);
            Rays (K, 2) := Ray_At (2);
            Rays (K, 3) := Ray_At (3);
            Rays (K, 4) := Inverse_Depth;
            Project (L, Seen (Truth, Ray_At, Inverse_Depth), P, Ahead);
            Seen_At (K) := (P.U + 0.1 * Gaussian, P.V + 0.1 * Gaussian);
         end;
      end loop;
      Locate (L, Rays, Seen_At, Sigmas,
              (Rotation => Times (Exp ([0.0, Radians (10.0), 0.0]), Truth.Rotation),
               Translation => Truth.Translation + [0.1, 0.0, 0.0]), R);
      Check (R.Ok, "40 points place no eye: misfit" & R.Misfit'Image);
      Check (Degrees (Angle (Times (Transposed (Truth.Rotation), R.Pose.Rotation))) < 0.1,
             "the eye's turn is" & Real'Image (Degrees (Angle (Times (Transposed (Truth.Rotation), R.Pose.Rotation))))
             & " degrees off");
      Check (abs (R.Pose.Translation - Truth.Translation) < 0.01,
             "the eye's place is" & Real'Image (abs (R.Pose.Translation - Truth.Translation)) & " off");
   end Locate_Places;

   procedure Register is
   begin
      Driver.Tests.Register ("rig.calibrate", "a walk's stills of two eyes carried as one do not tell where the second "
                             & "sits on the first within half a degree, or their lenses within 2 %, with a start or "
                             & "none, or with one still's matches of the first eye those of a wrong motion",
                             Walk_Calibrates'Access);
      Driver.Tests.Register ("rig.locate", "an eye is not placed from points whose places it sees",
                             Locate_Places'Access);
   end Register;

end Driver.Rig.Tests;
