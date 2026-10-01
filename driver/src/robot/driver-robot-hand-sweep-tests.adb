with Ada.Numerics.Long_Elementary_Functions;
with Driver.Bytes;
with Driver.Tests;

package body Driver.Robot.Hand.Sweep.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;
   use type Driver.Clock.Beat;
   use type Driver.Bytes.Offset;

   W : constant := 160;
   H : constant := 120;

   --  Two dark fingers enter from the bottom border; at closer reading R
   --  (1 open, 0 closed) each has moved (1 - R) * 45 columns inwards.
   function Left_Edge (Finger : Positive; R : Real) return Integer is
     (if Finger = 1 then 10 + Integer (45.0 * (1.0 - R)) else 130 - Integer (45.0 * (1.0 - R)));

   function On_Finger (C, Row : Natural; R : Real) return Natural is
   begin
      for F in 1 .. 2 loop
         if Row >= 40 and then Integer (C) in Left_Edge (F, R) .. Left_Edge (F, R) + 19 then
            return F;
         end if;
      end loop;
      return 0;
   end On_Finger;

   function Frame (R : Real) return Driver.Images.Image is
      Data : Driver.Bytes.Byte_Array (1 .. 3 * W * H);
   begin
      for Row in 0 .. H - 1 loop
         for C in 0 .. W - 1 loop
            declare
               --  A textured table, so every background pixel has its own value.
               Table : constant Natural :=
                 Natural (128.0 + 60.0 * Sin (Real (C) * 0.37) * Cos (Real (Row) * 0.23) + Real ((C * 7 + Row * 13) mod 19));
               L : constant Driver.Bytes.Byte := Driver.Bytes.Byte (if On_Finger (C, Row, R) > 0 then 20 else Table);
               K : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Row * W + C));
            begin
               Data (K + 1) := L;
               Data (K + 2) := L;
               Data (K + 3) := L;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (W, H, Data);
   end Frame;

   --  The matcher's answers for points of the view at reading From, looked
   --  for in the view at reading To.
   function Answers (Points : Driver.Instrument.Point_Array; From, To : Real) return Driver.Instrument.Answer_Array is
      Result : Driver.Instrument.Answer_Array (Points'Range);
   begin
      for K in Points'Range loop
         declare
            P : constant Driver.Images.Pixel := Points (K);
            C : constant Natural := Natural (Real'Floor (P.U));
            R : constant Natural := Natural (Real'Floor (P.V));
            F : constant Natural := On_Finger (C, R, From);
         begin
            if F > 0 then
               Result (K) := (Found => True, Certainty => 1.0, Back => P,
                              To => (U => P.U + Real (Left_Edge (F, To) - Left_Edge (F, From)), V => P.V));
            elsif On_Finger (C, R, To) > 0 then
               --  Covered in the other view: nowhere to go, no way back.
               Result (K) := (Found => True, Certainty => 0.1, To => (U => P.U + 7.0, V => P.V),
                              Back => (U => P.U + 11.0, V => P.V + 3.0));
            else
               Result (K) := (Found => True, Certainty => 1.0, To => P, Back => P);
            end if;
         end;
      end loop;
      return Result;
   end Answers;

   procedure Two_Fingers_Swept is
      S : State := Start (W, H, Channels => 1, Closer_Noise => [1 => 0.0], Arm_Noise => [1 => 0.0]);
      B : Driver.Clock.Beat := 0;
      procedure Hold (R : Real; Frames : Positive) is
      begin
         for I in 1 .. Frames loop
            Observe (S, B, True, [1 => R], [1 => 0.0], Frame (R));
            B := B + 1;
         end loop;
      end Hold;
      procedure Move (R : Real) is
      begin
         Observe (S, B, False, [1 => R], [1 => 0.0], Frame (R));
         B := B + 1;
      end Move;
   begin
      --  Open, then pushed closed in two steps, then opened again.
      Hold (1.0, 3);
      Move (0.7);
      Hold (0.5, 2);
      Move (0.2);
      Hold (0.0, 3);
      Move (0.5);
      Hold (1.0, 2);
      Move (1.0);
      Check (Wants_Correspondences (S, 1), "a full sweep did not ask for correspondences");
      if Wants_Correspondences (S, 1) then
         declare
            Points : constant Driver.Instrument.Point_Array := Query_Points (S, 1);
            Low  : constant Real := Views.Reading (Low_End (S, 1), 1);
            High : constant Real := Views.Reading (High_End (S, 1), 1);
         begin
            Check (Low = 0.0 and then High = 1.0, "the ends are not closed and open");
            --  The box of the change spans both fingers' travel from the top row down.
            Check (Points'Length = (149 - 10 + 1) * (H - 40), "the query box is not the change's box:"
                   & Natural'Image (Points'Length));
            Asked (S, 1);
            Check (Status (S, 1) = Requested and then not Wants_Correspondences (S, 1), "asking was not recorded");
            Answer (S, 1, Points, Answers (Points, Low, High), Answers (Points, High, Low), Driver.Images.Create (W, H));
         end;
         Check (Status (S, 1) = Measured, "the answers gave no lobes");
         if Status (S, 1) = Measured then
            Check (Natural (Lobes_Of (S, 1).Length) = 2, "two fingers gave"
                   & Natural'Image (Natural (Lobes_Of (S, 1).Length)) & " lobes");
            Check (Closing_Known (S, 1) and then not Closed_End_Is_High (S, 1),
                   "the closed end is not the low reading");
         end if;
      end if;
   end Two_Fingers_Swept;

   procedure Nothing_Seen is
      --  A channel whose push changes nothing this eye sees.
      S : State := Start (W, H, Channels => 1, Closer_Noise => [1 => 0.0], Arm_Noise => [1 => 0.0]);
   begin
      for I in 1 .. 3 loop
         Observe (S, Driver.Clock.Beat (I), True, [1 => 1.0], [1 => 0.0], Frame (1.0));
      end loop;
      Observe (S, 4, False, [1 => 0.5], [1 => 0.0], Frame (1.0));
      for I in 5 .. 7 loop
         Observe (S, Driver.Clock.Beat (I), True, [1 => 0.0], [1 => 0.0], Frame (1.0));
      end loop;
      Observe (S, 8, False, [1 => 0.5], [1 => 0.0], Frame (1.0));
      Check (Status (S, 1) = Nothing_Moves and then not Wants_Correspondences (S, 1),
             "a push that changes nothing asked the matcher");
   end Nothing_Seen;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.sweep.two", "a closer swept open to closed does not yield its lobes and closed end",
                             Two_Fingers_Swept'Access);
      Driver.Tests.Register ("hand.sweep.nothing", "a push that changes nothing in the eye is sent to the matcher",
                             Nothing_Seen'Access);
   end Register;

end Driver.Robot.Hand.Sweep.Tests;
