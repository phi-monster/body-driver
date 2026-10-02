with Ada.Exceptions;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Driver.Bytes;
with Driver.Tests;

package body Driver.Robot.Hand.Sweep.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;
   use type Driver.Clock.Beat;
   use type Driver.Bytes.Offset;

   W : constant := 160;
   H : constant := 120;

   function At_Beat (B : Driver.Clock.Beat) return Observation is ((Beat => B, others => <>));
   --  The sweep reads only the beat; its readings and image are passed beside it.

   --  Two dark fingers enter from the bottom border; at closer reading R
   --  (1 open, 0 closed) each has moved (1 - R) * 45 columns inwards. A view
   --  Scale times as wide and high shows the same, Scale times as large.
   function Left_Edge (Finger : Positive; R : Real; Scale : Positive := 1) return Integer is
     (Scale * (if Finger = 1 then 10 + Integer (45.0 * (1.0 - R)) else 130 - Integer (45.0 * (1.0 - R))));

   function On_Finger (C, Row : Natural; R : Real; Scale : Positive := 1) return Natural is
   begin
      for F in 1 .. 2 loop
         if Row >= Scale * 40
           and then Integer (C) in Left_Edge (F, R, Scale) .. Left_Edge (F, R, Scale) + Scale * 20 - 1
         then
            return F;
         end if;
      end loop;
      return 0;
   end On_Finger;

   function Frame (R : Real; Scale : Positive := 1) return Driver.Images.Image is
      Data : Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset (3 * Scale * W * Scale * H));
   begin
      for Row in 0 .. Scale * H - 1 loop
         for C in 0 .. Scale * W - 1 loop
            declare
               --  A textured table, so every background pixel has its own value.
               Table : constant Natural :=
                 Natural (128.0 + 60.0 * Sin (Real (C) * 0.37) * Cos (Real (Row) * 0.23) + Real ((C * 7 + Row * 13) mod 19));
               L : constant Driver.Bytes.Byte :=
                 Driver.Bytes.Byte (if On_Finger (C, Row, R, Scale) > 0 then 20 else Table);
               K : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Row * Scale * W + C));
            begin
               Data (K + 1) := L;
               Data (K + 2) := L;
               Data (K + 3) := L;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (Scale * W, Scale * H, Data);
   end Frame;

   --  The matcher's answers for points of the view at reading From, looked
   --  for in the view at reading To.
   function Answers (Points : Driver.Instrument.Point_Array; From, To : Real; Scale : Positive := 1)
     return Driver.Instrument.Answer_Array is
   begin
      return Result : Driver.Instrument.Answer_Array (Points'Range) do
         for K in Points'Range loop
            declare
               P : constant Driver.Images.Pixel := Points (K);
               C : constant Natural := Natural (Real'Floor (P.U));
               R : constant Natural := Natural (Real'Floor (P.V));
               F : constant Natural := On_Finger (C, R, From, Scale);
            begin
               if F > 0 then
                  Result (K) := (Found => True, Certainty => 1.0, Back => P,
                                 To => (U => P.U + Real (Left_Edge (F, To, Scale) - Left_Edge (F, From, Scale)),
                                        V => P.V));
               elsif On_Finger (C, R, To, Scale) > 0 then
                  --  Covered in the other view: nowhere to go, no way back.
                  Result (K) := (Found => True, Certainty => 0.1, To => (U => P.U + 7.0, V => P.V),
                                 Back => (U => P.U + 11.0, V => P.V + 3.0));
               else
                  Result (K) := (Found => True, Certainty => 1.0, To => P, Back => P);
               end if;
            end;
         end loop;
      end return;
   end Answers;

   procedure Two_Fingers_Swept is
      S : State := Start (W, H, Channels => 1, Closer_Noise => [1 => 0.0], Rest_Noise => [1 => 0.0]);
      B : Driver.Clock.Beat := 0;
      procedure Hold (R : Real; Frames : Positive) is
      begin
         for I in 1 .. Frames loop
            Observe (S, At_Beat (B), True, [1 => R], [1 => 0.0], Frame (R));
            B := B + 1;
         end loop;
      end Hold;
      procedure Move (R : Real) is
      begin
         Observe (S, At_Beat (B), False, [1 => R], [1 => 0.0], Frame (R));
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

   procedure Swept_In_A_Task is
      --  The hand's estimate runs inside the decider's task, whose stack is
      --  GNAT's default: the closer swept open to closed and back in a VGA
      --  eye the fingers fill much of, the change between its ends asked of
      --  the matcher pixel by pixel both ways round, and the lobes found from
      --  the answers. Every per-pixel quantity of that is megabytes.
      Scale    : constant := 4;
      Readings : constant Real_Array := [1.0, 0.7, 0.5, 0.2, 0.0];
      type Frame_Array is array (Readings'Range) of Driver.Images.Image;
      Frames   : constant Frame_Array := [for I in Readings'Range => Frame (Readings (I), Scale)];
      S        : State := Start (Scale * W, Scale * H, Channels => 1, Closer_Noise => [1 => 0.0],
                                 Rest_Noise => [1 => 0.0]);
      Done     : Boolean := False with Atomic;
      Asked_At : Natural := 0;
      Found    : Natural := 0;
      Failure  : Ada.Strings.Unbounded.Unbounded_String;

      function Frame_At (R : Real) return Driver.Images.Image is
      begin
         for I in Readings'Range loop
            if Readings (I) = R then
               return Frames (I);
            end if;
         end loop;
         raise Program_Error with "no frame drawn at reading" & R'Image;
      end Frame_At;
   begin
      declare
         task Decider;
         task body Decider is
            B : Driver.Clock.Beat := 0;
            procedure Hold (R : Real; Count : Positive) is
            begin
               for I in 1 .. Count loop
                  Observe (S, At_Beat (B), True, [1 => R], [1 => 0.0], Frame_At (R));
                  B := B + 1;
               end loop;
            end Hold;
            procedure Move (R : Real) is
            begin
               Observe (S, At_Beat (B), False, [1 => R], [1 => 0.0], Frame_At (R));
               B := B + 1;
            end Move;
         begin
            Hold (1.0, 3);
            Move (0.7);
            Hold (0.5, 2);
            Move (0.2);
            Hold (0.0, 3);
            Move (0.5);
            Hold (1.0, 2);
            Move (1.0);
            if Wants_Correspondences (S, 1) then
               declare
                  Points : constant Driver.Instrument.Point_Array := Query_Points (S, 1);
                  Low    : constant Real := Views.Reading (Low_End (S, 1), 1);
                  High   : constant Real := Views.Reading (High_End (S, 1), 1);
               begin
                  Asked_At := Points'Length;
                  Asked (S, 1);
                  Answer (S, 1, Points, Answers (Points, Low, High, Scale), Answers (Points, High, Low, Scale),
                          Driver.Images.Create (Scale * W, Scale * H));
               end;
               if Status (S, 1) = Measured then
                  Found := Natural (Lobes_Of (S, 1).Length);
               end if;
            end if;
            Done := True;
         exception
            when E : others =>
               Failure := Ada.Strings.Unbounded.To_Unbounded_String (Ada.Exceptions.Exception_Information (E));
         end Decider;
      begin
         null;
      end;
      Check (Done, "the sweep's estimate failed in a task with the default stack: "
             & Ada.Strings.Unbounded.To_String (Failure));
      Check (not Done or else Found = 2, "two fingers in a VGA view gave" & Found'Image & " lobes, from"
             & Asked_At'Image & " pixels asked");
   end Swept_In_A_Task;

   procedure Nothing_Seen is
      --  A channel whose push changes nothing this eye sees.
      S : State := Start (W, H, Channels => 1, Closer_Noise => [1 => 0.0], Rest_Noise => [1 => 0.0]);
   begin
      for I in 1 .. 3 loop
         Observe (S, At_Beat (Driver.Clock.Beat (I)), True, [1 => 1.0], [1 => 0.0], Frame (1.0));
      end loop;
      Observe (S, At_Beat (4), False, [1 => 0.5], [1 => 0.0], Frame (1.0));
      for I in 5 .. 7 loop
         Observe (S, At_Beat (Driver.Clock.Beat (I)), True, [1 => 0.0], [1 => 0.0], Frame (1.0));
      end loop;
      Observe (S, At_Beat (8), False, [1 => 0.5], [1 => 0.0], Frame (1.0));
      Check (Status (S, 1) = Nothing_Moves and then not Wants_Correspondences (S, 1),
             "a push that changes nothing asked the matcher");
   end Nothing_Seen;

   procedure Refused (Lasting : Boolean) is
      --  A sweep asked, refused, and swept again with new ends.
      S : State := Start (W, H, Channels => 1, Closer_Noise => [1 => 0.0], Rest_Noise => [1 => 0.0]);
      B : Driver.Clock.Beat := 0;
      Rest_At : Real := 0.0;   --  the rest of the body, which the boot moves between sweeps
      procedure Hold (R : Real; Frames : Positive) is
      begin
         for I in 1 .. Frames loop
            Observe (S, At_Beat (B), True, [1 => R], [1 => Rest_At], Frame (R));
            B := B + 1;
         end loop;
      end Hold;
      procedure Move (R : Real) is
      begin
         Observe (S, At_Beat (B), False, [1 => R], [1 => Rest_At], Frame (R));
         B := B + 1;
      end Move;
      procedure Sweep is
      begin
         Hold (1.0, 3);
         Move (0.5);
         Hold (0.0, 3);
         Move (0.5);
         Hold (1.0, 3);
         Move (1.0);
      end Sweep;
   begin
      Sweep;
      Check (Wants_Correspondences (S, 1), "a full sweep did not ask for correspondences");
      if Wants_Correspondences (S, 1) then
         Asked (S, 1);
         Refuse (S, 1, Lasting, "no address was given for the instrument service");
         --  The arm moved, and the closer was swept again there: new ends.
         Rest_At := 0.5;
         Sweep;
         if Lasting then
            Check (Status (S, 1) = Unanswerable and then not Wants_Correspondences (S, 1)
                   and then Refusal (S) = "no address was given for the instrument service",
                   "an instrument that can never answer is asked again for new ends, or the reason is lost");
         else
            Check (Wants_Correspondences (S, 1) and then Refusal (S) = "",
                   "new ends are not asked for after a refusal that may pass");
         end if;
      end if;
   end Refused;

   procedure Refused_For_Good is
   begin
      Refused (Lasting => True);
   end Refused_For_Good;

   procedure Refused_For_Now is
   begin
      Refused (Lasting => False);
   end Refused_For_Now;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.sweep.lasting",
                             "an instrument that can never answer is asked again whenever the closer's ends are new",
                             Refused_For_Good'Access);
      Driver.Tests.Register ("hand.sweep.transient", "new ends are not asked for after a refusal that may pass",
                             Refused_For_Now'Access);
      Driver.Tests.Register ("hand.sweep.two", "a closer swept open to closed does not yield its lobes and closed end",
                             Two_Fingers_Swept'Access);
      Driver.Tests.Register ("hand.sweep.task", "the sweep's estimate fails in a task with the default stack, as the "
                             & "decider's does", Swept_In_A_Task'Access);
      Driver.Tests.Register ("hand.sweep.nothing", "a push that changes nothing in the eye is sent to the matcher",
                             Nothing_Seen'Access);
   end Register;

end Driver.Robot.Hand.Sweep.Tests;
