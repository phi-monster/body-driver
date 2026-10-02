with Ada.Containers.Ordered_Sets;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Stats;

package body Driver.Robot.Hand.Lobes is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Images;

   Mad_Efficiency : constant := 0.367_5;
   --  The asymptotic efficiency of the median absolute deviation for Gaussian
   --  data: its scale is worth that share of as many degrees of freedom.

   package Natural_Vectors is new Ada.Containers.Vectors (Natural, Natural);

   --  Component labels of a mask, one per pixel (row-major), 0 off the mask.
   type Labels is record
      Width, Height : Natural := 0;
      Of_Pixel      : Natural_Vectors.Vector;
      Count         : Natural := 0;
   end record;

   function Index (W : Natural; Column, Row : Natural) return Natural is (Row * W + Column);

   function Pixel_Of (P : Pixel; Width, Height : Positive; Column, Row : out Natural) return Boolean;
   --  The pixel containing a continuous coordinate, if it is in the image.

   function Pixel_Of (P : Pixel; Width, Height : Positive; Column, Row : out Natural) return Boolean is
   begin
      Column := 0;
      Row := 0;
      if not (P.U >= 0.0 and then P.V >= 0.0 and then P.U < Real (Width) and then P.V < Real (Height)) then
         return False;
      end if;
      Column := Natural (Real'Floor (P.U));
      Row := Natural (Real'Floor (P.V));
      return Column < Width and then Row < Height;
   end Pixel_Of;

   function Noise_Of (Still : Correspondence_Array) return Matcher_Noise is
      Count : Natural := 0;
   begin
      for C of Still loop
         if C.Matched then
            Count := Count + 1;
         end if;
      end loop;
      if Count = 0 then
         return (Displacement      => Unknown,
                 Round_Trip        => Unknown,
                 Displacement_Gate => Vector_Gate (2),
                 Round_Trip_Gate   => Vector_Gate (2));
      end if;
      declare
         Moves, Trips : Real_Array (1 .. 2 * Count);
         K : Natural := 0;
      begin
         for C of Still loop
            if C.Matched then
               K := K + 1;
               Moves (K) := C.To.U - C.From.U;
               Trips (K) := C.Back.U - C.From.U;
               K := K + 1;
               Moves (K) := C.To.V - C.From.V;
               Trips (K) := C.Back.V - C.From.V;
            end if;
         end loop;
         declare
            Dof : constant Natural := Natural (Real'Floor (Mad_Efficiency * Real (2 * Count)));
         begin
            return (Displacement      => (Value => 0.0, Sigma => Driver.Stats.Robust_Sigma (Moves),
                                          Degrees_Of_Freedom => Dof),
                    Round_Trip        => (Value => 0.0, Sigma => Driver.Stats.Robust_Sigma (Trips),
                                          Degrees_Of_Freedom => Dof),
                    Displacement_Gate => Vector_Gate (2, Dof),
                    Round_Trip_Gate   => Vector_Gate (2, Dof));
         end;
      end;
   end Noise_Of;

   function Displacement (C : Correspondence) return Real is
     (Sqrt ((C.To.U - C.From.U) ** 2 + (C.To.V - C.From.V) ** 2));

   function Moves (C : Correspondence; Noise : Matcher_Noise) return Boolean is
     (C.Matched
      and then Significant (Noise.Displacement_Gate, Displacement (C), Noise.Displacement.Sigma)
      and then not Significant (Noise.Round_Trip_Gate, Sqrt ((C.Back.U - C.From.U) ** 2 + (C.Back.V - C.From.V) ** 2),
                                Noise.Round_Trip.Sigma));
   --  A displacement and a round trip are two-dimensional: their lengths are
   --  tested against the vector gate of their noise.

   function Moving (Matches : Correspondence_Array; Noise : Matcher_Noise; Width, Height : Positive) return Mask is
      M : Mask := Create (Width, Height);
      Column, Row : Natural;
   begin
      for C of Matches loop
         if Moves (C, Noise) and then Pixel_Of (C.From, Width, Height, Column, Row) then
            Include (M, Column, Row);
         end if;
      end loop;
      return M;
   end Moving;

   function Components (M : Mask) return Labels;
   --  Eight-connected components.

   function Components (M : Mask) return Labels is
      W : constant Natural := Width (M);
      H : constant Natural := Height (M);
      L : Labels := (Width => W, Height => H, Of_Pixel => Natural_Vectors.To_Vector (0, Ada.Containers.Count_Type (W * H)),
                     Count => 0);
      Stack : Natural_Vectors.Vector;
   begin
      for Row in 0 .. H - 1 loop
         for Column in 0 .. W - 1 loop
            if Contains (M, Column, Row) and then L.Of_Pixel (Index (W, Column, Row)) = 0 then
               L.Count := L.Count + 1;
               L.Of_Pixel.Replace_Element (Index (W, Column, Row), L.Count);
               Stack.Append (Index (W, Column, Row));
               while not Stack.Is_Empty loop
                  declare
                     P : constant Natural := Stack.Last_Element;
                     C : constant Integer := P mod W;
                     R : constant Integer := P / W;
                  begin
                     Stack.Delete_Last;
                     for Dr in -1 .. 1 loop
                        for Dc in -1 .. 1 loop
                           if C + Dc in 0 .. W - 1 and then R + Dr in 0 .. H - 1
                             and then Contains (M, C + Dc, R + Dr)
                             and then L.Of_Pixel (Index (W, C + Dc, R + Dr)) = 0
                           then
                              L.Of_Pixel.Replace_Element (Index (W, C + Dc, R + Dr), L.Count);
                              Stack.Append (Index (W, C + Dc, R + Dr));
                           end if;
                        end loop;
                     end loop;
                  end;
               end loop;
            end if;
         end loop;
      end loop;
      return L;
   end Components;

   function Label_At (L : Labels; P : Pixel) return Natural is
      Column, Row : Natural;
   begin
      if L.Width = 0 or else not Pixel_Of (P, L.Width, L.Height, Column, Row) then
         return 0;
      end if;
      return L.Of_Pixel (Index (L.Width, Column, Row));
   end Label_At;

   procedure Tip_Of (Lobe_Mask, Seeds, Attached : Mask; Tip : out Pixel; Known : out Boolean);
   --  The seed of the lobe farthest along the lobe from where it is attached
   --  (shortest paths through the lobe, steps of 1 and sqrt 2): a pixel that
   --  passed only the single test may be a still one next to the lobe.

   procedure Tip_Of (Lobe_Mask, Seeds, Attached : Mask; Tip : out Pixel; Known : out Boolean) is
      W : constant Natural := Width (Lobe_Mask);
      H : constant Natural := Height (Lobe_Mask);
      package Real_Vectors is new Ada.Containers.Vectors (Natural, Real);
      Dist : Real_Vectors.Vector := Real_Vectors.To_Vector (Real'Last, Ada.Containers.Count_Type (W * H));
      Queue : Natural_Vectors.Vector;
      Diagonal : constant Real := Sqrt (2.0);
      Has_Attached : constant Boolean := Width (Attached) = W and then Height (Attached) = H;

      function Attachment (C, R : Natural) return Boolean is
      begin
         if C = 0 or else R = 0 or else C = W - 1 or else R = H - 1 then
            return True;
         end if;
         if Has_Attached then
            for Dr in -1 .. 1 loop
               for Dc in -1 .. 1 loop
                  if Contains (Attached, C + Dc, R + Dr) and then not Contains (Lobe_Mask, C + Dc, R + Dr) then
                     return True;
                  end if;
               end loop;
            end loop;
         end if;
         return False;
      end Attachment;
   begin
      Tip := (U => 0.0, V => 0.0);
      Known := False;
      for R in 0 .. H - 1 loop
         for C in 0 .. W - 1 loop
            if Contains (Lobe_Mask, C, R) and then Attachment (C, R) then
               Dist.Replace_Element (Index (W, C, R), 0.0);
               Queue.Append (Index (W, C, R));
            end if;
         end loop;
      end loop;
      if Queue.Is_Empty then
         return;
      end if;
      --  Relax distances until no path through the lobe gets shorter (the
      --  queue holds every pixel whose distance just improved).
      declare
         Head : Natural := 0;
      begin
         while Head < Natural (Queue.Length) loop
            declare
               P : constant Natural := Queue (Head);
               C : constant Integer := P mod W;
               R : constant Integer := P / W;
            begin
               Head := Head + 1;
               for Dr in -1 .. 1 loop
                  for Dc in -1 .. 1 loop
                     if (Dr /= 0 or else Dc /= 0) and then C + Dc in 0 .. W - 1 and then R + Dr in 0 .. H - 1
                       and then Contains (Lobe_Mask, C + Dc, R + Dr)
                     then
                        declare
                           Q    : constant Natural := Index (W, C + Dc, R + Dr);
                           Step : constant Real := (if Dr /= 0 and then Dc /= 0 then Diagonal else 1.0);
                        begin
                           if Dist (P) + Step < Dist (Q) then
                              Dist.Replace_Element (Q, Dist (P) + Step);
                              Queue.Append (Q);
                           end if;
                        end;
                     end if;
                  end loop;
               end loop;
            end;
         end loop;
      end;
      declare
         Best : Real := Real'First;
      begin
         for R in 0 .. H - 1 loop
            for C in 0 .. W - 1 loop
               if Contains (Lobe_Mask, C, R) and then Contains (Seeds, C, R) and then Dist (Index (W, C, R)) < Real'Last
                 and then Dist (Index (W, C, R)) > Best
               then
                  Best := Dist (Index (W, C, R));
                  --  The pixel's centre.
                  Tip := (U => Real (C) + 0.5, V => Real (R) + 0.5);
                  Known := True;
               end if;
            end loop;
         end loop;
      end;
   end Tip_Of;

   procedure Summarize (M : Mask; Count : out Natural; Centre : out Pixel);

   procedure Summarize (M : Mask; Count : out Natural; Centre : out Pixel) is
      Su, Sv : Real := 0.0;
   begin
      Count := 0;
      for R in 0 .. Height (M) - 1 loop
         for C in 0 .. Width (M) - 1 loop
            if Contains (M, C, R) then
               Count := Count + 1;
               Su := Su + Real (C) + 0.5;
               Sv := Sv + Real (R) + 0.5;
            end if;
         end loop;
      end loop;
      Centre := (if Count > 0 then (U => Su / Real (Count), V => Sv / Real (Count)) else (U => 0.0, V => 0.0));
   end Summarize;

   function Tested (Matches : Correspondence_Array) return Natural is
      N : Natural := 0;
   begin
      for C of Matches loop
         N := N + Boolean'Pos (C.Matched);
      end loop;
      return N;
   end Tested;

   function Seeds (Matches : Correspondence_Array; Noise : Matcher_Noise; Family : Gate; Width, Height : Positive)
     return Mask
   is
      M : Mask := Create (Width, Height);
      Column, Row : Natural;
   begin
      for C of Matches loop
         if Moves (C, Noise) and then Significant (Family, Displacement (C), Noise.Displacement.Sigma)
           and then Pixel_Of (C.From, Width, Height, Column, Row)
         then
            Include (M, Column, Row);
         end if;
      end loop;
      return M;
   end Seeds;

   function Seeded_Components (M, Seeds : Mask) return Labels is
      --  The components of M that hold a seed, numbered from one; the others
      --  are not moving parts and get label zero.
      All_Of : Labels := Components (M);
      Seeded : array (1 .. All_Of.Count) of Boolean := [others => False];
      Number : array (1 .. All_Of.Count) of Natural := [others => 0];
      Kept   : Natural := 0;
   begin
      for R in 0 .. All_Of.Height - 1 loop
         for C in 0 .. All_Of.Width - 1 loop
            if All_Of.Of_Pixel (Index (All_Of.Width, C, R)) > 0 and then Contains (Seeds, C, R) then
               Seeded (All_Of.Of_Pixel (Index (All_Of.Width, C, R))) := True;
            end if;
         end loop;
      end loop;
      for L in Seeded'Range loop
         if Seeded (L) then
            Kept := Kept + 1;
            Number (L) := Kept;
         end if;
      end loop;
      for P in All_Of.Of_Pixel.First_Index .. All_Of.Of_Pixel.Last_Index loop
         if All_Of.Of_Pixel (P) > 0 then
            All_Of.Of_Pixel.Replace_Element (P, Number (All_Of.Of_Pixel (P)));
         end if;
      end loop;
      All_Of.Count := Kept;
      return All_Of;
   end Seeded_Components;

   function Find
     (Forward  : Correspondence_Array;
      Backward : Correspondence_Array;
      Noise    : Matcher_Noise;
      Attached : Mask;
      Width, Height : Positive) return Lobe_Vectors.Vector
   is
      --  Every pixel tested in either view is one test of the family
      --  "something moves here".
      Tests  : constant Natural := Tested (Forward) + Tested (Backward);
      Result : Lobe_Vectors.Vector;
   begin
      if Tests = 0 then
         return Result;
      end if;
      declare
         Family      : constant Gate := Vector_Gate (2, Noise.Displacement.Degrees_Of_Freedom, Tests => Tests);
         Seeds_Here  : constant Mask := Seeds (Forward, Noise, Family, Width, Height);
         Seeds_There : constant Mask := Seeds (Backward, Noise, Family, Width, Height);
         A : constant Labels := Seeded_Components (Moving (Forward, Noise, Width, Height), Seeds_Here);
         B : constant Labels := Seeded_Components (Moving (Backward, Noise, Width, Height), Seeds_There);
         --  A link (a, b): a moving pixel of component a of this view lands on
         --  component b of the other view, or the other way round. Few
         --  components link, so the links are kept as a set of pairs.
         type Link is record
            A_Label, B_Label : Positive;
         end record;
         function "<" (L, R : Link) return Boolean is
           (L.A_Label < R.A_Label or else (L.A_Label = R.A_Label and then L.B_Label < R.B_Label));
         package Link_Sets is new Ada.Containers.Ordered_Sets (Link);
         Links : Link_Sets.Set;
         function Linked (La, Lb : Positive) return Boolean is (Links.Contains ((A_Label => La, B_Label => Lb)));
         Group_A : array (1 .. A.Count) of Natural := [others => 0];
         Group_B : array (1 .. B.Count) of Natural := [others => 0];
         Groups  : Natural := 0;

         procedure Spread (From_A : Boolean; Index_Of : Positive; G : Positive);
         --  Marks everything linked to a component as group G.

         procedure Spread (From_A : Boolean; Index_Of : Positive; G : Positive) is
         begin
            if From_A then
               Group_A (Index_Of) := G;
               for Lb in 1 .. B.Count loop
                  if Linked (Index_Of, Lb) and then Group_B (Lb) = 0 then
                     Spread (False, Lb, G);
                  end if;
               end loop;
            else
               Group_B (Index_Of) := G;
               for La in 1 .. A.Count loop
                  if Linked (La, Index_Of) and then Group_A (La) = 0 then
                     Spread (True, La, G);
                  end if;
               end loop;
            end if;
         end Spread;
      begin
         for C of Forward loop
            declare
               La : constant Natural := Label_At (A, C.From);
               Lb : constant Natural := Label_At (B, C.To);
            begin
               if La > 0 and then Lb > 0 and then Moves (C, Noise) then
                  Links.Include ((A_Label => La, B_Label => Lb));
               end if;
            end;
         end loop;
         for C of Backward loop
            declare
               Lb : constant Natural := Label_At (B, C.From);
               La : constant Natural := Label_At (A, C.To);
            begin
               if La > 0 and then Lb > 0 and then Moves (C, Noise) then
                  Links.Include ((A_Label => La, B_Label => Lb));
               end if;
            end;
         end loop;
         --  Each group of linked components is one or more lobes: as many as
         --  the view where they are separate has components. A component
         --  linked to nothing in the other view is not a moving part.
         for La in 1 .. A.Count loop
            if Group_A (La) = 0 then
               for Lb in 1 .. B.Count loop
                  if Linked (La, Lb) then
                     Groups := Groups + 1;
                     Spread (True, La, Groups);
                     exit;
                  end if;
               end loop;
            end if;
         end loop;
         for G in 1 .. Groups loop
            declare
               In_A, In_B : Natural := 0;
            begin
               for La in 1 .. A.Count loop
                  In_A := In_A + Boolean'Pos (Group_A (La) = G);
               end loop;
               for Lb in 1 .. B.Count loop
                  In_B := In_B + Boolean'Pos (Group_B (Lb) = G);
               end loop;
               declare
                  Split_Here : constant Boolean := In_A >= In_B;
                  --  The lobes are the components of the view where they are
                  --  separate; the other view's pixels go to the lobe their
                  --  match lands in.
               begin
                  for Part in 1 .. (if Split_Here then A.Count else B.Count) loop
                     if (if Split_Here then Group_A (Part) = G else Group_B (Part) = G) then
                        declare
                           L : Lobe;
                        begin
                           L.Here := Create (Width, Height);
                           L.There := Create (Width, Height);
                           for C of Forward loop
                              declare
                                 Column, Row : Natural;
                                 La : constant Natural := Label_At (A, C.From);
                                 Lb : constant Natural := Label_At (B, C.To);
                              begin
                                 if La > 0 and then Group_A (La) = G
                                   and then (if Split_Here then La = Part else Lb = Part)
                                   and then Pixel_Of (C.From, Width, Height, Column, Row)
                                 then
                                    Include (L.Here, Column, Row);
                                 end if;
                              end;
                           end loop;
                           for C of Backward loop
                              declare
                                 Column, Row : Natural;
                                 Lb : constant Natural := Label_At (B, C.From);
                                 La : constant Natural := Label_At (A, C.To);
                              begin
                                 if Lb > 0 and then Group_B (Lb) = G
                                   and then (if Split_Here then La = Part else Lb = Part)
                                   and then Pixel_Of (C.From, Width, Height, Column, Row)
                                 then
                                    Include (L.There, Column, Row);
                                 end if;
                              end;
                           end loop;
                           Summarize (L.Here, L.Count_Here, L.Centre_Here);
                           Summarize (L.There, L.Count_There, L.Centre_There);
                           if L.Count_Here > 0 and then L.Count_There > 0 then
                              Tip_Of (L.Here, Seeds_Here, Attached, L.Tip_Here, L.Tip_Known_Here);
                              Tip_Of (L.There, Seeds_There, Attached, L.Tip_There, L.Tip_Known_There);
                              Result.Append (L);
                           end if;
                        end;
                     end if;
                  end loop;
               end;
            end;
         end loop;
      end;
      return Result;
   end Find;

   function Direction (Lobes : Lobe_Vectors.Vector; Attached : Mask; Noise : Matcher_Noise) return Closing is
      --  A centre of n pixels moves by less than its pixels' own noise, which
      --  is the matcher's and the pixel grid's (a uniform square of side one).
      Pixel_Variance : constant Real := Noise.Displacement.Sigma ** 2 + 1.0 / 12.0;
      Change   : Real := 0.0;
      Variance : Real := 0.0;
   begin
      if Natural (Lobes.Length) >= 2 then
         for I in Lobes.First_Index .. Lobes.Last_Index loop
            for J in I + 1 .. Lobes.Last_Index loop
               declare
                  Li : constant Lobe := Lobes (I);
                  Lj : constant Lobe := Lobes (J);
                  Here  : constant Real := Sqrt ((Li.Centre_Here.U - Lj.Centre_Here.U) ** 2
                                                 + (Li.Centre_Here.V - Lj.Centre_Here.V) ** 2);
                  There : constant Real := Sqrt ((Li.Centre_There.U - Lj.Centre_There.U) ** 2
                                                 + (Li.Centre_There.V - Lj.Centre_There.V) ** 2);
               begin
                  Change := Change + (There - Here);
                  Variance := Variance + Pixel_Variance * (1.0 / Real (Li.Count_Here) + 1.0 / Real (Lj.Count_Here)
                                                           + 1.0 / Real (Li.Count_There) + 1.0 / Real (Lj.Count_There));
               end;
            end loop;
         end loop;
      elsif Natural (Lobes.Length) = 1 and then Width (Attached) > 0 and then Count (Attached) > 0 then
         --  One lobe closes against what it is attached to: the distance from
         --  its centre to the nearest still pixel of the robot.
         declare
            L : constant Lobe := Lobes (Lobes.First_Index);
            function Nearest (P : Pixel) return Real is
               Best : Real := Real'Last;
            begin
               for R in 0 .. Height (Attached) - 1 loop
                  for C in 0 .. Width (Attached) - 1 loop
                     if Contains (Attached, C, R) then
                        Best := Real'Min (Best, Sqrt ((Real (C) + 0.5 - P.U) ** 2 + (Real (R) + 0.5 - P.V) ** 2));
                     end if;
                  end loop;
               end loop;
               return Best;
            end Nearest;
         begin
            Change := Nearest (L.Centre_There) - Nearest (L.Centre_Here);
            Variance := Pixel_Variance * (1.0 / Real (L.Count_Here) + 1.0 / Real (L.Count_There));
         end;
      else
         return Undecided;
      end if;
      if not Significant (Change, Sqrt (Variance), Noise.Displacement.Degrees_Of_Freedom) then
         return Undecided;
      end if;
      return (if Change < 0.0 then Towards_There else Towards_Here);
   end Direction;

end Driver.Robot.Hand.Lobes;
