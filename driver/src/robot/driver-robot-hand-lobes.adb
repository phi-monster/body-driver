with Ada.Containers.Generic_Array_Sort;
with Ada.Containers.Ordered_Sets;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Stats;

package body Driver.Robot.Hand.Lobes is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Images;

   package Natural_Vectors is new Ada.Containers.Vectors (Natural, Natural);

   --  Everything sized by pixels or components lives on the heap: the
   --  estimates also run in the decider's task, whose stack is small.
   type Real_Access is access Real_Array;
   subtype Count_Array is Places;
   type Count_Access is access Count_Array;
   subtype Flag_Array is Flags;
   type Flags_Access is access Flag_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Real_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Count_Array, Count_Access);
   procedure Free is new Ada.Unchecked_Deallocation (Flag_Array, Flags_Access);

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
         Moves : Real_Access := new Real_Array (1 .. 2 * Count);
         Trips : Real_Access := new Real_Array (1 .. 2 * Count);
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
            Moves_Sigma : constant Real := Driver.Stats.Robust_Sigma (Moves.all);
            Trips_Sigma : constant Real := Driver.Stats.Robust_Sigma (Trips.all);
         begin
            Free (Moves);
            Free (Trips);
            return (Displacement      => (Value => 0.0, Sigma => Moves_Sigma,
                                          Degrees_Of_Freedom => Dof),
                    Round_Trip        => (Value => 0.0, Sigma => Trips_Sigma,
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

   package Move_Vectors is new Ada.Containers.Vectors (Positive, Move);

   function To_Array (V : Move_Vectors.Vector) return Move_Array is
   begin
      --  Built where it is returned, off the stack: a lobe has as many moves as pixels.
      return Result : Move_Array (1 .. Natural (V.Length)) do
         for I in Result'Range loop
            Result (I) := V (I);
         end loop;
      end return;
   end To_Array;

   function Index_Of (V : Move_Vectors.Vector; From : Pixel) return Natural is
   begin
      for I in V.First_Index .. V.Last_Index loop
         if V (I).From = From then
            return I;
         end if;
      end loop;
      return 0;
   end Index_Of;
   --  Both are pixel centres, so equal ones are the same pixel.

   function On_Border (M : Mask) return Boolean is
      W : constant Natural := Width (M);
      H : constant Natural := Height (M);
   begin
      for C in 0 .. W - 1 loop
         if Contains (M, C, 0) or else Contains (M, C, H - 1) then
            return True;
         end if;
      end loop;
      for R in 0 .. H - 1 loop
         if Contains (M, 0, R) or else Contains (M, W - 1, R) then
            return True;
         end if;
      end loop;
      return False;
   end On_Border;

   function Seeded_Components (M, Seeds : Mask) return Labels is
      --  The components of M that hold a seed, numbered from one; the others
      --  are not moving parts and get label zero.
      All_Of : Labels := Components (M);
      Seeded : Flags_Access := new Flag_Array'(1 .. All_Of.Count => False);
      Number : Count_Access := new Count_Array'(1 .. All_Of.Count => 0);
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
      Free (Seeded);
      Free (Number);
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
         Group_A : Count_Access := new Count_Array'(1 .. A.Count => 0);
         Group_B : Count_Access := new Count_Array'(1 .. B.Count => 0);
         Groups  : Natural := 0;

         procedure Spread (From_A : Boolean; Index_Of : Positive; G : Positive);
         --  Marks everything linked to a component as group G: by a list of
         --  the components still to visit, not by recursion, as the chain of
         --  links can be as long as there are components.

         procedure Spread (From_A : Boolean; Index_Of : Positive; G : Positive) is
            type Visit is record
               In_A  : Boolean;
               Label : Positive;
            end record;
            package Visit_Vectors is new Ada.Containers.Vectors (Positive, Visit);
            To_Visit : Visit_Vectors.Vector;
         begin
            if From_A then
               Group_A (Index_Of) := G;
            else
               Group_B (Index_Of) := G;
            end if;
            To_Visit.Append (Visit'(In_A => From_A, Label => Index_Of));
            while not To_Visit.Is_Empty loop
               declare
                  V : constant Visit := To_Visit.Last_Element;
               begin
                  To_Visit.Delete_Last;
                  if V.In_A then
                     for Lb in 1 .. B.Count loop
                        if Linked (V.Label, Lb) and then Group_B (Lb) = 0 then
                           Group_B (Lb) := G;
                           To_Visit.Append (Visit'(In_A => False, Label => Lb));
                        end if;
                     end loop;
                  else
                     for La in 1 .. A.Count loop
                        if Linked (La, V.Label) and then Group_A (La) = 0 then
                           Group_A (La) := G;
                           To_Visit.Append (Visit'(In_A => True, Label => La));
                        end if;
                     end loop;
                  end if;
               end;
            end loop;
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
                           Ahead, Behind : Move_Vectors.Vector;
                        begin
                           L.Here := Create (Width, Height);
                           L.There := Create (Width, Height);
                           --  A pixel of a moving component moved by the single
                           --  test and came back: its match is where it went.
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
                                    Ahead.Append (Move'(From => C.From, To => C.To));
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
                                    Behind.Append (Move'(From => C.From, To => C.To));
                                 end if;
                              end;
                           end loop;
                           Summarize (L.Here, L.Count_Here, L.Centre_Here);
                           Summarize (L.There, L.Count_There, L.Centre_There);
                           if L.Count_Here > 0 and then L.Count_There > 0 then
                              Tip_Of (L.Here, Seeds_Here, Attached, L.Tip_Here, L.Tip_Known_Here);
                              Tip_Of (L.There, Seeds_There, Attached, L.Tip_There, L.Tip_Known_There);
                              L.Moves_Here := Move_Holders.To_Holder (To_Array (Ahead));
                              L.Moves_There := Move_Holders.To_Holder (To_Array (Behind));
                              L.Tip_Move_Here := (if L.Tip_Known_Here then Index_Of (Ahead, L.Tip_Here) else 0);
                              L.Tip_Move_There := (if L.Tip_Known_There then Index_Of (Behind, L.Tip_There) else 0);
                              L.Bordered_Here := On_Border (L.Here);
                              L.Bordered_There := On_Border (L.There);
                              Result.Append (L);
                           end if;
                        end;
                     end if;
                  end loop;
               end;
            end;
         end loop;
         Free (Group_A);
         Free (Group_B);
      end;
      return Result;
   end Find;

   procedure Sort_Reals is new Ada.Containers.Generic_Array_Sort (Positive, Real, Real_Array);

   procedure Cut_In_Two (Values : Real_Array; Cut : out Real; Share : out Real; Cuttable : out Boolean);
   --  Otsu's cut: the value that parts Values in two groups with the least
   --  variance within them; Share is the part of the variance of all of them
   --  that their two means explain. Not cuttable when they are all the same.

   procedure Cut_In_Two (Values : Real_Array; Cut : out Real; Share : out Real; Cuttable : out Boolean) is
      N      : constant Natural := Values'Length;
      Sorted : Real_Access := new Real_Array'(Values);
      Total  : Real := 0.0;
      Mean   : Real;
      Spread : Real := 0.0;
      Left   : Real := 0.0;
      Best   : Real := -1.0;
   begin
      Cut := 0.0;
      Share := 0.0;
      Cuttable := False;
      if N >= 2 then
         Sort_Reals (Sorted.all);
         for V of Sorted.all loop
            Total := Total + V;
         end loop;
         Mean := Total / Real (N);
         for V of Sorted.all loop
            Spread := Spread + (V - Mean) ** 2;
         end loop;
         Spread := Spread / Real (N);
         if Spread > 0.0 then
            for K in 1 .. N - 1 loop
               Left := Left + Sorted (K);
               if Sorted (K) < Sorted (K + 1) then
                  declare
                     Low     : constant Real := Real (K) / Real (N);
                     Between : constant Real :=
                       Low * (1.0 - Low) * (Left / Real (K) - (Total - Left) / Real (N - K)) ** 2;
                  begin
                     if Between > Best then
                        Best := Between;
                        Cut := (Sorted (K) + Sorted (K + 1)) / 2.0;
                     end if;
                  end;
               end if;
            end loop;
            Cuttable := Best >= 0.0;
            Share := Best / Spread;
         end if;
      end if;
      Free (Sorted);
   end Cut_In_Two;

   function Opened (M : Mask) return Mask;
   --  What is left of the set when the pixels one pixel across are taken off:
   --  those with a neighbour (of the eight) outside the set, then the pixels
   --  of the set that touch what is left. The image's border is taken to be
   --  the same beyond itself: a part that comes in from the border keeps it.

   function Opened (M : Mask) return Mask is
      W      : constant Natural := Width (M);
      H      : constant Natural := Height (M);
      Core   : Mask := Create (W, H);
      Result : Mask := Create (W, H);
      function Inside (C, R : Integer) return Boolean is
        (Contains (M, Integer'Max (0, Integer'Min (W - 1, C)), Integer'Max (0, Integer'Min (H - 1, R))));
   begin
      for R in 0 .. H - 1 loop
         for C in 0 .. W - 1 loop
            if Contains (M, C, R) and then (for all Dr in -1 .. 1 => (for all Dc in -1 .. 1 => Inside (C + Dc, R + Dr)))
            then
               Include (Core, C, R);
            end if;
         end loop;
      end loop;
      for R in 0 .. H - 1 loop
         for C in 0 .. W - 1 loop
            if Contains (M, C, R) then
               declare
                  Touches : Boolean := False;
               begin
                  for Dr in -1 .. 1 loop
                     for Dc in -1 .. 1 loop
                        if C + Dc in 0 .. W - 1 and then R + Dr in 0 .. H - 1 and then Contains (Core, C + Dc, R + Dr) then
                           Touches := True;
                        end if;
                     end loop;
                  end loop;
                  if Touches then
                     Include (Result, C, R);
                  end if;
               end;
            end if;
         end loop;
      end loop;
      return Result;
   end Opened;

   --  One eight-connected part of a set of pixels.
   type Part is record
      Count    : Natural := 0;
      Su, Sv   : Real := 0.0;
      Attached : Boolean := False;   --  some pixel on the image's border, or next to an attached one
      Kept     : Natural := 0;       --  its number among the parts kept, 0 when dropped
   end record;

   type Part_Array is array (Positive range <>) of Part;
   type Part_Access is access Part_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Part_Array, Part_Access);

   function Centre_Of (P : Part) return Pixel is (U => P.Su / Real (P.Count), V => P.Sv / Real (P.Count));

   function Parts_Of (L : Labels; Attached : Mask; Doubt : Real) return Part_Access;
   --  Each part's pixels counted, its centre summed, and whether it is attached:
   --  on the image's border, or touching the robot's pixels that did not
   --  change (as Tip_Of takes an attachment). The kept ones are numbered: the
   --  attached ones that hold more pixels than Doubt, the number of changed
   --  pixels expected to have been given to the wrong end; a part no larger
   --  could be made of nothing but those.

   function Parts_Of (L : Labels; Attached : Mask; Doubt : Real) return Part_Access is
      W      : constant Natural := L.Width;
      H      : constant Natural := L.Height;
      Result : constant Part_Access := new Part_Array (1 .. L.Count);
      Has    : constant Boolean := Width (Attached) = W and then Height (Attached) = H;
      Kept   : Natural := 0;
   begin
      for R in 0 .. H - 1 loop
         for C in 0 .. W - 1 loop
            declare
               K : constant Natural := L.Of_Pixel (Index (W, C, R));
            begin
               if K > 0 then
                  Result (K).Count := Result (K).Count + 1;
                  Result (K).Su := Result (K).Su + Real (C) + 0.5;
                  Result (K).Sv := Result (K).Sv + Real (R) + 0.5;
                  if C = 0 or else R = 0 or else C = W - 1 or else R = H - 1 then
                     Result (K).Attached := True;
                  elsif Has and then not Result (K).Attached then
                     for Dr in -1 .. 1 loop
                        for Dc in -1 .. 1 loop
                           if Contains (Attached, C + Dc, R + Dr) and then L.Of_Pixel (Index (W, C + Dc, R + Dr)) /= K then
                              Result (K).Attached := True;
                           end if;
                        end loop;
                     end loop;
                  end if;
               end if;
            end;
         end loop;
      end loop;
      for K in Result'Range loop
         if Result (K).Attached and then Real (Result (K).Count) > Doubt then
            Kept := Kept + 1;
            Result (K).Kept := Kept;
         end if;
      end loop;
      return Result;
   end Parts_Of;

   type Kinds_Access is access Kinds;
   procedure Free is new Ada.Unchecked_Deallocation (Kinds, Kinds_Access);

   procedure Tell_Ends
     (W, H     : Positive;
      At_Pixel : Count_Array;
      Seeds    : Flag_Array;
      Anchored : Real_Array;
      Other    : Real_Array;
      Given    : out Kinds;
      Rounds   : out Natural;
      Doubt    : out Real)
   is
      Total  : constant Positive := At_Pixel'Length;
      Levels : constant := 256;
      Reach  : constant := 8;   --  neighbours of a pixel: their labels add up to this much either way
      Tail   : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
      Turn   : constant Real := Driver.Conventions.Z ** 2 / 2.0;   --  the log of the likelihood ratio a part is turned for
      --  The log of the odds against, as far as a share needs it: past half the
      --  exponent range the share is zero to working precision, and its
      --  exponential cannot overflow.
      Odds_Range : constant Real := Log (Real'Last) / 2.0;
      Slot   : Count_Access := new Count_Array (1 .. W * H);   --  the changed pixel at a place, 0 for none
      Share  : Real_Access := new Real_Array (1 .. Total);     --  of each pixel, in the kind with the robot at the anchored end
      Next   : Real_Access := new Real_Array (1 .. Total);
      Label  : Flags_Access := new Flag_Array (1 .. Total);    --  its label so far: that kind
      Near   : Count_Access := new Count_Array (1 .. Total);   --  its neighbours' labels, that kind less the other, from 0
      A_Bin  : Count_Access := new Count_Array (1 .. Total);
      B_Bin  : Count_Access := new Count_Array (1 .. Total);

      function Bin (V : Real) return Natural is
        (Natural (Real'Max (0.0, Real'Min (Real (Levels - 1), Real'Rounding (V)))));

      type Histogram is array (0 .. Levels - 1) of Real;
      type Votes is array (0 .. 2 * Reach) of Real;

      Flips   : Natural;
      Doubt_Now, Doubt_Before, Doubt_Earlier : Real := 0.0;   --  the expected error after the last round, the one before and the one before that

      function Changed_Neighbour (J : Positive; Dc, Dr : Integer) return Natural is
         P : constant Natural := At_Pixel (At_Pixel'First + J - 1);
         C : constant Integer := P mod W + Dc;
         R : constant Integer := P / W + Dr;
      begin
         return (if C in 0 .. W - 1 and then R in 0 .. H - 1 then Slot (Index (W, C, R) + 1) else 0);
      end Changed_Neighbour;

      procedure Round (Heard : Boolean) is
         --  One round: the histograms from the shares, the shares from the
         --  histograms (the neighbours' labels too when Heard).
         Robot, World : Histogram := [others => 1.0];
         Of_A, Of_B   : Votes := [others => 1.0];
         Robots, Worlds, Votes_A, Votes_B : Real := 0.0;
         Kind_A       : Real := 1.0;
         Cap          : Real;
      begin
         Rounds := Rounds + 1;
         for J in 1 .. Total loop
            declare
               Sum : Integer := 0;
            begin
               for Dr in -1 .. 1 loop
                  for Dc in -1 .. 1 loop
                     declare
                        Neighbour : constant Natural := (if Dr /= 0 or else Dc /= 0 then Changed_Neighbour (J, Dc, Dr) else 0);
                     begin
                        if Neighbour > 0 then
                           Sum := Sum + (if Label (Neighbour) then 1 else -1);
                        end if;
                     end;
                  end loop;
               end loop;
               Near (J) := Reach + Sum;
            end;
         end loop;
         for J in 1 .. Total loop
            Robot (A_Bin (J)) := Robot (A_Bin (J)) + Share (J);
            World (B_Bin (J)) := World (B_Bin (J)) + Share (J);
            Robot (B_Bin (J)) := Robot (B_Bin (J)) + (1.0 - Share (J));
            World (A_Bin (J)) := World (A_Bin (J)) + (1.0 - Share (J));
            Of_A (Near (J)) := Of_A (Near (J)) + Share (J);
            Of_B (Near (J)) := Of_B (Near (J)) + (1.0 - Share (J));
            Kind_A := Kind_A + Share (J);
         end loop;
         for X of Robot loop
            Robots := Robots + X;
         end loop;
         for X of World loop
            Worlds := Worlds + X;
         end loop;
         for X of Of_A loop
            Votes_A := Votes_A + X;
         end loop;
         for X of Of_B loop
            Votes_B := Votes_B + X;
         end loop;
         Cap := Kind_A / (Real (Total) + 2.0);
         Flips := 0;
         Doubt_Earlier := Doubt_Before;
         Doubt_Before := Doubt_Now;
         Doubt_Now := 0.0;
         for J in 1 .. Total loop
            declare
               La : constant Real :=
                 Log (Cap) + Log (Robot (A_Bin (J)) / Robots) + Log (World (B_Bin (J)) / Worlds)
                 + (if Heard then Log (Of_A (Near (J)) / Votes_A) else 0.0);
               Lb : constant Real :=
                 Log (1.0 - Cap) + Log (World (A_Bin (J)) / Worlds) + Log (Robot (B_Bin (J)) / Robots)
                 + (if Heard then Log (Of_B (Near (J)) / Votes_B) else 0.0);
            begin
               Next (J) := 1.0 / (1.0 + Exp (Real'Min (Lb - La, Odds_Range)));
               Flips := Flips + Boolean'Pos ((Next (J) > 0.5) /= Label (J));
               Doubt_Now := Doubt_Now + Real'Min (Next (J), 1.0 - Next (J));
            end;
         end loop;
         for J in 1 .. Total loop
            Share (J) := Next (J);
            Label (J) := Next (J) > 0.5;
         end loop;
      end Round;

      --  An iterative estimate has stopped changing when one more round moves
      --  it by less than Unchanged_Fraction of its own size (the convention).
      --  Before the neighbours are heard the estimate is the labels, which a
      --  round moves by Flips of Total: run on, the brightness alone drifts to
      --  a kind of everything. After, it is the doubt, the expected error,
      --  which goes on sharpening for many rounds after the labels have
      --  stopped: A14's final ends, 89 294 pixels, left 12 962 to neither when
      --  the rounds ended with the labels (nine rounds), 3 840 when they ended
      --  with the doubt (23), and 3 468 after 1 120 rounds. A round that takes
      --  the doubt back to what it was two rounds before has not moved it
      --  either, though the round before did: two pixels whose neighbours each
      --  tell them to take the other's label swap it for ever. And the rounds
      --  end when a label has crossed the picture.
      function Labels_Settled return Boolean is (Real (Flips) < Driver.Conventions.Unchanged_Fraction * Real (Total));
      function Doubt_Settled return Boolean is
        (abs (Doubt_Now - Doubt_Before) <= Driver.Conventions.Unchanged_Fraction * Doubt_Now
         or else abs (Doubt_Now - Doubt_Earlier) <= Driver.Conventions.Unchanged_Fraction * Doubt_Now);
      function Crossed return Boolean is (Rounds >= W + H);

      procedure Turn_Parts (Turned : out Natural) is
         --  Each connected part of one label (eight-connected, among the
         --  changed pixels), against the histograms of all the pixels but its
         --  own, as it is and the other way round.
         Robot, World : Histogram := [others => 1.0];
         Robots, Worlds : Real := 0.0;
         Seen    : Flags_Access := new Flag_Array (1 .. Total);
         Part    : Natural_Vectors.Vector;
         Pending : Natural_Vectors.Vector;
      begin
         Turned := 0;
         Seen.all := [others => False];
         for J in 1 .. Total loop
            if Label (J) then
               Robot (A_Bin (J)) := Robot (A_Bin (J)) + 1.0;
               World (B_Bin (J)) := World (B_Bin (J)) + 1.0;
            else
               Robot (B_Bin (J)) := Robot (B_Bin (J)) + 1.0;
               World (A_Bin (J)) := World (A_Bin (J)) + 1.0;
            end if;
         end loop;
         for X of Robot loop
            Robots := Robots + X;
         end loop;
         for X of World loop
            Worlds := Worlds + X;
         end loop;
         for J in 1 .. Total loop
            if not Seen (J) then
               Part.Clear;
               Pending.Clear;
               Seen (J) := True;
               Pending.Append (J);
               while not Pending.Is_Empty loop
                  declare
                     P : constant Positive := Pending.Last_Element;
                  begin
                     Pending.Delete_Last;
                     Part.Append (P);
                     for Dr in -1 .. 1 loop
                        for Dc in -1 .. 1 loop
                           declare
                              Neighbour : constant Natural :=
                                (if Dr /= 0 or else Dc /= 0 then Changed_Neighbour (P, Dc, Dr) else 0);
                           begin
                              if Neighbour > 0 and then not Seen (Neighbour) and then Label (Neighbour) = Label (J) then
                                 Seen (Neighbour) := True;
                                 Pending.Append (Neighbour);
                              end if;
                           end;
                        end loop;
                     end loop;
                  end;
               end loop;
               declare
                  Was_A : constant Boolean := Label (J);
                  Size  : constant Real := Real (Part.Length);
                  As_It_Is, Turned_Over : Real := 0.0;
                  Flip  : Boolean;
                  Now_A : Boolean;
               begin
                  --  The histograms of all the rest.
                  for P of Part loop
                     if Was_A then
                        Robot (A_Bin (P)) := Robot (A_Bin (P)) - 1.0;
                        World (B_Bin (P)) := World (B_Bin (P)) - 1.0;
                     else
                        Robot (B_Bin (P)) := Robot (B_Bin (P)) - 1.0;
                        World (A_Bin (P)) := World (A_Bin (P)) - 1.0;
                     end if;
                  end loop;
                  Robots := Robots - Size;
                  Worlds := Worlds - Size;
                  for P of Part loop
                     declare
                        As_A : constant Real :=
                          Log (Robot (A_Bin (P)) / Robots) + Log (World (B_Bin (P)) / Worlds);
                        As_B : constant Real :=
                          Log (World (A_Bin (P)) / Worlds) + Log (Robot (B_Bin (P)) / Robots);
                     begin
                        As_It_Is := As_It_Is + (if Was_A then As_A else As_B);
                        Turned_Over := Turned_Over + (if Was_A then As_B else As_A);
                     end;
                  end loop;
                  Flip := Turned_Over - As_It_Is > Turn;
                  Now_A := Was_A /= Flip;
                  Turned := Turned + Boolean'Pos (Flip);
                  --  Back among the rest, as it now is.
                  for P of Part loop
                     if Now_A then
                        Robot (A_Bin (P)) := Robot (A_Bin (P)) + 1.0;
                        World (B_Bin (P)) := World (B_Bin (P)) + 1.0;
                     else
                        Robot (B_Bin (P)) := Robot (B_Bin (P)) + 1.0;
                        World (A_Bin (P)) := World (A_Bin (P)) + 1.0;
                     end if;
                     if Flip then
                        Label (P) := Now_A;
                        Share (P) := (if Now_A then 1.0 else 0.0);
                     end if;
                  end loop;
                  Robots := Robots + Size;
                  Worlds := Worlds + Size;
               end;
            end if;
         end loop;
         Free (Seen);
      end Turn_Parts;

      Turned : Natural;
      Before : Natural := Natural'Last;   --  parts turned by the pass before
   begin
      Slot.all := [others => 0];
      for J in 1 .. Total loop
         Slot (At_Pixel (At_Pixel'First + J - 1) + 1) := J;
         Share (J) := (if Seeds (Seeds'First + J - 1) then 1.0 else 0.0);
         Label (J) := Seeds (Seeds'First + J - 1);
         A_Bin (J) := Bin (Anchored (Anchored'First + J - 1));
         B_Bin (J) := Bin (Other (Other'First + J - 1));
      end loop;
      Rounds := 0;
      --  Parts are turned until none is, or until a pass turns no fewer than
      --  the pass before: two parts that each explain the other better turned
      --  can be turned back and forth for ever (A14's stage of 62 570 changed
      --  pixels turned four to nine parts at every pass for a thousand rounds).
      loop
         loop
            Round (Heard => False);
            exit when Labels_Settled or else Crossed;
         end loop;
         Turn_Parts (Turned);
         exit when Turned = 0 or else Turned >= Before or else Crossed;
         Before := Turned;
      end loop;
      loop
         Round (Heard => True);
         exit when Doubt_Settled or else Crossed;
      end loop;
      --  The seeds say which kind is which: more of them are of the kind with
      --  the robot at the anchored end than not.
      declare
         Seeded : Natural := 0;
         Right  : Natural := 0;
      begin
         for J in 1 .. Total loop
            if Seeds (Seeds'First + J - 1) then
               Seeded := Seeded + 1;
               Right := Right + Boolean'Pos (Label (J));
            end if;
         end loop;
         if 2 * Right < Seeded then
            for J in 1 .. Total loop
               Share (J) := 1.0 - Share (J);
            end loop;
         end if;
      end;
      Doubt := 0.0;
      for J in 1 .. Total loop
         Given (Given'First + J - 1) :=
           (if Share (J) >= 1.0 - Tail then Anchored_End elsif Share (J) <= Tail then Other_End else Neither);
         Doubt := Doubt + Real'Min (Share (J), 1.0 - Share (J));
      end loop;
      Free (Slot);
      Free (Share);
      Free (Next);
      Free (Label);
      Free (Near);
      Free (A_Bin);
      Free (B_Bin);
   end Tell_Ends;

   function Large_From (Sizes : Real_Array) return Real;
   --  The size from which a part of an end is one of the lobes' and not a
   --  fragment of what the mixture was unsure of, from the sizes of the end's
   --  parts: the log sizes are parted in two groups (Cut_In_Two), and the
   --  parting stands when the two groups' mean log sizes differ by more than Z
   --  standard errors of their difference, from the spread the log sizes show
   --  within the groups, and the biggest part is more than N times the size
   --  parted at, N the number of parts: what is left out is smaller than the
   --  share 1 / N the biggest would be if all the parts were of its size. The
   --  fingers of a hand are of one order of size and what the mixture
   --  misplaces is not: A14's low end held parts of 29 219 and 19 664 pixels
   --  and of 400, 334, 149 ... , the high end 19 689 and 11 680 and 832, 670,
   --  300 ... A finger half the size of the others is not left out, nor are
   --  fingers that touch, one part twice as large as the other end's two.
   --  Zero, nothing parted, below three parts (a spread within groups needs a
   --  third value) and when the parting does not stand.

   function Large_From (Sizes : Real_Array) return Real is
      N : constant Natural := Sizes'Length;
   begin
      if N < 3 then
         return 0.0;
      end if;
      declare
         Logs     : Real_Access := new Real_Array (1 .. N);
         Cut      : Real;
         Share    : Real;
         Cuttable : Boolean;
         Result   : Real := 0.0;
      begin
         for K in 1 .. N loop
            Logs (K) := Log (Sizes (Sizes'First + K - 1));
         end loop;
         Cut_In_Two (Logs.all, Cut, Share, Cuttable);
         if Cuttable then
            declare
               Big, Small         : Natural := 0;
               Big_Sum, Small_Sum : Real := 0.0;
               Max_Log            : Real := Real'First;
            begin
               for V of Logs.all loop
                  Max_Log := Real'Max (Max_Log, V);
                  if V > Cut then
                     Big := Big + 1;
                     Big_Sum := Big_Sum + V;
                  else
                     Small := Small + 1;
                     Small_Sum := Small_Sum + V;
                  end if;
               end loop;
               if Big > 0 and then Small > 0 then
                  declare
                     Big_Mean   : constant Real := Big_Sum / Real (Big);
                     Small_Mean : constant Real := Small_Sum / Real (Small);
                     Within     : Real := 0.0;
                  begin
                     for V of Logs.all loop
                        Within := Within + (V - (if V > Cut then Big_Mean else Small_Mean)) ** 2;
                     end loop;
                     if Significant (Big_Mean - Small_Mean,
                                     Sqrt (Within / Real (N - 2)) * Sqrt (1.0 / Real (Big) + 1.0 / Real (Small)))
                       and then Exp (Max_Log - Cut) > Real (N)
                     then
                        Result := Exp (Cut);
                     end if;
                  end;
               end if;
            end;
         end if;
         Free (Logs);
         return Result;
      end;
   end Large_From;

   procedure Keep_Large (Parts : Part_Access; Large : Real);
   --  Of the parts that count, the kept ones are those from the size Large on,
   --  numbered again from one.

   procedure Keep_Large (Parts : Part_Access; Large : Real) is
      Kept : Natural := 0;
   begin
      for P of Parts.all loop
         if P.Kept > 0 then
            if Real (P.Count) >= Large then
               Kept := Kept + 1;
               P.Kept := Kept;
            else
               P.Kept := 0;
            end if;
         end if;
      end loop;
   end Keep_Large;

   procedure Reach_Tip (Lobe_Mask, Attached : Mask; Tip : out Pixel; Known : out Boolean);
   --  The tip of a lobe attached to the image's border or to the robot's
   --  pixels that did not change: the pixel farthest along the lobe's reach,
   --  the way from where it is attached to where its pixels are. The farthest
   --  pixel along the paths through the lobe (Tip_Of) is led off by whatever
   --  hangs on the lobe's side: in A14 the open end's wedge had a strip of
   --  keyboard pixels along the bottom of the picture, 143 pixels from the
   --  left border where the wedge's own apex was 118, and the strip was its tip.
   --  Along the reach that strip is 110 pixels from where the lobe is
   --  attached and the apex 157.

   procedure Reach_Tip (Lobe_Mask, Attached : Mask; Tip : out Pixel; Known : out Boolean) is
      W : constant Natural := Width (Lobe_Mask);
      H : constant Natural := Height (Lobe_Mask);
      Has_Attached : constant Boolean := Width (Attached) = W and then Height (Attached) = H;
      Anchors, Pixels : Natural := 0;
      Au, Av, Cu, Cv  : Real := 0.0;   --  sums, then centres, of the attached pixels and of the lobe

      function Is_Attached (C, R : Natural) return Boolean is
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
      end Is_Attached;
   begin
      Tip := (U => 0.0, V => 0.0);
      Known := False;
      for R in 0 .. H - 1 loop
         for C in 0 .. W - 1 loop
            if Contains (Lobe_Mask, C, R) then
               Pixels := Pixels + 1;
               Cu := Cu + Real (C) + 0.5;
               Cv := Cv + Real (R) + 0.5;
               if Is_Attached (C, R) then
                  Anchors := Anchors + 1;
                  Au := Au + Real (C) + 0.5;
                  Av := Av + Real (R) + 0.5;
               end if;
            end if;
         end loop;
      end loop;
      if Anchors = 0 then
         return;
      end if;
      Au := Au / Real (Anchors);
      Av := Av / Real (Anchors);
      Cu := Cu / Real (Pixels);
      Cv := Cv / Real (Pixels);
      declare
         Reach_U : constant Real := Cu - Au;
         Reach_V : constant Real := Cv - Av;
         Length  : constant Real := Sqrt (Reach_U ** 2 + Reach_V ** 2);
         Best    : Real := Real'First;
      begin
         if Length = 0.0 then
            return;
         end if;
         for R in 0 .. H - 1 loop
            for C in 0 .. W - 1 loop
               if Contains (Lobe_Mask, C, R) then
                  declare
                     Along : constant Real := ((Real (C) + 0.5 - Au) * Reach_U + (Real (R) + 0.5 - Av) * Reach_V) / Length;
                  begin
                     if Along > Best then
                        Best := Along;
                        Tip := (U => Real (C) + 0.5, V => Real (R) + 0.5);
                        Known := True;
                     end if;
                  end;
               end if;
            end loop;
         end loop;
      end;
   end Reach_Tip;

   function Lobes_Of_Sets
     (Here_Set, There_Set : Mask;
      Attached            : Mask;
      Doubt               : Real;
      Parts_Here          : out Natural;
      Parts_There         : out Natural) return Lobe_Vectors.Vector
   is
      W      : constant Natural := Width (Here_Set);
      H      : constant Natural := Height (Here_Set);
      Result : Lobe_Vectors.Vector;
   begin
      Parts_Here := 0;
      Parts_There := 0;
      declare
               Here_Labels  : constant Labels := Components (Opened (Here_Set));
               There_Labels : constant Labels := Components (Opened (There_Set));
               Here_Parts   : Part_Access := Parts_Of (Here_Labels, Attached, Doubt);
               There_Parts  : Part_Access := Parts_Of (There_Labels, Attached, Doubt);
               Kept_Here    : Natural := 0;
               Kept_There   : Natural := 0;
            begin
               --  Of the parts that are attached and above the doubt, those of the
               --  size of the lobes: the size distribution of both ends parts the
               --  fragments from them.
               declare
                  function Sizes_Of (Parts : Part_Access) return Real_Array is
                     Count : Natural := 0;
                  begin
                     for P of Parts.all loop
                        Count := Count + Boolean'Pos (P.Kept > 0);
                     end loop;
                     return Sizes : Real_Array (1 .. Count) do
                        Count := 0;
                        for P of Parts.all loop
                           if P.Kept > 0 then
                              Count := Count + 1;
                              Sizes (Count) := Real (P.Count);
                           end if;
                        end loop;
                     end return;
                  end Sizes_Of;
                  Large_Here  : constant Real := Large_From (Sizes_Of (Here_Parts));
                  Large_There : constant Real := Large_From (Sizes_Of (There_Parts));
               begin
                  --  An end with too few parts to part, or none to part, has the other end's size: the fingers
                  --  are the same at both ends.
                  Keep_Large (Here_Parts, (if Large_Here > 0.0 then Large_Here else Large_There));
                  Keep_Large (There_Parts, (if Large_There > 0.0 then Large_There else Large_Here));
               end;
               for P of Here_Parts.all loop
                  Kept_Here := Kept_Here + Boolean'Pos (P.Kept > 0);
               end loop;
               for P of There_Parts.all loop
                  Kept_There := Kept_There + Boolean'Pos (P.Kept > 0);
               end loop;
               Parts_Here := Kept_Here;
               Parts_There := Kept_There;
               if Kept_Here = 0 or else Kept_There = 0 then
                  Free (Here_Parts);
                  Free (There_Parts);
                  return Result;
               end if;
               declare
                  --  The lobes are the parts of the end that has the more of them
                  --  (the first when equal); each part of the other goes to the
                  --  nearest.
                  Split_Here : constant Boolean := Kept_Here >= Kept_There;
                  Lobes      : constant Natural := (if Split_Here then Kept_Here else Kept_There);
                  Splitting  : constant Part_Access := (if Split_Here then Here_Parts else There_Parts);
                  Joining    : constant Part_Access := (if Split_Here then There_Parts else Here_Parts);
                  Owner      : Count_Access := new Count_Array (Joining'Range);   --  the lobe each joining part goes to
                  Made       : Lobe_Vectors.Vector;
                  Joined     : Natural := 0;

                  function Nearest_Lobe (U, V : Real) return Natural is
                     Best : Real := Real'Last;
                     Lobe : Natural := 0;
                  begin
                     for I in Splitting'Range loop
                        if Splitting (I).Kept > 0 then
                           declare
                              Near : constant Pixel := Centre_Of (Splitting (I));
                              D    : constant Real := (Near.U - U) ** 2 + (Near.V - V) ** 2;
                           begin
                              if D < Best then
                                 Best := D;
                                 Lobe := Splitting (I).Kept;
                              end if;
                           end;
                        end if;
                     end loop;
                     return Lobe;
                  end Nearest_Lobe;
               begin
                  for J in Joining'Range loop
                     Owner (J) := 0;
                     if Joining (J).Kept > 0 then
                        declare
                           Best : Real := Real'Last;
                           Here_Of_J : constant Pixel := Centre_Of (Joining (J));
                        begin
                           for I in Splitting'Range loop
                              if Splitting (I).Kept > 0 then
                                 declare
                                    Near : constant Pixel := Centre_Of (Splitting (I));
                                    D    : constant Real := (Near.U - Here_Of_J.U) ** 2 + (Near.V - Here_Of_J.V) ** 2;
                                 begin
                                    if D < Best then
                                       Best := D;
                                       Owner (J) := Splitting (I).Kept;
                                    end if;
                                 end;
                              end if;
                           end loop;
                        end;
                     end if;
                  end loop;
                  for J in Joining'Range loop
                     Joined := Joined + Boolean'Pos (Joining (J).Kept > 0);
                  end loop;
                  for I in 1 .. Lobes loop
                     declare
                        Empty : Lobe;
                     begin
                        Empty.Here := Create (W, H);
                        Empty.There := Create (W, H);
                        Made.Append (Empty);
                     end;
                  end loop;
                  for R in 0 .. H - 1 loop
                     for C in 0 .. W - 1 loop
                        declare
                           Of_Here  : constant Natural := Here_Labels.Of_Pixel (Index (W, C, R));
                           Of_There : constant Natural := There_Labels.Of_Pixel (Index (W, C, R));
                           Into_Here, Into_There : Natural := 0;
                        begin
                           if Of_Here > 0 then
                              Into_Here := (if Split_Here then Here_Parts (Of_Here).Kept
                                            elsif Here_Parts (Of_Here).Kept = 0 then 0
                                            elsif Joined < Lobes then Nearest_Lobe (Real (C) + 0.5, Real (R) + 0.5)
                                            else Owner (Of_Here));
                           end if;
                           if Of_There > 0 then
                              Into_There := (if not Split_Here then There_Parts (Of_There).Kept
                                             elsif There_Parts (Of_There).Kept = 0 then 0
                                             elsif Joined < Lobes then Nearest_Lobe (Real (C) + 0.5, Real (R) + 0.5)
                                             else Owner (Of_There));
                           end if;
                           if Into_Here > 0 then
                              Include (Made.Reference (Into_Here).Here, C, R);
                           end if;
                           if Into_There > 0 then
                              Include (Made.Reference (Into_There).There, C, R);
                           end if;
                        end;
                     end loop;
                  end loop;
                  Free (Owner);
                  for L of Made loop
                     Summarize (L.Here, L.Count_Here, L.Centre_Here);
                     Summarize (L.There, L.Count_There, L.Centre_There);
                     if L.Count_Here > 0 and then L.Count_There > 0 then
                        --  Every pixel of a lobe changed, so any may be its tip: the one farthest along its reach.
                        Reach_Tip (L.Here, Attached, L.Tip_Here, L.Tip_Known_Here);
                        Reach_Tip (L.There, Attached, L.Tip_There, L.Tip_Known_There);
                        L.Bordered_Here := On_Border (L.Here);
                        L.Bordered_There := On_Border (L.There);
                        Result.Append (L);
                     end if;
                  end loop;
               end;
               Free (Here_Parts);
               Free (There_Parts);
      end;
      return Result;
   end Lobes_Of_Sets;

   function From_Change
     (Changed          : Mask;
      Here, There      : Driver.Pixels.View;
      Anchor           : Driver.Pixels.View;
      Anchored_At_Here : Boolean;
      Spread           : Real;
      Attached         : Mask) return Located
   is
      W      : constant Natural := Width (Changed);
      H      : constant Natural := Height (Changed);
      Total  : constant Natural := Count (Changed);
      Gate_Z : constant Real := Threshold (Scalar_Gate);
      Result : Located;
   begin
      Result.Changed := Total;
      if Total = 0 then
         return Result;
      end if;
      declare
         At_Pixel   : Count_Access := new Count_Array (1 .. Total);    --  each changed pixel's place, row after row
         Logs       : Real_Access := new Real_Array (1 .. Total);      --  its log deviation over the poses
         Here_Mean  : Real_Access := new Real_Array (1 .. W * H);
         There_Mean : Real_Access := new Real_Array (1 .. W * H);
         Anchored_Level : Real_Access := new Real_Array (1 .. Total);  --  how bright it is at the anchored end
         Other_Level    : Real_Access := new Real_Array (1 .. Total);  --  and at the other
         Seeds      : Flags_Access := new Flag_Array (1 .. Total);
         Given      : Kinds_Access := new Kinds (1 .. Total);
         Cut        : Real;
         Cuttable   : Boolean;
         Seen       : Natural := 0;
         K          : Natural := 0;

         procedure Release is
         begin
            Free (At_Pixel);
            Free (Logs);
            Free (Here_Mean);
            Free (There_Mean);
            Free (Anchored_Level);
            Free (Other_Level);
            Free (Seeds);
            Free (Given);
         end Release;
      begin
         for R in 0 .. H - 1 loop
            for C in 0 .. W - 1 loop
               if Contains (Changed, C, R) then
                  K := K + 1;
                  At_Pixel (K) := Index (W, C, R);
                  Logs (K) := 0.5 * Log (Driver.Pixels.Variance (Anchor, C, R));
               end if;
            end loop;
         end loop;
         Cut_In_Two (Logs.all, Cut, Result.Share, Cuttable);
         if not Cuttable then
            Result.How := Unseparated;
            Release;
            return Result;
         end if;
         Result.Cut := Exp (Cut);
         Driver.Pixels.Means (Here, Here_Mean.all);
         Driver.Pixels.Means (There, There_Mean.all);
         --  The seeds, and how the world varies where it changed: if the poses
         --  did not move the eye against its surroundings, the world there
         --  varies no more than two views of it differ at a pixel that did not
         --  change (Spread is that of the difference, so its pixel is a
         --  root-two of it), and the poses tell nothing.
         declare
            Above : Real_Access := new Real_Array (1 .. Total);
            Over  : Natural := 0;
         begin
            for J in 1 .. Total loop
               Seeds (J) := Logs (J) < Cut;
               Anchored_Level (J) := (if Anchored_At_Here then Here_Mean (At_Pixel (J) + 1) else There_Mean (At_Pixel (J) + 1));
               Other_Level (J) := (if Anchored_At_Here then There_Mean (At_Pixel (J) + 1) else Here_Mean (At_Pixel (J) + 1));
               if Seeds (J) then
                  Seen := Seen + 1;
               else
                  Over := Over + 1;
                  Above (Over) := Logs (J);
               end if;
            end loop;
            Result.Seeds := Seen;
            if Seen = 0 or else Over = 0
              or else Exp (Driver.Stats.Median (Above (1 .. Over))) <= Gate_Z * Spread / Sqrt (2.0)
            then
               Result.How := Unseparated;
               Free (Above);
               Release;
               return Result;
            end if;
            Free (Above);
         end;
         Tell_Ends (W, H, At_Pixel.all, Seeds.all, Anchored_Level.all, Other_Level.all, Given.all, Result.Rounds, Result.Doubt);
         declare
            Here_Set  : Mask := Create (W, H);
            There_Set : Mask := Create (W, H);
         begin
            for J in 1 .. Total loop
               declare
                  Column : constant Natural := At_Pixel (J) mod W;
                  Row    : constant Natural := At_Pixel (J) / W;
               begin
                  case Given (J) is
                     when Neither =>
                        Result.Unassigned := Result.Unassigned + 1;
                     when Anchored_End =>
                        if Anchored_At_Here then
                           Include (Here_Set, Column, Row);
                        else
                           Include (There_Set, Column, Row);
                        end if;
                     when Other_End =>
                        if Anchored_At_Here then
                           Include (There_Set, Column, Row);
                        else
                           Include (Here_Set, Column, Row);
                        end if;
                  end case;
               end;
            end loop;
            Result.Here := Count (Here_Set);
            Result.There := Count (There_Set);
            Release;
            Result.Lobes := Lobes_Of_Sets (Here_Set, There_Set, Attached, Result.Doubt, Result.Parts_Here, Result.Parts_There);
            Result.How := (if Result.Lobes.Is_Empty then One_Sided else Placed);
         end;
      end;
      return Result;
   end From_Change;

   function Change_Of
     (Lobes              : Lobe_Vectors.Vector;
      Attached           : Mask;
      Pixel_Variance     : Real;
      Degrees_Of_Freedom : Natural;
      Averaged           : Boolean) return Estimate;
   --  How the lobes' distances changed between the views, a pixel's place being
   --  known to Pixel_Variance. A centre of n pixels moves by less than its
   --  pixels' own noise when each is placed on its own (Averaged: a matcher's);
   --  when the error is in where the lobe's edge is, every pixel moves with it
   --  and the centre is no better known than a pixel.

   function Change_Of
     (Lobes              : Lobe_Vectors.Vector;
      Attached           : Mask;
      Pixel_Variance     : Real;
      Degrees_Of_Freedom : Natural;
      Averaged           : Boolean) return Estimate
   is
      function Share (Count : Natural) return Real is (if Averaged then 1.0 / Real (Count) else 1.0);
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
                  Variance := Variance + Pixel_Variance * (Share (Li.Count_Here) + Share (Lj.Count_Here)
                                                           + Share (Li.Count_There) + Share (Lj.Count_There));
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
            Variance := Pixel_Variance * (Share (L.Count_Here) + Share (L.Count_There));
         end;
      else
         return Unknown;
      end if;
      return (Value => Change, Sigma => Sqrt (Variance), Degrees_Of_Freedom => Degrees_Of_Freedom);
   end Change_Of;

   --  The matcher's noise and the pixel grid (a uniform square of side one).
   function Closing_Change (Lobes : Lobe_Vectors.Vector; Attached : Mask; Noise : Matcher_Noise) return Estimate is
     (Change_Of (Lobes, Attached, Noise.Displacement.Sigma ** 2 + 1.0 / 12.0, Noise.Displacement.Degrees_Of_Freedom, Averaged => True));

   function Closing_Change (Lobes : Lobe_Vectors.Vector; Attached : Mask) return Estimate is
     (Change_Of (Lobes, Attached, 1.0 / 12.0, 0, Averaged => False));

   function Decided (Change : Estimate) return Closing is
   begin
      if not Known (Change) or else not Significant (Change.Value, Change.Sigma, Change.Degrees_Of_Freedom) then
         return Undecided;
      end if;
      return (if Change.Value < 0.0 then Towards_There else Towards_Here);
   end Decided;

   function Direction (Lobes : Lobe_Vectors.Vector; Attached : Mask; Noise : Matcher_Noise) return Closing is
     (Decided (Closing_Change (Lobes, Attached, Noise)));

   function Direction (Lobes : Lobe_Vectors.Vector; Attached : Mask) return Closing is
     (Decided (Closing_Change (Lobes, Attached)));

end Driver.Robot.Hand.Lobes;
