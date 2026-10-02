with Ada.Numerics.Long_Elementary_Functions;
with Driver.Bytes;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Stats;
with Driver.World.Regions;

package body Driver.World.Tracking is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Images;
   use type Driver.Bytes.Offset;

   Mad_Efficiency : constant := 0.367_5;
   --  The asymptotic efficiency of the median absolute deviation for Gaussian
   --  data: its scale is worth that share of as many degrees of freedom.

   function Cut (Frame : Image; Column_0, Row_0, Columns, Rows : Natural) return Image is
      --  The box of a frame as an image of its own.
      Result : Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset (3 * Columns * Rows));
      procedure Copy (RGB : Driver.Bytes.Byte_Array) is
         W : constant Natural := Width (Frame);
      begin
         for R in 0 .. Rows - 1 loop
            for C in 0 .. Columns - 1 loop
               for K in 1 .. 3 loop
                  Result (Driver.Bytes.Offset (3 * (R * Columns + C) + K)) :=
                    RGB (RGB'First + Driver.Bytes.Offset (3 * ((Row_0 + R) * W + Column_0 + C) + K - 1));
               end loop;
            end loop;
         end loop;
      end Copy;
   begin
      Query (Frame, Copy'Access);
      return Create (Columns, Rows, Result);
   end Cut;

   function Start (Region : Mask; On : Image; Beat : Driver.Clock.Beat) return Track is
      B      : constant Driver.World.Regions.Box := Driver.World.Regions.Bounds (Region);
      --  As far around the region again as its own half-width.
      Margin : constant Natural := Natural (Real'Ceiling (Driver.World.Regions.Radius (Region)));
      T      : Track;
   begin
      T.State := Holding;
      T.Region := Mask_Holders.To_Holder (Region);
      T.Measured_Beat := Beat;
      T.Measured_Image := Image_Holders.To_Holder (On);
      T.Column_0 := (if B.Column_0 > Margin then B.Column_0 - Margin else 0);
      T.Row_0 := (if B.Row_0 > Margin then B.Row_0 - Margin else 0);
      T.Columns := Natural'Min (Width (Region) - 1, B.Column_1 + Margin) - T.Column_0 + 1;
      T.Rows := Natural'Min (Height (Region) - 1, B.Row_1 + Margin) - T.Row_0 + 1;
      T.Reference := Driver.Pixels.Empty (T.Columns, T.Rows);
      Driver.Pixels.Add (T.Reference, Cut (On, T.Column_0, T.Row_0, T.Columns, T.Rows));
      T.Latest_Image := Image_Holders.To_Holder (On);
      T.Latest_At := Beat;
      T.Was_Still := True;
      return T;
   end Start;

   function Changes (T : Track) return Mask is
      --  Which pixels of the box changed: the latest two still frames against
      --  the frames the region was measured on, with the noise every pixel
      --  showed over those; the same pixel of the same camera varies as much
      --  now, and two frames alone cannot say how much.
      Latest_Two : Driver.Pixels.View := Driver.Pixels.Empty (T.Columns, T.Rows);
      Count_Ref  : constant Positive := Driver.Pixels.Frames (T.Reference);
      Gate       : constant Driver.Uncertain.Gate := Scalar_Gate (Count_Ref - 1);
      Ref_Mean, Ref_Variance, Now_Mean : Real_Array (1 .. T.Columns * T.Rows);
      Flags      : Mask := Create (T.Columns, T.Rows);
   begin
      Driver.Pixels.Add (Latest_Two, T.Older.Element);
      Driver.Pixels.Add (Latest_Two, T.Newer.Element);
      Driver.Pixels.Means (T.Reference, Ref_Mean);
      Driver.Pixels.Variances (T.Reference, Ref_Variance);
      Driver.Pixels.Means (Latest_Two, Now_Mean);
      for R in 0 .. T.Rows - 1 loop
         for C in 0 .. T.Columns - 1 loop
            declare
               I : constant Positive := R * T.Columns + C + 1;
            begin
               if Significant (Gate, Now_Mean (I) - Ref_Mean (I),
                               Sqrt (Ref_Variance (I) * (1.0 / Real (Driver.Pixels.Frames (Latest_Two))
                                                         + 1.0 / Real (Count_Ref))))
               then
                  Include (Flags, C, R);
               end if;
            end;
         end loop;
      end loop;
      return Flags;
   end Changes;

   function Changed (T : Track) return Boolean is
      --  More of the region's pixels changed than the test passes by chance:
      --  the count of chance passes is binomial.
      Region : constant Mask := T.Region.Element;
      Flags  : constant Mask := Changes (T);
      Tested, Moved : Natural := 0;
      Rate   : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
   begin
      for R in 0 .. T.Rows - 1 loop
         for C in 0 .. T.Columns - 1 loop
            if Contains (Region, T.Column_0 + C, T.Row_0 + R) then
               Tested := Tested + 1;
               Moved := Moved + Boolean'Pos (Contains (Flags, C, R));
            end if;
         end loop;
      end loop;
      declare
         Expected : constant Real := Real (Tested) * Rate;
      begin
         return Real (Moved) > Expected
           and then Significant (Real (Moved) - Expected, Sqrt (Expected * (1.0 - Rate)));
      end;
   end Changed;

   procedure Observe (T : in out Track; Frame : Image; Beat : Driver.Clock.Beat; Still : Boolean) is
   begin
      T.Was_Still := Still and then not Is_Empty (Frame);
      if not T.Was_Still or else T.State = Gone then
         return;
      end if;
      T.Latest_Image := Image_Holders.To_Holder (Frame);
      T.Latest_At := Beat;
      T.Older := T.Newer;
      T.Newer := Image_Holders.To_Holder (Cut (Frame, T.Column_0, T.Row_0, T.Columns, T.Rows));
      if T.State /= Holding then
         return;
      end if;
      --  The frames it was measured on need two at least to say how much a
      --  pixel varies on its own; until then a still frame only adds to them.
      if Driver.Pixels.Frames (T.Reference) >= 2 and then not T.Older.Is_Empty and then Changed (T) then
         T.State := Lost;
         --  The frames to look again with are taken after the change.
         T.Older := Image_Holders.Empty_Holder;
         return;
      end if;
      Driver.Pixels.Add (T.Reference, T.Newer.Element);
   end Observe;

   function State (T : Track) return Phase is (T.State);
   function Seen (T : Track) return Boolean is (T.State = Holding and then T.Was_Still);
   function Region (T : Track) return Mask is (T.Region.Element);
   function Measured_At (T : Track) return Driver.Clock.Beat is (T.Measured_Beat);
   function Measured_On (T : Track) return Image is (T.Measured_Image.Element);
   function Latest (T : Track) return Image is (T.Latest_Image.Element);
   function Latest_Beat (T : Track) return Driver.Clock.Beat is (T.Latest_At);

   function Wants_Match (T : Track) return Boolean is
     (T.State = Lost and then T.Was_Still and then not T.Older.Is_Empty);

   function Region_Points (T : Track) return Natural is (Count (T.Region.Element));

   function Region_And_Around (T : Track) return Driver.Instrument.Point_Array is
      Region : constant Mask := T.Region.Element;
      Points : Driver.Instrument.Point_Array (1 .. T.Columns * T.Rows);
      Own    : Natural := 0;
      Around : Natural := Count (Region);
   begin
      for R in 0 .. T.Rows - 1 loop
         for C in 0 .. T.Columns - 1 loop
            --  The pixel's centre, in the whole image: the region's first.
            if Contains (Region, T.Column_0 + C, T.Row_0 + R) then
               Own := Own + 1;
               Points (Own) := (U => Real (T.Column_0 + C) + 0.5, V => Real (T.Row_0 + R) + 0.5);
            else
               Around := Around + 1;
               Points (Around) := (U => Real (T.Column_0 + C) + 0.5, V => Real (T.Row_0 + R) + 0.5);
            end if;
         end loop;
      end loop;
      return Points;
   end Region_And_Around;

   function Match_Points (T : Track) return Driver.Instrument.Point_Array is (Region_And_Around (T));

   procedure Asked_Match (T : in out Track) is
   begin
      T.State := Matching;
      T.Asked_On := T.Latest_Image;
      T.Asked_At := T.Latest_At;
      T.Own_Points := Count (T.Region.Element);
   end Asked_Match;

   function Round_Trip (A : Driver.Instrument.Answer; From : Driver.Instrument.Pixel) return Real is
     (Sqrt ((A.Back.U - From.U) ** 2 + (A.Back.V - From.V) ** 2));

   procedure Matched (T : in out Track; Points : Driver.Instrument.Point_Array; Answers : Driver.Instrument.Answer_Array)
   is
      Still_Found : Natural := 0;
   begin
      for I in T.Own_Points + 1 .. Answers'Length loop
         Still_Found := Still_Found + Boolean'Pos (Answers (Answers'First + I - 1).Found);
      end loop;
      if Still_Found = 0 then
         --  Nothing around it to tell the matcher's error by.
         T.State := Gone;
         return;
      end if;
      declare
         --  The matcher's error, from the round trips of the pixels around it.
         Trips : Real_Array (1 .. 2 * Still_Found);
         K     : Natural := 0;
      begin
         for I in T.Own_Points + 1 .. Answers'Length loop
            declare
               A : Driver.Instrument.Answer renames Answers (Answers'First + I - 1);
               P : constant Driver.Instrument.Pixel := Points (Points'First + I - 1);
            begin
               if A.Found then
                  Trips (K + 1) := A.Back.U - P.U;
                  Trips (K + 2) := A.Back.V - P.V;
                  K := K + 2;
               end if;
            end;
         end loop;
         declare
            Sigma   : constant Real := Driver.Stats.Robust_Sigma (Trips);
            Gate    : constant Driver.Uncertain.Gate :=
              Vector_Gate (2, Natural (Real'Floor (Mad_Efficiency * Real (2 * Still_Found))));
            Inner   : constant Driver.Instrument.Pixel := Driver.World.Regions.Inner_Point (T.Region.Element);
            Box     : Driver.Instrument.Box := (X0 => Real'Last, Y0 => Real'Last, X1 => Real'First, Y1 => Real'First);
            Back    : Natural := 0;
            Nearest : Real := Real'Last;
            W       : constant Real := Real (Width (T.Asked_On.Element));
            H       : constant Real := Real (Height (T.Asked_On.Element));
         begin
            for I in 1 .. T.Own_Points loop
               declare
                  A : Driver.Instrument.Answer renames Answers (Answers'First + I - 1);
                  P : constant Driver.Instrument.Pixel := Points (Points'First + I - 1);
               begin
                  if A.Found and then A.To.U >= 0.0 and then A.To.V >= 0.0 and then A.To.U < W and then A.To.V < H
                    and then not Significant (Gate, Round_Trip (A, P), Sigma)
                  then
                     Back := Back + 1;
                     Box := (X0 => Real'Min (Box.X0, A.To.U), Y0 => Real'Min (Box.Y0, A.To.V),
                             X1 => Real'Max (Box.X1, A.To.U), Y1 => Real'Max (Box.Y1, A.To.V));
                     if (P.U - Inner.U) ** 2 + (P.V - Inner.V) ** 2 < Nearest then
                        Nearest := (P.U - Inner.U) ** 2 + (P.V - Inner.V) ** 2;
                        T.Prompt_Point := A.To;
                     end if;
                  end if;
               end;
            end loop;
            if Back = 0 then
               T.State := Gone;
               return;
            end if;
            T.Prompt_Box := Box;
            T.State := Segmenting;
            T.Segment_Asked := False;
         end;
      end;
   end Matched;

   function Wants_Segment (T : Track) return Boolean is (T.State = Segmenting and then not T.Segment_Asked);

   procedure Segment_Prompt (T : Track; Around : out Driver.Instrument.Box; At_Point : out Driver.Instrument.Pixel) is
   begin
      Around := T.Prompt_Box;
      At_Point := T.Prompt_Point;
   end Segment_Prompt;

   function Segment_On (T : Track) return Image is (T.Asked_On.Element);

   procedure Asked_Segment (T : in out Track) is
   begin
      T.Segment_Asked := True;
   end Asked_Segment;

   procedure Segmented (T : in out Track; Found : Mask) is
   begin
      if Count (Found) = 0 then
         T.State := Gone;
         return;
      end if;
      declare
         Again : constant Track := Start (Found, T.Asked_On.Element, T.Asked_At);
      begin
         T := Again;
      end;
   end Segmented;

   procedure Failed (T : in out Track) is
   begin
      T.State := Gone;
   end Failed;

end Driver.World.Tracking;
