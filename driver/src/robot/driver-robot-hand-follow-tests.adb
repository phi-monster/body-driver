with Ada.Numerics.Long_Elementary_Functions;
with Ada.Streams;
use type Ada.Streams.Stream_Element_Offset;
with Driver.Bytes;
with Driver.Tests;

package body Driver.Robot.Hand.Follow.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;

   Width  : constant := 160;
   Height : constant := 120;

   --  A bar, a finger seen from the side, swung about its root at (40, 60) over a still table with a texture of its
   --  own; the bar's own texture, blobs rather than stripes, turns with it. Long and Wide in pixels, Turn in radians
   --  from pointing right.
   Root_U : constant := 40.0;
   Root_V : constant := 60.0;
   Long   : constant := 90.0;
   Wide   : constant := 22.0;

   function Table (U, V : Real) return Real is
     (110.0 + 14.0 * Sin (0.61 * U) * Cos (0.47 * V) + 9.0 * Sin (0.19 * U + 0.27 * V));

   function Bar (X, Y : Real) return Real is
     (60.0 + 25.0 * Sin (0.55 * X) * Cos (0.75 * Y) + 15.0 * Cos (0.21 * X - 0.33 * Y));

   procedure Place (U, V, Turn : Real; X, Y : out Real) is
   begin
      X := (U - Root_U) * Cos (Turn) + (V - Root_V) * Sin (Turn);
      Y := -(U - Root_U) * Sin (Turn) + (V - Root_V) * Cos (Turn);
   end Place;

   function On_Bar (U, V, Turn : Real) return Boolean is
      X, Y : Real;
   begin
      Place (U, V, Turn, X, Y);
      return X >= 0.0 and then X <= Long and then abs Y <= Wide / 2.0;
   end On_Bar;

   --  The picture with the bar turned by Turn, each pixel the mean of four by four samples across it.
   function Picture (Turn : Real) return Driver.Images.Image is
      Samples : constant := 4;
      RGB : Driver.Bytes.Byte_Array (1 .. Ada.Streams.Stream_Element_Offset (3 * Width * Height));
   begin
      for R in 0 .. Height - 1 loop
         for C in 0 .. Width - 1 loop
            declare
               Sum : Real := 0.0;
            begin
               for I in 1 .. Samples loop
                  for J in 1 .. Samples loop
                     declare
                        U : constant Real := Real (C) + (Real (I) - 0.5) / Real (Samples);
                        V : constant Real := Real (R) + (Real (J) - 0.5) / Real (Samples);
                        X, Y : Real;
                     begin
                        Place (U, V, Turn, X, Y);
                        Sum := Sum + (if On_Bar (U, V, Turn) then Bar (X, Y) else Table (U, V));
                     end;
                  end loop;
               end loop;
               declare
                  Level : constant Ada.Streams.Stream_Element :=
                    Ada.Streams.Stream_Element (Integer'Min (255, Integer'Max (0, Integer (Sum / Real (Samples ** 2)))));
                  At_Byte : constant Ada.Streams.Stream_Element_Offset :=
                    Ada.Streams.Stream_Element_Offset (3 * (R * Width + C));
               begin
                  RGB (At_Byte + 1) := Level;
                  RGB (At_Byte + 2) := Level;
                  RGB (At_Byte + 3) := Level;
               end;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (Width, Height, RGB);
   end Picture;

   function Tip (Turn : Real) return Driver.Images.Pixel is
     ((U => Root_U + Long * Cos (Turn), V => Root_V + Long * Sin (Turn)));

   --  Where the point of the bar at P when it was turned by From is when it is turned by To.
   function Turned (P : Driver.Images.Pixel; From, To : Real) return Driver.Images.Pixel is
      X, Y : Real;
   begin
      Place (P.U, P.V, From, X, Y);
      return (U => Root_U + X * Cos (To) - Y * Sin (To), V => Root_V + X * Sin (To) + Y * Cos (To));
   end Turned;

   function Mask_At (Turn : Real) return Driver.Images.Mask is
      M : Driver.Images.Mask := Driver.Images.Create (Width, Height);
   begin
      for R in 0 .. Height - 1 loop
         for C in 0 .. Width - 1 loop
            if On_Bar (Real (C) + 0.5, Real (R) + 0.5, Turn) then
               Driver.Images.Include (M, C, R);
            end if;
         end loop;
      end loop;
      return M;
   end Mask_At;

   --  The bar swung from Open to Closed in Steps equal turns.
   procedure Swing (Open, Closed : Real; Steps : Positive; Result : out Followed) is
      Views : Pyramid_Array (1 .. Steps + 1);
      Tips  : Pixel_Array (1 .. Steps + 1);
   begin
      for K in Views'Range loop
         declare
            Turn : constant Real := Open + (Closed - Open) * Real (K - 1) / Real (Steps);
         begin
            Views (K) := Driver.Alignment.Pyramid_Of (Picture (Turn));
            Tips (K) := Tip (Turn);
         end;
      end loop;
      Result := Run (Views, Tips, Mask_At (Open));
   end Swing;

   procedure Followed_Through_Small_Turns is
      Open   : constant Real := 0.0;
      Closed : constant Real := 0.6;   --  radians: the tip sweeps 54 pixels
      Half   : constant Real := Real (Driver.Alignment.Half_Patch (Width, Height));
      --  As many equal turns as keep the tip's move within what the matcher searches at its finest.
      Steps  : constant Positive :=
        Positive (Real'Ceiling (Long * (Closed - Open) / Driver.Alignment.Finest_Radius (Width, Height)));
      Result : Followed;
   begin
      Swing (Open, Closed, Steps, Result);
      Check (Ada.Strings.Unbounded.Length (Result.Why) = 0, "nothing followed: " & Ada.Strings.Unbounded.To_String (Result.Why));
      Check (2 * Result.Count > Result.Asked, "only" & Result.Count'Image & " of" & Result.Asked'Image
             & " points were followed through" & Steps'Image & " turns");
      if Result.Count > 0 then
         declare
            Moves  : Lobes.Move_Array renames Result.Moves.Constant_Reference.Element.all;
            Errors : Real_Array (Moves'Range);
            Worst  : Real := 0.0;
         begin
            for I in Moves'Range loop
               declare
                  Truth : constant Driver.Images.Pixel := Turned (Moves (I).From, Open, Closed);
               begin
                  Errors (I) := Sqrt ((Moves (I).To.U - Truth.U) ** 2 + (Moves (I).To.V - Truth.V) ** 2);
                  Worst := Real'Max (Worst, Errors (I));
               end;
            end loop;
            Sort (Errors);
            --  No point is taken for another a patch away, and the typical one lands within its own stated noise: the
            --  median length of a two-dimensional error of that sigma a coordinate is 1.18 sigmas.
            Check (Worst <= Half, "a followed point ended" & Worst'Image & " pixels from where the bar took it");
            Check (Errors ((Errors'First + Errors'Last) / 2) <= 2.0 * Result.Sigma,
                   "the median point ended" & Errors ((Errors'First + Errors'Last) / 2)'Image
                   & " pixels from the truth, against a stated sigma of" & Result.Sigma'Image);
         end;
      end if;
   end Followed_Through_Small_Turns;

   procedure Wide_Step_Not_Followed is
      Result : Followed;
   begin
      --  The whole swing in one step: the tip moves 54 pixels, past any half patch of a picture this size.
      Swing (0.0, 0.6, 1, Result);
      Check (Result.Count = 0 and then Ada.Strings.Unbounded.Length (Result.Why) > 0,
             "a step wider than the matcher's half patch was followed (" & Result.Count'Image & " points)");
   end Wide_Step_Not_Followed;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.follow.turns", "a lobe swung about its root is not followed through views a small turn "
                             & "apart, or its points end away from where it took them", Followed_Through_Small_Turns'Access);
      Driver.Tests.Register ("hand.follow.wide", "a step the matcher's patch cannot carry is followed",
                             Wide_Step_Not_Followed'Access);
   end Register;

end Driver.Robot.Hand.Follow.Tests;
