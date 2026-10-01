with Ada.Numerics.Long_Elementary_Functions;

package body Driver.Robot.Hand.Views is

   use Ada.Numerics.Long_Elementary_Functions;

   function Start (Width, Height : Positive; Closer_Noise, Rest_Noise : Real_Array) return Tracker is
     ((Width        => Width,
       Height       => Height,
       Closer_Noise => Noise_Holders.To_Holder (Closer_Noise),
       Rest_Noise    => Noise_Holders.To_Holder (Rest_Noise),
       Current      => View_Holders.Empty_Holder,
       Ends         => End_Holders.To_Holder ([Closer_Noise'Range => (others => <>)])));

   function Moved (A, B, Noise : Real_Array; Except : Natural := 0) return Boolean;
   --  Some reading other than the one numbered Except differs significantly:
   --  each reading carries its noise, so their difference carries it twice over.

   function Moved (A, B, Noise : Real_Array; Except : Natural := 0) return Boolean is
   begin
      if A'Length /= B'Length then
         return True;
      end if;
      for I in A'Range loop
         if I - A'First + 1 /= Except
           and then Significant (A (I) - B (B'First + I - A'First), Sqrt (2.0) * Noise (Noise'First + I - A'First))
         then
            return True;
         end if;
      end loop;
      return False;
   end Moved;

   function Reading (V : View; Channel : Positive) return Real is
     (V.Closer.Element (V.Closer.Element'First + Channel - 1));

   function Comparable (T : Tracker; A, B : View; Channel : Positive) return Boolean is
     (not Moved (A.Closer.Element, B.Closer.Element, T.Closer_Noise.Element, Except => Channel)
      and then not Moved (A.Rest.Element, B.Rest.Element, T.Rest_Noise.Element));
   --  The same background and the same other channels: only this channel differs.

   procedure Consider (T : in out Tracker; V : View);
   --  A finished view becomes an end of each channel's travel it extends.

   procedure Consider (T : in out Tracker; V : View) is
      Ends : End_Array := T.Ends.Element;
   begin
      for C in Ends'Range loop
         declare
            Noise : constant Real := Sqrt (2.0) * T.Closer_Noise.Element (T.Closer_Noise.Element'First + C - 1);
            P     : End_Pair renames Ends (C);
         begin
            if P.Low.Is_Empty or else not Comparable (T, P.Low.Element, V, C) then
               --  The first view of this channel, or the background changed:
               --  the old ends cannot be compared with what comes now.
               P.Low := View_Holders.To_Holder (V);
               P.High := View_Holders.To_Holder (V);
            else
               declare
                  Low  : constant Real := Reading (P.Low.Element, C);
                  High : constant Real := Reading (P.High.Element, C);
                  Here : constant Real := Reading (V, C);
                  Frames_Here : constant Natural := Driver.Pixels.Frames (V.Frames);
               begin
                  --  Beyond an end, or at it with more frames to say how still it was.
                  if Significant (Low - Here, Noise) and then Here < Low then
                     P.Low := View_Holders.To_Holder (V);
                  elsif not Significant (Low - Here, Noise)
                    and then Frames_Here > Driver.Pixels.Frames (P.Low.Element.Frames)
                  then
                     P.Low := View_Holders.To_Holder (V);
                  end if;
                  if Significant (Here - High, Noise) and then Here > High then
                     P.High := View_Holders.To_Holder (V);
                  elsif not Significant (Here - High, Noise)
                    and then Frames_Here > Driver.Pixels.Frames (P.High.Element.Frames)
                  then
                     P.High := View_Holders.To_Holder (V);
                  end if;
               end;
            end if;
         end;
      end loop;
      T.Ends := End_Holders.To_Holder (Ends);
   end Consider;

   procedure Close_Current (T : in out Tracker);

   procedure Close_Current (T : in out Tracker) is
   begin
      if not T.Current.Is_Empty then
         Consider (T, T.Current.Element);
         T.Current := View_Holders.Empty_Holder;
      end if;
   end Close_Current;

   function Readings_Moved (T : Tracker; Closer, Rest : Real_Array) return Boolean;
   --  The view being gathered was taken at other readings.

   function Readings_Moved (T : Tracker; Closer, Rest : Real_Array) return Boolean is
      Now : constant View_Holders.Constant_Reference_Type := T.Current.Constant_Reference;
   begin
      return Moved (Now.Element.Closer.Element, Closer, T.Closer_Noise.Element)
        or else Moved (Now.Element.Rest.Element, Rest, T.Rest_Noise.Element);
   end Readings_Moved;

   procedure Observe
     (T      : in out Tracker;
      Seen   : Observation;
      Still  : Boolean;
      Closer : Real_Array;
      Rest   : Real_Array;
      Image  : Driver.Images.Image)
   is
   begin
      if not Still or else Driver.Images.Is_Empty (Image) or else Driver.Images.Width (Image) /= T.Width
        or else Driver.Images.Height (Image) /= T.Height
      then
         Close_Current (T);
         return;
      end if;
      if not T.Current.Is_Empty and then Readings_Moved (T, Closer, Rest) then
         Close_Current (T);
      end if;
      if T.Current.Is_Empty then
         T.Current := View_Holders.To_Holder
           ((Closer => Reading_Holders.To_Holder (Closer),
             Rest   => Reading_Holders.To_Holder (Rest),
             Frames => Driver.Pixels.Empty (T.Width, T.Height),
             Last   => Image,
             Seen   => Seen,
             From   => Seen.Beat,
             To     => Seen.Beat));
      end if;
      declare
         R : constant View_Holders.Reference_Type := T.Current.Reference;
      begin
         Driver.Pixels.Add (R.Element.Frames, Image);
         R.Element.Last := Image;
         R.Element.To := Seen.Beat;
      end;
   end Observe;

   function Has_Ends (T : Tracker; Channel : Positive) return Boolean is
      Ends : constant End_Holders.Constant_Reference_Type := T.Ends.Constant_Reference;
   begin
      if Channel > Ends.Element'Last or else Ends.Element (Channel).Low.Is_Empty
        or else Ends.Element (Channel).High.Is_Empty
      then
         return False;
      end if;
      declare
         Low  : constant View_Holders.Constant_Reference_Type := Ends.Element (Channel).Low.Constant_Reference;
         High : constant View_Holders.Constant_Reference_Type := Ends.Element (Channel).High.Constant_Reference;
      begin
         --  Two frames each let the pixels say how much they vary on their own.
         return Driver.Pixels.Frames (Low.Element.Frames) >= 2 and then Driver.Pixels.Frames (High.Element.Frames) >= 2
           and then Significant (Reading (High.Element.all, Channel) - Reading (Low.Element.all, Channel),
                                 Sqrt (2.0) * T.Closer_Noise.Element (T.Closer_Noise.Element'First + Channel - 1));
      end;
   end Has_Ends;

   function Low_End (T : Tracker; Channel : Positive) return View is (T.Ends.Element (Channel).Low.Element);

   function High_End (T : Tracker; Channel : Positive) return View is (T.Ends.Element (Channel).High.Element);

   function Low_Beat (T : Tracker; Channel : Positive) return Driver.Clock.Beat is
     (T.Ends.Constant_Reference.Element (Channel).Low.Constant_Reference.Element.From);

   function High_Beat (T : Tracker; Channel : Positive) return Driver.Clock.Beat is
     (T.Ends.Constant_Reference.Element (Channel).High.Constant_Reference.Element.From);

end Driver.Robot.Hand.Views;
