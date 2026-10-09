with Ada.Numerics.Long_Elementary_Functions;

package body Driver.Robot.Hand.Selfsight is

   use Ada.Numerics.Long_Elementary_Functions;

   function Start (Width, Height : Positive; Key_Noise : Real_Array) return Memory is
     ((Width    => Width,
       Height   => Height,
       Noise    => Real_Holders.To_Holder (Key_Noise),
       Settings => Setting_Vectors.Empty_Vector,
       Clock    => 0));

   procedure Set_Key_Noise (M : in out Memory; Key_Noise : Real_Array) is
   begin
      M.Noise := Real_Holders.To_Holder (Key_Noise);
   end Set_Key_Noise;

   function Key_Noise_Length (M : Memory) return Natural is (M.Noise.Element'Length);

   function Same (A, B, Noise : Real_Array) return Boolean is
     (A'Length = B'Length and then A'Length = Noise'Length
      and then (for all I in A'Range =>
                  not Significant (A (I) - B (B'First + I - A'First),
                                   Sqrt (2.0) * Noise (Noise'First + I - A'First))));
   --  Every reading within its noise of the other's, each carrying it, so
   --  their difference twice over.

   function Find (M : Memory; Key : Real_Array) return Natural is
   begin
      for I in M.Settings.First_Index .. M.Settings.Last_Index loop
         if Same (M.Settings (I).Key.Element, Key, M.Noise.Element) then
            return I;
         end if;
      end loop;
      return 0;
   end Find;

   procedure Observe
     (M          : in out Memory;
      Key        : Real_Array;
      Rest       : Real_Array;
      Still      : Boolean;
      Image      : Driver.Images.Image;
      Rest_Moved : not null access function (Before, After : Real_Array) return Boolean)
   is
      Here    : Natural;
      Nothing : Driver.Images.Image;   --  what a frame already in the view makes room for
   begin
      if not Still or else Driver.Images.Width (Image) /= M.Width or else Driver.Images.Height (Image) /= M.Height then
         return;
      end if;
      M.Clock := M.Clock + 1;
      Here := Find (M, Key);
      if Here = 0 then
         --  The closer is at a setting it was not at: the settings it left
         --  after one pose are forgotten.
         declare
            I : Natural := M.Settings.First_Index;
         begin
            while I <= M.Settings.Last_Index loop
               if M.Settings (I).Poses < 2 then
                  M.Settings.Delete (I);
               else
                  I := I + 1;
               end if;
            end loop;
         end;
         M.Settings.Append
           (Setting'(Key   => Real_Holders.To_Holder (Key),
                     Rest  => Real_Holders.To_Holder (Rest),
                     First => Image,
                     Seen  => <>,
                     Poses => 1,
                     Tick  => M.Clock));
         return;
      end if;
      declare
         S : Setting := M.Settings (Here);
      begin
         S.Tick := M.Clock;
         if Rest_Moved (S.Rest.Element, Rest) then
            if S.Poses = 1 then
               S.Seen := Driver.Pixels.Empty (M.Width, M.Height);
               Driver.Pixels.Add (S.Seen, S.First);
               S.First := Nothing;
            end if;
            Driver.Pixels.Add (S.Seen, Image);
            S.Poses := S.Poses + 1;
            S.Rest := Real_Holders.To_Holder (Rest);
         end if;
         M.Settings.Replace_Element (Here, S);
      end;
      --  No more settings of two poses or more than memory holds: the
      --  fewest-posed goes, the longest unseen of those.
      declare
         Kept : Natural := 0;
      begin
         for S of M.Settings loop
            Kept := Kept + Boolean'Pos (S.Poses >= 2);
         end loop;
         while Kept > Capacity loop
            declare
               Worst : Natural := 0;
            begin
               for I in M.Settings.First_Index .. M.Settings.Last_Index loop
                  if M.Settings (I).Poses >= 2
                    and then (Worst = 0 or else M.Settings (I).Poses < M.Settings (Worst).Poses
                              or else (M.Settings (I).Poses = M.Settings (Worst).Poses
                                       and then M.Settings (I).Tick < M.Settings (Worst).Tick))
                  then
                     Worst := I;
                  end if;
               end loop;
               M.Settings.Delete (Worst);
               Kept := Kept - 1;
            end;
         end loop;
      end;
   end Observe;

   function Poses (M : Memory; Key : Real_Array) return Natural is
      I : constant Natural := Find (M, Key);
   begin
      return (if I = 0 then 0 else M.Settings (I).Poses);
   end Poses;

   function Anchored (M : Memory; Key : Real_Array) return Boolean is (Poses (M, Key) >= 2);

   function Anchor_For (M : Memory; Key : Real_Array) return Driver.Pixels.View is
     (M.Settings (Find (M, Key)).Seen);

end Driver.Robot.Hand.Selfsight;
