with Ada.Numerics.Long_Elementary_Functions;

package body Driver.Robot.Hand.Aims is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   function Across (Sight, V : Vec3) return Vec3 is
      Along : constant Real := V * Sight;
   begin
      return V - Along * Sight;
   end Across;
   --  The part of V across a unit Sight.

   function Tilted (Sight, Away : Vec3; Tilt : Real) return Vec3 is
      U : constant Vec3 := Unit (Sight);
      A : constant Vec3 := Unit (Across (U, Away));
   begin
      return Cos (Tilt) * U + Sin (Tilt) * A;
   end Tilted;

   function Any_Across (V : Vec3) return Vec3 is
      --  Across V, from the coordinate axis furthest from it.
      Axis : constant Vec3 :=
        (if abs V (1) <= abs V (2) and then abs V (1) <= abs V (3) then [1.0, 0.0, 0.0]
         elsif abs V (2) <= abs V (3) then [0.0, 1.0, 0.0] else [0.0, 0.0, 1.0]);
   begin
      return Unit (Cross (V, Axis));
   end Any_Across;

   function Turned_About (Tool : Rigid; Eye, Along, Into : Vec3) return Rigid is
      Now  : constant Vec3 := Tool.Rotation * Unit (Along);
      Goal : constant Vec3 := Unit (Into);
      Axis : constant Vec3 := Cross (Now, Goal);
      Size : constant Real := abs Axis;
      --  The least rotation takes Now to Goal about their common normal; when
      --  they are opposite, any axis across them does, by half a turn.
      Turn : constant Mat3 :=
        (if Size > Real'Model_Epsilon then Exp (Arctan (Size, Now * Goal) * ((1.0 / Size) * Axis))
         elsif Now * Goal > 0.0 then Identity3
         else Exp (Ada.Numerics.Pi * Any_Across (Now)));
      Rotation : constant Mat3 := Turn * Tool.Rotation;
      Eye_At   : constant Vec3 := Tool * Eye;
   begin
      return (Rotation => Rotation, Translation => Eye_At - Rotation * Eye);
   end Turned_About;

   function Away_From (Sight : Vec3; Rest : Direction_Array) return Vec3 is
      U   : constant Vec3 := Unit (Sight);
      Sum : Vec3 := Zero3;
   begin
      for O of Rest loop
         Sum := Sum + Unit (O);
      end loop;
      declare
         Toward : constant Vec3 := Across (U, Sum);
      begin
         if Rest'Length = 0 or else abs Toward <= Real'Model_Epsilon then
            return Zero3;
         end if;
         return -Unit (Toward);
      end;
   end Away_From;

   function Spread (Sight : Vec3; Rest : Direction_Array) return Real is
      U    : constant Vec3 := Unit (Sight);
      Best : Real := Real'Last;
   begin
      if Rest'Length = 0 then
         return 0.0;
      end if;
      for O of Rest loop
         Best := Real'Min (Best, Arctan (abs Cross (U, Unit (O)), U * Unit (O)));
      end loop;
      return Best;
   end Spread;

end Driver.Robot.Hand.Aims;
