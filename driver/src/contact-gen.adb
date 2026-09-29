with Ada.Numerics; use Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Containers.Generic_Array_Sort;
with Ada.Strings.Fixed;
package body Contact.Gen is

   function Nat_Img (N : Natural) return String is (Ada.Strings.Fixed.Trim (Natural'Image (N), Ada.Strings.Left));

   type Float_Array is array (Natural range <>) of Long_Float;
   procedure Sort_Floats is new Ada.Containers.Generic_Array_Sort (Natural, Long_Float, Float_Array);

   --  中位数;空表 ⇒ 'Last(拿它当门槛时谁都过线)
   function Median (V : Float_Array) return Long_Float is
      C : Float_Array := V;
   begin
      if C'Length = 0 then
         return Long_Float'Last;
      end if;
      Sort_Floats (C);
      return C (C'First + C'Length / 2);
   end Median;

   --  容器按元素访问在 GNAT 里每次都要造一个引用对象(带终结),百万次就是秒级;几何内环全走裸数组,容器只在进出口出现
   type V3_Array is array (Natural range <>) of V3;
   type Nat_Array is array (Natural range <>) of Natural;
   function To_Array (V : V3_Vectors.Vector) return V3_Array is
      A : V3_Array (0 .. Natural (V.Length) - 1);
      K : Natural := 0;
   begin
      for P of V loop
         A (K) := P;
         K := K + 1;
      end loop;
      return A;
   end To_Array;

   --  ── 支撑面在哪,变成一个参数 ──

   function Inverse (R : Rot) return Rot is ((Axis => R.Axis, Ang => -R.Ang));

   --  ── 另外两种手 ──

   function Gap_Of (Ca : V3_Array) return Long_Float is
      N : constant Natural := Ca'Length;
      D : Float_Array (0 .. N - 1);
      K : Natural := 0;
   begin
      for I in 0 .. N - 1 loop
         declare
            Best : Long_Float := Long_Float'Last;
            A : V3 renames Ca (I);
         begin
            for J in 0 .. N - 1 loop
               if J /= I then
                  declare
                     B : V3 renames Ca (J);
                  begin
                     Best := Long_Float'Min (Best, Norm ([B (0) - A (0), B (1) - A (1), B (2) - A (2)]));
                  end;
               end if;
            end loop;
            if Best'Valid then
               D (K) := Best;
               K := K + 1;
            end if;
         end;
      end loop;
      if K = 0 then
         return 0.0;
      end if;
      return Median (D (0 .. K - 1));
   end Gap_Of;

   function Sampling_Gap (Cloud : V3_Vectors.Vector) return Long_Float is (Gap_Of (To_Array (Cloud)));

end Contact.Gen;
