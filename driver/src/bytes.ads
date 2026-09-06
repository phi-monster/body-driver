--  字节 / 数串 / 字串的容器。整个驱动只用标准库,没有第三方依赖。
with Ada.Containers.Vectors;
with Ada.Containers.Indefinite_Vectors;
with Interfaces; use Interfaces;
package Bytes is
   subtype U8 is Interfaces.Unsigned_8;
   package U8_Vectors is new Ada.Containers.Vectors (Natural, U8);
   subtype Buf is U8_Vectors.Vector;
   package F64_Vectors is new Ada.Containers.Vectors (Natural, Long_Float);
   subtype Floats is F64_Vectors.Vector;
   package Bool_Vectors is new Ada.Containers.Vectors (Natural, Boolean);
   subtype Bools is Bool_Vectors.Vector;
   package Int_Vectors is new Ada.Containers.Vectors (Natural, Integer);
   subtype Ints is Int_Vectors.Vector;
   package Str_Vectors is new Ada.Containers.Indefinite_Vectors (Natural, String);
   subtype Strs is Str_Vectors.Vector;

   function To_String (B : Buf; First, Len : Natural) return String;
   function From_String (S : String) return Buf;
   procedure Append (B : in out Buf; S : String);
   function Zeros (N : Natural) return Floats;
   function Filled (N : Natural; V : Long_Float) return Floats;
end Bytes;
