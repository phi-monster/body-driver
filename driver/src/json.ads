--  够用的 JSON 读取器(节点池),外加给提示词转义用的 Escape。零依赖。
with Bytes; use Bytes;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
package Json is
   type Kind is (J_Null, J_Bool, J_Num, J_Str, J_Arr, J_Obj);
   type Node is record
      K : Kind := J_Null;
      B : Boolean := False;
      F : Long_Float := 0.0;
      S : Unbounded_String;
      Kids : Ints;      --  Arr:元素;Obj:键(Str 节点),值,键,值…
   end record;
   package Node_Vectors is new Ada.Containers.Vectors (Natural, Node);
   type Doc is record
      Nodes : Node_Vectors.Vector;
   end record;
   function Parse (Src : String; D : out Doc; Err : out Unbounded_String) return Boolean;  --  根 = 0
   function Kind_Of (D : Doc; N : Integer) return Kind;
   function Get (D : Doc; Obj : Integer; Name : String) return Integer;   --  值节点或 -1
   function Text (D : Doc; N : Integer) return String;
   function Num (D : Doc; N : Integer) return Long_Float;
   function Is_Num (D : Doc; N : Integer) return Boolean;
   function Bool (D : Doc; N : Integer) return Boolean;
   function Count (D : Doc; N : Integer) return Natural;
   function Child (D : Doc; N : Integer; I : Natural) return Integer;
   function Escape (S : String) return String;      --  \ " 换行 → JSON 字符串体
end Json;
