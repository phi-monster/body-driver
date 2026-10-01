--  msgpack 读写:读进一片"节点池"(索引引用,整份文档一起释放),写直接往字节串里推。
--  键可能是文本也可能是字节串(msgpack_numpy 用字节键);nd 数组是 {nd:true,type,shape,data} 这种映射。
with Bytes; use Bytes;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
package Msgpack is
   type Kind is (Nil, Bool, Int, Flt, Str, Bin, Arr, Map, Ext);
   type Node is record
      K : Kind := Nil;
      B : Boolean := False;
      I : Long_Long_Integer := 0;
      F : Long_Float := 0.0;
      S : Unbounded_String;
      Bin_First : Natural := 0;
      Bin_Len : Natural := 0;
      Kids : Ints;      --  Arr:元素;Map:键,值,键,值…
   end record;
   package Node_Vectors is new Ada.Containers.Vectors (Natural, Node);
   type Doc is record
      Nodes : Node_Vectors.Vector;
      Raw : Buf;        --  原始字节(Bin 只记切片位置,不复制)
   end record;

   function Decode (Data : Buf; D : out Doc) return Boolean;   --  根 = 节点 0
   function Kind_Of (D : Doc; N : Integer) return Kind;
   function Key (D : Doc; Map_Node : Integer; Name : String) return Integer;   --  值节点或 -1
   function Text (D : Doc; N : Integer) return String;          --  Str/Bin 当文本
   function Is_Text (D : Doc; N : Integer) return Boolean;
   function Is_Num (D : Doc; N : Integer) return Boolean;
   function Num (D : Doc; N : Integer) return Long_Float;
   function Count (D : Doc; N : Integer) return Natural;        --  Arr 长度 / Map 对数
   function Child (D : Doc; N : Integer; I : Natural) return Integer;
   function Map_Key (D : Doc; N : Integer; I : Natural) return Integer;
   function Map_Val (D : Doc; N : Integer; I : Natural) return Integer;
   --  一串数:普通数组,或 nd 的 f4/f8/i*/u* 数据。
   function Numbers (D : Doc; N : Integer) return Floats;
   function Is_Nd (D : Doc; N : Integer) return Boolean;
   function Nd_Type (D : Doc; N : Integer) return String;
   function Nd_Shape (D : Doc; N : Integer) return Ints;
   procedure Nd_Data (D : Doc; N : Integer; First, Len : out Natural);

   procedure Put_Nil (S : in out Buf);
   procedure Put_Bool (S : in out Buf; V : Boolean);
   procedure Put_Int (S : in out Buf; V : Long_Long_Integer);
   procedure Put_Float (S : in out Buf; V : Long_Float);
   procedure Put_Str (S : in out Buf; V : String);
   procedure Put_Bin (S : in out Buf; V : Buf; First, Len : Natural);
   procedure Put_Array (S : in out Buf; N : Natural);
   procedure Put_Map (S : in out Buf; N : Natural);
   procedure Put_Node (S : in out Buf; D : Doc; N : Integer);   --  原样回写一棵子树
end Msgpack;
