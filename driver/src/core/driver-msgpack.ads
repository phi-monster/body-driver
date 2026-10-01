--  MessagePack decoding into a read-only document, and encoding into a buffer.
--
--  A decoded document keeps the raw bytes once and refers to strings and
--  binary payloads by position, so large images are never copied while the
--  tree is walked. Numeric arrays are either plain arrays of numbers or the
--  msgpack-numpy form {"nd": true, "type": dtype, "shape": [...], "data": bin};
--  every numpy integer, float and boolean dtype of either byte order is read.

with Ada.Containers.Vectors;
with Driver.Bytes;

package Driver.Msgpack is

   type Kind is
     (Nil_Value, Boolean_Value, Integer_Value, Float_Value, String_Value, Binary_Value,
      Array_Value, Map_Value, Extension_Value);

   type Node is new Natural;
   No_Node : constant Node := 0;

   type Document is private;

   procedure Decode (Data : Driver.Bytes.Byte_Array; Doc : out Document; Ok : out Boolean);
   --  Ok is False for truncated or malformed input; the document is then empty.

   function Root (Doc : Document) return Node;

   function Kind_Of (Doc : Document; N : Node) return Kind;
   function Count (Doc : Document; N : Node) return Natural;
   --  Elements of an array, pairs of a map, zero for anything else.

   function Element (Doc : Document; N : Node; Index : Positive) return Node;
   function Key (Doc : Document; N : Node; Index : Positive) return Node;
   function Value (Doc : Document; N : Node; Index : Positive) return Node;
   --  No_Node when N is not an array (Element) or map (Key, Value), or Index is too large.

   function Lookup (Doc : Document; Map : Node; Name : String) return Node;
   --  The value of the first key equal to Name (text or binary key); No_Node if none.

   function Text (Doc : Document; N : Node) return String;
   --  A string or binary value as text; empty for anything else.

   function Is_Number (Doc : Document; N : Node) return Boolean;
   function Number (Doc : Document; N : Node) return Real
     with Pre => Is_Number (Doc, N);
   function Is_True (Doc : Document; N : Node) return Boolean;

   function Is_Ndarray (Doc : Document; N : Node) return Boolean;
   function Dtype (Doc : Document; N : Node) return String
     with Pre => Is_Ndarray (Doc, N);

   function Shape (Doc : Document; N : Node) return Natural_Array;
   --  For an ndarray its shape; for a number []; for a rectangular nest of
   --  plain arrays of numbers its dimensions; otherwise [] with Is_Numeric False.

   function Is_Numeric (Doc : Document; N : Node) return Boolean;
   --  A number, a rectangular nest of plain arrays of numbers, or a numeric ndarray.

   function Is_Byte_Image (Doc : Document; N : Node) return Boolean;
   --  An ndarray of one-byte integers (u1 or i1).

   function Numbers (Doc : Document; N : Node) return Real_Array
     with Pre => Is_Numeric (Doc, N);
   --  All values in row-major order.

   procedure Read_Binary
     (Doc     : Document;
      N       : Node;
      Process : not null access procedure (Data : Driver.Bytes.Byte_Array));
   --  The payload of a binary value, or the data of an ndarray, in place.

   procedure Put_Nil (B : in out Driver.Bytes.Buffer);
   procedure Put_Boolean (B : in out Driver.Bytes.Buffer; V : Boolean);
   procedure Put_Integer (B : in out Driver.Bytes.Buffer; V : Long_Long_Integer);
   procedure Put_Float (B : in out Driver.Bytes.Buffer; V : Real);
   procedure Put_String (B : in out Driver.Bytes.Buffer; V : String);
   procedure Put_Binary (B : in out Driver.Bytes.Buffer; V : Driver.Bytes.Byte_Array);
   procedure Put_Array_Header (B : in out Driver.Bytes.Buffer; Length : Natural);
   procedure Put_Map_Header (B : in out Driver.Bytes.Buffer; Pairs : Natural);
   procedure Put_Node (B : in out Driver.Bytes.Buffer; Doc : Document; N : Node);
   --  Re-encodes a decoded subtree unchanged in meaning.

private

   type Node_Record is record
      Of_Kind     : Kind := Nil_Value;
      Flag        : Boolean := False;
      Int         : Long_Long_Integer := 0;
      Flt         : Real := 0.0;
      First, Size : Natural := 0;     --  string, binary and extension payload in Raw
      Kids, Kids_Count : Natural := 0;--  children in Children, from index Kids
   end record;

   package Node_Vectors is new Ada.Containers.Vectors (Positive, Node_Record);
   package Child_Vectors is new Ada.Containers.Vectors (Positive, Node);

   type Document is record
      Raw      : Driver.Bytes.Holders.Holder;
      Nodes    : Node_Vectors.Vector;
      Children : Child_Vectors.Vector;
   end record;

end Driver.Msgpack;
