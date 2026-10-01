--  JSON reading into a read-only document, and the pieces for writing it.
--
--  Strings are kept as UTF-8; \u escapes, surrogate pairs included, are
--  decoded to UTF-8. Numbers are written so they read back to the same bits.

with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;

package Driver.Json is

   type Kind is (Null_Value, Boolean_Value, Number_Value, String_Value, Array_Value, Object_Value);

   type Node is new Natural;
   No_Node : constant Node := 0;

   type Document is private;

   procedure Parse (Text : String; Doc : out Document; Ok : out Boolean; Why : out Ada.Strings.Unbounded.Unbounded_String);
   --  The whole text must be one value; anything but white space after it fails.

   function Root (Doc : Document) return Node;
   function Kind_Of (Doc : Document; N : Node) return Kind;
   function Count (Doc : Document; N : Node) return Natural;
   --  Elements of an array, members of an object.
   function Element (Doc : Document; N : Node; Index : Positive) return Node;
   function Member_Name (Doc : Document; N : Node; Index : Positive) return String;
   function Member_Value (Doc : Document; N : Node; Index : Positive) return Node;
   function Lookup (Doc : Document; Object : Node; Name : String) return Node;
   function Text (Doc : Document; N : Node) return String;
   function Number (Doc : Document; N : Node) return Real;
   --  A number; null reads as NaN (Write_Number writes NaN and infinities as null).
   function Is_True (Doc : Document; N : Node) return Boolean;

   function Quote (S : String) return String;
   --  S as a JSON string literal, quotes included.

   function Number_Image (X : Real) return String;
   --  The shortest form that reads back to the same double; null when not finite.

private

   type Node_Record is record
      Of_Kind : Kind := Null_Value;
      Flag    : Boolean := False;
      Value   : Real := 0.0;
      Str     : Ada.Strings.Unbounded.Unbounded_String;
      Kids, Kids_Count : Natural := 0;   --  arrays: elements; objects: name node, value node, ...
   end record;

   package Node_Vectors is new Ada.Containers.Vectors (Positive, Node_Record);
   package Child_Vectors is new Ada.Containers.Vectors (Positive, Node);

   type Document is record
      Nodes    : Node_Vectors.Vector;
      Children : Child_Vectors.Vector;
   end record;

end Driver.Json;
