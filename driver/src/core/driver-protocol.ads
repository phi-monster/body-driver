--  The message envelope of the body protocol (docs/body-protocol.md, 2 and
--  4): what kind of request arrived, where its observation is, and the reply
--  that echoes the request's identifiers.

with Ada.Strings.Unbounded;
with Driver.Bytes;
with Driver.Msgpack;

package Driver.Protocol is

   use Ada.Strings.Unbounded;

   type Message_Kind is (Hello, Prepare_Case, Reset, Call, Infer, Trial_End, Heartbeat, Unknown);

   type Request is record
      Kind          : Message_Kind := Unknown;
      Function_Name : Unbounded_String;           --  for Call, for example "get_action"
      Doc           : Driver.Msgpack.Document;
      Observation   : Driver.Msgpack.Node := Driver.Msgpack.No_Node;
   end record;

   function Has_Observation (R : Request) return Boolean is
     (Driver.Msgpack."/=" (R.Observation, Driver.Msgpack.No_Node));

   function Wants_Action (R : Request) return Boolean is
     (R.Kind = Call and then To_String (R.Function_Name) = "get_action");

   procedure Decode (Data : Driver.Bytes.Byte_Array; R : out Request; Ok : out Boolean);

   procedure Encode_Reply
     (R          : Request;
      Has_Action : Boolean;
      Action     : Driver.Bytes.Byte_Array;
      Reply      : out Driver.Bytes.Buffer);
   --  The reply for R. For get_action, Action is the encoded action map
   --  (Driver.Replies); without one the result list is empty (nothing can be
   --  sent yet). Action is ignored for every other request.

end Driver.Protocol;
