--  Recordings of everything that crosses the driver's boundary (the main
--  program's --record), one record per message:
--
--    header   "BDWIRE1" LF
--    record   kind (one byte) | nanoseconds (u64 LE) | length (u32 LE) | payload
--
--  Kinds: R robot to driver, D driver to robot (binary messages), r and d the
--  same for text messages, C a new robot connection, S a service request and
--  T a service reply (payload: service name, LF, then the body).

with Ada.Streams.Stream_IO;
with Driver.Bytes;

package Driver.Recording is

   type Record_Kind is
     (Robot_Message, Driver_Message, Robot_Text, Driver_Text, Connection, Service_Request, Service_Reply);

   type Reader is limited private;

   procedure Open (R : in out Reader; Path : String; Ok : out Boolean);
   --  Ok is False when the file cannot be opened or lacks the header.

   procedure Next
     (R           : in out Reader;
      Kind        : out Record_Kind;
      Nanoseconds : out Long_Long_Integer;
      Payload     : in out Driver.Bytes.Buffer;
      Ok          : out Boolean);
   --  Ok is False at the end of the file or on a truncated record.

   procedure Close (R : in out Reader);

   type Writer is limited private;

   procedure Create (W : in out Writer; Path : String);
   procedure Write (W : in out Writer; Kind : Record_Kind; Payload : Driver.Bytes.Byte_Array);
   procedure Close (W : in out Writer);
   function Is_Open (W : Writer) return Boolean;

   procedure Start_Shared (Path : String);
   procedure Write_Shared (Kind : Record_Kind; Payload : Driver.Bytes.Byte_Array);
   procedure Stop_Shared;
   --  The process-wide recording the main loop and the service workers write
   --  to, one record at a time; Write_Shared does nothing until it is started.

private

   type Reader is limited record
      File : Ada.Streams.Stream_IO.File_Type;
   end record;

   type Writer is limited record
      File : Ada.Streams.Stream_IO.File_Type;
   end record;

end Driver.Recording;
