--  Recordings of everything that crosses the driver's boundary (the main
--  program's --record), one record per message:
--
--    header   "BDWIRE1" LF
--    record   kind (one byte) | nanoseconds (u64 LE) | length (u32 LE) | payload
--
--  Kinds: R robot to driver, D driver to robot (binary messages), r and d the
--  same for text messages, C a new robot connection, S a service request,
--  T a service reply, F a file the driver read, E a decider's call for the
--  estimates at once (Driver.Robot.Estimate_Now, no payload), W a
--  decider's write into the world (Driver.World.Replay_Write), and, while the
--  heavier estimates are computed apart from the main loop (Driver.Apart, no
--  payloads), K where the models went apart, A each part of a message the
--  estimator gave them and B where the main loop took them back. A service record starts
--  with a line naming the call: the service, its call number (which pairs a
--  reply with its request) and, for a call an estimator submitted, "beat"
--  and the beat it was submitted at. A request goes on with the path, LF,
--  and the body; a reply with "ok" or what went wrong, LF, and the body. A
--  file record starts with a line naming the file: what it is ("body", the
--  body file Driver.Robot.Load_Body reads), a space and its path; then LF
--  and the text read, so a replay is given the file as the run read it, at
--  the point the run read it, even after the run rewrote the file.

with Ada.Streams.Stream_IO;
with Driver.Bytes;

package Driver.Recording is

   type Record_Kind is
     (Robot_Message, Driver_Message, Robot_Text, Driver_Text, Connection, Service_Request, Service_Reply,
      File_Read, Estimates_Asked, World_Written, Estimates_Apart, Taken_In, Estimates_Back);

   type Reader is limited private;

   procedure Open (R : in out Reader; Path : String; Ok : out Boolean);
   --  Ok is False when the file cannot be opened or lacks the header. The
   --  reader only reads forward, so Path may be a pipe (/dev/stdin fed by
   --  zstd -dc): a compressed recording is read without unpacking it to disk.

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
   function Shared_Started return Boolean;
   --  The process-wide recording the main loop and the service workers write
   --  to, one record at a time; Write_Shared does nothing until it is started.
   --  Shared_Started says whether it is being written, so a writer can skip
   --  encoding a record nobody keeps.

   procedure Write_Shared (Kind : Record_Kind; Payload : Driver.Bytes.Byte_Array; Place : out Positive);
   procedure Write_Shared
     (Kind    : Record_Kind;
      Payload : Driver.Bytes.Byte_Array;
      Then_Do : not null access procedure (Place : Positive));
   --  Every shared record has a place in one order, 1 for the first: every
   --  record counts, written or not, so the places are the same with and
   --  without a recording. Then_Do runs with the record's place before any
   --  other record is written, so what it makes known is known from exactly
   --  that point of the recording on (a service reply, Driver.Services).

private

   type Reader is limited record
      File : Ada.Streams.Stream_IO.File_Type;
   end record;

   type Writer is limited record
      File : Ada.Streams.Stream_IO.File_Type;
   end record;

end Driver.Recording;
