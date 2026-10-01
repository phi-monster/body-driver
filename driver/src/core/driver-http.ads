--  An HTTP/1.1 client for the services: one POST of a JSON body per
--  connection, the reply read to its end (by Content-Length, chunked
--  encoding, or the server closing the connection).
--
--  There is no timeout: the main loop never waits on a service (the robot
--  holds meanwhile), and a streamed reply can be stopped by its reader.

with Ada.Strings.Unbounded;

package Driver.Http is

   use Ada.Strings.Unbounded;

   type Response is record
      Ok        : Boolean := False;   --  connected, sent, read to the end, status 2xx
      Status    : Natural := 0;
      Body_Text : Unbounded_String;   --  also kept when the status is an error
      Why       : Unbounded_String;   --  what went wrong, in plain words
   end record;

   function Post (Host : String; Port : Natural; Path : String; Body_Text : String) return Response;

   function Post_Streaming
     (Host      : String;
      Port      : Natural;
      Path      : String;
      Body_Text : String;
      On_Data   : not null access procedure (Data : String; Stop : out Boolean)) return Response;
   --  Like Post, but On_Data sees the reply body piece by piece as it
   --  arrives (chunked encoding already removed). Stop closes the connection;
   --  the response then holds what arrived up to that point and is Ok.

end Driver.Http;
