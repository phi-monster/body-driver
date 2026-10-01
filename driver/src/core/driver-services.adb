package body Driver.Services is

   --  The HTTP transport and the recording of replies are in progress.

   procedure Configure (S : Service; Host : String; Port : Natural) is
      pragma Unreferenced (S, Host, Port);
   begin
      null;
   end Configure;

   Not_Ready : constant Reply := (Ok => False, Text => Null_Unbounded_String,
                                  Why => To_Unbounded_String ("service transport not built yet"));

   function Call (S : Service; Path : String; Request : String) return Reply is
      pragma Unreferenced (S, Path, Request);
   begin
      return Not_Ready;
   end Call;

   function Call_Streaming
     (S       : Service;
      Path    : String;
      Request : String;
      On_Text : not null access procedure (Chunk : String; Stop : out Boolean)) return Reply
   is
      pragma Unreferenced (S, Path, Request, On_Text);
   begin
      return Not_Ready;
   end Call_Streaming;

   function Submit (S : Service; Path : String; Request : String; Beat : Driver.Clock.Beat) return Ticket is
      pragma Unreferenced (S, Path, Request, Beat);
   begin
      return 0;
   end Submit;

   function Ready (T : Ticket) return Boolean is (T > 0);

   function Collect (T : Ticket) return Reply is
      pragma Unreferenced (T);
   begin
      return Not_Ready;
   end Collect;

end Driver.Services;
