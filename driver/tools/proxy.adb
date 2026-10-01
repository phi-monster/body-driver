--  proxy LISTEN_PORT DRIVER_HOST DRIVER_PORT RECORDING
--
--  The wire recorder: listens where the robot expects its driver, passes
--  every message on unchanged in both directions, and records both
--  directions in the format of the driver's own --record (Driver.Recording).
--  It never decodes a message, so the recording is the exact conversation
--  whichever driver answered; it is how a driver that cannot record itself is
--  recorded. Each robot connection gets its own connection to the driver, and
--  either side closing ends both; the proxy then waits for the next robot.
--  Every record is flushed whole, so stopping the proxy from outside loses at
--  most the message in flight.

with Ada.Command_Line;
with Ada.Text_IO;
with Driver.Bytes;
with Driver.Recording;
with Driver.Wire;

procedure Proxy is

   use Ada.Command_Line;
   use Driver.Recording;
   use type Driver.Wire.Message_Kind;

   type Side is (Robot_Side, Driver_Side);

   Ends : array (Side) of Driver.Wire.Connection;

   function Record_Of (From : Side; Kind : Driver.Wire.Message_Kind) return Record_Kind is
     (case From is
         when Robot_Side  => (if Kind = Driver.Wire.Text then Robot_Text else Robot_Message),
         when Driver_Side => (if Kind = Driver.Wire.Text then Driver_Text else Driver_Message));

   task type Pump (From : Side);
   --  Carries one direction until either end closes, then closes the other.

   task body Pump is
      To      : constant Side := (if From = Robot_Side then Driver_Side else Robot_Side);
      Kind    : Driver.Wire.Message_Kind;
      Message : Driver.Bytes.Buffer;
      Ok      : Boolean := True;

      procedure Forward (Data : Driver.Bytes.Byte_Array) is
      begin
         Write_Shared (Record_Of (From, Kind), Data);
         Driver.Wire.Send (Ends (To), Data, Ok, Kind);
      end Forward;
   begin
      loop
         Driver.Wire.Receive (Ends (From), Kind, Message);
         exit when Kind = Driver.Wire.Closed;
         Message.Query (Forward'Access);
         exit when not Ok;
      end loop;
      Driver.Wire.Disconnect (Ends (To));
   end Pump;

   Ok : Boolean;

begin
   if Argument_Count /= 4 then
      Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error, "usage: proxy LISTEN_PORT DRIVER_HOST DRIVER_PORT RECORDING");
      Set_Exit_Status (Failure);
      return;
   end if;
   Driver.Wire.Listen (Ends (Robot_Side), Natural'Value (Argument (1)), Ok);
   if not Ok then
      Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error, "proxy: cannot listen on port " & Argument (1));
      Set_Exit_Status (Failure);
      return;
   end if;
   Start_Shared (Argument (4));
   loop
      Driver.Wire.Accept_Client (Ends (Robot_Side), Ok);
      if Ok then
         Write_Shared (Connection, Driver.Bytes.To_Bytes (""));
         Driver.Wire.Connect (Ends (Driver_Side), Argument (2), Natural'Value (Argument (3)), Ok);
         if Ok then
            declare
               Up   : Pump (Robot_Side);
               Down : Pump (Driver_Side);
            begin
               null;   --  the block ends when both directions have ended
            end;
         else
            Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error, "proxy: the driver did not accept a connection");
            Driver.Wire.Disconnect (Ends (Robot_Side));
         end if;
      end if;
   end loop;
end Proxy;
