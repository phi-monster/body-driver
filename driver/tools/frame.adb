--  frame RECORDING CAMERA BEAT OUTPUT.ppm
--
--  Writes one camera image of one beat of a recording as a binary PPM file,
--  for looking at what an eye saw. CAMERA and BEAT count from 1 and 0 as the
--  driver does (cameras in layout order, beats in arrival order).

with Ada.Command_Line;
with Ada.Streams.Stream_IO;
with Driver.Bytes;
with Driver.Clock;
with Driver.Images;
with Driver.Log;
with Driver.Observations;
with Driver.Protocol;
with Driver.Recording;

procedure Frame is

   use Ada.Command_Line;
   use Driver.Log;
   use type Driver.Recording.Record_Kind;

   Camera : Driver.Observations.Camera_Id;
   Wanted : Natural;
   R      : Driver.Recording.Reader;
   Kind   : Driver.Recording.Record_Kind;
   Ns     : Long_Long_Integer;
   Data   : Driver.Bytes.Buffer;
   More, Known, Done : Boolean := False;
   Layout : Driver.Observations.Layout;
   Beat   : Natural := 0;

   procedure Write (I : Driver.Images.Image) is
      use Ada.Streams.Stream_IO;
      F : File_Type;
      procedure Put_Pixels (RGB : Driver.Bytes.Byte_Array) is
      begin
         Write (F, RGB);
      end Put_Pixels;
   begin
      Create (F, Out_File, Argument (4));
      Write (F, Driver.Bytes.To_Bytes ("P6" & ASCII.LF & Image (Driver.Images.Width (I)) & " "
                                       & Image (Driver.Images.Height (I)) & ASCII.LF & "255" & ASCII.LF));
      Driver.Images.Query (I, Put_Pixels'Access);
      Close (F);
   end Write;

   procedure Robot_Message (Message : Driver.Bytes.Byte_Array) is
      Req : Driver.Protocol.Request;
      Ok  : Boolean;
      O   : Driver.Observations.Observation;
   begin
      Driver.Protocol.Decode (Message, Req, Ok);
      if not Ok or else not Driver.Protocol.Has_Observation (Req) then
         return;
      end if;
      if not Known then
         Driver.Observations.Recognize (Req.Doc, Req.Observation, Layout, Known);
      end if;
      if Known then
         if Beat = Wanted then
            Driver.Observations.Parse (Req.Doc, Req.Observation, Layout, Driver.Clock.Beat (Beat), O);
            if Driver.Observations.Has_Image (O, Camera) then
               Write (O.Images (Camera));
               Line (Core, "wrote camera" & Camera'Image & " of beat" & Beat'Image);
            else
               Line (Core, "camera" & Camera'Image & " has no image at beat" & Beat'Image);
            end if;
            Done := True;
         end if;
         Beat := Beat + 1;
      end if;
   end Robot_Message;

begin
   if Argument_Count /= 4 then
      Line (Core, "usage: frame RECORDING CAMERA BEAT OUTPUT.ppm");
      Set_Exit_Status (Failure);
      return;
   end if;
   Camera := Driver.Observations.Camera_Id'Value (Argument (2));
   Wanted := Natural'Value (Argument (3));
   Driver.Recording.Open (R, Argument (1), More);
   while More and then not Done loop
      Driver.Recording.Next (R, Kind, Ns, Data, More);
      if More and then Kind = Driver.Recording.Robot_Message then
         Data.Query (Robot_Message'Access);
      end if;
   end loop;
   Driver.Recording.Close (R);
   if not Done then
      Line (Core, "the recording has fewer than" & Natural'Image (Wanted + 1) & " beats");
      Set_Exit_Status (Failure);
   end if;
end Frame;
