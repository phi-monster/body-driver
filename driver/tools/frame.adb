--  frame RECORDING CAMERA BEAT OUTPUT.ppm
--  frame RECORDING CAMERA @BEATS OUTPUT_DIR
--
--  Writes one camera image of one beat of a recording as a binary PPM file,
--  for looking at what an eye saw. CAMERA and BEAT count from 1 and 0 as the
--  driver does (cameras in layout order, beats in arrival order). With @BEATS,
--  a file of beats (one a line, in any order), every one of them in one pass
--  over the recording, each as OUTPUT_DIR/eyeCAMERA_BEAT.ppm (the names
--  Replay's --dump writes): a study of an arm's keyframes reads a recording
--  of gigabytes once, not once a keyframe.

with Ada.Command_Line;
with Ada.Containers.Ordered_Sets;
with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;
with Ada.Text_IO;
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

   package Beat_Sets is new Ada.Containers.Ordered_Sets (Natural);

   Camera : Driver.Observations.Camera_Id;
   Wanted : Natural;
   Listed : Boolean := False;   --  the beats come from a file (@BEATS), the pictures go to a folder
   Beats  : Beat_Sets.Set;
   Last   : Natural := 0;       --  the latest beat listed
   R      : Driver.Recording.Reader;
   Kind   : Driver.Recording.Record_Kind;
   Ns     : Long_Long_Integer;
   Data   : Driver.Bytes.Buffer;
   More, Known, Done : Boolean := False;
   Layout : Driver.Observations.Layout;
   Beat   : Natural := 0;

   procedure Write (I : Driver.Images.Image; Name : String) is
      use Ada.Streams.Stream_IO;
      F : File_Type;
      procedure Put_Pixels (RGB : Driver.Bytes.Byte_Array) is
      begin
         Write (F, RGB);
      end Put_Pixels;
   begin
      Create (F, Out_File, Name);
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
         if (Listed and then Beats.Contains (Beat)) or else (not Listed and then Beat = Wanted) then
            Driver.Observations.Parse (Req.Doc, Req.Observation, Layout, Driver.Clock.Beat (Beat), O);
            if Driver.Observations.Has_Image (O, Camera) then
               Write (O.Images (Camera),
                      (if Listed
                       then Argument (4) & "/eye" & Ada.Strings.Fixed.Trim (Camera'Image, Ada.Strings.Left) & "_"
                            & Ada.Strings.Fixed.Trim (Beat'Image, Ada.Strings.Left) & ".ppm"
                       else Argument (4)));
               Line (Core, "wrote camera" & Camera'Image & " of beat" & Beat'Image);
            else
               Line (Core, "camera" & Camera'Image & " has no image at beat" & Beat'Image);
            end if;
            Done := not Listed or else Beat >= Last;
         end if;
         Beat := Beat + 1;
      end if;
   end Robot_Message;

begin
   if Argument_Count /= 4 then
      Line (Core, "usage: frame RECORDING CAMERA BEAT OUTPUT.ppm | frame RECORDING CAMERA @BEATS OUTPUT_DIR");
      Set_Exit_Status (Failure);
      return;
   end if;
   Camera := Driver.Observations.Camera_Id'Value (Argument (2));
   Listed := Argument (3)'Length > 1 and then Argument (3) (Argument (3)'First) = '@';
   if Listed then
      declare
         use Ada.Text_IO;
         F : File_Type;
      begin
         Open (F, In_File, Argument (3) (Argument (3)'First + 1 .. Argument (3)'Last));
         while not End_Of_File (F) loop
            declare
               L : constant String := Ada.Strings.Fixed.Trim (Get_Line (F), Ada.Strings.Both);
            begin
               if L'Length > 0 then
                  Beats.Include (Natural'Value (L));
                  Last := Natural'Max (Last, Natural'Value (L));
               end if;
            end;
         end loop;
         Close (F);
      end;
      Wanted := Last;
   else
      Wanted := Natural'Value (Argument (3));
   end if;
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
