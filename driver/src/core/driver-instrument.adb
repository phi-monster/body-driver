with Driver.Base64;
with Driver.Bytes;
with Driver.Json;

package body Driver.Instrument is

   use Driver.Bytes;
   use type Driver.Json.Kind;
   use type Driver.Json.Node;

   --  The 24-bit BMP layout (a BITMAPFILEHEADER and a BITMAPINFOHEADER).
   File_Header    : constant := 14;
   Info_Header    : constant := 40;
   Bits_Per_Pixel : constant := 24;
   Row_Alignment  : constant := 4;   --  every row is padded to a multiple of four bytes
   Channels       : constant := 3;

   procedure Put_Little (B : in out Buffer; Value : Natural; Width : Positive);
   --  An unsigned integer in Width bytes, least significant first.

   procedure Put_Little (B : in out Buffer; Value : Natural; Width : Positive) is
      V : Natural := Value;
   begin
      for I in 1 .. Width loop
         B.Append (Byte (V mod 2 ** 8));
         V := V / 2 ** 8;
      end loop;
   end Put_Little;

   function Bitmap (I : Driver.Images.Image) return String is
      W        : constant Natural := Driver.Images.Width (I);
      H        : constant Natural := Driver.Images.Height (I);
      Row_Size : constant Natural := (Channels * W + Row_Alignment - 1) / Row_Alignment * Row_Alignment;
      Offset_Of_Pixels : constant Natural := File_Header + Info_Header;
      B        : Buffer;
   begin
      B.Append ("BM");
      Put_Little (B, Offset_Of_Pixels + Row_Size * H, 4);
      Put_Little (B, 0, 4);                     --  two reserved words
      Put_Little (B, Offset_Of_Pixels, 4);
      Put_Little (B, Info_Header, 4);
      Put_Little (B, W, 4);
      Put_Little (B, H, 4);                     --  positive: rows from the bottom up
      Put_Little (B, 1, 2);                     --  one plane
      Put_Little (B, Bits_Per_Pixel, 2);
      Put_Little (B, 0, 4);                     --  no compression
      Put_Little (B, Row_Size * H, 4);
      Put_Little (B, 0, 4 * 4);                 --  resolution and palette: unused
      for Row in reverse 0 .. H - 1 loop
         for Column in 0 .. W - 1 loop
            B.Append (Byte (Driver.Images.Blue (I, Column, Row)));
            B.Append (Byte (Driver.Images.Green (I, Column, Row)));
            B.Append (Byte (Driver.Images.Red (I, Column, Row)));
         end loop;
         for Pad in Channels * W + 1 .. Row_Size loop
            B.Append (Byte'(0));
         end loop;
      end loop;
      return Driver.Base64.Encode (B.To_Array);
   end Bitmap;

   function Quoted_Image (I : Driver.Images.Image) return String is ("""" & Bitmap (I) & """");

   procedure Store (I : Driver.Images.Image; Frame : out Frame_Number; Ok : out Boolean; Why : out Unbounded_String) is
      Reply : constant Driver.Services.Reply :=
        Driver.Services.Call (Driver.Services.Instrument, "/frame", "{""image"":" & Quoted_Image (I) & "}");
      Doc   : Driver.Json.Document;
   begin
      Frame := 0;
      Ok := False;
      Why := Reply.Why;
      if not Reply.Ok then
         return;
      end if;
      Driver.Json.Parse (To_String (Reply.Text), Doc, Ok, Why);
      if not Ok then
         return;
      end if;
      declare
         Id : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Driver.Json.Root (Doc), "id");
      begin
         Ok := Id /= Driver.Json.No_Node and then Driver.Json.Kind_Of (Doc, Id) = Driver.Json.Number_Value
           and then Driver.Json.Number (Doc, Id) >= 0.0;
         if Ok then
            Frame := Frame_Number (Driver.Json.Number (Doc, Id));
         else
            Why := To_Unbounded_String ("the frame reply has no frame number");
         end if;
      end;
   end Store;

   function Source_Field (Name : String; S : Source) return String is
     (if S.Stored then """" & Name & "_id"":" & Natural'Image (Natural (S.Frame))
      else """" & Name & """:" & Quoted_Image (S.Image));

   function Match_Request (A, B : Source; Points : Point_Array; Round_Trip : Boolean) return String is
      R : Unbounded_String := To_Unbounded_String ("{" & Source_Field ("a", A) & "," & Source_Field ("b", B)
                                                   & ",""num"":0,""points"":[");
   begin
      for K in Points'Range loop
         Append (R, (if K > Points'First then ",[" else "[") & Driver.Json.Number_Image (Points (K).U) & ","
                 & Driver.Json.Number_Image (Points (K).V) & "]");
      end loop;
      Append (R, "],""back"":" & (if Round_Trip then "true" else "false") & "}");
      return To_String (R);
   end Match_Request;

   --  The pair or triple of numbers of a reply array element, or nothing.
   function Pair (Doc : Driver.Json.Document; N : Driver.Json.Node; U, V : out Real) return Boolean is
   begin
      U := 0.0;
      V := 0.0;
      if N = Driver.Json.No_Node or else Driver.Json.Kind_Of (Doc, N) /= Driver.Json.Array_Value
        or else Driver.Json.Count (Doc, N) < 2
      then
         return False;
      end if;
      U := Driver.Json.Number (Doc, Driver.Json.Element (Doc, N, 1));
      V := Driver.Json.Number (Doc, Driver.Json.Element (Doc, N, 2));
      --  The service writes -1 for what it could not answer; NaN fails every comparison.
      return U >= 0.0 and then V >= 0.0;
   end Pair;

   procedure Read_Match
     (Reply      : Driver.Services.Reply;
      Round_Trip : Boolean;
      Result     : out Answer_Array;
      Ok         : out Boolean;
      Why        : out Unbounded_String)
   is
      Doc : Driver.Json.Document;
   begin
      Result := [others => (others => <>)];
      Ok := False;
      Why := Reply.Why;
      if not Reply.Ok then
         return;
      end if;
      Driver.Json.Parse (To_String (Reply.Text), Doc, Ok, Why);
      if not Ok then
         return;
      end if;
      Ok := False;
      declare
         Root   : constant Driver.Json.Node := Driver.Json.Root (Doc);
         Points : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "points");
         Back   : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "back");
      begin
         if Points = Driver.Json.No_Node or else Driver.Json.Count (Doc, Points) /= Result'Length then
            Why := To_Unbounded_String ("the match reply answers another number of points than was asked");
            return;
         end if;
         if Round_Trip and then (Back = Driver.Json.No_Node or else Driver.Json.Count (Doc, Back) /= Result'Length) then
            Why := To_Unbounded_String ("the match reply has no round trip for every point");
            return;
         end if;
         for K in Result'Range loop
            declare
               Index : constant Positive := K - Result'First + 1;
               E     : constant Driver.Json.Node := Driver.Json.Element (Doc, Points, Index);
               M     : Answer;
            begin
               M.Found := Pair (Doc, E, M.To.U, M.To.V);
               if M.Found and then Driver.Json.Count (Doc, E) > 2 then
                  M.Certainty := Driver.Json.Number (Doc, Driver.Json.Element (Doc, E, 3));
               end if;
               if Round_Trip then
                  M.Found := Pair (Doc, Driver.Json.Element (Doc, Back, Index), M.Back.U, M.Back.V) and then M.Found;
               end if;
               Result (K) := M;
            end;
         end loop;
         Ok := True;
      end;
   end Read_Match;

   procedure Match
     (A, B       : Source;
      Points     : Point_Array;
      Round_Trip : Boolean;
      Result     : out Answer_Array;
      Ok         : out Boolean;
      Why        : out Unbounded_String)
   is
   begin
      Read_Match (Driver.Services.Call (Driver.Services.Instrument, "/match", Match_Request (A, B, Points, Round_Trip)),
                  Round_Trip, Result, Ok, Why);
   end Match;

   function Submit_Match
     (A, B : Source; Points : Point_Array; Round_Trip : Boolean; Beat : Driver.Clock.Beat)
      return Driver.Services.Ticket is
     (Driver.Services.Submit (Driver.Services.Instrument, "/match", Match_Request (A, B, Points, Round_Trip), Beat));

   function Segment_Request (I : Driver.Images.Image; Has_Box : Boolean; Around : Box; Points : Prompt_Array)
     return String
   is
      R : Unbounded_String := To_Unbounded_String ("{""image"":" & Quoted_Image (I));
   begin
      if Has_Box then
         Append (R, ",""box"":[" & Driver.Json.Number_Image (Around.X0) & "," & Driver.Json.Number_Image (Around.Y0)
                 & "," & Driver.Json.Number_Image (Around.X1) & "," & Driver.Json.Number_Image (Around.Y1) & "]");
      end if;
      if Points'Length > 0 then
         Append (R, ",""points"":[");
         for K in Points'Range loop
            Append (R, (if K > Points'First then "," else "") & "[" & Driver.Json.Number_Image (Points (K).At_Pixel.U)
                    & "," & Driver.Json.Number_Image (Points (K).At_Pixel.V) & ","
                    & (if Points (K).On then "1" else "0") & "]");
         end loop;
         Append (R, "]");
      end if;
      Append (R, "}");
      return To_String (R);
   end Segment_Request;

   procedure Read_Segment
     (Reply  : Driver.Services.Reply;
      Width  : Positive;
      Height : Positive;
      Region : out Driver.Images.Mask;
      Score  : out Real;
      Ok     : out Boolean;
      Why    : out Unbounded_String)
   is
      Doc : Driver.Json.Document;
   begin
      Region := Driver.Images.Create (Width, Height);
      Score := 0.0;
      Ok := False;
      Why := Reply.Why;
      if not Reply.Ok then
         return;
      end if;
      Driver.Json.Parse (To_String (Reply.Text), Doc, Ok, Why);
      if not Ok then
         return;
      end if;
      Ok := False;
      declare
         Root : constant Driver.Json.Node := Driver.Json.Root (Doc);
         W    : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "w");
         H    : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "h");
         Runs : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "runs");
         S    : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "score");
         At_Pixel : Natural := 0;
         Total    : constant Natural := Width * Height;
      begin
         if W = Driver.Json.No_Node or else H = Driver.Json.No_Node or else Runs = Driver.Json.No_Node
           or else Driver.Json.Number (Doc, W) /= Real (Width) or else Driver.Json.Number (Doc, H) /= Real (Height)
         then
            Why := To_Unbounded_String ("the segment reply is for another image size");
            return;
         end if;
         --  Runs alternate: pixels off the thing, then on it, starting off it.
         for K in 1 .. Driver.Json.Count (Doc, Runs) loop
            declare
               Run : constant Real := Driver.Json.Number (Doc, Driver.Json.Element (Doc, Runs, K));
            begin
               if not (Run >= 0.0) or else Run > Real (Total - At_Pixel) then
                  Why := To_Unbounded_String ("the segment reply's runs run past the image");
                  return;
               end if;
               if K mod 2 = 0 then
                  for P in At_Pixel .. At_Pixel + Natural (Run) - 1 loop
                     Driver.Images.Include (Region, P mod Width, P / Width);
                  end loop;
               end if;
               At_Pixel := At_Pixel + Natural (Run);
            end;
         end loop;
         if At_Pixel /= Total then
            Why := To_Unbounded_String ("the segment reply's runs do not cover the image");
            Region := Driver.Images.Create (Width, Height);
            return;
         end if;
         if S /= Driver.Json.No_Node then
            Score := Driver.Json.Number (Doc, S);
         end if;
         Ok := True;
      end;
   end Read_Segment;

   procedure Segment
     (I       : Driver.Images.Image;
      Has_Box : Boolean;
      Around  : Box;
      Points  : Prompt_Array;
      Region  : out Driver.Images.Mask;
      Score   : out Real;
      Ok      : out Boolean;
      Why     : out Unbounded_String)
   is
   begin
      Read_Segment (Driver.Services.Call (Driver.Services.Instrument, "/segment", Segment_Request (I, Has_Box, Around, Points)),
                    Driver.Images.Width (I), Driver.Images.Height (I), Region, Score, Ok, Why);
   end Segment;

   function Submit_Segment
     (I : Driver.Images.Image; Has_Box : Boolean; Around : Box; Points : Prompt_Array; Beat : Driver.Clock.Beat)
      return Driver.Services.Ticket is
     (Driver.Services.Submit (Driver.Services.Instrument, "/segment", Segment_Request (I, Has_Box, Around, Points), Beat));

end Driver.Instrument;
