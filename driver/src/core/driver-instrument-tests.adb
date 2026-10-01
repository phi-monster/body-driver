with Driver.Base64;
with Driver.Bytes;
with Driver.Json;
with Driver.Tests;

package body Driver.Instrument.Tests is

   use Driver.Tests;
   use type Driver.Json.Node;

   procedure Base64_Vectors is
      function E (S : String) return String is (Driver.Base64.Encode (Driver.Bytes.To_Bytes (S)));
   begin
      --  RFC 4648, section 10.
      Check (E ("") = "" and then E ("f") = "Zg==" and then E ("fo") = "Zm8=" and then E ("foo") = "Zm9v",
             "base64 of f, fo, foo");
      Check (E ("foob") = "Zm9vYg==" and then E ("fooba") = "Zm9vYmE=" and then E ("foobar") = "Zm9vYmFy",
             "base64 of foob, fooba, foobar");
   end Base64_Vectors;

   procedure Bitmap_Layout is
      use Driver.Bytes;
      use type Driver.Bytes.Byte_Array;
      --  One pixel of red 1, green 2, blue 3: a 54-byte header, then the pixel
      --  as blue, green, red and one byte of padding to a four-byte row.
      One : constant Driver.Images.Image := Driver.Images.Create (1, 1, [1, 2, 3]);
      Expected : constant Byte_Array (1 .. 58) :=
        [66, 77, 58, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0,
         40, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 24, 0, 0, 0, 0, 0, 4, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         3, 2, 1, 0];
      --  Two rows of one pixel: the bottom row comes first.
      Two : constant Driver.Images.Image := Driver.Images.Create (1, 2, [10, 20, 30, 40, 50, 60]);
   begin
      Check (Bitmap (One) = Driver.Base64.Encode (Expected), "a one-pixel bitmap is laid out wrongly");
      declare
         Header : Byte_Array := Expected (1 .. 54);
      begin
         Header (3) := 62;          --  file size 54 + 2 rows of 4
         Header (23) := 2;          --  height
         Header (35) := 8;          --  image size
         Check (Bitmap (Two) = Driver.Base64.Encode (Header & Byte_Array'[60, 50, 40, 0, 30, 20, 10, 0]),
                "bitmap rows are not bottom up");
      end;
   end Bitmap_Layout;

   procedure Request_Text is
      Doc : Driver.Json.Document;
      Ok  : Boolean;
      Why : Unbounded_String;
      Text : constant String :=
        Match_Request ((Stored => True, Frame => 7), (Stored => True, Frame => 9),
                       [(U => 1.5, V => 2.25), (U => 639.5, V => 0.5)], Round_Trip => True);
   begin
      Driver.Json.Parse (Text, Doc, Ok, Why);
      Check (Ok, "the match request is not JSON: " & To_String (Why));
      if Ok then
         declare
            Root : constant Driver.Json.Node := Driver.Json.Root (Doc);
            Points : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "points");
         begin
            Check (Driver.Json.Number (Doc, Driver.Json.Lookup (Doc, Root, "a_id")) = 7.0
                   and then Driver.Json.Number (Doc, Driver.Json.Lookup (Doc, Root, "b_id")) = 9.0,
                   "stored frames are not referred to by number");
            Check (Driver.Json.Is_True (Doc, Driver.Json.Lookup (Doc, Root, "back")), "the round trip was not asked for");
            Check (Points /= Driver.Json.No_Node and then Driver.Json.Count (Doc, Points) = 2
                   and then Driver.Json.Number (Doc, Driver.Json.Element (Doc, Driver.Json.Element (Doc, Points, 2), 1))
                     = 639.5,
                   "the query points are not sent as they are");
         end;
      end if;
   end Request_Text;

   function Reply_Of (Text : String) return Driver.Services.Reply is
     ((Ok => True, Text => To_Unbounded_String (Text), Why => Null_Unbounded_String));

   procedure Match_Replies is
      R   : Answer_Array (1 .. 2);
      Ok  : Boolean;
      Why : Unbounded_String;
   begin
      Read_Match (Reply_Of ("{""ok"":true,""points"":[[3.5,4.0,0.9],[-1,-1,-1]],""back"":[[1.5,2.0],[0.5,0.5]]}"),
                  True, R, Ok, Why);
      Check (Ok, "a well-formed match reply was refused: " & To_String (Why));
      Check (R (1).Found and then R (1).To.U = 3.5 and then R (1).Back.V = 2.0 and then R (1).Certainty = 0.9,
             "a match was read wrongly");
      Check (not R (2).Found, "an unanswered point (-1) was taken as found");
      Read_Match (Reply_Of ("{""ok"":true,""points"":[[3.5,4.0,0.9]]}"), False, R, Ok, Why);
      Check (not Ok, "a reply with fewer answers than points was accepted");
      Read_Match (Reply_Of ("{""ok"":true,""points"":[[3.5,4.0,0.9],[1,1,1]]}"), True, R, Ok, Why);
      Check (not Ok, "a reply without the round trips asked for was accepted");
      Read_Match ((Ok => False, Text => Null_Unbounded_String, Why => To_Unbounded_String ("down")), True, R, Ok, Why);
      Check (not Ok and then To_String (Why) = "down", "a failed call was not passed on");
   end Match_Replies;

   procedure Segment_Replies is
      M   : Driver.Images.Mask;
      S   : Real;
      Ok  : Boolean;
      Why : Unbounded_String;
   begin
      --  A 3 x 2 image: two off, three on, one off.
      Read_Segment (Reply_Of ("{""ok"":true,""w"":3,""h"":2,""score"":0.8,""runs"":[2,3,1]}"), 3, 2, M, S, Ok, Why);
      Check (Ok, "a well-formed segment reply was refused: " & To_String (Why));
      Check (Driver.Images.Count (M) = 3 and then Driver.Images.Contains (M, 2, 0) and then Driver.Images.Contains (M, 0, 1)
             and then Driver.Images.Contains (M, 1, 1) and then not Driver.Images.Contains (M, 1, 0)
             and then S = 0.8,
             "runs were read wrongly");
      Read_Segment (Reply_Of ("{""ok"":true,""w"":3,""h"":2,""runs"":[2,3]}"), 3, 2, M, S, Ok, Why);
      Check (not Ok, "runs short of the image were accepted");
      Read_Segment (Reply_Of ("{""ok"":true,""w"":4,""h"":2,""runs"":[8]}"), 3, 2, M, S, Ok, Why);
      Check (not Ok, "a mask of another size was accepted");
      Read_Segment (Reply_Of ("{""ok"":true,""w"":3,""h"":2,""runs"":[2,9]}"), 3, 2, M, S, Ok, Why);
      Check (not Ok, "runs past the image were accepted");
   end Segment_Replies;

   procedure Register is
   begin
      Driver.Tests.Register ("instrument.base64", "base64 differs from RFC 4648", Base64_Vectors'Access);
      Driver.Tests.Register ("instrument.bitmap", "images reach the service laid out wrongly", Bitmap_Layout'Access);
      Driver.Tests.Register ("instrument.request", "a match request does not say what was meant", Request_Text'Access);
      Driver.Tests.Register ("instrument.match", "a match reply that does not fit is used", Match_Replies'Access);
      Driver.Tests.Register ("instrument.segment", "a segment reply that does not fit is used", Segment_Replies'Access);
   end Register;

end Driver.Instrument.Tests;
