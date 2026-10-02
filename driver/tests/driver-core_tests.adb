with Ada.Numerics;
with Ada.Streams;
with GNAT.Sockets;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Driver.Beats;
with Driver.Bytes;
with Driver.Clock;
with Driver.Commands;
with Driver.Images;
with Driver.Json;
with Driver.Msgpack;
with Driver.Observations;
with Driver.Replies;
with Driver.Services;
with Driver.Numerics.Dense;
with Driver.Stats;
with Driver.Tests;
with Driver.Uncertain;

package body Driver.Core_Tests is

   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use type Driver.Bytes.Byte;
   use type Driver.Bytes.Offset;

   Pi : constant := Ada.Numerics.Pi;

   function Max_Abs (A : Mat3) return Real is
      M : Real := 0.0;
   begin
      for I in 1 .. 3 loop
         for J in 1 .. 3 loop
            M := Real'Max (M, abs A (I, J));
         end loop;
      end loop;
      return M;
   end Max_Abs;

   procedure Rotation_Round_Trip is
      Samples : constant array (Positive range <>) of Vec3 :=
        [[0.3, -0.2, 0.9], [1.0e-9, 0.0, 0.0], [0.0, Pi - 1.0e-6, 0.0], [Pi, 0.0, 0.0],
         [-1.2, 2.0, 0.4], [0.0, 0.0, 0.0]];
   begin
      for W of Samples loop
         declare
            R    : constant Mat3 := Exp (W);
            Back : constant Mat3 := Exp (Log (R));
         begin
            Check (Max_Abs (R * Transpose (R) - Identity3) < 1.0e-12, "Exp is not orthonormal");
            Check (Max_Abs (Back - R) < 1.0e-9, "Exp (Log (R)) differs from R");
            Check (Angle (R) <= Pi + 1.0e-12, "Log angle outside [0, pi]");
         end;
      end loop;
   end Rotation_Round_Trip;

   procedure Quaternion_Round_Trip is
      R : constant Mat3 := Exp ([0.4, -1.9, 0.7]);
   begin
      Check (Max_Abs (To_Matrix (To_Quaternion (R)) - R) < 1.0e-12, "quaternion round trip");
      Check (Max_Abs (To_Matrix (To_Quaternion (Exp ([Pi, 0.0, 0.0]))) - Exp ([Pi, 0.0, 0.0])) < 1.0e-12,
             "quaternion round trip at pi");
   end Quaternion_Round_Trip;

   procedure Rigid_Inverse is
      T : constant Rigid := (Rotation => Exp ([0.2, 0.5, -0.3]), Translation => [1.0, -2.0, 0.5]);
      P : constant Vec3 := [0.3, 0.4, -0.7];
   begin
      Check (abs (Inverse (T) * (T * P) - P) < 1.0e-12, "inverse does not undo");
      Check (abs ((T * Inverse (T)) * P - P) < 1.0e-12, "T * inverse is not the identity");
   end Rigid_Inverse;

   procedure Least_Squares_Exact is
      A : constant Real_Matrix := [[1.0, 0.0], [1.0, 1.0], [1.0, 2.0], [1.0, 3.0]];
      B : constant Real_Vector := [2.0, 5.0, 8.0, 11.0];
      X : Real_Vector (1 .. 2);
      Ok : Boolean;
   begin
      Driver.Numerics.Dense.Least_Squares (A, B, X, Ok);
      Check (Ok, "full-rank system reported deficient");
      Check_Close (X (1), 2.0, 1.0e-12, "intercept");
      Check_Close (X (2), 3.0, 1.0e-12, "slope");
      Driver.Numerics.Dense.Least_Squares ([[1.0, 2.0], [2.0, 4.0], [3.0, 6.0]], [1.0, 2.0, 3.0], X, Ok);
      Check (not Ok, "rank-deficient system reported full rank");
   end Least_Squares_Exact;

   procedure Cholesky_Solve is
      A : constant Real_Matrix := [[4.0, 2.0, 0.4], [2.0, 5.0, 1.0], [0.4, 1.0, 3.0]];
      B : constant Real_Vector := [1.0, -2.0, 0.5];
      L : Real_Matrix (1 .. 3, 1 .. 3);
      Ok : Boolean;
   begin
      Driver.Numerics.Dense.Cholesky (A, L, Ok);
      Check (Ok, "positive-definite matrix rejected");
      Check (abs (A * Driver.Numerics.Dense.Cholesky_Solve (L, B) - B) < 1.0e-12, "solution does not satisfy A x = b");
      declare
         L2 : Real_Matrix (1 .. 2, 1 .. 2);
      begin
         Driver.Numerics.Dense.Cholesky ([[1.0, 2.0], [2.0, 1.0]], L2, Ok);
         Check (not Ok, "indefinite matrix accepted");
      end;
   end Cholesky_Solve;

   procedure Robust_Statistics is
      X : constant Real_Array := [1.0, 2.0, 3.0, 4.0, 1000.0];
   begin
      Check_Close (Driver.Stats.Median (X), 3.0, 0.0, "median");
      Check (Driver.Stats.Robust_Sigma (X) < 2.0, "one outlier moves the robust sigma");
      Check_Close (Driver.Stats.Correlation ([1.0, 2.0, 3.0], [2.0, 4.0, 6.0]), 1.0, 1.0e-12, "perfect correlation");
      declare
         L : constant Driver.Stats.Line := Driver.Stats.Fit_Line ([0.0, 1.0, 2.0, 3.0], [1.0, 3.0, 5.0, 7.0]);
      begin
         Check_Close (L.Slope.Value, 2.0, 1.0e-12, "line slope");
         Check_Close (L.Intercept.Value, 1.0, 1.0e-12, "line intercept");
      end;
   end Robust_Statistics;

   procedure Significance is
      use Driver.Uncertain;
   begin
      Check (Significant (3.1, 1.0), "3.1 sigma not significant");
      Check (not Significant (2.9, 1.0), "2.9 sigma significant");
      Check (not Significant (1.0e9, Real'Last), "unknown sigma made something significant");
      Check (not Significant (Estimate'(10.0, 2.0, 0), Estimate'(5.0, 2.0, 0)), "5 apart with combined sigma 2.83");
      Check (Significant (Estimate'(10.0, 1.0, 0), Estimate'(5.0, 1.0, 0)), "5 apart with combined sigma 1.41");
   end Significance;

   procedure Point_Significance is
      use Driver.Uncertain;
      use Ada.Numerics.Long_Elementary_Functions;

      function Point (Mean : Vec3; Variances : Vec3) return Point_Estimate is
         P : Point_Estimate := (Mean => Mean, Covariance => [others => [others => 0.0]]);
      begin
         for I in 1 .. 3 loop
            P.Covariance (I, I) := Variances (I);
         end loop;
         return P;
      end Point;

      Half_Each : constant Vec3 := [0.5, 0.5, 0.5];   --  two such points: unit variance per axis
      Origin    : constant Point_Estimate := Point (Zero3, Half_Each);
      G         : Ada.Numerics.Float_Random.Generator;
      Draws     : constant := 40_000;
      Alarms    : Natural := 0;

      function Gaussian return Real is
         --  Box and Muller; 1 - U keeps the logarithm finite.
         U : constant Real := 1.0 - Real (Ada.Numerics.Float_Random.Random (G));
         V : constant Real := Real (Ada.Numerics.Float_Random.Random (G));
      begin
         return Sqrt (-2.0 * Log (U)) * Cos (2.0 * Pi * V);
      end Gaussian;
   begin
      --  The false-alarm rate under pure noise is the scalar rule's, not the
      --  far higher rate of a length tested along the direction noise chose.
      Ada.Numerics.Float_Random.Reset (G, 1);
      for K in 1 .. Draws loop
         if Significant (Origin, Point ([Gaussian, Gaussian, Gaussian], Half_Each)) then
            Alarms := Alarms + 1;
         end if;
      end loop;
      declare
         P : constant Real := 2.0 * (1.0 - 0.998_650_101_968_369_9);   --  two-sided Gaussian tail at Z = 3
         Expected : constant Real := Real (Draws) * P;
      begin
         Check (abs (Real (Alarms) - Expected) <= 3.0 * Sqrt (Expected * (1.0 - P)),
                "isotropic noise raised" & Natural'Image (Alarms) & " alarms in" & Natural'Image (Draws)
                & ", expected about" & Natural'Image (Natural (Expected)));
      end;
      Check (not Significant (Origin, Point ([3.5, 0.0, 0.0], Half_Each)),
             "3.5 sigma in three dimensions is within the 0.27 % tail of a chi with 3 degrees of freedom");
      Check (Significant (Origin, Point ([4.0, 0.0, 0.0], Half_Each)), "4 sigma in three dimensions not significant");
      --  Along the separation the spread looks wide; across the narrow axis it is not.
      Check (Significant (Point (Zero3, [0.5, 0.005, 0.5]), Point ([2.8, 0.28, 0.0], [0.5, 0.005, 0.5])),
             "a separation hidden by projecting onto its own direction was missed");
      Check (Significant (Point (Zero3, [1.0, 1.0, 0.0]), Point ([0.0, 0.0, 1.0e-6], [0.0, 0.0, 0.0])),
             "a separation along a direction both points pin down exactly was not significant");
      Check (not Significant (Point (Zero3, [1.0, 1.0, 0.0]), Point ([1.0, 1.0, 0.0], [0.0, 0.0, 0.0])),
             "a separation within the spread of a degenerate covariance was significant");
      Check (not Significant (Origin, (Mean => [1.0e9, 0.0, 0.0], others => <>)),
             "an unknown covariance made a separation significant");
   end Point_Significance;

   procedure Clock_Range is
   begin
      Check (Driver.Clock.Nanoseconds_Of (1.0) = 1_000_000_000, "one second");
      Check (Driver.Clock.Nanoseconds_Of (Duration'Small) = 1, "one nanosecond");
      Check (Driver.Clock.Nanoseconds_Of (3_600.0) = 3_600_000_000_000, "an hour");
      Check (Driver.Clock.Nanoseconds_Of (8_640_000.0) = 8_640_000_000_000_000, "a hundred days");
   end Clock_Range;

   procedure Buffer_Growth is
      B : Driver.Bytes.Buffer;
      Empty : constant Driver.Bytes.Byte_Array (1 .. 0) := [others => 0];
   begin
      B.Append (Empty);
      Check (B.Length = 0, "appending nothing to an empty buffer");
      for I in 1 .. 1000 loop
         B.Append (Driver.Bytes.Byte (I mod 256));
      end loop;
      B.Append ("abc");
      Check (B.Length = 1003, "length after appends");
      Check (B.Element (1000) = Driver.Bytes.Byte (1000 mod 256), "element kept across growth");
      declare
         C : constant Driver.Bytes.Buffer := B;
      begin
         B.Clear;
         Check (C.Length = 1003, "a copy shares storage with its source");
      end;
   end Buffer_Growth;

   procedure Image_Access is
      use Driver.Bytes;
      Data : Byte_Array (1 .. 3 * 4 * 2) := [others => 0];
   begin
      Data (3 * (1 * 4 + 2) + 1) := 200;   --  red of column 2, row 1
      declare
         I : constant Driver.Images.Image := Driver.Images.Create (4, 2, Data);
         M : Driver.Images.Mask := Driver.Images.Create (4, 2);
      begin
         Check (Driver.Images.Red (I, 2, 1) = 200, "pixel addressed by column and row");
         Check (Driver.Images.Red (I, 1, 2 - 1) = 0, "neighbouring pixel");
         Driver.Images.Include (M, 3, 1);
         Check (Driver.Images.Contains (M, 3, 1) and then Driver.Images.Count (M) = 1, "mask membership");
      end;
      --  The whole-image luma is the per-pixel luma, pixel for pixel, in row order.
      for K in Data'Range loop
         Data (K) := Byte ((7 * Natural (K)) mod 256);
      end loop;
      declare
         I     : constant Driver.Images.Image := Driver.Images.Create (4, 2, Data);
         Whole : Driver.Real_Array (1 .. 8);
      begin
         Driver.Images.Luma (I, Whole);
         for Row in 0 .. 1 loop
            for Column in 0 .. 3 loop
               Check (Whole (Row * 4 + Column + 1) = Driver.Images.Luma (I, Column, Row),
                      "the whole-image luma of column" & Column'Image & ", row" & Row'Image & " differs");
            end loop;
         end loop;
      end;
   end Image_Access;


   procedure Msgpack_Round_Trip is
      use Driver.Msgpack;
      B   : Driver.Bytes.Buffer;
      Doc : Document;
      Ok  : Boolean;
   begin
      Put_Map_Header (B, 4);
      Put_String (B, "ints");
      Put_Array_Header (B, 5);
      Put_Integer (B, 5); Put_Integer (B, -7); Put_Integer (B, -129); Put_Integer (B, 70_000); Put_Integer (B, -2 ** 40);
      Put_String (B, "x");
      Put_Float (B, -0.125);
      Put_String (B, "flag");
      Put_Boolean (B, True);
      Put_String (B, "nested");
      Put_Array_Header (B, 2);
      Put_Array_Header (B, 2); Put_Float (B, 1.0); Put_Float (B, 2.0);
      Put_Array_Header (B, 2); Put_Float (B, 3.0); Put_Float (B, 4.0);
      Decode (B.To_Array, Doc, Ok);
      Check (Ok, "a well-formed document did not decode");
      declare
         Ints : constant Real_Array := Numbers (Doc, Lookup (Doc, Root (Doc), "ints"));
         Nest : constant Natural_Array := Shape (Doc, Lookup (Doc, Root (Doc), "nested"));
      begin
         Check (Ints = [5.0, -7.0, -129.0, 70_000.0, -2.0 ** 40], "integers changed in a round trip");
         Check_Close (Number (Doc, Lookup (Doc, Root (Doc), "x")), -0.125, 0.0, "float");
         Check (Is_True (Doc, Lookup (Doc, Root (Doc), "flag")), "boolean");
         Check (Nest = [2, 2], "shape of a nest of plain arrays");
      end;
      Decode (Driver.Bytes.To_Bytes ("" & Character'Val (16#92#) & Character'Val (1)), Doc, Ok);
      Check (not Ok, "a truncated array decoded");
   end Msgpack_Round_Trip;

   procedure Put_Ndarray (B : in out Driver.Bytes.Buffer; Dtype : String; Shape : Natural_Array;
                          Data : Driver.Bytes.Byte_Array) is
      use Driver.Msgpack;
   begin
      Put_Map_Header (B, 4);
      Put_String (B, "nd"); Put_Boolean (B, True);
      Put_String (B, "type"); Put_String (B, Dtype);
      Put_String (B, "shape"); Put_Array_Header (B, Shape'Length);
      for S of Shape loop
         Put_Integer (B, Long_Long_Integer (S));
      end loop;
      Put_String (B, "data"); Put_Binary (B, Data);
   end Put_Ndarray;

   procedure Ndarray_Dtypes is
      use Driver.Msgpack;
      B   : Driver.Bytes.Buffer;
      Doc : Document;
      Ok  : Boolean;
   begin
      --  Big-endian int16 -2 and 3, then little-endian float16 1.5 (0x3E00).
      Put_Map_Header (B, 2);
      Put_String (B, "i");
      Put_Ndarray (B, ">i2", [2], [16#FF#, 16#FE#, 16#00#, 16#03#]);
      Put_String (B, "h");
      Put_Ndarray (B, "<f2", [1], [16#00#, 16#3E#]);
      Decode (B.To_Array, Doc, Ok);
      Check (Ok, "ndarray document did not decode");
      Check (Numbers (Doc, Lookup (Doc, Root (Doc), "i")) = [-2.0, 3.0], "big-endian int16 misread");
      Check (Numbers (Doc, Lookup (Doc, Root (Doc), "h")) = [1.5], "float16 misread");
   end Ndarray_Dtypes;

   procedure Layout_By_Shape is
      use Driver.Msgpack;
      use Driver.Observations;
      use Ada.Strings.Unbounded;
      B   : Driver.Bytes.Buffer;
      Doc : Document;
      Ok  : Boolean;
      L   : Layout;
      O   : Observation;
      Pixels : constant Driver.Bytes.Byte_Array (1 .. 3 * 4 * 2) := [others => 7];
      Depth  : constant Driver.Bytes.Byte_Array (1 .. 4 * 4 * 2) := [others => 0];
      K      : constant Driver.Bytes.Byte_Array (1 .. 4 * 9) := [others => 0];
   begin
      --  {cam: {rgb: u1 2x4x3, size: [2, 4], depth: f4 2x4, k: f4 3x3},
      --   state: {arm: [0.1, 0.2]}, echo: {arm: [0.1, 0.2]}, mode: [3.0], instruction: "go"}
      Put_Map_Header (B, 5);
      Put_String (B, "cam");
      Put_Map_Header (B, 4);
      Put_String (B, "rgb"); Put_Ndarray (B, "|u1", [2, 4, 3], Pixels);
      Put_String (B, "size"); Put_Array_Header (B, 2); Put_Integer (B, 2); Put_Integer (B, 4);
      Put_String (B, "depth"); Put_Ndarray (B, "<f4", [2, 4], Depth);
      Put_String (B, "k"); Put_Ndarray (B, "<f4", [3, 3], K);
      Put_String (B, "state"); Put_Map_Header (B, 1);
      Put_String (B, "arm"); Put_Array_Header (B, 2); Put_Float (B, 0.1); Put_Float (B, 0.2);
      Put_String (B, "echo"); Put_Map_Header (B, 1);
      Put_String (B, "arm"); Put_Array_Header (B, 2); Put_Float (B, 0.1); Put_Float (B, 0.2);
      Put_String (B, "mode"); Put_Array_Header (B, 1); Put_Float (B, 3.0);
      Put_String (B, "instruction"); Put_String (B, "go");
      Decode (B.To_Array, Doc, Ok);
      Recognize (Doc, Root (Doc), L, Ok);
      Check (Ok, "a layout with a camera and a group was rejected");
      Check (Natural (L.Cameras.Length) = 1 and then L.Cameras (1).Width = 4 and then L.Cameras (1).Height = 2,
             "camera not recognized by its shape");
      Check (To_String (L.Cameras (1).Depth_Path) = "cam/depth", "depth not paired with the camera of its size");
      Check (Natural (L.Groups.Length) = 2, "camera metadata, intrinsics or the echo became reading groups");
      Check (To_String (L.Groups (1).Command_Key) = "arm" and then To_String (L.Groups (1).Echo_Path) = "echo/arm",
             "the twin key was not taken as a command key with its echo");
      Check (not Is_Commandable (L, 2), "a group without a twin became commandable while others echo");
      Check (L.Has_Instruction, "instruction not found");
      Parse (Doc, Root (Doc), L, 5, O);
      Check (O.Readings.Element (1) = [0.1, 0.2] and then O.Echoes.Element (1) = [0.1, 0.2], "readings or echo");
      Check (Driver.Images.Width (O.Images (1)) = 4, "image not parsed");
      Check (To_String (O.Instruction) = "go", "instruction text");
   end Layout_By_Shape;

   procedure Hold_Rules is
      use Driver.Observations;
      use Ada.Strings.Unbounded;
      L : Layout;
      O : Observation;
      S : Driver.Replies.State;
      B : Driver.Bytes.Buffer;
      C : Driver.Commands.Command;
      Sent : Driver.Commands.Command;
   begin
      L.Groups.Append (Group_Info'(Path => To_Unbounded_String ("a"), Keys => To_Unbounded_String ("a"), Size => 2,
                                   Command_Key => To_Unbounded_String ("a"), others => <>));
      L.Groups.Append (Group_Info'(Path => To_Unbounded_String ("b"), Keys => To_Unbounded_String ("b"), Size => 1,
                                   Command_Key => To_Unbounded_String ("b"), others => <>));
      O.Readings.Append (Real_Array'(1.0, 2.0));
      O.Readings.Append (Real_Array'(1 .. 0 => 0.0));
      O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
      O.Echoes.Append (Real_Array'(1 .. 0 => 0.0));
      Driver.Replies.Write_Action (S, L, O, C, B, Sent);
      Check (Driver.Commands.Target (Sent, 1) = [1.0, 2.0], "an uncommanded group did not hold its reading");
      Check (not Driver.Commands.Has_Target (Sent, 2), "a value was invented for a group with nothing known");
      Driver.Commands.Set_Target (C, 2, [0.5]);
      Driver.Replies.Write_Action (S, L, O, C, B, Sent);
      Driver.Replies.Write_Action (S, L, O, Driver.Commands.Hold, B, Sent);
      Check (Driver.Commands.Target (Sent, 2) = [0.5], "a commanded group did not hold its last target");
      Driver.Replies.New_Episode (S);
      Driver.Replies.Write_Action (S, L, O, Driver.Commands.Hold, B, Sent);
      Check (Driver.Commands.Target (Sent, 2) = [0.5], "with no reading the last value sent was not repeated");
   end Hold_Rules;

   procedure Json_Round_Trip is
      use Driver.Json;
      use Ada.Strings.Unbounded;
      Doc : Document;
      Ok  : Boolean;
      Why : Unbounded_String;
      X   : constant Real := 0.1 + 0.2;
   begin
      Parse ("{""a"": [1, -2.5e3, 1e-2], ""s"": ""xé😀\n"", ""t"": true, ""n"": null, ""x"": "
             & Number_Image (X) & "}", Doc, Ok, Why);
      Check (Ok, "valid JSON rejected: " & To_String (Why));
      Check (Number (Doc, Element (Doc, Lookup (Doc, Root (Doc), "a"), 2)) = -2500.0, "exponent without a point");
      Check (Number (Doc, Element (Doc, Lookup (Doc, Root (Doc), "a"), 3)) = 0.01, "1e-2");
      Check (Text (Doc, Lookup (Doc, Root (Doc), "s")) =
               "x" & Character'Val (16#C3#) & Character'Val (16#A9#) & Character'Val (16#F0#) & Character'Val (16#9F#)
               & Character'Val (16#98#) & Character'Val (16#80#) & ASCII.LF,
             "\u escapes and a surrogate pair not decoded to UTF-8");
      Check (Is_True (Doc, Lookup (Doc, Root (Doc), "t")), "true");
      Check (Number (Doc, Lookup (Doc, Root (Doc), "x")) = X, "a number does not read back to the same bits");
      Parse ("[1, 2] 3", Doc, Ok, Why);
      Check (not Ok, "text after the value accepted");
      Parse ("{""a"": nan}", Doc, Ok, Why);
      Check (not Ok, "an unknown word accepted");
      Check (Quote ("a""b" & ASCII.LF) = """a\""b\n""", "quoting");
   end Json_Round_Trip;

   procedure Unconfigured_Service is
      use Driver.Services;
      R : constant Reply := Call (Brain, "/v1/chat/completions", "{}");
   begin
      Check (not R.Ok and then Ada.Strings.Unbounded.Length (R.Why) > 0,
             "a call without an address pretended to succeed or gave no reason");
   end Unconfigured_Service;

   procedure Streaming_Error_Body is
      --  A service that answers a streamed request with an error and a JSON
      --  body: the caller must get the body, not an empty stream.
      use GNAT.Sockets;
      Error_Body : constant String := "{""error"":{""message"":""bad request""}}";

      task Server is
         entry Listening (Port : out Natural);
      end Server;

      task body Server is
         Listener : Socket_Type;
         Client   : Socket_Type;
         Address  : Sock_Addr_Type := (Family => Family_Inet, Addr => Loopback_Inet_Addr, Port => Any_Port);
         Buffer   : Ada.Streams.Stream_Element_Array (1 .. 4096);
         Last     : Ada.Streams.Stream_Element_Offset;
         Reply    : constant String :=
           "HTTP/1.1 400 Bad Request" & ASCII.CR & ASCII.LF & "Content-Type: application/json" & ASCII.CR & ASCII.LF
           & "Content-Length:" & Natural'Image (Error_Body'Length) & ASCII.CR & ASCII.LF & ASCII.CR & ASCII.LF
           & Error_Body;
      begin
         Create_Socket (Listener);
         Bind_Socket (Listener, Address);
         Listen_Socket (Listener);
         Address := Get_Socket_Name (Listener);
         accept Listening (Port : out Natural) do
            Port := Natural (Address.Port);
         end Listening;
         Accept_Socket (Listener, Client, Address);
         Receive_Socket (Client, Buffer, Last);   --  the request; its content does not matter here
         Send_Socket (Client, Driver.Bytes.To_Bytes (Reply), Last);
         Close_Socket (Client);
         Close_Socket (Listener);
      end Server;

      procedure Ignore (Chunk : String; Stop : out Boolean) is
         pragma Unreferenced (Chunk);
      begin
         Stop := False;
      end Ignore;

      Port : Natural;
   begin
      Server.Listening (Port);
      Driver.Services.Configure (Driver.Services.Brain, "127.0.0.1", Port);
      declare
         R : constant Driver.Services.Reply :=
           Driver.Services.Call_Streaming (Driver.Services.Brain, "/v1/chat/completions", "{}", Ignore'Access);
      begin
         Check (not R.Ok, "an error status passed as an answer");
         Check (Ada.Strings.Unbounded.To_String (R.Text) = Error_Body,
                "the error body was lost: """ & Ada.Strings.Unbounded.To_String (R.Text) & """");
      end;
      Driver.Services.Configure (Driver.Services.Brain, "", 0);
   end Streaming_Error_Body;

   procedure Beat_Window is
      Looked : Boolean := False with Atomic;

      procedure Look is
      begin
         Looked := True;
      end Look;

      task Decider;
      task body Decider is
      begin
         Driver.Beats.Within_A_Beat (Look'Access);
      end Decider;

      Took  : Boolean := False;
      Reply : Driver.Commands.Command;
      O     : Driver.Observations.Observation;
   begin
      --  A beat is taken only once the decider waits for it.
      while not Took loop
         Driver.Beats.Offer (0, O, Driver.Commands.Hold, Took);
         if not Took then
            delay 0.001;
         end if;
      end loop;
      Driver.Beats.Await (Reply);
      Check (Looked, "the procedure of a beat window did not run in it");
      Check (Driver.Commands.Is_Hold (Reply), "a beat taken only to look did not answer hold");
   end Beat_Window;

   procedure Released_Beat is
      --  A decider that fails while it holds a beat must not leave the main
      --  loop waiting for an answer that never comes.
      task Failing;
      task body Failing is
         Beat : Driver.Clock.Beat;
      begin
         Driver.Beats.Next (Beat);
         raise Program_Error;
      exception
         when others =>
            Driver.Beats.Release;
      end Failing;

      task Waiter is
         entry Answered (C : out Driver.Commands.Command);
      end Waiter;
      task body Waiter is
         Reply : Driver.Commands.Command;
      begin
         Driver.Beats.Await (Reply);
         accept Answered (C : out Driver.Commands.Command) do
            C := Reply;
         end Answered;
      end Waiter;

      Took  : Boolean := False;
      Reply : Driver.Commands.Command;
      O     : Driver.Observations.Observation;
   begin
      while not Took loop
         Driver.Beats.Offer (0, O, Driver.Commands.Hold, Took);
         if not Took then
            delay 0.001;
         end if;
      end loop;
      select
         Waiter.Answered (Reply);
         Check (Driver.Commands.Is_Hold (Reply), "a failed decider's beat was answered with a move");
      or
         delay 10.0;
         Check (False, "a failed decider's beat was never answered");
         abort Waiter;
      end select;
   end Released_Beat;

   procedure Person_Words is
      use Driver.Beats;
      Before : constant Natural := Words_Heard;
   begin
      Hear ("put the cup down");
      Check (Latest_Words = "put the cup down" and then Words_Heard = Before + 1, "new words were not heard");
      Hear ("put the cup down");
      Check (Words_Heard = Before + 1, "the same words counted as new");
      Hear ("");
      Check (Latest_Words = "put the cup down", "an observation without words erased the last ones");
      Hear ("now the box");
      Check (Latest_Words = "now the box" and then Words_Heard = Before + 2, "changed words were not noticed");
   end Person_Words;

   procedure Replayed_Services is
      use Driver.Services;
      R  : constant Reply := (Ok => True, Text => Ada.Strings.Unbounded.To_Unbounded_String ("answer"), others => <>);
      T1, T2, T3 : Ticket;
   begin
      Start_Replay ([Instrument => True, Brain => False]);
      --  A recorded reply answers the call with the same request, and only
      --  from the beat after the call.
      Replay_Beat (3);
      T1 := Submit (Instrument, "/match", "first", 3);
      T2 := Submit (Instrument, "/match", "second", 3);
      Replay_Reply (Instrument, "/match", "second", R);
      Check (Ready (T2) = False, "a replayed reply was ready on the beat of its call");
      Replay_Beat (4);
      Check (Ready (T2), "a replayed reply was not ready on the next beat");
      Check (not Ready (T1), "a reply answered a call with another request");
      Check (Ada.Strings.Unbounded.To_String (Collect (T2).Text) = "answer", "the recorded text was not delivered");
      --  A reply recorded before an identical call is submitted waits for it.
      Replay_Reply (Instrument, "/segment", "box", R);
      T3 := Submit (Instrument, "/segment", "box", 4);
      Check (not Ready (T3), "a waiting reply was ready on the beat of its call");
      Replay_Beat (5);
      Check (Ready (T3), "a reply recorded before its call was lost");
      --  A service without recorded replies is called live; without an
      --  address it answers at once that it cannot.
      T3 := Submit (Brain, "/chat", "hello", 5);
      Replay_Beat (6);
      Check (Ready (T3) and then not Collect (T3).Ok, "an unconfigured live service pretended to answer");
      End_Replay;
   end Replayed_Services;

   procedure Register is
   begin
      Driver.Tests.Register ("core.rotation", "Exp and Log disagree near 0 or pi", Rotation_Round_Trip'Access);
      Driver.Tests.Register ("core.quaternion", "quaternion conversion loses a rotation", Quaternion_Round_Trip'Access);
      Driver.Tests.Register ("core.rigid", "a rigid inverse does not undo the transform", Rigid_Inverse'Access);
      Driver.Tests.Register ("core.least_squares", "QR least squares wrong or blind to rank loss",
                             Least_Squares_Exact'Access);
      Driver.Tests.Register ("core.cholesky", "Cholesky accepts indefinite input or solves wrongly",
                             Cholesky_Solve'Access);
      Driver.Tests.Register ("core.stats", "robust statistics moved by a single outlier", Robust_Statistics'Access);
      Driver.Tests.Register ("core.significance", "the one significance rule misjudges a difference",
                             Significance'Access);
      Driver.Tests.Register ("core.point_significance",
                             "a separation of points alarms more often than the scalar rule, or a real one is missed",
                             Point_Significance'Access);
      Driver.Tests.Register ("core.clock", "record times overflow after a few seconds or lose nanoseconds",
                             Clock_Range'Access);
      Driver.Tests.Register ("core.buffer", "a byte buffer loses data when it grows or copies",
                             Buffer_Growth'Access);
      Driver.Tests.Register ("core.image", "pixels are addressed by the wrong column or row", Image_Access'Access);
      Driver.Tests.Register ("core.msgpack", "a value changes in an encode-decode round trip, or bad input decodes",
                             Msgpack_Round_Trip'Access);
      Driver.Tests.Register ("core.ndarray", "a numpy dtype or byte order is misread", Ndarray_Dtypes'Access);
      Driver.Tests.Register ("core.layout", "a leaf is recognized by value or name instead of by its shape",
                             Layout_By_Shape'Access);
      Driver.Tests.Register ("core.replies", "a held group sends an invented value or forgets its target",
                             Hold_Rules'Access);
      Driver.Tests.Register ("core.json", "JSON loses UTF-8, accepts junk, or a number changes its bits",
                             Json_Round_Trip'Access);
      Driver.Tests.Register ("core.services", "a call to an unconfigured service pretends to succeed",
                             Unconfigured_Service'Access);
      Driver.Tests.Register ("core.streaming_error", "an error reply to a streamed call loses its body",
                             Streaming_Error_Body'Access);
      Driver.Tests.Register ("core.beat_window", "a decider that only looks moves the robot or never runs",
                             Beat_Window'Access);
      Driver.Tests.Register ("core.released_beat", "a decider that fails holding a beat leaves the main loop waiting",
                             Released_Beat'Access);
      Driver.Tests.Register ("core.person_words", "new words are missed, or old ones counted again",
                             Person_Words'Access);
      Driver.Tests.Register ("core.replayed_services",
                             "a replayed service reply arrives on another beat or answers another call",
                             Replayed_Services'Access);
   end Register;

end Driver.Core_Tests;
