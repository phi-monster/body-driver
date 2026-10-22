--  A study, not the driver: where the fake's truth puts the points of one of an arm's keyframes, to judge what the
--  aligner answered against.
--
--  truth_points SEED TRUTH CAMERA DUMP
--
--  DUMP is a keyframe's points as the driver wrote them (its first line: "beat B reference R ..."; then a line per
--  point: its index and its place in the reference picture, then anything). The fake world of SEED (the template of
--  the servo runs of body 3: two jaws, arm 2 dead, servos of 4096 ticks) is posed at the joints the TRUTH file gives
--  for beat R; the ray through each point from CAMERA meets a solid; that solid's point, carried to beat B, is seen by
--  CAMERA there unless it is behind it, outside its picture or behind something nearer. One line a point: its index,
--  its reference place, and its true place, or "none" and why; then the solid it lies on.

with Ada.Command_Line;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver;
with Driver.Json;
with Driver.Numerics;
with Fake_Render;
with Fake_World;

procedure Truth_Points is
   use Ada.Strings.Unbounded;
   use Ada.Text_IO;
   use Driver;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Fake_World;
   use type Driver.Json.Node;

   Seed       : constant Natural := Natural'Value (Ada.Command_Line.Argument (1));
   Truth_Path : constant String := Ada.Command_Line.Argument (2);
   Cam_Name   : constant String := Ada.Command_Line.Argument (3);
   Dump_Path  : constant String := Ada.Command_Line.Argument (4);
   T          : constant Template := (Hands => [Jaw, Jaw], Ticks => 4096, Servo => True, others => <>);
   W          : World;
   Cam        : Natural := 0;

   --  The truth's line of a beat, "" when it has none.
   function Line_Of (Beat : Natural) return String is
      F   : File_Type;
      Key : constant String := """call"":" & Ada.Strings.Fixed.Trim (Beat'Image, Ada.Strings.Left) & ",";
   begin
      Open (F, In_File, Truth_Path);
      while not End_Of_File (F) loop
         declare
            L : constant String := Get_Line (F);
         begin
            if Ada.Strings.Fixed.Index (L, Key) > 0 then
               Close (F);
               return L;
            end if;
         end;
      end loop;
      Close (F);
      return "";
   end Line_Of;

   --  The world posed at the joints the truth gives for the beat.
   procedure Pose_At (Beat : Natural; Ok : out Boolean) is
      L      : constant String := Line_Of (Beat);
      Doc    : Driver.Json.Document;
      Parsed : Boolean;
      Why    : Unbounded_String;
   begin
      Ok := False;
      if L'Length = 0 then
         return;
      end if;
      Driver.Json.Parse (L, Doc, Parsed, Why);
      if not Parsed then
         return;
      end if;
      declare
         State : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Driver.Json.Root (Doc), "state");
      begin
         for G of W.Groups loop
            declare
               A : constant Driver.Json.Node := Driver.Json.Lookup (Doc, State, To_String (G.Name));
            begin
               if A /= Driver.Json.No_Node and then Driver.Json.Count (Doc, A) = Natural (G.Joints.Length) then
                  for K in 1 .. Natural (G.Joints.Length) loop
                     W.Links (G.Joints (G.Joints.First_Index + K - 1)).Q :=
                       Driver.Json.Number (Doc, Driver.Json.Element (Doc, A, K));
                  end loop;
               end if;
            end;
         end loop;
      end;
      Forward (W);
      Ok := True;
   end Pose_At;

   type Point is record
      Index : Natural := 0;
      U, V  : Real := 0.0;
      Solid : Natural := 0;
      Local : Vec3 := [0.0, 0.0, 0.0];
   end record;
   type Point_Array is array (Positive range <>) of Point;
   Points : Point_Array (1 .. 4096);
   Count  : Natural := 0;
   Beat, Reference : Natural := 0;
   Ok : Boolean;
begin
   Build (W, Seed, 320, 240, T);
   for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
      if To_String (W.Cameras (C).Name) = Cam_Name then
         Cam := C;
      end if;
   end loop;
   if Cam = 0 then
      Put_Line ("no camera " & Cam_Name);
      return;
   end if;
   declare
      F : File_Type;
   begin
      Open (F, In_File, Dump_Path);
      declare
         Head : constant String := Get_Line (F);
         B    : constant Natural := Ada.Strings.Fixed.Index (Head, "beat ");
         R    : constant Natural := Ada.Strings.Fixed.Index (Head, " reference ");
         E    : constant Natural := Ada.Strings.Fixed.Index (Head, " eye ");
      begin
         Beat := Natural'Value (Head (B + 5 .. R - 1));
         Reference := Natural'Value (Head (R + 11 .. E - 1));
      end;
      while not End_Of_File (F) loop
         declare
            L     : constant String := Ada.Strings.Fixed.Trim (Get_Line (F), Ada.Strings.Both);
            S1    : constant Natural := Ada.Strings.Fixed.Index (L, " ");
            S2    : constant Natural := Ada.Strings.Fixed.Index (L (S1 + 1 .. L'Last), " ");
            S3    : constant Natural := Ada.Strings.Fixed.Index (L (S2 + 1 .. L'Last), " ");
         begin
            Count := Count + 1;
            Points (Count) := (Index => Natural'Value (L (L'First .. S1 - 1)),
                               U => Real'Value (L (S1 + 1 .. S2 - 1)), V => Real'Value (L (S2 + 1 .. S3 - 1)),
                               others => <>);
         end;
      end loop;
      Close (F);
   end;
   Pose_At (Reference, Ok);
   if not Ok then
      Put_Line ("no truth at beat" & Reference'Image);
      return;
   end if;
   for P of Points (1 .. Count) loop
      declare
         Along : Real;
      begin
         Fake_Render.Nearest (W, W.Cameras (Cam), P.U, P.V, P.Solid, Along);
         if P.Solid > 0 then
            declare
               C  : Camera renames W.Cameras (Cam);
               Px : constant Real := (P.U - C.Cx) / C.Fx;
               Py : constant Real := (P.V - C.Cy) / C.Fy;
               D  : constant Vec3 := C.Pose.Rotation * Vec3'[Px, Py, 1.0];
               X  : constant Vec3 := C.Pose.Translation + (Along / (abs D)) * D;
            begin
               P.Local := Inverse (W.Solids (P.Solid).Pose) * X;
            end;
         end if;
      end;
   end loop;
   Pose_At (Beat, Ok);
   if not Ok then
      Put_Line ("no truth at beat" & Beat'Image);
      return;
   end if;
   Put_Line ("truth of beat" & Beat'Image & " against reference" & Reference'Image & ", " & Cam_Name);
   for P of Points (1 .. Count) loop
      declare
         C : Camera renames W.Cameras (Cam);
      begin
         if P.Solid = 0 then
            Put_Line (P.Index'Image & " none sky");
         else
            declare
               X : constant Vec3 := W.Solids (P.Solid).Pose * P.Local;
               Y : constant Vec3 := Inverse (C.Pose) * X;
            begin
               if Y (3) <= 0.0 then
                  Put_Line (P.Index'Image & " none behind " & To_String (W.Solids (P.Solid).Name));
               else
                  declare
                     U  : constant Real := C.Fx * Y (1) / Y (3) + C.Cx;
                     V  : constant Real := C.Fy * Y (2) / Y (3) + C.Cy;
                     S  : Natural;
                     At_Distance : Real;
                  begin
                     if U < 0.0 or else V < 0.0 or else U > Real (C.Width) or else V > Real (C.Height) then
                        Put_Line (P.Index'Image & " none outside " & To_String (W.Solids (P.Solid).Name));
                     else
                        Fake_Render.Nearest (W, C, U, V, S, At_Distance);
                        Put_Line (P.Index'Image & Real'Image (U) & Real'Image (V)
                                  & (if S /= P.Solid and then At_Distance < (abs (X - C.Pose.Translation)) * (1.0 - 1.0E-6)
                                     then " hidden " else " seen ")
                                  & To_String (W.Solids (P.Solid).Name) & Real'Image ((abs (X - C.Pose.Translation))));
                     end if;
                  end;
               end if;
            end;
         end if;
      end;
   end loop;
end Truth_Points;
