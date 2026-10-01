with Driver.Bytes;

package body Driver.Observations is

   use Driver.Msgpack;

   type Leaf is record
      Keys, Shown, Parent, Last_Key : Unbounded_String;
      N                             : Node := No_Node;
   end record;

   package Leaf_Vectors is new Ada.Containers.Vectors (Positive, Leaf);

   procedure Walk
     (Doc    : Document;
      N      : Node;
      Keys   : Unbounded_String;
      Shown  : Unbounded_String;
      Parent : Unbounded_String;
      Last   : Unbounded_String;
      Leaves : in out Leaf_Vectors.Vector) is
   begin
      if Kind_Of (Doc, N) = Map_Value and then not Is_Ndarray (Doc, N) then
         for I in 1 .. Count (Doc, N) loop
            declare
               K : constant String := Text (Doc, Key (Doc, N, I));
            begin
               Walk (Doc, Value (Doc, N, I),
                     (if Length (Keys) = 0 then To_Unbounded_String (K) else Keys & Key_Separator & K),
                     (if Length (Shown) = 0 then To_Unbounded_String (K) else Shown & "/" & K),
                     Keys, To_Unbounded_String (K), Leaves);
            end;
         end loop;
      else
         Leaves.Append (Leaf'(Keys => Keys, Shown => Shown, Parent => Parent, Last_Key => Last, N => N));
      end if;
   end Walk;

   function Find (Doc : Document; Root : Node; Keys : Unbounded_String) return Node is
      Path  : constant String := To_String (Keys);
      Cur   : Node := Root;
      Start : Positive := Path'First;
   begin
      for I in Path'Range loop
         if Path (I) = Key_Separator then
            Cur := Lookup (Doc, Cur, Path (Start .. I - 1));
            Start := I + 1;
         end if;
      end loop;
      return Lookup (Doc, Cur, Path (Start .. Path'Last));
   end Find;

   function Image_Size (Doc : Document; N : Node; Width, Height : out Natural) return Boolean is
   begin
      Width := 0;
      Height := 0;
      if not Is_Byte_Image (Doc, N) then
         return False;
      end if;
      declare
         S : constant Natural_Array := Shape (Doc, N);
      begin
         if S'Length = 3 and then S (S'First + 2) = 3 and then S (S'First) > 0 and then S (S'First + 1) > 0 then
            Height := S (S'First);
            Width := S (S'First + 1);
            return True;
         end if;
      end;
      return False;
   end Image_Size;

   function Is_Float_Grid (Doc : Document; N : Node; Width, Height : out Natural) return Boolean is
   begin
      Width := 0;
      Height := 0;
      if not Is_Ndarray (Doc, N) or else not Is_Numeric (Doc, N) then
         return False;
      end if;
      declare
         T : constant String := Dtype (Doc, N);
         S : constant Natural_Array := Shape (Doc, N);
      begin
         if T'Length >= 2 and then T (T'Last - 1) = 'f' and then S'Length = 2 then
            Height := S (S'First);
            Width := S (S'First + 1);
            return True;
         end if;
      end;
      return False;
   end Is_Float_Grid;

   function Common_Prefix (A, B : String) return Natural is
      N : Natural := 0;
   begin
      while N < A'Length and then N < B'Length and then A (A'First + N) = B (B'First + N) loop
         N := N + 1;
      end loop;
      return N;
   end Common_Prefix;

   procedure Recognize (Doc : Document; Root : Node; L : out Layout; Ok : out Boolean) is
      Leaves : Leaf_Vectors.Vector;
      Camera_Parents : Path_Vectors.Vector;
      W, H   : Natural;
   begin
      L := (others => <>);
      Walk (Doc, Root, Null_Unbounded_String, Null_Unbounded_String, Null_Unbounded_String,
            Null_Unbounded_String, Leaves);

      for F of Leaves loop
         if Image_Size (Doc, F.N, W, H) then
            L.Cameras.Append (Camera_Info'(Path => F.Shown, Keys => F.Keys, Width => W, Height => H, others => <>));
            Camera_Parents.Append (F.Parent);
         end if;
      end loop;

      for F of Leaves loop
         declare
            Is_Camera : constant Boolean := Image_Size (Doc, F.N, W, H);
            Of_Camera : constant Boolean := (for some P of Camera_Parents => Length (P) > 0 and then P = F.Parent);
         begin
            if Is_Camera then
               null;
            elsif Length (F.Parent) = 0 and then To_String (F.Last_Key) = "instruction"
              and then Kind_Of (Doc, F.N) = String_Value
            then
               L.Has_Instruction := True;
            elsif Is_Float_Grid (Doc, F.N, W, H) then
               --  A float grid the size of a camera is that camera's depth, paired
               --  by the longest common key path; any other grid (intrinsics,
               --  extrinsics) is recognized and left unused.
               declare
                  Best : Camera_Id'Base := 0;
                  Best_Length : Integer := -1;
               begin
                  for C in L.Cameras.First_Index .. L.Cameras.Last_Index loop
                     if L.Cameras (C).Width = W and then L.Cameras (C).Height = H
                       and then Length (L.Cameras (C).Depth_Keys) = 0
                       and then Common_Prefix (To_String (L.Cameras (C).Keys), To_String (F.Keys)) > Best_Length
                     then
                        Best := C;
                        Best_Length := Common_Prefix (To_String (L.Cameras (C).Keys), To_String (F.Keys));
                     end if;
                  end loop;
                  if Best > 0 then
                     L.Cameras (Best).Depth_Path := F.Shown;
                     L.Cameras (Best).Depth_Keys := F.Keys;
                  else
                     L.Unused.Append (F.Shown);
                  end if;
               end;
            elsif not Of_Camera and then Is_Numeric (Doc, F.N) and then Shape (Doc, F.N)'Length <= 1
              and then Numbers (Doc, F.N)'Length > 0
            then
               L.Groups.Append (Group_Info'(Path => F.Shown, Keys => F.Keys, Size => Numbers (Doc, F.N)'Length,
                                 Command_Key => Null_Unbounded_String, others => <>));
            else
               L.Unused.Append (F.Shown);
            end if;
         end;
      end loop;

      --  A last key that appears twice is a command key: one group is the
      --  reading, the other the robot's echo of the command it received.
      declare
         Kept       : Group_Vectors.Vector;
         Any_Twin   : Boolean := False;
         Taken      : array (L.Groups.First_Index .. L.Groups.Last_Index) of Boolean := [others => False];

         function Last_Key (G : Group_Info) return String is
            P : constant String := To_String (G.Keys);
         begin
            for I in reverse P'Range loop
               if P (I) = Key_Separator then
                  return P (I + 1 .. P'Last);
               end if;
            end loop;
            return P;
         end Last_Key;
      begin
         for I in Taken'Range loop
            if not Taken (I) then
               declare
                  G : Group_Info := L.Groups (I);
               begin
                  Taken (I) := True;
                  for J in I + 1 .. Taken'Last loop
                     if not Taken (J) and then Last_Key (L.Groups (J)) = Last_Key (G)
                       and then L.Groups (J).Size = G.Size and then Length (G.Command_Key) = 0
                     then
                        Taken (J) := True;
                        G.Command_Key := To_Unbounded_String (Last_Key (G));
                        G.Echo_Path := L.Groups (J).Path;
                        G.Echo_Keys := L.Groups (J).Keys;
                        Any_Twin := True;
                     end if;
                  end loop;
                  Kept.Append (G);
               end;
            end if;
         end loop;
         --  A robot that echoes nothing: every group is tried as a command key.
         if not Any_Twin then
            for G of Kept loop
               G.Command_Key := To_Unbounded_String (Last_Key (G));
            end loop;
         end if;
         L.Groups := Kept;
      end;

      Ok := not L.Cameras.Is_Empty and then not L.Groups.Is_Empty;
   end Recognize;

   function Describe (L : Layout) return String is
      Result : Unbounded_String;
      LF     : constant Character := ASCII.LF;
   begin
      for C in L.Cameras.First_Index .. L.Cameras.Last_Index loop
         Append (Result, "camera" & Camera_Id'Image (C) & ": " & To_String (L.Cameras (C).Path)
                 & Positive'Image (L.Cameras (C).Width) & " x" & Positive'Image (L.Cameras (C).Height)
                 & (if Length (L.Cameras (C).Depth_Path) > 0
                    then ", depth " & To_String (L.Cameras (C).Depth_Path) else "") & LF);
      end loop;
      for G in L.Groups.First_Index .. L.Groups.Last_Index loop
         Append (Result, "group" & Group_Id'Image (G) & ": " & To_String (L.Groups (G).Path)
                 & "," & Positive'Image (L.Groups (G).Size) & " values"
                 & (if Length (L.Groups (G).Command_Key) > 0
                    then ", command key " & To_String (L.Groups (G).Command_Key) else ", not commandable")
                 & (if Length (L.Groups (G).Echo_Path) > 0
                    then ", echoed at " & To_String (L.Groups (G).Echo_Path) else "") & LF);
      end loop;
      for U of L.Unused loop
         Append (Result, "unused: " & To_String (U) & LF);
      end loop;
      if L.Has_Instruction then
         Append (Result, "instruction: present" & LF);
      end if;
      return To_String (Result);
   end Describe;

   procedure Parse
     (Doc  : Document;
      Root : Node;
      L    : Layout;
      Beat : Driver.Clock.Beat;
      O    : out Observation)
   is
      None : constant Real_Array (1 .. 0) := [others => 0.0];

      function Values (Keys : Unbounded_String; Size : Natural) return Real_Array is
         N : constant Node := (if Length (Keys) = 0 then No_Node else Find (Doc, Root, Keys));
      begin
         if N /= No_Node and then Is_Numeric (Doc, N) then
            declare
               V : constant Real_Array := Numbers (Doc, N);
            begin
               if V'Length = Size then
                  return V;
               end if;
            end;
         end if;
         return None;
      end Values;
   begin
      O := (Beat => Beat, Received => Driver.Clock.Seconds, others => <>);
      for C of L.Cameras loop
         declare
            N : constant Node := Find (Doc, Root, C.Keys);
            W, H : Natural;
            Img : Driver.Images.Image := Driver.Images.No_Image;

            procedure Take (Data : Driver.Bytes.Byte_Array) is
            begin
               Img := Driver.Images.Create (W, H, Data);
            end Take;
         begin
            if Image_Size (Doc, N, W, H) and then W = C.Width and then H = C.Height then
               Read_Binary (Doc, N, Take'Access);
            end if;
            O.Images.Append (Img);
            O.Depth.Append (Values (C.Depth_Keys, C.Width * C.Height));
         end;
      end loop;
      for G of L.Groups loop
         O.Readings.Append (Values (G.Keys, G.Size));
         O.Echoes.Append (Values (G.Echo_Keys, G.Size));
      end loop;
      if L.Has_Instruction then
         O.Instruction := To_Unbounded_String (Text (Doc, Lookup (Doc, Root, "instruction")));
      end if;
   end Parse;

end Driver.Observations;
