with Ada.Strings.Unbounded;

package body Driver.Replies is

   use Ada.Strings.Unbounded;
   use Driver.Observations;

   procedure New_Episode (S : in out State) is
   begin
      S.Targets.Clear;
      S.Held.Clear;
   end New_Episode;

   procedure Write_Action
     (S    : in out State;
      L    : Layout;
      O    : Observation;
      C    : Driver.Commands.Command;
      B    : in out Driver.Bytes.Buffer;
      Sent : out Driver.Commands.Command)
   is
      Count : Natural := 0;

      --  What group G carries this beat; empty when nothing is known for it. A group never commanded this
      --  episode holds at the reading of the beat it was first held at, kept (Held) until a target comes: sent
      --  its reading of every beat, as it was, a body that gives under its load was told each beat to stand
      --  where it had sagged to, sagged from there again, and crept without end (fake body B0F1, 10-13: no
      --  command but holds, arm 2's fingers rose 72 cm over 8000 beats; the readings' creep locked in the
      --  same way the protocol already refused for what closes a hand).
      function Value_Of (G : Group_Id) return Real_Array is
      begin
         if Driver.Commands.Has_Target (C, G) then
            return Driver.Commands.Target (C, G);
         elsif S.Targets.Contains (G) then
            return S.Targets.Element (G);
         elsif S.Held.Contains (G) then
            return S.Held.Element (G);
         elsif Has_Reading (O, G) then
            return O.Readings.Element (G);
         elsif S.Last_Sent.Contains (G) then
            return S.Last_Sent.Element (G);
         end if;
         return [1 .. 0 => 0.0];
      end Value_Of;
   begin
      Sent := Driver.Commands.Hold;
      for G in L.Groups.First_Index .. L.Groups.Last_Index loop
         if Is_Commandable (L, G) and then Value_Of (G)'Length = L.Groups (G).Size then
            Count := Count + 1;
         end if;
      end loop;
      Driver.Msgpack.Put_Map_Header (B, Count);
      for G in L.Groups.First_Index .. L.Groups.Last_Index loop
         if Is_Commandable (L, G) then
            declare
               V : constant Real_Array := Value_Of (G);
            begin
               if V'Length = L.Groups (G).Size then
                  Driver.Msgpack.Put_String (B, To_String (L.Groups (G).Command_Key));
                  Driver.Msgpack.Put_Array_Header (B, V'Length);
                  for X of V loop
                     Driver.Msgpack.Put_Float (B, X);
                  end loop;
                  Driver.Commands.Set_Target (Sent, G, V);
                  S.Last_Sent.Include (G, V);
                  if Driver.Commands.Has_Target (C, G) then
                     S.Targets.Include (G, V);
                  elsif not S.Targets.Contains (G) and then not S.Held.Contains (G) then
                     S.Held.Include (G, V);
                  end if;
               end if;
            end;
         end if;
      end loop;
   end Write_Action;

   function Read_Action
     (L      : Layout;
      Doc    : Driver.Msgpack.Document;
      Action : Driver.Msgpack.Node) return Driver.Commands.Command
   is
      use Driver.Msgpack;
      C : Driver.Commands.Command := Driver.Commands.Hold;
   begin
      for G in L.Groups.First_Index .. L.Groups.Last_Index loop
         if Is_Commandable (L, G) then
            declare
               V : constant Node := Lookup (Doc, Action, To_String (L.Groups (G).Command_Key));
            begin
               if V /= No_Node and then Is_Numeric (Doc, V) and then Numbers (Doc, V)'Length = L.Groups (G).Size then
                  Driver.Commands.Set_Target (C, G, Numbers (Doc, V));
               end if;
            end;
         end if;
      end loop;
      return C;
   end Read_Action;

end Driver.Replies;
