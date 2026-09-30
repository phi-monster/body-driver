separate (Plug)
function Jaw_Values (L : in out Link; Ji : Natural; Mine : Boolean; C : Cmd; Cur : Floats) return Floats is
   V : Floats;
begin
   while Natural (L.Jaw_Set.Length) <= Ji loop
      L.Jaw_Set.Append (F64_Vectors.Empty_Vector);
   end loop;
   while Natural (L.Jaw_Sent.Length) <= Ji loop
      L.Jaw_Sent.Append (F64_Vectors.Empty_Vector);
   end loop;
   if Mine then
      --  这条命令给了这一组前几个通道的目标:整段记下(连着一段、没有空位 ⇒ 不用编数去填前面没给的)
      declare
         S : Floats := L.Jaw_Set (Ji);
      begin
         for K in 0 .. Natural (C.Jaw.Length) - 1 loop
            if K < Natural (S.Length) then
               S.Replace_Element (K, C.Jaw (K));
            else
               S.Append (C.Jaw (K));
            end if;
         end loop;
         L.Jaw_Set.Replace_Element (Ji, S);
      end;
   end if;
   declare
      Given : constant Floats := L.Jaw_Set (Ji);
      Last : constant Floats := L.Jaw_Sent (Ji);
      N : constant Natural := (if Cur.Is_Empty then Natural (Last.Length) else Natural (Cur.Length));
   begin
      for K in 0 .. N - 1 loop
         V.Append (if K < Natural (Given.Length) then Given (K) elsif K < Natural (Cur.Length) then Cur (K) else Last (K));
      end loop;
   end;
   if not V.Is_Empty then
      L.Jaw_Sent.Replace_Element (Ji, V);
   end if;
   return V;
end Jaw_Values;
