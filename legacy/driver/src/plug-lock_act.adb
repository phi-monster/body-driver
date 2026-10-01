separate (Plug)
function Lock_Act (C : Cmd) return Boolean is
   Cj : Cmd := C;
   Ok : Boolean := True;
   procedure Put_Q (G : Natural; Q : Floats) is
   begin
      while Natural (Lock_Q.Length) <= G loop
         Lock_Q.Append (F64_Vectors.Empty_Vector);
      end loop;
      Lock_Q.Replace_Element (G, Q);
   end Put_Q;
begin
   if C.Kind = Hold then
      return True;
   end if;
   if C.Kind = Ee then
      if Hook_C = null then
         return False;   --  没有运动学:位姿命令解不成关节(按拍对齐只合关节动作)
      end if;
      Hook_C (Cj, Ok);
      if not Ok then
         return False;
      end if;
   end if;
   if Cj.Kind /= Joint then
      return False;
   end if;
   if not Cj.Groups.Is_Empty then
      for K in 0 .. Natural'Min (Natural (Cj.Groups.Length), Natural (Cj.Qs.Length)) - 1 loop
         if Cj.Groups (K) >= 0 then
            Put_Q (Natural (Cj.Groups (K)), Cj.Qs (K));
         end if;
      end loop;
   elsif Cj.Group >= 0 then
      Put_Q (Natural (Cj.Group), Cj.Q);
   else
      Put_Q (Cj.Arm, Cj.Q);   --  只报关节的身体:按臂
   end if;
   if not C.Jaw.Is_Empty then
      while Natural (Lock_Jaw.Length) <= C.Arm loop
         Lock_Jaw.Append (F64_Vectors.Empty_Vector);
      end loop;
      Lock_Jaw.Replace_Element (C.Arm, C.Jaw);
   end if;
   Lock_Changed := True;
   return True;
end Lock_Act;
