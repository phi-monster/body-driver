separate (Jointboot)
procedure Save_If_Grown (Changed_End : Boolean) is
   Grown : Boolean := False;
begin
   if Length (St_Kin_Path) = 0 then
      return;
   end if;
   for A in 0 .. Natural (St_Worlds.Length) - 1 loop
      declare
         W : Arm_World renames St_Worlds (A);
      begin
         for J in 0 .. Natural'Min (Natural (W.Got_Hi.Length), Natural (St_Saved_Ghi (A).Length)) - 1 loop
            if (W.Step_Hi (J) > 0.0 and then W.Got_Hi (J) - St_Saved_Ghi (A) (J) >= W.Step_Hi (J))
              or else (W.Step_Lo (J) > 0.0 and then St_Saved_Glo (A) (J) - W.Got_Lo (J) >= W.Step_Lo (J))
            then
               Grown := True;
            end if;
         end loop;
      end;
   end loop;
   if not (Changed_End or else Grown) then
      return;
   end if;
   for I in 0 .. Natural (St_Worlds.Length) - 1 loop
      if St_Kin_Idx (I) < Natural (St_Kin.Worlds.Length) then
         declare
            Kw : Arm_World := St_Kin.Worlds (St_Kin_Idx (I));
         begin
            Kw.Lo := St_Worlds (I).Lo; Kw.Hi := St_Worlds (I).Hi;
            Kw.Got_Lo := St_Worlds (I).Got_Lo; Kw.Got_Hi := St_Worlds (I).Got_Hi;
            St_Kin.Worlds.Replace_Element (St_Kin_Idx (I), Kw);
         end;
      end if;
      St_Saved_Glo.Replace_Element (I, St_Worlds (I).Got_Lo); St_Saved_Ghi.Replace_Element (I, St_Worlds (I).Got_Hi);
   end loop;
   begin
      Save_Kin (To_String (St_Kin_Path), St_Kin, Images => False);
   exception
      when others =>
         Say ("到过的关节范围 / 尽头写不回 " & To_String (St_Kin_Path));
   end;
end Save_If_Grown;
