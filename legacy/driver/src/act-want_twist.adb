separate (Act)
procedure Want_Twist (C : in out Context; F : Plug.Frame; W : Want; Arm : Integer; M : out Contact.Twist; Ok : out Boolean; Note : out Unbounded_String) is
   use Sinew;
   K : Contact.Qty.Kind := Contact.Qty.Height;
   Dir : Integer := W.Dir;
   Known : Boolean := True;
   Sc : Contact.Qty.Scene;
begin
   case W.Rel is
      when Re_Qty => Known := Qty_Kind (To_String (W.Qty), K);
      when Re_Nearer | Re_Touching => K := Contact.Qty.Gap; Dir := -1;
      when Re_Farther | Re_Clear => K := Contact.Qty.Gap; Dir := 1;
      when Re_Above => K := Contact.Qty.Rise; Dir := 1;
      when Re_Below => K := Contact.Qty.Rise; Dir := -1;
      when Re_Right => K := Contact.Qty.Across; Dir := 1;
      when Re_Left => K := Contact.Qty.Across; Dir := -1;
      when Re_Onto => K := Contact.Qty.Rest_On; Dir := 1;
      when Re_Off => K := Contact.Qty.Height; Dir := 1;
      when Re_Facing => K := Contact.Qty.Aim; Dir := 1;
      when others => Known := False;
   end case;
   M := Contact.Still ([others => 0.0]);
   if not Known then
      Ok := False;
      Note := S ((if W.Rel = Re_Qty then To_String (W.Qty) else Rel_Word (W.Rel)) & " is not a quantity I measure on it");
      return;
   end if;
   Want_Scene (C, F, W, Arm, Sc);
   Contact.Qty.Motion (K, Dir, Sc, M, Ok, Note);
end Want_Twist;
