separate (Jointboot)
procedure Note_Command (A : Natural; Q, Full : Floats; Clamped : Boolean) is
   W : constant Arm_World := St_Worlds (A);
   P : Pend_State := St_Pend (A);
   Tol : constant Long_Float := Tol_Of (W);
   Beyond : Boolean := False;
begin
   for J in 0 .. Natural'Min (Natural (Q.Length), Natural (W.Got_Hi.Length)) - 1 loop
      if Q (J) > W.Got_Hi (J) + Tol or else Q (J) < W.Got_Lo (J) - Tol then
         Beyond := True;
      end if;
   end loop;
   --  只在"开始被夹住"那一刻说一句(之后跟着往前重发的都还是它,不重复说):夹得最多的那个关节要到哪、这一条只到哪
   if Clamped and then not P.Held_Back then
      declare
         Jm : Natural := 0;
         Dm : Long_Float := -1.0;
      begin
         for J in 0 .. Natural'Min (Natural (Q.Length), Natural (Full.Length)) - 1 loop
            if abs (Full (J) - Q (J)) > Dm then
               Dm := abs (Full (J) - Q (J)); Jm := J;
            end if;
         end loop;
         Say ("第" & Codec.Img (A + 1) & " 只手:第 " & Codec.Img (Jm) & " 个关节要到 " & Codec.Fmt (Full (Jm), 4) & ",这一条只发到 " & Codec.Fmt (Q (Jm), 4)
              & "(到过的范围 + 往外一步)⇒ 手走过去、范围长了就跟着往前重发(到过的范围)");
      end;
   end if;
   P.Held_Back := Clamped;
   P.Cmd_Glo := W.Got_Lo; P.Cmd_Ghi := W.Got_Hi;
   --  上一条还没核就来了新的 = 被打断了,不核(到过的范围原话)
   P.Live := Beyond;
   if Beyond then
      P.Q_Cmd := Q; P.Q_At := St_Last (A); P.Glo := W.Got_Lo; P.Ghi := W.Got_Hi; P.Still := 0;
   end if;
   St_Pend.Replace_Element (A, P);
end Note_Command;
