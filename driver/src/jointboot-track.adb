separate (Jointboot)
procedure Track (A : Natural; Prev, Qn : Floats) is
   W : Arm_World := St_Worlds (A);
   P : Pend_State := St_Pend (A);
   Tol : constant Long_Float := Tol_Of (W);
   Changed_End : Boolean := False;
   Jx : Integer;
   Hs : Boolean;
   Who : constant String := "第" & Codec.Img (A + 1) & " 只手";
begin
   for J in 0 .. Natural'Min (Natural (Qn.Length), Natural'Min (Natural (W.Got_Lo.Length), Natural (W.Got_Hi.Length))) - 1 loop
      W.Got_Lo.Replace_Element (J, Long_Float'Min (W.Got_Lo (J), Qn (J)));
      W.Got_Hi.Replace_Element (J, Long_Float'Max (W.Got_Hi (J), Qn (J)));
   end loop;
   while End_Passed (Qn, W.Lo, W.Hi, Tol, Jx, Hs) loop
      Say (Who & "第" & Codec.Img (Jx) & " 个关节读数 " & Codec.Fmt (Qn (Jx), 4) & " 越过了记下的" & (if Hs then "正" else "负") & "那一头 "
           & Codec.Fmt ((if Hs then W.Hi (Jx) else W.Lo (Jx)), 4) & " ⇒ 那个尽头记错了,删掉");
      if Hs then
         W.Hi.Replace_Element (Jx, Long_Float'Last);
      else
         W.Lo.Replace_Element (Jx, Long_Float'First);
      end if;
      Changed_End := True;
   end loop;
   if P.Live then
      declare
         Mv, Big : Long_Float := 0.0;
      begin
         for J in 0 .. Natural'Min (Natural (Qn.Length), Natural (Prev.Length)) - 1 loop
            Mv := Long_Float'Max (Mv, abs (Qn (J) - Prev (J)));
         end loop;
         for J in 0 .. Natural'Min (Natural (P.Q_Cmd.Length), Natural (P.Q_At.Length)) - 1 loop
            Big := Long_Float'Max (Big, abs (P.Q_Cmd (J) - P.Q_At (J)));
         end loop;
         P.Still := (if Mv <= Long_Float'Max (St_Noise, Still_Frac * Big) then P.Still + 1 else 0);
         if P.Still >= 2 then
            P.Live := False;
            case Judge_End (P.Q_Cmd, P.Q_At, Qn, P.Glo, P.Ghi, W.Step_Lo, W.Step_Hi, Tol, Jx, Hs) is
               when End_Hit =>
                  Say (Who & "第" & Codec.Img (Jx) & " 个关节往" & (if Hs then "正" else "负") & "走:要到 " & Codec.Fmt (P.Q_Cmd (Jx), 4) & "(到过的范围只到 "
                       & Codec.Fmt ((if Hs then P.Ghi (Jx) else P.Glo (Jx)), 4) & "),停在 " & Codec.Fmt (Qn (Jx), 4) & ",往外走的不到一半,别的关节都到了"
                       & " ⇒ 这一头到了,记下(以后反解不往那边算;哪天真走过去了再删)");
                  if Hs then
                     W.Hi.Replace_Element (Jx, Qn (Jx));
                  else
                     W.Lo.Replace_Element (Jx, Qn (Jx));
                  end if;
                  Changed_End := True;
               when Blocked =>
                  if Jx >= 0 then
                     Say (Who & "第" & Codec.Img (Jx) & " 个关节往范围外走不到一半,可别的关节也没到 / 被顶偏了 ⇒ 是手碰上东西了,不是关节到头,不记");
                  end if;
               when Ambiguous =>
                  Say (Who & "好几个关节往范围外都走不到一半 ⇒ 分不清是哪一个到头了,不记");
               when Reached =>
                  null;
            end case;
         end if;
      end;
   end if;
   St_Worlds.Replace_Element (A, W);
   St_Pend.Replace_Element (A, P);
   Save_If_Grown (Changed_End);
end Track;
