separate (Plug)
procedure Boot (Port : Natural; L : in out Link; Ok : out Boolean) is
   Tries : Natural := 0;
begin
   Ok := False;
   Put_Line ("[装] 在" & Natural'Image (Port) & " 上等这台机器人连过来…");
   Websocket.Listen (Port, L.Conn, Ok);
   if not Ok then
      return;
   end if;
   Put_Line ("[装] 接上了。先听一帧,认这台机器人报的东西长什么样。");
   loop
      if not Pump (L) then
         Ok := False;
         return;
      end if;
      Layout.Recognise (L.Last, L.Last_Obs, L.Lay);
      declare
         M : constant String := Layout.Missing (L.Lay);
      begin
         if M = "" then
            L.Have_Layout := True;
            Layout.Say (L.Lay);
            Ok := True;
            return;
         end if;
         Tries := Tries + 1;
         if Tries <= 3 then
            Put_Line ("[装] 拿到观测了,但认不出来 ⇒ " & M);
            Layout.Say (L.Lay);
         end if;
         if Tries > 4000 then
            Ok := False;
            return;
         end if;
      end;
   end loop;
end Boot;
