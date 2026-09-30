separate (Plug)
procedure Reply (L : in out Link; Req : Doc; Kind : String; Payload : Buf) is
   S : Buf;
   N : Natural := 4;   --  message_type, message_id, step, payload
   Extras : Strs;
   Ok : Boolean;
begin
   Extras.Append ("evaluation_id"); Extras.Append ("action_case_id"); Extras.Append ("trial_id");
   Extras.Append ("repeat_index"); Extras.Append ("sent_at");
   for E of Extras loop
      if Key (Req, 0, E) >= 0 then
         N := N + 1;
      end if;
   end loop;
   Put_Map (S, N);
   Put_Str (S, "message_type"); Put_Str (S, Kind);
   Put_Str (S, "message_id"); Put_Node (S, Req, Key (Req, 0, "message_id"));
   for E of Extras loop
      declare
         K : constant Integer := Key (Req, 0, E);
      begin
         if K >= 0 then
            Put_Str (S, E); Put_Node (S, Req, K);
         end if;
      end;
   end loop;
   Put_Str (S, "step");
   declare
      K : constant Integer := Key (Req, 0, "step");
   begin
      if K >= 0 then
         Put_Node (S, Req, K);
      else
         Put_Int (S, 0);
      end if;
   end;
   Put_Str (S, "payload");
   for B of Payload loop
      S.Append (B);
   end loop;
   Websocket.Send_Binary (L.Conn, S, Ok);
end Reply;
