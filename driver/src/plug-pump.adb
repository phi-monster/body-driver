separate (Plug)
function Pump (L : in out Link) return Boolean is
   Kind : Websocket.Op;
   Data : Buf;
   Ok : Boolean;
   Idle : Natural := 0;
begin
   loop
      Websocket.Read_Message (L.Conn, Kind, Data, Ok);
      if not Ok or else Kind = Websocket.Op_Close then
         Put_Line ("[链] 线断了 ⇒ 在同一个口上等对方重新接上(身体量到的东西都留着)…");
         Websocket.Accept_Client (L.Conn, Ok);
         if not Ok then
            Put_Line ("[链] 没等到 ⇒ 取不到画面");
            return False;
         end if;
         --  攥着的命令和上一条命令都留着:断线重连不改变世界。上一条也不能丢 —— 丢了就退回"照现在报的位姿保持",而报的位姿落后一步,
         --  手臂正在走时等于把它拽回起点(EK:重连后第一步实到 0.000,就是这么被拽回去的)
         Put_Line ("[链] 重新接上了(不当作新的一集:对方明说 reset 才算;攥着的命令照发)");
      elsif Kind = Websocket.Op_Binary then
         declare
            D : Doc;
         begin
            if Decode (Data, D) then
               declare
                  MT : constant String := Text (D, Key (D, 0, "message_type"));
                  P : constant Integer := Key (D, 0, "payload");
                  Fn : constant String := Text (D, Key (D, P, "func_name"));
                  Obs : Integer := Key (D, P, "obs");
                  Ack : constant String :=
                    (if MT = "hello" then "hello_ack"
                     elsif MT = "prepare_case" then "prepare_case_ack"
                     elsif MT = "reset" then "reset_result"
                     elsif MT = "call" then "call_result"
                     elsif MT = "infer" then "infer_result"
                     elsif MT = "trial_end" then "trial_end_ack"
                     elsif MT = "heartbeat" then "heartbeat_ack"
                     else "");
                  New_Frame : Boolean := False;
                  Payload : Buf;
               begin
                  Idle := Idle + 1;
                  if Idle mod 1000 = 0 then
                     Put_Line ("[链] 对方连发" & Natural'Image (Idle) & " 条没带画面的消息,线还通,继续等");
                  end if;
                  if MT = "reset" then
                     L.Reset_Flag := True;
                     L.Ep_Seq0 := L.Seq;   --  新的一集从零数拍
                     L.Jaw_Set.Clear;      --  新的一集爪子回到对方的初始状态,上一集给过的目标作废
                     L.Jaw_Sent.Clear;     --  上一集发出去的那一串也作废(没读数的那一拍不许把上一集的数发进新的一集)
                  end if;
                  if Ack /= "" then
                     if Obs < 0 then
                        Obs := Key (D, P, "observation");
                     end if;
                     if Obs >= 0 then
                        L.Last := D;
                        L.Last_Obs := Obs;
                        New_Frame := True;
                     end if;
                     if Fn = "get_action" then
                        declare
                           Action : Buf;
                        begin
                           if L.Has_Pending then
                              Action := L.Pending;
                              L.Last_Sent := L.Pending;
                              L.Has_Last := True;
                              L.Has_Pending := False;
                           elsif L.Has_Last then
                              Action := L.Last_Sent;
                           else
                              Action := Hold_Action (L);
                           end if;
                           Put_Map (Payload, 1);
                           Put_Str (Payload, "result");
                           if Action.Is_Empty then
                              Put_Array (Payload, 0);
                           else
                              Put_Array (Payload, 1);
                              for B of Action loop
                                 Payload.Append (B);
                              end loop;
                           end if;
                        end;
                     elsif Ack = "hello_ack" then
                        Put_Map (Payload, 3);
                        Put_Str (Payload, "ok"); Put_Bool (Payload, True);
                        Put_Str (Payload, "server"); Put_Str (Payload, "xpolicylab_policy_server");
                        Put_Str (Payload, "server_instance_id"); Put_Str (Payload, "body-driver");
                     else
                        Put_Map (Payload, 1);
                        Put_Str (Payload, "ok"); Put_Bool (Payload, True);
                     end if;
                     Reply (L, D, Ack, Payload);
                     if New_Frame then
                        return True;
                     end if;
                  end if;
               end;
            end if;
         end;
      end if;
   end loop;
end Pump;
