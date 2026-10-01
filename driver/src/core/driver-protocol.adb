package body Driver.Protocol is

   use Driver.Msgpack;

   Echoed : constant array (Positive range <>) of Unbounded_String :=
     [To_Unbounded_String ("message_id"), To_Unbounded_String ("evaluation_id"),
      To_Unbounded_String ("action_case_id"), To_Unbounded_String ("trial_id"),
      To_Unbounded_String ("repeat_index"), To_Unbounded_String ("sent_at")];
   --  Optional identifiers returned unchanged when the request carries them.

   procedure Decode (Data : Driver.Bytes.Byte_Array; R : out Request; Ok : out Boolean) is
      Top, Payload : Node;
   begin
      R := (others => <>);
      Driver.Msgpack.Decode (Data, R.Doc, Ok);
      if not Ok then
         return;
      end if;
      Top := Root (R.Doc);
      Ok := Kind_Of (R.Doc, Top) = Map_Value;
      if not Ok then
         return;
      end if;
      declare
         T : constant String := Text (R.Doc, Lookup (R.Doc, Top, "message_type"));
      begin
         R.Kind := (if T = "hello" then Hello
                    elsif T = "prepare_case" then Prepare_Case
                    elsif T = "reset" then Reset
                    elsif T = "call" then Call
                    elsif T = "infer" then Infer
                    elsif T = "trial_end" then Trial_End
                    elsif T = "heartbeat" then Heartbeat
                    else Unknown);
      end;
      Payload := Lookup (R.Doc, Top, "payload");
      R.Function_Name := To_Unbounded_String (Text (R.Doc, Lookup (R.Doc, Payload, "func_name")));
      R.Observation := Lookup (R.Doc, Payload, "obs");
      if R.Observation = No_Node then
         R.Observation := Lookup (R.Doc, Payload, "observation");
      end if;
      if Kind_Of (R.Doc, R.Observation) /= Map_Value then
         R.Observation := No_Node;
      end if;
   end Decode;

   function Reply_Type (K : Message_Kind) return String is
     (case K is
         when Hello        => "hello_ack",
         when Prepare_Case => "prepare_case_ack",
         when Reset        => "reset_result",
         when Call         => "call_result",
         when Infer        => "infer_result",
         when Trial_End    => "trial_end_ack",
         when Heartbeat    => "heartbeat_ack",
         when Unknown      => "error");

   function Is_Echoed (Field : String) return Boolean is
     (Field = "step" or else (for some Name of Echoed => To_String (Name) = Field));

   procedure Encode_Reply
     (R          : Request;
      Has_Action : Boolean;
      Action     : Driver.Bytes.Byte_Array;
      Reply      : out Driver.Bytes.Buffer)
   is
      Top    : constant Node := Root (R.Doc);
      Pairs  : Natural := 3;   --  message_type, step, payload
      Step   : constant Node := Lookup (R.Doc, Top, "step");
   begin
      Reply.Clear;
      for Name of Echoed loop
         if Lookup (R.Doc, Top, To_String (Name)) /= No_Node then
            Pairs := Pairs + 1;
         end if;
      end loop;
      Put_Map_Header (Reply, Pairs);
      Put_String (Reply, "message_type");
      Put_String (Reply, Reply_Type (R.Kind));
      for Name of Echoed loop
         declare
            V : constant Node := Lookup (R.Doc, Top, To_String (Name));
         begin
            if V /= No_Node then
               Put_String (Reply, To_String (Name));
               Put_Node (Reply, R.Doc, V);
            end if;
         end;
      end loop;
      Put_String (Reply, "step");
      if Step /= No_Node then
         Put_Node (Reply, R.Doc, Step);
      else
         Put_Integer (Reply, 0);
      end if;
      Put_String (Reply, "payload");
      if Wants_Action (R) then
         Put_Map_Header (Reply, 1);
         Put_String (Reply, "result");
         if Has_Action then
            Put_Array_Header (Reply, 1);
            Reply.Append (Action);
         else
            Put_Array_Header (Reply, 0);
         end if;
      elsif R.Kind = Hello then
         Put_Map_Header (Reply, 3);
         Put_String (Reply, "ok");
         Put_Boolean (Reply, True);
         Put_String (Reply, "server");
         Put_String (Reply, "xpolicylab_policy_server");
         Put_String (Reply, "server_instance_id");
         Put_String (Reply, "body-driver");
      else
         Put_Map_Header (Reply, 1);
         Put_String (Reply, "ok");
         Put_Boolean (Reply, R.Kind /= Unknown);
      end if;
   end Encode_Reply;

end Driver.Protocol;
