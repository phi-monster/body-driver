separate (Act.Geo_Boot_Support)
procedure Touch_Tips_Together (Arms, Cams : Geom.Nat_Vectors.Vector) is
   N : constant Natural := Natural (Arms.Length);
   Seq0 : constant Natural := L.Seq;
   T0 : constant Ada.Calendar.Time := Ada.Calendar.Clock;
   task type Hand_Task (H, Hc : Natural) with Storage_Size => 64 * 1024 * 1024;
   task body Hand_Task is
   begin
      Lockstep.Begin_Hand (H);
      begin
         Touch_Tips (H, Hc);
      exception
         when E : others =>
            Put_Line ("[身] 📐 〔手" & Codec.Img (H + 1) & "〕碰指尖这一段出错 ⇒ 这只手这回没有指尖:" & Ada.Exceptions.Exception_Information (E));
      end;
      Lockstep.Done;
   end Hand_Task;
   type Hand_Ref is access Hand_Task;
   Hands : array (0 .. N - 1) of Hand_Ref;
   Ok : Boolean := True;
begin
   Geo_Say (Codec.Img (N) & " 只手同时碰桌面量指尖:每一拍一条关节命令带几只手的目标,各按各的步子走、各自判碰到(日志里〔手K〕是哪只手说的)");
   Lockstep.Clear;
   Plug.Lock_Begin;
   for I in 0 .. N - 1 loop
      Hands (I) := new Hand_Task (Arms (I), Cams (I));
      Lockstep.Start (Arms (I), Hands (I).all'Identity);
   end loop;
   loop
      declare
         All_Done : Boolean := True;
      begin
         for I in 0 .. N - 1 loop
            Lockstep.Run (Arms (I));
            if not Lockstep.Finished (Arms (I)) then
               All_Done := False;
            end if;
         end loop;
         exit when All_Done;
      end;
      Plug.Lock_Beat (L, F, Ok);
   end loop;
   Plug.Lock_End;
   Lockstep.Clear;
   Geo_Say (Codec.Img (N) & " 只手同时碰完:" & Codec.Img (L.Seq - Seq0) & " 拍、" & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0)), 1) & " 秒");
end Touch_Tips_Together;
