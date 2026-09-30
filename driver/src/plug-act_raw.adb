separate (Plug)
function Act_Raw (L : in out Link; C : Cmd) return Boolean is
   S : Buf;
   N : constant Natural := Arms (L);
begin
   if C.Kind = Hold then
      return True;
   end if;
   if L.Last_Obs < 0 or else N = 0 then
      return False;
   end if;
   if C.Kind = Base then
      if L.Lay.Base.Is_Empty then
         return False;
      end if;
      Put_Map (S, Natural (L.Lay.Base.Length));
      for I in 0 .. Natural (L.Lay.Base.Length) - 1 loop
         Put_Str (S, Layout.Last_Seg (L.Lay.Base (I)));
         if Natural (L.Lay.Base.Length) = 1 then
            Put_Array (S, Natural (C.V.Length));
            for X of C.V loop
               Put_Float (S, X);
            end loop;
         else
            Put_Float (S, (if I < Natural (C.V.Length) then C.V (I) else 0.0));
         end if;
      end loop;
      L.Pending := S; L.Has_Pending := True;
      return True;
   end if;
   --  身体也报位姿,但开机量胳膊要一个关节一个关节地转(V1b,2026-09-26):发一条【只有关节】的动作。
   --  每个不同名字的关节组发一份(名字相同的读数组 / 命令回声组只发一次,取第一个):C.Group 那一组的名字发 C.Q,其余照此刻的读数保持;
   --  抓握通道同样按名字去重、照此刻的读数保持。一条动作里只有关节这一类,不混位姿(对方按键名认动作类型)
   if C.Kind = Joint and then not Joint_Mode (L) and then (C.Group >= 0 or else not C.Groups.Is_Empty) then
      if C.Group >= Natural (L.Lay.Joints.Length) then
         return False;
      end if;
      declare
         Target : constant String := (if C.Group >= 0 then Layout.Last_Seg (L.Lay.Joints (C.Group)) else "");
         --  这个名字的关节组在 C.Groups 里排第几(-1 = 不在)
         function In_Groups (Nm : String) return Integer is
         begin
            for K in 0 .. Natural (C.Groups.Length) - 1 loop
               if C.Groups (K) >= 0 and then C.Groups (K) < Natural (L.Lay.Joints.Length)
                 and then Layout.Last_Seg (L.Lay.Joints (Natural (C.Groups (K)))) = Nm and then K < Natural (C.Qs.Length)
               then
                  return Integer (K);
               end if;
            end loop;
            return -1;
         end In_Groups;
         J_Names, W_Names : Strs;
         J_First, W_First : Ints;
         W_Vals : Floats_Vectors.Vector;   --  每个抓握键这回发的那一串(Jaw_Values;空 = 这回不发)
         N_Keys : Natural;
         function Has (V : Strs; X : String) return Boolean is
         begin
            for Y of V loop
               if Y = X then
                  return True;
               end if;
            end loop;
            return False;
         end Has;
      begin
         for I in 0 .. Natural (L.Lay.Joints.Length) - 1 loop
            if not Has (J_Names, Layout.Last_Seg (L.Lay.Joints (I))) then
               J_Names.Append (Layout.Last_Seg (L.Lay.Joints (I))); J_First.Append (I);
            end if;
         end loop;
         for I in 0 .. Natural (L.Lay.Jaw.Length) - 1 loop
            if not Has (W_Names, Layout.Last_Seg (L.Lay.Jaw (I))) then
               W_Names.Append (Layout.Last_Seg (L.Lay.Jaw (I))); W_First.Append (I);
            end if;
         end loop;
         N_Keys := Natural (J_Names.Length);
         for K in 0 .. Natural (W_Names.Length) - 1 loop
            declare
               J : constant Floats := Nums_At (L, L.Lay.Jaw (W_First (K)));
               --  这条臂自己的抓握通道给了目标就发目标(位姿命令解成关节目标时带着,V1b 3c),别的发这一集给过的最后一个目标(见 Jaw_Set)
               Mine : constant Boolean := W_First (K) = C.Arm and then not C.Jaw.Is_Empty;
            begin
               --  不截:读数范围是开机两头推到头量的(V1b ②;原来截在 [0, 1],x5 的约定)
               W_Vals.Append (Jaw_Values (L, W_First (K), Mine, C, J));
               if not W_Vals.Last_Element.Is_Empty then
                  N_Keys := N_Keys + 1;
               end if;
            end;
         end loop;
         Put_Map (S, N_Keys);
         for K in 0 .. Natural (J_Names.Length) - 1 loop
            Put_Str (S, J_Names (K));
            declare
               Kg : constant Integer := In_Groups (J_Names (K));
               Q : constant Floats := (if Kg >= 0 then C.Qs (Natural (Kg)) elsif J_Names (K) = Target then C.Q else Nums_At (L, L.Lay.Joints (J_First (K))));
            begin
               Put_Array (S, Natural (Q.Length));
               for X of Q loop
                  Put_Float (S, X);
               end loop;
            end;
         end loop;
         for K in 0 .. Natural (W_Names.Length) - 1 loop
            if not W_Vals (K).Is_Empty then
               Put_Str (S, W_Names (K));
               Put_Array (S, Natural (W_Vals (K).Length));
               for X of W_Vals (K) loop
                  Put_Float (S, X);
               end loop;
            end if;
         end loop;
      end;
      L.Pending := S; L.Has_Pending := True;
      return True;
   end if;
   if (C.Kind = Joint) /= Joint_Mode (L) then
      return False;    --  关节命令只在关节模式,位姿命令只在位姿模式:一条动作里不许混两种类型
   end if;
   declare
      --  每只手的抓握键先凑好这回发的那一串(Jaw_Values;空 = 这回不发:这一拍没读数、这一集也没发过),再数一条动作里有几个键
      Jv : Floats_Vectors.Vector;
      Ji_Of : Ints;
      N_Keys : Natural := N;
   begin
      for I in 0 .. N - 1 loop
         if L.Lay.Jaw.Is_Empty then
            Jv.Append (F64_Vectors.Empty_Vector); Ji_Of.Append (0);
         else
            declare
               Ji : constant Natural := Natural'Min (I, Natural (L.Lay.Jaw.Length) - 1);
               Mine : constant Boolean := (I = C.Arm or else Natural (L.Lay.Jaw.Length) = 1) and then not C.Jaw.Is_Empty;
            begin
               --  没给命令的通道发这一集给过它的最后一个目标(一次只动脑点名的那一根手指;见 Jaw_Set);不截(同上)
               Jv.Append (Jaw_Values (L, Ji, Mine, C, Nums_At (L, L.Lay.Jaw (Ji))));
               Ji_Of.Append (Ji);
            end;
         end if;
         if not Jv.Last_Element.Is_Empty then
            N_Keys := N_Keys + 1;
         end if;
      end loop;
      Put_Map (S, N_Keys);
      for I in 0 .. N - 1 loop
         if Joint_Mode (L) then
            Put_Str (S, Layout.Last_Seg (L.Lay.Joints (I)));
            declare
               --  发给第几组:给了 Group 就按它(开机前半段按读数组认手,V1b 3c),没给按臂
               Tg : constant Natural := (if C.Group >= 0 then Natural (C.Group) else C.Arm);
               Kg : Integer := -1;
            begin
               for K in 0 .. Natural'Min (Natural (C.Groups.Length), Natural (C.Qs.Length)) - 1 loop
                  if C.Groups (K) = I then
                     Kg := Integer (K);
                  end if;
               end loop;
               declare
                  Q : constant Floats := (if Kg >= 0 then C.Qs (Natural (Kg)) elsif C.Groups.Is_Empty and then I = Tg then C.Q else Nums_At (L, L.Lay.Joints (I)));
               begin
                  Put_Array (S, Natural (Q.Length));
                  for X of Q loop
                     Put_Float (S, X);
                  end loop;
               end;
            end;
         else
            Put_Str (S, Layout.Last_Seg (L.Lay.EE (I)));
            if I = C.Arm then
               Put_Array (S, 7);
               for K in 0 .. 6 loop
                  Put_Float (S, C.Pose (K));
               end loop;
            else
               declare
                  P : constant Floats := Nums_At (L, L.Lay.EE (I));
               begin
                  Put_Array (S, Natural (P.Length));
                  for X of P loop
                     Put_Float (S, X);
                  end loop;
               end;
            end if;
         end if;
         if not Jv (I).Is_Empty then
            Put_Str (S, Layout.Last_Seg (L.Lay.Jaw (Natural (Ji_Of (I)))));
            Put_Array (S, Natural (Jv (I).Length));
            for X of Jv (I) loop
               Put_Float (S, X);
            end loop;
         end if;
      end loop;
   end;
   L.Pending := S; L.Has_Pending := True;
   return True;
end Act_Raw;
