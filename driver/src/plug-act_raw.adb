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
   --  抓握通道同样按名字去重、照此刻的读数保持。一条动作里只有关节这一类,不混位姿(对方按键名认动作类型)。
   --  开机按量认完(Lay.Measured,I1 10-01):关节命令一律走这一支(只报关节的身体也走;没给组号 ⇒ 按臂:第 A 条臂的关节组就是 Joints 的第 A 组);
   --  抓握键按量出来的归属发(第几条臂、在那条臂接起来的目标里从第几个起);别的命令组(Holds:扛着全身的、零件、哑巴、推不动的)照此刻的读数保持
   if C.Kind = Joint and then (L.Lay.Measured or else (not Joint_Mode (L) and then (C.Group >= 0 or else not C.Groups.Is_Empty))) then
      if C.Group >= Natural (L.Lay.Joints.Length) then
         return False;
      end if;
      declare
         Tg : constant Integer := (if C.Group >= 0 then C.Group
                                   elsif C.Groups.Is_Empty and then C.Arm < Natural (L.Lay.Joints.Length) then Integer (C.Arm) else -1);
         Target : constant String := (if Tg >= 0 then Layout.Last_Seg (L.Lay.Joints (Natural (Tg))) else "");
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
         --  量出来的布局里 Jaw 的第 Wi 组归哪条臂(-1 = 回声),Off = 它在那条臂接起来的抓握目标里从第几个起;旧布局:下标就是臂
         function Owner (Wi : Natural; Off : out Natural) return Integer is
         begin
            Off := 0;
            if not L.Lay.Measured then
               return Integer (Wi);
            end if;
            for A in 0 .. Natural'Min (L.Lay.N_Arms, Natural'Min (Natural (L.Lay.Closing_First.Length), Natural (L.Lay.Closing_N.Length))) - 1 loop
               declare
                  First : constant Natural := Natural (L.Lay.Closing_First (A));
                  N_A : constant Natural := Natural (L.Lay.Closing_N (A));
               begin
                  if Wi >= First and then Wi < First + N_A then
                     for K in First .. Wi - 1 loop
                        Off := Off + (if K < Natural (L.Lay.Jaw_Len.Length) then Natural (L.Lay.Jaw_Len (K)) else 0);
                     end loop;
                     return Integer (A);
                  end if;
               end;
            end loop;
            return -1;
         end Owner;
         J_Names, W_Names : Strs;
         J_First, W_First : Ints;
         Keys : Strs;
         Vals : Floats_Vectors.Vector;
      begin
         for I in 0 .. Natural (L.Lay.Joints.Length) - 1 loop
            if not Has_Name (J_Names, Layout.Last_Seg (L.Lay.Joints (I))) then
               J_Names.Append (Layout.Last_Seg (L.Lay.Joints (I))); J_First.Append (I);
            end if;
         end loop;
         for I in 0 .. Natural (L.Lay.Jaw.Length) - 1 loop
            if not Has_Name (W_Names, Layout.Last_Seg (L.Lay.Jaw (I))) and then not Has_Name (J_Names, Layout.Last_Seg (L.Lay.Jaw (I))) then
               W_Names.Append (Layout.Last_Seg (L.Lay.Jaw (I))); W_First.Append (I);
            end if;
         end loop;
         for K in 0 .. Natural (J_Names.Length) - 1 loop
            declare
               Kg : constant Integer := In_Groups (J_Names (K));
               --  没给目标的组照此刻的读数保持(这一拍没读数 ⇒ 上一回发出去的;都没有 ⇒ 这个键这回不发,不编)
               Q : constant Floats := (if Kg >= 0 then C.Qs (Natural (Kg)) elsif J_Names (K) = Target then C.Q
                                       else Hold_Value (L, J_Names (K), L.Lay.Joints (J_First (K))));
            begin
               if not Q.Is_Empty then
                  Keys.Append (J_Names (K)); Vals.Append (Q);
               end if;
            end;
         end loop;
         for K in 0 .. Natural (W_Names.Length) - 1 loop
            declare
               J : constant Floats := Nums_At (L, L.Lay.Jaw (W_First (K)));
               Off : Natural;
               Ow : constant Integer := Owner (W_First (K), Off);
               Cs : Cmd := C;
               Mine : Boolean;
            begin
               if L.Lay.Measured then
                  --  这条臂接起来的抓握目标里属于这一组的那一截(给了几个就是几个;没给到这一组 ⇒ 这一组照这一集给过的 / 读数保持)
                  Cs.Jaw.Clear;
                  if Ow >= 0 and then Ow = Integer (C.Arm) then
                     declare
                        N_K : constant Natural := (if W_First (K) < Natural (L.Lay.Jaw_Len.Length) then Natural (L.Lay.Jaw_Len (W_First (K))) else 0);
                     begin
                        for X in Off .. Natural'Min (Off + N_K, Natural (C.Jaw.Length)) - 1 loop
                           Cs.Jaw.Append (C.Jaw (X));
                        end loop;
                     end;
                  end if;
                  Mine := not Cs.Jaw.Is_Empty;
               else
                  --  这条臂自己的抓握通道给了目标就发目标(位姿命令解成关节目标时带着,V1b 3c),别的发这一集给过的最后一个目标(见 Jaw_Set)
                  Mine := W_First (K) = C.Arm and then not C.Jaw.Is_Empty;
               end if;
               --  不截:读数范围是开机两头推到头量的(V1b ②;原来截在 [0, 1],x5 的约定)
               declare
                  V : constant Floats := Jaw_Values (L, W_First (K), Mine, Cs, J);
               begin
                  if not V.Is_Empty then
                     Keys.Append (W_Names (K)); Vals.Append (V);
                  end if;
               end;
            end;
         end loop;
         for P of L.Lay.Holds loop
            declare
               Nm : constant String := Layout.Last_Seg (P);
            begin
               if not Has_Name (Keys, Nm) and then not Has_Name (J_Names, Nm) and then not Has_Name (W_Names, Nm) then
                  declare
                     V : constant Floats := Hold_Value (L, Nm, P);
                  begin
                     if not V.Is_Empty then
                        Keys.Append (Nm); Vals.Append (V);
                     end if;
                  end;
               end if;
            end;
         end loop;
         Put_Keys (S, Keys, Vals);
         Note_Sent (L, Keys, Vals);
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
