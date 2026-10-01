separate (Jointboot)
procedure Self_Check (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; Ds : Sweep_Vectors.Vector; Dump : String) is
   use Geom;
   Fo : Ada.Text_IO.File_Type;
   Have_Fo : Boolean := False;
   Na : constant Natural := Natural (St_Worlds.Length);
   --  每只手的目标:两格"几个关节一起动"的读数正中(扫描时没去过的地方)按运动学算到的腕眼位姿,搬到世界里
   Cmb : array (0 .. Natural'Max (1, Na) - 1) of Ints;
   N_Done : array (0 .. Natural'Max (1, Na) - 1) of Natural := [others => 0];
   Sum_P, Max_P, Max_Pe : array (0 .. Natural'Max (1, Na) - 1) of Long_Float := [others => 0.0];
   T0c : constant Ada.Calendar.Time := Ada.Calendar.Clock;
   Seq0 : constant Natural := L.Seq;
begin
   if Dump /= "" then
      begin
         Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/ik_check.txt");
         Have_Fo := True;
      exception
         when others => null;
      end;
   end if;
   for A in 0 .. Na - 1 loop
      for K in 1 .. Natural (Ds (St_Worlds (A).Sweep).Frames.Length) - 1 loop
         if Ds (St_Worlds (A).Sweep).Frames (K).Joint < 0 then
            Cmb (A).Append (K);
         end if;
      end loop;
   end loop;
   --  几只手同时走:每一处一条关节命令带几只手的目标(每只手的目标按位姿命令同一条路 Pose_To_Q 解)
   for I in 0 .. N_Check - 1 loop
      declare
         Targets : Plug.Pose_Vectors.Vector;
         Pes, Rrs : Floats;
         Gs : Ints;
         Qs : Plug.Floats_Vectors.Vector;
         Tol : Long_Float := Long_Float'Last;
         Dl : Table.Vec;
         Fr : Natural;
         Okg : Boolean;
      begin
         for A in 0 .. Na - 1 loop
            declare
               W : constant Arm_World := St_Worlds (A);
               Target : Plug.Arm_Pose := [others => 0.0];
               Q : Floats;
               Pe, Re : Long_Float := 0.0;
            begin
               if I + 1 < Natural (Cmb (A).Length) then
                  declare
                     Qa : constant Floats := Ds (W.Sweep).Frames (Natural (Cmb (A) (I))).Q;
                     Qb : constant Floats := Ds (W.Sweep).Frames (Natural (Cmb (A) (I + 1))).Q;
                     Qm : Floats;
                     Rr : M3;
                     Tt : V3;
                  begin
                     for J in 0 .. Natural'Min (Natural (Qa.Length), Natural (Qb.Length)) - 1 loop
                        Qm.Append (0.5 * (Qa (J) + Qb (J)));
                     end loop;
                     Kinem.FK (W.Model, Qm, Rr, Tt);
                     declare
                        R0 : constant M3 := Mul (W.Ra, Rr);
                        Rt : constant V3 := Ap (W.Ra, Tt);
                        T0 : constant V3 := [W.S * Rt (0) + W.Ta (0) - St_O (0), W.S * Rt (1) + W.Ta (1) - St_O (1), W.S * Rt (2) + W.Ta (2) - St_O (2)];
                     begin
                        Target := Kinem.To_Pose (Mul (St_Rw, R0), Ap (St_Rw, T0));
                     end;
                     declare
                        Cl : Boolean;
                     begin
                        Pose_To_Q (A, Target, True, Q, Pe, Re, Cl);
                     end;
                     Gs.Append (W.Group); Qs.Append (Q);
                     --  到了 = 每个关节差不到它这一下要走的三分之一(比例,同扫描)
                     if W.Group < Natural (F.Joints.Length) then
                        for J in 0 .. Natural'Min (Natural (Q.Length), Natural (F.Joints (W.Group).Length)) - 1 loop
                           if abs (Q (J) - F.Joints (W.Group) (J)) > 0.0 then
                              Tol := Long_Float'Min (Tol, Third * abs (Q (J) - F.Joints (W.Group) (J)));
                           end if;
                        end loop;
                     end if;
                  end;
               end if;
               Targets.Append (Target); Pes.Append (Pe); Rrs.Append (Re);
            end;
         end loop;
         exit when Gs.Is_Empty;
         Selfmap.Go (L, M, 0, [others => 0.0], F64_Vectors.Empty_Vector, F, Dl, Fr, Okg, Groups => Gs, Qs => Qs,
                     Tol => (if Tol < Long_Float'Last then Tol else 0.0));
         exit when not Okg;
         for A in 0 .. Na - 1 loop
            if I + 1 < Natural (Cmb (A).Length) and then A < Natural (F.EE.Length) then
               declare
                  W : constant Arm_World := St_Worlds (A);
                  Target : constant Plug.Arm_Pose := Targets (A);
                  Got : constant Plug.Arm_Pose := F.EE (A);
                  Dp : constant Long_Float := Norm ([Got (0) - Target (0), Got (1) - Target (1), Got (2) - Target (2)]);
               begin
                  N_Done (A) := N_Done (A) + 1;
                  Sum_P (A) := Sum_P (A) + Dp; Max_P (A) := Long_Float'Max (Max_P (A), Dp); Max_Pe (A) := Long_Float'Max (Max_Pe (A), Pes (A));
                  if Have_Fo then
                     Ada.Text_IO.Put (Fo, Codec.Img (A) & " " & Codec.Img (W.Sweep) & " " & Codec.Img (Fr) & " |");
                     for X of Target loop
                        Ada.Text_IO.Put (Fo, " " & Codec.Fmt (X, 7));
                     end loop;
                     Ada.Text_IO.Put (Fo, " |");
                     for X of Got loop
                        Ada.Text_IO.Put (Fo, " " & Codec.Fmt (X, 7));
                     end loop;
                     Ada.Text_IO.Put (Fo, " | " & Codec.Fmt (Pes (A), 7) & " " & Codec.Fmt (Rrs (A), 7) & " |");
                     if W.Group < Natural (F.Joints.Length) then
                        for X of F.Joints (W.Group) loop
                           Ada.Text_IO.Put (Fo, " " & Codec.Fmt (X, 7));
                        end loop;
                     end if;
                     Ada.Text_IO.Put (Fo, " ||");
                     if W.Sweep < Natural (F.Reported_EE.Length) then
                        for X of F.Reported_EE (W.Sweep) loop
                           Ada.Text_IO.Put (Fo, " " & Codec.Fmt (X, 7));   --  身体报的手的位姿:只给离线打分,驱动不读
                        end loop;
                     end if;
                     Ada.Text_IO.New_Line (Fo);
                  end if;
               end;
            end if;
         end loop;
      end;
   end loop;
   for A in 0 .. Na - 1 loop
      Say ("开机自检(V1b ②):第" & Codec.Img (A + 1) & " 只手走到 " & Codec.Img (N_Done (A)) & " 处扫描时没去过的地方(两格""几个关节一起动""的读数正中)⇒ "
           & "按关节读数算到的离目标 平均 " & Codec.Fmt ((if N_Done (A) > 0 then Sum_P (A) / Long_Float (N_Done (A)) else 0.0), 4) & "、最大 " & Codec.Fmt (Max_P (A), 4)
           & " 单位;反解最多还差 " & Codec.Fmt (Max_Pe (A), 4) & " 单位(真值只落盘打分)");
   end loop;
   Say ("  自检 " & Codec.Img (L.Seq - Seq0) & " 拍、" & Codec.Fmt (Long_Float (Ada.Calendar."-" (Ada.Calendar.Clock, T0c)), 1) & " 秒(几只手同时走)");
   if Have_Fo then
      Ada.Text_IO.Close (Fo);
   end if;
end Self_Check;
