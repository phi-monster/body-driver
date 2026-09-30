separate (Jointboot)
procedure Dump_Kin (Dump : String; K : Kin_Store) is
   use Ada.Text_IO;
   Fo : File_Type;
begin
   if Dump = "" then
      return;
   end if;
   for A in 0 .. Natural (K.Worlds.Length) - 1 loop
      declare
         W : constant Arm_World := K.Worlds (A);
      begin
         Create (Fo, Out_File, Dump & "/kinem_arm" & Codec.Img (A) & ".txt");
         Put_Line (Fo, "arm " & Codec.Img (A) & " n " & Codec.Img (W.Model.N) & " f " & Codec.Fmt (W.Model.F, 6) & " cx " & Codec.Fmt (W.Model.Cx, 3) & " cy " & Codec.Fmt (W.Model.Cy, 3));
         Put (Fo, "q0");
         for X of W.Model.Q0 loop
            Put (Fo, " " & F9 (X));
         end loop;
         New_Line (Fo);
         for J in 0 .. W.Model.N - 1 loop
            Put_Line (Fo, "axis " & Codec.Img (J) & " " & F9 (W.Model.Ax (J).W (0)) & " " & F9 (W.Model.Ax (J).W (1)) & " " & F9 (W.Model.Ax (J).W (2)) & " "
                      & F9 (W.Model.Ax (J).P (0)) & " " & F9 (W.Model.Ax (J).P (1)) & " " & F9 (W.Model.Ax (J).P (2)) & " " & Kind_Word (W.Model.Ax (J)));
         end loop;
         for P of W.Model.Eye loop   --  长在眼上的像素(身体文件不存 ⇒ 读回来的没有)
            Put_Line (Fo, "eye " & Codec.Fmt (P.U, 3) & " " & Codec.Fmt (P.V, 3));
         end loop;
         Close (Fo);
         if A > 0 then
            Create (Fo, Out_File, Dump & "/align_arm" & Codec.Img (A) & ".txt");
            Put (Fo, "S " & F9 (W.S) & " R");
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  Put (Fo, " " & F9 (W.Ra (I, J)));
               end loop;
            end loop;
            Put_Line (Fo, " T " & F9 (W.Ta (0)) & " " & F9 (W.Ta (1)) & " " & F9 (W.Ta (2)));
            Close (Fo);
         end if;
      end;
   end loop;
   Create (Fo, Out_File, Dump & "/world.txt");
   for I in 0 .. 2 loop
      for J in 0 .. 2 loop
         Put (Fo, F9 (K.Rw (I, J)) & " ");
      end loop;
   end loop;
   Put_Line (Fo, F9 (K.O (0)) & " " & F9 (K.O (1)) & " " & F9 (K.O (2)));
   Close (Fo);
   if K.Fixed_Eye.Valid then
      --  存的不动的眼是世界系的(对齐交出来时已按 Rw、O 换过);落盘同对齐那份 = 第一只手的系:X_手 = Rwᵀ X_世界 + O
      declare
         Pos : constant Geom.V3 := Geom.Ap (Geom.Tr (K.Rw), K.Fixed_Eye.Pos);
         Rc : constant Geom.M3 := Geom.Mul (Geom.Tr (K.Rw), K.Fixed_Eye.R_Ce);
      begin
         Create (Fo, Out_File, Dump & "/fixed_eye.txt");
         Put (Fo, "f " & Codec.Fmt (K.Fixed_Eye.F, 6) & " cx " & Codec.Fmt (K.Fixed_Eye.Cx, 3) & " cy " & Codec.Fmt (K.Fixed_Eye.Cy, 3) & " rms " & Codec.Fmt (K.Fixed_Eye.Rms, 4) & " used 0 of 0 pos "
              & F9 (Pos (0) + K.O (0)) & " " & F9 (Pos (1) + K.O (1)) & " " & F9 (Pos (2) + K.O (2)) & " R");
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Put (Fo, " " & F9 (Rc (I, J)));
            end loop;
         end loop;
         New_Line (Fo);
         Close (Fo);
      end;
   end if;
exception
   when others =>
      if Is_Open (Fo) then
         Close (Fo);
      end if;
end Dump_Kin;
