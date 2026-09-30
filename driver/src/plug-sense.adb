separate (Plug)
function Sense (L : in out Link; F : out Frame) return Boolean is
   use Ada.Calendar;
   T0 : constant Time := Clock;
   T1 : Time;
begin
   if Lockstep.Current_Hand >= 0 then
      --  手的任务:把棒交还主线程,这一拍由主线程收(Lock_Beat),醒来拿那一帧
      Lockstep.Yield;
      F := Lock_F;
      return Lock_Ok;
   end if;
   F := (others => <>);
   if not Pump (L) then
      return False;
   end if;
   T1 := Clock;
   L.Seq := L.Seq + 1;
   Frame_Of (L, F);
   Note_Beat (L, F);   --  这一拍记下来(量画面比读数晚几拍用;见 Beat)
   --  录像:BL_VID 全分辨率灰度(开头密、后面疏,编号连续 ⇒ mkvid 能拼);BL_FILM 半分辨率抽帧。
   declare
      Vid : constant String := Codec.Env ("BL_VID");
      Film : constant String := Codec.Env ("BL_FILM");
   begin
      if Vid /= "" then
         --  每一帧的位姿读数都落盘(poses.txt:帧号、这一帧存下的画面编号或 -1、每条臂 xyz + wxyz),离线能核"画面和位姿是不是同一刻"
         --  (2026-09-24:人形腕眼焦距几炮都偏低 1–6%,x5 上在 1% 以内;要量的是画面是不是比位姿晚)
         declare
            Fo : File_Type;
            Pth : constant String := Vid & "/poses.txt";
            Saved : constant Boolean := L.Seq <= 2000 or else L.Seq mod 20 = 0;
         begin
            Codec.Make_Dir (Vid);
            begin
               Open (Fo, Append_File, Pth);
            exception
               when others => Create (Fo, Out_File, Pth);
            end;
            Put (Fo, Codec.Img (L.Seq) & " " & (if Saved then Codec.Img (L.Vid_N) else "-1"));
            for P of F.Reported_EE loop   --  身体自己报的(只给离线打分)
               for I in P'Range loop
                  Put (Fo, " " & Codec.Fmt (P (I), 6));
               end loop;
            end loop;
            New_Line (Fo);
            Close (Fo);
            --  同一帧的关节读数(joints.txt:帧号、画面编号或 -1、每组关节 "| v…";组的顺序同开机 [认] 关节角那一行)。
            --  2026-09-26 V1b:离线量"关节转多少、手到哪" —— 只记录,不改行为
            begin
               Open (Fo, Append_File, Vid & "/joints.txt");
            exception
               when others => Create (Fo, Out_File, Vid & "/joints.txt");
            end;
            Put (Fo, Codec.Img (L.Seq) & " " & (if Saved then Codec.Img (L.Vid_N) else "-1"));
            for Q of F.Joints loop
               Put (Fo, " |");
               for X of Q loop
                  Put (Fo, " " & Codec.Fmt (X, 6));
               end loop;
            end loop;
            New_Line (Fo);
            Close (Fo);
         exception
            when others => null;
         end;
         --  2000 / 20 是帧计数(无量纲),只管落图密度
         if L.Seq <= 2000 or else L.Seq mod 20 = 0 then
            Codec.Make_Dir (Vid);
            for Ci in 0 .. Natural (F.Cams.Length) - 1 loop
               Codec.Write_PGM (Vid & "/f" & Codec.Pad6 (L.Vid_N) & "_c" & Codec.Img (Ci) & ".pgm",
                                F.Cams (Ci).Gray, F.Cams (Ci).W, F.Cams (Ci).H);
            end loop;
            L.Vid_N := L.Vid_N + 1;
         end if;
      end if;
      if Film /= "" then
         declare
            Stride : constant Natural := Natural'Max (1, Codec.Env_Nat ("BL_FILM_STRIDE", 6));
            Max : constant Natural := Codec.Env_Nat ("BL_FILM_MAX", 6000);
         begin
            if L.Seq mod Stride = 0 and then L.Film_N < Max then
               Codec.Make_Dir (Film);
               for Ci in 0 .. Natural (F.Cams.Length) - 1 loop
                  declare
                     C : Cam renames F.Cams (Ci);
                     Hw : constant Natural := C.W / 2;
                     Hh : constant Natural := C.H / 2;
                     G : Buf;
                  begin
                     for Y in 0 .. Hh - 1 loop
                        for X in 0 .. Hw - 1 loop
                           G.Append (C.Gray.Element ((Y * 2) * C.W + X * 2));
                        end loop;
                     end loop;
                     Codec.Write_PGM (Film & "/c" & Codec.Img (Ci) & "_" & Codec.Pad6 (L.Film_N) & ".pgm", G, Hw, Hh);
                  end;
               end loop;
               L.Film_N := L.Film_N + 1;
            end if;
         end;
      end if;
   end;
   L.Wait_Us := L.Wait_Us + Long_Float (T1 - T0) * 1.0e6;
   L.Parse_Us := L.Parse_Us + Long_Float (Clock - T1) * 1.0e6;
   if L.Seq mod 50 = 0 then
      --  50 帧(次数)× 1e6 微秒
      L.Frame_S := (L.Wait_Us + L.Parse_Us) / 50.0e6;
      Put_Line ("      [计时] 近 50 帧:等帧 " & Codec.Fmt (L.Wait_Us / 50000.0, 1) & " ms/帧 · 解图 " &
                Codec.Fmt (L.Parse_Us / 50000.0, 1) & " ms/帧 · 一拍 " & Codec.Fmt (L.Frame_S, 3) &
                " s · 相机" & Natural'Image (Natural (F.Cams.Length)) & " 台");
      L.Wait_Us := 0.0; L.Parse_Us := 0.0;
   end if;
   if Hook_P /= null then
      Hook_P (F);
      --  按关节读数算出来的手的位姿也落盘(fk_poses.txt,格式同 poses.txt):离线和身体报的比,驱动不读这个文件
      declare
         Vid : constant String := Codec.Env ("BL_VID");
         Fo : File_Type;
      begin
         if Vid /= "" then
            begin
               Open (Fo, Append_File, Vid & "/fk_poses.txt");
            exception
               when others => Create (Fo, Out_File, Vid & "/fk_poses.txt");
            end;
            Put (Fo, Codec.Img (L.Seq) & " -1");
            for P of F.EE loop
               for I in P'Range loop
                  Put (Fo, " " & Codec.Fmt (P (I), 6));
               end loop;
            end loop;
            New_Line (Fo);
            Close (Fo);
         end if;
      exception
         when others => null;
      end;
   end if;
   return True;
end Sense;
