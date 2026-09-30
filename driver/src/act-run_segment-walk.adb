separate (Act.Run_Segment)
procedure Walk (Ok_Out : out Boolean) is
begin
   Before_All := All_Gray (F);
   Was := Pts;
   Was_EE := F.EE (Arm);
   Was_Regs := (if Cam_Arm (C, Cam) /= Integer (Arm) then Cut_Things (C, F, Cam) else Picture.Region_Vectors.Empty_Vector);
   Step_Arm (L, C, F, Arm, Note.Cmd, Jaw, Note.Got, Ok_Out, C.Fast, Watch_Things'Unrestricted_Access);
   Beats := Since (L, Beats0);
   if Note.Halted then
      Put_Line ("[身]     途中眼睛叫停:被跟的东西快出画面或看不见了,这一步没走完");
   end if;
   if not Ok_Out then
      return;
   end if;
   Steps_Taken := Steps_Taken + 1;
   declare
      Sv : Backup.Step_Vec := [others => 0.0];
   begin
      for K in 0 .. Chan.Per_Arm - 1 loop
         Sv (K) := Note.Got (K);
         --  🔴 学死区:命令发了而身体没动 ⇒ 这一档不够,抬上去;真动了 ⇒ 说明这一档够,压下来。
         --  抬到刚才那一档的一半再加一次(=1.5 倍,倍数无量纲),压到刚好走成的那一档。
         declare
            Cn : constant Natural := Arm * Chan.Per_Arm + K;
            Half : constant Long_Float := abs Note.Cmd (K) / 2.0;
         begin
            if Cn < Natural (C.Dead.Length) and then abs Note.Cmd (K) > C.Map.EE_Noise then
               if abs Note.Got (K) <= C.Map.EE_Noise then
                  C.Dead.Replace_Element
                    (Cn, Long_Float'Max (C.Dead.Element (Cn), abs Note.Cmd (K) + Half));
               else
                  C.Dead.Replace_Element
                    (Cn, Long_Float'Min (C.Dead.Element (Cn), abs Note.Cmd (K)));
               end if;
            end if;
         end;
      end loop;
      Backup.Remember (Ring, Sv);
   end;
   Note.Pic_Delta := Long_Float (Picture.Max_Diff (Before_All (Cam), F.Cams (Cam).Gray));
end Walk;
