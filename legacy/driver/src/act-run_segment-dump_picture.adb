separate (Act.Run_Segment)
procedure Dump_Picture (Tag : String) is
begin
   if C.Dump_Dir = "" then
      return;
   end if;
   declare
      RGB : Buf := F.Cams (Cam).RGB;
      Regs : constant Picture.Regions := Cut_Things (C, F, Cam);
      N : Natural := 0;
   begin
      for R of Regs loop
         N := N + 1;
         Draw.Numbered_Box (RGB, Cw, Ch, R.X0, R.Y0, R.X1, R.Y1, N, Draw.Green, 2);
      end loop;
      for P of Pts loop
         declare
            Hw : constant Long_Float := Long_Float'Max (P.Box_W, Track_Win) * 0.5;   --  框(没有就用一个跟踪窗;比例,无量纲)
            Hh : constant Long_Float := Long_Float'Max (P.Box_H, Track_Win) * 0.5;
         begin
            Draw.Numbered_Box (RGB, Cw, Ch,
                               Natural (Long_Float'Max (0.0, (P.Cu - Hw) * Long_Float (Cw))),
                               Natural (Long_Float'Max (0.0, (P.Cv - Hh) * Long_Float (Ch))),
                               Natural (Long_Float'Min (Long_Float (Cw - 1), (P.Cu + Hw) * Long_Float (Cw))),
                               Natural (Long_Float'Min (Long_Float (Ch - 1), (P.Cv + Hh) * Long_Float (Ch))),
                               0, Draw.Pink, 2);
         end;
      end loop;
      Codec.Write_BMP (To_String (C.Dump_Dir) & "/" & Tag & "_" & Codec.Pad6 (C.Round_N) & "_" & Codec.Pad6 (Steps_Taken) & ".bmp", RGB, Cw, Ch);
      Put_Line ("[身]     落图 " & Tag & "_" & Codec.Pad6 (C.Round_N) & "_" & Codec.Pad6 (Steps_Taken) & ".bmp(切出" & Natural'Image (N) & " 块)");
   end;
end Dump_Picture;
