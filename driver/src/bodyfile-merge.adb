separate (Bodyfile)
procedure Merge (Stored, Fresh : Selfmap.Body_Map; Merged : out Selfmap.Body_Map; Replaced, Kept : out Natural) is
begin
   Merged := Fresh;
   Replaced := 0; Kept := 0;
   Merged.Amp_Hist := Stored.Amp_Hist; Merged.Deliv_Hist := Stored.Deliv_Hist;
   while Natural (Merged.Amp_Hist.Length) < Fresh.Channels loop
      Merged.Amp_Hist.Append (F64_Vectors.Empty_Vector);
   end loop;
   while Natural (Merged.Deliv_Hist.Length) < Fresh.Channels loop
      Merged.Deliv_Hist.Append (F64_Vectors.Empty_Vector);
   end loop;
   for Ch in 0 .. Fresh.Channels - 1 loop
      if Fresh.Seen (Ch) then
         declare
            Ha : Floats := Merged.Amp_Hist (Ch);
            Hd : Floats := Merged.Deliv_Hist (Ch);
            Old_Amp : constant Long_Float := (if Ha.Is_Empty then Fresh.Amp (Ch) else Median (Ha));
         begin
            Ha.Append (Fresh.Amp (Ch)); Hd.Append (Fresh.Delivered (Ch));
            while Natural (Ha.Length) > History_Depth loop
               Ha.Delete_First;
            end loop;
            while Natural (Hd.Length) > History_Depth loop
               Hd.Delete_First;
            end loop;
            Merged.Amp_Hist.Replace_Element (Ch, Ha);
            Merged.Deliv_Hist.Replace_Element (Ch, Hd);
            Merged.Amp.Replace_Element (Ch, Median (Ha));
            Merged.Delivered.Replace_Element (Ch, Median (Hd));
            if abs (Merged.Amp (Ch) - Old_Amp) > 0.0 then
               Replaced := Replaced + 1;
            else
               Kept := Kept + 1;
            end if;
         end;
      end if;
   end loop;
   --  噪声地板只放大不缩小(LAB 8-18:跨炮一致不代表 σ 能变小)
   Merged.EE_Noise := Long_Float'Max (Stored.EE_Noise, Fresh.EE_Noise);
   Merged.Rot_Noise := Long_Float'Max (Stored.Rot_Noise, Fresh.Rot_Noise);
   Merged.Jaw_Noise := Long_Float'Max (Stored.Jaw_Noise, Fresh.Jaw_Noise);
   Merged.Measured_Times := Stored.Measured_Times + 1;
end Merge;
