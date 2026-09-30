separate (Plug)
procedure Note_Beat (L : in out Link; F : Frame) is
   B : Beat;
begin
   B.Seq := F.Seq; B.Joints := F.Joints; B.Reported_EE := F.Reported_EE;
   for Ci in 0 .. Natural (F.Cams.Length) - 1 loop
      declare
         W : constant Natural := F.Cams (Ci).W;
         H : constant Natural := F.Cams (Ci).H;
         Sum : Long_Float := 0.0;
         Cnt : Natural := 0;
      begin
         --  这一拍、上一拍都有这台的画面(而且一样大)才量;占位的那一拍、占位之后的那一拍这台都记"没量"(Img_Ok = False)
         if Has_Picture (F.Cams (Ci)) and then Ci < Natural (L.Prev_Gray.Length) and then Natural (L.Prev_Gray (Ci).Length) = W * H then
            declare
               G0 : Buf renames L.Prev_Gray (Ci);
               G1 : Buf renames F.Cams (Ci).Gray;
            begin
               for Y in 0 .. (H - 1) / Img_Stride loop
                  for X in 0 .. (W - 1) / Img_Stride loop
                     declare
                        I : constant Natural := Y * Img_Stride * W + X * Img_Stride;
                     begin
                        Sum := Sum + abs (Long_Float (G1 (I)) - Long_Float (G0 (I)));
                        Cnt := Cnt + 1;
                     end;
                  end loop;
               end loop;
            end;
         end if;
         B.Img_Chg.Append (if Cnt > 0 then Sum / Long_Float (Cnt) else 0.0);
         B.Img_Ok.Append (Cnt > 0);
      end;
   end loop;
   for Gi in 0 .. Natural (F.Joints.Length) - 1 loop
      declare
         Mx : Long_Float := 0.0;
      begin
         if not L.Beats.Is_Empty and then Gi < Natural (L.Beats.Last_Element.Joints.Length) then
            declare
               Q0 : constant Floats := L.Beats.Last_Element.Joints (Gi);
            begin
               for K in 0 .. Natural'Min (Natural (Q0.Length), Natural (F.Joints (Gi).Length)) - 1 loop
                  Mx := Long_Float'Max (Mx, abs (F.Joints (Gi) (K) - Q0 (K)));
               end loop;
            end;
         end if;
         B.Q_Chg.Append (Mx);
      end;
   end loop;
   L.Beats.Append (B);
   if Natural (L.Beats.Length) > Keep_Beats then
      L.Beats.Delete_First (Ada.Containers.Count_Type (Natural (L.Beats.Length) - Keep_Beats));
   end if;
   L.Prev_Gray.Clear;
   for C of F.Cams loop
      L.Prev_Gray.Append (C.Gray);   --  占位的那台存空的 ⇒ 下一拍这台也不量(没有"上一拍")
   end loop;
end Note_Beat;
