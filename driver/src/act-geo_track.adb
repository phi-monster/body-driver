separate (Act)
procedure Geo_Track (C : in out Context; F : Plug.Frame; Cam : Natural; Slot : Integer; U, V : out Long_Float; Seen : out Boolean;
                     Name : Unbounded_String := Null_Unbounded_String) is
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
   Bx : Integer := -1;
begin
   Seen := False; U := 0.0; V := 0.0;
   if not (C.Cut_Seq = F.Seq and then C.Cut_Cam = Integer (Cam)) then
      World.Observe (C.Wld, Cam, Cut_Things (C, F, Cam), Cw, Ch);
   end if;
   --  带着名字来的:这只眼里叫这个名字的那一件,这一帧量到了就是它(槽是脑看着的那只眼的记账,走路的眼未必有槽)
   if Length (Name) > 0 then
      Bx := Boxed_By (C, Cam, Name);
      if Bx >= 0 and then C.Boxed (Natural (Bx)).Seen then
         U := C.Boxed (Natural (Bx)).Cu * Long_Float (Cw); V := C.Boxed (Natural (Bx)).Cv * Long_Float (Ch); Seen := True;
      end if;
      return;
   end if;
   --  🔴 跟的是【脑点过名、我在框里重量出来的那一块】,不是槽。槽是全图切块的记账,认槽靠"就近",
   --  H23 2026-09-22 实测:框里明明量到了(离线复算 3760 px、形心 (244,381)),槽却没对上 ⇒ 报"看丢了"。
   --  点过名的东西按名字找;只有没点过名的才退回槽。
   if Slot >= 0 and then Natural (Slot) < World.Count (C.Wld, Cam) then
      declare
         Sl : constant World.Slot := World.Get (C.Wld, Cam, Natural (Slot));
         Ref : constant Picture.Region := (if Sl.Present then Sl.R else Sl.Shadow);
      begin
         Bx := Boxed_Index (C, Cam, Ref.Cu, Ref.Cv);
         if Bx < 0 then
            --  槽已经被挪到预测处、和框里的读数对不上号 ⇒ 按名字找这只眼里点过名的那一件
            for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
               if C.Boxed (Bi).Cam = Cam and then C.Boxed (Bi).Seen then
                  Bx := Integer (Bi);
               end if;
            end loop;
         end if;
         if Bx >= 0 and then C.Boxed (Natural (Bx)).Seen then
            U := C.Boxed (Natural (Bx)).Cu * Long_Float (Cw); V := C.Boxed (Natural (Bx)).Cv * Long_Float (Ch); Seen := True;
            return;
         end if;
         if Sl.Present and then Sl.Seen then
            U := Sl.R.Cu * Long_Float (Cw); V := Sl.R.Cv * Long_Float (Ch); Seen := True;
         end if;
      end;
   end if;
end Geo_Track;
