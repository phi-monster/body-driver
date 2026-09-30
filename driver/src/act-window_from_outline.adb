separate (Act)
procedure Window_From_Outline (C : in out Context; F : Plug.Frame; Cam : Natural; Name : Unbounded_String) is
   G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
   A2 : constant Integer := Cam_Arm (C, Cam);
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
   X0 : Integer := Integer'Last;
   Y0 : Integer := Integer'Last;
   X1 : Integer := -1;
   Y1 : Integer := -1;
   N : Natural := 0;
   Bt : Boxed_Thing;
begin
   if Boxed_By (C, Cam, Name) >= 0 or else not C.Sil_Valid or else C.Sil_Name /= Name then
      return;
   end if;
   if not ((A2 < 0 and then G.Fixed) or else (A2 >= 0 and then G.Valid and then G.F > 0.0 and then A2 < Integer (F.EE.Length))) then
      return;
   end if;
   for P of C.Sil_Pts loop
      declare
         U, V : Long_Float;
         Front : Boolean;
      begin
         if A2 < 0 then
            Geom.Project_Fixed (G, P, U, V, Front);
         else
            Geom.Project (G, F.EE (Natural (A2)), P, U, V, Front);
         end if;
         if Front and then U >= 0.0 and then V >= 0.0 and then U < Long_Float (Cw) and then V < Long_Float (Ch) then
            X0 := Integer'Min (X0, Integer (U));
            X1 := Integer'Max (X1, Integer (U));
            Y0 := Integer'Min (Y0, Integer (V));
            Y1 := Integer'Max (Y1, Integer (V));
            N := N + 1;
         end if;
      end;
   end loop;
   if N < 8 or else X1 <= X0 or else Y1 <= Y0 then   --  点数
      return;
   end if;
   Bt.Name := Name;
   Bt.Cam := Cam;
   Bt.X0 := X0; Bt.Y0 := Y0; Bt.X1 := X1; Bt.Y1 := Y1;
   Bt.Cu := Long_Float (X0 + X1) / 2.0 / Long_Float (Cw);
   Bt.Cv := Long_Float (Y0 + Y1) / 2.0 / Long_Float (Ch);
   Bt.Seen := False;
   C.Boxed.Append (Bt);
   C.Cut_Cam := -1;
   Geo_Say ("第" & Codec.Img (Cam) & " 台眼里没有它的窗 ⇒ 把它顶面的点投进这只眼:窗 [" & Codec.Img (X0) & " " & Codec.Img (Y0) & " " & Codec.Img (X1) & " " & Codec.Img (Y1)
            & "](" & Codec.Img (N) & " 个点落在画面里),窗里哪一片是它照常重量");
end Window_From_Outline;
