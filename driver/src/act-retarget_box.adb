separate (Act)
procedure Retarget_Box (C : in out Context; F : Plug.Frame; Cam, Arm : Natural; Name : Unbounded_String; P : Geom.V3) is
   Bx : constant Integer := Boxed_By (C, Cam, Name);
   G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
   Pu, Pv : Long_Float;
   Front : Boolean;
begin
   if Bx < 0 or else Cam >= Natural (F.Cams.Length) or else Arm >= Natural (F.EE.Length) then
      return;
   end if;
   Geom.Project (G, F.EE (Arm), P, Pu, Pv, Front);
   declare
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      B : Boxed_Thing := C.Boxed (Natural (Bx));
      Hw : constant Integer := (Integer (B.X1) - Integer (B.X0)) / 2;   --  半宽(纯数学的一半)
      Hh : constant Integer := (Integer (B.Y1) - Integer (B.Y0)) / 2;
      function Px (V2 : Long_Float; Span : Natural) return Natural is
        (Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Span - 1), V2))));
   begin
      if Front and then Pu >= 0.0 and then Pv >= 0.0 and then Pu < Long_Float (Cw) and then Pv < Long_Float (Ch)
        and then Hw > 0 and then Hh > 0
      then
         if B.Pu_On >= 0.0 then   --  它身上那一点跟着框平移(框心从旧的挪到预测处)
            B.Pu_On := Pu + (B.Pu_On - 0.5 * Long_Float (B.X0 + B.X1));
            B.Pv_On := Pv + (B.Pv_On - 0.5 * Long_Float (B.Y0 + B.Y1));
         end if;
         B.X0 := Px (Pu - Long_Float (Hw), Cw); B.X1 := Px (Pu + Long_Float (Hw), Cw);
         B.Y0 := Px (Pv - Long_Float (Hh), Ch); B.Y1 := Px (Pv + Long_Float (Hh), Ch);
         B.Blind := False;   --  这只眼看的地方变了,以前"这儿没有它"不再算数
         C.Boxed.Replace_Element (Natural (Bx), B);
         C.Cut_Cam := -1;    --  这一帧按挪过的窗重量
         Geo_Say ("它该出现在 (" & Codec.Fmt (Pu, 1) & "," & Codec.Fmt (Pv, 1) & ")(按转过/挪过的眼算)⇒ 窗挪过去再量");
      end if;
   end;
end Retarget_Box;
