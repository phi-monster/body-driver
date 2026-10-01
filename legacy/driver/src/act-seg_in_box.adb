separate (Act)
procedure Seg_In_Box (C : Context; F : Plug.Frame; Cam : Natural; X0, Y0, X1, Y1 : Natural; Got, Iso : out Boolean; R : out Picture.Region; M : out Bools;
                      Pu_On, Pv_On : Long_Float := -1.0) is
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
begin
   if Length (C.Inst_Host) = 0 then
      Got := False; Iso := False; R := (others => <>); M := Bool_Vectors.Empty_Vector;
      if not Said_No_Seg then
         Said_No_Seg := True;
         Put_Line ("[身] 📦 没配分割仪器 ⇒ 量不出脑框出来的东西是哪些像素(这一样只有仪器这一种量法)");
      end if;
      return;
   end if;
   declare
      Area : Natural;
      Score : Long_Float;
      Ok : Boolean;
      Err : Unbounded_String;
      On_Pts : Instrument.Seg_Pt_Vectors.Vector;
   begin
      if Pu_On >= 0.0 and then Pv_On >= 0.0 and then Pu_On < Long_Float (Cw) and then Pv_On < Long_Float (Ch) then
         On_Pts.Append (Instrument.Seg_Pt'(U => Pu_On, V => Pv_On, On => True));
      end if;
      Instrument.Segment (To_String (C.Inst_Host), C.Inst_Port, F.Cams (Cam).RGB, Cw, Ch, X0, Y0, X1, Y1, On_Pts, M, Area, Score, Ok, Err);
      if not Ok or else Area = 0 then
         Got := False; Iso := False; R := (others => <>);
         if not Ok then
            Put_Line ("[身] 📦 分割仪器没回来(" & To_String (Err) & ")");
         end if;
         return;
      end if;
      Picture.Region_Of_Mask (M, Cw, Ch, R, Got);
      Iso := Got and then R.X0 > 0 and then R.Y0 > 0 and then R.X1 + 1 < Cw and then R.Y1 + 1 < Ch;
   end;
end Seg_In_Box;
