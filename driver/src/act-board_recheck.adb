separate (Act)
procedure Board_Recheck (F : Plug.Frame; C : in out Context; Found : out Natural; Said : out Unbounded_String) is
   Wc : constant Natural := C.Map.World_Cam;
begin
   Found := 0; Said := Null_Unbounded_String;
   if C.Board.Is_Empty then
      Said := To_Unbounded_String ("板上没有点");
      return;
   end if;
   if Length (C.Inst_Host) = 0 then
      Said := To_Unbounded_String ("没配配点仪器");
      return;
   end if;
   if C.Fixed_Ref.Is_Empty or else Wc >= Natural (C.Geo.Length) or else Wc >= Natural (F.Cams.Length)
     or else not (C.Geo (Wc).Valid and then C.Geo (Wc).Fixed) or else F.Cams (Wc).W = 0
   then
      Said := To_Unbounded_String ("没有不动的眼(或者它这会儿没有画面)");
      return;
   end if;
   declare
      Q : Instrument.Match_Vectors.Vector;
      Err : Unbounded_String;
      Img : Buf := F.Cams (Wc).RGB;
      W : Natural := F.Cams (Wc).W;
      H : Natural := F.Cams (Wc).H;
      M : Instrument.Match_Vectors.Vector;
      Seen : Bools;
   begin
      for T in 1 .. C.Fixed_Turn loop
         Img := Turn_90 (Img, W, H);
         declare
            W0 : constant Natural := W;
         begin
            W := H; H := W0;
         end;
      end loop;
      for S of C.Board loop
         Q.Append (Instrument.Match_Pt'(U => S.U, V => S.V, Cert => 0.0, others => <>));
      end loop;
      M := Instrument.Match (To_String (C.Inst_Host), C.Inst_Port, C.Fixed_Ref, C.Fixed_Ref_W, C.Fixed_Ref_H, Img, W, H, Q, Err, Back => True);
      if Natural (M.Length) /= Natural (Q.Length) then
         Said := To_Unbounded_String ("仪器没配成(" & To_String (Err) & ")");
         return;
      end if;
      for I in 0 .. Natural (M.Length) - 1 loop
         declare
            Ok : constant Boolean := M (I).U >= 0.0 and then M (I).V >= 0.0 and then M (I).U < Long_Float (W) and then M (I).V < Long_Float (H)
              and then Geom.Round_Trip_Ok (Q (I).U, Q (I).V, M (I).Bu, M (I).Bv);
         begin
            Seen.Append (Ok);
            if Ok then
               Found := Found + 1;
            end if;
         end;
      end loop;
      C.Board_Seen := Seen;
   end;
end Board_Recheck;
