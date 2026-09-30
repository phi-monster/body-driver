separate (Geom)
procedure Load (Path : String; Gs : in out Geo_Vectors.Vector; N_Cams : Natural; Note : out String) is
   Fh : Ada.Text_IO.File_Type;
   Src : Unbounded_String;
   D : Json.Doc;
   Err : Unbounded_String;
   procedure Put_Note (S : String) is
   begin
      Note := [others => ' '];
      Note (Note'First .. Note'First + Integer'Min (S'Length, Note'Length) - 1) := S (S'First .. S'First + Integer'Min (S'Length, Note'Length) - 1);
   end Put_Note;
begin
   Gs.Clear;
   for K in 0 .. N_Cams - 1 loop
      Gs.Append (No_Geo);
   end loop;
   begin
      Ada.Text_IO.Open (Fh, Ada.Text_IO.In_File, Path);
   exception
      when others =>
         Put_Note ("没有几何文件,从零量");
         return;
   end;
   while not Ada.Text_IO.End_Of_File (Fh) loop
      Append (Src, Ada.Text_IO.Get_Line (Fh));
   end loop;
   Ada.Text_IO.Close (Fh);
   if not Json.Parse (To_String (Src), D, Err) then
      Put_Note ("几何文件读不回:" & To_String (Err));
      return;
   end if;
   declare
      Cams : constant Integer := Json.Get (D, 0, "cams");
      Loaded : Natural := 0;
   begin
      if Cams < 0 then
         Put_Note ("几何文件里没有 cams");
         return;
      end if;
      for I in 0 .. Json.Count (D, Cams) - 1 loop
         declare
            Nd : constant Integer := Json.Child (D, Cams, I);
            K : constant Integer := Integer (Json.Num (D, Json.Get (D, Nd, "cam")));
            G : Cam_Geo;
            Rn : constant Integer := Json.Get (D, Nd, "r_ce");
            Tn : constant Integer := Json.Get (D, Nd, "tip");
         begin
            if K >= 0 and then K < N_Cams then
               G.Valid := Json.Bool (D, Json.Get (D, Nd, "valid"));
               G.F := Json.Num (D, Json.Get (D, Nd, "f")); G.Cx := Json.Num (D, Json.Get (D, Nd, "cx")); G.Cy := Json.Num (D, Json.Get (D, Nd, "cy"));
               G.K1 := Json.Num (D, Json.Get (D, Nd, "k1")); G.K2 := Json.Num (D, Json.Get (D, Nd, "k2"));   --  旧文件没有 ⇒ 0(理想针孔)
               G.Rms := Json.Num (D, Json.Get (D, Nd, "rms"));
               if Rn >= 0 and then Json.Count (D, Rn) = 9 then
                  for A in 0 .. 2 loop
                     for B in 0 .. 2 loop
                        G.R_Ce (A, B) := Json.Num (D, Json.Child (D, Rn, A * 3 + B));
                     end loop;
                  end loop;
               else
                  G.Valid := False;
               end if;
               G.Tip_Valid := Json.Bool (D, Json.Get (D, Nd, "tip_valid"));
               G.Tip_Touch := Json.Bool (D, Json.Get (D, Nd, "tip_touch"));   --  旧文件没有 ⇒ False(按头顶眼交的,不算)
               if Tn >= 0 and then Json.Count (D, Tn) = 3 then
                  for A in 0 .. 2 loop
                     G.Tip (A) := Json.Num (D, Json.Child (D, Tn, A));
                  end loop;
               else
                  G.Tip_Valid := False;
               end if;
               G.Gap := Json.Num (D, Json.Get (D, Nd, "gap"));
               --  每一瓣的尖(09-29 起存;老文件没有 ⇒ 空,接触集照实说"没量每一瓣")
               declare
                  Ln : constant Integer := Json.Get (D, Nd, "lobes");
               begin
                  G.Tip_Sd := Json.Num (D, Json.Get (D, Nd, "tip_sd"));
                  if Ln >= 0 then
                     for Li in 0 .. Json.Count (D, Ln) - 1 loop
                        declare
                           Lnd : constant Integer := Json.Child (D, Ln, Li);
                           Lt : constant Integer := Json.Get (D, Lnd, "tip");
                           Lg : Lobe_Geo;
                        begin
                           if Lt >= 0 and then Json.Count (D, Lt) = 3 then
                              for A in 0 .. 2 loop
                                 Lg.Tip (A) := Json.Num (D, Json.Child (D, Lt, A));
                              end loop;
                              Lg.Wide := Json.Num (D, Json.Get (D, Lnd, "wide"));
                              Lg.Thin := Json.Num (D, Json.Get (D, Lnd, "thin"));
                              G.Lobes.Append (Lg);
                           end if;
                        end;
                     end loop;
                  end if;
               end;
               declare
                  Sn : constant Integer := Json.Get (D, Nd, "stride");   --  老文件没有这一项 ⇒ 0,开机再量
                  Sr : constant Integer := Json.Get (D, Nd, "stride_rot");
                  Fm : constant Integer := Json.Get (D, Nd, "f_meas");
                  Fp : constant Integer := Json.Get (D, Nd, "f_prior");
                  Fs : constant Integer := Json.Get (D, Nd, "f_prior_sd");
               begin
                  if Sn >= 0 then
                     G.Stride := Json.Num (D, Sn);
                  end if;
                  if Sr >= 0 then
                     G.Stride_Rot := Json.Num (D, Sr);
                  end if;
                  if Fm >= 0 then
                     G.F_Meas := Json.Num (D, Fm);
                  end if;
                  if Fp >= 0 and then Fs >= 0 then
                     G.F_Prior := Json.Num (D, Fp); G.F_Prior_Sd := Json.Num (D, Fs);
                  end if;
                  declare
                     On : constant Integer := Json.Get (D, Nd, "off");   --  老文件没有 ⇒ 0
                  begin
                     if On >= 0 and then Json.Count (D, On) = 3 then
                        for A in 0 .. 2 loop
                           G.Off (A) := Json.Num (D, Json.Child (D, On, A));
                        end loop;
                     end if;
                  end;
               end;
               declare
                  Fx : constant Integer := Json.Get (D, Nd, "fixed");
                  Pn : constant Integer := Json.Get (D, Nd, "pos");
               begin
                  G.Fixed := Fx >= 0 and then Json.Bool (D, Fx) and then Pn >= 0 and then Json.Count (D, Pn) = 3;
                  if G.Fixed then
                     for A in 0 .. 2 loop
                        G.Pos (A) := Json.Num (D, Json.Child (D, Pn, A));
                     end loop;
                  end if;
               end;
               Gs.Replace_Element (K, G);
               Loaded := Loaded + 1;
            end if;
         end;
      end loop;
      Put_Note ("装回几何文件:" & Codec.Img (Loaded) & " 台相机");
   end;
end Load;
