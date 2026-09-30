separate (Geom)
procedure Save (Path : String; Gs : Geo_Vectors.Vector) is
   Fh : Ada.Text_IO.File_Type;
   B : Unbounded_String;
begin
   Append (B, "{""cams"":[");
   for K in 0 .. Natural (Gs.Length) - 1 loop
      declare
         G : constant Cam_Geo := Gs (K);
      begin
         if K > 0 then
            Append (B, ",");
         end if;
         Append (B, "{""cam"":" & Codec.Img (K) & ",""valid"":" & (if G.Valid then "true" else "false") &
                   ",""f"":" & Codec.Fmt (G.F, 4) & ",""cx"":" & Codec.Fmt (G.Cx, 3) & ",""cy"":" & Codec.Fmt (G.Cy, 3) & ",""k1"":" & Codec.Fmt (G.K1, 6)
                   & ",""k2"":" & Codec.Fmt (G.K2, 6) & ",""rms"":" & Codec.Fmt (G.Rms, 3) & ",""r_ce"":[");
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Append (B, (if I + J > 0 then "," else "") & Codec.Fmt (G.R_Ce (I, J), 7));
            end loop;
         end loop;
         Append (B, "],""tip_valid"":" & (if G.Tip_Valid then "true" else "false") & ",""tip_touch"":" & (if G.Tip_Touch then "true" else "false") & ",""tip"":[" & Codec.Fmt (G.Tip (0), 5) & "," & Codec.Fmt (G.Tip (1), 5) & "," &
                   Codec.Fmt (G.Tip (2), 5) & "],""gap"":" & Codec.Fmt (G.Gap, 5) & ",""stride"":" & Codec.Fmt (G.Stride, 5) & ",""stride_rot"":" & Codec.Fmt (G.Stride_Rot, 5) &
                   ",""f_meas"":" & Codec.Fmt (G.F_Meas, 3) & ",""f_prior"":" & Codec.Fmt (G.F_Prior, 3) & ",""f_prior_sd"":" & Codec.Fmt (G.F_Prior_Sd, 3) &
                   ",""off"":[" & Codec.Fmt (G.Off (0), 5) & "," & Codec.Fmt (G.Off (1), 5) & "," & Codec.Fmt (G.Off (2), 5) & "]" &
                   ",""fixed"":" & (if G.Fixed then "true" else "false") & ",""pos"":[" & Codec.Fmt (G.Pos (0), 5) & "," & Codec.Fmt (G.Pos (1), 5) & "," & Codec.Fmt (G.Pos (2), 5) & "]"
                   & ",""tip_sd"":" & Codec.Fmt (G.Tip_Sd, 6) & ",""lobes"":[");
         for Li in 0 .. Natural (G.Lobes.Length) - 1 loop
            declare
               Lg : constant Lobe_Geo := G.Lobes (Li);
            begin
               Append (B, (if Li > 0 then "," else "") & "{""tip"":[" & Codec.Fmt (Lg.Tip (0), 6) & "," & Codec.Fmt (Lg.Tip (1), 6) & "," & Codec.Fmt (Lg.Tip (2), 6)
                       & "],""wide"":" & Codec.Fmt (Lg.Wide, 6) & ",""thin"":" & Codec.Fmt (Lg.Thin, 6) & "}");
            end;
         end loop;
         Append (B, "]}");
      end;
   end loop;
   Append (B, "]}");
   Ada.Text_IO.Create (Fh, Ada.Text_IO.Out_File, Path);
   Ada.Text_IO.Put_Line (Fh, To_String (B));
   Ada.Text_IO.Close (Fh);
end Save;
