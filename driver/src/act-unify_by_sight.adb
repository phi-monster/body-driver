separate (Act)
procedure Unify_By_Sight (C : in out Context; F : Plug.Frame; Cam : Natural; W : String; R : Picture.Region) is
   G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
   A1 : constant Integer := Cam_Arm (C, Cam);
   Kw : constant Natural := F.Cams (Cam).W;
   Kh : constant Natural := F.Cams (Cam).H;
   Ok1 : Boolean := False;
   S1 : Geom.Sight;
begin
   if A1 < 0 and then G.Fixed then
      S1 := (O => G.Pos, D => Geom.Ray_Fixed (G, R.Cu * Long_Float (Kw), R.Cv * Long_Float (Kh)));
      Ok1 := True;
   elsif A1 >= 0 and then G.Valid and then G.F > 0.0 and then A1 < Integer (F.EE.Length) then
      declare
         P : constant Plug.Arm_Pose := F.EE (Natural (A1));
      begin
         S1 := (O => Geom.Cam_Pos (G, P), D => Geom.Ray (G, P, R.Cu * Long_Float (Kw), R.Cv * Long_Float (Kh)));
         Ok1 := True;
      end;
   end if;
   if not Ok1 then
      return;
   end if;
   for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
      declare
         B : constant Boxed_Thing := C.Boxed (Bi);
      begin
         if B.Cam /= Cam and then B.Seen and then To_String (B.Name) /= W and then B.Cam < Natural (C.Geo.Length) and then B.Cam < Natural (F.Cams.Length) then
            declare
               Gm : constant Geom.Cam_Geo := C.Geo (B.Cam);
               A2 : constant Integer := Cam_Arm (C, B.Cam);
               W2 : constant Natural := F.Cams (B.Cam).W;
               H2 : constant Natural := F.Cams (B.Cam).H;
               S2 : Geom.Sight;
               Ok2 : Boolean := False;
            begin
               if A2 < 0 and then Gm.Fixed then
                  S2 := (O => Gm.Pos, D => Geom.Ray_Fixed (Gm, B.Cu * Long_Float (W2), B.Cv * Long_Float (H2)));
                  Ok2 := True;
               elsif A2 >= 0 and then Gm.Valid and then Gm.F > 0.0 and then A2 < Integer (F.EE.Length) then
                  declare
                     P2 : constant Plug.Arm_Pose := F.EE (Natural (A2));
                  begin
                     S2 := (O => Geom.Cam_Pos (Gm, P2), D => Geom.Ray (Gm, P2, B.Cu * Long_Float (W2), B.Cv * Long_Float (H2)));
                     Ok2 := True;
                  end;
               end if;
               if Ok2 then
                  declare
                     Rays : Geom.Sight_Vectors.Vector;
                     Mok : Boolean;
                     Spread : Long_Float;
                     Pm : Geom.V3;
                  begin
                     Rays.Append (S1);
                     Rays.Append (S2);
                     Pm := Geom.Meet (Rays, Mok, Spread);
                     if Mok then
                        declare
                           --  那块东西自己有多大(米):它在那只眼里框的对角线 × 那只眼到交点的距离 ÷ 焦距(全是量出来的);两条视线差得不超过它的一半(纯比例)就是同一件
                           D2 : constant Long_Float := Geom.Norm ([Pm (0) - S2.O (0), Pm (1) - S2.O (1), Pm (2) - S2.O (2)]);
                           Diag : constant Long_Float := Sqrt (Long_Float (B.X1 - B.X0 + 1) ** 2 + Long_Float (B.Y1 - B.Y0 + 1) ** 2);
                           Size : constant Long_Float := (if Gm.F > 0.0 then Diag * D2 / Gm.F else 0.0);
                        begin
                           if Spread <= 0.5 * Size then
                              declare
                                 Old : constant String := To_String (B.Name);
                                 B2 : Boxed_Thing := B;
                              begin
                                 Put_Line ("[身] 📦 第" & Codec.Img (B.Cam) & " 台里你叫「" & Old & "」的和这只眼里你叫「" & W & "」的,视线交在一点(偏差 "
                                           & Mm (Spread) & ",它本身约 " & Mm (Size) & ")⇒ 同一件,以后都叫它「" & W & "」");
                                 B2.Name := To_Unbounded_String (W);
                                 C.Boxed.Replace_Element (Bi, B2);
                                 if To_String (C.Geo_Pw_Name) = Old then
                                    C.Geo_Pw_Name := To_Unbounded_String (W);
                                 end if;
                                 if To_String (C.Geo_Name) = Old then
                                    C.Geo_Name := To_Unbounded_String (W);
                                 end if;
                                 if To_String (C.Sil_Name) = Old then
                                    C.Sil_Name := To_Unbounded_String (W);
                                 end if;
                              end;
                           end if;
                        end;
                     end if;
                  end;
               end if;
            end;
         end if;
      end;
   end loop;
end Unify_By_Sight;
