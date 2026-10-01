separate (Act)
function Sightlines_Now (C : in out Context; F : Plug.Frame; Cam, Arm : Natural; Its_Name : Unbounded_String;
                         Seen, Whole : Boolean; U, V : Long_Float; Who : out Unbounded_String; Sds : out Floats) return Geom.Sight_Vectors.Vector is
   Rays : Geom.Sight_Vectors.Vector;
   G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
begin
   Who := Null_Unbounded_String;
   Sds.Clear;
   if Seen and then Whole then
      declare
         P : constant Plug.Arm_Pose := F.EE (Arm);
         Rok : Boolean;
         D : constant Geom.V3 := Geom.Ray (G, P, U, V, Rok);   --  去不了畸变(像素在镜头模型够不到的地方)⇒ 这只眼不给视线
      begin
         if Rok then
            Rays.Append (Geom.Sight'(O => Geom.Cam_Pos (G, P), D => D));
            Sds.Append (Ray_Sd (G));
         end if;
         Append (Who, "第" & Codec.Img (Cam) & " 台");
      end;
   end if;
   if Length (Its_Name) = 0 then
      return Rays;
   end if;
   for Cm in 0 .. C.Map.N_Cams - 1 loop
      if Cm /= Cam and then Cm < Natural (C.Geo.Length) and then Cm < Natural (F.Cams.Length) then
         declare
            Gm : constant Geom.Cam_Geo := C.Geo (Cm);
            A2 : constant Integer := Cam_Arm (C, Cm);
            Usable : constant Boolean := (A2 < 0 and then Gm.Fixed) or else (A2 >= 0 and then Gm.Valid and then Gm.F > 0.0 and then A2 < Integer (F.EE.Length));
         begin
            if Usable then
               --  这只眼这一帧再量一遍它(脑点过名 ⇒ 在上一帧量到它的地方原样重量)
               World.Observe (C.Wld, Cm, Cut_Things (C, F, Cm), F.Cams (Cm).W, F.Cams (Cm).H);
               for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
                  declare
                     B : constant Boxed_Thing := C.Boxed (Bi);
                     Edge : constant Boolean := B.X0 = 0 or else B.Y0 = 0 or else B.X1 + 1 >= F.Cams (Cm).W or else B.Y1 + 1 >= F.Cams (Cm).H;
                  begin
                     if B.Cam = Cm and then B.Seen and then not Edge and then B.Name = Its_Name then
                        declare
                           Pu : constant Long_Float := B.Cu * Long_Float (F.Cams (Cm).W);
                           Pv : constant Long_Float := B.Cv * Long_Float (F.Cams (Cm).H);
                           Hand_On_It : constant Boolean := Hand_Covers (C, F, Arm, Cm, B);
                        begin
                           if Hand_On_It then
                              null;   --  这一眼不给视线
                           elsif A2 < 0 then
                              declare
                                 Rok : Boolean;
                                 D : constant Geom.V3 := Geom.Ray_Fixed (Gm, Pu, Pv, Rok);
                              begin
                                 if Rok then
                                    Rays.Append (Geom.Sight'(O => Gm.Pos, D => D));
                                    Sds.Append (Ray_Sd (Gm));
                                 end if;
                              end;
                           else
                              declare
                                 P2 : constant Plug.Arm_Pose := F.EE (Natural (A2));
                                 Rok : Boolean;
                                 D : constant Geom.V3 := Geom.Ray (Gm, P2, Pu, Pv, Rok);
                              begin
                                 if Rok then
                                    Rays.Append (Geom.Sight'(O => Geom.Cam_Pos (Gm, P2), D => D));
                                    Sds.Append (Ray_Sd (Gm));
                                 end if;
                              end;
                           end if;
                           Append (Who, (if Length (Who) > 0 then "+" else "") & "第" & Codec.Img (Cm) & " 台");
                        end;
                     end if;
                  end;
               end loop;
            end if;
         end;
      end if;
   end loop;
   return Rays;
end Sightlines_Now;
