separate (Act)
procedure Head_Tip_Check (C : Context; A, Hc : Natural; Tips : Geom.V3_Vectors.Vector) is
   Wc : constant Natural := C.Map.World_Cam;
   package Sorting is new F64_Vectors.Generic_Sorting;
   procedure Report (Name : String; E : in out Floats) is
      Within : Natural := 0;
      V1_Line : constant Long_Float := 2.0;   --  V1 验收线 2 px(PLAN.md §1 协议里的判据;只数一数,不当门)
   begin
      if E.Is_Empty then
         Geo_Say ("  对账(头顶眼按指尖,V1 口径):" & Name & " 一笔都没有");
         return;
      end if;
      Sorting.Sort (E);
      for X of E loop
         if X <= V1_Line then
            Within := Within + 1;
         end if;
      end loop;
      Geo_Say ("  对账(头顶眼按指尖,V1 口径):它开机时标的" & Name & " " & Codec.Img (Natural (E.Length)) & " 笔,离碰出来的指尖投进它眼里的那一点 中位 "
               & Codec.Fmt (E (Natural (E.Length) / 2), 2) & " px、最大 " & Codec.Fmt (E (Natural (E.Length) - 1), 1) & " px,2 px 内 " & Codec.Img (Within) & " 笔");
   end Report;
begin
   if Wc >= Natural (C.Geo.Length) or else not (C.Geo (Wc).Valid and then C.Geo (Wc).Fixed) or else Hc >= Natural (C.Geo.Length) then
      return;
   end if;
   declare
      Gw : constant Geom.Cam_Geo := C.Geo (Wc);
      G : constant Geom.Cam_Geo := C.Geo (Hc);
      El, Em, Eo : Floats;
      function Tip_At (P : Plug.Arm_Pose; K : Natural) return Geom.V3 is
         O : constant Geom.V3 := Geom.Cam_Pos (G, P);
         T : constant Geom.V3 := Geom.Ap (Geom.Cam_R (G, P), Tips (K));
      begin
         return [O (0) + T (0), O (1) + T (1), O (2) + T (2)];
      end Tip_At;
   begin
      for Ob of C.Lobe_Obs loop
         if Ob.Pt = A then
            declare
               Best : Long_Float := Long_Float'Last;
            begin
               for K in 0 .. Natural (Tips.Length) - 1 loop
                  declare
                     U, V : Long_Float;
                     Front : Boolean;
                  begin
                     Geom.Project_Fixed (Gw, Tip_At (Ob.Pose, K), U, V, Front);
                     if Front then
                        Best := Long_Float'Min (Best, Sqrt ((U - Ob.U) ** 2 + (V - Ob.V) ** 2));
                     end if;
                  end;
               end loop;
               if Best < Long_Float'Last then
                  El.Append (Best);
               end if;
            end;
         end if;
      end loop;
      for I in 0 .. Natural (C.Fixed_Obs.Length) - 1 loop
         declare
            Ob : constant Geom.Obs_Pt := C.Fixed_Obs (I);
         begin
            if Ob.Pt = A then
               declare
                  U, V : Long_Float;
                  Front : Boolean;
               begin
                  Geom.Project_Fixed (Gw, Tip_World (C, A, Ob.Pose), U, V, Front);
                  if Front then
                     Em.Append (Sqrt ((U - Ob.U) ** 2 + (V - Ob.V) ** 2));
                  end if;
               end;
               --  碰出来的每一瓣指尖投进它眼里,离那一笔里它看见的手指像素最近多远(落在手指上 = 0):不比分割出来的"尖"那一点(远处小夹爪的尖一会儿一个样)
               if I < Natural (C.Mark_Px.Length) and then not C.Mark_Px (I).Is_Empty then
                  for K in 0 .. Natural (Tips.Length) - 1 loop
                     declare
                        U, V : Long_Float;
                        Front : Boolean;
                        Best : Long_Float := Long_Float'Last;
                     begin
                        Geom.Project_Fixed (Gw, Tip_At (Ob.Pose, K), U, V, Front);
                        if Front then
                           for P of C.Mark_Px (I) loop
                              Best := Long_Float'Min (Best, Sqrt ((Long_Float (P.U) - U) ** 2 + (Long_Float (P.V) - V) ** 2));
                           end loop;
                           if Best < Long_Float'Last then
                              Eo.Append (Best);
                           end if;
                        end if;
                     end;
                  end loop;
               end if;
            end if;
         end;
      end loop;
      Report ("每一瓣的尖", El);
      Report ("各瓣的中点", Em);
      Report ("手指像素(碰出来的每一瓣指尖离它最近多远)", Eo);
   end;
end Head_Tip_Check;
