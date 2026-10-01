separate (Act)
procedure Feel (C : in out Context; F : Plug.Frame) is
   function Clamp (X : Long_Float) return Long_Float is (Long_Float'Max (0.0, Long_Float'Min (1.0, X)));
begin
   for A in 0 .. C.Map.Arms - 1 loop
      for Cm in 0 .. C.Map.N_Cams - 1 loop
         if Cam_Arm (C, Cm) /= Integer (A) and then A < Natural (F.EE.Length) and then Track_Idx (C, A, Cm) < Natural (C.Zones.Length) then
            declare
               Diff : Table.Vec;
               Dist : Long_Float;
               Si : constant Integer := Schema.Nearest (C.Sch, A, Cm, F.EE (A), C.Map.Amp, Chan.Per_Arm, Diff, Dist);
            begin
               if Si >= 0 then
                  declare
                     Sm : constant Schema.Sample := C.Sch.S (Natural (Si));
                     Tr : Zone_Track := C.Zones (Track_Idx (C, A, Cm));
                     Reach : Table.Vec := Unit_Reach;
                     Gp : constant Schema.Part_Pos := Sm.Parts (Chan.Per_Arm);   --  握合通道带的那块 = 手指
                     function Shift (Blob : Integer) return Table.Vec3 is
                        Idx : Integer := Find_Effect (C, A, Cm, Piece_Pt, Chan.Per_Arm, Blob);
                     begin
                        if Idx < 0 then
                           Idx := Find_Effect (C, A, Cm, Piece_Pt, Chan.Per_Arm, -1);
                        end if;
                        if Idx >= 0 then
                           for K in 0 .. Chan.Per_Arm - 1 loop
                              Reach (K) := Long_Float'Max (Reach (K), C.Tables (Natural (Idx)).Reach (K));
                           end loop;
                           return Table.Predict (C.Tables (Natural (Idx)).E, Diff);
                        end if;
                        return Table.Zero3;
                     end Shift;
                     Sa : constant Table.Vec3 := Shift (0);
                     Sb : constant Table.Vec3 := Shift (1);
                  begin
                     if Gp.Valid then
                        Tr.Valid := True;
                        --  🔴🔴 外推炸了就不许用,退回样本里存的原样,并且不许再自称"我知道"。
                        --  箱上真数据(cal.json,臂1/相机0,64 个样本)存的是:两瓣 u = 0.8475 / 0.9847,
                        --  隔 0.137,都在画面右边 —— 样本是对的。而身体报给脑的是"左边,第 2 格和第 19 格,
                        --  隔四分之三个画面"。坏在这一行:胳膊离样本远时 Sa/Sb 这个外推量会炸,
                        --  Clamp 把它夹到 0 或 1 ⇒ 一瓣被夹到画面最左、另一瓣在别处。
                        --  而"我知不知道"只看【位姿差多大】,不看【算出来的结果合不合理】⇒
                        --  它一边给荒谬的位置一边说"我知道",十三炮的伺服全是拿这个位置算的。
                        --  判据零系数:算出来的两瓣间距,和【样本里那两瓣本来隔多远】比;
                        --  差得比它本身还大 ⇒ 这次外推不作数。
                        declare
                           Pu0 : constant Long_Float := Gp.B0u + Sa (0);
                           Pv0 : constant Long_Float := Gp.B0v + Sa (1);
                           Pu1 : constant Long_Float := Gp.B1u + Sb (0);
                           Pv1 : constant Long_Float := Gp.B1v + Sb (1);
                           Was : constant Long_Float :=
                             Sqrt ((Gp.B0u - Gp.B1u) ** 2 + (Gp.B0v - Gp.B1v) ** 2);
                           Now : constant Long_Float := Sqrt ((Pu0 - Pu1) ** 2 + (Pv0 - Pv1) ** 2);
                           Blew : constant Boolean :=
                             Gp.N_Blobs >= 2 and then Extrapolation_Blew (Was, Now);
                        begin
                           if Blew then
                              --  外推不作数:用样本里的原样,并在下面把 Known 判掉(身体会因此先看一眼)
                              Tr.Au := Gp.B0u; Tr.Av := Gp.B0v;
                              Tr.Bu := Gp.B1u; Tr.Bv := Gp.B1v;
                           else
                              Tr.Au := Clamp (Pu0); Tr.Av := Clamp (Pv0);
                              Tr.Bu := Clamp (Pu1); Tr.Bv := Clamp (Pv1);
                           end if;
                           Tr.Blew_Up := Blew;
                        end;
                        Tr.Has_Lobes := Gp.N_Blobs >= 1;
                        if Gp.N_Blobs >= 2 then
                           Tr.Cu := (Tr.Au + Tr.Bu) / 2.0; Tr.Cv := (Tr.Av + Tr.Bv) / 2.0;
                        else
                           Tr.Cu := Tr.Au; Tr.Cv := Tr.Av;
                        end if;
                        if Gp.Z > 0.0 then
                           Tr.Z := Gp.Z + (if Gp.N_Blobs >= 2 then (Sa (2) + Sb (2)) / 2.0 else Sa (2));
                        end if;
                        Tr.Known := not Tr.Blew_Up;   --  外推炸过 ⇒ 这一处的位置不作数,别再自称知道
                        for K in 0 .. Chan.Per_Arm - 1 loop
                           if abs Diff (K) > Long_Float'Max (1.0e-6, C.Map.Amp (A * Chan.Per_Arm + K)) * Cap_Mult * Reach (K) then
                              Tr.Known := False;
                           end if;
                        end loop;
                        Tr.Pieces (Chan.Per_Arm) := (True, Tr.Cu, Tr.Cv, Tr.Z, Gp.X0, Gp.Y0, Gp.X1, Gp.Y1, Gp.N_Blobs, Tr.Au, Tr.Av, Tr.Bu, Tr.Bv);
                        Tr.Pieces_Known (Chan.Per_Arm) := Tr.Known;
                     end if;
                     --  零件:样本里的位置 + 这个零件自己的响应表外推(没表 ⇒ 只在位姿几乎没差时算"知道")
                     for K in 0 .. Chan.Per_Arm - 1 loop
                        if Sm.Parts (K).Valid then
                           declare
                              Idx : constant Integer := Find_Effect (C, A, Cm, Piece_Pt, K, -1);
                              Sh : constant Table.Vec3 := (if Idx >= 0 then Table.Predict (C.Tables (Natural (Idx)).E, Diff) else Table.Zero3);
                              Pr : Schema.Part_Pos := Sm.Parts (K);
                              Kn : Boolean := True;
                              R2 : constant Table.Vec := (if Idx >= 0 then C.Tables (Natural (Idx)).Reach else Table.Zero_Vec);
                           begin
                              Pr.Cu := Clamp (Sm.Parts (K).Cu + Sh (0)); Pr.Cv := Clamp (Sm.Parts (K).Cv + Sh (1));
                              if Sm.Parts (K).Z > 0.0 then
                                 Pr.Z := Sm.Parts (K).Z + Sh (2);
                              end if;
                              --  框跟着形心平移
                              Pr.X0 := Natural (Long_Float'Max (0.0, Long_Float (Sm.Parts (K).X0) + Sh (0) * Long_Float (F.Cams (Cm).W)));
                              Pr.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (F.Cams (Cm).W - 1), Long_Float (Sm.Parts (K).X1) + Sh (0) * Long_Float (F.Cams (Cm).W))));
                              Pr.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Sm.Parts (K).Y0) + Sh (1) * Long_Float (F.Cams (Cm).H)));
                              Pr.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (F.Cams (Cm).H - 1), Long_Float (Sm.Parts (K).Y1) + Sh (1) * Long_Float (F.Cams (Cm).H))));
                              for J in 0 .. Chan.Per_Arm - 1 loop
                                 if abs Diff (J) > Long_Float'Max (1.0e-6, C.Map.Amp (A * Chan.Per_Arm + J)) * (if Idx >= 0 then Cap_Mult * Long_Float'Max (1.0, R2 (J)) else 1.0) then
                                    Kn := False;
                                 end if;
                              end loop;
                              Tr.Pieces (K) := Pr;
                              Tr.Pieces_Known (K) := Kn;
                           end;
                        end if;
                     end loop;
                     C.Zones.Replace_Element (Track_Idx (C, A, Cm), Tr);
                  end;
               end if;
            end;
         end if;
      end loop;
   end loop;
end Feel;
