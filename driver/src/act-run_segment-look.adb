separate (Act.Run_Segment)
procedure Look is
   Need_Refind : Boolean := False;
begin
   for I in 0 .. Natural (Pts.Length) - 1 loop
      declare
         P : Point := Pts (I);
         W0 : constant Point := Was (I);
         Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Got);
      begin
         P.Has_Meas := False;
         if P.Kind = Piece_Pt and then Cam_Arm (C, P.Cam) /= Integer (P.Arm) then
            declare
               Diff : Table.Vec;
               Dist : Long_Float;
               Si : constant Integer := Schema.Nearest (C.Sch, P.Arm, Cam, F.EE (P.Arm), C.Map.Amp, Chan.Per_Arm, Diff, Dist);
               Familiar : Boolean := False;
               In_Map : Boolean := False;
            begin
               if Si >= 0 then
                  In_Map := P.Chan_K <= Chan.Per_Arm and then C.Sch.S (Natural (Si)).Parts (P.Chan_K).Valid
                            and then (P.Blob < 0 or else C.Sch.S (Natural (Si)).Parts (P.Chan_K).N_Blobs > Natural (P.Blob));
               end if;
               if In_Map then
                  declare
                     Gp : constant Schema.Part_Pos := C.Sch.S (Natural (Si)).Parts (P.Chan_K);
                     Pm : constant Table.Vec3 := Table.Predict (Effs (I), Diff);
                     Su : constant Long_Float := (if P.Blob = 1 then Gp.B1u elsif P.Blob = 0 then Gp.B0u else Gp.Cu);
                     Sv : constant Long_Float := (if P.Blob = 1 then Gp.B1v elsif P.Blob = 0 then Gp.B0v else Gp.Cv);
                  begin
                     P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, Su + Pm (0)));
                     P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, Sv + Pm (1)));
                     --  🔴 距离不可能是负的(物理,不是人拍的门槛)。这里只检查了【旧】深度是正的,
                     --  没检查【算出来的新】深度 —— 画面坐标 u/v 都夹在 [0,1] 里,唯独深度一个夹子都没有。
                     --  GV 实测:预测把它推成 -0.526 ⇒ 远近整行作废 ⇒ 身体只在画面上对齐、
                     --  停在离球 0.08 画幅处还报"差 0.005 m"。(同一个坑 LAB 记过:ca3641b。)
                     if Gp.Z > 0.0 and then Gp.Z + Pm (2) > 0.0 then
                        P.Z := Gp.Z + Pm (2);
                     end if;
                     Familiar := True;
                     for K in 0 .. Chan.Per_Arm - 1 loop
                        if abs Diff (K) > Long_Float'Max (1.0e-6, C.Map.Amp (P.Arm * Chan.Per_Arm + K)) * Cap_Mult * Reach (K) then
                           Familiar := False;
                        end if;
                     end loop;
                  end;
               else
                  P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, W0.Cu + Pr (0)));
                  P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, W0.Cv + Pr (1)));
                  --  同上:算出来的新深度必须仍是正的,否则这一步的预测就是错的,宁可留着旧值
                  if W0.Z > 0.0 and then W0.Z + Pr (2) > 0.0 then
                     P.Z := W0.Z + Pr (2);
                  end if;
               end if;
               --  🔴🔴 位置从姿态表里查出来之后,【深度要在深度图上就地重读】。
               --  以前这一路的 Z 全是姿态表里存的那个数加上表的预测 —— 也就是【猜】出来的,
               --  从来没被眼睛校过。IY 实测(真深度也开着):球读 0.640 m,而挨着它的指尖读 2.40 m,
               --  差了四倍;每一步 Z 平滑地变 0.007,像预测不像测量。于是"远近"那一行永远差着,
               --  手在前后方向上要么不动要么一路顶,合手全是空的。
               --  LAB 判定这就是 FO"夹太靠上、一合把球顶飞"的根子。修法(4c24742 + 闸 ad6d76e)
               --  在 a7ab7e9 回滚里被一起退掉了,这里捞回来。
               if F.Cams (P.Cam).Has_Depth then
                  declare
                     Zn : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, P.Cam);
                     --  🔴 读"我离相机多远"要读在【我这一瓣自己身上】。
                     --  区心是【两指之间的空】,那儿什么都没有,读到的是它背后的东西 ——
                     --  HW 实测:爪子读 2.19 m 而球读 3.53 m,差了 1.34 m;桌面上不可能有这么大的高度差,
                     --  是读窗落在空处、读到了更靠近相机的自己的大臂。
                     --  (LAB 3b9d570 原话:"区心是两指之间的空,读到的是桌面"。)
                     --  瓣是量出来的:一瓣=吸盘,两瓣=两指,七瓣=七指,这里取第一瓣的位置,零身体假设。
                     Lb : constant Zone.Lobe := Zone.Lobe_Of (Zn, 0);
                     Ru : constant Long_Float := (if P.Kind = Piece_Pt and then P.Blob < 0
                                                  and then Zn.Valid and then Lb.Valid then Lb.Cu else P.Cu);
                     Rv : constant Long_Float := (if P.Kind = Piece_Pt and then P.Blob < 0
                                                  and then Zn.Valid and then Lb.Valid then Lb.Cv else P.Cv);
                     Zd : constant Long_Float :=
                       Picture.Near_Depth (F.Cams (P.Cam).Depth, F.Cams (P.Cam).W, F.Cams (P.Cam).H,
                                           Ru, Rv, Lobe_Win (Zn, F.Cams (P.Cam).W, F.Cams (P.Cam).H));
                     --  🔴 闸盯【上一次真读到的】远近,不是 P.Z —— P.Z 可能是按位姿猜的、从没被眼睛校过
                     Old_Z : constant Long_Float := (if P.Z_Seen > 0.0 then P.Z_Seen else P.Z);
                  begin
                     --  🔴 收读数前先过闸:读窗里同时有指头和它【后面那个面】时,读数会在两者之间来回跳
                     --  (JD 实测:指尖深度在 0.61 和 0.45 之间几乎每步翻一次,差 16 cm,而它在画面里几乎没动
                     --   ⇒ 前后那一维的误差每步翻符号 ⇒ 手被拉过去又拉回来,视频里就是发癫)。
                     --  原版这道闸写的是"表预测的变化 + 距离的【一成】",那个一成是人拍的;
                     --  换成这一点自己量到的深度抖动地板(Z_Noise),零系数,而且比一成更对。
                     if not Picture.Is_Nan (Zd) and then Zd > 0.0 then
                        if Depth_Ok (Zd, Old_Z, Old_Z + Pr (2), P.Z_Noise,
                                     (if P.At_Edge then 0.0 else P.Z_Rej))
                        then
                           P.Z := Zd; P.Z_Seen := Zd; P.Z_Rej := 0.0;
                        else
                           --  🔴 打出来:读到多少、上次真读到多少、表预测这一步走多少、抖动多少、上次被拒的是多少。
                           --  HR 实测:深度 8 推纹丝不动 2.062,而点在画面里确实在动 ⇒ 每一读都被挡,
                           --  光看"差 1.498 m"看不出是挡的还是真没动。
                           Put_Line ("[身]     深度被挡:读到 " & Codec.Fmt (Zd, 3)
                                     & " · 上次真读到 " & Codec.Fmt (Old_Z, 3)
                                     & " · 表说这一步走 " & Codec.Fmt (Pr (2), 3)
                                     & " · 这一点读深抖动 " & Codec.Fmt (P.Z_Noise, 3)
                                     & " · 上次被拒 " & Codec.Fmt (P.Z_Rej, 3)
                                     & (if P.At_Edge then " · 此刻贴在画面边上(出路不给)" else ""));
                           P.Z_Rej := Zd;   --  记下被拒的那个数;连着两次一致就说明旧基准陈了
                           P.Z := Old_Z;    --  这一帧读到的是别的面,留上一次真读到的
                        end if;
                     end if;
                  end;
               end if;
               if Note.Big_Step or else not Familiar then
                  P.Lost := True;
                  Need_Refind := True;
               else
                  declare
                     Q : Point := W0;
                     Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, P.Cam, Jaw_K_Of (P.Chan_K));
                  begin
                     Retrack (C, F, P.Cam, Before_All (P.Cam), Q, W0.Cu + Pr (0), W0.Cv + Pr (1), True, (if W0.Z > 0.0 then W0.Z + Pr (2) else -1.0));
                     --  眼睛和图对不上(差过张幅的四分之一,比例,无量纲;再小也有两个跟踪地板)⇒ 去看
                     if Q.Lost or else Sqrt ((Q.Cu - P.Cu) ** 2 + (Q.Cv - P.Cv) ** 2) > Long_Float'Max (Z.Span * 0.25, Fl.Track * 2.0) then
                        P.Lost := True;
                        Need_Refind := True;
                     else
                        P.Lost := False;
                        P.Has_Meas := True; P.Meas_U := Q.Cu; P.Meas_V := Q.Cv; P.Meas_Z := Q.Z;
                     end if;
                  end;
               end if;
            end;
         else
            Retrack (C, F, P.Cam, Before_All (P.Cam), P, W0.Cu + Pr (0), W0.Cv + Pr (1), True, (if W0.Z > 0.0 then W0.Z + Pr (2) else -1.0));
         end if;
         --  🔴 认错了东西要说出来,不许悄悄换目标。
         --  GM 实测:被跟的那块从 (0.44,0.93) 深 0.81 m 一步跳到 (0.21,0.60) 深 2.13 m —— 那是球后面的墙。
         --  身体自己知道(信表从 0.94 掉到 0.31),脑一个字没听到,然后追着墙把关节顶死 60 步。
         --  判据不用新系数,用【物理上不可能】:这一步就算把额度用满,表说这块最多能跑多远?
         --  跑得比那还远 ⇒ 不是同一个东西。
         if not P.Lost and then P.Z > 0.0 and then W0.Z > 0.0 then
            declare
               Most : constant Table.Vec3 := Table.Predict (Effs (I), Note.Cap);
               Jump : constant Long_Float := abs (P.Z - W0.Z);
            begin
               if Jump > abs (Most (2)) + Long_Float'Max (0.0, P.Z_Noise) then
                  C.Blind_Say := S ("the thing I am tracking jumped further in one push than any push of mine could move it"
                                    & " - I have probably locked onto something else, and I kept going");
                  Put_Line ("[身]     认错了?这一步它跑了 " & Mm (Jump) & ",而用满额度最多也只跑得动 "
                            & Mm (abs (Most (2))) & "(读深抖动 " & Mm (P.Z_Noise) & ")");
               end if;
            end;
         end if;
         Pts.Replace_Element (I, P);
      end;
   end loop;
   if Need_Refind then
      Refind_Pieces (L, C, F, Cam, Pts);
      Beats := Since (L, Beats0);
   end if;
   Note.Lost_All := True;
   for P of Pts loop
      if not P.Lost then
         Note.Lost_All := False;
      end if;
      if P.Unsure then
         Note.Unsure := True;
      end if;
   end loop;
end Look;
