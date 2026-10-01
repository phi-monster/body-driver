separate (Act)
procedure Retrack (C : in out Context; F : Plug.Frame; Cam : Natural; Before : Buf; P : in out Point; Pred_U, Pred_V : Long_Float; Moved_Arm : Boolean; Pred_Z : Long_Float := -1.0) is
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
begin
   --  这台相机这一拍、或者上一拍没有画面(插头留的空位,09-30):这一拍跟不了,照实记成跟丢,不拿空图去比
   if not Plug.Has_Picture (F.Cams (Cam)) or else Natural (Before.Length) /= Cw * Ch then
      P.Lost := True;
      return;
   end if;
   P.Lost := False;
   case P.Kind is
      when Piece_Pt =>
         if Cam_Arm (C, Cam) = Integer (P.Arm) then
            return;    --  自己的手上相机:握区是固定像素
         end if;
         declare
            --  半分辨率算光流(3 层 30 轮:次数),在这一点一小片取平均位移
            Hw : constant Natural := Cw / 2;
            Hh : constant Natural := Ch / 2;
            A, B : Buf;
            Fl : Flow.Field;
            Du, Dv : Long_Float;
            Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam, Jaw_K_Of (P.Chan_K));
            Old_Z : constant Long_Float := (if P.Z_Seen > 0.0 then P.Z_Seen else P.Z);
         begin
            A.Reserve_Capacity (Ada.Containers.Count_Type (Hw * Hh));
            B.Reserve_Capacity (Ada.Containers.Count_Type (Hw * Hh));
            for Y in 0 .. Hh - 1 loop
               for X in 0 .. Hw - 1 loop
                  A.Append (Before.Element ((2 * Y) * Cw + 2 * X));
                  B.Append (F.Cams (Cam).Gray.Element ((2 * Y) * Cw + 2 * X));
               end loop;
            end loop;
            --  🔴 搜多宽由【这一步预计跑多远】定,不是写死 3 层。
            --  预计位移 = 从上一个位置到预测位置的距离(画幅)× 这半分辨率图的宽(像素)。
            --  预计跑得远 ⇒ 多加几层,最粗那层的位移落到一个像素以内,光流才找得准。
            Fl := Flow.Compute
              (A, B, Hw, Hh,
               Levels_For (Sqrt ((Pred_U - P.Cu) ** 2 + (Pred_V - P.Cv) ** 2) * Long_Float (Hw)),
               30);
            --  取平均的那一片 = 张幅的四分之一(比例,无量纲),再小也有一个像素百分比
            Flow.Sample (Fl, P.Cu, P.Cv, Long_Float'Max (0.01, Z.Span * 0.25), Du, Dv);
            if Moved_Arm and then Sqrt (Du * Du + Dv * Dv) * Long_Float (Cw) < 0.5 then
               P.Cu := Pred_U; P.Cv := Pred_V; P.Lost := True;      --  手臂动了,这儿画面却没流:跟丢的迹象,用预测
            else
               P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cu + Du));
               P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cv + Dv));
            end if;
            if F.Cams (Cam).Has_Depth then
               declare
                  --  深度读在这一瓣自己的位置上(区心是两指之间的空,读到的是桌面);窗口 = 张幅的四分之一(比例,无量纲)
                  Win : constant Long_Float := Long_Float'Max (0.005, Z.Span * 0.25);
                  --  ⚠️ 撤回(HY 实测):曾经在这里除以"量出来的放大倍数"。**那是过头了** ——
                  --  那个倍数量的是"推一米读数变几米"(灵敏度),不是"读数的绝对尺度错几倍"。
                  --  拿灵敏度去除绝对值 ⇒ 1.4 m 被除成 0.003 m(手离镜头 3 毫米,物理上不可能),
                  --  差距当场从 1.366 炸到 2460。倍数只作【自知之明】用,不许改读数。
                  Zd : constant Long_Float :=
                    Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, P.Cu, P.Cv, Win);
               begin
                  if not Picture.Is_Nan (Zd) then
                     --  一步之内深度跳了超过"预测的变化 + 这一点自己的读深抖动"⇒ 读到的不是我的手指,留预测。
                     --  🔴 没有预测值时这道闸以前【整条失效】(Pred_Z <= 0.0 直接短路成真),于是任何读数都收:
                     --  FS 实测手指的"离相机多远"一步从 0.454 m 跳到 0.010 m(离镜头一厘米,物理上不可能),
                     --  抓握的高低判据当场作废。没有预测就退回"一步最多变自己抖动那么多",而不是不管。
                     --  🔴 出路只给【我此刻真看得见自己】的时候用:墙是极其"可重复"的,
                     --  两次读到同一面墙也一致。HF 实测:点飘到画面角落之后,出路把 2.19 m 一路放到 5.109 m
                     --  (场景渲染出来的深度只到 4.359 m,物理上不可能)。跟丢的时候不许走出路。
                     if Depth_Ok (Zd, Old_Z, Pred_Z, P.Z_Noise, (if P.At_Edge then 0.0 else P.Z_Rej)) then
                        P.Z := Zd; P.Z_Seen := Zd; P.Z_Rej := 0.0;
                     else
                        P.Z_Rej := Zd;   --  记下这次被拒的:下一次要是又读到同一个数,就是它对、旧的陈了
                        if Pred_Z > 0.0 then
                           P.Z := Pred_Z;
                        else
                           P.Z := Old_Z;
                        end if;
                     end if;
                  end if;
               end;
            end if;
         end;
      when Thing_Pt =>
         --  重新认脑点名的那一块:除了"位置近、大小差不多",还要"胖瘦像、颜色像"(ER:球 4703 px 和乐高 6334 px 大小分不开,跟错了)。
         --  🔴 认东西是脑的活:两块一样像的时候不许自己挑 —— 记 Unsure,回去问脑要号。
         declare
            Regs : constant Picture.Regions := Cut_Things (C, F, Cam);
            Best, Second : Integer := -1;
            Bd, Sd : Long_Float := 1.0e9;
            Tol : constant Long_Float := Long_Float'Max (P.Box_W, P.Box_H) * 0.75 + Track_Win;
            --  不像的程度:位置差几个跟踪窗 + 大小差几成 + 胖瘦差几成 + 灰度差几成(都是比例,无量纲)。
            --  🔴 大小只参与"像不像",不许当一票否决的硬门槛 —— 越走近它越大,硬门槛会在最该抓住的时候把它判丢(ET 实测)
            function Unlike (R : Picture.Region) return Long_Float is
               D : constant Long_Float := Sqrt ((R.Cu - Pred_U) ** 2 + (R.Cv - Pred_V) ** 2) / Long_Float'Max (Tol, 1.0e-9);
               Sz : constant Long_Float := (if P.Count > 0 and then R.Count > 0 then
                                               Long_Float (Integer'Max (R.Count, P.Count) - Integer'Min (R.Count, P.Count))
                                               / Long_Float (Integer'Max (R.Count, P.Count))
                                            else 0.0);
               E : constant Long_Float := abs (R.Elong - P.Elong) / Long_Float'Max (1.0, P.Elong);
               G : constant Long_Float := (if P.Gray >= 0.0 then
                                              abs (Picture.Mean_Gray (F.Cams (Cam).Gray, Cw, Ch, R) - P.Gray) / 255.0
                                           else 0.0);
            begin
               return D + Sz + E + G;
            end Unlike;
         begin
            --  🔴🔴 不许只在一个小窗里挑(2026-08-27 V2 实测):窗口比真实位移小的时候,
            --  它会在窗里挑一个【完全错误而读起来毫无异常】的位置,从不报错。
            --  改成【全画面都参与排序】—— 远的靠 Unlike 里那一项自己吃亏,但不再被一刀切掉。
            --  (我 2026-09-15 一度把探针幅度缩小来迁就小窗口,那是修反了:
            --   缩幅度等于把信号缩进噪声里,记录 D6 写着"探针步子太小 ⇒ 一列只解释掉 42%"。)
            for I in 0 .. Natural (Regs.Length) - 1 loop
               declare
                  R : constant Picture.Region := Regs (I);
                  D : constant Long_Float := Sqrt ((R.Cu - Pred_U) ** 2 + (R.Cv - Pred_V) ** 2);
                  U : constant Long_Float := Unlike (R);
               begin
                  if True then
                     if U < Bd then
                        Sd := Bd; Second := Best;
                        Bd := U; Best := I;
                     elsif U < Sd then
                        Sd := U; Second := I;
                     end if;
                  end if;
               end;
            end loop;
            if Best >= 0 then
               declare
                  R : Picture.Region := Regs (Best);
               begin
                  --  挨在一起的碎片算同一块:走近时物体会被切成几瓣(EV 落图:球裂成上沿+左右两条边)。
                  --  把外框挨着最像那块的碎片并进来,大小/形状按并集算
                  for Q of Regs loop
                     if Q.Count > 0 and then Q.X0 <= R.X1 and then Q.X1 >= R.X0 and then Q.Y0 <= R.Y1 and then Q.Y1 >= R.Y0 then
                        declare
                           Cx : constant Long_Float := (R.Cu * Long_Float (R.Count) + Q.Cu * Long_Float (Q.Count)) / Long_Float (R.Count + Q.Count);
                           Cy : constant Long_Float := (R.Cv * Long_Float (R.Count) + Q.Cv * Long_Float (Q.Count)) / Long_Float (R.Count + Q.Count);
                        begin
                           R.X0 := Natural'Min (R.X0, Q.X0); R.Y0 := Natural'Min (R.Y0, Q.Y0);
                           R.X1 := Natural'Max (R.X1, Q.X1); R.Y1 := Natural'Max (R.Y1, Q.Y1);
                           R.Cu := Cx; R.Cv := Cy;
                           R.Count := R.Count + Q.Count;
                           if Q.Depth > 0.0 and then (R.Depth <= 0.0 or else Q.Depth < R.Depth) then
                              R.Depth := Q.Depth;   --  并起来之后取最近的那一片的远近
                           end if;
                        end;
                     end if;
                  end loop;
                  P.Z := R.Depth; P.Height := R.Height; P.Count := R.Count;
                  P.Box_W := Long_Float (R.X1 - R.X0) / Long_Float (Cw);
                  P.Box_H := Long_Float (R.Y1 - R.Y0) / Long_Float (Ch);
                  P.Cu := R.Cu; P.Cv := R.Cv;
                  --  🔴 按【面积】算,不按外接框:框被一颗杂散像素并进来就跳,面积几乎不动。
                  --  GF 实测:框版的"看着多大"散得比自己的均值还大 ⇒ 体检把它摘掉 ⇒ 腕相机里
                  --  一个能判距离的信号都不剩(2026-09-08 曾因此把这一项写死关掉,那是治标)。
                  P.Size := Sqrt (Long_Float (P.Count) / Long_Float'Max (1.0, Long_Float (Cw * Ch)));
                  P.Ang := 2.0 * Arctan (R.Av, R.Au);
                  P.Elong := R.Elong;
                  --  朝向算多少分,看这块有多"长条":圆的(长短轴一样)自动为零 —— 球没有朝向,给满分就是追噪声
                  if P.Wang > 0.0 then
                     P.Wang := Long_Float'Max (0.0, 1.0 - 1.0 / Long_Float'Max (1.0, P.Elong));
                  end if;
                  --  只有真打平才叫分不开:第二像的和最像的差不到一成(比例,无量纲)。
                  --  松了会天天停下问(EU:球和它自己裂出来的小块也算"一样像")
                  P.Unsure := Second >= 0 and then Sd <= Bd * 1.1;
               end;
            else
               P.Cu := Pred_U; P.Cv := Pred_V; P.Lost := True;
            end if;
         end;
   end case;
   --  🔴 统一收口:算出来的位置落在【画面边界上】就不是一次测量 —— 它是被夹回来的,
   --  真值在画面外。以前各处都写 Max(0.0, Min(1.0, …)) 把它夹回来却【不标跟丢】,
   --  于是解算一本正经地朝一个编出来的位置收敛,深度也跟着读到那儿的墙。
   --  HF 实测:点一路走到 (0.000,0.000) 画面左上角,深度读出 5.109 m ——
   --  而这个场景渲染出来的深度范围只有 0.646~4.359 m,物理上不可能;差距当场从 0.294 炸到 2.575。
   --  (这就是 5754c72 那条修法,a7ab7e9 回滚里丢掉的 13 条里我漏捞的那一条。)
   P.At_Edge := P.Cu <= 0.0 or else P.Cu >= 1.0 or else P.Cv <= 0.0 or else P.Cv >= 1.0;
   if P.At_Edge then
      P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cu));
      P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cv));
      P.Lost := True;
   end if;
end Retrack;
