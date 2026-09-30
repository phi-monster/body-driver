separate (Act)
function Cut_Things_Raw (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
   Raw : Picture.Regions;
   Kept : Picture.Regions;
begin
   --  🔴 M2 实测(850 轮 850 次撑爆、一段程序没问出来):没深度时这里原来直接返回空,
   --  THINGS OUT IN THE WORLD 整节是空的 —— 官方观测就是 3 路 RGB,这等于把眼睛关掉。
   --  而我第一次的修法(落到按颜色切)更糟:单帧清单涨到 267 条,把脑淹死
   --  —— 同一段代码上面那行警告早写着"在能看见东西的桌子上开着它,清单会从 7 条涨到 46 条"。
   --  ⇒ 正确的那条路是【按明暗切】(Cut_Bright):它自带门槛,并且在非自眼里把握区里的自己剔掉。
   --  长在手上的眼也走这条(手指在自己眼里是黑的);它一块都切不出来时才退回深度那一路。
   if Cam_Arm (C, Cam) >= 0 or else not F.Cams (Cam).Has_Depth then
      Raw := Cut_Bright (C, F, Cam);
      if not Raw.Is_Empty then
         for R of Raw loop
            declare
               Mine : Boolean := False;
            begin
               for A in 0 .. C.Map.Arms - 1 loop
                  if Cam_Arm (C, Cam) < 0 and then Zone.Is_Self (Zone_Of (C, A, Cam), R, Cw, Ch) then
                     Mine := True;
                  end if;
               end loop;
               if not Mine then
                  Kept.Append (R);
               end if;
            end;
         end loop;
         return Kept;
      end if;
   end if;
   if not F.Cams (Cam).Has_Depth then
      return Kept;
   end if;
   --  🔴🔴 尺子不是选一把,是【每一把都看一遍,合起来】。
   --  "鼓出来"是相对周围说的:窗口比这块东西小的时候,这块东西自己就是周围 ⇒ 它鼓 0、整块消失。
   --  而窗口是按"两指在画面里张多开"缩放的 ⇒ 手越近、窗口越大、能被抹掉的东西越大 —— 方向正好反了。
   --  GM 实测:手一凑近,球(3027 px)和乐高(5098 px)双双从清单里消失,只剩 2 米外 10 px 的墙斑;
   --  而旧的"一块都切不出才换尺子"因为墙斑还在,根本不触发 ⇒ 最后一步反而瞎了 ⇒ 模板飘到墙上 ⇒
   --  手追着 2.13 m 外的墙把关节顶死。(同一条 GB 也实测过,修法 fe2ec8e 写过,被 a7ab7e9 回滚掉了。)
   --  合并规则:粗尺子切出来的块,只有当它的形心还没被任何已收的块盖住时才收(不重复列)。
   declare
      Win : constant Long_Float := Cut_Window (C, Cam, F);
      --  这台相机长在某条胳膊上吗:长着的话,要抓的东西贴到画面边是常态,不许因为贴边就丢
      Own_Cam_Here : constant Boolean := Cam_Arm (C, Cam) >= 0;
      --  🔴 第二把尺子 = 【脑点名那个东西自己有多大】(身体量的,不是我拍的系数)。
      --  闭运算填的是比窗口窄的东西 ⇒ 比窗口【宽】的东西自己就是背景,鼓 0、整块消失。
      --  所以会消失的恰恰是"比尺子宽"的那个,拿它自己的宽度当第二把尺子正好够着它。
      --  🔴🔴 而"多大"必须【按这一台相机算】:同一个球在头顶相机里 144 px、在腕相机里 96 px 宽却
      --  占 0.15 画幅。GP 实测:段跑在头顶相机 ⇒ Want_Size 是头顶那个小数 ⇒ 到腕相机不够大 ⇒
      --  球又被抹掉(腕相机只切出 3 块、2 块判成自己、只剩一块 516 px)。
      --  这一台相机自己记着"你上次点名那块在我这儿多大",拿它。取不到才退回 Want_Size。
      Wide : constant Long_Float := Long_Float'Max (Named_Span (C, Cam, Cw), C.Want_Size);
   begin
      Raw := Picture.Cut (F.Cams (Cam).Depth, Cw, Ch, Win, Sigma_Mult, Keep_Edge => Own_Cam_Here);
      if Wide > Win then
         declare
            More : constant Picture.Regions :=
              Picture.Cut (F.Cams (Cam).Depth, Cw, Ch, Wide, Sigma_Mult, Keep_Edge => Own_Cam_Here);
         begin
            for R of More loop
               declare
                  Covered : Boolean := False;
               begin
                  for Q of Raw loop
                     if Picture.Inside (Q, R.Cu, R.Cv, Cw, Ch, 0.0) then
                        Covered := True;
                     end if;
                  end loop;
                  if not Covered then
                     Raw.Append (R);
                  end if;
               end;
            end loop;
         end;
      end if;
   end;
   --  🔴 只有【深度上一个都看不出来】的时候才按颜色切:桌面木纹、瓷砖缝的颜色台阶比线还明显,
   --  在能看见东西的桌子上开着它,清单会从 7 条涨到 46 条,脑子被淹掉(ES 实测)。
   --  线板那种场合深度切不出任何东西,颜色这一路才接手。
   if Raw.Is_Empty then
   --  再按颜色切一遍,把深度上鼓不出来的细东西(线、缝、刀口)补进来:
   --  门槛 = 这台相机静止时颜色抖多少(量出来的)的几倍;贴画面边的是桌面/墙,丢掉(本仓既有规矩);
   --  已经被深度块盖住的不重复列
   declare
      --  门槛:比相机噪声大,也要比这张画面自己的纹理粗(木纹、布纹都会被纹理这一项吃掉);倍数无量纲
      Noise_C : constant Natural := (if Cam < Natural (C.Map.Pic_Floor.Length) and then C.Map.Pic_Floor (Cam) > 0
                                     then Natural (C.Map.Pic_Floor (Cam)) else 0);
      Floor_C : constant Long_Float :=
        Long_Float'Max (Long_Float (Noise_C) * 2.0 + 1.0, Picture.Texture_Level (F.Cams (Cam).RGB, Cw, Ch) * 4.0);
      Thin : constant Picture.Regions := Picture.Cut_Colour (F.Cams (Cam).RGB, Cw, Ch, Floor_C, Picture.Min_Pixels (Cw, Ch));
   begin
      for R of Thin loop
         declare
            Edge : constant Boolean := R.X0 = 0 or else R.Y0 = 0 or else R.X1 >= Cw - 1 or else R.Y1 >= Ch - 1;
            Covered : Boolean := False;
         begin
            for Q of Raw loop
               if Picture.Inside (Q, R.Cu, R.Cv, Cw, Ch, 0.0) then
                  Covered := True;
               end if;
            end loop;
            if not Edge and then not Covered then
               declare
                  Q : Picture.Region := R;
               begin
                  --  颜色切出来的块也要有远近:在它自己的位置上读一小片深度(窗口 = 它自己框的四分之一,比例,无量纲)
                  if F.Cams (Cam).Has_Depth then
                     declare
                        Zd : constant Long_Float := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, R.Cu, R.Cv,
                                                                       Long_Float'Max (0.005, Long_Float (R.X1 - R.X0) / Long_Float (Cw) * 0.25));
                     begin
                        if not Picture.Is_Nan (Zd) then
                           Q.Depth := Zd;
                        end if;
                     end;
                  end if;
                  Raw.Append (Q);
               end;
            end if;
         end;
      end loop;
   end;
   end if;
   declare
      Mine_N : Natural := 0;
      Big_W : Natural := 0;
   begin
   for R of Raw loop
      declare
         Mine : Boolean := False;
      begin
         for A in 0 .. C.Map.Arms - 1 loop
            if Zone.Is_Self (Zone_Of (C, A, Cam), R, Cw, Ch) then
               Mine := True;
            end if;
         end loop;
         if not Mine then
            Kept.Append (R);
            Big_W := Natural'Max (Big_W, R.X1 - R.X0 + 1);
         else
            Mine_N := Mine_N + 1;
         end if;
      end;
   end loop;
   --  GM 里手一凑近,球(3027 px)和乐高人(5098 px)双双从清单里消失,只剩 2 米外 10 px 的墙斑。
   --  两个可疑处各印一个数,别再靠猜:闭运算窗口(比它窄的凸起会被当背景填平)· 被判成"我自己"而丢掉的块数。
   if Codec.Env ("BL_CUTLOG") /= "" then
      Put_Line ("[身]     切块(相机" & Natural'Image (Cam) & "):窗口 " & Codec.Fmt (Cut_Window (C, Cam, F), 3) & " 画幅 = "
                & Codec.Img (Natural (Long_Float (Cw) * Cut_Window (C, Cam, F))) & " px · 切出 " & Codec.Img (Natural (Raw.Length))
                & " 块,其中 " & Codec.Img (Mine_N) & " 块判成我自己丢掉 · 留下最宽的一块 " & Codec.Img (Big_W) & " px");
   end if;
   end;
   return Kept;
end Cut_Things_Raw;
