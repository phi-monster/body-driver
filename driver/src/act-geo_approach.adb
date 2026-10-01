separate (Act)
procedure Geo_Approach (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam, Arm : Natural; Slot : Integer;
                        Step_Limit : Natural; Event : out Unbounded_String; Steps_Taken : out Natural; Beats : out Natural;
                        Above : Boolean := False; Amt : Long_Float := 0.5; Until_Touch : Boolean := False;
                        Name : Unbounded_String := Null_Unbounded_String) is
   G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
   Beats0 : constant Natural := Plug.Steps (L);
   --  🔴 没说步数 ⇒ 我不设上限:走到到位 / 碰到 / 被顶住 / 看丢为止(身体不许自己收工)。H28 2026-09-22 实测:我自设的 12 推上限
   --  (其中 5 推是转眼)让 above 停在离合拢点 0.111 m 处,还报"你要的步数走完了"—— 脑根本没要过步数,接着就合了个空。
   Limit : constant Natural := (if Step_Limit > 0 then Step_Limit else Natural'Last);
   Tol : constant Long_Float := 0.1 * G.Gap;      --  到位容差 = 张口的一成(比例,无量纲)
   Inward : constant Long_Float := 0.15 * G.Gap;  --  指尖中点再往手心里一点 = 张口的 15%(比例,无量纲):别咬在皮上
   Want : Geom.V3 := G.Tip;
   U, V : Long_Float;
   Seen, Mok : Boolean;
   --  🔴 一条命令最多走多远,由【脑说的步子档位】定(small / medium / large,语言 §4.3:按身体自己量出的幅度计价),不由我自己调。
   --  单位 = 测距那一下横挪的大小(4 倍探针幅度,每一段开头它都刚被证明走得到);small = 1 个单位,medium = 2,large = 4。
   --  H8 2026-09-22 实测为什么要有上限:横挪 0.026 m 实到 0.024 m;之后每步命令 0.14 m(其中往下 0.097 m)实到 ≈ 0,连着 5 步 ——
   --  仿真日志 65 行 "continuous ik did not converge … falling back to global IK":大步先被连续逆解拒掉,退回全局逆解
   --  又因为目标在桌面高度而无解 ⇒ 静默不动。GB5 的球心离桌面 3.4 cm,一步 170 mm 过得去;平躺的剪刀过不去。
   --  ⚠️ 我先写过一版"走成了加倍、没走成减半",被自由棘轮拦下(owner 09-03:驱动不许自己调步子)—— 已撤。
   --  命令了没走到 ⇒ 我不自己换打法,如实说"没走成"交回脑(它可以说 small,也可以说合手)。
   --  一条命令最多走多远:看着走(09-28 定)—— 不再乘脑的档位;远的时候走还差的六成再看一眼(下面的 Frac),
   --  一条命令走不到的那一截由"到过的范围 + 往外一步"拆开、手一动就跟着往前重发(Selfmap.Go);量出来的最大一档只用来判"量过没有"
   Step_Cap : constant Long_Float := (if Stride_Of (C, Arm) > 0.0 then Long_Float'Last else 0.0);
   --  🔴 被一个面顶住之后:顶住的只是【那个方向】(命令了没走到的那个方向,量出来的),剩下的误差里沿着面的那一部分照样走得了。
   --  H12 2026-09-22 实测:垂直下探碰到桌面即停,此刻剪刀在两指正前方 0.021 m(沿桌面);整段就此停下 ⇒ 合手合了个空(读数 0.000 = 空手值)。
   --  "touching" 要的是合拢点到它身上;桌面不让我再往下,不等于不让我往前。这是在量到的接触下继续解同一个约束,不是换打法。
   --  🔴 身体不许自己收工(总规矩 09-13 / 分叉最后一个提交 09-18):脑写的是 until touched,那就一直往它身上走到【真的碰到】为止;
   --  我自己估出来的"到位了"只能说出来,不能当停的理由。H13 2026-09-22 实测:估计差 0.005 m 就停了,指尖还悬在剪刀上方,
   --  合手合到 0.000(空手值)。平躺在桌上的东西,只有往下走到被桌面顶住,指尖才真的在它两侧。
   --  🔴 不可信的观测不进解算。近处它有一截出了画面,"看到的那一块"的形心不再是同一个物理点(H14 2026-09-22 实测:
   --  下探到近处,估计位置乱跳,手往上往后走了两步,然后"看丢了")。远处那几眼看到的是完整的一块,交出来的位置是准的,
   --  而手的位姿读数每步只差 1 mm ⇒ 看不全了就不再更新它的位置,凭已知位置 + 位姿读数走完(LAB D2:不看也在)。
   Whole, Edge : Boolean;
   Its_Name : Unbounded_String;
   Said_Blind : Boolean := False;
   Known : Boolean := False;             --  此刻没眼看得清它,但它在哪我量过(C.Geo_Pw)
   Said_Known : Boolean := False;
   Said_Cut : Boolean := False;          --  说过一次"这只眼里它顶着画面边,轮廓不记"
   Said_Clamp : Boolean := False;        --  说过一次"交点出了它躺的面的范围,贴回面上"
   Who : Unbounded_String;
   Pressing : Boolean := False;          --  估计已到位,正沿原方向接着往它身上走
   Press_Dir : Geom.V3 := [0.0, 0.0, 0.0];
   Held_Back : Boolean := False;
   Wall : Geom.V3 := [0.0, 0.0, 0.0];   --  顶住我的那个方向(世界系单位向量,指向面里)
   --  这一段路空走的底(走一步 Selfmap.Step 判挡没挡用:开头装进开机探针量的那几步,这一段里每一步空走的再加进来)。
   --  挡住 = 这一步自己停下、没到、少走的比空走时多出 Blocked 的门(同碰指尖那一判;原来"沿命令方向实到不到一半"——走到一半就算没挡住)
   Wk : Selfmap.Walk;
   Rep : Selfmap.Leg_Step;
begin
   Event := Null_Unbounded_String; Steps_Taken := 0; Beats := 0;
   Want (2) := Want (2) + Inward;   --  相机 -z 朝前 ⇒ 往手心方向 = +z
   --  🔴 每一段从头量:上一段留下的那几眼(转过手、离得远)和这一段近处的眼搅在一起,交点会飞
   --  (H25 2026-09-22 实测:悬停 12 cm 处重新指了它,交点却算到 0.7 m 外、偏 54 cm,手往反方向走)。
   --  两只眼同时看见就一帧出数;只有一只眼就横挪一步当基线 —— 这一段自己的眼。
   C.Geo_Obs.Clear;
   --  上一段末尾转过手/挪过手(指尖朝下那一转尤其大)⇒ 它在这只眼里早不在旧窗那儿了;它在哪我量过 ⇒ 先把窗投到它该在的地方
   if Length (Name) > 0 and then C.Geo_Pw_Valid and then C.Geo_Pw_Name = Name then
      Retarget_Box (C, F, Cam, Arm, Name, C.Geo_Pw);
   end if;
   if Length (Name) > 0 then
      Window_From_Outline (C, F, Cam, Name);
   end if;
   Geo_Track (C, F, Cam, Slot, U, V, Seen, Name);
   Slot_Whole (C, F, Cam, Slot, Whole, Edge, Its_Name, Name);
   --  🔴 看着它走:它被画面边切掉时形心不是同一个物理点,H21 2026-09-22 实测整段路只有开头两眼算数,
   --  一条 26 mm 的基线量 0.42 m 外的东西,落点偏了 5–8 cm。⇒ 被画面边切到就先转眼把它整个看进来。
   if Above then
      C.Fingers_Aimed := False;   --  又要去它上方 ⇒ 到了再重新指
   end if;
   if Seen and then Edge and then not (C.Fingers_Aimed and then not Above) then
      declare
         Ev : Unbounded_String;
         St : Natural;
         --  🔴 先算好再传:从 F 算出的东西不许直接当实参交给会改 F 的调用(GC12 / H24 2026-09-22 同一处崩:
         --  Plug.Sense 里帧的 finalize 报 PROGRAM_ERROR)
         Ray_Ok : Boolean;
         Want : constant Geom.V3 := Geom.Ray (G, F.EE (Arm), U, V, Ray_Ok);
      begin
         if Ray_Ok then
            Geo_Turn (L, C, F, Arm, Want, Amt, Ev, St);
         else
            Ev := S ("its pixel is outside what my lens model covers, so I cannot tell which way to turn");
            St := 0;
         end if;
         Steps_Taken := Steps_Taken + St;
         Geo_Say ("它被画面边切着 ⇒ 转眼看着它(" & To_String (Ev) & ")");
         --  转过之后它的方向没变(世界系那条视线),把窗投到这条视线在新位姿画面里的落点
         declare
            Pn : constant Plug.Arm_Pose := F.EE (Arm);
         begin
            Retarget_Box (C, F, Cam, Arm, Name,
                          (if C.Geo_Pw_Valid and then C.Geo_Pw_Name = Name then C.Geo_Pw
                           else [Pn (0) + Want (0), Pn (1) + Want (1), Pn (2) + Want (2)]));
         end;
         Geo_Track (C, F, Cam, Slot, U, V, Seen, Name);
         Slot_Whole (C, F, Cam, Slot, Whole, Edge, Its_Name, Name);
      end;
   end if;
   if (Length (Its_Name) = 0 and then C.Geo_Slot /= Slot) or else (Length (Its_Name) > 0 and then C.Geo_Name /= Its_Name) then
      C.Geo_Obs.Clear; C.Geo_Came := 0.0;
   end if;
   C.Geo_Slot := Slot;
   if Length (Its_Name) > 0 then
      C.Geo_Name := Its_Name;
   end if;
   --  🔴 此刻没有一只眼看得清它,但它在哪我上一段刚量过 ⇒ 凭记住的位置走,并如实说前提是它没动。
   --  H30 2026-09-22 实测:手贴到剪刀 8 mm 时腕眼里它糊了、被切了,脑指不出 ⇒ 我报"看不见"、一步不走,而它在哪我明明知道。
   Known := Length (Its_Name) > 0 and then C.Geo_Pw_Valid and then C.Geo_Pw_Name = Its_Name;
   --  它在哪只有一种量法:两只眼同一刻的视线交点(09-22 owner 定的地基;09-26 owner:一个量只许一种量法 ⇒
   --  以前的"一条视线落到它躺的面上""我自己横挪几眼算视差(前提是它没动)"都删了)。此刻交不上 ⇒ 用上一次交出来的位置并说出来
   if not Seen and then not Known then
      Event := S ("lost: I cannot see the thing you named in this eye right now");
      return;
   end if;
   if Step_Cap <= 0.0 then
      Event := S ("refused: I have not measured how far one command moves this arm, so I cannot walk toward it");
      return;
   end if;
   loop
      if Plug.Reset_Pending (L) then
         Event := S (Reset_Event);
         exit;
      end if;         declare
         Cur : constant Plug.Arm_Pose := F.EE (Arm);
         Pw, Pc, D : Geom.V3;
         Pw_Up_Sd : Long_Float := Long_Float'Last;   --  它的位置沿"上"有多不准(交点的几何算出来的;量不出 = 最大)
         Dist : Long_Float;
      begin
         --  🔴 它此刻在哪:问【此刻】每一只看得见它的眼 —— 两条以上视线一交就是它,它动不动都一样。这是唯一的量法;
         --  交不上 ⇒ 用上一次两眼交出来的位置并说出来(09-26 删了"我自己挪过的那几眼"和"一条视线落到它躺的面上"两种)。
         declare
            Sds : Floats;
            Rays : constant Geom.Sight_Vectors.Vector := Sightlines_Now (C, F, Cam, Arm, Its_Name, Seen, Whole, U, V, Who, Sds);
            Mok : Boolean;
            Spread : Long_Float;
            Pm : Geom.V3;
         begin
            Pm := Geom.Meet (Rays, Mok, Spread);
            --  🔴 交点可信的条件:几条视线离交点的最大偏差不超过【眼自己量朝向时的像素残差】换算到那个距离上的米数(量过的数,不是拍的)。
            --  H42 2026-09-22 实测:一个偏差 0.079 m 的交点被当真记住,后面每一段都往 8 cm 高的空中走。
            if Mok then
               declare
                  Gw : constant Geom.Cam_Geo := Geo_Of (C, C.Map.World_Cam);
                  Hp : constant Plug.Arm_Pose := F.EE (Arm);
                  Tol_Hand : constant Long_Float := (if G.F > 0.0 then G.Rms * Geom.Norm ([Pm (0) - Hp (0), Pm (1) - Hp (1), Pm (2) - Hp (2)]) / G.F else 0.0);
                  Tol_Still : constant Long_Float := (if Gw.Fixed and then Gw.F > 0.0 then Gw.Rms * Geom.Norm ([Pm (0) - Gw.Pos (0), Pm (1) - Gw.Pos (1), Pm (2) - Gw.Pos (2)]) / Gw.F else 0.0);
                  Tol : constant Long_Float := Long_Float'Max (Tol_Hand, Tol_Still);
               begin
                  --  09-13 总规矩:动起来之后身体不许有闸 ⇒ 交点照用,偏差说出来(它就是这个位置有多不准)。
                  --  以前偏差超过眼的误差就扔掉交点(H42 之后加的):标定准到 1 mm 之后,长条的东西被画面边切着、两只眼的"中心"不是同一点,
                  --  1.7 cm 的偏差回回被扔,SHOT1 一集扔了 32 次、一次都没走到它身上
                  if Spread > Tol then
                     Geo_Say ("此刻 " & To_String (Who) & " 相机的视线交在 (" & Mm (Pm (0)) & "," & Mm (Pm (1)) & "," & Mm (Pm (2)) & "),视线间偏差 "
                              & Mm (Spread) & ",比眼自己的误差(" & Mm (Tol) & ")大 —— 两只眼看到的中心可能不是同一点;照这个交点走,它的位置按差 " & Mm (Spread) & " 算");
                  end if;
               end;
            end if;
            if Mok then
               Pw := Pm;
               Pw_Up_Sd := Geom.Meet_Sd (Rays, Sds, Pm, Up_Dir (C));
               Geo_Say ("此刻 " & To_String (Who) & " 相机同时看见它 ⇒ 视线交在 (" & Mm (Pw (0)) & "," & Mm (Pw (1)) & "," & Mm (Pw (2))
                        & "),视线间最大偏差 " & Mm (Spread) & ";按两只眼各自的误差和视线夹角,高低上不准 "
                        & (if Pw_Up_Sd < Long_Float'Last then Mm (Pw_Up_Sd) else "(量不出)"));
            elsif Known then
               Pw := C.Geo_Pw;
               Pw_Up_Sd := C.Geo_Pw_Up_Sd;
               if not Said_Known then
                  Said_Known := True;
                  Geo_Say ("此刻没有两只眼同时看见它 ⇒ 按上一次两眼交出来的位置 (" & Mm (Pw (0)) & "," & Mm (Pw (1)) & "," & Mm (Pw (2))
                           & ") 走(前提是它没动)");
               end if;
            else
               Event := S ("lost: two of my eyes have not seen it at the same moment, so I cannot tell where it is");
               exit;
            end if;
            if Mok and then Length (Its_Name) > 0 then
               --  记住它在哪:下一段看不清时凭这个走。两眼交点(偏差毫米级)比单眼挪出来的准得多(H35 2026-09-22 实测:交点 z=0.628,
               --  之后手指朝下近处单眼挪出来的 z=0.745 把它盖掉了,下一段就按 12 cm 高的空中走)⇒ 这一段里有过交点就不让单眼盖
               --  H47 2026-09-23 实测:单眼挪出来的一个坏位置 (0.43, −0.21, 0.90) 被记住,之后十几段全按它走、一步没走。
               --  ⇒ 只记两眼交出来的(09-26 起也只有这一种估计)
               if Mok then
                  C.Geo_Pw := Pw; C.Geo_Pw_Valid := True; C.Geo_Pw_Name := Its_Name; C.Geo_Pw_Met := True; C.Geo_Pw_Up_Sd := Pw_Up_Sd;
               end if;
            end if;
         end;
         --  它躺在我碰过的面上 ⇒ 交点在面之下 / 比张口还高出面的,贴回面上再当目标(H53:交点在桌面之下 12 cm,"上方一个张口"就成了桌面之下,一路顶着桌子)
         Pw := Plane_Point (C, Pw, Up_Dir (C), Say => not Said_Clamp);
         if C.Touch_Valid and then not Said_Clamp then
            Said_Clamp := True;
         end if;
         Pc := Geom.To_Cam (G, Cur, Pw);
         --  它的位置是两眼交出来的 ⇒ 每只看全了它的眼都记一份它顶面的点(留最细的);哪儿夹得住由接触集从这上面算(PLAN 1.5),不再在像素上扫弦。
         --  腕眼里它常常顶着画面边(H48 的框就贴着 y=479)⇒ 那一眼的轮廓不完整、不记;不动的眼/另一只手的眼看全了它、我的手又没压在它上面 ⇒ 记
         if Length (Its_Name) > 0 then
            if Seen and then Whole then
               Take_Silhouette (C, F, Cam, Arm, Its_Name, Pw, Pw_Up_Sd);
            elsif Seen and then not Said_Cut then
               Said_Cut := True;
               Geo_Say ("这只眼里它顶着画面边,轮廓不完整 ⇒ 这一眼不记它的顶面点,看别的眼");
            end if;
            for Cm in 0 .. C.Map.N_Cams - 1 loop
               if Cm /= Cam and then Cm < Natural (C.Geo.Length) and then Cm < Natural (F.Cams.Length) then
                  declare
                     Bx2 : constant Integer := Boxed_By (C, Cm, Its_Name);
                  begin
                     if Bx2 >= 0 then
                        declare
                           B2 : constant Boxed_Thing := C.Boxed (Natural (Bx2));
                           Edge2 : constant Boolean := B2.X0 = 0 or else B2.Y0 = 0 or else B2.X1 + 1 >= F.Cams (Cm).W or else B2.Y1 + 1 >= F.Cams (Cm).H;
                        begin
                           if B2.Seen and then not Edge2 and then not Hand_Covers (C, F, Arm, Cm, B2) then
                              Take_Silhouette (C, F, Cm, Arm, Its_Name, Pw, Pw_Up_Sd);
                           end if;
                        end;
                     end if;
                  end;
               end if;
            end loop;
         end if;
         declare
            --  到它上方 ⇒ 它该落在"指尖合拢那一点"正下方一个张口处:把世界系的"往下一个张口"转进相机系,加到目标上。
            --  "上方" = 它躺的那个面的自由一侧:碰过的面按量到的法向,没碰过按重力的上(和转指尖那一条同一个约定)
            Nn_Up : constant Geom.V3 := Up_Dir (C);
            Down_C : constant Geom.V3 :=
              (if Above then Geom.Ap (Geom.Tr (Geom.Cam_R (G, Cur)), [-G.Gap * Nn_Up (0), -G.Gap * Nn_Up (1), -G.Gap * Nn_Up (2)]) else [0.0, 0.0, 0.0]);
         begin
            D := [Pc (0) - Want (0) - Down_C (0), Pc (1) - Want (1) - Down_C (1), Pc (2) - Want (2) - Down_C (2)];
         end;
         Dist := Geom.Norm (D);
         C.Geo_Dist := Dist; C.Geo_Round := C.Round_N; C.Geo_At := Cur; C.Geo_At_Arm := Integer (Arm); C.Geo_At_Above := Above;
         Geo_Say ("它在相机前 " & Mm (-Pc (2)) & "(左右 " & Mm (Pc (0)) & " 上下 " & Mm (Pc (1)) & "),离指尖该到的那点还差 " & Mm (Dist) &
                  "(左右 " & Mm (D (0)) & " 上下 " & Mm (D (1)) & " 前后 " & Mm (D (2)) & ")");
         if -Pc (2) <= 0.0 then
            Event := S ("lost: my sightlines do not meet in front of me (the thing may have moved)");
            exit;
         end if;
         if Held_Back then
            declare
               --  剩余误差搬到世界系,去掉指向面里的那一份;剩下的长度才是"还走得了的差距"
               Rcw : constant Geom.M3 := Geom.Cam_R (G, Cur);
               Dwf : Geom.V3 := Geom.Ap (Rcw, D);
               Into : constant Long_Float := Dwf (0) * Wall (0) + Dwf (1) * Wall (1) + Dwf (2) * Wall (2);
            begin
               if Into > 0.0 then
                  Dwf := [Dwf (0) - Into * Wall (0), Dwf (1) - Into * Wall (1), Dwf (2) - Into * Wall (2)];
               end if;
               D := Geom.Ap (Geom.Tr (Rcw), Dwf);
               Dist := Geom.Norm (D);
               Geo_Say ("被一个面顶着:沿着面还差 " & Mm (Dist) & "(往面里那一份 " & Mm (Long_Float'Max (0.0, Into)) & " 走不了,不算)");
            end;
         end if;
         if (Pressing or else Dist <= Tol) and then Until_Touch and then (not Above or else Pressing) and then not Held_Back
           and then (Geom.Norm (C.Geo_Dir) > 0.0 or else Pressing) and then Steps_Taken < Limit
         then
            if not Pressing then
               Pressing := True; Press_Dir := C.Geo_Dir;
               Geo_Say ("我估着到位了(差 " & Mm (Dist) & "),可你说的是碰到为止 ⇒ 沿来的方向接着往它身上走,到真被顶住");
            end if;
            declare
               Ln : constant Long_Float := 4.0 * Geo_Base (C, Arm);     --  一个量距单位(刚被证明走得到的那一档)
               Dw : constant Geom.V3 := [Press_Dir (0) * Ln, Press_Dir (1) * Ln, Press_Dir (2) * Ln];
            begin
               Geo_Move (L, C, F, Arm, Dw, Mok, Wk, Rep);
               Steps_Taken := Steps_Taken + 1;
               declare
                  Now : constant Plug.Arm_Pose := F.EE (Arm);
                  Got : constant Long_Float := Rep.Went;   --  沿命令方向实到多少
               begin
                  if Rep.Blocked_T then
                     Event := S ("contact: I kept going toward it as you asked and something stopped my hand (I commanded " & Len (C, Ln)
                                 & " and went " & Len (C, Got) & "); by my own estimate the thing sits at where my fingers close");
                     C.Geo_At := Now; C.Geo_At_Arm := Integer (Arm); C.Geo_At_Above := False;   --  压到它身上了:接下来合手不用再下去
                     exit;
                  end if;
               end;
            end;
         elsif Dist <= Tol then
            Event := S ((if Held_Back
                         then "contact: I am against a surface and as close as it lets me (the thing sits " & Len (C, Dist) & " from where my fingers close, measured along that surface)"
                         elsif Above
                         then "amount: arrived above it (it sits one hand-opening, " & Len (C, G.Gap) & ", straight below where my fingers close, within " & Len (C, Dist) & ")"
                         else "amount: arrived (the thing sits " & Len (C, Dist) & " from where my fingers close)"));
            --  🔴 到了它上方,顺手把【手指】指向它躺着的那个面(面 = 我碰过的那个面的法向;没碰过就按"上"的反向)。
            --  这不是抓剪刀的规矩,是"在它上方"对一副夹爪的含义:两指要能落到它两侧,指尖得朝着它来。
            --  H15/H19 2026-09-22 实测:手指斜着伸,合拢点到了它身上 3–5 mm 内,指尖却还悬在它上方 ⇒ 合空。
            --  转的是量过的方向(指尖方向 = 量过的指尖偏置),转多少由几何定,指尖位置边转边补;转不动就如实说。
            if Above and then G.Tip_Valid then
               declare
                  Nn : constant Geom.V3 := Up_Dir (C);
                  Ev : Unbounded_String;
                  St : Natural;
               begin
                  Geo_Turn (L, C, F, Arm, [-Nn (0), -Nn (1), -Nn (2)], Amt, Ev, St, Along => G.Tip);
                  Steps_Taken := Steps_Taken + St;
                  Retarget_Box (C, F, Cam, Arm, Name, Pw);   --  指尖朝下这一转很大:把它的窗投进转过的眼,下一段才认得出它
                  C.Fingers_Aimed := Index (Ev, "amount: arrived") > 0;
                  Append (Event, (if C.Fingers_Aimed then "; my fingers now point down at it"
                                  else "; I tried to point my fingers down at it: " & To_String (Ev)));
               end;
            end if;
            --  🔴 "到它上方、碰到为止":到了上方,脑说的是碰到为止 ⇒ 顺着它躺的面的法向往它身上压,到真被顶住(和 touching 的"接着往它身上走"同一条)。
            --  H37 2026-09-22 实测:Qwen 十有八九写 `above X until touched`;上方永远碰不到,那一行就原地重跑十遍,然后它写 farther 走了。
            --  "until touched" 是脑明说的:没碰到就接着走。
            if Above and then Until_Touch and then not Held_Back and then Steps_Taken < Limit then
               declare
                  Nn_P : constant Geom.V3 := Up_Dir (C);
               begin
                  Pressing := True; Press_Dir := [-Nn_P (0), -Nn_P (1), -Nn_P (2)];
                  Geo_Say ("到了它上方,可你说的是碰到为止 ⇒ 顺着法向往它身上压,到真被顶住");
               end;
            else
               exit;
            end if;
         end if;
         if Steps_Taken >= Limit then
            Event := S ("steps: I took the steps you asked for (still " & Len (C, Dist) & " from where my fingers close)");
            exit;
         end if;
         if not Pressing then
         --  走一步(Selfmap.Step):目标 = 指尖该到的那一点,这一步走还差的 Frac、最长 Step_Cap(量过步幅 ⇒ 不限:一条命令走不到的那一截由
         --  "到过的范围 + 往外一步"拆开、手一动就跟着往前重发)
         Geo_Move (L, C, F, Arm, Geom.Ap (Geom.Cam_R (G, Cur), D), Mok, Wk, Rep,
                   Frac => (if Dist > G.Gap then 0.6 else 1.0),   --  远时走六成再看一眼(比例,无量纲);近了一步到
                   Track => Step_Cap);
         declare
            Dw : constant Geom.V3 := [Rep.Cmd (0), Rep.Cmd (1), Rep.Cmd (2)];   --  这一步命令的平移(世界系)
            Ln : constant Long_Float := Rep.Len;
         begin
            Steps_Taken := Steps_Taken + 1;
            declare
               Now : constant Plug.Arm_Pose := F.EE (Arm);
               --  实到 = 沿【命令的方向】真走了多少(不是位移的长度:H10 2026-09-22 实测,命令往下 0.041 m 只下去 0.006 m,
               --  手却横着滑了 0.025 m —— 按长度比就被当成"走成了",接着在一个撞着桌面的姿势上继续算、算飞)
               Got : constant Long_Float := Rep.Went;
            begin
               if Rep.Blocked_T and then not Held_Back then
                  --  第一次被顶住:记下顶住我的方向 = 命令的位移减去实到的位移(量出来的),之后只走沿着面的那一部分
                  declare
                     Miss : constant Geom.V3 := [Dw (0) - (Now (0) - Cur (0)), Dw (1) - (Now (1) - Cur (1)), Dw (2) - (Now (2) - Cur (2))];
                     Ml : constant Long_Float := Geom.Norm (Miss);
                  begin
                     if Ml > 0.0 then
                        Wall := [Miss (0) / Ml, Miss (1) / Ml, Miss (2) / Ml];
                        Held_Back := True;
                        --  碰过的点进地图:面上的一点 = 此刻指尖的世界位置,法向 = 顶住我的方向反过来。
                        --  🔴 只有【朝下】被顶住的才是它躺的面(东西靠着它抵住重力);顶住我的方向横着的,是墙、或是我自己够不着了
                        --  (H31 2026-09-22 实测:右臂横跨整张桌去够,在 (-0.26,-0.97,0) 方向被自己的关节顶住,我把它记成了"面",
                        --  于是"上方"和"指尖朝下"都朝了横向,后面全乱)。朝下 = 竖直分量比水平分量大(纯比较);"下"= 位姿系 -z,同抬手那条约定。
                        if abs (Wall (2)) > Sqrt (Wall (0) ** 2 + Wall (1) ** 2) then
                           declare
                              Tw : constant Geom.V3 := Geom.Ap (Geom.Cam_R (G, Now), G.Tip);
                           begin
                              Note_Support (C, [Now (0) + Tw (0), Now (1) + Tw (1), Now (2) + Tw (2)], [-Wall (0), -Wall (1), -Wall (2)],
                                            "这一步要 " & Mm (Ln) & " 只到 " & Mm (Got) & ",方向 ("
                                            & Codec.Fmt (Wall (0), 2) & "," & Codec.Fmt (Wall (1), 2) & "," & Codec.Fmt (Wall (2), 2) & ")");
                           end;
                        else
                           C.Walls.Append (Wall_Mark'(Arm => Arm, P => Tip_World (C, Arm, Now), W => Wall));
                           Geo_Say ("这一步要 " & Mm (Ln) & " 只到 " & Mm (Got) & " ⇒ 被横着顶住了,方向 ("
                                    & Codec.Fmt (Wall (0), 2) & "," & Codec.Fmt (Wall (1), 2) & "," & Codec.Fmt (Wall (2), 2)
                                    & "):不是它躺的面(是墙,或我自己的关节到头了),不记成面;记下「这条臂到这儿为止」,沿着它接着走");
                        end if;
                     end if;
                  end;
               elsif Rep.Blocked_T then              --  又被挡住(沿着面走的这一步也比空走时少走得多)= 命令了,身体没走
                  Event := S ("resist: I commanded a step of " & Len (C, Ln) & " toward it and my hand only went " & Len (C, Got)
                              & " (" & Len (C, Dist) & " from where my fingers close) - either something is holding my hand there, "
                              & "or that step was more than I can do in one command from this pose");
                  exit;
               end if;
            end;
            C.Geo_Came := C.Geo_Came + Ln;
            if Ln > 0.0 then
               C.Geo_Dir := [Dw (0) / Ln, Dw (1) / Ln, Dw (2) / Ln];
            end if;
            --  🔴 迈了一大步之后,它在我这只眼里的位置和大小都变了(H5 2026-09-22 实测:走到差 0.071 m 时跟丢 ——
            --  点过名的东西是"在上一帧量到它的地方原样再量",一步 14 cm 之后它早不在那儿了)。
            --  可我【知道】它该在哪:它的位置是我刚用两条视线交出来的(Pw),我挪了多少是位姿读数说的 ⇒
            --  把它投到新位姿的画面里,就是它这一帧该出现的像素;离我近了几成,它就大了几成。先把重量的窗挪过去、放大,再量。
            declare
               Have_Slot : constant Boolean := Slot >= 0 and then Natural (Slot) < World.Count (C.Wld, Cam);
            begin
            if Length (Name) > 0 or else Have_Slot then
               declare
                  Bx : constant Integer :=
                    (if Length (Name) > 0 then Boxed_By (C, Cam, Name)
                     else Boxed_Index (C, Cam, World.Get (C.Wld, Cam, Natural (Slot)).R.Cu, World.Get (C.Wld, Cam, Natural (Slot)).R.Cv));
                  Pu, Pv : Long_Float;
                  Front : Boolean;
                  Z_Was : constant Long_Float := -Pc (2);
                  Z_Now : constant Long_Float := -Geom.To_Cam (G, F.EE (Arm), Pw) (2);
                  Cw : constant Natural := F.Cams (Cam).W;
                  Ch : constant Natural := F.Cams (Cam).H;
               begin
                  Geom.Project (G, F.EE (Arm), Pw, Pu, Pv, Front);
                  if Bx >= 0 and then Front and then Z_Was > 0.0 and then Z_Now > 0.0
                    and then Pu >= 0.0 and then Pv >= 0.0 and then Pu < Long_Float (Cw) and then Pv < Long_Float (Ch)
                  then
                     declare
                        B : Boxed_Thing := C.Boxed (Natural (Bx));
                        Grow : constant Long_Float := Z_Was / Z_Now;
                        Hw : constant Long_Float := 0.5 * (Long_Float (B.X1 - B.X0) * Grow);   --  半宽(纯数学的一半)
                        Hh : constant Long_Float := 0.5 * (Long_Float (B.Y1 - B.Y0) * Grow);
                        function Px (V2 : Long_Float; Span : Natural) return Natural is
                          (Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Span - 1), V2))));
                     begin
                        if B.Pu_On >= 0.0 then   --  它身上那一点跟着框平移、按远近缩放
                           B.Pu_On := Pu + Grow * (B.Pu_On - 0.5 * Long_Float (B.X0 + B.X1));
                           B.Pv_On := Pv + Grow * (B.Pv_On - 0.5 * Long_Float (B.Y0 + B.Y1));
                        end if;
                        B.X0 := Px (Pu - Hw, Cw); B.X1 := Px (Pu + Hw, Cw);
                        B.Y0 := Px (Pv - Hh, Ch); B.Y1 := Px (Pv + Hh, Ch);
                        B.Count := Natural (Long_Float (B.Count) * Grow * Grow);   --  它该有的像素数跟着远近变(面积 = 线尺寸的平方,纯数学)
                        if B.X1 > B.X0 and then B.Y1 > B.Y0 then
                           C.Boxed.Replace_Element (Natural (Bx), B);
                           --  槽里记的那一块也挪到预测处,好让这一帧量到的新块对得上同一个槽(World.Observe 按形心就近认槽)
                           if Have_Slot then
                              World.Shift_Slot (C.Wld, Cam, Natural (Slot), Pu / Long_Float (Cw), Pv / Long_Float (Ch), Grow);
                           end if;
                           Geo_Say ("它该出现在 (" & Codec.Fmt (Pu, 1) & "," & Codec.Fmt (Pv, 1) & "),大了 " & Codec.Fmt (Grow, 2) & " 倍 ⇒ 到那儿去量");
                        end if;
                     end;
                  end if;
               end;
            end if;
            end;
            Geo_Track (C, F, Cam, Slot, U, V, Seen, Name);
            Slot_Whole (C, F, Cam, Slot, Whole, Edge, Its_Name, Name);
            --  手指已经指着它躺的面时,最后贴上去这一段不再为了看它而转手(转了指尖就不朝下了);看不全就按位姿读数走
            if Seen and then Edge and then not Pressing and then not (C.Fingers_Aimed and then not Above) then
               declare
                  Ev : Unbounded_String;
                  St : Natural;
                  Ray_Ok : Boolean;
                  Want : constant Geom.V3 := Geom.Ray (G, F.EE (Arm), U, V, Ray_Ok);   --  先算好再传(见上)
               begin
                  if Ray_Ok then
                     Geo_Turn (L, C, F, Arm, Want, Amt, Ev, St);
                  else
                     Ev := S ("its pixel is outside what my lens model covers, so I cannot tell which way to turn");
                     St := 0;
                  end if;
                  Steps_Taken := Steps_Taken + St;
                  Geo_Say ("它被画面边切着 ⇒ 转眼看着它(" & To_String (Ev) & ")");
                  Retarget_Box (C, F, Cam, Arm, Name, Pw);   --  它在哪这一步刚算过(Pw)⇒ 投进转过的眼
                  Geo_Track (C, F, Cam, Slot, U, V, Seen, Name);
                  Slot_Whole (C, F, Cam, Slot, Whole, Edge, Its_Name, Name);
               end;
            end if;
            if (Known or else C.Geo_Pw_Valid) and then Seen and then not Whole then
               if not Said_Blind then
                  Said_Blind := True;
                  Geo_Say ("它有一截出了画面/被挡住,这一眼不可信 ⇒ 不再更新它的位置;凭上一次两眼交出来的位置走完");
               end if;
            elsif not Seen then
               --  最后一步它进了指缝、被手指挡住也正常:上一眼已经在两倍容差内(倍数,无量纲)
               if Dist <= 2.0 * Tol then
                  Event := S ("amount: arrived (I lost sight of it on the last step; it was " & Len (C, Dist) & " from where my fingers close)");
                  exit;
               elsif Known or else C.Geo_Pw_Valid then
                  --  看不见了,可它在哪我这一段(或上一段)量过 ⇒ 凭记住的位置走完
                  Known := True;
                  if not Said_Known then
                     Said_Known := True;
                     Geo_Say ("这一步之后看不见它了 ⇒ 按我量到的位置走完(前提是它没动)");
                  end if;
               else
                  Event := S ("lost: I lost sight of it after that step (it was " & Len (C, Dist) & " away)");
                  exit;
               end if;
            end if;
         end;
         end if;   --  not Pressing
      end;
   end loop;
   Beats := Beats_Since (L, Beats0);
end Geo_Approach;
