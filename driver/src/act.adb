with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Codec;
with Draw;
with Flow;
with Monitor;
with Backup;
package body Act is
   Sigma_Mult : constant Long_Float := 3.0;   --  鼓出来超过背景自己稳健 σ 的几倍才算一块(在真实深度图上验过:3 中,5 杀光);无量纲
   Track_Win : constant Long_Float := 0.10;   --  一步里任何被跟踪的点在画面里最多跑十分之一画幅(跟踪窗,比例,无量纲)
   Cap_Mult : constant Long_Float := 2.0;     --  一步命令上限 = 探针幅度(点在画面里跑过地板的那一档)的几倍(倍数,无量纲;EH:8 倍让阻尼当家,步子反而只剩探针的一倍)
   Step_Cap : constant := 60;                 --  一段最多几步(安全上限,不是策略)
   Unit_Reach : constant Table.Vec := [others => 1.0];

   function S (X : String) return Unbounded_String renames To_Unbounded_String;

   function Zone_Of (C : Context; Arm, Cam : Natural) return Zone.Hand_Zone is
   begin
      if Arm < Natural (C.Hands.Length) and then Cam < Natural (C.Hands (Arm).Zones.Length) then
         return C.Hands (Arm).Zones (Cam);
      end if;
      return (others => <>);
   end Zone_Of;

   --  🔴 身体不许自己发明"该抓在这块东西的哪个高度"(owner 2026-09-09:"这不是作弊?你给一个棒球写死规则?")。
   --  "抓在哪儿"是一个决定,决定归脑。身体只把爪子带到这块【量到的】位置,一点偏移都不加;
   --  要更低,脑说"再往下"。
   function Grab_Depth (O : Item) return Long_Float is (O.Depth);

   --  读"我这一瓣离相机多远"时,窗口要用【这一瓣自己在画面里多大】,不能用整只手的张幅。
   --  🔴 张幅那么大的窗口会把自己的大臂一起框进来,而大臂比手指更靠近相机 ⇒ 靠近的那一档分位一路爬到大臂上。
   --  实测(FO):爪子的"离相机多远"9 步从 1.07 米滑到 0.50 米,每一步都正好卡在"一步最多变一成"的限速上 ——
   --  限速只是把错误读数拖慢,拦不住它;拦得住的是别把大臂框进来。
   function Lobe_Win (Z : Zone.Hand_Zone; Cw, Ch : Natural) return Long_Float is
      W1 : constant Long_Float := Long_Float (Z.A.X1 - Z.A.X0 + 1) / Long_Float (Cw);
      H1 : constant Long_Float := Long_Float (Z.A.Y1 - Z.A.Y0 + 1) / Long_Float (Ch);
      W2 : constant Long_Float := (if Z.B.Valid then Long_Float (Z.B.X1 - Z.B.X0 + 1) / Long_Float (Cw) else W1);
      H2 : constant Long_Float := (if Z.B.Valid then Long_Float (Z.B.Y1 - Z.B.Y0 + 1) / Long_Float (Ch) else H1);
   begin
      --  还没量到手的时候退回一个百分之一画幅的小窗(比例,无量纲:占画面的多少,跟相机、镜头、机器人大小都无关)
      if not Z.Valid or else not Z.A.Valid then
         return 0.01;
      end if;
      --  取最窄的那一边的四分之一:落在手指身上,不碰到旁边。再小也留半个千分之四画幅,免得窗口小到一个像素(比例,无量纲)
      return Long_Float'Max (0.004, 0.25 * Long_Float'Min (Long_Float'Min (W1, H1), Long_Float'Min (W2, H2)));
   end Lobe_Win;

   function Track_Idx (C : Context; Arm, Cam : Natural) return Natural is (Arm * C.Map.N_Cams + Cam);

   function Cam_Arm (C : Context; Cam : Natural) return Integer is
   begin
      for A in 0 .. Natural (C.Map.Cam_On_Arm.Length) - 1 loop
         if C.Map.Cam_On_Arm (A) = Integer (Cam) then
            return A;
         end if;
      end loop;
      return -1;
   end Cam_Arm;

   procedure Init_Tracks (C : in out Context) is
   begin
      --  死区一开始当作零(还没证据说哪个通道推不动),边走边学
      C.Dead.Clear;
      for K in 0 .. C.Map.Arms * Chan.Per_Arm loop
         C.Dead.Append (0.0);
      end loop;
      C.Zones.Clear;
      for A in 0 .. C.Map.Arms - 1 loop
         for Cm in 0 .. C.Map.N_Cams - 1 loop
            declare
               Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cm);
               T : Zone_Track;
            begin
               T.Valid := Z.Valid;
               T.Cu := Z.Cu; T.Cv := Z.Cv; T.Z := Z.Depth;
               T.Au := Z.A.Cu; T.Av := Z.A.Cv; T.Bu := Z.B.Cu; T.Bv := Z.B.Cv;
               T.Has_Lobes := Z.Valid and then Z.N_Lobes >= 1;
               T.Known := Z.Valid;
               if Z.Valid then
                  T.Pieces (Chan.Per_Arm) := (True, Z.Cu, Z.Cv, (if Picture.Is_Nan (Z.Depth) then 0.0 else Z.Depth), Z.X0, Z.Y0, Z.X1, Z.Y1, Z.N_Lobes, Z.A.Cu, Z.A.Cv, Z.B.Cu, Z.B.Cv);
                  T.Pieces_Known (Chan.Per_Arm) := True;
               end if;
               --  开机每个通道推过一下:跟着动的那块 = 这个通道带的零件(不长在这只手上的相机里才算)
               if Cam_Arm (C, Cm) /= Integer (A) then
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     declare
                        Pi : constant Natural := (A * Chan.Per_Arm + K) * C.Map.N_Cams + Cm;
                     begin
                        if Pi < Natural (C.Map.Parts.Length) and then C.Map.Parts (Pi).Valid then
                           declare
                              P : constant Selfmap.Part := C.Map.Parts (Pi);
                           begin
                              T.Pieces (K) := (True, P.Cu, P.Cv, 0.0, P.X0, P.Y0, P.X1, P.Y1, 1, P.Cu, P.Cv, 0.0, 0.0);
                              T.Pieces_Known (K) := True;
                           end;
                        end if;
                     end;
                  end loop;
               end if;
               C.Zones.Append (T);
            end;
         end loop;
      end loop;
   end Init_Tracks;

   function Cut_Window (C : Context; Cam : Natural; F : Plug.Frame) return Long_Float is
      A : constant Integer := Cam_Arm (C, Cam);
   begin
      --  长在手上的相机斜看桌面:窗口 = 两指张幅按"指深 / 画面中位深"缩到画面深处("比这还大的不是能拿的东西");世界相机用画幅八分之一
      --  下面的 0.02 / 0.125 都是画幅的比例(无量纲):窗口的下限与上限
      if A >= 0 then
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
            Dp : Floats := F.Cams (Cam).Depth;
            Med : Long_Float;
         begin
            --  窗口至少要比正在跟的那块大半圈(倍数,无量纲),否则它一走近就被闭运算填平、只剩一圈边
            --  ⚠️ 试过"窗口跟着被跟的东西放大",错的:窗口一到画幅四分之一,桌面自己的起伏就盖过物体,
            --  深度那一路彻底切不出东西,颜色那一路接管、把墙缝和衣服切成上百块(EZ 逐步落图坐实)。窗口保持小。
            if Z.Valid and then Z.Span > 0.0 and then not Picture.Is_Nan (Z.Depth) and then F.Cams (Cam).Has_Depth then
               declare
                  Samp : Floats;
                  I : Natural := 0;
               begin
                  while I < Natural (Dp.Length) loop
                     if not Picture.Is_Nan (Dp (I)) and then Dp (I) > 0.0 then
                        Samp.Append (Dp (I));
                     end if;
                     I := I + 37;
                  end loop;
                  if Natural (Samp.Length) >= 16 then
                     Med := Picture.Quantile (Samp, 0.5);
                     if Med > 0.0 then
                        --  夹在画幅的 0.02 与 0.125 之间(比例,无量纲)
                        return Long_Float'Max (0.02, Long_Float'Min (0.125, Z.Span * (Z.Depth / Med) * 0.5));
                     end if;
                  end if;
               end;
            end if;
         end;
      end if;
      return 0.125;   --  世界相机:画幅八分之一(比例,无量纲)
   end Cut_Window;

   function Cut_Things_Raw (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Raw : Picture.Regions;
      Kept : Picture.Regions;
   begin
      --  🔴 没有深度就直接走颜色那一路(真机没有深度相机)。以前这里直接返回空 = 什么都看不见。
      if F.Cams (Cam).Has_Depth then
      Raw := Picture.Cut (F.Cams (Cam).Depth, Cw, Ch, Cut_Window (C, Cam, F), Sigma_Mult);
      --  🔴 一个都没切出来 ⇒ 换个大一号的尺子再看一遍。
      --  "鼓出来"是相对周围说的:窗口比这块东西还小的时候,这块东西【自己就是周围】,于是它鼓 0、整个消失。
      --  实测(FO):手伸到球跟前时,手腕相机里球占了小半幅画面,深度这一路一块也切不出来,退化成按颜色切出几十块碎片
      --  ⇒ 越靠近越看不见要抓的东西。尺子一路放大到半幅画面为止,先看出东西的那一档算数。
      declare
         Base : constant Long_Float := Cut_Window (C, Cam, F);
         Win : Long_Float;
         --  🔴 长在这只手上的相机:被画面切掉一角的块【一开始就算数】。
         --  这台相机跟着手走,越靠近要抓的东西,它越贴边;而我自己的胳膊在这台相机【后面】,
         --  不会从边上伸进来(手指另有扫过的像素在剔)。第三方相机则相反,胳膊天天贴边 ⇒ 那里仍然严。
         --  实测(FP):球贴住右边缘 ⇒ 整块消失 ⇒ 一连四轮身体都说"我看不见它",最后的接近根本无从谈起。
         Own : constant Boolean := Cam_Arm (C, Cam) >= 0;
      begin
         if Own then
            Raw := Picture.Cut (F.Cams (Cam).Depth, Cw, Ch, Base, Sigma_Mult, Keep_Edge => True);
         end if;
         --  🔴 尺子不是选一把,是【每一把都看一遍,合起来】。
         --  "鼓出来"是相对周围说的:窗口比这块东西小的时候,这块东西自己就是周围 ⇒ 它鼓 0、整个消失。
         --  实测(GB):球贴到 5 cm 时占满画面中央,深度这一路一块也切不出它,而画面里别的东西还切得出来
         --  ⇒ 旧的"一块都没有才换大尺子"根本不触发 ⇒ 最后一步反而瞎了。
         --  合并规则:粗尺子切出来的块,只有当它的形心还没被任何已收的块盖住时才收(不重复列)。
         Win := Base;
         while Win < 0.5 loop
            Win := Win * 2.0;
            declare
               More : constant Picture.Regions :=
                 Picture.Cut (F.Cams (Cam).Depth, Cw, Ch, Long_Float'Min (0.5, Win), Sigma_Mult, Keep_Edge => Own);
            begin
               --  🔴 粗尺子的块【吃掉】它盖住的那些细碎块,合成一个整块;盖不住任何东西就当新东西收进来;
               --  只是边角擦到、没盖住谁的中心 ⇒ 不收(那是别的东西)。
               --  两个极端都踩过:太松 ⇒ 同一个球被列两遍、清单 40 多件、编号一轮一变(GD 手停在 22 cm);
               --  太紧("一点重叠就不收")⇒ 球被一个小碎块挡住就整块消失(GE 球到了两指正前方却点不了名)。
               for R of More loop
                  declare
                     Ate : Natural := 0;
                     Keep : Picture.Regions;
                     Touch : Boolean := False;
                     Big_Swallow : Boolean := False;
                  begin
                     for Q of Raw loop
                        --  🔴 只有"大小是一个量级"的才算同一个东西的粗细两版。粗块比某个细块大十倍以上
                        --  (倍数,无量纲),它就是背景 —— 一条桌面长条 —— 不是那个东西的全貌,整块不要。
                        --  实测(GH):球被一条横贯下沿的桌面长条吃掉,球不再是能点名的东西,整炮卡死。
                        if Picture.Inside (R, Q.Cu, Q.Cv, Cw, Ch, 0.0) and then Q.Count * 10 < R.Count then
                           Big_Swallow := True;
                        end if;
                        if Picture.Inside (R, Q.Cu, Q.Cv, Cw, Ch, 0.0) and then Q.Count * 10 >= R.Count then
                           Ate := Ate + 1;          --  这个细块的中心在粗块里、大小同一量级 ⇒ 它是粗块的一部分,丢掉
                        else
                           Keep.Append (Q);
                           if R.X0 <= Q.X1 and then Q.X0 <= R.X1
                             and then R.Y0 <= Q.Y1 and then Q.Y0 <= R.Y1
                           then
                              Touch := True;        --  只擦到边,没盖住中心
                           end if;
                        end if;
                     end loop;
                     if Big_Swallow then
                        null;                        --  背景,整块不要
                     elsif Ate > 0 then
                        Raw := Keep;
                        Raw.Append (R);
                     elsif not Touch then
                        Raw.Append (R);
                     end if;
                  end;
               end loop;
            end;
         end loop;
         --  尺子放到头还是一块都没有 ⇒ 这才允许"被画面切掉一角"的块算数(严格规则下它整块消失)。
         --  放在最后一档:第三方相机里从画面外伸进来的胳膊也贴边,平时不许它变成"一件东西"。
         if Raw.Is_Empty then
            Win := Base;
            loop
               Raw := Picture.Cut (F.Cams (Cam).Depth, Cw, Ch, Long_Float'Min (0.5, Win), Sigma_Mult, Keep_Edge => True);
               exit when not Raw.Is_Empty or else Win >= 0.5;
               Win := Win * 2.0;
            end loop;
         end if;
      end;
      end if;
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
         --  🔴🔴 颜色切块只收【稳定的块】:一个真东西,门槛变一倍它的样子几乎不变;
         --  而桌面、墙面、纹理碎片会随门槛剧烈变化(实测:粗门槛把白球和棕桌并成一块,
         --  细门槛把整幅画面碎成 1112 块 —— 两头都不能用)。
         --  做法:在三档门槛(量出来那一档的 1/4、1/2、1 倍;倍数无量纲)各切一遍,
         --  只保留【在相邻那一档里也能找到一个位置和大小都差不多的块】的那些 —— 稳定 = 真东西。
         function Colour_Stable return Picture.Regions is
            R1 : constant Picture.Regions := Picture.Cut_Colour (F.Cams (Cam).RGB, Cw, Ch, Floor_C * 0.25, Picture.Min_Pixels (Cw, Ch));
            R2 : constant Picture.Regions := Picture.Cut_Colour (F.Cams (Cam).RGB, Cw, Ch, Floor_C * 0.5, Picture.Min_Pixels (Cw, Ch));
            R3 : constant Picture.Regions := Picture.Cut_Colour (F.Cams (Cam).RGB, Cw, Ch, Floor_C, Picture.Min_Pixels (Cw, Ch));
            Out_R : Picture.Regions;
            function Same_There (Rs : Picture.Regions; R : Picture.Region) return Boolean is
            begin
               for Q of Rs loop
                  if Picture.Inside (R, Q.Cu, Q.Cv, Cw, Ch, 0.0)
                    and then Q.Count * 3 >= R.Count * 2 and then R.Count * 3 >= Q.Count * 2
                  then
                     return True;      --  位置在它里面,而且大小差不到三分之一 ⇒ 这一档也认得它
                  end if;
               end loop;
               return False;
            end Same_There;
            --  🔴 再加一路:让画面自己把明暗分成两拨(Otsu,分不开就说分不开),两拨各自连通成块。
            --  白球在棕桌上、红乐高在木桌上,都是这一路一刀就切出来的;门槛是【算出来的】,不是我定的。
            --  一个全局颜色门槛切不出球:球身上有明暗渐变,门槛低了碎成几瓣,高了和桌面并成一块(HD 实测)。
            function By_Brightness return Picture.Regions is
               Gs : Floats;
               Cut_At : Long_Float;
               Hi, Lo : Bools;
               Res : Picture.Regions;
            begin
               for I in 0 .. Cw * Ch - 1 loop
                  Gs.Append (Long_Float (F.Cams (Cam).Gray.Element (I)));
               end loop;
               Cut_At := Picture.Split (Gs);
               if Picture.Is_Nan (Cut_At) then
                  return Res;      --  分不开(单峰)⇒ 这一路没有东西可给
               end if;
               Hi := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
               Lo := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
               for I in 0 .. Cw * Ch - 1 loop
                  if Long_Float (F.Cams (Cam).Gray.Element (I)) > Cut_At then
                     Hi.Replace_Element (I, True);
                  else
                     Lo.Replace_Element (I, True);
                  end if;
               end loop;
               Res := Picture.Components (Hi, Cw, Ch, Picture.Min_Pixels (Cw, Ch));
               for R of Picture.Components (Lo, Cw, Ch, Picture.Min_Pixels (Cw, Ch)) loop
                  Res.Append (R);
               end loop;
               return Res;
            end By_Brightness;
         begin
            for R of By_Brightness loop
               declare
                  Dup : Boolean := False;
               begin
                  for Q of Out_R loop
                     if Picture.Inside (Q, R.Cu, R.Cv, Cw, Ch, 0.0) then
                        Dup := True;
                     end if;
                  end loop;
                  if not Dup then
                     Out_R.Append (R);
                  end if;
               end;
            end loop;
            for R of R2 loop
               if Same_There (R1, R) or else Same_There (R3, R) then
                  declare
                     Dup : Boolean := False;
                  begin
                     for Q of Out_R loop
                        if Picture.Inside (Q, R.Cu, R.Cv, Cw, Ch, 0.0) then
                           Dup := True;
                        end if;
                     end loop;
                     if not Dup then
                        Out_R.Append (R);
                     end if;
                  end;
               end if;
            end loop;
            return Out_R;
         end Colour_Stable;
         Thin : constant Picture.Regions := Colour_Stable;
      begin
         for R of Thin loop
            declare
               --  只丢横跨整幅的(左右都贴边、或上下都贴边)= 桌面/墙;贴一条边的只是被画面切了一角,仍然是块
               Edge : constant Boolean := (R.X0 = 0 and then R.X1 >= Cw - 1) or else (R.Y0 = 0 and then R.Y1 >= Ch - 1);
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
            end if;
         end;
      end loop;
      return Kept;
   end Cut_Things_Raw;

   --  同一帧、同一台相机只切一次(颜色切块要扫全图两遍,一步里被问好几次)
   function Cut_Things (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is
      Self : constant access Context := C'Unrestricted_Access;
   begin
      if C.Cut_Seq = F.Seq and then C.Cut_Cam = Integer (Cam) then
         return C.Cut_Regs;
      end if;
      Self.Cut_Regs := Cut_Things_Raw (C, F, Cam);
      Self.Cut_Seq := F.Seq;
      Self.Cut_Cam := Integer (Cam);
      return Self.Cut_Regs;
   end Cut_Things;

   function Cell_Of (C : Context; U, V : Long_Float) return Natural is
      Col : constant Natural := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (C.Cols) - 1.0, Long_Float'Floor (U * Long_Float (C.Cols)))));
      Row : constant Natural := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (C.Rows) - 1.0, Long_Float'Floor (V * Long_Float (C.Rows)))));
   begin
      return Row * C.Cols + Col + 1;
   end Cell_Of;

   --  ── 编号表:先我身上的,再世界里的 ──
   procedure Build_Listing (C : in out Context; F : Plug.Frame; Cam : Natural; RGB : in out Buf; Text : out Unbounded_String) is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      T : Unbounded_String;
      Named_U, Named_V : Long_Float := -1.0;
      Have_Named : Boolean := False;
      function Rel (U, V : Long_Float) return String is
         Half : constant String := (if U < 0.5 then "LEFT" else "RIGHT");
      begin
         if Have_Named then
            declare
               D : constant Long_Float := Sqrt (((U - Named_U) * Long_Float (C.Cols)) ** 2 + ((V - Named_V) * Long_Float (C.Rows)) ** 2);
            begin
               --  🔴 不给脑数字:大小/距离/高度这类数,大模型判得比人差远了,而"是哪一个/什么关系"它比专门
               --  训练的模型还强。数还会主动骗人(GB:身体报"还差 2.2 步",我据此提前合爪,合了个空)。
               --  只留"远/近"这种关系词和左右半幅。
               --  只是把"几格远"翻成人话的三档,分界按【格数】说(比例,无量纲:占画面几分之几,与相机无关)
               return ", " & (if D > Long_Float (C.Cols) * 0.4 then "far from" elsif D > 1.5 then "near" else "right next to")
                      & " the thing you last named, in the " & Half & " half of the picture";
            end;
         end if;
         return ", in the " & Half & " half of the picture";
      end Rel;
      procedure Push (It : Item; Line : String; Col : Draw.Color; Thick : Natural) is
      begin
         C.Items.Append (It);
         if It.Located then
            Draw.Numbered_Box (RGB, Cw, Ch, It.X0, It.Y0, It.X1, It.Y1, Natural (C.Items.Length), Col, Thick);
         end if;
         Append (T, "  item " & Codec.Img (Natural (C.Items.Length)) & ": " & Line & ASCII.LF);
      end Push;
   begin
      C.Items.Clear;
      if C.Wld.Cams (Cam).Named >= 0 then
         declare
            Sl : constant World.Slot := World.Get (C.Wld, Cam, Natural (C.Wld.Cams (Cam).Named));
         begin
            if Sl.Seen then
               Have_Named := True;
               Named_U := (if Sl.Present then Sl.R.Cu else Sl.Shadow.Cu);
               Named_V := (if Sl.Present then Sl.R.Cv else Sl.Shadow.Cv);
            end if;
         end;
      end if;
      Append (T, "PIECES OF YOURSELF (measured just now: you moved one channel at a time and watched which part of the picture followed; you closed each hand on nothing and watched which pixels swept). Each is boxed and NUMBERED on the picture in orange:" & ASCII.LF);
      for A in 0 .. C.Map.Arms - 1 loop
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
            Tr : constant Zone_Track := (if Track_Idx (C, A, Cam) < Natural (C.Zones.Length) then C.Zones (Track_Idx (C, A, Cam)) else (others => <>));
            Own_Cam : constant Boolean := Cam_Arm (C, Cam) = Integer (A);
            Du : constant Long_Float := (if Own_Cam then 0.0 else Tr.Cu - Z.Cu);
            Dv : constant Long_Float := (if Own_Cam then 0.0 else Tr.Cv - Z.Cv);
            procedure Finger (Lb : Zone.Lobe; Which : Natural) is
               It : Item;
               --  这一瓣挪了多少:身体图给了各瓣位置就按瓣,否则整区平移
               Lu : constant Long_Float := (if Own_Cam then 0.0 elsif Tr.Has_Lobes then (if Which = 0 then Tr.Au else Tr.Bu) - Lb.Cu else Du);
               Lv : constant Long_Float := (if Own_Cam then 0.0 elsif Tr.Has_Lobes then (if Which = 0 then Tr.Av else Tr.Bv) - Lb.Cv else Dv);
            begin
               It.Kind := Finger; It.Arm := A; It.Which := Which;
               if Z.Valid and then Lb.Valid and then Tr.Valid then
                  It.Located := True;
                  It.Cu := Lb.Cu + Lu; It.Cv := Lb.Cv + Lv;
                  It.X0 := Natural (Long_Float'Max (0.0, Long_Float (Lb.X0) + Lu * Long_Float (Cw)));
                  It.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), Long_Float (Lb.X1) + Lu * Long_Float (Cw))));
                  It.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Lb.Y0) + Lv * Long_Float (Ch)));
                  It.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), Long_Float (Lb.Y1) + Lv * Long_Float (Ch))));
                  It.Depth := Tr.Z; It.Count := Lb.Count;
                  Push (It, "a finger of arm " & Codec.Img (A + 1) & " (it moves when that arm's grip channel moves), now in cell " &
                        Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & Rel (It.Cu, It.Cv) &
                        (if Own_Cam or else Tr.Known then "" else " (placed from my joints; I have not yet looked at my hand here)"), Draw.Orange, 2);
               else
                  Push (It, "a finger of arm " & Codec.Img (A + 1) & " - NOT locatable in this picture right now, do not name it", Draw.Orange, 0);
               end if;
            end Finger;
            G : Item;
         begin
            Finger (Z.A, 0);
            Finger (Z.B, 1);
            G.Kind := Grip; G.Arm := A;
            if Z.Valid and then Tr.Valid then
               G.Located := True;
               G.Cu := Tr.Cu; G.Cv := Tr.Cv; G.Depth := Tr.Z;
               G.X0 := Natural (Long_Float'Max (0.0, Long_Float (Z.X0) + Du * Long_Float (Cw)));
               G.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), Long_Float (Z.X1) + Du * Long_Float (Cw))));
               G.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Z.Y0) + Dv * Long_Float (Ch)));
               G.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), Long_Float (Z.Y1) + Dv * Long_Float (Ch))));
               Push (G, "grip " & Codec.Img (A + 1) & " - the space between the fingers of arm " & Codec.Img (A + 1) &
                     " (closing = grip close with grip_arm " & Codec.Img (A + 1) & "; a thing must sit in this box to be held), now in cell " &
                     Codec.Img (Cell_Of (C, G.Cu, G.Cv)) & Rel (G.Cu, G.Cv), Draw.Pink, 2);
            else
               Push (G, "grip " & Codec.Img (A + 1) & " (the space between the fingers of arm " & Codec.Img (A + 1) & ") - not locatable in this picture right now", Draw.Pink, 0);
            end if;
            --  全身零件:每个通道带的那一块(从那个关节往外的全部),位置按此刻位姿从身体图来
            if not Own_Cam then
               for K in 0 .. Chan.Per_Arm - 1 loop
                  declare
                     Pc : constant Schema.Part_Pos := Tr.Pieces (K);
                     It : Item;
                  begin
                     if Pc.Valid then
                        It.Kind := Piece; It.Arm := A; It.Which := K; It.Located := True;
                        It.Cu := Pc.Cu; It.Cv := Pc.Cv; It.Depth := Pc.Z;
                        It.X0 := Pc.X0; It.Y0 := Pc.Y0; It.X1 := Pc.X1; It.Y1 := Pc.Y1;
                        Push (It, "a piece of you: everything that swings when channel " & Codec.Img (K) & " of arm " & Codec.Img (A + 1) & " moves (measured), now in cell " &
                              Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & Rel (It.Cu, It.Cv) &
                              (if Tr.Pieces_Known (K) then "" else " (placed from my joints; not yet looked at here)"), Draw.Orange, 1);
                     end if;
                  end;
               end loop;
            end if;
         end;
      end loop;
      Append (T, "THINGS OUT IN THE WORLD (cut out of the depth picture; you do not know what they are called). Each is boxed and NUMBERED on the picture in green:" & ASCII.LF);
      for Si in 0 .. World.Count (C.Wld, Cam) - 1 loop
         declare
            Sl : constant World.Slot := World.Get (C.Wld, Cam, Si);
            It : Item;
         begin
            It.Slot := Si;
            if C.Wld.Holding and then C.Wld.Held_Slot = Si and then C.Wld.Held_Cam = Integer (Cam) then
               declare
                  A : constant Natural := Natural (C.Wld.Held_Arm);
                  Tr : constant Zone_Track := C.Zones (Track_Idx (C, A, Cam));
                  Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
               begin
                  It.Kind := Thing_Held; It.Arm := A; It.Located := Tr.Valid;
                  It.Cu := Tr.Cu; It.Cv := Tr.Cv; It.Depth := Tr.Z;
                  It.X0 := Z.X0; It.Y0 := Z.Y0; It.X1 := Z.X1; It.Y1 := Z.Y1;
                  It.Count := Sl.Shadow.Count; It.Height := Sl.Shadow.Height; It.Top := Sl.Shadow.Top;
                  Push (It, "the thing between the fingers of arm " & Codec.Img (A + 1) & " (it moves with that arm), now in cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)), Draw.Green, 2);
               end;
            elsif Sl.Present then
               It.Kind := Thing; It.Located := True;
               It.Cu := Sl.R.Cu; It.Cv := Sl.R.Cv; It.Depth := Sl.R.Depth; It.Height := Sl.R.Height; It.Top := Sl.R.Top; It.Count := Sl.R.Count;
               It.X0 := Sl.R.X0; It.Y0 := Sl.R.Y0; It.X1 := Sl.R.X1; It.Y1 := Sl.R.Y1;
               It.Au := Sl.R.Au; It.Av := Sl.R.Av; It.Elong := Sl.R.Elong;
               It.Gray := Picture.Mean_Gray (F.Cams (Cam).Gray, Cw, Ch, Sl.R);
               Push (It, "a thing, now in cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & Rel (It.Cu, It.Cv), Draw.Green, 2);
            elsif Sl.Seen then
               It.Kind := Thing_Remembered; It.Located := True;
               It.Cu := Sl.Shadow.Cu; It.Cv := Sl.Shadow.Cv; It.Depth := Sl.Shadow.Depth; It.Height := Sl.Shadow.Height; It.Top := Sl.Shadow.Top; It.Count := Sl.Shadow.Count;
               It.X0 := Sl.Shadow.X0; It.Y0 := Sl.Shadow.Y0; It.X1 := Sl.Shadow.X1; It.Y1 := Sl.Shadow.Y1;
               It.Au := Sl.Shadow.Au; It.Av := Sl.Shadow.Av; It.Elong := Sl.Shadow.Elong;
               Push (It, "a thing you saw before, remembered where it was last seen, cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)) &
                     " (not visible right now - probably under my hand)", Draw.Dim_Green, 1);
            else
               It.Kind := Thing_Remembered;
               Push (It, "(a slot with nothing in it right now)", Draw.Dim_Green, 0);
            end if;
         end;
      end loop;
      --  相机表
      declare
         K : Natural := 2;
      begin
         Append (T, "CAMERAS (say look = k to see through that camera next turn): 1 = this picture (camera index " & Codec.Img (Cam) & ")");
         for Ci in 0 .. C.Map.N_Cams - 1 loop
            if Ci /= Cam then
               declare
                  A : constant Integer := Cam_Arm (C, Ci);
               begin
                  Append (T, "; " & Codec.Img (K) & " = camera index " & Codec.Img (Ci) & (if A >= 0 then " (rides on arm " & Codec.Img (Natural (A) + 1) & ": when that arm moves, that whole picture changes)" else ""));
                  K := K + 1;
               end;
            end if;
         end loop;
         Append (T, ASCII.LF);
      end;
      declare
         A : constant Integer := Cam_Arm (C, Cam);
      begin
         if A >= 0 then
            Append (T, "- this picture rides on arm " & Codec.Img (Natural (A) + 1) & ": its fingers and grip stay put in this picture, the world moves when that arm moves" & ASCII.LF);
            --  🔴 身体要把【自己量不到什么】说出来。这一台相机跟着这只胳膊动 ⇒ 在它里面对齐,
            --  只说明东西在两指的正前方,而那条线转个手腕就满足了,和手离它多远无关
            --  (II 实测:在这一台里对齐了、也一直在满幅推,球就是不变大,38 步没靠近)。
            --  怎么办是脑的活(换那台不跟着这只胳膊动的相机去对齐),身体只负责把这句实话摆出来 ——
            --  不说,脑就没有理由去换,而它在这一台里会一直以为自己在进步。
            Append (T, "- from this picture alone I cannot tell how far a thing is from my fingers: this picture moves with that arm, so lining a thing up here only means it is straight ahead of my fingers, not that it is near. A picture that does NOT ride on this arm can tell me." & ASCII.LF);
         end if;
      end;
      if Have_Named then
         for I in 0 .. Natural (C.Items.Length) - 1 loop
            if C.Items (I).Kind in Thing | Thing_Remembered | Thing_Held and then C.Items (I).Slot = C.Wld.Cams (Cam).Named then
               Append (T, "- the thing you last named is item " & Codec.Img (I + 1) & ", now in cell " & Codec.Img (Cell_Of (C, Named_U, Named_V)) & ASCII.LF);
            end if;
         end loop;
      end if;
      Append (T, "- there is " & (if C.Wld.Holding then "ALREADY something" else "NOTHING") & " between your fingers right now" & ASCII.LF);
      Text := T;
   end Build_Listing;

   --  ── 被跟踪的点 ──
   type Point is record
      Arm : Natural := 0;
      Kind : Track_Kind := Piece_Pt;
      Slot : Integer := -1;
      Item_No : Natural := 0;
      Chan_K : Natural := 0;     --  带这块的通道(Chan.Per_Arm = 握合通道 ⇒ 这块是手指)
      Blob : Integer := -1;      --  这块的第几团(手指两团时一团一个点:两指各自到位,歪了就有一团不到位 —— 倾斜自然被罚)
      Cu, Cv, Z : Long_Float := 0.0;
      Tu, Tv, Tz : Long_Float := 0.0;
      Wz : Long_Float := 0.0;
      Desc : Unbounded_String;
      Box_W, Box_H : Long_Float := 0.0;
      Count : Natural := 0;
      Height : Long_Float := 0.0;
      Err0 : Long_Float := 0.0;
      Lost : Boolean := False;   --  这一步没在画面里认出它,位置是按表猜的
      Has_Meas : Boolean := False;              --  眼睛(光流)另外量到的位置,只用来修表
      Meas_U, Meas_V, Meas_Z : Long_Float := 0.0;
      Known : Boolean := True;                  --  这个位置是真看过的/离真看过的样本不超过一步 ⇒ 不是,走之前先看一眼
      Elong : Long_Float := 1.0;                --  脑点名那一刻这块的胖瘦(长轴/短轴),用来和别的块区分
      Gray : Long_Float := -1.0;                --  脑点名那一刻这块的平均灰度(< 0 = 没量到)
      Unsure : Boolean := False;                --  重新认的时候有两块一样像 ⇒ 不许自己挑,回去问脑
      Size : Long_Float := 0.0;                 --  这一块在画面里看着多大(框的边长,画幅):离得越近越大
      Ang : Long_Float := 0.0;                  --  这一块的朝向(主轴角的两倍,弧度;两倍 = 让主轴的正负两种写法算同一个)
      Tsize, Tang : Long_Float := 0.0;          --  要它看着多大 / 转到多少
      Wsize, Wang : Long_Float := 0.0;          --  这两样这一段要不要(0 = 不管)
      Steps_Err : Long_Float := 0.0;            --  上一步算出来的"还差几步"(三样都除以推一步能改多少之后的总和)
      Err_U, Err_V, Err_Z : Long_Float := 0.0;  --  拆开的五样(左右 / 上下 / 远近 / 大小 / 朝向),单位都是"还差几步"
      Err_S, Err_A : Long_Float := 0.0;
      Raw_Err : Long_Float := 0.0;              --  不随表变的差距(全是比例):画面距离 + 远近差几成 + 大小差几成 + 朝向差几成。
                                                --  判"有没有在靠近"只能用它 —— "还差几步"的刻度每步都在变,尺子一缩就看着像退步
      Par_Tu, Par_Tv : Long_Float := 0.0;       --  两团展开时,整块的目标(看清各团真实位置后按它重算各团目标)
      --  🔴 认死【脑指的那一刻它长什么样】。以前每一步都拿上一帧那块当样子,一步错一点,
      --  60 步之后整个走到墙上去了(IC 实测:手转到窗户前,身体还报"它就在我指间、大小也对")。
      --  样子只存一次,以后每一帧都和这一份比;比不上就是跟丢,不许跟到别的东西上。
      Anc : Buf;                                --  那一刻那块的灰度(半幅分辨率下的一小片)
      Anc_W, Anc_H : Natural := 0;              --  这一小片多宽多高(0 = 还没存)
      Scale : Long_Float := 1.0;                --  现在看着是那一刻的几倍(绝对,不是一步步乘出来的)
   end record;
   package Point_Vectors is new Ada.Containers.Vectors (Natural, Point);

   --  读一个点的"离相机多远"时该框多大:自己的零件用这一瓣自己的大小,世界上的块用那块自己的框。
   --  一句话:框住【要读的那个东西本身】,别框住它旁边的东西。
   --  下界都是"占画幅的多少"(比例,无量纲),只为了窗口不缩到一个像素
   function Depth_Win (P : Point; Z : Zone.Hand_Zone; Cw, Ch : Natural) return Long_Float is
     (if P.Kind = Piece_Pt then Lobe_Win (Z, Cw, Ch)
      elsif P.Box_W > 0.0 and then P.Box_H > 0.0 then Long_Float'Max (0.005, 0.25 * Long_Float'Min (P.Box_W, P.Box_H))
      else Long_Float'Max (0.005, Z.Span * 0.25));

   type Effect_Array is array (Natural range <>) of Table.Effect;
   procedure Refind_Pieces (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector);

   --  还差多少:只算画面上的距离(画幅)。远近不混进来 —— 混着求和是错的判据(LAB 2026-08-17)
   --  角度差绕回 (-π, π](两倍角的世界里,这等于把主轴的正负两种写法当成同一个)
   function Wrap (X : Long_Float) return Long_Float is
      Two_Pi : constant Long_Float := 2.0 * Ada.Numerics.Pi;
      Y : Long_Float := X;
   begin
      while Y > Ada.Numerics.Pi loop
         Y := Y - Two_Pi;
      end loop;
      while Y <= -Ada.Numerics.Pi loop
         Y := Y + Two_Pi;
      end loop;
      return Y;
   end Wrap;

   function Err_Of (P : Point) return Long_Float is
   begin
      return Sqrt ((P.Tu - P.Cu) ** 2 + (P.Tv - P.Cv) ** 2);
   end Err_Of;

   --  "看着多大"最小分得清多少:框变一个像素(画幅的 1/宽)。仿真里静止两拍画面一模一样,拿"静止时抖多少"当地板会得到 0 = 没有地板
   function Size_Floor (Cw : Natural) return Long_Float is (1.0 / Long_Float (Natural'Max (1, Cw)));
   --  朝向最小分得清多少:这块最长的那边偏一个像素(两倍角 ⇒ ×4)
   function Ang_Floor (P : Point; Cw, Ch : Natural) return Long_Float is
     (4.0 / Long_Float'Max (4.0, Long_Float'Max (P.Box_W * Long_Float (Cw), P.Box_H * Long_Float (Ch))));

   --  到位了没:画面上进了跟踪噪声,且远近的差不超过这块东西自己的尺寸(全是量出来的,没有写死的容差)
   function Reached (P : Point; Track_Floor : Long_Float) return Boolean is
      Tol : constant Long_Float := Long_Float'Max (P.Height, Long_Float'Max (P.Box_W, P.Box_H) * P.Z);
   begin
      if Err_Of (P) > Track_Floor then
         return False;
      end if;
      if P.Wsize > 0.0 and then P.Tsize > 0.0 and then abs (P.Tsize - P.Size) / P.Tsize > 0.25 then
         return False;   --  看着差过四分之一就还没到(比例,无量纲)
      end if;
      if P.Wz > 0.0 and then P.Z > 0.0 and then not Picture.Is_Nan (P.Tz) then
         return abs (P.Tz - P.Z) <= Long_Float'Max (Tol, 1.0e-9);
      end if;
      return True;
   end Reached;

   function Held_Now (C : Context; Arm : Natural) return Integer is
     (if C.Wld.Holding and then C.Wld.Held_Arm = Integer (Arm) then C.Wld.Held_Slot else -1);

   function Find_Effect (C : Context; Arm, Cam : Natural; Kind : Track_Kind; Chan_K : Natural; Blob : Integer := -1) return Integer is
   begin
      for I in 0 .. Natural (C.Tables.Length) - 1 loop
         if C.Tables (I).Arm = Arm and then C.Tables (I).Cam = Cam and then C.Tables (I).Kind = Kind and then C.Tables (I).Chan_K = Chan_K and then C.Tables (I).Blob = Blob
           and then C.Tables (I).Held = Held_Now (C, Arm) then
            return I;
         end if;
      end loop;
      return -1;
   end Find_Effect;

   procedure Store_Effect (C : in out Context; Arm, Cam : Natural; Kind : Track_Kind; Chan_K : Natural; Blob : Integer; E : Table.Effect; Trust : Table.Mask; Reach : Table.Vec := Unit_Reach;
                           Pose : Plug.Arm_Pose := [others => 0.0]; Has_Pose : Boolean := False) is
      I : constant Integer := Find_Effect (C, Arm, Cam, Kind, Chan_K, Blob);
      Old : constant Integer := I;
      Se : Stored_Effect := (Arm, Cam, Kind, Chan_K, Blob, E, Trust, Reach, Pose, Has_Pose, Held_Now (C, Arm));
   begin
      if not Has_Pose and then Old >= 0 then
         Se.Pose := C.Tables (Natural (Old)).Pose;      --  没带位姿的更新:沿用这张表原来量的位姿
         Se.Has_Pose := C.Tables (Natural (Old)).Has_Pose;
      end if;
      if I >= 0 then
         C.Tables.Replace_Element (Natural (I), Se);
      else
         C.Tables.Append (Se);
      end if;
   end Store_Effect;

   --  感觉:按此刻的位姿,从身体图算出每只手在每台(不长在它上面的)相机里的两瓣位置 —— 不看画面。
   --  Known = 离最近的真看过的样本不超过一步核实过的步幅;超了就是生地,走过去要看一眼(抖手指)。
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
                           Tr.Au := Clamp (Gp.B0u + Sa (0)); Tr.Av := Clamp (Gp.B0v + Sa (1));
                           Tr.Bu := Clamp (Gp.B1u + Sb (0)); Tr.Bv := Clamp (Gp.B1v + Sb (1));
                           Tr.Has_Lobes := Gp.N_Blobs >= 1;
                           if Gp.N_Blobs >= 2 then
                              Tr.Cu := (Tr.Au + Tr.Bu) / 2.0; Tr.Cv := (Tr.Av + Tr.Bv) / 2.0;
                           else
                              Tr.Cu := Tr.Au; Tr.Cv := Tr.Av;
                           end if;
                           if Gp.Z > 0.0 then
                              Tr.Z := Gp.Z + (if Gp.N_Blobs >= 2 then (Sa (2) + Sb (2)) / 2.0 else Sa (2));
                           end if;
                           Tr.Known := True;
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

   --  重新定位一个点:握区靠光流平流(世界相机)/固定(自己的手上相机);世界块重切后就近对上
   procedure Retrack (C : in out Context; F : Plug.Frame; Cam : Natural; Before : Buf; P : in out Point; Pred_U, Pred_V : Long_Float; Moved_Arm : Boolean; Pred_Z : Long_Float := -1.0) is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
   begin
      P.Lost := False;
      --  🔴 出了画面的点没有意义:它的位置是编出来的,而解算会一本正经地朝它收敛。
      --  实测(GK):被跟的点跑到 (0.63,1.11) 和 (0.85,1.48)(竖直超过 1 = 画面下沿以外),
      --  身体报"差 0.8 步就到了",而真正的球在画面左边好好待着,手一路往外走。
      --  删掉"不许推出画面"那道闸之后,这个洞就没人堵了 ⇒ 在这里堵:夹回画面里,并记成跟丢。
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
               Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
               Old_Z : constant Long_Float := P.Z;
            begin
               A.Reserve_Capacity (Ada.Containers.Count_Type (Hw * Hh));
               B.Reserve_Capacity (Ada.Containers.Count_Type (Hw * Hh));
               for Y in 0 .. Hh - 1 loop
                  for X in 0 .. Hw - 1 loop
                     A.Append (Before.Element ((2 * Y) * Cw + 2 * X));
                     B.Append (F.Cams (Cam).Gray.Element ((2 * Y) * Cw + 2 * X));
                  end loop;
               end loop;
               Fl := Flow.Compute (A, B, Hw, Hh, 3, 30);
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
                     --  深度读在这一瓣自己的位置上(区心是两指之间的空,读到的是桌面);窗口 = 这一瓣自己的大小
                     Win : constant Long_Float := Lobe_Win (Z, Cw, Ch);
                     Zd : constant Long_Float := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, P.Cu, P.Cv, Win);
                  begin
                     if not Picture.Is_Nan (Zd) then
                        --  一步之内深度跳了超过"预测的变化 + 距离的一成"(比例,无量纲)⇒ 读到的不是我的手指,留预测。
                        --  🔴 没有预测值时这道闸以前【整条失效】,于是任何读数都收:FS 实测手指的"离相机多远"
                        --  一步从 0.454 m 跳到 0.010 m(离相机一厘米,物理上不可能),抓握的高低判据当场作废。
                        --  没有预测就退回"一步最多变一成",而不是不管。
                        if Old_Z <= 0.0 then
                           P.Z := Zd;
                        elsif Pred_Z <= 0.0 then
                           P.Z := (if abs (Zd - Old_Z) <= 0.1 * Old_Z then Zd else Old_Z);
                        elsif abs (Zd - Pred_Z) <= abs (Pred_Z - Old_Z) + 0.1 * Old_Z then
                           P.Z := Zd;
                        else
                           P.Z := Pred_Z;
                        end if;
                     end if;
                     --  🔴 "离相机多远"不可能 ≤ 0。预测是线性外推,越推越小就会推过零点,
                     --  而一旦变成负数,上面那道"一步不许跳太多"的闸(它以 Old_Z > 0 为前提)就【永久失效】,
                     --  之后什么读数都收(FY 实测:深 -0.032 之后整段乱走)。留住上一份正数。
                     if P.Z <= 0.0 then
                        P.Z := Old_Z;
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
               for I in 0 .. Natural (Regs.Length) - 1 loop
                  declare
                     R : constant Picture.Region := Regs (I);
                     D : constant Long_Float := Sqrt ((R.Cu - Pred_U) ** 2 + (R.Cv - Pred_V) ** 2);
                     U : constant Long_Float := Unlike (R);
                  begin
                     if D <= Tol then
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
                     P.Size := Sqrt (Long_Float'Max (0.0, P.Box_W * P.Box_H));
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
                  --  🔴 没有切块可以对上(没有深度、颜色也切不出来)⇒ 靠画面本身把这一小片追下去:
                  --  算光流,在这一点周围取平均位移,把点挪过去。这样【脑指出来的那个东西】不需要任何分割
                  --  就能一直被跟住 —— 认东西是脑的活,跟住是身体的活。
                  declare
                     Hw : constant Natural := Cw / 2;
                     Hh : constant Natural := Ch / 2;
                     A, B : Buf;
                     Fl : Flow.Field;
                     Du, Dv : Long_Float;
                  begin
                     A.Reserve_Capacity (Ada.Containers.Count_Type (Hw * Hh));
                     B.Reserve_Capacity (Ada.Containers.Count_Type (Hw * Hh));
                     for Y in 0 .. Hh - 1 loop
                        for X in 0 .. Hw - 1 loop
                           A.Append (Before.Element ((2 * Y) * Cw + 2 * X));
                           B.Append (F.Cams (Cam).Gray.Element ((2 * Y) * Cw + 2 * X));
                        end loop;
                     end loop;
                     pragma Unreferenced (Fl, Du, Dv);
                     --  🔴 拿它【上一帧的样子】去对,而不是取一片光流的平均:平均会被旁边的东西带走,
                     --  实测十步左右就飘到墙上、飘到剪刀上,而且飘了不吭声。
                     --  做法:在预测位置周围搜一圈,找和上一帧那块最像的位置(灰度差平方和最小);
                     --  最像的那个也不够像 ⇒ 明说跟丢,不许悄悄跟到别的东西上。
                     declare
                        --  模板 = 【脑指的那一刻这块长什么样】,存下来就不再改(见 Point 里的说明)。
                        --  还没存过就用这一刻的:取这块自己的半个身子,再小也有画幅的百分之三(比例,无量纲)
                        Pw : constant Integer :=
                          (if P.Anc_W > 0 then (P.Anc_W - 1) / 2
                           else Integer'Max (3, Integer (Long_Float'Max (P.Box_W, 0.03) * Long_Float (Hw) * 0.5)));
                        Ph : constant Integer :=
                          (if P.Anc_H > 0 then (P.Anc_H - 1) / 2
                           else Integer'Max (3, Integer (Long_Float'Max (P.Box_H, 0.03) * Long_Float (Hh) * 0.5)));
                        Cx : constant Integer := Integer (P.Cu * Long_Float (Hw));
                        Cy : constant Integer := Integer (P.Cv * Long_Float (Hh));
                        --  "够不够像"的门槛用【这台相机静止时自己抖多少】(量出来的)算(倍数,无量纲)
                        Noise_G : constant Long_Float :=
                          (if Cam < Natural (C.Map.Pic_Floor.Length) and then C.Map.Pic_Floor (Cam) > 0
                           then Long_Float (C.Map.Pic_Floor (Cam)) else 4.0);
                        Bad_D : constant Long_Float := 0.5;   --  相关不到一半就不算是它(比例,无量纲)
                        Cs : constant Positive := 4;   --  粗搜隔几个像素取一个(次数,无量纲)
                        --  比一个位置:上一帧那块(A 里以 Cx,Cy 为心)对这一帧挪了 Ox,Oy 又放大 Sc 的那块(B 里)。
                        --  Step = 隔几个像素取一个;只和"长得像它"的候选比 —— 平均亮度要接近,起伏也要接近,
                        --  否则平整的墙面和木纹到处都能凑出一个"最像"(HT 实测:模板锁到墙上,手腕越抬越高)。
                        function Score (Ox, Oy : Integer; Sc : Long_Float; Step : Positive) return Long_Float is
                           N_Pix : Natural := 0;
                           Sa, Sb, Qa, Qb, Sab : Long_Float := 0.0;
                           Ny : constant Integer := Integer'Max (1, Ph / Step);
                           Nx : constant Integer := Integer'Max (1, Pw / Step);
                        begin
                           for Yi in -Ny .. Ny loop
                              for Xi in -Nx .. Nx loop
                                 declare
                                    X : constant Integer := Xi * Step;
                                    Y : constant Integer := Yi * Step;
                                    Bx2 : constant Integer := Cx + Integer (Long_Float (X) * Sc) + Ox;
                                    By2 : constant Integer := Cy + Integer (Long_Float (Y) * Sc) + Oy;
                                 begin
                                    if Bx2 >= 0 and then By2 >= 0 and then Bx2 < Hw and then By2 < Hh then
                                       declare
                                          Va : constant Long_Float :=
                                            Long_Float (P.Anc.Element ((Ph + Y) * (2 * Pw + 1) + (Pw + X)));
                                          Vb : constant Long_Float := Long_Float (B.Element (By2 * Hw + Bx2));
                                       begin
                                          Sa := Sa + Va; Sb := Sb + Vb;
                                          Qa := Qa + Va * Va; Qb := Qb + Vb * Vb;
                                          Sab := Sab + Va * Vb;
                                          N_Pix := N_Pix + 1;
                                       end;
                                    end if;
                                 end;
                              end loop;
                           end loop;
                           --  🔴 重叠不够就不算"像":候选挪到画面边上时只剩一两个像素落在画面里,
                           --  按像素平均的差值反而最小 ⇒ 整幅搜索必然赢在边角上(IA 实测:一次探针
                           --  "跑了 0.9955 画幅",点被甩到 v=1.000 和 v=0.000)。至少要有一半模板落在画面里。
                           if N_Pix * 2 < (2 * Nx + 1) * (2 * Ny + 1) then
                              return Long_Float'Last;
                           end if;
                           declare
                              Ma : constant Long_Float := Sa / Long_Float (N_Pix);
                              Mb : constant Long_Float := Sb / Long_Float (N_Pix);
                              Da : constant Long_Float := Sqrt (Long_Float'Max (0.0, Qa / Long_Float (N_Pix) - Ma * Ma));
                              Db : constant Long_Float := Sqrt (Long_Float'Max (0.0, Qb / Long_Float (N_Pix) - Mb * Mb));
                                 begin
                                    --  🔴 比"像不像"用【相关】,不用灰度差:灰度差有一个致命的偏心 ——
                                    --  模板缩小一点采到的是更平滑的一片,差值自然更小 ⇒ 每一步都判"它变小了",
                                    --  框按 0.94 一路乘下去缩成零,而"看着多大"那一项正比于 1/框 ⇒ 冲到十万
                                    --  (IB 实测:大小那一项 220 → 108331,差距一路涨,手越走越偏)。
                                    --  相关系数把两边各自的亮度和起伏都除掉,缩放不再天然占便宜。
                                    --  另一半好处:【平的一片没有相关可言】—— 墙面、木纹的起伏低于相机自己的抖动,
                                    --  直接出局,不用再单独写"别锁到墙上"那条规矩。
                                    if Da <= Noise_G or else Db <= Noise_G then
                                       return Long_Float'Last;
                                    end if;
                                    return 1.0 - (Sab / Long_Float (N_Pix) - Ma * Mb) / (Da * Db);
                                 end;
                        end Score;
                        Best_D : Long_Float := Long_Float'Last;
                        Bx, By : Integer := 0;
                        Best_S : Long_Float := P.Scale;   --  现在看着是那一刻的几倍(绝对值,不是一步步乘出来的)
                        Base_D : Long_Float := Long_Float'Last;   --  原地不动有多像
                        Sum_D : Long_Float := 0.0;     --  整幅画面上"随便一个位置"平均多像
                        N_Try : Natural := 0;
                        Sx0, Sy0 : Integer := 0;
                        --  尺度搜索:五档,最低 0.94,每档三个百分点(都是比例,无量纲)。
                        --  太粗就当不了控制信号 —— 一步要么判"没变"要么判"变了一成半"。
                        Sc_Lo : constant Long_Float := 0.94;
                        Sc_Step : constant Long_Float := 0.03;
                        --  🔴 整幅搜索必须带一条【它不会瞬移】:同样像的两处,信离预测近的那一处。
                        --  不带这一条,画面里任何一块浅色的东西都可能在某一帧比真身更像 ——
                        --  IE 实测:跟的框从球跳到剪刀那只浅绿手柄上,而球就在旁边好好地待着。
                        --  罚 = 离预测多远(按模板自己的大小折算,比例,无量纲);远一个模板就贵一倍。
                        function Near (Ox, Oy : Integer; S : Long_Float) return Long_Float is
                           Dx : constant Long_Float := Long_Float (Cx + Ox) - Pred_U * Long_Float (Hw);
                           Dy : constant Long_Float := Long_Float (Cy + Oy) - Pred_V * Long_Float (Hh);
                           R : constant Long_Float := Long_Float (Integer'Max (Pw, Ph));
                        begin
                           if S >= Long_Float'Last then
                              return S;
                           end if;
                           return S * (1.0 + Sqrt (Dx * Dx + Dy * Dy) / R);
                        end Near;
                     begin
                        --  第一次跟这块:把它此刻的样子存下来,以后每一帧都和这一份比。
                        if P.Anc_W = 0 then
                           P.Anc.Clear;
                           for Y in -Ph .. Ph loop
                              for X in -Pw .. Pw loop
                                 declare
                                    Ax : constant Integer := Integer'Max (0, Integer'Min (Hw - 1, Cx + X));
                                    Ay : constant Integer := Integer'Max (0, Integer'Min (Hh - 1, Cy + Y));
                                 begin
                                    P.Anc.Append (A.Element (Ay * Hw + Ax));
                                 end;
                              end loop;
                           end loop;
                           P.Anc_W := 2 * Pw + 1; P.Anc_H := 2 * Ph + 1;
                        end if;
                        Base_D := Near (0, 0, Score (0, 0, P.Scale, 1));
                        --  🔴 先在粗的一档上把【整幅画面】搜一遍,再回到细的一档只在赢家附近搜。
                        --  手上的相机一动,整幅画面都在跑,固定半径的搜索圈根本追不上 —— HZ 实测:
                        --  探针把某个关节推到 0.53,球早跑出搜索圈,身体记成"这个通道推了没反应",表里写进
                        --  一列零 ⇒ 解算认为哪个通道都没用,连着 12 步一个命令都没发出来,手一动没动。
                        for Oy in -(Hh / Cs) .. Hh / Cs loop
                           for Ox in -(Hw / Cs) .. Hw / Cs loop
                              declare
                                 S : constant Long_Float := Near (Ox * Cs, Oy * Cs, Score (Ox * Cs, Oy * Cs, P.Scale, Cs));
                              begin
                                 if S < Long_Float'Last then
                                    Sum_D := Sum_D + S; N_Try := N_Try + 1;
                                    if S < Best_D then
                                       Best_D := S; Sx0 := Ox * Cs; Sy0 := Oy * Cs;
                                    end if;
                                 end if;
                              end;
                           end loop;
                        end loop;
                        --  细搜:在粗搜赢的那一点周围一个粗格之内,连【尺度】一起搜。
                        --  尺度五档,每档三个百分点(比例,无量纲):模板这一帧变大还是变小,就是
                        --  "离得越近看着越大"那条距离信号 —— 没有深度之后这是身体唯一往前的感觉。
                        Best_D := Long_Float'Last;
                        for Si in 0 .. 4 loop
                           declare
                              Sc : constant Long_Float := P.Scale * (Sc_Lo + Sc_Step * Long_Float (Si));
                           begin
                              for Oy in -Cs .. Cs loop
                                 for Ox in -Cs .. Cs loop
                                    declare
                                       S : constant Long_Float := Near (Sx0 + Ox, Sy0 + Oy, Score (Sx0 + Ox, Sy0 + Oy, Sc, 1));
                                    begin
                                       if S < Best_D then
                                          Best_D := S; Bx := Sx0 + Ox; By := Sy0 + Oy; Best_S := Sc;
                                       end if;
                                    end;
                                 end loop;
                              end loop;
                           end;
                        end loop;
                        --  🔴 尺度要【连续】地估:五档里挑一档是个台阶,小的真变化跨不过台阶就被判成"没变",
                        --  于是表里"推一下它看着变大多少"越学越小,而"还差几步 = 差多少 ÷ 推一下能改多少"
                        --  就炸上天(IF 实测:第 1–9 步差距 0.353 → 0.164 一路在靠近,第 10 步起大小那一项
                        --  4 → 20 → 262 → 1342,手随即开始乱转)。样子是认死的那一份,尺度是【相对它的绝对倍数】,
                        --  不会一步步乘出漂移,所以可以放心在赢的那一档和左右两档之间插值。
                        declare
                           Sv : array (0 .. 4) of Long_Float;
                           Bi : Natural := 2;
                           Half : constant Long_Float := 0.5;   --  半档(比例,无量纲)
                        begin
                           for Si in 0 .. 4 loop
                              Sv (Si) := Near (Bx, By, Score (Bx, By, P.Scale * (Sc_Lo + Sc_Step * Long_Float (Si)), 1));
                           end loop;
                           for Si in 0 .. 4 loop
                              if Sv (Si) < Sv (Bi) then
                                 Bi := Si;
                              end if;
                           end loop;
                           Best_S := P.Scale * (Sc_Lo + Sc_Step * Long_Float (Bi));
                           if Bi > 0 and then Bi < 4
                             and then Sv (Bi - 1) < Long_Float'Last and then Sv (Bi + 1) < Long_Float'Last
                           then
                              declare
                                 Den : constant Long_Float := Sv (Bi - 1) - 2.0 * Sv (Bi) + Sv (Bi + 1);
                              begin
                                 if Den > 0.0 then
                                    --  抛物线顶点,只许在自己这一档里挪(半档;比例,无量纲)
                                    Best_S := P.Scale *
                                      (Sc_Lo + Sc_Step * (Long_Float (Bi)
                                       + Long_Float'Max (-Half, Long_Float'Min (Half,
                                           Half * (Sv (Bi - 1) - Sv (Bi + 1)) / Den))));
                                 end if;
                              end;
                           end if;
                        end;
                        --  🔴 认得住要满足两条:①最像的那个本身够像(不超过噪声门槛,或者比原地明显好);
                        --  ②它要明显比【整幅画面上随便一个位置】好(不到平均的一半;比例,无量纲)——
                        --  否则说明这一片到处都差不多(木纹、墙面),最像的只是巧合(HN 实测:锁到球拍、锁到墙)。
                        if Best_D = Long_Float'Last
                          or else (Best_D > Bad_D and then Best_D > Base_D * 0.9)
                          or else (N_Try > 0 and then Best_D > 0.5 * (Sum_D / Long_Float (N_Try)))
                        then
                           P.Cu := Pred_U; P.Cv := Pred_V; P.Lost := True;
                        else
                           P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cu + Long_Float (Bx) / Long_Float (Hw)));
                           P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cv + Long_Float (By) / Long_Float (Hh)));
                           P.Scale := Best_S;
                           P.Box_W := Long_Float (P.Anc_W) / Long_Float (Hw) * Best_S;
                           P.Box_H := Long_Float (P.Anc_H) / Long_Float (Hh) * Best_S;
                           P.Size := Sqrt (Long_Float'Max (0.0, P.Box_W * P.Box_H));
                           P.Lost := False;
                        end if;
                     end;
                  end;
               end if;
            end;
      end case;
      --  统一收口:任何路径算出来的位置都不许留在画面外(见上面的说明)
      if P.Cu < 0.0 or else P.Cu > 1.0 or else P.Cv < 0.0 or else P.Cv > 1.0 then
         P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cu));
         P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cv));
         P.Lost := True;
      end if;
   end Retrack;

   --  发一步并等稳;返回实到(通道)
   procedure Step_Arm (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; A : Table.Vec; Jaw : Floats;
                       Delivered : out Table.Vec; Ok : out Boolean; Quick : Boolean := False; Watch : Selfmap.Watcher := null) is
      P0 : constant Plug.Arm_Pose := F.EE (Arm);
      Frames : Natural;
   begin
      Selfmap.Go (L, C.Map, Arm, Chan.Compose (P0, A), Jaw, F, Delivered, Frames, Ok, Quick, Watch);
   end Step_Arm;

   --  没有表的点(同一只手的几个点一起):每个通道推一下量一列。幅度从开机看得见的那一档起,翻倍到每个点在画面里
   --  跑过 4 个跟踪地板、或深度变过深度地板为止(倍数,无量纲;EF 实测:最小可见幅度量出的列全是噪声,解算据此拧手腕);
   --  深度地板 = 这一点连着两拍读深度抖多少的 4 倍,再小也有距离的百分之一(比例,无量纲);翻到上限还看不出动的通道,这一段不用它。推完推回起点。
   procedure Probe_Effects (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector;
                            Effs : in out Effect_Array; Trust : out Table.Mask; Ok : out Boolean) is
      Arm : constant Natural := Pts (0).Arm;
      P0 : constant Plug.Arm_Pose := F.EE (Arm);
      Jaw : Floats;
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Floor_Px : constant Long_Float := 4.0 / Long_Float (Cw);   --  跟踪地板:4 个像素(倍数,无量纲)
      Floor_Z : array (0 .. Natural (Pts.Length) - 1) of Long_Float := [others => 0.0];
      --  新加的两行也要有自己的噪声地板:探针那一点点幅度下,"看着多大/朝向"的变化可能比噪声还小,
      --  连符号都会是反的 ⇒ 身体照着反方向走(FB 实测:它一路往后退)。地板 = 静止两拍的抖动的 4 倍(倍数,无量纲)
      Floor_S : array (0 .. Natural (Pts.Length) - 1) of Long_Float := [others => 0.0];
      Floor_A : array (0 .. Natural (Pts.Length) - 1) of Long_Float := [others => 0.0];
      Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
   begin
      Trust := [others => False];
      Jaw.Append (Selfmap.Jaw_Of (F, Arm));
      for I in 0 .. Natural (Pts.Length) - 1 loop
         Table.Reset (Effs (I), Chan.Per_Arm, 1.0);
         for K in 0 .. Chan.Per_Arm - 1 loop
            declare
               Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Arm * Chan.Per_Arm + K));
            begin
               Table.Set_Prior (Effs (I), K, 100.0 / (Am * Am));   --  先验按探针幅度定(倍数,无量纲)
            end;
         end loop;
      end loop;
      --  深度读数地板:什么都不做,连着两拍在各点读深度
      if F.Cams (Cam).Has_Depth then
         declare
            Z1 : array (0 .. Natural (Pts.Length) - 1) of Long_Float := [others => -1.0];
            Ok2 : Boolean;
         begin
            for I in 0 .. Natural (Pts.Length) - 1 loop
               --  读深窗口 = 要读的那个东西自己的大小
               Z1 (I) := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, Pts (I).Cu, Pts (I).Cv, Depth_Win (Pts (I), Z, Cw, Ch));
            end loop;
            declare
               Was0 : constant Point_Vectors.Vector := Pts;
               Before0 : constant Buf := F.Cams (Cam).Gray;
            begin
               Selfmap.Idle (L, F, 1, Ok2);
               --  静止一拍,量"看着多大/朝向"自己抖多少
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  declare
                     P2 : Point := Pts (I);
                  begin
                     Retrack (C, F, Cam, Before0, P2, Was0 (I).Cu, Was0 (I).Cv, False);
                     Floor_S (I) := Long_Float'Max (4.0 * abs (P2.Size - Was0 (I).Size), Size_Floor (Cw));
                     Floor_A (I) := Long_Float'Max (4.0 * abs (Wrap (P2.Ang - Was0 (I).Ang)), Ang_Floor (Was0 (I), Cw, Ch));
                  end;
               end loop;
            end;
            for I in 0 .. Natural (Pts.Length) - 1 loop
               declare
                  --  读深窗口 = 要读的那个东西自己的大小
                  Z2 : constant Long_Float := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, Pts (I).Cu, Pts (I).Cv, Depth_Win (Pts (I), Z, Cw, Ch));
                  Zr : constant Long_Float := (if Pts (I).Z > 0.0 then Pts (I).Z else 1.0);
               begin
                  --  地板 = 两拍读深抖动的 4 倍(倍数,无量纲),再小也有距离的百分之一(比例,无量纲)
                  if not Picture.Is_Nan (Z1 (I)) and then not Picture.Is_Nan (Z2) then
                     Floor_Z (I) := Long_Float'Max (4.0 * abs (Z1 (I) - Z2), 0.01 * Zr);
                  else
                     Floor_Z (I) := 0.01 * Zr;
                  end if;
               end;
            end loop;
         end;
      end if;
      Ok := True;
      Put_Line ("[身]   这些点还没有响应表 ⇒ 六个通道各推一下量列(幅度从开机看得见的那一档起翻倍,到点真的动过地板为止)");
      --  六个通道一起解:转动不禁(owner 2026-09-07:禁了就永远和桌面平行,格斗全成直线)。让转动有对错的是"两根手指各自到位":
      --  转歪了必有一指不到位;让转动不比平移便宜的是按各自探针幅度计价。
      for K in 0 .. Chan.Per_Arm - 1 loop
         declare
            Chn : constant Natural := Arm * Chan.Per_Arm + K;
            Amp : Long_Float := C.Map.Amp (Chn);
            --  🔴 探针能推多大:一路翻倍,直到【点在画面里跑满一个跟踪窗】—— 眼睛还跟得住的最大一下。
            --  以前卡死在"开机那一档的 2 倍",只能翻一次,于是"看着多大"这一行永远变化不过它自己的地板,
            --  那一列永远是零(GR 实测),而深度读数又不重复 ⇒ 身体手里没有任何能用的"我在靠近吗"。
            --  最多翻四次(次数,无量纲)兜底,免得某个通道怎么推画面都不动时一直翻下去。
            Cap_Amp : constant Long_Float := C.Map.Amp (Chn) * 16.0;
         begin
            if not C.Map.Seen (Chn) or else Amp <= 0.0 then
               Put_Line ("[身]     通道" & Natural'Image (Chn) & " 开机时没看见它动,这一列留零");
            else
               loop
                  declare
                     A : Table.Vec := Table.Zero_Vec;
                     Before : constant Buf := F.Cams (Cam).Gray;
                     Was : constant Point_Vectors.Vector := Pts;
                     Deliv, Back : Table.Vec;
                     Ok2 : Boolean;
                     Frames : Natural;
                     Seen_Enough : Boolean := False;   --  🔴 只要【有一个】被跟的点真的动过,这一列就量到了
                     N_Moved : Natural := 0;
                     Size_Seen : Boolean := False;     --  这一下推得够不够大,让"看着多大"那一行也量到了
                     Ran_Max : Long_Float := 0.0;
                  begin
                     A (K) := Amp;
                     Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok2);
                     if not Ok2 then
                        Ok := False;
                        return;
                     end if;
                     for I in 0 .. Natural (Pts.Length) - 1 loop
                        declare
                           P : Point := Pts (I);
                           W0 : constant Point := Was (I);
                           Ran, Dz : Long_Float;
                        begin
                           Retrack (C, F, Cam, Before, P, W0.Cu, W0.Cv, True);
                           Ran := Sqrt ((P.Cu - W0.Cu) ** 2 + (P.Cv - W0.Cv) ** 2);
                           Dz := (if P.Z > 0.0 and then W0.Z > 0.0 then abs (P.Z - W0.Z) else 0.0);
                           Ran_Max := Long_Float'Max (Ran_Max, Ran);
                           --  🔴 跟丢了就是【什么也没量到】,不许当成"这个通道不动它"记成一列零。
                           --  没有深度的时候 Dz 恒为 0 而 Floor_Z 也是 0,于是 "Dz >= Floor_Z" 恒真 ——
                           --  一次跟丢的探针会被当成一次成功的测量写进表(HZ 实测:整张表全零,
                           --  解算说"哪个通道都没用",连着 12 步一个命令都没发出来)。
                           if abs Deliv (K) > C.Map.EE_Noise and then not P.Lost
                             and then (Ran >= Floor_Px
                                       or else (P.Z > 0.0 and then W0.Z > 0.0 and then Dz >= Floor_Z (I)))
                           then
                              declare
                                 Col : Table.Vec3;
                              begin
                                 Col (0) := (P.Cu - W0.Cu) / Deliv (K);
                                 Col (1) := (P.Cv - W0.Cv) / Deliv (K);
                                 Col (2) := (if P.Z > 0.0 and then W0.Z > 0.0 then (P.Z - W0.Z) / Deliv (K) else 0.0);
                                 --  推一下这块看着变大变小多少、转了多少(圆的东西转不出来 ⇒ 这一列恒零 ⇒ 自动不参与)
                                 --  只有变化过了自己的噪声地板才敢写进表,否则这一格留零(留零 = 归一时这一行自动不参与)
                                 if P.Size > 0.0 and then W0.Size > 0.0 and then abs (P.Size - W0.Size) > Floor_S (I) then
                                    Col (3) := (P.Size - W0.Size) / Deliv (K);
                                    Size_Seen := True;
                                 else
                                    Col (3) := 0.0;
                                 end if;
                                 Col (4) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (Wrap (P.Ang - W0.Ang)) > Floor_A (I)
                                             then Wrap (P.Ang - W0.Ang) / Deliv (K) else 0.0);
                                 Table.Set_Col (Effs (I), K, Col);
                              end;
                              Seen_Enough := True;
                              N_Moved := N_Moved + 1;
                           end if;
                           --  🔴 没动过的点这一列【留零】,而留零本身就是一次正确的测量("这个通道不动它"),
                           --  归一时它自动不参与。以前是"任一个点没动 ⇒ 整条通道作废",于是两指里被挡住一根
                           --  就把一整个自由度扔掉:FQ 实测 6 个通道扔掉 5 个,只剩 1 个还想管三个方向,
                           --  于是"还差几步"算出 5528 步、手来回摆。同一条教训 LAB 里记过一次(丢点要按【全部】判)。
                           Pts.Replace_Element (I, P);
                        end;
                     end loop;
                     if Seen_Enough then
                        Trust (K) := True;
                        Put_Line ("[身]     通道" & Natural'Image (Chn) & ":命令 " & Codec.Fmt (Amp, 4) & " 实到 " & Codec.Fmt (Deliv (K), 4) & " ⇒ " &
                                  Natural'Image (N_Moved) & "/" & Natural'Image (Natural (Pts.Length)) & " 个点动了,最多的跑了 " &
                                  Codec.Fmt (Ran_Max, 4) & " 画幅,深度变 " & Codec.Fmt ((if Pts (0).Z > 0.0 and then Was (0).Z > 0.0 then Pts (0).Z - Was (0).Z else 0.0), 4));
                     end if;
                     declare
                        Before2 : constant Buf := F.Cams (Cam).Gray;
                        Z_Out : array (0 .. Natural'Max (0, Natural (Pts.Length) - 1)) of Long_Float :=
                          [others => -1.0];
                     begin
                        for I in 0 .. Natural (Pts.Length) - 1 loop
                           Z_Out (I) := Pts (I).Z;      --  推出去之后读到的远近
                        end loop;
                        Selfmap.Go (L, C.Map, Arm, P0, Jaw, F, Back, Frames, Ok2);
                        if not Ok2 then
                           Ok := False;
                           return;
                        end if;
                        for I in 0 .. Natural (Pts.Length) - 1 loop
                           declare
                              P : Point := Pts (I);
                              Zb : Long_Float;
                           begin
                              Retrack (C, F, Cam, Before2, P, Was (I).Cu, Was (I).Cv, True);
                              Zb := P.Z;               --  推回起点之后又读一次
                              --  🔴 远近这一列必须【来回都对得上】才算量到:推出去改了多少、推回来就该改回多少。
                              --  一次抖动就能把这一列写成真实值的几十倍 —— 实测(GN):表说"推一下改 16 cm",
                              --  实际一步只改几毫米,于是身体永远以为"再一两步就到",只迈一小步,永远压不进去。
                              --  对不上就把这一格留零(留零 = 归一时这一行自动不参与),而不是写一个假的大数。
                              if Was (I).Z > 0.0 and then Z_Out (I) > 0.0 and then Zb > 0.0 then
                                 declare
                                    D_Out : constant Long_Float := Z_Out (I) - Was (I).Z;
                                    D_Back : constant Long_Float := Zb - Z_Out (I);
                                 begin
                                    --  🔴 只杀【真正的漂移】:推出去和推回来【同一个方向】= 这个读数在自己往一边跑,不是响应。
                                    --  方向相反、只是大小对不齐 ⇒ 那是噪声,不是假信号,留着用(FO 靠的正是这种"有噪声但方向对"的
                                    --  距离信号,从 32 cm 一路读到 8.4 cm 并真的碰到了球)。
                                    --  之前写成"和超过出程的一半就清零",太严,把六个通道全判死,身体连距离都不会改了。
                                    if D_Out * D_Back > 0.0 then
                                       declare
                                          Col : Table.Vec3 := Table.Col (Effs (I), K);
                                       begin
                                          Col (2) := 0.0;
                                          Table.Set_Col (Effs (I), K, Col);
                                       end;
                                    end if;
                                 end;
                              end if;
                              P.Cu := Was (I).Cu; P.Cv := Was (I).Cv; P.Z := Was (I).Z;   --  推回起点了:点回到原处(比光流往返的累积误差可信)
                              Pts.Replace_Element (I, P);
                           end;
                        end loop;
                     end;
                     --  🔴 推到"大小也真的变过它自己的地板"为止,不只是"点动过"。
                     --  GR 实测:探针那一下太小,球在画面里的大小变化没过地板 ⇒ 那一列永远是零 ⇒
                     --  "看着多大"这条最稳的远近信号根本没被量到,而深度读数又不重复 ⇒
                     --  身体手里一个能用的"我在靠近吗"都没有,压不进去一整晚。
                     --  和"推到点真的动过"是同一条规矩,只是这次盯的是大小那一行;仍然被幅度上限兜着。
                     --  🔴 "点已经跑够一个跟踪窗了"不许当作可以收手 —— 那正是把这一条规矩废掉的那个口子:
                     --  位置在很小的一推下就动了,于是探到那儿就停,而"看着多大"根本没变过地板 ⇒ 那一列恒零。
                     --  II 实测:命令、实到都是满幅的一推,手也在动,球却一直不变大,38 步差距 0.84 纹丝不动 ——
                     --  身体压根不知道往哪边是"更近"。现在整幅都能搜回来,推大一点不怕跟丢,仍被幅度上限兜着。
                     exit when Trust (K) and then (Size_Seen or else Amp * 2.0 > Cap_Amp);
                     if Amp * 2.0 > Cap_Amp then
                        Put_Line ("[身]     通道" & Natural'Image (Chn) & ":到 " & Codec.Fmt (Amp, 4) & " 一个点也没动过地板(最多的跑了 " & Codec.Fmt (Ran_Max, 4) & " 画幅,地板 " & Codec.Fmt (Floor_Px, 4) & ")⇒ 这一段不用它");
                        exit;
                     end if;
                     Amp := Amp * 2.0;
                  end;
               end loop;
            end if;
         end;
      end loop;
   end Probe_Effects;

   function Amount_Factor (A : Unbounded_String) return Long_Float is
     (if A = "small" then 0.25 elsif A = "large" then 1.0 else 0.5);   --  探针上限的几分之几(比例,无量纲)


   --  握区的点展开成瓣点(两瓣时):每一瓣各自到位,目标 = 区目标 + 这一瓣相对区心的偏移(保持此刻的开合朝向);
   --  歪了就有一瓣不到位,倾斜不用规则自然被罚。自己的手上相机里握区是固定像素,不展开。
   procedure Expand_Lobes (C : Context; F : Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector) is
      Out_P : Point_Vectors.Vector;
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
   begin
      for P of Pts loop
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
         begin
            if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm and then Cam_Arm (C, Cam) /= Integer (P.Arm) and then Z.Valid and then Z.N_Lobes = 2 then
               for Lb in 0 .. 1 loop
                  declare
                     Q : Point := P;
                     Tr : constant Zone_Track := C.Zones (Track_Idx (C, P.Arm, Cam));
                     --  瓣相对区心的偏移:身体图给了此刻各瓣位置就用它(转过的手瓣也跟着转),否则用开机量的
                     Ou : constant Long_Float := (if Tr.Has_Lobes then (if Lb = 0 then Tr.Au else Tr.Bu) - Tr.Cu else (if Lb = 0 then Z.A.Cu else Z.B.Cu) - Z.Cu);
                     Ov : constant Long_Float := (if Tr.Has_Lobes then (if Lb = 0 then Tr.Av else Tr.Bv) - Tr.Cv else (if Lb = 0 then Z.A.Cv else Z.B.Cv) - Z.Cv);
                     Zd : Long_Float := P.Z;
                  begin
                     Q.Blob := Lb;
                     Q.Par_Tu := P.Tu; Q.Par_Tv := P.Tv;
                     Q.Cu := P.Cu + Ou; Q.Cv := P.Cv + Ov;
                     Q.Tu := P.Tu + Ou; Q.Tv := P.Tv + Ov;
                     if F.Cams (Cam).Has_Depth then
                        --  读深窗口 = 这一瓣自己的大小
                        Zd := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, Q.Cu, Q.Cv, Lobe_Win (Z, Cw, Ch));
                        if Picture.Is_Nan (Zd) then
                           Zd := P.Z;
                        end if;
                     end if;
                     Q.Z := Zd;
                     if Lb = 1 then
                        Q.Desc := Null_Unbounded_String;
                     end if;
                     Out_P.Append (Q);
                  end;
               end loop;
            else
               Out_P.Append (P);
            end if;
         end;
      end loop;
      Pts := Out_P;
   end Expand_Lobes;

   --  ── 一段 = 反复做五件事:①打算怎么走 ②走 ③看 ④学 ⑤判 ──
   --  每件事一个小过程;这一步发生了什么全记在 Note 里(字段名就是人话),五件事之间只靠它说话。
   procedure Run_Segment (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector;
                          Until_Kind : Monitor.Until_Kind; Step_Limit : Natural; Amount : Long_Float; Avoid : Item_Vectors.Vector;
                          Event : out Unbounded_String; Steps_Taken : out Natural; Blocked_Out : out Boolean; Beats : out Natural) is
      Arm : constant Natural := Pts (0).Arm;
      Beats0 : constant Natural := Plug.Steps (L);
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Own_Cam : constant Boolean := Cam_Arm (C, Cam) = Integer (Arm);
      Effs : Effect_Array (0 .. Natural (Pts.Length) - 1);
      Trusts : array (0 .. Natural (Pts.Length) - 1) of Table.Mask := [others => [others => True]];
      Reach : Table.Vec := Unit_Reach;   --  每通道核实过的步幅倍数(存表里,越用越强)
      Fl : Monitor.Floors;
      W : Monitor.Watch;
      Ring : Backup.Ring;
      Jaw : Floats;
      Last_Err : Long_Float := -1.0;     --  -1 = 还没算过(哨兵,无量纲)
      Last_Raw : Long_Float := -1.0;     --  上一步不随表变的差距
      Best_Raw : Long_Float := -1.0;     --  到目前为止最好的一次(判"有没有在靠近"和它比,不和上一步比 —— 噪声一晃就成退步)
      Blocked_Run : Natural := 0;        --  连着几步"零表更准":身体自己说这张地图不如"什么都不会发生"准
      Start_H : Long_Float := -1.0;      --  这一段开始时,我点名的那块比它周围鼓出多少(米):它离开台面,这个数就长
      Trust : Long_Float := 0.5;         --  这张表有多准,就走它算出来的多大比例(半开始;预测差一半就只走一半,准了再放开)
      Lost_Run : Natural := 0;           --  连着几步全部认不到

      --  这一步的账
      type Note_Rec is record
         Cmd, Got, Cap : Table.Vec := Table.Zero_Vec;   --  要走的 / 实际走的 / 各通道这一步的上限
         Active : Table.Mask := [others => False];
         Floor_Cmd : Long_Float := 0.0;                 --  最小探针幅度:比它一半还小的命令说明不了"顶住"
         Err_Now : Long_Float := 0.0;                   --  这一步之后还差几步(尺度会变,只用来说话)
         Raw_Now : Long_Float := 0.0;                   --  这一步之后不随表变的差距(判有没有在靠近就看它)
         Pic_Delta : Long_Float := 0.0;
         Big_Step : Boolean := False;                   --  大到光流跟不住 ⇒ 走完抖一下认自己
         Halted : Boolean := False;                     --  途中眼睛叫停
         Blocked : Boolean := False;                    --  顶住了(零表更准)
         Free : Boolean := False;                       --  我点名的那块不再挨着它原来站的那个面
         Not_Followed : Boolean := False;               --  整步没照做
         Touched : Boolean := False;                    --  我没在推的东西也动了 = 碰到
         Lost_All : Boolean := False;                   --  被跟的全都认不到
         Unsure : Boolean := False;                     --  两块一样像,不许自己挑
         Say_Stop : Unbounded_String;                   --  非空 = 这一段到此为止,内容就是给脑的话
      end record;
      Note : Note_Rec;
      Before : Buf;                      --  走之前那一拍的灰度(光流用)
      Was : Point_Vectors.Vector;        --  走之前各点在哪
      Was_Regs : Picture.Regions;        --  走之前世界里各块在哪

      --  走的途中每一拍看一眼:被跟的东西还找得到吗、离画面边够不够远(转一点点就该知道不对劲)
      function Watch_Things (Fr : Plug.Frame) return Boolean is
         Regs : Picture.Regions;
         Have : Boolean := False;
      begin
         for P of Pts loop
            if P.Kind = Thing_Pt then
               if not Have then
                  Regs := Cut_Things (C, Fr, Cam);
                  Have := True;
               end if;
               declare
                  Found : Boolean := False;
                  Tol : constant Long_Float := Long_Float'Max (P.Box_W, P.Box_H) * 0.75 + Track_Win;   --  和 Retrack 同一个认领半径(比例,无量纲)
               begin
                  for R of Regs loop
                     if R.Count * 3 >= P.Count and then R.Count <= P.Count * 3 and then Sqrt ((R.Cu - P.Cu) ** 2 + (R.Cv - P.Cv) ** 2) <= Tol then
                        Found := True;
                        if R.Cu < Track_Win or else R.Cu > 1.0 - Track_Win or else R.Cv < Track_Win or else R.Cv > 1.0 - Track_Win then
                           Note.Halted := True;
                           return True;
                        end if;
                     end if;
                  end loop;
                  --  🔴 切不出块的时候,这只眼睛【什么也没看见】,不是"东西不见了"。
                  --  关掉深度以后颜色切块常常一块都切不出来(脑指出来的东西本来就切不出来,
                  --  那正是"脑指"存在的理由)⇒ 老写法每一步都叫停,一步都走不完(HZ 实测:
                  --  12 推全部"途中眼睛叫停",手一动没动)。判不了就别插嘴。
                  if not Found and then not Regs.Is_Empty then
                     Note.Halted := True;
                     return True;
                  end if;
               end;
            end if;
         end loop;
         return False;
      end Watch_Things;

      --  落一张图,把这一拍切出来的块和被跟的点画上去(出岔子时给人看)
      procedure Dump_Picture (Tag : String) is
      begin
         if C.Dump_Dir = "" then
            return;
         end if;
         declare
            RGB : Buf := F.Cams (Cam).RGB;
            Regs : constant Picture.Regions := Cut_Things (C, F, Cam);
            N : Natural := 0;
         begin
            for R of Regs loop
               N := N + 1;
               Draw.Numbered_Box (RGB, Cw, Ch, R.X0, R.Y0, R.X1, R.Y1, N, Draw.Green, 2);
            end loop;
            for P of Pts loop
               declare
                  Hw : constant Long_Float := Long_Float'Max (P.Box_W, Track_Win) * 0.5;   --  框(没有就用一个跟踪窗;比例,无量纲)
                  Hh : constant Long_Float := Long_Float'Max (P.Box_H, Track_Win) * 0.5;
               begin
                  --  🔴 删掉"不许推出画面"那道闸之后,被跟的点会真的跑出画面(实测 v = -0.056),
                  --  两个角都要先夹回画面里再取整,否则画框这一步 Natural(负数) 直接把整个驱动崩掉
                  --  (GC 实测:act.adb:1182 range check failed,rc=1,一炮当场死)
                  Draw.Numbered_Box (RGB, Cw, Ch,
                                     Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), (P.Cu - Hw) * Long_Float (Cw)))),
                                     Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), (P.Cv - Hh) * Long_Float (Ch)))),
                                     Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), (P.Cu + Hw) * Long_Float (Cw)))),
                                     Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), (P.Cv + Hh) * Long_Float (Ch)))),
                                     0, Draw.Pink, 2);
               end;
            end loop;
            Codec.Write_BMP (To_String (C.Dump_Dir) & "/" & Tag & "_" & Codec.Pad6 (C.Round_N) & "_" & Codec.Pad6 (Steps_Taken) & ".bmp", RGB, Cw, Ch);
            Put_Line ("[身]     落图 " & Tag & "_" & Codec.Pad6 (C.Round_N) & "_" & Codec.Pad6 (Steps_Taken) & ".bmp(切出" & Natural'Image (N) & " 块)");
         end;
      end Dump_Picture;

      --  段前:每个点的响应表 —— 装回(必须是同一位姿、同样拿着东西、至少有一列信得过)或当场重量
      procedure Ready_Tables (Ok_Out : out Boolean) is
         Need : Boolean := False;
      begin
         Ok_Out := True;
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               Idx : constant Integer := Find_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob);
            begin
               --  🔴 有表就用,不再因为"位姿走远了"重探一遍。
               --  账:实测一"推"要花 13~21 拍,而真正干活的那一下只占一两拍 —— 差出来的全是【每一段重探六个通道】。
               --  官方上限 200 拍,FO 用了 1153 拍,超的 5.7 倍几乎全在这个倍数上。
               --  而每一步本来就是一次测量(命令了多少、画面里发生了什么),表是递推更新的,
               --  所以除了开机第一次,专门的探针阶段是纯浪费:走两步它自己就修回来了,而重探要花上百拍。
               --  位姿走远了不再当作"作废",只是头几步预测会差一点 —— 那正是"信表"这个数会自动压住的。
               if Idx >= 0 and then (for some K in 0 .. Chan.Per_Arm - 1 => C.Tables (Natural (Idx)).Trust (K))
                 and then C.Tables (Natural (Idx)).Has_Pose
               then
                  Effs (I) := C.Tables (Natural (Idx)).E;
                  Trusts (I) := C.Tables (Natural (Idx)).Trust;
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     Reach (K) := Long_Float'Max (Reach (K), C.Tables (Natural (Idx)).Reach (K));
                  end loop;
               else
                  Need := True;
               end if;
            end;
         end loop;
         if Need then
            declare
               Trust : Table.Mask;
               Ok : Boolean;
            begin
               Probe_Effects (L, C, F, Cam, Pts, Effs, Trust, Ok);
               if not Ok then
                  Ok_Out := False;
                  return;
               end if;
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  Trusts (I) := Trust;
                  Store_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob, Effs (I), Trust, Unit_Reach, F.EE (Arm), True);
               end loop;
            end;
         end if;
      end Ready_Tables;

      --  ①a 定目标:每个点的五样差距,各自除以"推一步最多能改多少",变成"还差几步"
      procedure Aim (Terms : out Table.Term_Vectors.Vector) is
      begin
         Terms.Clear;
         declare
            Big : Long_Float := 0.0;
         begin
            for P of Pts loop
               if P.Kind = Thing_Pt then
                  Big := Long_Float'Max (Big, Long_Float'Max (P.Box_W, P.Box_H));
               end if;
            end loop;
            C.Want_Size := Big;
         end;
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               P : constant Point := Pts (I);
               T : Table.Term;
            begin
               T.E := Effs (I);
               --  🔴 离得越远,"落在两指正中间"越不该急着满足:那是到跟前才成立的几何。
               --  权重 = 指尖有多近 ÷ 它有多远(量出来的两个深度之比):远的时候远近压过画面 ⇒ 先走过去;
               --  越走近画面越重要 ⇒ 最后才精确对准。不这么定,它会在 25 cm 外用转手腕把画面对齐,手一步没靠近(FI 实测)
               declare
                  Near : constant Long_Float :=
                    (if P.Wz > 0.0 and then P.Z > 0.0 and then not Picture.Is_Nan (P.Tz) and then P.Tz > 0.0
                     then Long_Float'Min (1.0, P.Tz / P.Z) else 1.0);
               begin
                  --  🔴🔴 两块东西一前一后时,【画面上重合 ≠ 真的在一起】。
                  --  投影的规矩是:同一个真实横移,离相机越近在画面上跑得越多(跑的距离 ∝ 1/远近)。
                  --  所以要对齐的不是 u,而是 u × 远近 —— 这样比出来的是真实的横向差,而镜头焦距在两边同样出现、
                  --  自动约掉,一个标定参数都不需要。
                  --  实测(FZ):头顶相机报"爪子离球只差 0.062 幅、几乎压上了",而切到手腕相机一看,
                  --  球根本不在视野里 —— 爪子在球上方 30 cm,画面上却正好叠住。只比 u 就是在比影子。
                  --  把目标【投影到我这一点自己的那个远近平面上】再比:同一个真实横移,离相机越近在画面上跑得越多,
                  --  所以远处的目标 u 要按 远近之比 从画面中心往外放大,才是"我要走到的那个 u"。
                  --  这样误差仍然是 u 的单位(表、预测、走多远的检查全部不变),但比的是真实位置而不是影子。
                  if P.Z > 0.0 and then P.Tz > 0.0 then
                     declare
                        R : constant Long_Float := P.Tz / P.Z;
                     begin
                        T.Err (0) := (0.5 + (P.Tu - 0.5) * R) - P.Cu;
                        T.Err (1) := (0.5 + (P.Tv - 0.5) * R) - P.Cv;
                     end;
                  else
                     T.Err (0) := P.Tu - P.Cu;
                     T.Err (1) := P.Tv - P.Cv;
                  end if;
                  T.W (0) := Near;
                  T.W (1) := Near;
               end;
               --  远近:画面位置和远近一起要,不许替它定"先对准再靠近"的顺序(那等于叫它先扭脖子)
               if P.Wz > 0.0 and then P.Z > 0.0 and then not Picture.Is_Nan (P.Tz) then
                  T.Err (2) := P.Tz - P.Z; T.W (2) := 1.0;
               end if;
               --  看着多大:离得越近越大,这是最稳的远近信号(画面上量的)
               if P.Wsize > 0.0 and then P.Size > 0.0 and then P.Tsize > 0.0 then
                  T.Err (3) := P.Tsize - P.Size; T.W (3) := 1.0;
               end if;
               --  朝向:差绕回 (-π, π];圆的东西这一行谁也改不动,归一时自动关掉
               if P.Wang > 0.0 then
                  T.Err (4) := Wrap (P.Tang - P.Ang); T.W (4) := P.Wang;
               end if;
               --  🔴 五样单位不同,混着求和就是错的判据。不换算成米,改成【只比较】:
               --  每一样除以"推一步最多能把它改多少",都变成"还差几步"(无量纲),本来就可比。
               --  扭手腕改不了远近 ⇒ 它在那一栏拿不到分,偷不了便宜。
               for R in 0 .. Table.Rows - 1 loop
                  declare
                     Per_Step : Long_Float := 0.0;
                  begin
                     for K in 0 .. Chan.Per_Arm - 1 loop
                        if C.Map.Seen (Arm * Chan.Per_Arm + K) and then Trusts (I) (K) then
                           Per_Step := Long_Float'Max (Per_Step, abs (T.E.B (K, R)) * Long_Float'Max (1.0e-9, C.Map.Amp (Arm * Chan.Per_Arm + K)));
                        end if;
                     end loop;
                     if Per_Step > 0.0 then
                        T.Err (R) := T.Err (R) / Per_Step;
                        for K in 0 .. Chan.Per_Arm - 1 loop
                           T.E.B (K, R) := T.E.B (K, R) / Per_Step;
                        end loop;
                        --  🔴 一步就是一推:任何一行这一步最多也只能要求一推。不压这一条,
                        --  一行算出"还差一千步"就会把整个目标吃掉,手开始乱转(IF 实测:大小 4 → 1342)。
                        --  压完之后各行都只剩"这一步管不管你",方向由解算把它们放在一起挑。
                        T.Err (R) := Long_Float'Max (-1.0, Long_Float'Min (1.0, T.Err (R)));
                        --  🔴 差不到一推的,这一步就别管它 —— 身体本来就分辨不到比一推更细。
                        --  不加这一条,已经瞄准到"零点一推"的那两行会死死拽住手不让它往前:
                        --  往前必然要动画面位置,而那两行不肯让 ⇒ 解算给出的命令只有一推的二十分之一,
                        --  正负号每步翻,手在原地抖了 60 步一点没靠近(IH 实测:差距 0.856 纹丝不动)。
                        if abs T.Err (R) < 1.0 then
                           T.Err (R) := 0.0;
                        end if;
                     else
                        T.W (R) := 0.0;   --  这一行一个通道都改不动 ⇒ 这一步不管它
                     end if;
                  end;
               end loop;
               declare
                  Q : Point := P;
               begin
                  Q.Steps_Err := 0.0;
                  for R in 0 .. Table.Rows - 1 loop
                     Q.Steps_Err := Q.Steps_Err + (T.Err (R) * T.W (R)) ** 2;
                  end loop;
                  Q.Steps_Err := Sqrt (Q.Steps_Err);
                  Q.Err_U := T.Err (0) * T.W (0); Q.Err_V := T.Err (1) * T.W (1); Q.Err_Z := T.Err (2) * T.W (2);
                  Q.Err_S := T.Err (3) * T.W (3); Q.Err_A := T.Err (4) * T.W (4);
                  Q.Raw_Err := Sqrt ((P.Tu - P.Cu) ** 2 + (P.Tv - P.Cv) ** 2
                                     + (if P.Wz > 0.0 and then P.Z > 0.0 then ((P.Tz - P.Z) / P.Z) ** 2 else 0.0)
                                     + (if P.Wsize > 0.0 and then P.Tsize > 0.0 then ((P.Tsize - P.Size) / P.Tsize) ** 2 else 0.0)
                                     + (if P.Wang > 0.0 then (P.Wang * Wrap (P.Tang - P.Ang) / Ada.Numerics.Pi) ** 2 else 0.0));
                  Pts.Replace_Element (I, Q);
               end;
               Terms.Append (T);
            end;
         end loop;
         if Last_Err < 0.0 then
            Last_Err := 0.0; Last_Raw := 0.0;
            for P of Pts loop
               Last_Err := Last_Err + P.Steps_Err;
               Last_Raw := Last_Raw + P.Raw_Err;
            end loop;
            Best_Raw := Last_Raw;
         end if;
      end Aim;

      --  ①b 定额度:各通道这一步最多走多少 —— 两条取小:①眼睛跟得住的那么多(表说走多少画面跑满一个跟踪窗)
      --  ②自己那一档 × 核实过的倍数。只用①,在画面里几乎不动的通道会拿到无限额度(甩腕 2 rad)
      procedure Budget (Terms : Table.Term_Vectors.Vector; Solved : out Boolean) is
         Damp : Table.Vec := Table.Zero_Vec;
      begin
         for K in 0 .. Chan.Per_Arm - 1 loop
            declare
               Ch_No : constant Natural := Arm * Chan.Per_Arm + K;
               Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Ch_No));
               All_Trust : Boolean := True;
               Px : Long_Float := 0.0;
            begin
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  if not Trusts (I) (K) then
                     All_Trust := False;
                  end if;
                  --  🔴 "搅动画面多少"必须把【远近】也算进去,否则一个几乎不改变画面位置、却让手大幅前后
                  --  移动的通道在账本上等于免费 ⇒ 拿到近乎无限的额度(FW/FX 实测:步幅长到 31 倍、
                  --  单步命令 0.19,手冲过头再拉回来)。远近除以此刻的距离变成【相对变化】,
                  --  和"跑了几分之一画幅"同一个量纲(比例,无量纲),可以直接一起算。
                  --  这就是"转腕在账本上便宜十六倍"那个老坑的一般形式:便宜的方向会被买爆。
                  declare
                     Zr : constant Long_Float := (if Pts (I).Z > 0.0 then Pts (I).Z else 1.0);
                  begin
                     Px := Long_Float'Max (Px, Sqrt (Effs (I).B (K, 0) ** 2 + Effs (I).B (K, 1) ** 2
                                                     + (Effs (I).B (K, 2) / Zr) ** 2));
                  end;
               end loop;
               if C.Map.Seen (Ch_No) and then All_Trust then
                  Note.Active (K) := True;
                  --  🔴 只有【后果全量清楚了】的方向才准迈大步。某一格没量出来(探针时变化没过地板)会被留成 0,
                  --  而 0 的意思是"没影响",解算就当它免费 —— 转腕对"远近/看着多大"正是这样,于是它拿转腕去修画面位置,
                  --  一转就把距离搞坏(FD 实测:0.6 rad 的腕,球越走越远)。没量清楚的方向只给探针那一档。
                  declare
                     Known_All : Boolean := Px * Am > Fl.Track * 2.0;
                  begin
                     for I in 0 .. Natural (Pts.Length) - 1 loop
                        for R in 0 .. Table.Rows - 1 loop
                           if Terms (I).W (R) > 0.0 and then abs (Effs (I).B (K, R)) <= 0.0 then
                              Known_All := False;
                           end if;
                        end loop;
                     end loop;
                     --  上限 = 自己那一档 × 核实过的倍数,再压在"眼睛跟得住"这个天花板下。
                     --  倍数只有靠"表说会挪多少 vs 实际挪了多少"对上才涨(见 Learn),没证明过就不许迈大步
                     --  🔴 下限 = 自己那一档:开机量到"命令小于这一档,点在画面里根本不动"。
                     --  比它还小的一步等于没走,而没走会被判成"身体没照做"⇒ 步幅再减半 ⇒ 更走不动。
                     --  实测(FP):某通道量到要 0.0256 才看得见动,而"小步"给出的额度只有 0.008,
                     --  连着三步实到全 0,身体报"要么有东西拽着我,要么这条胳膊到头了" —— 其实只是自己没迈够。
                     --  ⇒ "小/中/大"只能把一步放大,不能把它缩到动不了。
                     --  🔴 后果没量清楚的方向,只给【探针那一档】,不许再乘"核实过的倍数"。
                     --  以前两边都乘 Reach,而 Reach 是"走成一步就 ×2"且没有真正的上限 ⇒ FW 实测长到 31 倍、
                     --  单步命令 0.198(自己那一档的 8 倍),手开始冲过头再拉回来:差距 405→93 之后反弹到 125,
                     --  命令正负号每步翻。注释本来就写着"没量清楚的方向只给探针那一档",代码没照做。
                     --  🔴 额度只剩两样:自己量到的那一档 × 脑说的大小(small/medium/large)。
                     --  身体自己长出来的倍数(走成了就 ×2)、"眼睛跟得住"的天花板、"没量清楚就少给"
                     --  —— 全是身体在替脑决定走多远,按 owner 2026-09-09 的命令删掉。
                     --  🔴 油门装回来,但只许【往下踩】:Reach 只取不超过 1 的那一半 ——
                     --  地图说了不算就把这一步咬小一口,准了也不许超过脑要的那一档。
                     --  老版是"只会减速、没有底线"⇒ 一路减到零卡死(那才是 bug);
                     --  新版有底线(Am,自己量到的推得动的最小量),所以减得下去、踩不死。
                     --  🔴 下限改成【身体自己的噪声的两倍】,不是"探针那一档"。
                     --  实测:FO 每一步的命令是 0.006 = 探针那一档的四分之一,照样一步推进 8 厘米、44 推抓到球;
                     --  我把下限设成整整一档 ⇒ 今晚每一步是 FO 的 4 倍(0.026),表当场不准、球被甩出视野。
                     --  真正要挡的是"命令小到身体根本不动"(本体噪声那一档),不是"比探针小"。
                     --  下限 = 这个通道自己量出来的死区(见 act.ads),再小也不低于本体噪声的两倍
                     Note.Cap (K) := Long_Float'Max
                       (Long_Float'Max (2.0 * C.Map.EE_Noise,
                                        (if Ch_No < Natural (C.Dead.Length) then C.Dead.Element (Ch_No) else 0.0)),
                        Am * Cap_Mult * Amount * Long_Float'Min (1.0, Reach (K)))
                       + 0.0 * (if Known_All then 1.0 else 0.0);
                  end;
                  Note.Floor_Cmd := (if Note.Floor_Cmd <= 0.0 then Am else Long_Float'Min (Note.Floor_Cmd, Am));
               end if;
               --  🔴 标价改成"这个动作把画面搅动多少":一单位命令让被跟的点在画面里跑几个跟踪窗,就付几分钱(无量纲)。
               --  以前按"自己那一档"计价,而转腕那一档(0.0256)比平移那一档(0.0064)大四倍 ⇒ 转腕在账本上便宜十六倍,
               --  于是它一直买转腕,而转腕不会让手靠近(FJ 实测:横挪 4 cm,球反而从 0.333 m 退到 0.360 m)
               Damp (K) := (Px / Track_Win) ** 2;
            end;
         end loop;
         Table.Solve (Terms, Chan.Per_Arm, Note.Cap, Note.Active, Damp, Note.Cmd, Solved);
      end Budget;

      --  ①c 修步子:缩到眼睛跟得住,且不把被跟的东西推出视野、不让我身上任何一块压到"不许碰"的框
      procedure Trim is
         Scale : Long_Float := 1.0;
      begin
         --  🔴🔴 这一条装回来了(owner 2026-09-09 认可):它【不是否决权,是眼睛的快门速度】。
         --  身体认东西靠"这一帧和上一帧比";一步挪太大,那块东西变大了、被手指挡掉一块、位置跳了,
         --  对号就跳到旁边另一个东西上,而且从那一刻起它朝着【错的东西】走,自己不知道。
         --  证据:FN–FO 连着五炮都真的碰到了球,靠的就是当时每一步都极小(被一堆 bug 砍的);
         --  今晚把砍步子的全删掉之后,步子一大,一段之内必跟丢(GG:十步之后身体说球在右下角)。
         --  ⚠️ 和"一步不许小到动不了"那条【一起】才对:能动得起来,又跟得住。
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Cmd);
               D : constant Long_Float := Sqrt (Pr (0) ** 2 + Pr (1) ** 2);
            begin
               if D > Track_Win then
                  Scale := Long_Float'Min (Scale, Track_Win / D);
               end if;
            end;
         end loop;
         if Scale < 1.0e-3 then
            Note.Big_Step := True;   --  缩到千分之一还不够(比例,无量纲)= 表已经不可信
         end if;
         --  不许把被跟的东西推出视野;不许让【我身上任何一块】压到"不许碰"的框里(只查一个点等于没查)
         --  出了留边多远(比例,无量纲):0 = 还在里面。
         --  🔴 留边 = 【这一块自己在画面里的半个身子】,不是固定的一成画幅。
         --  含义是"别把它推到一半以上都在画面外",而多大算一半是量出来的:小手指能走到画面 97% 处还认得住,
         --  大块就得早点停。固定一成留边在最后一刻恰恰是致命的:FT 实测手指走到 0.90 就被钉死,
         --  一连三段推 0 下,而球还差 30 cm 没下去。
         declare
            --  留边 = 这一块自己的半个身子 + 半个跟踪窗:半个身子保证"还认得出是它",
            --  半个跟踪窗是"一步最多跑这么远"的余量(两个都是量出来的;比例,无量纲)。
            --  只留半个身子太薄:FU 实测手指被允许一路推到画面外(u=1.00)然后跟丢。
            function Margin (P : Point) return Long_Float is
              (0.5 * Long_Float'Max (Long_Float'Max (P.Box_W, P.Box_H), Track_Win));
            function Out_Of (U, V, M : Long_Float) return Long_Float is
              (Long_Float'Max
                 (0.0,
                  Long_Float'Max (Long_Float'Max (M - U, U - (1.0 - M)),
                                  Long_Float'Max (M - V, V - (1.0 - M)))));
         begin
         for Round in 1 .. 4 loop
            declare
               Hit : Boolean := False;
               function In_Avoid (X0, Y0, X1, Y1 : Long_Float) return Boolean is
               begin
                  for Av of Avoid loop
                     if Av.Located and then X0 <= Long_Float (Av.X1) and then X1 >= Long_Float (Av.X0)
                       and then Y0 <= Long_Float (Av.Y1) and then Y1 >= Long_Float (Av.Y0)
                     then
                        return True;
                     end if;
                  end loop;
                  return False;
               end In_Avoid;
            begin
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  declare
                     Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Cmd);
                     Nu : constant Long_Float := Pts (I).Cu + Pr (0) * Scale;
                     Nv : constant Long_Float := Pts (I).Cv + Pr (1) * Scale;
                  begin
                     --  🔴 只拦"越走越出画面"的步子。以前是"新位置在安全带外就拦",而【本来就在带外】的点
                     --  (手指本来就贴着画面右边)会让任何一步都被拦掉 —— FR 实测:这一段推了 0 下,
                     --  身体报"每一步都会把我盯着的东西推出视野",而它其实是想往回走。
                     --  出界量:0 = 在带内,越大出得越远;新的比现在还远才拦。
                     declare
                        M : constant Long_Float := Margin (Pts (I));
                     begin
                        if Out_Of (Nu, Nv, M) > Out_Of (Pts (I).Cu, Pts (I).Cv, M) and then Out_Of (Nu, Nv, M) > 0.0 then
                           Hit := True;
                        end if;
                     end;
                     if In_Avoid (Nu * Long_Float (Cw), Nv * Long_Float (Ch), Nu * Long_Float (Cw), Nv * Long_Float (Ch)) then
                        Hit := True;
                     end if;
                  end;
               end loop;
               if not Own_Cam and then Track_Idx (C, Arm, Cam) < Natural (C.Zones.Length) then
                  declare
                     Tr : constant Zone_Track := C.Zones (Track_Idx (C, Arm, Cam));
                  begin
                     for K in 0 .. Chan.Per_Arm loop
                        if Tr.Pieces (K).Valid then
                           declare
                              Idx : constant Integer := Find_Effect (C, Arm, Cam, Piece_Pt, K, -1);
                              Sh : constant Table.Vec3 := (if Idx >= 0 then Table.Predict (C.Tables (Natural (Idx)).E, Note.Cmd) else Table.Zero3);
                              Du : constant Long_Float := Sh (0) * Scale * Long_Float (Cw);
                              Dv : constant Long_Float := Sh (1) * Scale * Long_Float (Ch);
                           begin
                              if In_Avoid (Long_Float (Tr.Pieces (K).X0) + Du, Long_Float (Tr.Pieces (K).Y0) + Dv,
                                           Long_Float (Tr.Pieces (K).X1) + Du, Long_Float (Tr.Pieces (K).Y1) + Dv)
                              then
                                 Hit := True;
                              end if;
                           end;
                        end if;
                     end loop;
                  end;
               end if;
               --  🔴 只剩脑自己点名的"别碰这些"(那是听话);缩到最小也照走,不许拒绝
               exit when not Hit or else Round = 4;
               Scale := Scale * 0.5;
            end;
         end loop;
         end;
         for K in 0 .. Chan.Per_Arm - 1 loop
            --  信表打折也装回来:表越不准走得越保守。后面那段"掉到动不了就整体抬回去"是它的底线,
            --  所以打折只会变慢,不会变成零(老版没有那段底线,才会卡死)。
            Note.Cmd (K) := Note.Cmd (K) * Scale * Trust;
         end loop;
         --  同一条规矩的第二半:缩完之后若整步又掉到"动不了"以下,按比例整体抬回去 ——
         --  方向听解算的,大小至少迈到自己量到的那一档,再压回各自的上限之下(比例,无量纲)。
         declare
            Most : Long_Float := 0.0;                  --  最大的那个通道走了自己那一档的几成
            Room : Long_Float := Long_Float'Last;      --  还能整体放大几倍才顶到上限
         begin
            for K in 0 .. Chan.Per_Arm - 1 loop
               if Note.Active (K) then
                  declare
                     Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Arm * Chan.Per_Arm + K));
                  begin
                     Most := Long_Float'Max (Most, abs Note.Cmd (K) / Am);
                     if abs Note.Cmd (K) > 1.0e-12 and then Note.Cap (K) > 0.0 then
                        Room := Long_Float'Min (Room, Note.Cap (K) / abs Note.Cmd (K));
                     end if;
                  end;
               end if;
            end loop;
            if Most > 0.0 and then Most < 1.0 and then Room > 1.0 then
               declare
                  G : constant Long_Float := Long_Float'Min (1.0 / Most, Room);
               begin
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     Note.Cmd (K) := Note.Cmd (K) * G;
                  end loop;
               end;
            end if;
         end;
         null;   --  步子小到噪声里也照发,不许因此停下
      end Trim;

      --  ① 打算怎么走 = 定目标 → 定额度 → 修步子
      procedure Plan is
         Terms : Table.Term_Vectors.Vector;
         Solved : Boolean;
      begin
         Note := (others => <>);
         Aim (Terms);
         Budget (Terms, Solved);
         if not Solved then
            Put_Line ("[身]     这一步没解出该推哪些通道 ⇒ 这一步不动,接着走(不许因此停)");
         end if;
         Trim;
      end Plan;

      --  ② 走:记下走之前的样子,发命令,途中盯着
      procedure Walk (Ok_Out : out Boolean) is
      begin
         Before := F.Cams (Cam).Gray;
         Was := Pts;
         Was_Regs := (if not Own_Cam then Cut_Things (C, F, Cam) else Picture.Region_Vectors.Empty_Vector);
         Step_Arm (L, C, F, Arm, Note.Cmd, Jaw, Note.Got, Ok_Out, C.Fast, Watch_Things'Unrestricted_Access);
         Beats := Plug.Steps (L) - Beats0;
         if Note.Halted then
            Put_Line ("[身]     途中眼睛叫停:被跟的东西快出画面或看不见了,这一步没走完");
         end if;
         if not Ok_Out then
            return;
         end if;
         Steps_Taken := Steps_Taken + 1;
         declare
            Sv : Backup.Step_Vec := [others => 0.0];
         begin
            for K in 0 .. Chan.Per_Arm - 1 loop
               Sv (K) := Note.Got (K);
               --  🔴 学死区:命令发了而身体没动 ⇒ 这一档不够,抬上去;真动了 ⇒ 说明这一档够,压下来。
               --  抬 1.5 倍、压到刚好走成的那一档(倍数,无量纲)。
               declare
                  Cn : constant Natural := Arm * Chan.Per_Arm + K;
               begin
                  if Cn < Natural (C.Dead.Length) and then abs Note.Cmd (K) > C.Map.EE_Noise then
                     if abs Note.Got (K) <= C.Map.EE_Noise then
                        C.Dead.Replace_Element (Cn, Long_Float'Max (C.Dead.Element (Cn), abs Note.Cmd (K) * 1.5));
                     else
                        C.Dead.Replace_Element (Cn, Long_Float'Min (C.Dead.Element (Cn), abs Note.Cmd (K)));
                     end if;
                  end if;
               end;
            end loop;
            Backup.Remember (Ring, Sv);
         end;
         Note.Pic_Delta := Long_Float (Picture.Max_Diff (Before, F.Cams (Cam).Gray));
      end Walk;

      --  ③ 看:每个点现在在哪。我的零件(别人的相机里)先按位姿"感觉",熟地让眼睛核对,生地或大步就抖一下去看;
      --  世界里的块每步重切就近对上。全都认不到才叫看不见。
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
               if P.Kind = Piece_Pt and then not Own_Cam then
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
                           if Gp.Z > 0.0 then
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
                        if W0.Z > 0.0 then
                           P.Z := W0.Z + Pr (2);
                        end if;
                     end if;
                     if Note.Big_Step or else not Familiar then
                        P.Lost := True;
                        Need_Refind := True;
                     else
                        declare
                           Q : Point := W0;
                           Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
                        begin
                           Retrack (C, F, Cam, Before, Q, W0.Cu + Pr (0), W0.Cv + Pr (1), True, (if W0.Z > 0.0 then W0.Z + Pr (2) else -1.0));
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
                  Retrack (C, F, Cam, Before, P, W0.Cu + Pr (0), W0.Cv + Pr (1), True, (if W0.Z > 0.0 then W0.Z + Pr (2) else -1.0));
               end if;
               Pts.Replace_Element (I, P);
            end;
         end loop;
         if Need_Refind then
            Refind_Pieces (L, C, F, Cam, Pts);
            Beats := Plug.Steps (L) - Beats0;
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

      --  ④ 学:拿这一步的实际结果修表;判"整步没照做";按通道各自放宽/收紧步幅;看有没有碰到别的东西
      procedure Learn is
         All_Verified : Boolean := True;
         Any_Wrong : Boolean := False;
      begin
         Note.Err_Now := 0.0;
         Note.Raw_Now := 0.0;
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               P : constant Point := Pts (I);
               W0 : constant Point := Was (I);
               Dy : Table.Vec3;
               E : Table.Effect := Effs (I);
            begin
               if P.Lost then
                  All_Verified := False;
                  Any_Wrong := True;
               else
                  --  修表只用眼睛量到的(光流核对值或抖认到的),按图猜的位置不喂回表
                  Dy (0) := (if P.Has_Meas then P.Meas_U else P.Cu) - W0.Cu;
                  Dy (1) := (if P.Has_Meas then P.Meas_V else P.Cv) - W0.Cv;
                  Dy (2) := (if P.Has_Meas then (if P.Meas_Z > 0.0 and then W0.Z > 0.0 then P.Meas_Z - W0.Z else 0.0)
                             elsif P.Z > 0.0 and then W0.Z > 0.0 then P.Z - W0.Z else 0.0);
                  --  同样的地板:没过就当没变(不然把量化噪声学进表里,符号可能是反的)
                  Dy (3) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (P.Size - W0.Size) > Size_Floor (Cw)
                             then P.Size - W0.Size else 0.0);
                  Dy (4) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (Wrap (P.Ang - W0.Ang)) > Ang_Floor (W0, Cw, Ch)
                             then Wrap (P.Ang - W0.Ang) else 0.0);
                  Table.Update (E, Note.Got, Dy, Fl.Track * 2.0, Long_Float'Max (C.Map.EE_Noise, 0.5 * Note.Floor_Cmd));
                  if Table.Blocked (E) then
                     Note.Blocked := True;
                  end if;
                  if not (E.Null_Res > Fl.Track * 2.0 and then E.Free_Res < E.Null_Res) then
                     All_Verified := False;
                  end if;
                  if E.Free_Res > Fl.Track * 2.0 and then E.Free_Res >= E.Null_Res then
                     Any_Wrong := True;
                  end if;
               end if;
               Effs (I) := E;
               Note.Err_Now := Note.Err_Now + P.Steps_Err;
               Note.Raw_Now := Note.Raw_Now + P.Raw_Err;
            end;
         end loop;
         --  整步没照做:各通道按自己的探针幅度归一后,实到与命令差过一半(逐个通道判会被同量级的小出入触发)
         if not Note.Halted then
            declare
               Dn, An : Long_Float := 0.0;
            begin
               for K in 0 .. Chan.Per_Arm - 1 loop
                  declare
                     Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Arm * Chan.Per_Arm + K));
                  begin
                     Dn := Dn + ((Note.Got (K) - Note.Cmd (K)) / Am) ** 2;
                     An := An + (Note.Cmd (K) / Am) ** 2;
                  end;
               end loop;
               Dn := Sqrt (Dn); An := Sqrt (An);
               if An > 1.0 and then Dn > 0.5 * An then
                  Any_Wrong := True; All_Verified := False; Note.Not_Followed := True;
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     if Note.Active (K) then
                        Reach (K) := Long_Float'Max (1.0, Reach (K) * 0.5);
                     end if;
                  end loop;
                  Put_Line ("[身]     整步没照做:要走的和实际走的差了 " & Codec.Fmt (Dn / Long_Float'Max (1.0e-9, An) * 100.0, 0) & "% ⇒ 步幅缩回上一档");
               end if;
            end;
         end if;
         --  🔴 步幅只认一件事:这一步【表说会挪多少】和【实际挪了多少】对不对得上。
         --  对得上 ⇒ 这几个用到的通道可以把步子放大一倍;差过一半 ⇒ 立刻缩回去。
         --  不管是平移还是转腕,都得先证明自己说话算数才有资格迈大步(FD/FE:转腕说了不算,一转球就更远)
         declare
            Pred_Ok : Boolean := True;
            Any_Meas : Boolean := False;
         begin
            for I in 0 .. Natural (Pts.Length) - 1 loop
               if not Pts (I).Lost then
                  declare
                     W0 : constant Point := Was (I);
                     Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Got);
                     Act_U : constant Long_Float := (if Pts (I).Has_Meas then Pts (I).Meas_U else Pts (I).Cu) - W0.Cu;
                     Act_V : constant Long_Float := (if Pts (I).Has_Meas then Pts (I).Meas_V else Pts (I).Cv) - W0.Cv;
                     Pred : constant Long_Float := Sqrt (Pr (0) ** 2 + Pr (1) ** 2);
                     Act : constant Long_Float := Sqrt (Act_U ** 2 + Act_V ** 2);
                  begin
                     if Pred > Fl.Track * 2.0 or else Act > Fl.Track * 2.0 then
                        Any_Meas := True;
                        --  差过预测的一半就算说了不算;再给一个和跟踪精度挂钩的绝对宽容(四分之一个跟踪窗),
                        --  否则步子越小相对误差越大,永远判"说了不算",步子就永远放不大(FF 实测每步都判不准)
                        if abs (Act - Pred) > 0.5 * Pred + Track_Win * 0.25 then
                           Pred_Ok := False;
                        end if;
                     end if;
                  end;
               end if;
            end loop;
            for K in 0 .. Chan.Per_Arm - 1 loop
               if Note.Active (K) and then abs Note.Cmd (K) > Long_Float'Max (Note.Floor_Cmd, C.Map.EE_Noise) then
                  --  🔴 "零表更准"= 这张表已经不如"什么都不会发生"准了 ⇒ 立刻降档。
                  --  FW 实测:最后三步都印了这句,而步幅一直是 31 倍没动过 —— 印出来了却没生效。
                  if Note.Blocked then
                     Reach (K) := Long_Float'Max (1.0, Reach (K) * 0.5);
                  elsif Any_Meas and then Pred_Ok and then (not Note.Halted) and then not Note.Not_Followed then
                     Reach (K) := Long_Float'Min (Reach (K) * 2.0, Track_Win / Long_Float'Max (1.0e-9, C.Map.Amp (Arm * Chan.Per_Arm + K)));
                  elsif Any_Meas and then not Pred_Ok then
                     Reach (K) := Long_Float'Max (1.0, Reach (K) * 0.5);
                  end if;
               end if;
            end loop;
            if Any_Meas and then not Pred_Ok then
               Put_Line ("[身]     表说了不算:预测挪的和实际挪的差过一半 ⇒ 用到的通道步子缩回去");
            end if;
            --  这张表这一步准到什么程度 ⇒ 下一步走它算出来的多大比例(准 = 走满,差一半 = 走一半)
            if Any_Meas then
               declare
                  Worst_Rel : Long_Float := 0.0;
               begin
                  for I in 0 .. Natural (Pts.Length) - 1 loop
                     if not Pts (I).Lost then
                        declare
                           W0 : constant Point := Was (I);
                           Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Got);
                           Au : constant Long_Float := (if Pts (I).Has_Meas then Pts (I).Meas_U else Pts (I).Cu) - W0.Cu;
                           Av : constant Long_Float := (if Pts (I).Has_Meas then Pts (I).Meas_V else Pts (I).Cv) - W0.Cv;
                           Pd : constant Long_Float := Sqrt (Pr (0) ** 2 + Pr (1) ** 2);
                           Ac : constant Long_Float := Sqrt (Au ** 2 + Av ** 2);
                        begin
                           Worst_Rel := Long_Float'Max (Worst_Rel, abs (Ac - Pd) / Long_Float'Max (Pd, Track_Win * 0.25));
                        end;
                     end if;
                  end loop;
                  Trust := 0.5 * Trust + 0.5 / (1.0 + Worst_Rel);
               end;
            end if;
         end;
         for I in 0 .. Natural (Pts.Length) - 1 loop
            Store_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob, Effs (I), Trusts (I), Reach);
         end loop;
         --  碰到 = 我没在推的东西自己动了(跟着这只手动的相机里满画面都在动,分不出来 ⇒ 不下结论)
         if not Own_Cam and then not Was_Regs.Is_Empty then
            declare
               Now_Regs : constant Picture.Regions := Cut_Things (C, F, Cam);
            begin
               for R of Now_Regs loop
                  declare
                     Mine : Boolean := False;
                     Found_Prev : Boolean := False;
                     Best : Long_Float := 0.0;
                  begin
                     for P of Pts loop
                        if Sqrt ((R.Cu - P.Cu) ** 2 + (R.Cv - P.Cv) ** 2) <= Long_Float'Max (P.Box_W, P.Box_H) then
                           Mine := True;
                        end if;
                     end loop;
                     if not Mine then
                        for Q of Was_Regs loop
                           if Q.Count * 3 >= R.Count and then R.Count * 3 >= Q.Count then
                              declare
                                 D : constant Long_Float := Sqrt ((R.Cu - Q.Cu) ** 2 + (R.Cv - Q.Cv) ** 2);
                              begin
                                 if not Found_Prev or else D < Best then
                                    Best := D; Found_Prev := True;
                                 end if;
                              end;
                           end if;
                        end loop;
                        if Found_Prev and then Best > Fl.Track * 2.0 then
                           Note.Touched := True;
                        end if;
                     end if;
                  end;
               end loop;
            end;
         end if;
         if Note.Touched then
            Put_Line ("[身]     我没在推的东西也动了 ⇒ 碰到它了");
         end if;
      end Learn;

      --  ⑤ 判:这一步之后接着走,还是到了 / 出事了 / 拿不准
      procedure Judge is
      begin
         --  进度只看不随表变的那把尺(Raw):"还差几步"的刻度每步都在变,用它判进度会把靠近判成退步(ES 实测两步就报停滞)
         --  和"到目前为止最好的一次"比:和上一步比的话,一次噪声就被当成退步
         Monitor.Step (W, Monitor.Floor (Long_Float'Max (0.0, Note.Pic_Delta)), Monitor.Bounded (Best_Raw), Monitor.Bounded (Note.Raw_Now),
                       Monitor.Floor (Long_Float'Max (0.0, Table.Norm (Note.Got, Chan.Per_Arm))), Fl);
         Put_Line ("[身]     步" & Natural'Image (Steps_Taken) & (if Note.Big_Step then "(大步)" else "") &
                   ":差距 " & Codec.Fmt (Last_Raw, 3) & " → " & Codec.Fmt (Note.Raw_Now, 3) & " · 还差 " & Codec.Fmt (Note.Err_Now, 1) & " 步(左右 " & Codec.Fmt (Pts (0).Err_U, 1) &
                   " 上下 " & Codec.Fmt (Pts (0).Err_V, 1) & " 远近 " & Codec.Fmt (Pts (0).Err_Z, 1) &
                   " 大小 " & Codec.Fmt (Pts (0).Err_S, 1) & " 朝向 " & Codec.Fmt (Pts (0).Err_A, 1) & ")· 拍 " & Codec.Img (Beats) &
                   " · 信表 " & Codec.Fmt (Trust, 2) & " · 步幅 ×[" & Codec.Fmt (Reach (0), 0) & " " & Codec.Fmt (Reach (1), 0) & " " & Codec.Fmt (Reach (2), 0) & " " &
                   Codec.Fmt (Reach (3), 0) & " " & Codec.Fmt (Reach (4), 0) & " " & Codec.Fmt (Reach (5), 0) &
                   "] · 命令 [" & Codec.Fmt (Note.Cmd (0), 3) & " " & Codec.Fmt (Note.Cmd (1), 3) & " " & Codec.Fmt (Note.Cmd (2), 3) & " " &
                   Codec.Fmt (Note.Cmd (3), 3) & " " & Codec.Fmt (Note.Cmd (4), 3) & " " & Codec.Fmt (Note.Cmd (5), 3) &
                   "] · 实到 [" & Codec.Fmt (Note.Got (0), 4) & " " & Codec.Fmt (Note.Got (1), 4) & " " & Codec.Fmt (Note.Got (2), 4) & " " &
                   Codec.Fmt (Note.Got (3), 3) & " " & Codec.Fmt (Note.Got (4), 3) & " " & Codec.Fmt (Note.Got (5), 3) &
                   "] · 点 (" & Codec.Fmt (Pts (0).Cu, 3) & "," & Codec.Fmt (Pts (0).Cv, 3) & ") 深 " & Codec.Fmt (Pts (0).Z, 3) &
                   (if Note.Blocked then " · 零表更准(顶住?)" else ""));
         Last_Err := Note.Err_Now;
         Last_Raw := Note.Raw_Now;
         if Best_Raw < 0.0 or else Note.Raw_Now < Best_Raw then
            Best_Raw := Note.Raw_Now;
         end if;
         if Codec.Env ("BL_STEPSHOT") /= "" then
            Dump_Picture ("step");   --  逐步落图:看被跟的那块在靠近时到底怎么变(BL_STEPSHOT 打开才存)
         end if;
         --  🔴 平时不重量表(每段重量 = 一推 13~21 拍的老账);但身体一旦【连着三步说"我的地图不如零假设准"】,
         --  就当场重量一遍 —— 拿着一张被证明错的表一路开,是今晚"手在动、球的距离一点不变"的直接原因。
         --  这是"只在被证明错的时候才重量",既不回到每段重量,也不拿假表开车。
         if Note.Blocked then
            Blocked_Run := Blocked_Run + 1;
         else
            Blocked_Run := 0;
         end if;
         if Blocked_Run >= 3 then
            Blocked_Run := 0;
            Put_Line ("[身]     连着三步都是零表更准 ⇒ 这张表已经被证明不准,当场重量一遍");
            declare
               Trust2 : Table.Mask;
               Ok2 : Boolean;
            begin
               Probe_Effects (L, C, F, Cam, Pts, Effs, Trust2, Ok2);
               if Ok2 then
                  for I in 0 .. Natural (Pts.Length) - 1 loop
                     Trusts (I) := Trust2;
                     Store_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob, Effs (I), Trust2, Unit_Reach, F.EE (Arm), True);
                  end loop;
               end if;
            end;
         end if;
         if Note.Blocked or else Monitor.Refusing (W) then
            Blocked_Out := True;
         end if;
         --  认不到:第一次落图给人看,连着两步才停(被挡住一团是常事)
         Lost_Run := (if Note.Lost_All then Lost_Run + 1 else 0);
         if Lost_Run = 1 then
            Dump_Picture ("lost");
         end if;
         if Lost_Run >= 2 then
            Put_Line ("[身]     连着两步认不到被跟的点 ⇒ 只记下来,照走(身体没有停下来的权利)");
         end if;
         --  🔴 认东西是脑的活:两块一样像的时候身体不许自己挑
         if Note.Unsure then
            Dump_Picture ("unsure");
            Put_Line ("[身]     两块一样像,分不清 ⇒ 只记下来,照走(身体没有停下来的权利)");
         end if;
         if Note.Not_Followed then
            Put_Line ("[身]     没照做这一步不算数,步幅已缩回;接着走");
         end if;
         --  它还挨不挨着它站着的那个面:一块东西坐在台面上时,深度切块量到的"鼓出多少"就是它自己的厚度;
         --  被提起来之后,它下面露出的还是台面,于是"鼓出多少"会长出提起来的那一截。
         --  长过它自己厚度的四分之一(比例,无量纲)就算离开了台面 —— 这是"抬起来了"的字面定义。
         declare
            H_Now : Long_Float := -1.0;
         begin
            for P of Pts loop
               if P.Kind = Thing_Pt and then P.Height > 0.0 then
                  H_Now := P.Height;
                  exit;
               end if;
            end loop;
            if Start_H < 0.0 and then H_Now > 0.0 then
               Start_H := H_Now;
            end if;
            Note.Free := Start_H > 0.0 and then H_Now > 0.0 and then H_Now - Start_H > 0.25 * Start_H;
         end;
         if Monitor.Fired (Until_Kind, W, Step_Limit, Note.Blocked, Monitor.Bounded (Selfmap.Jaw_Of (F, Arm)),
                           Monitor.Bounded (if Arm < Natural (C.Hands.Length) then C.Hands (Arm).Empty_Close else 0.0),
                           Monitor.Floor (C.Map.Jaw_Noise), Note.Touched, Note.Free)
         then
            Note.Say_Stop := (case Until_Kind is
                                when Monitor.U_Steps => S ("steps: I took the steps you asked for"),
                                when Monitor.U_Contact => S ("contact: something I was not pushing moved when I moved - I am touching it"),
                                when Monitor.U_Resist => S ("resist: I commanded a push and my body did not go"),
                                when Monitor.U_Slip => S ("slip: what I was holding has left my fingers"),
                                when Monitor.U_Settle => S ("settle: the picture stopped changing"),
                                when Monitor.U_Free => S ("free: the thing you named is no longer touching what it was standing on"));
            return;
         end if;
         declare
            All_There : Boolean := True;
         begin
            for P of Pts loop
               if not Reached (P, Fl.Track * 2.0) then
                  All_There := False;
               end if;
            end loop;
            if All_There then
               Note.Say_Stop := S ("amount: arrived (in the picture and at the same distance as my fingers)");
               return;
            end if;
         end;
         --  🔴 "我不再靠近了 ⇒ 不走了"删掉:身体没有下这个结论的权利(owner 2026-09-09)。
         --  只报,不停 —— 走到脑点名的 until 事件为止,或者走完脑给的步数。
         if Steps_Taken > 1 and then Monitor.Stalled (W) then
            Put_Line ("[身]     连着几步没更靠近(还差约 " & Codec.Fmt (Note.Err_Now, 1) & " 步)⇒ 只记下来,接着走");
         end if;
      end Judge;

      Ok : Boolean;
   begin
      Event := S ("hit the safety cap on steps");
      Steps_Taken := 0;
      Beats := 0;
      Blocked_Out := False;
      Jaw.Append (Selfmap.Jaw_Of (F, Arm));
      Fl.Track := Long_Float'Max (1.0 / Long_Float (Cw), 0.0);
      Fl.Picture := Long_Float (Integer'(if Cam < Natural (C.Map.Pic_Floor.Length) then C.Map.Pic_Floor (Cam) else 0));
      Fl.Reading := C.Map.Jaw_Noise;
      Fl.Delivery := C.Map.EE_Noise;
      Backup.Clear (Ring);
      Ready_Tables (Ok);
      if not Ok then
         Event := S ("the body stopped answering while I measured my response table");
         return;
      end if;
      for Step in 1 .. Natural'Min (Step_Cap, (if Step_Limit > 0 then Step_Limit else Step_Cap)) loop
         Plan;
         if Note.Say_Stop /= "" then
            Event := Note.Say_Stop;
            return;
         end if;
         Walk (Ok);
         if not Ok then
            Event := S ("the body refused the command");
            return;
         end if;
         Look;
         Learn;
         Judge;
         if Note.Say_Stop /= "" then
            Event := Note.Say_Stop;
            Beats := Plug.Steps (L) - Beats0;
            return;
         end if;
      end loop;
      Event := S ("steps: hit the step cap (" & Codec.Img (Steps_Taken) & ")");
   end Run_Segment;

   --  合/张:最多 Max_Iter 拍,或到画面不再变;Sweep_Cam >= 0 时把那台相机里动过的像素累进 Sweep(手指自己扫过的地方)
   procedure Jaw_Sweep (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; Target : Long_Float; Max_Iter : Natural;
                        Sweep_Cam : Integer; Sweep : in out Bools; Steps : out Natural; Reading : out Long_Float) is
      Jaw : Floats;
      Prev : Long_Float := Selfmap.Jaw_Of (F, Arm);
      Prev_Cams : Plug.Cam_Vectors.Vector := F.Cams;
      Still : Natural := 0;
      Cm : Plug.Cmd;
   begin
      Jaw.Append (Target);
      Steps := 0;
      Reading := Prev;
      --  读数是命令的回声,"停住"只认画面:每台相机连着两拍不变
      for I in 1 .. Max_Iter loop
         Cm.Kind := Plug.Ee; Cm.Arm := Arm; Cm.Pose := F.EE (Arm); Cm.Jaw := Jaw;
         exit when not Plug.Act (L, Cm) or else not Plug.Sense (L, F);
         Steps := I;
         Reading := Selfmap.Jaw_Of (F, Arm);
         if Sweep_Cam >= 0 and then Natural (Sweep_Cam) < Natural (F.Cams.Length) and then Natural (Sweep_Cam) < Natural (C.Map.Floors.Length) then
            Sweep := Picture.Either (Sweep, Picture.Moved (Prev_Cams (Natural (Sweep_Cam)).Gray, F.Cams (Natural (Sweep_Cam)).Gray, C.Map.Floors (Natural (Sweep_Cam))));
         end if;
         if abs (Reading - Prev) <= C.Map.Jaw_Noise and then Selfmap.Pictures_Still (C.Map, Prev_Cams, F.Cams) then
            Still := Still + 1;
         else
            Still := 0;
         end if;
         Prev := Reading;
         Prev_Cams := F.Cams;
         exit when Still >= 2 and then I >= 3;
      end loop;
   end Jaw_Sweep;

   --  合/张到读数不再变
   procedure Move_Jaw (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; Target : Long_Float; Steps : out Natural; Reading : out Long_Float) is
      None : Bools;
   begin
      Jaw_Sweep (L, C, F, Arm, Target, 40, -1, None, Steps, Reading);
   end Move_Jaw;

   --  生地/大步之后在世界相机里重新看见自己:手指 = 抖一下手指(合几拍再张回来),零件 = 推一下它自己的通道再推回来;
   --  动过的像素就是它,每个点认离预测最近的那团。抖的幅度不是常数:手指合"量出来的稳定拍数"那么久;零件推开机看得见的那一档。认不到的留预测、记 Lost。
   procedure Refind_Pieces (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector) is
      Arm : constant Natural := Pts (0).Arm;
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
      J0 : constant Long_Float := Selfmap.Jaw_Of (F, Arm);
      Steps_J : Natural;
      Reading : Long_Float;
      Any_Fingers : Boolean := False;
      Jaw : Floats;
      --  在累积的"动过"掩膜里给一个点认领最近的一团;Tol = 认领半径(画幅比例,无量纲)
      procedure Claim (P : in out Point; Tol, Win : Long_Float; Taken : in out Bools; Regs : Picture.Regions) is
         Best : Integer := -1;
         Bd : Long_Float := 1.0e9;
      begin
         for R in 0 .. Natural (Regs.Length) - 1 loop
            declare
               D : constant Long_Float := Sqrt ((Regs (R).Cu - P.Cu) ** 2 + (Regs (R).Cv - P.Cv) ** 2);
            begin
               if not Taken (R) and then D <= Tol and then D < Bd then
                  Bd := D; Best := R;
               end if;
            end;
         end loop;
         if Best >= 0 then
            Taken.Replace_Element (Natural (Best), True);
            declare
               Old_Z : constant Long_Float := P.Z;
               Zd : Long_Float;
            begin
               P.Cu := Regs (Best).Cu; P.Cv := Regs (Best).Cv; P.Lost := False;
               P.Box_W := Long_Float (Regs (Best).X1 - Regs (Best).X0) / Long_Float (Cw);
               P.Box_H := Long_Float (Regs (Best).Y1 - Regs (Best).Y0) / Long_Float (Ch);
               if F.Cams (Cam).Has_Depth then
                  --  深度一步跳过"距离的一成"(比例,无量纲)就是读到别的东西了
                  Zd := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, P.Cu, P.Cv, Win);
                  if not Picture.Is_Nan (Zd) and then Zd > 0.0
                    and then (Old_Z <= 0.0 or else abs (Zd - Old_Z) <= 0.1 * Old_Z)
                  then
                     P.Z := Zd;
                  end if;
               end if;
            end;
         else
            P.Lost := True;
         end if;
      end Claim;
   begin
      Jaw.Append (J0);
      for P of Pts loop
         if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm then
            Any_Fingers := True;
         end if;
      end loop;
      if Any_Fingers then
         declare
            Sweep : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
            Regs : Picture.Regions;
            Taken : Bools;
         begin
            Jaw_Sweep (L, C, F, Arm, 0.0, C.Map.Settle + 1, Integer (Cam), Sweep, Steps_J, Reading);
            Jaw_Sweep (L, C, F, Arm, J0, C.Map.Settle + 1, Integer (Cam), Sweep, Steps_J, Reading);
            Regs := Picture.Components (Sweep, Cw, Ch, Picture.Min_Pixels (Cw, Ch));
            Taken := Bool_Vectors.To_Vector (False, Regs.Length);
            for I in 0 .. Natural (Pts.Length) - 1 loop
               declare
                  P : Point := Pts (I);
               begin
                  if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm then
                     --  认领半径:一个张幅,再小也有一个跟踪窗;读深窗口 = 张幅的四分之一,再小也有半个百分点的画幅(比例,无量纲)
                     --  没真看过的位置(只是按关节推的)可能差得远 ⇒ 认领半径放到整幅画面(比例,无量纲)
                     Claim (P, (if P.Known then Long_Float'Max (Z.Span, Track_Win) else 1.0), Lobe_Win (Z, Cw, Ch), Taken, Regs);
                     if not P.Lost then
                        P.Known := True;
                     end if;
                     Pts.Replace_Element (I, P);
                  end if;
               end;
            end loop;
         end;
      end if;
      --  零件:各自推一下自己的通道(开机看得见的那一档)再推回来
      for I in 0 .. Natural (Pts.Length) - 1 loop
         declare
            P : Point := Pts (I);
         begin
            if P.Kind = Piece_Pt and then P.Chan_K < Chan.Per_Arm then
               declare
                  K : constant Natural := P.Chan_K;
                  Chn : constant Natural := Arm * Chan.Per_Arm + K;
                  A : Table.Vec := Table.Zero_Vec;
                  Deliv : Table.Vec;
                  Ok : Boolean;
                  B0 : constant Buf := F.Cams (Cam).Gray;
                  Sweep : Bools;
                  Regs : Picture.Regions;
                  Taken : Bools;
                  P0 : constant Plug.Arm_Pose := F.EE (Arm);
                  Frames : Natural;
               begin
                  if Chn < Natural (C.Map.Amp.Length) and then C.Map.Amp (Chn) > 0.0 and then Cam < Natural (C.Map.Floors.Length) then
                     A (K) := C.Map.Amp (Chn);
                     Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok);
                     Sweep := Picture.Moved (B0, F.Cams (Cam).Gray, C.Map.Floors (Cam));
                     declare
                        B1 : constant Buf := F.Cams (Cam).Gray;
                     begin
                        Selfmap.Go (L, C.Map, Arm, P0, Jaw, F, Deliv, Frames, Ok);
                        Sweep := Picture.Either (Sweep, Picture.Moved (B1, F.Cams (Cam).Gray, C.Map.Floors (Cam)));
                     end;
                     Regs := Picture.Components (Sweep, Cw, Ch, Picture.Min_Pixels (Cw, Ch));
                     Taken := Bool_Vectors.To_Vector (False, Regs.Length);
                     --  认领半径:这块自己的框那么大,再小也有一个跟踪窗;读深窗口 = 框的四分之一(比例,无量纲)
                     Claim (P, (if P.Known then Long_Float'Max (Long_Float'Max (P.Box_W, P.Box_H), Track_Win) else 1.0), Long_Float'Max (0.005, Long_Float'Max (P.Box_W, P.Box_H) * 0.25), Taken, Regs);
                     if not P.Lost then
                        P.Known := True;
                     end if;
                  else
                     P.Lost := True;
                  end if;
                  Pts.Replace_Element (I, P);
               end;
            end if;
         end;
      end loop;
      --  认到的记进身体图:这个位姿下,这只手的这些零件(手指也是零件)在这台相机里就在这儿(下次到这附近不用看)
      declare
         X : Schema.Sample;
         Gp : Schema.Part_Pos;   --  手指那块:各团合成
         All_Fingers : Boolean := True;
         N, Nz : Natural := 0;
         Zmin : Long_Float := 1.0e30;   --  哨兵(无量纲)
         Any_Part : Boolean := False;
         Zh : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
      begin
         X.Arm := Arm; X.Cam := Cam; X.Pose := F.EE (Arm);
         for P of Pts loop
            if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm then
               if P.Lost then
                  All_Fingers := False;
               end if;
               N := N + 1;
               Gp.Cu := Gp.Cu + P.Cu; Gp.Cv := Gp.Cv + P.Cv;
               if P.Blob = 1 then
                  Gp.B1u := P.Cu; Gp.B1v := P.Cv;
               else
                  Gp.B0u := P.Cu; Gp.B0v := P.Cv;
               end if;
               if P.Z > 0.0 then
                  Zmin := Long_Float'Min (Zmin, P.Z); Nz := Nz + 1;
               end if;
            elsif P.Kind = Piece_Pt and then not P.Lost and then P.Chan_K < Chan.Per_Arm then
               X.Parts (P.Chan_K) := (True, P.Cu, P.Cv, P.Z,
                                      Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), (P.Cu - P.Box_W / 2.0) * Long_Float (Cw)))), Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), (P.Cv - P.Box_H / 2.0) * Long_Float (Ch)))),
                                      Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), (P.Cu + P.Box_W / 2.0) * Long_Float (Cw)))), Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), (P.Cv + P.Box_H / 2.0) * Long_Float (Ch)))),
                                      1, P.Cu, P.Cv, 0.0, 0.0);
               Any_Part := True;
            end if;
         end loop;
         if All_Fingers and then N > 0 then
            Gp.Valid := True;
            Gp.Cu := Gp.Cu / Long_Float (N); Gp.Cv := Gp.Cv / Long_Float (N);
            Gp.N_Blobs := N;
            Gp.Z := (if Nz > 0 then Zmin else 0.0);
            Gp.X0 := Zh.X0; Gp.Y0 := Zh.Y0; Gp.X1 := Zh.X1; Gp.Y1 := Zh.Y1;   --  框先沿用开机量的(Feel 会按形心平移)
            X.Parts (Chan.Per_Arm) := Gp;
         end if;
         if (All_Fingers and then N > 0) or else Any_Part then
            Schema.Add (C.Sch, X, C.Map.EE_Noise, C.Map.Rot_Noise);
         end if;
      end;
      Put_Line ("[身]     生地/大步之后看一眼自己(手指抖一下 / 零件推一下):" &
                (if Pts (0).Lost then "没认到,按图猜" else "认到了 (" & Codec.Fmt (Pts (0).Cu, 3) & "," & Codec.Fmt (Pts (0).Cv, 3) & ") 深 " & Codec.Fmt (Pts (0).Z, 3)) &
                " · 这台相机里这只手的身体图 " & Codec.Img (Schema.Count (C.Sch, Arm, Cam)) & " 个样本");
   end Refind_Pieces;

   --  握住了没:抬一小截,看东西跟不跟我走。手上相机里 = 它的块还在握区框里;世界相机里 = 它原来那块地方空了。读数不算数(回声)。
   --  Sure = 有没有【不跟着手动的相机】能核实。没有就只能说"我说不准",不许把状态记成"手里有东西"
   procedure Held_Test (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Cam : Natural; Origin : Picture.Region;
                        Obj_Count : Natural; Held : out Boolean; Sure : out Boolean; Note : out Unbounded_String) is
      A : Table.Vec := Table.Zero_Vec;
      Deliv : Table.Vec;
      Ok : Boolean;
      Hc : constant Integer := (if Arm < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (Arm) else -1);
      Jaw : Floats;
      Seen_In_Hand : Boolean := False;
      Gone_From_Table : Boolean := False;
      Could_Judge : Boolean := False;
      --  合完之后要量出"到底发生了什么",不是只答"拿住了没":旁边有没有东西被我碰动、那一块是不是断成了两块
      World_Cam : constant Integer := (if Cam < Natural (F.Cams.Length) and then Cam_Arm (C, Cam) < 0 then Integer (Cam) else -1);
      Before_Regs : Picture.Regions;
      Moved_Others : Natural := 0;
      Pieces_Now : Natural := 0;
      Hand_U0, Hand_V0 : Long_Float := 0.0;
      Have_Hand0 : Boolean := False;
      Follows : Boolean := False;   --  它跟着我的手走了同样一段
      Follow_Note : Unbounded_String;
      --  三件量出来的事(全是测量,一句判断都没有)
      Touch_Me : Boolean := False;      --  它现在挨着我
      Off_Sup : Boolean := False;       --  它不再挨着它原来站的那个面
      Score : Long_Float := 0.0;        --  它跟着我的手走了几成(比值)
      Have_Score : Boolean := False;
      Start_Height : constant Long_Float := Origin.Height;
   begin
      if World_Cam >= 0 then
         Before_Regs := Cut_Things (C, F, Natural (World_Cam));
         --  抬之前记下:那东西在哪、我的手在哪(拿住的唯一硬证据是"它跟着我的手走了同样一段")
         Feel (C, F);   --  先按当前关节把手在这台相机里的位置算准,否则"抬之前"读的是上一段留下的旧位置
         if Track_Idx (C, Arm, Natural (World_Cam)) < Natural (C.Zones.Length) then
            declare
               Tr : constant Zone_Track := C.Zones (Track_Idx (C, Arm, Natural (World_Cam)));
            begin
               Hand_U0 := Tr.Cu; Hand_V0 := Tr.Cv; Have_Hand0 := Tr.Valid;
            end;
         end if;
      end if;
      Jaw.Append (Selfmap.Jaw_Of (F, Arm));
      A (2) := C.Map.Amp (Arm * Chan.Per_Arm + 2) * 4.0;   --  抬起 = 看得见的探针幅度的几倍(倍数,无量纲),不假设哪根轴朝上:2 号轴是身体报的第三个平移通道
      Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok);
      if Hc >= 0 and then Natural (Hc) < Natural (F.Cams.Length) then
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Natural (Hc));
            Regs : constant Picture.Regions := Cut_Things (C, F, Natural (Hc));
            Cw : constant Natural := F.Cams (Natural (Hc)).W;
            Ch : constant Natural := F.Cams (Natural (Hc)).H;
         begin
            if Z.Valid then
               Could_Judge := True;
               for R of Regs loop
                  if R.Count * 3 >= Obj_Count and then R.Cu * Long_Float (Cw) >= Long_Float (Z.X0) and then R.Cu * Long_Float (Cw) <= Long_Float (Z.X1)
                    and then R.Cv * Long_Float (Ch) >= Long_Float (Z.Y0) and then R.Cv * Long_Float (Ch) <= Long_Float (Z.Y1)
                  then
                     Seen_In_Hand := True;
                  end if;
               end loop;
            end if;
         end;
      end if;
      if Cam < Natural (F.Cams.Length) and then Cam_Arm (C, Cam) < 0 and then Origin.Count > 0 then
         Could_Judge := True;
         Gone_From_Table := World.Vanished (Cut_Things (C, F, Cam), Origin, F.Cams (Cam).W, F.Cams (Cam).H);
      end if;
      --  🔴 拿住了 = 抬手时它跟着我的手走了【同样一段】。只看"原地空了"会把【撞跑】当成拿住(FO 实测:
      --  球被撞到画面角落,原地空了,身体报"拿住",而两指之间什么都没有)
      if World_Cam >= 0 and then Have_Hand0 and then Origin.Count > 0 then
         declare
            After : constant Picture.Regions := Cut_Things (C, F, Natural (World_Cam));
            Best : Integer := -1;
            Bd : Long_Float := 0.0;
            Hand_Du, Hand_Dv : Long_Float := 0.0;
         begin
            Feel (C, F);
            if Track_Idx (C, Arm, Natural (World_Cam)) < Natural (C.Zones.Length) then
               declare
                  Tr : constant Zone_Track := C.Zones (Track_Idx (C, Arm, Natural (World_Cam)));
               begin
                  Hand_Du := Tr.Cu - Hand_U0; Hand_Dv := Tr.Cv - Hand_V0;
               end;
            end if;
            --  找抬完之后最像它的那一块(大小相近的里面离原处最近的)
            for I in 0 .. Natural (After.Length) - 1 loop
               if After (I).Count * 3 >= Origin.Count and then Origin.Count * 3 >= After (I).Count then
                  declare
                     D : constant Long_Float := Sqrt ((After (I).Cu - Origin.Cu) ** 2 + (After (I).Cv - Origin.Cv) ** 2);
                  begin
                     if Best < 0 or else D < Bd then
                        Bd := D; Best := I;
                     end if;
                  end;
               end if;
            end loop;
            declare
               Hand_Len : constant Long_Float := Sqrt (Hand_Du ** 2 + Hand_Dv ** 2);
            begin
               if Best >= 0 and then Hand_Len > 0.0 then
                  declare
                     Ou : constant Long_Float := After (Best).Cu - Origin.Cu;
                     Ov : constant Long_Float := After (Best).Cv - Origin.Cv;
                     Miss : constant Long_Float := Sqrt ((Ou - Hand_Du) ** 2 + (Ov - Hand_Dv) ** 2);
                  begin
                     --  它挪的和我的手挪的差得比"我的手挪了多少"的一半还小 ⇒ 它跟着我走
                     Follows := Miss <= 0.5 * Hand_Len;
                     --  打分 = 它跟着我走了几成。两段位移在同一张画面、同一时刻、同一把尺子上量,
                     --  尺子错了两边同样错 ⇒ 比值不变(不需要任何绝对距离)。
                     Score := Long_Float'Max (0.0, Long_Float'Min (1.0, 1.0 - Miss / Hand_Len));
                     Have_Score := True;
                     --  它挨着我没有:那一块和我这只手扫过的两瓣,框贴住 + 远近对得上
                     declare
                        Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Natural (World_Cam));
                        Cw2 : constant Natural := F.Cams (Natural (World_Cam)).W;
                        Ch2 : constant Natural := F.Cams (Natural (World_Cam)).H;
                        function Lobe_Reg (Lb : Zone.Lobe) return Picture.Region is
                           R : Picture.Region;
                        begin
                           R.X0 := Lb.X0; R.Y0 := Lb.Y0; R.X1 := Lb.X1; R.Y1 := Lb.Y1;
                           R.Top := (if Picture.Is_Nan (Z.Depth) then 0.0 else Z.Depth);
                           return R;
                        end Lobe_Reg;
                     begin
                        if Z.Valid then
                           Touch_Me :=
                             (Z.A.Valid and then Picture.Adjacent (After (Best), Lobe_Reg (Z.A), Cw2, Ch2, Track_Win * 0.5))
                             or else (Z.B.Valid and then Picture.Adjacent (After (Best), Lobe_Reg (Z.B), Cw2, Ch2, Track_Win * 0.5));
                        end if;
                     end;
                     --  它还挨不挨着它原来站的那个面:坐在台面上时"鼓出多少"就是它自己的厚度,
                     --  被提起来之后它下面露出的还是台面 ⇒ 这个数会长出提起来的那一截(比例,无量纲:它自己厚度的四分之一)
                     Off_Sup := Start_Height > 0.0 and then After (Best).Height > Start_Height * 1.25;
                     Follow_Note := S (" (my hand moved " & Codec.Fmt (Hand_Len, 3) & " of a frame, it moved " &
                                       Codec.Fmt (Sqrt (Ou ** 2 + Ov ** 2), 3) & ", they differ by " & Codec.Fmt (Miss, 3) &
                                       "; it stood " & Codec.Fmt (Start_Height, 3) & " out of the surface before and " &
                                       Codec.Fmt (After (Best).Height, 3) & " now)");
                  end;
               elsif Best < 0 then
                  Follow_Note := S (" (after the lift I could not find it anywhere in the still camera)");
               end if;
            end;
         end;
      end if;
      --  旁边动了几件 · 原地现在剩几块(断成两块的话会多出一块)
      if World_Cam >= 0 then
         declare
            After : constant Picture.Regions := Cut_Things (C, F, Natural (World_Cam));
            Tol : constant Long_Float := 1.0 / Long_Float (F.Cams (Natural (World_Cam)).W);   --  一个像素(画幅比例,无量纲)
         begin
            for Q of Before_Regs loop
               declare
                  Best : Long_Float := 0.0;
                  Found : Boolean := False;
               begin
                  for R of After loop
                     if Q.Count * 3 >= R.Count and then R.Count * 3 >= Q.Count then
                        declare
                           D : constant Long_Float := Sqrt ((R.Cu - Q.Cu) ** 2 + (R.Cv - Q.Cv) ** 2);
                        begin
                           if not Found or else D < Best then
                              Best := D; Found := True;
                           end if;
                        end;
                     end if;
                  end loop;
                  --  不是我夹的那件,却挪过了噪声地板 ⇒ 我碰动了它
                  if Found and then Best > Tol * 4.0 and then Origin.Count > 0
                    and then Sqrt ((Q.Cu - Origin.Cu) ** 2 + (Q.Cv - Origin.Cv) ** 2) > Long_Float'Max (Origin.Sig_U, Origin.Sig_V) * 2.0
                  then
                     Moved_Others := Moved_Others + 1;
                  end if;
               end;
            end loop;
            for R of After loop
               if Origin.Count > 0 and then Sqrt ((R.Cu - Origin.Cu) ** 2 + (R.Cv - Origin.Cv) ** 2) <= Long_Float'Max (Origin.Sig_U, Origin.Sig_V) * 3.0 then
                  Pieces_Now := Pieces_Now + 1;
               end if;
            end loop;
         end;
      end if;
      --  🔴🔴 身体【不许自己下"拿住了"这个结论】(owner 2026-09-09:"你只给拿住了这一个东西写死定义,
      --  这算不算作弊,世界任务无数呢")。"拿住了"是一句关于世界的知识 = 任务词,属于脑;
      --  身体只交三件【量出来的】事,一句判断都不加:
      --    ① 它现在挨着我没有(两块框贴住 + 远近对得上)
      --    ② 它还挨不挨着它原来站的那个面(鼓出多少长了没有)
      --    ③ 它跟着我的手走了几成(两段位移的比值 —— 比值,尺子错了两边同样错,自动约掉)
      --  身体不再声称这件事,它就不可能在这件事上说谎(FM/FO 各假报过一次"拿住了")。
      --  内部那个"手里有东西"的状态也只认①:那是测量,不是判断。
      Held := Touch_Me;
      Sure := Could_Judge or else Touch_Me;
      declare
         Sc : constant String :=
           (if not Have_Score then "I could not tell whether it came with me"
            --  打分本身就是两段位移的比值(比例,无量纲);这里只是把它翻成三句人话
            elsif Score > 0.8 then "it came with me almost exactly"
            elsif Score > 0.4 then "it came with me only partly - it is slipping"
            else "it did not come with me at all");
      begin
         Note := S ("after a small lift: it is " & (if Touch_Me then "" else "NOT ") & "touching me; it is "
                    & (if Off_Sup then "no longer" else "still") & " touching what it was standing on; " & Sc);
      end;
      if World_Cam >= 0 then
         Append (Note, ". While I closed and lifted, " & Codec.Img (Moved_Others) & " other thing(s) I was not pushing moved");
         if Pieces_Now >= 2 then
            Append (Note, ", and where it stood there are now " & Codec.Img (Pieces_Now) & " separate pieces");
         end if;
      end if;
   end Held_Test;

   function Mode_Line (C : Context; Until_Text : String) return String is
     ("MODE: " & (if C.Wld.Holding then "holding something with arm " & Codec.Img (Natural (C.Wld.Held_Arm) + 1) else "hands empty") &
      "; without new words from you I hold still and keep my grip as it is; this segment ended on: " & Until_Text & ".");

   --  ── 一轮 ──
   procedure Round (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context) is
      Cam : constant Natural := Natural'Min (C.Cam, Natural (F.Cams.Length) - 1);
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      RGB : Buf := F.Cams (Cam).RGB;
      Listing : Unbounded_String;
      Say : Brain.Say;
      Err : Unbounded_String;
      Report : Unbounded_String;
   begin
      C.Round_N := C.Round_N + 1;
      C.Cam := Cam;
      Feel (C, F);   --  先感觉手在哪(按位姿查身体图),不看
      --  切块 → 世界槽
      World.Observe (C.Wld, Cam, Cut_Things (C, F, Cam), Cw, Ch);
      Draw.Grid (RGB, Cw, Ch, C.Cols, C.Rows, C.Cells_U, C.Cells_V);
      Build_Listing (C, F, Cam, RGB, Listing);
      Put_Line ("[身] ── 第" & Natural'Image (C.Round_N) & " 轮(第" & Natural'Image (Cam) & " 台相机)── 这一集已用 " & Codec.Img (Plug.Steps (L)) & " 拍(开机量身体 " & Codec.Img (C.Boot_Steps) & " 拍)");
      Put (To_String (Listing));
      if C.Dump_Dir /= "" then
         Codec.Write_BMP (To_String (C.Dump_Dir) & "/grid_" & Codec.Pad6 (C.Round_N) & ".bmp", RGB, Cw, Ch);
      end if;
      declare
         --  拍数只进日志(我们自己记账),不进问脑的话:真实世界没有"步",脑只看画面
         Recent : constant String := Memory.Text (C.Mem) & To_String (C.Recent);
      begin
         if not Brain.Ask (To_String (C.Eye_Host), C.Eye_Port, To_String (C.Task_Text), To_String (Listing), Recent,
                           C.Cols, C.Rows, Natural (C.Items.Length), C.Map.N_Cams, C.Map.Arms, RGB, Cw, Ch, Say, Err)
         then
            Put_Line ("[身] 🧠 问不通(" & To_String (Err) & ")⇒ 这一拍不动,下一拍重问");
            return;
         end if;
      end;
      Put_Line ("[身] 🧠 它说:" & To_String (Say.Text) & " ‖ 看见=" & To_String (Say.See) & " · 动" & Natural'Image (Natural (Say.Moves.Length)) &
                " 条 · 抓握=" & To_String (Say.Grip) & (if Say.Grip_Arm > 0 then "(第" & Natural'Image (Say.Grip_Arm) & " 只手" & (if Say.Grip_On > 0 then ",在第" & Natural'Image (Say.Grip_On) & " 号上" else "") & ")" else "") &
                " · 到 " & To_String (Say.Until_Kind) & (if Say.Until_Kind = "steps" then Natural'Image (Say.Steps) & " 步" else "") & " 为止" &
                (if Say.Fast then " · 快" else "") & (if Say.Done then " · 它说已经做完了" else ""));
      --  它点名的世界块 ⇒ 记槽号
      for G of Say.Moves loop
         declare
            Ns : constant array (1 .. 2) of Natural := [G.Of_Item, G.Item];
         begin
         for N of Ns loop
            if N >= 1 and then N <= Natural (C.Items.Length) and then C.Items (N - 1).Kind in Thing | Thing_Remembered | Thing_Held then
               declare
                  Cs : World.Cam_State := C.Wld.Cams (Cam);
               begin
                  Cs.Named := C.Items (N - 1).Slot;
                  C.Wld.Cams.Replace_Element (Cam, Cs);
               end;
            end if;
         end loop;
         end;
      end loop;
      --  🔴 脑直接指:"我说的那个东西在第 N 格"。身体就把那一格里的那一片当成一个东西收进世界,
      --  给它一个号,以后照常跟。没有深度的时候身体分不清"什么是一个东西",而认东西本来就是脑的活;
      --  身体只负责跟住和量 —— 跟住靠位置+大小+颜色对号,不需要分割(HD/HE 实测:关掉深度后
      --  头顶相机一个东西都切不出来,而我看着图一眼就知道球在哪)。
      --  指的时候用一张【更细】的格子:列数行数各细四倍。画出来的粗格子一格太大,指出来的那一片
      --  常常落在东西旁边而不是东西上(HL 实测:框压在球的左上角,模板追的是桌面)。
      --  细两倍还不够:远处的东西在广角相机里只有二十几个像素宽,而半格有五十多,
      --  一格的中心根本落不到它身上(IJ 实测:头顶相机里没有任何一个细格中心落在球上)。
      if Say.Point_At >= 1 and then Say.Point_At <= C.Cols * 4 * C.Rows * 4 and then Cam < Natural (F.Cams.Length) then
         declare
            Cwp : constant Natural := F.Cams (Cam).W;
            Chp : constant Natural := F.Cams (Cam).H;
            Fc : constant Natural := C.Cols * 4;   --  和 brain 里那段说明同一个细度(倍数,无量纲)
            Fr : constant Natural := C.Rows * 4;
            Col_I : constant Natural := (Say.Point_At - 1) mod Fc;
            Row_I : constant Natural := (Say.Point_At - 1) / Fc;
            Hu : constant Long_Float := 0.5 / Long_Float (Fc);   --  半格(比例,无量纲)
            Hv : constant Long_Float := 0.5 / Long_Float (Fr);
            U : constant Long_Float := (Long_Float (Col_I) + 0.5) / Long_Float (Fc);
            V : constant Long_Float := (Long_Float (Row_I) + 0.5) / Long_Float (Fr);
            R : Picture.Region;
            Regs : Picture.Regions;
         begin
            R.Cu := U; R.Cv := V;
            R.X0 := Natural (Long_Float'Max (0.0, (U - Hu) * Long_Float (Cwp)));
            R.Y0 := Natural (Long_Float'Max (0.0, (V - Hv) * Long_Float (Chp)));
            R.X1 := Natural (Long_Float'Min (Long_Float (Cwp - 1), (U + Hu) * Long_Float (Cwp)));
            R.Y1 := Natural (Long_Float'Min (Long_Float (Chp - 1), (V + Hv) * Long_Float (Chp)));
            R.Count := (R.X1 - R.X0 + 1) * (R.Y1 - R.Y0 + 1);
            R.Sig_U := Hu; R.Sig_V := Hv;
            --  🔴 脑说的是【在哪儿】,身体量的是【这一片到哪儿为止】:从那一点按颜色长出去,
            --  长成了就换成长出来的框 —— 那才是那个东西本身。一格里常有一半是桌面,
            --  拿半格桌面当模板去追,追上的就是桌面(HL/HU 实测:框压在球边上,手腕越抬越高)。
            declare
               Noise_P : constant Natural := (if Cam < Natural (C.Map.Pic_Floor.Length) and then C.Map.Pic_Floor (Cam) > 0
                                              then Natural (C.Map.Pic_Floor (Cam)) else 0);
               --  门槛和切块用的是同一个:相机噪声的 2 倍、这张画面纹理的 4 倍,取大(倍数无量纲)
               Tol_P : constant Long_Float :=
                 Long_Float'Max (Long_Float (Noise_P) * 2.0 + 1.0,
                                 Picture.Texture_Level (F.Cams (Cam).RGB, Cwp, Chp) * 4.0);
               --  🔴 一个门槛不够:太紧只长出球上的一小块白(缝线、明暗都能挡住),太松一路淌到桌面。
               --  用和切块同一条判据 —— 【真东西的边,门槛翻倍它几乎不变】:门槛一档档翻倍各长一遍,
               --  取【下一档只多出不到半成】的那一档里最大的那个。长过半幅的当作没长成。
               Levels : constant Natural := 7;   --  档数(次数,无量纲)
               Gs : array (0 .. Levels - 1) of Picture.Region;
               Cn : array (0 .. Levels - 1) of Long_Float := (others => Long_Float (Cwp * Chp));
               Gok : Boolean;
               Pick : Integer := -1;
               Tl : Long_Float := Tol_P;
            begin
               for K in 0 .. Levels - 1 loop
                  Picture.Grow_From (F.Cams (Cam).RGB, Cwp, Chp, U, V, Tl, Gs (K), Gok);
                  if Gok then
                     Cn (K) := Long_Float (Gs (K).Count);
                  end if;
                  Tl := Tl * 1.4;   --  一档松四成(倍数,无量纲):翻倍太粗,球面和桌面之间只隔一档
               end loop;
               --  一直放松到【跨过去就变一个数量级】的那一档为止:东西自己的边就在那儿。
               --  球面上的缝线、明暗只让它慢慢变大(几成到一倍),而跨到桌面上是几十倍。
               if Cn (0) < Long_Float (Cwp * Chp) then
                  Pick := 0;
                  for K in 0 .. Levels - 2 loop
                     exit when Cn (K + 1) > Cn (K) * 4.0;   --  一档翻四倍以上 = 越过了这个东西的边(倍数,无量纲)
                     Pick := K + 1;
                  end loop;
               end if;
               declare
                  Cs : String (1 .. 0) := (others => ' ');
                  pragma Unreferenced (Cs);
                  Line : Unbounded_String := To_Unbounded_String ("[身] 一档档放松门槛,这一片长到:");
               begin
                  for K in 0 .. Levels - 1 loop
                     Line := Line & Natural'Image (Natural (Cn (K)));
                  end loop;
                  Put_Line (To_String (Line));
               end;
               if Pick >= 0 then
                  R := Gs (Pick);
                  --  🔴 框要把这一片自己的【边】框进来:一块纯白的中段和墙上任何一块白长得一模一样,
                  --  真正把它和别的白东西分开的是它的边。以形心为中心取对称框,再各向外放四分之一
                  --  (比例,无量纲)。这一步是给"照着样子找回它"用的,不改这一片本身。
                  declare
                     Cx : constant Long_Float := R.Cu * Long_Float (Cwp);
                     Cy : constant Long_Float := R.Cv * Long_Float (Chp);
                     Hw : constant Long_Float :=
                       Long_Float'Max (Cx - Long_Float (R.X0), Long_Float (R.X1) - Cx) * 1.25;
                     Hh : constant Long_Float :=
                       Long_Float'Max (Cy - Long_Float (R.Y0), Long_Float (R.Y1) - Cy) * 1.25;
                  begin
                     R.X0 := Natural (Long_Float'Max (0.0, Cx - Hw));
                     R.X1 := Natural (Long_Float'Min (Long_Float (Cwp - 1), Cx + Hw));
                     R.Y0 := Natural (Long_Float'Max (0.0, Cy - Hh));
                     R.Y1 := Natural (Long_Float'Min (Long_Float (Chp - 1), Cy + Hh));
                     R.Sig_U := Hw / Long_Float (Cwp);
                     R.Sig_V := Hh / Long_Float (Chp);
                  end;
                  Put_Line ("[身] 从那一点按颜色长出去(放松到第" & Integer'Image (Pick + 1) & " 档就到边了)⇒ 这一片"
                            & Natural'Image (R.Count) & " 个像素,拿它当这个东西的框");
               end if;
            end;
            Regs.Append (R);
            World.Observe (C.Wld, Cam, Regs, Cwp, Chp);
            World.Pin (C.Wld, Cam, R.Cu, R.Cv);   --  脑指的那个:钉住,以后不许因为"这一帧没对上"就说看不见
            Put_Line ("[身] 脑指了第" & Natural'Image (Say.Point_At) & " 格 ⇒ 把那一片当成一个东西收下,以后照常跟");
         end;
      end if;
      if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) and then C.Items (Say.Grip_On - 1).Kind in Thing | Thing_Remembered then
         declare
            Cs : World.Cam_State := C.Wld.Cams (Cam);
         begin
            Cs.Named := C.Items (Say.Grip_On - 1).Slot;
            C.Wld.Cams.Replace_Element (Cam, Cs);
         end;
      end if;
      C.Fast := Say.Fast;
      if Say.Look >= 2 then
         declare
            K : Natural := 2;
         begin
            for Ci in 0 .. C.Map.N_Cams - 1 loop
               if Ci /= Cam then
                  if K = Say.Look then
                     C.Cam := Ci;
                     Put_Line ("[身]    它要换到第" & Natural'Image (Ci) & " 台相机 ⇒ 下一轮在那台里列块、问、执行");
                  end if;
                  K := K + 1;
               end if;
            end loop;
         end;
      end if;
      if Say.Done then
         C.Recent := S ("you said it is already done. the body did nothing and is looking again. " & Mode_Line (C, "you said done"));
         return;
      end if;
      if C.Look_Only then
         C.Recent := S ("(look only) I did not move. " & Mode_Line (C, "look only"));
         return;
      end if;
      --  ── 执行 ──
      declare
         Until_K : constant Monitor.Until_Kind :=
           (if Say.Until_Kind = "contact" then Monitor.U_Contact elsif Say.Until_Kind = "resist" then Monitor.U_Resist
            elsif Say.Until_Kind = "slip" then Monitor.U_Slip elsif Say.Until_Kind = "settle" then Monitor.U_Settle
            elsif Say.Until_Kind = "free" then Monitor.U_Free else Monitor.U_Steps);
         Step_Limit : constant Natural := (if Say.Until_Kind = "steps" then Natural'Max (1, Say.Steps) else 0);
         Avoid : Item_Vectors.Vector;
         Pts : Point_Vectors.Vector;
         Amount : Long_Float := 1.0;   --  没说 amount 时用满(比例);说了按它的
         Event : Unbounded_String;
         Steps_Taken : Natural := 0;
         Beats : Natural := 0;
         Blocked : Boolean;
         Desc : Unbounded_String;
         Grip_Arm : constant Integer := (if Say.Grip_Arm >= 1 and then Say.Grip_Arm <= C.Map.Arms then Integer (Say.Grip_Arm) - 1 else -1);
         Did_Grip : Unbounded_String;
      --  2a 把脑说的话变成要求:别动的,目标就是它现在的位置;要动的,目标是格子或与某号的关系;
      --  抓某号,目标是"和我张开的那片地方重合"(位置 / 远近 / 看着多大 / 朝向)
      --  把去哪翻成目标:格子 / 与某号的关系(碰到它 · 上下左右 · 前后 · 离远点)。全是量出来的位置,没有写死的距离
      procedure Set_Target (G : Brain.Goal; P : in out Point; Ok_Pt : in out Boolean) is
      begin
                           --  目标
                           if G.Cell >= 1 and then G.Cell <= Natural (C.Cells_U.Length) then
                              P.Tu := C.Cells_U (G.Cell - 1); P.Tv := C.Cells_V (G.Cell - 1); P.Tz := P.Z; P.Wz := 0.0;
                              P.Desc := S ("item " & Codec.Img (G.Item) & " to cell " & Codec.Img (G.Cell));
                           elsif G.Rel /= "" and then G.Of_Item >= 1 and then G.Of_Item <= Natural (C.Items.Length) then
                              declare
                                 O : constant Item := C.Items (G.Of_Item - 1);
                                 Ow : constant Long_Float := Long_Float (O.X1 - O.X0) / Long_Float (Cw);
                                 Oh : constant Long_Float := Long_Float (O.Y1 - O.Y0) / Long_Float (Ch);
                                 Rl : constant String := To_String (G.Rel);
                              begin
                                 if not O.Located then
                                    Report := S ("goal: item " & Codec.Img (G.Of_Item) & " is not locatable right now. ");
                                    Ok_Pt := False;
                                 else
                                    P.Desc := S ("item " & Codec.Img (G.Item) & " " & Rl & " item " & Codec.Img (G.Of_Item));
                                    P.Tu := O.Cu; P.Tv := O.Cv; P.Tz := P.Z; P.Wz := 0.0;
                                    if Rl = "at" then
                                       if P.Kind = Thing_Pt and then O.Kind in Finger | Grip then
                                          --  X 装进握区:区心、区深
                                          declare
                                             Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
                                          begin
                                             --  目标取【那几瓣的共同中心】,并把"看着多大"这一项打开:
                                             --  东西真到了两指之间,它在画面里就该和合空时扫过的那片一样大。
                                             --  没有深度的时候这是唯一的"往前"信号 —— 不打开这一项,伺服只会转手腕
                                             --  在画面里挪球,最后把手臂仰到天上(HQ/HR 实测,"大小"那一行一直是 0)。
                                             declare
                                                Lu, Lv, Ln : Long_Float := 0.0;
                                             begin
                                                if Z.A.Valid then
                                                   Lu := Lu + Z.A.Cu; Lv := Lv + Z.A.Cv; Ln := Ln + 1.0;
                                                end if;
                                                if Z.B.Valid then
                                                   Lu := Lu + Z.B.Cu; Lv := Lv + Z.B.Cv; Ln := Ln + 1.0;
                                                end if;
                                                P.Tu := (if Ln > 0.0 then Lu / Ln else Z.Cu);
                                                P.Tv := (if Ln > 0.0 then Lv / Ln else Z.Cv);
                                             end;
                                             P.Tz := Z.Depth; P.Wz := (if Picture.Is_Nan (Z.Depth) then 0.0 else 1.0);
                                             P.Size := Sqrt (Long_Float'Max (0.0, P.Box_W * P.Box_H));
                                             P.Tsize := Sqrt (Long_Float'Max (0.0,
                                               (Long_Float (Z.X1 - Z.X0) / Long_Float (Cw)) * (Long_Float (Z.Y1 - Z.Y0) / Long_Float (Ch))));
                                             P.Wsize := (if P.Tsize > 0.0 and then P.Size > 0.0 then 1.0 else 0.0);
                                             --  🔴 远的时候不许把目标定在两指那个位置。手指贴着镜头,它们在这张画面里
                                             --  落在最下沿;而远处的东西在画面里根本到不了那儿 —— 硬要它去,手腕就一路
                                             --  往上仰,直到东西从画面下沿掉出去(HT/HU 实测,反复)。
                                             --  正确的目标:远 ⇒ 画面中央(手指指出去的方向);越近 ⇒ 越往两指的位置靠。
                                             --  远近用"看着多大"的比例衡量,不需要深度(比例,无量纲)。
                                             if P.Wsize > 0.0 then
                                                declare
                                                   R : constant Long_Float := Long_Float'Min (1.0, P.Size / P.Tsize);
                                                begin
                                                   P.Tu := 0.5 + (P.Tu - 0.5) * R;
                                                   P.Tv := 0.5 + (P.Tv - 0.5) * R;
                                                end;
                                             end if;
                                          end;
                                       elsif P.Kind = Piece_Pt and then O.Kind in Thing | Thing_Remembered then
                                          --  "到它那儿" = 到它那一面,不替它挑高低(owner 2026-09-08:"半腰"假设了两指从侧面夹一个立在台面上的东西,
                                          --  吸盘、拆弹的线上不成立)。要更低就由脑说"下",词表里有
                                          P.Tz := O.Depth; P.Wz := (if O.Depth > 0.0 then 1.0 else 0.0);
                                       else
                                          P.Tz := O.Depth; P.Wz := (if O.Depth > 0.0 and then P.Z > 0.0 then 1.0 else 0.0);
                                       end if;
                                    elsif Rl = "above" then
                                       P.Tv := O.Cv - Long_Float'Max (Oh, 1.0 / Long_Float (Ch));
                                    elsif Rl = "below" then
                                       P.Tv := O.Cv + Long_Float'Max (Oh, 1.0 / Long_Float (Ch));
                                    elsif Rl = "left" then
                                       P.Tu := O.Cu - Long_Float'Max (Ow, 1.0 / Long_Float (Cw));
                                    elsif Rl = "right" then
                                       P.Tu := O.Cu + Long_Float'Max (Ow, 1.0 / Long_Float (Cw));
                                    elsif Rl = "front" or else Rl = "back" then
                                       declare
                                          --  "一截" = 它自己在画面里的宽 × 它的深度(米,全是量的),没量到深度就用它鼓起的高度
                                          Sz : constant Long_Float := Long_Float'Max (O.Height, Long_Float'Max (P.Height, Ow * O.Depth));
                                       begin
                                          if O.Depth > 0.0 and then P.Z > 0.0 then
                                             P.Tz := (if Rl = "front" then O.Depth - Sz else O.Depth + Sz);
                                             P.Wz := 1.0;
                                          else
                                             --  🔴 没有深度也照样量得到"离相机近一点/远一点":
                                             --  【画面里的位置不变、自己看着变大或变小】—— 同一件事的另一种说法,
                                             --  量的是这一块【自己】的框(手离相机近,它的框就大),而手就在相机跟前、
                                             --  框有一百多个像素宽,推一下就变好几个像素,远比"远处那个小东西变大多少"灵敏。
                                             --  差多少不重要(归一那一段把每一行都压在一推之内),重要的是往哪边。
                                             --  这一条把"沿着一条视线往里走"变成一个能说出口的词 —— 在一台相机里对齐之后,
                                             --  剩下的那一维就是它(IL 实测:稳定相机里差距压到 0.003、判据报"到了",
                                             --  合手却是空的 —— 差的正是这一维,而当时没有词能说它)。
                                             P.Size := Sqrt (Long_Float'Max (0.0, P.Box_W * P.Box_H));
                                             if P.Size > 0.0 then
                                                --  往哪边:近一点 ⇒ 看着变大(比例,无量纲;大小不承重,方向承重)
                                                P.Tsize := P.Size * (if Rl = "front" then 1.25 else 0.8);
                                                P.Wsize := 1.0;
                                             end if;
                                          end if;
                                       end;
                                    elsif Rl = "down" or else Rl = "up" then
                                       --  🔴 "朝地面 / 离开地面" = 朝它站着的那个【量出来的面】。
                                       --  做法:在这张深度画面里拟合出最大的那个平面,算出这一点比那个面高出多少,
                                       --  然后把远近朝那个面推一截。面拟合不出来(或相机恰好贴着那个面看)就当没有这个词。
                                       --  这不是"世界上有张桌子"的假设:量得到就用,量不到就说没有。
                                       declare
                                          Ca, Cb, Cc : Long_Float;
                                          Plane_Ok : Boolean;
                                          Step : constant Long_Float := Long_Float'Max (O.Height, Long_Float'Max (P.Height, Ow * O.Depth));
                                       begin
                                          if F.Cams (Cam).Has_Depth then
                                             Picture.Fit_Plane (F.Cams (Cam).Depth, Cw, Ch, Ca, Cb, Cc, Plane_Ok);
                                          else
                                             Plane_Ok := False;
                                          end if;
                                          if not Plane_Ok or else P.Z <= 0.0 then
                                             Report := Report & "I cannot tell which way is toward the surface here, so I did not use that word. ";
                                             Ok_Pt := False;
                                          else
                                             declare
                                                Face : constant Long_Float := Ca * P.Cu + Cb * P.Cv + Cc;   --  这一点正下方那个面有多远
                                                Gap : constant Long_Float := Face - P.Z;                    --  比那个面高出多少(同一张画面同一把尺子)
                                             begin
                                                P.Tu := P.Cu; P.Tv := P.Cv;
                                                P.Tz := (if Rl = "down"
                                                         then P.Z + Long_Float'Min (Step, Long_Float'Max (0.0, Gap))
                                                         else P.Z - Step);
                                                P.Wz := 1.0;
                                             end;
                                          end if;
                                       end;
                                    elsif Rl = "away" then
                                       declare
                                          Du : constant Long_Float := P.Cu - O.Cu;
                                          Dv : constant Long_Float := P.Cv - O.Cv;
                                          Ln : constant Long_Float := Long_Float'Max (1.0e-9, Sqrt (Du * Du + Dv * Dv));
                                          St : constant Long_Float := Long_Float'Max (Ow, 1.0 / Long_Float (Cw));
                                       begin
                                          P.Tu := P.Cu + Du / Ln * St; P.Tv := P.Cv + Dv / Ln * St;
                                       end;
                                    end if;
                                 end if;
                              end;
                           else
                              Ok_Pt := False;
                           end if;
      end Set_Target;

      procedure Build_Goals is
      begin
            --  "保持不动" → 也变成一个点:它的目标就是它此刻在哪(误差为零)。这样解算必须选一条【不动它】的走法,
            --  而不是"没人管它" —— 一只手捏住不动、另一只手干活,全靠这一条(以前这种条目被直接跳过)
            for G of Say.Moves loop
               if G.Item >= 1 and then G.Item <= Natural (C.Items.Length) and then G.Stay then
                  declare
                     It : constant Item := C.Items (G.Item - 1);
                     P : Point;
                  begin
                     if It.Located then
                        P.Item_No := G.Item;
                        if It.Kind in Finger | Grip | Piece | Thing_Held then
                           P.Arm := It.Arm; P.Kind := Piece_Pt;
                           P.Chan_K := (if It.Kind = Piece then It.Which else Chan.Per_Arm);
                           declare
                              Tr : constant Zone_Track := C.Zones (Track_Idx (C, P.Arm, Cam));
                           begin
                              P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z; P.Known := Tr.Known;
                           end;
                        elsif Cam_Arm (C, Cam) >= 0 then
                           P.Arm := Natural (Cam_Arm (C, Cam)); P.Kind := Thing_Pt; P.Slot := It.Slot;
                           P.Cu := It.Cu; P.Cv := It.Cv; P.Z := It.Depth; P.Height := It.Height; P.Count := It.Count;
                           P.Box_W := Long_Float (It.X1 - It.X0) / Long_Float (Cw); P.Box_H := Long_Float (It.Y1 - It.Y0) / Long_Float (Ch);
                           P.Elong := It.Elong; P.Gray := It.Gray;
                        end if;
                        P.Tu := P.Cu; P.Tv := P.Cv; P.Tz := P.Z;
                        P.Wz := (if P.Z > 0.0 then 1.0 else 0.0);
                        P.Desc := S ("item " & Codec.Img (G.Item) & " stays exactly where it is");
                        if Pts.Is_Empty or else Pts (0).Arm = P.Arm then
                           Pts.Append (P);
                        end if;
                     end if;
                  end;
               end if;
               if G.Item >= 1 and then G.Item <= Natural (C.Items.Length) and then not G.Stay then
                  declare
                     It : constant Item := C.Items (G.Item - 1);
                     P : Point;
                     Own : constant Boolean := It.Kind in Finger | Grip | Thing_Held | Piece;
                     Cam_A : constant Integer := Cam_Arm (C, Cam);
                     Ok_Pt : Boolean := True;
                     Pre_Targeted : Boolean := False;
                  begin
                     Amount := Amount_Factor (G.Amount);
                     P.Item_No := G.Item;
                     if not It.Located then
                        Report := S ("goal: item " & Codec.Img (G.Item) & " is not locatable in this picture right now. ");
                        Ok_Pt := False;
                     elsif It.Kind = Piece then
                        --  我身上的一块零件:点 = 它此刻的形心,表按需量(六个通道各推一下)
                        P.Arm := It.Arm; P.Kind := Piece_Pt; P.Chan_K := It.Which; P.Blob := -1;
                        P.Cu := It.Cu; P.Cv := It.Cv; P.Z := It.Depth;
                        P.Known := C.Zones (Track_Idx (C, P.Arm, Cam)).Pieces_Known (It.Which);
                        P.Box_W := Long_Float (It.X1 - It.X0) / Long_Float (Cw); P.Box_H := Long_Float (It.Y1 - It.Y0) / Long_Float (Ch);
                     elsif Own then
                        P.Arm := It.Arm; P.Kind := Piece_Pt; P.Chan_K := Chan.Per_Arm;   --  手指 = 握合通道带的那块
                        declare
                           Tr : constant Zone_Track := C.Zones (Track_Idx (C, P.Arm, Cam));
                        begin
                           P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z; P.Known := Tr.Known or else Cam_A = Integer (P.Arm);
                        end;
                        --  这一块自己在画面里多大 —— 没有深度的时候"离相机近一点/远一点"就靠它(见 front/back)
                        P.Box_W := Long_Float (It.X1 - It.X0) / Long_Float (Cw);
                        P.Box_H := Long_Float (It.Y1 - It.Y0) / Long_Float (Ch);
                        if Cam_A = Integer (P.Arm) and then G.Rel /= "" and then G.Of_Item >= 1 and then G.Of_Item <= Natural (C.Items.Length)
                          and then C.Items (G.Of_Item - 1).Kind = Thing
                        then
                           --  自己的手上相机里"我的手到 X" = 让 X 的像素来到【两指合上的地方】:改跟 X,
                           --  🔴 而目标必须取【握区】,不能再拿 X 自己算 —— 以前那样等于"让球去到球自己那儿",
                           --  一开始就满足,身体推 80 步、连报两次"到位了",球一厘米没近(GX 实测)。
                           declare
                              O : constant Item := C.Items (G.Of_Item - 1);
                              Z : constant Zone.Hand_Zone := Zone_Of (C, It.Arm, Cam);
                              Sz : constant Long_Float := Long_Float'Max (O.Height, 1.0e-3);
                              Rl : constant String := To_String (G.Rel);
                              Lu, Lv, Ln : Long_Float := 0.0;
                           begin
                              P.Kind := Thing_Pt; P.Slot := O.Slot; P.Cu := O.Cu; P.Cv := O.Cv; P.Z := O.Depth; P.Height := O.Height; P.Count := O.Count;
                              P.Box_W := Long_Float (O.X1 - O.X0) / Long_Float (Cw); P.Box_H := Long_Float (O.Y1 - O.Y0) / Long_Float (Ch);
                              if Z.Valid then
                                 if Z.A.Valid then
                                    Lu := Lu + Z.A.Cu; Lv := Lv + Z.A.Cv; Ln := Ln + 1.0;
                                 end if;
                                 if Z.B.Valid then
                                    Lu := Lu + Z.B.Cu; Lv := Lv + Z.B.Cv; Ln := Ln + 1.0;
                                 end if;
                                 P.Tu := (if Ln > 0.0 then Lu / Ln else Z.Cu);
                                 P.Tv := (if Ln > 0.0 then Lv / Ln else Z.Cv);
                                 if not Picture.Is_Nan (Z.Depth) and then Z.Depth > 0.0 then
                                    P.Tz := (if Rl = "front" then Z.Depth - Sz
                                             elsif Rl = "back" then Z.Depth + Sz
                                             else Z.Depth);
                                    P.Wz := 1.0;
                                 else
                                    P.Tz := P.Z; P.Wz := 0.0;
                                 end if;
                                 --  没有深度的时候,这一台相机里能给的距离只剩"看着多大"。它够不着也没关系:
                                 --  下面归一那一段把每一行的要求都压在一推之内,它只负责给一个"往前"的方向,终点是碰上。
                                 --  ⚠️ 真正的距离来自【两条视线交会】—— 在两台位置不同、都不跟着这只手动的相机里
                                 --  同时把手压在东西上,两条线一交就只能是真的在那儿,而且误差还是像素,和左右上下
                                 --  一样灵敏。这台机器上两台都有(头顶那台 + 另一只胳膊腕上那台,推这只手时它不动),
                                 --  只是另一只胳膊可能正看着别处 —— 把它转过去看是【脑】的活,身体不替它决定,
                                 --  身体要做的只是把"我从这一台判断不出远近"说出来(见下面那句)。
                                 --  手上自己那台给不出距离:它那条"东西在两指连线上"的约束,转个手腕就满足了。
                                 P.Size := Sqrt (Long_Float'Max (0.0, P.Box_W * P.Box_H));
                                 P.Tsize := Sqrt (Long_Float'Max (0.0,
                                   (Long_Float (Z.X1 - Z.X0) / Long_Float (Cw)) * (Long_Float (Z.Y1 - Z.Y0) / Long_Float (Ch))));
                                 P.Wsize := (if P.Tsize > 0.0 and then P.Size > 0.0 then 1.0 else 0.0);
                                 P.Desc := S ("item " & Codec.Img (G.Of_Item) & " to come to where my fingers close");
                                 Pre_Targeted := True;
                              end if;
                           end;
                        end if;
                     elsif It.Kind = Thing and then Cam_A >= 0 then
                        P.Arm := Natural (Cam_A); P.Kind := Thing_Pt; P.Slot := It.Slot;
                        P.Cu := It.Cu; P.Cv := It.Cv; P.Z := It.Depth; P.Height := It.Height; P.Count := It.Count;
                        P.Box_W := Long_Float (It.X1 - It.X0) / Long_Float (Cw); P.Box_H := Long_Float (It.Y1 - It.Y0) / Long_Float (Ch);
                     else
                        Report := S ("goal: item " & Codec.Img (G.Item) & " is a thing I am not holding; I can only move things I hold (say grip close on it first). ");
                        Ok_Pt := False;
                     end if;
                     if Ok_Pt and then not Pre_Targeted then
                        Set_Target (G, P, Ok_Pt);
                     end if;
                     if Ok_Pt then
                        if Pts.Is_Empty or else Pts (0).Arm = P.Arm then
                           Pts.Append (P);
                        else
                           Report := Report & "goal for item " & Codec.Img (G.Item) & " needs a different arm than the first goal; I do one arm per segment. ";
                        end if;
                     end if;
                  end;
               end if;
            end loop;
            --  抓握 close 在某块上:先把那块装进握区(手上相机里跟块;否则世界相机里握区去它的顶面),笼住了才合
            if Say.Grip = "close" and then Grip_Arm >= 0 and then Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length)
              and then C.Items (Say.Grip_On - 1).Kind in Thing | Thing_Remembered and then C.Items (Say.Grip_On - 1).Located
            then
               declare
                  O : constant Item := C.Items (Say.Grip_On - 1);
                  A : constant Natural := Natural (Grip_Arm);
                  Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
                  P : Point;
                  Own_Cam : constant Boolean := Cam_Arm (C, Cam) = Integer (A);
               begin
                  if Pts.Is_Empty and then Z.Valid then
                     P.Arm := A; P.Item_No := Say.Grip_On;
                     if Own_Cam then
                        --  🔴 "抓住它" = 让它在画面里和我张开的那片地方重合:位置对上、远近对上、看着一样大、朝向一样。
                        --  这四件全是量出来的区域属性,一个字没提手指几根 —— 两指、七指、吸盘、软体臂同一句话。
                        --  只对中心那一版是错的:三个数管不住六个通道,剩下的自由度乱走(手腕拧、球转出画面)。
                        --  "看着一样大"顺带就是最稳的远近信号(离得越近越大),比一步只变 5 mm、自己抖 5 mm 的深度读数强。
                        --  圆的东西没有朝向 ⇒ 那一行谁也改不动 ⇒ 自动不参与,不需要写规则。
                        P.Kind := Thing_Pt; P.Slot := O.Slot; P.Cu := O.Cu; P.Cv := O.Cv; P.Z := O.Depth; P.Height := O.Height; P.Count := O.Count;
                        P.Box_W := Long_Float (O.X1 - O.X0) / Long_Float (Cw); P.Box_H := Long_Float (O.Y1 - O.Y0) / Long_Float (Ch);
                        P.Size := Sqrt (Long_Float'Max (0.0, P.Box_W * P.Box_H));
                        P.Ang := 2.0 * Arctan (O.Av, O.Au);
                        P.Elong := O.Elong; P.Gray := O.Gray;
                        --  🔴 目标取【合拢时扫过的那几瓣的共同中心】,不是整片扫过区的中心:手腕相机里手指离镜头很近,
                        --  扫过的那一片几乎半个屏幕,它的中心没有意义。瓣是量出来的 —— 一瓣 = 吸盘,两瓣 = 两指,
                        --  七瓣 = 七指,同一段代码,不含"几根手指"的假设。
                        declare
                           Lu : Long_Float := 0.0;
                           Lv : Long_Float := 0.0;
                           Ln : Long_Float := 0.0;
                        begin
                           if Z.A.Valid then
                              Lu := Lu + Z.A.Cu; Lv := Lv + Z.A.Cv; Ln := Ln + 1.0;
                           end if;
                           if Z.B.Valid then
                              Lu := Lu + Z.B.Cu; Lv := Lv + Z.B.Cv; Ln := Ln + 1.0;
                           end if;
                           P.Tu := (if Ln > 0.0 then Lu / Ln else Z.Cu);
                           P.Tv := (if Ln > 0.0 then Lv / Ln else Z.Cv);
                        end;
                        P.Tz := Z.Depth; P.Wz := (if Picture.Is_Nan (Z.Depth) then 0.0 else 1.0);
                        P.Tsize := Sqrt (Long_Float'Max (0.0, (Long_Float (Z.X1 - Z.X0) / Long_Float (Cw)) * (Long_Float (Z.Y1 - Z.Y0) / Long_Float (Ch))));
                        P.Tang := 2.0 * Arctan (Z.Av, Z.Au);
                        --  🔴 手指要落在【顶面到它站着的那个面之间的一半】处,不是贴着顶面。顶面和"鼓多高"都是这一块
                        --  自己量出来的,一个字没提它是什么:平的东西鼓 0 ⇒ 一半就是表面;球鼓一个球 ⇒ 一半就是赤道。
                        --  这一行盯的是这块自己的中位深度,所以把差额加在目标上(FN/FO 实测:不加就夹在球的很偏上处,一合撞飞)。
                        null;   --  不加任何高度偏移:抓在哪儿由脑说
                        --  🔴 "看着多大"这一项【重新打开】(2026-09-10)。当初关掉是因为切块忽大忽小(2026-09-08);
                        --  今天多尺度切块 + 认号修完之后块稳了,而且非关不可:
                        --  实测(GQ)六个通道的远近响应【全部通不过"推出去推回来要对得上"的复核】——
                        --  近距离下"这东西离我多远"这个读数根本不重复,拿它开车就是拿假数开车,
                        --  60 步里"远近还差多少"一直是 0.0,手只能横着挪。
                        --  而"离得越近看着越大"是画面上直接量的,不吃深度噪声 —— 驱动自己的注释早就写着
                        --  它才是最稳的远近信号。接近改由它驱动。
                        P.Wsize := 1.0;
                        --  朝向的分量 = 这块有多长条(圆的为零)
                        P.Wang := Long_Float'Max (0.0, 1.0 - 1.0 / Long_Float'Max (1.0, O.Elong));
                        P.Desc := S ("item " & Codec.Img (Say.Grip_On) & " to sit where my fingers close (same place, same distance, same apparent size, same lie)");
                     else
                        declare
                           Tr : constant Zone_Track := C.Zones (Track_Idx (C, A, Cam));
                        begin
                           P.Kind := Piece_Pt; P.Chan_K := Chan.Per_Arm; P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z; P.Known := Tr.Known;
                           --  同上:合到它那一面,高低由脑说了算
                           --  同上:手指去"顶面到桌面之间的一半"处
                           P.Tu := O.Cu; P.Tv := O.Cv; P.Tz := Grab_Depth (O);
                           P.Wz := (if O.Depth > 0.0 and then Tr.Z > 0.0 then 1.0 else 0.0);
                           P.Desc := S ("grip " & Codec.Img (A + 1) & " onto item " & Codec.Img (Say.Grip_On) & " (fingertips halfway down what sticks up)");
                        end;
                     end if;
                     Pts.Append (P);
                  end if;
               end;
            end if;
      end Build_Goals;

      --  2c 抓握:先看笼住没有,合到画面不再变,再抬一截量结果(跟我走了没有 / 我没推的东西动了几件 / 原地剩几块)
      procedure Do_Grip is
      begin
            --  ── 抓握 ──
            if Say.Grip = "close" and then Grip_Arm >= 0 then
               declare
                  A : constant Natural := Natural (Grip_Arm);
                  Caged : Boolean := True;
                  Cage_Note : Unbounded_String;
                  Steps_J : Natural;
                  Reading : Long_Float;
                  Hz : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
                  Close_Sweep : Bools;
                  Sweep_Now : Long_Float := -1.0;
               begin
                  --  笼判据:点名的那块的像素在握区框里(它的形心落在区框内),深度和手指对得上
                  if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then
                     declare
                        Pin : Point;
                        Found : Boolean := False;
                        Nz : Long_Float := 0.0;
                     begin
                        for P of Pts loop
                           if P.Item_No = Say.Grip_On then
                              if P.Kind = Piece_Pt then
                                 --  瓣点取均值 = 区心;深度取最近的那一瓣
                                 if not Found then
                                    Pin := P; Pin.Cu := 0.0; Pin.Cv := 0.0; Pin.Z := 1.0e30;
                                 end if;
                                 Pin.Cu := Pin.Cu + P.Cu; Pin.Cv := Pin.Cv + P.Cv; Nz := Nz + 1.0;
                                 if P.Z > 0.0 then
                                    Pin.Z := Long_Float'Min (Pin.Z, P.Z);
                                 end if;
                              else
                                 Pin := P;
                              end if;
                              Found := True;
                           end if;
                        end loop;
                        if Found and then Pin.Kind = Piece_Pt and then Nz > 0.0 then
                           Pin.Cu := Pin.Cu / Nz; Pin.Cv := Pin.Cv / Nz;
                           --  1e29 = "没读到"的哨兵(无量纲)
                           if Pin.Z >= 1.0e29 then
                              Pin.Z := 0.0;
                           end if;
                        end if;
                        if Found and then Pin.Kind = Thing_Pt and then Hz.Valid then
                           --  笼住 = 它已经和我张开的那片地方重合:画面里位置进了跟踪噪声、看着一样大、远近对得上。
                           --  只看"中心在区框里"不够 —— 手上相机里区框就是整个下半幅,那条判据恒真(EI/EM 实测)
                           declare
                              Dp : constant Long_Float := Sqrt ((Pin.Tu - Pin.Cu) ** 2 + (Pin.Tv - Pin.Cv) ** 2);
                              Ds : constant Long_Float := (if Pin.Wsize > 0.0 and then Pin.Tsize > 0.0 then abs (Pin.Tsize - Pin.Size) / Pin.Tsize else 0.0);
                              Tol : constant Long_Float := Long_Float'Max (Track_Win * 0.5, Hz.Span * 0.25);
                              Depth_Ok : constant Boolean := Picture.Is_Nan (Hz.Depth) or else Pin.Z <= 0.0
                                                            or else abs (Pin.Z - Hz.Depth) <= Long_Float'Max (Pin.Height, Long_Float'Max (Pin.Box_W, Pin.Box_H) * Pin.Z);
                              --  看着一样大 = 差不超过四分之一(比例,无量纲)
                              Size_Ok : constant Boolean := Pin.Wsize <= 0.0 or else Ds <= 0.25;
                           begin
                              Caged := True;   --  同上:只报不拦
                              pragma Unreferenced (Depth_Ok, Size_Ok, Tol, Dp, Ds);
                              Cage_Note := S ("in my hand camera it is " & (if Dp <= Tol then "where my fingers close" else "NOT yet where my fingers close") &
                                              ", and its distance " & (if Depth_Ok then "matches" else "does not match") & " my fingertips");
                           end;
                        elsif Found and then Pin.Kind = Piece_Pt then
                           declare
                              O : constant Item := C.Items (Say.Grip_On - 1);
                              Dist : constant Long_Float := Sqrt ((Pin.Cu - O.Cu) ** 2 + (Pin.Cv - O.Cv) ** 2);
                              Tol : constant Long_Float := Long_Float'Max (Hz.Span * 0.5, Track_Win * 0.5);
                              Want : constant Long_Float := Grab_Depth (O);
                              --  🔴 高低也要对上:只看"画面里挨着"会在【还差一截高】时就合(FO 实测:合了、把球撞飞,
                              --  而两指之间什么都没有)。容差 = 这块鼓起来的四分之一。
                              Deep_Ok : constant Boolean := O.Height <= 0.0 or else Pin.Z <= 0.0 or else Want <= 0.0
                                                           or else abs (Pin.Z - Want) <= 0.25 * O.Height;
                           begin
                              Caged := True;   --  🔴 身体不许否决合爪:脑说合就合,判据只当【说明】报回去
                              pragma Unreferenced (Deep_Ok, Tol, Dist);
                              Cage_Note := S ("in this camera my grip is " & (if Dist <= Tol then "on the thing" else "NOT yet on the thing") &
                                              ", and my fingers are " & (if Deep_Ok then "level with it" else "not level with it yet"));
                           end;
                        end if;
                     end;
                  end if;
                  if Caged then
                     --  🔴 合的时候顺便量"这次手指扫过了多少画面":比合空时明显少 ⇒ 手指没走完就被挡住了
                     --  = 中间有东西。这台机器人的爪子读数是命令的回声(见 LAB),所以读数那条路不能用;
                     --  这条是量出来的,而且不含任何"几根手指 / 什么东西"的假设。
                     Jaw_Sweep (L, C, F, A, 0.0, 40, Integer (Cam), Close_Sweep, Steps_J, Reading);
                     Sweep_Now := Picture.Fraction (Close_Sweep);
                     declare
                        Empty : constant Long_Float := C.Hands (A).Empty_Close;
                        By_Reading : Boolean := Reading - Empty > C.Map.Jaw_Noise;
                        Sure_Held : Boolean := False;
                        Note : Unbounded_String;
                        Origin : Picture.Region;
                        Obj_Count : Natural := 0;
                     begin
                        if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then
                           Origin := World.Get (C.Wld, Cam, Natural (C.Items (Say.Grip_On - 1).Slot)).Shadow;
                           Obj_Count := C.Items (Say.Grip_On - 1).Count;
                        end if;
                        Held_Test (L, C, F, A, Cam, Origin, Obj_Count, By_Reading, Sure_Held, Note);
                        if not Sure_Held then
                           By_Reading := False;   --  说不准 ⇒ 不许记成"手里有东西"(记错了下一步它就去"搬"而不是重抓)
                        end if;
                        declare
                           Es : constant Long_Float := C.Hands (A).Empty_Sweep;
                           --  少到合空时的六成以下才算被挡住(比例,无量纲:两次都是"占画面的几分之几",
                           --  同一把尺子,相机换了两边同样变,比值不变)
                           Blocked_Fingers : constant Boolean := Es > 0.0 and then Sweep_Now >= 0.0 and then Sweep_Now < Es * 0.6;
                        begin
                           Did_Grip := S ("I closed grip " & Codec.Img (A + 1) & " until the picture stopped changing; "
                                          & (if Es <= 0.0 or else Sweep_Now < 0.0 then "I could not tell whether anything stopped my fingers"
                                             elsif Blocked_Fingers then "my fingers travelled much less than they do when I close on nothing, so something stopped them"
                                             else "my fingers travelled as far as they do when I close on nothing, so nothing was between them")
                                          & "; " & To_String (Note));
                           if Blocked_Fingers then
                              By_Reading := True;
                           end if;
                        end;
                        if By_Reading then
                           C.Wld.Holding := True; C.Wld.Held_Arm := Integer (A); C.Wld.Held_Cam := Integer (Cam);
                           if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then
                              C.Wld.Held_Slot := C.Items (Say.Grip_On - 1).Slot;
                              C.Wld.Held_Origin := World.Get (C.Wld, Cam, Natural (C.Items (Say.Grip_On - 1).Slot)).Shadow;
                           else
                              C.Wld.Held_Slot := -1;
                           end if;
                           Memory.Set (C.Mem, "holding", "arm " & Codec.Img (A + 1) & " closed on item " & Codec.Img (Say.Grip_On) & " at reading " & Codec.Fmt (Reading, 3));
                        else
                           --  🔴 不自作主张张开:合完之后是"再压低一点重来"还是"松手"是脑的决定,不是身体的。
                           --  以前身体自己张开,把现场毁掉,脑连"刚才夹到哪儿了"都看不见。
                           C.Wld.Holding := False; C.Wld.Held_Arm := -1;
                           Append (Did_Grip, "; my fingers are still closed where they are - say open if you want them opened");
                        end if;
                     end;
                  else
                     Did_Grip := S ("I did NOT close grip " & Codec.Img (A + 1) & ": " & To_String (Cage_Note));
                  end if;
                  if Cage_Note /= "" and then Caged then
                     Append (Did_Grip, " (" & To_String (Cage_Note) & ")");
                  end if;
               end;
            elsif Say.Grip = "open" and then Grip_Arm >= 0 then
               declare
                  A : constant Natural := Natural (Grip_Arm);
                  Steps_J : Natural;
                  Reading : Long_Float;
               begin
                  Move_Jaw (L, C, F, A, C.Hands (A).Open_Reading, Steps_J, Reading);
                  C.Wld.Holding := False; C.Wld.Held_Arm := -1; C.Wld.Held_Slot := -1;
                  Memory.Set (C.Mem, "holding", "");
                  Did_Grip := S ("I opened grip " & Codec.Img (A + 1) & " (" & Codec.Img (Steps_J) & " steps, reading " & Codec.Fmt (Reading, 3) & ")");
               end;
            end if;
            if Did_Grip /= "" then
               Report := Report & To_String (Did_Grip) & ". ";
            end if;
      end Do_Grip;

      begin
         for N of Say.Avoid loop
            if N >= 1 and then N <= Natural (C.Items.Length) then
               Avoid.Append (C.Items (N - 1));
            end if;
         end loop;
         Build_Goals;
         if not Pts.Is_Empty then
            Expand_Lobes (C, F, Cam, Pts);
            --  生地先看一眼:我的手/零件在这台相机里的位置若只是按关节推的(没真看过),先抖/推一下认清,再按认清的位置重算各团目标,再量表、再走
            --  (EI:按关节推的手指位置差 0.2 画幅,在错地方量表 ⇒ 六列全空 ⇒ 一步没走)
            declare
               Need_Look : Boolean := False;
            begin
               for P of Pts loop
                  if P.Kind = Piece_Pt and then Cam_Arm (C, Cam) /= Integer (P.Arm) and then not P.Known then
                     Need_Look := True;
                  end if;
               end loop;
               if Need_Look then
                  Put_Line ("[身] 生地:我的手/零件在这台相机里的位置只是按关节推的 ⇒ 先动一下认清自己再走");
                  Refind_Pieces (L, C, F, Cam, Pts);
                  Feel (C, F);
                  declare
                     Su, Sv, N : Long_Float := 0.0;
                  begin
                     for P of Pts loop
                        if P.Kind = Piece_Pt and then P.Blob >= 0 and then not P.Lost then
                           Su := Su + P.Cu; Sv := Sv + P.Cv; N := N + 1.0;
                        end if;
                     end loop;
                     if N > 0.0 then
                        for I in 0 .. Natural (Pts.Length) - 1 loop
                           declare
                              P : Point := Pts (I);
                           begin
                              if P.Kind = Piece_Pt and then P.Blob >= 0 and then not P.Lost then
                                 P.Tu := P.Par_Tu + (P.Cu - Su / N); P.Tv := P.Par_Tv + (P.Cv - Sv / N);
                                 Pts.Replace_Element (I, P);
                              end if;
                           end;
                        end loop;
                     end if;
                  end;
                  declare
                     Lost_Any : Boolean := False;
                  begin
                     for P of Pts loop
                        if P.Lost then
                           Lost_Any := True;
                        end if;
                     end loop;
                     if Lost_Any then
                        Report := Report & "I moved my own piece to find it in this picture and could not see it, so I did not move toward the goal. ";
                        Pts.Clear;
                     end if;
                  end;
               end if;
            end;
            for P of Pts loop
               if P.Desc /= "" then
                  Append (Desc, (if Desc = "" then "" else " and ") & To_String (P.Desc));
               end if;
            end loop;
            if not Pts.Is_Empty then
               Put_Line ("[身] ⚙ 一起解" & Natural'Image (Natural (Pts.Length)) & " 条:" & To_String (Desc));
               Run_Segment (L, C, F, Cam, Pts, Until_K, Step_Limit, Amount, Avoid, Event, Steps_Taken, Blocked, Beats);
               Feel (C, F);
               Report := Report & "you asked " & Desc & ": " & Event & ". I took " & Codec.Img (Steps_Taken) & " pushes; ";
               Put_Line ("[身]   这一段:" & Codec.Img (Steps_Taken) & " 推 · " & Codec.Img (Beats) & " 拍 · 这一集累计 " & Codec.Img (Plug.Steps (L)) & " 拍");
               for P of Pts loop
                  if P.Blob <= 0 then
                     --  🔴 不给脑坐标/远近/"还差几步":那些数骗过我一次(GB:"还差 2.2 步"⇒ 我提前合爪合了个空)。
                     --  只说它现在在第几格,以及还差得远不远 —— 剩下的看画面。
                     --  🔴 脑每一轮真正需要的那一句:这东西在我指尖【前面 / 齐平 / 后面】。
                     --  两个读数都在同一张画面里(它多远、我合拢时扫过的那几瓣多远),直接说成人话,
                     --  不需要任何单位,也不需要脑自己去算。
                     declare
                        Hz : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
                        Own : constant Boolean := Cam_Arm (C, Cam) = Integer (P.Arm);
                        Thick : constant Long_Float := Long_Float'Max (P.Height, 1.0e-3);
                        --  没有深度的时候用"看着多大"说同一句话:比它该有的小 = 还远,大 = 走过头了
                        Sz_Now : constant Long_Float := P.Size;
                        Sz_Want : constant Long_Float := P.Tsize;
                        Where : constant String :=
                          (if P.Kind /= Thing_Pt or else not Own then ""
                           elsif Hz.Valid and then not Picture.Is_Nan (Hz.Depth) and then Hz.Depth > 0.0 and then P.Z > 0.0 then
                             (if P.Z > Hz.Depth + Thick then " and it is still beyond my fingertips"
                              elsif P.Z < Hz.Depth - Thick then " and I have gone past it - it is behind my fingertips"
                              else " and it is level with my fingertips")
                           elsif Sz_Now > 0.0 and then Sz_Want > 0.0 then
                             --  两个都是"占画幅的多少",比值(无量纲):差两成以内就算一样大
                             (if Sz_Now < Sz_Want * 0.8 then " and it still looks too small, so it is not at my fingertips yet"
                              elsif Sz_Now > Sz_Want * 1.25 then " and it now looks bigger than my fingers' own span, so I have gone past it"
                              else " and it looks the size it should when it sits between my fingers")
                           else "");
                     begin
                        Report := Report & "item " & Codec.Img (P.Item_No) & (if P.Blob = 0 then " (finger A)" else "") & " is now in cell " &
                                  Codec.Img (Cell_Of (C, P.Cu, P.Cv)) & ", " &
                                  --  "还差几步"已经是无量纲的,这里只翻成三句人话
                                  (if P.Steps_Err > 10.0 then "still a long way from where you want it"
                                   elsif P.Steps_Err > 2.0 then "getting close to where you want it"
                                   else "about where you want it") & Where & "; ";
                     end;
                  end if;
               end loop;
            else
               Event := S ("lost: could not see my own piece after moving it");
            end if;
         elsif Say.Moves.Is_Empty and then Say.Grip = "none" then
            Report := S ((if Say.See = "not_here" then "you said the thing is not in that picture; the body did not move. "
                          elsif Say.See = "unclear" then "you said you could not tell; the body did not move. "
                          else "you gave no move and no grip; the body did not move. "));
         end if;
         Do_Grip;
         Report := Report & Mode_Line (C, To_String (Event));
      end;
      C.Recent := Report;
      Put_Line ("[身]   ⇒ " & To_String (Report));
   end Round;
end Act;
