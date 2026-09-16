with Ada.Text_IO; use Ada.Text_IO;
with Ada.Strings.Fixed;
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
   --  🔴 探针为了量【远近那一行】可以往上翻到开机那一档的几倍(倍数,无量纲),只对【平移】通道:
   --  单目深度下,平移几毫米在深度图上量不出变化(NJK 实测:z 那一档 1.6 mm,深度变 -0.04 全是噪声),翻两倍就放弃 ⇒
   --  没有一根通道管远近 ⇒ 手永远不会朝球降下去。翻到 16 倍 = 两三厘米,球在画面里才看得出变近/变大。
   --  转动通道不许这么翻:0.0256 rad × 16 = 0.41 rad,JA 实测"机械臂全程在发癫"就是它。
   Probe_Cap_Pos : constant Long_Float := 16.0;
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

   --  抓在这块的哪个高度上 = 它的顶面到它站着的那个面之间的一半。
   --  两个数都是这一块【自己】在这张画面里量出来的:顶面 = 它自己深度里最近的那一档;
   --  鼓多高 = 它比周围背景高出多少。所以平的东西鼓 0 ⇒ 一半就是它的表面;
   --  球鼓一个球 ⇒ 一半就是赤道;瓶子鼓一个瓶身 ⇒ 一半就是瓶腰。没有一个字提它是什么东西,
   --  也没有一个字提手上有几根手指。量不到就退回中位深度(至少不比原来差)。
   function Grab_Depth (O : Item) return Long_Float is
     (if O.Top > 0.0 and then O.Height > 0.0 then O.Top + 0.5 * O.Height else O.Depth);

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
      --  取最窄的那一边的四分之一:落在手指身上,不碰到旁边。再小也留千分之四画幅,免得窗口小到一个像素(比例,无量纲)
      return Long_Float'Max (0.004, 0.25 * Long_Float'Min (Long_Float'Min (W1, H1), Long_Float'Min (W2, H2)));
   end Lobe_Win;

   --  差 + 差 <= 手挪的 —— 写成加法是为了不引入一个手挑的系数
   function Came_With_Me (Obj_Du, Obj_Dv, Hand_Du, Hand_Dv : Long_Float) return Boolean is
      Hand_Len : constant Long_Float := Sqrt (Hand_Du ** 2 + Hand_Dv ** 2);
      Miss : constant Long_Float := Sqrt ((Obj_Du - Hand_Du) ** 2 + (Obj_Dv - Hand_Dv) ** 2);
   begin
      return Hand_Len > 0.0 and then Miss + Miss <= Hand_Len;
   end Came_With_Me;

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

   --  🔴 长在手上的眼里【不用深度切东西】:单目深度在腕眼里连相对量都是反的(NJK 存图离线实测:球从 76 px 长到 99 px
   --  越来越近,按指头锚定的读数却从 1.67 涨到 2.5)。这只眼里干净的只有两样:东西在画面里的【位置】和【看着多大】。
   --  所以在这只眼里按明暗切:让画面自己把明暗分两拨(Otsu,分界是算出来的),亮的那一拨连成片就是一块。
   --  门槛不是人定的;横跨整幅的(桌面/墙)丢掉;深度只当记录,鼓多高一律 0(量不到就不说)。
   function Cut_Bright (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      G : constant Buf := F.Cams (Cam).Gray;
      Samp : Floats;
      T : Long_Float;
      T_First : Long_Float := 0.0;
      Mask : Bools;
      Out_R : Picture.Regions;
      I : Natural := 0;
   begin
      if Natural (G.Length) < Cw * Ch then
         return Out_R;
      end if;
      --  分界用抽样算(每 7 个像素取一个:次数,无量纲,只为省时间)
      while I < Cw * Ch loop
         Samp.Append (Long_Float (G.Element (I)));
         I := I + 7;
      end loop;
      T := Picture.Split (Samp);
      if Picture.Is_Nan (T) then
         return Out_R;   --  全一样 ⇒ 这只眼里按明暗切不出东西,如实交空
      end if;
      T_First := T;
      --  🔴 分两次:第一刀分的是"暗桌面 vs 亮的一切"(NJK 存图离线:分界 111,浅色木纹和白球并成一块);
      --  在亮的那一拨里再分一刀,才把最亮的一撮(白球、乐高的黄)从浅木纹里切出来。两刀的分界都是算出来的。
      declare
         Upper : Floats;
         T2 : Long_Float;
      begin
         for X of Samp loop
            if X > T then
               Upper.Append (X);
            end if;
         end loop;
         T2 := Picture.Split (Upper);
         if not Picture.Is_Nan (T2) then
            T := T2;
         end if;
      end;
      --  🔴 暗的东西也要认(黑键盘、红乐高、深色把手):在暗的那一拨里再分一刀,最暗的一撮单独成块;
      --  贴着画面边的暗块是我自己的胳膊/手指(它们从画面外伸进来),丢掉
      declare
         Lower : Floats;
         T_Low : Long_Float;
         Dark : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
      begin
         for X of Samp loop
            if X <= T_First then
               Lower.Append (X);
            end if;
         end loop;
         T_Low := Picture.Split (Lower);
         if not Picture.Is_Nan (T_Low) then
            for J in 0 .. Cw * Ch - 1 loop
               if Long_Float (G.Element (J)) < T_Low then
                  Dark.Replace_Element (J, True);
               end if;
            end loop;
            for R of Picture.Components (Dark, Cw, Ch, Picture.Min_Pixels (Cw, Ch)) loop
               declare
                  Q : Picture.Region := R;
                  Edge : constant Boolean := R.X0 = 0 or else R.Y0 = 0 or else R.X1 + 1 >= Cw or else R.Y1 + 1 >= Ch;
               begin
                  if not Edge then
                     Q.Height := 0.0; Q.Top := 0.0; Q.Depth := 0.0;
                     Out_R.Append (Q);
                  end if;
               end;
            end loop;
         end if;
      end;
      Mask := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
      for J in 0 .. Cw * Ch - 1 loop
         if Long_Float (G.Element (J)) > T then
            Mask.Replace_Element (J, True);
         end if;
      end loop;
      for R of Picture.Components (Mask, Cw, Ch, Picture.Min_Pixels (Cw, Ch)) loop
         declare
            Q : Picture.Region := R;
            Span_W : constant Boolean := R.X0 = 0 and then R.X1 + 1 >= Cw;
            Span_H : constant Boolean := R.Y0 = 0 and then R.Y1 + 1 >= Ch;
         begin
            if not Span_W and then not Span_H then
               Q.Height := 0.0; Q.Top := 0.0;
               if F.Cams (Cam).Has_Depth then
                  declare
                     --  读深窗口 = 这块自己最窄边的四分之一,再小也有半个百分点的画幅(比例,无量纲);只当记录
                     Zd : constant Long_Float := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, R.Cu, R.Cv,
                                                                    Long_Float'Max (0.005, 0.25 * Long_Float'Min (Long_Float (R.X1 - R.X0 + 1) / Long_Float (Cw),
                                                                                                                 Long_Float (R.Y1 - R.Y0 + 1) / Long_Float (Ch))));
                  begin
                     Q.Depth := (if Picture.Is_Nan (Zd) then 0.0 else Zd);
                  end;
               end if;
               Out_R.Append (Q);
            end if;
         end;
      end loop;
      return Out_R;
   end Cut_Bright;

   function Cut_Things_Raw (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Raw : Picture.Regions;
      Kept : Picture.Regions;
   begin
      --  长在手上的眼、或者根本没有深度的眼(真机:手机 + 腕部 RGB):按明暗切(见 Cut_Bright);
      --  长在手上的眼只有一块都切不出时才退回深度那一路
      if Cam_Arm (C, Cam) >= 0 or else not F.Cams (Cam).Has_Depth then
         Raw := Cut_Bright (C, F, Cam);
         if not Raw.Is_Empty then
            for R of Raw loop
               declare
                  Mine : Boolean := False;
               begin
                  --  自己手上的眼里,手指是黑的,按明暗切出来的亮块不可能是手指 ⇒ 不按握区框剔"我自己"
                  --  (GC1:球一进画面下半幅,形心落进瓣框就被当成手扔掉,清单里一个可见的东西都没有)
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
      Raw := Picture.Cut (F.Cams (Cam).Depth, Cw, Ch, Cut_Window (C, Cam, F), Sigma_Mult);
      --  🔴 一个都没切出来 ⇒ 换个大一号的尺子再看一遍。
      --  "鼓出来"是相对周围说的:窗口比这块东西还小的时候,这块东西【自己就是周围】,于是它鼓 0、整个消失。
      --  实测(FO):手伸到球跟前时,手腕相机里球占了小半幅画面,深度这一路一块也切不出来,退化成按颜色切出几十块碎片
      --  ⇒ 越靠近越看不见要抓的东西。尺子一路放大到半幅画面为止,先看出东西的那一档算数。
      declare
         Base : constant Long_Float := Cut_Window (C, Cam, F);
         Win : Long_Float;
      begin
         Win := Base;
         while Raw.Is_Empty and then Win < 0.5 loop
            Win := Win * 2.0;
            Raw := Picture.Cut (F.Cams (Cam).Depth, Cw, Ch, Long_Float'Min (0.5, Win), Sigma_Mult);
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
               return ", " & Codec.Fmt (D, 1) & " cells from the thing you last named, in the " & Half & " half of the picture";
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
               Push (It, "a thing, now in cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & " (" & Codec.Img (It.Count) & " px, standing " &
                     Codec.Fmt (It.Height, 3) & " out of the surface)" & Rel (It.Cu, It.Cv), Draw.Green, 2);
            elsif Sl.Seen then
               It.Kind := Thing_Remembered; It.Located := True;
               It.Cu := Sl.Shadow.Cu; It.Cv := Sl.Shadow.Cv; It.Depth := Sl.Shadow.Depth; It.Height := Sl.Shadow.Height; It.Top := Sl.Shadow.Top; It.Count := Sl.Shadow.Count;
               It.X0 := Sl.Shadow.X0; It.Y0 := Sl.Shadow.Y0; It.X1 := Sl.Shadow.X1; It.Y1 := Sl.Shadow.Y1;
               It.Au := Sl.Shadow.Au; It.Av := Sl.Shadow.Av; It.Elong := Sl.Shadow.Elong;
               Push (It, "a thing you saw before, remembered where it was last seen, cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)) &
                     " (not visible right now - probably under my hand; " & Codec.Img (It.Count) & " px)", Draw.Dim_Green, 1);
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
      To_Grip : Boolean := False;               --  自己手上相机里"我的手到 X"= 改跟 X、目标是握区(手在自己眼里不动,驱动的是 X 的像素)
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
                  P.Cu := Pred_U; P.Cv := Pred_V; P.Lost := True;
               end if;
            end;
      end case;
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
            Cap_Amp : constant Long_Float := C.Map.Amp (Chn) * (if K < Chan.Pos_Channels then Probe_Cap_Pos else Cap_Mult);
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
                     Seen_Enough : Boolean := True;
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
                           if abs Deliv (K) > C.Map.EE_Noise and then (Ran >= Floor_Px or else Dz >= Floor_Z (I)) then
                              declare
                                 Col : Table.Vec3;
                              begin
                                 Col (0) := (P.Cu - W0.Cu) / Deliv (K);
                                 Col (1) := (P.Cv - W0.Cv) / Deliv (K);
                                 Col (2) := (if P.Z > 0.0 and then W0.Z > 0.0 then (P.Z - W0.Z) / Deliv (K) else 0.0);
                                 --  推一下这块看着变大变小多少、转了多少(圆的东西转不出来 ⇒ 这一列恒零 ⇒ 自动不参与)
                                 --  只有变化过了自己的噪声地板才敢写进表,否则这一格留零(留零 = 归一时这一行自动不参与)
                                 Col (3) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (P.Size - W0.Size) > Floor_S (I)
                                             then (P.Size - W0.Size) / Deliv (K) else 0.0);
                                 Col (4) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (Wrap (P.Ang - W0.Ang)) > Floor_A (I)
                                             then Wrap (P.Ang - W0.Ang) / Deliv (K) else 0.0);
                                 Table.Set_Col (Effs (I), K, Col);
                              end;
                           else
                              Seen_Enough := False;
                           end if;
                           Pts.Replace_Element (I, P);
                        end;
                     end loop;
                     if Seen_Enough then
                        Trust (K) := True;
                        Put_Line ("[身]     通道" & Natural'Image (Chn) & ":命令 " & Codec.Fmt (Amp, 4) & " 实到 " & Codec.Fmt (Deliv (K), 4) & " ⇒ 点跑了 " &
                                  Codec.Fmt (Ran_Max, 4) & " 画幅,深度变 " & Codec.Fmt ((if Pts (0).Z > 0.0 and then Was (0).Z > 0.0 then Pts (0).Z - Was (0).Z else 0.0), 4));
                     end if;
                     declare
                        Before2 : constant Buf := F.Cams (Cam).Gray;
                     begin
                        Selfmap.Go (L, C.Map, Arm, P0, Jaw, F, Back, Frames, Ok2);
                        if not Ok2 then
                           Ok := False;
                           return;
                        end if;
                        for I in 0 .. Natural (Pts.Length) - 1 loop
                           declare
                              P : Point := Pts (I);
                           begin
                              Retrack (C, F, Cam, Before2, P, Was (I).Cu, Was (I).Cv, True);
                              P.Cu := Was (I).Cu; P.Cv := Was (I).Cv; P.Z := Was (I).Z;   --  推回起点了:点回到原处(比光流往返的累积误差可信)
                              Pts.Replace_Element (I, P);
                           end;
                        end loop;
                     end;
                     exit when Trust (K);
                     if Amp * 2.0 > Cap_Amp then
                        Put_Line ("[身]     通道" & Natural'Image (Chn) & ":到 " & Codec.Fmt (Amp, 4) & " 点还没动过地板(跑 " & Codec.Fmt (Ran_Max, 4) & " 画幅,地板 " & Codec.Fmt (Floor_Px, 4) & ")⇒ 这一段不用它");
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
                  if not Found then
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
                  Draw.Numbered_Box (RGB, Cw, Ch,
                                     Natural (Long_Float'Max (0.0, (P.Cu - Hw) * Long_Float (Cw))),
                                     Natural (Long_Float'Max (0.0, (P.Cv - Hh) * Long_Float (Ch))),
                                     Natural (Long_Float'Min (Long_Float (Cw - 1), (P.Cu + Hw) * Long_Float (Cw))),
                                     Natural (Long_Float'Min (Long_Float (Ch - 1), (P.Cv + Hh) * Long_Float (Ch))),
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
               if Idx >= 0 and then (for some K in 0 .. Chan.Per_Arm - 1 => C.Tables (Natural (Idx)).Trust (K))
                 and then C.Tables (Natural (Idx)).Has_Pose
                 and then (for all K in 0 .. Chan.Per_Arm - 1 =>
                             abs Chan.Delivered (C.Tables (Natural (Idx)).Pose, F.EE (Arm)) (K)
                             <= Long_Float'Max (1.0e-6, C.Map.Amp (Arm * Chan.Per_Arm + K)) * Cap_Mult * C.Tables (Natural (Idx)).Reach (K))
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
                  T.Err (0) := P.Tu - P.Cu;  T.W (0) := Near;
                  T.Err (1) := P.Tv - P.Cv;  T.W (1) := Near;
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
                  Px := Long_Float'Max (Px, Sqrt (Effs (I).B (K, 0) ** 2 + Effs (I).B (K, 1) ** 2));
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
                     Note.Cap (K) := Long_Float'Min (Am * Cap_Mult * Reach (K),
                                                     (if Known_All then Track_Win / Px else Am * Cap_Mult * Reach (K))) * Amount;
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
         --  求稳不求快:一步里任何被跟的点在画面里最多跑一个跟踪窗
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
                     if Nu < Track_Win or else Nu > 1.0 - Track_Win or else Nv < Track_Win or else Nv > 1.0 - Track_Win then
                        Hit := True;
                     end if;
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
               exit when not Hit;
               if Round = 4 then
                  Note.Say_Stop := S ("stopped: every step would push what I am tracking out of my sight, or put some part of me onto a thing I must not touch");
                  return;
               end if;
               Scale := Scale * 0.5;
            end;
         end loop;
         for K in 0 .. Chan.Per_Arm - 1 loop
            Note.Cmd (K) := Note.Cmd (K) * Scale * Trust;   --  表有多准就走多少(不然每步走过头,下一步再拉回来,来回晃)
         end loop;
         if Table.Norm (Note.Cmd, Chan.Per_Arm) <= C.Map.EE_Noise then
            Note.Say_Stop := S ("amount: already there (what is left to push is within my own noise)");
         end if;
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
            Note.Say_Stop := S ("could not solve which channels to push");
            return;
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
                           --  🔴 距离不可能是负的:算出来的【新】深度也要是正的,否则这一步的预测就是错的,宁可留旧值
                           --  (GV 实测:预测把它推成 -0.526 ⇒ 远近整行作废 ⇒ 身体只在画面上对齐、停在离球 0.08 画幅处还报"差 0.005 m")
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
                        --  同上:算出来的新深度必须仍是正的
                        if W0.Z > 0.0 and then W0.Z + Pr (2) > 0.0 then
                           P.Z := W0.Z + Pr (2);
                        end if;
                     end if;
                     --  🔴🔴 位置从姿态表里查出来之后,【深度要在深度图上就地重读】。
                     --  以前这一路的 Z 全是姿态表里存的那个数加上表的预测 —— 也就是【猜】出来的,从来没被眼睛校过。
                     --  IY 实测(真深度也开着):球读 0.640 m,而挨着它的指尖读 2.40 m,差了四倍;每一步 Z 平滑地变 0.007,
                     --  像预测不像测量。于是"远近"那一行永远差着,手在前后方向上要么不动要么一路顶,合手全是空的。
                     --  LAB 判定这就是 FO"夹太靠上、一合把球顶飞"的根子。
                     --  收读数前先过闸:读窗里同时有指头和它后面那个面时,读数会在两者之间来回跳(JD:0.61↔0.45 每步翻)
                     --  ⇒ 一步之内不许跳过"表预测的变化 + 距离的一成"(一成 = 0.1 倍距离,比例,无量纲;和 Retrack 里那道闸同一个数)。
                     if F.Cams (Cam).Has_Depth then
                        declare
                           Zn : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
                           Zd : constant Long_Float :=
                             Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, P.Cu, P.Cv, Lobe_Win (Zn, Cw, Ch));
                           Old_Z : constant Long_Float := P.Z;
                        begin
                           if not Picture.Is_Nan (Zd) and then Zd > 0.0 then
                              --  一成 = 0.1 倍距离(比例,无量纲;和 Retrack 里那道闸同一个数)
                              if Old_Z <= 0.0 or else abs (Zd - Old_Z) <= abs (Pr (2)) + 0.1 * Old_Z then
                                 P.Z := Zd;
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
                  if Any_Meas and then Pred_Ok and then (not Note.Halted) and then not Note.Not_Followed then
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
         if Note.Blocked or else Monitor.Refusing (W) then
            Blocked_Out := True;
         end if;
         --  认不到:第一次落图给人看,连着两步才停(被挡住一团是常事)
         Lost_Run := (if Note.Lost_All then Lost_Run + 1 else 0);
         if Lost_Run = 1 then
            Dump_Picture ("lost");
         end if;
         if Lost_Run >= 2 then
            Note.Say_Stop := S ("lost sight: two steps in a row I could not find what I am tracking in this picture; I stopped rather than move blind");
            return;
         end if;
         --  🔴 认东西是脑的活:两块一样像的时候身体不许自己挑
         if Note.Unsure then
            Dump_Picture ("unsure");
            Note.Say_Stop := S ("not sure which one is yours: two things here look equally like the one you named (same size, same distance); "
                                & "I stopped instead of guessing - look again and tell me its number");
            return;
         end if;
         if Note.Not_Followed then
            Put_Line ("[身]     没照做这一步不算数,步幅已缩回;接着走");
         end if;
         if Monitor.Fired (Until_Kind, W, Step_Limit, Note.Blocked, Monitor.Bounded (Selfmap.Jaw_Of (F, Arm)),
                           Monitor.Bounded (if Arm < Natural (C.Hands.Length) then C.Hands (Arm).Empty_Close else 0.0),
                           Monitor.Floor (C.Map.Jaw_Noise), Note.Touched)
         then
            Note.Say_Stop := (case Until_Kind is
                                when Monitor.U_Steps => S ("steps: I took the steps you asked for"),
                                when Monitor.U_Contact => S ("contact: something I was not pushing moved when I moved - I am touching it"),
                                when Monitor.U_Resist => S ("resist: I commanded a push and my body did not go"),
                                when Monitor.U_Slip => S ("slip: what I was holding has left my fingers"),
                                when Monitor.U_Settle => S ("settle: the picture stopped changing"),
                                when Monitor.U_Stall => S ("amount: stopped getting closer - for several steps in a row the gap did not shrink, as you asked me to report"));
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
         if Steps_Taken > 1 and then Monitor.Stalled (W) then
            Note.Say_Stop := S ("amount: stopped getting closer (still about " & Codec.Fmt (Note.Err_Now, 1) &
                                " pushes away) - either something holds me or this arm cannot reach farther from here");
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
      Still_R : Natural := 0;
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
         if Sweep_Cam < 0 then
            Put_Line ("[身]     爪 第" & Codec.Img (I) & " 拍:读数 " & Codec.Fmt (Reading, 3) & "(要去 " & Codec.Fmt (Target, 3) & ")");
         end if;
         if Sweep_Cam >= 0 and then Natural (Sweep_Cam) < Natural (F.Cams.Length) and then Natural (Sweep_Cam) < Natural (C.Map.Floors.Length) then
            Sweep := Picture.Either (Sweep, Picture.Moved (Prev_Cams (Natural (Sweep_Cam)).Gray, F.Cams (Natural (Sweep_Cam)).Gray, C.Map.Floors (Natural (Sweep_Cam))));
         end if;
         if abs (Reading - Prev) <= C.Map.Jaw_Noise and then Selfmap.Pictures_Still (C.Map, Prev_Cams, F.Cams) then
            Still := Still + 1;
         else
            Still := 0;
         end if;
         --  合/张(不是开机扫描)时只看读数:读数连着三拍不变就是到底了,画面里球的影子在动不算(GB5:40 拍里 30 拍白等)
         if abs (Reading - Prev) <= C.Map.Jaw_Noise then
            Still_R := Still_R + 1;
         else
            Still_R := 0;
         end if;
         Prev := Reading;
         Prev_Cams := F.Cams;
         exit when Still >= 2 and then I >= 3;
         exit when Sweep_Cam < 0 and then Still_R >= 3 and then I >= 3;
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
                  if not Picture.Is_Nan (Zd) and then (Old_Z <= 0.0 or else abs (Zd - Old_Z) <= 0.1 * Old_Z) then
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
                                      Natural (Long_Float'Max (0.0, (P.Cu - P.Box_W / 2.0) * Long_Float (Cw))), Natural (Long_Float'Max (0.0, (P.Cv - P.Box_H / 2.0) * Long_Float (Ch))),
                                      Natural (Long_Float'Min (Long_Float (Cw - 1), (P.Cu + P.Box_W / 2.0) * Long_Float (Cw))), Natural (Long_Float'Min (Long_Float (Ch - 1), (P.Cv + P.Box_H / 2.0) * Long_Float (Ch))),
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
      --  合完之后要量出"到底发生了什么",不是只答"拿住了没":旁边有没有东西被我碰动、那一块是不是断成了两块。
      --  🔴 判"拿住了没"要用一台【不跟着这只手动】的相机。以前只认"当前这只眼",而当前这只正好长在手上时
      --  ⇒ 没人能核实 ⇒ "我说不准" ⇒ 松手(GI 实测:合到底了又张开)。改成:当前眼不长在手上就用它;
      --  否则找任何一台不长在任何胳膊上的,再不行找不长在【这条】胳膊上的。它原来在那台相机里的哪儿,
      --  用那台相机自己记着的影子(脑在那台里点过名),不能拿当前眼睛里的位置去比。
      function Still_Cam_Of return Integer is
      begin
         if Cam < Natural (F.Cams.Length) and then Cam_Arm (C, Cam) < 0 then
            return Integer (Cam);
         end if;
         for Cm in 0 .. Natural (F.Cams.Length) - 1 loop
            if Cam_Arm (C, Cm) < 0 then
               return Cm;
            end if;
         end loop;
         for Cm in 0 .. Natural (F.Cams.Length) - 1 loop
            if Cam_Arm (C, Cm) /= Integer (Arm) then
               return Cm;
            end if;
         end loop;
         return -1;
      end Still_Cam_Of;
      World_Cam : constant Integer := Still_Cam_Of;
      function Org_Of return Picture.Region is
      begin
         if World_Cam < 0 or else Natural (World_Cam) = Cam then
            return Origin;
         end if;
         if Natural (World_Cam) < Natural (C.Wld.Cams.Length) and then C.Wld.Cams (Natural (World_Cam)).Named >= 0
           and then Natural (C.Wld.Cams (Natural (World_Cam)).Named) < World.Count (C.Wld, Natural (World_Cam))
         then
            return World.Get (C.Wld, Natural (World_Cam), Natural (C.Wld.Cams (Natural (World_Cam)).Named)).Shadow;
         end if;
         return (others => <>);   --  那台相机里脑没点过名 ⇒ 判不了(Count = 0)
      end Org_Of;
      Org : Picture.Region := Org_Of;
      Before_Regs : Picture.Regions;
      Moved_Others : Natural := 0;
      Pieces_Now : Natural := 0;
      --  抬之前我的手在【那台不跟着我动的相机】里的哪儿(拿住的唯一硬证据是"它跟着我的手走了同样一段")
      Hand_U0, Hand_V0 : Long_Float := 0.0;
      Have_Hand0 : Boolean := False;
      Follows : Boolean := False;
      Found_After : Boolean := False;
      Follow_Note : Unbounded_String;
   begin
      if World_Cam >= 0 then
         Before_Regs := Cut_Things (C, F, Natural (World_Cam));
         if Track_Idx (C, Arm, Natural (World_Cam)) < Natural (C.Zones.Length) then
            declare
               Tr : constant Zone_Track := C.Zones (Track_Idx (C, Arm, Natural (World_Cam)));
            begin
               Hand_U0 := Tr.Cu; Hand_V0 := Tr.Cv; Have_Hand0 := Tr.Valid;
            end;
         end if;
         --  那台不动的相机里脑没点过名(名字是在手上那只眼里认的)⇒ 拿【离我的手最近的那一块】当它:
         --  合手时它就在两指之间,不动的眼里离手最近的东西就是它。找不到就照旧"判不了"
         if Org.Count = 0 and then Have_Hand0 then
            declare
               Zw : constant Zone.Hand_Zone := Zone_Of (C, Arm, Natural (World_Cam));
               Reach_Frac : constant Long_Float := Long_Float'Max (Track_Win, Zw.Span) * 2.0;
               Bd : Long_Float := 1.0e9;
            begin
               for R of Before_Regs loop
                  declare
                     D : constant Long_Float := Sqrt ((R.Cu - Hand_U0) ** 2 + (R.Cv - Hand_V0) ** 2);
                  begin
                     if D <= Reach_Frac and then D < Bd then
                        Bd := D; Org := R;
                     end if;
                  end;
               end loop;
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
      if World_Cam >= 0 and then Org.Count > 0 then
         Could_Judge := True;
         Gone_From_Table := World.Vanished (Cut_Things (C, F, Natural (World_Cam)), Org, F.Cams (Natural (World_Cam)).W, F.Cams (Natural (World_Cam)).H);
      end if;
      --  🔴 拿住了 = 抬手时它跟着我的手走了【同样一段】。只看"原地空了"会把【撞跑】当成拿住(FO 实测:
      --  球被撞到画面角落,原地空了,身体报"拿住",而两指之间什么都没有)
      if World_Cam >= 0 and then Have_Hand0 and then Org.Count > 0 then
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
                  if Tr.Valid then
                     Hand_Du := Tr.Cu - Hand_U0; Hand_Dv := Tr.Cv - Hand_V0;
                  else
                     Have_Hand0 := False;   --  这一抬我把自己的手跟丢了 ⇒ 判不了,老实说
                  end if;
               end;
            end if;
            --  抬完之后最像它的那一块:大小相近的里面离原处最近的
            for I in 0 .. Natural (After.Length) - 1 loop
               if After (I).Count * 3 >= Org.Count and then Org.Count * 3 >= After (I).Count then
                  declare
                     D : constant Long_Float := Sqrt ((After (I).Cu - Org.Cu) ** 2 + (After (I).Cv - Org.Cv) ** 2);
                  begin
                     if Best < 0 or else D < Bd then
                        Bd := D; Best := I;
                     end if;
                  end;
               end if;
            end loop;
            Found_After := Best >= 0;
            if Found_After and then Have_Hand0 then
               declare
                  Ou : constant Long_Float := After (Best).Cu - Org.Cu;
                  Ov : constant Long_Float := After (Best).Cv - Org.Cv;
               begin
                  Follows := Came_With_Me (Ou, Ov, Hand_Du, Hand_Dv);
                  Follow_Note := S (" (my hand moved " & Codec.Fmt (Sqrt (Hand_Du ** 2 + Hand_Dv ** 2), 3) &
                                    " of a frame, it moved " & Codec.Fmt (Sqrt (Ou ** 2 + Ov ** 2), 3) &
                                    ", the two differ by " & Codec.Fmt (Sqrt ((Ou - Hand_Du) ** 2 + (Ov - Hand_Dv) ** 2), 3) & ")");
               end;
            elsif not Found_After then
               Follow_Note := S (" (after the lift I could not find it anywhere in the camera that does not move with me)");
            else
               Follow_Note := S (" (I lost track of my own hand during the lift, so I cannot tell)");
            end if;
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
                  if Found and then Best > Tol * 4.0 and then Org.Count > 0
                    and then Sqrt ((Q.Cu - Org.Cu) ** 2 + (Q.Cv - Org.Cv) ** 2) > Long_Float'Max (Org.Sig_U, Org.Sig_V) * 2.0
                  then
                     Moved_Others := Moved_Others + 1;
                  end if;
               end;
            end loop;
            for R of After loop
               if Org.Count > 0 and then Sqrt ((R.Cu - Org.Cu) ** 2 + (R.Cv - Org.Cv) ** 2) <= Long_Float'Max (Org.Sig_U, Org.Sig_V) * 3.0 then
                  Pieces_Now := Pieces_Now + 1;
               end if;
            end loop;
         end;
      end if;
      --  🔴 "拿住了"唯一分得开的硬证据:抬手时它【跟着我的手走了同样一段】。
      --  "它原来待的地方空了"分不开【撞跑】—— 球被撞到画面角落,原地照样空了,身体照样报"拿住"(FO 实测)。
      --  手上相机里"还在握区框里"更不算数 —— 那个框在手上相机里几乎是半个屏幕(FM 实测)。
      --  判不了就老实说"我说不准",不许自称拿住。
      Held := (if World_Cam >= 0 and then Have_Hand0 and then Found_After then Follows else Seen_In_Hand);
      Sure := World_Cam >= 0 and then Have_Hand0 and then Found_After;
      if Sure and then Follows then
         Note := S ("after a small lift it came with my hand ⇒ held") & Follow_Note
                 & (if Seen_In_Hand then ", and my hand camera still shows it between my fingers" else "");
      elsif Sure then
         Note := S ("after a small lift it did NOT come with my hand ⇒ not held")
                 & (if Gone_From_Table then S (" - and its old place is empty, so I knocked it away rather than picked it up") else S (""))
                 & Follow_Note
                 & (if Seen_In_Hand then " (my hand camera still shows something between my fingers, which proves nothing)" else "");
      elsif World_Cam >= 0 then
         Note := S ("after a small lift I could not judge whether it came with me") & Follow_Note;
      elsif Seen_In_Hand then
         Note := S ("after a small lift the thing is still inside my grip box in my hand camera; no still camera could check, so I am not sure");
      elsif Could_Judge then
         Note := S ("after a small lift the thing did not come with me ⇒ not held");
      else
         Note := S ("I could not judge whether it is held (no camera could see it)");
      end if;
      if World_Cam >= 0 then
         Append (Note, ". While I closed and lifted, " & Codec.Img (Moved_Others) & " other thing(s) I was not pushing moved");
         if Pieces_Now >= 2 then
            Append (Note, ", and where it stood there are now " & Codec.Img (Pieces_Now) & " separate pieces");
         end if;
      end if;
   end Held_Test;


   --  ══ 几何驾驶:腕眼里只用【彩色图 + 手的位姿读数 + 焦距】,深度通道一个字不读 ══
   --  以前"前后"那一维靠深度/看着多大/模板/单目网,180 炮没有一个稳过 10 步。现在是大拇指测距:手挪一段【读数说的】米数,
   --  看它在画面里跳多少像素,两条视线一交就是它在哪。误差随距离平方缩:远处两厘米、指尖前两毫米,正合抓取。
   procedure Geo_Say (S : String) is
   begin
      Put_Line ("[身] 📐 " & S);
   end Geo_Say;

   function Mm (X : Long_Float) return String is (Codec.Fmt (X * 1000.0, 0) & " mm");

   function Geo_Of (C : Context; Cam : Natural) return Geom.Cam_Geo is
     (if Cam < Natural (C.Geo.Length) then C.Geo (Cam) else Geom.No_Geo);

   --  这台相机的几何能不能开工:焦距 + 指尖都有(朝向没有可以现量)
   function Geo_Ready (C : Context; Cam : Natural) return Boolean is
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
   begin
      return G.Tip_Valid and then G.F > 0.0;
   end Geo_Ready;

   --  观测里带了焦距就记进这台相机的几何(没带就留着以前存的)
   procedure Geo_Take_K (C : in out Context; F : Plug.Frame; Cam : Natural) is
      G : Geom.Cam_Geo := Geo_Of (C, Cam);
   begin
      if Cam < Natural (F.Cams.Length) and then F.Cams (Cam).Has_K and then Cam < Natural (C.Geo.Length) then
         G.F := F.Cams (Cam).Focal; G.Cx := F.Cams (Cam).Cx; G.Cy := F.Cams (Cam).Cy;
         C.Geo.Replace_Element (Cam, G);
      end if;
   end Geo_Take_K;

   --  点名的那块此刻在这台相机里的像素(这一帧还没切过就切一遍、槽号对上)。
   --  世界槽的对号半径只有半个框,几何一步能让它在画面里跳几百像素 ⇒ 对不上时按【预测的像素】(有的话)或上次位置,
   --  在更大的半径里找一块大小同量级的,找到就把槽接上。
   procedure Geo_Track (C : in out Context; F : Plug.Frame; Cam : Natural; Slot : Integer; U, V : out Long_Float; Seen : out Boolean;
                        Pred_U : Long_Float := -1.0; Pred_V : Long_Float := -1.0) is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
   begin
      Seen := False; U := 0.0; V := 0.0;
      if not (C.Cut_Seq = F.Seq and then C.Cut_Cam = Integer (Cam)) then
         World.Observe (C.Wld, Cam, Cut_Things (C, F, Cam), Cw, Ch);
      end if;
      if Slot < 0 or else Natural (Slot) >= World.Count (C.Wld, Cam) then
         return;
      end if;
      declare
         Sl : World.Slot := World.Get (C.Wld, Cam, Natural (Slot));
      begin
         if Sl.Present and then Sl.Seen then
            U := Sl.R.Cu * Long_Float (Cw); V := Sl.R.Cv * Long_Float (Ch); Seen := True;
            return;
         end if;
         declare
            Ref : constant Picture.Region := (if Sl.Present then Sl.R else Sl.Shadow);
            Bw : constant Long_Float := Long_Float'Max (Long_Float (Ref.X1 - Ref.X0) / Long_Float (Cw), Long_Float (Ref.Y1 - Ref.Y0) / Long_Float (Ch));
            Has_Pred : constant Boolean := Pred_U >= 0.0 and then Pred_V >= 0.0;
            Cu0 : constant Long_Float := (if Has_Pred then Pred_U / Long_Float (Cw) else Ref.Cu);
            Cv0 : constant Long_Float := (if Has_Pred then Pred_V / Long_Float (Ch) else Ref.Cv);
            --  找回半径:有预测时三个框、没有时两个框(倍数,无量纲);面积得在上次的 0.3 到 6 倍之间(靠近时它会变大,比例,无量纲)
            Radius : constant Long_Float := (if Has_Pred then 3.0 else 2.0) * Long_Float'Max (Bw, 1.0 / Long_Float (Cw));
            Regs : constant Picture.Regions := Cut_Things (C, F, Cam);
            Best : Integer := -1;
            Bd : Long_Float := Long_Float'Last;
         begin
            for Ri in 0 .. Natural (Regs.Length) - 1 loop
               declare
                  R : constant Picture.Region := Regs (Ri);
                  Ratio : constant Long_Float := Long_Float (R.Count) / Long_Float (Natural'Max (1, Ref.Count));
                  D : constant Long_Float := Sqrt ((R.Cu - Cu0) ** 2 + (R.Cv - Cv0) ** 2);
               begin
                  --  面积比 0.3 到 6 倍(比例,无量纲)
                  if Ratio >= 0.3 and then Ratio <= 6.0 and then D <= Radius and then D < Bd then
                     Bd := D; Best := Ri;
                  end if;
               end;
            end loop;
            if Best >= 0 then
               declare
                  Cs : World.Cam_State := C.Wld.Cams (Cam);
               begin
                  Sl.Present := True; Sl.Seen := True; Sl.R := Regs (Best); Sl.Shadow := Regs (Best);
                  Cs.Slots.Replace_Element (Natural (Slot), Sl);
                  C.Wld.Cams.Replace_Element (Cam, Cs);
               end;
               U := Regs (Best).Cu * Long_Float (Cw); V := Regs (Best).Cv * Long_Float (Ch); Seen := True;
               Geo_Say ("槽对不上,按" & (if Has_Pred then "预测" else "上次位置") & "找回来了:(" & Codec.Fmt (U, 1) & "," & Codec.Fmt (V, 1) & ")," &
                        Codec.Img (Regs (Best).Count) & " px(上次 " & Codec.Img (Ref.Count) & " px)");
            end if;
         end;
      end;
   end Geo_Track;

   --  只平移(世界系),不转
   --  Jaw_Target < 0 = 抓握通道保持读数;拿着东西挪的时候必须继续给"合到底"的目标,不然驱动把目标换成当前读数 = 不再使劲,球就掉
   --  Quick = 到量出来的稳定拍数就走(小步慢抬用,省拍数);大步不许 Quick:GC3 第一截 15 cm 在稳定拍数上读到的是走了一半的位姿
   --  (命令 z −80 mm 读到 +65 mm),几何全算歪
   procedure Geo_Move (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; Dw : Geom.V3; Ok : out Boolean;
                       Jaw_Target : Long_Float := -1.0; Quick : Boolean := False) is
      A : Table.Vec := Table.Zero_Vec;
      Jaw : Floats;
      Del : Table.Vec;
   begin
      A (0) := Dw (0); A (1) := Dw (1); A (2) := Dw (2);
      if Jaw_Target >= 0.0 then
         Jaw.Append (Jaw_Target);
      end if;
      Step_Arm (L, C, F, Arm, A, Jaw, Del, Ok, Quick => Quick);
      Geo_Say ("挪 (" & Mm (Dw (0)) & "," & Mm (Dw (1)) & "," & Mm (Dw (2)) & ") ⇒ 实到 (" & Mm (Del (0)) & "," & Mm (Del (1)) & "," & Mm (Del (2)) &
               "),差 " & Mm (Geom.Norm ([Dw (0) - Del (0), Dw (1) - Del (1), Dw (2) - Del (2)])) & (if Ok then "" else " · 身体说没走成"));
   end Geo_Move;

   --  这只手一步能走出来又看得见的那一档(开机量的,米)
   function Geo_Base (C : Context; Arm : Natural) return Long_Float is
      K : constant Natural := Arm * C.Map.Per_Arm;
   begin
      if K < Natural (C.Map.Amp.Length) and then C.Map.Amp (K) > 0.0 then
         return C.Map.Amp (K);
      end if;
      return C.Map.EE_Noise;
   end Geo_Base;

   --  指尖在相机里的位置:开机那一帧里两根手指(合空扫过的像素)各自最靠上的那一截 = 指尖;有深度那一帧读一次深度
   --  (真机:一台相机一辈子量一次,用尺子也行;之后再也不读深度)
   procedure Geo_Measure_Tips (C : in out Context; F : Plug.Frame; Cam, Arm : Natural) is
      G : Geom.Cam_Geo := Geo_Of (C, Cam);
      Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Tips : array (0 .. 1) of Geom.V3 := [others => [others => 0.0]];
      Nt : Natural := 0;
      procedure One (Lb : Zone.Lobe) is
         Dmin : Long_Float := Long_Float'Last;
         Top : Integer := -1;
         Su, Sv : Long_Float := 0.0;
         Cnt : Natural := 0;
         Ds : Floats;
         --  合空扫过的像素(手指本身)装回身体文件后没有,那就只靠"这个框里最近的那一团"
         Use_Mask : constant Boolean := Natural (Z.Fingers.Length) = Cw * Ch;
         function Dep (X, Y : Natural) return Long_Float is
            I : constant Natural := Y * Cw + X;
         begin
            if (not Use_Mask or else (I < Natural (Z.Fingers.Length) and then Z.Fingers (I))) and then I < Natural (F.Cams (Cam).Depth.Length) then
               declare
                  D : constant Long_Float := F.Cams (Cam).Depth (I);
               begin
                  if D > 0.0 and then not Picture.Is_Nan (D) then
                     return D;
                  end if;
               end;
            end if;
            return -1.0;
         end Dep;
         --  手指是它自己框里离相机最近的那一团:比最近处远过一半的像素不是手指(桌面、影子;倍数,无量纲)
         function Near (X, Y : Natural) return Boolean is
            D : constant Long_Float := Dep (X, Y);
         begin
            return D > 0.0 and then D < Dmin * 1.5;
         end Near;
      begin
         if not Lb.Valid then
            return;
         end if;
         for Y in Lb.Y0 .. Lb.Y1 loop
            for X in Lb.X0 .. Lb.X1 loop
               declare
                  D : constant Long_Float := Dep (X, Y);
               begin
                  if D > 0.0 and then D < Dmin then
                     Dmin := D;
                  end if;
               end;
            end loop;
         end loop;
         if Dmin >= Long_Float'Last then
            return;
         end if;
         --  指尖 = 手指那一团最靠上的一行(一行里至少 5 个像素,免得认到孤零零的杂点;次数)
         for Y in Lb.Y0 .. Lb.Y1 loop
            declare
               Row : Natural := 0;
            begin
               for X in Lb.X0 .. Lb.X1 loop
                  if Near (X, Y) then
                     Row := Row + 1;
                  end if;
               end loop;
               if Row >= 5 then
                  Top := Y;
                  exit;
               end if;
            end;
         end loop;
         if Top < 0 then
            return;
         end if;
         --  指尖那一截 = 最靠上的 1/80 画幅高(比例,无量纲)
         for Y in Top .. Natural'Min (Lb.Y1, Top + Ch / 80) loop
            for X in Lb.X0 .. Lb.X1 loop
               if Near (X, Y) then
                  Su := Su + Long_Float (X); Sv := Sv + Long_Float (Y); Cnt := Cnt + 1;
                  Ds.Append (Dep (X, Y));
               end if;
            end loop;
         end loop;
         if Cnt = 0 then
            return;
         end if;
         --  深度取中位数(排序)
         declare
            Arr : array (0 .. Natural (Ds.Length) - 1) of Long_Float;
            U : constant Long_Float := Su / Long_Float (Cnt);
            V : constant Long_Float := Sv / Long_Float (Cnt);
            Dm : Long_Float;
         begin
            for I in Arr'Range loop
               Arr (I) := Ds (I);
            end loop;
            for I in Arr'First + 1 .. Arr'Last loop
               declare
                  Key : constant Long_Float := Arr (I);
                  J : Integer := I - 1;
               begin
                  while J >= Arr'First and then Arr (J) > Key loop
                     Arr (J + 1) := Arr (J); J := J - 1;
                  end loop;
                  Arr (J + 1) := Key;
               end;
            end loop;
            Dm := Arr (Arr'Length / 2);
            Tips (Nt) := [(U - G.Cx) / G.F * Dm, -(V - G.Cy) / G.F * Dm, -Dm];
            Nt := Nt + 1;
            Geo_Say ("指尖:像素 (" & Codec.Fmt (U, 1) & "," & Codec.Fmt (V, 1) & ") 离相机 " & Mm (Dm) & "(这根手指最近处 " & Mm (Dmin) & ")");
         end;
      end One;
   begin
      if not Z.Valid or else G.F <= 0.0 or else not F.Cams (Cam).Has_Depth then
         Geo_Say ("第" & Codec.Img (Cam) & " 台相机的指尖量不了:" &
                  (if not Z.Valid then "握区没量到" elsif G.F <= 0.0 then "没有焦距" else "这一帧没有深度(真机:用尺子量一次填进几何文件)"));
         return;
      end if;
      One (Z.A);
      if Nt < 2 then
         One (Z.B);
      end if;
      --  两根指尖离相机的远近要对得上(差不过两成,比例,无量纲),否则是认错了,不许存
      if Nt = 2 and then abs (Tips (0) (2) - Tips (1) (2)) > 0.2 * abs (Tips (0) (2) + Tips (1) (2)) / 2.0 then
         Geo_Say ("第" & Codec.Img (Cam) & " 台相机:两根指尖远近对不上(" & Mm (-Tips (0) (2)) & " vs " & Mm (-Tips (1) (2)) & ")⇒ 指尖没量到");
         Nt := 0;
      end if;
      if Nt = 2 then
         G.Tip := [(Tips (0) (0) + Tips (1) (0)) / 2.0, (Tips (0) (1) + Tips (1) (1)) / 2.0, (Tips (0) (2) + Tips (1) (2)) / 2.0];
         G.Gap := Geom.Norm ([Tips (0) (0) - Tips (1) (0), Tips (0) (1) - Tips (1) (1), Tips (0) (2) - Tips (1) (2)]);
         G.Tip_Valid := True;
         C.Geo.Replace_Element (Cam, G);
         Geo_Say ("第" & Codec.Img (Cam) & " 台相机:指尖中点在相机前 " & Mm (-G.Tip (2)) & ",两指尖相距 " & Mm (G.Gap));
      else
         Geo_Say ("第" & Codec.Img (Cam) & " 台相机:只认出 " & Codec.Img (Nt) & " 根指尖 ⇒ 指尖没量到");
      end if;
   end Geo_Measure_Tips;

   procedure Geo_Boot (F : Plug.Frame; C : in out Context; Body_Path : String) is
      Note : String (1 .. 160);
   begin
      C.Geo_Path := S (Body_Path & ".geo.json");
      Geom.Load (To_String (C.Geo_Path), C.Geo, C.Map.N_Cams, Note);
      Geo_Say (Ada.Strings.Fixed.Trim (Note, Ada.Strings.Both));
      for Cam in 0 .. C.Map.N_Cams - 1 loop
         Geo_Take_K (C, F, Cam);
         declare
            A : constant Integer := Cam_Arm (C, Cam);
            G : Geom.Cam_Geo;
         begin
            if A >= 0 and then Cam < Natural (F.Cams.Length) then
               if not Geo_Of (C, Cam).Tip_Valid then
                  Geo_Measure_Tips (C, F, Cam, Natural (A));
               end if;
               G := Geo_Of (C, Cam);
               Geo_Say ("第" & Codec.Img (Cam) & " 台相机(长在第" & Codec.Img (Natural (A) + 1) & " 只手上):焦距 " &
                        (if G.F > 0.0 then Codec.Fmt (G.F, 1) & " px" else "没有") & " · 朝向 " & (if G.Valid then "量过(残差 " & Codec.Fmt (G.Rms, 2) & " px)" else "没量,用到时现量") &
                        " · 指尖 " & (if G.Tip_Valid then "有" else "没有"));
            end if;
         end;
      end loop;
      if not C.Geo.Is_Empty then
         Geom.Save (To_String (C.Geo_Path), C.Geo);
      end if;
   end Geo_Boot;

   --  ── 三维记忆:腕眼每停一次,把看见的【每一样东西】的(位姿, 像素)记一笔;两笔以上、基线够长,就能算它在哪 ──
   procedure Geo_New_Episode (C : in out Context) is
   begin
      C.Geo_Obs.Clear; C.Geo_Slot := -1; C.Geo_Dist := -1.0; C.Geo_Came := 0.0;
      C.Geo_Slot_Obs.Clear; C.Geo_Map_Cam := -1;
      C.Blind_Mask := 0;
   end Geo_New_Episode;

   procedure Geo_Record_All (C : in out Context; F : Plug.Frame; Cam : Natural) is
      Arm : constant Integer := Cam_Arm (C, Cam);
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
   begin
      if Arm < 0 then
         return;
      end if;
      if C.Geo_Map_Cam /= Integer (Cam) then
         C.Geo_Slot_Obs.Clear; C.Geo_Map_Cam := Integer (Cam);
      end if;
      for Si in 0 .. World.Count (C.Wld, Cam) - 1 loop
         declare
            Sl : constant World.Slot := World.Get (C.Wld, Cam, Si);
         begin
            while Natural (C.Geo_Slot_Obs.Length) <= Si loop
               C.Geo_Slot_Obs.Append (Geom.Obs_Vectors.Empty_Vector);
            end loop;
            if Sl.Present and then Sl.Seen then
               declare
                  V : Geom.Obs_Vectors.Vector := C.Geo_Slot_Obs (Si);
               begin
                  V.Append (Geom.Obs'(Pose => F.EE (Natural (Arm)), U => Sl.R.Cu * Long_Float (Cw), V => Sl.R.Cv * Long_Float (Ch)));
                  while Natural (V.Length) > 12 loop   --  每样东西最多留 12 笔(次数)
                     V.Delete_First;
                  end loop;
                  C.Geo_Slot_Obs.Replace_Element (Si, V);
               end;
            end if;
         end;
      end loop;
   end Geo_Record_All;

   --  这一槽的东西在三维哪儿(世界系,相机当在手上):要两笔以上、且看它的地方相隔够远(基线 ≥ 两倍探针档,倍数无量纲)
   function Geo_Locate (C : Context; Cam : Natural; Slot : Integer; Pw : out Geom.V3) return Boolean is
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
      Arm : constant Integer := Cam_Arm (C, Cam);
   begin
      Pw := [others => 0.0];
      if Arm < 0 or else not G.Valid or else Slot < 0 or else Natural (Slot) >= Natural (C.Geo_Slot_Obs.Length) then
         return False;
      end if;
      declare
         V : constant Geom.Obs_Vectors.Vector := C.Geo_Slot_Obs (Natural (Slot));
         Base : constant Long_Float := 2.0 * Geo_Base (C, Natural (Arm));
         Far : Long_Float := 0.0;
      begin
         if Natural (V.Length) < 2 then
            return False;
         end if;
         for I in 0 .. Natural (V.Length) - 1 loop
            for J in I + 1 .. Natural (V.Length) - 1 loop
               Far := Long_Float'Max (Far, Geom.Norm ([V (I).Pose (0) - V (J).Pose (0), V (I).Pose (1) - V (J).Pose (1), V (I).Pose (2) - V (J).Pose (2)]));
            end loop;
         end loop;
         if Far < Base then
            return False;
         end if;
         Pw := Geom.Triangulate (G, V);
         declare
            Pc : constant Geom.V3 := Geom.To_Cam (G, V (Natural (V.Length) - 1).Pose, Pw);
         begin
            return -Pc (2) > 0.0;
         end;
      end;
   end Geo_Locate;

   --  到它上方(拿着东西也行):指尖该到的点 = 它的三维位置正上方两个张口(倍数,无量纲);分截走,只走平移,拿着就继续使劲
   procedure Geo_Hover (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam, Arm : Natural; Pw : Geom.V3;
                        Event : out Unbounded_String; Steps_Taken : out Natural; Beats : out Natural) is
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
      Beats0 : constant Natural := Plug.Steps (L);
      Leg_Max : constant Long_Float := 0.5 * G.Gap;   --  一截最多半个张口(比例,无量纲)
      Mok : Boolean;
      Guard : Natural := 0;
   begin
      Steps_Taken := 0; Beats := 0; Event := Null_Unbounded_String;
      loop
         declare
            Cur : constant Plug.Arm_Pose := F.EE (Arm);
            Rc : constant Geom.M3 := Geom.Cam_R (G, Cur);
            Tip_W : constant Geom.V3 := Geom.Ap (Rc, G.Tip);
            Fingers : constant Geom.V3 := [Cur (0) + Tip_W (0), Cur (1) + Tip_W (1), Cur (2) + Tip_W (2)];
            Target : constant Geom.V3 := [Pw (0), Pw (1), Pw (2) + 2.0 * G.Gap];
            D : constant Geom.V3 := [Target (0) - Fingers (0), Target (1) - Fingers (1), Target (2) - Fingers (2)];
            Dist : constant Long_Float := Geom.Norm (D);
            Tol : constant Long_Float := 0.1 * G.Gap;   --  容差 = 张口的一成(比例,无量纲)
         begin
            Geo_Say ("到它上方:它在 (" & Mm (Pw (0)) & "," & Mm (Pw (1)) & "," & Mm (Pw (2)) & "),指尖还差 " & Mm (Dist));
            exit when Dist <= Tol;
            Guard := Guard + 1;
            if Guard > 12 then   --  次数
               Event := S ("steps: I took the steps you asked for (still " & Mm (Dist) & " from above it)");
               Beats := Plug.Steps (L) - Beats0;
               return;
            end if;
            declare
               Frac : constant Long_Float := (if Dist > Leg_Max then Leg_Max / Dist else 1.0);
            begin
               Geo_Move (L, C, F, Arm, [D (0) * Frac, D (1) * Frac, D (2) * Frac], Mok, Jaw_Target => (if C.Wld.Holding then 0.0 else -1.0));
               Steps_Taken := Steps_Taken + 1;
            end;
         end;
      end loop;
      Event := S ("amount: arrived (my fingertips are above it)");
      Beats := Plug.Steps (L) - Beats0;
   end Geo_Hover;

   --  量相机朝向:盯着点名那块,手做四次平移,每次停稳记一笔,回起点,解朝向,存文件
   procedure Geo_Calibrate (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam, Arm : Natural; Slot : Integer; Ok : out Boolean) is
      G : Geom.Cam_Geo;
      Home : constant Plug.Arm_Pose := F.EE (Arm);
      B : constant Long_Float := 4.0 * Geo_Base (C, Arm);   --  四倍那一档(倍数,无量纲):远处一步要跳得过跟踪噪声
      Moves : constant array (1 .. 4) of Geom.V3 := [[B, 0.0, 0.0], [0.0, 0.0, B], [0.0, B, 0.0], [-B, 0.0, B]];
      Obs : Geom.Obs_Vectors.Vector;
      U, V : Long_Float;
      Seen, Mok : Boolean;
   begin
      Ok := False;
      Geo_Take_K (C, F, Cam);
      G := Geo_Of (C, Cam);
      if G.F <= 0.0 then
         Geo_Say ("这台相机没有焦距(观测里没带、也没量过)⇒ 量不了朝向");
         return;
      end if;
      Geo_Track (C, F, Cam, Slot, U, V, Seen);
      if not Seen then
         Geo_Say ("起点就看不见点名的那块 ⇒ 量不了朝向");
         return;
      end if;
      Obs.Append (Geom.Obs'(Pose => F.EE (Arm), U => U, V => V));
      for M of Moves loop
         declare
            Cur : constant Plug.Arm_Pose := F.EE (Arm);
            Dw : constant Geom.V3 := [Home (0) + M (0) - Cur (0), Home (1) + M (1) - Cur (1), Home (2) + M (2) - Cur (2)];
         begin
            Geo_Move (L, C, F, Arm, Dw, Mok);
            Geo_Track (C, F, Cam, Slot, U, V, Seen);
            Geo_Record_All (C, F, Cam);
            declare
               Rot : constant Long_Float := Geom.Angle_Between (Home, F.EE (Arm));
               --  身体拿转动凑平移的那一停不算:转动引起的相机位移和平移之比 > 一成就扔(比例,无量纲)
               Turned : constant Boolean := Rot > 0.1;
            begin
               if Seen and then not Turned then
                  Obs.Append (Geom.Obs'(Pose => F.EE (Arm), U => U, V => V));
               end if;
               Geo_Say ("量朝向:挪 (" & Mm (M (0)) & "," & Mm (M (1)) & "," & Mm (M (2)) & ") ⇒ " &
                        (if Seen then "它在 (" & Codec.Fmt (U, 1) & "," & Codec.Fmt (V, 1) & ")" else "没看见它") &
                        (if Turned then ",手转了 " & Codec.Fmt (Rot * 57.3, 1) & "°,这一停不算" else ""));
            end;
         end;
      end loop;
      declare
         Cur : constant Plug.Arm_Pose := F.EE (Arm);
      begin
         Geo_Move (L, C, F, Arm, [Home (0) - Cur (0), Home (1) - Cur (1), Home (2) - Cur (2)], Mok);
      end;
      Geom.Fit (G, Obs, Ok);
      if Ok then
         C.Geo.Replace_Element (Cam, G);
         Geom.Save (To_String (C.Geo_Path), C.Geo);
         Geo_Say ("相机朝向量好:" & Codec.Img (Natural (Obs.Length)) & " 停,像素残差 " & Codec.Fmt (G.Rms, 2) & " px,存进 " & To_String (C.Geo_Path));
      else
         Geo_Say ("朝向解不出来(能用的停只有 " & Codec.Img (Natural (Obs.Length)) & " 个)");
      end if;
   end Geo_Calibrate;

   --  几何逼近:让"指尖该到的那一点"(指尖中点再往手心里一点)和点名那块重合。每段走一截、停稳、再看一眼、再算。
   procedure Geo_Approach (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam, Arm : Natural; Slot : Integer;
                           Step_Limit : Natural; Event : out Unbounded_String; Steps_Taken : out Natural; Beats : out Natural) is
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
      Beats0 : constant Natural := Plug.Steps (L);
      Limit : constant Natural := (if Step_Limit > 0 then Step_Limit else 12);   --  没说步数时的安全上限(次数)
      Tol : constant Long_Float := 0.1 * G.Gap;      --  到位容差 = 张口的一成(比例,无量纲)
      --  指尖中点再往手心里 = 张口的三成(比例,无量纲):GA7 逐帧量过,放 15% 时手指只合 5 mm 就顶住 —— 夹的是球最前面那层皮,一抬就滑
      Inward : constant Long_Float := 0.3 * G.Gap;
      Want : Geom.V3 := G.Tip;
      U, V : Long_Float;
      Seen, Mok : Boolean;
      Pw_Last : Geom.V3 := [others => 0.0];
      Have_Pw : Boolean := False;
   begin
      Event := Null_Unbounded_String; Steps_Taken := 0; Beats := 0;
      Want (2) := Want (2) + Inward;   --  相机 -z 朝前 ⇒ 往手心方向 = +z
      if C.Geo_Slot /= Slot then
         C.Geo_Obs.Clear; C.Geo_Slot := Slot; C.Geo_Came := 0.0;
      end if;
      Geo_Track (C, F, Cam, Slot, U, V, Seen);
      Geo_Record_All (C, F, Cam);
      if not Seen then
         Event := S ("lost: I cannot see the thing you named in this eye right now");
         return;
      end if;
      C.Geo_Obs.Append (Geom.Obs'(Pose => F.EE (Arm), U => U, V => V));
      if Natural (C.Geo_Obs.Length) < 2 then
         --  只有一笔观测 ⇒ 先横挪一步当基线(拇指测距的"换只眼")
         declare
            Rc : constant Geom.M3 := Geom.Cam_R (G, F.EE (Arm));
            B : constant Long_Float := 4.0 * Geo_Base (C, Arm);   --  同量朝向那一档(倍数,无量纲)
            Dw : constant Geom.V3 := Geom.Ap (Rc, [B, 0.0, 0.0]);
            U0 : constant Long_Float := U;
         begin
            Geo_Move (L, C, F, Arm, Dw, Mok);
            Steps_Taken := Steps_Taken + 1;
            Geo_Track (C, F, Cam, Slot, U, V, Seen);
            Geo_Record_All (C, F, Cam);
            if not Seen then
               Event := S ("lost: it left my sight when I stepped sideways to measure its distance");
               Beats := Plug.Steps (L) - Beats0;
               return;
            end if;
            C.Geo_Obs.Append (Geom.Obs'(Pose => F.EE (Arm), U => U, V => V));
            Geo_Say ("视差基线:横挪 " & Mm (B) & ",它在画面里从 u=" & Codec.Fmt (U0, 1) & " 跳到 u=" & Codec.Fmt (U, 1));
         end;
      end if;
      loop
         declare
            Cur : constant Plug.Arm_Pose := F.EE (Arm);
            Nobs : constant Natural := Natural (C.Geo_Obs.Length);
            Use_Obs : Geom.Obs_Vectors.Vector;
            Pw, Pc, D : Geom.V3;
            Dist : Long_Float;
         begin
            --  用最近的几笔观测(它不动,我动过的地方越多交点越稳;最多 6 笔,次数)
            for K in Natural'Max (0, Nobs - 6) .. Nobs - 1 loop
               Use_Obs.Append (C.Geo_Obs (K));
            end loop;
            Pw := Geom.Triangulate (G, Use_Obs);
            Pw_Last := Pw; Have_Pw := True;
            Pc := Geom.To_Cam (G, Cur, Pw);
            D := [Pc (0) - Want (0), Pc (1) - Want (1), Pc (2) - Want (2)];
            Dist := Geom.Norm (D);
            C.Geo_Dist := Dist; C.Geo_Round := C.Round_N;
            Geo_Say ("它在相机前 " & Mm (-Pc (2)) & "(左右 " & Mm (Pc (0)) & " 上下 " & Mm (Pc (1)) & "),离指尖该到的那点还差 " & Mm (Dist) &
                     "(左右 " & Mm (D (0)) & " 上下 " & Mm (D (1)) & " 前后 " & Mm (D (2)) & ")");
            if -Pc (2) <= 0.0 then
               Event := S ("lost: my sightlines do not meet in front of me (the thing may have moved)");
               exit;
            end if;
            if Dist <= Tol then
               Event := S ("amount: arrived (the thing sits " & Mm (Dist) & " from where my fingers close)");
               exit;
            end if;
            if Steps_Taken >= Limit then
               Event := S ("steps: I took the steps you asked for (still " & Mm (Dist) & " from where my fingers close)");
               exit;
            end if;
            declare
               Frac : constant Long_Float := (if Dist > G.Gap then 0.6 else 1.0);   --  远时走六成再看一眼(比例,无量纲);近了一步到
               Step : constant Geom.V3 := [D (0) * Frac, D (1) * Frac, D (2) * Frac];
               Rc : constant Geom.M3 := Geom.Cam_R (G, Cur);
               Dw : constant Geom.V3 := Geom.Ap (Rc, Step);
               Ln : constant Long_Float := Geom.Norm (Dw);
            begin
               Geo_Move (L, C, F, Arm, Dw, Mok);
               Steps_Taken := Steps_Taken + 1;
               --  命令发出去手没跟着走(GC6 布局 4/17:要 18 cm 只动了几毫米,仿真对那个位姿解不出逆运动学就静默不动):
               --  身体唯一看得见的是"读数没变"。不到要的两成(比例,无量纲)= 那儿够不着 ⇒ 立刻把控制权交回脑、说清楚,
               --  不许再把同一截重发五次;怎么办是脑的事,身体只报事实
               declare
                  Stuck_Frac : constant Long_Float := 0.2;
                  Cur3 : constant Plug.Arm_Pose := F.EE (Arm);
                  Moved : constant Long_Float := Geom.Norm ([Cur3 (0) - Cur (0), Cur3 (1) - Cur (1), Cur3 (2) - Cur (2)]);
               begin
                  if Moved < Stuck_Frac * Ln then
                     Event := S ("stalled: my arm did not follow my own command toward it (I asked for " & Mm (Ln) & ", it moved " & Mm (Moved)
                                 & ") - that place seems out of my reach the way this hand is held; it is still " & Mm (Dist) & " from where my fingers close");
                     Geo_Say ("手没跟着走(要 " & Mm (Ln) & " 只动了 " & Mm (Moved) & ")⇒ 够不着,交回脑");
                     exit;
                  end if;
               end;
               C.Geo_Came := C.Geo_Came + Ln;
               if Ln > 0.0 then
                  C.Geo_Dir := [Dw (0) / Ln, Dw (1) / Ln, Dw (2) / Ln];
               end if;
               --  最后一截整段走完就算到:这么近它已经撑满画面、压着画面下沿,切出来的重心不再是球心,再量只会量歪
               --  (GA8:多量三次把手带到天上去了);最后几厘米靠手自己的位姿读数走,它准到毫米
               if Frac >= 1.0 then
                  declare
                     --  最后一截差得多就按读数再补(GC6 第 1 桌:命令下 21 mm 只到 12.6,差 8.8 mm 也算"到了",
                     --  合手咬在球顶,一抬就滑)。补的门槛 = 半个容差(比例,无量纲);最多补两次(次数);
                     --  一补几乎没动(不到要的两成,比例,无量纲)= 顶住了,再推也没用,停
                     Fix_Frac : constant Long_Float := 0.5;
                     Fix_Max : constant Natural := 2;
                     Stuck_Frac : constant Long_Float := 0.2;
                     Cur2 : Plug.Arm_Pose := F.EE (Arm);
                     Went : Geom.V3 := [Cur2 (0) - Cur (0), Cur2 (1) - Cur (1), Cur2 (2) - Cur (2)];
                     Rest : Geom.V3 := [Dw (0) - Went (0), Dw (1) - Went (1), Dw (2) - Went (2)];
                     Short : Long_Float := Geom.Norm (Rest);
                     Fixes : Natural := 0;
                  begin
                     while Short > Fix_Frac * Tol and then Fixes < Fix_Max loop
                        declare
                           Before : constant Plug.Arm_Pose := F.EE (Arm);
                           Moved : Long_Float;
                        begin
                           Geo_Move (L, C, F, Arm, Rest, Mok);
                           Fixes := Fixes + 1;
                           Cur2 := F.EE (Arm);
                           Moved := Geom.Norm ([Cur2 (0) - Before (0), Cur2 (1) - Before (1), Cur2 (2) - Before (2)]);
                           Went := [Cur2 (0) - Cur (0), Cur2 (1) - Cur (1), Cur2 (2) - Cur (2)];
                           Rest := [Dw (0) - Went (0), Dw (1) - Went (1), Dw (2) - Went (2)];
                           Geo_Say ("最后一截差 " & Mm (Short) & " ⇒ 按读数补第" & Codec.Img (Fixes) & " 次,动了 " & Mm (Moved) & ",还差 " & Mm (Geom.Norm (Rest)));
                           if Moved < Stuck_Frac * Short then
                              Geo_Say ("补了几乎没动 ⇒ 顶住了,不再推");
                              Short := Geom.Norm (Rest);
                              exit;
                           end if;
                           Short := Geom.Norm (Rest);
                        end;
                     end loop;
                     C.Geo_Dist := Short; C.Geo_Round := C.Round_N;
                     Event := S ("amount: arrived (I went the last " & Mm (Ln) & " by my own arm's reckoning"
                                 & (if Fixes > 0 then ", then corrected" & Natural'Image (Fixes) & " time(s) by the same reckoning" else "")
                                 & "; it fell short by " & Mm (Short) & ")");
                     exit;
                  end;
               end if;
               --  走完按几何预测它该在画面哪儿,拿预测去找
               declare
                  Pu, Pv : Long_Float := -1.0;
                  Front : Boolean;
               begin
                  if Have_Pw then
                     Geom.Project (G, F.EE (Arm), Pw_Last, Pu, Pv, Front);
                     if not Front then
                        Pu := -1.0; Pv := -1.0;
                     end if;
                  end if;
                  Geo_Track (C, F, Cam, Slot, U, V, Seen, Pu, Pv);
                  Geo_Record_All (C, F, Cam);
               end;
               if not Seen then
                  --  最后一步它进了指缝、被手指挡住也正常:上一眼已经在两倍容差内(倍数,无量纲)
                  if Dist <= 2.0 * Tol then
                     Event := S ("amount: arrived (I lost sight of it on the last step; it was " & Mm (Dist) & " from where my fingers close)");
                  else
                     Event := S ("lost: I lost sight of it after that step (it was " & Mm (Dist) & " away)");
                  end if;
                  exit;
               end if;
               C.Geo_Obs.Append (Geom.Obs'(Pose => F.EE (Arm), U => U, V => V));
            end;
         end;
      end loop;
      Beats := Plug.Steps (L) - Beats0;
   end Geo_Approach;

   --  拿住了没(几何版):眼睛长在手上 ⇒ 真拿住的东西在这只眼里【不动】;留在桌上的东西一抬手就在画面里跑掉/变小。
   --  合完沿来的路退半个张口那么远,再看它在不在原来的像素上。爪读数回到合空 = 没夹到。
   procedure Geo_Held (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm, Cam : Natural; Slot : Integer;
                       Reading_Says : Boolean; Held, Sure : out Boolean; Note : out Unbounded_String) is
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
      Cw : constant Natural := F.Cams (Cam).W;
      U0, V0, U1, V1 : Long_Float;
      Seen0, Seen1, Mok : Boolean;
      N0 : Natural := 0;
      Lift : constant Long_Float := 0.5 * G.Gap;   --  抬半个张口那么远(比例,无量纲)
   begin
      Held := False; Sure := False; Note := Null_Unbounded_String;
      if not Reading_Says then
         Note := S ("my grip closed all the way to its empty reading, so there is nothing between my fingers ⇒ not held");
         Sure := True;
         return;
      end if;
      Geo_Track (C, F, Cam, Slot, U0, V0, Seen0);
      if Seen0 and then Slot >= 0 and then Natural (Slot) < World.Count (C.Wld, Cam) then
         N0 := World.Get (C.Wld, Cam, Natural (Slot)).R.Count;
      end if;
      --  🔴 抬 = 沿位姿读数那个坐标系的第三根轴(z)直上。GA9 逐帧:沿来的路退是后上 30°,先把球在桌上往后拖 39 mm 才抬,
      --  球被搓出指缝。"哪边是上"身体现在量不出(真机由惯导报重力),这里先当读数系 z 朝上,并且说出来。
      Geo_Say ("抬 " & Mm (Lift) & ":沿位姿读数的 z 轴直上(当它朝上;真机该由重力读数定),爪子继续往合到底使劲");
      Geo_Move (L, C, F, Arm, [0.0, 0.0, Lift], Mok, Jaw_Target => 0.0);
      if not Seen0 then
         Note := S ("I could not see it in my hand camera before the lift, so I could not judge whether it came with me");
         return;
      end if;
      Geo_Track (C, F, Cam, Slot, U1, V1, Seen1, U0, V0);
      declare
         Moved : constant Long_Float := (if Seen1 then Sqrt ((U1 - U0) ** 2 + (V1 - V0) ** 2) / Long_Float (Cw) else 1.0);
         N1 : constant Natural := (if Seen1 then World.Get (C.Wld, Cam, Natural (Slot)).R.Count else 0);
         Ratio : constant Long_Float := Long_Float (N1) / Long_Float (Natural'Max (1, N0));
      begin
         Sure := True;
         --  没动 = 挪不过一成画幅、看着大小没变过一倍(比例,无量纲)
         if Seen1 and then Moved <= 0.1 and then Ratio >= 0.5 and then Ratio <= 2.0 then
            Held := True;
            Note := S ("after lifting " & Mm (Lift) & " straight up, it stayed put in my hand camera (moved " &
                       Codec.Fmt (Moved * Long_Float (Cw), 0) & " px, size x" & Codec.Fmt (Ratio, 2) & ") and my grip reads above empty ⇒ held");
         else
            Note := S ("after lifting " & Mm (Lift) & " straight up, it did " & (if Seen1 then "move in my hand camera (" & Codec.Fmt (Moved * Long_Float (Cw), 0) &
                       " px, size x" & Codec.Fmt (Ratio, 2) & ")" else "leave my hand camera") & " ⇒ it did NOT come with my hand");
         end if;
      end;
   end Geo_Held;


   --  离远点(拿着东西):沿来的路退,退它来时那么远(全是量的,两段走)
   procedure Geo_Retreat (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm, Cam : Natural;
                          Event : out Unbounded_String; Steps_Taken : out Natural; Beats : out Natural) is
      Beats0 : constant Natural := Plug.Steps (L);
      G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
      --  一截 = 张口的一成(比例,无量纲),共 20 截 = 两个张口高(次数)。GB3:半个张口一截抬得快,球在指间一点点往下滑
      --  (仿真接触解算速度迭代 0 次),第二截就掉;小步慢抬
      Leg : constant Long_Float := 0.1 * G.Gap;
      Legs : constant Natural := 12;   --  12 截 = 1.2 个张口 ≈ 11 cm,加上判拿住那半个张口,离桌 > 10 cm 够了(次数)
      Mok : Boolean;
      U, V : Long_Float;
      Seen : Boolean;
      Up : Long_Float := 0.0;
      Slot : constant Integer := (if C.Wld.Holding then C.Wld.Held_Slot else -1);
      U0, V0 : Long_Float := -1.0;
   begin
      Steps_Taken := 0; Beats := 0;
      if Leg <= 0.0 then
         Event := S ("amount: stopped (I do not know my own gap, so I do not know how far to lift)");
         return;
      end if;
      if Slot >= 0 then
         Geo_Track (C, F, Cam, Slot, U0, V0, Seen);
         if not Seen then
            U0 := -1.0; V0 := -1.0;
         end if;
      end if;
      Geo_Say ("离远 = 直上 " & Codec.Img (Legs) & " 截,每截 " & Mm (Leg) & "(沿位姿读数的 z 轴,当它朝上;拿着就继续使劲,每截看一眼它还在不在手里)");
      for K in 1 .. Legs loop
         Geo_Move (L, C, F, Arm, [0.0, 0.0, Leg], Mok, Jaw_Target => (if C.Wld.Holding then 0.0 else -1.0), Quick => True);
         Steps_Taken := Steps_Taken + 1;
         Up := Up + Leg;
         if C.Wld.Holding and then Slot >= 0 and then U0 >= 0.0 then
            Geo_Track (C, F, Cam, Slot, U, V, Seen, U0, V0);
            --  拿住的东西在腕眼里不该动:挪过一成画幅(比例,无量纲)就是掉了
            if not Seen or else Sqrt ((U - U0) ** 2 + (V - V0) ** 2) / Long_Float (F.Cams (Cam).W) > 0.1 then
               Event := S ("slip: what I was holding has left my fingers on the way up (after " & Mm (Up) & ")");
               C.Wld.Holding := False; C.Wld.Held_Arm := -1; C.Wld.Held_Slot := -1;
               Memory.Set (C.Mem, "holding", "");
               Beats := Plug.Steps (L) - Beats0;
               return;
            end if;
         end if;
      end loop;
      Event := S ("amount: arrived (I lifted straight up " & Mm (Up) & (if C.Wld.Holding then ", still holding it" else "") & ")");
      Beats := Plug.Steps (L) - Beats0;
   end Geo_Retreat;

   function Mode_Line (C : Context; Until_Text : String) return String is
     ("MODE: " & (if C.Wld.Holding then "holding something with arm " & Codec.Img (Natural (C.Wld.Held_Arm) + 1) else "hands empty") &
      "; without new words from you I hold still and keep my grip as it is; this segment ended on: " & Until_Text & ".");

   --  ── Sinew 接缝:脑的话 → FO 执行核的一轮;段末事件 → 结局词。每一张表都只此一处,自检逐词钉死 ──
   function Role_Wants (R : Sinew.Role; K : Item_Kind) return Boolean is
     (case R is
         --  grasper = 我量到能相向靠拢、中间扫出一片能装东西的那一组
         when Sinew.Rl_Grasper => K = Grip,
         --  pusher = 推得动东西、但【合不拢】的部件;合得拢的一律不算
         when Sinew.Rl_Pusher => K = Piece,
         --  me = 整个我,只有分不出零件的机体才有它;这具身体量得出手指和爪心 ⇒ me 绑不上
         when others => False);

   function Rel_Cmd (R : Sinew.Rel) return String is
     (case R is
         when Sinew.Re_Touching => "at",   when Sinew.Re_Above => "above", when Sinew.Re_Below => "below",
         when Sinew.Re_Left => "left",     when Sinew.Re_Right => "right",
         when Sinew.Re_Nearer => "front",  when Sinew.Re_Farther => "back",
         when Sinew.Re_Onto => "onto",     when Sinew.Re_Off => "off", when Sinew.Re_Into => "into",
         when Sinew.Re_Facing => "face",   when Sinew.Re_Press => "press",
         when others => "?");   --  close / open / clear / still 各有各的分支
   function Rel_Has_Own_Branch (R : Sinew.Rel) return Boolean is
     (case R is when Sinew.Re_Close | Sinew.Re_Open | Sinew.Re_Clear | Sinew.Re_Still => True,
                when others => False);

   --  结局词 → 判法。free 只在合手那一节有意义(合完抬一截由 Held_Test 判),它前面的靠近段按步数走
   function Until_Word (O : Sinew.Outcome) return String is
     (case O is
         when Sinew.Oc_Touched => "contact",
         when Sinew.Oc_Stuck   => "resist",
         when Sinew.Oc_Slipped => "slip",
         when Sinew.Oc_Settled => "settle",
         when Sinew.Oc_Stalled => "stall",
         when others           => "steps");
   function Kind_Of_Word (W : String) return Monitor.Until_Kind is
     (if W = "contact" then Monitor.U_Contact
      elsif W = "resist" then Monitor.U_Resist
      elsif W = "slip" then Monitor.U_Slip
      elsif W = "settle" then Monitor.U_Settle
      elsif W = "stall" then Monitor.U_Stall
      else Monitor.U_Steps);
   function Until_Of (O : Sinew.Outcome) return Monitor.Until_Kind is
     (case O is
         when Sinew.Oc_Touched => Monitor.U_Contact,
         when Sinew.Oc_Stuck   => Monitor.U_Resist,
         when Sinew.Oc_Slipped => Monitor.U_Slip,
         when Sinew.Oc_Settled => Monitor.U_Settle,
         when Sinew.Oc_Stalled => Monitor.U_Stall,
         when others           => Monitor.U_Steps);

   --  身体报的那句事件 + 合手那句话 ⇒ 结局词。合手的话优先:合完抬一截它跟着我走了 = free(它离开了那个面);
   --  合了没拿住 = slipped;没敢合(没笼住)= stalled。顺序要紧:"settle: the picture stopped changing" 含 stopped,
   --  "lost sight: two steps" 含 steps —— 先查专名,再查兜底词。
   function Classify (Event, Grip_Note : String) return Sinew.Outcome is
      function Has (S, P : String) return Boolean is (Ada.Strings.Fixed.Index (S, P) > 0);
      function Starts (S, P : String) return Boolean is
        (S'Length >= P'Length and then S (S'First .. S'First + P'Length - 1) = P);
   begin
      if Has (Grip_Note, "⇒ held") then
         return Sinew.Oc_Free;
      elsif Has (Grip_Note, "NOT come with my hand") or else Has (Grip_Note, "not held")
        or else Has (Grip_Note, "could not judge") or else Has (Grip_Note, "not sure")
        or else Has (Grip_Note, "opened it again")
      then
         return Sinew.Oc_Slipped;
      elsif Has (Grip_Note, "did NOT close") then
         return Sinew.Oc_Stalled;
      elsif Starts (Event, "amount: arrived") or else Starts (Event, "amount: already there") then
         return Sinew.Oc_Arrived;
      elsif Starts (Event, "contact") then
         return Sinew.Oc_Touched;
      elsif Starts (Event, "resist") or else Has (Event, "the body refused") then
         return Sinew.Oc_Stuck;
      elsif Starts (Event, "slip") then
         return Sinew.Oc_Slipped;
      elsif Starts (Event, "settle") then
         return Sinew.Oc_Settled;
      elsif Starts (Event, "lost") then
         return Sinew.Oc_Lost;
      elsif Starts (Event, "amount: stopped") or else Starts (Event, "stopped:") or else Has (Event, "could not solve")
        or else Has (Event, "stopped answering")
      then
         return Sinew.Oc_Stalled;
      elsif Starts (Event, "steps") or else Has (Event, "safety cap") then
         return Sinew.Oc_Timeout;
      elsif Event = "" then
         return Sinew.Oc_Arrived;   --  这一节没有要走的段(只合/只张),而合手那句话没说失败
      end if;
      return Sinew.Oc_Refused;
   end Classify;

   --  编译器要知道的、关于每个名词的事实。编号和给脑看的清单一致(1 起),0 号空着。
   function Build_Facts (C : Context; Cw, Ch : Natural) return Plan.Facts_Vectors.Vector is
      Fs : Plan.Facts_Vectors.Vector;
      Zero : Plan.Item_Facts;
   begin
      Fs.Append (Zero);
      for I in 0 .. Natural (C.Items.Length) - 1 loop
         declare
            It : constant Item := C.Items (I);
            Ft : Plan.Item_Facts;
         begin
            Ft.Exists := It.Located or else It.Kind in Finger | Grip | Piece;
            Ft.Mine := It.Kind in Finger | Grip | Piece;
            Ft.Grasp := It.Kind = Grip;
            Ft.Arm := It.Arm;
            --  量得出它鼓出它站的那个面多少 ⇒ 才有"那个面"可言
            Ft.Stands := It.Height > 0.0;
            Ft.Span := (if It.Kind = Grip then Zone_Of (C, It.Arm, C.Cam).Span else 0.0);
            Ft.Size := Long_Float'Max (Long_Float (It.X1 - It.X0) / Long_Float (Natural'Max (1, Cw)),
                                       Long_Float (It.Y1 - It.Y0) / Long_Float (Natural'Max (1, Ch)));
            Ft.Label := To_Unbounded_String
              ((case It.Kind is
                   when Grip => "grasper(第" & Codec.Img (It.Arm + 1) & " 只手)",
                   when Finger => "grasper 的一瓣",
                   when Piece => "第" & Codec.Img (It.Arm + 1) & " 只手" & Codec.Img (It.Which) & " 轴带的那一块",
                   when others => ""));
            Fs.Append (Ft);
         end;
      end loop;
      return Fs;
   end Build_Facts;

   --  把 Sinew 的一段区间落成 FO 执行核那一轮的命令。角色在这儿变成具体的那一块。
   procedure Fill_Say (C : in out Context; I : Sinew.Instr; Answer : out Brain.Say) is
      use Sinew;
      function Step_Word (Sp : Step) return String is
        (case Sp is when Sp_Small => "small", when Sp_Medium => "medium",
            when Sp_Large => "large", when Sp_None => "medium");
      function Item_Of (N : Noun) return Natural is
         A : constant Integer := Plan.Look_Up (C.Binds, N);
      begin
         return (if A > 0 then Natural (A) else 0);
      end Item_Of;
   begin
      Answer := (others => <>);
      Answer.See := To_Unbounded_String ("target");
      Answer.Grip := To_Unbounded_String ("none");
      Answer.Fast := False;
      Answer.Until_Kind := To_Unbounded_String (Until_Word (I.Until_Oc));
      Answer.Steps := I.Max_Steps;
      for K in 0 .. Natural (I.Cons.Length) - 1 loop
         declare
            Cn : constant Constraint := I.Cons (K);
            Sub : constant Natural := Item_Of (Cn.Subj);
            Obj : constant Natural := Item_Of (Cn.Obj);
         begin
            case Cn.R is
               when Re_Close =>
                  Answer.Grip := To_Unbounded_String ("close");
                  Answer.Grip_Arm := (if Sub >= 1 and then Sub <= Natural (C.Items.Length)
                                      then C.Items (Sub - 1).Arm + 1 else 1);
                  Answer.Grip_On := Obj;
               when Re_Open =>
                  Answer.Grip := To_Unbounded_String ("open");
                  Answer.Grip_Arm := (if Sub >= 1 and then Sub <= Natural (C.Items.Length)
                                      then C.Items (Sub - 1).Arm + 1 else 1);
               when Re_Clear =>
                  Answer.Avoid.Append (Integer (Obj));
               when Re_Still =>
                  Answer.Moves.Append (Brain.Goal'(Item => Sub, Cell => 0, Rel => Null_Unbounded_String,
                                                   Of_Item => 0, Amount => Null_Unbounded_String, Stay => True));
               when others =>
                  Answer.Moves.Append
                    (Brain.Goal'(Item => Sub, Cell => 0, Rel => To_Unbounded_String (Rel_Cmd (Cn.R)),
                                 Of_Item => Obj, Amount => To_Unbounded_String (Step_Word (Cn.Sp)), Stay => False));
            end case;
         end;
      end loop;
      C.Eye_Want := I.Eye;
   end Fill_Say;

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

      --  脑交的是一段程序:没在跑的程序 ⇒ 问脑 → 解析 → 名词落到块上 → 检查 → 空转 → 存起来;
      --  在跑的 ⇒ 往前走到下一条要真动身体的 do,落成这一轮的命令。Stop = 这一轮到此为止,什么都不动。
      procedure Program_Step (Big : Buf; Bh : Natural; Recent : String; Answer : out Brain.Say; Stop : out Boolean) is
         use type Sinew.Eye_Pick;
         use type Sinew.Op;
         use type Sinew.Noun_Kind;
         use type Sinew.Role;
      begin
         Answer := (others => <>);
         Answer.Grip := S ("none");
         Answer.See := S ("target");
         Stop := False;
         if not C.Have_Prog then
            declare
               Text, E2 : Unbounded_String;
            begin
               if not Brain.Ask_Prog (To_String (C.Eye_Host), C.Eye_Port, To_String (C.Task_Text), To_String (Listing), Recent,
                                      Sinew.Grammar, To_String (C.Refused), C.Cols, C.Rows, Big, Cw, Bh, Text, E2)
               then
                  Put_Line ("[身] 🧠 问不通(" & To_String (E2) & ")⇒ 这一拍不动,下一拍重问");
                  Stop := True;
                  return;
               end if;
               Put_Line ("[身] 🧠 它交上来一段程序:");
               Put_Line (To_String (Text));
               declare
                  P : constant Sinew.Program := Sinew.Parse (To_String (Text));
                  Facts : constant Plan.Facts_Vectors.Vector := Build_Facts (C, Cw, Ch);
                  Binds : Plan.Bind_Vectors.Vector;
                  V : Plan.Verdict;
                  Grasp_Arm : Integer := -1;
                  Own_Eye : Boolean := False;
                  Missing_Here : Boolean := False;   --  这一轮脑对某个名字答了"这只眼里没有它"

                  function First_Eye return Sinew.Eye_Pick is
                  begin
                     for K in 0 .. Natural (P.Code.Length) - 1 loop
                        if P.Code (K).O = Sinew.Op_Interval and then P.Code (K).Eye /= Sinew.Ey_None then
                           return P.Code (K).Eye;
                        end if;
                     end loop;
                     return Sinew.Ey_None;
                  end First_Eye;

                  --  角色靠量出来的东西绑定。挑哪只手不许用"这张画面里离它最近"去判别的胳膊:
                  --  这只眼睛长在哪条胳膊上就用那条;不长在任何胳膊上(看得见全场)才比远近。
                  function Bind_Role (R : Sinew.Role; Near_U, Near_V : Long_Float; Has_Near : Boolean) return Integer is
                     Own : constant Integer := Cam_Arm (C, Cam);
                     Best : Integer := -1;
                     Bd : Long_Float := 1.0e9;
                  begin
                     for K in 0 .. Natural (C.Items.Length) - 1 loop
                        declare
                           It : constant Item := C.Items (K);
                           D : constant Long_Float :=
                             (if Has_Near and then It.Located then Sqrt ((It.Cu - Near_U) ** 2 + (It.Cv - Near_V) ** 2) else 0.0);
                        begin
                           if Role_Wants (R, It.Kind) and then It.Located
                             and then (Own < 0 or else Integer (It.Arm) = Own) and then D < Bd
                           then
                              Bd := D; Best := Integer (K) + 1;
                           end if;
                        end;
                     end loop;
                     return Best;
                  end Bind_Role;

                  --  名字靠身体自己去认:画面已经切成带编号的块,只让脑在这些块里挑一个。编号从头到尾没进语言。
                  function Bind_Name (W : String; Tried : out Unbounded_String) return Integer is
                     Which : Natural := 0;
                     E3 : Unbounded_String;
                  begin
                     Tried := Null_Unbounded_String;
                     if Brain.Find (To_String (C.Eye_Host), C.Eye_Port, W, To_String (Listing),
                                    Natural (C.Items.Length), Big, Cw, Bh, Which, E3)
                     then
                        if Which >= 1 and then Which <= Natural (C.Items.Length) then
                           if C.Blind_Cam = Integer (Cam) then
                              C.Blind_Cam := -1;
                           end if;
                           C.Blind_Mask := 0;
                           C.Name_Cam := Integer (Cam);
                           return Integer (Which);
                        end if;
                        C.Blind_Cam := Integer (Cam);
                        if (C.Blind_Mask / 2 ** Cam) mod 2 = 0 then
                           C.Blind_Mask := C.Blind_Mask + 2 ** Cam;
                        end if;
                        Missing_Here := True;
                        Tried := S ("我把看得见的每一块都过了一遍,没有一块是它");
                        return -1;
                     end if;
                     Tried := S ("我问自己的眼睛时没问通(" & To_String (E3) & ")");
                     return -1;
                  end Bind_Name;

                  procedure Bind_All is
                     Nu, Nv : Long_Float := 0.0;
                     Has_Near : Boolean := False;
                     procedure One (N : Sinew.Noun) is
                        Key : constant String := Plan.Key_Of (N);
                        E : Plan.Bind_Entry;
                     begin
                        if N.K /= Sinew.Nk_Thing or else Key = "" then
                           return;
                        end if;
                        for I2 in 0 .. Natural (Binds.Length) - 1 loop
                           if To_String (Binds (I2).Key) = Key then
                              return;
                           end if;
                        end loop;
                        E.Key := S (Key);
                        E.Item := Bind_Name (Key, E.Tried);
                        Binds.Append (E);
                        if E.Item >= 1 and then E.Item <= Integer (C.Items.Length)
                          and then C.Items (Natural (E.Item) - 1).Located and then not Has_Near
                        then
                           Nu := C.Items (Natural (E.Item) - 1).Cu;
                           Nv := C.Items (Natural (E.Item) - 1).Cv;
                           Has_Near := True;
                        end if;
                     end One;
                  begin
                     for Ix in 0 .. Natural (P.Code.Length) - 1 loop
                        if P.Code (Ix).O = Sinew.Op_Interval then
                           for Ci in 0 .. Natural (P.Code (Ix).Cons.Length) - 1 loop
                              One (P.Code (Ix).Cons (Ci).Subj);
                              One (P.Code (Ix).Cons (Ci).Obj);
                           end loop;
                        end if;
                     end loop;
                     for R in Sinew.Role loop
                        if R /= Sinew.Rl_None then
                           declare
                              E : Plan.Bind_Entry;
                           begin
                              E.Key := S (Sinew.Role_Word (R));
                              E.Item := Bind_Role (R, Nu, Nv, Has_Near);
                              if R = Sinew.Rl_Grasper and then E.Item > 0 and then E.Item <= Integer (C.Items.Length) then
                                 Grasp_Arm := Integer (C.Items (Natural (E.Item) - 1).Arm);
                              end if;
                              if E.Item <= 0 then
                                 E.Tried := S ("这只眼睛里没有一块符合它");
                              end if;
                              Binds.Append (E);
                           end;
                        end if;
                     end loop;
                  end Bind_All;
               begin
                  if not P.Ok then
                     C.Refused := S ("line " & Codec.Img (P.Err_Line) & ": " & To_String (P.Err) & "  -> 文法我每一轮都给你,照着改一行就行");
                     C.Recent := S ("I refused your program before anything moved. " & To_String (C.Refused)
                                    & " Nothing has moved. " & Mode_Line (C, "refused"));
                     Put_Line ("[身] ⚖ 说不出口:" & To_String (C.Refused));
                     Stop := True;
                     return;
                  end if;
                  C.Eye_Want := First_Eye;
                  Bind_All;
                  for I2 in 0 .. Natural (Binds.Length) - 1 loop
                     Put_Line ("[身] 🔎 " & To_String (Binds (I2).Key) & " ⇒ "
                               & (if Binds (I2).Item > 0 then "第" & Codec.Img (Natural (Binds (I2).Item)) & " 块"
                                  else "绑不上:" & To_String (Binds (I2).Tried)));
                  end loop;
                  --  🔴 这只眼里认不到它(脑答"没有")⇒ 身体自己换到一只还没问过的、长在胳膊上的眼,让脑在那只眼里再认一遍
                  --  (GC6 布局 41:球贴在右爪旁,头顶眼被爪子挡住,只有右腕眼看得见;语言里没有"用右手",只能身体自己换眼)。
                  --  每只眼只问一次(按位记),都问过还没有才照实拒绝。
                  if Missing_Here then
                     declare
                        Next_Eye : Integer := -1;
                     begin
                        for K in 0 .. C.Map.N_Cams - 1 loop
                           if K /= Cam and then Cam_Arm (C, K) >= 0 and then (C.Blind_Mask / 2 ** K) mod 2 = 0 and then Next_Eye < 0 then
                              Next_Eye := Integer (K);
                           end if;
                        end loop;
                        if Next_Eye >= 0 then
                           Put_Line ("[身] 👁 这只眼里没有它 ⇒ 换到第" & Codec.Img (Next_Eye) & " 台相机(长在第"
                                     & Codec.Img (Cam_Arm (C, Natural (Next_Eye)) + 1) & " 只手上)再认一遍,这一轮不动");
                           C.Cam := Natural (Next_Eye);
                           C.Recent := S ("Not in that eye. I moved to the eye that rides on my arm " & Codec.Img (Cam_Arm (C, Natural (Next_Eye)) + 1)
                                          & " - the numbers you see now belong to that eye; say the same thing again. "
                                          & Mode_Line (C, "moved to my other eye to look for it"));
                           Stop := True;
                           return;
                        end if;
                     end;
                  end if;
                  --  🔴 脑点了眼睛就换过去,换完这一轮不动,让它再说一遍(编号是按这只眼列的,换眼要重新列、重新认)。
                  --  still = 变得最少的那只 = 世界眼;moving = 长在 grasper 那条胳膊上的那只。量不出就照实说,不瞎挑。
                  if C.Eye_Want /= Sinew.Ey_None then
                     declare
                        Pick : Integer := -1;
                     begin
                        if C.Eye_Want = Sinew.Ey_Still then
                           Pick := Integer (C.Map.World_Cam);
                        elsif Grasp_Arm >= 0 and then Natural (Grasp_Arm) < Natural (C.Map.Cam_On_Arm.Length) then
                           Pick := C.Map.Cam_On_Arm (Natural (Grasp_Arm));
                        end if;
                        if Pick < 0 then
                           Put_Line ("[身] 👁 你点名要跟着我动的那只眼,可我没量到有哪只眼长在这条胳膊上 ⇒ 留在这只眼里,照实说");
                           Append (C.Prog_Log, "you asked for my moving eye but I have not measured an eye that rides on that arm, so I stayed in this eye. ");
                        elsif Natural (Pick) /= Cam then
                           Put_Line ("[身] 👁 你点名要" & (if C.Eye_Want = Sinew.Ey_Still then "不跟着我动" else "跟着我动")
                                     & "的那只眼睛 ⇒ 第" & Codec.Img (Natural (Pick)) & " 只 ⇒ 换过去,这一轮不动,你再说一遍");
                           C.Cam := Natural (Pick);
                           C.Recent := S ("I moved to the eye you asked for. Nothing moved. The numbers you see now belong to that eye - say the same thing again. "
                                          & Mode_Line (C, "moved to the eye you named"));
                           Stop := True;
                           return;
                        end if;
                     end;
                  end if;
                  Own_Eye := Grasp_Arm >= 0 and then Cam_Arm (C, Cam) = Grasp_Arm;
                  V := Plan.Check (P, Facts, Binds, Own_Eye);
                  if V.Ok then
                     V := Plan.Dry_Run (P, Facts, Binds);
                     if V.Ok then
                        Put_Line ("[身] ⚖ 编译过了,空转也走得通");
                     else
                        Put_Line ("[身] ⚖ 空转就走不通:" & Plan.Say (V));
                     end if;
                  else
                     Put_Line ("[身] ⚖ " & Plan.Say (V));
                  end if;
                  if not V.Ok then
                     C.Refused := S ("line " & Codec.Img (V.Err_Line) & ": " & To_String (V.Err)
                                     & (if Length (V.Instead) > 0 then "  -> " & To_String (V.Instead) else ""));
                     C.Recent := S ("I refused your program before anything moved. " & To_String (C.Refused)
                                    & " Nothing has moved. " & Mode_Line (C, "refused"));
                     Stop := True;
                     return;
                  end if;
                  C.Refused := Null_Unbounded_String;
                  C.Prog := P;
                  C.Binds := Binds;
                  C.M := (others => <>);
                  C.Have_Prog := True;
                  C.Prog_Log := Null_Unbounded_String;
               end;
            end;
         end if;
         --  往前走到下一条要真动身体的指令。说人话在这儿就地办掉。
         declare
            What : Runtime.Yield;
            Ins : Sinew.Instr;
            Guard : Natural := 0;
         begin
            loop
               Guard := Guard + 1;
               exit when Guard > 64;
               Runtime.Advance (C.Prog, C.M, What, Ins);
               case What is
                  when Runtime.Y_Say =>
                     Put_Line ("[身] 🧠 它说:" & To_String (Ins.Text));
                  when Runtime.Y_Remember =>
                     Put_Line ("[身] 📍 这版记不住地方,跳过这一行(编译期本该拦下)");
                  when Runtime.Y_Done =>
                     Answer.Done := True;
                     exit;
                  when Runtime.Y_Finished =>
                     C.Have_Prog := False;
                     C.Recent := S (To_String (C.Prog_Log) & " That was the whole program. " & Mode_Line (C, "program finished"));
                     Put_Line ("[身] ■ 程序跑完了");
                     Stop := True;
                     return;
                  when Runtime.Y_Broken =>
                     C.Have_Prog := False;
                     C.Refused := S (Runtime.Broken_Why (C.M));
                     C.Recent := S (To_String (C.Prog_Log) & " " & Runtime.Broken_Why (C.M) & " " & Mode_Line (C, "program broke"));
                     Put_Line ("[身] ■ 程序坏了:" & Runtime.Broken_Why (C.M));
                     Stop := True;
                     return;
                  when Runtime.Y_Interval =>
                     Fill_Say (C, Ins, Answer);
                     Put_Line ("[身] ▶ " & Sinew.Unparse (Ins));
                     exit;
               end case;
            end loop;
         end;
      end Program_Step;
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
         --  🔴 每一轮把【所有相机】一起给脑:编号那张在上面(格子和编号只管它),其余几只眼睛按半幅
         --  拼在下面一条。以前一轮只给一台,想看别的得先说"下一轮换一台" —— 那是【盲切】:说完看不到
         --  结果,切过去才发现手腕正对着墙(DS/EI/EA 各记过一次)。而且切一台就丢一轮,跨相机的判断
         --  根本做不了。下面那一条不画格子、不编号 —— 它只是"我另外几只眼睛现在看见什么"。
         declare
            Sh : constant Natural := Ch / 2;
            Sw : constant Natural := Cw / 2;
            Bh : constant Natural := Ch + (if C.Map.N_Cams > 1 then Sh else 0);
            Big : Buf := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (Cw * Bh * 3));
            Slot : Natural := 0;
         begin
            for I in 0 .. Cw * Ch * 3 - 1 loop
               Big.Replace_Element (I, RGB.Element (I));
            end loop;
            if C.Map.N_Cams > 1 then
               for K in 0 .. Natural (F.Cams.Length) - 1 loop
                  if K /= Cam and then Slot * Sw < Cw then
                     declare
                        Kw : constant Natural := F.Cams (K).W;
                        Kh : constant Natural := F.Cams (K).H;
                        Ox : constant Natural := Slot * Sw;
                     begin
                        if Natural (F.Cams (K).RGB.Length) >= Kw * Kh * 3 then
                           for Y in 0 .. Sh - 1 loop
                              for X in 0 .. Sw - 1 loop
                                 declare
                                    Sx : constant Natural := Natural'Min (Kw - 1, X * Kw / Sw);
                                    Sy : constant Natural := Natural'Min (Kh - 1, Y * Kh / Sh);
                                    D : constant Natural := ((Ch + Y) * Cw + Ox + X) * 3;
                                    Sp : constant Natural := (Sy * Kw + Sx) * 3;
                                 begin
                                    if Ox + X < Cw then
                                       Big.Replace_Element (D, F.Cams (K).RGB.Element (Sp));
                                       Big.Replace_Element (D + 1, F.Cams (K).RGB.Element (Sp + 1));
                                       Big.Replace_Element (D + 2, F.Cams (K).RGB.Element (Sp + 2));
                                    end if;
                                 end;
                              end loop;
                           end loop;
                           Draw.Numbered_Box (Big, Cw, Bh, Ox, Ch, Natural'Min (Cw - 1, Ox + Sw - 1), Bh - 1,
                                              K + 1, Draw.White, 2);
                        end if;
                        Slot := Slot + 1;
                     end;
                  end if;
               end loop;
            end if;
         if C.Use_Json then
            if not Brain.Ask (To_String (C.Eye_Host), C.Eye_Port, To_String (C.Task_Text), To_String (Listing), Recent,
                              C.Cols, C.Rows, Natural (C.Items.Length), C.Map.N_Cams, C.Map.Arms, Big, Cw, Bh, Say, Err)
            then
               Put_Line ("[身] 🧠 问不通(" & To_String (Err) & ")⇒ 这一拍不动,下一拍重问");
               return;
            end if;
         else
            declare
               Stop : Boolean;
            begin
               Program_Step (Big, Bh, Recent, Say, Stop);
               if Stop then
                  return;
               end if;
            end;
         end if;
         end;
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
         --  脑说 done = 这一集到此为止:交一个空动作,对方结束这一集(不然剩下的几百拍全是"还要我做什么")
         Plug.End_Episode (L);
         C.Recent := S ("you said it is done. I ended this episode. " & Mode_Line (C, "you said done"));
         Put_Line ("[身] ■ 脑说 done ⇒ 交空动作,这一集到此为止");
         return;
      end if;
      if C.Look_Only then
         C.Recent := S ("(look only) I did not move. " & Mode_Line (C, "look only"));
         if not C.Use_Json and then C.Have_Prog then
            --  只看不动:这一节当"步数用完"喂回去,程序照样往前走,不然它会卡在同一节上永远不再问脑
            Runtime.Report (C.Prog, C.M, Sinew.Oc_Timeout);
            Append (C.Prog_Log, (if Length (C.Prog_Log) > 0 then ASCII.LF & "" else "") & "[look only] this do-line was not run.");
         end if;
         return;
      end if;
      --  ── 执行 ──
      declare
         Until_K : constant Monitor.Until_Kind :=
           (if Say.Until_Kind = "contact" then Monitor.U_Contact elsif Say.Until_Kind = "resist" then Monitor.U_Resist
            elsif Say.Until_Kind = "slip" then Monitor.U_Slip elsif Say.Until_Kind = "settle" then Monitor.U_Settle else Monitor.U_Steps);
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
         Geo_Case : Natural := 0;            --  0 = 老路;1 = 几何贴近;2 = 直上离远;3 = 合(不先走);4 = 到它上方
         Geo_Slot_Now : Integer := -1;
         Geo_Desc : Unbounded_String;
         Geo_Pw : Geom.V3 := [others => 0.0];
      --  2a 把脑说的话变成要求:别动的,目标就是它现在的位置;要动的,目标是格子或与某号的关系;
      --  抓某号,目标是"和我张开的那片地方重合"(位置 / 远近 / 看着多大 / 朝向)
      --  把去哪翻成目标:格子 / 与某号的关系(碰到它 · 上下左右 · 前后 · 离远点)。全是量出来的位置,没有写死的距离
      procedure Set_Target (G : Brain.Goal; P : in out Point; Ok_Pt : in out Boolean) is
      begin
                           --  目标
                           if G.Cell >= 1 and then G.Cell <= Natural (C.Cells_U.Length) then
                              P.Tu := C.Cells_U (G.Cell - 1); P.Tv := C.Cells_V (G.Cell - 1); P.Tz := P.Z; P.Wz := 0.0;
                              P.Desc := S ("item " & Codec.Img (G.Item) & " to cell " & Codec.Img (G.Cell));
                           elsif P.To_Grip and then G.Rel /= "" and then G.Of_Item >= 1 and then G.Of_Item <= Natural (C.Items.Length) then
                              --  🔴 自己手上相机里"我的手到 X":手在这只眼里永远不动,能动的是 X 的像素 ⇒ 目标 = 让 X 来到握区。
                              --  以前这一支把目标算成 X 自己的位置(误差恒为 0,一开始就"满足",推 80 步报两次到位 —— a7b3fda)。
                              --  上下左右按"X 该出现在握区的哪一边"翻过来写;远近:贴上 = X 的皮到指头那么深,瞄进 = 再深半个鼓高。
                              declare
                                 O : constant Item := C.Items (G.Of_Item - 1);
                                 Ow : constant Long_Float := Long_Float (O.X1 - O.X0) / Long_Float (Cw);
                                 Oh : constant Long_Float := Long_Float (O.Y1 - O.Y0) / Long_Float (Ch);
                                 Rl : constant String := To_String (G.Rel);
                                 Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
                              begin
                                 if not O.Located or else not Z.Valid then
                                    Report := S ("goal: item " & Codec.Img (G.Of_Item) & " is not locatable right now. ");
                                    Ok_Pt := False;
                                 else
                                    P.Desc := S ("item " & Codec.Img (G.Of_Item) & " into my grip (" & Rl & ", in my own hand camera)");
                                    P.Tu := Z.Cu; P.Tv := Z.Cv;
                                    --  🔴 这只眼里远近读数不可用(见 Cut_Bright 头注)⇒ 远近那一行关掉,靠【看着多大】往下走:
                                    --  目标大小 = 我张开的那片地方有多大(量的)。它比任何真到指尖的东西都大 ⇒ 这一行永远在说"再近一点",
                                    --  于是手一路朝它降,降到顶住(until stuck)为止 —— 高低由碰到来收口,不由读数。
                                    P.Tz := P.Z; P.Wz := 0.0;
                                    P.Tsize := Sqrt (Long_Float'Max (0.0, (Long_Float (Z.X1 - Z.X0) / Long_Float (Cw)) * (Long_Float (Z.Y1 - Z.Y0) / Long_Float (Ch))));
                                    P.Wsize := (if P.Size > 0.0 and then P.Tsize > 0.0 then 1.0 else 0.0);
                                    if Rl = "front" or else Rl = "back" then
                                       --  在自己手上的眼里"比它更近/更远" = 让它【看着】变大一倍 / 变小一半(纯倍数),位置留在原处。
                                       --  这只眼跟着手走,所以"X 看着变小一半" = 我离 X 远了一倍(拿着球之后靠旁边的东西量抬起来了多少)
                                       P.Tu := O.Cu; P.Tv := O.Cv;
                                       P.Tsize := (if Rl = "front" then P.Size * 2.0 else P.Size * 0.5);
                                       P.Wsize := (if P.Size > 0.0 then 1.0 else 0.0);
                                       P.Desc := S ("item " & Codec.Img (G.Of_Item) & (if Rl = "front" then " looking twice as big" else " looking half as big") & " (in my own hand camera)");
                                    elsif Rl = "above" then
                                       P.Tv := Z.Cv + Long_Float'Max (Oh, 1.0 / Long_Float (Ch)); P.Wsize := 0.0;
                                    elsif Rl = "below" then
                                       P.Tv := Z.Cv - Long_Float'Max (Oh, 1.0 / Long_Float (Ch)); P.Wsize := 0.0;
                                    elsif Rl = "left" then
                                       P.Tu := Z.Cu + Long_Float'Max (Ow, 1.0 / Long_Float (Cw)); P.Wsize := 0.0;
                                    elsif Rl = "right" then
                                       P.Tu := Z.Cu - Long_Float'Max (Ow, 1.0 / Long_Float (Cw)); P.Wsize := 0.0;
                                    end if;
                                 end if;
                              end;
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
                                    if Rl = "into" then
                                       --  瞄进它身子里:顶面到它站着的那个面之间的一半(球 = 赤道)。由脑说出口,身体不自己挑高低
                                       P.Tz := Grab_Depth (O); P.Wz := (if O.Depth > 0.0 and then P.Z > 0.0 then 1.0 else 0.0);
                                    elsif Rl = "onto" then
                                       --  朝它靠着的那个面压过去:面 = 顶面再往下一个鼓高(都是这块自己量的)
                                       P.Tz := (if O.Top > 0.0 and then O.Height > 0.0 then O.Top + O.Height else O.Depth + O.Height);
                                       P.Wz := (if O.Depth > 0.0 and then P.Z > 0.0 then 1.0 else 0.0);
                                    elsif Rl = "off" then
                                       --  离开那个面:在这只眼里"鼓出来"= 比周围近,所以离开面 = 朝这只眼靠近;
                                       --  一截 = 它自己两个身位(高或宽取大),从【我此刻的远近】起算 ⇒ 反复说就反复抬
                                       declare
                                          Sz : constant Long_Float := Long_Float'Max (O.Height, Long_Float'Max (Ow, Oh) * O.Depth);
                                       begin
                                          P.Tu := P.Cu; P.Tv := P.Cv;
                                          P.Tz := P.Z - 2.0 * Sz; P.Wz := (if P.Z > 0.0 and then Sz > 0.0 then 1.0 else 0.0);
                                       end;
                                    elsif Rl = "at" then
                                       if P.Kind = Thing_Pt and then O.Kind in Finger | Grip then
                                          --  X 装进握区:区心、区深
                                          declare
                                             Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
                                          begin
                                             P.Tu := Z.Cu; P.Tv := Z.Cv; P.Tz := Z.Depth; P.Wz := (if Picture.Is_Nan (Z.Depth) then 0.0 else 1.0);
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
                                          P.Tz := (if Rl = "front" then O.Depth - Sz else O.Depth + Sz);
                                          P.Wz := (if O.Depth > 0.0 and then P.Z > 0.0 then 1.0 else 0.0);
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
                        if Cam_A = Integer (P.Arm) and then G.Rel /= "" and then G.Of_Item >= 1 and then G.Of_Item <= Natural (C.Items.Length)
                          and then C.Items (G.Of_Item - 1).Kind = Thing
                        then
                           --  自己的手上相机里"我的手到 X" = 让 X 的像素来到握区:改跟 X(目标在 Set_Target 里按 To_Grip 算)
                           declare
                              O : constant Item := C.Items (G.Of_Item - 1);
                           begin
                              P.Kind := Thing_Pt; P.Slot := O.Slot; P.Cu := O.Cu; P.Cv := O.Cv; P.Z := O.Depth; P.Height := O.Height; P.Count := O.Count;
                              P.Box_W := Long_Float (O.X1 - O.X0) / Long_Float (Cw); P.Box_H := Long_Float (O.Y1 - O.Y0) / Long_Float (Ch);
                              P.Size := Sqrt (Long_Float'Max (0.0, P.Box_W * P.Box_H));
                              P.Ang := 2.0 * Arctan (O.Av, O.Au);
                              P.Elong := O.Elong; P.Gray := O.Gray;
                              P.To_Grip := True;
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
                     if Ok_Pt then
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
                        P.Tu := Z.Cu; P.Tv := Z.Cv;
                        P.Tsize := Sqrt (Long_Float'Max (0.0, (Long_Float (Z.X1 - Z.X0) / Long_Float (Cw)) * (Long_Float (Z.Y1 - Z.Y0) / Long_Float (Ch))));
                        P.Tang := 2.0 * Arctan (Z.Av, Z.Au);
                        --  🔴 这只眼里远近读数不可用(单目深度在腕眼里连相对量都是反的,见 Cut_Bright 头注)⇒ 远近关掉;
                        --  "看着多大"打开:目标 = 我张开的那片地方有多大,它比任何真到指尖的东西都大 ⇒ 手一路朝它降,
                        --  降到顶住为止。2026-09-08 关掉这一项是因为深度切块的框忽大忽小;明暗切出来的球是整块、不抖。
                        P.Tz := P.Z; P.Wz := 0.0;
                        P.Wsize := (if P.Size > 0.0 and then P.Tsize > 0.0 then 1.0 else 0.0);
                        --  朝向的分量 = 这块有多长条(圆的为零)
                        P.Wang := Long_Float'Max (0.0, 1.0 - 1.0 / Long_Float'Max (1.0, O.Elong));
                        P.Desc := S ("item " & Codec.Img (Say.Grip_On) & " to sit where my fingers close (same place, same distance, same apparent size, same lie)");
                     else
                        declare
                           Tr : constant Zone_Track := C.Zones (Track_Idx (C, A, Cam));
                        begin
                           P.Kind := Piece_Pt; P.Chan_K := Chan.Per_Arm; P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z; P.Known := Tr.Known;
                           --  手指去"顶面到桌面之间的一半"处(球 = 赤道)
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
                  --  几何逼近刚算过它离指尖该到的那点多远(这一轮或上一轮)⇒ 笼住与否由那个数说,不再拿像素框/深度猜
                  Geo_Cage : constant Boolean := Geo_Case = 3;   --  和上面"合前不再走"是同一个判断,只此一处
               begin
                  if Geo_Cage then
                     declare
                        --  合手容差 = 张口的两成(比例,无量纲):指缝本来就有余量
                        Allow : constant Long_Float := 0.2 * Geo_Of (C, Cam).Gap;
                     begin
                        Caged := C.Geo_Dist <= Allow;
                        Cage_Note := S ("cage check by sightlines: the thing is " & Mm (C.Geo_Dist) & " from where my fingers close (allowed " & Mm (Allow) & ")");
                     end;
                  end if;
                  --  笼判据:点名的那块的像素在握区框里(它的形心落在区框内),深度和手指对得上
                  if not Geo_Cage and then Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then
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
                              --  远近那一行关着(Wz = 0)时不拿深度卡合手 —— 这只眼里的深度读数不可用
                              Depth_Ok : constant Boolean := Pin.Wz <= 0.0 or else Picture.Is_Nan (Hz.Depth) or else Pin.Z <= 0.0
                                                            or else abs (Pin.Z - Hz.Depth) <= Long_Float'Max (Pin.Height, Long_Float'Max (Pin.Box_W, Pin.Box_H) * Pin.Z);
                              --  看着多大在这只眼里是【方向】不是【地方】(目标 = 握区那么大,永远到不了):不拿它卡合手;
                              --  高低由"降到顶住"收口。差不超过四分之一(比例,无量纲)那条只对真有目标大小的情形
                              Size_Ok : constant Boolean := True or else Pin.Wsize <= 0.0 or else Ds <= 0.25;
                           begin
                              Caged := Dp <= Tol and then Depth_Ok and then Size_Ok;
                              Cage_Note := S ("cage check in this hand camera: it is " & Codec.Fmt (Dp, 3) & " of a frame from where my fingers close (allowed " &
                                              Codec.Fmt (Tol, 3) & "), looks " & Codec.Fmt (Pin.Size / Long_Float'Max (1.0e-9, Pin.Tsize) * 100.0, 0) &
                                              "% of the size it should, and its distance " & (if Depth_Ok then "matches" else "does not match") & " my fingertips");
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
                              Caged := Dist <= Tol and then Deep_Ok;
                              Cage_Note := S ("cage check in this camera: my grip centre is " & Codec.Fmt (Dist, 3) & " of a frame from the thing (allowed " &
                                              Codec.Fmt (Tol, 3) & "), and my fingers sit at " & Codec.Fmt (Pin.Z, 3) & " while the middle of the thing is at " &
                                              Codec.Fmt (Want, 3) & " ⇒ " & (if Deep_Ok then "level with it" else "NOT level with it, I must go further before closing"));
                           end;
                        end if;
                     end;
                  end if;
                  if Caged then
                     Move_Jaw (L, C, F, A, 0.0, Steps_J, Reading);
                     declare
                        Iok : Boolean;
                     begin
                        --  合到底后再等两倍稳定拍数(倍数,无量纲),让夹爪把劲使上再抬
                        Selfmap.Idle (L, F, 2 * Natural'Max (1, C.Map.Settle), Iok);
                        Reading := Selfmap.Jaw_Of (F, A);
                     end;
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
                        if Geo_Cage then
                           Geo_Held (L, C, F, A, Cam, (if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then C.Items (Say.Grip_On - 1).Slot else -1),
                                     By_Reading, By_Reading, Sure_Held, Note);
                        else
                           Held_Test (L, C, F, A, Cam, Origin, Obj_Count, By_Reading, Sure_Held, Note);
                        end if;
                        if not Sure_Held then
                           By_Reading := False;   --  说不准 ⇒ 不许记成"手里有东西"(记错了下一步它就去"搬"而不是重抓)
                        end if;
                        Did_Grip := S ("I closed grip " & Codec.Img (A + 1) & " until the picture stopped changing (" & Codec.Img (Steps_J) & " steps, reading " & Codec.Fmt (Reading, 3) &
                                       ", empty-close reading " & Codec.Fmt (Empty, 3) & "); " & To_String (Note));
                        if By_Reading then
                           C.Geo_Obs.Clear;   --  它在手里了,以前那些视线作废
                           C.Wld.Holding := True; C.Wld.Held_Arm := Integer (A); C.Wld.Held_Cam := Integer (Cam);
                           if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then
                              C.Wld.Held_Slot := C.Items (Say.Grip_On - 1).Slot;
                              C.Wld.Held_Origin := World.Get (C.Wld, Cam, Natural (C.Items (Say.Grip_On - 1).Slot)).Shadow;
                           else
                              C.Wld.Held_Slot := -1;
                           end if;
                           Memory.Set (C.Mem, "holding", "arm " & Codec.Img (A + 1) & " closed on item " & Codec.Img (Say.Grip_On) & " at reading " & Codec.Fmt (Reading, 3));
                        else
                           C.Wld.Holding := False; C.Wld.Held_Arm := -1;
                           Move_Jaw (L, C, F, A, C.Hands (A).Open_Reading, Steps_J, Reading);
                           Append (Did_Grip, "; I opened it again");
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
         --  ── 几何驾驶(腕眼)──:贴近/瞄进 = 视线交点;合 = 刚算过的距离说了算;离远 = 沿原路退
         declare
            Own : constant Integer := Cam_Arm (C, Cam);
         begin
            if Own >= 0 and then Geo_Ready (C, Cam) then
               if Say.Grip = "none" and then Natural (Say.Moves.Length) = 1 then
                  declare
                     G0 : constant Brain.Goal := Say.Moves (0);
                     Rl : constant String := To_String (G0.Rel);
                  begin
                     if G0.Item >= 1 and then G0.Item <= Natural (C.Items.Length) and then C.Items (G0.Item - 1).Kind in Finger | Grip
                       and then Integer (C.Items (G0.Item - 1).Arm) = Own
                     then
                        if (Rl = "at" or else Rl = "into") and then G0.Of_Item >= 1 and then G0.Of_Item <= Natural (C.Items.Length)
                          and then C.Items (G0.Of_Item - 1).Kind = Thing and then C.Items (G0.Of_Item - 1).Located
                        then
                           Geo_Case := 1; Geo_Slot_Now := C.Items (G0.Of_Item - 1).Slot;
                           Geo_Desc := S ("item " & Codec.Img (G0.Item) & " " & Rl & " item " & Codec.Img (G0.Of_Item) & " (by sightlines, in my own hand camera)");
                        elsif Rl = "back" and then C.Wld.Holding and then C.Wld.Held_Arm = Own then
                           Geo_Case := 2;
                           Geo_Desc := S ("item " & Codec.Img (G0.Item) & " back the way it came, holding");
                        elsif Rl = "above" and then G0.Of_Item >= 1 and then G0.Of_Item <= Natural (C.Items.Length)
                          and then C.Items (G0.Of_Item - 1).Kind in Thing | Thing_Remembered
                        then
                           --  到它上方:靠这一集里记下的三维位置(现在看不看得见它都行)
                           if Geo_Locate (C, Cam, C.Items (G0.Of_Item - 1).Slot, Geo_Pw) then
                              Geo_Case := 4;
                              Geo_Desc := S ("item " & Codec.Img (G0.Item) & " above item " & Codec.Img (G0.Of_Item) & " (by the place I remember it, in my own hand camera)");
                           else
                              Report := S ("I have not seen item " & Codec.Img (G0.Of_Item) & " from enough different places to know where it is in space, so I cannot go above it by geometry. ");
                           end if;
                        end if;
                     end if;
                  end;
               elsif Say.Grip = "close" then
                  --  合:几何账还新鲜(中间没别的段动过手,轮数只差几轮)就不再先走一段,笼住与否由刚算的距离说
                  Geo_Say ("合手前的几何账:上次算的差 " & (if C.Geo_Dist >= 0.0 then Mm (C.Geo_Dist) else "没有") & " · 那是第" & Codec.Img (C.Geo_Round) &
                           " 轮,现在第" & Codec.Img (C.Round_N) & " 轮 · 合的是第" & Codec.Img (Grip_Arm + 1) & " 只手,这只眼长在第" & Codec.Img (Own + 1) & " 只手上");
                  if Grip_Arm = Own and then C.Geo_Dist >= 0.0 and then C.Round_N - C.Geo_Round <= 3 then
                     Geo_Case := 3;
                  end if;
               end if;
            end if;
         end;
         if Geo_Case = 1 and then not Geo_Of (C, Cam).Valid then
            declare
               Cok : Boolean;
            begin
               Put_Line ("[身] 📐 这台相机的朝向还没量 ⇒ 先盯着它挪四下量出来");
               Geo_Calibrate (L, C, F, Cam, Natural (Cam_Arm (C, Cam)), Geo_Slot_Now, Cok);
               if not Cok then
                  Geo_Case := 0;
                  Report := S ("I tried to measure how my hand camera sits on my hand and could not, so I fell back to pushing by feel. ");
               end if;
            end;
         end if;
         if Geo_Case = 1 then
            Put_Line ("[身] ⚙ 几何驾驶:" & To_String (Geo_Desc));
            Geo_Approach (L, C, F, Cam, Natural (Cam_Arm (C, Cam)), Geo_Slot_Now, Step_Limit, Event, Steps_Taken, Beats);
            Feel (C, F);
            Report := Report & "you asked " & To_String (Geo_Desc) & ": " & To_String (Event) & ". I took " & Codec.Img (Steps_Taken) & " pushes; ";
            Put_Line ("[身]   这一段:" & Codec.Img (Steps_Taken) & " 推 · " & Codec.Img (Beats) & " 拍 · 这一集累计 " & Codec.Img (Plug.Steps (L)) & " 拍");
         elsif Geo_Case = 2 then
            Put_Line ("[身] ⚙ 几何驾驶:" & To_String (Geo_Desc));
            Geo_Retreat (L, C, F, Natural (Cam_Arm (C, Cam)), Cam, Event, Steps_Taken, Beats);
            Feel (C, F);
            Report := Report & "you asked " & To_String (Geo_Desc) & ": " & To_String (Event) & ". I took " & Codec.Img (Steps_Taken) & " pushes; ";
            Put_Line ("[身]   这一段:" & Codec.Img (Steps_Taken) & " 推 · " & Codec.Img (Beats) & " 拍");
         elsif Geo_Case = 3 then
            Put_Line ("[身] ⚙ 几何驾驶:合手前不再走,笼住与否由刚算的 " & Mm (C.Geo_Dist) & " 说");
         elsif Geo_Case = 4 then
            Put_Line ("[身] ⚙ 几何驾驶:" & To_String (Geo_Desc));
            Geo_Hover (L, C, F, Cam, Natural (Cam_Arm (C, Cam)), Geo_Pw, Event, Steps_Taken, Beats);
            Feel (C, F);
            Report := Report & "you asked " & To_String (Geo_Desc) & ": " & To_String (Event) & ". I took " & Codec.Img (Steps_Taken) & " pushes; ";
            Put_Line ("[身]   这一段:" & Codec.Img (Steps_Taken) & " 推 · " & Codec.Img (Beats) & " 拍");
         else
         Build_Goals;
         if not Pts.Is_Empty then
            C.Geo_Dist := -1.0;   --  老路要动手了,几何账作废
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
                     Report := Report & "item " & Codec.Img (P.Item_No) & (if P.Blob = 0 then " (finger A)" else "") & " now at (" & Codec.Fmt (P.Cu, 2) & "," & Codec.Fmt (P.Cv, 2) &
                               ") depth " & Codec.Fmt (P.Z, 2) & ", still " & Codec.Fmt (P.Steps_Err, 1) & " pushes away; ";
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
         end if;
         Do_Grip;
         Report := Report & Mode_Line (C, To_String (Event));
         --  程序模式:这一节的事件翻成结局词喂回状态机(try/repeat/if 只认它),并把这一节的话攒起来一起给脑
         if not C.Use_Json and then C.Have_Prog then
            declare
               O : constant Sinew.Outcome := Classify (To_String (Event), To_String (Did_Grip));
            begin
               Runtime.Report (C.Prog, C.M, O);
               Append (C.Prog_Log, (if Length (C.Prog_Log) > 0 then ASCII.LF & "" else "")
                       & "[" & Sinew.Outcome_Word (O) & "] " & To_String (Report));
               Put_Line ("[身] ◀ 这一节的结局:" & Sinew.Outcome_Word (O) & "(" & Sinew.Outcome_Cn (O) & ")");
            end;
         end if;
      end;
      C.Recent := Report;
      Put_Line ("[身]   ⇒ " & To_String (Report));
   end Round;
end Act;
