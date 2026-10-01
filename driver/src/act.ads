--  执行器:把脑说的"第几号 → 去哪 → 直到什么事件为止"解成通道命令。
--  跟踪的点只有两种:我的握区(世界相机里靠光流跟;手上相机里是固定像素)、世界里的一块(每步重切)。
--  解算 = 带限的加权最小二乘分配;表每步递推重估;碰上/推不动 = 零表比走的表更准;抓 = 块装进握区,笼住了才合。
with Bytes; use Bytes;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
with Plug;
with Geom;
with Instrument;
with Selfmap;
with Zone;
with World;
with Brain;
with Picture;
with Table;
with Memory;
with Schema;
with Chan;
with Learned;
with Plan;
with Sinew;
with Runtime;
with Monitor;
with Contact;
with Contact.Search;
with Contact.Qty;
package Act is
   --  🔴 脑写的结局词 → 身体的判法。**只有这一处**。
   --  以前它散在两个局部函数里(Outcome → 字符串 → Until_Kind),中间那一跳把 lost / free / refused
   --  三个词悄悄并进了兜底的"走够步数":脑写一个词,身体做的是另一个词的事,还回报"步子走完还没到"。
   --  自检逐词钉死这张表(selfcheck「每个结局词都有自己的判法」),再想并词就会当场红。
   function Until_Of (O : Sinew.Outcome) return Monitor.Until_Kind;
   --  只有脑真写了 arrived,身体才准因为"约束满足了"而停(timeout 同样走步数上限,但不许自称到了)
   function Wants_Arrive (O : Sinew.Outcome) return Boolean;
   function Until_Word (O : Sinew.Outcome) return String;   --  给旧的 Brain.Say 用的同一张表
   function Kind_Of_Word (W : String) return Monitor.Until_Kind;   --  字符串那一跳的反向表(自检钉死它和 Until_Of 一致)
   --  关系词 → 执行器认得的那个字。四个词(close / open / clear / still)不走这里,它们各有各的分支。
   --  剩下的每一个都必须有自己的、非空非 "?" 的字 —— 自检钉死,防的是和结局词同一类的悄悄降级。
   function Rel_Cmd (R : Sinew.Rel) return String;
   function Rel_Has_Own_Branch (R : Sinew.Rel) return Boolean;
   type Item_Kind is (Finger, Grip, Piece, Thing, Thing_Remembered, Thing_Held);   --  Piece = 我身上某个通道带的一块(Which = 通道号)
   type Item is record
      Kind : Item_Kind := Thing;
      Arm : Natural := 0;
      Which : Natural := 0;
      Jaw_K : Natural := 0;          --  Finger/Grip:这是这条臂的第几个抓握通道(五指手一根手指一个)
      Slot : Integer := -1;
      Located : Boolean := False;
      Cu, Cv : Long_Float := 0.0;
      X0, Y0, X1, Y1 : Natural := 0;
      Depth, Height : Long_Float := 0.0;
      Count : Natural := 0;
      Au, Av : Long_Float := 0.0;    --  这一块自己的主轴(画面里的单位向量)
      Elong : Long_Float := 1.0;     --  长轴/短轴
      Gray : Long_Float := -1.0;     --  框里的平均灰度(< 0 = 没量到)
      --  🔴 这一块是在【哪台相机】里看见的。以前整张清单默认就是"当前这台",
      --  于是脑只能点名当前那台里的东西 —— GM 里我答"一个都不是",而【头顶相机里球一直看得见】,
      --  只是它没有号可点。带上这一位,清单才谈得上跨相机。
      Cam : Natural := 0;
   end record;
   --  角色 → 它肯收哪种自己的零件。**只有这一处**,自检钉死 grasper 和 pusher 不许收同一种
   --  (语言里 pusher 就是"推得动东西、但【合不拢】的部件";以前它把 Grip 也收了,
   --  于是两个角色绑到同一块,语言里的角色区分是假的)。
   function Role_Wants (R : Sinew.Role; K : Item_Kind) return Boolean;
   --  🔴 从身体图外推手的位置,炸没炸。Was = 样本里那两瓣本来隔多远,Now = 外推之后隔多远。
   --  差得比它本身还大 ⇒ 这次外推不作数(零系数:两个都是量出来的长度)。
   --  箱上真数据:样本存的是 0.137,而身体报给脑的是四分之三个画面 —— 就是这里炸的。
   function Extrapolation_Blew (Was, Now : Long_Float) return Boolean;
   --  🔴 拿住了没,唯一分得开的那一条:抬手时它跟着我的手走了【同样一段】。
   --  只看"它原来待的地方空了"分不开【撞跑】—— 球被我撞到画面角落,原地照样空了,
   --  身体照样报"拿住"并开始抬爪,而两指之间什么都没有(FM/FO 实测,三次假拿住全是这么来的)。
   --  零系数:两段位移的差比【我的手自己挪了多远】的一半还小 ⇒ 它跟着我走了。
   --  手一步没挪 ⇒ 判不了(Hand 位移为 0 时恒假),由调用方报"我说不准",不许自称拿住。
   function Came_With_Me (Obj_Du, Obj_Dv, Hand_Du, Hand_Dv : Long_Float) return Boolean;
   --  🔴🔴 眼睛要搜多宽:一步预计在画面里跑多远(像素),就要几层金字塔。
   --  每加一层分辨率减半,所以最粗那一层看到的位移 = 原位移 ÷ 2^(层-1);
   --  光流只有在【最粗那层的位移小到一个像素上下】时才找得准。
   --  这不是调参:1 个像素是这台相机的分辨率极限本身,是量出来的。
   --  记录 2026-08-27 V2:搜索范围比真实位移小的时候,它会【静默返回一个完全错误的位置,从不报错】。
   --  记录 2026-08-26 D6:反过来把步子缩小去迁就窄搜索 ⇒ 信号和噪声一样大,一列只解释掉 42%。
   --  所以只有一条路:**推得够大,搜得够宽**。
   function Levels_For (Px_Move : Long_Float) return Positive;
   --  🔴🔴 体检:一根【平移】通道推一米,我离相机的远近最多变一米(正好沿着相机看的方向走时取到 1)。
   --  所以响应表里"远近"那一格的绝对值【大于 1 就是物理上不可能】—— 那不是噪声,是我的深度读数尺度错了。
   --  HW 实测:ch8 那一格是 -36.7、-60.3 ⇒ 深度读数至少被放大了几十倍,而这个证据一直躺在表里没人看。
   --  这就是"拿自己的胳膊当尺子":我知道自己真走了多少米,也看得见深度读数变了多少,一除就是那个倍数。
   function Depth_Scale_Bad (Depth_Per_Metre : Long_Float) return Boolean;
   --  🔴🔴 来回对表【一行一判】。Dif = 去程和回程这一行之差的长度,Con = 两遍之和的一半的长度。
   --  HZ 2026-09-15 实测为什么必须分行:把五行合成一个数来判整根通道时,那个数几乎全是【深度行】贡献的
   --  (ch7:左右 -0.148/-0.019,远近 -5.024/-3.799 —— 深度比左右大三十多倍),
   --  于是深度一行的噪声就能把一整根【画面里量得准准的】通道判死。当场后果:6 根判死 4 根,
   --  活下来的两根左右都是 0.000 ⇒ 解算连着 10 步返回全零、身体一动不动,而日志每一行都是绿的。
   --  Con <= 0 = 这一行两遍都是零 ⇒ 没有证据说它错(也没有证据说它对),照原样留着。
   function Row_Is_Measurement (Dif, Con : Long_Float) return Boolean;
   --  🔴🔴 拿自己的胳膊当【尺子】,不是当温度计(2026-09-15 定)。
   --  以前 `Depth_Scale_Bad` 只量出"我的距离感坏了几十倍",量完就打印一句话,**从来没拿它量过任何距离**。
   --  真正的尺子是这个:我自己挪了 Moved 米(关节读数给的,不碰深度图),
   --  这一块在画面里游了 Ran 画幅 ⇒ 它有多近 = Ran / Moved(画幅每米)。越近游得越多。
   --  ⚠️ 挪得不够就不是测量(三角形太扁,误差 ∝ 距离²/基线):Moved 不过本体读数抖动、
   --  或者 Ran 不过跟踪抖动 ⇒ 不出数(返回 0),不许给一个看起来正常的烂数。
   --  记录 2026-08-16:挪 4 mm 而至少要 50 mm,当时的做法就是【当场拒绝】。
   function Near_From_Motion (Ran, Moved, Ran_Floor, Move_Floor : Long_Float) return Long_Float;
   --  🔴🔴 "它比我远几倍" —— 前后这一维唯一不需要任何常数的判据。
   --  同一台相机、同一段挪动里量到的两个游速之比 = 两者【离这台相机的距离】之比,
   --  焦距、基线、深度尺度全部约掉。== 1 就是"它和我在同一个远近上"。
   --  ⚠️ 只有【相机长在我没动的那条胳膊上】时两边才都游得起来:
   --  相机长在我正在推的这条胳膊上 ⇒ 我的爪心跟着相机走,恒不游;相机不动 ⇒ 世界里的东西恒不游。
   --  两种退化都会让这个比值变成 0 或无穷,所以测距必须换一只【长在我没动的部件上的】眼睛。
   --  量不出来(谁没游够)⇒ 返回 0 = "我说不准",不许假装等于 1。
   function Farther_By (Near_Me, Near_It : Long_Float) return Long_Float;
   --  🔴🔴 一只眼 + 会动 + 知道自己动了多远 ⇒ 真实米数。所有机体通用
   --  (一条胳膊、两条胳膊、无人机都成立;第一版写成"甩另一条胳膊"是把这台机器人当成了所有机体,已撤)。
   --  同一下拨动,在离我 Z 的东西上滑 S ∝ 1/Z;走近 D 之后再拨【同样一下】,滑 S₂ ∝ 1/(Z−D)
   --  ⇒ 此刻它离我 = D × S₁ ÷ (S₂ − S₁)。单位就是 D 的单位(米),焦距/基线/深度尺度全部约掉。
   --  ⚠️ D 用的是【总共走了多远】,公式要的是【朝它走了多远】;两者只有一直朝它走时相等,
   --  否则真实距离比这个数【更近】—— 调用方必须把这一句照实说出去。
   --  ⚠️ S₂ − S₁ 要过的是【滑速本身的噪声】,不是画幅噪声 —— 单位必须对上(IC 2026-09-15 实测):
   --  滑速 = 滑了几幅 ÷ 挪了几米,单位是"幅每米";而跟踪抖动的单位是"幅"。
   --  拿"幅"当"幅每米"的门槛 ⇒ 门槛小了三个数量级 ⇒ 这道闸永不响 ⇒ 噪声当场变成距离:
   --  IC 实测两次只隔 7 mm、滑速差 0.0022(而跟踪抖动就有 0.0016)⇒ 报出"离我 0.014 m",
   --  而球在几十厘米外。滑速的噪声 = 跟踪抖动 ÷ 这一拨挪了多少米,调用方要按这个传。
   --  🔴 算出来的距离【不许短于这一段走过的路】(IJ 2026-09-15 实测报出"离我 0.000 m")。
   --  正好等于是可以的:滑速翻一倍 = 它离我只剩一半 ⇒ 它就在我刚走的那么远处。
   --  它要是比我刚走的那一段还近,我早该从它身上穿过去了 —— 而它还在我前面看得见,自相矛盾。
   --  真正的原因是【两次量之间走得太短】(IJ 只走了 2 mm):这么短的行程只分辨得出几毫米的距离。
   --  分辨得出多远,也是量出来的:滑速 × 走了多远 ÷ 跟踪抖动。超过这个就只说"它比这个远"。
   function Distance_Now (Travelled, Swim_Then, Swim_Now, Floor : Long_Float) return Long_Float;
   --  这一段走这么远,最远能分辨到多远(超过它,滑速的变化就淹在跟踪抖动里了)
   function Can_Tell_Upto (Swim_Now, Travelled, Floor : Long_Float) return Long_Float;
   --  🔴🔴 "同一下拨两遍"里的【同一下】,指的是【在世界里往同一个方向挪同样多】,
   --  不是"同一根关节同样的命令"(IC 2026-09-15 实测栽在这儿)。
   --  一块在我眼里滑多远 = 焦距 × 我【横着】挪了多少 ÷ 它有多远。
   --  同一根关节的同一个命令,在不同姿势下把手往【不同方向】推 ⇒ 横着的那一份变了 ⇒
   --  滑速跟着变,而它跟远近毫无关系。于是"滑得比上次多"被读成"我走近了",
   --  实测报出"离我 0.014 m"而球在几十厘米外 —— 数看着完全正常,没有任何一处不一致。
   --  身体自己知道每一拨把手推向了哪儿(位姿里就有),所以这件事只要【比一下方向】就挡得住。
   --  🔴 容差不是我拍的,而且不能用"位置读数抖动 ÷ 挪了多远"(II 2026-09-15 实测那样写永远不放行:
   --  抖动量出来是 0 ⇒ 门槛正好 1.0 ⇒ `cos > 1.0` 恒假 ⇒ 方向一致度 1.000 也被判成"不是同一下")。
   --  正确的容差来自【这点方向差看不看得出来】:方向差 θ 让滑动变化约 (1−cos θ) 倍,
   --  小于跟踪抖动 ÷ 这一次滑了多少 就淹在噪声里,两拨当然算同一下。两个数都是量出来的。
   --  🔴 "同一下"要同时管【方向】和【幅度】(IK 2026-09-15:只管方向 ⇒ 报出"离我 0.014 m")。
   --  实测两拨一个挪 0.0033 m、一个只挪 0.0004 m —— 差八倍,方向却完全一致,于是过关。
   --  可幅度差八倍时,那一拨落在关节的死区里,滑速和幅度早就不成正比了,除下来的滑速根本不能比。
   --  容差跟方向用同一条:这点差看不看得出来 = 跟踪抖动 ÷ 这一次滑了多少。
   function Same_Nudge (Dot, Len_A, Len_B, Slid, Floor : Long_Float) return Boolean;
   --  🔴 这一帧读到的远近收不收。Old_Z = 上一次【真读到】的(不是按位姿猜的),Pred_Z = 表预测这一步该到哪儿,
   --  Noise = 这一点自己量到的读深抖动。没有预测(Pred_Z<=0)时【不许整条放行】—— 那正是 FS 实测
   --  "手指离相机 0.454 m 一步跳到 0.010 m(一厘米,物理上不可能)"被收下的原因;退回"一步最多变自己抖动那么多"。
   --  Last_Rejected = 上一次被这道闸拒掉的读数(0 = 上一次是收下的)。
   --  连着两次被拒而两次读数互相吻合 ⇒ 新值可重复、旧基准才是陈的 ⇒ 收下 ——
   --  否则闸拒了一次就永远拿旧值当基准,真实深度一变就锁死(HE 实测连着 12 步一模一样 2.182 m)。
   function Depth_Ok (Zd, Old_Z, Pred_Z, Noise, Last_Rejected : Long_Float) return Boolean;
   --  🔴 画面上重合 ≠ 真的在一起。目标在它自己那个远近上,我在我的远近上;同一段真实横移,
   --  离相机越近在画面里跑得越多。所以要比的不是画面坐标本身,而是【把目标搬到我这个远近平面上】之后的坐标。
   --  焦距在两边同样出现、自动约掉 —— 一个标定参数都不要。
   --  FZ 实测:头顶相机报"差 0.062 幅、几乎压上了",而爪子在球上方 30 厘米。只比画面坐标就是在比影子。
   function On_My_Plane (T_Pic, T_Depth, My_Depth : Long_Float) return Long_Float;
   --  🔴 这一步的命令上限:眼睛跟得住的那个天花板,底下垫一块【身体自己动得起来】的地板。
   --  地板 = 身体噪声的两倍(量出来的)。命令比这还小 ⇒ 发出去身体一动不动,这一步白走。
   --  地板【不是】探针那一档 —— 探针那一档是 FO 用的四倍(0.026 vs 0.006),
   --  按探针那一档当地板,球被甩出视野;FO 正是拿它的四分之一,一步推进 8 厘米、44 推抓到球。
   --  Dead = 这个通道自己量出来的死区(命令比它小,身体不动);还没学到就是 0。
   function Push_Cap (Ceiling, Noise, Dead : Long_Float) return Long_Float;
   --  🔴 into 瞄哪儿:它自己的皮(这块的中位深度)和它站着的那个面,正中间。两个都是量出来的深度。
   function Into_Depth (Skin, Surface : Long_Float) return Long_Float;
   --  🔴 这一段到底能走几步。脑写了 or N steps 就是 N,没写就用安全上限。
   --  **永远不许返回 0** —— 上限 0 交给 Monitor.Fired,U_Steps 判 W.Steps >= 0 第一步就成立,
   --  一段只走一推(GM:三段 until arrived 各 1 推 5 拍,身体却回报"步子走完还没到")。自检钉死。
   function Effective_Cap (Say_Steps : Natural) return Positive;
   function Safety_Cap return Positive;   --  脑没写步数时用的那个上限(自检钉死它不是 1 —— "没写"不等于"只走一步")
   package Item_Vectors is new Ada.Containers.Vectors (Natural, Item);

   --  响应表已挪进 Learned(体检要审判它,执行器要用它 —— 谁也不该依赖谁的上层)
   subtype Track_Kind is Learned.Track_Kind;
   subtype Stored_Effect is Learned.Stored_Effect;
   package Effect_Vectors renames Learned.Effect_Vectors;
   type Known_Array is array (0 .. Chan.Per_Arm) of Boolean;
   type Zone_Track is record
      Valid : Boolean := False;
      Cu, Cv, Z : Long_Float := 0.0;
      Stale : Natural := 0;
      Au, Av, Bu, Bv : Long_Float := 0.0;   --  两瓣各自的位置(从身体图按此刻位姿算出)
      Has_Lobes : Boolean := False;
      Known : Boolean := False;             --  此刻位姿离某个真看过的样本不超过一步核实过的步幅 ⇒ 不用看就知道
      Blew_Up : Boolean := False;           --  这次从样本外推炸了(算出来的两瓣间距和样本里的差得比它本身还大)
      Pieces : Schema.Part_Array;           --  这只手每个通道带的零件此刻在这台相机里的位置(按位姿从身体图算)
      Pieces_Known : Known_Array := [others => False];
   end record;
   package Zone_Track_Vectors is new Ada.Containers.Vectors (Natural, Zone_Track);

   --  记住的一个地方:那一刻它在这台相机画面里的位置和远近。名字是脑起的,数留在身体里。
   type Place is record
      Name : Unbounded_String;
      Cam : Natural := 0;
      Cu, Cv, Z : Long_Float := 0.0;
   end record;
   package Place_Vectors is new Ada.Containers.Vectors (Natural, Place);

   --  脑点过名的一件东西:它说"在这一框里",我在框里量出了它。之后每一帧,我在【上一帧量到它的地方】
   --  原样再量一遍(Picture.Measure_In_Box)—— 跟住它靠的是重新量,不是靠全图切块里碰巧有一块像它。
   --  名字是脑起的,框和像素留在身体里;编号照旧不进语言。
   type Boxed_Thing is record
      Name : Unbounded_String;
      Cam : Natural := 0;
      X0, Y0, X1, Y1 : Natural := 0;      --  上一次量到它的像素框(闭区间)
      Cu, Cv : Long_Float := 0.0;         --  上一次量到的形心(归一化画幅;认槽用)
      Seen : Boolean := False;            --  这一帧量到了吗
      Blind : Boolean := False;           --  脑看着这只眼的图说过"我指不出它"(按【这个名字 × 这只眼】记;脑再指一次就作废)
      Gray, Bg : Long_Float := -1.0;      --  脑指它那一帧:它的像素平均多亮、它框里的背景平均多亮(< 0 = 没量);每帧重量时认它靠这个
      Isolated : Boolean := False;        --  量到的那一块是单独的吗(没顶到让出来的那一圈)
      Mask : Bools;                       --  这一帧它的像素(整幅;合手前在它身上挑夹得住的那一处要用)
      Count : Natural := 0;               --  上一次认出它时它有多少像素(窗挪到预测处时按远近比例缩放):这一帧量到的块小到不足它的四分之一就不是它
      --  它身上离形心最近的那个像素(< 0 = 没有):窗挪到预测处时跟着平移、缩放,给分割仪器当"就是这一点"。
      --  只给框不给点,框一放大 SAM 就抠了框里别的东西(SHOT2 2026-09-26:窗大了 2 倍,抠出来的块亮 110、剪刀本来 198);
      --  不用形心本身:剪刀这种中间空的东西,形心落在桌面上
      Pu_On, Pv_On : Long_Float := -1.0;
   end record;
   package Boxed_Vectors is new Ada.Containers.Vectors (Natural, Boxed_Thing);

   package Buf_Vectors is new Ada.Containers.Vectors (Natural, Buf, U8_Vectors."=");
   --  这条臂横着被顶住过的地方:面上一点(那一刻的指尖)+ 顶住我的方向(指向面里)。不是它躺的面(那种记成 Touch),是墙、或我自己的关节到头
   --  ⇒ 这一集里,这条臂再选落点时,顶住方向那一侧的都算"够不着"(量出来的无能,不是意见)
   type Wall_Mark is record
      Arm : Natural := 0;
      P, W : Geom.V3 := [others => 0.0];
   end record;
   package Wall_Vectors is new Ada.Containers.Vectors (Natural, Wall_Mark);
   --  一件东西和这只身体之间量到的摩擦(接触集重写 09-29):合上、抬一点它跟着走 ⇒ 这一组最坏要的摩擦它给得起(下限往上走);
   --  没跟着走 ⇒ 这一组按量到的法向要的摩擦它给不起(上限往下走)。挑下手处按它们,不再"这一处拉黑、换下一个"
   --  I5 要怎么动(大并行 §4 第 0 步;路 7 从语言填,路 5 / 6 读;10-01 主代理批的加法,别的一处不动)。
   --  一个要 = 哪件(清单号,1 起)、哪个量(语言的根那一句:Rel = Re_Qty、Qty = 量的名字;两件东西那一句:Rel = 那个关系词,Qty 空)、
   --  往哪变(+1 往上 / -1 往下;关系词那一句由关系词定,照样填 ±1)、参照的那一件(清单号;0 = 没有)。可以同时几个
   type Want is record
      Thing : Natural := 0;
      Rel : Sinew.Rel := Sinew.Re_None;
      Qty : Unbounded_String;
      Dir : Integer := 0;
      Ref : Natural := 0;
   end record;
   package Want_Vectors is new Ada.Containers.Vectors (Natural, Want);

   type Grip_Mu is record
      Name : Ada.Strings.Unbounded.Unbounded_String;
      Lb : Long_Float := 0.0;
      Ub : Long_Float := Long_Float'Last;
   end record;
   package Grip_Mu_Vectors is new Ada.Containers.Vectors (Natural, Grip_Mu);
   --  不动的眼一笔合空标记里它看见的手指像素(2026-09-26,V1"头顶眼按指尖"的量法:碰出来的指尖投进它眼里,离这片像素最近多远)
   type Px2 is record
      U, V : Natural := 0;
   end record;
   package Px2_Vectors is new Ada.Containers.Vectors (Natural, Px2);
   package Px_List_Vectors is new Ada.Containers.Vectors (Natural, Px2_Vectors.Vector, Px2_Vectors."=");
   --  标定板用的一停(2026-09-25):腕眼标定里只平移、手没转的那几停,手上那只眼的图 + 手的位姿 + 同一刻不动的眼的图。
   --  Seg = 那一集开始时的帧号:复位之后桌上的东西换了,不同集的停不互相配
   type Board_Stop is record
      Cam, Arm : Natural := 0;
      Seg : Natural := 0;
      Pose : Plug.Arm_Pose := [others => 0.0];
      W, H : Natural := 0;
      RGB : Buf;
      Hw, Hh : Natural := 0;
      Head : Buf;
   end record;
   package Board_Stop_Vectors is new Ada.Containers.Vectors (Natural, Board_Stop);
   type Context is record
      Map : Selfmap.Body_Map;
      Hands : Zone.Hand_Vectors.Vector;
      Wld : World.State;
      Mem : Memory.Store;
      Cam : Natural := 0;
      Tables : Effect_Vectors.Vector;
      Zones : Zone_Track_Vectors.Vector;     --  (臂 × N_Cams + 相机)
      --  🔴 每个通道自己的【死区】:命令小于它,身体根本不动(实测随姿势和关节而变 ——
      --  FO 时 0.006 能动,GV 时同一批通道 0.013 实到 0.000)。身体本来就看得见"我命令了多少、实到多少",
      --  只是从来没拿它去调下限。命令发了实到为零 ⇒ 把这一档抬上去;真动了 ⇒ 把它压下来。
      Dead : Floats;                         --  按全局通道号索引(机体自己的单位;只和自己比)
      --  🔴🔴 每一根通道推一个单位,我的手在【世界里】真走几米(探针时顺手量的,关节读数给的)。
      --  这是把尺子量出来的【米】接进解算的那个换算:还差几步 = 还差几米 ÷ 一推走几米。
      --  没有它,尺子量出来的距离就只能打印出来给脑看,进不了解算 ——
      --  IM 2026-09-15 实测:横向对到 2 毫米、前后还差 0.535 m,而解算说"还差 0.0 步",
      --  因为前后那一栏靠的还是放大几十倍的深度读数,没有任何真东西在驱动它。
      Reach_M : Floats;                      --  按全局通道号索引(米 / 单位命令)
      --  🔴 自我那一格:我量出来自己的深度读数被放大了几倍(0 = 还没查出问题)。
      --  这是拿自己的胳膊当尺子量出来的,不是别人告诉我的。
      Depth_Scale : Long_Float := 0.0;
      --  "我变了什么":本次开机以来身体自己注意到的变化(某根通道不听话了、某台眼睛看不见我了…)
      Changed_Say : Unbounded_String;
      Recent : Unbounded_String;
      Task_Text : Unbounded_String;
      Items : Item_Vectors.Vector;
      --  每台相机【画过框、编过号】的那一份图。条带里给脑看的就是它 ——
      --  以前条带只给原图,别的相机里的东西看得见却没有号,脑点不了名。
      Shown : Buf_Vectors.Vector;
      Cells_U, Cells_V : Floats;
      Cols : Natural := 6;
      Rows : Natural := 4;
      Look_Only : Boolean := False;
      Eye_Host : Unbounded_String;
      Eye_Port : Natural := 8079;
      Inst_Host : Unbounded_String;    --  仪器进程(空 = 没配,几何全靠身体自己量)
      Inst_Port : Natural := 8077;
      Fixed_Obs : Geom.Obs_Pt_Vectors.Vector;   --  开机各停里不动的眼给各条臂指尖做的合空标记(Pt = 臂号;像素 + 那一停的位姿),开机末尾一起解不动的眼
      Mark_Px : Px_List_Vectors.Vector;         --  和 Fixed_Obs 一一对应:那一笔里不动的眼看见的手指像素
      Lobe_Obs : Geom.Obs_Pt_Vectors.Vector;    --  同一批标记里每一瓣手指各自的尖(Pt = 臂号,Kind = 这一笔的瓣数):解完不动的眼后认哪一瓣落在腕眼哪条瓣视线上 = 指尖
      Board_Stops : Board_Stop_Vectors.Vector;  --  腕眼标定各平移停的图和位姿(开机末尾配点做标定板)
      Board : Geom.Scene_Pt_Vectors.Vector;     --  标定板的点:腕眼几停三角出来的桌上的点(世界位置 + 协方差)和它们在不动的眼里的像素
      Board_Tracks : Geom.Board_Track_Vectors.Vector;   --  同一批点在腕眼里的几停原样(一起解腕眼和不动的眼时按新几何重新三角)
      Board_Plane : Boolean := False;           --  标定板的点拟合出了它们躺的那张面
      Fixed_Ref : Buf;                          --  不动的眼标好(或上一次核对)那一刻的图:每轮拿它和此刻的图配板上的点,核它挪没挪、挡没挡
      Fixed_Ref_W, Fixed_Ref_H : Natural := 0;
      Fixed_Best : Geom.Fixed_Best;             --  不动的眼这一次放好以来,核对时对得上的最多点数:整幅的和每一块的(挡没挡按它比;挪过就重记)
      Board_Pt, Board_N : Geom.V3 := [others => 0.0];
      Board_Rms : Long_Float := 0.0;            --  板上躺在面上的那些点离面的离散(米)
      --  板上每个点上一回在不动的眼里重找(Board_Recheck)找没找到:和 Board 一一对应;空 = 开机量完还没重找过(按量的那一刻)。
      --  找不到的(被挪来的东西盖住、被手挡着)挑空地时不算量过的桌面(2026-09-28 V1B47)
      Board_Seen : Bools;
      --  压之前先看底下(09-30 V1B70 / V1B73):手上那只眼往下压的第一步前后两帧三角出、比面高出(同挑空地的"高出面")的点(世界位置 + 协方差)。
      --  挑空地时和高出面的板点一样挡
      Seen_Above : Geom.Scene_Pt_Vectors.Vector;
      --  同一对立体像里躺在面上、而且高低量得够细的点(10-01 大并行 §2 第 5 条"看底下看见的点也能当量过的桌面"):挑空地时和躺在面上的板点一样
      --  当量过的桌面(围得住落点圈、能当落点)。够细 = 沿法向 Z 倍的不确定度不过碰指尖认几下对得上的那道门(比它矮的东西压上去也认得出;
      --  视线挨着眼走的方向那一片远近定不住,进不来)。碰指尖那一段量、用;换了板一起作废(同 Seen_Above)
      Seen_On : Geom.Scene_Pt_Vectors.Vector;
      Fixed_Turn_Sd : Long_Float := 0.0;        --  仪器把参考图配到"它自己转了 90°"那张时的配点噪声(像素,均方根;核对时判新位姿用,0 = 没量)
      Fixed_Said : Boolean := False;            --  这一次开机第一次核对的结果说过了(以后只在挪了、挡了、又看全了时说)
      Fixed_Covered : Boolean := False;         --  上一次核对判成挡住了
      Fixed_Turn_Next : Natural := 0;           --  挡住期间下一次"把画面转回去再配"在第几轮试(间隔每试一次翻倍:越挡越久试得越少)
      Fixed_Turn_Gap : Natural := 1;
      Fixed_Turn : Natural := 0;                --  每轮核对先把此刻的图顺时针转几个 90° 再配(相机被转过之后,转正了再配,配点一直是转正的精度)
      Dump_Dir : Unbounded_String;
      Round_N : Natural := 0;
      Fast : Boolean := False;
      Boot_Steps : Natural := 0;   --  开机量身体用掉的拍数(记账,不是上限)
      Sch : Schema.Map;            --  身体图:位姿 → 手指在各相机画面里的位置(只存真看见过的)
      Want_Size : Long_Float := 0.0;   --  正在跟的那块东西现在看着多大(画幅):切块的窗口要比它大,否则闭运算把它填平、只剩一圈边(EV 实测球被切成三块)
      Cut_Seq : Natural := 0;      --  切块缓存:这一帧的编号(同一帧同一台相机不重切,颜色切块很贵)
      Cut_Cam : Integer := -1;
      Cut_Regs : Picture.Regions;
      Boxed : Boxed_Vectors.Vector;    --  脑点过名、我在框里量出来的那几件东西(每帧原地重量,见 Boxed_Thing)
      --  🔴 脑不再一轮填一张表,而是交【一段程序】。程序编译过了就存在这里,一轮跑一小节,
      --  跑完才回去问下一段 —— 这才是"少问几百次"的来源。
      Prog : Sinew.Program;            --  脑交的那一段程序(带循环/分支/定义)
      M : Runtime.Machine;             --  跑到哪一条了
      Binds : Plan.Bind_Vectors.Vector;--  每个名词落到了哪一块
      Have_Prog : Boolean := False;
      Refused : Unbounded_String;      --  上一段被退回的话:理由 + 能照抄的替代,随下一轮一起给脑
      Places : Place_Vectors.Vector;   --  remember 记下的地方:身体自己能重新找到的位置,不是坐标
      Last_Outcome : Sinew.Outcome := Sinew.Oc_None;   --  上一节的结局(八个词之一)
      Blind_Say : Unbounded_String;    --  身体照走了,但有件事要如实说给脑(不是停,是说)
      Eye_Chosen : Boolean := False;   --  这一集已经自己换过一次眼睛了(不许来回弹)
      Reckless : Boolean := False;     --  这一节写了 anyway:身体的一切谨慎作废
      Eye_Want : Sinew.Eye_Pick := Sinew.Ey_None;   --  这一节脑点了用哪只眼睛(没点 = 身体自己挑)
      Tgt_Cam : Integer := -1;         --  脑点名的那一块在哪台相机里(-1 = 这一节没点名东西)
      Name_Cam : Integer := -1;        --  脑【最近一次真的认出来】一个名字时,身体在哪只眼里(-1 = 还没认出过)
      --  脑说过"这只眼里没有它"记在 Boxed 里(名字 × 眼);只按眼记一只会在两只看不见的眼之间来回弹,只按眼记又会让一个绑不上的 it 把整只眼判死
      --  这一轮最多列几件世界里的东西。0 = 不设限。
      --  🔴 不是拍的数:脑装不下时回包里写着限额和用量(实测 "maximum context length is 8192 tokens…
      --  your prompt contains at least 7493 input tokens"),驱动照着把它减半再来,减到装得下为止。
      --  以前从不量"我能给多少",只会一股脑全给 ⇒ 单帧 347 件、3589 轮里 3577 轮撞墙,脑几乎没看见过画面。
      List_Cap : Natural := 0;
      --  上一段程序的原文,和它有没有让身体动过。
      --  🔴 CS3 实测:45 段里 43 段第一句一字不差(`remember where grasper is as myhand`),
      --  全炮只有 3 种开头 —— 那不是 45 个样本,是 1 个样本的 43 份复印件。
      --  死循环是闭合的:它写了一句不动身体的话 ⇒ 世界没变 ⇒ 提示词没变 ⇒ 温度 0 ⇒ 又写同一句。
      --  身体要把这件事【如实说出来】(这是报告,不是窍门)。
      Last_Prog : Unbounded_String;
      Last_Moved : Boolean := True;
      Prog_Log : Unbounded_String;     --  🔴 这一段程序里【每一节】的结果都攒在这儿。
                                       --  以前只留最后一节,而最后那一轮恰好是"程序跑完了"的空话,
                                       --  于是前几节说了什么全被冲掉,脑只能去翻日志 —— 等于身体不会说话。
      --  ── 几何驾驶(腕眼里只用彩色图 + 手的位姿读数 + 焦距;不读深度)──
      Geo : Geom.Geo_Vectors.Vector;      --  每台相机一份:焦距、朝向、指尖
      Geo_Path : Unbounded_String;        --  几何常数存哪(身体文件旁边)
      Geo_Dist : Long_Float := -1.0;      --  上一次几何逼近结束时,它离"指尖该到的那一点"还差多少米(< 0 = 没有)
      Geo_Round : Natural := 0;           --  那是第几轮
      Geo_At : Plug.Arm_Pose := [others => 0.0];   --  算那个距离时手在哪(位姿读数);手没挪开,那个距离就还作数
      Geo_At_Arm : Integer := -1;
      Geo_At_Above : Boolean := False;    --  那个距离是到【它上方一个张口】的,不是到它身上的 ⇒ 合手前得先下去
      Geo_Pw_Met : Boolean := False;      --  记住的位置来自两眼交点(这一段里);单眼挪出来的估计不许盖掉它
      Geo_Came : Long_Float := 0.0;       --  几何逼近一共走了多远(米);"离远点"就沿原路退这么远
      Geo_Dir : Geom.V3 := [others => 0.0];   --  逼近的方向(世界系单位向量)
      Geo_Obs : Geom.Obs_Vectors.Vector;  --  这一集里点名那块在腕眼里的历次观测(位姿 + 像素)
      Geo_Slot : Integer := -1;
      Geo_Name : Unbounded_String;        --  这些观测是哪件【点过名的东西】的(按名字记,不按槽:近处重新指一次会换槽,远处那几眼好观测不能因此作废)
      --  它最后一次被量到的世界位置(视线交点 / 我自己挪过的几眼)。手贴近时它在腕眼里糊了、被切了,脑指不出 ⇒ 凭这个走(前提是它没动,并如实说)
      Geo_Pw : Geom.V3 := [others => 0.0];
      Geo_Pw_Up_Sd : Long_Float := Long_Float'Last;   --  那个位置沿"上"有多不准(两眼交点的几何按各眼的误差算,Geom.Meet_Sd;量不出 = 最大)
      --  (10-01 路 4 加)那个位置沿当时走的方向有多不准(同 Meet_Sd);它朝我这边的半径(手上那只眼里的框按远近折的;量不出 = 最大)。
      --  看不全它的那几步凭这两样定"可能碰到它的那条带子"(Selfmap.Plan_Approach)
      Geo_Pw_Sd : Long_Float := Long_Float'Last;
      Geo_R_Obj : Long_Float := Long_Float'Last;
      Geo_Pw_Valid : Boolean := False;
      Geo_Pw_Name : Unbounded_String;
      --  我最后一次被一个面顶住的地方:面上的一点(指尖世界位置)和它的法向(指向我这边)。
      --  不动的眼只给方向不给远近;东西躺在它靠着的面上 ⇒ 视线和这个面一交就是它在哪(LAB D2:碰过的点进地图)。
      --  ── 接触集(PLAN 1.5)──
      --  每次这只眼看全了它、它的位置又是两眼交出来的,就把它顶面的点记一份:轮廓像素各发一条视线落到它躺的面上(Contact.Surface.On_Plane)
      Sil_Pts : Contact.V3_Vectors.Vector;
      Sil_Valid : Boolean := False;
      Sil_Name : Unbounded_String;
      Sil_Cam : Integer := -1;
      Sil_N : Geom.V3 := [0.0, 0.0, 1.0];    --  取点时用的面法向(碰过的面按量到的,没碰过按上)
      Sil_P0 : Geom.V3 := [others => 0.0];   --  面过的那一点 = 它量到的位置
      Sil_Pitch : Long_Float := 0.0;         --  这份点的采样间距(米)
      Sil_Err : Long_Float := 0.0;           --  这份点的预期误差(米)= 那只眼量朝向时的像素残差 ÷ 焦距 × 眼到面的距离:几只眼都看全了它就留误差最小的那份
      Sil_H_Sd : Long_Float := Long_Float'Last;   --  这份点落的那张面(过它量到的位置)高低有多不准 = 那个位置沿"上"的不准(Geo_Pw_Up_Sd);Sil_Err 只管轮廓横着的误差
      Sil_Rays : Geom.Sight_Vectors.Vector;  --  出这份点的那些视线:碰到它躺的面之后按真高度重投一遍(取点时面的高度可能只是交点估的)
      --  合上时交出去的那个接触集(手里东西的接触点 + 锥);拿住之后锥放开(拿住 = 摩擦够,这就是身体量 μ 的办法)
      Held_Set : Contact.Set;
      Held_Set_Valid : Boolean := False;
      --  拿住那一刻它的实心模型(顶面轮廓往下补到它躺的面,同接触集)和这只手的位姿:之后它在哪 = 手从那一刻起挪过的刚体运动带着它走
      --  (拿住 = 抬一点它跟着手走,量过的;大并行路 5,10-01,加法)
      Held_Shape : Contact.V3_Vectors.Vector;
      Held_Pose : Plug.Arm_Pose := [others => 0.0];
      Grip_Mus : Grip_Mu_Vectors.Vector;     --  每件东西量到的摩擦上下限(见 Grip_Mu)
      --  脑这一轮要这件东西怎么动(大并行路 5,10-01,加法:I5 的一小块;Round 按脑说的"它的哪个量往哪变"填,接触集按它布置;没说 ⇒ 按"跟着手离开它躺的面")
      Want_Move : Contact.Want;
      --  I5:这一句脑要的那几个(见 Want)。今天 Round 按脑说的语言的根那一句填一个(Rel = Re_Qty、Ref = 0);两件东西那一句等路 7 从语言填 Ref
      Wants : Want_Vectors.Vector;
      Walls : Wall_Vectors.Vector;           --  这一集里各条臂横着被顶住过的地方(见 Wall_Mark)
      No_Reach_Arm : Integer := -1;          --  这一集里"它身上一段都在够不着那侧"的那条臂(-1 = 没有):下次选手绕开它
      Fingers_Aimed : Boolean := False;   --  上一段"到它上方"末尾已把手指指向它躺的面 ⇒ 接下来贴上去的那一段不再为了看它而转手
      Touch_Valid : Boolean := False;
      Touch_Pt, Touch_N : Geom.V3 := [others => 0.0];
      Touch_Fresh : Boolean := False;        --  这张面是这一集里碰出来的(False = 上一集留下的,新一集第一次朝下被顶住就换成新的,再往后只让更低的换)
      Bumps : Contact.V3_Vectors.Vector;     --  这一集里朝下被顶住、却比它躺的面高的地方(躺在面上的别的东西,或它自己):记进地图,不当成面
   end record;
   --  不动的眼每轮核一次(V1:被转了、被挡了一半 ⇒ 身体自己发现、重新标、接着干):Fixed_Ref → 此刻的图,仪器把板上的点配过来 ⇒ Geom.Check_Fixed。
   --  挪过 ⇒ 位姿换成按板重解的、说出来、存几何文件;挡住一大块 ⇒ 说出来(位姿照旧)。没配仪器、没有板 ⇒ 不核(量不出来)
   procedure Check_Fixed_Eye (F : Plug.Frame; C : in out Context);
   --  标定板随身体文件存、随身体文件装回(<几何文件>.board.txt + .board_ref.bmp):下一次开机装回身体时板和参考图也回来,每轮核对照常;
   --  两次开机之间相机被挪过 ⇒ 第一轮核对就发现、重标
   procedure Board_Save (C : Context);
   --  朝下被顶住的一点进地图(有板的面时只和它对账、不换它);开机碰桌面时在板上找一块空的面(压的那一瓣和别的瓣落点连成的几段 R 之内没有高出面的板点)。
   --  导出只为自检
   procedure Note_Support (C : in out Context; P, N : Geom.V3; How : String);
   --  身体的尺子(④):第一只碰桌面量过指尖的手,眼到两瓣指尖中点的距离(世界单位;没量过 = 0);给脑的长度按它说("X hand-lengths")
   function Hand_Len (C : Context) return Long_Float;
   function Len (C : Context; X : Long_Float) return String;
   --  这条臂第 K 个抓握通道带不带手指 = 开机推到头时有没有哪台相机量出了握区(看见东西跟着动);没带手指的通道不列手指 / 爪心、不说"你的手指之间"。
   --  Arm_Has_Fingers = 这条臂上有没有;Any_Fingers = 这具身体上有没有(= 进不进"只说东西的量"那种说法)。导出给自检
   function Has_Fingers (C : Context; Arm : Natural; K : Natural := 0) return Boolean;
   function Arm_Has_Fingers (C : Context; Arm : Natural) return Boolean;
   function Any_Fingers (C : Context) return Boolean;
   --  RGB 图顺时针转 90°(W×H → 宽 H、高 W):原图的 (u, v) 落到新图的 (H − 1 − v, u)。量仪器转着看时配得多细用;导出只为自检
   function Turn_90 (Img : Buf; W, H : Natural) return Buf;
   --  原图(宽 W、高 H)顺时针转了 Turns 个 90° 之后那张图里的 (U, V) 换算回原图的像素。核对时画面可能被转了,转回去配完再换算回来;导出只为自检
   procedure Unturn (U, V : Long_Float; Turns, W, H : Natural; U0, V0 : out Long_Float);
   procedure Board_Free_Spots (C : Context; Lp : Geom.V3_Vectors.Vector; Tb : Floats; R : Long_Float; Deltas : out Geom.V3_Vectors.Vector);
   --  压之前先看底下要问的点(导出给离线工具):Kinem 那张格点,再加压的那一瓣的落点圈(Spot 为心、半径 R)和到 Far_Ends 里每一处(别的瓣的落点)
   --  的带子(同 Board_Free_Spots 挡的那几处)里按 Step_Px 像素的间距(这一瓣尖那一截的厚:比指尖还窄的东西才可能漏)铺的点,
   --  间距按眼离面多高折成世界里的长度、铺在面上、投回位姿 P0 那一帧。手指像素照样问:手指长在眼上、两帧里不动,两条视线平行,
   --  交不出点、或者交在面下面很远(不确定度也大),判不成高出面。09-30 V1B77:原来按握区的手指像素不问 —— 那是张开到合上扫过的一整片,
   --  两根手指中间也在里面,电扇正好在那儿、一个点都没问,手往下一压,两根手指中间先顶在电扇上
   function Look_Points (C : Context; G : Geom.Cam_Geo; P0 : Plug.Arm_Pose; W, H : Natural;
                         Spot : Geom.V3; Far_Ends : Geom.V3_Vectors.Vector; R, Step_Px : Long_Float) return Instrument.Match_Vectors.Vector;
   --  压之前先看底下(09-30):同一只眼两个位姿 P0 → P1 各一帧(W × H),问的点 (Qu, Qv) 配到 (Mu, Mv)、配回来落在 (Bu, Bv)(< 0 = 配不回来)
   --  ⇒ 比面高出的点(Above:世界位置 + 协方差)。Matched = 配上的(往返 1 px 以内、落在画面里),Tri = 其中两条视线交成、两帧对得上的;
   --  Sig = 这一批的配点噪声(像素,每轴)。导出只为自检
   --  On 给了 ⇒ 交成、躺在面上(离面不过 Plane_Tol)的点也追加进去(世界位置 + 协方差;够不够细由调用方按它的门挑)
   procedure Seen_Above_Of (C : Context; G : Geom.Cam_Geo; P0, P1 : Plug.Arm_Pose; W, H : Natural; Qu, Qv, Mu, Mv, Bu, Bv : Floats;
                            Above : out Geom.Scene_Pt_Vectors.Vector; Matched, Tri : out Natural; Sig : out Long_Float;
                            On : access Geom.Scene_Pt_Vectors.Vector := null);
   --  板上的点在不动的眼此刻的画面里重找一遍(往返配,同核对不动的眼)⇒ C.Board_Seen;没有不动的眼 / 没配仪器 / 没配成 ⇒ Board_Seen 不动,Said 说为什么
   procedure Board_Recheck (F : Plug.Frame; C : in out Context; Found : out Natural; Said : out Unbounded_String);
   --  读数 R 离"空手合"那头往张开那头走了多远(方向按开机量的两头,不假设读数变小 = 合)。导出只为自检
   function Past_Empty (H : Zone.Hand; R : Long_Float) return Long_Float;

   procedure Init_Tracks (C : in out Context);
   --  开机后半段的几何从前半段来(V1b 09-27):每台相机一份(Geo:手上那只眼 = 运动学量的焦距、主点,插头给的手的位姿就是它的位姿 ⇒ 不转、不偏;
   --  不动的眼 = 对齐量的,世界系)、标定板(Board:放进世界的手三角出、配进不动的眼的点)、世界系的桌面(Plane_*)、不动的眼那一刻的画面(Ref:
   --  板上的点在它里面的像素就是在这张图里配的,每轮核对拿它比)。这几样前半段量过,后半段不再量(朝向、不动的眼);指尖、步幅后面照量。
   --  长度单位 = 第一只手运动学的单位(从零量的每一炮不一样 ⇒ 不装回上一炮的几何文件;前半段按身体文件装回的是同一个单位 ⇒ Keep_Tips)
   procedure Geo_Install (F : Plug.Frame; C : in out Context; Body_Path : String; Geo : Geom.Geo_Vectors.Vector; Board : Geom.Scene_Pt_Vectors.Vector;
                          Plane_Pt, Plane_N : Geom.V3; Plane_Rms : Long_Float; Ref : Plug.Cam; Keep_Tips : Boolean := False);
   procedure Geo_Boot_Support (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context);
   --  ④ 每条臂一条命令能走多远还走得到(阶梯探)
   --  这只手一条命令转得到的最大一档(弧度):按运动学在量到的关节限位里问反解(Plug.Reach,不动胳膊)—— 从位姿 P0 绕世界 x 轴转,
   --  从一档转动(Notch)起每次翻倍,反解位置还差不到 Tol_P、朝向还差不到 Tol_R ⇒ 转得到;第一次转不到就停、取上一档;最多到 π
   --  (转动向量过 π 是反方向的小转动,纯几何)。没有运动学 ⇒ 0(量不了)
   function Kin_Turn_Reach (Arm : Natural; P0 : Plug.Arm_Pose; Notch, Tol_P, Tol_R : Long_Float) return Long_Float;
   --  接触集(09-29 重写):量出来的手在记下的形状上挑一组下手处(导出只为自检)
   procedure Plan_Contact (C : in out Context; F : Plug.Frame; Arm, Cam : Natural; Name : Unbounded_String;
                           Pick : out Contact.Search.Candidate; Note : out Unbounded_String; Ok : out Boolean);
   --  手拿着它绕 M 的那根轴(过 M.Pivot)转 Th 弧度:手的位姿要到哪(位置绕那一点转过去、朝向转同一个角)
   function Carry_Goal (Cur : Plug.Arm_Pose; M : Contact.Twist; Th : Long_Float) return Plug.Arm_Pose;
   --  I5 ⇒ 要它怎么动(大并行路 5,10-01):量到的几何(Want_Scene:它的实心模型、它躺的面、不跟着这条臂走的那只眼、脑看着的那只眼、参照那一件的视线交点)
   --  ⇒ 让那个量变得最快的那个旋量(Want_Twist;量的名字按登记表 Qty_Kind,两件东西那一句按关系词)。Arm = 动它的那条臂(-1 = 还没定)
   procedure Want_Scene (C : in out Context; F : Plug.Frame; W : Want; Arm : Integer; Sc : out Contact.Qty.Scene);
   procedure Want_Twist (C : in out Context; F : Plug.Frame; W : Want; Arm : Integer; M : out Contact.Twist; Ok : out Boolean; Note : out Unbounded_String);
   --  接触集往下伸怎么走(纯函数,导出给自检):悬停时离下手处 Stand;最靠前的尖和它顶面那一层沿进场方向差 Tip_Over(= X_Tip − X_Top,≤ 0 就是尖还没到顶面那一层);
   --  顶面的不准 = 轮廓横着的 Sil_Err ⊕ 那张面高低的不准 H_Sd 在进场方向上的那一份(An = |进场方向 · 面法向|;H_Sd = Long_Float'Last 表示量不出);
   --  尖的不准 Tip_Sd、这一次到位还差 Miss、读数噪声 Noise;Floor = 身体量得出的最细那一档。
   --  交出:Band = 尖碰到它顶面那一层"可能早也可能晚"的那条带子的半宽(Stats.Z 倍合起来的不准;量不出 = Long_Float'Last);
   --  Lstep = 带子里一步 = 手自己的不准(尖、到位、读数,Stats.Z 倍)÷ Selfmap.Free_Base —— 比手自己的不准还细的步子多不出东西,粗了就丢准头;不细过 Floor;
   --  Fast = 带子之前先一条命令下多远(再留出 Blocked 当底的那几步;≤ Lstep 就不走这一条);
   --  Fine_End = 小步探到多深为止(过了顶面那一层的带子,尖就在它两边;再往下到下手处一条命令,Blocked 拿前面小步空走的当底判挡没挡)
   type Descent is record
      Band, Lstep, Fast, Fine_End : Long_Float := 0.0;
   end record;
   function Plan_Descent (Stand, Tip_Over, Sil_Err, H_Sd, An, Tip_Sd, Miss, Noise, Floor : Long_Float) return Descent;
   procedure Geo_Boot_Stride (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context);
   procedure Round (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context);
end Act;
