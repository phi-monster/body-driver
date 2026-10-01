--  接触集 —— 「脑对身体说的那句话」本身,四格齐(08-15 架构 §1.2 原文):
--  ① 碰物体表面的哪几个点 · ② 每点的法向,和那里允许往哪使劲的【锥】(只给方向,不给大小)
--  · ③【物体】要怎么动(一个旋量)· ④ 每点的容差(碰到的地方毫米级,只是路过的地方厘米级)。
--  这里一个字不提机体:吸盘 = 1 个点 + 只允许法向的锥;两指 = 2 点;三指 = 3 点;五指 = 5 点。谁来执行是身体层的事。
--  力只给方向不给大小:一个苹果要多少牛顿是世界属性;这条胳膊命令一下产出多少牛顿是身体属性,随电量/磨损漂 ⇒ 第②格只有锥,没有牛顿。
--  2026-08 用 Rust 写成、十三个动词逐个验过(commit ef10664:contact-set/src/lib.rs + many.rs),09-07 重写成 Ada 时丢了。
--  这里逐段搬回来,判据一个没改;每一条当年的单元测试都焊进 selfcheck。
with Geom;
with Ada.Containers.Vectors;
package Contact is
   subtype V3 is Geom.V3;
   use type Geom.V3;
   function Dot (A, B : V3) return Long_Float;
   function Cross (A, B : V3) return V3;
   function Norm (A : V3) return Long_Float renames Geom.Norm;
   function Unit (A : V3; Ok : out Boolean) return V3;   --  模太小或不是数 ⇒ Ok = False(那不是一个方向)

   --  ② 的后一半:这一点上允许往哪使劲。只有方向 —— 一个轴 + 一个半张角,没有牛顿。
   --  半张角 0 ⇒ 只准沿轴(吸盘:只能沿法向吸,不能侧推);π/2 ⇒ 半空间;摩擦锥的半张角 = atan(μ),μ 谁量到谁填。
   type Cone is record
      Axis : V3 := [0.0, 0.0, 1.0];
      Half_Angle : Long_Float := 0.0;
   end record;

   --  ① 这个接触是谁跟物体之间的。
   --  手:执行层要去访问它,它变成航点。带编号 —— 一只五指手的五个点共一个手腕、一个朝向;双臂抱一个箱子是两只手腕,合成一个朝向没有意义。
   --  编号不带语义(0 不必是"左手"),只说"这些点归同一个执行器管"。
   --  世界:桌面、墙、卡具、另一只手按住的地方。执行层不访问它(手够不到桌子底下那条边),但它参与"能不能驱动"的计算 ——
   --  撬:手只有一个点,单个接触力产生不出③要的纯力矩;少的那一个接触是桌子给的反力,它一直都在,只是以前没地方记。
   type Who_Kind is (Hand, World);
   type Who is record
      Kind : Who_Kind := Hand;
      Id : Natural := 0;
   end record;

   --  ①②④ 合起来:一个接触点。
   type Point is record
      By : Who;
      Pos : V3 := [others => 0.0];       --  ① 碰物体表面的哪儿(世界系,米)
      Normal : V3 := [others => 0.0];    --  ② 那一点的表面法向,指向物体外侧
      --  (Push / Pull 这两个字段名是八月的旧名,主代理的 selfcheck.adb 执行层焊点按完整的具名聚合在用;合并时改成 Allowed / Tension)
      Push : Cone;                       --  ② 那一点允许往哪使劲
      --  ② 这个接触能传哪几种东西(都是身体属性,要量;谁填谁负责,别默认 True 混过去):
      Pull : Boolean := False;           --  能不能传拉力(真空吸盘/电磁/胶带能;手指往外一使劲就离开表面了)。少了它,吸盘吸住了也离不开面
      Torsion : Boolean := False;        --  绕自己的法向扭不扭得动(指腹是一片面 ⇒ 能;硬针尖 ⇒ 不能)。少了它,两指捏着勺子兜起来在静力学上直接判死
      Peel : Boolean := False;           --  绕切向轴掰不掰得动(抗不抗剥离:吸盘/胶垫/大贴片能;点接触不能)。少了它,吸盘只转得动、翻不动
      Tol_M : Long_Float := 0.0;         --  ④ 这一个点的容差(米);每点各一个,不是每个计划一个
   end record;
   package Point_Vectors is new Ada.Containers.Vectors (Natural, Point);

   --  ③ 物体要怎么动 —— 一个旋量:平移 + 绕某轴转;绕轴转必须说清绕哪一点。
   --  撬是绕物体贴着支撑面的那条边,拧是绕物体自己的轴 —— 同样的角速度,支点不同,结果完全不同。
   type Twist is record
      Lin : V3 := [others => 0.0];       --  平移,米
      Ang : V3 := [others => 0.0];       --  轴角:方向 = 转轴,模长 = 转多少弧度;零向量 = 不转
      Pivot : V3 := [others => 0.0];     --  绕哪一点转(世界系,米);Ang 为零时它不起作用
   end record;
   function Still (Pivot : V3) return Twist;                   --  什么都不动。"物体不动"是一个合法的答案,不是缺省值
   function Slide (Lin : V3) return Twist;                     --  纯平移
   --  绕 Pivot 的一条轴转 Rad;轴不是方向 ⇒ Ok = False(旧名:主代理的 selfcheck.adb 执行层焊点在用;合并时改成 Rotation)
   function Turn (Axis : V3; Rad : Long_Float; Pivot : V3; Ok : out Boolean) return Twist;
   function Angle (T : Twist) return Long_Float;               --  转多少弧度
   function Moving (T : Twist) return Boolean;                 --  平移或转,任一个不为零
   function Apply (T : Twist; P : V3) return V3;               --  把一个世界点按这个旋量搬过去:先绕 Pivot 转,再整体平移(罗德里格斯)

   --  脑要这件东西怎么动(I5 的一小块:路 7 从语言填、路 5 读;这一版由 Round 按脑说的"它的哪个量往哪变"填):
   --  Given = 说了;没说 ⇒ 接触集按"它跟着手离开它躺的面"布置(合上以后抬一点验它跟不跟手,验的就是这个)。Move 只看方向和绕哪儿转
   type Want is record
      Given : Boolean := False;
      Move : Twist;
   end record;

   --  一个接触集 —— 四格齐。里面只有物体,没有身体。
   type Set is record
      Points : Point_Vectors.Vector;     --  ①②④,≥1 个(吸盘 1、两指 2、三指 3、五指 5);"≥2"那句话是对"抓"说的,不是对这个结构说的
      Motion : Twist;                    --  ③
      --  四格定不下来的那一个自由度:手从哪个方向进场。单点接触(推/压/吸盘)时锥轴就是进场方向,这里可以不填;
      --  对夹时两个锥正好相反、合成为零,剩下的约束只有"⊥ 接触点连线",那是一整圈 ⇒ 由看得见空隙的那一层填。填不了就拒绝,不许在执行层瞎挑。
      Has_Approach : Boolean := False;
      Approach : V3 := [others => 0.0];
   end record;

   --  一个接触集哪儿填不下去。必须点名是哪一格,不许含糊。
   type Gap_Kind is
     (Fine,
      No_Points,        --  ① 一个接触点都没有
      Bad_Normal,       --  ② 某一点的法向不是一个方向(零向量 / 不是数)
      Bad_Cone,         --  ② 某一点的锥轴不是一个方向,或半张角不在 [0, π]
      Motion_Still,     --  ③ 旋量既不平移也不转,而这个动词要求物体动
      No_Pivot,         --  ③ 要转,但绕哪一点不是数
      Bad_Tolerance);   --  ④ 某一点的容差不是一个正数
   type Gap is record
      Kind : Gap_Kind := Fine;
      Index : Natural := 0;              --  哪一点(Bad_Normal / Bad_Cone / Bad_Tolerance)
   end record;
   function Img (G : Gap) return String;
   --  逐格自检:每一格填得像不像样(法向、锥、容差、要动时旋量不为零、转时绕的那一点是数)。Must_Move:要不要求物体动。
   --  "这几处接触能不能让它照要的那样动"不在这儿判:那是物理检查(Contact.Wrench,连同它躺的面、按重力配平),一个量一种量法
   --  (八月那一版"需要的力旋量方向 ∝ 要的旋量"的判据不算重力、不算面,10-01 删了)
   function Check (S : Set; Must_Move : Boolean) return Gap;

   --  ── 一个接触集说不完的三件事:一串 · 并存 · 过渡 ──
   --  一段不够(来回若干道;插进去 → 兜起来 → 离开)⇒ In_Order(段间重新下手,过渡由执行层自己产生)或 Keep(不松手);
   --  要同时成立(握住 + 扣扳机)⇒ Meanwhile:第一段维持(第③格必须不动),其余在动;
   --  "不要碰"(躲拳)⇒ Clear:零接触点 + 一个净空;
   --  物体不参与(够/Reach)⇒ 不进接口:每段开头的"悬停"就是它,由执行层自己产生。给它一个变体等于让脑去操心手怎么绕过去。
   type Move_Kind is (One, In_Order, Keep, Clear, Meanwhile);
   package Nat_Vectors is new Ada.Containers.Vectors (Natural, Natural);
   package V3_Vectors is new Ada.Containers.Vectors (Natural, V3);
   --  树存成一张表:每个节点记自己的种类和子节点的编号(Ada 里递归的变体要么走访问类型,要么走编号;这里走编号)。
   type Node is record
      Kind : Move_Kind := One;
      S : Set;                           --  One
      Items : Nat_Vectors.Vector;        --  In_Order / Keep / Meanwhile:子节点编号,按先后
      Keep_Out : V3_Vectors.Vector;      --  Clear:要躲开的那些地方(世界系,米)
      By_M : Long_Float := 0.0;          --  Clear:至少要留多宽(米)
      From : V3_Vectors.Vector;          --  Clear:手上那些点现在在哪(由调用方给:身体层知道手在哪,这一层不知道,也不该知道)
   end record;
   package Node_Vectors is new Ada.Containers.Vectors (Natural, Node);
   type Move is record
      Nodes : Node_Vectors.Vector;
      Root : Natural := 0;
   end record;
   package Move_Vectors is new Ada.Containers.Vectors (Natural, Move);

   --  一串 / 并存填不满时,点名是第几段的哪一格。
   type Many_Kind is
     (Fine,
      Inside,                      --  里面某一段自己就填不满 —— 带上是第几段(Path)、哪一格(G)
      Empty,                       --  In_Order / Keep / Meanwhile 里一段都没有
      Holder_Moves,                --  Meanwhile 的第一段不是"维持":它的第③格在动
      Nothing_To_Pair_With,        --  Meanwhile 只有一段 ⇒ 没有"并存"可言
      Keep_Breaks_Contact,         --  Keep 说"不松手",而下一段的接触点不在上一段末了那个位置上(Seg = 第几段,Off_M = 差多少米)
      Keep_Changes_Point_Count,    --  Keep 前后两段的接触点个数不一样 —— 不松手不可能换手指数
      No_Keep_Out,                 --  Clear 一个要躲的地方都没给
      Bad_Clearance);              --  Clear 的净空不是正数,或"手现在在哪"没给
   type Many_Gap is record
      Kind : Many_Kind := Fine;
      Path : Nat_Vectors.Vector;   --  出事的是哪一段:从根往下每一层的编号
      G : Gap;
      Seg : Natural := 0;
      Off_M : Long_Float := 0.0;
   end record;
   function Img (M : Many_Gap) return String;
   function Moves (M : Move) return Boolean;                        --  这条计划里有没有任何一段要求物体动(躲开是手在动,不是物体 ⇒ False)
   function Start_Points (M : Move) return V3_Vectors.Vector;       --  开头那些手接触点在哪(世界接触不算:手够不到桌子底下那条边)
   function End_Points (M : Move) return V3_Vectors.Vector;         --  末了那些手接触点在哪:起点让第③格搬过去;Meanwhile 停在维持的那一段上
end Contact;
