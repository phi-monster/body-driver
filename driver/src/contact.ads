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
   function Admits (K : Cone; Dir : V3) return Boolean;   --  这个方向在不在锥里:判据是角度,不是力

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
      Push : Cone;                       --  ② 那一点允许往哪使劲
      --  ② 这个接触能传哪几种东西(都是身体属性,要量;谁填谁负责,别默认 True 混过去):
      Pull : Boolean := False;           --  拉不拉得动(真空吸盘/电磁/胶带能拉;手指一拉就离开表面了)。少了它,吸盘吸住了也抬不起任何东西
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
   function Still (Pivot : V3) return Twist;                   --  什么都不动:压/敲/抓的第③格。"物体不动"是一个合法的答案,不是缺省值
   function Slide (Lin : V3) return Twist;                     --  纯平移
   function Turn (Axis : V3; Rad : Long_Float; Pivot : V3; Ok : out Boolean) return Twist;   --  绕 Pivot 的一条轴转 Rad;轴不是方向 ⇒ Ok = False
   function Angle (T : Twist) return Long_Float;               --  转多少弧度
   function Moving (T : Twist) return Boolean;                 --  平移或转,任一个不为零
   function Apply (T : Twist; P : V3) return V3;               --  把一个世界点按这个旋量搬过去:先绕 Pivot 转,再整体平移(罗德里格斯)

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
      Cannot_Drive,     --  ② 所有接触加在一起都产生不出③要的那个力旋量:你说只能这样使劲,又说物体要那样动
      Motion_Still,     --  ③ 旋量既不平移也不转,而这个动词要求物体动
      No_Pivot,         --  ③ 要转,但绕哪一点不是数
      Bad_Tolerance);   --  ④ 某一点的容差不是一个正数
   type Gap is record
      Kind : Gap_Kind := Fine;
      Index : Natural := 0;              --  哪一点(Bad_Normal / Bad_Cone / Bad_Tolerance)
   end record;
   function Img (G : Gap) return String;
   --  逐格自检。Must_Move:这个动词要不要求物体动(压/敲/抓不要求,推/撬/拧要求)。
   function Check (S : Set; Must_Move : Boolean) return Gap;
   --  所有接触的摩擦锥张成的凸锥,包不包含③要的那个力旋量方向。
   --  判据必须在【集合】上,不在单点上:两指捏着横向搬运,任何单指都做不到(切向力出了自己的摩擦锥),但两指一起可以 ——
   --  内部的对夹力互相抵消,切向的摩擦力叠加。逐点判会把「放」「反例」两条合法接触集判死(2026-08-16 实测)。
   --  建模:准静态,需要的力旋量方向 ∝ ③的旋量;参考点取接触点质心(物体质心的代理,不是 Pivot:拿着东西挥的时候合力本来就不为零,那个力是胳膊给的);
   --  力与力矩用特征长度 L(各接触到参考点的平均距离)配平;只判方向在不在锥里,不判大小(大小是"捏多紧",归执行层)。
   function Can_Drive (S : Set) return Boolean;

   --  ── 一个接触集说不完的三件事:一串 · 并存 · 过渡 ──
   --  一段不够(擦:来回若干道;舀:插进去 → 兜起来 → 抬出来)⇒ In_Turn(段间重新下手,过渡由执行层自己产生)或 Keep(不松手);
   --  要同时成立(握住 + 扣扳机)⇒ Meanwhile:第一段维持(第③格必须不动),其余在动;
   --  "不要碰"(躲拳)⇒ Clear:零接触点 + 一个净空;
   --  物体不参与(够/Reach)⇒ 不进接口:每段开头的"悬停"就是它,由执行层自己产生。给它一个变体等于让脑去操心手怎么绕过去。
   type Move_Kind is (One, In_Turn, Keep, Clear, Meanwhile);
   package Nat_Vectors is new Ada.Containers.Vectors (Natural, Natural);
   package V3_Vectors is new Ada.Containers.Vectors (Natural, V3);
   --  树存成一张表:每个节点记自己的种类和子节点的编号(Ada 里递归的变体要么走访问类型,要么走编号;这里走编号)。
   type Node is record
      Kind : Move_Kind := One;
      S : Set;                           --  One
      Items : Nat_Vectors.Vector;        --  In_Turn / Keep / Meanwhile:子节点编号,按先后
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
   function One_Of (S : Set) return Move;
   function Chain (Kind : Move_Kind; Items : Move_Vectors.Vector) return Move;   --  Kind ∈ In_Turn / Keep / Meanwhile
   function Clear_Of (Keep_Out : V3_Vectors.Vector; By_M : Long_Float; From : V3_Vectors.Vector) return Move;

   --  一串 / 并存填不满时,点名是第几段的哪一格。
   type Many_Kind is
     (Fine,
      Inside,                      --  里面某一段自己就填不满 —— 带上是第几段(Path)、哪一格(G)
      Empty,                       --  In_Turn / Keep / Meanwhile 里一段都没有
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
   --  逐段自检。In_Turn / Keep 的每一段都按 Must_Move 判;Meanwhile 的第一段永远按"不动"判(它是维持的那一个),其余按 Must_Move。
   --  Keep 的接续条件:下一段的接触点就是上一段末了那些点,门槛用下一段自己声明的最严容差,不另拍一个常数。
   function Check (M : Move; Must_Move : Boolean) return Many_Gap;
   function Moves (M : Move) return Boolean;                        --  这条计划里有没有任何一段要求物体动(躲开是手在动,不是物体 ⇒ False)
   function Start_Points (M : Move) return V3_Vectors.Vector;       --  开头那些手接触点在哪(世界接触不算:手够不到桌子底下那条边)
   function End_Points (M : Move) return V3_Vectors.Vector;         --  末了那些手接触点在哪:起点让第③格搬过去;Meanwhile 停在维持的那一段上
   function Flatten (M : Move) return Nat_Vectors.Vector;           --  按时间先后的那些 One 节点编号(Meanwhile 摊平后仍然是并存的;摊平只用来数数与遍历)
end Contact;
