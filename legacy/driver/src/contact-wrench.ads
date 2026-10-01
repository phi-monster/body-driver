--  接触集的物理检查(大并行路 5,10-01;§2 第 16 条的前两件):这几处接触,连同东西躺的那张面,能不能让它照要的那样动、手要使多大的力 ——
--  一个线性规划,一段代码。要的动不一样,只是面怎么参与、力怎么配平不一样;代码里没有动作,只有一个旋量。
--  准静态(动得慢,惯性不算):手的接触力 + 面的反力 + 重力 = 0,力按单位重量。
--  手的接触跟着东西走(不滑):每处一个摩擦锥(线性化成几条棱),面接触还能绕自己的法向拧(拧的上限 = 摩擦系数 × 法向力 × 能拧的半径;
--  拧和横着的摩擦各占一份法向力,比真的椭圆锥保守,不会说出做不到的事)。
--  东西躺的面 = 一处接触,是一片(东西在面上的那一片:表面点投到面上,量的)。每一点按要的旋量算它的速度:
--   · 全都离开面(沿面的法向往外)⇒ 面不给力;
--   · 有一点往面里去 ⇒ 面挡着,做不到(照实说是面挡着);
--   · 贴着面的那些点托着它:法向压力可以落在它们的凸包里任何地方(压心走到边上它才会翻,刚体静力学);
--     在滑的点:摩擦跟它滑的方向相反、大小 = 摩擦系数 × 压力,压力按均匀分摊到贴着的每一点
--     (只许"有一个分布"会把整份压力压在转轴上那一点,转起来一点摩擦力矩都没有,那不是真的;摩擦也不是锥 —— 锥会让面帮着推它);
--     一个都没在滑(翻的那条边、它不动)⇒ 摩擦在锥里。
--  "还贴着 / 在转轴上"分得多细:那一片的采样间距(比它近的分不出来,量的)。
--  最少多少:只数手的法向力(面的反力不花手的力气)。不需要知道东西多重、手多有劲:力只比方向和比例(同接触集的第②格);
--  摩擦系数由调用方给(这只身体量到的),还单独给"最少要多大的摩擦才做得到"。
--  (09-29 的 Contact.Hold 只会一种要:托住重心处的重量,没有面。它的名字 Squeeze / Mu_Need / Load 留在这里,是同一个规划没有面时的那一种;
--  旧包名 Contact.Hold 10-01 合并时删了)
with Ada.Containers.Vectors;
package Contact.Wrench is
   --  手上的一处接触:在哪、手往东西里推的方向(单位向量,朝东西里面)、能拧的半径(世界单位;0 = 点接触)
   type Touch is record
      P : V3 := [others => 0.0];
      N : V3 := [0.0, 0.0, 1.0];
      Twist_R : Long_Float := 0.0;
   end record;
   package Touch_Vectors is new Ada.Containers.Vectors (Natural, Touch);
   --  东西躺的那张面(一处接触):面的法向(朝东西那边)、东西在面上的那一片(面上的点)、那一片的采样间距
   type Surface is record
      Present : Boolean := False;
      Up : V3 := [0.0, 0.0, 1.0];
      Foot : V3_Vectors.Vector;
      Pitch : Long_Float := 0.0;
   end record;
   No_Surface : constant Surface := (others => <>);
   --  东西在面上的那一片:表面点投到面上(面过 P0、法向 Up),按采样间距稀疏成一格一个点(格里取平均)
   function Footprint (Pts : V3_Vectors.Vector; P0, Up : V3; Pitch : Long_Float) return Surface;
   No_Way : constant Long_Float := Long_Float'Last;
   --  做不到的时候是哪一条:面挡着(要的动往面里去)/ 这几处接触加上面怎么配都配不平
   type Why_Kind is (Fine, Surface_In_Way, Unbalanced);
   --  它要照 M 那样动(只看方向和绕哪儿转;多快不管,准静态),重心 Com,重力朝 −Up(按单位重量);
   --  手的接触 Ts(摩擦系数 Mu_Hand)、它躺的面 Sup(摩擦系数 Mu_Surf)⇒ 手的法向力之和最少多少(每单位重量);做不到 ⇒ No_Way,Why 说是哪一条
   function Need (Ts : Touch_Vectors.Vector; Com, Up : V3; Sup : Surface; M : Twist; Mu_Hand, Mu_Surf : Long_Float; Why : out Why_Kind) return Long_Float;
   --  同一个规划,要的不是配平重力,而是这几处接触(加上面)一起产生力 F、绕 Ref 的力矩 Mo(按单位重量)。
   --  Align:每处接触摩擦锥的第一条棱对准它在接触面上的那一份(线性化的锥只在那个方向上是准的;没有就任取)
   function Least (Ts : Touch_Vectors.Vector; Ref, F, Mo, Align : V3; Sup : Surface; M : Twist; Mu_Hand, Mu_Surf : Long_Float; Why : out Why_Kind) return Long_Float;
   --  它底下贴着面的那一片(放下、摞上去之前问):它的点里最低的那一层 —— 最低点往上一个采样间距以内的那些点(点按这个间距铺,
   --  比它近的高低分不出)—— 投到过最低点、法向 Up 的那张面上(Footprint)。点空 / 间距不是正数 ⇒ No_Surface
   function Base_Of (Pts : V3_Vectors.Vector; Up : V3; Pitch : Long_Float) return Surface;
   --  单靠下面那张面托不托得住它(松手之前问,放下、摞上去同一条;大并行路 5,10-01 主代理批的):Sup = 它底下贴着那张面的那一片,重力在 Com、朝 −Up;
   --  手不碰它、它不动(Still)—— 就是 Need,手一处都没有。重心按量到的不准挪:沿那一片凸包每条边的外法向各挪 Stats.Z 倍 Com_Sd,
   --  每一种都托得住才算托得住(凸的那一片:半径 Z 倍不准的圆整个在里面 ⇔ 圆心朝每条边的外法向各挪这么远都还托得住)。
   --  面的摩擦按 Mu(不知道就给 0:不靠摩擦也托得住才算)。
   --  Margin = 重心离那一片凸包最近的那条边多远(在面里量,里面为正、外面为负);那一片不到三个不共线的点(没有面积)⇒ Ok = False、Margin = 负无穷
   procedure Rests (Sup : Surface; Com, Up : V3; Com_Sd, Mu : Long_Float; Ok : out Boolean; Margin : out Long_Float);
   --  没有面时要这几处接触一起产生的:力 F(按单位重量;托住 = 抵掉重力 ⇒ 朝上 1)作用在 C,另加转矩 M
   type Load is record
      F : V3 := [0.0, 0.0, 1.0];
      C : V3 := [others => 0.0];
      M : V3 := [others => 0.0];
   end record;
   --  没有面:按摩擦系数 Mu 产生 L 要的全部接触法向力之和的最小值;做不到 ⇒ No_Way(第一条棱对准 L.F)
   function Squeeze (Ts : Touch_Vectors.Vector; L : Load; Mu : Long_Float) return Long_Float;
   --  没有面:最少要多大的摩擦才做得到(不管夹多紧):摩擦系数从小翻倍找到做得到的那一档、再二分;翻到 Mu_Top 还做不到 ⇒ No_Way
   --  (比如几处接触都在一边、只能推不能夹)
   Mu_Top : constant := 64.0;   --  找摩擦系数的上限(次数的上限:从 1/64 翻倍 12 次;真实的摩擦系数大都在 0.1–2)
   function Mu_Need (Ts : Touch_Vectors.Vector; L : Load) return Long_Float;
   --  线性规划本身(导出给自检):最小化 Σ Cost_j · x_j、A x = B、x ≥ 0;A 是 Rows × Cols(按行排成一串);Cost 不给 = 每个变量都算 1。做不到 ⇒ Ok = False
   type Real_Array is array (Natural range <>) of Long_Float;
   No_Cost : constant Real_Array (1 .. 0) := [others => 0.0];
   procedure Min_Sum (A : Real_Array; Rows, Cols : Positive; B : Real_Array; Obj : out Long_Float; Ok : out Boolean; Cost : Real_Array := No_Cost);
end Contact.Wrench;
