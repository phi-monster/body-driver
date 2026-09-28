--  接触集重写 ⑤(09-29,owner:"接触集要改就一次改好,别给未来埋雷"):一把下手处好不好,只用一个物理量说 ——
--  按脑要的动作,这几处接触要产生那个力旋量,所有接触的法向力加起来最少要多少(每单位重量)。夹得越松越好。
--  不需要知道东西多重、手多有劲、摩擦多大:力只比方向和比例(同接触集的第②格);摩擦系数由调用方给(这只手量到的下限),
--  还单独给"最少要多大的摩擦才做得到"。
--  接触:摩擦锥(线性化成若干条棱)+ 面接触能绕自己的法向拧(拧的上限 = 摩擦系数 × 法向力 × 接触面半径;半径 0 = 点接触);
--  拧和横着的摩擦各占一份法向力(比真的椭圆锥保守,不会说出做不到的事)。
with Ada.Containers.Vectors;
package Contact.Hold is
   --  一处接触:在哪、手往东西里推的方向(单位向量,朝东西里面)、能拧的半径(世界单位;0 = 点接触)
   type Touch is record
      P : V3 := [others => 0.0];
      N : V3 := [0.0, 0.0, 1.0];
      Twist_R : Long_Float := 0.0;
   end record;
   package Touch_Vectors is new Ada.Containers.Vectors (Natural, Touch);
   --  要这几处接触一起产生的:力 F(按单位重量;托住 = 抵掉重力 ⇒ 朝上 1)作用在 C(重心),另加转矩 M
   type Load is record
      F : V3 := [0.0, 0.0, 1.0];
      C : V3 := [others => 0.0];
      M : V3 := [others => 0.0];
   end record;
   No_Way : constant Long_Float := Long_Float'Last;
   --  最少要夹多紧:按摩擦系数 Mu,产生 L 要的全部接触法向力之和的最小值;做不到 ⇒ No_Way。
   --  线性规划(两阶段单纯形):每条棱 / 每个拧的方向是"一份单位法向力能产生的力旋量",求非负组合、法向力之和最小
   function Squeeze (Ts : Touch_Vectors.Vector; L : Load; Mu : Long_Float) return Long_Float;
   --  最少要多大的摩擦才做得到(不管夹多紧):摩擦系数从小翻倍找到做得到的那一档、再二分;翻到 Mu_Top 还做不到 ⇒ No_Way
   --  (比如几处接触都在一边、只能推不能夹)
   Mu_Top : constant := 64.0;   --  找摩擦系数的上限(次数的上限:从 1/64 翻倍 12 次;真实的摩擦系数大都在 0.1–2)
   function Mu_Need (Ts : Touch_Vectors.Vector; L : Load) return Long_Float;
   --  线性规划本身(导出给自检):最小化 Σ x、A x = B、x ≥ 0;A 是 Rows × Cols(按行排成一串)。做不到 ⇒ Ok = False
   type Real_Array is array (Natural range <>) of Long_Float;
   procedure Min_Sum (A : Real_Array; Rows, Cols : Positive; B : Real_Array; Obj : out Long_Float; Ok : out Boolean);
end Contact.Hold;
