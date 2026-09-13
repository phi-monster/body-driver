--  通道响应表(效能矩阵):每个通道动一点,某个被跟踪的【那一块】在画面里五样各变多少:
--  左右 u · 上下 v · 远近(深度)· 看起来多大(块的边长 = √像素数)· 朝向(主轴角的两倍,避开正负两种写法)。
--  后两样是"区域和区域重合"要的:大小告诉远近(离得越近看着越大,比深度读数稳得多),朝向告诉转多少。
--  对圆的东西朝向本来就没有意义 ⇒ 那一行谁也改不动 ⇒ 自动不参与(不需要写规则)。
--  在线递推最小二乘(带遗忘)重估;两套模型的责任:空着走的表 vs "顶住了"的零表,谁预测准信谁 ⇒ 碰上/推不动从这里长出来。
--  解算:带上下限的加权最小二乘分配(飞控的控制分配),没有减半/放大/禁用这类规则。
with Ada.Containers.Vectors;
package Table is
   Max_Ch : constant := 64;
   subtype Ch_Index is Natural range 0 .. Max_Ch - 1;
   type Vec is array (Ch_Index) of Long_Float;
   Rows : constant := 5;    --  一块东西在画面里被量的五样(次数,无量纲)
   type Vec3 is array (0 .. Rows - 1) of Long_Float;
   type Mat3 is array (Ch_Index, 0 .. Rows - 1) of Long_Float;
   type Cov is array (Ch_Index, Ch_Index) of Long_Float;
   type Mask is array (Ch_Index) of Boolean;
   type Counts is array (Ch_Index) of Natural;
   Zero_Vec : constant Vec := [others => 0.0];
   Zero3 : constant Vec3 := [others => 0.0];

   type Effect is record
      N : Natural := 0;
      B : Mat3 := [others => [others => 0.0]];
      P : Cov := [others => [others => 0.0]];
      Lambda : Long_Float := 0.98;      --  遗忘因子(无量纲协议:近的样本比远的重)
      Free_Res, Null_Res : Long_Float := 0.0;
      Null_Wins : Natural := 0;
      Updates : Natural := 0;
      Last_Pred_Err : Long_Float := 0.0;
      --  同一个推法重复了几次(次数,无量纲),以及每(通道,行)的【散布 ÷ |均值|】(比例,无量纲)。
      --  一次推动只证明"它动过",证明不了"它稳"。体检只放行 重复≥2 且 散布 < 均值本身 的行。
      Reps : Counts := [others => 0];
      Scatter : Mat3 := [others => [others => 0.0]];
   end record;

   procedure Reset (E : in out Effect; N : Natural; P0 : Long_Float);
   procedure Set_Prior (E : in out Effect; Ch : Natural; P0 : Long_Float);   --  这一通道的先验不确定度(按它的命令量级定)
   procedure Set_Col (E : in out Effect; Ch : Natural; D : Vec3);
   procedure Set_Spread (E : in out Effect; Ch : Natural; N : Natural; S : Vec3);
   function Col (E : Effect; Ch : Natural) return Vec3;
   function Predict (E : Effect; A : Vec) return Vec3;
   procedure Update (E : in out Effect; A : Vec; Dy : Vec3; Motion_Floor, Cmd_Floor : Long_Float);
   function Blocked (E : Effect) return Boolean;       --  连着两步"零表"比"走的表"预测得准
   function Spread (E : Effect) return Long_Float;     --  协方差迹的平均:还有多不确定
   function Norm (A : Vec; N : Natural) return Long_Float;
   function Norm3 (V : Vec3) return Long_Float;

   type Term is record
      E : Effect;
      Err : Vec3 := [others => 0.0];
      W : Vec3 := [others => 1.0];
   end record;
   package Term_Vectors is new Ada.Containers.Vectors (Natural, Term);
   --  最小化 Σ w·|B a − e|² + μ|a|²,|a_k| ≤ cap_k,只动 Active 的通道。解不出来 Ok = False。
   --  Damp (k) = 这一通道每单位命令的阻尼(按它的探针幅度归一:μ/幅²,所有通道都以"几个探针幅度"计价)
   procedure Solve (Terms : Term_Vectors.Vector; N : Natural; Cap : Vec; Active : Mask; Damp : Vec;
                    A : out Vec; Ok : out Boolean);
   --  带优先级的解:Hard 里的约束【不许被牺牲】,Soft 只能在剩下的自由度里做文章。
   --  做法是真的零空间投影,不是"给硬的加大权重"—— 加权重只是让它更重要,不是让它不被牺牲。
   --  先解 Hard 得 A1;再把 Soft 的雅可比右乘投影阵 P = I − QᵀQ(Q = 硬约束行的正交化),
   --  解出 z,最终 A = A1 + P·z。P·z 恒落在硬约束的零空间里 ⇒ 走它不改变硬约束已经达成的那几行。
   --  上下限:A1 由第一段自己守;越界只缩 z 那一半,方向不变,硬约束照旧成立。
   procedure Solve_Priority (Hard, Soft : Term_Vectors.Vector; N : Natural; Cap : Vec; Active : Mask; Damp : Vec;
                             A : out Vec; Ok : out Boolean);
   --  这一行在这具身体上"推一格能被推动多少"(所有通道里最响的那个)。
   --  把每一行的误差按它自己的这个尺度归一,五行才在同一种货币里比较 —— 否则量纲最大的那一行独吞方程。
   function Row_Scale (E : Effect; Notch : Vec; R : Natural) return Long_Float;
   --  🔴 "这一行证明过了没有" 只在这里定义一次 —— 体检和执行器都问它,免得两处判据分叉。
   --  证明过 = 一格推得动(尺度 > 0)+ 同一个推法重复过至少两次 + 散布小于均值本身。
   function Row_Proven (E : Effect; Notch : Vec; R : Natural) return Boolean;
   function Row_Why (E : Effect; Notch : Vec; R : Natural) return String;   --  没证过时,一句人话
end Table;
