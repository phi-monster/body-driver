--  通道响应表(效能矩阵):每个通道动一点,某个被跟踪的点在画面里 (u,v,深) 变多少。
--  在线递推最小二乘(带遗忘)重估;两套模型的责任:空着走的表 vs "顶住了"的零表,谁预测准信谁 ⇒ 碰上/推不动从这里长出来。
--  解算:带上下限的加权最小二乘分配(飞控的控制分配),没有减半/放大/禁用这类规则。
with Ada.Containers.Vectors;
package Table is
   Max_Ch : constant := 64;
   subtype Ch_Index is Natural range 0 .. Max_Ch - 1;
   type Vec is array (Ch_Index) of Long_Float;
   type Vec3 is array (0 .. 2) of Long_Float;
   type Mat3 is array (Ch_Index, 0 .. 2) of Long_Float;
   type Cov is array (Ch_Index, Ch_Index) of Long_Float;
   type Mask is array (Ch_Index) of Boolean;
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
   end record;

   procedure Reset (E : in out Effect; N : Natural; P0 : Long_Float);
   procedure Set_Prior (E : in out Effect; Ch : Natural; P0 : Long_Float);   --  这一通道的先验不确定度(按它的命令量级定)
   procedure Set_Col (E : in out Effect; Ch : Natural; D : Vec3);
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
end Table;
