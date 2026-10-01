--  身体图:"关节这样的时候,我的每一块零件在这台相机的画面里在哪儿"。人不用认自己的手,因为它从小记住了这张图。
--  零件 = 某个通道单独动时跟着动的那一块(从那个关节往外的全部);握合通道也是通道,它带的那块就是手指。
--  样本 = 一次真看见自己(推一下通道 / 合一下手,看哪些像素动了)时的:位姿 + 看见的那几块零件的位置。
--  查:找位姿最近的样本(位姿差按各通道探针幅度归一),差的那一点由响应表外推。样本越多,越不用看。
with Plug;
with Table;
with Chan;
with Bytes; use Bytes;
with Ada.Containers.Vectors;
package Schema is
   type Part_Pos is record
      Valid : Boolean := False;
      Cu, Cv, Z : Long_Float := 0.0;        --  形心(归一化画幅)、深度(米;<= 0 = 没读到)
      X0, Y0, X1, Y1 : Natural := 0;        --  像素框
      N_Blobs : Natural := 0;               --  这块由几团组成(两指 = 2 团;先只存前两团的形心 —— 五指是欠账)
      B0u, B0v, B1u, B1v : Long_Float := 0.0;
   end record;
   --  每个通道一块:0 .. Per_Arm-1 = 位姿通道带的,Per_Arm + k = 第 k 个抓握通道带的那一块(手指)
   type Part_Array is array (0 .. Chan.Per_Arm + Chan.Max_Jaws - 1) of Part_Pos;
   type Sample is record
      Arm, Cam : Natural := 0;
      Pose : Plug.Arm_Pose := [others => 0.0];
      Parts : Part_Array;
   end record;
   package Sample_Vectors is new Ada.Containers.Vectors (Natural, Sample);
   type Map is record
      S : Sample_Vectors.Vector;
   end record;
   Max_Per_Pair : constant := 64;   --  每 (臂,相机) 最多留几个样本(次数,无量纲);满了丢最早的
   --  加一个样本:同 (臂,相机) 且位姿差在本体噪声内 ⇒ 合进旧的(只盖住这次真看见的那几块),同一个地方看了两次信新的
   procedure Add (M : in out Map; X : Sample; EE_Noise, Rot_Noise : Long_Float);
   --  最近样本:位姿差按各通道探针幅度归一(Amp 每臂 Per_Arm 个)。返回样本号或 -1;Diff = 现位姿 − 样本位姿(按通道);Dist = 归一距离
   function Nearest (M : Map; Arm, Cam : Natural; Pose : Plug.Arm_Pose; Amp : Floats; Per_Arm : Natural;
                     Diff : out Table.Vec; Dist : out Long_Float) return Integer;
   function Count (M : Map; Arm, Cam : Natural) return Natural;
end Schema;
