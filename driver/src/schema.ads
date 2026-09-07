--  身体图:"关节这样的时候,我的手指在这台相机的画面里在哪儿"。人不用认自己的手,因为它从小记住了这张图。
--  样本 = 一次真看见自己手指(只动手指、看哪些像素动了)时的:位姿 + 两瓣位置 + 区心 + 手指深度。
--  查:找位姿最近的样本(位姿差按各通道探针幅度归一),差的那一点由响应表外推。样本越多,越不用看。
with Plug;
with Table;
with Chan;
with Bytes; use Bytes;
with Ada.Containers.Vectors;
package Schema is
   --  一个零件在画面里的位置:某个通道单独动时跟着动的那一块(从那个关节往外的全部),真看见过才 Valid
   type Part_Pos is record
      Valid : Boolean := False;
      Cu, Cv, Z : Long_Float := 0.0;        --  形心(归一化画幅)、深度(米;<= 0 = 没读到)
      X0, Y0, X1, Y1 : Natural := 0;        --  像素框
   end record;
   type Part_Array is array (0 .. Chan.Per_Arm - 1) of Part_Pos;
   type Sample is record
      Arm, Cam : Natural := 0;
      Pose : Plug.Arm_Pose := [others => 0.0];
      Lobes_Valid : Boolean := False;       --  这次看见的是手指(合手/抖手指)
      N_Lobes : Natural := 0;
      Au, Av, Bu, Bv : Long_Float := 0.0;   --  两瓣在画面里的位置(归一化画幅)
      Cu, Cv : Long_Float := 0.0;           --  区心
      Z : Long_Float := 0.0;                --  手指深度(米);<= 0 = 没读到
      Parts : Part_Array;                   --  这只手每个通道带的零件(全身:每个马达一块)
   end record;
   package Sample_Vectors is new Ada.Containers.Vectors (Natural, Sample);
   type Map is record
      S : Sample_Vectors.Vector;
   end record;
   Max_Per_Pair : constant := 64;   --  每 (臂,相机) 最多留几个样本(次数,无量纲);满了丢最早的
   --  加一个样本:同 (臂,相机) 且位姿差在本体噪声内 ⇒ 合进旧的(只盖住这次真看见的那几项:手指 / 哪几个零件),同一个地方看了两次信新的
   procedure Add (M : in out Map; X : Sample; EE_Noise, Rot_Noise : Long_Float);
   --  最近样本:位姿差按各通道探针幅度归一(Amp 每臂 Per_Arm 个)。返回样本号或 -1;Diff = 现位姿 − 样本位姿(按通道);Dist = 归一距离
   function Nearest (M : Map; Arm, Cam : Natural; Pose : Plug.Arm_Pose; Amp : Floats; Per_Arm : Natural;
                     Diff : out Table.Vec; Dist : out Long_Float) return Integer;
   function Count (M : Map; Arm, Cam : Natural) return Natural;
end Schema;
