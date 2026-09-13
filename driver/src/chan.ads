--  通道 ↔ 位姿的纯数学:末端模式一条臂六个通道(三个平移、三个绕世界轴的小转动)。没有任何身体量。
with Plug;
with Table;
with Limits;
package Chan is
   subtype Pose is Plug.Arm_Pose;
   Per_Arm : constant := 6;
   --  一条臂上最多几个【抓握通道】。实际有几个是量出来的(见 Selfmap.Body_Map.Jaws),这里只是数组上限。
   Max_Jaws : constant := Limits.Max_Jaws;
   Pos_Channels : constant := 3;   --  前三个通道 = 末端位姿的位置部分(观测的结构,不是身体假设)
   --  从 P0 出发,按通道量 A(前 6 个)合成一个绝对位姿命令
   function Compose (P0 : Pose; A : Table.Vec; Offset : Natural := 0) return Pose;
   --  两个位姿之间实际走了多少,按通道分解(平移差 + 相对转动向量)
   function Delivered (P0, P1 : Pose) return Table.Vec;
   function Rot_Vec (Q0, Q1 : Pose) return Table.Vec3;   --  Q1 相对 Q0 的转动向量(世界系)
   function Quat_Mul (A, B : Pose) return Pose;           --  只用 3..6 四元数位
end Chan;
