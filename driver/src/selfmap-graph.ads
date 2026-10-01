--  身体图的通用问法(大并行 I1,路 1;10-01):别处要知道"几条臂、这条臂的位姿通道、它有几个合拢通道、长在它上面的眼、
--  不长在任何臂上的眼、扛着全身走的那几组",只问这几句 —— 不按下标算通道号(臂 × 每臂几个 + 第几轴)、不认"一条臂一只眼"、
--  不认"一条臂至少一个合拢通道"、不认"只有一只不动的眼"。
--  开机逐组推一下量过(M.Groups,10-01 I1)⇒ 眼、扛着全身的组从它答;几条臂、位姿通道、合拢通道个数照旧字段答(Arms、Per_Arm、Jaws;
--  旧字段照旧填、照旧能用)。答法在这里换,问的人一行不改。纯函数(只读 M),导出给自检。
--  (这里的"身体图"是 Body_Map:哪些通道、哪些眼归哪一块;Schema.Map 那张"位姿 → 手指在画面哪儿"的样本表另是一样东西)
package Selfmap.Graph is
   --  几条臂(能摆位姿的零件;0 = 一条都没有)
   function Arm_Count (M : Body_Map) return Natural;
   --  这条臂的位姿通道:第 K 个 = 这条臂位姿第 K 个分量(Chan 的次序:三个平移、三个绕世界轴的小转动)的通道号,
   --  也就是 M.Amp / M.Delivered / M.Seen 的下标、M.Parts(通道 × 相机)里通道那一维。没有这条臂 ⇒ 空
   function Pose_Channels (M : Body_Map; Arm : Natural) return Ints;
   --  这条臂有几个合拢通道:量到几个就是几个,可以 0 个;没记 ⇒ 0(不按"至少一个"猜)
   function Closing_Count (M : Body_Map; Arm : Natural) return Natural;
   --  长在这条臂上的眼(相机号,一串:可以几只,可以没有)
   function Eyes_On (M : Body_Map; Arm : Natural) return Ints;
   --  不长在任何臂上的眼(相机号,从小到大,一串:可以 0 只)
   function Eyes_Off_Arms (M : Body_Map) return Ints;
   --  扛着全身走的那几组(推一下每只眼一起动、世界在所有眼里一起流的那组读数;大并行 §2 第 2 条):M.Groups 的下标(开机逐组推一下认的);
   --  没按组量过 ⇒ 空
   function Carrying_Groups (M : Body_Map) return Ints;
   --  开机报告里念身体图的那一行:只按上面几问念,不读字段
   function Say (M : Body_Map) return String;
end Selfmap.Graph;
