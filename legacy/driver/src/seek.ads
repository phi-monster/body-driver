--  对准、插进去、挂上去(大并行.md §2 第 21 条,路 6)。
--  要的量:拿着的东西上那一处(Held 的形状里的一点:此刻在世界里在 Here)对目标上那一处(孔口、环、钩子的那一点,估计在 Est,路 3 的东西)
--  的相对位置,和它的协方差 Cov(两边的不准加起来);往里送的方向 Axis = 目标那一处的轴
--  (孔的轴、环的法向),深度 Depth ± Depth_Sd = 送进去多深算到位(量的)。插、挂、对都走这一段,代码里不分。
--  ① 先在估计的那一处沿轴往里送:送到 Est 沿轴的那一截 + Depth(减去沿轴不准的 Z 倍)就算到位;被挡住(Selfmap.Blocked 那一个判法,
--     由 Move 报)⇒ 退回送之前那一处,
--  ② 在横着的不准那一片里换地方再送:候选 = 垂直于轴的面上一格一格的点,只要横着的不准的二维门(Z 倍椭圆)以内的;最可能的先试
--     (按马氏距离从近到远)。格子多密:缝(Gap:孔和插进去的东西之间横着能偏多少还进得去,两边的形状量的)接得住多偏就隔多远 ——
--     方格离格点最远的地方 = 间距 / √2 ⇒ 间距 = √2 × 缝;缝不知道(0)/ 比手靠得住的最小一步(Resolution)还细 ⇒ 按最小一步。
--     眼的不准比缝小时一格都不用换(第一次就进去了);比缝大多少,就要多少格 —— 这是这件事本来的代价,照实报试了几处。
--  ③ 那一片里都试过了还进不去 ⇒ 照实说"不准那一片里没找到"(试了几处、最远到几倍不准)。
--  不认单位和尺度(所有的长度都是调用方给的、量的);代码里没有东西的名字、没有动作的名字
with Geom;
package Seek is
   subtype V3 is Geom.V3;
   subtype M3 is Geom.M3;
   --  身体走一步(调用方:Selfmap.Step 走拿着的那一处):沿 Dir(单位向量)走 Len
   type Step_Report is record
      Ok : Boolean := False;              --  这一步做没做成(线断了、反解不到 ⇒ False)
      At_Now : V3 := [others => 0.0];     --  走完以后拿着的那一处在世界里在哪(量的:关节读数 + 拿着的形状;每一步重量,不按走了多少累加 ——
                                          --  累加的话每一步的读数噪声越攒越多,格子横着就散了)
      Blocked : Boolean := False;         --  被挡住了(Selfmap.Blocked)
   end record;
   --  Too_Shallow:深度不比沿轴的不准深多少(Depth ≤ Z × (沿轴的不准 + 送到位那一处的不准)),顶在孔口上和送到底分不出 ⇒ 不送,照实说
   type Seek_End is (Reached, Not_Found, Body_Failed, No_Axis, Too_Shallow);
   type Report is record
      How : Seek_End := Not_Found;
      Tries : Natural := 0;              --  往里送了几次
      Candidates : Natural := 0;         --  不准那一片里一共几处可试
      Spacing : Long_Float := 0.0;       --  格子隔多远
      Lateral : V3 := [others => 0.0];   --  进去的那一次(或最后试的那一次)横着离估计的那一处偏了多少(世界系)
      Went_In : Long_Float := 0.0;       --  那一次沿轴送了多远
      Mahal_Max : Long_Float := 0.0;     --  试过的最远一处离估计几倍不准(马氏距离)
   end record;
   procedure Run (Here, Est : V3; Cov : M3; Axis : V3; Depth, Depth_Sd, Gap, Resolution : Long_Float;
                  Move : access procedure (Dir : V3; Len : Long_Float; R : out Step_Report);
                  Rep : out Report);
   function Say (Rep : Report) return String;
end Seek;
