--  两只普通相机 → 表面点。没有深度传感器:owner 09-11 定的架构底线,这条无深度的路是地板。
--  两只眼的相对位姿由本体感受免费给(相机拧在身体上,机器人知道它们各在哪),不需要外部标定。
--  哪些像素是这个物体、左像素对哪个右像素 —— 那是学的活(眼睛给),这里只算几何:视线落到面上、两条视线交一点、顶面拉到支撑面、把桌面那张平面扔掉。
--  2026-08 用 Rust 写成(commit ef10664 point-gen/src/lib.rs 的 triangulate / extrude_to_support / merge / drop_support_plane),搬回 Ada;
--  视线的来源就是驱动现成的 Geom.Ray / Ray_Fixed(腕眼、头顶眼都行)。
with Geom;
package Contact.Surface is
   --  一个物体的轮廓像素各发一条视线,全落到它躺的面(过 P0、法向 N)上 ⇒ 它顶面的点。视线和面平行或交在身后的丢掉;丢了几条要报出来,悄悄丢就成了"点云很干净"的假象
   procedure On_Plane (Rays : Geom.Sight_Vectors.Vector; P0, N : V3; Pts : in out V3_Vectors.Vector; Dropped : out Natural);
   --  同一个假设(实心、从顶面一直连到支撑面),只补表面:顶面的点照留,只从轮廓那一圈(格子边长两个 Pitch —— 同接触集判"同一块料"的邻居范围,
   --  点不在格点上时里面才不会冒出空格;四邻有空格的那些格)
   --  沿 -N 每隔 Pitch 往下补到支撑面(过 Support_P、法向 N)—— 世界系里做,不用先转正。里面不填(接触集按表面点算法向,实心的点会把法向带歪)
   procedure Walls_To_Support (Top : V3_Vectors.Vector; N, Support_P : V3; Pitch : Long_Float; Pts : out V3_Vectors.Vector);
end Contact.Surface;
