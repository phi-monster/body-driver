--  两只普通相机 → 表面点。没有深度传感器:owner 09-11 定的架构底线,这条无深度的路是地板。
--  两只眼的相对位姿由本体感受免费给(相机拧在身体上,机器人知道它们各在哪),不需要外部标定。
--  哪些像素是这个物体、左像素对哪个右像素 —— 那是学的活(眼睛给),这里只算几何:视线落到面上、两条视线交一点、顶面拉到支撑面、把桌面那张平面扔掉。
--  2026-08 用 Rust 写成(commit ef10664 point-gen/src/lib.rs 的 triangulate / extrude_to_support / merge / drop_support_plane),搬回 Ada;
--  视线的来源就是驱动现成的 Geom.Ray / Ray_Fixed(腕眼、头顶眼都行)。
with Geom;
package Contact.Surface is
   --  一个物体的轮廓像素各发一条视线,全落到它躺的面(过 P0、法向 N)上 ⇒ 它顶面的点。视线和面平行或交在身后的丢掉;丢了几条要报出来,悄悄丢就成了"点云很干净"的假象
   procedure On_Plane (Rays : Geom.Sight_Vectors.Vector; P0, N : V3; Pts : in out V3_Vectors.Vector; Dropped : out Natural);
   --  两条视线在三维里一般不相交,取它们最近的那一段的中点;差得太远(Miss > Tol_M)就是左右眼配错了点,当场拒绝,不许当成一个点收下;交在身后也不算
   function Pair (A, B : Geom.Sight; Tol_M : Long_Float; Ok : out Boolean; Miss : out Long_Float) return V3;
   --  把看得见的顶面朝支撑面(z = Support_Z,先 To_Upright)拉下去,补出侧面。这一条是【假设】不是测量:物体是实心的、从顶面一直连到支撑面。
   --  马克杯把手、拱形件、悬臂、有凹槽的东西下面拉出来的"侧面"根本不存在,爪子会合到空气上;摆在桌上的紧凑实心件(积木、瓶子、剪刀)占绝大多数。
   --  用它就要明说用了。首选仍然是换个视角再看一眼(Merge),那条一个假设都不用
   procedure Extrude_To_Support (Pts : in out V3_Vectors.Vector; Support_Z, Step_M : Long_Float);
   --  几个视角的点合到一起,不需要任何假设:每一帧的相机位姿由本体感受给,视线落出来的本来就是世界坐标
   procedure Merge (Into : in out V3_Vectors.Vector; More : V3_Vectors.Vector);
   --  把支撑面那张平面上的点扔掉:确定性 RANSAC(不引随机数,同一份数据两次给同一个答案)找内点最多的那张平面,再把内点扔掉。
   --  相机斜着看时一块 10 cm 的桌面自带 60 mm 高差,按深度筛会把整片桌面当成物体(2026-08-16 实测掩膜是个规整的圆盘)
   procedure Drop_Support_Plane (Pts : in out V3_Vectors.Vector; Tol_M : Long_Float; Normal : out V3; On_Plane_Count : out Natural);
end Contact.Surface;
