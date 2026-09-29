--  ②a 接触集的几何零件(09-29 起,两块相向合的下手处由 Contact.Grasp 挑;这里原来那张八月的排序规矩单 Candidates / Close_Yaw / To_Set 删了:
--  挑的方向和真去夹的方向差 90° 就出在那儿)。剩下:转正(支撑面不水平)、采样间距、吸盘、环抓 —— 后两种手还没并进 Contact.Grasp 的同一个结构。
--  它替掉的是【我的手】:力臂往哪挪 · 爪面朝哪 · 挪多少会挪出物体外 —— 全是人在拍脑袋,形状一换就废。
--  包围盒是不够的(实测):包围盒说"这一段 6 厘米实心",剪刀的真身是两片薄刃夹一条缝 —— 按盒子选的下手点,爪子从缝里合过去,指间什么都没有。
--  所以这里吃的是表面点,不是盒子。身体常数全是传进来的参数,不是这里读出来的(方向单向:反过来就破了"换机体不重训"那堵墙)。
--  2026-08 用 Rust 写成、每一条排序规矩都是真抓失败逼出来的(commit ef10664 contact-gen/src/{lib,support,hands}.rs);逐段搬回 Ada,数一个没改。
with Ada.Containers.Vectors;
package Contact.Gen is
   --  交接给接触集时,为什么交不出去
   type Handoff_Kind is (Fine, Mu_Unknown, Would_Slip);
   type Handoff is record
      Kind : Handoff_Kind := Fine;
      Need_Rad, Have_Rad : Long_Float := 0.0;   --  Would_Slip:两个夹持面歪了 Need,摩擦锥只有 Have
   end record;
   --  支撑面不水平的机器:把点云转到"支撑面法向 = +z"的那个系里算,算完再转回来。算法一个字不用动,假设变成显式输入。
   --  它买到的是朝向无关,不是重力无关:哪一面是支撑面仍然由调用方说(这一层不知道重力往哪儿,也不该知道)
   type Rot is record
      Axis : V3 := [0.0, 0.0, 1.0];
      Ang : Long_Float := 0.0;
   end record;
   function Inverse (R : Rot) return Rot;

   --  另外两种手:吸盘(1 点)· 环抓(n 点)。同一张接触集表,三条不同的几何路径填:表不认识机体,机体各自算各自的
   type No_Hand_Kind is (Fine, Too_Few_Points, No_Flat_Patch, Nothing_In_Direction, Not_Surrounding, Handed_Off);
   type No_Hand is record
      Kind : No_Hand_Kind := Fine;
      Found_R, Need_R : Long_Float := 0.0;   --  No_Flat_Patch:实测的最大平坦半径 / 要求的半径
      Direction : Natural := 0;              --  Nothing_In_Direction:第几个方向摸不到料
      H : Handoff;                           --  Handed_Off:转发原因
   end record;
   function Sampling_Gap (Cloud : V3_Vectors.Vector) return Long_Float;   --  点云自己的采样间距:每个点到最近邻的距离取中位。量得出来就不许拍
end Contact.Gen;
