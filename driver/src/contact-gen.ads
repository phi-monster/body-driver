--  ②a 接触集的几何零件(09-29 起,下手处由 Contact.Search 挑;这里原来那张八月的排序规矩单 Candidates / Close_Yaw / To_Set 删了:
--  挑的方向和真去夹的方向差 90° 就出在那儿;10-01 没人用的转正、吸盘、环抓那几个类型也删了)。剩下:采样间距。
--  它替掉的是【我的手】:力臂往哪挪 · 爪面朝哪 · 挪多少会挪出物体外 —— 全是人在拍脑袋,形状一换就废。
--  包围盒是不够的(实测):包围盒说"这一段 6 厘米实心",剪刀的真身是两片薄刃夹一条缝 —— 按盒子选的下手点,爪子从缝里合过去,指间什么都没有。
--  所以这里吃的是表面点,不是盒子。身体常数全是传进来的参数,不是这里读出来的(方向单向:反过来就破了"换机体不重训"那堵墙)。
--  2026-08 用 Rust 写成、每一条排序规矩都是真抓失败逼出来的(commit ef10664 contact-gen/src/{lib,support,hands}.rs);逐段搬回 Ada,数一个没改。
with Ada.Containers.Vectors;
package Contact.Gen is
   function Sampling_Gap (Cloud : V3_Vectors.Vector) return Long_Float;   --  点云自己的采样间距:每个点到最近邻的距离取中位。量得出来就不许拍
end Contact.Gen;
