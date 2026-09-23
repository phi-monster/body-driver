--  仪器:驱动旁边的一个小 HTTP 进程(harness/instruments/serve.py),用任务无关的学习型模型从一帧里量物理量,
--  连不确定度一起还回来。驱动只认数字不认模型:换模型只改那边。
--  没配仪器(--inst 空)⇒ 这里一律 Ok = False,几何照旧全靠身体自己量;配了也只是多一条带不确定度的观测,
--  和自己量的那份在同一套最小二乘里对账,不是另一条路。
with Bytes; use Bytes;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Geom;
package Instrument is
   type Calib is record
      Ok : Boolean := False;
      F, F_Sd : Long_Float := 0.0;        --  焦距 ± 不确定度(像素)
      Up : Geom.V3 := [others => 0.0];    --  图里"上"的方向(驱动的相机系:x 右、y 上、z 朝后;单位向量)
      Up_Sd : Long_Float := 0.0;          --  弧度
      Ms : Long_Float := 0.0;             --  仪器那边花的毫秒
      Model : Unbounded_String;           --  哪个模型、哪个版本(钉死的)
   end record;
   --  一张图 ⇒ 焦距 + "上"的方向。Err 说清为什么没量到(没配 / 连不上 / 它说量不了)
   function Calibrate (Host : String; Port : Natural; RGB : Buf; W, H : Natural; Err : out Unbounded_String) return Calib;
end Instrument;
