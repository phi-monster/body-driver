--  仪器:驱动旁边的一个小 HTTP 进程(harness/instruments/serve.py),用任务无关的学习型模型从一帧里量物理量,
--  连不确定度一起还回来。驱动只认数字不认模型:换模型只改那边。
--  没配仪器(--inst 空)⇒ 这里一律 Ok = False,几何照旧全靠身体自己量;配了也只是多一条带不确定度的观测,
--  和自己量的那份在同一套最小二乘里对账,不是另一条路。
with Bytes; use Bytes;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
package Instrument is
   --  一段跟踪:在这一帧里点几个像素,之后每来一帧问一次它们到哪了(记忆在仪器那边,驱动只拿段号)。
   --  换帧重切块会对不上号(V1C 2026-09-24:标定 8 停丢 5 停),跟点不重切
   type Track_Pt is record
      U, V : Long_Float := 0.0;
      Seen : Boolean := False;     --  仪器认为还看得见(概率过了它自己的阈值)
      Conf : Long_Float := 0.0;    --  看得见的概率(0..1)
   end record;
   package Track_Vectors is new Ada.Containers.Vectors (Natural, Track_Pt);
   --  开一段:Id < 0 = 没开成(Err 说为什么);返回这些点在这一帧里的位置(就是点的地方)
   function Track_Start (Host : String; Port : Natural; RGB : Buf; W, H : Natural; Pts : Track_Vectors.Vector;
                         Id : out Integer; Err : out Unbounded_String) return Track_Vectors.Vector;
   --  下一帧:返回同样多的点(顺序不变);空 = 这一帧没问到(Err 说为什么)
   function Track_Step (Host : String; Port : Natural; Id : Integer; RGB : Buf; W, H : Natural; Err : out Unbounded_String) return Track_Vectors.Vector;
   procedure Track_End (Host : String; Port : Natural; Id : Integer);
end Instrument;
