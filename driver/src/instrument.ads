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
   --  配点:两帧(两台相机,或同一台相机两个位置)里哪两个像素是同一个真实的点。Pts = A 里的像素(Cert 不看),
   --  返回每个点在 B 里落在哪(顺序不变)和模型自己给的可信度(0..1)。驱动不拿可信度当真,只拿几何去核(多停三角的重投、各停交叉)。
   --  空 = 这一对没问到(Err 说为什么)
   type Match_Pt is record
      U, V : Long_Float := 0.0;
      Cert : Long_Float := 0.0;
   end record;
   package Match_Vectors is new Ada.Containers.Vectors (Natural, Match_Pt);
   function Match (Host : String; Port : Natural; RGB_A : Buf; W_A, H_A : Natural; RGB_B : Buf; W_B, H_B : Natural;
                   Pts : Match_Vectors.Vector; Err : out Unbounded_String) return Match_Vectors.Vector;
   --  存一帧在仪器那边,之后配点只报编号(开机扫描同一帧要和几十帧配,不必每次都把图传一遍);Id < 0 = 没存成(Err 说为什么)
   procedure Frame_Put (Host : String; Port : Natural; RGB : Buf; W, H : Natural; Id : out Integer; Err : out Unbounded_String);
   --  两帧(按编号)之间让仪器抽 Num 对对应点:每对 = A 里的像素、B 里的像素。仪器按它自己的把握抽(抽得多的地方它有把握),
   --  驱动不拿可信度当真,只拿几何去核。空 = 这一对没问到(Err 说为什么)
   type Pair_Pt is record
      Ua, Va, Ub, Vb : Long_Float := 0.0;
   end record;
   package Pair_Vectors is new Ada.Containers.Vectors (Natural, Pair_Pt);
   function Sample_Ids (Host : String; Port : Natural; Ia, Ib : Natural; Num : Natural; Err : out Unbounded_String) return Pair_Vectors.Vector;
   --  分割(SAM 2.1,2026-09-26 owner 批准):脑给一个框(X1 < X0 = 没有框)、或几个点(在它身上 / 不在)⇒ 那件东西在这一帧里的整片像素。
   --  Mask 按行展开(W*H 个,是 = 它);Score = 模型自报的 IoU(只报数、不当门);Area = 像素数。没配仪器 / 没问到 ⇒ Ok = False(Err 说为什么)
   type Seg_Pt is record
      U, V : Long_Float := 0.0;
      On : Boolean := True;
   end record;
   package Seg_Pt_Vectors is new Ada.Containers.Vectors (Natural, Seg_Pt);
   procedure Segment (Host : String; Port : Natural; RGB : Buf; W, H : Natural; X0, Y0, X1, Y1 : Integer; Pts : Seg_Pt_Vectors.Vector;
                      Mask : out Bools; Area : out Natural; Score : out Long_Float; Ok : out Boolean; Err : out Unbounded_String);
end Instrument;
