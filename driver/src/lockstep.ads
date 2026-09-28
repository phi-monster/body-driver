--  几只手各做各的那一段(每只手一个任务),按拍对齐(2026-09-28,PLAN ⑧ (g)):同一时刻只有一个线程在跑 —— 主线程,或者某一只手,
--  像交接力棒;所以身体的量、链、这一帧这些共用的东西不会被两个线程同时改,每只手那一段的写法和一只手单独做时一模一样。
--  一只手要下一帧(Plug.Sense)时把棒交还主线程、等被叫醒;它发的命令(Plug.Act)只记下这只手这一拍的目标。
--  主线程一轮:挨个叫醒还没做完的手、等它交还;都交还了 ⇒ 把几只手记下的目标合成一条命令发出去、收一帧(Plug.Lock_Beat),再下一轮。
--  一只手在算(挑落点、配点、解)的时候不交还 ⇒ 这一拍等它,别的手也不往前走(按拍对齐,不抢拍)。
with Ada.Task_Identification;
package Lockstep is
   Max_Hands : constant := 8;   --  一次最多几只手一起(次数)
   --  主线程:第 H 只手由这个任务做(叫醒它之前登记)
   procedure Start (H : Natural; Id : Ada.Task_Identification.Task_Id);
   --  手的任务一开头调:等主线程第一次叫醒它
   procedure Begin_Hand (H : Natural);
   --  手的任务要下一帧时调:交还主线程,等下一拍被叫醒
   procedure Yield;
   --  手的任务做完了调:交还主线程,以后不再叫它
   procedure Done;
   --  主线程:叫醒第 H 只手,等它交还(Yield 或 Done)
   procedure Run (H : Natural);
   function Finished (H : Natural) return Boolean;
   --  调用者是第几只手的任务(-1 = 不是:主线程或别的线程)
   function Current_Hand return Integer;
   --  主线程:这一段做完,清掉登记
   procedure Clear;
end Lockstep;
