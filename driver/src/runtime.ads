--  Sinew 的执行器:一台纯状态机。它只管【下一条该干什么】,不碰相机、不碰电机、不碰网络。
--  纯 ⇒ 能离线测,也能被"空转"直接复用(同一台机器,喂预测的结局而不是真结局)。
with Sinew;
package Runtime is

   --  往前走一步之后,调用方拿到的是什么
   type Yield is (Y_Interval,    --  有一段区间要执行(真动身体)
                  Y_Say,         --  一句人话
                  Y_Remember,    --  记住一个地方
                  Y_Done,        --  脑说做完了
                  Y_Finished,    --  程序跑完了
                  Y_Broken);     --  程序本身坏了(调用了不存在的名字、跑飞了)

   Max_Stack : constant := 32;   --  嵌套多深(次数,无量纲)
   --  一段程序最多执行多少条指令 —— 挡住不终止的循环(次数,无量纲)
   Max_Instr : constant := 100_000;

   type Frame is record
      Head : Natural := 0;       --  循环头在哪
      Exit_At : Natural := 0;    --  跳出去到哪
      Left : Natural := 0;       --  还剩几次(0 且 Counted 为真 = 用完了)
      Counted : Boolean := True;
      Cond : Sinew.Outcome := Sinew.Oc_None;
   end record;

   type Addr_Stack is array (1 .. Max_Stack) of Natural;
   type Frame_Stack is array (1 .. Max_Stack) of Frame;

   type Machine is record
      PC : Natural := 0;
      Last : Sinew.Outcome := Sinew.Oc_None;   --  上一段区间的结局:控制流唯一能读的东西
      Ticks : Natural := 0;
      Note : Sinew.Instr;                       --  坏掉时是哪一条
      Why : Sinew.Outcome := Sinew.Oc_None;
      Calls : Addr_Stack := [others => 0];
      Call_N : Natural := 0;
      Loops : Frame_Stack;
      Loop_N : Natural := 0;
      Tries : Addr_Stack := [others => 0];
      Try_N : Natural := 0;
   end record;

   --  走到下一条需要调用方参与的指令。控制类指令(跳转/循环/分支/调用)在里面走完,不返回。
   procedure Advance (P : Sinew.Program; M : in out Machine; What : out Yield; I : out Sinew.Instr);

   --  上一段区间跑完了,把结局喂回来。失败且身处 try 里 ⇒ 跳到 or 那一段。
   procedure Report (P : Sinew.Program; M : in out Machine; O : Sinew.Outcome);

   function Broken_Why (M : Machine) return String;
end Runtime;
