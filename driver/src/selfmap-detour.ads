--  路上不撞,直走被挡时绕过去的那一段(大并行 §2 第 25 条;路 4,10-01):在关节空间里找一条绕过去的路。
--  算法:RRT-Connect(Kuffner & LaValle,ICRA 2000)—— 起点、终点各长一棵树;每一轮在量到的关节范围里随机取一处,一棵树朝它长,
--  另一棵树朝新长出来的那一处连;两棵树接上就是一条路。最后按同一个判法把路上隔着几处还能直接连的两处连上(捷径),去掉多绕的。
--  一段关节直线能不能走(Segment):从一头起,每一步走多远由量到的净空定 —— 这一处离最近的"可能碰到"的带子还有 Margin (Q)
--  (世界长度;≤ 0 = 已经在带子里),这一步所有表面点最多挪 Shift (Q, Q')(世界长度,沿这段关节直线挪的路程的上界)不超过它就碰不上
--  (保守推进);每一步至少要挪出看得出的一档 Res(世界长度:这只手一步看得见的那一档)才算往前走了 —— 贴着带子走、一步挪不出一档 ⇒
--  这一段走不通。步长、净空都是量的(Margin 来自路 1 的 Links.Clear_Of,Shift 来自同一份表面点);没有拍的分辨率。
--  纯的(只调传进来的两个函数),导出给自检
with Bytes; use Bytes;
with Plug;
package Selfmap.Detour is
   type Margin_Fn is access function (Q : Floats) return Long_Float;
   type Shift_Fn is access function (Q0, Q1 : Floats) return Long_Float;
   --  一段关节直线 A → B 能不能走;Reach = 走得到这一段的几成(0..1;Free ⇒ 1)
   procedure Segment (A, B : Floats; Margin : Margin_Fn; Shift : Shift_Fn; Res : Long_Float; Reach : out Long_Float; Free : out Boolean);
   --  从 Start 到 Goal 找一条每一段都走得通的路:Path = 一串关节位置(头一个 = Start、最后一个 = Goal);Lo / Hi = 量到的关节范围(随机取点只在这里面);
   --  Seed = 随机数的种子(同一个种子同一条路,只为可重放)。直走就通 ⇒ Path = [Start, Goal]、Tries = 0。
   --  Found = False ⇒ 在试满的次数里没找到(照实说;不当成没有路)
   procedure Plan (Start, Goal, Lo, Hi : Floats; Margin : Margin_Fn; Shift : Shift_Fn; Res : Long_Float; Seed : Integer;
                   Path : out Plug.Floats_Vectors.Vector; Found : out Boolean; Tries : out Natural);
   --  试多少回(保险上限:有出口 —— 两棵树接上就停;试满还没接上照实说没找到)
   Max_Tries : constant := 2000;
end Selfmap.Detour;
