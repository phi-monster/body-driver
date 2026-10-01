with Bytes; use Bytes;
--  探针那一推看没看见(路 1,10-01;Act.Probe_Effects 用它):推一根位姿通道一下,画面里跟的点挪没挪 —— 只认量得出来的挪动。
--  原来:"挪过 4 像素 或者 远近变过远近的地板",驱动不读深度以后远近地板恒为 0、后一半恒真 ⇒ 只要命令实到超过读数噪声就算"量到了",
--  点一个像素都没挪也写进表(V1B79:54 次推里 48 次点跑了 0.0000 画幅,照样记成这一列是零、还信它)。
--  改成:① 跟踪地板是量的 —— 什么都不做的两帧上每个点重跟一次,挪了多少就是跟踪噪声(每轴 σ),地板 = Stats.Z 倍(同 Links 判"挪了");
--  ② 这一推里一个点算挪了 = 它看得见(不是长在这只眼上的那只手的点:那只手在它自己的眼里永远不动,零是按构造知道的,不是量的)、
--     重跟没跟丢(Retrack 说 Lost:手动了这儿却没流、出了画面、找不到那一块)、挪的超过地板;
--  ③ 一推里没有一个点挪了 ⇒ 加倍再推(到上限为止);加倍以后挪的(只比量得出来的)一点没多 ⇒ 这一列就是零。
package Probe is
   --  每轴跟踪噪声(画幅):静止两帧上重跟,各点挪了的平方(du² + dv²)⇒ √(Σ / 2N)(二维正态每轴 σ 的最大似然);一个都没有 ⇒ 0
   function Track_Sigma (Static_D2 : Floats) return Long_Float;
   --  跟踪地板 = Stats.Z × 每轴 σ
   function Floor_Of (Sigma : Long_Float) return Long_Float;
   --  这一推里这一点算不算挪了:看得见(Own = False)、没跟丢、挪的(画幅)超过地板
   function Moved (Own, Lost : Boolean; Ran, Floor : Long_Float) return Boolean is (not Own and then not Lost and then Ran > Floor);
   --  一推以后下一步怎么走:Seen = 有一个点挪了;Ran_Meas = 这一推里量得出来的挪动最大多少(看得见、没跟丢的点;一个都没有 ⇒ < 0);
   --  Last_Ran = 上一推(幅度一半)的那个数(< 0 = 上一推一个点都没量出来 / 还没推过);Amp / Cap = 这一推的幅度和上限
   type Next_Step is (Take_It, Double_It, Is_Zero, At_Cap);
   function Next (Seen : Boolean; Ran_Meas, Last_Ran, Amp, Cap : Long_Float) return Next_Step;
end Probe;
