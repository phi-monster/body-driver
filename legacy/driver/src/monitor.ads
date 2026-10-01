--  监视器(SPARK,常数内存,纯函数):脑说的"直到什么为止"在这里被判。
--  所有门槛都是现场量出来的噪声地板;这里只做比较和计数,不含任何身体量。
pragma SPARK_Mode (On);
package Monitor is
   subtype Bounded is Long_Float range -1.0e12 .. 1.0e12;
   subtype Floor is Long_Float range 0.0 .. 1.0e12;
   type Floors is record
      Track : Floor := 0.0;       --  各块位置在画面里抖多少
      Picture : Floor := 0.0;     --  整幅画抖几级
      Reading : Floor := 0.0;     --  抓握读数抖多少
      Delivery : Floor := 0.0;    --  本体报的"实到"抖多少
   end record;
   subtype Count is Natural range 0 .. 2 ** 30;   --  计数封顶(饱和),证明里加一不会溢出
   type Watch is record
      Quiet : Count := 0;          --  画面连着几步没变
      No_Progress : Count := 0;    --  误差连着几步没缩
      Steps : Count := 0;
      Refused : Count := 0;        --  连着几步一步没走
      Blind : Count := 0;          --  连着几步【被判的那些点一个都没看见】(位置是按身体图猜的)
   end record;
   --  🔴 脑写的每一个结局词都要有【自己】的判法。以前 lost / free / refused 三个词
   --  统统落进兜底的 U_Steps,身体收下这个词然后做的是别的事,还回报"步子走完还没到"。
   --  free 尤其致命:语言里 free = "它离开了原来靠着的面" = 【被拿起来了】,正是本任务的判据,
   --  而它当时被接到 Slipped(爪子读数掉回空手)上,和"拿起来"毫无关系。
   type Until_Kind is (U_Steps, U_Contact, U_Resist, U_Slip, U_Settle, U_Stall, U_Lost, U_Free);

   procedure Step (W : in out Watch; Pic_Delta : Floor; Err_Before, Err_After : Bounded;
                   Delivered : Floor; F : Floors; Seen : Boolean)
     with Post => W.Steps = Natural'Min (W'Old.Steps + 1, Count'Last)
       and then (if Pic_Delta <= F.Picture then W.Quiet = Natural'Min (W'Old.Quiet + 1, Count'Last) else W.Quiet = 0)
       and then (if Delivered <= F.Delivery then W.Refused = Natural'Min (W'Old.Refused + 1, Count'Last) else W.Refused = 0)
       and then (if Seen then W.Blind = 0 else W.Blind = Natural'Min (W'Old.Blind + 1, Count'Last));
   --  🔴🔴 "画面不再变了"必须配一条旁证:【我这几步真动过】(IZ 2026-09-15 实测)。
   --  换姿势之后表是空的(每一行"一推能改"都量到 0.0000)⇒ 解算推不出任何命令
   --  ⇒ 身体不动 ⇒ 画面当然不变 ⇒ `until settled` 被【假满足】:4 推就报"到了、还差 0.0 推"。
   --  判据本身没错,错在它分不出"到位了"和"我根本没推动"。
   --  身体早就在量这件事:命令发了而实到落进本体噪声 ⇒ Refused 计数。
   --  这一条和今晚给"碰到"立的那条同构:**没动过就不许自称到了**。
   --  🔴 第三条旁证(JB 2026-09-15 实测):【我得真看见了我在判的那些点】。
   --  实测段尾原话:"could not see 2 of 2 of the points I am tracking; I am going on where my body
   --  map says they are" —— 跟丢之后位置是按身体图推的,画面当然不再变,于是 settle 在一个
   --  【幻影】上成立:身体报"差 0.022 m",而画面里手早飘回最右边几乎出框。
   --  跟丢不是停下的理由(照走、说出来),但跟丢【绝对不能算到了】。
   function Settled (W : Watch) return Boolean is
     (W.Quiet >= 2 and then W.Refused = 0 and then W.Blind = 0);
   --  连着几步没比"到目前为止最好的一次"更好才算走不动了(次数,无量纲)。
   --  2 太急:每一步只缩掉剩余差距的百分之一二,噪声一晃就被判死,一段永远走不完(FL 实测)
   function Stalled (W : Watch) return Boolean is (W.No_Progress >= 5);
   function Refusing (W : Watch) return Boolean is (W.Refused >= 2);
   function Slipped (Reading, Empty : Bounded; Noise : Floor) return Boolean is (Reading - Empty <= Noise);
   --  离开原来靠着的面 = 它此刻"鼓出背景"的高度,比这一段开始时高出的量超过了这个量自己的抖动。
   --  两个高度都是量出来的(米),抖动也是量出来的(这一点的深度噪声地板)——这里只做比较。
   function Came_Free (Height_Now, Height_Then : Bounded; Noise : Floor) return Boolean is
     (Height_Now - Height_Then > Noise);
   --  🔴 "碰到"和"顶住"是两件事,不许压成一条(它们以前共用"零表更准或连着被拒",而那一条同时对应五种原因:
   --  指尖碰到目标 · 别处撞上 · 控制器拒了命令 · 还没生效 · 跟丢了)。分法用的是两个量得到的量:
   --    碰到 = 我在动,而【我没在推的那个东西】也动了(在不跟着这只手动的相机里量);
   --    顶住 = 命令发了而【身体没走】(实到落进本体噪声,连着两步)。
   --  两条都不成立时,这一段不许自称"碰到了"——不确定就继续走或者回去问脑。
   function Touching (Moved_Other : Boolean) return Boolean is (Moved_Other);
   function Fired (U : Until_Kind; W : Watch; Step_Cap : Natural; Blocked : Boolean;
                   Reading, Empty : Bounded; Reading_Noise : Floor; Moved_Other : Boolean := False;
                   Lost : Boolean := False;
                   Height_Now : Bounded := 0.0; Height_Then : Bounded := 0.0;
                   Height_Noise : Floor := 0.0) return Boolean is
     (case U is
         when U_Steps => W.Steps >= Step_Cap,
         when U_Contact => Touching (Moved_Other),
         when U_Resist => Blocked or else Refusing (W),
         when U_Slip => Slipped (Reading, Empty, Reading_Noise),
         when U_Settle => Settled (W),
       when U_Stall => Stalled (W),
         when U_Lost => Lost,
         when U_Free => Came_Free (Height_Now, Height_Then, Height_Noise));
end Monitor;
