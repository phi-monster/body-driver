--  一次走不完的(大并行.md §2 第 22 条,路 6):手腕转到头就松开、转回、再握,接着转 —— 拧螺丝、转钥匙、倒手挪长东西都是这一段。
--  一个量 q:手拿着那件东西走的那一个数(手腕绕它的轴转了多少弧度、或者手沿一个方向挪了多少)。脑要它变 Want(带正负);
--  "拧紧 = 转不动" ⇒ Want = ±Long_Float'Last,走到转不动为止。手在这个量上能走的范围 [Lo, Hi] 是量的(开机量到的关节范围、够得着的那一截),
--  手此刻在 At_Start;Resolution = 手在这个量上靠得住的最小一步(量的)。
--  ① 拿着往要的方向走(Stroke):一下走到要的、或者走到范围的头(哪个先到);
--  ② 被挡住(Selfmap.Blocked,由 Stroke 报)而离范围的头还不止最小一步 ⇒ 转不动了(Tight;Want 是"到转不动为止"时就是做完了);
--  ③ 到了范围的头还没走够 ⇒ 松开(Release)、手空着走回范围的另一头(Free;回到头 = 下一下能走得最长)、再握(Regrasp:接触集重搜一次),接着走;
--  ④ 再握没握上 ⇒ 照实说(Regrasp_Failed);整个范围都比最小一步短 ⇒ 一下都走不了(No_Room);脑给的步数(Budget,"或者 N 步")用完 ⇒ Out_Of_Steps。
--  一下走多少、回多少全按量到的范围和最小一步定,没有拍的次数;不认单位(弧度、长度都一样走);代码里没有东西的名字、没有动作的名字
package Strokes is
   type Step_Report is record
      Ok : Boolean := False;            --  这一步做没做成
      At_Now : Long_Float := 0.0;       --  走完以后这个量在哪(量的,每一步重量)
      Blocked : Boolean := False;       --  被挡住了(Selfmap.Blocked)
   end record;
   type Strokes_End is (Reached, Tight, Regrasp_Failed, Body_Failed, No_Room, Out_Of_Steps);
   type Report is record
      How : Strokes_End := Body_Failed;
      Done : Long_Float := 0.0;         --  拿着走了多少(带正负,同 Want)
      Strokes : Natural := 0;           --  拿着走了几下
      Regrasps : Natural := 0;          --  松开、转回、再握了几回
      Steps : Natural := 0;             --  身体一共走了几步(拿着的 + 空着的)
   end record;
   procedure Run (Want, Lo, Hi, At_Start, Resolution : Long_Float; Budget : Natural;
                  Stroke : access procedure (D : Long_Float; R : out Step_Report);
                  Release : access procedure (Ok : out Boolean);
                  Free : access procedure (D : Long_Float; R : out Step_Report);
                  Regrasp : access procedure (Ok : out Boolean);
                  Rep : out Report);
   function Say (Rep : Report) return String;
end Strokes;
