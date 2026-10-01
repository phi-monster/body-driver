--  同时两套接触(大并行.md §2 第 19 条,路 6):脑一次说两个要 —— 一件东西 A 不动,它上面的一块 B 沿两块之间那根轴动
--  (握住 + 扣扳机、按住 + 拧开盖子、按住 + 拉开抽屉都是这一句)。物理检查把所有接触和两个要一起算(准静态,同 Contact.Wrench):
--  ① B 推得动:碰 B 的每一处,沿它允许的方向(摩擦锥里)往 B 沿轴动的方向使劲 —— 锥里和"B 在这一点沿轴动的速度"夹角最小的那个方向
--     (锥轴转向速度、转到锥边为止);做的功是正的才算推得动它(B 自己要多大的劲不知道:轴上的摩擦、弹簧没量过 ⇒ 只问方向)。
--  ② B 不动的那一部分(轴那里的约束力)和推它的力都传到 A 上:B 只沿轴动、本身不算重量 ⇒ A 受到的就是碰 B 那几处使的力,
--     作用在那几处。A 要不动:握着 A 的那几处(连同 A 躺的面)要抵得住它的重量,还要抵得住这个力。
--     B 要多大的劲不知道,推 B 的力从零到很大都可能 ⇒ 两样分开各算一次(Contact.Wrench.Need / Least),两样都抵得住,加起来也抵得住
--     (抵得住的力和力矩是一个凸锥:两个在里面,它们的正组合都在里面)。
--  做不到时照实说是哪一条。不认单位(力按方向和比例,同接触集的第②格);代码里没有东西的名字、没有动作的名字
with Contact.Wrench;
package Linkage.Together is
   type Why_Kind is (Fine, Axis_Unknown, Not_Driven, A_Falls, A_Pushed_Away);
   type Report is record
      Why : Why_Kind := Axis_Unknown;
      Driving : Natural := 0;                       --  碰 B 的几处推得动它(做的功是正的)
      F, Mo : V3 := [others => 0.0];                --  推 B 的力(合起来)和绕 Ref 的力矩(每处使一个单位的劲,方向按 ①)
      Ref : V3 := [others => 0.0];                  --  力矩绕哪一点(碰 B 那几处的中点)
      Need_Weight : Long_Float := Contact.Wrench.No_Way;   --  握着 A 的那几处抵住 A 的重量,手的法向力之和最少多少(每单位重量)
      Need_Push : Long_Float := Contact.Wrench.No_Way;     --  抵住推 B 的那个力,手的法向力之和最少多少(每单位推 B 的力)
      Why_A : Contact.Wrench.Why_Kind := Contact.Wrench.Fine;
   end record;
   --  Ts_A = 握着 A 的那几处(N = 手往 A 里推的方向);Com_A = A 的重心;Up = 上;Sup = A 躺的面(没有 = Contact.Wrench.No_Surface);
   --  Ts_B = 碰 B 的那几处;Allowed_Half = 碰 B 那几处摩擦锥的半张角(atan 摩擦系数,这只手量的);J、Pa = 两块之间的轴、A 此刻的位姿
   --  (Linkage.Fit 量的);Sign = B 沿轴往哪边动(+1 / −1,脑要的)
   procedure Check (Ts_A : Contact.Wrench.Touch_Vectors.Vector; Com_A, Up : V3; Sup : Contact.Wrench.Surface;
                    Ts_B : Contact.Wrench.Touch_Vectors.Vector; Allowed_Half : Long_Float;
                    J : Joint; Pa : Pose; Sign : Long_Float; Mu_Hand, Mu_Surf : Long_Float; Rep : out Report);
   function Say (Rep : Report) return String;
end Linkage.Together;
