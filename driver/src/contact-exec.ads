--  ②b 执行层:接触集 → 一串航点。闭式,零学习,不认识任何动词。
--  这里没有 match verb:十三个动词之所以能塌成一张模板,是因为差别全在接触集里,不在这一层里。这一层只做一件事:
--  把"这几个点要这样动"翻译成"手要依次到哪几个位姿"。一旦这里出现 if 动词 = 拧,那张模板就白设计了 —— 那说明差别没被接触集吃掉。
--  姿态是完整朝向,不是一个偏角:腕压死朝下时够得到 0.419 m,不压死是 0.602 m(2026-08-16 实测),那 18 cm 里住着一整晚的"命令发出去而手一步没动"。
--  2026-08 用 Rust 写成(commit ef10664 contact-exec/src/plan.rs),航点级验收逐条搬回 Ada。
with Geom;
package Contact.Exec is
   subtype M3 is Geom.M3;   --  一只手的完整朝向:列 = (开合轴 x, y, 工具轴 z) 在世界里
   use type Geom.M3;
   package M3_Vectors is new Ada.Containers.Vectors (Natural, M3);
   type Step_Kind is (Hover, Touch, Carry, Dodge);
   --  一个该发的航点。它描述的是"这几个接触点各自该在哪",不是"末端在哪" —— 末端在哪是身体层按自己的运动学去解的事
   type Step is record
      Pos : V3_Vectors.Vector;      --  每个接触点这一刻该到的世界位置(吸盘 1 个、五指 5 个),与接触集的手接触点一一对应
      Frame : M3_Vectors.Vector;    --  每个接触点它那只手这一刻的完整朝向(同一只手的点填同一个值;双臂抱一个箱子是两个手腕,各算各的)
      Hand : Nat_Vectors.Vector;    --  每个点归哪一只手(编号不带语义)
      Touching : Boolean := False;  --  这一刻算不算已经接触(True = 允许有力,False = 只是路过)
      Tol_M : Long_Float := 0.0;    --  这一步的容差 = 参与的那些点里最严的那一个
      Kind : Step_Kind := Hover;    --  只为日志与判据;执行层自己不读它(Dodge 时身体层保持当前朝向,Frame 只是占位)
   end record;
   package Step_Vectors is new Ada.Containers.Vectors (Natural, Step);
   --  这具身体在这一层需要的东西,全部由调用方量了再递进来
   type Hand_Limits is record
      Standoff_M : Long_Float := 0.0;   --  从接触点往回退多远算"悬停":这具身体的进场余量,不是场景常数
      Repeat_M : Long_Float := 0.0;     --  这具身体自己的重复精度;容差不许比它更紧 —— 比它紧就是要求身体做不到的事
   end record;
   --  出不了航点时,点名是哪一格或哪一条
   type No_Plan_Kind is
     (Fine,
      Bad,                     --  接触集自己就没填对 —— 转发那一格(G)
      Tol_Tighter_Than_Body,   --  容差比这具身体的重复精度还紧(Index = 哪一点):拍过 5 mm 的门槛,而落点残差中位就是 3.5 mm ⇒ 一半的段被判偏,合爪一次都没执行到
      No_Frame,                --  那几个接触点张不成一个朝向
      No_Hand_Contact,         --  ① 里一个手的接触都没有,全是世界那一侧的约束
      Many,                    --  一串/并存那一层就填不满 —— 转发(M)
      Cannot_Clear);           --  躲不开:在有限的候选方向里找不到一个同时离开所有要躲之处的位置(候选是有限的,所以是"我找不到",不是"不存在")
   type No_Plan is record
      Kind : No_Plan_Kind := Fine;
      G : Gap;
      Index : Natural := 0;
      M : Many_Gap;
   end record;
   function Img (N : No_Plan) return String;
   function Tool_Axis (F : M3) return V3;   --  第三列
   --  从"每点的法向 + 用力方向"算出工具该怎么摆:工具轴 = 进场方向(接触集给的那一项;没给就用各点用力方向的合;合成为零再用各点法向的反向合;再为零就拒绝),
   --  开合轴 = 相距最远的一对手接触点的连线投到与工具轴垂直的平面上(单点时任取一条垂直的)。这就是"手腕该怎么摆"被算出来而不是被挑出来的地方
   function Frame_From (S : Set; Ok : out Boolean) return M3;
   --  接触集 → 一串航点:悬停(沿各自那只手的工具轴往回退 Standoff,容差放宽到进场余量)→ 贴上(容差用最严的)→ 按③把接触点分 Arc 段搬过去,手跟着物体转。
   --  转必须分步:一步到位等于让手沿直线穿过物体,而接触点是沿圆弧走的 —— 撬、翻、倒、拧全都吃它。不动的动词到"贴上"就结束,那也是一个完整的计划
   procedure Steps (S : Set; L : Hand_Limits; Must_Move : Boolean; Arc : Positive; Out_Steps : out Step_Vectors.Vector; Why : out No_Plan);
   --  一串 / 并存 → 一串航点。"够"(Reach)不需要变体:每一段的第一个航点(悬停)就是它,执行层自己产生过渡。
   --  Keep:除第一段外把每段开头的悬停扔掉(手已经握着东西在那儿),并把前面几段已经转过的角带进来(不带的话手腕悄悄转回去);
   --  Meanwhile:朝向由维持的那一段定;Clear:一步、永远不接触。过渡的避障没做:那是世界属性(学),不该在这一层里硬编
   procedure Script (M : Move; L : Hand_Limits; Must_Move : Boolean; Arc : Positive; Out_Steps : out Step_Vectors.Vector; Why : out No_Plan);
   --  把一个点让到"离每一个要躲的地方都至少 By_M 远"的位置。不是"从最近那个推开"(会推向另一个);取有限的候选方向,各算出"走多远才彻底出了所有的球",取最短
   function Dodge_To (P : V3; Keep_Out : V3_Vectors.Vector; By_M : Long_Float; Ok : out Boolean) return V3;
   --  走完一段之后:偏了没有。只是路过的点永远不算偏;要碰的点,门槛只能由这具身体自己的重复精度给
   function Off_Course (L : Hand_Limits; St : Step; Residual_M : Long_Float) return Boolean;
end Contact.Exec;
