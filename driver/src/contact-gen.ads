--  ②a 下手点生成器 —— 零学习,纯几何。眼前这块表面上,这具身体真正能用的下手点有哪些。
--  它替掉的是【我的手】:力臂往哪挪 · 爪面朝哪 · 挪多少会挪出物体外 —— 全是人在拍脑袋,形状一换就废。
--  包围盒是不够的(实测):包围盒说"这一段 6 厘米实心",剪刀的真身是两片薄刃夹一条缝 —— 按盒子选的下手点,爪子从缝里合过去,指间什么都没有。
--  所以这里吃的是表面点,不是盒子。身体常数全是传进来的参数,不是这里读出来的(方向单向:反过来就破了"换机体不重训"那堵墙)。
--  2026-08 用 Rust 写成、每一条排序规矩都是真抓失败逼出来的(commit ef10664 contact-gen/src/{lib,support,hands}.rs);逐段搬回 Ada,数一个没改。
with Ada.Containers.Vectors;
package Contact.Gen is
   --  爪能张多开 —— 它的出处必须跟着它走。本仓最贵的一次手填就在这个量上:驱动里是拒绝态,代码里却一直用着手填的 8 cm。
   type Span_Source is (Measured, Declared, Unknown);
   type Jaw_Span is record
      Source : Span_Source := Unknown;   --  Measured 才能当事实;Declared 能用但每条结果都背着"这是声明值";Unknown ⇒ 不许猜,整层拒绝出候选
      Metres : Long_Float := 0.0;
   end record;
   --  这具身体的尺寸,全部由驱动量出来后传进来;这里一个字面量都没有
   type Gripper is record
      Jaw : Jaw_Span;
      Reach_Lo, Reach_Hi : Long_Float := 0.0;   --  够得到的半径带(离臂根,米)—— 必须是带腕姿约束量出来的那一份
      Base_X, Base_Y : Long_Float := 0.0;       --  臂根在世界里的水平位置,米
   end record;
   --  切多少层、量多少个方向:可观测性参数(分辨率),不是身体常数。三个长度是身体量:爪面本身有多高(一层的厚度就是它,不许由"物体高度 ÷ 层数"定)、
   --  指头本身有多宽(量宽度只能量指头覆盖到的那一条:实测爪停在哪与整层跨度的相关系数只有 0.256)、离支撑面至少多高才伸得进去(下限,不是越高越好:
   --  写成最大化会专挑鞋口那圈软皮)。Gap_M 是采样密度的函数:两个表面点隔多远就不算同一块料。全部由调用方给,这里不填缺省数。
   type Grid is record
      Bands : Positive := 1;
      Dirs : Positive := 1;
      Min_Pts : Positive := 1;
      Jaw_H_M : Long_Float := 0.0;
      Min_Above_M : Long_Float := 0.0;
      Finger_W_M : Long_Float := 0.0;
      Gap_M : Long_Float := 0.0;
   end record;
   --  一条下手点候选 —— 接触集的几何那一半。剩下那一半(往哪使劲 · 物体怎么动)由动词和眼睛填,不在这一层。
   type Candidate is record
      Pos : V3 := [others => 0.0];        --  这一段截面的中心,世界坐标,米
      Close_Yaw : Long_Float := 0.0;      --  合爪方向在水平面里的朝向,弧度;爪面垂直于它
      Width_M : Long_Float := 0.0;        --  这一段沿合爪方向有多宽(料的一段有多厚,不是最左到最右有多远:甜甜圈量出来是圈边,不是外径)
      Margin_M : Long_Float := 0.0;       --  爪张开度减去这一段的宽度
      Above_Support_M : Long_Float := 0.0;
      Reach_R : Long_Float := 0.0;
      Reachable : Boolean := False;
      Jaw_Declared : Boolean := False;    --  用声明值(而非实测值)的爪张开度排出来的,一路传到落盘
      Within_Jaw : Boolean := False;      --  跨度在爪张开度以内吗。只用来排序,永远不用来删候选(放不下的段仍然留在表里,只是垫底)
      N_Pts : Natural := 0;
      Depth_M : Long_Float := 0.0;        --  沿指头方向,厚度还差不多的那一段有多长:指头压在又深又匀的料上才咬得住;压在尖端上接触面积趋零
      Face_Tilt_Rad : Long_Float := 0.0;  --  两个夹持面离"正对着"歪了多少:摩擦锥那一条,写成不需要 μ 的形式(排序只要"谁更小")
      Com_Offset_M : Long_Float := 0.0;   --  离这团点的重心多远(水平):管"提起来会不会转出去";摩擦锥管"横着滑走",两件事都得算
      Off_Ok, Tilt_Ok, Com_Ok : Boolean := False;   --  排序用的过线/不过线(后两个的门槛 = 这一批候选自己的中位数,不拍角度)
      Seq : Natural := 0;                 --  生成顺序:排序的最后一键,让同分的保持生成顺序
   end record;
   package Cand_Vectors is new Ada.Containers.Vectors (Natural, Candidate);
   --  为什么一条候选都给不出来:拒绝要说得出理由,不许静默返回空表。No_Section 不表示"这具身体夹不住这个东西"(从数字判可抓性被禁)
   type Refusal is (Fine, Jaw_Span_Unknown, Too_Few_Points, Flat, No_Section);
   --  主入口:一堆表面点 + 这具身体 → 能用的下手点,已排序:够得到 → 跨度在爪张开度以内 → 离支撑面够高(下限)→ 面正对(过线)→ 离重心近(过线)→ 料越深越前
   procedure Candidates (Pts : V3_Vectors.Vector; G : Gripper; Support_Z : Long_Float; Gd : Grid; Found : out Cand_Vectors.Vector; Why : out Refusal);
   --  爪子停在这儿,这块料多厚?量的是"从外面合过来会碰到什么"(两片刃相距 7 cm 时跨的是两片刃的外缘,不是单片刃的 9 mm)。那儿没有料 ⇒ Ok = False,不是 0
   function Thickness_At (Pts : V3_Vectors.Vector; Px, Py, Pz, Close_Yaw, Band_H_M, Finger_W_M : Long_Float; Ok : out Boolean) return Long_Float;
   --  交接给接触集时,为什么交不出去
   type Handoff_Kind is (Fine, Mu_Unknown, Would_Slip);
   type Handoff is record
      Kind : Handoff_Kind := Fine;
      Need_Rad, Have_Rad : Long_Float := 0.0;   --  Would_Slip:两个夹持面歪了 Need,摩擦锥只有 Have
   end record;
   --  把一条候选变成一个接触集(四格 + 进场方向):两个接触点 = 中心 ± (宽/2) × 合爪方向,法向朝外,锥朝里、半张角 = atan(μ)。
   --  μ 必须由调用方给(身体×世界的耦合,拿指头在参考面上蹭一下量得出来);没量过就 Mu_Unknown;Face_Tilt > atan(μ) 就 Would_Slip 并报差多少
   procedure To_Set (C : Candidate; Mu : Long_Float; Motion : Twist; Tol_M : Long_Float; S : out Set; Why : out Handoff);

   --  支撑面不水平的机器:把点云转到"支撑面法向 = +z"的那个系里算,算完再转回来。算法一个字不用动,假设变成显式输入。
   --  它买到的是朝向无关,不是重力无关:哪一面是支撑面仍然由调用方说(这一层不知道重力往哪儿,也不该知道)
   type Rot is record
      Axis : V3 := [0.0, 0.0, 1.0];
      Ang : Long_Float := 0.0;
   end record;
   function Between (From, To : V3; Ok : out Boolean) return Rot;   --  把 From 转到 To 的最小旋转;两者反向时任取一条垂直轴
   function Inverse (R : Rot) return Rot;
   function Dir (R : Rot; V : V3) return V3;                       --  转一个方向 / 绕原点转一个点
   function Rotate (R : Rot; S : Set) return Set;                  --  点、法向、锥轴、旋量、进场方向一个都不能漏
   procedure To_Upright (Cloud : in out V3_Vectors.Vector; Support_Normal : V3; Back : out Rot; Ok : out Boolean);

   --  另外两种手:吸盘(1 点)· 环抓(n 点)。同一张接触集表,三条不同的几何路径填:表不认识机体,机体各自算各自的
   type No_Hand_Kind is (Fine, Too_Few_Points, No_Flat_Patch, Nothing_In_Direction, Not_Surrounding, Handed_Off);
   type No_Hand is record
      Kind : No_Hand_Kind := Fine;
      Found_R, Need_R : Long_Float := 0.0;   --  No_Flat_Patch:实测的最大平坦半径 / 要求的半径
      Direction : Natural := 0;              --  Nothing_In_Direction:第几个方向摸不到料
      H : Handoff;                           --  Handed_Off:转发原因
   end record;
   --  吸盘:找一片够大、够平的面,给出一个接触点。Cup_R_M 是身体常数(量出来传进来);Flat_Tol_M 是采样噪声的函数,不是身体常数。
   --  取铺得最满的那一片(不是最高的那一片);"背面"(薄板另一面,偏离量不随半径变)筛掉,"弯曲"(偏离量随半径长大)判死
   procedure Suction (Cloud : V3_Vectors.Vector; Cup_R_M, Flat_Tol_M, Mu : Long_Float; Motion : Twist; Tol_M : Long_Float; S : out Set; Why : out No_Hand);
   --  环抓:在一个高度上绕物体一圈,给出 N 个接触点(三指 = 3,五指 = 5)。每个方向上取最外那个真表面点;N 个内法向必须正张成平面,否则那不是握是推
   procedure Ring (Cloud : V3_Vectors.Vector; At_Z, Band_M : Long_Float; N : Positive; Mu : Long_Float; Motion : Twist; Tol_M : Long_Float; S : out Set; Why : out No_Hand);
   function Sampling_Gap (Cloud : V3_Vectors.Vector) return Long_Float;   --  点云自己的采样间距:每个点到最近邻的距离取中位。量得出来就不许拍
   function Img (R : Refusal) return String;
   function Img (H : Handoff) return String;
   function Img (N : No_Hand) return String;
end Contact.Gen;
