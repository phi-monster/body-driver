--  量 → 要它怎么动(大并行 §2 第 16 条,路 5,10-01)。脑说"这件东西的某个量往哪变",或者"它对另一件的某个关系"(两件东西那一句;参照的那一件 = Ref,
--  语言那头等 owner 定、路 7 从句子填)。身体从量到的几何算出让那个量变得最快的那个刚体运动:一个旋量(方向 + 绕哪儿转;模长 1,走多远归执行层按量到的步幅定)。
--  这里只有几何。量到的东西都从参数进来,没有一个约定:"上" = 它躺的面的法向(量的),"横" = 脑看着的那只眼的横轴(量的),"我" = 不跟着动它的那条臂走的那只眼,
--  它在哪、它的长轴、它的底多高、参照那一件在哪、顶面多高多宽,都是量的,合起来的不准是 Sd。缺哪一样就照实说缺哪一样,不编。
--  每个量一种量法:定义写在 Kind 旁边,方向从定义求得(让它变得最快的那个动);它贴着它躺的面时,要的动里往面里去的那一份去掉(面挡着,只在面里走)。
with Ada.Strings.Unbounded;
package Contact.Qty is
   type Kind is
     (Height,    --  它离它躺的面多高(沿"上")
      Heading,   --  它的长轴在它躺的面里朝哪(绕过它中心的"上"那根轴的角;往上 = 按"上"的右手定则正着转)
      Tilt,      --  它斜了多少:绕过它中心的一根水平轴转,这根轴和"上"、和"我看它的方向"都垂直;往上 = 它的顶往远离我的那边倒
      Away,      --  它离我多远(在它躺的面里量:从我那只眼到它中心的水平距离)
      Gap,       --  它和参照那一件之间多远(两件的中心之间)
      Rise,      --  它比参照那一件高多少(沿"上")
      Across,    --  它在参照那一件的哪一侧(脑看着的那只眼里的横向,放平到面里;往上 = 往那只眼的右边)
      Rest_On,   --  离"它躺在参照那一件的顶面上"还差多少:顶面高出不准那么多以上 ⇒ 横着到它正上方 ⇒ 往下;不够高 ⇒ 先往上
      Aim);      --  它的长轴和"它指向参照那一件"的方向差多少(绕"上")

   --  量到的那些东西(各项没量到就是 False,别的格不用看)
   type Scene is record
      Up : V3 := [others => 0.0];               --  它躺的面的法向(单位向量;零 = 没量)
      Center : V3 := [others => 0.0];           --  它的中心(它实心模型的形心,或视线交点)
      Has_Center : Boolean := False;
      Axis : V3 := [others => 0.0];             --  它在面里的长轴(轮廓的主轴;长短两轴分不开 ⇒ 没有)
      Has_Axis : Boolean := False;
      Bottom : Long_Float := 0.0;               --  它的底离它躺的面多高(沿 Up)
      Has_Bottom : Boolean := False;
      Me : V3 := [others => 0.0];               --  我:不跟着动它的那条臂走的那只眼在哪
      Has_Me : Boolean := False;
      View_Right : V3 := [others => 0.0];       --  脑看着的那只眼的横轴(朝右,世界系)
      Has_View : Boolean := False;
      Ref : V3 := [others => 0.0];              --  参照那一件的中心
      Has_Ref : Boolean := False;
      Ref_Top : Long_Float := 0.0;              --  参照那一件的顶面离它躺的面多高(沿 Up)
      Has_Ref_Top : Boolean := False;
      Sd : Long_Float := 0.0;                   --  这些位置合起来的不准(一倍标准差;判"已经到了那一侧 / 那个高度"按 Stats.Z 倍)
      Ang_Sd : Long_Float := 0.0;               --  长轴朝向的不准(弧度,一倍标准差)
   end record;

   --  要的动。Dir = +1 往上 / -1 往下(关系那几个由关系词定,调用方照关系词给);Ok = False ⇒ Note 照实说缺的是哪一样。
   --  已经在那儿了(Rest_On 已经贴着它的顶面、Aim 已经指着它,差的不到不准)⇒ Ok,M 不动(Moving (M) = False),Note 说为什么
   procedure Motion (K : Kind; Dir : Integer; S : Scene; M : out Twist; Ok : out Boolean; Note : out Ada.Strings.Unbounded.Unbounded_String);

   --  松手以后它还在不在原处(大并行路 5,10-01,主代理批的第 3 条):松手那一刻按手带着它算的它的中心 Before(不准 Sd_Before)、
   --  松手、手退开以后重新量到的 After(不准 Sd_After;同一种量法:它的实心模型的形心)。挪的那段比合起来的不准的 Stats.Z 倍还长 ⇒ 它挪了
   --  (倒了、滑了、被手带走了);不准是不是数、是不是负的 ⇒ 判不了,当挪了(不许没量出来就说它躺住了)
   function Moved_Off (Before, After : V3; Sd_Before, Sd_After : Long_Float) return Boolean;

   --  Rest_On 走到哪一段了(给执行层和焊点):先往上 / 横着到正上方 / 往下 / 已经贴着(按 Bottom 和 Ref_Top、它和参照那一件的水平距离)
   type Leg is (Up_First, Over, Down, Resting, Unknown);
   function Rest_Leg (S : Scene) return Leg;
end Contact.Qty;
