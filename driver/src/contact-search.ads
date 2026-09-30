--  接触集的搜索(09-29 起;10-01 路 5 从 Contact.Grasp 搬来改名:它不是"抓",捏住只是其中一种)。替掉 Contact.Gen.Candidates 那张手排的规矩单。
--  一条机制:手是量出来的几块(每块:张开时尖在哪、合拢时往哪走、走多远、多宽);东西是量出来的表面点。
--  候选 = 几何上让这只手真合一次:手按某个进场方向、某个转角、某个位置落下去,每一块沿它量到的那条路合过去,碰到表面点就停 ——
--  停在哪儿就是接触点,那儿的表面法向就是接触法向。量宽度的方向和合的方向是同一个(旧版差 90° 那种错从结构上没了)。
--  硬条件三条,全是量的:手指落下去的地方是空的(落在东西上的不要)、东西伸进手里不超过指尖到手掌那么深、够得着(调用方按量到的关节范围反解)。
--  排序一个数:脑要它怎么动(一个旋量;没说 = 跟着手离开它躺的面),这几处接触连同它躺的那张面(Contact.Wrench:托着它、有摩擦)
--  要让它照那样动,手的法向力之和最少多少(每单位重量),接触法向按量得出的误差取最坏;
--  摩擦按这件东西量到的下限算(它跟手、跟面按同一个数:分开量是 §2 第 13 条的事),从没量过时按
--  "这一批里跟着手离开面最不需要摩擦的那一组,按误差最坏刚好够、再留它自己那份误差"算。
--  没有"比一半好就过线"的门,没有规矩单;最坏的比名义的只大不小 ⇒ 按名义的排好一个个算最坏的,算到后面的名义都比已经挑出来的还大就停(精确,不截断)。
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Contact.Wrench;
with Bytes;
package Contact.Search is
   --  手上一块能碰东西的地方(全在手上那只眼的系里、世界单位,都是量的)
   type Pad is record
      Tip : V3 := [others => 0.0];   --  张开时这一块碰东西的那一面在尖上的那一点
      Dir : V3 := [others => 0.0];   --  合拢时它往哪走(单位向量)
      Travel : Long_Float := 0.0;    --  合到头最多走多远
      Width : Long_Float := 0.0;     --  这一块横着(垂直于合拢方向、垂直于手指)有多宽
      Thick : Long_Float := 0.0;     --  碰东西的那一面后面手指还有多厚(沿合拢方向):手指落下去要这么宽的空
   end record;
   package Pad_Vectors is new Ada.Containers.Vectors (Natural, Pad);
   type Hand_Model is record
      Pads : Pad_Vectors.Vector;
      Tool : V3 := [0.0, 0.0, -1.0];   --  手指伸出去的方向(眼 → 各块尖的中点,单位向量):进场时它朝着东西
      Reach_In : Long_Float := 0.0;    --  东西最多能伸进手里多深(从指尖沿手指往手掌量;= 指尖在眼前面多深)
      Pos_Err : Long_Float := 0.0;     --  手落到哪儿的误差(量的:碰指尖量出来的指尖位置误差);下去之前先合到离料还剩它 + 点的误差 + 一个采样间距
      Valid : Boolean := False;
      Why : Ada.Strings.Unbounded.Unbounded_String;   --  量不成时照实说为什么
   end record;
   --  两块相向合的手(x5 这种):两块的尖都碰桌面量过(碰到的是手指底下正中那一点)⇒ 两块沿两个尖的连线相向走;
   --  指肚宽、手指沿合拢方向多厚都量过(同一种量法:那一瓣在指尖那一截的像素宽 × 深度 ÷ 焦距)⇒ 碰东西的那一面在尖往里半个手指厚处,
   --  两面之间的空各走一半
   function Two_Pads (Tip_A, Tip_B : V3; Width, Thick, Pos_Err : Long_Float) return Hand_Model;
   --  这只手没量全 ⇒ 不成立,照实说
   function Not_Measured (Why : String) return Hand_Model;

   type Candidate is record
      R : Geom.M3 := Geom.Identity;   --  下手那一刻手上那只眼的朝向(眼 → 世界)
      T : V3 := [others => 0.0];      --  下手那一刻眼的位置(各块张开、尖落到下手的高度)
      Approach : V3 := [others => 0.0];   --  进场方向(世界,单位向量,朝东西)
      Touches : Contact.Wrench.Touch_Vectors.Vector;   --  合上以后的接触(世界;法向 = 手往东西里推的方向)
      Travel : Bytes.Floats;          --  每一块从张到头合了多远才碰到
      Pre : Long_Float := 0.0;        --  下去之前每一块先从张到头合多少(一个自由度的手,几块一起合;执行层先合到这儿再下去)
      Half_W, Half_H : Bytes.Floats;  --  每处接触碰到的那一片有多大:沿指肚宽的半宽、沿手指的半高(法向在这两个方向上各准到多少就靠它)
      --  跟着手离开它躺的面(合上以后抬一点验它跟不跟手,验的就是这个;摩擦的上下限按它记)最少要多大的摩擦:按量到的法向 / 法向按量得出的误差取最坏
      Mu_Nom, Mu_Worst : Long_Float := 0.0;
      --  要的动(没说 = 跟着手离开面):手的法向力之和最少多少,每单位重量(摩擦按 Mu_Ref、法向取最坏)—— 排序就按它
      Squeeze : Long_Float := Contact.Wrench.No_Way;
      Width : Long_Float := 0.0;      --  接触点两两之间最远相距多远(记账)
      Com_Off : Long_Float := 0.0;    --  接触的中点离重心的水平距离(重力绕这一组的力臂;记账)
   end record;
   package Cand_Vectors is new Ada.Containers.Vectors (Natural, Candidate);
   --  一次规划的账:试了多少个手的位姿、各被哪一条硬条件挡掉多少,用的摩擦;要的动做不到的有几个、是不是它躺的面挡着
   type Plan_Stats is record
      Poses, Air, Landed_On, Palm_Hit, Unbalanced, Blocked, Unreachable, Kept : Natural := 0;
      No_Force : Natural := 0;        --  这一组怎么配都配不平要的动
      Over_Ub : Natural := 0;         --  跟着手离开面要的摩擦,比这件东西以前没跟上时要的还多(它给不起)
      In_Way : Boolean := False;      --  要的动往它躺的面里去:哪一组都做不到
      Mu_Ref : Long_Float := 0.0;
      Com : V3 := [others => 0.0];
   end record;
   --  Pts = 东西的表面点(世界;只要表面 —— 顶面、侧壁,不要实心往里填的点:法向按指肚看过去的那一层表面拟),Pitch_In = 采样间距(点多时按体素稀疏,门跟着放大),Sigma = 每点的位置误差(量的);
   --  Around = 旁边别的东西的表面点(不含它躺的面):手指落在上面、合的路上先碰到它、顶到手掌的都不要;Up / Support_P = 它躺的面(法向朝上、面上一点);
   --  Mu_Lb = 这件东西量到的摩擦下限(没量过 = 0);Reach = 这个眼的位姿够不够得着(调用方按量到的关节范围反解);
   --  Standoff = 悬停时沿进场方向往回退多远(够不着悬停点的也不要)。Found 按 Squeeze 排好,最多 Want_K 个。
   --  Want = 脑要它怎么动(没说 ⇒ 跟着手离开它躺的面);Mu_Ub = 这件东西量到的摩擦上限(没量过 = 最大)
   procedure Plan (Pts, Around : V3_Vectors.Vector; Pitch_In, Sigma : Long_Float; Up, Support_P : V3; H : Hand_Model; Mu_Lb, Standoff : Long_Float;
                   Reach : not null access function (R : Geom.M3; T : V3) return Boolean;
                   Want_K : Positive; Found : out Cand_Vectors.Vector; St : out Plan_Stats;
                   Want : Contact.Want := (others => <>); Mu_Ub : Long_Float := Long_Float'Last);
end Contact.Search;
