--  一件东西的估计(大并行第 0 步 I4;路 3 填,路 4 / 5 / 6 读;§2 第 10 条)。
--  一件东西的形状只有一个估计,量到的约束全放进去:
--    · 每一只看过它的眼的每一眼(View):那一刻那只眼在世界里的位姿、框它的窗、窗里它的像素 —— 它一定在那只眼的轮廓锥里;
--    · 它躺的面:它在面上方;
--    · 碰到它的点、量到的它表面上的点(两只眼配上点交出来的、深度读到的):那儿一定是它。
--  一眼里哪儿算"不是它"(一件东西是连着的一块,它在一只眼里的投影也是连着的一块):它的投影 = 量到的那一片(掩膜)
--  + 它被挡着的那几截。被挡着的只能在【挨着掩膜的那几片挡着的像素】里(我自己的手指、别的东西的像素;挡着的像素一片连一片地挨过去都算);
--  掩膜顶着窗边 ⇒ 它可能有一截在窗外,窗外全算说不出(分割只在窗里作数;10-01 路 5 量到:当场的窗只盖住东西的一部分,C1 第 102 拍 482 / 1704 点);
--  它或挨着它的挡着的那几片顶到画幅边 ⇒ 它可能伸到画幅外(Beyond)。这些之外的像素 —— 窗里没被挡着的、掩膜没顶着窗边时窗外没被挡着的 —— 都不是它。
--  (10-01 箱上 C1 重放:原来"挨着手指 ⇒ 窗外全算说不出",腕眼那几眼只在窗里雕,外包顺着头顶眼的锥一直伸到 10 单位高)
--  解法(Solve):八叉树按轮廓雕 —— 一格投进某只眼,它的投影(按那只眼的像素不准 ⊕ 位姿不准放宽 Z 倍)里一个它的像素都没有、
--  这只眼又说得上话 ⇒ 空;在每只说得上话的眼里都整个落在它的像素里 ⇒ 实;别的拆成八格接着问,拆到"再细哪只眼也分不出"
--  (格子投进去不比放宽的那一圈两倍大)为止。实和空交界的那些最细的格子 = 表面:每点带外法向、不准、哪一面被眼看过、几眼说得上话。
--  看不见 ≠ 没有:贴着它躺的面那一面、没有眼对着的那几面照实标成没看过;雕出来的是它的外包(凹进去的地方雕不出来),
--  一个方向上它多厚,只靠从别的方向看的那几眼夹着 —— 都写在 Note 里。
--  透明、反光、没纹理的东西配不上点,靠的就是轮廓(分割)和碰。
--  会动、会变形的:新的一眼和已经雕出来的外包对不上(它的像素平均落在外包投影外面比轮廓自己的不准还远)⇒ 它动过了,以前的眼全作废(Moved)。
with Geom;
with Bytes; use Bytes;
with Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
package Things is
   subtype V3 is Geom.V3;

   --  一眼
   type View is record
      Cam : Geom.Cam_Geo;              --  那一刻那只眼在世界里:R_Ce = 相机 → 世界,Pos,F,Cx,Cy,K1,K2(同 Geom 不动的眼的约定;Fixed = True)
      Cam_Index : Natural := 0;        --  第几台相机
      Px_Sd : Long_Float := 0.0;       --  轮廓的像素不准(像素;那只眼的像素残差 ⊕ 分割的边量出来的抖动,Add_View 合)
      Pos_Sd : Long_Float := 0.0;      --  那只眼的位置有多不准(世界单位;手放进世界的不准、不动的眼解出来的不准)
      W, H : Natural := 0;
      X0, Y0, X1, Y1 : Integer := 0;   --  窗(闭区间,像素):掩膜只在窗里作数
      Mask : Bools;                    --  整幅 W × H:它的像素
      Occl : Bools;                    --  整幅 W × H(可以空 = 没有):挡在它前面、也许挡着它的像素(腕眼自己的手指 = 开机量的指图;别的东西的像素)——
                                       --  挡着的地方后面有没有它说不出,不算"不是它"
      Seq : Natural := 0;              --  那一拍的帧号
      --  Add_View 填(Reach,见上):
      Unknown : Bools;                 --  它可能在、可又没量到的像素:挨着掩膜的那几片挡着的;掩膜顶着窗边时再加窗外全部
      Beyond : Boolean := False;       --  它可能伸到画幅外
      Whole : Boolean := False;        --  这只眼看得见的它整个在掩膜里(Unknown 空、不伸到画幅外)
   end record;
   package View_Vectors is new Ada.Containers.Vectors (Natural, View);

   --  表面上的一点
   type Surf_Pt is record
      P : V3 := [0.0, 0.0, 0.0];
      N : V3 := [0.0, 0.0, 0.0];       --  外法向(单位;0 = 说不出)
      Sd : Long_Float := 0.0;          --  外包的边界本身定得多准(世界单位;最细格子的半边长 ⊕ 最准那一眼在那儿的一个标准差)
      Seen : Boolean := False;         --  这一面有眼看过(有一只说得上话的眼在它外头那一侧)
      Touched : Boolean := False;      --  碰到过
      Looks : Natural := 0;            --  几眼在这一点说得上话(1 = 只有一眼:沿那一眼的视线它在哪只靠外包夹着)
   end record;
   package Surf_Vectors is new Ada.Containers.Vectors (Natural, Surf_Pt);

   type Touch is record
      P : V3 := [0.0, 0.0, 0.0];
      Sd : Long_Float := 0.0;
      Seq : Natural := 0;
   end record;
   package Touch_Vectors is new Ada.Containers.Vectors (Natural, Touch);
   --  量到的它表面上的一点(两只眼配上点交出来的、深度相机读到的):那儿一定是它。From = 从哪儿看见的。
   --  (眼和它之间那条视线是空的;一条视线雕不出一格,只记着。法向说不出 ⇒ 表面点里它的 N = 0)
   type Seen_Pt is record
      P : V3 := [0.0, 0.0, 0.0];
      Sd : Long_Float := 0.0;
      From : V3 := [0.0, 0.0, 0.0];
      Seq : Natural := 0;
   end record;
   package Seen_Vectors is new Ada.Containers.Vectors (Natural, Seen_Pt);

   --  位置随时间(给会动的,路 4 读):每次解完记一笔它的中心、不准、帧号
   type Track_Pt is record
      Seq : Natural := 0;
      C : V3 := [0.0, 0.0, 0.0];
      Sd : Long_Float := 0.0;
   end record;
   package Track_Vectors is new Ada.Containers.Vectors (Natural, Track_Pt);

   --  一格(八叉树的):中心、半边长
   type Cell is record
      C : V3 := [0.0, 0.0, 0.0];
      H : Long_Float := 0.0;
      Looks : Natural := 0;            --  几眼在这一格说得上话(碰到过 / 量到过表面点也算一眼)
   end record;
   package Cell_Vectors is new Ada.Containers.Vectors (Natural, Cell);

   --  它的一个部件(路 6 填;一个都没有 = 一整块,没量过):它那一份表面点、它绕哪根轴转 / 沿哪根轴滑(没量 ⇒ No_Axis)
   type Axis_Kind is (No_Axis, Turn, Slide);
   type Part is record
      Surface : Surf_Vectors.Vector;
      Axis : Axis_Kind := No_Axis;
      Axis_P, Axis_D : V3 := [0.0, 0.0, 0.0];
      Axis_Sd : Long_Float := 0.0;
   end record;
   package Part_Vectors is new Ada.Containers.Vectors (Natural, Part);

   type Estimate is record
      Name : Ada.Strings.Unbounded.Unbounded_String;
      Views : View_Vectors.Vector;
      Touches : Touch_Vectors.Vector;
      Points : Seen_Vectors.Vector;     --  量到的表面点
      Has_Support : Boolean := False;   --  它躺的面(过 Support_P、法向 Support_N 朝它那一侧)
      Support_P, Support_N : V3 := [0.0, 0.0, 0.0];
      --  解出来的(Solve)
      Valid : Boolean := False;
      Surface : Surf_Vectors.Vector;    --  表面点
      Solid : Cell_Vectors.Vector;      --  外包:没被雕掉、有眼说过话的格子(实的 + 压着轮廓的)
      Unseen : Cell_Vectors.Vector;     --  没眼说过话的格子:不算它,也不能说不是它(新的一眼核"动没动"时连它一起算它可能在的地方)
      Pitch : Long_Float := 0.0;        --  表面点的间距(最细那一层格子的边长,世界单位)
      --  外包里至少两眼说得上话的那一截(Core):沿一眼的视线它伸多远,得有从别处看的一眼夹着才算数。
      --  只一眼说得上话的那几截(那一眼锥里别的眼看不到 / 投不进画幅的地方)也在外包里,不在 Core 里:那儿有没有它说不出
      Lo, Hi : V3 := [0.0, 0.0, 0.0];   --  Core 的外接盒(世界系)
      Center : V3 := [0.0, 0.0, 0.0];   --  Core 的体积形心
      Hull_Lo, Hull_Hi : V3 := [0.0, 0.0, 0.0];   --  整个外包的外接盒
      One_Look_Frac : Long_Float := 0.0;   --  外包的体积里只一眼说得上话的那一份
      Center_Sd : Long_Float := 0.0;
      Seen_Frac : Long_Float := 0.0;    --  表面点里有眼看过的那一份
      N_Eyes : Natural := 0;            --  有几台相机看过它
      Spread : Long_Float := 0.0;       --  看它的那几眼,方向两两夹角里最大的(弧度):越小,沿视线方向它多厚越只靠外包夹着
      Thick_Unknown : Boolean := False; --  所有的眼都从同一处看它(眼心挪的比放宽那一圈还小)⇒ 沿视线它多厚只靠它躺的面和眼夹着
      Note : Ada.Strings.Unbounded.Unbounded_String;   --  照实说:几只眼、几眼、哪几面没看过、是外包
      Track : Track_Vectors.Vector;
      Moves : Natural := 0;             --  新的一眼和外包对不上、判成"它动过了"几次
      Solved_Seq : Natural := 0;        --  上一次解的时候最新那一眼的帧号(核"动没动"用的外包最多旧一拍)
      Inconsistent : Boolean := False;  --  上一次解:几眼的锥交不到一起 / 整个雕没了(几眼互相对不上:它在几眼之间动过)
      --  别的路填的(这里只占位,默认值就是"没量"):部件和轴(路 6;没量 = 一整块)、谁拿着它(路 5 / 6;-1 = 没人)
      Parts : Part_Vectors.Vector;
      Held_Arm : Integer := -1;
   end record;
   package Estimate_Vectors is new Ada.Containers.Vectors (Natural, Estimate);

   --  这一眼加进它的估计:先算它在这一眼里可能在的像素(Reach)、合上分割的边量出来的抖动;同一台相机、眼心没挪出放宽那一圈的上一眼换掉(同一个锥);
   --  已经解过一次(有外包)而这一眼和外包对不上 ⇒ 它动过了(Moved,Moves + 1)。这里不重解(每一拍都会进来,重解要好几秒),Solve 才重解
   procedure Add_View (E : in out Estimate; V : View);
   procedure Add_Touch (E : in out Estimate; T : Touch);
   procedure Add_Point (E : in out Estimate; P : Seen_Pt);
   procedure Set_Support (E : in out Estimate; P, N : V3);
   --  它动过了(被推、被拿起、自己走):以前的眼、碰到的点、量到的表面点、雕出来的格子都作废(位置随时间那一串留着)
   procedure Moved (E : in out Estimate);
   --  按全部的眼、面、碰到的点重解。Need = 调用方要多细(世界单位,表面点的间距;0 = 眼分得出多细就多细):
   --  比要的还细只多花工夫,拿去下手的人说出自己的不准(比如它量到的指尖不准),就不必拆到眼的分辨率(C1 重放:拆到 0.5 mm,68 万格、13 秒)。
   --  几眼的锥交不到一起 / 整个雕没了 ⇒ 它在几眼之间动过:只留最新那一拍的几眼重解一次(Moves + 1)
   procedure Solve (E : in out Estimate; Need : Long_Float := 0.0);
   --  这台相机量出来的掩膜边的抖(像素,一个标准差;同一只眼没挪没转的两眼比出来的中位数;没量过 ⇒ 像素量化的 1 / √12)
   function Edge_Sd (Cam : Natural) return Long_Float;
   --  这一眼里它可能在、可又没量到的像素(Unknown)、可能伸到画幅外(Beyond)、整个在掩膜里(Whole)—— 按掩膜、窗、挡着的像素算(见上)
   procedure Reach (V : in out View);
   --  = Reach 以后的 Whole
   function Whole_In_Window (V : View) return Boolean;

   --  点 X 按这一眼:在它的轮廓锥外面(空)/ 里面 / 压在轮廓上(Mixed)/ 说不出(落在它可能在、可又没量到的像素上、画幅外而它可能伸出去、眼后面);
   --  投影按这一眼的像素不准 ⊕ 位姿不准放宽 Z 倍
   type Verdict is (Free, Inside, Mixed, No_Info);
   function Point_In (V : View; X : V3) return Verdict;

   --  一个驱动一份:每件叫得出名字的东西一个估计(按名字找;找不到 ⇒ 新开一个)。换集清空(World.Reset_All)
   procedure Clear_All;
   function Index_Of (Name : String) return Integer;
   function Get (Name : String) return Estimate;
   --  解过的那一份(没解过、或者解完又添了眼 ⇒ 现解,存回去);Need 同 Solve
   function Solved (Name : String; Need : Long_Float := 0.0) return Estimate;
   procedure Put (E : Estimate);
   --  脑把它改了名(路 7 改名时一起调):估计跟着名字走;新名字已经有一份 ⇒ 两份的眼、碰到的点并成一份
   procedure Rename (From, To : String);
   function Count return Natural;
   function Get_At (I : Natural) return Estimate;
end Things;
