--  认出这台机器人的观测长什么样 —— 只看形状与值域,不看键名。
--  6–7 个 ±2π 内的浮点 = 关节角;7 个且后 4 个模长≈1 = 末端位姿;单个 [0,1] = 夹爪;
--  字节 dtype + 三维 shape = 彩色相机;浮点 dtype + 二维 shape 且与某台相机同尺寸 = 深度图(按最长公共路径前缀配对)。
with Bytes; use Bytes;
with Msgpack;
with Ada.Containers.Vectors;
package Layout is
   type Path is record
      Segs : Strs;
   end record;
   package Path_Vectors is new Ada.Containers.Vectors (Natural, Path);
   subtype Paths is Path_Vectors.Vector;
   type Body_Layout is record
      Joints, EE, Jaw, Cams, Depth, Base : Paths;
      Intr : Paths;          --  每台相机的内参矩阵(按形状认:3×3 浮点、针孔样子;没有的那台留空路径)
      Ambiguous : Strs;
      Leaves : Strs;
      --  ── I1(大并行 路 1,10-01):身体报的每一组数,不按"几个数、值在哪"分 —— 开机逐组推一下,量出它是什么(Jointboot.Find_Arms)──
      --  Groups = 每一个数值数组(画面、深度、内参按形状认,不在这里);Twin = 每组:最后一节同名的另一组(-1 = 没有)。
      --  动作按最后一节的名字发(插头的约定),对方观测里同一个名字出现两回 = 它把上一条命令回给我们看了 ⇒ 这个名字是命令。
      --  旧字段 Joints / EE / Jaw 照旧按形状填(别处的旧用处照旧能用);开机按量认完以后 Joints / Jaw 换成量出来的(Set_Measured)
      Groups : Paths;
      Twin : Ints;
      --  开机按量认完(Measured):Holds = 别的命令组(不是臂也不是合拢通道:扛着全身的、一块零件、哑巴、推不动的)—— 对方要每条动作都带齐的键,
      --  发命令时照它们此刻的读数保持;N_Arms = 量出来几条臂;第 A 条臂的合拢通道 = Jaw 里从 Closing_First (A) 起的 Closing_N (A) 组
      --  (一条臂几组都行、可以 0 组;这条臂的抓握读数 = 这几组按顺序接起来)。Jaw 里排在各臂合拢组后面的是它们的回声
      Holds : Paths;
      Measured : Boolean := False;
      N_Arms : Natural := 0;
      Closing_First, Closing_N : Ints;
      Jaw_Len : Ints;        --  Jaw 每一组几个数(开机量的那一刻;一条臂接起来的抓握目标按它切给各组)
   end record;

   procedure Recognise (D : Msgpack.Doc; Obs : Integer; L : out Body_Layout);
   --  "" = 够了:至少一台相机、至少一组数(不要求认得出关节、夹爪:有没有、几组,开机推一下才知道;10-01 原来没有夹爪不开机、
   --  有一组数不像弧度也不像 [0,1] 就"分不开、拒绝硬认")
   function Missing (L : Body_Layout) return String;
   --  命令组:Groups 里有同名另一组的那些(两个都算,先出现的那个当读数、后一个是回声);一组都没有同名的 ⇒ 每一组都当命令组
   --  (对方不回命令,就每一组都推推看;推了不跟的照实说)。下标 = Groups 的下标,按出现的先后
   function Command_Groups (L : Body_Layout) return Ints;
   --  开机按量认的时候:Joints 换成全部命令组(推哪一组都走同一条发关节命令的路,插头每条动作带齐每一个命令键)、Jaw 清空
   procedure Probe_Mode (L : in out Body_Layout);
   --  按量认完:Joints = 量出来的臂和它们的回声,Jaw = 各臂的合拢通道(按臂的顺序;见 Closing_First / Closing_N)和它们的回声,
   --  Holds = 别的命令组
   procedure Set_Measured (L : in out Body_Layout; Joints, Jaw, Holds : Paths; Closing_First, Closing_N, Jaw_Len : Ints; N_Arms : Natural);
   procedure Say (L : Body_Layout);
   function Find (D : Msgpack.Doc; Root : Integer; P : Path) return Integer;   --  节点或 -1
   function Last_Seg (P : Path) return String;
   function Joined (P : Path) return String;
   function Is_Image (D : Msgpack.Doc; N : Integer; W, H : out Natural) return Boolean;
   function Is_Depth (D : Msgpack.Doc; N : Integer; W, H : out Natural) return Boolean;
   --  3×3 浮点、[f 0 cx; 0 f cy; 0 0 1] 的样子 = 针孔内参(只看形状与值的样子,不看键名)
   function Is_Intrinsic (D : Msgpack.Doc; N : Integer; F, Cx, Cy : out Long_Float) return Boolean;
end Layout;
