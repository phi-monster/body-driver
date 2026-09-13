--  体检:一台机器上每一样【能被程序引用的东西】,必须先在这里拿到一个判决。
--  判决四种,只有 Usable 允许出现在程序里 —— 【未经证明的量,不许参与动作】。
--  这不是一份贴在墙上的报告,是【否决权】:编译器只收 Usable,其余一律编译错误,
--  并且必须给出一句人话(为什么)和一个能照抄的替代(那你可以改说什么)。
--  纯判断,不连线也能跑:拿一份存下来的身体文件就能出判决书。
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
with Selfmap;
with Learned;
package Exam is

   --  Usable   量到了,而且重复几次一致 ⇒ 可以被引用
   --  Unproven 量到了,但没重复测过 ⇒ 不知道稳不稳 ⇒ 一样不许引用(这是"没有未经证明的一步"的全部含义)
   --  Unstable 重复测过,几次结果互相打架 ⇒ 不许引用
   --  Dead     推遍所有通道,它一次都没动过 ⇒ 这个量在这具身体上【不存在】
   type Verdict is (Usable, Unproven, Unstable, Dead);

   --  画面里被量的五样,和响应表的五行一一对应
   type Row_Id is (Sideways, Updown, Nearness, Bigness, Facing);

   type Row_Check is record
      V : Verdict := Dead;
      Per_Notch : Long_Float := 0.0;   --  一格探针把这一行推动多少(这一行自己的单位)
      Best_Chan : Integer := -1;       --  推得最动的是哪个通道
      Spread : Long_Float := 0.0;      --  重复几次的散布 ÷ 均值(比例,无量纲)
      Reps : Natural := 0;             --  重复测了几次(次数,无量纲)
      Why : Unbounded_String;
      Instead : Unbounded_String;
   end record;
   type Row_Checks is array (Row_Id) of Row_Check;

   --  一块被跟着的东西(我的一个零件,或世界里的一团)在某台相机里的五行判决
   type Thing_Check is record
      Arm, Cam : Natural := 0;
      Kind : Learned.Track_Kind := Learned.Piece_Pt;
      Chan_K : Natural := 0;
      Blob : Integer := -1;
      Rows : Row_Checks;
      Live_Rows : Natural := 0;
      Loudest, Faintest : Long_Float := 0.0;   --  活着的行里,一格效果最大的与最小的
      Ratio : Long_Float := 1.0;               --  两者之比(比例,无量纲):不做归一化时,这就是最响那行的话语权倍数
   end record;
   package Thing_Vectors is new Ada.Containers.Vectors (Natural, Thing_Check);

   type Chan_Check is record
      V : Verdict := Dead;
      Arm, K : Natural := 0;
      Amp, Delivered, Obey : Long_Float := 0.0;   --  Obey = 实到 ÷ 命令(比例,无量纲)
      Why : Unbounded_String;
   end record;
   package Chan_Vectors is new Ada.Containers.Vectors (Natural, Chan_Check);

   --  眼睛活没活,不能用"它安静不安静"来判 —— 一只死掉的眼睛最安静。
   --  只能用【这一帧画面自己内部有没有明暗差】来判:全黑或糊死的眼睛,这一项是 0。
   type Eye_Check is record
      V : Verdict := Dead;
      Still_Floor : Natural := 0;      --  静止时最大灰度差(它小只说明安静,不说明活着)
      Contrast : Natural := 0;         --  一帧之内的灰度跨度(0 = 瞎了)
      Has_Contrast : Boolean := False; --  这一项到底有没有被量过
      Rides_On : Integer := -1;        --  长在哪只手上(-1 = 不跟着任何一只手动)
      Moves : Long_Float := 0.0;       --  手一动它变了多少画面(比例,无量纲)
      Why : Unbounded_String;
   end record;
   package Eye_Vectors is new Ada.Containers.Vectors (Natural, Eye_Check);

   type Report is record
      Things : Thing_Vectors.Vector;
      Chans : Chan_Vectors.Vector;
      Eyes : Eye_Vectors.Vector;
      Self_Noise_Never_Measured : Boolean := False;  --  本体读数噪声量出来恰好是 0 ⇒ 所有"动得比噪声大吗"的闸永远为真
      World_Cam_By_Stillness : Boolean := False;     --  主相机是按"变化最少"挑的 ⇒ 一只死眼永远夺冠
   end record;

   --  纯判断:输入是身体量到的东西,输出是判决书。不动电机、不连线。
   function Judge (M : Selfmap.Body_Map; Tables : Learned.Effect_Vectors.Vector) return Report;

   --  一个量能不能被程序引用 —— 编译器唯一该问的问题
   function Allowed (R : Row_Check) return Boolean is (R.V = Usable);

   procedure Say (R : Report);
   function Row_Name (X : Row_Id) return String;
   function Verdict_Name (V : Verdict) return String;
end Exam;
