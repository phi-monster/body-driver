--  开机量身体:每个通道推一下再推回来,看每台相机里哪片画面跟着动(部件图)、哪台相机长在哪只手上、
--  一条命令实际交付多少、本体读数抖多少、画面抖多少。探针幅度从极小起翻倍,直到走得出来又看得见。
with Bytes; use Bytes;
with Plug;
with Picture;
with Table;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
package Selfmap is
   type Part is record
      Valid : Boolean := False;
      X0, Y0, X1, Y1 : Natural := 0;
      Cu, Cv : Long_Float := 0.0;
      Count : Natural := 0;
      Frac : Long_Float := 0.0;
   end record;
   package Part_Vectors is new Ada.Containers.Vectors (Natural, Part);
   package Floor_Vectors is new Ada.Containers.Vectors (Natural, Picture.Floor_Map, Picture."=");
   type Body_Map is record
      Arms : Natural := 0;
      N_Cams : Natural := 0;
      Per_Arm : Natural := 6;
      Channels : Natural := 0;             --  Arms × Per_Arm
      Parts : Part_Vectors.Vector;         --  (通道 × N_Cams + 相机)
      Cam_Frac : Floats;                   --  (臂 × N_Cams + 相机):这只手一动,那台相机变了多少画面
      Cam_On_Arm : Ints;                   --  每只手:长在它上面的相机号,-1 = 没有
      World_Cam : Natural := 0;            --  变得最少的那台
      Amp : Floats;                        --  每通道:走得出来又看得见的探针幅度
      Delivered : Floats;                  --  每通道:那一幅度实际交付了多少
      Seen : Bools;                        --  每通道:有没有一台相机看见它动
      EE_Noise : Long_Float := 0.0;        --  本体位置读数抖多少(米)
      Rot_Noise : Long_Float := 0.0;       --  本体姿态读数抖多少(弧度)
      Jaw_Noise : Long_Float := 0.0;
      Joint_Noise : Long_Float := 0.0;     --  关节读数不动时抖多少(读数的单位;V1b 2026-09-26:按关节目标挪手时"停稳"看它)

      Floors : Floor_Vectors.Vector;       --  每台相机的静止噪声地板
      Pic_Floor : Ints;                    --  每台相机:整幅画静止时最大灰度差
      Jaws : Ints;                         --  每条臂量到几个抓握通道(五指手 5,两指手 1)
      Settle : Natural := 2;               --  一条命令发出后读数稳下来要几拍(量出来的)
      --  越用越强:历次量到的幅度/实到(现值取中位数),量过几次
      Amp_Hist : Plug.Floats_Vectors.Vector;
      Deliv_Hist : Plug.Floats_Vectors.Vector;
      Measured_Times : Natural := 0;
   end record;
   --  快速核对:每只手推一个通道(存的幅度),实到和存的差一半以内且画面里看得见 ⇒ 身体没变
   type String_Note is record
      Text : Ada.Strings.Unbounded.Unbounded_String;
   end record;
   procedure Verify (L : in out Plug.Link; M : Body_Map; F : in out Plug.Frame; Ok_Body, Ok_Link : out Boolean; Note : out String_Note);

   --  发一条位姿命令并等它稳:返回实际交付(按通道)与用掉的拍数。F 更新到最后一帧。
   --  Quick:脑说"快"—— 过了量出来的稳定拍数就走,不再等读数连着两拍不动
   --  Watch:走的途中每一拍问一句"出事了没"(被跟的东西快出画面 / 看不见了);说出事就当拍把目标改成"停在这儿",不走完这一步
   --  Group >= 0:这一条发的不是位姿,是第 Group 组关节读数的目标 Joints(开机一个关节一个关节扫,V1b 2026-09-26);
   --  Groups / Qs 非空:同一条命令给几组读数各自的目标(几只手一起扫)。同一条发命令的路(Target / Jaw 这时不用)。
   --  关节目标的"停稳":读数到了目标 Tol 以内再有一拍不动就算到(Tol = 0 不这样判),否则连着两拍不动
   --  位姿目标:"停了" = 连着两拍,这一拍挪的不到这条命令的百分之一 —— 平移、转动都折成"在自己那只眼里挪几个像素"来比
   --  (1 像素 = 给了的 Tol / Tol_Rot;没给 ⇒ Fold_P / Fold_R;再没给 ⇒ 这只手量过的"一步看得见的那一档" M.Amp),静止噪声只当下限;
   --  给了 Tol(平移)/ Tol_Rot(转动)还判"到了":位姿到了目标这么近连着两拍就算到(没给不判:一步那么小的命令一开始就在"一步以内")。
   --  09-28 H4:原来没给档时按"连着两拍挪不到读数噪声"判停 —— x5 上停下以后十几微米的蠕动要等 13–24 拍(V1B21),
   --  人形的静止噪声量在上一个动作的尾巴上(0.012 单位)、比一步还大,推一步 2 拍就算停(实到 61%)。都没有像素单位 ⇒ 只能按读数噪声判
   type Watcher is access function (F : Plug.Frame) return Boolean;
   procedure Go (L : in out Plug.Link; M : Body_Map; Arm : Natural; Target : Plug.Arm_Pose; Jaw : Floats;
                 F : in out Plug.Frame; Delivered : out Table.Vec; Frames : out Natural; Ok : out Boolean; Quick : Boolean := False;
                 Watch : Watcher := null; Joints : Floats := F64_Vectors.Empty_Vector; Group : Integer := -1;
                 Groups : Ints := Int_Vectors.Empty_Vector; Qs : Plug.Floats_Vectors.Vector := Plug.Floats_Vectors.Empty_Vector;
                 Tol : Long_Float := 0.0; Tol_Rot : Long_Float := 0.0;
                 Tols : Plug.Floats_Vectors.Vector := Plug.Floats_Vectors.Empty_Vector;
                 Fold_P : Long_Float := 0.0; Fold_R : Long_Float := 0.0);
   --  Tols(和 Qs 同形:每组每个关节一道门)给了 ⇒ "到了" = 每个关节差不到它自己那道门(关节目标);"停了"的门照旧按 Tol。
   --  开机扫描用:扫的那根差不到这一格的三分之一,别的关节差不到每根轴单独起步收格子的门(Kinem.Clean_Tol;H1 2026-09-28)
   --  一组关节这一拍"到了没有"(Go 里用的就是它;纯函数,导出给自检):Tols 这一位 > 0 ⇒ 这个关节按它自己的门,否则按 Tol;门 ≤ 0 的关节永远不算到
   function Joints_Arrived (Now, Target, Tols : Floats; Tol : Long_Float) return Boolean;
   --  位姿这一拍"停了没有"(Go 里用的就是它;纯函数,导出给自检):这一拍挪的(Moved_P 平移、Rot 转动)折成像素(Fp / Fr = 1 像素的平移 / 转动)
   --  不到这条命令(Cmd:平移三项、转动三项)折成像素的百分之一;静止噪声(Noise_P / Noise_R)折成像素只当下限。Fp 或 Fr 不 > 0 ⇒ 只按读数噪声
   function Pose_Still (Moved_P, Rot : Long_Float; Cmd : Table.Vec; Fp, Fr, Noise_P, Noise_R : Long_Float) return Boolean;
   --  量静止噪声之前上一个动作还在不在收(Measure_Idle 用的就是它):这一拍的变化比上一拍小(平移或转动任一样)= 还在收
   function Still_Settling (Dp, Dr, Last_P, Last_R : Long_Float) return Boolean is (Dp < Last_P or else Dr < Last_R);
   procedure Idle (L : in out Plug.Link; F : in out Plug.Frame; N : Natural; Ok : out Boolean);   --  不下命令空等 N 拍
   --  什么都不做时读数抖多少、画面抖多少(静止对,4 拍):位姿 / 姿态 / 抓握 / 关节读数的噪声 + 每台相机的灰度地板。
   --  Measure 开头用它;只报关节的身体开机前半段(还没有位姿)也用它(同一种量法)。M.Arms 条臂的位姿噪声(没有位姿 = 0)
   procedure Measure_Idle (L : in out Plug.Link; F : in out Plug.Frame; M : in out Body_Map; Ok : out Boolean);
   --  等到每台相机的画面连着两拍都不再变(各自的灰度地板以内),最多 Max 拍;返回用了几拍
   --  Prev_Pic 给了 ⇒ 等完时里面是最后一帧之前那一帧(两帧都是画面停下以后的:抓握通道推到头时"看没看见动了"两次比较、不共用一帧用,Picture.Seen_Twice)
   procedure Wait_Still (L : in out Plug.Link; M : Body_Map; F : in out Plug.Frame; Max : Natural; Used : out Natural; Ok : out Boolean;
                         Prev_Pic : access Plug.Cam_Vectors.Vector := null);
   function Pictures_Still (M : Body_Map; Before, After : Plug.Cam_Vectors.Vector) return Boolean;
   --  只看第 Cam 台:两帧之间超过噪声地板的像素凑不成一团
   function Picture_Still (M : Body_Map; Before, After : Plug.Cam; Cam : Natural) return Boolean;
   --  Eyes / World:开机前半段只用关节命令已经认出了"哪台相机长在哪只手上、哪台是世界相机"(Jointboot.Find_Arms)⇒ 照用,
   --  这里不再按位姿探针另认一遍(一个量一种量法);空 = 这里认
   --  Step_Px:每只手"一步看得见"的幅度 = 在它自己那只眼里画面挪 1 像素(V1b 09-27;第 A 个 = [平移, 转动]:平移 = 眼离桌面的距离 ÷ 焦距,
   --  转动 = 1 ÷ 焦距 弧度,开机前半段量的)。每个通道按它推一次再推回来:量走没走到(Delivered)、哪块跟着动(零件)。
   --  原来从极小起翻倍、推过去推回来两张图都变了一块就算看见 —— V1B17 仿真渲染噪声下第 1 只手推 0.0003 单位(约 0.016 mm)就被噪声凑成"看见了"。
   --  没有这一项的手(没量成运动学)⇒ 它的通道量不了
   procedure Measure (L : in out Plug.Link; F : in out Plug.Frame; M : out Body_Map; Ok : out Boolean;
                      Step_Px : Plug.Floats_Vectors.Vector;
                      Eyes : Ints := Int_Vectors.Empty_Vector; World : Integer := -1);
   function Jaw_Of (F : Plug.Frame; Arm : Natural; K : Natural := 0) return Long_Float;
   function Jaw_Count (F : Plug.Frame; Arm : Natural) return Natural;   --  这条臂量到几个抓握通道
   function Jaw_All (F : Plug.Frame; Arm : Natural) return Floats;      --  这条臂全部抓握通道此刻的读数
   function Jaw_Index (F : Plug.Frame; Arm : Natural) return Natural;
end Selfmap;
