--  开机前半段:只用关节命令(V1b 第三步,2026-09-26)。身体报不报"手在哪"都不读 —— 运动学量好之前,手是按关节目标挪的。
--  ① 认身体:每组关节读数一起转一小格,哪台相机整幅都变、而且比第二名多一倍 = 长在这只手上的眼(同 Selfmap.Measure 的判法);
--     跟着一起变的别的组 = 同一只手的回声组(命令的回显),不单算一只手;所有手动时变得最少的那台 = 世界相机。
--  ② 每只有眼的手做关节扫描(每个关节单独两个方向一格一格转;到头 / 被顶住 / 别的关节被顶偏 / 走满 12 格停),每一格的画面和读数留在内存;
--  ③ 手指遮罩:整段扫描里画面一次都没变过的那些像素 = 跟着眼一起动的自己的手指(或者什么都没有的空白),配点不要它们。
with Plug;
with Selfmap;
with Kinem;
with Geom;
with Bytes; use Bytes;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
package Jointboot is
   type Arm_Info is record
      Group : Natural := 0;          --  这只手的关节读数是第几组(F.Joints 的下标)
      Eye : Integer := -1;           --  长在它上面的那台相机(-1 = 没有)
      Frac : Floats;                 --  这只手一动,每台相机变了多少画面(比例)
      Probe : Long_Float := 0.0;     --  认出来时每个关节一起转了多少(读数的单位)
      Echoes : Ints;                 --  跟着一起变的别的组(回声)
   end record;
   package Arm_Vectors is new Ada.Containers.Vectors (Natural, Arm_Info);

   procedure Find_Arms (L : in out Plug.Link; F : in out Plug.Frame; M : in out Selfmap.Body_Map;
                        Arms : out Arm_Vectors.Vector; World_Cam : out Natural; Ok : out Boolean);

   --  一只手扫描下来的全部格子
   type Sweep_Data is record
      Frames : Kinem.Frame_Vectors.Vector;   --  每一格的读数 + 扫的是哪个关节(起点 = -1)
      Imgs : Plug.Cam_Vectors.Vector;        --  每一格手上那只眼的画面
      Runs : Ints;                           --  第几段(同一个关节同一个方向算一段;起点 = 0)
      Mask : Bools;                          --  手指遮罩(W × H,按行;是 = 整段扫描里一次都没变过)
      W, H : Natural := 0;
   end record;
   package Sweep_Vectors is new Ada.Containers.Vectors (Natural, Sweep_Data);
   package Corr_Set_Vectors is new Ada.Containers.Vectors (Natural, Kinem.Corr_Vectors.Vector, Kinem.Corr_Vectors."=");

   --  ② 关节扫描:有眼的几只手同时扫(一条命令带几组目标),每个关节两个方向一格一格转;到头 / 被顶住 / 别的关节被顶偏 / 走满 8 格就停,转回起点。
   --  每一段开头在每只手的眼里铺一片格点(认出来那一下动过的像素里)交给跟点仪器,每一格问一次它们到哪了:
   --  配点(起点 ↔ 每一格、相邻两格、不同关节头两格之间)全从这批格点来,每格画面挪了多少也从它读(按它放大 / 缩小下一格)。
   --  Host / Port = 仪器;Dump 非空 = 落盘 sweep_*.bmp + sweep.txt。Ds / Css 和 Arms 里有眼的手一一对应(没眼的那只 Ds 空)
   procedure Sweep_All (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; Arms : Arm_Vectors.Vector;
                        Host : String; Port : Natural; Dump : String; Ds : out Sweep_Vectors.Vector; Css : out Corr_Set_Vectors.Vector);

   --  ④ 这只手的运动学:Kinem.Fit(配点来自扫描时的跟点;手指遮罩里的不要)
   --  Note = 这一步的报告(几只手各开一个线程同时解 ⇒ 不在这里打印,解完由调用方按顺序打)
   procedure Fit_Arm (A : Natural; D : Sweep_Data; Cs : Kinem.Corr_Vectors.Vector; Dump : String; M : out Kinem.Model; Ok : out Boolean;
                      Note : out Ada.Strings.Unbounded.Unbounded_String);

   --  ⑤ 世界:每只手的运动学在它自己参照读数时那只眼的系里 ⇒ 用两只眼都看得见的桌面点对齐(各自三角、跨手配点、相似变换);
   --  "上" = 桌面法向(朝第一只手的眼那边),原点 = 第一只手参照眼在桌面上的垂足,x = 那只眼的 x 轴投到桌面上;长度单位 = 第一只手的模型单位
   type Arm_World is record
      Group : Natural := 0;
      Model : Kinem.Model;
      S : Long_Float := 1.0;                  --  这只手参照眼系 → 第一只手参照眼系:X0 = S · Ra · X + Ta
      Ra : Geom.M3 := Geom.Identity;
      Ta : Geom.V3 := [0.0, 0.0, 0.0];
      Lo, Hi : Floats;                        --  扫描时每个关节实际到过的两头(反解不出这个范围:只去量过的地方)
      Valid : Boolean := False;
   end record;
   package Arm_World_Vectors is new Ada.Containers.Vectors (Natural, Arm_World);
   procedure Align (Ds : Sweep_Vectors.Vector; Worlds : in out Arm_World_Vectors.Vector; Css : Corr_Set_Vectors.Vector;
                    Host : String; Port : Natural; Rw : out Geom.M3; O : out Geom.V3; Ok : out Boolean);

   --  ⑥ 装上:从此插头每一帧的手的位姿 = 按关节读数算出的世界里的腕眼位姿;位姿命令 = 在量过的范围里解关节目标
   procedure Install (Worlds : Arm_World_Vectors.Vector; Rw : Geom.M3; O : Geom.V3);
   --  插头的两个钩子(Install 登记)
   procedure Pose_Hook (F : in out Plug.Frame);
   procedure Cmd_Hook (C : in out Plug.Cmd; Ok : out Boolean);
end Jointboot;
