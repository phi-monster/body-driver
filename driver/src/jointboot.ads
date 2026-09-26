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

   --  World_Cam = 不长在哪只手上的相机里、手动时变得最少的那台(每台都长在手上 ⇒ -1)
   procedure Find_Arms (L : in out Plug.Link; F : in out Plug.Frame; M : in out Selfmap.Body_Map;
                        Arms : out Arm_Vectors.Vector; World_Cam : out Integer; Ok : out Boolean);

   --  一只手扫描下来的全部格子
   type Sweep_Data is record
      Frames : Kinem.Frame_Vectors.Vector;   --  每一格的读数 + 扫的是哪个关节(起点 = -1)
      Imgs : Plug.Cam_Vectors.Vector;        --  每一格手上那只眼的画面
      Runs : Ints;                           --  第几段(同一个关节同一个方向算一段;起点 = 0)
      W, H : Natural := 0;
      Ids : Ints;                            --  每一格在配点仪器那边存的编号(Instrument.Frame_Put;-1 = 没存成)
      World_Img : Plug.Cam;                  --  不动的眼(头顶眼)在扫描起点那一刻的画面(没有不动的眼 = 空)
      World_Id : Integer := -1;              --  它在配点仪器那边的编号
   end record;
   package Sweep_Vectors is new Ada.Containers.Vectors (Natural, Sweep_Data);
   package Corr_Set_Vectors is new Ada.Containers.Vectors (Natural, Kinem.Corr_Vectors.Vector, Kinem.Corr_Vectors."=");

   --  ② 关节扫描:有眼的几只手同时扫(一条命令带几组目标),每个关节两个方向一格一格转;到头 / 被顶住 / 别的关节被顶偏 / 走满 3 格就停,
   --  直接去下一段的头一格(回起点和下一段头一格是同一个动作)。每一段头一格和起点那一对当场配点:画面挪了多少 ÷ 实到的转角 = 这个关节
   --  每个读数单位挪几像素,往后每格按它定步子(一格挪画幅宽的 1/5)。配点(起点 ↔ 每一格、每段头两格、相邻关节头一格之间、几个关节一起动的相邻两格)
   --  由配点仪器配(粗配、单向);头一格那几对扫描时配,别的扫完再配。
   --  Host / Port = 仪器;Dump 非空 = 落盘 sweep_*.bmp + sweep.txt。Ds / Css 和 Arms 里有眼的手一一对应(没眼的那只 Ds 空)
   --  World_Cam = 不动的眼是第几台(Find_Arms 认的;-1 = 没有):起点那一刻它的画面也存下,对齐几只手用
   procedure Sweep_All (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; Arms : Arm_Vectors.Vector;
                        Host : String; Port : Natural; Dump : String; Ds : out Sweep_Vectors.Vector; Css : out Corr_Set_Vectors.Vector;
                        World_Cam : Integer := -1);

   --  ④ 这只手的运动学:Kinem.Fit(配点来自扫描的配对)
   --  Note = 这一步的报告(几只手各开一个线程同时解 ⇒ 不在这里打印,解完由调用方按顺序打)
   procedure Fit_Arm (A : Natural; D : Sweep_Data; Cs : Kinem.Corr_Vectors.Vector; Dump : String; M : out Kinem.Model; Ok : out Boolean;
                      Note : out Ada.Strings.Unbounded.Unbounded_String);

   --  ⑤ 世界:每只手的运动学在它自己参照读数时那只眼的系里 ⇒ 拿看得见整张桌子的不动的眼(头顶眼)当桥对到一个系:
   --  每只手从自己扫描的格子里三角出桌面点;这些点在头顶眼里的像素 = 腕眼那一格配到头顶眼、再配回来,往返 1 px 内的才算;
   --  第一只手的点当板解头顶眼(焦距 + 在第一只手系里的位姿,Geom.Fit_Fixed_Board);别的手:落在它自己桌面上的点,
   --  头顶眼那条视线交第一只手系里的桌面 ⇒ 同一个点在两个系里 ⇒ 相似变换(抗野点)。长度倍数靠同一张桌面(只靠一只不动的眼,绕它缩放分不出来)。
   --  (V1B11 2026-09-26:原来用两只腕眼起点那一格互相配 —— 两只眼看桌子两头、一点不重叠,倍数解成 0.72、真 1.007)
   --  "上" = 桌面法向(朝第一只手的眼那边),原点 = 第一只手参照眼在桌面上的垂足,x = 那只眼的 x 轴投到桌面上;长度单位 = 第一只手的模型单位。
   --  Fixed_Eye = 解出来的头顶眼(世界系:R_Ce = 相机 → 世界,Pos;Valid = False 就是没解成)
   type Arm_World is record
      Group : Natural := 0;
      Model : Kinem.Model;
      S : Long_Float := 1.0;                  --  这只手参照眼系 → 第一只手参照眼系:X0 = S · Ra · X + Ta
      Ra : Geom.M3 := Geom.Identity;
      Ta : Geom.V3 := [0.0, 0.0, 0.0];
      Lo, Hi : Floats;                        --  扫描时每个关节实际到过的两头(反解不出这个范围:只去量过的地方)
      Valid : Boolean := False;
      Sweep : Natural := 0;                   --  扫描数据(Sweep_All 的 Ds)里是第几只手
   end record;
   package Arm_World_Vectors is new Ada.Containers.Vectors (Natural, Arm_World);
   --  Dump 非空 = 落盘 align_arm<k>.txt(第一行 = 相似变换;每一条配点:世界里哪只眼(第几只手、第几格;-1 = 不长在手上的眼)、这只手的第几个三角点、
   --  那只眼里的像素、这只手那一格里的像素、往返差、放进世界后的点、这只手系里的点)、
   --  fixed_eye.txt(头顶眼,第一只手的系里)、world.txt(世界系:Rw、O),离线回放 / 打分用
   --  Board / Plane_* 交给开机后半段(V1b 09-27):板 = 放进世界的手三角出、配进不动的眼的点(世界系位置和协方差、在不动的眼那张起点画面里的像素、
   --  那一批往返差换成的每轴噪声);Plane_Pt / Plane_N = 世界系里的桌面(原点就在桌面上、法向 = +z),Plane_Rms = 桌面上的点离面的离散(标准差)
   procedure Align (Ds : Sweep_Vectors.Vector; Worlds : in out Arm_World_Vectors.Vector; Css : Corr_Set_Vectors.Vector;
                    Host : String; Port : Natural; Rw : out Geom.M3; O : out Geom.V3; Ok : out Boolean; Fixed_Eye : out Geom.Cam_Geo;
                    Board : out Geom.Scene_Pt_Vectors.Vector; Plane_Pt, Plane_N : out Geom.V3; Plane_Rms : out Long_Float; Dump : String := "");

   --  ⑥ 装上:从此插头每一帧的手的位姿 = 按关节读数算出的世界里的腕眼位姿;位姿命令 = 在量过的范围里解关节目标
   procedure Install (Worlds : Arm_World_Vectors.Vector; Rw : Geom.M3; O : Geom.V3);
   --  插头的两个钩子(Install 登记)
   procedure Pose_Hook (F : in out Plug.Frame);
   procedure Cmd_Hook (C : in out Plug.Cmd; Ok : out Boolean);
   --  开机自检(V1b 的 ②):每只装上的手走到扫描时没去过的几处 —— 两格"几个关节一起动"的读数的正中(每个关节都在量过的范围里),
   --  按运动学算出那一处眼的位姿当目标,按位姿命令同一条路(Pose_To_Q:在量过的范围里反解)解成关节目标;几只手同时走(一条命令带几组目标),
   --  停稳后记:目标、反解还差多少、实到的读数。身体报的位姿只落盘给离线打分(Dump/ik_check.txt),驱动不读
   procedure Self_Check (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; Ds : Sweep_Vectors.Vector; Dump : String);
end Jointboot;
