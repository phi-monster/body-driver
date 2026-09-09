--  握区:合空一次,每台相机里"合拢通道扫过的像素"= 手指;两瓣之间那片 = 能装东西的区域;
--  一瓣(吸盘、腔)= 那一块自己。区的中心、主轴、张幅、深度全从画面量;两指、五指、吸盘同一段代码。
with Bytes; use Bytes;
with Plug;
with Picture;
with Selfmap;
with Ada.Containers.Vectors;
package Zone is
   type Lobe is record
      Valid : Boolean := False;
      X0, Y0, X1, Y1 : Natural := 0;
      Cu, Cv : Long_Float := 0.0;
      Count : Natural := 0;
   end record;
   type Hand_Zone is record
      Valid : Boolean := False;
      Cu, Cv : Long_Float := 0.0;      --  区心(归一化)
      Au, Av : Long_Float := 0.0;      --  瓣到瓣的方向(单瓣时为主轴)
      Span : Long_Float := 0.0;        --  瓣心距(归一化画幅)
      Depth : Long_Float := 0.0;       --  手指深度(米);NaN = 读不到
      N_Lobes : Natural := 0;
      A, B : Lobe;
      Fingers : Bools;                 --  扫过的像素(手指本身)
      X0, Y0, X1, Y1 : Natural := 0;   --  区框
   end record;
   package Zone_Vectors is new Ada.Containers.Vectors (Natural, Hand_Zone);
   type Hand is record
      Arm : Natural := 0;
      Zones : Zone_Vectors.Vector;     --  每台相机一个
      Empty_Close : Long_Float := 0.0; --  合空时的读数
      Empty_Sweep : Long_Float := 0.0; --  合空时,合拢过程中动过的像素占画面的几分之几(比例,无量纲)。
                                       --  真合一次如果动过的明显更少,说明手指被中间的东西挡住了 —— 这是
                                       --  【量出来的"夹住了没有"】,不看画面判断,也不需要力传感器;
                                       --  这台机器人的爪子读数是命令的回声(见 LAB),所以读数那条路不能用。
      Open_Reading : Long_Float := 1.0;
      Close_Steps : Natural := 0;
      Pose : Plug.Arm_Pose := [others => 0.0];   --  合空时这只手的位姿(别的相机里的握区只在这个位姿下成立)
   end record;
   package Hand_Vectors is new Ada.Containers.Vectors (Natural, Hand);

   procedure Measure (L : in out Plug.Link; M : Selfmap.Body_Map; Arm : Natural; F : in out Plug.Frame; H : out Hand; Ok : out Boolean);
   --  从"合空扫过的像素 + 张开时的深度 + 合上时的深度"算出握区(纯函数,可离线测):
   --  近的那一拨(张开时就在近处)= 手指;扫过但张开时是远处 = 手指合拢时要盖过的地方 = 能装东西的区。
   function From_Sweep (Swept : Bools; Depth_Open, Depth_Closed : Floats; Has_Depth : Boolean; W, Hh : Natural) return Hand_Zone;
   --  把手指像素从深度切块结果里剔掉(块心落在手指框或区框里 = 我自己)
   function Is_Self (Z : Hand_Zone; R : Picture.Region; W, Hh : Natural) return Boolean;
end Zone;
