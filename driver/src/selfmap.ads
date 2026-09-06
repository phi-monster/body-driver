--  开机量身体:每个通道推一下再推回来,看每台相机里哪片画面跟着动(部件图)、哪台相机长在哪只手上、
--  一条命令实际交付多少、本体读数抖多少、画面抖多少。探针幅度从极小起翻倍,直到走得出来又看得见。
with Bytes; use Bytes;
with Plug;
with Picture;
with Table;
with Ada.Containers.Vectors;
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
      Floors : Floor_Vectors.Vector;       --  每台相机的静止噪声地板
      Pic_Floor : Ints;                    --  每台相机:整幅画静止时最大灰度差
      Settle : Natural := 2;               --  一条命令发出后读数稳下来要几拍(量出来的)
   end record;

   --  发一条位姿命令并等它稳:返回实际交付(按通道)与用掉的拍数。F 更新到最后一帧。
   procedure Go (L : in out Plug.Link; M : Body_Map; Arm : Natural; Target : Plug.Arm_Pose; Jaw : Floats;
                 F : in out Plug.Frame; Delivered : out Table.Vec; Frames : out Natural; Ok : out Boolean);
   procedure Idle (L : in out Plug.Link; F : in out Plug.Frame; N : Natural; Ok : out Boolean);   --  不下命令空等 N 拍
   --  等到每台相机的画面连着两拍都不再变(各自的灰度地板以内),最多 Max 拍;返回用了几拍
   procedure Wait_Still (L : in out Plug.Link; M : Body_Map; F : in out Plug.Frame; Max : Natural; Used : out Natural; Ok : out Boolean);
   function Pictures_Still (M : Body_Map; Before, After : Plug.Cam_Vectors.Vector) return Boolean;
   procedure Measure (L : in out Plug.Link; F : in out Plug.Frame; M : out Body_Map; Ok : out Boolean);
   function Jaw_Of (F : Plug.Frame; Arm : Natural) return Long_Float;
   function Jaw_Index (F : Plug.Frame; Arm : Natural) return Natural;
end Selfmap;
