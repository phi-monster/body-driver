--  插头:这台机器人通过 msgpack/WebSocket 说话。收观测、回应答、在对方问"给我动作"时把攥着的命令交出去。
--  规矩:①应答的形状由对方定;②没有新命令就重发上一条(空动作 = 这一集到此为止);③线断了在同一个口上等它重接。
with Bytes; use Bytes;
with Layout;
with Msgpack;
with Websocket;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
package Plug is
   type Arm_Pose is array (0 .. 6) of Long_Float;
   package Pose_Vectors is new Ada.Containers.Vectors (Natural, Arm_Pose);
   package Floats_Vectors is new Ada.Containers.Vectors (Natural, Floats, F64_Vectors."=");
   type Cam is record
      W, H : Natural := 0;
      Gray, RGB : Buf;
      Has_Depth : Boolean := False;
      Depth : Floats;
   end record;
   package Cam_Vectors is new Ada.Containers.Vectors (Natural, Cam);
   type Frame is record
      Joints : Floats_Vectors.Vector;   --  每个关节组一串
      EE : Pose_Vectors.Vector;          --  每条臂 xyz + wxyz
      Jaw : Floats_Vectors.Vector;       --  每条臂一串:这条臂【全部】抓握通道的读数
                                         --  (以前只留第一个 ⇒ 五指手的后四根手指整组丢掉)
      Cams : Cam_Vectors.Vector;
      Seq : Natural := 0;
      Instruction : Unbounded_String;    --  观测里带的任务句
   end record;

   type Cmd_Kind is (Hold, Ee, Joint, Base);
   type Cmd is record
      Kind : Cmd_Kind := Hold;
      Arm : Natural := 0;
      Pose : Arm_Pose := [others => 0.0];   --  Ee:绝对位姿
      Jaw : Floats;                          --  这条臂全部抓握通道的目标(空 = 保持读数)
      Q : Floats;                            --  Joint:这一组的绝对关节角
      V : Floats;                            --  Base:速度
   end record;

   type Link is record
      Conn : Websocket.Conn;
      Lay : Layout.Body_Layout;
      Have_Layout : Boolean := False;
      Last : Msgpack.Doc;
      Last_Obs : Integer := -1;
      Pending : Buf;
      Has_Pending : Boolean := False;
      Last_Sent : Buf;
      Has_Last : Boolean := False;
      Reset_Flag : Boolean := False;
      Seq : Natural := 0;
      Ep_Seq0 : Natural := 0;           --  这一集开始时的帧号(对方说 reset 时记下)⇒ 本集用了几拍 = Seq - Ep_Seq0
      Vid_N : Natural := 0;
      Film_N : Natural := 0;
      Wait_Us, Parse_Us : Long_Float := 0.0;
      Frame_S : Long_Float := 0.0;      --  量出来的帧时(秒/帧)
   end record;

   procedure Boot (Port : Natural; L : in out Link; Ok : out Boolean);
   function Sense (L : in out Link; F : out Frame) return Boolean;
   function Act (L : in out Link; C : Cmd) return Boolean;
   function Take_Reset (L : in out Link) return Boolean;
   function Steps (L : Link) return Natural;            --  这一集到现在收了几拍画面(一拍 = 对方走一步;只数,不停)
   function Arms (L : Link) return Natural;
   function Joint_Mode (L : Link) return Boolean;     --  没有末端位姿、只有关节角
end Plug;
