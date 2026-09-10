--  世界:每台相机各自记"切出来的块"进槽(跨轮对号,槽号稳定);看不见的留影子;握着的东西跟手走。
with Picture;
with Ada.Containers.Vectors;
package World is
   type Slot is record
      Present : Boolean := False;
      R : Picture.Region;
      Seen : Boolean := False;
      Shadow : Picture.Region;      --  上次看见的样子
      Pinned : Boolean := False;    --  🔴 脑亲口指出来的那个东西:永远留在清单里,不许因为"这一帧没对上"就说看不见。
                                    --  没有深度的时候切块本来就常常对不上,而这个东西是脑指的,不是切出来的。
   end record;
   package Slot_Vectors is new Ada.Containers.Vectors (Natural, Slot);
   type Cam_State is record
      Slots : Slot_Vectors.Vector;
      Named : Integer := -1;        --  脑上次点名的槽
   end record;
   package Cam_Vectors is new Ada.Containers.Vectors (Natural, Cam_State);
   type State is record
      Cams : Cam_Vectors.Vector;
      Holding : Boolean := False;
      Held_Arm : Integer := -1;
      Held_Cam : Integer := -1;
      Held_Slot : Integer := -1;
      Held_Origin : Picture.Region;  --  合上时它在桌上最后的样子
   end record;
   procedure Init (S : in out State; N_Cams : Natural);
   procedure Reset_All (S : in out State);
   procedure Observe (S : in out State; Cam : Natural; Regs : Picture.Regions; W, H : Natural);
   procedure Pin (S : in out State; Cam : Natural; U, V : Long_Float);   --  把最靠近 (U,V) 的槽标成"脑指的"
   function Count (S : State; Cam : Natural) return Natural;
   function Get (S : State; Cam : Natural; I : Natural) return Slot;
   function Vanished (Regs : Picture.Regions; Origin : Picture.Region; W, H : Natural) return Boolean;
end World;
