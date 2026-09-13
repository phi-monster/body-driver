--  身体学到的东西:一张【就地】的响应表 —— 在这个位姿、这台相机里、这一块东西上,
--  每个通道动一点,画面里那五样各变多少。表是量出来的,不是算出来的;换了位姿或手里换了东西就不成立。
--  从 Act 里挪出来,好让【体检】能在不依赖执行器的前提下审判它(执行器反过来依赖体检)。
with Plug;
with Table;
with Ada.Containers.Vectors;
package Learned is
   type Track_Kind is (Piece_Pt, Thing_Pt);   --  Piece_Pt:我身上的一块零件(Chan_K = 带它的通道;握合通道 = Chan.Per_Arm,那块就是手指)
   type Stored_Effect is record
      Arm, Cam : Natural := 0;
      Kind : Track_Kind := Piece_Pt;
      Chan_K : Natural := 0;     --  带这块的通道(Chan.Per_Arm = 握合通道)
      Blob : Integer := -1;      --  这块的第几团(-1 = 整块;手指 0/1 = 两指各自)
      E : Table.Effect;
      Trust : Table.Mask := [others => True];   --  探针时这个点真跑过地板的通道
      Reach : Table.Vec := [others => 1.0];   --  每个通道各自被核实过的步幅(探针上限的倍数)
      Pose : Plug.Arm_Pose := [others => 0.0];   --  这张表是在哪个位姿下量的:表是【就地】的,离得远了不成立
      Has_Pose : Boolean := False;
      Held : Integer := -1;      --  量这张表的时候手里是什么(-1 = 空手,否则是那一槽)
   end record;
   package Effect_Vectors is new Ada.Containers.Vectors (Natural, Stored_Effect);
end Learned;
