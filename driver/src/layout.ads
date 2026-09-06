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
      Ambiguous : Strs;
      Leaves : Strs;
   end record;

   procedure Recognise (D : Msgpack.Doc; Obs : Integer; L : out Body_Layout);
   function Missing (L : Body_Layout) return String;      --  "" = 够了
   procedure Say (L : Body_Layout);
   function Find (D : Msgpack.Doc; Root : Integer; P : Path) return Integer;   --  节点或 -1
   function Last_Seg (P : Path) return String;
   function Joined (P : Path) return String;
   function Is_Image (D : Msgpack.Doc; N : Integer; W, H : out Natural) return Boolean;
   function Is_Depth (D : Msgpack.Doc; N : Integer; W, H : out Natural) return Boolean;
end Layout;
