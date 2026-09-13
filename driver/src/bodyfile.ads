--  身体文件:这台机器自己量到的身体,跟着它走。开机装回,推一下核对,对不上的格才重量;重量的结果和旧的合成(取中位数),不覆盖。
--  只存身体量(通道幅度与实到、相机归属、噪声地板、手上相机里的握区、响应表初值);世界、任务、世界相机里的位置一概不存。
--  钥匙 = 这具身体报回来的形状(几条臂、几个抓握通道、几台相机、多大画幅)。
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Selfmap;
with Zone;
with Act;
with Plug;
with Schema;
package Bodyfile is
   function Fingerprint (L : Plug.Link; F : Plug.Frame) return String;
   --  读回:钥匙对得上才装;返回是否装上了。装回的 M 里 History 带着历次读数(取中位数当现值)。
   function Load (Path : String; Key : String; M : in out Selfmap.Body_Map; Hands : in out Zone.Hand_Vectors.Vector;
                  Tables : in out Act.Effect_Vectors.Vector; Sch : in out Schema.Map; Note : out Unbounded_String;
                  With_Tables : Boolean := False) return Boolean;
   --  With_Tables:跑的时候一律 False —— 响应表是【就地】学的,跟错了东西就把"往哪走会靠近"学反,
   --  存回来再用会一路朝反方向走(FK/FL 实测)。只有【离线体检】才把它读出来审判。
   --  写盘:把这一次量到的合进历史再写(每格最多留 History_Depth 次)。
   procedure Save (Path : String; Key : String; M : Selfmap.Body_Map; Hands : Zone.Hand_Vectors.Vector; Tables : Act.Effect_Vectors.Vector; Sch : Schema.Map);
   --  把新量到的一次合进 M(通道幅度/实到取历次中位数;噪声地板取历次最大 —— 只放大不缩小)
   procedure Merge (Stored, Fresh : Selfmap.Body_Map; Merged : out Selfmap.Body_Map; Replaced, Kept : out Natural);
   History_Depth : constant := 7;    --  每格留几次(次数,无量纲)
   --  量法版本:量身体/量握区的代码一改就 +1,存的东西版本对不上就重量(协议,无量纲)
   --  2 = 连通块按大小排序之后;1 及以前量的握区不可信(碎点被当成手指)
   Method_Ver : constant := 2;
end Bodyfile;
