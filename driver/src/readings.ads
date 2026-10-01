--  认读数(大并行 路 1,§2 第 1 条;10-01):推一组读数一下,每只眼里量它是"整幅在动"还是"只一块在动" —— 纯函数,开机认组(Jointboot.Find_Arms)、
--  自检、离线重放用同一份。
--  眼长在推的那组上 ⇒ 世界在这只眼里整幅地挪:画面每一处有纹理的地方都跟着变;推的那组只带动画面里一块(手指、别的胳膊、躺在眼前的零件)⇒
--  那一块以外的世界纹理再强也一点不变。离线(10-01,落盘的逐拍画面):x5 V1B78 手指合拢时腕眼里够强的格 153 / 366 变了、
--  胳膊推 0.0001 弧度时 128 / 133;人形 H4 第 1 只手推 0.0001 时腕眼 68 / 122(右下那一象限是跟着眼不动的自己的手,1 / 21)、
--  头顶眼 6 / 126;无人机 DR2 推 0.0004 时机身那只眼 112 / 120 —— 24 个(推法 × 眼)全判对。
--  不认零件个数、不认几只眼、不认单位;用到的只有:这台相机量过的静止地板(Picture.Floor_Map)、"看没看见动了"同一个判法
--  (两次比较、不共用一帧都超过地板,Picture.Seen_Twice)、全仓那一张格子(Kinem.Gx × Kinem.Gy)、中位数、四个象限(画面横竖各分两半)。
with Bytes; use Bytes;
with Picture;
package Readings is
   --  Nothing   = 这只眼里没看见东西动(或者判不了:有一帧没画面、两帧不一样大)
   --  Whole     = 整幅在动(这只眼长在推的那组上):够强的格里变了的不少于没变的(这只眼看见的大半是世界,世界挪了),而且不止一个象限在变
   --  Part      = 只一块在动:够强的格里变了的少于没变的(世界没挪)
   --  Undecided = 够强的格大半变了,可全挤在一个象限里(一大块零件和整幅挪分不开):推大一点再看
   --  "够强的格" = 推之前那一帧里这一格最强的边(相邻像素灰度差的最大)不比"变了的格"的中位弱 —— 这一推能让它变的那些格(这一推本身定的门)
   type Eye_Verdict is (Nothing, Whole, Part, Undecided);
   --  A1 → B1 = 推过去(推之前那一帧 → 走完那一帧),A2 → B2 = 推回来(再读的那一帧 → 推回来那一帧):两次比较都超过地板的像素才算变了
   function Verdict (A1, B1, A2, B2 : Buf; F : Picture.Floor_Map; W, H : Natural) return Eye_Verdict;
   --  同上,另交出四个象限各自的够强的格 / 其中变了的(左上、右上、左下、右下;开机报告和离线重放印出来)
   type Quad_Counts is array (0 .. 3) of Natural;
   procedure Verdict_Of (A1, B1, A2, B2 : Buf; F : Picture.Floor_Map; W, H : Natural; V : out Eye_Verdict; Strong, Changed : out Quad_Counts);
   function Image (V : Eye_Verdict) return String;
end Readings;
