--  监视器(SPARK,常数内存,纯函数):脑说的"直到什么为止"在这里被判。
--  所有门槛都是现场量出来的噪声地板;这里只做比较和计数,不含任何身体量。
pragma SPARK_Mode (On);
package Monitor is
   subtype Bounded is Long_Float range -1.0e12 .. 1.0e12;
   subtype Floor is Long_Float range 0.0 .. 1.0e12;
   type Floors is record
      Track : Floor := 0.0;       --  各块位置在画面里抖多少
      Picture : Floor := 0.0;     --  整幅画抖几级
      Reading : Floor := 0.0;     --  抓握读数抖多少
      Delivery : Floor := 0.0;    --  本体报的"实到"抖多少
   end record;
   subtype Count is Natural range 0 .. 2 ** 30;   --  计数封顶(饱和),证明里加一不会溢出
   type Watch is record
      Quiet : Count := 0;          --  画面连着几步没变
      No_Progress : Count := 0;    --  误差连着几步没缩
      Steps : Count := 0;
      Refused : Count := 0;        --  连着几步一步没走
   end record;
   type Until_Kind is (U_Steps, U_Contact, U_Resist, U_Slip, U_Settle);

   procedure Step (W : in out Watch; Pic_Delta : Floor; Err_Before, Err_After : Bounded;
                   Delivered : Floor; F : Floors)
     with Post => W.Steps = Natural'Min (W'Old.Steps + 1, Count'Last)
       and then (if Pic_Delta <= F.Picture then W.Quiet = Natural'Min (W'Old.Quiet + 1, Count'Last) else W.Quiet = 0)
       and then (if Delivered <= F.Delivery then W.Refused = Natural'Min (W'Old.Refused + 1, Count'Last) else W.Refused = 0);
   function Settled (W : Watch) return Boolean is (W.Quiet >= 2);
   function Stalled (W : Watch) return Boolean is (W.No_Progress >= 2);
   function Refusing (W : Watch) return Boolean is (W.Refused >= 2);
   function Slipped (Reading, Empty : Bounded; Noise : Floor) return Boolean is (Reading - Empty <= Noise);
   --  这一段该不该停:U_Steps 到步数;U_Contact/U_Resist 由"零表更准"(Blocked)或连着被拒判;U_Slip 读数回到空合;U_Settle 画面静止。
   function Fired (U : Until_Kind; W : Watch; Step_Cap : Natural; Blocked : Boolean;
                   Reading, Empty : Bounded; Reading_Noise : Floor) return Boolean is
     (case U is
         when U_Steps => W.Steps >= Step_Cap,
         when U_Contact | U_Resist => Blocked or else Refusing (W),
         when U_Slip => Slipped (Reading, Empty, Reading_Noise),
         when U_Settle => Settled (W));
end Monitor;
