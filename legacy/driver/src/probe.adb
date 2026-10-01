with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Stats;
package body Probe is
   function Track_Sigma (Static_D2 : Floats) return Long_Float is
      S : Long_Float := 0.0;
   begin
      if Static_D2.Is_Empty then
         return 0.0;
      end if;
      for D of Static_D2 loop
         S := S + D;
      end loop;
      return Sqrt (S / (2.0 * Long_Float (Static_D2.Length)));   --  每个点两个轴(du、dv)
   end Track_Sigma;

   function Floor_Of (Sigma : Long_Float) return Long_Float is (Stats.Z * Sigma);

   function Next (Seen : Boolean; Ran_Meas, Last_Ran, Amp, Cap : Long_Float) return Next_Step is
   begin
      if Seen then
         return Take_It;
      end if;
      --  加倍还在上限以内才加倍(同原来:幅度 × 2 比上限大 ⇒ 这一段不用它)
      if Amp * 2.0 > Cap then
         return At_Cap;
      end if;
      --  上一推、这一推都量出了挪动,加倍以后一点没多 ⇒ 这根通道不动这些点(零本身是一次量);
      --  有一推一个点都没量出来(跟丢 / 不到地板)⇒ 说明不了"没多",接着加倍
      if Last_Ran >= 0.0 and then Ran_Meas >= 0.0 and then Ran_Meas <= Last_Ran then
         return Is_Zero;
      end if;
      return Double_It;
   end Next;
end Probe;
