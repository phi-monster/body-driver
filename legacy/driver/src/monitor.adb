pragma SPARK_Mode (On);
package body Monitor is
   procedure Step (W : in out Watch; Pic_Delta : Floor; Err_Before, Err_After : Bounded;
                   Delivered : Floor; F : Floors; Seen : Boolean) is
   begin
      W.Steps := Natural'Min (W.Steps + 1, Count'Last);
      if Pic_Delta <= F.Picture then
         W.Quiet := Natural'Min (W.Quiet + 1, Count'Last);
      else
         W.Quiet := 0;
      end if;
      if Err_Before - Err_After > F.Track then
         W.No_Progress := 0;
      else
         W.No_Progress := Natural'Min (W.No_Progress + 1, Count'Last);
      end if;
      if Delivered <= F.Delivery then
         W.Refused := Natural'Min (W.Refused + 1, Count'Last);
      else
         W.Refused := 0;
      end if;
      if Seen then
         W.Blind := 0;
      else
         W.Blind := Natural'Min (W.Blind + 1, Count'Last);
      end if;
   end Step;
end Monitor;
