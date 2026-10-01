with Codec;
package body Strokes is

   procedure Run (Want, Lo, Hi, At_Start, Resolution : Long_Float; Budget : Natural;
                  Stroke : access procedure (D : Long_Float; R : out Step_Report);
                  Release : access procedure (Ok : out Boolean);
                  Free : access procedure (D : Long_Float; R : out Step_Report);
                  Regrasp : access procedure (Ok : out Boolean);
                  Rep : out Report) is
      S : constant Long_Float := (if Want >= 0.0 then 1.0 else -1.0);    --  往哪边走
      To_Tight : constant Boolean := abs Want = Long_Float'Last;        --  走到转不动为止
      At_Q : Long_Float := At_Start;
      --  这一边离范围的头还有多远
      function Room return Long_Float is (if S > 0.0 then Hi - At_Q else At_Q - Lo);
      function Left return Long_Float is (if To_Tight then Long_Float'Last else abs Want - S * Rep.Done);
   begin
      Rep := (others => <>);
      if Stroke = null or else Release = null or else Free = null or else Regrasp = null or else not (Resolution > 0.0) then
         Rep.How := Body_Failed;
         return;
      end if;
      if not (Hi - Lo >= Resolution) then
         Rep.How := No_Room;
         return;
      end if;
      loop
         if Left <= Resolution then
            Rep.How := Reached;
            return;
         end if;
         if Rep.Steps >= Budget then
            Rep.How := Out_Of_Steps;
            return;
         end if;
         if Room < Resolution then
            --  到了范围的头:松开、空着走回另一头、再握
            declare
               Ok : Boolean;
               R : Step_Report;
               Before : constant Long_Float := At_Q;
            begin
               Release (Ok);
               if not Ok then
                  Rep.How := Body_Failed;
                  return;
               end if;
               Free ((if S > 0.0 then Lo else Hi) - At_Q, R);
               Rep.Steps := Rep.Steps + 1;
               if not R.Ok then
                  Rep.How := Body_Failed;
                  return;
               end if;
               At_Q := R.At_Now;
               if Room < Resolution or else abs (At_Q - Before) < Resolution then
                  Rep.How := No_Room;   --  空着也走不回去(被挡住了):一下都走不了
                  return;
               end if;
               Regrasp (Ok);
               if not Ok then
                  Rep.How := Regrasp_Failed;
                  return;
               end if;
               Rep.Regrasps := Rep.Regrasps + 1;
            end;
         else
            --  拿着往要的方向走:走到要的、或者走到范围的头
            declare
               R : Step_Report;
               Before : constant Long_Float := At_Q;
            begin
               Stroke (S * Long_Float'Min (Left, Room), R);
               Rep.Steps := Rep.Steps + 1;
               Rep.Strokes := Rep.Strokes + 1;
               if not R.Ok then
                  Rep.How := Body_Failed;
                  return;
               end if;
               At_Q := R.At_Now;
               Rep.Done := Rep.Done + (At_Q - Before);
               --  被挡住、或者量到一点都没走,而离范围的头还不止最小一步 ⇒ 转不动了
               if (R.Blocked or else S * (At_Q - Before) < Resolution) and then Room >= Resolution then
                  Rep.How := Tight;
                  return;
               end if;
            end;
         end if;
      end loop;
   end Run;

   function Say (Rep : Report) return String is
     ((case Rep.How is
          when Reached => "走够了",
          when Tight => "转不动了",
          when Regrasp_Failed => "再握没握上",
          when Body_Failed => "身体那一步没做成",
          when No_Room => "范围里一下都走不了",
          when Out_Of_Steps => "步数用完了")
      & "(拿着走了 " & Codec.Fmt (Rep.Done, 4) & ",走了 " & Codec.Img (Rep.Strokes) & " 下,松开再握 " & Codec.Img (Rep.Regrasps)
      & " 回,身体一共 " & Codec.Img (Rep.Steps) & " 步)");
end Strokes;
