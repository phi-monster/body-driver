with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
package body Runtime is

   use type Sinew.Op;
   use type Sinew.Outcome;

   Broken : Unbounded_String;

   function Broken_Why (M : Machine) return String is
      pragma Unreferenced (M);
   begin
      return To_String (Broken);
   end Broken_Why;

   function Find_Def (P : Sinew.Program; Name : Unbounded_String) return Integer is
   begin
      for I in 0 .. Natural (P.Defs.Length) - 1 loop
         if P.Defs (I).Name = Name then
            return Integer (P.Defs (I).At_Addr);
         end if;
      end loop;
      return -1;
   end Find_Def;

   procedure Advance (P : Sinew.Program; M : in out Machine; What : out Yield; I : out Sinew.Instr) is
      use Sinew;
   begin
      I := (others => <>);
      loop
         if M.PC >= Natural (P.Code.Length) then
            --  掉出末尾:如果还在一段定义体里(不该发生),当跑完
            What := Y_Finished;
            return;
         end if;
         M.Ticks := M.Ticks + 1;
         if M.Ticks > Max_Instr then
            Broken := To_Unbounded_String ("这段程序一直在原地打转,走了太多条指令还没停 —— 循环没有出口");
            What := Y_Broken;
            return;
         end if;
         I := P.Code (M.PC);
         case I.O is
            when Op_Interval =>
               What := Y_Interval;
               return;
            when Op_Say =>
               M.PC := M.PC + 1;
               What := Y_Say;
               return;
            when Op_Remember =>
               M.PC := M.PC + 1;
               What := Y_Remember;
               return;
            when Op_Done =>
               M.PC := M.PC + 1;
               What := Y_Done;
               return;
            when Op_Jump =>
               if I.Target < 0 then
                  Broken := To_Unbounded_String ("有一个块没有 end");
                  What := Y_Broken;
                  return;
               end if;
               M.PC := Natural (I.Target);
            when Op_If =>
               if M.Last = I.Cond then
                  M.PC := M.PC + 1;
               elsif I.Target >= 0 then
                  M.PC := Natural (I.Target);
               else
                  M.PC := M.PC + 1;
               end if;
            when Op_Loop =>
               if M.Loop_N >= Max_Stack then
                  Broken := To_Unbounded_String ("循环套得太深了");
                  What := Y_Broken;
                  return;
               end if;
               declare
                  F : Frame;
               begin
                  F.Head := M.PC;
                  F.Exit_At := (if I.Target >= 0 then Natural (I.Target) else M.PC + 1);
                  F.Counted := I.Cond = Oc_None;
                  F.Left := I.Count;
                  F.Cond := I.Cond;
                  if F.Counted and then F.Left = 0 then
                     M.PC := F.Exit_At;                 --  repeat 0 times
                  elsif (not F.Counted) and then M.Last = F.Cond then
                     M.PC := F.Exit_At;                 --  进门就已经是那个结局了
                  else
                     M.Loop_N := M.Loop_N + 1;
                     M.Loops (M.Loop_N) := F;
                     M.PC := M.PC + 1;
                  end if;
               end;
            when Op_Next =>
               if M.Loop_N = 0 then
                  Broken := To_Unbounded_String ("循环尾没有对应的循环头");
                  What := Y_Broken;
                  return;
               end if;
               declare
                  F : Frame := M.Loops (M.Loop_N);
                  Out_Now : Boolean := False;
               begin
                  if F.Counted then
                     F.Left := (if F.Left > 0 then F.Left - 1 else 0);
                     Out_Now := F.Left = 0;
                  else
                     Out_Now := M.Last = F.Cond;
                  end if;
                  if Out_Now then
                     M.Loop_N := M.Loop_N - 1;
                     M.PC := F.Exit_At;
                  else
                     M.Loops (M.Loop_N) := F;
                     M.PC := F.Head + 1;
                  end if;
               end;
            when Op_Call =>
               declare
                  A : constant Integer := Find_Def (P, I.Name);
               begin
                  if A < 0 then
                     Broken := To_Unbounded_String ("我不认得「" & To_String (I.Name) & "」这段行为 —— 你没有 to 它");
                     What := Y_Broken;
                     return;
                  end if;
                  if M.Call_N >= Max_Stack then
                     Broken := To_Unbounded_String ("一段行为把自己叫得太深了");
                     What := Y_Broken;
                     return;
                  end if;
                  M.Call_N := M.Call_N + 1;
                  M.Calls (M.Call_N) := M.PC + 1;
                  M.PC := Natural (A);
               end;
            when Op_Ret =>
               if M.Call_N = 0 then
                  What := Y_Finished;      --  定义体是被跳过的,正常流不该走到这儿
                  return;
               end if;
               M.PC := M.Calls (M.Call_N);
               M.Call_N := M.Call_N - 1;
            when Op_Try =>
               if M.Try_N >= Max_Stack then
                  Broken := To_Unbounded_String ("try 套得太深了");
                  What := Y_Broken;
                  return;
               end if;
               M.Try_N := M.Try_N + 1;
               M.Tries (M.Try_N) := (if I.Target >= 0 then Natural (I.Target) else M.PC + 1);
               M.PC := M.PC + 1;
            when Op_Endtry =>
               if M.Try_N > 0 then
                  M.Try_N := M.Try_N - 1;
               end if;
               M.PC := M.PC + 1;
         end case;
      end loop;
   end Advance;

   procedure Report (P : Sinew.Program; M : in out Machine; O : Sinew.Outcome) is
      pragma Unreferenced (P);
      use Sinew;
   begin
      M.Last := O;
      if Is_Failure (O) and then M.Try_N > 0 then
         --  这一段没成,而且我们身处一个 try 里 ⇒ 走 or 那一段
         M.PC := M.Tries (M.Try_N);
         M.Try_N := M.Try_N - 1;
      else
         M.PC := M.PC + 1;
      end if;
   end Report;

end Runtime;
