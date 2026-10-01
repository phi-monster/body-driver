separate (Jointboot)
function Judge_End (Q_Cmd, Q_At, Q_Now, Got_Lo, Got_Hi, Step_Lo, Step_Hi : Floats; Tol : Long_Float; J : out Integer; Hi_Side : out Boolean) return End_Verdict is
   N : constant Natural := Natural'Min (Natural'Min (Natural (Q_Cmd.Length), Natural (Q_At.Length)),
                                        Natural'Min (Natural (Q_Now.Length), Natural'Min (Natural (Got_Lo.Length), Natural (Got_Hi.Length))));
   Shorts : Natural := 0;
   Blocked_Any : Boolean := False;
   --  到了 = 差不到 Tol,或者差不到它这一下要走的三分之一(比例,同扫描"到了")
   function Arrived (K : Natural) return Boolean is
     (abs (Q_Now (K) - Q_Cmd (K)) <= Long_Float'Max (Tol, abs (Q_Cmd (K) - Q_At (K)) * Third));
begin
   J := -1; Hi_Side := False;
   for K in 0 .. N - 1 loop
      declare
         --  要往范围外走的那一截不到半步(这个关节那一边量过的步子;没量过 = 0)⇒ 这个关节不核:走没走到都判不准,
         --  和手爬到位的误差一个量级(V1B64:只多要 0.0057 弧度、停在一半不到 ⇒ 记成尽头,第 2 只手接着 11 回"在量到的关节限位里解不出来",
         --  读数越过它才删掉)。每拍跟着重发时,真碰到尽头的那一条多要的正好一整步(范围不长了、界还在一步开外)⇒ 照样核得到
         Half_Hi : constant Long_Float := (if K < Natural (Step_Hi.Length) then 0.5 * Step_Hi (K) else 0.0);
         Half_Lo : constant Long_Float := (if K < Natural (Step_Lo.Length) then 0.5 * Step_Lo (K) else 0.0);
         Up : constant Boolean := Q_Cmd (K) > Got_Hi (K) + Long_Float'Max (Tol, Half_Hi);
         Down : constant Boolean := Q_Cmd (K) < Got_Lo (K) - Long_Float'Max (Tol, Half_Lo);
         Small : constant Boolean := not Up and then not Down and then (Q_Cmd (K) > Got_Hi (K) + Tol or else Q_Cmd (K) < Got_Lo (K) - Tol);
      begin
         if Small then
            null;
         elsif Up or else Down then
            declare
               B : constant Long_Float := (if Up then Got_Hi (K) else Got_Lo (K));
               Ext : constant Long_Float := abs (Q_Cmd (K) - B);
               Prog : constant Long_Float := (if Up then Q_Now (K) - B else B - Q_Now (K));
            begin
               if Prog + Prog < Ext then   --  走到的不到要往外走的那一截的一半(纯数学的一半,同扫描)
                  Shorts := Shorts + 1;
                  J := K; Hi_Side := Up;
               elsif not Arrived (K) then
                  Blocked_Any := True;
               end if;
            end;
         elsif not Arrived (K) then
            Blocked_Any := True;   --  范围里的关节没到 / 被顶偏
         end if;
      end;
   end loop;
   if Blocked_Any then
      if Shorts /= 1 then
         J := -1;
      end if;
      return Blocked;
   elsif Shorts = 1 then
      return End_Hit;
   elsif Shorts > 1 then
      J := -1;
      return Ambiguous;
   end if;
   return Reached;
end Judge_End;
