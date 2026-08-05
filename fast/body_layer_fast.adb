--  body_layer_fast.adb -- implementation of the fast face.
--
--  Every loop below is bounded by Joint_Index, every arithmetic operand is a constrained subtype,
--  and there is no allocation, no access type and no exception handler.  That is what lets the
--  postconditions in the spec be discharged by proof rather than by hoping.

pragma SPARK_Mode (On);

package body Body_Layer_Fast is

   ---------------------------------------------------------------- Initial

   function Initial return State is
      S : State;
   begin
      --  The defaults in the private record already encode "refusing".  Naming it here so that
      --  a future edit to the record cannot silently make the default permissive.
      S.Is_Installed := False;
      S.Is_Halted    := True;
      S.Why          := Not_Installed;
      S.N            := 0;
      S.Cap          := 0.0;
      S.Deadline     := 0;
      S.Last_Ok      := 0;
      S.Lim          := (others => (Lo => 0.0, Hi => 0.0));
      S.Hold         := (others => 0.0);
      return S;
   end Initial;

   --------------------------------------------------------- Install_Limits

   procedure Install_Limits
     (S        : in out State;
      Lim      :        Limit_Array;
      N        :        Joint_Count;
      Cap      :        Newton;
      Deadline :        Millis;
      Now      :        Millis)
   is
   begin
      S.Lim          := Lim;
      S.N            := N;
      S.Cap          := Cap;
      S.Deadline     := Deadline;
      S.Last_Ok      := Now;
      S.Is_Installed := True;
      S.Is_Halted    := False;
      S.Why          := None;

      --  Hold starts at the midpoint of each installed range, which is inside the envelope by
      --  construction.  It must NOT start at zero: zero is only inside the envelope by luck, and
      --  a "safe hold" that sits outside the limits is the opposite of safe.
      for I in Joint_Index loop
         if Natural (I) <= N then
            S.Hold (I) := (Lim (I).Lo + Lim (I).Hi) / 2.0;
         else
            S.Hold (I) := 0.0;
         end if;
         pragma Loop_Invariant
           (for all K in Joint_Index'First .. I =>
              (if Natural (K) <= N then
                 S.Hold (K) >= Lim (K).Lo and then S.Hold (K) <= Lim (K).Hi));
      end loop;
   end Install_Limits;

   ------------------------------------------------------------------ Admit

   procedure Admit
     (S       : in out State;
      Cmd     :        Joint_Array;
      Force   :        Newton;
      Now     :        Millis;
      Out_Cmd :    out Joint_Array;
      Ok      :    out Boolean)
   is
      Within : Boolean := True;
   begin
      --  Order matters and is deliberate: a latched halt is answered FIRST, so that a fresh
      --  in-range command cannot walk the system out of a halt by accident.  Leaving a halt has
      --  exactly one door, and it is Clear.
      if S.Is_Halted then
         Out_Cmd := S.Hold;
         Ok      := False;
         return;
      end if;

      if not S.Is_Installed then
         S.Is_Halted := True;
         S.Why       := Not_Installed;
         Out_Cmd     := S.Hold;
         Ok          := False;
         return;
      end if;

      if Elapsed (S, Now) > S.Deadline then
         S.Is_Halted := True;
         S.Why       := Watchdog_Expired;
         Out_Cmd     := S.Hold;
         Ok          := False;
         return;
      end if;

      if Force > S.Cap then
         S.Is_Halted := True;
         S.Why       := Force_Exceeded;
         Out_Cmd     := S.Hold;
         Ok          := False;
         return;
      end if;

      --  🔴 OUT OF RANGE HALTS.  It does NOT clamp.  Clamping would let a broken upstream keep
      --  producing motion that looks obedient while its intent was nonsense -- and every failure
      --  this project has paid for had that shape: the reading stayed plausible while the thing
      --  it described was wrong.  A limit violation is information; swallowing it destroys the
      --  information and keeps the motion.
      for I in Joint_Index loop
         if Natural (I) <= S.N
           and then (Cmd (I) < S.Lim (I).Lo or else Cmd (I) > S.Lim (I).Hi)
         then
            Within := False;
         end if;
         pragma Loop_Invariant
           (if Within then
              (for all K in Joint_Index'First .. I =>
                 (if Natural (K) <= S.N then
                    Cmd (K) >= S.Lim (K).Lo and then Cmd (K) <= S.Lim (K).Hi)));
      end loop;

      if not Within then
         S.Is_Halted := True;
         S.Why       := Limit_Violation;
         Out_Cmd     := S.Hold;
         Ok          := False;
         return;
      end if;

      S.Hold    := Cmd;
      S.Last_Ok := Now;
      Out_Cmd   := Cmd;
      Ok        := True;
   end Admit;

   ------------------------------------------------------------------- Tick

   procedure Tick (S : in out State; Now : Millis) is
   begin
      if not S.Is_Halted and then Elapsed (S, Now) > S.Deadline then
         S.Is_Halted := True;
         S.Why       := Watchdog_Expired;
      end if;
   end Tick;

   ------------------------------------------------------------------- Stop

   procedure Stop (S : in out State) is
   begin
      S.Is_Halted := True;
      S.Why       := External_Stop;
   end Stop;

   ------------------------------------------------------------------ Clear

   procedure Clear
     (S       : in out State;
      Witness :        Halt_Reason;
      Now     :        Millis;
      Ok      :    out Boolean)
   is
   begin
      --  The witness must name the reason actually latched.  A caller that does not know why the
      --  system halted has no business restarting it -- that is the entire content of this check,
      --  and it is why Clear takes an argument at all.
      if S.Is_Halted and then S.Why = Witness and then S.Is_Installed then
         S.Is_Halted := False;
         S.Why       := None;
         S.Last_Ok   := Now;   --  restart the watchdog window, do not inherit a stale one
         Ok          := True;
      else
         Ok := False;
      end if;
   end Clear;

end Body_Layer_Fast;
