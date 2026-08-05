--  body_layer_fast.adb -- implementation of the fast face.
--
--  Every loop below is bounded by Joint_Index, every arithmetic operand is a constrained subtype,
--  and there is no allocation, no access type and no exception handler.  That is what lets the
--  postconditions in the spec be discharged by proof rather than by hoping.

pragma SPARK_Mode (On);

package body Body_Layer_Fast is

   ---------------------------------------------------------------- Initial

   function Initial return State is
   begin
      --  Written out in full rather than relying on the record's defaults: a future edit to those
      --  defaults must not be able to make the initial state permissive without touching this
      --  function, where the intent is stated.
      return (Is_Installed => False,
              Is_Halted    => True,
              Why          => Not_Installed,
              N            => 0,
              Lim          => [others => (Lo => 0.0, Hi => 0.0)],
              Cap          => 0.0,
              Deadline     => 0,
              Last_Ok      => 0,
              Hold         => [others => 0.0]);
   end Initial;

   --------------------------------------------------------- Install_Limits

   procedure Install_Limits
     (S        : in out State;
      Lim      :        Limit_Array;
      N        :        Joint_Count;
      Hold0    :        Joint_Array;
      Cap      :        Newton;
      Deadline :        Millis;
      Now      :        Millis)
   is
   begin
      S := (S with delta
              Lim          => Lim,
              N            => N,
              Hold         => Hold0,   --  where the arm is, supplied and checked, never invented
              Cap          => Cap,
              Deadline     => Deadline,
              Last_Ok      => Now,
              Is_Installed => True,
              Is_Halted    => False,
              Why          => None);

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
         --  🔴 One delta aggregate, not two field assignments.  Between `Is_Halted := True` and
         --  `Why := ...` the record's invariant (a halt always carries a reason) is momentarily
         --  false, and the prover is right to object -- an interrupt landing there would observe
         --  a state that must not exist.  Updating in one step removes the window.
         S := (S with delta Is_Halted => True, Why => Not_Installed);
         Out_Cmd     := S.Hold;
         Ok          := False;
         return;
      end if;

      if Elapsed (S, Now) > S.Deadline then
         S := (S with delta Is_Halted => True, Why => Watchdog_Expired);
         Out_Cmd     := S.Hold;
         Ok          := False;
         return;
      end if;

      if Force > S.Cap then
         S := (S with delta Is_Halted => True, Why => Force_Exceeded);
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
         S := (S with delta Is_Halted => True, Why => Limit_Violation);
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
         S := (S with delta Is_Halted => True, Why => Watchdog_Expired);
      end if;
   end Tick;

   ------------------------------------------------------------------- Stop

   procedure Stop (S : in out State) is
   begin
      S := (S with delta Is_Halted => True, Why => External_Stop);
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
         --  Restart the watchdog window in the same step; inheriting a stale one would halt again
         --  immediately and read as "it will not clear" rather than "it cleared and timed out".
         S := (S with delta Is_Halted => False, Why => None, Last_Ok => Now);
         Ok          := True;
      else
         Ok := False;
      end if;
   end Clear;

end Body_Layer_Fast;
