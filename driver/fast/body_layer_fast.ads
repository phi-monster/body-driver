--  body_layer_fast.ads -- the FAST face of the body layer.  SPARK 2014, hard real time.
--
--    world model + body layer
--    世界模型 + 身体层
--    世界靠学，身体靠量。
--
--  WHAT LIVES HERE AND WHY IT IS A SEPARATE FACE.  Limits, force cap, watchdog, e-stop.  The
--  CONTENT is identical to the slow face's -- the numbers clamped here are the numbers the slow
--  face measured.  They are split for exactly one reason:
--
--      a force limit that is checked once a second is not a safety limit.
--
--  WHY SPARK AND WHY NOW.  The commercial shape of this project is selling compliance, so a
--  kernel with a machine-checked proof of no runtime errors is a PRODUCT FEATURE, not engineering
--  vanity.  And the owner's standing order is explicit: write the SPARK part in SPARK from day
--  one, never "Rust now, SPARK later".  The stated reason is the honest one -- you will not get
--  around to it.  This repository's hit rate on "we will switch later" is zero.
--
--  DESIGN RULES THAT MAKE THE PROOF POSSIBLE (violate any and the proof dies)
--    * no dynamic allocation, no access types, no exceptions, no unbounded loops
--    * every numeric type is CONSTRAINED, so range checks are discharged statically
--    * every subprogram carries a postcondition strong enough for its caller to rely on
--    * the e-stop is a LATCH: it is entered by any of several conditions and leaves only through
--      one explicit door that requires a fresh witness.  A safety state you can exit by doing
--      nothing is not a safety state.
--
--  WHAT THIS FACE MUST NEVER DO.  It never measures, never learns, never talks to the network,
--  never allocates.  If it needs a number it did not receive through Install_Limits, it refuses.

pragma SPARK_Mode (On);

package Body_Layer_Fast is

   ----------------------------------------------------------------- dimensions

   Max_Joints : constant := 16;
   type Joint_Index is range 1 .. Max_Joints;
   subtype Joint_Count is Natural range 0 .. Max_Joints;

   --  +/- two full turns.  A command outside this is a CALLER bug, not a limit violation, and it
   --  is caught at the type boundary rather than clamped silently -- silent clamping is how a
   --  broken upstream keeps looking healthy.
   subtype Rad is Long_Float range -12.6 .. 12.6;

   --  Newtons.  Upper bound is deliberately far above any tabletop arm: this type exists to make
   --  the arithmetic provable, not to encode a policy.  The policy lives in Install_Limits.
   subtype Newton is Long_Float range 0.0 .. 10_000.0;

   type Joint_Array is array (Joint_Index) of Rad;

   type Limit_Pair is record
      Lo : Rad;
      Hi : Rad;
   end record
     with Dynamic_Predicate => Limit_Pair.Lo <= Limit_Pair.Hi;

   type Limit_Array is array (Joint_Index) of Limit_Pair;

   --  Milliseconds, monotonic, wrapping.  Modular so that wrap-around arithmetic is defined and
   --  provable rather than undefined at 49 days.
   type Millis is mod 2 ** 32;

   -------------------------------------------------------------------- reasons

   type Halt_Reason is
     (None,
      Not_Installed,      --  asked to act before any limits were installed
      Limit_Violation,    --  a command left the installed envelope
      Force_Exceeded,     --  measured force above the installed cap
      Watchdog_Expired,   --  no fresh command within the deadline
      External_Stop);     --  somebody pressed the button

   ---------------------------------------------------------------------- state

   --  Private so no caller can fabricate a "healthy" state.  Every field is set only through the
   --  operations below, each of which carries a postcondition.
   type State is private;

   function Installed   (S : State) return Boolean;
   function Halted      (S : State) return Boolean;
   function Reason      (S : State) return Halt_Reason;
   function Joints      (S : State) return Joint_Count;
   function Force_Cap   (S : State) return Newton;
   function Deadline_Ms (S : State) return Millis;

   --  A fresh state is NOT usable: it is halted with Not_Installed.  The default has to be the
   --  refusing one, because a body layer whose default is "permit" is a body layer that permits
   --  whenever somebody forgets to configure it.
   function Initial return State
     with Post => Halted (Initial'Result)
                  and then Reason (Initial'Result) = Not_Installed
                  and then not Installed (Initial'Result);

   ------------------------------------------------------------- configuration

   --  Install the envelope this body was MEASURED to have.  N = 0 is rejected: a body with no
   --  joints cannot be commanded, and accepting it would let an unconfigured layer look configured.
   --  🔴 `Hold0` is where the arm ACTUALLY IS right now, and it is an argument rather than
   --  something this package computes.  An earlier version invented the midpoint of each range.
   --  That is a fabricated body constant -- the exact thing this layer exists to abolish -- and
   --  it is also wrong on its own terms: the safe place to hold an arm is where the arm is, not
   --  the centre of its travel, which may be through the table.  The precondition makes "inside
   --  the envelope" the caller's obligation, checked, rather than an assumption.
   procedure Install_Limits
     (S        : in out State;
      Lim      :        Limit_Array;
      N        :        Joint_Count;
      Hold0    :        Joint_Array;
      Cap      :        Newton;
      Deadline :        Millis;
      Now      :        Millis)
     with Pre  => N > 0 and then Deadline > 0
                  and then (for all I in Joint_Index =>
                              (if Natural (I) <= N then
                                 Hold0 (I) >= Lim (I).Lo
                                 and then Hold0 (I) <= Lim (I).Hi)),
          Post => Installed (S)
                  and then Joints (S) = N
                  and then Force_Cap (S) = Cap
                  and then Deadline_Ms (S) = Deadline
                  and then not Halted (S)
                  and then Reason (S) = None;

   ------------------------------------------------------------------ the gate

   --  Admit one command.  This is the only path to motion.
   --
   --  Postcondition is the whole point: IF it admits, every element within N is inside the
   --  installed envelope -- proved, not tested.  If it refuses, the state is halted and carries a
   --  reason, and Out_Cmd is the safe hold (the last admitted command), never zeros.  Zeros would
   --  be a MOVE, and "fail safe" cannot mean "fly to the origin".
   procedure Admit
     (S       : in out State;
      Cmd     :        Joint_Array;
      Force   :        Newton;
      Now     :        Millis;
      Out_Cmd :    out Joint_Array;
      Ok      :    out Boolean)
     with Post => (if Ok then
                     (not Halted (S)
                      and then (for all I in Joint_Index =>
                                  (if Natural (I) <= Joints (S) then
                                     Out_Cmd (I) >= Lo_Of (S, I)
                                     and then Out_Cmd (I) <= Hi_Of (S, I))))
                   else Halted (S) and then Reason (S) /= None);

   --  Call at least once per control period even when idle.  Missing the deadline halts.
   procedure Tick (S : in out State; Now : Millis)
     with Post => (if not Halted (S'Old) and then Elapsed (S'Old, Now) > Deadline_Ms (S'Old)
                   then Halted (S) and then Reason (S) = Watchdog_Expired);

   procedure Stop (S : in out State)
     with Post => Halted (S) and then Reason (S) = External_Stop;

   --  The ONLY door out of a halt, and it demands a witness: the caller must pass the reason it
   --  believes is being cleared.  A blind Reset is how a latch degrades into a no-op.
   procedure Clear
     (S       : in out State;
      Witness :        Halt_Reason;
      Now     :        Millis;
      Ok      :    out Boolean)
     with Pre  => Witness /= None,
          --  🔴 CORRECTED 2026-08-06, and the prover is what caught it.  The old text said the
          --  failing branch leaves the state HALTED -- false when `Clear` is called on a state
          --  that was never halted, which is a perfectly legal call.  The honest contract is
          --  "on failure nothing changes", and that is also the stronger guarantee.
          Post => (if Ok then not Halted (S) and then Reason (S) = None
                   else S = S'Old);

   ------------------------------------------------- accessors used in contracts

   function Lo_Of (S : State; I : Joint_Index) return Rad;
   function Hi_Of (S : State; I : Joint_Index) return Rad
     with Post => Hi_Of'Result >= Lo_Of (S, I);

   function Elapsed (S : State; Now : Millis) return Millis;

private

   --  🔴 The invariant the provers need, and that a reader needs just as much: a halt ALWAYS
   --  carries a reason, and a running state never does.  Without writing it down, "if it refused
   --  then Reason /= None" is unprovable -- not because it is false, but because nothing said so.
   type State is record
      Is_Installed : Boolean     := False;
      Is_Halted    : Boolean     := True;
      Why          : Halt_Reason := Not_Installed;
      N            : Joint_Count := 0;
      Lim          : Limit_Array := [others => (Lo => 0.0, Hi => 0.0)];
      Cap          : Newton      := 0.0;
      Deadline     : Millis      := 0;
      Last_Ok      : Millis      := 0;
      Hold         : Joint_Array := [others => 0.0];
   end record
     with Dynamic_Predicate => (State.Is_Halted = (State.Why /= None));

   function Installed   (S : State) return Boolean     is (S.Is_Installed);
   function Halted      (S : State) return Boolean     is (S.Is_Halted);
   function Reason      (S : State) return Halt_Reason is (S.Why);
   function Joints      (S : State) return Joint_Count is (S.N);
   function Force_Cap   (S : State) return Newton      is (S.Cap);
   function Deadline_Ms (S : State) return Millis      is (S.Deadline);

   function Lo_Of (S : State; I : Joint_Index) return Rad is (S.Lim (I).Lo);
   function Hi_Of (S : State; I : Joint_Index) return Rad is (S.Lim (I).Hi);

   function Elapsed (S : State; Now : Millis) return Millis is (Now - S.Last_Ok);

end Body_Layer_Fast;
