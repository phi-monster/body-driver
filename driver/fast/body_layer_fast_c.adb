--  body_layer_fast_c.adb -- validate, then delegate.  No saturation, no clamping, no best-effort.

--  🔴 SPARK_Mode is OFF for this body, and only for this body.  Reason, so nobody "restores" it:
--  an Ada exception must never cross an FFI boundary -- Rust aborts the process on a foreign
--  exception, turning a caller's contract violation into a crash of the whole robot stack.
--  Catching needs handlers, and SPARK forbids handlers.
--
--  What is given up is nothing that mattered.  `Body_Layer_Fast` -- every limit, force, watchdog
--  and latch rule -- stays SPARK_Mode On and stays proven.  This file only validates C values,
--  calls that proven code, and converts.  It is deliberately small enough to read in one sitting,
--  because it is the one part of the fast face carried by review instead of by proof.
pragma SPARK_Mode (Off);

package body Body_Layer_Fast_C is

   use Body_Layer_Fast;
   use type Interfaces.C.double;
   use type Interfaces.C.unsigned;
   use type Interfaces.C.int;

   S : State := Initial;

   --  A raw C double is usable only if it lands inside the constrained subtype the proven core was
   --  proven for.  `X = X` is False for NaN, and NaN is precisely what a broken caller sends.
   function In_Rad (X : C_Double) return Boolean is
     (X = X
      and then X >= C_Double (Rad'First)
      and then X <= C_Double (Rad'Last));

   function In_Newton (X : C_Double) return Boolean is
     (X = X
      and then X >= C_Double (Newton'First)
      and then X <= C_Double (Newton'Last));

   ------------------------------------------------------------------ Reset

   procedure Reset is
   begin
      S := Initial;
   end Reset;

   ---------------------------------------------------------------- Install

   function Install
     (Lo       : access constant C_Joints;
      Hi       : access constant C_Joints;
      Hold0    : access constant C_Joints;
      N        : C_Uint;
      Cap      : C_Double;
      Deadline : C_Uint;
      Now      : C_Uint) return C_Int
   is
      Lim   : Limit_Array := [others => (Lo => 0.0, Hi => 0.0)];
      Hold  : Joint_Array := [others => 0.0];
      N_Nat : Natural;
   begin
      if Lo = null or else Hi = null or else Hold0 = null then
         return BL_EINVAL;
      end if;
      if N = 0 or else N > C_Uint (Max_Joints) or else Deadline = 0 then
         return BL_EINVAL;
      end if;
      if not In_Newton (Cap) then
         return BL_EINVAL;
      end if;
      N_Nat := Natural (N);

      for I in Joint_Index loop
         if Natural (I) <= N_Nat then
            declare
               K : constant Integer  := Natural (I) - 1;
               L : constant C_Double := Lo.all (K);
               H : constant C_Double := Hi.all (K);
               D : constant C_Double := Hold0.all (K);
            begin
               --  🔴 Each of these returns EINVAL rather than repairing the value.  A wrapper that
               --  "fixes" an out-of-domain input makes the core's proof describe a number the
               --  caller never sent.
               if not In_Rad (L) or else not In_Rad (H) or else not In_Rad (D) then
                  return BL_EINVAL;
               end if;
               if L > H then
                  return BL_EINVAL;
               end if;
               if D < L or else D > H then
                  --  The arm is not where the caller says it is, or the envelope excludes it.
                  --  Either way, installing would seat the safe hold outside the safe region.
                  return BL_EINVAL;
               end if;
               Lim (I)  := (Lo => Rad (L), Hi => Rad (H));
               Hold (I) := Rad (D);
            end;
         end if;
         pragma Loop_Invariant
           (for all K in Joint_Index'First .. I =>
              (if Natural (K) <= N_Nat then
                 Hold (K) >= Lim (K).Lo and then Hold (K) <= Lim (K).Hi));
      end loop;

      Install_Limits (S, Lim, N_Nat, Hold, Newton (Cap), Millis (Deadline), Millis (Now));
      return BL_OK;
   exception
      --  A contract violation here is a CALLER bug.  Reported as one, state left alone -- never
      --  allowed to become a process abort on the other side of the boundary.
      when others =>
         return BL_EINVAL;
   end Install;

   ------------------------------------------------------------------ Admit

   function Admit
     (Cmd     : access constant C_Joints;
      Force   : C_Double;
      Now     : C_Uint;
      Out_Cmd : access C_Joints) return C_Int
   is
      Cmd_A : Joint_Array := [others => 0.0];
      Out_A : Joint_Array;
      Ok    : Boolean;
   begin
      if Cmd = null or else Out_Cmd = null then
         return BL_EINVAL;
      end if;
      if not In_Newton (Force) then
         return BL_EINVAL;
      end if;

      for I in Joint_Index loop
         if Natural (I) <= Joints (S) then
            declare
               V : constant C_Double := Cmd.all (Natural (I) - 1);
            begin
               if not In_Rad (V) then
                  return BL_EINVAL;
               end if;
               Cmd_A (I) := Rad (V);
            end;
         end if;
      end loop;

      Body_Layer_Fast.Admit (S, Cmd_A, Newton (Force), Millis (Now), Out_A, Ok);

      for I in Joint_Index loop
         Out_Cmd.all (Natural (I) - 1) := C_Double (Out_A (I));
      end loop;

      --  BL_REFUSE is an ANSWER, not an error.  The caller must count it separately from a task
      --  failure: "not permitted" and "tried and failed" are different facts.
      return (if Ok then BL_OK else BL_REFUSE);
   exception
      when others =>
         --  Fail CLOSED: latch, then report.  A gate that answers "permitted" when it does not
         --  know what happened is not a gate.
         Body_Layer_Fast.Stop (S);
         return BL_EINVAL;
   end Admit;

   ------------------------------------------------------------------- misc

   procedure Tick (Now : C_Uint) is
   begin
      Body_Layer_Fast.Tick (S, Millis (Now));
   exception
      when others =>
         Body_Layer_Fast.Stop (S);   --  fail closed
   end Tick;

   procedure Stop is
   begin
      Body_Layer_Fast.Stop (S);
   end Stop;

   function Clear (Witness : C_Int; Now : C_Uint) return C_Int is
      Ok : Boolean;
   begin
      if Witness <= 0 or else Witness > C_Int (Halt_Reason'Pos (Halt_Reason'Last)) then
         return BL_EINVAL;
      end if;
      Body_Layer_Fast.Clear (S, Halt_Reason'Val (Witness), Millis (Now), Ok);
      return (if Ok then BL_OK else BL_REFUSE);
   exception
      when others =>
         return BL_EINVAL;   --  refusing to leave a halt is the safe direction
   end Clear;

   function Halted return C_Int is
     (if Body_Layer_Fast.Halted (S) then 1 else 0);

   function Reason return C_Int is
     (C_Int (Halt_Reason'Pos (Body_Layer_Fast.Reason (S))));

end Body_Layer_Fast_C;
