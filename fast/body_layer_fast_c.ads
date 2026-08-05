--  body_layer_fast_c.ads -- the C-callable face of the proven fast core.
--
--  WHY A SEPARATE PACKAGE.  `Body_Layer_Fast` is proven, and its interface uses Ada types that
--  carry their own constraints (`Rad`, `Newton`, a private `State` with an invariant).  Those
--  types are exactly what makes the proof possible, and exactly what C cannot express.
--
--  So the conversion happens HERE, in one place, and in the safe direction: this package
--  VALIDATES the raw C values and REFUSES anything outside the proven core's domain.  The proven
--  core never sees a value it was not proven for.
--
--  🔴 This is where a wrapper usually rots.  The usual failure is one that saturates or wraps an
--  out-of-domain value so the core "can" accept it -- after which the core's proof is about a
--  number nobody sent, which is worse than no proof.  This one returns an error code.  A caller
--  that sends garbage is told so; it is not quietly corrected.
--
--  ARRAY CONVENTION, stated once and enforced by the type: every joint array crossing this
--  boundary has EXACTLY `Max_Joints` elements, regardless of how many joints the robot has.
--  A length that travels separately from its buffer is a length that can disagree with it.

pragma SPARK_Mode (On);

with Interfaces.C;
with Body_Layer_Fast;

package Body_Layer_Fast_C is

   subtype C_Int    is Interfaces.C.int;
   subtype C_Double is Interfaces.C.double;
   subtype C_Uint   is Interfaces.C.unsigned;

   --  Mirrors `bl_status` in ../abi/body_layer.h.  Named constants rather than an enumeration so
   --  the numbering sits visibly next to the header it has to match.
   BL_OK     : constant C_Int := 0;
   BL_REFUSE : constant C_Int := 1;
   BL_EINVAL : constant C_Int := 2;

   Max_Joints : constant := Body_Layer_Fast.Max_Joints;

   type C_Joints is array (0 .. Max_Joints - 1) of aliased C_Double
     with Convention => C;

   --  One state per process.  The fast face is per-robot and per-process; an array of them would
   --  invite the "which one am I talking to" class of bug for no gain.
   procedure Reset
     with Export, Convention => C, External_Name => "blf_reset";

   --  Install the MEASURED envelope.  `Hold0` is where the arm IS -- see the note in
   --  Body_Layer_Fast.Install_Limits on why that is an argument and not a computed midpoint.
   --  Returns BL_EINVAL, leaving the state untouched, if any value is outside the proven domain,
   --  if Lo > Hi, or if Hold0 is not inside [Lo, Hi].
   function Install
     (Lo       : access constant C_Joints;
      Hi       : access constant C_Joints;
      Hold0    : access constant C_Joints;
      N        : C_Uint;
      Cap      : C_Double;
      Deadline : C_Uint;
      Now      : C_Uint) return C_Int
     with Export, Convention => C, External_Name => "blf_install";

   --  The gate.  BL_OK and `Out_Cmd` written when admitted; BL_REFUSE and the safe hold written
   --  when not; BL_EINVAL when the C values are outside the proven domain.
   function Admit
     (Cmd     : access constant C_Joints;
      Force   : C_Double;
      Now     : C_Uint;
      Out_Cmd : access C_Joints) return C_Int
     with Export, Convention => C, External_Name => "blf_admit";

   procedure Tick (Now : C_Uint)
     with Export, Convention => C, External_Name => "blf_tick";

   procedure Stop
     with Export, Convention => C, External_Name => "blf_stop";

   --  Witness must name the reason actually latched; see Body_Layer_Fast.Clear.
   function Clear (Witness : C_Int; Now : C_Uint) return C_Int
     with Export, Convention => C, External_Name => "blf_clear";

   --  0 = running, 1 = halted; and the reason as `Halt_Reason'Pos`.
   function Halted return C_Int
     with Export, Convention => C, External_Name => "blf_halted";

   function Reason return C_Int
     with Export, Convention => C, External_Name => "blf_reason";

end Body_Layer_Fast_C;
