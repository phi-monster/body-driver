package body Driver.Robot.Hand is

   --  Path B replaces these placeholders.

   Unbuilt : exception;

   procedure Observe (H : in out Hands; M : Model; O : Observation; Sent : Driver.Commands.Command) is
      pragma Unreferenced (H, M, O, Sent);
   begin
      null;
   end Observe;

   procedure Measure (H : in out Hands; M : in out Model) is
   begin
      raise Unbuilt with "Measure";
   end Measure;

   function Hand_Count (H : Hands) return Natural is (H.Count);
   function Closer_Group (H : Hands; Id : Hand_Id) return Group_Id is (raise Unbuilt with "Closer_Group");
   function Arm_Of (H : Hands; Id : Hand_Id) return Arm_Id is (raise Unbuilt with "Arm_Of");
   function Lobe_Count (H : Hands; Id : Hand_Id) return Positive is (raise Unbuilt with "Lobe_Count");

   function Tip (H : Hands; M : Model; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; O : Observation)
     return Point_Estimate is (raise Unbuilt with "Tip");

   function Tip_Now (H : Hands; M : Model; Id : Hand_Id; Lobe : Positive; O : Observation) return Point_Estimate is
     (raise Unbuilt with "Tip_Now");

   function Grip_Centre (H : Hands; M : Model; Id : Hand_Id; O : Observation) return Point_Estimate is
     (raise Unbuilt with "Grip_Centre");

end Driver.Robot.Hand;
