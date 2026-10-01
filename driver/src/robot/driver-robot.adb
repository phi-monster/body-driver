package body Driver.Robot is

   --  Path A replaces these placeholders with the measured body.

   Unbuilt : exception;

   procedure Observe (M : in out Model; O : Observation; Sent : Driver.Commands.Command) is
      pragma Unreferenced (M, O, Sent);
   begin
      null;
   end Observe;

   function Booted (M : Model) return Boolean is (M.Is_Booted);

   function Role (M : Model; G : Group_Id) return Group_Role is (raise Unbuilt with "Role");
   function Arm_Count (M : Model) return Natural is (raise Unbuilt with "Arm_Count");
   function Arm_Group (M : Model; A : Arm_Id) return Group_Id is (raise Unbuilt with "Arm_Group");
   function Eye_Count (M : Model) return Natural is (raise Unbuilt with "Eye_Count");
   function Eye_Mount (M : Model; E : Eye_Id) return Mount is (raise Unbuilt with "Eye_Mount");

   function Eye_Pose (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate is
     (raise Unbuilt with "Eye_Pose");

   procedure Project
     (M       : Model;
      E       : Eye_Id;
      O       : Observation;
      Point   : Vec3;
      Px      : out Driver.Images.Pixel;
      Visible : out Boolean)
   is
   begin
      raise Unbuilt with "Project";
   end Project;

   function Ray (M : Model; E : Eye_Id; O : Observation; Px : Driver.Images.Pixel) return Ray_Estimate is
     (raise Unbuilt with "Ray");

   function Up (M : Model) return Direction_Estimate is (raise Unbuilt with "Up");

   function Tool_Pose (M : Model; A : Arm_Id; O : Observation) return Pose_Estimate is
     (raise Unbuilt with "Tool_Pose");

   function Self_Mask (M : Model; E : Eye_Id; O : Observation) return Driver.Images.Mask is
     (raise Unbuilt with "Self_Mask");

   function Clearance (M : Model; Point : Vec3; O : Observation) return Estimate is
     (raise Unbuilt with "Clearance");

   function Still (M : Model) return Boolean is (raise Unbuilt with "Still");

end Driver.Robot;
