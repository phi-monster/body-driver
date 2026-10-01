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

   procedure Estimate_Now (M : in out Model) is
      pragma Unreferenced (M);
   begin
      null;
   end Estimate_Now;

   function Eye_In_Tool (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate is
     (raise Unbuilt with "Eye_In_Tool");
   function Blocked (M : Model; A : Arm_Id; O : Observation) return Boolean is (raise Unbuilt with "Blocked");
   function Group_Count (M : Model) return Natural is (raise Unbuilt with "Group_Count");
   function Group_Size (M : Model; G : Group_Id) return Natural is (raise Unbuilt with "Group_Size");
   function Is_Commandable (M : Model; G : Group_Id) return Boolean is (raise Unbuilt with "Is_Commandable");
   function Reading_Noise (M : Model; G : Group_Id; Channel : Positive) return Real is
     (raise Unbuilt with "Reading_Noise");
   function Visible_Step (M : Model; G : Group_Id; Channel : Positive) return Estimate is
     (raise Unbuilt with "Visible_Step");
   function Response (M : Model; G : Group_Id; E : Eye_Id) return Eye_Response is (raise Unbuilt with "Response");
   function Image_Lag (M : Model; E : Eye_Id) return Integer is (raise Unbuilt with "Image_Lag");
   function Closer_Arm (M : Model; G : Group_Id) return Arm_Id'Base is (raise Unbuilt with "Closer_Arm");
   function Carrier_Group (M : Model) return Group_Id'Base is (raise Unbuilt with "Carrier_Group");
   function Contract_Breach (M : Model; G : Group_Id) return Natural is (raise Unbuilt with "Contract_Breach");
   function Describe (M : Model) return String is ("the body is not measured yet");

end Driver.Robot;
