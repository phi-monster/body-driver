package body Driver.Robot is

   --  Path A replaces these placeholders with the measured body. Until a
   --  quantity is measured it reads as an unmeasured body reports it, so
   --  callers can ask from the first beat and stay idle until it is known.

   Unbuilt : exception;

   procedure Observe (M : in out Model; O : Observation; Sent : Driver.Commands.Command) is
      pragma Unreferenced (M, O, Sent);
   begin
      null;
   end Observe;

   function Booted (M : Model) return Boolean is (M.Is_Booted);

   function Role (M : Model; G : Group_Id) return Group_Role is (Unclassified);
   function Arm_Count (M : Model) return Natural is (0);
   function Arm_Group (M : Model; A : Arm_Id) return Group_Id is (raise Unbuilt with "Arm_Group");
   function Eye_Count (M : Model) return Natural is (0);
   function Eye_Mount (M : Model; E : Eye_Id) return Mount is ((Kind => Unmeasured));

   function Eye_Pose (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate is ((others => <>));

   procedure Project
     (M       : Model;
      E       : Eye_Id;
      O       : Observation;
      Point   : Vec3;
      Px      : out Driver.Images.Pixel;
      Visible : out Boolean)
   is
      pragma Unreferenced (M, E, O, Point);
   begin
      Px := (U => 0.0, V => 0.0);
      Visible := False;
   end Project;

   function Ray (M : Model; E : Eye_Id; O : Observation; Px : Driver.Images.Pixel) return Ray_Estimate is
     ((others => <>));
   function Eye_Ray (M : Model; E : Eye_Id; Px : Driver.Images.Pixel) return Ray_Estimate is ((others => <>));
   function Up (M : Model) return Direction_Estimate is ((others => <>));
   function Tool_Pose (M : Model; A : Arm_Id; O : Observation) return Pose_Estimate is ((others => <>));
   function Eye_In_Tool (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate is ((others => <>));
   function Blocked (M : Model; A : Arm_Id; O : Observation) return Boolean is (False);

   function Self_Mask (M : Model; E : Eye_Id; O : Observation) return Driver.Images.Mask is
     (Driver.Images.Create (Driver.Images.Width (O.Images (E)), Driver.Images.Height (O.Images (E))));

   function Clearance (M : Model; Point : Vec3; O : Observation) return Estimate is (Unknown);
   function Still (M : Model) return Boolean is (False);

   procedure Estimate_Now (M : in out Model) is
      pragma Unreferenced (M);
   begin
      null;
   end Estimate_Now;

   function Group_Count (M : Model) return Natural is (0);
   function Group_Size (M : Model; G : Group_Id) return Natural is (0);
   function Is_Commandable (M : Model; G : Group_Id) return Boolean is (False);
   function Reading_Noise (M : Model; G : Group_Id; Channel : Positive) return Real is (Real'Last);
   function Visible_Step (M : Model; G : Group_Id; Channel : Positive) return Estimate is (Unknown);
   function Response (M : Model; G : Group_Id; E : Eye_Id) return Eye_Response is (Unmeasured);
   function Image_Lag (M : Model; E : Eye_Id) return Integer is (raise Unbuilt with "Image_Lag");
   function Closer_Arm (M : Model; G : Group_Id) return Arm_Id'Base is (0);
   function Carrier_Group (M : Model) return Group_Id'Base is (0);
   function Contract_Breach (M : Model; G : Group_Id) return Natural is (0);
   function Describe (M : Model) return String is ("the body is not measured yet");

end Driver.Robot;
