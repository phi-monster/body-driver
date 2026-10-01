package body Driver.World is

   --  Path B replaces these placeholders.

   Unbuilt : exception;

   procedure Observe
     (S    : in out Scene;
      M    : Driver.Robot.Model;
      H    : Driver.Robot.Hand.Hands;
      O    : Observation;
      Sent : Driver.Commands.Command)
   is
      pragma Unreferenced (S, M, H, O, Sent);
   begin
      null;
   end Observe;

   procedure New_Episode (S : in out Scene) is
   begin
      S.Things := 0;
   end New_Episode;

   function Thing_Count (S : Scene) return Natural is (S.Things);

   procedure Adopt
     (S      : in out Scene;
      M      : Driver.Robot.Model;
      E      : Eye_Id;
      O      : Observation;
      Region : Driver.Images.Mask;
      Thing  : out Thing_Id)
   is
   begin
      raise Unbuilt with "Adopt";
   end Adopt;

   function Seen_In (S : Scene; T : Thing_Id; E : Eye_Id) return Boolean is (raise Unbuilt with "Seen_In");

   function Region_In (S : Scene; T : Thing_Id; E : Eye_Id) return Driver.Images.Mask is
     (raise Unbuilt with "Region_In");

   function Centre (S : Scene; T : Thing_Id) return Point_Estimate is (raise Unbuilt with "Centre");

   function Resting_On (S : Scene; T : Thing_Id) return Surface_Id'Base is (raise Unbuilt with "Resting_On");

   function Height_Above_Support (S : Scene; T : Thing_Id) return Estimate is
     (raise Unbuilt with "Height_Above_Support");

   function Held_By (S : Scene; T : Thing_Id) return Driver.Robot.Hand.Hand_Id'Base is
     (raise Unbuilt with "Held_By");

   function Moving (S : Scene; T : Thing_Id) return Boolean is (raise Unbuilt with "Moving");

   procedure Remember (S : in out Scene; Point : Point_Estimate; Place : out Place_Id) is
   begin
      raise Unbuilt with "Remember";
   end Remember;

   function Where (S : Scene; P : Place_Id) return Point_Estimate is (raise Unbuilt with "Where");

end Driver.World;
