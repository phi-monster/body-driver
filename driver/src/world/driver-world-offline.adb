package body Driver.World.Offline is

   package Estimates renames Driver.World.Estimates;

   procedure Observe
     (B         : in out Bench;
      Eyes      : Natural;
      Camera_Of : not null access function (E : Eye_Id; Seen : not null access constant Observation)
                                             return Driver.World.Cameras.Camera'Class;
      Up        : Direction_Estimate;
      Still     : Boolean;
      O         : Observation) is
   begin
      Estimates.Observe (B.State, Eyes, Camera_Of, Up, Still, O);
   end Observe;

   procedure Adopt (B : in out Bench; E : Eye_Id; O : Observation; Region : Driver.Images.Mask; Thing : out Thing_Id) is
   begin
      Estimates.Adopt (B.State, E, O, Region, Thing);
   end Adopt;

   procedure New_Episode (B : in out Bench) is
   begin
      Estimates.New_Episode (B.State);
   end New_Episode;

   function Thing_Count (B : Bench) return Natural is (Estimates.Thing_Count (B.State));

   function Seen_In (B : Bench; T : Thing_Id; E : Eye_Id) return Boolean is (Estimates.Seen_In (B.State, T, E));
   function Region_In (B : Bench; T : Thing_Id; E : Eye_Id) return Driver.Images.Mask is
     (Estimates.Region_In (B.State, T, E));

   function Points_Of (B : Bench; T : Thing_Id) return Driver.World.Pairs.Match_Vectors.Vector is
     (Estimates.Points_Of (B.State, T));

   function Centre (B : Bench; T : Thing_Id) return Point_Estimate is (Estimates.Centre (B.State, T));

   function Resting_On (B : Bench; T : Thing_Id) return Surface_Id'Base is (Estimates.Resting_On (B.State, T));

   function Height_Above_Support (B : Bench; T : Thing_Id) return Estimate is
     (Estimates.Height_Above_Support (B.State, T));

   function No_Gap_Seen (B : Bench; T : Thing_Id) return Boolean is (Estimates.No_Gap_Seen (B.State, T));

   function Surface_Count (B : Bench) return Natural is (Estimates.Surface_Count (B.State));

   function Surface_Of (B : Bench; F : Surface_Id) return Driver.World.Supports.Surface is
     (Estimates.Surface_Of (B.State, F));

   function Plane_Of (B : Bench; F : Surface_Id) return Driver.Geometry.Plane_Estimate is
     (Estimates.Plane_Of (B.State, F));

   function Scene_Round (B : Bench) return Natural is (Estimates.Scene_Round (B.State));
   function Scene_Size (B : Bench) return Natural is (Estimates.Scene_Size (B.State));
   function Scene_At (B : Bench; K : Positive) return Point_Estimate is (Estimates.Scene_At (B.State, K));
   function Scene_Grid_At (B : Bench; K : Positive) return Driver.World.Supports.Grid_Point is
     (Estimates.Scene_Grid_At (B.State, K));

end Driver.World.Offline;
