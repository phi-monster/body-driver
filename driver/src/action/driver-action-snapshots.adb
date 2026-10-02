package body Driver.Action.Snapshots is

   use type Driver.World.Thing_Id;
   use type Driver.World.Surface_Id;
   use type Driver.World.Place_Id;
   use type Driver.Robot.Arm_Id;
   use type Driver.Robot.Hand.Hand_Id;

   function Has_Thing (S : Snapshot; T : Thing_Id) return Boolean is
     (for some X of S.Things => X.Id = T);

   function Thing (S : Snapshot; T : Thing_Id) return Thing_State is
   begin
      for X of S.Things loop
         if X.Id = T then
            return X;
         end if;
      end loop;
      raise Program_Error with "Thing called without Has_Thing";
   end Thing;

   function Has_Surface (S : Snapshot; Id : Surface_Id) return Boolean is
     (for some X of S.Surfaces => X.Id = Id);

   function Surface (S : Snapshot; Id : Surface_Id) return Surface_State is
   begin
      for X of S.Surfaces loop
         if X.Id = Id then
            return X;
         end if;
      end loop;
      raise Program_Error with "Surface called without Has_Surface";
   end Surface;

   function Has_Arm (S : Snapshot; A : Arm_Id) return Boolean is
     (for some X of S.Arms => X.Id = A);

   function Arm (S : Snapshot; A : Arm_Id) return Arm_State is
   begin
      for X of S.Arms loop
         if X.Id = A then
            return X;
         end if;
      end loop;
      raise Program_Error with "Arm called without Has_Arm";
   end Arm;

   function Has_Hand (S : Snapshot; H : Hand_Id) return Boolean is
     (for some X of S.Hands => X.Id = H);

   function Hand (S : Snapshot; H : Hand_Id) return Hand_State is
   begin
      for X of S.Hands loop
         if X.Id = H then
            return X;
         end if;
      end loop;
      raise Program_Error with "Hand called without Has_Hand";
   end Hand;

   function Has_Place (S : Snapshot; P : Place_Id) return Boolean is
     (for some X of S.Places => X.Id = P);

   function Place (S : Snapshot; P : Place_Id) return Place_State is
   begin
      for X of S.Places loop
         if X.Id = P then
            return X;
         end if;
      end loop;
      raise Program_Error with "Place called without Has_Place";
   end Place;

end Driver.Action.Snapshots;
