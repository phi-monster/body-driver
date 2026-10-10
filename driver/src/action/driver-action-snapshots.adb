package body Driver.Action.Snapshots is

   use Driver.Numerics.Arrays;

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

   function Open_Tip (L : Lobe_State) return Vec3 is
     (if L.Path.Is_Empty then Zero3 else L.Path.First_Element.Tip);

   function Closed_Tip (L : Lobe_State) return Vec3 is
     (if L.Path.Is_Empty then Zero3 else L.Path.Last_Element.Tip);

   function Tip_Sigma (L : Lobe_State) return Real is
      Largest : Real := 0.0;
   begin
      if L.Path.Is_Empty then
         return Real'Last;
      end if;
      for N of L.Path loop
         Largest := Real'Max (Largest, N.Sigma);
      end loop;
      return Largest;
   end Tip_Sigma;

   --  The nodes around a fraction: the last whose fraction is not beyond it and the one after, or the end
   --  segment beyond an end; the same node twice for a path of one.
   procedure Around (L : Lobe_State; Fraction : Real; Before, After : out Tip_Node);

   procedure Around (L : Lobe_State; Fraction : Real; Before, After : out Tip_Node) is
      Last : constant Positive := L.Path.Last_Index;
      K    : Positive := L.Path.First_Index;
   begin
      if Last = L.Path.First_Index then
         Before := L.Path.First_Element;
         After := Before;
         return;
      end if;
      while K < Last - 1 and then L.Path (K + 1).Fraction <= Fraction loop
         K := K + 1;
      end loop;
      Before := L.Path (K);
      After := L.Path (K + 1);
   end Around;

   function Tip_At (L : Lobe_State; Fraction : Real) return Vec3 is
      Before, After : Tip_Node;
   begin
      if L.Path.Is_Empty then
         return Zero3;
      end if;
      Around (L, Fraction, Before, After);
      if After.Fraction = Before.Fraction then
         return Before.Tip;
      end if;
      return Before.Tip + ((Fraction - Before.Fraction) / (After.Fraction - Before.Fraction)) * (After.Tip - Before.Tip);
   end Tip_At;

   function Facing_At (L : Lobe_State; Fraction : Real) return Vec3 is
      Before, After : Tip_Node;
   begin
      if L.Path.Is_Empty then
         return Zero3;
      end if;
      Around (L, Fraction, Before, After);
      if abs (After.Tip - Before.Tip) = 0.0 then
         return Zero3;
      end if;
      return Unit (After.Tip - Before.Tip);
   end Facing_At;

   function Straight (Open_Tip, Closed_Tip : Vec3; Sigma : Real) return Lobe_State is
      L : Lobe_State;
   begin
      L.Path.Append (Tip_Node'(Fraction => 0.0, Tip => Open_Tip, Sigma => Sigma));
      L.Path.Append (Tip_Node'(Fraction => 1.0, Tip => Closed_Tip, Sigma => Sigma));
      return L;
   end Straight;

end Driver.Action.Snapshots;
