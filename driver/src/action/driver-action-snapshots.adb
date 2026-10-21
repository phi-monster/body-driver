with Ada.Numerics.Long_Elementary_Functions;

package body Driver.Action.Snapshots is

   use Ada.Numerics.Long_Elementary_Functions;
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

   function Largest_Sigma (Covariance : Mat3) return Real is
     (if Covariance (1, 1) >= Real'Last or else Covariance (2, 2) >= Real'Last or else Covariance (3, 3) >= Real'Last
      then Real'Last
      else Sqrt (Real'Max (Covariance (1, 1), Real'Max (Covariance (2, 2), Covariance (3, 3)))));

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

   function Unseen_Sides (Seen : Sample_Vectors.Vector; Pitch : Real; On, Up : Vec3) return Sample_Vectors.Vector is
      Result : Sample_Vectors.Vector;
      N      : constant Natural := Natural (Seen.Length);
   begin
      if N < 3 or else not (Pitch > 0.0) or else not (abs Up > 0.0) then
         return Result;
      end if;
      declare
         U : constant Vec3 := Unit (Up);
         --  Across Up: from the axis it leans along least, which is never along it.
         Least : Positive := 1;
         T1, T2 : Vec3;
         type Flat is record
            A, B, H : Real;   --  along T1 and T2 in the surface, and up from it
         end record;
         Pts  : array (1 .. N) of Flat;
         Hull : array (1 .. 2 * N) of Positive := [others => 1];
         Top  : Natural := 0;
         function Turn (O, P, Q : Positive) return Real is
           ((Pts (P).A - Pts (O).A) * (Pts (Q).B - Pts (O).B) - (Pts (P).B - Pts (O).B) * (Pts (Q).A - Pts (O).A));
         Order : array (1 .. N) of Positive;
         function Seen_Near (P : Vec3) return Boolean is
         begin
            for S of Seen loop
               if abs (S.Point - P) <= Pitch then
                  return True;
               end if;
            end loop;
            return False;
         end Seen_Near;
      begin
         for K in 2 .. 3 loop
            if abs U (K) < abs U (Least) then
               Least := K;
            end if;
         end loop;
         T1 := Unit (Cross (U, [for K in 1 .. 3 => (if K = Least then 1.0 else 0.0)]));
         T2 := Cross (U, T1);
         for K in 1 .. N loop
            declare
               D : constant Vec3 := Seen (K).Point - On;
            begin
               Pts (K) := (A => D * T1, B => D * T2, H => D * U);
               Order (K) := K;
            end;
         end loop;
         --  The convex outline, counter-clockwise about Up (Andrew's monotone chain), in sorted order by insertion.
         for I in 2 .. N loop
            declare
               V : constant Positive := Order (I);
               J : Natural := I - 1;
            begin
               while J >= 1 and then (Pts (Order (J)).A > Pts (V).A
                                      or else (Pts (Order (J)).A = Pts (V).A and then Pts (Order (J)).B > Pts (V).B))
               loop
                  Order (J + 1) := Order (J);
                  J := J - 1;
               end loop;
               Order (J + 1) := V;
            end;
         end loop;
         for I in 1 .. N loop
            while Top >= 2 and then Turn (Hull (Top - 1), Hull (Top), Order (I)) <= 0.0 loop
               Top := Top - 1;
            end loop;
            Top := Top + 1;
            Hull (Top) := Order (I);
         end loop;
         declare
            Lower : constant Natural := Top + 1;
         begin
            for I in reverse 1 .. N - 1 loop
               while Top >= Lower and then Turn (Hull (Top - 1), Hull (Top), Order (I)) <= 0.0 loop
                  Top := Top - 1;
               end loop;
               Top := Top + 1;
               Hull (Top) := Order (I);
            end loop;
         end;
         --  Hull (Top) is Hull (1) again: the edges are Hull (I) to Hull (I + 1).
         for I in 1 .. Top - 1 loop
            declare
               Pa : constant Flat := Pts (Hull (I));
               Pb : constant Flat := Pts (Hull (I + 1));
               Da : constant Real := Pb.A - Pa.A;
               Db : constant Real := Pb.B - Pa.B;
               L  : constant Real := Sqrt (Da * Da + Db * Db);
            begin
               if L > 0.0 then
                  declare
                     Out_Of : constant Vec3 := (Db / L) * T1 - (Da / L) * T2;
                     Steps  : constant Positive := Positive (Real'Ceiling (L / Pitch));
                  begin
                     for K in 0 .. Steps - 1 loop
                        declare
                           S      : constant Real := Real (K) / Real (Steps);
                           Height : constant Real := Pa.H + S * (Pb.H - Pa.H);
                           Base   : constant Vec3 := On + (Pa.A + S * Da) * T1 + (Pa.B + S * Db) * T2;
                        begin
                           if Height >= 0.0 then
                              for J in 0 .. Natural (Real'Floor (Height / Pitch)) loop
                                 declare
                                    P : constant Vec3 := Base + (Real (J) * Pitch) * U;
                                 begin
                                    if not Seen_Near (P) then
                                       Result.Append (Sample'(Point => P, Normal => Out_Of));
                                    end if;
                                 end;
                              end loop;
                           end if;
                        end;
                     end loop;
                  end;
               end if;
            end;
         end loop;
      end;
      return Result;
   end Unseen_Sides;

end Driver.Action.Snapshots;
