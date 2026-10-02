with Driver.Action.Execution;
with Driver.Action.Goals;
with Driver.Action.Plants.Live;
with Driver.Action.Snapshots;
with Driver.Beats;

package body Driver.Action is

   use Driver.Action.Snapshots;
   use Driver.Uncertain;
   use type Driver.Robot.Arm_Id;

   --  The estimates at the latest beat; within a beat's window.
   function Now (C : Context) return Snapshot is
     (Driver.Action.Plants.Live.Snapshot_Of (C.Robot.all, C.Hands.all, C.Scene.all, Driver.Beats.Latest.all));

   function Quantities (C : Context) return Name_Vectors.Vector is
      Can : constant Goals.Quantity_Set := Goals.Changeable (Now (C));
      R   : Name_Vectors.Vector;
   begin
      for Q in Goals.Quantity loop
         if Can (Q) then
            R.Append (String'(Goals.Word (Q)));
         end if;
      end loop;
      return R;
   end Quantities;

   function Meaning (Quantity : String) return String is
   begin
      for Q in Goals.Quantity loop
         if Goals.Word (Q) = Quantity then
            return Goals.Meaning (Q);
         end if;
      end loop;
      return "";
   end Meaning;

   function Can_Bind (C : Context; R : Role) return Boolean is (Driver.Action.Execution.Bindable (Now (C), R));

   function Role_Hand (C : Context; R : Role) return Driver.Robot.Hand.Hand_Id'Base is
      S : constant Snapshot := Now (C);
      A : Arm_Id;
   begin
      if R = Grasper and then Driver.Action.Execution.Bound_Arm (S, R, A) then
         for H of S.Hands loop
            if H.Arm = A then
               return H.Id;
            end if;
         end loop;
      end if;
      return 0;
   end Role_Hand;

   function Role_Point (C : Context; R : Role) return Point_Estimate is
      S : constant Snapshot := Now (C);
      A : Arm_Id;
   begin
      if Driver.Action.Execution.Bound_Arm (S, R, A) then
         return Driver.Action.Execution.Part_Point (S, A);
      end if;
      return (others => <>);
   end Role_Point;

   function Usable_In (S : Snapshot; R : Relation) return Boolean renames Driver.Action.Execution.Usable;

   function Usable (C : Context; R : Relation) return Boolean is (Usable_In (Now (C), R));

   function No (Why : String; Instead : String := "") return Verdict is
     ((Ok => False, Why => To_Unbounded_String (Why), Alternative => To_Unbounded_String (Instead)));

   --  The gates in milliseconds: it can be said with this body, what it
   --  relies on is measured, and nothing measured already rules it out.
   function Check (C : Context; W : Want) return Verdict is
      S : constant Snapshot := Now (C);
   begin
      if W.Until_Endings (Arrived) then
         return No ("only the one who wrote it can tell when it has arrived; wait for what I can measure",
                    "until settled");
      elsif W.Until_Endings (Refused) then
         return No ("refused is what I answer, not something to wait for", "until stuck");
      elsif not (for some E in Ending => W.Until_Endings (E)) then
         return No ("it says nothing to end on", "until settled");
      elsif not (for some A of S.Arms => Known (A.Step) and then Known (A.Rate)) then
         return No ("I have not measured how my arms answer a command yet, so I cannot plan a step");
      end if;
      case W.Kind is
         when Change =>
            declare
               Can   : constant Goals.Quantity_Set := Goals.Changeable (S);
               Count : Natural := 0;
               Q     : Goals.Quantity := Goals.Quantity'First;
            begin
               if not Has_Thing (S, W.Thing) then
                  return No ("I do not see that thing now");
               end if;
               for K in Goals.Quantity loop
                  if Can (K) then
                     Count := Count + 1;
                     if Count = W.Quantity then
                        Q := K;
                     end if;
                  end if;
               end loop;
               if W.Quantity > Count then
                  return No ("I cannot measure and change that quantity of it now");
               elsif Thing (S, W.Thing).Samples.Is_Empty then
                  return No ("I have not measured the surface of it, so I cannot choose where to meet it");
               end if;
               declare
                  A : constant Goals.Answer := Goals.Twist_Of (S, W.Thing, Q, W.Increase);
               begin
                  if not A.Ok then
                     return No (To_String (A.Why));
                  end if;
               end;
            end;
         when Interval =>
            if W.Constraints.Is_Empty then
               return No ("it asks for nothing");
            end if;
            for K of W.Constraints loop
               if K.Subject.Kind = Role_Operand and then not Driver.Action.Execution.Bindable (S, K.Subject.The_Role) then
                  return No ("no part of me can be that role now");
               elsif K.Subject.Kind = Thing_Operand and then not Has_Thing (S, K.Subject.Thing) then
                  return No ("I do not see the thing that is to move");
               elsif K.Object.Kind = Thing_Operand and then not Has_Thing (S, K.Object.Thing) then
                  return No ("I do not see the thing it is to be " & Relation'Image (K.Relation) & " of");
               elsif K.Object.Kind = Place_Operand and then not Has_Place (S, K.Object.Place) then
                  return No ("I do not know that place");
               elsif not Usable_In (S, K.Relation) then
                  return No ("I cannot measure or bring about " & Relation'Image (K.Relation) & " with this body now");
               end if;
            end loop;
      end case;
      return (Ok => True);
   end Check;

   procedure Execute (C : in out Context; W : Want; R : out Result) is
      P : Driver.Action.Plants.Live.Live (C.Robot, C.Hands, C.Scene);
   begin
      Driver.Action.Execution.Execute (P, W, R);
   end Execute;

end Driver.Action;
