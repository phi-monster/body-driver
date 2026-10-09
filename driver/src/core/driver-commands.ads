--  A command: targets for some reading groups at one beat.
--
--  A group without a target holds: the reply carries the last target the
--  driver sent for it in this episode, or its current reading if it was never
--  commanded (Driver.Replies). Holding a group still against an obstacle is a
--  decision, so a decider that wants to stop pushing sets the target to the
--  current reading itself.

with Ada.Containers.Indefinite_Ordered_Maps;
with Driver.Observations;

package Driver.Commands is

   subtype Group_Id is Driver.Observations.Group_Id;

   type Command is private;

   Hold : constant Command;
   --  No targets: every group holds.

   procedure Set_Target (C : in out Command; G : Group_Id; Values : Real_Array);
   procedure Clear_Target (C : in out Command; G : Group_Id);

   function Has_Target (C : Command; G : Group_Id) return Boolean;
   function Target (C : Command; G : Group_Id) return Real_Array
     with Pre => Has_Target (C, G);

   function Is_Hold (C : Command) return Boolean;

   procedure Merge (Into : in out Command; From : Command; Clash : out Boolean);
   --  Every target of From joins Into, as the lanes of one beat send theirs
   --  (Driver.Beats.At_Once). A group both target keeps Into's and is a clash:
   --  two lanes may not move one group.

private

   use type Driver.Observations.Group_Id;

   package Target_Maps is new Ada.Containers.Indefinite_Ordered_Maps (Group_Id, Real_Array);

   type Command is record
      Targets : Target_Maps.Map;
   end record;

   Hold : constant Command := (Targets => Target_Maps.Empty_Map);

end Driver.Commands;
