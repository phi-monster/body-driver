--  The action map sent to the robot (docs/body-protocol.md 4).
--
--  Every command key is sent every beat, with as many values as its reading.
--  A group with a target gets the target. A group without one holds: it gets
--  the last target the driver sent it in this episode; if it was never
--  commanded in this episode, its reading of this beat; if this beat has no
--  reading, whatever was last sent for it; and if nothing was ever sent, the
--  key is left out. No value is ever invented.

with Ada.Containers.Indefinite_Ordered_Maps;
with Driver.Bytes;
with Driver.Commands;
with Driver.Msgpack;
with Driver.Observations;

package Driver.Replies is

   type State is private;

   procedure New_Episode (S : in out State);
   --  Forgets the targets of the previous episode.

   procedure Write_Action
     (S    : in out State;
      L    : Driver.Observations.Layout;
      O    : Driver.Observations.Observation;
      C    : Driver.Commands.Command;
      B    : in out Driver.Bytes.Buffer;
      Sent : out Driver.Commands.Command);
   --  Encodes the action map for one beat into B. Sent records the values
   --  actually sent for every group, holds included, for the estimators.

   function Read_Action
     (L      : Driver.Observations.Layout;
      Doc    : Driver.Msgpack.Document;
      Action : Driver.Msgpack.Node) return Driver.Commands.Command;
   --  The inverse: the command an action map carries, by command key. Used
   --  to replay what another driver sent in a recording.

private

   use type Driver.Observations.Group_Id;

   package Value_Maps is new Ada.Containers.Indefinite_Ordered_Maps (Driver.Observations.Group_Id, Real_Array);

   type State is record
      Targets   : Value_Maps.Map;   --  last target per group in this episode
      Last_Sent : Value_Maps.Map;   --  last values sent per group, any episode
   end record;

end Driver.Replies;
