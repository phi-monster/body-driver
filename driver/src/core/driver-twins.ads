--  Which of a command key's two groups is its reading.
--
--  A robot that reports back the command it received gives the command key
--  two groups of one size under the same last key (docs/body-protocol.md, 3):
--  its reading and its echo of the command. Nothing in the observation says
--  which is which, and its order says nothing either (a scrambled run gave
--  the echo first, and the driver read its own commands for the joints: A92).
--  They differ in how they change. The echo is the command: it changes at the
--  beats the command it echoes changed, and at no other. The reading is the
--  body: it moves when the body moves, whatever the command did, and stands
--  when the body stands. So a group that changed at a beat the command did
--  not, or stood at a beat the command changed, is no echo; when one group of
--  the pair has been found no echo and the other has not, the first is the
--  reading. The command a beat's observation echoes is the one sent before it,
--  every value of it (Driver.Replies, holds included).
--
--  Until a pair is told apart its first group, in the observation's order,
--  stands for the reading. An observation is given with each pair's reading
--  in Readings and its echo in Echoes, in the order found; at the beat a pair
--  is found the other way round, Turned is set for it, so that what kept the
--  readings of the beats before can turn them round too.

with Driver.Commands;
with Driver.Observations;

private with Ada.Containers.Indefinite_Holders;
private with Ada.Containers.Vectors;

package Driver.Twins is

   type Watch is private;

   procedure Order
     (W    : in out Watch;
      L    : Driver.Observations.Layout;
      O    : in out Driver.Observations.Observation;
      Sent : Driver.Commands.Command);
   --  One beat: what O's pairs tell of which group is the reading, Sent the
   --  values last sent to the robot (the command O echoes), and O given in the
   --  order found.

   function Reading_Is_Echo_Path (W : Watch; G : Driver.Observations.Group_Id) return Boolean;
   --  The pair of group G was found the other way round: its reading is at the
   --  layout's echo path, its echo at the group's own path.

   function Told_Apart (W : Watch; G : Driver.Observations.Group_Id) return Boolean;
   --  The pair of group G has been told apart (False for a group without an echo).

private

   package Array_Holders is new Ada.Containers.Indefinite_Holders (Real_Array);

   type Pair_State is record
      Seen           : Boolean := False;   --  a beat with both groups and the command is kept
      First_No_Echo  : Boolean := False;   --  the layout's group changed when the command did not, or the other way
      Second_No_Echo : Boolean := False;   --  the same of the group at the layout's echo path
      Decided        : Boolean := False;
      Turned         : Boolean := False;   --  the reading is at the echo path
      Last_First, Last_Second, Last_Sent : Array_Holders.Holder;   --  the beat before
   end record;

   package Pair_Vectors is new Ada.Containers.Vectors (Driver.Observations.Group_Id, Pair_State);

   type Watch is record
      Pairs : Pair_Vectors.Vector;
   end record;

end Driver.Twins;
