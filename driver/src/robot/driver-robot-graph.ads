--  The body as a graph of groups and eyes, read off what each group's push
--  does to each eye (Driver.Robot.Lockin).
--
--  A commandable group that moves the whole image of every eye, with two or
--  more eyes, carries the body; one that moves the whole image of some eyes
--  is an arm carrying them. An eye whole for several arms rides on the one
--  whose push moves the largest share of it. A group that only moves
--  patches is a closer of the arm in whose eye its patch is strongest, or a
--  part when no arm's eye shows it; a group some eye is undecided about
--  (that eye might ride on it) is neither until the eye decides. Groups the
--  robot does not take commands for are sensors when their readings change
--  and inert when they never do. Arms are numbered in the order of their
--  groups.
--
--  The porting contract is checked on the way: a commandable group whose
--  pushes never moved its reading breaks clause 1; one whose reading moved
--  but which no eye saw move breaks clause 2.

private package Driver.Robot.Graph is

   procedure Derive (M : in out Model);
   --  Recomputes roles, arms, eye mounts, the carrier and contract clauses
   --  from the current effects and streams.

   function Effect (M : Model; G : Group_Id; E : Eye_Id) return Eye_Effect;

end Driver.Robot.Graph;
