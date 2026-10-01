--  What a want asks of a thing, as the rigid motion that changes it fastest.
--
--  Each quantity and each relation has one definition, written beside it, and
--  its direction comes from that definition: the twist (Driver.Action.Contact)
--  of unit size that changes it fastest. How far to go is decided step by
--  step by the execution, which reads Gap to know how much is left and
--  whether the motion closes it. Everything is measured: up is away from what
--  the subject rests on, else gravity's up; a heading is the long axis of the
--  measured footprint; left, right, nearer and farther are as the still eye
--  that sees the object best sees them. What is missing is said, never filled
--  in.

with Driver.Action.Contact;
with Driver.Action.Snapshots;
with Driver.Numerics;
with Driver.Uncertain;

package Driver.Action.Goals is

   use Driver.Numerics;
   use Driver.Uncertain;
   use Driver.Action.Snapshots;

   type Quantity is (Height, Heading, Tilt);
   --  Height   how far it is from the surface it rests on, along that
   --           surface's normal; up takes it off that surface
   --  Heading  the direction of its long axis about its up; up turns it
   --           counter-clockwise seen from above
   --  Tilt     how far it leans about the level axis square to the line from
   --           the still eye that sees it best; up leans its top away

   function Word (Q : Quantity) return String;
   --  The keyboard word.

   type Quantity_Set is array (Quantity) of Boolean;

   function Changeable (S : Snapshot) return Quantity_Set;
   --  The quantities this body can measure and change now.

   type Answer is record
      Ok     : Boolean := False;
      Motion : Driver.Action.Contact.Twist;
      Gap    : Estimate;              --  how far the relation still is from holding; Unknown when it has no end
      Done   : Boolean := False;      --  it holds already, as far as can be measured
      Why    : Unbounded_String;      --  what is missing when not Ok; what is done first, when that differs
   end record;
   --  Motion is of unit size: one unit of length along it, or one radian
   --  about it, closes one unit of Gap.

   function Up_Of (S : Snapshot; T : Thing_Id) return Vec3
     with Pre => Has_Thing (S, T);
   --  Away from what the thing rests on, or gravity's up when it rests on
   --  nothing measured; zero when neither is measured.

   function Long_Axis (S : Snapshot; T : Thing_Id; Sigma : out Real) return Vec3
     with Pre => Has_Thing (S, T);
   --  The long axis of its footprint in the plane square to Up_Of, and that
   --  axis's angular sigma; zero when the footprint is as long one way as any
   --  other within its measurement.

   function Twist_Of (S : Snapshot; T : Thing_Id; Q : Quantity; Increase : Boolean) return Answer
     with Pre => Has_Thing (S, T);
   --  The motion that changes the quantity; Gap is Unknown, a change of a
   --  quantity has no end of its own.

   type Item is record
      Centre  : Point_Estimate;
      Samples : Sample_Vectors.Vector;   --  its measured surface; empty for a point
      Sigma   : Real := Real'Last;       --  of a sample
      Up      : Vec3 := Zero3;           --  away from what it rests on; zero when it rests on nothing
      Eye     : Natural := 0;            --  the still eye that sees it best; 0 when none does
   end record;
   --  One side of a relation: a thing, a remembered place, or a part of the
   --  body.

   function Item_Of (S : Snapshot; T : Thing_Id) return Item
     with Pre => Has_Thing (S, T);

   function Point_Item (P : Point_Estimate) return Item;
   --  A remembered place, or a point of the body.

   type Pair_Relation is (Nearer, Farther, Above, Below, Left, Right, Onto, Off, Facing);
   --  Of a subject to an object:
   --    Nearer, Farther  its centre nearer to, or farther from, the still eye
   --                     that sees the object best than the object's centre
   --    Above, Below     straight over or under it along the object's up,
   --                     clear of its top or bottom
   --    Left, Right      left or right of it as that eye's image shows them
   --    Onto             resting on its top
   --    Off              clear of its top
   --    Facing           the subject's long axis pointing at it

   function Toward (S : Snapshot; Subject, Object : Item; R : Pair_Relation; Long : Vec3 := Zero3;
                    Long_Sigma : Real := Real'Last) return Answer;
   --  The motion of the subject that brings the relation about. A subject
   --  that rests on a surface moves along it, never into it. Long is the
   --  subject's long axis, for Facing.

end Driver.Action.Goals;
