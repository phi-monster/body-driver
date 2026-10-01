--  A hand's presses and the tips they measure.
--
--  Each lobe's tip is seen at both openings along a line of sight in the
--  tool frame. A press made at one opening goes to the lobe that leads into
--  the surface: before anything is fitted, the lobe whose line of sight lies
--  closest to the direction the tool was pressing; once the surface is
--  fitted, the lobe whose tip is foremost along its normal. All presses, on
--  one surface nothing measured before, are fitted together
--  (Driver.Robot.Hand.Touch), and given out again whenever the fit moves
--  the lobe that leads, so where a press goes rests on everything measured.

with Ada.Containers.Indefinite_Holders;
with Ada.Containers.Vectors;
with Driver.Geometry;
with Driver.Robot.Hand.Presses;
with Driver.Robot.Hand.Touch;

package Driver.Robot.Hand.Tips is

   type Sight_Of is record
      Known : Boolean := False;
      Ray   : Ray_Estimate;    --  tool frame
   end record;

   type Lobe_Sights is array (Opening) of Sight_Of;
   type Sight_Table is array (Positive range <>) of Lobe_Sights;
   --  One row per lobe.

   type Book is private;

   procedure Set_Sights (B : in out Book; Sights : Sight_Table);
   --  The lobes' lines of sight; a change in the number of lobes forgets the
   --  presses, which were given to lobes by number.

   procedure Add (B : in out Book; Press : Driver.Robot.Hand.Presses.Event; At_Opening : Opening);
   --  A press made with the closer at that opening.

   function Lobes (B : Book) return Natural;
   function Pressed (B : Book) return Natural;
   --  Every press kept, agreeing or not.

   function Tip (B : Book; Lobe : Positive; At_Opening : Opening) return Point_Estimate;
   --  Unknown until enough agreeing presses fix it.

   function Direction (B : Book; Lobe : Positive; At_Opening : Opening) return Direction_Estimate;
   --  The direction into the surface at the tip's agreeing presses, tool
   --  frame, averaged; its sigma is their spread.

   function Agreeing (B : Book; Lobe : Positive; At_Opening : Opening) return Natural;

   function Surface (B : Book) return Driver.Geometry.Plane_Estimate;

private

   type Kept is record
      Event   : Driver.Robot.Hand.Presses.Event;
      Opening : Hand.Opening;
      Lobe    : Natural := 0;   --  0: not given to a lobe yet
      Agrees  : Boolean := False;
   end record;

   package Kept_Vectors is new Ada.Containers.Vectors (Positive, Kept);
   package Table_Holders is new Ada.Containers.Indefinite_Holders (Sight_Table);
   package Fit_Holders is new Ada.Containers.Indefinite_Holders (Driver.Robot.Hand.Touch.Fit_Result,
                                                                 Driver.Robot.Hand.Touch."=");

   type Book is record
      Sights : Table_Holders.Holder;
      Kept   : Kept_Vectors.Vector;
      Fitted : Fit_Holders.Holder;
   end record;

end Driver.Robot.Hand.Tips;
