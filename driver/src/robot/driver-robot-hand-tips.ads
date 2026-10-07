--  A hand's presses and the tips they measure.
--
--  Each lobe's tip is seen at both openings along a line of sight in the
--  tool frame. A press made at one opening goes to the lobe that leads into
--  the surface: before anything is fitted, the lobe whose line of sight lies
--  closest to the direction the tool was pressing; once the surface is
--  fitted, the lobe whose tip is foremost along its normal. All presses, on
--  one surface, are fitted together (Driver.Robot.Hand.Touch), and given out
--  again whenever the fit moves the lobe that leads, so where a press goes
--  rests on everything measured.
--
--  The presses' poses and the surface are in one frame, the arm's own
--  (Driver.Robot.Tool_In_Arm): what a hand and its arm measure by pressing
--  needs nothing of where the arm stands in the world. The surface is the
--  table the arm's own eye saw (Driver.Robot.Table_In_Arm) when it saw one,
--  so the first presses already predict their contact, and what the presses
--  make of it otherwise. That frame is what the arm's fit makes it, and the
--  arm goes on being fitted as it moves: a press keeps the arm's readings its
--  pose came from, and the presses take their poses again when the frame moves
--  (Set_Frame).

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

   procedure Set_Frame
     (B       : in out Book;
      Surface : Driver.Geometry.Plane_Estimate;
      Pose_Of : not null access function (Arm : Real_Array) return Pose_Estimate;
      Moved   : out Boolean);
   --  The frame the presses are in, as the arm's fit makes it now. Surface is
   --  the surface they are made on as measured before them, in that frame: the
   --  prior they fit it from; not Known (Driver.Geometry.Known) when nothing
   --  measured it, and the presses alone find it. Pose_Of is the tool's pose
   --  in that frame at given readings of the arm. A surface other than the
   --  last given means the arm was fitted again, and with it its frame and
   --  its unit moved: every press kept takes the pose its readings have now,
   --  and all are fitted again (Moved). A press made when the arm's pose was
   --  not known keeps the pose it has.

   procedure Add (B : in out Book; Press : Driver.Robot.Hand.Presses.Event; At_Opening : Opening);
   --  A press made with the closer at that opening.

   function Lobes (B : Book) return Natural;
   function Pressed (B : Book) return Natural;
   --  Every press kept, agreeing or not.

   function Tip (B : Book; Lobe : Positive; At_Opening : Opening) return Point_Estimate;
   --  The tip as the presses fix it: the lowest of their hits, unknown until
   --  a press stopped by the tip fixes it (Driver.Robot.Hand.Touch). One
   --  press fixes it and nothing has checked it: provisional, as long as it
   --  is not Confirmed.

   function Confirmed (B : Book; Lobe : Positive; At_Opening : Opening) return Boolean;
   --  A second press, from a pose distinct from the first's, landed on the
   --  tip within the noise.

   function Distance (B : Book; Lobe : Positive; At_Opening : Opening) return Estimate;
   --  How far along its line of sight from the eye the tip is, in the arm's
   --  own unit; unknown when the tip is.

   function Direction (B : Book; Lobe : Positive; At_Opening : Opening) return Direction_Estimate;
   --  The direction into the surface at the presses the tip rests on, tool
   --  frame, averaged; its sigma is their spread.

   function Agreeing (B : Book; Lobe : Positive; At_Opening : Opening) return Natural;
   --  The presses the tip rests on.

   function Latest_Agrees (B : Book) return Boolean;
   --  The press kept last is one its tip rests on: the tip stopped it, and
   --  not something else that left the tip above the surface.

   function Surface (B : Book) return Driver.Geometry.Plane_Estimate;
   --  The surface as the presses and the prior make it, in the frame of the
   --  presses' poses; unknown until the presses fit it.

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
      Table  : Driver.Geometry.Plane_Estimate;   --  the prior surface, as Set_Surface was given it
      Fitted : Fit_Holders.Holder;
   end record;

end Driver.Robot.Hand.Tips;
