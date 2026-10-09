--  What an eye that rides on an arm shows of the robot itself, from the
--  arm's own motion: the parts of the robot that go with the eye stay where
--  they are in its picture while the arm moves, and the world does not.
--
--  The memory keeps, for each setting of the readings that change how the
--  robot looks in the eye (the key: a closer's readings), the eye's still
--  frames of that setting, one frame for each pose of the rest of the body it
--  was seen still in, as per-pixel statistics (Driver.Pixels.View). The
--  deviation of a pixel over those poses is small where the robot is and
--  large where the world is, wherever the world has texture; where it is
--  flat, nothing tells it from the robot. Whoever asks reads the deviation
--  (Variance of the anchor): what to make of it, and where it is not enough
--  to tell, is not decided here.
--
--  A key the closer leaves after one pose is forgotten, and no more than
--  Capacity keys of two poses or more are kept: the fewest-posed goes. Frames
--  count only while the eye's picture has stopped changing (Still), and a pose
--  is new when the rest of the body moved by the caller's one test of motion
--  (the body's own: a channel an eye watches moves only by a step that eye
--  can see).
--
--  Nothing here knows what a hand is. The quantity is the Model's: the pixels
--  of an eye that show the robot itself (Driver.Robot.Self_Mask), which is
--  path A's. This package is written with no Hand types so that it can move
--  there as a rename, Driver.Robot.Hand.Selfsight to Driver.Robot.Self_Image,
--  when the Model keeps one memory for each eye that rides on an arm; until
--  then Driver.Robot.Hand.Sweep holds one for each closer and eye.

with Driver.Images;
with Driver.Pixels;

private with Ada.Containers.Indefinite_Holders;
private with Ada.Containers.Vectors;

package Driver.Robot.Hand.Selfsight is

   Needed : constant := 2;
   --  The poses of the rest of the body a deviation needs (Anchored).

   Capacity : constant := 8;
   --  How many keys seen from two poses or more an eye remembers: memory,
   --  one view of the eye's frame size (two reals a pixel) for each.

   type Memory is private;

   function Start (Width, Height : Positive; Key_Noise : Real_Array) return Memory;
   --  Key_Noise: the noise of each reading of the key at rest (path A
   --  measures them); two readings are the same key when their difference
   --  is within it, twice over; a zero noise means the reading repeats
   --  exactly.

   procedure Set_Key_Noise (M : in out Memory; Key_Noise : Real_Array)
     with Pre => Key_Noise'Length = Key_Noise_Length (M);
   --  The keys' noise as the caller now measures it, for the readings to come
   --  and the settings kept.

   function Key_Noise_Length (M : Memory) return Natural;

   procedure Observe
     (M          : in out Memory;
      Key        : Real_Array;
      Rest       : Real_Array;
      Still      : Boolean;
      Image      : Driver.Images.Image;
      Rest_Moved : not null access function (Before, After : Real_Array) return Boolean);
   --  One beat of the eye: the key's readings now, every other reading, whether
   --  the eye's picture has stopped changing, and the picture. A still frame is
   --  kept under the key when the rest of the body has moved since the frame
   --  kept before it (Rest_Moved), or when it is the first.

   function Poses (M : Memory; Key : Real_Array) return Natural;
   --  How many poses of the rest of the body the key's readings were seen
   --  still in; 0 when they never were, or were forgotten.

   function Anchored (M : Memory; Key : Real_Array) return Boolean;
   --  The key was seen from two poses or more, which a deviation needs.

   function Anchor_For (M : Memory; Key : Real_Array) return Driver.Pixels.View
     with Pre => Anchored (M, Key);
   --  The frames of those poses, one each: its Frames is Poses, the variance
   --  of a pixel its variance over the poses.

private

   package Real_Holders is new Ada.Containers.Indefinite_Holders (Real_Array);

   type Setting is record
      Key   : Real_Holders.Holder;
      Rest  : Real_Holders.Holder;   --  the rest of the body at the latest kept frame
      First : Driver.Images.Image;   --  the one frame of a key seen from one pose
      Seen  : Driver.Pixels.View;    --  from two poses on, all the frames
      Poses : Natural := 0;
      Tick  : Natural := 0;          --  when the key was last seen
   end record;

   package Setting_Vectors is new Ada.Containers.Vectors (Positive, Setting);

   type Memory is record
      Width, Height : Natural := 0;
      Noise         : Real_Holders.Holder;
      Settings      : Setting_Vectors.Vector;
      Clock         : Natural := 0;
   end record;

end Driver.Robot.Hand.Selfsight;
