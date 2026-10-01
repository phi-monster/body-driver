--  What the robot reports each beat, recognized by shape rather than by key
--  names (docs/body-protocol.md, section 3).
--
--  The observation is a tree of maps; every leaf has a path (the keys from
--  the root). A Layout is learned from the first observation and fixes, for
--  the whole run, which leaves are camera images, which are depth maps, which
--  are reading groups, and which reading groups are command keys (their last
--  key name appears twice: once as the reading and once as the echo of the
--  command the robot received). Everything the driver does not use, such as
--  intrinsic matrices or a reported end-effector pose, is still recognized so
--  that it is never mistaken for something else.

with Ada.Containers.Indefinite_Vectors;
with Ada.Containers.Vectors;
with Ada.Strings.Unbounded;
with Driver.Bytes;
with Driver.Clock;
with Driver.Images;

package Driver.Observations is

   use Ada.Strings.Unbounded;

   type Camera_Id is new Positive;
   type Group_Id is new Positive;

   type Camera_Info is record
      Path          : Unbounded_String;
      Width, Height : Positive;
      Depth_Path    : Unbounded_String;   --  empty when the camera reports no depth
   end record;

   type Group_Info is record
      Path        : Unbounded_String;
      Size        : Positive;              --  number of values in the group
      Command_Key : Unbounded_String;      --  empty when the group cannot be commanded
      Echo_Path   : Unbounded_String;      --  where the robot echoes the last command
   end record;

   package Camera_Vectors is new Ada.Containers.Vectors (Camera_Id, Camera_Info);
   package Group_Vectors is new Ada.Containers.Vectors (Group_Id, Group_Info);
   package Path_Vectors is new Ada.Containers.Vectors (Positive, Unbounded_String);

   type Layout is record
      Cameras          : Camera_Vectors.Vector;
      Groups           : Group_Vectors.Vector;
      Instruction_Path : Unbounded_String;
      Unused           : Path_Vectors.Vector;   --  recognized leaves the driver ignores
   end record;

   function Is_Commandable (L : Layout; G : Group_Id) return Boolean is
     (Ada.Strings.Unbounded.Length (L.Groups (G).Command_Key) > 0);

   package Reading_Vectors is new Ada.Containers.Indefinite_Vectors (Group_Id, Real_Array);
   package Image_Vectors is new Ada.Containers.Vectors (Camera_Id, Driver.Images.Image, Driver.Images."=");
   package Depth_Vectors is new Ada.Containers.Indefinite_Vectors (Camera_Id, Real_Array);

   type Observation is record
      Beat        : Driver.Clock.Beat := 0;
      Received    : Duration := 0.0;      --  Driver.Clock.Seconds when it arrived
      Images      : Image_Vectors.Vector;  --  one per camera; No_Image when missing this beat
      Readings    : Reading_Vectors.Vector;--  one per group; empty when missing this beat
      Echoes      : Reading_Vectors.Vector;--  one per group; the echoed command, or empty
      Depth       : Depth_Vectors.Vector;  --  one per camera; empty when not reported
      Instruction : Unbounded_String;
   end record;
   --  Images, Readings, Echoes and Depth always have one element per camera
   --  or group of the layout, so an index never shifts when something is
   --  missing for a beat.

   function Has_Reading (O : Observation; G : Group_Id) return Boolean is
     (O.Readings.Element (G)'Length > 0);

   function Has_Image (O : Observation; C : Camera_Id) return Boolean is
     (not Driver.Images.Is_Empty (O.Images (C)));

   procedure Recognize (Message : Driver.Bytes.Byte_Array; L : out Layout; Ok : out Boolean);
   --  Learns the layout from one decoded observation message.

   procedure Parse
     (Message : Driver.Bytes.Byte_Array;
      L       : Layout;
      Beat    : Driver.Clock.Beat;
      O       : out Observation;
      Ok      : out Boolean);
   --  Reads one observation message against a known layout.

end Driver.Observations;
