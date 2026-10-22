--  What a desktop camera does to the picture its sensor makes, after the renderer: an exposure it adapts to what
--  it sees, a step of it while it starts, the sensor's noise, and MJPEG's 8x8 blocks, which leave a flat patch the
--  same from frame to frame and make a textured one jump (the kiwi's wrist and head cameras, 10-13 and 10-14: flat
--  pixels identical between frames, textured ones not, and a start-up exposure step that set the rest counts'
--  maximum for good); and now and then no frame at all for a while (a USB camera's hiccups: R4's wrist eye left
--  the bus). Every camera draws its own quality, exposure and hiccups from the seed.

with Driver;
with Driver.Bytes;
with Interfaces;

package Fake_Camera is

   subtype Real is Driver.Real;

   type State is private;

   type Noise_Source is private;
   --  The sensors' noise: one generator the cameras draw from in turn (xorshift, four of its uniforms to a Gaussian).

   function Start_Noise (Seed : Natural) return Noise_Source;

   function Start (Seed : Natural; Index : Positive; Exposure, Mjpeg, Gaps : Boolean) return State;
   --  Camera Index of the body of Seed, with the quirks given.

   procedure Expose
     (S             : in out State;
      Radiance      : Driver.Real_Array;
      Width, Height : Positive;
      Noise_Level   : Real;
      Noise         : in out Noise_Source;
      Picture       : out Driver.Bytes.Byte_Array)
     with Pre => Radiance'Length = 3 * Width * Height and then Picture'Length = Radiance'Length;
   --  Radiance is RGB in grey levels as the renderer gives it, rows top to bottom; Picture the bytes the camera
   --  sends: exposed, noisy, rounded and clipped, and (with MJPEG) compressed and decoded back. The exposure moves
   --  towards a mid grey by a third of a stop a frame at most, once it is off by more than a sixth of one.

   procedure Next_Beat (S : in out State);
   --  Whether the camera gives a frame this beat (Gives): a hiccup, once begun, lasts a drawn number of beats.

   function Gives (S : State) return Boolean;

   function Describe (S : State) return String;

private

   type Noise_Source is record
      State : Interfaces.Unsigned_64 := 1;
   end record;

   type State is record
      Seed      : Natural := 0;
      Index     : Positive := 1;
      Exposure  : Boolean := True;
      Mjpeg     : Boolean := True;
      Gaps      : Boolean := True;
      Quality   : Natural := 75;
      Gain      : Real := 1.0;        --  the exposure, a factor on the radiance
      Rng       : Interfaces.Unsigned_64 := 1;
      Gap_Left  : Natural := 0;       --  beats of the hiccup still to come
      Gap_Rate  : Real := 0.0;        --  the chance a hiccup begins in a beat
      Gap_Most  : Positive := 1;      --  the longest a hiccup lasts, beats
   end record;

end Fake_Camera;
