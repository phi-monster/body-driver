--  The fake robot's pictures made on a GPU (tools/gpu/fake_gpu.cu): the renderer's rays, the settle's accumulation and
--  the camera's exposure, noise and MJPEG, a picture a call, in single precision; the bytes back. The library is
--  loaded at run time from where FAKE_GPU names it, so the fake builds and runs without one on any machine (then
--  Available is False and Fake_Render and Fake_Camera make the pictures).

with Driver.Bytes;
with Fake_Camera;
with Fake_World;

package Fake_GPU is

   function Available return Boolean;
   --  The library is loaded and found a device; tried once.

   procedure Picture
     (W           : Fake_World.World;
      Index       : Positive;
      Render      : Boolean;
      Share       : Driver.Real;
      S           : in out Fake_Camera.State;
      Noise       : in out Fake_Camera.Noise_Source;
      Bytes       : out Driver.Bytes.Byte_Array)
     with Pre => Available;
   --  Camera Index's picture: rendered anew where Render is set (else the last render stands), accumulated by Share
   --  (Fake_World's settle), exposed at the camera's gain with the world's image noise and coded as the camera codes;
   --  the camera's exposure then adapted to it (Fake_Camera.Adapt).

end Fake_GPU;
