--  Pictures of the fake world: a ray through every pixel of a camera, the
--  nearest solid it meets, shaded by a fixed light (diffuse, a little
--  specular on glossy surfaces, and a view-dependent share, so that a moving
--  eye sees the light change as the lab's renderer showed it), the surface's
--  procedural texture in its own frame. No shadows, no anti-aliasing: the
--  renderer's own smoothing over beats is Fake_World's settle quirk.

with Driver;
with Fake_World;

package Fake_Render is

   procedure Render (W : Fake_World.World; C : Fake_World.Camera; Into : in out Driver.Real_Array);
   --  RGB in [0, 255] per pixel, rows top to bottom, three values a pixel;
   --  Into'Length is 3 * C.Width * C.Height.

end Fake_Render;
