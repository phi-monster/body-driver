--  Pictures of the fake world: a ray through every pixel of a camera, the
--  nearest solid it meets, shaded by a fixed light (diffuse, a little
--  specular on glossy surfaces, and a view-dependent share, so that a moving
--  eye sees the light change as the lab's renderer showed it), the surface's
--  procedural texture in its own frame. No shadows, no anti-aliasing: the
--  renderer's own smoothing over beats is Fake_World's settle quirk.

with Driver;
with Fake_World;

package Fake_Render is

   Supersample : Positive := 1;
   --  Rays per pixel along each axis, their light averaged: a camera's pixel integrates the light over its area, and a
   --  texture finer than a pixel shows as its mean there, where one ray through the pixel's centre shows whatever value
   --  of the texture the centre falls on (fake_robot --supersample).

   procedure Render (W : Fake_World.World; C : aliased Fake_World.Camera; Pixels : aliased in out Driver.Real_Array);

   procedure Nearest
     (W : Fake_World.World; C : Fake_World.Camera; U, V : Driver.Real; Solid : out Natural; Along : out Driver.Real);
   --  The solid the ray through the picture's place (U, V) meets first (pixels from the picture's corner, a pixel's
   --  centre at a half), and how far along the ray; 0 and Real'Last for none. For studies against the fake's truth
   --  (tools/truth_points): the same rays and the same solids as Render's.
   --  RGB in [0, 255] per pixel, rows top to bottom, three values a pixel;
   --  Pixels'Length is 3 * C.Width * C.Height. One picture at a time: the
   --  bands are rendered by workers that live as long as the program.

end Fake_Render;
