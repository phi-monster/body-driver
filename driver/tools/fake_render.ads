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
   --  Its square the rays per pixel, their light averaged, their places in the pixel a lattice that puts each at its
   --  own distance along any line: a camera's pixel integrates the light over its area, a texture finer than a pixel
   --  shows as its mean there and an edge moving across it changes it in proportion, where one ray through the pixel's
   --  centre shows whatever value of the texture the centre falls on and an edge all at once (fake_robot
   --  --supersample).

   Lens_Blur : Driver.Real := 0.0;
   --  The lens's spread of a point's light over the picture, a Gaussian's standard deviation in pixels: a real camera's
   --  lens and its sensor's demosaicing blur what a pinhole would show, and a picture of no blur at all aliases (an
   --  edge that moves a tenth of a pixel changes the pixels by what its place in the pixel makes it, not by its slope).

   Even_Light : Boolean := False;
   --  Every surface lit alike whatever way it faces, no highlight: a surface that turns keeps its shade (studies of
   --  the picture's change as an arm turns, tools/driver-robot-flow_bench FLOW_FLAT).

   function Raw_Shift (Axis : Positive) return Driver.Real;
   --  How far the rays of a pixel fall on average from its centre along Axis (1 across, 2 down), in pixels, at the
   --  present Supersample and Lens_Blur, before Render takes it off every ray: the lattice's and the blur's draws
   --  are finitely many (64 rays and a blur of 0.6 px: -0.028 across, -0.006 down). Render's pixels hold the light
   --  about their centres; the renders before 10-21 held it about their centres plus this, their content off the
   --  camera's own principal point by it (studies of their caches, tools/driver-robot-flow_bench FLOW_STUDY_CENTRED).

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
