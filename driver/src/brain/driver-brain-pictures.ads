--  The pictures the brain is shown, and their encoding for the brain service
--  (docs/brain-service.md).
--
--  A round's question carries one picture: the eye the brain looks through,
--  at full size, with every other eye that has a picture this beat in one
--  strip below it, side by side in camera order, each scaled to share the
--  width and keeping its own proportions. Nothing is drawn on the pictures:
--  drawn marks were measured to hurt the model's sight (LANGUAGE.md 17.7).
--  Pictures travel as uncompressed 24-bit BMP in base64, encoded as the
--  instrument's pictures are (Driver.Instrument.Bitmap).

with Driver.Images;
with Driver.Brain.Names;
with Driver.Instrument;

package Driver.Brain.Pictures is

   function Compose
     (Main   : Driver.Images.Image;
      Rest : Driver.Brain.Names.Eye_Vectors.Vector;
      Images : not null access function (E : Driver.Brain.Names.Eye_Id) return Driver.Images.Image)
      return Driver.Images.Image
     with Pre => not Driver.Images.Is_Empty (Main);
   --  Main above, the Rest below it in the order given.

   function Data_Url (I : Driver.Images.Image) return String is
     ("data:image/bmp;base64," & Driver.Instrument.Bitmap (I))
     with Pre => not Driver.Images.Is_Empty (I);

end Driver.Brain.Pictures;
