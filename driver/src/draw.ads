--  给脑画图:网格、编号框、数字。只是可视化,没有判据。
with Bytes; use Bytes;
package Draw is
   type Color is record
      R, G, B : U8;
   end record;
   Orange : constant Color := (255, 160, 32);
   Green : constant Color := (32, 220, 32);
   Dim_Green : constant Color := (32, 140, 32);
   Pink : constant Color := (255, 64, 200);
   White : constant Color := (255, 255, 255);
   Red : constant Color := (255, 32, 32);
   procedure Rect (RGB : in out Buf; W, H, X0, Y0, X1, Y1 : Natural; C : Color; Thick : Natural);
   procedure Number (RGB : in out Buf; W, H, X, Y, N : Natural; C : Color; Scale : Natural);
   procedure Numbered_Box (RGB : in out Buf; W, H, X0, Y0, X1, Y1, N : Natural; C : Color; Thick : Natural);
   procedure Dot (RGB : in out Buf; W, H, X, Y, Radius : Natural; C : Color);
   --  画网格并返回格心(归一化;格号 = 行优先从 1 起)
   procedure Grid (RGB : in out Buf; W, H, Cols, Rows : Natural; Centers_U, Centers_V : out Floats);
end Draw;
