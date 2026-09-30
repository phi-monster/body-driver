with Ada.Text_IO;
with Ada.Directories;
with Chan;
with Bytes; use Bytes;
with Codec;
with Json;
with Layout;
with Picture;
with Table;
package body Bodyfile is
   function Fingerprint (L : Plug.Link; F : Plug.Frame) return String is
      R : Unbounded_String;
   begin
      Append (R, "arms=" & Codec.Img (Natural (F.EE.Length)) & ";jaws=" & Codec.Img (Natural (F.Jaw.Length)) & ";cams=");
      for C of F.Cams loop
         Append (R, Codec.Img (C.W) & "x" & Codec.Img (C.H) & (if C.Has_Depth then "d" else "") & ",");
      end loop;
      Append (R, ";ee=");
      for P of L.Lay.EE loop
         Append (R, Layout.Last_Seg (P) & ",");
      end loop;
      Append (R, ";joints=");
      for P of L.Lay.Joints loop
         Append (R, Layout.Last_Seg (P) & ",");
      end loop;
      return To_String (R);
   end Fingerprint;

   function Jaws_Recorded (M : Selfmap.Body_Map) return Boolean is (Natural (M.Jaws.Length) = M.Arms);

   --  ── 写 ──
   procedure Put_Ints (B : in out Unbounded_String; Name : String; V : Ints) is
   begin
      Append (B, """" & Name & """:[");
      for I in 0 .. Natural (V.Length) - 1 loop
         Append (B, (if I > 0 then "," else "") & Codec.Img (V (I)));
      end loop;
      Append (B, "]");
   end Put_Ints;

   function Median (V : Floats) return Long_Float is
      C : Floats := V;
   begin
      if C.Is_Empty then
         return 0.0;
      end if;
      return Picture.Quantile (C, 0.5);
   end Median;

   --  手指像素(握区合空扫过的,整幅画面一格一个)按游程存:先"不是"的一段、再"是"的一段……交替,只存段长。
   --  以前不存 ⇒ 装回身体后 Zone.Tip_Px 一个指尖都认不出(Zone_Tip 悄悄退成区心、自己的手指也剔不掉),开机碰桌面量指尖直接说"没量到"(X5C3 2026-09-26)
   function Runs (M : Bools) return String is
      R : Unbounded_String;
      Cur : Boolean := False;
      N : Natural := 0;
      First : Boolean := True;
   begin
      for X of M loop
         if X /= Cur then
            Append (R, (if First then "" else ",") & Codec.Img (N));
            First := False;
            Cur := X; N := 0;
         end if;
         N := N + 1;
      end loop;
      if not M.Is_Empty then
         Append (R, (if First then "" else ",") & Codec.Img (N));
      end if;
      return To_String (R);
   end Runs;

   procedure Save (Path : String; Key : String; M : Selfmap.Body_Map; Hands : Zone.Hand_Vectors.Vector; Tables : Act.Effect_Vectors.Vector; Sch : Schema.Map) is separate;

   --  ── 读 ──
   function Load (Path : String; Key : String; M : in out Selfmap.Body_Map; Hands : in out Zone.Hand_Vectors.Vector;
                  Tables : in out Act.Effect_Vectors.Vector; Sch : in out Schema.Map; Note : out Unbounded_String) return Boolean is separate;

   procedure Merge (Stored, Fresh : Selfmap.Body_Map; Merged : out Selfmap.Body_Map; Replaced, Kept : out Natural) is separate;
end Bodyfile;
