--  base64 / 十六进制 / 不压缩 BMP / PGM / 数字排版。全是格式,没有一个身体量。
with Bytes; use Bytes;
package Codec is
   function Base64 (B : Buf) return String;
   function Base64_Of_String (S : String) return String;
   function Hex_To_Bytes (H : String) return Buf;
   --  24 位 BMP,高度写负数 = 自上而下(省掉翻行;翻错了的图模型照样会给一个点)。BGR。
   function BMP24 (RGB : Buf; W, H : Natural) return Buf;
   procedure Write_File (Path : String; B : Buf);
   procedure Write_PGM (Path : String; Gray : Buf; W, H : Natural);
   procedure Write_BMP (Path : String; RGB : Buf; W, H : Natural);
   procedure Make_Dir (Path : String);
   --  数字排版:定点小数、整数、六位补零(录像帧名)。
   function Fmt (X : Long_Float; Aft : Natural := 3) return String;
   function Img (N : Integer) return String;
   function Pad6 (N : Natural) return String;
   function Env (Name : String) return String;       --  没有就 ""
   function Env_Nat (Name : String; Default : Natural) return Natural;
end Codec;
