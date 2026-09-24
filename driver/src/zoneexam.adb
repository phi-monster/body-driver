--  离线看握区(2026-09-26):拿张开 / 合上两张灰度图(PGM,P5)跑驱动同一段 Zone.From_Frames,打出每一瓣的指尖像素、大小、框,和几瓣平均的指尖。
--  用来离线试"手上哪个点"的认法(G2C:同一只手换个姿势,头顶眼里的瓣数在 1 和 2 之间跳),不用再开一炮。
--  用法:zoneexam open.pgm closed.pgm
with Ada.Command_Line;
with Ada.Containers;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Streams.Stream_IO;
with Bytes; use Bytes;
with Zone;
with Codec;
procedure Zoneexam is
   --  读 P5 灰度图:头部三个数(宽、高、最大值)之后是原始字节
   procedure Read_PGM (Path : String; G : out Buf; W, H : out Natural) is
      package SIO renames Ada.Streams.Stream_IO;
      Fi : SIO.File_Type;
      S : SIO.Stream_Access;
      C : Character;
      function Next_Num return Natural is
         N : Natural := 0;
         Got : Boolean := False;
      begin
         loop
            Character'Read (S, C);
            if C = '#' then
               while C /= ASCII.LF loop
                  Character'Read (S, C);
               end loop;
            elsif C in '0' .. '9' then
               N := N * 10 + (Character'Pos (C) - Character'Pos ('0'));   --  十进制(纯数学)
               Got := True;
            elsif Got then
               return N;
            end if;
         end loop;
      end Next_Num;
   begin
      SIO.Open (Fi, SIO.In_File, Path);
      S := SIO.Stream (Fi);
      Character'Read (S, C);
      Character'Read (S, C);   --  "P5"
      W := Next_Num;
      H := Next_Num;
      declare
         Mx : constant Natural := Next_Num;
         pragma Unreferenced (Mx);
      begin
         null;
      end;
      G := U8_Vectors.Empty_Vector;
      G.Reserve_Capacity (Ada.Containers.Count_Type (W * H));
      for I in 1 .. W * H loop
         Character'Read (S, C);
         G.Append (U8 (Character'Pos (C)));
      end loop;
      SIO.Close (Fi);
   end Read_PGM;
   Open_G, Closed_G : Buf;
   W, H, W2, H2 : Natural;
begin
   if Ada.Command_Line.Argument_Count < 2 then
      Put_Line ("用法:zoneexam open.pgm closed.pgm");
      return;
   end if;
   Read_PGM (Ada.Command_Line.Argument (1), Open_G, W, H);
   Read_PGM (Ada.Command_Line.Argument (2), Closed_G, W2, H2);
   if W /= W2 or else H /= H2 then
      Put_Line ("两张图大小不一样");
      return;
   end if;
   declare
      Z : constant Zone.Hand_Zone := Zone.From_Frames (Open_G, Closed_G, W, H);
      Su, Sv : Long_Float := 0.0;
      Nt : Natural := 0;
   begin
      if not Z.Valid then
         Put_Line ("zone 无");
         return;
      end if;
      Put ("zone " & Codec.Img (Z.N_Lobes) & " 瓣 框 " & Codec.Img (Z.X0) & " " & Codec.Img (Z.Y0) & " " & Codec.Img (Z.X1) & " " & Codec.Img (Z.Y1));
      for I in 0 .. Z.N_Lobes - 1 loop
         declare
            Lb : constant Zone.Lobe := Zone.Lobe_Of (Z, I);
            U, V : Long_Float;
            Ok : Boolean;
         begin
            Zone.Tip_Px (Z, Lb, W, H, U, V, Ok);
            Put (" | 瓣 " & Codec.Img (I) & " 尖 " & (if Ok then Codec.Fmt (U, 1) & " " & Codec.Fmt (V, 1) else "- -") & " 大小 " & Codec.Img (Lb.Count)
                 & " 框 " & Codec.Img (Lb.X0) & " " & Codec.Img (Lb.Y0) & " " & Codec.Img (Lb.X1) & " " & Codec.Img (Lb.Y1));
            if Ok then
               Su := Su + U; Sv := Sv + V; Nt := Nt + 1;
            end if;
         end;
      end loop;
      if Nt > 0 then
         Put (" | 平均 " & Codec.Fmt (Su / Long_Float (Nt), 1) & " " & Codec.Fmt (Sv / Long_Float (Nt), 1));
      end if;
      New_Line;
   end;
end Zoneexam;
