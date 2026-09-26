--  离线量跟点仪器准不准(2026-09-26,V1b 5 分钟一炮):开机扫描落盘的一段画面(look/sweep_*.bmp)里,
--  整幅格点从第一张一路跟到后面每一张(Track-On2),同时拿配点仪器从第一张直接配到那一张(RoMa)、再配回来 ——
--  回到原处 1 px 以内的那些当参照,逐张报:跟点说看得见的有几个、跟点和配点差多少(中位 / 九成)。不改驱动的任何量。
--  用法:trackexam <仪器主机> <端口> <第一张.bmp> <第二张.bmp> ...
with Ada.Command_Line; use Ada.Command_Line;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Bytes; use Bytes;
with Codec;
with Instrument;
with Geom;
procedure Trackexam is
   package Sorting is new F64_Vectors.Generic_Sorting;
   Host : constant String := Argument (1);
   Port : constant Natural := Natural'Value (Argument (2));
   Gx : constant := 32;   --  格点 32 × 24(同驱动,次数)
   Gy : constant := 24;
   Rgb0 : Buf;
   W, H : Natural;
   Ok : Boolean;
   Pts : Instrument.Track_Vectors.Vector;
   Qpts : Instrument.Match_Vectors.Vector;
   Id : Integer;
   Err : Unbounded_String;
   function Q (V : Floats; F : Long_Float) return Long_Float is
      S : Floats := V;
   begin
      if S.Is_Empty then
         return -1.0;
      end if;
      Sorting.Sort (S);
      return S (Natural'Min (Natural (S.Length) - 1, Natural (F * Long_Float (S.Length))));
   end Q;
begin
   if Argument_Count < 4 then
      Put_Line ("用法:trackexam <仪器主机> <端口> <第一张.bmp> <第二张.bmp> ...");
      return;
   end if;
   Codec.Read_BMP (Argument (3), Rgb0, W, H, Ok);
   if not Ok then
      Put_Line ("读不了 " & Argument (3));
      return;
   end if;
   for Iy in 0 .. Gy - 1 loop
      for Ix in 0 .. Gx - 1 loop
         declare
            U : constant Long_Float := (Long_Float (Ix) + 0.5) * Long_Float (W) / Long_Float (Gx);
            V : constant Long_Float := (Long_Float (Iy) + 0.5) * Long_Float (H) / Long_Float (Gy);
         begin
            Pts.Append (Instrument.Track_Pt'(U => U, V => V, Seen => True, Conf => 1.0));
            Qpts.Append (Instrument.Match_Pt'(U => U, V => V, Cert => 0.0, others => <>));
         end;
      end loop;
   end loop;
   declare
      Dummy : constant Instrument.Track_Vectors.Vector := Instrument.Track_Start (Host, Port, Rgb0, W, H, Pts, Id, Err);
      pragma Unreferenced (Dummy);
   begin
      if Id < 0 then
         Put_Line ("跟点没开成:" & To_String (Err));
         return;
      end if;
   end;
   for A in 4 .. Argument_Count loop
      declare
         Rgb : Buf;
         Wk, Hk : Natural;
      begin
         Codec.Read_BMP (Argument (A), Rgb, Wk, Hk, Ok);
         if Ok then
            declare
               T : constant Instrument.Track_Vectors.Vector := Instrument.Track_Step (Host, Port, Id, Rgb, Wk, Hk, Err);
               R : constant Instrument.Match_Vectors.Vector := Instrument.Match (Host, Port, Rgb0, W, H, Rgb, Wk, Hk, Qpts, Err);
               Rc : constant Instrument.Match_Vectors.Vector := Instrument.Match (Host, Port, Rgb0, W, H, Rgb, Wk, Hk, Qpts, Err, Coarse => True);
               Dc : Floats;
               Back_Q : Instrument.Match_Vectors.Vector;
               Idx : Ints;
               Diffs, Disp : Floats;
               N_Seen, N_Ref : Natural := 0;
            begin
               if Natural (R.Length) = Natural (Qpts.Length) then
                  for P in 0 .. Natural (Qpts.Length) - 1 loop
                     if R (P).U >= 0.0 and then R (P).U < Long_Float (Wk) and then R (P).V >= 0.0 and then R (P).V < Long_Float (Hk) then
                        Back_Q.Append (Instrument.Match_Pt'(U => R (P).U, V => R (P).V, Cert => 0.0, others => <>));
                        Idx.Append (P);
                     end if;
                  end loop;
               end if;
               declare
                  Bk : constant Instrument.Match_Vectors.Vector := Instrument.Match (Host, Port, Rgb, Wk, Hk, Rgb0, W, H, Back_Q, Err);
               begin
                  for K in 0 .. Natural (Idx.Length) - 1 loop
                     declare
                        P : constant Natural := Natural (Idx (K));
                     begin
                        if Natural (Bk.Length) = Natural (Back_Q.Length)
                          and then Geom.Norm ([Bk (K).U - Qpts (P).U, Bk (K).V - Qpts (P).V, 0.0]) < 1.0   --  配回原处 1 px 以内当参照(像素,协议)
                        then
                           N_Ref := N_Ref + 1;
                           if P < Natural (T.Length) and then T (P).Seen then
                              Diffs.Append (Geom.Norm ([T (P).U - R (P).U, T (P).V - R (P).V, 0.0]));
                              Disp.Append (Geom.Norm ([R (P).U - Qpts (P).U, R (P).V - Qpts (P).V, 0.0]));
                           end if;
                           if P < Natural (Rc.Length) then
                              Dc.Append (Geom.Norm ([Rc (P).U - R (P).U, Rc (P).V - R (P).V, 0.0]));
                           end if;
                        end if;
                     end;
                  end loop;
               end;
               for P of T loop
                  if P.Seen then
                     N_Seen := N_Seen + 1;
                  end if;
               end loop;
               Put_Line (Argument (A) & ":跟点说看得见 " & Codec.Img (N_Seen) & " / " & Codec.Img (Natural (T.Length)) & " · 配点配得回来 " & Codec.Img (N_Ref)
                         & " · 两边都有 " & Codec.Img (Natural (Diffs.Length)) & ",离第一张挪了 中位 " & Codec.Fmt (Q (Disp, 0.5), 1) & " px · 跟点和配点差 中位 "
                         & Codec.Fmt (Q (Diffs, 0.5), 2) & " px、九成 " & Codec.Fmt (Q (Diffs, 0.9), 2) & " px · 粗配和完整配点差 中位 "
                         & Codec.Fmt (Q (Dc, 0.5), 2) & " px、九成 " & Codec.Fmt (Q (Dc, 0.9), 2) & " px");
            end;
         end if;
      end;
   end loop;
   Instrument.Track_End (Host, Port, Id);
end Trackexam;
