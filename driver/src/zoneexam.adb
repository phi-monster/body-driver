--  离线看握区(2026-09-24):拿张开 / 合上两张灰度图(PGM,P5)跑驱动同一段 Zone.From_Frames,打出每一瓣的指尖像素、大小、框,和几瓣平均的指尖。
--  用来离线试"手上哪个点"的认法(G2C:同一只手换个姿势,头顶眼里的瓣数在 1 和 2 之间跳),不用再开一炮。
--  用法:zoneexam open.pgm closed.pgm [turned.pgm 仪器主机 仪器端口 焦距 主点x 主点y [out_prefix]]
--  给了转出去以后那一帧(第二张图 = 转之前停住的那一头)、配点仪器和这只眼的焦距 / 主点:照驱动判"哪头张开"那一段(Zone.Measure)
--  问那张格点配到哪,Kinem.Fit_Eye_Turn + Classify_Rides 判长在眼上 / 世界 / 分不开,数瓣里 / 合到的区里各有几个长在眼上、按两个比例之差判哪头张开
--  (灰度图按三个通道一样当彩图发;驱动发的是彩图)。给了 out_prefix 还写出手指、瓣两张掩码(PGM)。
--  09-30 V1B69 第 1 只手按灰度判不出哪头张开:手一转光照就变,换成按配点判
with Ada.Command_Line;
with Ada.Containers;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Streams.Stream_IO;
with Bytes; use Bytes;
with Zone;
with Codec;
with Kinem;
with Geom;
with Instrument;
with Stats;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
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
            U, V, Wd, Th : Long_Float;
            Ok : Boolean;
         begin
            Zone.Tip_Section (Z, Lb, W, H, U, V, Wd, Th, Ok);   --  驱动认指尖的那一条定义(09-30:Tip_Px 没人调了、删了)
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
      if Ada.Command_Line.Argument_Count >= 8 then
         declare
            use type Kinem.Ride;
            Turned : Buf;
            W3, H3 : Natural;
            In_Lobe : constant Bools := Zone.Lobe_Pixels (Z, W, H);
            Eye : Geom.Cam_Geo := Geom.No_Geo;
            function Rgb (G : Buf) return Buf is
               R : Buf;
            begin
               for P of G loop
                  R.Append (P); R.Append (P); R.Append (P);
               end loop;
               return R;
            end Rgb;
            Q : Instrument.Match_Vectors.Vector;
            Err : Unbounded_String;
            Ride_L, Ride_A, N_L, N_A, N_Unk : Natural := 0;
         begin
            Read_PGM (Ada.Command_Line.Argument (3), Turned, W3, H3);
            if W3 /= W or else H3 /= H then
               Put_Line ("转出去那一帧大小不一样");
               return;
            end if;
            Eye.F := Long_Float'Value (Ada.Command_Line.Argument (6));
            Eye.Cx := Long_Float'Value (Ada.Command_Line.Argument (7));
            Eye.Cy := Long_Float'Value (Ada.Command_Line.Argument (8));
            Eye.Valid := True;
            for Gyy in 0 .. Kinem.Gy - 1 loop
               for Gxx in 0 .. Kinem.Gx - 1 loop
                  Q.Append (Instrument.Match_Pt'(U => Kinem.Grid_U (Gxx, W), V => Kinem.Grid_V (Gyy, H), others => <>));
               end loop;
            end loop;
            declare
               Mt : constant Instrument.Match_Vectors.Vector :=
                 Instrument.Match (Ada.Command_Line.Argument (4), Natural'Value (Ada.Command_Line.Argument (5)), Rgb (Closed_G), W, H, Rgb (Turned), W, H, Q, Err,
                                   Coarse => True);
               Nv : Natural := 0;
            begin
               if Natural (Mt.Length) /= Natural (Q.Length) then
                  Put_Line ("配点仪器没配成:" & To_String (Err));
                  return;
               end if;
               for G of Mt loop
                  if G.U >= 0.0 and then G.V >= 0.0 then
                     Nv := Nv + 1;
                  end if;
               end loop;
               declare
                  Pu, Pv, Bu, Bv : Kinem.Vec (0 .. Nv - 1);
                  Rd : Kinem.Ride_Vec (0 .. Nv - 1);
                  J : Natural := 0;
                  Sig : Long_Float;
                  Settled : Boolean;
               begin
                  for G in 0 .. Natural (Q.Length) - 1 loop
                     if Mt (G).U >= 0.0 and then Mt (G).V >= 0.0 then
                        Pu (J) := Q (G).U; Pv (J) := Q (G).V; Bu (J) := Mt (G).U; Bv (J) := Mt (G).V;
                        J := J + 1;
                     end if;
                  end loop;
                  declare
                     Rot : Geom.V3;
                     Fitted : Boolean;
                  begin
                     Kinem.Fit_Eye_Turn (Eye, Pu, Pv, Bu, Bv, Rot, Sig, Settled, Fitted);
                     if Fitted then
                        Kinem.Classify_Rides (Eye, Rot, Sig, Pu, Pv, Bu, Bv, Rd);
                        --  驱动同一段:瓣按长在眼上补全(张开的就是第二张图那一头时),打出补之前 / 之后每一瓣的尖
                        declare
                           Gr : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Kinem.Gx * Kinem.Gy));
                           Jg : Natural := 0;
                           Z2 : Zone.Hand_Zone := Z;
                           Ps : Zone.Probe_Vectors.Vector;
                           Added : Natural := 0;
                        begin
                           for G in 0 .. Natural (Q.Length) - 1 loop
                              if Mt (G).U >= 0.0 and then Mt (G).V >= 0.0 then
                                 Gr.Replace_Element (G, Rd (Jg) = Kinem.Rides);
                                 Jg := Jg + 1;
                              end if;
                           end loop;
                           Ps := Zone.Refine_Probes (Z2, W, H, Gr);
                           if not Ps.Is_Empty then
                              declare
                                 Q2 : Instrument.Match_Vectors.Vector;
                                 Err2 : Unbounded_String;
                                 Mu, Mv : Floats;
                              begin
                                 for P of Ps loop
                                    Q2.Append (Instrument.Match_Pt'(U => P.U, V => P.V, others => <>));
                                 end loop;
                                 declare
                                    M2 : constant Instrument.Match_Vectors.Vector :=
                                      Instrument.Match (Ada.Command_Line.Argument (4), Natural'Value (Ada.Command_Line.Argument (5)), Rgb (Closed_G), W, H, Rgb (Turned), W, H, Q2, Err2,
                                                        Coarse => False);
                                 begin
                                    if Natural (M2.Length) = Natural (Q2.Length) then
                                       for G of M2 loop
                                          Mu.Append (G.U); Mv.Append (G.V);
                                       end loop;
                                       Zone.Apply_Refine (Z2, W, H, Ps, Mu, Mv, Eye, Rot, Sig, Added);
                                    else
                                       Put_Line ("补全:配点仪器没配成 " & To_String (Err2));
                                    end if;
                                 end;
                              end;
                           end if;
                           Put ("补全:问 " & Codec.Img (Natural (Ps.Length)) & " 个像素、并进 " & Codec.Img (Added) & " 个");
                           for Kl in 0 .. Z2.N_Lobes - 1 loop
                              declare
                                 U0, V0, U1, V1, Wd, Th : Long_Float;
                                 O0, O1 : Boolean;
                              begin
                                 Zone.Tip_Section (Z, Zone.Lobe_Of (Z, Kl), W, H, U0, V0, Wd, Th, O0);
                                 Zone.Tip_Section (Z2, Zone.Lobe_Of (Z2, Kl), W, H, U1, V1, Wd, Th, O1);
                                 Put (" | 瓣 " & Codec.Img (Kl) & " 尖 " & (if O0 then Codec.Fmt (U0, 1) & " " & Codec.Fmt (V0, 1) else "-") & " → "
                                      & (if O1 then Codec.Fmt (U1, 1) & " " & Codec.Fmt (V1, 1) else "-"));
                              end;
                           end loop;
                           New_Line;
                        end;
                     end if;
                  end;
                  for I in Rd'Range loop
                     declare
                        Px : constant Natural := Natural (Long_Float'Floor (Pv (I))) * W + Natural (Long_Float'Floor (Pu (I)));
                     begin
                        if Z.Fingers.Element (Px) then
                           if Rd (I) = Kinem.Unknown then
                              N_Unk := N_Unk + 1;
                           elsif In_Lobe.Element (Px) then
                              N_L := N_L + 1;
                              if Rd (I) = Kinem.Rides then
                                 Ride_L := Ride_L + 1;
                              end if;
                           else
                              N_A := N_A + 1;
                              if Rd (I) = Kinem.Rides then
                                 Ride_A := Ride_A + 1;
                              end if;
                           end if;
                        end if;
                     end;
                  end loop;
                  Put ("配点噪声 " & Codec.Fmt (Sig, 2) & " px" & (if Settled then "" else "(没收住)") & " · 长在眼上的:瓣里 " & Codec.Img (Ride_L) & " / " & Codec.Img (N_L)
                       & "、合到的区里 " & Codec.Img (Ride_A) & " / " & Codec.Img (N_A) & "(分不开 " & Codec.Img (N_Unk) & ")");
                  if N_L > 0 and then N_A > 0 then
                     declare
                        Pl : constant Long_Float := Long_Float (Ride_L) / Long_Float (N_L);
                        Pa : constant Long_Float := Long_Float (Ride_A) / Long_Float (N_A);
                        P : constant Long_Float := Long_Float (Ride_L + Ride_A) / Long_Float (N_L + N_A);
                        Sd : constant Long_Float := Sqrt (P * (1.0 - P) * Long_Float (N_L + N_A) / (Long_Float (N_L) * Long_Float (N_A)));
                     begin
                        Put_Line (if abs (Pl - Pa) > Stats.Z * Sd then (if Pl > Pa then " ⇒ 转之前停的那一头张开" else " ⇒ 另一头张开")
                                  else " ⇒ 看不出(之差 " & Codec.Fmt (abs (Pl - Pa), 2) & " 不过 " & Codec.Fmt (Stats.Z * Sd, 2) & ")");
                     end;
                  else
                     Put_Line (" ⇒ 看不出(有一类没有判得了的格点)");
                  end if;
               end;
            end;
            if Ada.Command_Line.Argument_Count >= 9 then
               declare
                  Mk_F, Mk_L : Buf := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (W * H));
               begin
                  for I in 0 .. W * H - 1 loop
                     if Z.Fingers.Element (I) then
                        Mk_F.Replace_Element (I, 255);
                        if In_Lobe.Element (I) then
                           Mk_L.Replace_Element (I, 255);
                        end if;
                     end if;
                  end loop;
                  Codec.Write_PGM (Ada.Command_Line.Argument (9) & "_fingers.pgm", Mk_F, W, H);
                  Codec.Write_PGM (Ada.Command_Line.Argument (9) & "_lobes.pgm", Mk_L, W, H);
               end;
            end if;
         end;
      end if;
   end;
end Zoneexam;
