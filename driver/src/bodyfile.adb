with Ada.Text_IO;
with Ada.Directories;
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

   --  ── 写 ──
   procedure Put_Floats (B : in out Unbounded_String; Name : String; V : Floats) is
   begin
      Append (B, """" & Name & """:[");
      for I in 0 .. Natural (V.Length) - 1 loop
         Append (B, (if I > 0 then "," else "") & Codec.Fmt (V (I), 6));
      end loop;
      Append (B, "]");
   end Put_Floats;

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

   procedure Save (Path : String; Key : String; M : Selfmap.Body_Map; Hands : Zone.Hand_Vectors.Vector; Tables : Act.Effect_Vectors.Vector; Sch : Schema.Map) is
      B : Unbounded_String;
      Seen : Ints;
   begin
      Append (B, "{""key"":""" & Json.Escape (Key) & """,");
      Append (B, """arms"":" & Codec.Img (M.Arms) & ",""cams"":" & Codec.Img (M.N_Cams) & ",""per_arm"":" & Codec.Img (M.Per_Arm) & ",");
      Append (B, """ee_noise"":" & Codec.Fmt (M.EE_Noise, 6) & ",""rot_noise"":" & Codec.Fmt (M.Rot_Noise, 6) & ",""jaw_noise"":" & Codec.Fmt (M.Jaw_Noise, 6) & ",""settle"":" & Codec.Img (M.Settle) & ",");
      Put_Floats (B, "amp", M.Amp); Append (B, ",");
      Put_Floats (B, "delivered", M.Delivered); Append (B, ",");
      for X of M.Seen loop
         Seen.Append (if X then 1 else 0);
      end loop;
      Put_Ints (B, "seen", Seen); Append (B, ",");
      Put_Floats (B, "cam_frac", M.Cam_Frac); Append (B, ",");
      Put_Ints (B, "cam_on_arm", M.Cam_On_Arm); Append (B, ",");
      Append (B, """world_cam"":" & Codec.Img (M.World_Cam) & ",");
      Put_Ints (B, "pic_floor", M.Pic_Floor); Append (B, ",");
      --  历史:每通道历次 amp/delivered(取中位数当现值),最多 History_Depth 次
      Append (B, """amp_hist"":[");
      for Ch in 0 .. M.Channels - 1 loop
         Append (B, (if Ch > 0 then "," else "") & "[");
         if Ch < Natural (M.Amp_Hist.Length) then
            for I in 0 .. Natural (M.Amp_Hist (Ch).Length) - 1 loop
               Append (B, (if I > 0 then "," else "") & Codec.Fmt (M.Amp_Hist (Ch) (I), 6));
            end loop;
         end if;
         Append (B, "]");
      end loop;
      Append (B, "],""deliv_hist"":[");
      for Ch in 0 .. M.Channels - 1 loop
         Append (B, (if Ch > 0 then "," else "") & "[");
         if Ch < Natural (M.Deliv_Hist.Length) then
            for I in 0 .. Natural (M.Deliv_Hist (Ch).Length) - 1 loop
               Append (B, (if I > 0 then "," else "") & Codec.Fmt (M.Deliv_Hist (Ch) (I), 6));
            end loop;
         end if;
         Append (B, "]");
      end loop;
      Append (B, "],""measured_times"":" & Codec.Img (M.Measured_Times) & ",");
      --  手:空合读数、张开读数;手上相机里的握区(固定像素,是身体);别的相机里的不存(随位姿变)
      Append (B, """hands"":[");
      for A in 0 .. Natural (Hands.Length) - 1 loop
         declare
            H : constant Zone.Hand := Hands (A);
            Hc : constant Integer := (if A < Natural (M.Cam_On_Arm.Length) then M.Cam_On_Arm (A) else -1);
         begin
            Append (B, (if A > 0 then "," else "") & "{""empty_close"":" & Codec.Fmt (H.Empty_Close, 6) & ",""open"":" & Codec.Fmt (H.Open_Reading, 6));
            if Hc >= 0 and then Natural (Hc) < Natural (H.Zones.Length) and then H.Zones (Natural (Hc)).Valid then
               declare
                  Z : constant Zone.Hand_Zone := H.Zones (Natural (Hc));
               begin
                  Append (B, ",""own_cam"":" & Codec.Img (Natural (Hc)) & ",""zone"":{""cu"":" & Codec.Fmt (Z.Cu, 5) & ",""cv"":" & Codec.Fmt (Z.Cv, 5) &
                          ",""au"":" & Codec.Fmt (Z.Au, 5) & ",""av"":" & Codec.Fmt (Z.Av, 5) & ",""span"":" & Codec.Fmt (Z.Span, 5) & ",""depth"":" & Codec.Fmt (Z.Depth, 5) &
                          ",""n_lobes"":" & Codec.Img (Z.N_Lobes) & ",""box"":[" & Codec.Img (Z.X0) & "," & Codec.Img (Z.Y0) & "," & Codec.Img (Z.X1) & "," & Codec.Img (Z.Y1) & "]" &
                          ",""a"":[" & Codec.Img (Z.A.X0) & "," & Codec.Img (Z.A.Y0) & "," & Codec.Img (Z.A.X1) & "," & Codec.Img (Z.A.Y1) & "," & Codec.Fmt (Z.A.Cu, 5) & "," & Codec.Fmt (Z.A.Cv, 5) & "," & Codec.Img (Z.A.Count) & "]" &
                          ",""b"":[" & Codec.Img (Z.B.X0) & "," & Codec.Img (Z.B.Y0) & "," & Codec.Img (Z.B.X1) & "," & Codec.Img (Z.B.Y1) & "," & Codec.Fmt (Z.B.Cu, 5) & "," & Codec.Fmt (Z.B.Cv, 5) & "," & Codec.Img (Z.B.Count) & "]}");
               end;
            end if;
            Append (B, "}");
         end;
      end loop;
      Append (B, "],""tables"":[");
      for I in 0 .. Natural (Tables.Length) - 1 loop
         declare
            T : constant Act.Stored_Effect := Tables (I);
         begin
            Append (B, (if I > 0 then "," else "") & "{""arm"":" & Codec.Img (T.Arm) & ",""cam"":" & Codec.Img (T.Cam) & ",""kind"":" & Codec.Img (Act.Track_Kind'Pos (T.Kind)) &
                    ",""lobe"":" & Codec.Img (T.Lobe) & ",""n"":" & Codec.Img (T.E.N) & ",""b"":[");
            for K in 0 .. T.E.N - 1 loop
               for R in 0 .. 2 loop
                  Append (B, (if K + R > 0 then "," else "") & Codec.Fmt (T.E.B (K, R), 6));
               end loop;
            end loop;
            Append (B, "],""trust"":[");
            for K in 0 .. T.E.N - 1 loop
               Append (B, (if K > 0 then "," else "") & (if T.Trust (K) then "1" else "0"));
            end loop;
            Append (B, "],""reach"":" & Codec.Fmt (T.Reach, 3) & "}");
         end;
      end loop;
      --  身体图:只存真看见过的样本(位姿 + 瓣位置 + 深度)
      Append (B, "],""schema"":[");
      for I in 0 .. Natural (Sch.S.Length) - 1 loop
         declare
            X : constant Schema.Sample := Sch.S (I);
         begin
            Append (B, (if I > 0 then "," else "") & "{""arm"":" & Codec.Img (X.Arm) & ",""cam"":" & Codec.Img (X.Cam) & ",""pose"":[");
            for K in 0 .. 6 loop
               Append (B, (if K > 0 then "," else "") & Codec.Fmt (X.Pose (K), 6));
            end loop;
            Append (B, "],""n"":" & Codec.Img (X.N_Lobes) & ",""a"":[" & Codec.Fmt (X.Au, 5) & "," & Codec.Fmt (X.Av, 5) & "],""b"":[" & Codec.Fmt (X.Bu, 5) & "," & Codec.Fmt (X.Bv, 5) &
                    "],""c"":[" & Codec.Fmt (X.Cu, 5) & "," & Codec.Fmt (X.Cv, 5) & "],""z"":" & Codec.Fmt (X.Z, 5) & "}");
         end;
      end loop;
      Append (B, "]}");
      declare
         Dir : constant String := Ada.Directories.Containing_Directory (Path);
      begin
         Codec.Make_Dir (Dir);
      exception
         when others => null;
      end;
      Codec.Write_File (Path, From_String (To_String (B)));
   exception
      when others => Ada.Text_IO.Put_Line ("[装] 身体文件写不进 " & Path);
   end Save;

   --  ── 读 ──
   function Load (Path : String; Key : String; M : in out Selfmap.Body_Map; Hands : in out Zone.Hand_Vectors.Vector;
                  Tables : in out Act.Effect_Vectors.Vector; Sch : in out Schema.Map; Note : out Unbounded_String) return Boolean is
      D : Json.Doc;
      Err : Unbounded_String;
      Text : Unbounded_String;
   begin
      Note := Null_Unbounded_String;
      if not Ada.Directories.Exists (Path) then
         Note := To_Unbounded_String ("没有身体文件 " & Path & " ⇒ 从零量");
         return False;
      end if;
      declare
         use Ada.Text_IO;
         F : File_Type;
      begin
         Open (F, In_File, Path);
         while not End_Of_File (F) loop
            Append (Text, Get_Line (F));
         end loop;
         Close (F);
      exception
         when others =>
            Note := To_Unbounded_String ("身体文件读不出来 ⇒ 从零量");
            return False;
      end;
      if not Json.Parse (To_String (Text), D, Err) then
         Note := To_Unbounded_String ("身体文件不是合法 JSON(" & To_String (Err) & ")⇒ 从零量");
         return False;
      end if;
      if Json.Text (D, Json.Get (D, 0, "key")) /= Key then
         Note := To_Unbounded_String ("身体文件的钥匙对不上(这具身体报的形状变了)⇒ 从零量");
         return False;
      end if;
      declare
         function Num (K : String) return Long_Float is (Json.Num (D, Json.Get (D, 0, K)));
         function Arr (N : Integer) return Floats is
            V : Floats;
         begin
            for I in 0 .. Json.Count (D, N) - 1 loop
               V.Append (Json.Num (D, Json.Child (D, N, I)));
            end loop;
            return V;
         end Arr;
         Amp_H : constant Integer := Json.Get (D, 0, "amp_hist");
         Del_H : constant Integer := Json.Get (D, 0, "deliv_hist");
      begin
         M.Arms := Natural (Num ("arms")); M.N_Cams := Natural (Num ("cams")); M.Per_Arm := Natural (Num ("per_arm"));
         M.Channels := M.Arms * M.Per_Arm;
         M.EE_Noise := Num ("ee_noise"); M.Rot_Noise := Num ("rot_noise"); M.Jaw_Noise := Num ("jaw_noise");
         M.Settle := Natural (Num ("settle"));
         M.Amp := Arr (Json.Get (D, 0, "amp"));
         M.Delivered := Arr (Json.Get (D, 0, "delivered"));
         M.Cam_Frac := Arr (Json.Get (D, 0, "cam_frac"));
         M.Seen.Clear;
         for X of Arr (Json.Get (D, 0, "seen")) loop
            M.Seen.Append (X > 0.5);
         end loop;
         M.Cam_On_Arm.Clear;
         for X of Arr (Json.Get (D, 0, "cam_on_arm")) loop
            M.Cam_On_Arm.Append (Integer (X));
         end loop;
         M.World_Cam := Natural (Num ("world_cam"));
         M.Pic_Floor.Clear;
         for X of Arr (Json.Get (D, 0, "pic_floor")) loop
            M.Pic_Floor.Append (Integer (X));
         end loop;
         M.Measured_Times := Natural (Num ("measured_times"));
         M.Amp_Hist.Clear; M.Deliv_Hist.Clear;
         for Ch in 0 .. M.Channels - 1 loop
            M.Amp_Hist.Append (Arr (Json.Child (D, Amp_H, Ch)));
            M.Deliv_Hist.Append (Arr (Json.Child (D, Del_H, Ch)));
         end loop;
         --  历次中位数当现值(LAB 8-18:N 炮合成一份,值取中位数)
         for Ch in 0 .. M.Channels - 1 loop
            if Ch < Natural (M.Amp_Hist.Length) and then not M.Amp_Hist (Ch).Is_Empty then
               M.Amp.Replace_Element (Ch, Median (M.Amp_Hist (Ch)));
               M.Delivered.Replace_Element (Ch, Median (M.Deliv_Hist (Ch)));
            end if;
         end loop;
         if Natural (M.Amp.Length) /= M.Channels or else Natural (M.Cam_On_Arm.Length) /= M.Arms then
            Note := To_Unbounded_String ("身体文件残缺 ⇒ 从零量");
            return False;
         end if;
         --  手
         Hands.Clear;
         declare
            Hs : constant Integer := Json.Get (D, 0, "hands");
         begin
            for A in 0 .. Json.Count (D, Hs) - 1 loop
               declare
                  Hn : constant Integer := Json.Child (D, Hs, A);
                  H : Zone.Hand;
                  Zn : constant Integer := Json.Get (D, Hn, "zone");
               begin
                  H.Arm := A;
                  H.Empty_Close := Json.Num (D, Json.Get (D, Hn, "empty_close"));
                  H.Open_Reading := Json.Num (D, Json.Get (D, Hn, "open"));
                  for C in 0 .. M.N_Cams - 1 loop
                     H.Zones.Append (Zone.Hand_Zone'(others => <>));
                  end loop;
                  if Zn >= 0 then
                     declare
                        Own : constant Natural := Natural (Json.Num (D, Json.Get (D, Hn, "own_cam")));
                        Z : Zone.Hand_Zone;
                        Bx : constant Floats := Arr (Json.Get (D, Zn, "box"));
                        Aa : constant Floats := Arr (Json.Get (D, Zn, "a"));
                        Bb : constant Floats := Arr (Json.Get (D, Zn, "b"));
                     begin
                        Z.Valid := True;
                        Z.Cu := Json.Num (D, Json.Get (D, Zn, "cu")); Z.Cv := Json.Num (D, Json.Get (D, Zn, "cv"));
                        Z.Au := Json.Num (D, Json.Get (D, Zn, "au")); Z.Av := Json.Num (D, Json.Get (D, Zn, "av"));
                        Z.Span := Json.Num (D, Json.Get (D, Zn, "span")); Z.Depth := Json.Num (D, Json.Get (D, Zn, "depth"));
                        Z.N_Lobes := Natural (Json.Num (D, Json.Get (D, Zn, "n_lobes")));
                        if Natural (Bx.Length) = 4 then
                           Z.X0 := Natural (Bx (0)); Z.Y0 := Natural (Bx (1)); Z.X1 := Natural (Bx (2)); Z.Y1 := Natural (Bx (3));
                        end if;
                        if Natural (Aa.Length) = 7 then
                           Z.A := (True, Natural (Aa (0)), Natural (Aa (1)), Natural (Aa (2)), Natural (Aa (3)), Aa (4), Aa (5), Natural (Aa (6)));
                        end if;
                        if Natural (Bb.Length) = 7 and then Z.N_Lobes = 2 then
                           Z.B := (True, Natural (Bb (0)), Natural (Bb (1)), Natural (Bb (2)), Natural (Bb (3)), Bb (4), Bb (5), Natural (Bb (6)));
                        end if;
                        if Own < Natural (H.Zones.Length) then
                           H.Zones.Replace_Element (Own, Z);
                        end if;
                     end;
                  end if;
                  Hands.Append (H);
               end;
            end loop;
         end;
         --  响应表初值
         Tables.Clear;
         declare
            Ts : constant Integer := Json.Get (D, 0, "tables");
         begin
            for I in 0 .. Json.Count (D, Ts) - 1 loop
               declare
                  Tn : constant Integer := Json.Child (D, Ts, I);
                  T : Act.Stored_Effect;
                  Bv : constant Floats := Arr (Json.Get (D, Tn, "b"));
                  Tv : constant Floats := Arr (Json.Get (D, Tn, "trust"));
                  N : constant Natural := Natural (Json.Num (D, Json.Get (D, Tn, "n")));
               begin
                  T.Arm := Natural (Json.Num (D, Json.Get (D, Tn, "arm")));
                  T.Cam := Natural (Json.Num (D, Json.Get (D, Tn, "cam")));
                  T.Kind := Act.Track_Kind'Val (Integer (Json.Num (D, Json.Get (D, Tn, "kind"))));
                  T.Lobe := Integer (Json.Num (D, Json.Get (D, Tn, "lobe")));
                  if Json.Get (D, Tn, "reach") >= 0 then
                     T.Reach := Long_Float'Max (1.0, Json.Num (D, Json.Get (D, Tn, "reach")));
                  end if;
                  Table.Reset (T.E, N, 1.0);
                  for K in 0 .. N - 1 loop
                     if 3 * K + 2 < Natural (Bv.Length) then
                        Table.Set_Col (T.E, K, [Bv (3 * K), Bv (3 * K + 1), Bv (3 * K + 2)]);
                     end if;
                     T.Trust (K) := K < Natural (Tv.Length) and then Tv (K) > 0.5;
                  end loop;
                  Tables.Append (T);
               end;
            end loop;
         end;
         --  身体图(旧文件没有这一节 ⇒ 空)
         Sch.S.Clear;
         declare
            Ss : constant Integer := Json.Get (D, 0, "schema");
         begin
            if Ss >= 0 then
               for I in 0 .. Json.Count (D, Ss) - 1 loop
                  declare
                     Sn : constant Integer := Json.Child (D, Ss, I);
                     X : Schema.Sample;
                     Pv : constant Floats := Arr (Json.Get (D, Sn, "pose"));
                     Av : constant Floats := Arr (Json.Get (D, Sn, "a"));
                     Bv : constant Floats := Arr (Json.Get (D, Sn, "b"));
                     Cv : constant Floats := Arr (Json.Get (D, Sn, "c"));
                  begin
                     X.Arm := Natural (Json.Num (D, Json.Get (D, Sn, "arm")));
                     X.Cam := Natural (Json.Num (D, Json.Get (D, Sn, "cam")));
                     X.N_Lobes := Natural (Json.Num (D, Json.Get (D, Sn, "n")));
                     X.Z := Json.Num (D, Json.Get (D, Sn, "z"));
                     if Natural (Pv.Length) = 7 and then Natural (Av.Length) = 2 and then Natural (Bv.Length) = 2 and then Natural (Cv.Length) = 2 then
                        for K in 0 .. 6 loop
                           X.Pose (K) := Pv (K);
                        end loop;
                        X.Au := Av (0); X.Av := Av (1); X.Bu := Bv (0); X.Bv := Bv (1); X.Cu := Cv (0); X.Cv := Cv (1);
                        Sch.S.Append (X);
                     end if;
                  end;
               end loop;
            end if;
         end;
      end;
      Note := To_Unbounded_String ("装回身体文件(量过 " & Codec.Img (M.Measured_Times) & " 次,身体图 " & Codec.Img (Natural (Sch.S.Length)) & " 个样本)");
      return True;
   exception
      when others =>
         Note := To_Unbounded_String ("身体文件读的时候出错 ⇒ 从零量");
         return False;
   end Load;

   procedure Merge (Stored, Fresh : Selfmap.Body_Map; Merged : out Selfmap.Body_Map; Replaced, Kept : out Natural) is
   begin
      Merged := Fresh;
      Replaced := 0; Kept := 0;
      Merged.Amp_Hist := Stored.Amp_Hist; Merged.Deliv_Hist := Stored.Deliv_Hist;
      while Natural (Merged.Amp_Hist.Length) < Fresh.Channels loop
         Merged.Amp_Hist.Append (F64_Vectors.Empty_Vector);
      end loop;
      while Natural (Merged.Deliv_Hist.Length) < Fresh.Channels loop
         Merged.Deliv_Hist.Append (F64_Vectors.Empty_Vector);
      end loop;
      for Ch in 0 .. Fresh.Channels - 1 loop
         if Fresh.Seen (Ch) then
            declare
               Ha : Floats := Merged.Amp_Hist (Ch);
               Hd : Floats := Merged.Deliv_Hist (Ch);
               Old_Amp : constant Long_Float := (if Ha.Is_Empty then Fresh.Amp (Ch) else Median (Ha));
            begin
               Ha.Append (Fresh.Amp (Ch)); Hd.Append (Fresh.Delivered (Ch));
               while Natural (Ha.Length) > History_Depth loop
                  Ha.Delete_First;
               end loop;
               while Natural (Hd.Length) > History_Depth loop
                  Hd.Delete_First;
               end loop;
               Merged.Amp_Hist.Replace_Element (Ch, Ha);
               Merged.Deliv_Hist.Replace_Element (Ch, Hd);
               Merged.Amp.Replace_Element (Ch, Median (Ha));
               Merged.Delivered.Replace_Element (Ch, Median (Hd));
               if abs (Merged.Amp (Ch) - Old_Amp) > 0.0 then
                  Replaced := Replaced + 1;
               else
                  Kept := Kept + 1;
               end if;
            end;
         end if;
      end loop;
      --  噪声地板只放大不缩小(LAB 8-18:跨炮一致不代表 σ 能变小)
      Merged.EE_Noise := Long_Float'Max (Stored.EE_Noise, Fresh.EE_Noise);
      Merged.Rot_Noise := Long_Float'Max (Stored.Rot_Noise, Fresh.Rot_Noise);
      Merged.Jaw_Noise := Long_Float'Max (Stored.Jaw_Noise, Fresh.Jaw_Noise);
      Merged.Measured_Times := Stored.Measured_Times + 1;
   end Merge;
end Bodyfile;
