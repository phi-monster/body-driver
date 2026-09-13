with Ada.Text_IO;
with Ada.Directories;
with Chan;
with Bytes; use Bytes;
with Codec;
with Json; use type Json.Kind;
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
      Append (B, "{""key"":""" & Json.Escape (Key) & """,""method_ver"":" & Codec.Img (Method_Ver) & ",");
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
      --  手:空合读数、张开读数、合空时的位姿;每台相机里的握区都存(手上相机里的是固定像素;别的相机里的只在同一位姿下成立 —— 开机位姿一样就不用再合空)
      Append (B, """hands"":[");
      for A in 0 .. Natural (Hands.Length) - 1 loop
         declare
            H : constant Zone.Hand := Hands (A);
            First : Boolean := True;
         begin
            Append (B, (if A > 0 then "," else "") & "{""empty_close"":" & Codec.Fmt (H.Empty_Close, 6) & ",""open"":" & Codec.Fmt (H.Open_Reading, 6) & ",""pose"":[");
            for K in 0 .. 6 loop
               Append (B, (if K > 0 then "," else "") & Codec.Fmt (H.Pose (K), 6));
            end loop;
            Append (B, "],""zones"":[");
            for Cm in 0 .. Natural (H.Zones.Length) - 1 loop
               if H.Zones (Cm).Valid then
                  declare
                     Z : constant Zone.Hand_Zone := H.Zones (Cm);
                  begin
                     Append (B, (if First then "" else ",") & "{""cam"":" & Codec.Img (Cm) & ",""cu"":" & Codec.Fmt (Z.Cu, 5) & ",""cv"":" & Codec.Fmt (Z.Cv, 5) &
                             ",""au"":" & Codec.Fmt (Z.Au, 5) & ",""av"":" & Codec.Fmt (Z.Av, 5) & ",""span"":" & Codec.Fmt (Z.Span, 5) & ",""depth"":" & Codec.Fmt (Z.Depth, 5) &
                             ",""n_lobes"":" & Codec.Img (Z.N_Lobes) & ",""box"":[" & Codec.Img (Z.X0) & "," & Codec.Img (Z.Y0) & "," & Codec.Img (Z.X1) & "," & Codec.Img (Z.Y1) & "]" &
                             ",""a"":[" & Codec.Img (Z.A.X0) & "," & Codec.Img (Z.A.Y0) & "," & Codec.Img (Z.A.X1) & "," & Codec.Img (Z.A.Y1) & "," & Codec.Fmt (Z.A.Cu, 5) & "," & Codec.Fmt (Z.A.Cv, 5) & "," & Codec.Img (Z.A.Count) & "]" &
                             ",""b"":[" & Codec.Img (Z.B.X0) & "," & Codec.Img (Z.B.Y0) & "," & Codec.Img (Z.B.X1) & "," & Codec.Img (Z.B.Y1) & "," & Codec.Fmt (Z.B.Cu, 5) & "," & Codec.Fmt (Z.B.Cv, 5) & "," & Codec.Img (Z.B.Count) & "]}");
                     First := False;
                  end;
               end if;
            end loop;
            Append (B, "]}");
         end;
      end loop;
      Append (B, "],""tables"":[");
      for I in 0 .. Natural (Tables.Length) - 1 loop
         declare
            T : constant Act.Stored_Effect := Tables (I);
         begin
            Append (B, (if I > 0 then "," else "") & "{""arm"":" & Codec.Img (T.Arm) & ",""cam"":" & Codec.Img (T.Cam) & ",""kind"":" & Codec.Img (Act.Track_Kind'Pos (T.Kind)) &
                    ",""chan"":" & Codec.Img (T.Chan_K) & ",""blob"":" & Codec.Img (T.Blob) & ",""held"":" & Codec.Img (T.Held) & ",""n"":" & Codec.Img (T.E.N) & ",""b"":[");
            for K in 0 .. T.E.N - 1 loop
               for R in 0 .. Table.Rows - 1 loop
                  Append (B, (if K + R > 0 then "," else "") & Codec.Fmt (T.E.B (K, R), 6));
               end loop;
            end loop;
            Append (B, "],""trust"":[");
            for K in 0 .. T.E.N - 1 loop
               Append (B, (if K > 0 then "," else "") & (if T.Trust (K) then "1" else "0"));
            end loop;
            Append (B, "],""tpose"":[");
            for K in 0 .. 6 loop
               Append (B, (if K > 0 then "," else "") & Codec.Fmt (T.Pose (K), 6));
            end loop;
            Append (B, "],""has_pose"":" & (if T.Has_Pose then "1" else "0") & ",""reach"":[");
            for K in 0 .. T.E.N - 1 loop
               Append (B, (if K > 0 then "," else "") & Codec.Fmt (T.Reach (K), 3));
            end loop;
            Append (B, "]}");
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
            Append (B, "],""parts"":[");
            declare
               First : Boolean := True;
            begin
               for K in Schema.Part_Array'Range loop
                  if X.Parts (K).Valid then
                     Append (B, (if First then "" else ",") & "[" & Codec.Img (K) & "," & Codec.Fmt (X.Parts (K).Cu, 5) & "," & Codec.Fmt (X.Parts (K).Cv, 5) & "," & Codec.Fmt (X.Parts (K).Z, 5) & "," &
                             Codec.Img (X.Parts (K).X0) & "," & Codec.Img (X.Parts (K).Y0) & "," & Codec.Img (X.Parts (K).X1) & "," & Codec.Img (X.Parts (K).Y1) & "," &
                             Codec.Img (X.Parts (K).N_Blobs) & "," & Codec.Fmt (X.Parts (K).B0u, 5) & "," & Codec.Fmt (X.Parts (K).B0v, 5) & "," & Codec.Fmt (X.Parts (K).B1u, 5) & "," & Codec.Fmt (X.Parts (K).B1v, 5) & "]");
                     First := False;
                  end if;
               end loop;
            end;
            Append (B, "]}");
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
                  Tables : in out Act.Effect_Vectors.Vector; Sch : in out Schema.Map; Note : out Unbounded_String;
                  With_Tables : Boolean := False) return Boolean is
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
         --  手:量法版本对不上 ⇒ 握区不装回(重新合空量一次;通道幅度那些不受影响)
         Hands.Clear;
         if Integer (Json.Num (D, Json.Get (D, 0, "method_ver"))) /= Method_Ver then
            Note := To_Unbounded_String ("身体文件是老量法(存的版本 " & Codec.Img (Integer (Json.Num (D, Json.Get (D, 0, "method_ver")))) &
                                         ",现在 " & Codec.Img (Method_Ver) & ")⇒ 握区重量,其余照用");
            return True;
         end if;
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
                  declare
                     Pv : constant Floats := Arr (Json.Get (D, Hn, "pose"));
                  begin
                     if Natural (Pv.Length) = 7 then
                        for K in 0 .. 6 loop
                           H.Pose (K) := Pv (K);
                        end loop;
                     end if;
                  end;
                  declare
                     procedure Read_Zone (Zn : Integer; Cm : Natural) is
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
                        if Cm < Natural (H.Zones.Length) then
                           H.Zones.Replace_Element (Cm, Z);
                        end if;
                     end Read_Zone;
                     Zs : constant Integer := Json.Get (D, Hn, "zones");
                     Zn : constant Integer := Json.Get (D, Hn, "zone");
                  begin
                     if Zs >= 0 then
                        for J in 0 .. Json.Count (D, Zs) - 1 loop
                           declare
                              Zj : constant Integer := Json.Child (D, Zs, J);
                           begin
                              Read_Zone (Zj, Natural (Long_Float'Max (0.0, Json.Num (D, Json.Get (D, Zj, "cam")))));
                           end;
                        end loop;
                     elsif Zn >= 0 then
                        Read_Zone (Zn, Natural (Json.Num (D, Json.Get (D, Hn, "own_cam"))));
                     end if;
                  end;
                  Hands.Append (H);
               end;
            end loop;
         end;
         --  响应表【不跨炮沿用】:它是在某一次跟踪里学出来的,跟错了东西就会把"往哪走会靠近"学反,
         --  存进档案再拿回来用,下一炮会一路朝反方向走(FK/FL 实测,清掉表当场重量之后球才第一次变近)。
         --  重量一遍只要几十拍,不值得冒这个险。身体图、通道幅度、握区照旧沿用。
         Tables.Clear;
         --  🔴 原来这里是 `if True then ... return True; end if;` —— 它把【身体图】也一起跳过了。
         --  注释只说"响应表不沿用",可那一个 return 落在身体图读取【之前】,于是每次开机
         --  都把上一炮攒下来的"位姿 → 我的零件在画面里的位置"整份丢掉(今晚这份档案里有 16 个样本)。
         --  改成只挡响应表:身体图照常装回。
         if With_Tables then
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
                  T.Chan_K := Natural (Long_Float'Max (0.0, Json.Num (D, Json.Get (D, Tn, "chan"))));
                  T.Blob := Integer (Json.Num (D, Json.Get (D, Tn, "blob")));
                  T.Held := (if Json.Get (D, Tn, "held") >= 0 then Integer (Json.Num (D, Json.Get (D, Tn, "held"))) else -1);
                  declare
                     Tp : constant Floats := Arr (Json.Get (D, Tn, "tpose"));
                  begin
                     if Natural (Tp.Length) = 7 then
                        for K in 0 .. 6 loop
                           T.Pose (K) := Tp (K);
                        end loop;
                        T.Has_Pose := Json.Num (D, Json.Get (D, Tn, "has_pose")) > 0.5;
                     end if;
                  end;
                  declare
                     Rn : constant Integer := Json.Get (D, Tn, "reach");
                  begin
                     if Rn >= 0 and then Json.Kind_Of (D, Rn) = Json.J_Arr then
                        declare
                           Rv : constant Floats := Arr (Rn);
                        begin
                           for K in 0 .. Natural'Min (N, Natural (Rv.Length)) - 1 loop
                              T.Reach (K) := Long_Float'Max (1.0, Rv (K));
                           end loop;
                        end;
                     elsif Rn >= 0 then
                        T.Reach := [others => Long_Float'Max (1.0, Json.Num (D, Rn))];
                     end if;
                  end;
                  Table.Reset (T.E, N, 1.0);
                  for K in 0 .. N - 1 loop
                     if Table.Rows * K + Table.Rows - 1 < Natural (Bv.Length) then
                        declare
                           Cl : Table.Vec3;
                        begin
                           for R in 0 .. Table.Rows - 1 loop
                              Cl (R) := Bv (Table.Rows * K + R);
                           end loop;
                           Table.Set_Col (T.E, K, Cl);
                        end;
                     end if;
                     T.Trust (K) := K < Natural (Tv.Length) and then Tv (K) > 0.5;
                  end loop;
                  Tables.Append (T);
               end;
            end loop;
         end;
         end if;
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
                  begin
                     X.Arm := Natural (Json.Num (D, Json.Get (D, Sn, "arm")));
                     X.Cam := Natural (Json.Num (D, Json.Get (D, Sn, "cam")));
                     declare
                        Ps : constant Integer := Json.Get (D, Sn, "parts");
                     begin
                        if Ps >= 0 then
                           for J in 0 .. Json.Count (D, Ps) - 1 loop
                              declare
                                 Pv2 : constant Floats := Arr (Json.Child (D, Ps, J));
                              begin
                                 if Natural (Pv2.Length) = 13 and then Pv2 (0) >= 0.0 and then Integer (Pv2 (0)) <= Chan.Per_Arm then
                                    X.Parts (Integer (Pv2 (0))) := (True, Pv2 (1), Pv2 (2), Pv2 (3), Natural (Pv2 (4)), Natural (Pv2 (5)), Natural (Pv2 (6)), Natural (Pv2 (7)),
                                                                    Natural (Pv2 (8)), Pv2 (9), Pv2 (10), Pv2 (11), Pv2 (12));
                                 end if;
                              end;
                           end loop;
                        end if;
                     end;
                     if Natural (Pv.Length) = 7 then
                        for K in 0 .. 6 loop
                           X.Pose (K) := Pv (K);
                        end loop;
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
