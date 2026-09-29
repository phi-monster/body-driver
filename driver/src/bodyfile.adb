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

   procedure Save (Path : String; Key : String; M : Selfmap.Body_Map; Hands : Zone.Hand_Vectors.Vector; Tables : Act.Effect_Vectors.Vector; Sch : Schema.Map) is
      B : Unbounded_String;
      Seen : Ints;
      --  每个数都按 Json.Number 写(写出去再读回来一个比特不差)。不是有限数的(NaN、正负无穷)JSON 里写不了 ⇒ 写成 null,
      --  并把是哪几格照实印出来;读回来是 NaN = 这一格没有一个数,不编成 0。
      --  原来按 Codec.Fmt 定点印:NaN 印成 nan、大过 1e15 的印成 inf —— 都不是 JSON,json.adb 读回来要么整份读不回来,
      --  要么把 nan 当成 null 还吃掉后面的逗号、读错一位不报;比印的那一位还小的量(读数噪声这种)读回来就成了 0
      Bad : Unbounded_String;
      Bad_N : Natural := 0;
      function Num (Where : String; X : Long_Float) return String is
      begin
         if not Json.Finite (X) then
            Bad_N := Bad_N + 1;
            Append (Bad, (if Bad_N > 1 then "、" else "") & Where & " = " & Codec.Fmt (X));
         end if;
         return Json.Number (X);
      end Num;
      procedure Put_Floats (Name : String; V : Floats) is
      begin
         Append (B, """" & Name & """:[");
         for I in 0 .. Natural (V.Length) - 1 loop
            Append (B, (if I > 0 then "," else "") & Num (Name & "[" & Codec.Img (I) & "]", V (I)));
         end loop;
         Append (B, "]");
      end Put_Floats;
      function Pose_Text (Where : String; P : Plug.Arm_Pose) return String is
         R : Unbounded_String;
      begin
         for K in P'Range loop
            Append (R, (if K > P'First then "," else "") & Num (Where & "[" & Codec.Img (K) & "]", P (K)));
         end loop;
         return To_String (R);
      end Pose_Text;
   begin
      Append (B, "{""key"":""" & Json.Escape (Key) & """,""method_ver"":" & Codec.Img (Method_Ver) & ",");
      Append (B, """arms"":" & Codec.Img (M.Arms) & ",""cams"":" & Codec.Img (M.N_Cams) & ",""per_arm"":" & Codec.Img (M.Per_Arm) & ",");
      Append (B, """ee_noise"":" & Num ("ee_noise", M.EE_Noise) & ",""rot_noise"":" & Num ("rot_noise", M.Rot_Noise)
              & ",""jaw_noise"":" & Num ("jaw_noise", M.Jaw_Noise) & ",""settle"":" & Codec.Img (M.Settle) & ",");
      Put_Floats ("amp", M.Amp); Append (B, ",");
      Put_Floats ("delivered", M.Delivered); Append (B, ",");
      for X of M.Seen loop
         Seen.Append (if X then 1 else 0);
      end loop;
      Put_Ints (B, "seen", Seen); Append (B, ",");
      Put_Floats ("cam_frac", M.Cam_Frac); Append (B, ",");
      Put_Ints (B, "cam_on_arm", M.Cam_On_Arm); Append (B, ",");
      --  每条臂量到几个抓握通道(Selfmap.Measure 按身体这一拍报的抓握读数数出来的)。原来不存 ⇒ 装回以后一律当 1 个:
      --  五指手第 1 号往后的握区全丢,而且每次存盘都把少了的那份写回去
      Put_Ints (B, "jaws", M.Jaws); Append (B, ",");
      Append (B, """world_cam"":" & Codec.Img (M.World_Cam) & ",");
      Put_Ints (B, "pic_floor", M.Pic_Floor); Append (B, ",");
      --  历史:每通道历次 amp/delivered(取中位数当现值),最多 History_Depth 次
      Append (B, """amp_hist"":[");
      for Ch in 0 .. M.Channels - 1 loop
         Append (B, (if Ch > 0 then "," else "") & "[");
         if Ch < Natural (M.Amp_Hist.Length) then
            for I in 0 .. Natural (M.Amp_Hist (Ch).Length) - 1 loop
               Append (B, (if I > 0 then "," else "") & Num ("amp_hist[" & Codec.Img (Ch) & "][" & Codec.Img (I) & "]", M.Amp_Hist (Ch) (I)));
            end loop;
         end if;
         Append (B, "]");
      end loop;
      Append (B, "],""deliv_hist"":[");
      for Ch in 0 .. M.Channels - 1 loop
         Append (B, (if Ch > 0 then "," else "") & "[");
         if Ch < Natural (M.Deliv_Hist.Length) then
            for I in 0 .. Natural (M.Deliv_Hist (Ch).Length) - 1 loop
               Append (B, (if I > 0 then "," else "") & Num ("deliv_hist[" & Codec.Img (Ch) & "][" & Codec.Img (I) & "]", M.Deliv_Hist (Ch) (I)));
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
            Hw : constant String := "hands[" & Codec.Img (A) & "].";
            First : Boolean := True;
         begin
            Append (B, (if A > 0 then "," else "") & "{""arm"":" & Codec.Img (H.Arm) & ",""k"":" & Codec.Img (H.K)
                    & ",""empty_close"":" & Num (Hw & "empty_close", H.Empty_Close) & ",""open"":" & Num (Hw & "open", H.Open_Reading)
                    & ",""close_steps"":" & Codec.Img (H.Close_Steps)
                    & ",""pose"":[" & Pose_Text (Hw & "pose", H.Pose) & "],""zones"":[");
            for Cm in 0 .. Natural (H.Zones.Length) - 1 loop
               if H.Zones (Cm).Valid then
                  declare
                     Z : constant Zone.Hand_Zone := H.Zones (Cm);
                     Zw : constant String := Hw & "zones[cam " & Codec.Img (Cm) & "].";
                  begin
                     Append (B, (if First then "" else ",") & "{""cam"":" & Codec.Img (Cm) & ",""cu"":" & Num (Zw & "cu", Z.Cu) & ",""cv"":" & Num (Zw & "cv", Z.Cv) &
                             ",""au"":" & Num (Zw & "au", Z.Au) & ",""av"":" & Num (Zw & "av", Z.Av) & ",""span"":" & Num (Zw & "span", Z.Span) &
                             ",""depth"":" & Num (Zw & "depth", Z.Depth) &
                             ",""n_lobes"":" & Codec.Img (Z.N_Lobes) & ",""box"":[" & Codec.Img (Z.X0) & "," & Codec.Img (Z.Y0) & "," & Codec.Img (Z.X1) & "," & Codec.Img (Z.Y1) & "]" &
                             ",""a"":[" & Codec.Img (Z.A.X0) & "," & Codec.Img (Z.A.Y0) & "," & Codec.Img (Z.A.X1) & "," & Codec.Img (Z.A.Y1) & ","
                             & Num (Zw & "a.cu", Z.A.Cu) & "," & Num (Zw & "a.cv", Z.A.Cv) & "," & Codec.Img (Z.A.Count) & "]" &
                             ",""b"":[" & Codec.Img (Z.B.X0) & "," & Codec.Img (Z.B.Y0) & "," & Codec.Img (Z.B.X1) & "," & Codec.Img (Z.B.Y1) & ","
                             & Num (Zw & "b.cu", Z.B.Cu) & "," & Num (Zw & "b.cv", Z.B.Cv) & "," & Codec.Img (Z.B.Count) & "]" &
                             ",""fingers"":[" & Runs (Z.Fingers) & "]}");
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
            Tw : constant String := "tables[" & Codec.Img (I) & "].";
         begin
            Append (B, (if I > 0 then "," else "") & "{""arm"":" & Codec.Img (T.Arm) & ",""cam"":" & Codec.Img (T.Cam) & ",""kind"":" & Codec.Img (Act.Track_Kind'Pos (T.Kind)) &
                    ",""chan"":" & Codec.Img (T.Chan_K) & ",""blob"":" & Codec.Img (T.Blob) & ",""held"":" & Codec.Img (T.Held) & ",""n"":" & Codec.Img (T.E.N) & ",""b"":[");
            for K in 0 .. T.E.N - 1 loop
               for R in 0 .. Table.Rows - 1 loop
                  Append (B, (if K + R > 0 then "," else "") & Num (Tw & "b[" & Codec.Img (K) & "," & Codec.Img (R) & "]", T.E.B (K, R)));
               end loop;
            end loop;
            Append (B, "],""reps"":[");
            for K in 0 .. T.E.N - 1 loop
               Append (B, (if K > 0 then "," else "") & Codec.Img (T.E.Reps (K)));
            end loop;
            Append (B, "],""scatter"":[");
            for K in 0 .. T.E.N - 1 loop
               for R in 0 .. Table.Rows - 1 loop
                  Append (B, (if K + R > 0 then "," else "") & Num (Tw & "scatter[" & Codec.Img (K) & "," & Codec.Img (R) & "]", T.E.Scatter (K, R)));
               end loop;
            end loop;
            Append (B, "],""trust"":[");
            for K in 0 .. T.E.N - 1 loop
               Append (B, (if K > 0 then "," else "") & (if T.Trust (K) then "1" else "0"));
            end loop;
            Append (B, "],""tpose"":[" & Pose_Text (Tw & "tpose", T.Pose) & "],""has_pose"":" & (if T.Has_Pose then "1" else "0") & ",""reach"":[");
            for K in 0 .. T.E.N - 1 loop
               Append (B, (if K > 0 then "," else "") & Num (Tw & "reach[" & Codec.Img (K) & "]", T.Reach (K)));
            end loop;
            Append (B, "]}");
         end;
      end loop;
      --  身体图:只存真看见过的样本(位姿 + 瓣位置 + 深度)
      Append (B, "],""schema"":[");
      for I in 0 .. Natural (Sch.S.Length) - 1 loop
         declare
            X : constant Schema.Sample := Sch.S (I);
            Sw : constant String := "schema[" & Codec.Img (I) & "].";
         begin
            Append (B, (if I > 0 then "," else "") & "{""arm"":" & Codec.Img (X.Arm) & ",""cam"":" & Codec.Img (X.Cam) & ",""pose"":[" & Pose_Text (Sw & "pose", X.Pose) & "],""parts"":[");
            declare
               First : Boolean := True;
            begin
               for K in Schema.Part_Array'Range loop
                  if X.Parts (K).Valid then
                     declare
                        P : constant Schema.Part_Pos := X.Parts (K);
                        Pw : constant String := Sw & "parts[" & Codec.Img (K) & "].";
                     begin
                        Append (B, (if First then "" else ",") & "[" & Codec.Img (K) & "," & Num (Pw & "cu", P.Cu) & "," & Num (Pw & "cv", P.Cv) & "," & Num (Pw & "z", P.Z) & "," &
                                Codec.Img (P.X0) & "," & Codec.Img (P.Y0) & "," & Codec.Img (P.X1) & "," & Codec.Img (P.Y1) & "," &
                                Codec.Img (P.N_Blobs) & "," & Num (Pw & "b0u", P.B0u) & "," & Num (Pw & "b0v", P.B0v) & "," & Num (Pw & "b1u", P.B1u) & "," & Num (Pw & "b1v", P.B1v) & "]");
                     end;
                     First := False;
                  end if;
               end loop;
            end;
            Append (B, "]}");
         end;
      end loop;
      Append (B, "]}");
      if Bad_N > 0 then
         Ada.Text_IO.Put_Line ("[装] 身体文件里有 " & Codec.Img (Bad_N) & " 格不是有限数,照原样写成 null(装回来还是""没有一个数"",不编成 0):" & To_String (Bad));
      end if;
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
         --  数一律按 Json.Real 读:写的时候不是有限数的那几格是 null,读回来还是 NaN(不编成 0)
         function Val (N : Integer) return Long_Float is (Json.Real (D, N));
         function Num (K : String) return Long_Float is (Val (Json.Get (D, 0, K)));
         function Arr (N : Integer) return Floats is
            V : Floats;
         begin
            for I in 0 .. Json.Count (D, N) - 1 loop
               V.Append (Val (Json.Child (D, N, I)));
            end loop;
            return V;
         end Arr;
         Amp_H : constant Integer := Json.Get (D, 0, "amp_hist");
         Del_H : constant Integer := Json.Get (D, 0, "deliv_hist");
         function Jaw_Note return String is
           (if Jaws_Recorded (M) then "" else ";这份文件没记每条臂几个抓握通道 ⇒ 要重量(不按 1 个猜)");
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
         --  每条臂几个抓握通道:文件里记了(一条臂一个数)就照记的装;没记(09-30 以前的文件)就空着 —— 不猜,开机照实说要重量(Jaws_Recorded)
         M.Jaws.Clear;
         declare
            Jn : constant Integer := Json.Get (D, 0, "jaws");
         begin
            if Json.Count (D, Jn) = M.Arms then
               for X of Arr (Jn) loop
                  M.Jaws.Append (Integer (X));
               end loop;
            end if;
         end;
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
                                         ",现在 " & Codec.Img (Method_Ver) & ")⇒ 握区重量,其余照用" & Jaw_Note);
            return True;
         end if;
         declare
            Hs : constant Integer := Json.Get (D, 0, "hands");
         begin
            for A in 0 .. Json.Count (D, Hs) - 1 loop
               declare
                  Hn : constant Integer := Json.Child (D, Hs, A);
                  H : Zone.Hand;
                  Hk : constant Integer := Json.Get (D, Hn, "k");
                  Ha : constant Integer := Json.Get (D, Hn, "arm");
               begin
                  H.Arm := A;
                  H.Empty_Close := Val (Json.Get (D, Hn, "empty_close"));
                  H.Open_Reading := Val (Json.Get (D, Hn, "open"));
                  --  合一次要几拍:09-30 起才存;以前的文件没有 ⇒ 0(抖手指重认那一处照实说"没量过合一次要几拍",不猜)
                  declare
                     Cs : constant Integer := Json.Get (D, Hn, "close_steps");
                  begin
                     H.Close_Steps := (if Cs >= 0 then Natural (Json.Num (D, Cs)) else 0);
                  end;
                  H.Measured := True;   --  存下来的都是开机量成的
                  for C in 0 .. M.N_Cams - 1 loop
                     H.Zones.Append (Zone.Hand_Zone'(others => <>));
                  end loop;
                  declare
                     Pv : constant Floats := Arr (Json.Get (D, Hn, "pose"));
                  begin
                     if Natural (Pv.Length) = H.Pose'Length then
                        for K in H.Pose'Range loop
                           H.Pose (K) := Pv (K - H.Pose'First);
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
                        Z.Cu := Val (Json.Get (D, Zn, "cu")); Z.Cv := Val (Json.Get (D, Zn, "cv"));
                        Z.Au := Val (Json.Get (D, Zn, "au")); Z.Av := Val (Json.Get (D, Zn, "av"));
                        Z.Span := Val (Json.Get (D, Zn, "span")); Z.Depth := Val (Json.Get (D, Zn, "depth"));
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
                        declare
                           Fr : constant Floats := Arr (Json.Get (D, Zn, "fingers"));   --  游程(见 Runs)
                           Cur : Boolean := False;
                        begin
                           for R of Fr loop
                              for I in 1 .. Natural (Long_Float'Max (0.0, R)) loop
                                 Z.Fingers.Append (Cur);
                              end loop;
                              Cur := not Cur;
                           end loop;
                        end;
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
                  H.Arm := (if Ha >= 0 then Natural (Long_Float'Max (0.0, Json.Num (D, Ha))) else A);
                  H.K := (if Hk >= 0 then Natural (Long_Float'Max (0.0, Json.Num (D, Hk))) else 0);
                  Hands.Append (H);
               end;
            end loop;
         end;
         --  响应表【不跨炮沿用】:它是在某一次跟踪里学出来的,跟错了东西就会把"往哪走会靠近"学反,
         --  存进档案再拿回来用,下一炮会一路朝反方向走(FK/FL 实测,清掉表当场重量之后球才第一次变近)。
         --  重量一遍只要几十拍,不值得冒这个险。身体图、通道幅度、握区照旧沿用。
         --  (原来还有一条 With_Tables 的路把表读回来给离线体检审;那个体检 bodyexam 09-30 随死代码删了,这条路再没人走,一起删)
         Tables.Clear;
         --  🔴 原来这里是 `if True then ... return True; end if;` —— 它把【身体图】也一起跳过了。
         --  注释只说"响应表不沿用",可那一个 return 落在身体图读取【之前】,于是每次开机
         --  都把上一炮攒下来的"位姿 → 我的零件在画面里的位置"整份丢掉(今晚这份档案里有 16 个样本)。
         --  改成只挡响应表:身体图照常装回。
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
                     if Natural (Pv.Length) = X.Pose'Length then
                        for K in X.Pose'Range loop
                           X.Pose (K) := Pv (K - X.Pose'First);
                        end loop;
                        Sch.S.Append (X);
                     end if;
                  end;
               end loop;
            end if;
         end;
         Note := To_Unbounded_String ("装回身体文件(量过 " & Codec.Img (M.Measured_Times) & " 次,身体图 " & Codec.Img (Natural (Sch.S.Length)) & " 个样本)" & Jaw_Note);
      end;
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
