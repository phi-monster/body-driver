separate (Bodyfile)
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
   --  静止噪声记着是第几版量法量的(Selfmap.Idle_Ver):装回时版本对不上就不信、开机重量
   Append (B, """noise_ver"":" & Codec.Img (Selfmap.Idle_Ver) & ",");
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
                          ",""a"":[" & Codec.Img (Zone.Lobe_Of (Z, 0).X0) & "," & Codec.Img (Zone.Lobe_Of (Z, 0).Y0) & "," & Codec.Img (Zone.Lobe_Of (Z, 0).X1) & "," & Codec.Img (Zone.Lobe_Of (Z, 0).Y1) & ","
                          & Num (Zw & "a.cu", Zone.Lobe_Of (Z, 0).Cu) & "," & Num (Zw & "a.cv", Zone.Lobe_Of (Z, 0).Cv) & "," & Codec.Img (Zone.Lobe_Of (Z, 0).Count) & "]" &
                          ",""b"":[" & Codec.Img (Zone.Lobe_Of (Z, 1).X0) & "," & Codec.Img (Zone.Lobe_Of (Z, 1).Y0) & "," & Codec.Img (Zone.Lobe_Of (Z, 1).X1) & "," & Codec.Img (Zone.Lobe_Of (Z, 1).Y1) & ","
                          & Num (Zw & "b.cu", Zone.Lobe_Of (Z, 1).Cu) & "," & Num (Zw & "b.cv", Zone.Lobe_Of (Z, 1).Cv) & "," & Codec.Img (Zone.Lobe_Of (Z, 1).Count) & "]" &
                          ",""lobes"":" & Zone.Lobes_Json (Z) &   --  每一瓣(I2);上面 a / b 两格照旧写,旧的读法还读得了
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
