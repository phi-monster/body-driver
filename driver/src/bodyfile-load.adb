separate (Bodyfile)
function Load (Path : String; Key : String; M : in out Selfmap.Body_Map; Hands : in out Zone.Hand_Vectors.Vector;
               Tables : in out Act.Effect_Vectors.Vector; Sch : in out Schema.Map; Note : out Unbounded_String) return Boolean is
   D : Json.Doc;
   Err : Unbounded_String;
   Text : Unbounded_String;
   Noise_Note : Unbounded_String;   --  静止噪声不信时开机报告那一句
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
      --  静止噪声是第几版量法量的(没记 = 更老):对不上 ⇒ 不信,记成负数(开机重量:Merge 只用这一回量的,照用存的那条路先换成这一回量的)
      declare
         Nv : constant Integer := Json.Get (D, 0, "noise_ver");
         Ver : constant Integer := (if Nv >= 0 then Integer (Json.Num (D, Nv)) else 0);
      begin
         if Ver /= Selfmap.Idle_Ver then
            M.EE_Noise := -1.0; M.Rot_Noise := -1.0; M.Jaw_Noise := -1.0;
            Noise_Note := To_Unbounded_String (";静止噪声是老量法量的(第 " & Codec.Img (Ver) & " 版,现在第 " & Codec.Img (Selfmap.Idle_Ver)
                                               & " 版)⇒ 不信,开机重量");
         end if;
      end;
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
                                      ",现在 " & Codec.Img (Method_Ver) & ")⇒ 握区重量,其余照用" & Jaw_Note) & Noise_Note;
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
                  begin
                     Z.Valid := True;
                     Z.Cu := Val (Json.Get (D, Zn, "cu")); Z.Cv := Val (Json.Get (D, Zn, "cv"));
                     Z.Au := Val (Json.Get (D, Zn, "au")); Z.Av := Val (Json.Get (D, Zn, "av"));
                     Z.Span := Val (Json.Get (D, Zn, "span")); Z.Depth := Val (Json.Get (D, Zn, "depth"));
                     if Natural (Bx.Length) = 4 then
                        Z.X0 := Natural (Bx (0)); Z.Y0 := Natural (Bx (1)); Z.X1 := Natural (Bx (2)); Z.Y1 := Natural (Bx (3));
                     end if;
                     Zone.Lobes_From_Json (D, Zn, Z);   --  每一瓣(I2:"lobes";I2 以前的文件按 "n_lobes" 从 "a" / "b" 里取)
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
      Note := To_Unbounded_String ("装回身体文件(量过 " & Codec.Img (M.Measured_Times) & " 次,身体图 " & Codec.Img (Natural (Sch.S.Length)) & " 个样本)" & Jaw_Note)
              & Noise_Note;
   end;
   return True;
exception
   when others =>
      Note := To_Unbounded_String ("身体文件读的时候出错 ⇒ 从零量");
      return False;
end Load;
