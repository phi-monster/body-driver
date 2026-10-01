with Selfmap.Graph;
separate (Selfcheck)
procedure Welds_Path_1 is
   --  路 1 的焊点(大并行.md §5 路 1):每条写清"错了会是什么病",带一颗牙(去掉那一改就红)
   function Vec (A : Bytes.Int_Vectors.Vector) return Ints is (A);
   function Same (A, B : Ints) return Boolean is
     (Natural (A.Length) = Natural (B.Length) and then (for all I in 0 .. Natural (A.Length) - 1 => A (I) = B (I)));
   function Show (V : Ints) return String is
      R : Unbounded_String;
   begin
      for I in 0 .. Natural (V.Length) - 1 loop
         Append (R, (if I > 0 then "," else "") & Codec.Img (V (I)));
      end loop;
      return "[" & To_String (R) & "]";
   end Show;
   --  一张只填了身体图那几个旧字段的 Body_Map(别的字段不影响这几问)
   function Map_Of (Arms, Cams : Natural; Jaws, On_Arm : Ints) return Selfmap.Body_Map is
      M : Selfmap.Body_Map;
   begin
      M.Arms := Arms; M.N_Cams := Cams; M.Per_Arm := Chan.Per_Arm; M.Channels := Arms * Chan.Per_Arm;
      M.Jaws := Jaws; M.Cam_On_Arm := On_Arm;
      return M;
   end Map_Of;
   Empty : constant Ints := Int_Vectors.Empty_Vector;
begin
   --  ── I1 身体图的通用问法(Selfmap.Graph):别路只问这几句,不按下标算通道号、不认"一条臂一只眼 / 至少一个合拢通道 / 只有一只不动的眼" ──
   --  ① x5 的样子(V1B78 身体文件里的字段:两条臂、三台相机、每条臂一个合拢通道、第 1 / 2 台是两只腕眼)
   --     ⇒ 答得和今天的字段一模一样,开机报告那一行逐字对。
   --  病:这几问和字段答得不一样 ⇒ 别路照着写的新代码在 x5 上悄悄换了通道 / 眼。
   --  牙:开机报告那一行照旧读字段(Integer'Image (Cam_On_Arm))⇒ 这一行对不上
   declare
      M : constant Selfmap.Body_Map := Map_Of (2, 3, Vec ([1, 1]), Vec ([1, 2]));
      Want_Say : constant String :=
        "身体图(通用问法):2 条臂 · 第 1 条臂:位姿通道 0,1,2,3,4,5、合拢通道 1 个、长在它上面的眼(相机号)1"
        & " · 第 2 条臂:位姿通道 6,7,8,9,10,11、合拢通道 1 个、长在它上面的眼(相机号)2"
        & " · 不长在任何臂上的眼(相机号)0 · 扛着全身走的组 没有";
      Ok_Arms : constant Boolean := Selfmap.Graph.Arm_Count (M) = 2;
      Ok_Pose : constant Boolean := Same (Selfmap.Graph.Pose_Channels (M, 0), Vec ([0, 1, 2, 3, 4, 5]))
                                    and then Same (Selfmap.Graph.Pose_Channels (M, 1), Vec ([6, 7, 8, 9, 10, 11]));
      Ok_Close : constant Boolean := Selfmap.Graph.Closing_Count (M, 0) = 1 and then Selfmap.Graph.Closing_Count (M, 1) = 1;
      Ok_Eyes : constant Boolean := Same (Selfmap.Graph.Eyes_On (M, 0), Vec ([1])) and then Same (Selfmap.Graph.Eyes_On (M, 1), Vec ([2]))
                                    and then Same (Selfmap.Graph.Eyes_Off_Arms (M), Vec ([0]));
      Ok_Carry : constant Boolean := Selfmap.Graph.Carrying_Groups (M).Is_Empty;
      Ok_Say : constant Boolean := Selfmap.Graph.Say (M) = Want_Say;
   begin
      Check (Ok_Arms and then Ok_Pose and then Ok_Close and then Ok_Eyes and then Ok_Carry and then Ok_Say,
             "身体图通用问法 · x5 的样子(两条臂、三台相机、各一只腕眼、各一个合拢通道):几条臂 " & Boolean'Image (Ok_Arms)
             & " · 位姿通道 " & Show (Selfmap.Graph.Pose_Channels (M, 0)) & Show (Selfmap.Graph.Pose_Channels (M, 1))
             & " · 合拢通道 " & Codec.Img (Selfmap.Graph.Closing_Count (M, 0)) & "/" & Codec.Img (Selfmap.Graph.Closing_Count (M, 1))
             & " · 长在臂上的眼 " & Show (Selfmap.Graph.Eyes_On (M, 0)) & Show (Selfmap.Graph.Eyes_On (M, 1))
             & " · 不长在臂上的眼 " & Show (Selfmap.Graph.Eyes_Off_Arms (M))
             & " · 扛着全身的组 " & Show (Selfmap.Graph.Carrying_Groups (M)) & " · 开机报告一行逐字对 " & Boolean'Image (Ok_Say)
             & (if Ok_Say then "" else "(念出来的:" & Selfmap.Graph.Say (M) & ")"));
   end;
   --  ② 三条臂、五台相机;第 2 条臂没有眼、也没有合拢通道,第 3 条臂五个合拢通道、眼是第 1 台;
   --     第 0、2、4 台不长在任何臂上;问第 4 条臂(没有)⇒ 空。
   --  病:第 3 条臂的通道号按两条臂算错;没有合拢通道的臂被当成有一个(DR1 / DR2 09-28:对着不存在的通道合空、清单里列出不存在的手指);
   --  几只不动的眼只认一只(世界相机 = 一只);问一条不存在的臂拿到下一段通道号(越界读 Amp)。
   --  牙:合拢通道按"至少一个"答(Selfmap.Measure 原来的 Max (1, …))⇒ 第 2 条臂 1 个;不长在臂上的眼只认一只 ⇒ [0];
   --     位姿通道不查"有没有这条臂" ⇒ 第 4 条臂 18..23
   declare
      M : constant Selfmap.Body_Map := Map_Of (3, 5, Vec ([1, 0, 5]), Vec ([3, -1, 1]));
      Ok_Pose : constant Boolean := Same (Selfmap.Graph.Pose_Channels (M, 2), Vec ([12, 13, 14, 15, 16, 17]))
                                    and then Selfmap.Graph.Pose_Channels (M, 3).Is_Empty;
      Ok_Close : constant Boolean := Selfmap.Graph.Closing_Count (M, 0) = 1 and then Selfmap.Graph.Closing_Count (M, 1) = 0
                                     and then Selfmap.Graph.Closing_Count (M, 2) = 5 and then Selfmap.Graph.Closing_Count (M, 3) = 0;
      Ok_Eyes : constant Boolean := Same (Selfmap.Graph.Eyes_On (M, 0), Vec ([3])) and then Selfmap.Graph.Eyes_On (M, 1).Is_Empty
                                    and then Same (Selfmap.Graph.Eyes_On (M, 2), Vec ([1])) and then Selfmap.Graph.Eyes_On (M, 3).Is_Empty;
      Ok_Off : constant Boolean := Same (Selfmap.Graph.Eyes_Off_Arms (M), Vec ([0, 2, 4]));
      Say : constant String := Selfmap.Graph.Say (M);
      Ok_Say : constant Boolean :=
        Ada.Strings.Fixed.Index (Say, "第 2 条臂:位姿通道 6,7,8,9,10,11、合拢通道 0 个、长在它上面的眼(相机号)没有") > 0
        and then Ada.Strings.Fixed.Index (Say, "第 3 条臂:位姿通道 12,13,14,15,16,17、合拢通道 5 个、长在它上面的眼(相机号)1") > 0
        and then Ada.Strings.Fixed.Index (Say, "不长在任何臂上的眼(相机号)0,2,4") > 0;
   begin
      Check (Ok_Pose and then Ok_Close and then Ok_Eyes and then Ok_Off and then Ok_Say,
             "身体图通用问法 · 三条臂、五台相机、第 2 条臂没眼没合拢通道:第 3 条臂位姿通道 " & Show (Selfmap.Graph.Pose_Channels (M, 2))
             & "、问第 4 条臂 " & Show (Selfmap.Graph.Pose_Channels (M, 3)) & "(该空)"
             & " · 合拢通道 " & Codec.Img (Selfmap.Graph.Closing_Count (M, 0)) & "/" & Codec.Img (Selfmap.Graph.Closing_Count (M, 1)) & "/"
             & Codec.Img (Selfmap.Graph.Closing_Count (M, 2)) & "(该 1/0/5)"
             & " · 长在臂上的眼 " & Show (Selfmap.Graph.Eyes_On (M, 0)) & Show (Selfmap.Graph.Eyes_On (M, 1)) & Show (Selfmap.Graph.Eyes_On (M, 2))
             & " · 不长在任何臂上的眼 " & Show (Selfmap.Graph.Eyes_Off_Arms (M)) & "(该 [0,2,4])"
             & " · 开机报告念全了 " & Boolean'Image (Ok_Say));
   end;
   --  ③ 身体文件没记每条臂几个合拢通道(09-30 以前写的:H4、DR2 那两份就没有 jaws)⇒ 每条臂 0 个;Act.Any_Fingers 问的就是它(Jaws_Of → Closing_Count),
   --  握区表里就算留着一只量过的手,也不说"我有手指"(开机照实说要重量)。
   --  病:没记就当 1 个(09-30 以前 Jaws_Of 的 `else 1`)⇒ 五指手装回来只剩第 0 号、没有抓握的身体被说成有手指。牙:没记按 1 个答 ⇒ 1/1、有手指
   declare
      C : Act.Context;
      Hd : Zone.Hand;
      Zv : Zone.Hand_Zone;
      Zero_Each, No_Fingers : Boolean;
   begin
      C.Map := Map_Of (2, 3, Empty, Vec ([1, 2]));
      Zv.Valid := True;
      Hd.Arm := 0; Hd.K := 0;
      Hd.Zones.Append (Zv);
      C.Hands.Append (Hd);
      Zero_Each := Selfmap.Graph.Closing_Count (C.Map, 0) = 0 and then Selfmap.Graph.Closing_Count (C.Map, 1) = 0;
      No_Fingers := not Act.Any_Fingers (C);
      Check (Zero_Each and then No_Fingers,
             "身体图通用问法 · 身体文件没记合拢通道:每条臂 " & Codec.Img (Selfmap.Graph.Closing_Count (C.Map, 0)) & "/"
             & Codec.Img (Selfmap.Graph.Closing_Count (C.Map, 1)) & " 个(该 0/0,不猜 1)· 握区表里留着一只量过的手也不说有手指 " & Boolean'Image (No_Fingers));
   end;
end Welds_Path_1;
