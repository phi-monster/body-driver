with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Codec;
package body Selfmap.Graph is
   function Arm_Count (M : Body_Map) return Natural is (M.Arms);

   --  今天每条臂的位姿通道挨着排:第 Arm 条臂占 Arm × Per_Arm 起的 Per_Arm 个(Selfmap.Measure、Bodyfile 都这么排)
   function Pose_Channels (M : Body_Map; Arm : Natural) return Ints is
      R : Ints;
   begin
      if Arm < M.Arms then
         for K in 0 .. M.Per_Arm - 1 loop
            R.Append (Arm * M.Per_Arm + K);
         end loop;
      end if;
      return R;
   end Pose_Channels;

   --  M.Jaws 一条臂一个数(Selfmap.Measure 数的 / 身体文件记的);没记(09-30 以前的身体文件)⇒ 0
   function Closing_Count (M : Body_Map; Arm : Natural) return Natural is
     (if Arm < Natural (M.Jaws.Length) then Natural (Integer'Max (0, M.Jaws (Arm))) else 0);

   --  开机逐组推一下量过(M.Groups 非空,I1 10-01):这条臂那一组推的时候整幅跟着动的每一只眼(一条臂几只都行);
   --  没按组量过(旧的身体图)⇒ 照 M.Cam_On_Arm 答(一条臂最多记一只,−1 = 没有)
   function Eyes_On (M : Body_Map; Arm : Natural) return Ints is
      R : Ints;
   begin
      if not M.Groups.Is_Empty then
         for G of M.Groups loop
            if G.Role = Selfmap.Arm and then G.Arm = Integer (Arm) then
               for E of G.Eyes loop
                  if not R.Contains (E) then
                     R.Append (E);
                  end if;
               end loop;
            end if;
         end loop;
         return R;
      end if;
      if Arm < Natural (M.Cam_On_Arm.Length) and then M.Cam_On_Arm (Arm) >= 0 then
         R.Append (M.Cam_On_Arm (Arm));
      end if;
      return R;
   end Eyes_On;

   --  不长在任何臂上、也不被扛着全身的那组带着走的眼
   function Eyes_Off_Arms (M : Body_Map) return Ints is
      R : Ints;
      function On_Body (Cm : Natural) return Boolean is
      begin
         if M.Groups.Is_Empty then
            return M.Cam_On_Arm.Contains (Cm);
         end if;
         for G of M.Groups loop
            if G.Role in Selfmap.Arm | Selfmap.Carrying and then G.Eyes.Contains (Cm) then
               return True;
            end if;
         end loop;
         return False;
      end On_Body;
   begin
      for Cm in 0 .. M.N_Cams - 1 loop
         if not On_Body (Cm) then
            R.Append (Cm);
         end if;
      end loop;
      return R;
   end Eyes_Off_Arms;

   --  推一下每只眼都整幅在动的那几组(Groups 的下标;开机没按组量过 ⇒ 空)
   function Carrying_Groups (M : Body_Map) return Ints is
      R : Ints;
   begin
      for G in 0 .. Natural (M.Groups.Length) - 1 loop
         if M.Groups (G).Role = Selfmap.Carrying then
            R.Append (G);
         end if;
      end loop;
      return R;
   end Carrying_Groups;

   function Whole_Group (M : Body_Map) return Integer is
      N_Arm : Natural := 0;
      Arm_G : Integer := -1;
      Arm_Idx : Integer := -1;
   begin
      for G in 0 .. Natural (M.Groups.Length) - 1 loop
         if M.Groups (G).Role = Selfmap.Carrying then
            return Integer (G);
         end if;
      end loop;
      for G in 0 .. Natural (M.Groups.Length) - 1 loop
         case M.Groups (G).Role is
            when Selfmap.Arm =>
               N_Arm := N_Arm + 1; Arm_G := Integer (G); Arm_Idx := M.Groups (G).Arm;
            when Selfmap.Piece =>
               return -1;   --  有一块长在哪儿量不出的零件:它不一定跟着这条臂动
            when others =>
               null;
         end case;
      end loop;
      if N_Arm /= 1 then
         return -1;
      end if;
      for G of M.Groups loop
         if G.Role = Selfmap.Closing and then G.Arm /= Arm_Idx then
            return -1;
         end if;
      end loop;
      return Arm_G;
   end Whole_Group;

   function Whole_Arm (M : Body_Map) return Integer is
      G : constant Integer := Whole_Group (M);
   begin
      return (if G >= 0 and then M.Groups (Natural (G)).Role = Selfmap.Arm then M.Groups (Natural (G)).Arm else -1);
   end Whole_Arm;

   function Why_No_Me (M : Body_Map) return String is
      G : constant Integer := Whole_Group (M);
      N_Arm : Natural := 0;
      Arm_G : Integer := -1;
      Arm_Idx : Integer := -1;
   begin
      if Whole_Arm (M) >= 0 then
         return "";
      end if;
      if G >= 0 then
         return "整个我是第 " & Codec.Img (Natural (G)) & " 组(推它,我每一只看得出的眼里整幅画面都跟着动 = 扛着全身的那组);"
           & "它不是一条臂,我只会按臂上的零件走,还不会推着它走";
      end if;
      if M.Groups.Is_Empty then
         return "这一次开机没有一组一组推着认,我说不出哪一组带着我身上的每一样";
      end if;
      for I in 0 .. Natural (M.Groups.Length) - 1 loop
         if M.Groups (I).Role = Selfmap.Piece then
            return "第 " & Codec.Img (I) & " 组推了只有画面里的一块动,我量不出它长在哪条臂上 ⇒ 说不出推哪一组我身上的每一样都跟着动";
         end if;
         if M.Groups (I).Role = Selfmap.Arm then
            N_Arm := N_Arm + 1; Arm_G := Integer (I); Arm_Idx := M.Groups (I).Arm;
         end if;
      end loop;
      if N_Arm = 0 then
         return "我没量出臂(推哪一组,哪只眼里整幅画面都不动),也没有一组扛着全身";
      end if;
      if N_Arm > 1 then
         return "我量出 " & Codec.Img (N_Arm) & " 条臂,哪一条都不带着别的(推一条,别的那几条长着的眼不跟着整幅动),也没有一组扛着全身";
      end if;
      for I in 0 .. Natural (M.Groups.Length) - 1 loop
         if M.Groups (I).Role = Selfmap.Closing and then M.Groups (I).Arm /= Arm_Idx then
            return "第 " & Codec.Img (I) & " 组合拢通道不长在第 " & Codec.Img (Natural (Arm_G)) & " 组那条臂上 ⇒ 推那条臂,它不一定跟着动";
         end if;
      end loop;
      return "";
   end Why_No_Me;

   function Say (M : Body_Map) return String is
      function List (V : Ints) return String is
         R : Unbounded_String;
      begin
         if V.Is_Empty then
            return "没有";
         end if;
         for I in 0 .. Natural (V.Length) - 1 loop
            Append (R, (if I > 0 then "," else "") & Codec.Img (V (I)));
         end loop;
         return To_String (R);
      end List;
      R : Unbounded_String;
   begin
      Append (R, "身体图(通用问法):" & Codec.Img (Arm_Count (M)) & " 条臂");
      for A in 0 .. Arm_Count (M) - 1 loop
         Append (R, " · 第 " & Codec.Img (A + 1) & " 条臂:位姿通道 " & List (Pose_Channels (M, A)) & "、合拢通道 " & Codec.Img (Closing_Count (M, A))
                 & " 个、长在它上面的眼(相机号)" & List (Eyes_On (M, A)));
      end loop;
      Append (R, " · 不长在任何臂上的眼(相机号)" & List (Eyes_Off_Arms (M)) & " · 扛着全身走的组 " & List (Carrying_Groups (M)));
      if Whole_Group (M) >= 0 then
         Append (R, " · 整个我 = 第 " & Codec.Img (Natural (Whole_Group (M))) & " 组" & (if Whole_Arm (M) >= 0 then "(第 " & Codec.Img (Natural (Whole_Arm (M)) + 1) & " 条臂)" else ""));
      end if;
      return To_String (R);
   end Say;
end Selfmap.Graph;
