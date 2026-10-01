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
      return To_String (R);
   end Say;
end Selfmap.Graph;
