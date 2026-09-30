separate (Jointboot)
procedure Fit_Arm (A : Natural; D : Sweep_Data; Cs : Kinem.Corr_Vectors.Vector; Dump : String; M : out Kinem.Model; Ok : out Boolean;
                   Note : out Unbounded_String) is
   Rep : Kinem.Fit_Report;
   procedure Say (S : String) is
   begin
      Append (Note, "[身] 📐 " & S & ASCII.LF);
   end Say;
begin
   M := (others => <>); Ok := False; Note := Null_Unbounded_String;
   if Natural (D.Frames.Length) < 3 or else Cs.Is_Empty then
      Say ("  运动学 · 第" & Codec.Img (A + 1) & " 只手:扫描格子太少 / 没有配点 ⇒ 量不了");
      return;
   end if;
   Kinem.Fit (D.Frames, 0, Cs, Long_Float (D.W) / 2.0, Long_Float (D.H) / 2.0, Long_Float (D.W), M, Rep, Ok);
   declare
      T : Unbounded_String;
   begin
      for J in 0 .. Natural (Rep.Joint_Med.Length) - 1 loop
         declare
            function Px (X : Long_Float) return String is (if X < 0.0 then "试不了" else Codec.Fmt (X, 3));
            Sl : constant Boolean := J < Natural (Rep.Slide.Length) and then Rep.Slide (J);
         begin
            Append (T, " " & (if Rep.Joint_Med (J) < 0.0 then "量不了" elsif Sl then "走 " else "转 ") & Px (Rep.Joint_Med (J))
                    & (if J < Natural (Rep.Joint_Med_Turn.Length) then "[" & (if Sl then "按转 " & Px (Rep.Joint_Med_Turn (J)) else "按走 " & Px (Rep.Joint_Med_Slide (J))) & "]" else "")
                    & "(" & Codec.Img (Rep.Joint_Frames (J)) & " 格)");
         end;
      end loop;
      Say ("  运动学 · 第" & Codec.Img (A + 1) & " 只手:" & (if Ok then "量成" else "没量成") & " · 长在眼上的像素 " & Codec.Img (Rep.Eye_Px)
           & " 个(两个以上关节单独转时各有一格没挪 = 自己身上的),从它们出发的配点 " & Codec.Img (Rep.Eye_Corrs) & " / " & Codec.Img (Rep.N_Corr)
           & " 笔不进解 · 每根轴单独(转 / 走两样各解一次,残差小的那样)的残差中位(像素):" & To_String (T));
      T := Null_Unbounded_String;
      for X of Rep.Rho loop
         Append (T, " " & Codec.Fmt (X, 3));
      end loop;
      Append (T, " · 定比例用了 " & Codec.Img (Rep.Rho_Pairs) & " 对(三对起步 " & Codec.Fmt (Rep.Rho_Start_Px, 3) & " px → 全部重解中位 " & Codec.Fmt (Rep.Rho_Px, 3) & " px)");
      Append (T, " · 各步秒数");
      for X of Rep.Secs loop
         Append (T, " " & Codec.Fmt (X, 1));
      end loop;
      Say ("    焦距 起步 " & Codec.Fmt (Rep.F_Start, 1) & " → " & Codec.Fmt (Rep.F_Axes, 1) & " → " & Codec.Fmt (Rep.F, 1) & " · 一起解的残差中位 " & Codec.Fmt (Rep.Med_Px, 3) & " px、九成 "
           & Codec.Fmt (Rep.P90_Px, 3) & " px · 内点 " & Codec.Img (Rep.N_Used) & " / " & Codec.Img (Rep.N_Corr) & " · 各轴远近比例(以第"
           & Codec.Img (Rep.Ref_Joint) & " 根为 1):" & To_String (T) & (if Rep.Flipped then " · 平移反过一次号" else ""));
      if Length (Rep.Unsettled) > 0 then
         Say ("    ⚠ 碰到保险上限还没收住:" & To_String (Rep.Unsettled));
      end if;
      Say ("    多视图一起解(起点那格的格点配进各格 = 轨迹,按重投影):" & Codec.Img (Rep.Mv_Tracks) & " 条轨迹 " & Codec.Img (Rep.Mv_Obs) & " 笔 · 重投影中位 "
           & Codec.Fmt (Rep.Mv_Start_Px, 3) & " → " & Codec.Fmt (Rep.Mv_Px, 3) & " px、九成 " & Codec.Fmt (Rep.Mv_P90_Px, 3) & " px · " & Codec.Img (Rep.Mv_Iters) & " 轮 · 焦距 "
           & Codec.Fmt (Rep.F, 1));
   end;
   if Dump /= "" and then Ok then
      declare
         Fo : Ada.Text_IO.File_Type;
      begin
         Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Dump & "/kinem_arm" & Codec.Img (A) & ".txt");
         Ada.Text_IO.Put_Line (Fo, "arm " & Codec.Img (A) & " n " & Codec.Img (M.N) & " f " & Codec.Fmt (M.F, 6) & " cx " & Codec.Fmt (M.Cx, 3) & " cy " & Codec.Fmt (M.Cy, 3));
         Ada.Text_IO.Put (Fo, "q0");
         for X of M.Q0 loop
            Ada.Text_IO.Put (Fo, " " & Codec.Fmt (X, 9));
         end loop;
         Ada.Text_IO.New_Line (Fo);
         for J in 0 .. M.N - 1 loop
            Ada.Text_IO.Put_Line (Fo, "axis " & Codec.Img (J) & " " & Codec.Fmt (M.Ax (J).W (0), 9) & " " & Codec.Fmt (M.Ax (J).W (1), 9) & " "
                                  & Codec.Fmt (M.Ax (J).W (2), 9) & " " & Codec.Fmt (M.Ax (J).P (0), 9) & " " & Codec.Fmt (M.Ax (J).P (1), 9) & " "
                                  & Codec.Fmt (M.Ax (J).P (2), 9) & " " & Kind_Word (M.Ax (J)));
         end loop;
         for P of M.Eye loop   --  长在眼上的像素(离线回放 alignexam 读回,三角时同样不用)
            Ada.Text_IO.Put_Line (Fo, "eye " & Codec.Fmt (P.U, 3) & " " & Codec.Fmt (P.V, 3));
         end loop;
         Ada.Text_IO.Close (Fo);
      exception
         when others => null;
      end;
   end if;
end Fit_Arm;
