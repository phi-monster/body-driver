with Ada.Text_IO; use Ada.Text_IO;
with Codec;
package body Exam is

   function Row_Name (X : Row_Id) return String is
     (case X is
         when Sideways => "左右",
         when Updown   => "上下",
         when Nearness => "远近",
         when Bigness  => "看着多大",
         when Facing   => "朝哪");

   function Verdict_Name (V : Verdict) return String is
     (case V is
         when Usable   => "能用",
         when Unproven => "没证过",
         when Unstable => "不稳",
         when Dead     => "死的");

   --  这一行在这具身体上,最响的那个通道推一格能把它推动多少。
   --  一格 = 那个通道自己量出来的探针幅度 ⇒ 所有行都换算到同一种货币("几格"),行与行之间才可比。
   procedure Row_Effect (M : Selfmap.Body_Map; T : Learned.Stored_Effect; R : Row_Id;
                         Per_Notch : out Long_Float; Best : out Integer) is
      Ri : constant Natural := Row_Id'Pos (R);
   begin
      Per_Notch := 0.0;
      Best := -1;
      for C in 0 .. T.E.N - 1 loop
         declare
            G : constant Natural := T.Arm * M.Per_Arm + C;
            Notch : constant Long_Float := (if G < Natural (M.Amp.Length) then M.Amp (G) else 0.0);
            E : constant Long_Float := abs (T.E.B (C, Ri)) * Notch;
         begin
            if E > Per_Notch then
               Per_Notch := E;
               Best := C;
            end if;
         end;
      end loop;
   end Row_Effect;

   function Judge (M : Selfmap.Body_Map; Tables : Learned.Effect_Vectors.Vector) return Report is
      Rep : Report;
   begin
      --  ① 每个通道:身体听不听自己的话
      for Ch in 0 .. (if M.Channels > 0 then M.Channels - 1 else 0) loop
         exit when Ch >= Natural (M.Amp.Length);
         declare
            Cc : Chan_Check;
            A : constant Long_Float := M.Amp (Ch);
            D : constant Long_Float := (if Ch < Natural (M.Delivered.Length) then M.Delivered (Ch) else 0.0);
         begin
            Cc.Arm := (if M.Per_Arm > 0 then Ch / M.Per_Arm else 0);
            Cc.K := (if M.Per_Arm > 0 then Ch mod M.Per_Arm else Ch);
            Cc.Amp := A;
            Cc.Delivered := D;
            Cc.Obey := (if A /= 0.0 then abs D / abs A else 0.0);
            if Ch >= Natural (M.Seen.Length) or else not M.Seen (Ch) then
               Cc.V := Dead;
               Cc.Why := To_Unbounded_String ("推它的时候,没有任何一台相机看见有东西跟着动");
            else
               Cc.V := Unproven;
               Cc.Why := To_Unbounded_String ("看见它动了,但同一个推法没有重复过 ⇒ 不知道它稳不稳");
            end if;
            Rep.Chans.Append (Cc);
         end;
      end loop;

      --  ② 每只眼睛:活没活。安静【不能】当作活着的证据 —— 死眼最安静。
      for Cam in 0 .. (if M.N_Cams > 0 then M.N_Cams - 1 else 0) loop
         declare
            Ec : Eye_Check;
            Mx : Long_Float := 0.0;
         begin
            Ec.Still_Floor := (if Cam < Natural (M.Pic_Floor.Length) then Natural (Integer'Max (0, M.Pic_Floor (Cam))) else 0);
            Ec.Has_Contrast := False;      --  一帧之内的明暗跨度:开机时根本没量过这一项
            Ec.Contrast := 0;
            for A in 0 .. (if M.Arms > 0 then M.Arms - 1 else 0) loop
               declare
                  Idx : constant Natural := A * M.N_Cams + Cam;
               begin
                  if Idx < Natural (M.Cam_Frac.Length) and then M.Cam_Frac (Idx) > Mx then
                     Mx := M.Cam_Frac (Idx);
                  end if;
               end;
            end loop;
            Ec.Moves := Mx;
            Ec.Rides_On := -1;
            for A in 0 .. (if M.Arms > 0 then M.Arms - 1 else 0) loop
               if A < Natural (M.Cam_On_Arm.Length) and then M.Cam_On_Arm (A) = Integer (Cam) then
                  Ec.Rides_On := Integer (A);
               end if;
            end loop;
            if not Ec.Has_Contrast then
               Ec.V := Unproven;
               Ec.Why := To_Unbounded_String ("从来没量过它一帧之内有没有明暗差 ⇒ 【证不出它没瞎】。"
                 & "它安静不算证据:一只全黑的眼睛最安静");
            elsif Ec.Contrast = 0 then
               Ec.V := Dead;
               Ec.Why := To_Unbounded_String ("一帧之内一点明暗差都没有 ⇒ 瞎的");
            else
               Ec.V := Usable;
            end if;
            Rep.Eyes.Append (Ec);
         end;
      end loop;

      --  ③ 每一块被跟着的东西:它的五行各自能不能用
      for I in 0 .. Natural (Tables.Length) - 1 loop
         declare
            T : constant Learned.Stored_Effect := Tables (I);
            Tc : Thing_Check;
            Lo : Long_Float := 0.0;
            Hi : Long_Float := 0.0;
            First : Boolean := True;
         begin
            Tc.Arm := T.Arm; Tc.Cam := T.Cam; Tc.Kind := T.Kind; Tc.Chan_K := T.Chan_K; Tc.Blob := T.Blob;
            for R in Row_Id loop
               declare
                  Rc : Row_Check;
                  P : Long_Float;
                  B : Integer;
               begin
                  Row_Effect (M, T, R, P, B);
                  Rc.Per_Notch := P;
                  Rc.Best_Chan := B;
                  Rc.Reps := 0;
                  if P = 0.0 then
                     Rc.V := Dead;
                     Rc.Why := To_Unbounded_String ("这具身体上所有通道推遍,这一行【一次都没动过】");
                     Rc.Instead := To_Unbounded_String ("凡是要靠「" & Row_Name (R) & "」的话,这里都说不出口");
                  else
                     Rc.V := Unproven;
                     Rc.Why := To_Unbounded_String ("量到了(最响的是第" & Codec.Img (B) & " 号通道),但同一个推法没重复过");
                  end if;
                  Tc.Rows (R) := Rc;
                  if P > 0.0 then
                     Tc.Live_Rows := Tc.Live_Rows + 1;
                     if First then Lo := P; Hi := P; First := False;
                     else
                        if P < Lo then Lo := P; end if;
                        if P > Hi then Hi := P; end if;
                     end if;
                  end if;
               end;
            end loop;
            Tc.Loudest := Hi;
            Tc.Faintest := Lo;
            Tc.Ratio := (if Lo > 0.0 then Hi / Lo else 1.0);
            Rep.Things.Append (Tc);
         end;
      end loop;

      Rep.Self_Noise_Never_Measured := M.EE_Noise = 0.0 and then M.Rot_Noise = 0.0;
      Rep.World_Cam_By_Stillness :=
        M.World_Cam < Natural (Rep.Eyes.Length) and then Rep.Eyes (M.World_Cam).V /= Usable;
      return Rep;
   end Judge;

   procedure Say (R : Report) is
      Blocked : Natural := 0;
   begin
      Put_Line ("");
      Put_Line ("══ 体检判决书 ══  (只有【能用】的量,才允许出现在程序里)");
      Put_Line ("");
      Put_Line ("眼睛:");
      for I in 0 .. Natural (R.Eyes.Length) - 1 loop
         declare
            E : constant Eye_Check := R.Eyes (I);
         begin
            Put_Line ("  第" & Codec.Img (I) & " 只  " & Verdict_Name (E.V)
              & " · 静止时最大灰度差 " & Codec.Img (Integer (E.Still_Floor))
              & " · 手一动它变 " & Codec.Fmt (E.Moves, 3) & " 幅"
              & (if E.Rides_On >= 0 then " · 长在第" & Codec.Img (E.Rides_On + 1) & " 只手上" else " · 不跟手动"));
            if E.V /= Usable then
               Put_Line ("        " & To_String (E.Why));
            end if;
         end;
      end loop;
      Put_Line ("");
      Put_Line ("通道(身体听不听自己的话):");
      for I in 0 .. Natural (R.Chans.Length) - 1 loop
         declare
            C : constant Chan_Check := R.Chans (I);
         begin
            Put_Line ("  第" & Codec.Img (C.Arm + 1) & " 只手第" & Codec.Img (C.K) & " 轴  " & Verdict_Name (C.V)
              & " · 命令 " & Codec.Fmt (C.Amp, 4) & " 实到 " & Codec.Fmt (C.Delivered, 4)
              & " · 照做了 " & Codec.Fmt (C.Obey, 2) & " 成");
         end;
      end loop;
      Put_Line ("");
      Put_Line ("被跟着的每一块东西,五行各自的判决(单位:推一格能把它推动多少):");
      for I in 0 .. Natural (R.Things.Length) - 1 loop
         declare
            T : constant Thing_Check := R.Things (I);
         begin
            Put_Line ("  ── 第" & Codec.Img (T.Arm + 1) & " 只手 · 第" & Codec.Img (T.Cam)
              & " 台相机 · 通道" & Codec.Img (T.Chan_K) & " 带的第" & Codec.Img (T.Blob) & " 团 ──");
            for Rw in Row_Id loop
               declare
                  Rc : constant Row_Check := T.Rows (Rw);
               begin
                  Put_Line ("     " & Row_Name (Rw) & "  " & Verdict_Name (Rc.V)
                    & "   一格推动 " & Codec.Fmt (Rc.Per_Notch, 6));
                  if Rc.V /= Usable then
                     Put_Line ("            " & To_String (Rc.Why));
                     Blocked := Blocked + 1;
                  end if;
                  if Length (Rc.Instead) > 0 then
                     Put_Line ("            ⇒ " & To_String (Rc.Instead));
                  end if;
               end;
            end loop;
            if T.Ratio > 1.0 then
               Put_Line ("     活着的行里,最响的比最哑的大 " & Codec.Fmt (T.Ratio, 0) & " 倍"
                 & " ⇒ 不把每行换算成「几格」就直接解方程的话,最响那行的话语权是最哑那行的 "
                 & Codec.Fmt (T.Ratio * T.Ratio, 0) & " 倍(解方程按平方计权)");
            end if;
         end;
      end loop;
      Put_Line ("");
      if R.Self_Noise_Never_Measured then
         Put_Line ("🔴 本体读数抖多少,量出来【恰好是 0】 ⇒ 所有「它动得比噪声大吗」的闸永远为真 = 等于没有闸");
      end if;
      if R.World_Cam_By_Stillness then
         Put_Line ("🔴 主相机是按「手动时画面变化最少」挑的,而这台相机【没被证明活着】"
           & " ⇒ 一只死眼变化恒为 0,按这个规矩永远夺冠");
      end if;
      Put_Line ("");
      Put_Line ("总计:" & Codec.Img (Blocked) & " 个量不许被程序引用。");
      Put_Line ("");
   end Say;

end Exam;
