with Ada.Text_IO; use Ada.Text_IO;
with Codec;
with Table;
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
                         Per_Notch : out Long_Float; Best : out Integer;
                         Reps : out Natural; Scatter : out Long_Float) is
      Ri : constant Natural := Row_Id'Pos (R);
   begin
      Per_Notch := 0.0;
      Best := -1;
      Reps := 0;
      Scatter := 0.0;
      for C in 0 .. T.E.N - 1 loop
         declare
            G : constant Natural := T.Arm * M.Per_Arm + C;
            Notch : constant Long_Float := (if G < Natural (M.Amp.Length) then M.Amp (G) else 0.0);
            E : constant Long_Float := abs (T.E.B (C, Ri)) * Notch;
         begin
            if E > Per_Notch then
               Per_Notch := E;
               Best := C;
               Reps := T.E.Reps (C);
               Scatter := T.E.Scatter (C, Ri);
            end if;
         end;
      end loop;
   end Row_Effect;

   --  这张表所属那条臂的"每通道一格是多大"
   function Notch_Of (M : Selfmap.Body_Map; T : Learned.Stored_Effect) return Table.Vec is
      N : Table.Vec := [others => 0.0];
   begin
      for C in 0 .. T.E.N - 1 loop
         declare
            G : constant Natural := T.Arm * M.Per_Arm + C;
         begin
            N (C) := (if G < Natural (M.Amp.Length) then M.Amp (G) else 0.0);
         end;
      end loop;
      return N;
   end Notch_Of;

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
                  P, Sc : Long_Float;
                  B : Integer;
                  Nr : Natural;
               begin
                  Row_Effect (M, T, R, P, B, Nr, Sc);
                  Rc.Per_Notch := P;
                  Rc.Best_Chan := B;
                  Rc.Reps := Nr;
                  Rc.Spread := Sc;
                  --  判"能不能用"只用 Table.Row_Proven 那一个定义(全仓唯一),这里只负责把理由说成人话
                  if Table.Row_Proven (T.E, Notch_Of (M, T), Row_Id'Pos (R)) then
                     Rc.V := Usable;
                  elsif P = 0.0 then
                     Rc.V := Dead;
                     Rc.Why := To_Unbounded_String ("这具身体上所有通道推遍,这一行【一次都没动过】");
                     Rc.Instead := To_Unbounded_String ("凡是要靠「" & Row_Name (R) & "」的话,这里都说不出口");
                  elsif Nr < 2 then
                     Rc.V := Unproven;
                     Rc.Why := To_Unbounded_String ("量到了,但推得动它的通道里,没有一个同一个推法做过两次以上 ⇒ 证不出稳");
                  elsif Sc >= 1.0 then
                     --  散布不小于均值本身 = 同一个推法给出的结果彼此打架,拿它算动作就是拿噪声算动作
                     Rc.V := Unstable;
                     Rc.Why := To_Unbounded_String ("同一个推法重复" & Codec.Img (Nr) & " 次,结果的散布是均值的 "
                       & Codec.Fmt (Sc, 2) & " 倍 ⇒ 它自己跟自己打架");
                     Rc.Instead := To_Unbounded_String ("换一行稳的,或者换一个不靠「" & Row_Name (R) & "」的说法");
                  else
                     Rc.V := Unproven;
                     Rc.Why := To_Unbounded_String ("推得动它的通道里,没有一个又稳又够格");
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

end Exam;
