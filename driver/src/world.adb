with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
package body World is
   type Bool_Array is array (Natural range <>) of Boolean;
   procedure Init (S : in out State; N_Cams : Natural) is
   begin
      S.Cams.Clear;
      for I in 1 .. N_Cams loop
         S.Cams.Append (Cam_State'(others => <>));
      end loop;
      S.Holding := False; S.Held_Arm := -1; S.Held_Cam := -1; S.Held_Slot := -1;
   end Init;

   procedure Reset_All (S : in out State) is
      N : constant Natural := Natural (S.Cams.Length);
   begin
      Init (S, N);
   end Reset_All;

   procedure Pin (S : in out State; Cam : Natural; U, V : Long_Float) is
      Bd : Long_Float := 1.0e9;
      Best : Integer := -1;
   begin
      if Cam >= Natural (S.Cams.Length) then
         return;
      end if;
      declare
         Cs : Cam_State := S.Cams (Cam);
      begin
         for I in 0 .. Natural (Cs.Slots.Length) - 1 loop
            declare
               R : constant Picture.Region := (if Cs.Slots (I).Present then Cs.Slots (I).R else Cs.Slots (I).Shadow);
               D : constant Long_Float := Sqrt ((R.Cu - U) ** 2 + (R.Cv - V) ** 2);
            begin
               if D < Bd then
                  Bd := D; Best := I;
               end if;
            end;
         end loop;
         if Best >= 0 then
            declare
               Sl : Slot := Cs.Slots (Natural (Best));
            begin
               Sl.Pinned := True; Sl.Present := True; Sl.Seen := True;
               Cs.Slots.Replace_Element (Natural (Best), Sl);
            end;
            S.Cams.Replace_Element (Cam, Cs);
         end if;
      end;
   end Pin;

   procedure Observe (S : in out State; Cam : Natural; Regs : Picture.Regions; W, H : Natural) is
   begin
      if Cam >= Natural (S.Cams.Length) then
         return;
      end if;
      declare
         Cs : Cam_State := S.Cams (Cam);
         Used : Bool_Array (0 .. Natural'Max (0, Natural (Regs.Length) - 1)) := [others => False];
      begin
         for Si in 0 .. Natural (Cs.Slots.Length) - 1 loop
            declare
               Sl : Slot := Cs.Slots (Si);
               Ref : constant Picture.Region := (if Sl.Present then Sl.R else Sl.Shadow);
               --  🔴 认号的半径原来是"这块自己框的一半",太小:手一动、块一变大,它就对不上号,
               --  于是同一个东西被当成【新的一件】接在清单末尾 ⇒ 编号每轮都在变,脑每轮都要重新看图找它是几号
               --  (实测:一晚上一多半的轮次花在重新认号上,还点错过好几次)。
               --  改:半径放到"这块自己一个半身位,再小也有十分之一画幅";同时要求【大小是一个量级】
               --  (三倍以内)才准认 —— 位置放宽、身份收紧,比原来两头都松的做法稳。
               --  半径 = 这块自己的一个半身位,再小也有十分之一画幅(两个都是【占画幅的比例】,无量纲:
               --  跟相机、镜头、机器人大小都无关)
               Tol : constant Long_Float := Long_Float'Max (
                  Long_Float'Max (Long_Float (Ref.X1 - Ref.X0) / Long_Float (W), Long_Float (Ref.Y1 - Ref.Y0) / Long_Float (H)) * 1.5,
                  0.1);
               Best : Integer := -1;
               Bd : Long_Float := 1.0e9;
            begin
               for Ri in 0 .. Natural (Regs.Length) - 1 loop
                  if not Used (Ri)
                    and then Regs (Ri).Count * 3 >= Ref.Count and then Ref.Count * 3 >= Regs (Ri).Count
                  then
                     declare
                        D : constant Long_Float := Sqrt ((Regs (Ri).Cu - Ref.Cu) ** 2 + (Regs (Ri).Cv - Ref.Cv) ** 2);
                     begin
                        if D <= Tol and then D < Bd then
                           Bd := D; Best := Ri;
                        end if;
                     end;
                  end if;
               end loop;
               if Best >= 0 then
                  Used (Best) := True;
                  Sl.Present := True; Sl.R := Regs (Best); Sl.Shadow := Regs (Best); Sl.Seen := True;
               elsif Sl.Pinned then
                  null;             --  脑指的那个:这一帧没对上也留着,位置留上一次的
               elsif Regs.Is_Empty then
                  --  🔴 这张画面里一块都切不出来(没有深度、颜色也切不出)⇒ 不能因此说"东西不见了"。
                  --  它没消失,是我们切不出来 —— 位置留着上一次的,照样当它在,由光流在段内把它追下去。
                  --  以前这里一律标成"看不见",于是脑指出来的那个东西下一轮就点不了名(HG/HH 实测)。
                  null;
               else
                  Sl.Present := False;
               end if;
               Cs.Slots.Replace_Element (Si, Sl);
            end;
         end loop;
         for Ri in 0 .. Natural (Regs.Length) - 1 loop
            if not Used (Ri) then
               Cs.Slots.Append (Slot'(Present => True, R => Regs (Ri), Seen => True, Shadow => Regs (Ri), Pinned => False));
            end if;
         end loop;
         S.Cams.Replace_Element (Cam, Cs);
      end;
   end Observe;

   function Count (S : State; Cam : Natural) return Natural is
     (if Cam < Natural (S.Cams.Length) then Natural (S.Cams (Cam).Slots.Length) else 0);

   function Get (S : State; Cam : Natural; I : Natural) return Slot is
     (if Cam < Natural (S.Cams.Length) and then I < Natural (S.Cams (Cam).Slots.Length) then S.Cams (Cam).Slots (I) else (others => <>));

   function Vanished (Regs : Picture.Regions; Origin : Picture.Region; W, H : Natural) return Boolean is
   begin
      for R of Regs loop
         if R.Count * 2 >= Origin.Count and then Picture.Inside (Origin, R.Cu, R.Cv, W, H, 0.5) then
            return False;
         end if;
      end loop;
      return True;
   end Vanished;
end World;
