with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
package body Schema is
   function Count (M : Map; Arm, Cam : Natural) return Natural is
      N : Natural := 0;
   begin
      for X of M.S loop
         if X.Arm = Arm and then X.Cam = Cam then
            N := N + 1;
         end if;
      end loop;
      return N;
   end Count;

   procedure Add (M : in out Map; X : Sample; EE_Noise, Rot_Noise : Long_Float) is
   begin
      for I in 0 .. Natural (M.S.Length) - 1 loop
         declare
            Y : constant Sample := M.S (I);
         begin
            if Y.Arm = X.Arm and then Y.Cam = X.Cam then
               declare
                  D : constant Table.Vec := Chan.Delivered (Y.Pose, X.Pose);
                  Dp : constant Long_Float := Table.Norm (D, 3);
                  Dr : constant Long_Float := D (3) ** 2 + D (4) ** 2 + D (5) ** 2;
               begin
                  if Dp <= EE_Noise and then Dr <= Rot_Noise * Rot_Noise then
                     declare
                        Mg : Sample := Y;
                     begin
                        Mg.Pose := X.Pose;
                        for K in Part_Array'Range loop
                           if X.Parts (K).Valid then
                              Mg.Parts (K) := X.Parts (K);
                           end if;
                        end loop;
                        M.S.Replace_Element (I, Mg);
                     end;
                     return;
                  end if;
               end;
            end if;
         end;
      end loop;
      if Count (M, X.Arm, X.Cam) >= Max_Per_Pair then
         for I in 0 .. Natural (M.S.Length) - 1 loop
            if M.S (I).Arm = X.Arm and then M.S (I).Cam = X.Cam then
               M.S.Delete (I);
               exit;
            end if;
         end loop;
      end if;
      M.S.Append (X);
   end Add;

   function Nearest (M : Map; Arm, Cam : Natural; Pose : Plug.Arm_Pose; Amp : Floats; Per_Arm : Natural;
                     Diff : out Table.Vec; Dist : out Long_Float) return Integer is
      Best : Integer := -1;
      Bd : Long_Float := 1.0e30;   --  哨兵(无量纲)
   begin
      Diff := Table.Zero_Vec;
      Dist := 1.0e30;
      for I in 0 .. Natural (M.S.Length) - 1 loop
         declare
            Y : constant Sample := M.S (I);
         begin
            if Y.Arm = Arm and then Y.Cam = Cam then
               declare
                  D : constant Table.Vec := Chan.Delivered (Y.Pose, Pose);
                  Sum : Long_Float := 0.0;
               begin
                  for K in 0 .. Per_Arm - 1 loop
                     declare
                        Ch : constant Natural := Arm * Per_Arm + K;
                        A : constant Long_Float := (if Ch < Natural (Amp.Length) and then Amp (Ch) > 0.0 then Amp (Ch) else 1.0);
                     begin
                        Sum := Sum + (D (K) / A) ** 2;
                     end;
                  end loop;
                  if Sum < Bd then
                     Bd := Sum; Best := I; Diff := D;
                  end if;
               end;
            end if;
         end;
      end loop;
      if Best >= 0 then
         Dist := Sqrt (Bd);
      end if;
      return Best;
   end Nearest;
end Schema;
