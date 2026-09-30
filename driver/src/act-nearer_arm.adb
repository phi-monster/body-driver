separate (Act)
function Nearer_Arm (C : Context; N : Natural) return Integer is
begin
   if N < 1 or else N > Natural (C.Items.Length) then
      return -1;
   end if;
   declare
      It : constant Item := C.Items (N - 1);
      A2 : constant Integer := Cam_Arm (C, It.Cam);
      Best : Integer := -1;
      Best_D : Long_Float := 0.0;
   begin
      if It.Kind not in Thing | Thing_Remembered then
         return -1;
      end if;
      --  在哪只腕眼里被点了名就是那条臂 —— 除非这一集里那条臂已经试过"它身上一段都在够不着那侧"(H60 2026-09-23:右臂虽然横着到头,
      --  剪刀手柄那头仍在够得着这侧,正是它拿起来的;所以只在一段都够不着时才换另一条)
      if A2 >= 0 then
         if C.No_Reach_Arm = A2 and then C.Map.Arms = 2 then
            return 1 - A2;
         end if;
         return A2;
      end if;
      for A in 0 .. C.Map.Arms - 1 loop
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, A, It.Cam, 0);
            D : constant Long_Float := Sqrt ((Z.Cu - It.Cu) ** 2 + (Z.Cv - It.Cv) ** 2);
         begin
            if Z.Valid and then C.No_Reach_Arm /= Integer (A) and then (Best < 0 or else D < Best_D) then
               Best := Integer (A);
               Best_D := D;
            end if;
         end;
      end loop;
      return Best;
   end;
end Nearer_Arm;
