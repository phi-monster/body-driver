separate (Act)
procedure Geo_Away (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Step_Limit : Natural; Amt : Long_Float;
                    Event : out Unbounded_String; Steps_Taken : out Natural; Beats : out Natural) is
   Beats0 : constant Natural := Plug.Steps (L);
   Ln : constant Long_Float := Stride_Of (C, Arm) * Amt;   --  一步 = 量出来的最大一档 × 脑的档位
   N : constant Natural := (if Step_Limit > 0 then Step_Limit else 1);
   Mok : Boolean;
   Went : Long_Float := 0.0;
begin
   Steps_Taken := 0; Beats := 0;
   if Geom.Norm (C.Geo_Dir) <= 0.0 or else Ln <= 0.0 then
      Event := S ("refused: I have not walked toward it yet, so I do not know which way is away from it");
      return;
   end if;
   for K in 1 .. N loop
      declare
         Cur : constant Plug.Arm_Pose := F.EE (Arm);
         Dw : constant Geom.V3 := [-C.Geo_Dir (0) * Ln, -C.Geo_Dir (1) * Ln, -C.Geo_Dir (2) * Ln];
      begin
         Geo_Move (L, C, F, Arm, Dw, Mok);
         Steps_Taken := Steps_Taken + 1;
         declare
            Now : constant Plug.Arm_Pose := F.EE (Arm);
            Got : constant Long_Float := ((Now (0) - Cur (0)) * Dw (0) + (Now (1) - Cur (1)) * Dw (1) + (Now (2) - Cur (2)) * Dw (2)) / Ln;
         begin
            Went := Went + Got;
            C.Geo_Dist := C.Geo_Dist + Got; C.Geo_At := Now;   --  离它远了这么多;刚算的"笼住"距离跟着变
            if Got + Got < Ln then   --  实到不到要的一半(纯数学的一半)= 被顶住了
               Event := S ("resist: I commanded a step of " & Len (C, Ln) & " away from it and my hand only went " & Len (C, Got));
               Beats := Beats_Since (L, Beats0);
               return;
            end if;
         end;
      end;
   end loop;
   Event := S ("amount: arrived (I moved " & Len (C, Went) & " away from it, along the line I had come in on)");
   Beats := Beats_Since (L, Beats0);
end Geo_Away;
