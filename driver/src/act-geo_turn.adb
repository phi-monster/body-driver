separate (Act)
procedure Geo_Turn (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Want : Geom.V3;
                    Amt : Long_Float; Event : out Unbounded_String; Steps_Taken : out Natural;
                    Along : Geom.V3 := [0.0, 0.0, -1.0]) is
   Hc : constant Integer := (if Arm < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (Arm) else -1);
   Notch : constant Long_Float := (if Arm * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (Arm * Chan.Per_Arm + 3) else 0.0);
   --  一条命令最多转多少 = 开机按运动学量出来的"一条命令转得到的最大一档"(看着走,09-28 定:步子是身体的事,不再乘脑的档位;
   --  C1 09-29:乘了一半,转 0.4 rad 花 5 条命令)。Amt 不再用来定步子
   pragma Unreferenced (Amt);
   Cap : constant Long_Float := (if Hc >= 0 and then Natural (Hc) < Natural (C.Geo.Length) then C.Geo (Natural (Hc)).Stride_Rot else 0.0);
begin
   Event := Null_Unbounded_String; Steps_Taken := 0;
   if Hc < 0 or else Natural (Hc) >= Natural (C.Geo.Length) or else Notch <= 0.0 or else Cap <= 0.0 then
      Event := S ("refused: I cannot turn this eye - it does not ride on this arm, or my turning stride has not been measured");
      return;
   end if;
   --  这只眼在手上怎么装的只有一种量法:开机量(运动学 + 对齐);开机没量出 ⇒ 照实说(09-30 删了"干活时盯着一块挪几下现量"那条后备)
   if not C.Geo (Natural (Hc)).Valid then
      Event := S ("refused: I cannot turn this eye - how it sits on my hand was not measured at boot");
      return;
   end if;
   declare
      G : constant Geom.Cam_Geo := C.Geo (Natural (Hc));
      Wk : Selfmap.Walk;   --  这一段转空走的底(走一步判挡没挡用:开头装进开机探针量的那几步)
   begin
      for Step_No in 1 .. 40 loop
         if Plug.Reset_Pending (L) then
            Event := S (Reset_Event);
            return;
         end if;
         declare
            P : constant Plug.Arm_Pose := F.EE (Arm);
            Rc : constant Geom.M3 := Geom.Cam_R (G, P);
            Al : constant Long_Float := Geom.Norm (Along);
            Fwd : constant Geom.V3 := Geom.Ap (Rc, (if Al > 0.0 then [Along (0) / Al, Along (1) / Al, Along (2) / Al] else [0.0, 0.0, -1.0]));
            Cr : constant Geom.V3 := [Fwd (1) * Want (2) - Fwd (2) * Want (1), Fwd (2) * Want (0) - Fwd (0) * Want (2), Fwd (0) * Want (1) - Fwd (1) * Want (0)];
            Sn : constant Long_Float := Geom.Norm (Cr);
            Cs : constant Long_Float := Fwd (0) * Want (0) + Fwd (1) * Want (1) + Fwd (2) * Want (2);
            Ang : constant Long_Float := Arctan (Sn, Cs);
         begin
            if Ang <= Notch then
               Event := S ("amount: arrived (my eye now points there, off by " & Codec.Fmt (Ang, 3) & " rad)");
               return;
            end if;
            declare
               --  正前方正好背对着要的方向时叉积为零:随便取一根和正前方垂直的轴(和世界 z、世界 x 各叉一次,取长的那根)
               Az : constant Geom.V3 := [Fwd (1), -Fwd (0), 0.0];          --  Fwd × z
               Ax : constant Geom.V3 := [0.0, Fwd (2), -Fwd (1)];          --  Fwd × x
               Alt : constant Geom.V3 := (if Geom.Norm (Az) >= Geom.Norm (Ax) then Az else Ax);
               Aln : constant Long_Float := Geom.Norm (Alt);
               Axis : constant Geom.V3 := (if Sn > 1.0e-9 then [Cr (0) / Sn, Cr (1) / Sn, Cr (2) / Sn]
                                           elsif Aln > 1.0e-9 then [Alt (0) / Aln, Alt (1) / Aln, Alt (2) / Aln] else [0.0, 0.0, 1.0]);
               Stp : constant Long_Float := Long_Float'Min (Ang, Cap);
               Rv : constant Geom.V3 := [Axis (0) * Stp, Axis (1) * Stp, Axis (2) * Stp];
               Rn : constant Geom.M3 := Geom.Mul (Geom.Rodrigues (Rv), Geom.Quat_To_R (P));
               Tip0 : constant Geom.V3 := Geom.Ap (Rc, G.Tip);
               Tip1 : constant Geom.V3 := Geom.Ap (Geom.Mul (Rn, G.R_Ce), G.Tip);
               A : Table.Vec := Table.Zero_Vec;
               Ok : Boolean;
               Legs : Selfmap.Leg_Vectors.Vector;
               Rs : Selfmap.Leg_Step_Vectors.Vector;
               Frames : Natural;
               Lim : Selfmap.Limits;   --  一整步(Frac 1)、不另设上限;到了一步看得见的那一档以内就算到(同 Step_Arm 的 Geo_Settle)
               Rep : Selfmap.Leg_Step;
            begin
               A (0) := Tip0 (0) - Tip1 (0); A (1) := Tip0 (1) - Tip1 (1); A (2) := Tip0 (2) - Tip1 (2);
               A (3) := Rv (0); A (4) := Rv (1); A (5) := Rv (2);
               --  走一步(Selfmap.Step):目标 = 转过、指尖补回原处的那个位姿
               Legs.Append (Selfmap.Leg'(Arm => Arm, Goal => Chan.Compose (P, A), Jaw => <>));
               Selfmap.Step (L, C.Map, Legs, Lim, F, Wk, Rs, Frames, Ok);
               if not Rs.Is_Empty then
                  Rep := Rs (0);
               end if;
               Steps_Taken := Steps_Taken + 1;
               declare
                  Got : constant Long_Float := Rep.Turned;   --  沿要转的那根轴实到多少
               begin
                  Geo_Say ("转 " & Codec.Fmt (Stp, 3) & " rad ⇒ 实到 " & Codec.Fmt (Got, 3) & " rad(还差 " & Codec.Fmt (Ang, 3) & ")"
                           & (if Ok then "" else " · 身体说没走成"));
                  --  转不动 = 这一步自己停下、没到、少转的比这一段空转时多出 Blocked 的门(原来:实到不到命令的一半)
                  if Rep.Blocked_R then
                     Event := S ("resist: I commanded a turn of " & Codec.Fmt (Stp, 3) & " rad and my hand only turned " & Codec.Fmt (Got, 3)
                                 & " (still " & Codec.Fmt (Ang, 3) & " rad from pointing there) - a joint is at its end or the pose is not reachable");
                     return;
                  end if;
               end;
            end;
         end;
      end loop;
      Event := S ("steps: I took 40 turning steps and my eye is still not pointing there");
   end;
end Geo_Turn;
