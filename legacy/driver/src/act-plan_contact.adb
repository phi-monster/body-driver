with Contact.Search;
separate (Act)
procedure Plan_Contact (C : in out Context; F : Plug.Frame; Arm, Cam : Natural; Name : Unbounded_String;
                        Pick : out Contact.Search.Candidate; Note : out Unbounded_String; Ok : out Boolean) is
   G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
   Have_Plane : constant Boolean := C.Touch_Valid or else C.Board_Plane;
   Up : constant Geom.V3 := Lie_N (C);
   Sp : constant Geom.V3 := Lie_P (C);
   Tol_P : constant Long_Float := Geo_Base (C, Arm);
   Tol_R : constant Long_Float := (if Arm * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (Arm * Chan.Per_Arm + 3) else 0.0);
   Mu_Lb : Long_Float := 0.0;
   Mu_Ub : Long_Float := Long_Float'Last;
   H : Contact.Search.Hand_Model;
   Surf, Around : Contact.V3_Vectors.Vector;
   Found : Contact.Search.Cand_Vectors.Vector;
   St : Contact.Search.Plan_Stats;
   Reprojected : Boolean := False;
   --  眼在 (R, T) 时手的位姿:眼 → 世界 = 手 → 世界 · 相机 → 手 ⇒ 手 → 世界 = R · R_ceᵀ;眼的中心 = 手的位置 + 手 → 世界 · Off
   function Reach (R : Geom.M3; T : Geom.V3) return Boolean is
      Rp : constant Geom.M3 := Geom.Mul (R, Geom.Tr (G.R_Ce));
      Ow : constant Geom.V3 := Geom.Ap (Rp, G.Off);
      Pe, Re : Long_Float;
      Rok : Boolean;
   begin
      Plug.Reach (Arm, Kinem.To_Pose (Rp, [T (0) - Ow (0), T (1) - Ow (1), T (2) - Ow (2)]), Pe, Re, Rok);
      return not Rok or else (Pe <= Tol_P and then (Tol_R <= 0.0 or else Re <= Tol_R));
   end Reach;
   --  要的动说成话(按它躺的面说:离开 / 沿着 / 往里,绕什么轴转;不说动作)
   function Motion_Words (M : Contact.Twist) return String is
      Ok_L, Ok_A : Boolean;
      L : constant Geom.V3 := Contact.Unit (M.Lin, Ok_L);
      A : constant Geom.V3 := Contact.Unit (M.Ang, Ok_A);
      function Off_Normal (V : Geom.V3) return Long_Float is
        (Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, V (0) * Up (0) + V (1) * Up (1) + V (2) * Up (2)))));
   begin
      if Ok_A then
         return "rotating about an axis " & Codec.Fmt (Off_Normal (A), 2) & " rad off the normal of the surface it lies on";
      elsif Ok_L then
         return "moving " & Codec.Fmt (Off_Normal (L), 2) & " rad off the normal of the surface it lies on (0 = straight off it, "
                & Codec.Fmt (Ada.Numerics.Pi, 2) & " = straight into it)";
      end if;
      return "staying where it is";
   end Motion_Words;
begin
   Ok := False;
   Pick := (others => <>);
   if not C.Sil_Valid or else C.Sil_Name /= Name then
      Note := S ("I have no measured outline of " & To_String (Name) & " (no eye saw it whole while I knew where it was)");
      return;
   end if;
   if not Have_Plane then
      Note := S ("I have not measured the surface it lies on, so I cannot tell how tall it is or where my fingers can go down beside it");
      return;
   end if;
   if G.Lobes.Is_Empty then
      Note := S ("I have no measured fingertips in this eye (my body file has no per-finger tips; they have to be measured once from scratch)");
      return;
   end if;
   --  手 = 这只眼里量到的每一瓣(尖、指肚宽、手指厚),一瓣不少;落位的误差 = 尖的误差和这只手走一档里大的那个
   declare
      Ls : Contact.Search.Lobe_In_Vectors.Vector;
   begin
      for Lg of G.Lobes loop
         Ls.Append (Contact.Search.Lobe_In'(Tip => Lg.Tip, Width => Lg.Wide, Thick => Lg.Thin));
      end loop;
      H := Contact.Search.From_Lobes (Ls, Long_Float'Max (G.Tip_Sd, Tol_P));
   end;
   if not H.Valid then
      Note := S ("my measured fingers do not make a hand I can lay contacts out with: " & To_String (H.Why));
      return;
   end if;
   for Gm of C.Grip_Mus loop
      if Gm.Name = Name then
         Mu_Lb := Gm.Lb; Mu_Ub := Gm.Ub;
      end if;
   end loop;
   --  它的实心模型(取轮廓时面的高度可能只是交点估的;碰过它躺的面 ⇒ 按真的面重投,见 Solid_Of)
   Solid_Of (C, Name, Surf, Reprojected);
   --  旁边的东西:被顶住过、比面高、又不在它自己身上的点。在它身上 = 离它最近的表面点不到一个采样间距(表面上任何一点离最近的采样点都在一个间距以内),
   --  再加上两边的不准(Z 倍的:顶住那一刻尖的误差、它顶面点的误差、位姿读数的噪声)
   declare
      Own : constant Long_Float := C.Sil_Pitch + Stats.Z * Sqrt (G.Tip_Sd ** 2 + C.Sil_Err ** 2 + C.Map.EE_Noise ** 2);
      Own_Bumps : Contact.V3_Vectors.Vector;
      Extra : Contact.V3_Vectors.Vector;
   begin
      for B of C.Bumps loop
         declare
            Near : Boolean := False;
         begin
            for Q of Surf loop
               if Geom.Norm ([B (0) - Q (0), B (1) - Q (1), B (2) - Q (2)]) <= Own then
                  Near := True;
                  exit;
               end if;
            end loop;
            if Near then
               Own_Bumps.Append (B);
            else
               Around.Append (B);
            end if;
         end;
      end loop;
      --  挡住的那一点在它身上(§2 第 27 条"挡了 ⇒ 挡住的那一点记进形状"):手指在那儿碰到了它 ⇒ 它在那儿至少有那么大、那么高
      --  (尖那一刻的位置,误差同上);连同往下到它躺的面的侧壁补进它的形状(同 Solid_Of 的实心、竖壁的假设),下一次就不往那儿下手指
      if not Own_Bumps.Is_Empty then
         Contact.Surface.Walls_To_Support (Own_Bumps, Up, Sp, C.Sil_Pitch, Extra);
         for Q of Extra loop
            Surf.Append (Q);
         end loop;
      end if;
   end;
   --  只要挑中的那一组(量到的摩擦上限在搜索里就用上了;往下伸被挡住会重挑)
   Contact.Search.Plan (Surf, Around, C.Sil_Pitch, C.Sil_Err, Up, Sp, H, Mu_Lb, G.Gap, Reach'Access, 1, Found, St,
                        Want => C.Want_Move, Mu_Ub => Mu_Ub);
   declare
      Asked : constant String := (if C.Want_Move.Given then Motion_Words (C.Want_Move.Move)
                                  else "coming off the surface it lies on together with my hand (you did not say how it should move)");
      Feasible : constant Natural := St.Kept - Natural'Min (St.Kept, St.No_Force + St.Over_Ub);
   begin
      if Found.Is_Empty then
         Note := S ("from " & Codec.Img (Natural (Surf.Length)) & " surface points of " & To_String (Name) & " (top outline from eye " & Codec.Img (Natural (Integer'Max (0, C.Sil_Cam)))
                    & " extended straight down to the surface it lies on, which assumes solid upright sides) I tried " & Codec.Img (St.Poses) & " placements of my hand for "
                    & Asked & ": "
                    & (if St.In_Way then "that way goes into the surface it lies on, so no contacts can do it; "
                       else Codec.Img (St.Air) & " close on nothing, " & Codec.Img (St.Landed_On) & " put a finger down on it, " & Codec.Img (St.Blocked) & " hit something beside it, "
                            & Codec.Img (St.Palm_Hit) & " would have it reach into my palm, " & Codec.Img (St.No_Force) & " cannot make it move that way, "
                            & Codec.Img (St.Over_Ub) & " need more friction than " & To_String (Name) & " gave me before, " & Codec.Img (St.Unreachable) & " out of my reach"));
         return;
      end if;
      Pick := Found (0);
      Note := S ("contacts on " & To_String (Name) & " for " & Asked & ": " & Codec.Img (Feasible) & " of " & Codec.Img (St.Poses) & " hand placements can do it (from "
                 & Codec.Img (Natural (Surf.Length)) & " surface points, top outline from eye " & Codec.Img (Natural (Integer'Max (0, C.Sil_Cam))) & " at " & Len (C, C.Sil_Pitch)
                 & " pitch, expected error " & Len (C, C.Sil_Err) & (if Reprojected then ", re-laid on the surface I touched" else "")
                 & ", sides assumed solid and upright down to the surface); best: contacts " & Len (C, Pick.Width) & " apart, coming in "
                 & Codec.Fmt (Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, -(Pick.Approach (0) * Up (0) + Pick.Approach (1) * Up (1) + Pick.Approach (2) * Up (2))))), 2)
                 & " rad from straight down"
                 & ", closing " & Len (C, Pick.Pre) & " before going down; the normal forces of my contacts add up to at least " & Codec.Fmt (Pick.Squeeze, 2)
                 & " times its weight in the worst case (friction taken as " & Codec.Fmt (St.Mu_Ref, 2) & " against my fingers and against the surface alike)"
                 & "; to come off the surface with my hand it needs friction at least " & Codec.Fmt (Pick.Mu_Worst, 2) & " in the worst case"
                 & (if Mu_Lb > 0.0 then " (" & To_String (Name) & " has come with my hand at " & Codec.Fmt (Mu_Lb, 2) & ")" else " (friction on it not measured yet)"));
      Ok := True;
   end;
end Plan_Contact;
