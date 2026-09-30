separate (Act)
procedure Plan_Contact (C : in out Context; F : Plug.Frame; Arm, Cam : Natural; Name : Unbounded_String;
                        Pick : out Contact.Grasp.Candidate; Note : out Unbounded_String; Ok : out Boolean) is
   G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
   Have_Plane : constant Boolean := C.Touch_Valid or else C.Board_Plane;
   Up : constant Geom.V3 := (if C.Touch_Valid then C.Touch_N elsif C.Board_Plane then C.Board_N else Protocol_Up);
   Sp : constant Geom.V3 := (if C.Touch_Valid then C.Touch_Pt else C.Board_Pt);
   Tol_P : constant Long_Float := Geo_Base (C, Arm);
   Tol_R : constant Long_Float := (if Arm * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (Arm * Chan.Per_Arm + 3) else 0.0);
   Mu_Lb : Long_Float := 0.0;
   Mu_Ub : Long_Float := Long_Float'Last;
   H : Contact.Grasp.Hand_Model;
   Top : Contact.V3_Vectors.Vector := C.Sil_Pts;
   Surf, Around : Contact.V3_Vectors.Vector;
   Found : Contact.Grasp.Cand_Vectors.Vector;
   St : Contact.Grasp.Plan_Stats;
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
   if Natural (G.Lobes.Length) /= 2 then
      Note := S ("my fingers in this eye are " & Codec.Img (Natural (G.Lobes.Length)) & " measured pads (I lay out holds for two pads closing on each other"
                 & (if G.Lobes.Is_Empty then "; my body file has no per-finger tips, it has to be measured once from scratch" else "") & ")");
      return;
   end if;
   H := Contact.Grasp.Two_Pads (G.Lobes (0).Tip, G.Lobes (1).Tip, Long_Float'Min (G.Lobes (0).Wide, G.Lobes (1).Wide),
                                Long_Float'Max (G.Lobes (0).Thin, G.Lobes (1).Thin), Long_Float'Max (G.Tip_Sd, Tol_P));
   if not H.Valid then
      Note := S ("my measured fingers do not make a hand I can lay a hold out with: " & To_String (H.Why));
      return;
   end if;
   for Gm of C.Grip_Mus loop
      if Gm.Name = Name then
         Mu_Lb := Gm.Lb; Mu_Ub := Gm.Ub;
      end if;
   end loop;
   --  取轮廓时面的高度可能只是交点估的;碰过它躺的面 ⇒ 按真的面重投那些视线
   if C.Touch_Valid and then not C.Sil_Rays.Is_Empty then
      declare
         P0 : constant Geom.V3 := Plane_Point (C, C.Sil_P0, C.Sil_N, Say => False);
         Dropped : Natural;
         Again : Contact.V3_Vectors.Vector;
      begin
         if Geom.Norm ([P0 (0) - C.Sil_P0 (0), P0 (1) - C.Sil_P0 (1), P0 (2) - C.Sil_P0 (2)]) > C.Sil_Pitch then
            Contact.Surface.On_Plane (C.Sil_Rays, P0, C.Sil_N, Again, Dropped);
            if Natural (Again.Length) >= 8 then   --  点数
               Top := Again;
               Reprojected := True;
            end if;
         end if;
      end;
   end if;
   Contact.Surface.Walls_To_Support (Top, Up, Sp, C.Sil_Pitch, Surf);
   --  旁边的东西:被顶住过、比面高、离它自己的表面点超过两个采样间距的(贴着它的那些是它自己被顶住)
   for B of C.Bumps loop
      declare
         Near : Boolean := False;
      begin
         for Q of Surf loop
            if Geom.Norm ([B (0) - Q (0), B (1) - Q (1), B (2) - Q (2)]) <= 2.0 * C.Sil_Pitch then   --  两个采样间距(纯几何:对角邻居在 √2 个以内)
               Near := True;
               exit;
            end if;
         end loop;
         if not Near then
            Around.Append (B);
         end if;
      end;
   end loop;
   Contact.Grasp.Plan (Surf, Around, C.Sil_Pitch, C.Sil_Err, Up, Sp, H, Mu_Lb, G.Gap, Reach'Access, 8, Found, St);   --  留前 8 组(个数)
   --  量到的摩擦上限:这件东西以前没拿住过的那一组要的摩擦它给不起 ⇒ 要得比它还多的不要
   declare
      Kept : Contact.Grasp.Cand_Vectors.Vector;
   begin
      for Cd of Found loop
         if Cd.Mu_Nom < Mu_Ub then
            Kept.Append (Cd);
         end if;
      end loop;
      if Kept.Is_Empty then
         Note := S ("from " & Codec.Img (Natural (Surf.Length)) & " surface points of " & To_String (Name) & " (top outline from eye " & Codec.Img (Natural (Integer'Max (0, C.Sil_Cam)))
                    & " pulled straight down to the surface it lies on, which assumes solid upright sides) I tried " & Codec.Img (St.Poses) & " placements of my hand: "
                    & Codec.Img (St.Air) & " close on nothing, " & Codec.Img (St.Landed_On) & " put a finger down on it, " & Codec.Img (St.Blocked) & " hit something beside it, "
                    & Codec.Img (St.Palm_Hit) & " push it into my palm, " & Codec.Img (St.No_Hold) & " cannot hold it up, " & Codec.Img (St.Unreachable) & " out of my reach"
                    & (if Natural (Found.Length) > 0 then ", and every hold left needs more friction than " & To_String (Name) & " gave me before" else ""));
         return;
      end if;
      Pick := Kept (0);
      Note := S ("hold on " & To_String (Name) & ": " & Codec.Img (Natural (Kept.Length)) & " holds kept of " & Codec.Img (St.Poses) & " hand placements (from "
                 & Codec.Img (Natural (Surf.Length)) & " surface points, top outline from eye " & Codec.Img (Natural (Integer'Max (0, C.Sil_Cam))) & " at " & Len (C, C.Sil_Pitch)
                 & " pitch, expected error " & Len (C, C.Sil_Err) & (if Reprojected then ", re-laid on the surface I touched" else "")
                 & ", sides assumed solid and upright down to the surface); best: fingers " & Len (C, Pick.Width) & " apart, coming in "
                 & Codec.Fmt (Arccos (Long_Float'Max (-1.0, Long_Float'Min (1.0, -(Pick.Approach (0) * Up (0) + Pick.Approach (1) * Up (1) + Pick.Approach (2) * Up (2))))), 2)
                 & " rad from straight down"
                 & ", closing " & Len (C, Pick.Pre) & " before going down, needs friction at least " & Codec.Fmt (Pick.Mu_Worst, 2) & " in the worst case"
                 & (if Mu_Lb > 0.0 then " (" & To_String (Name) & " has held at " & Codec.Fmt (Mu_Lb, 2) & ")" else " (friction on it not measured yet)"));
      Ok := True;
   end;
end Plan_Contact;
