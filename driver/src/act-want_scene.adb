separate (Act)
procedure Want_Scene (C : in out Context; F : Plug.Frame; W : Want; Arm : Integer; Sc : out Contact.Qty.Scene) is
   Up : constant Geom.V3 := Up_Dir (C);
   Sp : constant Geom.V3 := Lie_P (C);
   Name : constant Unbounded_String := (if W.Thing >= 1 and then W.Thing <= Natural (C.Items.Length) then Item_Name (C, W.Thing) else Null_Unbounded_String);
   Now_Pts : Contact.V3_Vectors.Vector;
   Var : Long_Float := C.Map.EE_Noise ** 2;
   function H_Of (Q : Geom.V3) return Long_Float is ((Q (0) - Sp (0)) * Up (0) + (Q (1) - Sp (1)) * Up (1) + (Q (2) - Sp (2)) * Up (2));
   --  它此刻的实心模型(世界系表面点):拿着它 ⇒ 合上那一刻的那份按手从那一刻起挪过的刚体运动带过来(拿住 = 抬一点它跟着手走,量过的);
   --  没拿着 ⇒ Solid_Of(记下的顶面轮廓往下补到它躺的面,同接触集);都没有 ⇒ 空。
   --  Rest = 它还躺在面上时的那一份(拿着的 = 合上那一刻的;没拿着的就是 Shape):它的底离面多高 = 此刻的最低点比躺着时的最低点高出多少
   --  (模型的侧壁补到离面不到一个采样间距处为止,躺着时的最低点本来就在面上方一点 —— 量"抬起多高"要从那儿算)
   procedure Shape_Now (Shape, Rest : out Contact.V3_Vectors.Vector) is
      Rp : Boolean;
      pragma Warnings (Off, Rp);   --  重投过没有只在接触集的那句话里说
   begin
      Shape.Clear;
      Rest.Clear;
      if Arm >= 0 and then C.Wld.Holding and then C.Wld.Held_Arm = Arm and then not C.Held_Shape.Is_Empty and then Natural (Arm) < Natural (F.EE.Length) then
         declare
            P0 : constant Plug.Arm_Pose := C.Held_Pose;
            P1 : constant Plug.Arm_Pose := F.EE (Natural (Arm));
            Rd : constant Geom.M3 := Geom.Mul (Geom.Quat_To_R (P1), Geom.Tr (Geom.Quat_To_R (P0)));   --  手从合上那一刻起转了多少(世界系)
         begin
            for Q of C.Held_Shape loop
               declare
                  D : constant Geom.V3 := Geom.Ap (Rd, [Q (0) - P0 (0), Q (1) - P0 (1), Q (2) - P0 (2)]);
               begin
                  Shape.Append (Geom.V3'[P1 (0) + D (0), P1 (1) + D (1), P1 (2) + D (2)]);
               end;
            end loop;
         end;
         Rest := C.Held_Shape;
      else
         Solid_Of (C, Name, Shape, Rp);
         Rest := Shape;
      end if;
   end Shape_Now;
   Rest_Pts : Contact.V3_Vectors.Vector;
begin
   Sc := (others => <>);
   Sc.Up := Up;
   Shape_Now (Now_Pts, Rest_Pts);
   if not Now_Pts.Is_Empty then
      declare
         N : constant Long_Float := Long_Float (Now_Pts.Length);
         Cm : Geom.V3 := [others => 0.0];
         Lo, Lo0 : Long_Float := Long_Float'Last;
         Oe : Boolean;
         Seed : constant Geom.V3 := (if abs Up (0) < abs Up (1) then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);
         E1 : constant Geom.V3 := Contact.Unit (Contact.Cross (Up, Seed), Oe);
         E2 : constant Geom.V3 := Contact.Cross (Up, E1);
         Sxx, Sxy, Syy : Long_Float := 0.0;
      begin
         for Q of Now_Pts loop
            for K in 0 .. 2 loop
               Cm (K) := Cm (K) + Q (K) / N;
            end loop;
            Lo := Long_Float'Min (Lo, H_Of (Q));
         end loop;
         for Q of Rest_Pts loop
            Lo0 := Long_Float'Min (Lo0, H_Of (Q));
         end loop;
         Sc.Center := Cm; Sc.Has_Center := True;
         Sc.Bottom := Lo - Lo0; Sc.Has_Bottom := True;
         for Q of Now_Pts loop
            declare
               D : constant Geom.V3 := [Q (0) - Cm (0), Q (1) - Cm (1), Q (2) - Cm (2)];
               A : constant Long_Float := Contact.Dot (D, E1);
               B : constant Long_Float := Contact.Dot (D, E2);
            begin
               Sxx := Sxx + A * A / N; Sxy := Sxy + A * B / N; Syy := Syy + B * B / N;
            end;
         end loop;
         declare
            Th : constant Long_Float := 0.5 * Arctan (2.0 * Sxy, Sxx - Syy);
            Rad : constant Long_Float := Sqrt (0.25 * (Sxx - Syy) ** 2 + Sxy ** 2);
            L1 : constant Long_Float := 0.5 * (Sxx + Syy) + Rad;
         begin
            if Oe and then Rad > 0.0 then
               Sc.Ang_Sd := C.Sil_Err * Sqrt (L1 / N) / (Rad + Rad);
               if Stats.Z * Sc.Ang_Sd < 0.5 * Ada.Numerics.Pi then
                  Sc.Axis := [Cos (Th) * E1 (0) + Sin (Th) * E2 (0), Cos (Th) * E1 (1) + Sin (Th) * E2 (1), Cos (Th) * E1 (2) + Sin (Th) * E2 (2)];
                  Sc.Has_Axis := True;
               end if;
            end if;
         end;
         Var := Var + C.Sil_Err ** 2 + (if C.Sil_H_Sd < Long_Float'Last then C.Sil_H_Sd ** 2 else 0.0);
      end;
   elsif C.Geo_Pw_Valid and then C.Geo_Pw_Name = Name then
      Sc.Center := C.Geo_Pw; Sc.Has_Center := True;
      if C.Geo_Pw_Up_Sd < Long_Float'Last then
         Var := Var + C.Geo_Pw_Up_Sd ** 2;
      end if;
   end if;
   if Arm >= 0 then
      declare
         Sc_Cam : constant Integer := Still_Cam (C, F, Natural (Arm));
      begin
         if Sc_Cam >= 0 and then Natural (Sc_Cam) < Natural (C.Geo.Length) then
            declare
               Gm : constant Geom.Cam_Geo := C.Geo (Natural (Sc_Cam));
               A2 : constant Integer := Cam_Arm (C, Natural (Sc_Cam));
            begin
               if A2 < 0 and then Gm.Fixed then
                  Sc.Me := Gm.Pos; Sc.Has_Me := True;
               elsif A2 >= 0 and then Gm.Valid and then Natural (A2) < Natural (F.EE.Length) then
                  Sc.Me := Geom.Cam_Pos (Gm, F.EE (Natural (A2))); Sc.Has_Me := True;
               end if;
            end;
         end if;
      end;
   end if;
   declare
      Gv : constant Geom.Cam_Geo := Geo_Of (C, C.Cam);
      Av : constant Integer := Cam_Arm (C, C.Cam);
   begin
      if Av < 0 and then Gv.Fixed then
         Sc.View_Right := Geom.Ap (Gv.R_Ce, [1.0, 0.0, 0.0]); Sc.Has_View := True;
      elsif Av >= 0 and then Gv.Valid and then Natural (Av) < Natural (F.EE.Length) then
         Sc.View_Right := Geom.Ap (Geom.Cam_R (Gv, F.EE (Natural (Av))), [1.0, 0.0, 0.0]); Sc.Has_View := True;
      end if;
   end;
   if W.Ref >= 1 and then W.Ref <= Natural (C.Items.Length) then
      declare
         Who : Unbounded_String;
         Sds : Floats;
         Rays : constant Geom.Sight_Vectors.Vector :=
           Sightlines_Now (C, F, C.Map.N_Cams, Natural (Integer'Max (0, Arm)), Item_Name (C, W.Ref), False, False, 0.0, 0.0, Who, Sds);
         Mok : Boolean;
         Spread : Long_Float;
         Pm : constant Geom.V3 := Geom.Meet (Rays, Mok, Spread);
      begin
         if Mok then
            Sc.Ref := Pm; Sc.Has_Ref := True;
            declare
               Su : constant Long_Float := Geom.Meet_Sd (Rays, Sds, Pm, Up);
            begin
               if Su < Long_Float'Last then
                  Sc.Ref_Top := H_Of (Pm); Sc.Has_Ref_Top := True;
                  Var := Var + Su ** 2;
               end if;
            end;
         end if;
      end;
   end if;
   Sc.Sd := Sqrt (Var);
end Want_Scene;
