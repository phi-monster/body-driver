separate (Act)
procedure Take_Silhouette (C : in out Context; F : Plug.Frame; Cam, Arm : Natural; Name : Unbounded_String; P0 : Geom.V3; P0_Up_Sd : Long_Float) is
   Bx : constant Integer := Boxed_By (C, Cam, Name);
   G : constant Geom.Cam_Geo := Geo_Of (C, Cam);
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
   Keep : constant := 2000;   --  最多留这么多个点(点数)
   A2 : constant Integer := Cam_Arm (C, Cam);
   Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam, 0);
   N : constant Geom.V3 := Up_Dir (C);
   Rays : Geom.Sight_Vectors.Vector;
   Pts : Contact.V3_Vectors.Vector;
   Dropped : Natural;
   K : Natural := 0;
   Stride : Positive := 1;
begin
   if Bx < 0 or else (A2 < 0 and then not G.Fixed) then
      return;
   end if;
   declare
      B : constant Boxed_Thing := C.Boxed (Natural (Bx));
      --  我自己的手指像素只在"这只眼长在正走路的这条胳膊上"时才剔:别的眼里我的手随位姿到处走,握区那张手指图不是此刻的
      Self_Px : constant Boolean := A2 = Integer (Arm) and then Z.Valid and then Natural (Z.Fingers.Length) = Cw * Ch;
   begin
      if not B.Seen or else Natural (B.Mask.Length) /= Cw * Ch then
         return;
      end if;
      for I in B.Y0 .. B.Y1 loop
         for J in B.X0 .. B.X1 loop
            if B.Mask (I * Cw + J) then
               K := K + 1;
            end if;
         end loop;
      end loop;
      if K = 0 then
         return;
      end if;
      Stride := Positive'Max (1, Positive (Long_Float'Ceiling (Sqrt (Long_Float (K) / Long_Float (Keep)))));
      for I in B.Y0 .. B.Y1 loop
         if I mod Stride = 0 then
            for J in B.X0 .. B.X1 loop
               if J mod Stride = 0 and then B.Mask (I * Cw + J) and then not (Self_Px and then Z.Fingers (I * Cw + J)) then
                  declare
                     U : constant Long_Float := Long_Float (J);
                     V : constant Long_Float := Long_Float (I);
                  begin
                     if A2 >= 0 then
                        declare
                           P : constant Plug.Arm_Pose := F.EE (Natural (A2));
                        begin
                           Rays.Append (Geom.Sight'(O => Geom.Cam_Pos (G, P), D => Geom.Ray (G, P, U, V)));
                        end;
                     else
                        Rays.Append (Geom.Sight'(O => G.Pos, D => Geom.Ray_Fixed (G, U, V)));
                     end if;
                  end;
               end if;
            end loop;
         end if;
      end loop;
   end;
   --  面过哪一点:它量到的位置(两眼交点)。可它躺在我碰过的那个面上:交点不可能在那个面之下,也不可能比我的张口还高出面(那样我也夹不住它)
   --  —— 两条视线都近乎竖直时交点的深度是病态的(H53 2026-09-23 实测:交点在桌面之下 9–28 cm)。出了这个范围就把面贴回碰过的那一点
   --  (当它厚度为零),并说出来;范围之内照用(那一截就是它的厚度)
   Contact.Surface.On_Plane (Rays, Plane_Point (C, P0, N, Say => True), N, Pts, Dropped);
   if Natural (Pts.Length) < 8 then   --  点数
      return;
   end if;
   declare
      Pitch : constant Long_Float := Contact.Gen.Sampling_Gap (Pts);
      --  这份点的预期误差:那只眼量朝向时的像素残差 ÷ 焦距 × 眼到面的距离(全是量过的数)。H57 2026-09-23 实测:头顶眼残差 11 px、离桌 1 m ⇒ 4 cm,
      --  腕眼 0.3 px、离桌 0.3 m ⇒ 0.2 mm;"留最细的一份"留下了头顶眼那份(3 mm 间距但整片偏了几厘米),三把合空 ⇒ 留【误差最小】的那份
      Eye_O : constant Geom.V3 := (if A2 >= 0 then [F.EE (Natural (A2)) (0), F.EE (Natural (A2)) (1), F.EE (Natural (A2)) (2)] else G.Pos);
      Sp0 : constant Geom.V3 := Pts.First_Element;
      Dist : constant Long_Float := Geom.Norm ([Sp0 (0) - Eye_O (0), Sp0 (1) - Eye_O (1), Sp0 (2) - Eye_O (2)]);
      Err : constant Long_Float := (if G.F > 0.0 then G.Rms * Dist / G.F else Dist);
      --  已有的那份(同一件)只让误差更小(相同则更细)的盖它。面的高度变了不算数:视线存着,碰到面后会按真高度重投
      --  (H60 2026-09-23 实测:交点高度一抖,头顶眼那份 2.5 cm 误差的把腕眼 0.2 mm 的盖掉了)
      Fresh : constant Boolean := C.Sil_Valid and then C.Sil_Name = Name;
   begin
      if Pitch <= 0.0 or else (Fresh and then (Err > C.Sil_Err or else (Err = C.Sil_Err and then Pitch > C.Sil_Pitch))) then
         return;
      end if;
      C.Sil_Pts := Pts; C.Sil_Valid := True; C.Sil_Name := Name; C.Sil_Cam := Integer (Cam); C.Sil_N := N; C.Sil_Pitch := Pitch; C.Sil_Err := Err;
      C.Sil_H_Sd := P0_Up_Sd;
      C.Sil_P0 := Sp0;   --  面过的点:就取这份点里的一个(它们全在那张面上)
      C.Sil_Rays := Rays;
      Geo_Say ("第" & Codec.Img (Cam) & " 台眼看全了它 ⇒ 记下它顶面的 " & Codec.Img (Natural (Pts.Length)) & " 个点(轮廓像素隔 " & Codec.Img (Stride)
               & " 个取一个,落到它躺的面上,采样间距 " & Mm (Pitch) & ",预期误差 " & Mm (Err) & ";" & Codec.Img (Dropped) & " 条视线落不到面上)");
   end;
end Take_Silhouette;
