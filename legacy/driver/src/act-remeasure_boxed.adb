with Things;
with Jointboot;
with Stats;
separate (Act)
procedure Remeasure_Boxed (C : in out Context; F : Plug.Frame; Cam : Natural; Regs : in out Picture.Regions) is
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
   --  这只眼长在哪条胳膊上(问身体图;不长在胳膊上 ⇒ -1)
   function Eye_Arm return Integer is
   begin
      for A in 0 .. Selfmap.Graph.Arm_Count (C.Map) - 1 loop
         if Selfmap.Graph.Eyes_On (C.Map, A).Contains (Integer (Cam)) then
            return Integer (A);
         end if;
      end loop;
      return -1;
   end Eye_Arm;
   --  给分割的框比上一次量到的框四面各让出它边上抖的 Z 倍(量的,至少一个像素):它没动、也没被切着时,这一次的掩膜就不会顶到框边;
   --  顶到了才是它有一截在框外(不让这一圈,掩膜永远顶着自己上一次的外接框,每一帧都得多分割一次)
   Margin : constant Natural := Natural (Long_Float'Ceiling (Stats.Z * Things.Edge_Sd (Cam)));
begin
   for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
      if C.Boxed (Bi).Cam = Cam and then not C.Boxed (Bi).Blind then   --  脑说"这只眼里没有它"的那一条没有框可量
         declare
            B : Boxed_Thing := C.Boxed (Bi);
            Found, Iso : Boolean;
            R : Picture.Region;
            --  这一次给分割的窗(闭区间):它的掩膜只在窗里作数
            Wx0 : Natural := (if B.X0 > Margin then B.X0 - Margin else 0);
            Wy0 : Natural := (if B.Y0 > Margin then B.Y0 - Margin else 0);
            Wx1 : Natural := Natural'Min (Cw - 1, B.X1 + Margin);
            Wy1 : Natural := Natural'Min (Ch - 1, B.Y1 + Margin);
         begin
            Seg_In_Box (C, F, Cam, Wx0, Wy0, Wx1, Wy1, Found, Iso, R, B.Mask, B.Pu_On, B.Pv_On);
            --  量到的那一块顶到了窗边(不是画幅边)= 它有一截在窗外:窗往顶着的那几边各长出它自己那么宽 / 高,带着它身上那一点再量,
            --  直到不再顶着窗边、或者长不出新的像素。新的掩膜得盖住旧掩膜的里头才算同一件(盖不住 = 分割跳到了别的东西上,留旧的)。
            --  (10-01 路 5 量到:C1 第 102 拍当场的窗只盖住鼠标的 28%,S1A5 只有剪刀转轴那一段 —— 下一帧的框 = 这一帧掩膜的外接框,
            --  分割又不出框,框只缩不长;原来这里判的是"顶没顶到画幅边",窗切着东西时从不长。我一动它平移出窗的那一截也是同一回事)
            loop
               exit when not Found;
               declare
                  Nx0 : Natural := Wx0;
                  Ny0 : Natural := Wy0;
                  Nx1 : Natural := Wx1;
                  Ny1 : Natural := Wy1;
                  Grew : Boolean;
                  Pu, Pv : Long_Float;
                  F2, I2 : Boolean;
                  R2 : Picture.Region;
                  M2 : Bools;
               begin
                  Picture.Grow_Window (R, Cw, Ch, Nx0, Ny0, Nx1, Ny1, Grew);
                  exit when not Grew;
                  On_Pixel (B.Mask, Cw, Ch, R, Pu, Pv);
                  Seg_In_Box (C, F, Cam, Nx0, Ny0, Nx1, Ny1, F2, I2, R2, M2, Pu, Pv);
                  exit when not F2 or else not Picture.Covers_Interior (M2, B.Mask, Cw, Ch, R);
                  Wx0 := Nx0; Wy0 := Ny0; Wx1 := Nx1; Wy1 := Ny1;
                  exit when R2.X0 = R.X0 and then R2.Y0 = R.Y0 and then R2.X1 = R.X1 and then R2.Y1 = R.Y1 and then R2.Count = R.Count;
                  R := R2; Iso := I2; B.Mask := M2;
               end;
            end loop;
            --  🔴 量到的那一块还是不是它:拿它和周围的明暗【哪边亮】对。脑指它那一帧记下"它比周围亮还是暗";这一帧量到的块要是反过来了,
            --  那是别的东西(H31 2026-09-22 实测:预测窗漂到另一只手的黑爪子上,框里"最大的一块"就成了爪子,视线交点算到 14 cm 高的空中)。
            --  只比方向不比幅度:H32 实测手的影子一盖,剪刀从 216 暗到 165(背景 127),按幅度就把真剪刀判成了别的东西。认不出就老实说看不见,不许锁错。
            if Found and then B.Gray >= 0.0 and then B.Bg >= 0.0 then
               declare
                  Tg, Bk : Long_Float;
               begin
                  Blob_Levels (F.Cams (Cam).Gray, Cw, Ch, B.Mask, R, Tg, Bk);
                  --  09-13 总规矩:动起来之后身体不许有闸 ⇒ 说出来、照走(2026-09-26 以前这里判"不是它,算看不见")
                  if Tg >= 0.0 and then Bk >= 0.0 and then (Tg - Bk) * (B.Gray - B.Bg) <= 0.0 and then B.Seen then
                     Put_Line ("[身] 📦 " & To_String (B.Name) & "(第" & Codec.Img (Cam) & " 台):框里量到的那块平均亮 "
                               & Codec.Fmt (Tg, 0) & "、周围 " & Codec.Fmt (Bk, 0) & ",它当初 " & Codec.Fmt (B.Gray, 0) & "、周围 " & Codec.Fmt (B.Bg, 0)
                               & " ⇒ 明暗反了(可能不是它,也可能是影子盖住了);照这块跟");
                  end if;
               end;
            end if;
            --  🔴 量到的那一块还是不是它,第二样:大小。窗挪到预测处时它该有多少像素是算过的(上一次的像素数 × 远近比例的平方);
            --  这一帧量到的是单独的一块、却不到预期的四分之一(线尺寸的一半,纯数学)⇒ 那是窗底下的别的东西,不是它
            --  (H61 2026-09-23 实测:交点算深了 14 cm,窗一路漂到桌面上,540 px 的一小块桌纹当成了 7000 px 的剪刀,合空)。
            --  顶着窗边的块不判(它可能只露了一截);更大也不判(它可能刚露全)
            if Found and then Iso and then B.Count > 0 and then R.Count * 4 < B.Count and then B.Seen then   --  说出来、照走(同上)
               Put_Line ("[身] 📦 " & To_String (B.Name) & "(第" & Codec.Img (Cam) & " 台):框里量到的那块只有 " & Codec.Img (R.Count)
                         & " px,它该有约 " & Codec.Img (B.Count) & " px ⇒ 小得不像它(可能不是它);照这块跟");
            end if;
            B.Seen := Found;
            if Found then
               B.X0 := R.X0; B.Y0 := R.Y0; B.X1 := R.X1; B.Y1 := R.Y1;
               B.Cu := R.Cu; B.Cv := R.Cv; B.Isolated := Iso; B.Count := R.Count;
               On_Pixel (B.Mask, Cw, Ch, R, B.Pu_On, B.Pv_On);
               --  它身上的碎片:形心落在它框里的那些块,由这一整块顶替
               for Ri in reverse 0 .. Natural (Regs.Length) - 1 loop
                  if Picture.Inside (R, Regs (Ri).Cu, Regs (Ri).Cv, Cw, Ch, 0.0) then
                     Regs.Delete (Ri);
                  end if;
               end loop;
               --  调用方都拿 (0) 当最大的一块 ⇒ 按像素数插回去,不许打乱从多到少的次序
               declare
                  At_I : Natural := Natural (Regs.Length);
               begin
                  for Ri in 0 .. Natural (Regs.Length) - 1 loop
                     if Regs (Ri).Count < R.Count then
                        At_I := Ri;
                        exit;
                     end if;
                  end loop;
                  if At_I >= Natural (Regs.Length) then
                     Regs.Append (R);
                  else
                     Regs.Insert (At_I, R);
                  end if;
               end;
               --  这一眼交给它的估计(I4,Things):那一刻这只眼在世界里的位姿、这一次的窗、窗里它的像素、
               --  也许挡着它的像素(这只眼长在的那只手的手指、这只眼里别的东西的像素)。
               --  🔴 还缺一样(要主代理的 act.adb 加一行):别的手压在它上面的那一眼不该交(那一片是被挡住,不是"不是它")——
               --  路 1 的 Hand_Covers 声明在这个分开编译的桩后面,这里看不见它;等它前移(或身体零件 I7 能投进每只眼当挡着的像素)再接上
               declare
                  G : constant Geom.Cam_Geo := (if Cam < Natural (C.Geo.Length) then C.Geo (Cam) else Geom.No_Geo);
                  A : constant Integer := Eye_Arm;
                  Usable : constant Boolean := G.Valid and then G.F > 0.0
                    and then (if A < 0 then G.Fixed else A < Integer (F.EE.Length));
               begin
                  if Usable then
                     declare
                        V : Things.View;
                        E : Things.Estimate := Things.Get (To_String (B.Name));
                     begin
                        V.Cam := G;
                        if A >= 0 then
                           V.Cam.R_Ce := Geom.Cam_R (G, F.EE (Natural (A))); V.Cam.Pos := Geom.Cam_Pos (G, F.EE (Natural (A)));
                           V.Cam.Off := [0.0, 0.0, 0.0]; V.Cam.Fixed := True;
                           declare
                              Z : constant Zone.Hand_Zone := Zone_Of (C, Natural (A), Cam);
                           begin
                              if Z.Valid and then Natural (Z.Fingers.Length) = Cw * Ch then
                                 V.Occl := Z.Fingers;
                              end if;
                           end;
                        end if;
                        --  别的东西在这只眼里的像素也可能挡着它(它的一截在别的东西后面):一起算挡着的
                        for Bj in 0 .. Natural (C.Boxed.Length) - 1 loop
                           if Bj /= Bi and then C.Boxed (Bj).Cam = Cam and then C.Boxed (Bj).Seen and then C.Boxed (Bj).Name /= B.Name
                             and then Natural (C.Boxed (Bj).Mask.Length) = Cw * Ch
                           then
                              if Natural (V.Occl.Length) /= Cw * Ch then
                                 V.Occl := C.Boxed (Bj).Mask;
                              else
                                 for I in 0 .. Cw * Ch - 1 loop
                                    if C.Boxed (Bj).Mask (I) then
                                       V.Occl (I) := True;
                                    end if;
                                 end loop;
                              end if;
                           end if;
                        end loop;
                        V.Cam_Index := Cam; V.Px_Sd := G.Rms; V.Pos_Sd := (if A >= 0 then Jointboot.Arm_Sd (Natural (A)) else G.Pos_Sd);
                        V.W := Cw; V.H := Ch; V.X0 := Wx0; V.Y0 := Wy0; V.X1 := Wx1; V.Y1 := Wy1; V.Mask := B.Mask; V.Seq := F.Seq;
                        if C.Touch_Valid then
                           Things.Set_Support (E, C.Touch_Pt, C.Touch_N);
                        elsif C.Board_Plane then
                           Things.Set_Support (E, C.Board_Pt, C.Board_N);
                        end if;
                        Things.Add_View (E, V);
                        Things.Put (E);
                     end;
                  end if;
               end;
            end if;
            C.Boxed.Replace_Element (Bi, B);
         end;
      end if;
   end loop;
end Remeasure_Boxed;
