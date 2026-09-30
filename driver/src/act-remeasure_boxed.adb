separate (Act)
procedure Remeasure_Boxed (C : in out Context; F : Plug.Frame; Cam : Natural; Regs : in out Picture.Regions) is
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
begin
   for Bi in 0 .. Natural (C.Boxed.Length) - 1 loop
      if C.Boxed (Bi).Cam = Cam and then not C.Boxed (Bi).Blind then   --  脑说"这只眼里没有它"的那一条没有框可量
         declare
            B : Boxed_Thing := C.Boxed (Bi);
            Found, Iso : Boolean;
            R : Picture.Region;
         begin
            Seg_In_Box (C, F, Cam, B.X0, B.Y0, B.X1, B.Y1, Found, Iso, R, B.Mask, B.Pu_On, B.Pv_On);
            --  我一动,长在我手上的眼里它会平移一截(GB5:横挪 25.6 mm,它从 u=288 跳到 260)。
            --  量到的那一块顶到了窗边 = 它有一部分在窗外 ⇒ 把窗挪到【量到的这一块】身上再量,直到整块落进窗里或不再变。
            --  还是同一个量法,只是跟着它走;最多跟 4 回(次数)。
            for Again in 1 .. 4 loop
               exit when not Found or else Iso;
               declare
                  F2, I2 : Boolean;
                  R2 : Picture.Region;
                  M2 : Bools;
               begin
                  Seg_In_Box (C, F, Cam, R.X0, R.Y0, R.X1, R.Y1, F2, I2, R2, M2, B.Pu_On, B.Pv_On);
                  exit when not F2 or else (R2.X0 = R.X0 and then R2.Y0 = R.Y0 and then R2.X1 = R.X1 and then R2.Y1 = R.Y1);
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
            end if;
            C.Boxed.Replace_Element (Bi, B);
         end;
      end if;
   end loop;
end Remeasure_Boxed;
