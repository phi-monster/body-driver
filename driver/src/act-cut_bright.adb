separate (Act)
function Cut_Bright (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
   G : constant Buf := F.Cams (Cam).Gray;
   Samp : Floats;
   T : Long_Float;
   T_First : Long_Float := 0.0;
   T_Low_Keep : Long_Float := 0.0;   --  暗刀的分界,中间那一段要用它当下沿
   Mask : Bools;
   Out_R : Picture.Regions;
   I : Natural := 0;
begin
   if Natural (G.Length) < Cw * Ch then
      return Out_R;
   end if;
   --  分界按每一个像素算(09-30:原来每 7 个取一个 —— 新的 Split 要按置信界证出一道真谷,样本少了证不出,
   --  头顶眼里桌上的东西和桌面并成一块丢掉;Split 按直方图算,全像素也只多一遍直方图)
   while I < Cw * Ch loop
      Samp.Append (Long_Float (G.Element (I)));
      I := I + 1;
   end loop;
   T := Picture.Split (Samp);
   if Picture.Is_Nan (T) then
      return Out_R;   --  全一样 ⇒ 这只眼里按明暗切不出东西,如实交空
   end if;
   T_First := T;
   --  🔴 分两次:第一刀分的是"暗桌面 vs 亮的一切"(NJK 存图离线:分界 111,浅色木纹和白球并成一块);
   --  在亮的那一拨里再分一刀,才把最亮的一撮(白球、乐高的黄)从浅木纹里切出来。两刀的分界都是算出来的。
   declare
      Upper : Floats;
      T2 : Long_Float;
   begin
      for X of Samp loop
         if X > T then
            Upper.Append (X);
         end if;
      end loop;
      T2 := Picture.Split (Upper);
      if not Picture.Is_Nan (T2) then
         T := T2;
      end if;
   end;
   --  🔴 暗的东西也要认(黑键盘、红乐高、深色把手):在暗的那一拨里再分一刀,最暗的一撮单独成块;
   --  贴着画面边的暗块是我自己的胳膊/手指(它们从画面外伸进来),丢掉
   declare
      Lower : Floats;
      T_Low : Long_Float;
      Dark : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
   begin
      for X of Samp loop
         if X <= T_First then
            Lower.Append (X);
         end if;
      end loop;
      T_Low := Picture.Split (Lower);
      T_Low_Keep := (if Picture.Is_Nan (T_Low) then 0.0 else T_Low);
      if not Picture.Is_Nan (T_Low) then
         for J in 0 .. Cw * Ch - 1 loop
            if Long_Float (G.Element (J)) < T_Low then
               Dark.Replace_Element (J, True);
            end if;
         end loop;
         for R of Picture.Components (Dark, Cw, Ch, Picture.Min_Pixels (Cw, Ch)) loop
            declare
               Q : Picture.Region := R;
               Edge : constant Boolean := R.X0 = 0 or else R.Y0 = 0 or else R.X1 + 1 >= Cw or else R.Y1 + 1 >= Ch;
            begin
               if not Edge then
                  Q.Height := 0.0; Q.Depth := 0.0;   --  main 的 Region 没有 Top 这一位
                  Out_R.Append (Q);
               end if;
            end;
         end loop;
      end if;
   end;
   --  🔴🔴 中间那一段以前整个当桌面扔了 —— 而"不亮不暗"的东西正好落在那儿。
   --  SC1 实测(头顶眼):剪刀那一片中位 95、桌面中位 135 —— 剪刀【比桌子暗】,
   --  却又没暗过暗刀的分界,于是亮刀和暗刀都不要它,画面上一个框都没有,
   --  脑连它的号都拿不到 ⇒ 点不了名 ⇒ 一步都动不了(验收线 1「剪刀」因此一次都没试成)。
   --  ⇒ 中间这一段再分一刀(还是 Otsu,和上下两刀同一个办法,不新加门槛):
   --    分完两拨里【少的那一拨】是东西,多的那一拨是桌面 —— 桌子总是占大头,这是数出来的,不是我拍的。
   declare
      Mid : Floats;
      T_Mid : Long_Float;
      N_Lo, N_Hi : Natural := 0;
      Thing_Is_Darker : Boolean;
      Band : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
   begin
      for X of Samp loop
         if X > T_Low_Keep and then X <= T then
            Mid.Append (X);
         end if;
      end loop;
      T_Mid := Picture.Split (Mid);
      if not Picture.Is_Nan (T_Mid) then
         for X of Mid loop
            if X <= T_Mid then
               N_Lo := N_Lo + 1;
            else
               N_Hi := N_Hi + 1;
            end if;
         end loop;
         Thing_Is_Darker := N_Lo < N_Hi;
         for J in 0 .. Cw * Ch - 1 loop
            declare
               V : constant Long_Float := Long_Float (G.Element (J));
            begin
               if V > T_Low_Keep and then V <= T
                 and then ((Thing_Is_Darker and then V <= T_Mid)
                           or else (not Thing_Is_Darker and then V > T_Mid))
               then
                  Band.Replace_Element (J, True);
               end if;
            end;
         end loop;
         for R of Picture.Components (Band, Cw, Ch, Picture.Min_Pixels (Cw, Ch)) loop
            declare
               Q : Picture.Region := R;
               Span_W : constant Boolean := R.X0 = 0 and then R.X1 + 1 >= Cw;
               Span_H : constant Boolean := R.Y0 = 0 and then R.Y1 + 1 >= Ch;
            begin
               if not Span_W and then not Span_H then
                  Q.Height := 0.0; Q.Depth := 0.0;
                  Out_R.Append (Q);
               end if;
            end;
         end loop;
      end if;
   end;
   Mask := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
   for J in 0 .. Cw * Ch - 1 loop
      if Long_Float (G.Element (J)) > T then
         Mask.Replace_Element (J, True);
      end if;
   end loop;
   Last_Bright := Mask;   --  留给接触集扫轮廓用(同一张掩膜,不另切一遍)
   Last_Bright_Cam := Integer (Cam);
   Last_Split := T;
   for R of Picture.Components (Mask, Cw, Ch, Picture.Min_Pixels (Cw, Ch)) loop
      declare
         Q : Picture.Region := R;
         Span_W : constant Boolean := R.X0 = 0 and then R.X1 + 1 >= Cw;
         Span_H : constant Boolean := R.Y0 = 0 and then R.Y1 + 1 >= Ch;
      begin
         if not Span_W and then not Span_H then
            Q.Height := 0.0;   --  main 的 Region 没有 Top 这一位
            if F.Cams (Cam).Has_Depth then
               declare
                  --  读深窗口 = 这块自己最窄边的四分之一,再小也有半个百分点的画幅(比例,无量纲);只当记录
                  Zd : constant Long_Float := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, R.Cu, R.Cv,
                                                                 Long_Float'Max (0.005, 0.25 * Long_Float'Min (Long_Float (R.X1 - R.X0 + 1) / Long_Float (Cw),
                                                                                                              Long_Float (R.Y1 - R.Y0 + 1) / Long_Float (Ch))));
               begin
                  Q.Depth := (if Picture.Is_Nan (Zd) then 0.0 else Zd);
               end;
            end if;
            Out_R.Append (Q);
         end if;
      end;
   end loop;
   return Out_R;
end Cut_Bright;
