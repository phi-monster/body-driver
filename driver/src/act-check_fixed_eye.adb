separate (Act)
procedure Check_Fixed_Eye (F : Plug.Frame; C : in out Context) is
   Wc : constant Natural := C.Map.World_Cam;
   Deg_Say : constant String := "°";
begin
   if Length (C.Inst_Host) = 0 then
      Say_No_Check;
      return;
   end if;
   if C.Board.Is_Empty or else C.Fixed_Ref.Is_Empty or else Wc >= Natural (C.Geo.Length) or else Wc >= Natural (F.Cams.Length)
     or else not (C.Geo (Wc).Valid and then C.Geo (Wc).Fixed) or else F.Cams (Wc).W = 0
   then
      return;
   end if;
   declare
      Q : Instrument.Match_Vectors.Vector;
      Err : Unbounded_String;
      Now : Geom.Scene_Pt_Vectors.Vector;
      G : Geom.Cam_Geo := C.Geo (Wc);
      R : Geom.Fixed_Check;
      Ok : Boolean;
      Turned : Natural := 0;   --  此刻的图转回几个 90° 才配上的
      --  此刻的图先顺时针转 Turns 个 90°(Turn_90)再配;配到的像素一步步换算回没转的画面:转一步前高 h 的图里 (u, v) ← 转后的 (u', v') = (v', h − u')(连续坐标,见 Unturn)
      function Matched (Turns : Natural; Got : out Boolean) return Geom.Scene_Pt_Vectors.Vector is
         Img : Buf := F.Cams (Wc).RGB;
         W : Natural := F.Cams (Wc).W;
         H : Natural := F.Cams (Wc).H;
         M : Instrument.Match_Vectors.Vector;
         Res : Geom.Scene_Pt_Vectors.Vector;
      begin
         for T in 1 .. Turns loop
            Img := Turn_90 (Img, W, H);
            declare
               W0 : constant Natural := W;
            begin
               W := H; H := W0;
            end;
         end loop;
         --  往返配(同扫描、对齐):配过去再配回来 1 px 以内才算看见,挡住的那一块仪器编出来的点配不回来(见 Geom.Trip_Px)
         M := Instrument.Match (To_String (C.Inst_Host), C.Inst_Port, C.Fixed_Ref, C.Fixed_Ref_W, C.Fixed_Ref_H, Img, W, H, Q, Err, Back => True);
         Got := Natural (M.Length) = Natural (Q.Length);
         if not Got then
            return Res;
         end if;
         for I in 0 .. Natural (M.Length) - 1 loop
            declare
               P : Geom.Scene_Pt := C.Board (I);
               In_Pic : constant Boolean := M (I).U >= 0.0 and then M (I).V >= 0.0 and then M (I).U < Long_Float (W) and then M (I).V < Long_Float (H)
                 and then Geom.Round_Trip_Ok (Q (I).U, Q (I).V, M (I).Bu, M (I).Bv);
               U, V : Long_Float;
            begin
               Unturn (M (I).U, M (I).V, Turns, F.Cams (Wc).W, F.Cams (Wc).H, U, V);
               P.U := (if In_Pic then U else -1.0); P.V := (if In_Pic then V else -1.0);
               Res.Append (P);
            end;
         end loop;
         return Res;
      end Matched;
   begin
      for S of C.Board loop
         Q.Append (Instrument.Match_Pt'(U => S.U, V => S.V, Cert => 0.0, others => <>));
      end loop;
      declare
         B0 : constant Geom.Fixed_Best := C.Fixed_Best;   --  这一轮之前的"放好以来最多"(别的转法各自从它起算)
      begin
         Now := Matched (C.Fixed_Turn, Ok);
         if not Ok then
            Geo_Say ("核对不动的眼:仪器没配成(" & To_String (Err) & ")⇒ 这一轮不核");
            return;
         end if;
         Geom.Check_Fixed (G, C.Board, Now, C.Fixed_Best, R, Turn_Sd => C.Fixed_Turn_Sd);
         Turned := C.Fixed_Turn;
         --  挪过,或看不全(看不全只在刚变的那一轮、之后隔 1、2、4、8……轮:次数翻倍,挡着的时候也可能被转,代价随挡的时长按对数涨):
         --  此刻的图按四个转法(转 0/90/180/270°)各配一次,挑新门里解释点最多的那个 —— 转正的那个配得最细、点最多
         --  (RoMa 转 90° 配上六成、配得糙,转 180° 一个都配不上;X5E2 2026-09-26 第一个过关的是转 180° 那个,相对还差 90°,采纳了一份差 2.9 cm、4.75 px 的位姿)。
         --  采纳之后每轮就按这个转法配(C.Fixed_Turn),配点一直是转正的精度,细门不被糙解抬高
         if R.Moved or else (R.Covered and then (not C.Fixed_Covered or else C.Round_N >= C.Fixed_Turn_Next)) then
            if R.Covered then
               if not C.Fixed_Covered then
                  C.Fixed_Turn_Gap := 1;
               else
                  C.Fixed_Turn_Gap := 2 * C.Fixed_Turn_Gap;
               end if;
               C.Fixed_Turn_Next := C.Round_N + C.Fixed_Turn_Gap;
            end if;
            declare
               Base : constant Natural := R.Consistent_Now;
               Have : Boolean := R.Moved;
               Best_G : Geom.Cam_Geo := G;
               Best_R : Geom.Fixed_Check := R;
               Best_B : Geom.Fixed_Best := C.Fixed_Best;
               Best_T : Natural := C.Fixed_Turn;
            begin
               for T in 0 .. 3 loop
                  if T /= C.Fixed_Turn then
                     declare
                        Gt : Geom.Cam_Geo := C.Geo (Wc);
                        Bt : Geom.Fixed_Best := B0;
                        Rt : Geom.Fixed_Check;
                        Okt : Boolean;
                        Nt : constant Geom.Scene_Pt_Vectors.Vector := Matched (T, Okt);
                     begin
                        if Okt then
                           Geom.Check_Fixed (Gt, C.Board, Nt, Bt, Rt, Turn_Sd => C.Fixed_Turn_Sd, Base_Now => Base);
                           if Rt.Moved and then (not Have or else Rt.Consistent > Best_R.Consistent) then
                              Best_G := Gt; Best_R := Rt; Best_B := Bt; Best_T := T; Have := True;
                           end if;
                        end if;
                     end;
                  end if;
               end loop;
               if Have then
                  G := Best_G; R := Best_R; C.Fixed_Best := Best_B; Turned := Best_T; C.Fixed_Turn := Best_T;
               end if;
            end;
         end if;
      end;
      if R.Moved then
         C.Geo.Replace_Element (Wc, G);
         --  参考图和板上的点在参考图里的像素都不换(一直是标好那一刻的):换成此刻的,一挡住参考图就跟着坏,错一轮接一轮地叠(X5B 2026-09-25)。
         --  它按旧位姿做的轮廓作废
         if C.Sil_Valid and then C.Sil_Cam = Integer (Wc) then
            C.Sil_Valid := False;
         end if;
         Geom.Save (To_String (C.Geo_Path), C.Geo);
         Board_Save (C);
         Geo_Say ("核对不动的眼:它被挪过 —— 转了 " & Codec.Fmt (R.Turn_Deg, 1) & Deg_Say & "、挪了 " & Mm (R.Move_M) & ",板上的点在画面里挪了 " & Codec.Fmt (R.Shift_Px, 1)
                  & " px(" & Codec.Fmt (R.Shift_Sd, 1) & " 个配点噪声)"
                  & (if Turned > 0 then ",此刻的图顺时针转 " & Codec.Img (90 * Turned) & Deg_Say & " 配得最好(以后每轮都这么转了再配)" else "")
                  & " ⇒ 按板重新标好(" & Codec.Img (R.Consistent) & "/" & Codec.Img (R.Asked) & " 个点对得上,残差 " & Codec.Fmt (R.Rms, 2) & " px),接着干");
      elsif R.Covered then
         if not C.Fixed_Covered then
            Geo_Say ("核对不动的眼:板上 " & Codec.Img (R.Asked) & " 个点这会儿只有 " & Codec.Img (R.Consistent_Now) & " 个还对得上(放好以来最多 " & Codec.Img (C.Fixed_Best.All_N)
                     & (if R.Dark >= 0 then ";画面" & Geom.Region_Name (Natural (R.Dark)) & "放好以来看见过 " & Codec.Img (R.Dark_Best) & " 个,这会儿只剩 "
                        & Codec.Img (R.Dark_Now) & " 个" else "")
                     & " 个)⇒ 它被挡住了一大块(或看不见了);位姿照旧,它这会儿看见的东西先别全信");
         end if;
      elsif C.Fixed_Covered or else not C.Fixed_Said then
         Geo_Say ("核对不动的眼" & (if C.Fixed_Said then "" else "(这次开机第一次)") & ":板上 " & Codec.Img (R.Asked) & " 个点此刻 " & Codec.Img (R.Consistent_Now)
                  & " 个对得上(放好以来最多 " & Codec.Img (C.Fixed_Best.All_N) & " 个)⇒ " & (if C.Fixed_Covered then "又看全了" else "没挪、没挡"));
      end if;
      --  挡没挡只在变的那一轮说(X5C 每轮报一遍"挡住了")
      C.Fixed_Covered := R.Covered and then not R.Moved;
      C.Fixed_Said := True;
      --  每一轮核对的数落盘(BL_DUMP/check.txt):轮、配到、原位姿对得上、新解对得上、放好以来最多、挪没挪、挡没挡、转回几个 90°、细门(像素)
      if Length (C.Dump_Dir) > 0 then
         declare
            Fo : Ada.Text_IO.File_Type;
            Path : constant String := To_String (C.Dump_Dir) & "/check.txt";
         begin
            begin
               Ada.Text_IO.Open (Fo, Ada.Text_IO.Append_File, Path);
            exception
               when others => Ada.Text_IO.Create (Fo, Ada.Text_IO.Out_File, Path);
            end;
            Ada.Text_IO.Put_Line (Fo, Codec.Img (C.Round_N) & " " & Codec.Img (R.Matched) & " " & Codec.Img (R.Consistent_Now) & " " & Codec.Img (R.Consistent) & " "
                                  & Codec.Img (C.Fixed_Best.All_N) & " " & (if R.Moved then "1" else "0") & " " & (if R.Covered then "1" else "0") & " " & Codec.Img (Turned)
                                  & " " & Codec.Fmt (R.Gate, 3) & " " & Integer'Image (R.Dark) & " " & Codec.Img (R.Dark_Now) & " " & Codec.Img (R.Dark_Best));
            --  列:轮、配到、原位姿对得上、新解对得上、放好以来最多、挪没挪、挡没挡、转回几个 90°、细门(像素)、看不见的那一块(-1 = 没有)、它此刻 / 放好以来对得上几个
            Ada.Text_IO.Close (Fo);
         exception
            when others => null;
         end;
      end if;
   end;
end Check_Fixed_Eye;
