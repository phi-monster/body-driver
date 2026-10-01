separate (Act)
procedure Expand_Lobes (C : Context; F : Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector) is
   Out_P : Point_Vectors.Vector;
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
begin
   for P of Pts loop
      declare
         Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, P.Cam, Jaw_K_Of (P.Chan_K));
      begin
         --  🔴🔴 每一瓣一个接触点,瓣数【读身体量到的那个数】,不许写死。
         --  以前这里写 Z.N_Lobes = 2:7 指爪、软体臂、吸盘一律不展开 —— owner 揪出过一次,换了个地方又长出来。
         --  以前还写 Cam_Arm (C, Cam) /= P.Arm(手【自己】的相机里不展开):而 GM 全程跑在手腕相机里
         --  ⇒ 这段代码等于从没执行 ⇒ 全程只跟一个中心点 ⇒ 3 个自由度在零空间里乱走
         --  (6b3ad77 原话:手腕乱拧、球被转出画面、指尖落在球旁边 —— 一字不差就是 GM 的死法)。
         --  手自己的相机里瓣是固定像素,那正好:它们就是【要抓的东西的几侧必须去到的地方】,
         --  两点 ×(左右/上下/远近) = 6 个约束,正好按住 6 个通道。
         if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm and then Z.Valid and then Z.N_Lobes >= 2 then
            for Lb in 0 .. Z.N_Lobes - 1 loop
               declare
                  Q : Point := P;
                  Tr : constant Zone_Track := C.Zones (Track_Idx (C, P.Arm, Cam));
                  --  瓣相对区心的偏移:身体图给了此刻各瓣位置就用它(转过的手瓣也跟着转),否则用开机量的
                  Lo : constant Zone.Lobe := Zone.Lobe_Of (Z, Lb);
                  Ou : constant Long_Float :=
                    (if Tr.Has_Lobes and then Lb <= 1 then (if Lb = 0 then Tr.Au else Tr.Bu) - Tr.Cu
                     else Lo.Cu - Z.Cu);
                  Ov : constant Long_Float :=
                    (if Tr.Has_Lobes and then Lb <= 1 then (if Lb = 0 then Tr.Av else Tr.Bv) - Tr.Cv
                     else Lo.Cv - Z.Cv);
                  Zd : Long_Float := P.Z;
               begin
                  Q.Blob := Lb;
                  Q.Par_Tu := P.Tu; Q.Par_Tv := P.Tv;
                  Q.Cu := P.Cu + Ou; Q.Cv := P.Cv + Ov;
                  Q.Tu := P.Tu + Ou; Q.Tv := P.Tv + Ov; Q.Tuv_Z := P.Tuv_Z;
                  if F.Cams (Cam).Has_Depth then
                     --  读深窗口 = 张幅的四分之一,再小也有半个百分点的画幅(比例,无量纲)
                     Zd := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, Q.Cu, Q.Cv, Long_Float'Max (0.005, Z.Span * 0.25));
                     if Picture.Is_Nan (Zd) then
                        Zd := P.Z;
                     end if;
                  end if;
                  Q.Z := Zd;
                  if Lb > 0 then
                     Q.Desc := Null_Unbounded_String;
                  end if;
                  Out_P.Append (Q);
               end;
            end loop;
         elsif P.Kind = Thing_Pt and then Z.Valid and then Z.N_Lobes >= 2
           and then Zone.Lobe_Of (Z, 0).Valid and then Zone.Lobe_Of (Z, 1).Valid
         then
            --  🔴🔴 要抓的那一块,也沿【合拢方向】拆成和爪瓣一样多的点。
            --  只跟它的中心时,一个点只给 3 行(左右/上下/远近)去定 6 个通道 ⇒ 3 个自由度在零空间里
            --  自由乱走 —— 6b3ad77 原话:"手腕乱拧、球被转出画面、指尖落在球旁边",GU 实测:
            --  爪子到过距球 0.09 画幅,下一条命令又晃回去。两点 × 3 行 = 6 个约束,正好按住 6 个通道。
            --  拆几个不写死:爪有几瓣就几个(Z.N_Lobes,身体量的)。方向用爪的合拢方向(Z.Au,Z.Av,量的),
            --  半径用这一块自己的半宽(量的)。这是"跟这块沿合拢方向的两侧",讲的是【物体】的性质,
            --  和几根手指无关 —— owner 当年撤掉的是"写死两根手指"那一版实现,不是这个做法。
            begin
               for Lb in 0 .. Z.N_Lobes - 1 loop
                  declare
                     Q : Point := P;
                     Lo : constant Zone.Lobe := Zone.Lobe_Of (Z, Lb);
                     --  🔴 拆开多远,不自己算半宽(那要写 ×0.5,是人拍的系数),
                     --  直接用【身体量到的瓣位相对区心的偏移】—— 球的接触点本来就该落在手指将来所在的地方。
                     Ou : constant Long_Float := Lo.Cu - Z.Cu;
                     Ov : constant Long_Float := Lo.Cv - Z.Cv;
                  begin
                     Q.Blob := Lb;
                     Q.Par_Tu := P.Tu; Q.Par_Tv := P.Tv;
                     Q.Cu := P.Cu + Ou;
                     Q.Cv := P.Cv + Ov;
                     Q.Tu := P.Tu + Ou; Q.Tuv_Z := P.Tuv_Z;
                     Q.Tv := P.Tv + Ov;
                     if Lb > 0 then
                        Q.Desc := Null_Unbounded_String;
                     end if;
                     Out_P.Append (Q);
                  end;
               end loop;
            end;
         else
            Out_P.Append (P);
         end if;
      end;
   end loop;
   Pts := Out_P;
end Expand_Lobes;
