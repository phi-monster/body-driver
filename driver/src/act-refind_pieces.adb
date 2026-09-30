separate (Act)
procedure Refind_Pieces (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector) is
   --  Pts 空的时候 `Pts (0)` 当场越界,而它在声明区 ⇒ 异常记在【调用处】,
   --  栈里根本看不到这个子程序这一帧(实测查了半天)。空就当第 0 条胳膊,下面第一句直接回。
   Pts_Empty : constant Boolean := Natural (Pts.Length) = 0;
   Arm : constant Natural := (if Pts_Empty then 0 else Pts (0).Arm);
   --  🔴🔴 抖之前先记下【我猜的】位置。抖完拿"我看到的"和它一比,就是这具身体
   --  唯一一次能自己验证"我的手在哪"的机会 —— 而它一直没比过。
   --  十三炮的终局全是"伺服往错的方向推",而错的源头就是这个猜出来的位置
   --  (GW 实测:右臂两根手指被放到画面左边、相隔四分之三个画面)。
   Guess_U : constant Long_Float := Pts (0).Cu;
   Guess_V : constant Long_Float := Pts (0).Cv;
   Guess_Z : constant Long_Float := Pts (0).Z;
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
   Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
   Steps_J : Natural;
   Reading : Long_Float;
   Any_Fingers : Boolean := False;
   Jaw : Floats;
   --  在累积的"动过"掩膜里给一个点认领最近的一团;Tol = 认领半径(画幅比例,无量纲)
   procedure Claim (P : in out Point; Tol, Win : Long_Float; Taken : in out Bools; Regs : Picture.Regions) is
      Best : Integer := -1;
      Bd : Long_Float := 1.0e9;
   begin
      for R in 0 .. Natural (Regs.Length) - 1 loop
         declare
            D : constant Long_Float := Sqrt ((Regs (R).Cu - P.Cu) ** 2 + (Regs (R).Cv - P.Cv) ** 2);
         begin
            if not Taken (R) and then D <= Tol and then D < Bd then
               Bd := D; Best := R;
            end if;
         end;
      end loop;
      if Best >= 0 then
         Taken.Replace_Element (Natural (Best), True);
         declare
            Old_Z : constant Long_Float := P.Z;
            Zd : Long_Float;
         begin
            P.Cu := Regs (Best).Cu; P.Cv := Regs (Best).Cv; P.Lost := False;
            P.Box_W := Long_Float (Regs (Best).X1 - Regs (Best).X0) / Long_Float (Cw);
            P.Box_H := Long_Float (Regs (Best).Y1 - Regs (Best).Y0) / Long_Float (Ch);
            if F.Cams (Cam).Has_Depth then
               --  深度一步跳过"距离的一成"(比例,无量纲)就是读到别的东西了
               Zd := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, P.Cu, P.Cv, Win);
               if not Picture.Is_Nan (Zd) and then (Old_Z <= 0.0 or else abs (Zd - Old_Z) <= 0.1 * Old_Z) then
                  P.Z := Zd;
               end if;
            end if;
         end;
      else
         P.Lost := True;
      end if;
   end Claim;
begin
   if Pts_Empty then
      return;
   end if;
   --  这条臂此刻的抓握读数原样当目标(挪手的时候手指不动);这一拍没读数就不带(插头按"这一集给过的最后一个目标"保持)
   Jaw := Selfmap.Jaw_All (F, Arm);
   for P of Pts loop
      if P.Kind = Piece_Pt and then P.Chan_K >= Chan.Per_Arm then
         Any_Fingers := True;
      end if;
   end loop;
   if Any_Fingers then
      declare
         Sweep : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
         Regs : Picture.Regions;
         Taken : Bools;
      begin
         --  这些点分别属于哪几个抓握通道,就抖哪几个(五指手:只抖被跟着的那几根)
         for Kk in 0 .. Jaws_Of (C, Arm) - 1 loop
            declare
               Wanted : Boolean := False;
            begin
               for P of Pts loop
                  if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm + Kk then
                     Wanted := True;
                  end if;
               end loop;
               if Wanted then
                  declare
                     --  抖一下手指重新认它:合到这只手量过的"合空"那头、再回到抖之前的读数(09-30:原来合的目标写死 0.0 = x5"0 = 合",
                     --  回的目标又是合完以后才读的读数 ⇒ 手指张不回去)。拍数上限 = 开机量到的合一次要几拍(Close_Steps)
                     Hk : constant Zone.Hand := Hand_Of (C, Arm, Kk);
                  begin
                     if Selfmap.Has_Jaw (F, Arm, Kk) and then Hk.Measured and then Hk.Close_Steps > 0 then
                        declare
                           R0 : constant Long_Float := Selfmap.Jaw_Of (F, Arm, Kk);
                        begin
                           Jaw_Sweep (L, C, F, Arm, Kk, Hk.Empty_Close, Hk.Close_Steps, Integer (Cam), Sweep, Steps_J, Reading);
                           Jaw_Sweep (L, C, F, Arm, Kk, R0, Hk.Close_Steps, Integer (Cam), Sweep, Steps_J, Reading);
                        end;
                     else
                        Put_Line ("[身] ✋ 第" & Codec.Img (Arm + 1) & " 只手第 " & Codec.Img (Kk + 1) & " 个抓握通道"
                                  & (if not Selfmap.Has_Jaw (F, Arm, Kk) then "这一拍没有读数" else "没量过合一次要几拍") & " ⇒ 不抖它来重认");
                     end if;
                  end;
               end if;
            end;
         end loop;
         Regs := Picture.Components (Sweep, Cw, Ch, Picture.Min_Pixels (Cw, Ch));
         Taken := Bool_Vectors.To_Vector (False, Regs.Length);
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               P : Point := Pts (I);
            begin
               if P.Kind = Piece_Pt and then P.Chan_K >= Chan.Per_Arm then
                  --  认领半径:一个张幅,再小也有一个跟踪窗;读深窗口 = 张幅的四分之一,再小也有半个百分点的画幅(比例,无量纲)
                  --  没真看过的位置(只是按关节推的)可能差得远 ⇒ 认领半径放到整幅画面(比例,无量纲)
                  Claim (P, (if P.Known then Long_Float'Max (Z.Span, Track_Win) else 1.0), Long_Float'Max (0.005, Z.Span * 0.25), Taken, Regs);
                  if not P.Lost then
                     P.Known := True;
                  end if;
                  Pts.Replace_Element (I, P);
               end if;
            end;
         end loop;
      end;
   end if;
   --  零件:各自推一下自己的通道(开机看得见的那一档)再推回来
   for I in 0 .. Natural (Pts.Length) - 1 loop
      declare
         P : Point := Pts (I);
      begin
         if P.Kind = Piece_Pt and then P.Chan_K < Chan.Per_Arm then
            declare
               K : constant Natural := P.Chan_K;
               Chn : constant Natural := Arm * Chan.Per_Arm + K;
               A : Table.Vec := Table.Zero_Vec;
               Deliv : Table.Vec;
               Ok : Boolean;
               B0 : constant Buf := F.Cams (Cam).Gray;
               Sweep : Bools;
               Regs : Picture.Regions;
               Taken : Bools;
               P0 : constant Plug.Arm_Pose := F.EE (Arm);
               Frames : Natural;
            begin
               if Chn < Natural (C.Map.Amp.Length) and then C.Map.Amp (Chn) > 0.0 and then Cam < Natural (C.Map.Floors.Length) then
                  A (K) := C.Map.Amp (Chn);
                  Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok);
                  Sweep := Picture.Moved (B0, F.Cams (Cam).Gray, C.Map.Floors (Cam));
                  declare
                     B1 : constant Buf := F.Cams (Cam).Gray;
                  begin
                     Selfmap.Go (L, C.Map, Arm, P0, Jaw, F, Deliv, Frames, Ok);
                     Sweep := Picture.Either (Sweep, Picture.Moved (B1, F.Cams (Cam).Gray, C.Map.Floors (Cam)));
                  end;
                  Regs := Picture.Components (Sweep, Cw, Ch, Picture.Min_Pixels (Cw, Ch));
                  Taken := Bool_Vectors.To_Vector (False, Regs.Length);
                  --  认领半径:这块自己的框那么大,再小也有一个跟踪窗;读深窗口 = 框的四分之一(比例,无量纲)
                  Claim (P, (if P.Known then Long_Float'Max (Long_Float'Max (P.Box_W, P.Box_H), Track_Win) else 1.0), Long_Float'Max (0.005, Long_Float'Max (P.Box_W, P.Box_H) * 0.25), Taken, Regs);
                  if not P.Lost then
                     P.Known := True;
                  end if;
               else
                  P.Lost := True;
               end if;
               Pts.Replace_Element (I, P);
            end;
         end if;
      end;
   end loop;
   --  认到的记进身体图:这个位姿下,这只手的这些零件(手指也是零件)在这台相机里就在这儿(下次到这附近不用看)
   declare
      X : Schema.Sample;
      Gp : Schema.Part_Pos;   --  手指那块:各团合成
      All_Fingers : Boolean := True;
      N, Nz : Natural := 0;
      Zmin : Long_Float := 1.0e30;   --  哨兵(无量纲)
      Any_Part : Boolean := False;
      Zh : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
   begin
      X.Arm := Arm; X.Cam := Cam; X.Pose := F.EE (Arm);
      for P of Pts loop
         if P.Kind = Piece_Pt and then P.Chan_K >= Chan.Per_Arm then
            if P.Lost then
               All_Fingers := False;
            end if;
            N := N + 1;
            Gp.Cu := Gp.Cu + P.Cu; Gp.Cv := Gp.Cv + P.Cv;
            if P.Blob = 1 then
               Gp.B1u := P.Cu; Gp.B1v := P.Cv;
            else
               Gp.B0u := P.Cu; Gp.B0v := P.Cv;
            end if;
            if P.Z > 0.0 then
               Zmin := Long_Float'Min (Zmin, P.Z); Nz := Nz + 1;
            end if;
         elsif P.Kind = Piece_Pt and then not P.Lost and then P.Chan_K < Chan.Per_Arm then
            X.Parts (P.Chan_K) := (True, P.Cu, P.Cv, P.Z,
                                   Natural (Long_Float'Max (0.0, (P.Cu - P.Box_W / 2.0) * Long_Float (Cw))), Natural (Long_Float'Max (0.0, (P.Cv - P.Box_H / 2.0) * Long_Float (Ch))),
                                   Natural (Long_Float'Min (Long_Float (Cw - 1), (P.Cu + P.Box_W / 2.0) * Long_Float (Cw))), Natural (Long_Float'Min (Long_Float (Ch - 1), (P.Cv + P.Box_H / 2.0) * Long_Float (Ch))),
                                   1, P.Cu, P.Cv, 0.0, 0.0);
            Any_Part := True;
         end if;
      end loop;
      if All_Fingers and then N > 0 then
         Gp.Valid := True;
         Gp.Cu := Gp.Cu / Long_Float (N); Gp.Cv := Gp.Cv / Long_Float (N);
         Gp.N_Blobs := N;
         Gp.Z := (if Nz > 0 then Zmin else 0.0);
         Gp.X0 := Zh.X0; Gp.Y0 := Zh.Y0; Gp.X1 := Zh.X1; Gp.Y1 := Zh.Y1;   --  框先沿用开机量的(Feel 会按形心平移)
         X.Parts (Chan.Per_Arm) := Gp;
      end if;
      if (All_Fingers and then N > 0) or else Any_Part then
         Schema.Add (C.Sch, X, C.Map.EE_Noise, C.Map.Rot_Noise);
      end if;
   end;
   --  🔴 认不出自己的时候,光说"按图猜"不够 —— 要说清【猜到哪儿了】。
   --  GP 实测:猜到 (1.000,0.475),那是画面最右边一列;从画面外的位置算出来的误差全是垃圾,
   --  于是 60 步一个像素没动,而日志一路"绿"。猜到画面边上 = 这台相机判不了这一段,
   --  必须让脑知道,好换一只眼睛(和"看不见目标的眼睛干不了这一段"是同一条规矩,只是换到我自己这一半)。
   declare
      U : constant Long_Float := Pts (0).Cu;
      V : constant Long_Float := Pts (0).Cv;
      --  边不是人拍的:一个跟踪窗那么宽 —— 眼睛跟得住的最小尺度,比它还靠边就没法量位移了
      Edge : constant Long_Float := Track_Win;
      Off : constant Boolean := U <= Edge or else U >= 1.0 - Edge or else V <= Edge or else V >= 1.0 - Edge;
   begin
      --  猜的 vs 看到的:差了多少。比不出来就不说(没认到时看到的那一份不存在)
      if not Pts (0).Lost then
         declare
            D : constant Long_Float := Sqrt ((U - Guess_U) ** 2 + (V - Guess_V) ** 2);
         begin
            if D > Track_Win then
               Put_Line ("[身]     🔴 我猜我在 (" & Codec.Fmt (Guess_U, 3) & "," & Codec.Fmt (Guess_V, 3)
                         & ") 深 " & Codec.Fmt (Guess_Z, 3) & ",一看其实在 (" & Codec.Fmt (U, 3) & ","
                         & Codec.Fmt (V, 3) & ") 深 " & Codec.Fmt (Pts (0).Z, 3) & " —— 差 "
                         & Codec.Fmt (D, 3) & " 画幅(眼睛能跟住的一个窗口才 " & Codec.Fmt (Track_Win, 4)
                         & ")。这一段之前算的误差全是拿猜的位置算的");
               C.Blind_Say := S ("careful: where my body map said my hand was and where I just saw it are "
                                 & Codec.Fmt (D, 3) & " of the picture apart - everything I worked out before "
                                 & "this look was measured from the wrong place");
            end if;
         end;
      end if;
      Put_Line ("[身]     生地/大步之后看一眼自己(手指抖一下 / 零件推一下):" &
                (if Pts (0).Lost then "没认到,按图猜" else "认到了") &
                " (" & Codec.Fmt (U, 3) & "," & Codec.Fmt (V, 3) & ") 深 " & Codec.Fmt (Pts (0).Z, 3) &
                (if Off then " 🔴 这个位置贴在画面边上(边宽 " & Codec.Fmt (Edge, 3)
                   & " 画幅)—— 从画面外算出来的误差是垃圾,这台相机判不了这一段" else "") &
                " · 这台相机里这只手的身体图 " & Codec.Img (Schema.Count (C.Sch, Arm, Cam)) & " 个样本");
      if Off then
         C.Blind_Say := S ("I could not find my own part in this eye and the place my body map guesses for it "
                           & "is right at the edge of the picture, so anything I measure from it is rubbish - "
                           & "name what you want in one of my other eyes and I will work there");
      end if;
   end;
end Refind_Pieces;
