separate (Jointboot)
procedure Find_Arms (L : in out Plug.Link; F : in out Plug.Frame; M : in out Selfmap.Body_Map;
                     Arms : out Arm_Vectors.Vector; World_Cam : out Integer; Ok : out Boolean) is
   Ng : constant Natural := Natural (F.Joints.Length);
   Nc : constant Natural := Natural (F.Cams.Length);
   Echo : array (0 .. Natural'Max (1, Ng) - 1) of Boolean := [others => False];
begin
   Arms.Clear;
   World_Cam := 0;
   Ok := False;
   if Ng = 0 or else Nc = 0 then
      Say ("身体不报关节读数或没有相机 ⇒ 认不了身体");
      return;
   end if;
   for G in 0 .. Ng - 1 loop
      if not Echo (G) and then not F.Joints (G).Is_Empty then
         declare
            Q0 : constant Floats := F.Joints (G);
            Amp : Long_Float := Long_Float'Max (Start_Amp, 4.0 * M.Joint_Noise);
            Info : Arm_Info;
            Found : Boolean := False;
         begin
            Info.Group := G;
            for Try in 0 .. Max_Doublings loop
               declare
                  Tgt : Floats := Q0;
                  F0 : constant Plug.Cam_Vectors.Vector := F.Cams;
                  J0 : constant Plug.Floats_Vectors.Vector := F.Joints;
                  F1, F1b : Plug.Cam_Vectors.Vector;   --  转到那头:走完那一帧、再读的一帧
                  S0 : Natural := L.Seq;
                  J1 : Plug.Floats_Vectors.Vector;
                  Okg : Boolean;
                  Got : Long_Float := Long_Float'Last;
                  Visible : Boolean := False;
                  Fr : Floats;
               begin
                  for K in 0 .. Natural (Tgt.Length) - 1 loop
                     Tgt.Replace_Element (K, Q0 (K) + Amp);
                  end loop;
                  S0 := L.Seq;   --  从发命令这一拍数起,读数几拍才停住(Settle,开机前半段就量;09-30 原来前半段不量、恒为缺省 2)
                  Go_Group (L, F, M, Natural (Arms.Length), G, Tgt, Amp * Third, Okg);
                  exit when not Okg;
                  F1 := F.Cams; J1 := F.Joints;
                  --  转到那头再读一帧:画面比读数晚一拍(开机量的),"读数到了"那一刻的前一帧画面还没到那头(V1B58 2026-09-28:拿它当第二帧,
                  --  x5 第 1 只手 0.0001 / 0.0002 两档都判成没看见、第 2 只手认不出眼);走完那一帧和再读的这一帧都是到了以后的画面
                  exit when not Plug.Sense (L, F);
                  F1b := F.Cams;
                  M.Settle := Natural'Max (M.Settle, Selfmap.Settle_Since (L, S0, M.Joint_Noise));
                  S0 := L.Seq;
                  Go_Group (L, F, M, Natural (Arms.Length), G, Q0, Amp * Third, Okg);
                  exit when not Okg;
                  M.Settle := Natural'Max (M.Settle, Selfmap.Settle_Since (L, S0, M.Joint_Noise));
                  for K in 0 .. Natural'Min (Natural (J1 (G).Length), Natural (J0 (G).Length)) - 1 loop
                     Got := Long_Float'Min (Got, J1 (G) (K) - J0 (G) (K));   --  这组读数里跟得最少的那个关节
                  end loop;
                  for C in 0 .. Nc - 1 loop
                     --  这台相机这四拍里有一拍没画面(插头留的空位,09-30):这一次看不出它动没动,不算它
                     if not (Plug.Has_Picture (F0 (C)) and then Plug.Has_Picture (F1 (C)) and then Plug.Has_Picture (F1b (C))
                             and then Plug.Has_Picture (F.Cams (C)))
                     then
                        Fr.Append (0.0);
                        goto Next_Cam;
                     end if;
                     declare
                        Fl : Picture.Floor_Map renames M.Floors (C);
                        M1 : constant Bools := Picture.Moved (F0 (C).Gray, F1 (C).Gray, Fl);
                        M2 : constant Bools := Picture.Moved (F1 (C).Gray, F.Cams (C).Gray, Fl);
                        --  看没看见动了:两次比较、不共用一帧(转之前 → 走完那一帧;再读的那一帧 → 转回来;同逐通道推、抓握通道推到头:Picture.Seen_Twice)
                        Comps : constant Picture.Regions :=
                          Picture.Seen_Twice (F0 (C).Gray, F1 (C).Gray, F1b (C).Gray, F.Cams (C).Gray, Fl, F.Cams (C).W, F.Cams (C).H);
                     begin
                        Fr.Append (Picture.Fraction (Picture.Either (M1, M2)));
                        if not Comps.Is_Empty then
                           Visible := True;
                        end if;
                     end;
                     <<Next_Cam>>
                  end loop;
                  if Got >= 0.5 * Amp and then Visible then   --  读数跟上了命令的一半(比例,同 Selfmap)、有相机看见了
                     Found := True;
                     Info.Frac := Fr;
                     Info.Probe := Amp;
                     --  别的组跟着变了同样多 ⇒ 这只手的回声组
                     for G2 in 0 .. Ng - 1 loop
                        if G2 /= G and then G2 < Natural (J1.Length) and then G2 < Natural (J0.Length)
                          and then Natural (J1 (G2).Length) = Natural (J0 (G2).Length) and then not J1 (G2).Is_Empty
                        then
                           declare
                              Mn : Long_Float := Long_Float'Last;
                           begin
                              for K in 0 .. Natural (J1 (G2).Length) - 1 loop
                                 Mn := Long_Float'Min (Mn, J1 (G2) (K) - J0 (G2) (K));
                              end loop;
                              if Mn >= 0.5 * Amp then
                                 Echo (G2) := True;
                                 Info.Echoes.Append (G2);
                              end if;
                           end;
                        end if;
                     end loop;
                     exit;
                  end if;
                  Amp := Amp * Grow;
               end;
            end loop;
            if Found then
               --  哪台相机长在这只手上:整幅都变,而且比第二名多一倍(倍数,无量纲;同 Selfmap)
               declare
                  Best : Integer := -1;
                  Bv, Second : Long_Float := 0.0;
               begin
                  for C in 0 .. Nc - 1 loop
                     if Info.Frac (C) > Bv then
                        Second := Bv; Bv := Info.Frac (C); Best := C;
                     elsif Info.Frac (C) > Second then
                        Second := Info.Frac (C);
                     end if;
                  end loop;
                  if Best >= 0 and then Bv > 0.0 and then Bv >= 2.0 * Second then
                     Info.Eye := Best;
                  end if;
                  Say ("第" & Codec.Img (Natural (Arms.Length) + 1) & " 只手 = 第" & Codec.Img (G) & " 组关节读数(" & Codec.Img (Natural (Q0.Length))
                       & " 个,每个一起转 " & Codec.Fmt (Amp, 4) & " 就看得见)"
                       & (if Info.Eye >= 0 then ";第" & Codec.Img (Natural (Info.Eye)) & " 台相机变了 " & Codec.Fmt (100.0 * Bv, 0) & "% 的画面 ⇒ 它长在这只手上"
                          else ";哪台相机都不比第二名多一倍 ⇒ 这只手上没有眼")
                       & (if Info.Echoes.Is_Empty then "" else ";第" & Codec.Img (Natural (Info.Echoes (0))) & " 组跟着一起变 = 它的回声,不单算"));
                  Arms.Append (Info);
               end;
            else
               Say ("第" & Codec.Img (G) & " 组关节读数:探到 " & Codec.Fmt (Amp, 4) & " 读数还跟不上或哪台相机都没看见 ⇒ 不算一只手");
            end if;
         end;
      end if;
   end loop;
   --  世界相机 = 不长在哪只手上的那台里,所有手动时变得最少的(每台相机都长在某只手上 ⇒ 没有,-1;DIY 身体可以没有不动的眼)
   declare
      Bv : Long_Float := Long_Float'Last;
   begin
      World_Cam := -1;
      for C in 0 .. Nc - 1 loop
         declare
            Mx : Long_Float := 0.0;
            On_Arm : Boolean := False;
         begin
            for A of Arms loop
               if C < Natural (A.Frac.Length) then
                  Mx := Long_Float'Max (Mx, A.Frac (C));
               end if;
               On_Arm := On_Arm or else A.Eye = Integer (C);
            end loop;
            if not On_Arm and then Mx < Bv then
               Bv := Mx; World_Cam := C;
            end if;
         end;
      end loop;
   end;
   Ok := not Arms.Is_Empty;
end Find_Arms;
