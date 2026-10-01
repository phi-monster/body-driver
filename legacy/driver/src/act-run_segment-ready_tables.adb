separate (Act.Run_Segment)
procedure Ready_Tables (Ok_Out : out Boolean) is
   Need : Boolean := False;
begin
   Ok_Out := True;
   for I in 0 .. Natural (Pts.Length) - 1 loop
      declare
         Idx : constant Integer := Find_Effect (C, Arm, Pts (I).Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob);
      begin
         if Idx >= 0 and then (for some K in 0 .. Chan.Per_Arm - 1 => C.Tables (Natural (Idx)).Trust (K))
           and then C.Tables (Natural (Idx)).Has_Pose
           and then (for all K in 0 .. Chan.Per_Arm - 1 =>
                       abs Chan.Delivered (C.Tables (Natural (Idx)).Pose, F.EE (Arm)) (K)
                       <= Long_Float'Max (1.0e-6, C.Map.Amp (Arm * Chan.Per_Arm + K)) * Cap_Mult * C.Tables (Natural (Idx)).Reach (K))
         then
            Effs (I) := C.Tables (Natural (Idx)).E;
            Trusts (I) := C.Tables (Natural (Idx)).Trust;
            for K in 0 .. Chan.Per_Arm - 1 loop
               Reach (K) := Long_Float'Max (Reach (K), C.Tables (Natural (Idx)).Reach (K));
            end loop;
         else
            Need := True;
         end if;
      end;
   end loop;
   if Need then
      declare
         Trust : Table.Mask;
         Ok : Boolean;
      begin
         Probe_Effects (L, C, F, Cam, Pts, Effs, Trust, Ok, Amount * Cap_Mult);
         if not Ok then
            Ok_Out := False;
            return;
         end if;
         for I in 0 .. Natural (Pts.Length) - 1 loop
            Trusts (I) := Trust;
            Store_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob, Effs (I), Trust, Unit_Reach, F.EE (Arm), True);
         end loop;
         --  🔴 量完就把表打出来:每根通道推 +1,画面左右跑多少 / 离相机远近变多少(正=变远)。
         --  方向对不对全看正负。HU 实测:左右已经对到 0.008 m,而远近误差越走越大(1.357→1.711),
         --  手朝相机走而球在 3.5 m 外 —— 光看误差分不出是表的符号反了还是解算没得选。
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               Ln : Unbounded_String :=
                 S ("[身]   表 点" & Codec.Img (I) & ":");
            begin
               for K in 0 .. Chan.Per_Arm - 1 loop
                  Append (Ln, " ch" & Codec.Img (Arm * Chan.Per_Arm + K) & "(左右"
                          & Codec.Fmt (Effs (I).B (K, 0), 3) & " 远近"
                          & Codec.Fmt (Effs (I).B (K, 2), 3)
                          & (if Trust (K) then "" else " 没证过") & ")");
               end loop;
               Put_Line (To_String (Ln));
               --  🔴🔴 体检:平移通道推一米,远近最多变一米。绝对值 > 1 = 物理上不可能。
               --  取所有平移通道里最大的那个,就是【我的深度读数被放大了几倍】的下界 ——
               --  这就是拿自己的胳膊当尺子:我知道自己走了几米,也看得见深度读数变了多少。
               --  🔴 分两句说:信得过的列里最坏多少 · 【已经作废的列里】最坏多少。
               --  只统计信得过的列会把病情说小:HY 实测报"1.4 倍",而同一炮里有一根
               --  命令 0.0032 实到 0.0020、深度却变 0.7605 ⇒ 380 倍 —— 那一根已被来回对表判死,
               --  于是不参与,病情就被漏报了。作废的那些不参与修正,但必须说出来给脑看病。
               declare
                  Worst : Long_Float := 0.0;
                  Worst_Dead : Long_Float := 0.0;
               begin
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     if Depth_Scale_Bad (Effs (I).B (K, 2)) then
                        if Trust (K) then
                           Worst := Long_Float'Max (Worst, abs Effs (I).B (K, 2));
                        else
                           Worst_Dead := Long_Float'Max (Worst_Dead, abs Effs (I).B (K, 2));
                        end if;
                     end if;
                  end loop;
                  if Worst_Dead > 0.0 then
                     Put_Line ("[身]   🔴 体检(已作废的那些列里):最坏的一根是 我真走一米、深度读数变 "
                               & Codec.Fmt (Worst_Dead, 1) & " 米 —— 它已经被来回对表判死了,不参与,"
                               & "但这就是我的距离感到底有多烂。");
                  end if;
                  if Worst > 0.0 then
                     C.Depth_Scale := Long_Float'Max (C.Depth_Scale, Worst);
                     Put_Line ("[身]   🔴 体检:我真走一米,深度读数变了 " & Codec.Fmt (Worst, 1)
                               & " 米 —— 物理上最多一米。我的深度读数被放大了至少 "
                               & Codec.Fmt (Worst, 1) & " 倍,这一维我不当真的量看。");
                     C.Blind_Say := S ("I checked myself: one metre of my own real motion changes my depth reading by "
                                       & Codec.Fmt (Worst, 1) & " metres, and the most that is physically possible is one. "
                                       & "So my sense of distance is inflated by at least that much and I do not trust it "
                                       & "as a real measurement - I am using it only for direction, not for how far.");
                  end if;
               end;
            end;
         end loop;
      end;
   end if;
end Ready_Tables;
