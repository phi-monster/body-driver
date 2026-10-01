separate (Act.Run_Segment)
procedure Plan is
   Terms : Table.Term_Vectors.Vector;
   Solved : Boolean;
begin
   Note := (others => <>);
   Aim (Terms);
   Budget (Terms, Solved);
   --  🔴🔴 "算出来了,而算出来的是一动不动" 和 "算不出来" 是同一件事(HZ 2026-09-15 实测)。
   --  HZ:6 根通道被判死 4 根,活下来的两根左右都是 0.000 ⇒ 解算每一步都返回全零、还报成功,
   --  于是身体连着 10 步一根关节都没转,差距 0.479 → 0.565(还涨了),而日志每一行都绿。
   --  一步命令全零 = 这一步没走。不许把它当成"走过了"。
   if Solved and then (for all K in 0 .. Chan.Per_Arm - 1 => abs Note.Cmd (K) <= 0.0) then
      Solved := False;
   end if;
   if not Solved then
      --  🔴 解不出来也不许停:用手上最好的那个估计推一步,并说清楚这一步是硬凑的。
      --  🔴 挑哪一根:能用的里面【画面里动得最多】的那一根 —— 以前写死推 0 号,
      --  0 号不能用就等于什么都不推,"不许停"变成了一句空话(HZ 实测全零 10 步)。
      for K in 0 .. Chan.Per_Arm - 1 loop
         Note.Cmd (K) := 0.0;
      end loop;
      declare
         Best : Integer := -1;
         Best_Px : Long_Float := -1.0;
      begin
         for K in 0 .. Chan.Per_Arm - 1 loop
            if Note.Active (K) then
               declare
                  Px : Long_Float := 0.0;
               begin
                  for T of Terms loop
                     Px := Long_Float'Max
                       (Px, Sqrt (T.E.B (K, 0) ** 2 + T.E.B (K, 1) ** 2));
                  end loop;
                  if Px > Best_Px then
                     Best_Px := Px;
                     Best := K;
                  end if;
               end;
            end if;
         end loop;
         --  一根能用的都没有 ⇒ 还是要动:推身上量到过幅度的第一根,并说清楚这是硬凑的。
         if Best < 0 then
            for K in 0 .. Chan.Per_Arm - 1 loop
               if C.Map.Seen (Arm * Chan.Per_Arm + K) then
                  Best := K;
                  exit;
               end if;
            end loop;
         end if;
         if Best >= 0 then
            Note.Cmd (Natural (Best)) :=
              C.Map.Amp (Arm * Chan.Per_Arm + Natural (Best)) * Amount;
            Note.Active (Natural (Best)) := True;
         end if;
      end;
      C.Blind_Say := S ("I could not work out which channels to push, so this step was a guess");
      return;
   end if;
   Trim;
end Plan;
