with Kinem;
package body Readings is

   procedure Verdict_Of (A1, B1, A2, B2 : Buf; F : Picture.Floor_Map; W, H : Natural; V : out Eye_Verdict; Strong, Changed : out Quad_Counts) is
      N : constant Natural := W * H;
   begin
      Strong := [others => 0]; Changed := [others => 0];
      V := Nothing;
      if N = 0 or else Natural (A1.Length) /= N or else Natural (B1.Length) /= N
        or else Natural (A2.Length) /= N or else Natural (B2.Length) /= N
      then
         return;   --  有一帧没画面 / 不一样大:判不了,不说动了
      end if;
      declare
         Mv : constant Bools := Picture.Both (Picture.Moved (A1, B1, F), Picture.Moved (A2, B2, F));
         --  全仓那一张格子(Kinem.Gx × Kinem.Gy);画面比格子还窄的那一维,一个像素一格
         Ncx : constant Positive := Natural'Max (1, Natural'Min (Kinem.Gx, W));
         Ncy : constant Positive := Natural'Max (1, Natural'Min (Kinem.Gy, H));
         Cw : constant Positive := Natural'Max (1, W / Ncx);
         Ch : constant Positive := Natural'Max (1, H / Ncy);
         Tex : array (0 .. Ncx * Ncy - 1) of Natural := [others => 0];
         Chg : array (0 .. Ncx * Ncy - 1) of Boolean := [others => False];
         Tex_Chg : Floats;
      begin
         --  看没看见动了:同一个判法(两次比较都超过地板的像素连成的块,不小于最小连通块)
         if Picture.Components (Mv, W, H, Picture.Min_Pixels (W, H)).Is_Empty then
            return;
         end if;
         for J in 0 .. Ncy - 1 loop
            for I in 0 .. Ncx - 1 loop
               declare
                  Cell : constant Natural := J * Ncx + I;
               begin
                  for Y in J * Ch .. Natural'Min (H, (J + 1) * Ch) - 1 loop
                     for X in I * Cw .. Natural'Min (W, (I + 1) * Cw) - 1 loop
                        declare
                           K : constant Natural := Y * W + X;
                           P : constant Integer := Integer (A1 (K));
                        begin
                           if Mv (K) then
                              Chg (Cell) := True;
                           end if;
                           if X + 1 < W then
                              Tex (Cell) := Natural'Max (Tex (Cell), abs (Integer (A1 (K + 1)) - P));
                           end if;
                           if Y + 1 < H then
                              Tex (Cell) := Natural'Max (Tex (Cell), abs (Integer (A1 (K + W)) - P));
                           end if;
                        end;
                     end loop;
                  end loop;
                  if Chg (Cell) then
                     Tex_Chg.Append (Long_Float (Tex (Cell)));
                  end if;
               end;
            end loop;
         end loop;
         if Tex_Chg.Is_Empty then
            return;
         end if;
         declare
            --  这一推能让哪些格变:纹理不比"变了的格"的中位弱的那些(中位:一半变了的格比它弱、一半比它强)
            G_Star : constant Long_Float := Picture.Quantile (Tex_Chg, 0.5);
            N_Strong, N_Changed, Places : Natural := 0;
         begin
            for J in 0 .. Ncy - 1 loop
               for I in 0 .. Ncx - 1 loop
                  declare
                     Cell : constant Natural := J * Ncx + I;
                     --  象限:画面横竖各分两半(0 左上、1 右上、2 左下、3 右下)
                     Q : constant Natural := (if J < Ncy / 2 then 0 else 2) + (if I < Ncx / 2 then 0 else 1);
                  begin
                     if Long_Float (Tex (Cell)) >= G_Star then
                        Strong (Q) := Strong (Q) + 1;
                        if Chg (Cell) then
                           Changed (Q) := Changed (Q) + 1;
                        end if;
                     end if;
                  end;
               end loop;
            end loop;
            for Q in Quad_Counts'Range loop
               N_Strong := N_Strong + Strong (Q); N_Changed := N_Changed + Changed (Q);
               if Changed (Q) > 0 then
                  Places := Places + 1;
               end if;
            end loop;
            --  这只眼看见的大半是世界:够强的格里变了的不少于没变的 = 世界在这只眼里挪了(整幅);少于 = 世界没挪、只一块在动。
            --  整幅还要不止一处在变(一个象限以上):纹理全挤在一处时,一大块零件和整幅挪分不开 ⇒ 推大一点再看。
            --  (原来按象限:每个象限都要过半才算整幅 —— 腕眼里自己的手指 / 手占了一个象限、跟着眼不动,人形 H4 第 1 只手那一推
            --  右下象限 1 / 21,整幅判不出来,推多大都一样)
            V := (if N_Changed < N_Strong - N_Changed then Part elsif Places > 1 then Whole else Undecided);
         end;
      end;
   end Verdict_Of;

   function Verdict (A1, B1, A2, B2 : Buf; F : Picture.Floor_Map; W, H : Natural) return Eye_Verdict is
      V : Eye_Verdict;
      S, C : Quad_Counts;
   begin
      Verdict_Of (A1, B1, A2, B2, F, W, H, V, S, C);
      return V;
   end Verdict;

   function Image (V : Eye_Verdict) return String is
     (case V is
         when Nothing => "没看见动",
         when Whole => "整幅在动",
         when Part => "只一块在动",
         when Undecided => "分不出(推得太小)");
end Readings;
