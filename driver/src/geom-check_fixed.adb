separate (Geom)
procedure Check_Fixed (G : in out Cam_Geo; Scene : Scene_Pt_Vectors.Vector; Now : Scene_Pt_Vectors.Vector; Best : in out Fixed_Best; Rep : out Fixed_Check;
                       Turn_Sd : Long_Float := 0.0; Base_Now : Integer := -1) is
   Cur : Scene_Pt_Vectors.Vector;
   Gn : Cam_Geo := G;
   Fr : Fixed_Report;
   Ok : Boolean;
   Deg : constant Long_Float := 180.0 / Ada.Numerics.Pi;   --  弧度 → 度(换算,无量纲)
   Reg_Now : Region_Counts := [others => 0];               --  此刻每一块里和现在的位姿对得上几个
   --  和位姿 Pg 对得上的点(门 Gt 以内)数一遍,再按点在 Pg 下该落在哪块分着数
   procedure Tally (Pg : Cam_Geo; Gt : Long_Float; Total : out Natural; Reg : out Region_Counts) is
   begin
      Total := 0; Reg := [others => 0];
      for P of Cur loop
         declare
            U, V : Long_Float;
            Front : Boolean;
         begin
            Project_Fixed (Pg, P.Pw, U, V, Front);
            if Front and then Sqrt ((U - P.U) ** 2 + (V - P.V) ** 2) <= Gt then
               Total := Total + 1;
               Add_Regions (Pg, U, V, Reg);
            end if;
         end;
      end loop;
   end Tally;
begin
   Rep := (Asked => Natural (Scene.Length), others => <>);
   for I in 0 .. Natural'Min (Natural (Scene.Length), Natural (Now.Length)) - 1 loop
      if Now (I).U >= 0.0 and then Now (I).V >= 0.0 then
         declare
            P : Scene_Pt := Scene (I);
         begin
            P.U := Now (I).U; P.V := Now (I).V;
            Cur.Append (P);
         end;
      end if;
   end loop;
   Rep.Matched := Natural (Cur.Length);
   if Gn.F <= 0.0 then
      return;   --  焦距都没有:没法核
   end if;
   --  三个候选:原来的位姿本身、从原位姿起步重解的、从零盲搜的(焦距已知 ⇒ 只解位姿)。拿同一道门数每个候选解释得了几个点:
   --  门 = 3 倍(倍数无量纲)"原来那份标定自己的像素残差"(它是按多细的配点解出来的,就按多细判;数值精度兜底 1e-9 px)。
   --  不按新解自己的残差定门:挡住的那半边 RoMa 不是乱配,是顺着看得见的那半边"编"出一片平滑的配点,一个错的位姿能以 4 px 的残差把它们全吃下,
   --  门跟着放到 12 px,就把"挡住"判成"挪了 7.6°、10.6 cm"(X5C 2026-09-25)。按原来那份的精度判,编出来的那片只有粗解得了,细的门里没有它
   --  新位姿的门另按"仪器转着看时配得多细"放宽到 max(细门, 3 倍 Turn_Sd):RoMa 转 90° 配点噪声约 0.8 px,标定时残差只有 0.16 px 的眼
   --  真被转了,按细门只数得到三成的点,会被"至少四分之一"那条挡掉(X5B 的数)。原来的位姿仍按细门数:小挪也抓得到
   declare
      Gate : constant Long_Float := 3.0 * Long_Float'Max (1.0e-9, G.Rms);
      Gate_New : constant Long_Float := Long_Float'Max (Gate, 3.0 * Turn_Sd);
      function Count (Pg : Cam_Geo; Gt : Long_Float) return Natural is
         K : Natural := 0;
      begin
         for P of Cur loop
            declare
               U, V : Long_Float;
               Front : Boolean;
            begin
               Project_Fixed (Pg, P.Pw, U, V, Front);
               if Front and then Sqrt ((U - P.U) ** 2 + (V - P.V) ** 2) <= Gt then
                  K := K + 1;
               end if;
            end;
         end loop;
         return K;
      end Count;
      Ga : Cam_Geo := G;
      Fa, Fb : Fixed_Report;
      Oka, Okb : Boolean;
      Na, Nb : Natural := 0;
   begin
      Tally (G, Gate, Rep.Consistent_Now, Reg_Now);
      Rep.Gate := Gate;
      Fit_Fixed_Board (Ga, Cur, Fa, Oka, Start_Here => True);
      Fit_Fixed_Board (Gn, Cur, Fb, Okb);
      if Oka then
         Na := Count (Ga, Gate_New);
      end if;
      if Okb then
         Nb := Count (Gn, Gate_New);
      end if;
      if Oka and then (not Okb or else Na >= Nb) then
         Gn := Ga; Fr := Fa; Ok := True; Rep.Consistent := Na;
      elsif Okb then
         Fr := Fb; Ok := True; Rep.Consistent := Nb;
      else
         Ok := False;
      end if;
   end;
   if not Ok then
      Rep.Covered := True;   --  配到的点和任何一个位姿都对不上:看不见了 / 挡住了
      return;
   end if;
   Rep.Rms := Fr.Scene_Rms;
   Rep.Turn_Deg := Norm (Rot_Vec (Mul (Tr (G.R_Ce), Gn.R_Ce))) * Deg;
   Rep.Move_M := Norm ([Gn.Pos (0) - G.Pos (0), Gn.Pos (1) - G.Pos (1), Gn.Pos (2) - G.Pos (2)]);
   --  新解把板上的点投到的地方比原位姿投到的挪了多少(报数用)
   declare
      Px : Param_Vec (0 .. Natural'Max (1, Natural (Cur.Length)) - 1) := [others => 0.0];
      N : Natural := 0;
   begin
      for P of Cur loop
         declare
            U0, V0, U1, V1 : Long_Float;
            F0, F1 : Boolean;
         begin
            Project_Fixed (G, P.Pw, U0, V0, F0);
            Project_Fixed (Gn, P.Pw, U1, V1, F1);
            if F0 and then F1 then
               Px (N) := Sqrt ((U1 - U0) ** 2 + (V1 - V0) ** 2);
               N := N + 1;
            end if;
         end;
      end loop;
      Rep.Shift_Px := Median (Px, N);
      Rep.Shift_Sd := (if G.Rms > 0.0 then Rep.Shift_Px / G.Rms else 0.0);
   end;
   --  挪过 = 三条都成立:原来的位姿解释得不到新解一半(比例);新解至少解释得了放好以来最多那次的四分之一(比例,同"挡住"那条的四分之三);
   --  新旧位姿投出来的板点差得比细门远(差不到门里 = 同一个位姿,只是这会儿配得糙)。
   --  看得见、配得上的不到四分之一时解出来的位姿不可信 —— X5C4 2026-09-26 转 90° 重标后再挡一半,仪器整幅配飞,此刻的位姿一个点都解释不了,
   --  一份错得离谱的位姿以 18.9 px 的残差在门里凑到 35/782 个,就被当成"挪了 0.84 m"换上了。三条不全 ⇒ 按挡没挡报,位姿不动
   if (if Base_Now >= 0 then 2 * Base_Now else 2 * Rep.Consistent_Now) < Rep.Consistent and then 4 * Rep.Consistent >= Best.All_N and then Rep.Shift_Px > Rep.Gate then
      Rep.Moved := True;
      Gn.F := G.F; Gn.F_Meas := G.F_Meas; Gn.F_Sd := G.F_Sd;   --  焦距照旧
      --  以后按新解配得多细来判:新位姿解释得了的那些点(新门内)像素误差的中位 × 1/√ln2(Board_Rms:二维高斯误差的均方根 ÷ 中位)。
      --  不拿解的时候那份没加权的均方根:加权挑点留下了三角得不准的点,它们的大误差把均方根抬到 1.31 px(标定时 0.16),门跟着放到 3.9 px(X5E2 2026-09-26)
      declare
         Gate_New : constant Long_Float := Long_Float'Max (3.0 * Long_Float'Max (1.0e-9, G.Rms), 3.0 * Turn_Sd);
         Br : constant Long_Float := Board_Rms (Gn, Cur, Gate_New);
      begin
         Gn.Rms := (if Br > 0.0 then Br else Fr.Scene_Rms);
         Rep.Rms := Gn.Rms;
      end;
      G := Gn;
      --  重新放好了:从这一刻起重记"看见过的最多"—— 按新位姿自己的门数(以后每轮按它数;09-27 以前记的是按放宽的新门数的那份,
      --  下一轮按自己的门数就少一截,V1B39 重标后 630 → 569,凭空离"挡住了"近了一截)
      declare
         N_Own : Natural;
         Reg_Own : Region_Counts;
      begin
         Tally (G, 3.0 * Long_Float'Max (1.0e-9, G.Rms), N_Own, Reg_Own);
         Best := (All_N => N_Own, Region => Reg_Own);
      end;
   else
      Best.All_N := Natural'Max (Best.All_N, Rep.Consistent_Now);
      for R in Reg_Now'Range loop
         Best.Region (R) := Natural'Max (Best.Region (R), Reg_Now (R));
      end loop;
      --  看不全 = 整幅比放好以来最多的少了四分之一以上(比例);或者画面的某一块(一半 / 四分之一;放好以来在那儿至少看见过 Min_Pts 个)
      --  少了四分之三以上 —— 那一块基本看不见了(比例)。09-27 V1B39:转过 90° 以后板上的点多在右边,挡住左半只挡掉整幅的 25%,整幅那条擦线没报;
      --  左半那一块其实一个都不剩
      Rep.Covered := 4 * Rep.Consistent_Now < 3 * Best.All_N;
      for R in Reg_Now'Range loop
         if Best.Region (R) >= Min_Pts and then 4 * Reg_Now (R) < Best.Region (R)
           and then (Rep.Dark < 0 or else Reg_Now (R) * Rep.Dark_Best < Rep.Dark_Now * Best.Region (R))   --  剩得比例最少的那块
         then
            Rep.Covered := True;
            Rep.Dark := R; Rep.Dark_Now := Reg_Now (R); Rep.Dark_Best := Best.Region (R);
         end if;
      end loop;
   end if;
end Check_Fixed;
