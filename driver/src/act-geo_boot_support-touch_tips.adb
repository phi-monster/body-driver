separate (Act.Geo_Boot_Support)
procedure Touch_Tips (A, Hc : Natural) is
   Z : constant Zone.Hand_Zone := Zone_Of (C, A, Hc);
   Cw : constant Natural := F.Cams (Hc).W;
   Ch : constant Natural := F.Cams (Hc).H;
   G0 : constant Geom.Cam_Geo := C.Geo (Hc);   --  碰之前那份(身体文件里的指尖,没核过)
   Home : constant Plug.Arm_Pose := F.EE (A);
   --  量到的那张面的法向(朝眼那边):往面里压 = 逆着它,抬 = 顺着它。不按协议的"上"(Protocol_Up):面是墙、是地面一样压得了
   Nb0 : constant Geom.V3 := C.Board_N;
   Into : constant Geom.V3 := [-Nb0 (0), -Nb0 (1), -Nb0 (2)];
   --  这只手以前轻碰时确定空走的那几档各少走多少(核下一回轻碰的第一档是不是已经压着;09-29 V1B66 第 2 只手头一下:粗找压深了,
   --  抬 4.4 mm 手指只离开 0.7 mm,轻碰当"空走的底"的是压着的那一档,11 档都没认出)
   Notch_Pool : aliased Floats;
   --  中位(拷一份插入排序;几十个数)
   function Median_Of (V : Floats) return Long_Float is
      W : Floats := V;
   begin
      for I in 1 .. Natural (W.Length) - 1 loop
         declare
            X : constant Long_Float := W (I);
            J : Integer := I - 1;
         begin
            while J >= 0 and then W (J) > X loop
               W.Replace_Element (J + 1, W (J));
               J := J - 1;
            end loop;
            W.Replace_Element (J + 1, X);
         end;
      end loop;
      return (if W.Is_Empty then 0.0 else W (Natural (W.Length) / 2));
   end Median_Of;
   --  离中位的散布:中位绝对偏差 × 1.4826(正态下等于标准差;统计换算常数,无量纲)
   function Spread_Of (V : Floats; Med : Long_Float) return Long_Float is
      Dv : Floats;
   begin
      for X of V loop
         Dv.Append (abs (X - Med));
      end loop;
      return 1.4826 * Median_Of (Dv);
   end Spread_Of;
   Tu, Tv, Nw, Nt : Floats;   --  每一瓣指尖的像素、指尖那一小截的像素跨度(宽的那个 / 窄的那个)
   D : Geom.V3_Vectors.Vector;   --  每一瓣指尖的相机系单位视线
   Lobe_At : Geom.Nat_Vectors.Vector;   --  和 D 一一对应:它是握区里的第几瓣(认不出尖的那一瓣不进 D,下标会错开)
   Nl : Natural := 0;
   Who : constant String := "第" & Codec.Img (A + 1) & " 只手";
   Top_H : Long_Float := Long_Float'First;  --  压之前到过的最高处(眼沿面的法向有多高)
   Limit : Boolean := False;   --  上一次 Press_At:压到一半这一压在量到的关节限位里解不出来(停下不是碰到)
   Eqs : Geom.Press_Eq_Vectors.Vector;   --  这只手压过的每一下(顶住那一刻的方程)
   Eq_Lobe : Geom.Nat_Vectors.Vector;    --  每一下对准的是第几瓣
   Est : Geom.V3_Vectors.Vector;         --  每一瓣的尖此刻的估计(相机系;Has_Est 为假 ⇒ 没有,按它的视线交面)
   Has_Est : Bools;
   Gate : constant Long_Float := 4.0 * Geo_Base (C, A);   --  一小步 = 4 倍最小一档(同 Geo_Go 默认的一压、同原来两处对不对得上的门)
   Small : constant Long_Float := 4.0 * Geo_Base (C, A);  --  同上(压的时候的一小步)
   Deg : constant := 0.0174532925199433;   --  1° 的弧度(换算,无量纲)
   N_Tilt : constant := 5;      --  斜着压的下数(次数)
   --  斜着压只朝这一瓣手指身子的反方向那半边斜:身子在画面里从尖往它伸进画面的那一头去(Zone.Lobe_Entry),朝反方向那半边(±90° 以内)斜时,
   --  身子上每一点离"朝下"只会更远、只会更高(纯几何:斜 θ 朝 w 时,方向 u 上的一点离朝下 |αu − θw|,u·w ≤ 0 ⇒ 不比 α 小)。
   --  09-30 V1B79 第 2 只手第 2 瓣:原来一圈五个方位各差 72°,朝手指身子那边斜的那几下(36°–144°)板的另一头沉得比尖还低
   --  (拍 1218:尖离面 2.026,身子上一点 2.011、离眼比尖还远),压在东西上,8 下只有 3 下碰的是尖。五下均匀铺在那半边:各差 180° ÷ 5
   Az_Step : constant Long_Float := Ada.Numerics.Pi / Long_Float (N_Tilt);
   Az0 : Floats;   --  和 D 一一对应:这一瓣手指身子反方向的方位(Geom.Azim_Of,Tilt_Dir 的量法)
   N_Extra : constant := 2;     --  对不上时补压的下数(次数;方位取前两个方位中间的)
   function Dot (P, Q : Geom.V3) return Long_Float is (P (0) * Q (0) + P (1) * Q (1) + P (2) * Q (2));
   --  世界里一点沿面的法向落到面上
   function Proj (Q : Geom.V3) return Geom.V3 is
      Hq : constant Long_Float := Dot ([Q (0) - C.Board_Pt (0), Q (1) - C.Board_Pt (1), Q (2) - C.Board_Pt (2)], C.Board_N);
   begin
      return [Q (0) - Hq * C.Board_N (0), Q (1) - Hq * C.Board_N (1), Q (2) - Hq * C.Board_N (2)];
   end Proj;
   --  爪子张开那头的命令(每个抓握通道一个;开机推到头量的读数,Zone.Measure 停在那一头时发的就是它)。量过的通道按它,没量过的保持此刻的读数
   function Open_Cmd return Floats is
      Cur : constant Floats := Selfmap.Jaw_All (F, A);
      R : Floats;
   begin
      for K in 0 .. Natural (Cur.Length) - 1 loop
         declare
            V : Long_Float := Cur (K);
         begin
            for Hh of C.Hands loop
               if Hh.Arm = A and then Hh.K = K and then Hh.Measured then
                  V := Hh.Open_Reading;
               end if;
            end loop;
            R.Append (V);
         end;
      end loop;
      return R;
   end Open_Cmd;
   Jaw_Open : constant Floats := Open_Cmd;
   --  在板上一块空的面上压一下:让手上 Tilt_Dir (这一瓣的视线, Tilt, Azim) 那个方向朝正下(绕眼转,转最少),
   --  按转完以后这一瓣的尖(估计)落在面上的点、别的瓣的视线落在面上的点(再平移 Shift)找一块空的面,
   --  转和挪一条命令走完(反解在量到的关节限位里解;V1B21 2026-09-27:原来按"一条命令转得到的最大一档"一步一步转完再挪,一瓣要 7–8 条命令);
   --  挪完核这一瓣的尖落在哪:离挑好的那块超过指尖那一小截的宽 ⇒ 没转到或没挪到,这一下不压(V1B21:挪 0.22 m 没走到、朝向也被带歪约 30°)。
   --  压到被顶住以后不再往下顶、让手歇下来再读位姿(V1B21 仿真真值:顶着的时候手指压进桌面 6.6 mm,命令一换成停在此刻两拍后回到 2.5 mm)。
   --  Got = 真顶住了(那一刻的方程记进 Eqs、对准第 K 瓣);S_Ray = 此刻这一瓣的视线交面离眼多远(朝下那一下给后面几下当"尖大概在哪"的起点)
   --  Lifted / H_First / Prev_Off:转的时候手指撑在面上、抬起来再转的那几回(09-28 H4,见下面"挪到了没有"),已经抬了多少、
   --  第一回那一刻眼离面多高、上一回挪完落点差多少(这一回没比上一回近 = 抬了没用,挡住它的不是面)。
   --  Seen:压之前刚在此刻这个位姿看过底下、看见挡着原来挑的那一处(见下面"压之前先看底下"),这一回是从这儿重挑的
   procedure Press_At (K : Natural; Tilt, Azim : Long_Float; Shift : Geom.V3; Got : out Boolean; S_Ray : out Long_Float;
                       Lifted : Long_Float := 0.0; H_First : Long_Float := -1.0; Prev_Off : Long_Float := Long_Float'Last;
                       Seen : Boolean := False) is
      P : constant Plug.Arm_Pose := F.EE (A);
      Gk : constant Geom.Cam_Geo := C.Geo (Hc);
      O : constant Geom.V3 := Geom.Cam_Pos (Gk, P);
      Rp : constant Geom.M3 := Geom.Cam_R (Gk, P);
      Nb : constant Geom.V3 := C.Board_N;
      H : constant Long_Float := Dot ([O (0) - C.Board_Pt (0), O (1) - C.Board_Pt (1), O (2) - C.Board_Pt (2)], Nb);
      Rv : constant Geom.V3 := Geom.Turn_To (Geom.Ap (Rp, Geom.Tilt_Dir (D (K), Tilt, Azim)), Into);
      Rr : constant Geom.M3 := Geom.Rodrigues (Rv);
      Ang : constant Long_Float := Geom.Norm (Rv);
      Dk : constant Geom.V3 := Geom.Ap (Rr, Geom.Ray (Gk, P, Tu (K), Tv (K)));   --  转完以后这一瓣的视线(绕眼转,眼不动)
      Ck : constant Long_Float := -Dot (Nb, Dk);   --  它和朝下的夹角的余弦
      --  转完以后这一瓣的尖(相对眼,世界系):有估计按估计;没有 ⇒ 按它的视线交面(视线朝正下时就是正下方那一点)
      Tk : constant Geom.V3 := (if Has_Est (K) then Geom.Ap (Rr, Geom.Ap (Rp, Est (K)))
                                elsif Ck > 1.0e-9 then [H / Ck * Dk (0), H / Ck * Dk (1), H / Ck * Dk (2)] else [0.0, 0.0, 0.0]);
      A0 : constant Geom.V3 := Proj ([O (0) + Tk (0), O (1) + Tk (1), O (2) + Tk (2)]);   --  这一瓣的尖碰到面时落在哪
      Lp : Geom.V3_Vectors.Vector;
      Tb : Floats;   --  别的瓣:从压的这一瓣的尖到它的尖那条连线的坡度(两根手指一样长时,Board_Free_Spots 算它的手指离面多高)
      Dl : Geom.V3 := [0.0, 0.0, 0.0];
      All_Hit : Boolean := Ck > 1.0e-9;
      --  指尖那一小截的宽(像素)落到尖那么远:有估的尖按它离眼多远(量的);没有 ⇒ 按眼离面多高(尖在眼和面之间,上限)。
      --  09-30 V1B77:原来一直按眼离面多高,压不成、抬高以后圈跟着变大(0.798 单位),那一带一处空地都挑不出
      R : constant Long_Float := Nw (K) * (if Has_Est (K) then Long_Float'Min (Long_Float'Max (0.0, H), Geom.Norm (Est (K))) else Long_Float'Max (0.0, H)) / Gk.F;
      Spot : Geom.V3;
      Aim_O : Geom.V3 := [0.0, 0.0, 0.0];
      Moved : Boolean := True;   --  这一回转和挪那条命令真要动(超过这只手平移、转动各自一步看得见的那一档)
   begin
      Got := False; S_Ray := 0.0; Limit := False;
      Lp.Append (Geom.V3'[A0 (0) + Shift (0), A0 (1) + Shift (1), A0 (2) + Shift (2)]);
      Tb.Append (0.0);
      for J in 0 .. Nl - 1 loop
         if J /= K then
            declare
               Dj : constant Geom.V3 := Geom.Ap (Rr, Geom.Ray (Gk, P, Tu (J), Tv (J)));
               Cj : constant Long_Float := -Dot (Nb, Dj);
               Vt : constant Geom.V3 := [Dj (0) - Dk (0), Dj (1) - Dk (1), Dj (2) - Dk (2)];
               Hz : constant Long_Float := Dot (Nb, Vt);
               Hv : constant Geom.V3 := [Vt (0) - Hz * Nb (0), Vt (1) - Hz * Nb (1), Vt (2) - Hz * Nb (2)];
               Lh : constant Long_Float := Geom.Norm (Hv);
               --  带子到另一根手指的尖为止:两根手指一样长(挑空地这一条本来就这么算)⇒ 它的尖离眼和这一瓣的尖一样远 —— 有估的尖按它,
               --  没有 ⇒ 这一瓣视线交面那么远(尖在眼和面之间,上限);也不超过它自己的视线交面。尖外面没有手指,东西再高也碰不着。
               --  09-30 V1B80:原来一直拉到它的视线交面(第一下眼离面 5.6 ⇒ 7.4 单位、约 40 cm),原位附近带子里全是东西,
               --  只挑得到 47 cm 外的空地,到了那儿被挡,剩下的空地在量到的关节限位里全解不出
               Sj : constant Long_Float := (if Cj > 1.0e-9 then Long_Float'Min (H / Cj, (if Has_Est (K) then Geom.Norm (Est (K)) elsif Ck > 1.0e-9 then H / Ck else H / Cj))
                                            else 0.0);
            begin
               All_Hit := All_Hit and then Cj > 1.0e-9;
               Lp.Append (Geom.V3'[A0 (0) + Sj * Hv (0) + Shift (0), A0 (1) + Sj * Hv (1) + Shift (1), A0 (2) + Sj * Hv (2) + Shift (2)]);
               Tb.Append ((if Lh > 0.0 then Long_Float'Max (0.0, Hz) / Lh else 0.0));
            end;
         end if;
      end loop;
      if H <= 0.0 or else not All_Hit then
         Geo_Say ("  眼在板的面之下、或手指的视线落不到面上(眼离面 " & Mm (H) & ")⇒ 这一下压不成");
         return;
      end if;
      --  挑落点:空的面按离得近排,一处一处先问反解(转和挪到那儿在量到的关节限位里解不解得出来,还差一步看得见的那一档以内才去);
      --  有尖的估计时,压到"尖在面以下一小步"那么低也要解得出来(V1B31 2026-09-27:往下走到指尖离桌面 5 mm 时到了关节限位、手停住被当成碰到)
      declare
         Ds : Geom.V3_Vectors.Vector;
         Found : Boolean := False;
         Asked : Natural := 0;
         Tol_P : constant Long_Float := Geo_Base (C, A);
         Tol_R : constant Long_Float := (if A * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (A * Chan.Per_Arm + 3) else 0.0);
      begin
         Board_Free_Spots (C, Lp, Tb, R, Ds);
         for D1 of Ds loop
            declare
               Av : Table.Vec := Table.Zero_Vec;
               Pe, Re : Long_Float;
               Rok : Boolean;
            begin
               for I in 0 .. 2 loop
                  Av (I) := Shift (I) + D1 (I);
                  Av (3 + I) := Rv (I);
               end loop;
               Plug.Reach (A, Chan.Compose (P, Av), Pe, Re, Rok);
               Asked := Asked + 1;
               if Rok and then Pe <= Tol_P and then Re <= Tol_R and then Has_Est (K) then
                  declare
                     Dz : constant Long_Float := Long_Float'Max (0.0, H - (-Dot (Nb, Tk) - Small));
                     Pe2, Re2 : Long_Float;
                     Rok2 : Boolean;
                     Av2 : Table.Vec := Av;
                  begin
                     for I in 0 .. 2 loop
                        Av2 (I) := Av (I) + Dz * Into (I);
                     end loop;
                     Plug.Reach (A, Chan.Compose (P, Av2), Pe2, Re2, Rok2);
                     Pe := Long_Float'Max (Pe, Pe2); Re := Long_Float'Max (Re, Re2);
                  end;
               end if;
               if not Rok or else (Pe <= Tol_P and then Re <= Tol_R) then
                  Dl := D1; Found := True;
                  exit;
               end if;
            end;
         end loop;
         if not Found then
            if Ds.Is_Empty then
               Geo_Say ("  板上量过、此刻还找得到的桌面里没有一处落点圈(半径 " & Mm (R) & ")整个在里面、又躲得开高出面的点 ⇒ 这一下压不成");
            else
               Geo_Say ("  板上空的面 " & Codec.Img (Natural (Ds.Length)) & " 处(指尖那一小截宽的上限 " & Mm (R) & "),问了 " & Codec.Img (Asked)
                        & " 处,转和挪到那儿在量到的关节限位里都解不出来 ⇒ 这一下压不成");
            end if;
            return;
         end if;
         if Asked > 1 then
            Geo_Say ("  空的面按远近问了 " & Codec.Img (Asked) & " 处,前 " & Codec.Img (Asked - 1) & " 处在量到的关节限位里解不出来,去第 " & Codec.Img (Asked) & " 处");
         end if;
      end;
      Spot := [Lp (0) (0) + Dl (0), Lp (0) (1) + Dl (1), Lp (0) (2) + Dl (2)];
      declare
         Av : Table.Vec := Table.Zero_Vec;
         Del : Table.Vec;
         Mok : Boolean;
      begin
         for I in 0 .. 2 loop
            Av (I) := Shift (I) + Dl (I);
            Av (3 + I) := Rv (I);
         end loop;
         Aim_O := Geom.Cam_Pos (Gk, Chan.Compose (P, Av));   --  这条命令要眼到的地方(下面核它有没有被顶高)
         declare
            Tol_R : constant Long_Float := (if A * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (A * Chan.Per_Arm + 3) else 0.0);
         begin
            Moved := Geom.Norm ([Av (0), Av (1), Av (2)]) > Geo_Base (C, A) or else Ang > Tol_R;
         end;
         --  爪子这一条命令里按张开那头发(复位以后爪子的目标作废,不发就跟着读数走)
         Step_Arm (L, C, F, A, Av, Jaw_Open, Del, Mok, Geo_Settle => True);
         Geo_Say ("  让手上" & (if Tilt > 0.0 then "这一瓣的视线朝方位 " & Codec.Fmt (Azim / Deg, 0) & "° 斜 " & Codec.Fmt (Tilt / Deg, 1) & "° 那个方向" else "这一瓣的视线")
                  & "朝下:转 " & Codec.Fmt (Ang, 3) & " rad、挪 (" & Mm (Av (0)) & "," & Mm (Av (1)) & "," & Mm (Av (2))
                  & ") 到板上空的那块,一条命令 ⇒ 实到转 " & Codec.Fmt (Sqrt (Del (3) ** 2 + Del (4) ** 2 + Del (5) ** 2), 3) & " rad、挪 ("
                  & Mm (Del (0)) & "," & Mm (Del (1)) & "," & Mm (Del (2)) & ")(指尖那一小截宽的上限 " & Mm (R) & ",眼离面 " & Mm (H) & ")"
                  & (if Mok then "" else " · 身体说没走成"));
      end;
      --  挪到了没有:按此刻的位姿重算这一瓣的尖落在面上哪儿,离挑好的那块超过手指宽上限 ⇒ 没挪到(够不着那么远),不在没核过的地方压
      declare
         P2 : constant Plug.Arm_Pose := F.EE (A);
         O2 : constant Geom.V3 := Geom.Cam_Pos (Gk, P2);
         Hok : Boolean := True;
         Q2 : constant Geom.V3 := (if Has_Est (K)
                                   then Proj (Geom.V3'[O2 (0) + Geom.Ap (Geom.Cam_R (Gk, P2), Est (K)) (0), O2 (1) + Geom.Ap (Geom.Cam_R (Gk, P2), Est (K)) (1),
                                                       O2 (2) + Geom.Ap (Geom.Cam_R (Gk, P2), Est (K)) (2)])
                                   else Geom.Hit_Plane (O2, Geom.Ray (Gk, P2, Tu (K), Tv (K)), C.Board_Pt, Nb, Hok));
         Off_By : constant Long_Float := Geom.Norm ([Q2 (0) - Spot (0), Q2 (1) - Spot (1), Q2 (2) - Spot (2)]);
         --  眼比命令的高多少(沿面的法向):手指一转就撑在面上、把手顶起来了(09-28 H4:人形手指约和眼离桌一样长,原地转向下,
         --  腕俯仰要到 1.60 只到 1.17、眼被顶高约 5 cm)
         Up_By : constant Long_Float := Dot ([O2 (0) - Aim_O (0), O2 (1) - Aim_O (1), O2 (2) - Aim_O (2)], Nb);
         H0 : constant Long_Float := (if H_First > 0.0 then H_First else H);
         Ln : constant Long_Float := Stride_Of (C, A);
      begin
         if not Hok or else Off_By > R then
            --  抬了没用(这一回落点没比上一回近)⇒ 挡住它的不是面:关节到了真尽头、或撞在别处(09-28 H5:第一只手腕俯仰顶在仿真的真尽头 1.609,
            --  反解不知道,照样往 1.7 以上解,手停在偏高的位姿,被当成撑在面上连抬 4 回、眼抬到离面 45 cm)
            if Up_By > Small and then Ln > 0.0 and then Lifted + Ln <= H0 and then Off_By < Prev_Off then
               --  被面顶起来了 ⇒ 沿法向抬一大步(步幅),从那儿把这一下重算一遍再转(抬的总量不超过第一回眼离面的高度:
               --  手指比那还长的身体这样碰不出来,下面照实说)
               Geo_Say ("  转的时候手指撑在面上了(眼比命令的高 " & Mm (Up_By) & ")⇒ 抬一大步(" & Mm (Ln) & ")再转");
               declare
                  Mok : Boolean;
               begin
                  Geo_Move (L, C, F, A, [Ln * Nb (0), Ln * Nb (1), Ln * Nb (2)], Mok);
               end;
               Press_At (K, Tilt, Azim, Shift, Got, S_Ray, Lifted + Ln, H0, Off_By);
               return;
            end if;
            Geo_Say ("  没挪到那块空的面(落点差 " & Mm (Off_By) & ",手指宽上限 " & Mm (R)
                     & (if Up_By > Small and then Lifted > 0.0 and then Off_By >= Prev_Off
                        then ",眼比命令的高 " & Mm (Up_By) & ",可抬了一大步落点没比上一回(差 " & Mm (Prev_Off) & ")近 ⇒ 挡住它的不是面"
                        elsif Up_By > Small then ",眼比命令的高 " & Mm (Up_By) & ",已经抬过 " & Mm (Lifted) & "、再抬就超过眼离面的高度 " & Mm (H0)
                        else "") & ")⇒ 这一下不压");
            return;
         end if;
      end;
      --  往下压(沿量到的那张面的法向压进去),两段:粗找(找到面在哪)+ 轻碰(在那儿读位姿)。
      --  每一步都等胳膊沿压的方向真停下来再读(Selfmap.Go 的 Press:动起来以后连着两拍挪不到这一步的百分之一 = 停了)。
      --  碰到没有 = Selfmap.Blocked:比上一步空走时多少走的量超过"这一步的百分之一 / 3 倍读数噪声 / 3 倍前两步空走之差"里最大的那样
      --  (09-29 台架:x5 两步空走少走的量前后只差约 1e-5 单位,碰上的第一步多少走至少 0.0014;原来的门 = 第一步 + 3 × 静止噪声,
      --  仿真读数不抖 ⇒ 门 = 第一步,差一丝就认成碰到:V1B60 虚认 37 次、V1B65 18 次)。
      --  粗找:有尖的估计时先一条命令下到"按估的尖算,离面三小步",再一小步一小步往下;这一下就被顶住了(比估的长)⇒ 抬两大步,按头一回的走法;
      --  小步下到"按估的尖算的桌面以下一大步"还没碰到(比估的短)⇒ 从那儿接着按大步压。头一回(没有估计):一大步一大步往下,碰到了
      --  ⇒ 退回碰到的那一大步开始的地方,等手指回过来(自己那只眼里画面停下;09-29 V1B66:大步压下去 21 mm,手指被顶开,退回后回弹约 5 拍,
      --  这期间第一小步碰上了却一点没少走),再一小步一小步找。
      --  轻碰:抬一小步 + 两档,等手指回过来,再一档一档往下;第一档比这只手以前确定空走的一档多少走得多 ⇒ 手指还压着(粗找压深了)
      --  ⇒ 再抬一小步重来;碰到 ⇒ 就在那一刻读位姿(不歇:位置控制下停在此刻卸不掉压着的那一点)
      declare
         Start : constant Plug.Arm_Pose := F.EE (A);
         Ln : constant Long_Float := Stride_Of (C, A);   --  一压 = 步幅
         Cap : constant Natural := (if Ln > 0.0 then Natural (Long_Float'Ceiling (H / Ln)) + 1 else 0);
         Notch : constant Long_Float := Geo_Base (C, A);
         Direct : Boolean := False;
         Coarse : Boolean := False;    --  粗找找到了面
         Touched : Boolean := False;   --  轻碰碰到了
         --  沿 Into 走一步 Lstep:先问反解(同 Geo_Go:位置还差超过这一步的一半、或朝向差超过转动一步看得见的那一档 = 到了量到的关节限位,不走)
         procedure Step_Down (Lstep : Long_Float; Short : out Long_Float; At_Limit : out Boolean) is
            Cur : constant Plug.Arm_Pose := F.EE (A);
            Av : Table.Vec := Table.Zero_Vec;
            Pe, Re : Long_Float;
            Rok, Mok : Boolean;
            Tol_R : constant Long_Float := (if A * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (A * Chan.Per_Arm + 3) else 0.0);
         begin
            Short := 0.0; At_Limit := False;
            for I in 0 .. 2 loop
               Av (I) := Lstep * Into (I);
            end loop;
            Plug.Reach (A, Chan.Compose (Cur, Av), Pe, Re, Rok);
            if Rok and then (Pe + Pe > Lstep or else (Tol_R > 0.0 and then Re > Tol_R)) then
               At_Limit := True;
               return;
            end if;
            --  等胳膊沿压的方向停下来再读(Press):被顶住的软手指那点转动蠕动不等;伸远了还在漂就接着等,不到固定拍数就读
            Geo_Move (L, C, F, A, [Av (0), Av (1), Av (2)], Mok, Press => True);
            declare
               Now : constant Plug.Arm_Pose := F.EE (A);
            begin
               Short := Lstep - ((Now (0) - Cur (0)) * Into (0) + (Now (1) - Cur (1)) * Into (1) + (Now (2) - Cur (2)) * Into (2));
            end;
         end Step_Down;
         --  等这只手自己那只眼里的画面停下来(被顶开的手指回过来),最多同 Go 的上限;用了几拍照说
         procedure Settle (Why : String) is
            Prev : Plug.Cam := F.Cams (Hc);
            Still : Natural := 0;
            Used : Natural := 0;
         begin
            for I in 1 .. 12 + C.Map.Settle loop
               exit when not Plug.Sense (L, F);
               Used := I;
               Still := (if Selfmap.Picture_Still (C.Map, Prev, F.Cams (Hc), Hc) then Still + 1 else 0);
               Prev := F.Cams (Hc);
               exit when Still >= 2;
            end loop;
            Geo_Say ("  " & Why & ":等自己那只眼里画面停下(手指回过来)用了 " & Codec.Img (Used) & " 拍" & (if Still >= 2 then "" else ",到上限还在动"));
         end Settle;
         --  一步一步往下(每步 Lstep,最多 Steps 步),按 Selfmap.Blocked 认碰到。Base0 给了 = 已经走过的一步空走的少走量(当第一个底);
         --  Free 给了 = 判成空走的每一步各少走多少
         procedure Descend (Lstep : Long_Float; Steps : Natural; Got_It : out Boolean; From : out Plug.Arm_Pose; Said : String;
                            Base0 : Long_Float := Long_Float'First; Free : access Floats := null) is
            Prev, Prev2, Sh : Long_Float := 0.0;
            N_Free : Natural := 0;
            Lim : Boolean;
         begin
            Got_It := False;
            From := F.EE (A);
            if Base0 /= Long_Float'First then
               Prev := Base0; N_Free := 1;
               if Free /= null then
                  Free.Append (Base0);
               end if;
            end if;
            for I in 1 .. Steps loop
               From := F.EE (A);   --  这一步开始的地方(碰到的那一步开始时手指还没碰到:上一步是空走的)
               Step_Down (Lstep, Sh, Lim);
               if Lim then
                  Limit := True;
                  Geo_Say ("  " & Said & ":再往下一步在量到的关节限位里解不出来(停下不是碰到)⇒ 这一下不算");
                  return;
               end if;
               if Selfmap.Blocked (Sh, Prev, Prev2, N_Free, Lstep, C.Map.EE_Noise) then
                  Got_It := True;
                  Geo_Say ("  " & Said & ":第 " & Codec.Img (I) & " 步(一步 " & Mm (Lstep) & ")少走 " & Mm (Sh) & ",空走时少走 " & Mm (Prev)
                           & "(门 " & Mm (Prev + Long_Float'Max (Selfmap.Negligible * Lstep,
                                                                3.0 * Long_Float'Max (C.Map.EE_Noise, (if N_Free >= 2 then abs (Prev - Prev2) else 0.0))))
                           & ")⇒ 碰到");
                  return;
               end if;
               Prev2 := Prev; Prev := Sh; N_Free := N_Free + 1;
               if Free /= null then
                  Free.Append (Sh);
               end if;
            end loop;
            Geo_Say ("  " & Said & ":往下 " & Codec.Img (Steps) & " 步(一步 " & Mm (Lstep) & ")都没认出碰到");
         end Descend;
         --  大步找:一大步一大步(一步 = 步幅)往下;碰到的那一大步开始的地方手指还没碰到 ⇒ 退回那儿、等手指回过来,
         --  再一小步一小步找(最多一大步那么深再多两步,次数)。小步往下一大步那么深都没碰着 ⇒ 大步那一下是虚的 ⇒ 从这儿接着大步往下,
         --  直到小步真碰着、到了量到的关节限位、或者眼走到面那么低(纯几何)。Base0 = 刚走过的一大步空走的少走量(压之前看底下那一步;
         --  给了 ⇒ 第一大步就有得比,见 Descend)
         procedure Big_Press (Base0 : Long_Float := Long_Float'First) is
            Fr : Plug.Arm_Pose;
            Hit : Boolean;
            B0 : Long_Float := Base0;
         begin
            loop
               Descend (Ln, Cap, Hit, Fr, "一大步一大步找", Base0 => B0);
               B0 := Long_Float'First;
               exit when not Hit;
               declare
                  Now : constant Plug.Arm_Pose := F.EE (A);
                  Mok : Boolean;
               begin
                  Geo_Move (L, C, F, A, [Fr (0) - Now (0), Fr (1) - Now (1), Fr (2) - Now (2)], Mok);
                  Settle ("退回碰到的那一大步开始的地方");
                  Descend (Small, Natural (Long_Float'Ceiling (Ln / Small)) + 2, Coarse, Fr, "退回碰到的那一大步开始的地方、一小步一小步找");
               end;
               exit when Coarse or else Limit;
               declare
                  On : constant Geom.V3 := Geom.Cam_Pos (Gk, F.EE (A));
                  Hn : constant Long_Float := Dot ([On (0) - C.Board_Pt (0), On (1) - C.Board_Pt (1), On (2) - C.Board_Pt (2)], Nb);
               begin
                  exit when Hn <= 0.0;
                  Geo_Say ("  小步往下一大步那么深都没碰着 ⇒ 大步那一下是虚的 ⇒ 接着按大步压(眼离面 " & Mm (Hn) & ")");
               end;
            end loop;
         end Big_Press;
         --  有尖的估计时一条命令下多少,到"按估的尖算,离面三小步"(按此刻的位姿)。三小步(次数:估的尖差一两小步时第一小步照样是空走的)。
         --  09-28 V1B51 试过两小步:第 1 只手第 2 瓣斜 216° 那一下第一小步就碰着了(当底的那一步坏了)⇒ 这一瓣差到 4.3 mm ⇒ 三小步
         function Drop_To_Est return Long_Float is
            P3 : constant Plug.Arm_Pose := F.EE (A);
            O3 : constant Geom.V3 := Geom.Cam_Pos (Gk, P3);
            H3 : constant Long_Float := Dot ([O3 (0) - C.Board_Pt (0), O3 (1) - C.Board_Pt (1), O3 (2) - C.Board_Pt (2)], Nb);
         begin
            return H3 - (-Dot (Nb, Geom.Ap (Geom.Cam_R (Gk, P3), Est (K))) + 3.0 * Small);
         end Drop_To_Est;
         --  压之前先看底下(09-30 V1B70 / V1B73):开机量的板只有不动的眼看得见、腕眼三角得出的那片,手自己挡着的那块没有板点 ——
         --  V1B70 / V1B73 第 1 只手底下那块是一台电子琴,挑空地只拿板点挡,另一瓣(V1B73 连压的那一瓣)压在琴上查不出。
         --  往下压的第一步本身就是一对立体像:Im0 / P0 = 走之前那一帧和位姿,此刻 = 走之后;两帧之间眼只平移(位姿读数量的)。
         --  问的点见 Look_Points(手指像素照样问)。比面高出的(Seen_Above_Of)进 C.Seen_Above;
         --  Blocked = 按新看见的点,挑好的这一处(Dl)不再是空的
         procedure Look_Below (Im0 : Plug.Cam; P0 : Plug.Arm_Pose; Blocked : out Boolean) is
            Q, M : Instrument.Match_Vectors.Vector;
            Err : Unbounded_String;
            Qu, Qv, Mu, Mv, Bu, Bv : Floats;
            New_Pts : Geom.Scene_Pt_Vectors.Vector;
            Matched, Tri : Natural;
            Sig : Long_Float;
            P1 : constant Plug.Arm_Pose := F.EE (A);
            Went : constant Long_Float := Geom.Norm ([Geom.Cam_Pos (Gk, P1) (0) - Geom.Cam_Pos (Gk, P0) (0), Geom.Cam_Pos (Gk, P1) (1) - Geom.Cam_Pos (Gk, P0) (1),
                                                      Geom.Cam_Pos (Gk, P1) (2) - Geom.Cam_Pos (Gk, P0) (2)]);
         begin
            Blocked := False;
            declare
               Ends : Geom.V3_Vectors.Vector;
            begin
               for J in 1 .. Natural (Lp.Length) - 1 loop
                  Ends.Append (Geom.V3'[Lp (J) (0) + Dl (0), Lp (J) (1) + Dl (1), Lp (J) (2) + Dl (2)]);
               end loop;
               Q := Look_Points (C, Gk, P0, Cw, Ch, Spot, Ends, R, Nt (K));
            end;
            M := Instrument.Match (To_String (C.Inst_Host), C.Inst_Port, Im0.RGB, Cw, Ch, F.Cams (Hc).RGB, Cw, Ch, Q, Err, Back => True);
            if Natural (M.Length) /= Natural (Q.Length) then
               Geo_Say ("  压之前看底下:仪器没配成(" & To_String (Err) & ")⇒ 这一处按原来知道的那份压");
               return;
            end if;
            for I in 0 .. Natural (Q.Length) - 1 loop
               Qu.Append (Q (I).U); Qv.Append (Q (I).V); Mu.Append (M (I).U); Mv.Append (M (I).V); Bu.Append (M (I).Bu); Bv.Append (M (I).Bv);
            end loop;
            Seen_Above_Of (C, Gk, P0, P1, Cw, Ch, Qu, Qv, Mu, Mv, Bu, Bv, New_Pts, Matched, Tri, Sig);
            for Pt of New_Pts loop
               C.Seen_Above.Append (Pt);
            end loop;
            if not New_Pts.Is_Empty then
               declare
                  Ds2 : Geom.V3_Vectors.Vector;
               begin
                  Board_Free_Spots (C, Lp, Tb, R, Ds2);
                  Blocked := not (for some D2 of Ds2 => Geom."=" (D2, Dl));   --  候选是同一批板点算的:没被挡就原样还在
               end;
            end if;
            --  被挡了 ⇒ 这一次新看见的点里挡住它的那几个(同 Board_Free_Spots 的挡法:落点圈里的;到别的瓣的带子里、高出那儿手指离面的);
            --  一个都没有 ⇒ 挡它的是以前看见的
            if Blocked then
               declare
                  N_Circle, N_Strip : Natural := 0;
                  Worst : Unbounded_String;
                  Worst_Over : Long_Float := Long_Float'First;
               begin
                  for Pt of New_Pts loop
                     declare
                        Hp : constant Long_Float := Dot ([Pt.Pw (0) - C.Board_Pt (0), Pt.Pw (1) - C.Board_Pt (1), Pt.Pw (2) - C.Board_Pt (2)], Nb);
                        Q : constant Geom.V3 := [Pt.Pw (0) - Hp * Nb (0) - Spot (0), Pt.Pw (1) - Hp * Nb (1) - Spot (1), Pt.Pw (2) - Hp * Nb (2) - Spot (2)];
                     begin
                        if Geom.Norm (Q) <= R then
                           N_Circle := N_Circle + 1;
                           if Hp > Worst_Over then
                              Worst_Over := Hp;
                              Worst := To_Unbounded_String ("落点圈里离落点 " & Mm (Geom.Norm (Q)) & "、高 " & Mm (Hp));
                           end if;
                        end if;
                        for J in 1 .. Natural (Lp.Length) - 1 loop
                           declare
                              Aj : constant Geom.V3 := [Lp (J) (0) - Lp (0) (0), Lp (J) (1) - Lp (0) (1), Lp (J) (2) - Lp (0) (2)];
                              Ln_J : constant Long_Float := Geom.Norm (Aj);
                              Rho : constant Long_Float := (if Ln_J > 0.0 then Long_Float'Max (0.0, Long_Float'Min (Ln_J, Dot (Q, Aj) / Ln_J)) else 0.0);
                              Side : constant Long_Float := (if Ln_J > 0.0 then Geom.Norm ([Q (0) - Rho * Aj (0) / Ln_J, Q (1) - Rho * Aj (1) / Ln_J, Q (2) - Rho * Aj (2) / Ln_J])
                                                             else Geom.Norm (Q));
                           begin
                              if Side <= R and then Hp >= Rho * Tb (J) and then Rho > 0.0 then
                                 N_Strip := N_Strip + 1;
                                 if Hp - Rho * Tb (J) > Worst_Over then
                                    Worst_Over := Hp - Rho * Tb (J);
                                    Worst := To_Unbounded_String ("到第 " & Codec.Img (J) & " 条带子里沿带子 " & Mm (Rho) & "、高 " & Mm (Hp) & "(那儿手指离面 " & Mm (Rho * Tb (J)) & ")");
                                 end if;
                              end if;
                           end;
                        end loop;
                     end;
                  end loop;
                  Geo_Say ("  挡住这一处的(这一次新看见的):落点圈里 " & Codec.Img (N_Circle) & " 个、带子里 " & Codec.Img (N_Strip) & " 个"
                           & (if N_Circle + N_Strip > 0 then ",最要紧的一个" & To_String (Worst) else " ⇒ 挡它的是以前看见的"));
               end;
            end if;
            Geo_Say ("  压之前看底下(往下第一步前后两帧,眼挪了 " & Mm (Went) & "):问 " & Codec.Img (Natural (Q.Length)) & " 个点(格点 + 落点圈和带子里密铺的)、配上 "
                     & Codec.Img (Matched) & " 个、交成且两帧对得上 " & Codec.Img (Tri) & " 个(配点噪声 " & Codec.Fmt (Sig, 2) & " px)、比面高出的 "
                     & Codec.Img (Natural (New_Pts.Length)) & " 个(看见的一共 " & Codec.Img (Natural (C.Seen_Above.Length)) & ")⇒ "
                     & (if Blocked then "挑好的这一处被挡了 ⇒ 退回去,从这儿重挑" else "这一处照样空"));
         end Look_Below;
         Look_Free : Long_Float := Long_Float'First;   --  压之前看底下那一步空走的少走量(大步找的第一大步拿它当底)
      begin
         Top_H := Long_Float'Max (Top_H, Start (0) * Nb0 (0) + Start (1) * Nb0 (1) + Start (2) * Nb0 (2));
         --  压之前先看底下(见 Look_Below):往下压的第一步 —— 大步找的第一大步;有尖的估计时是"下到尖离面约三小步"那一条命令的头一大步 ——
         --  走完了看。看见挡着挑好的这一处 ⇒ 退回去,从这儿重挑(Press_At 再来一遍;挡的点只多不少 ⇒ 这一处不会再被挑中,空地只会少,挑不到照实说)。
         --  刚看过、重挑挑中的又是原处(这一回没挪)⇒ 不再看。没配配点仪器 ⇒ 看不了,照原来知道的那份压
         if not (Seen and then not Moved) and then Length (C.Inst_Host) > 0 and then Ln > 0.0 and then Cw > 0 then
            declare
               Im0 : constant Plug.Cam := F.Cams (Hc);
               P0 : constant Plug.Arm_Pose := F.EE (A);
               First : constant Long_Float := (if Has_Est (K) and then Small > 0.0 then Long_Float'Min (Ln, Drop_To_Est) else Ln);
               Sh : Long_Float;
               Lim, Blocked, Mok : Boolean;
            begin
               if First > 0.0 then
                  Step_Down (First, Sh, Lim);
                  if Lim then
                     Limit := True;
                     Geo_Say ("  压之前看底下:往下第一步在量到的关节限位里解不出来(停下不是碰到)⇒ 这一下不算");
                  else
                     Look_Below (Im0, P0, Blocked);
                     if Blocked then
                        declare
                           Now : constant Plug.Arm_Pose := F.EE (A);
                        begin
                           Geo_Move (L, C, F, A, [P0 (0) - Now (0), P0 (1) - Now (1), P0 (2) - Now (2)], Mok);
                        end;
                        Press_At (K, Tilt, Azim, [0.0, 0.0, 0.0], Got, S_Ray, Seen => True);
                        return;
                     end if;
                     if First = Ln then
                        Look_Free := Sh;
                     end if;
                  end if;
               end if;
            end;
         end if;
         if not Limit and then Has_Est (K) and then Small > 0.0 and then Ln > 0.0 then
            declare
               P3 : constant Plug.Arm_Pose := F.EE (A);
               O3 : constant Geom.V3 := Geom.Cam_Pos (Gk, P3);
               H3 : constant Long_Float := Dot ([O3 (0) - C.Board_Pt (0), O3 (1) - C.Board_Pt (1), O3 (2) - C.Board_Pt (2)], Nb);
               Depth3 : constant Long_Float := -Dot (Nb, Geom.Ap (Geom.Cam_R (Gk, P3), Est (K)));   --  估的尖此刻在眼下多深
               Dn : constant Long_Float := Drop_To_Est;
               Mok : Boolean;
            begin
               if Dn > 0.0 then
                  declare
                     Jaw : Floats;
                     Del : Table.Vec;
                     Av : Table.Vec := Table.Zero_Vec;
                  begin
                     for I in 0 .. 2 loop
                        Av (I) := Dn * Into (I);
                     end loop;
                     Step_Arm (L, C, F, A, Av, Jaw, Del, Mok, Geo_Settle => True);
                     Look_Free := Long_Float'First;   --  手挪过了:看底下那一步的少走量不再是大步找的底
                     declare
                        Went : constant Long_Float := Del (0) * Into (0) + Del (1) * Into (1) + Del (2) * Into (2);
                     begin
                        Direct := Went + Geo_Base (C, A) >= Dn;
                        Geo_Say ("  按估的尖(离眼 " & Mm (Geom.Norm (Est (K))) & "、此刻在眼下 " & Mm (Depth3) & ")一条命令下 " & Mm (Dn) & " 到尖离面约三小步 ⇒ 实到 " & Mm (Went)
                                 & (if Direct then ",一小步一小步找" else ",这一下就被顶住了(比估的长)⇒ 抬两大步,按头一回的走法"));
                        if not Direct then
                           --  抬两大步(两 = 次数:被顶住时尖在面上或更低,抬一大步第一大步未必是空走的)
                           Geo_Move (L, C, F, A, [2.0 * Ln * Nb0 (0), 2.0 * Ln * Nb0 (1), 2.0 * Ln * Nb0 (2)], Mok);
                           Settle ("被顶住、抬两大步以后");
                        end if;
                     end;
                  end;
               else
                  Direct := True;
               end if;
            end;
         end if;
         if Limit then
            null;   --  看底下那一步就到了量到的关节限位:这一下不算(上面说过了),下面只抬起来
         elsif Direct then
            declare
               Fr : Plug.Arm_Pose;
            begin
               Descend (Small, Natural (Long_Float'Ceiling ((3.0 * Small + Ln) / Small)) + 1, Coarse, Fr, "一小步一小步找");   --  三小步 + 一大步那么深(次数)
            end;
            if not Coarse and then not Limit then
               Geo_Say ("  下到按估的尖算的桌面以下一大步还没碰到(比估的短)⇒ 接着按大步压");
               Big_Press;
            end if;
         else
            Big_Press (Look_Free);
         end if;
         --  粗找认成碰到、轻碰往下一小步那么深都没碰着 ⇒ 粗找那一下是虚的 ⇒ 细的(轻碰)否掉粗的,从这儿接着一小步一小步往下找
         --  (还没碰到 ⇒ 大步),直到轻碰真碰着、到了量到的关节限位、或者眼走到面那么低(眼到不了面以下,纯几何)
         loop
            exit when not Coarse;
            --  轻碰:抬一小步 + 两档(粗找多压不到一小步,纯几何),等手指回过来,再一档一档往下(最多抬的那么多再加一小步)
            declare
               Mok, Lim : Boolean;
               Up : constant Long_Float := Small + 2.0 * Notch;
               Lifted : Long_Float := Up;
               Fr : Plug.Arm_Pose;
               S1 : Long_Float := 0.0;
               Free_Now : aliased Floats;
            begin
               Geo_Move (L, C, F, A, [Up * Nb0 (0), Up * Nb0 (1), Up * Nb0 (2)], Mok);
               Settle ("抬起来以后");
               --  第一档:比这只手以前确定空走的一档多少走得多(同 Blocked 的门)⇒ 手指还压着 ⇒ 再抬一小步重来(最多多抬一大步)
               loop
                  Step_Down (Notch, S1, Lim);
                  exit when Lim;
                  declare
                     N_P : constant Natural := Natural (Notch_Pool.Length);
                     Med : constant Long_Float := (if N_P >= 3 then Median_Of (Notch_Pool) else 0.0);
                     Spr : constant Long_Float := (if N_P >= 3 then Spread_Of (Notch_Pool, Med) else 0.0);
                     Pressed : constant Boolean := N_P >= 3 and then Selfmap.Blocked (S1, Med, Med - Spr, 2, Notch, C.Map.EE_Noise);
                  begin
                     exit when not Pressed or else Lifted >= Up + Ln;
                     Geo_Say ("  轻碰第一档就少走 " & Mm (S1) & "(这只手确定空走的一档中位 " & Mm (Med) & ")⇒ 手指还压着(粗找压深了)⇒ 再抬一小步");
                     Geo_Move (L, C, F, A, [(Small + Notch) * Nb0 (0), (Small + Notch) * Nb0 (1), (Small + Notch) * Nb0 (2)], Mok);
                     Lifted := Lifted + Small;
                     Settle ("再抬一小步以后");
                  end;
               end loop;
               if Lim then
                  Limit := True;
                  Geo_Say ("  轻碰(一档一档):再往下一步在量到的关节限位里解不出来(停下不是碰到)⇒ 这一下不算");
               else
                  Descend (Notch, Natural (Long_Float'Ceiling ((Lifted + Small) / Notch)), Touched, Fr, "轻碰(一档一档)",
                           Base0 => S1, Free => Free_Now'Access);
                  if Touched then
                     --  确定空走的那几档进这只手的底子(碰到的前一档可能已经擦着,不要)
                     for I in 0 .. Natural (Free_Now.Length) - 2 loop
                        Notch_Pool.Append (Free_Now (I));
                     end loop;
                  end if;
               end if;
            end;
            exit when Touched or else Limit;
            declare
               On : constant Geom.V3 := Geom.Cam_Pos (Gk, F.EE (A));
               Hn : constant Long_Float := Dot ([On (0) - C.Board_Pt (0), On (1) - C.Board_Pt (1), On (2) - C.Board_Pt (2)], Nb);
               Fr : Plug.Arm_Pose;
            begin
               exit when Hn <= 0.0;
               Geo_Say ("  轻碰往下一小步那么深都没碰着 ⇒ 粗找那一下是虚的 ⇒ 从这儿接着一小步一小步找(眼离面 " & Mm (Hn) & ")");
               Descend (Small, Natural (Long_Float'Ceiling (Ln / Small)) + 2, Coarse, Fr, "接着一小步一小步找");   --  一大步那么深再多两步(次数)
            end;
            if not Coarse and then not Limit then
               Geo_Say ("  一大步那么深还没碰到 ⇒ 接着按大步压");
               Big_Press;
            end if;
         end loop;
         if Touched then
            --  碰到那一刻手上最低的那一点在面上 ⇒ 一条方程;碰到的是不是这一瓣的尖、是不是桌面,由几下对不对得上管(Fit_Presses)
            declare
               Pc : constant Plug.Arm_Pose := F.EE (A);
               Vs : Geom.Board_View_Vectors.Vector;
               Row : Geom.Plane_Tip_Vectors.Vector;
               Jr : constant Floats := Selfmap.Jaw_All (F, A);
               Jd : Long_Float := 0.0;   --  爪子读数离张开那头最多差多少(只记账:仿真里读数是上一拍命令的回声,手指被顶开 20% 行程以上它才跟着变)
            begin
               for I in 0 .. Natural'Min (Natural (Jr.Length), Natural (Jaw_Open.Length)) - 1 loop
                  Jd := Long_Float'Max (Jd, abs (Jr (I) - Jaw_Open (I)));
               end loop;
               Eqs.Append (Geom.Press_Of (C.Geo (Hc), Pc, C.Board_Pt, Nb));
               Eq_Lobe.Append (K);
               Vs.Append (Geom.Board_View'(Pose => Pc, U => Tu (K), V => Tv (K)));
               Row := Geom.Tips_On_Plane (C.Geo (Hc), Vs, C.Board_Pt, Nb, C.Board_Rms);
               if Row (0).Ok then
                  S_Ray := Row (0).S;
               end if;
               Got := True;
               Geo_Say ("  碰到:眼离面 " & Mm (-Eqs.Last_Element.B) & "、这一瓣的视线交面离眼 " & (if Row (0).Ok then Mm (S_Ray) else "交不到")
                        & " · 爪子读数离张开那头 " & Codec.Fmt (Jd, 4));
            end;
         end if;
         --  压完抬两压(次数)就走,不回压之前的高处:下一处从这个高度横挪过去(⑧ 的 (a));最高到过哪儿照样记着,最后回原处上方
         declare
            Mok : Boolean;
         begin
            Geo_Move (L, C, F, A, [2.0 * Ln * Nb0 (0), 2.0 * Ln * Nb0 (1), 2.0 * Ln * Nb0 (2)], Mok);
         end;
      end;
   end Press_At;
   --  压一下;压到一半在量到的关节限位里解不出来(不是没挑到空地)⇒ 沿面挪开 4 倍一压换两边各试一处(同原来第一处的挪法)
   Far0 : constant Long_Float := 4.0 * Small;   --  4 倍一压(倍数,无量纲)
   function Along (Dd : Long_Float) return Geom.V3 is
      Nb : constant Geom.V3 := C.Board_N;
      Ax : constant Geom.V3 := (if abs (Nb (0)) < abs (Nb (1)) then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);
      T0 : constant Geom.V3 := [Nb (1) * Ax (2) - Nb (2) * Ax (1), Nb (2) * Ax (0) - Nb (0) * Ax (2), Nb (0) * Ax (1) - Nb (1) * Ax (0)];
      Tn : constant Long_Float := Geom.Norm (T0);
   begin
      return (if Tn > 0.0 then [Dd * T0 (0) / Tn, Dd * T0 (1) / Tn, Dd * T0 (2) / Tn] else [0.0, 0.0, 0.0]);
   end Along;
   procedure Press_Try (K : Natural; Tilt, Azim : Long_Float; Got : out Boolean; S_Ray : out Long_Float) is
   begin
      Press_At (K, Tilt, Azim, [0.0, 0.0, 0.0], Got, S_Ray);
      for Try in 1 .. 2 loop   --  两边(次数)
         exit when Got or else not Limit;
         Geo_Say ("  压到一半在量到的关节限位里解不出来 ⇒ 挪开 " & Mm (Far0) & " 换一处再碰");
         Press_At (K, Tilt, Azim, Along ((if Try = 1 then Far0 else -2.0 * Far0)), Got, S_Ray);   --  第二次挪到另一边(从第一次那儿挪两倍,纯几何)
      end loop;
   end Press_Try;
   --  第 K 瓣按压过的几下解(对准它的几下进解,别的瓣的几下只当"它不许在面之下"核);解出来的尖要落在这一瓣看得见的手指上
   --  (Geom.Finger_View;这一瓣穿过画面、尖在画面外 ⇒ 不核)
   function Fit_Of (K : Natural) return Geom.Press_Fit is
      E : Geom.Press_Eq_Vectors.Vector;
      Through : Boolean;
      Mask : constant Bools := Zone.Lobe_Mask (Z, Zone.Lobe_Of (Z, Lobe_At (K)), Cw, Ch, Through);
   begin
      for I in 0 .. Natural (Eqs.Length) - 1 loop
         E.Append (Geom.Press_Eq'(A => Eqs (I).A, B => Eqs (I).B, Aimed => Eq_Lobe (I) = K));
      end loop;
      return Geom.Fit_Presses (E, Gate, (if Through then Geom.No_View else Geom.Finger_View'(G => G0, W => Cw, H => Ch, Mask => Mask)));
   end Fit_Of;
   function Aimed_At (K : Natural) return Natural is
      N : Natural := 0;
   begin
      for Q of Eq_Lobe loop
         if Q = K then
            N := N + 1;
         end if;
      end loop;
      return N;
   end Aimed_At;
begin
   for K in 0 .. Z.N_Lobes - 1 loop
      declare
         Lb : constant Zone.Lobe := Zone.Lobe_Of (Z, K);
         U, V, Wd, Wt : Long_Float;
         Ok : Boolean;
      begin
         Zone.Tip_Section (Z, Lb, Cw, Ch, U, V, Wd, Wt, Ok);
         if Ok and then Z.Valid then
            declare
               Dir_Ok : Boolean;
               Dc : Geom.V3 := Geom.Cam_Dir (G0, U, V, Dir_Ok);   --  相机系单位视线(去掉镜头畸变;去不了 ⇒ 这一瓣不要)
               Nn : constant Long_Float := Geom.Norm (Dc);
            begin
               if Dir_Ok and then Nn > 0.0 then
                  for I in 0 .. 2 loop
                     Dc (I) := Dc (I) / Nn;
                  end loop;
                  declare
                     Eu, Ev : Long_Float;
                     Eok, Dok : Boolean;
                  begin
                     Zone.Lobe_Entry (Z, Lb, Cw, Ch, Eu, Ev, Eok);
                     declare
                        De : constant Geom.V3 := (if Eok then Geom.Cam_Dir (G0, Eu, Ev, Dok) else Geom.V3'[0.0, 0.0, 0.0]);
                        Dd : constant Long_Float := De (0) * Dc (0) + De (1) * Dc (1) + De (2) * Dc (2);
                     begin
                        --  身子的方向 = 伸进画面那一头的视线扣掉沿尖那条视线的那一截;反方向那半边的正中
                        Az0.Append (Geom.Azim_Of (Dc, [-(De (0) - Dd * Dc (0)), -(De (1) - Dd * Dc (1)), -(De (2) - Dd * Dc (2))]));
                     end;
                  end;
                  D.Append (Dc);
                  Lobe_At.Append (K);
                  Tu.Append (U); Tv.Append (V);
                  Nw.Append (Wd);   --  指尖那一小截的像素跨度(不是整瓣:V1B21 整瓣 124 px 落到面上 90 mm,空的面挑到了半米外)
                  Nt.Append (Wt);
               else
                  Geo_Say (Who & ":第 " & Codec.Img (K + 1) & " 瓣的尖落在镜头模型够不到的地方 ⇒ 这一瓣不量");
               end if;
            end;
         end if;
      end;
   end loop;
   Nl := Natural (D.Length);
   if Nl = 0 then
      Geo_Say (Who & ":它自己眼里没量到手指的尖 ⇒ 指尖量不了(东西躺的面用标定板的)");
      return;
   end if;
   for K in 0 .. Nl - 1 loop
      Est.Append (Geom.V3'[S_Known * D (K) (0), S_Known * D (K) (1), S_Known * D (K) (2)]);
      Has_Est.Append (S_Known > 0.0);
   end loop;
   declare
      Tips : Geom.V3_Vectors.Vector;
      Fits : array (0 .. Nl - 1) of Geom.Press_Fit;
      Failed : Boolean := False;
   begin
      --  没核过的指尖不拿来补转手时的平移:支点一直是眼本身(转的时候每个指尖都在离眼 S 的球面上,不会比眼低 S 以上)
      declare
         G : Geom.Cam_Geo := C.Geo (Hc);
      begin
         G.Tip := [0.0, 0.0, 0.0]; G.Tip_Valid := False;
         C.Geo.Replace_Element (Hc, G);
      end;
      for K in 0 .. Nl - 1 loop
         exit when Failed;
         declare
            --  斜多少:这一瓣视线和最近的另一瓣视线夹角的三分之一;只有一瓣 ⇒ 这只手一条命令转得到的那一档(开机量的)
            Theta : constant Long_Float := Geom.Tilt_Angle (D, K, C.Geo (Hc).Stride_Rot);
            Got : Boolean;
            S1 : Long_Float;
            --  尖大概在哪:压到的几下里这一瓣的视线交面离眼最近的那一下(后面几下挑落点、快下多深都按它;解出来以后换成解的)。
            --  别的东西先顶住只会让手停得更高、交面显得更远,不会更近(Press_Eq 的"只错一边")⇒ 来了近一小步以上的就换成它。
            --  09-30 V1B75 第 1 只手第 2 瓣:朝下那一下压在约 7.6 cm 高的东西上(交面 3.432 单位,真的约 1.87),原来只信这一下,
            --  后面几下按它挑落点、在关节限位里解不出,6 下只压成 4 下(其中 3 下交面 1.856–1.889)⇒ 这一瓣量不成
            procedure Note_Ray (S : Long_Float) is
            begin
               if Got and then S > 0.0 and then (not Has_Est (K) or else S + Gate < Geom.Norm (Est (K))) then
                  if Has_Est (K) then
                     Geo_Say ("  这一下视线交面离眼 " & Mm (S) & ",比估的尖(" & Mm (Geom.Norm (Est (K))) & ")近一小步以上 ⇒ 估的尖换成它(停早了只会显得更远)");
                  end if;
                  Est.Replace_Element (K, Geom.V3'[S * D (K) (0), S * D (K) (1), S * D (K) (2)]);
                  Has_Est.Replace_Element (K, True);
               end if;
            end Note_Ray;
         begin
            if Theta <= 0.0 then
               Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣:只有这一瓣、一条命令转得到的那一档也没量 ⇒ 斜不了,这一瓣量不成");
               Failed := True;
            else
               Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣:让它指尖的视线朝下压 1 下、再朝它手指身子反方向那半边的五个方位(正中 "
                        & Codec.Fmt (Az0 (K) / Deg, 0) & "°、各差 " & Codec.Fmt (Az_Step / Deg, 0) & "°)各斜 " & Codec.Fmt (Theta / Deg, 1)
                        & "° 压 1 下(" & (if Nl >= 2 then "它和最近的另一瓣视线夹角 " & Codec.Fmt (3.0 * Theta / Deg, 1) & "° 的三分之一" else "一条命令转得到的那一档")
                        & ")⇒ 每一下手上最低那一点落在面上,几下一起解它在手系里在哪");
               --  压之前板上的点在不动的眼里重找一遍:这一瓣只在此刻还找得到的那片桌面上挑落点(09-28 V1B47:手把电子琴推进了板量过的那片)
               declare
                  Found : Natural;
                  Why : Unbounded_String;
               begin
                  Board_Recheck (F, C, Found, Why);
                  if Length (Why) = 0 then
                     Geo_Say ("  板上 " & Codec.Img (Natural (C.Board.Length)) & " 个点此刻在不动的眼里还找得到 " & Codec.Img (Found)
                              & " 个(找不到的当没量过:被挪来的东西盖住了、或者此刻被手挡着)");
                  else
                     Geo_Say ("  板这会儿没法在不动的眼里重找(" & To_String (Why) & ")⇒ 按上一回知道的那份挑落点");
                  end if;
               end;
               Press_Try (K, 0.0, 0.0, Got, S1);
               Note_Ray (S1);
               if Got and then S1 > 0.0 and then S_Known <= 0.0 then
                  S_Known := S1;
               end if;
               for I in 0 .. N_Tilt - 1 loop
                  Press_Try (K, Theta, Az0 (K) + Long_Float (2 * I - (N_Tilt - 1)) / 2.0 * Az_Step, Got, S1);   --  以正中为心左右铺开(一半,纯数学)
                  Note_Ray (S1);
               end loop;
               Fits (K) := Fit_Of (K);
               for E in 0 .. N_Extra - 1 loop
                  exit when Fits (K).Ok;
                  Geo_Say ("  第 " & Codec.Img (K + 1) & " 瓣压了 " & Codec.Img (Aimed_At (K)) & " 下:"
                           & (if Fits (K).Ambiguous then "有两组一样多、互相对不上(认不出哪一下是坏的)" else "找不到 4 下以上互相对得上的(3 个未知数 + 1 条自己核)")
                           & " ⇒ 补压一下(方位 " & Codec.Fmt ((Az0 (K) + (Long_Float (E) - 0.5) * Az_Step) / Deg, 0) & "°)");
                  Press_Try (K, Theta, Az0 (K) + (Long_Float (E) - 0.5) * Az_Step, Got, S1);   --  正中两边各半格(一半,纯数学)
                  Note_Ray (S1);
                  Fits (K) := Fit_Of (K);
               end loop;
               if not Fits (K).Ok then
                  Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣:压了 " & Codec.Img (Aimed_At (K)) & " 下,"
                           & (if Fits (K).Ambiguous then "认不出哪几下是坏的" else "找不到 4 下以上互相对得上的") & " ⇒ 这一瓣量不成");
                  Failed := True;
               else
                  declare
                     X : constant Geom.V3 := Fits (K).X;
                     Along_Ray : constant Long_Float := Dot (X, D (K));
                     Off_Ray : constant Long_Float := Geom.Norm ([X (0) - Along_Ray * D (K) (0), X (1) - Along_Ray * D (K) (1), X (2) - Along_Ray * D (K) (2)]);
                  begin
                     Est.Replace_Element (K, X);
                     Has_Est.Replace_Element (K, True);
                     Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣:压了 " & Codec.Img (Aimed_At (K)) & " 下、" & Codec.Img (Natural (Fits (K).Used.Length))
                              & " 下互相对得上(别的几下预测每一下最多差 " & Mm (Fits (K).Worst) & ",门 " & Mm (Gate) & ")⇒ 尖离眼 " & Mm (Geom.Norm (X))
                              & ",离它指尖那条视线 " & Mm (Off_Ray) & " · 不确定度 (" & Mm (Fits (K).Sd (0)) & "," & Mm (Fits (K).Sd (1)) & "," & Mm (Fits (K).Sd (2)) & ")");
                  end;
               end if;
            end if;
         end;
      end loop;
      --  每一瓣再按这只手压过的全部几下核一遍:别的瓣压的那几下它也不许在面之下(Fit_Presses 的组外核);对不上 ⇒ 如实说、不收
      if not Failed then
         for K in 0 .. Nl - 1 loop
            Fits (K) := Fit_Of (K);
            if not Fits (K).Ok then
               Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣:按全部 " & Codec.Img (Natural (Eqs.Length)) & " 下再核 ⇒ 别的瓣压的时候它落到了面之下(或认不出坏的那一下)⇒ 指尖这回量不成");
               Failed := True;
            elsif Geom.Ray_Owner (Fits (K).X, D) /= K then
               --  解出来的点离别的瓣的视线比离它自己的还近 ⇒ 几下碰着的是那一瓣(它长得多,斜着压时一直是它先碰到)
               Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣:解出来的尖离" & (if Geom.Ray_Owner (Fits (K).X, D) < Nl then "第 " & Codec.Img (Geom.Ray_Owner (Fits (K).X, D) + 1) & " 瓣" else "哪一瓣")
                        & "的视线比离它自己的还近 ⇒ 碰着的不是这一瓣;指尖这回量不成");
               Failed := True;
            else
               Tips.Append (Fits (K).X);
            end if;
         end loop;
      end if;
      if Failed then
         C.Geo.Replace_Element (Hc, G0);
         Geo_Say (Who & ":指尖这回没量成 ⇒ " & (if G0.Tip_Valid then "身体文件里那份照旧(没核过)" else "没有指尖"));
      else
         declare
            G : Geom.Cam_Geo := C.Geo (Hc);
            Tip : Geom.V3 := [0.0, 0.0, 0.0];
         begin
            for K in 0 .. Nl - 1 loop
               for I in 0 .. 2 loop
                  Tip (I) := Tip (I) + Tips (K) (I) / Long_Float (Nl);
               end loop;
            end loop;
            G.Tip := Tip; G.Tip_Valid := True; G.Tip_Touch := True;
            --  每一瓣的尖和尖那一截的截面(像素跨度 × 这一瓣的尖有多深 ÷ 焦距)进身体文件:接触集的手按它们来
            G.Lobes.Clear; G.Tip_Sd := 0.0;
            for K in 0 .. Nl - 1 loop
               declare
                  Dk : constant Long_Float := Long_Float'Max (0.0, -Tips (K) (2));
               begin
                  G.Lobes.Append (Geom.Lobe_Geo'(Tip => Tips (K), Wide => Nw (K) * Dk / G.F, Thin => Nt (K) * Dk / G.F));
                  for I in 0 .. 2 loop
                     G.Tip_Sd := Long_Float'Max (G.Tip_Sd, Fits (K).Sd (I));
                  end loop;
               end;
            end loop;
            if Nl = 2 then
               G.Gap := Geom.Norm ([Tips (0) (0) - Tips (1) (0), Tips (0) (1) - Tips (1) (1), Tips (0) (2) - Tips (1) (2)]);
            end if;
            C.Geo.Replace_Element (Hc, G);
            Geom.Save (To_String (C.Geo_Path), C.Geo);
            declare
               Say : Unbounded_String := To_Unbounded_String (Who & ":指尖碰桌面量好(换倾角碰,一共压了 " & Codec.Img (Natural (Eqs.Length)) & " 下)—— 每瓣离眼");
            begin
               for K in 0 .. Nl - 1 loop
                  Append (Say, " " & Mm (Geom.Norm (Tips (K))));
               end loop;
               Append (Say, " · 指尖中点离眼 " & Mm (Geom.Norm (Tip)) & (if Nl = 2 then " · 两指尖相距 " & Mm (G.Gap) else ""));
               if G0.Tip_Valid then
                  Append (Say, "(身体文件里那份:离眼 " & Mm (Geom.Norm (G0.Tip)) & "、张口 " & Mm (G0.Gap) & ",作废)");
               end if;
               Geo_Say (To_String (Say));
            end;
            --  每一瓣的尖(眼系,世界单位)落一行:打分脚本按它逐瓣和网格比
            for K in 0 .. Nl - 1 loop
               Geo_Say (Who & "第 " & Codec.Img (K + 1) & " 瓣的尖在眼系 (" & Codec.Fmt (Tips (K) (0), 5) & ", " & Codec.Fmt (Tips (K) (1), 5) & ", "
                        & Codec.Fmt (Tips (K) (2), 5) & ")");
            end loop;
            Head_Tip_Check (C, A, Hc, Tips);
         end;
      end if;
   end;
   --  回到原处上方(手指这会儿朝下:按原处的高度回去,手指会戳进面里;回到压之前最高的那一处的高度)
   declare
      Hh : constant Long_Float := Home (0) * Nb0 (0) + Home (1) * Nb0 (1) + Home (2) * Nb0 (2);
      Up_By : constant Long_Float := Long_Float'Max (0.0, Top_H - Hh);
   begin
      Go_Back (A, [Home (0) + Up_By * Nb0 (0), Home (1) + Up_By * Nb0 (1), Home (2) + Up_By * Nb0 (2), Home (3), Home (4), Home (5), Home (6)]);
   end;
end Touch_Tips;
