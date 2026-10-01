with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Contact.Search;
with Contact.Wrench;
with Contact.Qty;
with Contact.Surface;
separate (Selfcheck)
procedure Welds_Path_5 is
   --  路 5 的焊点(大并行.md §5 路 5):每条写清"错了会是什么病",带一颗牙(去掉那一改就红;10-01 每颗都真拆掉跑过,红的是哪几条写在各条注释里)
   --
   --  §2 第 16 条的前两件:物理检查按"要东西怎么动"(一个旋量)算要的力,不再只会托住重量;东西躺的那张面当成一处接触(托着它、有摩擦)。
   --  焊点矩阵第一批:东西(条、方块、圆柱、平放的剪刀)× 要怎么动(往上、沿面往 +x、绕竖轴转;外加往下 = 面挡着)× 手(两瓣、五指、没有手指)。
   --  两层:
   --   ① 物理检查本身(Contact.Wrench.Need):接触按那种手真会碰到的地方摆,和手算的解析解比(摩擦 μ = 0.5,力按单位重量);
   --   ② 搜索 + 物理(Contact.Search.Plan 带上要的动):同一件东西,要的动一换,挑中的那一组跟着换;几条对任何一组都成立的物理关系。
   --  手算的几条(μ = 0.5):
   --   往上:它离开面,只有手的摩擦往上托 ⇒ 法向力之和 ≥ 1/μ = 2;两处相对的接触正对着夹在重心两侧 = 2(线性化的锥第一条棱对准"上",这个方向上是准的)。
   --   沿面往 +x,有一处接触的法向正好 +x:它推(法向力 N)、它的摩擦顺手托一部分重量,面托剩下的、摩擦 μ × 面的压力往后拖
   --     ⇒ N = μ (1 − μN)… 最省力时 N = μ / (1 + μ²) = 0.4(推的方向正好穿过面的摩擦中心,不然转)。
   --   沿面往 +x,两处接触的法向都 ⊥ x(夹的方向和走的方向垂直):只能靠两处的摩擦拖着走 ⇒ 圆锥时 1 / √(1 + μ²) = 0.894,
   --     八条棱的锥内接,最多再大 1 / cos(π/8) 倍 = 0.968。
   --   绕竖轴(过重心)转:面按均匀压强给摩擦力矩 μ · 面的压力 · ρ̄(ρ̄ = 那一片离转轴的平均距离,贴着转轴的那几点不滑、不算);
   --     两处接触在转轴两侧 ±w 用摩擦给力偶、同时托住 (1 − 面的压力):最省力 = (ρ̄ / w) / √(1 + a²),a = μ ρ̄ / w;八条棱最多再大 1 / cos(π/8)。
   --     手掌从上面压下去(能拧的半径 r):μ r N = μ ρ̄ (1 + N) ⇒ N = ρ̄ / (r − ρ̄);r ≤ ρ̄ ⇒ 转不动。
   --   往下(它躺在面上)⇒ 面挡着,哪一组都做不到。
   use Ada.Numerics.Long_Elementary_Functions;
   package Wr renames Contact.Wrench;
   package Se renames Contact.Search;
   use type Wr.Why_Kind;
   Mu : constant Long_Float := 0.5;
   Pitch : constant Long_Float := 0.002;
   Up : constant Contact.V3 := [0.0, 0.0, 1.0];
   Zero3 : constant Contact.V3 := [0.0, 0.0, 0.0];
   Edge_Gap : constant Long_Float := 1.0 / Cos (Ada.Numerics.Pi / 8.0);   --  八条棱的内接锥比圆锥最多少这么多
   Eps : constant Long_Float := 1.0e-6;
   type Shape_Fn is access function (X, Y : Long_Float) return Boolean;
   --  平躺在面上(z = 0,上 = +z)的东西的表面点:顶面 z = Thick 上 2 mm 一格;轮廓那一圈往下每 2 mm 补一层侧壁到面(同驱动的 Walls_To_Support 的假设)。
   --  格点 = 间距的整数倍(对原点对称到最后一位:X0 + 间距 × i 这种铺法在浮点上左右不对称,x = −0.02 进了圆、+0.02 没进,重心和摩擦中心差出一点,
   --  单个手掌推就真的推不出纯平移 —— 那是物理算对了、形状不对称)
   function Slab (X0, X1, Y0, Y1, Thick : Long_Float; In_Shape : Shape_Fn) return Contact.V3_Vectors.Vector is
      V : Contact.V3_Vectors.Vector;
      I0 : constant Integer := Integer (Long_Float'Floor (X0 / Pitch));
      I1 : constant Integer := Integer (Long_Float'Ceiling (X1 / Pitch));
      J0 : constant Integer := Integer (Long_Float'Floor (Y0 / Pitch));
      J1 : constant Integer := Integer (Long_Float'Ceiling (Y1 / Pitch));
      Nz : constant Natural := Natural (Thick / Pitch);
   begin
      for I in I0 .. I1 loop
         for J in J0 .. J1 loop
            declare
               X : constant Long_Float := Pitch * Long_Float (I);
               Y : constant Long_Float := Pitch * Long_Float (J);
            begin
               if In_Shape (X, Y) then
                  V.Append (Contact.V3'([X, Y, Thick]));
                  if not In_Shape (X - Pitch, Y) or else not In_Shape (X + Pitch, Y) or else not In_Shape (X, Y - Pitch) or else not In_Shape (X, Y + Pitch) then
                     for K in 0 .. Nz - 1 loop
                        V.Append (Contact.V3'([X, Y, Pitch * Long_Float (K)]));
                     end loop;
                  end if;
               end if;
            end;
         end loop;
      end loop;
      return V;
   end Slab;
   function Com_Of (Pts : Contact.V3_Vectors.Vector) return Contact.V3 is
      C : Contact.V3 := Zero3;
   begin
      for Q of Pts loop
         for K in 0 .. 2 loop
            C (K) := C (K) + Q (K) / Long_Float (Pts.Length);
         end loop;
      end loop;
      return C;
   end Com_Of;
   function Bar (X, Y : Long_Float) return Boolean is (abs X <= 0.01 and then abs Y <= 0.1);
   function Block (X, Y : Long_Float) return Boolean is (abs X <= 0.02 and then abs Y <= 0.02);
   function Disc (X, Y : Long_Float) return Boolean is (X * X + Y * Y <= 0.02 ** 2);
   --  平放的剪刀:刀身从刀尖(y = −6 cm,半宽 2 mm)到转轴那头(y = 6.5 cm,半宽 6 mm),两个把手圈(外径 1.1 cm、内径 0.65 cm,圆心 (±1.3, 7.3) cm),厚 4 mm
   function Blade_W (Y : Long_Float) return Long_Float is (0.002 + 0.004 * (Y + 0.06) / 0.125);
   function Scissors (X, Y : Long_Float) return Boolean is
      function Ring (Cx, Cy : Long_Float) return Boolean is
        ((X - Cx) ** 2 + (Y - Cy) ** 2 <= 0.011 ** 2 and then (X - Cx) ** 2 + (Y - Cy) ** 2 >= 0.0065 ** 2);
   begin
      return (Y >= -0.06 and then Y <= 0.065 and then abs X <= Blade_W (Y)) or else Ring (-0.013, 0.073) or else Ring (0.013, 0.073);
   end Scissors;
   function Spin (Pivot : Contact.V3) return Contact.Twist is ((Lin => Zero3, Ang => Up, Pivot => Pivot));
   Along_X : constant Contact.Twist := Contact.Slide ([1.0, 0.0, 0.0]);
   Along_Y : constant Contact.Twist := Contact.Slide ([0.0, 1.0, 0.0]);
   Lift : constant Contact.Twist := Contact.Slide (Up);
   Down : constant Contact.Twist := Contact.Slide ([0.0, 0.0, -1.0]);
   --  转的时候面给的那份摩擦力矩按的平均半径:那一片每一点离转轴的水平距离,贴着转轴的不滑、不算 —— "贴着"同 Contact.Wrench 那一条判法:
   --  速度(角速度 1 × 离轴的距离)不超过 1 × 采样间距 + 算作零的 1e-12 × 离转轴那一点最远的距离(恰好离轴一个间距的点两边要分得一样)
   function Rho (Sup : Wr.Surface; Pivot : Contact.V3) return Long_Float is
      S : Long_Float := 0.0;
      Rmax : Long_Float := 0.0;
   begin
      for P of Sup.Foot loop
         Rmax := Long_Float'Max (Rmax, Sqrt ((P (0) - Pivot (0)) ** 2 + (P (1) - Pivot (1)) ** 2 + (P (2) - Pivot (2)) ** 2));
      end loop;
      for P of Sup.Foot loop
         declare
            D : constant Long_Float := Sqrt ((P (0) - Pivot (0)) ** 2 + (P (1) - Pivot (1)) ** 2);
         begin
            if D > Sup.Pitch + 1.0e-12 * Rmax then
               S := S + D;
            end if;
         end;
      end loop;
      return S / Long_Float (Sup.Foot.Length);
   end Rho;
   function T (P, N : Contact.V3; R : Long_Float := 0.0) return Wr.Touch is ((P => P, N => N, Twist_R => R));
   function Set_Of (A, B : Wr.Touch) return Wr.Touch_Vectors.Vector is
      V : Wr.Touch_Vectors.Vector;
   begin
      V.Append (A); V.Append (B);
      return V;
   end Set_Of;
   function One (A : Wr.Touch) return Wr.Touch_Vectors.Vector is
      V : Wr.Touch_Vectors.Vector;
   begin
      V.Append (A);
      return V;
   end One;
   function F4 (X : Long_Float) return String is (Codec.Fmt (X, 4));
   function Nw (X : Long_Float) return String is (if X = Wr.No_Way then "做不到" else F4 (X));
   function Always (R : Geom.M3; T : Contact.V3) return Boolean is
      pragma Unreferenced (R, T);   --  这几条焊点不考够不够得着
   begin
      return True;
   end Always;

   --  ① 物理检查本身 —— 一件东西:两处相对的接触(两瓣:沿 x 夹 / 沿 y 夹)、五处(五指)、手掌(没有手指:侧面推、侧面偏着推、上面压)
   procedure Physics (Name : String; Pts : Contact.V3_Vectors.Vector; Wx, Wy : Long_Float; Five : Wr.Touch_Vectors.Vector; Five_Exact_Spin : Boolean;
                      Palm_R, Top_Z, Top_Big, Top_Small : Long_Float; Pair_Y : Boolean) is
      Com : constant Contact.V3 := Com_Of (Pts);
      Sup : constant Wr.Surface := Wr.Footprint (Pts, Zero3, Up, Pitch);
      Rb : constant Long_Float := Rho (Sup, Com);
      Why : Wr.Why_Kind;
      Px : constant Wr.Touch_Vectors.Vector := Set_Of (T ([Com (0) - Wx, Com (1), Com (2)], [1.0, 0.0, 0.0], Palm_R), T ([Com (0) + Wx, Com (1), Com (2)], [-1.0, 0.0, 0.0], Palm_R));
      Py : constant Wr.Touch_Vectors.Vector := Set_Of (T ([Com (0), Com (1) - Wy, Com (2)], [0.0, 1.0, 0.0], Palm_R), T ([Com (0), Com (1) + Wy, Com (2)], [0.0, -1.0, 0.0], Palm_R));
      Side : constant Wr.Touch_Vectors.Vector := One (T ([Com (0) - Wx, Com (1), Com (2)], [1.0, 0.0, 0.0], Palm_R));
      Side_Off : constant Wr.Touch_Vectors.Vector := One (T ([Com (0) - Wx, Com (1) + 0.5 * Wy, Com (2)], [1.0, 0.0, 0.0], Palm_R));
      Top_B : constant Wr.Touch_Vectors.Vector := One (T ([Com (0), Com (1), Top_Z], [0.0, 0.0, -1.0], Top_Big));
      Top_S : constant Wr.Touch_Vectors.Vector := One (T ([Com (0), Com (1), Top_Z], [0.0, 0.0, -1.0], Top_Small));
      function N_Of (Ts : Wr.Touch_Vectors.Vector; M : Contact.Twist) return Long_Float is (Wr.Need (Ts, Com, Up, Sup, M, Mu, Mu, Why));
      Push : constant Long_Float := Mu / (1.0 + Mu * Mu);                  --  0.4
      Drag : constant Long_Float := 1.0 / Sqrt (1.0 + Mu * Mu);            --  0.894
      A_Sp : constant Long_Float := Mu * Rb / Wx;
      Couple : constant Long_Float := (Rb / Wx) / Sqrt (1.0 + A_Sp * A_Sp);
      Palm_Top : constant Long_Float := (if Top_Big > Rb then Rb / (Top_Big - Rb) else Wr.No_Way);
      L_Px, S_Px, Sy_Px, Sp_Px, D_Px, L_Py, S_Py, L_5, S_5, Sp_5, L_Sd, S_Sd, Sp_Sd, So_Sd, Sp_Tb, Sp_Ts : Long_Float;
      W_Px_Down, W_Sd_Up, W_Sp : Wr.Why_Kind;
   begin
      L_Px := N_Of (Px, Lift);
      S_Px := N_Of (Px, Along_X);
      Sy_Px := N_Of (Px, Along_Y);
      Sp_Px := N_Of (Px, Spin (Com));
      D_Px := N_Of (Px, Down); W_Px_Down := Why;
      L_5 := N_Of (Five, Lift);
      S_5 := N_Of (Five, Along_X);
      Sp_5 := N_Of (Five, Spin (Com));
      L_Sd := N_Of (Side, Lift); W_Sd_Up := Why;
      S_Sd := N_Of (Side, Along_X);
      Sp_Sd := N_Of (Side, Spin (Com)); W_Sp := Why;
      So_Sd := N_Of (Side_Off, Along_X);
      Sp_Tb := N_Of (Top_B, Spin (Com));
      Sp_Ts := N_Of (Top_S, Spin (Com));
      --  病:物理检查只会托住重量(不看要的动)⇒ 沿面、转都按 2 算;面不当接触 ⇒ 手掌一推推不动(没有东西托住重量)、夹着沿面走也得整个抬起来(2);
      --  面的摩擦当成锥 ⇒ 面反过来帮着推,推它不要力(0);压力可以全压在转轴上 ⇒ 转它不要力矩(几乎 0);往下的时候不看面 ⇒ 说做得到
      Check (abs (L_Px - 1.0 / Mu) < Eps and then abs (S_Px - Push) < Eps and then Sy_Px >= Drag - Eps and then Sy_Px <= Drag * Edge_Gap + Eps
             and then Sp_Px >= Couple - Eps and then Sp_Px <= Couple * Edge_Gap + Eps and then Sp_Px <= 1.0 / Mu + Eps
             and then D_Px = Wr.No_Way and then W_Px_Down = Wr.Surface_In_Way,
             "接触集物理·" & Name & "·两瓣沿 x 夹(两处 ±" & F4 (Wx) & "):往上 " & Nw (L_Px) & "(要 2)· 沿面往 +x " & Nw (S_Px) & "(要 μ/(1+μ²) = " & F4 (Push)
             & ")· 沿面往 +y " & Nw (Sy_Px) & "(要 " & F4 (Drag) & " … " & F4 (Drag * Edge_Gap) & ")· 绕竖轴转 " & Nw (Sp_Px) & "(ρ̄ " & F4 (Rb) & " ⇒ 要 " & F4 (Couple)
             & " … " & F4 (Couple * Edge_Gap) & ",不超过抬起来转的 2)· 往下 " & Nw (D_Px) & "(" & Wr.Why_Kind'Image (W_Px_Down) & ",要面挡着)");
      if Pair_Y then
         L_Py := N_Of (Py, Lift);
         S_Py := N_Of (Py, Along_X);
         --  病同上:夹的方向和走的方向垂直时只能靠摩擦拖,一定比夹的方向顺着走的方向贵
         Check (abs (L_Py - 1.0 / Mu) < Eps and then S_Py >= Drag - Eps and then S_Py <= Drag * Edge_Gap + Eps and then S_Py > S_Px,
                "接触集物理·" & Name & "·两瓣沿 y 夹:往上 " & Nw (L_Py) & "(要 2)· 沿面往 +x " & Nw (S_Py) & "(要 " & F4 (Drag) & " … " & F4 (Drag * Edge_Gap)
                & ",比沿 x 夹的 " & Nw (S_Px) & " 贵)");
      end if;
      --  五指:接触多了只会更省(或一样),往上照样 ≥ 2(只有摩擦往上托);沿面往 +x 有一处法向正好 +x ⇒ 0.4
      Check (abs (L_5 - 1.0 / Mu) < Eps and then abs (S_5 - Push) < Eps and then Sp_5 > 0.0
             and then (if Five_Exact_Spin then Sp_5 >= Couple - Eps and then Sp_5 <= Couple * Edge_Gap + Eps else Sp_5 <= Sp_Px + Eps),
             "接触集物理·" & Name & "·五指(" & Codec.Img (Natural (Five.Length)) & " 处):往上 " & Nw (L_5) & "(要 2)· 沿面往 +x " & Nw (S_5) & "(要 " & F4 (Push)
             & ")· 绕竖轴转 " & Nw (Sp_5) & (if Five_Exact_Spin then "(五处都在同一个半径上 ⇒ 要 " & F4 (Couple) & " … " & F4 (Couple * Edge_Gap) & ")"
                                              else "(不比两瓣的 " & Nw (Sp_Px) & " 贵:四个手指的法向力还能给力偶)"));
      --  没有手指(手掌):侧面一处推 —— 抬不起来(没有东西从另一边夹住)、沿面推得动(0.4,推的方向穿过面的摩擦中心)、偏着推就转了(纯平移做不到)、
      --  侧面一处转不了(净力不为零);从上面压下去的手掌能拧的半径比 ρ̄ 大才转得动
      Check (L_Sd = Wr.No_Way and then W_Sd_Up = Wr.Unbalanced and then abs (S_Sd - Push) < Eps and then So_Sd = Wr.No_Way and then Sp_Sd = Wr.No_Way
             and then W_Sp = Wr.Unbalanced
             and then (if Palm_Top < Wr.No_Way then abs (Sp_Tb - Palm_Top) < 1.0e-3 * Palm_Top else Sp_Tb = Wr.No_Way) and then Sp_Ts = Wr.No_Way,
             "接触集物理·" & Name & "·没有手指(手掌):侧面推往上 " & Nw (L_Sd) & "、沿面往 +x " & Nw (S_Sd) & "(要 " & F4 (Push) & ")、偏着推 " & Nw (So_Sd)
             & "、侧面转 " & Nw (Sp_Sd) & " · 上面压、能拧的半径 " & F4 (Top_Big) & " ⇒ 转 " & Nw (Sp_Tb) & "(要 ρ̄/(r − ρ̄) = " & Nw (Palm_Top) & ")· 半径 " & F4 (Top_Small)
             & " ⇒ " & Nw (Sp_Ts) & "(要做不到)");
   end Physics;

   --  ② 搜索 + 物理:同一件东西 × 一只手,要的动一个个换
   procedure Searched (Name : String; Pts : Contact.V3_Vectors.Vector; H : Se.Hand_Model; Hand_Name : String; Jaw_Follows : Boolean) is
      Com : constant Contact.V3 := Com_Of (Pts);
      None : Contact.V3_Vectors.Vector;
      function Run (W : Contact.Want; St : out Se.Plan_Stats) return Se.Cand_Vectors.Vector is
         Fd : Se.Cand_Vectors.Vector;
      begin
         Se.Plan (Pts, None, Pitch, 0.0005, Up, Zero3, H, Mu, 0.09, Always'Access, 1, Fd, St, Want => W);
         return Fd;
      end Run;
      St_U, St_X, St_Y, St_S, St_D : Se.Plan_Stats;
      Fu : constant Se.Cand_Vectors.Vector := Run ((others => <>), St_U);
      Fx : constant Se.Cand_Vectors.Vector := Run ((Given => True, Move => Along_X), St_X);
      Fy : constant Se.Cand_Vectors.Vector := Run ((Given => True, Move => Along_Y), St_Y);
      Fs : constant Se.Cand_Vectors.Vector := Run ((Given => True, Move => Spin (Com)), St_S);
      Fdn : constant Se.Cand_Vectors.Vector := Run ((Given => True, Move => Down), St_D);
      function Sq (F : Se.Cand_Vectors.Vector) return Long_Float is (if F.Is_Empty then Wr.No_Way else F.First_Element.Squeeze);
      --  挑中那一组第 0 块在世界里往哪合
      function Jaw (F : Se.Cand_Vectors.Vector) return Contact.V3 is (if F.Is_Empty then Zero3 else Geom.Ap (F.First_Element.R, H.Pads.First_Element.Dir));
      --  挑中那一组里有没有一对法向相对的接触(夹在料的两侧)
      function Opposed (F : Se.Cand_Vectors.Vector) return Boolean is
      begin
         if F.Is_Empty then
            return False;
         end if;
         declare
            C : constant Se.Candidate := F.First_Element;
         begin
            for I in 0 .. Natural (C.Touches.Length) - 1 loop
               for J in I + 1 .. Natural (C.Touches.Length) - 1 loop
                  if Contact.Dot (C.Touches (I).N, C.Touches (J).N) < 0.0 then
                     return True;
                  end if;
               end loop;
            end loop;
            return False;
         end;
      end Opposed;
      Jx : constant Contact.V3 := Jaw (Fx);
      Jy : constant Contact.V3 := Jaw (Fy);
   begin
      --  病:搜索不看要的动(挑的总是"托住"最省的那一组)⇒ 方块往 +x、往 +y 挑的是同一组,合拢方向总有一个不顺着走的方向;
      --  物理检查不看面 ⇒ 沿面走和抬起来一样贵;不看"往面里去" ⇒ 往下也挑得出一组
      Check (not Fu.Is_Empty and then not Fx.Is_Empty and then not Fs.Is_Empty and then St_U.Mu_Ref >= Mu and then St_X.Mu_Ref = St_U.Mu_Ref
             and then Sq (Fu) >= 1.0 / St_U.Mu_Ref - Eps and then Opposed (Fu) and then Sq (Fx) < Sq (Fu) and then Sq (Fs) <= Sq (Fu) + Eps
             and then Fdn.Is_Empty and then St_D.In_Way
             and then (not Jaw_Follows or else (not Fy.Is_Empty and then abs Jx (0) > 0.95 and then abs Jy (1) > 0.95)),
             "接触集搜索·" & Name & "·" & Hand_Name & ":" & Codec.Img (St_U.Poses) & " 个手位,摩擦按 " & F4 (St_U.Mu_Ref) & " · 往上挑中的要 " & Nw (Sq (Fu))
             & "(≥ 1/μ,两侧相对 " & (if Opposed (Fu) then "是" else "否") & ")· 沿面往 +x " & Nw (Sq (Fx)) & "(比往上省)· 绕竖轴转 " & Nw (Sq (Fs)) & "(不比往上贵)· 往下 "
             & (if St_D.In_Way then "面挡着" else "没挡住")
             & (if Jaw_Follows then " · 往 +x 挑的合拢方向 (" & Codec.Fmt (Jx (0), 2) & "," & Codec.Fmt (Jx (1), 2) & ")、往 +y 的 (" & Codec.Fmt (Jy (0), 2) & ","
                & Codec.Fmt (Jy (1), 2) & ")(要各自顺着走的方向)" else ""));
   end Searched;

   --  两瓣(同自检里 Contact.Grasp 那几条的手):两个尖在眼前 9 cm、相距 9 cm,手指沿合拢方向厚 1 cm,指肚宽 1.5 cm,手落位的误差 2 mm
   function Two_Hand return Se.Hand_Model is
      Ls : Se.Lobe_In_Vectors.Vector;
   begin
      Ls.Append (Se.Lobe_In'(Tip => [-0.045, 0.0, -0.09], Width => 0.015, Thick => 0.01));
      Ls.Append (Se.Lobe_In'(Tip => [0.045, 0.0, -0.09], Width => 0.015, Thick => 0.01));
      return Se.From_Lobes (Ls, 0.002);
   end Two_Hand;
   Two : constant Se.Hand_Model := Two_Hand;
   --  五指:五个尖在眼前 9 cm 的一圈上(半径 4.5 cm),各自朝圆心合(一个自由度,一起合),指肚宽 1.5 cm、手指厚 1 cm —— 同一个 From_Lobes
   function Five_Hand return Se.Hand_Model is
      Ls : Se.Lobe_In_Vectors.Vector;
   begin
      for K in 0 .. 4 loop
         declare
            A : constant Long_Float := 2.0 * Ada.Numerics.Pi * Long_Float (K) / 5.0;
         begin
            Ls.Append (Se.Lobe_In'(Tip => [0.045 * Cos (A), 0.045 * Sin (A), -0.09], Width => 0.015, Thick => 0.01));
         end;
      end loop;
      return Se.From_Lobes (Ls, 0.002);
   end Five_Hand;
   Five : constant Se.Hand_Model := Five_Hand;
   Bar_P : constant Contact.V3_Vectors.Vector := Slab (-0.012, 0.012, -0.102, 0.102, 0.01, Bar'Access);
   Block_P : constant Contact.V3_Vectors.Vector := Slab (-0.022, 0.022, -0.022, 0.022, 0.04, Block'Access);
   Disc_P : constant Contact.V3_Vectors.Vector := Slab (-0.022, 0.022, -0.022, 0.022, 0.03, Disc'Access);
   Sc_P : constant Contact.V3_Vectors.Vector := Slab (-0.026, 0.026, -0.062, 0.086, 0.004, Scissors'Access);
   --  五指在方块 / 条上:四个手指在 −x 那一面、拇指在 +x 那一面(人手那样);在圆柱上:五处在同一个半径上一圈
   function Fingers_Thumb (Com : Contact.V3; W, Spread : Long_Float) return Wr.Touch_Vectors.Vector is
      V : Wr.Touch_Vectors.Vector;
   begin
      for K in 0 .. 3 loop
         V.Append (T ([Com (0) - W, Com (1) + Spread * (Long_Float (K) - 1.5), Com (2)], [1.0, 0.0, 0.0], 0.005));
      end loop;
      V.Append (T ([Com (0) + W, Com (1), Com (2)], [-1.0, 0.0, 0.0], 0.005));
      return V;
   end Fingers_Thumb;
   function Ring5 (Com : Contact.V3; R : Long_Float) return Wr.Touch_Vectors.Vector is
      V : Wr.Touch_Vectors.Vector;
   begin
      for K in 0 .. 4 loop
         declare
            A : constant Long_Float := Ada.Numerics.Pi + 2.0 * Ada.Numerics.Pi * Long_Float (K) / 5.0;   --  第 0 处在 −x 那边,法向 +x
         begin
            V.Append (T ([Com (0) + R * Cos (A), Com (1) + R * Sin (A), Com (2)], [-Cos (A), -Sin (A), 0.0], 0.005));
         end;
      end loop;
      return V;
   end Ring5;
begin
   --  ① 物理检查本身
   declare
      Cb : constant Contact.V3 := Com_Of (Bar_P);
      Ck : constant Contact.V3 := Com_Of (Block_P);
      Cd : constant Contact.V3 := Com_Of (Disc_P);
   begin
      Physics ("条 2 × 20 cm", Bar_P, 0.01, 0.1, Fingers_Thumb (Cb, 0.01, 0.02), False, 0.004, 0.01, 0.01, 0.004, False);
      Physics ("方块 4 cm", Block_P, 0.02, 0.02, Fingers_Thumb (Ck, 0.02, 0.01), False, 0.01, 0.04, 0.02, 0.01, True);
      Physics ("圆柱 直径 4 cm", Disc_P, 0.02, 0.02, Ring5 (Cd, 0.02), True, 0.01, 0.03, 0.02, 0.01, True);
   end;
   --  平放的剪刀:形状不对称,按解析解只核两件 —— 手掌侧面推、推的方向穿过面的摩擦中心(那一片的形心)⇒ 0.4;偏着推 ⇒ 转;
   --  两瓣横着夹在重心那一截刀身上:往上 2、沿面往 +x 比往上省、转不比往上贵、往下面挡着;上面压的手掌只有刀身那么宽 ⇒ 转不动
   declare
      Com : constant Contact.V3 := Com_Of (Sc_P);
      Sup : constant Wr.Surface := Wr.Footprint (Sc_P, Zero3, Up, Pitch);
      Fc : Contact.V3 := Zero3;
      Why, W_Down : Wr.Why_Kind;
      pragma Warnings (Off, Why);   --  做不到的是哪一条只看往下那一格(W_Down),别的只看数
      Pair : Wr.Touch_Vectors.Vector;
      Palm, Palm_Off, Top : Wr.Touch_Vectors.Vector;
      L_P, S_P, Sp_P, D_P, S_Palm, So_Palm, Sp_Top : Long_Float;
   begin
      for P of Sup.Foot loop
         Fc (0) := Fc (0) + P (0) / Long_Float (Sup.Foot.Length); Fc (1) := Fc (1) + P (1) / Long_Float (Sup.Foot.Length);
      end loop;
      Pair := Set_Of (T ([-Blade_W (Com (1)), Com (1), 0.002], [1.0, 0.0, 0.0], 0.002), T ([Blade_W (Com (1)), Com (1), 0.002], [-1.0, 0.0, 0.0], 0.002));
      Palm := One (T ([Fc (0) - Blade_W (Fc (1)), Fc (1), 0.002], [1.0, 0.0, 0.0], 0.002));
      Palm_Off := One (T ([Fc (0) - Blade_W (Fc (1) - 0.03), Fc (1) - 0.03, 0.002], [1.0, 0.0, 0.0], 0.002));
      Top := One (T ([Com (0), Com (1), 0.004], [0.0, 0.0, -1.0], Blade_W (Com (1))));
      L_P := Wr.Need (Pair, Com, Up, Sup, Lift, Mu, Mu, Why);
      S_P := Wr.Need (Pair, Com, Up, Sup, Along_X, Mu, Mu, Why);
      Sp_P := Wr.Need (Pair, Com, Up, Sup, Spin (Com), Mu, Mu, Why);
      D_P := Wr.Need (Pair, Com, Up, Sup, Down, Mu, Mu, W_Down);
      S_Palm := Wr.Need (Palm, Com, Up, Sup, Along_X, Mu, Mu, Why);
      So_Palm := Wr.Need (Palm_Off, Com, Up, Sup, Along_X, Mu, Mu, Why);
      Sp_Top := Wr.Need (Top, Com, Up, Sup, Spin (Com), Mu, Mu, Why);
      Check (abs (L_P - 1.0 / Mu) < Eps and then S_P < L_P and then Sp_P <= L_P + Eps and then D_P = Wr.No_Way and then W_Down = Wr.Surface_In_Way
             and then abs (S_Palm - Mu / (1.0 + Mu * Mu)) < Eps and then So_Palm = Wr.No_Way and then Sp_Top = Wr.No_Way,
             "接触集物理·平放的剪刀:两瓣横着夹在重心那一截刀身(y = " & F4 (Com (1)) & "):往上 " & Nw (L_P) & "(要 2)· 沿面往 +x " & Nw (S_P) & " · 转 " & Nw (Sp_P)
             & " · 往下 " & Nw (D_P) & " · 手掌在摩擦中心(y = " & F4 (Fc (1)) & ")那一截侧面推 " & Nw (S_Palm) & "(要 0.4)、偏 3 cm 推 " & Nw (So_Palm)
             & "(要做不到)、上面压(刀身那么宽)转 " & Nw (Sp_Top) & "(要做不到)");
   end;
   --  没有面的时候(它离开了面、或者在手里)和 09-29 的"托住要多紧"是同一个规划:要"往上"时,面在不在都一样(面不给力)
   declare
      Ck : constant Contact.V3 := Com_Of (Block_P);
      Sup : constant Wr.Surface := Wr.Footprint (Block_P, Zero3, Up, Pitch);
      Px : constant Wr.Touch_Vectors.Vector := Set_Of (T ([Ck (0) - 0.02, Ck (1) + 0.01, Ck (2)], [1.0, 0.0, 0.0], 0.005), T ([Ck (0) + 0.02, Ck (1) + 0.01, Ck (2)], [-1.0, 0.0, 0.0], 0.005));
      Why : Wr.Why_Kind;
      With_Sup : constant Long_Float := Wr.Need (Px, Ck, Up, Sup, Lift, Mu, Mu, Why);
      Old : constant Long_Float := Wr.Squeeze (Px, (F => Up, C => Ck, M => Zero3), Mu);
   begin
      Check (abs (With_Sup - Old) < 1.0e-9 * Old and then With_Sup > 1.0 / Mu,
             "接触集物理·往上时面不给力:偏开重心 1 cm 夹的方块,连同面 " & Nw (With_Sup) & " = 不算面的托住要多紧 " & Nw (Old) & "(偏开了要拧,比 2 大)");
   end;
   --  ② 搜索 + 物理
   Searched ("方块 4 cm", Block_P, Two, "两瓣", True);
   Searched ("圆柱 直径 4 cm", Disc_P, Two, "两瓣", True);
   Searched ("条 2 × 20 cm", Bar_P, Two, "两瓣", False);
   Searched ("平放的剪刀", Sc_P, Two, "两瓣", False);
   Searched ("方块 4 cm", Block_P, Five, "五指", False);
   Searched ("圆柱 直径 4 cm", Disc_P, Five, "五指", False);
   Searched ("条 2 × 20 cm", Bar_P, Five, "五指", False);
   Searched ("平放的剪刀", Sc_P, Five, "五指", False);
   --  没有手指 / 只有一瓣:建手照实说为什么不成(瓣数不认,同一个 From_Lobes);合拢这一条路上一组都搜不出来,照实交空的(胳膊那条路接进同一个搜索是下一件)
   declare
      Ls0, Ls1 : Se.Lobe_In_Vectors.Vector;
      H0, H1 : Se.Hand_Model;
      Fd : Se.Cand_Vectors.Vector;
      St : Se.Plan_Stats;
      None : Contact.V3_Vectors.Vector;
   begin
      Ls1.Append (Se.Lobe_In'(Tip => [0.0, 0.0, -0.09], Width => 0.015, Thick => 0.01));
      H0 := Se.From_Lobes (Ls0, 0.002);
      H1 := Se.From_Lobes (Ls1, 0.002);
      Se.Plan (Block_P, None, Pitch, 0.0005, Up, Zero3, H0, Mu, 0.09, Always'Access, 1, Fd, St, Want => (Given => True, Move => Along_X));
      Check (not H0.Valid and then not H1.Valid and then Fd.Is_Empty and then St.Poses = 0,
             "接触集搜索·没有手指:建手说「" & To_String (H0.Why) & "」;只有一瓣说「" & To_String (H1.Why) & "」;合拢这一条路上一组都搜不出来(试了 "
             & Codec.Img (St.Poses) & " 个手位),照实交空的");
   end;
   --  驱动那一段(Act.Plan_Contact)按这只眼里量到的全部瓣建手(同主代理那条"接触集接进执行层"焊点的搭法,手换成五瓣):方块 4 cm 躺在面上,
   --  脑要它沿面往 +x ⇒ 布得出一组、说的是要的那个动;脑要它往下 ⇒ 照实说面挡着。
   --  病:建手只认两瓣 ⇒ 五瓣的手一组都布不出来("my fingers in this eye are 5 measured pads");要的动没交进去 ⇒ 往下也布得出来
   declare
      Cx : Act.Context;
      Fx : Plug.Frame;
      Gx : Geom.Cam_Geo;
      Pick, Pick_D : Contact.Grasp.Candidate;
      Nt, Nd : Unbounded_String;
      Okp, Okd : Boolean;
      procedure Any_Reach (Arm : Natural; Pose : Plug.Arm_Pose; Pos_Err, Rot_Err : out Long_Float) is
         pragma Unreferenced (Arm, Pose);
      begin
         Pos_Err := 0.0; Rot_Err := 0.0;
      end Any_Reach;
   begin
      Gx.Valid := True; Gx.F := 400.0; Gx.Cx := 320.0; Gx.Cy := 240.0; Gx.Gap := 0.09;
      Gx.Tip := [0.0, 0.0, -0.09]; Gx.Tip_Valid := True; Gx.Tip_Touch := True; Gx.Tip_Sd := 0.0005;
      for K in 0 .. 4 loop
         declare
            A : constant Long_Float := 2.0 * Ada.Numerics.Pi * Long_Float (K) / 5.0;
         begin
            Gx.Lobes.Append (Geom.Lobe_Geo'(Tip => [0.045 * Cos (A), 0.045 * Sin (A), -0.09], Wide => 0.015, Thin => 0.01));
         end;
      end loop;
      Cx.Geo.Append (Geom.No_Geo); Cx.Geo.Append (Gx);
      Cx.Map.Amp := Bytes.F64_Vectors.To_Vector (0.0, 6);
      Cx.Map.Amp.Replace_Element (0, 0.001); Cx.Map.Amp.Replace_Element (3, 0.0025);
      Cx.Touch_Valid := True; Cx.Touch_Pt := [0.0, 0.0, 0.0]; Cx.Touch_N := [0.0, 0.0, 1.0];
      for I in -10 .. 10 loop
         for J in -10 .. 10 loop
            Cx.Sil_Pts.Append (Geom.V3'[Pitch * Long_Float (I), Pitch * Long_Float (J), 0.04]);
         end loop;
      end loop;
      Cx.Sil_Valid := True; Cx.Sil_Name := To_Unbounded_String ("block"); Cx.Sil_Cam := 1; Cx.Sil_N := [0.0, 0.0, 1.0];
      Cx.Sil_P0 := Cx.Sil_Pts.First_Element; Cx.Sil_Pitch := Pitch; Cx.Sil_Err := 0.0005;
      Plug.Set_Reach (Any_Reach'Unrestricted_Access);
      Cx.Want_Move := (Given => True, Move => Along_X);
      Act.Plan_Contact (Cx, Fx, 0, 1, To_Unbounded_String ("block"), Pick, Nt, Okp);
      Cx.Want_Move := (Given => True, Move => Down);
      Act.Plan_Contact (Cx, Fx, 0, 1, To_Unbounded_String ("block"), Pick_D, Nd, Okd);
      Plug.Set_Reach (null);
      Check (Okp and then Natural (Pick.Touches.Length) >= 2 and then Index (Nt, "rad off the normal") > 0 and then not Okd and then Index (Nd, "goes into the surface") > 0,
             "接触集接进执行层·五瓣的手:方块沿面往 +x ⇒ " & (if Okp then "布出一组(" & Codec.Img (Natural (Pick.Touches.Length)) & " 处接触)" else "布不出:" & To_String (Nt))
             & " · 往下 ⇒ " & (if Okd then "布出来了(错)" else "照实说:" & To_String (Nd) (1 .. Natural'Min (Length (Nd), 120))));
   end;
   --  一串瓣的 From_Lobes 在两瓣时和 09-29 的两瓣写法逐位相同:按原来那几行公式现场算一遍当参照(碰东西的面在尖往对面半个手指厚、各走两面之间的空的一半)
   --  病:建手改成一串以后,两瓣的手悄悄变了(碰东西的面、行程、合拢方向),x5 挑出来的接触跟着变
   declare
      A : constant Contact.V3 := [-0.045, 0.0, -0.09];
      B : constant Contact.V3 := [0.045, 0.0, -0.09];
      Ok_U : Boolean;
      D : constant Contact.V3 := [B (0) - A (0), B (1) - A (1), B (2) - A (2)];
      U : constant Contact.V3 := Contact.Unit (D, Ok_U);
      Th : constant Long_Float := 0.01;
      Half : constant Long_Float := 0.5 * (Contact.Norm (D) - Th);
      Ref_A : constant Se.Pad := (Tip => [A (0) + 0.5 * Th * U (0), A (1) + 0.5 * Th * U (1), A (2) + 0.5 * Th * U (2)], Dir => U, Travel => Half, Width => 0.015, Thick => Th);
      Ref_B : constant Se.Pad := (Tip => [B (0) - 0.5 * Th * U (0), B (1) - 0.5 * Th * U (1), B (2) - 0.5 * Th * U (2)], Dir => [-U (0), -U (1), -U (2)], Travel => Half,
                                  Width => 0.015, Thick => Th);
      function Near (P, Q : Se.Pad) return Boolean is
        (Contact.Norm ([P.Tip (0) - Q.Tip (0), P.Tip (1) - Q.Tip (1), P.Tip (2) - Q.Tip (2)]) < 1.0e-12
         and then Contact.Norm ([P.Dir (0) - Q.Dir (0), P.Dir (1) - Q.Dir (1), P.Dir (2) - Q.Dir (2)]) < 1.0e-12
         and then abs (P.Travel - Q.Travel) < 1.0e-12 and then P.Width = Q.Width and then P.Thick = Q.Thick);
   begin
      Check (Ok_U and then Two.Valid and then Natural (Two.Pads.Length) = 2 and then Near (Two.Pads (0), Ref_A) and then Near (Two.Pads (1), Ref_B)
             and then abs (Two.Reach_In - 0.09) < 1.0e-12,
             "接触集建手:一串瓣的写法在两瓣时和 09-29 的两瓣写法逐位相同(碰东西的面、合拢方向、行程 " & F4 (Two.Pads (0).Travel) & "、宽、厚、指尖到眼 "
             & F4 (Two.Reach_In) & ")");
   end;
   --  ── 量 → 要它怎么动(§2 第 16 条后半:量都变成旋量,10-01)──
   --  ① height 和 10-01 以前逐位一样:以前 = 沿 Up_Dir(碰过的面按量到的法向,没碰过按协议的上)走,往下取反;现在 = Act.Want_Twist 的平移。
   --     三种面:碰过的面(法向是量出来又归一过的,模长差一丝不到 1)、没碰过也没有板、只有标定板(Up_Dir 仍是协议的上),上下各一次,三个分量逐位比。
   --     病:换成旋量以后 height 悄悄变了一丝(比如把量到的法向再归一一遍),x5 抬东西的每一步跟着变,和 09-23 以来的落盘对不上
   declare
      function Old_Height (Cx : Act.Context; Dir : Integer) return Contact.V3 is
         Ax : constant Contact.V3 := (if Cx.Touch_Valid then Cx.Touch_N else [0.0, 0.0, 1.0]);
      begin
         return (if Dir < 0 then [-Ax (0), -Ax (1), -Ax (2)] else Ax);
      end Old_Height;
      --  一个"量出来又归一过"的法向,模长在浮点上不正好是 1(牙要咬得住:再归一一遍末位会变)
      function Measured_N return Contact.V3 is
         Ok : Boolean;
         V : Contact.V3 := [0.0, 0.0, 1.0];
      begin
         for K in 1 .. 50 loop
            V := Contact.Unit ([0.013 * Long_Float (K), -0.021, 0.9997], Ok);
            exit when Contact.Norm (V) /= 1.0;
         end loop;
         return V;
      end Measured_N;
      Nm : constant Contact.V3 := Measured_N;
      Same : Boolean := Contact.Norm (Nm) /= 1.0;
      Fx : Plug.Frame;
      Seen : Unbounded_String;
   begin
      for Case_K in 0 .. 2 loop
         for Dk in 0 .. 1 loop
            declare
               Dir : constant Integer := (if Dk = 0 then 1 else -1);
               Cx : Act.Context;
               W : constant Act.Want := (Thing => 0, Rel => Sinew.Re_Qty, Qty => To_Unbounded_String ("height"), Dir => Dir, Ref => 0);
               M : Contact.Twist;
               Ok : Boolean;
               Note : Unbounded_String;
            begin
               if Case_K = 0 then
                  Cx.Touch_Valid := True; Cx.Touch_N := Nm; Cx.Touch_Pt := [0.0, 0.0, 0.0];
               elsif Case_K = 2 then
                  Cx.Board_Plane := True; Cx.Board_N := Nm; Cx.Board_Pt := [0.0, 0.0, 0.0];
               end if;
               Act.Want_Twist (Cx, Fx, W, -1, M, Ok, Note);
               declare
                  E : constant Contact.V3 := Old_Height (Cx, Dir);
               begin
                  Same := Same and then Ok and then M.Lin (0) = E (0) and then M.Lin (1) = E (1) and then M.Lin (2) = E (2) and then Contact.Angle (M) = 0.0;
                  Append (Seen, " (" & F4 (M.Lin (0)) & "," & F4 (M.Lin (1)) & "," & F4 (M.Lin (2)) & ")");
               end;
            end;
         end loop;
      end loop;
      Check (Same, "量变旋量·height 和以前逐位一样(碰过的面 / 没碰过 / 只有板,上下各一次;法向模长 " & Codec.Fmt (Contact.Norm (Nm) - 1.0, 17) & " 偏离 1):" & To_String (Seen));
   end;
   --  ② 每个量往哪变(Contact.Qty.Motion,量到的几何直接给):它在 (0.10, 0.20) 躺在面上(上 = +z),中心高 0.02;我那只眼在 (0.50, 0.20, 0.60);
   --     脑看着的那只眼横轴朝 -y;参照那一件在 (-0.10, 0.20),顶面高 0.03。
   --     · height 往上 = +z、往下 = -z;heading 往上 = 绕过它中心的 +z 转;tilt 往上 = 它的顶往远离我那边倒(绕 z × 水平离我的方向);away 往上 = 水平离我更远
   --     · gap 往下(nearer)= 朝参照那一件;参照那一件比它低(中心更低)时往面里去的那一份去掉,只在面里走;rise ±z;across 往上 = 那只眼的右边(-y 放平)
   --     · rest_on:底不比参照的顶高出不准那么多 ⇒ 先往上;够高、不在正上方 ⇒ 横着朝它;在正上方、比它的顶高 ⇒ 往下;贴着它的顶 ⇒ 不动
   --     · aim:长轴沿 x,参照那一件在 +y 方向 ⇒ 绕 +z 转(逆时针)转过去;在 -y ⇒ 绕 -z;正对着长轴(两头都算)⇒ 不动
   --     · 缺什么照实说:没有"我"那只眼 ⇒ tilt / away 说不出;没有参照那一件 ⇒ gap 说不出
   --  病:量的方向写反 / 绕错点转 / 贴着面躺的东西被要求往面里走 / onto 横着直接往参照那一件身上撞 / 长轴只认一头,转大半圈
   declare
      package Q renames Contact.Qty;
      S0 : Q.Scene;
      function Mo (K : Q.Kind; Dir : Integer; S : Q.Scene; M : out Contact.Twist; Note : out Unbounded_String) return Boolean is
         Ok : Boolean;
      begin
         Q.Motion (K, Dir, S, M, Ok, Note);
         return Ok;
      end Mo;
      function Near (A, B : Contact.V3) return Boolean is (Contact.Norm ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]) < 1.0e-9);
      M_H, M_Hd, M_Hd2, M_T, M_A, M_G, M_G2, M_R, M_X, M_O1, M_O2, M_O3, M_O4, M_Am, M_Am2, M_Am3, M_Miss1, M_Miss2 : Contact.Twist;
      N1, N2, N3, N4, N5, N6, N7, N8, N9, N10, N11, N12, N13, N14, N15, N16, N17, N18 : Unbounded_String;
      Ok_All : Boolean := True;
      Tilt_Away : Boolean := False;
      Away_Up : Boolean := False;
      Gap_Flat : Boolean := False;
   begin
      S0.Up := [0.0, 0.0, 1.0];
      S0.Center := [0.10, 0.20, 0.02]; S0.Has_Center := True;
      S0.Bottom := 0.0; S0.Has_Bottom := True;
      S0.Axis := [1.0, 0.0, 0.0]; S0.Has_Axis := True; S0.Ang_Sd := 0.001;
      S0.Me := [0.50, 0.20, 0.60]; S0.Has_Me := True;
      S0.View_Right := [0.0, -1.0, 0.0]; S0.Has_View := True;
      S0.Ref := [-0.10, 0.20, 0.015]; S0.Has_Ref := True; S0.Ref_Top := 0.03; S0.Has_Ref_Top := True;
      S0.Sd := 0.001;
      Ok_All := Mo (Q.Height, 1, S0, M_H, N1) and then Near (M_H.Lin, [0.0, 0.0, 1.0]);
      Ok_All := Ok_All and then Mo (Q.Height, -1, S0, M_Hd, N2) and then Near (M_Hd.Lin, [0.0, 0.0, -1.0]);
      Ok_All := Ok_All and then Mo (Q.Heading, 1, S0, M_Hd2, N3) and then Near (M_Hd2.Ang, [0.0, 0.0, 1.0]) and then Near (M_Hd2.Pivot, S0.Center);
      --  tilt 往上:它顶上那一点(中心 + 0.05 z)绕那根轴转一丝 ⇒ 水平离我更远
      if Mo (Q.Tilt, 1, S0, M_T, N4) then
         declare
            Top : constant Contact.V3 := [S0.Center (0), S0.Center (1), S0.Center (2) + 0.05];
            Sm : constant Contact.Twist := (Lin => [0.0, 0.0, 0.0], Ang => [0.01 * M_T.Ang (0), 0.01 * M_T.Ang (1), 0.01 * M_T.Ang (2)], Pivot => M_T.Pivot);
            T1 : constant Contact.V3 := Contact.Apply (Sm, Top);
            function Hd (P : Contact.V3) return Long_Float is (Sqrt ((P (0) - S0.Me (0)) ** 2 + (P (1) - S0.Me (1)) ** 2));
         begin
            Tilt_Away := Hd (T1) > Hd (Top) and then Near (M_T.Pivot, S0.Center);
         end;
      end if;
      if Mo (Q.Away, 1, S0, M_A, N5) then
         Away_Up := Near (M_A.Lin, [-1.0, 0.0, 0.0]);   --  我在 +x 那边 ⇒ 远离我 = -x
      end if;
      Ok_All := Ok_All and then Mo (Q.Gap, -1, S0, M_G, N6) and then Near (M_G.Lin, [-1.0, 0.0, 0.0]);
      --  参照那一件的中心比它低:往它那边走有往下的一份,它贴着面躺 ⇒ 去掉,只在面里走
      Gap_Flat := Mo (Q.Gap, -1, S0, M_G2, N7) and then M_G2.Lin (2) >= 0.0;
      Ok_All := Ok_All and then Mo (Q.Rise, 1, S0, M_R, N8) and then Near (M_R.Lin, [0.0, 0.0, 1.0]);
      Ok_All := Ok_All and then Mo (Q.Across, 1, S0, M_X, N9) and then Near (M_X.Lin, [0.0, -1.0, 0.0]);
      --  rest_on 四段
      declare
         S1 : Q.Scene := S0;
      begin
         Ok_All := Ok_All and then Mo (Q.Rest_On, 1, S1, M_O1, N10) and then Near (M_O1.Lin, [0.0, 0.0, 1.0]);   --  底 0 < 顶 0.03 ⇒ 先往上
         S1.Bottom := 0.05; S1.Center (2) := 0.07;
         Ok_All := Ok_All and then Mo (Q.Rest_On, 1, S1, M_O2, N11) and then Near (M_O2.Lin, [-1.0, 0.0, 0.0]);  --  够高、不在正上方 ⇒ 横着朝它
         S1.Center (0) := S1.Ref (0); S1.Center (1) := S1.Ref (1);
         Ok_All := Ok_All and then Mo (Q.Rest_On, 1, S1, M_O3, N12) and then Near (M_O3.Lin, [0.0, 0.0, -1.0]);  --  在正上方 ⇒ 往下
         S1.Bottom := S1.Ref_Top;
         Ok_All := Ok_All and then Mo (Q.Rest_On, 1, S1, M_O4, N13) and then not Contact.Moving (M_O4);           --  贴着它的顶 ⇒ 不动
      end;
      --  aim
      declare
         S2 : Q.Scene := S0;
      begin
         S2.Ref := [0.10, 0.50, 0.02];   --  在 +y
         Ok_All := Ok_All and then Mo (Q.Aim, 1, S2, M_Am, N14) and then Near (M_Am.Ang, [0.0, 0.0, 1.0]);
         S2.Ref := [0.10, -0.10, 0.02];  --  在 -y
         Ok_All := Ok_All and then Mo (Q.Aim, 1, S2, M_Am2, N15) and then Near (M_Am2.Ang, [0.0, 0.0, -1.0]);
         S2.Ref := [-0.20, 0.20, 0.02];  --  在长轴的另一头
         Ok_All := Ok_All and then Mo (Q.Aim, 1, S2, M_Am3, N16) and then not Contact.Moving (M_Am3);
      end;
      --  缺的照实说
      declare
         S3 : Q.Scene := S0;
      begin
         S3.Has_Me := False;
         Ok_All := Ok_All and then not Mo (Q.Tilt, 1, S3, M_Miss1, N17) and then Index (N17, "still") > 0;
         S3.Has_Ref := False;
         Ok_All := Ok_All and then not Mo (Q.Gap, -1, S3, M_Miss2, N18) and then Index (N18, "other thing") > 0;
      end;
      Check (Ok_All and then Tilt_Away and then Away_Up and then Gap_Flat,
             "量变旋量·每个量往哪变:height ±z · heading 绕它中心的 +z · tilt 往上顶往远离我那边倒 " & (if Tilt_Away then "是" else "否")
             & " · away 往上 (" & F4 (M_A.Lin (0)) & "," & F4 (M_A.Lin (1)) & ")· gap 往下朝参照那一件、它贴着面 ⇒ 不往面里去(z " & F4 (M_G2.Lin (2)) & ")"
             & " · across 往右 = 那只眼的右边 · rest_on 先上 / 横着 / 往下 / 不动 · aim 逆时针 / 顺时针 / 两头都算不动 · 缺的照实说「" & To_String (N17) & "」「" & To_String (N18) & "」");
   end;
   --  ③ 它此刻在哪(Act.Want_Scene):没拿着 ⇒ 记下的轮廓补成实心(同接触集那一份)的形心、底贴着面、长条的长轴;
   --     拿着 ⇒ 合上那一刻那份按手挪过的刚体运动搬过来(手平移 (0.10, 0, 0.05)、绕 z 转 90°:形心跟着走、长轴跟着转、底离面 0.05)。
   --  病:拿起来以后还按桌上那份算(onto 永远在"先往上",heading 绕桌上的旧中心转);长轴不跟着手转
   declare
      Cx : Act.Context;
      Fx : Plug.Frame;
      W : constant Act.Want := (Thing => 1, Rel => Sinew.Re_Qty, Qty => To_Unbounded_String ("heading"), Dir => 1, Ref => 0);
      Sc0, Sc1 : Contact.Qty.Scene;
      It : Act.Item;
      Moved_Ok, Axis_Ok, Lying_Ok : Boolean := False;
   begin
      Cx.Touch_Valid := True; Cx.Touch_Pt := [0.0, 0.0, 0.0]; Cx.Touch_N := [0.0, 0.0, 1.0];
      for I in -20 .. 20 loop
         for J in -3 .. 3 loop
            Cx.Sil_Pts.Append (Geom.V3'[Pitch * Long_Float (I), Pitch * Long_Float (J), 0.02]);   --  8 cm × 1.2 cm 的条,顶面高 2 cm
         end loop;
      end loop;
      Cx.Sil_Valid := True; Cx.Sil_Name := To_Unbounded_String ("bar"); Cx.Sil_Cam := 0; Cx.Sil_N := [0.0, 0.0, 1.0];
      Cx.Sil_P0 := Cx.Sil_Pts.First_Element; Cx.Sil_Pitch := Pitch; Cx.Sil_Err := 0.0005;
      It.Kind := Act.Thing;
      Cx.Items.Append (It);
      Cx.Boxed.Append (Act.Boxed_Thing'(Name => To_Unbounded_String ("bar"), others => <>));
      Fx.EE.Append (Plug.Arm_Pose'[0.0, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0]);
      Act.Want_Scene (Cx, Fx, W, 0, Sc0);
      Lying_Ok := Sc0.Has_Center and then Sc0.Has_Bottom and then abs Sc0.Bottom < 1.0e-12 and then Sc0.Has_Axis and then abs Sc0.Axis (0) > 0.999
        and then abs Sc0.Center (0) < Pitch and then abs Sc0.Center (1) < Pitch;
      --  拿住:合上那一刻手在 (0, 0, 0.1);现在手平移 (0.10, 0, 0.05)、绕 z 转 90°
      Cx.Wld.Holding := True; Cx.Wld.Held_Arm := 0;
      Contact.Surface.Walls_To_Support (Cx.Sil_Pts, Up, Zero3, Pitch, Cx.Held_Shape);   --  同 Solid_Of(这里没有视线可重投)
      Cx.Held_Pose := Fx.EE (0);
      Fx.EE.Replace_Element (0, Plug.Arm_Pose'[0.10, 0.0, 0.15, Cos (Ada.Numerics.Pi / 4.0), 0.0, 0.0, Sin (Ada.Numerics.Pi / 4.0)]);
      Act.Want_Scene (Cx, Fx, W, 0, Sc1);
      --  形心:相对合上那一刻的手 (0, 0, 0.1) 的那一段绕 z 转 90°((x, y, z) ⇒ (-y, x, z)),再接到现在的手 (0.10, 0, 0.15) 上
      --  (实心模型的侧壁按格子补,左右差一格,形心不正好在 0;所以参照按桌上那份现算)
      Moved_Ok := Sc1.Has_Center and then abs (Sc1.Center (0) - (0.10 - Sc0.Center (1))) < 1.0e-9 and then abs (Sc1.Center (1) - Sc0.Center (0)) < 1.0e-9
        and then abs (Sc1.Center (2) - (Sc0.Center (2) + 0.05)) < 1.0e-9 and then abs (Sc1.Bottom - 0.05) < 1.0e-9;
      Axis_Ok := Sc1.Has_Axis and then abs Sc1.Axis (1) > 0.999;   --  长轴跟着转 90°:x ⇒ y
      Check (Lying_Ok and then Moved_Ok and then Axis_Ok,
             "量变旋量·它此刻在哪:桌上那份形心 (" & F4 (Sc0.Center (0)) & "," & F4 (Sc0.Center (1)) & "," & F4 (Sc0.Center (2)) & ")、底 " & F4 (Sc0.Bottom)
             & "、长轴 (" & F4 (Sc0.Axis (0)) & "," & F4 (Sc0.Axis (1)) & ") · 拿起来手挪 (0.10,0,0.05)、转 90° ⇒ 形心 (" & F4 (Sc1.Center (0)) & "," & F4 (Sc1.Center (1)) & ","
             & F4 (Sc1.Center (2)) & ")、底 " & F4 (Sc1.Bottom) & "、长轴 (" & F4 (Sc1.Axis (0)) & "," & F4 (Sc1.Axis (1)) & ")");
   end;
   --  ④ 拿着它转(Act.Carry_Goal):手按它给的位姿走,它身上每一点正好绕那根轴(过它中心)转了要的角 —— 和 Want_Scene 搬它用的是同一个刚体变换
   --  病:手原地转(位置不绕那一点转过去)⇒ 它绕手腕转,中心被甩出去;转的轴按手系而不是世界系
   declare
      Cur : constant Plug.Arm_Pose := [0.30, -0.10, 0.25, Cos (0.3), Sin (0.3) * 0.6, 0.0, Sin (0.3) * 0.8];
      M : constant Contact.Twist := (Lin => [0.0, 0.0, 0.0], Ang => [0.0, 0.0, 1.0], Pivot => [0.32, -0.05, 0.02]);
      Th : constant Long_Float := 0.4;
      Gl : constant Plug.Arm_Pose := Act.Carry_Goal (Cur, M, Th);
      Rd : constant Geom.M3 := Geom.Mul (Geom.Quat_To_R (Gl), Geom.Tr (Geom.Quat_To_R (Cur)));
      Worst : Long_Float := 0.0;
      type V3_Arr is array (0 .. 3) of Contact.V3;
      Ps : constant V3_Arr := [[0.32, -0.05, 0.02], [0.36, -0.05, 0.02], [0.32, 0.0, 0.06], [0.25, -0.12, 0.01]];
   begin
      for P of Ps loop
         declare
            D : constant Geom.V3 := Geom.Ap (Rd, [P (0) - Cur (0), P (1) - Cur (1), P (2) - Cur (2)]);
            Carried : constant Contact.V3 := [Gl (0) + D (0), Gl (1) + D (1), Gl (2) + D (2)];
            Want_P : constant Contact.V3 := Contact.Apply ((Lin => [0.0, 0.0, 0.0], Ang => [0.0, 0.0, Th], Pivot => M.Pivot), P);
         begin
            Worst := Long_Float'Max (Worst, Contact.Norm ([Carried (0) - Want_P (0), Carried (1) - Want_P (1), Carried (2) - Want_P (2)]));
         end;
      end loop;
      Check (Worst < 1.0e-9, "量变旋量·拿着它转:手按 Carry_Goal 走,它身上四个点和「绕过它中心的竖轴转 0.4 rad」最多差 " & Codec.Fmt (Worst, 12));
   end;
end Welds_Path_5;
