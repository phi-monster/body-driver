with Ada.Numerics; use Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Containers.Generic_Array_Sort;
with Ada.Strings.Fixed;
package body Contact.Gen is

   function Nat_Img (N : Natural) return String is (Ada.Strings.Fixed.Trim (Natural'Image (N), Ada.Strings.Left));

   type Float_Array is array (Natural range <>) of Long_Float;
   procedure Sort_Floats is new Ada.Containers.Generic_Array_Sort (Natural, Long_Float, Float_Array);

   --  中位数;空表 ⇒ 'Last(拿它当门槛时谁都过线)
   function Median (V : Float_Array) return Long_Float is
      C : Float_Array := V;
   begin
      if C'Length = 0 then
         return Long_Float'Last;
      end if;
      Sort_Floats (C);
      return C (C'First + C'Length / 2);
   end Median;

   --  容器按元素访问在 GNAT 里每次都要造一个引用对象(带终结),百万次就是秒级;几何内环全走裸数组,容器只在进出口出现
   type V3_Array is array (Natural range <>) of V3;
   type Nat_Array is array (Natural range <>) of Natural;
   function To_Array (V : V3_Vectors.Vector) return V3_Array is
      A : V3_Array (0 .. Natural (V.Length) - 1);
      K : Natural := 0;
   begin
      for P of V loop
         A (K) := P;
         K := K + 1;
      end loop;
      return A;
   end To_Array;

   --  把一层里的点按"挨着不挨着"分块(单链)。下手点只能落在一块连着的料上:不分块的话,两块分开的料之间会算出一个悬空的下手点
   --  (相距 1.2 m 的两根杆,沿某个方向看"这一层只有 2 cm 宽",而那 2 cm 的中心在半空中 —— 单元测试逮到的,不是推想的)。
   --  返回每个点的块号(按首次出现的顺序编)和块数。
   procedure Clusters (Band : V3_Array; Gp : Long_Float; Label : out Nat_Array; N_Clusters : out Natural) is
      N : constant Natural := Band'Length;
      Owner : Nat_Array (0 .. N - 1);
      Slot : array (0 .. N - 1) of Integer := [others => -1];
      G2 : constant Long_Float := Gp * Gp;
      function Find (I0 : Natural) return Natural is
         I : Natural := I0;
      begin
         while Owner (I) /= I loop
            Owner (I) := Owner (Owner (I));
            I := Owner (I);
         end loop;
         return I;
      end Find;
   begin
      N_Clusters := 0;
      for I in 0 .. N - 1 loop
         Owner (I) := I;
      end loop;
      for I in 0 .. N - 1 loop
         for J in I + 1 .. N - 1 loop
            declare
               Dx : constant Long_Float := Band (Band'First + I) (0) - Band (Band'First + J) (0);
               Dy : constant Long_Float := Band (Band'First + I) (1) - Band (Band'First + J) (1);
            begin
               if Dx * Dx + Dy * Dy <= G2 then
                  declare
                     A : constant Natural := Find (I);
                     B : constant Natural := Find (J);
                  begin
                     if A /= B then
                        Owner (A) := B;
                     end if;
                  end;
               end if;
            end;
         end loop;
      end loop;
      for I in 0 .. N - 1 loop
         declare
            R : constant Natural := Find (I);
         begin
            if Slot (R) < 0 then
               Slot (R) := N_Clusters;
               N_Clusters := N_Clusters + 1;
            end if;
            Label (Label'First + I) := Slot (R);
         end;
      end loop;
   end Clusters;

   type QPt is record
      Q, U, X, Y : Long_Float;
   end record;
   type QPt_Array is array (Natural range <>) of QPt;
   function Q_Less (A, B : QPt) return Boolean is (A.Q < B.Q);
   procedure Sort_QPts is new Ada.Containers.Generic_Array_Sort (Natural, QPt, QPt_Array, Q_Less);
   --  一条带子里的一段连续的料:沿合爪方向的近端/远端,和它的点的 x/y/u 和、点数
   type Seg is record
      Lo, Hi, Sx, Sy, Su : Long_Float;
      K : Natural;
   end record;
   package Seg_Vectors is new Ada.Containers.Vectors (Natural, Seg);
   type Pair is record
      Mn, Mx, Sx, Sy : Long_Float;
      K : Natural;
   end record;
   package Pair_Vectors is new Ada.Containers.Vectors (Natural, Pair);

   --  排序:够得到 → 跨度在爪张开度以内 → 离桌面够高(下限)→ 面正对(过线)→ 离重心近(过线)→ 料越深越前 → 生成顺序。
   --  第二项是排序不是否决(仓里唯一那条可抓性规矩禁的是"拿钳口张开度当阈值筛掉");过线/不过线是门槛不是最大化(写成最大化会一票压死后面所有项:
   --  "离桌面高"写成最大化就专挑鞋口那圈软皮,"面正对"写成最大化就让一片孤零零的薄片因为离散化噪声赢)。
   function Before (A, B : Candidate) return Boolean is
   begin
      if A.Reachable /= B.Reachable then
         return A.Reachable;
      end if;
      if A.Within_Jaw /= B.Within_Jaw then
         return A.Within_Jaw;
      end if;
      if A.Off_Ok /= B.Off_Ok then
         return A.Off_Ok;
      end if;
      if A.Tilt_Ok /= B.Tilt_Ok then
         return A.Tilt_Ok;
      end if;
      if A.Com_Ok /= B.Com_Ok then
         return A.Com_Ok;
      end if;
      if A.Depth_M /= B.Depth_M then
         return A.Depth_M > B.Depth_M;
      end if;
      return A.Seq < B.Seq;
   end Before;
   package Cand_Sort is new Cand_Vectors.Generic_Sorting (Before);

   procedure Candidates (Pts : V3_Vectors.Vector; G : Gripper; Support_Z : Long_Float; Gd : Grid; Found : out Cand_Vectors.Vector; Why : out Refusal) is
      Np : constant Natural := Natural (Pts.Length);
      Pa : constant V3_Array := To_Array (Pts);
      Bd : V3_Array (0 .. Np - 1);
      Nb : Natural;
      Span : Long_Float;
      Z0 : Long_Float := Long_Float'Last;
      Z1 : Long_Float := Long_Float'First;
      Com_X, Com_Y : Long_Float := 0.0;
      Seq : Natural := 0;
      Fw : constant Long_Float := Long_Float'Max (Gd.Finger_W_M, 1.0e-4);
      Gp : constant Long_Float := Long_Float'Max (Gd.Gap_M, 1.0e-4);
      Hh : constant Long_Float := 0.5 * Long_Float'Max (Gd.Jaw_H_M, 1.0e-4);
   begin
      Found := Cand_Vectors.Empty_Vector;
      Why := Fine;
      if G.Jaw.Source = Unknown or else G.Jaw.Metres <= 0.0 then
         Why := Jaw_Span_Unknown;
         return;
      end if;
      Span := G.Jaw.Metres;
      if Np < 8 then   --  少于 8 个点算不出截面(点数)
         Why := Too_Few_Points;
         return;
      end if;
      --  这团点云的水平重心:表面点的质心,不是真重心(密度不均时两者不同)。几何能给的只有这个,而它已经足以把"抓在剪刀手柄圆环上"排到后面去
      for P of Pa loop
         Z0 := Long_Float'Min (Z0, P (2));
         Z1 := Long_Float'Max (Z1, P (2));
         Com_X := Com_X + P (0) / Long_Float (Np);
         Com_Y := Com_Y + P (1) / Long_Float (Np);
      end loop;
      if Z1 - Z0 < 1.0e-4 then
         Why := Flat;
         return;
      end if;
      for Bi in 0 .. Gd.Bands - 1 loop
         --  层的中心在物体高度上均匀铺开,而层的厚度是爪面高度:厚度归身体,位置归场景
         declare
            Cz : constant Long_Float := Z0 + (Z1 - Z0) * (Long_Float (Bi) + 0.5) / Long_Float (Gd.Bands);
            Lo : constant Long_Float := Cz - Hh;
            Hi : constant Long_Float := Cz + Hh;
         begin
            Nb := 0;
            for P of Pa loop
               if P (2) >= Lo and then P (2) <= Hi then
                  Bd (Nb) := P;
                  Nb := Nb + 1;
               end if;
            end loop;
            if Nb >= Gd.Min_Pts then
             declare
               Band : V3_Array renames Bd (0 .. Nb - 1);
               Label : Nat_Array (0 .. Nb - 1);
               Nc : Natural;
             begin
               Clusters (Band, Gp, Label, Nc);
               for Ci in 0 .. Nc - 1 loop
                  declare
                     Cnt : Natural := 0;
                  begin
                     for I in 0 .. Nb - 1 loop
                        if Label (I) = Ci then
                           Cnt := Cnt + 1;
                        end if;
                     end loop;
                  if Cnt >= Gd.Min_Pts then
                   declare
                     Cl : Nat_Array (0 .. Cnt - 1);
                     Kc : Natural := 0;
                   begin
                     for I in 0 .. Nb - 1 loop
                        if Label (I) = Ci then
                           Cl (Kc) := I;
                           Kc := Kc + 1;
                        end if;
                     end loop;
                     for Di in 0 .. Gd.Dirs - 1 loop
                        declare
                           --  N = 合爪方向;T = 与它垂直的水平方向(指头沿 T 铺开)
                           Th : constant Long_Float := Pi * Long_Float (Di) / Long_Float (Gd.Dirs);
                           Nx : constant Long_Float := Cos (Th);
                           Ny : constant Long_Float := Sin (Th);
                           Tx : constant Long_Float := -Ny;
                           Ty : constant Long_Float := Nx;
                           T_Lo : Long_Float := Long_Float'Last;
                           T_Hi : Long_Float := Long_Float'First;
                           N_Strip : Natural;
                           --  扫一条带子 [A, B] 里所有连续的料段:沿合爪方向按有料/没料切(空隙用 Gp,和分块同一把尺,不引新常数)。
                           --  量的是"料的一段有多厚",不是"最左到最右有多远":拿去量一个甜甜圈,旧写法说"厚 10 厘米",真实是 2 cm 圈边 + 6 cm 洞 + 2 cm 圈边。
                           --  实测代价(2026-08-28):剪刀 156 条候选全部"比钳口宽" —— 指环量出来是外径 3 cm,而人夹的是那 3 mm 的环壁,那个候选在池子里根本不存在
                           function Scan (A, B : Long_Float) return Seg_Vectors.Vector is
                              V : QPt_Array (0 .. Cl'Length - 1);
                              Kq : Natural := 0;
                              Res : Seg_Vectors.Vector;
                              I0 : Natural := 0;
                           begin
                              for I of Cl loop
                                 declare
                                    P : V3 renames Band (I);
                                    U : constant Long_Float := P (0) * Tx + P (1) * Ty;
                                 begin
                                    if U >= A and then U <= B then
                                       V (Kq) := (Q => P (0) * Nx + P (1) * Ny, U => U, X => P (0), Y => P (1));
                                       Kq := Kq + 1;
                                    end if;
                                 end;
                              end loop;
                              Sort_QPts (V (0 .. Kq - 1));
                              for I in 1 .. Kq loop
                                 if I = Kq or else V (I).Q - V (I - 1).Q > Gp then
                                    declare
                                       Sg : Seg := (Lo => V (I0).Q, Hi => V (I - 1).Q, Sx => 0.0, Sy => 0.0, Su => 0.0, K => 0);
                                    begin
                                       for J in I0 .. I - 1 loop
                                          Sg.Sx := Sg.Sx + V (J).X;
                                          Sg.Sy := Sg.Sy + V (J).Y;
                                          Sg.Su := Sg.Su + V (J).U;
                                          Sg.K := Sg.K + 1;
                                       end loop;
                                       Res.Append (Sg);
                                       I0 := I;
                                    end;
                                 end if;
                              end loop;
                              return Res;
                           end Scan;
                        begin
                           for I of Cl loop
                              declare
                                 U : constant Long_Float := Band (I) (0) * Tx + Band (I) (1) * Ty;
                              begin
                                 T_Lo := Long_Float'Min (T_Lo, U);
                                 T_Hi := Long_Float'Max (T_Hi, U);
                              end;
                           end loop;
                           --  沿 T 切成【指头宽】的条,逐条量沿 N 的跨度:这才是爪子真要跨过的东西,指头只覆盖它自己那一条
                           N_Strip := Natural'Max (1, Natural (Long_Float'Ceiling ((T_Hi - T_Lo) / Fw)));
                           for Si in 0 .. N_Strip - 1 loop
                              declare
                                 A : constant Long_Float := T_Lo + Fw * Long_Float (Si);
                                 B : constant Long_Float := T_Lo + Fw * Long_Float (Si + 1);
                                 Half : constant Long_Float := 0.5 * (A + B);
                                 Segs : constant Seg_Vectors.Vector := Scan (A, B);
                                 Ns : constant Natural := Natural (Segs.Length);
                                 Pairs : Pair_Vectors.Vector;
                              begin
                                 --  一条带子里不止一种夹法 —— 料和洞排成一串,配对有三种:
                                 --  ① 夹住一块料(同一段料的两侧)⇒ 拿起来;② 从外面捏拢(料 i 的外面 ~ 料 j 的外面)⇒ 把中间的东西捏住;
                                 --  ③ 从里面撑开(料 i 的里面 ~ 料 j 的里面,手指在洞里)⇒ 驱动机构(手指伸进两个环、往外撑把剪刀张开)。
                                 --  三种在候选里长得一模一样,区别只在接触集那一栏的运动方向,所以这里只管把它们都生出来
                                 for I in 0 .. Ns - 1 loop
                                    for J in I .. Ns - 1 loop
                                       declare
                                          Sa : constant Seg := Segs (I);
                                          Sb : constant Seg := Segs (J);
                                       begin
                                          if I = J then
                                             Pairs.Append (Pair'(Mn => Sa.Lo, Mx => Sa.Hi, Sx => Sa.Sx, Sy => Sa.Sy, K => Sa.K));
                                          else
                                             Pairs.Append (Pair'(Mn => Sa.Lo, Mx => Sb.Hi, Sx => Sa.Sx + Sb.Sx, Sy => Sa.Sy + Sb.Sy, K => Sa.K + Sb.K));
                                             if Sb.Lo > Sa.Hi then
                                                Pairs.Append (Pair'(Mn => Sa.Hi, Mx => Sb.Lo, Sx => Sa.Sx + Sb.Sx, Sy => Sa.Sy + Sb.Sy, K => Sa.K + Sb.K));
                                             end if;
                                          end if;
                                       end;
                                    end loop;
                                 end loop;
                                 for Pr of Pairs loop
                                    --  这里不许因为"太宽"而丢掉一条候选:宽度只排序,不否决
                                    if Pr.K >= Gd.Min_Pts and then Pr.Mx - Pr.Mn > 0.0 then
                                       declare
                                          Width : constant Long_Float := Pr.Mx - Pr.Mn;
                                          Mid_Q : constant Long_Float := 0.5 * (Pr.Mn + Pr.Mx);
                                          Depth : Long_Float := Fw;
                                          Tilt : Long_Float;
                                          Px : constant Long_Float := Pr.Sx / Long_Float (Pr.K);
                                          Py : constant Long_Float := Pr.Sy / Long_Float (Pr.K);
                                          R : constant Long_Float := Sqrt ((Px - G.Base_X) ** 2 + (Py - G.Base_Y) ** 2);
                                          C : Candidate;
                                       begin
                                          --  深度:向两侧数,同一段料(中心最接近的那一段)厚度还在同一档(上下四分之一,比例)的邻条能连多长。
                                          --  必须按段去比,不能按整条的最小最大比 —— 否则一个圈的"深度"会是外径的连续性,和这一段环壁毫无关系
                                          for Side in 0 .. 1 loop
                                             declare
                                                Stp : constant Integer := (if Side = 0 then -1 else 1);
                                                J : Integer := Si + Stp;
                                             begin
                                                while J >= 0 and then J < N_Strip loop
                                                   declare
                                                      Nb : constant Seg_Vectors.Vector := Scan (T_Lo + Fw * Long_Float (J), T_Lo + Fw * Long_Float (J + 1));
                                                      Best_D : Long_Float := Long_Float'Last;
                                                      Have : Boolean := False;
                                                      M2, X2 : Long_Float := 0.0;
                                                   begin
                                                      for Sg of Nb loop
                                                         if Sg.K >= Gd.Min_Pts then
                                                            declare
                                                               D : constant Long_Float := abs (0.5 * (Sg.Lo + Sg.Hi) - Mid_Q);
                                                            begin
                                                               if D < Best_D then
                                                                  Best_D := D;
                                                                  Have := True;
                                                                  M2 := Sg.Lo;
                                                                  X2 := Sg.Hi;
                                                               end if;
                                                            end;
                                                         end if;
                                                      end loop;
                                                      exit when not Have;
                                                      exit when X2 - M2 <= 0.0 or else abs ((X2 - M2) - Width) > 0.25 * Width;
                                                      Depth := Depth + Fw;
                                                      J := J + Stp;
                                                   end;
                                                end loop;
                                             end;
                                          end loop;
                                          --  两个夹持面歪多少 —— 摩擦锥那一条,写成不需要 μ 的形式:力封闭要求两点连线落在各自的摩擦锥内,锥的半角是 atan(μ),
                                          --  连线就是合爪方向 ⇒ 条件等价于每个接触面的法向与合爪方向的夹角 < atan(μ)。这里量的正是那个夹角:
                                          --  沿指头宽方向把这一段再切两半,看近面/远面的深度随位置变化的斜率取反正切
                                          declare
                                             N_Lo, N_Hi : Long_Float := Long_Float'Last;
                                             X_Lo, X_Hi : Long_Float := Long_Float'First;
                                             C_Lo, C_Hi : Natural := 0;
                                          begin
                                             for I of Cl loop
                                                declare
                                                   P : constant V3 := Band (I);
                                                   U : constant Long_Float := P (0) * Tx + P (1) * Ty;
                                                   Q : constant Long_Float := P (0) * Nx + P (1) * Ny;
                                                begin
                                                   if U >= A and then U <= B and then Q >= Pr.Mn - Gp and then Q <= Pr.Mx + Gp then
                                                      if U < Half then
                                                         N_Lo := Long_Float'Min (N_Lo, Q);
                                                         X_Lo := Long_Float'Max (X_Lo, Q);
                                                         C_Lo := C_Lo + 1;
                                                      else
                                                         N_Hi := Long_Float'Min (N_Hi, Q);
                                                         X_Hi := Long_Float'Max (X_Hi, Q);
                                                         C_Hi := C_Hi + 1;
                                                      end if;
                                                   end if;
                                                end;
                                             end loop;
                                             if C_Lo = 0 or else C_Hi = 0 then
                                                --  半条上没有点 ⇒ 量不出斜率。不许当成 0(那是"完美正对"):拿不到值必须倒向不利的那一边,给一个直角
                                                Tilt := 0.5 * Pi;
                                             else
                                                declare
                                                   Run : constant Long_Float := 0.5 * Fw;
                                                begin
                                                   Tilt := Long_Float'Max (abs Arctan ((N_Hi - N_Lo) / Run), abs Arctan ((X_Hi - X_Lo) / Run));
                                                end;
                                             end if;
                                          end;
                                          C.Pos := [Px, Py, 0.5 * (Lo + Hi)];
                                          C.Close_Yaw := Th + 0.5 * Pi;
                                          C.Width_M := Width;
                                          C.Margin_M := Span - Width;
                                          C.Above_Support_M := Long_Float'Max (Lo, Z0) - Support_Z;
                                          C.Reach_R := R;
                                          C.Reachable := R >= G.Reach_Lo and then R <= G.Reach_Hi;
                                          C.Jaw_Declared := G.Jaw.Source = Declared;
                                          C.Within_Jaw := Width < Span;
                                          C.N_Pts := Pr.K;
                                          C.Depth_M := Depth;
                                          C.Face_Tilt_Rad := Tilt;
                                          C.Com_Offset_M := Sqrt ((Px - Com_X) ** 2 + (Py - Com_Y) ** 2);
                                          C.Off_Ok := C.Above_Support_M >= Gd.Min_Above_M;
                                          C.Seq := Seq;
                                          Seq := Seq + 1;
                                          Found.Append (C);
                                       end;
                                    end if;
                                 end loop;
                              end;
                           end loop;
                        end;
                     end loop;
                   end;
                  end if;
                  end;
               end loop;
             end;
            end if;
         end;
      end loop;
      if Found.Is_Empty then
         Why := No_Section;
         return;
      end if;
      --  门槛由这一批候选自己的中位数给,不是我拍一个角度:μ 没量过,拍一个 atan(μ) 就是编数;而"比这批里一半的候选更正"不用任何常数
      declare
         Nf : constant Natural := Natural (Found.Length);
         Tilts, Coms : Float_Array (0 .. Nf - 1);
         Tilt_Med, Com_Med : Long_Float;
      begin
         for I in 0 .. Nf - 1 loop
            Tilts (I) := Found (I).Face_Tilt_Rad;
            Coms (I) := Found (I).Com_Offset_M;
         end loop;
         Tilt_Med := Median (Tilts);
         Com_Med := Median (Coms);
         for I in 0 .. Nf - 1 loop
            declare
               C : Candidate := Found (I);
            begin
               C.Tilt_Ok := C.Face_Tilt_Rad <= Tilt_Med;
               C.Com_Ok := C.Com_Offset_M <= Com_Med;
               Found.Replace_Element (I, C);
            end;
         end loop;
      end;
      Cand_Sort.Sort (Found);
   end Candidates;

   function Thickness_At (Pts : V3_Vectors.Vector; Px, Py, Pz, Close_Yaw, Band_H_M, Finger_W_M : Long_Float; Ok : out Boolean) return Long_Float is
      --  Close_Yaw 是爪面朝向;合爪方向与它垂直
      Th : constant Long_Float := Close_Yaw - 0.5 * Pi;
      Nx : constant Long_Float := Cos (Th);
      Ny : constant Long_Float := Sin (Th);
      Tx : constant Long_Float := -Sin (Th);
      Ty : constant Long_Float := Cos (Th);
      U0 : constant Long_Float := Px * Tx + Py * Ty;
      Hw : constant Long_Float := 0.5 * Long_Float'Max (Finger_W_M, 1.0e-4);
      Hh : constant Long_Float := 0.5 * Long_Float'Max (Band_H_M, 1.0e-4);
      Mn : Long_Float := Long_Float'Last;
      Mx : Long_Float := Long_Float'First;
      K : Natural := 0;
   begin
      Ok := False;
      if Natural (Pts.Length) < 4 then
         return 0.0;
      end if;
      for Q of To_Array (Pts) loop
         if abs (Q (2) - Pz) <= Hh and then abs (Q (0) * Tx + Q (1) * Ty - U0) <= Hw then
            declare
               V : constant Long_Float := Q (0) * Nx + Q (1) * Ny;
            begin
               Mn := Long_Float'Min (Mn, V);
               Mx := Long_Float'Max (Mx, V);
               K := K + 1;
            end;
         end if;
      end loop;
      --  那儿根本没有料 ⇒ Ok = False,不是 0:合爪合到空气里和夹住一片薄刃是两件事
      if K < 3 then
         return 0.0;
      end if;
      Ok := True;
      return Mx - Mn;
   end Thickness_At;

   --  两个接触点 = 中心 ± (宽/2) × 合爪方向。这不是假设两根手指 —— 它是"沿这个方向、隔这么宽,有两个相对的面"这件几何事实;三指五指由别的生成器给更多点。
   --  锥 = 摩擦锥,半张角 = Half(有多大),不是 Face_Tilt(要多大):上一版填反了,越差的抓取在判据里看起来越可行
   procedure Build (C : Candidate; Half : Long_Float; Motion : Twist; Tol_M : Long_Float; S : out Set) is
      Cy : constant Long_Float := Cos (C.Close_Yaw);
      Sy : constant Long_Float := Sin (C.Close_Yaw);
      Hw : constant Long_Float := 0.5 * C.Width_M;
      function Mk (Sign : Long_Float) return Point is
        ((By => (Hand, 0),
          Pos => [C.Pos (0) + Cy * Hw * Sign, C.Pos (1) + Sy * Hw * Sign, C.Pos (2)],
          Normal => [Cy * Sign, Sy * Sign, 0.0],
          Push => (Axis => [-Cy * Sign, -Sy * Sign, 0.0], Half_Angle => Half),
          Pull => False, Torsion => False, Peel => False, Tol_M => Tol_M));
   begin
      S := (Points => Point_Vectors.Empty_Vector, Motion => Motion, Has_Approach => True, Approach => [0.0, 0.0, -1.0]);
      S.Points.Append (Mk (-1.0));
      S.Points.Append (Mk (1.0));
      --  进场方向 = 支撑面法向的反向:②a 本来就建立在"有一张支撑面"之上(按水平层切片、按 Min_Above 判伸不伸得进去)。
      --  换一台把支撑面立起来的机器,先 To_Upright 把点云转过来,算完 Rotate 回去 —— 这一项就跟着支撑面走了
   end Build;

   procedure To_Set (C : Candidate; Mu : Long_Float; Motion : Twist; Tol_M : Long_Float; S : out Set; Why : out Handoff) is
   begin
      S := (Points => Point_Vectors.Empty_Vector, Motion => Motion, Has_Approach => False, Approach => [others => 0.0]);
      Why := (Kind => Fine, Need_Rad => 0.0, Have_Rad => 0.0);
      if not Mu'Valid or else Mu <= 0.0 then
         Why.Kind := Mu_Unknown;
         return;
      end if;
      declare
         Have : constant Long_Float := Arctan (Mu);
      begin
         if C.Face_Tilt_Rad > Have then
            Why := (Kind => Would_Slip, Need_Rad => C.Face_Tilt_Rad, Have_Rad => Have);
            return;
         end if;
         Build (C, Have, Motion, Tol_M, S);
      end;
   end To_Set;

   procedure To_Set_Least_Mu (C : Candidate; Motion : Twist; Tol_M : Long_Float; S : out Set) is
   begin
      Build (C, C.Face_Tilt_Rad, Motion, Tol_M, S);
   end To_Set_Least_Mu;

   --  ── 支撑面在哪,变成一个参数 ──

   function Between (From, To : V3; Ok : out Boolean) return Rot is
      Oa, Ob, Oc : Boolean;
      A : constant V3 := Unit (From, Oa);
      B : constant V3 := Unit (To, Ob);
      D : Long_Float;
   begin
      Ok := False;
      if not (Oa and Ob) then
         return (Axis => [0.0, 0.0, 1.0], Ang => 0.0);
      end if;
      D := Long_Float'Max (-1.0, Long_Float'Min (1.0, Dot (A, B)));
      if D > 1.0 - 1.0e-12 then
         Ok := True;
         return (Axis => [0.0, 0.0, 1.0], Ang => 0.0);
      end if;
      if D < -1.0 + 1.0e-12 then
         --  正好反向:任取一条与 A 垂直的轴转 π(0.9 是无量纲的比较:挑一条不和 A 平行的种子轴)
         declare
            Seed : constant V3 := (if abs A (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);
            Ax : constant V3 := Unit (Cross (A, Seed), Oc);
         begin
            Ok := Oc;
            return (Axis => Ax, Ang => Pi);
         end;
      end if;
      declare
         Ax : constant V3 := Unit (Cross (A, B), Oc);
      begin
         Ok := Oc;
         return (Axis => Ax, Ang => Arccos (D));
      end;
   end Between;

   function Inverse (R : Rot) return Rot is ((Axis => R.Axis, Ang => -R.Ang));

   function Dir (R : Rot; V : V3) return V3 is
      C : constant Long_Float := Cos (R.Ang);
      Sn : constant Long_Float := Sin (R.Ang);
      Kv : constant V3 := Cross (R.Axis, V);
      Kd : constant Long_Float := Dot (R.Axis, V);
      O : V3;
   begin
      for I in 0 .. 2 loop
         O (I) := V (I) * C + Kv (I) * Sn + R.Axis (I) * Kd * (1.0 - C);
      end loop;
      return O;
   end Dir;

   function Rotate (R : Rot; S : Set) return Set is
      O : Set := S;
   begin
      for I in 0 .. Natural (O.Points.Length) - 1 loop
         declare
            P : Point := O.Points (I);
         begin
            P.Pos := Dir (R, P.Pos);
            P.Normal := Dir (R, P.Normal);
            P.Push.Axis := Dir (R, P.Push.Axis);
            O.Points.Replace_Element (I, P);
         end;
      end loop;
      O.Motion := (Lin => Dir (R, S.Motion.Lin), Ang => Dir (R, S.Motion.Ang), Pivot => Dir (R, S.Motion.Pivot));
      if S.Has_Approach then
         O.Approach := Dir (R, S.Approach);
      end if;
      return O;
   end Rotate;

   procedure To_Upright (Cloud : in out V3_Vectors.Vector; Support_Normal : V3; Back : out Rot; Ok : out Boolean) is
      T : constant Rot := Between (Support_Normal, [0.0, 0.0, 1.0], Ok);
   begin
      Back := Inverse (T);
      if not Ok then
         return;
      end if;
      for I in 0 .. Natural (Cloud.Length) - 1 loop
         declare
            Pq : constant V3 := Cloud (I);
         begin
            Cloud.Replace_Element (I, Dir (T, Pq));
         end;
      end loop;
   end To_Upright;

   --  ── 另外两种手 ──

   function Gap_Of (Ca : V3_Array) return Long_Float is
      N : constant Natural := Ca'Length;
      D : Float_Array (0 .. N - 1);
      K : Natural := 0;
   begin
      for I in 0 .. N - 1 loop
         declare
            Best : Long_Float := Long_Float'Last;
            A : V3 renames Ca (I);
         begin
            for J in 0 .. N - 1 loop
               if J /= I then
                  declare
                     B : V3 renames Ca (J);
                  begin
                     Best := Long_Float'Min (Best, Norm ([B (0) - A (0), B (1) - A (1), B (2) - A (2)]));
                  end;
               end if;
            end loop;
            if Best'Valid then
               D (K) := Best;
               K := K + 1;
            end if;
         end;
      end loop;
      if K = 0 then
         return 0.0;
      end if;
      return Median (D (0 .. K - 1));
   end Gap_Of;

   function Sampling_Gap (Cloud : V3_Vectors.Vector) return Long_Float is (Gap_Of (To_Array (Cloud)));

   --  给一条法向配两条与它垂直的基(0.9 是无量纲的比较:挑一条不和法向平行的种子轴)
   procedure Plane_Basis (N : V3; U, W : out V3) is
      Seed : constant V3 := (if abs N (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);
      D : constant Long_Float := Dot (Seed, N);
      Ok : Boolean;
      U0 : constant V3 := Unit ([Seed (0) - N (0) * D, Seed (1) - N (1) * D, Seed (2) - N (2) * D], Ok);
   begin
      U := (if Ok then U0 else [1.0, 0.0, 0.0]);
      W := Cross (N, U);
   end Plane_Basis;

   function Centroid (Cloud : V3_Array) return V3 is
      K : constant Long_Float := Long_Float (Cloud'Length);
      G : V3 := [others => 0.0];
   begin
      for P of Cloud loop
         for I in 0 .. 2 loop
            G (I) := G (I) + P (I) / K;
         end loop;
      end loop;
      return G;
   end Centroid;

   --  一组点的协方差最小特征向量 = 局部平面的法向。3×3 循环 Jacobi,零依赖
   function Min_Eigvec (Pts : V3_Array; Ok : out Boolean) return V3 is
      K : constant Long_Float := Long_Float (Pts'Length);
      M : V3 := [others => 0.0];
      C : array (0 .. 2, 0 .. 2) of Long_Float := [others => [others => 0.0]];
      V : array (0 .. 2, 0 .. 2) of Long_Float := [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
      Lo : Natural := 0;
   begin
      for P of Pts loop
         for I in 0 .. 2 loop
            M (I) := M (I) + P (I) / K;
         end loop;
      end loop;
      for P of Pts loop
         declare
            D : constant V3 := [P (0) - M (0), P (1) - M (1), P (2) - M (2)];
         begin
            for I in 0 .. 2 loop
               for J in 0 .. 2 loop
                  C (I, J) := C (I, J) + D (I) * D (J);
               end loop;
            end loop;
         end;
      end loop;
      --  最多 40 轮(次数);非对角项平方和小于 1e-28、单项小于 1e-20 就算作零(无量纲意义上的"算作零":比双精度舍入还小几个量级)
      for Sweep in 1 .. 40 loop
         declare
            Off : Long_Float := 0.0;
         begin
            for I in 0 .. 2 loop
               for J in I + 1 .. 2 loop
                  Off := Off + C (I, J) * C (I, J);
               end loop;
            end loop;
            exit when Off < 1.0e-28;   --  算作零(无量纲意义:比双精度舍入小几个量级)
         end;
         for P in 0 .. 2 loop
            for Q in P + 1 .. 2 loop
               --  单项小于 1e-20 算作零(无量纲意义:比双精度舍入小几个量级)
               if abs C (P, Q) >= 1.0e-20 then
                  declare
                     Th : constant Long_Float := (C (Q, Q) - C (P, P)) / (2.0 * C (P, Q));
                     Sg : constant Long_Float := (if Th >= 0.0 then 1.0 else -1.0);
                     T : constant Long_Float := Sg / (abs Th + Sqrt (Th * Th + 1.0));
                     Cc : constant Long_Float := 1.0 / Sqrt (T * T + 1.0);
                     Ss : constant Long_Float := T * Cc;
                  begin
                     for K2 in 0 .. 2 loop
                        declare
                           Kp : constant Long_Float := C (K2, P);
                           Kq : constant Long_Float := C (K2, Q);
                        begin
                           C (K2, P) := Cc * Kp - Ss * Kq;
                           C (K2, Q) := Ss * Kp + Cc * Kq;
                        end;
                     end loop;
                     for K2 in 0 .. 2 loop
                        declare
                           Pk : constant Long_Float := C (P, K2);
                           Qk : constant Long_Float := C (Q, K2);
                        begin
                           C (P, K2) := Cc * Pk - Ss * Qk;
                           C (Q, K2) := Ss * Pk + Cc * Qk;
                        end;
                     end loop;
                     for K2 in 0 .. 2 loop
                        declare
                           Kp : constant Long_Float := V (K2, P);
                           Kq : constant Long_Float := V (K2, Q);
                        begin
                           V (K2, P) := Cc * Kp - Ss * Kq;
                           V (K2, Q) := Ss * Kp + Cc * Kq;
                        end;
                     end loop;
                  end;
               end if;
            end loop;
         end loop;
      end loop;
      for I in 1 .. 2 loop
         if C (I, I) < C (Lo, Lo) then
            Lo := I;
         end if;
      end loop;
      return Unit ([V (0, Lo), V (1, Lo), V (2, Lo)], Ok);
   end Min_Eigvec;

   procedure Suction (Cloud : V3_Vectors.Vector; Cup_R_M, Flat_Tol_M, Mu : Long_Float; Motion : Twist; Tol_M : Long_Float; S : out Set; Why : out No_Hand) is
      Ca : constant V3_Array := To_Array (Cloud);
      Spacing : Long_Float;
      Have_Best : Boolean := False;
      Best_Span : Long_Float := 0.0;
      Best_At, Best_N : V3 := [others => 0.0];
      Seen_R : Long_Float := 0.0;
   begin
      S := (Points => Point_Vectors.Empty_Vector, Motion => Motion, Has_Approach => False, Approach => [others => 0.0]);
      Why := (others => <>);
      if Natural (Cloud.Length) < 8 then   --  点数
         Why.Kind := Too_Few_Points;
         return;
      end if;
      if not Mu'Valid or else Mu <= 0.0 then
         Why.Kind := Handed_Off;
         Why.H.Kind := Mu_Unknown;
         return;
      end if;
      Spacing := Gap_Of (Ca);
      for Cp of Ca loop
         declare
            Mid : constant V3 := Cp;
            Near : V3_Array (0 .. Ca'Length - 1);
            Kn : Natural := 0;
            Ok0 : Boolean;
            function Along (P, N : V3) return Long_Float is (Dot ([P (0) - Mid (0), P (1) - Mid (1), P (2) - Mid (2)], N));
            function Flat_R (P, N : V3) return Long_Float is
               D : constant V3 := [P (0) - Mid (0), P (1) - Mid (1), P (2) - Mid (2)];
               A : constant Long_Float := Dot (D, N);
            begin
               return Norm ([D (0) - N (0) * A, D (1) - N (1) * A, D (2) - N (2) * A]);
            end Flat_R;
         begin
            for P of Ca loop
               if Norm ([P (0) - Mid (0), P (1) - Mid (1), P (2) - Mid (2)]) <= Cup_R_M then
                  Near (Kn) := P;
                  Kn := Kn + 1;
               end if;
            end loop;
            if Kn >= 6 then   --  邻居少于 6 个拟不出平面(点数)
               declare
                  N0 : constant V3 := Min_Eigvec (Near (0 .. Kn - 1), Ok0);
                  Far_Lo : Long_Float := Long_Float'Last;
                  Far_Hi : Long_Float := Long_Float'First;
                  R_Lo : Long_Float := Long_Float'Last;
                  R_Hi : Long_Float := Long_Float'First;
                  N_Far : Natural := 0;
                  Keep : Boolean := True;
               begin
                  if Ok0 then
                     --  把"背面"和"弯曲"分开 —— 两件事长得一样,处理方式完全相反。
                     --  一张 4 mm 厚的板子两个面都落在吸盘半径以内 ⇒ 拟出来的"平面"横跨两面,法向是错的 ⇒ 背面那一片要筛掉;
                     --  但一个球的邻居也一样偏离平面,那是真的不平,必须判死(只挑最平的一小块去拟,球也吸得住了 —— 那一版球从 1/13 跳到 10/13,是假的)。
                     --  判别式:偏离量跟不跟半径走。背面:偏离量 ≈ 板厚,与半径无关;弯曲:偏离量 ≈ ρ²/2R,随半径长大
                     for P of Near (0 .. Kn - 1) loop
                        declare
                           A : constant Long_Float := abs Along (P, N0);
                        begin
                           if A > Flat_Tol_M then
                              N_Far := N_Far + 1;
                              Far_Lo := Long_Float'Min (Far_Lo, A);
                              Far_Hi := Long_Float'Max (Far_Hi, A);
                              declare
                                 Rr : constant Long_Float := Flat_R (P, N0);
                              begin
                                 R_Lo := Long_Float'Min (R_Lo, Rr);
                                 R_Hi := Long_Float'Max (R_Hi, Rr);
                              end;
                           end if;
                        end;
                     end loop;
                     if N_Far > 0 then
                        Keep := (Far_Hi - Far_Lo) <= Flat_Tol_M and then R_Hi - R_Lo > 0.5 * Cup_R_M;
                     end if;
                     if Keep then
                        declare
                           Near2 : V3_Array (0 .. Kn - 1);
                           K2 : Natural := 0;
                           Ok1 : Boolean;
                        begin
                           for P of Near (0 .. Kn - 1) loop
                              if abs Along (P, N0) <= Flat_Tol_M then
                                 Near2 (K2) := P;
                                 K2 := K2 + 1;
                              end if;
                           end loop;
                           if K2 >= 6 then
                              declare
                                 N : constant V3 := Min_Eigvec (Near2 (0 .. K2 - 1), Ok1);
                                 U, W : V3;
                                 Off : Long_Float := 0.0;
                                 --  8 个扇区(次数):每个方向都得有料,铺满程度取各扇区里最小的那一个 —— 只取"最远的邻居有多远",吸盘落在面的边沿上也能过
                                 Sector : array (0 .. 7) of Long_Float := [others => 0.0];
                                 Span : Long_Float := Long_Float'Last;
                              begin
                                 if Ok1 then
                                    Plane_Basis (N, U, W);
                                    for P of Near2 (0 .. K2 - 1) loop
                                       declare
                                          D : constant V3 := [P (0) - Mid (0), P (1) - Mid (1), P (2) - Mid (2)];
                                          A : constant Long_Float := Dot (D, N);
                                          Fl : constant V3 := [D (0) - N (0) * A, D (1) - N (1) * A, D (2) - N (2) * A];
                                          Rr : constant Long_Float := Norm (Fl);
                                       begin
                                          Off := Long_Float'Max (Off, abs A);
                                          if Rr >= 1.0e-12 then
                                             declare
                                                Ang : constant Long_Float := Arctan (Dot (Fl, W), Dot (Fl, U));
                                                --  一圈分 8 个扇区(次数,无量纲)
                                                Kk : constant Natural := Natural (Long_Float'Floor ((Ang / (2.0 * Pi) + 1.0) * 8.0)) mod 8;
                                             begin
                                                Sector (Kk) := Long_Float'Max (Sector (Kk), Rr);
                                             end;
                                          end if;
                                       end;
                                    end loop;
                                    if Off <= Flat_Tol_M then
                                       for Sv of Sector loop
                                          Span := Long_Float'Min (Span, Sv);
                                       end loop;
                                       Seen_R := Long_Float'Max (Seen_R, Span);
                                       --  采样密度是分辨率的下界:门槛是 span + 采样间距 ≥ 吸盘半径,不是 span ≥ 吸盘半径(后者要求恰好有一个采样点落在吸盘边缘上,那是采样伪影)
                                       if Span + Spacing + 1.0e-12 >= Cup_R_M and then (not Have_Best or else Span > Best_Span) then
                                          Have_Best := True;
                                          Best_Span := Span;
                                          Best_At := Mid;
                                          Best_N := N;
                                       end if;
                                    end if;
                                 end if;
                              end;
                           end if;
                        end;
                     end if;
                  end if;
               end;
            end if;
         end;
      end loop;
      if not Have_Best then
         Why := (Kind => No_Flat_Patch, Found_R => Seen_R, Need_R => Cup_R_M, Direction => 0, H => <>);
         return;
      end if;
      --  法向要指向物体外侧:取远离点云重心的那一支
      declare
         Gc : constant V3 := Centroid (Ca);
         N : V3 := Best_N;
      begin
         if Dot (N, [Best_At (0) - Gc (0), Best_At (1) - Gc (1), Best_At (2) - Gc (2)]) < 0.0 then
            N := [-N (0), -N (1), -N (2)];
         end if;
         --  吸盘的锥 = 密封面与物体之间的摩擦锥,半张角 atan(μ)。写死成 0(只准沿法向吸)把吸盘废掉一半:推/放/擦/倒/撬/翻/舀十四格全判死,
         --  而真空吸盘搬箱子天天在做侧向移动 —— 0 不是保守,是错的模型。它拉得动、拧得动(密封圈是一片面)、掰得动(密封面有半径)
         S.Points.Append (Point'(By => (Hand, 0), Pos => Best_At, Normal => N,
                                 Push => (Axis => [-N (0), -N (1), -N (2)], Half_Angle => Arctan (Mu)),
                                 Pull => True, Torsion => True, Peel => True, Tol_M => Tol_M));
         S.Has_Approach := True;
         S.Approach := [-N (0), -N (1), -N (2)];
      end;
   end Suction;

   procedure Ring (Cloud : V3_Vectors.Vector; At_Z, Band_M : Long_Float; N : Positive; Mu : Long_Float; Motion : Twist; Tol_M : Long_Float; S : out Set; Why : out No_Hand) is
      Ca : constant V3_Array := To_Array (Cloud);
      Band : V3_Array (0 .. Ca'Length - 1);
      Nb : Natural := 0;
      Cx, Cy : Long_Float := 0.0;
      Pts, Dirs : V3_Vectors.Vector;
      Sum : V3 := [others => 0.0];
      Half : Long_Float;
   begin
      S := (Points => Point_Vectors.Empty_Vector, Motion => Motion, Has_Approach => False, Approach => [others => 0.0]);
      Why := (others => <>);
      if N < 2 then
         Why.Kind := Not_Surrounding;
         return;
      end if;
      if not Mu'Valid or else Mu <= 0.0 then
         Why.Kind := Handed_Off;
         Why.H.Kind := Mu_Unknown;
         return;
      end if;
      for P of Ca loop
         if abs (P (2) - At_Z) <= 0.5 * Band_M then
            Band (Nb) := P;
            Nb := Nb + 1;
         end if;
      end loop;
      if Nb < 2 * N then
         Why.Kind := Too_Few_Points;
         return;
      end if;
      for P of Band (0 .. Nb - 1) loop
         Cx := Cx + P (0) / Long_Float (Nb);
         Cy := Cy + P (1) / Long_Float (Nb);
      end loop;
      for K in 0 .. N - 1 loop
         declare
            A : constant Long_Float := 2.0 * Pi * Long_Float (K) / Long_Float (N);
            Dx : constant Long_Float := Cos (A);
            Dy : constant Long_Float := Sin (A);
            Tn : constant Long_Float := Tan (Pi / Long_Float (N));
            Have : Boolean := False;
            Best : Long_Float := 0.0;
            Far : V3 := [others => 0.0];
         begin
            --  这个方向上、贴着这条射线的那些点里最外的一个(横向偏移不超过沿径向距离的 tan(π/N):落在这个扇区里)
            for P of Band (0 .. Nb - 1) loop
               declare
                  Ux : constant Long_Float := P (0) - Cx;
                  Uy : constant Long_Float := P (1) - Cy;
                  Along : constant Long_Float := Ux * Dx + Uy * Dy;
                  Off : constant Long_Float := abs (Ux * (-Dy) + Uy * Dx);
               begin
                  if Along > 0.0 and then Off <= Along * Tn and then (not Have or else Along > Best) then
                     Have := True;
                     Best := Along;
                     Far := P;
                  end if;
               end;
            end loop;
            if not Have then
               Why := (Kind => Nothing_In_Direction, Direction => K, others => <>);
               return;
            end if;
            Pts.Append (Far);
            Dirs.Append (V3'([Dx, Dy, 0.0]));
            Sum := [Sum (0) + Dx, Sum (1) + Dy, Sum (2)];
         end;
      end loop;
      --  正张成检查:内法向的合必须近乎为零(围住了),而不是全挤在一侧(0.5 是无量纲:N 个单位向量之和的模)
      if Norm (Sum) > 0.5 then
         Why.Kind := Not_Surrounding;
         return;
      end if;
      Half := Arctan (Mu);
      for K in 0 .. N - 1 loop
         declare
            D : constant V3 := Dirs (K);
         begin
            --  法向由质心指向外 = 物体外侧;锥朝里夹。指尖当点接触;有指腹的把 Torsion 改成 True(那是量出来的身体属性)
            S.Points.Append (Point'(By => (Hand, 0), Pos => Pts (K), Normal => D,
                                    Push => (Axis => [-D (0), -D (1), -D (2)], Half_Angle => Half),
                                    Pull => False, Torsion => False, Peel => False, Tol_M => Tol_M));
         end;
      end loop;
      S.Has_Approach := True;
      S.Approach := [0.0, 0.0, -1.0];
   end Ring;

   function Img (R : Refusal) return String is (Refusal'Image (R));

   function Img (H : Handoff) return String is
   begin
      case H.Kind is
         when Fine => return "fine";
         when Mu_Unknown => return "MuUnknown";
         when Would_Slip => return "WouldSlip need" & Long_Float'Image (H.Need_Rad) & " have" & Long_Float'Image (H.Have_Rad);
      end case;
   end Img;

   function Img (N : No_Hand) return String is
   begin
      case N.Kind is
         when Fine => return "fine";
         when Too_Few_Points => return "TooFewPoints";
         when No_Flat_Patch => return "NoFlatPatch found" & Long_Float'Image (N.Found_R) & " need" & Long_Float'Image (N.Need_R);
         when Nothing_In_Direction => return "NothingInDirection(" & Nat_Img (N.Direction) & ")";
         when Not_Surrounding => return "NotSurrounding";
         when Handed_Off => return "Handoff(" & Img (N.H) & ")";
      end case;
   end Img;

end Contact.Gen;
