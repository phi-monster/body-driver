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
