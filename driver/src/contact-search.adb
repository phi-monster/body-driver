with Ada.Numerics; use Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Containers.Ordered_Sets;
with Ada.Containers.Ordered_Maps;
with Ada.Containers.Generic_Array_Sort;
package body Contact.Search is
   use Ada.Strings.Unbounded;
   package Hd renames Contact.Wrench;
   use type Ada.Containers.Count_Type;

   function Not_Measured (Why : String) return Hand_Model is
      H : Hand_Model;
   begin
      H.Valid := False;
      H.Why := To_Unbounded_String (Why);
      return H;
   end Not_Measured;

   function Sub (A, B : V3) return V3 is ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]);
   function Add (A, B : V3) return V3 is ([A (0) + B (0), A (1) + B (1), A (2) + B (2)]);
   function Scl (K : Long_Float; A : V3) return V3 is ([K * A (0), K * A (1), K * A (2)]);

   function From_Lobes (Ls : Lobe_In_Vectors.Vector; Pos_Err : Long_Float) return Hand_Model is
      H : Hand_Model;
      Mid : V3 := [others => 0.0];
      Om : Boolean;
   begin
      if Ls.Is_Empty then
         return Not_Measured ("这只眼里量不到手指(一瓣都没有)");
      end if;
      if not Pos_Err'Valid or else Pos_Err < 0.0 then
         return Not_Measured ("手落位的误差没量出来");
      end if;
      for L of Ls loop
         if not L.Width'Valid or else L.Width <= 0.0 or else not L.Thick'Valid or else L.Thick < 0.0 then
            return Not_Measured ("有一瓣的指肚宽 / 手指厚没量出来");
         end if;
         Mid := Add (Mid, Scl (1.0 / Long_Float (Ls.Length), L.Tip));
      end loop;
      H.Tool := Unit (Mid, Om);
      if not Om then
         return Not_Measured ("指尖和眼重合");
      end if;
      H.Reach_In := Norm (Mid);   --  指尖在眼前面这么深:东西伸进手里超过它就顶到手掌 / 眼了
      for L of Ls loop
         declare
            Od : Boolean;
            To_C : constant V3 := Sub (Mid, L.Tip);
            U : constant V3 := Unit (To_C, Od);
            Dist : constant Long_Float := Norm (To_C);
         begin
            if not Od then
               return Not_Measured ("量到的只有一瓣(或几瓣的尖重合),没有可以相向合的 —— 合拢那一路没量出来");
            end if;
            if Dist <= 0.5 * L.Thick then
               return Not_Measured ("张开时有一瓣的尖离中心不到半个手指厚(尖或手指厚量错了)");
            end if;
            --  碰东西的那一面在尖往中心半个手指厚;合到头 = 走到中心再留半个手指厚(几瓣一起合,走到那儿就碰上对面的了)。
            --  ⚠ "朝中心、各走各那一段"没量(.ads 说了):一边固定一边动的夹爪、拇指对几指那种不对称的手就错(动的走满、固定的不动)。
            --  要的是每一瓣合空时的尖(开机合空那一下就看得到;今天的 Lobe_Geo 只存张开时的):有了 ⇒ Dir / Travel 按两头的尖算,这一条删
            H.Pads.Append (Pad'(Tip => Add (L.Tip, Scl (0.5 * L.Thick, U)), Dir => U, Travel => Dist - 0.5 * L.Thick, Width => L.Width, Thick => L.Thick));
         end;
      end loop;
      H.Pos_Err := Pos_Err;
      H.Valid := True;
      return H;
   end From_Lobes;

   --  绕单位轴 K 转 Th(罗德里格斯)
   function Rot (V, K : V3; Th : Long_Float) return V3 is
      C : constant Long_Float := Cos (Th);
      S : constant Long_Float := Sin (Th);
      Kv : constant V3 := Cross (K, V);
      Kd : constant Long_Float := Dot (K, V);
   begin
      return [V (0) * C + Kv (0) * S + K (0) * Kd * (1.0 - C), V (1) * C + Kv (1) * S + K (1) * Kd * (1.0 - C), V (2) * C + Kv (2) * S + K (2) * Kd * (1.0 - C)];
   end Rot;


   --  去重用的钥匙:进场方向第几个、转角第几档、指尖中点按半个采样间距取整
   type Key is record
      A, Phi, X, Y, Z : Integer;
   end record;
   function "<" (L, R : Key) return Boolean is
     (L.A < R.A or else (L.A = R.A and then (L.Phi < R.Phi or else (L.Phi = R.Phi and then (L.X < R.X or else (L.X = R.X and then
        (L.Y < R.Y or else (L.Y = R.Y and then L.Z < R.Z))))))));
   package Key_Sets is new Ada.Containers.Ordered_Sets (Key);

   --  均匀地稀疏:按体素每格留一个点(体素边长从采样间距起,点还多于 Max 就放大 1.25 倍再来;倍数只管快慢)。
   --  之后所有"一个采样间距"的门都用这个边长 —— 按下标隔着取会在一大片顶面上留下几毫米宽的空档,手指落在空档里,"落在料上"就判漏了(09-29 焊点)
   type Down is record
      Pts : V3_Vectors.Vector;
      Pitch : Long_Float := 0.0;
   end record;
   function Voxel_Down (Pts : V3_Vectors.Vector; Pitch0 : Long_Float; Max : Positive) return Down is
      --  每个体素取格里所有点的平均(只留第一个点会让所有点系统地偏在格子的一角,轮廓的法向跟着歪 —— 09-29 焊点:板的墙法向歪 27°)
      type Acc is record
         S : V3 := [others => 0.0];
         K : Natural := 0;
      end record;
      package Acc_Maps is new Ada.Containers.Ordered_Maps (Key, Acc);
      D : Down;
   begin
      D.Pitch := Pitch0;
      loop
         declare
            Mp : Acc_Maps.Map;
         begin
            for Q of Pts loop
               declare
                  K : constant Key := (A => 0, Phi => 0, X => Integer (Long_Float'Floor (Q (0) / D.Pitch)), Y => Integer (Long_Float'Floor (Q (1) / D.Pitch)),
                                       Z => Integer (Long_Float'Floor (Q (2) / D.Pitch)));
                  C : constant Acc_Maps.Cursor := Mp.Find (K);
               begin
                  if Acc_Maps.Has_Element (C) then
                     declare
                        E : Acc := Acc_Maps.Element (C);
                     begin
                        E.S := Add (E.S, Q); E.K := E.K + 1;
                        Mp.Replace_Element (C, E);
                     end;
                  else
                     Mp.Insert (K, Acc'(S => Q, K => 1));
                  end if;
               end;
            end loop;
            if Natural (Mp.Length) <= Max then
               for E of Mp loop
                  D.Pts.Append (Scl (1.0 / Long_Float (E.K), E.S));
               end loop;
               return D;
            end if;
         end;
         D.Pitch := 1.25 * D.Pitch;   --  放大的倍数(无量纲,只管快慢)
      end loop;
   end Voxel_Down;

   --  把接触法向按量得出的误差转最坏的那个方向(同一根世界轴、同一个转向,几处接触的法向就不再正对)
   function Tilted (Ts : Hd.Touch_Vectors.Vector; Dl : Bytes.Floats; Axis : V3) return Hd.Touch_Vectors.Vector is
      Out_T : Hd.Touch_Vectors.Vector := Ts;
   begin
      for I in 0 .. Natural (Ts.Length) - 1 loop
         declare
            T : Hd.Touch := Ts (I);
         begin
            T.N := Rot (T.N, Axis, Dl (I));
            Out_T.Replace_Element (I, T);
         end;
      end loop;
      return Out_T;
   end Tilted;

   --  每处接触法向的误差(Ax = 0:绕手指的轴,按那一片沿指肚宽的半宽;Ax = 1:绕指肚宽的轴,按沿手指的半高)。返回值不用(只为能写在声明里)
   function Set_Dl (Cd : Candidate; Ax : Natural; Pitch, Sigma : Long_Float; Dl : in out Bytes.Floats) return Boolean is
   begin
      Dl.Clear;
      for I in 0 .. Natural (Cd.Touches.Length) - 1 loop
         declare
            Half : constant Long_Float := (if Ax = 0 then Cd.Half_W (I) else Cd.Half_H (I));
         begin
            Dl.Append (Arctan (Long_Float'Max (Sigma, 0.5 * Pitch) / Long_Float'Max (Pitch, Half)));
         end;
      end loop;
      return True;
   end Set_Dl;

   procedure Plan (Pts, Around : V3_Vectors.Vector; Pitch_In, Sigma : Long_Float; Up, Support_P : V3; H : Hand_Model; Mu_Lb, Standoff : Long_Float;
                   Reach : not null access function (R : Geom.M3; T : V3) return Boolean;
                   Want_K : Positive; Found : out Cand_Vectors.Vector; St : out Plan_Stats;
                   Want : Contact.Want := (others => <>); Mu_Ub : Long_Float := Long_Float'Last) is
      Np0 : constant Natural := Natural (Pts.Length);
      --  扫的时候最多用这么多点(按体素均匀地稀疏,见 Voxel_Down;次数:点再多只是更慢)
      Max_Scan : constant := 1500;
      Dn : constant Down := (if Np0 = 0 or else Pitch_In <= 0.0 then (Pts => V3_Vectors.Empty_Vector, Pitch => Pitch_In) else Voxel_Down (Pts, Pitch_In, Max_Scan));
      Pitch : constant Long_Float := Dn.Pitch;   --  稀疏以后的采样间距:所有"一个采样间距"的门都用它
      Np : constant Natural := Natural (Dn.Pts.Length);
      type V3_Arr is array (Natural range <>) of V3;
      P : V3_Arr (0 .. Natural'Max (Np, 1) - 1);
      Com : V3 := [others => 0.0];
      Oku : Boolean;
      U : constant V3 := Unit (Up, Oku);
      Down : constant V3 := [-U (0), -U (1), -U (2)];
      --  手系里的底:工具轴、第 0 块合拢方向(垂直于工具轴的那一分量)、两者的叉乘
      Tool_H : constant V3 := H.Tool;
      Mid_H : V3 := [others => 0.0];
      Jaw_H : V3;
      Side_H : V3;
      Span : Long_Float := 0.0;
      Pad_W : Long_Float := 0.0;
      Th_Max : Long_Float := 0.0;
      O : V3_Vectors.Vector;   --  旁边的东西:手够得着的那一圈、按体素稀疏
      Po : Long_Float := Pitch;   --  它稀疏以后的间距(旁边的东西那几道门的余量按它)
      Seen : Key_Sets.Set;
      All_C : Cand_Vectors.Vector;
      Tilt_Steps : constant := 3;   --  进场方向离竖直分几档:把直角分成这么多份、取 0、1、2 份 = 0°、30°、60°(次数,分辨率;90° 是贴着桌面平着进,不取)
      Azims : constant := 8;      --  斜着进场时绕竖直分几个方位(分辨率)
      Rots : constant := 32;      --  绕进场方向转几档(分辨率:11.25°)
   begin
      Found.Clear;
      St := (others => <>);
      if not H.Valid or else Np0 = 0 or else Pitch <= 0.0 or else not Oku or else H.Pads.Length < 2 then
         return;
      end if;
      for I in 0 .. Np - 1 loop
         P (I) := Dn.Pts (I);
      end loop;
      for Q of Pts loop
         Com := Add (Com, Scl (1.0 / Long_Float (Np0), Q));
      end loop;
      St.Com := Com;
      --  重心按表面点的形心(密度量不出来,照实说)
      for Pd of H.Pads loop
         Mid_H := Add (Mid_H, Scl (1.0 / Long_Float (H.Pads.Length), Pd.Tip));
         Pad_W := Long_Float'Max (Pad_W, Pd.Width);
      end loop;
      Span := Norm (Sub (H.Pads (1).Tip, H.Pads (0).Tip));
      for Pd of H.Pads loop
         Span := Long_Float'Max (Span, 2.0 * Norm (Sub (Pd.Tip, Mid_H)));
         Th_Max := Long_Float'Max (Th_Max, Pd.Thick);
      end loop;
      if not Around.Is_Empty then
         declare
            Lo, Hi : V3 := P (0);
            Mg : constant Long_Float := Span + Pad_W + Th_Max + H.Reach_In;   --  手够得着的那一圈:东西的外框往外放张口 + 指肚宽 + 手指厚 + 指尖到手掌
            Near : V3_Vectors.Vector;
         begin
            for I in 1 .. Np - 1 loop
               for K in 0 .. 2 loop
                  Lo (K) := Long_Float'Min (Lo (K), P (I) (K));
                  Hi (K) := Long_Float'Max (Hi (K), P (I) (K));
               end loop;
            end loop;
            for Q of Around loop
               if (for all K in 0 .. 2 => Q (K) >= Lo (K) - Mg and then Q (K) <= Hi (K) + Mg) then
                  Near.Append (Q);
               end if;
            end loop;
            if not Near.Is_Empty then
               declare
                  Dn_O : constant Contact.Search.Down := Voxel_Down (Near, Pitch, Max_Scan);
               begin
                  O := Dn_O.Pts;
                  Po := Dn_O.Pitch;
               end;
            end if;
         end;
      end if;
      declare
         D0 : constant V3 := H.Pads (0).Dir;
         Dd : constant Long_Float := Dot (D0, Tool_H);
         Oj, Os : Boolean;
      begin
         Jaw_H := Unit ([D0 (0) - Dd * Tool_H (0), D0 (1) - Dd * Tool_H (1), D0 (2) - Dd * Tool_H (2)], Oj);
         Side_H := Unit (Cross (Tool_H, Jaw_H), Os);
         if not Oj or else not Os then
            return;
         end if;
      end;
      declare
         --  桌面内的一组底(进场方向按它转)
         Seed : constant V3 := (if abs U (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);   --  0.9:挑一条不和"上"平行的种子轴(无量纲)
         Oe : Boolean;
         E1 : constant V3 := Unit (Cross (U, Seed), Oe);
         E2 : constant V3 := Cross (U, E1);
         Ai : Natural := 0;
      begin
         for Ti in 0 .. Tilt_Steps - 1 loop
            for Az in 0 .. (if Ti = 0 then 0 else Azims - 1) loop
               declare
                  Ph : constant Long_Float := 2.0 * Pi * Long_Float (Az) / Long_Float (Azims);
                  Hz : constant V3 := Add (Scl (Cos (Ph), E1), Scl (Sin (Ph), E2));
                  Tilt : constant Long_Float := Long_Float (Ti) * Pi / Long_Float (2 * Tilt_Steps);
                  A : constant V3 := Add (Scl (Cos (Tilt), Down), Scl (Sin (Tilt), Hz));   --  进场方向(朝下、斜 Tilt)
                  Oa : Boolean;
                  J0 : constant V3 := Unit (Sub (E1, Scl (Dot (E1, A), A)), Oa);
                  K0 : constant V3 := Cross (A, J0);
               begin
                  Ai := Ai + 1;
                  if Oa then
                     --  合拢方向只取水平的:东西是顶面轮廓往下补的柱体,侧壁都是竖的,斜着合的那些在这个模型里没有意义
                     --  (09-29 焊点:斜着进场、合拢方向上下斜的那几把,一块在顶面、一块在侧壁,捏同一条边)。竖着进场 ⇒ 水平面里转一圈;
                     --  斜着进场 ⇒ 垂直于进场方向的水平方向只有一个(正反两个)。有了真三维的表面点再放开
                     for Ri in 0 .. (if Ti = 0 then Rots - 1 else 1) loop
                        declare
                           Hj : constant V3 := Unit (Cross (U, A), Oa);   --  斜着进场时唯一的水平合拢方向
                           Phi : constant Long_Float := 2.0 * Pi * Long_Float (Ri) / Long_Float (Rots);
                           J : constant V3 := (if Ti = 0 then Add (Scl (Cos (Phi), J0), Scl (Sin (Phi), K0))
                                               elsif Ri = 0 then Hj else [-Hj (0), -Hj (1), -Hj (2)]);
                           Kx : constant V3 := Cross (A, J);
                           --  眼 → 世界:把手系的 (Jaw_H, Side_H, Tool_H) 转到世界的 (J, A × J, A)
                           R : Geom.M3;
                           Np_Pads : constant Natural := Natural (H.Pads.Length);
                           type Pad_Arr is array (0 .. Np_Pads - 1) of V3;
                           Off, Dw, Ww : Pad_Arr;
                           Low : Long_Float := Long_Float'Last;
                           X_Lo, X_Hi, Y_Lo, Y_Hi : Long_Float := 0.0;
                           Have_Box : Boolean := False;
                        begin
                           for I in 0 .. 2 loop
                              for K in 0 .. 2 loop
                                 R (I, K) := J (I) * Jaw_H (K) + Kx (I) * Side_H (K) + A (I) * Tool_H (K);
                              end loop;
                           end loop;
                           for Ip in 0 .. Np_Pads - 1 loop
                              Off (Ip) := Geom.Ap (R, Sub (H.Pads (Ip).Tip, Mid_H));
                              Dw (Ip) := Geom.Ap (R, H.Pads (Ip).Dir);
                              declare
                                 Ow : Boolean;
                              begin
                                 Ww (Ip) := Unit (Cross (A, Dw (Ip)), Ow);
                                 if not Ow then
                                    Ww (Ip) := Kx;
                                 end if;
                              end;
                              Low := Long_Float'Min (Low, Dot (Off (Ip), U));
                           end loop;
                           for I in 0 .. Np - 1 loop
                              declare
                                 Q : constant V3 := Sub (P (I), Com);
                                 Qx : constant Long_Float := Dot (Q, J);
                                 Qy : constant Long_Float := Dot (Q, Kx);
                              begin
                                 if not Have_Box then
                                    X_Lo := Qx; X_Hi := Qx; Y_Lo := Qy; Y_Hi := Qy; Have_Box := True;
                                 else
                                    X_Lo := Long_Float'Min (X_Lo, Qx); X_Hi := Long_Float'Max (X_Hi, Qx);
                                    Y_Lo := Long_Float'Min (Y_Lo, Qy); Y_Hi := Long_Float'Max (Y_Hi, Qy);
                                 end if;
                              end;
                           end loop;
                           declare
                              Span_Steps : constant := 8;   --  张口分几段放起点(次数,分辨率:合拢以后按两边碰到的深浅再对中)
                              Sx : constant Long_Float := Long_Float'Max (Pitch, Span / Long_Float (Span_Steps));    --  沿合拢方向的起点间距
                              Sy : constant Long_Float := Long_Float'Max (Pitch, 0.5 * Pad_W);   --  横着的间距(分辨率:半个指肚宽)
                              Nx : constant Natural := Natural (Long_Float'Ceiling ((X_Hi - X_Lo + Span) / Sx)) + 1;
                              Ny : constant Natural := Natural (Long_Float'Ceiling ((Y_Hi - Y_Lo + Pad_W) / Sy)) + 1;
                           begin
                              for Gx in 0 .. Nx - 1 loop
                                 for Gy in 0 .. Ny - 1 loop
                                    declare
                                       X0 : V3 := Add (Com, Add (Scl (X_Lo - 0.5 * Span + Sx * Long_Float (Gx), J), Scl (Y_Lo - 0.5 * Pad_W + Sy * Long_Float (Gy), Kx)));
                                       Trav : Bytes.Floats;
                                       Hit : array (0 .. Np_Pads - 1) of Integer;
                                       Ok_Pose : Boolean := True;
                                       Why : Natural := 0;   --  1 = 落在料上,2 = 顶到手掌,3 = 合空,4 = 两边碰得不齐,5 = 旁边的东西挡着(落在它上面 / 合的路上先碰到它 / 它顶到手掌)
                                       M : V3;
                                       Pre : Long_Float := 0.0;   --  下去之前每一块先合多少
                                    begin
                                       St.Poses := St.Poses + 1;
                                       for Pass in 1 .. 3 loop   --  对中最多三遍(次数)
                                          --  指尖落到面上:最低那块的尖正好到它躺的面
                                          declare
                                             H0 : constant Long_Float := Dot (Sub (X0, Support_P), U) + Low;
                                             Au : constant Long_Float := -Dot (A, U);
                                          begin
                                             M := Add (X0, Scl (H0 / Au, A));
                                          end;
                                          Trav.Clear;
                                          Ok_Pose := True; Why := 0;
                                          --  每一块从张到头的地方沿它的路合过去,最先碰到的料
                                          for Ip in 0 .. Np_Pads - 1 loop
                                             declare
                                                S0 : constant V3 := Add (M, Off (Ip));
                                                Best : Long_Float := Long_Float'Last;
                                                Bi : Integer := -1;
                                                Hw : constant Long_Float := 0.5 * H.Pads (Ip).Width;
                                             begin
                                                for I in 0 .. Np - 1 loop
                                                   declare
                                                      Q : constant V3 := Sub (P (I), S0);
                                                      Sd : constant Long_Float := Dot (Q, Dw (Ip));
                                                   begin
                                                      if Sd > 0.0 and then Sd <= H.Pads (Ip).Travel and then Sd < Best and then abs Dot (Q, Ww (Ip)) <= Hw
                                                        and then -Dot (Q, A) >= -Pitch and then -Dot (Q, A) <= H.Reach_In
                                                      then
                                                         Best := Sd; Bi := I;
                                                      end if;
                                                   end;
                                                end loop;
                                                Hit (Ip) := Bi;
                                                Trav.Append (Best);
                                             end;
                                          end loop;
                                          declare
                                             N_Hit : Natural := 0;
                                             Mean : Long_Float := 0.0;
                                             Shift : V3 := [others => 0.0];
                                             Spread : Long_Float := 0.0;
                                          begin
                                             for Ip in 0 .. Np_Pads - 1 loop
                                                if Hit (Ip) >= 0 then
                                                   N_Hit := N_Hit + 1;
                                                   Mean := Mean + Trav (Ip);
                                                end if;
                                             end loop;
                                             if N_Hit < 2 then
                                                Ok_Pose := False; Why := 3;   --  合空(碰到料的不到两块)
                                                exit;
                                             end if;
                                             Mean := Mean / Long_Float (N_Hit);
                                             for Ip in 0 .. Np_Pads - 1 loop
                                                if Hit (Ip) >= 0 then
                                                   Spread := Long_Float'Max (Spread, abs (Trav (Ip) - Mean));
                                                   --  碰得早的那块往后让:整只手顺着它合拢的方向挪 (t − 平均)(几块的平均;两块相向时正好对中)
                                                   declare
                                                      Dp : constant V3 := Sub (Dw (Ip), Scl (Dot (Dw (Ip), A), A));
                                                   begin
                                                      Shift := Add (Shift, Scl ((Trav (Ip) - Mean) / Long_Float (N_Hit), Dp));
                                                   end;
                                                end if;
                                             end loop;
                                             exit when Spread <= Pitch;
                                             if Pass = 3 then
                                                Ok_Pose := False; Why := 4;
                                             else
                                                X0 := Add (X0, Shift);
                                             end if;
                                          end;
                                       end loop;
                                       --  下去之前先合一点(一个自由度的手:几块一起合):合到每一块离料还剩"手落位的误差 + 点的误差 + 一个采样间距"为止 —— 张到头下去,
                                       --  手指会落在旁边的东西上(09-29 焊点:8 cm 的张口,条旁边 1 cm 外的方块正好在手指底下);然后在合过的地方核落手:
                                       --  手指(碰东西的那一面到它后面一个手指厚,横着多留一个采样间距)从上往下那一溜有料 ⇒ 落在料上;有旁边的东西、
                                       --  或者合过去碰到料之前先碰到旁边的东西 ⇒ 旁边的东西挡着;东西 / 旁边的东西伸进手里超过指尖到手掌那么深 ⇒ 顶到手掌
                                       if Ok_Pose then
                                          declare
                                             Clear : constant Long_Float := H.Pos_Err + Sigma + Pitch;
                                             Min_T : Long_Float := Long_Float'Last;
                                          begin
                                             for Ip in 0 .. Np_Pads - 1 loop
                                                if Hit (Ip) >= 0 then
                                                   Min_T := Long_Float'Min (Min_T, Trav (Ip));
                                                end if;
                                             end loop;
                                             Pre := Long_Float'Max (0.0, Min_T - Clear);
                                          end;
                                          for Ip in 0 .. Np_Pads - 1 loop
                                             declare
                                                S0 : constant V3 := Add (M, Off (Ip));
                                                Hw : constant Long_Float := 0.5 * H.Pads (Ip).Width;
                                                Th : constant Long_Float := H.Pads (Ip).Thick;
                                                Lim : constant Long_Float := (if Hit (Ip) >= 0 then Trav (Ip) else H.Pads (Ip).Travel);
                                             begin
                                                for I in 0 .. Np - 1 loop
                                                   declare
                                                      Q : constant V3 := Sub (P (I), S0);
                                                      Sd : constant Long_Float := Dot (Q, Dw (Ip));
                                                   begin
                                                      if abs Dot (Q, Ww (Ip)) <= Hw + Pitch and then -Dot (Q, A) >= -Pitch and then Sd >= Pre - Th - Pitch and then Sd <= Pre + Pitch then
                                                         Ok_Pose := False; Why := 1;
                                                         exit;
                                                      end if;
                                                   end;
                                                end loop;
                                                exit when not Ok_Pose;
                                                for Q0 of O loop
                                                   declare
                                                      Q : constant V3 := Sub (Q0, S0);
                                                      Sd : constant Long_Float := Dot (Q, Dw (Ip));
                                                      Sh : constant Long_Float := -Dot (Q, A);
                                                   begin
                                                      if abs Dot (Q, Ww (Ip)) <= Hw + Po and then Sh >= -Po
                                                        and then ((Sd >= Pre - Th - Po and then Sd <= Pre + Po) or else (Sh <= H.Reach_In and then Sd > Pre and then Sd <= Lim))
                                                      then
                                                         Ok_Pose := False; Why := 5;
                                                         exit;
                                                      end if;
                                                   end;
                                                end loop;
                                                exit when not Ok_Pose;
                                             end;
                                          end loop;
                                       end if;
                                       if Ok_Pose then
                                          for I in 0 .. Np - 1 loop
                                             declare
                                                Q : constant V3 := Sub (P (I), M);
                                             begin
                                                if -Dot (Q, A) > H.Reach_In and then abs Dot (Q, J) <= 0.5 * Span and then abs Dot (Q, Kx) <= 0.5 * Pad_W then
                                                   Ok_Pose := False; Why := 2;
                                                   exit;
                                                end if;
                                             end;
                                          end loop;
                                       end if;
                                       if Ok_Pose then
                                          for Q0 of O loop
                                             declare
                                                Q : constant V3 := Sub (Q0, M);
                                             begin
                                                if -Dot (Q, A) > H.Reach_In and then abs Dot (Q, J) <= 0.5 * Span + Po and then abs Dot (Q, Kx) <= 0.5 * Pad_W + Po then
                                                   Ok_Pose := False; Why := 5;
                                                   exit;
                                                end if;
                                             end;
                                          end loop;
                                       end if;
                                       if not Ok_Pose then
                                          case Why is
                                             when 1 => St.Landed_On := St.Landed_On + 1;
                                             when 2 => St.Palm_Hit := St.Palm_Hit + 1;
                                             when 3 => St.Air := St.Air + 1;
                                             when 5 => St.Blocked := St.Blocked + 1;
                                             when others => St.Unbalanced := St.Unbalanced + 1;
                                          end case;
                                       else
                                          declare
                                             --  取整到半个采样间距(去重)
                                             Kq : constant Key := (A => Ai, Phi => Ri,
                                                                   X => Integer (Long_Float'Floor (M (0) / (0.5 * Pitch))),
                                                                   Y => Integer (Long_Float'Floor (M (1) / (0.5 * Pitch))),
                                                                   Z => Integer (Long_Float'Floor (M (2) / (0.5 * Pitch))));
                                          begin
                                             if not Seen.Contains (Kq) then
                                                Seen.Insert (Kq);
                                                declare
                                                   Cd : Candidate;
                                                   Mid_C : V3 := [others => 0.0];
                                                   Nh : Natural := 0;
                                                   function First_Sh (Ip : Natural; S0 : V3) return Long_Float is
                                                      Hs : Bytes.Floats;
                                                      Hw0 : constant Long_Float := 0.5 * H.Pads (Ip).Width;
                                                   begin
                                                      for I in 0 .. Np - 1 loop
                                                         declare
                                                            Q : constant V3 := Sub (P (I), S0);
                                                            Sd : constant Long_Float := Dot (Q, Dw (Ip));
                                                            Sh : constant Long_Float := -Dot (Q, A);
                                                         begin
                                                            if abs Dot (Q, Ww (Ip)) <= Hw0 and then Sh >= -Pitch and then Sh <= H.Reach_In and then Sd > 0.0 and then Sd <= Trav (Ip) + 0.5 * Pitch then
                                                               Hs.Append (Sh);
                                                            end if;
                                                         end;
                                                      end loop;
                                                      if Hs.Is_Empty then
                                                         return -Dot (Sub (P (Hit (Ip)), S0), A);
                                                      end if;
                                                      declare
                                                         package Fs is new Bytes.F64_Vectors.Generic_Sorting;
                                                      begin
                                                         Fs.Sort (Hs);
                                                      end;
                                                      return Hs (Natural (Hs.Length) / 2);
                                                   end First_Sh;
                                                begin
                                                   Cd.R := R;
                                                   Cd.T := Sub (M, Geom.Ap (R, Mid_H));
                                                   Cd.Approach := A;
                                                   Cd.Travel := Trav;
                                                   Cd.Pre := Pre;
                                                   for Ip in 0 .. Np_Pads - 1 loop
                                                      if Hit (Ip) >= 0 then
                                                         declare
                                                            S0 : constant V3 := Add (M, Off (Ip));
                                                            Hw : constant Long_Float := 0.5 * H.Pads (Ip).Width;
                                                            Front : V3_Vectors.Vector;
                                                            Nn : V3;
                                                            W_Lo, W_Hi, H_Lo, H_Hi : Long_Float := 0.0;
                                                            First : Boolean := True;
                                                            T : Hd.Touch;
                                                            --  碰到的那一片沿手指在哪儿:最先碰到的那些点(离最近处不到半个采样间距)沿手指的中位(不是随便挑的那一个最先碰到的点:
                                                            --  两块手指各挑一个,高低不一样,两处接触的连线就斜了 —— 09-29 焊点:平条两面正对却说要 0.14 的摩擦)
                                                            Sh_Hit : constant Long_Float := First_Sh (Ip, S0);
                                                         begin
                                                            --  碰到的那一片:这一块的扫掠框里、离最先碰到的那一点不到一个采样间距深、沿手指离它不到半个指肚宽的点
                                                            --  (指肚沿手指多长没量 ⇒ 按一个指肚宽算,方的一块)
                                                            for I in 0 .. Np - 1 loop
                                                               declare
                                                                  Q : constant V3 := Sub (P (I), S0);
                                                                  Sd : constant Long_Float := Dot (Q, Dw (Ip));
                                                                  Sw : constant Long_Float := Dot (Q, Ww (Ip));
                                                                  Sh : constant Long_Float := -Dot (Q, A);
                                                               begin
                                                                  if abs Sw <= Hw and then Sh >= -Pitch and then Sh <= H.Reach_In and then abs (Sh - Sh_Hit) <= Hw
                                                                    and then Sd >= Trav (Ip) and then Sd <= Trav (Ip) + Pitch
                                                                  then
                                                                     Front.Append (P (I));
                                                                     if First then
                                                                        W_Lo := Sw; W_Hi := Sw; H_Lo := Sh; H_Hi := Sh; First := False;
                                                                     else
                                                                        W_Lo := Long_Float'Min (W_Lo, Sw); W_Hi := Long_Float'Max (W_Hi, Sw);
                                                                        H_Lo := Long_Float'Min (H_Lo, Sh); H_Hi := Long_Float'Max (H_Hi, Sh);
                                                                     end if;
                                                                  end if;
                                                               end;
                                                            end loop;
                                                            --  接触点 = 碰到的那一片的中心,按最先碰到的那个深度放(不是最先碰到的那一个采样点,也不缩进料里)
                                                            declare
                                                               Cen : V3 := [others => 0.0];
                                                            begin
                                                               for Q of Front loop
                                                                  Cen := Add (Cen, Scl (1.0 / Long_Float (Front.Length), Q));
                                                               end loop;
                                                               if Front.Is_Empty then
                                                                  T.P := P (Hit (Ip));
                                                               else
                                                                  T.P := Sub (Cen, Scl (Dot (Sub (Cen, S0), Dw (Ip)) - Trav (Ip), Dw (Ip)));
                                                               end if;
                                                            end;
                                                            --  接触法向(09-29):平的指肚合到东西上,碰的是指肚面中间(面贴面,或东西的尖 / 凸棱 / 圆柱面顶在指肚面上)⇒ 力沿指肚的朝向;
                                                            --  只有指肚的边擦在一面斜墙上(墙过了指肚的边还在往指肚这边来)⇒ 力沿那面墙的法向。
                                                            --  按水平的横向位置 x 看东西朝指肚这一面的深度 d(Kh = 指肚宽的方向的水平分量;侧壁是竖的):最先碰到的那一点 (x0, d0);
                                                            --  往 +Kh 那边(横着离它半格到两格半,一格一个采样间距)所有点里,(d − d0) / (x − x0) 最小的那个 = 从它往那边的表面最往指肚这边斜的那一条
                                                            --  (下凸包的边:藏在墙后面的点在它上面,永远挑不中);−Kh 那边同理取最大。哪边的斜率说"还在变近"(斜过分辨率)⇒ 那边是指肚的边、
                                                            --  擦着的是一面斜墙,墙的斜率就是它;两边都不再变近 ⇒ 碰在指肚面中间 ⇒ 指肚的朝向。看的范围比指肚宽出三个采样间距(只为看墙过了指肚的边往哪走)。
                                                            --  分辨率:斜率 = 深度误差(Set_Dl 里那个 max (点的误差, 半个采样间距))的一半 ÷ 一个采样间距 —— 比它缓的斜按指肚的朝向算,差出的角度在 Set_Dl 给的误差里。
                                                            --  (09-29 焊点一路查下来的:拿周围的点拟面会把顶面拟进来;按格取最近点,格里没有墙上的点时(指肚边上的窄格、斜 45° 的面采样稀)
                                                            --  最近的是藏在墙后面的顶面点,判成"两边都更远的尖",板的墙法向错成了指肚的朝向)
                                                            declare
                                                               Wh : constant V3 := Sub (Ww (Ip), Scl (Dot (Ww (Ip), U), U));
                                                               Okh : Boolean;
                                                               Kh0 : constant V3 := Unit (Wh, Okh);
                                                               Kh : constant V3 := (if Okh then Kh0 else Ww (Ip));
                                                               Ext : constant Long_Float := 3.0 * Pitch_In;
                                                               G_Tol : constant Long_Float := 0.5 * Long_Float'Max (Sigma, 0.5 * Pitch_In) / Pitch_In;
                                                               X0, D0 : Long_Float := 0.0;
                                                               Have0 : Boolean := False;
                                                               G_R : Long_Float := Long_Float'Last;    --  往 +Kh 那边最往指肚这边斜的斜率(dd/dx;没点 = Last)
                                                               G_L : Long_Float := Long_Float'First;   --  往 −Kh 那边(没点 = First)
                                                               --  这一块碰到的那一带:沿指肚宽在指肚里(Wide = 0)或多看几格,沿手指在碰到的那一片的高度里,在指肚前面
                                                               function In_Band (Q : V3; Wide : Long_Float) return Boolean is
                                                                 (abs Dot (Q, Ww (Ip)) <= Hw + Wide and then -Dot (Q, A) >= -Pitch_In and then -Dot (Q, A) <= H.Reach_In
                                                                  and then abs (-Dot (Q, A) - Sh_Hit) <= Hw and then Dot (Q, Dw (Ip)) > 0.0);
                                                            begin
                                                               for Q0 of Pts loop   --  原来那一份点(没稀疏过的),格宽按它的采样间距
                                                                  declare
                                                                     Q : constant V3 := Sub (Q0, S0);
                                                                  begin
                                                                     if In_Band (Q, 0.0) and then (not Have0 or else Dot (Q, Dw (Ip)) < D0) then
                                                                        X0 := Dot (Q, Kh); D0 := Dot (Q, Dw (Ip)); Have0 := True;
                                                                     end if;
                                                                  end;
                                                               end loop;
                                                               Nn := Dw (Ip);
                                                               if Have0 then
                                                                  for Q0 of Pts loop
                                                                     declare
                                                                        Q : constant V3 := Sub (Q0, S0);
                                                                     begin
                                                                        if In_Band (Q, Ext) then
                                                                           declare
                                                                              Dx : constant Long_Float := Dot (Q, Kh) - X0;
                                                                           begin
                                                                              --  横着离它半格到两格半(按采样间距的比例:太近的点斜率不稳,最近的一格里点稀时还够得着下一格)
                  if abs Dx >= 0.5 * Pitch_In and then abs Dx <= 2.5 * Pitch_In then
                                                                                 declare
                                                                                    G : constant Long_Float := (Dot (Q, Dw (Ip)) - D0) / Dx;
                                                                                 begin
                                                                                    if Dx > 0.0 then
                                                                                       G_R := Long_Float'Min (G_R, G);
                                                                                    else
                                                                                       G_L := Long_Float'Max (G_L, G);
                                                                                    end if;
                                                                                 end;
                                                                              end if;
                                                                           end;
                                                                        end if;
                                                                     end;
                                                                  end loop;
                                                                  declare
                                                                     Go_R : constant Boolean := G_R < -G_Tol;   --  往 +Kh 那边过了这一点还在变近(那边没点 ⇒ Last,不算)
                                                                     Go_L : constant Boolean := G_L > G_Tol;
                                                                     Slope : Long_Float := 0.0;   --  深度随横向位置的斜率(dd/dx);墙在水平面里的切向 = Kh + 斜率 · Dw ⇒ 法向 ∝ Dw − 斜率 · Kh
                                                                     Ou : Boolean;
                                                                  begin
                                                                     --  两边都还在变近 = 指肚卡在一道比它窄的槽里,两条边各擦一面、横着的分量对消 ⇒ 按指肚的朝向
                                                                     if Go_R and then not Go_L then
                                                                        Slope := G_R;
                                                                     elsif Go_L and then not Go_R then
                                                                        Slope := G_L;
                                                                     end if;
                                                                     Nn := Unit (Sub (Dw (Ip), Scl (Slope, Kh)), Ou);
                                                                     if not Ou then
                                                                        Nn := Dw (Ip);
                                                                     end if;
                                                                  end;
                                                               end if;
                                                            end;
                                                            T.N := Nn;
                                                            --  指肚沿手指多长没量 ⇒ 按一个指肚宽算(方的一块):碰到的那一片沿手指最多这么高
                                                            declare
                                                               Hh : constant Long_Float := 0.5 * Long_Float'Min (H_Hi - H_Lo, 2.0 * Hw);
                                                               Hw2 : constant Long_Float := 0.5 * (W_Hi - W_Lo);
                                                            begin
                                                               --  能拧的半径 = 碰到的那一片的半径(两个方向里小的那个),不超过半个指肚宽
                                                               T.Twist_R := Long_Float'Min (Hw, Long_Float'Min (Hw2, Hh));
                                                               Cd.Half_W.Append (Hw2);
                                                               Cd.Half_H.Append (Hh);
                                                            end;
                                                            Cd.Touches.Append (T);
                                                            Mid_C := Add (Mid_C, T.P);
                                                            Nh := Nh + 1;
                                                         end;
                                                      end if;
                                                   end loop;
                                                   Mid_C := Scl (1.0 / Long_Float (Nh), Mid_C);
                                                   for Ia in 0 .. Nh - 1 loop   --  两两之间最远的那一对
                                                      for Ib in Ia + 1 .. Nh - 1 loop
                                                         Cd.Width := Long_Float'Max (Cd.Width, Norm (Sub (Cd.Touches (Ib).P, Cd.Touches (Ia).P)));
                                                      end loop;
                                                   end loop;
                                                   declare
                                                      Dc : constant V3 := Sub (Mid_C, Com);
                                                   begin
                                                      Cd.Com_Off := Norm (Sub (Dc, Scl (Dot (Dc, U), U)));
                                                   end;
                                                   All_C.Append (Cd);
                                                end;
                                             end if;
                                          end;
                                       end if;
                                    end;
                                 end loop;
                              end loop;
                           end;
                        end;
                     end loop;
                  end if;
               end;
            end loop;
         end loop;
      end;
      --  物理(Contact.Wrench),每一组两样:
      --  ① 跟着手离开它躺的面最少要多大的摩擦 —— 合上以后抬一点验它跟不跟手,验的就是这个;这件东西的摩擦上下限按它记
      --     (Mu_Nom 按量到的法向、Mu_Worst 法向按误差取最坏);
      --  ② 要的动(没说 = ①那一种),连同它躺的那张面(托着它、有摩擦):手的法向力之和最少多少,摩擦按 Mu_Ref(它跟手、跟面按同一个数),
      --     法向按误差取最坏 —— 排序就按它。
      --  都是精确的,不截断:"做不做得到"对摩擦是单调的,要知道一组能不能比已有的更好,在门槛上问一次做不做得到就够(一次规划),
      --  做不到就跳过,做得到才细算;最坏的比名义的只大不小 ⇒ 按名义的排好一个个算最坏的,名义的已经不比挑出来的小就停
      declare
         Ld : constant Hd.Load := (F => U, C => Com, M => [others => 0.0]);
         Move : constant Twist := (if Want.Given then Want.Move else Slide (U));
         Sup : constant Hd.Surface := Hd.Footprint (Pts, Support_P, U, Pitch);
         Nc : constant Natural := Natural (All_C.Length);
         type Real_Arr is array (Natural range <>) of Long_Float;
         type Idx_Arr is array (Natural range <>) of Natural;
         Nom_Done, Mu_Done : array (0 .. Natural'Max (Nc, 1) - 1) of Boolean := [others => False];
         Mu_First : Long_Float := Long_Float'Last;
         --  法向在两个方向上各准到多少:点的误差(量的,不小于半个采样间距)除以碰到的那一片在那个方向上的半径(不小于一个采样间距)——
         --  绕手指的轴转(Ax = 0)看那一片沿指肚宽有多宽,绕指肚宽的轴转(Ax = 1)看它沿手指有多高;每个方向正反各转一次
         function Axis_Of (Cd : Candidate; Ax : Natural) return V3 is
           (if Ax = 0 then Cd.Approach else Unit (Cross (Cd.Approach, Sub (Cd.Touches (1).P, Cd.Touches (0).P)), Oku));
         function Tilt_Set (Cd : Candidate; Ax : Natural; Sign : Long_Float) return Hd.Touch_Vectors.Vector is
            Dl : Bytes.Floats;
            Dummy : constant Boolean := Set_Dl (Cd, Ax, Pitch, Sigma, Dl);
         begin
            for I in 0 .. Natural (Dl.Length) - 1 loop
               Dl.Replace_Element (I, Sign * Dl (I));
            end loop;
            return Tilted (Cd.Touches, Dl, Axis_Of (Cd, Ax));
         end Tilt_Set;
         --  ①的名义 / 最坏(算过就不再算)
         procedure Fill_Mu_Nom (I : Natural) is
            Cd : Candidate := All_C (I);
         begin
            if not Nom_Done (I) then
               Cd.Mu_Nom := Hd.Mu_Need (Cd.Touches, Ld);
               All_C.Replace_Element (I, Cd);
               Nom_Done (I) := True;
            end if;
         end Fill_Mu_Nom;
         procedure Fill_Mu_Worst (I : Natural) is
            Cd : Candidate;
            Worst : Long_Float;
         begin
            if Mu_Done (I) then
               return;
            end if;
            Fill_Mu_Nom (I);
            Cd := All_C (I);
            Worst := Cd.Mu_Nom;
            for Ax in 0 .. 1 loop
               Worst := Long_Float'Max (Worst, Long_Float'Max (Hd.Mu_Need (Tilt_Set (Cd, Ax, 1.0), Ld), Hd.Mu_Need (Tilt_Set (Cd, Ax, -1.0), Ld)));
            end loop;
            Cd.Mu_Worst := Worst;
            All_C.Replace_Element (I, Cd);
            Mu_Done (I) := True;
         end Fill_Mu_Worst;
         --  ①的最坏(要用时才算)
         function Mu_Worst_Of (I : Natural) return Long_Float is
         begin
            Fill_Mu_Worst (I);
            return All_C (I).Mu_Worst;
         end Mu_Worst_Of;
         --  ①在摩擦 Mu 下,名义的和四个倾斜的都做得到吗(= 最坏的不比 Mu 大)
         function All_Tilts_At (Cd : Candidate; Mu : Long_Float) return Boolean is
         begin
            for Ax in 0 .. 1 loop
               if Hd.Squeeze (Tilt_Set (Cd, Ax, 1.0), Ld, Mu) = Hd.No_Way or else Hd.Squeeze (Tilt_Set (Cd, Ax, -1.0), Ld, Mu) = Hd.No_Way then
                  return False;
               end if;
            end loop;
            return True;
         end All_Tilts_At;
         --  ②的最坏
         function Need_Worst (Cd : Candidate) return Long_Float is
            Why : Hd.Why_Kind;
            Worst : Long_Float := Hd.Need (Cd.Touches, Com, U, Sup, Move, St.Mu_Ref, St.Mu_Ref, Why);
         begin
            for Ax in 0 .. 1 loop
               Worst := Long_Float'Max (Worst, Long_Float'Max (Hd.Need (Tilt_Set (Cd, Ax, 1.0), Com, U, Sup, Move, St.Mu_Ref, St.Mu_Ref, Why),
                                                              Hd.Need (Tilt_Set (Cd, Ax, -1.0), Com, U, Sup, Move, St.Mu_Ref, St.Mu_Ref, Why)));
            end loop;
            return Worst;
         end Need_Worst;
         --  按键排下标(键一样按下标,稳定)
         procedure Sort_By (Key : Real_Arr; Ix : in out Idx_Arr; N : Natural) is
            function Lt (A, B : Natural) return Boolean is (Key (A) < Key (B) or else (Key (A) = Key (B) and then A < B));
            procedure Srt is new Ada.Containers.Generic_Array_Sort (Natural, Natural, Idx_Arr, Lt);
         begin
            if N > 0 then
               Srt (Ix (0 .. N - 1));
            end if;
         end Sort_By;
      begin
         St.Kept := Nc;
         --  这件东西的摩擦没量过时的先验:这一批里①最坏刚好够、再留它自己那份误差(2 × 最坏 − 名义),最小的那一组要的。
         --  一组要把它压低:名义的得在它之下做得到,四个倾斜的得在 (它 + 名义)/2 之下做得到 —— 先问这两句,都过了才细算
         for I in 0 .. Nc - 1 loop
            declare
               Cd : constant Candidate := All_C (I);
            begin
               if Mu_First = Long_Float'Last or else Hd.Squeeze (Cd.Touches, Ld, Mu_First) < Hd.No_Way then
                  Fill_Mu_Nom (I);
                  if All_C (I).Mu_Nom < Hd.No_Way
                    and then (Mu_First = Long_Float'Last or else All_Tilts_At (All_C (I), 0.5 * (Mu_First + All_C (I).Mu_Nom)))
                  then
                     Fill_Mu_Worst (I);
                     if All_C (I).Mu_Worst < Hd.No_Way then
                        Mu_First := Long_Float'Min (Mu_First, All_C (I).Mu_Worst + (All_C (I).Mu_Worst - All_C (I).Mu_Nom));
                     end if;
                  end if;
               end if;
            end;
         end loop;
         St.Mu_Ref := Long_Float'Max (Mu_Lb, (if Mu_First < Long_Float'Last then Mu_First else 0.0));
         --  量到的上限(它没跟上的那一组最坏要的,见 Act 的 Note_Grip_Mu):先验不许比它大 —— 不假设它比量过的还滑得少
         if Mu_Ub < Long_Float'Last then
            St.Mu_Ref := Long_Float'Min (St.Mu_Ref, Mu_Ub);
         end if;
         --  ②按名义的法向先算一遍;①在这件东西量到的摩擦上限下做不到的不要(它以前没跟上的那一组要的它给不起);
         --  要的动往它躺的面里去 ⇒ 哪一组都做不到
         declare
            Pool : Idx_Arr (0 .. Natural'Max (Nc, 1) - 1);
            Np2 : Natural := 0;
            Nn : Real_Arr (0 .. Natural'Max (Nc, 1) - 1) := [others => Hd.No_Way];
            Worst_Of : Real_Arr (0 .. Natural'Max (Nc, 1) - 1) := [others => Hd.No_Way];
            Reach_Of : array (0 .. Natural'Max (Nc, 1) - 1) of Integer := [others => -1];   --  -1 = 还没问,0 = 够不着,1 = 够得着
            Ev : Idx_Arr (0 .. Natural'Max (Nc, 1) - 1) := [others => 0];   --  算过最坏的,按 (最坏的力, ①的最坏摩擦) 排好
            Ne : Natural := 0;
            Next : Natural := 0;
            --  排在前面:最坏的力小;一样大时①的最坏摩擦小(打平了才补算它)
            function Before (A, B : Natural) return Boolean is
            begin
               if Worst_Of (A) /= Worst_Of (B) then
                  return Worst_Of (A) < Worst_Of (B);
               end if;
               Fill_Mu_Worst (A); Fill_Mu_Worst (B);
               return All_C (A).Mu_Worst < All_C (B).Mu_Worst;
            end Before;
         begin
            for I in 0 .. Nc - 1 loop
               --  这件东西量到的摩擦上限 Ub(它没跟上的那一组法向取最坏时要的):①法向取最坏时要的不比 Ub 小的不要 ——
               --  在 Ub 那儿最坏的都做不到 ⇒ 要得比 Ub 还多;做得到再算准确的最坏(没跟上的那一组正好等于 Ub,严格地不要它)
               if Mu_Ub < Long_Float'Last
                 and then (Hd.Squeeze (All_C (I).Touches, Ld, Mu_Ub) = Hd.No_Way or else not All_Tilts_At (All_C (I), Mu_Ub))
               then
                  St.Over_Ub := St.Over_Ub + 1;
               elsif Mu_Ub < Long_Float'Last and then Mu_Worst_Of (I) >= Mu_Ub then
                  St.Over_Ub := St.Over_Ub + 1;
               else
                  declare
                     Why : Hd.Why_Kind;
                     use type Hd.Why_Kind;
                  begin
                     Nn (I) := Hd.Need (All_C (I).Touches, Com, U, Sup, Move, St.Mu_Ref, St.Mu_Ref, Why);
                     if Why = Hd.Surface_In_Way then
                        St.In_Way := True;
                        return;
                     end if;
                     if Nn (I) < Hd.No_Way then
                        Pool (Np2) := I; Np2 := Np2 + 1;
                     else
                        St.No_Force := St.No_Force + 1;
                     end if;
                  end;
               end if;
            end loop;
            Sort_By (Nn, Pool, Np2);
            loop
               declare
                  Bound : constant Long_Float := (if Next < Np2 then Nn (Pool (Next)) else Long_Float'Last);
                  Got : Natural := 0;
               begin
                  --  按排好的看:最坏的力比还没算的那些的名义还小的,够得着就收;收够了(或者都算完了)就停
                  Found.Clear;
                  for E in 0 .. Ne - 1 loop
                     declare
                        I : constant Natural := Ev (E);
                        Cd : constant Candidate := All_C (I);
                     begin
                        exit when Got >= Want_K or else (Worst_Of (I) >= Bound and then Next < Np2);
                        if Worst_Of (I) < Hd.No_Way then
                           if Reach_Of (I) < 0 then
                              --  够不够得着:下手那一刻和悬停那一刻的眼的位姿都要在量到的关节范围里解得出来
                              Reach_Of (I) := (if Reach (Cd.R, Cd.T) and then Reach (Cd.R, Sub (Cd.T, Scl (Standoff, Cd.Approach))) then 1 else 0);
                              if Reach_Of (I) = 0 then
                                 St.Unreachable := St.Unreachable + 1;
                              end if;
                           end if;
                           if Reach_Of (I) = 1 then
                              Found.Append (Cd);
                              Got := Got + 1;
                           end if;
                        end if;
                     end;
                  end loop;
                  exit when Got >= Want_K or else Next >= Np2;
                  --  再算一组的最坏,插进排好的里
                  declare
                     I : constant Natural := Pool (Next);
                     Cd : Candidate := All_C (I);
                     Pos : Natural := Ne;
                  begin
                     Worst_Of (I) := Need_Worst (Cd);
                     Cd.Squeeze := Worst_Of (I);
                     All_C.Replace_Element (I, Cd);
                     while Pos > 0 and then Before (I, Ev (Pos - 1)) loop
                        Ev (Pos) := Ev (Pos - 1);
                        Pos := Pos - 1;
                     end loop;
                     Ev (Pos) := I;
                     Ne := Ne + 1;
                     Next := Next + 1;
                  end;
               end;
            end loop;
            --  挑出来的每一组:①的名义和最坏都给全(合上以后验它跟不跟手、记摩擦就按它们)
            Found.Clear;
            declare
               Got : Natural := 0;
            begin
               for E in 0 .. Ne - 1 loop
                  exit when Got >= Want_K;
                  declare
                     I : constant Natural := Ev (E);
                  begin
                     if Worst_Of (I) < Hd.No_Way and then Reach_Of (I) = 1 then
                        Fill_Mu_Worst (I);
                        Found.Append (All_C (I));
                        Got := Got + 1;
                     end if;
                  end;
               end loop;
            end;
         end;
      end;
   end Plan;

   procedure Note_Hold (Lb, Ub : in out Long_Float; Mu_Nom, Mu_Worst : Long_Float; Came : Boolean) is
   begin
      if Came then
         Lb := Long_Float'Max (Lb, Mu_Nom);
      else
         Ub := Long_Float'Min (Ub, Mu_Worst);
      end if;
   end Note_Hold;

end Contact.Search;
