with Ada.Containers.Ordered_Sets;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
package body Contact.Surface is

   procedure On_Plane (Rays : Geom.Sight_Vectors.Vector; P0, N : V3; Pts : in out V3_Vectors.Vector; Dropped : out Natural) is
      Ok : Boolean;
   begin
      Dropped := 0;
      for R of Rays loop
         declare
            H : constant V3 := Geom.Hit_Plane (R.O, R.D, P0, N, Ok);
         begin
            if Ok then
               Pts.Append (H);
            else
               Dropped := Dropped + 1;
            end if;
         end;
      end loop;
   end On_Plane;

   function Pair (A, B : Geom.Sight; Tol_M : Long_Float; Ok : out Boolean; Miss : out Long_Float) return V3 is
      W : constant V3 := [A.O (0) - B.O (0), A.O (1) - B.O (1), A.O (2) - B.O (2)];
      Aa : constant Long_Float := Dot (A.D, A.D);
      Bb : constant Long_Float := Dot (A.D, B.D);
      Cc : constant Long_Float := Dot (B.D, B.D);
      Dd : constant Long_Float := Dot (A.D, W);
      Ee : constant Long_Float := Dot (B.D, W);
      Den : constant Long_Float := Aa * Cc - Bb * Bb;
      S, T : Long_Float;
      Pa, Pb : V3;
   begin
      Ok := False;
      Miss := Long_Float'Last;
      if abs Den < 1.0e-12 then
         return [others => 0.0];   --  两条线平行 ⇒ 定不下来
      end if;
      S := (Bb * Ee - Cc * Dd) / Den;
      T := (Aa * Ee - Bb * Dd) / Den;
      if S <= 0.0 or else T <= 0.0 then
         Miss := 0.0;
         return [others => 0.0];   --  交在身后
      end if;
      Pa := [A.O (0) + A.D (0) * S, A.O (1) + A.D (1) * S, A.O (2) + A.D (2) * S];
      Pb := [B.O (0) + B.D (0) * T, B.O (1) + B.D (1) * T, B.O (2) + B.D (2) * T];
      Miss := Norm ([Pa (0) - Pb (0), Pa (1) - Pb (1), Pa (2) - Pb (2)]);
      if Miss > Tol_M then
         return [others => 0.0];
      end if;
      Ok := True;
      return [0.5 * (Pa (0) + Pb (0)), 0.5 * (Pa (1) + Pb (1)), 0.5 * (Pa (2) + Pb (2))];
   end Pair;

   procedure Extrude_To_Support (Pts : in out V3_Vectors.Vector; Support_Z, Step_M : Long_Float) is
      N : constant Natural := Natural (Pts.Length);
      Stp : constant Long_Float := Long_Float'Max (Step_M, 1.0e-4);
   begin
      for I in 0 .. N - 1 loop
         declare
            P : constant V3 := Pts (I);
            Z : Long_Float := P (2) - Stp;
         begin
            while Z > Support_Z loop
               Pts.Append (V3'([P (0), P (1), Z]));
               Z := Z - Stp;
            end loop;
         end;
      end loop;
   end Extrude_To_Support;

   procedure Walls_To_Support (Top : V3_Vectors.Vector; N, Support_P : V3; Pitch : Long_Float; Pts : out V3_Vectors.Vector) is
      Ok : Boolean;
      Nu : constant V3 := Unit (N, Ok);
      Ax : constant V3 := (if abs Nu (0) <= abs Nu (1) then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);   --  x、y 里离 N 更远的那根
      E1 : V3;
      E2 : V3;
      type Cell is record
         I, J : Integer;
      end record;
      function "<" (A, B : Cell) return Boolean is (A.I < B.I or else (A.I = B.I and then A.J < B.J));
      package Cell_Sets is new Ada.Containers.Ordered_Sets (Cell);
      Occ : Cell_Sets.Set;
      Side : constant Long_Float := 2.0 * Pitch;   --  格子边长两个采样间距(对角邻居在 √2 个以内,取 2 个;纯几何)
      function Cell_Of (P : V3) return Cell is
        (I => Integer (Long_Float'Floor (Dot (P, E1) / Side)), J => Integer (Long_Float'Floor (Dot (P, E2) / Side)));
   begin
      Pts := Top;
      if not Ok or else Pitch <= 0.0 then
         return;
      end if;
      declare
         C1 : constant V3 := Cross (Nu, Ax);
         Ok1 : Boolean;
      begin
         E1 := Unit (C1, Ok1);
         E2 := Cross (Nu, E1);
      end;
      for P of Top loop
         Occ.Include (Cell_Of (P));
      end loop;
      for P of Top loop
         declare
            C : constant Cell := Cell_Of (P);
            Edge : constant Boolean := not Occ.Contains ((C.I + 1, C.J)) or else not Occ.Contains ((C.I - 1, C.J))
              or else not Occ.Contains ((C.I, C.J + 1)) or else not Occ.Contains ((C.I, C.J - 1));
            H : constant Long_Float := Dot ([P (0) - Support_P (0), P (1) - Support_P (1), P (2) - Support_P (2)], Nu);
            K : Positive := 1;
         begin
            if Edge then
               while Long_Float (K) * Pitch < H - 0.5 * Pitch loop   --  最低一层离面至少半个间距(一半,纯数学:面本身不补,碰桌面归面管)
                  Pts.Append (V3'([P (0) - Long_Float (K) * Pitch * Nu (0), P (1) - Long_Float (K) * Pitch * Nu (1), P (2) - Long_Float (K) * Pitch * Nu (2)]));
                  K := K + 1;
               end loop;
            end if;
         end;
      end loop;
   end Walls_To_Support;

   procedure Merge (Into : in out V3_Vectors.Vector; More : V3_Vectors.Vector) is
   begin
      for P of More loop
         Into.Append (P);
      end loop;
   end Merge;

   --  确定性 RANSAC 的那一段搜(Drop_Support_Plane 和 Support_Plane 共用):Tol_M 内点最多的那张平面 n·x = D
   procedure Best_Plane (Pts : V3_Vectors.Vector; Tol_M : Long_Float; Best_N : out V3; Best_D : out Long_Float; Best_Cnt : out Natural) is
      N : constant Natural := Natural (Pts.Length);
      --  确定性地取若干三元组:约 17 个起点(采样次数),第二、第三个点各按 3 倍、7 倍步长跳
      Stp : constant Natural := Natural'Max (N / 17, 1);
      Ia : Natural := 0;
   begin
      Best_N := [0.0, 0.0, 1.0]; Best_D := 0.0; Best_Cnt := 0;
      while Ia < N loop
         declare
            Ib : Natural := Ia + Stp;
         begin
            while Ib < N loop
               declare
                  Ic : Natural := Ib + Stp;
               begin
                  while Ic < N loop
                     declare
                        P : constant V3 := Pts (Ia);
                        Q : constant V3 := Pts (Ib);
                        R : constant V3 := Pts (Ic);
                        U : constant V3 := [Q (0) - P (0), Q (1) - P (1), Q (2) - P (2)];
                        V : constant V3 := [R (0) - P (0), R (1) - P (1), R (2) - P (2)];
                        Ok : Boolean;
                        Nv : constant V3 := Unit (Cross (U, V), Ok);
                     begin
                        if Ok then
                           declare
                              D : constant Long_Float := Dot (Nv, P);
                              Cnt : Natural := 0;
                           begin
                              for T of Pts loop
                                 if abs (Dot (Nv, T) - D) <= Tol_M then
                                    Cnt := Cnt + 1;
                                 end if;
                              end loop;
                              if Cnt > Best_Cnt then
                                 Best_Cnt := Cnt;
                                 Best_N := Nv;
                                 Best_D := D;
                              end if;
                           end;
                        end if;
                     end;
                     Ic := Ic + 7 * Stp;
                  end loop;
               end;
               Ib := Ib + 3 * Stp;
            end loop;
         end;
         Ia := Ia + Stp;
      end loop;
   end Best_Plane;

   procedure Drop_Support_Plane (Pts : in out V3_Vectors.Vector; Tol_M : Long_Float; Normal : out V3; On_Plane_Count : out Natural) is
      Best_N : V3;
      Best_D : Long_Float;
      Best_Cnt : Natural;
   begin
      Normal := [0.0, 0.0, 1.0];
      On_Plane_Count := 0;
      if Natural (Pts.Length) < 16 then   --  点数太少拟不出一张平面(点数)
         return;
      end if;
      Best_Plane (Pts, Tol_M, Best_N, Best_D, Best_Cnt);
      declare
         Kept : V3_Vectors.Vector;
      begin
         for T of Pts loop
            if abs (Dot (Best_N, T) - Best_D) > Tol_M then
               Kept.Append (T);
            end if;
         end loop;
         Pts := Kept;
      end;
      Normal := Best_N;
      On_Plane_Count := Best_Cnt;
   end Drop_Support_Plane;

   procedure Support_Plane (Pts : V3_Vectors.Vector; Tol_M : Long_Float; Up : V3; Point, Normal : out V3; Inliers : out Natural; Rms : out Long_Float) is
      Nv : V3;
      D : Long_Float;
      Cnt : Natural;
   begin
      Point := [0.0, 0.0, 0.0]; Normal := Up; Inliers := 0; Rms := 0.0;
      if Natural (Pts.Length) < 16 then   --  同 Drop_Support_Plane 的点数下限(点数)
         return;
      end if;
      Best_Plane (Pts, Tol_M, Nv, D, Cnt);
      if Cnt < 3 then
         return;
      end if;
      --  拿内点精修两遍(次数):形心 + 散布矩阵最小特征值的方向(反幂迭代,每遍 30 次,次数),再按精修后的面重挑内点
      for Round in 1 .. 2 loop
         declare
            Cg : V3 := [others => 0.0];
            S : Geom.M3 := [others => [others => 0.0]];
            K : Natural := 0;
         begin
            for T of Pts loop
               if abs (Dot (Nv, T) - D) <= Tol_M then
                  for I in 0 .. 2 loop
                     Cg (I) := Cg (I) + T (I);
                  end loop;
                  K := K + 1;
               end if;
            end loop;
            exit when K < 3;
            for I in 0 .. 2 loop
               Cg (I) := Cg (I) / Long_Float (K);
            end loop;
            for T of Pts loop
               if abs (Dot (Nv, T) - D) <= Tol_M then
                  for I in 0 .. 2 loop
                     for J in 0 .. 2 loop
                        S (I, J) := S (I, J) + (T (I) - Cg (I)) * (T (J) - Cg (J));
                     end loop;
                  end loop;
               end if;
            end loop;
            for It in 1 .. 30 loop
               declare
                  X : constant V3 := Geom.Solve3 (S, Nv);
                  Ok : Boolean;
                  Xn : constant V3 := Unit (X, Ok);
               begin
                  exit when not Ok;   --  散布矩阵奇异(点正好在一张面上):当前法向就是答案
                  Nv := Xn;
               end;
            end loop;
            D := Dot (Nv, Cg);
            Point := Cg;
            Inliers := K;
         end;
      end loop;
      if Dot (Nv, Up) < 0.0 then
         Nv := [-Nv (0), -Nv (1), -Nv (2)];
         D := -D;
      end if;
      Normal := Nv;
      declare
         Sum : Long_Float := 0.0;
         K : Natural := 0;
      begin
         for T of Pts loop
            if abs (Dot (Nv, T) - D) <= Tol_M then
               Sum := Sum + (Dot (Nv, T) - D) ** 2;
               K := K + 1;
            end if;
         end loop;
         Inliers := K;
         Rms := (if K > 0 then Sqrt (Sum / Long_Float (K)) else 0.0);
      end;
   end Support_Plane;

end Contact.Surface;
