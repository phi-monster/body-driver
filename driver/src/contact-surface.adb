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

   procedure Merge (Into : in out V3_Vectors.Vector; More : V3_Vectors.Vector) is
   begin
      for P of More loop
         Into.Append (P);
      end loop;
   end Merge;

   procedure Drop_Support_Plane (Pts : in out V3_Vectors.Vector; Tol_M : Long_Float; Normal : out V3; On_Plane_Count : out Natural) is
      N : constant Natural := Natural (Pts.Length);
      Best_Cnt : Natural := 0;
      Best_N : V3 := [0.0, 0.0, 1.0];
      Best_D : Long_Float := 0.0;
      --  确定性地取若干三元组:约 17 个起点(采样次数),第二、第三个点各按 3 倍、7 倍步长跳
      Stp : constant Natural := Natural'Max (N / 17, 1);
   begin
      Normal := [0.0, 0.0, 1.0];
      On_Plane_Count := 0;
      if N < 16 then   --  点数太少拟不出一张平面(点数)
         return;
      end if;
      declare
         Pa : array (0 .. N - 1) of V3;
         Ia : Natural := 0;
      begin
         for I in 0 .. N - 1 loop
            Pa (I) := Pts (I);
         end loop;
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
                           P : V3 renames Pa (Ia);
                           Q : V3 renames Pa (Ib);
                           R : V3 renames Pa (Ic);
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
                                 for T of Pa loop
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
         declare
            Kept : V3_Vectors.Vector;
         begin
            for T of Pa loop
               if abs (Dot (Best_N, T) - Best_D) > Tol_M then
                  Kept.Append (T);
               end if;
            end loop;
            Pts := Kept;
         end;
      end;
      Normal := Best_N;
      On_Plane_Count := Best_Cnt;
   end Drop_Support_Plane;

end Contact.Surface;
