separate (Geom)
function Meet (Rays : Sight_Vectors.Vector; Ok : out Boolean; Spread : out Long_Float) return V3 is
   A : M3 := [others => [others => 0.0]];
   B : V3 := [others => 0.0];
   P : V3 := [others => 0.0];
   Used : Natural := 0;   --  真有视线的几条(零向量 = 那个像素去不了畸变,没有视线:当成一条会把交点往它的起点拽)
begin
   Ok := False; Spread := 0.0;
   for R of Rays loop
      if Norm (R.D) > 0.0 then
         Used := Used + 1;
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               declare
                  Pm : constant Long_Float := (if I = J then 1.0 else 0.0) - R.D (I) * R.D (J);
               begin
                  A (I, J) := A (I, J) + Pm;
                  B (I) := B (I) + Pm * R.O (J);
               end;
            end loop;
         end loop;
      end if;
   end loop;
   if Used < 2 then   --  一条视线交不出点
      return P;
   end if;
   --  视线全平行时 A 退化(行列式为零),交点没有意义
   declare
      Det : constant Long_Float :=
        A (0, 0) * (A (1, 1) * A (2, 2) - A (1, 2) * A (2, 1))
        - A (0, 1) * (A (1, 0) * A (2, 2) - A (1, 2) * A (2, 0))
        + A (0, 2) * (A (1, 0) * A (2, 1) - A (1, 1) * A (2, 0));
   begin
      if abs Det < 1.0e-9 then
         return P;
      end if;
   end;
   P := Solve3 (A, B);
   for R of Rays loop
      if Norm (R.D) > 0.0 then
         declare
            W : constant V3 := [P (0) - R.O (0), P (1) - R.O (1), P (2) - R.O (2)];
            T : constant Long_Float := W (0) * R.D (0) + W (1) * R.D (1) + W (2) * R.D (2);
            Perp : constant V3 := [W (0) - T * R.D (0), W (1) - T * R.D (1), W (2) - T * R.D (2)];
         begin
            Spread := Long_Float'Max (Spread, Norm (Perp));
            if T <= 0.0 then
               return P;      --  交点在某只眼的背后 ⇒ 不是它,Ok 留 False
            end if;
         end;
      end if;
   end loop;
   Ok := True;
   return P;
end Meet;
