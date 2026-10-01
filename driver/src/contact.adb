with Ada.Numerics; use Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
package body Contact is

   function Nat_Img (N : Natural) return String is (Ada.Strings.Fixed.Trim (Natural'Image (N), Ada.Strings.Left));

   function Dot (A, B : V3) return Long_Float is (A (0) * B (0) + A (1) * B (1) + A (2) * B (2));

   function Cross (A, B : V3) return V3 is
     ([A (1) * B (2) - A (2) * B (1), A (2) * B (0) - A (0) * B (2), A (0) * B (1) - A (1) * B (0)]);

   function Unit (A : V3; Ok : out Boolean) return V3 is
      N : constant Long_Float := Norm (A);
   begin
      if N'Valid and then N > 1.0e-12 then
         Ok := True;
         return [A (0) / N, A (1) / N, A (2) / N];
      end if;
      Ok := False;
      return [others => 0.0];
   end Unit;

   function Is_Dir (A : V3) return Boolean is
      N : constant Long_Float := Norm (A);
   begin
      return N'Valid and then N > 1.0e-12;
   end Is_Dir;

   function Still (Pivot : V3) return Twist is (Lin => [others => 0.0], Ang => [others => 0.0], Pivot => Pivot);
   function Slide (Lin : V3) return Twist is (Lin => Lin, Ang => [others => 0.0], Pivot => [others => 0.0]);

   function Rotation (Axis : V3; Rad : Long_Float; Pivot : V3; Ok : out Boolean) return Twist is
      A : constant V3 := Unit (Axis, Ok);
   begin
      return (Lin => [others => 0.0], Ang => [A (0) * Rad, A (1) * Rad, A (2) * Rad], Pivot => Pivot);
   end Rotation;

   function Angle (T : Twist) return Long_Float is (Norm (T.Ang));
   function Moving (T : Twist) return Boolean is (Norm (T.Lin) > 1.0e-9 or else Angle (T) > 1.0e-9);

   function Apply (T : Twist; P : V3) return V3 is
      Th : constant Long_Float := Angle (T);
      R : V3 := P;
   begin
      if Th >= 1.0e-12 then
         declare
            K : constant V3 := [T.Ang (0) / Th, T.Ang (1) / Th, T.Ang (2) / Th];
            Q : constant V3 := [P (0) - T.Pivot (0), P (1) - T.Pivot (1), P (2) - T.Pivot (2)];
            C : constant Long_Float := Cos (Th);
            Sn : constant Long_Float := Sin (Th);
            Kq : constant V3 := Cross (K, Q);
            Kd : constant Long_Float := Dot (K, Q);
         begin
            for I in 0 .. 2 loop
               R (I) := T.Pivot (I) + Q (I) * C + Kq (I) * Sn + K (I) * Kd * (1.0 - C);
            end loop;
         end;
      end if;
      return [R (0) + T.Lin (0), R (1) + T.Lin (1), R (2) + T.Lin (2)];
   end Apply;

   function Img (G : Gap) return String is
   begin
      case G.Kind is
         when Fine => return "fine";
         when No_Points => return "NoPoints";
         when Bad_Normal => return "BadNormal(" & Nat_Img (G.Index) & ")";
         when Bad_Cone => return "BadCone(" & Nat_Img (G.Index) & ")";
         when Motion_Still => return "MotionStill";
         when No_Pivot => return "NoPivot";
         when Bad_Tolerance => return "BadTolerance(" & Nat_Img (G.Index) & ")";
      end case;
   end Img;

   function Check (S : Set; Must_Move : Boolean) return Gap is
   begin
      if S.Points.Is_Empty then
         return (No_Points, 0);
      end if;
      for I in 0 .. Natural (S.Points.Length) - 1 loop
         declare
            P : constant Point := S.Points (I);
         begin
            if not Is_Dir (P.Normal) then
               return (Bad_Normal, I);
            end if;
            if not Is_Dir (P.Allowed.Axis) or else not P.Allowed.Half_Angle'Valid
              or else P.Allowed.Half_Angle < 0.0 or else P.Allowed.Half_Angle > Pi
            then
               return (Bad_Cone, I);
            end if;
            if not P.Tol_M'Valid or else P.Tol_M <= 0.0 then
               return (Bad_Tolerance, I);
            end if;
         end;
      end loop;
      if Must_Move and then not Moving (S.Motion) then
         return (Motion_Still, 0);
      end if;
      if Angle (S.Motion) > 1.0e-9 then
         --  绕轴转必须说清绕哪一点。这里只查它是不是一个数;"填得对不对"是几何层的事
         for I in 0 .. 2 loop
            if not S.Motion.Pivot (I)'Valid then
               return (No_Pivot, 0);
            end if;
         end loop;
      end if;
      return (Fine, 0);
   end Check;


   --  ── 一串 · 并存 · 过渡 ──

   --  这个接触集里【手】那几个点在哪(世界接触不算 —— 手够不到桌子底下那条边)
   function Hand_At (S : Set) return V3_Vectors.Vector is
      V : V3_Vectors.Vector;
   begin
      for P of S.Points loop
         if P.By.Kind = Hand then
            V.Append (P.Pos);
         end if;
      end loop;
      return V;
   end Hand_At;

   function Moves_At (M : Move; Id : Natural) return Boolean is
      N : constant Node := M.Nodes (Id);
   begin
      case N.Kind is
         when One =>
            return Moving (N.S.Motion);
         when In_Order | Keep | Meanwhile =>
            for C of N.Items loop
               if Moves_At (M, C) then
                  return True;
               end if;
            end loop;
            return False;
         when Clear =>
            return False;   --  躲开是手在动,不是物体在动
      end case;
   end Moves_At;

   function Moves (M : Move) return Boolean is (Moves_At (M, M.Root));

   function Start_At (M : Move; Id : Natural) return V3_Vectors.Vector is
      N : constant Node := M.Nodes (Id);
   begin
      case N.Kind is
         when One =>
            return Hand_At (N.S);
         when In_Order | Keep | Meanwhile =>
            if N.Items.Is_Empty then
               return V3_Vectors.Empty_Vector;
            end if;
            return Start_At (M, N.Items.First_Element);
         when Clear =>
            return N.From;
      end case;
   end Start_At;

   function End_At (M : Move; Id : Natural) return V3_Vectors.Vector is
      N : constant Node := M.Nodes (Id);
   begin
      case N.Kind is
         when One =>
            declare
               V : V3_Vectors.Vector := Hand_At (N.S);
            begin
               for J in 0 .. Natural (V.Length) - 1 loop
                  declare
                     Pj : constant V3 := V (J);
                  begin
                     V.Replace_Element (J, Apply (N.S.Motion, Pj));
                  end;
               end loop;
               return V;
            end;
         when In_Order | Keep =>
            if N.Items.Is_Empty then
               return V3_Vectors.Empty_Vector;
            end if;
            return End_At (M, N.Items.Last_Element);
         when Clear =>
            return N.From;   --  躲完手在哪由执行层算;这一层只说"别碰那儿"
         when Meanwhile =>
            if N.Items.Is_Empty then
               return V3_Vectors.Empty_Vector;
            end if;
            return End_At (M, N.Items.First_Element);   --  并存:末了停在维持的那一段上
      end case;
   end End_At;

   function Start_Points (M : Move) return V3_Vectors.Vector is (Start_At (M, M.Root));
   function End_Points (M : Move) return V3_Vectors.Vector is (End_At (M, M.Root));

   function Path_Img (P : Nat_Vectors.Vector) return String is
      U : Unbounded_String;
   begin
      for I of P loop
         if Length (U) > 0 then
            Append (U, "/");
         end if;
         Append (U, Nat_Img (I));
      end loop;
      return "[" & To_String (U) & "]";
   end Path_Img;

   function Img (M : Many_Gap) return String is
   begin
      case M.Kind is
         when Fine => return "fine";
         when Inside => return "At" & Path_Img (M.Path) & " " & Img (M.G);
         when Empty => return "Empty" & Path_Img (M.Path);
         when Holder_Moves => return "HolderMoves" & Path_Img (M.Path);
         when Nothing_To_Pair_With => return "NothingToPairWith" & Path_Img (M.Path);
         when Keep_Breaks_Contact => return "KeepBreaksContact" & Path_Img (M.Path) & " seg " & Nat_Img (M.Seg) & " off" & Long_Float'Image (M.Off_M);
         when Keep_Changes_Point_Count => return "KeepChangesPointCount" & Path_Img (M.Path) & " seg " & Nat_Img (M.Seg);
         when No_Keep_Out => return "NoKeepOut" & Path_Img (M.Path);
         when Bad_Clearance => return "BadClearance" & Path_Img (M.Path);
      end case;
   end Img;

end Contact;
