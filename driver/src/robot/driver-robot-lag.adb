with Ada.Numerics.Long_Elementary_Functions;
with Driver.Robot.Channels;

package body Driver.Robot.Lag is

   use Ada.Numerics.Long_Elementary_Functions;
   use type Driver.Observations.Group_Id;

   type Flags is array (Positive range <>) of Boolean;

   --  A series over the beats of the stream (beat B stored at B + 1), with
   --  gaps where it is not available.
   type Series (Length : Natural) is record
      Value : Real_Array (1 .. Length) := [others => 0.0];
      Have  : Flags (1 .. Length) := [others => False];
   end record;

   --  Replaces the available values by their ranks among themselves, ties
   --  sharing the average rank.
   procedure Rank (S : in out Series) is
      N : Natural := 0;
   begin
      for B of S.Have loop
         if B then
            N := N + 1;
         end if;
      end loop;
      if N = 0 then
         return;
      end if;
      declare
         Index : array (1 .. N) of Positive;
         K     : Natural := 0;
         Ranks : Real_Array (1 .. N);

         procedure Sift (Start, Stop : Natural) is
            Root  : Natural := Start;
            Child : Natural;
            T     : Positive;
         begin
            loop
               Child := 2 * Root;
               exit when Child > Stop;
               if Child < Stop and then S.Value (Index (Child)) < S.Value (Index (Child + 1)) then
                  Child := Child + 1;
               end if;
               exit when not (S.Value (Index (Root)) < S.Value (Index (Child)));
               T := Index (Root);
               Index (Root) := Index (Child);
               Index (Child) := T;
               Root := Child;
            end loop;
         end Sift;
      begin
         for I in S.Have'Range loop
            if S.Have (I) then
               K := K + 1;
               Index (K) := I;
            end if;
         end loop;
         --  Heap sort of the indexes by value.
         for Start in reverse 1 .. N / 2 loop
            Sift (Start, N);
         end loop;
         for Stop in reverse 2 .. N loop
            declare
               T : constant Positive := Index (1);
            begin
               Index (1) := Index (Stop);
               Index (Stop) := T;
            end;
            Sift (1, Stop - 1);
         end loop;
         declare
            I : Positive := 1;
         begin
            while I <= N loop
               declare
                  J : Positive := I;
               begin
                  while J < N and then S.Value (Index (J + 1)) = S.Value (Index (I)) loop
                     J := J + 1;
                  end loop;
                  for L in I .. J loop
                     Ranks (L) := Real (I + J) / 2.0;
                  end loop;
                  I := J + 1;
               end;
            end loop;
         end;
         for L in 1 .. N loop
            S.Value (Index (L)) := Ranks (L);
         end loop;
      end;
   end Rank;

   --  The first difference of a series, available where both terms are.
   function Differenced (S : Series) return Series is
      D : Series (S.Length);
   begin
      for I in S.Value'First + 1 .. S.Value'Last loop
         if S.Have (I) and then S.Have (I - 1) then
            D.Value (I) := S.Value (I) - S.Value (I - 1);
            D.Have (I) := True;
         end if;
      end loop;
      return D;
   end Differenced;

   --  The z-score of the correlation of A (beat B) with R (beat B - Shift)
   --  over the beats both have; zero when fewer than three pairs or no spread.
   function Shifted_Z (A, R : Series; Shift : Integer) return Real is
      N : Natural := 0;
      Sa, Sr, Saa, Srr, Sar : Real := 0.0;
   begin
      for I in A.Value'Range loop
         declare
            J : constant Integer := I - Shift;
         begin
            if J in R.Value'Range and then A.Have (I) and then R.Have (J) then
               N := N + 1;
               Sa := Sa + A.Value (I);
               Sr := Sr + R.Value (J);
               Saa := Saa + A.Value (I) ** 2;
               Srr := Srr + R.Value (J) ** 2;
               Sar := Sar + A.Value (I) * R.Value (J);
            end if;
         end;
      end loop;
      --  A correlation needs three pairs to have a spread left over.
      if N < 3 then
         return 0.0;
      end if;
      declare
         Cov : constant Real := Sar - Sa * Sr / Real (N);
         Va  : constant Real := Saa - Sa * Sa / Real (N);
         Vr  : constant Real := Srr - Sr * Sr / Real (N);
      begin
         if Va <= 0.0 or else Vr <= 0.0 then
            return 0.0;
         end if;
         return Cov / Sqrt (Va * Vr) * Sqrt (Real (N - 1));
      end;
   end Shifted_Z;

   procedure Measure (M : in out Model) is
      Last : constant Integer := M.Beats - 1;
   begin
      M.Lags.Clear;
      M.Lag_Known.Clear;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         M.Lags.Append (0);
         M.Lag_Known.Append (False);
      end loop;
      if Last < 1 then
         return;
      end if;
      declare
         Speeds : array (M.Groups.First_Index .. M.Groups.Last_Index) of Series (M.Beats);
      begin
         for G in Speeds'Range loop
            if M.Groups (G).Commandable then
               for B in 1 .. Last loop
                  if Channels.Has_Reading (M, G, B) and then Channels.Has_Reading (M, G, B - 1) then
                     declare
                        Sum : Real := 0.0;
                     begin
                        for C in 1 .. M.Groups (G).Size loop
                           Sum := Sum + Channels.Change (M, G, B, C) ** 2;
                        end loop;
                        Speeds (G).Value (B + 1) := (if Channels.Pushed (M, G, B) then Sqrt (Sum) else 0.0);
                        Speeds (G).Have (B + 1) := True;
                     end;
                  end if;
               end loop;
               Speeds (G) := Differenced (Speeds (G));
               Rank (Speeds (G));
            end if;
         end loop;
         for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
            declare
               S      : Eye_Stream renames M.Eyes (E);
               N      : constant Natural := Cells (S.Grid);
               Motion : Series (M.Beats);
               Best   : Real := 0.0;
               Lag    : Integer := 0;
               Tests  : Natural := 0;
            begin
               if N > 0 then
                  for B in 1 .. Natural'Min (Last, Natural (S.Measured.Length) - 1) loop
                     if S.Measured (B) then
                        declare
                           Sum : Real := 0.0;
                        begin
                           for C in 0 .. N - 1 loop
                              Sum := Sum + Sqrt (S.Du (B * N + C) ** 2 + S.Dv (B * N + C) ** 2);
                           end loop;
                           Motion.Value (B + 1) := Sum / Real (N);
                           Motion.Have (B + 1) := True;
                        end;
                     end if;
                  end loop;
                  Motion := Differenced (Motion);
                  Rank (Motion);
                  for G in Speeds'Range loop
                     if M.Groups (G).Commandable then
                        for Shift in -Last .. Last loop
                           declare
                              Z : constant Real := Shifted_Z (Motion, Speeds (G), Shift);
                           begin
                              Tests := Tests + 1;
                              if Z > Best then
                                 Best := Z;
                                 Lag := Shift;
                              end if;
                           end;
                        end loop;
                     end if;
                  end loop;
                  --  The best of many shifts is one of a family: the lag is
                  --  measured only when it stands out of all of them.
                  if Tests > 0
                    and then Driver.Uncertain.Significant (Driver.Uncertain.Scalar_Gate (Tests => Tests), Best, 1.0)
                  then
                     M.Lags.Replace_Element (E, Lag);
                     M.Lag_Known.Replace_Element (E, True);
                  end if;
               end if;
            end;
         end loop;
      end;
   end Measure;

end Driver.Robot.Lag;
