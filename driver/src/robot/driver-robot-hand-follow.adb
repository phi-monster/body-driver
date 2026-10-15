with Ada.Containers.Vectors;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Uncertain;

package body Driver.Robot.Hand.Follow is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use type Driver.Alignment.Verdict;

   package Lobes renames Driver.Robot.Hand.Lobes;

   Gate : constant Driver.Uncertain.Gate := Driver.Uncertain.Vector_Gate (2);

   function Tip_Step (Tips : Pixel_Array; K : Positive) return Real is
     (Sqrt ((Tips (K + 1).U - Tips (K).U) ** 2 + (Tips (K + 1).V - Tips (K).V) ** 2));

   function Widest_Step (Tips : Pixel_Array; Width, Height : Positive) return Natural is
      Widest : constant Real := Driver.Alignment.Finest_Radius (Width, Height);
   begin
      for K in Tips'First .. Tips'Last - 1 loop
         if Tip_Step (Tips, K) > Widest then
            return K;
         end if;
      end loop;
      return 0;
   end Widest_Step;

   --  The place found matches back to where it came from: the round trip's miss, against the two answers'
   --  covariances, is no longer than the gate of two dimensions allows.
   function Returns (From : Driver.Images.Pixel; Forth, Back : Driver.Alignment.Answer) return Boolean is
      Du  : constant Real := Back.To.U - From.U;
      Dv  : constant Real := Back.To.V - From.V;
      A   : constant Real := Forth.Cov.UU + Back.Cov.UU;
      B   : constant Real := Forth.Cov.UV + Back.Cov.UV;
      C   : constant Real := Forth.Cov.VV + Back.Cov.VV;
      Det : constant Real := A * C - B * B;
   begin
      return Det > 0.0
        and then not Driver.Uncertain.Significant
                       (Gate, Sqrt (Real'Max (0.0, (C * Du * Du - 2.0 * B * Du * Dv + A * Dv * Dv) / Det)), 1.0);
   end Returns;

   type Point is record
      Start : Driver.Images.Pixel;   --  at the open end
      Now   : Driver.Images.Pixel;   --  in the view reached
      Var   : Real := 0.0;           --  of one coordinate, gathered step by step
   end record;

   package Point_Vectors is new Ada.Containers.Vectors (Positive, Point);


   function Run (Views : Pyramid_Array; Tips : Pixel_Array; Lobe : Driver.Images.Mask) return Followed is
      Width  : constant Natural := Driver.Images.Width (Lobe);
      Height : constant Natural := Driver.Images.Height (Lobe);
      Result : Followed;
      Live   : Point_Vectors.Vector;
   begin
      if Width = 0 or else Height = 0 then
         Result.Why := To_Unbounded_String ("the lobe has no pixels at the open end");
         return Result;
      end if;
      declare
         Wide : constant Natural := Widest_Step (Tips, Width, Height);
      begin
         if Wide > 0 then
            Result.Why := To_Unbounded_String
              ("its tip moves" & Natural'Image (Natural (Tip_Step (Tips, Wide))) & " pixels between views"
               & Wide'Image & " and" & Natural'Image (Wide + 1) & ", farther than the matcher searches at its finest ("
               & Natural'Image (Natural (Driver.Alignment.Finest_Radius (Width, Height))) & " pixels): the views are too far"
               & " apart to follow");
            return Result;
         end if;
      end;
      --  The points asked: the lobe's pixels on the half patch's grid whose patch is more the lobe's than not.
      declare
         Half : constant Natural := Natural'Max (1, Driver.Alignment.Half_Patch (Width, Height));
         Row  : Natural := Half / 2;
      begin
         while Row < Height loop
            declare
               Column : Natural := Half / 2;
            begin
               while Column < Width loop
                  if Driver.Images.Contains (Lobe, Column, Row) then
                     declare
                        Ours, All_Of : Natural := 0;
                     begin
                        for R in Integer'Max (0, Row - Half) .. Integer'Min (Height - 1, Row + Half) loop
                           for C in Integer'Max (0, Column - Half) .. Integer'Min (Width - 1, Column + Half) loop
                              All_Of := All_Of + 1;
                              if Driver.Images.Contains (Lobe, C, R) then
                                 Ours := Ours + 1;
                              end if;
                           end loop;
                        end loop;
                        if 2 * Ours > All_Of then
                           declare
                              P : constant Driver.Images.Pixel := (U => Real (Column) + 0.5, V => Real (Row) + 0.5);
                           begin
                              Live.Append (Point'(Start => P, Now => P, Var => 0.0));
                           end;
                        end if;
                     end;
                  end if;
                  Column := Column + Half;
               end loop;
            end;
            Row := Row + Half;
         end loop;
      end;
      Result.Asked := Natural (Live.Length);
      Result.Fewest := Result.Asked;
      if Live.Is_Empty then
         Result.Why := To_Unbounded_String ("no point of the lobe has a patch that is mostly the lobe's");
         return Result;
      end if;
      for K in Views'First .. Views'Last - 1 loop
         declare
            --  The window of every point is as wide as the tip's move: no point of the lobe moves farther.
            S    : constant Real := Driver.Alignment.Sigma_Of_Window (Tip_Step (Tips, K));
            Cov  : constant Driver.Alignment.Place_Covariance := (UU => S * S, UV => 0.0, VV => S * S);
            Kept : Point_Vectors.Vector;
            type Reason_Counts is array (Driver.Alignment.Reason) of Natural;
            Missed     : Reason_Counts := [others => 0];   --  forth not found, by the matcher's reason
            Wide_Found : Natural := 0;                     --  found, but in a window wider than a patch
            Unreturned : Natural := 0;                     --  found, but not matched back to where it came from
         begin
            for P of Live loop
               declare
                  Forth : constant Driver.Alignment.Answer :=
                    Driver.Alignment.Align (Views (K), Views (K + 1), (From => P.Now, To => P.Now, Cov => Cov, others => <>),
                                            Widen => False);
               begin
                  if Forth.Verdict /= Driver.Alignment.Found then
                     Missed (Forth.Because) := Missed (Forth.Because) + 1;
                  elsif not Forth.Narrow then
                     Wide_Found := Wide_Found + 1;
                  else
                     declare
                        Back : constant Driver.Alignment.Answer :=
                          Driver.Alignment.Align (Views (K + 1), Views (K),
                                                  (From => Forth.To, To => P.Now, Cov => Cov, others => <>),
                                                  Widen => False);
                     begin
                        if Back.Verdict = Driver.Alignment.Found and then Returns (P.Now, Forth, Back) then
                           Kept.Append (Point'(Start => P.Start, Now => Forth.To,
                                               Var => P.Var + (Forth.Cov.UU + Forth.Cov.VV) / 2.0));
                        else
                           Unreturned := Unreturned + 1;
                        end if;
                     end;
                  end if;
               end;
            end loop;
            if Natural (Kept.Length) < Natural (Live.Length)
              and then (Result.Worst = 0
                        or else Natural (Live.Length) - Natural (Kept.Length) > Result.Lost_Count)
            then
               Result.Lost_Count := Natural (Live.Length) - Natural (Kept.Length);
               Result.Worst := K;
               Result.Lost := Null_Unbounded_String;
               for R in Missed'Range loop
                  if Missed (R) > 0 then
                     Append (Result.Lost, Missed (R)'Image & " " & Driver.Alignment.Reason'Image (R) & ",");
                  end if;
               end loop;
               if Wide_Found > 0 then
                  Append (Result.Lost, Wide_Found'Image & " found in a window wider than a patch,");
               end if;
               if Unreturned > 0 then
                  Append (Result.Lost, Unreturned'Image & " not matched back,");
               end if;
            end if;
            Result.Fewest := Natural'Min (Result.Fewest, Natural (Kept.Length));
            Live := Kept;
         end;
         exit when Live.Is_Empty;
      end loop;
      Result.Count := Natural (Live.Length);
      if Live.Is_Empty then
         Result.Why := To_Unbounded_String
           ("none of its" & Result.Asked'Image & " points was followed past the step from view" & Result.Worst'Image
            & " (lost there:" & To_String (Result.Lost) & ")");
         return Result;
      end if;
      declare
         Moves : Lobes.Move_Array (1 .. Result.Count);
         Vars  : Real_Array (1 .. Result.Count);
      begin
         for I in 1 .. Result.Count loop
            Moves (I) := (From => Live (I).Start, To => Live (I).Now);
            Vars (I) := Live (I).Var;
         end loop;
         Sort (Vars);
         Result.Moves := Lobes.Move_Holders.To_Holder (Moves);
         Result.Sigma := Sqrt ((Vars ((Vars'First + Vars'Last) / 2) + Vars ((Vars'First + Vars'Last + 1) / 2)) / 2.0);
      end;
      return Result;
   end Run;

end Driver.Robot.Hand.Follow;
