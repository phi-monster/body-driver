with Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Deallocation;
with Driver.Stats;

package body Driver.Robot.Hand.Hull is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;

   type Luma_Access is access Real_Array;
   type Luma_Table is array (Positive range <>) of Luma_Access;

   function Free_Run (Views : View_Array; Project : Projector; From, Toward : Vec3; Step : Real) return Run is
      Result : Run;
      Width  : constant Natural := (if Views'Length > 0 then Driver.Images.Width (Views (Views'First).Picture) else 0);
      Height : constant Natural := (if Views'Length > 0 then Driver.Images.Height (Views (Views'First).Picture) else 0);
   begin
      if Views'Length < 2 then
         Result.Why := Ada.Strings.Unbounded.To_Unbounded_String
           ("fewer than two pictures of the hand: a way into it is seen past by two or more");
         return Result;
      end if;
      for V of Views loop
         if Driver.Images.Width (V.Picture) /= Width or else Driver.Images.Height (V.Picture) /= Height
           or else Width = 0 or else Height = 0
         then
            Result.Why := Ada.Strings.Unbounded.To_Unbounded_String ("the pictures are not all of one size");
            return Result;
         end if;
      end loop;
      declare
         Pixels   : constant Positive := Width * Height;
         Lumas    : Luma_Table (Views'Range);
         Backdrop : Real_Array (1 .. Pixels);
         Noise    : Real_Array (Views'Range);
         Has_Backdrop : array (1 .. Pixels) of Boolean := [others => False];
         function Near_Hand (K : Positive; P : Positive) return Boolean is
           (Sqrt ((Real ((P - 1) mod Width) + 0.5 - Views (K).Hand_At.U) ** 2
                  + (Real ((P - 1) / Width) + 0.5 - Views (K).Hand_At.V) ** 2) <= Views (K).Hand_Radius);
         Way      : constant Vec3 := Toward - From;
         Length   : constant Real := abs Way;
      begin
         for K in Views'Range loop
            Lumas (K) := new Real_Array (1 .. Pixels);
            Driver.Images.Luma (Views (K).Picture, Lumas (K).all);
         end loop;
         --  Each pixel's backdrop: its median over the pictures that do not have the hand near it.
         declare
            Column : Real_Array (Views'Range);
            Count  : Natural;
         begin
            for P in 1 .. Pixels loop
               Count := 0;
               for K in Views'Range loop
                  if not Near_Hand (K, P) then
                     Count := Count + 1;
                     Column (Views'First + Count - 1) := Lumas (K) (P);
                  end if;
               end loop;
               Has_Backdrop (P) := Count > 0;
               Backdrop (P) := (if Count > 0 then Driver.Stats.Median (Column (Views'First .. Views'First + Count - 1))
                                else 0.0);
            end loop;
         end;
         --  Each picture's noise against it, over its pixels clear of the hand.
         declare
            Differences : Real_Array (1 .. Pixels);
            Count       : Natural;
         begin
            for K in Views'Range loop
               Count := 0;
               for P in 1 .. Pixels loop
                  if Has_Backdrop (P) and then not Near_Hand (K, P) then
                     Count := Count + 1;
                     Differences (Count) := Lumas (K) (P) - Backdrop (P);
                  end if;
               end loop;
               Noise (K) := (if Count > 0 then Driver.Stats.Robust_Sigma (Differences (1 .. Count)) else 0.0);
            end loop;
         end;
         if Length = 0.0 then
            Result.Why := Ada.Strings.Unbounded.To_Unbounded_String ("the way in has no length");
         else
            declare
               Unit_Way : constant Vec3 := (1.0 / Length) * Way;
               Steps    : constant Natural := Natural (Real'Floor (Length / Step));
               Last_Free : Integer := -1;
            begin
               for S in 0 .. Steps loop
                  declare
                     Here : constant Vec3 := From + (Real (S) * Step) * Unit_Way;
                     Seen : Natural := 0;
                     Past : Natural := 0;
                  begin
                     for K in Views'Range loop
                        declare
                           At_Eye   : constant Vec3 := Views (K).Eye_From_Tool * Here;
                           Ahead    : constant Vec3 := Views (K).Eye_From_Tool * (Here + Step * Unit_Way);
                           U, V     : Real;
                           U2, V2   : Real;
                           Front    : Boolean;
                           Front2   : Boolean;
                        begin
                           Project (At_Eye, U, V, Front);
                           Project (Ahead, U2, V2, Front2);
                           if Front and then Front2 then
                              declare
                                 --  The pixels the point may land on: within its blur, and within half the step's
                                 --  footprint, so the steps leave no pixel between them untested.
                                 Radius : constant Real := Views (K).Blur + Sqrt ((U2 - U) ** 2 + (V2 - V) ** 2) / 2.0;
                                 Reach  : constant Natural := Natural (Real'Ceiling (Radius));
                                 C0     : constant Integer := Integer (Real'Floor (U));
                                 R0     : constant Integer := Integer (Real'Floor (V));
                                 --  A pixel some part of which lies within the radius of where the point lands.
                                 function Within (C, R : Integer) return Boolean is
                                   (Sqrt (Real'Max (0.0, Real'Max (Real (C) - U, U - Real (C + 1))) ** 2
                                          + Real'Max (0.0, Real'Max (Real (R) - V, V - Real (R + 1))) ** 2) <= Radius);
                              begin
                                 if C0 - Reach >= 0 and then C0 + Reach <= Width - 1
                                   and then R0 - Reach >= 0 and then R0 + Reach <= Height - 1
                                 then
                                    Seen := Seen + 1;
                                    declare
                                       Count : Natural := 0;
                                       Front_Of_Backdrop : Boolean := False;
                                    begin
                                       for R in R0 - Reach .. R0 + Reach loop
                                          for C in C0 - Reach .. C0 + Reach loop
                                             Count := Count + Boolean'Pos (Within (C, R));
                                          end loop;
                                       end loop;
                                       declare
                                          Gate : constant Driver.Uncertain.Gate :=
                                            Driver.Uncertain.Scalar_Gate (Tests => Positive'Max (1, Count));
                                       begin
                                          for R in R0 - Reach .. R0 + Reach loop
                                             for C in C0 - Reach .. C0 + Reach loop
                                                if Within (C, R)
                                                  and then (not Has_Backdrop (R * Width + C + 1)
                                                            or else Driver.Uncertain.Significant
                                                              (Gate, Lumas (K) (R * Width + C + 1)
                                                                     - Backdrop (R * Width + C + 1),
                                                               Noise (K)))
                                                then
                                                   Front_Of_Backdrop := True;
                                                end if;
                                             end loop;
                                          end loop;
                                       end;
                                       if not Front_Of_Backdrop then
                                          Past := Past + 1;
                                       end if;
                                    end;
                                 end if;
                              end;
                           end if;
                        end;
                     end loop;
                     Result.Samples := Result.Samples + 1;
                     Result.Seen := Seen;
                     Result.Past := Past;
                     exit when Past < 2;
                     Last_Free := S;
                  end;
               end loop;
               if Last_Free < 0 then
                  Result.Why := Ada.Strings.Unbounded.To_Unbounded_String
                    ("the middle of the tips is not seen past: it lands in" & Result.Seen'Image & " pictures, of which"
                     & Result.Past'Image & " see past it");
               elsif Last_Free = Steps then
                  Result.Why := Ada.Strings.Unbounded.To_Unbounded_String
                    ("every point of the way is seen past, to its end: the hand is not met on it");
                  Result.Length := Driver.Uncertain.Unknown;
               else
                  --  The hand begins between the last point seen past and the next: halfway, as a spread even over
                  --  the step.
                  Result.Length := (Value => (Real (Last_Free) + 0.5) * Step, Sigma => Step / Sqrt (12.0),
                                    Degrees_Of_Freedom => 0);
                  Result.Why := Ada.Strings.Unbounded.To_Unbounded_String
                    ("the hand is met where the point lands in" & Result.Seen'Image & " pictures, of which"
                     & Result.Past'Image & " see past it");
               end if;
            end;
         end if;
         for K in Lumas'Range loop
            declare
               procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Luma_Access);
            begin
               Free (Lumas (K));
            end;
         end loop;
      end;
      return Result;
   end Free_Run;

end Driver.Robot.Hand.Hull;
