with Driver.Conventions;

package body Driver.Robot.Hand.Lowering is

   use Driver.Numerics.Arrays;

   function Down (Pose : Rigid; Point, Into : Vec3) return Real is (Into * (Pose * Point));
   --  How far along Into a point of the tool frame stands with the tool at Pose.

   function Pushes (T : Track) return Natural is (T.Count);

   procedure Judge
     (T      : in out Track;
      From   : Pose_Estimate;
      Target : Pose_Estimate;
      To     : Pose_Estimate;
      Into   : Vec3;
      Where  : Points;
      Least  : Real;
      Said   : out Judgment)
   is
      --  A share of free pushes has a scatter from three on.
      Enough      : constant Boolean := T.Count > 2;
      Any_Lowers  : Boolean := False;   --  some point was asked to go down at all
      Any_Asked   : Boolean := False;   --  and some by what the tool's noise tells
      Any_Stopped : Boolean := False;   --  and some of those stopped short as free pushes do not
      Worst       : Judgment;           --  of the points asked, the one that fell shortest by share
      Stall       : Judgment;           --  and, of those that stopped, the same
      Biggest     : Real := 0.0;        --  the largest share, either way, of any point asked
   begin
      Said := (others => <>);
      for I in Where'Range loop
         declare
            Start : constant Real := Down (From.Pose, Where (I), Into);
            Ask   : constant Real := Down (Target.Pose, Where (I), Into) - Start;
            Got   : constant Real := Down (To.Pose, Where (I), Into) - Start;
            Short : constant Real := Ask - Got;
         begin
            Any_Lowers := Any_Lowers or else Ask > 0.0;
            if Ask >= Least then
               declare
                  Share : constant Real := Short / Ask;
                  Here  : constant Judgment :=
                    (Result => Lowered, Point => I, Asked => Ask, Went => Got, Share => Share, Free => T.Largest);
               begin
                  Biggest := Real'Max (Biggest, abs Share);
                  if not Any_Asked or else Share > Worst.Share then
                     Worst := Here;
                  end if;
                  Any_Asked := True;
                  --  A shortfall the tool's noise tells, and larger by share than any free push fell short by.
                  if Enough and then Short >= Least and then Share > Driver.Conventions.Z * T.Largest then
                     if not Any_Stopped or else Share > Stall.Share then
                        Stall := Here;
                     end if;
                     Any_Stopped := True;
                  end if;
               end;
            end if;
         end;
      end loop;
      if not Any_Lowers then
         --  A retreat or a hold: the next descent is compared with itself.
         T := (others => <>);
         return;
      end if;
      if not Any_Asked then
         return;
      end if;
      if Any_Stopped then
         Said := Stall;
         Said.Result := Stalled;
         return;
      end if;
      Said := Worst;
      Said.Result := (if Enough then Lowered else Too_Few);
      T.Count := T.Count + 1;
      T.Largest := Real'Max (T.Largest, Biggest);
   end Judge;

end Driver.Robot.Hand.Lowering;
