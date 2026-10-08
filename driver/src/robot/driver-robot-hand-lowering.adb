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
      Joints : Joint_Push;
      Said   : out Judgment)
   is
      --  A share of free pushes has a scatter from three on.
      Enough      : constant Boolean := T.Count > 2;
      --  What the push asked, and whether the shortfall of it is something the one test of motion sees: the floor
      --  under both measures.
      Seen        : constant Boolean := Joints.Asked and then Joints.Seen;
      Any_Lowers  : Boolean := False;   --  some point was asked to go down at all
      Any_Stopped : Boolean := False;   --  and some of them stopped short as free pushes do not
      Worst       : Judgment;           --  of the points asked, the one that fell shortest by share
      Stall       : Judgment;           --  and, of those that stopped, the same
      Biggest     : Real := 0.0;        --  the largest share, either way, of any point asked
      Asked_Any   : Boolean := False;   --  some point was asked to go down by this push, which asked a motion that is seen
      --  The readings: the share of the ask's length they stopped short of the target by, and whether
      --  that is more than free pushes stop short by.
      Joint_Share : constant Real :=
        (if Joints.Asked and then Joints.Length > 0.0 then Joints.Short / Joints.Length else 0.0);
      Joint_Stops : constant Boolean :=
        Enough and then Seen and then Joint_Share > Driver.Conventions.Z * T.Joint_Largest;
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
            if Ask > 0.0 and then Joints.Asked then
               declare
                  Share : constant Real := Short / Ask;
                  Here  : constant Judgment :=
                    (Result => Lowered, Point => I, Asked => Ask, Went => Got, Share => Share, Free => T.Largest,
                     others => <>);
               begin
                  Biggest := Real'Max (Biggest, abs Share);
                  if not Asked_Any or else Share > Worst.Share then
                     Worst := Here;
                  end if;
                  Asked_Any := True;
                  --  Larger by share than any free push fell short by.
                  if Enough and then Seen and then Share > Driver.Conventions.Z * T.Largest then
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
      if not Asked_Any then
         return;
      end if;
      Said := (if Any_Stopped then Stall else Worst);
      Said.Point_Stalled := Any_Stopped;
      Said.Joint_Share := Joint_Share;
      Said.Joint_Free := T.Joint_Largest;
      Said.Joint_Stalled := Joint_Stops;
      if Any_Stopped or else Joint_Stops then
         Said.Result := Stalled;
         return;
      end if;
      Said.Result := (if Enough then Lowered else Too_Few);
      T.Count := T.Count + 1;
      T.Largest := Real'Max (T.Largest, Biggest);
      T.Joint_Largest := Real'Max (T.Joint_Largest, Joint_Share);
   end Judge;

end Driver.Robot.Hand.Lowering;
