with Ada.Command_Line;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Driver.Log;
with Driver.Robot.Kinematics.Hand_Eye;
with Driver.Robot.Kinematics.Hand_Eye.Dumps;

--  usage: driver-robot-hand_eye_replay DUMP [FX FY CX CY]
--  What the fit is given (per arm: keyframes, tracks, sightings, how many sightings each track has, the keyframes
--  that turned one joint alone, the tracks Infer_Link gives a link), then the fit itself, and the lens against the
--  truth's when it is named.
procedure Driver.Robot.Hand_Eye_Replay is
   use Ada.Text_IO;
   package Hand renames Driver.Robot.Kinematics.Hand_Eye;
   function Image (X : Real; Aft : Natural := 2) return String is (Driver.Log.Image (X, Aft));
begin
   if Ada.Command_Line.Argument_Count /= 1 and then Ada.Command_Line.Argument_Count /= 5 then
      Put_Line (Standard_Error, "usage: driver-robot-hand_eye_replay DUMP [FX FY CX CY]");
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
      return;
   end if;
   declare
      L      : constant Hand.Dumps.Loaded := Hand.Dumps.Load (Ada.Command_Line.Argument (1));
      Prior  : Hand.Fit_Report;
      Report : Hand.Fit_Report;
   begin
      Put_Line ("hand-eye dump of" & L.Count'Image & " arms, a picture of" & L.Width'Image & " by" & L.Height'Image);
      for K in 1 .. L.Count loop
         declare
            A       : Hand.Watched renames L.Arms (K).all;
            Per     : array (1 .. A.Tracks) of Natural := [others => 0];
            Linked  : Natural := 0;
            Alone   : array (1 .. A.Joints) of Natural := [others => 0];
            Hist    : array (0 .. 10) of Natural := [others => 0];
         begin
            for S of A.Seen loop
               if S.Track in Per'Range then
                  Per (S.Track) := Per (S.Track) + 1;
               end if;
            end loop;
            for N of Per loop
               Hist (Natural'Min (10, N)) := Hist (Natural'Min (10, N)) + 1;
            end loop;
            for T in 1 .. A.Tracks loop
               if Hand.Infer_Link (A, T) > 0 then
                  Linked := Linked + 1;
               end if;
            end loop;
            --  Keyframes that turned one joint alone from the reference (the arm's first keyframe).
            for F in 1 .. A.Frames loop
               declare
                  Moved, Which : Natural := 0;
               begin
                  for J in 1 .. A.Joints loop
                     if A.Visible (J) > 0.0 and then abs A.Change (F, J) >= A.Visible (J) then
                        Moved := Moved + 1;
                        Which := J;
                     end if;
                  end loop;
                  if Moved = 1 then
                     Alone (Which) := Alone (Which) + 1;
                  end if;
               end;
            end loop;
            Put ("arm" & K'Image & ":" & A.Joints'Image & " joints," & A.Frames'Image & " keyframes," & A.Tracks'Image
                 & " tracks," & A.Sightings'Image & " sightings; tracks by their sightings (0 .. 10+):");
            for N of Hist loop
               Put (N'Image);
            end loop;
            Put ("; keyframes of one joint alone from the reference, by joint:");
            for N of Alone loop
               Put (N'Image);
            end loop;
            Put_Line ("; tracks given a link:" & Linked'Image);
         end;
      end loop;
      Hand.Fit_Eye (L.Arms, L.Width, L.Height, Prior, Report);
      Put ("fit: " & (if Report.Determined then "DETERMINED" else "not determined") & ", focal "
           & Image (Report.L.Fx) & " x " & Image (Report.L.Fy) & ", centre " & Image (Report.L.Cx) & ", "
           & Image (Report.L.Cy) & ", used" & Report.Used'Image & " of" & Report.Offered'Image & ", noise "
           & Image (Report.Sigma_Px, 3) & " px");
      if Ada.Command_Line.Argument_Count = 5 and then Report.L.Fx > 0.0 then
         Put (", focal off the truth by " & Image (100.0 * (Report.L.Fx / Real'Value (Ada.Command_Line.Argument (2)) - 1.0), 1)
              & " %");
      end if;
      New_Line;
      for K in 1 .. Natural (Report.Arms.Length) loop
         declare
            Linked : Natural := 0;
         begin
            for X of Report.Arms (K).Link loop
               if X > 0 then
                  Linked := Linked + 1;
               end if;
            end loop;
            Put_Line ("  arm" & K'Image & ": " & (if Report.Arms (K).Placed then "placed" else "not placed")
                      & ", tracks the model gives a link at the end:" & Linked'Image);
         end;
      end loop;
      if not Report.Determined then
         Put_Line ("  why: " & Ada.Strings.Unbounded.To_String (Report.Why));
      end if;
   end;
end Driver.Robot.Hand_Eye_Replay;
