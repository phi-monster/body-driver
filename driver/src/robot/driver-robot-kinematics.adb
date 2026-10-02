with Ada.Strings.Unbounded;
with Driver.Instrument;
with Driver.Log;
with Driver.Robot.Channels;
with Driver.Robot.Flow;

package body Driver.Robot.Kinematics is

   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;

   --  The arm's eye: the one the graph mounts on it; 0 when none.
   function Eye_Of (M : Model; A : Arm_Id) return Eye_Id'Base is
   begin
      for E in M.Graph.Mounts.First_Index .. M.Graph.Mounts.Last_Index loop
         if M.Graph.Mounts (E).Kind = Arm_Carried and then M.Graph.Mounts (E).Arm = A then
            return E;
         end if;
      end loop;
      return 0;
   end Eye_Of;

   --  The centres of the eye's cells that respond to the group's push: they
   --  show the world the arm moves its eye through. Every textured cell when
   --  the lock-in has not told yet.
   procedure Query_Points (M : Model; E : Eye_Id; G : Group_Id; U, V : out Real_Vectors.Vector) is
      S    : Eye_Stream renames M.Eyes (E);
      N    : constant Natural := Cells (S.Grid);
      Kept : constant Natural := Natural (S.Kept_Groups.Length);
   begin
      U.Clear;
      V.Clear;
      for C in 1 .. N loop
         declare
            Responds : Boolean := Kept = 0;
            X0, X1, Y0, Y1 : Natural;
         begin
            for K in 0 .. Kept - 1 loop
               if S.Kept_Groups (K) = Natural (G) and then S.Shifts ((C - 1) * Kept + K) > 0.0 then
                  Responds := True;
               end if;
            end loop;
            if Responds and then (Natural (S.Textured.Length) < N or else S.Textured (C - 1)) then
               Flow.Bounds (S.Grid, C, X0, X1, Y0, Y1);
               U.Append (Real (X0 + X1) / 2.0);
               V.Append (Real (Y0 + Y1) / 2.0);
            end if;
         end;
      end loop;
   end Query_Points;

   function Readings_Of (M : Model; G : Group_Id; Beat : Natural) return Real_Array is
      Result : Real_Array (1 .. M.Groups (G).Size);
   begin
      for C in Result'Range loop
         Result (C) := Channels.Reading (M, G, Beat, C);
      end loop;
      return Result;
   end Readings_Of;

   procedure Collect (R : in out Arm_Evidence) is
      K : Natural := R.Pending.First_Index;
   begin
      while K <= R.Pending.Last_Index loop
         if Driver.Services.Ready (R.Pending (K).Ticket) then
            declare
               P      : constant Pending_Match := R.Pending (K);
               Reply  : constant Driver.Services.Reply := Driver.Services.Collect (P.Ticket);
               Result : Driver.Instrument.Answer_Array (1 .. Natural (R.Query_U.Length));
               Ok     : Boolean;
               Why    : Ada.Strings.Unbounded.Unbounded_String;
               Set    : Match_Set;
            begin
               Driver.Instrument.Read_Match (Reply, True, Result, Ok, Why);
               if Ok then
                  Set.Frame := P.Frame;
                  for A of Result loop
                     Set.To_U.Append (A.To.U);
                     Set.To_V.Append (A.To.V);
                     Set.Back_U.Append (A.Back.U);
                     Set.Back_V.Append (A.Back.V);
                     Set.Found.Append (A.Found);
                  end loop;
                  R.Matches.Append (Set);
               elsif Reply.Lasting then
                  if not R.Unanswerable then
                     Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image
                                      & " gets no matches, and asks no more: " & Ada.Strings.Unbounded.To_String (Why));
                  end if;
                  R.Unanswerable := True;
               else
                  Driver.Log.Line (Driver.Log.Robot, "kinematics: arm" & R.Arm'Image & " keyframe" & P.Frame'Image
                                   & " has no matches: " & Ada.Strings.Unbounded.To_String (Why));
               end if;
               R.Pending.Delete (K);
            end;
         else
            K := K + 1;
         end if;
      end loop;
   end Collect;

   procedure Observe (M : in out Model; O : Observation) is
      Beat : constant Natural := M.Beats;
   begin
      for A in 1 .. Arm_Count (M) loop
         declare
            Arm : constant Arm_Id := Arm_Id (A);
            G   : constant Group_Id := Arm_Group (M, Arm);
            E   : constant Eye_Id'Base := Eye_Of (M, Arm);
            Index : Natural := 0;
         begin
            if E > 0 then
               for K in M.Kinematics.First_Index .. M.Kinematics.Last_Index loop
                  if M.Kinematics (K).Arm = Arm then
                     Index := K;
                  end if;
               end loop;
               --  An arm met for the first time, or one whose group or eye the
               --  graph now tells differently, starts its evidence afresh.
               if Index = 0 then
                  M.Kinematics.Append (Arm_Evidence'(Arm => Arm, Group => G, Eye => E, others => <>));
                  Index := M.Kinematics.Last_Index;
               elsif M.Kinematics (Index).Group /= G or else M.Kinematics (Index).Eye /= E then
                  M.Kinematics.Replace_Element (Index, (Arm => Arm, Group => G, Eye => E, others => <>));
               end if;
               declare
                  R : Arm_Evidence renames M.Kinematics (Index);
                  S : Eye_Stream renames M.Eyes (E);
               begin
                  Collect (R);
                  if Beat > 0 and then Beat < Natural (S.Judged.Length)
                    and then S.Judged (Beat) and then S.Still_At (Beat)
                    and then S.Judged (Beat - 1) and then S.Still_At (Beat - 1)
                    and then Channels.Has_Reading (M, G, Beat) and then Channels.Has_Reading (M, G, Beat - 1)
                    and then not Channels.Moving (M, G, Beat)
                    and then E <= O.Images.Last_Index and then Driver.Observations.Has_Image (O, E)
                  then
                     declare
                        Now   : constant Real_Array := Readings_Of (M, G, Beat);
                        Fresh : Boolean := True;
                     begin
                        --  A view worth matching shows something new: against every
                        --  keyframe, some channel moved by at least the step its eye
                        --  can see (Visible_Step). None is taken before those steps
                        --  are measured.
                        for F of R.Frames loop
                           declare
                              Apart : Boolean := False;
                           begin
                              for C in Now'Range loop
                                 declare
                                    V : constant Estimate := Visible_Step (M, G, C);
                                 begin
                                    if Known (V) and then abs (Now (C) - F.Readings (C - 1)) >= V.Value then
                                       Apart := True;
                                    end if;
                                 end;
                              end loop;
                              Fresh := Fresh and then Apart;
                           end;
                        end loop;
                        declare
                           Seeable : Boolean := False;
                        begin
                           for C in Now'Range loop
                              Seeable := Seeable or else Known (Visible_Step (M, G, C));
                           end loop;
                           Fresh := Fresh and then Seeable;
                        end;
                        if Fresh and then not R.Unanswerable then
                           declare
                              K : Keyframe;
                           begin
                              K.Beat := Beat;
                              for X of Now loop
                                 K.Readings.Append (X);
                              end loop;
                              K.Image := O.Images (E);
                              R.Frames.Append (K);
                           end;
                           if Natural (R.Frames.Length) = 1 then
                              Query_Points (M, E, G, R.Query_U, R.Query_V);
                           elsif not R.Query_U.Is_Empty and then not R.Unanswerable then
                              declare
                                 Points : Driver.Instrument.Point_Array (1 .. Natural (R.Query_U.Length));
                              begin
                                 for P in Points'Range loop
                                    Points (P) := (U => R.Query_U (P - 1), V => R.Query_V (P - 1));
                                 end loop;
                                 R.Pending.Append
                                   (Pending_Match'(Frame  => R.Frames.Last_Index,
                                     Ticket => Driver.Instrument.Submit_Match
                                       ((Stored => False, Image => R.Frames.First_Element.Image),
                                        (Stored => False, Image => O.Images (E)),
                                        Points, True, O.Beat)));
                              end;
                           end if;
                        end if;
                     end;
                  end if;
               end;
            end if;
         end;
      end loop;
   end Observe;

   function Matched (M : Model; A : Arm_Id) return Natural is
   begin
      for R of M.Kinematics loop
         if R.Arm = A then
            return Natural (R.Matches.Length);
         end if;
      end loop;
      return 0;
   end Matched;

end Driver.Robot.Kinematics;
