with Ada.Containers;
with Ada.Unchecked_Deallocation;
with Driver.Clock;
with Driver.Log;
with Driver.Robot.Channels;
with Driver.Robot.Flow;
with Driver.Robot.Graph;
with Driver.Robot.Lag;
with Driver.Robot.Lockin;
with Driver.Robot.Stillness;

package body Driver.Robot is

   use type Ada.Containers.Count_Type;
   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;

   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Luma_Access);

   --  The displacement of every cell of the eye from its previous frame to
   --  its current one, appended to its stream.
   procedure Measure_Displacement (S : in out Eye_Stream)
     with Pre => S.Has_Previous and then Cells (S.Grid) > 0
   is
      N : constant Positive := Cells (S.Grid);
      Du, Dv, Condition, Cell_Noise : Real_Array (1 .. N);
   begin
      for C in 1 .. N loop
         Cell_Noise (C) := S.Luma_Variance.Element (C - 1);
      end loop;
      Flow.Displacements (S.Grid, S.Previous.all, S.Current.all, Cell_Noise, Du, Dv, Condition);
      for C in 1 .. N loop
         S.Du.Append (Du (C));
         S.Dv.Append (Dv (C));
         S.Condition.Append (Condition (C));
      end loop;
      S.Measured.Append (True);
   end Measure_Displacement;

   procedure Observe_Eyes (M : in out Model; O : Observation) is
   begin
      if M.Eyes.Is_Empty then
         for E in O.Images.First_Index .. O.Images.Last_Index loop
            M.Eyes.Append (Eye_Stream'(others => <>));
         end loop;
      end if;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            S    : Eye_Stream renames M.Eyes (E);
            Have : constant Boolean := E <= O.Images.Last_Index and then Driver.Observations.Has_Image (O, E);
         begin
            if Have then
               declare
                  Size : constant Natural := Driver.Images.Width (O.Images (E)) * Driver.Images.Height (O.Images (E));
               begin
                  if S.Current = null or else S.Current'Length /= Size then
                     Free (S.Current);
                     S.Current := new Real_Array (1 .. Size);
                  end if;
                  Driver.Images.Luma (O.Images (E), S.Current.all);
               end;
               if S.Grid.Width = 0 then
                  S.Grid := Flow.Grid_Of (Driver.Images.Width (O.Images (E)), Driver.Images.Height (O.Images (E)));
                  S.Du.Append (0.0, Ada.Containers.Count_Type (Cells (S.Grid) * M.Beats));
                  S.Dv.Append (0.0, Ada.Containers.Count_Type (Cells (S.Grid) * M.Beats));
                  S.Condition.Append (0.0, Ada.Containers.Count_Type (Cells (S.Grid) * M.Beats));
                  S.Measured.Append (False, Ada.Containers.Count_Type (M.Beats));
               end if;
            end if;
            declare
               N    : constant Natural := Cells (S.Grid);
               Same : constant Boolean :=
                 Have and then Driver.Images.Width (O.Images (E)) = S.Grid.Width
                 and then Driver.Images.Height (O.Images (E)) = S.Grid.Height;
            begin
               if Same and then S.Has_Previous then
                  Measure_Displacement (S);
               elsif N > 0 then
                  S.Du.Append (0.0, Ada.Containers.Count_Type (N));
                  S.Dv.Append (0.0, Ada.Containers.Count_Type (N));
                  S.Condition.Append (0.0, Ada.Containers.Count_Type (N));
                  S.Measured.Append (False);
               end if;
               if Have then
                  Stillness.Judge_Eye (S, O.Images (E), S.Current.all);
                  if S.Luma_Variance.Is_Empty then
                     Stillness.Measure_Luma_Noise (S);
                  end if;
               end if;
               --  This frame is the next one's previous; a missing frame, or
               --  one of another size, breaks the chain: the next displacement
               --  would span two beats.
               if Same then
                  declare
                     Spare : constant Luma_Access := S.Previous;
                  begin
                     S.Previous := S.Current;
                     S.Current := Spare;
                  end;
               end if;
               S.Has_Previous := Same;
            end;
         end;
      end loop;
   end Observe_Eyes;

   procedure Estimate_Now (M : in out Model) is
      Start : constant Duration := Driver.Clock.Seconds;
   begin
      Channels.Measure_Noise (M);
      for S of M.Eyes loop
         if S.Has_Settled then
            Stillness.Measure_Luma_Noise (S);
         end if;
      end loop;
      Channels.Measure_Pushes (M);
      Lag.Measure (M);
      Lockin.Measure (M);
      Graph.Derive (M);
      M.Graph_Evidence := M.Beats;
      Driver.Log.Line (Driver.Log.Robot, "estimated from" & M.Beats'Image & " beats in"
                       & Driver.Log.Image (Real (Driver.Clock.Seconds - Start), 1) & " s");
   end Estimate_Now;

   procedure Observe (M : in out Model; O : Observation; Sent : Driver.Commands.Command) is
   begin
      Channels.Append (M, O, Sent);
      Observe_Eyes (M, O);
      M.Beats := M.Beats + 1;
      --  The estimates are redone whenever the evidence behind them has
      --  doubled: a logarithmic number of times over any stream.
      if M.Beats >= 2 * M.Graph_Evidence then
         Estimate_Now (M);
      end if;
   end Observe;

   function Booted (M : Model) return Boolean is (M.Is_Booted);

   function Role (M : Model; G : Group_Id) return Group_Role is
     (if G <= M.Graph.Roles.Last_Index then M.Graph.Roles (G) else Unclassified);

   function Arm_Count (M : Model) return Natural is (Natural (M.Graph.Arms.Length));

   function Arm_Group (M : Model; A : Arm_Id) return Group_Id is (M.Graph.Arms (A));

   function Eye_Count (M : Model) return Natural is (Natural (M.Eyes.Length));

   function Eye_Mount (M : Model; E : Eye_Id) return Mount is
     (if E <= M.Graph.Mounts.Last_Index then M.Graph.Mounts (E) else (Kind => Unmeasured));

   function Eye_Pose (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate is ((others => <>));

   procedure Project
     (M       : Model;
      E       : Eye_Id;
      O       : Observation;
      Point   : Vec3;
      Px      : out Driver.Images.Pixel;
      Visible : out Boolean)
   is
      pragma Unreferenced (M, E, O, Point);
   begin
      Px := (U => 0.0, V => 0.0);
      Visible := False;
   end Project;

   function Ray (M : Model; E : Eye_Id; O : Observation; Px : Driver.Images.Pixel) return Ray_Estimate is
     ((others => <>));

   function Eye_Ray (M : Model; E : Eye_Id; Px : Driver.Images.Pixel) return Ray_Estimate is ((others => <>));

   function Up (M : Model) return Direction_Estimate is ((others => <>));

   function Tool_Pose (M : Model; A : Arm_Id; O : Observation) return Pose_Estimate is ((others => <>));

   function Eye_In_Tool (M : Model; E : Eye_Id; O : Observation) return Pose_Estimate is ((others => <>));

   function Blocked (M : Model; A : Arm_Id; O : Observation) return Boolean is (False);

   function Self_Mask (M : Model; E : Eye_Id; O : Observation) return Driver.Images.Mask is
     (if E <= O.Images.Last_Index then Driver.Images.Create (Driver.Images.Width (O.Images (E)), Driver.Images.Height (O.Images (E)))
      else Driver.Images.Create (0, 0));

   function Clearance (M : Model; Point : Vec3; O : Observation) return Estimate is (Unknown);

   function Still (M : Model) return Boolean is
     (Stillness.All_Still (M));

   function Group_Count (M : Model) return Natural is (Natural (M.Groups.Length));

   function Group_Size (M : Model; G : Group_Id) return Natural is
     (if G <= M.Groups.Last_Index then M.Groups (G).Size else 0);

   function Is_Commandable (M : Model; G : Group_Id) return Boolean is
     (G <= M.Groups.Last_Index and then M.Groups (G).Commandable);

   function Reading_Noise (M : Model; G : Group_Id; Channel : Positive) return Real is
     (Channels.Noise (M, G, Channel));

   function Visible_Step (M : Model; G : Group_Id; Channel : Positive) return Estimate is (Unknown);

   function Response (M : Model; G : Group_Id; E : Eye_Id) return Eye_Response is
     (Graph.Effect (M, G, E).Verdict);

   function Image_Lag (M : Model; E : Eye_Id) return Integer is
     (if E <= M.Lags.Last_Index then M.Lags (E) else 0);

   function Lag_Known (M : Model; E : Eye_Id) return Boolean is
     (E <= M.Lag_Known.Last_Index and then M.Lag_Known (E));

   function Closer_Arm (M : Model; G : Group_Id) return Arm_Id'Base is
     (if Role (M, G) = Closer then M.Graph.Arm_Of (G) else 0);

   function Carrier_Group (M : Model) return Group_Id'Base is (M.Graph.Carrier);

   function Contract_Breach (M : Model; G : Group_Id) return Natural is
     (if G <= M.Graph.Breach.Last_Index then M.Graph.Breach (G) else 0);

   function Describe (M : Model) return String is
      use Driver.Log;
      T : Unbounded_String;
   begin
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         Append (T, "group" & Group_Id'Image (G) & ": " & Image (M.Groups (G).Size) & " values, "
                 & (if M.Groups (G).Commandable then "commandable" else "not commandable") & ", "
                 & Group_Role'Image (Role (M, G)));
         if M.Graph.Arm_Of.Length > 0 and then M.Graph.Arm_Of (G) > 0 then
            Append (T, " (arm" & Arm_Id'Image (M.Graph.Arm_Of (G)) & ")");
         end if;
         if Contract_Breach (M, G) > 0 then
            Append (T, ", breaks clause" & Natural'Image (Contract_Breach (M, G)));
         end if;
         for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
            declare
               F : constant Eye_Effect := Graph.Effect (M, G, E);
            begin
               if F.Verdict /= Unmeasured then
                  Append (T, "; eye" & Eye_Id'Image (E) & " " & Eye_Response'Image (F.Verdict) & " "
                          & Image (F.Responding) & "/" & Image (F.Textured));
               end if;
            end;
         end loop;
         Append (T, ASCII.LF);
      end loop;
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            Mt : constant Mount := Eye_Mount (M, E);
         begin
            Append (T, "eye" & Eye_Id'Image (E) & ": "
                    & (if Lag_Known (M, E) then "image lag" & Integer'Image (Image_Lag (M, E)) & " beats, "
                        else "image lag unmeasured, ")
                    & Mount_Kind'Image (Mt.Kind)
                    & (if Mt.Kind = Arm_Carried then " on arm" & Arm_Id'Image (Mt.Arm) else "") & ASCII.LF);
         end;
      end loop;
      return To_String (T);
   end Describe;

end Driver.Robot;
