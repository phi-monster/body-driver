with Driver.Beats;
with Driver.Brain.Round;
with Driver.Brain.Words;
with Driver.Instrument;
with Driver.Log;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.Uncertain;

package body Driver.Brain.Live is

   use type Driver.World.Surface_Id;
   use type Driver.Robot.Hand.Hand_Id;
   use type Driver.Action.Operand_Kind;
   use type Driver.Action.Role;

   --  Inside a beat's window the decider may read the models and
   --  Driver.Beats.Latest; the beat is answered with a hold.
   procedure Within_A_Beat (During : not null access procedure) renames Driver.Beats.Within_A_Beat;

   procedure Start (B : in out Body_Link) is
   begin
      B.Episode := Driver.Beats.Episode;
      B.Heard := Driver.Beats.Words_Heard;
   end Start;

   overriding function Episode_Over (B : Body_Link) return Boolean is (Driver.Beats.Episode /= B.Episode);

   function Words_Of (V : Driver.Action.Name_Vectors.Vector) return Driver.Brain.Keyboard.Word_Vectors.Vector is
      R : Driver.Brain.Keyboard.Word_Vectors.Vector;
   begin
      for W of V loop
         R.Append (W);
      end loop;
      return R;
   end Words_Of;

   overriding procedure Look
     (B : in out Body_Link; Named : Driver.Brain.Names.Table; Now : out Driver.Brain.Rounds.Snapshot)
   is
      procedure During is
         O        : constant Driver.Observations.Observation := Driver.Beats.Latest.all;
         S        : Driver.World.Scene renames B.C.Scene.all;
         Surface  : Boolean := False;
         Meanings : Driver.Brain.Keyboard.Word_Vectors.Vector;
         Q        : constant Driver.Brain.Keyboard.Word_Vectors.Vector := Words_Of (Driver.Action.Quantities (B.C.all));
      begin
         B.Now := O;
         B.Heard := Driver.Beats.Words_Heard;
         Now.Images := O.Images;
         Now.Instruction := To_Unbounded_String (Driver.Beats.Latest_Words);
         for E in O.Images.First_Index .. O.Images.Last_Index loop
            Now.Eyes.Append (Driver.Brain.Round.Eye_Facts'(Eye   => E,
                                                           Mount => Driver.Robot.Eye_Mount (B.C.Robot.all, E)));
         end loop;
         for I in 1 .. Named.Named_Count loop
            declare
               T : constant Driver.World.Thing_Id := Named.Named_Thing (I);
               F : Driver.Brain.Round.Thing_Facts;
            begin
               F.Name := To_Unbounded_String (Named.Name_Of (T));
               for E in O.Images.First_Index .. O.Images.Last_Index loop
                  if Driver.World.Seen_In (S, T, E) then
                     F.Seen_By.Append (E);
                  end if;
               end loop;
               F.Held := Driver.World.Held_By (S, T) /= 0;
               F.Height := Driver.World.Height_Above_Support (S, T);
               Surface := Surface or else Driver.World.Resting_On (S, T) /= 0;
               Now.Things.Append (F);
            end;
         end loop;
         for W of Q loop
            Meanings.Append (Driver.Action.Meaning (W));
         end loop;
         Now.Keys := Driver.Brain.Keyboard.Choose
           (Quantities       => Q,
            Meanings         => Meanings,
            Roles            => [for R in Driver.Action.Role => Driver.Action.Can_Bind (B.C.all, R)],
            Relations        => [for R in Driver.Action.Relation => Driver.Action.Usable (B.C.all, R)],
            Surface_Measured => Surface,
            Two_Things       => [others => False],
            Eyes             => B.Eyes);
      end During;
   begin
      Now := (others => <>);
      Within_A_Beat (During'Access);
   end Look;

   overriding function Eyes (B : Body_Link) return Driver.Brain.Names.Eye_Vectors.Vector is
      R : Driver.Brain.Names.Eye_Vectors.Vector;
   begin
      for E in B.Now.Images.First_Index .. B.Now.Images.Last_Index loop
         if Driver.Observations.Has_Image (B.Now, E) then
            R.Append (E);
         end if;
      end loop;
      return R;
   end Eyes;

   overriding function Sees (B : Body_Link; T : Driver.Brain.Names.Thing_Id; E : Driver.Brain.Names.Eye_Id)
                             return Boolean
   is
      Seen : Boolean := False;

      procedure During is
      begin
         Seen := Driver.World.Seen_In (B.C.Scene.all, T, E);
      end During;
   begin
      Within_A_Beat (During'Access);
      return Seen;
   end Sees;

   overriding procedure Ask_Where
     (B      : in out Body_Link;
      E      : Driver.Brain.Names.Eye_Id;
      Name   : String;
      Answer : out Driver.Brain.Names.Pointing;
      Where  : out Driver.Brain.Names.Box;
      Why    : out Unbounded_String)
   is
   begin
      Driver.Brain.Service.Ask_Where (B.Now.Images (E), Name, Answer, Where, Why);
   end Ask_Where;

   function Own_Point (M : Driver.Images.Mask; Found : out Boolean) return Driver.Images.Pixel is
      Su, Sv : Real := 0.0;
      N      : Natural := 0;
      Best   : Driver.Images.Pixel;
      Near   : Real := Real'Last;
      Half   : constant := 0.5;   --  the centre of a pixel (Driver.Images)
   begin
      for R in 0 .. Driver.Images.Height (M) - 1 loop
         for C in 0 .. Driver.Images.Width (M) - 1 loop
            if Driver.Images.Contains (M, C, R) then
               Su := Su + Real (C) + Half;
               Sv := Sv + Real (R) + Half;
               N := N + 1;
            end if;
         end loop;
      end loop;
      Found := N > 0;
      if not Found then
         return (others => <>);
      end if;
      for R in 0 .. Driver.Images.Height (M) - 1 loop
         for C in 0 .. Driver.Images.Width (M) - 1 loop
            if Driver.Images.Contains (M, C, R) then
               declare
                  D : constant Real := (Real (C) + Half - Su / Real (N)) ** 2 + (Real (R) + Half - Sv / Real (N)) ** 2;
               begin
                  if D < Near then
                     Near := D;
                     Best := (U => Real (C) + Half, V => Real (R) + Half);
                  end if;
               end;
            end if;
         end loop;
      end loop;
      return Best;
   end Own_Point;

   overriding procedure Identify
     (B     : in out Body_Link;
      E     : Driver.Brain.Names.Eye_Id;
      Where : Driver.Brain.Names.Box;
      Found : out Driver.Brain.Names.Patch;
      T     : out Driver.Brain.Names.Thing_Id;
      Why   : out Unbounded_String)
   is
      Region : Driver.Images.Mask;
      Score  : Real;
      Ok     : Boolean;
      Point  : Driver.Images.Pixel;
      No_Points : constant Driver.Instrument.Prompt_Array := [];

      --  Is the patch part of the body, a thing already known, or a new one?
      --  A point on both the body and a known thing is that thing (held in
      --  front of the fingers).
      procedure During is
         O    : constant Driver.Observations.Observation := Driver.Beats.Latest.all;
         S    : Driver.World.Scene renames B.C.Scene.all;
         Self : constant Driver.Images.Mask := Driver.Robot.Self_Mask (B.C.Robot.all, E, O);
         C    : constant Natural := Natural (Real'Floor (Point.U));
         R    : constant Natural := Natural (Real'Floor (Point.V));
      begin
         if Driver.Images.Contains (Self, C, R) then
            for K in 1 .. Driver.World.Thing_Count (S) loop
               declare
                  Known : constant Driver.World.Thing_Id := Driver.World.Thing_Id (K);
               begin
                  if Driver.World.Seen_In (S, Known, E)
                    and then Driver.Images.Contains (Driver.World.Region_In (S, Known, E), C, R)
                  then
                     Found := Driver.Brain.Names.A_Thing;
                     T := Known;
                     return;
                  end if;
               end;
            end loop;
            Found := Driver.Brain.Names.Part_Of_Me;
            Why := To_Unbounded_String ("its pixels lie on my own body in that eye");
            return;
         end if;
         Driver.World.Adopt (S, B.C.Robot.all, E, O, Region, T);
         Found := Driver.Brain.Names.A_Thing;
      end During;
   begin
      T := Driver.Brain.Names.Thing_Id'First;
      Why := Null_Unbounded_String;
      Found := Driver.Brain.Names.No_Patch;
      Driver.Instrument.Segment
        (B.Now.Images (E), Has_Box => True,
         Around => (X0 => Where.Top_Left.U, Y0 => Where.Top_Left.V, X1 => Where.Bottom_Right.U, Y1 => Where.Bottom_Right.V),
         Points => No_Points, Region => Region, Score => Score, Ok => Ok, Why => Why);
      if not Ok then
         Why := "the instrument could not segment it: " & Why;
         return;
      end if;
      Point := Own_Point (Region, Ok);
      if not Ok then
         Why := To_Unbounded_String ("nothing in the box stands apart from its surroundings");
         return;
      end if;
      Within_A_Beat (During'Access);
   end Identify;

   overriding function Quantities (B : Body_Link) return Driver.Brain.Keyboard.Word_Vectors.Vector is
      R : Driver.Brain.Keyboard.Word_Vectors.Vector;

      procedure During is
      begin
         R := Words_Of (Driver.Action.Quantities (B.C.all));
      end During;
   begin
      Within_A_Beat (During'Access);
      return R;
   end Quantities;

   overriding function Check (B : Body_Link; W : Driver.Action.Want) return Driver.Action.Verdict is
      V : Driver.Action.Verdict;

      procedure During is
      begin
         V := Driver.Action.Check (B.C.all, W);
      end During;
   begin
      Within_A_Beat (During'Access);
      return V;
   end Check;

   overriding procedure Execute (B : in out Body_Link; W : Driver.Action.Want; R : out Driver.Action.Result) is
   begin
      Driver.Action.Execute (B.C.all, W, R);
   end Execute;

   overriding procedure Place_Of
     (B     : in out Body_Link;
      Who   : Driver.Action.Operand;
      Place : out Driver.World.Place_Id;
      Ok    : out Boolean;
      Why   : out Unbounded_String)
   is
      procedure During is
         S : Driver.World.Scene renames B.C.Scene.all;
         P : Driver.Uncertain.Point_Estimate;
      begin
         case Who.Kind is
            when Driver.Action.Thing_Operand =>
               P := Driver.World.Centre (S, Who.Thing);
            when Driver.Action.Place_Operand =>
               P := Driver.World.Where (S, Who.Place);
            when Driver.Action.Role_Operand =>
               --  Which hand is the grasper is settled per stretch; with one hand
               --  it can only be that one, and with more the body does not guess.
               if Who.The_Role /= Driver.Action.Grasper then
                  Ok := False;
                  Why := To_Unbounded_String ("I cannot yet tell where " & Driver.Brain.Words.Word (Who.The_Role)
                                              & " is as a place");
                  return;
               elsif Driver.Robot.Hand.Hand_Count (B.C.Hands.all) /= 1 then
                  Ok := False;
                  Why := To_Unbounded_String
                    ("I have" & Natural'Image (Driver.Robot.Hand.Hand_Count (B.C.Hands.all))
                     & " hands and which one is the grasper is settled stretch by stretch, so I do not guess which"
                     & " one you mean; remember where the thing in it is instead");
                  return;
               end if;
               P := Driver.Robot.Hand.Grip_Centre (B.C.Hands.all, B.C.Robot.all, Driver.Robot.Hand.Hand_Id'First,
                                                   Driver.Beats.Latest.all);
               if not Driver.Uncertain.Known (P) then
                  Ok := False;
                  Why := To_Unbounded_String ("I have not measured yet where my grasper closes");
                  return;
               end if;
            when Driver.Action.Nothing =>
               Ok := False;
               Why := To_Unbounded_String ("there is nothing here to remember");
               return;
         end case;
         Driver.World.Remember (S, P, Place);
         Ok := True;
      end During;
   begin
      Place := Driver.World.Place_Id'First;
      Ok := False;
      Why := Null_Unbounded_String;
      Within_A_Beat (During'Access);
   end Place_Of;

   overriding procedure Say (B : in out Body_Link; Sentence : String) is
   begin
      Driver.Log.Line (Driver.Log.Brain, "the brain says: " & Sentence);
   end Say;

   overriding function Interrupted (B : Body_Link) return Boolean is
     (B.Episode_Over or else Driver.Beats.Words_Heard /= B.Heard);

   overriding function Write_Program
     (T       : in out Brain_Link;
      Picture : Driver.Images.Image;
      Prompt  : String;
      Keys    : Driver.Brain.Keyboard.Keyboard) return Driver.Brain.Service.Answer
   is
      pragma Unreferenced (T);
   begin
      return Driver.Brain.Service.Write_Program (Picture, Prompt, Keys);
   end Write_Program;

end Driver.Brain.Live;
