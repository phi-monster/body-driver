with Driver.Beats;
with Driver.Brain.Round;
with Driver.Brain.Words;
with Driver.Instrument;
with Driver.Log;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.Robot.Surfaces;
with Driver.Alignment;
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

   function Mostly (Part, Whole : Driver.Images.Mask) return Boolean is
      Inside : Natural := 0;
   begin
      for R in 0 .. Natural'Min (Driver.Images.Height (Part), Driver.Images.Height (Whole)) - 1 loop
         for C in 0 .. Natural'Min (Driver.Images.Width (Part), Driver.Images.Width (Whole)) - 1 loop
            if Driver.Images.Contains (Whole, C, R) and then Driver.Images.Contains (Part, C, R) then
               Inside := Inside + 1;
            end if;
         end loop;
      end loop;
      return 2 * Inside > Driver.Images.Count (Whole);
   end Mostly;

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
      No_Points : constant Driver.Instrument.Prompt_Array := [];

      --  Is the patch part of the body, a thing already known, or a new one?
      --  A patch mostly on the body is a known thing it is also mostly on
      --  (held in front of the fingers), or else the body itself.
      procedure During is
         O    : constant Driver.Observations.Observation := Driver.Beats.Latest.all;
         S    : Driver.World.Scene renames B.C.Scene.all;
         Self : constant Driver.Images.Mask := Driver.Robot.Self_Mask (B.C.Robot.all, E, O);
      begin
         if Mostly (Self, Region) then
            for K in 1 .. Driver.World.Thing_Count (S) loop
               declare
                  Known : constant Driver.World.Thing_Id := Driver.World.Thing_Id (K);
               begin
                  if Driver.World.Seen_In (S, Known, E) and then Mostly (Driver.World.Region_In (S, Known, E), Region)
                  then
                     Found := Driver.Brain.Names.A_Thing;
                     T := Known;
                     return;
                  end if;
               end;
            end loop;
            Found := Driver.Brain.Names.Part_Of_Me;
            Why := To_Unbounded_String ("most of its pixels lie on my own body in that eye");
            return;
         end if;
         Driver.World.Adopt (S, B.C.Robot.all, E, O, Region, T);
         Found := Driver.Brain.Names.A_Thing;
      end During;
      --  An eye that rides on a fitted arm tells which pixels in the box are the thing by itself: those whose
      --  points, placed from the places the arm's sweep held the eye at, stand above the surface they rest on
      --  (Driver.Robot.Surfaces), each the patch of half a patch's width about it that the point was asked at. No
      --  instrument segments the box for it: the body on the Pi has none, and what stands on a surface is what
      --  the body can reach for. An eye that stands still in the world has no such views of its own, and needs the
      --  instrument's segmentation; without it the next eye is asked (Driver.Brain.Names).
      Placed_By_Eye : Boolean := False;
      procedure Placing is
         use type Driver.Robot.Mount_Kind;
         O    : constant Driver.Observations.Observation := Driver.Beats.Latest.all;
         M    : Driver.Robot.Model renames B.C.Robot.all;
         S    : Driver.World.Scene renames B.C.Scene.all;
      begin
         if Natural (E) > Driver.Robot.Eye_Count (M)
           or else Driver.Robot.Eye_Mount (M, E).Kind /= Driver.Robot.Arm_Carried
           or else Natural (E) > Natural (O.Images.Length)
         then
            return;
         end if;
         declare
            W    : constant Natural := Driver.Images.Width (O.Images (E));
            H    : constant Natural := Driver.Images.Height (O.Images (E));
            Self : constant Driver.Images.Mask := Driver.Robot.Self_Mask (M, E, O);
            Box_Mask : Driver.Images.Mask := Driver.Images.Create (W, H);
            Half : constant Natural := Natural'Max (1, Driver.Alignment.Half_Patch (W, H)) / 2;
         begin
            for R in Natural (Real'Max (0.0, Real'Floor (Where.Top_Left.V))) ..
                     Natural (Real'Min (Real (H - 1), Real'Floor (Where.Bottom_Right.V)))
            loop
               for C in Natural (Real'Max (0.0, Real'Floor (Where.Top_Left.U))) ..
                        Natural (Real'Min (Real (W - 1), Real'Floor (Where.Bottom_Right.U)))
               loop
                  if not (Driver.Images.Width (Self) = W and then Driver.Images.Contains (Self, C, R)) then
                     Driver.Images.Include (Box_Mask, C, R);
                  end if;
               end loop;
            end loop;
            declare
               Placed : constant Driver.Robot.Surfaces.Surface_Point_Array :=
                 Driver.Robot.Surfaces.In_Region (M, E, O, Box_Mask);
            begin
               Placed_By_Eye := True;
               if Placed'Length = 0 then
                  Why := To_Unbounded_String ("nothing in the box stands above the surface it rests on, as this eye "
                                              & "places it from the places its arm held it at");
                  return;
               end if;
               Region := Driver.Images.Create (W, H);
               for P of Placed loop
                  for R in Natural (Real'Max (0.0, Real'Floor (P.Pixel.V) - Real (Half))) ..
                           Natural (Real'Min (Real (H - 1), Real'Floor (P.Pixel.V) + Real (Half)))
                  loop
                     for C in Natural (Real'Max (0.0, Real'Floor (P.Pixel.U) - Real (Half))) ..
                              Natural (Real'Min (Real (W - 1), Real'Floor (P.Pixel.U) + Real (Half)))
                     loop
                        if Driver.Images.Contains (Box_Mask, C, R) then
                           Driver.Images.Include (Region, C, R);
                        end if;
                     end loop;
                  end loop;
               end loop;
               for K in 1 .. Driver.World.Thing_Count (S) loop
                  declare
                     Known : constant Driver.World.Thing_Id := Driver.World.Thing_Id (K);
                  begin
                     if Driver.World.Seen_In (S, Known, E) and then Mostly (Driver.World.Region_In (S, Known, E), Region)
                     then
                        Found := Driver.Brain.Names.A_Thing;
                        T := Known;
                        return;
                     end if;
                  end;
               end loop;
               Driver.World.Adopt (S, M, E, O, Region, Placed, T);
               Found := Driver.Brain.Names.A_Thing;
            end;
         end;
      end Placing;
   begin
      T := Driver.Brain.Names.Thing_Id'First;
      Why := Null_Unbounded_String;
      Found := Driver.Brain.Names.No_Patch;
      Within_A_Beat (Placing'Access);
      if Placed_By_Eye then
         return;
      end if;
      Driver.Instrument.Segment
        (B.Now.Images (E), Has_Box => True,
         Around => (X0 => Where.Top_Left.U, Y0 => Where.Top_Left.V, X1 => Where.Bottom_Right.U, Y1 => Where.Bottom_Right.V),
         Points => No_Points, Region => Region, Score => Score, Ok => Ok, Why => Why);
      if not Ok then
         Why := "the instrument could not segment it: " & Why;
         return;
      end if;
      if Driver.Images.Count (Region) = 0 then
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
               --  Where the part the role is bound to now is (Driver.Action):
               --  the same for every body, however many parts could play it.
               P := Driver.Action.Role_Point (B.C.all, Who.The_Role);
               if not Driver.Uncertain.Known (P) then
                  Ok := False;
                  Why := To_Unbounded_String
                    ("no part of me plays " & Driver.Brain.Words.Word (Who.The_Role)
                     & " now, or I have not measured where it is, so I cannot remember where it is");
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
