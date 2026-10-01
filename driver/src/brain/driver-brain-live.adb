with Driver.Beats;
with Driver.Brain.Pictures;
with Driver.Brain.Round;
with Driver.Json;
with Driver.Log;
with Driver.Robot;
with Driver.Robot.Hand;
with Driver.Services;
with Driver.Uncertain;

package body Driver.Brain.Live is

   use type Driver.World.Surface_Id;
   use type Driver.Robot.Hand.Hand_Id;
   use type Driver.Action.Operand_Kind;

   --  Inside a beat's window the decider may read the models and
   --  Driver.Beats.Latest; the beat then gets Hold. Taking a beat and
   --  replying Hold must go through Driver.Robot.Motion, the only sender, and
   --  the primitive for it is not in Motion's specification yet.
   procedure Within_A_Beat (During : not null access procedure) is
   begin
      raise Program_Error with "the brain reads the body inside a beat's window, which needs a Driver.Robot.Motion"
        & " primitive that takes one beat and holds the body still";
   end Within_A_Beat;

   procedure Start (B : in out Body_Link) is
   begin
      B.Episode := Driver.Beats.Episode;
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
         Now.Images := O.Images;
         Now.Instruction := O.Instruction;
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
            Meanings.Append ("");
         end loop;
         Now.Keys := Driver.Brain.Keyboard.Choose
           (Quantities       => Q,
            Meanings         => Meanings,
            Roles            => [for R in Driver.Action.Role => Driver.Action.Can_Bind (B.C.all, R)],
            Relations        => [others => True],
            Surface_Measured => Surface,
            Two_Things       => [others => False]);
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

   function Region_Of_Runs (Width, Height : Natural; Runs : Driver.Natural_Array; Ok : out Boolean)
     return Driver.Images.Mask
   is
      M      : Driver.Images.Mask := Driver.Images.Create (Width, Height);
      At_Px  : Natural := 0;
      Inside : Boolean := False;
   begin
      for R of Runs loop
         if At_Px + R > Width * Height then
            Ok := False;
            return M;
         end if;
         if Inside then
            for P in At_Px .. At_Px + R - 1 loop
               Driver.Images.Include (M, P mod Width, P / Width);
            end loop;
         end if;
         At_Px := At_Px + R;
         Inside := not Inside;
      end loop;
      Ok := At_Px = Width * Height;
      return M;
   end Region_Of_Runs;

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

   --  The instrument's /segment (docs/instrument-service.md): the pixels of
   --  the thing inside a box.
   procedure Segment
     (Picture : Driver.Images.Image;
      Where   : Driver.Brain.Names.Box;
      Region  : out Driver.Images.Mask;
      Ok      : out Boolean;
      Why     : out Unbounded_String)
   is
      function Pixel_Image (X : Real) return String is (Driver.Log.Image (Integer (X)));
      Request : constant String :=
        "{""image"":" & Driver.Json.Quote (Driver.Brain.Pictures.Base64 (Driver.Brain.Pictures.Bmp (Picture)))
        & ",""box"":[" & Pixel_Image (Real'Floor (Where.Top_Left.U)) & "," & Pixel_Image (Real'Floor (Where.Top_Left.V))
        & "," & Pixel_Image (Real'Ceiling (Where.Bottom_Right.U)) & ","
        & Pixel_Image (Real'Ceiling (Where.Bottom_Right.V)) & "]}";
      R   : constant Driver.Services.Reply := Driver.Services.Call (Driver.Services.Instrument, "/segment", Request);
      Doc : Driver.Json.Document;
   begin
      Region := Driver.Images.Create (0, 0);
      Ok := False;
      if not R.Ok then
         Why := "the instrument could not segment it: " & R.Why;
         return;
      end if;
      Driver.Json.Parse (To_String (R.Text), Doc, Ok, Why);
      if not Ok then
         Why := "the instrument's answer is not JSON: " & Why;
         return;
      end if;
      declare
         Root : constant Driver.Json.Node := Driver.Json.Root (Doc);
         Runs : constant Driver.Json.Node := Driver.Json.Lookup (Doc, Root, "runs");
         Count : constant Natural := Driver.Json.Count (Doc, Runs);
         Lengths : Driver.Natural_Array (1 .. Count);
      begin
         if not Driver.Json.Is_True (Doc, Driver.Json.Lookup (Doc, Root, "ok")) then
            Ok := False;
            Why := "the instrument says: " & To_Unbounded_String
              (Driver.Json.Text (Doc, Driver.Json.Lookup (Doc, Root, "err")));
            return;
         end if;
         for I in Lengths'Range loop
            Lengths (I) := Natural (Driver.Json.Number (Doc, Driver.Json.Element (Doc, Runs, I)));
         end loop;
         Region := Region_Of_Runs (Driver.Images.Width (Picture), Driver.Images.Height (Picture), Lengths, Ok);
         if not Ok then
            Why := To_Unbounded_String ("the instrument's runs do not cover the picture");
         end if;
      end;
   end Segment;

   overriding procedure Identify
     (B     : in out Body_Link;
      E     : Driver.Brain.Names.Eye_Id;
      Where : Driver.Brain.Names.Box;
      Found : out Driver.Brain.Names.Patch;
      T     : out Driver.Brain.Names.Thing_Id;
      Why   : out Unbounded_String)
   is
      Region : Driver.Images.Mask;
      Ok     : Boolean;
      Point  : Driver.Images.Pixel;

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
      Segment (B.Now.Images (E), Where, Region, Ok, Why);
      if not Ok then
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
            when Driver.Action.Role_Operand | Driver.Action.Nothing =>
               Ok := False;
               Why := To_Unbounded_String ("I cannot yet tell where a part of me is as a place");
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

   overriding function Interrupted (B : Body_Link) return Boolean is (B.Episode_Over);

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
