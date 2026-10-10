with Driver.Beats;
with Driver.Clock;
with Driver.Commands;
with Driver.Observations;
with Driver.Tests;

package body Driver.Lanes_Tests is

   use Driver.Tests;

   --  Who is inside a window now, and the most that ever were at once.
   protected Windows is
      procedure Enter;
      procedure Leave;
      function Most return Natural;
      procedure Reset;
   private
      Inside, Peak : Natural := 0;
   end Windows;

   protected body Windows is
      procedure Enter is
      begin
         Inside := Inside + 1;
         Peak := Natural'Max (Peak, Inside);
      end Enter;

      procedure Leave is
      begin
         Inside := Inside - 1;
      end Leave;

      function Most return Natural is (Peak);

      procedure Reset is
      begin
         Inside := 0;
         Peak := 0;
      end Reset;
   end Windows;

   --  The main loop's side: offers beats and collects their replies until the
   --  decider has ended, counting the replies that carried both lanes'
   --  targets and the targets each group had.
   type Group_Counts is array (Driver.Commands.Group_Id range 1 .. 2) of Natural;

   procedure Run_Main_Loop
     (Ended : not null access function return Boolean;
      Both  : out Natural;
      Moved : out Group_Counts)
   is
      O     : Driver.Observations.Observation;
      Took  : Boolean;
      Reply : Driver.Commands.Command;
      Beat  : Natural := 0;
   begin
      Both := 0;
      Moved := [others => 0];
      while not Ended.all loop
         Beat := Beat + 1;
         O.Beat := Driver.Clock.Beat (Beat);
         Driver.Beats.Offer (Driver.Clock.Beat (Beat), O, Driver.Commands.Hold, Took);
         if Took then
            Driver.Beats.Await (Reply);
            for G in Moved'Range loop
               if Driver.Commands.Has_Target (Reply, G) then
                  Moved (G) := Moved (G) + 1;
               end if;
            end loop;
            if Driver.Commands.Has_Target (Reply, 1) and then Driver.Commands.Has_Target (Reply, 2) then
               Both := Both + 1;
            end if;
         else
            delay 0.001;
         end if;
      end loop;
   end Run_Main_Loop;

   Steps : constant := 20;   --  the beats each lane moves its group in

   --  Two lanes, each moving its own group for Steps beats: no two windows are
   --  ever open at once, every beat a lane took carries its target, and the
   --  beats both lanes took carry both.
   procedure Two_Lanes_Share_The_Beats is
      Done : Boolean := False with Atomic;

      procedure Move (Lane : Positive) is
         B : Driver.Clock.Beat;
         C : Driver.Commands.Command;
      begin
         for K in 1 .. Steps loop
            Driver.Beats.Next (B);
            Windows.Enter;
            C := Driver.Commands.Hold;
            Driver.Commands.Set_Target (C, Driver.Commands.Group_Id (Lane), [1 => Real (K)]);
            delay 0.001;   --  a window that takes time, as a decider's does
            Windows.Leave;
            Driver.Beats.Send (C);
         end loop;
      end Move;

      procedure Both_Lanes is new Driver.Beats.At_Once (Move);

      task Decider;
      task body Decider is
      begin
         Both_Lanes (2);
         Done := True;
      exception
         when others =>
            Done := True;
      end Decider;

      function Ended return Boolean is (Done);

      Both  : Natural;
      Moved : Group_Counts;
   begin
      Windows.Reset;
      Run_Main_Loop (Ended'Access, Both, Moved);
      Check (Windows.Most = 1, "two lanes had their windows open at once");
      Check (Moved = [Steps, Steps], "a lane's targets did not reach the robot once for each beat it took:"
             & Moved (1)'Image & Moved (2)'Image);
      Check (Both > 0, "the two lanes never shared a beat");
   end Two_Lanes_Share_The_Beats;

   --  A lane that fails ends At_Once with its exception once the other lane
   --  has ended, and the main loop is answered every beat meanwhile.
   procedure A_Failed_Lane_Is_Raised_At_The_End is
      Done    : Boolean := False with Atomic;
      Raised  : Boolean := False with Atomic;
      Other   : Natural := 0 with Atomic;

      procedure Move (Lane : Positive) is
         B : Driver.Clock.Beat;
      begin
         for K in 1 .. Steps loop
            Driver.Beats.Next (B);
            if Lane = 2 and then K = 3 then
               raise Constraint_Error with "lane two fails";
            end if;
            if Lane = 1 then
               Other := K;
            end if;
            Driver.Beats.Send (Driver.Commands.Hold);
         end loop;
      end Move;

      procedure Both_Lanes is new Driver.Beats.At_Once (Move);

      task Decider;
      task body Decider is
      begin
         Both_Lanes (2);
         Done := True;
      exception
         when Constraint_Error =>
            Raised := True;
            Done := True;
         when others =>
            Done := True;
      end Decider;

      function Ended return Boolean is (Done);

      Both  : Natural;
      Moved : Group_Counts;
   begin
      Run_Main_Loop (Ended'Access, Both, Moved);
      Check (Raised, "the failure of a lane was not raised by At_Once");
      Check (Other = Steps, "At_Once returned before the lane that did not fail had ended:" & Other'Image);
   end A_Failed_Lane_Is_Raised_At_The_End;

   --  Two lanes may not move one group in a beat: the second one's Send fails.
   procedure Two_Lanes_May_Not_Move_One_Group is
      Done   : Boolean := False with Atomic;
      Raised : Boolean := False with Atomic;

      procedure Move (Lane : Positive) is
         pragma Unreferenced (Lane);
         B : Driver.Clock.Beat;
         C : Driver.Commands.Command;
      begin
         for K in 1 .. Steps loop
            Driver.Beats.Next (B);
            C := Driver.Commands.Hold;
            Driver.Commands.Set_Target (C, 1, [1 => Real (K)]);
            Driver.Beats.Send (C);
         end loop;
      end Move;

      procedure Both_Lanes is new Driver.Beats.At_Once (Move);

      task Decider;
      task body Decider is
      begin
         Both_Lanes (2);
         Done := True;
      exception
         when Program_Error =>
            Raised := True;
            Done := True;
         when others =>
            Done := True;
      end Decider;

      function Ended return Boolean is (Done);

      Both  : Natural;
      Moved : Group_Counts;
   begin
      Run_Main_Loop (Ended'Access, Both, Moved);
      Check (Raised, "two lanes moved one group in one beat and nothing failed");
   end Two_Lanes_May_Not_Move_One_Group;

   procedure Register is
   begin
      Driver.Tests.Register ("core.lanes", "lanes share the beats one window at a time, their targets merged",
                             Two_Lanes_Share_The_Beats'Access);
      Driver.Tests.Register ("core.lanes_failure", "a failed lane is raised by At_Once once the others have ended",
                             A_Failed_Lane_Is_Raised_At_The_End'Access);
      Driver.Tests.Register ("core.lanes_clash", "two lanes may not move one group in a beat",
                             Two_Lanes_May_Not_Move_One_Group'Access);
   end Register;

end Driver.Lanes_Tests;
