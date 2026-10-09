with Ada.Containers.Vectors;
with Driver.Beats;
with Driver.Bytes;
with Driver.Recording;
with Driver.Services;

package body Driver.Apart is

   package Message_Vectors is new Ada.Containers.Vectors (Positive, Message);

   procedure Note (Kind : Driver.Recording.Record_Kind) is
   begin
      Driver.Recording.Write_Shared (Kind, Driver.Bytes.To_Bytes (""));
   end Note;

   --  A part the estimator gives the models is a step for the services'
   --  replies (Driver.Services.Step_Begins), begun with its record.
   procedure Take_Step is
      Place : Positive;
   begin
      Driver.Recording.Write_Shared (Driver.Recording.Taken_In, Driver.Bytes.To_Bytes (""), Place);
      Driver.Services.Step_Begins (Place);
   end Take_Step;

   --  Home: the main loop has the models. To_Compute: they went apart and the
   --  estimates are next. Computing: the estimator computes them. Taking_In:
   --  it gives the models the rest of a message and what was kept back.
   type Phase_Kind is (Home, To_Compute, Computing, Taking_In);
   type Work_Kind is (Compute_Now, Rest_Of, Kept_Back, Done);

   protected Backlog is
      procedure Go_Apart;
      procedure Go_Apart_With (M : Message);
      procedure Arrive (Held : out Boolean; Back : out Boolean);
      procedure Append (M : Message);
      entry Settle;
      entry Next_Work (W : out Work_Kind; M : out Message);
      procedure Computed;
      procedure Due_Again (M : Message);
      function Apart return Boolean;
   private
      Phase    : Phase_Kind := Home;
      Queue    : Message_Vectors.Vector;
      Has_Rest : Boolean := False;   --  the rest of Rest_Msg is still to be given
      Rest_Msg : Message;
      Mark     : Natural := 0;       --  messages kept back when the last answer went
   end Backlog;

   protected body Backlog is

      procedure Go_Apart is
      begin
         Phase := To_Compute;
         Has_Rest := False;
         Mark := 0;
      end Go_Apart;

      procedure Go_Apart_With (M : Message) is
      begin
         Phase := To_Compute;
         Has_Rest := True;
         Rest_Msg := M;
         Mark := 0;
      end Go_Apart_With;

      procedure Arrive (Held : out Boolean; Back : out Boolean) is
      begin
         Back := Phase = Taking_In and then Queue.Is_Empty and then not Has_Rest and then Next_Work'Count > 0;
         if Back then
            Phase := Home;   --  the estimator, waiting in Next_Work, is told Done
         end if;
         Held := Phase /= Home;
      end Arrive;

      procedure Append (M : Message) is
      begin
         Queue.Append (M);
      end Append;

      --  An answer waits until fewer messages are kept back than at the last
      --  answer, or until the estimator has given the models everything and
      --  waits: then the next message finds it waiting and the models come
      --  back (an estimator busy with the last message whenever the next one
      --  came would keep them apart for good).
      entry Settle
        when Phase /= Taking_In or else Natural (Queue.Length) < Mark
             or else (Queue.Is_Empty and then not Has_Rest and then Next_Work'Count > 0) is
      begin
         Mark := Natural (Queue.Length);
      end Settle;

      entry Next_Work (W : out Work_Kind; M : out Message)
        when Phase = To_Compute or else Phase = Home
             or else (Phase = Taking_In and then (Has_Rest or else not Queue.Is_Empty)) is
      begin
         case Phase is
            when To_Compute =>
               W := Compute_Now;
               Phase := Computing;
            when Home =>
               W := Done;
            when others =>
               if Has_Rest then
                  W := Rest_Of;
                  M := Rest_Msg;
                  Has_Rest := False;
               else
                  W := Kept_Back;
                  M := Queue.First_Element;
                  Queue.Delete_First;
               end if;
         end case;
      end Next_Work;

      procedure Computed is
      begin
         Phase := Taking_In;
      end Computed;

      procedure Due_Again (M : Message) is
      begin
         Phase := To_Compute;
         Has_Rest := True;
         Rest_Msg := M;
      end Due_Again;

      function Apart return Boolean is (Phase /= Home);

   end Backlog;

   task Estimator is
      entry Start;
   end Estimator;

   --  Each part of a message goes into the recording (kind A) before the
   --  models are given it, after the message's own record, which the main
   --  loop wrote before keeping it back.
   task body Estimator is
      W : Work_Kind;
      M : Message;
   begin
      loop
         select
            accept Start;
         or
            terminate;
         end select;
         loop
            Backlog.Next_Work (W, M);
            exit when W = Done;
            case W is
               when Compute_Now =>
                  Compute;
                  Backlog.Computed;
               when Rest_Of =>
                  Take_Step;
                  Rest (M);
               when Kept_Back =>
                  Take_Step;
                  Robot_Part (M);
                  if Due then
                     Backlog.Due_Again (M);
                  else
                     Rest (M);
                  end if;
               when Done =>
                  null;
            end case;
         end loop;
      end loop;
   exception
      when E : others =>
         Failed (E);
   end Estimator;

   procedure Arrive (Held : out Boolean) is
      Back : Boolean;
   begin
      Backlog.Arrive (Held, Back);
      if Back then
         Note (Driver.Recording.Estimates_Back);
         Driver.Beats.Estimates_Adopted;
      end if;
   end Arrive;

   procedure Hold_Back (M : Message) is
   begin
      Backlog.Append (M);
      Backlog.Settle;
   end Hold_Back;

   procedure Take_In (M : Message; Went_Apart : out Boolean) is
   begin
      Robot_Part (M);
      Went_Apart := Due;
      if Went_Apart then
         Note (Driver.Recording.Estimates_Apart);
         Backlog.Go_Apart_With (M);
         Estimator.Start;
      else
         Rest (M);
      end if;
   end Take_In;

   procedure After_Reply is
   begin
      if Due then
         Note (Driver.Recording.Estimates_Apart);
         Backlog.Go_Apart;
         Estimator.Start;
      end if;
   end After_Reply;

   function Is_Apart return Boolean is (Backlog.Apart);

   --  Replay: one task, so no protection.

   Replaying_Apart : Boolean := False;
   Replay_Queue    : Message_Vectors.Vector;
   Replay_Has_Rest : Boolean := False;
   Replay_Rest_Msg : Message;

   procedure Replay_Take (M : Message) is
   begin
      Robot_Part (M);
      if Due then
         Compute;
         Replay_Rest_Msg := M;
         Replay_Has_Rest := True;
      else
         Rest (M);
      end if;
   end Replay_Take;

   --  Computed in place, the rest of a message followed the estimates at once;
   --  it is given here, when the next record that needs it comes.
   procedure Flush_In_Place is
   begin
      if Replay_Has_Rest and then not Replaying_Apart then
         Replay_Has_Rest := False;
         Rest (Replay_Rest_Msg);
      end if;
   end Flush_In_Place;

   procedure Replay_Message (M : Message) is
   begin
      if Replaying_Apart then
         Replay_Queue.Append (M);
      else
         Flush_In_Place;
         Replay_Take (M);
      end if;
   end Replay_Message;

   procedure Replay_Decider is
   begin
      Flush_In_Place;
   end Replay_Decider;

   procedure Replay_Apart is
   begin
      Replaying_Apart := True;
   end Replay_Apart;

   procedure Replay_Taken_In (Ok : out Boolean) is
   begin
      Ok := True;
      if Replay_Has_Rest then
         Replay_Has_Rest := False;
         Rest (Replay_Rest_Msg);
      elsif not Replay_Queue.Is_Empty then
         declare
            M : constant Message := Replay_Queue.First_Element;
         begin
            Replay_Queue.Delete_First;
            Replay_Take (M);
         end;
      else
         Ok := False;
      end if;
   end Replay_Taken_In;

   procedure Replay_Back (Ok : out Boolean) is
   begin
      Ok := not Replay_Has_Rest and then Replay_Queue.Is_Empty;
      Replaying_Apart := False;
   end Replay_Back;

   procedure Replay_End is
   begin
      Flush_In_Place;
   end Replay_End;

end Driver.Apart;
