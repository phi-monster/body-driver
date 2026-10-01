package body Driver.Action.Monitor
  with SPARK_Mode
is

   function Start return Watch is ((Count => 0, Mark => Unknown, Owed => 0.0, Stalled => False));

   function Steps (W : Watch) return Natural is (W.Count);

   function Stalled (W : Watch) return Boolean is (W.Stalled);

   procedure Step (W : in out Watch; F : Facts) is
   begin
      if W.Count < Natural'Last then
         W.Count := W.Count + 1;
      end if;
      if not Known (F.Gap) then
         W.Mark := Unknown;
         W.Owed := 0.0;
         return;
      elsif not Known (W.Mark) then
         W.Mark := F.Gap;
         W.Owed := 0.0;
         return;
      end if;
      if F.Owed > 0.0 and then W.Owed < Real'Last - F.Owed then
         W.Owed := W.Owed + F.Owed;
      end if;
      declare
         Closed : constant Estimate := Difference (W.Mark, F.Gap);
      begin
         --  Two hypotheses: the motion closes the gap as owed, or not at all.
         --  Whichever the closing is first told apart from decides: away
         --  from none is progress, short of the owed is a stall.
         if Closed.Value > 0.0 and then Significant (Closed.Value, Closed.Sigma, Closed.Degrees_Of_Freedom) then
            W.Mark := F.Gap;
            W.Owed := 0.0;
            W.Stalled := False;
         elsif W.Owed > Closed.Value
           and then Significant (W.Owed - Closed.Value, Closed.Sigma, Closed.Degrees_Of_Freedom)
         then
            W.Stalled := True;
         end if;
      end;
   end Step;

   function Rose (E : Estimate) return Boolean is
     (Known (E) and then E.Value > 0.0 and then Significant (E.Value, E.Sigma, E.Degrees_Of_Freedom));

   --  The carried thing is gone from the hand: it fell behind it, or the
   --  closer that held it reached closed on nothing.
   function Dropped (F : Facts) return Boolean is
     (Rose (F.Left_Behind)
      or else (Known (F.Closed_Short)
               and then not Significant (F.Closed_Short.Value, F.Closed_Short.Sigma,
                                         F.Closed_Short.Degrees_Of_Freedom)));

   function Out_Of_Steps (W : Watch; F : Facts; Max_Steps : Natural) return Boolean is
     (F.Out_Of_Beats or else (Max_Steps > 0 and then W.Count >= Max_Steps));

   function Holds (W : Watch; F : Facts; E : Ending; Max_Steps : Natural) return Boolean is
     (case E is
         when Arrived  => False,
         when Touched  => F.Touch,
         when Stuck    => F.Blocked,
         when Slipped  => Dropped (F),
         when Lost     => not F.Seen,
         when Free     => Rose (F.Height_Gain),
         when Settled  => F.Still and then not F.Commanded,
         when Stalled  => W.Stalled,
         when Timeout  => Out_Of_Steps (W, F, Max_Steps),
         when Refused  => False);

   --  A fact that makes going on pointless, whether or not it was wanted.
   function Ends_Anyway (W : Watch; F : Facts; E : Ending; Wanted : Ending_Set; Max_Steps : Natural) return Boolean
   is
     (case E is
         when Stuck   => F.Blocked or else (F.Exhausted and then not Wanted (Settled)),
         when Slipped => Dropped (F),
         when Lost    => not F.Followable,
         when Stalled => W.Stalled,
         when Timeout => Out_Of_Steps (W, F, Max_Steps),
         when others  => False);

   function Fired (W : Watch; F : Facts; Wanted : Ending_Set; Max_Steps : Natural) return Boolean is
     ((for some E in Ending => Wanted (E) and then Holds (W, F, E, Max_Steps))
      or else (for some E in Ending => Ends_Anyway (W, F, E, Wanted, Max_Steps)));

   function Ending_Of (W : Watch; F : Facts; Wanted : Ending_Set; Max_Steps : Natural) return Ending is
   begin
      for E in Ending loop
         if Wanted (E) and then Holds (W, F, E, Max_Steps) then
            return E;
         end if;
      end loop;
      for E in Ending loop
         if Ends_Anyway (W, F, E, Wanted, Max_Steps) then
            return E;
         end if;
      end loop;
      return Timeout;
   end Ending_Of;

end Driver.Action.Monitor;
