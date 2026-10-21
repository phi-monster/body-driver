with Ada.Strings.Unbounded;
with Driver.Log;

package body Driver.Twins is

   use Ada.Strings.Unbounded;
   use type Driver.Observations.Group_Id;

   procedure Order
     (W    : in out Watch;
      L    : Driver.Observations.Layout;
      O    : in out Driver.Observations.Observation;
      Sent : Driver.Commands.Command)
   is
   begin
      while W.Pairs.Last_Index < L.Groups.Last_Index loop
         W.Pairs.Append (Pair_State'(others => <>));
      end loop;
      for G in L.Groups.First_Index .. L.Groups.Last_Index loop
         if Length (L.Groups (G).Echo_Keys) > 0 and then G <= O.Readings.Last_Index and then G <= O.Echoes.Last_Index then
            declare
               P      : Pair_State := W.Pairs (G);
               First  : constant Real_Array := O.Readings (G);
               Second : constant Real_Array := O.Echoes (G);
               Size   : constant Positive := L.Groups (G).Size;
               Whole  : constant Boolean :=
                 First'Length = Size and then Second'Length = Size and then Driver.Commands.Has_Target (Sent, G)
                 and then Driver.Commands.Target (Sent, G)'Length = Size;
            begin
               if Whole and then not P.Decided then
                  if P.Seen then
                     declare
                        Commanded : constant Boolean := Driver.Commands.Target (Sent, G) /= P.Last_Sent.Element;
                        First_Moved  : constant Boolean := First /= P.Last_First.Element;
                        Second_Moved : constant Boolean := Second /= P.Last_Second.Element;
                     begin
                        --  An echo changes with the command and only with it.
                        P.First_No_Echo := P.First_No_Echo or else First_Moved /= Commanded;
                        P.Second_No_Echo := P.Second_No_Echo or else Second_Moved /= Commanded;
                     end;
                     if P.First_No_Echo /= P.Second_No_Echo then
                        P.Decided := True;
                        P.Turned := P.Second_No_Echo;
                        Driver.Log.Line
                          (Driver.Log.Core,
                           "group" & G'Image & ": its readings are at "
                           & To_String (if P.Turned then L.Groups (G).Echo_Path else L.Groups (G).Path)
                           & ", the robot's echo of its command at "
                           & To_String (if P.Turned then L.Groups (G).Path else L.Groups (G).Echo_Path)
                           & ": the echo changed with the command and only with it, the readings did not");
                     end if;
                  end if;
                  P.Seen := True;
                  P.Last_First := Array_Holders.To_Holder (First);
                  P.Last_Second := Array_Holders.To_Holder (Second);
                  P.Last_Sent := Array_Holders.To_Holder (Driver.Commands.Target (Sent, G));
                  W.Pairs.Replace_Element (G, P);
                  if P.Decided and then P.Turned then
                     O.Turned.Replace_Element (G, True);
                  end if;
               end if;
               if W.Pairs (G).Turned then
                  O.Readings.Replace_Element (G, Second);
                  O.Echoes.Replace_Element (G, First);
                  if G <= O.Precisions.Last_Index and then G <= O.Echo_Precisions.Last_Index then
                     declare
                        Of_First : constant Natural := O.Precisions (G);
                     begin
                        O.Precisions.Replace_Element (G, O.Echo_Precisions (G));
                        O.Echo_Precisions.Replace_Element (G, Of_First);
                     end;
                  end if;
               end if;
            end;
         end if;
      end loop;
   end Order;

   function Reading_Is_Echo_Path (W : Watch; G : Driver.Observations.Group_Id) return Boolean is
     (G <= W.Pairs.Last_Index and then W.Pairs (G).Turned);

   function Told_Apart (W : Watch; G : Driver.Observations.Group_Id) return Boolean is
     (G <= W.Pairs.Last_Index and then W.Pairs (G).Decided);

end Driver.Twins;
