with Contact.Wrench;
separate (Act)
procedure Let_Go_If_It_Rests (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; W : Want; Qty : String;
                           Fallback_Name : Unbounded_String; Event : out Unbounded_String; Steps_Taken : in out Natural) is
   Up : constant Geom.V3 := Up_Dir (C);
   Name : constant Unbounded_String := (if W.Thing >= 1 and then W.Thing <= Natural (C.Items.Length) then Item_Name (C, W.Thing) else Fallback_Name);
   Sc : Contact.Qty.Scene;
   Shape, Rest : Contact.V3_Vectors.Vector;
begin
   Want_Scene (C, F, W, Integer (Arm), Sc);
   Thing_Shape (C, F, Integer (Arm), Name, Shape, Rest);
   if Shape.Is_Empty or else not Sc.Has_Bottom or else not Sc.Has_Center then
      Event := S ("touched: something stopped it on the way down (changing its " & Qty & "); I have no shape of it to tell whether it would stay up on its own, so I did not let go");
      return;
   end if;
   if Sc.Bottom > Stats.Z * Sc.Sd then
      Event := S ("touched: it was stopped with its bottom " & Len (C, Sc.Bottom) & " above the surface it lay on (my measurement of that height is good to "
                  & Len (C, Stats.Z * Sc.Sd) & ") - it is standing on something whose top I have not measured, so I cannot tell whether it would stay up; I did not let go");
      return;
   end if;
   declare
      Base : constant Contact.Wrench.Surface := Contact.Wrench.Base_Of (Shape, Up, C.Sil_Pitch);
      Stays : Boolean;
      Margin : Long_Float;
   begin
      Contact.Wrench.Rests (Base, Sc.Center, Up, Sc.Sd, 0.0, Stays, Margin);
      if not Stays then
         Event := S ("touched: it is down on the surface but would not stay up on its own - its centre is "
                     & (if Margin > Long_Float'First then Len (C, Margin) & (if Margin >= 0.0 then " inside" else " outside") & " the edge of what it stands on"
                        else "over a base with no area")
                     & " and I know where its centre is only to within " & Len (C, Stats.Z * Sc.Sd) & "; I did not let go");
         return;
      end if;
      declare
         Jk : constant Natural := Natural (Integer'Max (0, C.Wld.Held_Jaw));
         Hk : constant Zone.Hand := Hand_Of (C, Arm, Jk);
         Before : constant Geom.V3 := Sc.Center;
         Sd_Before : constant Long_Float := Sc.Sd;
         Steps_J : Natural;
         Reading : Long_Float;
         Mok, Got : Boolean;
         Hc : constant Integer := Hand_Eye_Of (C, Integer (Arm));
         Back : constant Long_Float := (if Hc >= 0 then Geo_Of (C, Natural (Hc)).Gap else 0.0);   --  手退开多远:一个张口(同接触集悬停离下手处那么远)
      begin
         if not Hk.Measured then
            Event := S ("touched: it is down on the surface and would stay up on its own, but I never measured which reading opens this grip, so I did not let go");
            return;
         end if;
         Move_Jaw (L, C, F, Arm, Hk.Open_Reading, Steps_J, Reading, Jk);
         C.Wld.Holding := False; C.Wld.Held_Arm := -1; C.Wld.Held_Jaw := -1; C.Wld.Held_Slot := -1;
         C.Held_Set_Valid := False;
         Memory.Set (C.Mem, "holding", "");
         Geo_Move (L, C, F, Arm, [Back * Up (0), Back * Up (1), Back * Up (2)], Mok);
         Steps_Taken := Steps_Taken + 1;
         Measure_Again (C, F, Arm, Name, Before, Got);
         if not Got then
            Event := S ("touched: I let go of it on the surface (it would stay up on its own: its centre is " & Len (C, Margin)
                        & " inside the edge of what it stands on); afterwards no eye of mine saw it whole, so I could not check that it stayed where I put it");
            return;
         end if;
         declare
            Sa : Contact.Qty.Scene;
            D : Long_Float;
         begin
            Want_Scene (C, F, W, Integer (Arm), Sa);
            D := Geom.Norm ([Sa.Center (0) - Before (0), Sa.Center (1) - Before (1), Sa.Center (2) - Before (2)]);
            if Contact.Qty.Moved_Off (Before, Sa.Center, Sd_Before, Sa.Sd) then
               Event := S ("slipped: I let go of it on the surface and it moved " & Len (C, D) & " from where I put it - more than my measurement can explain ("
                           & Len (C, Stats.Z * Sqrt (Sd_Before ** 2 + Sa.Sd ** 2)) & "), so it slid or fell after I let go");
            else
               Event := S ("touched: it lies on the surface where I put it - I let go and, measured again, it is " & Len (C, D) & " from there (within my measurement, "
                           & Len (C, Stats.Z * Sqrt (Sd_Before ** 2 + Sa.Sd ** 2)) & ")");
            end if;
         end;
      end;
   end;
end Let_Go_If_It_Rests;
