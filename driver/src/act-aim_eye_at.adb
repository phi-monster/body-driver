separate (Act)
procedure Aim_Eye_At (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Fixed_Cam : Natural;
                      U, V : Long_Float; Amt : Long_Float; Event : out Unbounded_String; Ok : out Boolean) is
   G0 : constant Geom.Cam_Geo := Geo_Of (C, Fixed_Cam);
   Ray : constant Geom.V3 := Geom.Ray_Fixed (G0, U, V);
   P0 : constant Geom.V3 := (if C.Touch_Valid then C.Touch_Pt else Tip_World (C, Arm, F.EE (Arm)));
   Nn : constant Geom.V3 := Up_Dir (C);
   Hok : Boolean;
   P : constant Geom.V3 := Geom.Hit_Plane (G0.Pos, Ray, P0, Nn, Hok);
   Steps : Natural;
begin
   Ok := False;
   Event := Null_Unbounded_String;
   if not G0.Fixed then
      Event := S ("refused: the eye that sees it does not know where it sits in the world - I have not measured that yet");
      return;
   end if;
   if not Hok then
      Event := S ("lost: the still eye's line of sight to it does not meet the surface I know");
      return;
   end if;
   declare
      Hp : constant Plug.Arm_Pose := F.EE (Arm);
      D : Geom.V3 := [P (0) - Hp (0), P (1) - Hp (1), P (2) - Hp (2)];
      Ln : constant Long_Float := Geom.Norm (D);
   begin
      Geo_Say ("不动的眼说它在 (" & Mm (P (0)) & "," & Mm (P (1)) & "," & Mm (P (2)) & ")"
               & (if C.Touch_Valid then "(视线落到我碰过的那个面上)" else "(还没碰过任何面,先按指尖此刻的高度算)")
               & ",离第" & Codec.Img (Arm + 1) & " 只手 " & Mm (Ln) & " ⇒ 把这只手的眼转向它");
      if Ln <= 0.0 then
         return;
      end if;
      D := [D (0) / Ln, D (1) / Ln, D (2) / Ln];
      Geo_Turn (L, C, F, Arm, D, Amt, Event, Steps);
      Ok := Index (Event, "amount: arrived") > 0;
   end;
end Aim_Eye_At;
