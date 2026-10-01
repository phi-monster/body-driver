separate (Act)
procedure Geo_Away (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Step_Limit : Natural; Amt : Long_Float;
                    Event : out Unbounded_String; Steps_Taken : out Natural; Beats : out Natural;
                    Gear : Unbounded_String := Null_Unbounded_String) is
   pragma Unreferenced (Amt);
   Beats0 : constant Natural := Plug.Steps (L);
   Dist : constant Long_Float := C.Geo_Came;
   Went : Long_Float;
   Blocked, Arrived : Boolean;
begin
   Steps_Taken := 0; Beats := 0;
   if Geom.Norm (C.Geo_Dir) <= 0.0 or else Dist <= 0.0 then
      Event := S ("refused: I have not walked toward it yet, so I do not know which way is away from it");
      return;
   end if;
   --  退回逼近开始的那一处:每一步走还差的全部(上限:反解够得到、脑说的档位),走满脑给的步数就停(Retrace)。
   --  被挡住 = 这一步自己停下、没到、少走的比这一段空走时多出 Blocked 的门(原来:实到不到要的一半)
   Retrace (L, C, F, Arm, Step_Limit, Gear, Went, Steps_Taken, Blocked, Arrived);
   Event := S ((if Blocked then "resist: going away from it along the line I came in on, my hand was stopped after " & Len (C, Went)
                elsif Arrived then "amount: arrived (I moved " & Len (C, Went) & " away from it, back to where I started coming in)"
                elsif Step_Limit > 0 and then Steps_Taken >= Step_Limit
                then "steps: I took the steps you asked for (I moved " & Len (C, Went) & " away from it along the line I came in on)"
                else "amount: stopped getting closer to where I started coming in (I moved " & Len (C, Went) & " away from it)"));
   Beats := Beats_Since (L, Beats0);
end Geo_Away;
