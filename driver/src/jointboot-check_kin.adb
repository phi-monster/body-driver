separate (Jointboot)
procedure Check_Kin (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; K : Kin_Store; Host : String; Port : Natural;
                     Ok : out Boolean; Note : out Unbounded_String) is
   Gs : Ints;
   Qs : Plug.Floats_Vectors.Vector;
   Tol : Long_Float := Long_Float'Last;
   Moved : Long_Float := 0.0;
   function Med (D : Floats) return Long_Float is
      package Sorting is new F64_Vectors.Generic_Sorting;
      X : Floats := D;
   begin
      if X.Is_Empty then
         return -1.0;
      end if;
      Sorting.Sort (X);
      return X (Natural (X.Length) / 2);
   end Med;
begin
   Ok := False; Note := Null_Unbounded_String;
   --  存的每只手:读数组、眼都得在这具身体上
   for A in 0 .. Natural (K.Worlds.Length) - 1 loop
      if K.Worlds (A).Valid then
         if K.Worlds (A).Group >= Natural (F.Joints.Length) or else K.Eyes (A) < 0 or else Natural (K.Eyes (A)) >= Natural (F.Cams.Length) then
            Note := To_Unbounded_String ("第" & Codec.Img (A + 1) & " 只手的读数组 / 眼这具身体上没有");
            return;
         end if;
         declare
            G : constant Natural := K.Worlds (A).Group;
            Q0 : constant Floats := K.Worlds (A).Model.Q0;
         begin
            for J in 0 .. Natural'Min (Natural (Q0.Length), Natural (F.Joints (G).Length)) - 1 loop
               declare
                  Dq : constant Long_Float := abs (Q0 (J) - F.Joints (G) (J));
               begin
                  Moved := Long_Float'Max (Moved, Dq);
                  if Dq > 0.0 then
                     Tol := Long_Float'Min (Tol, Third * Dq);   --  到了 = 差不到这一下要走的三分之一(比例,同开机自检)
                  end if;
               end;
            end loop;
            Gs.Append (G); Qs.Append (Q0);
         end;
      end if;
   end loop;
   --  回到存的参照读数(已经在那儿 = 差不过关节读数的静止噪声,不动)
   if Moved > 3.0 * M.Joint_Noise then   --  3 倍静止噪声(统计常数)
      declare
         Dl : Table.Vec;
         Fr : Natural;
         Okg : Boolean;
      begin
         Selfmap.Go (L, M, 0, [others => 0.0], F64_Vectors.Empty_Vector, F, Dl, Fr, Okg, Groups => Gs, Qs => Qs, Tol => (if Tol < Long_Float'Last then Tol else 0.0));
         if not Okg then
            Note := To_Unbounded_String ("回存的参照读数时线断了 / 走不到");
            return;
         end if;
      end;
   end if;
   declare
      Ok2 : Boolean;
   begin
      Selfmap.Idle (L, F, 2, Ok2);   --  画面比读数晚 1 拍:停两拍再拍(次数)
      if not Ok2 then
         Note := To_Unbounded_String ("停稳时线断了");
         return;
      end if;
   end;
   Ok := True;
   Append (Note, "回到存的参照读数(最多差 " & Codec.Fmt (Moved, 6) & ")");
   for A in 0 .. Natural (K.Worlds.Length) - 1 loop
      if K.Worlds (A).Valid then
         declare
            Disp : Floats;
            Err : Unbounded_String;
            E : constant Natural := Natural (K.Eyes (A));
         begin
            View_Shift (Host, Port, K.Ds (A).Imgs (0), F.Cams (E), Disp, Err);
            Append (Note, " · 第" & Codec.Img (A + 1) & " 只手的眼(第" & Codec.Img (E) & " 台)和存的图配上 " & Codec.Img (Natural (Disp.Length)) & " 个点、位移中位 "
                    & Codec.Fmt (Med (Disp), 2) & " px ⇒ " & (if Same_View (Disp) then "没动" else "动了"));
            Ok := Ok and then Same_View (Disp);
         end;
      end if;
   end loop;
   if K.World_Cam >= 0 then
      if Natural (K.World_Cam) >= Natural (F.Cams.Length) then
         Note := Note & " · 存的不动的眼这具身体上没有";
         Ok := False;
      else
         declare
            Disp : Floats;
            Err : Unbounded_String;
         begin
            View_Shift (Host, Port, K.Ds (0).World_Img, F.Cams (Natural (K.World_Cam)), Disp, Err);
            Append (Note, " · 不动的眼(第" & Integer'Image (K.World_Cam) & " 台)和存的图配上 " & Codec.Img (Natural (Disp.Length)) & " 个点、位移中位 "
                    & Codec.Fmt (Med (Disp), 2) & " px ⇒ " & (if Same_View (Disp) then "没挪" else "挪了"));
            Ok := Ok and then Same_View (Disp);
         end;
      end if;
   end if;
end Check_Kin;
