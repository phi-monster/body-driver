with Readings;
separate (Jointboot)
procedure Check_Kin (L : in out Plug.Link; F : in out Plug.Frame; M : Selfmap.Body_Map; K : Kin_Store; Host : String; Port : Natural;
                     Ok : out Boolean; Note : out Unbounded_String) is
   Gs : Ints;
   Qs : Plug.Floats_Vectors.Vector;
   Tol : Long_Float := Long_Float'Last;
   Moved : Long_Float := 0.0;
   use type Readings.View_Says;
   Any_Moved, Any_Same : Boolean := False;
   --  存的核对图读回来只有彩色:按插头同一个式子折成灰度(看得出看不出按灰度判)
   function Gray_Of (C : Plug.Cam) return Buf is
      G : Buf;
   begin
      if not C.Gray.Is_Empty then
         return C.Gray;
      end if;
      for I in 0 .. C.W * C.H - 1 loop
         exit when 3 * I + 2 >= Natural (C.RGB.Length);
         G.Append (U8 ((Natural (C.RGB (3 * I)) * 299 + Natural (C.RGB (3 * I + 1)) * 587 + Natural (C.RGB (3 * I + 2)) * 114) / 1000));
      end loop;
      return G;
   end Gray_Of;
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
   --  一只眼:存的图和此刻的图各自看不看得出(Readings.Can_Judge),看得出才问配点仪器;没动 / 看不出 / 动了(Readings.View_Verdict)
   procedure One_Eye (Stored, Now : Plug.Cam; Cam : Natural; What, Same_W, Moved_W : String) is
      Fl : constant Picture.Floor_Map := (if Cam < Natural (M.Floors.Length) then M.Floors (Cam) else Picture.Floor_Map'(others => <>));
      Ja : constant Boolean := Readings.Can_Judge (Gray_Of (Stored), Fl, Stored.W, Stored.H);
      Jb : constant Boolean := Readings.Can_Judge (Now.Gray, Fl, Now.W, Now.H);
      Disp : Floats;
      Err : Unbounded_String;
      V : Readings.View_Says;
   begin
      if Ja and then Jb then
         View_Shift (Host, Port, Stored, Now, Disp, Err);
      end if;
      V := Readings.View_Verdict (Ja, Jb, Natural (Disp.Length), Min_Inl, Same_View (Disp));
      Append (Note, " · " & What & "和存的图" & (if Ja and then Jb then "配上 " & Codec.Img (Natural (Disp.Length)) & " 个点、位移中位 " & Codec.Fmt (Med (Disp), 2) & " px"
                                                  else "(" & (if not Ja then "存的那张" else "此刻这张") & "没纹理,不问配点)")
              & " ⇒ " & (case V is when Readings.Same => Same_W, when Readings.Moved => Moved_W, when Readings.Unseen => "看不出"));
      Any_Moved := Any_Moved or else V = Readings.Moved;
      Any_Same := Any_Same or else V = Readings.Same;
   end One_Eye;
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
            E : constant Natural := Natural (K.Eyes (A));
         begin
            One_Eye (K.Ds (A).Imgs (0), F.Cams (E), E, "第" & Codec.Img (A + 1) & " 只手的眼(第" & Codec.Img (E) & " 台)", "没动", "动了");
         end;
      end if;
   end loop;
   if K.World_Cam >= 0 then
      if Natural (K.World_Cam) >= Natural (F.Cams.Length) then
         Note := Note & " · 存的不动的眼这具身体上没有";
         Ok := False;
      else
         One_Eye (K.Ds (0).World_Img, F.Cams (Natural (K.World_Cam)), Natural (K.World_Cam), "不动的眼(第" & Integer'Image (K.World_Cam) & " 台)", "没挪", "挪了");
      end if;
   end if;
   --  看不出 ≠ 动了(10-01,路 8 P8I 白桌白墙:配不上一个点被当成"动了",整份从零量,又量不出,退出):只有看得出、而且动了才从零量;
   --  哪只眼都看不出 ⇒ 照存的装回,照实说是凭读数回到了参照读数(身体真换了样子,这里看不出)
   Ok := Ok and then not Any_Moved;
   if Ok and then not Any_Same then
      Append (Note, " · 哪只眼都看不出(没纹理)⇒ 凭读数回到了存的参照读数照存的装回(身体真换了样子这里看不出)");
   end if;
end Check_Kin;
