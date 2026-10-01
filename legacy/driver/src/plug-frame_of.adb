separate (Plug)
procedure Frame_Of (L : Link; F : in out Frame) is
begin
   F.Seq := L.Seq;
   for P of L.Lay.Joints loop
      F.Joints.Append (Nums_At (L, P));
   end loop;
   for P of L.Lay.EE loop
      declare
         A : constant Floats := Nums_At (L, P);
         Pose : Arm_Pose := [others => 0.0];
      begin
         if Natural (A.Length) = 7 then
            for I in 0 .. 6 loop
               Pose (I) := A (I);
            end loop;
         end if;
         F.Reported_EE.Append (Pose);   --  V1b 3c:身体报的位姿驱动不读(F.EE 由运动学按关节读数算,Pose_Hook 填)
      end;
   end loop;
   if L.Lay.Measured then
      --  开机按量认完(I1,10-01):第 A 条臂的抓握读数 = 它的合拢通道(Jaw 里从 Closing_First (A) 起 Closing_N (A) 组)按顺序接起来,
      --  一条臂一格(0 组 = 空格,下标照样占住:Selfmap.Jaw_Count 按臂取);有一组这一拍没读数 ⇒ 这条臂这一拍整格空(接一半下标就错位了);
      --  各臂之后是这些通道的回声
      declare
         Used : Natural := 0;
      begin
         for A in 0 .. L.Lay.N_Arms - 1 loop
            declare
               Cat : Floats;
               Whole : Boolean := True;
               N_A : constant Natural := (if A < Natural (L.Lay.Closing_N.Length) then Natural (L.Lay.Closing_N (A)) else 0);
               First : constant Natural := (if A < Natural (L.Lay.Closing_First.Length) then Natural (L.Lay.Closing_First (A)) else 0);
            begin
               for K in 0 .. N_A - 1 loop
                  if First + K < Natural (L.Lay.Jaw.Length) then
                     declare
                        V : constant Floats := Nums_At (L, L.Lay.Jaw (First + K));
                     begin
                        Whole := Whole and then not V.Is_Empty;
                        for X of V loop
                           Cat.Append (X);
                        end loop;
                     end;
                  end if;
               end loop;
               F.Jaw.Append (if Whole then Cat else F64_Vectors.Empty_Vector);
               Used := Natural'Max (Used, First + N_A);
            end;
         end loop;
         for I in Used .. Natural (L.Lay.Jaw.Length) - 1 loop
            F.Jaw.Append (Nums_At (L, L.Lay.Jaw (I)));
         end loop;
      end;
   else
      for P of L.Lay.Jaw loop
         declare
            A : constant Floats := Nums_At (L, P);
         begin
            F.Jaw.Append (A);   --  整组留下,不再只取 A (0)
         end;
      end loop;
   end if;
   for P of L.Lay.Groups loop
      F.Groups.Append (Nums_At (L, P));   --  身体报的每一组数(I1;开机按量认组用)
   end loop;
   declare
      Ins : constant Integer := Key (L.Last, L.Last_Obs, "instruction");
   begin
      if Ins >= 0 then
         F.Instruction := To_Unbounded_String (Text (L.Last, Ins));
      end if;
   end;
   for Ci in 0 .. Natural (L.Lay.Cams.Length) - 1 loop
      declare
         N : constant Integer := Layout.Find (L.Last, L.Last_Obs, L.Lay.Cams (Ci));
         W, H : Natural;
         C : Cam;   --  没收到这台的画面 ⇒ 这一格就是占位(W = H = 0、缓冲空),下标照样占住
         First, Len : Natural;
      begin
         if Layout.Is_Image (L.Last, N, W, H) then
            Nd_Data (L.Last, N, First, Len);
            if Len >= W * H * 3 then
               C.W := W; C.H := H;
               --  预分配后按下标写(比逐字节 Append 快好几倍:一帧三台相机近两百万个像素)
               C.RGB := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (W * H * 3));
               C.Gray := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (W * H));
               for I in 0 .. W * H - 1 loop
                  declare
                     R : constant Natural := Natural (L.Last.Raw.Element (First + 3 * I));
                     G : constant Natural := Natural (L.Last.Raw.Element (First + 3 * I + 1));
                     B : constant Natural := Natural (L.Last.Raw.Element (First + 3 * I + 2));
                  begin
                     C.RGB.Replace_Element (3 * I, U8 (R)); C.RGB.Replace_Element (3 * I + 1, U8 (G)); C.RGB.Replace_Element (3 * I + 2, U8 (B));
                     C.Gray.Replace_Element (I, U8 ((R * 299 + G * 587 + B * 114) / 1000));
                  end;
               end loop;
               --  身体另外给的深度图、相机内参:认得出(Layout 按形状认,免得当成别的读数),但驱动不读(铁律 1,2026-09-26 owner:
               --  每个量只有一种量法 —— 远近、焦距都由身体自己量;以前"给了就用、没给就量"是两种量法)。Has_Depth / Has_K 永远是 False
            end if;
         end if;
         F.Cams.Append (C);
      end;
   end loop;
end Frame_Of;
