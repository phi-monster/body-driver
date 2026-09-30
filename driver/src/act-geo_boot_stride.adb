separate (Act)
procedure Geo_Boot_Stride (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context) is
   Rungs : constant array (1 .. 3) of Long_Float := [4.0, 16.0, 64.0];
begin
   --  转动那一档:每次开机按运动学算(09-28 S1A2:阶梯只推到第三档 0.161 弧度就停,那是阶梯的顶,不是身体的顶 —— 碰指尖时一条命令
   --  转 1.36 弧度;转眼看剪刀要 17 步、67 拍,一集 200 拍)。不动胳膊、不占拍数;平移那一档照旧按阶梯量(脑的"一个单位"按它)
   for A in 0 .. C.Map.Arms - 1 loop
      declare
         Hc : constant Integer := (if A < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (A) else -1);
         Notch_R : constant Long_Float := (if A * Chan.Per_Arm + 3 < Natural (C.Map.Amp.Length) then C.Map.Amp (A * Chan.Per_Arm + 3) else 0.0);
      begin
         if Hc >= 0 and then Natural (Hc) < Natural (C.Geo.Length) and then A < Natural (F.EE.Length) then
            declare
               G : Geom.Cam_Geo := C.Geo (Natural (Hc));
               P0 : constant Plug.Arm_Pose := F.EE (A);
            begin
               G.Stride_Rot := Kin_Turn_Reach (A, P0, Notch_R, Geo_Base (C, A), Notch_R);
               C.Geo.Replace_Element (Natural (Hc), G);
               Geo_Say ("第" & Codec.Img (A + 1) & " 只手:一条命令转得到的最大一档 = " & Codec.Fmt (G.Stride_Rot, 3)
                        & " 弧度(按运动学在量到的关节限位里问反解,不动胳膊;一档转动 " & Codec.Fmt (Notch_R, 4) & " 弧度起翻倍)");
            end;
         end if;
      end;
   end loop;
   for A in 0 .. C.Map.Arms - 1 loop
      declare
         Hc : constant Integer := (if A < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (A) else -1);
         Amp : constant Long_Float := Geo_Base (C, A);
      begin
         if Hc >= 0 and then Natural (Hc) < Natural (C.Geo.Length) and then A < Natural (F.EE.Length) and then Amp > 0.0
           and then C.Geo (Natural (Hc)).Stride <= 0.0
         then
            declare
               G : Geom.Cam_Geo := C.Geo (Natural (Hc));
               Best : Long_Float := 0.0;
               Tried : Natural := 0;   --  真试过几档(对方复位打断时一档没试就不许下"走不了路"的结论:V1C/V1E 右臂就是这么被冤枉的)
               --  先往上探(离桌面远,安全);第一档往上就走不到(手在上限)⇒ 往下探。哪个方向走得到就记哪个
               Dirs : constant array (1 .. 2) of Long_Float := [1.0, -1.0];
            begin
               --  (09-27 起不再每停给不动的眼打指尖标记:不动的眼由开机前半段对齐量了,那些标记只给旧的标法用;一笔要合一次爪、约 11 拍)
               for Dir of Dirs loop
                  exit when Best > 0.0;
                  for R of Rungs loop
                     if Plug.Reset_Pending (L) and then Plug.Take_Reset (L) then
                        Geo_Say ("对方复位(新的一集)⇒ 手回了原处,步幅接着量");
                     end if;
                     declare
                        Ln : constant Long_Float := R * Amp;
                        Av : Table.Vec := Table.Zero_Vec;
                        Jaw : Floats;
                        Del : Table.Vec;
                        Ok : Boolean;
                        Got : Long_Float;
                     begin
                        Av (2) := Dir * Ln;
                        Step_Arm (L, C, F, A, Av, Jaw, Del, Ok, Geo_Settle => True);
                        Got := Dir * Del (2);
                        Tried := Tried + 1;
                        Geo_Say ("第" & Codec.Img (A + 1) & " 只手:一条命令往" & (if Dir > 0.0 then "上 " else "下 ") & Mm (Ln) & " ⇒ 实到 " & Mm (Got));
                        declare
                           Back : Table.Vec := Table.Zero_Vec;
                        begin
                           Back (0) := -Del (0); Back (1) := -Del (1); Back (2) := -Del (2);
                           Step_Arm (L, C, F, A, Back, Jaw, Del, Ok, Geo_Settle => True);
                        end;
                        exit when Got + Got < Ln;
                        Best := Ln;
                     end;
                  end loop;
               end loop;

               if Tried = 0 then
                  Geo_Say ("第" & Codec.Img (A + 1) & " 只手:步幅没量成(对方复位打断,一档都没试)⇒ 下次开机再量");
               else
                  G.Stride := Best;
                  C.Geo.Replace_Element (Natural (Hc), G);
                  Geom.Save (To_String (C.Geo_Path), C.Geo);
                  Geo_Say ("第" & Codec.Img (A + 1) & " 只手:一条命令走得到的最大一档 = " & Mm (Best)
                           & (if Best <= 0.0 then "(上下都走不到 ⇒ 这条臂走不了路)" else "") & ",存进几何文件");
               end if;
            end;
         end if;
      end;
   end loop;
end Geo_Boot_Stride;
