separate (Act)
procedure Geo_Boot_Support (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context) is
   Down : constant Geom.V3 := [-Protocol_Up (0), -Protocol_Up (1), -Protocol_Up (2)];
   S_Known : Long_Float := 0.0;   --  这次开机头一瓣朝下那一下视线交面离眼多远(世界单位):别的瓣、别的手先一条命令下到按它算的高度

   procedure Go_Back (A : Natural; To : Plug.Arm_Pose) is
      Cur : constant Plug.Arm_Pose := F.EE (A);
      Mok : Boolean;
   begin
      Geo_Move (L, C, F, A, [To (0) - Cur (0), To (1) - Cur (1), To (2) - Cur (2)], Mok);
   end Go_Back;

   --  有板的面:每一瓣换倾角碰几下,量指尖
   procedure Touch_Tips (A, Hc : Natural) is separate;

   --  几只手同时碰(2026-09-28 PLAN ⑧ (g)):每只手一个任务照原样做它那一段(Touch_Tips),按拍对齐(Lockstep:同一时刻只有一个线程在跑);
   --  主线程每拍把几只手的目标合成一条关节命令发出去(Plug.Lock_Beat)。V1B50 两只手一只一只碰用了 975 拍
   procedure Touch_Tips_Together (Arms, Cams : Geom.Nat_Vectors.Vector) is separate;
   Tip_Arms, Tip_Cams : Geom.Nat_Vectors.Vector;   --  要碰桌面量指尖的手、它们各自的眼
begin
   for A in 0 .. C.Map.Arms - 1 loop
      declare
         Hc : constant Integer := (if A < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (A) else -1);
         Have : constant Boolean := Hc >= 0 and then Natural (Hc) < Natural (C.Geo.Length) and then Natural (Hc) < Natural (F.Cams.Length) and then A < Natural (F.EE.Length)
           and then C.Geo (Natural (Hc)).Valid and then C.Geo (Natural (Hc)).F > 0.0;
      begin
         if Plug.Reset_Pending (L) and then Plug.Take_Reset (L) then
            Geo_Say ("对方复位(新的一集)⇒ 手回了原处,接着摸面");
         end if;
         if not Have then
            Geo_Say ("第" & Codec.Img (A + 1) & " 只手:眼的朝向没量 ⇒ 这只手先不去摸它下面的面");
         elsif C.Board_Plane and then C.Geo (Natural (Hc)).Tip_Valid and then C.Geo (Natural (Hc)).Tip_Touch then
            --  缺什么才量什么:指尖是碰桌面量过的(几何文件里存着)、东西躺的面是板的(随板装回)⇒ 这回不碰
            --  (X5C4 2026-09-26:装回身体干活,开机每瓣碰一次用掉 900 多拍,官方一集只有 200 步)
            Geo_Say ("第" & Codec.Img (A + 1) & " 只手:指尖是碰桌面量过的(离眼 " & Mm (Geom.Norm (C.Geo (Natural (Hc)).Tip)) & "、张口 " & Mm (C.Geo (Natural (Hc)).Gap)
                     & "),桌面是板的 ⇒ 这回不碰");
         elsif C.Board_Plane then
            if A < Lockstep.Max_Hands then
               Tip_Arms.Append (A); Tip_Cams.Append (Natural (Hc));
            else
               Touch_Tips (A, Natural (Hc));
            end if;
         else
            --  东西躺的面只有一种量法:开机前半段按板量(09-30 删了"没有板就碰一下、顶住点 = 面"那条后备 —— 一个量两种量法);
            --  没纹理的世界配不出板的点,照实说(PLAN ⑦:改成碰出来的点连面带指尖一起解,那时候就是唯一的量法)
            Geo_Say ("第" & Codec.Img (A + 1) & " 只手:没有标定板量出的面 ⇒ 量不了它下面的面,也就没法碰桌面量指尖(世界里配不出板的点?)");
         end if;
      end;
   end loop;
   if Natural (Tip_Arms.Length) = 1 then
      Touch_Tips (Tip_Arms (0), Tip_Cams (0));
   elsif Natural (Tip_Arms.Length) > 1 then
      Touch_Tips_Together (Tip_Arms, Tip_Cams);
   end if;
end Geo_Boot_Support;
