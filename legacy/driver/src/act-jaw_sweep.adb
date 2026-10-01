separate (Act)
procedure Jaw_Sweep (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm, K : Natural; Target : Long_Float; Max_Iter : Natural;
                     Sweep_Cam : Integer; Sweep : in out Bools; Steps : out Natural; Reading : out Long_Float) is
   Jaw : Floats;
   Prev : Long_Float := (if Selfmap.Has_Jaw (F, Arm, K) then Selfmap.Jaw_Of (F, Arm, K) else 0.0);
   Prev_Cams : Plug.Cam_Vectors.Vector := F.Cams;
   Still : Natural := 0;
   Cm : Plug.Cmd;
   --  "停住"只看读数和【这条臂自己那只眼】(手指就在它里面);别的眼里别的东西在动跟合爪无关
   --  (S1 2026-09-23 实测:等三台相机全静止,一次合爪 35 拍,官方一集只有 200 拍)。没有自己的眼就看全部
   Hc : constant Integer := (if Arm < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (Arm) else -1);
   function Own_Eye_Still return Boolean is
     (if Hc >= 0 and then Natural (Hc) < Natural (F.Cams.Length) and then Natural (Hc) < Natural (Prev_Cams.Length)
         and then Natural (Hc) < Natural (C.Map.Floors.Length)
      then Selfmap.Picture_Still (C.Map, Prev_Cams (Natural (Hc)), F.Cams (Natural (Hc)), Natural (Hc))
      else Selfmap.Pictures_Still (C.Map, Prev_Cams, F.Cams));
begin
   Steps := 0;
   Reading := Prev;
   if not Selfmap.Has_Jaw (F, Arm, K) then
      --  这一拍没有这个通道的读数:不推(不拿编的数当读数、也不拿它当别的通道的目标;09-30 原来补 1.0 = x5"1 = 张开")
      Put_Line ("[身] ✋ 第" & Codec.Img (Arm + 1) & " 只手第 " & Codec.Img (K + 1) & " 个抓握通道这一拍没有读数 ⇒ 不合不张");
      return;
   end if;
   --  只动点名的那一个抓握通道,其余保持它们此刻的读数(五指手:合一根不牵动另外四根)
   declare
      Rest : constant Floats := Selfmap.Jaw_All (F, Arm);
   begin
      for I in 0 .. Natural (Rest.Length) - 1 loop
         Jaw.Append (if I = K then Target else Rest (I));
      end loop;
   end;
   --  读数是命令的回声,"停住"只认画面:每台相机连着两拍不变
   for I in 1 .. Max_Iter loop
      Cm.Kind := Plug.Ee; Cm.Arm := Arm; Cm.Pose := F.EE (Arm); Cm.Jaw := Jaw;
      exit when not Plug.Act (L, Cm) or else not Plug.Sense (L, F);
      Steps := I;
      if Selfmap.Has_Jaw (F, Arm, K) then
         Reading := Selfmap.Jaw_Of (F, Arm, K);
      else
         Still := 0;   --  这一拍没读数:算不上"停住"
      end if;
      if Sweep_Cam >= 0 and then Natural (Sweep_Cam) < Natural (F.Cams.Length) and then Natural (Sweep_Cam) < Natural (C.Map.Floors.Length) then
         Sweep := Picture.Either (Sweep, Picture.Moved (Prev_Cams (Natural (Sweep_Cam)).Gray, F.Cams (Natural (Sweep_Cam)).Gray, C.Map.Floors (Natural (Sweep_Cam))));
      end if;
      if abs (Reading - Prev) <= C.Map.Jaw_Noise and then Own_Eye_Still then
         Still := Still + 1;
      else
         Still := 0;
      end if;
      Prev := Reading;
      Prev_Cams := F.Cams;
      exit when Still >= 2 and then I >= 3;
   end loop;
end Jaw_Sweep;
