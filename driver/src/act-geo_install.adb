separate (Act)
procedure Geo_Install (F : Plug.Frame; C : in out Context; Body_Path : String; Geo : Geom.Geo_Vectors.Vector; Board : Geom.Scene_Pt_Vectors.Vector;
                       Plane_Pt, Plane_N : Geom.V3; Plane_Rms : Long_Float; Ref : Plug.Cam; Keep_Tips : Boolean := False) is
   pragma Unreferenced (F);
begin
   C.Geo_Path := S (Body_Path & ".geo.json");
   C.Geo := Geo;
   if Keep_Tips and then Body_Path /= "" then
      --  前半段装回的:存的指尖、张口(碰桌面量的)、步幅(一条命令走多远 / 转多远,开机按阶梯量的)并进来 —— 同一次从零量的结果、同一个世界单位。
      --  09-28 S1A1:原来只并指尖和张口,步幅在记分那一集里整套重量,吃掉 122 拍(一集 200 拍)
      declare
         Old : Geom.Geo_Vectors.Vector;
         Note : String (1 .. 160);
      begin
         Geom.Load (To_String (C.Geo_Path), Old, Natural (C.Geo.Length), Note);
         for Cm in 0 .. Natural'Min (Natural (Old.Length), Natural (C.Geo.Length)) - 1 loop
            if not C.Geo (Cm).Fixed then
               declare
                  G : Geom.Cam_Geo := C.Geo (Cm);
               begin
                  if Old (Cm).Tip_Valid and then Old (Cm).Tip_Touch then
                     G.Tip := Old (Cm).Tip; G.Gap := Old (Cm).Gap; G.Tip_Valid := True; G.Tip_Touch := True;
                     G.Lobes := Old (Cm).Lobes; G.Tip_Sd := Old (Cm).Tip_Sd;   --  每一瓣的尖和截面(接触集的手)跟着指尖一起并回来
                  end if;
                  if Old (Cm).Stride > 0.0 then
                     G.Stride := Old (Cm).Stride;
                  end if;
                  if Old (Cm).Stride_Rot > 0.0 then
                     G.Stride_Rot := Old (Cm).Stride_Rot;
                  end if;
                  C.Geo.Replace_Element (Cm, G);
               end;
            end if;
         end loop;
      end;
   end if;
   C.Board := Board; C.Board_Seen.Clear; C.Seen_Above.Clear;   --  换了板:以前压之前看见的点按旧的面判的高低,作废
   C.Board_Pt := Plane_Pt; C.Board_N := Plane_N; C.Board_Rms := Plane_Rms;
   C.Board_Plane := not Board.Is_Empty;
   --  有板的面 ⇒ 东西躺的面就是它(同 Note_Support:有板时朝下顶住的点只和它对账、不换它),装上就登记;不等第一次朝下被顶住。
   --  09-28 S1A1:装回开机不碰桌面、碰指尖那几下又不记接触 ⇒ "碰过的面"一直空着,不动的眼看见剪刀时按指尖此刻的高度当面,
   --  剪刀被放到桌面上方 5 个单位,腕眼转了 1.7 弧度还没转到、顶到关节尽头。Touch_Fresh 不设:这一集里还没真碰过
   if C.Board_Plane then
      C.Touch_Pt := Plane_Pt; C.Touch_N := Plane_N; C.Touch_Valid := True;
   end if;
   C.Fixed_Ref := Ref.RGB; C.Fixed_Ref_W := Ref.W; C.Fixed_Ref_H := Ref.H;
   C.Fixed_Best := (others => <>);
   --  不动的眼核对用的细门按板定,和重标那一份同一个算法(Geom.Board_Rms):板上每个点(参考图里的像素)按标定的位姿投回去,门以内误差的中位 × 1.2
   for Cam in 0 .. Natural (C.Geo.Length) - 1 loop
      declare
         G : Geom.Cam_Geo := C.Geo (Cam);
      begin
         if G.Fixed and then G.Valid and then not C.Board.Is_Empty then
            declare
               Br : constant Long_Float := Geom.Board_Rms (G, C.Board, 3.0 * Long_Float'Max (1.0e-9, G.Rms));   --  解的时候的门(3 倍,协议;同核对)
            begin
               if Br > 0.0 then
                  Geo_Say ("不动的眼按板配得多细:板上 " & Codec.Img (Natural (C.Board.Length)) & " 个点投回去,误差中位 × 1.2 = " & Codec.Fmt (Br, 2)
                           & " px(解的时候的均方根 " & Codec.Fmt (G.Rms, 2) & " px)⇒ 核对的细门按它定");
                  G.Rms := Br;
                  C.Geo.Replace_Element (Cam, G);
               end if;
            end;
         end if;
      end;
   end loop;
   for Cam in 0 .. Natural (C.Geo.Length) - 1 loop
      declare
         G : constant Geom.Cam_Geo := C.Geo (Cam);
         A : constant Integer := Cam_Arm (C, Cam);
      begin
         if G.Fixed then
            Geo_Say ("第" & Codec.Img (Cam) & " 台相机(不长在手上):焦距 " & Codec.Fmt (G.F, 1) & " px、在世界 (" & Codec.Fmt (G.Pos (0), 3) & ", " & Codec.Fmt (G.Pos (1), 3) & ", "
                     & Codec.Fmt (G.Pos (2), 3) & ") 单位(开机前半段对齐量的)");
         elsif A >= 0 and then G.Valid then
            Geo_Say ("第" & Codec.Img (Cam) & " 台相机(长在第" & Codec.Img (Natural (A) + 1) & " 只手上):焦距 " & Codec.Fmt (G.F, 1) & " px(运动学量的)· 手的位姿就是它的位姿 · 指尖 "
                     & (if G.Tip_Valid and then G.Tip_Touch then "存的(碰桌面量过,离眼 " & Mm (Geom.Norm (G.Tip)) & ")" else "待碰桌面量"));
         end if;
      end;
   end loop;
   Geo_Say ("标定板 " & Codec.Img (Natural (C.Board.Length)) & " 个点(开机前半段三角出、配进不动的眼的)· 桌面 = 世界 z = 0、离散 " & Codec.Fmt (C.Board_Rms, 4)
            & " 单位(长度单位 = 第一只手运动学的单位)");
   if not C.Geo.Is_Empty then
      Geom.Save (To_String (C.Geo_Path), C.Geo);
   end if;
end Geo_Install;
