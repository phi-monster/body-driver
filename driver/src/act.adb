with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Codec;
with Draw;
with Flow;
with Monitor;
with Backup;
with Learned; use Learned;
with Lang;
with Exam;
package body Act is
   Sigma_Mult : constant Long_Float := 3.0;   --  鼓出来超过背景自己稳健 σ 的几倍才算一块(在真实深度图上验过:3 中,5 杀光);无量纲
   Track_Win : constant Long_Float := 0.10;   --  一步里任何被跟踪的点在画面里最多跑十分之一画幅(跟踪窗,比例,无量纲)
   Cap_Mult : constant Long_Float := 2.0;     --  一步命令上限 = 探针幅度(点在画面里跑过地板的那一档)的几倍(倍数,无量纲;EH:8 倍让阻尼当家,步子反而只剩探针的一倍)
   Step_Cap : constant := 60;                 --  一段最多几步(安全上限,不是策略)
   Unit_Reach : constant Table.Vec := [others => 1.0];

   function S (X : String) return Unbounded_String renames To_Unbounded_String;

   --  按(第几只手,第几个抓握通道)查握区。一条臂可以有好几个通道(五指手),所以不能拿臂号当下标。
   function Zone_Of (C : Context; Arm, Cam : Natural; K : Natural := 0) return Zone.Hand_Zone is
   begin
      for I in 0 .. Natural (C.Hands.Length) - 1 loop
         if C.Hands (I).Arm = Arm and then C.Hands (I).K = K
           and then Cam < Natural (C.Hands (I).Zones.Length)
         then
            return C.Hands (I).Zones (Cam);
         end if;
      end loop;
      return (others => <>);
   end Zone_Of;

   function Jaws_Of (C : Context; Arm : Natural) return Natural is
     (if Arm < Natural (C.Map.Jaws.Length) then Natural'Max (1, C.Map.Jaws (Arm)) else 1);

   --  按(第几只手,第几个抓握通道)取那只"手"。一条臂可以有好几个,拿臂号当下标是错的。
   function Hand_Of (C : Context; Arm : Natural; K : Natural := 0) return Zone.Hand is
   begin
      for I in 0 .. Natural (C.Hands.Length) - 1 loop
         if C.Hands (I).Arm = Arm and then C.Hands (I).K = K then
            return C.Hands (I);
         end if;
      end loop;
      return (others => <>);
   end Hand_Of;

   --  这个点属于哪个抓握通道(不是抓握通道带的就当 0 号)
   function Jaw_K_Of (Ck : Natural) return Natural is
     (if Ck >= Chan.Per_Arm then Ck - Chan.Per_Arm else 0);

   function Track_Idx (C : Context; Arm, Cam : Natural) return Natural is (Arm * C.Map.N_Cams + Cam);

   function Cam_Arm (C : Context; Cam : Natural) return Integer is
   begin
      for A in 0 .. Natural (C.Map.Cam_On_Arm.Length) - 1 loop
         if C.Map.Cam_On_Arm (A) = Integer (Cam) then
            return A;
         end if;
      end loop;
      return -1;
   end Cam_Arm;

   procedure Init_Tracks (C : in out Context) is
   begin
      C.Zones.Clear;
      for A in 0 .. C.Map.Arms - 1 loop
         for Cm in 0 .. C.Map.N_Cams - 1 loop
            declare
               Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cm);
               T : Zone_Track;
            begin
               T.Valid := Z.Valid;
               T.Cu := Z.Cu; T.Cv := Z.Cv; T.Z := Z.Depth;
               T.Au := Z.A.Cu; T.Av := Z.A.Cv; T.Bu := Z.B.Cu; T.Bv := Z.B.Cv;
               T.Has_Lobes := Z.Valid and then Z.N_Lobes >= 1;
               T.Known := Z.Valid;
               if Z.Valid then
                  T.Pieces (Chan.Per_Arm) := (True, Z.Cu, Z.Cv, (if Picture.Is_Nan (Z.Depth) then 0.0 else Z.Depth), Z.X0, Z.Y0, Z.X1, Z.Y1, Z.N_Lobes, Z.A.Cu, Z.A.Cv, Z.B.Cu, Z.B.Cv);
                  T.Pieces_Known (Chan.Per_Arm) := True;
               end if;
               --  开机每个通道推过一下:跟着动的那块 = 这个通道带的零件(不长在这只手上的相机里才算)
               if Cam_Arm (C, Cm) /= Integer (A) then
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     declare
                        Pi : constant Natural := (A * Chan.Per_Arm + K) * C.Map.N_Cams + Cm;
                     begin
                        if Pi < Natural (C.Map.Parts.Length) and then C.Map.Parts (Pi).Valid then
                           declare
                              P : constant Selfmap.Part := C.Map.Parts (Pi);
                           begin
                              T.Pieces (K) := (True, P.Cu, P.Cv, 0.0, P.X0, P.Y0, P.X1, P.Y1, 1, P.Cu, P.Cv, 0.0, 0.0);
                              T.Pieces_Known (K) := True;
                           end;
                        end if;
                     end;
                  end loop;
               end if;
               C.Zones.Append (T);
            end;
         end loop;
      end loop;
   end Init_Tracks;

   function Cut_Window (C : Context; Cam : Natural; F : Plug.Frame) return Long_Float is
      A : constant Integer := Cam_Arm (C, Cam);
   begin
      --  长在手上的相机斜看桌面:窗口 = 两指张幅按"指深 / 画面中位深"缩到画面深处("比这还大的不是能拿的东西");世界相机用画幅八分之一
      --  下面的 0.02 / 0.125 都是画幅的比例(无量纲):窗口的下限与上限
      if A >= 0 then
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
            Dp : Floats := F.Cams (Cam).Depth;
            Med : Long_Float;
         begin
            --  窗口至少要比正在跟的那块大半圈(倍数,无量纲),否则它一走近就被闭运算填平、只剩一圈边
            --  ⚠️ 试过"窗口跟着被跟的东西放大",错的:窗口一到画幅四分之一,桌面自己的起伏就盖过物体,
            --  深度那一路彻底切不出东西,颜色那一路接管、把墙缝和衣服切成上百块(EZ 逐步落图坐实)。窗口保持小。
            if Z.Valid and then Z.Span > 0.0 and then not Picture.Is_Nan (Z.Depth) and then F.Cams (Cam).Has_Depth then
               declare
                  Samp : Floats;
                  I : Natural := 0;
               begin
                  while I < Natural (Dp.Length) loop
                     if not Picture.Is_Nan (Dp (I)) and then Dp (I) > 0.0 then
                        Samp.Append (Dp (I));
                     end if;
                     I := I + 37;
                  end loop;
                  if Natural (Samp.Length) >= 16 then
                     Med := Picture.Quantile (Samp, 0.5);
                     if Med > 0.0 then
                        --  夹在画幅的 0.02 与 0.125 之间(比例,无量纲)
                        return Long_Float'Max (0.02, Long_Float'Min (0.125, Z.Span * (Z.Depth / Med) * 0.5));
                     end if;
                  end if;
               end;
            end if;
         end;
      end if;
      return 0.125;   --  世界相机:画幅八分之一(比例,无量纲)
   end Cut_Window;

   function Cut_Things_Raw (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Raw : Picture.Regions;
      Kept : Picture.Regions;
   begin
      if not F.Cams (Cam).Has_Depth then
         return Kept;
      end if;
      Raw := Picture.Cut (F.Cams (Cam).Depth, Cw, Ch, Cut_Window (C, Cam, F), Sigma_Mult);
      --  🔴 只有【深度上一个都看不出来】的时候才按颜色切:桌面木纹、瓷砖缝的颜色台阶比线还明显,
      --  在能看见东西的桌子上开着它,清单会从 7 条涨到 46 条,脑子被淹掉(ES 实测)。
      --  线板那种场合深度切不出任何东西,颜色这一路才接手。
      if Raw.Is_Empty then
      --  再按颜色切一遍,把深度上鼓不出来的细东西(线、缝、刀口)补进来:
      --  门槛 = 这台相机静止时颜色抖多少(量出来的)的几倍;贴画面边的是桌面/墙,丢掉(本仓既有规矩);
      --  已经被深度块盖住的不重复列
      declare
         --  门槛:比相机噪声大,也要比这张画面自己的纹理粗(木纹、布纹都会被纹理这一项吃掉);倍数无量纲
         Noise_C : constant Natural := (if Cam < Natural (C.Map.Pic_Floor.Length) and then C.Map.Pic_Floor (Cam) > 0
                                        then Natural (C.Map.Pic_Floor (Cam)) else 0);
         Floor_C : constant Long_Float :=
           Long_Float'Max (Long_Float (Noise_C) * 2.0 + 1.0, Picture.Texture_Level (F.Cams (Cam).RGB, Cw, Ch) * 4.0);
         Thin : constant Picture.Regions := Picture.Cut_Colour (F.Cams (Cam).RGB, Cw, Ch, Floor_C, Picture.Min_Pixels (Cw, Ch));
      begin
         for R of Thin loop
            declare
               Edge : constant Boolean := R.X0 = 0 or else R.Y0 = 0 or else R.X1 >= Cw - 1 or else R.Y1 >= Ch - 1;
               Covered : Boolean := False;
            begin
               for Q of Raw loop
                  if Picture.Inside (Q, R.Cu, R.Cv, Cw, Ch, 0.0) then
                     Covered := True;
                  end if;
               end loop;
               if not Edge and then not Covered then
                  declare
                     Q : Picture.Region := R;
                  begin
                     --  颜色切出来的块也要有远近:在它自己的位置上读一小片深度(窗口 = 它自己框的四分之一,比例,无量纲)
                     if F.Cams (Cam).Has_Depth then
                        declare
                           Zd : constant Long_Float := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, R.Cu, R.Cv,
                                                                          Long_Float'Max (0.005, Long_Float (R.X1 - R.X0) / Long_Float (Cw) * 0.25));
                        begin
                           if not Picture.Is_Nan (Zd) then
                              Q.Depth := Zd;
                           end if;
                        end;
                     end if;
                     Raw.Append (Q);
                  end;
               end if;
            end;
         end loop;
      end;
      end if;
      for R of Raw loop
         declare
            Mine : Boolean := False;
         begin
            for A in 0 .. C.Map.Arms - 1 loop
               if Zone.Is_Self (Zone_Of (C, A, Cam), R, Cw, Ch) then
                  Mine := True;
               end if;
            end loop;
            if not Mine then
               Kept.Append (R);
            end if;
         end;
      end loop;
      return Kept;
   end Cut_Things_Raw;

   --  同一帧、同一台相机只切一次(颜色切块要扫全图两遍,一步里被问好几次)
   function Cut_Things (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is
      Self : constant access Context := C'Unrestricted_Access;
   begin
      if C.Cut_Seq = F.Seq and then C.Cut_Cam = Integer (Cam) then
         return C.Cut_Regs;
      end if;
      Self.Cut_Regs := Cut_Things_Raw (C, F, Cam);
      Self.Cut_Seq := F.Seq;
      Self.Cut_Cam := Integer (Cam);
      return Self.Cut_Regs;
   end Cut_Things;

   function Cell_Of (C : Context; U, V : Long_Float) return Natural is
      Col : constant Natural := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (C.Cols) - 1.0, Long_Float'Floor (U * Long_Float (C.Cols)))));
      Row : constant Natural := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (C.Rows) - 1.0, Long_Float'Floor (V * Long_Float (C.Rows)))));
   begin
      return Row * C.Cols + Col + 1;
   end Cell_Of;

   --  ── 编号表:先我身上的,再世界里的 ──
   procedure Build_Listing (C : in out Context; F : Plug.Frame; Cam : Natural; RGB : in out Buf; Text : out Unbounded_String) is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      T : Unbounded_String;
      Named_U, Named_V : Long_Float := -1.0;
      Have_Named : Boolean := False;
      function Rel (U, V : Long_Float) return String is
         Half : constant String := (if U < 0.5 then "LEFT" else "RIGHT");
      begin
         if Have_Named then
            declare
               D : constant Long_Float := Sqrt (((U - Named_U) * Long_Float (C.Cols)) ** 2 + ((V - Named_V) * Long_Float (C.Rows)) ** 2);
            begin
               return ", " & Codec.Fmt (D, 1) & " cells from the thing you last named, in the " & Half & " half of the picture";
            end;
         end if;
         return ", in the " & Half & " half of the picture";
      end Rel;
      procedure Push (It : Item; Line : String; Col : Draw.Color; Thick : Natural) is
      begin
         C.Items.Append (It);
         if It.Located then
            Draw.Numbered_Box (RGB, Cw, Ch, It.X0, It.Y0, It.X1, It.Y1, Natural (C.Items.Length), Col, Thick);
         end if;
         Append (T, "  item " & Codec.Img (Natural (C.Items.Length)) & ": " & Line & ASCII.LF);
      end Push;
   begin
      C.Items.Clear;
      if C.Wld.Cams (Cam).Named >= 0 then
         declare
            Sl : constant World.Slot := World.Get (C.Wld, Cam, Natural (C.Wld.Cams (Cam).Named));
         begin
            if Sl.Seen then
               Have_Named := True;
               Named_U := (if Sl.Present then Sl.R.Cu else Sl.Shadow.Cu);
               Named_V := (if Sl.Present then Sl.R.Cv else Sl.Shadow.Cv);
            end if;
         end;
      end if;
      Append (T, "PIECES OF YOURSELF (measured just now: you moved one channel at a time and watched which part of the picture followed; you closed each hand on nothing and watched which pixels swept). Each is boxed and NUMBERED on the picture in orange:" & ASCII.LF);
      for A in 0 .. C.Map.Arms - 1 loop
       --  一条臂上量到几个抓握通道就列几组:两指手 1 组,五指手 5 组。代码里没有"一只手一个夹爪"这个假设。
       for Jk in 0 .. Jaws_Of (C, A) - 1 loop
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam, Jk);
            Tr : constant Zone_Track := (if Track_Idx (C, A, Cam) < Natural (C.Zones.Length) then C.Zones (Track_Idx (C, A, Cam)) else (others => <>));
            Own_Cam : constant Boolean := Cam_Arm (C, Cam) = Integer (A);
            Du : constant Long_Float := (if Own_Cam then 0.0 else Tr.Cu - Z.Cu);
            Dv : constant Long_Float := (if Own_Cam then 0.0 else Tr.Cv - Z.Cv);
            procedure Finger (Lb : Zone.Lobe; Which : Natural) is
               It : Item;
               --  这一瓣挪了多少:身体图给了各瓣位置就按瓣,否则整区平移
               Lu : constant Long_Float := (if Own_Cam then 0.0 elsif Tr.Has_Lobes then (if Which = 0 then Tr.Au else Tr.Bu) - Lb.Cu else Du);
               Lv : constant Long_Float := (if Own_Cam then 0.0 elsif Tr.Has_Lobes then (if Which = 0 then Tr.Av else Tr.Bv) - Lb.Cv else Dv);
            begin
               It.Kind := Finger; It.Arm := A; It.Which := Which; It.Jaw_K := Jk;
               if Z.Valid and then Lb.Valid and then Tr.Valid then
                  It.Located := True;
                  It.Cu := Lb.Cu + Lu; It.Cv := Lb.Cv + Lv;
                  It.X0 := Natural (Long_Float'Max (0.0, Long_Float (Lb.X0) + Lu * Long_Float (Cw)));
                  It.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), Long_Float (Lb.X1) + Lu * Long_Float (Cw))));
                  It.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Lb.Y0) + Lv * Long_Float (Ch)));
                  It.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), Long_Float (Lb.Y1) + Lv * Long_Float (Ch))));
                  It.Depth := Tr.Z; It.Count := Lb.Count;
                  Push (It, "a finger of arm " & Codec.Img (A + 1) & " (it moves when grip channel " & Codec.Img (Jk)
                        & " of that arm moves), now in cell " &
                        Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & Rel (It.Cu, It.Cv) &
                        (if Own_Cam or else Tr.Known then "" else " (placed from my joints; I have not yet looked at my hand here)"), Draw.Orange, 2);
               else
                  Push (It, "a finger of arm " & Codec.Img (A + 1) & " (grip channel " & Codec.Img (Jk)
                        & ") - NOT locatable in this picture right now, do not name it", Draw.Orange, 0);
               end if;
            end Finger;
            G : Item;
         begin
            Finger (Z.A, 0);
            Finger (Z.B, 1);
            G.Kind := Grip; G.Arm := A; G.Jaw_K := Jk;
            if Z.Valid and then Tr.Valid then
               G.Located := True;
               G.Cu := Tr.Cu; G.Cv := Tr.Cv; G.Depth := Tr.Z;
               G.X0 := Natural (Long_Float'Max (0.0, Long_Float (Z.X0) + Du * Long_Float (Cw)));
               G.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), Long_Float (Z.X1) + Du * Long_Float (Cw))));
               G.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Z.Y0) + Dv * Long_Float (Ch)));
               G.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), Long_Float (Z.Y1) + Dv * Long_Float (Ch))));
               Push (G, "grip " & Codec.Img (A + 1) & " - the space between the fingers of arm " & Codec.Img (A + 1) &
                     " (closing = grip close with grip_arm " & Codec.Img (A + 1) & "; a thing must sit in this box to be held), now in cell " &
                     Codec.Img (Cell_Of (C, G.Cu, G.Cv)) & Rel (G.Cu, G.Cv), Draw.Pink, 2);
            else
               Push (G, "grip " & Codec.Img (A + 1) & " (the space between the fingers of arm " & Codec.Img (A + 1) & ") - not locatable in this picture right now", Draw.Pink, 0);
            end if;
            --  全身零件:每个通道带的那一块(从那个关节往外的全部),位置按此刻位姿从身体图来
            if not Own_Cam and then Jk = 0 then
               for K in 0 .. Chan.Per_Arm - 1 loop
                  declare
                     Pc : constant Schema.Part_Pos := Tr.Pieces (K);
                     It : Item;
                  begin
                     if Pc.Valid then
                        It.Kind := Piece; It.Arm := A; It.Which := K; It.Located := True;
                        It.Cu := Pc.Cu; It.Cv := Pc.Cv; It.Depth := Pc.Z;
                        It.X0 := Pc.X0; It.Y0 := Pc.Y0; It.X1 := Pc.X1; It.Y1 := Pc.Y1;
                        Push (It, "a piece of you: everything that swings when channel " & Codec.Img (K) & " of arm " & Codec.Img (A + 1) & " moves (measured), now in cell " &
                              Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & Rel (It.Cu, It.Cv) &
                              (if Tr.Pieces_Known (K) then "" else " (placed from my joints; not yet looked at here)"), Draw.Orange, 1);
                     end if;
                  end;
               end loop;
            end if;
         end;
       end loop;
      end loop;
      Append (T, "THINGS OUT IN THE WORLD (cut out of the depth picture; you do not know what they are called). Each is boxed and NUMBERED on the picture in green:" & ASCII.LF);
      for Si in 0 .. World.Count (C.Wld, Cam) - 1 loop
         declare
            Sl : constant World.Slot := World.Get (C.Wld, Cam, Si);
            It : Item;
         begin
            It.Slot := Si;
            if C.Wld.Holding and then C.Wld.Held_Slot = Si and then C.Wld.Held_Cam = Integer (Cam) then
               declare
                  A : constant Natural := Natural (C.Wld.Held_Arm);
                  Tr : constant Zone_Track := C.Zones (Track_Idx (C, A, Cam));
                  --  拿着东西的是哪一个抓握通道:身体记着(C.Wld.Held_Jaw)
                  Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam, Natural (Integer'Max (0, C.Wld.Held_Jaw)));
               begin
                  It.Kind := Thing_Held; It.Arm := A; It.Located := Tr.Valid;
                  It.Cu := Tr.Cu; It.Cv := Tr.Cv; It.Depth := Tr.Z;
                  It.X0 := Z.X0; It.Y0 := Z.Y0; It.X1 := Z.X1; It.Y1 := Z.Y1;
                  It.Count := Sl.Shadow.Count; It.Height := Sl.Shadow.Height;
                  Push (It, "the thing between the fingers of arm " & Codec.Img (A + 1) & " (it moves with that arm), now in cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)), Draw.Green, 2);
               end;
            elsif Sl.Present then
               It.Kind := Thing; It.Located := True;
               It.Cu := Sl.R.Cu; It.Cv := Sl.R.Cv; It.Depth := Sl.R.Depth; It.Height := Sl.R.Height; It.Count := Sl.R.Count;
               It.X0 := Sl.R.X0; It.Y0 := Sl.R.Y0; It.X1 := Sl.R.X1; It.Y1 := Sl.R.Y1;
               It.Au := Sl.R.Au; It.Av := Sl.R.Av; It.Elong := Sl.R.Elong;
               It.Gray := Picture.Mean_Gray (F.Cams (Cam).Gray, Cw, Ch, Sl.R);
               Push (It, "a thing, now in cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & " (" & Codec.Img (It.Count) & " px, standing " &
                     Codec.Fmt (It.Height, 3) & " out of the surface)" & Rel (It.Cu, It.Cv), Draw.Green, 2);
            elsif Sl.Seen then
               It.Kind := Thing_Remembered; It.Located := True;
               It.Cu := Sl.Shadow.Cu; It.Cv := Sl.Shadow.Cv; It.Depth := Sl.Shadow.Depth; It.Height := Sl.Shadow.Height; It.Count := Sl.Shadow.Count;
               It.X0 := Sl.Shadow.X0; It.Y0 := Sl.Shadow.Y0; It.X1 := Sl.Shadow.X1; It.Y1 := Sl.Shadow.Y1;
               It.Au := Sl.Shadow.Au; It.Av := Sl.Shadow.Av; It.Elong := Sl.Shadow.Elong;
               Push (It, "a thing you saw before, remembered where it was last seen, cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)) &
                     " (not visible right now - probably under my hand; " & Codec.Img (It.Count) & " px)", Draw.Dim_Green, 1);
            else
               It.Kind := Thing_Remembered;
               Push (It, "(a slot with nothing in it right now)", Draw.Dim_Green, 0);
            end if;
         end;
      end loop;
      --  相机表
      declare
         K : Natural := 2;
      begin
         Append (T, "CAMERAS (say look = k to see through that camera next turn): 1 = this picture (camera index " & Codec.Img (Cam) & ")");
         for Ci in 0 .. C.Map.N_Cams - 1 loop
            if Ci /= Cam then
               declare
                  A : constant Integer := Cam_Arm (C, Ci);
               begin
                  Append (T, "; " & Codec.Img (K) & " = camera index " & Codec.Img (Ci) & (if A >= 0 then " (rides on arm " & Codec.Img (Natural (A) + 1) & ": when that arm moves, that whole picture changes)" else ""));
                  K := K + 1;
               end;
            end if;
         end loop;
         Append (T, ASCII.LF);
      end;
      declare
         A : constant Integer := Cam_Arm (C, Cam);
      begin
         if A >= 0 then
            Append (T, "- this picture rides on arm " & Codec.Img (Natural (A) + 1) & ": its fingers and grip stay put in this picture, the world moves when that arm moves" & ASCII.LF);
         end if;
      end;
      if Have_Named then
         for I in 0 .. Natural (C.Items.Length) - 1 loop
            if C.Items (I).Kind in Thing | Thing_Remembered | Thing_Held and then C.Items (I).Slot = C.Wld.Cams (Cam).Named then
               Append (T, "- the thing you last named is item " & Codec.Img (I + 1) & ", now in cell " & Codec.Img (Cell_Of (C, Named_U, Named_V)) & ASCII.LF);
            end if;
         end loop;
      end if;
      Append (T, "- there is " & (if C.Wld.Holding then "ALREADY something" else "NOTHING") & " between your fingers right now" & ASCII.LF);
      Text := T;
   end Build_Listing;

   --  ── 被跟踪的点 ──
   type Point is record
      Arm : Natural := 0;
      Kind : Track_Kind := Piece_Pt;
      Slot : Integer := -1;
      Item_No : Natural := 0;
      Chan_K : Natural := 0;     --  带这块的通道(Chan.Per_Arm = 握合通道 ⇒ 这块是手指)
      Blob : Integer := -1;      --  这块的第几团(手指两团时一团一个点:两指各自到位,歪了就有一团不到位 —— 倾斜自然被罚)
      Cu, Cv, Z : Long_Float := 0.0;
      Tu, Tv, Tz : Long_Float := 0.0;
      Wz : Long_Float := 0.0;
      Desc : Unbounded_String;
      Box_W, Box_H : Long_Float := 0.0;
      Count : Natural := 0;
      Height : Long_Float := 0.0;
      Err0 : Long_Float := 0.0;
      Lost : Boolean := False;   --  这一步没在画面里认出它,位置是按表猜的
      Has_Meas : Boolean := False;              --  眼睛(光流)另外量到的位置,只用来修表
      Meas_U, Meas_V, Meas_Z : Long_Float := 0.0;
      Known : Boolean := True;                  --  这个位置是真看过的/离真看过的样本不超过一步 ⇒ 不是,走之前先看一眼
      Elong : Long_Float := 1.0;                --  脑点名那一刻这块的胖瘦(长轴/短轴),用来和别的块区分
      Gray : Long_Float := -1.0;                --  脑点名那一刻这块的平均灰度(< 0 = 没量到)
      Unsure : Boolean := False;                --  重新认的时候有两块一样像 ⇒ 不许自己挑,回去问脑
      Size : Long_Float := 0.0;                 --  这一块在画面里看着多大(框的边长,画幅):离得越近越大
      Ang : Long_Float := 0.0;                  --  这一块的朝向(主轴角的两倍,弧度;两倍 = 让主轴的正负两种写法算同一个)
      Tsize, Tang : Long_Float := 0.0;          --  要它看着多大 / 转到多少
      Wsize, Wang : Long_Float := 0.0;          --  这两样这一段要不要(0 = 不管)
      Steps_Err : Long_Float := 0.0;            --  上一步算出来的"还差几步"(三样都除以推一步能改多少之后的总和)
      Err_U, Err_V, Err_Z : Long_Float := 0.0;  --  拆开的五样(左右 / 上下 / 远近 / 大小 / 朝向),单位都是"还差几步"
      Err_S, Err_A : Long_Float := 0.0;
      Raw_Err : Long_Float := 0.0;              --  不随表变的差距(全是比例):画面距离 + 远近差几成 + 大小差几成 + 朝向差几成。
                                                --  判"有没有在靠近"只能用它 —— "还差几步"的刻度每步都在变,尺子一缩就看着像退步
      Par_Tu, Par_Tv : Long_Float := 0.0;       --  两团展开时,整块的目标(看清各团真实位置后按它重算各团目标)
      Hard : Boolean := False;                  --  脑说的是 hold ⇒ 这一条整段不许被牺牲(解算时进硬约束,软目标只能在它的零空间里做文章)
   end record;
   package Point_Vectors is new Ada.Containers.Vectors (Natural, Point);
   type Effect_Array is array (Natural range <>) of Table.Effect;
   procedure Refind_Pieces (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector);

   --  还差多少:只算画面上的距离(画幅)。远近不混进来 —— 混着求和是错的判据(LAB 2026-08-17)
   --  角度差绕回 (-π, π](两倍角的世界里,这等于把主轴的正负两种写法当成同一个)
   function Wrap (X : Long_Float) return Long_Float is
      Two_Pi : constant Long_Float := 2.0 * Ada.Numerics.Pi;
      Y : Long_Float := X;
   begin
      while Y > Ada.Numerics.Pi loop
         Y := Y - Two_Pi;
      end loop;
      while Y <= -Ada.Numerics.Pi loop
         Y := Y + Two_Pi;
      end loop;
      return Y;
   end Wrap;

   function Err_Of (P : Point) return Long_Float is
   begin
      return Sqrt ((P.Tu - P.Cu) ** 2 + (P.Tv - P.Cv) ** 2);
   end Err_Of;

   --  "看着多大"最小分得清多少:框变一个像素(画幅的 1/宽)。仿真里静止两拍画面一模一样,拿"静止时抖多少"当地板会得到 0 = 没有地板
   function Size_Floor (Cw : Natural) return Long_Float is (1.0 / Long_Float (Natural'Max (1, Cw)));
   --  朝向最小分得清多少:这块最长的那边偏一个像素(两倍角 ⇒ ×4)
   function Ang_Floor (P : Point; Cw, Ch : Natural) return Long_Float is
     (4.0 / Long_Float'Max (4.0, Long_Float'Max (P.Box_W * Long_Float (Cw), P.Box_H * Long_Float (Ch))));

   --  到位了没:画面上进了跟踪噪声,且远近的差不超过这块东西自己的尺寸(全是量出来的,没有写死的容差)
   function Reached (P : Point; Track_Floor : Long_Float) return Boolean is
      Tol : constant Long_Float := Long_Float'Max (P.Height, Long_Float'Max (P.Box_W, P.Box_H) * P.Z);
   begin
      if Err_Of (P) > Track_Floor then
         return False;
      end if;
      if P.Wsize > 0.0 and then P.Tsize > 0.0 and then abs (P.Tsize - P.Size) / P.Tsize > 0.25 then
         return False;   --  看着差过四分之一就还没到(比例,无量纲)
      end if;
      if P.Wz > 0.0 and then P.Z > 0.0 and then not Picture.Is_Nan (P.Tz) then
         return abs (P.Tz - P.Z) <= Long_Float'Max (Tol, 1.0e-9);
      end if;
      return True;
   end Reached;

   function Held_Now (C : Context; Arm : Natural) return Integer is
     (if C.Wld.Holding and then C.Wld.Held_Arm = Integer (Arm) then C.Wld.Held_Slot else -1);

   function Find_Effect (C : Context; Arm, Cam : Natural; Kind : Track_Kind; Chan_K : Natural; Blob : Integer := -1) return Integer is
   begin
      for I in 0 .. Natural (C.Tables.Length) - 1 loop
         if C.Tables (I).Arm = Arm and then C.Tables (I).Cam = Cam and then C.Tables (I).Kind = Kind and then C.Tables (I).Chan_K = Chan_K and then C.Tables (I).Blob = Blob
           and then C.Tables (I).Held = Held_Now (C, Arm) then
            return I;
         end if;
      end loop;
      return -1;
   end Find_Effect;

   procedure Store_Effect (C : in out Context; Arm, Cam : Natural; Kind : Track_Kind; Chan_K : Natural; Blob : Integer; E : Table.Effect; Trust : Table.Mask; Reach : Table.Vec := Unit_Reach;
                           Pose : Plug.Arm_Pose := [others => 0.0]; Has_Pose : Boolean := False) is
      I : constant Integer := Find_Effect (C, Arm, Cam, Kind, Chan_K, Blob);
      Old : constant Integer := I;
      Se : Stored_Effect := (Arm, Cam, Kind, Chan_K, Blob, E, Trust, Reach, Pose, Has_Pose, Held_Now (C, Arm));
   begin
      if not Has_Pose and then Old >= 0 then
         Se.Pose := C.Tables (Natural (Old)).Pose;      --  没带位姿的更新:沿用这张表原来量的位姿
         Se.Has_Pose := C.Tables (Natural (Old)).Has_Pose;
      end if;
      if I >= 0 then
         C.Tables.Replace_Element (Natural (I), Se);
      else
         C.Tables.Append (Se);
      end if;
   end Store_Effect;

   --  感觉:按此刻的位姿,从身体图算出每只手在每台(不长在它上面的)相机里的两瓣位置 —— 不看画面。
   --  Known = 离最近的真看过的样本不超过一步核实过的步幅;超了就是生地,走过去要看一眼(抖手指)。
   procedure Feel (C : in out Context; F : Plug.Frame) is
      function Clamp (X : Long_Float) return Long_Float is (Long_Float'Max (0.0, Long_Float'Min (1.0, X)));
   begin
      for A in 0 .. C.Map.Arms - 1 loop
         for Cm in 0 .. C.Map.N_Cams - 1 loop
            if Cam_Arm (C, Cm) /= Integer (A) and then A < Natural (F.EE.Length) and then Track_Idx (C, A, Cm) < Natural (C.Zones.Length) then
               declare
                  Diff : Table.Vec;
                  Dist : Long_Float;
                  Si : constant Integer := Schema.Nearest (C.Sch, A, Cm, F.EE (A), C.Map.Amp, Chan.Per_Arm, Diff, Dist);
               begin
                  if Si >= 0 then
                     declare
                        Sm : constant Schema.Sample := C.Sch.S (Natural (Si));
                        Tr : Zone_Track := C.Zones (Track_Idx (C, A, Cm));
                        Reach : Table.Vec := Unit_Reach;
                        Gp : constant Schema.Part_Pos := Sm.Parts (Chan.Per_Arm);   --  握合通道带的那块 = 手指
                        function Shift (Blob : Integer) return Table.Vec3 is
                           Idx : Integer := Find_Effect (C, A, Cm, Piece_Pt, Chan.Per_Arm, Blob);
                        begin
                           if Idx < 0 then
                              Idx := Find_Effect (C, A, Cm, Piece_Pt, Chan.Per_Arm, -1);
                           end if;
                           if Idx >= 0 then
                              for K in 0 .. Chan.Per_Arm - 1 loop
                                 Reach (K) := Long_Float'Max (Reach (K), C.Tables (Natural (Idx)).Reach (K));
                              end loop;
                              return Table.Predict (C.Tables (Natural (Idx)).E, Diff);
                           end if;
                           return Table.Zero3;
                        end Shift;
                        Sa : constant Table.Vec3 := Shift (0);
                        Sb : constant Table.Vec3 := Shift (1);
                     begin
                        if Gp.Valid then
                           Tr.Valid := True;
                           Tr.Au := Clamp (Gp.B0u + Sa (0)); Tr.Av := Clamp (Gp.B0v + Sa (1));
                           Tr.Bu := Clamp (Gp.B1u + Sb (0)); Tr.Bv := Clamp (Gp.B1v + Sb (1));
                           Tr.Has_Lobes := Gp.N_Blobs >= 1;
                           if Gp.N_Blobs >= 2 then
                              Tr.Cu := (Tr.Au + Tr.Bu) / 2.0; Tr.Cv := (Tr.Av + Tr.Bv) / 2.0;
                           else
                              Tr.Cu := Tr.Au; Tr.Cv := Tr.Av;
                           end if;
                           if Gp.Z > 0.0 then
                              Tr.Z := Gp.Z + (if Gp.N_Blobs >= 2 then (Sa (2) + Sb (2)) / 2.0 else Sa (2));
                           end if;
                           Tr.Known := True;
                           for K in 0 .. Chan.Per_Arm - 1 loop
                              if abs Diff (K) > Long_Float'Max (1.0e-6, C.Map.Amp (A * Chan.Per_Arm + K)) * Cap_Mult * Reach (K) then
                                 Tr.Known := False;
                              end if;
                           end loop;
                           Tr.Pieces (Chan.Per_Arm) := (True, Tr.Cu, Tr.Cv, Tr.Z, Gp.X0, Gp.Y0, Gp.X1, Gp.Y1, Gp.N_Blobs, Tr.Au, Tr.Av, Tr.Bu, Tr.Bv);
                           Tr.Pieces_Known (Chan.Per_Arm) := Tr.Known;
                        end if;
                        --  零件:样本里的位置 + 这个零件自己的响应表外推(没表 ⇒ 只在位姿几乎没差时算"知道")
                        for K in 0 .. Chan.Per_Arm - 1 loop
                           if Sm.Parts (K).Valid then
                              declare
                                 Idx : constant Integer := Find_Effect (C, A, Cm, Piece_Pt, K, -1);
                                 Sh : constant Table.Vec3 := (if Idx >= 0 then Table.Predict (C.Tables (Natural (Idx)).E, Diff) else Table.Zero3);
                                 Pr : Schema.Part_Pos := Sm.Parts (K);
                                 Kn : Boolean := True;
                                 R2 : constant Table.Vec := (if Idx >= 0 then C.Tables (Natural (Idx)).Reach else Table.Zero_Vec);
                              begin
                                 Pr.Cu := Clamp (Sm.Parts (K).Cu + Sh (0)); Pr.Cv := Clamp (Sm.Parts (K).Cv + Sh (1));
                                 if Sm.Parts (K).Z > 0.0 then
                                    Pr.Z := Sm.Parts (K).Z + Sh (2);
                                 end if;
                                 --  框跟着形心平移
                                 Pr.X0 := Natural (Long_Float'Max (0.0, Long_Float (Sm.Parts (K).X0) + Sh (0) * Long_Float (F.Cams (Cm).W)));
                                 Pr.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (F.Cams (Cm).W - 1), Long_Float (Sm.Parts (K).X1) + Sh (0) * Long_Float (F.Cams (Cm).W))));
                                 Pr.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Sm.Parts (K).Y0) + Sh (1) * Long_Float (F.Cams (Cm).H)));
                                 Pr.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (F.Cams (Cm).H - 1), Long_Float (Sm.Parts (K).Y1) + Sh (1) * Long_Float (F.Cams (Cm).H))));
                                 for J in 0 .. Chan.Per_Arm - 1 loop
                                    if abs Diff (J) > Long_Float'Max (1.0e-6, C.Map.Amp (A * Chan.Per_Arm + J)) * (if Idx >= 0 then Cap_Mult * Long_Float'Max (1.0, R2 (J)) else 1.0) then
                                       Kn := False;
                                    end if;
                                 end loop;
                                 Tr.Pieces (K) := Pr;
                                 Tr.Pieces_Known (K) := Kn;
                              end;
                           end if;
                        end loop;
                        C.Zones.Replace_Element (Track_Idx (C, A, Cm), Tr);
                     end;
                  end if;
               end;
            end if;
         end loop;
      end loop;
   end Feel;

   --  重新定位一个点:握区靠光流平流(世界相机)/固定(自己的手上相机);世界块重切后就近对上
   procedure Retrack (C : in out Context; F : Plug.Frame; Cam : Natural; Before : Buf; P : in out Point; Pred_U, Pred_V : Long_Float; Moved_Arm : Boolean; Pred_Z : Long_Float := -1.0) is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
   begin
      P.Lost := False;
      case P.Kind is
         when Piece_Pt =>
            if Cam_Arm (C, Cam) = Integer (P.Arm) then
               return;    --  自己的手上相机:握区是固定像素
            end if;
            declare
               --  半分辨率算光流(3 层 30 轮:次数),在这一点一小片取平均位移
               Hw : constant Natural := Cw / 2;
               Hh : constant Natural := Ch / 2;
               A, B : Buf;
               Fl : Flow.Field;
               Du, Dv : Long_Float;
               Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam, Jaw_K_Of (P.Chan_K));
               Old_Z : constant Long_Float := P.Z;
            begin
               A.Reserve_Capacity (Ada.Containers.Count_Type (Hw * Hh));
               B.Reserve_Capacity (Ada.Containers.Count_Type (Hw * Hh));
               for Y in 0 .. Hh - 1 loop
                  for X in 0 .. Hw - 1 loop
                     A.Append (Before.Element ((2 * Y) * Cw + 2 * X));
                     B.Append (F.Cams (Cam).Gray.Element ((2 * Y) * Cw + 2 * X));
                  end loop;
               end loop;
               Fl := Flow.Compute (A, B, Hw, Hh, 3, 30);
               --  取平均的那一片 = 张幅的四分之一(比例,无量纲),再小也有一个像素百分比
               Flow.Sample (Fl, P.Cu, P.Cv, Long_Float'Max (0.01, Z.Span * 0.25), Du, Dv);
               if Moved_Arm and then Sqrt (Du * Du + Dv * Dv) * Long_Float (Cw) < 0.5 then
                  P.Cu := Pred_U; P.Cv := Pred_V; P.Lost := True;      --  手臂动了,这儿画面却没流:跟丢的迹象,用预测
               else
                  P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cu + Du));
                  P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cv + Dv));
               end if;
               if F.Cams (Cam).Has_Depth then
                  declare
                     --  深度读在这一瓣自己的位置上(区心是两指之间的空,读到的是桌面);窗口 = 张幅的四分之一(比例,无量纲)
                     Win : constant Long_Float := Long_Float'Max (0.005, Z.Span * 0.25);
                     Zd : constant Long_Float := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, P.Cu, P.Cv, Win);
                  begin
                     if not Picture.Is_Nan (Zd) then
                        --  一步之内深度跳了超过"预测的变化 + 距离的一成"(比例,无量纲)⇒ 读到的不是我的手指,留预测
                        if Old_Z <= 0.0 or else Pred_Z <= 0.0 or else abs (Zd - Pred_Z) <= abs (Pred_Z - Old_Z) + 0.1 * Old_Z then
                           P.Z := Zd;
                        else
                           P.Z := Pred_Z;
                        end if;
                     end if;
                  end;
               end if;
            end;
         when Thing_Pt =>
            --  重新认脑点名的那一块:除了"位置近、大小差不多",还要"胖瘦像、颜色像"(ER:球 4703 px 和乐高 6334 px 大小分不开,跟错了)。
            --  🔴 认东西是脑的活:两块一样像的时候不许自己挑 —— 记 Unsure,回去问脑要号。
            declare
               Regs : constant Picture.Regions := Cut_Things (C, F, Cam);
               Best, Second : Integer := -1;
               Bd, Sd : Long_Float := 1.0e9;
               Tol : constant Long_Float := Long_Float'Max (P.Box_W, P.Box_H) * 0.75 + Track_Win;
               --  不像的程度:位置差几个跟踪窗 + 大小差几成 + 胖瘦差几成 + 灰度差几成(都是比例,无量纲)。
               --  🔴 大小只参与"像不像",不许当一票否决的硬门槛 —— 越走近它越大,硬门槛会在最该抓住的时候把它判丢(ET 实测)
               function Unlike (R : Picture.Region) return Long_Float is
                  D : constant Long_Float := Sqrt ((R.Cu - Pred_U) ** 2 + (R.Cv - Pred_V) ** 2) / Long_Float'Max (Tol, 1.0e-9);
                  Sz : constant Long_Float := (if P.Count > 0 and then R.Count > 0 then
                                                  Long_Float (Integer'Max (R.Count, P.Count) - Integer'Min (R.Count, P.Count))
                                                  / Long_Float (Integer'Max (R.Count, P.Count))
                                               else 0.0);
                  E : constant Long_Float := abs (R.Elong - P.Elong) / Long_Float'Max (1.0, P.Elong);
                  G : constant Long_Float := (if P.Gray >= 0.0 then
                                                 abs (Picture.Mean_Gray (F.Cams (Cam).Gray, Cw, Ch, R) - P.Gray) / 255.0
                                              else 0.0);
               begin
                  return D + Sz + E + G;
               end Unlike;
            begin
               for I in 0 .. Natural (Regs.Length) - 1 loop
                  declare
                     R : constant Picture.Region := Regs (I);
                     D : constant Long_Float := Sqrt ((R.Cu - Pred_U) ** 2 + (R.Cv - Pred_V) ** 2);
                     U : constant Long_Float := Unlike (R);
                  begin
                     if D <= Tol then
                        if U < Bd then
                           Sd := Bd; Second := Best;
                           Bd := U; Best := I;
                        elsif U < Sd then
                           Sd := U; Second := I;
                        end if;
                     end if;
                  end;
               end loop;
               if Best >= 0 then
                  declare
                     R : Picture.Region := Regs (Best);
                  begin
                     --  挨在一起的碎片算同一块:走近时物体会被切成几瓣(EV 落图:球裂成上沿+左右两条边)。
                     --  把外框挨着最像那块的碎片并进来,大小/形状按并集算
                     for Q of Regs loop
                        if Q.Count > 0 and then Q.X0 <= R.X1 and then Q.X1 >= R.X0 and then Q.Y0 <= R.Y1 and then Q.Y1 >= R.Y0 then
                           declare
                              Cx : constant Long_Float := (R.Cu * Long_Float (R.Count) + Q.Cu * Long_Float (Q.Count)) / Long_Float (R.Count + Q.Count);
                              Cy : constant Long_Float := (R.Cv * Long_Float (R.Count) + Q.Cv * Long_Float (Q.Count)) / Long_Float (R.Count + Q.Count);
                           begin
                              R.X0 := Natural'Min (R.X0, Q.X0); R.Y0 := Natural'Min (R.Y0, Q.Y0);
                              R.X1 := Natural'Max (R.X1, Q.X1); R.Y1 := Natural'Max (R.Y1, Q.Y1);
                              R.Cu := Cx; R.Cv := Cy;
                              R.Count := R.Count + Q.Count;
                              if Q.Depth > 0.0 and then (R.Depth <= 0.0 or else Q.Depth < R.Depth) then
                                 R.Depth := Q.Depth;   --  并起来之后取最近的那一片的远近
                              end if;
                           end;
                        end if;
                     end loop;
                     P.Z := R.Depth; P.Height := R.Height; P.Count := R.Count;
                     P.Box_W := Long_Float (R.X1 - R.X0) / Long_Float (Cw);
                     P.Box_H := Long_Float (R.Y1 - R.Y0) / Long_Float (Ch);
                     P.Cu := R.Cu; P.Cv := R.Cv;
                     P.Size := Sqrt (Long_Float'Max (0.0, P.Box_W * P.Box_H));
                     P.Ang := 2.0 * Arctan (R.Av, R.Au);
                     P.Elong := R.Elong;
                     --  朝向算多少分,看这块有多"长条":圆的(长短轴一样)自动为零 —— 球没有朝向,给满分就是追噪声
                     if P.Wang > 0.0 then
                        P.Wang := Long_Float'Max (0.0, 1.0 - 1.0 / Long_Float'Max (1.0, P.Elong));
                     end if;
                     --  只有真打平才叫分不开:第二像的和最像的差不到一成(比例,无量纲)。
                     --  松了会天天停下问(EU:球和它自己裂出来的小块也算"一样像")
                     P.Unsure := Second >= 0 and then Sd <= Bd * 1.1;
                  end;
               else
                  P.Cu := Pred_U; P.Cv := Pred_V; P.Lost := True;
               end if;
            end;
      end case;
   end Retrack;

   --  发一步并等稳;返回实到(通道)
   procedure Step_Arm (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; A : Table.Vec; Jaw : Floats;
                       Delivered : out Table.Vec; Ok : out Boolean; Quick : Boolean := False; Watch : Selfmap.Watcher := null) is
      P0 : constant Plug.Arm_Pose := F.EE (Arm);
      Frames : Natural;
   begin
      Selfmap.Go (L, C.Map, Arm, Chan.Compose (P0, A), Jaw, F, Delivered, Frames, Ok, Quick, Watch);
   end Step_Arm;

   --  没有表的点(同一只手的几个点一起):每个通道推一下量一列。幅度从开机看得见的那一档起,翻倍到每个点在画面里
   --  跑过 4 个跟踪地板、或深度变过深度地板为止(倍数,无量纲;EF 实测:最小可见幅度量出的列全是噪声,解算据此拧手腕);
   --  深度地板 = 这一点连着两拍读深度抖多少的 4 倍,再小也有距离的百分之一(比例,无量纲);翻到上限还看不出动的通道,这一段不用它。推完推回起点。
   procedure Probe_Effects (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector;
                            Effs : in out Effect_Array; Trust : out Table.Mask; Ok : out Boolean) is
      Arm : constant Natural := Pts (0).Arm;
      P0 : constant Plug.Arm_Pose := F.EE (Arm);
      Jaw : Floats;
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Floor_Px : constant Long_Float := 4.0 / Long_Float (Cw);   --  跟踪地板:4 个像素(倍数,无量纲)
      Floor_Z : array (0 .. Natural (Pts.Length) - 1) of Long_Float := [others => 0.0];
      --  新加的两行也要有自己的噪声地板:探针那一点点幅度下,"看着多大/朝向"的变化可能比噪声还小,
      --  连符号都会是反的 ⇒ 身体照着反方向走(FB 实测:它一路往后退)。地板 = 静止两拍的抖动的 4 倍(倍数,无量纲)
      Floor_S : array (0 .. Natural (Pts.Length) - 1) of Long_Float := [others => 0.0];
      Floor_A : array (0 .. Natural (Pts.Length) - 1) of Long_Float := [others => 0.0];
      Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
      --  一次推动只证明"它动过",证明不了"它稳"。同一个推法重复这么多次,量散布(次数,无量纲)。
      Reps_Wanted : constant := 3;
      N_Pts : constant Natural := Natural (Pts.Length);
      type Sum_Grid is array (0 .. N_Pts - 1, 0 .. Chan.Per_Arm - 1, 0 .. Table.Rows - 1) of Long_Float;
      S1 : Sum_Grid := [others => [others => [others => 0.0]]];   --  各次列值之和
      S2 : Sum_Grid := [others => [others => [others => 0.0]]];   --  各次列值平方和
      Nrep : array (0 .. Chan.Per_Arm - 1) of Natural := [others => 0];
      --  重复够了(或者中途翻脸了)⇒ 把均值写进表,把散布÷|均值| 写进散布格
      procedure Finalise (K : Natural) is
      begin
         for I in 0 .. N_Pts - 1 loop
            declare
               Nk : constant Long_Float := Long_Float (Natural'Max (1, Nrep (K)));
               Mean, Sc : Table.Vec3;
            begin
               for R in 0 .. Table.Rows - 1 loop
                  Mean (R) := S1 (I, K, R) / Nk;
                  declare
                     Var : constant Long_Float := Long_Float'Max (0.0, S2 (I, K, R) / Nk - Mean (R) * Mean (R));
                  begin
                     Sc (R) := (if abs Mean (R) > 0.0 then Sqrt (Var) / abs Mean (R) else 0.0);
                  end;
               end loop;
               Table.Set_Col (Effs (I), K, Mean);
               Table.Set_Spread (Effs (I), K, Nrep (K), Sc);
            end;
         end loop;
      end Finalise;
   begin
      Trust := [others => False];
      Jaw := Selfmap.Jaw_All (F, Arm);
      for I in 0 .. Natural (Pts.Length) - 1 loop
         Table.Reset (Effs (I), Chan.Per_Arm, 1.0);
         for K in 0 .. Chan.Per_Arm - 1 loop
            declare
               Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Arm * Chan.Per_Arm + K));
            begin
               Table.Set_Prior (Effs (I), K, 100.0 / (Am * Am));   --  先验按探针幅度定(倍数,无量纲)
            end;
         end loop;
      end loop;
      --  深度读数地板:什么都不做,连着两拍在各点读深度
      if F.Cams (Cam).Has_Depth then
         declare
            Z1 : array (0 .. Natural (Pts.Length) - 1) of Long_Float := [others => -1.0];
            Ok2 : Boolean;
         begin
            for I in 0 .. Natural (Pts.Length) - 1 loop
               --  读深窗口 = 张幅的四分之一,再小也有半个百分点的画幅(比例,无量纲)
               Z1 (I) := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, Pts (I).Cu, Pts (I).Cv, Long_Float'Max (0.005, Z.Span * 0.25));
            end loop;
            declare
               Was0 : constant Point_Vectors.Vector := Pts;
               Before0 : constant Buf := F.Cams (Cam).Gray;
            begin
               Selfmap.Idle (L, F, 1, Ok2);
               --  静止一拍,量"看着多大/朝向"自己抖多少
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  declare
                     P2 : Point := Pts (I);
                  begin
                     Retrack (C, F, Cam, Before0, P2, Was0 (I).Cu, Was0 (I).Cv, False);
                     Floor_S (I) := Long_Float'Max (4.0 * abs (P2.Size - Was0 (I).Size), Size_Floor (Cw));
                     Floor_A (I) := Long_Float'Max (4.0 * abs (Wrap (P2.Ang - Was0 (I).Ang)), Ang_Floor (Was0 (I), Cw, Ch));
                  end;
               end loop;
            end;
            for I in 0 .. Natural (Pts.Length) - 1 loop
               declare
                  --  读深窗口 = 张幅的四分之一,再小也有半个百分点的画幅(比例,无量纲)
                  Z2 : constant Long_Float := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, Pts (I).Cu, Pts (I).Cv, Long_Float'Max (0.005, Z.Span * 0.25));
                  Zr : constant Long_Float := (if Pts (I).Z > 0.0 then Pts (I).Z else 1.0);
               begin
                  --  地板 = 两拍读深抖动的 4 倍(倍数,无量纲),再小也有距离的百分之一(比例,无量纲)
                  if not Picture.Is_Nan (Z1 (I)) and then not Picture.Is_Nan (Z2) then
                     Floor_Z (I) := Long_Float'Max (4.0 * abs (Z1 (I) - Z2), 0.01 * Zr);
                  else
                     Floor_Z (I) := 0.01 * Zr;
                  end if;
               end;
            end loop;
         end;
      end if;
      Ok := True;
      Put_Line ("[身]   这些点还没有响应表 ⇒ 六个通道各推一下量列(幅度从开机看得见的那一档起翻倍,到点真的动过地板为止)");
      --  六个通道一起解:转动不禁(owner 2026-09-07:禁了就永远和桌面平行,格斗全成直线)。让转动有对错的是"两根手指各自到位":
      --  转歪了必有一指不到位;让转动不比平移便宜的是按各自探针幅度计价。
      for K in 0 .. Chan.Per_Arm - 1 loop
         declare
            Chn : constant Natural := Arm * Chan.Per_Arm + K;
            Amp : Long_Float := C.Map.Amp (Chn);
            Cap_Amp : constant Long_Float := C.Map.Amp (Chn) * Cap_Mult;
         begin
            if not C.Map.Seen (Chn) or else Amp <= 0.0 then
               Put_Line ("[身]     通道" & Natural'Image (Chn) & " 开机时没看见它动,这一列留零");
            else
               loop
                  declare
                     A : Table.Vec := Table.Zero_Vec;
                     Before : constant Buf := F.Cams (Cam).Gray;
                     Was : constant Point_Vectors.Vector := Pts;
                     Deliv, Back : Table.Vec;
                     Ok2 : Boolean;
                     Frames : Natural;
                     Seen_Enough : Boolean := True;
                     Ran_Max : Long_Float := 0.0;
                  begin
                     A (K) := Amp;
                     Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok2);
                     if not Ok2 then
                        Ok := False;
                        return;
                     end if;
                     for I in 0 .. Natural (Pts.Length) - 1 loop
                        declare
                           P : Point := Pts (I);
                           W0 : constant Point := Was (I);
                           Ran, Dz : Long_Float;
                        begin
                           Retrack (C, F, Cam, Before, P, W0.Cu, W0.Cv, True);
                           Ran := Sqrt ((P.Cu - W0.Cu) ** 2 + (P.Cv - W0.Cv) ** 2);
                           Dz := (if P.Z > 0.0 and then W0.Z > 0.0 then abs (P.Z - W0.Z) else 0.0);
                           Ran_Max := Long_Float'Max (Ran_Max, Ran);
                           if abs Deliv (K) > C.Map.EE_Noise and then (Ran >= Floor_Px or else Dz >= Floor_Z (I)) then
                              declare
                                 Col : Table.Vec3;
                              begin
                                 Col (0) := (P.Cu - W0.Cu) / Deliv (K);
                                 Col (1) := (P.Cv - W0.Cv) / Deliv (K);
                                 Col (2) := (if P.Z > 0.0 and then W0.Z > 0.0 then (P.Z - W0.Z) / Deliv (K) else 0.0);
                                 --  推一下这块看着变大变小多少、转了多少(圆的东西转不出来 ⇒ 这一列恒零 ⇒ 自动不参与)
                                 --  只有变化过了自己的噪声地板才敢写进表,否则这一格留零(留零 = 归一时这一行自动不参与)
                                 Col (3) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (P.Size - W0.Size) > Floor_S (I)
                                             then (P.Size - W0.Size) / Deliv (K) else 0.0);
                                 Col (4) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (Wrap (P.Ang - W0.Ang)) > Floor_A (I)
                                             then Wrap (P.Ang - W0.Ang) / Deliv (K) else 0.0);
                                 for R in 0 .. Table.Rows - 1 loop
                                    S1 (I, K, R) := S1 (I, K, R) + Col (R);
                                    S2 (I, K, R) := S2 (I, K, R) + Col (R) * Col (R);
                                 end loop;
                              end;
                           else
                              Seen_Enough := False;
                           end if;
                           Pts.Replace_Element (I, P);
                        end;
                     end loop;
                     if Seen_Enough then
                        Nrep (K) := Nrep (K) + 1;
                        Put_Line ("[身]     通道" & Natural'Image (Chn) & " 第" & Natural'Image (Nrep (K)) & " 次:命令 " & Codec.Fmt (Amp, 4) & " 实到 " & Codec.Fmt (Deliv (K), 4) & " ⇒ 点跑了 " &
                                  Codec.Fmt (Ran_Max, 4) & " 画幅,深度变 " & Codec.Fmt ((if Pts (0).Z > 0.0 and then Was (0).Z > 0.0 then Pts (0).Z - Was (0).Z else 0.0), 4));
                     end if;
                     declare
                        Before2 : constant Buf := F.Cams (Cam).Gray;
                     begin
                        Selfmap.Go (L, C.Map, Arm, P0, Jaw, F, Back, Frames, Ok2);
                        if not Ok2 then
                           Ok := False;
                           return;
                        end if;
                        for I in 0 .. Natural (Pts.Length) - 1 loop
                           declare
                              P : Point := Pts (I);
                           begin
                              Retrack (C, F, Cam, Before2, P, Was (I).Cu, Was (I).Cv, True);
                              P.Cu := Was (I).Cu; P.Cv := Was (I).Cv; P.Z := Was (I).Z;   --  推回起点了:点回到原处(比光流往返的累积误差可信)
                              Pts.Replace_Element (I, P);
                           end;
                        end loop;
                     end;
                     if Seen_Enough then
                        if Nrep (K) >= Reps_Wanted then
                           Finalise (K);
                           Trust (K) := True;
                           exit;
                        end if;
                        --  同一幅度再来一次(不翻倍):现在要证的是"它稳",不是"它动过"
                     else
                        if Nrep (K) > 0 then
                           --  前面动过,这一次同样的推法没动 ⇒ 这就是"不稳"本身,如实收档交给体检判
                           Finalise (K);
                           Trust (K) := True;
                           Put_Line ("[身]     通道" & Natural'Image (Chn) & ":同一个推法第" & Natural'Image (Nrep (K) + 1) & " 次没动 ⇒ 不稳,如实记下");
                           exit;
                        end if;
                        if Amp * 2.0 > Cap_Amp then
                           Put_Line ("[身]     通道" & Natural'Image (Chn) & ":到 " & Codec.Fmt (Amp, 4) & " 点还没动过地板(跑 " & Codec.Fmt (Ran_Max, 4) & " 画幅,地板 " & Codec.Fmt (Floor_Px, 4) & ")⇒ 这一段不用它");
                           exit;
                        end if;
                        Amp := Amp * 2.0;
                     end if;
                  end;
               end loop;
            end if;
         end;
      end loop;
   end Probe_Effects;

   function Amount_Factor (A : Unbounded_String) return Long_Float is
     (if A = "small" then 0.25 elsif A = "large" then 1.0 else 0.5);   --  探针上限的几分之几(比例,无量纲)


   --  握区的点展开成瓣点(两瓣时):每一瓣各自到位,目标 = 区目标 + 这一瓣相对区心的偏移(保持此刻的开合朝向);
   --  歪了就有一瓣不到位,倾斜不用规则自然被罚。自己的手上相机里握区是固定像素,不展开。
   procedure Expand_Lobes (C : Context; F : Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector) is
      Out_P : Point_Vectors.Vector;
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
   begin
      for P of Pts loop
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam, Jaw_K_Of (P.Chan_K));
         begin
            if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm and then Cam_Arm (C, Cam) /= Integer (P.Arm) and then Z.Valid and then Z.N_Lobes = 2 then
               for Lb in 0 .. 1 loop
                  declare
                     Q : Point := P;
                     Tr : constant Zone_Track := C.Zones (Track_Idx (C, P.Arm, Cam));
                     --  瓣相对区心的偏移:身体图给了此刻各瓣位置就用它(转过的手瓣也跟着转),否则用开机量的
                     Ou : constant Long_Float := (if Tr.Has_Lobes then (if Lb = 0 then Tr.Au else Tr.Bu) - Tr.Cu else (if Lb = 0 then Z.A.Cu else Z.B.Cu) - Z.Cu);
                     Ov : constant Long_Float := (if Tr.Has_Lobes then (if Lb = 0 then Tr.Av else Tr.Bv) - Tr.Cv else (if Lb = 0 then Z.A.Cv else Z.B.Cv) - Z.Cv);
                     Zd : Long_Float := P.Z;
                  begin
                     Q.Blob := Lb;
                     Q.Par_Tu := P.Tu; Q.Par_Tv := P.Tv;
                     Q.Cu := P.Cu + Ou; Q.Cv := P.Cv + Ov;
                     Q.Tu := P.Tu + Ou; Q.Tv := P.Tv + Ov;
                     if F.Cams (Cam).Has_Depth then
                        --  读深窗口 = 张幅的四分之一,再小也有半个百分点的画幅(比例,无量纲)
                        Zd := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, Q.Cu, Q.Cv, Long_Float'Max (0.005, Z.Span * 0.25));
                        if Picture.Is_Nan (Zd) then
                           Zd := P.Z;
                        end if;
                     end if;
                     Q.Z := Zd;
                     if Lb = 1 then
                        Q.Desc := Null_Unbounded_String;
                     end if;
                     Out_P.Append (Q);
                  end;
               end loop;
            else
               Out_P.Append (P);
            end if;
         end;
      end loop;
      Pts := Out_P;
   end Expand_Lobes;

   --  ── 一段 = 反复做五件事:①打算怎么走 ②走 ③看 ④学 ⑤判 ──
   --  每件事一个小过程;这一步发生了什么全记在 Note 里(字段名就是人话),五件事之间只靠它说话。
   procedure Run_Segment (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector;
                          Until_Kind : Monitor.Until_Kind; Step_Limit : Natural; Amount : Long_Float; Avoid : Item_Vectors.Vector;
                          Event : out Unbounded_String; Steps_Taken : out Natural; Blocked_Out : out Boolean; Beats : out Natural) is
      Arm : constant Natural := Pts (0).Arm;
      Beats0 : constant Natural := Plug.Steps (L);
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Own_Cam : constant Boolean := Cam_Arm (C, Cam) = Integer (Arm);
      Effs : Effect_Array (0 .. Natural (Pts.Length) - 1);
      Trusts : array (0 .. Natural (Pts.Length) - 1) of Table.Mask := [others => [others => True]];
      Reach : Table.Vec := Unit_Reach;   --  每通道核实过的步幅倍数(存表里,越用越强)
      Fl : Monitor.Floors;
      W : Monitor.Watch;
      Ring : Backup.Ring;
      Jaw : Floats;
      Last_Err : Long_Float := -1.0;     --  -1 = 还没算过(哨兵,无量纲)
      Last_Raw : Long_Float := -1.0;     --  上一步不随表变的差距
      Best_Raw : Long_Float := -1.0;     --  到目前为止最好的一次(判"有没有在靠近"和它比,不和上一步比 —— 噪声一晃就成退步)
      Trust : Long_Float := 0.5;         --  这张表有多准,就走它算出来的多大比例(半开始;预测差一半就只走一半,准了再放开)
      Lost_Run : Natural := 0;           --  连着几步全部认不到

      --  这一步的账
      type Note_Rec is record
         Cmd, Got, Cap : Table.Vec := Table.Zero_Vec;   --  要走的 / 实际走的 / 各通道这一步的上限
         Active : Table.Mask := [others => False];
         Floor_Cmd : Long_Float := 0.0;                 --  最小探针幅度:比它一半还小的命令说明不了"顶住"
         Err_Now : Long_Float := 0.0;                   --  这一步之后还差几步(尺度会变,只用来说话)
         Raw_Now : Long_Float := 0.0;                   --  这一步之后不随表变的差距(判有没有在靠近就看它)
         Pic_Delta : Long_Float := 0.0;
         Big_Step : Boolean := False;                   --  大到光流跟不住 ⇒ 走完抖一下认自己
         Halted : Boolean := False;                     --  途中眼睛叫停
         Blocked : Boolean := False;                    --  顶住了(零表更准)
         Not_Followed : Boolean := False;               --  整步没照做
         Touched : Boolean := False;                    --  我没在推的东西也动了 = 碰到
         Lost_All : Boolean := False;                   --  被跟的全都认不到
         Unsure : Boolean := False;                     --  两块一样像,不许自己挑
         Say_Stop : Unbounded_String;                   --  非空 = 这一段到此为止,内容就是给脑的话
      end record;
      Note : Note_Rec;
      Before : Buf;                      --  走之前那一拍的灰度(光流用)
      Was : Point_Vectors.Vector;        --  走之前各点在哪
      Was_Regs : Picture.Regions;        --  走之前世界里各块在哪

      --  走的途中每一拍看一眼:被跟的东西还找得到吗、离画面边够不够远(转一点点就该知道不对劲)
      function Watch_Things (Fr : Plug.Frame) return Boolean is
         Regs : Picture.Regions;
         Have : Boolean := False;
      begin
         for P of Pts loop
            if P.Kind = Thing_Pt then
               if not Have then
                  Regs := Cut_Things (C, Fr, Cam);
                  Have := True;
               end if;
               declare
                  Found : Boolean := False;
                  Tol : constant Long_Float := Long_Float'Max (P.Box_W, P.Box_H) * 0.75 + Track_Win;   --  和 Retrack 同一个认领半径(比例,无量纲)
               begin
                  for R of Regs loop
                     if R.Count * 3 >= P.Count and then R.Count <= P.Count * 3 and then Sqrt ((R.Cu - P.Cu) ** 2 + (R.Cv - P.Cv) ** 2) <= Tol then
                        Found := True;
                        if R.Cu < Track_Win or else R.Cu > 1.0 - Track_Win or else R.Cv < Track_Win or else R.Cv > 1.0 - Track_Win then
                           Note.Halted := True;
                           return True;
                        end if;
                     end if;
                  end loop;
                  if not Found then
                     Note.Halted := True;
                     return True;
                  end if;
               end;
            end if;
         end loop;
         return False;
      end Watch_Things;

      --  落一张图,把这一拍切出来的块和被跟的点画上去(出岔子时给人看)
      procedure Dump_Picture (Tag : String) is
      begin
         if C.Dump_Dir = "" then
            return;
         end if;
         declare
            RGB : Buf := F.Cams (Cam).RGB;
            Regs : constant Picture.Regions := Cut_Things (C, F, Cam);
            N : Natural := 0;
         begin
            for R of Regs loop
               N := N + 1;
               Draw.Numbered_Box (RGB, Cw, Ch, R.X0, R.Y0, R.X1, R.Y1, N, Draw.Green, 2);
            end loop;
            for P of Pts loop
               declare
                  Hw : constant Long_Float := Long_Float'Max (P.Box_W, Track_Win) * 0.5;   --  框(没有就用一个跟踪窗;比例,无量纲)
                  Hh : constant Long_Float := Long_Float'Max (P.Box_H, Track_Win) * 0.5;
               begin
                  Draw.Numbered_Box (RGB, Cw, Ch,
                                     Natural (Long_Float'Max (0.0, (P.Cu - Hw) * Long_Float (Cw))),
                                     Natural (Long_Float'Max (0.0, (P.Cv - Hh) * Long_Float (Ch))),
                                     Natural (Long_Float'Min (Long_Float (Cw - 1), (P.Cu + Hw) * Long_Float (Cw))),
                                     Natural (Long_Float'Min (Long_Float (Ch - 1), (P.Cv + Hh) * Long_Float (Ch))),
                                     0, Draw.Pink, 2);
               end;
            end loop;
            Codec.Write_BMP (To_String (C.Dump_Dir) & "/" & Tag & "_" & Codec.Pad6 (C.Round_N) & "_" & Codec.Pad6 (Steps_Taken) & ".bmp", RGB, Cw, Ch);
            Put_Line ("[身]     落图 " & Tag & "_" & Codec.Pad6 (C.Round_N) & "_" & Codec.Pad6 (Steps_Taken) & ".bmp(切出" & Natural'Image (N) & " 块)");
         end;
      end Dump_Picture;

      --  段前:每个点的响应表 —— 装回(必须是同一位姿、同样拿着东西、至少有一列信得过)或当场重量
      procedure Ready_Tables (Ok_Out : out Boolean) is
         Need : Boolean := False;
      begin
         Ok_Out := True;
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               Idx : constant Integer := Find_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob);
            begin
               if Idx >= 0 and then (for some K in 0 .. Chan.Per_Arm - 1 => C.Tables (Natural (Idx)).Trust (K))
                 and then C.Tables (Natural (Idx)).Has_Pose
                 and then (for all K in 0 .. Chan.Per_Arm - 1 =>
                             abs Chan.Delivered (C.Tables (Natural (Idx)).Pose, F.EE (Arm)) (K)
                             <= Long_Float'Max (1.0e-6, C.Map.Amp (Arm * Chan.Per_Arm + K)) * Cap_Mult * C.Tables (Natural (Idx)).Reach (K))
               then
                  Effs (I) := C.Tables (Natural (Idx)).E;
                  Trusts (I) := C.Tables (Natural (Idx)).Trust;
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     Reach (K) := Long_Float'Max (Reach (K), C.Tables (Natural (Idx)).Reach (K));
                  end loop;
               else
                  Need := True;
               end if;
            end;
         end loop;
         if Need then
            declare
               Trust : Table.Mask;
               Ok : Boolean;
            begin
               Probe_Effects (L, C, F, Cam, Pts, Effs, Trust, Ok);
               if not Ok then
                  Ok_Out := False;
                  return;
               end if;
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  Trusts (I) := Trust;
                  Store_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob, Effs (I), Trust, Unit_Reach, F.EE (Arm), True);
               end loop;
            end;
         end if;
      end Ready_Tables;

      --  ①a 定目标:每个点的五样差距,各自除以"推一步最多能改多少",变成"还差几步"
      procedure Aim (Terms : out Table.Term_Vectors.Vector) is
      begin
         Terms.Clear;
         declare
            Big : Long_Float := 0.0;
         begin
            for P of Pts loop
               if P.Kind = Thing_Pt then
                  Big := Long_Float'Max (Big, Long_Float'Max (P.Box_W, P.Box_H));
               end if;
            end loop;
            C.Want_Size := Big;
         end;
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               P : constant Point := Pts (I);
               T : Table.Term;
            begin
               T.E := Effs (I);
               --  🔴 离得越远,"落在两指正中间"越不该急着满足:那是到跟前才成立的几何。
               --  权重 = 指尖有多近 ÷ 它有多远(量出来的两个深度之比):远的时候远近压过画面 ⇒ 先走过去;
               --  越走近画面越重要 ⇒ 最后才精确对准。不这么定,它会在 25 cm 外用转手腕把画面对齐,手一步没靠近(FI 实测)
               declare
                  Near : constant Long_Float :=
                    (if P.Wz > 0.0 and then P.Z > 0.0 and then not Picture.Is_Nan (P.Tz) and then P.Tz > 0.0
                     then Long_Float'Min (1.0, P.Tz / P.Z) else 1.0);
               begin
                  T.Err (0) := P.Tu - P.Cu;  T.W (0) := Near;
                  T.Err (1) := P.Tv - P.Cv;  T.W (1) := Near;
               end;
               --  远近:画面位置和远近一起要,不许替它定"先对准再靠近"的顺序(那等于叫它先扭脖子)
               if P.Wz > 0.0 and then P.Z > 0.0 and then not Picture.Is_Nan (P.Tz) then
                  T.Err (2) := P.Tz - P.Z; T.W (2) := 1.0;
               end if;
               --  看着多大:离得越近越大,这是最稳的远近信号(画面上量的)
               if P.Wsize > 0.0 and then P.Size > 0.0 and then P.Tsize > 0.0 then
                  T.Err (3) := P.Tsize - P.Size; T.W (3) := 1.0;
               end if;
               --  朝向:差绕回 (-π, π];圆的东西这一行谁也改不动,归一时自动关掉
               if P.Wang > 0.0 then
                  T.Err (4) := Wrap (P.Tang - P.Ang); T.W (4) := P.Wang;
               end if;
               --  🔴 五样单位不同,混着求和就是错的判据。不换算成米,改成【只比较】:
               --  每一样除以"推一步最多能把它改多少",都变成"还差几步"(无量纲),本来就可比。
               --  扭手腕改不了远近 ⇒ 它在那一栏拿不到分,偷不了便宜。
               for R in 0 .. Table.Rows - 1 loop
                  declare
                     Per_Step : Long_Float := 0.0;
                  begin
                     for K in 0 .. Chan.Per_Arm - 1 loop
                        if C.Map.Seen (Arm * Chan.Per_Arm + K) and then Trusts (I) (K) then
                           Per_Step := Long_Float'Max (Per_Step, abs (T.E.B (K, R)) * Long_Float'Max (1.0e-9, C.Map.Amp (Arm * Chan.Per_Arm + K)));
                        end if;
                     end loop;
                     if Per_Step > 0.0 then
                        T.Err (R) := T.Err (R) / Per_Step;
                        for K in 0 .. Chan.Per_Arm - 1 loop
                           T.E.B (K, R) := T.E.B (K, R) / Per_Step;
                        end loop;
                     else
                        T.W (R) := 0.0;   --  这一行一个通道都改不动 ⇒ 这一步不管它
                     end if;
                  end;
               end loop;
               declare
                  Q : Point := P;
               begin
                  Q.Steps_Err := 0.0;
                  for R in 0 .. Table.Rows - 1 loop
                     Q.Steps_Err := Q.Steps_Err + (T.Err (R) * T.W (R)) ** 2;
                  end loop;
                  Q.Steps_Err := Sqrt (Q.Steps_Err);
                  Q.Err_U := T.Err (0) * T.W (0); Q.Err_V := T.Err (1) * T.W (1); Q.Err_Z := T.Err (2) * T.W (2);
                  Q.Err_S := T.Err (3) * T.W (3); Q.Err_A := T.Err (4) * T.W (4);
                  Q.Raw_Err := Sqrt ((P.Tu - P.Cu) ** 2 + (P.Tv - P.Cv) ** 2
                                     + (if P.Wz > 0.0 and then P.Z > 0.0 then ((P.Tz - P.Z) / P.Z) ** 2 else 0.0)
                                     + (if P.Wsize > 0.0 and then P.Tsize > 0.0 then ((P.Tsize - P.Size) / P.Tsize) ** 2 else 0.0)
                                     + (if P.Wang > 0.0 then (P.Wang * Wrap (P.Tang - P.Ang) / Ada.Numerics.Pi) ** 2 else 0.0));
                  Pts.Replace_Element (I, Q);
               end;
               Terms.Append (T);
            end;
         end loop;
         if Last_Err < 0.0 then
            Last_Err := 0.0; Last_Raw := 0.0;
            for P of Pts loop
               Last_Err := Last_Err + P.Steps_Err;
               Last_Raw := Last_Raw + P.Raw_Err;
            end loop;
            Best_Raw := Last_Raw;
         end if;
      end Aim;

      --  ①b 定额度:各通道这一步最多走多少 —— 两条取小:①眼睛跟得住的那么多(表说走多少画面跑满一个跟踪窗)
      --  ②自己那一档 × 核实过的倍数。只用①,在画面里几乎不动的通道会拿到无限额度(甩腕 2 rad)
      procedure Budget (Terms : Table.Term_Vectors.Vector; Solved : out Boolean) is
         Damp : Table.Vec := Table.Zero_Vec;
      begin
         for K in 0 .. Chan.Per_Arm - 1 loop
            declare
               Ch_No : constant Natural := Arm * Chan.Per_Arm + K;
               Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Ch_No));
               All_Trust : Boolean := True;
               Px : Long_Float := 0.0;
            begin
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  if not Trusts (I) (K) then
                     All_Trust := False;
                  end if;
                  Px := Long_Float'Max (Px, Sqrt (Effs (I).B (K, 0) ** 2 + Effs (I).B (K, 1) ** 2));
               end loop;
               if C.Map.Seen (Ch_No) and then All_Trust then
                  Note.Active (K) := True;
                  --  🔴 只有【后果全量清楚了】的方向才准迈大步。某一格没量出来(探针时变化没过地板)会被留成 0,
                  --  而 0 的意思是"没影响",解算就当它免费 —— 转腕对"远近/看着多大"正是这样,于是它拿转腕去修画面位置,
                  --  一转就把距离搞坏(FD 实测:0.6 rad 的腕,球越走越远)。没量清楚的方向只给探针那一档。
                  declare
                     Known_All : Boolean := Px * Am > Fl.Track * 2.0;
                  begin
                     for I in 0 .. Natural (Pts.Length) - 1 loop
                        for R in 0 .. Table.Rows - 1 loop
                           if Terms (I).W (R) > 0.0 and then abs (Effs (I).B (K, R)) <= 0.0 then
                              Known_All := False;
                           end if;
                        end loop;
                     end loop;
                     --  上限 = 自己那一档 × 核实过的倍数,再压在"眼睛跟得住"这个天花板下。
                     --  倍数只有靠"表说会挪多少 vs 实际挪了多少"对上才涨(见 Learn),没证明过就不许迈大步
                     Note.Cap (K) := Long_Float'Min (Am * Cap_Mult * Reach (K),
                                                     (if Known_All then Track_Win / Px else Am * Cap_Mult * Reach (K))) * Amount;
                  end;
                  Note.Floor_Cmd := (if Note.Floor_Cmd <= 0.0 then Am else Long_Float'Min (Note.Floor_Cmd, Am));
               end if;
               --  🔴 标价改成"这个动作把画面搅动多少":一单位命令让被跟的点在画面里跑几个跟踪窗,就付几分钱(无量纲)。
               --  以前按"自己那一档"计价,而转腕那一档(0.0256)比平移那一档(0.0064)大四倍 ⇒ 转腕在账本上便宜十六倍,
               --  于是它一直买转腕,而转腕不会让手靠近(FJ 实测:横挪 4 cm,球反而从 0.333 m 退到 0.360 m)
               Damp (K) := (Px / Track_Win) ** 2;
            end;
         end loop;
         --  hold 的那几条进硬约束:先把它们解到位,软目标只能在剩下的自由度里做文章。
         --  平权解在挤不下的时候一定会牺牲朝向(自检里那条 5.500 就是),所以这里不能用平权。
         declare
            Hard, Soft : Table.Term_Vectors.Vector;
         begin
            for I in 0 .. Natural (Terms.Length) - 1 loop
               if I < Natural (Pts.Length) and then Pts (I).Hard then
                  Hard.Append (Terms (I));
               else
                  Soft.Append (Terms (I));
               end if;
            end loop;
            Table.Solve_Priority (Hard, Soft, Chan.Per_Arm, Note.Cap, Note.Active, Damp, Note.Cmd, Solved);
         end;
      end Budget;

      --  ①c 修步子:缩到眼睛跟得住,且不把被跟的东西推出视野、不让我身上任何一块压到"不许碰"的框
      procedure Trim is
         Scale : Long_Float := 1.0;
      begin
         --  求稳不求快:一步里任何被跟的点在画面里最多跑一个跟踪窗
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Cmd);
               D : constant Long_Float := Sqrt (Pr (0) ** 2 + Pr (1) ** 2);
            begin
               if D > Track_Win then
                  Scale := Long_Float'Min (Scale, Track_Win / D);
               end if;
            end;
         end loop;
         if Scale < 1.0e-3 then
            Note.Big_Step := True;   --  缩到千分之一还不够(比例,无量纲)= 表已经不可信
         end if;
         --  不许把被跟的东西推出视野;不许让【我身上任何一块】压到"不许碰"的框里(只查一个点等于没查)
         for Round in 1 .. 4 loop
            declare
               Hit : Boolean := False;
               function In_Avoid (X0, Y0, X1, Y1 : Long_Float) return Boolean is
               begin
                  for Av of Avoid loop
                     if Av.Located and then X0 <= Long_Float (Av.X1) and then X1 >= Long_Float (Av.X0)
                       and then Y0 <= Long_Float (Av.Y1) and then Y1 >= Long_Float (Av.Y0)
                     then
                        return True;
                     end if;
                  end loop;
                  return False;
               end In_Avoid;
            begin
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  declare
                     Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Cmd);
                     Nu : constant Long_Float := Pts (I).Cu + Pr (0) * Scale;
                     Nv : constant Long_Float := Pts (I).Cv + Pr (1) * Scale;
                  begin
                     if Nu < Track_Win or else Nu > 1.0 - Track_Win or else Nv < Track_Win or else Nv > 1.0 - Track_Win then
                        Hit := True;
                     end if;
                     if In_Avoid (Nu * Long_Float (Cw), Nv * Long_Float (Ch), Nu * Long_Float (Cw), Nv * Long_Float (Ch)) then
                        Hit := True;
                     end if;
                  end;
               end loop;
               if not Own_Cam and then Track_Idx (C, Arm, Cam) < Natural (C.Zones.Length) then
                  declare
                     Tr : constant Zone_Track := C.Zones (Track_Idx (C, Arm, Cam));
                  begin
                     for K in 0 .. Chan.Per_Arm loop
                        if Tr.Pieces (K).Valid then
                           declare
                              Idx : constant Integer := Find_Effect (C, Arm, Cam, Piece_Pt, K, -1);
                              Sh : constant Table.Vec3 := (if Idx >= 0 then Table.Predict (C.Tables (Natural (Idx)).E, Note.Cmd) else Table.Zero3);
                              Du : constant Long_Float := Sh (0) * Scale * Long_Float (Cw);
                              Dv : constant Long_Float := Sh (1) * Scale * Long_Float (Ch);
                           begin
                              if In_Avoid (Long_Float (Tr.Pieces (K).X0) + Du, Long_Float (Tr.Pieces (K).Y0) + Dv,
                                           Long_Float (Tr.Pieces (K).X1) + Du, Long_Float (Tr.Pieces (K).Y1) + Dv)
                              then
                                 Hit := True;
                              end if;
                           end;
                        end if;
                     end loop;
                  end;
               end if;
               exit when not Hit;
               if Round = 4 then
                  Note.Say_Stop := S ("stopped: every step would push what I am tracking out of my sight, or put some part of me onto a thing I must not touch");
                  return;
               end if;
               Scale := Scale * 0.5;
            end;
         end loop;
         for K in 0 .. Chan.Per_Arm - 1 loop
            Note.Cmd (K) := Note.Cmd (K) * Scale * Trust;   --  表有多准就走多少(不然每步走过头,下一步再拉回来,来回晃)
         end loop;
         if Table.Norm (Note.Cmd, Chan.Per_Arm) <= C.Map.EE_Noise then
            Note.Say_Stop := S ("amount: already there (what is left to push is within my own noise)");
         end if;
      end Trim;

      --  ① 打算怎么走 = 定目标 → 定额度 → 修步子
      procedure Plan is
         Terms : Table.Term_Vectors.Vector;
         Solved : Boolean;
      begin
         Note := (others => <>);
         Aim (Terms);
         Budget (Terms, Solved);
         if not Solved then
            Note.Say_Stop := S ("could not solve which channels to push");
            return;
         end if;
         Trim;
      end Plan;

      --  ② 走:记下走之前的样子,发命令,途中盯着
      procedure Walk (Ok_Out : out Boolean) is
      begin
         Before := F.Cams (Cam).Gray;
         Was := Pts;
         Was_Regs := (if not Own_Cam then Cut_Things (C, F, Cam) else Picture.Region_Vectors.Empty_Vector);
         Step_Arm (L, C, F, Arm, Note.Cmd, Jaw, Note.Got, Ok_Out, C.Fast, Watch_Things'Unrestricted_Access);
         Beats := Plug.Steps (L) - Beats0;
         if Note.Halted then
            Put_Line ("[身]     途中眼睛叫停:被跟的东西快出画面或看不见了,这一步没走完");
         end if;
         if not Ok_Out then
            return;
         end if;
         Steps_Taken := Steps_Taken + 1;
         declare
            Sv : Backup.Step_Vec := [others => 0.0];
         begin
            for K in 0 .. Chan.Per_Arm - 1 loop
               Sv (K) := Note.Got (K);
            end loop;
            Backup.Remember (Ring, Sv);
         end;
         Note.Pic_Delta := Long_Float (Picture.Max_Diff (Before, F.Cams (Cam).Gray));
      end Walk;

      --  ③ 看:每个点现在在哪。我的零件(别人的相机里)先按位姿"感觉",熟地让眼睛核对,生地或大步就抖一下去看;
      --  世界里的块每步重切就近对上。全都认不到才叫看不见。
      procedure Look is
         Need_Refind : Boolean := False;
      begin
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               P : Point := Pts (I);
               W0 : constant Point := Was (I);
               Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Got);
            begin
               P.Has_Meas := False;
               if P.Kind = Piece_Pt and then not Own_Cam then
                  declare
                     Diff : Table.Vec;
                     Dist : Long_Float;
                     Si : constant Integer := Schema.Nearest (C.Sch, P.Arm, Cam, F.EE (P.Arm), C.Map.Amp, Chan.Per_Arm, Diff, Dist);
                     Familiar : Boolean := False;
                     In_Map : Boolean := False;
                  begin
                     if Si >= 0 then
                        In_Map := P.Chan_K <= Chan.Per_Arm and then C.Sch.S (Natural (Si)).Parts (P.Chan_K).Valid
                                  and then (P.Blob < 0 or else C.Sch.S (Natural (Si)).Parts (P.Chan_K).N_Blobs > Natural (P.Blob));
                     end if;
                     if In_Map then
                        declare
                           Gp : constant Schema.Part_Pos := C.Sch.S (Natural (Si)).Parts (P.Chan_K);
                           Pm : constant Table.Vec3 := Table.Predict (Effs (I), Diff);
                           Su : constant Long_Float := (if P.Blob = 1 then Gp.B1u elsif P.Blob = 0 then Gp.B0u else Gp.Cu);
                           Sv : constant Long_Float := (if P.Blob = 1 then Gp.B1v elsif P.Blob = 0 then Gp.B0v else Gp.Cv);
                        begin
                           P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, Su + Pm (0)));
                           P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, Sv + Pm (1)));
                           if Gp.Z > 0.0 then
                              P.Z := Gp.Z + Pm (2);
                           end if;
                           Familiar := True;
                           for K in 0 .. Chan.Per_Arm - 1 loop
                              if abs Diff (K) > Long_Float'Max (1.0e-6, C.Map.Amp (P.Arm * Chan.Per_Arm + K)) * Cap_Mult * Reach (K) then
                                 Familiar := False;
                              end if;
                           end loop;
                        end;
                     else
                        P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, W0.Cu + Pr (0)));
                        P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, W0.Cv + Pr (1)));
                        if W0.Z > 0.0 then
                           P.Z := W0.Z + Pr (2);
                        end if;
                     end if;
                     if Note.Big_Step or else not Familiar then
                        P.Lost := True;
                        Need_Refind := True;
                     else
                        declare
                           Q : Point := W0;
                           Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam, Jaw_K_Of (P.Chan_K));
                        begin
                           Retrack (C, F, Cam, Before, Q, W0.Cu + Pr (0), W0.Cv + Pr (1), True, (if W0.Z > 0.0 then W0.Z + Pr (2) else -1.0));
                           --  眼睛和图对不上(差过张幅的四分之一,比例,无量纲;再小也有两个跟踪地板)⇒ 去看
                           if Q.Lost or else Sqrt ((Q.Cu - P.Cu) ** 2 + (Q.Cv - P.Cv) ** 2) > Long_Float'Max (Z.Span * 0.25, Fl.Track * 2.0) then
                              P.Lost := True;
                              Need_Refind := True;
                           else
                              P.Lost := False;
                              P.Has_Meas := True; P.Meas_U := Q.Cu; P.Meas_V := Q.Cv; P.Meas_Z := Q.Z;
                           end if;
                        end;
                     end if;
                  end;
               else
                  Retrack (C, F, Cam, Before, P, W0.Cu + Pr (0), W0.Cv + Pr (1), True, (if W0.Z > 0.0 then W0.Z + Pr (2) else -1.0));
               end if;
               Pts.Replace_Element (I, P);
            end;
         end loop;
         if Need_Refind then
            Refind_Pieces (L, C, F, Cam, Pts);
            Beats := Plug.Steps (L) - Beats0;
         end if;
         Note.Lost_All := True;
         for P of Pts loop
            if not P.Lost then
               Note.Lost_All := False;
            end if;
            if P.Unsure then
               Note.Unsure := True;
            end if;
         end loop;
      end Look;

      --  ④ 学:拿这一步的实际结果修表;判"整步没照做";按通道各自放宽/收紧步幅;看有没有碰到别的东西
      procedure Learn is
         All_Verified : Boolean := True;
         Any_Wrong : Boolean := False;
      begin
         Note.Err_Now := 0.0;
         Note.Raw_Now := 0.0;
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               P : constant Point := Pts (I);
               W0 : constant Point := Was (I);
               Dy : Table.Vec3;
               E : Table.Effect := Effs (I);
            begin
               if P.Lost then
                  All_Verified := False;
                  Any_Wrong := True;
               else
                  --  修表只用眼睛量到的(光流核对值或抖认到的),按图猜的位置不喂回表
                  Dy (0) := (if P.Has_Meas then P.Meas_U else P.Cu) - W0.Cu;
                  Dy (1) := (if P.Has_Meas then P.Meas_V else P.Cv) - W0.Cv;
                  Dy (2) := (if P.Has_Meas then (if P.Meas_Z > 0.0 and then W0.Z > 0.0 then P.Meas_Z - W0.Z else 0.0)
                             elsif P.Z > 0.0 and then W0.Z > 0.0 then P.Z - W0.Z else 0.0);
                  --  同样的地板:没过就当没变(不然把量化噪声学进表里,符号可能是反的)
                  Dy (3) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (P.Size - W0.Size) > Size_Floor (Cw)
                             then P.Size - W0.Size else 0.0);
                  Dy (4) := (if P.Size > 0.0 and then W0.Size > 0.0 and then abs (Wrap (P.Ang - W0.Ang)) > Ang_Floor (W0, Cw, Ch)
                             then Wrap (P.Ang - W0.Ang) else 0.0);
                  Table.Update (E, Note.Got, Dy, Fl.Track * 2.0, Long_Float'Max (C.Map.EE_Noise, 0.5 * Note.Floor_Cmd));
                  if Table.Blocked (E) then
                     Note.Blocked := True;
                  end if;
                  if not (E.Null_Res > Fl.Track * 2.0 and then E.Free_Res < E.Null_Res) then
                     All_Verified := False;
                  end if;
                  if E.Free_Res > Fl.Track * 2.0 and then E.Free_Res >= E.Null_Res then
                     Any_Wrong := True;
                  end if;
               end if;
               Effs (I) := E;
               Note.Err_Now := Note.Err_Now + P.Steps_Err;
               Note.Raw_Now := Note.Raw_Now + P.Raw_Err;
            end;
         end loop;
         --  整步没照做:各通道按自己的探针幅度归一后,实到与命令差过一半(逐个通道判会被同量级的小出入触发)
         if not Note.Halted then
            declare
               Dn, An : Long_Float := 0.0;
            begin
               for K in 0 .. Chan.Per_Arm - 1 loop
                  declare
                     Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Arm * Chan.Per_Arm + K));
                  begin
                     Dn := Dn + ((Note.Got (K) - Note.Cmd (K)) / Am) ** 2;
                     An := An + (Note.Cmd (K) / Am) ** 2;
                  end;
               end loop;
               Dn := Sqrt (Dn); An := Sqrt (An);
               if An > 1.0 and then Dn > 0.5 * An then
                  Any_Wrong := True; All_Verified := False; Note.Not_Followed := True;
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     if Note.Active (K) then
                        Reach (K) := Long_Float'Max (1.0, Reach (K) * 0.5);
                     end if;
                  end loop;
                  Put_Line ("[身]     整步没照做:要走的和实际走的差了 " & Codec.Fmt (Dn / Long_Float'Max (1.0e-9, An) * 100.0, 0) & "% ⇒ 步幅缩回上一档");
               end if;
            end;
         end if;
         --  🔴 步幅只认一件事:这一步【表说会挪多少】和【实际挪了多少】对不对得上。
         --  对得上 ⇒ 这几个用到的通道可以把步子放大一倍;差过一半 ⇒ 立刻缩回去。
         --  不管是平移还是转腕,都得先证明自己说话算数才有资格迈大步(FD/FE:转腕说了不算,一转球就更远)
         declare
            Pred_Ok : Boolean := True;
            Any_Meas : Boolean := False;
         begin
            for I in 0 .. Natural (Pts.Length) - 1 loop
               if not Pts (I).Lost then
                  declare
                     W0 : constant Point := Was (I);
                     Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Got);
                     Act_U : constant Long_Float := (if Pts (I).Has_Meas then Pts (I).Meas_U else Pts (I).Cu) - W0.Cu;
                     Act_V : constant Long_Float := (if Pts (I).Has_Meas then Pts (I).Meas_V else Pts (I).Cv) - W0.Cv;
                     Pred : constant Long_Float := Sqrt (Pr (0) ** 2 + Pr (1) ** 2);
                     Act : constant Long_Float := Sqrt (Act_U ** 2 + Act_V ** 2);
                  begin
                     if Pred > Fl.Track * 2.0 or else Act > Fl.Track * 2.0 then
                        Any_Meas := True;
                        --  差过预测的一半就算说了不算;再给一个和跟踪精度挂钩的绝对宽容(四分之一个跟踪窗),
                        --  否则步子越小相对误差越大,永远判"说了不算",步子就永远放不大(FF 实测每步都判不准)
                        if abs (Act - Pred) > 0.5 * Pred + Track_Win * 0.25 then
                           Pred_Ok := False;
                        end if;
                     end if;
                  end;
               end if;
            end loop;
            for K in 0 .. Chan.Per_Arm - 1 loop
               if Note.Active (K) and then abs Note.Cmd (K) > Long_Float'Max (Note.Floor_Cmd, C.Map.EE_Noise) then
                  if Any_Meas and then Pred_Ok and then (not Note.Halted) and then not Note.Not_Followed then
                     Reach (K) := Long_Float'Min (Reach (K) * 2.0, Track_Win / Long_Float'Max (1.0e-9, C.Map.Amp (Arm * Chan.Per_Arm + K)));
                  elsif Any_Meas and then not Pred_Ok then
                     Reach (K) := Long_Float'Max (1.0, Reach (K) * 0.5);
                  end if;
               end if;
            end loop;
            if Any_Meas and then not Pred_Ok then
               Put_Line ("[身]     表说了不算:预测挪的和实际挪的差过一半 ⇒ 用到的通道步子缩回去");
            end if;
            --  这张表这一步准到什么程度 ⇒ 下一步走它算出来的多大比例(准 = 走满,差一半 = 走一半)
            if Any_Meas then
               declare
                  Worst_Rel : Long_Float := 0.0;
               begin
                  for I in 0 .. Natural (Pts.Length) - 1 loop
                     if not Pts (I).Lost then
                        declare
                           W0 : constant Point := Was (I);
                           Pr : constant Table.Vec3 := Table.Predict (Effs (I), Note.Got);
                           Au : constant Long_Float := (if Pts (I).Has_Meas then Pts (I).Meas_U else Pts (I).Cu) - W0.Cu;
                           Av : constant Long_Float := (if Pts (I).Has_Meas then Pts (I).Meas_V else Pts (I).Cv) - W0.Cv;
                           Pd : constant Long_Float := Sqrt (Pr (0) ** 2 + Pr (1) ** 2);
                           Ac : constant Long_Float := Sqrt (Au ** 2 + Av ** 2);
                        begin
                           Worst_Rel := Long_Float'Max (Worst_Rel, abs (Ac - Pd) / Long_Float'Max (Pd, Track_Win * 0.25));
                        end;
                     end if;
                  end loop;
                  Trust := 0.5 * Trust + 0.5 / (1.0 + Worst_Rel);
               end;
            end if;
         end;
         for I in 0 .. Natural (Pts.Length) - 1 loop
            Store_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob, Effs (I), Trusts (I), Reach);
         end loop;
         --  碰到 = 我没在推的东西自己动了(跟着这只手动的相机里满画面都在动,分不出来 ⇒ 不下结论)
         if not Own_Cam and then not Was_Regs.Is_Empty then
            declare
               Now_Regs : constant Picture.Regions := Cut_Things (C, F, Cam);
            begin
               for R of Now_Regs loop
                  declare
                     Mine : Boolean := False;
                     Found_Prev : Boolean := False;
                     Best : Long_Float := 0.0;
                  begin
                     for P of Pts loop
                        if Sqrt ((R.Cu - P.Cu) ** 2 + (R.Cv - P.Cv) ** 2) <= Long_Float'Max (P.Box_W, P.Box_H) then
                           Mine := True;
                        end if;
                     end loop;
                     if not Mine then
                        for Q of Was_Regs loop
                           if Q.Count * 3 >= R.Count and then R.Count * 3 >= Q.Count then
                              declare
                                 D : constant Long_Float := Sqrt ((R.Cu - Q.Cu) ** 2 + (R.Cv - Q.Cv) ** 2);
                              begin
                                 if not Found_Prev or else D < Best then
                                    Best := D; Found_Prev := True;
                                 end if;
                              end;
                           end if;
                        end loop;
                        if Found_Prev and then Best > Fl.Track * 2.0 then
                           Note.Touched := True;
                        end if;
                     end if;
                  end;
               end loop;
            end;
         end if;
         if Note.Touched then
            Put_Line ("[身]     我没在推的东西也动了 ⇒ 碰到它了");
         end if;
      end Learn;

      --  ⑤ 判:这一步之后接着走,还是到了 / 出事了 / 拿不准
      procedure Judge is
      begin
         --  进度只看不随表变的那把尺(Raw):"还差几步"的刻度每步都在变,用它判进度会把靠近判成退步(ES 实测两步就报停滞)
         --  和"到目前为止最好的一次"比:和上一步比的话,一次噪声就被当成退步
         Monitor.Step (W, Monitor.Floor (Long_Float'Max (0.0, Note.Pic_Delta)), Monitor.Bounded (Best_Raw), Monitor.Bounded (Note.Raw_Now),
                       Monitor.Floor (Long_Float'Max (0.0, Table.Norm (Note.Got, Chan.Per_Arm))), Fl);
         Put_Line ("[身]     步" & Natural'Image (Steps_Taken) & (if Note.Big_Step then "(大步)" else "") &
                   ":差距 " & Codec.Fmt (Last_Raw, 3) & " → " & Codec.Fmt (Note.Raw_Now, 3) & " · 还差 " & Codec.Fmt (Note.Err_Now, 1) & " 步(左右 " & Codec.Fmt (Pts (0).Err_U, 1) &
                   " 上下 " & Codec.Fmt (Pts (0).Err_V, 1) & " 远近 " & Codec.Fmt (Pts (0).Err_Z, 1) &
                   " 大小 " & Codec.Fmt (Pts (0).Err_S, 1) & " 朝向 " & Codec.Fmt (Pts (0).Err_A, 1) & ")· 拍 " & Codec.Img (Beats) &
                   " · 信表 " & Codec.Fmt (Trust, 2) & " · 步幅 ×[" & Codec.Fmt (Reach (0), 0) & " " & Codec.Fmt (Reach (1), 0) & " " & Codec.Fmt (Reach (2), 0) & " " &
                   Codec.Fmt (Reach (3), 0) & " " & Codec.Fmt (Reach (4), 0) & " " & Codec.Fmt (Reach (5), 0) &
                   "] · 命令 [" & Codec.Fmt (Note.Cmd (0), 3) & " " & Codec.Fmt (Note.Cmd (1), 3) & " " & Codec.Fmt (Note.Cmd (2), 3) & " " &
                   Codec.Fmt (Note.Cmd (3), 3) & " " & Codec.Fmt (Note.Cmd (4), 3) & " " & Codec.Fmt (Note.Cmd (5), 3) &
                   "] · 实到 [" & Codec.Fmt (Note.Got (0), 4) & " " & Codec.Fmt (Note.Got (1), 4) & " " & Codec.Fmt (Note.Got (2), 4) & " " &
                   Codec.Fmt (Note.Got (3), 3) & " " & Codec.Fmt (Note.Got (4), 3) & " " & Codec.Fmt (Note.Got (5), 3) &
                   "] · 点 (" & Codec.Fmt (Pts (0).Cu, 3) & "," & Codec.Fmt (Pts (0).Cv, 3) & ") 深 " & Codec.Fmt (Pts (0).Z, 3) &
                   (if Note.Blocked then " · 零表更准(顶住?)" else ""));
         Last_Err := Note.Err_Now;
         Last_Raw := Note.Raw_Now;
         if Best_Raw < 0.0 or else Note.Raw_Now < Best_Raw then
            Best_Raw := Note.Raw_Now;
         end if;
         if Codec.Env ("BL_STEPSHOT") /= "" then
            Dump_Picture ("step");   --  逐步落图:看被跟的那块在靠近时到底怎么变(BL_STEPSHOT 打开才存)
         end if;
         if Note.Blocked or else Monitor.Refusing (W) then
            Blocked_Out := True;
         end if;
         --  认不到:第一次落图给人看,连着两步才停(被挡住一团是常事)
         Lost_Run := (if Note.Lost_All then Lost_Run + 1 else 0);
         if Lost_Run = 1 then
            Dump_Picture ("lost");
         end if;
         if Lost_Run >= 2 then
            Note.Say_Stop := S ("lost sight: two steps in a row I could not find what I am tracking in this picture; I stopped rather than move blind");
            return;
         end if;
         --  🔴 认东西是脑的活:两块一样像的时候身体不许自己挑
         if Note.Unsure then
            Dump_Picture ("unsure");
            Note.Say_Stop := S ("not sure which one is yours: two things here look equally like the one you named (same size, same distance); "
                                & "I stopped instead of guessing - look again and tell me its number");
            return;
         end if;
         if Note.Not_Followed then
            Put_Line ("[身]     没照做这一步不算数,步幅已缩回;接着走");
         end if;
         if Monitor.Fired (Until_Kind, W, Step_Limit, Note.Blocked, Monitor.Bounded (Selfmap.Jaw_Of (F, Arm)),
                           Monitor.Bounded (if Arm < Natural (C.Hands.Length) then C.Hands (Arm).Empty_Close else 0.0),
                           Monitor.Floor (C.Map.Jaw_Noise), Note.Touched)
         then
            Note.Say_Stop := (case Until_Kind is
                                when Monitor.U_Steps => S ("steps: I took the steps you asked for"),
                                when Monitor.U_Contact => S ("contact: something I was not pushing moved when I moved - I am touching it"),
                                when Monitor.U_Resist => S ("resist: I commanded a push and my body did not go"),
                                when Monitor.U_Slip => S ("slip: what I was holding has left my fingers"),
                                when Monitor.U_Settle => S ("settle: the picture stopped changing"));
            return;
         end if;
         declare
            All_There : Boolean := True;
         begin
            for P of Pts loop
               if not Reached (P, Fl.Track * 2.0) then
                  All_There := False;
               end if;
            end loop;
            if All_There then
               Note.Say_Stop := S ("amount: arrived (in the picture and at the same distance as my fingers)");
               return;
            end if;
         end;
         if Steps_Taken > 1 and then Monitor.Stalled (W) then
            Note.Say_Stop := S ("amount: stopped getting closer (still about " & Codec.Fmt (Note.Err_Now, 1) &
                                " pushes away) - either something holds me or this arm cannot reach farther from here");
         end if;
      end Judge;

      Ok : Boolean;
   begin
      Event := S ("hit the safety cap on steps");
      Steps_Taken := 0;
      Beats := 0;
      Blocked_Out := False;
      Jaw := Selfmap.Jaw_All (F, Arm);
      Fl.Track := Long_Float'Max (1.0 / Long_Float (Cw), 0.0);
      Fl.Picture := Long_Float (Integer'(if Cam < Natural (C.Map.Pic_Floor.Length) then C.Map.Pic_Floor (Cam) else 0));
      Fl.Reading := C.Map.Jaw_Noise;
      Fl.Delivery := C.Map.EE_Noise;
      Backup.Clear (Ring);
      Ready_Tables (Ok);
      if not Ok then
         Event := S ("the body stopped answering while I measured my response table");
         return;
      end if;
      --  🔴 证明可以晚,但不许没有:到【要拿这一行去算动作】的这一刻,它必须已经被证明过。
      --  编译期这一块还没量过响应时放行了,现在量完了,当场补判;判不过就一步都不走,把原话退回给脑。
      declare
         Bad : Unbounded_String;
         Notch : Table.Vec := Table.Zero_Vec;
      begin
         for K in 0 .. Chan.Per_Arm - 1 loop
            Notch (K) := C.Map.Amp (Arm * Chan.Per_Arm + K);
         end loop;
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               P : constant Point := Pts (I);
               procedure Want (R : Natural; Nm : String) is
               begin
                  if not Table.Row_Proven (Effs (I), Notch, R) then
                     Append (Bad, (if Length (Bad) > 0 then "; " else "")
                             & "item " & Codec.Img (P.Item_No) & " needs " & Nm & ", but "
                             & Table.Row_Why (Effs (I), Notch, R));
                  end if;
               end Want;
            begin
               if abs (P.Tu - P.Cu) > 0.0 then
                  Want (0, "sideways");
               end if;
               if abs (P.Tv - P.Cv) > 0.0 then
                  Want (1, "up-down");
               end if;
               if P.Wz > 0.0 and then abs (P.Tz - P.Z) > 0.0 then
                  Want (2, "nearness");
               end if;
               if P.Wsize > 0.0 then
                  Want (3, "apparent size");
               end if;
               if P.Wang > 0.0 then
                  Want (4, "facing");
               end if;
            end;
         end loop;
         if Length (Bad) > 0 then
            Event := S ("I did not move: " & To_String (Bad));
            return;
         end if;
      end;
      for Step in 1 .. Natural'Min (Step_Cap, (if Step_Limit > 0 then Step_Limit else Step_Cap)) loop
         Plan;
         if Note.Say_Stop /= "" then
            Event := Note.Say_Stop;
            return;
         end if;
         Walk (Ok);
         if not Ok then
            Event := S ("the body refused the command");
            return;
         end if;
         Look;
         Learn;
         Judge;
         if Note.Say_Stop /= "" then
            Event := Note.Say_Stop;
            Beats := Plug.Steps (L) - Beats0;
            return;
         end if;
      end loop;
      Event := S ("steps: hit the step cap (" & Codec.Img (Steps_Taken) & ")");
   end Run_Segment;

   --  合/张:最多 Max_Iter 拍,或到画面不再变;Sweep_Cam >= 0 时把那台相机里动过的像素累进 Sweep(手指自己扫过的地方)
   procedure Jaw_Sweep (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm, K : Natural; Target : Long_Float; Max_Iter : Natural;
                        Sweep_Cam : Integer; Sweep : in out Bools; Steps : out Natural; Reading : out Long_Float) is
      Jaw : Floats;
      Prev : Long_Float := Selfmap.Jaw_Of (F, Arm, K);
      Prev_Cams : Plug.Cam_Vectors.Vector := F.Cams;
      Still : Natural := 0;
      Cm : Plug.Cmd;
   begin
      --  只动点名的那一个抓握通道,其余保持它们此刻的读数(五指手:合一根不牵动另外四根)
      declare
         Rest : constant Floats := Selfmap.Jaw_All (F, Arm);
      begin
         for I in 0 .. Natural'Max (1, Natural (Rest.Length)) - 1 loop
            Jaw.Append (if I = K then Target elsif I < Natural (Rest.Length) then Rest (I) else 1.0);
         end loop;
      end;
      Steps := 0;
      Reading := Prev;
      --  读数是命令的回声,"停住"只认画面:每台相机连着两拍不变
      for I in 1 .. Max_Iter loop
         Cm.Kind := Plug.Ee; Cm.Arm := Arm; Cm.Pose := F.EE (Arm); Cm.Jaw := Jaw;
         exit when not Plug.Act (L, Cm) or else not Plug.Sense (L, F);
         Steps := I;
         Reading := Selfmap.Jaw_Of (F, Arm, K);
         if Sweep_Cam >= 0 and then Natural (Sweep_Cam) < Natural (F.Cams.Length) and then Natural (Sweep_Cam) < Natural (C.Map.Floors.Length) then
            Sweep := Picture.Either (Sweep, Picture.Moved (Prev_Cams (Natural (Sweep_Cam)).Gray, F.Cams (Natural (Sweep_Cam)).Gray, C.Map.Floors (Natural (Sweep_Cam))));
         end if;
         if abs (Reading - Prev) <= C.Map.Jaw_Noise and then Selfmap.Pictures_Still (C.Map, Prev_Cams, F.Cams) then
            Still := Still + 1;
         else
            Still := 0;
         end if;
         Prev := Reading;
         Prev_Cams := F.Cams;
         exit when Still >= 2 and then I >= 3;
      end loop;
   end Jaw_Sweep;

   --  合/张到读数不再变
   procedure Move_Jaw (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; Target : Long_Float; Steps : out Natural; Reading : out Long_Float;
                       K : Natural := 0) is
      None : Bools;
   begin
      Jaw_Sweep (L, C, F, Arm, K, Target, 40, -1, None, Steps, Reading);
   end Move_Jaw;

   --  生地/大步之后在世界相机里重新看见自己:手指 = 抖一下手指(合几拍再张回来),零件 = 推一下它自己的通道再推回来;
   --  动过的像素就是它,每个点认离预测最近的那团。抖的幅度不是常数:手指合"量出来的稳定拍数"那么久;零件推开机看得见的那一档。认不到的留预测、记 Lost。
   procedure Refind_Pieces (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector) is
      Arm : constant Natural := Pts (0).Arm;
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
      J0 : constant Long_Float := Selfmap.Jaw_Of (F, Arm);
      Steps_J : Natural;
      Reading : Long_Float;
      Any_Fingers : Boolean := False;
      Jaw : Floats;
      --  在累积的"动过"掩膜里给一个点认领最近的一团;Tol = 认领半径(画幅比例,无量纲)
      procedure Claim (P : in out Point; Tol, Win : Long_Float; Taken : in out Bools; Regs : Picture.Regions) is
         Best : Integer := -1;
         Bd : Long_Float := 1.0e9;
      begin
         for R in 0 .. Natural (Regs.Length) - 1 loop
            declare
               D : constant Long_Float := Sqrt ((Regs (R).Cu - P.Cu) ** 2 + (Regs (R).Cv - P.Cv) ** 2);
            begin
               if not Taken (R) and then D <= Tol and then D < Bd then
                  Bd := D; Best := R;
               end if;
            end;
         end loop;
         if Best >= 0 then
            Taken.Replace_Element (Natural (Best), True);
            declare
               Old_Z : constant Long_Float := P.Z;
               Zd : Long_Float;
            begin
               P.Cu := Regs (Best).Cu; P.Cv := Regs (Best).Cv; P.Lost := False;
               P.Box_W := Long_Float (Regs (Best).X1 - Regs (Best).X0) / Long_Float (Cw);
               P.Box_H := Long_Float (Regs (Best).Y1 - Regs (Best).Y0) / Long_Float (Ch);
               if F.Cams (Cam).Has_Depth then
                  --  深度一步跳过"距离的一成"(比例,无量纲)就是读到别的东西了
                  Zd := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, P.Cu, P.Cv, Win);
                  if not Picture.Is_Nan (Zd) and then (Old_Z <= 0.0 or else abs (Zd - Old_Z) <= 0.1 * Old_Z) then
                     P.Z := Zd;
                  end if;
               end if;
            end;
         else
            P.Lost := True;
         end if;
      end Claim;
   begin
      Jaw.Append (J0);
      for P of Pts loop
         if P.Kind = Piece_Pt and then P.Chan_K >= Chan.Per_Arm then
            Any_Fingers := True;
         end if;
      end loop;
      if Any_Fingers then
         declare
            Sweep : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
            Regs : Picture.Regions;
            Taken : Bools;
         begin
            --  这些点分别属于哪几个抓握通道,就抖哪几个(五指手:只抖被跟着的那几根)
            for Kk in 0 .. Jaws_Of (C, Arm) - 1 loop
               declare
                  Wanted : Boolean := False;
               begin
                  for P of Pts loop
                     if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm + Kk then
                        Wanted := True;
                     end if;
                  end loop;
                  if Wanted then
                     Jaw_Sweep (L, C, F, Arm, Kk, 0.0, C.Map.Settle + 1, Integer (Cam), Sweep, Steps_J, Reading);
                     Jaw_Sweep (L, C, F, Arm, Kk, Selfmap.Jaw_Of (F, Arm, Kk), C.Map.Settle + 1, Integer (Cam), Sweep, Steps_J, Reading);
                  end if;
               end;
            end loop;
            Regs := Picture.Components (Sweep, Cw, Ch, Picture.Min_Pixels (Cw, Ch));
            Taken := Bool_Vectors.To_Vector (False, Regs.Length);
            for I in 0 .. Natural (Pts.Length) - 1 loop
               declare
                  P : Point := Pts (I);
               begin
                  if P.Kind = Piece_Pt and then P.Chan_K >= Chan.Per_Arm then
                     --  认领半径:一个张幅,再小也有一个跟踪窗;读深窗口 = 张幅的四分之一,再小也有半个百分点的画幅(比例,无量纲)
                     --  没真看过的位置(只是按关节推的)可能差得远 ⇒ 认领半径放到整幅画面(比例,无量纲)
                     Claim (P, (if P.Known then Long_Float'Max (Z.Span, Track_Win) else 1.0), Long_Float'Max (0.005, Z.Span * 0.25), Taken, Regs);
                     if not P.Lost then
                        P.Known := True;
                     end if;
                     Pts.Replace_Element (I, P);
                  end if;
               end;
            end loop;
         end;
      end if;
      --  零件:各自推一下自己的通道(开机看得见的那一档)再推回来
      for I in 0 .. Natural (Pts.Length) - 1 loop
         declare
            P : Point := Pts (I);
         begin
            if P.Kind = Piece_Pt and then P.Chan_K < Chan.Per_Arm then
               declare
                  K : constant Natural := P.Chan_K;
                  Chn : constant Natural := Arm * Chan.Per_Arm + K;
                  A : Table.Vec := Table.Zero_Vec;
                  Deliv : Table.Vec;
                  Ok : Boolean;
                  B0 : constant Buf := F.Cams (Cam).Gray;
                  Sweep : Bools;
                  Regs : Picture.Regions;
                  Taken : Bools;
                  P0 : constant Plug.Arm_Pose := F.EE (Arm);
                  Frames : Natural;
               begin
                  if Chn < Natural (C.Map.Amp.Length) and then C.Map.Amp (Chn) > 0.0 and then Cam < Natural (C.Map.Floors.Length) then
                     A (K) := C.Map.Amp (Chn);
                     Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok);
                     Sweep := Picture.Moved (B0, F.Cams (Cam).Gray, C.Map.Floors (Cam));
                     declare
                        B1 : constant Buf := F.Cams (Cam).Gray;
                     begin
                        Selfmap.Go (L, C.Map, Arm, P0, Jaw, F, Deliv, Frames, Ok);
                        Sweep := Picture.Either (Sweep, Picture.Moved (B1, F.Cams (Cam).Gray, C.Map.Floors (Cam)));
                     end;
                     Regs := Picture.Components (Sweep, Cw, Ch, Picture.Min_Pixels (Cw, Ch));
                     Taken := Bool_Vectors.To_Vector (False, Regs.Length);
                     --  认领半径:这块自己的框那么大,再小也有一个跟踪窗;读深窗口 = 框的四分之一(比例,无量纲)
                     Claim (P, (if P.Known then Long_Float'Max (Long_Float'Max (P.Box_W, P.Box_H), Track_Win) else 1.0), Long_Float'Max (0.005, Long_Float'Max (P.Box_W, P.Box_H) * 0.25), Taken, Regs);
                     if not P.Lost then
                        P.Known := True;
                     end if;
                  else
                     P.Lost := True;
                  end if;
                  Pts.Replace_Element (I, P);
               end;
            end if;
         end;
      end loop;
      --  认到的记进身体图:这个位姿下,这只手的这些零件(手指也是零件)在这台相机里就在这儿(下次到这附近不用看)
      declare
         X : Schema.Sample;
         Gp : Schema.Part_Pos;   --  手指那块:各团合成
         All_Fingers : Boolean := True;
         N, Nz : Natural := 0;
         Zmin : Long_Float := 1.0e30;   --  哨兵(无量纲)
         Any_Part : Boolean := False;
         Zh : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
      begin
         X.Arm := Arm; X.Cam := Cam; X.Pose := F.EE (Arm);
         for P of Pts loop
            if P.Kind = Piece_Pt and then P.Chan_K >= Chan.Per_Arm then
               if P.Lost then
                  All_Fingers := False;
               end if;
               N := N + 1;
               Gp.Cu := Gp.Cu + P.Cu; Gp.Cv := Gp.Cv + P.Cv;
               if P.Blob = 1 then
                  Gp.B1u := P.Cu; Gp.B1v := P.Cv;
               else
                  Gp.B0u := P.Cu; Gp.B0v := P.Cv;
               end if;
               if P.Z > 0.0 then
                  Zmin := Long_Float'Min (Zmin, P.Z); Nz := Nz + 1;
               end if;
            elsif P.Kind = Piece_Pt and then not P.Lost and then P.Chan_K < Chan.Per_Arm then
               X.Parts (P.Chan_K) := (True, P.Cu, P.Cv, P.Z,
                                      Natural (Long_Float'Max (0.0, (P.Cu - P.Box_W / 2.0) * Long_Float (Cw))), Natural (Long_Float'Max (0.0, (P.Cv - P.Box_H / 2.0) * Long_Float (Ch))),
                                      Natural (Long_Float'Min (Long_Float (Cw - 1), (P.Cu + P.Box_W / 2.0) * Long_Float (Cw))), Natural (Long_Float'Min (Long_Float (Ch - 1), (P.Cv + P.Box_H / 2.0) * Long_Float (Ch))),
                                      1, P.Cu, P.Cv, 0.0, 0.0);
               Any_Part := True;
            end if;
         end loop;
         if All_Fingers and then N > 0 then
            Gp.Valid := True;
            Gp.Cu := Gp.Cu / Long_Float (N); Gp.Cv := Gp.Cv / Long_Float (N);
            Gp.N_Blobs := N;
            Gp.Z := (if Nz > 0 then Zmin else 0.0);
            Gp.X0 := Zh.X0; Gp.Y0 := Zh.Y0; Gp.X1 := Zh.X1; Gp.Y1 := Zh.Y1;   --  框先沿用开机量的(Feel 会按形心平移)
            X.Parts (Chan.Per_Arm) := Gp;
         end if;
         if (All_Fingers and then N > 0) or else Any_Part then
            Schema.Add (C.Sch, X, C.Map.EE_Noise, C.Map.Rot_Noise);
         end if;
      end;
      Put_Line ("[身]     生地/大步之后看一眼自己(手指抖一下 / 零件推一下):" &
                (if Pts (0).Lost then "没认到,按图猜" else "认到了 (" & Codec.Fmt (Pts (0).Cu, 3) & "," & Codec.Fmt (Pts (0).Cv, 3) & ") 深 " & Codec.Fmt (Pts (0).Z, 3)) &
                " · 这台相机里这只手的身体图 " & Codec.Img (Schema.Count (C.Sch, Arm, Cam)) & " 个样本");
   end Refind_Pieces;

   --  握住了没:抬一小截,看东西跟不跟我走。手上相机里 = 它的块还在握区框里;世界相机里 = 它原来那块地方空了。读数不算数(回声)。
   --  Sure = 有没有【不跟着手动的相机】能核实。没有就只能说"我说不准",不许把状态记成"手里有东西"
   procedure Held_Test (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Cam : Natural; Origin : Picture.Region;
                        Obj_Count : Natural; Held : out Boolean; Sure : out Boolean; Note : out Unbounded_String) is
      A : Table.Vec := Table.Zero_Vec;
      Deliv : Table.Vec;
      Ok : Boolean;
      Hc : constant Integer := (if Arm < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (Arm) else -1);
      Jaw : Floats;
      Seen_In_Hand : Boolean := False;
      Gone_From_Table : Boolean := False;
      Could_Judge : Boolean := False;
      --  合完之后要量出"到底发生了什么",不是只答"拿住了没":旁边有没有东西被我碰动、那一块是不是断成了两块
      World_Cam : constant Integer := (if Cam < Natural (F.Cams.Length) and then Cam_Arm (C, Cam) < 0 then Integer (Cam) else -1);
      Before_Regs : Picture.Regions;
      Moved_Others : Natural := 0;
      Pieces_Now : Natural := 0;
   begin
      if World_Cam >= 0 then
         Before_Regs := Cut_Things (C, F, Natural (World_Cam));
      end if;
      Jaw := Selfmap.Jaw_All (F, Arm);
      A (2) := C.Map.Amp (Arm * Chan.Per_Arm + 2) * 4.0;   --  抬起 = 看得见的探针幅度的几倍(倍数,无量纲),不假设哪根轴朝上:2 号轴是身体报的第三个平移通道
      Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok);
      if Hc >= 0 and then Natural (Hc) < Natural (F.Cams.Length) then
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Natural (Hc));
            Regs : constant Picture.Regions := Cut_Things (C, F, Natural (Hc));
            Cw : constant Natural := F.Cams (Natural (Hc)).W;
            Ch : constant Natural := F.Cams (Natural (Hc)).H;
         begin
            if Z.Valid then
               Could_Judge := True;
               for R of Regs loop
                  if R.Count * 3 >= Obj_Count and then R.Cu * Long_Float (Cw) >= Long_Float (Z.X0) and then R.Cu * Long_Float (Cw) <= Long_Float (Z.X1)
                    and then R.Cv * Long_Float (Ch) >= Long_Float (Z.Y0) and then R.Cv * Long_Float (Ch) <= Long_Float (Z.Y1)
                  then
                     Seen_In_Hand := True;
                  end if;
               end loop;
            end if;
         end;
      end if;
      if Cam < Natural (F.Cams.Length) and then Cam_Arm (C, Cam) < 0 and then Origin.Count > 0 then
         Could_Judge := True;
         Gone_From_Table := World.Vanished (Cut_Things (C, F, Cam), Origin, F.Cams (Cam).W, F.Cams (Cam).H);
      end if;
      --  旁边动了几件 · 原地现在剩几块(断成两块的话会多出一块)
      if World_Cam >= 0 then
         declare
            After : constant Picture.Regions := Cut_Things (C, F, Natural (World_Cam));
            Tol : constant Long_Float := 1.0 / Long_Float (F.Cams (Natural (World_Cam)).W);   --  一个像素(画幅比例,无量纲)
         begin
            for Q of Before_Regs loop
               declare
                  Best : Long_Float := 0.0;
                  Found : Boolean := False;
               begin
                  for R of After loop
                     if Q.Count * 3 >= R.Count and then R.Count * 3 >= Q.Count then
                        declare
                           D : constant Long_Float := Sqrt ((R.Cu - Q.Cu) ** 2 + (R.Cv - Q.Cv) ** 2);
                        begin
                           if not Found or else D < Best then
                              Best := D; Found := True;
                           end if;
                        end;
                     end if;
                  end loop;
                  --  不是我夹的那件,却挪过了噪声地板 ⇒ 我碰动了它
                  if Found and then Best > Tol * 4.0 and then Origin.Count > 0
                    and then Sqrt ((Q.Cu - Origin.Cu) ** 2 + (Q.Cv - Origin.Cv) ** 2) > Long_Float'Max (Origin.Sig_U, Origin.Sig_V) * 2.0
                  then
                     Moved_Others := Moved_Others + 1;
                  end if;
               end;
            end loop;
            for R of After loop
               if Origin.Count > 0 and then Sqrt ((R.Cu - Origin.Cu) ** 2 + (R.Cv - Origin.Cv) ** 2) <= Long_Float'Max (Origin.Sig_U, Origin.Sig_V) * 3.0 then
                  Pieces_Now := Pieces_Now + 1;
               end if;
            end loop;
         end;
      end if;
      --  🔴 "拿住了"只有一条硬证据:它原来待的地方空了。手上相机里"还在握区框里"不算数 ——
      --  那个框在手上相机里几乎是半个屏幕,球留在画面里就过关(FM 实测报了"拿住",而头顶相机里球还在桌上)。
      --  两台相机都判不了就老实说"我说不准",不许自称拿住。
      Held := (if World_Cam >= 0 then Gone_From_Table else Seen_In_Hand);
      Sure := World_Cam >= 0;
      if World_Cam >= 0 and then Gone_From_Table then
         Note := S ("after a small lift its place is empty in the camera that does not move with me ⇒ held"
                    & (if Seen_In_Hand then ", and my hand camera still shows it between my fingers" else ""));
      elsif World_Cam >= 0 then
         Note := S ("after a small lift it is still sitting where it was ⇒ NOT held"
                    & (if Seen_In_Hand then " (my hand camera still shows something between my fingers, which proves nothing)" else ""));
      elsif Seen_In_Hand then
         Note := S ("after a small lift the thing is still inside my grip box in my hand camera; no still camera could check, so I am not sure");
      elsif Could_Judge then
         Note := S ("after a small lift the thing did not come with me ⇒ not held");
      else
         Note := S ("I could not judge whether it is held (no camera could see it)");
      end if;
      if World_Cam >= 0 then
         Append (Note, ". While I closed and lifted, " & Codec.Img (Moved_Others) & " other thing(s) I was not pushing moved");
         if Pieces_Now >= 2 then
            Append (Note, ", and where it stood there are now " & Codec.Img (Pieces_Now) & " separate pieces");
         end if;
      end if;
   end Held_Test;

   function Mode_Line (C : Context; Until_Text : String) return String is
     ("MODE: " & (if C.Wld.Holding then "holding something with arm " & Codec.Img (Natural (C.Wld.Held_Arm) + 1) else "hands empty") &
      "; without new words from you I hold still and keep my grip as it is; this segment ended on: " & Until_Text & ".");

   --  编译器要知道的、关于每个名词的事实。编号和给脑看的清单一致(1 起),0 号空着。
   function Build_Facts (C : Context) return Plan.Facts_Vectors.Vector is
      Fs : Plan.Facts_Vectors.Vector;
      Zero : Plan.Item_Facts;
   begin
      Fs.Append (Zero);
      for I in 0 .. Natural (C.Items.Length) - 1 loop
         declare
            It : constant Item := C.Items (I);
            Ft : Plan.Item_Facts;
            Kk : constant Natural := (if It.Kind in Finger | Grip then Chan.Per_Arm + It.Jaw_K else It.Which);
         begin
            Ft.Exists := It.Located or else It.Kind in Finger | Grip | Piece;
            Ft.Mine := It.Kind in Finger | Grip | Piece;
            Ft.Grip := It.Kind = Grip;
            Ft.Arm := It.Arm;
            Ft.Thing_Idx := -1;
            --  量得出它鼓出它站的那个面多少 ⇒ 才有"那个面"可言。面不是全局开关,是每个东西自己的事。
            Ft.Stands := It.Height > 0.0;
            Ft.Jaw_K := It.Jaw_K;
            if Ft.Mine then
               for T in 0 .. Natural (C.Tables.Length) - 1 loop
                  if C.Tables (T).Arm = It.Arm and then C.Tables (T).Cam = C.Cam
                    and then C.Tables (T).Chan_K = Kk
                  then
                     Ft.Thing_Idx := Integer (T);
                     exit;
                  end if;
               end loop;
            end if;
            Fs.Append (Ft);
         end;
      end loop;
      return Fs;
   end Build_Facts;

   --  把编译过的程序落成【这一小节】:所有 hold 一直带着(它们是背景约束),再取下一条动作。
   --  程序跑完了才把 Have_Prog 放掉 —— 那时候才回去问脑。这就是"少问几百次"的全部机关。
   procedure Fill_Say (C : in out Context; Answer : out Brain.Say) is
      use Lang;
      function Old_Rel (R : Rel) return String is
        (case R is
            when R_Nearer => "front", when R_Farther => "back", when R_Facing => "face",
            when others => Rel_Word (R));
      function Old_Until (E : Event) return String is
        (case E is
            when E_Touch => "contact", when E_Free => "slip", when E_None => "steps",
            when others => Event_Word (E));
      Acted : Boolean := False;
   begin
      Answer := (others => <>);
      Answer.See := To_Unbounded_String ("target");
      Answer.Fast := True;
      Answer.Until_Kind := To_Unbounded_String ("steps");
      Answer.Steps := 1;
      if C.Prog_At = 0 then
         Answer.Text := C.Prog.Says;
         if C.Prog.Look >= 1 then
            Answer.Look := C.Prog.Look;
         end if;
      end if;
      for I in 0 .. Natural (C.Prog.Goals.Length) - 1 loop
         declare
            G : constant Plan.Goal := C.Prog.Goals (I);
         begin
            if G.Hard then
               Answer.Moves.Append (Brain.Goal'(Item => G.Subject, Cell => 0, Rel => To_Unbounded_String (Old_Rel (G.R)),
                                                Of_Item => G.Object, Amount => To_Unbounded_String (Amount_Word (G.Amt)),
                                                Stay => False, Hard => True));
            elsif G.Forbid then
               Answer.Avoid.Append (Integer (G.Object));
            end if;
         end;
      end loop;
      while C.Prog_At < Natural (C.Prog.Goals.Length)
        and then (not Acted or else C.Prog.Goals (C.Prog_At).Together)
      loop
         declare
            G : constant Plan.Goal := C.Prog.Goals (C.Prog_At);
         begin
            C.Prog_At := C.Prog_At + 1;
            if G.Hard or else G.Forbid then
               null;   --  背景约束,上面已经带上了
            else
               case G.V is
                  when V_Press =>
                     Answer.Moves.Append (Brain.Goal'(Item => G.Subject, Cell => 0,
                                                      Rel => To_Unbounded_String ("press"),
                                                      Of_Item => G.Object,
                                                      Amount => To_Unbounded_String
                                                        (case G.Ef is
                                                            when F_Light => "small", when F_Firm => "medium",
                                                            when F_Hard => "large", when F_None => "small"),
                                                      Stay => False, Hard => False));
                  when V_Reach =>
                     Answer.Moves.Append (Brain.Goal'(Item => G.Subject, Cell => 0, Rel => To_Unbounded_String (Old_Rel (G.R)),
                                                      Of_Item => G.Object, Amount => To_Unbounded_String (Amount_Word (G.Amt)),
                                                      Stay => False, Hard => False));
                  when V_Close =>
                     Answer.Grip := To_Unbounded_String ("close");
                     Answer.Grip_Arm := G.Subject_Arm + 1;
                     Answer.Grip_K := G.Subject_Jaw;
                     Answer.Grip_On := G.Object;
                  when V_Open =>
                     Answer.Grip := To_Unbounded_String ("open");
                     Answer.Grip_Arm := G.Subject_Arm + 1;
                     Answer.Grip_K := G.Subject_Jaw;
                  when others =>
                     null;
               end case;
               Answer.Until_Kind := To_Unbounded_String (Old_Until (G.Ev));
               Answer.Steps := (if G.Ev = E_Steps then Natural'Max (1, G.Steps) else 0);
               Acted := True;
            end if;
         end;
      end loop;
      if not Acted then
         --  这一段跑完了:回去问下一段
         C.Have_Prog := False;
         C.Prog_At := 0;
         Answer.Done := C.Prog.Done;
         Answer.Moves.Clear;
      end if;
   end Fill_Say;

   --  ── 一轮 ──
   procedure Round (L : in out Plug.Link; F : in out Plug.Frame; C : in out Context) is
      Cam : constant Natural := Natural'Min (C.Cam, Natural (F.Cams.Length) - 1);
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      RGB : Buf := F.Cams (Cam).RGB;
      Listing : Unbounded_String;
      Say : Brain.Say;
      Err : Unbounded_String;
      Report : Unbounded_String;
   begin
      C.Round_N := C.Round_N + 1;
      C.Cam := Cam;
      Feel (C, F);   --  先感觉手在哪(按位姿查身体图),不看
      --  切块 → 世界槽
      World.Observe (C.Wld, Cam, Cut_Things (C, F, Cam), Cw, Ch);
      Draw.Grid (RGB, Cw, Ch, C.Cols, C.Rows, C.Cells_U, C.Cells_V);
      Build_Listing (C, F, Cam, RGB, Listing);
      Put_Line ("[身] ── 第" & Natural'Image (C.Round_N) & " 轮(第" & Natural'Image (Cam) & " 台相机)── 这一集已用 " & Codec.Img (Plug.Steps (L)) & " 拍(开机量身体 " & Codec.Img (C.Boot_Steps) & " 拍)");
      Put (To_String (Listing));
      if C.Dump_Dir /= "" then
         Codec.Write_BMP (To_String (C.Dump_Dir) & "/grid_" & Codec.Pad6 (C.Round_N) & ".bmp", RGB, Cw, Ch);
      end if;
      declare
         --  拍数只进日志(我们自己记账),不进问脑的话:真实世界没有"步",脑只看画面
         Recent : constant String := Memory.Text (C.Mem) & To_String (C.Recent);
      begin
         --  🔴 每一轮把【所有相机】一起给脑:编号那张在上面(格子和编号只管它),其余几只眼睛按半幅
         --  拼在下面一条。以前一轮只给一台,想看别的得先说"下一轮换一台" —— 那是【盲切】:说完看不到
         --  结果,切过去才发现手腕正对着墙(DS/EI/EA 各记过一次)。而且切一台就丢一轮,跨相机的判断
         --  根本做不了。下面那一条不画格子、不编号 —— 它只是"我另外几只眼睛现在看见什么"。
         declare
            Sh : constant Natural := Ch / 2;
            Sw : constant Natural := Cw / 2;
            Bh : constant Natural := Ch + (if C.Map.N_Cams > 1 then Sh else 0);
            Big : Buf := U8_Vectors.To_Vector (0, Ada.Containers.Count_Type (Cw * Bh * 3));
            Slot : Natural := 0;
         begin
            for I in 0 .. Cw * Ch * 3 - 1 loop
               Big.Replace_Element (I, RGB.Element (I));
            end loop;
            if C.Map.N_Cams > 1 then
               for K in 0 .. Natural (F.Cams.Length) - 1 loop
                  if K /= Cam and then Slot * Sw < Cw then
                     declare
                        Kw : constant Natural := F.Cams (K).W;
                        Kh : constant Natural := F.Cams (K).H;
                        Ox : constant Natural := Slot * Sw;
                     begin
                        if Natural (F.Cams (K).RGB.Length) >= Kw * Kh * 3 then
                           for Y in 0 .. Sh - 1 loop
                              for X in 0 .. Sw - 1 loop
                                 declare
                                    Sx : constant Natural := Natural'Min (Kw - 1, X * Kw / Sw);
                                    Sy : constant Natural := Natural'Min (Kh - 1, Y * Kh / Sh);
                                    D : constant Natural := ((Ch + Y) * Cw + Ox + X) * 3;
                                    Sp : constant Natural := (Sy * Kw + Sx) * 3;
                                 begin
                                    if Ox + X < Cw then
                                       Big.Replace_Element (D, F.Cams (K).RGB.Element (Sp));
                                       Big.Replace_Element (D + 1, F.Cams (K).RGB.Element (Sp + 1));
                                       Big.Replace_Element (D + 2, F.Cams (K).RGB.Element (Sp + 2));
                                    end if;
                                 end;
                              end loop;
                           end loop;
                           Draw.Numbered_Box (Big, Cw, Bh, Ox, Ch, Natural'Min (Cw - 1, Ox + Sw - 1), Bh - 1,
                                              K + 1, Draw.White, 2);
                        end if;
                        Slot := Slot + 1;
                     end;
                  end if;
               end loop;
            end if;
         --  🔴 脑交上来的是【一段程序】,不是一张表。收到之后:解析 → 对着体检判决编译 → 过了才存起来跑。
         --  退回是免费的:一根手指都不动,理由和一个能照抄的替代随下一轮一起给它。
         if not C.Have_Prog then
            declare
               Text : Unbounded_String;
            begin
               if not Brain.Ask (To_String (C.Eye_Host), C.Eye_Port, To_String (C.Task_Text), To_String (Listing), Recent,
                                 Lang.Grammar, To_String (C.Refused),
                                 C.Cols, C.Rows, Natural (C.Items.Length), C.Map.N_Cams, C.Map.Arms, Big, Cw, Bh, Text, Err)
               then
                  Put_Line ("[身] 🧠 问不通(" & To_String (Err) & ")⇒ 这一拍不动,下一拍重问");
                  return;
               end if;
               Put_Line ("[身] 🧠 它交上来一段程序:");
               Put_Line (To_String (Text));
               declare
                  Rep : constant Exam.Report := Exam.Judge (C.Map, C.Tables);
                  Cm : constant Plan.Compiled :=
                    Plan.Compile (Lang.Parse (To_String (Text)), Rep, Build_Facts (C));
               begin
                  Put_Line ("[身] ⚖ " & Plan.Report_Text (Cm));
                  if not Cm.Ok then
                     C.Refused := S ("line " & Codec.Img (Cm.Err_Line) & ": " & To_String (Cm.Err)
                                     & (if Length (Cm.Instead) > 0 then "  -> " & To_String (Cm.Instead) else ""));
                     C.Recent := S ("I refused your program before anything moved. " & To_String (C.Refused)
                                    & " Nothing has moved. " & Mode_Line (C, "refused"));
                     return;
                  end if;
                  C.Refused := Null_Unbounded_String;
                  C.Prog := Cm;
                  C.Prog_At := 0;
                  C.Have_Prog := True;
               end;
            end;
         end if;
         Fill_Say (C, Say);
         end;
      end;
      Put_Line ("[身] 🧠 它说:" & To_String (Say.Text) & " ‖ 看见=" & To_String (Say.See) & " · 动" & Natural'Image (Natural (Say.Moves.Length)) &
                " 条 · 抓握=" & To_String (Say.Grip) & (if Say.Grip_Arm > 0 then "(第" & Natural'Image (Say.Grip_Arm) & " 只手" & (if Say.Grip_On > 0 then ",在第" & Natural'Image (Say.Grip_On) & " 号上" else "") & ")" else "") &
                " · 到 " & To_String (Say.Until_Kind) & (if Say.Until_Kind = "steps" then Natural'Image (Say.Steps) & " 步" else "") & " 为止" &
                (if Say.Fast then " · 快" else "") & (if Say.Done then " · 它说已经做完了" else ""));
      --  它点名的世界块 ⇒ 记槽号
      for G of Say.Moves loop
         declare
            Ns : constant array (1 .. 2) of Natural := [G.Of_Item, G.Item];
         begin
         for N of Ns loop
            if N >= 1 and then N <= Natural (C.Items.Length) and then C.Items (N - 1).Kind in Thing | Thing_Remembered | Thing_Held then
               declare
                  Cs : World.Cam_State := C.Wld.Cams (Cam);
               begin
                  Cs.Named := C.Items (N - 1).Slot;
                  C.Wld.Cams.Replace_Element (Cam, Cs);
               end;
            end if;
         end loop;
         end;
      end loop;
      if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) and then C.Items (Say.Grip_On - 1).Kind in Thing | Thing_Remembered then
         declare
            Cs : World.Cam_State := C.Wld.Cams (Cam);
         begin
            Cs.Named := C.Items (Say.Grip_On - 1).Slot;
            C.Wld.Cams.Replace_Element (Cam, Cs);
         end;
      end if;
      C.Fast := Say.Fast;
      if Say.Look >= 2 then
         declare
            K : Natural := 2;
         begin
            for Ci in 0 .. C.Map.N_Cams - 1 loop
               if Ci /= Cam then
                  if K = Say.Look then
                     C.Cam := Ci;
                     Put_Line ("[身]    它要换到第" & Natural'Image (Ci) & " 台相机 ⇒ 下一轮在那台里列块、问、执行");
                  end if;
                  K := K + 1;
               end if;
            end loop;
         end;
      end if;
      if Say.Done then
         C.Recent := S ("you said it is already done. the body did nothing and is looking again. " & Mode_Line (C, "you said done"));
         return;
      end if;
      if C.Look_Only then
         C.Recent := S ("(look only) I did not move. " & Mode_Line (C, "look only"));
         return;
      end if;
      --  ── 执行 ──
      declare
         Until_K : constant Monitor.Until_Kind :=
           (if Say.Until_Kind = "contact" then Monitor.U_Contact elsif Say.Until_Kind = "resist" then Monitor.U_Resist
            elsif Say.Until_Kind = "slip" then Monitor.U_Slip elsif Say.Until_Kind = "settle" then Monitor.U_Settle else Monitor.U_Steps);
         Step_Limit : constant Natural := (if Say.Until_Kind = "steps" then Natural'Max (1, Say.Steps) else 0);
         Avoid : Item_Vectors.Vector;
         Pts : Point_Vectors.Vector;
         Amount : Long_Float := 1.0;   --  没说 amount 时用满(比例);说了按它的
         Event : Unbounded_String;
         Steps_Taken : Natural := 0;
         Beats : Natural := 0;
         Blocked : Boolean;
         Desc : Unbounded_String;
         Grip_Arm : constant Integer := (if Say.Grip_Arm >= 1 and then Say.Grip_Arm <= C.Map.Arms then Integer (Say.Grip_Arm) - 1 else -1);
         Did_Grip : Unbounded_String;
      --  2a 把脑说的话变成要求:别动的,目标就是它现在的位置;要动的,目标是格子或与某号的关系;
      --  抓某号,目标是"和我张开的那片地方重合"(位置 / 远近 / 看着多大 / 朝向)
      --  把去哪翻成目标:格子 / 与某号的关系(碰到它 · 上下左右 · 前后 · 离远点)。全是量出来的位置,没有写死的距离
      procedure Set_Target (G : Brain.Goal; P : in out Point; Ok_Pt : in out Boolean) is
      begin
                           --  目标
                           if G.Cell >= 1 and then G.Cell <= Natural (C.Cells_U.Length) then
                              P.Tu := C.Cells_U (G.Cell - 1); P.Tv := C.Cells_V (G.Cell - 1); P.Tz := P.Z; P.Wz := 0.0;
                              P.Desc := S ("item " & Codec.Img (G.Item) & " to cell " & Codec.Img (G.Cell));
                           elsif G.Rel /= "" and then G.Of_Item >= 1 and then G.Of_Item <= Natural (C.Items.Length) then
                              declare
                                 O : constant Item := C.Items (G.Of_Item - 1);
                                 Ow : constant Long_Float := Long_Float (O.X1 - O.X0) / Long_Float (Cw);
                                 Oh : constant Long_Float := Long_Float (O.Y1 - O.Y0) / Long_Float (Ch);
                                 Rl : constant String := To_String (G.Rel);
                              begin
                                 if not O.Located then
                                    Report := S ("goal: item " & Codec.Img (G.Of_Item) & " is not locatable right now. ");
                                    Ok_Pt := False;
                                 else
                                    P.Desc := S ("item " & Codec.Img (G.Item) & " " & Rl & " item " & Codec.Img (G.Of_Item));
                                    P.Tu := O.Cu; P.Tv := O.Cv; P.Tz := P.Z; P.Wz := 0.0;
                                    if Rl = "at" then
                                       if P.Kind = Thing_Pt and then O.Kind in Finger | Grip then
                                          --  X 装进握区:区心、区深
                                          declare
                                             Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam, Jaw_K_Of (P.Chan_K));
                                          begin
                                             P.Tu := Z.Cu; P.Tv := Z.Cv; P.Tz := Z.Depth; P.Wz := (if Picture.Is_Nan (Z.Depth) then 0.0 else 1.0);
                                          end;
                                       elsif P.Kind = Piece_Pt and then O.Kind in Thing | Thing_Remembered then
                                          --  "到它那儿" = 到它那一面,不替它挑高低(owner 2026-09-08:"半腰"假设了两指从侧面夹一个立在台面上的东西,
                                          --  吸盘、拆弹的线上不成立)。要更低就由脑说"下",词表里有
                                          P.Tz := O.Depth; P.Wz := (if O.Depth > 0.0 then 1.0 else 0.0);
                                       else
                                          P.Tz := O.Depth; P.Wz := (if O.Depth > 0.0 and then P.Z > 0.0 then 1.0 else 0.0);
                                       end if;
                                    elsif Rl = "above" then
                                       P.Tv := O.Cv - Long_Float'Max (Oh, 1.0 / Long_Float (Ch));
                                    elsif Rl = "below" then
                                       P.Tv := O.Cv + Long_Float'Max (Oh, 1.0 / Long_Float (Ch));
                                    elsif Rl = "left" then
                                       P.Tu := O.Cu - Long_Float'Max (Ow, 1.0 / Long_Float (Cw));
                                    elsif Rl = "right" then
                                       P.Tu := O.Cu + Long_Float'Max (Ow, 1.0 / Long_Float (Cw));
                                    elsif Rl = "front" or else Rl = "back" then
                                       declare
                                          --  "一截" = 它自己在画面里的宽 × 它的深度(米,全是量的),没量到深度就用它鼓起的高度
                                          Sz : constant Long_Float := Long_Float'Max (O.Height, Long_Float'Max (P.Height, Ow * O.Depth));
                                       begin
                                          P.Tz := (if Rl = "front" then O.Depth - Sz else O.Depth + Sz);
                                          P.Wz := (if O.Depth > 0.0 and then P.Z > 0.0 then 1.0 else 0.0);
                                       end;
                                    elsif Rl = "away" then
                                       declare
                                          Du : constant Long_Float := P.Cu - O.Cu;
                                          Dv : constant Long_Float := P.Cv - O.Cv;
                                          Ln : constant Long_Float := Long_Float'Max (1.0e-9, Sqrt (Du * Du + Dv * Dv));
                                          St : constant Long_Float := Long_Float'Max (Ow, 1.0 / Long_Float (Cw));
                                       begin
                                          P.Tu := P.Cu + Du / Ln * St; P.Tv := P.Cv + Dv / Ln * St;
                                       end;
                                    elsif Rl = "onto" or else Rl = "off" or else Rl = "press" then
                                       --  它站的那个面在哪:它自己的深度 + 它鼓出多少(两个都是量出来的)。
                                       --  onto = 压到那个面那么深;off = 反过来离开那个面它自己那么高一截。
                                       --  press = 朝那个面【压过去一个到不了的深度】:走不到的那一截就是力。
                                       --  这具身体的观测里没有力那一路,所以"劲"只能是命令与实到之差 —— 那是任何身体都有的。
                                       declare
                                          Floor_Z : constant Long_Float := O.Depth + O.Height;
                                       begin
                                          if O.Depth > 0.0 and then P.Z > 0.0 and then O.Height > 0.0 then
                                             P.Tu := P.Cu; P.Tv := P.Cv;
                                             P.Tz := (if Rl = "off" then O.Depth - O.Height
                                                      elsif Rl = "onto" then Floor_Z
                                                      else Floor_Z + O.Height * Amount);
                                             P.Wz := 1.0;
                                          else
                                             Report := S ("goal: I cannot measure how far item " & Codec.Img (G.Of_Item)
                                                          & " stands out of what it rests on, so I do not know which way is into it. ");
                                             Ok_Pt := False;
                                          end if;
                                       end;
                                    elsif Rl = "face" then
                                       --  转到"从我这一点指向它那一点"的方向 = 我这一块的主轴方向。人不动,只转。
                                       --  存两倍角(和"朝哪"那一行同一个约定:主轴的正负两种写法算同一个)。
                                       declare
                                          Du : constant Long_Float := O.Cu - P.Cu;
                                          Dv : constant Long_Float := O.Cv - P.Cv;
                                       begin
                                          if abs Du > 0.0 or else abs Dv > 0.0 then
                                             P.Tu := P.Cu; P.Tv := P.Cv; P.Wz := 0.0;
                                             P.Tang := Wrap (2.0 * Arctan (Dv, Du));
                                             P.Wang := 1.0;
                                          else
                                             Report := S ("goal: item " & Codec.Img (G.Item) & " and item " & Codec.Img (G.Of_Item)
                                                          & " sit at the same spot in the picture, so there is no direction to turn to. ");
                                             Ok_Pt := False;
                                          end if;
                                       end;
                                    else
                                       --  🔴 认不得的关系【不许静悄悄地什么都不做】。词表长出一个新词而执行器还没实现它,
                                       --  静默 no-op 会让脑以为它说的话被执行了 —— 这正是整套设计要杀掉的那一类失败。
                                       Report := S ("goal: I do not know where the relation " & Rl & " would put me. ");
                                       Ok_Pt := False;
                                    end if;
                                 end if;
                              end;
                           else
                              Ok_Pt := False;
                           end if;
      end Set_Target;

      procedure Build_Goals is
      begin
            --  "保持不动" → 也变成一个点:它的目标就是它此刻在哪(误差为零)。这样解算必须选一条【不动它】的走法,
            --  而不是"没人管它" —— 一只手捏住不动、另一只手干活,全靠这一条(以前这种条目被直接跳过)
            for G of Say.Moves loop
               if G.Item >= 1 and then G.Item <= Natural (C.Items.Length) and then G.Stay then
                  declare
                     It : constant Item := C.Items (G.Item - 1);
                     P : Point;
                  begin
                     if It.Located then
                        P.Item_No := G.Item;
                        if It.Kind in Finger | Grip | Piece | Thing_Held then
                           P.Arm := It.Arm; P.Kind := Piece_Pt;
                           P.Chan_K := (if It.Kind = Piece then It.Which else Chan.Per_Arm + It.Jaw_K);
                           declare
                              Tr : constant Zone_Track := C.Zones (Track_Idx (C, P.Arm, Cam));
                           begin
                              P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z; P.Known := Tr.Known;
                           end;
                        elsif Cam_Arm (C, Cam) >= 0 then
                           P.Arm := Natural (Cam_Arm (C, Cam)); P.Kind := Thing_Pt; P.Slot := It.Slot;
                           P.Cu := It.Cu; P.Cv := It.Cv; P.Z := It.Depth; P.Height := It.Height; P.Count := It.Count;
                           P.Box_W := Long_Float (It.X1 - It.X0) / Long_Float (Cw); P.Box_H := Long_Float (It.Y1 - It.Y0) / Long_Float (Ch);
                           P.Elong := It.Elong; P.Gray := It.Gray;
                        end if;
                        P.Tu := P.Cu; P.Tv := P.Cv; P.Tz := P.Z;
                        P.Wz := (if P.Z > 0.0 then 1.0 else 0.0);
                        P.Desc := S ("item " & Codec.Img (G.Item) & " stays exactly where it is");
                        if Pts.Is_Empty or else Pts (0).Arm = P.Arm then
                           Pts.Append (P);
                        end if;
                     end if;
                  end;
               end if;
               if G.Item >= 1 and then G.Item <= Natural (C.Items.Length) and then not G.Stay then
                  declare
                     It : constant Item := C.Items (G.Item - 1);
                     P : Point;
                     Own : constant Boolean := It.Kind in Finger | Grip | Thing_Held | Piece;
                     Cam_A : constant Integer := Cam_Arm (C, Cam);
                     Ok_Pt : Boolean := True;
                  begin
                     Amount := Amount_Factor (G.Amount);
                     P.Item_No := G.Item;
                     P.Hard := G.Hard;   --  脑说的是 hold ⇒ 这一条进硬约束,解算时不许被牺牲
                     if not It.Located then
                        Report := S ("goal: item " & Codec.Img (G.Item) & " is not locatable in this picture right now. ");
                        Ok_Pt := False;
                     elsif It.Kind = Piece then
                        --  我身上的一块零件:点 = 它此刻的形心,表按需量(六个通道各推一下)
                        P.Arm := It.Arm; P.Kind := Piece_Pt; P.Chan_K := It.Which; P.Blob := -1;
                        P.Cu := It.Cu; P.Cv := It.Cv; P.Z := It.Depth;
                        P.Known := C.Zones (Track_Idx (C, P.Arm, Cam)).Pieces_Known (It.Which);
                        P.Box_W := Long_Float (It.X1 - It.X0) / Long_Float (Cw); P.Box_H := Long_Float (It.Y1 - It.Y0) / Long_Float (Ch);
                     elsif Own then
                        P.Arm := It.Arm; P.Kind := Piece_Pt; P.Chan_K := Chan.Per_Arm + It.Jaw_K;   --  手指 = 那个抓握通道带的那块
                        declare
                           Tr : constant Zone_Track := C.Zones (Track_Idx (C, P.Arm, Cam));
                        begin
                           P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z; P.Known := Tr.Known or else Cam_A = Integer (P.Arm);
                        end;
                        if Cam_A = Integer (P.Arm) and then G.Rel /= "" and then G.Of_Item >= 1 and then G.Of_Item <= Natural (C.Items.Length)
                          and then C.Items (G.Of_Item - 1).Kind = Thing
                        then
                           --  自己的手上相机里"我的手到 X" = 让 X 的像素来到握区:改跟 X
                           declare
                              O : constant Item := C.Items (G.Of_Item - 1);
                           begin
                              P.Kind := Thing_Pt; P.Slot := O.Slot; P.Cu := O.Cu; P.Cv := O.Cv; P.Z := O.Depth; P.Height := O.Height; P.Count := O.Count;
                              P.Box_W := Long_Float (O.X1 - O.X0) / Long_Float (Cw); P.Box_H := Long_Float (O.Y1 - O.Y0) / Long_Float (Ch);
                           end;
                        end if;
                     elsif It.Kind = Thing and then Cam_A >= 0 then
                        P.Arm := Natural (Cam_A); P.Kind := Thing_Pt; P.Slot := It.Slot;
                        P.Cu := It.Cu; P.Cv := It.Cv; P.Z := It.Depth; P.Height := It.Height; P.Count := It.Count;
                        P.Box_W := Long_Float (It.X1 - It.X0) / Long_Float (Cw); P.Box_H := Long_Float (It.Y1 - It.Y0) / Long_Float (Ch);
                     else
                        Report := S ("goal: item " & Codec.Img (G.Item) & " is a thing I am not holding; I can only move things I hold (say grip close on it first). ");
                        Ok_Pt := False;
                     end if;
                     if Ok_Pt then
                        Set_Target (G, P, Ok_Pt);
                     end if;
                     if Ok_Pt then
                        if Pts.Is_Empty or else Pts (0).Arm = P.Arm then
                           Pts.Append (P);
                        else
                           Report := Report & "goal for item " & Codec.Img (G.Item) & " needs a different arm than the first goal; I do one arm per segment. ";
                        end if;
                     end if;
                  end;
               end if;
            end loop;
            --  抓握 close 在某块上:先把那块装进握区(手上相机里跟块;否则世界相机里握区去它的顶面),笼住了才合
            if Say.Grip = "close" and then Grip_Arm >= 0 and then Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length)
              and then C.Items (Say.Grip_On - 1).Kind in Thing | Thing_Remembered and then C.Items (Say.Grip_On - 1).Located
            then
               declare
                  O : constant Item := C.Items (Say.Grip_On - 1);
                  A : constant Natural := Natural (Grip_Arm);
                  Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
                  P : Point;
                  Own_Cam : constant Boolean := Cam_Arm (C, Cam) = Integer (A);
               begin
                  if Pts.Is_Empty and then Z.Valid then
                     P.Arm := A; P.Item_No := Say.Grip_On;
                     if Own_Cam then
                        --  🔴 "抓住它" = 让它在画面里和我张开的那片地方重合:位置对上、远近对上、看着一样大、朝向一样。
                        --  这四件全是量出来的区域属性,一个字没提手指几根 —— 两指、七指、吸盘、软体臂同一句话。
                        --  只对中心那一版是错的:三个数管不住六个通道,剩下的自由度乱走(手腕拧、球转出画面)。
                        --  "看着一样大"顺带就是最稳的远近信号(离得越近越大),比一步只变 5 mm、自己抖 5 mm 的深度读数强。
                        --  圆的东西没有朝向 ⇒ 那一行谁也改不动 ⇒ 自动不参与,不需要写规则。
                        P.Kind := Thing_Pt; P.Slot := O.Slot; P.Cu := O.Cu; P.Cv := O.Cv; P.Z := O.Depth; P.Height := O.Height; P.Count := O.Count;
                        P.Box_W := Long_Float (O.X1 - O.X0) / Long_Float (Cw); P.Box_H := Long_Float (O.Y1 - O.Y0) / Long_Float (Ch);
                        P.Size := Sqrt (Long_Float'Max (0.0, P.Box_W * P.Box_H));
                        P.Ang := 2.0 * Arctan (O.Av, O.Au);
                        P.Elong := O.Elong; P.Gray := O.Gray;
                        P.Tu := Z.Cu; P.Tv := Z.Cv; P.Tz := Z.Depth; P.Wz := (if Picture.Is_Nan (Z.Depth) then 0.0 else 1.0);
                        P.Tsize := Sqrt (Long_Float'Max (0.0, (Long_Float (Z.X1 - Z.X0) / Long_Float (Cw)) * (Long_Float (Z.Y1 - Z.Y0) / Long_Float (Ch))));
                        P.Tang := 2.0 * Arctan (Z.Av, Z.Au);
                        --  ⚠️ "看着多大"这一项【实测不稳,先关掉】(2026-09-08):框随切块忽大忽小,一项就把目标和进度全带偏,
                        --  每一步都是它在变坏;今天唯一真的靠近过的那一炮(32 cm → 16 cm)恰恰没有这一项。
                        --  机制(五行的表)留着,等切块稳了再开。
                        P.Wsize := 0.0;
                        --  朝向的分量 = 这块有多长条(圆的为零)
                        P.Wang := Long_Float'Max (0.0, 1.0 - 1.0 / Long_Float'Max (1.0, O.Elong));
                        P.Desc := S ("item " & Codec.Img (Say.Grip_On) & " to sit where my fingers close (same place, same distance, same apparent size, same lie)");
                     else
                        declare
                           Tr : constant Zone_Track := C.Zones (Track_Idx (C, A, Cam));
                        begin
                           P.Kind := Piece_Pt; P.Chan_K := Chan.Per_Arm; P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z; P.Known := Tr.Known;
                           --  同上:合到它那一面,高低由脑说了算
                           P.Tu := O.Cu; P.Tv := O.Cv; P.Tz := O.Depth; P.Wz := (if O.Depth > 0.0 and then Tr.Z > 0.0 then 1.0 else 0.0);
                           P.Desc := S ("grip " & Codec.Img (A + 1) & " onto item " & Codec.Img (Say.Grip_On) & " (fingertips to its middle)");
                        end;
                     end if;
                     Pts.Append (P);
                  end if;
               end;
            end if;
      end Build_Goals;

      --  2c 抓握:先看笼住没有,合到画面不再变,再抬一截量结果(跟我走了没有 / 我没推的东西动了几件 / 原地剩几块)
      procedure Do_Grip is
      begin
            --  ── 抓握 ──
            if Say.Grip = "close" and then Grip_Arm >= 0 then
               declare
                  A : constant Natural := Natural (Grip_Arm);
                  Caged : Boolean := True;
                  Cage_Note : Unbounded_String;
                  Steps_J : Natural;
                  Reading : Long_Float;
                  Hz : constant Zone.Hand_Zone := Zone_Of (C, A, Cam, Say.Grip_K);
               begin
                  --  笼判据:点名的那块的像素在握区框里(它的形心落在区框内),深度和手指对得上
                  if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then
                     declare
                        Pin : Point;
                        Found : Boolean := False;
                        Nz : Long_Float := 0.0;
                     begin
                        for P of Pts loop
                           if P.Item_No = Say.Grip_On then
                              if P.Kind = Piece_Pt then
                                 --  瓣点取均值 = 区心;深度取最近的那一瓣
                                 if not Found then
                                    Pin := P; Pin.Cu := 0.0; Pin.Cv := 0.0; Pin.Z := 1.0e30;
                                 end if;
                                 Pin.Cu := Pin.Cu + P.Cu; Pin.Cv := Pin.Cv + P.Cv; Nz := Nz + 1.0;
                                 if P.Z > 0.0 then
                                    Pin.Z := Long_Float'Min (Pin.Z, P.Z);
                                 end if;
                              else
                                 Pin := P;
                              end if;
                              Found := True;
                           end if;
                        end loop;
                        if Found and then Pin.Kind = Piece_Pt and then Nz > 0.0 then
                           Pin.Cu := Pin.Cu / Nz; Pin.Cv := Pin.Cv / Nz;
                           --  1e29 = "没读到"的哨兵(无量纲)
                           if Pin.Z >= 1.0e29 then
                              Pin.Z := 0.0;
                           end if;
                        end if;
                        if Found and then Pin.Kind = Thing_Pt and then Hz.Valid then
                           --  笼住 = 它已经和我张开的那片地方重合:画面里位置进了跟踪噪声、看着一样大、远近对得上。
                           --  只看"中心在区框里"不够 —— 手上相机里区框就是整个下半幅,那条判据恒真(EI/EM 实测)
                           declare
                              Dp : constant Long_Float := Sqrt ((Pin.Tu - Pin.Cu) ** 2 + (Pin.Tv - Pin.Cv) ** 2);
                              Ds : constant Long_Float := (if Pin.Wsize > 0.0 and then Pin.Tsize > 0.0 then abs (Pin.Tsize - Pin.Size) / Pin.Tsize else 0.0);
                              Tol : constant Long_Float := Long_Float'Max (Track_Win * 0.5, Hz.Span * 0.25);
                              Depth_Ok : constant Boolean := Picture.Is_Nan (Hz.Depth) or else Pin.Z <= 0.0
                                                            or else abs (Pin.Z - Hz.Depth) <= Long_Float'Max (Pin.Height, Long_Float'Max (Pin.Box_W, Pin.Box_H) * Pin.Z);
                              --  看着一样大 = 差不超过四分之一(比例,无量纲)
                              Size_Ok : constant Boolean := Pin.Wsize <= 0.0 or else Ds <= 0.25;
                           begin
                              Caged := Dp <= Tol and then Depth_Ok and then Size_Ok;
                              Cage_Note := S ("cage check in this hand camera: it is " & Codec.Fmt (Dp, 3) & " of a frame from where my fingers close (allowed " &
                                              Codec.Fmt (Tol, 3) & "), looks " & Codec.Fmt (Pin.Size / Long_Float'Max (1.0e-9, Pin.Tsize) * 100.0, 0) &
                                              "% of the size it should, and its distance " & (if Depth_Ok then "matches" else "does not match") & " my fingertips");
                           end;
                        elsif Found and then Pin.Kind = Piece_Pt then
                           declare
                              O : constant Item := C.Items (Say.Grip_On - 1);
                              Dist : constant Long_Float := Sqrt ((Pin.Cu - O.Cu) ** 2 + (Pin.Cv - O.Cv) ** 2);
                              Tol : constant Long_Float := Long_Float'Max (Hz.Span * 0.5, Track_Win * 0.5);
                           begin
                              Caged := Dist <= Tol;
                              Cage_Note := S ("cage check in this camera: my grip centre is " & Codec.Fmt (Dist, 3) & " of a frame from the thing (allowed " & Codec.Fmt (Tol, 3) & ")");
                           end;
                        end if;
                     end;
                  end if;
                  if Caged then
                     Move_Jaw (L, C, F, A, 0.0, Steps_J, Reading, Say.Grip_K);
                     declare
                        Empty : constant Long_Float := C.Hands (A).Empty_Close;
                        By_Reading : Boolean := Reading - Empty > C.Map.Jaw_Noise;
                        Sure_Held : Boolean := False;
                        Note : Unbounded_String;
                        Origin : Picture.Region;
                        Obj_Count : Natural := 0;
                     begin
                        if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then
                           Origin := World.Get (C.Wld, Cam, Natural (C.Items (Say.Grip_On - 1).Slot)).Shadow;
                           Obj_Count := C.Items (Say.Grip_On - 1).Count;
                        end if;
                        Held_Test (L, C, F, A, Cam, Origin, Obj_Count, By_Reading, Sure_Held, Note);
                        if not Sure_Held then
                           By_Reading := False;   --  说不准 ⇒ 不许记成"手里有东西"(记错了下一步它就去"搬"而不是重抓)
                        end if;
                        Did_Grip := S ("I closed grip " & Codec.Img (A + 1) & " until the picture stopped changing (" & Codec.Img (Steps_J) & " steps, reading " & Codec.Fmt (Reading, 3) &
                                       ", empty-close reading " & Codec.Fmt (Empty, 3) & "); " & To_String (Note));
                        if By_Reading then
                           C.Wld.Holding := True; C.Wld.Held_Arm := Integer (A); C.Wld.Held_Jaw := Integer (Say.Grip_K); C.Wld.Held_Cam := Integer (Cam);
                           if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then
                              C.Wld.Held_Slot := C.Items (Say.Grip_On - 1).Slot;
                              C.Wld.Held_Origin := World.Get (C.Wld, Cam, Natural (C.Items (Say.Grip_On - 1).Slot)).Shadow;
                           else
                              C.Wld.Held_Slot := -1;
                           end if;
                           Memory.Set (C.Mem, "holding", "arm " & Codec.Img (A + 1) & " closed on item " & Codec.Img (Say.Grip_On) & " at reading " & Codec.Fmt (Reading, 3));
                        else
                           C.Wld.Holding := False; C.Wld.Held_Arm := -1; C.Wld.Held_Jaw := -1;
                           Move_Jaw (L, C, F, A, Hand_Of (C, A, Say.Grip_K).Open_Reading, Steps_J, Reading, Say.Grip_K);
                           Append (Did_Grip, "; I opened it again");
                        end if;
                     end;
                  else
                     Did_Grip := S ("I did NOT close grip " & Codec.Img (A + 1) & ": " & To_String (Cage_Note));
                  end if;
                  if Cage_Note /= "" and then Caged then
                     Append (Did_Grip, " (" & To_String (Cage_Note) & ")");
                  end if;
               end;
            elsif Say.Grip = "open" and then Grip_Arm >= 0 then
               declare
                  A : constant Natural := Natural (Grip_Arm);
                  Steps_J : Natural;
                  Reading : Long_Float;
               begin
                  Move_Jaw (L, C, F, A, Hand_Of (C, A, Say.Grip_K).Open_Reading, Steps_J, Reading, Say.Grip_K);
                  C.Wld.Holding := False; C.Wld.Held_Arm := -1; C.Wld.Held_Jaw := -1; C.Wld.Held_Slot := -1;
                  Memory.Set (C.Mem, "holding", "");
                  Did_Grip := S ("I opened grip " & Codec.Img (A + 1) & " (" & Codec.Img (Steps_J) & " steps, reading " & Codec.Fmt (Reading, 3) & ")");
               end;
            end if;
            if Did_Grip /= "" then
               Report := Report & To_String (Did_Grip) & ". ";
            end if;
      end Do_Grip;

      begin
         for N of Say.Avoid loop
            if N >= 1 and then N <= Natural (C.Items.Length) then
               Avoid.Append (C.Items (N - 1));
            end if;
         end loop;
         Build_Goals;
         if not Pts.Is_Empty then
            Expand_Lobes (C, F, Cam, Pts);
            --  生地先看一眼:我的手/零件在这台相机里的位置若只是按关节推的(没真看过),先抖/推一下认清,再按认清的位置重算各团目标,再量表、再走
            --  (EI:按关节推的手指位置差 0.2 画幅,在错地方量表 ⇒ 六列全空 ⇒ 一步没走)
            declare
               Need_Look : Boolean := False;
            begin
               for P of Pts loop
                  if P.Kind = Piece_Pt and then Cam_Arm (C, Cam) /= Integer (P.Arm) and then not P.Known then
                     Need_Look := True;
                  end if;
               end loop;
               if Need_Look then
                  Put_Line ("[身] 生地:我的手/零件在这台相机里的位置只是按关节推的 ⇒ 先动一下认清自己再走");
                  Refind_Pieces (L, C, F, Cam, Pts);
                  Feel (C, F);
                  declare
                     Su, Sv, N : Long_Float := 0.0;
                  begin
                     for P of Pts loop
                        if P.Kind = Piece_Pt and then P.Blob >= 0 and then not P.Lost then
                           Su := Su + P.Cu; Sv := Sv + P.Cv; N := N + 1.0;
                        end if;
                     end loop;
                     if N > 0.0 then
                        for I in 0 .. Natural (Pts.Length) - 1 loop
                           declare
                              P : Point := Pts (I);
                           begin
                              if P.Kind = Piece_Pt and then P.Blob >= 0 and then not P.Lost then
                                 P.Tu := P.Par_Tu + (P.Cu - Su / N); P.Tv := P.Par_Tv + (P.Cv - Sv / N);
                                 Pts.Replace_Element (I, P);
                              end if;
                           end;
                        end loop;
                     end if;
                  end;
                  declare
                     Lost_Any : Boolean := False;
                  begin
                     for P of Pts loop
                        if P.Lost then
                           Lost_Any := True;
                        end if;
                     end loop;
                     if Lost_Any then
                        Report := Report & "I moved my own piece to find it in this picture and could not see it, so I did not move toward the goal. ";
                        Pts.Clear;
                     end if;
                  end;
               end if;
            end;
            for P of Pts loop
               if P.Desc /= "" then
                  Append (Desc, (if Desc = "" then "" else " and ") & To_String (P.Desc));
               end if;
            end loop;
            if not Pts.Is_Empty then
               Put_Line ("[身] ⚙ 一起解" & Natural'Image (Natural (Pts.Length)) & " 条:" & To_String (Desc));
               Run_Segment (L, C, F, Cam, Pts, Until_K, Step_Limit, Amount, Avoid, Event, Steps_Taken, Blocked, Beats);
               Feel (C, F);
               Report := Report & "you asked " & Desc & ": " & Event & ". I took " & Codec.Img (Steps_Taken) & " pushes; ";
               Put_Line ("[身]   这一段:" & Codec.Img (Steps_Taken) & " 推 · " & Codec.Img (Beats) & " 拍 · 这一集累计 " & Codec.Img (Plug.Steps (L)) & " 拍");
               for P of Pts loop
                  if P.Blob <= 0 then
                     Report := Report & "item " & Codec.Img (P.Item_No) & (if P.Blob = 0 then " (finger A)" else "") & " now at (" & Codec.Fmt (P.Cu, 2) & "," & Codec.Fmt (P.Cv, 2) &
                               ") depth " & Codec.Fmt (P.Z, 2) & ", still " & Codec.Fmt (P.Steps_Err, 1) & " pushes away; ";
                  end if;
               end loop;
            else
               Event := S ("lost: could not see my own piece after moving it");
            end if;
         elsif Say.Moves.Is_Empty and then Say.Grip = "none" then
            Report := S ((if Say.See = "not_here" then "you said the thing is not in that picture; the body did not move. "
                          elsif Say.See = "unclear" then "you said you could not tell; the body did not move. "
                          else "you gave no move and no grip; the body did not move. "));
         end if;
         Do_Grip;
         Report := Report & Mode_Line (C, To_String (Event));
      end;
      C.Recent := Report;
      Put_Line ("[身]   ⇒ " & To_String (Report));
   end Round;
end Act;
