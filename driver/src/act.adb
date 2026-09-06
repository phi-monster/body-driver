with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Codec;
with Draw;
with Chan;
with Flow;
with Monitor;
with Backup;
package body Act is
   Sigma_Mult : constant Long_Float := 3.0;   --  鼓出来超过背景自己稳健 σ 的几倍才算一块(在真实深度图上验过:3 中,5 杀光);无量纲
   Track_Win : constant Long_Float := 0.10;   --  一步里任何被跟踪的点在画面里最多跑十分之一画幅(跟踪窗,比例,无量纲)
   Cap_Mult : constant Long_Float := 8.0;     --  一步命令上限 = 看得见的探针幅度的几倍(倍数,无量纲)
   Step_Cap : constant := 60;                 --  一段最多几步(安全上限,不是策略)

   function S (X : String) return Unbounded_String renames To_Unbounded_String;

   function Zone_Of (C : Context; Arm, Cam : Natural) return Zone.Hand_Zone is
   begin
      if Arm < Natural (C.Hands.Length) and then Cam < Natural (C.Hands (Arm).Zones.Length) then
         return C.Hands (Arm).Zones (Cam);
      end if;
      return (others => <>);
   end Zone_Of;

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

   function Cut_Things (C : Context; F : Plug.Frame; Cam : Natural) return Picture.Regions is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
      Raw : Picture.Regions;
      Kept : Picture.Regions;
   begin
      if not F.Cams (Cam).Has_Depth then
         return Kept;
      end if;
      Raw := Picture.Cut (F.Cams (Cam).Depth, Cw, Ch, Cut_Window (C, Cam, F), Sigma_Mult);
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
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
            Tr : constant Zone_Track := (if Track_Idx (C, A, Cam) < Natural (C.Zones.Length) then C.Zones (Track_Idx (C, A, Cam)) else (others => <>));
            Own_Cam : constant Boolean := Cam_Arm (C, Cam) = Integer (A);
            Du : constant Long_Float := (if Own_Cam then 0.0 else Tr.Cu - Z.Cu);
            Dv : constant Long_Float := (if Own_Cam then 0.0 else Tr.Cv - Z.Cv);
            procedure Finger (Lb : Zone.Lobe; Which : Natural) is
               It : Item;
            begin
               It.Kind := Finger; It.Arm := A; It.Which := Which;
               if Z.Valid and then Lb.Valid and then Tr.Valid then
                  It.Located := True;
                  It.Cu := Lb.Cu + Du; It.Cv := Lb.Cv + Dv;
                  It.X0 := Natural (Long_Float'Max (0.0, Long_Float (Lb.X0) + Du * Long_Float (Cw)));
                  It.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), Long_Float (Lb.X1) + Du * Long_Float (Cw))));
                  It.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Lb.Y0) + Dv * Long_Float (Ch)));
                  It.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), Long_Float (Lb.Y1) + Dv * Long_Float (Ch))));
                  It.Depth := Tr.Z; It.Count := Lb.Count;
                  Push (It, "a finger of arm " & Codec.Img (A + 1) & " (it moves when that arm's grip channel moves), now in cell " &
                        Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & Rel (It.Cu, It.Cv), Draw.Orange, 2);
               else
                  Push (It, "a finger of arm " & Codec.Img (A + 1) & " - NOT locatable in this picture right now, do not name it", Draw.Orange, 0);
               end if;
            end Finger;
            G : Item;
         begin
            Finger (Z.A, 0);
            Finger (Z.B, 1);
            G.Kind := Grip; G.Arm := A;
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
         end;
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
                  Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
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
               Push (It, "a thing, now in cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & " (" & Codec.Img (It.Count) & " px, standing " &
                     Codec.Fmt (It.Height, 3) & " out of the surface)" & Rel (It.Cu, It.Cv), Draw.Green, 2);
            elsif Sl.Seen then
               It.Kind := Thing_Remembered; It.Located := True;
               It.Cu := Sl.Shadow.Cu; It.Cv := Sl.Shadow.Cv; It.Depth := Sl.Shadow.Depth; It.Height := Sl.Shadow.Height; It.Count := Sl.Shadow.Count;
               It.X0 := Sl.Shadow.X0; It.Y0 := Sl.Shadow.Y0; It.X1 := Sl.Shadow.X1; It.Y1 := Sl.Shadow.Y1;
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
      Kind : Track_Kind := Zone_Pt;
      Slot : Integer := -1;
      Item_No : Natural := 0;
      Cu, Cv, Z : Long_Float := 0.0;
      Tu, Tv, Tz : Long_Float := 0.0;
      Wz : Long_Float := 0.0;
      Lateral_First : Boolean := False;
      Lat_Tol : Long_Float := 0.0;
      Desc : Unbounded_String;
      Box_W, Box_H : Long_Float := 0.0;
      Count : Natural := 0;
      Height : Long_Float := 0.0;
      Err0 : Long_Float := 0.0;
   end record;
   package Point_Vectors is new Ada.Containers.Vectors (Natural, Point);

   function Err_Of (P : Point) return Long_Float is
      Dz : constant Long_Float := (if P.Wz > 0.0 and then P.Z > 0.0 then (P.Tz - P.Z) / P.Z else 0.0);
   begin
      return Sqrt ((P.Tu - P.Cu) ** 2 + (P.Tv - P.Cv) ** 2 + Dz * Dz);
   end Err_Of;

   function Find_Effect (C : Context; Arm, Cam : Natural; Kind : Track_Kind) return Integer is
   begin
      for I in 0 .. Natural (C.Tables.Length) - 1 loop
         if C.Tables (I).Arm = Arm and then C.Tables (I).Cam = Cam and then C.Tables (I).Kind = Kind then
            return I;
         end if;
      end loop;
      return -1;
   end Find_Effect;

   procedure Store_Effect (C : in out Context; Arm, Cam : Natural; Kind : Track_Kind; E : Table.Effect; Trust : Table.Mask) is
      I : constant Integer := Find_Effect (C, Arm, Cam, Kind);
      Se : constant Stored_Effect := (Arm, Cam, Kind, E, Trust);
   begin
      if I >= 0 then
         C.Tables.Replace_Element (Natural (I), Se);
      else
         C.Tables.Append (Se);
      end if;
   end Store_Effect;

   --  重新定位一个点:握区靠光流平流(世界相机)/固定(自己的手上相机);世界块重切后就近对上
   procedure Retrack (C : in out Context; F : Plug.Frame; Cam : Natural; Before : Buf; P : in out Point; Pred_U, Pred_V : Long_Float; Moved_Arm : Boolean; Pred_Z : Long_Float := -1.0) is
      Cw : constant Natural := F.Cams (Cam).W;
      Ch : constant Natural := F.Cams (Cam).H;
   begin
      case P.Kind is
         when Zone_Pt =>
            if Cam_Arm (C, Cam) = Integer (P.Arm) then
               return;    --  自己的手上相机:握区是固定像素
            end if;
            declare
               --  半分辨率算光流(3 层 30 轮:次数),在区心一小片取平均位移
               Hw : constant Natural := Cw / 2;
               Hh : constant Natural := Ch / 2;
               A, B : Buf;
               Fl : Flow.Field;
               Du, Dv : Long_Float;
               Idx : constant Natural := Track_Idx (C, P.Arm, Cam);
               Tr : Zone_Track := C.Zones (Idx);
               Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
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
                  Tr.Stale := Tr.Stale + 1;      --  手臂动了,区心那儿画面却没流:跟丢的迹象
                  P.Cu := Pred_U; P.Cv := Pred_V;
               else
                  Tr.Stale := 0;
                  P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cu + Du));
                  P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, P.Cv + Dv));
               end if;
               if F.Cams (Cam).Has_Depth then
                  declare
                     --  深度读在两瓣上(区心是两指之间的空,读到的是桌面;EF 实测区心读出 0.588 = 桌面,手指其实在 0.414),
                     --  瓣的位置 = 区心 + 量握区时瓣相对区心的偏移(刚性);窗口 = 张幅的四分之一(比例,无量纲)
                     Win : constant Long_Float := Long_Float'Max (0.005, Z.Span * 0.25);
                     Zs : Floats;
                     Old_Z : constant Long_Float := P.Z;
                  begin
                     if Z.A.Valid then
                        declare
                           D : constant Long_Float := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, P.Cu + (Z.A.Cu - Z.Cu), P.Cv + (Z.A.Cv - Z.Cv), Win);
                        begin
                           if not Picture.Is_Nan (D) then
                              Zs.Append (D);
                           end if;
                        end;
                     end if;
                     if Z.B.Valid then
                        declare
                           D : constant Long_Float := Picture.Near_Depth (F.Cams (Cam).Depth, Cw, Ch, P.Cu + (Z.B.Cu - Z.Cu), P.Cv + (Z.B.Cv - Z.Cv), Win);
                        begin
                           if not Picture.Is_Nan (D) then
                              Zs.Append (D);
                           end if;
                        end;
                     end if;
                     if not Zs.Is_Empty then
                        declare
                           Zd : constant Long_Float := Picture.Quantile (Zs, 0.5);
                        begin
                           --  一步之内深度跳了超过"预测的变化 + 距离的一成"(比例,无量纲)⇒ 读到的不是我的手指,留预测
                           if Old_Z <= 0.0 or else abs (Zd - Pred_Z) <= abs (Pred_Z - Old_Z) + 0.1 * Old_Z then
                              P.Z := Zd;
                           else
                              P.Z := Pred_Z;
                              Tr.Stale := Tr.Stale + 1;
                           end if;
                        end;
                     end if;
                  end;
               end if;
               Tr.Cu := P.Cu; Tr.Cv := P.Cv; Tr.Z := P.Z; Tr.Valid := True;
               C.Zones.Replace_Element (Idx, Tr);
            end;
         when Thing_Pt =>
            declare
               Regs : constant Picture.Regions := Cut_Things (C, F, Cam);
               Best : Integer := -1;
               Bd : Long_Float := 1.0e9;
               Tol : constant Long_Float := Long_Float'Max (P.Box_W, P.Box_H) * 0.75 + Track_Win;
            begin
               for I in 0 .. Natural (Regs.Length) - 1 loop
                  declare
                     R : constant Picture.Region := Regs (I);
                     D : constant Long_Float := Sqrt ((R.Cu - Pred_U) ** 2 + (R.Cv - Pred_V) ** 2);
                  begin
                     if R.Count * 3 >= P.Count and then R.Count <= P.Count * 3 and then D <= Tol and then D < Bd then
                        Bd := D; Best := I;
                     end if;
                  end;
               end loop;
               if Best >= 0 then
                  declare
                     R : constant Picture.Region := Regs (Best);
                  begin
                     P.Cu := R.Cu; P.Cv := R.Cv; P.Z := R.Depth; P.Height := R.Height; P.Count := R.Count;
                     P.Box_W := Long_Float (R.X1 - R.X0) / Long_Float (Cw);
                     P.Box_H := Long_Float (R.Y1 - R.Y0) / Long_Float (Ch);
                  end;
               else
                  P.Cu := Pred_U; P.Cv := Pred_V;
               end if;
            end;
      end case;
   end Retrack;

   --  发一步并等稳;返回实到(通道)
   procedure Step_Arm (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; A : Table.Vec; Jaw : Floats;
                       Delivered : out Table.Vec; Ok : out Boolean) is
      P0 : constant Plug.Arm_Pose := F.EE (Arm);
      Frames : Natural;
   begin
      Selfmap.Go (L, C.Map, Arm, Chan.Compose (P0, A), Jaw, F, Delivered, Frames, Ok);
   end Step_Arm;

   --  没有表的点:每个通道推一下量一列。幅度从开机看得见的那一档起,翻倍到这个点在画面里跑过 4 个跟踪地板为止
   --  (倍数,无量纲;EF 实测:最小可见幅度量出来的列全是噪声,解算据此拧手腕 0.1 rad 把握区甩到了臂上);
   --  翻到上限还跑不过地板的通道,这一段不用它(Trust 为假)。推完推回起点。
   procedure Probe_Effect (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; P : in out Point; E : in out Table.Effect;
                           Trust : out Table.Mask; Ok : out Boolean) is
      Arm : constant Natural := P.Arm;
      P0 : constant Plug.Arm_Pose := F.EE (Arm);
      Jaw : Floats;
      Cw : constant Natural := F.Cams (Cam).W;
      Floor_Px : constant Long_Float := 4.0 / Long_Float (Cw);   --  跟踪地板:4 个像素(倍数,无量纲)
   begin
      Trust := [others => False];
      Jaw.Append (Selfmap.Jaw_Of (F, Arm));
      Table.Reset (E, Chan.Per_Arm, 1.0);
      --  先验不确定度按每个通道的探针幅度定:P0 = 100/幅²(倍数,无量纲)⇒ 头几步就能把探出来的列修正过来
      for K in 0 .. Chan.Per_Arm - 1 loop
         declare
            Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Arm * Chan.Per_Arm + K));
         begin
            Table.Set_Prior (E, K, 100.0 / (Am * Am));
         end;
      end loop;
      Ok := True;
      Put_Line ("[身]   这个点还没有响应表 ⇒ 六个通道各推一下量一列(幅度 = 开机看得见的那一档)");
      for K in 0 .. Chan.Per_Arm - 1 loop
         declare
            Ch : constant Natural := Arm * Chan.Per_Arm + K;
            Amp : Long_Float := C.Map.Amp (Ch);
            Cap_Amp : constant Long_Float := C.Map.Amp (Ch) * Cap_Mult;
         begin
            if not C.Map.Seen (Ch) or else Amp <= 0.0 then
               Put_Line ("[身]     通道" & Natural'Image (Ch) & " 开机时没看见它动,这一列留零");
            else
               loop
                  declare
                     A : Table.Vec := Table.Zero_Vec;
                     Before : constant Buf := F.Cams (Cam).Gray;
                     Was : constant Point := P;
                     Deliv, Back : Table.Vec;
                     Ok2 : Boolean;
                     Frames : Natural;
                     Ran : Long_Float := 0.0;
                  begin
                     A (K) := Amp;
                     Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok2);
                     if not Ok2 then
                        Ok := False;
                        return;
                     end if;
                     Retrack (C, F, Cam, Before, P, Was.Cu, Was.Cv, True);
                     Ran := Sqrt ((P.Cu - Was.Cu) ** 2 + (P.Cv - Was.Cv) ** 2);
                     if abs Deliv (K) > C.Map.EE_Noise and then Ran >= Floor_Px then
                        declare
                           Col : Table.Vec3;
                        begin
                           Col (0) := (P.Cu - Was.Cu) / Deliv (K);
                           Col (1) := (P.Cv - Was.Cv) / Deliv (K);
                           Col (2) := (if P.Z > 0.0 and then Was.Z > 0.0 then (P.Z - Was.Z) / Deliv (K) else 0.0);
                           Table.Set_Col (E, K, Col);
                           Trust (K) := True;
                           Put_Line ("[身]     通道" & Natural'Image (Ch) & ":命令 " & Codec.Fmt (Amp, 4) & " 实到 " & Codec.Fmt (Deliv (K), 4) & " ⇒ 点跑了 (" &
                                     Codec.Fmt (P.Cu - Was.Cu, 4) & "," & Codec.Fmt (P.Cv - Was.Cv, 4) & ", 深 " & Codec.Fmt (P.Z - Was.Z, 4) & ")");
                        end;
                     end if;
                     declare
                        Before2 : constant Buf := F.Cams (Cam).Gray;
                        Pu : constant Long_Float := Was.Cu;
                        Pv : constant Long_Float := Was.Cv;
                     begin
                        Selfmap.Go (L, C.Map, Arm, P0, Jaw, F, Back, Frames, Ok2);
                        if not Ok2 then
                           Ok := False;
                           return;
                        end if;
                        Retrack (C, F, Cam, Before2, P, Pu, Pv, True);
                        P.Cu := Was.Cu; P.Cv := Was.Cv; P.Z := Was.Z;   --  推回起点了:点回到原处(比光流往返的累积误差可信)
                     end;
                     exit when Trust (K);
                     if Amp * 2.0 > Cap_Amp then
                        Put_Line ("[身]     通道" & Natural'Image (Ch) & ":到 " & Codec.Fmt (Amp, 4) & " 点还只跑了 " & Codec.Fmt (Ran, 4) & " 画幅(地板 " & Codec.Fmt (Floor_Px, 4) & ")⇒ 这一段不用它");
                        exit;
                     end if;
                     Amp := Amp * 2.0;
                  end;
               end loop;
            end if;
         end;
      end loop;
   end Probe_Effect;

   function Amount_Factor (A : Unbounded_String) return Long_Float is
     (if A = "small" then 0.25 elsif A = "large" then 1.0 else 0.5);   --  探针上限的几分之几(比例,无量纲)

   --  ── 一段:把这些点推到各自的目标,直到事件 ──
   procedure Run_Segment (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector;
                          Until_Kind : Monitor.Until_Kind; Step_Limit : Natural; Amount : Long_Float; Avoid : Item_Vectors.Vector;
                          Event : out Unbounded_String; Steps_Taken : out Natural; Blocked_Out : out Boolean) is
      Arm : constant Natural := Pts (0).Arm;
      Effs : array (0 .. Natural (Pts.Length) - 1) of Table.Effect;
      Trusts : array (0 .. Natural (Pts.Length) - 1) of Table.Mask := [others => [others => True]];
      W : Monitor.Watch;
      Fl : Monitor.Floors;
      Cw : constant Natural := F.Cams (Cam).W;
      Ring : Backup.Ring;
      Jaw : Floats;
      Last_Err : Long_Float := 0.0;
      Ok : Boolean;
   begin
      Event := S ("hit the safety cap on steps");
      Steps_Taken := 0;
      Blocked_Out := False;
      Jaw.Append (Selfmap.Jaw_Of (F, Arm));
      Fl.Track := Long_Float'Max (1.0 / Long_Float (Cw), 0.0);
      Fl.Picture := Long_Float (Integer'(if Cam < Natural (C.Map.Pic_Floor.Length) then C.Map.Pic_Floor (Cam) else 0));
      Fl.Reading := C.Map.Jaw_Noise;
      Fl.Delivery := C.Map.EE_Noise;
      Backup.Clear (Ring);
      for I in 0 .. Natural (Pts.Length) - 1 loop
         declare
            Idx : constant Integer := Find_Effect (C, Arm, Cam, Pts (I).Kind);
            P : Point := Pts (I);
         begin
            if Idx >= 0 then
               Effs (I) := C.Tables (Natural (Idx)).E;
               Trusts (I) := C.Tables (Natural (Idx)).Trust;
            else
               Probe_Effect (L, C, F, Cam, P, Effs (I), Trusts (I), Ok);
               Pts.Replace_Element (I, P);
               if not Ok then
                  Event := S ("the body stopped answering while I measured my response table");
                  return;
               end if;
               Store_Effect (C, Arm, Cam, P.Kind, Effs (I), Trusts (I));
            end if;
         end;
      end loop;
      for I in 0 .. Natural (Pts.Length) - 1 loop
         Last_Err := Last_Err + Err_Of (Pts (I));
      end loop;
      for Step in 1 .. Natural'Min (Step_Cap, (if Step_Limit > 0 then Step_Limit else Step_Cap)) loop
         declare
            Terms : Table.Term_Vectors.Vector;
            Cap : Table.Vec := Table.Zero_Vec;
            Active : Table.Mask := [others => False];
            A : Table.Vec;
            Solved : Boolean;
            Scale : Long_Float := 1.0;
         begin
            for I in 0 .. Natural (Pts.Length) - 1 loop
               declare
                  P : constant Point := Pts (I);
                  T : Table.Term;
                  Lat : constant Long_Float := Sqrt ((P.Tu - P.Cu) ** 2 + (P.Tv - P.Cv) ** 2);
               begin
                  T.E := Effs (I);
                  T.Err (0) := P.Tu - P.Cu;
                  T.Err (1) := P.Tv - P.Cv;
                  T.W (0) := 1.0; T.W (1) := 1.0;
                  if P.Wz > 0.0 and then P.Z > 0.0 and then not Picture.Is_Nan (P.Tz) and then not (P.Lateral_First and then Lat > P.Lat_Tol) then
                     T.Err (2) := (P.Tz - P.Z) / P.Z;
                     T.W (2) := 1.0;
                     --  深度那一行的表也按 1/z 缩,和误差同一尺度
                     for K in 0 .. Chan.Per_Arm - 1 loop
                        T.E.B (K, 2) := T.E.B (K, 2) / P.Z;
                     end loop;
                  else
                     T.Err (2) := 0.0; T.W (2) := 0.0;
                  end if;
                  Terms.Append (T);
               end;
            end loop;
            declare
               Damp : Table.Vec := Table.Zero_Vec;
            begin
               for K in 0 .. Chan.Per_Arm - 1 loop
                  declare
                     Ch : constant Natural := Arm * Chan.Per_Arm + K;
                     Am : constant Long_Float := Long_Float'Max (1.0e-6, C.Map.Amp (Ch));
                     All_Trust : Boolean := True;
                  begin
                     for I in 0 .. Natural (Pts.Length) - 1 loop
                        if not Trusts (I) (K) then
                           All_Trust := False;
                        end if;
                     end loop;
                     if C.Map.Seen (Ch) and then All_Trust then
                        Active (K) := True;
                        Cap (K) := Am * Cap_Mult * Amount;
                     end if;
                     --  阻尼 = 1e-3 / 幅²:每个通道都以"几个探针幅度"计价(无量纲),转动不再比平移便宜
                     Damp (K) := 1.0e-3 / (Am * Am);
                  end;
               end loop;
               Table.Solve (Terms, Chan.Per_Arm, Cap, Active, Damp, A, Solved);
            end;
            if not Solved then
               Event := S ("could not solve which channels to push");
               return;
            end if;
            --  一步里任何点在画面里最多跑一个跟踪窗;要压进"别碰"框的就缩
            for I in 0 .. Natural (Pts.Length) - 1 loop
               declare
                  Pr : constant Table.Vec3 := Table.Predict (Effs (I), A);
                  D : constant Long_Float := Sqrt (Pr (0) ** 2 + Pr (1) ** 2);
               begin
                  if D > Track_Win then
                     Scale := Long_Float'Min (Scale, Track_Win / D);
                  end if;
               end;
            end loop;
            for Round in 1 .. 4 loop
               declare
                  Hit : Boolean := False;
               begin
                  for I in 0 .. Natural (Pts.Length) - 1 loop
                     declare
                        Pr : constant Table.Vec3 := Table.Predict (Effs (I), A);
                        Nu : constant Long_Float := Pts (I).Cu + Pr (0) * Scale;
                        Nv : constant Long_Float := Pts (I).Cv + Pr (1) * Scale;
                     begin
                        for Av of Avoid loop
                           if Av.Located and then Nu * Long_Float (Cw) >= Long_Float (Av.X0) and then Nu * Long_Float (Cw) <= Long_Float (Av.X1)
                             and then Nv * Long_Float (F.Cams (Cam).H) >= Long_Float (Av.Y0) and then Nv * Long_Float (F.Cams (Cam).H) <= Long_Float (Av.Y1)
                           then
                              Hit := True;
                           end if;
                        end loop;
                     end;
                  end loop;
                  exit when not Hit;
                  if Round = 4 then
                     Event := S ("stopped: every step would push me onto a thing I must not touch");
                     return;
                  end if;
                  Scale := Scale * 0.5;
               end;
            end loop;
            for K in 0 .. Chan.Per_Arm - 1 loop
               A (K) := A (K) * Scale;
            end loop;
            if Table.Norm (A, Chan.Per_Arm) <= C.Map.EE_Noise then
               Event := S ("amount: already there (what is left to push is within my own noise)");
               return;
            end if;
            declare
               Before : constant Buf := F.Cams (Cam).Gray;
               Deliv : Table.Vec;
               Pic_Delta : Long_Float;
               Err_Now : Long_Float := 0.0;
               Any_Blocked : Boolean := False;
               Moved : constant Boolean := True;
            begin
               Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok);
               if not Ok then
                  Event := S ("the body refused the command");
                  return;
               end if;
               Steps_Taken := Steps_Taken + 1;
               declare
                  Sv : Backup.Step_Vec := [others => 0.0];
               begin
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     Sv (K) := Deliv (K);
                  end loop;
                  Backup.Remember (Ring, Sv);
               end;
               Pic_Delta := Long_Float (Picture.Max_Diff (Before, F.Cams (Cam).Gray));
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  declare
                     P : Point := Pts (I);
                     Pr : constant Table.Vec3 := Table.Predict (Effs (I), Deliv);
                     Was_U : constant Long_Float := P.Cu;
                     Was_V : constant Long_Float := P.Cv;
                     Was_Z : constant Long_Float := P.Z;
                     Dy : Table.Vec3;
                     E : Table.Effect := Effs (I);
                  begin
                     Retrack (C, F, Cam, Before, P, Was_U + Pr (0), Was_V + Pr (1), Moved, (if Was_Z > 0.0 then Was_Z + Pr (2) else -1.0));
                     Dy (0) := P.Cu - Was_U; Dy (1) := P.Cv - Was_V;
                     Dy (2) := (if P.Z > 0.0 and then Was_Z > 0.0 then P.Z - Was_Z else 0.0);
                     Table.Update (E, Deliv, Dy, Fl.Track * 2.0, C.Map.EE_Noise);
                     if Table.Blocked (E) then
                        Any_Blocked := True;
                     end if;
                     Effs (I) := E;
                     Pts.Replace_Element (I, P);
                     Err_Now := Err_Now + Err_Of (P);
                  end;
               end loop;
               Monitor.Step (W, Monitor.Floor (Long_Float'Max (0.0, Pic_Delta)), Monitor.Bounded (Last_Err), Monitor.Bounded (Err_Now),
                             Monitor.Floor (Long_Float'Max (0.0, Table.Norm (Deliv, Chan.Per_Arm))), Fl);
               Put_Line ("[身]     步" & Natural'Image (Steps_Taken) & ":误 " & Codec.Fmt (Last_Err, 3) & " → " & Codec.Fmt (Err_Now, 3) &
                         " · 命令 [" & Codec.Fmt (A (0), 3) & " " & Codec.Fmt (A (1), 3) & " " & Codec.Fmt (A (2), 3) & " " & Codec.Fmt (A (3), 3) & " " & Codec.Fmt (A (4), 3) & " " & Codec.Fmt (A (5), 3) &
                         "] · 实到 [" & Codec.Fmt (Deliv (0), 4) & " " & Codec.Fmt (Deliv (1), 4) & " " & Codec.Fmt (Deliv (2), 4) & " " & Codec.Fmt (Deliv (3), 3) & " " & Codec.Fmt (Deliv (4), 3) & " " & Codec.Fmt (Deliv (5), 3) &
                         "] · 点 (" & Codec.Fmt (Pts (0).Cu, 3) & "," & Codec.Fmt (Pts (0).Cv, 3) & ") 深 " & Codec.Fmt (Pts (0).Z, 3) & (if Any_Blocked then " · 零表更准(顶住?)" else ""));
               Last_Err := Err_Now;
               if Any_Blocked or else Monitor.Refusing (W) then
                  Blocked_Out := True;
               end if;
               if Monitor.Fired (Until_Kind, W, Step_Limit, Any_Blocked, Monitor.Bounded (Selfmap.Jaw_Of (F, Arm)),
                                 Monitor.Bounded (if Arm < Natural (C.Hands.Length) then C.Hands (Arm).Empty_Close else 0.0), Monitor.Floor (C.Map.Jaw_Noise))
               then
                  Event := (case Until_Kind is
                              when Monitor.U_Steps => S ("steps: I took the steps you asked for"),
                              when Monitor.U_Contact => S ("contact: my pushes stopped producing motion (something is touched or the body will not go there)"),
                              when Monitor.U_Resist => S ("resist: it will not move any further"),
                              when Monitor.U_Slip => S ("slip: what I was holding has left my fingers"),
                              when Monitor.U_Settle => S ("settle: the picture stopped changing"));
                  return;
               end if;
               if Err_Now <= Fl.Track * 2.0 then
                  Event := S ("amount: arrived (remaining error within tracking noise)");
                  return;
               end if;
               if Monitor.Stalled (W) then
                  Event := S ("amount: stopped getting closer (remaining " & Codec.Fmt (Err_Now, 3) & " of a frame) - either something holds me or this arm cannot reach farther from here");
                  return;
               end if;
            end;
         end;
      end loop;
      Event := S ("steps: hit the step cap (" & Codec.Img (Steps_Taken) & ")");
   end Run_Segment;

   --  合/张到读数不再变
   procedure Move_Jaw (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; Target : Long_Float; Steps : out Natural; Reading : out Long_Float) is
      Jaw : Floats;
      Prev : Long_Float := Selfmap.Jaw_Of (F, Arm);
      Prev_Cams : Plug.Cam_Vectors.Vector := F.Cams;
      Still : Natural := 0;
      Cm : Plug.Cmd;
   begin
      Jaw.Append (Target);
      Steps := 0;
      Reading := Prev;
      --  读数是命令的回声,"停住"只认画面:每台相机连着两拍不变
      for I in 1 .. 40 loop
         Cm.Kind := Plug.Ee; Cm.Arm := Arm; Cm.Pose := F.EE (Arm); Cm.Jaw := Jaw;
         exit when not Plug.Act (L, Cm) or else not Plug.Sense (L, F);
         Steps := I;
         Reading := Selfmap.Jaw_Of (F, Arm);
         if abs (Reading - Prev) <= C.Map.Jaw_Noise and then Selfmap.Pictures_Still (C.Map, Prev_Cams, F.Cams) then
            Still := Still + 1;
         else
            Still := 0;
         end if;
         Prev := Reading;
         Prev_Cams := F.Cams;
         exit when Still >= 2 and then I >= 3;
      end loop;
   end Move_Jaw;

   --  握住了没:抬一小截,看东西跟不跟我走。手上相机里 = 它的块还在握区框里;世界相机里 = 它原来那块地方空了。读数不算数(回声)。
   procedure Held_Test (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Cam : Natural; Origin : Picture.Region;
                        Obj_Count : Natural; Held : out Boolean; Note : out Unbounded_String) is
      A : Table.Vec := Table.Zero_Vec;
      Deliv : Table.Vec;
      Ok : Boolean;
      Hc : constant Integer := (if Arm < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (Arm) else -1);
      Jaw : Floats;
      Seen_In_Hand : Boolean := False;
      Gone_From_Table : Boolean := False;
      Could_Judge : Boolean := False;
   begin
      Jaw.Append (Selfmap.Jaw_Of (F, Arm));
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
      Held := Seen_In_Hand or else (Gone_From_Table and then Hc < 0);
      if Seen_In_Hand then
         Note := S ("after a small lift the thing is still inside my grip box in my hand camera ⇒ held");
      elsif Gone_From_Table and then Hc >= 0 then
         Note := S ("after a small lift its place on the table is empty but my hand camera does not show it between my fingers ⇒ not counted as held");
      elsif Gone_From_Table then
         Note := S ("after a small lift its place on the table is empty ⇒ held");
      elsif Could_Judge then
         Note := S ("after a small lift the thing did not come with me ⇒ not held");
      else
         Note := S ("I could not judge whether it is held (no camera could see it)");
      end if;
   end Held_Test;

   function Mode_Line (C : Context; Until_Text : String) return String is
     ("MODE: " & (if C.Wld.Holding then "holding something with arm " & Codec.Img (Natural (C.Wld.Held_Arm) + 1) else "hands empty") &
      "; without new words from you I hold still and keep my grip as it is; this segment ended on: " & Until_Text & ".");

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
      --  切块 → 世界槽
      World.Observe (C.Wld, Cam, Cut_Things (C, F, Cam), Cw, Ch);
      Draw.Grid (RGB, Cw, Ch, C.Cols, C.Rows, C.Cells_U, C.Cells_V);
      Build_Listing (C, F, Cam, RGB, Listing);
      Put_Line ("[身] ── 第" & Natural'Image (C.Round_N) & " 轮(第" & Natural'Image (Cam) & " 台相机)──");
      Put (To_String (Listing));
      if C.Dump_Dir /= "" then
         Codec.Write_BMP (To_String (C.Dump_Dir) & "/grid_" & Codec.Pad6 (C.Round_N) & ".bmp", RGB, Cw, Ch);
      end if;
      declare
         Recent : constant String := Memory.Text (C.Mem) & To_String (C.Recent);
      begin
         if not Brain.Ask (To_String (C.Eye_Host), C.Eye_Port, To_String (C.Task_Text), To_String (Listing), Recent,
                           C.Cols, C.Rows, Natural (C.Items.Length), C.Map.N_Cams, C.Map.Arms, RGB, Cw, Ch, Say, Err)
         then
            Put_Line ("[身] 🧠 问不通(" & To_String (Err) & ")⇒ 这一拍不动,下一拍重问");
            return;
         end if;
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
         Amount : Long_Float := 0.5;
         Event : Unbounded_String;
         Steps_Taken : Natural := 0;
         Blocked : Boolean;
         Desc : Unbounded_String;
         Grip_Arm : constant Integer := (if Say.Grip_Arm >= 1 and then Say.Grip_Arm <= C.Map.Arms then Integer (Say.Grip_Arm) - 1 else -1);
         Did_Grip : Unbounded_String;
      begin
         for N of Say.Avoid loop
            if N >= 1 and then N <= Natural (C.Items.Length) then
               Avoid.Append (C.Items (N - 1));
            end if;
         end loop;
         --  移动条目 → 点
         for G of Say.Moves loop
            if G.Item >= 1 and then G.Item <= Natural (C.Items.Length) and then not G.Stay then
               declare
                  It : constant Item := C.Items (G.Item - 1);
                  P : Point;
                  Own : constant Boolean := It.Kind in Finger | Grip | Thing_Held;
                  Cam_A : constant Integer := Cam_Arm (C, Cam);
                  Ok_Pt : Boolean := True;
               begin
                  Amount := Amount_Factor (G.Amount);
                  P.Item_No := G.Item;
                  if not It.Located then
                     Report := S ("goal: item " & Codec.Img (G.Item) & " is not locatable in this picture right now. ");
                     Ok_Pt := False;
                  elsif Own then
                     P.Arm := It.Arm; P.Kind := Zone_Pt;
                     declare
                        Tr : constant Zone_Track := C.Zones (Track_Idx (C, P.Arm, Cam));
                     begin
                        P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z;
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
                                       Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
                                    begin
                                       P.Tu := Z.Cu; P.Tv := Z.Cv; P.Tz := Z.Depth; P.Wz := (if Picture.Is_Nan (Z.Depth) then 0.0 else 1.0);
                                       P.Lateral_First := True; P.Lat_Tol := Long_Float'Max (Z.Span * 0.25, Track_Win * 0.5);
                                    end;
                                 elsif P.Kind = Zone_Pt and then O.Kind in Thing | Thing_Remembered then
                                    P.Tz := O.Depth + O.Height * 0.5; P.Wz := (if O.Depth > 0.0 then 1.0 else 0.0);   --  指尖到它的半腰(顶面深 + 鼓起的一半,都是量的)
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
                              end if;
                           end if;
                        end;
                     else
                        Ok_Pt := False;
                     end if;
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
                     P.Kind := Thing_Pt; P.Slot := O.Slot; P.Cu := O.Cu; P.Cv := O.Cv; P.Z := O.Depth; P.Height := O.Height; P.Count := O.Count;
                     P.Box_W := Long_Float (O.X1 - O.X0) / Long_Float (Cw); P.Box_H := Long_Float (O.Y1 - O.Y0) / Long_Float (Ch);
                     P.Tu := Z.Cu; P.Tv := Z.Cv; P.Tz := Z.Depth; P.Wz := (if Picture.Is_Nan (Z.Depth) then 0.0 else 1.0);
                     P.Lateral_First := True; P.Lat_Tol := Long_Float'Max (Z.Span * 0.25, Track_Win * 0.5);
                     P.Desc := S ("item " & Codec.Img (Say.Grip_On) & " into grip " & Codec.Img (A + 1) & " (this hand camera)");
                  else
                     declare
                        Tr : constant Zone_Track := C.Zones (Track_Idx (C, A, Cam));
                     begin
                        P.Kind := Zone_Pt; P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z;
                        P.Tu := O.Cu; P.Tv := O.Cv; P.Tz := O.Depth + O.Height * 0.5; P.Wz := (if O.Depth > 0.0 and then Tr.Z > 0.0 then 1.0 else 0.0);
                        P.Desc := S ("grip " & Codec.Img (A + 1) & " onto item " & Codec.Img (Say.Grip_On) & " (fingertips to its middle)");
                     end;
                  end if;
                  Pts.Append (P);
               end if;
            end;
         end if;
         if not Pts.Is_Empty then
            for P of Pts loop
               Append (Desc, (if Desc = "" then "" else " and ") & To_String (P.Desc));
            end loop;
            Put_Line ("[身] ⚙ 一起解" & Natural'Image (Natural (Pts.Length)) & " 条:" & To_String (Desc));
            Run_Segment (L, C, F, Cam, Pts, Until_K, Step_Limit, Amount, Avoid, Event, Steps_Taken, Blocked);
            Report := Report & "you asked " & Desc & ": " & Event & ". I took " & Codec.Img (Steps_Taken) & " steps; ";
            for P of Pts loop
               Report := Report & "item " & Codec.Img (P.Item_No) & " now at (" & Codec.Fmt (P.Cu, 2) & "," & Codec.Fmt (P.Cv, 2) & ") depth " & Codec.Fmt (P.Z, 2) &
                         ", remaining error " & Codec.Fmt (Err_Of (P), 3) & " of a frame; ";
            end loop;
         elsif Say.Moves.Is_Empty and then Say.Grip = "none" then
            Report := S ((if Say.See = "not_here" then "you said the thing is not in that picture; the body did not move. "
                          elsif Say.See = "unclear" then "you said you could not tell; the body did not move. "
                          else "you gave no move and no grip; the body did not move. "));
         end if;
         --  ── 抓握 ──
         if Say.Grip = "close" and then Grip_Arm >= 0 then
            declare
               A : constant Natural := Natural (Grip_Arm);
               Caged : Boolean := True;
               Cage_Note : Unbounded_String;
               Steps_J : Natural;
               Reading : Long_Float;
               Hz : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
            begin
               --  笼判据:点名的那块的像素在握区框里(它的形心落在区框内),深度和手指对得上
               if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then
                  declare
                     Pin : Point;
                     Found : Boolean := False;
                  begin
                     for P of Pts loop
                        if P.Item_No = Say.Grip_On then
                           Pin := P; Found := True;
                        end if;
                     end loop;
                     if Found and then Pin.Kind = Thing_Pt and then Hz.Valid then
                        declare
                           Inside : constant Boolean := Pin.Cu * Long_Float (Cw) >= Long_Float (Hz.X0) and then Pin.Cu * Long_Float (Cw) <= Long_Float (Hz.X1)
                                                        and then Pin.Cv * Long_Float (Ch) >= Long_Float (Hz.Y0) and then Pin.Cv * Long_Float (Ch) <= Long_Float (Hz.Y1);
                           Depth_Ok : constant Boolean := Picture.Is_Nan (Hz.Depth) or else Pin.Z <= 0.0 or else abs (Pin.Z - Hz.Depth) <= Long_Float'Max (Pin.Height, Long_Float'Max (Pin.Box_W, Pin.Box_H) * Pin.Z);
                        begin
                           Caged := Inside and then Depth_Ok;
                           Cage_Note := S ("cage check in this hand camera: the thing's centre is " & (if Inside then "inside" else "OUTSIDE") & " my grip box and its depth " &
                                           (if Depth_Ok then "matches" else "does not match") & " my fingertips (" & Codec.Fmt (Pin.Z, 3) & " vs " & Codec.Fmt (Hz.Depth, 3) & ")");
                        end;
                     elsif Found and then Pin.Kind = Zone_Pt then
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
                  Move_Jaw (L, C, F, A, 0.0, Steps_J, Reading);
                  declare
                     Empty : constant Long_Float := C.Hands (A).Empty_Close;
                     By_Reading : Boolean := Reading - Empty > C.Map.Jaw_Noise;
                     Note : Unbounded_String;
                     Origin : Picture.Region;
                     Obj_Count : Natural := 0;
                  begin
                     if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then
                        Origin := World.Get (C.Wld, Cam, Natural (C.Items (Say.Grip_On - 1).Slot)).Shadow;
                        Obj_Count := C.Items (Say.Grip_On - 1).Count;
                     end if;
                     Held_Test (L, C, F, A, Cam, Origin, Obj_Count, By_Reading, Note);
                     Did_Grip := S ("I closed grip " & Codec.Img (A + 1) & " until the picture stopped changing (" & Codec.Img (Steps_J) & " steps, reading " & Codec.Fmt (Reading, 3) &
                                    ", empty-close reading " & Codec.Fmt (Empty, 3) & "); " & To_String (Note));
                     if By_Reading then
                        C.Wld.Holding := True; C.Wld.Held_Arm := Integer (A); C.Wld.Held_Cam := Integer (Cam);
                        if Say.Grip_On >= 1 and then Say.Grip_On <= Natural (C.Items.Length) then
                           C.Wld.Held_Slot := C.Items (Say.Grip_On - 1).Slot;
                           C.Wld.Held_Origin := World.Get (C.Wld, Cam, Natural (C.Items (Say.Grip_On - 1).Slot)).Shadow;
                        else
                           C.Wld.Held_Slot := -1;
                        end if;
                        Memory.Set (C.Mem, "holding", "arm " & Codec.Img (A + 1) & " closed on item " & Codec.Img (Say.Grip_On) & " at reading " & Codec.Fmt (Reading, 3));
                     else
                        C.Wld.Holding := False; C.Wld.Held_Arm := -1;
                        Move_Jaw (L, C, F, A, C.Hands (A).Open_Reading, Steps_J, Reading);
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
               Move_Jaw (L, C, F, A, C.Hands (A).Open_Reading, Steps_J, Reading);
               C.Wld.Holding := False; C.Wld.Held_Arm := -1; C.Wld.Held_Slot := -1;
               Memory.Set (C.Mem, "holding", "");
               Did_Grip := S ("I opened grip " & Codec.Img (A + 1) & " (" & Codec.Img (Steps_J) & " steps, reading " & Codec.Fmt (Reading, 3) & ")");
            end;
         end if;
         if Did_Grip /= "" then
            Report := Report & To_String (Did_Grip) & ". ";
         end if;
         Report := Report & Mode_Line (C, To_String (Event));
      end;
      C.Recent := Report;
      Put_Line ("[身]   ⇒ " & To_String (Report));
   end Round;
end Act;
