with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Codec;
with Draw;
with Flow;
with Monitor;
with Backup;
package body Act is
   Sigma_Mult : constant Long_Float := 3.0;   --  鼓出来超过背景自己稳健 σ 的几倍才算一块(在真实深度图上验过:3 中,5 杀光);无量纲
   Track_Win : constant Long_Float := 0.10;   --  一步里任何被跟踪的点在画面里最多跑十分之一画幅(跟踪窗,比例,无量纲)
   Cap_Mult : constant Long_Float := 2.0;     --  一步命令上限 = 探针幅度(点在画面里跑过地板的那一档)的几倍(倍数,无量纲;EH:8 倍让阻尼当家,步子反而只剩探针的一倍)
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
               --  这一瓣挪了多少:身体图给了各瓣位置就按瓣,否则整区平移
               Lu : constant Long_Float := (if Own_Cam then 0.0 elsif Tr.Has_Lobes then (if Which = 0 then Tr.Au else Tr.Bu) - Lb.Cu else Du);
               Lv : constant Long_Float := (if Own_Cam then 0.0 elsif Tr.Has_Lobes then (if Which = 0 then Tr.Av else Tr.Bv) - Lb.Cv else Dv);
            begin
               It.Kind := Finger; It.Arm := A; It.Which := Which;
               if Z.Valid and then Lb.Valid and then Tr.Valid then
                  It.Located := True;
                  It.Cu := Lb.Cu + Lu; It.Cv := Lb.Cv + Lv;
                  It.X0 := Natural (Long_Float'Max (0.0, Long_Float (Lb.X0) + Lu * Long_Float (Cw)));
                  It.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), Long_Float (Lb.X1) + Lu * Long_Float (Cw))));
                  It.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Lb.Y0) + Lv * Long_Float (Ch)));
                  It.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), Long_Float (Lb.Y1) + Lv * Long_Float (Ch))));
                  It.Depth := Tr.Z; It.Count := Lb.Count;
                  Push (It, "a finger of arm " & Codec.Img (A + 1) & " (it moves when that arm's grip channel moves), now in cell " &
                        Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & Rel (It.Cu, It.Cv) &
                        (if Own_Cam or else Tr.Known then "" else " (placed from my joints; I have not yet looked at my hand here)"), Draw.Orange, 2);
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
            --  全身零件:每个通道带的那一块(从那个关节往外的全部),位置按此刻位姿从身体图来
            if not Own_Cam then
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
      Kind : Track_Kind := Piece_Pt;
      Slot : Integer := -1;
      Item_No : Natural := 0;
      Chan_K : Natural := 0;     --  带这块的通道(Chan.Per_Arm = 握合通道 ⇒ 这块是手指)
      Blob : Integer := -1;      --  这块的第几团(手指两团时一团一个点:两指各自到位,歪了就有一团不到位 —— 倾斜自然被罚)
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
      Lost : Boolean := False;   --  这一步没在画面里认出它,位置是按表猜的
      Has_Meas : Boolean := False;              --  眼睛(光流)另外量到的位置,只用来修表
      Meas_U, Meas_V, Meas_Z : Long_Float := 0.0;
   end record;
   package Point_Vectors is new Ada.Containers.Vectors (Natural, Point);
   type Effect_Array is array (Natural range <>) of Table.Effect;
   procedure Refind_Pieces (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector);

   function Err_Of (P : Point) return Long_Float is
      Dz : constant Long_Float := (if P.Wz > 0.0 and then P.Z > 0.0 then (P.Tz - P.Z) / P.Z else 0.0);
   begin
      return Sqrt ((P.Tu - P.Cu) ** 2 + (P.Tv - P.Cv) ** 2 + Dz * Dz);
   end Err_Of;

   function Find_Effect (C : Context; Arm, Cam : Natural; Kind : Track_Kind; Chan_K : Natural; Blob : Integer := -1) return Integer is
   begin
      for I in 0 .. Natural (C.Tables.Length) - 1 loop
         if C.Tables (I).Arm = Arm and then C.Tables (I).Cam = Cam and then C.Tables (I).Kind = Kind and then C.Tables (I).Chan_K = Chan_K and then C.Tables (I).Blob = Blob then
            return I;
         end if;
      end loop;
      return -1;
   end Find_Effect;

   procedure Store_Effect (C : in out Context; Arm, Cam : Natural; Kind : Track_Kind; Chan_K : Natural; Blob : Integer; E : Table.Effect; Trust : Table.Mask; Reach : Long_Float := 1.0) is
      I : constant Integer := Find_Effect (C, Arm, Cam, Kind, Chan_K, Blob);
      Se : constant Stored_Effect := (Arm, Cam, Kind, Chan_K, Blob, E, Trust, Reach);
   begin
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
                        Reach : Long_Float := 1.0;
                        Gp : constant Schema.Part_Pos := Sm.Parts (Chan.Per_Arm);   --  握合通道带的那块 = 手指
                        function Shift (Blob : Integer) return Table.Vec3 is
                           Idx : Integer := Find_Effect (C, A, Cm, Piece_Pt, Chan.Per_Arm, Blob);
                        begin
                           if Idx < 0 then
                              Idx := Find_Effect (C, A, Cm, Piece_Pt, Chan.Per_Arm, -1);
                           end if;
                           if Idx >= 0 then
                              Reach := Long_Float'Max (Reach, C.Tables (Natural (Idx)).Reach);
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
                              if abs Diff (K) > Long_Float'Max (1.0e-6, C.Map.Amp (A * Chan.Per_Arm + K)) * Cap_Mult * Reach then
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
                                 R2 : constant Long_Float := (if Idx >= 0 then Long_Float'Max (1.0, C.Tables (Natural (Idx)).Reach) else 0.0);
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
                                    if abs Diff (J) > Long_Float'Max (1.0e-6, C.Map.Amp (A * Chan.Per_Arm + J)) * (if Idx >= 0 then Cap_Mult * R2 else 1.0) then
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
               Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
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
                  P.Cu := Pred_U; P.Cv := Pred_V; P.Lost := True;
               end if;
            end;
      end case;
   end Retrack;

   --  发一步并等稳;返回实到(通道)
   procedure Step_Arm (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; A : Table.Vec; Jaw : Floats;
                       Delivered : out Table.Vec; Ok : out Boolean; Quick : Boolean := False) is
      P0 : constant Plug.Arm_Pose := F.EE (Arm);
      Frames : Natural;
   begin
      Selfmap.Go (L, C.Map, Arm, Chan.Compose (P0, A), Jaw, F, Delivered, Frames, Ok, Quick);
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
      Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Cam);
   begin
      Trust := [others => False];
      Jaw.Append (Selfmap.Jaw_Of (F, Arm));
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
            Selfmap.Idle (L, F, 1, Ok2);
            for I in 0 .. Natural (Pts.Length) - 1 loop
               declare
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
                                 Table.Set_Col (Effs (I), K, Col);
                              end;
                           else
                              Seen_Enough := False;
                           end if;
                           Pts.Replace_Element (I, P);
                        end;
                     end loop;
                     if Seen_Enough then
                        Trust (K) := True;
                        Put_Line ("[身]     通道" & Natural'Image (Chn) & ":命令 " & Codec.Fmt (Amp, 4) & " 实到 " & Codec.Fmt (Deliv (K), 4) & " ⇒ 点跑了 " &
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
                     exit when Trust (K);
                     if Amp * 2.0 > Cap_Amp then
                        Put_Line ("[身]     通道" & Natural'Image (Chn) & ":到 " & Codec.Fmt (Amp, 4) & " 点还没动过地板(跑 " & Codec.Fmt (Ran_Max, 4) & " 画幅,地板 " & Codec.Fmt (Floor_Px, 4) & ")⇒ 这一段不用它");
                        exit;
                     end if;
                     Amp := Amp * 2.0;
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
            Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
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

   --  ── 一段:把这些点推到各自的目标,直到事件 ──
   procedure Run_Segment (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Cam : Natural; Pts : in out Point_Vectors.Vector;
                          Until_Kind : Monitor.Until_Kind; Step_Limit : Natural; Amount : Long_Float; Avoid : Item_Vectors.Vector;
                          Event : out Unbounded_String; Steps_Taken : out Natural; Blocked_Out : out Boolean; Beats : out Natural) is
      Arm : constant Natural := Pts (0).Arm;
      Beats0 : constant Natural := Plug.Steps (L);
      Reach : Long_Float := 1.0;   --  核实过的步幅倍数(存在表里,越用越强):表这一步报准了就翻倍,报错/认丢了就减半(翻倍协议,次数)
      Effs : Effect_Array (0 .. Natural (Pts.Length) - 1);
      Trusts : array (0 .. Natural (Pts.Length) - 1) of Table.Mask := [others => [others => True]];
      W : Monitor.Watch;
      Fl : Monitor.Floors;
      Cw : constant Natural := F.Cams (Cam).W;
      Ring : Backup.Ring;
      Jaw : Floats;
      Last_Err : Long_Float := 0.0;
      Ok : Boolean;
      Cmd_Floor : Long_Float := 0.0;   --  最小探针幅度:比它一半还小的命令说明不了"顶住"
   begin
      Event := S ("hit the safety cap on steps");
      Steps_Taken := 0;
      Beats := 0;
      Blocked_Out := False;
      Jaw.Append (Selfmap.Jaw_Of (F, Arm));
      Fl.Track := Long_Float'Max (1.0 / Long_Float (Cw), 0.0);
      Fl.Picture := Long_Float (Integer'(if Cam < Natural (C.Map.Pic_Floor.Length) then C.Map.Pic_Floor (Cam) else 0));
      Fl.Reading := C.Map.Jaw_Noise;
      Fl.Delivery := C.Map.EE_Noise;
      Backup.Clear (Ring);
      declare
         Need : Boolean := False;
      begin
         for I in 0 .. Natural (Pts.Length) - 1 loop
            declare
               Idx : constant Integer := Find_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob);
            begin
               if Idx >= 0 then
                  Effs (I) := C.Tables (Natural (Idx)).E;
                  Trusts (I) := C.Tables (Natural (Idx)).Trust;
                  Reach := Long_Float'Max (1.0, C.Tables (Natural (Idx)).Reach);
               else
                  Need := True;
               end if;
            end;
         end loop;
         if Need then
            declare
               Trust : Table.Mask;
            begin
               Probe_Effects (L, C, F, Cam, Pts, Effs, Trust, Ok);
               if not Ok then
                  Event := S ("the body stopped answering while I measured my response table");
                  return;
               end if;
               for I in 0 .. Natural (Pts.Length) - 1 loop
                  Trusts (I) := Trust;
                  Store_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob, Effs (I), Trust);
               end loop;
            end;
         end if;
      end;
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
            Jump : Boolean := False;   --  这一步预计有点在画面里跑过一个跟踪窗:光流跟不住 ⇒ 走完抖手指重新认自己
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
               Cmd_Floor := 0.0;
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
                        Cap (K) := Am * Cap_Mult * Amount * Reach;
                        Cmd_Floor := (if Cmd_Floor <= 0.0 then Am else Long_Float'Min (Cmd_Floor, Am));
                     end if;
                     --  阻尼 = 1e-4 / 幅²:每个通道都以"几个探针幅度"计价(无量纲),小到让上限当家而不是阻尼当家
                     Damp (K) := 1.0e-4 / (Am * Am);
                  end;
               end loop;
               Table.Solve (Terms, Chan.Per_Arm, Cap, Active, Damp, A, Solved);
            end;
            if not Solved then
               Event := S ("could not solve which channels to push");
               return;
            end if;
            --  步子不再按跟踪窗缩(那是小碎步的根子):跑过一个跟踪窗就记成"大步",走完靠抖手指认自己;要压进"别碰"框的才缩
            for I in 0 .. Natural (Pts.Length) - 1 loop
               declare
                  Pr : constant Table.Vec3 := Table.Predict (Effs (I), A);
                  D : constant Long_Float := Sqrt (Pr (0) ** 2 + Pr (1) ** 2);
               begin
                  if D > Track_Win then
                     Jump := True;
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
               Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok, C.Fast);
               Beats := Plug.Steps (L) - Beats0;
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
               declare
                  Was : constant Point_Vectors.Vector := Pts;
                  Need_Refind : Boolean := False;
                  All_Verified : Boolean := True;
                  Any_Wrong : Boolean := False;
               begin
                  --  ① 认位置。我的手指(世界相机里):先"感觉"—— 按此刻位姿从身体图算;熟地只让眼睛核对一下(光流),
                  --     生地或大步就去看(抖手指,看完记进图)。世界里的块:每步重切就近对上。
                  for I in 0 .. Natural (Pts.Length) - 1 loop
                     declare
                        P : Point := Pts (I);
                        Pr : constant Table.Vec3 := Table.Predict (Effs (I), Deliv);
                        W0 : constant Point := Was (I);
                     begin
                        P.Has_Meas := False;
                        if P.Kind = Piece_Pt and then Cam_Arm (C, Cam) /= Integer (P.Arm) then
                           declare
                              Diff : Table.Vec;
                              Dist : Long_Float;
                              Si : constant Integer := Schema.Nearest (C.Sch, P.Arm, Cam, F.EE (P.Arm), C.Map.Amp, Chan.Per_Arm, Diff, Dist);
                              Familiar : Boolean := False;
                              In_Map : Boolean := False;
                           begin
                              if Si >= 0 then
                                 declare
                                    Sm : constant Schema.Sample := C.Sch.S (Natural (Si));
                                 begin
                                    In_Map := P.Chan_K <= Chan.Per_Arm and then Sm.Parts (P.Chan_K).Valid and then (P.Blob < 0 or else Sm.Parts (P.Chan_K).N_Blobs > Natural (P.Blob));
                                 end;
                              end if;
                              if In_Map then
                                 declare
                                    Sm : constant Schema.Sample := C.Sch.S (Natural (Si));
                                    Pm : constant Table.Vec3 := Table.Predict (Effs (I), Diff);
                                    Gp : constant Schema.Part_Pos := Sm.Parts (P.Chan_K);
                                    Su : constant Long_Float := (if P.Blob = 1 then Gp.B1u elsif P.Blob = 0 then Gp.B0u else Gp.Cu);
                                    Sv : constant Long_Float := (if P.Blob = 1 then Gp.B1v elsif P.Blob = 0 then Gp.B0v else Gp.Cv);
                                    Sz : constant Long_Float := Gp.Z;
                                 begin
                                    P.Cu := Long_Float'Max (0.0, Long_Float'Min (1.0, Su + Pm (0)));
                                    P.Cv := Long_Float'Max (0.0, Long_Float'Min (1.0, Sv + Pm (1)));
                                    if Sz > 0.0 then
                                       P.Z := Sz + Pm (2);
                                    end if;
                                    Familiar := True;
                                    for K in 0 .. Chan.Per_Arm - 1 loop
                                       if abs Diff (K) > Long_Float'Max (1.0e-6, C.Map.Amp (P.Arm * Chan.Per_Arm + K)) * Cap_Mult * Reach then
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
                              if Jump or else not Familiar then
                                 P.Lost := True;
                                 Need_Refind := True;
                              else
                                 declare
                                    Q : Point := W0;
                                    Z : constant Zone.Hand_Zone := Zone_Of (C, P.Arm, Cam);
                                 begin
                                    Retrack (C, F, Cam, Before, Q, W0.Cu + Pr (0), W0.Cv + Pr (1), Moved, (if W0.Z > 0.0 then W0.Z + Pr (2) else -1.0));
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
                           Retrack (C, F, Cam, Before, P, W0.Cu + Pr (0), W0.Cv + Pr (1), Moved, (if W0.Z > 0.0 then W0.Z + Pr (2) else -1.0));
                        end if;
                        Pts.Replace_Element (I, P);
                     end;
                  end loop;
                  if Need_Refind then
                     Refind_Pieces (L, C, F, Cam, Pts);
                     Beats := Plug.Steps (L) - Beats0;
                  end if;
                  --  ② 修表:只用真认到的点(按表猜的位置不许再喂回表);表比零表准 = 核实,反之 = 报错
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
                           --  修表只用眼睛量到的(光流核对值或抖手指认到的),按图猜的位置不喂回表
                           Dy (0) := (if P.Has_Meas then P.Meas_U else P.Cu) - W0.Cu; Dy (1) := (if P.Has_Meas then P.Meas_V else P.Cv) - W0.Cv;
                           Dy (2) := (if P.Has_Meas then (if P.Meas_Z > 0.0 and then W0.Z > 0.0 then P.Meas_Z - W0.Z else 0.0)
                                      elsif P.Z > 0.0 and then W0.Z > 0.0 then P.Z - W0.Z else 0.0);
                           Table.Update (E, Deliv, Dy, Fl.Track * 2.0, Long_Float'Max (C.Map.EE_Noise, 0.5 * Cmd_Floor));
                           if Table.Blocked (E) then
                              Any_Blocked := True;
                           end if;
                           if not (E.Null_Res > Fl.Track * 2.0 and then E.Free_Res < E.Null_Res) then
                              All_Verified := False;
                           end if;
                           if E.Free_Res > Fl.Track * 2.0 and then E.Free_Res >= E.Null_Res then
                              Any_Wrong := True;
                           end if;
                        end if;
                        Effs (I) := E;
                        Store_Effect (C, Arm, Cam, P.Kind, P.Chan_K, P.Blob, E, Trusts (I), Reach);
                        Err_Now := Err_Now + Err_Of (P);
                     end;
                  end loop;
                  if All_Verified then
                     Reach := Reach * 2.0;
                  elsif Any_Wrong then
                     Reach := Long_Float'Max (1.0, Reach * 0.5);
                  end if;
                  for I in 0 .. Natural (Pts.Length) - 1 loop
                     Store_Effect (C, Arm, Cam, Pts (I).Kind, Pts (I).Chan_K, Pts (I).Blob, Effs (I), Trusts (I), Reach);
                  end loop;
               end;
               Monitor.Step (W, Monitor.Floor (Long_Float'Max (0.0, Pic_Delta)), Monitor.Bounded (Last_Err), Monitor.Bounded (Err_Now),
                             Monitor.Floor (Long_Float'Max (0.0, Table.Norm (Deliv, Chan.Per_Arm))), Fl);
               Put_Line ("[身]     步" & Natural'Image (Steps_Taken) & (if Jump then "(大步)" else "") & ":误 " & Codec.Fmt (Last_Err, 3) & " → " & Codec.Fmt (Err_Now, 3) & " · 步幅 ×" & Codec.Fmt (Reach, 1) & " · 拍 " & Codec.Img (Beats) &
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

   --  合/张:最多 Max_Iter 拍,或到画面不再变;Sweep_Cam >= 0 时把那台相机里动过的像素累进 Sweep(手指自己扫过的地方)
   procedure Jaw_Sweep (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; Target : Long_Float; Max_Iter : Natural;
                        Sweep_Cam : Integer; Sweep : in out Bools; Steps : out Natural; Reading : out Long_Float) is
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
      for I in 1 .. Max_Iter loop
         Cm.Kind := Plug.Ee; Cm.Arm := Arm; Cm.Pose := F.EE (Arm); Cm.Jaw := Jaw;
         exit when not Plug.Act (L, Cm) or else not Plug.Sense (L, F);
         Steps := I;
         Reading := Selfmap.Jaw_Of (F, Arm);
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
   procedure Move_Jaw (L : in out Plug.Link; C : Context; F : in out Plug.Frame; Arm : Natural; Target : Long_Float; Steps : out Natural; Reading : out Long_Float) is
      None : Bools;
   begin
      Jaw_Sweep (L, C, F, Arm, Target, 40, -1, None, Steps, Reading);
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
         if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm then
            Any_Fingers := True;
         end if;
      end loop;
      if Any_Fingers then
         declare
            Sweep : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Cw * Ch));
            Regs : Picture.Regions;
            Taken : Bools;
         begin
            Jaw_Sweep (L, C, F, Arm, 0.0, C.Map.Settle + 1, Integer (Cam), Sweep, Steps_J, Reading);
            Jaw_Sweep (L, C, F, Arm, J0, C.Map.Settle + 1, Integer (Cam), Sweep, Steps_J, Reading);
            Regs := Picture.Components (Sweep, Cw, Ch, Picture.Min_Pixels (Cw, Ch));
            Taken := Bool_Vectors.To_Vector (False, Regs.Length);
            for I in 0 .. Natural (Pts.Length) - 1 loop
               declare
                  P : Point := Pts (I);
               begin
                  if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm then
                     --  认领半径:一个张幅,再小也有一个跟踪窗;读深窗口 = 张幅的四分之一,再小也有半个百分点的画幅(比例,无量纲)
                     Claim (P, Long_Float'Max (Z.Span, Track_Win), Long_Float'Max (0.005, Z.Span * 0.25), Taken, Regs);
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
                     Claim (P, Long_Float'Max (Long_Float'Max (P.Box_W, P.Box_H), Track_Win), Long_Float'Max (0.005, Long_Float'Max (P.Box_W, P.Box_H) * 0.25), Taken, Regs);
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
            if P.Kind = Piece_Pt and then P.Chan_K = Chan.Per_Arm then
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
         Amount : Long_Float := 1.0;   --  没说 amount 时用满(比例);说了按它的
         Event : Unbounded_String;
         Steps_Taken : Natural := 0;
         Beats : Natural := 0;
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
                  Own : constant Boolean := It.Kind in Finger | Grip | Thing_Held | Piece;
                  Cam_A : constant Integer := Cam_Arm (C, Cam);
                  Ok_Pt : Boolean := True;
               begin
                  Amount := Amount_Factor (G.Amount);
                  P.Item_No := G.Item;
                  if not It.Located then
                     Report := S ("goal: item " & Codec.Img (G.Item) & " is not locatable in this picture right now. ");
                     Ok_Pt := False;
                  elsif It.Kind = Piece then
                     --  我身上的一块零件:点 = 它此刻的形心,表按需量(六个通道各推一下)
                     P.Arm := It.Arm; P.Kind := Piece_Pt; P.Chan_K := It.Which; P.Blob := -1;
                     P.Cu := It.Cu; P.Cv := It.Cv; P.Z := It.Depth;
                     P.Box_W := Long_Float (It.X1 - It.X0) / Long_Float (Cw); P.Box_H := Long_Float (It.Y1 - It.Y0) / Long_Float (Ch);
                  elsif Own then
                     P.Arm := It.Arm; P.Kind := Piece_Pt; P.Chan_K := Chan.Per_Arm;   --  手指 = 握合通道带的那块
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
                                 elsif P.Kind = Piece_Pt and then O.Kind in Thing | Thing_Remembered then
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
                        P.Kind := Piece_Pt; P.Chan_K := Chan.Per_Arm; P.Cu := Tr.Cu; P.Cv := Tr.Cv; P.Z := Tr.Z;
                        P.Tu := O.Cu; P.Tv := O.Cv; P.Tz := O.Depth + O.Height * 0.5; P.Wz := (if O.Depth > 0.0 and then Tr.Z > 0.0 then 1.0 else 0.0);
                        P.Desc := S ("grip " & Codec.Img (A + 1) & " onto item " & Codec.Img (Say.Grip_On) & " (fingertips to its middle)");
                     end;
                  end if;
                  Pts.Append (P);
               end if;
            end;
         end if;
         if not Pts.Is_Empty then
            Expand_Lobes (C, F, Cam, Pts);
            for P of Pts loop
               if P.Desc /= "" then
                  Append (Desc, (if Desc = "" then "" else " and ") & To_String (P.Desc));
               end if;
            end loop;
            Put_Line ("[身] ⚙ 一起解" & Natural'Image (Natural (Pts.Length)) & " 条:" & To_String (Desc));
            Run_Segment (L, C, F, Cam, Pts, Until_K, Step_Limit, Amount, Avoid, Event, Steps_Taken, Blocked, Beats);
            Feel (C, F);
            Report := Report & "you asked " & Desc & ": " & Event & ". I took " & Codec.Img (Steps_Taken) & " pushes; ";
            Put_Line ("[身]   这一段:" & Codec.Img (Steps_Taken) & " 推 · " & Codec.Img (Beats) & " 拍 · 这一集累计 " & Codec.Img (Plug.Steps (L)) & " 拍");
            for P of Pts loop
               if P.Blob <= 0 then
                  Report := Report & "item " & Codec.Img (P.Item_No) & (if P.Blob = 0 then " (finger A)" else "") & " now at (" & Codec.Fmt (P.Cu, 2) & "," & Codec.Fmt (P.Cv, 2) &
                            ") depth " & Codec.Fmt (P.Z, 2) & ", remaining error " & Codec.Fmt (Err_Of (P), 3) & " of a frame; ";
               end if;
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
                        declare
                           Inside : constant Boolean := Pin.Cu * Long_Float (Cw) >= Long_Float (Hz.X0) and then Pin.Cu * Long_Float (Cw) <= Long_Float (Hz.X1)
                                                        and then Pin.Cv * Long_Float (Ch) >= Long_Float (Hz.Y0) and then Pin.Cv * Long_Float (Ch) <= Long_Float (Hz.Y1);
                           Depth_Ok : constant Boolean := Picture.Is_Nan (Hz.Depth) or else Pin.Z <= 0.0 or else abs (Pin.Z - Hz.Depth) <= Long_Float'Max (Pin.Height, Long_Float'Max (Pin.Box_W, Pin.Box_H) * Pin.Z);
                        begin
                           Caged := Inside and then Depth_Ok;
                           Cage_Note := S ("cage check in this hand camera: the thing's centre is " & (if Inside then "inside" else "OUTSIDE") & " my grip box and its depth " &
                                           (if Depth_Ok then "matches" else "does not match") & " my fingertips (" & Codec.Fmt (Pin.Z, 3) & " vs " & Codec.Fmt (Hz.Depth, 3) & ")");
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
