with Ada.Text_IO;
with Codec;
with Limits;
package body Layout is
   use Msgpack;

   function Last_Seg (P : Path) return String is
     (if P.Segs.Is_Empty then "" else P.Segs.Last_Element);

   function Joined (P : Path) return String is
      R : String (1 .. 4096);
      N : Natural := 0;
   begin
      for I in 0 .. Natural (P.Segs.Length) - 1 loop
         declare
            S : constant String := (if I = 0 then "" else ".") & P.Segs (I);
         begin
            if N + S'Length <= R'Last then
               R (N + 1 .. N + S'Length) := S;
               N := N + S'Length;
            end if;
         end;
      end loop;
      return R (1 .. N);
   end Joined;

   function Find (D : Msgpack.Doc; Root : Integer; P : Path) return Integer is
      Cur : Integer := Root;
   begin
      for S of P.Segs loop
         Cur := Key (D, Cur, S);
         if Cur < 0 then
            return -1;
         end if;
      end loop;
      return Cur;
   end Find;

   function Is_Image (D : Msgpack.Doc; N : Integer; W, H : out Natural) return Boolean is
   begin
      W := 0; H := 0;
      if not Is_Nd (D, N) then
         return False;
      end if;
      declare
         T : constant String := Nd_Type (D, N);
         Sh : constant Ints := Nd_Shape (D, N);
      begin
         if T'Length < 2 or else (T (T'Last - 1 .. T'Last) /= "u1" and then T (T'Last - 1 .. T'Last) /= "i1") then
            return False;
         end if;
         if Natural (Sh.Length) = 3 and then Sh (2) = 3 then
            W := Sh (1); H := Sh (0);
            return True;
         end if;
         return False;
      end;
   end Is_Image;

   function Is_Depth (D : Msgpack.Doc; N : Integer; W, H : out Natural) return Boolean is
   begin
      W := 0; H := 0;
      if not Is_Nd (D, N) then
         return False;
      end if;
      declare
         T : constant String := Nd_Type (D, N);
         Sh : constant Ints := Nd_Shape (D, N);
      begin
         if T'Length < 2 or else (T (T'Last - 1 .. T'Last) /= "f4" and then T (T'Last - 1 .. T'Last) /= "f8") then
            return False;
         end if;
         if Natural (Sh.Length) = 2 then
            W := Sh (1); H := Sh (0);
            return True;
         end if;
         return False;
      end;
   end Is_Depth;

   procedure Recognise (D : Msgpack.Doc; Obs : Integer; L : out Body_Layout) is
      type Leaf is record
         P : Path;
         N : Integer;
      end record;
      package Leaf_Vectors is new Ada.Containers.Vectors (Natural, Leaf);
      Flat : Leaf_Vectors.Vector;

      procedure Walk (N : Integer; P : Path) is
      begin
         if Is_Nd (D, N) or else Kind_Of (D, N) /= Map then
            Flat.Append (Leaf'(P, N));
            return;
         end if;
         for I in 0 .. Count (D, N) - 1 loop
            declare
               Q : Path := P;
            begin
               Q.Segs.Append (Text (D, Map_Key (D, N, I)));
               Walk (Map_Val (D, N, I), Q);
            end;
         end loop;
      end Walk;

      W, H : Natural;
      Empty : Path;
      Intr_Raw : Paths;
      Kf, Kcx, Kcy : Long_Float;
   begin
      L := (others => <>);
      Walk (Obs, Empty);
      for F of Flat loop
         declare
            Shape : constant String :=
              (case Kind_Of (D, F.N) is
                  when Arr => "数组[" & Codec.Img (Count (D, F.N)) & "]",
                  when Bin => "字节",
                  when Map => (if Is_Nd (D, F.N) then "nd " & Nd_Type (D, F.N) else "映射"),
                  when others => Kind'Image (Kind_Of (D, F.N)));
         begin
            L.Leaves.Append (Joined (F.P) & "=" & Shape);
         end;
         if Is_Image (D, F.N, W, H) then
            L.Cams.Append (F.P);
         elsif Is_Intrinsic (D, F.N, Kf, Kcx, Kcy) then
            --  先认内参再认深度:3×3 浮点也是"浮点 + 二维",Is_Depth 在前会把它收成一张尺寸对不上的深度图然后丢掉
            --  (09-28 硬件组 PR #1,真 SO-101 + 合成机体实测;驱动本来就不读身体给的内参,这里只管别认错)
            Intr_Raw.Append (F.P);
         elsif Is_Depth (D, F.N, W, H) then
            L.Depth.Append (F.P);
         else
            declare
               Xs : constant Floats := Numbers (D, F.N);
               Nn : constant Natural := Natural (Xs.Length);
               All_Small : Boolean := True;
            begin
               --  I1:每一个数值数组都是一组读数(不看几个数、值在哪);是什么,开机推一下才知道
               if Nn >= 1 and then Kind_Of (D, F.N) in Arr | Map and then Kind_Of (D, F.N) /= Bool then
                  L.Groups.Append (F.P);
               end if;
               for X of Xs loop
                  if abs X > 7.0 then      --  7.0 rad ≈ 2π 带余量:关节角的物理量级,无量纲
                     All_Small := False;
                  end if;
               end loop;
               if Nn = 7 then
                  declare
                     Q : constant Long_Float :=
                       (Xs (3) * Xs (3) + Xs (4) * Xs (4) + Xs (5) * Xs (5) + Xs (6) * Xs (6));
                  begin
                     --  单位四元数的模长容差(无量纲)
                     if abs (Q - 1.0) < 2.0e-3 then
                        L.EE.Append (F.P);
                     elsif All_Small then
                        L.Joints.Append (F.P);
                     else
                        L.Ambiguous.Append (Joined (F.P));
                     end if;
                  end;
               elsif Nn = 6 then
                  if All_Small then
                     L.Joints.Append (F.P);
                  else
                     L.Ambiguous.Append (Joined (F.P));
                  end if;
               elsif Nn >= 1 and then Nn <= Limits.Max_Jaws and then Kind_Of (D, F.N) /= Bool
                 and then Kind_Of (D, F.N) in Arr | Map
                 and then (for all I in 0 .. Nn - 1 => Xs (I) >= 0.0 and then Xs (I) <= 1.0)
               then
                  --  🔴 一串都落在 [0,1] 的数 = 一组抓握通道。以前只认长度为 1 的,
                  --  于是五指手报回来的五个值整组被忽略掉 —— 那是"每条臂只有一个夹爪"这个身体假设的根。
                  L.Jaw.Append (F.P);
               end if;
            end;
         end if;
      end loop;
      --  和一台相机的画面挂在同一个父节点下的数(画幅 shape、时戳……)是那台相机的,不是身体的一组读数(看结构,不看值)
      declare
         function Parent (P : Path) return Path is
            R : Path := P;
         begin
            if not R.Segs.Is_Empty then
               R.Segs.Delete_Last;
            end if;
            return R;
         end Parent;
         Kept : Paths;
      begin
         for G of L.Groups loop
            declare
               Of_Cam : Boolean := False;
            begin
               for Cp of L.Cams loop
                  if not Parent (G).Segs.Is_Empty and then Joined (Parent (G)) = Joined (Parent (Cp)) then
                     Of_Cam := True;
                  end if;
               end loop;
               if not Of_Cam then
                  Kept.Append (G);
               end if;
            end;
         end loop;
         L.Groups := Kept;
      end;
      --  同名的另一组(最后一节一样):动作按最后一节发,对方观测里同一个名字出现两回 = 它把上一条命令回给我们看
      for I in 0 .. Natural (L.Groups.Length) - 1 loop
         declare
            T : Integer := -1;
         begin
            for J in 0 .. Natural (L.Groups.Length) - 1 loop
               if J /= I and then T < 0 and then Last_Seg (L.Groups (J)) = Last_Seg (L.Groups (I)) then
                  T := J;
               end if;
            end loop;
            L.Twin.Append (T);
         end;
      end loop;
      --  深度图必须和某台相机同尺寸,并按最长公共前缀配对(内参 3×3 / 外参 4×4 也是"浮点二维",靠尺寸剔掉)。
      declare
         Real : Paths;
         Ordered : Paths;
      begin
         for Dp of L.Depth loop
            declare
               Dw, Dh : Natural;
               Dn : constant Integer := Find (D, Obs, Dp);
               Hit : Boolean := False;
            begin
               if Is_Depth (D, Dn, Dw, Dh) then
                  for Cp of L.Cams loop
                     declare
                        Cw, Ch : Natural;
                        Cn : constant Integer := Find (D, Obs, Cp);
                     begin
                        if Is_Image (D, Cn, Cw, Ch) and then Cw = Dw and then Ch = Dh then
                           Hit := True;
                        end if;
                     end;
                  end loop;
               end if;
               if Hit then
                  Real.Append (Dp);
               end if;
            end;
         end loop;
         for Cp of L.Cams loop
            declare
               Best : Integer := -1;
               Best_Len : Integer := -1;
            begin
               for I in 0 .. Natural (Real.Length) - 1 loop
                  declare
                     Common : Natural := 0;
                     A : constant Path := Cp;
                     B : constant Path := Real (I);
                  begin
                     while Common < Natural (A.Segs.Length) and then Common < Natural (B.Segs.Length)
                       and then A.Segs (Common) = B.Segs (Common)
                     loop
                        Common := Common + 1;
                     end loop;
                     if Integer (Common) > Best_Len then
                        Best_Len := Integer (Common);
                        Best := I;
                     end if;
                  end;
               end loop;
               if Best >= 0 then
                  Ordered.Append (Real (Best));
               end if;
            end;
         end loop;
         L.Depth := (if Ordered.Is_Empty then Real else Ordered);
      end;
      --  内参按最长公共前缀配到相机;没配上的那台留空路径,保持和 Cams 同序
      for Cp of L.Cams loop
         declare
            Best : Integer := -1;
            Best_Len : Integer := 0;
         begin
            for I in 0 .. Natural (Intr_Raw.Length) - 1 loop
               declare
                  Common : Natural := 0;
                  A : constant Path := Cp;
                  B : constant Path := Intr_Raw (I);
               begin
                  while Common < Natural (A.Segs.Length) and then Common < Natural (B.Segs.Length)
                    and then A.Segs (Common) = B.Segs (Common)
                  loop
                     Common := Common + 1;
                  end loop;
                  if Integer (Common) > Best_Len then
                     Best_Len := Integer (Common);
                     Best := I;
                  end if;
               end;
            end loop;
            if Best >= 0 then
               L.Intr.Append (Intr_Raw (Best));
            else
               L.Intr.Append (Empty);
            end if;
         end;
      end loop;
   end Recognise;

   function Missing (L : Body_Layout) return String is
   begin
      if L.Cams.Is_Empty then
         return "没认出相机";
      end if;
      if L.Groups.Is_Empty then
         return "没认出一组数(读数 / 命令)";
      end if;
      return "";
   end Missing;

   function Command_Groups (L : Body_Layout) return Ints is
      R : Ints;
      Any_Twin : Boolean := False;
   begin
      for T of L.Twin loop
         if T >= 0 then
            Any_Twin := True;
         end if;
      end loop;
      for I in 0 .. Natural (L.Groups.Length) - 1 loop
         if not Any_Twin or else (I < Natural (L.Twin.Length) and then L.Twin (I) >= 0) then
            R.Append (I);
         end if;
      end loop;
      return R;
   end Command_Groups;

   procedure Probe_Mode (L : in out Body_Layout) is
   begin
      L.Joints.Clear;
      for I of Command_Groups (L) loop
         L.Joints.Append (L.Groups (I));
      end loop;
      L.Jaw.Clear; L.Holds.Clear; L.Closing_First.Clear; L.Closing_N.Clear; L.Jaw_Len.Clear;
      L.Measured := False; L.N_Arms := 0;
   end Probe_Mode;

   procedure Set_Measured (L : in out Body_Layout; Joints, Jaw, Holds : Paths; Closing_First, Closing_N, Jaw_Len : Ints; N_Arms : Natural) is
   begin
      L.Joints := Joints; L.Jaw := Jaw; L.Holds := Holds; L.Closing_First := Closing_First; L.Closing_N := Closing_N; L.Jaw_Len := Jaw_Len;
      L.N_Arms := N_Arms; L.Measured := True;
   end Set_Measured;

   procedure Say (L : Body_Layout) is
      procedure Line (Tag : String; Ps : Paths) is
         S : String (1 .. 4096);
         N : Natural := 0;
      begin
         for P of Ps loop
            declare
               T : constant String := (if N = 0 then "" else " · ") & Joined (P);
            begin
               if N + T'Length <= S'Last then
                  S (N + 1 .. N + T'Length) := T;
                  N := N + T'Length;
               end if;
            end;
         end loop;
         Ada.Text_IO.Put_Line ("[认] " & Tag & ":" & S (1 .. N));
      end Line;
   begin
      Line ("关节角", L.Joints);
      Line ("末端位姿", L.EE);
      Line ("夹爪", L.Jaw);
      Line ("相机", L.Cams);
      Line ("深度", L.Depth);
      for A of L.Ambiguous loop
         Ada.Text_IO.Put_Line ("[认] 🔴 分不开:" & A);
      end loop;
      declare
         S : String (1 .. 4096);
         N : Natural := 0;
      begin
         for I in 0 .. Natural (L.Groups.Length) - 1 loop
            declare
               T : constant String := (if N = 0 then "" else " · ") & Codec.Img (I) & "=" & Joined (L.Groups (I))
                 & (if I < Natural (L.Twin.Length) and then L.Twin (I) >= 0 then "(同名 " & Codec.Img (L.Twin (I)) & ")" else "");
            begin
               if N + T'Length <= S'Last then
                  S (N + 1 .. N + T'Length) := T;
                  N := N + T'Length;
               end if;
            end;
         end loop;
         Ada.Text_IO.Put_Line ("[认] 每组数(开机逐组推一下认它是什么):" & S (1 .. N));
      end;
      for Lf of L.Leaves loop
         Ada.Text_IO.Put_Line ("[认] 叶子:" & Lf);
      end loop;
   end Say;
   function Is_Intrinsic (D : Msgpack.Doc; N : Integer; F, Cx, Cy : out Long_Float) return Boolean is
   begin
      F := 0.0; Cx := 0.0; Cy := 0.0;
      if not Is_Nd (D, N) then
         return False;
      end if;
      declare
         T : constant String := Nd_Type (D, N);
         Sh : constant Ints := Nd_Shape (D, N);
      begin
         if T'Length < 2 or else (T (T'Last - 1 .. T'Last) /= "f4" and then T (T'Last - 1 .. T'Last) /= "f8") then
            return False;
         end if;
         if Natural (Sh.Length) /= 2 or else Sh (0) /= 3 or else Sh (1) /= 3 then
            return False;
         end if;
      end;
      declare
         V : constant Floats := Numbers (D, N);
      begin
         if Natural (V.Length) /= 9 or else V (0) <= 0.0 or else V (4) <= 0.0 then
            return False;
         end if;
         --  针孔的样子:对角是焦距、右下是 1、其余接近 0(容差是焦距的百万分之一,比例,无量纲)
         if abs (V (8) - 1.0) > 1.0e-6 or else abs V (1) > V (0) * 1.0e-6 or else abs V (3) > V (0) * 1.0e-6
           or else abs V (6) > V (0) * 1.0e-6 or else abs V (7) > V (0) * 1.0e-6
         then
            return False;
         end if;
         F := V (0); Cx := V (2); Cy := V (5);
         return True;
      end;
   end Is_Intrinsic;

end Layout;
