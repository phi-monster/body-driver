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
         elsif Is_Depth (D, F.N, W, H) then
            L.Depth.Append (F.P);
         else
            declare
               Xs : constant Floats := Numbers (D, F.N);
               Nn : constant Natural := Natural (Xs.Length);
               All_Small : Boolean := True;
            begin
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
   end Recognise;

   function Missing (L : Body_Layout) return String is
   begin
      if L.EE.Is_Empty and then L.Joints.Is_Empty then
         return "没认出末端位姿也没认出关节角";
      end if;
      if L.Jaw.Is_Empty then
         return "没认出夹爪开度";
      end if;
      if L.Cams.Is_Empty then
         return "没认出相机";
      end if;
      if not L.Ambiguous.Is_Empty then
         return "有形状分不开的读数,拒绝硬认";
      end if;
      return "";
   end Missing;

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
      for Lf of L.Leaves loop
         Ada.Text_IO.Put_Line ("[认] 叶子:" & Lf);
      end loop;
   end Say;
end Layout;
