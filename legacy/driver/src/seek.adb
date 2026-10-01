with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Containers.Vectors;
with Codec;
with Contact;
with Linkage;
with Stats;
package body Seek is
   function Add (A, B : V3) return V3 is ([A (0) + B (0), A (1) + B (1), A (2) + B (2)]);
   function Sub (A, B : V3) return V3 is ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]);
   function Scl (A : V3; S : Long_Float) return V3 is ([S * A (0), S * A (1), S * A (2)]);

   procedure Run (Here, Est : V3; Cov : M3; Axis : V3; Depth, Depth_Sd, Gap, Resolution : Long_Float;
                  Move : access procedure (Dir : V3; Len : Long_Float; R : out Step_Report);
                  Rep : out Report) is
      Ok : Boolean;
      A : constant V3 := Contact.Unit (Axis, Ok);
   begin
      Rep := (others => <>);
      if not Ok or else Move = null or else not (Resolution > 0.0) then
         Rep.How := No_Axis;
         return;
      end if;
      declare
         --  垂直于轴的一对轴:辅助方向取 A 分量绝对值最小的那根坐标轴(和 A 最不平行,叉乘永远不退化)
         Aux : V3 := [others => 0.0];
         Low : Natural := 0;
      begin
         for C in 1 .. 2 loop
            if abs A (C) < abs A (Low) then
               Low := C;
            end if;
         end loop;
         Aux (Low) := 1.0;
         declare
            E1 : constant V3 := Contact.Unit (Contact.Cross (A, Aux), Ok);
            E2 : constant V3 := Contact.Cross (A, E1);
            function Q (U, W : V3) return Long_Float is (Contact.Dot (U, Geom.Ap (Cov, W)));
            C11 : constant Long_Float := Q (E1, E1);
            C12 : constant Long_Float := Q (E1, E2);
            C22 : constant Long_Float := Q (E2, E2);
            Det : constant Long_Float := C11 * C22 - C12 ** 2;
            Rel : constant V3 := Sub (Est, Here);
            Ax_Est : constant Long_Float := Contact.Dot (Rel, A);
            Lat_Est : constant V3 := Sub (Rel, Scl (A, Ax_Est));
            Need : constant Long_Float := Ax_Est + Depth;                           --  从起点沿轴要送多远
            Need_Sd : constant Long_Float := Sqrt (Long_Float'Max (0.0, Q (A, A)) + Depth_Sd ** 2);
            G2 : constant Long_Float := Linkage.Gate (2);                          --  横着的二维门
            H : constant Long_Float := Long_Float'Max (Resolution, Sqrt (2.0) * Long_Float'Max (0.0, Gap));
            Pos : V3 := [others => 0.0];   --  拿着的那一处离起点 Here 多远(每一步走完重量的)
            type Cand is record
               I, J : Integer := 0;
               M2 : Long_Float := 0.0;
            end record;
            function Before (X, Y : Cand) return Boolean is
              (X.M2 < Y.M2 or else (X.M2 = Y.M2 and then (X.I < Y.I or else (X.I = Y.I and then X.J < Y.J))));
            package Cand_Vectors is new Ada.Containers.Vectors (Natural, Cand);
            package Cand_Sort is new Cand_Vectors.Generic_Sorting (Before);
            Cands : Cand_Vectors.Vector;
            procedure Go (Dir : V3; Len : Long_Float; R : out Step_Report) is
            begin
               R := (others => <>);
               if not (Len > 0.0) then
                  R.Ok := True;
                  return;
               end if;
               Move (Dir, Len, R);
               if R.Ok then
                  Pos := Sub (R.At_Now, Here);
               end if;
            end Go;
         begin
            Rep.Spacing := H;
            --  顶在孔口那一面上(沿轴走到 Ax_Est ± Z 倍沿轴的不准)和送到位(Need ± Z × Need_Sd)要隔得开,不然进没进去分不出
            if Depth <= Stats.Z * (Need_Sd + Sqrt (Long_Float'Max (0.0, Q (A, A)))) then
               Rep.How := Too_Shallow;
               return;
            end if;
            if not Ok or else not (Det > 0.0) then
               --  横着一点不准都没有(或者定不住):只在估计的那一处送一次
               Cands.Append (Cand'(I => 0, J => 0, M2 => 0.0));
            else
               declare
                  N1 : constant Integer := Integer (Long_Float'Floor (Sqrt (G2 * C11) / H));
                  N2 : constant Integer := Integer (Long_Float'Floor (Sqrt (G2 * C22) / H));
               begin
                  for I in -N1 .. N1 loop
                     for J in -N2 .. N2 loop
                        declare
                           X1 : constant Long_Float := H * Long_Float (I);
                           X2 : constant Long_Float := H * Long_Float (J);
                           M2 : constant Long_Float := (C22 * X1 ** 2 - 2.0 * C12 * X1 * X2 + C11 * X2 ** 2) / Det;
                        begin
                           if M2 <= G2 then
                              Cands.Append (Cand'(I => I, J => J, M2 => M2));
                           end if;
                        end;
                     end loop;
                  end loop;
               end;
            end if;
            Cand_Sort.Sort (Cands);
            Rep.Candidates := Natural (Cands.Length);
            for C of Cands loop
               declare
                  Target : constant V3 := Add (Lat_Est, Add (Scl (E1, H * Long_Float (C.I)), Scl (E2, H * Long_Float (C.J))));
                  Lat_Now : constant V3 := Sub (Pos, Scl (A, Contact.Dot (Pos, A)));
                  D : constant V3 := Sub (Target, Lat_Now);
                  Dl : constant Long_Float := Geom.Norm (D);
                  R : Step_Report;
                  Lat_Ok : Boolean := True;
               begin
                  Rep.Lateral := Sub (Target, Lat_Est);
                  Rep.Mahal_Max := Sqrt (C.M2);
                  --  横着挪到这一处(拿着的东西已经退回送之前那一处,横着不该被挡;被挡住 ⇒ 这一处去不了,换下一处)
                  if Dl > 0.0 then
                     Go (Scl (D, 1.0 / Dl), Dl, R);
                     if not R.Ok then
                        Rep.How := Body_Failed;
                        return;
                     end if;
                     Lat_Ok := not R.Blocked;
                  end if;
                  if Lat_Ok then
                     --  沿轴往里送
                     Go (A, Need - Contact.Dot (Pos, A), R);
                     if not R.Ok then
                        Rep.How := Body_Failed;
                        return;
                     end if;
                     Rep.Tries := Rep.Tries + 1;
                     Rep.Went_In := Contact.Dot (Pos, A);
                     if Rep.Went_In >= Need - Stats.Z * Need_Sd then
                        Rep.How := Reached;
                        return;
                     end if;
                     --  进不去:退回送之前那一处
                     Go (Scl (A, -1.0), Contact.Dot (Pos, A), R);
                     if not R.Ok then
                        Rep.How := Body_Failed;
                        return;
                     end if;
                  end if;
               end;
            end loop;
            Rep.How := Not_Found;
         end;
      end;
   end Run;

   function Say (Rep : Report) return String is
     ((case Rep.How is
          when Reached => "进去了",
          when Not_Found => "不准那一片里没找到",
          when Body_Failed => "身体那一步没做成",
          when No_Axis => "没有往里送的方向",
          when Too_Shallow => "送进去的深度比沿轴的不准还浅,进没进去分不出")
      & "(送了 " & Codec.Img (Rep.Tries) & " 次,不准那一片里一共 " & Codec.Img (Rep.Candidates) & " 处,格子隔 " & Codec.Fmt (Rep.Spacing, 4)
      & ",最远试到 " & Codec.Fmt (Rep.Mahal_Max, 2) & " 倍不准,沿轴送了 " & Codec.Fmt (Rep.Went_In, 4) & ")");
end Seek;
