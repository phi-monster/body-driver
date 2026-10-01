with Ada.Numerics.Float_Random;
with Ada.Containers.Vectors;
package body Selfmap.Detour is

   function Lerp (A, B : Floats; T : Long_Float) return Floats is
      R : Floats;
   begin
      for I in 0 .. Natural'Min (Natural (A.Length), Natural (B.Length)) - 1 loop
         R.Append (A (I) + T * (B (I) - A (I)));
      end loop;
      return R;
   end Lerp;

   procedure Segment (A, B : Floats; Margin : Margin_Fn; Shift : Shift_Fn; Res : Long_Float; Reach : out Long_Float; Free : out Boolean) is
      T : Long_Float := 0.0;   --  走到这一段的几成
   begin
      Reach := 0.0; Free := False;
      loop
         declare
            Qt : constant Floats := Lerp (A, B, T);
            M : constant Long_Float := Margin (Qt);
         begin
            if M <= 0.0 then
               Reach := T;            --  在带子里:走不通
               return;
            end if;
            if Shift (Qt, B) <= M then
               Reach := 1.0; Free := True;   --  剩下的这一截所有点挪的都不到净空:碰不上
               return;
            end if;
            --  这一步最多走到哪还碰不上(所有点挪的不超过净空 M):沿这一段二分,区间两头挪的差细过一档就停
            declare
               Lo_Dt : Long_Float := 0.0;
               Hi_Dt : Long_Float := 1.0 - T;
            begin
               while Shift (Lerp (A, B, T + Lo_Dt), Lerp (A, B, T + Hi_Dt)) > Res loop
                  declare
                     Mid : constant Long_Float := (Lo_Dt + Hi_Dt) / 2.0;   --  二分
                  begin
                     if Shift (Qt, Lerp (A, B, T + Mid)) <= M then
                        Lo_Dt := Mid;
                     else
                        Hi_Dt := Mid;
                     end if;
                  end;
               end loop;
               --  这一步挪不出一档 ⇒ 贴着带子,再走就进去了:走不通
               if Shift (Qt, Lerp (A, B, T + Lo_Dt)) < Res then
                  Reach := T;
                  return;
               end if;
               T := T + Lo_Dt;
            end;
         end;
      end loop;
   end Segment;

   type Node is record
      Q : Floats;
      Parent : Integer := -1;
   end record;
   package Node_Vectors is new Ada.Containers.Vectors (Natural, Node);
   type Ext is (Trapped, Advanced, Reached);

   procedure Plan (Start, Goal, Lo, Hi : Floats; Margin : Margin_Fn; Shift : Shift_Fn; Res : Long_Float; Seed : Integer;
                   Path : out Plug.Floats_Vectors.Vector; Found : out Boolean; Tries : out Natural) is
      N : constant Natural := Natural'Min (Natural (Start.Length), Natural'Min (Natural (Lo.Length), Natural (Hi.Length)));
      Gen : Ada.Numerics.Float_Random.Generator;
      Ta, Tb : Node_Vectors.Vector;
      A_From_Start : Boolean := True;   --  Ta 是从起点长的那一棵
      --  关节空间里的远近:每一维按量到的关节范围折成比例(范围是 0 的那一维按它自己的单位)
      function Dist2 (P, Q : Floats) return Long_Float is
         S : Long_Float := 0.0;
      begin
         for I in 0 .. N - 1 loop
            declare
               Span : constant Long_Float := Hi (I) - Lo (I);
               D : constant Long_Float := (P (I) - Q (I)) / (if Span > 0.0 then Span else 1.0);
            begin
               S := S + D * D;
            end;
         end loop;
         return S;
      end Dist2;
      function Nearest (T : Node_Vectors.Vector; Q : Floats) return Natural is
         Best : Natural := 0;
         Bd : Long_Float := Long_Float'Last;
      begin
         for I in 0 .. Natural (T.Length) - 1 loop
            if Dist2 (T (I).Q, Q) < Bd then
               Bd := Dist2 (T (I).Q, Q); Best := I;
            end if;
         end loop;
         return Best;
      end Nearest;
      --  这棵树朝 Q 长:从离它最近的那一处沿关节直线走到走不通为止(走通了就到 Q);长出来的那一处至少挪了一档才算长了
      procedure Extend (T : in out Node_Vectors.Vector; Q : Floats; R : out Ext; New_I : out Natural) is
         Near : constant Natural := Nearest (T, Q);
         Q_Near : constant Floats := T (Near).Q;   --  先拷出来:往同一棵树里加的时候不许还拿着它里面的一格
         Q_To : constant Floats := Q;
         Reach : Long_Float;
         Free : Boolean;
      begin
         New_I := Near;
         Segment (Q_Near, Q_To, Margin, Shift, Res, Reach, Free);
         if Free then
            T.Append (Node'(Q => Q_To, Parent => Integer (Near)));
            New_I := Natural (T.Length) - 1;
            R := Reached;
         elsif Reach > 0.0 and then Shift (Q_Near, Lerp (Q_Near, Q_To, Reach)) >= Res then
            T.Append (Node'(Q => Lerp (Q_Near, Q_To, Reach), Parent => Integer (Near)));
            New_I := Natural (T.Length) - 1;
            R := Advanced;
         else
            R := Trapped;
         end if;
      end Extend;
      --  一棵树从第 I 处一路倒回它的根
      function Back_To_Root (T : Node_Vectors.Vector; I : Natural) return Plug.Floats_Vectors.Vector is
         P : Plug.Floats_Vectors.Vector;
         K : Integer := Integer (I);
      begin
         while K >= 0 loop
            P.Append (T (Natural (K)).Q);
            K := T (Natural (K)).Parent;
         end loop;
         return P;
      end Back_To_Root;
      --  捷径:从头起,每一处直接连到还能直走通的最远那一处
      function Shortcut (P : Plug.Floats_Vectors.Vector) return Plug.Floats_Vectors.Vector is
         R : Plug.Floats_Vectors.Vector;
         I : Natural := 0;
         Last : constant Natural := Natural (P.Length) - 1;
      begin
         R.Append (P (0));
         while I < Last loop
            declare
               J : Natural := I + 1;
               Reach : Long_Float;
               Free : Boolean;
            begin
               for K in reverse I + 2 .. Last loop
                  Segment (P (I), P (K), Margin, Shift, Res, Reach, Free);
                  if Free then
                     J := K;
                     exit;
                  end if;
               end loop;
               R.Append (P (J));
               I := J;
            end;
         end loop;
         return R;
      end Shortcut;
   begin
      Path.Clear; Found := False; Tries := 0;
      if N = 0 then
         return;
      end if;
      declare
         Reach : Long_Float;
         Free : Boolean;
      begin
         Segment (Start, Goal, Margin, Shift, Res, Reach, Free);
         if Free then
            Path.Append (Start); Path.Append (Goal); Found := True;
            return;
         end if;
         if Margin (Start) <= 0.0 or else Margin (Goal) <= 0.0 then
            return;   --  起点 / 终点自己就在带子里:没有一条路是碰不上的(照实说没找到)
         end if;
      end;
      Ada.Numerics.Float_Random.Reset (Gen, Seed);
      Ta.Append (Node'(Q => Start, Parent => -1));
      Tb.Append (Node'(Q => Goal, Parent => -1));
      for K in 1 .. Max_Tries loop
         Tries := K;
         declare
            Q_Rand : Floats;
            R, R2 : Ext;
            I_New, J_New : Natural;
         begin
            for I in 0 .. N - 1 loop
               Q_Rand.Append (Lo (I) + Long_Float (Ada.Numerics.Float_Random.Random (Gen)) * (Hi (I) - Lo (I)));
            end loop;
            Extend (Ta, Q_Rand, R, I_New);
            if R /= Trapped then
               declare
                  Q_New : constant Floats := Ta (I_New).Q;
               begin
                  loop
                     Extend (Tb, Q_New, R2, J_New);
                     exit when R2 /= Advanced;
                  end loop;
               end;
               if R2 = Reached then
                  declare
                     Pa : constant Plug.Floats_Vectors.Vector := Back_To_Root (Ta, I_New);   --  Ta 的根 ← … ← 接上的那一处(倒着)
                     Pb : constant Plug.Floats_Vectors.Vector := Back_To_Root (Tb, J_New);   --  接上的那一处 → … → Tb 的根
                     Whole : Plug.Floats_Vectors.Vector;
                  begin
                     --  拼成 Ta 的根 → 接上的那一处 → Tb 的根(Tb 的第一个就是接上的那一处,不重复)
                     for I in reverse 0 .. Natural (Pa.Length) - 1 loop
                        Whole.Append (Pa (I));
                     end loop;
                     for I in 1 .. Natural (Pb.Length) - 1 loop
                        Whole.Append (Pb (I));
                     end loop;
                     if not A_From_Start then
                        Whole.Reverse_Elements;
                     end if;
                     Path := Shortcut (Whole);
                     Found := True;
                     return;
                  end;
               end if;
            end if;
            declare
               Tmp : constant Node_Vectors.Vector := Ta;
            begin
               Ta := Tb; Tb := Tmp;
               A_From_Start := not A_From_Start;
            end;
         end;
      end loop;
   end Plan;
end Selfmap.Detour;
