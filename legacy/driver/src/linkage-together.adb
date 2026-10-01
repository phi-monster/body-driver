with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Codec;
package body Linkage.Together is
   function "+" (A, B : V3) return V3 is ([A (0) + B (0), A (1) + B (1), A (2) + B (2)]);
   function "-" (A, B : V3) return V3 is ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]);

   procedure Check (Ts_A : Contact.Wrench.Touch_Vectors.Vector; Com_A, Up : V3; Sup : Contact.Wrench.Surface;
                    Ts_B : Contact.Wrench.Touch_Vectors.Vector; Allowed_Half : Long_Float;
                    J : Joint; Pa : Pose; Sign : Long_Float; Mu_Hand, Mu_Surf : Long_Float; Rep : out Report) is
      Ok : Boolean;
      --  B 沿轴动一个单位(往 Sign 那边)的旋量:它在一点的速度 = Lin + Ang × (p − Pivot)
      Tw : constant Contact.Twist := Motion_Of (J, Pa, (if Sign >= 0.0 then 1.0 else -1.0), Ok);
      function Vel (P : V3) return V3 is
        (Contact.Cross (Tw.Ang, [P (0) - Tw.Pivot (0), P (1) - Tw.Pivot (1), P (2) - Tw.Pivot (2)]) + Tw.Lin);
      Ca : constant Long_Float := Cos (Allowed_Half);
      Sa : constant Long_Float := Sin (Allowed_Half);
      Sum_P : V3 := [others => 0.0];
      Ps : Geom.V3_Vectors.Vector;
      Fs : Geom.V3_Vectors.Vector;
   begin
      Rep := (others => <>);
      if not Ok then
         Rep.Why := Axis_Unknown;
         return;
      end if;
      --  ① 碰 B 的每一处:锥里和这一点沿轴动的速度夹角最小的方向;做的功是正的才算推得动
      for T of Ts_B loop
         declare
            V : constant V3 := Vel (T.P);
            Okv, Okn : Boolean;
            Vu : constant V3 := Contact.Unit (V, Okv);
            N : constant V3 := Contact.Unit (T.N, Okn);
         begin
            if Okv and then Okn then
               declare
                  C : constant Long_Float := Contact.Dot (N, Vu);
                  Perp_V : constant V3 := [Vu (0) - C * N (0), Vu (1) - C * N (1), Vu (2) - C * N (2)];
                  Okp : Boolean;
                  Tu : constant V3 := Contact.Unit (Perp_V, Okp);
                  F : V3;
               begin
                  if C >= Ca then
                     F := Vu;                                   --  速度方向就在锥里
                  elsif Okp then
                     F := [Ca * N (0) + Sa * Tu (0), Ca * N (1) + Sa * Tu (1), Ca * N (2) + Sa * Tu (2)];   --  锥边上离速度最近的那一条
                  else
                     F := N;                                    --  速度正好和锥轴反着:锥里哪个方向都一样
                  end if;
                  if Contact.Dot (F, V) > 0.0 then
                     Rep.Driving := Rep.Driving + 1;
                     Ps.Append (T.P);
                     Fs.Append (F);
                     Sum_P := Sum_P + T.P;
                  end if;
               end;
            end if;
         end;
      end loop;
      if Rep.Driving = 0 then
         Rep.Why := Not_Driven;
         return;
      end if;
      Rep.Ref := [Sum_P (0) / Long_Float (Rep.Driving), Sum_P (1) / Long_Float (Rep.Driving), Sum_P (2) / Long_Float (Rep.Driving)];
      for K in 0 .. Rep.Driving - 1 loop
         declare
            Pk : constant V3 := Ps (K);
            Fk : constant V3 := Fs (K);
         begin
            Rep.F := Rep.F + Fk;
            Rep.Mo := Rep.Mo + Contact.Cross (Pk - Rep.Ref, Fk);
         end;
      end loop;
      --  ② A 抵得住自己的重量
      Rep.Need_Weight := Contact.Wrench.Need (Ts_A, Com_A, Up, Sup, Contact.Still (Com_A), Mu_Hand, Mu_Surf, Rep.Why_A);
      if Rep.Need_Weight = Contact.Wrench.No_Way then
         Rep.Why := A_Falls;
         return;
      end if;
      --  ③ A 抵得住推 B 的那个力(传到 A 上的就是它,作用在碰 B 的那几处):握着 A 的那几处要产生它的反方向
      declare
         Neg_F : constant V3 := [-Rep.F (0), -Rep.F (1), -Rep.F (2)];
         Neg_Mo : constant V3 := [-Rep.Mo (0), -Rep.Mo (1), -Rep.Mo (2)];
         Okf : Boolean;
         Al : constant V3 := Contact.Unit (Neg_F, Okf);
         Fn : constant Long_Float := Geom.Norm (Rep.F);
         Least : constant Long_Float :=
           Contact.Wrench.Least (Ts_A, Rep.Ref, Neg_F, Neg_Mo, (if Okf then Al else Up), Sup, Contact.Still (Rep.Ref), Mu_Hand, Mu_Surf, Rep.Why_A);
      begin
         if Least = Contact.Wrench.No_Way then
            Rep.Why := A_Pushed_Away;
            return;
         end if;
         Rep.Need_Push := (if Fn > 0.0 then Least / Fn else Least);
      end;
      Rep.Why := Fine;
   end Check;

   function Say (Rep : Report) return String is
     ((case Rep.Why is
          when Fine => "两个要一起做得到",
          when Axis_Unknown => "两块之间的轴没定下来,不知道 B 往哪动",
          when Not_Driven => "碰 B 的那几处哪一处都推不动它沿轴动",
          when A_Falls => "握着 A 的那几处抵不住它的重量",
          when A_Pushed_Away => "握着 A 的那几处抵不住推 B 的那个力(A 会被推走)")
      & "(推得动 B 的 " & Codec.Img (Rep.Driving) & " 处;抵住重量要 "
      & (if Rep.Need_Weight = Contact.Wrench.No_Way then "做不到" else Codec.Fmt (Rep.Need_Weight, 2))
      & "、抵住推 B 的力要 " & (if Rep.Need_Push = Contact.Wrench.No_Way then "做不到" else Codec.Fmt (Rep.Need_Push, 2)) & ")");
end Linkage.Together;
