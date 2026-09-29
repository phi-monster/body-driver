with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
package body Contact.Exec is

   function Nat_Img (N : Natural) return String is (Ada.Strings.Fixed.Trim (Natural'Image (N), Ada.Strings.Left));

   function Tool_Axis (F : M3) return V3 is ([F (0, 2), F (1, 2), F (2, 2)]);

   --  轴角 → 旋转阵;零向量 = 不转
   function Rot_Of (Ang : V3) return M3 is (if Norm (Ang) < 1.0e-12 then Geom.Identity else Geom.Rodrigues (Ang));

   function Frame_From (S : Set; Ok : out Boolean) return M3 is
      Z, X, Y : V3 := [others => 0.0];
      Oz : Boolean := False;
      Ox : Boolean := False;
      Best : Long_Float := 0.0;
      Have_X : Boolean := False;
      Dx : V3 := [others => 0.0];
      F : M3 := Geom.Identity;
   begin
      Ok := False;
      --  进场方向优先用接触集给的那一项。四格定不下它:对夹时两个锥正好相反、两个法向也正好相反,合成恰好为零
      if S.Has_Approach then
         Z := Unit (S.Approach, Oz);
      end if;
      if not Oz then
         declare
            Push : V3 := [others => 0.0];
            Oa : Boolean;
         begin
            for P of S.Points loop
               if P.By.Kind = Hand then
                  declare
                     A : constant V3 := Unit (P.Push.Axis, Oa);
                  begin
                     if Oa then
                        Push := [Push (0) + A (0), Push (1) + A (1), Push (2) + A (2)];
                     end if;
                  end;
               end if;
            end loop;
            Z := Unit (Push, Oz);
            if not Oz then
               --  合成为零时退回"沿各点法向的反向合";再为零就拒绝,不许瞎挑一个 —— 挑错了物体会被爪子侧面撞飞,而没有任何一个环节会不一致
               Push := [others => 0.0];
               for P of S.Points loop
                  if P.By.Kind = Hand then
                     declare
                        Nn : constant V3 := Unit (P.Normal, Oa);
                     begin
                        if Oa then
                           Push := [Push (0) - Nn (0), Push (1) - Nn (1), Push (2) - Nn (2)];
                        end if;
                     end;
                  end if;
               end loop;
               Z := Unit (Push, Oz);
               if not Oz then
                  return F;
               end if;
            end if;
         end;
      end if;
      --  开合轴:相距最远的一对手接触点的连线
      for I in 0 .. Natural (S.Points.Length) - 1 loop
         for J in I + 1 .. Natural (S.Points.Length) - 1 loop
            if S.Points (I).By.Kind = Hand and then S.Points (J).By.Kind = Hand then
               declare
                  D : constant V3 := [S.Points (J).Pos (0) - S.Points (I).Pos (0), S.Points (J).Pos (1) - S.Points (I).Pos (1), S.Points (J).Pos (2) - S.Points (I).Pos (2)];
                  Ln : constant Long_Float := Norm (D);
               begin
                  if Ln > Best then
                     Best := Ln;
                     Dx := D;
                     Have_X := True;
                  end if;
               end;
            end if;
         end loop;
      end loop;
      --  投到与工具轴垂直的平面上;没有开合轴(单点)或投影为零就任取一条垂直的(0.9 是无量纲的比较:挑一条不和工具轴平行的种子轴)
      if Have_X then
         declare
            V : constant V3 := Unit (Dx, Ox);
            Dd : constant Long_Float := Dot (V, Z);
         begin
            if Ox then
               X := Unit ([V (0) - Z (0) * Dd, V (1) - Z (1) * Dd, V (2) - Z (2) * Dd], Ox);
            end if;
         end;
      end if;
      if not Ox then
         declare
            --  0.9 是无量纲的比较:挑一条不和工具轴平行的种子轴
            Seed : constant V3 := (if abs Z (0) < 0.9 then [1.0, 0.0, 0.0] else [0.0, 1.0, 0.0]);
            Dd : constant Long_Float := Dot (Seed, Z);
         begin
            X := Unit ([Seed (0) - Z (0) * Dd, Seed (1) - Z (1) * Dd, Seed (2) - Z (2) * Dd], Ox);
            if not Ox then
               return F;
            end if;
         end;
      end if;
      Y := Cross (Z, X);
      for I in 0 .. 2 loop
         F (I, 0) := X (I);
         F (I, 1) := Y (I);
         F (I, 2) := Z (I);
      end loop;
      Ok := True;
      return F;
   end Frame_From;

   package Nat_Sort is new Nat_Vectors.Generic_Sorting;

   procedure Steps (S : Set; L : Hand_Limits; Must_Move : Boolean; Arc : Positive; Out_Steps : out Step_Vectors.Vector; Why : out No_Plan) is
      G : constant Gap := Check (S, Must_Move);
      Hp : Point_Vectors.Vector;
      Ids : Nat_Vectors.Vector;
      Frames_Of : M3_Vectors.Vector;
      Tol : Long_Float := Long_Float'Last;
      Here, Back : V3_Vectors.Vector;
      Who : Nat_Vectors.Vector;
      Fr : M3_Vectors.Vector;
   begin
      Out_Steps := Step_Vectors.Empty_Vector;
      Why := (others => <>);
      if G.Kind /= Fine then
         Why := (Kind => Bad, G => G, others => <>);
         return;
      end if;
      for I in 0 .. Natural (S.Points.Length) - 1 loop
         if S.Points (I).Tol_M < L.Repeat_M then
            Why := (Kind => Tol_Tighter_Than_Body, Index => I, others => <>);
            return;
         end if;
      end loop;
      --  只有手的接触变成航点。世界那一侧的接触(桌面顶着的那条边、卡具、墙)参与"能不能驱动"的计算,但手够不到它 —— 当航点发出去,机械臂会去戳桌子底下
      for P of S.Points loop
         if P.By.Kind = Hand then
            Hp.Append (P);
         end if;
      end loop;
      if Hp.Is_Empty then
         Why.Kind := No_Hand_Contact;
         return;
      end if;
      --  一只手一个朝向:按执行器编号分堆,各算各的(一只五指手是一个手腕;双臂抱一个箱子是两个手腕,合成一个没有意义)
      for P of Hp loop
         if not Ids.Contains (P.By.Id) then
            Ids.Append (P.By.Id);
         end if;
      end loop;
      Nat_Sort.Sort (Ids);
      for Id of Ids loop
         declare
            Sub : Set := S;
            Ok : Boolean;
            F : M3;
         begin
            Sub.Points := Point_Vectors.Empty_Vector;
            for P of S.Points loop
               if P.By.Kind = World or else P.By.Id = Id then
                  Sub.Points.Append (P);
               end if;
            end loop;
            F := Frame_From (Sub, Ok);
            if not Ok then
               Why.Kind := No_Frame;
               return;
            end if;
            Frames_Of.Append (F);
         end;
      end loop;
      for P of Hp loop
         declare
            K : constant Natural := Natural (Ids.Find_Index (P.By.Id));
            F : constant M3 := Frames_Of (K);
            Z : constant V3 := Tool_Axis (F);
         begin
            Tol := Long_Float'Min (Tol, P.Tol_M);
            Here.Append (P.Pos);
            --  悬停:各自沿自己那只手的工具轴往回退 Standoff(两只手退的方向可以不一样)
            Back.Append (V3'([P.Pos (0) - Z (0) * L.Standoff_M, P.Pos (1) - Z (1) * L.Standoff_M, P.Pos (2) - Z (2) * L.Standoff_M]));
            Who.Append (P.By.Id);
            Fr.Append (F);
         end;
      end loop;
      --  悬停:还没接触,容差放宽到进场余量那一档(路过的地方厘米级);贴上:开始接触,容差用最严的那一个
      Out_Steps.Append (Step'(Pos => Back, Frame => Fr, Hand => Who, Touching => False, Tol_M => Long_Float'Max (L.Standoff_M, Tol), Kind => Hover));
      Out_Steps.Append (Step'(Pos => Here, Frame => Fr, Hand => Who, Touching => True, Tol_M => Tol, Kind => Touch));
      --  按③把接触点一段一段搬过去;手跟着物体转 —— 每只手各自跟着转,同一个物体旋量作用在各自的手腕上
      if Moving (S.Motion) then
         for K in 1 .. Arc loop
            declare
               Fk : constant Long_Float := Long_Float (K) / Long_Float (Arc);
               Part : constant Twist := (Lin => [S.Motion.Lin (0) * Fk, S.Motion.Lin (1) * Fk, S.Motion.Lin (2) * Fk],
                                         Ang => [S.Motion.Ang (0) * Fk, S.Motion.Ang (1) * Fk, S.Motion.Ang (2) * Fk],
                                         Pivot => S.Motion.Pivot);
               R : constant M3 := Rot_Of (Part.Ang);
               At_K : V3_Vectors.Vector;
               Fr_K : M3_Vectors.Vector;
            begin
               for P of Here loop
                  At_K.Append (Apply (Part, P));
               end loop;
               for F of Fr loop
                  Fr_K.Append (Geom.Mul (R, F));
               end loop;
               Out_Steps.Append (Step'(Pos => At_K, Frame => Fr_K, Hand => Who, Touching => True, Tol_M => Tol, Kind => Carry));
            end;
         end loop;
      end if;
   end Steps;

   --  只是路过的点永远不算"偏了":它的职责只有"从物体上面绕过去",差几厘米不改变任何事(渲图拍到爪子已经张开停在物体正上方,而代码因为悬停点差 5.11 cm 判偏、一遍遍重算,一次都没往下走过)。
   --  要碰的点:门槛 = 这具身体重复精度的两倍(纯数学的倍数),不另拍一个毫米数
   function Off_Course (L : Hand_Limits; St : Step; Residual_M : Long_Float) return Boolean is
     (St.Touching and then Residual_M > 2.0 * L.Repeat_M);

   function Img (N : No_Plan) return String is
   begin
      case N.Kind is
         when Fine => return "fine";
         when Bad => return "Bad(" & Img (N.G) & ")";
         when Tol_Tighter_Than_Body => return "TolTighterThanBody(" & Nat_Img (N.Index) & ")";
         when No_Frame => return "NoFrame";
         when No_Hand_Contact => return "NoHandContact";
         when Many => return "Many(" & Img (N.M) & ")";
         when Cannot_Clear => return "CannotClear";
      end case;
   end Img;

end Contact.Exec;
