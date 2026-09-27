--  离线查不动的眼重标准不准(2026-09-27,V1:被转以后重标的那份按真值中位 1.2–1.4 px、开机那份 0.3 px):
--  拿一炮存的前半段(<身体文件>.kin.txt:板上的点 + 开机标的不动的眼,世界系),把开机那只眼绕自己的光轴转给定的角度当"真的",
--  板上的点按它投出"配点"(可加噪声;出了画面的配不到),原样跑驱动那一份 Geom.Check_Fixed(同驱动:位姿、门、放好以来最多),
--  看重标出来的位姿离"真的"多远 —— 分得清是解法 / 板的几何的毛病,还是在线配点的毛病。真值不进解。
--  用法:fixedexam <kin 文件> <转多少度> [配点噪声像素,默认 0] [转着看的配点噪声像素,默认 0]
with Ada.Command_Line; use Ada.Command_Line;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Numerics.Float_Random;
with Bytes; use Bytes;
with Codec;
with Geom; use Geom;
with Jointboot;
procedure Fixedexam is
   package FR renames Ada.Numerics.Float_Random;
   Gen : FR.Generator;
   function Gauss return Long_Float is
      A : constant Long_Float := Long_Float'Max (1.0e-12, Long_Float (FR.Random (Gen)));
      B : constant Long_Float := Long_Float (FR.Random (Gen));
   begin
      return Sqrt (-2.0 * Log (A)) * Cos (2.0 * Ada.Numerics.Pi * B);
   end Gauss;
   K : Jointboot.Kin_Store;
   Ok : Boolean;
   Note : Unbounded_String;
   Deg : constant := 0.0174532925199433;   --  1° 的弧度(换算,无量纲)
   Nx : constant := 32;   --  桌面格点:横着几个(次数)
   Ny : constant := 24;   --  竖着几个(次数)
   Roll : constant Long_Float := Long_Float'Value (Argument (2)) * Deg;
   Noise : constant Long_Float := (if Argument_Count >= 3 then Long_Float'Value (Argument (3)) else 0.0);
   Turn_Sd : constant Long_Float := (if Argument_Count >= 4 then Long_Float'Value (Argument (4)) else 0.0);
begin
   FR.Reset (Gen, 20260927);
   Jointboot.Load_Kin (Argument (1), K, Ok, Note);
   if not Ok or else not K.Fixed_Eye.Valid then
      Put_Line ("读不了 " & Argument (1) & ":" & To_String (Note));
      return;
   end if;
   declare
      G0 : constant Cam_Geo := K.Fixed_Eye;
      Gr : Cam_Geo := G0;   --  "真的":开机那只眼绕自己的光轴(相机系 z)转 Roll
      Now : Scene_Pt_Vectors.Vector;
      Best : Fixed_Best := Seen_All (G0, K.Board);
      G : Cam_Geo := G0;
      R : Fixed_Check;
      W : constant Long_Float := 2.0 * G0.Cx;
      H : constant Long_Float := 2.0 * G0.Cy;
      In_View : Natural := 0;
   begin
      Gr.R_Ce := Mul (G0.R_Ce, Rodrigues ([0.0, 0.0, Roll]));
      for P of K.Board loop
         declare
            N : Scene_Pt := P;
            U, V : Long_Float;
            Fr : Boolean;
         begin
            Project_Fixed (Gr, P.Pw, U, V, Fr);
            if Fr and then U >= 0.0 and then V >= 0.0 and then U < W and then V < H then
               N.U := U + Noise * Gauss; N.V := V + Noise * Gauss;
               In_View := In_View + 1;
            else
               N.U := -1.0; N.V := -1.0;
            end if;
            Now.Append (N);
         end;
      end loop;
      Geom.Check_Fixed (G, K.Board, Now, Best, R, Turn_Sd => Turn_Sd);
      --  重标出来的离"真的"多远:板上的点(转过的画面里看得见的)和桌面(世界 z = 0)上铺满画面的格点,各按两只眼投,差几像素
      declare
         function Px_Err (Pw : V3; Ok_Out : out Boolean) return Long_Float is
            U1, V1, U2, V2 : Long_Float;
            F1, F2 : Boolean;
         begin
            Project_Fixed (Gr, Pw, U1, V1, F1);
            Project_Fixed (G, Pw, U2, V2, F2);
            Ok_Out := F1 and then F2 and then U1 >= 0.0 and then V1 >= 0.0 and then U1 < W and then V1 < H;
            return Sqrt ((U1 - U2) ** 2 + (V1 - V2) ** 2);
         end Px_Err;
         Eb, Eg : Floats;
         package Sorting is new F64_Vectors.Generic_Sorting;
         function Q (E : Floats; F : Long_Float) return Long_Float is
            S : Floats := E;
         begin
            if S.Is_Empty then
               return -1.0;
            end if;
            Sorting.Sort (S);
            return S (Natural'Min (Natural (S.Length) - 1, Natural (F * Long_Float (Natural (S.Length) - 1))));
         end Q;
      begin
         for P of K.Board loop
            declare
               O : Boolean;
               E : constant Long_Float := Px_Err (P.Pw, O);
            begin
               if O then
                  Eb.Append (E);
               end if;
            end;
         end loop;
         for Iy in 0 .. Ny - 1 loop
            for Ix in 0 .. Nx - 1 loop
               declare
                  D : constant V3 := Ray_Fixed (Gr, (Long_Float (Ix) + 0.5) * W / Long_Float (Nx), (Long_Float (Iy) + 0.5) * H / Long_Float (Ny));
               begin
                  if D (2) < -1.0e-6 then   --  朝下的视线才落得到桌面(世界 z = 0,数值保护)
                     declare
                        T : constant Long_Float := -Gr.Pos (2) / D (2);
                        O : Boolean;
                        E : constant Long_Float := Px_Err ([Gr.Pos (0) + T * D (0), Gr.Pos (1) + T * D (1), 0.0], O);
                     begin
                        if O then
                           Eg.Append (E);
                        end if;
                     end;
                  end if;
               end;
            end loop;
         end loop;
         Put_Line ("转 " & Argument (2) & "° · 配点噪声 " & Codec.Fmt (Noise, 2) & " px · 板上 " & Codec.Img (Natural (K.Board.Length)) & " 个点、转过以后画面里 " & Codec.Img (In_View)
                   & " 个 ⇒ " & (if R.Moved then "认出挪过" else "没认出挪过") & (if R.Covered then "、报了挡" else "") & " · 重标:板上 " & Codec.Img (R.Consistent)
                   & " 个对得上、自报残差 " & Codec.Fmt (R.Rms, 3) & " px");
         Put_Line ("  离真的:转 " & Codec.Fmt (Norm (Rot_Vec (Mul (Tr (Gr.R_Ce), G.R_Ce))) / Deg, 4) & "°、挪 " & Codec.Fmt (Norm ([G.Pos (0) - Gr.Pos (0), G.Pos (1) - Gr.Pos (1), G.Pos (2) - Gr.Pos (2)]), 4)
                   & " 单位 · 板上的点按像素差 中位 " & Codec.Fmt (Q (Eb, 0.5), 3) & "、九成 " & Codec.Fmt (Q (Eb, 0.9), 3) & " px · 桌面上铺满画面的格点 中位 "
                   & Codec.Fmt (Q (Eg, 0.5), 3) & "、九成 " & Codec.Fmt (Q (Eg, 0.9), 3) & "、最大 " & Codec.Fmt (Q (Eg, 1.0), 3) & " px(" & Codec.Img (Natural (Eg.Length)) & " 个)");
      end;
   end;
end Fixedexam;
