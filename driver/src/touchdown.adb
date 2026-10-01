with Ada.Text_IO; use Ada.Text_IO;
with Bytes; use Bytes;
with Chan;
with Codec;
with Table;
with Lockstep;
with Selfmap.Graph;
package body Touchdown is
   --  同 Act.Geo_Say 的样子(几只手按拍对齐时带上哪只手说的)
   procedure Say (S : String) is
      H : constant Integer := Lockstep.Current_Hand;
   begin
      Put_Line ("[身] 📐 " & (if H >= 0 then "〔手" & Codec.Img (Natural (H) + 1) & "〕" else "") & S);
   end Say;

   function Mm (X : Long_Float) return String is (Codec.Fmt (X, 3) & " 单位");

   procedure Step_Down (L : in out Plug.Link; M : Selfmap.Body_Map; F : in out Plug.Frame; Arm : Natural; Into : Geom.V3; Lstep : Long_Float;
                        W : in out Selfmap.Walk; Short : out Long_Float; At_Limit, Hit : out Boolean; Rep : out Selfmap.Leg_Step) is
      Cur : constant Plug.Arm_Pose := F.EE (Arm);
      Seq0 : constant Natural := F.Seq;
      Chs : constant Ints := Selfmap.Graph.Pose_Channels (M, Arm);
      --  转动一步看得见的那一档(开机量的;这条臂没有转动通道 ⇒ 不按朝向判)
      Tol_R : constant Long_Float :=
        (if Natural (Chs.Length) > Chan.Pos_Channels and then Natural (Chs (Chan.Pos_Channels)) < Natural (M.Amp.Length)
         then M.Amp (Natural (Chs (Chan.Pos_Channels))) else 0.0);
      Av : Table.Vec := Table.Zero_Vec;
      Pe, Re : Long_Float;
      Rok, Mok : Boolean;
      Legs : Selfmap.Leg_Vectors.Vector;
      Lim : Selfmap.Limits;
      Rs : Selfmap.Leg_Step_Vectors.Vector;
      Frames : Natural;
   begin
      Short := 0.0; At_Limit := False; Hit := False; Rep := (others => <>);
      for I in 0 .. 2 loop
         Av (I) := Lstep * Into (I);
      end loop;
      Plug.Reach (Arm, Chan.Compose (Cur, Av), Pe, Re, Rok);
      if Rok and then (Pe + Pe > Lstep or else (Tol_R > 0.0 and then Re > Tol_R)) then
         At_Limit := True;
         return;
      end if;
      Legs.Append (Selfmap.Leg'(Arm => Arm, Goal => Chan.Compose (Cur, Av), Jaw => <>));
      Lim.Press := True; Lim.Loose := False;
      Selfmap.Step (L, M, Legs, Lim, F, W, Rs, Frames, Mok);
      if not Rs.Is_Empty then
         Rep := Rs (0);
      end if;
      Hit := Rep.Blocked_T;
      declare
         Now : constant Plug.Arm_Pose := F.EE (Arm);
      begin
         Short := Lstep - ((Now (0) - Cur (0)) * Into (0) + (Now (1) - Cur (1)) * Into (1) + (Now (2) - Cur (2)) * Into (2));
      end;
      --  每一步一行(同 Act.Geo_Move 那一行;拍号 = 这一步起止那两帧的帧号:离线按仿真真值给每一下压标"碰没碰到"用)
      Say ("挪 (" & Mm (Rep.Cmd (0)) & "," & Mm (Rep.Cmd (1)) & "," & Mm (Rep.Cmd (2)) & ") ⇒ 实到 (" & Mm (Rep.Got (0)) & "," & Mm (Rep.Got (1)) & ","
           & Mm (Rep.Got (2)) & "),差 " & Mm (Geom.Norm ([Rep.Cmd (0) - Rep.Got (0), Rep.Cmd (1) - Rep.Got (1), Rep.Cmd (2) - Rep.Got (2)]))
           & (if Mok then "" else " · 身体说没走成") & (if Hit then " · 被挡住(比这一段空走时少走得多)" else "")
           & " · 拍 " & Codec.Img (Seq0) & "→" & Codec.Img (F.Seq));
   end Step_Down;

   procedure Descend (L : in out Plug.Link; M : Selfmap.Body_Map; F : in out Plug.Frame; Arm : Natural; Into : Geom.V3; Lstep : Long_Float;
                      Steps : Natural; W : in out Selfmap.Walk; Got_It, At_Limit : out Boolean; From : out Plug.Arm_Pose; Used : out Natural;
                      Short : out Long_Float) is
      Hit : Boolean;
      Rep : Selfmap.Leg_Step;
   begin
      Got_It := False; At_Limit := False; Used := 0; Short := 0.0;
      From := F.EE (Arm);
      for I in 1 .. Steps loop
         From := F.EE (Arm);
         Step_Down (L, M, F, Arm, Into, Lstep, W, Short, At_Limit, Hit, Rep);
         exit when At_Limit;
         Used := I;
         if Hit then
            Got_It := True;
            return;
         end if;
      end loop;
   end Descend;

   function Free_Note (W : Selfmap.Walk; Arm : Natural) return String is
   begin
      for Lg of W.Legs loop
         if Lg.Arm = Arm then
            return "空走时平均少走 " & Codec.Fmt (100.0 * Lg.Tr.Mean, 1) & "%、散布 " & Codec.Fmt (100.0 * Selfmap.Free_Sd (Lg.Tr), 1) & "%(" & Codec.Img (Lg.Tr.N) & " 步)";
         end if;
      end loop;
      return "这一段还没有空走的底";
   end Free_Note;
end Touchdown;
