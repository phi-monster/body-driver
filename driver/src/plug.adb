with Ada.Text_IO; use Ada.Text_IO;
with Ada.Calendar;
with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Ada.Unchecked_Conversion;
with Interfaces; use Interfaces;
with Codec;
with Lockstep;
package body Plug is
   use Msgpack;
   function U32_To_F32 is new Ada.Unchecked_Conversion (Unsigned_32, Float);
   use type Websocket.Op;

   Hook_P : Pose_Hook := null;
   Hook_C : Cmd_Hook := null;
   procedure Set_Hooks (P : Pose_Hook; Q : Cmd_Hook) is
   begin
      Hook_P := P; Hook_C := Q;
   end Set_Hooks;
   Hook_L : Limit_Hook := null;
   procedure Set_Limit (H : Limit_Hook) is
   begin
      Hook_L := H;
   end Set_Limit;
   function Held_Back (Arm : Natural) return Limit_State is (if Hook_L = null then Free else Hook_L (Arm));
   Hook_R : Reach_Hook := null;
   procedure Set_Reach (R : Reach_Hook) is
   begin
      Hook_R := R;
   end Set_Reach;
   procedure Reach (Arm : Natural; Pose : Arm_Pose; Pos_Err, Rot_Err : out Long_Float; Ok : out Boolean) is
   begin
      Pos_Err := 0.0; Rot_Err := 0.0; Ok := Hook_R /= null;
      if Ok then
         Hook_R (Arm, Pose, Pos_Err, Rot_Err);
      end if;
   end Reach;

   --  按拍对齐时记下的目标(见 spec 的 Lock_Begin):每组关节读数一个目标(空 = 没给)、每只手的爪子一个目标(空 = 没给),
   --  这一拍有没有变;主线程收的那一帧(手的任务醒来拿它)
   Lock_Q : Floats_Vectors.Vector;
   Lock_Jaw : Floats_Vectors.Vector;
   Lock_Changed : Boolean := False;
   Lock_F : Frame;
   Lock_Ok : Boolean := True;

   function Arms (L : Link) return Natural is
   begin
      if L.Lay.Measured then
         return L.Lay.N_Arms;   --  开机按量认出来几条臂(I1,10-01;不再按"位姿键数 / 抓握键数"凑)
      end if;
      if not L.Lay.EE.Is_Empty then
         return Natural'Min (Natural (L.Lay.EE.Length), Natural'Max (1, Natural (L.Lay.Jaw.Length)));
      end if;
      return Natural (L.Lay.Joints.Length);
   end Arms;

   function Joint_Mode (L : Link) return Boolean is (L.Lay.EE.Is_Empty);

   function Steps (L : Link) return Natural is (if L.Seq >= L.Ep_Seq0 then L.Seq - L.Ep_Seq0 else L.Seq);

   function Beat_Index (L : Link; Seq : Natural) return Integer is
   begin
      if L.Beats.Is_Empty or else Seq < L.Beats.First_Element.Seq or else Seq > L.Beats.Last_Element.Seq then
         return -1;
      end if;
      return Seq - L.Beats.First_Element.Seq;   --  帧号连着(每收一帧记一拍)
   end Beat_Index;
   function Joints_At (L : Link; Seq : Natural) return Floats_Vectors.Vector is
      K : constant Integer := Beat_Index (L, Seq);
   begin
      return (if K >= 0 then L.Beats (K).Joints else Floats_Vectors.Empty_Vector);
   end Joints_At;
   function Reported_At (L : Link; Seq : Natural) return Pose_Vectors.Vector is
      K : constant Integer := Beat_Index (L, Seq);
   begin
      return (if K >= 0 then L.Beats (K).Reported_EE else Pose_Vectors.Empty_Vector);
   end Reported_At;
   function Image_Lag (L : Link; Cam, Group, From_Seq : Natural; Corr : out Floats) return Integer is separate;

   function Reset_Pending (L : Link) return Boolean is (L.Reset_Flag);

   function Take_Reset (L : in out Link) return Boolean is
      R : constant Boolean := L.Reset_Flag;
   begin
      L.Reset_Flag := False;
      return R;
   end Take_Reset;

   function Nums_At (L : Link; P : Layout.Path) return Floats is
   begin
      --  空路径 = 这一格没有读数(量出来的布局里某条臂没有合拢通道时占位用);不许当成整个观测去取数
      if L.Last_Obs < 0 or else P.Segs.Is_Empty then
         return F64_Vectors.Empty_Vector;
      end if;
      return Numbers (L.Last, Layout.Find (L.Last, L.Last_Obs, P));
   end Nums_At;

   --  一条动作里的一个键:名字 + 那一串数。先凑齐再数有几个键(读数空、这回不发的键不占 Put_Map 的个数)
   procedure Put_Keys (S : in out Buf; Keys : Strs; Vals : Floats_Vectors.Vector) is
   begin
      Put_Map (S, Natural (Keys.Length));
      for K in 0 .. Natural (Keys.Length) - 1 loop
         Put_Str (S, Keys (K));
         Put_Array (S, Natural (Vals (K).Length));
         for X of Vals (K) loop
            Put_Float (S, X);
         end loop;
      end loop;
   end Put_Keys;

   --  名字 Nm 这一回保持发哪一串:此刻的读数;这一拍没读数 ⇒ 上一回发出去的那一串(Note_Sent 记的);都没有 ⇒ 空(这个键这回不发,不编)
   function Hold_Value (L : Link; Nm : String; P : Layout.Path) return Floats is
      Cur : constant Floats := Nums_At (L, P);
   begin
      if not Cur.Is_Empty then
         return Cur;
      end if;
      for I in 0 .. Natural (L.Hold_Names.Length) - 1 loop
         if L.Hold_Names (I) = Nm then
            return L.Hold_Sent (I);
         end if;
      end loop;
      return F64_Vectors.Empty_Vector;
   end Hold_Value;

   --  这一条动作里每个键发出去的那一串记下(下一拍这个键没读数时照发它)
   procedure Note_Sent (L : in out Link; Keys : Strs; Vals : Floats_Vectors.Vector) is
   begin
      for K in 0 .. Natural (Keys.Length) - 1 loop
         declare
            Found : Boolean := False;
         begin
            for I in 0 .. Natural (L.Hold_Names.Length) - 1 loop
               if L.Hold_Names (I) = Keys (K) then
                  L.Hold_Sent.Replace_Element (I, Vals (K));
                  Found := True;
               end if;
            end loop;
            if not Found then
               L.Hold_Names.Append (Keys (K)); L.Hold_Sent.Append (Vals (K));
            end if;
         end;
      end loop;
   end Note_Sent;

   function Has_Name (V : Strs; X : String) return Boolean is
   begin
      for Y of V loop
         if Y = X then
            return True;
         end if;
      end loop;
      return False;
   end Has_Name;

   --  「照现在这样保持」:每一个命令键照此刻的读数原样回声(见 spec)
   function Hold_Action (L : in out Link) return Buf is
      S : Buf;
      Keys : Strs;
      Vals : Floats_Vectors.Vector;
   begin
      if L.Last_Obs < 0 then
         return S;
      end if;
      for I of Layout.Command_Groups (L.Lay) loop
         declare
            Nm : constant String := Layout.Last_Seg (L.Lay.Groups (I));
         begin
            if not Has_Name (Keys, Nm) then
               declare
                  V : constant Floats := Hold_Value (L, Nm, L.Lay.Groups (I));
               begin
                  if not V.Is_Empty then
                     Keys.Append (Nm); Vals.Append (V);
                  end if;
               end;
            end if;
         end;
      end loop;
      Put_Keys (S, Keys, Vals);
      Note_Sent (L, Keys, Vals);
      return S;
   end Hold_Action;

   procedure Reply (L : in out Link; Req : Doc; Kind : String; Payload : Buf) is separate;

   --  抽这条连接直到拿到一帧新观测。握手照回,要动作就交出攥着的那条。
   function Pump (L : in out Link) return Boolean is separate;

   procedure Boot (Port : Natural; L : in out Link; Ok : out Boolean) is separate;

   procedure Frame_Of (L : Link; F : in out Frame) is separate;

   procedure Note_Beat (L : in out Link; F : Frame) is separate;

   function Sense (L : in out Link; F : out Frame) return Boolean is separate;

   function Act_Raw (L : in out Link; C : Cmd) return Boolean;
   --  手的任务里的 Act:只记下这只手的目标(位姿命令先解成关节),这一拍由主线程合起来发
   function Lock_Act (C : Cmd) return Boolean is separate;
   function Act (L : in out Link; C : Cmd) return Boolean is
   begin
      if Lockstep.Current_Hand >= 0 then
         return Lock_Act (C);
      end if;
      if C.Kind = Ee and then Hook_C /= null then
         declare
            Cj : Cmd := C;
            Ok : Boolean;
         begin
            Hook_C (Cj, Ok);
            return Ok and then Act_Raw (L, Cj);
         end;
      end if;
      return Act_Raw (L, C);
   end Act;

   --  第 Ji 个抓握读数组这回发哪一串(见 spec):形状 = 这一拍的读数(没收到 ⇒ 上一回发出去的那一串);给了 ⇒ 给的,没给 ⇒ 给过的最后一个目标,
   --  一次没给过 ⇒ 此刻的读数,这一拍没读数 ⇒ 上一回发出去的那个数。一个都凑不出 ⇒ 空(这一组这回不发)
   function Jaw_Values (L : in out Link; Ji : Natural; Mine : Boolean; C : Cmd; Cur : Floats) return Floats is separate;

   function Act_Raw (L : in out Link; C : Cmd) return Boolean is separate;

   procedure Lock_Begin is
   begin
      Lock_Q.Clear; Lock_Jaw.Clear; Lock_Changed := False;
   end Lock_Begin;

   procedure Lock_Feed (F : Frame; Ok : Boolean := True) is
   begin
      Lock_F := F;
      Lock_Ok := Ok;
      Lock_Changed := False;
   end Lock_Feed;

   procedure Lock_End is
   begin
      Lock_Q.Clear; Lock_Jaw.Clear; Lock_Changed := False;
   end Lock_End;

   function Lock_Merged return Cmd is
      Cm : Cmd;
   begin
      Cm.Kind := Joint;
      for G in 0 .. Natural (Lock_Q.Length) - 1 loop
         if not Lock_Q (G).Is_Empty then
            Cm.Groups.Append (G);
            Cm.Qs.Append (Lock_Q (G));
         end if;
      end loop;
      return Cm;
   end Lock_Merged;

   --  主线程走一拍:这一拍有手发了新目标 ⇒ 几只手的关节目标合成一条动作(每只给过爪子目标的手各发一遍,把它的爪子目标登记进 Jaw_Set;
   --  最后那一遍的动作里关节是全部手的、爪子是各自最后一个目标),再收一帧,留给手的任务醒来拿
   procedure Lock_Beat (L : in out Link; F : in out Frame; Ok : out Boolean) is
   begin
      if Lock_Changed then
         declare
            Cm : Cmd := Lock_Merged;
            Any_Jaw : Boolean := False;
            Sent : Boolean;
         begin
            if not Cm.Groups.Is_Empty then
               for A in 0 .. Natural (Lock_Jaw.Length) - 1 loop
                  if not Lock_Jaw (A).Is_Empty then
                     Cm.Arm := A; Cm.Jaw := Lock_Jaw (A);
                     Sent := Act_Raw (L, Cm);
                     Any_Jaw := True;
                  end if;
               end loop;
               if not Any_Jaw then
                  Cm.Arm := 0; Cm.Jaw.Clear;
                  Sent := Act_Raw (L, Cm);
               end if;
               if not Sent then
                  Put_Line ("[链] 几只手合成的那条关节动作没发成(这一拍照旧重发上一条)");
               end if;
            end if;
            Lock_Changed := False;
         end;
      end if;
      Ok := Sense (L, F);
      Lock_F := F;
      Lock_Ok := Ok;
   end Lock_Beat;
end Plug;
