--  body_driver --listen <口> [--eye host:port]:装上之后跑的唯一一条命令。
--  开机:认布局 → 量身体(逐通道推、合空)→ 循环:看 → 列块 → 问脑 → 执行 → 报。不读不写任何标定文件。
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Command_Line;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Strings.Fixed;
with Codec;
with Plug;
with Selfmap;
with Zone;
with World;
with Memory;
with Act;
procedure Body_Driver is
   Port : Natural := 0;
   --  脑的默认端点(接线协议,不是身体量)
   Eye : Unbounded_String := To_Unbounded_String ("127.0.0.1:8079");
   L : Plug.Link;
   F : Plug.Frame;
   C : Act.Context;
   Ok : Boolean;
   I : Natural := 1;
   Order : constant String := Codec.Env ("BL_ORDER");
begin
   while I <= Ada.Command_Line.Argument_Count loop
      declare
         A : constant String := Ada.Command_Line.Argument (I);
      begin
         if A = "--listen" and then I < Ada.Command_Line.Argument_Count then
            Port := Natural'Value (Ada.Command_Line.Argument (I + 1)); I := I + 1;
         elsif A = "--eye" and then I < Ada.Command_Line.Argument_Count then
            Eye := To_Unbounded_String (Ada.Command_Line.Argument (I + 1)); I := I + 1;
         elsif (A = "--in" or else A = "--out") and then I < Ada.Command_Line.Argument_Count then
            I := I + 1;    --  旧开关:不再读写标定文件,每炮从零量
         else
            Put_Line ("不认识的开关:" & A);
         end if;
      end;
      I := I + 1;
   end loop;
   if Codec.Env ("BL_EYE") /= "" then
      Eye := To_Unbounded_String (Codec.Env ("BL_EYE"));
   end if;
   if Port = 0 then
      Put_Line ("用法:body_driver --listen <端口> [--eye host:port]");
      Ada.Command_Line.Set_Exit_Status (2);
      return;
   end if;
   declare
      E : constant String := To_String (Eye);
      P : constant Natural := Ada.Strings.Fixed.Index (E, ":");
   begin
      if P > 0 then
         C.Eye_Host := To_Unbounded_String (E (E'First .. P - 1));
         C.Eye_Port := Natural'Value (E (P + 1 .. E'Last));
      else
         C.Eye_Host := Eye;
      end if;
   end;
   C.Look_Only := Codec.Env ("BL_LOOK") /= "";
   C.Dump_Dir := To_Unbounded_String (Codec.Env ("BL_DUMP"));
   if C.Dump_Dir /= "" then
      Codec.Make_Dir (To_String (C.Dump_Dir));
   end if;
   Plug.Boot (Port, L, Ok);
   if not Ok then
      Put_Line ("[装] 没接上,退出");
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;
   if not Plug.Sense (L, F) then
      Put_Line ("[装] 取不到第一帧,退出");
      return;
   end if;
   Put_Line ("[装] 第一帧:" & Natural'Image (Natural (F.EE.Length)) & " 条臂 ·" & Natural'Image (Natural (F.Jaw.Length)) & " 个抓握通道 ·" &
             Natural'Image (Natural (F.Cams.Length)) & " 台相机" & (if F.Cams.Is_Empty then "" else "(" & Codec.Img (F.Cams (0).W) & "x" & Codec.Img (F.Cams (0).H) & (if F.Cams (0).Has_Depth then ",带深度" else ",无深度") & ")"));
   --  ── 量身体 ──
   Selfmap.Measure (L, F, C.Map, Ok);
   if not Ok then
      Put_Line ("[身] 身体量不了,退出");
      return;
   end if;
   for A in 0 .. C.Map.Arms - 1 loop
      declare
         H : Zone.Hand;
      begin
         Zone.Measure (L, C.Map, A, F, H, Ok);
         C.Hands.Append (H);
         if not Ok then
            Put_Line ("[身] 第" & Natural'Image (A + 1) & " 只手的握区量不了");
         end if;
      end;
   end loop;
   Act.Init_Tracks (C);
   World.Init (C.Wld, C.Map.N_Cams);
   for A in 0 .. C.Map.Arms - 1 loop
      Put_Line ("[身] 第" & Natural'Image (A + 1) & " 只手上的相机:" & Integer'Image (C.Map.Cam_On_Arm (A)) & " · 各相机变化比例:" &
                Codec.Fmt (C.Map.Cam_Frac (A * C.Map.N_Cams), 3) & " " & (if C.Map.N_Cams > 1 then Codec.Fmt (C.Map.Cam_Frac (A * C.Map.N_Cams + 1), 3) else "") & " " &
                (if C.Map.N_Cams > 2 then Codec.Fmt (C.Map.Cam_Frac (A * C.Map.N_Cams + 2), 3) else ""));
   end loop;
   C.Cam := C.Map.World_Cam;
   Put_Line ("[身] 身体量完 ⇒ 开始干活(脑在 " & To_String (C.Eye_Host) & ":" & Codec.Img (C.Eye_Port) & (if C.Look_Only then ",只看不动" else "") & ")");
   --  ── 干活循环 ──
   loop
      if not Plug.Sense (L, F) then
         Put_Line ("[链] 取不到画面 ⇒ 退出");
         exit;
      end if;
      if Plug.Take_Reset (L) then
         Put_Line ("[身] 对方复位(新的一集)⇒ 世界记忆清空,身体留着");
         World.Reset_All (C.Wld);
         Memory.Clear (C.Mem);
         C.Recent := Null_Unbounded_String;
         C.Cam := C.Map.World_Cam;
         Act.Init_Tracks (C);
      end if;
      if Order /= "" then
         C.Task_Text := To_Unbounded_String (Order);
      elsif F.Instruction /= "" then
         C.Task_Text := F.Instruction;
      end if;
      if C.Task_Text = "" then
         C.Task_Text := To_Unbounded_String ("(no instruction was given)");
      end if;
      Memory.Set (C.Mem, "task", To_String (C.Task_Text));
      if F.Cams.Is_Empty then
         Put_Line ("[身] 这一帧没有相机画面");
      else
         Act.Round (L, F, C);
      end if;
   end loop;
end Body_Driver;
