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
with Bodyfile;
with Bytes;
with Picture;
procedure Body_Driver is
   Port : Natural := 0;
   Body_Path : Unbounded_String;   --  身体文件(--in/--out;同一具身体越用越强)
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
            Body_Path := To_Unbounded_String (Ada.Command_Line.Argument (I + 1)); I := I + 1;   --  身体文件:装回、核对、合成、写回
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
   --  ── 量身体:先装回身体文件(钥匙 = 这具身体报的形状),推一下核对;对不上或没有 ⇒ 从零量;量到的合进历史再写回 ──
   declare
      Key : constant String := Bodyfile.Fingerprint (L, F);
      Stored : Selfmap.Body_Map;
      Stored_Hands : Zone.Hand_Vectors.Vector;
      Stored_Tables : Act.Effect_Vectors.Vector;
      Note : Unbounded_String;
      Loaded : Boolean := False;
      Use_Stored : Boolean := False;
   begin
      if Body_Path /= "" then
         Loaded := Bodyfile.Load (To_String (Body_Path), Key, Stored, Stored_Hands, Stored_Tables, Note);
         Put_Line ("[装] " & To_String (Note));
      end if;
      if Loaded then
         declare
            Ok_Body, Ok_Link : Boolean;
            Vn : Selfmap.String_Note;
         begin
            --  存的没有噪声地板图(那是当场的相机),先量一遍静止对再核
            Stored.Floors.Clear; Stored.Pic_Floor.Clear;
            declare
               Prev : constant Plug.Cam_Vectors.Vector := F.Cams;
               Ok2 : Boolean;
            begin
               Selfmap.Idle (L, F, 1, Ok2);
               for Cm in 0 .. Natural (F.Cams.Length) - 1 loop
                  Stored.Floors.Append (Picture.Null_Floor (Prev (Cm).Gray, F.Cams (Cm).Gray, F.Cams (Cm).W, F.Cams (Cm).H, Picture.Min_Pixels (F.Cams (Cm).W, F.Cams (Cm).H)));
                  Stored.Pic_Floor.Append (Picture.Max_Diff (Prev (Cm).Gray, F.Cams (Cm).Gray));
               end loop;
            end;
            Selfmap.Verify (L, Stored, F, Ok_Body, Ok_Link, Vn);
            Put_Line ("[装] 核对身体:" & To_String (Vn.Text) & (if Ok_Body then " ⇒ 同一具身体,直接用" else " ⇒ 重量"));
            if not Ok_Link then
               Put_Line ("[装] 核对时线断了,退出");
               return;
            end if;
            Use_Stored := Ok_Body;
         end;
      end if;
      if Use_Stored then
         C.Map := Stored;
         C.Tables := Stored_Tables;
      else
         Selfmap.Measure (L, F, C.Map, Ok);
         if not Ok then
            Put_Line ("[身] 身体量不了,退出");
            return;
         end if;
         if Loaded then
            declare
               Merged : Selfmap.Body_Map;
               Replaced, Kept : Natural;
            begin
               Bodyfile.Merge (Stored, C.Map, Merged, Replaced, Kept);
               C.Map := Merged;
               Put_Line ("[装] 越用越强:和存的合成 ⇒ 换掉 " & Codec.Img (Replaced) & " 格,持平 " & Codec.Img (Kept) & " 格(历次取中位数,噪声地板只放大)");
            end;
         else
            C.Map.Measured_Times := 1;
            for Ch in 0 .. C.Map.Channels - 1 loop
               declare
                  Ha, Hd : Bytes.Floats;
               begin
                  Ha.Append (C.Map.Amp (Ch)); Hd.Append (C.Map.Delivered (Ch));
                  C.Map.Amp_Hist.Append (Ha); C.Map.Deliv_Hist.Append (Hd);
               end;
            end loop;
         end if;
      end if;
      --  握区:合空每次开机都做(它同时量空合读数,也把世界相机里此刻的握区位置量出来);手上相机里的握区和存的比一眼
      for A in 0 .. C.Map.Arms - 1 loop
         declare
            H : Zone.Hand;
         begin
            Zone.Measure (L, C.Map, A, F, H, Ok);
            if not Ok then
               Put_Line ("[身] 第" & Natural'Image (A + 1) & " 只手的握区量不了");
            end if;
            if Use_Stored and then A < Natural (Stored_Hands.Length) then
               declare
                  Hc : constant Integer := (if A < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (A) else -1);
               begin
                  if Hc >= 0 and then Natural (Hc) < Natural (H.Zones.Length) and then Natural (Hc) < Natural (Stored_Hands (A).Zones.Length)
                    and then H.Zones (Natural (Hc)).Valid and then Stored_Hands (A).Zones (Natural (Hc)).Valid
                  then
                     declare
                        Zn : constant Zone.Hand_Zone := H.Zones (Natural (Hc));
                        Zo : constant Zone.Hand_Zone := Stored_Hands (A).Zones (Natural (Hc));
                     begin
                        Put_Line ("[装]   第" & Natural'Image (A + 1) & " 只手上相机里的握区:这次 (" & Codec.Fmt (Zn.Cu, 3) & "," & Codec.Fmt (Zn.Cv, 3) & ") 深 " & Codec.Fmt (Zn.Depth, 3) &
                                  " · 存的 (" & Codec.Fmt (Zo.Cu, 3) & "," & Codec.Fmt (Zo.Cv, 3) & ") 深 " & Codec.Fmt (Zo.Depth, 3));
                     end;
                  end if;
               end;
            end if;
            C.Hands.Append (H);
         end;
      end loop;
      if Body_Path /= "" then
         Bodyfile.Save (To_String (Body_Path), Key, C.Map, C.Hands, C.Tables);
         Put_Line ("[装] 身体写进 " & To_String (Body_Path) & "(量过 " & Codec.Img (C.Map.Measured_Times) & " 次)");
      end if;
   end;
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
         if Body_Path /= "" then
            Bodyfile.Save (To_String (Body_Path), Bodyfile.Fingerprint (L, F), C.Map, C.Hands, C.Tables);
         end if;
      end if;
   end loop;
end Body_Driver;
