separate (Act)
procedure Held_Test (L : in out Plug.Link; C : in out Context; F : in out Plug.Frame; Arm : Natural; Cam : Natural; Origin : Picture.Region;
                     Slot : Integer;
                     Obj_Count : Natural; Held : out Boolean; Sure : out Boolean; Note : out Unbounded_String) is
   A : Table.Vec := Table.Zero_Vec;
   Deliv : Table.Vec;
   Ok : Boolean;
   Grip_Says_Held : Boolean := False;   --  手指停在空手值之上 ⇒ 中间有东西(和画面完全独立的一条证据)
   Grip_Note : Unbounded_String;
   Hc : constant Integer := (if Arm < Natural (C.Map.Cam_On_Arm.Length) then C.Map.Cam_On_Arm (Arm) else -1);
   Jaw : Floats;
   Seen_In_Hand : Boolean := False;
   Home : Picture.Region := Origin;
   Gone_From_Table : Boolean := False;
   Could_Judge : Boolean := False;
   --  合完之后要量出"到底发生了什么",不是只答"拿住了没":旁边有没有东西被我碰动、那一块是不是断成了两块
   --  🔴 判"到底夹住没有"要的是【一台不跟着这只手动的相机】—— 以前只在【当前这只眼睛】里找,
   --  而当前这只正好长在手上 ⇒ 找不到 ⇒ "我不确定" ⇒ 把到手的东西又松开(GI 实测:
   --  笼住三项全过、合到底了,就因为没人核实而松手)。外面那台一直在那儿,去用它。
   World_Cam : constant Integer :=
     (if Cam < Natural (F.Cams.Length) and then Cam_Arm (C, Cam) < 0 then Integer (Cam)
      else Still_Cam (C, F, Arm));
   Before_Regs : Picture.Regions;
   Moved_Others : Natural := 0;
   Pieces_Now : Natural := 0;
   --  抬之前我的手在【那台不跟着我动的相机】里的哪儿(拿住的唯一硬证据是"它跟着我的手走了同样一段")
   Hand_U0, Hand_V0 : Long_Float := 0.0;
   Have_Hand0 : Boolean := False;
   Follows : Boolean := False;
   Found_After : Boolean := False;
   Follow_Note : Unbounded_String;
begin
   if World_Cam >= 0 and then Track_Idx (C, Arm, Natural (World_Cam)) < Natural (C.Zones.Length) then
      declare
         Tr : constant Zone_Track := C.Zones (Track_Idx (C, Arm, Natural (World_Cam)));
      begin
         Hand_U0 := Tr.Cu; Hand_V0 := Tr.Cv;
         Have_Hand0 := Tr.Valid and then not Tr.Blew_Up;
      end;
   end if;
   --  它原来在【那台相机】里的哪儿:用那台相机自己记着的影子,不能拿当前这只眼睛里的位置去比
   if World_Cam >= 0 and then Slot >= 0
     and then Natural (World_Cam) /= Cam
     and then Natural (Slot) < World.Count (C.Wld, Natural (World_Cam))
   then
      Home := World.Get (C.Wld, Natural (World_Cam), Natural (Slot)).Shadow;
   end if;
   if World_Cam >= 0 then
      Before_Regs := Cut_Things (C, F, Natural (World_Cam));
   end if;
   Jaw := Selfmap.Jaw_All (F, Arm);
   A (2) := C.Map.Amp (Arm * Chan.Per_Arm + 2) * 4.0;   --  抬起 = 看得见的探针幅度的几倍(倍数,无量纲),不假设哪根轴朝上:2 号轴是身体报的第三个平移通道
   Step_Arm (L, C, F, Arm, A, Jaw, Deliv, Ok);
   if Hc >= 0 and then Natural (Hc) < Natural (F.Cams.Length) then
      declare
         Z : constant Zone.Hand_Zone := Zone_Of (C, Arm, Natural (Hc));
         Regs : constant Picture.Regions := Cut_Things (C, F, Natural (Hc));
         Cw : constant Natural := F.Cams (Natural (Hc)).W;
         Ch : constant Natural := F.Cams (Natural (Hc)).H;
      begin
         if Z.Valid then
            Could_Judge := True;
            for R of Regs loop
               if R.Count * 3 >= Obj_Count and then R.Cu * Long_Float (Cw) >= Long_Float (Z.X0) and then R.Cu * Long_Float (Cw) <= Long_Float (Z.X1)
                 and then R.Cv * Long_Float (Ch) >= Long_Float (Z.Y0) and then R.Cv * Long_Float (Ch) <= Long_Float (Z.Y1)
               then
                  Seen_In_Hand := True;
               end if;
            end loop;
         end if;
      end;
   end if;
   if Cam < Natural (F.Cams.Length) and then Cam_Arm (C, Cam) < 0 and then Origin.Count > 0 then
      Could_Judge := True;
      Gone_From_Table := World.Vanished (Cut_Things (C, F, Cam), Origin, F.Cams (Cam).W, F.Cams (Cam).H);
   end if;
   --  旁边动了几件 · 原地现在剩几块(断成两块的话会多出一块)
   if World_Cam >= 0 then
      declare
         After : constant Picture.Regions := Cut_Things (C, F, Natural (World_Cam));
         Tol : constant Long_Float := 1.0 / Long_Float (F.Cams (Natural (World_Cam)).W);   --  一个像素(画幅比例,无量纲)
      begin
         for Q of Before_Regs loop
            declare
               Best : Long_Float := 0.0;
               Found : Boolean := False;
            begin
               for R of After loop
                  if Q.Count * 3 >= R.Count and then R.Count * 3 >= Q.Count then
                     declare
                        D : constant Long_Float := Sqrt ((R.Cu - Q.Cu) ** 2 + (R.Cv - Q.Cv) ** 2);
                     begin
                        if not Found or else D < Best then
                           Best := D; Found := True;
                        end if;
                     end;
                  end if;
               end loop;
               --  不是我夹的那件,却挪过了噪声地板 ⇒ 我碰动了它
               if Found and then Best > Tol * 4.0 and then Home.Count > 0
                 and then Sqrt ((Q.Cu - Home.Cu) ** 2 + (Q.Cv - Home.Cv) ** 2) > Long_Float'Max (Home.Sig_U, Home.Sig_V) * 2.0
               then
                  Moved_Others := Moved_Others + 1;
               end if;
            end;
         end loop;
         for R of After loop
            if Home.Count > 0 and then Sqrt ((R.Cu - Home.Cu) ** 2 + (R.Cv - Home.Cv) ** 2) <= Long_Float'Max (Home.Sig_U, Home.Sig_V) * 3.0 then
               Pieces_Now := Pieces_Now + 1;
            end if;
         end loop;
      end;
   end if;
   --  抬完之后:我的手挪了多远、它挪了多远,两段差多少
   --  🔴 判不了也要说【缺哪一样】。HC 实测这一整段没进来,而给脑的话只有"我判不出来"五个字,
   --  我是靠 grep 括号才发现判据根本没跑 —— 不说缺什么 = 让脑以为判据跑过了。
   if World_Cam < 0 then
      Follow_Note := S (" (I have no camera that stays put while this arm moves, so nothing could watch it travel)");
   elsif not Have_Hand0 then
      Follow_Note := S (" (I could not pin down where my own hand was in that still camera before the lift"
                        & ", so I had nothing to compare the thing's travel against)");
   elsif Home.Count <= 0 then
      Follow_Note := S (" (I had no picture of where the thing was sitting before the lift)");
   end if;
   if World_Cam >= 0 and then Have_Hand0 and then Home.Count > 0 then
      declare
         After : constant Picture.Regions := Cut_Things (C, F, Natural (World_Cam));
         Best : Integer := -1;
         Bd : Long_Float := 0.0;
         Hand_Du, Hand_Dv : Long_Float := 0.0;
      begin
         Feel (C, F);
         if Track_Idx (C, Arm, Natural (World_Cam)) < Natural (C.Zones.Length) then
            declare
               Tr : constant Zone_Track := C.Zones (Track_Idx (C, Arm, Natural (World_Cam)));
            begin
               if Tr.Valid and then not Tr.Blew_Up then
                  Hand_Du := Tr.Cu - Hand_U0; Hand_Dv := Tr.Cv - Hand_V0;
               else
                  Have_Hand0 := False;   --  这一抬我把自己的手跟丢了 ⇒ 判不了,老实说
               end if;
            end;
         end if;
         --  抬完之后最像它的那一块:大小相近的里面离原处最近的
         for I in 0 .. Natural (After.Length) - 1 loop
            if After (I).Count * 3 >= Home.Count and then Home.Count * 3 >= After (I).Count then
               declare
                  D : constant Long_Float := Sqrt ((After (I).Cu - Home.Cu) ** 2 + (After (I).Cv - Home.Cv) ** 2);
               begin
                  if Best < 0 or else D < Bd then
                     Bd := D; Best := I;
                  end if;
               end;
            end if;
         end loop;
         Found_After := Best >= 0;
         if Found_After and then Have_Hand0 then
            declare
               Ou : constant Long_Float := After (Best).Cu - Home.Cu;
               Ov : constant Long_Float := After (Best).Cv - Home.Cv;
            begin
               Follows := Came_With_Me (Ou, Ov, Hand_Du, Hand_Dv);
               Follow_Note := S (" (my hand moved " & Codec.Fmt (Sqrt (Hand_Du ** 2 + Hand_Dv ** 2), 3) &
                                 " of a frame, it moved " & Codec.Fmt (Sqrt (Ou ** 2 + Ov ** 2), 3) &
                                 ", the two differ by " & Codec.Fmt (Sqrt ((Ou - Hand_Du) ** 2 + (Ov - Hand_Dv) ** 2), 3) & ")");
            end;
         elsif not Found_After then
            Follow_Note := S (" (after the lift I could not find it anywhere in the camera that does not move with me)");
         else
            Follow_Note := S (" (I lost track of my own hand during the lift, so I cannot tell)");
         end if;
      end;
   end if;
   --  🔴 "拿住了"唯一分得开的硬证据:抬手时它【跟着我的手走了同样一段】。
   --  "它原来待的地方空了"分不开【撞跑】—— 球被撞到画面角落,原地照样空了,身体照样报"拿住"(FO 实测)。
   --  手上相机里"还在握区框里"更不算数 —— 那个框在手上相机里几乎是半个屏幕(FM 实测)。
   --  判不了就老实说"我说不准",不许自称拿住。
   --  🔴🔴 上面那三条全是【画面】信号。还有第四条,和画面完全独立:**手指停在哪儿**。
   --  爪子合在空气上会停在一个固定读数(开机量的空手值);中间夹着东西就停得更早。
   --  这一条撞跑伪造不了 —— 球被撞飞,手指照样合到空手值。
   --  ⇒ 两条正面证据【谁也不许否决谁】:手指卡住 = 拿住;跟着手走 = 拿住;两条都没有才叫没拿住。
   --  代价照记:反过来(拿画面一票否决)已经被实测判死 —— 纸杯蛋糕抬完 45 mm 读数远在空手值之上,
   --  只因画面里它变了样就被判滑掉、随即张手扔了。
   declare
      Jk : constant Natural := Natural (Integer'Max (0, C.Wld.Held_Jaw));
      --  空手值本来就量过、也存在身体文件里(Zone.Hand.Empty_Close,开机合空那一下的读数);
      --  这里只是【第一次把它拿来判拿住】,不新量一个。
      Hi : Integer := -1;
      Emp : Long_Float := -1.0;
      Have_R : constant Boolean := Selfmap.Has_Jaw (F, Arm, Jk);
      R_Now : constant Long_Float := (if Have_R then Selfmap.Jaw_Of (F, Arm, Jk) else 0.0);   --  没读数时不用它(下面都先问 Have_R)
   begin
      for I in 0 .. Natural (C.Hands.Length) - 1 loop
         if C.Hands (I).Arm = Arm and then C.Hands (I).K = Jk and then C.Hands (I).Measured then
            Hi := Integer (I);
         end if;
      end loop;
      if Hi >= 0 then
         Emp := C.Hands (Natural (Hi)).Empty_Close;
      end if;
      --  量得出空手值、这一拍也有读数,才谈得上问手指;门槛是读数自己的抖动(量出来的),不是我拍的容差。
      Grip_Says_Held := Hi >= 0 and then Have_R and then Past_Empty (C.Hands (Natural (Hi)), R_Now) > C.Map.Jaw_Noise;
      if not Have_R then
         Grip_Note := S (" (my fingers report no reading this beat, so I cannot ask them)");
      elsif Hi >= 0 then
         Grip_Note := S (" (my fingers stopped at " & Codec.Fmt (R_Now, 3)
                         & ", empty they stop at " & Codec.Fmt (Emp, 3)
                         & (if Grip_Says_Held then " - so something is wedged between them" else " - so there is nothing between them") & ")");
      else
         Grip_Note := S (" (I have never measured where my fingers stop on empty air, so I cannot ask them)");
      end if;
   end;
   Held := Grip_Says_Held
           or else (if World_Cam >= 0 and then Have_Hand0 and then Found_After then Follows else Seen_In_Hand);
   Sure := Grip_Says_Held or else (World_Cam >= 0 and then Have_Hand0 and then Found_After);
   if Grip_Says_Held and then not (World_Cam >= 0 and then Have_Hand0 and then Found_After and then Follows) then
      --  手指说有、画面说不出 ⇒ 以手指为准,并且把两边都说出来(不许只报结论)
      Note := S ("after a small lift my fingers are still held apart ⇒ held") & Grip_Note & Follow_Note;
   elsif Sure and then Follows then
      Note := S ("after a small lift it came with my hand ⇒ held") & Grip_Note & Follow_Note
              & (if Seen_In_Hand then ", and my hand camera still shows it between my fingers" else "");
   elsif Sure then
      Note := S ("after a small lift it did NOT come with my hand ⇒ not held") & Grip_Note
              & (if Gone_From_Table then S (" - and its old place is empty, so I knocked it away rather than picked it up") else S (""))
              & Follow_Note
              & (if Seen_In_Hand then " (my hand camera still shows something between my fingers, which proves nothing)" else "");
   elsif World_Cam >= 0 then
      Note := S ("after a small lift I could not judge whether it came with me") & Follow_Note;
   elsif Seen_In_Hand then
      Note := S ("after a small lift the thing is still inside my grip box in my hand camera; no still camera could check, so I am not sure");
   elsif Could_Judge then
      Note := S ("after a small lift the thing did not come with me ⇒ not held");
   else
      Note := S ("I could not judge whether it is held (no camera could see it)");
   end if;
   if World_Cam >= 0 then
      Append (Note, ". While I closed and lifted, " & Codec.Img (Moved_Others) & " other thing(s) I was not pushing moved");
      if Pieces_Now >= 2 then
         Append (Note, ", and where it stood there are now " & Codec.Img (Pieces_Now) & " separate pieces");
      end if;
   end if;
end Held_Test;
