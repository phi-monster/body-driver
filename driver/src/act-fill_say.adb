separate (Act)
procedure Fill_Say (C : in out Context; I : Sinew.Instr; Answer : out Brain.Say) is
   use Sinew;
   function Old_Rel (R : Rel) return String is (Rel_Cmd (R));   --  唯一那张表,不许在这儿再抄一份
   function Old_Until (O : Outcome) return String is (Until_Word (O));   --  唯一那张表,不许在这儿再抄一份
   function Old_Step (Sp : Step) return String is
     (case Sp is when Sp_Small => "small", when Sp_Medium => "medium",
         when Sp_Large => "large", when Sp_None => "medium");
   function Item_Of (N : Noun) return Natural is
      A : constant Integer := Plan.Look_Up (C.Binds, N);
   begin
      return (if A > 0 then Natural (A) else 0);
   end Item_Of;
   function Place_Of (N : Noun; Pl : out Place) return Boolean is
   begin
      Pl := (others => <>);
      if N.K /= Nk_Thing then
         return False;
      end if;
      for K in 0 .. Natural (C.Places.Length) - 1 loop
         if C.Places (K).Name = N.Word then
            Pl := C.Places (K);
            return True;
         end if;
      end loop;
      return False;
   end Place_Of;
begin
   Answer := (others => <>);
   Answer.See := To_Unbounded_String ("target");
   Answer.Fast := True;
   Answer.Until_Kind := To_Unbounded_String (Old_Until (I.Until_Oc));
   Answer.Steps := (if I.Max_Steps > 0 then I.Max_Steps else 0);
   for K in 0 .. Natural (I.Cons.Length) - 1 loop
      declare
         Cn : constant Constraint := I.Cons (K);
         Sub : constant Natural := Item_Of (Cn.Subj);
         Obj : constant Natural := Item_Of (Cn.Obj);
      begin
         case Cn.R is
            when Re_Qty =>
               --  语言的根:某件东西的某个量往哪变。手由身体选(grasper 那一绑);没拿着它 ⇒ 先合在它上(合的定义含从上方进场),再抬
               declare
                  Gn : constant Noun := (K => Nk_Role, R => Rl_Grasper, Word => Null_Unbounded_String);
                  Gi : constant Integer := Plan.Look_Up (C.Binds, Gn);
                  Bound_Arm : constant Natural := (if Gi >= 1 and then Gi <= Integer (C.Items.Length) then C.Items (Natural (Gi) - 1).Arm + 1 else 1);
                  --  哪只手去:离它近的那只(PLAN 第 2 步)。它在哪只腕眼里被点了名就是那条臂;在不动的眼里就比"它在画面里的位置"和"两只手在那只眼里各在哪"(握区量过的)
                  --  (H50/H56 2026-09-23 实测:右臂横跨整桌去够,关节到头,三把都合空)
                  --  手里已经拿着它 ⇒ 改它的量的就是拿着它的那只手,不再按远近选(H58 2026-09-23 实测:右手举着剪刀,头顶眼里它离左手近,左手去量眼、没动,还报"不在我手里")
                  Near_Arm : constant Integer := (if C.Wld.Holding and then C.Wld.Held_Arm >= 0 then C.Wld.Held_Arm else Nearer_Arm (C, Sub));
                  Arm1 : constant Natural := (if Near_Arm >= 0 then Natural (Near_Arm) + 1 else Bound_Arm);
                  Jk : Natural := 0;
               begin
                  for It of C.Items loop
                     if It.Kind = Grip and then It.Arm + 1 = Arm1 then
                        Jk := It.Jaw_K;
                     end if;
                  end loop;
                  if Near_Arm >= 0 and then Arm1 /= Bound_Arm then
                     Put_Line ("[身] ✋ 离它近的是第" & Codec.Img (Arm1) & " 只手(不是绑到的第" & Codec.Img (Bound_Arm) & " 只)⇒ 用它");
                  end if;
                  Answer.Qty := Cn.Obj.Word; Answer.Qty_Dir := Cn.Dir; Answer.Qty_Of := Sub;
                  Answer.Grip_Arm := Arm1;
                  Answer.Grip_K := Jk;
                  if not (C.Wld.Holding and then C.Wld.Held_Arm = Integer (Arm1) - 1) then
                     Answer.Grip := To_Unbounded_String ("close");
                     Answer.Grip_On := Sub;
                  end if;
               end;
            when Re_Close =>
               Answer.Grip := To_Unbounded_String ("close");
               Answer.Grip_Arm := (if Sub >= 1 and then Sub <= Natural (C.Items.Length)
                                   then C.Items (Sub - 1).Arm + 1 else 1);
               Answer.Grip_K := (if Sub >= 1 and then Sub <= Natural (C.Items.Length)
                                 then C.Items (Sub - 1).Jaw_K else 0);
               Answer.Grip_On := Obj;
            when Re_Open =>
               Answer.Grip := To_Unbounded_String ("open");
               Answer.Grip_Arm := (if Sub >= 1 and then Sub <= Natural (C.Items.Length)
                                   then C.Items (Sub - 1).Arm + 1 else 1);
               Answer.Grip_K := (if Sub >= 1 and then Sub <= Natural (C.Items.Length)
                                 then C.Items (Sub - 1).Jaw_K else 0);
            when Re_Clear =>
               Answer.Avoid.Append (Integer (Obj));
            when Re_Still =>
               Answer.Moves.Append (Brain.Goal'(Item => Sub, Cell => 0, Rel => Null_Unbounded_String,
                                                Of_Item => 0, Amount => Null_Unbounded_String,
                                                Stay => True, Hard => True,
                                                Has_Place => False, Pu => 0.0, Pv => 0.0, Pz => 0.0));
            when others =>
               declare
                  Pl : Place;
                  Is_Place : constant Boolean := Place_Of (Cn.Obj, Pl);
               begin
               Answer.Moves.Append
                 (Brain.Goal'(Item => Sub, Cell => 0,
                              Rel => To_Unbounded_String (Old_Rel (Cn.R)),
                              Of_Item => Obj,
                              Amount => To_Unbounded_String
                                (if Cn.R = Re_Press
                                 then (case Cn.Ef is
                                          when Ef_Light => "small", when Ef_Firm => "medium",
                                          when Ef_Hard => "large", when Ef_None => "small")
                                 else Old_Step (Cn.Sp)),
                              Stay => False, Hard => Cn.Rk = Rk_Must,
                              Has_Place => Is_Place, Pu => Pl.Cu, Pv => Pl.Cv, Pz => Pl.Z));
               end;
         end case;
      end;
   end loop;
   --  🔴 anyway:身体的一切认知性谨慎全部作废 —— 瞎着也走、离得远也合、顶着也推。
   C.Reckless := I.Anyway;
   C.Eye_Want := I.Eye;
end Fill_Say;
