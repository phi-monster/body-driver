separate (Act)
procedure Build_Listing (C : in out Context; F : Plug.Frame; Cam : Natural; RGB : in out Buf; Text : out Unbounded_String;
                         Keep : Boolean := False; Things_Only : Boolean := False) is
   Cw : constant Natural := F.Cams (Cam).W;
   Ch : constant Natural := F.Cams (Cam).H;
   T : Unbounded_String;
   Named_U, Named_V : Long_Float := -1.0;
   Have_Named : Boolean := False;
   --  语言的根(2026-09-23):有能合拢的手 ⇒ 脑的句子只说【东西的量往哪变】,我身上的零件它点不到、也不该看见
   --  (H45 实测:纸上列着 "a finger of arm 1 … grip 2",9B 的脑就把 "reach arm leftwards"、"grip above …" 填进东西的位置)。
   --  零件照旧进清单(绑 grasper 要用),只是不写给脑、不画框。
   --  "有能合拢的手"是量出来的:哪个抓握通道推到头时有相机看见手指来去(Has_Fingers);原来按抓握通道的个数(至少 1),
   --  什么都不带的抓握通道(无人机)也算 ⇒ 脑只能说"东西的量往哪变",可身体根本没有能拿起东西的零件(DR2 2026-09-28)
   Qmode : constant Boolean := Any_Fingers (C);
   function Rel (U, V : Long_Float) return String is
      Half : constant String := (if U < 0.5 then "LEFT" else "RIGHT");
   begin
      if Have_Named then
         declare
            D : constant Long_Float := Sqrt (((U - Named_U) * Long_Float (C.Cols)) ** 2 + ((V - Named_V) * Long_Float (C.Rows)) ** 2);
         begin
            return ", " & Codec.Fmt (D, 1) & " cells from the thing you last named, in the " & Half & " half of the picture";
         end;
      end if;
      return ", in the " & Half & " half of the picture";
   end Rel;
   --  这个框此刻跟我身上哪一块的框重叠吗(身体零件在世界那一段之前就已经列进 C.Items 了)
   function Under_Me (X0, Y0, X1, Y1 : Natural) return Boolean is
   begin
      for I in 0 .. Natural (C.Items.Length) - 1 loop
         declare
            B : constant Item := C.Items (I);
         begin
            if B.Kind in Finger | Grip | Piece and then B.Located
              and then B.X1 >= X0 and then X1 >= B.X0
              and then B.Y1 >= Y0 and then Y1 >= B.Y0
            then
               return True;
            end if;
         end;
      end loop;
      return False;
   end Under_Me;

   procedure Push (It_In : Item; Line : String; Col : Draw.Color; Thick : Natural) is
      It : Item := It_In;
   begin
      It.Cam := Cam;   --  这一块是在哪台相机里看见的:清单跨相机之后,这一位是它唯一的落脚点
      C.Items.Append (It);
      if Qmode and then It.Kind in Finger | Grip | Piece then
         return;   --  零件进清单不进纸(见 Qmode)
      end if;
      if It.Located then
         Draw.Numbered_Box (RGB, Cw, Ch, It.X0, It.Y0, It.X1, It.Y1, Natural (C.Items.Length), Col, Thick);
      end if;
      --  行首只放方括号里的号(对得上画面上那个框就够了),不再写 "item":T2 实测 Qwen 把这个记账词当成了东西的名字
      Append (T, "  [" & Codec.Img (Natural (C.Items.Length)) & "] " & Line & ASCII.LF);
   end Push;
begin
   if not Keep then
      C.Items.Clear;
   end if;
   if C.Wld.Cams (Cam).Named >= 0 then
      declare
         Sl : constant World.Slot := World.Get (C.Wld, Cam, Natural (C.Wld.Cams (Cam).Named));
      begin
         if Sl.Seen then
            Have_Named := True;
            Named_U := (if Sl.Present then Sl.R.Cu else Sl.Shadow.Cu);
            Named_V := (if Sl.Present then Sl.R.Cv else Sl.Shadow.Cv);
         end if;
      end;
   end if;
   --  🔴🔴 自述那一条通道(owner 2026-09-15 定;架构 2026-08-18 已为它留位):
   --  命令那条通道一个字不提身体 ⇒ 换一具机器照样能用,那是命根子,不许碰。
   --  这一条【必然提到身体】—— "我这根通道""我的手""我上次" —— 所以单独一段、单独出现。
   --  它是身体唯一一处可以讲【自己】的地方:我知道什么 · 我不知道什么 · 我变了什么。
   --  每一句后面都带【我凭什么信它】(量过几次 / 上次第几拍 / 来回对不对得上),
   --  没有这个,"我很有把握"和"我瞎猜的"在脑那边长得一模一样。
   if not Things_Only then
      Append (T, ASCII.LF & "ABOUT MYSELF (measured by me, on this body, with how much I trust each line):" & ASCII.LF);
      --  尺子(④ 09-27):给脑的长度按指尖长说,在这儿说一次单位是什么
      Append (T, (if Hand_Len (C) > 0.0
                  then "  MY RULER: every length I tell you is in hand-lengths. One hand-length is the distance from my eye to my fingertips, "
                       & "which I measured myself by touching the table. I have no other ruler and I do not know centimetres."
                  elsif not Any_Fingers (C)
                  then "  MY RULER: I have no fingers, so I have no hand-length to measure; any length I tell you is only in my own scale, "
                       & "which means nothing outside this body."
                  else "  MY RULER: I have not measured my own size yet, so any length I tell you is only in my own scale, "
                       & "which means nothing outside this body.") & ASCII.LF);
      --  没有手指的臂照实说一句(PLAN V1b 无人机第 2 条:"我没有手指,长度只有我自己的比例")
      for A in 0 .. C.Map.Arms - 1 loop
         if not Arm_Has_Fingers (C, A) then
            Append (T, "  I HAVE NO FINGERS on arm " & Codec.Img (A + 1) & ": I pushed its grip channel from one end to the other and nothing in any of my pictures moved. "
                    & "I cannot hold, pinch or lift anything with it." & ASCII.LF);
         end if;
      end loop;
      declare
         Said : Natural := 0;
      begin
         for I in 0 .. Natural (C.Tables.Length) - 1 loop
            declare
               Se : constant Stored_Effect := C.Tables (I);
            begin
               if Se.Cam = C.Cam and then Se.Kind = Piece_Pt and then Said < 4 then
                  for K in 0 .. Chan.Per_Arm - 1 loop
                     if Se.Trust (K) and then Said < 4 then
                        Append (T, "  I KNOW: push channel " & Codec.Img (Se.Arm * Chan.Per_Arm + K)
                                & " by one unit and a piece of me travels "
                                & Codec.Fmt (Sqrt (Se.E.B (K, 0) ** 2 + Se.E.B (K, 1) ** 2), 3)
                                & " of a frame here - learned " & Codec.Img (Se.N_Learned)
                                & " time(s), last confirmed at beat " & Codec.Img (Se.When_Beat)
                                & (if Se.Agree (K) < 0.0 then ", never checked out-and-back"
                                   elsif Se.Agree (K) < 1.0 then ", and out-and-back agree so I am confident"
                                   else ", but out-and-back DISAGREE so I do not trust it")
                                & ASCII.LF);
                        Said := Said + 1;
                     end if;
                  end loop;
               end if;
            end;
         end loop;
         if Said = 0 then
            Append (T, "  I KNOW: nothing yet about how my own pushes move me in this eye - I have not measured it here." & ASCII.LF);
         end if;
      end;
      if C.Depth_Scale > 0.0 then
         Append (T, "  I DO NOT KNOW HOW FAR: one metre of my own real motion changes my depth reading by "
                 & Codec.Fmt (C.Depth_Scale, 1) & " metres, and one metre is the most that is physically possible."
                 & " So my sense of distance is inflated by at least that much. I use it for which way, never for how far."
                 & ASCII.LF);
      end if;
      for A in 0 .. C.Map.Arms - 1 loop
         for Cm in 0 .. C.Map.N_Cams - 1 loop
            if Cam_Arm (C, Cm) /= Integer (A) and then Track_Idx (C, A, Cm) < Natural (C.Zones.Length)
              and then Cm = C.Cam
            then
               declare
                  Tr : constant Zone_Track := C.Zones (Track_Idx (C, A, Cm));
               begin
                  Append (T, "  I KNOW WHERE MY OWN HAND IS (arm " & Codec.Img (A + 1) & ") here only "
                          & (if not Tr.Valid then "not at all"
                             elsif Tr.Blew_Up then "as a guess that blew up - I refuse it"
                             elsif Tr.Known then "because I have actually looked at it from a pose close to this one"
                             else "by working it out from my joints; I have not looked at it from a pose like this")
                          & ASCII.LF);
               end;
            end if;
         end loop;
      end loop;
      --  🔴 我【怎么量远近】每只眼不一样,而且这是量过的,以前却从不告诉脑:
      --  长在手上的眼 —— 我横挪一段自己报得出的距离,看它在画面里跳多少,两条视线一交就是它离我多远 ⇒ 几大步走到
      --  (GB5 实测 4 步从差 284 mm 到差 8 mm);不跟着我动的眼 —— 它不动,我看不出远近 ⇒ 只能推一点看一点
      --  (T3 2026-09-21 实测 60 推差距 0.434 → 0.433)。T5/T6 里 Qwen 每一段都写 with my still eye,它不知道这件事。
      --  量了不说 = 把量到的东西藏起来。这是读数,不是窍门:没有例句,没有流程。
      for Cm in 0 .. C.Map.N_Cams - 1 loop
         declare
            A : constant Integer := Cam_Arm (C, Cm);
            Ready : constant Boolean := A >= 0 and then Cm < Natural (C.Geo.Length)
                                        and then C.Geo (Cm).Tip_Valid and then C.Geo (Cm).F > 0.0;
         begin
            if Ready then
               Append (T, "  HOW FAR AWAY A THING IS - through the eye that rides on arm " & Codec.Img (Natural (A) + 1)
                       & " (camera index " & Codec.Img (Cm) & ") I CAN measure it: I step sideways a distance I know and watch how far the thing jumps. "
                       & "Through that eye, touching is carried out by walking arm " & Codec.Img (Natural (A) + 1)
                       & " up to the thing in a few large steps." & ASCII.LF);
            elsif A < 0 then
               Append (T, "  HOW FAR AWAY A THING IS - through the eye that does not move with me (camera index " & Codec.Img (Cm)
                       & ") I CANNOT measure it. Through that eye I can only nudge and look again, about one pixel of progress per push." & ASCII.LF);
            end if;
         end;
      end loop;
      Append (T, To_String (C.Changed_Say));
      --  🔴 经历账读回来:它才说得出"上次我在这上面是怎么成的"。
      --  这一段【不是】命令通道的一部分 —— 它提到我自己干过什么,所以只出现在自述这一段里。
      declare
         Life : constant String := Codec.Tail_Lines (Life_Path, 6);
      begin
         if Life /= "" then
            Append (T, "  WHAT I HAVE DONE BEFORE (my own log, kept across every run I have ever had):" & ASCII.LF & Life);
         else
            Append (T, "  WHAT I HAVE DONE BEFORE: nothing - this is the first stretch I can remember." & ASCII.LF);
         end if;
      end;
   if not Qmode then
      Append (T, "PIECES OF YOURSELF (measured just now: you moved one channel at a time and watched which part of the picture followed; you closed each hand on nothing and watched which pixels swept). Each is boxed and NUMBERED on the picture in orange:" & ASCII.LF);
   end if;
   for A in 0 .. C.Map.Arms - 1 loop
    --  一条臂上量到几个抓握通道就列几组:两指手 1 组,五指手 5 组。代码里没有"一只手一个夹爪"这个假设。
    for Jk in 0 .. Jaws_Of (C, A) - 1 loop
      declare
         Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam, Jk);
         Tr : constant Zone_Track := (if Track_Idx (C, A, Cam) < Natural (C.Zones.Length) then C.Zones (Track_Idx (C, A, Cam)) else (others => <>));
         Own_Cam : constant Boolean := Cam_Arm (C, Cam) = Integer (A);
         Du : constant Long_Float := (if Own_Cam then 0.0 else Tr.Cu - Z.Cu);
         Dv : constant Long_Float := (if Own_Cam then 0.0 else Tr.Cv - Z.Cv);
         procedure Finger (Lb : Zone.Lobe; Which : Natural) is
            It : Item;
            --  这一瓣挪了多少:身体图给了各瓣位置就按瓣,否则整区平移
            Lu : constant Long_Float := (if Own_Cam then 0.0 elsif Tr.Has_Lobes then (if Which = 0 then Tr.Au else Tr.Bu) - Lb.Cu else Du);
            Lv : constant Long_Float := (if Own_Cam then 0.0 elsif Tr.Has_Lobes then (if Which = 0 then Tr.Av else Tr.Bv) - Lb.Cv else Dv);
         begin
            It.Kind := Finger; It.Arm := A; It.Which := Which; It.Jaw_K := Jk;
            if Z.Valid and then Lb.Valid and then Tr.Valid then
               It.Located := True;
               It.Cu := Lb.Cu + Lu; It.Cv := Lb.Cv + Lv;
               It.X0 := Natural (Long_Float'Max (0.0, Long_Float (Lb.X0) + Lu * Long_Float (Cw)));
               It.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), Long_Float (Lb.X1) + Lu * Long_Float (Cw))));
               It.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Lb.Y0) + Lv * Long_Float (Ch)));
               It.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), Long_Float (Lb.Y1) + Lv * Long_Float (Ch))));
               It.Depth := Tr.Z; It.Count := Lb.Count;
               Push (It, "a finger of arm " & Codec.Img (A + 1) & " (it moves when grip channel " & Codec.Img (Jk)
                     & " of that arm moves), now in cell " &
                     Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & Rel (It.Cu, It.Cv) &
                     (if Own_Cam or else Tr.Known then "" else " (placed from my joints; I have not yet looked at my hand here)"), Draw.Orange, 2);
            else
               Push (It, "a finger of arm " & Codec.Img (A + 1) & " (grip channel " & Codec.Img (Jk)
                     & ") - NOT locatable in this picture right now, do not name it", Draw.Orange, 0);
            end if;
         end Finger;
         G : Item;
      begin
         --  这个抓握通道开机推到头时哪台相机里都没看见东西跟着动 ⇒ 它不带手指:不列手指 / 爪心(零件照列,见下)
         if Has_Fingers (C, A, Jk) then
            Finger (Z.A, 0);
            Finger (Z.B, 1);
            --  🔴🔴 同一个爪的两瓣,在画面里应该只隔【量到的钳口张幅】那么远。
            --  差得离谱 = 我按关节推出来的位置在这台相机里根本不对,而这条我自己量得出来。
            --  GW 实测:arm 2(右臂)的两根手指被放到画面【左】边的第 2 格和第 19 格,相隔四分之三个画面,
            --  而它自己标着"我还没在这儿看过我的手"。位置错 ⇒ 误差错 ⇒ 往错的方向推 ⇒
            --  十炮里七炮"靠近→停在错的稳定点→退开"。必须说出来,别让脑拿它当真。
            if Z.Valid and then Z.A.Valid and then Z.B.Valid and then Z.Span > 0.0 then
               declare
                  Sep : constant Long_Float :=
                    Sqrt ((Z.A.Cu - Z.B.Cu) ** 2 + (Z.A.Cv - Z.B.Cv) ** 2);
               begin
                  --  比的是两个量出来的量,没有人拍的系数:隔得比张幅还远 ⇒ 对不上
                  if Sep > Z.Span + Z.Span and then not Qmode then
                     Append (T, "  (careful: I placed the two jaws of arm " & Codec.Img (A + 1)
                             & " " & Codec.Fmt (Sep, 3) & " of the picture apart, but the jaw span I measured on myself is only "
                             & Codec.Fmt (Z.Span, 3) & " - they cannot both be right, so where I think my hand is in this"
                             & " picture is not to be trusted)" & ASCII.LF);
                  end if;
               end;
            end if;
            G.Kind := Grip; G.Arm := A; G.Jaw_K := Jk;
            if Z.Valid and then Tr.Valid then
               G.Located := True;
               G.Cu := Tr.Cu; G.Cv := Tr.Cv; G.Depth := Tr.Z;
               G.X0 := Natural (Long_Float'Max (0.0, Long_Float (Z.X0) + Du * Long_Float (Cw)));
               G.X1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Cw - 1), Long_Float (Z.X1) + Du * Long_Float (Cw))));
               G.Y0 := Natural (Long_Float'Max (0.0, Long_Float (Z.Y0) + Dv * Long_Float (Ch)));
               G.Y1 := Natural (Long_Float'Max (0.0, Long_Float'Min (Long_Float (Ch - 1), Long_Float (Z.Y1) + Dv * Long_Float (Ch))));
               Push (G, "grip " & Codec.Img (A + 1) & " - the space between the fingers of arm " & Codec.Img (A + 1) &
                     " (closing = grip close with grip_arm " & Codec.Img (A + 1) & "; a thing must sit in this box to be held), now in cell " &
                     Codec.Img (Cell_Of (C, G.Cu, G.Cv)) & Rel (G.Cu, G.Cv), Draw.Pink, 2);
            else
               Push (G, "grip " & Codec.Img (A + 1) & " (the space between the fingers of arm " & Codec.Img (A + 1) & ") - not locatable in this picture right now", Draw.Pink, 0);
            end if;
         end if;
         --  全身零件:每个通道带的那一块(从那个关节往外的全部),位置按此刻位姿从身体图来
         if not Own_Cam and then Jk = 0 then
            for K in 0 .. Chan.Per_Arm - 1 loop
               declare
                  Pc : constant Schema.Part_Pos := Tr.Pieces (K);
                  It : Item;
               begin
                  if Pc.Valid then
                     It.Kind := Piece; It.Arm := A; It.Which := K; It.Located := True;
                     It.Cu := Pc.Cu; It.Cv := Pc.Cv; It.Depth := Pc.Z;
                     It.X0 := Pc.X0; It.Y0 := Pc.Y0; It.X1 := Pc.X1; It.Y1 := Pc.Y1;
                     Push (It, "a piece of you: everything that swings when channel " & Codec.Img (K) & " of arm " & Codec.Img (A + 1) & " moves (measured), now in cell " &
                           Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & Rel (It.Cu, It.Cv) &
                           (if Tr.Pieces_Known (K) then "" else " (placed from my joints; not yet looked at here)"), Draw.Orange, 1);
                  end if;
               end;
            end loop;
         end if;
      end;
    end loop;
   end loop;
   --  一块都没列出来就照实说一句(DR4 / DR5 2026-09-28:无人机这一行底下什么都没有,"每一块都框了编号"对着一张空单子)
   if not Qmode and then (for all It of C.Items => It.Cam /= Cam or else It.Kind not in Finger | Grip | Piece) then
      Append (T, "  (none: I found no piece of myself in this picture)" & ASCII.LF);
   end if;
   end if;
   Append (T, (if Things_Only
               then "THINGS IN MY EYE " & Codec.Img (Cam + 1) & " (same numbering - a number means the same thing everywhere I say it):"
               else "THINGS YOU HAVE NAMED (you told me which part of the picture each one is in; inside that part I measured it myself, and I measure it again every frame). Each is boxed and NUMBERED on the picture in green:") & ASCII.LF);
   --  没点过名的块不列(见下),所以这里不再需要"装不下就只列最大的几件"那一套;
   --  但它们【有多少】要如实说 —— 脑得知道这只眼里还有别的东西,只是它还没给它们起名字。
   declare
      N_Unnamed : Natural := 0;
   begin
   for Si in 0 .. World.Count (C.Wld, Cam) - 1 loop
      declare
         Sl : constant World.Slot := World.Get (C.Wld, Cam, Si);
         It : Item;
      begin
         It.Slot := Si;
         --  🔴 世界里只列【脑点过名】的东西(和此刻攥在手里的)。没点过名的块不列、不画框:
         --  编号早就不进语言了(LANGUAGE §3.1),认名字那一问也不再问"第几号"(Brain.Locate 问"在哪一框"),
         --  所以几百行 "a thing, now in cell N" 对脑没有任何用处;09-21 实测画上去的编号框还在伤它的视力
         --  (同一帧:干净画面在场 16/16,画上格子和编号框后 2/4)。全图切块照旧在跑,碰没碰到别的东西还靠它。
         declare
            Ref : constant Picture.Region := (if Sl.Present then Sl.R else Sl.Shadow);
            Held_Here : constant Boolean :=
              C.Wld.Holding and then C.Wld.Held_Slot = Si and then C.Wld.Held_Cam = Integer (Cam);
         begin
            if not Held_Here and then Boxed_Index (C, Cam, Ref.Cu, Ref.Cv) < 0 then
               if Sl.Present then
                  N_Unnamed := N_Unnamed + 1;
               end if;
               goto Next_Slot;
            end if;
         end;
         if C.Wld.Holding and then C.Wld.Held_Slot = Si and then C.Wld.Held_Cam = Integer (Cam) then
            declare
               A : constant Natural := Natural (C.Wld.Held_Arm);
               Tr : constant Zone_Track := C.Zones (Track_Idx (C, A, Cam));
               --  拿着东西的是哪一个抓握通道:身体记着(C.Wld.Held_Jaw)
               Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam, Natural (Integer'Max (0, C.Wld.Held_Jaw)));
            begin
               It.Kind := Thing_Held; It.Arm := A; It.Located := Tr.Valid;
               It.Cu := Tr.Cu; It.Cv := Tr.Cv; It.Depth := Tr.Z;
               It.X0 := Z.X0; It.Y0 := Z.Y0; It.X1 := Z.X1; It.Y1 := Z.Y1;
               It.Count := Sl.Shadow.Count; It.Height := Sl.Shadow.Height;
               Push (It, "the thing between the fingers of arm " & Codec.Img (A + 1) & " (it moves with that arm), now in cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)), Draw.Green, 2);
            end;
         elsif Sl.Present then
            It.Kind := Thing; It.Located := True;
            It.Cu := Sl.R.Cu; It.Cv := Sl.R.Cv; It.Depth := Sl.R.Depth; It.Height := Sl.R.Height; It.Count := Sl.R.Count;
            It.X0 := Sl.R.X0; It.Y0 := Sl.R.Y0; It.X1 := Sl.R.X1; It.Y1 := Sl.R.Y1;
            It.Au := Sl.R.Au; It.Av := Sl.R.Av; It.Elong := Sl.R.Elong;
            It.Gray := Picture.Mean_Gray (F.Cams (Cam).Gray, Cw, Ch, Sl.R);
            --  🔴 这一块的【胖瘦】和【亮度】我本来就量了(Elong / Gray,量它们的注释写着"用来和别的块区分"),
            --  却只留给自己重新找目标用,从不说给脑听。结果是清单上两百条字面一模一样的
            --  "a thing, now in cell N (NN px, standing 0.000)" —— 没有深度时"standing"全是 0.000,
            --  于是每一行只剩一个格号和一个像素数,谁也点不出名。点不出名就写不出"走到它那儿去",
            --  脑只好退回 close/open/still 这三个不用点名的词。**量了不说 = 把眼睛量到的东西藏起来。**
            --  ⇒ 量到什么就说什么:长宽比、平均亮度、以及这只眼这一帧自己算出来的明暗分界(基准)。
            --  这不是给窍门 —— 没有例句、没有"哪一块是球",只是把尺子上的读数念出来。
            declare
               Bx : constant Integer := Boxed_Index (C, Cam, Sl.R.Cu, Sl.R.Cv);
               Nm : constant String := (if Bx >= 0 then To_String (C.Boxed (Natural (Bx)).Name) else "");
               Alone : constant Boolean := Bx >= 0 and then C.Boxed (Natural (Bx)).Isolated;
            begin
               Push (It, "what you called " & Nm & ", now in cell " & Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & " (" & Codec.Img (It.Count) & " px, "
                     & Codec.Fmt (It.Elong, 1) & "x as long as it is wide"
                     & (if Alone then "" else "; in this eye it runs into something next to it or into the edge of the picture, so its middle and its long direction are not trustworthy here")
                     & ")" & Rel (It.Cu, It.Cv)
                     --  身体列它量得到的量(语言的根):脑要改的是这个量,不是我的手
                     & "; a quantity of it I measure and can change: height (how far it is above the surface it lies on)", Draw.Green, 2);
            end;
         elsif Sl.Seen then
            It.Kind := Thing_Remembered; It.Located := True;
            It.Cu := Sl.Shadow.Cu; It.Cv := Sl.Shadow.Cv; It.Depth := Sl.Shadow.Depth; It.Height := Sl.Shadow.Height; It.Count := Sl.Shadow.Count;
            It.X0 := Sl.Shadow.X0; It.Y0 := Sl.Shadow.Y0; It.X1 := Sl.Shadow.X1; It.Y1 := Sl.Shadow.Y1;
            It.Au := Sl.Shadow.Au; It.Av := Sl.Shadow.Av; It.Elong := Sl.Shadow.Elong;
            --  🔴 实测(另一棵树,同样这一行):凡"见过但现在看不见"的槽全列出来 ⇒ 涨到 item 317,
            --  提示词撑爆模型上下文 785 次,从第 189 轮起 560 轮一段程序都问不出来。
            --  ⇒ 只留【我确实自己挡住了它】的那些:它上次待的框跟我身上哪一块此刻的框重不重叠。
            --    重叠 = 伸手过去时那件东西消失的那一刻,留着有用;不重叠 = 它就是不见了,
            --    我说不出它在哪,也就不许拿它占脑的篇幅。
            if Under_Me (It.X0, It.Y0, It.X1, It.Y1) then
               Push (It, "what you called " & To_String (C.Boxed (Natural (Boxed_Index (C, Cam, It.Cu, It.Cv))).Name)
                     & ", now hidden behind a part of me, last seen in cell "
                     & Codec.Img (Cell_Of (C, It.Cu, It.Cv)) & " (" & Codec.Img (It.Count) & " px)", Draw.Dim_Green, 1);
            end if;
         else
            null;   --  空槽:里面此刻什么都没有,列出来只是占篇幅
         end if;
         <<Next_Slot>>
         null;
      end;
   end loop;
      if N_Unnamed > 0 then
         Append (T, "  (In this eye I can also make out " & Codec.Img (N_Unnamed)
                 & " other patches that you have not named. I do not list them: a name is how you point at a thing.)" & ASCII.LF);
      end if;
   end;
   --  相机表
   declare
      K : Natural := 2;
   begin
      Append (T, "CAMERAS (say look = k to see through that camera next turn): 1 = this picture (camera index " & Codec.Img (Cam) & ")");
      for Ci in 0 .. C.Map.N_Cams - 1 loop
         if Ci /= Cam then
            declare
               A : constant Integer := Cam_Arm (C, Ci);
            begin
               Append (T, "; " & Codec.Img (K) & " = camera index " & Codec.Img (Ci) & (if A >= 0 then " (rides on arm " & Codec.Img (Natural (A) + 1) & ": when that arm moves, that whole picture changes)" else ""));
               K := K + 1;
            end;
         end if;
      end loop;
      Append (T, ASCII.LF);
   end;
   declare
      A : constant Integer := Cam_Arm (C, Cam);
   begin
      if A >= 0 then
         Append (T, "- this picture rides on arm " & Codec.Img (Natural (A) + 1)
                 & (if Arm_Has_Fingers (C, Natural (A)) then ": its fingers and grip stay put in this picture, the world moves when that arm moves"
                    else ": the world moves in this picture when that arm moves (arm " & Codec.Img (Natural (A) + 1) & " has no fingers)")
                 & ASCII.LF);
      end if;
   end;
   if Have_Named then
      for I in 0 .. Natural (C.Items.Length) - 1 loop
         --  槽号是【每台相机各一套】的,不许拿别台相机的槽号来和这台的"上次点名"比
         if C.Items (I).Kind in Thing | Thing_Remembered | Thing_Held
           and then C.Items (I).Cam = Cam
           and then C.Items (I).Slot = C.Wld.Cams (Cam).Named
         then
            Append (T, "- the thing you last named is " & Say_Item (C, I + 1) & ", now in cell " & Codec.Img (Cell_Of (C, Named_U, Named_V)) & ASCII.LF);
         end if;
      end loop;
   end if;
   --  没有手指的身体不说"你的手指之间"(DR2 2026-09-28:无人机每一轮都被告知 "there is NOTHING between your fingers right now")
   if Any_Fingers (C) then
      Append (T, "- there is " & (if C.Wld.Holding then "ALREADY something" else "NOTHING") & " between your fingers right now" & ASCII.LF);
   else
      Append (T, "- I have no fingers: when I pushed my grip channel from one end to the other, nothing in any of my pictures moved" & ASCII.LF);
   end if;
   Text := T;
end Build_Listing;
