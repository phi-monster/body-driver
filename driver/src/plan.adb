with Codec;
with Runtime;
package body Plan is

   use type Sinew.Rel;
   use type Sinew.Noun_Kind;
   use type Sinew.Op;
   use type Sinew.Outcome;
   use type Sinew.Rank;
   use type Exam.Verdict;

   function Rows_Needed (R : Sinew.Rel) return Need is
      N : Need := [others => False];
   begin
      case R is
         when Sinew.Re_Touching =>
            --  🔴🔴 这里以前无条件也要「远近」,而【执行那一路根本不是这么写的】:
            --    P.Tu/Tv 始终带权重(画面上的对齐永远在算),
            --    距离那一路是两条各自按"读不读得到"给权重 ——
            --    `P.Wz := (if Is_Nan (Z.Depth) then 0.0 else 1.0)` 和
            --    `P.Wsize := (if Z.Span > 0.0 then 1.0 else 0.0)`,
            --    旁边的注释还写着「在手上这只眼睛里,握区的远近常常读不到(NaN),
            --    那时**「看着多大」是【唯一】的距离信号**」。
            --  ⇒ 授权表和执行器对不上,错的是授权表:它要了一行执行器并不单独依赖的量,
            --    于是无深度时「靠近」整个词被判死(CS5:79 段程序 0 条移动命令)。
            --  ⇒ 改成:左右/上下【都要】,距离信号【两条里有一条就行】(Rows_Any)。
            --    两条都没有时仍然拦 —— 那是合法的退回(量不出来),理由照样给脑。
            N (Exam.Sideways) := True; N (Exam.Updown) := True;
         when Sinew.Re_Above | Sinew.Re_Below =>
            N (Exam.Updown) := True;
         when Sinew.Re_Left | Sinew.Re_Right =>
            N (Exam.Sideways) := True;
         when Sinew.Re_Nearer | Sinew.Re_Farther | Sinew.Re_Onto | Sinew.Re_Off | Sinew.Re_Into | Sinew.Re_Press =>
            N (Exam.Nearness) := True;
         when Sinew.Re_Facing =>
            N (Exam.Facing) := True;
         when Sinew.Re_Clear =>
            N (Exam.Sideways) := True; N (Exam.Updown) := True;
         when Sinew.Re_Still | Sinew.Re_Close | Sinew.Re_Open | Sinew.Re_None =>
            null;
      end case;
      return N;
   end Rows_Needed;

   --  🔴 有些关系词的执行器【不依赖某一条具体的量,而是依赖"这几条里有一条读得到"】。
   --  `Rows_Needed` 是逐行取【与】,表达不了这件事,于是只能整行要下去 —— 那就是上面那个 bug。
   --  这张表回答的是另一个问题:这几行里【至少有一行】能用吗。空集 = 这个词没有这种依赖。
   function Rows_Any (R : Sinew.Rel) return Need is
      N : Need := [others => False];
   begin
      case R is
         when Sinew.Re_Touching =>
            --  "它离我还有多远":有深度就用深度,没有就用"看着多大"。执行器两条都带权重地用着。
            N (Exam.Nearness) := True; N (Exam.Bigness) := True;
         when others =>
            null;
      end case;
      return N;
   end Rows_Any;


   function Key_Of (N : Sinew.Noun) return String is
     (case N.K is
         when Sinew.Nk_Role => Sinew.Role_Word (N.R),
         when Sinew.Nk_None => "",
         when others => To_String (N.Word));

   function Look_Up (B : Bind_Vectors.Vector; N : Sinew.Noun) return Integer is
      K : constant String := Key_Of (N);
   begin
      if K = "" then
         return -1;
      end if;
      for I in 0 .. Natural (B.Length) - 1 loop
         if To_String (B (I).Key) = K then
            return B (I).Item;
         end if;
      end loop;
      return -1;
   end Look_Up;

   function Tried_Of (B : Bind_Vectors.Vector; N : Sinew.Noun) return String is
      K : constant String := Key_Of (N);
   begin
      for I in 0 .. Natural (B.Length) - 1 loop
         if To_String (B (I).Key) = K then
            return To_String (B (I).Tried);
         end if;
      end loop;
      return "";
   end Tried_Of;

   --  🔴🔴 负号下标【不是】越界,是「还没点名靶子」—— 造键盘那一刻(act.adb 用 Thing_Idx = -1
   --  问 Usable_Rels)必然如此:脑还没写出程序,当然还没有靶子。以前这两件被写成同一句
   --  `Thing_Idx < 0 or else Thing_Idx >= Length ⇒ False`,于是造键盘时每一行都判"不能用",
   --  每一个需要任何一行的关系词都被否掉,而 still/close/open 又被 Usable_Rels 按名字排除在外
   --  ⇒ **递给解码器的关系词表恒为空**。CS5 实测:79 段程序 0 条移动命令 —— 不是它不用,是按不出来。
   --  ⇒ 分开判。没点名靶子时,这一行"能不能用"问的是【这具身体上有没有哪一块量得出它】:
   --    有 ⇒ 这个键按得动,放上键盘;没有 ⇒ 这一行在这具身体上就是死的,不给。
   --    键盘只管"按得动吗";具体这一次点的那一块行不行,编译器到时候逐块判(Row_Ok 的正号分支),
   --    判不过就带着理由退回给脑 —— 闸只有编译器那一道,不在键盘上多设一道。
   function Row_Ok (R : Exam.Report; Thing_Idx : Integer; Row : Exam.Row_Id) return Boolean is
   begin
      if Thing_Idx < 0 then
         for I in 0 .. Integer (R.Things.Length) - 1 loop
            if Exam.Allowed (R.Things (Natural (I)).Rows (Row)) then
               return True;
            end if;
         end loop;
         return False;
      end if;
      if Thing_Idx >= Integer (R.Things.Length) then
         return False;
      end if;
      return Exam.Allowed (R.Things (Natural (Thing_Idx)).Rows (Row));
   end Row_Ok;

   --  这几行里有没有【任意一行】能用(空集恒真:这个词没有这种依赖)
   function Any_Row_Ok (R : Exam.Report; Thing_Idx : Integer; Rl : Sinew.Rel) return Boolean is
      A : constant Need := Rows_Any (Rl);
      Empty : Boolean := True;
   begin
      for Row in Exam.Row_Id loop
         if A (Row) then
            Empty := False;
            if Row_Ok (R, Thing_Idx, Row) then
               return True;
            end if;
         end if;
      end loop;
      return Empty;
   end Any_Row_Ok;

   function Rel_Ok (R : Exam.Report; Thing_Idx : Integer; Rl : Sinew.Rel; Surface : Boolean) return Boolean is
      N : constant Need := Rows_Needed (Rl);
   begin
      if Rl in Sinew.Re_Onto | Sinew.Re_Off | Sinew.Re_Into | Sinew.Re_Press and then not Surface then
         return False;
      end if;
      for Row in Exam.Row_Id loop
         if N (Row) and then not Row_Ok (R, Thing_Idx, Row) then
            return False;
         end if;
      end loop;
      return Any_Row_Ok (R, Thing_Idx, Rl);
   end Rel_Ok;

   function Oc_Waitable (O : Sinew.Outcome; Surface : Boolean) return Boolean is
   begin
      case O is
         when Sinew.Oc_None | Sinew.Oc_Arrived | Sinew.Oc_Refused =>
            return False;            --  到没到只有脑能判;refused 是我回给你的话,不是我能等来的事
         when Sinew.Oc_Free =>
            return Surface;          --  要知道它原来靠着哪个面,才谈得上"它离开了那个面"
         when others =>
            return True;
      end case;
   end Oc_Waitable;

   function Waitable_Outcomes (Surface : Boolean) return String is
      S : Unbounded_String;
   begin
      for O in Sinew.Outcome loop
         if Oc_Waitable (O, Surface) then
            Append (S, (if Length (S) > 0 then " " else "") & Sinew.Outcome_Word (O));
         end if;
      end loop;
      return To_String (S);
   end Waitable_Outcomes;

   function Usable_Rels (R : Exam.Report; Thing_Idx : Integer; Surface : Boolean) return String is
      S : Unbounded_String;
   begin
      for Rl in Sinew.Rel loop
         if Rl not in Sinew.Re_None | Sinew.Re_Still | Sinew.Re_Close | Sinew.Re_Open
           and then Rel_Ok (R, Thing_Idx, Rl, Surface)
         then
            Append (S, (if Length (S) > 0 then " " else "") & Sinew.Rel_Word (Rl));
         end if;
      end loop;
      return (if Length (S) = 0 then "(一个都没有)" else To_String (S));
   end Usable_Rels;

   --  🔴🔴 键盘和编译器必须是【同一个口径】,以前不是(SC4 2026-09-21 实测,离线体检打出原文):
   --    编译器(Check)对"还没量过响应的那个我"是【放行】的 —— `Unknown := Ti < 0` 跳过逐行检查,
   --    执行器(Run_Segment 的 Need 分支)当场推一遍量出来,量完补判;
   --    键盘却拿 -1 去问 Usable_Rels ⇒ Row_Ok 的负号分支 = "扫一遍【量过的】每一块"。
   --  身体文件里一张响应表都没有时(`装回来了 … 0 张响应表`)扫出来恒为空 ⇒ 递给解码器的关系词表
   --  = (一个都没有) ⇒ 脑一条移动命令都写不出来;而响应表【只有执行移动命令时】才会去量
   --  ⇒ 死锁:没表 ⇒ 没键 ⇒ 没命令 ⇒ 永远没表。谁当脑都一样,一步都走不了。
   --  "没量过"不是"做不到"。⇒ 键盘 = 此刻绑得上的每一个"我",各按编译器那一套判,取并集:
   --    量过的(Thing_Idx ≥ 0)⇒ Rel_Ok 逐行判,量死的行照样不给("零死键"那条不受影响);
   --    没量过的(Thing_Idx < 0)⇒ 编译器会放行 ⇒ 键盘也给;只有靠"面"的那几个词仍按 Surface 拦
   --    (和 Check 里 `not Surface ⇒ Reject` 是同一个条件)。
   function Usable_Rels_Any (R : Exam.Report; Subjects : Facts_Vectors.Vector; Surface : Boolean) return String is
      S : Unbounded_String;
   begin
      for Rl in Sinew.Rel loop
         if Rl not in Sinew.Re_None | Sinew.Re_Still | Sinew.Re_Close | Sinew.Re_Open then
            declare
               Any : Boolean := False;
            begin
               for I in 0 .. Natural (Subjects.Length) - 1 loop
                  if Subjects (I).Exists and then Subjects (I).Mine then
                     if Subjects (I).Thing_Idx < 0 then
                        Any := not (Rl in Sinew.Re_Onto | Sinew.Re_Off | Sinew.Re_Into | Sinew.Re_Press
                                    and then not Surface);
                     else
                        Any := Rel_Ok (R, Subjects (I).Thing_Idx, Rl, Surface);
                     end if;
                  end if;
                  exit when Any;
               end loop;
               if Any then
                  Append (S, (if Length (S) > 0 then " " else "") & Sinew.Rel_Word (Rl));
               end if;
            end;
         end if;
      end loop;
      return (if Length (S) = 0 then "(一个都没有)" else To_String (S));
   end Usable_Rels_Any;

   function Why_Row (R : Exam.Report; Thing_Idx : Integer; Row : Exam.Row_Id) return String is
   begin
      if Thing_Idx < 0 or else Thing_Idx >= Integer (R.Things.Length) then
         return "这一块我还没量过响应 —— 我不知道推哪根通道会让它动";
      end if;
      return To_String (R.Things (Natural (Thing_Idx)).Rows (Row).Why);
   end Why_Row;

   function Mine_List (F : Facts_Vectors.Vector) return String is
      S : Unbounded_String;
   begin
      for I in 0 .. Natural (F.Length) - 1 loop
         if F (I).Exists and then F (I).Mine and then Length (F (I).Label) > 0 then
            Append (S, (if Length (S) > 0 then " · " else "") & To_String (F (I).Label));
         end if;
      end loop;
      return (if Length (S) = 0 then "(一个都没有)" else To_String (S));
   end Mine_List;

   function Check (P : Sinew.Program; R : Exam.Report; Facts : Facts_Vectors.Vector;
                   B : Bind_Vectors.Vector) return Verdict is
      V : Verdict;

      procedure Reject (Line : Natural; Msg : String; Alt : String := "") is
      begin
         if V.Ok then
            V.Ok := False;
            V.Err := To_Unbounded_String (Msg);
            V.Instead := To_Unbounded_String (Alt);
            V.Err_Line := Line;
         end if;
      end Reject;

      function Resolve (N : Sinew.Noun; Line : Natural; Who : String; Num : out Natural) return Boolean is
         A : constant Integer := Look_Up (B, N);
         T : constant String := Tried_Of (B, N);
      begin
         Num := 0;
         if N.K = Sinew.Nk_None then
            Reject (Line, "这一句没说" & Who);
            return False;
         end if;
         if A < 0 or else A >= Integer (Facts.Length) or else not Facts (Natural (A)).Exists then
            if N.K = Sinew.Nk_Role then
               Reject (Line, "我身上没有「" & Key_Of (N) & "」这个东西",
                       "我量出来身上有的是:" & Mine_List (Facts));
            else
               Reject (Line, "我认不出「" & Key_Of (N) & "」"
                       & (if T = "" then "" else " —— " & T),
                       "换一个说法,或者先让我看清它");
            end if;
            return False;
         end if;
         Num := Natural (A);
         return True;
      end Resolve;
   begin
      if not P.Ok then
         V.Ok := False;
         V.Err := P.Err;
         V.Err_Line := P.Err_Line;
         V.Instead := To_Unbounded_String ("文法我每一轮都给你,照着改一行就行");
         return V;
      end if;
      for Ix in 0 .. Natural (P.Code.Length) - 1 loop
         exit when not V.Ok;
         declare
            I : constant Sinew.Instr := P.Code (Ix);
         begin
            if I.O = Sinew.Op_Interval then
               --  一节里两条 must 抢同一行 ⇒ 解算时一定得牺牲一条,不许跑
               declare
                  Taken : Need := [others => False];
               begin
                  for Ci in 0 .. Natural (I.Cons.Length) - 1 loop
                     if I.Cons (Ci).Rk = Sinew.Rk_Must then
                        declare
                           Nd : constant Need := Rows_Needed (I.Cons (Ci).R);
                        begin
                           for Row in Exam.Row_Id loop
                              if Nd (Row) and then Taken (Row) then
                                 Reject (I.Line, "这一节里有两条 must 都在管「" & Exam.Row_Name (Row)
                                         & "」这一行 —— 我一定得牺牲一条",
                                         "把其中一条的 must 去掉(去掉就成了可以被牺牲的那种),"
                                         & "或者换一个不抢这一行的关系");
                              end if;
                              if Nd (Row) then
                                 Taken (Row) := True;
                              end if;
                           end loop;
                        end;
                     end if;
                  end loop;
               end;
               --  🔴 「until refused」说不出口:refused 是【我做不到】,是我回给你的话,不是我能等来的事。
               --  等自己开口拒绝,这一段永远走不完。宁可当场说不认,也不许悄悄换成"走够步数"
               --  (lost / free / refused 三个词以前统统被换成步数上限,脑写什么身体做的是别的)。
               --  🔴🔴 「until arrived」也说不出口(owner 2026-09-14 死命令):
               --  "到了没到"只有【你】能判 —— 我只能量我量得到的事件(碰到 / 顶住 / 滑了 / 跟丢 /
               --  离开了面 / 合拢停住)。"我觉得够近了"是意见,不是事件,而且我判错过:
               --  一段只走 1 推就宣布"到了",实际离你点名的东西还差 0.2 米、只有该有大小的 7.7%。
               --  给我步数,我走完就回来把画面给你看,由你判。
               if I.Until_Oc = Sinew.Oc_Arrived then
                  Reject (I.Line, "「until arrived」我说不出口 —— 到没到只有你能判,我只量得到事件,"
                          & "判不了「够近了」;我自己判过,判错了(一推就说到了,实际还差得远)",
                          "写 or N steps:我走完 N 推就回来,把每台相机的画面给你,你看了再说下一步;"
                          & "想让我推到碰上为止写 until touched,推到推不动写 until stuck");
               end if;
               if I.Until_Oc = Sinew.Oc_Refused then
                  Reject (I.Line, "「until refused」我说不出口 —— refused 是我做不到的时候回给你的话,"
                          & "不是我能等来的事;等我自己开口拒绝,这一段永远走不完",
                          "想让我一直推到推不动为止,写 until stuck;想限步数,写 or N steps");
               end if;
               for Ci in 0 .. Natural (I.Cons.Length) - 1 loop
                  exit when not V.Ok;
                  declare
                     C : constant Sinew.Constraint := I.Cons (Ci);
                     Sub, Obj : Natural := 0;
                  begin
                     if not Resolve (C.Subj, I.Line, "【谁】", Sub) then
                        exit;
                     end if;
                     if not Facts (Sub).Mine then
                        Reject (I.Line, "「" & Key_Of (C.Subj) & "」不是我身上的东西 —— 我只推得动我自己",
                                "我量出来身上有的是:" & Mine_List (Facts));
                        exit;
                     end if;
                     if C.R in Sinew.Re_Close | Sinew.Re_Open and then not Facts (Sub).Grasp then
                        Reject (I.Line, "「" & Key_Of (C.Subj) & "」合不拢 —— 我没量到它有两块能相向靠拢的部件",
                                "我身上能合拢的是:" & Mine_List (Facts));
                        exit;
                     end if;
                     if C.Obj.K /= Sinew.Nk_None then
                        if not Resolve (C.Obj, I.Line, "【和谁】", Obj) then
                           exit;
                        end if;
                     elsif C.R = Sinew.Re_Close then
                        Obj := 0;   --  就在这儿合上:不点名任何东西
                     end if;
                     declare
                        Ti : constant Integer := Facts (Sub).Thing_Idx;
                        Nd : constant Need := Rows_Needed (C.R);
                        Surface : constant Boolean :=
                          C.Obj.K = Sinew.Nk_None or else Facts (Obj).Stands;
                        Unknown : constant Boolean := Ti < 0;   --  还没量过 ⇒ 执行器量完当场补判
                     begin
                        if C.R in Sinew.Re_Onto | Sinew.Re_Off | Sinew.Re_Into | Sinew.Re_Press and then not Surface then
                           Reject (I.Line, "我量不出「" & Key_Of (C.Obj) & "」鼓出它靠着的那个面多少,"
                                   & "所以我不知道哪个方向才算朝它压",
                                   "我现在说得出口的关系:" & Usable_Rels (R, Ti, False));
                           exit;
                        end if;
                        if I.Until_Oc = Sinew.Oc_Free and then not Surface then
                           Reject (I.Line, "「until free」要知道它原来靠着哪个面,我量不出来",
                                   "换成 until touched 或 until stuck —— 这两个不用面");
                           exit;
                        end if;
                        if not Unknown then
                           for Row in Exam.Row_Id loop
                              if Nd (Row) and then not Row_Ok (R, Ti, Row) then
                                 Reject (I.Line,
                                   "「" & Sinew.Rel_Word (C.R) & "」(" & Sinew.Rel_Cn (C.R) & ")要靠「"
                                   & Exam.Row_Name (Row) & "」这一行,而这一行在我身上不能用:"
                                   & Why_Row (R, Ti, Row),
                                   "我现在说得出口的关系:" & Usable_Rels (R, Ti, Surface));
                                 exit;
                              end if;
                           end loop;
                           --  🔴 上面那个循环管的是【每一行都得有】;这一条管【这几行里得有一行】。
                           --  把「靠近」从"无条件要远近"放开之后,这一道必须补上,否则就是拆了闸不换 ——
                           --  两条距离信号一条都读不到时,像素一对齐就会被判成"到了"(GE 实测差着 20 厘米)。
                           --  拦住的理由是"我量不出来",这是合法的退回,不是"我做不到"。
                           if not Any_Row_Ok (R, Ti, C.R) then
                              Reject (I.Line,
                                "「" & Sinew.Rel_Word (C.R) & "」(" & Sinew.Rel_Cn (C.R)
                                & ")要知道它离我还有多远,而这两条我一条都读不到:"
                                & "「" & Exam.Row_Name (Exam.Nearness) & "」" & Why_Row (R, Ti, Exam.Nearness)
                                & " · 「" & Exam.Row_Name (Exam.Bigness) & "」" & Why_Row (R, Ti, Exam.Bigness),
                                "我现在说得出口的关系:" & Usable_Rels (R, Ti, Surface));
                           end if;
                        end if;
                     end;
                  end;
               end loop;
            end if;
         end;
      end loop;
      return V;
   end Check;

   function Dry_Run (P : Sinew.Program; R : Exam.Report; Facts : Facts_Vectors.Vector;
                     B : Bind_Vectors.Vector) return Verdict is
      use Sinew;
      V : Verdict;
      M : Runtime.Machine;
      W : Runtime.Yield;
      I : Sinew.Instr;
      Segments : Natural := 0;

      --  这一节里有没有"心里就走不通"的事。有 ⇒ 直接判死,不绕结局
      --  (until stuck 本来就是合手的正常写法,绕结局绕不出来)。
      function Impossible (Ins : Sinew.Instr) return String is
         pragma Unreferenced (Ins);
      begin
         --  🔴 这里原来有一条"整块比爪口宽 ⇒ 这一节判死"。删了。
         --  T5 2026-09-21 实测:Qwen 写 `close mint green scissors`,被它挡在动手之前 —— 它比的是整块的【外廓】
         --  (剪刀 = 它的全长,那一集还连着旁边的风扇),而爪子夹的是它【窄】的那一向:剪刀永远比爪口长,
         --  于是"合上剪刀"这句话在这具身体上永远编译不过。闸门棘轮的注释 09-18 就点过它的名(把抓剪刀整段挡在动手之前)。
         --  总规矩(LAB 09-13):身体只准因为【量过期 / 依赖失效 / 量不出来】拒绝;成没成由合完提一提来判,不由我事先断言。
         return "";
      end Impossible;

      --  这一节【最好的情况】会是什么结局。乐观是故意的:乐观都还停不下来的循环,真跑更停不下来。
      function Predict (Ins : Sinew.Instr) return Outcome is
        (if Ins.Until_Oc = Oc_None then Oc_Arrived else Ins.Until_Oc);
   begin
      if not P.Ok then
         return (Ok => False, Err => P.Err, Instead => <>, Err_Line => P.Err_Line);
      end if;
      loop
         Runtime.Advance (P, M, W, I);
         case W is
            when Runtime.Y_Interval =>
               Segments := Segments + 1;
               if Segments > 4096 then      --  空转都走这么多节 = 它停不下来(次数,无量纲)
                  return (Ok => False,
                          Err => To_Unbounded_String ("我在心里把这段跑了一遍,它停不下来 —— 走了几千节还没到头"),
                          Instead => To_Unbounded_String ("给循环一个真到得了的出口,或者改成 repeat <几> times"),
                          Err_Line => I.Line);
               end if;
               declare
                  Bad : constant String := Impossible (I);
               begin
                  if Bad /= "" then
                     return (Ok => False,
                             Err => To_Unbounded_String ("这一节我在心里就走不通:" & Bad),
                             Instead => To_Unbounded_String ("换一个我夹得住的东西,或者先把它推到别处再夹"),
                             Err_Line => I.Line);
                  end if;
                  Runtime.Report (P, M, Predict (I));
               end;
            when Runtime.Y_Say | Runtime.Y_Remember =>
               null;
            when Runtime.Y_Done | Runtime.Y_Finished =>
               exit;
            when Runtime.Y_Broken =>
               return (Ok => False,
                       Err => To_Unbounded_String (Runtime.Broken_Why (M)),
                       Instead => To_Unbounded_String ("给循环一个真到得了的出口;调用的名字要先 to 过"),
                       Err_Line => I.Line);
         end case;
      end loop;
      pragma Unreferenced (R);
      return V;
   end Dry_Run;

   function Say (V : Verdict) return String is
   begin
      if V.Ok then
         return "编译过了";
      end if;
      return "第 " & Codec.Img (V.Err_Line) & " 行退回来了:" & To_String (V.Err)
        & (if Length (V.Instead) > 0 then ASCII.LF & "   ⇒ " & To_String (V.Instead) else "");
   end Say;

end Plan;
