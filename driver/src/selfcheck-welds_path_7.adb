separate (Selfcheck)
procedure Welds_Path_7 is
   --  路 7 的焊点(大并行.md §5 路 7):每条写清"错了会是什么病",带一颗牙(去掉那一改就红)
   use type Plan.Name_Verdict_Kind;

   function U (S : String) return Unbounded_String renames To_Unbounded_String;
   function Rec (Name : String; Eye : Natural; Boxed, Seen : Boolean; Blind : Boolean := False) return Plan.Named_Record is
     (Plan.Named_Record'(Name => U (Name), Eye => Eye, Boxed => Boxed, Seen => Seen, Blind => Blind));

   --  假画面:W × H,一件东西 = 一块矩形像素;它身上那一点 = 矩形正中那一格
   Fw : constant Positive := 12;
   Fh : constant Positive := 8;
   function Blob (X0, Y0, X1, Y1 : Natural) return Bools is
      M : Bools := Bool_Vectors.To_Vector (False, Ada.Containers.Count_Type (Fw * Fh));
   begin
      for Y in Y0 .. Y1 loop
         for X in X0 .. X1 loop
            M.Replace_Element (Y * Fw + X, True);
         end loop;
      end loop;
      return M;
   end Blob;

   --  ── 重放用的模型(落盘里摘的每一轮,按 Bind_Name 的顺序、用 Plan 里同一套判法)──
   --  一条记录 = Act.Boxed_Thing 那几样 + 它其实是什么(剪刀 / 我自己的胳膊 / 只记了一句"指不出"的那种);
   --  "其实是什么"只给打分用,判法看不见它
   type Ident is (Id_Scissors, Id_Arm, Id_Mark);
   type Model_Rec is record
      R : Plan.Named_Record;
      Id : Ident := Id_Mark;
      Mask : Bools;
      Pu, Pv : Long_Float := -1.0;
   end record;
   package Model_Vectors is new Ada.Containers.Vectors (Natural, Model_Rec);
   Store : Model_Vectors.Vector;
   Visible : array (0 .. 2) of Boolean := [others => False];   --  这一轮剪刀在第几台眼里看得见(日志里的清单)
   --  眼对一个名字怎么答:框在剪刀上 / 框在我自己的胳膊上 / 这里指不出(落盘里那只眼的原话;日志里没问过的,用落盘的画面重问过)
   type Answer is (Here_Scissors, Here_Arm, Not_Here);
   --  脑写这个名字是在说什么(打分用):剪刀 / 什么都不是(动作、乱码)/ 看不出
   type Meaning is (Means_Scissors, Means_Nothing, Means_Unclear);
   type Tally is record
      Right, Fail_Thing, Fail_Honest, Wrong : Natural := 0;
   end record;
   T : Tally;
   Prog : Plan.Bind_Vectors.Vector;                   --  这一轮程序里的名字(按行的先后)
   type Mean_Array is array (Positive range <>) of Meaning;
   Truths : Mean_Array (1 .. 64);                     --  和 Prog 同一个顺序
   Eye_Now : Natural := 0;

   function Records return Plan.Named_Vectors.Vector is
      Rs : Plan.Named_Vectors.Vector;
   begin
      for M of Store loop
         Rs.Append (M.R);
      end loop;
      return Rs;
   end Records;

   --  清单上的号:这一帧量到的那几条才在清单上(和 Act 一样,号 = 下标 + 1)
   function Item_Of (Bx : Natural) return Natural is
     (if Bx < Natural (Store.Length) and then Store (Bx).R.Seen then Bx + 1 else 0);

   procedure Mark_Blind (Eye : Natural; Name : Unbounded_String) is
   begin
      for I in 0 .. Natural (Store.Length) - 1 loop
         if Store (I).R.Eye = Eye and then Store (I).R.Name = Name then
            declare
               M : Model_Rec := Store (I);
            begin
               M.R.Blind := True; M.R.Seen := False;
               Store.Replace_Element (I, M);
            end;
            return;
         end if;
      end loop;
      Store.Append (Model_Rec'(R => (Name => Name, Eye => Eye, Boxed => False, Seen => False, Blind => True),
                               Id => Id_Mark, Mask => Bools'(Bool_Vectors.Empty_Vector), Pu => -1.0, Pv => -1.0));
   end Mark_Blind;

   --  Act.Clear_Blind 同一个规矩
   procedure Clear (Eye : Natural) is
   begin
      for I in reverse 0 .. Natural (Store.Length) - 1 loop
         if Store (I).R.Eye = Eye and then Store (I).R.Blind then
            if Plan.Forget_When_Eye_Moves (Store (I).R) then
               Store.Delete (I);
            else
               declare
                  M : Model_Rec := Store (I);
               begin
                  M.R.Blind := False;
                  Store.Replace_Element (I, M);
               end;
            end if;
         end if;
      end loop;
   end Clear;

   --  新的一轮在第 Eye 台眼里:每条没被标"指不出"的记录在它的框里重量一遍(Remeasure_Boxed)。
   --  剪刀:看得见才量得到;我的胳膊:一直在画面里;只记了一句"指不出"的那种一旦被解除,它的框是 (0,0,0,0),
   --  实测(S1A4、S1A1 的清单)框里总量得出一块来 ⇒ 它从此是一条"量到了"的记录 —— 一块假东西
   procedure Round (Eye : Natural) is
   begin
      Eye_Now := Eye;
      Prog.Clear;
      for I in 0 .. Natural (Store.Length) - 1 loop
         declare
            M : Model_Rec := Store (I);
         begin
            if M.R.Eye = Eye and then not M.R.Blind then
               case M.Id is
                  when Id_Scissors => M.R.Seen := Visible (Eye);
                  when Id_Arm => M.R.Seen := True;
                  when Id_Mark => M.R.Seen := True; M.R.Boxed := True;
               end case;
               Store.Replace_Element (I, M);
            end if;
         end;
      end loop;
   end Round;

   --  脑的一个名字按 Bind_Name 的顺序落下去:① 这只眼这一帧量到的里面有同一串字母的 ⇒ 它;② 问眼(答案是落盘的);
   --  指出来了 ⇒ 和这只眼里量到的哪一件同一片像素 ⇒ 就是它、改叫这个名字,不然记成新的一件(别的眼里同一件改叫这个名字 = 视线交在一点);
   --  ③ 眼说这里没有 ⇒ 只按字找(Plan.Without_Eye)。返回清单上的号(0 = 绑不上)
   function Bind (W : String; Ans : Answer) return Natural is
      V : Plan.Name_Verdict := Plan.Before_Eye (W, Eye_Now, Records);
   begin
      if V.Kind = Plan.Nv_This then
         return Item_Of (Natural (V.Index));
      end if;
      if Ans /= Not_Here then
         declare
            Mk : constant Bools := (if Ans = Here_Scissors then Blob (2, 2, 4, 5) else Blob (7, 1, 11, 7));
            Pu : constant Long_Float := (if Ans = Here_Scissors then 3.0 else 9.0);
            Pv : constant Long_Float := 4.0;
            Same : Integer := -1;
         begin
            for I in 0 .. Natural (Store.Length) - 1 loop
               if Same < 0 and then Store (I).R.Eye = Eye_Now and then Store (I).R.Seen
                 and then Plan.Same_Pixels (Store (I).Mask, Store (I).Pu, Store (I).Pv, Mk, Pu, Pv, Fw, Fh)
               then
                  Same := Integer (I);
               end if;
            end loop;
            if Same >= 0 then
               declare
                  Old : constant Unbounded_String := Store (Natural (Same)).R.Name;
               begin
                  for I in 0 .. Natural (Store.Length) - 1 loop
                     if Store (I).R.Name = Old then
                        declare
                           M : Model_Rec := Store (I);
                        begin
                           M.R.Name := U (W);
                           Store.Replace_Element (I, M);
                        end;
                     end if;
                  end loop;
               end;
               return Item_Of (Natural (Same));
            end if;
            declare
               Id : constant Ident := (if Ans = Here_Scissors then Id_Scissors else Id_Arm);
            begin
               Store.Append (Model_Rec'(R => (Name => U (W), Eye => Eye_Now, Boxed => True, Seen => True, Blind => False),
                                        Id => Id, Mask => Mk, Pu => Pu, Pv => Pv));
               for I in 0 .. Natural (Store.Length) - 2 loop
                  if Store (I).R.Eye /= Eye_Now and then Store (I).R.Seen and then Store (I).Id = Id then
                     declare
                        M : Model_Rec := Store (I);
                     begin
                        M.R.Name := U (W);
                        Store.Replace_Element (I, M);
                     end;
                  end if;
               end loop;
               return Natural (Store.Length);
            end;
         end;
      end if;
      V := Plan.Without_Eye (W, Eye_Now, Records);
      if V.Kind in Plan.Nv_This | Plan.Nv_Elsewhere and then Item_Of (Natural (V.Index)) > 0 then
         if V.Kind = Plan.Nv_Elsewhere then
            Mark_Blind (Eye_Now, V.Name);
         end if;
         return Item_Of (Natural (V.Index));
      end if;
      Mark_Blind (Eye_Now, (if V.Kind in Plan.Nv_This | Plan.Nv_Elsewhere | Plan.Nv_Not_Seen then V.Name else U (W)));
      return 0;
   end Bind;

   procedure Ask (W : String; Ans : Answer; Truth : Meaning) is
      E : Plan.Bind_Entry;
   begin
      E.Key := U (W);
      for B of Prog loop
         if To_String (B.Key) = W then
            return;   --  同一段程序里同一个名字只认一次(Bind_All 同一条)
         end if;
      end loop;
      E.Item := Integer (Bind (W, Ans));
      Prog.Append (E);
      Truths (Natural (Prog.Length)) := Truth;
   end Ask;

   --  这一轮的程序认完:第二遍(Plan.Rebind_Missing),再按脑说的是什么打分
   procedure Score is
      Got : Natural;
   begin
      Plan.Rebind_Missing (Prog, Records, Eye_Now, Item_Of'Access, Got);
      for I in 0 .. Natural (Prog.Length) - 1 loop
         declare
            It : constant Integer := Prog (I).Item;
            Id : constant Ident := (if It > 0 then Store (Natural (It) - 1).Id else Id_Mark);
         begin
            case Truths (I + 1) is
               when Means_Scissors =>
                  if It <= 0 then
                     T.Fail_Thing := T.Fail_Thing + 1;
                  elsif Id = Id_Scissors then
                     T.Right := T.Right + 1;
                  else
                     T.Wrong := T.Wrong + 1;
                  end if;
               when Means_Nothing | Means_Unclear =>
                  if It <= 0 then
                     T.Fail_Honest := T.Fail_Honest + 1;
                  elsif Id = Id_Scissors then
                     T.Right := T.Right + 1;   --  它的字里含着剪刀以前的名字:按字绑到剪刀
                  else
                     T.Wrong := T.Wrong + 1;
                  end if;
            end case;
         end;
      end loop;
   end Score;

   procedure Start_Run is
   begin
      Store.Clear;
      T := (others => <>);
      Visible := [others => False];
   end Start_Run;

   --  第 Eye 台眼里有过框的、其实是 Id 那件东西的记录有几条(同一件东西在同一只眼里只许有一条)
   function Count_Of (Id : Ident; Eye : Natural) return Natural is
      N : Natural := 0;
   begin
      for M of Store loop
         if M.Id = Id and then M.R.Eye = Eye and then M.R.Boxed then
            N := N + 1;
         end if;
      end loop;
      return N;
   end Count_Of;

   function Say (X : Tally) return String is
     ("绑对 " & Codec.Img (X.Right) & " · 该绑上没绑上(照实说了为什么)" & Codec.Img (X.Fail_Thing)
      & " · 本来就不是东西、照实说绑不上 " & Codec.Img (X.Fail_Honest) & " · 绑错 " & Codec.Img (X.Wrong));
begin
   Put_Line ("── 路 7:名字怎么绑(大并行 §2 第 15 条)──");

   --  ① 粘词。病:Qwen 在受限解码下常把名字粘成一个词(mintgreenscissors、pick upmint greenscissors),
   --  身体按原字符串比 ⇒ 同一件东西的同一个名字认不出,只能再问一遍眼(眼有时也认不出,就成了绑不上)。
   --  🦷 Plan.Same_Name 改回按原字符串比(A = B)⇒ 这一条红
   declare
      Rs : Plan.Named_Vectors.Vector;
      V : Plan.Name_Verdict;
   begin
      Rs.Append (Rec ("mint green scissors", 0, Boxed => True, Seen => True));
      V := Plan.Before_Eye ("mintgreenscissors", 0, Rs);
      Check (V.Kind = Plan.Nv_This and then V.Index = 0,
             "粘词:这只眼里量到的「mint green scissors」,脑写成「mintgreenscissors」⇒ 同一件,不再问眼(" & Plan.Name_Verdict_Kind'Image (V.Kind) & ")");
      V := Plan.Without_Eye ("MintGreen Scissors", 1, Rs);
      Check (V.Kind = Plan.Nv_Elsewhere and then V.Index = 0,
             "粘词:在另一只眼里写成「MintGreen Scissors」(大小写、空格都变了)、那只眼说这里没有 ⇒ 按字就是第 0 台里那一件(" & Plan.Name_Verdict_Kind'Image (V.Kind) & ")");
      --  大并行 §2 第 15 条点名的那一句。🦷 Plan.Holds_Name 改回按原字符串找 ⇒ 这一条红(pinktissueby 里找不到带空格的 pink tissue)
      Rs.Clear;
      Rs.Append (Rec ("pink tissue", 0, Boxed => True, Seen => True));
      V := Plan.Without_Eye ("pick upthe pinktissueby", 1, Rs);
      Check (V.Kind = Plan.Nv_Elsewhere and then V.Index = 0,
             "粘词:「pick upthe pinktissueby」原样含着「pink tissue」的字母 ⇒ 就是第 0 台里那一件(" & Plan.Name_Verdict_Kind'Image (V.Kind) & ")");
      Rs.Clear;
      Rs.Append (Rec ("the mint green", 0, Boxed => True, Seen => True));
      V := Plan.Without_Eye ("pick upthe pinktissueby", 1, Rs);
      Check (V.Kind = Plan.Nv_Unknown,
             "粘词(反例):只认得「the mint green」时,「pick upthe pinktissueby」不绑到它(共用一个 the 不算)⇒ 照实说绑不上");
   end;

   --  ② 拆词。病:同一个名字被拆开(scis sors),按原字符串比就成了另一件。🦷 同上
   declare
      Rs : Plan.Named_Vectors.Vector;
      V : Plan.Name_Verdict;
   begin
      Rs.Append (Rec ("scissors", 0, Boxed => True, Seen => True));
      V := Plan.Before_Eye ("scis sors", 0, Rs);
      Check (V.Kind = Plan.Nv_This and then V.Index = 0,
             "拆词:「scissors」写成「scis sors」⇒ 同一件(" & Plan.Name_Verdict_Kind'Image (V.Kind) & ")");
   end;

   --  ③ 错一个字母。病:字母差一个(scisors)按字认不出;要是再按"差几个字母算同一个"去认,cap 就成了 cup。
   --  这里由眼认:问眼它在哪一框,框出来的和这只眼里已经量到的那一件是同一片像素 ⇒ 同一件,不另起一件。
   --  反例:剪刀的把手(在剪刀身上,可剪刀身上那一点不在把手上)不算同一片;旁边另一件更不算。
   --  🦷 Plan.Same_Pixels 恒答 False(不按像素认)⇒ 重放里 S1A1 R8 同一把剪刀又记成新的一件(这一条和重放那几条红)
   declare
      Scissors : constant Bools := Blob (2, 2, 4, 5);
      Handle : constant Bools := Blob (2, 2, 2, 3);
      Other : constant Bools := Blob (7, 1, 11, 7);
      V : Plan.Name_Verdict;
      Rs : Plan.Named_Vectors.Vector;
   begin
      Check (Plan.Same_Pixels (Scissors, 3.0, 4.0, Scissors, 3.0, 3.0, Fw, Fh),
             "错一个字母:眼给「scisors」框出来的那一片,和我叫「scissors」的那一片各自身上那一点都落在对方身上 ⇒ 同一片");
      Check (not Plan.Same_Pixels (Scissors, 3.0, 4.0, Handle, 2.0, 2.0, Fw, Fh),
             "错一个字母(反例):剪刀的把手在剪刀身上,可剪刀身上那一点不在把手上 ⇒ 不是同一片(部件不并进整件)");
      Check (not Plan.Same_Pixels (Scissors, 3.0, 4.0, Other, 9.0, 4.0, Fw, Fh),
             "错一个字母(反例):旁边另一件 ⇒ 不是同一片");
      Rs.Append (Rec ("scissors", 0, Boxed => True, Seen => True));
      V := Plan.Without_Eye ("scisors", 0, Rs);
      Check (V.Kind = Plan.Nv_Unknown,
             "错一个字母:眼没指出来的时候按字不认(「scisors」不是「scissors」,也不含它)⇒ 照实说绑不上,不猜");
   end;

   --  ④ 不猜。病:按"共用一个词 / 差几个字母"认,the red ball 会绑到 the red cup;两件都对得上时挑一件更是瞎猜。
   --  🦷 Without_Eye 对得上两件时挑第一件 ⇒ 第一条红;改成"共用一个词就算"⇒ 第二条红
   declare
      Rs : Plan.Named_Vectors.Vector;
      V : Plan.Name_Verdict;
   begin
      Rs.Append (Rec ("the mint green", 0, Boxed => True, Seen => True));
      Rs.Append (Rec ("green", 0, Boxed => True, Seen => True));
      V := Plan.Without_Eye ("pick up the mint green", 1, Rs);
      Check (V.Kind = Plan.Nv_Ambiguous and then Index (V.Name, "the mint green") > 0 and then Index (V.Name, "「green」") > 0,
             "不猜:「pick up the mint green」原样含着「the mint green」也含着「green」⇒ 两件都对得上,照实说不猜(" & To_String (V.Name) & ")");
      Rs.Clear;
      Rs.Append (Rec ("the red cup", 0, Boxed => True, Seen => True));
      V := Plan.Without_Eye ("the red ball", 1, Rs);
      Check (V.Kind = Plan.Nv_Unknown and then Index (V.Known, "the red cup") > 0,
             "不猜:「the red ball」和「the red cup」共用 the red 两个词 ⇒ 不是同一件;绑不上时照实列出起过的名字(" & To_String (V.Known) & ")");
   end;

   --  ⑤ 只记了一句"这只眼里指不出它"的那条不是一件东西。病:S1A4 里没绑上的 reach right untilstuck 被记成一条框为 (0,0,0,0) 的记录,
   --  眼一转就解除,下一帧在画面左上角量出一块墙,此后 21 次名字绑到两块假东西上。
   --  🦷 Plan.Forget_When_Eye_Moves 恒答 False(眼转过以后照旧解除)⇒ 这一条红,重放里 S1A4 / S1A1 出现"绑错"
   Check (Plan.Forget_When_Eye_Moves (Rec ("reach right untilstuck", 1, Boxed => False, Seen => False, Blind => True))
          and then not Plan.Forget_When_Eye_Moves (Rec ("the mint green", 1, Boxed => True, Seen => False, Blind => True)),
          "指不出的记录:在这只眼里从来没有过框的,眼转过就删;有过框的只解除(框还在,按框重量)");
   --  🦷 Without_Eye 把没有过框的记录也当候选 ⇒ 这一条红:一句没绑上的话(green)不是以前说过的一件东西,
   --  不许把对的那一件(the mint green)搅成"对得上两件、不猜"
   declare
      Rs : Plan.Named_Vectors.Vector;
      V : Plan.Name_Verdict;
   begin
      Rs.Append (Rec ("green", 1, Boxed => False, Seen => False, Blind => True));
      Rs.Append (Rec ("the mint green", 0, Boxed => True, Seen => True));
      V := Plan.Without_Eye ("pick up the mint green", 1, Rs);
      Check (V.Kind = Plan.Nv_Elsewhere and then V.Index = 1 and then Index (V.Known, "「green」") = 0,
             "指不出的记录:「green」只记过一句指不出、从没有过框 ⇒ 不是以前说过的东西,不当候选;"
             & "「pick up the mint green」照样只对得上「the mint green」(" & Plan.Name_Verdict_Kind'Image (V.Kind) & ")");
   end;

   --  ⑥ 同一段程序里名字的绑法不随行的先后变。病:前一行的名字先问、没认出来(眼答这里没有),后一行的名字问眼认出一件新的,
   --  前一行按字本来就含着它 —— 可问它的时候那一件还没进清单 ⇒ 整段被退回。
   --  🦷 Plan.Rebind_Missing 什么都不做 ⇒ 这一条红
   declare
      Rs : Plan.Named_Vectors.Vector;
      B : Plan.Bind_Vectors.Vector;
      Got : Natural;
      function One_Item (Bx : Natural) return Natural is (Bx + 1);
   begin
      B.Append (Plan.Bind_Entry'(Key => U ("reach pick upmint green scissors"), Item => -1, Tried => U ("眼说这里没有")));
      B.Append (Plan.Bind_Entry'(Key => U ("mint green scissors"), Item => 1, Tried => Null_Unbounded_String));
      B.Append (Plan.Bind_Entry'(Key => U ("grasper"), Item => -1, Tried => Null_Unbounded_String));
      Rs.Append (Rec ("mint green scissors", 1, Boxed => True, Seen => True));
      Plan.Rebind_Missing (B, Rs, 1, One_Item'Access, Got);
      Check (Got = 1 and then B (0).Item = 1 and then B (2).Item = -1,
             "同一段程序:前一行「reach pick upmint green scissors」头一遍没绑上,整段认完按字它含着后一行认出来的「mint green scissors」⇒ 绑上;角色不归这一遍管");
   end;

   --  ⑧ LANGUAGE.md §17 的例子驱动都认。病:文档教的写法驱动不认(§12 里单独一行的 close grasper on ball、调用写成 do <名字>),
   --  照文档写的脑 / 用的人一句都编不过;"先 remember 再回来"那种写法以前在认名字那一步就被退回(地名被拿去问眼)。
   --  每个例子逐字在 LANGUAGE.md 里(自检在 driver/ 下跑,也找 driver/LANGUAGE.md),解析器全认;
   --  这段程序里 remember 起的地名编译期就认成地名(Plan.Remembered_Here),不问眼。
   --  🦷 把补救那个例子里的 do grasper close ball until stuck 改回 §12 的 close grasper on ball until stuck ⇒ 红;
   --  Plan.Remembered_Here 恒答 -1 ⇒ 红
   declare
      NL : constant String := "" & ASCII.LF;
      Examples : constant array (1 .. 5) of Unbounded_String :=
        [U ("do scissors height up until settled" & NL & "say I am lifting the scissors"),
         U ("do grasper still until settled or 20 steps"),
         U ("remember where grasper is as home" & NL & "do grasper touching home until touched or 20 steps"),
         U ("to pick up ball:" & NL
            & "  remember where grasper is as start" & NL
            & "  repeat 3 times:" & NL
            & "    do grasper touching ball until touched or 20 steps" & NL
            & "    do grasper close ball until stuck" & NL
            & "    do grasper farther ball until slipped or 10 steps" & NL
            & "    if slipped:" & NL
            & "      do grasper open until settled" & NL
            & "      do grasper touching start until touched or 20 steps" & NL
            & "    else:" & NL
            & "      done" & NL
            & "    end" & NL
            & "  end" & NL
            & "  say I tried three times and it is not in my hand" & NL
            & "end" & NL
            & "run pick up ball"),
         U ("do grasper close on ball until stuck")];
      Doc : Unbounded_String;
      Found_Doc : Boolean := False;
      procedure Read_Doc (Path : String) is
         F : Ada.Text_IO.File_Type;
      begin
         Ada.Text_IO.Open (F, Ada.Text_IO.In_File, Path);
         while not Ada.Text_IO.End_Of_File (F) loop
            Append (Doc, Ada.Text_IO.Get_Line (F) & NL);
         end loop;
         Ada.Text_IO.Close (F);
         Found_Doc := True;
      exception
         when others => null;
      end Read_Doc;
   begin
      Read_Doc ("LANGUAGE.md");
      if not Found_Doc then
         Read_Doc ("driver/LANGUAGE.md");
      end if;
      Check (Found_Doc, "LANGUAGE.md 读得到(自检在 driver/ 或仓库根目录下跑)");
      for K in Examples'Range loop
         declare
            Src : constant String := To_String (Examples (K));
            P : constant Sinew.Program := Sinew.Parse (Src);
         begin
            Check (P.Ok and then (not Found_Doc or else Index (Doc, Src) > 0),
                   "LANGUAGE.md §17 例子 " & Codec.Img (K) & " 逐字在文档里、解析器认"
                   & (if P.Ok then "" else "(退回:" & To_String (P.Err) & ")"));
         end;
      end loop;
      declare
         P3 : constant Sinew.Program := Sinew.Parse (To_String (Examples (3)));
         P4 : constant Sinew.Program := Sinew.Parse (To_String (Examples (4)));
      begin
         Check (Plan.Remembered_Here (P3, "home") >= 0 and then Plan.Remembered_Here (P4, "start") >= 0
                and then Plan.Remembered_Here (P4, "Start") >= 0 and then Plan.Remembered_Here (P4, "ball") < 0,
                "这段程序里 remember 起的地名(home、start,大小写不论)编译期就认成地名,不拿去问眼;ball 不是");
      end;
   end;

   --  ⑨ 一段怎么收尾 → 结局词,和脑等的那个词同一张表。病:以「差距连着几步不缩」收尾的一段(stall: …)被读成 refused,
   --  if stalled 永远不成立、repeat until stalled 永远出不去,在 try 里还被当成"没成"。
   --  🦷 Plan.Outcome_Of_Event 去掉 stall 那一行 ⇒ 红
   declare
      use type Sinew.Outcome;
      Round_Trip : Boolean := True;
      Missed : Unbounded_String;
   begin
      for O in Sinew.Outcome loop
         if O not in Sinew.Oc_None | Sinew.Oc_Refused | Sinew.Oc_Arrived | Sinew.Oc_Timeout
           and then Plan.Outcome_Of_Event (Act.Until_Word (O) & ": what my executor said") /= O
         then
            Round_Trip := False;
            Append (Missed, " " & Sinew.Outcome_Word (O));
         end if;
      end loop;
      Check (Round_Trip
             and then Plan.Outcome_Of_Event ("steps") = Sinew.Oc_Timeout
             and then Plan.Outcome_Of_Event ("amount: arrived (my eye now points there)") = Sinew.Oc_Arrived
             and then Plan.Outcome_Of_Event ("stall: I am still moving, but for several steps in a row the gap stopped shrinking") = Sinew.Oc_Stalled,
             "一段收尾那句话 → 结局词:脑能等的每个词(touched stuck slipped lost free settled stalled)执行器那句话都读回同一个词"
             & (if Length (Missed) > 0 then "(读错的:" & To_String (Missed) & ")" else ""));
   end;

   --  ⑦ 重放 S1A1–S1A5 落盘的每一轮(大并行 §5 路 7:S1A2–S1A4 落盘的轮次重放,粘在一起的名字都绑对)。
   --  每一轮:在哪只眼、脑的程序里按行的先后写了哪些名字、那只眼对每个名字怎么答(日志里的原话;日志里旧的认法没问眼就绑了的,
   --  拿那一轮落盘的画面、驱动一字不差的请求问过真 Qwen3.5-9B,10-01);脑说的是什么(打分用,判法看不见)。
   --  改之前的数(同一批名字,旧的认法,照日志数):
   --    S1A1 绑对 4 · 该绑上没绑上 2 · 本来就不是东西 3 · 绑错 1(我自己的胳膊);R8 同一把剪刀在同一只眼里记成三件
   --    S1A2 绑对 2 · 0 · 0 · 0
   --    S1A3 绑对 3 · 该绑上没绑上 1 · 0 · 0
   --    S1A4 绑对 3 · 该绑上没绑上 7 · 本来就不是东西 3 · 绑错 21(左腕眼里两块假东西)
   --    S1A5 绑对 4 · 0 · 本来就不是东西 1 · 0
   declare
      A : constant Answer := Here_Scissors;
      Arm : constant Answer := Here_Arm;
      No : constant Answer := Not_Here;
      S : constant Meaning := Means_Scissors;
      N : constant Meaning := Means_Nothing;
      Q : constant Meaning := Means_Unclear;
   begin
      --  S1A1:剪刀只在第 0 台(头顶眼)里看得见
      Start_Run;
      Visible := [0 => True, others => False];
      Round (0); Ask ("scissors", A, S); Ask ("arm reach ight", Arm, N); Score; Clear (1);
      Round (1); Ask ("arm reach the", No, N); Score;
      Round (0); Ask ("arm ride untilarm", No, N); Score;
      Round (1); Ask ("scissors", No, S); Score;
      Round (0); Ask ("scissors", A, S); Score; Clear (1);
      Round (1); Ask ("pick upuntil stuckscissors", No, S); Score;
      Round (0); Ask ("reach untilarm reachtheis", No, N); Ask ("reach untilpick upuntilstuckscissorsis", A, S);
      Ask ("pick upuntilstuckscissorsheight untilstuck", A, S); Score;
      Put_Line ("  重放 S1A1:" & Say (T));
      Check (T.Right = 6 and then T.Fail_Thing = 0 and then T.Fail_Honest = 3 and then T.Wrong = 1
             and then (for all M of Store => M.Id /= Id_Mark or else not M.R.Seen)
             and then Count_Of (Id_Scissors, 0) = 1,
             "重放 S1A1:剪刀的名字 6 个全绑对(R4 / R7 在看不见它的腕眼里按字绑上;R8 两种粘法都认成同一件,不再记成三件);"
             & "绑错 1 = R1 眼把我自己的右臂框成了「arm reach ight」(我自己的零件认不出来,要路 1 的每一节形状,见报告)");

      --  S1A2:第 2 轮起第 1 台(左腕眼)也看得见
      Start_Run;
      Visible := [0 => True, others => False];
      Round (0); Ask ("scissors", A, S); Score; Clear (1);
      Visible (1) := True;
      Round (1); Ask ("the mint green", A, S); Score;
      Put_Line ("  重放 S1A2:" & Say (T));
      Check (T.Right = 2 and then T.Fail_Thing = 0 and then T.Wrong = 0, "重放 S1A2:2 个名字全绑对");

      --  S1A3
      Start_Run;
      Visible := [0 => True, others => False];
      Round (0); Ask ("the mint green", A, S); Score; Clear (1);
      Visible (1) := True;
      Round (1); Ask ("reach left untilarmsalignedwithscissors", No, S); Ask ("lift scissors", A, S); Score;
      Round (1); Ask ("lift scissors", A, S); Score;
      Put_Line ("  重放 S1A3:" & Say (T));
      Check (T.Right = 3 and then T.Fail_Thing = 1 and then T.Wrong = 0,
             "重放 S1A3:3 个绑对;「reach left untilarmsalignedwithscissors」眼在看得见剪刀的那张图里也说没有(实炮和落盘重问都是),"
             & "字又不含任何起过的名字 ⇒ 照实说绑不上");

      --  S1A4:剪刀只在第 0 台里看得见(左腕眼整段没转到它)
      Start_Run;
      Visible := [0 => True, others => False];
      Round (0); Ask ("the mint green", A, S); Score; Clear (1);
      Round (1); Ask ("reach right untilstuck", No, N); Score;
      Round (0); Ask ("the mint green", A, S); Score; Clear (1);
      Round (1); Ask ("reach the mintgreenscissors", No, S); Ask ("lift the mintgreenscissors", No, S); Score;
      Round (0); Ask ("the mint green", A, S); Score; Clear (1);
      Round (1); Ask ("reach right untilstuck", No, N); Ask ("lift the mintgreenscissors", No, S);
      Ask ("saydone untildone untildone", No, N); Score;
      Round (1); Ask ("the mint greenscissors", No, S); Score;
      Round (1); Ask ("reach upcell untilstick", No, N); Ask ("pick upmint greenscissors", No, S); Score;
      Clear (0); Clear (2);   --  R18:say look = 2 / look = 3
      Round (2); Ask ("pick upmint greenscissors", No, S); Score;
      for Rn in 1 .. 4 loop   --  R22 R25 R29 R33
         Round (1); Ask ("pick upmint greenscissors", No, S); Score;
      end loop;
      Round (1); Ask ("reach right untilstuck", No, N); Ask ("reach upmints untilstuck", No, Q);
      Ask ("reach pick upmintgreenscissorsuntilst", No, S); Ask ("reach mint greenscissorsuntilstuck", No, S); Score;   --  R31
      for Rn in 1 .. 3 loop   --  R37 R40 R42
         Round (1); Ask ("pick upmint greenscissors", No, S); Score;
      end loop;
      Round (1); Ask ("reach upmints untilstuck", No, Q); Ask ("reach pick upmintgreenscissorsuntilst", No, S); Score;   --  R45
      Round (1); Ask ("reach upmints untilstuck", No, Q); Ask ("reach pick upmintgreenscissors", No, S); Score;          --  R47
      Round (1); Ask ("pick upmint greenscissors", No, S); Score;   --  R49
      Round (1); Ask ("reach upmints untilstuck", No, Q); Score;    --  R52
      Round (1); Ask ("reach upmints untilstuck", No, Q); Ask ("pick upmint greenscissors", No, S); Score;   --  R56
      for Rn in 1 .. 2 loop   --  R60 R63
         Round (1); Ask ("pick upmint greenscissors", No, S); Score;
      end loop;
      Put_Line ("  重放 S1A4:" & Say (T));
      Check (T.Right = 7 and then T.Fail_Thing = 17 and then T.Fail_Honest = 10 and then T.Wrong = 0
             and then (for all M of Store => M.Id /= Id_Mark or else not M.R.Seen),
             "重放 S1A4:绑错 21 → 0(左腕眼里不再有假东西);含着「the mint green」的粘词名字 4 个在看不见剪刀的腕眼里按字绑到头顶眼里那一件;"
             & "「pick upmint greenscissors」一族不含它、腕眼又看不见剪刀 ⇒ 照实说绑不上(回头顶眼那只眼能框出来,见报告)");

      --  S1A5:第 2 轮起左腕眼看得见;第 3 轮换了一集(世界记忆清空,身体留着)
      Start_Run;
      Visible := [0 => True, others => False];
      Round (0); Ask ("scissors", A, S); Score; Clear (1);
      Visible (1) := True;
      Round (1); Ask ("scissors upuntil toucheduntil", A, S); Score;
      Store.Clear;
      Round (0); Ask ("reach ouch downuntillifteduntil", No, N); Score;
      Round (1); Ask ("pick upmint greenscissors", A, S); Score;
      Round (1); Ask ("pick upmint greenscissors", A, S); Score;
      Put_Line ("  重放 S1A5:" & Say (T));
      Check (T.Right = 4 and then T.Fail_Honest = 1 and then T.Wrong = 0, "重放 S1A5:4 个绑对,1 个本来就不是东西");
   end;
end Welds_Path_7;
