with Codec;
with Runtime;
package body Plan is

   use type Sinew.Rel;
   use type Sinew.Noun_Kind;
   use type Sinew.Op;
   use type Sinew.Outcome;

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

   --  这一版在这只眼里说得出口的关系。手上那只眼里手不动:贴面(onto/off)要知道它靠着的面朝哪儿,这只眼判不了;
   --  更近/更远在这只眼里 = 让那东西的远近读数变一截(这只眼跟着手走),说得了
   function Rel_Ok_Here (R : Sinew.Rel; Own_Eye : Boolean) return Boolean is
   begin
      case R is
         when Sinew.Re_Touching | Sinew.Re_Into | Sinew.Re_Above | Sinew.Re_Below | Sinew.Re_Left | Sinew.Re_Right
            | Sinew.Re_Nearer | Sinew.Re_Farther
            | Sinew.Re_Clear | Sinew.Re_Still | Sinew.Re_Close | Sinew.Re_Open =>
            return True;
         when Sinew.Re_Onto | Sinew.Re_Off =>
            return not Own_Eye;
         when Sinew.Re_Facing | Sinew.Re_Press | Sinew.Re_None =>
            return False;
      end case;
   end Rel_Ok_Here;

   function Usable_Rels (Own_Eye : Boolean) return String is
      S : Unbounded_String;
   begin
      for Rl in Sinew.Rel loop
         if Rl /= Sinew.Re_None and then Rel_Ok_Here (Rl, Own_Eye) then
            Append (S, (if Length (S) > 0 then " " else "") & Sinew.Rel_Word (Rl));
         end if;
      end loop;
      return To_String (S);
   end Usable_Rels;

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

   function Check (P : Sinew.Program; Facts : Facts_Vectors.Vector; B : Bind_Vectors.Vector; Own_Eye : Boolean) return Verdict is
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
               Reject (Line, "我认不出「" & Key_Of (N) & "」" & (if T = "" then "" else " —— " & T),
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
            if I.O = Sinew.Op_Remember then
               null;   --  记地方:执行到这一行时把指尖此刻的位置存进 Spots,之后那个名字当 <what> 用
            elsif I.O = Sinew.Op_Interval then
               --  结局词:这一版身体量得到的事件是 碰到 / 顶住 / 滑了 / 画面不变 / 差距不缩 / 步数用完;
               --  free 只在合手那一节有意义(合完抬一截,它跟着我走了 = 它离开了那个面)
               declare
                  Has_Close : Boolean := False;
               begin
                  for Ci in 0 .. Natural (I.Cons.Length) - 1 loop
                     if I.Cons (Ci).R = Sinew.Re_Close then
                        Has_Close := True;
                     end if;
                  end loop;
                  if I.Until_Oc = Sinew.Oc_Arrived then
                     Reject (I.Line, "「until arrived」我说不出口 —— 到没到只有你能判,我只量得到事件",
                             "写 or N steps:我走完 N 推就回来把画面给你看;想推到碰上写 until touched,推到推不动写 until stuck");
                  elsif I.Until_Oc = Sinew.Oc_Refused then
                     Reject (I.Line, "「until refused」我说不出口 —— refused 是我做不到时回给你的话,不是我能等来的事",
                             "想推到推不动写 until stuck;想限步数写 or N steps");
                  elsif I.Until_Oc = Sinew.Oc_Lost then
                     Reject (I.Line, "「until lost」不用等:我跟丢了会自己停下来告诉你",
                             "换成 until touched / until stuck / or N steps");
                  elsif I.Until_Oc = Sinew.Oc_Free and then not Has_Close then
                     Reject (I.Line, "「until free」只在合手那一节有意义:合完我抬一截,它跟着我的手走了才算离开了那个面",
                             "写成 do grasper close <东西> until free");
                  end if;
               end;
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
                     if not Rel_Ok_Here (C.R, Own_Eye) then
                        Reject (I.Line, "「" & Sinew.Rel_Word (C.R) & "」(" & Sinew.Rel_Cn (C.R) & ")这一版"
                                & (if Own_Eye and then C.R in Sinew.Re_Onto | Sinew.Re_Off
                                   then "在跟着我动的这只眼里判不了(它靠着的面朝哪儿这只眼看不出来)"
                                   else "还做不了"),
                                "我在这只眼里说得出口的关系:" & Usable_Rels (Own_Eye)
                                & (if Own_Eye then ";要抬起,说 farther <旁边一个看得见的东西>;要贴面,加 with my still eye" else ""));
                        exit;
                     end if;
                     if C.Obj.K /= Sinew.Nk_None then
                        if not Resolve (C.Obj, I.Line, "【和谁】", Obj) then
                           exit;
                        end if;
                        if C.R in Sinew.Re_Onto | Sinew.Re_Off | Sinew.Re_Into and then not Facts (Obj).Stands then
                           Reject (I.Line, "我量不出「" & Key_Of (C.Obj) & "」鼓出它靠着的那个面多少,所以说不了 " & Sinew.Rel_Word (C.R),
                                   "换成 touching,或者先让我在不动的那只眼里看清它");
                           exit;
                        end if;
                     end if;
                  end;
               end loop;
            end if;
         end;
      end loop;
      return V;
   end Check;

   function Dry_Run (P : Sinew.Program; Facts : Facts_Vectors.Vector; B : Bind_Vectors.Vector) return Verdict is
      use Sinew;
      V : Verdict;
      M : Runtime.Machine;
      W : Runtime.Yield;
      I : Sinew.Instr;
      Segments : Natural := 0;

      --  这一节里有没有"心里就走不通"的事:张不到那么开却要去合它
      function Impossible (Ins : Sinew.Instr) return String is
      begin
         for Ci in 0 .. Natural (Ins.Cons.Length) - 1 loop
            declare
               C : constant Constraint := Ins.Cons (Ci);
               Sub : constant Integer := Look_Up (B, C.Subj);
               Obj : constant Integer := Look_Up (B, C.Obj);
            begin
               if C.R = Re_Close and then Sub > 0 and then Sub < Integer (Facts.Length)
                 and then Obj > 0 and then Obj < Integer (Facts.Length)
                 and then Facts (Natural (Sub)).Span > 0.0
                 and then Facts (Natural (Obj)).Size > Facts (Natural (Sub)).Span
               then
                  return "我张得开 " & Codec.Fmt (Facts (Natural (Sub)).Span, 3)
                    & " 幅,而它有 " & Codec.Fmt (Facts (Natural (Obj)).Size, 3)
                    & " 幅那么宽 —— 合下去也是空的";
               end if;
            end;
         end loop;
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
