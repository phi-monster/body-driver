separate (Act)
function Build_Facts (C : Context; F : Plug.Frame) return Plan.Facts_Vectors.Vector is
   Fs : Plan.Facts_Vectors.Vector;
   Zero : Plan.Item_Facts;
begin
   Fs.Append (Zero);
   for I in 0 .. Natural (C.Items.Length) - 1 loop
      declare
         It : constant Item := C.Items (I);
         Ft : Plan.Item_Facts;
         Kk : constant Natural := (if It.Kind in Finger | Grip then Chan.Per_Arm + It.Jaw_K else It.Which);
      begin
         Ft.Exists := It.Located or else It.Kind in Finger | Grip | Piece;
         Ft.Mine := It.Kind in Finger | Grip | Piece;
         Ft.Grasp := It.Kind = Grip;
         Ft.Arm := It.Arm;
         Ft.Thing_Idx := -1;
         --  量得出它鼓出它站的那个面多少 ⇒ 才有"那个面"可言。面不是全局开关,是每个东西自己的事。
         Ft.Stands := It.Height > 0.0;
         Ft.Jaw_K := It.Jaw_K;
         --  张得开多少 / 这一块多宽:空转拿它判"合下去是不是必然空的"
         Ft.Span := (if It.Kind = Grip then Zone_Of (C, It.Arm, C.Cam, It.Jaw_K).Span else 0.0);
         Ft.Size := Long_Float'Max (Long_Float (It.X1 - It.X0) / Long_Float (Natural'Max (1, Cw_Of (C, F))),
                                    Long_Float (It.Y1 - It.Y0) / Long_Float (Natural'Max (1, Ch_Of (C, F))));
         Ft.Label := To_Unbounded_String
           ((case It.Kind is
                when Grip => "grasper(第" & Codec.Img (It.Arm + 1) & " 只手第" & Codec.Img (It.Jaw_K) & " 组)",
                when Finger => "grasper 的一瓣",
                when Piece => "第" & Codec.Img (It.Arm + 1) & " 只手" & Codec.Img (It.Which) & " 轴带的那一块",
                when others => ""));
         if Ft.Mine then
            for T in 0 .. Natural (C.Tables.Length) - 1 loop
               if C.Tables (T).Arm = It.Arm and then C.Tables (T).Cam = C.Cam
                 and then C.Tables (T).Chan_K = Kk
               then
                  Ft.Thing_Idx := Integer (T);
                  exit;
               end if;
            end loop;
         end if;
         Fs.Append (Ft);
      end;
   end loop;
   return Fs;
end Build_Facts;
