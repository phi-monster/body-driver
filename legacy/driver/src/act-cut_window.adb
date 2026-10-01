separate (Act)
function Cut_Window (C : Context; Cam : Natural; F : Plug.Frame) return Long_Float is
   A : constant Integer := Cam_Arm (C, Cam);
begin
   --  长在手上的相机斜看桌面:窗口 = 两指张幅按"指深 / 画面中位深"缩到画面深处("比这还大的不是能拿的东西");世界相机用画幅八分之一
   --  下面的 0.02 / 0.125 都是画幅的比例(无量纲):窗口的下限与上限
   if A >= 0 then
      declare
         Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cam);
         Dp : Floats := F.Cams (Cam).Depth;
         Med : Long_Float;
      begin
         --  窗口至少要比正在跟的那块大半圈(倍数,无量纲),否则它一走近就被闭运算填平、只剩一圈边
         --  ⚠️ 试过"窗口跟着被跟的东西放大",错的:窗口一到画幅四分之一,桌面自己的起伏就盖过物体,
         --  深度那一路彻底切不出东西,颜色那一路接管、把墙缝和衣服切成上百块(EZ 逐步落图坐实)。窗口保持小。
         if Z.Valid and then Z.Span > 0.0 and then not Picture.Is_Nan (Z.Depth) and then F.Cams (Cam).Has_Depth then
            declare
               Samp : Floats;
               I : Natural := 0;
            begin
               while I < Natural (Dp.Length) loop
                  if not Picture.Is_Nan (Dp (I)) and then Dp (I) > 0.0 then
                     Samp.Append (Dp (I));
                  end if;
                  I := I + 37;
               end loop;
               if Natural (Samp.Length) >= 16 then
                  Med := Picture.Quantile (Samp, 0.5);
                  if Med > 0.0 then
                     --  夹在画幅的 0.02 与 0.125 之间(比例,无量纲)
                     return Long_Float'Max (0.02, Long_Float'Min (0.125, Z.Span * (Z.Depth / Med) * 0.5));
                  end if;
               end if;
            end;
         end if;
      end;
   end if;
   return 0.125;   --  世界相机:画幅八分之一(比例,无量纲)
end Cut_Window;
