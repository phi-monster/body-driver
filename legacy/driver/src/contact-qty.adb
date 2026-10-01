with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
with Stats;
package body Contact.Qty is
   use Ada.Strings.Unbounded;

   function Sub (A, B : V3) return V3 is ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]);
   function Scl (K : Long_Float; A : V3) return V3 is ([K * A (0), K * A (1), K * A (2)]);
   --  放平到面里:去掉沿 Up 的那一份
   function Flat (A, Up : V3) return V3 is (Sub (A, Scl (Dot (A, Up), Up)));

   function Moved_Off (Before, After : V3; Sd_Before, Sd_After : Long_Float) return Boolean is
   begin
      if not Sd_Before'Valid or else not Sd_After'Valid or else Sd_Before < 0.0 or else Sd_After < 0.0 then
         return True;
      end if;
      return Norm (Sub (After, Before)) > Stats.Z * Sqrt (Sd_Before ** 2 + Sd_After ** 2);
   end Moved_Off;

   function Rest_Leg (S : Scene) return Leg is
      Ou : Boolean;
      Up : constant V3 := Unit (S.Up, Ou);
   begin
      if not Ou or else not S.Has_Center or else not S.Has_Ref or else not S.Has_Ref_Top or else not S.Has_Bottom then
         return Unknown;
      end if;
      declare
         Gate : constant Long_Float := Stats.Z * S.Sd;
         Over_It : constant Boolean := Norm (Flat (Sub (S.Center, S.Ref), Up)) <= Gate;
      begin
         if Over_It then
            if abs (S.Bottom - S.Ref_Top) <= Gate then
               return Resting;
            elsif S.Bottom > S.Ref_Top then
               return Down;
            end if;
            return Up_First;
         elsif S.Bottom > S.Ref_Top + Gate then
            return Over;
         end if;
         return Up_First;
      end;
   end Rest_Leg;

   procedure Motion (K : Kind; Dir : Integer; S : Scene; M : out Twist; Ok : out Boolean; Note : out Unbounded_String) is
      Ou : Boolean;
      Up : constant V3 := Unit (S.Up, Ou);
      Sg : constant Long_Float := (if Dir < 0 then -1.0 else 1.0);
      procedure Missing (What : String) is
      begin
         Ok := False;
         Note := To_Unbounded_String ("I have not measured " & What);
      end Missing;
      --  平移,方向 D(已是单位向量);它贴着它躺的面时往面里去的那一份去掉(面挡着,只在面里走),去完不剩 ⇒ 照实说
      procedure Slide_Along (D : V3) is
         Lying : constant Boolean := S.Has_Bottom and then S.Bottom <= Stats.Z * S.Sd;
         Dd : V3 := D;
      begin
         if Lying and then Dot (D, Up) < 0.0 then
            declare
               Of_Flat : Boolean;
               Fd : constant V3 := Unit (Flat (D, Up), Of_Flat);
            begin
               if not Of_Flat then
                  Ok := False;
                  Note := To_Unbounded_String ("it lies on its surface and the only way to change that goes straight into the surface");
                  return;
               end if;
               Dd := Fd;
               Note := To_Unbounded_String ("it lies on its surface, so it goes along the surface (the part of that way going into the surface is left out)");
            end;
         end if;
         M := Slide (Dd);
         Ok := True;
      end Slide_Along;
      --  绕过它中心的一根轴转(旋量只看方向和绕哪儿转,转多远归执行层:模长取 1)
      procedure Rotate_About (Axis : V3) is
      begin
         M := Rotation (Axis, 1.0, S.Center, Ok);
      end Rotate_About;
   begin
      M := Still (S.Center);
      Ok := False;
      Note := Null_Unbounded_String;
      if not Ou then
         Missing ("the surface it lies on, so I do not know which way is up from it");
         return;
      end if;
      case K is
         when Height | Rise =>
            --  量到的法向原样用(它本来就是单位向量;再归一一遍末位会变,和 09-23 起"沿面的法向走一个单位"那一条就不是同一个数了)
            M := Slide (Scl (Sg, S.Up));
            Ok := True;
         when Heading =>
            if not S.Has_Center then
               Missing ("where its centre is");
               return;
            end if;
            Rotate_About (Scl (Sg, Up));
         when Tilt =>
            if not S.Has_Center then
               Missing ("where its centre is");
               return;
            elsif not S.Has_Me then
               Missing ("it from any eye of mine that stays still while my arm moves it");
               return;
            end if;
            declare
               Oh : Boolean;
               H : constant V3 := Unit (Flat (Sub (S.Center, S.Me), Up), Oh);
            begin
               if not Oh then
                  Ok := False;
                  Note := To_Unbounded_String ("it is straight under my still eye, so no direction along the surface points away from me");
                  return;
               end if;
               Rotate_About (Scl (Sg, Cross (Up, H)));
            end;
         when Across =>
            if not S.Has_View then
               Missing ("the eye you are looking through");
               return;
            end if;
            declare
               Or2 : Boolean;
               R : constant V3 := Unit (Flat (S.View_Right, Up), Or2);
            begin
               if not Or2 then
                  Ok := False;
                  Note := To_Unbounded_String ("the eye you are looking through looks along the up of its surface, so its left and right do not lie in the surface");
                  return;
               end if;
               Slide_Along (Scl (Sg, R));
            end;
         when Gap | Rest_On | Aim =>
            if not S.Has_Center then
               Missing ("where its centre is");
               return;
            elsif not S.Has_Ref then
               Missing ("where the other thing is");
               return;
            end if;
            case K is
               when Gap =>
                  declare
                     Od : Boolean;
                     D : constant V3 := Unit (Sub (S.Ref, S.Center), Od);
                  begin
                     if not Od then
                        Ok := False;
                        Note := To_Unbounded_String ("its centre and the other thing's centre are the same point as far as I can measure");
                        return;
                     end if;
                     Slide_Along (Scl (-Sg, D));
                  end;
               when Rest_On =>
                  case Rest_Leg (S) is
                     when Up_First =>
                        M := Slide (Up);
                        Ok := True;
                        Note := To_Unbounded_String ("its bottom is not yet clear of the other thing's top, so it goes up first");
                     when Over =>
                        declare
                           Oo : Boolean;
                           D : constant V3 := Unit (Flat (Sub (S.Ref, S.Center), Up), Oo);
                        begin
                           M := Slide (D);
                           Ok := Oo;
                           Note := To_Unbounded_String ("it is clear of the other thing's top, so it goes along the surface to straight above it");
                        end;
                     when Down =>
                        M := Slide (Scl (-1.0, Up));
                        Ok := True;
                        Note := To_Unbounded_String ("it is straight above the other thing, so it goes down onto its top");
                     when Resting =>
                        M := Still (S.Center);
                        Ok := True;
                        Note := To_Unbounded_String ("it is already on the other thing's top, as near as I can measure");
                     when Unknown =>
                        Missing ("how high the other thing's top is or how high its own bottom is");
                  end case;
               when Aim =>
                  if not S.Has_Axis then
                     Missing ("which way it is long (its outline is as wide one way as the other)");
                     return;
                  end if;
                  declare
                     Ot, Oa : Boolean;
                     T : constant V3 := Unit (Flat (Sub (S.Ref, S.Center), Up), Ot);
                     A0 : constant V3 := Unit (Flat (S.Axis, Up), Oa);
                  begin
                     if not Ot or else not Oa then
                        Ok := False;
                        Note := To_Unbounded_String ("the other thing is straight above or below it, or its long way stands along the up of its surface");
                        return;
                     end if;
                     declare
                        A : constant V3 := (if Dot (A0, T) < 0.0 then Scl (-1.0, A0) else A0);   --  长轴两头都算:取离那一件近的那头
                        Phi : constant Long_Float := Arctan (Dot (Cross (A, T), Up), Dot (A, T));
                     begin
                        if abs Phi <= Stats.Z * S.Ang_Sd then
                           M := Still (S.Center);
                           Ok := True;
                           Note := To_Unbounded_String ("its long way already points at the other thing, as near as I can measure");
                        else
                           Rotate_About ((if Phi > 0.0 then Up else Scl (-1.0, Up)));
                        end if;
                     end;
                  end;
               when others =>
                  null;
            end case;
      end case;
   end Motion;
end Contact.Qty;
