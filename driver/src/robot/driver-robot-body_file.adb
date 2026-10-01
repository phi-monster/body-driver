with Ada.Characters.Handling;
with Ada.Text_IO;
with Driver.Json;
with Driver.Log;
with Driver.Robot.Channels;
with Driver.Robot.Graph;

package body Driver.Robot.Body_File is

   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;

   function Num (X : Real) return String renames Driver.Json.Number_Image;

   function Int (N : Integer) return String renames Driver.Log.Image;

   function Word (Image : String) return String is
     (Driver.Json.Quote (Ada.Characters.Handling.To_Lower (Image)));

   function Flag (B : Boolean) return String is (if B then "true" else "false");

   function Text (M : Model) return String is
      T : Unbounded_String;
   begin
      Append (T, "{""key"": {""groups"": [");
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         Append (T, (if G > M.Groups.First_Index then ", " else "")
                 & "{""size"": " & Int (M.Groups (G).Size)
                 & ", ""commandable"": " & Flag (M.Groups (G).Commandable) & "}");
      end loop;
      Append (T, "], ""eyes"": [");
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         Append (T, (if E > M.Eyes.First_Index then ", " else "")
                 & "{""width"": " & Int (M.Eyes (E).Grid.Width)
                 & ", ""height"": " & Int (M.Eyes (E).Grid.Height) & "}");
      end loop;
      Append (T, "]}," & ASCII.LF);
      Append (T, " ""beats"": " & Int (M.Beats) & "," & ASCII.LF);
      Append (T, " ""groups"": [");
      for G in M.Groups.First_Index .. M.Groups.Last_Index loop
         Append (T, (if G > M.Groups.First_Index then "," & ASCII.LF & "  " else ASCII.LF & "  ")
                 & "{""role"": {""value"": " & Word (Group_Role'Image (Role (M, G)))
                 & ", ""method"": " & Int (Graph_Method) & "}"
                 & ", ""arm"": " & Int (if G <= M.Graph.Arm_Of.Last_Index then Integer (M.Graph.Arm_Of (G)) else 0)
                 & ", ""contract_breach"": " & Int (Contract_Breach (M, G))
                 & ", ""noise"": {""method"": " & Int (Noise_Method) & ", ""sigma"": [");
         for C in 1 .. M.Groups (G).Size loop
            Append (T, (if C > 1 then ", " else "") & Num (Channels.Noise (M, G, C)));
         end loop;
         Append (T, "]}}");
      end loop;
      Append (T, "]," & ASCII.LF & " ""eyes"": [");
      for E in M.Eyes.First_Index .. M.Eyes.Last_Index loop
         declare
            Mt : constant Mount := Eye_Mount (M, E);
         begin
            Append (T, (if E > M.Eyes.First_Index then "," & ASCII.LF & "  " else ASCII.LF & "  ")
                    & "{""lag"": {""value"": " & Int (Image_Lag (M, E)) & ", ""method"": " & Int (Lag_Method) & "}"
                    & ", ""mount"": {""kind"": " & Word (Mount_Kind'Image (Mt.Kind))
                    & ", ""arm"": " & Int (if Mt.Kind = Arm_Carried then Integer (Mt.Arm) else 0)
                    & ", ""method"": " & Int (Graph_Method) & "}"
                    & ", ""effects"": {""method"": " & Int (Lockin_Method) & ", ""by_group"": [");
            for G in M.Groups.First_Index .. M.Groups.Last_Index loop
               declare
                  F : constant Eye_Effect := Graph.Effect (M, G, E);
               begin
                  Append (T, (if G > M.Groups.First_Index then ", " else "")
                          & "{""verdict"": " & Word (Eye_Response'Image (F.Verdict))
                          & ", ""responding"": " & Int (F.Responding) & ", ""textured"": " & Int (F.Textured)
                          & ", ""fraction"": " & Num (F.Fraction.Value)
                          & ", ""sigma"": " & Num (F.Fraction.Sigma) & "}");
               end;
            end loop;
            Append (T, "]}}");
         end;
      end loop;
      Append (T, "]}" & ASCII.LF);
      return To_String (T);
   end Text;

   procedure Write (M : Model; Path : String; Ok : out Boolean) is
      F : Ada.Text_IO.File_Type;
   begin
      Ada.Text_IO.Create (F, Ada.Text_IO.Out_File, Path);
      Ada.Text_IO.Put (F, Text (M));
      Ada.Text_IO.Close (F);
      Ok := True;
   exception
      when Ada.Text_IO.Name_Error | Ada.Text_IO.Use_Error =>
         Ok := False;
   end Write;

end Driver.Robot.Body_File;
