with Codec;
with Http_Client;
with Json;
package body Instrument is
   --  仪器回的 points = [[u,v,vis,conf],...]
   function Parse_Pts (D : Json.Doc; N : Integer) return Track_Vectors.Vector is
      V : Track_Vectors.Vector;
   begin
      if N >= 0 then
         for I in 0 .. Json.Count (D, N) - 1 loop
            declare
               Pn : constant Integer := Json.Child (D, N, I);
               P : Track_Pt;
            begin
               if Pn >= 0 and then Json.Count (D, Pn) >= 4 then
                  P.U := Json.Num (D, Json.Child (D, Pn, 0));
                  P.V := Json.Num (D, Json.Child (D, Pn, 1));
                  P.Seen := Json.Num (D, Json.Child (D, Pn, 2)) > 0.5;   --  0/1 标志(无量纲)
                  P.Conf := Json.Num (D, Json.Child (D, Pn, 3));
               end if;
               V.Append (P);
            end;
         end loop;
      end if;
      return V;
   end Parse_Pts;

   --  发一次、读回 points;Ok = 仪器说 ok
   function Track_Post (Host : String; Port : Natural; Path, Body_Json : String; D : out Json.Doc; Err : out Unbounded_String) return Boolean is
      Reply, Jerr : Unbounded_String;
   begin
      Err := Null_Unbounded_String;
      if Host = "" or else Port = 0 then
         Err := To_Unbounded_String ("没配仪器");
         return False;
      end if;
      if not Http_Client.Post (Host, Port, Path, Body_Json, Reply) then
         Err := To_Unbounded_String ("连不上仪器 " & Host & ":" & Codec.Img (Port));
         return False;
      end if;
      if not Json.Parse (To_String (Reply), D, Jerr) then
         Err := To_Unbounded_String ("仪器回的不是 JSON:" & To_String (Jerr));
         return False;
      end if;
      declare
         Okn : constant Integer := Json.Get (D, 0, "ok");
         En : constant Integer := Json.Get (D, 0, "err");
      begin
         if Okn < 0 or else not Json.Bool (D, Okn) then
            Err := To_Unbounded_String ("仪器说不行" & (if En >= 0 then ":" & Json.Text (D, En) else ""));
            return False;
         end if;
      end;
      return True;
   end Track_Post;

   function Track_Start (Host : String; Port : Natural; RGB : Buf; W, H : Natural; Pts : Track_Vectors.Vector;
                         Id : out Integer; Err : out Unbounded_String) return Track_Vectors.Vector is
      D : Json.Doc;
      Q : Unbounded_String;
      Empty : Track_Vectors.Vector;
   begin
      Id := -1;
      if W = 0 or else H = 0 or else Pts.Is_Empty then
         Err := To_Unbounded_String ("没有点可跟");
         return Empty;
      end if;
      for I in 0 .. Natural (Pts.Length) - 1 loop
         Append (Q, (if I > 0 then "," else "") & "[" & Codec.Fmt (Pts (I).U, 2) & "," & Codec.Fmt (Pts (I).V, 2) & "]");
      end loop;
      if not Track_Post (Host, Port, "/track/start", "{""image"":""" & Codec.Base64 (Codec.BMP24 (RGB, W, H)) & """,""points"":[" & To_String (Q) & "]}", D, Err) then
         return Empty;
      end if;
      declare
         Idn : constant Integer := Json.Get (D, 0, "id");
      begin
         if Idn < 0 or else not Json.Is_Num (D, Idn) then
            Err := To_Unbounded_String ("仪器没给段号");
            return Empty;
         end if;
         Id := Integer (Json.Num (D, Idn));
      end;
      return Parse_Pts (D, Json.Get (D, 0, "points"));
   end Track_Start;

   function Track_Step (Host : String; Port : Natural; Id : Integer; RGB : Buf; W, H : Natural; Err : out Unbounded_String) return Track_Vectors.Vector is
      D : Json.Doc;
      Empty : Track_Vectors.Vector;
   begin
      if Id < 0 or else W = 0 or else H = 0 then
         Err := To_Unbounded_String ("没有开着的跟踪段");
         return Empty;
      end if;
      if not Track_Post (Host, Port, "/track/step", "{""id"":" & Codec.Img (Id) & ",""image"":""" & Codec.Base64 (Codec.BMP24 (RGB, W, H)) & """}", D, Err) then
         return Empty;
      end if;
      return Parse_Pts (D, Json.Get (D, 0, "points"));
   end Track_Step;

   procedure Track_End (Host : String; Port : Natural; Id : Integer) is
      D : Json.Doc;
      Err : Unbounded_String;
   begin
      if Id >= 0 and then Track_Post (Host, Port, "/track/end", "{""id"":" & Codec.Img (Id) & "}", D, Err) then
         null;
      end if;
   end Track_End;
end Instrument;
