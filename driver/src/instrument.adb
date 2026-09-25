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

   function Match (Host : String; Port : Natural; RGB_A : Buf; W_A, H_A : Natural; RGB_B : Buf; W_B, H_B : Natural;
                   Pts : Match_Vectors.Vector; Err : out Unbounded_String) return Match_Vectors.Vector is
      Empty, Res : Match_Vectors.Vector;
      Req, Reply, Jerr : Unbounded_String;
      D : Json.Doc;
   begin
      Err := Null_Unbounded_String;
      if Host = "" or else Port = 0 then
         Err := To_Unbounded_String ("没配仪器");
         return Empty;
      end if;
      if W_A = 0 or else H_A = 0 or else W_B = 0 or else H_B = 0 or else Pts.Is_Empty then
         Err := To_Unbounded_String ("没有图或没有点可配");
         return Empty;
      end if;
      --  请求体在堆上拼(两张图 ≈ 2.5 MB)
      Append (Req, "{""a"":""");
      Append (Req, Codec.Base64 (Codec.BMP24 (RGB_A, W_A, H_A)));
      Append (Req, """,""b"":""");
      Append (Req, Codec.Base64 (Codec.BMP24 (RGB_B, W_B, H_B)));
      Append (Req, """,""num"":0,""points"":[");
      for I in 0 .. Natural (Pts.Length) - 1 loop
         Append (Req, (if I > 0 then "," else "") & "[" & Codec.Fmt (Pts (I).U, 2) & "," & Codec.Fmt (Pts (I).V, 2) & "]");
      end loop;
      Append (Req, "]}");
      if not Http_Client.Post (Host, Port, "/match", Req, Reply) then
         Err := To_Unbounded_String ("连不上仪器 " & Host & ":" & Codec.Img (Port));
         return Empty;
      end if;
      if not Json.Parse (To_String (Reply), D, Jerr) then
         Err := To_Unbounded_String ("仪器回的不是 JSON:" & To_String (Jerr));
         return Empty;
      end if;
      declare
         Okn : constant Integer := Json.Get (D, 0, "ok");
         En : constant Integer := Json.Get (D, 0, "err");
         Pn : constant Integer := Json.Get (D, 0, "points");
      begin
         if Okn < 0 or else not Json.Bool (D, Okn) then
            Err := To_Unbounded_String ("仪器说不行" & (if En >= 0 then ":" & Json.Text (D, En) else ""));
            return Empty;
         end if;
         if Pn < 0 or else Json.Count (D, Pn) /= Natural (Pts.Length) then
            Err := To_Unbounded_String ("仪器回的点数对不上");
            return Empty;
         end if;
         for I in 0 .. Json.Count (D, Pn) - 1 loop
            declare
               Qn : constant Integer := Json.Child (D, Pn, I);
               M : Match_Pt;
            begin
               if Qn < 0 or else Json.Count (D, Qn) < 3 then
                  Err := To_Unbounded_String ("仪器回的点不是 [u,v,可信度]");
                  return Empty;
               end if;
               M.U := Json.Num (D, Json.Child (D, Qn, 0));
               M.V := Json.Num (D, Json.Child (D, Qn, 1));
               M.Cert := Json.Num (D, Json.Child (D, Qn, 2));
               Res.Append (M);
            end;
         end loop;
      end;
      return Res;
   end Match;
   procedure Segment (Host : String; Port : Natural; RGB : Buf; W, H : Natural; X0, Y0, X1, Y1 : Integer; Pts : Seg_Pt_Vectors.Vector;
                      Mask : out Bools; Area : out Natural; Score : out Long_Float; Ok : out Boolean; Err : out Unbounded_String) is
      Req, Reply, Jerr : Unbounded_String;
      D : Json.Doc;
   begin
      Mask.Clear; Area := 0; Score := 0.0; Ok := False; Err := Null_Unbounded_String;
      if Host = "" or else Port = 0 then
         Err := To_Unbounded_String ("没配仪器");
         return;
      end if;
      if W = 0 or else H = 0 or else (X1 < X0 and then Pts.Is_Empty) then
         Err := To_Unbounded_String ("没有图,或既没有框也没有点");
         return;
      end if;
      Append (Req, "{""image"":""");
      Append (Req, Codec.Base64 (Codec.BMP24 (RGB, W, H)));
      Append (Req, """");
      if X1 >= X0 then
         Append (Req, ",""box"":[" & Codec.Img (Natural'Max (0, X0)) & "," & Codec.Img (Natural'Max (0, Y0)) & "," & Codec.Img (Natural'Max (0, X1)) & ","
                 & Codec.Img (Natural'Max (0, Y1)) & "]");
      end if;
      if not Pts.Is_Empty then
         Append (Req, ",""points"":[");
         for I in 0 .. Natural (Pts.Length) - 1 loop
            Append (Req, (if I > 0 then "," else "") & "[" & Codec.Fmt (Pts (I).U, 2) & "," & Codec.Fmt (Pts (I).V, 2) & "," & (if Pts (I).On then "1" else "0") & "]");
         end loop;
         Append (Req, "]");
      end if;
      Append (Req, "}");
      if not Http_Client.Post (Host, Port, "/segment", Req, Reply) then
         Err := To_Unbounded_String ("连不上仪器 " & Host & ":" & Codec.Img (Port));
         return;
      end if;
      if not Json.Parse (To_String (Reply), D, Jerr) then
         Err := To_Unbounded_String ("仪器回的不是 JSON:" & To_String (Jerr));
         return;
      end if;
      declare
         Okn : constant Integer := Json.Get (D, 0, "ok");
         En : constant Integer := Json.Get (D, 0, "err");
         Rn : constant Integer := Json.Get (D, 0, "runs");
         Cur : Boolean := False;
      begin
         if Okn < 0 or else not Json.Bool (D, Okn) then
            Err := To_Unbounded_String ("仪器说不行" & (if En >= 0 then ":" & Json.Text (D, En) else ""));
            return;
         end if;
         if Rn < 0 then
            Err := To_Unbounded_String ("仪器没回像素");
            return;
         end if;
         for I in 0 .. Json.Count (D, Rn) - 1 loop
            declare
               N : constant Natural := Natural (Long_Float'Max (0.0, Json.Num (D, Json.Child (D, Rn, I))));
            begin
               for K in 1 .. N loop
                  Mask.Append (Cur);
               end loop;
               if Cur then
                  Area := Area + N;
               end if;
               Cur := not Cur;
            end;
         end loop;
         if Natural (Mask.Length) /= W * H then
            Err := To_Unbounded_String ("仪器回的像素数 " & Codec.Img (Natural (Mask.Length)) & " ≠ 画幅 " & Codec.Img (W * H));
            Mask.Clear; Area := 0;
            return;
         end if;
         Score := Json.Num (D, Json.Get (D, 0, "score"));
         Ok := True;
      end;
   end Segment;

end Instrument;
