with Codec;
with Http_Client;
with Json;
package body Instrument is

   --  配点仪器回的 "points"(和 Back 时的 "back")⇒ 每个查询点一条
   function Parse_Match (Reply : Unbounded_String; N : Natural; Back : Boolean; Err : out Unbounded_String) return Match_Vectors.Vector is
      Empty, Res : Match_Vectors.Vector;
      Jerr : Unbounded_String;
      D : Json.Doc;
   begin
      Err := Null_Unbounded_String;
      if not Json.Parse (To_String (Reply), D, Jerr) then
         Err := To_Unbounded_String ("仪器回的不是 JSON:" & To_String (Jerr));
         return Empty;
      end if;
      declare
         Okn : constant Integer := Json.Get (D, 0, "ok");
         En : constant Integer := Json.Get (D, 0, "err");
         Pn : constant Integer := Json.Get (D, 0, "points");
         Bn : constant Integer := (if Back then Json.Get (D, 0, "back") else -1);
      begin
         if Okn < 0 or else not Json.Bool (D, Okn) then
            Err := To_Unbounded_String ("仪器说不行" & (if En >= 0 then ":" & Json.Text (D, En) else ""));
            return Empty;
         end if;
         if Pn < 0 or else Json.Count (D, Pn) /= N then
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
               if Bn >= 0 and then I < Json.Count (D, Bn) then
                  declare
                     Bq : constant Integer := Json.Child (D, Bn, I);
                  begin
                     if Bq >= 0 and then Json.Count (D, Bq) >= 2 then
                        M.Bu := Json.Num (D, Json.Child (D, Bq, 0));
                        M.Bv := Json.Num (D, Json.Child (D, Bq, 1));
                     end if;
                  end;
               end if;
               Res.Append (M);
            end;
         end loop;
      end;
      return Res;
   end Parse_Match;

   function Points_Json (Pts : Match_Vectors.Vector) return Unbounded_String is
      R : Unbounded_String;
   begin
      Append (R, """points"":[");
      for I in 0 .. Natural (Pts.Length) - 1 loop
         Append (R, (if I > 0 then "," else "") & "[" & Codec.Fmt (Pts (I).U, 2) & "," & Codec.Fmt (Pts (I).V, 2) & "]");
      end loop;
      Append (R, "]");
      return R;
   end Points_Json;

   function Match (Host : String; Port : Natural; RGB_A : Buf; W_A, H_A : Natural; RGB_B : Buf; W_B, H_B : Natural;
                   Pts : Match_Vectors.Vector; Err : out Unbounded_String; Coarse : Boolean := False; Back : Boolean := False) return Match_Vectors.Vector is
      Empty : Match_Vectors.Vector;
      Req, Reply, Why : Unbounded_String;
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
      Append (Req, """,""num"":0," & (if Coarse then """coarse"":true," else "") & (if Back then """back"":true," else ""));
      Append (Req, Points_Json (Pts));
      Append (Req, "}");
      if not Http_Client.Post (Host, Port, "/match", Req, Reply, Why) then
         Err := To_Unbounded_String ("连不上仪器 " & Host & ":" & Codec.Img (Port) & ":" & To_String (Why));   --  原因照实带出来(09-30:HTTP 那一层不再吞掉)
         return Empty;
      end if;
      return Parse_Match (Reply, Natural (Pts.Length), Back, Err);
   end Match;

   function Match_Ids (Host : String; Port : Natural; Ia, Ib : Natural; Pts : Match_Vectors.Vector; Err : out Unbounded_String;
                       Coarse : Boolean := False; Back : Boolean := False) return Match_Vectors.Vector is
      Empty : Match_Vectors.Vector;
      Req, Reply, Why : Unbounded_String;
   begin
      Err := Null_Unbounded_String;
      if Host = "" or else Port = 0 then
         Err := To_Unbounded_String ("没配仪器");
         return Empty;
      end if;
      if Pts.Is_Empty then
         Err := To_Unbounded_String ("没有点可配");
         return Empty;
      end if;
      Append (Req, "{""a_id"":" & Codec.Img (Ia) & ",""b_id"":" & Codec.Img (Ib) & ",""num"":0," & (if Coarse then """coarse"":true," else "")
              & (if Back then """back"":true," else ""));
      Append (Req, Points_Json (Pts));
      Append (Req, "}");
      if not Http_Client.Post (Host, Port, "/match", Req, Reply, Why) then
         Err := To_Unbounded_String ("连不上仪器 " & Host & ":" & Codec.Img (Port) & ":" & To_String (Why));   --  原因照实带出来(09-30:HTTP 那一层不再吞掉)
         return Empty;
      end if;
      return Parse_Match (Reply, Natural (Pts.Length), Back, Err);
   end Match_Ids;
   function Describe (Host : String; Port : Natural; Ids : Ints; Err : out Unbounded_String) return Vec_Vectors.Vector is
      Empty, Res : Vec_Vectors.Vector;
      Req, Reply, Jerr, Why : Unbounded_String;
      D : Json.Doc;
   begin
      Err := Null_Unbounded_String;
      if Host = "" or else Port = 0 then
         Err := To_Unbounded_String ("没配仪器");
         return Empty;
      end if;
      Append (Req, "{""ids"":[");
      for I in 0 .. Natural (Ids.Length) - 1 loop
         Append (Req, (if I > 0 then "," else "") & Codec.Img (Ids (I)));
      end loop;
      Append (Req, "]}");
      if not Http_Client.Post (Host, Port, "/describe", Req, Reply, Why) then
         Err := To_Unbounded_String ("连不上仪器 " & Host & ":" & Codec.Img (Port) & ":" & To_String (Why));   --  原因照实带出来(09-30:HTTP 那一层不再吞掉)
         return Empty;
      end if;
      if not Json.Parse (To_String (Reply), D, Jerr) then
         Err := To_Unbounded_String ("仪器回的不是 JSON:" & To_String (Jerr));
         return Empty;
      end if;
      declare
         Okn : constant Integer := Json.Get (D, 0, "ok");
         Vn : constant Integer := Json.Get (D, 0, "vectors");
      begin
         if Okn < 0 or else not Json.Bool (D, Okn) or else Vn < 0 or else Json.Count (D, Vn) /= Natural (Ids.Length) then
            Err := To_Unbounded_String ("仪器没给整体特征");
            return Empty;
         end if;
         for I in 0 .. Json.Count (D, Vn) - 1 loop
            declare
               Qn : constant Integer := Json.Child (D, Vn, I);
               V : Floats;
            begin
               for K in 0 .. Json.Count (D, Qn) - 1 loop
                  V.Append (Json.Num (D, Json.Child (D, Qn, K)));
               end loop;
               Res.Append (V);
            end;
         end loop;
      end;
      return Res;
   end Describe;

   procedure Frame_Put (Host : String; Port : Natural; RGB : Buf; W, H : Natural; Id : out Integer; Err : out Unbounded_String) is
      Req, Reply, Jerr, Why : Unbounded_String;
      D : Json.Doc;
   begin
      Id := -1; Err := Null_Unbounded_String;
      if Host = "" or else Port = 0 or else W = 0 or else H = 0 then
         Err := To_Unbounded_String ("没配仪器或没有图");
         return;
      end if;
      Append (Req, "{""image"":""");
      Append (Req, Codec.Base64 (Codec.BMP24 (RGB, W, H)));
      Append (Req, """}");
      if not Http_Client.Post (Host, Port, "/frame", Req, Reply, Why) then
         Err := To_Unbounded_String ("连不上仪器 " & Host & ":" & Codec.Img (Port) & ":" & To_String (Why));   --  原因照实带出来(09-30:HTTP 那一层不再吞掉)
         return;
      end if;
      if not Json.Parse (To_String (Reply), D, Jerr) then
         Err := To_Unbounded_String ("仪器回的不是 JSON:" & To_String (Jerr));
         return;
      end if;
      declare
         Okn : constant Integer := Json.Get (D, 0, "ok");
         In_N : constant Integer := Json.Get (D, 0, "id");
      begin
         if Okn >= 0 and then Json.Bool (D, Okn) and then In_N >= 0 then
            Id := Integer (Json.Num (D, In_N));
         else
            Err := To_Unbounded_String ("仪器没存成这一帧");
         end if;
      end;
   end Frame_Put;

   procedure Segment (Host : String; Port : Natural; RGB : Buf; W, H : Natural; X0, Y0, X1, Y1 : Integer; Pts : Seg_Pt_Vectors.Vector;
                      Mask : out Bools; Area : out Natural; Score : out Long_Float; Ok : out Boolean; Err : out Unbounded_String) is
      Req, Reply, Jerr, Why : Unbounded_String;
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
      if not Http_Client.Post (Host, Port, "/segment", Req, Reply, Why) then
         Err := To_Unbounded_String ("连不上仪器 " & Host & ":" & Codec.Img (Port) & ":" & To_String (Why));   --  原因照实带出来(09-30:HTTP 那一层不再吞掉)
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
