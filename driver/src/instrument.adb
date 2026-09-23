with Codec;
with Http_Client;
with Json;
package body Instrument is
   function Calibrate (Host : String; Port : Natural; RGB : Buf; W, H : Natural; Err : out Unbounded_String) return Calib is
      R : Calib;
      Reply, Jerr : Unbounded_String;
      D : Json.Doc;
   begin
      Err := Null_Unbounded_String;
      if Host = "" or else Port = 0 or else W = 0 or else H = 0 then
         Err := To_Unbounded_String ("没配仪器");
         return R;
      end if;
      declare
         Body_Json : constant String := "{""image"":""" & Codec.Base64 (Codec.BMP24 (RGB, W, H)) & """}";
      begin
         if not Http_Client.Post (Host, Port, "/calib", Body_Json, Reply) then
            Err := To_Unbounded_String ("连不上仪器 " & Host & ":" & Codec.Img (Port));
            return R;
         end if;
      end;
      if not Json.Parse (To_String (Reply), D, Jerr) then
         Err := To_Unbounded_String ("仪器回的不是 JSON:" & To_String (Jerr));
         return R;
      end if;
      declare
         Okn : constant Integer := Json.Get (D, 0, "ok");
         En : constant Integer := Json.Get (D, 0, "err");
         Fn : constant Integer := Json.Get (D, 0, "f");
         Fs : constant Integer := Json.Get (D, 0, "f_sd");
         Un : constant Integer := Json.Get (D, 0, "up");
         Us : constant Integer := Json.Get (D, 0, "up_sd");
         Msn : constant Integer := Json.Get (D, 0, "ms");
         Mdn : constant Integer := Json.Get (D, 0, "model");
      begin
         if Okn < 0 or else not Json.Bool (D, Okn) or else Fn < 0 or else not Json.Is_Num (D, Fn) then
            Err := To_Unbounded_String ("仪器说量不了" & (if En >= 0 then ":" & Json.Text (D, En) else ""));
            return R;
         end if;
         R.F := Json.Num (D, Fn);
         if Fs >= 0 and then Json.Is_Num (D, Fs) then
            R.F_Sd := Json.Num (D, Fs);
         end if;
         if Un >= 0 and then Json.Count (D, Un) = 3 then
            for I in 0 .. 2 loop
               R.Up (I) := Json.Num (D, Json.Child (D, Un, I));
            end loop;
            if Us >= 0 and then Json.Is_Num (D, Us) then
               R.Up_Sd := Json.Num (D, Us);
            end if;
         end if;
         if Msn >= 0 and then Json.Is_Num (D, Msn) then
            R.Ms := Json.Num (D, Msn);
         end if;
         if Mdn >= 0 then
            R.Model := To_Unbounded_String (Json.Text (D, Mdn));
         end if;
         R.Ok := R.F > 0.0;
      end;
      return R;
   end Calibrate;
end Instrument;
