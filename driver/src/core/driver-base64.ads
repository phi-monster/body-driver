--  Base64 encoding (RFC 4648, the standard alphabet with padding), for
--  binary payloads carried in JSON or HTTP headers.

with Driver.Bytes;

package Driver.Base64 is

   function Encoded_Length (Bytes : Natural) return Natural is (4 * ((Bytes + 2) / 3));

   function Encode (Data : Driver.Bytes.Byte_Array) return String
     with Post => Encode'Result'Length = Encoded_Length (Data'Length);

end Driver.Base64;
