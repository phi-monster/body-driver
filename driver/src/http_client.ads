--  一次 HTTP POST,读到对端关闭为止(Connection: close)。脑的桥不发 Content-Length,只能这么读。
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
package Http_Client is
   --  等脑回话的上限(秒,接线协议:脑想得再久也别永远等)
   function Post (Host : String; Port : Natural; Path : String; Body_Text : String;
                  Reply_Body : out Unbounded_String; Timeout_S : Duration := 1800.0) return Boolean;
end Http_Client;
