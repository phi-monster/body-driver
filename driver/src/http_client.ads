--  一次 HTTP POST,读到对端关闭为止(Connection: close)。脑的桥不发 Content-Length,只能这么读。
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
package Http_Client is
   --  等脑回话的上限(秒,接线协议:脑想得再久也别永远等)
   function Post (Host : String; Port : Natural; Path : String; Body_Text : String;
                  Reply_Body : out Unbounded_String; Timeout_S : Duration := 1800.0) return Boolean;
   --  同一件事,请求体放在堆上(两张图的配点请求 ≈ 2.5 MB:拼成一个定长字串要在栈上摆好几份,8 MB 的栈装不下)
   function Post (Host : String; Port : Natural; Path : String; Body_Text : Unbounded_String;
                  Reply_Body : out Unbounded_String; Timeout_S : Duration := 1800.0) return Boolean;
end Http_Client;
