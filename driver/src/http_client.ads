--  一次 HTTP POST,读到对端关连接为止(请求里写着 Connection: close)。
--  成了 = 连上、发完、在时限内读到对端关连接、状态码 2xx、对方说了正文多长时一个字节不少(分块编码的正文按块拼回)。
--  别的一律返回 False,Why 里照实写为什么:连不上 / 发不出去 / 等到时限还没回完(用了多久、收到多少字节)/ 半路断了 /
--  回的不是 HTTP / 状态码不是 2xx(连状态行和对方回的正文)/ 正文短了。状态码不是 2xx 时 Reply_Body 照样放着对方回的正文
--  (里面常写着为什么 —— 脑的"上下文装不下"连限额和用量都写在里面,调用方要读)。
--  原来:超时、半路断线的异常在接收循环里被吞掉,收到过回包头就把半截回话当成功交出去;状态码从来不看,
--  400 / 500 的错误体也当成功交出去(09-30 审计 H2 / H3 查出)。
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
package Http_Client is
   --  等对方回话的上限(秒,接线协议:脑想得再久也别永远等)。从连接到读完一共这么久,不是每读一次这么久
   Default_Timeout : constant Duration := 1800.0;
   function Post (Host : String; Port : Natural; Path : String; Body_Text : String;
                  Reply_Body, Why : out Unbounded_String; Timeout_S : Duration := Default_Timeout) return Boolean;
   --  同一件事,请求体放在堆上(两张图的配点请求 ≈ 2.5 MB:拼成一个定长字串要在栈上摆好几份,8 MB 的栈装不下)
   function Post (Host : String; Port : Natural; Path : String; Body_Text : Unbounded_String;
                  Reply_Body, Why : out Unbounded_String; Timeout_S : Duration := Default_Timeout) return Boolean;
end Http_Client;
