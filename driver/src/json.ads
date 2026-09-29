--  够用的 JSON 读取器(节点池),外加给提示词转义用的 Escape、写数用的 Number。零依赖。
--  读得严:true / false / null 要整个词(原来只看头一个字母就往后跳 4 / 5 个字符 —— 身体文件里的 nan 被当成 null,还把后面的逗号或括号吃掉,
--  整份读错一位也不报);顶层的值后面还跟着东西 = 不是一份 JSON。
with Bytes; use Bytes;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Containers.Vectors;
package Json is
   type Kind is (J_Null, J_Bool, J_Num, J_Str, J_Arr, J_Obj);
   type Node is record
      K : Kind := J_Null;
      B : Boolean := False;
      F : Long_Float := 0.0;
      S : Unbounded_String;
      Kids : Ints;      --  Arr:元素;Obj:键(Str 节点),值,键,值…
   end record;
   package Node_Vectors is new Ada.Containers.Vectors (Natural, Node);
   type Doc is record
      Nodes : Node_Vectors.Vector;
   end record;
   function Parse (Src : String; D : out Doc; Err : out Unbounded_String) return Boolean;  --  根 = 0
   function Kind_Of (D : Doc; N : Integer) return Kind;
   function Get (D : Doc; Obj : Integer; Name : String) return Integer;   --  值节点或 -1
   function Text (D : Doc; N : Integer) return String;
   function Num (D : Doc; N : Integer) return Long_Float;
   function Is_Num (D : Doc; N : Integer) return Boolean;
   function Bool (D : Doc; N : Integer) return Boolean;
   function Count (D : Doc; N : Integer) return Natural;
   function Child (D : Doc; N : Integer; I : Natural) return Integer;
   function Is_Null (D : Doc; N : Integer) return Boolean;   --  这个节点在,而且写的就是 null(没有这个键不算)
   function Escape (S : String) return String;      --  \ " 换行 → JSON 字符串体
   --  一个数写成 JSON:有限数按"写出去再读回来一个比特不差"写 —— 科学记数、17 位有效数字(双精度 53 位尾数,要 ⌈53·log10 2⌉ + 1 = 17 位;
   --  离线拿 200 万个随机双精度逐个比过:17 位全对,少一位有 45% 读回来不一样)。原来按定点印几位小数:比那一位还小的量读回来就是 0,
   --  大过 1e15 的印成 inf(不是 JSON)。不是有限数(NaN、正负无穷)JSON 里写不了 ⇒ 写 null(= 这里没有一个数)
   function Finite (X : Long_Float) return Boolean;
   function Number (X : Long_Float) return String;
   --  Number 写出来的数读回来:null ⇒ NaN(Number 把不是有限数的写成 null,读回来还是"没有一个数",不编一个 0);其余同 Num
   function Real (D : Doc; N : Integer) return Long_Float;
end Json;
