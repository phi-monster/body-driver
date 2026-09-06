--  任务记忆:固定几个槽位,只存"不会自己动"的事实(任务句、握着什么、试过什么),位置一律不存 —— 位置要看画面。
with Bytes; use Bytes;
package Memory is
   type Store is record
      Names, Values : Strs;
   end record;
   procedure Set (M : in out Store; Name, Value : String);
   function Get (M : Store; Name : String) return String;
   function Text (M : Store) return String;
   procedure Clear (M : in out Store);
end Memory;
