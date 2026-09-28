package body Lockstep is
   use Ada.Task_Identification;

   --  一扇门:Open 开一次、Wait 过去一个(过去就关上)
   protected type Gate is
      entry Wait;
      procedure Open;
   private
      Is_Open : Boolean := False;
   end Gate;

   protected body Gate is
      entry Wait when Is_Open is
      begin
         Is_Open := False;
      end Wait;
      procedure Open is
      begin
         Is_Open := True;
      end Open;
   end Gate;

   Hand_Gate : array (0 .. Max_Hands - 1) of Gate;
   Main_Gate : Gate;
   --  登记只在主线程叫醒手之前写、手交还以后读:两边之间隔着门(保护对象),先写后读有先后
   Ids : array (0 .. Max_Hands - 1) of Task_Id := [others => Null_Task_Id];
   Fin : array (0 .. Max_Hands - 1) of Boolean := [others => True];

   procedure Start (H : Natural; Id : Task_Id) is
   begin
      Ids (H) := Id;
      Fin (H) := False;
   end Start;

   procedure Begin_Hand (H : Natural) is
   begin
      Hand_Gate (H).Wait;
   end Begin_Hand;

   function Current_Hand return Integer is
      Me : constant Task_Id := Current_Task;
   begin
      for H in Ids'Range loop
         if Ids (H) /= Null_Task_Id and then Ids (H) = Me then
            return H;
         end if;
      end loop;
      return -1;
   end Current_Hand;

   procedure Yield is
      H : constant Integer := Current_Hand;
   begin
      if H < 0 then
         return;
      end if;
      Main_Gate.Open;
      Hand_Gate (H).Wait;
   end Yield;

   procedure Done is
      H : constant Integer := Current_Hand;
   begin
      if H < 0 then
         return;
      end if;
      Fin (H) := True;
      Main_Gate.Open;
   end Done;

   procedure Run (H : Natural) is
   begin
      if Fin (H) then
         return;
      end if;
      Hand_Gate (H).Open;
      Main_Gate.Wait;
   end Run;

   function Finished (H : Natural) return Boolean is (Fin (H));

   procedure Clear is
   begin
      Ids := [others => Null_Task_Id];
      Fin := [others => True];
   end Clear;
end Lockstep;
