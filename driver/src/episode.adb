with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with World;
with Memory;
with Sinew;
package body Episode is
   procedure Begin_New (C : in out Act.Context) is
   begin
      --  上一集的世界
      World.Reset_All (C.Wld);
      Memory.Clear (C.Mem);
      C.Recent := Null_Unbounded_String;
      C.Cam := C.Map.World_Cam;
      --  脑起的名字、脑说过"这只眼里没有它"、手指指向 —— 都是上一集的世界,一起清;量过的身体留着。
      --  碰过的面留着但标成"上一集的":桌子一般不动,第一句话就有高度可用;新一集第一次朝下被顶住就换成新量的
      C.Boxed.Clear;
      C.Touch_Fresh := False; C.Bumps.Clear; C.Fingers_Aimed := False; C.Geo_Pw_Valid := False; C.Geo_Pw_Met := False; C.Geo_At_Above := False;
      C.Sil_Valid := False; C.Held_Set_Valid := False; C.Walls.Clear; C.No_Reach_Arm := -1;   --  每件东西量到的摩擦(C.Grip_Mus)留着:越用越准
      --  remember 记下的地方、上次认出名字的那只眼(路 7 查出,10-01)
      C.Places.Clear;
      C.Name_Cam := -1;
      --  脑交的那一段程序(C.Prog / C.M / C.Binds / C.Have_Prog 和它的账)这里不清:清了就是"半路扔掉整段程序"的第三处,
      --  闸门棘轮只许两处(跑完、编译期退回)—— 新的一集算不算第三处,等主代理定(10-01 报了:今天段里一见复位就收段,
      --  下一轮接着跑上一集那段程序的下一条)
      --  这一节 / 这一集的选择:换过眼、anyway、点了哪只眼、点名的那块在哪台相机
      C.Eye_Chosen := False;
      C.Reckless := False;
      C.Eye_Want := Sinew.Ey_None;
      C.Tgt_Cam := -1;
      --  几何逼近的账(这一集里点名那块的观测、走了多远、往哪走)和脑这一句要它怎么动
      C.Geo_Dist := -1.0; C.Geo_Round := 0; C.Geo_At_Arm := -1; C.Geo_Came := 0.0;
      C.Geo_Obs.Clear; C.Geo_Slot := -1; C.Geo_Name := Null_Unbounded_String; C.Geo_Pw_Name := Null_Unbounded_String;
      C.Wants.Clear;
      Act.Init_Tracks (C);
   end Begin_New;
end Episode;
