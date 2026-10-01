--  往一张面上压、压到碰到为止(碰指尖、合空时的尖都走它;大并行 §2 第 5 条,路 2):
--  每一步走 Selfmap.Step(路 4 的"走一步",压的那种步),碰到 = 它判的"挡住了"(Blocked_T)——
--  这一步少走的比这只手这一段空走时少走的(开机探针那几步 + 这一段里判成空走的每一步:平均、散布)多出门。
--  10-01 P8N(人形,手底下一个午餐肉罐):原来拿"这一段前两步空走"当底,头一步没得比、照收成空走的底,之后每步一样只走一半,
--  一步都认不出碰到,大步 / 小步来回找了 250 多拍。
with Plug;
with Selfmap;
with Geom;
package Touchdown is
   --  沿 Into(世界系单位方向)走一步 Lstep。先问反解:走完那一处位置还差超过这一步的一半、或朝向差超过转动一步看得见的那一档
   --  = 到了量到的关节限位 ⇒ 不走(At_Limit)。走 = Selfmap.Step(压的那种步;不按"到了那一档以内"收 —— 轻碰一步就一档那么小,
   --  一点没走也落在那一档里)。Hit = 这一步被挡住了;Short = 沿 Into 少走了多少;Rep = 这一步的账(Selfmap.Step 的)
   procedure Step_Down (L : in out Plug.Link; M : Selfmap.Body_Map; F : in out Plug.Frame; Arm : Natural; Into : Geom.V3; Lstep : Long_Float;
                        W : in out Selfmap.Walk; Short : out Long_Float; At_Limit, Hit : out Boolean; Rep : out Selfmap.Leg_Step);
   --  一步一步往下(每步 Lstep,最多 Steps 步):第一步就被挡住、哪一步被挡住都算碰到(Got_It);再往下一步到了量到的关节限位 ⇒ 停(At_Limit,不是碰到)。
   --  From = 碰到的那一步开始的地方(那一刻还没碰到);Used = 走了几步;Short = 最后一步少走了多少
   procedure Descend (L : in out Plug.Link; M : Selfmap.Body_Map; F : in out Plug.Frame; Arm : Natural; Into : Geom.V3; Lstep : Long_Float;
                      Steps : Natural; W : in out Selfmap.Walk; Got_It, At_Limit : out Boolean; From : out Plug.Arm_Pose; Used : out Natural;
                      Short : out Long_Float);
   --  这只手这一段空走时一步少走它自己的几成(判挡没挡的底:平均、散布、几步),只进日志
   function Free_Note (W : Selfmap.Walk; Arm : Natural) return String;
end Touchdown;
