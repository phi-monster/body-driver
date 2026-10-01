with Act;
--  对方复位、开新的一集(路 1,10-01):一集和一集之间不带经验 —— 上一集的世界、脑交的那一段程序、记下的地方、认名字的那只眼、
--  这一集的选择都清掉;量过的身体(身体图、响应表、握区、几何、碰过的面)留着。原来这一段写在 body_driver 的干活循环里,
--  漏了 remember 记下的地方(C.Places)、上次认出名字的那只眼(C.Name_Cam)(路 7 查出)、脑那一段程序(段里一见复位就收段,
--  下一轮接着跑上一集那段的下一条)、"这一集已经自己换过一次眼睛了"(C.Eye_Chosen,从来没人清:第一集换过一次以后每一集都不许换)、
--  几何逼近的账、脑这一句要它怎么动(C.Wants)
package Episode is
   procedure Begin_New (C : in out Act.Context);
end Episode;
