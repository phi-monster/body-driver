# -*- coding: utf-8 -*-
# 一具身体两条链(人形左右臂共用 robot_name)⇒ 规划器/逆解器按"名字/侧"分开存,不然后建的把先建的盖掉,两条臂用同一条链(G1K 2026-09-24:右臂的目标用左链正解,常量差 15–30 cm,右臂不动)
import io
p = "/root/RoboDojo/env/robot_manager/robot_manager.py"
s = io.open(p, encoding="utf-8").read()
if "[bd] planner key" not in s:
    old1 = "        planner = self.ik_solver[robot.robot_name]\n"
    new1 = "        planner = self.ik_solver[self._bd_pk(robot)]   # [bd] planner key: per arm chain\n"
    old2 = "            self.planner[robot.robot_name] = CuroboPlanner(\n"
    new2 = "            self.planner[self._bd_pk(robot)] = CuroboPlanner(\n"
    old3 = "            self.ik_solver[robot.robot_name] = self.planner[robot.robot_name]\n"
    new3 = ("            self.ik_solver[self._bd_pk(robot)] = self.planner[self._bd_pk(robot)]\n"
            "\n"
            "    @staticmethod\n"
            "    def _bd_pk(robot):\n"
            "        # [bd] planner key: a coupled robot (one articulation, several arm chains) shares robot_name across its sides\n"
            "        return f\"{robot.robot_name}/{getattr(robot, 'side', '')}\"\n")
    for o in (old1, old2, old3):
        assert s.count(o) == 1, o
    s = s.replace(old1, new1).replace(old2, new2).replace(old3, new3)
    io.open(p, "w", encoding="utf-8").write(s)
    print("patched")
else:
    print("already patched")
