# body-driver 测试配置(2026-09-26,PLAN §2b "身体有不动的眼"):官方 arx_x5 去掉头顶相机,只剩两只腕眼 ——
# 验没有不动的眼的身体:板只靠腕眼三角、腕眼按板一起解、东西躺的面照样拟合、碰桌面量指尖照样做。仿真侧的,驱动一个字不知道。
# 在箱子上用 RoboDojo 的 python 跑(cd /root/RoboDojo;/venv/RoboDojo/bin/python <这个文件>),写:
#   env_cfg/camera/camera_nohead.yml(camera_config.yml 去掉 cam_head)、env_cfg/arx_x5_nohead.yml(arx_x5 换这份相机配置)
import yaml, os
R = "/root/RoboDojo/env_cfg"
cam = yaml.safe_load(open(f"{R}/camera/camera_config.yml"))
cam.pop("cam_head", None)
if isinstance(cam.get("annotator"), dict):
    cam["annotator"].pop("cam_head", None)
open(f"{R}/camera/camera_nohead.yml", "w").write("# body-driver: camera_config.yml without cam_head (no fixed eye)\n" + yaml.safe_dump(cam, sort_keys=False))
env = yaml.safe_load(open(f"{R}/arx_x5.yml"))
env["config_name"] = "arx_x5_nohead"
env["config"]["camera"] = "camera_nohead"
open(f"{R}/arx_x5_nohead.yml", "w").write("# body-driver: official arx_x5 with the head camera removed (no fixed eye)\n" + yaml.safe_dump(env, sort_keys=False))
print("wrote", f"{R}/camera/camera_nohead.yml", f"{R}/arx_x5_nohead.yml")
