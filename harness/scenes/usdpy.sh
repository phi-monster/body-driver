#!/bin/bash
# 箱上离线的 USD python:Isaac 包里自带的 pxr(USD 核心 + UsdPhysics),不起 Isaac、不占卡(用 RoboDojo 那个 venv 的 python)。
# PhysX 自己的 schema 插件只有在 Kit 里才解得开(plugInfo 里的库路径是占位符),所以这里不加载它们。
# 用法:bash usdpy.sh 某个.py [参数…]
E=/venv/RoboDojo/lib/python3.11/site-packages/isaacsim/extscache
L=$(ls -d $E/omni.usd.libs-* | head -1)
export PYTHONPATH=$L:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/venv/RoboDojo/lib:$L/bin:${LD_LIBRARY_PATH:-}
exec /venv/RoboDojo/bin/python "$@"
