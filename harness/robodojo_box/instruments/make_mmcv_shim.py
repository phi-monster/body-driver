# -*- coding: utf-8 -*-
# 把 mmcv 的 multi_scale_deform_attn.py(Apache-2.0)改成纯 PyTorch:去掉 mmengine/ext 依赖,永远走 pytorch 采样路径
import io, os, re
src = io.open("/root/instruments/mmcv_msda_ref.py", encoding="utf-8").read()
src = src.replace("import mmengine\n", "")
src = src.replace("from mmengine.model import BaseModule, constant_init, xavier_init\nfrom mmengine.registry import MODELS\nfrom mmengine.utils import deprecated_api_warning\n", "")
src = src.replace("from mmcv.utils import (IS_CUDA_AVAILABLE, IS_MLU_AVAILABLE, IS_MUSA_AVAILABLE,\n                        IS_NPU_AVAILABLE)\nfrom ..utils import ext_loader\n\next_module = ext_loader.load_ext(\n    '_ext', ['ms_deform_attn_backward', 'ms_deform_attn_forward'])\n", "")
src = src.replace("@MODELS.register_module()\nclass MultiScaleDeformableAttention(BaseModule):", "class MultiScaleDeformableAttention(nn.Module):")
src = src.replace("                 init_cfg: Optional[mmengine.ConfigDict] = None,", "                 init_cfg=None,")
src = src.replace("        super().__init__(init_cfg)\n", "        super().__init__()\n")
src = src.replace("    @deprecated_api_warning({'residual': 'identity'},\n                            cls_name='MultiScaleDeformableAttention')\n", "")
src = src.replace("        if ((IS_CUDA_AVAILABLE and value.is_cuda)\n                or (IS_MLU_AVAILABLE and value.is_mlu)\n                or (IS_MUSA_AVAILABLE and value.is_musa)\n                or (IS_NPU_AVAILABLE and value.device.type == 'npu')):",
                  "        if False:   # 纯 PyTorch:没有编译的 CUDA 算子")
helpers = '''

def constant_init(module, val, bias=0.):
    nn.init.constant_(module.weight, val)
    if hasattr(module, 'bias') and module.bias is not None:
        nn.init.constant_(module.bias, bias)


def xavier_init(module, gain=1, bias=0., distribution='normal'):
    if distribution == 'uniform':
        nn.init.xavier_uniform_(module.weight, gain=gain)
    else:
        nn.init.xavier_normal_(module.weight, gain=gain)
    if hasattr(module, 'bias') and module.bias is not None:
        nn.init.constant_(module.bias, bias)

'''
src = src.replace("from torch.autograd.function import Function, once_differentiable\n", "from torch.autograd.function import Function, once_differentiable\n" + helpers)
for bad in ("mmengine", "ext_loader", "MODELS", "deprecated_api_warning", "IS_CUDA_AVAILABLE", "BaseModule"):
    assert bad not in src.replace("ext_module", ""), bad
os.makedirs("/root/instruments/track_on/mmcv/ops", exist_ok=True)
io.open("/root/instruments/track_on/mmcv/__init__.py", "w", encoding="utf-8").write("# 纯 PyTorch 顶替 mmcv.ops(只为 Track-On2 的 MultiScaleDeformableAttention)\n")
io.open("/root/instruments/track_on/mmcv/ops/__init__.py", "w", encoding="utf-8").write(src)
print("shim ok, lines", src.count("\n"))
