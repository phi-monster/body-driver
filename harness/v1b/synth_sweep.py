import synth_v1b as Sy
import io, contextlib
rows = []
for rg in (3, 5, 10, 20):
    for px, rn in ((0.5, 0.02), (1.0, 0.05), (2.0, 0.1)):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            res = Sy.run(rg, 30, rn, px, seed=1)
        rows.append((rg, px, rn, res["test_med_mm"], res["test_max_mm"], res["test_med_deg"]))
        print("关节 ±%2d° · 像素噪声 %.1f px · 转动噪声 %.2f° ⇒ 考试停眼的位置误差 中位 %6.2f mm、最大 %6.2f mm;朝向 %.3f°" % rows[-1], flush=True)
