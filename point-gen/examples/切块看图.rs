//! 拿一张真实的深度 PGM + 彩色 PGM,跑 `分块`,把编号框画出来。
//! 用法:切块看图 <depth.pgm> <近m> <远m> <cam.pgm> <out.ppm>
fn 读pgm(p: &str) -> Option<(usize, usize, Vec<u8>)> {
    let b = std::fs::read(p).ok()?;
    // 头三行:magic / "w h" / maxval,之后紧跟像素。逐行扫,不假设分隔位置。
    let mut i = 0usize;
    let mut 行 = Vec::new();
    while 行.len() < 3 && i < b.len() {
        let j = b[i..].iter().position(|c| *c == b'\n')? + i;
        行.push(String::from_utf8_lossy(&b[i..j]).to_string());
        i = j + 1;
    }
    let mut n = 行[1].split_whitespace();
    let w: usize = n.next()?.parse().ok()?;
    let h: usize = n.next()?.parse().ok()?;
    let px = &b[i..];
    Some((w, h, px[..(w * h).min(px.len())].to_vec()))
}
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let (w, h, g) = 读pgm(&a[1]).expect("深度读不了");
    let (lo, hi): (f64, f64) = (a[2].parse().unwrap(), a[3].parse().unwrap());
    let dep: Vec<f64> = g.iter().map(|v| if *v == 0 { f64::NAN } else { lo + (*v as f64 / 255.0) * (hi - lo) }).collect();
    let 最少 = ((w * h) as f64 * 3e-5).ceil().max(4.0) as usize;
    for 倍 in [3.0f64, 5.0, 8.0, 12.0] {
        let r = point_gen::分块(&dep, w, h, 最少, 倍);
        println!("倍={倍}: {} 块", r.len());
        for (i, x) in r.iter().take(8).enumerate() {
            println!("   {}: {:6}px 框{:?} 深{:.3} 鼓{:.4}", i + 1, x.像素数, x.框, x.深, x.高);
        }
    }
    let 区们 = point_gen::分块(&dep, w, h, 最少, 3.0);
    let mut 区们 = 区们;
    区们.retain(|r| r.框[0] > 0 && r.框[1] > 0 && r.框[2] + 1 < w && r.框[3] + 1 < h);
    println!("去掉贴边的之后剩 {} 块", 区们.len());
    for (i, x) in 区们.iter().enumerate() { println!("   {}: {:6}px 框{:?} 深{:.3} 鼓{:.4}", i + 1, x.像素数, x.框, x.深, x.高); }
    let (cw, ch, cg) = 读pgm(&a[4]).expect("彩色读不了");
    assert_eq!((cw, ch), (w, h));
    let mut rgb: Vec<u8> = cg.iter().flat_map(|v| [*v, *v, *v]).collect();
    for (i, r) in 区们.iter().take(12).enumerate() {
        // 画框
        let (x0, y0, x1, y1) = (r.框[0], r.框[1], r.框[2], r.框[3]);
        for x in x0..=x1 { for y in [y0, y1] { let k = (y * w + x) * 3; rgb[k] = 255; rgb[k+1] = 32; rgb[k+2] = 32; } }
        for y in y0..=y1 { for x in [x0, x1] { let k = (y * w + x) * 3; rgb[k] = 255; rgb[k+1] = 32; rgb[k+2] = 32; } }
        let _ = i;
    }
    let mut out = format!("P6\n{w} {h}\n255\n").into_bytes();
    out.extend_from_slice(&rgb);
    std::fs::write(&a[5], out).unwrap();
    println!("写好 {}", a[5]);
}
