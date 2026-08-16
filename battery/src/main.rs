//! 跑整张矩阵,印出**绝对数**。零 GPU。

use battery::*;

fn main() {
    let 形状: Vec<(&str, Vec<contact_gen::P3>, f64)> = vec![
        ("圆柱", 圆柱(0.03, 0.08, 0.90), 0.94),
        ("方块", 方块(0.05, 0.04, 0.06, 0.90), 0.93),
        ("球", 球(0.035, 0.935), 0.935),
        ("薄板", 薄板(0.10, 0.08, 0.004, 0.90), 0.94),
        // 🔴 下面两个是**造完判据之后才加的形状** —— 前四个我是一边修一边看着它们调的,
        // 拿它们报出来的分带着"照着考题复习"的味道。这两个没被针对过。
        ("圆锥", 圆锥(0.035, 0.09, 0.90), 0.93),
        ("L形", L形(0.09, 0.07, 0.008, 0.06, 0.90), 0.93),
    ];
    let 手 = [Hand::两指, Hand::吸盘, Hand::三指, Hand::五指, Hand::双臂];

    let (mut 成, mut 总) = (0usize, 0usize);
    let mut 具名 = Vec::new();

    for (名, cloud, z) in &形状 {
        println!("\n═══ {名} ═══");
        print!("{:6}", "");
        for v in VERBS {
            print!("{v:^4}");
        }
        println!();
        for h in 手 {
            print!("{:6}", h.名());
            let mut 本行 = 0usize;
            for v in VERBS {
                let c = 跑一格(cloud, h, v, *z);
                print!("{:^4}", c.记号());
                总 += 1;
                if c.ok() {
                    成 += 1;
                    本行 += 1;
                }
                match &c {
                    Cell::NoContact(e) => 具名.push(format!("{名}/{}/{v}  ①没有接触集: {e}", h.名())),
                    Cell::NoWaypoints(e) => 具名.push(format!("{名}/{}/{v}  ②出不了航点: {e}", h.名())),
                    Cell::Wrong => 具名.push(format!("{名}/{}/{v}  ③推演跟第③格对不上", h.名())),
                    _ => {}
                }
            }
            println!("   {本行}/{}", VERBS.len());
        }
    }

    println!("\n○ 走通并验上 · ◐ 走通但身体自己验不了(要眼睛)· → 执行层自己产生 · ✗ 没做到");
    println!("\n🔴 一共 {总} 格,做到 {成} 格,没做到 {} 格。", 总 - 成);
    if !具名.is_empty() {
        println!("\n没做到的,逐格点名:");
        for l in &具名 {
            println!("  {l}");
        }
    }
}
