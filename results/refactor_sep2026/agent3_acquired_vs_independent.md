# Free-search agent 3 (2026-09-07): 被收编 vs 独立 —— 可比公司结局、ARM 模式、收购条款、行情、副产品先例

Mandate: facts only, no verdict (agent's opinion in its own last section). 128 tool uses, 17 min. 数字/出处照录;查不到的写"查不到"。

## 总结(agent 人话)
1. 15 年里"机器人中间层 / 通用控制层 / 脑子"独立做成大生意的几乎没有;赚到钱的出口是被收编,价格与**人**强相关,与产品弱相关。Amazon–Covariant 2024:3.8 亿买非独占许可,条件 3 创始人 + 约 1/4 员工去 Amazon;剩下的壳被估"零到低两位数百万"(约 20 人)。同构:Microsoft–Inflection 6.5 亿、Google–Character.AI 27 亿、Google–Windsurf 24 亿、NVIDIA–Groq 200 亿。
2. 被整体收购后产品活下来的少:Kiva 不再外售;Fetch 2.9 亿买、4 年后 Zebra 减值退出、2026 卖给 Skild;Kindred 并入 Ocado;Energid 并入 UR;Vicarious→Intrinsic→2026 Intrinsic 并入 Google;Google 2013 买的 8 家大多关/卖;Boston Dynamics 被卖 3 次。反例:Universal Robots(2.85 亿买入,2024 收入 2.93 亿)、DeepMind。
3. "通用机器人 OS / 机体无关平台"独立公司融资几千万到一两亿美元,无一到 10 亿估值(Brain Corp 1.93 亿、Viam 1.17 亿、Realtime 0.7–1 亿、Wandelbots 1.26 亿、Micropsi 0.45 亿、Ready 0.42 亿后倒闭)。10 亿以上全是"脑子"公司:Skild 140 亿、PI 56 亿(传 110 亿)、Figure 390 亿、Field AI 20 亿。
4. ARM:1990 成立,1997 靠 Nokia 6110 拿到第一个大规模设计胜(Nokia 逼出 Thumb),2010 才占手机处理器 95%,约 20 年。收费 = 一次性授权费 100 万–1000 万美元 + 每片按售价百分比抽成;FY2026 总收入 49.2 亿,版税 26.1 亿。SoftBank 2016 年 243 亿英镑买(溢价 43%);NVIDIA 2020 年 400 亿想买,被美英欧中监管卡死,2022 放弃并留下 12.5 亿预付款;2023 重新上市估值 545 亿。RISC-V 实测渗透 2022 1.9% → 2023 3.9%(其余为预测)。
5. "授权被免费替代"的案例多于"授权赢":Windows Phone(每台 25 美元)被免费 Android 打到 3.6%;Symbian 2008 被迫免费;HEVC 专利池乱局催生免费 AV1;Unity 2023 按安装收费引发反弹、CEO 辞职、撤回;Microsoft Robotics Studio 输给 ROS。赢的(ARM、Qualcomm、Dolby、QNX、VHS)共同点:写进标准、客户换不掉、授权极宽松。
6. 2025–2026 行情:机器人创业融资 2025 约 140–150 亿美元,2026 至 9 月 188 亿(人形 86 亿)。机体公司对外部脑子分裂:Figure/Tesla/1X 自研(Figure 2025-02 踢掉 OpenAI:"we can't outsource AI for the same reason we can't outsource our hardware");BD/Apptronik/Agility/Agile 接 Google DeepMind;ABB/UR/MiR/Foxconn 接 Skild;Unitree 只卖身体、开放 SDK,2026-08 上市首日约 500 亿美元。NVIDIA(GR00T)、Google(Gemini Robotics API)、HF(LeRobot)、PI(π0 开源)都在免费发脑子。
7. "主业 A、副产品 B 成了标准"的历史:副产品的价值几乎总被"把它做成产品的那一方"拿走 —— Xerox PARC 的以太网/PostScript/GUI 分别被 3Com/Adobe/Apple 拿走;Willow Garage 造出 ROS 后自己关门;成功案例(Slack、AWS、Docker、Unity)都是母公司**自己转身把副产品当主业**。Willow 分拆 Unbounded 因分拆协议条款拿不到 A 轮,2015 死。

## 一、可比公司结局(节选;完整表见 agent 原报告要点)
被收购:Kiva(Amazon 2012,7.75 亿;停止外售)· 6 River(Shopify 4.5 亿 → Ocado 只付 1270 万)· Google Replicant 8 家(多数关/卖;Schaft 2018 关)· Boston Dynamics(Hyundai 约 8.8 亿买 80%;2026-06 再 3.25 亿买 SoftBank 剩余 9.65%,隐含约 33.7 亿)· DeepMind(>5 亿;2026-07 AlphaFold 团队解散)· Nest(32 亿;2016 试图出售)· Universal Robots(Teradyne 2.85 亿;2024 收入 2.93 亿)· MiR(1.48 亿 + 1.24 亿里程碑)· Energid(2500 万,"人才 + IP";2022 并入 UR 软件组)· Kinema(BD,未披露)· Auris(J&J 34 亿 + 23.5 亿里程碑;2024 判 J&J 赔 >10 亿)· Kindred+Haddington(Ocado 2.62 亿 + 2500 万)· Fetch(Zebra 2.9 亿;2025-12 退出计提约 5500 万;2026-04 卖给 Skild)· Vicarious(Intrinsic 2022)· OSRC(Intrinsic 2022;OSRF 独立;2024 OSRA)· Clearpath/OTTO(Rockwell 约 6.09 亿)· Veo(Symbotic 870 万资产)· Ghost Robotics(LIG Nex1 2.4 亿买 60%)· Franka(Agile Robots >3000 万欧,破产后)· Havok(Intel 1.1 亿 → Microsoft)· MuJoCo(DeepMind 2021,2022 开源)· ABB Robotics(SoftBank 53.75 亿)· Robust Intelligence/Protect AI/Lakera(Cisco 约 4 亿 / Palo Alto 6.345 亿 / Check Point 约 3 亿)。
独立:Brain Corp(约 1.93 亿融资,OEM 模式,>3 万台)· Viam(1.17 亿)· Realtime Robotics(7150 万)· Micropsi(约 4300 万;2024 收入约 670 万估算)· Wandelbots(1.26 亿)· Apex.AI(>7500 万;ASIL D 认证;2025 收入约 850 万估算)· Auterion(约 1.4–1.67 亿;约 6 亿估值;已盈利,签约收入约 2 亿)· Shield AI(127 亿估值 2026-03)· Applied Intuition(150 亿;ARR 约 8.3 亿)· Dexterity(16.5 亿)· Mujin(1.5 亿)· QNX(>2.75 亿辆车,约 2.5 美元/车;FY2027 指引 2.95–3.12 亿)· Wind River(Aptiv 43 亿)。
倒闭:Willow Garage(2014)· Unbounded(2015,分拆条款)· Rethink(2018 关,2024 复活,2025 再关)· READY Robotics(2024-08)· Airware(2018)· Anki/Jibo/Kuri · Covariant(壳)· iRobot(2025-12 Chapter 11)· 3DR 硬件 · Symbian/Windows Phone。

## 二、ARM 事实
版税 "based on a percentage of the ASP of the chip or a fixed fee per unit"(F-1);授权费 100 万–1000 万美元,Armv8 约 1–2% 售价,Armv9 约翻倍;Flexible Access 2019(0 美元起,应对 RISC-V);FY2025 收入 40.07 亿;FY2026 49.2 亿(版税 26.1,授权 23.1);累计出货 >3000 亿片;>99% 手机。时间线:1990 成立(Acorn+Apple+VLSI);1993–94 Nokia 逼出 Thumb;1997–98 Nokia 6110;1998 上市约 10 亿美元;Apple 1998–99 套现约 11 亿;2010 手机 95%。SoftBank 2016 每股 1700 便士、243 亿英镑;NVIDIA 2020-09 宣布 400 亿,2022-02 终止,SoftBank 留 12.5 亿;2023-09 Nasdaq 51 美元/股,545 亿。Qualcomm 诉讼 2024-12 陪审团、2025-10 终判 Qualcomm 胜。RISC-V:SHD 2024 预测 2030 年 162 亿片;2026 预测 2031 年 359 亿片;实测只有 2023 年 3.9%。
授权赢:Qualcomm QTL FY2024 55.72 亿;Dolby FY2024 12.737 亿,93% 授权;JVC VHS(1984 年 40 家 vs Betamax 12 家);Unreal 5%→3.5%;QNX/Wind River。被免费替代:Windows Phone、Symbian、MIPS/Imagination(2017 Apple 弃用后股价 -70%,5.5 亿英镑卖出)、Java vs Android(2021 最高法院 6:2)、HEVC vs AV1、Unity Runtime Fee、MRDS vs ROS。

## 三、大公司买什么;条款;许可+挖人
买人(Covariant、Inflection、Character、Windsurf、Groq、Adept);技术为非独占许可;把产品从市场拿走(Kiva);分发/现金流(UR、OTTO、ABB);数据(Adept、Meta–Scale 143 亿买 49%);只买 IP(Veo、Rethink、Jibo)。条款:里程碑(Auris 23.5 亿、MiR 1.24 亿、OTTO 4300 万)、回售权(BD 2021 协议 → 2026 行权)、非独占许可 + 不诉承诺(Inflection 6.2 亿 + 3000 万)、延迟支付(Covariant 一年后 2000 万)、治理条件(DeepMind 伦理委员会)、不持股(Windsurf 24 亿)、卖方拿买方股权(Zebra→Skild)、留任(Ghost、Franka、Windsurf 剩余员工由 Cognition 免 vesting cliff)。
许可+挖人:Microsoft–Inflection 2024-03 6.5 亿(早期投资人 1.5x / 后期 1.1x);Amazon–Adept 2024-06(约 2500 万,投资人"大致回本");Google–Character 2024-08 27 亿(约 2.5x);**Amazon–Covariant 2024-08-30 3.8 亿 + 1 年后 2000 万**;Meta–Scale 2025-06;Google–Windsurf 2025-07 24 亿(投资人 12 亿,被挖员工 12 亿,未被挖的约 200 人一分没拿;几天后 Cognition 约 2.5 亿买剩余);NVIDIA–Groq 2025-12 200 亿(约 3x)。监管:FTC 2025-01 6(b) 报告;FTC 主席明言查 acquihire 规避 HSR;CMA 认定 Microsoft–Inflection 属并购审查但放行。

## 四、2025–2026 行情与态度
Skild 140 亿(2026-01,C 轮 14 亿)· PI 56 亿(2025-11;2026-03 传 110 亿+;无公开收入)· Figure 390 亿 · Apptronik >55 亿 · Agility 约 21.2 亿 · 1X 传 100 亿+ · Field AI 20 亿 · Dyna >6 亿 · Genesis 传 30 亿 pre · Galbot 30 亿 · AgiBot 64 亿 · Unitree 上市约 500 亿美元(2025 收入 17.0 亿元、净利 2.78 亿元、人形 >5000 台)。态度:Figure/Tesla/1X 自研;BD(TRI 2024;DeepMind 2026-01)、Apptronik(DeepMind)、Agility/Agile/Enchanted(Gemini 受信)、UR(AI Accelerator;2026-03 接 Skild)、ABB/MiR/Foxconn(Skild;Foxconn 另与 Intrinsic JV)、Unitree 只卖身体、Samsung/LG/Hyundai 走股权、J&J 接 NVIDIA Isaac、Karl Storz 开放生态、Intuitive 封闭、Shield AI/Auterion 把脑/OS 授权给别家机身。免费发脑子:NVIDIA GR00T N1、Google Gemini Robotics On-Device + API、HF LeRobot(收购 Pollen)、PI openpi。

## 五、副产品先例
Slack(游戏公司转身,277 亿)· Flickr · AWS(2024 收入 1076 亿、营业利润 398 亿)· Docker(差点死)· Unity · Havok(被收)· MuJoCo(被收后开源)· ROS(母体关门;商业实体几经转手)· PX4→Auterion(分拆,盈利)· Xerox PARC(被离职者/外人拿走)· AlphaFold(团队后来流失)· BD(卖 3 次)· iRobot(破产)· Applied Intuition(仿真起家,150 亿)· AI 对抗测试三家被安全大厂买 · Unbounded(分拆条款致死)· Ghost Robotics(2.4 亿买 60%,团队留任)。

## 六、agent 的看法(标明)
1. 市场给"脑子/中间层"的出价是给人不是给产品;没有部署量时,被收编的价钱 ≈ 团队溢价。
2. ARM 模式在我们这个位置暂时不成立:缺一个被成本逼着要独门东西的 Nokia;身体层的天然买家是臂/工业机器人 OEM 和集成商,不是人形创业公司(它们明说脑子不能外包)。
3. 免费在往下压(NVIDIA/Google/HF/PI);唯一没人免费发的是"证过的安全核"——别的行业能长期收钱(QNX、VxWorks),但 Apex.AI 拿了 ASIL D 年收入才 850 万,说明认证本身不够,还得有量。
4. 副产品要么母体转身当主业,要么被外人拿走,没有第三种;不打算转身就先想清楚分拆条款。
5. 所有讨论排在"第一个棒球"后面:有估值的公司都至少有能放视频的部署,没有一家靠架构图拿到钱。
查不到:Kindred 团队去向、Kinema 是否独立销售、BD/PI/Skild 收入、Brain Corp/Mujin 估值、Adept 完整许可费、Google→SoftBank 出售 BD 官方价。
