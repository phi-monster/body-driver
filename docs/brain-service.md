# 脑服务:身体问什么、要什么回答

这份文档讲驱动(身体)和"脑"之间怎么说话。脑是你的模型,跑在你自己的推理服务上;驱动只是它的一个客户端。照这份文档,不用读驱动的源码,就能接上一个脑,或者自己写一个假脑来测接线。

语言本身(程序里每个词是什么意思、驱动认哪些写法)见 [`driver/LANGUAGE.md`](../driver/LANGUAGE.md),这里不重复。

## 1. 接在哪

- 启动参数 `--eye host:port`,或者环境变量 `BL_EYE=host:port`(两个都给时用环境变量;都不给是 `127.0.0.1:8079`)。
- 每一问都是一次 HTTP `POST http://host:port/v1/chat/completions`,请求体是 JSON(OpenAI 的 chat completions 形状)。一次请求从连接、发送到收完,一共最多等 1800 秒;到时限还没收完,就照实报"没问成",这一问作废。
- 回包要是 JSON。驱动只读三样:`choices[0].message.content`(回答原文)、`choices[0].finish_reason`,以及出错时回包的原文(整段带进日志)。

不走 HTTP 也行:设了 `BL_BRAIN=<目录>`,驱动就把问题写成文件、等回答文件,见第 5 节。

## 2. 第一种问法:写一段程序

每一轮问一次。这是脑真正"拿着循环"的那一问。

### 请求

```json
{
  "model": "eye",
  "chat_template_kwargs": {"enable_thinking": false},
  "structured_outputs": {"grammar": "<这一轮的键盘:GBNF 文法原文>"},
  "messages": [{"role": "user", "content": [
    {"type": "image_url", "image_url": {"url": "data:image/bmp;base64,<画面>"}},
    {"type": "text", "text": "<问的话>"}
  ]}]
}
```

- `model` 一律是 `"eye"`。服务端要用这个名字把模型挂出来(vLLM:`--served-model-name eye`)。
- 驱动自己不带 `temperature`、`max_tokens` 这一类数,不替脑定。采样设置由部署给:环境变量 `BL_BRAIN_SAMPLING` 是一段 JSON 对象,驱动把它的成员原样并进这一问的请求,放在 `"model"` 后面。用什么模型,就照它自己的推荐设(Qwen3.5 不思考模式的推荐是 `{"temperature": 0.7, "top_p": 0.8, "top_k": 20, "min_p": 0, "presence_penalty": 1.5, "repetition_penalty": 1.0}`)。没设就什么都不带,服务端按它自己的默认。读不成一个 JSON 对象,或者带了这一问自己的键(`model`、`messages`、`structured_outputs`、`chat_template_kwargs`),就不带,开机后第一问时在日志里照实说一次。
- 认名字那一问(第 3 节)不并这一份:那一问要的是最可能的那个框,用 `temperature: 0`。
- `structured_outputs.grammar` 是这一轮的键盘:一份 GBNF 文法,只放这具身体、这一轮真按得动的键。服务端要按它做受限解码,也就是文法外的字一个都打不出来(vLLM 0.29 的 `structured_outputs`;别的服务各有各的名字,适配的事你来做)。文法每一轮现场生成;同一轮写在问话里给脑看的那张"纸",和它是同一份,参数一样(见 `LANGUAGE.md` §17.1)。
- 画面:主画面是脑这一轮选的那只眼(第一轮是不动的眼)。上面画着编号格子和清单上东西的编号框。别的眼并排缩在下面一条,每只框着白框、标着相机号。图是 24 位 BMP,base64 编码,放在 data URL 里。
- 问的话依次是:
  1. 一句"你就是这具身体";格子怎么编号。
  2. `YOUR BODY`:身体自己量到的东西。量过什么、多可信、每只眼在哪、清单上点过名的东西。
  3. `WHAT YOU JUST DID AND WHAT HAPPENED`:上一段程序每一节怎么收的尾,也就是执行器那句话的原文。
  4. `WHAT YOU ARE TRYING TO DO`:任务句(观测里的 `instruction`,或者环境变量 `BL_ORDER`)。
  5. 要是上一段被退回了,有一段 `I REFUSED YOUR LAST PROGRAM BEFORE ANYTHING MOVED`:哪一行、为什么、能照抄的替代。
  6. `ANSWER WITH ONE PROGRAM`,后面是这一轮的那张语法纸,最后几句说明这门语言怎么被执行。

### 回答

- `choices[0].message.content` 就是程序原文,一行一句,外面不再包 JSON。
- 驱动按 `LANGUAGE.md` 解析。解析不过、或者跟身体量到的不符,这一段一根手指都不动,原因和替代随下一轮一起给脑。退回不花任何代价。
- `finish_reason = "length"`:写到服务端的上限被截断了。半截话不当回答,这一问作废,下一拍重问。
- 回包里没有 `content`(空串)、不是 JSON、连不上:照实记下原因(日志 `[身] 🧠 问不通(…)`),这一拍不动,下一拍重问。
- 服务端回的错里带着 `maximum context length`:问的话太长,放不进模型的上下文。驱动把清单上限砍一半再问(日志 `[身] 🧠 你读不下这么长`)。

### 量过的一件事(10-01)

09-30 以后,文法里名字那一格、`say` 那一句、行数都没有上限,驱动也不带 `max_tokens`。拿 S1A1–S1A5、H48/H49 落盘的画面,问 Qwen3.5-9B(vLLM 0.29,模型目录里没有 `generation_config.json`,于是用 vLLM 的默认采样:temperature 1.0、不截 top_p / top_k):单件 30 问里 20 问、两件 36 问里 26 问写到 1024 个 token 还没停,多半是在名字那一格里不停地写。照模型卡给不思考模式的那组采样参数(就是上面 `BL_BRAIN_SAMPLING` 那一段),跑飞降到 8/30、11/36(两件那一份是加了两件那一句的键盘)。不带 `presence_penalty`、只用另外五样,单件的跑飞几乎不降(22/30)。所以采样设置要按模型自己的推荐给全。

## 3. 第二种问法:它在哪一框

程序里每出现一个东西的名字,身体就问一次这只眼(这只眼指不出,就按相机的次序再问别的眼):这个名字指的是画面里哪一框。框里哪些像素是它,由身体自己量(见 `LANGUAGE.md` §17.7)。

### 请求

```json
{
  "model": "eye",
  "temperature": 0,
  "chat_template_kwargs": {"enable_thinking": false},
  "response_format": {"type": "json_schema", "json_schema": {"name": "where_is_it", "strict": true, "schema": {
    "type": "object", "additionalProperties": false, "required": ["found", "bbox_2d"],
    "properties": {
      "found": {"type": "boolean"},
      "bbox_2d": {"type": "array", "minItems": 4, "maxItems": 4,
                  "items": {"type": "integer", "minimum": 0, "maximum": 1000}}}}}},
  "messages": [{"role": "user", "content": [
    {"type": "image_url", "image_url": {"url": "data:image/bmp;base64,<这只眼的干净画面>"}},
    {"type": "text", "text": "Locate what someone would call: <名字>\nIf you can see it in this picture, answer with the box around it. If you cannot see it here, say so - that is a normal answer and I will look with another eye rather than guess."}
  ]}]
}
```

- 这一问的 `temperature` 是 0,因为要稳:同一张图、同一个名字,应当回同一个框。
- 画面是干净的,不画格子,也不画编号框。实测(09-21)画上去的标记会伤到这个模型的眼力。
- `<名字>` 就是脑在程序里写的那串字,一个字不改。

### 回答

`content` 是一段 JSON:`{"found": true, "bbox_2d": [左, 上, 右, 下]}`,四个数是画幅的千分比(0..1000),和画面大小无关。看不见就回 `{"found": false, "bbox_2d": [0, 0, 0, 0]}`,这是正常的回答,不算错。

- `found` 是 true 但框是空的(右 ≤ 左,或者下 ≤ 上):当它没指出来。
- 连不上、超时、`content` 读不出 JSON:这一问"没问通",这个名字这一轮绑不上;别的眼也不再问,因为脑那头不通。

## 4. 一集里叫了几次脑

每问一次,日志里印一行,`N` 是这一集里第几次叫脑:

```
[脑] 这一集第 N 次叫脑:写一段程序 · 2.3 秒 · 交回 3 行
[脑] 这一集第 N 次叫脑:问「scissors」在哪一框 · 1.5 秒 · 框 [297 283 345 413]
```

每轮开头那一行(`[身] ── 第 k 轮 …`)后面带着这一集到这时叫过几次脑。对方复位、新的一集开始时,先印一行 `[脑] 上一集一共叫了脑 N 次(写程序 a 次、问在哪 b 次)`,再从零数。

## 5. 人当脑(`BL_BRAIN=<目录>`)

不走 HTTP:驱动把问题写进那个目录,等一个回答文件出现。这条通道和产品是一个形状:一个只会说话的脑,靠同一套键盘开这具身体。

| 问法 | 驱动写 | 你写 |
|---|---|---|
| 写一段程序 | `prog.txt`(问的话,第一行是怎么写回答)、`prog.bmp`(画面) | `prog_answer.txt`:程序原文 |
| 它在哪一框 | `where.txt`、`where.bmp`(干净的画面) | `where_answer.txt`:`{"found": true, "bbox_2d": [l, t, r, b]}` 或 `{"found": false, "bbox_2d": [0,0,0,0]}` |

回答一律先写临时文件,再 `mv` 成上面的名字。改名是原子的,驱动看见文件就读走,读完删掉。人当脑时,键盘上的约束没人替你挡,写错了驱动照样退回、说明为什么。

## 6. 写一个假脑测接线

只要照第 2、3 节回话就能接上。[`tools/fake_brain.py`](../tools/fake_brain.py) 是照这份文档写的一个(没看驱动源码),只用来测接线:写程序那一问只回一句 `say …`,问在哪一律回 `found: false`。它从不让身体动,因为替脑写动作的"假脑"是不许的。
