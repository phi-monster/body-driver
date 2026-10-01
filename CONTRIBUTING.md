# 参与 Body Driver

谢谢你愿意帮忙。中文、英文都可以。

## 报告问题

在 GitHub 上开 issue:<https://github.com/phi-monster/body-driver/issues>。请写清楚:

1. 用的哪一版:`git rev-parse --short HEAD` 打出来的提交号;
2. 什么身体:真机还是仿真,几只手、几个关节、几路相机;脑和仪器用的是什么;
3. 你做了什么,想看到什么,实际看到了什么;
4. 驱动打出来的日志:从启动到出问题的那一段(长的放附件)。

日志和图片里不要带密码、令牌、内网地址这类东西。

安全问题(比如别人能远程让你的机器人动起来)不要公开发 issue,请发邮件到【待填:联系邮箱】。

## 发 pull request

1. 改动大的,先开 issue 说一下打算怎么改,再动手。
2. fork 仓库,从 `main` 拉一个分支改。一个 PR 只做一件事。
3. 提交前在本地跑一遍 `bash install.sh`(要先装好 Alire,也就是 `alr`)。它依次跑四道检查(`check_purity.sh`、`check_constants.sh`、`check_freedom.sh`、`check_gates.sh`)、编译、离线自检 `selfcheck`,能跑的机器上还跑 SPARK 证明,最后把驱动装到 `~/.local/bin/bl-calibrate`。哪一步没过,先修好再提。
4. PR 说明里写:改了什么,为什么改,怎么验的(跑了什么、数是多少;有一条正面、一条反面的对照最好)。
5. PR 说明里写上 CLA 那一句(见下一节)。没有这句话的 PR 不合并。

## CLA:贡献者许可协议

外部贡献要先同意 [`CLA.md`](CLA.md)(版本 1.0)。简单说:版权还是你的;你允许 phi-monster 永久、免费地使用你的贡献,并且可以按任何条款再授权给别人(包括 GNU AGPL-3.0-only 和 [《Body Driver 免费商业授权》](LICENSE-COMMERCIAL.md));你保证你有权贡献它。有了这一条,项目才能一边开源,一边让公司免费拿商业授权。

签法:在 PR 说明里写上这一句,原样复制:

```
我已阅读并同意 CLA.md(版本 1.0)/ I have read and agree to CLA.md (version 1.0)
```

替公司贡献的(比如这是你工作的一部分),改写这一句,把尖括号连同里面的字换成公司的正式名称:

```
我代表 <公司正式名称> 同意 CLA.md(版本 1.0)/ I agree to CLA.md (version 1.0) on behalf of <company legal name>
```

第一次写了,你以后的贡献也算在内;每个 PR 都写,是为了核对方便。

## 项目自己的规矩

每个量只有一种量法,所有身体都一样;不按任务写代码,不按机器人写分支(`驱动重写.md` §2)。驱动源码里出现 benchmark、任务或机器人的名字,`check_purity.sh` 会拦下。

## 许可证

你的贡献按 GNU AGPL-3.0-only 发布(见 [`LICENSE`](LICENSE)、[`NOTICE`](NOTICE));按 CLA,phi-monster 也可以把它用在免费商业授权和以后的授权条款里。
