# How to be good at research

> 转载自 @vivek 2026-06-10

Nobody really teaches you research. you get a desk, a problem someone else picked, and a vague instruction to produce something novel. so most people reverse-engineer the job from what they can see, which is papers, threads, and announcements, and what they end up learning is how to look like a researcher rather than how to be one. the actual skill is a stack of smaller skills, and almost every one of them can be deliberately trained.

没人真正教你如何做研究。你只有一张桌子，一个别人替你选好的问题，和一句模糊的指令——“做出点新东西”。于是，大多数人只能从他们看得见的东西里反向推导应该如何工作：论文、推特讨论、各种公告，最终学会的是如何看起来像一名研究者，而不是如何成为一名研究者。真正的研究能力是一些小技能，而其中几乎每一项都可以刻意训练学会。
## Pick your own problems

Richard Hamming had a habit at bell labs that made him unpopular at lunch. he'd ask whoever sat near him what the important problems in their field were, then ask why they weren't working on them. people changed tables. the question stings because most of us have no good answer. we don't choose problems, we absorb them, from an advisor, from whatever a big lab announced last quarter, from the paper everyone is quote-tweeting this week.

理查德·汉明在贝尔实验室有一个让他在午餐时不太受欢迎的习惯。他会问坐在附近的任何人，他们领域里那些重要的问题是什么，然后问，为什么他们没有在研究这些问题。人们受不了他，选择换桌子。这个问题之所以伤人，是因为我们大多数人没法回答。我们没有主动选择课题，而是被动接受课题——从导师那里，从某个大实验室上个季度宣布的东西里，从这周所有人都在引用转发的那篇论文里。

The trouble with an absorbed problem is that you hold the conclusion without the reasoning. you know some famous lab cares about a direction, but not why, not what they expect to find, not what would make them drop it. when they pivot, you find out a year later. and on a problem that's already fashionable, you're racing a thousand people who started earlier and have more compute than you.

被动接受的课题，麻烦在于你知道“要研究”，却不知道“为什么要研究”。你知道某个著名实验室在意一个方向，但不知道他们为什么在意，不知道他们期望发现什么，不知道他们会在什么时候止步。等他们转向了，你要过一年后才知道。此外，已经很火的研究领域竞争激烈，竞争者起步更早、算力更多。

John Schulman's guide to ml research splits the work into two modes. in one, you read the literature and hunt for things to improve. in the other, you choose an outcome you genuinely want to exist and reason backwards to the experiments. he argues for the second, and the quiet reason is that it manufactures originality. a goal you actually care about will drag you into territory no survey paper covers.

约翰·舒尔曼的机器学习研究指南将工作分成了两种模式。一种是阅读文献，寻找可以改进的地方。另一种是，先选定一个你真心希望它存在的成果，再逆向推导出实验。他主张第二种，理由是更利于创新。一个你真正在乎的目标，会把你拖进任何综述论文都覆盖不到的领地。

taste, meanwhile, gets discussed like a gift. it behaves more like a muscle. predict the result of every experiment before you run it. cover a paper's results section and guess the numbers from the method alone. mark down which of this month's releases will matter in two years and check your hit rate later. a forecast plus a correction, repeated a few hundred times, is how every good model gets trained, including the one in your head.

与此同时，“研究品味”常被当作一种天赋来讨论，但它表现得更像一块“肌肉”。在运行每个实验之前，预测它的结果。遮住一篇论文的结果部分，单凭论文使用的方法去猜那些数字。记下本月发布的成果中哪些在两年后依然重要，日后核对你的命中率。一个预测，加上一次修正，重复几百遍——这就是每一个好模型被训练出来的方式，包括你脑袋里那个。

## Upgrade your inputs

Shared reading lists produce shared ideas. if your information diet is the trending page of arxiv plus whatever survives the group chat filter, you will reliably reach the same conclusions as everyone else, at the same time, which makes those conclusions worth approximately nothing.

共同的阅读清单会培养出雷同的想法。如果你的信息食谱是 arXiv 的热门页面，外加群聊筛选后的残余，你将必然在同一时间得出和其他所有人一样的结论，这让这些结论的价值约等于零。

Old material is criminally underpriced. this field reruns its own past on a delay: mixture of experts dates to 1991, lstms to 1997, backprop went mainstream in 1986. Rich Sutton needed about a thousand words in 2019 to write the bitter lesson, and it predicts the shape of the field better than surveys ten times its length. Claude Shannon gave a talk on creative thinking in 1952 where his opening move was to **shrink a problem until it's nearly trivial, crack the small version, then reintroduce the difficulty one piece at a time.** That single trick will carry you through more walls than any modern productivity advice.

旧材料的价值被严重低估了。这个领域每过一段时间重演自己的过去：混合专家模型可追溯到 1991 年，LSTM 到 1997 年，反向传播从 1986 年起成为主流。理查德·萨顿在 2019 年用了大约一千字写下《苦涩的教训》，它对领域格局的预测，胜过篇幅十倍于它的综述。克劳德·香农在 1952 年做过一场关于创造性思维的演讲，他的开场方法是：把问题缩小，直到它近乎微不足道，破解这个微缩版本，然后一点一点把难度重新加回去。单单这一个技巧，就能带你穿过比任何现代生产力建议更多的墙。

Range matters as much as depth. interpretability borrows shamelessly from neuroscience. eval design is mechanism design wearing a lab coat. a working sense of how gpus actually move memory tells you which architecture papers are doomed before the benchmarks do. and honest statistics might be the rarest skill in ml, where a lot of published rigor is vibes with error bars.

广度和深度同等重要。可解释性研究毫不客气地从神经科学里借鉴。评估设计，就是穿着实验室白大褂的机制设计。对 GPU 实际如何搬运内存有实操性的体感，能让你在基准测试之前就知道哪些架构论文注定失败。而诚实的统计学，恐怕是机器学习中最罕见的技能——在这里，许多发表的严谨，不过是带上了误差条的氛围感。

One more thing. read the paper itself, not the thread summarizing it. the appendix is where the bodies are buried, and the limitations section is usually the most honest paragraph in the document.

还有一件事。去读论文本身，不要读那些总结。附录是埋藏尸体之处，而局限性部分，通常是整份文档中最诚实的一段。
## Write everything down

Paul Graham points out that an idea can feel fully formed right up until you try to put it into words. the page finds gaps your head papers over: the assumption you never tested, the step that doesn't actually follow, the two claims that quietly contradict each other.

保罗·格雷厄姆指出，一个想法在你试图把它付诸文字之前，会感觉已经完备成形。纸张会发现你头脑掩盖住的漏洞：那个你从未检验的假设，那个实际上推不出的步骤，那两个悄悄自相矛盾的说法。

**Feynman's rule was that the first person you must avoid fooling is yourself, because you're the easiest target.** Writing is the cheapest defense ever invented. Darwin went further and made it procedural. any fact that cut against his theory got written down on the spot, because he'd caught his own memory deleting inconvenient evidence faster than the convenient kind. your memory does the same thing to your failed runs. keep a log: hypothesis, setup, expectation, result, updated belief. rereading last month's entries is humbling in a way no reviewer can match.

费曼的法则：你必须避免愚弄的第一个人就是你自己，因为你是最容易的目标。写作是有史以来发明的最廉价的防御。达尔文走得更远，把它变成了程序。任何与他的理论相悖的事实，都会被他当场记下，因为他发现自己的记忆删除不利证据的速度，比删除有利证据更快。你的记忆对你那些失败的实验运行做着同样的事。保持一个日志：假说、设置、预期、结果、更新后的信念。重读上个月的条目，会让你感到一种任何审稿人都给不了的谦卑。

Then put some of it in public. olah and carter's research debt essay makes the case that fields choke on undigested ideas, and that a clear explanation is a genuine contribution rather than a service job. a lot of people working in interpretability today found the field through readable posts, not conference papers. a body of public writing also doubles as the strongest credential you can hold, because it's an unfakeable sample of how you think.

然后把其中一些放到公开场合。奥拉和卡特的《研究债务》一文阐述了，领域会因未被消化的想法而窒息，而一份清晰的解释是一种真正的贡献，而非服务性的活计。今天许多从事可解释性研究的人，是通过可读性强的博文，而不是会议论文，进入这个领域的。一组公开的写作作品，也能兼作你能拥有的最强凭证，因为它是你思考方式的一个无法伪造的样本。
## Tighten the loop

the stories about alec radford rarely involve a single stroke of genius. they involve volume. more runs per day, more wrong ideas discarded per week, a model of reality that updated faster than anyone else's. that's the actual game. research speed is mostly the speed at which you discover you're wrong.

关于亚历克·拉德福德的故事，很少涉及单次的灵光一现，它们涉及的是数量：每天更多的运行次数，每周更多被丢弃的错误想法，一个比任何人都更新得更快的现实模型。这才是真正的游戏。研究速度，主要就是你发现自己错了的速度。

Which makes tooling a first-class research activity. launching a run should be one command. plotting it should be one more. every experiment should be reproducible from its config, and comparing two runs should take seconds, not an afternoon of archaeology. karpathy's recipe for training neural networks has a step that pays for itself a hundred times over: overfit a single batch before training at scale. thirty seconds, half your bugs, gone. shrink everything until it's cheap, get it right, then spend the compute.

这便让工具建设成为一等的研究活动。启动一次运行应该是一个命令。画图应该是再一个命令。每个实验都应该能从它的配置文件复现，比较两次运行应该只需几秒，而不是一个下午的考古挖掘。卡帕西训练神经网络的秘方中，有一个步骤的回报是百倍以上的：在进行大规模训练之前，先对单个批次过拟合。三十秒，一半的缺陷，消失。把一切缩小，直到代价变得低廉，把它做对，然后再投入算力。

And retire the idea that engineering is the junior partner here. At the frontier the two jobs have fused. the researcher who can build the harness, the eval, and the data pipeline is the one whose hypotheses actually get tested. everyone else is waiting in a queue.

然后，请收起“工程在这里只是次要伙伴”的念头。在前沿地带，这两种工作已经融为一体。那个能自己搭建实验框架、评估管线与数据流水线的研究者，才是假说真正得到检验的人。其他人都在排队等候。

## Stare at the outputs

a descending loss curve is not analysis, it's reassurance. your experiments throw off far more information than you consume: transcripts, failure cases, the strange tail of the distribution. most of it dies unread in a logs folder.

一条下降的损失曲线不是分析，是安慰。你的实验抛出的信息，远比你消耗的多：模型生成的文本、失败案例、分布中奇特的尾部。其中大部分都未被阅读，便死在了日志文件夹里。

Karpathy's recipe starts before any training code gets written, with hours spent on the raw data by hand. most ml bugs live in the data, and they fail silently. nothing crashes. you simply get a mediocre model and a wrong theory about why.

卡帕西的秘方在写下任何训练代码之前就开始了，花数小时亲手处理原始数据。大多数机器学习缺陷都藏在数据里，而且它们会静默地失败。什么都不会崩溃，你只是得到一个平庸的模型，以及关于原因的、错误的理论。

andrew ng has taught the same unglamorous move for over a decade because nothing beats it. pull a hundred failures, read all of them, sort them into piles, attack the biggest pile. it works on models and it works on evals, where a benchmark you've never read transcripts from is a benchmark you don't actually understand. one transcript of genuinely strange behavior will teach you more than the next decimal of accuracy ever will.

吴恩达教授同一个毫无魅力可言的招式已有十余年，因为无出其右。抽取一百个失败案例，全部读完，把它们归类成堆，然后攻击最大的那堆。这对模型有效，对评估同样有效——一个你从未读过其输出文本的基准测试，就是一个你并未真正理解的基准测试。一份真正表现出奇怪行为的模型输出，教给你的东西，将比下一个精度的小数点多得多。
## Wander on purpose

your first subfield is an accident of timing, so treat it like one. spend real time in interpretability, in evals, in rl, in systems, before deciding where you live. somewhere in this field is a corner where your specific weirdness is an unfair advantage, and the only way to locate it is to pay tuition in several places. nobody waives the tuition.

你的第一个子领域，不过是时机造成的偶然，所以就这样对待它。在你决定在哪里定居之前，真正花些时间待在可解释性、评估、强化学习、系统里。这个领域的某处，有一个角落能让你特有的古怪成为不公平的竞争优势，而找到它的唯一方式就是在好几个地方交学费。没人会免除你的学费。

run the disposable version of every idea first and let most of them die young. tune your baselines until it hurts, because the graveyard of ml is full of gains that evaporated against a properly tuned baseline, and a reviewer is the worst possible person to learn that from. ablate until you know which component carries the result. it's usually one, and it's usually not the one in the title.

对每个想法，先运行一个可丢弃的版本，让它们中的大多数早早死掉。把你的基线调到痛不欲生，因为机器学习的坟场里，全是那种在面对精心调校的基线时便蒸发掉的提升，而审稿人是学这一课最糟糕的对象。做消融实验，直到你清楚哪个组件真正承载了那个结果。它通常只有一个，而且通常不是标题里的那个。

breadth is also insurance. subfields saturate, all of them, usually right after they peak on twitter. the people who keep producing through those transitions are the ones who already know their way around the neighboring territory.

广度也是一种保险。所有子领域都会饱和，无一例外，通常就在它们于推特上热度登顶之后。能在这些转变中持续产出的人，是那些已经熟悉邻近领域门道的人。
## find your people

Hamming noticed a pattern in who ended up doing important work. Colleagues with closed office doors got more done in any given year, and colleagues with open doors did the work that mattered, because the interruptions carried information about what the world actually needed. Your open door is probably an inbox. keep it that way.

汉明注意到，那些最终做出重要工作的人，有着某种模式。关着办公室门的同事，在任何单一年份里都能完成更多的事；而开着门的同事，做的却是真正重要的工作，因为那些打断携带着世界究竟需要什么的信息。你的敞开的门，大概就是一个收件箱。让它保持敞开。

Generosity compounds in research like nothing else. Replicate a result and publish what you find. release the tool you built for yourself. explain something hard in plain language. The returns arrive sideways, months later, as the collaboration or the reference or the role you couldn't have applied for. float your half-formed ideas in public too, because being wrong on the timeline is far cheaper than being wrong in print. and the collaborator who tells you an idea is bad before you sink three months into it is worth more than compute. that relationship can't be bought, only earned.

在研究中，慷慨的复利效应无与伦比。复现一个结果，并发表你的发现。发布你为自己构建的工具。用平实的语言解释困难的东西。回报会从侧面到来，在几个月后，以一次合作、一条引用或一个你本不可能申请的职位的形式。把你半成型想法也公开地放出来，因为在时间线上犯错，远比在印刷品上犯错代价低廉。还有，那个在你一头扎进去三个月之前告诉你某个想法很糟的合作者，比算力更有价值。这种关系，无法买到，只能挣来。
## The long game

Pasteur said luck favors the prepared mind, and hamming built a whole career philosophy on top of it: knowledge and productivity compound like interest. the daily edges look trivial in isolation. what you read, what you record, how fast your loop runs, who you argue with. give them a few years and they produce careers that look like luck from the outside. start compounding earlier than feels necessary. future you already knows this was the cheap part.

巴斯德说过，机遇偏爱有准备的头脑，而汉明在此之上构建了一整套职业哲学：知识和生产力的复利，如同利息。那些每日的微小优势，单独看来微不足道——你读什么，你记录什么，你的循环运转得多快，你和谁辩论。给它们几年时间，它们就会造就那些在外面看来如同运气的事业。在你感觉必要之前，就开始复利。未来的你早已知道，这是最便宜的部分。