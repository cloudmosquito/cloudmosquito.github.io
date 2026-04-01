# git 相关操作

> 推荐使用：VS Code + GitLens (paid) / \[ Git Graph + Git Blame \](free) .

## 配置

在 Ubuntu 中配置 Git 主要涉及以下常见操作，包括设置用户信息、配置 SSH 密钥、设置代理等。以下是详细步骤：

### **1. 安装 Git**

```bash
sudo apt update
sudo apt install git
```

### **2. 配置全局用户信息**

设置提交代码时的用户名和邮箱（全局生效）：

```bash
git config --global user.name "你的用户名"
git config --global user.email "你的邮箱"
```

### **3. 配置 SSH 密钥（推荐）**

通过 SSH 密钥可以免密操作远程仓库（如 GitHub、Gitee）。

#### **步骤 1：生成 SSH 密钥**

```bash
ssh-keygen -t ed25519 -C "你的邮箱"
```

- 按回车使用默认路径（`~/.ssh/id_ed25519`）。
- 可选：设置密钥密码（直接回车留空则无密码）。

#### **步骤 2：将公钥添加到远程仓库**

- 查看公钥内容：
  
  ```bash
  cat ~/.ssh/id_ed25519.pub
  ```

- 复制公钥内容（以 `ssh-ed25519` 开头），添加到：
  - **GitHub**：`Settings -> SSH and GPG keys -> New SSH key`
  - **Gitee**：`设置 -> SSH 公钥`

#### **步骤 3：测试 SSH 连接**

```bash
ssh -T git@github.com  # 测试 GitHub
ssh -T git@gitee.com   # 测试 Gitee
```

成功时会显示类似：`Hi username! You've successfully authenticated.`

### **4. 配置 Git 代理**

如果访问 GitHub 等仓库较慢，可以设置代理（需提前安装代理工具如 Clash）。

#### **设置 HTTP/HTTPS 代理**

```bash
git config --global http.proxy http://127.0.0.1:7890
git config --global https.proxy https://127.0.0.1:7890
```

- `7890` 是代理端口号，根据实际工具调整（如 Clash 默认为 `7890`）。

#### **设置 SSH 代理**

编辑 `~/.ssh/config` 文件（没有则新建）：

```bash
Host github.com
  HostName github.com
  User git
  ProxyCommand nc -x 127.0.0.1:7890 %h %p
```

#### **取消代理**

```bash
git config --global --unset http.proxy
git config --global --unset https.proxy
```

### **5. 配置 Git 默认编辑器**

设置提交代码时的默认编辑器（如 Vim、Nano）：

```bash
git config --global core.editor "vim"
# 或vscode:
git config --global core.editor "code"
```

### **6. 配置换行符处理**

避免跨平台（Windows/Linux/macOS）换行符问题：

```bash
git config --global core.autocrlf input  # Linux/macOS 推荐
git config --global core.autocrlf true   # Windows 推荐
```

### **验证所有配置**

```bash
git config --global --list
```

### **配置文件路径**

- 全局配置：`~/.gitconfig`
- 单个仓库配置：仓库目录下的 `.git/config`

按需调整配置即可提升 Git 使用效率！

---

## 基础操作

### 提交、新建分支、更改当前分支所在位置等

```shell
git add . # 暂存所有更改
git commit -m "commit message" # 提交
```

---

```shell
git fetch # 拉取最新远程信息
git merge # 将远程代码和本地代码合并
git push  # 推送到远程
git pull  # 等价于 git fetch + git merge
```

第一次推送到远程仓库：

```shell
git push -u origin <new_branch_name>
```

其中，`-u` 是 `--set-upstream` 的简写，它的作用是把本地分支和远端分支建立“上游/跟踪”关系（upstream），以后切到该分支，直接执行 `git push` 或 `git pull` 就会默认把改动推/拉到 `origin/new_branch_name`。

在此基础上，`git status` 会显示 `ahead` `behind` 相对 `origin/new_branch_name` 的状态。

如果本地分支名和远程分支名不同，可以用变体命令：

```shell
git push -u origin local-name:remote-name
```

---

```shell
git stash # 把当前修改存起来
git stash pop # 把存起来的修改拿出来
```

---

### 分支相关

切换到已有分支

```shell
git switch <branch>
git checkout <branch>
```

新建并切换到新分支

```shell
git switch -c <new>
git checkout -b <new>
```

新建孤儿分支

```shell
git checkout --orphan <new-branch-name>
```

切到某个提交（会出现 detached HEAD ）

```shell
git switch --detach <commit>
git checkout <commit>
```

> detached HEAD 指的是当前工作环境（HEAD）处于一个具体的提交或者一个没有名字的分支，在此基础上做的更改可能会丢失。


从远程分支创建并跟踪

```shell
git switch --track <branch>  
git checkout --track origin/branch

# 在本地创建一个分支并关联远程分支，同时切过去
git switch -c <local-branch-name> origin/<remote-branch-name> 
```

看哪个本地分支在跟踪哪个远端分支

```shell
git branch -vv
```

删除分支

```shell
# 本地分支
git branch -d <branch-name>  # 已merge的分支
git branch -D <branch-name>  # 未merge的分支
# 远程分支
git push origin --delete <branch-name>
```


## 撤销操作

查看提交记录：

```shell
git reflog
```

回到某一次状态：

```shell
git reset --hard HEAD@{3} # 或者 git reset --hard a1b2c3d
```

---

## 子模块

```shell
# 添加子模块
git submodule add [url]

# clone一个仓库后，递归地完成子模块初始化和更新
git submodule update --init --recursive

# 更新当前项目中的所有子模块
git submodule foreach "git submodule update"
```

## 撤回远程提交

当想删除某次 push 到远程的 commit 时，可以先 reset 到之前一次提交 (可以通过 VS Code 的图形化界面操作)，然后强行 push 上去。( VS Code 没有提交 commit 回退的选项。)

```shell
git reset --hard 12xasd # 提交编号
git push --force # 这里没有显式指明远程分支，推送到默认远程分支上。推荐还是显式指定一下，比如：
git push origin --force
```

这里的 `--force` 是无条件用本地分支覆盖远程分支历史；在团队协作中，如果别人在你更新前推了一版上去，那么他的更新会被覆盖，这不好。因此，团队协作更推荐使用 `--force-with-lease`，它会先检查远程分支是否等于你本地最后一次获知的值，若否，则放弃覆盖。

## worktree相关操作

worktree可以管理多个文件夹，每个文件夹对应一个分支，从而同时进行多分支的开发。worktree管理的多个分支共享一个.git，相较于多次clone而言，大大节省内存。

```shell
git worktree list # 查看worktree中管理了几个分支

git worktree add <path> <branch-name> # 把 <branch-name> 分支放到 <path>

git worktree remove <path>
```

## 7 `git rebase`操作

要处理当前提交到某一次先前的提交，可以先

```shell
git rebase -i <commit>~
# 以上，-i 代表Make a list of the commits which are about to be rebased.在git打开的文本文件中，你可以对先前的很多提交做处理
# ~ 表示包括<commit>本身
```

会跳出一个文本界面，跟着提示做就好了，关闭文本界面就代表完成一次编辑。

接下来，建议 `git status` 看一下是否处于 detached HEAD 。若是，则需要先

```shell
git branch -f <branch_name> HEAD # 把某个分支强制指向当前HEAD
git switch <branch_name> # 或者旧版 git 用 git checkout <branch_name> 切到该分支
git push origin --force-with-lease
```

或者

```shell
git switch -c <new_branch_name> # 在当前 HEAD 创建新分支并切到该分支
git push -u origin <new_branch_name> # 推送到远程
```

此外，如果更改过程中出现问题，以下是一些可能会用到的指令：

```shell
git rebase abort # 退出当前正在进行的 git rebase

git checkout -- . # 【注意】有个点。放弃当前文件夹下所有更改，但似乎对新建文件夹无效
git clean -fd # 清除所有没暂存的文件
```