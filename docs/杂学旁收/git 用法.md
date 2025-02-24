# git 相关操作

---

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

## 1 基础操作

提交、新建分支、更改当前分支所在位置等（略）。

## 2 子模块

```shell
# 添加子模块
git submodule add [url]

# clone一个仓库后，递归地完成子模块初始化和更新
git submodule update --init --recursive

# 更新当前项目中的所有子模块
git submodule foreach "git submodule update"
```

## 3

当想删除某次 push 到远程的 commit 时，可以先 reset 到之前一次提交 (可以通过 VS Code 的图形化界面操作)，然后强行 push 上去。( VS Code 没有提交 commit 回退的选项。)

```shell
git reset --hard 12xasd # 提交编号
git push --force # 这里没有显式指明远程分支，推送到默认远程分支上。推荐还是显式指定一下，比如：
git push origin --force
```

## 4 删除分支

```shell
# 本地分支
git branch -d <branch-name>  # 已merge的分支
git branch -D <branch-name>  # 未merge的分支
# 远程分支
git push origin --delete <branch-name>
```

## 5 worktree相关操作

worktree可以管理多个文件夹，每个文件夹对应一个分支，从而同时进行多分支的开发。worktree管理的多个分支共享一个.git，相较于多次clone而言，大大节省内存。

```shell
git worktree list # 查看worktree中管理了几个分支

git worktree add <path> <branch-name> # 把 <branch-name> 分支放到 <path>

git worktree remove <path>
```

## 6 `git rebase`操作

要处理当前提交到某一次先前的提交，可以先

```shell
git rebase -i <commit>~
# 以上，-i 代表Make a list of the commits which are about to be rebased.在git打开的文本文件中，你可以对先前的很多提交做处理
# ~ 表示包括<commit>本身
```

会跳出一个文本界面，跟着提示做就好了，关闭文本界面就代表完成一次编辑

```shell
git rebase abort # 退出当前正在进行的 git rebase
```

```shell
git checkout -- . # 【注意】有个点。放弃当前文件夹下所有更改，但似乎对新建文件夹无效
git clean -fd # 清除所有没暂存的文件
```

## 7 IDE

在某些系统（比如ubuntu），默认的IDE似乎是nano，需要切换成你熟悉的IDE

```shell
# 如vim:
git config --global core.editor "vim"

```
