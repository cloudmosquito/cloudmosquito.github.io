# git 相关操作

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
# 或vscode:
git config --global core.editor "code"
```
