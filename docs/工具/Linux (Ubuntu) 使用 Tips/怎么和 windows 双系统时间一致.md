# 怎么设置双系统时间一致

默认情况下，Ubuntu（和大多数其他 Linux 发行版）假设硬件时钟设置为协调世界时间（UTC + 0），而 Windows 则假设硬件时钟设置为当地时间，这导致 Ubuntu 快 8 小时。

这种差异会导致你在双启动系统中切换操作系统时，经常遇到时间显示不正确的问题。要解决这个问题，有两种常用方法，要么让 Linux 将就 Windows，要么让 Windows 将就 Linux，考虑到注册表经不起折腾，还是修改 Linux 为好。

## 修改 Ubuntu 设置使硬件时钟使用本地时间

首先设置时间，让时间和 [http://time.windows.com](https://link.zhihu.com/?target=http%3A//time.windows.com) 同步：

```text
sudo apt install ntpdate
sudo ntpdate time.windows.com
sudo hwclock --localtime --systohc
```

此时系统时间应该减去了八小时。

你可以在 Ubuntu 中调整设置，使其将硬件时钟视为本地时间，这样就与 Windows 保持一致。要做到这一点，你可以打开终端并使用以下命令：

```bash
timedatectl set-local-rtc 1 --adjust-system-clock
```

这个命令会修改 Ubuntu 的时间处理方式，使其假定硬件时钟是以本地时间运行的。这样两系统的时间均显示正确，且不会出现其他问题