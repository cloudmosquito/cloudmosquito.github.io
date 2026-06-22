# WSL2 踩坑经历

Windows 的文件都在以下路径里：

```shell
/mnt/<磁盘名，如c,d,e等>/
```

## 代理

### 方法1

首先，windows 上的代理软件需要允许来自局域网的连接。

然后，在 C:\Users\UserName 下新建一个 .wslconfig 文件，内容为：

```
[wsl2]
networkingMode=mirrored
autoProxy=true
```

**问题**：采用以上操作之后，windows 的 VS Code 将无法连上 wsl2 .

### 方法2

首先，windows 上的代理软件需要允许来自局域网的连接。

其次，把以下内容写入 `~/.bashrc` 。（需要将其中的端口号修改成代理软件监听的端口号。）

```
WIN_HOST_IP=$(ip route show | grep -i default | awk '{ print $3}')
export HTTP_PROXY="http://${WIN_HOST_IP}:7890"   # 端口号 7890 需修改
export HTTPS_PROXY="http://${WIN_HOST_IP}:7890"  # 端口号 7890 需修改
export http_proxy="$HTTP_PROXY"
export https_proxy="$HTTPS_PROXY"
export NO_PROXY="localhost,127.0.0.1,::1"
export no_proxy="$NO_PROXY"
```