# 怎么换源 pip install

在 Ubuntu 中修改 `pip` 的安装源为国内镜像源（如清华、阿里云、中科大等），可以通过以下两种方法实现：

---

## 方法 1：临时指定国内源

在安装包时直接通过 `-i` 参数指定镜像源地址：

```bash
pip install 包名 -i https://pypi.tuna.tsinghua.edu.cn/simple
```

常用国内源地址：

- 清华源：`https://pypi.tuna.tsinghua.edu.cn/simple`
- 阿里云：`https://mirrors.aliyun.com/pypi/simple`
- 中科大：`https://pypi.mirrors.ustc.edu.cn/simple`

---

## 方法 2：永久修改为国内源

通过修改 `pip` 的配置文件，永久生效。

---

### 步骤 1：创建或修改配置文件

- 对于当前用户：
  ```bash
  mkdir -p ~/.pip  # 如果目录不存在则创建
  nano ~/.pip/pip.conf
  ```

  或（新版本 pip 可能使用）：

  ```bash
  mkdir -p ~/.config/pip
  nano ~/.config/pip/pip.conf
  ```

- 对于系统全局配置（需管理员权限）：
  ```bash
  sudo nano /etc/pip.conf
  ```

### 步骤 2：写入镜像源配置

在文件中添加以下内容（以清华源为例）：

```ini
[global]
index-url = https://pypi.tuna.tsinghua.edu.cn/simple
trusted-host = pypi.tuna.tsinghua.edu.cn
```

- `index-url`：指定镜像源地址。
- `trusted-host`：标记镜像源为可信，避免 HTTPS 证书问题。

### 步骤 3：保存并退出

按 `Ctrl + O` 保存，再按 `Ctrl + X` 退出编辑器。

---

### 另一种做法

也可以不用这么麻烦，直接在命令行中执行以下命令

```bash
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
pip config set global.trusted-host pypi.tuna.tsinghua.edu.cn
```

---

## 验证配置

运行以下命令查看当前配置：

```bash
pip config list
```

或安装测试包观察下载源：

```bash
pip install numpy
```

---

## 其他注意事项

1. **虚拟环境**：如果在虚拟环境（如 `venv` 或 `conda`）中使用，需在虚拟环境中单独配置。
2. **pip 版本**：确保使用 `pip3` 对应 Python 3（如系统默认是 Python 2）。
3. **清除缓存**：若之前下载过包，可能需要清除缓存：
   ```bash
   pip cache purge
   ```

---

常用国内镜像源替换示例：

```ini
# 阿里云
index-url = https://mirrors.aliyun.com/pypi/simple
trusted-host = mirrors.aliyun.com

# 中科大
index-url = https://pypi.mirrors.ustc.edu.cn/simple
trusted-host = pypi.mirrors.ustc.edu.cn
```

按需选择镜像源即可提升下载速度！
