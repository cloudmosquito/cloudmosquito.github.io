# uv 常用命令

## 现代做法

首先：

```shell
uv init # 在已有文件夹下
uv init <folder_name> # 新建一个文件夹
```

uv 会创建以下文件：

```shell
.
├── .python-version
├── README.md
├── main.py
└── pyproject.toml
```

接下来，在首次运行项目命令，比如 `uv run` `uv sync` `uv lock` `uv add` 的时候，uv 还会在项目根目录中创建一个虚拟环境和 `uv.lock` 文件。

```shell
.
├── .venv
│   ├── bin
│   ├── lib
│   └── pyvenv.cfg
├── .python-version
├── README.md
├── main.py
├── pyproject.toml
└── uv.lock
```

然后，用 `uv add <package_name>` 添加依赖，例如

```shell
uv add --dev Zensical # 
```

这本质上是在修改 `pyproject.toml` 的内容，并下载包。`--dev` 就是将 Zensical 添加到 `[dependency-groups]` 字段下。

---

如果我们拿到别人的一个项目，只要里面有 pyproject.toml 和 uv.lock ，就可以 `uv sync` 一键配环境！

## 古典派做法（向前兼容）

针对当前文件夹：

```shell
# 新建一个文件夹，用于管理该文件夹的Python环境
uv venv <文件夹名，默认规范 .venv> 

.venv\Scripts\activate.ps1 # 激活该环境(Powershell)
.venv\Scripts\activate.bat # 激活该环境(CMD)
```

!!! tip

    在 windows 的 Powershell 下，可能会遇到权限不够的问题，报错为：

    .\\.venv\Scripts\activate.ps1 : 无法加载文件 D:\lky\cloudmosquito.github.io\.venv\Scripts\activate.ps1，因为在此系统上禁止运行脚本。有关详细信息，请参阅 https:/go.microsoft.com/fwlink/?LinkID=135170 中的 about_Execution_Policies。

    解决方法为用管理员权限重新打开一个 Powershell 终端，更改当前用户的权限：

    ```shell
    #该命令的作用是允许当前用户运行本地创建的脚本，但远程下载的未签名脚本仍会被拦截
    Set-ExecutionPolicy RemoteSigned -Scope CurrentUser
    ```


激活成功后，终端前缀会显示 `(.venv)`，比如：`(.venv) PS D:\my-mkdocs-project>`。

接下来，uv 支持 pip install 安装 Python 包，速度超快！

```shell
uv pip install <pkg_name>
```

要退出当前环境也很容易，只需要 `deactivate` 即可。

下次使用时，还是按照老方法运行脚本启动环境。

如果要更新某个包，可以 `uv pip install --upgrade <pkg_name>` 。