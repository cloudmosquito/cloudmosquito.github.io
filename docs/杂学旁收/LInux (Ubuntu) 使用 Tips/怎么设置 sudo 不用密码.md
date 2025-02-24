# sudo 不用密码

```shell
sudo visudo # 打开/etc/sudoers.tmp
```

把 `%admin` 和 `%sudo` 所在行改成：

```plaintext
%admin ALL=(ALL) NOPASSWD: ALL
%sudo  ALL=(ALL:ALL) NOPASSWD: ALL
```
