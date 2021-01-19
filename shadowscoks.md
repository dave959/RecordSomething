# ssh连接服务器

ip:108.160.136.239
username:root
bR}55Fq28bv4!?CD

```ssh root@108.160.136.239```

启动Shadowocks在后台运行
```ssserver -c /etc/shadowsocks.json -d start```

# shadowsocks配置文件

{
    "server":"108.160.136.239",
    "server_port":443,
    "password":"fsd+23",
    "timeout":600,
    "method":"aes-256-cfb",
    "fast_open": false
}

{
	"server":"45.32.34.207",
    "server_port":5495,
    "local_address": "127.0.0.1",
    "local_port":1080,
    "password":"56hsjv93",
    "timeout":300,
    "method":"aes-256-cfb",
    "fast_open": false,
    "workers": 1,
    "prefer_ipv6": false
}

# shadowscoks开机自启

```vim /etc/rc.local```
在rc.local文件中，“exit 0”行前添加
```ssserver -c /etc/shadowsocks.json -d start```
然后保持退出vim

执行以下三条指令
```
sudo chown root:root /etc/rc.local
sudo chomd 755 /etc/rc.local
sudo systemctl enable rc-local.service
```
重启后shadowsocks即可开机自启


[Unit]

Description=Shadowsocks Client Service

After=network.target

[Service]

Type=simple

User=root

ExecStart=/usr/bin/sslocal -c /etc/shadowsocks.json

[Install]

WantedBy=multi-user.target


## 启动Shadowocks在后台运行
```ssserver -c /etc/shadowsocks/config.json -d start```

## shadowsocks-qt5安装

```
chmod a + x Shadowsocks-Qt5-x86_64.AppImage
./Shadowsocks-Qt5-x86_64.AppImage
```

