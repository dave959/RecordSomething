## docker镜像源

在/etc/docker/daemon.json中加入

```
{"registry-mirrors":["https://1a1o6wl3.mirror.aliyuncs.com/"]}
```

```
https://docker.mirrors.ustc.edu.cn/    中国科技大学
https://hub-mirror.c.163.com/          网易云
https://1a1o6wl3.mirror.aliyuncs.com/  阿里云
https://reg-mirror.qiniu.com/          青牛
```

之后重新启动服务：

```
sudo systemctl daemon-reload
sudo systemctl restart docker
```



## LGSVL和Apollo仿真

### 1.配置Apollo

1）启动并进入apollo docker

​	启动docker（不要使用sudo运行以下命令）
```
./docker/scripts/dev_start.sh
```

​	进入docker
```
./docker/scripts/dev_into.sh
```

​	确保NVIDIA显卡在docker中能运行
```
nvidia-smi
```

​	用以下命令构建Apollo（优化，无调试，支持GPU，只需执行一次）
```
./apollo.sh build_opt_gpu
```
​	提示FATAL: mkdir('/apollo/.cache/bazel'): (error: 13): Permission denied	
​	需要使用sudo

2）与Apollo协同运行模拟器
​	启动Cyber bridge
```
./scripts/bridge.sh
```
​	此时bash并无输出，apollo在等待lgsvl发送消息过来。
​	启动lgsvl仿真器。	
打开新的命令行窗口，使用以下命令进入docker
```
./docker/scripts/dev_into.sh
```
使用```cyber_monitor```命令来查看消息，使用```cyber_visualizer```命令来可视化

3）运行Apollo模组

​	启动Dreamview
```
./scripts/bootstrap_lgsvl.sh
```
​	通过localhost:8888在浏览器中打开Dreamview
​	mode：Mkz_Lgsvl;
​	vehicle：Lincoln2017MKZ_LGSVL
​	map：Borregas_Ave
​	打开Module Controller
​	使能Localization, Transform, Perception, Traffic Light, Planning, Prediction, Routing and Control.
​	导航到Route Editing标签
​	通过单击车道线并单击Send Routing Request来选择目的地

