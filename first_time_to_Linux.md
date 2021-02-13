# 一、常用指令
## 1. 安装指令
### 1）deb文件
```sudo dpkg -i ***.deb```

### 2）run文件
方法1：```sudo sh ***.run```
方法2：
添加执行权限：```sudo chmod +x ***.run```
运行：```sudo ./***.run```

### 3） bundle（sh、bin）文件安装
1、进入文件目录，赋予文件执行权限：```sudo chmod +x ***.bundle```
2、安装文件：```sudo ./***.bundle```



## 2.其它指令
### 1）root权限
在Ubuntu终端输入sudo加命令可以实现，如果需要root则使用su，即```sudo su```直接把$变成#。退出时使用: ``` exit```。

### 2）解压
tar.xz结尾的压缩文件，需要解压的话:
一、通过 yum装个baixz压缩包，然后du解压，操作如下：
yum search xz
xz -d 你的文件.tar.xz
tar -xvf 你的文件.tar.xz
二、创建zhi或解压tar.xz文件的方法
习惯了 tar czvf 或 tar xzvf 的人可能碰到 tar.xz也会想用单一命令搞定解压或压缩。
其实不行 tar里面没有征对xz格式的参数比如 z是针对 gzip，j是针对 bzip2。
创建tar.xz文件：只要先 tar cvf xxx.tar xxx/ 这样创建xxx.tar文件先，然后使用 xz -z xxx.tar 来将 xxx.tar压缩成为 xxx.tar.xz


# 二、建议安装

## 1、Dia

流程图软件
```sudo apt-get dia```

## 2、主题

1）安装tweak tool
```sudo apt install gnome-tweak-tool```
2）安装user themes
```sudo apt install gnome-shell-extensions```
3）在tweak的Extensions中确保User themes开启
- 主题：放置到`~/.themes`目录中
- 图标：放置到`/usr/share/icons/`目录中
- 指针：放置到`~/.icons`目录中

## 3、壁纸
**Wallch的特点**
Live Earth - Live Earth Wallpaper将您的桌面背景设置为地球的“实时”壁纸。
Picture of the day- 当天的图片设置为您的dekstop背景从维基百科选择的图片，每日更新！
Wallpaper Clocks- 壁纸时钟将美丽的壁纸与时间和日期结合起来。有各种壁纸时钟，你可以选择。从VladStudio.com挑选你的最爱。 Wallch负责在您的系统中安装壁纸时钟和壁纸。
Live Website- 实时网站可以设置您的桌面背景您选择的网页。这是Wallch 3.0发布之后的一个早期开发的功能，现在它已经发展并允许您登录到各种各样的网站并截取其他受限制的页面。实时网站还允许您裁剪页面的一部分。
**安装Wallch**
```sudo apt-get install wallch -y```

## 4、Steam\
```sudo apt install steam```

## 5、Typora安装

在终端输入以下命令：
```
wget -qO - https://typora.io/linux/public-key.asc | sudo apt-key add -
sudo add-apt-repository 'deb https://typora.io/linux ./'
sudo apt-get update
sudo apt-get install typora
```

# 三、异常

## 1、Ubuntu更新错误“等待unattended-upgr退出”
1）停止自动更新程序。
```sudo dpkg-reconfigure -plow unattended-upgrades```  
在第一个提示下，选择不下载并安装更新。重新启动。

2）确保正确安装了处于不干净状态的所有软件包。
```sudo dpkg --configure -a```  

3）获取系统up-top-date。
```sudo apt update && sudo apt -f install && sudo apt full-upgrade```  

4）清除障碍后，重新打开自动更新程序。
```sudo dpkg-reconfigure -plow unattended-upgrades```  
再次选择软件包unattended-upgrades

## 2、改变git默认编辑器为vscode
```git config --global core.editor "code -w"```
