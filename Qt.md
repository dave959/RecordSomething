## 1、Visual Studio + Qt 编译Release版本时出现如下无法找到入口错误
![](/home/dave/Code/RecordSomething/images/CANZ无法找到入口.png)

使用  
```D:\Qt\5.15.2\msvc2019_64\bin\windeployqt.exe D:\QtApl\CANZ.exe```  
自动生成需要文件，然后将文件拷贝到项目目录.\CANZ\x64\Release下。



## 2、windows下qt+vs生成.exe程序图标
### vs+qt项目中如何设置.exe图标
1.准备好一张图片，在网上找个图片在线转换网站(https://www.easyicon.net/covert/)转换成.ico文件
2.项目右键->添加->资源->icon->导入->选中.ico->打开->重新编译.exe就有图标了

### Qt Creator项目中如何设置.exe图标
1、准备一张.ico文件，例如名称为obj.ico;
2、在项目工程路径下新建一个ico文件夹，将obj.ico拷贝到这个文件夹中；
3、在项目工程路径（快速定位：项目中右击->在Explorer中显示）下新建立一个文本文件，重命名为obj.rc，在文件中添加：
    IDI_ICON ICON  DISCARDABLE   "./ico/obj.ico"  添加完后保存文件
4、双击.pro文件。在文件的最后面添加：RC_FILE+=obj.rc
3、重新运行程序就可以看到.exe文件上有图标了
注意：一定要把.png或者其他格式图片用工具转换成.ico，千万不要只改后缀名。

## 3、tr()函数未识别
查看一下，使用tr的地方所在的类是否继承自QObject类或者在不在某一类中，如果不是继承自QObject或根本不在一个类中，那么就直接用类名引用QObject::tr( )