# 一、项目介绍

## 1.1简介

实验室遗留了一个雷达，该项目用来测试雷达是否有问题，能否正常使用以及在其他项目中使用该雷达。

## 1.2历程

1. 在淘宝询问各卖该设备的商家是否有使用手册（都为二手贩子，都没有）。
2. 登入该厂家官网[企业介绍-深圳市镭神智能系统有限公司](https://www.leishen-lidar.com/about)，寻找对应资料。联系在线客服获取雷达的输入电压以及SDK。
3. 在中控中下载调试雷达。

## 1.3软硬件

雷达供电电压为24V，项目使用的软硬件型号如下

| 设备     | 型号            |
| :------- | :-------------- |
| 雷达设备 | 镭神智能N301-P  |
| 中控型号 | Jeston Orin AGX |

# 二、项目实现的功能

1. 获取雷达数据
2. 可视化雷达数据

# 三、如何部署使用

在doc目录中有ZH_linux_and_win_demo_SDK_v2.2.0说明文档。

## 3.1官方说明

**注：官方说明可能与雷达实际不一致，如果按官方说明测试失败请看3.2自测说明**

### 3.1.1准备工作

1. 通过网线把中控与雷达相连，并通过拔插网线并观察命令行中ifconfig的变换，确定有线连接设备名。
2. 把ip地址改为192.168.1.102，子网掩码为255.255.255.0，代码如下，其中eth1为有线连接设备名。
   `ifconfig eth1 192.168.1.102 netmask 255.255.255.0`
3. 如果没有事先安装好pcl点云库，可输入以下命令安装
   `sudo apt-get install libpcl-dev`

### 3.1.2编译和运行

1. 切换到文件夹所在目录，并执行以下代码进行编译

   ```linux
   cd build
   cmake ..
   make -j4
   ```

2. 进入生成可执行文件目录中

   ```
   cd ../bin
   ```

   | 作用                          | 命令               |
   | ----------------------------- | :----------------- |
   | 获取数据   demo               | ./demo             |
   | 可视化 demo_viewer            | ./demo_viewer      |
   | 可视化离线包 demo_viewer_Pcap | ./demo_viewer_Pcap |

   **注意：离线的文件要放到./demo/PcapPacketPath文件夹中并在源文件main_PCL_Pcap.cpp 中修改对应的文件名，修改离线包的名称**

## 3.2自测说明

1. 打开终端：ping 雷达 IP，测试硬件是否连接正常，若ping通则正常，否则检测硬件连接。

   ```
   ping 192.168.1.222
   ```

   ![1745466801701](https://gitee.com/JiangPeng-57/images/raw/master/1745466801701.jpg)

2. 使用tcpdump查看雷达发送数据包情况，如果显示雷达发送到目的端数据包为1206个字节，则表示雷达数据发送正常。（eth1为有线连接设备名）

   ```
   sudo tcpdump -ni eth1
   ```

   ![1745467808066](https://gitee.com/JiangPeng-57/images/raw/master/1745467808066.jpg)

   **注**：如果显示雷达发送到目的端数据包为28个字节，则为雷达数据传不到中控，原因可能为

   1. 网线连接问题
   2. 中控ip未能和雷达ip处在同一局域网，可使用ifconfig eth1查看中控设置的IP

3. 由步骤2中IP 192.168.1.22.2369 > 192.168.1.125.2368: UDP可知

   雷达ip为192.168.1.22，发送消息的端口号为2369

   中控ip为192.168.1.125，接收消息的端口号为2368

   使用UDP协议进行通讯

   **注**：这里我把eth1的ip改为了192.168.1.102，并使用tcpdump进行抓包，也可接收到雷达数据，因为它们此刻处于一个局域网。

   ![1745468478802](https://gitee.com/JiangPeng-57/images/raw/master/1745468478802.jpg)

4. 打开demo文件夹中的main.cpp和main_PCL.cpp，把以下ip和端口修改为步骤3中雷达pi、端口和中控ip、端口
   ![1745470995904](https://gitee.com/JiangPeng-57/images/raw/master/1745470995904.jpg)

   ![1745471099012](https://gitee.com/JiangPeng-57/images/raw/master/1745471099012.jpg)

5. 按3.1.2编译和运行步骤重新编译运行，下图为执行./demo_viewer后的可视化结果图，蓝色点为检测到的障碍物。
   ![d3be008acac1a7334d2640ac253a590](https://gitee.com/JiangPeng-57/images/raw/master/d3be008acac1a7334d2640ac253a590.png)

   注：如果输出为Failed to bind socket，可能是端口2368被占

   ![1745472075493](https://gitee.com/JiangPeng-57/images/raw/master/1745472075493.jpg)

   使用lsof命令查出占用2368端口的进程，并使用kill命令进行消灭

   ```
   sudo lsof -i :2368  # 检查2368端口占用
   sudo kill -9 XXXX  #XXXX为sudo lsof -i :2368命令输出的进程号
   ```

# 四、代码组织结构

| bin            | 编译生成的可执行文件                      |                                                              |
| :------------- | ----------------------------------------- | ------------------------------------------------------------ |
| build          | 编译的目录                                |                                                              |
| include        | 头文件和源文件的目录                      |                                                              |
| doc            | 说明文档的路径                            |                                                              |
| demo           | main.cpp                                  | 一般般示例，获取点云的文件， 输出点云数据                    |
| demo           | main_PCL.cpp                              | 获取点云的文件，增加PCL 可视化的库，显示点云                 |
| demo           | main_PCL_Pcap.cpp                         | 获取点云的文件，增加PCL 可视化的库，pcap的离线解 析示例，显示点云: 注意：离线的文件要放到 demo/PcapPacketPath文件 夹内并在源文件中修改对应 的文件main_PCL_Pcap.cpp 内修改离线包的名称 |
| CMakeLists.txt | Cmake文件，配置编译项目<br />配置编译规则 |                                                              |



