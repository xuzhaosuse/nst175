# NST175

## 1、介绍

这是一个在RT-Thread上，NST175温度传感器的驱动。 实现了高精度温度的读取，并支持`Finsh/MSH`测试命令。
本驱动编程实现方便读取多个NST175传感器。地址IO的控制留给用户自行编程处理。

### 1.1 目录结构

| 名称 | 说明nst175 |
| ---- | ---- |
| nst175.h | 头文件 |
| nst175.c | 源代码 |
| SConscript | RT-Thread 默认的构建脚本 |
| README.md | 软件包使用说明 |

### 1.2 许可证

NST175 package 遵循 `LGPLv2.1` 许可，详见 `LICENSE` 文件。

### 1.3 依赖

依赖 `RT-Thread I2C` 设备驱动框架。

## 2、如何获取软件包

使用 nst175 package 需要在 RT-Thread 的包管理器中选择它，具体路径如下：

```
RT-Thread online packages
    peripheral libraries and drivers  --->
        [*] nst175: digital temperature sensor nst175  driver library
```

然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。

## 3、使用 nst175

本驱动实现基于RT-Thread的I2C设备驱动框架，在打开 nst175 package 后，请务必打开至少1个I2C设备。


## 4、注意事项

- 请注意nst175芯片的地址引脚默认地址 0x48,0x49,0x4a

## 5、联系方式 & 感谢

* 维护：站在烈日下乘凉
* 邮箱：xuzhaosuse@163.com
* 主页：https://github.com/xuzhaosuse/nst175
