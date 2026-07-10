---
title: 激光测距AD模块PCB学习日志
tags:
  - STM32
  - FMCW
  - 嵌入式
  - PCB
categories:
  - 嵌入式
description: AD信号采集、频率法及相位法测距算法解算，H750VB平台PCB设计计划。
cover: /img/pcb0710.png
abbrlink: b4c1c9ec
date: 2026-07-10 17:49:00
---

## PCB学习日志

### 1、核心功能

AD信号采集，频率法及相位法测距算法解算，结果输出

### 2、核心模块

USB接口、调试接口、LDO稳压、按键、复位电路、信号接口、外部晶振、LED

### 3、拓展板模块

信号产生（DAC、PWM）、特殊信号接口、OLED屏幕、电源接口、额外按键、CAN总线、日志与基础通信组、留空自由GPIO、LED

### 4、目前旧版原理图需要修改问题

- 运放输出端与GND之间加一个3.3V的稳压管。
- BOOT0通过电阻直接下拉到了GND，需要给BOOT0增加一个排针跳线帽，或者一个按键。当需要通过USB烧录时，将BOOT0拉高到3.3V，复位后即可进入Bootloader。
- 模拟电源（VDDA/VREF+）缺乏独立滤波。

![原理图](/img/原理图0710.png)

![PCB](/img/pcb0710.png)
