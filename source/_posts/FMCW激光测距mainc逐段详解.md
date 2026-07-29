---
title: FMCW 激光测距 main.c 逐段详解
tags:
  - STM32
  - FMCW
  - 嵌入式
  - DSP
  - H750
categories:
  - 嵌入式
description: 逐段拆解 STM32H750 FMCW 激光测距代码
abbrlink: 4460cbdf
date: 2026-07-29 17:30:00
---

# FMCW 激光测距 main.c 逐段详解

---

## 第一部分：文件头与包含

### 文件头注释 (1-7)
```c
/**
  * FMCW 激光测距 
  * 启动检测同步 → 按键切模式
  * KEY0: 波形↔测距  WKUP: 校准相对距离零点
  * 测距: USB输出频率/绝对距离/相对距离
  */
```
功能描述，说明系统支持两种工作模式（波形采集/测距）、WKUP按键用来归零。

### CubeMX自动生成的头文件 (根据ioc自动生成)
```c
#include "main.h"       // HAL库主头文件，包含HAL_StatusTypeDef、外设句柄类型等
#include "adc.h"        // ADC1初始化声明 (MX_ADC1_Init)
#include "dma.h"        // DMA初始化声明 (MX_DMA_Init)，ADC→内存需要DMA
#include "tim.h"        // TIM2初始化声明 (MX_TIM2_Init)，200kHz触发ADC
#include "usb_device.h" // USB设备初始化声明 (MX_USB_DEVICE_Init)
#include "gpio.h"       // GPIO初始化声明 (MX_GPIO_Init)，引脚配置
```


### 手动添加的头文件
```c
#include <stdio.h>         // printf/sprintf/snprintf
#include <math.h>          // sqrtf/fabsf/atan2f/roundf
#include <string.h>        // memset等
#include "arm_math.h"      // CMSIS-DSP库: arm_rfft_fast_f32 / arm_cmplx_mag_f32 / arm_cos_f32
#include "usbd_cdc_if.h"   // USB CDC接口: CDC_Transmit_FS函数
#include "usbd_cdc.h"      // USB CDC类驱动: USBD_CDC_HandleTypeDef结构体
extern USBD_HandleTypeDef hUsbDeviceFS;  // 全局USB句柄，main.c需要读取它的状态
```

---

## 第二部分：结构体与宏定义 

### 自定义结构体 
```c
typedef struct{float32_t amp,phase,dc,rmse;}SineFitResult;
```
LMS 正弦拟合的一次返回结果：
- `amp`：拟合正弦波幅度
- `phase`：拟合初相位 (rad)
- `dc`：直流偏置
- `rmse`：均方根误差（拟合质量的评判标准，搜索最优频率时就是靠这个比大小）

### FMCW 物理参数宏 
```c
#define SAMPLE_RATE     1000000.0f   // 采样率 1MHz (但TIM2实际是200kHz)
#define RAMP_TIME_TOTAL 0.001f       // 激光调频上升沿时间 1ms
#define VALID_POINTS    1000         // 有效采样点数 (1ms × 1MHz = 1000)
#define FFT_LENGTH      4096         // FFT点数 (补零到4096，提高频率分辨率)
#define WAVELENGTH      1552.0e-9f   // 激光波长 1552nm (C波段)
#define LIGHT_SPEED     299792458.0f // 光速 m/s
#define FREQ_DEV        4260000000.0f // 激光扫频带宽 4.26GHz
```
这些宏是整个测距公式的物理基础。**改任何一个都会影响最终距离计算结果。**

> 公式关系：频率分辨率 = SAMPLE_RATE/FFT_LENGTH = 1M/4096 ≈ 244Hz/bin  
> 补零到4096的意义：1000点只够分辨1000Hz/bin，补零后等效244Hz/bin，频率分辨率提高4倍

---

## 第三部分：全局变量 

```c
uint16_t *buf=(uint16_t*)0x30000000;   // DMA目标地址，指向SRAM1 (0x30000000)
```
**核心缓冲区**。DMA 把 ADC 数据直接搬运到这里。为什么是 `0x30000000`？
- H750/H743 有多块 SRAM，SRAM1 起始于 `0x30000000`
- 通过 MPU 关闭了这片区域的 Cache（见 `mpu_buf_init`），保证 CPU 读到的是 DMA 最新写入的物理内存，而不是 Cache 里的旧数据

```c
char wf_out[8192];          // USB发送波形的字符串缓冲 (1000个点×"XXXX\r\n" ≈ 8000字节)
float32_t fft_in[FFT_LENGTH];   // FFT输入: 4096个float (16KB)
float32_t fft_out[FFT_LENGTH];  // FFT输出: 4096个float (复数排列)
float32_t fft_mag[FFT_LENGTH/2];// 频谱幅度: 2048个float (8KB)
float32_t lms_in[VALID_POINTS]; // LMS输入: 1000个float (去直流但未加窗的原始数据)
```
内存占用：FFT 相关约 48KB + 波形缓冲 8KB + LMS缓冲 4KB = 约 60KB。H750有1MB SRAM，轻松。

```c
arm_rfft_fast_instance_f32 fft_inst; // CMSIS-DSP FFT实例 (存旋转因子表等)
volatile uint8_t full=0;             // DMA完成标志: ADC转换完成中断里置1
uint8_t sync_ok=0;                   // 同步信号检测结果: 1=有同步信号
float32_t ref_dist=0;                // 相对距离零点 (WKUP按键设置)
uint32_t k0_tm,wk_tm;               // 按键去抖计时
```

---

## 第四部分：底层驱动函数 

### ADC 完成回调 
```c
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*h){
    if(h->Instance==ADC1) full=1;   // ADC1的DMA搬运完成后硬件自动调用
}
```
这是 HAL 库的中断回调。`HAL_ADC_Start_DMA` 启动后，DMA 把 1000 个采样点搬到 `buf`，搬完自动进 ADC 中断 → HAL 处理 → 调用此函数 → `full=1`。此时主循环的 `while(!full && --dto)` 就退出等待了。

### USB 发送函数 
```c
void usb_send(char*s,int n){
    if(hUsbDeviceFS.dev_state!=USBD_STATE_CONFIGURED)return;  // USB还没连好? 直接返回
    USBD_CDC_HandleTypeDef*h=(USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if(!h)return;
    uint32_t to=50000000;
    while(h->TxState!=0 && --to){}  // 等上一包发完 (超时保护)
    if(to>0) CDC_Transmit_FS((uint8_t*)s,n);  // USB CDC发送
}
```
`h->TxState`：USB 发送状态机，0=空闲，1=正在发。必须先等它变0才能发下一包，否则数据会错乱或丢失。

### printf 劫持 
```c
#ifdef __GNUC__
int _write(int f,char*p,int n){usb_send(p,n);return n;}
#endif
```
GCC 编译时，`printf` 底层调用 `_write`。重写这个函数，把 printf 的输出全部转到 USB CDC。效果：`printf("Hello")` → USB 串口收到 "Hello"。**比用串口转USB方便得多。**

### DWT 精确延时 
```c
void DWT_Init(void){
    // 打开ARM内核的调试单元，启动硬件周期计数器
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 使能调试模块
    DWT->LAR = 0xC5ACCE55;                            // 解锁DWT寄存器 (软件解锁序列)
    DWT->CYCCNT = 0;                                   // 计数器清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // 启动周期计数
}
void Delay_us(uint32_t us){
    uint32_t s = DWT->CYCCNT;                          // 当前计数值
    uint32_t d = us * (SystemCoreClock/1000000);       // 要等的周期数
    while((DWT->CYCCNT - s) < d);                      // 忙等
}
```
**DWT->CYCCNT 是 Cortex-M7 内核内部的 32 位硬件计数器，每个 CPU 时钟周期自动 +1。** 在 480MHz 下，每 2.08ns 加 1。Delay_us(1) 就等于等 480 个时钟周期。比 HAL_Delay（基于 SysTick，精度 1ms）精确 500 倍。

### MPU 缓冲区初始化 
```c
void mpu_buf_init(void){
    HAL_MPU_Disable();
    MPU_Region_InitTypeDef m={0};
    m.Enable      = MPU_REGION_ENABLE;
    m.Number      = MPU_REGION_NUMBER1;
    m.BaseAddress = 0x30000000;          // buf所在的内存地址
    m.Size        = MPU_REGION_SIZE_16KB; // 保护16KB
    m.AccessPermission = MPU_REGION_FULL_ACCESS;  // 可读可写
    m.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    m.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;    // ★核心: 禁止Cache
    m.IsBufferable= MPU_ACCESS_NOT_BUFFERABLE;
    m.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&m);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
```
**为什么要禁止 Cache？** H750 有 16KB L1 数据 Cache。如果 Cache 开启，CPU 读 `buf[0]` 时会先去 Cache 找。DMA 把新数据写入物理内存，但 Cache 里还是旧的——CPU 拿到了错误的数据！这叫做 **DMA 缓存一致性问题**。设置 `NOT_CACHEABLE` 后 CPU 每次都绕过 Cache 直接读物理内存。

---

## 第五部分：同步检测 

```c
int check_sync(void){
    uint32_t t=5000000;
    uint8_t s=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0); // 读PC0当前电平
    uint8_t c=0;                                   // 跳变计数器
    while(--t){
        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0) != s){  // 电平变了
            c++;
            s = !s;    // 翻转参考电平
        }
        if(c >= 4) return 1;  // 检测到≥4次跳变 → 有同步信号
    }
    return 0;  // 超时，无同步信号
}
```
逻辑：在一个超时窗口内，反复读取 PC0 引脚，如果电平来回跳变 ≥4 次（即至少 2 个完整方波周期），就认为激光模块的同步信号正常接入。返回 0 表示没检测到同步。

---

## 第六部分：LMS 正弦拟合 

### 数学模型
给定数据 `d[0..n-1]` 和一个**已知频率** `f`，用最小二乘法拟合：
$$\hat{y}_i = A\cos(\omega i) + B\sin(\omega i) + C$$
其中 $\omega = 2\pi f / f_s$。这是一个**线性**最小二乘问题（参数 A, B, C 线性出现）。

然后：
$$\text{幅度} = \sqrt{A^2 + B^2}$$
$$\text{相位} = \text{atan2}(-B, A)$$
$$\text{直流} = C$$
$$\text{RMSE} = \sqrt{\frac{1}{n}\sum(y_i - \hat{y}_i)^2}$$

### 代码逐行

```c
SineFitResult Sine_Fit_LMS(float32_t*d, int n, float32_t f, float32_t fs){
    float32_t w = 2*M_PI*f/fs;  // 角频率 (每次调用的f不同，所以w每次都要算)
```

**第一遍循环——累加正规方程 ：**
```c
    float32_t sc2=0, ss2=0, scs=0, sc=0, ss=0, syc=0, sys=0, sy=0;
    for(int i=0; i<n; i++){
        float32_t cs = arm_cos_f32(w*i);  // cos(ωi)  ← 每个点都调一次trig!
        float32_t sn = arm_sin_f32(w*i);  // sin(ωi)
        float32_t y  = d[i];              // 采样值
        sc2 += cs*cs;   ss2 += sn*sn;     // Σcos², Σsin²
        scs += cs*sn;                     // Σcos·sin
        sc  += cs;      ss  += sn;        // Σcos, Σsin
        syc += y*cs;    sys += y*sn;      // Σy·cos, Σy·sin
        sy  += y;                         // Σy
    }
```
这里是 9 个累加器，对应正规方程矩阵的 9 个元素。`arm_cos_f32` 是 CMSIS-DSP 的快速三角函数（查表+线性插值，约 30-50 周期）。

**第二遍——Cramer法则解3×3方程组 ：**
```c
    // 行列式计算
    float32_t det = sc2*(ss2*n - ss*ss) - scs*(scs*n - ss*sc) + sc*(scs*ss - ss2*sc);
    if(fabsf(det) < 1e-6f) return (SineFitResult){0,0,0,-1};  // 矩阵奇异，拟合失败
    
    float32_t iv = 1/det;  // 倒数 (一次FP除法 ≈ 14周期)
    
    // Cramer法则: 分别用RHS替换第1/2/3列求A/B/C
    float32_t A = (syc*(ss2*n-ss*ss) - scs*(sys*n-ss*sy) + sc*(sys*ss-ss2*sy)) * iv;
    float32_t B = (sc2*(sys*n-ss*sy) - syc*(scs*n-ss*sc) + sc*(scs*sy-sys*sc)) * iv;
    float32_t C = (sc2*(ss2*sy-ss*sys) - scs*(scs*sy-ss*syc) + syc*(scs*ss-ss2*sc)) * iv;
    
    SineFitResult r = {.amp=sqrtf(A*A+B*B), .phase=atan2f(-B,A), .dc=C, .rmse=0};
```
注意 `atan2f(-B, A)` 里的负号：标准写法是 `atan2(B, A)` 求 cos 相位，但这里的模型用 `A*cos+B*sin`，相位定义有差异，加负号纠正。实际上不影响后续距离计算，因为相位解包时用的是 `atan2f(-B, A)` 和反正切的结果本身。

**第三遍——计算RMSE ：**
```c
    float32_t es=0;
    for(int i=0; i<n; i++){
        float32_t v = r.amp*arm_cos_f32(w*i + r.phase) + r.dc;  // 用拟合参数重建波形
        float32_t df = d[i] - v;    // 残差
        es += df*df;                // 累加平方误差
    }
    r.rmse = sqrtf(es/n);
    return r;
}
```
---

## 第七部分：DSP测距主函数 

### Step 1: 去直流 + Blackman加窗 + LMS数据备份
```c
    float32_t mean=0;
    for(int i=0; i<VALID_POINTS; i++){
        fft_in[i] = (float32_t)buf[i];  // uint16→float 转换
        mean += fft_in[i];              // 累加求平均
    }
    mean /= VALID_POINTS;               // 直流分量
    
    for(int i=0; i<VALID_POINTS; i++){
        fft_in[i] -= mean;              // 去直流 (把波形中心拉到0)
        lms_in[i] = fft_in[i];          // ★保存一份去直流但未加窗的数据给LMS用
        
        float a = 2.0f*M_PI*i/(VALID_POINTS-1);
        fft_in[i] *= (0.42f - 0.5f*arm_cos_f32(a) + 0.08f*arm_cos_f32(2*a));
        // Blackman窗: 0.42 - 0.5cos(a) + 0.08cos(2a)
    }
```
**为什么 LMS 用不加窗的数据？** 窗函数会改变正弦波的幅度（加窗后的波形两端被压扁了）。FFT 需要窗来压制频谱泄漏，但 LMS 拟合假设输入是纯正弦波——用加窗的数据会导致拟合的幅度失真、RMSE不准。所以 LMS 用 `lms_in`（仅去直流），FFT 用 `fft_in`（去直流+加窗）。

### Step 2: 补零 + 实数FFT 
```c
    for(int i=VALID_POINTS; i<FFT_LENGTH; i++) fft_in[i]=0;  // 1000→4096 补3096个0
    arm_rfft_fast_f32(&fft_inst, fft_in, fft_out, 0);         // 实数FFT (RFFT)
    arm_cmplx_mag_f32(fft_out, fft_mag, FFT_LENGTH/2);        // 求模，得2048个幅度值
```
`fft_out` 排列：`[Re0, Re1, Im1, Re2, Im2, ..., Re2047, Im2047, Re2048]` （CMSIS-DSP RFFT的压缩格式）

### Step 3: 噪声门限 + 峰值搜索 + 抛物线插值 
```c
    // 求平均底噪
    float32_t mag_mean=0;
    for(int i=1; i<FFT_LENGTH/2; i++) mag_mean += fft_mag[i];
    mag_mean /= (FFT_LENGTH/2-1);
    
    // 找最大峰 (要超过底噪2倍)
    float32_t mv=0; uint32_t mi=1;
    for(int i=1; i<FFT_LENGTH/2; i++){
        if(fft_mag[i] > mag_mean*2 && fft_mag[i] > mv){
            mv = fft_mag[i]; mi = i;
        }
    }
    
    // 抛物线插值: 利用峰值和左右邻居，算出亚像素精度的峰位置
    float32_t y0=fft_mag[mi-1], y1=fft_mag[mi], y2=fft_mag[mi+1];
    float32_t den = y0 - 2*y1 + y2;               // 二阶差分的分母
    float32_t delta = (fabsf(den)<1e-9f) ? 0 : 0.5f*(y0-y2)/den;
    float32_t fi = (float32_t)mi + delta;          // 插值后的精确bin位置 (带小数)
    float fft_freq = fi * (SAMPLE_RATE / (float32_t)FFT_LENGTH);  // 转为频率 Hz
```
**抛物线插值原理：**
```
    y1  ← 最大峰值
   /  \
  y0   y2
  |    |
 mi-1  mi  mi+1

抛物线顶点公式: delta = 0.5 * (y0-y2) / (y0 - 2*y1 + y2)
真实峰值位置 = mi + delta (可能是 53 + 0.34 = 53.34)
```
这能让频率估计精度从 ±122Hz（一个 bin）提升到更细，为后续 LMS 缩小搜索范围。

### Step 4: LMS精搜 
```c
    float f_start = fft_freq - 150;  if(f_start<1) f_start=1;
    float f_end   = fft_freq + 150;
    float best_f=fft_freq, best_rmse=1e9f;
    SineFitResult r;
    
    for(float ft=f_start; ft<=f_end; ft+=1.0f){        // 1Hz步长穷举
        SineFitResult t = Sine_Fit_LMS(lms_in, VALID_POINTS, ft, SAMPLE_RATE);
        if(t.rmse>=0 && t.rmse<best_rmse){
            best_rmse = t.rmse;
            best_f = ft;
            r = t;             // 保存最优拟合结果 (含相位)
        }
    }
```
±150Hz 以 1Hz 步长搜索 = **301 次 LMS 调用**。每次 LMS 内部 3000 次 trig。这就是我们反复讨论的瓶颈。

**为什么搜 ±150Hz？** FFT 的 bin 宽度是 244Hz，加上抛物线插值的不确定性，±150Hz 足够覆盖真实频率可能的范围。

### Step 5: 距离计算  
```c
    // FMCW基础公式: R = c * fb * Tc / (2 * B)
    float R_abs = (LIGHT_SPEED * best_f * RAMP_TIME_TOTAL) / (2.0f * FREQ_DEV);
    
    
    printf("\r\nfbeat=%luHz Abs=%d.%06dm Rel=%c%d.%06dm\r\n", ...);
}
```
---

## 第八部分：main函数——初始化 

```c
int main(void){
    // === CubeMX自动生成的硬件初始化 ===
    MPU_Config();           // 默认MPU配置 (CubeMX生成)
    HAL_Init();             // HAL库初始化 (SysTick等)
    SystemClock_Config();   // 时钟配置: HSE→PLL→480MHz
    MX_GPIO_Init();         // GPIO初始化
    MX_DMA_Init();          // DMA初始化
    MX_ADC1_Init();         // ADC1初始化
    MX_USB_DEVICE_Init();   // USB设备初始化
    MX_TIM2_Init();         // TIM2初始化 (200kHz触发)
    
    // === 用户手动初始化 ===
    HAL_PWREx_EnableUSBVoltageDetector();  // USB VBUS检测使能
    mpu_buf_init();         // ★覆盖CubeMX的MPU: 给buf区域关Cache
    DWT_Init();             // 启动微秒延时
    setvbuf(stdout, NULL, _IONBF, 0);     // printf无缓冲(实时输出)
    HAL_Delay(200);         // 等电源稳定
    
    // USB等待(3秒超时)
    uint32_t usb_wait_start = HAL_GetTick();
    while(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED){
        if(HAL_GetTick() - usb_wait_start > 3000) break;  // 超时不卡死
    }
    HAL_Delay(500);
    
    arm_rfft_fast_init_f32(&fft_inst, FFT_LENGTH);  // 初始化FFT旋转因子表
    
    sync_ok = check_sync();  // 检测有没有同步信号
    printf("\r\n=== FMCW Ranging ===\r\nSync:%s\r\n", sync_ok?"OK":"NO");
```

`mpu_buf_init()` 和 `MPU_Config()` 的关系：
- `MPU_Config()` 是 CubeMX 生成的默认配置，把整个 4GB 空间设为 `NO_ACCESS`
- `mpu_buf_init()` 追加一条规则，把 `0x30000000` 的 16KB 设为 `FULL_ACCESS` + `NOT_CACHEABLE`
- MPU 规则有优先级，`mpu_buf_init` 的规则更具体（地址范围小），优先级更高，覆盖了默认规则

---

## 第九部分：main函数——主循环 

### WKUP按键处理 
```c
    uint32_t now = HAL_GetTick();
    uint8_t wk = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    if(wk && !wk_st){ wk_tm = now; wk_st = 1; }         // 按下瞬间记录时间
    if(!wk && wk_st && (now - wk_tm > 30)){              // 松开后超过30ms (去抖)
        wk_st = 0; ref_dist = 0; has_ref = 0;            // 归零
        printf("\r\n[Zero Set]\r\n");
    }
```

### 有同步信号时的主流程 
```c
    if(sync_ok){
        // ① 刹车上一次传输
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_TIM_Base_Stop(&htim2);
        
        // ② 等待同步信号边沿
        uint32_t to=20000000; uint8_t ok=1;
        while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_SET && --to);  // 等高→低
        if(!to) ok=0;
        if(ok){
            to=20000000;
            while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_RESET && --to); // 等低→高
            if(!to) ok=0;
        }
        
        if(!ok){
            HAL_Delay(10);  // 同步超时，短暂等待后重试
        } else {
            // ③ 同步到了！延迟避开激光起始非线性区
            Delay_us(50);
            
            // ④ 清标志 + 启动ADC+TIM2+DMA
            full=0;
            __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
            HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buf, VALID_POINTS);  // 锁1000点
            HAL_TIM_Base_Start(&htim2);       // TIM2开始发TRGO→ADC开始采
            
            // ⑤ CPU死等DMA采满1000点
            volatile uint32_t dto=50000000;
            while(!full && --dto);
            
            // ⑥ 停机
            HAL_TIM_Base_Stop(&htim2);
            Delay_us(2);                      // 等最后一个ADC转换完成
            HAL_ADC_Stop_DMA(&hadc1);
            
            // ⑦ 数据到手
            if(dto){
                // 发波形到USB
                int pos=0;
                for(int i=0; i<VALID_POINTS; i++) pos+=sprintf(wf_out+pos,"%d\r\n",buf[i]);
                usb_send(wf_out, pos);
                printf("0\r\n0\r\n0\r\n");    // 波形结束标记
                
                dsp_ranging();                 // ★ DSP测距
            }
        }
        
        // ⑧ 重新启动ADC+DMA等待下一帧 (≠等待同步，而是"预启动")
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buf, 2048);
        HAL_TIM_Base_Start(&htim2);
    }
```

注意第 ⑧ 步：**主循环每次迭代的末尾又重启了 ADC+DMA**。这意味着：
- 第一次进主循环：等待同步 → 采 1000 点 → 发波形 → DSP → **重启 ADC**
- 第二次进主循环：**先 Stop ADC**（第 ① 步把上次末尾启动的停掉） → 重新等同步 → ...

这是一个"预启动"策略，让 ADC 在等待同步期间就开始转换，减小延迟。

### 无同步信号时的自由波形模式 
```c
    } else {
        // 无同步: 直接采集，不等边沿
        full=0;
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buf, VALID_POINTS);
        HAL_TIM_Base_Start(&htim2);
        volatile uint32_t dto=50000000;
        while(!full && --dto);
        HAL_TIM_Base_Stop(&htim2);
        Delay_us(2);
        HAL_ADC_Stop_DMA(&hadc1);
        if(dto){
            int pos=0;
            for(int i=0; i<VALID_POINTS; i++) pos+=sprintf(wf_out+pos,"%d\r\n",buf[i]);
            usb_send(wf_out,pos);
            printf("0\r\n0\r\n0\r\n");
        }
        HAL_Delay(50);  // 不加DSP，只发波形，间隔50ms
    }
```
无同步时不跑 `dsp_ranging()`，只发波形数据供调试。

---

## 第十部分：时钟配置 

```c
void SystemClock_Config(void){
    // HSE: 外部8MHz晶振
    // PLL: 8MHz / M(1) * N(120) / P(2) = 480MHz → SYSCLK
    // PLLQ = 15 → 8*120/15 = 64MHz → 48MHz用的分频源 (USB/HSI48)
    
    // 关键参数:
    // SYSCLK = 480MHz (HCLK/2 = 240MHz)
    // APB1/2 = 120MHz
    // FLASH_LATENCY_4: 480MHz需要4个等待周期
}
```

## 第十一部分：MPU默认配置 

```c
void MPU_Config(void){
    // 把整个4GB地址空间设为默认禁止访问
    // 然后后续的具体区域(如mpu_buf_init)用更具体的规则覆盖
    // SubRegionDisable=0x87 是一个技巧: 只禁用部分子区域
}
```
---

## 程序执行时序总结

```
上电
 ↓
硬件初始化 (MPU/HAL/Clock/外设)
 ↓
USB等待 (3秒超时)
 ↓
FFT初始化 + 同步检测
 ↓
┌─────────────────────────────────────────────────┐
│                主循环 (while 1)                  │
│                                                  │
│  ┌─ 读WKUP按键 → 归零?                           │
│  │                                               │
│  ├─ sync_ok==1?                                  │
│  │   ├─ Stop ADC + TIM2                          │
│  │   ├─ 等同步边沿(上升沿)                        │
│  │   ├─ Delay_us(50) 避开非线性区                 │
│  │   ├─ Start ADC(DMA 1000pt) + Start TIM2       │
│  │   ├─ while(!full) CPU死等                     │
│  │   ├─ Stop TIM2 + ADC                          │
│  │   ├─ USB发波形数据                             │
│  │   ├─ dsp_ranging()                            │
│  │   └─ 预启动ADC等待下一帧                       │
│  │                                               │
│  └─ sync_ok==0?                                  │
│      └─ 自由采集 + USB发波形 (无DSP)              │
└─────────────────────────────────────────────────┘
```
