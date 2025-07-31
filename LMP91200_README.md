# LMP91200 pH传感器驱动使用指南

## 概述

LMP91200是Texas Instruments的一款高精度pH传感器接口芯片，专为pH测量应用设计。本驱动提供了完整的pH传感器数据采集、处理和传输功能。

## 特性

- **高精度pH测量**: 支持0.01 pH精度
- **温度补偿**: 内置温度传感器，支持温度补偿
- **可配置增益**: 支持1x到32x可编程增益
- **多种采样率**: 支持1Hz到32Hz采样率
- **I2C接口**: 标准I2C通信接口
- **数据压缩**: 支持TSZ压缩算法
- **BLE传输**: 通过BLE实时传输pH数据

## 硬件连接

### I2C连接
- SDA: P0.26
- SCL: P0.27
- 地址: 0x90 (7位地址)

### 中断引脚（可选）
- INT: P0.28

### pH电极连接
- pH电极正极连接到LMP91200的pH输入
- pH电极负极连接到LMP91200的参考输入
- 温度传感器连接到LMP91200的TEMP输入

## 配置

### 基本配置

```rust
lmp91200_config! {
    sample_rate: Lmp91200Odr::_4Hz,        // 采样率
    gain: Lmp91200Gain::_8,                 // 增益设置
    reference_voltage: Lmp91200Vref::_3_3V, // 参考电压
    temperature_compensation: true,          // 温度补偿
    calibration_enabled: true,               // 校准功能
    fifo_threshold: 16,                     // FIFO阈值
    rows_per_payload: 50,                   // 每包数据行数
}
```

### 配置参数说明

| 参数 | 选项 | 说明 |
|------|------|------|
| sample_rate | _1Hz, _2Hz, _4Hz, _8Hz, _16Hz, _32Hz | 采样率设置 |
| gain | _1, _2, _4, _8, _16, _32 | 放大器增益 |
| reference_voltage | _2_5V, _3_3V, _5V | 参考电压 |
| temperature_compensation | true/false | 温度补偿开关 |
| calibration_enabled | true/false | 校准功能开关 |

## 使用方法

### 1. 创建任务

```rust
// 创建LMP91200任务
create_lmp91200_tasks!(lmp0, TWIM0);
```

### 2. 初始化硬件

```rust
// 初始化I2C总线
let i2c = Twim::new(p.TWIM0, Irqs, p.P0_26, p.P0_27, Default::default());
let bus = I2cBus::new(i2c);

// 初始化中断引脚
let int_pin = InterruptPin::new(p.P0_28);
```

### 3. 启动任务

```rust
// 启动LMP91200 pH传感器任务
lmp0::spawn(0x01, &spawners, &LMP91200_CONF, &LMP91200_CONF, bus, int_pin);
```

## 数据处理

### pH值计算

驱动使用Nernst方程计算pH值：

```
pH = 7.0 - (voltage - 0.0) / 0.0592
```

其中：
- voltage: 从pH电极测量的电压
- 0.0592: Nernst常数（25°C时）

### 温度补偿

温度补偿使用以下公式：

```
Temperature = (temp_adc * 0.0625) - 256
```

### 数据格式

每个数据包包含：
- pH值 (i16, 精度0.001)
- 温度值 (i16, 精度0.01°C)
- 电压值 (i16, 精度0.001V)

## 校准

### 两点校准

1. **pH 7.0校准**: 使用pH 7.0标准缓冲液
2. **pH 4.0校准**: 使用pH 4.0标准缓冲液

### 校准步骤

1. 将pH电极浸入pH 7.0缓冲液
2. 等待读数稳定（约30秒）
3. 记录pH 7.0读数
4. 将pH电极浸入pH 4.0缓冲液
5. 等待读数稳定
6. 记录pH 4.0读数
7. 计算校准参数

## 故障排除

### 常见问题

1. **通信失败**
   - 检查I2C连接
   - 确认设备地址正确
   - 检查电源电压

2. **pH读数不准确**
   - 检查电极是否清洁
   - 确认缓冲液新鲜
   - 检查温度补偿设置

3. **数据丢失**
   - 检查FIFO阈值设置
   - 确认BLE连接稳定
   - 检查采样率设置

### 调试信息

驱动会输出以下调试信息：
- 设备ID确认
- 采样状态
- 数据处理统计
- 错误信息

## 示例代码

完整示例请参考 `src/bin/ph_sensor.rs`。

## 注意事项

1. **电极维护**: 定期清洁和校准pH电极
2. **温度影响**: pH测量对温度敏感，确保温度补偿正确
3. **缓冲液**: 使用新鲜的校准缓冲液
4. **电源稳定性**: 确保稳定的电源供应
5. **电磁干扰**: 避免强电磁干扰环境

## 技术规格

- **pH范围**: 0-14 pH
- **精度**: ±0.01 pH
- **温度范围**: -40°C to +125°C
- **工作电压**: 2.5V to 5.5V
- **接口**: I2C (400kHz max)
- **封装**: 16-pin TSSOP 