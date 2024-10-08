# 运放的基本使用 - ADC采样PGA的输出电压
## 关键字: PGA放大电压
 
**【功能描述】**
+ 内部电阻模式下，PGA对输入电压放大，并使用ADC采样PGA的输出电压。

**【示例配置】**
+ 本示例中使用了PGA的内部电阻模式，电压输出到PGA端，通过PGA进行2倍的放大，并通过ADC将最后采样结果输出。

+ 本示例中采用内部电阻模式， 增益放大值为X2倍，增益值可在PGA模块的配置界面中进行更改，其对应的设置选项为“g_pga.gain”。

**【示例效果】**
+ PGA的运放结果会通过ADC的采样进行输出，并通过串口0将结果输出。
 
**【注意事项】**
+ 增益可编程为x2, x4, x8, x16。