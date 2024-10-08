# 配置GPIO管脚的中断功能，实现GPIO不同触发方式下中断功能测试
## 关键字: GPIO, 中断，触发方式

**【功能描述】**
+ 示例代码基于HAL接口完成时钟、GPIO控制器初始化和功能配置。在示例代码的中断功能测试中使用GPIO的两个管脚，一个用于输出信号，另一个用于作为中断管脚接收信号，在不同中断方式下触发GPIO的中断功能。

**【示例配置】**
+ GPIO管脚选择：示例代码中选择了两个GPIO管脚用于中断功能测试。也可以选择其他GPIO管脚用于中断功能测试，在"GPIO_Init()"接口中的"g_gpiox.baseAddress"可以配置其它GPIOX，g_gpiox.pins可以配置“GPIO_PIN_0-GPIO_PIN_7”中的任意一个。
  
+ GPIO管脚初始化：调用接口"HAL_GPIO_Init()”完成对示例代码中GPIO两个管脚的方向、电平、中断模式配置。在进行GPIO管脚配置时，需将其中一个管脚配置成输出模式、低电平、不使用中断模式，将另一个管脚配置成输入模式、低电平、高电平中断模式。之后，再调用中断相关接口注册、开启中断功能。

+ GPIO中断功能测试过程： 
    1. 对于输出管脚调用接口"HAL_GPIO_SetValue()"变换管脚输出电平，从而直接或间接构造出高电平、低电平、上升沿、下降沿输出信号；
    2. 对于输入管脚调用接口"HAL_GPIO_SetIrqType()"设置管脚不同的中断触发类型；
    3. 通过输出管脚和输入管脚的相互配合实现不同触发类型的中断功能测试。

**【示例效果】**
+ 当用户烧录编译后的示例代码后，初始化和配置完成后，示例代码中会依次进行不同触发模式的中断功能测试。功能正常时Debug串口打印成功log信息；功能异常时Debug串口打印失败log信息。

**【注意事项】**
+ 示例代码使用UART0进行结果打印输出，需要对UART0配置。
+ 示例代码中使用GPIO两个管脚需连接在一起。