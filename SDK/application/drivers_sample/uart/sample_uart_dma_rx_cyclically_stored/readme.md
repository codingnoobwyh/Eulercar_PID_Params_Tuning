# UART-DMA模式循环接收数据
## 关键字: UART, DMA循环接收

**【功能描述】**
+ 在UART DMA模式下，当UART接收数据完成后，DMA接收到UART的搬运请求，将数据循环搬运到预设置的内存中， 当预设置的内存写满之后，
将会从内存的起始地址开始写, 循环此过程。

**【示例配置】**
+ 预设置的缓存：需要使用者自行开辟内存空间，将内存首地址和需要接收的字符长度作为入参传入HAL_UART_ReadDMAAndCyclicallyStored()中。

+ 串口0的接收模式配置为DMA模式。并初始化DMA接收通道，需配置源地址和目标地址增长方式以及中断等。可通过DMA配置界面进行更改。

+ DMA接收数据完成，会将接收的数据存储在预设置的缓存中，并通过串口打印。

+ 结果读取：通过读取预设置的内存，并通过函数“HAL_UART_ReadDMAGetPos”来判断相对于指定内存的偏移位置。

**【示例效果】**
+ 不断向串口0发送数据，串口0会打印出接收的数据，并将数据储存到预设的内存中，当预设置的内存写满之后，会从内存的起始地址开始写。

**【注意事项】**
+ 串口通信的波特率，校验位等配置需保持一致，可以通过UART配置界面进行更改。