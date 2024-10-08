# SPI使用阻塞方式进行主从通信，此示例代码作为主机使用
## 关键字: SPI, 阻塞, 主机

**【功能描述】**
+ 示例代码基于HAL接口完成时钟、SPI控制器初始化和功能配置。主机读从机数据或者向从机发送数据。

**【示例配置】**
+ 在"SystemInit()”接口中配置SPI时钟极性、时钟相位、帧格式等参数。

+ 定义数组，数组中存放主机需要发送的数据。

+ 首先主机调用“HAL_SPI_WriteBlocking()”向从机发送10个数据，之后调用“HAL_SPI_ReadBlocking()”读10个数据，最后调用“HAL_SPI_WriteReadBlocking()”边写边读10个数据，不断循环。

**【示例效果】**
+ 当用户烧录编译后的示例代码后，初始化和配置完成后，示例代码通过串口不断打印主机接收到的数据、发送的数据。

**【注意事项】**
+ 使用此示例代码必须对接spi_slave一起使用，使用两个设备一个做master一个做slave。