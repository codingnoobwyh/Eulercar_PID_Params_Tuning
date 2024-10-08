# 控制局域网-发送扩展数据帧和接收符合过滤规则的扩展帧数据
## 关键字: CAN, 总线
 
**【功能描述】**
+ CAN向总线发送扩展帧数据，并从总线上接收符合过滤规则的扩展帧数据。

+ CAN模块数据的发送和接收采用的都是中断形式。

**【示例配置】**
+ 初始化CAN：通过g_can.typeMode配置CAN的工作模式，配置CAN的传输波特率，接收FIFO的深度以及否开启自动重传。这些配置可以通过CAN的配置界面进行更改。默认配置参数可以通过CAN的配置界面或“system_init.c”文件中的进行查看。

+ 发送数据帧g_sendFrame：分配发送数据帧的类型为扩展帧，指定发送帧的ID，填入需要发送的数据和长度，调用“HAL_CAN_Write”函数进行数据的发送。发送成功之后会调用回调函数“Can_WriteFinish”，此回调函数可以通过“HAL_CAN_RegisterCallBack”进行注册。

+ 接收数据帧g_receiveFrame: 存储接收到的数据，会存入接收到的帧类型，帧ID，帧数据域。

+ 过滤条件rxFilter：配置过滤的帧类型和过滤ID和过滤掩码。并通过“HAL_CAN_ReadIT”使过滤规则生效，接收成功之后，会调用接收成功回调函数“Can_ReadFinish”，此回调函数也可以通过“HAL_CAN_RegisterCallBack”进行注册。

**【示例效果】**
+ 编译烧录此示例，此示例会向总线发送ID为0x1314的扩展数据帧，数据内容为字符‘0’。接收总线上ID为0x1X14的扩展帧（X代表任意0-F的数字)。

+ 串口0打印提示信息：CAN发送给总线的数据和接收到总线的数据。
 
**【注意事项】**
+ 发送给总线的数据帧为扩展帧类型，且满足过滤条件，才会被接收，其他数据默认会被总线丢弃。

+ 此示例默认开启自动重发，数据发送失败，会重新发送。若不想重发，需在配置中关闭“Auto Retransmission”项，或在“system_init.c”中关闭“g_can.autoRetrans”配置项。