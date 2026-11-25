/**
  * @file    attribute_typedef.h
  * @brief   结构体紧凑属性。遥控器/上位机通信帧必须按字节逐个排列，不能有编译器填充，
  *          所以原RM代码在结构体声明后面写 __packed__。
  */
#ifndef ATTRIBUTE_TYPEDEF_H
#define ATTRIBUTE_TYPEDEF_H

/* 注意: 标准库 <sys/cdefs.h> 里已经有 __packed, 再定义一遍会告警, 所以要判重 */
#ifndef __packed
#define __packed    __attribute__((packed))
#endif

#ifndef __packed__
#define __packed__  __attribute__((packed))
#endif

#endif

/*------------------------------ End of File ------------------------------*/
