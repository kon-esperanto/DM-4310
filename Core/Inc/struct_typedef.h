/**
  * @file    struct_typedef.h
  * @brief   电机例程里通用的基础类型定义(RoboMaster/达妙代码族的公共头文件)
  *          这里直接复用标准 stdint.h, 避免和 CMSIS 里的整数类型定义冲突
  */
#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include <stdint.h>
#include <stdbool.h>

typedef unsigned char bool_t;
typedef float         fp32;
typedef double        fp64;

#endif

/*------------------------------ End of File ------------------------------*/
