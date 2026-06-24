/**
 * @file ssd1306.h
 * @brief 极简 SSD1306 OLED 驱动 (128x64, I2C, 8x16 字符)
 *        默认从机地址 0x78 (= 0x3C<<1 写位)
 */
#ifndef __SSD1306_H
#define __SSD1306_H
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"

void OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_Clear(void);
void OLED_FillTest(uint8_t pattern);
/* page: 0..7 (一页 8 行); col: 0..127 */
void OLED_ShowString(uint8_t page, uint8_t col, const char *str);

/* 探测 OLED I2C 地址 (0x78/0x7A), 返回找到的写地址, 0=未找到 */
uint8_t OLED_Probe(void);
/* 返回当前使用的写地址 */
uint8_t OLED_GetAddr(void);
/* 绕过 IsDeviceReady, 直接向 write_addr 写一个命令; 返回 HAL 状态(0=OK) */
int OLED_TryWrite(uint8_t write_addr);
/* 返回初始化失败的命令数 (0=全部成功, -1=未初始化) */
int OLED_GetInitFail(void);

#ifdef __cplusplus
}
#endif
#endif
