#ifndef TASK_OLED_H
#define TASK_OLED_H

#include <stdint.h>

/* SSD1306 128x64 I2C OLED 显示驱动（页寻址模式） */

void oled_init(void);                              /* 初始化序列 */
void oled_clear(void);                             /* 全屏清空 */
void oled_show_char(uint8_t page, uint8_t col, char ch);   /* page 0~7，col 0~127 */
void oled_show_string(uint8_t page, uint8_t col, const char *str);
void oled_show_num(uint8_t page, uint8_t col, int32_t num, uint8_t len); /* 右对齐定宽 */

#endif
