#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#include <stdint.h>
#include <stdbool.h>

extern uint8_t enable_group1_irq;

void Interrupt_Init(void);

/* ===== K230 UART Ring Buffer ===== */
#define K230_RX_BUF_SIZE    128
#define K230_LINE_BUF_SIZE  128  /* Must match RX_BUF_SIZE to prevent livelock on long lines */

extern volatile uint8_t  k230_rx_buf[K230_RX_BUF_SIZE];
extern volatile uint16_t k230_rx_head;
extern volatile uint16_t k230_rx_tail;

/* Get a complete line from the ring buffer (newline-terminated).
 * Returns true if a line was copied to 'line', false if no line available */
bool k230_read_line(uint8_t *line, uint16_t *len);

#endif  /* #ifndef _INTERRUPT_H_ */