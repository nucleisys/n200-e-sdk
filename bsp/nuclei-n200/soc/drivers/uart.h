// See LICENSE for license details.

#ifndef _NUCLEI_UART_H
#define _NUCLEI_UART_H

/* Register offsets */
#define UART_REG_TXFIFO         0x00
#define UART_REG_RXFIFO         0x04
#define UART_REG_TXCTRL         0x08
#define UART_REG_RXCTRL         0x0c
#define UART_REG_IE             0x10
#define UART_REG_IP             0x14
#define UART_REG_DIV            0x18

/* Tx/RxFIFO register */
#define UART_FIFO_FULL          0x80000000
#define UART_FIFO_EMPTY         0x80000000

/* TXCTRL register */
#define UART_TXEN               0x1
#define UART_TXWM(x)            (((x) & 0xffff) << 16)

/* RXCTRL register */
#define UART_RXEN               0x1
#define UART_RXWM(x)            (((x) & 0xffff) << 16)

/* IE register */
#define UART_IE_TXWM            0x1
#define UART_IE_RXWM            0x2

/* IP register */
#define UART_IP_TXWM            0x1
#define UART_IP_RXWM            0x2

#endif /* _NUCLEI_UART_H */
