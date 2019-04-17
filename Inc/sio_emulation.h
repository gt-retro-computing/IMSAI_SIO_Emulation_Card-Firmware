#pragma once

#include "stm32f1xx_hal.h"
#include <stdlib.h>
#include <stdint.h>

#define INPUT_FLOATING_PORT_H 0x44444444
#define OUTPUT_PP_PORT_H 0x33333333

#define CHANNEL_1_DATA_PORT 0x2
#define CHANNEL_1_CTRL_PORT 0x3

#define activate_addr_match do{GPIOA->BSRR = GPIO_PIN_8;}while(0)
#define deactivate_addr_match do{GPIOA->BSRR = (GPIO_PIN_8) << 16u;}while(0)

// #define activate_addr_match HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
// #define deactivate_addr_match HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define serial_1_hasdata (serial_1_read_pointer != serial_1_old_write_pointer)

#define pDBIN (1U << 1U)
#define pWR (1U << 15U)
#define sINP (1U << 7U)
#define sOUT (1U << 6U)

#define get_serial_state(x) (0x1U | (serial_1_rx_overrun << 4U) | serial_1_hasdata << 1U | (((x)->SR >> 5U) & 4U))


extern uint8_t serial_1_rx_overrun;
extern size_t serial_1_old_write_pointer;
extern const uint8_t FIX_BITS[256];


int check_reset(void);
uint8_t get_address(void);
void ioline_handle_interrupt(void);

#define SERIAL_1_RX_BUFFER_SIZE 64
extern char serial_1_rx_buffer[SERIAL_1_RX_BUFFER_SIZE];
extern size_t serial_1_read_pointer;

void serial_data_interrupt(DMA_HandleTypeDef *huart);
void fetch_out_instruction_data();
void dump_in_instruction_data();