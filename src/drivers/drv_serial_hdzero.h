#pragma once

#include <stdint.h>

#include "drv_osd.h"

<<<<<<< HEAD
=======
#define HDZERO_ROWS 18
#define HDZERO_COLS 50

>>>>>>> master
void hdzero_init();
bool hdzero_is_ready();
void hdzero_intro();

uint8_t hdzero_clear_async();
osd_system_t hdzero_check_system();

<<<<<<< HEAD
void hdzero_txn_start(osd_transaction_t *txn, uint8_t attr, uint8_t x, uint8_t y);
void hdzero_txn_write_char(osd_transaction_t *txn, const char val);
void hdzero_txn_write_data(osd_transaction_t *txn, const uint8_t *buffer, uint8_t size);
void hdzero_txn_submit(osd_transaction_t *txn);
=======
bool hdzero_can_fit(uint8_t size);
bool hdzero_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size);
bool hdzero_flush();
>>>>>>> master
