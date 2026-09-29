#pragma once
void optical_probe_start(void);
extern volatile int g_optical_raw;
extern volatile int g_optical_present;
extern volatile int g_ir_tx_raw;
extern volatile int g_ir_rx_raw;
