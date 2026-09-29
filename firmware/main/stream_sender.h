/**
 * @file stream_sender.h
 * @brief UDP stream sender with WireGuard/LAN dual-target fallback.
 */

#ifndef STREAM_SENDER_H
#define STREAM_SENDER_H

#include <stdint.h>
#include <stddef.h>

int stream_sender_init(void);
int stream_sender_init_with(const char *ip, uint16_t port);
void stream_sender_set_fallback(const char *ip, uint16_t port);
int stream_sender_send(const uint8_t *data, size_t len);
void stream_sender_deinit(void);

#endif /* STREAM_SENDER_H */
