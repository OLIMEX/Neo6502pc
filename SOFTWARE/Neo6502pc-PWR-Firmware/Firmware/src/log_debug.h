#ifndef __LOG_DEBUG_H
#define __LOG_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#ifdef LOD_DEBUG_ENABLE
#define LOG_DEBUG(f_, ...) do { printf((f_), ##__VA_ARGS__); } while(0)
#else
#define LOG_DEBUG(f_, ...)
#endif

#define EOL "\r\n"

void LOG_DEBUG_Configure(uint32_t baudrate, uint16_t stop_bits, uint16_t parity);

#ifdef __cplusplus
}
#endif

#endif /* __LOG_DEBUG_H */
