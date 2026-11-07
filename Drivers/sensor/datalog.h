#ifndef DATALOG_H
#define DATALOG_H

#include <stdint.h>

/* One saved reading. Same 11-byte size the other Dragino nodes send
 * when they replay missed samples (fPort 3).
 *   0..1  primary value (distance mm in distance mode, temperature otherwise)
 *   2..3  second value (signal strength, or humidity)
 *   4..5  battery millivolts
 *   6     bit7 = network did not acknowledge this reading
 *         bit6 = reply to a history poll
 *         bits 3..0 = work mode
 *   7..10 Unix time, seconds, big-endian
 */
#define DATALOG_RECORD_LEN 11
#define DATALOG_FPORT 3

/* Written into the settings page so an older config (unused bytes were
 * never set) cannot turn this on by accident. */
#define DATALOG_CFG_MAGIC 0xD1

void datalog_init(void);
void datalog_clear(void);
void datalog_store(const uint8_t record[DATALOG_RECORD_LEN]);
void datalog_remember(const uint8_t record[DATALOG_RECORD_LEN]);
/* Interrupt-safe: records the result only. */
void datalog_on_tx_done(uint8_t ack_received);
/* Main-loop context: applies a recorded result, which may write flash.
 * datalog_remember and datalog_replay_ready call it themselves. */
void datalog_process(void);
/* The packet never left the radio. Keep a live reading, or try a replay again later. */
void datalog_defer(void);
uint8_t datalog_replay_ready(void);
uint8_t datalog_fill_replay(uint8_t *buf, uint8_t max_len);

/* 1 = keep readings the network did not acknowledge, and send them later.
 * Same switch as AT+PNACKMD / downlink 34 01 on the other nodes. */
extern uint8_t pnackmd_switch;
/* 1 = set the clock from the network. Same switch as AT+SYNCMOD / 28 01. */
extern uint8_t time_synchronization_method;
/* How many days between clock updates. Same as AT+SYNCTDC. */
extern uint8_t time_synchronization_interval;

/* Defined in main.c. Starts or stops the clock-update timer. */
void time_sync_apply(void);

#endif
