#ifndef DATALOG_H
#define DATALOG_H

#include <stdint.h>

/* One saved reading. Same 11-byte layout Dragino's -LB nodes send on
 * fPort 3 when they replay missed samples or answer a 0x31 poll.
 *   0..1  primary value (distance mm in distance mode, temperature x10 otherwise)
 *   2..3  second value (signal strength, or humidity x10; 0xFFFF = none)
 *   4..5  battery millivolts
 *   6     bit7 = network did not acknowledge this reading
 *         bit6 = reply to a 0x31 history poll (set on the air, not in flash)
 *         bits 3..0 = work mode
 *   7..10 Unix time, seconds, big-endian
 */
#define DATALOG_RECORD_LEN 11
#define DATALOG_FPORT 3
#define DATALOG_FLAG_NOACK 0x80
#define DATALOG_FLAG_POLL  0x40

/* Written into the settings page so an older config (unused bytes were
 * never set) cannot turn this on by accident. */
#define DATALOG_CFG_MAGIC 0xD1

void datalog_init(void);
void datalog_clear(void);

/* Append a reading to flash. Every reading is logged, like Dragino's nodes,
 * so a 0x31 poll can return history whether or not it was acknowledged. */
void datalog_store(const uint8_t record[DATALOG_RECORD_LEN]);

/* A confirmed live frame is leaving the radio; keep its reading until the
 * ACK result arrives, then datalog_process stores it with bit7 as needed. */
void datalog_remember(const uint8_t record[DATALOG_RECORD_LEN]);
/* Interrupt-safe: records the result only. */
void datalog_on_tx_done(uint8_t ack_received);
/* Main-loop context: applies a recorded result, which may write flash.
 * datalog_remember and datalog_replay_ready call it themselves. */
void datalog_process(void);
/* The packet never left the radio. Keep a live reading, or try a replay again later. */
void datalog_defer(void);

/* PNACKMD replay: unacknowledged readings go out oldest first on fPort 3
 * after the next successful confirmed uplink. */
uint8_t datalog_replay_ready(void);
uint8_t datalog_fill_replay(uint8_t *buf, uint8_t max_len);

/* Downlink 0x31 <start:4> <end:4> <interval_s:1>: send every stored reading
 * with start <= time <= end, oldest first, bit6 set, one frame per interval. */
void datalog_poll_request(uint32_t start, uint32_t end, uint8_t interval_s);
uint8_t datalog_poll_ready(void);
uint8_t datalog_fill_poll(uint8_t *buf, uint8_t max_len);
void datalog_poll_defer(void);

/* For AT+PDTA / AT+PLDTA. index 0 = oldest stored reading. */
uint16_t datalog_count(void);
uint8_t datalog_read(uint16_t index, uint8_t record[DATALOG_RECORD_LEN]);

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
