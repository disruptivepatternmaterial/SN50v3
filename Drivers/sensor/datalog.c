#include <string.h>
#include "datalog.h"
#include "flash_eraseprogram.h"
#include "tremo_flash.h"
#include "tremo_delay.h"
#include "tremo_iwdg.h"
#include "log.h"
#include "timer.h"
#include "LoRaMac.h"

/* Flash map reserved by the Dragino ASR6601 template (LA66 and this
 * board share it). Nothing else in this firmware reads or writes the
 * span, so it holds the reading log. Keys stay at FLASH_USER_START_ADDR_KEY,
 * which is the end address.
 *
 * The log is a ring of 16-byte slots. Byte 11 of a slot is programmed to
 * 0x00 when the slot holds a record; erased flash reads 0xFF. Head and tail
 * are recovered by scanning the markers at boot, so no metadata page is
 * rewritten per reading. A page is erased only when the write position
 * enters it, which is once per 256 readings. */
#define DATALOG_PAGE      0x1000u
#define DATALOG_DATA      FLASH_SENSOR_DATA_START_ADDR
#define DATALOG_END       FLASH_SENSOR_DATA_END_ADDR
#define DATALOG_SLOT      16u
#define DATALOG_MARK      11u
#define DATALOG_PER_PAGE  (DATALOG_PAGE / DATALOG_SLOT)
#define DATALOG_PAGES     ((DATALOG_END - DATALOG_DATA) / DATALOG_PAGE)
#define DATALOG_CAP       (DATALOG_PAGES * DATALOG_PER_PAGE)
#define DATALOG_REPLAY_GAP_MS 10000u
#define DATALOG_POLL_MIN_GAP_MS 5000u

uint8_t pnackmd_switch = 0;
uint8_t time_synchronization_method = 0;
uint8_t time_synchronization_interval = 10;

static uint16_t log_head = 0;
static uint16_t log_tail = 0;
static uint16_t log_count = 0;

/* Replay walks forward from this offset (from log_head) looking for records
 * stored with bit7. It starts at the end of the log on boot: flash cannot be
 * marked "replayed" without rewriting a programmed word, so readings missed
 * before a reboot are left to the 0x31 poll instead of being re-sent. */
static uint16_t replay_cursor = 0;
static uint16_t replay_cursor_next = 0;

static uint8_t inflight[DATALOG_RECORD_LEN];
static uint8_t inflight_kind = 0; /* 1 = live sample waiting for an answer, 2 = replay */
static uint8_t inflight_count = 0;
static uint8_t replay_due = 0;
static TimerTime_t replay_not_before = 0;

static uint8_t poll_active = 0;
static uint32_t poll_start = 0;
static uint32_t poll_end = 0;
static uint32_t poll_gap_ms = 0;
static uint16_t poll_cursor = 0;      /* offset from log_head */
static uint16_t poll_cursor_prev = 0;
static TimerTime_t poll_not_before = 0;

/* McpsConfirm arrives from the RTC timer interrupt. Flash erase/program
 * takes tens of ms with interrupts off, so the result is parked here and
 * applied from the main loop by datalog_process(). */
static volatile uint8_t tx_done_pending = 0;
static volatile uint8_t tx_done_ack = 0;

static uintptr_t slot_addr(uint16_t index)
{
	return (uintptr_t)DATALOG_DATA + ((uintptr_t)index * DATALOG_SLOT);
}

static uint8_t slot_written(uint16_t index)
{
	return (*(const uint8_t *)(slot_addr(index) + DATALOG_MARK)) != 0xFF;
}

static uint32_t record_time(const uint8_t *rec)
{
	return ((uint32_t)rec[7] << 24) | ((uint32_t)rec[8] << 16) | ((uint32_t)rec[9] << 8) | rec[10];
}

static void datalog_erase_page(uint16_t page)
{
	uintptr_t addr = (uintptr_t)DATALOG_DATA + ((uintptr_t)page * DATALOG_PAGE);

	if (addr >= (uintptr_t)DATALOG_END) {
		return;
	}
	__disable_irq();
	if (flash_erase_page(addr) == ERRNO_FLASH_SEC_ERROR) {
		LOG_PRINTF(LL_DEBUG, "datalog page erase error\r\n");
	}
	__enable_irq();
	delay_ms(5);
	iwdg_reload();
}

static void datalog_erase_all(void)
{
	uint16_t page;

	for (page = 0; page < DATALOG_PAGES; page++) {
		datalog_erase_page(page);
	}
	log_head = 0;
	log_tail = 0;
	log_count = 0;
	replay_cursor = 0;
}

/* Records occupy a contiguous run of slots in ring order, and the page the
 * run is about to grow into is erased before it is used, so at least one
 * slot is always blank. tail is the blank slot after the run. */
static void datalog_scan(void)
{
	uint16_t i;
	uint16_t tail = 0;
	uint8_t found = 0;
	uint16_t count = 0;

	for (i = 0; i < DATALOG_CAP; i++) {
		uint16_t prev = (uint16_t)((i + DATALOG_CAP - 1u) % DATALOG_CAP);
		if (!slot_written(i) && slot_written(prev)) {
			tail = i;
			found = 1;
			break;
		}
	}
	if (!found) {
		/* Either empty, or a full ring that should not happen. */
		for (i = 0; i < DATALOG_CAP; i++) {
			if (slot_written(i)) {
				datalog_erase_all();
				return;
			}
		}
		log_head = 0;
		log_tail = 0;
		log_count = 0;
		return;
	}
	i = (uint16_t)((tail + DATALOG_CAP - 1u) % DATALOG_CAP);
	while (count < DATALOG_CAP && slot_written(i)) {
		count++;
		i = (uint16_t)((i + DATALOG_CAP - 1u) % DATALOG_CAP);
	}
	log_tail = tail;
	log_count = count;
	log_head = (uint16_t)((tail + DATALOG_CAP - count) % DATALOG_CAP);
}

static uint16_t datalog_next_unacked(uint16_t from)
{
	while (from < log_count) {
		uint16_t index = (uint16_t)((log_head + from) % DATALOG_CAP);
		if (*(const uint8_t *)(slot_addr(index) + 6) & DATALOG_FLAG_NOACK) {
			break;
		}
		from++;
	}
	return from;
}

static void datalog_schedule_replay(void)
{
	if (pnackmd_switch == 1 && datalog_next_unacked(replay_cursor) < log_count) {
		replay_due = 1;
		replay_not_before = TimerGetCurrentTime() + DATALOG_REPLAY_GAP_MS;
	}
}

void datalog_init(void)
{
	datalog_scan();
	replay_cursor = log_count;
	LOG_PRINTF(LL_DEBUG, "datalog: %u stored readings\r\n", (unsigned int)log_count);
}

void datalog_clear(void)
{
	datalog_erase_all();
	inflight_kind = 0;
	inflight_count = 0;
	replay_due = 0;
	poll_active = 0;
}

void datalog_store(const uint8_t record[DATALOG_RECORD_LEN])
{
	uint8_t raw[DATALOG_SLOT];
	uint16_t index;

	if (record == NULL) {
		return;
	}
	index = log_tail;
	if ((index % DATALOG_PER_PAGE) == 0) {
		/* Entering a page: it must be blank. If it still holds the oldest
		 * records they are dropped with it. */
		uint16_t page = (uint16_t)(index / DATALOG_PER_PAGE);
		if (log_count > 0 && page == (uint16_t)(log_head / DATALOG_PER_PAGE)) {
			uint16_t drop = (uint16_t)(DATALOG_PER_PAGE - (log_head % DATALOG_PER_PAGE));
			if (drop > log_count) {
				drop = log_count;
			}
			log_head = (uint16_t)((log_head + drop) % DATALOG_CAP);
			log_count = (uint16_t)(log_count - drop);
			replay_cursor = (replay_cursor > drop) ? (uint16_t)(replay_cursor - drop) : 0;
			if (poll_active) {
				poll_cursor = (poll_cursor > drop) ? (uint16_t)(poll_cursor - drop) : 0;
			}
		}
		datalog_erase_page(page);
	}

	memset(raw, 0xFF, sizeof(raw));
	memcpy(raw, record, DATALOG_RECORD_LEN);
	raw[6] = (uint8_t)(raw[6] & (uint8_t)~DATALOG_FLAG_POLL);
	raw[DATALOG_MARK] = 0x00;
	__disable_irq();
	if (flash_program_bytes(slot_addr(index), raw, DATALOG_SLOT) == ERRNO_FLASH_SEC_ERROR) {
		__enable_irq();
		LOG_PRINTF(LL_DEBUG, "datalog store error\r\n");
		return;
	}
	__enable_irq();

	log_tail = (uint16_t)((index + 1u) % DATALOG_CAP);
	log_count++;
}

void datalog_remember(const uint8_t record[DATALOG_RECORD_LEN])
{
	if (record == NULL) {
		return;
	}
	/* Settle the previous frame before its slot is reused. */
	datalog_process();
	memcpy(inflight, record, DATALOG_RECORD_LEN);
	inflight_kind = 1;
	inflight_count = 1;
}

void datalog_defer(void)
{
	if (inflight_kind == 2) {
		inflight_kind = 0;
		inflight_count = 0;
		datalog_schedule_replay();
		return;
	}
	datalog_on_tx_done(0);
}

void datalog_on_tx_done(uint8_t ack_received)
{
	tx_done_ack = ack_received;
	tx_done_pending = 1;
}

void datalog_process(void)
{
	uint8_t ack_received;

	if (tx_done_pending == 0) {
		return;
	}
	ack_received = tx_done_ack;
	tx_done_pending = 0;

	if (inflight_kind == 1) {
		if (ack_received) {
			inflight[6] = (uint8_t)(inflight[6] & (uint8_t)~DATALOG_FLAG_NOACK);
		} else {
			inflight[6] |= DATALOG_FLAG_NOACK;
		}
		datalog_store(inflight);
		inflight_kind = 0;
		if (ack_received) {
			datalog_schedule_replay();
		}
		return;
	}
	if (inflight_kind == 2) {
		if (ack_received) {
			replay_cursor = replay_cursor_next;
			inflight_kind = 0;
			datalog_schedule_replay();
		} else {
			inflight_kind = 0;
			replay_due = 0;
		}
	}
}

uint8_t datalog_replay_ready(void)
{
	datalog_process();
	if (pnackmd_switch != 1 || replay_due == 0 || inflight_kind != 0 || poll_active) {
		return 0;
	}
	if (datalog_next_unacked(replay_cursor) >= log_count) {
		replay_due = 0;
		return 0;
	}
	if ((int32_t)(TimerGetCurrentTime() - replay_not_before) < 0) {
		return 0;
	}
	return 1;
}

static uint8_t datalog_room(uint8_t max_len)
{
	LoRaMacTxInfo_t txInfo;
	uint8_t room;

	if (LoRaMacQueryTxPossible(0, &txInfo) != LORAMAC_STATUS_OK) {
		room = max_len;
	} else {
		room = txInfo.MaxPossiblePayload;
	}
	if (room > max_len) {
		room = max_len;
	}
	return room;
}

uint8_t datalog_fill_replay(uint8_t *buf, uint8_t max_len)
{
	uint8_t nmax;
	uint8_t n = 0;
	uint16_t off;

	if (buf == NULL) {
		return 0;
	}
	nmax = (uint8_t)(datalog_room(max_len) / DATALOG_RECORD_LEN);
	off = replay_cursor;
	while (n < nmax) {
		uint16_t index;
		off = datalog_next_unacked(off);
		if (off >= log_count) {
			break;
		}
		index = (uint16_t)((log_head + off) % DATALOG_CAP);
		memcpy(buf + (n * DATALOG_RECORD_LEN), (const uint8_t *)slot_addr(index), DATALOG_RECORD_LEN);
		n++;
		off++;
	}
	if (n == 0) {
		replay_due = 0;
		return 0;
	}
	replay_cursor_next = off;
	inflight_kind = 2;
	inflight_count = n;
	replay_due = 0;
	return (uint8_t)(n * DATALOG_RECORD_LEN);
}

void datalog_poll_request(uint32_t start, uint32_t end, uint8_t interval_s)
{
	if (end < start) {
		return;
	}
	poll_start = start;
	poll_end = end;
	poll_gap_ms = (uint32_t)interval_s * 1000u;
	if (poll_gap_ms < DATALOG_POLL_MIN_GAP_MS) {
		poll_gap_ms = DATALOG_POLL_MIN_GAP_MS;
	}
	poll_cursor = 0;
	poll_cursor_prev = 0;
	poll_active = 1;
	poll_not_before = TimerGetCurrentTime() + DATALOG_POLL_MIN_GAP_MS;
	LOG_PRINTF(LL_DEBUG, "datalog poll %u..%u every %us\r\n", (unsigned int)start, (unsigned int)end, (unsigned int)interval_s);
}

uint8_t datalog_poll_ready(void)
{
	datalog_process();
	if (poll_active == 0 || inflight_kind != 0) {
		return 0;
	}
	if ((int32_t)(TimerGetCurrentTime() - poll_not_before) < 0) {
		return 0;
	}
	return 1;
}

uint8_t datalog_fill_poll(uint8_t *buf, uint8_t max_len)
{
	uint8_t nmax;
	uint8_t n = 0;

	if (buf == NULL || poll_active == 0) {
		return 0;
	}
	nmax = (uint8_t)(datalog_room(max_len) / DATALOG_RECORD_LEN);
	poll_cursor_prev = poll_cursor;
	while (poll_cursor < log_count && n < nmax) {
		uint16_t index = (uint16_t)((log_head + poll_cursor) % DATALOG_CAP);
		const uint8_t *src = (const uint8_t *)slot_addr(index);
		uint32_t t = record_time(src);
		poll_cursor++;
		if (t < poll_start || t > poll_end) {
			continue;
		}
		memcpy(buf + (n * DATALOG_RECORD_LEN), src, DATALOG_RECORD_LEN);
		buf[(n * DATALOG_RECORD_LEN) + 6] |= DATALOG_FLAG_POLL;
		n++;
	}
	if (poll_cursor >= log_count) {
		poll_active = 0;
	}
	poll_not_before = TimerGetCurrentTime() + poll_gap_ms;
	return (uint8_t)(n * DATALOG_RECORD_LEN);
}

void datalog_poll_defer(void)
{
	poll_cursor = poll_cursor_prev;
	poll_active = 1;
	poll_not_before = TimerGetCurrentTime() + DATALOG_POLL_MIN_GAP_MS;
}

uint16_t datalog_count(void)
{
	return log_count;
}

uint8_t datalog_read(uint16_t index, uint8_t record[DATALOG_RECORD_LEN])
{
	uint16_t slot;

	if (record == NULL || index >= log_count) {
		return 0;
	}
	slot = (uint16_t)((log_head + index) % DATALOG_CAP);
	memcpy(record, (const uint8_t *)slot_addr(slot), DATALOG_RECORD_LEN);
	return 1;
}
