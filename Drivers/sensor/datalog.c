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
 * board share it). LA66 erases this span in
 * FLASH_erase_all_sensor_data_storage and never fills it. This file
 * uses that span as the queue of readings the network did not hear.
 * Keys stay at FLASH_USER_START_ADDR_KEY, which is the end address. */
#define DATALOG_PAGE      0x1000u
#define DATALOG_META      FLASH_SENSOR_DATA_START_ADDR
#define DATALOG_DATA      (FLASH_SENSOR_DATA_START_ADDR + DATALOG_PAGE)
#define DATALOG_END       FLASH_SENSOR_DATA_END_ADDR
#define DATALOG_SLOT      16u
#define DATALOG_PER_PAGE  (DATALOG_PAGE / DATALOG_SLOT)
#define DATALOG_CAP       ((DATALOG_END - DATALOG_DATA) / DATALOG_SLOT)
#define DATALOG_MAGIC     0x444C4731u
#define DATALOG_REPLAY_GAP_MS 10000u

uint8_t pnackmd_switch = 0;
uint8_t time_synchronization_method = 0;
uint8_t time_synchronization_interval = 10;

static uint16_t log_head = 0;
static uint16_t log_tail = 0;
static uint16_t log_count = 0;

static uint8_t inflight[DATALOG_RECORD_LEN];
static uint8_t inflight_kind = 0; /* 1 = live sample waiting for an answer, 2 = replay */
static uint8_t inflight_count = 0;
static uint8_t replay_due = 0;
static TimerTime_t replay_not_before = 0;

/* McpsConfirm arrives from the RTC timer interrupt. Flash erase/program
 * takes tens of ms with interrupts off, so the result is parked here and
 * applied from the main loop by datalog_process(). */
static volatile uint8_t tx_done_pending = 0;
static volatile uint8_t tx_done_ack = 0;

static void datalog_save_meta(void)
{
	uint8_t raw[16];
	uint32_t magic = DATALOG_MAGIC;

	memset(raw, 0xFF, sizeof(raw));
	memcpy(raw, &magic, 4);
	raw[4] = (uint8_t)(log_head & 0xFF);
	raw[5] = (uint8_t)(log_head >> 8);
	raw[6] = (uint8_t)(log_tail & 0xFF);
	raw[7] = (uint8_t)(log_tail >> 8);
	raw[8] = (uint8_t)(log_count & 0xFF);
	raw[9] = (uint8_t)(log_count >> 8);

	__disable_irq();
	if (flash_erase_page(DATALOG_META) == ERRNO_FLASH_SEC_ERROR) {
		LOG_PRINTF(LL_DEBUG, "datalog meta erase error\r\n");
	}
	delay_ms(5);
	if (flash_program_bytes(DATALOG_META, raw, 16) == ERRNO_FLASH_SEC_ERROR) {
		LOG_PRINTF(LL_DEBUG, "datalog meta write error\r\n");
	}
	__enable_irq();
	iwdg_reload();
}

static void datalog_erase_data_page(uint16_t page)
{
	uint32_t addr = DATALOG_DATA + ((uint32_t)page * DATALOG_PAGE);

	if (addr < DATALOG_DATA || addr >= DATALOG_END) {
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
	uint32_t addr;

	for (addr = DATALOG_META; addr < DATALOG_END; addr += DATALOG_PAGE) {
		__disable_irq();
		if (flash_erase_page(addr) == ERRNO_FLASH_SEC_ERROR) {
			LOG_PRINTF(LL_DEBUG, "datalog erase error\r\n");
		}
		__enable_irq();
		delay_ms(20);
		iwdg_reload();
	}
	log_head = 0;
	log_tail = 0;
	log_count = 0;
	datalog_save_meta();
}

static void datalog_load_meta(void)
{
	const uint8_t *p = (const uint8_t *)DATALOG_META;
	uint32_t magic;

	memcpy(&magic, p, 4);
	log_head = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
	log_tail = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
	log_count = (uint16_t)p[8] | ((uint16_t)p[9] << 8);
	if (magic != DATALOG_MAGIC || log_head >= DATALOG_CAP || log_tail >= DATALOG_CAP || log_count > DATALOG_CAP) {
		datalog_erase_all();
	}
}

/* Drop the oldest page so the page we are about to write is blank. */
static void datalog_drop_oldest_page(void)
{
	uint16_t page;
	uint16_t drop;

	if (log_count == 0) {
		return;
	}
	page = (uint16_t)((log_head % DATALOG_CAP) / DATALOG_PER_PAGE);
	drop = (uint16_t)(DATALOG_PER_PAGE - ((log_head % DATALOG_CAP) % DATALOG_PER_PAGE));
	if (drop > log_count) {
		drop = log_count;
	}
	log_head = (uint16_t)((log_head + drop) % DATALOG_CAP);
	log_count = (uint16_t)(log_count - drop);
	datalog_erase_data_page(page);
}

static void datalog_schedule_replay(void)
{
	if (pnackmd_switch == 1 && log_count > 0) {
		replay_due = 1;
		replay_not_before = TimerGetCurrentTime() + DATALOG_REPLAY_GAP_MS;
	}
}

void datalog_init(void)
{
	datalog_load_meta();
}

void datalog_clear(void)
{
	datalog_erase_all();
	inflight_kind = 0;
	inflight_count = 0;
	replay_due = 0;
}

void datalog_store(const uint8_t record[DATALOG_RECORD_LEN])
{
	uint8_t raw[DATALOG_SLOT];
	uint16_t index;
	uint32_t addr;
	uint16_t page;

	if (record == NULL) {
		return;
	}
	index = (uint16_t)(log_tail % DATALOG_CAP);
	page = (uint16_t)(index / DATALOG_PER_PAGE);
	if ((index % DATALOG_PER_PAGE) == 0) {
		if (log_count > 0 && page == (uint16_t)((log_head % DATALOG_CAP) / DATALOG_PER_PAGE)) {
			datalog_drop_oldest_page();
			index = (uint16_t)(log_tail % DATALOG_CAP);
			page = (uint16_t)(index / DATALOG_PER_PAGE);
		}
		datalog_erase_data_page(page);
	} else if (log_count + 1u >= DATALOG_CAP) {
		datalog_drop_oldest_page();
		index = (uint16_t)(log_tail % DATALOG_CAP);
	}

	memset(raw, 0xFF, sizeof(raw));
	memcpy(raw, record, DATALOG_RECORD_LEN);
	addr = DATALOG_DATA + ((uint32_t)index * DATALOG_SLOT);
	__disable_irq();
	if (flash_program_bytes(addr, raw, DATALOG_SLOT) == ERRNO_FLASH_SEC_ERROR) {
		LOG_PRINTF(LL_DEBUG, "datalog store error\r\n");
		__enable_irq();
		return;
	}
	__enable_irq();

	log_tail = (uint16_t)((index + 1u) % DATALOG_CAP);
	if (log_count < DATALOG_CAP) {
		log_count++;
	}
	datalog_save_meta();
	LOG_PRINTF(LL_DEBUG, "datalog stored, waiting=%u\r\n", (unsigned int)log_count);
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
		if (ack_received == 0 && pnackmd_switch == 1) {
			datalog_store(inflight);
		}
		inflight_kind = 0;
		if (ack_received != 0) {
			datalog_schedule_replay();
		}
		return;
	}
	if (inflight_kind == 2) {
		if (ack_received != 0) {
			uint8_t n = inflight_count;
			while (n > 0 && log_count > 0) {
				uint16_t page_before = (uint16_t)((log_head % DATALOG_CAP) / DATALOG_PER_PAGE);
				log_head = (uint16_t)((log_head + 1u) % DATALOG_CAP);
				log_count--;
				n--;
				if (page_before != (uint16_t)((log_head % DATALOG_CAP) / DATALOG_PER_PAGE)) {
					datalog_erase_data_page(page_before);
				}
			}
			datalog_save_meta();
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
	if (pnackmd_switch != 1 || replay_due == 0 || inflight_kind != 0 || log_count == 0) {
		return 0;
	}
	if ((int32_t)(TimerGetCurrentTime() - replay_not_before) < 0) {
		return 0;
	}
	return 1;
}

uint8_t datalog_fill_replay(uint8_t *buf, uint8_t max_len)
{
	LoRaMacTxInfo_t txInfo;
	uint8_t room;
	uint8_t nrec;
	uint8_t i;

	if (buf == NULL || log_count == 0) {
		return 0;
	}
	if (LoRaMacQueryTxPossible(0, &txInfo) != LORAMAC_STATUS_OK) {
		room = max_len;
	} else {
		room = txInfo.MaxPossiblePayload;
	}
	if (room > max_len) {
		room = max_len;
	}
	nrec = (uint8_t)(room / DATALOG_RECORD_LEN);
	if (nrec > log_count) {
		nrec = (uint8_t)log_count;
	}
	if (nrec == 0) {
		return 0;
	}
	for (i = 0; i < nrec; i++) {
		uint16_t index = (uint16_t)((log_head + i) % DATALOG_CAP);
		const uint8_t *src = (const uint8_t *)(DATALOG_DATA + ((uint32_t)index * DATALOG_SLOT));
		memcpy(buf + (i * DATALOG_RECORD_LEN), src, DATALOG_RECORD_LEN);
	}
	inflight_kind = 2;
	inflight_count = nrec;
	replay_due = 0;
	return (uint8_t)(nrec * DATALOG_RECORD_LEN);
}
