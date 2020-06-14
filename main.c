/*
 * Copyright (c) 2020 Damien Miller <djm@mindrot.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include <util/delay.h>

#define SPINDLE_START_TIME_MS	500	/* duration of start pulse */
#define SPINDLE_COAST_TIME_MS	1000	/* holdoff after spindle stop */
#define COLD_START_TIME_MS	2000	/* holdoff on startup */
#define ERROR_RECOVER_TIME_MS	5000	/* holdoff after error */
#define STATUS_TIME_UNIT_MS	100	/* status LED morse code time unit */
#define STATUS_TIME_DOT		(1 * STATUS_TIME_UNIT_MS)
#define STATUS_TIME_DASH	(3 * STATUS_TIME_UNIT_MS)
#define STATUS_TIME_INTERVAL	(1 * STATUS_TIME_UNIT_MS)
#define STATUS_TIME_GAP		(7 * STATUS_TIME_UNIT_MS)

enum state {
	S_ERROR = 0,		/* error state, e.g. when fwd+rev asserted */
	S_COLD_START,		/* initial state */
	S_ESTOPPED,		/* estop asserted */
	S_READY,		/* estop clear but no spindle dir asserted */
	S_FWD_START,		/* spindle fwd asserted; start pulse active */
	S_FWD,			/* spindle fwd */
	S_FWD_SPINDOWN,		/* hold delay after spindle fwd deassert */
	S_REV_START,		/* spindle rev asserted; start pulse active */
	S_REV,			/* spindle rev */
	S_REV_SPINDOWN,		/* hold delay after spindle fwd deassert */
	S_MAX,			/* maximum state value: do not use */
};

/* current state */
static enum state state = S_COLD_START;

/* 1KHz timer interrupt */
static uint32_t timer_1k;		/* tick count; don't use directly */
static uint16_t timer_1k_oneshot;	/* oneshot countdown timer */
static bool timer_1k_done;		/* timer expired; don't use directly */

ISR(TIM0_COMPA_vect)
{
	timer_1k++;
	if (timer_1k_oneshot != 0) {
		if (--timer_1k_oneshot == 0)
			timer_1k_done = true;
	}
}

static void
timer_1k_init(void)
{
	cli();
	TCCR0A = (1 << WGM01); /* timer0 CTC mode */
	TCCR0B = 0;
	TIMSK0 = (1 << OCIE0A); /* enable CTC interrupt */

	OCR0A = 125; /* 1ms for 1MHz CPU and /8 prescale */
	TCCR0B = (1 << CS01); /* /8 prescale */
	sei();
}

/* access to monotonic 1KHz tick count. NB. wraps */
static uint32_t
timer_1k_val(void)
{
	uint32_t ret;

	cli();
	ret = timer_1k;
	sei();
	return ret;
}

/* start oneshot countdown timer, clobbering any existing timer running */
static void
timer_oneshot(uint16_t ms)
{
	cli();
	timer_1k_done = false;
	timer_1k_oneshot = ms;
	sei();
}

/* access to countdown timer expired status */
static bool
timer_oneshot_done(void)
{
	bool ret;

	cli();
	ret = timer_1k_done;
	sei();
	return ret;
}

/* cancel a scheduled countdown timer */
static void
timer_oneshot_cancel(void)
{
	timer_oneshot(0);
}

/* Outputs */

static void
out_light(bool on)
{
	PORTA = (PORTA & ~(1<<0)) | (on ? (1<<0) : 0);
}

static void
out_inhibit(bool on)
{
	PORTA = (PORTA & ~(1<<1)) | (on ? (1<<1) : 0);
}

static void
out_start(bool on)
{
	PORTA = (PORTA & ~(1<<2)) | (on ? (1<<2) : 0);
}

static void
out_direction(bool on)
{
	PORTA = (PORTA & ~(1<<3)) | (on ? (1<<3) : 0);
}

static void
out_status(bool on)
{
	PORTA = (PORTA & ~(1<<4)) | (on ? (1<<4) : 0);
}

/*
 * Status LED morse code patterns.
 * The upper nibble contains the pattern length, the lower nibble contains
 * the dot/dash sequence (set bits are dash, clear are dots).
 */
uint8_t stateblink[] = {
	(4 << 4) | 0x9, /* S_ERROR		morse: -..-	'X' */
	(3 << 4) | 0x0, /* S_COLD_START		morse: ...	'S' */
	(1 << 4) | 0x1, /* S_ESTOPPED		morse: .	'E' */
	(3 << 4) | 0x2, /* S_READY		morse: .-.	'R' */
	(2 << 4) | 0x2, /* S_FWD_START		morse: .-	'A' */
	(4 << 4) | 0x1, /* S_FWD		morse: -...	'B' */
	(4 << 4) | 0x3, /* S_FWD_SPINDOWN	morse: -.-.	'C' */
	(2 << 4) | 0x0, /* S_REV_START		morse: ..	'I' */
	(4 << 4) | 0xe, /* S_REV		morse: .---	'J' */
	(3 << 4) | 0x5, /* S_REV_SPINDOWN	morse: -.-	'K' */
	(3 << 4) | 0x4, /* unknown		morse: ..-	'U' */
};

/* state advance functions; these enforce preconditions and start/stop timer */

static void
advance_error(void)
{
	timer_oneshot(ERROR_RECOVER_TIME_MS);
	state = S_ERROR;
}

static void
advance_estopped(void)
{
	switch (state) {
	case S_ERROR:
	case S_COLD_START:
	case S_READY:
	case S_FWD_SPINDOWN:
	case S_REV_SPINDOWN:
		break;
	default:
		advance_error();
		return;
	}
	timer_oneshot_cancel();
	state = S_ESTOPPED;
}

static void
advance_ready(void)
{
	switch (state) {
	case S_ESTOPPED:
	case S_FWD_SPINDOWN:
	case S_REV_SPINDOWN:
		break;
	default:
		advance_error();
		return;
	}
	timer_oneshot_cancel();
	state = S_READY;
}

static void
advance_fwd_start(void)
{
	switch (state) {
	case S_READY:
	case S_FWD_SPINDOWN:
		break;
	default:
		advance_error();
		return;
	}
	timer_oneshot(SPINDLE_START_TIME_MS);
	state = S_FWD_START;
}

static void
advance_fwd(void)
{
	switch (state) {
	case S_FWD_START:
		break;
	default:
		advance_error();
		return;
	}
	timer_oneshot_cancel();
	state = S_FWD;
}

static void
advance_fwd_spindown(void)
{
	switch (state) {
	case S_FWD_START:
	case S_FWD:
		break;
	default:
		advance_error();
		return;
	}
	timer_oneshot(SPINDLE_COAST_TIME_MS);
	state = S_FWD_SPINDOWN;
}

static void
advance_rev_start(void)
{
	switch (state) {
	case S_READY:
	case S_REV_SPINDOWN:
		break;
	default:
		advance_error();
		return;
	}
	timer_oneshot(SPINDLE_START_TIME_MS);
	state = S_REV_START;
}

static void
advance_rev(void)
{
	switch (state) {
	case S_REV_START:
		break;
	default:
		advance_error();
		return;
	}
	timer_oneshot_cancel();
	state = S_REV;
}

static void
advance_rev_spindown(void)
{
	switch (state) {
	case S_REV_START:
	case S_REV:
		break;
	default:
		advance_error();
		return;
	}
	timer_oneshot(SPINDLE_COAST_TIME_MS);
	state = S_REV_SPINDOWN;
}

int
main(void)
{
	enum state ostate;
	uint32_t status_timeout = 0;
	uint32_t status_times[32];
	uint8_t i, j, x, status_len = 0, status_phase = 0;

	/* Leave clock at 1MHz; plenty fast for this */
#if 0
	CLKPR = 0x80;
	CLKPR = 0x00; /* 8 MHz */
#endif

	DDRA = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);
	DDRB = 0;
	PORTA = (1 << 7); /* pullup: estopok */
	PORTB = (1 << 0) | (1 << 1) | (1 << 2); /* pullup: light, fwd, rev */

	timer_1k_init();

	timer_oneshot(COLD_START_TIME_MS);
	for (;;) {
		/* pins are active low */
		bool in_light = !(PINB & (1<<2));
		bool in_fwd = !(PINB & (1<<1));
		bool in_rev = !(PINB & (1<<0));
		bool in_estopok = !(PINA & (1<<7));

		/* Update state based on inputs */
		ostate = state;
		switch (state) {
		case S_ERROR:
			/* stay in error state if inputs still bad */
			if (in_fwd && in_rev)
				advance_error();
			else if (timer_oneshot_done())
				advance_estopped(); /* recover after timeout */
			break;
		case S_COLD_START:
			if (timer_oneshot_done())
				advance_estopped();
			break;
		case S_ESTOPPED:
			if (in_fwd && in_rev)
				advance_error();
			else if (in_estopok)
				advance_ready();
			break;
		case S_READY:
			if (in_fwd && in_rev)
				advance_error();
			else if (!in_estopok)
				advance_estopped();
			else if (in_fwd)
				advance_fwd_start();
			else if (in_rev)
				advance_rev_start();
			break;
		case S_FWD_START:
			if (in_rev)
				advance_error();
			else if (!in_estopok || !in_fwd)
				advance_fwd_spindown();
			else if (timer_oneshot_done())
				advance_fwd();
			break;
		case S_FWD:
			if (in_rev)
				advance_error();
			else if (!in_estopok || !in_fwd)
				advance_fwd_spindown();
			break;
		case S_FWD_SPINDOWN:
			if (in_rev)
				advance_error();
			else if (in_estopok && in_fwd)
				advance_fwd_start();
			else if (timer_oneshot_done()) {
				if (!in_estopok)
					advance_estopped();
				else
					advance_ready();
			}
			break;
		case S_REV_START:
			if (in_fwd)
				advance_error();
			else if (!in_estopok || !in_rev)
				advance_rev_spindown();
			else if (timer_oneshot_done())
				advance_rev();
			break;
		case S_REV:
			if (in_fwd)
				advance_error();
			else if (!in_estopok || !in_rev)
				advance_rev_spindown();
			break;
		case S_REV_SPINDOWN:
			if (in_fwd)
				advance_error();
			else if (in_estopok && in_rev)
				advance_rev_start();
			else if (timer_oneshot_done()) {
				if (!in_estopok)
					advance_estopped();
				else
					advance_ready();
			}
			break;
		default:
			/* shouldn't happen */
			advance_error();
			break;
		}

		/* prepare/update Morse code status pattern on state change */
		if (status_len == 0 || ostate != state) {
			j = 0;
			status_times[j++] = STATUS_TIME_GAP;
			/*
			 * prepare tick intervals in status_times[]
			 * odd-numbered phases correspond to lit symbols
			 * (i.e. dots or dashes), even numbered entries
			 * are inter-symbol intervals or inter-letter gaps.
			 */
			for (i = 0; i < stateblink[state] >> 4; i++) {
				x = ((stateblink[state] & (1 << i)) != 0);
				status_times[j++] = x ? STATUS_TIME_DASH :
				    STATUS_TIME_DOT;
				status_times[j++] = STATUS_TIME_INTERVAL;
			}
			status_len = j;
			status_phase = 0; /* start new sequence with gap */
			status_timeout = timer_1k_val() + status_times[0];
		} else if (timer_1k_val() == status_timeout) {
			/* advance phase at expiry of current interval */
			status_phase = (status_phase + 1) % status_len;
			status_timeout = timer_1k_val() +
			    status_times[status_phase];
		}

		/* display status */
		out_status(status_phase & 1);

		/* act on current state */
		switch (state) {
		case S_COLD_START:
			out_light(0);
			out_inhibit(0);
			out_start(0);
			out_direction(0);
			break;
		case S_ESTOPPED:
		case S_READY:
			out_light(in_light);
			out_inhibit(0);
			out_start(0);
			out_direction(0);
			break;
		case S_FWD_START:
			out_light(in_light);
			out_inhibit(1);
			out_start(1);
			out_direction(0);
			break;
		case S_FWD:
			out_light(in_light);
			out_inhibit(1);
			out_start(0);
			out_direction(0);
			break;
		case S_FWD_SPINDOWN:
			out_light(in_light);
			out_inhibit(0);
			out_start(0);
			out_direction(0);
			break;
		case S_REV_START:
			out_light(in_light);
			out_inhibit(1);
			out_start(1);
			out_direction(1);
			break;
		case S_REV:
			out_light(in_light);
			out_inhibit(1);
			out_start(0);
			out_direction(1);
			break;
		case S_REV_SPINDOWN:
			out_light(in_light);
			out_inhibit(0);
			out_start(0);
			out_direction(1);
			break;
		default:
			out_light(0);
			out_inhibit(0);
			out_start(0);
			/*
			 * NB. don't touch direction on error since we don't
			 * know what its previous state was and we might be
			 * coming from S_REV (i.e. energised and reversed)
			 * and must not switch direction until the motor
			 * has spun down. The S_ERROR->S_ESTOPPED recovery
			 * will take care of resetting it eventually.
			 */
			break;
		}
	}
}
