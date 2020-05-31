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

static uint32_t timer_1k;
static uint16_t timer_1k_oneshot;
static bool timer_1k_done;

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
	TCCR0B |= (1 << WGM02); /* timer1 CTC mode */
	TIMSK0 |= (1 << OCIE0A); /* enable CTC interrupt */

	cli();
	OCR0A = 125; /* 1ms for 1MHz CPU and /8 prescale */
	TCCR0B |= (1 << CS01); /* /8 prescale */
	sei();
}

static uint32_t
timer_1k_val(void)
{
	uint32_t ret;

	cli();
	ret = timer_1k;
	sei();
	return ret;
}

static void
timer_oneshot(uint16_t ms)
{
	cli();
	timer_1k_done = false;
	timer_1k_oneshot = ms;
	sei();
}

static bool
timer_oneshot_done(void)
{
	bool ret;

	cli();
	ret = timer_1k_done;
	sei();
	return ret;
}

static void
timer_oneshot_cancel(void)
{
	timer_oneshot(0);
}

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

enum state {
	S_ERROR = 0,		/* error state, e.g. when fwd+rev asserted */
	S_ESTOPPED,		/* initial state, default when estop asserted */
	S_READY,		/* estop clear but no spindle asserted */
	S_FWD_START,		/* spindle fwd started; delay for start pulse */
	S_FWD,			/* spindle fwd */
	S_FWD_SPINDOWN,		/* hold delay after spindle fwd deassert */
	S_REV_START,		/* spindle rev started; delay for start pulse */
	S_REV,			/* spindle rev */
	S_REV_SPINDOWN,		/* hold delay after spindle fwd deassert */
	S_MAX,			/* maximum state value: do not use */
};

uint8_t stateblink[] = {
	(4 << 4) | 0x9, /* S_ERROR		morse: -..-	'X' */
	(1 << 4) | 0x1, /* S_ESTOPPED		morse: .	'E' */
	(3 << 4) | 0x2, /* S_READY		morse: .-.	'R' */
	(2 << 4) | 0x2, /* S_FWD_START		morse: .-	'A' */
	(4 << 4) | 0x1, /* S_FWD		morse: -...	'B' */
	(4 << 4) | 0x3, /* S_FWD_SPINDOWN	morse: -.-.	'C' */
	(2 << 4) | 0x0, /* S_FWD_START		morse: ..	'I' */
	(4 << 4) | 0xe, /* S_FWD		morse: .---	'J' */
	(3 << 4) | 0x5, /* S_FWD_SPINDOWN	morse: -.-	'K' */
	(3 << 4) | 0x4, /* unknown		morse: ..-	'U' */
};

#define SPINDLE_START_TIME_MS	200
#define SPINDLE_COAST_TIME_MS	1000
#define ERROR_RECOVER_TIME_MS	2000
#define STATUS_TIME_UNIT	100	/* ms */
#define STATUS_TIME_DOT		(1 * STATUS_TIME_UNIT)
#define STATUS_TIME_DASH	(3 * STATUS_TIME_UNIT)
#define STATUS_TIME_INTERVAL	(1 * STATUS_TIME_UNIT)
#define STATUS_TIME_GAP		(3 * STATUS_TIME_UNIT)

int
main(void)
{
	enum state ostate, state;
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

	state = ostate = S_ESTOPPED;
	for (;;) {
		bool in_light = !!(PINB & (1<<2));
		bool in_fwd = !!(PINB & (1<<1));
		bool in_rev = !!(PINB & (1<<0));
		bool in_estopok = !!(PINA & (1<<7));

		/* Update state based on inputs */
		ostate = state;
		switch (state) {
		case S_ERROR:
			/* recover from errors after long timeout */
			if (timer_oneshot_done())
				state = S_ESTOPPED;
			break;
		case S_ESTOPPED:
			if (in_estopok)
				state = S_READY;
			break;
		case S_READY:
			if (in_fwd && in_rev) {
				state = S_ERROR;
				timer_oneshot(ERROR_RECOVER_TIME_MS);
			} else if (!in_estopok)
				state = S_ESTOPPED;
			else if (in_fwd) {
				timer_oneshot(SPINDLE_START_TIME_MS);
				state = S_FWD_START;
			} else if (in_rev) {
				timer_oneshot(SPINDLE_START_TIME_MS);
				state = S_REV_START;
			}
			break;
		case S_FWD_START:
			if (in_rev) {
				state = S_ERROR;
				timer_oneshot(ERROR_RECOVER_TIME_MS);
			} else if (!in_estopok || !in_fwd) {
				timer_oneshot(SPINDLE_COAST_TIME_MS);
				state = S_FWD_SPINDOWN;
			} else if (timer_oneshot_done())
				state = S_FWD;
			break;
		case S_FWD:
			if (in_rev) {
				state = S_ERROR;
				timer_oneshot(ERROR_RECOVER_TIME_MS);
			} else if (!in_estopok || !in_fwd) {
				timer_oneshot(SPINDLE_COAST_TIME_MS);
				state = S_FWD_SPINDOWN;
			}
			break;
		case S_FWD_SPINDOWN:
			if (in_rev) {
				state = S_ERROR;
				timer_oneshot(ERROR_RECOVER_TIME_MS);
			} else if (in_estopok && in_fwd) {
				timer_oneshot_cancel();
				state = S_FWD_START;
			} else if (timer_oneshot_done()) {
				if (!in_estopok)
					state = S_ESTOPPED;
				else
					state = S_READY;
			}
			break;
		case S_REV_START:
			if (in_fwd) {
				state = S_ERROR;
				timer_oneshot(ERROR_RECOVER_TIME_MS);
			} else if (!in_estopok || !in_rev) {
				timer_oneshot(SPINDLE_COAST_TIME_MS);
				state = S_REV_SPINDOWN;
			} else if (timer_oneshot_done())
				state = S_REV;
			break;
		case S_REV:
			if (in_fwd) {
				state = S_ERROR;
				timer_oneshot(ERROR_RECOVER_TIME_MS);
			} else if (!in_estopok || !in_rev) {
				timer_oneshot(SPINDLE_COAST_TIME_MS);
				state = S_REV_SPINDOWN;
			}
			break;
		case S_REV_SPINDOWN:
			if (in_fwd) {
				state = S_ERROR;
				timer_oneshot(ERROR_RECOVER_TIME_MS);
			} else if (in_estopok && in_rev) {
				timer_oneshot_cancel();
				state = S_REV_START;
			} else if (timer_oneshot_done()) {
				if (!in_estopok)
					state = S_ESTOPPED;
				else
					state = S_READY;
			}
			break;
		default:
			/* shouldn't happen */
			state = S_ERROR;
			timer_oneshot(ERROR_RECOVER_TIME_MS);
			break;
		}

		/* prepare/update Morse code status pattern */
		if (status_len == 0 || ostate != state) {
			j = 0;
			status_times[j++] = STATUS_TIME_GAP;
			/*
			 * prepare tick intervals in status_times[]
			 * odd-numbered entries correspond to lit phases
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
			/* advance phase */
			status_phase = (status_phase + 1) % status_len;
			status_timeout = timer_1k_val() +
			    status_times[status_phase];
		}

		/* act on state */
		switch (state) {
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
			/* don't touch direction */
			break;
		}

		/* display status */
		out_status(status_phase & 1);
	}
}
