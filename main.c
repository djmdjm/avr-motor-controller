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
#define STATUS_BLINK_MS		150

#define APPROX_TIMEOUT(now, ms) ((now) + ((uint32_t)(ms) * (F_CPU / 1000UL)))

int
main(void)
{
	enum state state = S_ESTOPPED;
	uint32_t timeout = 0, ctr = 0;

	CLKPR = 0x80;
	CLKPR = 0x00; /* 8 MHz */

	DDRA = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);
	PORTA = (1 << 7); /* pullup: estopok */
	DDRB = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
	PORTB = 0;

	for (;; ctr++) {
		bool in_light = !!(PORTB & (1<<2));
		bool in_fwd = !!(PORTB & (1<<1));
		bool in_rev = !!(PORTB & (1<<0));
		bool in_estopok = !!(PORTA & (1<<7));

		out_light(in_light);

		/* Update state based on inputs */
		switch (state) {
		case S_ERROR:
			/* XXX sticks here, maybe not? */
			break;
		case S_ESTOPPED:
			if (in_estopok)
				state = S_READY;
			break;
		case S_READY:
			if (in_fwd && in_rev)
				state = S_ERROR;
			else if (!in_estopok)
				state = S_ESTOPPED;
			else if (in_fwd) {
				timeout = APPROX_TIMEOUT(ctr,
				    SPINDLE_START_TIME_MS);
				state = S_FWD_START;
			} else if (in_rev) {
				timeout = APPROX_TIMEOUT(ctr,
				    SPINDLE_START_TIME_MS);
				state = S_REV_START;
			}
			break;
		case S_FWD_START:
			if (in_rev)
				state = S_ERROR;
			else if (!in_estopok || !in_fwd)
				state = S_FWD_SPINDOWN;
			else if (timeout == ctr)
				state = S_FWD;
			break;
		case S_FWD:
			if (in_rev)
				state = S_ERROR;
			else if (!in_estopok || !in_fwd) {
				timeout = APPROX_TIMEOUT(ctr,
				    SPINDLE_COAST_TIME_MS);
				state = S_FWD_SPINDOWN;
			}
			break;
		case S_FWD_SPINDOWN:
			if (in_rev)
				state = S_ERROR;
			else if (in_estopok && in_fwd) {
				/* allow restart if coasting */
				state = S_FWD;
			} else if (timeout == ctr) {
				if (!in_estopok)
					state = S_ESTOPPED;
				else
					state = S_READY;
			}
			break;
		case S_REV_START:
			if (in_fwd)
				state = S_ERROR;
			else if (!in_estopok || !in_rev)
				state = S_REV_SPINDOWN;
			else if (timeout == ctr) {
				state = S_REV;
			}
			break;
		case S_REV:
			if (in_fwd)
				state = S_ERROR;
			else if (!in_estopok || !in_rev) {
				timeout = APPROX_TIMEOUT(ctr,
				    SPINDLE_COAST_TIME_MS);
				state = S_REV_SPINDOWN;
			}
			break;
		case S_REV_SPINDOWN:
			if (in_fwd)
				state = S_ERROR;
			else if (in_estopok && in_rev) {
				/* allow restart if coasting */
				state = S_REV;
			} else if (timeout == ctr) {
				/* XXX cancel timer */
				if (!in_estopok)
					state = S_ESTOPPED;
				else
					state = S_READY;
			}
			break;
		default:
			/* shouldn't happen */
			state = S_ERROR;
			break;
		}

		/* act on state */
		switch (state) {
		case S_ESTOPPED:
		case S_READY:
			out_inhibit(0);
			out_start(0);
			out_direction(0);
			break;
		case S_FWD_START:
			out_inhibit(1);
			out_start(1);
			out_direction(0);
		case S_FWD:
			out_inhibit(1);
			out_start(0);
			out_direction(0);
			break;
		case S_FWD_SPINDOWN:
			out_inhibit(0);
			out_start(0);
			out_direction(0);
			break;
		case S_REV_START:
			out_inhibit(1);
			out_start(1);
			out_direction(1);
		case S_REV:
			out_inhibit(1);
			out_start(0);
			out_direction(1);
			break;
		case S_REV_SPINDOWN:
			out_inhibit(0);
			out_start(0);
			out_direction(1);
			break;
		default:
			break;
		}

		/* display status */
		/* XXX */
		out_status(1);
	}
}
