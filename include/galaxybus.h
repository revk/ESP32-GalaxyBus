// Galaxy RS485 bus
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0

#ifndef	GALAXYBUS_H
#define	GALAXYBUS_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>

#define	GALAXYBUSMAX		64
#define	GALAXYBUSMISSED		-1
#define	GALAXYBUSTOOBIG		-2
#define	GALAXYBUSSTARTBIT	-3
#define	GALAXYBUSSTOPBIT	-4
#define	GALAXYBUSCHECKSUM	-5
#define	GALAXYBUSBREAK		-6
#define	GALAXYBUSBUSY		-7

typedef struct galaxybus_s galaxybus_t;

// Set up
galaxybus_t *galaxybus_init (int8_t timer, int8_t tx, int8_t rx, int8_t de, int8_t re, int8_t clk, uint8_t slave);
void galaxybus_set_timing (galaxybus_t * g, uint32_t pre, uint32_t post, uint32_t gap);
void galaxybus_start (galaxybus_t *);
void *galaxybus_end (galaxybus_t *);

int galaxybus_tx (galaxybus_t *, int len, uint8_t * buf);
int galaxybus_ready (galaxybus_t *);
int galaxybus_rx (galaxybus_t *, int max, uint8_t * buf);

#endif
