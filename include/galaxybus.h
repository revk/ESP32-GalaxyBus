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
#define	galaxybus_errs	\
	p(OK)	\
	p(MISSED)	\
	p(TOOBIG)	\
	p(STARTBIT)	\
	p(STOPBIT)	\
	p(CHECKSUM)	\
	p(BREAK)	\
	p(BUSY)		\
	p(PENDING)	\
	p(MAX)		\

typedef enum
{
#define p(n)    GALAXYBUS_ERR_##n,
	   galaxybus_errs
#undef p
} galaxybus_err_t;

typedef struct galaxybus_s galaxybus_t;

// Set up
galaxybus_t *galaxybus_init (int8_t timer, int8_t tx, int8_t rx, int8_t de, int8_t re, int8_t clk, uint8_t slave);
void galaxybus_set_timing (galaxybus_t * g, uint8_t txpre, uint8_t txpost, uint8_t rxpre,uint8_t rxpost);
void galaxybus_start (galaxybus_t *);
void *galaxybus_end (galaxybus_t *);

int galaxybus_tx (galaxybus_t *, int len, uint8_t * buf);
int galaxybus_ready (galaxybus_t *);
int galaxybus_rx (galaxybus_t *, int max, uint8_t * buf);

const char * galaxybus_err_to_name (int e);

#endif
