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

typedef struct galaxybus_s galaxybus_t;

// Set up
galaxybus_t *galaxybus_init (int8_t uart, int8_t tx, int8_t rx, int8_t de, int8_t re, uint8_t address);
void *galaxybus_end (galaxybus_t *);

// Low level messaging
void galaxybus_tx (galaxybus_t *, int len, uint8_t * buf);
int galaxybus_rx (galaxybus_t *, int max, uint8_t * buf);
int galaxybus_poll (galaxybus_t *, uint8_t address, int max, uint8_t * buf);


// Higher level task as slave
// TODO

#endif
