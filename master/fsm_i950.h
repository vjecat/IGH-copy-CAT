/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2008  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

/**
   \file
   EtherCAT slave scanning state machine.
*/

/*****************************************************************************/

#ifndef __EC_FSM_I950_H__
#define __EC_FSM_I950_H__

#include "globals.h"
#include "datagram.h"
#include "slave.h"
#include "fsm_sii.h"
#include "fsm_change.h"
#include "fsm_coe.h"
#include "fsm_pdo.h"
#include "fsm_slave_scan.h"

/*****************************************************************************/

typedef struct ec_fsm_i950 ec_fsm_i950_t;

/** Finite state machine for scanning an EtherCAT slave.
 */
struct ec_fsm_i950
{
    ec_slave_t *slave; /**< Slave the FSM runs on. */
    ec_datagram_t *datagram; /**< Datagram used in the state machine. */
    // ec_fsm_slave_config_t *fsm_slave_config; /**< Slave conf. state machine to use. */
    // ec_fsm_pdo_t *fsm_pdo; /**< PDO configuration state machine to use. */
    unsigned int retries; /**< Retries on datagram timeout. */

    void (*state)(ec_fsm_i950_t *); /**< State function. */
    ec_fsm_coe_t *fsm_coe; /**< CoE state machine to use. */
    unsigned long jiffies_start; /**< Start timestamp. */

};

/*****************************************************************************/

// ec_fsm_slave_config_t *, ,  ec_fsm_pdo_t *
void ec_fsm_i950_init(ec_fsm_i950_t *, ec_datagram_t *);
void ec_fsm_i950_clear(ec_fsm_i950_t *);

void ec_fsm_i950_start(ec_fsm_i950_t *, ec_slave_t *);
void ec_fsm_i950_enter(ec_fsm_i950_t *, ec_slave_t *);

int ec_fsm_i950_exec(ec_fsm_i950_t *);
int ec_fsm_i950_success(const ec_fsm_i950_t *);

/*****************************************************************************/

#endif
