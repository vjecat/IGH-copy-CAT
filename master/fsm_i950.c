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
   EtherCAT slave state machines.
*/

/*****************************************************************************/

#include "globals.h"
#include "master.h"
#include "mailbox.h"
#include "slave_config.h"

#include "fsm_i950.h"

/*****************************************************************************/


void ec_fsm_i950_state_start(ec_fsm_i950_t *);
void ec_fsm_i950_state_fmmu0(ec_fsm_i950_t *);
void ec_fsm_i950_state_fmmu2(ec_fsm_i950_t *);
void ec_fsm_i950_state_sm2(ec_fsm_i950_t *);
void ec_fsm_i950_state_sm3(ec_fsm_i950_t *);
// void ec_fsm_i950_state_state(ec_fsm_i950_t *);
// void ec_fsm_slave_scan_state_dc_cap(ec_fsm_slave_scan_t *);
// void ec_fsm_slave_scan_state_dc_times(ec_fsm_slave_scan_t *);
// void ec_fsm_slave_scan_state_datalink(ec_fsm_slave_scan_t *);
// void ec_fsm_slave_scan_state_preop(ec_fsm_slave_scan_t *);
// void ec_fsm_slave_scan_state_sync(ec_fsm_slave_scan_t *);
// void ec_fsm_slave_scan_state_pdos(ec_fsm_slave_scan_t *);

void ec_fsm_i950_state_end(ec_fsm_i950_t *);
void ec_fsm_i950_state_error(ec_fsm_i950_t *);

/*****************************************************************************/

// ec_fsm_pdo_t *fsm_pdo /**< PDO configuration machine to use. */
/** Constructor.
 */
void ec_fsm_i950_init(
        ec_fsm_i950_t *fsm, /**< slave state machine. by luigi */
        ec_datagram_t *datagram /**< Datagram to use. */
        )
{
    fsm->datagram = datagram;
    fsm->state = NULL;
    // fsm->fsm_slave_config = fsm_slave_config;
    // fsm->fsm_pdo = fsm_pdo;

}

/*****************************************************************************/

/** Destructor.
 */
void ec_fsm_i950_clear(ec_fsm_i950_t *fsm /**< slave state machine */)
{
    // clear sub state machines
    // ec_fsm_sii_clear(&fsm->fsm_sii);
}

/*****************************************************************************/

/**
 * Start slave scan state machine.
 */

void ec_fsm_i950_start(
        ec_fsm_i950_t *fsm, /**< slave state machine */
        ec_slave_t *slave /**< slave to configure */
        )
{
    fsm->slave = slave;
    fsm->state = ec_fsm_i950_state_start;
}

/*****************************************************************************/

/**
   \return false, if state machine has terminated
*/

int ec_fsm_i950_running(const ec_fsm_i950_t *fsm /**< slave state machine */)
{
    return fsm->state != ec_fsm_i950_state_end
        && fsm->state != ec_fsm_i950_state_error;
}

/*****************************************************************************/

/**
   Executes the current state of the state machine.
   If the state machine's datagram is not sent or received yet, the execution
   of the state machine is delayed to the next cycle.
   \return false, if state machine has terminated
*/

int ec_fsm_i950_exec(ec_fsm_i950_t *fsm /**< slave state machine */)
{
    if (fsm->datagram->state == EC_DATAGRAM_SENT
        || fsm->datagram->state == EC_DATAGRAM_QUEUED) {
        // datagram was not sent or received yet.
        return ec_fsm_i950_running(fsm);
    }

    fsm->state(fsm);
    return ec_fsm_i950_running(fsm);
}

/*****************************************************************************/

/**
   \return true, if the state machine terminated gracefully
*/

int ec_fsm_i950_success(const ec_fsm_i950_t *fsm /**< slave state machine */)
{
    return fsm->state == ec_fsm_i950_state_end;
}

/******************************************************************************
 *  slave scan state machine
 *****************************************************************************/

/**
   Slave scan state: START.
   First state of the slave state machine. 
   set SM0,SM1 clear SM2,SM3
   * 
   * 
   * int ec_datagram_fprw(
   *     ec_datagram_t *datagram,     ...  EtherCAT datagram. 
   *     uint16_t configured_address,  ... Configured station address. 
   *     uint16_t mem_address,      ...   Physical memory address. 
   *     size_t data_size           ... Number of bytes to write. 
****/

void ec_fsm_i950_state_start(ec_fsm_i950_t *fsm )
{
    // set FMMU-2
    ec_datagram_fprd(fsm->datagram, fsm->slave->station_address, 0x0620, 16);
    // datagram 00 00 00 09 01 00 00 00
    EC_WRITE_U64(fsm->datagram->data, 0x0000000109000000);
    // datagram 0d 08 00 01 01 00 00 00
    EC_WRITE_U64(fsm->datagram->data+8, 0x000000010100080d);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_i950_state_fmmu2;
}


/*****************************************************************************/

/**
   Slave scan state: fmmu2
*/


void ec_fsm_i950_state_fmmu2(
        ec_fsm_i950_t *fsm 
        )
{
    ec_datagram_t *datagram = fsm->datagram;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
        return;

    if (datagram->state != EC_DATAGRAM_RECEIVED) {
        fsm->state = ec_fsm_i950_state_error;
        EC_SLAVE_ERR(fsm->slave,
                "Failed to receive station FMMU-2 datagram: ");
        ec_datagram_print_state(datagram);
        return;
    }

    if (datagram->working_counter != 1) {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_i950_state_error;
        EC_SLAVE_ERR(fsm->slave, "Failed to write FMMU-2: ");
        ec_datagram_print_wc_error(datagram);
        return;
    }

	fsm->state = ec_fsm_i950_state_end;		// nastavak u slave_scan
    
}

/*****************************************************************************/

/**
   nastavak ... enter SM2
*/


void ec_fsm_i950_enter(
        ec_fsm_i950_t *fsm, /**< slave state machine */
        ec_slave_t *slave /**< slave to configure */ 
        )
{

    // twincat procedure cont.
    
    EC_SLAVE_DBG(fsm->slave, 1, "SM2, SM3 by luigi .\n");
    // set SM2, SM3
    ec_datagram_fprd(fsm->datagram, fsm->slave->station_address, 0x0810, 16);
    // datagram 00 14 20 00 64 00 01 00
    EC_WRITE_U64(fsm->datagram->data, 0x0001006400201400);
    // datagram 00 17 20 00 20 00 01 00
    EC_WRITE_U64(fsm->datagram->data+8, 0x0001002000201700);
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_i950_state_sm2;
}


/*****************************************************************************/

/**
   Slave scan state: SM2
*/


void ec_fsm_i950_state_sm2(
        ec_fsm_i950_t *fsm 
        )
{
    ec_datagram_t *datagram = fsm->datagram;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
        return;

    if (datagram->state != EC_DATAGRAM_RECEIVED) {
        fsm->state = ec_fsm_i950_state_error;
        EC_SLAVE_ERR(fsm->slave,
                "Failed to receive station SM2 datagram: ");
        ec_datagram_print_state(datagram);
        return;
    }

    if (datagram->working_counter != 1) {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_i950_state_error;
        EC_SLAVE_ERR(fsm->slave, "Failed to write SM2: ");
        ec_datagram_print_wc_error(datagram);
        return;
    }

	EC_SLAVE_DBG(fsm->slave, 1, "FMMU-0, FMMU-1 by luigi .\n");

    // set FMMU-0, FMMU-1
    ec_datagram_fprd(fsm->datagram, fsm->slave->station_address, 0x0600, 32);
    // datagram 00 00 00 01 20 00 00 07
    EC_WRITE_U64(fsm->datagram->data, 0x0700002001000000);
    // datagram 00 14 00 02 01 00 00 00
    EC_WRITE_U64(fsm->datagram->data+8, 0x0000000102001400);
    // datagram 00 00 00 01 20 00 00 07
    EC_WRITE_U64(fsm->datagram->data+16, 0x0700002001000000);
    // datagram 00 17 00 01 01 00 00 00
    EC_WRITE_U64(fsm->datagram->data+24, 0x0000000101001700);
    
    fsm->retries = EC_FSM_RETRIES;
    fsm->state = ec_fsm_i950_state_sm3;

    
}

/*****************************************************************************/

/**
   Slave scan state: SM3 ... samo isprati i EXIT
*/


void ec_fsm_i950_state_sm3(
        ec_fsm_i950_t *fsm 
        )
{
    ec_datagram_t *datagram = fsm->datagram;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
        return;

    if (datagram->state != EC_DATAGRAM_RECEIVED) {
        fsm->state = ec_fsm_i950_state_error;
        EC_SLAVE_ERR(fsm->slave,
                "Failed to receive station FMMU0 datagram: ");
        ec_datagram_print_state(datagram);
        return;
    }

    if (datagram->working_counter != 1) {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_i950_state_error;
        EC_SLAVE_ERR(fsm->slave, "Failed to write FMMU0: ");
        ec_datagram_print_wc_error(datagram);
        return;
    }

    fsm->state = ec_fsm_i950_state_end;			// exit
    
}

/*****************************************************************************/

/*
void ec_fsm_i950_state_state(
        ec_fsm_i950_t *fsm 
        )
{
    ec_datagram_t *datagram = fsm->datagram;
    ec_slave_t *slave = fsm->slave;

    if (datagram->state == EC_DATAGRAM_TIMED_OUT && fsm->retries--)
        return;

    if (datagram->state != EC_DATAGRAM_RECEIVED) {
        fsm->state = ec_fsm_i950_state_error;
        EC_SLAVE_ERR(slave, "Failed to receive AL state datagram: ");
        ec_datagram_print_state(datagram);
        return;
    }

    if (datagram->working_counter != 1) {
        fsm->slave->error_flag = 1;
        fsm->state = ec_fsm_i950_state_error;
        EC_SLAVE_ERR(slave, "Failed to read AL state: ");
        ec_datagram_print_wc_error(datagram);
        return;
    }

    slave->current_state = EC_READ_U8(datagram->data);
    if (slave->current_state & EC_SLAVE_STATE_ACK_ERR) {
        char state_str[EC_STATE_STRING_SIZE];
        ec_state_string(slave->current_state, state_str, 0);
        EC_SLAVE_WARN(slave, "Slave has state error bit set (%s)!\n",
                state_str);
    }

	// TODO set state
}
*/





/******************************************************************************
 * Common state functions
 *****************************************************************************/

/** State: ERROR.
 */
void ec_fsm_i950_state_error(
        ec_fsm_i950_t *fsm /**< slave state machine */
        )
{
}

/*****************************************************************************/

/** State: END.
 */
void ec_fsm_i950_state_end(
        ec_fsm_i950_t *fsm /**< slave state machine */
        )
{
}

/*****************************************************************************/
