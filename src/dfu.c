/*
 * low-level DFU message sending routines, originally taken from
 * $Id: dfu.c,v 1.3 2006/06/20 06:28:04 schmidtw Exp $
 * (part of dfu-programmer).
 *
 * (C) 2005-2006 Weston Schmidt <weston_schmidt@alumni.purdue.edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdio.h>
#include <libusb.h>
#include "dfu.h"

/* DFU commands */
#define DFU_DETACH      0
#define DFU_DNLOAD      1
#define DFU_UPLOAD      2
#define DFU_GETSTATUS   3
#define DFU_CLRSTATUS   4
#define DFU_GETSTATE    5
#define DFU_ABORT       6

#define INVALID_DFU_TIMEOUT -1

static int dfu_timeout = INVALID_DFU_TIMEOUT;
extern unsigned short transaction;

static int dfu_debug_level = 0;

void dfu_init( const int timeout )
{
    if( timeout > 0 ) {
        dfu_timeout = timeout;
    } else {
        if( 0 != dfu_debug_level )
            fprintf( stderr, "dfu_init: Invalid timeout value %d.\n", timeout );
    }
}

static int dfu_verify_init( const char *function )
{
    if( INVALID_DFU_TIMEOUT == dfu_timeout ) {
        if( 0 != dfu_debug_level )
            fprintf( stderr,
                     "%s: dfu system not property initialized.\n",
                     function );
        return -1;
    }

    return 0;
}

void dfu_debug( const int level )
{
    dfu_debug_level = level;
}


/*
 *  DFU_DETACH Request (DFU Spec 1.0, Section 5.1)
 *
 *  device    - the libusb_device_handle to communicate with
 *  interface - the interface to communicate with
 *  timeout   - the timeout in ms the USB device should wait for a pending
 *              USB reset before giving up and terminating the operation
 *
 *  returns 0 or < 0 on error
 */
int dfu_detach( struct libusb_device_handle *device,
                const unsigned short interface,
                const unsigned short timeout )
{
    if( 0 != dfu_verify_init(__FUNCTION__) )
        return -1;

    return libusb_control_transfer(device,
        /* bmRequestType */ LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
        /* bRequest      */ DFU_DETACH,
        /* wValue        */ timeout,
        /* wIndex        */ interface,
        /* Data          */ NULL,
        /* wLength       */ 0,
                            dfu_timeout);
}


/*
 *  DFU_DNLOAD Request (DFU Spec 1.0, Section 6.1.1)
 *
 *  device    - the libusb_device_handle to communicate with
 *  interface - the interface to communicate with
 *  length    - the total number of bytes to transfer to the USB
 *              device - must be less than wTransferSize
 *  data      - the data to transfer
 *
 *  returns the number of bytes written or < 0 on error
 */
int dfu_download(struct libusb_device_handle *device,
                 const unsigned short interface,
                 const unsigned short length,
                 char *data,
                 unsigned short transaction)
{
    int status;

    // Verify DFU initialization
    if (0 != dfu_verify_init(__FUNCTION__))
        return -1;

    /* Sanity checks */
    if ((0 != length) && (NULL == data)) {
        if (0 != dfu_debug_level)
            fprintf(stderr, "%s: data was NULL, but length != 0\n", __FUNCTION__);
        return -1;
    }

    if ((0 == length) && (NULL != data)) {
        if (0 != dfu_debug_level)
            fprintf(stderr, "%s: data was not NULL, but length == 0\n", __FUNCTION__);
        return -2;
    }

    // Perform DFU download via a control transfer
    status = libusb_control_transfer(device,
                                     /* bmRequestType */ LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
                                     /* bRequest      */ DFU_DNLOAD,
                                     /* wValue        */ transaction,
                                     /* wIndex        */ interface,
                                     /* Data          */ (unsigned char *)data,
                                     /* wLength       */ length,
                                     dfu_timeout);
    if (status < 0) {
        fprintf(stderr, "%s: libusb_control_transfer returned %d: %s\n",
                __FUNCTION__,
                status,
                libusb_strerror(status));
    }

    return status;
}


/*
 *  DFU_UPLOAD Request (DFU Spec 1.0, Section 6.2)
 *
 *  device    - the libusb_device_handle to communicate with
 *  interface - the interface to communicate with
 *  length    - the maximum number of bytes to receive from the USB
 *              device - must be less than wTransferSize
 *  data      - the buffer to put the received data in
 *
 *  returns the number of bytes received or < 0 on error
 */
int dfu_upload( struct libusb_device_handle *device,
                const unsigned short interface,
                const unsigned short length,
                char* data, unsigned short transaction )
{
    int status;

    if( 0 != dfu_verify_init(__FUNCTION__) )
        return -1;

    /* Sanity checks */
    if( (0 == length) || (NULL == data) ) {
        if( 0 != dfu_debug_level )
            fprintf( stderr,
                     "%s: data was NULL, or length is 0\n",
                     __FUNCTION__ );
        return -1;
    }

    status = libusb_control_transfer(device,
        /* bmRequestType */ LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
        /* bRequest      */ DFU_UPLOAD,
        /* wValue        */ transaction,
        /* wIndex        */ interface,
        /* Data          */ (unsigned char*)data,
        /* wLength       */ length,
                            dfu_timeout);
    if( status < 0 ) {
        fprintf( stderr, "%s: libusb_control_transfer returned %d: %s\n",
		 __FUNCTION__,
         status, libusb_error_name(status) );
    }

    return status;
}


/*
 *  DFU_GETSTATUS Request (DFU Spec 1.0, Section 6.1.2)
 *
 *  device    - the libusb_device_handle to communicate with
 *  interface - the interface to communicate with
 *  status    - the data structure to be populated with the results
 *
 *  return the number of bytes read in or < 0 on an error
 */
int dfu_get_status( struct libusb_device_handle *device,
                    const unsigned short interface,
                    struct dfu_status *status )
{
    char buffer[6];
    int result;

    if( 0 != dfu_verify_init(__FUNCTION__) )
        return -1;

    /* Initialize the status data structure */
    status->bStatus       = DFU_STATUS_ERROR_UNKNOWN;
    status->bwPollTimeout = 0;
    status->bState        = STATE_DFU_ERROR;
    status->iString       = 0;

    result = libusb_control_transfer(device,
        /* bmRequestType */ LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
        /* bRequest      */ DFU_GETSTATUS,
        /* wValue        */ 0,
        /* wIndex        */ interface,
        /* Data          */ (unsigned char*)buffer,
        /* wLength       */ 6,
                            dfu_timeout);

    if( 6 == result ) {
        status->bStatus = buffer[0];
        status->bwPollTimeout = ((0xff & buffer[3]) << 16) |
                                ((0xff & buffer[2]) << 8)  |
                                (0xff & buffer[1]);

        status->bState  = buffer[4];
        status->iString = buffer[5];
    }

    return result;
}


/*
 *  DFU_CLRSTATUS Request (DFU Spec 1.0, Section 6.1.3)
 *
 *  device    - the libusb_device_handle to communicate with
 *  interface - the interface to communicate with
 *
 *  return 0 or < 0 on an error
 */
int dfu_clear_status( struct libusb_device_handle *device,
                      const unsigned short interface )
{
    if( 0 != dfu_verify_init(__FUNCTION__) )
        return -1;

    return libusb_control_transfer(device,
        /* bmRequestType */ LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
        /* bRequest      */ DFU_CLRSTATUS,
        /* wValue        */ 0,
        /* wIndex        */ interface,
        /* Data          */ NULL,
        /* wLength       */ 0,
                            dfu_timeout);
}


/*
 *  DFU_GETSTATE Request (DFU Spec 1.0, Section 6.1.5)
 *
 *  device    - the libusb_device_handle to communicate with
 *  interface - the interface to communicate with
 *  length    - the maximum number of bytes to receive from the USB
 *              device - must be less than wTransferSize
 *  data      - the buffer to put the received data in
 *
 *  returns the state or < 0 on error
 */
int dfu_get_state( struct libusb_device_handle *device,
                   const unsigned short interface )
{
    int result;
    char buffer[1];

    if( 0 != dfu_verify_init(__FUNCTION__) )
        return -1;

    result = libusb_control_transfer(device,
          /* bmRequestType */ LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
          /* bRequest      */ DFU_GETSTATE,
          /* wValue        */ 0,
          /* wIndex        */ interface,
          /* Data          */ (unsigned char*)buffer,
          /* wLength       */ 1,
                              dfu_timeout);

    /* Return the error if there is one. */
    if( result < 1 ) {
        return result;
    }

    /* Return the state. */
    return buffer[0];
}


/*
 *  DFU_ABORT Request (DFU Spec 1.0, Section 6.1.4)
 *
 *  device    - the libusb_device_handle to communicate with
 *  interface - the interface to communicate with
 *
 *  returns 0 or < 0 on an error
 */
int dfu_abort( struct libusb_device_handle *device,
               const unsigned short interface )
{
    if( 0 != dfu_verify_init(__FUNCTION__) )
        return -1;

    return libusb_control_transfer(device,
        /* bmRequestType */ LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
        /* bRequest      */ DFU_ABORT,
        /* wValue        */ 0,
        /* wIndex        */ interface,
        /* Data          */ NULL,
        /* wLength       */ 0,
                            dfu_timeout);
}


char* dfu_state_to_string( int state )
{
    char *message = NULL;

    switch( state ) {
        case STATE_APP_IDLE:
            message = "appIDLE";
            break;
        case STATE_APP_DETACH:
            message = "appDETACH";
            break;
        case STATE_DFU_IDLE:
            message = "dfuIDLE";
            break;
        case STATE_DFU_DOWNLOAD_SYNC:
            message = "dfuDNLOAD-SYNC";
            break;
        case STATE_DFU_DOWNLOAD_BUSY:
            message = "dfuDNBUSY";
            break;
        case STATE_DFU_DOWNLOAD_IDLE:
            message = "dfuDNLOAD-IDLE";
            break;
        case STATE_DFU_MANIFEST_SYNC:
            message = "dfuMANIFEST-SYNC";
            break;
        case STATE_DFU_MANIFEST:
            message = "dfuMANIFEST";
            break;
        case STATE_DFU_MANIFEST_WAIT_RESET:
            message = "dfuMANIFEST-WAIT-RESET";
            break;
        case STATE_DFU_UPLOAD_IDLE:
            message = "dfuUPLOAD-IDLE";
            break;
        case STATE_DFU_ERROR:
            message = "dfuERROR";
            break;
    }

    return message;
}

/* Chapter 6.1.2 */
static const char *dfu_status_names[] = {
	[DFU_STATUS_OK]			= "No error condition is present",
	[DFU_STATUS_errTARGET]		= 
		"File is not targeted for use by this device",
	[DFU_STATUS_errFILE]		=
		"File is for this device but fails some vendor-specific test",
	[DFU_STATUS_errWRITE]		=
		"Device is unable to write memory",
	[DFU_STATUS_errERASE]		=
		"Memory erase function failed",
	[DFU_STATUS_errCHECK_ERASED]	=
		"Memory erase check failed",
	[DFU_STATUS_errPROG]		=
		"Program memory function failed",
	[DFU_STATUS_errVERIFY]		=
		"Programmed memory failed verification",
	[DFU_STATUS_errADDRESS]		=
		"Cannot program memory due to received address that is out of range",
	[DFU_STATUS_errNOTDONE]		=
		"Received DFU_DNLOAD with wLength = 0, but device does not think that it has all data yet",
	[DFU_STATUS_errFIRMWARE]	=
		"Device's firmware is corrupt. It cannot return to run-time (non-DFU) operations",
	[DFU_STATUS_errVENDOR]		=
		"iString indicates a vendor specific error",
	[DFU_STATUS_errUSBR]		=
		"Device detected unexpected USB reset signalling",
	[DFU_STATUS_errPOR]		=
		"Device detected unexpected power on reset",
	[DFU_STATUS_errUNKNOWN]		=
		"Something went wrong, but the device does not know what it was",
	[DFU_STATUS_errSTALLEDPKT]	=
		"Device stalled an unexpected request",
};


const char *dfu_status_to_string(int status)
{
	if (status > DFU_STATUS_errSTALLEDPKT)
		return "INVALID";
	return dfu_status_names[status];
}

