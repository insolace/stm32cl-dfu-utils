#ifndef _SAM7DFU_H
#define _SAM7DFU_H

int dfuload_do_upload(struct libusb_device_handle *usb_handle, int interface,
		      int xfer_size, const char *fname);
int dfuload_do_dnload(struct libusb_device_handle *usb_handle, int interface,
		      int xfer_size, const char *fname);

#endif
