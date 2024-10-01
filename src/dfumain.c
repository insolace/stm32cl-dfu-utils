/*
 * dfu-util
 *
 * (C) 2007-2008 by OpenMoko, Inc.
 * Written by Harald Welte <laforge@openmoko.org>
 *
 * Based on existing code of dfu-programmer-0.4
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

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <libusb.h>
#include <errno.h>

#include "dfumain.h"
#include "dfu.h"
#include "usb_dfu.h"
#include "dfu_load.h"
#include "dfuse.h"
#include "quirks.h"
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_USBPATH_H
#include <usbpath.h>
#endif

/* define a portable function for reading a 16bit little-endian word */
unsigned short get_int16_le(const void *p)
{
    const unsigned char *cp = p;

    return ( cp[0] ) | ( ((unsigned short)cp[1]) << 8 );
}

int debug;
//static int verbose = 0;

#define DFU_IFF_DFU		0x0001	/* DFU Mode, (not Runtime) */
#define DFU_IFF_VENDOR		0x0100
#define DFU_IFF_PRODUCT		0x0200
#define DFU_IFF_CONFIG		0x0400
#define DFU_IFF_IFACE		0x0800
#define DFU_IFF_ALT		0x1000
#define DFU_IFF_DEVNUM		0x2000
#define DFU_IFF_PATH		0x4000

struct usb_vendprod {
	u_int16_t vendor;
	u_int16_t product;
};

struct dfu_if {
	u_int16_t vendor;
	u_int16_t product;
	u_int8_t configuration;
	u_int8_t interface;
	u_int8_t altsetting;
	int bus;
	u_int8_t devnum;
	const char *path;
	unsigned int flags;
    struct libusb_device *dev;

    struct libusb_device_handle *dev_handle;
};

void warningBlock(); // used to supress errors for unused functions

static int _get_first_cb(struct dfu_if *dif, void *v)
{
	struct dfu_if *v_dif = v;

    memcpy(v_dif, dif, sizeof(*v_dif)-sizeof(struct libusb_device_handle *));

	/* return a value that makes find_dfu_if return immediately */
	return 1;
}

/* Find a DFU interface (and altsetting) in a given device */
static int find_dfu_if(struct libusb_device *dev, int (*handler)(struct dfu_if *, void *), void *v)
{
    struct libusb_config_descriptor *cfg;
    const struct libusb_interface_descriptor *intf;
    struct dfu_if _dif, *dfu_if = &_dif;
    struct libusb_device_descriptor desc;
    int cfg_idx, intf_idx, alt_idx;
    int rc;

    memset(dfu_if, 0, sizeof(*dfu_if));

    // Get the device descriptor
    rc = libusb_get_device_descriptor(dev, &desc);
    if (rc != LIBUSB_SUCCESS) {
        fprintf(stderr, "Failed to get device descriptor: %s\n", libusb_error_name(rc));
        return rc;
    }

    // Loop through each configuration descriptor
    for (cfg_idx = 0; cfg_idx < desc.bNumConfigurations; cfg_idx++) {
        rc = libusb_get_config_descriptor(dev, cfg_idx, &cfg);
        if (rc != LIBUSB_SUCCESS) {
            fprintf(stderr, "Failed to get configuration descriptor: %s\n", libusb_error_name(rc));
            continue;
        }

        // Loop through each interface in the configuration
        for (intf_idx = 0; intf_idx < cfg->bNumInterfaces; intf_idx++) {
            const struct libusb_interface *uif = &cfg->interface[intf_idx];

            // Loop through each alternate setting in the interface
            for (alt_idx = 0; alt_idx < uif->num_altsetting; alt_idx++) {
                intf = &uif->altsetting[alt_idx];

                // Check if this is a DFU interface
                if (intf->bInterfaceClass == 0xFE && intf->bInterfaceSubClass == 1) {
                    dfu_if->dev = dev;
                    dfu_if->vendor = desc.idVendor;
                    dfu_if->product = desc.idProduct;
                    dfu_if->configuration = cfg_idx;
                    dfu_if->interface = intf->bInterfaceNumber;
                    dfu_if->altsetting = intf->bAlternateSetting;

                    // Set the DFU flag based on the interface protocol
                    if (intf->bInterfaceProtocol == 2) {
                        dfu_if->flags |= DFU_IFF_DFU;
                    } else {
                        dfu_if->flags &= ~DFU_IFF_DFU;
                    }

                    // Call the handler function if provided
                    if (!handler) {
                        libusb_free_config_descriptor(cfg);
                        return 1;
                    }

                    rc = handler(dfu_if, v);
                    if (rc != 0) {
                        libusb_free_config_descriptor(cfg);
                        return rc;
                    }
                }
            }
        }

        // Free the configuration descriptor when done
        libusb_free_config_descriptor(cfg);
    }

    return 0;
}

static int get_first_dfu_if(struct dfu_if *dif)
{
    // Pass the device (libusb_device*) and call find_dfu_if, passing the callback
    return find_dfu_if(dif->dev, _get_first_cb, (void *)dif);
}

#define MAX_STR_LEN 64

static int print_dfu_if(struct dfu_if *dfu_if, void *v)
{
    (void)v; // unused
    struct libusb_device *dev = dfu_if->dev;
    int if_name_str_idx;
    char name[MAX_STR_LEN+1] = "UNDEFINED";
    struct libusb_device_handle *dev_handle = NULL;
    struct libusb_config_descriptor *config = NULL;
    struct libusb_device_descriptor desc;

    int ret = libusb_get_device_descriptor(dev, &desc);
    if (ret != LIBUSB_SUCCESS) {
        fprintf(stderr, "Error getting device descriptor: %s\n", libusb_error_name(ret));
        return -1;
    }

    ret = libusb_get_active_config_descriptor(dev, &config);
    if (ret != LIBUSB_SUCCESS || !config) {
        fprintf(stderr, "Error getting config descriptor: %s\n", libusb_error_name(ret));
        return -1;
    }

    if_name_str_idx = config->interface[dfu_if->interface]
                          .altsetting[dfu_if->altsetting].iInterface;

    if (if_name_str_idx) {
        ret = libusb_open(dev, &dev_handle);
        if (ret == LIBUSB_SUCCESS && dev_handle) {
            ret = libusb_get_string_descriptor_ascii(dev_handle, if_name_str_idx, (unsigned char *)name, MAX_STR_LEN);
            if (ret < 0) {
                fprintf(stderr, "Error getting string descriptor: %s\n", libusb_error_name(ret));
                strncpy(name, "UNDEFINED", MAX_STR_LEN);
            }
        } else {
            fprintf(stderr, "Error opening device: %s\n", libusb_error_name(ret));
        }
    }

    printf("Found %s: [0x%04x:0x%04x] devnum=%u, cfg=%u, intf=%u, "
           "alt=%u, name=\"%s\"\n",
           dfu_if->flags & DFU_IFF_DFU ? "DFU" : "Runtime",
           desc.idVendor, desc.idProduct,
           libusb_get_bus_number(dev), dfu_if->configuration, dfu_if->interface,
           dfu_if->altsetting, name);

    if (dev_handle) {
        libusb_close(dev_handle);
    }

    libusb_free_config_descriptor(config);
    return 0;
}

static int alt_by_name(struct dfu_if *dfu_if, void *v)
{
    struct libusb_device *dev = dfu_if->dev;
    int if_name_str_idx;
    char name[MAX_STR_LEN+1] = "UNDEFINED";
    libusb_device_handle *dev_handle = dfu_if->dev_handle;

    // Get the interface string index
    struct libusb_config_descriptor *config;
    int result = libusb_get_config_descriptor(dev, dfu_if->configuration, &config);
    if (result < 0 || !config) {
        return 0;
    }

    if_name_str_idx = config->interface[dfu_if->interface]
                          .altsetting[dfu_if->altsetting].iInterface;
    if (!if_name_str_idx) {
        libusb_free_config_descriptor(config);
        return 0;
    }

    // Open the device handle if it isn't already opened
    if (!dev_handle) {
        result = libusb_open(dev, &dev_handle);
        if (result < 0 || !dev_handle) {
            libusb_free_config_descriptor(config);
            return 0;
        }
        dfu_if->dev_handle = dev_handle;  // Save the device handle
    }

    // Get the interface name string
    result = libusb_get_string_descriptor_ascii(dev_handle, if_name_str_idx,
                                                (unsigned char*)name, MAX_STR_LEN);
    libusb_free_config_descriptor(config);

    if (result < 0) {
        return 0;  // Could not retrieve the string
    }

    // Compare the interface name with the provided name
    if (strcmp(name, (char*)v) != 0) {
        return 0;  // Name doesn't match
    }

    // Return altsetting+1 to indicate the correct interface
    return dfu_if->altsetting + 1;
}

static int _count_cb(struct dfu_if *dif, void *v)
{
    (void)dif; // unused
	int *count = v;

	(*count)++;

	return 0;
}

/* Count DFU interfaces within a single device */
static int count_dfu_interfaces(struct libusb_device *dev)
{
	int num_found = 0;

	find_dfu_if(dev, &_count_cb, (void *) &num_found);

	return num_found;
}


/* Iterate over all matching DFU capable devices within system */
static int iterate_dfu_devices(struct dfu_if *dif,
    int (*action)(struct libusb_device *dev, void *user), void *user)
{
    libusb_device **list;
    ssize_t cnt;
    int retval = 0;

    // Get a list of all connected USB devices
    cnt = libusb_get_device_list(NULL, &list);
    if (cnt < 0) {
        fprintf(stderr, "Error getting USB device list: %s\n", libusb_error_name(cnt));
        return (int)cnt;
    }

    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device *dev = list[i];
        struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(dev, &desc);
        if (r < 0) {
            fprintf(stderr, "Error getting device descriptor: %s\n", libusb_error_name(r));
            continue;
        }

        // If a `dif` is provided, check for matching Vendor/Product or device number
        if (dif && (dif->flags & (DFU_IFF_VENDOR | DFU_IFF_PRODUCT)) &&
            (desc.idVendor != dif->vendor || desc.idProduct != dif->product)) {
            continue;
        }

        if (dif && (dif->flags & DFU_IFF_DEVNUM) &&
            (libusb_get_bus_number(dev) != dif->bus ||
             libusb_get_device_address(dev) != dif->devnum)) {
            continue;
        }

        // Check if the device contains any DFU interfaces
        if (!count_dfu_interfaces(dev))
            continue;

        // Perform the action on the device
        retval = action(dev, user);
        if (retval)
            break;
    }

    // Free the device list
    libusb_free_device_list(list, 1);
    return retval;
}


static int found_dfu_device(struct libusb_device *dev, void *user)
{
	struct dfu_if *dif = user;

	dif->dev = dev;
	return 1;
}


/* Find the first DFU-capable device, save it in dfu_if->dev */
static int get_first_dfu_device(struct dfu_if *dif)
{
	return iterate_dfu_devices(dif, found_dfu_device, dif);
}


static int count_one_dfu_device(struct libusb_device *dev, void *user)
{
    (void)dev; // unused
	int *num = user;

	(*num)++;
	return 0;
}


/* Count DFU capable devices within system */
static int count_dfu_devices(struct dfu_if *dif)
{
	int num_found = 0;

	iterate_dfu_devices(dif, count_one_dfu_device, &num_found);
	return num_found;
}


static int list_dfu_interfaces(void)
{
    libusb_device **devs;
    libusb_device *dev;
    ssize_t cnt;
    ssize_t i;

    // Initialize libusb context if not already done
    libusb_context *ctx = NULL;
    int result = libusb_init(&ctx);
    if (result < 0) {
        fprintf(stderr, "Error initializing libusb: %s\n", libusb_error_name(result));
        return result;
    }

    // Get the list of USB devices
    cnt = libusb_get_device_list(ctx, &devs);
    if (cnt < 0) {
        fprintf(stderr, "Error getting device list: %s\n", libusb_error_name(cnt));
        libusb_exit(ctx);
        return (int)cnt;
    }

    // Loop through all the devices and look for DFU interfaces
    for (i = 0; i < cnt; i++) {
        dev = devs[i];
        find_dfu_if(dev, &print_dfu_if, NULL);
    }

    // Free the device list and deinitialize libusb
    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);

    return 0;
}

static int parse_vendprod(struct usb_vendprod *vp, const char *str)
{
	unsigned long vend, prod;
	const char *colon;

	colon = strchr(str, ':');
	if (!colon || strlen(colon) < 2)
		return -EINVAL;

	vend = strtoul(str, NULL, 16);
	prod = strtoul(colon+1, NULL, 16);

	if (vend > 0xffff || prod > 0xffff)
		return -EINVAL;

	vp->vendor = vend;
	vp->product = prod;

	return 0;
}


#ifdef HAVE_USBPATH_H

static int resolve_device_path(struct dfu_if *dif)
{
	int res;

	res = usb_path2devnum(dif->path);
	if (res < 0)
		return -EINVAL;
	if (!res)
		return 0;

	dif->bus = atoi(dif->path);
	dif->devnum = res;
	dif->flags |= DFU_IFF_DEVNUM;
	return res;
}

#else /* HAVE_USBPATH_H */

static int resolve_device_path(struct dfu_if *dif)
{
    (void)dif; // unused
	fprintf(stderr,
	    "USB device paths are not supported by this dfu-util.\n");
    return 1;
}

#endif /* !HAVE_USBPATH_H */

/* Look for descriptor in the configuration descriptor output */
static int usb_get_extra_descriptor(libusb_device_handle *udev, unsigned char type,
            unsigned char index, void *resbuf, int size)
{
    unsigned char *cbuf;
    int desclen, conflen, smallest;
    int ret;
    int p = 0;
    int foundlen = 0;
    struct libusb_config_descriptor *config;

    // Get the active configuration descriptor
    ret = libusb_get_active_config_descriptor(libusb_get_device(udev), &config);
    if (ret < 0) {
        fprintf(stderr, "Error getting active config descriptor: %s\n", libusb_error_name(ret));
        return ret;
    }

    conflen = config->wTotalLength;
    cbuf = malloc(conflen);

    // Get the configuration descriptor
    ret = libusb_get_descriptor(udev, LIBUSB_DT_CONFIG, index, cbuf, conflen);
    if (ret < conflen) {
        fprintf(stderr, "Warning: failed to retrieve complete configuration descriptor\n");
        conflen = ret;
    }
    while (p + 1 < conflen) {
        desclen = (int) cbuf[p];
        if (cbuf[p + 1] == type) {
            smallest = desclen < size ? desclen : size;
            if (resbuf != NULL) {
                memcpy(resbuf, &cbuf[p], smallest);
            } else {
                fprintf(stderr, "Error: NULL pointer passed as resbuf.\n");
            }
            foundlen = smallest;
            break;
        }
        p += desclen;
    }
    free(cbuf);
    libusb_free_config_descriptor(config);

//    if (foundlen > 1)
//        return foundlen;

    // Fallback to libusb_control_transfer if not found in the configuration descriptor
    return libusb_control_transfer(udev,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_DEVICE,
        LIBUSB_REQUEST_GET_DESCRIPTOR, (type << 8) | index, 0, resbuf, size, 1000);
}

static void help(void)
{
	printf("Usage: dfu-util [options] ...\n"
		"  -h --help\t\t\tPrint this help message\n"
		"  -V --version\t\t\tPrint the version number\n"
		"  -v --verbose\t\t\tPrint verbose debug statements\n"
		"  -l --list\t\t\tList the currently attached DFU capable USB devices\n"
		"  -d --device vendor:product\tSpecify Vendor/Product ID of DFU device\n"
		"  -p --path bus-port. ... .port\tSpecify path to DFU device\n"
		"  -c --cfg config_nr\t\tSpecify the Configuration of DFU device\n"
		"  -i --intf intf_nr\t\tSpecify the DFU Interface number\n"
		"  -a --alt alt\t\t\tSpecify the Altsetting of the DFU Interface\n"
		"\t\t\t\tby name or by number\n"
		"  -t --transfer-size\t\tSpecify the number of bytes per USB Transfer\n"
		"  -U --upload file\t\tRead firmware from device into <file>\n"
		"  -D --download file\t\tWrite firmware from <file> into device\n"
		"  -R --reset\t\t\tIssue USB Reset signalling once we're finished\n"
		"  -s --dfuse address|default\tST DfuSe mode, specify address to download\n"
		"\t\t\t\traw file instead of DfuSe file\n"
		);
}

static void print_version(void)
{
	printf("dfu-util version %s\n", VERSION);
}

static struct option opts[] = {
	{ "help", 0, 0, 'h' },
	{ "version", 0, 0, 'V' },
	{ "verbose", 0, 0, 'v' },
	{ "list", 0, 0, 'l' },
	{ "device", 1, 0, 'd' },
	{ "path", 1, 0, 'p' },
	{ "configuration", 1, 0, 'c' },
	{ "cfg", 1, 0, 'c' },
	{ "interface", 1, 0, 'i' },
	{ "intf", 1, 0, 'i' },
	{ "altsetting", 1, 0, 'a' },
	{ "alt", 1, 0, 'a' },
	{ "transfer-size", 1, 0, 't' },
	{ "upload", 1, 0, 'U' },
	{ "download", 1, 0, 'D' },
	{ "reset", 0, 0, 'R' },
	{ "dfuse", 1, 0, 's' },
};

enum mode {
	MODE_NONE,
	MODE_UPLOAD,
	MODE_DOWNLOAD,
};

int dfumain(int argc, char **argv)
{
    libusb_context *ctx = NULL;
    libusb_device **devs = NULL;
    //libusb_device_handle *dev_handle = NULL;
    struct dfu_if _rt_dif, _dif, *dif = &_dif;
    struct dfu_status status;
    struct usb_dfu_func_descriptor func_dfu;
    char *filename = NULL;
    char *alt_name = NULL;
    int final_reset = 0;
    int dfuse = 0;
    unsigned int dfuse_address = 0;
    unsigned int transfer_size = 0;
    unsigned int default_transfer_size = 1024;
    unsigned int host_page_size; // = getpagesize();
    int ret = 0;
    ssize_t num_devs;
    int num_ifs;
    enum mode mode = MODE_NONE;

    struct usb_vendprod vendprod = {0};
    char *end;

	printf("dfu-util - (C) 2005-2008 by Weston Schmidt, Harald Welte and OpenMoko Inc.\n"
	       "           (C) 2010 Tormod Volden (experimental DfuSe support)\n"
	       "This program is Free Software and has ABSOLUTELY NO WARRANTY\n\n");

    printf("dfu-util - Using libusb-1.0\n\n");

	host_page_size = getpagesize();
	memset(dif, 0, sizeof(*dif));

    // Initialize libusb context
    if (libusb_init(&ctx) < 0) {
        fprintf(stderr, "libusb initialization failed\n");
        return -1;
    }

    // Device list fetch
    num_devs = libusb_get_device_list(ctx, &devs);
    if (num_devs < 0) {
        fprintf(stderr, "Error getting device list\n");
        libusb_exit(ctx);
        return -1;
    }

    // Argument parsing (preserve original behavior for now)
    while (1) {
        int c, option_index = 0;
        c = getopt_long(argc, argv, "hVvld:p:c:i:a:t:U:D:Rs:", opts, &option_index);
        if (c == -1)
            break;

        switch (c) {
        case 'h':
            help();
            return 0;
        case 'V':
            print_version();
            return 0;
        case 'v':
            libusb_set_debug(ctx, LIBUSB_LOG_LEVEL_DEBUG);
            break;
        case 'l':
            list_dfu_interfaces();
            return 0;
        case 'd':
            if (parse_vendprod(&vendprod, optarg) != 0)
            {
                fprintf(stderr, "Error: Unable to parse vendor:product from %s\n", optarg);
                    return 2;  // Or handle the error as appropriate
            }
            dif->vendor = vendprod.vendor;
            dif->product = vendprod.product;
            dif->flags |= (DFU_IFF_VENDOR | DFU_IFF_PRODUCT);
            break;
        case 'p':
            dif->path = optarg;
            dif->flags |= DFU_IFF_PATH;
            ret = resolve_device_path(dif);
            if (ret < 0 || !ret) {
                fprintf(stderr, "Cannot find device at path %s\n", optarg);
                return 1;
            }
            break;
        case 'c':
            dif->configuration = atoi(optarg);
            dif->flags |= DFU_IFF_CONFIG;
            break;
        case 'i':
            dif->interface = atoi(optarg);
            dif->flags |= DFU_IFF_IFACE;
            break;
        case 'a':
            dif->altsetting = strtoul(optarg, &end, 0);
            if (*end)
                alt_name = optarg;
            dif->flags |= DFU_IFF_ALT;
            break;
        case 't':
            transfer_size = atoi(optarg);
            break;
        case 'U':
            mode = MODE_UPLOAD;
            filename = optarg;
            break;
        case 'D':
            mode = MODE_DOWNLOAD;
            filename = optarg;
            break;
        case 'R':
            final_reset = 1;
            break;
        case 's':
            dfuse = 1;
            dfuse_address = strtol(optarg, &end, 0);
            if (!dfuse_address || (*end)) {
                fprintf(stderr, "Invalid dfuse address: %s\n", optarg);
                return 2;
            }
            break;
        default:
            help();
            return 2;
        }
    }

    if (mode == MODE_NONE || !filename) {
        fprintf(stderr, "You need to specify one of -D or -U and a filename\n");
        help();
        return 2;
    }
#ifdef DEBUG_DRY
    goto skip_setup;
#endif

    // Initialize the DFU context (timeout in milliseconds)
    dfu_init(5000);

    // Count available DFU devices
    num_devs = count_dfu_devices(dif);
    if (num_devs == 0) {
        fprintf(stderr, "No DFU capable USB device found\n");
        return 1;
    } else if (num_devs > 1) {
        // More than one DFU device is problematic due to resetting during DFU
        fprintf(stderr, "More than one DFU capable USB device found, "
                "please try `--list` and disconnect all but one device\n");
        return 3;
    }

    // Open the first DFU device
    if (!get_first_dfu_device(dif)) {
        return 3;
    }

    printf("Opening DFU USB device...\n");
    // Open the device using libusb-1.0
    ret = libusb_open(dif->dev, &dif->dev_handle);
    if (ret < 0) {
        fprintf(stderr, "Cannot open device: %s\n", libusb_error_name(ret));
        return 1;
    }

    // Find the first DFU interface
    memcpy(&_rt_dif, dif, sizeof(_rt_dif));
    if (!get_first_dfu_if(&_rt_dif)) {
        return 1;
    }

    printf("ID %04x:%04x\n", _rt_dif.vendor, _rt_dif.product);

    // Handle quirks for this particular device
    set_quirks(_rt_dif.vendor, _rt_dif.product);

    if (!(_rt_dif.flags & DFU_IFF_DFU)) {
        // In Runtime mode, the DFU interface will behave differently
        printf("Claiming USB DFU Runtime Interface...\n");

        // Claim the interface with libusb
        ret = libusb_claim_interface(_rt_dif.dev_handle, _rt_dif.interface);
        if (ret < 0) {
            fprintf(stderr, "Cannot claim interface %d: %s\n", _rt_dif.interface, libusb_error_name(ret));
            return 1;
        }

        // Set the alternate interface if needed
        ret = libusb_set_interface_alt_setting(_rt_dif.dev_handle, _rt_dif.interface, 0);
        if (ret < 0) {
            fprintf(stderr, "Cannot set alt interface: %s\n", libusb_error_name(ret));
            return 1;
        }

        // Retrieve the device status
        printf("Determining device status...\n");
        ret = dfu_get_status(_rt_dif.dev_handle, _rt_dif.interface, &status);
        if (ret < 0) {
            fprintf(stderr, "Error getting DFU status: %s\n", libusb_error_name(ret));
            return 1;
        }
        printf("State = %s, Status = %d\n", dfu_state_to_string(status.bState), status.bStatus);

        if (!(quirks & QUIRK_POLLTIMEOUT)) {
            usleep(status.bwPollTimeout * 1000);
        }

        // Handle different DFU states
        switch (status.bState) {
        case DFU_STATE_appIDLE:
        case DFU_STATE_appDETACH:
            printf("Device in Runtime Mode, sending DFU detach request...\n");

            // Send the detach request
            if (dfu_detach(_rt_dif.dev_handle, _rt_dif.interface, 1000) < 0) {
                fprintf(stderr, "Error detaching: %s\n", libusb_error_name(dfu_detach(_rt_dif.dev_handle, _rt_dif.interface, 1000)));
                return 1;
            }

            // Reset the USB connection
            printf("Resetting USB...\n");
            ret = libusb_reset_device(_rt_dif.dev_handle);
            if (ret < 0 && ret != LIBUSB_ERROR_NO_DEVICE) {
                fprintf(stderr, "Error resetting after detach: %s\n", libusb_error_name(ret));
            }
            sleep(2); // Allow the device to reset
            break;

        case DFU_STATE_dfuERROR:
            printf("Device in dfuERROR state, clearing status...\n");

            // Clear the DFU error state
            if (dfu_clear_status(_rt_dif.dev_handle, _rt_dif.interface) < 0) {
                fprintf(stderr, "Error clearing DFU status: %s\n", libusb_error_name(dfu_clear_status(_rt_dif.dev_handle, _rt_dif.interface)));
                return 1;
            }
            break;

        default:
            fprintf(stderr, "WARNING: Device already in DFU mode.\n");
            goto dfustate; // Skip reset and continue
        }

        // Re-scan the bus to find the device again
        libusb_device **device_list;
        ssize_t count = libusb_get_device_list(ctx, &device_list);
        if (count < 2) {
            printf("Not enough device changes found!\n");
        }

        // Resolve device path after reset
        if (dif->flags & DFU_IFF_PATH) {
            ret = resolve_device_path(dif);
            if (ret < 0) {
                fprintf(stderr, "Internal error: cannot re-parse `%s`\n", dif->path);
                abort();
            }
            if (!ret) {
                fprintf(stderr, "Cannot resolve path after RESET\n");
                return 1;
            }
        }

        // Check if the device is still present
        num_devs = count_dfu_devices(dif);
        if (num_devs == 0) {
            fprintf(stderr, "Lost device after RESET\n");
            return 1;
        } else if (num_devs > 1) {
            fprintf(stderr, "More than one DFU capable USB device found. Please disconnect all but one.\n");
            return 1;
        }

        // Open the device again
        if (!get_first_dfu_device(dif)) {
            return 3;
        }

        printf("Re-opening USB Device...\n");
        ret = libusb_open(dif->dev, &dif->dev_handle);
        if (ret < 0) {
            fprintf(stderr, "Cannot open device: %s\n", libusb_error_name(ret));
            return 1;
        }
    } else {
        // The device is already in DFU mode, no detach/reset needed
        printf("Device already in DFU mode.\n");
    }

dfustate:
    if (alt_name) {
        int n;

        // Find the alternate setting by name
        n = find_dfu_if(dif->dev, &alt_by_name, alt_name);
        if (!n) {
            fprintf(stderr, "No such Alternate Setting: \"%s\"\n", alt_name);
            return 1;
        }
        if (n < 0) {
            fprintf(stderr, "Error %d in name lookup\n", n);
            return 1;
        }
        dif->altsetting = n - 1; // Adjust altsetting index
    }

    print_dfu_if(dif, NULL); // Print the DFU interface information

    // Count the number of DFU interfaces
    num_ifs = count_dfu_interfaces(dif->dev);
    if (num_ifs < 0) {
        fprintf(stderr, "No DFU Interface after RESET?!?\n");
        return 1;
    } else if (num_ifs == 1) {
        if (!get_first_dfu_if(dif)) {
            fprintf(stderr, "Can't find the single available DFU IF\n");
            return 1;
        }
    } else if (num_ifs > 1 && !(dif->flags & (DFU_IFF_IFACE | DFU_IFF_ALT))) {
        fprintf(stderr, "We have %u DFU Interfaces/Altsettings, "
                "you have to specify one via --intf / --alt options\n", num_ifs);
        return 1;
    }

#if 0
    // Optional: Setting configuration (commented out in original code)
    printf("Setting Configuration %u...\n", dif->configuration);
    if (libusb_set_configuration(dif->dev_handle, dif->configuration) < 0) {
        fprintf(stderr, "Cannot set configuration: %s\n", libusb_error_name(ret));
        return 1;
    }
#endif

    // Claim the DFU interface using libusb-1.0
    printf("Claiming USB DFU Interface...\n");
    ret = libusb_claim_interface(dif->dev_handle, dif->interface);
    if (ret < 0) {
        fprintf(stderr, "Cannot claim interface: %s\n", libusb_error_name(ret));
        return 1;
    }

    // Set the alternate setting for the DFU interface
    printf("Setting Alternate Setting #%d ...\n", dif->altsetting);
    ret = libusb_set_interface_alt_setting(dif->dev_handle, dif->interface, dif->altsetting);
    if (ret < 0) {
        fprintf(stderr, "Cannot set alternate interface: %s\n", libusb_error_name(ret));
        return 1;
    }

status_again:
    printf("Determining device status: ");
    if (dfu_get_status(dif->dev_handle, dif->interface, &status) < 0) {
        fprintf(stderr, "can't detach: %s\n", libusb_strerror(libusb_detach_kernel_driver(dif->dev_handle, dif->interface)));
        return 1;
    }
    printf("state = %s, status = %d\n",
           dfu_state_to_string(status.bState), status.bStatus);

    // Apply polling timeout if necessary
    if (!(quirks & QUIRK_POLLTIMEOUT)) {
        usleep(status.bwPollTimeout * 1000);
    }

    // Check device state
    switch (status.bState) {
    case DFU_STATE_appIDLE:
    case DFU_STATE_appDETACH:
        fprintf(stderr, "Device still in Runtime Mode!\n");
        return 1;
        break;
    case DFU_STATE_dfuERROR:
        printf("dfuERROR, clearing status\n");
        if (dfu_clear_status(dif->dev_handle, dif->interface) < 0) {
            fprintf(stderr, "error clear_status: %s\n", libusb_error_name(ret));
            return 1;
        }
        goto status_again;
        break;
    case DFU_STATE_dfuDNLOAD_IDLE:
    case DFU_STATE_dfuUPLOAD_IDLE:
        printf("Aborting previous incomplete transfer\n");
        if (dfu_abort(dif->dev_handle, dif->interface) < 0) {
            fprintf(stderr, "Can't send DFU_ABORT: %s\n", libusb_error_name(ret));
            return 1;
        }
        goto status_again;
        break;
    case DFU_STATE_dfuIDLE:
        printf("dfuIDLE, continuing\n");
        break;
    case 99:
        warningBlock();
    }


    // Retrieve transfer size from functional descriptor
    if (!transfer_size) {
        ret = libusb_control_transfer(dif->dev_handle, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_INTERFACE, LIBUSB_REQUEST_GET_DESCRIPTOR, USB_DT_DFU, dif->interface, (unsigned char *)&func_dfu, sizeof(func_dfu), 1000);
        if (ret < 0) {
            fprintf(stderr, "Error obtaining DFU functional descriptor: %s\n", libusb_error_name(ret));
        } else {
            transfer_size = get_int16_le(&func_dfu.wTransferSize);
            printf("Device returned transfer size %i\n", transfer_size);
        }
    }

    // Handle transfer size and ensure it fits within host and device limitations
    if (!transfer_size) {
        transfer_size = default_transfer_size;
        printf("Warning: Trying default transfer size %i\n", transfer_size);
    }
    if (transfer_size > host_page_size) {
        transfer_size = host_page_size;
        printf("Limited transfer size to %i\n", transfer_size);
    }

    struct libusb_device_descriptor desc;
    ret = libusb_get_device_descriptor(dif->dev, &desc);
    if (ret < 0) {
        fprintf(stderr, "Failed to get device descriptor: %s\n", libusb_error_name(ret));
        return 1;
    }

    // Now, you can safely access bMaxPacketSize0
    if (transfer_size < desc.bMaxPacketSize0) {
        transfer_size = desc.bMaxPacketSize0;
        printf("Adjusted transfer size to %i\n", transfer_size);
    }

    // Check if device status is OK, retry if necessary
    if (DFU_STATUS_OK != status.bStatus) {
        printf("WARNING: DFU Status: '%s'\n", dfu_status_to_string(status.bStatus));
        dfu_clear_status(dif->dev_handle, dif->interface);
        dfu_get_status(dif->dev_handle, dif->interface, &status);

        if (DFU_STATUS_OK != status.bStatus) {
            fprintf(stderr, "Error: %d\n", status.bStatus);
            return 1;
        }
        if (!(quirks & QUIRK_POLLTIMEOUT)) {
            usleep(status.bwPollTimeout * 1000);
        }
    }

    #ifdef DEBUG_DRY
    skip_setup:
        if (!transfer_size) transfer_size = default_transfer_size;
    #endif

    switch (mode) {
    case MODE_UPLOAD:
        if (dfuse) {
            if (dfuse_address)
                dfuse_set_address_pointer(dif->dev_handle, dif->interface, dfuse_address, 0);
            if (dfuse_do_upload(dif->dev_handle, dif->interface, transfer_size, filename) < 0)
                return 1;
        } else {
            if (dfuload_do_upload(dif->dev_handle, dif->interface, transfer_size, filename) < 0)
                return 1;
        }
        break;

    case MODE_DOWNLOAD:
        if (dfuse) {
            if (dfuse_address) {
                if (dfuse_do_bin_dnload(dif->dev_handle, dif->interface, transfer_size, filename, dfuse_address) < 0)
                    return 1;
            } else {
                if (dfuse_do_dfuse_dnload(dif->dev_handle, dif->interface, transfer_size, filename) < 0)
                    return 1;
            }
        } else {
            if (dfuload_do_dnload(dif->dev_handle, dif->interface, transfer_size, filename) < 0)
                return 1;
        }
        break;

    default:
        fprintf(stderr, "Unsupported mode: %u\n", mode);
        return 1;
    }

    // Handle final reset if requested
    if (final_reset) {
        if (dfu_detach(dif->dev_handle, dif->interface, 1000) < 0) {
            fprintf(stderr, "can't detach: %s\n", libusb_error_name(ret));
        }
        printf("Resetting USB to switch back to runtime mode\n");

        // Using libusb_reset_device for the reset
        ret = libusb_reset_device(dif->dev_handle);
        if (ret < 0 && ret != LIBUSB_ERROR_NO_DEVICE) {
            fprintf(stderr, "error resetting after download: %s\n", libusb_error_name(ret));
        }
    }

    return 0;
}

void warningBlock()
{
    usb_get_extra_descriptor(0, 0, 0, 0, 0);
}
