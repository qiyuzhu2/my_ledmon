/*
 * AMD LED is controlled by writing raw registers.
 * Copyright (C) 2023-, Advanced Micro Devices, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libgen.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <sys/file.h>
#include <sys/types.h>
#include <time.h>

#if _HAVE_DMALLOC_H
#include <dmalloc.h>
#endif

#include "config.h"
#include "led/libled.h"
#include "list.h"
#include "utils.h"
#include "amd.h"
#include "amd_raw_register.h"
#include "libled_private.h"

static uint8_t sgpio_ibpi_pattern[] = {
	[LED_IBPI_PATTERN_NONE]		= 0x00,
	[LED_IBPI_PATTERN_NORMAL]	= 0x20,
	[LED_IBPI_PATTERN_REBUILD]	= 0x07,
	[LED_IBPI_PATTERN_HOTSPARE]	= 0x02,
	[LED_IBPI_PATTERN_PFA]		= 0x03,
	[LED_IBPI_PATTERN_FAILED_DRIVE]	= 0x01,
	[LED_IBPI_PATTERN_LOCATE]	= 0x08,
	[LED_IBPI_PATTERN_LOCATE_OFF]	= 0x00
};

#define NVME_LED_CTL_OFFSET	6
#define NVME_LED_CTL_MASK	(0xf << NVME_LED_CTL_OFFSET)

static uint8_t nvme_ibpi_pattern[] = {
	[LED_IBPI_PATTERN_NORMAL]	= 0xf,
	[LED_IBPI_PATTERN_FAILED_DRIVE]	= 0xd,   /* solid on */
	[LED_IBPI_PATTERN_LOCATE]	= 0xb,   /* blink */
	[LED_IBPI_PATTERN_LOCATE_OFF]	= 0xf,
};

struct initiator_ports_cache {
	enum led_ibpi_pattern leds[4];
	uint8_t blink_gen_a;
	uint8_t blink_gen_b;
	uint16_t reserved;
};

static struct initiator_ports_cache ports_cache[16];

#define HBA_SMN_HOST_BASE	0x3101000
#define HBA_SMN_HOST_SIZE	0x100000
#define EM_LOC_OFFSET		0x1C
#define EM_CTL_OFFSET		0x20
#define GHC_EM_CTL_RST		(1 << 9)
#define GHC_EM_CTL_TM		(1 << 8)
#define GHC_CAP_EMS		(1 << 6)


/* sgpio mmio */
#define SGPIO_CFG		0x4
#define SGPIO_REG_CNT		0x8
#define SGPIO_WRITE_DATA	0xc
#define SGPIO_WRITE_DATA_SIZE	0x8

#define SGPIO_WAIT 	2000

struct sgpio_message {
	FILE *fp;
	int host_nr;
	unsigned int sgpio_base;
	unsigned char reg_type;
	unsigned char reg_index;
	unsigned char reg_count;
	unsigned int buf[SGPIO_WRITE_DATA_SIZE];
};

#define SGPIO_FUNC_AND_FRAME_TYPE	0x8240

static int read_pci_config_byte(FILE *fp, int offset, unsigned char *data)
{
	int ret;

	if ((offset < 0) && (offset > 4095)) {
		return -1;
	}
	ret = fseek(fp, offset, SEEK_SET);
	if (ret) {
		return ret;
	}
	ret = fread(data, 1, 1, fp);
	if (ret != 1) {
		return -1;
	}

	return ret;
}

static int read_pci_config_short(FILE *fp, int offset, unsigned short *data)
{
	int ret;

	if ((offset < 0) && (offset > 4094)) {
		return -1;
	}
	ret = fseek(fp, offset, SEEK_SET);
	if (ret) {
		return ret;
	}
	ret = fread(data, 2, 1, fp);
	if (ret != 1) {
		return -1;
	}

	return ret;
}

static int write_pci_config_short(FILE *fp, int offset, unsigned short data)
{
	int ret;

	if ((offset < 0) && (offset > 4094)) {
		return -1;
	}

	ret = fseek(fp, offset, SEEK_SET);
	if (ret) {
		return ret;
	}
	ret = fwrite(&data, 2, 1, fp);
	if (ret != 1) {
		return -1;
	}

	return ret;
}

static int smn_write_data(FILE *fp, unsigned int addr, unsigned int data)
{
	int ret;

	ret = fseek(fp, 0x60, SEEK_SET);
	if (ret) {
		return ret;
	}
	ret = fwrite(&addr, 4, 1, fp);
	if (ret != 1) {
		return -1;
	}
	ret = fwrite(&data, 4, 1, fp);
	if (ret != 1) {
		return -1;
	}

	return 0;
}

static int smn_read_data(FILE *fp, unsigned int addr, unsigned int *buf)
{
	int ret;

	ret = fseek(fp, 0x60, SEEK_SET);
	if (ret) {
		return ret;
	}
	ret = fwrite(&addr, 4, 1, fp);
	if (ret != 1) {
		return -1;
	}
	ret = fread(buf, 4, 1, fp);
	if (ret != 1) {
		return -1;
	}

	return 0;
}

static int is_block_controller(const char *path, const char *type)
{
	char temp[PATH_MAX], link[PATH_MAX], *t;

	snprintf(temp, sizeof(temp), "%s/driver", path);

	if (realpath(temp, link) == NULL)
		return 0;

	t = strrchr(link, '/');
	if ((t != NULL) && (strcmp(t + 1, type) != 0))
		return 0;

	return 1;
}

static int is_ahci_controller(const char *path)
{
	return is_block_controller(path, "ahci");
}

static int is_nvme_controller(const char *path)
{
	return is_block_controller(path, "nvme");
}

static char *get_rc_config_path(struct led_ctx *ctx, const char *path)
{
	unsigned int rc_bus_nr;
	char *tmp;
	char *rc_config_path;
	char tmp_array[3] = {'\0'};

	tmp = strstr(path, "pci0000:");
	if (!tmp) {
		lib_log(ctx, LED_LOG_LEVEL_INFO,
			"[sata]Get root complex bus num fail\n");
		return NULL;
	}
	strncpy(tmp_array, tmp + 8, 2);
	rc_bus_nr = strtoul(tmp_array, NULL, 16);

	rc_config_path = malloc(PATH_MAX);
	if (!rc_config_path) {
		lib_log(ctx, LED_LOG_LEVEL_INFO,
			"[sata]Alloc mem for rc_config_path fail!\n");
		return NULL;
	}
	memset(rc_config_path, 0, PATH_MAX);
	snprintf(rc_config_path, PATH_MAX,
		"/sys/devices/pci0000:%02x/0000:%02x:00.0/config",
		rc_bus_nr, rc_bus_nr);

	return rc_config_path;
}

static char *get_nvme_config_path(struct led_ctx *ctx, const char *path)
{
	char *nvme_config_path;
	char *tmp;
	int len1, len2;

	nvme_config_path = malloc(PATH_MAX);
	if (!nvme_config_path) {
		lib_log(ctx, LED_LOG_LEVEL_INFO,
			"[nvme] Alloc mem for nvme_config_path fail!\n");
		return NULL;
	}
	tmp = strrchr(path, '/');
	len1 = strlen(tmp);
	len2 = strlen(path);

	memset(nvme_config_path, 0, PATH_MAX);
	memcpy(nvme_config_path, path, (len2 - len1));
	nvme_config_path[len2 - len1] = '/';
	memcpy(nvme_config_path + (len2 - len1) + 1, "config", 6);
	lib_log(ctx, LED_LOG_LEVEL_INFO, "nvme_config_path is： %s\n", nvme_config_path);

	return nvme_config_path;
}

static void put_path(void *path)
{
	free(path);
	path = NULL;
}

static int get_slot_control_reg_offset(struct led_ctx *ctx, FILE *fp)
{
	unsigned char pcie_status_reg;
	unsigned char capabilities_pointer;
	unsigned char capability_id;
	unsigned char capability_reg;
	unsigned char slot_cap_reg;

	read_pci_config_byte(fp, 0x06, &pcie_status_reg);
	if (!(pcie_status_reg & (1 << 4))) {
		lib_log(ctx, LED_LOG_LEVEL_INFO, "[nvme]The PCIe not support cap list!\n");
		return -1;
	}
	read_pci_config_byte(fp, 0x34, &capabilities_pointer);
	read_pci_config_byte(fp, capabilities_pointer, &capability_id);
	while((0x10 != capability_id) && (capabilities_pointer != 0)) {
		read_pci_config_byte(fp, capabilities_pointer + 1, &capabilities_pointer);
		read_pci_config_byte(fp, capabilities_pointer, &capability_id);
	}
	lib_log(ctx, LED_LOG_LEVEL_INFO,
		"[nvme]The PCIe capability struct offset = 0x%x\n", capabilities_pointer);
	if (!capabilities_pointer) {
		lib_log(ctx, LED_LOG_LEVEL_INFO, "[nvme]Can't find the PCIe cap structure!\n");
		return -1;
	}

	read_pci_config_byte(fp, capabilities_pointer + 0x3, &capability_reg);
	if (!(capability_reg & (1 << 0))) {
		lib_log(ctx, LED_LOG_LEVEL_INFO, "[nvme]The PCIe not support slot!\n");
		return -1;
	}

	read_pci_config_byte(fp, capabilities_pointer + 0x14, &slot_cap_reg);
	if (!(slot_cap_reg & (1 << 3))  || !(slot_cap_reg & (1 << 4))) {
		lib_log(ctx, LED_LOG_LEVEL_INFO,
			"[nvme]The slot cap not support attention and power indicator!\n");
		return -1;
	}

	return (capabilities_pointer + 0x18);
}

static int check_nvme_fw_config(struct led_ctx *ctx, const char *path)
{
	FILE *fp;
	char *nvme_config_path;

	nvme_config_path = get_nvme_config_path(ctx, path);
	fp = fopen(nvme_config_path, "rb+");
	if (!fp) {
		lib_log(ctx, LED_LOG_LEVEL_INFO,
			"Open %s fail!, please check the root device of sata controller\n",
			nvme_config_path);
		goto fail1;
	}

	if (get_slot_control_reg_offset(ctx, fp) < 0) {
		lib_log(ctx, LED_LOG_LEVEL_INFO,
			"Firmware has not configured with nvme slot control enable!\n");
		goto fail2;
	}

	fclose(fp);
	put_path(nvme_config_path);

	return 0;

fail2:
	fclose(fp);
fail1:
	put_path(nvme_config_path);
	return -1;
}

static int check_sgpio_fw_config(struct led_ctx *ctx, const char *path, int host_nr)
{
	FILE *fp;
	unsigned int em_loc_data;
	unsigned int em_loc_addr;
	unsigned int em_config_data;
	unsigned int em_config_addr;
	char *rc_config_path;

	rc_config_path = get_rc_config_path(ctx, path);
	fp = fopen(rc_config_path, "rb+");
	if (!fp) {
		lib_log(ctx, LED_LOG_LEVEL_INFO,
			"Open %s fail!, please check the root device of sata controller\n",
			rc_config_path);
		goto fail1;
	}
	em_config_addr = HBA_SMN_HOST_BASE + host_nr * HBA_SMN_HOST_SIZE;
	smn_read_data(fp, em_config_addr, &em_config_data);
	if (!(em_config_data & GHC_CAP_EMS)) {
		lib_log(ctx, LED_LOG_LEVEL_INFO,
			"Firmware has not configured with sgpio enable!\n");
		goto fail2;
	}

	em_loc_addr = HBA_SMN_HOST_BASE + EM_LOC_OFFSET + host_nr * HBA_SMN_HOST_SIZE;
	smn_read_data(fp, em_loc_addr, &em_loc_data);
	if (em_loc_data == 0) {
		lib_log(ctx, LED_LOG_LEVEL_INFO,
			"The sgpio mmio offset and size is not set in the firmware!\n");
		goto fail2;
	}
	fclose(fp);
	put_path(rc_config_path);

	return 0;

fail2:
	fclose(fp);
fail1:
	put_path(rc_config_path);

	return -1;
}

static int send_sgpio_message(struct led_ctx *ctx, struct sgpio_message *msg)
{
	unsigned int cfg_data;
	unsigned int data;
	unsigned int addr;

	if (!msg || !msg->fp || !msg->sgpio_base || !msg->reg_count) {
		lib_log(ctx, LED_LOG_LEVEL_INFO, "Msg Parameter error!\n");
		return -1;
	}

	cfg_data = ((msg->reg_index & 0xff) << 24) |
			((msg->reg_type & 0xff) << 16) |
			(SGPIO_FUNC_AND_FRAME_TYPE & 0xffff);
	lib_log(ctx, LED_LOG_LEVEL_INFO, "cfg_data = 0x%x, count = %d,  data = %x\n", cfg_data,
		msg->reg_count, msg->buf[0]);
	smn_write_data(msg->fp, msg->sgpio_base + SGPIO_CFG, cfg_data);
	smn_write_data(msg->fp, msg->sgpio_base + SGPIO_REG_CNT, msg->reg_count);
	for (int i = 0; i < msg->reg_count; i++)
		smn_write_data(msg->fp, msg->sgpio_base + SGPIO_WRITE_DATA + 4 * i, msg->buf[i]);

	/* send */
	addr = HBA_SMN_HOST_BASE + EM_CTL_OFFSET + msg->host_nr * HBA_SMN_HOST_SIZE;
	smn_read_data(msg->fp, addr, &data);
	while (data & GHC_EM_CTL_TM) {
		smn_read_data(msg->fp, addr, &data);
		usleep(SGPIO_WAIT);
	}
	data |= GHC_EM_CTL_TM;
	smn_write_data(msg->fp, addr, data);
	smn_read_data(msg->fp, addr, &data);
	while (data & GHC_EM_CTL_TM) {
		smn_read_data(msg->fp, addr, &data);
		usleep(SGPIO_WAIT);
	}

	return 0;
}

static int config_nvme_slot(struct led_ctx *ctx, FILE *fp, enum led_ibpi_pattern ibpi)
{
	unsigned char cap_reg_offset;
	unsigned short slot_ctl_reg;

	cap_reg_offset = get_slot_control_reg_offset(ctx, fp);
	if (cap_reg_offset < 0) {
		lib_log(ctx, LED_LOG_LEVEL_INFO, "Get pcie cap structure offset fail!\n");
		return -1;
	}

	read_pci_config_short(fp, cap_reg_offset, &slot_ctl_reg);

	lib_log(ctx, LED_LOG_LEVEL_INFO,
		"slot ctl register offset is 0x%02x, value = 0x%02x, ibpi = 0x%02x!\n",
		(unsigned int)cap_reg_offset, slot_ctl_reg, ibpi);

	slot_ctl_reg &= (~NVME_LED_CTL_MASK);
	slot_ctl_reg |= ((nvme_ibpi_pattern[ibpi] << NVME_LED_CTL_OFFSET) &
				NVME_LED_CTL_MASK);

	lib_log(ctx, LED_LOG_LEVEL_INFO, "slot ctl register = 0x%02x!\n", slot_ctl_reg);
	write_pci_config_short(fp, cap_reg_offset, slot_ctl_reg);

	return 0;
}

static int config_sgpio_initiator(struct led_ctx *ctx, FILE *fp, int host_nr, int port_nr,
							enum led_ibpi_pattern ibpi)
{
	int ret, i;
	unsigned int data0, data1;
	unsigned int sgpio_offset;
	unsigned int sgpio_addr;
	unsigned int em_loc_data;
	unsigned int em_loc_addr;
	struct sgpio_message *msg;
	struct initiator_ports_cache *cache;
	unsigned int led_pattern = 0;

	em_loc_addr = HBA_SMN_HOST_BASE + EM_LOC_OFFSET + host_nr * HBA_SMN_HOST_SIZE;
	smn_read_data(fp, em_loc_addr, &em_loc_data);
	sgpio_offset = ((em_loc_data >> 16) & 0xffff) * 4;
	lib_log(ctx, LED_LOG_LEVEL_INFO,
		"sgpio offset  = 0x%04x, sgpio size = 0x%04x\n", sgpio_offset,
		(em_loc_data & 0xff) * 4);

	sgpio_addr = HBA_SMN_HOST_BASE + host_nr * HBA_SMN_HOST_SIZE + sgpio_offset;
	lib_log(ctx, LED_LOG_LEVEL_INFO, "sgpio addr = 0x%08x \n", sgpio_addr);

	msg = malloc(sizeof(struct sgpio_message));
	if (!msg) {
		lib_log(ctx, LED_LOG_LEVEL_INFO, "malloc err!\n");
		return -1;
	}
	msg->fp = fp;
	msg->host_nr = host_nr;
	msg->sgpio_base = sgpio_addr;

	/* select the sgpio initiator */
	msg->reg_type = 0xc0;
	msg->reg_index = 0x0;
	msg->reg_count = 0x1;
	msg->buf[0] =  (port_nr / 4);
	ret = send_sgpio_message(ctx, msg);
	if (ret < 0) {
		lib_log(ctx, LED_LOG_LEVEL_INFO, "Fail: select the sgpio initiator!\n");
		goto fail1;
	}

	/* led pattern */
	if (port_nr < 4)
		cache = ports_cache + 2 * host_nr;
	else
		cache = ports_cache + 2 * host_nr + 1;

	for (i = 0; i < 4; i++)
		led_pattern |= (sgpio_ibpi_pattern[cache->leds[i]] << ((3 - i) * 8));
	led_pattern &= ~(0xff << ((3 - (port_nr % 4)) * 8));
	led_pattern |= (sgpio_ibpi_pattern[ibpi] << ((3 - (port_nr % 4)) * 8));
	cache->leds[port_nr % 4] = ibpi;

	msg->reg_type = 0x03;
	msg->reg_index = 0x0;
	msg->reg_count = 0x1;
	msg->buf[0] = led_pattern;
	ret = send_sgpio_message(ctx, msg);
	if (ret < 0) {
		lib_log(ctx, LED_LOG_LEVEL_INFO, "Fail: sgpio led pattern!\n");
		goto fail1;
	}

	/* sgpio config */
	data0 = 0x00800000;
	data1 = 0x0f0f7700;
	msg->reg_type = 0x00;
	msg->reg_index = 0x0;
	msg->reg_count = 0x2;
	msg->buf[0] = data0;
	msg->buf[1] = data1;
	ret = send_sgpio_message(ctx, msg);
	if (ret < 0) {
		lib_log(ctx, LED_LOG_LEVEL_INFO, "Fail: sgpio initiator config!\n");
		goto fail1;
	}
	free(msg);

	return 0;

fail1:
	free(msg);
	return -1;
}

static int get_amd_sgpio_host_nr(const char *start_path)
{
	int rc;
	int host_nr;
	struct list dir;
	char *dir_name;
	const char *dir_path;

	rc = scan_dir(start_path, &dir);
	if (rc) {
		return -1;
	}

	host_nr = 0;
	list_for_each(&dir, dir_path) {
		dir_name = strrchr(dir_path, '/');
		if (!dir_name)
			continue;
		/* skip past the leading '/' */
		dir_name++;
		if (strncmp(dir_name, "ata", 3) == 0) {
			sscanf(dir_name, "ata%d", &host_nr);
			return ((host_nr - 1) / 8);
		}
	}

	return -1;
}

static int _set_raw_register(struct block_device *device, enum led_ibpi_pattern ibpi)
{
	int host_nr;
	int port_nr;
	int ret;
	FILE *fp;
	struct led_ctx *ctx = device->cntrl->ctx;

	/* sata */
	if (is_ahci_controller(device->cntrl->sysfs_path)) {
		fp = fopen(device->cntrl_path, "rb+");
		if (!fp) {
			lib_log(ctx, LED_LOG_LEVEL_INFO,
				"Open %s fail!, please check the root device of sata controller\n",
				device->cntrl_path);
			goto fail1;
		}
		port_nr = device->host_id;
		host_nr = port_nr / 8;
		ret = config_sgpio_initiator(ctx, fp, host_nr, port_nr, ibpi);
		if (ret) {
			lib_log(ctx, LED_LOG_LEVEL_INFO, "config sgpio initiator fail!\n");
			goto fail2;
		}
		fclose(fp);

	/* nvme */
	} else if (is_nvme_controller(device->cntrl->sysfs_path)){
		fp = fopen(device->cntrl_path, "rb+");
		if (!fp) {
			lib_log(ctx, LED_LOG_LEVEL_INFO,
				"Open %s fail!, please check the root device of nvme controller\n",
				device->cntrl_path);
			goto fail1;
		}
		ret = config_nvme_slot(ctx, fp, ibpi);
		if (ret) {
			lib_log(ctx, LED_LOG_LEVEL_INFO, "config nvme slot fail!\n");
			goto fail2;
		}
		fclose(fp);
	} else {
		lib_log(ctx, LED_LOG_LEVEL_INFO, "The device(%s) is not supported by AMD!\n",
			device->cntrl_path);
		return -1;
	}
	device->ibpi_prev = ibpi;

	return 0;

fail2:
	fclose(fp);
fail1:
	return -1;
}

int _amd_raw_register_em_enabled(const char *path, struct led_ctx *ctx)
{
	int sata_host_nr;
	struct initiator_ports_cache *cache1, *cache2;
	int i;

	if (is_ahci_controller(path)) {
		sata_host_nr = get_amd_sgpio_host_nr(path);
		if (sata_host_nr < 0) {
			lib_log(ctx, LED_LOG_LEVEL_INFO,
				"Failed to get the sata controller nr:  %s!\n", path);
			return 0;
		}
		/* check the sgpio for sata or pcie slot for nvme */
		if(check_sgpio_fw_config(ctx, path, sata_host_nr)) {
			return 0;
		}
		/* init the led to normal */
		cache1 = ports_cache + sata_host_nr * 2;
		cache2 = ports_cache + sata_host_nr * 2 + 1;
		for (i = 0; i < 4; i++) {
			cache1->leds[i] = LED_IBPI_PATTERN_NORMAL;
			cache2->leds[i] = LED_IBPI_PATTERN_NORMAL;
		}
	} else if (is_nvme_controller(path)) {
		/* check the sgpio for sata or pcie slot for nvme */
		if(check_nvme_fw_config(ctx, path)) {
			return 0;
		}
	} else {
		lib_log(ctx, LED_LOG_LEVEL_INFO,
			"The device was not recognized! %s\n", path);
		return 0;
	}

	/* set the default led pattern */
	return 1;
}

int _amd_raw_register_write(struct block_device *device, enum led_ibpi_pattern ibpi)
{
	if(!device || !device->sysfs_path || !device->cntrl_path)
		__set_errno_and_return(ENOTSUP);

	if ((ibpi < LED_IBPI_PATTERN_NORMAL) || (ibpi > LED_IBPI_PATTERN_LOCATE_OFF))
		__set_errno_and_return(ERANGE);

	if ((ibpi == LED_IBPI_PATTERN_DEGRADED) ||
	    (ibpi == LED_IBPI_PATTERN_FAILED_ARRAY))
		__set_errno_and_return(ENOTSUP);

	return _set_raw_register(device, ibpi);
}

char *_amd_raw_register_get_path(const char *cntrl_path, struct led_ctx *ctx)
{
	char *tmp;

	if (is_ahci_controller(cntrl_path)) {
		return get_rc_config_path(ctx, cntrl_path);
	}
	if (is_nvme_controller(cntrl_path)) {
		tmp =  get_nvme_config_path(ctx, cntrl_path);
		/* TODO: init the led */

		return tmp;
	}

	return NULL;
}

