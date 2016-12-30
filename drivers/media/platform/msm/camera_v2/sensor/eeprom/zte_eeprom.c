/* Copyright (c) 2011-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s: %s: " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_eeprom.h"

/* #define SENSOR_EEPROM_DEBUG */
#undef CDBG
#ifdef SENSOR_EEPROM_DEBUG
#define CDBG(fmt, args...) pr_info(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

typedef struct {
	uint16_t id;
	const char *sensor_module_name;
	const char *chromtix_lib_name;
	const char *default_chromtix_lib_name;
} MODULE_Map_Table;

#define OV8856_SENSOR_INFO_MODULE_ID_SUNNY	0x01
#define OV8856_SENSOR_INFO_MODULE_ID_TRULY	0x02
#define OV8856_SENSOR_INFO_MODULE_ID_A_KERR	0x03
#define OV8856_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define OV8856_SENSOR_INFO_MODULE_ID_DARLING	0x05
#define OV8856_SENSOR_INFO_MODULE_ID_QTECH	0x06
#define OV8856_SENSOR_INFO_MODULE_ID_OFLIM	0x07
#define OV8856_SENSOR_INFO_MODULE_ID_FOXCONN	0x11
#define OV8856_SENSOR_INFO_MODULE_ID_IMPORTEK	0x12
#define OV8856_SENSOR_INFO_MODULE_ID_ALTEK	0x13
#define OV8856_SENSOR_INFO_MODULE_ID_ABICO	0x14
#define OV8856_SENSOR_INFO_MODULE_ID_LITE_ON	0x15
#define OV8856_SENSOR_INFO_MODULE_ID_CHICONNY	0x16
#define OV8856_SENSOR_INFO_MODULE_ID_PRIMAX	0x17
#define OV8856_SENSOR_INFO_MODULE_ID_SHARP	0x21
#define OV8856_SENSOR_INFO_MODULE_ID_MCNEX	0x31

MODULE_Map_Table OV8856_MODULE_MAP[] = {
	{ OV8856_SENSOR_INFO_MODULE_ID_SUNNY, "sunny_ov8856", "sunny_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_TRULY, "truly_ov8856", "truly_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_A_KERR, "a_kerr_ov8856", "a_kerr_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_LITEARRAY, "litearray_ov8856", "litearray_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_DARLING, "darling_ov8856", "darling_tov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_QTECH, "qtech_ov8856"," qtech_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_OFLIM, "oflim_ov8856","oflim_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_FOXCONN, "foxconn_ov8856","foxconn_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_IMPORTEK, "importek_ov8856","importek_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_ALTEK, "altek_ov8856","altek_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_ABICO, "abico_ov8856","abico_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_LITE_ON, "lite_on_ov8856","lite_on_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_CHICONNY, "chiconny_ov8856","chiconny_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_PRIMAX, "primax_ov8856","primax_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_SHARP, "sharp_ov8856","sharp_ov8856", NULL},
	{ OV8856_SENSOR_INFO_MODULE_ID_MCNEX, "mcnex_ov8856","mcnex_ov8856", NULL},
};

#define S5K2T8_SENSOR_INFO_MODULE_ID_SUNNY	0x01
#define S5K2T8_SENSOR_INFO_MODULE_ID_TRULY	0x02
#define S5K2T8_SENSOR_INFO_MODULE_ID_A_KERR	0x03
#define S5K2T8_SENSOR_INFO_MODULE_ID_LITEARRAY	0x04
#define S5K2T8_SENSOR_INFO_MODULE_ID_DARLING	0x05
#define S5K2T8_SENSOR_INFO_MODULE_ID_QTECH	0x06
#define S5K2T8_SENSOR_INFO_MODULE_ID_OFLIM	0x07
#define S5K2T8_SENSOR_INFO_MODULE_ID_FOXCONN	0x11
#define S5K2T8_SENSOR_INFO_MODULE_ID_IMPORTEK	0x12
#define S5K2T8_SENSOR_INFO_MODULE_ID_ALTEK	0x13
#define S5K2T8_SENSOR_INFO_MODULE_ID_ABICO	0x14
#define S5K2T8_SENSOR_INFO_MODULE_ID_LITE_ON	0x15
#define S5K2T8_SENSOR_INFO_MODULE_ID_SEMCO	0xA1

MODULE_Map_Table S5K2T8_MODULE_MAP[] = {
	{ S5K2T8_SENSOR_INFO_MODULE_ID_SUNNY, "sunny_s5k2t8", "sunny_s5k2t8", NULL},
	{ S5K2T8_SENSOR_INFO_MODULE_ID_TRULY, "truly_s5k2t8", "truly_s5k2t8", NULL},
	{ S5K2T8_SENSOR_INFO_MODULE_ID_A_KERR, "a_kerr_s5k2t8", "a_kerr_s5k2t8", NULL},
	{ S5K2T8_SENSOR_INFO_MODULE_ID_LITEARRAY, "litearray_s5k2t8", "litearray_s5k2t8", NULL},
	{ S5K2T8_SENSOR_INFO_MODULE_ID_DARLING, "darling_s5k2t8", "darling_s5k2t8", NULL},
	{ S5K2T8_SENSOR_INFO_MODULE_ID_QTECH, "qtech_s5k2t8", "qtech_s5k2t8", NULL},
	{ S5K2T8_SENSOR_INFO_MODULE_ID_OFLIM, "oflim_s5k2t8", "oflim_s5k2t8", NULL},
	{ S5K2T8_SENSOR_INFO_MODULE_ID_FOXCONN, "foxconn_s5k2t8", "foxconn_s5k2t8", NULL},
	{ S5K2T8_SENSOR_INFO_MODULE_ID_IMPORTEK, "importek_s5k2t8", "importek_s5k2t8", NULL},
	{ S5K2T8_SENSOR_INFO_MODULE_ID_ALTEK, "altek_s5k2t8", "altek_s5k2t8", NULL},
	{ S5K2T8_SENSOR_INFO_MODULE_ID_ABICO, "abico_s5k2t8", "abico_s5k2t8", NULL},
	{ S5K2T8_SENSOR_INFO_MODULE_ID_LITE_ON, "lite_on_s5k2t8", "lite_on_s5k2t8", NULL},
	{ S5K2T8_SENSOR_INFO_MODULE_ID_SEMCO, "semco_s5k2t8", "semco_s5k2t8", NULL},
};

DEFINE_MSM_MUTEX(zte_eeprom_mutex);
#ifdef CONFIG_COMPAT
static struct v4l2_file_operations zte_eeprom_v4l2_subdev_fops;
#endif

typedef int (*zte_read_eeprom_memory_func_t)(struct msm_eeprom_ctrl_t *,
					     struct msm_eeprom_memory_block_t *);


static int zte_eeprom_get_cmm_data(struct msm_eeprom_ctrl_t *e_ctrl,
				   struct msm_eeprom_cfg_data *cdata)
{
	struct msm_eeprom_cmm_t *cmm_data = &e_ctrl->eboard_info->cmm_data;
	cdata->cfg.get_cmm_data.cmm_support = cmm_data->cmm_support;
	cdata->cfg.get_cmm_data.cmm_compression = cmm_data->cmm_compression;
	cdata->cfg.get_cmm_data.cmm_size = cmm_data->cmm_size;
	return 0;
}

static int eeprom_config_read_cal_data(struct msm_eeprom_ctrl_t *e_ctrl,
				       struct msm_eeprom_cfg_data *cdata)
{
	int rc;

	/* check range */
	if (cdata->cfg.read_data.num_bytes > e_ctrl->cal_data.num_data) {
		CDBG("Invalid size. exp %u, req %u\n",
			e_ctrl->cal_data.num_data,
			cdata->cfg.read_data.num_bytes);
		return -EINVAL;
	}
	if (!e_ctrl->cal_data.mapdata)
		return -EFAULT;

	rc = copy_to_user(cdata->cfg.read_data.dbuffer,
		e_ctrl->cal_data.mapdata,
		cdata->cfg.read_data.num_bytes);

	return rc;
}

static int zte_eeprom_config(struct msm_eeprom_ctrl_t *e_ctrl,
			     void __user *argp)
{
	int rc = 0;
	size_t length = 0;
	struct msm_eeprom_cfg_data *cdata = (struct msm_eeprom_cfg_data *)argp;

	CDBG("E\n");

	switch (cdata->cfgtype) {
	case CFG_EEPROM_GET_INFO:
		CDBG("E CFG_EEPROM_GET_INFO\n");
		cdata->is_supported = e_ctrl->is_supported;
		length = strlen(e_ctrl->eboard_info->eeprom_name) + 1;
		if (length > MAX_EEPROM_NAME) {
			pr_err("invalid eeprom_name length %d\n", (int)length);
			rc = -EINVAL;
			break;
		}
		memcpy(cdata->cfg.eeprom_name,
			e_ctrl->eboard_info->eeprom_name, length);
		break;
	case CFG_EEPROM_GET_CAL_DATA:
		CDBG("E CFG_EEPROM_GET_CAL_DATA\n");
		cdata->cfg.get_data.num_bytes = e_ctrl->cal_data.num_data;
		break;
	case CFG_EEPROM_READ_CAL_DATA:
		CDBG("E CFG_EEPROM_READ_CAL_DATA\n");
		rc = eeprom_config_read_cal_data(e_ctrl, cdata);
		break;
	case CFG_EEPROM_GET_MM_INFO:
		CDBG("E CFG_EEPROM_GET_MM_INFO\n");
		rc = zte_eeprom_get_cmm_data(e_ctrl, cdata);
		break;
	default:
		break;
	}

	CDBG("X rc: %d\n", rc);
	return rc;
}

static int zte_eeprom_get_subdev_id(struct msm_eeprom_ctrl_t *e_ctrl, void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;

	CDBG("E\n");

	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	*subdev_id = e_ctrl->subdev_id;

	CDBG("X subdev_id %d\n", *subdev_id);

	return 0;
}

static long zte_eeprom_subdev_ioctl(struct v4l2_subdev *sd,
				    unsigned int cmd, void *arg)
{
	struct msm_eeprom_ctrl_t *e_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;

	CDBG("E\n");
	CDBG("a_ctrl %p argp %p\n", e_ctrl, argp);

	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return zte_eeprom_get_subdev_id(e_ctrl, argp);
	case VIDIOC_MSM_EEPROM_CFG:
		return zte_eeprom_config(e_ctrl, argp);
	}

	CDBG("X cmd = %u\n", cmd);
	return -ENOIOCTLCMD;
}

static struct msm_camera_i2c_fn_t zte_eeprom_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay = msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll = msm_camera_cci_i2c_poll,
};

static int zte_eeprom_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct msm_eeprom_ctrl_t *e_ctrl = v4l2_get_subdevdata(sd);

	if (!e_ctrl) {
		pr_err("failed e_ctrl is NULL\n");
		return -EINVAL;
	}
	return 0;
}

static int zte_eeprom_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct msm_eeprom_ctrl_t *e_ctrl = v4l2_get_subdevdata(sd);

	if (!e_ctrl) {
		pr_err("failed e_ctrl is NULL\n");
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_subdev_internal_ops zte_eeprom_internal_ops = {
	.open = zte_eeprom_open,
	.close = zte_eeprom_close,
};

static struct v4l2_subdev_core_ops zte_eeprom_subdev_core_ops = {
	.ioctl = zte_eeprom_subdev_ioctl,
};

static struct v4l2_subdev_ops zte_eeprom_subdev_ops = {
	.core = &zte_eeprom_subdev_core_ops,
};

static int zte_eeprom_get_dt_data(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int rc = 0, i = 0;
	struct msm_eeprom_board_info *eb_info;
	struct msm_camera_power_ctrl_t *power_info =
		&e_ctrl->eboard_info->power_info;
	struct device_node *of_node = NULL;
	struct msm_camera_gpio_conf *gconf = NULL;
	uint8_t gpio_array_size = 0;
	uint16_t *gpio_array = NULL;

	eb_info = e_ctrl->eboard_info;
	if (e_ctrl->eeprom_device_type == MSM_CAMERA_SPI_DEVICE)
		of_node = e_ctrl->i2c_client.
			spi_client->spi_master->dev.of_node;
	else if (e_ctrl->eeprom_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		of_node = e_ctrl->pdev->dev.of_node;

	if (!of_node) {
		pr_err("of_node is NULL\n");
		return -ENOMEM;
	}

	rc = msm_camera_get_dt_vreg_data(of_node, &power_info->cam_vreg,
					 &power_info->num_vreg);
	if (rc < 0)
		return rc;

	rc = msm_camera_get_dt_power_setting_data(of_node,
		power_info->cam_vreg, power_info->num_vreg,
		power_info);
	if (rc < 0)
		goto ERROR1;

	power_info->gpio_conf = kzalloc(sizeof(struct msm_camera_gpio_conf),
					GFP_KERNEL);
	if (!power_info->gpio_conf) {
		pr_err("kzalloc power_info->gpio_conf\n");
		rc = -ENOMEM;
		goto ERROR2;
	}
	gconf = power_info->gpio_conf;
	gpio_array_size = of_gpio_count(of_node);
	CDBG("gpio count %d\n", gpio_array_size);

	if (gpio_array_size) {
		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
			GFP_KERNEL);
		if (!gpio_array) {
			pr_err("failed\n");
			goto ERROR3;
		}
		for (i = 0; i < gpio_array_size; i++) {
			gpio_array[i] = of_get_gpio(of_node, i);
			CDBG("gpio_array[%d] = %d\n", i, gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("failed\n");
			goto ERROR4;
		}

		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("failed\n");
			goto ERROR4;
		}
		kfree(gpio_array);
	}

	return rc;
ERROR4:
	kfree(gpio_array);
ERROR3:
	kfree(power_info->gpio_conf);
ERROR2:
	kfree(power_info->cam_vreg);
ERROR1:
	kfree(power_info->power_setting);
	return rc;
}

#ifdef CONFIG_COMPAT
static int eeprom_config_read_cal_data32(struct msm_eeprom_ctrl_t *e_ctrl,
					 void __user *arg)
{
	int rc;
	uint8_t *ptr_dest = NULL;
	struct msm_eeprom_cfg_data32 *cdata32 =
		(struct msm_eeprom_cfg_data32 *)arg;
	struct msm_eeprom_cfg_data cdata;

	cdata.cfgtype = cdata32->cfgtype;
	cdata.is_supported = cdata32->is_supported;
	cdata.cfg.read_data.num_bytes = cdata32->cfg.read_data.num_bytes;
	/* check range */
	if (cdata.cfg.read_data.num_bytes > e_ctrl->cal_data.num_data) {
		CDBG("Invalid size. exp %u, req %u\n",
			e_ctrl->cal_data.num_data,
			cdata.cfg.read_data.num_bytes);
		return -EINVAL;
	}
	if (!e_ctrl->cal_data.mapdata)
		return -EFAULT;

	ptr_dest = (uint8_t *)compat_ptr(cdata32->cfg.read_data.dbuffer);

	rc = copy_to_user(ptr_dest, e_ctrl->cal_data.mapdata,
			  cdata.cfg.read_data.num_bytes);

	return rc;
}

static int zte_eeprom_config32(struct msm_eeprom_ctrl_t *e_ctrl,
			       void __user *argp)
{
	struct msm_eeprom_cfg_data *cdata = (struct msm_eeprom_cfg_data *)argp;
	int rc = 0;
	size_t length = 0;

	CDBG("E\n");

	switch (cdata->cfgtype) {
	case CFG_EEPROM_GET_INFO:
		CDBG("E CFG_EEPROM_GET_INFO\n");
		cdata->is_supported = e_ctrl->is_supported;
		length = strlen(e_ctrl->eboard_info->eeprom_name) + 1;
		if (length > MAX_EEPROM_NAME) {
			pr_err("invalid eeprom_name length %d\n", (int)length);
			rc = -EINVAL;
			break;
		}
		memcpy(cdata->cfg.eeprom_name,
			e_ctrl->eboard_info->eeprom_name, length);
		break;
	case CFG_EEPROM_GET_CAL_DATA:
		CDBG("E CFG_EEPROM_GET_CAL_DATA\n");
		cdata->cfg.get_data.num_bytes = e_ctrl->cal_data.num_data;
		break;
	case CFG_EEPROM_READ_CAL_DATA:
		CDBG("E CFG_EEPROM_READ_CAL_DATA\n");
		rc = eeprom_config_read_cal_data32(e_ctrl, argp);
		break;
	default:
		break;
	}

	CDBG("X rc %d\n", rc);

	return rc;
}

static long zte_eeprom_subdev_ioctl32(struct v4l2_subdev *sd,
				      unsigned int cmd, void *arg)
{
	struct msm_eeprom_ctrl_t *e_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;

	CDBG("E\n");
	CDBG("a_ctrl %p argp %p\n", e_ctrl, argp);

	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return zte_eeprom_get_subdev_id(e_ctrl, argp);
	case VIDIOC_MSM_EEPROM_CFG32:
		return zte_eeprom_config32(e_ctrl, argp);
	}

	CDBG("X cmd = %u\n", cmd);
	return -ENOIOCTLCMD;
}

static long zte_eeprom_subdev_do_ioctl32(struct file *file,
					 unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);

	return zte_eeprom_subdev_ioctl32(sd, cmd, arg);
}

static long zte_eeprom_subdev_fops_ioctl32(struct file *file,
					   unsigned int cmd, unsigned long arg)
{
	return video_usercopy(file, cmd, arg, zte_eeprom_subdev_do_ioctl32);
}
#endif

static void parse_module_name(struct msm_eeprom_ctrl_t *e_ctrl,
			      MODULE_Map_Table *map,
			      uint16_t len, uint16_t sensor_module_id)
{
	int i;

	for (i = 0; i < len; i++) {
		if (map[i].id != sensor_module_id)
			continue;
		e_ctrl->sensor_module_name = map[i].sensor_module_name;
		e_ctrl->chromtix_lib_name = map[i].chromtix_lib_name;
		e_ctrl->default_chromtix_lib_name = map[i].default_chromtix_lib_name;
		pr_info("sensor_module_name: %s\n", e_ctrl->sensor_module_name);
		return;
	}

	pr_err("unknown sensor_module_id: %d\n", sensor_module_id);
}

void ov8856_read_eeprom_init(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t temp;

	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),
		0x5000, &temp, MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("0x5000 temp=0x%X\n", temp);

	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),
		0x5001, &temp, MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("0x5001 temp=0x%X\n", temp);

	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x0100,0x01, MSM_CAMERA_I2C_BYTE_DATA);

	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x5001, (0x00 & 0x08) | (temp & (~0x08)),
		MSM_CAMERA_I2C_BYTE_DATA);

	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x3d84, 0xc0, MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client),
		0x3d84, &temp, MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x3d88, 0x70, MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x3d89, 0x10, MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x3d8a, 0x72, MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x3d8b, 0x0a, MSM_CAMERA_I2C_BYTE_DATA);
	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(&(e_ctrl->i2c_client),
		0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	udelay(5);
}

void ov8856_read_eeprom_end(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t temp;

	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(
		&(e_ctrl->i2c_client), 0x5001, &temp,
		MSM_CAMERA_I2C_BYTE_DATA);

	CDBG("0x5001 temp=0x%X", temp);

	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
		&(e_ctrl->i2c_client), 0x5001,
		(0x08 & 0x08) | (temp & (~0x08)),
		MSM_CAMERA_I2C_BYTE_DATA);

	e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
		&(e_ctrl->i2c_client), 0x0100,
		0x00,
		MSM_CAMERA_I2C_BYTE_DATA);
}

static int ov8856_validflag_check_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int flag = 0;

	if (((e_ctrl->cal_data.mapdata[0x7010 - 0x7010] &0xC0) != 0x40) &&
	    ((e_ctrl->cal_data.mapdata[0x7010 - 0x7010] &0x30) != 0x10)){
		pr_err("AWB flag invalid 0x%x\n", e_ctrl->cal_data.mapdata[48]);
		flag |= 0x1;
	}
	if (((e_ctrl->cal_data.mapdata[0x7028 - 0x7010] &0xC0) != 0x40) &&
	    ((e_ctrl->cal_data.mapdata[0x7028 - 0x7010] &0x30) != 0x10)){
		pr_err("LSC flag invalid 0x%x\n", e_ctrl->cal_data.mapdata[80]);
		flag |= 0x4;
	}

	CDBG("valid info flag = 0x%x %s\n", flag, flag ? "false" : "true");

	return flag;
}

enum {
	Invlid_Group,
	Group_One,
	Group_Two,
	Group_Three,
};

int32_t ov8856_check_module_info_group(struct msm_eeprom_ctrl_t *e_ctrl)
{
	uint16_t temp;

	e_ctrl->i2c_client.i2c_func_tbl->i2c_read(
		&(e_ctrl->i2c_client), 0x7010, &temp,
		MSM_CAMERA_I2C_BYTE_DATA);

	CDBG("temp=0x%X", temp);

	if ((temp & 0xC0) == 0x40)
		return Group_One;
	if ((temp & 0x30) == 0x10)
		return Group_Two;

	return Invlid_Group;
}

static int ov8856_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
				     struct msm_eeprom_memory_block_t *block)
{
	int rc = 0;
	struct msm_eeprom_memory_map_t *emap = block->map;
	struct msm_eeprom_board_info *eb_info;
	uint8_t *memptr = block->mapdata;
	uint16_t  sensor_module_id = 0;
	int32_t group_number;
	uint32_t module_id_addr;
	int i;

	pr_info("E\n");

	if (!e_ctrl) {
		pr_err("e_ctrl is NULL\n");
		return -EINVAL;
	}

	eb_info = e_ctrl->eboard_info;
	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	//init before read eeprom
	ov8856_read_eeprom_init(e_ctrl);

	group_number = ov8856_check_module_info_group(e_ctrl);
	switch (group_number) {
	case Group_One:
		module_id_addr = 0x7011;
		break;
	case Group_Two:
		module_id_addr = 0x7019;
		break;
	default:
		break;
	}

	if (module_id_addr) {
		e_ctrl->i2c_client.i2c_func_tbl->i2c_read(
				&(e_ctrl->i2c_client),
				module_id_addr, &sensor_module_id,
				MSM_CAMERA_I2C_BYTE_DATA);
		pr_info("sensor_module_id = 0x%X\n", sensor_module_id);
		parse_module_name(e_ctrl, OV8856_MODULE_MAP,
				  sizeof(OV8856_MODULE_MAP)/sizeof(MODULE_Map_Table),
				  sensor_module_id);
	}

	for (i = 0; i < block->num_map; i++) {
		e_ctrl->i2c_client.addr_type = emap[i].mem.addr_t;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(e_ctrl->i2c_client), emap[i].mem.addr,
				memptr, emap[i].mem.valid_size);
		if (rc < 0) {
			pr_err("read failed\n");
			return rc;
		}
		memptr += emap[i].mem.valid_size;
	}

	e_ctrl->valid_flag = ov8856_validflag_check_eeprom(e_ctrl);
	e_ctrl->checksum = 0;
	ov8856_read_eeprom_end(e_ctrl);

	pr_err("X\n");
	return rc;
}

static int eeprom_verify_checksum(uint8_t *mapdata,
				  uint32_t start, uint32_t end)
{
	int j, checksum = 0;

	for (j = start; j < end; j++)
		checksum += mapdata[j];

	if (((checksum & 0xff) != mapdata[end + 1]) ||
	    (((checksum >> 8) & 0xff) != mapdata[end]))
		return -EINVAL;

	return 0;
}

static int s5k2t8_checksum_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int rc = 0;

	pr_info("PCB version = 0x%x", e_ctrl->cal_data.mapdata[0x001c]);

	if (eeprom_verify_checksum(e_ctrl->cal_data.mapdata, 0, 41)) {
		pr_err("basic info checksum fail\n");
		rc |= 0x1;
	}

	if (eeprom_verify_checksum(e_ctrl->cal_data.mapdata, 48, 53)) {
		pr_err("af info checksum fail\n");
		rc |= 0x2;
	}

	if (eeprom_verify_checksum(e_ctrl->cal_data.mapdata, 64, 78)) {
		pr_err("awb info checksum fail\n");
		rc |= 0x4;
	}

	if (eeprom_verify_checksum(e_ctrl->cal_data.mapdata, 80, 1190)) {
		pr_err("lsc info checksum fail\n");
		rc |= 0x8;
	}

	if (eeprom_verify_checksum(e_ctrl->cal_data.mapdata, 1200, 1239)) {
		pr_err("ois info checksum fail\n");
		rc |= 0x10;
	}

	if (eeprom_verify_checksum(e_ctrl->cal_data.mapdata, 1248, 5368)) {
		pr_err("pdaf info checksum fail\n");
		rc |= 0x20;
	}

	if (eeprom_verify_checksum(e_ctrl->cal_data.mapdata, 0, 5370)) {
		pr_err("total info checksum fail\n");
		rc |= 0x40;
	}

	pr_info("cal info checksum rc = 0x%x\n", rc);

	return rc;
}

static int s5k2t8_validflag_check_eeprom(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int flag = 0;

	if (((e_ctrl->cal_data.mapdata[48] >> 6) & 0x3) != 0x1) {
		pr_err("AF flag invalid 0x%x\n", e_ctrl->cal_data.mapdata[48]);
		flag |= 0x1;
	}

	if (((e_ctrl->cal_data.mapdata[64] >> 6) & 0x3) != 0x1) {
		pr_err("AWB flag invalid 0x%x\n", e_ctrl->cal_data.mapdata[64]);
		flag |= 0x2;
	}

	if (((e_ctrl->cal_data.mapdata[80] >> 6) & 0x3) != 0x1) {
		pr_err("LSC flag invalid 0x%x\n", e_ctrl->cal_data.mapdata[80]);
		flag |= 0x4;
	}

	if (((e_ctrl->cal_data.mapdata[1200] >> 6) & 0x3) != 0x1) {
		pr_err("OIS flag invalid 0x%x\n", e_ctrl->cal_data.mapdata[1200]);
		flag |= 0x8;
	}

	if (((e_ctrl->cal_data.mapdata[1248] >> 6) & 0x3) != 0x1) {
		pr_err("PDAF flag invalid 0x%x\n", e_ctrl->cal_data.mapdata[1248]);
		flag |= 0x10;
	}

	pr_err("valid info flag = 0x%x %s\n", flag, flag ? "false" : "true");

	return flag;

}
static int s5k2t8_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
				     struct msm_eeprom_memory_block_t *block)
{
	int j, rc = 0;
	struct msm_eeprom_memory_map_t *emap = block->map;
	uint8_t *memptr = block->mapdata;
	uint16_t  sensor_module_id = 0;

	CDBG("E\n");

	if (!e_ctrl) {
		pr_err("e_ctrl is NULL\n");
		return -EINVAL;
	}

	for (j = 0; j < block->num_map; j++) {
		e_ctrl->i2c_client.addr_type = emap[j].mem.addr_t;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(e_ctrl->i2c_client), emap[0].mem.addr,
				memptr, emap[j].mem.valid_size);
		if (rc < 0) {
			pr_err("read failed\n");
			return rc;
		}
		memptr += emap[j].mem.valid_size;
	}
	e_ctrl->checksum = s5k2t8_checksum_eeprom(e_ctrl);
	e_ctrl->valid_flag= s5k2t8_validflag_check_eeprom(e_ctrl);
	sensor_module_id = block->mapdata[0];
	parse_module_name(e_ctrl, S5K2T8_MODULE_MAP,
			  sizeof(S5K2T8_MODULE_MAP)/sizeof(MODULE_Map_Table),
			  sensor_module_id);
	CDBG("X\n");
	return rc;
}

static int zte_eeprom_generate_map(struct device_node *of,
				   struct msm_eeprom_memory_block_t *data)
{
	int i, rc = 0;
	char property[PROPERTY_MAXSIZE];
	uint32_t count = 6;
	struct msm_eeprom_memory_map_t *map;
	snprintf(property, PROPERTY_MAXSIZE, "zte,num-blocks");
	rc = of_property_read_u32(of, property, &data->num_map);
	CDBG("%s %d\n", property, data->num_map);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}
	map = kzalloc((sizeof(*map) * data->num_map), GFP_KERNEL);
	if (!map) {
		pr_err("kzalloc data->num_map\n");
		return -ENOMEM;
	}
	data->map = map;
	for (i = 0; i < data->num_map; i++) {
		snprintf(property, PROPERTY_MAXSIZE, "zte,mem%d", i);
		rc = of_property_read_u32_array(of, property,
				(uint32_t *) &map[i].mem, count);
		if (rc < 0) {
			pr_err("failed\n");
			goto ERROR;
		}
		data->num_data += map[i].mem.valid_size;
	}
	CDBG("num_bytes %d\n", data->num_data);
	data->mapdata = kzalloc(data->num_data, GFP_KERNEL);
	if (!data->mapdata) {
		pr_err("failed\n");
		rc = -ENOMEM;
		goto ERROR;
	}
	return rc;
ERROR:
	kfree(data->map);
	memset(data, 0, sizeof(*data));
	return rc;
}

static const struct of_device_id zte_eeprom_dt_match[] = {
	{ .compatible = "zte,eeprom-ov8856", .data = (void *)ov8856_read_eeprom_memory },
	{ .compatible = "zte,eeprom-s5k2t8", .data = (void *)s5k2t8_read_eeprom_memory },
	{ }
};

static int zte_eeprom_platform_probe(struct platform_device *pdev)
{
	int rc = 0;
	uint32_t temp;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_eeprom_ctrl_t *e_ctrl = NULL;
	struct msm_eeprom_board_info *eb_info = NULL;
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	const struct of_device_id *match;
	zte_read_eeprom_memory_func_t zte_read_eeprom_memory = NULL;

	CDBG("E\n");

	match = of_match_device(zte_eeprom_dt_match, &pdev->dev);
	zte_read_eeprom_memory = (zte_read_eeprom_memory_func_t)match->data;

	e_ctrl = kzalloc(sizeof(*e_ctrl), GFP_KERNEL);
	if (!e_ctrl) {
		pr_err("kzalloc e_ctrl\n");
		return -ENOMEM;
	}

	e_ctrl->eeprom_v4l2_subdev_ops = &zte_eeprom_subdev_ops;
	e_ctrl->eeprom_mutex = &zte_eeprom_mutex;

	if (!of_node) {
		pr_err("dev.of_node NULL\n");
		return -EINVAL;
	}

	/* Set platform device handle */
	e_ctrl->pdev = pdev;
	/* Set device type as platform device */
	e_ctrl->eeprom_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	e_ctrl->i2c_client.i2c_func_tbl = &zte_eeprom_cci_func_tbl;
	e_ctrl->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!e_ctrl->i2c_client.cci_client) {
		pr_err("kzalloc e_ctrl->i2c_client.cci_client\n");
		rc = -ENOMEM;
		goto ectrl_free;
	}

	e_ctrl->eboard_info = kzalloc(sizeof(
		struct msm_eeprom_board_info), GFP_KERNEL);
	if (!e_ctrl->eboard_info) {
		pr_err("kzalloc e_ctrl->eboard_info\n");
		rc = -ENOMEM;
		goto cciclient_free;
	}

	eb_info = e_ctrl->eboard_info;
	power_info = &eb_info->power_info;
	cci_client = e_ctrl->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;
	power_info->dev = &pdev->dev;

	/*Get clocks information*/
	rc = msm_camera_get_clk_info(e_ctrl->pdev,
		&power_info->clk_info,
		&power_info->clk_ptr,
		&power_info->clk_info_size);
	if (rc < 0) {
		pr_err("failed msm_camera_get_clk_info rc %d\n", rc);
		goto board_free;
	}

	rc = of_property_read_u32(of_node, "cell-index",
		&pdev->id);
	CDBG("cell-index %d rc %d\n", pdev->id, rc);
	if (rc < 0) {
		pr_err("failed read cell-index rc %d\n", rc);
		goto board_free;
	}
	e_ctrl->subdev_id = pdev->id;

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&e_ctrl->cci_master);
	CDBG("qcom,cci-master %d rc %d\n", e_ctrl->cci_master, rc);
	if (rc < 0) {
		pr_err("failed read qcom,cci-master rc %d\n", rc);
		goto board_free;
	}
	cci_client->cci_i2c_master = e_ctrl->cci_master;

	rc = of_property_read_u32(of_node, "qcom,slave-addr",
		&temp);
	eb_info->i2c_slaveaddr = temp;
	CDBG("qcom,slave-addr 0x%X rc %d\n", eb_info->i2c_slaveaddr, rc);
	if (rc < 0) {
		pr_err("failed read qcom,slave-addr rc %d\n", rc);
		goto board_free;
	}
	cci_client->sid = eb_info->i2c_slaveaddr >> 1;

	rc = of_property_read_string(of_node, "qcom,eeprom-name",
		&eb_info->eeprom_name);
	CDBG("qcom,eeprom-name %s rc %d\n", eb_info->eeprom_name, rc);
	if (rc < 0) {
		pr_err("failed read qcom,eeprom_name rc %d\n", rc);
		goto board_free;
	}

	rc = zte_eeprom_get_dt_data(e_ctrl);
	if (rc)
		goto board_free;

	rc = zte_eeprom_generate_map(of_node, &e_ctrl->cal_data);
	if (rc < 0)
		goto board_free;

	rc = msm_camera_power_up(power_info, e_ctrl->eeprom_device_type,
				 &e_ctrl->i2c_client);
	if (rc) {
		pr_err("failed msm_camera_power_up rc %d\n", rc);
		goto memdata_free;
	}

	rc = zte_read_eeprom_memory(e_ctrl, &e_ctrl->cal_data);
	if (rc < 0) {
		pr_err("read_eeprom_memory failed rc %d\n", rc);
		goto power_down;
	}

	e_ctrl->is_supported = 1;

	rc = msm_camera_power_down(power_info, e_ctrl->eeprom_device_type,
				   &e_ctrl->i2c_client);
	if (rc) {
		pr_err("failed msm_camera_power_down rc %d\n", rc);
		goto memdata_free;
	}

	v4l2_subdev_init(&e_ctrl->msm_sd.sd,
		e_ctrl->eeprom_v4l2_subdev_ops);
	v4l2_set_subdevdata(&e_ctrl->msm_sd.sd, e_ctrl);
	platform_set_drvdata(pdev, &e_ctrl->msm_sd.sd);
	e_ctrl->msm_sd.sd.internal_ops = &zte_eeprom_internal_ops;
	e_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(e_ctrl->msm_sd.sd.name,
		ARRAY_SIZE(e_ctrl->msm_sd.sd.name), "msm_eeprom");
	media_entity_init(&e_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	e_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	e_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_EEPROM;
	msm_sd_register(&e_ctrl->msm_sd);
#ifdef CONFIG_COMPAT
	zte_eeprom_v4l2_subdev_fops = v4l2_subdev_fops;
	zte_eeprom_v4l2_subdev_fops.compat_ioctl32 =
		zte_eeprom_subdev_fops_ioctl32;
	e_ctrl->msm_sd.sd.devnode->fops = &zte_eeprom_v4l2_subdev_fops;
#endif
	e_ctrl->is_supported = (e_ctrl->is_supported << 1) | 1;

	pr_info("successfully initialized camera sensor %s\n",
		eb_info->eeprom_name);
	return 0;
power_down:
	msm_camera_power_down(power_info, e_ctrl->eeprom_device_type,
			      &e_ctrl->i2c_client);
memdata_free:
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
board_free:
	kfree(e_ctrl->eboard_info);
cciclient_free:
	kfree(e_ctrl->i2c_client.cci_client);
ectrl_free:
	kfree(e_ctrl);
	return rc;
}

static int zte_eeprom_platform_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct msm_eeprom_ctrl_t  *e_ctrl;
	if (!sd) {
		pr_err("subdevice is NULL\n");
		return 0;
	}

	e_ctrl = (struct msm_eeprom_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!e_ctrl) {
		pr_err("eeprom device is NULL\n");
		return 0;
	}

	kfree(e_ctrl->i2c_client.cci_client);
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
	if (e_ctrl->eboard_info) {
		msm_camera_put_clk_info(e_ctrl->pdev,
			&e_ctrl->eboard_info->power_info.clk_info,
			&e_ctrl->eboard_info->power_info.clk_ptr,
			e_ctrl->eboard_info->power_info.clk_info_size);
		kfree(e_ctrl->eboard_info->power_info.gpio_conf);
		kfree(e_ctrl->eboard_info);
	}
	kfree(e_ctrl);
	return 0;
}

static struct platform_driver zte_eeprom_platform_driver = {
	.driver = {
		.name = "zte_eeprom",
		.owner = THIS_MODULE,
		.of_match_table = zte_eeprom_dt_match,
	},
	.remove = zte_eeprom_platform_remove,
};

static int __init zte_eeprom_init_module(void)
{
	int rc;
	CDBG("E\n");
	rc = platform_driver_probe(&zte_eeprom_platform_driver,
		zte_eeprom_platform_probe);
	CDBG("platform_driver_probe rc %d\n", rc);
	return rc;
}

static void __exit zte_eeprom_exit_module(void)
{
	platform_driver_unregister(&zte_eeprom_platform_driver);
}

module_init(zte_eeprom_init_module);
module_exit(zte_eeprom_exit_module);
MODULE_DESCRIPTION("MSM EEPROM driver for ZTE");
MODULE_LICENSE("GPL v2");
