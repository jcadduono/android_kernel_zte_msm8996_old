/*
 * Driver for zte misc functions
 * function1: used for translate hardware GPIO to SYS GPIO number
 * function2: update fingerprint status to kernel from fingerprintd,2016/01/18
 */

#define pr_fmt(fmt) "%s: %s: " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>	/* for of_iomap() */
#include <linux/delay.h>
#include <linux/power_supply.h>	/* for charging enable/disable */
#include <soc/qcom/socinfo.h>	/* for socinfo_get_*() */

struct zte_gpio_info {
	int sys_num;	/* system pin number */
	const char *name;
};

#define MAX_SUPPORT_GPIOS 16
struct zte_gpio_info zte_gpios[MAX_SUPPORT_GPIOS];

static struct of_device_id zte_misc_of_match[] = {
	{ .compatible = "zte-misc", },
	{ },
};
MODULE_DEVICE_TABLE(of, zte_misc_of_match);

int get_sysnumber_byname(char *name)
{
	int i;
	for (i = 0; i < MAX_SUPPORT_GPIOS; i++) {
		if (zte_gpios[i].name && !strcmp(zte_gpios[i].name, name))
			return zte_gpios[i].sys_num;
	}
	return -1;
}

static int get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	int count = -1;
	pr_info("translate hardware pin to system pin\n");
	node = dev->of_node;
	if (node == NULL)
		return -ENODEV;

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (!of_find_property(pp, "label", NULL)) {
			dev_warn(dev, "Found without labels\n");
			continue;
		}
		count++;
		zte_gpios[count].name = kstrdup(of_get_property(pp, "label", NULL),
								GFP_KERNEL);
		zte_gpios[count].sys_num = of_get_gpio(pp, 0);

		pr_info("sys_number=%d name=%s\n",
			zte_gpios[count].sys_num, zte_gpios[count].name);
	}
	return 0;
}

/*
 * Emode function to enable/disable 0% shutdown
 */
int enable_to_shutdown = 1;
static int set_enable_to_shutdown(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	pr_warn("set_enable_to_shutdown to %d\n", enable_to_shutdown);
	return 0;
}

module_param_call(enable_to_shutdown, set_enable_to_shutdown, param_get_uint,
		  &enable_to_shutdown, 0644);

static int zte_misc_charging_enabled; /* defined in ****-charger.c */
static int zte_misc_control_charging(const char *val, struct kernel_param *kp)
{
	struct power_supply *batt_psy;
	int rc;
	const union power_supply_propval enable = {1,};
	const union power_supply_propval disable = {0,};

	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("error setting value %d\n", rc);
		return rc;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		if (zte_misc_charging_enabled) {
			rc = batt_psy->set_property(batt_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &enable);
			pr_info("enable charging\n");
		} else {
			rc = batt_psy->set_property(batt_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &disable);
			pr_info("disable charging\n");
		}
		if (rc)
			pr_err("battery does not export CHARGING_ENABLED: %d\n",
				rc);
	} else
		pr_err("batt_psy is NULL\n");

	return 0;
}

static int zte_misc_get_charging_enabled_node(char *val, struct kernel_param *kp)
{
	struct power_supply *batt_psy;
	int rc;
	union power_supply_propval pval = {0,};

	batt_psy = power_supply_get_by_name("battery");
	if (batt_psy) {
		rc = batt_psy->get_property(batt_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
		pr_info("enable charging status %d\n", pval.intval);
		if (rc) {
			pr_err("battery charging_enabled node is not exist: %d\n", rc);
			return sprintf(val, "%d", -1);
		}
		zte_misc_charging_enabled = pval.intval;
	} else
		pr_err("batt_psy is NULL\n");

	return sprintf(val, "%d", pval.intval);
}

module_param_call(charging_enabled, zte_misc_control_charging,
		  zte_misc_get_charging_enabled_node,
		  &zte_misc_charging_enabled, 0644);


/*************************************************
haptics detector
bit(0): is_haptics 1=haptics 0=normal
**************************************************/
static uint32_t *pm_parameters = NULL;
static uint32_t is_haptics = 0;
module_param(is_haptics, uint, 0644);
static void zte_misc_parse_imem(void)
{
	struct device_node *np;
	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-pm_parameter");
	if (!np) {
		pr_err("unable to find DT imem msm-imem-pm_parameter node\n");
	}else {
		pm_parameters =(uint32_t *)of_iomap(np, 0);
		if (!pm_parameters)
			pr_err("unable to map imem [pm_parameter]\n");
		else
			is_haptics = (*pm_parameters) & 0x1;
	}
	pr_info("is_haptics=%d\n", is_haptics);
}

bool is_haptics_zte(void)
{
	return 1;  //ZTE  msm8996 all haptic
	//return (is_haptics==1);
}

/*************************************************
fingerprint id detector
In MSM8996, Goodix and Synaptics.
Goodix id pin NO PULL;
Synaptics id pin PULL UP.
set id pin to pull up, then check the pin status:
0 for Goodix, 1 for Synaptics
**************************************************/

#define FINGERPRINT_HW_GOODIX	0
#define FINGERPRINT_HW_SYNAFP	1
static uint32_t fingerprint_hw = -1;
module_param(fingerprint_hw, uint, 0644);

static void zte_misc_fingerprint_hw_check(struct device *dev)
{
	struct device_node *np;
	uint32_t *buf = NULL;
	uint32_t board_id = 0;
	int force_hw = socinfo_get_fp_hw();

	np = of_find_compatible_node(NULL, NULL, "qcom,msm-imem-board-id");
	if (!np)
		pr_err("unable to find DT imem qcom,msm-imem-board-id\n");
	else {
		buf = (uint32_t *)of_iomap(np, 0);
		if (!buf)
			pr_err("unable to map imem [board-id]\n");
		else
			board_id = *buf;
	}

	pr_info("board_id = %d\n", board_id);

	if (force_hw != -1) {
		fingerprint_hw = force_hw;
		pr_info("force set fingerprint_hw = %d\n", force_hw);
		goto info;
	}

	if (board_id < 3) {
		fingerprint_hw = FINGERPRINT_HW_SYNAFP;
	} else {
		fingerprint_hw = FINGERPRINT_HW_GOODIX;
	}
info:
	pr_info("fingerprint_hw = %d %s\n", fingerprint_hw, 
		(fingerprint_hw == FINGERPRINT_HW_SYNAFP ?
		 "SYNAFP" : "GOODIX"));
}

bool is_goodix_fp(void)
{
	return fingerprint_hw == FINGERPRINT_HW_GOODIX;
}

bool is_synafp_fp(void)
{
	return fingerprint_hw == FINGERPRINT_HW_SYNAFP;
}

/*************************************************
 * Fingerprint Status Updater
 *************************************************/
//The same as the definitions in ap\hardware\libhardware\include\hardware\Fingerprint.h
typedef enum fingerprint_msg_type {
	FINGERPRINT_ERROR = -1,
	FINGERPRINT_ACQUIRED = 1,
	FINGERPRINT_TEMPLATE_ENROLLING = 3,
	FINGERPRINT_TEMPLATE_REMOVED = 4,
	FINGERPRINT_AUTHENTICATED = 5,
	FINGERPRINT_DETECTED = 6,
	FINGERPRINT_REMOVED = 7,
} fingerprint_msg_type_t;

//fp_msg_disable: set to 1 to disable lcd accelerator, for debug
static int fp_msg_disable = 0;
static int fp_msg_type = FINGERPRINT_ERROR;

extern void fb_blank_update_oem(void);//defined in drivers\video\fbdev\core\fbmem.c
static int fp_msg_type_set(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	pr_info("DBG set fp_msg_type to %d, fp_msg_disable=%d\n",
		fp_msg_type, fp_msg_disable);

	if (fp_msg_disable == 1)
		return 0;

	if (fp_msg_type == FINGERPRINT_DETECTED) {
		pr_info("DBG fp_msg_set acquired\n");
		fb_blank_update_oem();
	}

	return 0;
}
module_param_call(fp_msg_type, fp_msg_type_set, param_get_int,
		  &fp_msg_type, 0644);
module_param(fp_msg_disable, int, 0644);

static int zte_misc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int error;

	pr_info("+++++\n");

	error = get_devtree_pdata(dev);
	if (error)
		return error;

	zte_misc_parse_imem();
	zte_misc_fingerprint_hw_check(dev);

	pr_info("----\n");
	return 0;
}

static int zte_misc_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver zte_misc_device_driver = {
	.probe		= zte_misc_probe,
	.remove		= zte_misc_remove,
	.driver		= {
		.name	= "zte-misc",
		.owner	= THIS_MODULE,
		.of_match_table = zte_misc_of_match,
	}
};

int __init zte_misc_init(void)
{
	return platform_driver_register(&zte_misc_device_driver);
}

static void __exit zte_misc_exit(void)
{
	platform_driver_unregister(&zte_misc_device_driver);
}

fs_initcall(zte_misc_init);
module_exit(zte_misc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Misc driver for ZTE");
MODULE_ALIAS("platform:zte-misc");
