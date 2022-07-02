/*
 * Driver for the SGM sgm41510 charger.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/power/charger-manager.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/usb/phy.h>
#include <uapi/linux/usb/charger.h>

#define SGM41510_REG_0				0x0
#define SGM41510_REG_1				0x1
#define SGM41510_REG_2				0x2
#define SGM41510_REG_3				0x3
#define SGM41510_REG_4				0x4
#define SGM41510_REG_5				0x5
#define SGM41510_REG_6				0x6
#define SGM41510_REG_7				0x7
#define SGM41510_REG_8				0x8
#define SGM41510_REG_9				0x7
#define SGM41510_REG_A				0xa
#define SGM41510_REG_B				0xb
#define SGM41510_REG_C				0xc
#define SGM41510_REG_D				0xd
#define SGM41510_REG_E				0xe
#define SGM41510_REG_F				0xf
#define SGM41510_REG_10				0x10
#define SGM41510_REG_11				0x11
#define SGM41510_REG_12				0x12
#define SGM41510_REG_13				0x13
#define SGM41510_REG_14				0x14
#define SGM41510_REG_15				0x15

#define SGM41510_BATTERY_NAME			"sc27xx-fgu"
#define BIT_DP_DM_BC_ENB			BIT(0)

/* iindpm current  REG0*/
#define SGM4154x_IINDPM_I_MASK		GENMASK(5, 0)
#define SGM4154x_IINDPM_I_MIN_uA	100000
#define SGM4154x_IINDPM_I_MAX_uA	4900000
#define SGM4154x_IINDPM_STEP_uA	    100000
#define SGM4154x_IINDPM_DEF_uA	    500000

#define SGM41510_REG_EN_HIZ_MASK	GENMASK(7, 7)

/*  REG3*/
#define SGM41510_REG_WATCHDOG_MASK		GENMASK(6, 6)
#define SGM41510_REG_OTG_MASK			GENMASK(5, 5)

/* charge current  REG4*/
#define SGM4154x_ICHRG_I_MASK		GENMASK(6, 0)
#define SGM4154x_ICHRG_I_MIN_uA		0
#define SGM4154x_ICHRG_I_MAX_uA		5056000
#define SGM4154x_ICHRG_I_STEP_uA	64000
#define SGM4154x_ICHRG_I_DEF_uA		2048000

/* termination current  REG5*/
#define SGM4154x_TERMCHRG_I_MASK	GENMASK(3, 0)
#define SGM4154x_TERMCHRG_I_STEP_uA	64000
#define SGM4154x_TERMCHRG_I_MIN_uA	64000
#define SGM4154x_TERMCHRG_I_MAX_uA	1024000
#define SGM4154x_TERMCHRG_I_DEF_uA	256000

/* charge voltage  REG6*/
#define SGM41510_VREG_V_MASK		GENMASK(7, 2)
#define SGM41510_VREG_V_MAX_uV	    4608000
#define SGM41510_VREG_V_MIN_uV	    3840000
#define SGM41510_VREG_V_STEP_uV	    16000
#define SGM41510_VREG_V_DEF_uV	    4208000
#define SGM41510_VREG_SHIFT	        2

/* charge voltage  REG7*/
#define SGM41510_REG_EN_TIMER	GENMASK(3, 3)

/* vindpm voltage  REGD*/
#define SGM41510_VINDPM_V_MASK      GENMASK(6, 0)
#define SGM41510_VINDPM_V_MIN_uV    2600000
#define SGM41510_VINDPM_V_MAX_uV    15300000
#define SGM41510_VINDPM_STEP_uV     100000
#define SGM41510_VINDPM_DEF_uV	    4400000

/*  REG14*/
#define SGM41510_REG_RESET_MASK			GENMASK(7, 7)

/* over voltage  REG15*/
#define SGM41510_REG_OVP_MASK		GENMASK(4, 2)
#define SGM41510_OVP_V_MIN_uV       10300000
#define SGM41510_OVP_V_MAX_uV       15100000
#define SGM41510_OVP_STEP_uV        1600000
#define SGM41510_OVP_DEF_uV	        15100000
#define SGM41510_REG_OVP_SHIFT			2


#define SGM41510_DISABLE_PIN_MASK_2730		BIT(0)
#define SGM41510_DISABLE_PIN_MASK_2721		BIT(15)
#define SGM41510_DISABLE_PIN_MASK_2720		BIT(0)

#define SGM41510_OTG_VALID_MS			500
#define SGM41510_FEED_WATCHDOG_VALID_MS		50
#define SGM41510_OTG_RETRY_TIMES			10


#define SGM41510_ROLE_MASTER_DEFAULT		1
#define SGM41510_ROLE_SLAVE			2


#define SGM41510_FAST_CHARGER_VOLTAGE_MAX	10500000
#define SGM41510_NORMAL_CHARGER_VOLTAGE_MAX	6500000

struct sgm41510_charger_info {
	struct i2c_client *client;
	struct device *dev;
	struct usb_phy *usb_phy;
	struct notifier_block usb_notify;
	struct power_supply *psy_usb;
	struct power_supply_charge_current cur;
	struct work_struct work;
	struct mutex lock;
	bool charging;
	u32 limit;
	struct delayed_work otg_work;
	struct delayed_work wdt_work;
	struct regmap *pmic;
	u32 charger_detect;
	u32 charger_pd;
	u32 charger_pd_mask;
	struct gpio_desc *gpiod;
	struct extcon_dev *edev;
	u32 last_limit_current;
	u32 role;
	bool need_disable_Q1;
	int termination_cur;
};

static bool vddvbus_registered = false ;

static int
sgm41510_charger_set_limit_current(struct sgm41510_charger_info *info,
				  u32 limit_cur);

static bool sgm41510_charger_is_bat_present(struct sgm41510_charger_info *info)
{
	struct power_supply *psy;
	union power_supply_propval val;
	bool present = false;
	int ret;

	psy = power_supply_get_by_name(SGM41510_BATTERY_NAME);
	if (!psy) {
		dev_err(info->dev, "Failed to get psy of sc27xx_fgu\n");
		return present;
	}
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_PRESENT,
					&val);
	if (ret == 0 && val.intval)
		present = true;
	power_supply_put(psy);

	if (ret)
		dev_err(info->dev,
			"Failed to get property of present:%d\n", ret);

	return present;
}

static int sgm41510_read(struct sgm41510_charger_info *info, u8 reg, u8 *data)
{
	int ret;

	ret = i2c_smbus_read_byte_data(info->client, reg);
	if (ret < 0)
		return ret;

	*data = ret;
	return 0;
}

static int sgm41510_write(struct sgm41510_charger_info *info, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(info->client, reg, data);
}

static int sgm41510_update_bits(struct sgm41510_charger_info *info, u8 reg,
			       u8 mask, u8 data)
{
	u8 v;
	int ret;

	ret = sgm41510_read(info, reg, &v);
	if (ret < 0)
		return ret;

	v &= ~mask;
	v |= (data & mask);

	return sgm41510_write(info, reg, v);
}

static int
sgm41510_charger_set_vindpm(struct sgm41510_charger_info *info, u32 vol)
{
	u8 reg_val;

	if (vol <= 3900)
		vol = 3900;
	else if (vol > SGM41510_VINDPM_V_MAX_uV)
		vol = SGM41510_VINDPM_V_MAX_uV;
	
	reg_val = (vol - SGM41510_VINDPM_V_MIN_uV) / SGM41510_VINDPM_STEP_uV;

	return sgm41510_update_bits(info, SGM41510_REG_D,
				   SGM41510_VINDPM_V_MASK, reg_val);
}

static int
sgm41510_charger_set_ovp(struct sgm41510_charger_info *info, u32 vol)
{
	u8 reg_val;

	if (vol <= SGM41510_OVP_V_MIN_uV)
		vol = SGM41510_OVP_V_MIN_uV;
	else if (vol > SGM41510_OVP_V_MAX_uV)
		vol = SGM41510_VINDPM_V_MAX_uV;
	
	reg_val = (vol - SGM41510_OVP_V_MIN_uV) / SGM41510_OVP_STEP_uV;
	
	return sgm41510_update_bits(info, SGM41510_REG_15,
				   SGM41510_REG_OVP_MASK,
				   reg_val << SGM41510_REG_OVP_SHIFT);
}

static int
sgm41510_charger_set_termina_vol(struct sgm41510_charger_info *info, u32 vol)
{
	u8 reg_val;

	if (vol <= SGM41510_VREG_V_MIN_uV)
		vol = SGM41510_VREG_V_MIN_uV;
	else if (vol > SGM41510_VREG_V_MAX_uV)
		vol = SGM41510_VREG_V_MAX_uV;
	
	reg_val = (vol - SGM41510_VREG_V_MIN_uV) / SGM41510_VREG_V_STEP_uV;

	dev_info(info->dev,"sprocomm sgm41510_charger_set_termina_vol target vol=%d, reg_val=0x%x", vol, reg_val);

	return sgm41510_update_bits(info, SGM41510_REG_6,
				   SGM41510_VREG_V_MASK,
				   reg_val << SGM41510_VREG_SHIFT);
}

static int
sgm41510_charger_set_termina_cur(struct sgm41510_charger_info *info, u32 cur)
{
	u8 reg_val;

	if (cur <= SGM4154x_TERMCHRG_I_MIN_uA)
		cur = SGM4154x_TERMCHRG_I_MIN_uA;
	else if (cur > SGM4154x_TERMCHRG_I_MAX_uA)
		cur = SGM4154x_TERMCHRG_I_MAX_uA;
	
	reg_val = (cur - SGM4154x_TERMCHRG_I_MIN_uA) / SGM4154x_TERMCHRG_I_STEP_uA;
	
	dev_info(info->dev,"sprocomm sgm41510_charger_set_termina_cur target cur=%d, reg_val=0x%x", cur, reg_val);
	return sgm41510_update_bits(info, SGM41510_REG_5,
				   SGM4154x_TERMCHRG_I_MASK,
				   reg_val);
}

static int sgm41510_charger_hw_init(struct sgm41510_charger_info *info)
{
	struct power_supply_battery_info bat_info = { };
	int voltage_max_microvolt, current_max_ua, current_term_ua;
	int ret;

	ret = power_supply_get_battery_info(info->psy_usb, &bat_info, 0);
	if (ret) {
		dev_warn(info->dev, "no battery information is supplied\n");

		/*
		 * If no battery information is supplied, we should set
		 * default charge termination current to 100 mA, and default
		 * charge termination voltage to 4.2V.
		 */
		info->cur.sdp_limit = 500000;
		info->cur.sdp_cur = 500000;
		info->cur.dcp_limit = 5000000;
		info->cur.dcp_cur = 500000;
		info->cur.cdp_limit = 5000000;
		info->cur.cdp_cur = 1500000;
		info->cur.unknown_limit = 5000000;
		info->cur.unknown_cur = 500000;
	} else {
		info->cur.sdp_limit = bat_info.cur.sdp_limit;
		info->cur.sdp_cur = bat_info.cur.sdp_cur;
		info->cur.dcp_limit = bat_info.cur.dcp_limit;
		info->cur.dcp_cur = bat_info.cur.dcp_cur;
		info->cur.cdp_limit = bat_info.cur.cdp_limit;
		info->cur.cdp_cur = bat_info.cur.cdp_cur;
		info->cur.unknown_limit = bat_info.cur.unknown_limit;
		info->cur.unknown_cur = bat_info.cur.unknown_cur;
		info->cur.fchg_limit = bat_info.cur.fchg_limit;
		info->cur.fchg_cur = bat_info.cur.fchg_cur;

		voltage_max_microvolt =
			bat_info.constant_charge_voltage_max_uv;
		current_max_ua = bat_info.constant_charge_current_max_ua;
		current_term_ua = bat_info.charge_term_current_ua;
		info->termination_cur = bat_info.charge_term_current_ua;
		
		power_supply_put_battery_info(info->psy_usb, &bat_info);

		ret = sgm41510_update_bits(info, SGM41510_REG_14,
					  SGM41510_REG_RESET_MASK,
					  SGM41510_REG_RESET_MASK);
		if (ret) {
			dev_err(info->dev, "reset sgm41510 failed\n");
			return ret;
		}

		if (info->role == SGM41510_ROLE_MASTER_DEFAULT) {
			ret = sgm41510_charger_set_ovp(info, SGM41510_OVP_V_MIN_uV);
			if (ret) {
				dev_err(info->dev, "set sgm41510 ovp failed\n");
				return ret;
			}
		} else if (info->role == SGM41510_ROLE_SLAVE) {
			ret = sgm41510_charger_set_ovp(info, SGM41510_OVP_V_MIN_uV);
			if (ret) {
				dev_err(info->dev, "set sgm41510 slave ovp failed\n");
				return ret;
			}
		}

		ret = sgm41510_charger_set_vindpm(info, voltage_max_microvolt);
		if (ret) {
			dev_err(info->dev, "set sgm41510 vindpm vol failed\n");
			return ret;
		}

		ret = sgm41510_charger_set_termina_vol(info,
						      voltage_max_microvolt);
		if (ret) {
			dev_err(info->dev, "set sgm41510 terminal vol failed\n");
			return ret;
		}

		ret = sgm41510_charger_set_termina_cur(info, current_term_ua);
		if (ret) {
			dev_err(info->dev, "set sgm41510 terminal cur failed\n");
			return ret;
		}

		ret = sgm41510_charger_set_limit_current(info,
							info->cur.unknown_cur);
		if (ret)
			dev_err(info->dev, "set sgm41510 limit current failed\n");
	}

	return ret;
}

static int
sgm41510_charger_get_charge_voltage(struct sgm41510_charger_info *info,
				   u32 *charge_vol)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = power_supply_get_by_name(SGM41510_BATTERY_NAME);
	if (!psy) {
		dev_err(info->dev, "failed to get SGM41510_BATTERY_NAME\n");
		return -ENODEV;
	}

	ret = power_supply_get_property(psy,
					POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
					&val);
	power_supply_put(psy);
	if (ret) {
		dev_err(info->dev, "failed to get CONSTANT_CHARGE_VOLTAGE\n");
		return ret;
	}

	*charge_vol = val.intval;

	return 0;
}

static int sgm41510_charger_start_charge(struct sgm41510_charger_info *info)
{
	int ret;
	printk("%s, enter\n",__func__);
	ret = sgm41510_update_bits(info, SGM41510_REG_0,
				  SGM41510_REG_EN_HIZ_MASK, 0);
	if (ret)
		dev_err(info->dev, "disable HIZ mode failed\n");

	if (info->role == SGM41510_ROLE_MASTER_DEFAULT) {
		ret = regmap_update_bits(info->pmic, info->charger_pd,
					 info->charger_pd_mask, 0);
		if (ret) {
			dev_err(info->dev, "enable sgm41510 charge failed\n");
			return ret;
		}
	} else if (info->role == SGM41510_ROLE_SLAVE) {
		gpiod_set_value_cansleep(info->gpiod, 0);
	}

	ret = sgm41510_charger_set_limit_current(info,
						info->last_limit_current);
	if (ret)
		dev_err(info->dev, "failed to set limit current\n");
	
	
	ret = sgm41510_charger_set_termina_cur(info, info->termination_cur);
	if(ret)
		dev_err(info->dev, "failed to set terminal current\n");

	ret = sgm41510_update_bits(info,SGM41510_REG_7,
				  SGM41510_REG_EN_TIMER, 0);
	if (ret)
		dev_err(info->dev, "failed to set disable Charge Safety Timer\n");

	return ret;
}

static void sgm41510_charger_stop_charge(struct sgm41510_charger_info *info)
{
	int ret;
	bool present = sgm41510_charger_is_bat_present(info);
	
	printk("%s enter \n",__func__);

	if (info->role == SGM41510_ROLE_MASTER_DEFAULT) {
		if (!present || info->need_disable_Q1) {
			ret = sgm41510_update_bits(info, SGM41510_REG_0,
						  SGM41510_REG_EN_HIZ_MASK,
						  SGM41510_REG_EN_HIZ_MASK);
			if (ret)
				dev_err(info->dev, "enable HIZ mode failed\n");

			info->need_disable_Q1 = false;
		}

		ret = regmap_update_bits(info->pmic, info->charger_pd,
					 info->charger_pd_mask,
					 info->charger_pd_mask);
		if (ret)
			dev_err(info->dev, "disable sgm41510 charge failed\n");
	} else if (info->role == SGM41510_ROLE_SLAVE) {
		ret = sgm41510_update_bits(info, SGM41510_REG_0,
					  SGM41510_REG_EN_HIZ_MASK,
					  SGM41510_REG_EN_HIZ_MASK);
		if (ret)
			dev_err(info->dev, "enable HIZ mode failed\n");

		gpiod_set_value_cansleep(info->gpiod, 1);
	}
}

static int sgm41510_charger_set_current(struct sgm41510_charger_info *info,
				       u32 cur)
{
	u8 reg_val;

	printk("%s enter cut:%d\n",__func__,cur);
	if (cur >= SGM4154x_ICHRG_I_MAX_uA)
		cur = SGM4154x_ICHRG_I_MAX_uA;
	
	reg_val = (cur - SGM4154x_ICHRG_I_MIN_uA)/SGM4154x_ICHRG_I_STEP_uA;
	
	return sgm41510_update_bits(info, SGM41510_REG_4,
				   SGM4154x_ICHRG_I_MASK,
				   reg_val);
}

static int sgm41510_charger_get_current(struct sgm41510_charger_info *info,
				       u32 *cur)
{
	u8 reg_val;
	int ret;

	ret = sgm41510_read(info, SGM41510_REG_4, &reg_val);
	if (ret < 0)
		return ret;

	reg_val &= SGM4154x_ICHRG_I_MASK;
	*cur = reg_val * SGM4154x_ICHRG_I_STEP_uA + SGM4154x_ICHRG_I_MIN_uA;
	
	if (*cur >= SGM4154x_ICHRG_I_MAX_uA)
		*cur = SGM4154x_ICHRG_I_MAX_uA;

	return 0;
}

static int
sgm41510_charger_set_limit_current(struct sgm41510_charger_info *info,
				  u32 limit_cur)
{
	u8 reg_val;
	int ret;

	if (limit_cur >= SGM4154x_IINDPM_I_MAX_uA)
		limit_cur = SGM4154x_IINDPM_I_MAX_uA;	
	
	reg_val = (limit_cur - SGM4154x_IINDPM_I_MIN_uA)/SGM4154x_IINDPM_STEP_uA;

	ret = sgm41510_update_bits(info, SGM41510_REG_0,
				  SGM4154x_IINDPM_I_MASK,reg_val);
	if (ret)
	{
		dev_err(info->dev, "set sgm41510 limit cur failed\n");
		return ret;
	}
		
	info->last_limit_current = limit_cur;
	return ret;
}

static u32
sgm41510_charger_get_limit_current(struct sgm41510_charger_info *info,
				  u32 *limit_cur)
{
	u8 reg_val;
	int ret;

	ret = sgm41510_read(info, SGM41510_REG_0, &reg_val);
	if (ret < 0)
		return ret;

	reg_val &= SGM4154x_IINDPM_I_MASK;
	*limit_cur = reg_val * SGM4154x_IINDPM_STEP_uA + SGM4154x_IINDPM_I_MIN_uA;
	
	if (*limit_cur >= SGM4154x_IINDPM_I_MAX_uA)
		*limit_cur = SGM4154x_IINDPM_I_MAX_uA;

	return 0;
}

static int sgm41510_charger_get_health(struct sgm41510_charger_info *info,
				      u32 *health)
{
	*health = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int sgm41510_charger_get_online(struct sgm41510_charger_info *info,
				      u32 *online)
{
	if (info->limit)
		*online = true;
	else
		*online = false;

	return 0;
}

static int sgm41510_charger_feed_watchdog(struct sgm41510_charger_info *info,
					 u32 val)
{
	int i = 0;
	u8 reg = 0;
	int ret = 0;

#if 0
	ret = sgm41510_update_bits(info, SGM41510_REG_14,
				  SGM41510_REG_RESET_MASK,
				  SGM41510_REG_RESET_MASK);
#endif	
	ret = sgm41510_update_bits(info, SGM41510_REG_3,
				  SGM41510_REG_WATCHDOG_MASK,
				  SGM41510_REG_WATCHDOG_MASK);
			  
	for(i=0; i<=SGM41510_REG_15; i++) {
		sgm41510_read(info, i, &reg);
		dev_info(info->dev, "%s REG%X  0x%X\n", __func__, i, reg);
	}

	return ret;
}

static int sgm41510_charger_set_fchg_current(struct sgm41510_charger_info *info,
					    u32 val)
{
	int ret, limit_cur, cur;

	if (val == CM_FAST_CHARGE_ENABLE_CMD) {
		limit_cur = info->cur.fchg_limit;
		cur = info->cur.fchg_cur;
	} else if (val == CM_FAST_CHARGE_DISABLE_CMD) {
		limit_cur = info->cur.dcp_limit;
		cur = info->cur.dcp_cur;
	} else {
		return 0;
	}

	ret = sgm41510_charger_set_limit_current(info, limit_cur);
	if (ret) {
		dev_err(info->dev, "failed to set fchg limit current\n");
		return ret;
	}

	ret = sgm41510_charger_set_current(info, cur);
	if (ret) {
		dev_err(info->dev, "failed to set fchg current\n");
		return ret;
	}

	return 0;
}

static int sgm41510_charger_get_status(struct sgm41510_charger_info *info)
{
	if (info->charging)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int sgm41510_charger_set_status(struct sgm41510_charger_info *info,
				      int val)
{
	int ret = 0;
	u32 input_vol;

	if (val == CM_FAST_CHARGE_ENABLE_CMD) {
		ret = sgm41510_charger_set_fchg_current(info, val);
		if (ret) {
			dev_err(info->dev, "failed to set 9V fast charge current\n");
			return ret;
		}
		ret = sgm41510_charger_set_ovp(info, SGM41510_OVP_V_MIN_uV);// customer define
		if (ret) {
			dev_err(info->dev, "failed to set fast charge 9V ovp\n");
			return ret;
		}
	} else if (val == CM_FAST_CHARGE_DISABLE_CMD) {
		ret = sgm41510_charger_set_fchg_current(info, val);
		if (ret) {
			dev_err(info->dev, "failed to set 5V normal charge current\n");
			return ret;
		}
		ret = sgm41510_charger_set_ovp(info, SGM41510_OVP_V_MIN_uV);// customer define
		if (ret) {
			dev_err(info->dev, "failed to set fast charge V ovp\n");
			return ret;
		}
		if (info->role == SGM41510_ROLE_MASTER_DEFAULT) {
			ret = sgm41510_charger_get_charge_voltage(info, &input_vol);
			if (ret) {
				dev_err(info->dev, "failed to get V charge voltage\n");
				return ret;
			}
			if (input_vol > SGM41510_FAST_CHARGER_VOLTAGE_MAX)
				info->need_disable_Q1 = true;
		}
	} else if ((val == false) &&
		   (info->role == SGM41510_ROLE_MASTER_DEFAULT)) {
		ret = sgm41510_charger_get_charge_voltage(info, &input_vol);
		if (ret) {
			dev_err(info->dev, "failed to get 5V charge voltage\n");
			return ret;
		}
		if (input_vol > SGM41510_NORMAL_CHARGER_VOLTAGE_MAX)
			info->need_disable_Q1 = true;
	}

	if (val > CM_FAST_CHARGE_NORMAL_CMD)
		return 0;

	if (!val && info->charging) {
		sgm41510_charger_stop_charge(info);
		info->charging = false;
	} else if (val && !info->charging) {
		ret = sgm41510_charger_start_charge(info);
		if (ret)
			dev_err(info->dev, "start charge failed\n");
		else
			info->charging = true;
	}

	return ret;
}

static void sgm41510_charger_work(struct work_struct *data)
{
	struct sgm41510_charger_info *info =
		container_of(data, struct sgm41510_charger_info, work);
	bool present = sgm41510_charger_is_bat_present(info);

	dev_info(info->dev, "battery present = %d, charger type = %d\n",
		 present, info->usb_phy->chg_type);
	cm_notify_event(info->psy_usb, CM_EVENT_CHG_START_STOP, NULL);
}


static int sgm41510_charger_usb_change(struct notifier_block *nb,
				      unsigned long limit, void *data)
{
	struct sgm41510_charger_info *info =
		container_of(nb, struct sgm41510_charger_info, usb_notify);

	info->limit = limit;
	printk("%s enter\n",__func__);
	/*
	 * only master should do work when vbus change.
	 * let info->limit = limit, slave will online, too.
	 */
	if (info->role == SGM41510_ROLE_SLAVE)
		return NOTIFY_OK;

	schedule_work(&info->work);
	return NOTIFY_OK;
}

static int sgm41510_charger_usb_get_property(struct power_supply *psy,
					    enum power_supply_property psp,
					    union power_supply_propval *val)
{
	struct sgm41510_charger_info *info = power_supply_get_drvdata(psy);
	u32 cur, online, health, enabled = 0;
	enum usb_charger_type type;
	int ret = 0;

	mutex_lock(&info->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (info->limit)
			val->intval = sgm41510_charger_get_status(info);
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (!info->charging) {
			val->intval = 0;
		} else {
			ret = sgm41510_charger_get_current(info, &cur);
			if (ret)
				goto out;

			val->intval = cur;
		}
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (!info->charging) {
			val->intval = 0;
		} else {
			ret = sgm41510_charger_get_limit_current(info, &cur);
			if (ret)
				goto out;

			val->intval = cur;
		}
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		ret = sgm41510_charger_get_online(info, &online);
		if (ret)
			goto out;

		val->intval = online;

		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (info->charging) {
			val->intval = 0;
		} else {
			ret = sgm41510_charger_get_health(info, &health);
			if (ret)
				goto out;

			val->intval = health;
		}
		break;

	case POWER_SUPPLY_PROP_USB_TYPE:
		type = info->usb_phy->chg_type;

		switch (type) {
		case SDP_TYPE:
			val->intval = POWER_SUPPLY_USB_TYPE_SDP;
			break;

		case DCP_TYPE:
			val->intval = POWER_SUPPLY_USB_TYPE_DCP;
			break;

		case CDP_TYPE:
			val->intval = POWER_SUPPLY_USB_TYPE_CDP;
			break;

		default:
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		}

		break;

	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		if (info->role == SGM41510_ROLE_MASTER_DEFAULT) {
			ret = regmap_read(info->pmic, info->charger_pd, &enabled);
			if (ret) {
				dev_err(info->dev, "get sgm41510 charge status failed\n");
				goto out;
			}
		} else if (info->role == SGM41510_ROLE_SLAVE) {
			enabled = gpiod_get_value_cansleep(info->gpiod);
		}

		val->intval = !enabled;
		break;
	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&info->lock);
	return ret;
}

static int sgm41510_charger_usb_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct sgm41510_charger_info *info = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&info->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sgm41510_charger_set_current(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set charge current failed\n");
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sgm41510_charger_set_limit_current(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set input current limit failed\n");
		break;

	case POWER_SUPPLY_PROP_STATUS:
		ret = sgm41510_charger_set_status(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set charge status failed\n");
		break;

	case POWER_SUPPLY_PROP_FEED_WATCHDOG:
		ret = sgm41510_charger_feed_watchdog(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "feed charger watchdog failed\n");
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = sgm41510_charger_set_termina_vol(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "failed to set terminate voltage\n");
		break;
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		if(val->intval == true){
			ret = sgm41510_charger_start_charge(info);
			if(ret)
				dev_err(info->dev,"start charge  failed\n");
		}else if(val->intval == false){
			sgm41510_charger_stop_charge(info);
		}
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&info->lock);
	return ret;
}

static int sgm41510_charger_property_is_writeable(struct power_supply *psy,
						 enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
	case POWER_SUPPLY_PROP_STATUS:
		ret = 1;
		break;

	default:
		ret = 0;
	}

	return ret;
}

static enum power_supply_usb_type sgm41510_charger_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID
};

static enum power_supply_property sgm41510_usb_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
};

static const struct power_supply_desc sgm41510_charger_desc = {
	.name			= "sgm41510_charger",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= sgm41510_usb_props,
	.num_properties		= ARRAY_SIZE(sgm41510_usb_props),
	.get_property		= sgm41510_charger_usb_get_property,
	.set_property		= sgm41510_charger_usb_set_property,
	.property_is_writeable	= sgm41510_charger_property_is_writeable,
	.usb_types		= sgm41510_charger_usb_types,
	.num_usb_types		= ARRAY_SIZE(sgm41510_charger_usb_types),
};

static const struct power_supply_desc sgm41510_slave_charger_desc = {
	.name			= "sgm41510_slave_charger",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= sgm41510_usb_props,
	.num_properties		= ARRAY_SIZE(sgm41510_usb_props),
	.get_property		= sgm41510_charger_usb_get_property,
	.set_property		= sgm41510_charger_usb_set_property,
	.property_is_writeable	= sgm41510_charger_property_is_writeable,
	.usb_types		= sgm41510_charger_usb_types,
	.num_usb_types		= ARRAY_SIZE(sgm41510_charger_usb_types),
};

static void sgm41510_charger_detect_status(struct sgm41510_charger_info *info)
{
	unsigned int min, max;

	/*
	 * If the USB charger status has been USB_CHARGER_PRESENT before
	 * registering the notifier, we should start to charge with getting
	 * the charge current.
	 */
	if (info->usb_phy->chg_state != USB_CHARGER_PRESENT)
		return;

	usb_phy_get_charger_current(info->usb_phy, &min, &max);
	info->limit = min;

	/*
	 * slave no need to start charge when vbus change.
	 * due to charging in shut down will check each psy
	 * whether online or not, so let info->limit = min.
	 */
	if (info->role == SGM41510_ROLE_SLAVE)
		return;
	schedule_work(&info->work);
}

static void
sgm41510_charger_feed_watchdog_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sgm41510_charger_info *info = container_of(dwork,
							 struct sgm41510_charger_info,
							 wdt_work);
	int ret;

	printk("%s enter\n",__func__);
	ret = sgm41510_update_bits(info, SGM41510_REG_3,
				  SGM41510_REG_WATCHDOG_MASK,
				  SGM41510_REG_WATCHDOG_MASK);
	if (ret) {
		dev_err(info->dev, "reset sgm41510 failed\n");
		return;
	}
	schedule_delayed_work(&info->wdt_work, HZ * 15);
}

#ifdef CONFIG_REGULATOR
static void sgm41510_charger_otg_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct sgm41510_charger_info *info = container_of(dwork,
			struct sgm41510_charger_info, otg_work);
	bool otg_valid = extcon_get_state(info->edev, EXTCON_USB);
	int ret, retry = 0;

	if (otg_valid)
		goto out;

	do {
		ret = sgm41510_update_bits(info, SGM41510_REG_3,
					  SGM41510_REG_OTG_MASK,
					  SGM41510_REG_OTG_MASK);
		if (ret)
			dev_err(info->dev, "restart sgm41510 charger otg failed\n");

		otg_valid = extcon_get_state(info->edev, EXTCON_USB);
	} while (!otg_valid && retry++ < SGM41510_OTG_RETRY_TIMES);

	if (retry >= SGM41510_OTG_RETRY_TIMES) {
		dev_err(info->dev, "Restart OTG failed\n");
		return;
	}

out:
	schedule_delayed_work(&info->otg_work, msecs_to_jiffies(500));
}

static int sgm41510_charger_enable_otg(struct regulator_dev *dev)
{
	struct sgm41510_charger_info *info = rdev_get_drvdata(dev);
	int ret;

	dev_info(info->dev,"sgm41510_charger_enable_otg");
	/*
	 * Disable charger detection function in case
	 * affecting the OTG timing sequence.
	 */
	ret = regmap_update_bits(info->pmic, info->charger_detect,
				 BIT_DP_DM_BC_ENB, BIT_DP_DM_BC_ENB);
	if (ret) {
		dev_err(info->dev, "failed to disable bc1.2 detect function.\n");
		return ret;
	}

	ret = sgm41510_update_bits(info, SGM41510_REG_3,
				  SGM41510_REG_OTG_MASK,
				  SGM41510_REG_OTG_MASK);
	if (ret) {
		dev_err(info->dev, "enable sgm41510 otg failed\n");
		regmap_update_bits(info->pmic, info->charger_detect,
				   BIT_DP_DM_BC_ENB, 0);
		return ret;
	}

	schedule_delayed_work(&info->wdt_work,
			      msecs_to_jiffies(SGM41510_FEED_WATCHDOG_VALID_MS));
	schedule_delayed_work(&info->otg_work,
			      msecs_to_jiffies(SGM41510_OTG_VALID_MS));

	return 0;
}

static int sgm41510_charger_disable_otg(struct regulator_dev *dev)
{
	struct sgm41510_charger_info *info = rdev_get_drvdata(dev);
	int ret;

	dev_info(info->dev,"sgm41510_charger_disable_otg");

	cancel_delayed_work_sync(&info->wdt_work);
	cancel_delayed_work_sync(&info->otg_work);
	ret = sgm41510_update_bits(info, SGM41510_REG_3,
				  SGM41510_REG_OTG_MASK,
				  0);
	if (ret) {
		dev_err(info->dev, "disable sgm41510 otg failed\n");
		return ret;
	}

	/* Enable charger detection function to identify the charger type */
	return regmap_update_bits(info->pmic, info->charger_detect,
				  BIT_DP_DM_BC_ENB, 0);
}

static int sgm41510_charger_vbus_is_enabled(struct regulator_dev *dev)
{
	struct sgm41510_charger_info *info = rdev_get_drvdata(dev);
	int ret;
	u8 val;

	ret = sgm41510_read(info, SGM41510_REG_3, &val);
	if (ret) {
		dev_err(info->dev, "failed to get sgm41510 otg status\n");
		return ret;
	}

	val &= SGM41510_REG_OTG_MASK;

	dev_info(info->dev,"sgm41510_charger_vbus_is_enabled:%d", val);
	return val;
}

static const struct regulator_ops sgm41510_charger_vbus_ops = {
	.enable = sgm41510_charger_enable_otg,
	.disable = sgm41510_charger_disable_otg,
	.is_enabled = sgm41510_charger_vbus_is_enabled,
};

static const struct regulator_desc sgm41510_charger_vbus_desc = {
	.name = "otg-vbus",
	.of_match = "otg-vbus",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &sgm41510_charger_vbus_ops,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static int
sgm41510_charger_register_vbus_regulator(struct sgm41510_charger_info *info)
{
	struct regulator_config cfg = { };
	struct regulator_dev *reg;
	int ret = 0;

	if(vddvbus_registered)
		return ret;

	dev_info(info->dev,"sgm41510_charger_register_vbus_regulator");

	cfg.dev = info->dev;
	cfg.driver_data = info;
	reg = devm_regulator_register(info->dev,
				      &sgm41510_charger_vbus_desc, &cfg);
	if (IS_ERR(reg)) {
		ret = PTR_ERR(reg);
		dev_err(info->dev, "Can't register regulator:%d\n", ret);
	}

	vddvbus_registered = true;
	return ret;
}

#else
static int
sgm41510_charger_register_vbus_regulator(struct sgm41510_charger_info *info)
{
	return 0;
}
#endif

static int sgm41510_charger_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct power_supply_config charger_cfg = { };
	struct sgm41510_charger_info *info;
	struct device_node *regmap_np;
	struct platform_device *regmap_pdev;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "No support for SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}
	printk("%s enter\n",__func__);

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->client = client;
	info->dev = dev;
	mutex_init(&info->lock);
	INIT_WORK(&info->work, sgm41510_charger_work);

	ret = device_property_read_bool(dev, "role-slave");
	if (ret)
		info->role = SGM41510_ROLE_SLAVE;
	else
		info->role = SGM41510_ROLE_MASTER_DEFAULT;

	if (info->role == SGM41510_ROLE_SLAVE) {
		info->gpiod = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
		if (IS_ERR(info->gpiod)) {
			dev_err(dev, "failed to get enable gpio\n");
			return PTR_ERR(info->gpiod);
		}
	}

	info->usb_phy = devm_usb_get_phy_by_phandle(dev, "phys", 0);
	if (IS_ERR(info->usb_phy)) {
		dev_err(dev, "failed to find USB phy\n");
		return PTR_ERR(info->usb_phy);
	}

	info->edev = extcon_get_edev_by_phandle(info->dev, 0);
	if (IS_ERR(info->edev)) {
		dev_err(dev, "failed to find vbus extcon device.\n");
		return PTR_ERR(info->edev);
	}

	/*
	 * only master to support otg
	 */
	if (info->role == SGM41510_ROLE_MASTER_DEFAULT) {
		ret = sgm41510_charger_register_vbus_regulator(info);
		if (ret) {
			dev_err(dev, "failed to register vbus regulator.\n");
			return ret;
		}
	}

	regmap_np = of_find_compatible_node(NULL, NULL, "sprd,sc27xx-syscon");
	if (!regmap_np) {
		dev_err(dev, "unable to get syscon node\n");
		return -ENODEV;
	}

	ret = of_property_read_u32_index(regmap_np, "reg", 1,
					 &info->charger_detect);
	if (ret) {
		dev_err(dev, "failed to get charger_detect\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_index(regmap_np, "reg", 2,
					 &info->charger_pd);
	if (ret) {
		dev_err(dev, "failed to get charger_pd reg\n");
		return ret;
	}

	if (of_device_is_compatible(regmap_np->parent, "sprd,sc2730"))
		info->charger_pd_mask = SGM41510_DISABLE_PIN_MASK_2730;
	else if (of_device_is_compatible(regmap_np->parent, "sprd,sc2721"))
		info->charger_pd_mask = SGM41510_DISABLE_PIN_MASK_2721;
	else if (of_device_is_compatible(regmap_np->parent, "sprd,sc2720"))
		info->charger_pd_mask = SGM41510_DISABLE_PIN_MASK_2720;
	else {
		dev_err(dev, "failed to get charger_pd mask\n");
		return -EINVAL;
	}

	regmap_pdev = of_find_device_by_node(regmap_np);
	if (!regmap_pdev) {
		of_node_put(regmap_np);
		dev_err(dev, "unable to get syscon device\n");
		return -ENODEV;
	}

	of_node_put(regmap_np);
	info->pmic = dev_get_regmap(regmap_pdev->dev.parent, NULL);
	if (!info->pmic) {
		dev_err(dev, "unable to get pmic regmap device\n");
		return -ENODEV;
	}

	charger_cfg.drv_data = info;
	charger_cfg.of_node = dev->of_node;
	if (info->role == SGM41510_ROLE_MASTER_DEFAULT) {
		info->psy_usb = devm_power_supply_register(dev,
							   &sgm41510_charger_desc,
							   &charger_cfg);
	} else if (info->role == SGM41510_ROLE_SLAVE) {
		info->psy_usb = devm_power_supply_register(dev,
							   &sgm41510_slave_charger_desc,
							   &charger_cfg);
	}

	if (IS_ERR(info->psy_usb)) {
		dev_err(dev, "failed to register power supply\n");
		return PTR_ERR(info->psy_usb);
	}

	ret = sgm41510_charger_hw_init(info);
	if (ret)
		return ret;

	info->usb_notify.notifier_call = sgm41510_charger_usb_change;
	ret = usb_register_notifier(info->usb_phy, &info->usb_notify);
	if (ret) {
		dev_err(dev, "failed to register notifier:%d\n", ret);
		return ret;
	}

	sgm41510_charger_detect_status(info);
	INIT_DELAYED_WORK(&info->otg_work, sgm41510_charger_otg_work);
	INIT_DELAYED_WORK(&info->wdt_work,
			  sgm41510_charger_feed_watchdog_work);

	return 0;
}

static int sgm41510_charger_remove(struct i2c_client *client)
{
	struct sgm41510_charger_info *info = i2c_get_clientdata(client);

	usb_unregister_notifier(info->usb_phy, &info->usb_notify);

	return 0;
}

static const struct i2c_device_id sgm41510_i2c_id[] = {
	{"sgm41510_chg", 0},
	{}
};

static const struct of_device_id sgm41510_charger_of_match[] = {
	{ .compatible = "sgm,sgm41510_chg", },
	{ }
};

static const struct i2c_device_id sgm41510_slave_i2c_id[] = {
	{"sgm41510_slave_chg", 0},
	{}
};

static const struct of_device_id sgm41510_slave_charger_of_match[] = {
	{ .compatible = "sgm,sgm41510_slave_chg", },
	{ }
};

MODULE_DEVICE_TABLE(of, sgm41510_charger_of_match);
MODULE_DEVICE_TABLE(of, sgm41510_slave_charger_of_match);

static struct i2c_driver sgm41510_master_charger_driver = {
	.driver = {
		.name = "sgm41510_chg",
		.of_match_table = sgm41510_charger_of_match,
	},
	.probe = sgm41510_charger_probe,
	.remove = sgm41510_charger_remove,
	.id_table = sgm41510_i2c_id,
};

static struct i2c_driver sgm41510_slave_charger_driver = {
	.driver = {
		.name = "sgm41510_slave_chg",
		.of_match_table = sgm41510_slave_charger_of_match,
	},
	.probe = sgm41510_charger_probe,
	.remove = sgm41510_charger_remove,
	.id_table = sgm41510_slave_i2c_id,
};

module_i2c_driver(sgm41510_master_charger_driver);
module_i2c_driver(sgm41510_slave_charger_driver);
MODULE_DESCRIPTION("SGM41510 Charger Driver");
MODULE_LICENSE("GPL v2");
