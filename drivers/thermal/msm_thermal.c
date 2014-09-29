/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * 2014-09 void: Reworked approach to deal with thermal issues.
 * Base was Faux's intelli-thermal and original Oppo 4.2 source,
 * but just offlining cores isn't the best approach imo.
 * We need a combination of throttling and core offlining, together with
 * focus on keeping cpu0 on highest possible freq.
 * Also, limit_idx_low/high have to be read dynamically. Original
 * approach just took compile-time min/max, ignoring the actual
 * cpu policy. Pretty fatal with oc'ing enabled freq tables...
 * Another addition is the re-introduction of overtemp. I just feel
 * it's a good idea to keep a safety net, esp. with oc/ov in mind...
 *
 */
 
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/msm_tsens.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/msm_thermal.h>
#include <mach/cpufreq.h>

#define CORE_MAX_FREQ_HYSTERESIS 5

static int enabled;
static struct msm_thermal_data msm_thermal_info;
static struct delayed_work check_temp_work;
static struct workqueue_struct *intellithermal_wq;
static bool core_control_enabled;
static uint32_t cpus_offlined;
static DEFINE_MUTEX(core_control_mutex);

static struct cpufreq_frequency_table *table;
static int limit_idx_low;
static int limit_idx_high;
static int limit_idx = INT_MAX;
static long cur_temp = 0;
static bool on_overtemp = false;

/* module parameters */
module_param_named(temp, cur_temp, long, 0644);
module_param_named(poll_ms, msm_thermal_info.poll_ms, uint, 0664);
module_param_named(limit_overtemp_degC, msm_thermal_info.limit_overtemp_degC,
			int, 0664);
module_param_named(limit_temp_degC, msm_thermal_info.limit_temp_degC,
			int, 0664);
module_param_named(temp_hysteresis_degC, msm_thermal_info.temp_hysteresis_degC,
			int, 0664);
module_param_named(freq_control_mask, msm_thermal_info.freq_control_mask,
			uint, 0664);
module_param_named(core_limit_temp_degC, msm_thermal_info.core_limit_temp_degC,
			int, 0664);
module_param_named(core_control_mask, msm_thermal_info.core_control_mask,
			uint, 0664);

/* called from sysfs change to scaling_max/min_freq (and during init) */
int msm_thermal_update_limits(void)
{
	struct cpufreq_policy pol;
	int i = 0;
	int ret = 0;
	int old_idx_high = limit_idx_high;

	ret = cpufreq_get_policy(&pol, 0);
	if (unlikely(ret)) {
		pr_debug("%s: error reading cpufreq policy\n", KBUILD_MODNAME);
	} else {
		for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++) {
			if (table[i].frequency == pol.min)
				limit_idx_low = i;
			if (table[i].frequency == pol.max)
				limit_idx_high = i;
		}
		/* scaling_max_freq either went up, or down */
		if (old_idx_high < limit_idx_high ||
				limit_idx > limit_idx_high)
			limit_idx = limit_idx_high;
	}

	pr_debug("%s_dbg: limit_high %i, limit_low %i, limit_cur %i\n",
		KBUILD_MODNAME, limit_idx_high, limit_idx_low, limit_idx);
	return ret;
}

static int msm_thermal_get_freq_table(void)
{
	int ret = 0;
	struct cpu_freq *limit = NULL;

	table = cpufreq_frequency_get_table(0);
	if (table == NULL) {
		pr_debug("%s: error reading cpufreq table\n", KBUILD_MODNAME);
		ret = -EINVAL;
		goto fail;
	}

	ret = msm_thermal_update_limits();
	if (ret) {
		ret = -EINVAL;
		goto fail;
	}

	limit = &per_cpu(cpu_freq_info, 0);
	if (!limit->limits_init) {
		ret = msm_cpufreq_limits_init();
		if (ret)
			goto fail;
	}

	limit_idx = limit_idx_high;
	pr_info("%s: init completed\n", KBUILD_MODNAME);
	BUG_ON(limit_idx_high <= 0 || limit_idx_high <= limit_idx_low);
fail:
	return ret;
}

static int update_cpu_max_freq(int cpu, uint32_t max_freq)
{
	int ret = 0;
	struct cpu_freq *limit = &per_cpu(cpu_freq_info, cpu);

	if (max_freq <= limit->max && max_freq >= limit->min) {
		limit->allowed_max = max_freq;
		pr_info("%s: Limiting cpu%d max frequency to %d (%ld C)\n",
			KBUILD_MODNAME, cpu, max_freq, cur_temp);
	
	} else if (max_freq == MSM_CPUFREQ_NO_LIMIT &&
			limit->allowed_max != limit->max) {
		limit->allowed_max = limit->max;
		pr_info("%s: Max frequency reset for cpu%d (%ld C)\n",
			KBUILD_MODNAME, cpu, cur_temp);
	} else {
		pr_debug("%s_dbg: cpu%d already on %d\n", KBUILD_MODNAME, cpu, max_freq);
		return 0;
	}

	if (cpu_online(cpu)) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);
		if (!policy)
			return ret;
		ret = cpufreq_driver_target(policy, policy->cur,
				CPUFREQ_RELATION_H);
		cpufreq_cpu_put(policy);
	}

	return ret;
}

#ifdef CONFIG_SMP
static void __ref do_core_control(void)
{
	int i = 0;
	int ret = 0;

	if (!core_control_enabled)
		return;
	
	mutex_lock(&core_control_mutex);
	if (cur_temp >= msm_thermal_info.core_limit_temp_degC) {
		for (i = num_possible_cpus(); i > 0; i--) {
			if (!(msm_thermal_info.core_control_mask & BIT(i)))
				continue;
			if (cpus_offlined & BIT(i) && !cpu_online(i))
				continue;
			
			pr_info("%s: Set Offline: CPU%d (%ld C)\n",
					KBUILD_MODNAME, i, cur_temp);
			ret = cpu_down(i);
			if (unlikely(ret))
				pr_err("%s: Error %d offline core %d\n",
					KBUILD_MODNAME, ret, i);
			cpus_offlined |= BIT(i);
			break;
		}
	
	} else if (cpus_offlined &&
			cur_temp <= (msm_thermal_info.core_limit_temp_degC -
			msm_thermal_info.core_temp_hysteresis_degC)) {
		for (i = 0; i < num_possible_cpus(); i++) {
			if (!(cpus_offlined & BIT(i)))
				continue;
			cpus_offlined &= ~BIT(i);
			pr_info("%s: Allow Online CPU%d (%ld C)\n",
					KBUILD_MODNAME, i, cur_temp);
			/* If this core is already online, then bring up the
			 * next offlined core.
			 */
			if (cpu_online(i))
				continue;
			ret = cpu_up(i);
			if (unlikely(ret))
				pr_err("%s: Error %d online core %d\n",
						KBUILD_MODNAME, ret, i);
			break;
		}
	}
	mutex_unlock(&core_control_mutex);
}
#else
static void __ref do_core_control(void)
{
	return;
}
#endif

static void __ref do_handle_overtemp(void)
{
	int ret = 0;
	int cpu = 0;
	uint32_t max_freq = 0;

	/* overtemp, ignore limit_idx_low and throttle rigorously */
	if (cur_temp > msm_thermal_info.limit_overtemp_degC) {
		pr_info("%s: !!! OVERTEMP !!! emergency min freq!\n", KBUILD_MODNAME);
		on_overtemp = true;
		max_freq = table[1].frequency;
		for_each_possible_cpu(cpu) {
			ret = update_cpu_max_freq(cpu, max_freq);
			if (ret)
				pr_info("%s: Unable to limit cpu%d max freq to %d\n",
					KBUILD_MODNAME, cpu, max_freq);
		}

	/* overtemp gone, all governed cores to limit_idx_low */
	} else if (on_overtemp &&
			cur_temp < msm_thermal_info.limit_temp_degC) {
		pr_info("%s: OVERTEMP gone, resuming normal operation\n", KBUILD_MODNAME);
			
		/* cores with disabled freq_control have to be set to max,
		 * else they're stuck on throttled overtemp freq
		*/
		for_each_possible_cpu(cpu) {
			if (msm_thermal_info.freq_control_mask & BIT(cpu))
				max_freq = table[limit_idx_low].frequency;
			else
				max_freq = MSM_CPUFREQ_NO_LIMIT;
			
			ret = update_cpu_max_freq(cpu, max_freq);
			if (ret)
				pr_info("%s: Unable to limit cpu%d max freq to %d\n",
					KBUILD_MODNAME, cpu, max_freq);
		}
	
		/* limit_idx_low to prevent overtemp oscilation */
		limit_idx = limit_idx_low;
		on_overtemp = false;
	}
	
	/* offlining cores might have failed previously, causing
	 * the overtemp condition in the first place -> try again
	*/
	do_core_control();
}

static void __ref do_freq_control(void)
{
	int ret = 0;
	int cpu = 0;
	int cpu_limit_idx = limit_idx;
	uint32_t max_freq = MSM_CPUFREQ_NO_LIMIT;

	for_each_possible_cpu(cpu) {
		if (!(msm_thermal_info.freq_control_mask & BIT(cpu)))
			continue;
		
		cpu_limit_idx = limit_idx;
		
		/* decrease freq on high temp */
		if (cur_temp >= msm_thermal_info.limit_temp_degC) {
			if (limit_idx <= limit_idx_low)
				return;
			cpu_limit_idx -= (cpu+1);
			if (cpu_limit_idx < limit_idx_low)
				cpu_limit_idx = limit_idx_low;
			max_freq = table[cpu_limit_idx].frequency;
			
		/* increase freq on low temp */
		} else if (cur_temp < msm_thermal_info.limit_temp_degC -
				msm_thermal_info.temp_hysteresis_degC) {
			if (limit_idx >= limit_idx_high)
				return;
			
			/* boost freq to max on very low temp */
			if (cur_temp < msm_thermal_info.limit_temp_degC -
					msm_thermal_info.temp_hysteresis_degC - CORE_MAX_FREQ_HYSTERESIS)
				cpu_limit_idx = limit_idx_high;
			else
				cpu_limit_idx += (num_possible_cpus()-cpu);
			
			if (cpu_limit_idx >= limit_idx_high) {
				cpu_limit_idx = limit_idx_high;
				max_freq = MSM_CPUFREQ_NO_LIMIT;
			} else
				max_freq = table[cpu_limit_idx].frequency;
		
		/* do nothing within throtteling band */
		} else {
			pr_debug("%s_dbg: no freq change within bounds (%ld C)\n",
				KBUILD_MODNAME, cur_temp);
			return;
		}
		
		ret = update_cpu_max_freq(cpu, max_freq);
		if (ret)
			pr_info("%s: Unable to limit cpu%d max freq to %d\n",
				KBUILD_MODNAME, cpu, max_freq);
	}

	if (cpu_limit_idx == limit_idx_high) {
		limit_idx = limit_idx_high;
		return;
	}	else if (cpu_limit_idx < limit_idx)
		limit_idx--;
	else
		limit_idx++;
	
	if (limit_idx < limit_idx_low)
		limit_idx = limit_idx_low;
	else if (limit_idx > limit_idx_high)
		limit_idx = limit_idx_high;
}

static void __ref check_temp(struct work_struct *work)
{
	static int limit_init;
	struct tsens_device tsens_dev;
	int ret = 0;

	if (!limit_init) {
		ret = msm_thermal_get_freq_table();
		if (ret)
			goto reschedule;
		else
			limit_init = 1;
	}

	tsens_dev.sensor_num = msm_thermal_info.sensor_id;
	ret = tsens_get_temp(&tsens_dev, &cur_temp);
	if (ret) {
		pr_info("%s: Unable to read TSENS sensor %d\n",
				KBUILD_MODNAME, tsens_dev.sensor_num);
		goto reschedule;
	}

	/* Overtemp is nearly impossible to trigger on "stock" parameters.
	 * But better keep it as safety net when oc/ov'ing ...
	*/
	if (on_overtemp ||
			cur_temp > msm_thermal_info.limit_overtemp_degC)
		do_handle_overtemp();
	else {
		if (msm_thermal_info.core_control_mask)
			do_core_control();
		if (msm_thermal_info.freq_control_mask)
			do_freq_control();
	}

reschedule:
	if (enabled)
		queue_delayed_work(intellithermal_wq, &check_temp_work,
				msecs_to_jiffies(msm_thermal_info.poll_ms));
}

static int __ref msm_thermal_cpu_callback(struct notifier_block *nfb,
		unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;

	if (action == CPU_UP_PREPARE || action == CPU_UP_PREPARE_FROZEN) {
		if (core_control_enabled &&
			(msm_thermal_info.core_control_mask & BIT(cpu)) &&
			(cpus_offlined & BIT(cpu))) {
/* OPPO 2013-09-12 zhenwx Delete begin for prink the log too much result in usage of cpu  too high */
#if 0
			pr_info(
			"%s: Preventing cpu%d from coming online.\n",
				KBUILD_MODNAME, cpu);
#endif
/* OPPO 2013-09-12 zhenwx Delete end */
			return NOTIFY_BAD;
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block __refdata msm_thermal_cpu_notifier = {
	.notifier_call = msm_thermal_cpu_callback,
};

/**
 * We will reset the cpu frequencies limits here. The core online/offline
 * status will be carried over to the process stopping the msm_thermal, as
 * we dont want to online a core and bring in the thermal issues.
 */
static void __ref disable_msm_thermal(void)
{
	int cpu = 0;

	flush_workqueue(intellithermal_wq);

	for_each_possible_cpu(cpu) {
		update_cpu_max_freq(cpu, MSM_CPUFREQ_NO_LIMIT);
	}
}

static int __ref set_enabled(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	if (*val == '0' || *val == 'n' || *val == 'N') {
		enabled = 0;
		disable_msm_thermal();
		pr_info("msm_thermal: disabling...\n");
	} else {
		if (!enabled) {
			enabled = 1;
			queue_delayed_work(intellithermal_wq,
					   &check_temp_work, 0);
			pr_info("msm_thermal: rescheduling...\n");
		} else
			pr_info("msm_thermal: already running...\n");
	}
	pr_info("%s: enabled = %d\n", KBUILD_MODNAME, enabled);
	ret = param_set_bool(val, kp);

	return ret;
}

static struct kernel_param_ops module_ops = {
	.set = set_enabled,
	.get = param_get_bool,
};

module_param_cb(enabled, &module_ops, &enabled, 0644);
MODULE_PARM_DESC(enabled, "enforce thermal limit on cpu");

#ifdef CONFIG_SMP
/* Call with core_control_mutex locked */
static int __ref update_offline_cores(int val)
{
	int cpu = 0;
	int ret = 0;

	cpus_offlined = msm_thermal_info.core_control_mask & val;
	if (!core_control_enabled)
		return 0;

	for_each_possible_cpu(cpu) {
		if (!(cpus_offlined & BIT(cpu)))
			continue;
		if (!cpu_online(cpu))
			continue;
		ret = cpu_down(cpu);
		if (ret)
			pr_err("%s: Unable to offline cpu%d\n",
				KBUILD_MODNAME, cpu);
	}
	return ret;
}
#else
static int update_offline_cores(int val)
{
	return 0;
}
#endif

static ssize_t show_cc_enabled(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", core_control_enabled);
}

static ssize_t __ref store_cc_enabled(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int val = 0;

	mutex_lock(&core_control_mutex);
	ret = kstrtoint(buf, 10, &val);
	if (ret) {
		pr_err("%s: Invalid input %s\n", KBUILD_MODNAME, buf);
		goto done_store_cc;
	}

	if (core_control_enabled == !!val)
		goto done_store_cc;

	core_control_enabled = !!val;
	if (core_control_enabled) {
		pr_info("%s: Core control enabled\n", KBUILD_MODNAME);
		register_cpu_notifier(&msm_thermal_cpu_notifier);
		update_offline_cores(cpus_offlined);
	} else {
		pr_info("%s: Core control disabled\n", KBUILD_MODNAME);
		unregister_cpu_notifier(&msm_thermal_cpu_notifier);
	}

done_store_cc:
	mutex_unlock(&core_control_mutex);
	return count;
}

static ssize_t show_cpus_offlined(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", cpus_offlined);
}

static ssize_t __ref store_cpus_offlined(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	uint32_t val = 0;

	mutex_lock(&core_control_mutex);
	ret = kstrtouint(buf, 10, &val);
	if (ret) {
		pr_err("%s: Invalid input %s\n", KBUILD_MODNAME, buf);
		goto done_cc;
	}

	if (enabled) {
		pr_err("%s: Ignoring request; polling thread is enabled.\n",
				KBUILD_MODNAME);
		goto done_cc;
	}

	if (cpus_offlined == val)
		goto done_cc;

	update_offline_cores(val);
done_cc:
	mutex_unlock(&core_control_mutex);
	return count;
}

static __refdata struct kobj_attribute cc_enabled_attr =
__ATTR(enabled, 0644, show_cc_enabled, store_cc_enabled);

static __refdata struct kobj_attribute cpus_offlined_attr =
__ATTR(cpus_offlined, 0644, show_cpus_offlined, store_cpus_offlined);

static __refdata struct attribute *cc_attrs[] = {
	&cc_enabled_attr.attr,
	&cpus_offlined_attr.attr,
	NULL,
};

static __refdata struct attribute_group cc_attr_group = {
	.attrs = cc_attrs,
};

static __init int msm_thermal_add_cc_nodes(void)
{
	struct kobject *module_kobj = NULL;
	struct kobject *cc_kobj = NULL;
	int ret = 0;

	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("%s: cannot find kobject for module\n",
			KBUILD_MODNAME);
		ret = -ENOENT;
		goto done_cc_nodes;
	}

	cc_kobj = kobject_create_and_add("core_control", module_kobj);
	if (!cc_kobj) {
		pr_err("%s: cannot create core control kobj\n",
				KBUILD_MODNAME);
		ret = -ENOMEM;
		goto done_cc_nodes;
	}

	ret = sysfs_create_group(cc_kobj, &cc_attr_group);
	if (ret) {
		pr_err("%s: cannot create group\n", KBUILD_MODNAME);
		goto done_cc_nodes;
	}

	return 0;

done_cc_nodes:
	if (cc_kobj)
		kobject_del(cc_kobj);
	return ret;
}

int __init msm_thermal_init(struct msm_thermal_data *pdata)
{
	int ret = 0;

	BUG_ON(!pdata);
	BUG_ON(pdata->sensor_id >= TSENS_MAX_SENSORS);
	memcpy(&msm_thermal_info, pdata, sizeof(struct msm_thermal_data));

	enabled = 1;
	if (num_possible_cpus() > 1)
		core_control_enabled = 1;
	intellithermal_wq = alloc_workqueue("intellithermal",
				WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	INIT_DELAYED_WORK(&check_temp_work, check_temp);
	queue_delayed_work(intellithermal_wq, &check_temp_work, 0);

	if (num_possible_cpus() > 1)
		register_cpu_notifier(&msm_thermal_cpu_notifier);

	return ret;
}

int __init msm_thermal_late_init(void)
{
	if (num_possible_cpus() > 1)
		msm_thermal_add_cc_nodes();

	return 0;
}
late_initcall(msm_thermal_late_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Praveen Chidambaram <pchidamb@codeaurora.org>");
MODULE_AUTHOR("void");
MODULE_DESCRIPTION("intelligent thermal driver for Qualcomm based SOCs");
MODULE_DESCRIPTION("originally from Qualcomm's open source repo");
