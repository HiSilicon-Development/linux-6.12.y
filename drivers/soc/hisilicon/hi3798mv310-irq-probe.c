// SPDX-License-Identifier: GPL-2.0-only
/* RAM bring-up only: BSP CPU0 SP804 vs software-pended, owned SPI. */
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define TIMER_LOAD	0x00
#define TIMER_VALUE	0x04
#define TIMER_CTRL	0x08
#define TIMER_INTCLR	0x0c
#define TIMER_RIS	0x10
#define TIMER_MIS	0x14
#define TIMER_32BIT	BIT(1)
#define TIMER_ONESHOT	(BIT(7) | BIT(5) | TIMER_32BIT | BIT(0))

enum probe_phase { PROBE_IDLE, PROBE_HARDWARE, PROBE_SOFTWARE };

struct irq_probe {
	void __iomem *base;
	struct proc_dir_entry *proc;
	struct mutex lock;
	int irq;
	unsigned long hwirq;
	unsigned long rate;
	u32 reload;
	bool irq_enabled;
	enum probe_phase phase;
	atomic_t total, hardware, software, ris_events;
	int last_cpu;
	u32 last_ris;
};

static void irq_probe_stop_timer(struct irq_probe *p)
{
	writel(0, p->base + TIMER_CTRL);
	writel(1, p->base + TIMER_INTCLR);
}

static void irq_probe_disable(struct irq_probe *p)
{
	if (p->irq_enabled) {
		disable_irq(p->irq);
		p->irq_enabled = false;
	}
}

static irqreturn_t irq_probe_handler(int irq, void *data)
{
	struct irq_probe *p = data;
	u32 ris = readl(p->base + TIMER_RIS);

	writel(1, p->base + TIMER_INTCLR);
	WRITE_ONCE(p->last_cpu, smp_processor_id());
	WRITE_ONCE(p->last_ris, ris);
	atomic_inc(&p->total);
	if (ris & 1)
		atomic_inc(&p->ris_events);
	if (READ_ONCE(p->phase) == PROBE_HARDWARE)
		atomic_inc(&p->hardware);
	else if (READ_ONCE(p->phase) == PROBE_SOFTWARE)
		atomic_inc(&p->software);
	return IRQ_HANDLED;
}

static bool irq_probe_single_cpu(void)
{
	return num_online_cpus() == 1 && cpu_online(0);
}

static int irq_probe_show(struct seq_file *m, void *v)
{
	struct irq_probe *p = m->private;
	unsigned long flags;
	u32 load, value, ctrl, ris, mis, last_ris;
	int total, hardware, software, ris_events, last_cpu;
	int pending_rc, active_rc, masked_rc;
	bool pending = false, active = false, masked = false;
	enum probe_phase phase;

	cpus_read_lock();
	if (!irq_probe_single_cpu()) {
		cpus_read_unlock();
		return -EBUSY;
	}
	mutex_lock(&p->lock);
	get_cpu();
	local_irq_save(flags);
	load = readl(p->base + TIMER_LOAD);
	value = readl(p->base + TIMER_VALUE);
	ctrl = readl(p->base + TIMER_CTRL);
	ris = readl(p->base + TIMER_RIS);
	mis = readl(p->base + TIMER_MIS);
	pending_rc = irq_get_irqchip_state(p->irq, IRQCHIP_STATE_PENDING, &pending);
	active_rc = irq_get_irqchip_state(p->irq, IRQCHIP_STATE_ACTIVE, &active);
	masked_rc = irq_get_irqchip_state(p->irq, IRQCHIP_STATE_MASKED, &masked);
	phase = p->phase;
	total = atomic_read(&p->total);
	hardware = atomic_read(&p->hardware);
	software = atomic_read(&p->software);
	ris_events = atomic_read(&p->ris_events);
	last_cpu = READ_ONCE(p->last_cpu);
	last_ris = READ_ONCE(p->last_ris);
	local_irq_restore(flags);
	put_cpu();
	mutex_unlock(&p->lock);
	cpus_read_unlock();

	seq_puts(m, "IRQ_PROBE_BEGIN CPU0-only no-clockevent-replacement\n");
	seq_printf(m, "PHASE=%s LINUX_IRQ=%d HWIRQ=%lu RATE=%lu RELOAD=%u\n",
		   phase == PROBE_HARDWARE ? "hardware" :
		   phase == PROBE_SOFTWARE ? "software" : "idle",
		   p->irq, p->hwirq, p->rate, p->reload);
	seq_printf(m, "LOAD=0x%08x VALUE=0x%08x CTRL=0x%08x RIS=0x%08x MIS=0x%08x\n",
		   load, value, ctrl, ris, mis);
	seq_printf(m, "TOTAL=%d HARDWARE_PHASE=%d SOFTWARE_PHASE=%d RIS_EVENTS=%d LAST_CPU=%d LAST_RIS=0x%08x\n",
		   total, hardware, software, ris_events, last_cpu, last_ris);
	seq_printf(m, "PENDING=%u(rc=%d) ACTIVE=%u(rc=%d) MASKED=%u(rc=%d)\n",
		   pending, pending_rc, active, active_rc, masked, masked_rc);
	seq_puts(m, "IRQ_PROBE_END\n");
	return 0;
}

static int irq_probe_open(struct inode *inode, struct file *file)
{
	return single_open(file, irq_probe_show, pde_data(inode));
}

static ssize_t irq_probe_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *pos)
{
	struct irq_probe *p = pde_data(file_inode(file));
	char command[8];
	enum probe_phase phase;
	int ret;

	if (!count || count >= sizeof(command))
		return -EINVAL;
	if (copy_from_user(command, buf, count))
		return -EFAULT;
	command[count] = '\0';
	if (sysfs_streq(command, "start"))
		phase = PROBE_HARDWARE;
	else if (sysfs_streq(command, "pend"))
		phase = PROBE_SOFTWARE;
	else if (sysfs_streq(command, "stop"))
		phase = PROBE_IDLE;
	else
		return -EINVAL;

	cpus_read_lock();
	if (!irq_probe_single_cpu()) {
		cpus_read_unlock();
		return -EBUSY;
	}
	mutex_lock(&p->lock);
	irq_probe_disable(p);
	irq_probe_stop_timer(p);
	WRITE_ONCE(p->phase, PROBE_IDLE);
	ret = irq_set_irqchip_state(p->irq, IRQCHIP_STATE_PENDING, false);
	if (ret)
		goto out;
	WRITE_ONCE(p->phase, phase);
	if (phase == PROBE_HARDWARE) {
		atomic_set(&p->total, 0);
		atomic_set(&p->hardware, 0);
		atomic_set(&p->software, 0);
		atomic_set(&p->ris_events, 0);
		WRITE_ONCE(p->last_cpu, -1);
		WRITE_ONCE(p->last_ris, 0);
		writel(TIMER_32BIT, p->base + TIMER_CTRL);
		/* The MV310 BSP programs LOAD twice before enabling the event. */
		writel(p->reload, p->base + TIMER_LOAD);
		writel(p->reload, p->base + TIMER_LOAD);
		writel(TIMER_ONESHOT, p->base + TIMER_CTRL);
	} else if (phase == PROBE_SOFTWARE) {
		ret = irq_set_irqchip_state(p->irq, IRQCHIP_STATE_PENDING, true);
		if (ret) {
			WRITE_ONCE(p->phase, PROBE_IDLE);
			goto out;
		}
	}
	if (phase != PROBE_IDLE) {
		enable_irq(p->irq);
		p->irq_enabled = true;
	}
out:
	mutex_unlock(&p->lock);
	cpus_read_unlock();
	return ret ? ret : count;
}

static const struct proc_ops irq_probe_proc_ops = {
	.proc_open = irq_probe_open,
	.proc_read = seq_read,
	.proc_write = irq_probe_write,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static void irq_probe_release(void *data)
{
	struct irq_probe *p = data;

	/* Rundown prevents open proc operations from outliving devm resources. */
	proc_remove(p->proc);
	irq_probe_disable(p);
	irq_probe_stop_timer(p);
	irq_set_irqchip_state(p->irq, IRQCHIP_STATE_PENDING, false);
	irq_clear_status_flags(p->irq, IRQ_DISABLE_UNLAZY);
}

static int irq_probe_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct irq_probe *p;
	struct clk *clk;
	int ret;

	cpus_read_lock();
	ret = irq_probe_single_cpu() ? 0 : -EBUSY;
	cpus_read_unlock();
	if (ret)
		return dev_err_probe(dev, ret, "requires CPU0 as the only online CPU\n");
	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	mutex_init(&p->lock);
	p->last_cpu = -1;
	clk = devm_clk_get_enabled(dev, "apb_pclk");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "timer clock\n");
	p->rate = clk_get_rate(clk);
	if (p->rate < 10 || p->rate / 10 > U32_MAX)
		return dev_err_probe(dev, -EINVAL, "invalid 100ms timer rate\n");
	p->reload = p->rate / 10;
	/* Resource0 is the BSP clocksource; resource1 alone is CPU0's event. */
	p->base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);
	p->irq = platform_get_irq(pdev, 0);
	if (p->irq < 0)
		return p->irq;
	p->hwirq = irqd_to_hwirq(irq_get_irq_data(p->irq));
	irq_probe_stop_timer(p);
	ret = devm_request_irq(dev, p->irq, irq_probe_handler,
		IRQF_TIMER | IRQF_NOBALANCING | IRQF_NO_AUTOEN,
		"ec6110t-irq-probe", p);
	if (ret)
		return dev_err_probe(dev, ret, "request timer IRQ\n");
	ret = irq_force_affinity(p->irq, cpumask_of(0));
	if (ret)
		return dev_err_probe(dev, ret, "route timer IRQ to CPU0\n");
	ret = irq_set_irqchip_state(p->irq, IRQCHIP_STATE_PENDING, false);
	if (ret)
		return dev_err_probe(dev, ret, "clear owned timer IRQ pending\n");
	ret = devm_add_action_or_reset(dev, irq_probe_release, p);
	if (ret)
		return ret;
	/* Keep a software-pended level IRQ from being consumed while disabled. */
	irq_set_status_flags(p->irq, IRQ_DISABLE_UNLAZY);
	p->proc = proc_create_data("ec6110t_irq_probe", 0600, NULL,
				   &irq_probe_proc_ops, p);
	if (!p->proc)
		return -ENOMEM;
	dev_info(dev, "CPU0 diagnostic ready: IRQ%d INTID%lu %luHz; idle/disarmed\n",
		 p->irq, p->hwirq, p->rate);
	return 0;
}

static const struct of_device_id irq_probe_match[] = {
	{ .compatible = "hisilicon,hisp804" },
	{ }
};

static struct platform_driver irq_probe_driver = {
	.probe = irq_probe_probe,
	.driver = {
		.name = "ec6110t-irq-probe",
		.of_match_table = irq_probe_match,
	},
};
builtin_platform_driver(irq_probe_driver);
