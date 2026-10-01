// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3798CV200 system timer support
 */

#define pr_fmt(fmt) "hi3798cv200-timer: " fmt

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_clk.h>
#include <linux/of_irq.h>
#include <linux/percpu.h>
#include <linux/sched_clock.h>

#define HI3798_TIMER_LOAD		0x00
#define HI3798_TIMER_VALUE		0x04
#define HI3798_TIMER_CTRL		0x08
#define HI3798_TIMER_INTCLR		0x0c

#define HI3798_TIMER_CTRL_ONESHOT	BIT(0)
#define HI3798_TIMER_CTRL_32BIT		BIT(1)
#define HI3798_TIMER_CTRL_IE		BIT(5)
#define HI3798_TIMER_CTRL_PERIODIC	BIT(6)
#define HI3798_TIMER_CTRL_ENABLE		BIT(7)

struct hi3798_timer_event {
	struct clock_event_device clkevt;
	void __iomem *base;
	unsigned long rate;
	unsigned long reload;
	unsigned int irq;
	bool requested;
	char name[24];
};

static struct hi3798_timer_event __percpu *hi3798_events;
static void __iomem *hi3798_clocksource_base;
static struct clk *hi3798_timer_clk;

static struct hi3798_timer_event *
to_hi3798_event(struct clock_event_device *clkevt)
{
	return container_of(clkevt, struct hi3798_timer_event, clkevt);
}

static void hi3798_timer_disable(void __iomem *base)
{
	writel(0, base + HI3798_TIMER_CTRL);
	writel(1, base + HI3798_TIMER_INTCLR);
}

static void hi3798_timer_load(void __iomem *base, unsigned long value)
{
	/* The load register must be sampled twice before enabling the channel. */
	writel(value, base + HI3798_TIMER_LOAD);
	writel(value, base + HI3798_TIMER_LOAD);
}

static int hi3798_event_shutdown(struct clock_event_device *clkevt)
{
	struct hi3798_timer_event *event = to_hi3798_event(clkevt);

	hi3798_timer_disable(event->base);

	return 0;
}

static int hi3798_event_set_periodic(struct clock_event_device *clkevt)
{
	struct hi3798_timer_event *event = to_hi3798_event(clkevt);
	u32 ctrl = HI3798_TIMER_CTRL_32BIT | HI3798_TIMER_CTRL_IE |
		   HI3798_TIMER_CTRL_PERIODIC | HI3798_TIMER_CTRL_ENABLE;

	hi3798_timer_disable(event->base);
	hi3798_timer_load(event->base, event->reload);
	writel(ctrl, event->base + HI3798_TIMER_CTRL);

	return 0;
}

static int hi3798_event_set_next(unsigned long next,
				 struct clock_event_device *clkevt)
{
	struct hi3798_timer_event *event = to_hi3798_event(clkevt);
	u32 ctrl = HI3798_TIMER_CTRL_32BIT | HI3798_TIMER_CTRL_IE |
		   HI3798_TIMER_CTRL_ONESHOT | HI3798_TIMER_CTRL_ENABLE;

	hi3798_timer_disable(event->base);
	hi3798_timer_load(event->base, next);
	writel(ctrl, event->base + HI3798_TIMER_CTRL);

	return 0;
}

static irqreturn_t hi3798_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *clkevt = dev_id;
	struct hi3798_timer_event *event = to_hi3798_event(clkevt);

	/* Match the BSP's ordered clear before the GIC EOI. */
	writel(1, event->base + HI3798_TIMER_INTCLR);
	clkevt->event_handler(clkevt);

	return IRQ_HANDLED;
}

static int hi3798_timer_starting_cpu(unsigned int cpu)
{
	struct hi3798_timer_event *event = per_cpu_ptr(hi3798_events, cpu);
	int ret;

	ret = irq_force_affinity(event->irq, cpumask_of(cpu));
	if (ret)
		pr_warn("failed to route IRQ%u to CPU%u: %d\n",
			event->irq, cpu, ret);

	clockevents_config_and_register(&event->clkevt, event->rate,
					0xf, 0x7fffffff);
	enable_irq(event->irq);

	return 0;
}

static int hi3798_timer_dying_cpu(unsigned int cpu)
{
	struct hi3798_timer_event *event = per_cpu_ptr(hi3798_events, cpu);

	hi3798_event_shutdown(&event->clkevt);
	disable_irq_nosync(event->irq);

	return 0;
}

static u64 hi3798_clocksource_read(struct clocksource *cs)
{
	return ~readl_relaxed(hi3798_clocksource_base + HI3798_TIMER_VALUE);
}

static u64 notrace hi3798_sched_clock_read(void)
{
	return ~readl_relaxed(hi3798_clocksource_base + HI3798_TIMER_VALUE);
}

static void hi3798_clocksource_enable(void)
{
	hi3798_timer_disable(hi3798_clocksource_base);
	writel(U32_MAX, hi3798_clocksource_base + HI3798_TIMER_LOAD);
	writel(U32_MAX, hi3798_clocksource_base + HI3798_TIMER_VALUE);
	writel(HI3798_TIMER_CTRL_32BIT | HI3798_TIMER_CTRL_PERIODIC |
	       HI3798_TIMER_CTRL_ENABLE,
	       hi3798_clocksource_base + HI3798_TIMER_CTRL);
}

static void hi3798_clocksource_resume(struct clocksource *cs)
{
	hi3798_clocksource_enable();
}

static struct clocksource hi3798_clocksource = {
	.name		= "hi3798cv200-timer",
	/* Keep the architected timer initialized, but prefer the vendor timer. */
	.rating		= 499,
	.read		= hi3798_clocksource_read,
	.resume		= hi3798_clocksource_resume,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void hi3798_timer_cleanup(unsigned int initialized_cpus)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		struct hi3798_timer_event *event;

		if (cpu >= initialized_cpus)
			break;

		event = per_cpu_ptr(hi3798_events, cpu);
		if (event->requested) {
			free_irq(event->irq, &event->clkevt);
			irq_clear_status_flags(event->irq, IRQ_NOAUTOEN);
		}
		if (event->irq)
			irq_dispose_mapping(event->irq);
		if (event->base)
			iounmap(event->base);
	}
}

static int __init hi3798_timer_init(struct device_node *np)
{
	unsigned int cpu, initialized_cpus = 0;
	unsigned long rate;
	int ret;

	if (of_address_count(np) < num_possible_cpus() + 1 ||
	    of_irq_count(np) < num_possible_cpus()) {
		pr_err("%pOF: missing per-CPU timer resources\n", np);
		return -EINVAL;
	}

	hi3798_events = alloc_percpu(struct hi3798_timer_event);
	if (!hi3798_events)
		return -ENOMEM;

	hi3798_timer_clk = of_clk_get_by_name(np, "timer");
	if (IS_ERR(hi3798_timer_clk)) {
		ret = PTR_ERR(hi3798_timer_clk);
		goto err_free_events;
	}

	ret = clk_prepare_enable(hi3798_timer_clk);
	if (ret)
		goto err_put_clk;

	rate = clk_get_rate(hi3798_timer_clk);
	if (!rate) {
		ret = -EINVAL;
		goto err_disable_clk;
	}

	for_each_possible_cpu(cpu) {
		struct hi3798_timer_event *event;
		struct clock_event_device *clkevt;

		event = per_cpu_ptr(hi3798_events, cpu);
		event->base = of_iomap(np, cpu + 1);
		if (!event->base) {
			ret = -ENOMEM;
			goto err_cleanup_events;
		}

		event->irq = irq_of_parse_and_map(np, cpu);
		if (!event->irq) {
			ret = -EINVAL;
			goto err_cleanup_current;
		}

		event->rate = rate;
		event->reload = DIV_ROUND_CLOSEST(rate, HZ);
		snprintf(event->name, sizeof(event->name),
			 "hi3798cv200-cpu%u", cpu);

		clkevt = &event->clkevt;
		clkevt->name = event->name;
		clkevt->features = CLOCK_EVT_FEAT_PERIODIC |
				   CLOCK_EVT_FEAT_ONESHOT;
		clkevt->rating = 500;
		clkevt->irq = event->irq;
		clkevt->cpumask = cpumask_of(cpu);
		clkevt->set_state_shutdown = hi3798_event_shutdown;
		clkevt->set_state_periodic = hi3798_event_set_periodic;
		clkevt->set_state_oneshot = hi3798_event_shutdown;
		clkevt->tick_resume = hi3798_event_shutdown;
		clkevt->set_next_event = hi3798_event_set_next;

		hi3798_timer_disable(event->base);
		irq_set_status_flags(event->irq, IRQ_NOAUTOEN);
		ret = request_irq(event->irq, hi3798_timer_interrupt,
				  IRQF_TIMER | IRQF_NOBALANCING,
				  event->name, clkevt);
		if (ret) {
			irq_clear_status_flags(event->irq, IRQ_NOAUTOEN);
			goto err_cleanup_current;
		}
		event->requested = true;
		initialized_cpus++;
	}

	hi3798_clocksource_base = of_iomap(np, 0);
	if (!hi3798_clocksource_base) {
		ret = -ENOMEM;
		goto err_cleanup_events;
	}

	ret = cpuhp_setup_state(CPUHP_AP_HISI_TIMER_STARTING,
				"clockevents/hi3798cv200:starting",
				hi3798_timer_starting_cpu,
				hi3798_timer_dying_cpu);
	if (ret)
		goto err_unmap_clocksource;

	hi3798_clocksource_enable();
	ret = clocksource_register_hz(&hi3798_clocksource, rate);
	if (ret)
		goto err_remove_cpuhp;

	sched_clock_register(hi3798_sched_clock_read, 32, rate);
	pr_info("registered %lu Hz clocksource and %u clock events\n",
		rate, num_possible_cpus());

	return 0;

err_remove_cpuhp:
	cpuhp_remove_state(CPUHP_AP_HISI_TIMER_STARTING);
err_unmap_clocksource:
	iounmap(hi3798_clocksource_base);
err_cleanup_events:
	hi3798_timer_cleanup(initialized_cpus);
	goto err_disable_clk;
err_cleanup_current:
	initialized_cpus++;
	hi3798_timer_cleanup(initialized_cpus);
err_disable_clk:
	clk_disable_unprepare(hi3798_timer_clk);
err_put_clk:
	clk_put(hi3798_timer_clk);
err_free_events:
	free_percpu(hi3798_events);
	return ret;
}

TIMER_OF_DECLARE(hi3798cv200_timer, "hisilicon,hi3798cv200-timer",
		 hi3798_timer_init);
