// SPDX-License-Identifier: GPL-2.0-or-later OR MIT
/*
 * Copyright (c) 2023 David Yang
 */

#include <linux/err.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define RNG_CTRL		0x0
#define  RNG_V100_SOURCE		GENMASK(1, 0)
#define  RNG_V100_DROP_ENABLE		BIT(5)
#define  RNG_V100_POST_PROCESS_ENABLE	BIT(7)
#define  RNG_V100_POST_PROCESS_DEPTH	GENMASK(15, 8)
#define  RNG_V200_DRBG_ENABLE		BIT(3)
#define RNG_NUMBER		0x4
#define RNG_STAT		0x8
#define  RNG_V100_DATA_COUNT		GENMASK(2, 0)	/* max 4 */
#define  RNG_V200_DATA_COUNT		GENMASK(15, 8)
#define  RNG_V200_ALARMS		GENMASK(22, 16)

struct histb_rng_data {
	void (*init)(void __iomem *base, unsigned int depth);
	u32 data_count_mask;
	u32 alarm_mask;
	bool has_depth;
};

struct histb_rng_priv {
	struct hwrng rng;
	void __iomem *base;
	const struct histb_rng_data *data;
	struct device *dev;
};

/*
 * Observed:
 * depth = 1 -> ~1ms
 * depth = 255 -> ~16ms
 */
static int histb_rng_wait(struct histb_rng_priv *priv)
{
	u32 val;
	int ret;

	ret = readl_relaxed_poll_timeout(priv->base + RNG_STAT, val,
					 (val & priv->data->data_count_mask) ||
					 (val & priv->data->alarm_mask),
					 1000, 30 * 1000);
	if (ret)
		return ret;

	if (val & priv->data->alarm_mask) {
		dev_err_ratelimited(priv->dev, "hardware alarm: %#x\n",
				    val & priv->data->alarm_mask);
		return -EIO;
	}

	return 0;
}

static void histb_rng_v100_init(void __iomem *base, unsigned int depth)
{
	u32 val;

	val = readl_relaxed(base + RNG_CTRL);

	val &= ~RNG_V100_SOURCE;
	val |= 2;

	val &= ~RNG_V100_POST_PROCESS_DEPTH;
	val |= min(depth, 0xffu) << 8;

	val |= RNG_V100_POST_PROCESS_ENABLE;
	val |= RNG_V100_DROP_ENABLE;

	writel_relaxed(val, base + RNG_CTRL);
}

static void histb_rng_v200_init(void __iomem *base, unsigned int depth)
{
	writel_relaxed(2 | RNG_V200_DRBG_ENABLE, base + RNG_CTRL);
}

static int histb_rng_read(struct hwrng *rng, void *data, size_t max, bool wait)
{
	struct histb_rng_priv *priv = container_of(rng, typeof(*priv), rng);
	void __iomem *base = priv->base;

	for (int i = 0; i < max; i += sizeof(u32)) {
		u32 stat = readl_relaxed(base + RNG_STAT);
		int ret;

		if (stat & priv->data->alarm_mask) {
			dev_err_ratelimited(priv->dev, "hardware alarm: %#x\n",
					    stat & priv->data->alarm_mask);
			return -EIO;
		}

		if (!(stat & priv->data->data_count_mask)) {
			if (!wait)
				return i;
			ret = histb_rng_wait(priv);
			if (ret) {
				if (ret == -EIO)
					return ret;
				pr_err("failed to generate random number, generated %d\n",
				       i);
				return i ? i : -ETIMEDOUT;
			}
		}
		*(u32 *)(data + i) = readl_relaxed(base + RNG_NUMBER);
	}

	return max;
}

static unsigned int histb_rng_get_depth(void __iomem *base)
{
	return (readl_relaxed(base + RNG_CTRL) &
		RNG_V100_POST_PROCESS_DEPTH) >> 8;
}

static ssize_t
depth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct histb_rng_priv *priv = dev_get_drvdata(dev);
	void __iomem *base = priv->base;

	return sprintf(buf, "%d\n", histb_rng_get_depth(base));
}

static ssize_t
depth_store(struct device *dev, struct device_attribute *attr,
	    const char *buf, size_t count)
{
	struct histb_rng_priv *priv = dev_get_drvdata(dev);
	void __iomem *base = priv->base;
	unsigned int depth;

	if (kstrtouint(buf, 0, &depth))
		return -ERANGE;

	priv->data->init(base, depth);
	return count;
}

static DEVICE_ATTR_RW(depth);

static struct attribute *histb_rng_attrs[] = {
	&dev_attr_depth.attr,
	NULL,
};

static umode_t
histb_rng_attr_is_visible(struct kobject *kobj, struct attribute *attr,
			  int unused)
{
	struct histb_rng_priv *priv = dev_get_drvdata(kobj_to_dev(kobj));

	return priv->data->has_depth ? attr->mode : 0;
}

static const struct attribute_group histb_rng_group = {
	.attrs = histb_rng_attrs,
	.is_visible = histb_rng_attr_is_visible,
};

static const struct attribute_group *histb_rng_groups[] = {
	&histb_rng_group,
	NULL,
};

static const struct histb_rng_data histb_rng_v100_data = {
	.init = histb_rng_v100_init,
	.data_count_mask = RNG_V100_DATA_COUNT,
	.has_depth = true,
};

static const struct histb_rng_data histb_rng_v200_data = {
	.init = histb_rng_v200_init,
	.data_count_mask = RNG_V200_DATA_COUNT,
	.alarm_mask = RNG_V200_ALARMS,
};

static int histb_rng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_rng_priv *priv;
	void __iomem *base;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	priv->base = base;
	priv->data = device_get_match_data(dev);
	priv->dev = dev;
	if (!priv->data)
		return -ENODEV;

	priv->data->init(base, 144);
	if (histb_rng_wait(priv)) {
		dev_err(dev, "cannot bring up device\n");
		return -ENODEV;
	}

	priv->rng.name = pdev->name;
	priv->rng.read = histb_rng_read;
	ret = devm_hwrng_register(dev, &priv->rng);
	if (ret) {
		dev_err(dev, "failed to register hwrng: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, priv);
	dev_set_drvdata(dev, priv);
	return 0;
}

static const struct of_device_id histb_rng_of_match[] = {
	{ .compatible = "hisilicon,histb-rng", .data = &histb_rng_v100_data },
	{ .compatible = "hisilicon,hi3798cv200-rng", .data = &histb_rng_v200_data },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_rng_of_match);

static struct platform_driver histb_rng_driver = {
	.probe = histb_rng_probe,
	.driver = {
		.name = "histb-rng",
		.of_match_table = histb_rng_of_match,
		.dev_groups = histb_rng_groups,
	},
};

module_platform_driver(histb_rng_driver);

MODULE_DESCRIPTION("Hisilicon STB random number generator driver");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
