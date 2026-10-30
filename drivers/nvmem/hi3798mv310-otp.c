// SPDX-License-Identifier: GPL-2.0-only
/* MV310 OTPv200 read transactions. Programming is deliberately not exposed. */
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/unaligned.h>

struct mv310_otp {
	void __iomem *base;
	struct mutex lock;
};

static int mv310_otp_read(void *context, unsigned int offset, void *buf,
			  size_t bytes)
{
	struct mv310_otp *otp = context;
	u32 channel, control, address, mode, status, word;
	bool temp_cell = offset == 0x258 && bytes == sizeof(word);
	bool cpu_cell = offset >= 0x44c && offset <= 0x44e && bytes == sizeof(u8);
	int ret;

	/* Expose only temperature and CPU voltage calibration words. */
	if (!temp_cell && !cpu_cell)
		return -EINVAL;
	mutex_lock(&otp->lock);
	ret = readl_poll_timeout(otp->base + 0xc, status,
				status & BIT(0), 10, 100000);
	if (ret)
		goto unlock;
	if (status & BIT(4)) {
		ret = -EACCES;
		goto unlock;
	}
	channel = readl(otp->base);
	control = readl(otp->base + 4);
	address = readl(otp->base + 0x18);
	mode = readl(otp->base + 0x20);
	if (control & BIT(2)) {
		ret = -EBUSY;
		goto unlock;
	}
	writel((channel & ~3U) | 2, otp->base);
	writel((control & ~0x37U) | 0x22, otp->base + 4);
	writel(mode | 0x34f, otp->base + 0x20);
	writel(offset, otp->base + 0x18);
	readl(otp->base + 0x18);
	usleep_range(1000, 1100);
	ret = readl_poll_timeout(otp->base + 0xc, status,
				status & BIT(0), 10, 100000);
	word = readl(otp->base + 0x10);
	writel(0, otp->base + 4);
	writel(address, otp->base + 0x18);
	writel(mode, otp->base + 0x20);
	writel(channel, otp->base);
	writel(control, otp->base + 4);
	readl(otp->base + 4);
	/* An access error can also be reported by the completed transaction. */
	if (!ret && (status & BIT(4)))
		ret = -EACCES;
	if (!ret) {
		if (temp_cell)
			put_unaligned_le32(word, buf);
		else
			*(u8 *)buf = word & 0xff;
	}
unlock:
	mutex_unlock(&otp->lock);
	return ret;
}

static int mv310_otp_probe(struct platform_device *pdev)
{
	struct nvmem_config config = {
		.dev = &pdev->dev,
		.name = "hi3798mv310-calibration",
		.read_only = true,
		.add_legacy_fixed_of_cells = true,
		.root_only = true,
		.stride = 1,
		.word_size = 1,
		.size = 0x450,
		.reg_read = mv310_otp_read,
	};
	struct mv310_otp *otp;

	otp = devm_kzalloc(&pdev->dev, sizeof(*otp), GFP_KERNEL);
	if (!otp)
		return -ENOMEM;
	otp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(otp->base))
		return PTR_ERR(otp->base);
	mutex_init(&otp->lock);
	config.priv = otp;
	return PTR_ERR_OR_ZERO(devm_nvmem_register(&pdev->dev, &config));
}

static const struct of_device_id mv310_otp_match[] = {
	{ .compatible = "hisilicon,hi3798mv310-otp" },
	{ }
};
MODULE_DEVICE_TABLE(of, mv310_otp_match);

static struct platform_driver mv310_otp_driver = {
	.probe = mv310_otp_probe,
	.driver = {
		.name = "hi3798mv310-otp",
		.of_match_table = mv310_otp_match,
	},
};
module_platform_driver(mv310_otp_driver);
MODULE_DESCRIPTION("Hi3798MV310 read-only temperature calibration OTP");
MODULE_LICENSE("GPL");
