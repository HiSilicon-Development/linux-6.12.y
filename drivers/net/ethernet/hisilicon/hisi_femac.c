// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hisilicon Fast Ethernet MAC Driver
 *
 * Copyright (c) 2016 HiSilicon Technologies Co., Ltd.
 */

#include <linux/circ_buf.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

/* MAC control register list */
#define MAC_PORTSEL			0x0200
#define MAC_PORTSEL_STAT_CPU		BIT(0)
#define MAC_PORTSEL_RMII		BIT(1)
#define MAC_PORTSET			0x0208
#define MAC_PORTSET_DUPLEX_FULL		BIT(0)
#define MAC_PORTSET_LINKED		BIT(1)
#define MAC_PORTSET_SPEED_100M		BIT(2)
#define MAC_SET				0x0210
#define MAX_FRAME_SIZE			1600
#define MAX_FRAME_SIZE_MASK		GENMASK(10, 0)
#define BIT_PAUSE_EN			BIT(18)
#define RX_COALESCE_SET			0x0340
#define RX_COALESCED_FRAME_OFFSET	24
#define RX_COALESCED_FRAMES		8
#define RX_COALESCED_TIMER		0x74
#define QLEN_SET			0x0344
#define RX_DEPTH_OFFSET			8
#define MAX_HW_FIFO_DEPTH		64
#define HW_TX_FIFO_DEPTH		12
#define HW_RX_FIFO_DEPTH		(MAX_HW_FIFO_DEPTH - HW_TX_FIFO_DEPTH)
#define IQFRM_DES			0x0354
#define RX_FRAME_LEN_MASK		GENMASK(11, 0)
#define IQ_ADDR				0x0358
#define EQ_ADDR				0x0360
#define EQFRM_LEN			0x0364
#define ADDRQ_STAT			0x036C
#define TX_CNT_INUSE_MASK		GENMASK(5, 0)
#define BIT_TX_READY			BIT(24)
#define BIT_RX_READY			BIT(25)
/* global control register list */
#define GLB_HOSTMAC_L32			0x0000
#define GLB_HOSTMAC_H16			0x0004
#define GLB_SOFT_RESET			0x0008
#define SOFT_RESET_ALL			BIT(0)
#define GLB_FWCTRL			0x0010
#define FWCTRL_VLAN_ENABLE		BIT(0)
#define FWCTRL_FW2CPU_ENA		BIT(5)
#define FWCTRL_FWALL2CPU		BIT(7)
#define GLB_MACTCTRL			0x0014
#define MACTCTRL_UNI2CPU		BIT(1)
#define MACTCTRL_MULTI2CPU		BIT(3)
#define MACTCTRL_BROAD2CPU		BIT(5)
#define MACTCTRL_MACT_ENA		BIT(7)
#define GLB_IRQ_STAT			0x0030
#define GLB_IRQ_ENA			0x0034
#define IRQ_ENA_PORT0_MASK		GENMASK(7, 0)
#define IRQ_ENA_PORT0			BIT(18)
#define IRQ_ENA_ALL			BIT(19)
#define GLB_IRQ_RAW			0x0038
#define IRQ_INT_RX_RDY			BIT(0)
#define IRQ_INT_TX_PER_PACKET		BIT(1)
#define IRQ_INT_TX_FIFO_EMPTY		BIT(6)
#define IRQ_INT_MULTI_RXRDY		BIT(7)
#define DEF_INT_MASK			(IRQ_INT_MULTI_RXRDY | \
					IRQ_INT_TX_PER_PACKET | \
					IRQ_INT_TX_FIFO_EMPTY)
#define GLB_MAC_L32_BASE		0x0100
#define GLB_MAC_H16_BASE		0x0104
#define MACFLT_HI16_MASK		GENMASK(15, 0)
#define BIT_MACFLT_ENA			BIT(17)
#define BIT_MACFLT_FW2CPU		BIT(21)
#define GLB_MAC_H16(reg)		(GLB_MAC_H16_BASE + ((reg) * 0x8))
#define GLB_MAC_L32(reg)		(GLB_MAC_L32_BASE + ((reg) * 0x8))
#define MAX_MAC_FILTER_NUM		8
#define MAX_UNICAST_ADDRESSES		2
#define MAX_MULTICAST_ADDRESSES		(MAX_MAC_FILTER_NUM - \
					MAX_UNICAST_ADDRESSES)
/* software tx and rx queue number, should be power of 2 */
#define TXQ_NUM				64
#define RXQ_NUM				128
#define FEMAC_POLL_WEIGHT		16

/* MV310 BSP hieth/hieth.h:61-188: identical port-0 FIFO/IRQ layout,
 * with global registers at MAC + 0x1300, not a separate IP block.
 */
#define MV310_GLB_OFFSET		0x1300
#define GLB_ENDIAN_MOD			0x0018
#define ENDIAN_IN_OUT			(BIT(1) | BIT(0))
#define MAC_TX_IPGCTRL			0x0218
#define TX_PRE_CNT_LIMIT		GENMASK(25, 23)
#define RX_COE_CTRL			0x0380
#define COE_PAYLOAD_DROP		BIT(14)
#define MV310_MAX_FRAME_SIZE		1535

/* BSP hieth/mdio.h:6-42; finish is active high, not a busy bit. */
#define MV310_MDIO_CTRL			0x1100
#define MV310_MDIO_DATA			0x1104
#define MV310_MDIO_FINISH		BIT(15)
#define MV310_MDIO_WRITE		BIT(13)
#define MV310_PHY_ADDR			0x0118
#define MV310_PHY_LDO			0x0844
#define MV310_PHY_LDO_MASK		GENMASK(7, 4)
#define MV310_PHY_LDO_VALUE		(BIT(4) | BIT(6))
#define FESTA_EXP_DATA			0x1d
#define FESTA_EXP_ADDR			0x1e

struct hisi_femac_soc_data {
	u32 glb_offset;
	u32 irq_mask;
};

static const struct hisi_femac_soc_data hisi_femac_mv310_data = {
	.glb_offset = MV310_GLB_OFFSET,
	.irq_mask = IRQ_INT_MULTI_RXRDY | IRQ_INT_TX_FIFO_EMPTY,
};
#define PHY_RESET_DELAYS_PROPERTY	"hisilicon,phy-reset-delays-us"

enum phy_reset_delays {
	PRE_DELAY,
	PULSE,
	POST_DELAY,
	DELAYS_NUM,
};

struct hisi_femac_queue {
	struct sk_buff **skb;
	dma_addr_t *dma_phys;
	int num;
	unsigned int head;
	unsigned int tail;
};

struct hisi_femac_priv {
	void __iomem *port_base;
	void __iomem *glb_base;
	struct clk *clk;
	struct reset_control *mac_rst;
	struct reset_control *phy_rst;
	u32 phy_reset_delays[DELAYS_NUM];
	const struct hisi_femac_soc_data *soc;
	struct clk *phy_clk;
	struct regmap *peri;
	struct mii_bus *mii_bus;
	u32 phy_addr;
	u32 trim;
	bool trim_present;
	spinlock_t irq_lock;
	u32 link_status;

	struct device *dev;
	struct net_device *ndev;

	struct hisi_femac_queue txq;
	struct hisi_femac_queue rxq;
	u32 tx_fifo_used_cnt;
	struct napi_struct napi;
};

static void hisi_femac_port_init(struct hisi_femac_priv *priv);
static void hisi_femac_net_set_rx_mode(struct net_device *dev);

static u32 hisi_femac_irq_mask(struct hisi_femac_priv *priv)
{
	return priv->soc ? priv->soc->irq_mask : DEF_INT_MASK;
}

static void hisi_femac_irq_enable(struct hisi_femac_priv *priv, int irqs)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&priv->irq_lock, flags);
	val = readl(priv->glb_base + GLB_IRQ_ENA);
	writel(val | irqs, priv->glb_base + GLB_IRQ_ENA);
	spin_unlock_irqrestore(&priv->irq_lock, flags);
}

static void hisi_femac_irq_disable(struct hisi_femac_priv *priv, int irqs)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&priv->irq_lock, flags);
	val = readl(priv->glb_base + GLB_IRQ_ENA);
	writel(val & (~irqs), priv->glb_base + GLB_IRQ_ENA);
	spin_unlock_irqrestore(&priv->irq_lock, flags);
}

static void hisi_femac_tx_dma_unmap(struct hisi_femac_priv *priv,
				    struct sk_buff *skb, unsigned int pos)
{
	dma_addr_t dma_addr;

	dma_addr = priv->txq.dma_phys[pos];
	dma_unmap_single(priv->dev, dma_addr, skb->len, DMA_TO_DEVICE);
}

static void hisi_femac_xmit_reclaim(struct net_device *dev)
{
	struct sk_buff *skb;
	struct hisi_femac_priv *priv = netdev_priv(dev);
	struct hisi_femac_queue *txq = &priv->txq;
	unsigned int bytes_compl = 0, pkts_compl = 0;
	u32 val;

	netif_tx_lock(dev);

	val = readl(priv->port_base + ADDRQ_STAT) & TX_CNT_INUSE_MASK;
	while (val < priv->tx_fifo_used_cnt) {
		skb = txq->skb[txq->tail];
		if (unlikely(!skb)) {
			netdev_err(dev, "xmitq_cnt_inuse=%d, tx_fifo_used=%d\n",
				   val, priv->tx_fifo_used_cnt);
			break;
		}
		hisi_femac_tx_dma_unmap(priv, skb, txq->tail);
		pkts_compl++;
		bytes_compl += skb->len;
		dev_kfree_skb_any(skb);

		priv->tx_fifo_used_cnt--;

		val = readl(priv->port_base + ADDRQ_STAT) & TX_CNT_INUSE_MASK;
		txq->skb[txq->tail] = NULL;
		txq->tail = (txq->tail + 1) % txq->num;
	}

	netdev_completed_queue(dev, pkts_compl, bytes_compl);

	if (unlikely(netif_queue_stopped(dev)) && pkts_compl)
		netif_wake_queue(dev);

	netif_tx_unlock(dev);
}

static void hisi_femac_adjust_link(struct net_device *dev)
{
	struct hisi_femac_priv *priv = netdev_priv(dev);
	struct phy_device *phy = dev->phydev;
	u32 status = 0;

	if (phy->link)
		status |= MAC_PORTSET_LINKED;
	if (phy->duplex == DUPLEX_FULL)
		status |= MAC_PORTSET_DUPLEX_FULL;
	if (phy->speed == SPEED_100)
		status |= MAC_PORTSET_SPEED_100M;

	if ((status != priv->link_status) &&
	    ((status | priv->link_status) & MAC_PORTSET_LINKED)) {
		writel(status, priv->port_base + MAC_PORTSET);
		priv->link_status = status;
		phy_print_status(phy);
	}
}

static void hisi_femac_rx_refill(struct hisi_femac_priv *priv)
{
	struct hisi_femac_queue *rxq = &priv->rxq;
	struct sk_buff *skb;
	u32 pos;
	u32 len = MAX_FRAME_SIZE;
	dma_addr_t addr;

	pos = rxq->head;
	while (readl(priv->port_base + ADDRQ_STAT) & BIT_RX_READY) {
		if (!CIRC_SPACE(pos, rxq->tail, rxq->num))
			break;
		if (unlikely(rxq->skb[pos])) {
			netdev_err(priv->ndev, "err skb[%d]=%p\n",
				   pos, rxq->skb[pos]);
			break;
		}
		skb = netdev_alloc_skb_ip_align(priv->ndev, len);
		if (unlikely(!skb))
			break;

		addr = dma_map_single(priv->dev, skb->data, len,
				      DMA_FROM_DEVICE);
		if (dma_mapping_error(priv->dev, addr)) {
			dev_kfree_skb_any(skb);
			break;
		}
		rxq->dma_phys[pos] = addr;
		rxq->skb[pos] = skb;
		writel(addr, priv->port_base + IQ_ADDR);
		pos = (pos + 1) % rxq->num;
	}
	rxq->head = pos;
}

static int hisi_femac_rx(struct net_device *dev, int limit)
{
	struct hisi_femac_priv *priv = netdev_priv(dev);
	struct hisi_femac_queue *rxq = &priv->rxq;
	struct sk_buff *skb;
	dma_addr_t addr;
	u32 rx_pkt_info, pos, len, rx_pkts_num = 0;

	pos = rxq->tail;
	while (readl(priv->glb_base + GLB_IRQ_RAW) & IRQ_INT_RX_RDY) {
		rx_pkt_info = readl(priv->port_base + IQFRM_DES);
		len = rx_pkt_info & RX_FRAME_LEN_MASK;
		if (unlikely(len < ETH_FCS_LEN || len > MAX_FRAME_SIZE))
			len = 0;
		else
			len -= ETH_FCS_LEN;

		/* tell hardware we will deal with this packet */
		writel(IRQ_INT_RX_RDY, priv->glb_base + GLB_IRQ_RAW);

		rx_pkts_num++;

		skb = rxq->skb[pos];
		if (unlikely(!skb)) {
			netdev_err(dev, "rx skb NULL. pos=%d\n", pos);
			break;
		}
		rxq->skb[pos] = NULL;

		addr = rxq->dma_phys[pos];
		dma_unmap_single(priv->dev, addr, MAX_FRAME_SIZE,
				 DMA_FROM_DEVICE);
		if (unlikely(len < ETH_HLEN)) {
			netdev_err(dev, "rcv len err, len = %d\n", len);
			dev->stats.rx_errors++;
			dev->stats.rx_length_errors++;
			dev_kfree_skb_any(skb);
			goto next;
		}
		skb_put(skb, len);

		skb->protocol = eth_type_trans(skb, dev);
		napi_gro_receive(&priv->napi, skb);
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += len;
next:
		pos = (pos + 1) % rxq->num;
		if (rx_pkts_num >= limit)
			break;
	}
	rxq->tail = pos;

	hisi_femac_rx_refill(priv);

	return rx_pkts_num;
}

static int hisi_femac_poll(struct napi_struct *napi, int budget)
{
	struct hisi_femac_priv *priv = container_of(napi,
					struct hisi_femac_priv, napi);
	struct net_device *dev = priv->ndev;
	int work_done = 0, task = budget;
	int ints, num;
	u32 mask = hisi_femac_irq_mask(priv);

	if (priv->soc && !budget) {
		hisi_femac_xmit_reclaim(dev);
		return 0;
	}

	do {
		hisi_femac_xmit_reclaim(dev);
		num = hisi_femac_rx(dev, task);
		work_done += num;
		task -= num;
		if (work_done >= budget)
			break;

		ints = readl(priv->glb_base + GLB_IRQ_RAW);
		writel(ints & mask,
		       priv->glb_base + GLB_IRQ_RAW);
	} while ((ints & mask) && !priv->soc);

	if (work_done < budget) {
		if (napi_complete_done(napi, work_done))
			hisi_femac_irq_enable(priv, mask & (~IRQ_INT_TX_PER_PACKET));
	}

	return work_done;
}

static irqreturn_t hisi_femac_interrupt(int irq, void *dev_id)
{
	int ints;
	struct net_device *dev = (struct net_device *)dev_id;
	struct hisi_femac_priv *priv = netdev_priv(dev);

	ints = readl(priv->glb_base + GLB_IRQ_RAW);
	if (priv->soc)
		ints &= readl(priv->glb_base + GLB_IRQ_ENA);

	if (likely(ints & hisi_femac_irq_mask(priv))) {
		writel(ints & hisi_femac_irq_mask(priv),
		       priv->glb_base + GLB_IRQ_RAW);
		hisi_femac_irq_disable(priv, hisi_femac_irq_mask(priv));
		napi_schedule(&priv->napi);
	} else if (priv->soc) {
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int hisi_femac_init_queue(struct device *dev,
				 struct hisi_femac_queue *queue,
				 unsigned int num)
{
	queue->skb = devm_kcalloc(dev, num, sizeof(struct sk_buff *),
				  GFP_KERNEL);
	if (!queue->skb)
		return -ENOMEM;

	queue->dma_phys = devm_kcalloc(dev, num, sizeof(dma_addr_t),
				       GFP_KERNEL);
	if (!queue->dma_phys)
		return -ENOMEM;

	queue->num = num;
	queue->head = 0;
	queue->tail = 0;

	return 0;
}

static int hisi_femac_init_tx_and_rx_queues(struct hisi_femac_priv *priv)
{
	int ret;

	ret = hisi_femac_init_queue(priv->dev, &priv->txq, TXQ_NUM);
	if (ret)
		return ret;

	ret = hisi_femac_init_queue(priv->dev, &priv->rxq, RXQ_NUM);
	if (ret)
		return ret;

	priv->tx_fifo_used_cnt = 0;

	return 0;
}

static void hisi_femac_free_skb_rings(struct hisi_femac_priv *priv)
{
	struct hisi_femac_queue *txq = &priv->txq;
	struct hisi_femac_queue *rxq = &priv->rxq;
	struct sk_buff *skb;
	dma_addr_t dma_addr;
	u32 pos;

	pos = rxq->tail;
	while (pos != rxq->head) {
		skb = rxq->skb[pos];
		if (unlikely(!skb)) {
			netdev_err(priv->ndev, "NULL rx skb. pos=%d, head=%d\n",
				   pos, rxq->head);
			pos = (pos + 1) % rxq->num;
			continue;
		}

		dma_addr = rxq->dma_phys[pos];
		dma_unmap_single(priv->dev, dma_addr, MAX_FRAME_SIZE,
				 DMA_FROM_DEVICE);

		dev_kfree_skb_any(skb);
		rxq->skb[pos] = NULL;
		pos = (pos + 1) % rxq->num;
	}
	rxq->tail = pos;

	pos = txq->tail;
	while (pos != txq->head) {
		skb = txq->skb[pos];
		if (unlikely(!skb)) {
			netdev_err(priv->ndev, "NULL tx skb. pos=%d, head=%d\n",
				   pos, txq->head);
			pos = (pos + 1) % txq->num;
			continue;
		}
		hisi_femac_tx_dma_unmap(priv, skb, pos);
		dev_kfree_skb_any(skb);
		txq->skb[pos] = NULL;
		pos = (pos + 1) % txq->num;
	}
	txq->tail = pos;
	priv->tx_fifo_used_cnt = 0;
}

static int hisi_femac_set_hw_mac_addr(struct hisi_femac_priv *priv,
				      const unsigned char *mac)
{
	u32 reg;

	reg = mac[1] | (mac[0] << 8);
	writel(reg, priv->glb_base + GLB_HOSTMAC_H16);

	reg = mac[5] | (mac[4] << 8) | (mac[3] << 16) | (mac[2] << 24);
	writel(reg, priv->glb_base + GLB_HOSTMAC_L32);

	return 0;
}

static int hisi_femac_port_reset(struct hisi_femac_priv *priv)
{
	u32 val;

	val = readl(priv->glb_base + GLB_SOFT_RESET);
	val |= SOFT_RESET_ALL;
	writel(val, priv->glb_base + GLB_SOFT_RESET);

	if (priv->soc)
		usleep_range(1000, 10000);
	else
		usleep_range(500, 800);

	val &= ~SOFT_RESET_ALL;
	writel(val, priv->glb_base + GLB_SOFT_RESET);
	if (priv->soc) {
		/* BSP hieth.c:135-147 uses two reset pulses on the SF MAC. */
		usleep_range(1000, 10000);
		writel(val | SOFT_RESET_ALL, priv->glb_base + GLB_SOFT_RESET);
		usleep_range(1000, 10000);
		writel(val, priv->glb_base + GLB_SOFT_RESET);
	}

	return 0;
}

static int hisi_femac_net_open(struct net_device *dev)
{
	struct hisi_femac_priv *priv = netdev_priv(dev);

	hisi_femac_port_reset(priv);
	if (priv->soc) {
		hisi_femac_port_init(priv);
		hisi_femac_net_set_rx_mode(dev);
	}
	hisi_femac_set_hw_mac_addr(priv, dev->dev_addr);
	hisi_femac_rx_refill(priv);

	netif_carrier_off(dev);
	netdev_reset_queue(dev);
	netif_start_queue(dev);
	napi_enable(&priv->napi);

	priv->link_status = 0;
	if (dev->phydev)
		phy_start(dev->phydev);

	writel(IRQ_ENA_PORT0_MASK, priv->glb_base + GLB_IRQ_RAW);
	hisi_femac_irq_enable(priv, IRQ_ENA_ALL | IRQ_ENA_PORT0 |
			     hisi_femac_irq_mask(priv));

	return 0;
}

static int hisi_femac_net_close(struct net_device *dev)
{
	struct hisi_femac_priv *priv = netdev_priv(dev);

	hisi_femac_irq_disable(priv, IRQ_ENA_PORT0);
	if (priv->soc) {
		hisi_femac_irq_disable(priv, IRQ_ENA_ALL | IRQ_ENA_PORT0_MASK);
		synchronize_irq(dev->irq);
	}

	if (dev->phydev)
		phy_stop(dev->phydev);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);
	if (priv->soc)
		hisi_femac_port_reset(priv);

	hisi_femac_free_skb_rings(priv);
	if (priv->soc)
		netdev_reset_queue(dev);

	return 0;
}

static netdev_tx_t hisi_femac_net_xmit(struct sk_buff *skb,
				       struct net_device *dev)
{
	struct hisi_femac_priv *priv = netdev_priv(dev);
	struct hisi_femac_queue *txq = &priv->txq;
	dma_addr_t addr;
	u32 val;

	val = readl(priv->port_base + ADDRQ_STAT);
	val &= BIT_TX_READY;
	if (!val) {
		hisi_femac_irq_enable(priv, priv->soc ? IRQ_INT_TX_FIFO_EMPTY :
				     IRQ_INT_TX_PER_PACKET);
		dev->stats.tx_dropped++;
		dev->stats.tx_fifo_errors++;
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}

	if (unlikely(!CIRC_SPACE(txq->head, txq->tail,
				 txq->num))) {
		hisi_femac_irq_enable(priv, priv->soc ? IRQ_INT_TX_FIFO_EMPTY :
				     IRQ_INT_TX_PER_PACKET);
		dev->stats.tx_dropped++;
		dev->stats.tx_fifo_errors++;
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}

	addr = dma_map_single(priv->dev, skb->data,
			      skb->len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(priv->dev, addr))) {
		dev_kfree_skb_any(skb);
		dev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	txq->dma_phys[txq->head] = addr;

	txq->skb[txq->head] = skb;
	txq->head = (txq->head + 1) % txq->num;

	writel(addr, priv->port_base + EQ_ADDR);
	writel(skb->len + ETH_FCS_LEN, priv->port_base + EQFRM_LEN);

	priv->tx_fifo_used_cnt++;

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;
	netdev_sent_queue(dev, skb->len);

	return NETDEV_TX_OK;
}

static int hisi_femac_set_mac_address(struct net_device *dev, void *p)
{
	struct hisi_femac_priv *priv = netdev_priv(dev);
	struct sockaddr *skaddr = p;

	if (!is_valid_ether_addr(skaddr->sa_data))
		return -EADDRNOTAVAIL;

	eth_hw_addr_set(dev, skaddr->sa_data);
	dev->addr_assign_type &= ~NET_ADDR_RANDOM;

	hisi_femac_set_hw_mac_addr(priv, dev->dev_addr);

	return 0;
}

static void hisi_femac_enable_hw_addr_filter(struct hisi_femac_priv *priv,
					     unsigned int reg_n, bool enable)
{
	u32 val;

	val = readl(priv->glb_base + GLB_MAC_H16(reg_n));
	if (enable)
		val |= BIT_MACFLT_ENA;
	else
		val &= ~BIT_MACFLT_ENA;
	writel(val, priv->glb_base + GLB_MAC_H16(reg_n));
}

static void hisi_femac_set_hw_addr_filter(struct hisi_femac_priv *priv,
					  unsigned char *addr,
					  unsigned int reg_n)
{
	unsigned int high, low;
	u32 val;

	high = GLB_MAC_H16(reg_n);
	low = GLB_MAC_L32(reg_n);

	val = (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | addr[5];
	writel(val, priv->glb_base + low);

	val = readl(priv->glb_base + high);
	val &= ~MACFLT_HI16_MASK;
	val |= ((addr[0] << 8) | addr[1]);
	val |= (BIT_MACFLT_ENA | BIT_MACFLT_FW2CPU);
	writel(val, priv->glb_base + high);
}

static void hisi_femac_set_promisc_mode(struct hisi_femac_priv *priv,
					bool promisc_mode)
{
	u32 val;

	val = readl(priv->glb_base + GLB_FWCTRL);
	if (promisc_mode)
		val |= FWCTRL_FWALL2CPU;
	else
		val &= ~FWCTRL_FWALL2CPU;
	writel(val, priv->glb_base + GLB_FWCTRL);
}

/* Handle multiple multicast addresses (perfect filtering)*/
static void hisi_femac_set_mc_addr_filter(struct hisi_femac_priv *priv)
{
	struct net_device *dev = priv->ndev;
	u32 val;

	val = readl(priv->glb_base + GLB_MACTCTRL);
	if ((netdev_mc_count(dev) > MAX_MULTICAST_ADDRESSES) ||
	    (dev->flags & IFF_ALLMULTI)) {
		val |= MACTCTRL_MULTI2CPU;
	} else {
		int reg = MAX_UNICAST_ADDRESSES;
		int i;
		struct netdev_hw_addr *ha;

		for (i = reg; i < MAX_MAC_FILTER_NUM; i++)
			hisi_femac_enable_hw_addr_filter(priv, i, false);

		netdev_for_each_mc_addr(ha, dev) {
			hisi_femac_set_hw_addr_filter(priv, ha->addr, reg);
			reg++;
		}
		val &= ~MACTCTRL_MULTI2CPU;
	}
	writel(val, priv->glb_base + GLB_MACTCTRL);
}

/* Handle multiple unicast addresses (perfect filtering)*/
static void hisi_femac_set_uc_addr_filter(struct hisi_femac_priv *priv)
{
	struct net_device *dev = priv->ndev;
	u32 val;

	val = readl(priv->glb_base + GLB_MACTCTRL);
	if (netdev_uc_count(dev) > MAX_UNICAST_ADDRESSES) {
		val |= MACTCTRL_UNI2CPU;
	} else {
		int reg = 0;
		int i;
		struct netdev_hw_addr *ha;

		for (i = reg; i < MAX_UNICAST_ADDRESSES; i++)
			hisi_femac_enable_hw_addr_filter(priv, i, false);

		netdev_for_each_uc_addr(ha, dev) {
			hisi_femac_set_hw_addr_filter(priv, ha->addr, reg);
			reg++;
		}
		val &= ~MACTCTRL_UNI2CPU;
	}
	writel(val, priv->glb_base + GLB_MACTCTRL);
}

static void hisi_femac_net_set_rx_mode(struct net_device *dev)
{
	struct hisi_femac_priv *priv = netdev_priv(dev);

	if (dev->flags & IFF_PROMISC) {
		hisi_femac_set_promisc_mode(priv, true);
	} else {
		hisi_femac_set_promisc_mode(priv, false);
		hisi_femac_set_mc_addr_filter(priv);
		hisi_femac_set_uc_addr_filter(priv);
	}
}

static const struct ethtool_ops hisi_femac_ethtools_ops = {
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
};

static const struct net_device_ops hisi_femac_netdev_ops = {
	.ndo_open		= hisi_femac_net_open,
	.ndo_stop		= hisi_femac_net_close,
	.ndo_start_xmit		= hisi_femac_net_xmit,
	.ndo_eth_ioctl		= phy_do_ioctl_running,
	.ndo_set_mac_address	= hisi_femac_set_mac_address,
	.ndo_set_rx_mode	= hisi_femac_net_set_rx_mode,
};

static int hisi_femac_mv310_mdio_ready(struct hisi_femac_priv *priv)
{
	u32 val;

	return readl_poll_timeout(priv->port_base + MV310_MDIO_CTRL, val,
				  val & MV310_MDIO_FINISH, 1, 1000);
}

static int hisi_femac_mv310_mdio_read(struct mii_bus *bus, int addr, int reg)
{
	struct hisi_femac_priv *priv = bus->priv;
	int ret;

	ret = hisi_femac_mv310_mdio_ready(priv);
	if (ret)
		return ret;
	writel((addr << 8) | reg, priv->port_base + MV310_MDIO_CTRL);
	ret = hisi_femac_mv310_mdio_ready(priv);
	if (ret)
		return ret;
	return readl(priv->port_base + MV310_MDIO_DATA) & 0xffff;
}

static int hisi_femac_mv310_mdio_write(struct mii_bus *bus, int addr, int reg,
				     u16 value)
{
	struct hisi_femac_priv *priv = bus->priv;
	int ret;

	ret = hisi_femac_mv310_mdio_ready(priv);
	if (ret)
		return ret;
	writel(((u32)value << 16) | MV310_MDIO_WRITE | (addr << 8) | reg,
	       priv->port_base + MV310_MDIO_CTRL);
	/* BSP hieth/mdio.c:103-109 requires 500 us after a PHY write. */
	usleep_range(500, 600);
	return hisi_femac_mv310_mdio_ready(priv);
}

static int hisi_femac_mv310_exp_read(struct hisi_femac_priv *priv, u16 reg)
{
	int ret;

	ret = hisi_femac_mv310_mdio_write(priv->mii_bus, priv->phy_addr,
					FESTA_EXP_ADDR, reg);
	if (ret)
		return ret;
	return hisi_femac_mv310_mdio_read(priv->mii_bus, priv->phy_addr,
				       FESTA_EXP_DATA);
}

static int hisi_femac_mv310_exp_write(struct hisi_femac_priv *priv, u16 reg,
				    u16 value)
{
	int ret;

	ret = hisi_femac_mv310_mdio_write(priv->mii_bus, priv->phy_addr,
					FESTA_EXP_ADDR, reg);
	if (ret)
		return ret;
	return hisi_femac_mv310_mdio_write(priv->mii_bus, priv->phy_addr,
					FESTA_EXP_DATA, value);
}

static int hisi_femac_mv310_trim(struct hisi_femac_priv *priv)
{
	static const u16 masks[] = { 0x1f, 0x7, 0x3f };
	int i, val, ret;

	/* BSP hieth/phy.c:82-145: absent OTP means wait for ATE auto-trim;
	 * do not manufacture trim values from a neighbouring SoC.
	 */
	if (!priv->trim_present) {
		for (i = 0; i < 3; i++) {
			msleep(250);
			val = hisi_femac_mv310_exp_read(priv, 0x3057);
			if (val < 0)
				return val;
			if (val & BIT(0))
				return 0;
		}
		return -ETIMEDOUT;
	}

	for (i = 0; i < ARRAY_SIZE(masks); i++) {
		val = hisi_femac_mv310_exp_read(priv, 0x3050 + i);
		if (val < 0)
			return val;
		val = (val & ~masks[i]) | ((priv->trim >> (8 * i)) & masks[i]);
		ret = hisi_femac_mv310_exp_write(priv, 0x3050 + i, val);
		if (ret)
			return ret;
	}
	val = hisi_femac_mv310_exp_read(priv, 0x3053);
	if (val < 0)
		return val;
	ret = hisi_femac_mv310_exp_write(priv, 0x3053, val | BIT(0));
	if (ret)
		return ret;
	ret = read_poll_timeout(hisi_femac_mv310_exp_read, val,
				val < 0 || (val & BIT(1)), 100, 450000, false,
				priv, 0x3053);
	return val < 0 ? val : ret;
}

static int hisi_femac_mv310_mdio_reset(struct mii_bus *bus)
{
	struct hisi_femac_priv *priv = bus->priv;
	int ret;

	/* Hardware reset value, once after a real MAC/PHY reset, as in
	 * BSP hieth/mdio.h:44-46. Never rewrite this on a busy timeout.
	 */
	writel(MV310_MDIO_FINISH, priv->port_base + MV310_MDIO_CTRL);
	ret = hisi_femac_mv310_trim(priv);
	if (ret)
		return dev_err_probe(priv->dev, ret, "FEPHY trim failed\n");
	msleep(5);
	return 0;
}

static void hisi_femac_mv310_power_off(struct hisi_femac_priv *priv)
{
	reset_control_assert(priv->mac_rst);
	clk_disable_unprepare(priv->clk);
	reset_control_assert(priv->phy_rst);
	clk_disable_unprepare(priv->phy_clk);
}

static int hisi_femac_mv310_power_on(struct hisi_femac_priv *priv)
{
	int ret;

	ret = reset_control_assert(priv->mac_rst);
	if (ret)
		return ret;
	/* Ensure the MAC clock is off while the PHY address is latched,
	 * including when the bootloader left it enabled (hieth.c:1501).
	 */
	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;
	clk_disable_unprepare(priv->clk);
	/* BSP hieth/phy.c:392-432, including the explicit MV310 branch. */
	ret = regmap_update_bits(priv->peri, MV310_PHY_LDO,
				 MV310_PHY_LDO_MASK, MV310_PHY_LDO_VALUE);
	if (ret)
		return ret;
	ret = clk_prepare_enable(priv->phy_clk);
	if (ret)
		return ret;
	ret = regmap_update_bits(priv->peri, MV310_PHY_ADDR, 0x1f,
				 priv->phy_addr);
	if (ret)
		goto reset_phy;
	ret = reset_control_assert(priv->phy_rst);
	if (ret)
		goto reset_phy;
	usleep_range(10, 1000);
	ret = reset_control_deassert(priv->phy_rst);
	if (ret)
		goto reset_phy;
	msleep(20);
	ret = clk_prepare_enable(priv->clk);
	if (ret)
		goto reset_phy;
	ret = reset_control_deassert(priv->mac_rst);
	if (ret)
		goto disable_mac;
	/* BSP hieth/hieth.c:1509: MDCK must run for 5 ms before MDIO. */
	msleep(5);
	return 0;

disable_mac:
	clk_disable_unprepare(priv->clk);
reset_phy:
	reset_control_assert(priv->phy_rst);
	clk_disable_unprepare(priv->phy_clk);
	return ret;
}

static int hisi_femac_mv310_init(struct hisi_femac_priv *priv)
{
	struct device *dev = priv->dev;
	struct device_node *mdio, *phy, *parent = NULL, *trim;
	phy_interface_t interface;
	u32 raw;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	priv->peri = syscon_regmap_lookup_by_phandle(dev->of_node,
						   "hisilicon,peri-syscon");
	if (IS_ERR(priv->peri))
		return PTR_ERR(priv->peri);
	priv->clk = devm_clk_get(dev, "mac");
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);
	priv->phy_clk = devm_clk_get(dev, "phy");
	if (IS_ERR(priv->phy_clk))
		return PTR_ERR(priv->phy_clk);
	priv->mac_rst = devm_reset_control_get_exclusive(dev, "mac");
	if (IS_ERR(priv->mac_rst))
		return PTR_ERR(priv->mac_rst);
	priv->phy_rst = devm_reset_control_get_exclusive(dev, "phy");
	if (IS_ERR(priv->phy_rst))
		return PTR_ERR(priv->phy_rst);
	ret = of_get_phy_mode(dev->of_node, &interface);
	if (ret || interface != PHY_INTERFACE_MODE_MII)
		return -EINVAL;
	mdio = of_get_child_by_name(dev->of_node, "mdio");
	if (!mdio)
		return -EINVAL;
	phy = of_parse_phandle(dev->of_node, "phy-handle", 0);
	if (!phy) {
		ret = -EINVAL;
		goto put_nodes;
	}
	parent = of_get_parent(phy);
	ret = of_property_read_u32(phy, "reg", &priv->phy_addr);
	if (parent != mdio || ret || priv->phy_addr >= PHY_MAX_ADDR ||
	    !of_device_is_available(phy) || !of_device_is_available(mdio)) {
		ret = -EINVAL;
		goto put_nodes;
	}

	/* Preserve the bootloader handoff policy in BSP hieth.c:1367-1404.
	 * A missing property is NOT equivalent to a supplied zero OTP word.
	 */
	trim = of_find_node_by_path("/chiptrim");
	if (trim && !of_property_read_u32(trim, "chiptrim0", &raw)) {
		priv->trim = raw ? (((raw >> 11) & 0x1f) |
					  (((raw >> 8) & 0x7) << 8) |
					  ((raw & 0x3f) << 16)) : 0x16030e;
		priv->trim_present = !!priv->trim;
	}
	of_node_put(trim);
	priv->mii_bus = devm_mdiobus_alloc(dev);
	if (!priv->mii_bus) {
		ret = -ENOMEM;
		goto put_nodes;
	}
	priv->mii_bus->name = "MV310 FEPHY MDIO";
	priv->mii_bus->parent = dev;
	priv->mii_bus->priv = priv;
	priv->mii_bus->read = hisi_femac_mv310_mdio_read;
	priv->mii_bus->write = hisi_femac_mv310_mdio_write;
	priv->mii_bus->reset = hisi_femac_mv310_mdio_reset;
	snprintf(priv->mii_bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));
	ret = hisi_femac_mv310_power_on(priv);
	if (ret)
		goto put_nodes;
	/* mdiobus reset/trim runs before of_mdiobus_register probes PHY IDs. */
	ret = of_mdiobus_register(priv->mii_bus, mdio);
	if (ret)
		hisi_femac_mv310_power_off(priv);
put_nodes:
	of_node_put(parent);
	of_node_put(phy);
	of_node_put(mdio);
	return ret;
}

static void hisi_femac_core_reset(struct hisi_femac_priv *priv)
{
	reset_control_assert(priv->mac_rst);
	reset_control_deassert(priv->mac_rst);
}

static void hisi_femac_sleep_us(u32 time_us)
{
	u32 time_ms;

	if (!time_us)
		return;

	time_ms = DIV_ROUND_UP(time_us, 1000);
	if (time_ms < 20)
		usleep_range(time_us, time_us + 500);
	else
		msleep(time_ms);
}

static void hisi_femac_phy_reset(struct hisi_femac_priv *priv)
{
	/* To make sure PHY hardware reset success,
	 * we must keep PHY in deassert state first and
	 * then complete the hardware reset operation
	 */
	reset_control_deassert(priv->phy_rst);
	hisi_femac_sleep_us(priv->phy_reset_delays[PRE_DELAY]);

	reset_control_assert(priv->phy_rst);
	/* delay some time to ensure reset ok,
	 * this depends on PHY hardware feature
	 */
	hisi_femac_sleep_us(priv->phy_reset_delays[PULSE]);
	reset_control_deassert(priv->phy_rst);
	/* delay some time to ensure later MDIO access */
	hisi_femac_sleep_us(priv->phy_reset_delays[POST_DELAY]);
}

static void hisi_femac_port_init(struct hisi_femac_priv *priv)
{
	u32 val;

	/* MAC gets link status info and phy mode by software config */
	val = MAC_PORTSEL_STAT_CPU;
	if (priv->ndev->phydev->interface == PHY_INTERFACE_MODE_RMII)
		val |= MAC_PORTSEL_RMII;
	writel(val, priv->port_base + MAC_PORTSEL);

	/*clear all interrupt status */
	writel(IRQ_ENA_PORT0_MASK, priv->glb_base + GLB_IRQ_RAW);
	hisi_femac_irq_disable(priv, IRQ_ENA_PORT0_MASK | IRQ_ENA_PORT0);

	val = readl(priv->glb_base + GLB_FWCTRL);
	val &= ~(FWCTRL_VLAN_ENABLE | FWCTRL_FWALL2CPU);
	val |= FWCTRL_FW2CPU_ENA;
	writel(val, priv->glb_base + GLB_FWCTRL);

	val = readl(priv->glb_base + GLB_MACTCTRL);
	val |= (MACTCTRL_BROAD2CPU | MACTCTRL_MACT_ENA);
	writel(val, priv->glb_base + GLB_MACTCTRL);

	val = readl(priv->port_base + MAC_SET);
	val &= ~MAX_FRAME_SIZE_MASK;
	val |= priv->soc ? MV310_MAX_FRAME_SIZE : MAX_FRAME_SIZE;
	writel(val, priv->port_base + MAC_SET);

	if (priv->soc) {
		/* BSP hieth.c:158-162,186-203; no coalescer programming. */
		val = readl(priv->glb_base + GLB_ENDIAN_MOD);
		writel(val | ENDIAN_IN_OUT, priv->glb_base + GLB_ENDIAN_MOD);
		val = readl(priv->port_base + MAC_TX_IPGCTRL);
		writel(val & ~TX_PRE_CNT_LIMIT, priv->port_base + MAC_TX_IPGCTRL);
		val = readl(priv->port_base + RX_COE_CTRL);
		writel(val & ~COE_PAYLOAD_DROP, priv->port_base + RX_COE_CTRL);
	} else {
		val = RX_COALESCED_TIMER |
			(RX_COALESCED_FRAMES << RX_COALESCED_FRAME_OFFSET);
		writel(val, priv->port_base + RX_COALESCE_SET);
	}

	val = (HW_RX_FIFO_DEPTH << RX_DEPTH_OFFSET) | HW_TX_FIFO_DEPTH;
	writel(val, priv->port_base + QLEN_SET);
}

static int hisi_femac_drv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct net_device *ndev;
	struct hisi_femac_priv *priv;
	struct phy_device *phy;
	int ret;

	ndev = alloc_etherdev(sizeof(*priv));
	if (!ndev)
		return -ENOMEM;

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	priv = netdev_priv(ndev);
	priv->dev = dev;
	priv->ndev = ndev;
	priv->soc = of_device_get_match_data(dev);
	spin_lock_init(&priv->irq_lock);

	priv->port_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->port_base)) {
		ret = PTR_ERR(priv->port_base);
		goto out_free_netdev;
	}

	if (priv->soc) {
		priv->glb_base = priv->port_base + priv->soc->glb_offset;
		ret = hisi_femac_mv310_init(priv);
		if (ret) {
			dev_err_probe(dev, ret, "MV310 MAC/PHY initialization failed\n");
			goto out_free_netdev;
		}
	} else {
		priv->glb_base = devm_platform_ioremap_resource(pdev, 1);
		if (IS_ERR(priv->glb_base)) {
			ret = PTR_ERR(priv->glb_base);
			goto out_free_netdev;
		}
		priv->clk = devm_clk_get(dev, NULL);
		if (IS_ERR(priv->clk)) {
			ret = PTR_ERR(priv->clk);
			goto out_free_netdev;
		}
		ret = clk_prepare_enable(priv->clk);
		if (ret)
			goto out_free_netdev;
		priv->mac_rst = devm_reset_control_get(dev, "mac");
		if (IS_ERR(priv->mac_rst)) {
			ret = PTR_ERR(priv->mac_rst);
			goto out_disable_clk;
		}
		hisi_femac_core_reset(priv);
		priv->phy_rst = devm_reset_control_get(dev, "phy");
		if (IS_ERR(priv->phy_rst)) {
			priv->phy_rst = NULL;
		} else {
			ret = of_property_read_u32_array(node,
						PHY_RESET_DELAYS_PROPERTY,
						priv->phy_reset_delays, DELAYS_NUM);
			if (ret)
				goto out_disable_clk;
			hisi_femac_phy_reset(priv);
		}
	}

	phy = of_phy_get_and_connect(ndev, node, hisi_femac_adjust_link);
	if (!phy) {
		dev_err(dev, "connect to PHY failed!\n");
		ret = -ENODEV;
		goto out_disable_clk;
	}

	phy_attached_print(phy, "phy_id=0x%.8lx, phy_mode=%s\n",
			   (unsigned long)phy->phy_id,
			   phy_modes(phy->interface));

	ret = of_get_ethdev_address(node, ndev);
	if (ret) {
		eth_hw_addr_random(ndev);
		dev_warn(dev, "using random MAC address %pM\n",
			 ndev->dev_addr);
	}

	ndev->watchdog_timeo = 6 * HZ;
	ndev->priv_flags |= IFF_UNICAST_FLT;
	if (priv->soc)
		ndev->max_mtu = ETH_DATA_LEN;
	ndev->netdev_ops = &hisi_femac_netdev_ops;
	ndev->ethtool_ops = &hisi_femac_ethtools_ops;
	netif_napi_add_weight(ndev, &priv->napi, hisi_femac_poll,
			      FEMAC_POLL_WEIGHT);

	hisi_femac_port_init(priv);

	ret = hisi_femac_init_tx_and_rx_queues(priv);
	if (ret)
		goto out_disconnect_phy;

	ndev->irq = platform_get_irq(pdev, 0);
	if (ndev->irq < 0) {
		ret = ndev->irq;
		goto out_disconnect_phy;
	}

	ret = devm_request_irq(dev, ndev->irq, hisi_femac_interrupt,
			       IRQF_SHARED, pdev->name, ndev);
	if (ret) {
		dev_err(dev, "devm_request_irq %d failed!\n", ndev->irq);
		goto out_disconnect_phy;
	}

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(dev, "register_netdev failed!\n");
		goto out_free_irq;
	}

	return ret;

out_free_irq:
	devm_free_irq(dev, ndev->irq, ndev);
out_disconnect_phy:
	netif_napi_del(&priv->napi);
	phy_disconnect(phy);
out_disable_clk:
	if (priv->soc) {
		mdiobus_unregister(priv->mii_bus);
		hisi_femac_mv310_power_off(priv);
	} else {
		clk_disable_unprepare(priv->clk);
	}
out_free_netdev:
	free_netdev(ndev);

	return ret;
}

static void hisi_femac_drv_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct hisi_femac_priv *priv = netdev_priv(ndev);

	unregister_netdev(ndev);
	devm_free_irq(&pdev->dev, ndev->irq, ndev);
	netif_napi_del(&priv->napi);
	phy_disconnect(ndev->phydev);
	if (priv->soc) {
		mdiobus_unregister(priv->mii_bus);
		hisi_femac_mv310_power_off(priv);
	} else {
		clk_disable_unprepare(priv->clk);
	}
	free_netdev(ndev);
}

#ifdef CONFIG_PM
static int hisi_femac_drv_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct hisi_femac_priv *priv = netdev_priv(ndev);

	disable_irq(ndev->irq);
	if (netif_running(ndev)) {
		hisi_femac_net_close(ndev);
		netif_device_detach(ndev);
	}

	if (priv->soc)
		hisi_femac_mv310_power_off(priv);
	else
		clk_disable_unprepare(priv->clk);

	return 0;
}

static int hisi_femac_drv_resume(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct hisi_femac_priv *priv = netdev_priv(ndev);

	if (priv->soc) {
		int ret;

		ret = hisi_femac_mv310_power_on(priv);
		if (ret)
			return ret;
		ret = hisi_femac_mv310_mdio_reset(priv->mii_bus);
		if (!ret)
			ret = phy_init_hw(ndev->phydev);
		if (ret) {
			hisi_femac_mv310_power_off(priv);
			return ret;
		}
	} else {
		clk_prepare_enable(priv->clk);
		if (priv->phy_rst)
			hisi_femac_phy_reset(priv);
	}

	if (netif_running(ndev)) {
		if (!priv->soc)
			hisi_femac_port_init(priv);
		hisi_femac_net_open(ndev);
		netif_device_attach(ndev);
	}
	enable_irq(ndev->irq);

	return 0;
}
#endif

static const struct of_device_id hisi_femac_match[] = {
	{.compatible = "hisilicon,hisi-femac-v1",},
	{.compatible = "hisilicon,hisi-femac-v2",},
	{.compatible = "hisilicon,hi3516cv300-femac",},
	{.compatible = "hisilicon,hi3798mv310-femac",
	 .data = &hisi_femac_mv310_data,},
	{},
};

MODULE_DEVICE_TABLE(of, hisi_femac_match);

static struct platform_driver hisi_femac_driver = {
	.driver = {
		.name = "hisi-femac",
		.of_match_table = hisi_femac_match,
	},
	.probe = hisi_femac_drv_probe,
	.remove_new = hisi_femac_drv_remove,
#ifdef CONFIG_PM
	.suspend = hisi_femac_drv_suspend,
	.resume = hisi_femac_drv_resume,
#endif
};

module_platform_driver(hisi_femac_driver);

MODULE_DESCRIPTION("Hisilicon Fast Ethernet MAC driver");
MODULE_AUTHOR("Dongpo Li <lidongpo@hisilicon.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:hisi-femac");
