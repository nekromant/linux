#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/usb/musb.h>
#include <linux/usb/of.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/workqueue.h>
#include "musb_core.h"

/* reg offsets */
#define  USBC_REG_o_ISCR	0x0400
#define  USBC_REG_o_PHYCTL	0x0404
#define  USBC_REG_o_PHYBIST	0x0408
#define  USBC_REG_o_PHYTUNE	0x040c

#define  USBC_REG_o_VEND0	0x0043

/* Interface Status and Control */
#define  USBC_BP_ISCR_VBUS_VALID_FROM_DATA	30
#define  USBC_BP_ISCR_VBUS_VALID_FROM_VBUS	29
#define  USBC_BP_ISCR_EXT_ID_STATUS		28
#define  USBC_BP_ISCR_EXT_DM_STATUS		27
#define  USBC_BP_ISCR_EXT_DP_STATUS		26
#define  USBC_BP_ISCR_MERGED_VBUS_STATUS	25
#define  USBC_BP_ISCR_MERGED_ID_STATUS		24

#define  USBC_BP_ISCR_ID_PULLUP_EN		17
#define  USBC_BP_ISCR_DPDM_PULLUP_EN		16
#define  USBC_BP_ISCR_FORCE_ID			14
#define  USBC_BP_ISCR_FORCE_VBUS_VALID		12
#define  USBC_BP_ISCR_VBUS_VALID_SRC		10

#define  USBC_BP_ISCR_HOSC_EN			7
#define  USBC_BP_ISCR_VBUS_CHANGE_DETECT	6
#define  USBC_BP_ISCR_ID_CHANGE_DETECT		5
#define  USBC_BP_ISCR_DPDM_CHANGE_DETECT	4
#define  USBC_BP_ISCR_IRQ_ENABLE		3
#define  USBC_BP_ISCR_VBUS_CHANGE_DETECT_EN	2
#define  USBC_BP_ISCR_ID_CHANGE_DETECT_EN	1
#define  USBC_BP_ISCR_DPDM_CHANGE_DETECT_EN	0

/* usb id type */
#define  USBC_ID_TYPE_DISABLE		0
#define  USBC_ID_TYPE_HOST		1
#define  USBC_ID_TYPE_DEVICE		2

/* usb vbus valid type */
#define  USBC_VBUS_TYPE_DISABLE		0
#define  USBC_VBUS_TYPE_LOW		1
#define  USBC_VBUS_TYPE_HIGH		2

/* usb io type */
#define  USBC_IO_TYPE_PIO		0
#define  USBC_IO_TYPE_DMA		1

/* usb ep type */
#define  USBC_EP_TYPE_IDLE		0
#define  USBC_EP_TYPE_EP0		1
#define  USBC_EP_TYPE_TX		2
#define  USBC_EP_TYPE_RX		3

/* vendor0 */
#define  USBC_BP_VEND0_DRQ_SEL		1
#define  USBC_BP_VEND0_BUS_SEL		0

struct sunxi_glue {
	struct device		*dev;
	struct platform_device	*musb;
	struct regmap		*sc;
	struct phy		*phy;
	struct usb_phy		*usb_phy;
	struct clk		*clk;
	struct gpio_desc	*id_det_gpio;
	struct gpio_desc	*vbus_det_gpio;
	int			id_det_irq;
	int			vbus_det_irq;
	struct delayed_work	detect;
};

static irqreturn_t sunxi_musb_interrupt(int irq, void *__hci)
{
	unsigned long   flags;
	irqreturn_t     retval = IRQ_NONE;
	struct musb     *musb = __hci;

	spin_lock_irqsave(&musb->lock, flags);

	/* read and clear interrupts 
	 * NOTE: clearing is necessary!
	 */
	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB);
	if (musb->int_usb)
		musb_writeb(musb->mregs, MUSB_INTRUSB, musb->int_usb);

	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX);
	if (musb->int_tx)
		musb_writew(musb->mregs, MUSB_INTRTX, musb->int_tx);

	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX);
	if (musb->int_rx)
		musb_writew(musb->mregs, MUSB_INTRRX, musb->int_rx);

	if (musb->int_usb || musb->int_tx || musb->int_rx)
		retval |= musb_interrupt(musb);

	spin_unlock_irqrestore(&musb->lock, flags);

	return retval;
}

static u32 __USBC_WakeUp_ClearChangeDetect(u32 reg_val)
{
	u32 temp = reg_val;

	temp &= ~(1 << USBC_BP_ISCR_VBUS_CHANGE_DETECT);
	temp &= ~(1 << USBC_BP_ISCR_ID_CHANGE_DETECT);
	temp &= ~(1 << USBC_BP_ISCR_DPDM_CHANGE_DETECT);

	return temp;
}

void USBC_EnableIdPullUp(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val |= (1 << USBC_BP_ISCR_ID_PULLUP_EN);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

void USBC_DisableIdPullUp(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(1 << USBC_BP_ISCR_ID_PULLUP_EN);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

static void USBC_EnableDpDmPullUp(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val |= (1 << USBC_BP_ISCR_DPDM_PULLUP_EN);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

static void __USBC_ForceVbusValidDisable(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(0x03 << USBC_BP_ISCR_FORCE_VBUS_VALID);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

static void __USBC_ForceVbusValidToLow(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(0x03 << USBC_BP_ISCR_FORCE_VBUS_VALID);
	reg_val |= (0x02 << USBC_BP_ISCR_FORCE_VBUS_VALID);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

static void __USBC_ForceVbusValidToHigh(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(0x03 << USBC_BP_ISCR_FORCE_VBUS_VALID);
	reg_val |= (0x03 << USBC_BP_ISCR_FORCE_VBUS_VALID);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

/* force vbus valid to (id_type) */
static void USBC_ForceVbusValid(__iomem void *base, u32 vbus_type)
{
	pr_debug("musb: %s(): vbus_type %s\n", __func__,
		vbus_type == USBC_VBUS_TYPE_LOW ? "low" :
		(vbus_type == USBC_VBUS_TYPE_HIGH ? "high" : "disable"));

	switch (vbus_type) {
	case USBC_VBUS_TYPE_LOW:
		__USBC_ForceVbusValidToLow(base);
		break;

	case USBC_VBUS_TYPE_HIGH:
		__USBC_ForceVbusValidToHigh(base);
		break;

	default:
		__USBC_ForceVbusValidDisable(base);
		break;
	}
}

static void __USBC_ForceIdDisable(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(0x03 << USBC_BP_ISCR_FORCE_ID);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

static void __USBC_ForceIdToLow(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(0x03 << USBC_BP_ISCR_FORCE_ID);
	reg_val |= (0x02 << USBC_BP_ISCR_FORCE_ID);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

static void __USBC_ForceIdToHigh(__iomem void *base)
{
	u32 reg_val;

	reg_val = musb_readl(base, USBC_REG_o_ISCR);
	reg_val &= ~(0x03 << USBC_BP_ISCR_FORCE_ID);
	reg_val |= (0x03 << USBC_BP_ISCR_FORCE_ID);
	reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val);
	musb_writel(base, USBC_REG_o_ISCR, reg_val);
}

/* force id to (id_type) */
static void USBC_ForceId(__iomem void *base, u32 id_type)
{
	pr_debug("musb %s(): id_type %s\n", __func__,
		id_type == USBC_ID_TYPE_HOST ? "host" :
		(id_type == USBC_ID_TYPE_DEVICE ? "device" : "disable"));

	switch (id_type) {
	case USBC_ID_TYPE_HOST:
		__USBC_ForceIdToLow(base);
		break;

	case USBC_ID_TYPE_DEVICE:
		__USBC_ForceIdToHigh(base);
		break;

	default:
		__USBC_ForceIdDisable(base);
		break;
	}
}

static void id_vbus_det_scan(struct work_struct *work)
{
	struct sunxi_glue *glue =
		container_of(work, struct sunxi_glue, detect.work);
	struct musb *musb = dev_get_drvdata(&glue->musb->dev);
	int id, vbus;

	id = gpiod_get_value_cansleep(glue->id_det_gpio);
	vbus = gpiod_get_value_cansleep(glue->vbus_det_gpio);
	pr_err("id %d vbus %d\n", id, vbus);

	if (id == 0) {
		/* Host mode */
		USBC_ForceId(musb->mregs, USBC_ID_TYPE_HOST);
		USBC_ForceVbusValid(musb->mregs, USBC_VBUS_TYPE_HIGH);
		dev_info(musb->controller, "Set USB VBUS power on\n");
		phy_power_on(musb->phy);
		msleep(10);
		u8 devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		dev_info(musb->controller, "DEVCTL %02x\n", (int)devctl);
		devctl |= MUSB_DEVCTL_SESSION;
		MUSB_HST_MODE(musb);
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
	} else {
		/* Device mode */
		USBC_ForceId(musb->mregs, USBC_ID_TYPE_DEVICE);
		USBC_ForceVbusValid(musb->mregs, USBC_VBUS_TYPE_LOW);
		dev_info(musb->controller, "Set USB VBUS power off\n");
		phy_power_off(musb->phy);
		msleep(10);
		u8 devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		dev_info(musb->controller, "DEVCTL %02x\n", (int)devctl);
	}
}

static irqreturn_t id_vbus_det_irq(int irq, void *dev_id)
{
	struct sunxi_glue *glue = dev_id;

	/* vbus or id changed, let the pins settle and then scan them */
	queue_delayed_work(system_wq, &glue->detect, 100);

	return IRQ_HANDLED;
}

static int sunxi_musb_init(struct musb *musb)
{
	struct device *dev = musb->controller;
	struct sunxi_glue *glue = dev_get_drvdata(dev->parent);
	int ret;

	musb->phy = glue->phy;
	musb->xceiv= glue->usb_phy;

	ret = devm_request_threaded_irq(dev, glue->id_det_irq,
		NULL, id_vbus_det_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"musb-id-det", glue);
	if (ret) {
		dev_err(glue->dev, "Error requesting id-det-irq: %d\n", ret);
		return ret;
	}

	ret = devm_request_threaded_irq(dev, glue->vbus_det_irq,
		NULL, id_vbus_det_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"musb-vbus-det", glue);
	if (ret) {
		dev_err(glue->dev, "Error requesting vbus-det-irq: %d\n", ret);
		return ret;
	}

	/* TODO: give nice name to bit #0 */
	ret = regmap_update_bits(glue->sc, 0, BIT(0), BIT(0));
	if (ret)
		return ret;

	ret = phy_init(glue->phy);
	if (ret)
		return ret;

	ret = usb_phy_init(glue->usb_phy);
	if (ret) 
		goto err_phy_exit;

	ret = clk_prepare_enable(glue->clk);
	if (ret)
		goto err_usb_phy_shutdown;

	musb->isr = sunxi_musb_interrupt;

	USBC_EnableDpDmPullUp(musb->mregs);
	USBC_EnableIdPullUp(musb->mregs);

	return 0;

err_usb_phy_shutdown:
	usb_phy_shutdown(glue->usb_phy);
err_phy_exit:
	phy_exit(glue->phy);
	return ret;
}

static int sunxi_musb_exit(struct musb *musb)
{
	struct device *dev = musb->controller;
	struct sunxi_glue *glue = dev_get_drvdata(dev->parent);

	cancel_delayed_work_sync(&glue->detect);
	clk_disable_unprepare(glue->clk);
	usb_phy_shutdown(glue->usb_phy);
	phy_exit(glue->phy);

	/* FIXME disable pull-ups */

	return 0;
}

static void sunxi_musb_enable(struct musb *musb)
{
	/* select PIO mode */
	musb_writeb(musb->mregs, USBC_REG_o_VEND0, 0);
}

static void sunxi_musb_disable(struct musb *musb)
{
	dev_dbg(musb->controller, "%s()\n", __func__);
}

static int sunxi_musb_set_mode(struct musb *musb, u8 musb_mode)
{
	dev_dbg(musb->controller, "%s(): musb_mode %d\n", __func__, musb_mode);

	switch (musb_mode) {
	case MUSB_HOST:
		USBC_ForceId(musb->mregs, USBC_ID_TYPE_HOST);
		USBC_ForceVbusValid(musb->mregs, USBC_VBUS_TYPE_HIGH);
		dev_info(musb->controller, "Set USB VBUS power on\n");
		phy_power_on(musb->phy);
		break;

	case MUSB_PERIPHERAL:
		USBC_ForceId(musb->mregs, USBC_ID_TYPE_DEVICE);
		USBC_ForceVbusValid(musb->mregs, USBC_VBUS_TYPE_HIGH);
		dev_info(musb->controller, "Set USB VBUS power off\n");
		phy_power_off(musb->phy);
		break;

	case MUSB_OTG:
	default:
		USBC_ForceId(musb->mregs, USBC_ID_TYPE_DEVICE);
		USBC_ForceVbusValid(musb->mregs, USBC_VBUS_TYPE_LOW);
		/* set vbus? */
		break;
	}

	return 0;
}

u32 sunxi_fifo_offset(u8 epnum)
{
	return (epnum * 4);
}

static const struct musb_platform_ops sunxi_ops = {
	.init		= sunxi_musb_init,
	.exit		= sunxi_musb_exit,

	.enable		= sunxi_musb_enable,
	.disable	= sunxi_musb_disable,

//	.fifo_offset	= sunxi_fifo_offset,

	.set_mode	= sunxi_musb_set_mode,
};

/* Allwinner OTG supports up to 5 endpoints */
#define SUNXI_MUSB_MAX_EP_NUM	6
#define SUNXI_MUSB_RAM_BITS	11

static struct musb_fifo_cfg sunxi_musb_mode_cfg[] = {
	MUSB_EP_FIFO_SINGLE(1, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(1, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(5, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(5, FIFO_RX, 512),
};

static struct musb_hdrc_config sunxi_musb_hdrc_config = {
	.fifo_cfg       = sunxi_musb_mode_cfg,
	.fifo_cfg_size  = ARRAY_SIZE(sunxi_musb_mode_cfg),
	.multipoint	= true,
	.dyn_fifo	= true,
	.soft_con       = true,
	.num_eps	= SUNXI_MUSB_MAX_EP_NUM,
	.ram_bits	= SUNXI_MUSB_RAM_BITS,
	.dma		= 0,
};

static int sunxi_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	pdata;
	struct platform_device_info	pinfo;
	struct sunxi_glue		*glue;
	struct device_node		*np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "Error no device tree node found\n");
		return -EINVAL;
	}

	memset(&pdata, 0, sizeof(pdata));
	switch (of_usb_get_dr_mode(np)) {
		case USB_DR_MODE_HOST:
			pdata.mode = MUSB_HOST;
			break;

		case USB_DR_MODE_PERIPHERAL:
			pdata.mode = MUSB_PERIPHERAL;
			break;

		case USB_DR_MODE_OTG:
			pdata.mode = MUSB_OTG;
			break;

		case USB_DR_MODE_UNKNOWN:
		default:
			dev_err(&pdev->dev, "No 'dr_mode' property found\n");
			return -EINVAL;
	}
	pdata.platform_ops	= &sunxi_ops;
	pdata.config 		= &sunxi_musb_hdrc_config;

	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		return -ENOMEM;

	glue->dev = &pdev->dev;
	INIT_DELAYED_WORK(&glue->detect, id_vbus_det_scan);

	glue->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(glue->clk)) {
		dev_err(&pdev->dev, "Error getting clock: %ld\n",
			PTR_ERR(glue->clk));
		return PTR_ERR(glue->clk);
	}

	glue->sc = syscon_regmap_lookup_by_phandle(np, "syscons");
	if (IS_ERR(glue->sc)) {
		dev_err(&pdev->dev, "Error getting syscon %ld\n",
			PTR_ERR(glue->sc));
		return PTR_ERR(glue->sc);
	}

	glue->phy = devm_phy_get(&pdev->dev, "usb");
	if (IS_ERR(glue->phy)) {
		if (PTR_ERR(glue->phy) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_err(&pdev->dev, "Error getting phy %ld\n",
			PTR_ERR(glue->phy));
		return PTR_ERR(glue->phy);
	}

	/* FIXME */
	usb_phy_generic_register();
	glue->usb_phy = usb_get_phy(USB_PHY_TYPE_USB2);
	if (IS_ERR(glue->usb_phy)) {
		if (PTR_ERR(glue->usb_phy) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_err(&pdev->dev, "Error getting usb-phy %ld\n",
			PTR_ERR(glue->usb_phy));
		return PTR_ERR(glue->usb_phy);
	}

	glue->id_det_gpio =
		devm_gpiod_get(&pdev->dev, "id_det", GPIOD_IN);
	if (IS_ERR(glue->id_det_gpio)) {
		dev_err(&pdev->dev, "Error getting id_det gpio\n");
		return PTR_ERR(glue->id_det_gpio);
	}

	glue->vbus_det_gpio =
		devm_gpiod_get(&pdev->dev, "vbus_det", GPIOD_IN);
	if (IS_ERR(glue->vbus_det_gpio)) {
		dev_err(&pdev->dev, "Error getting vbus_det gpio\n");
		return PTR_ERR(glue->vbus_det_gpio);
	}

	glue->id_det_irq = gpiod_to_irq(glue->id_det_gpio);
	if (glue->id_det_irq < 0) {
		dev_err(&pdev->dev, "Error getting id_det irq\n");
		return -EINVAL;
	}

	glue->vbus_det_irq = gpiod_to_irq(glue->vbus_det_gpio);
	if (glue->vbus_det_irq < 0) {
		dev_err(&pdev->dev, "Error getting vbus_det irq\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, glue);

	memset(&pinfo, 0, sizeof(pinfo));
	pinfo.name	 = "musb-hdrc";
	pinfo.id	= PLATFORM_DEVID_AUTO;
	pinfo.parent	= &pdev->dev;
	pinfo.res	= pdev->resource;
	pinfo.num_res	= pdev->num_resources;
	pinfo.data	= &pdata;
	pinfo.size_data = sizeof(pdata);

	glue->musb = platform_device_register_full(&pinfo);
	if (IS_ERR(glue->musb)) {
		dev_err(&pdev->dev, "Error registering musb device: %ld\n",
			PTR_ERR(glue->musb));
		return PTR_ERR(glue->musb);
	}

	return 0;
}

static int sunxi_remove(struct platform_device *pdev)
{
	struct sunxi_glue *glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->musb);

	return 0;
}

static const struct of_device_id sunxi_match[] = {
	{ .compatible = "allwinner,sun4i-a10-musb", },
	{}
};

static struct platform_driver sunxi_driver = {
	.probe = sunxi_probe,
	.remove = sunxi_remove,
	.driver = {
		.name= "musb-sunxi",
		.of_match_table = sunxi_match,
	},
};

MODULE_DESCRIPTION("Allwinner sunxi MUSB Glue Layer");
MODULE_AUTHOR("Roman Byshko <rbyshko@gmail.com>, Chen-Yu Tsai <wens@csie.org>");
MODULE_LICENSE("GPL v2");
module_platform_driver(sunxi_driver);
