// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for GC9A01 panels
 *
 * Copyright 2022 KisonHe <kisonhepenpal@gmail.com>
 *
 * Based on ili9341.c:
 * Copyright 2018 David Lechner
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modeset_helper.h>
#include <video/mipi_display.h>

// #define GC9A01_FRMCTR1		0xb1
// #define GC9A01_DISCTRL		0xb6
// #define GC9A01_ETMOD		0xb7

// #define GC9A01_PWCTRL1		0xc0
// #define GC9A01_PWCTRL2		0xc1
// #define GC9A01_VMCTRL1		0xc5
// #define GC9A01_VMCTRL2		0xc7
// #define GC9A01_PWCTRLA		0xcb
// #define GC9A01_PWCTRLB		0xcf

// #define GC9A01_PGAMCTRL		0xe0
// #define GC9A01_NGAMCTRL		0xe1
// #define GC9A01_DTCTRLA		0xe8
// #define GC9A01_DTCTRLB		0xea
// #define GC9A01_PWRSEQ		0xed

// #define GC9A01_EN3GAM		0xf2
// #define GC9A01_PUMPCTRL		0xf7

// #define GC9A01_MADCTL_BGR	BIT(3)
// #define GC9A01_MADCTL_MV	BIT(5)
// #define GC9A01_MADCTL_MX	BIT(6)
// #define GC9A01_MADCTL_MY	BIT(7)

#define GC9A01_MADCTL_BGR	0x08
#define GC9A01_MADCTL_MV	0x20
#define GC9A01_MADCTL_MX	0x40
#define GC9A01_MADCTL_MY	0x80


static void gc9a01_module_enable(struct drm_simple_display_pipe *pipe,
			     struct drm_crtc_state *crtc_state,
			     struct drm_plane_state *plane_state)
{
	printk("kisonhe::Entering gc9a01_module_enable!!!");
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	u8 addr_mode;
	int ret, idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		goto out_exit;
	if (ret == 1)
		goto out_enable;


	// from .pio/libdeps/HoloCube/TFT_eSPI/TFT_Drivers/GC9A01_Init.h
	mipi_dbi_command(dbi,0xEF);
	mipi_dbi_command(dbi,0xEB,0x14); //what? didnt see on data sheet, different GC9A01 & GC9A01A?

	// Inter Register Enable
	mipi_dbi_command(dbi,0xFE);
	mipi_dbi_command(dbi,0xEF);

	mipi_dbi_command(dbi,0xEB,0x14); //WTF? didnt see on data sheet & why again?
	mipi_dbi_command(dbi,0x84,0x40);
	mipi_dbi_command(dbi,0x85,0xFF);
	mipi_dbi_command(dbi,0x86,0xFF);
	mipi_dbi_command(dbi,0x87,0xFF);
	mipi_dbi_command(dbi,0x88,0x0A);
	mipi_dbi_command(dbi,0x89,0x21);
	mipi_dbi_command(dbi,0x8A,0x00);
	mipi_dbi_command(dbi,0x8B,0x80);
	mipi_dbi_command(dbi,0x8C,0x01);
	mipi_dbi_command(dbi,0x8D,0x01);
	mipi_dbi_command(dbi,0x8E,0xFF);
	mipi_dbi_command(dbi,0x8F,0xFF);
	mipi_dbi_command(dbi,0xB6,0x00,0x20);
	mipi_dbi_command(dbi,0x3A,0x05);
	mipi_dbi_command(dbi,0x90,0x08,0x08,0x08,0x08);
	mipi_dbi_command(dbi,0xBD,0x06);
	mipi_dbi_command(dbi,0xBC,0x00);
	mipi_dbi_command(dbi,0xFF,0x60,0x01,0x04);
	mipi_dbi_command(dbi,0xC3,0x13,0xC4,0x13);
	mipi_dbi_command(dbi,0xC9,0x22);
	mipi_dbi_command(dbi,0xBE,0x11);
	mipi_dbi_command(dbi,0xE1,0x10,0x0E);
	mipi_dbi_command(dbi,0xDF,0x21,0x0C,0x02);
	mipi_dbi_command(dbi,0xF0,0x45,0x09,0x08,0x08,0x26,0x2A);
	mipi_dbi_command(dbi,0xF1,0x43,0x70,0x72,0x36,0x37,0x6F);
	mipi_dbi_command(dbi,0xF2,0x45,0x09,0x08,0x08,0x26,0x2A);
	mipi_dbi_command(dbi,0xF3,0x43,0x70,0x72,0x36,0x37,0x6F);
	mipi_dbi_command(dbi,0xED,0x1B,0x0B);
	mipi_dbi_command(dbi,0xAE,0x77);
	mipi_dbi_command(dbi,0xCD,0x63);
	mipi_dbi_command(dbi,0x70,0x07,0x07,0x04,0x0E,0x0F,0x09,0x07,0x08,0x03);

	mipi_dbi_command(dbi,0xE8,0x34);

	mipi_dbi_command(dbi,0xE8,0x34);
	mipi_dbi_command(dbi,0x62,0x18,0x0D,0x71,0xED,0x70,0x70,0x18,0x0F,0x71,0xEF,0x70,0x70);
	mipi_dbi_command(dbi,0x63,0x18,0x11,0x71,0xF1,0x70,0x70,0x18,0x13,0x71,0xF3,0x70,0x70);
	mipi_dbi_command(dbi,0x64,0x28,0x29,0xF1,0x01,0xF1,0x00,0x07);
	mipi_dbi_command(dbi,0x66,0x3C,0x00,0xCD,0x67,0x45,0x45,0x10,0x00,0x00,0x00);
	mipi_dbi_command(dbi,0x67,0x00,0x3C,0x00,0x00,0x00,0x01,0x54,0x10,0x32,0x98);
	mipi_dbi_command(dbi,0x74,0x10,0x85,0x80,0x00,0x00,0x4E,0x00);

	mipi_dbi_command(dbi,0x98,0x3e,0x07);
	mipi_dbi_command(dbi,0x35);
	mipi_dbi_command(dbi,0x21);

	mipi_dbi_command(dbi,0x11);
	msleep(120);
	mipi_dbi_command(dbi,0x29);
	msleep(20);

out_enable:
	switch (dbidev->rotation) {
	default:
		addr_mode = 0;
		break;
	case 90:
		addr_mode = GC9A01_MADCTL_MV | GC9A01_MADCTL_MX;
		break;
	case 180:
		addr_mode = GC9A01_MADCTL_MY | GC9A01_MADCTL_MX;
		break;
	case 270:
		addr_mode = GC9A01_MADCTL_MV | GC9A01_MADCTL_MY;
		break;
	}
	addr_mode |= GC9A01_MADCTL_BGR;
	mipi_dbi_command(dbi, 0x36, addr_mode);
	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs gc9a01_pipe_funcs = {
	.enable = gc9a01_module_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = mipi_dbi_pipe_update,
};

static const struct drm_display_mode gc9a01_module_mode = {
	DRM_SIMPLE_MODE(240, 240, 35, 35),
};

DEFINE_DRM_GEM_CMA_FOPS(gc9a01_fops);

static const struct drm_driver gc9a01_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &gc9a01_fops,
	DRM_GEM_CMA_DRIVER_OPS_VMAP,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "gc9a01",
	.desc			= "Galaxycore GC9A01",
	.date			= "20220629",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id gc9a01_of_match[] = {
	{ .compatible = "kisonhe,gc9a01_module" },
	{ }
};
MODULE_DEVICE_TABLE(of, gc9a01_of_match);

static const struct spi_device_id gc9a01_id[] = {
	{ "gc9a01_module", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, gc9a01_id);

static int gc9a01_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	int ret;

	dbidev = devm_drm_dev_alloc(dev, &gc9a01_driver,
				    struct mipi_dbi_dev, drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	dbi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset))
		return dev_err_probe(dev, PTR_ERR(dbi->reset), "Failed to get GPIO 'reset'\n");

	dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc))
		return dev_err_probe(dev, PTR_ERR(dc), "Failed to get GPIO 'dc'\n");

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	ret = mipi_dbi_dev_init(dbidev, &gc9a01_pipe_funcs, &gc9a01_module_mode, rotation);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_fbdev_generic_setup(drm, 0);

	return 0;
}

static int gc9a01_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}

static void gc9a01_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver gc9a01_spi_driver = {
	.driver = {
		.name = "gc9a01",
		.of_match_table = gc9a01_of_match,
	},
	.id_table = gc9a01_id,
	.probe = gc9a01_probe,
	.remove = gc9a01_remove,
	.shutdown = gc9a01_shutdown,
};
module_spi_driver(gc9a01_spi_driver);

MODULE_DESCRIPTION("Galaxycore GC9A01 DRM driver");
MODULE_AUTHOR("KisonHe <kisonhepenpal@gmail.com>");
MODULE_LICENSE("GPL");
