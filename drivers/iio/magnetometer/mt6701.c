// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the Asahi Kasei EMD Corporation MT6701
 * and Aichi Steel AMI305 magnetometer chips.
 * Based on a patch from Samu Onkalo and the AK8975 IIO driver.
 *
 * Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
 * Copyright (c) 2010 NVIDIA Corporation.
 * Copyright (C) 2016 Linaro Ltd.
 *
 * Author: Samu Onkalo <samu.p.onkalo@nokia.com>
 * Author: Linus Walleij <linus.walleij@linaro.org>
 */
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

struct mt6701_data {
	struct spi_device *spi;
	const char *name;
};

static uint8_t tableCRC6[64] = {
 0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
 0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
 0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
 0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
 0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
 0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
 0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02
};


/*32-bit input data, right alignment, Calculation over 18 bits (mult. of 6) */
static u8 CRC6_43_18bit (u32 w_InputData)
{
 u8 b_Index = 0;
 u8 b_CRC = 0;

 b_Index = (u8 )(((u32)w_InputData >> 12u) & 0x0000003Fu);

 b_CRC = (u8 )(((u32)w_InputData >> 6u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = (u8 )((u32)w_InputData & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = tableCRC6[b_Index];

 return b_CRC;
} 


static int mt6701_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2,
			   long mask)
{
	struct mt6701_data *st = iio_priv(indio_dev);
	int ret; 
	u8 buf[3] = {0,0,0};
	u32 spi_32 = 0;
	u32 angle_spi = 0;
	u8 field_status = 0;
	u8 push_status = 0;
	u8 loss_status = 0;
	u8 received_crc = 0;
	u8 calculated_crc = 0;


	printk("kisonhe::mt6701_read_raw running!");


	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
	case IIO_CHAN_INFO_RAW:
		*val2=0;
		ret = spi_read(st->spi,buf,3);
		if (ret)
			return ret;

		spi_32 = (buf[0] << 16) | (buf[1] << 8) | buf[2];
		angle_spi = spi_32 >> 10;
		field_status = (spi_32 >> 6) & 0x3;
		push_status = (spi_32 >> 8) & 0x1;
		loss_status = (spi_32 >> 9) & 0x1;
		received_crc = spi_32 & 0x3F;
		calculated_crc = CRC6_43_18bit(spi_32 >> 6);
		
		if (received_crc == calculated_crc) {
			// new_angle = (float)angle_spi * 2 * PI / 16384;
		} else {
			printk("Bad CRC. expected %d, actual %d\nbuf 0-2: %x %x %x\nspi_32:%x", calculated_crc, received_crc,buf[0], buf[1], buf[2],spi_32);
			return -EINVAL; // TODO return what
		}
		*val = angle_spi;

		return IIO_VAL_INT;
		

		break;
	default:
		return -EINVAL;
	}

	// return IIO_VAL_INT;
	return -EINVAL;
}


/*
 * We have no datasheet for the MT6701 but we guess that its
 * ADC is 12 bits. The AMI305 and AMI306 certainly has 12bit
 * ADC.
 */
static const struct iio_chan_spec mt6701_channels[] = {
	{
		.type = IIO_ANGL,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_PROCESSED) |
				// BIT(IIO_CHAN_INFO_INT_TIME) |
				BIT(IIO_CHAN_INFO_SCALE),
		// .event_spec = veml6030_event_spec,
		// .num_event_specs = ARRAY_SIZE(veml6030_event_spec),
	},
};

static struct attribute *mt6701_attributes[] = {
	// &iio_dev_attr_sensor_sensitivity.dev_attr.attr,
	// &iio_dev_attr_noise_level_tripped.dev_attr.attr,
	NULL,
};

static const struct attribute_group mt6701_attribute_group = {
	.attrs = mt6701_attributes,
};

static const struct iio_info mt6701_info = {
	.read_raw = &mt6701_read_raw,
};

static int mt6701_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct mt6701_data *data;
	int ret;

	printk("kisonhe::mt6701_probe running!");

	// /* Be sure lightning event interrupt is specified */
	// if (!spi->irq) {
	// 	dev_err(dev, "unable to get event interrupt\n");
	// 	return -EINVAL;
	// }
	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;
	if (!spi){
		printk("spi is nullptr!");
		return -EINVAL;
	}
	printk("spi is %x",spi);
	data = iio_priv(indio_dev);
	if (!data){
		printk("data is nullptr!");
		return -EINVAL;
	}
	printk("data is %x",data);
	data->spi = spi;

	spi_set_drvdata(spi, indio_dev);

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->channels = mt6701_channels;
	indio_dev->num_channels = ARRAY_SIZE(mt6701_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &mt6701_info;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret < 0) {
		dev_err(dev, "unable to register device\n");
		return ret;
	}
	return 0;
}

static const struct of_device_id mt6701_of_match[] = {
	{ .compatible = "MagnTek,mt6701", },
	{}
};
MODULE_DEVICE_TABLE(of, mt6701_of_match);

static const struct spi_device_id mt6701_id[] = {
	{"mt6701", 0},
	{},
};
MODULE_DEVICE_TABLE(spi, mt6701_id);

static struct spi_driver mt6701_driver = {
	.driver	 = {
		.name	= "mt6701",
		.pm = NULL,
		.of_match_table = mt6701_of_match,
	},
	.probe	  = mt6701_probe,
	.id_table = mt6701_id,
};

module_spi_driver(mt6701_driver);

MODULE_DESCRIPTION("MT6701 magnetometer driver");
MODULE_AUTHOR("Kison He");
MODULE_LICENSE("GPL v2");
