#include "linux/init.h"
#include "linux/io.h"
#include "linux/ioport.h"
#include "linux/printk.h"
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqhandler.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/uio.h>
#include <linux/uio_driver.h>
#include <linux/version.h>

#define ENCODER_INIT 0

static int encoder_driver_probe(struct platform_device *pdev) {
  int retval;
  struct uio_info *info;
  struct resource *resource;
  retval = 0;
  info = devm_kzalloc(&pdev->dev, sizeof(struct uio_info), GFP_KERNEL);
  if (!info) {
    retval = -ENOMEM;
    goto end;
  }
  info->name = "encoder";
  resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if(!resource){
    retval = -ENODEV;
    goto end;
  }
  info->mem[0].addr = resource->start;
  info->mem[0].size = resource_size(resource);
  info->mem[0].memtype = UIO_MEM_PHYS;
  info->mem[0].internal_addr = devm_ioremap_resource(&pdev->dev, resource);
  iowrite32(ENCODER_INIT, info->mem[0].internal_addr + sizeof(u32));
  mb();
  dev_set_drvdata(&pdev->dev, info);
  retval = uio_register_device(&pdev->dev, info);
  if(retval){
      devm_iounmap(&pdev->dev, info->mem[0].internal_addr);
  }
  goto end;
end:
  return retval;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
static void encoder_driver_remove(struct platform_device* dev)
{
    struct uio_info* info = dev_get_drvdata(&dev->dev);
    if(!info){
        printk("debounce_driver: uio_info is null\n");
        return;
    }
    uio_unregister_device(info);
    devm_iounmap(&dev->dev, info->mem[0].internal_addr);
}
#else
static int encoder_driver_remove(struct platform_device* dev)
{
    struct uio_info* info = dev_get_drvdata(&dev->dev);
    if(!info){
        printk("debounce_driver: uio_info is null\n");
        return -ENODEV;
    }
    uio_unregister_device(info);
    devm_iounmap(info->mem[0].internal_addr);
    return 0;
}
#endif

static const struct of_device_id devs[] = {{.compatible = "encoder"}, {}};
MODULE_DEVICE_TABLE(of, devs);

static struct platform_driver encoder_driver = 
{
    .probe = &encoder_driver_probe,
    .remove = &encoder_driver_remove, 
    .driver =
    {
        .name = "encoder",
        .owner = THIS_MODULE,
        .of_match_table = devs,
    }

};

module_platform_driver(encoder_driver);
MODULE_LICENSE("GPL");

