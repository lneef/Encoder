#include <linux/uio.h>
#include "asm/io.h"
#include "linux/bitops.h"
#include "linux/init.h"
#include "linux/irqreturn.h"
#include "linux/printk.h"
#include "linux/types.h"
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/uio_driver.h>

#define NAME "debouncer"
#define ARRAY_ELEMS 4

struct debounce_driver_info{
    unsigned long irq_status;
};

enum IRQ_STATUS{
    IRQ_DISABLED = 0
};

static irqreturn_t debounce_irq_handler(int irqno, struct uio_info* info) {
  return IRQ_HANDLED;
}

static const struct of_device_id devs[] = {{.compatible = "debouncer"}, {}};
MODULE_DEVICE_TABLE(of, devs);


// adapted from https://github.com/torvalds/linux/blob/master/drivers/uio/uio_pdrv_genirq.c
// not thread safe
static int debounce_driver_irqcontrol(struct uio_info* info, s32 irq_on){
    struct debounce_driver_info* dd_info = info->priv;
    if(irq_on){
        if(__test_and_clear_bit(IRQ_DISABLED, &dd_info->irq_status))
            enable_irq(info->irq);
    }else{
        if(!__test_and_set_bit(IRQ_DISABLED, &dd_info->irq_status))
            disable_irq_nosync(info->irq);
    }
    return 0;
}

static int debounce_driver_probe(struct platform_device *dev) 
{
  struct uio_info* info;
  int irqno, retval;
  info = devm_kzalloc(&dev->dev, sizeof(struct uio_info), GFP_KERNEL);
  if(!info)
      return -ENOMEM;
  info->version = "0.0.1";
  info->name = "debouncer";
  irqno = platform_get_irq(dev, 0);
  if (irqno < 0){
      retval = irqno;
      goto end;
  }
  info->irq = irqno;
  info->irq_flags = IRQF_TRIGGER_RISING;
  info->handler = debounce_irq_handler;
  dev_set_drvdata(&dev->dev, info);
  retval = uio_register_device(&dev->dev, info);
  if(retval){
      goto end;
  }
  retval = 0;
end:
  return retval;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
static void debounce_driver_remove(struct platform_device* dev)
{
    struct uio_info* info = dev_get_drvdata(&dev->dev);
    if(!info){
        printk("debounce_driver: uio_info is null\n");
        return;
    }
    uio_unregister_device(info);
}
#else
static int debounce_driver_remove(struct platform_device* dev)
{
    struct uio_info* info = dev_get_drvdata(&dev->dev);
    if(!info){
        printk("debounce_driver: uio_info is null\n");
        return -ENODEV;
    }
    uio_unregister_device(info);
    return 0;
}
#endif
static struct platform_driver debounce_driver = 
{
    .probe = &debounce_driver_probe,
    .remove = &debounce_driver_remove, 
    .driver =
    {
        .name = "debouncer",
        .owner = THIS_MODULE,
        .of_match_table = devs,
    }

};

module_platform_driver(debounce_driver);
MODULE_LICENSE("GPL");


