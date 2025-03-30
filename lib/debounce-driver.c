#include <linux/uio.h>
#include "linux/init.h"
#include "linux/printk.h"
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

#define NAME "uio_pdrv_genirq"

static const struct of_device_id devs[] = {{.compatible = "debouncer"}, {}};
MODULE_DEVICE_TABLE(of, devs);


static int debounce_driver_probe(struct platform_device *dev) 
{
  struct uio_info* info;
  int irqno, retval;
  info = devm_kzalloc(&dev->dev, sizeof(struct uio_info), GFP_KERNEL);
  if(!info)
      return -ENOMEM;
  retval = 0;
  irqno = platform_get_irq(dev, 0);
  printk("debounce_driver: irqno = %d\n", irqno);
  if(irqno < 0){
      retval = irqno;
      goto end;
  }
  info->name = "debouncer";
  info->version = "0.0.1";
  info->irq_flags = IRQF_TRIGGER_RISING;
  info->irq = irqno;

  dev->name = NAME;
  dev->resource = NULL;
  dev->num_resources = 0;
  dev->dev.platform_data = info;
end:
  return retval;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
static void debounce_driver_remove(struct platform_device* dev)
{
    printk("debounce_driver: removing\n");
}
#else
static int debounce_driver_remove(struct platform_device* dev)
{
    printk("debounce_driver: removing\n");
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
    },

};

module_platform_driver(debounce_driver);
MODULE_LICENSE("GPL");

