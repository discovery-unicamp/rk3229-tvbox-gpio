#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>      /* For platform devices */
#include <linux/of.h>                   /* For DT*/
#include <linux/printk.h>               /* For printk */
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>
#include <linux/serdev.h>
#include <linux/jiffies.h>

#define GPIO_NUM 18 // Number of GPIO pins

#define OPERATION_READ  0x00 // Bit 7 (0 = read, 1 = write)
#define OPERATION_WRITE 0x80

// ########################################################
// VARIABLES
// ########################################################

static DEFINE_MUTEX(gpio_lock);

static struct gpio_chip chip;
static struct platform_device *pdev;
static struct serdev_device *global_serdev;

struct serdev_data {
    struct serdev_device *serdev;
    int response[4];
    bool response_ready;
};

static const char *gpio_names[] = {
};

static bool virt_gpio_directions[GPIO_NUM]; // false for input, true for output


// ########################################################
// SERIAL FUNCTIONS
// ########################################################

static int serial_comm_write_data(struct serdev_device *serdev, const unsigned char *data, size_t len)
{
    int ret;
    unsigned long timeout;

    timeout = msecs_to_jiffies(1000); // 1000 ms timeout for the write operation

    /* Send data */
    ret = serdev_device_write(serdev, data, len, timeout);
    if (ret < 0) {
        dev_err(&serdev->dev, "serdev_device_write failed: %d\n", ret);
        return ret;
    }

    serdev_device_wait_until_sent(serdev, timeout);

    return 0; /* Success */
}

static int serial_comm_receive_buf(struct serdev_device *serdev, const unsigned char *buf, size_t count)
{
    printk(KERN_INFO "Received data: ");
    for (size_t i = 0; i < count; i++) {
        printk(KERN_CONT "%02x ", buf[i]);
    }
    printk(KERN_CONT "\n");

    struct serdev_data *serdev_data = serdev_device_get_drvdata(serdev);

    if (count == 4) {
        serdev_data->response[0] = buf[0];
        serdev_data->response[1] = buf[1];
        serdev_data->response[2] = buf[2];
        serdev_data->response[3] = buf[3];
        serdev_data->response_ready = true;
    }

    return count;
}

static void serial_comm_write_wakeup(struct serdev_device *serdev)
{
    return;
}

static const struct serdev_device_ops serial_comm_ops = {
    .receive_buf = serial_comm_receive_buf,
    .write_wakeup = serial_comm_write_wakeup,
};


// ########################################################
// GPIO FUNCTIONS
// ########################################################

static int virt_gpio_direction_output(struct gpio_chip *gc, unsigned offset, int val)
{
    if (offset >= GPIO_NUM)
        return -EINVAL; // Invalid pin number

    virt_gpio_directions[offset] = true;

    printk(KERN_INFO "virt_gpio_direction_output: offset=%u set value=%d\n", offset, val);
    return 0;
}

static int virt_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
    if (offset >= GPIO_NUM)
        return -EINVAL; // Invalid pin number

    virt_gpio_directions[offset] = false;
    printk(KERN_INFO "virt_gpio_direction_input: offset=%u\n", offset);
    return 0;
}

static int virt_gpio_get_value(struct gpio_chip *gc, unsigned offset)
{
    mutex_lock_interruptible(&gpio_lock);

    printk(KERN_INFO "virt_gpio_get_value: offset=%u\n", offset);

    unsigned long timeout;
    int value = 1;
    char cmd[4] = {offset | OPERATION_READ, 0, 0x00, 0x00};

    struct serdev_data *serdev_data = serdev_device_get_drvdata(global_serdev);
    serdev_data->response_ready = false;

    serial_comm_write_data(global_serdev, cmd, 4);

    timeout = jiffies + msecs_to_jiffies(1000);

    // Busy-wait loop with timeout
    while (time_before(jiffies, timeout)) {
        if (serdev_data->response_ready) {
            value = serdev_data->response[1];
            printk(KERN_INFO "virt_gpio_get_value: offset=%u value=%d\n", offset, value);
            mutex_unlock(&gpio_lock);
            return value;
        }
        // Yield the processor (to prevent hogging the CPU)
        cpu_relax();
    }

    mutex_unlock(&gpio_lock);

    // if we get here, the timeout has expired, so return an error
    pr_err("virt_gpio_get_value: timed out waiting for value\n");
    return -ETIMEDOUT;
}

static void virt_gpio_set_value(struct gpio_chip *gc, unsigned offset, int val)
{
    mutex_lock_interruptible(&gpio_lock);

    printk(KERN_INFO "virt_gpio_set_value: offset=%u set value=%d\n", offset, val);

    unsigned long timeout;

    char cmd[4] = {offset | OPERATION_WRITE, val, 0x00, 0x00};

    struct serdev_data *serdev_data = serdev_device_get_drvdata(global_serdev);

    serdev_data->response_ready = false;

    serial_comm_write_data(global_serdev, cmd, 4);

    timeout = jiffies + msecs_to_jiffies(1000);

    // Busy-wait loop with timeout
    while (time_before(jiffies, timeout)) {
        if (serdev_data->response_ready) {
            printk(KERN_INFO "virt_gpio_set_value: offset=%u set value=%d confirmed\n", offset, val);
            mutex_unlock(&gpio_lock);
            return;
        }
        // Yield the processor (to prevent hogging the CPU)
        cpu_relax();
    }

    // if we get here, the timeout has expired, so return an error
    mutex_unlock(&gpio_lock);
    pr_err("virt_gpio_set_value: timed out waiting for confirmation\n");
}


// ########################################################
// MATCHING STRUCTURES
// ########################################################

static const struct of_device_id virt_gpio_gpiochip_ids[] = {
    { .compatible = "tvbox,tvbox-gpio-chip", },
    { /* sentinel */ }
};

static const struct of_device_id serial_comm_of_match[] = {
        { .compatible = "tvbox,tvbox-gpio-serdev", },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, serial_comm_of_match);


// ########################################################
// PROBE AND REMOVE FUNCTIONS
// ########################################################

static int virt_gpio_probe (struct platform_device *pdev)
{
    chip.label = pdev->name;
    chip.base = -1;
    chip.parent = &pdev->dev;
    chip.owner = THIS_MODULE;
    chip.names = gpio_names;
    chip.ngpio = GPIO_NUM;
    chip.can_sleep = 0;
    chip.get = virt_gpio_get_value;
    chip.set = virt_gpio_set_value;
    chip.direction_output = virt_gpio_direction_output;
    chip.direction_input = virt_gpio_direction_input;

    int ret = gpiochip_add(&chip);
    if (ret < 0) {
        pr_err("Failed to add GPIO chip: %d\n", ret);
    }
    return ret;
}

static int serial_comm_probe(struct serdev_device *serdev)
{
    printk(KERN_INFO "serial_comm_probe\n");

    struct serdev_data *serdev_data = devm_kzalloc(&serdev->dev, sizeof(*serdev_data), GFP_KERNEL);
    if (!serdev_data)
        return -ENOMEM;

    serdev_data->serdev = serdev;

    serdev_device_set_drvdata(serdev, serdev_data);

    int ret;

    global_serdev = serdev;

    dev_info(&serdev->dev, "Probing serial device\n");

    /* Set up the callback functions for receiving data and waking up writes */
    serdev_device_set_client_ops(serdev, &serial_comm_ops);

    /* Open the serial device after configuration */
    ret = serdev_device_open(serdev);

    if (ret) {
        dev_err(&serdev->dev, "Failed to open serial device: %d\n", ret);
        return ret;
    }

    /* Configure the serial device parameters */
    serdev_device_set_baudrate(serdev, 115200); // Set the baudrate
    serdev_device_set_flow_control(serdev, false); // Set the flow control

    dev_info(&serdev->dev, "Serial device setup complete\n");
    return 0;

}

static int virt_gpio_remove(struct platform_device *pdev)
{
    gpiochip_remove(&chip);
    return 0;
}


// ########################################################
// DRIVER STRUCTURES
// ########################################################

static struct platform_driver virt_gpio_driver = {
    .probe      = virt_gpio_probe,
    .remove     = virt_gpio_remove,
    .driver     = {
        .name     = "tvbox-gpio-chip",
        .of_match_table = of_match_ptr(virt_gpio_gpiochip_ids),
        .owner    = THIS_MODULE,
    },
};

static struct serdev_device_driver serial_comm_driver = {
        .driver = {
                .name = "tvbox-gpio-serdev",
                .of_match_table = serial_comm_of_match,
        },
        .probe = serial_comm_probe,
};


// ########################################################
// MODULE INIT AND EXIT
// ########################################################

static int __init tvbox_gpio_init(void) {
    int inst_id = 0;
    int ret;

    // Register the gpio driver
    ret = platform_driver_register(&virt_gpio_driver);
    if (ret) {
        pr_err("Failed to register tvbox-gpio-chip driver\n");
        return ret;
    }

    // Register the serial driver
    ret = serdev_device_driver_register(&serial_comm_driver);
    if (ret) {
        pr_err("Failed to register serdev driver\n");
        platform_driver_unregister(&virt_gpio_driver);
        return ret;
    }

    // Create and add the gpio device
    pdev = platform_device_alloc("tvbox-gpio-chip", inst_id);
    if (!pdev) {
        pr_err("Failed to allocate tvbox-gpio-chip device\n");
        platform_driver_unregister(&virt_gpio_driver);
        return -ENOMEM;
    }

    ret = platform_device_add(pdev);
    if (ret) {
        pr_err("Failed to add tvbox-gpio-chip device\n");
        platform_device_put(pdev);
        platform_driver_unregister(&virt_gpio_driver);
        return ret;
    }

    pr_info("tvbox-gpio-chip added\n");

    /* // Create and add the serial device */
    /* serdev = serdev_device_alloc(); */
    /* if (!serdev) { */
    /*     pr_err("Failed to allocate tvbox-gpio-serdev device\n"); */
    /*     platform_device_unregister(pdev); */
    /*     platform_driver_unregister(&virt_gpio_driver); */
    /*     serdev_device_driver_unregister(&serial_comm_driver); */
    /*     return -ENOMEM; */
    /* } */

    /* ret = serdev_device_add(serdev); */
    /* if (ret) { */
    /*     pr_err("Failed to add tvbox-gpio-serdev device\n"); */
    /*     serdev_device_put(serdev); */
    /*     platform_device_unregister(pdev); */
    /*     platform_driver_unregister(&virt_gpio_driver); */
    /*     serdev_device_driver_unregister(&serial_comm_driver); */
    /*     return ret; */
    /* } */

    pr_info("tvbox-gpio-serdev added\n");

    pr_info("tvbox-gpio module loaded\n");
    return 0;
}

static void __exit tvbox_gpio_exit(void) {
    pr_info("removing tvbox-gpio module\n");
    platform_device_unregister(pdev);
    platform_driver_unregister(&virt_gpio_driver);
    serdev_device_close(global_serdev);
    serdev_device_driver_unregister(&serial_comm_driver);
    pr_info("tvbox-gpio-chip removed\n");
}

module_init(tvbox_gpio_init);
module_exit(tvbox_gpio_exit);

MODULE_LICENSE("MIT");
MODULE_AUTHOR("Igor Borges <igor.gcc.borges@gmail.com>");
