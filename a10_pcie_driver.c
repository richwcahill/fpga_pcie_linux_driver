/*  Super simple Altera PCI Express Driver 
    This driver supports:
      udev: hot insertion with dynmaic major/minor number calls
      mmap: user control mapping of PCIe bars into User Virtual memory
      ioctl: user control callback to access driver features
      dma: Host memory mapping for Requester memory mastering

Author: Rich Cahill
Email:  rcahill@altera.com 
*/

#include <linux/types.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/interrupt.h>

#include "a10_pcie_driver.h"

static unsigned int count = 1;

static struct pci_cdev_struct fpga_pcie_dev;   /* Global fpga/pci char device declaration */
static struct class *pcie_udev_class;         /* udev class declaration */

// PCIe Vendor & Device IDs are parameters passed to the module when it's loaded
static unsigned short ALTERA_VID = 0x1172;  //default to Altera's Vendor ID
static unsigned short ALTERA_DID = 0xE001;  // default to PCIe Reference Design Device ID 
//static unsigned short vid = 0x106B; // default to Apple's Vendor ID
//static unsigned short did = 0x1186; // default to PCIe Reference Design Device ID

/* Open and Release */
static int open_chrdevice(struct inode *inode, struct file *file) {
  static int counter = 0;
  dev_dbg(fpga_pcie_dev.device_driver, "Opening device: %s\n", DEV_NAME);
  dev_dbg(fpga_pcie_dev.device_driver, "Major = %d, Minor = %d\n", imajor(inode), iminor(inode));
  counter++;
  dev_info(fpga_pcie_dev.device_driver, "Device opened %d times\n", counter);
  return 0;
}

/* Basic Read and Write Character functions */
static ssize_t read_chrdevice(struct file *file, char __user *buf, size_t lbuf, loff_t *ppos) {    
  int nbytes, maxbytes, bytes_to_do;
  maxbytes = fpga_pcie_dev.pcie_control.bar_length[0] - *ppos;
  bytes_to_do = fpga_pcie_dev.pcie_control.bar_length[0] > lbuf ? lbuf : maxbytes; //verify user buffer size vs. bar size
  if (bytes_to_do == 0)
    dev_info(fpga_pcie_dev.device_driver,"Reached end of the device on a read");
  
  nbytes = bytes_to_do - copy_to_user(buf, fpga_pcie_dev.bar[0] + *ppos, bytes_to_do);
  *ppos += nbytes;
  dev_info(fpga_pcie_dev.device_driver,"Leaving the READ function, nbytes=%d, pos=%d\n", nbytes, (int)*ppos);
  return nbytes;
}

static ssize_t write_chrdevice(struct file *file, const char __user *buf, size_t lbuf, loff_t *ppos) {
  unsigned long bar_start = pci_resource_start(fpga_pcie_dev.pci_device, 0);
  unsigned long *bar_address = (long *)bar_start;

  int nbytes = lbuf-copy_from_user(bar_address + *ppos, buf, lbuf); // number of bytes copied
  *ppos += nbytes;

  dev_info(fpga_pcie_dev.device_driver,"Writing: nbytes=%d, pointer=%d\n", nbytes, (int)*ppos);
  return nbytes;    
}




static irqreturn_t isr_chrdevice(int irq, void *dev_id) { 
struct pci_cdev_struct *pci_cdev = (struct pci_cdev_struct *)dev_id;
  if (!pci_cdev)
    return IRQ_NONE;
  pci_cdev->irq_count++;
  dev_info(fpga_pcie_dev.device_driver, "Device opened %d times\n", pci_cdev->irq_count);
  return IRQ_HANDLED;
}




static int mmap_chrdevice(struct file *filp, struct vm_area_struct *vma) {
  unsigned long bar_start = pci_resource_start(fpga_pcie_dev.pci_device, fpga_pcie_dev.pcie_control.bar_select);
  unsigned long bar_end = pci_resource_end(fpga_pcie_dev.pci_device, fpga_pcie_dev.pcie_control.bar_select);
  unsigned long bar_len = pci_resource_len(fpga_pcie_dev.pci_device, fpga_pcie_dev.pcie_control.bar_select);
  unsigned long offset = (vma->vm_pgoff << PAGE_SHIFT) + bar_start;

  //dev_info(fpga_pcie_dev.device_driver, "MMAP called %s bar start=0x%08x end=0x%08x len=%d offset=0x%08x vma start=0x%08x end=0x%08x size=%d\n", DEV_NAME, bar_start, bar_end, bar_len, offset, vma->vm_start, vma->vm_end, vma->vm_end - vma->vm_start);

  if (bar_start != 0 && bar_len != 0) {

      if((offset + (vma->vm_end - vma->vm_start)) > bar_end)
        return -EINVAL;

      //offset += (unsigned long)ivshmem_dev.data_mmio_start;

      vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

      if(io_remap_pfn_range(vma, vma->vm_start,
                            offset >> PAGE_SHIFT,
                            vma->vm_end - vma->vm_start,
                            vma->vm_page_prot))
        return -EAGAIN;
  }

  return 0;
}

static long ioctl_chrdevice(struct file *f, unsigned int cmd, unsigned long arg) {
  //int bar_select = 0;
  struct pcie_ioctl ioctl_control;

  switch (cmd) {
    case IOCTL_BAR_SET:
      
      copy_from_user(&ioctl_control, (void __user *)arg, sizeof(ioctl_control));
      dev_info(fpga_pcie_dev.device_driver,"%s: IOCTL_BAR_SET choose BAR %d\n", DEV_NAME, ioctl_control.bar_select);
      
    break;

    case IOCTL_BAR_GET:
      ioctl_control.bar_select =0;
      dev_info(fpga_pcie_dev.device_driver,"%s: IOCTL_BAR_GET choose BAR %d\n", DEV_NAME, ioctl_control.bar_select);
      dev_info(fpga_pcie_dev.device_driver,"%s: IOCTL_BAR_GET BAR %d size = %lu\n", DEV_NAME, ioctl_control.bar_select, fpga_pcie_dev.pcie_control.bar_length[ioctl_control.bar_select]);
      
    break;
  
    default:
    return -EINVAL;
    // Copy our pci_dev.pci_device from local to the global
    fpga_pcie_dev.pcie_control.bar_select = ioctl_control.bar_select;
  }
  return 0;
}

static int release_chrdevice(struct inode *inode, struct file *file)  {
	dev_info(fpga_pcie_dev.device_driver,"%s: Closing device\n", DEV_NAME);
	return 0;
}

static struct file_operations pci_fops = {
  .owner            = THIS_MODULE,
  .open             = open_chrdevice,
  .read             = read_chrdevice,
  .write            = write_chrdevice,
  .mmap             = mmap_chrdevice,
  .unlocked_ioctl   = ioctl_chrdevice,
  .release          = release_chrdevice
};

/* PCIe BAR Helper functions */
static int scan_bars(struct pci_dev *dev) {
	unsigned long bar_start, bar_end, bar_flags, bar_length;
  int i;
  dev_info(&dev->dev, "Scanning: %s:\n", DEV_NAME);
  for (i = 0; i < ALTERA_DMA_BAR_NUM; i++) {
    bar_start = pci_resource_start(dev, i);
    bar_end = pci_resource_end(dev, i);
    bar_flags = pci_resource_flags(dev, i);
    bar_length = pci_resource_len(dev, i);
    dev_info(&dev->dev, "BAR[%d] 0x%lx-0x%lx flags 0x%lx, length 0x%lx", i, bar_start, bar_end, bar_flags, bar_length);
  }
  return 0; 
}

static void unmap_bars(struct pci_cdev_struct *pci_cdev, struct pci_dev *dev) {
  int i;
  for (i = 0; i < ALTERA_DMA_BAR_NUM; i++) {
    if (pci_cdev->bar[i]) {
      pci_iounmap(dev, pci_cdev->bar[i]);
      pci_cdev->bar[i] = NULL;
      dev_info(&dev->dev, "BAR %d Unmapped\n",i);
    } 
    else 
      dev_info(&dev->dev, "BAR %d not mapped\n",i);
  }
}

/* PCI functions */
static int pcie_probe(struct pci_dev *dev, const struct pci_device_id *id)  {
  int i;
  /* create a pointer, same as pci struct for probe set up */
  struct pci_cdev_struct *pci_cdev;

  /* allocate memory for per-board book keeping */
  pci_cdev = kzalloc(sizeof(struct pci_cdev_struct), GFP_KERNEL);
  if(!pci_cdev) {
    dev_err(&dev->dev,"Could not kzalloc()ate memory.\n");
    return (-ENOMEM);
  }

  // Disable ISR for now, may need in future
  atomic_set(&pci_cdev->intr_disabled, 1);

  /* Enable PCI Device */
  /* Check vendor ID */
  if((dev->vendor == ALTERA_VID) && (dev->device == ALTERA_DID)) {
    pci_cdev->vendor_id = dev->vendor;
    pci_cdev->device_id = dev->device;
    snprintf(pci_cdev->name, sizeof(pci_cdev->name), "%s%d", pci_name(dev), 0);
    dev_info(&dev->dev, "Found FPGA at: %s\n", pci_cdev->name);
    dev_info(&dev->dev, "Vendor ID: 0x%04X\n", dev->vendor);
    dev_info(&dev->dev, "Device ID: 0x%04X\n", dev->device);
    dev_info(&dev->dev, "Subsystem Vendor ID: 0x%04X\n", dev->subsystem_device);
    if (pci_enable_device(dev)) {
      dev_err(&dev->dev, "pci_enable_device() failed\n");
      kfree(pci_cdev);
      return (-ENODEV);
    }

    /* Enable Bus Mastering */
    pci_set_master(dev);

    // Query Endpoint to verify 64Bit address supported
    if (!pci_set_dma_mask(dev, DMA_BIT_MASK(64))) {
      // If DMA can directly access "consistent memory" in System RAM above 4G physical address, register this
      pci_set_consistent_dma_mask(dev, DMA_BIT_MASK(64));
      dev_info(&dev->dev, "Using a 64-bit IRQ mask\n");
    } else {
      dev_info(&dev->dev, "Unable to use 64-bit IRQ mask\n");
      pci_disable_device(dev);
      kfree(pci_cdev);
      return -1;
    }

    /* Reserve memory regions for this device */
    if (pci_request_regions(dev, DEV_NAME)) {
      dev_err(&dev->dev, "pci_request_regions() failed\n");
      pci_disable_device(dev);
      kfree(pci_cdev);
      return (-ENODEV);
    }

    // Simple function to scan BARs
    scan_bars(dev);

    // Map Bar functions
    for (i = 0; i < ALTERA_DMA_BAR_NUM; i++) {
      pci_cdev->pcie_control.bar_start[i] = pci_resource_start(dev, i);
      pci_cdev->pcie_control.bar_end[i] = pci_resource_end(dev, i);
      pci_cdev->pcie_control.bar_flags[i] = pci_resource_flags(dev, i);
      pci_cdev->pcie_control.bar_length[i] = pci_resource_len(dev, i);

      if (!pci_cdev->pcie_control.bar_length[i]) {
          pci_cdev->bar[i] = NULL; 
          continue; //Pop out to the top of the for loop
      }

      // Map BARs into Kernel memory
      pci_cdev->bar[i] = pci_iomap(dev, i, 0);
      dev_info(&dev->dev, "BAR %d Address: %lx\n", i, pci_cdev->pcie_control.bar_start[i]);
      dev_info(&dev->dev, "BAR %d Length: %ld bytes\n", i, pci_cdev->pcie_control.bar_length[i]);
      if (!pci_cdev->bar[i]) {
        dev_err(&dev->dev, "Could not ioremap BAR %d\n", i);
        pci_release_regions(dev);
        pci_disable_device(dev);
        kfree(pci_cdev);
      return (-ENODEV);
      }
    }

    // initialize our module's wait-queue 
    init_waitqueue_head( &fpga_pcie_dev.my_wq );
    
    /* Enable Messaging Interrupts */
    if (pci_enable_msi(dev)) {
      dev_err(&dev->dev, "pci_enable_msi() failed\n");
      pci_cdev->msi_enabled = 0;
    } else {
      dev_info(&dev->dev, "pci_enable_msi() successful\n");
      pci_cdev->msi_enabled = 1;
    }

    // Read Configuration Space
    pci_read_config_byte(dev, PCI_REVISION_ID, &fpga_pcie_dev.revision);
    pci_read_config_byte(dev, PCI_INTERRUPT_PIN, &fpga_pcie_dev.irq_pin);
    pci_read_config_byte(dev, PCI_INTERRUPT_LINE, (u8 *)&fpga_pcie_dev.irq_line);

    // Read the FPGA MSI Structure
    pci_read_config_dword( dev, 0x50, &fpga_pcie_dev.msi_cap[ 0 ] ); 
    pci_read_config_dword( dev, 0x54, &fpga_pcie_dev.msi_cap[ 1 ] ); 
    pci_read_config_dword( dev, 0x54, &fpga_pcie_dev.msi_cap[ 2 ] ); 
    pci_read_config_dword( dev, 0x5c, &fpga_pcie_dev.msi_cap[ 3 ] );
    pci_read_config_word(dev, 0x52, &fpga_pcie_dev.msi_control_message);

    dev_info(&dev->dev,"0x50: %x\n", fpga_pcie_dev.msi_cap[ 0 ] );
    dev_info(&dev->dev,"0x54: %x\n", fpga_pcie_dev.msi_cap[ 1 ] );
    dev_info(&dev->dev,"0x58: %x\n", fpga_pcie_dev.msi_cap[ 2 ] );
    dev_info(&dev->dev,"0x5c: %x\n", fpga_pcie_dev.msi_cap[ 3 ] );
    dev_info(&dev->dev,"MSI CTL Reg: 0x%02X\n", fpga_pcie_dev.msi_control_message );

    // configure the Pro1000 to generate Message Signaled Interrupts
    //msi_data = 0x4100 | intID;
    //msi_addr = 0xFEE0100C;
    //msi_ctrl = msi_cap[0] | 0x00010000;
    //pci_write_config_dword( dev, 0xDC, msi_data );
    //pci_write_config_dword( dev, 0xD4, msi_addr );
    //pci_write_config_dword( dev, 0xD0, msi_ctrl );

    if (request_irq(fpga_pcie_dev.irq_line, isr_chrdevice, IRQF_SHARED, DEV_NAME, &fpga_pcie_dev)) {
      dev_info(&dev->dev,"Failed to reserve IRQ Handler for IRQ: %d\n", fpga_pcie_dev.irq_line);
      return -1;
    };
    dev_info(&dev->dev,"IRQ Pin: %u\n", fpga_pcie_dev.irq_pin);
    dev_info(&dev->dev,"IRQ Line: %u\n", fpga_pcie_dev.irq_line);
    dev_info(&dev->dev,"IRQ: %u\n", dev->irq);
    dev_info(&dev->dev,"Successfully loading ISR handler\n");

    // copy local pci_cdev to pci struct
    pci_set_drvdata(dev, pci_cdev);
    pci_cdev->pci_device = dev;
    // Copy our pci_dev.pci_device from local to the global
    fpga_pcie_dev.pci_device = pci_cdev->pci_device;
    fpga_pcie_dev.pcie_control = pci_cdev->pcie_control;

    dev_info(&dev->dev, "Enabled PCI device with VID 0x%04X, DID 0x%04X\n", dev->vendor, dev->device);

    return 0;
    } else {
      dev_dbg(&dev->dev,"This PCI device does not match VID 0x%04X, DID 0x%04X\n", ALTERA_VID, ALTERA_DID);
    return -ENODEV;
  }
}

static void pcie_remove(struct pci_dev *dev) {
  struct pci_cdev_struct *pci_cdev = pci_get_drvdata(dev);

  dev_info(&dev->dev, "Removing driver(dev = 0x%p) where device = 0x%p\n", dev, pci_cdev);
  dev_info(&dev->dev, "Vendor ID: 0x%04X\n", pci_cdev->vendor_id);
  dev_info(&dev->dev, "Device ID: 0x%04X\n", pci_cdev->device_id);
 
  // Release the interrupt handler
  synchronize_irq(fpga_pcie_dev.irq_line);
  dev_info(&dev->dev, "Freeing IRQ: %d for device 0x%p\n", fpga_pcie_dev.irq_line, dev);
  free_irq(fpga_pcie_dev.irq_line, &fpga_pcie_dev);

  //remove the Interrupt
	if (pci_cdev->msi_enabled )  {
    pci_disable_msi(dev);
    fpga_pcie_dev.msi_enabled = 0;
    dev_info(&dev->dev, "MSI Disabled\n");
  }

  unmap_bars (pci_cdev, dev);

  pci_disable_device(dev);
  pci_release_regions(dev);
  dev_info(&dev->dev, "PCIe resources released\n");
}

/* Altera PCIe Declaration  */
static struct pci_device_id pci_ids[] = {
	{ PCI_DEVICE(PCI_ANY_ID, PCI_ANY_ID), },
  { PCI_DEVICE(0x1172, 0xE001), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, pci_ids);

/* PCI Data stucture */
static struct pci_driver pcie_driver_ops = {
	.name = DEV_NAME,
	.id_table = pci_ids,
	.probe = pcie_probe,
	.remove = pcie_remove,
};

/* Initialize and Exit Functions */
static int __init init_driver(void) { 
	/* allocate major/minor numbers */
	if (alloc_chrdev_region(&fpga_pcie_dev.first, 0, count, DEV_NAME) < 0) {
		dev_err(fpga_pcie_dev.device_driver,"Failed to allocate Character Device Region\n\n");
		return -1;
	}
	
	/* register cdev structure */
	if (!(fpga_pcie_dev.char_device = cdev_alloc())) {
		dev_err(fpga_pcie_dev.device_driver, "cdev_alloc() failed to allocate structure\n\n");
		unregister_chrdev_region(fpga_pcie_dev.first, count);
		return -1;
	}
	
	/* initialize/assign driver to fops */
	cdev_init(fpga_pcie_dev.char_device, &pci_fops);
	
	/* go live with driver */
	if (cdev_add(fpga_pcie_dev.char_device, fpga_pcie_dev.first, count) < 0) {
		dev_err(fpga_pcie_dev.device_driver,"cdev_add() failed to bring altera_pcie_driver live\n\n");
		cdev_del(fpga_pcie_dev.char_device);
		unregister_chrdev_region(fpga_pcie_dev.first, count);
		return -1;
	}
	
  pcie_udev_class = class_create(THIS_MODULE, "pcie_class");
	fpga_pcie_dev.device_driver = device_create(pcie_udev_class, NULL, fpga_pcie_dev.first, NULL, "%s", DEV_NAME);
	
  /* register pci driver */
  if (pci_register_driver(&pcie_driver_ops)) {
    dev_err(fpga_pcie_dev.device_driver, "%s: PCIe Driver Registration failed\n",DEV_NAME);
    cdev_del(fpga_pcie_dev.char_device);
    unregister_chrdev_region(fpga_pcie_dev.first, count);
    return -1;
    }

  dev_info(fpga_pcie_dev.device_driver,"Registered Character Device in /dev : %s\n", DEV_NAME);
  dev_info(fpga_pcie_dev.device_driver,"Major number = %d, Minor number = %d\n\n", MAJOR(fpga_pcie_dev.first), MINOR(fpga_pcie_dev.first));
	return 0;
}

static void __exit exit_driver(void) {
	/* delete the structure, release major/minor, and free ram */
  device_destroy(pcie_udev_class, fpga_pcie_dev.first);
	class_destroy(pcie_udev_class);
	if (fpga_pcie_dev.char_device)
		cdev_del(fpga_pcie_dev.char_device);
    unregister_chrdev_region(fpga_pcie_dev.first, count);

    /* unregister this driver from the PCI bus driver */
    pci_unregister_driver(&pcie_driver_ops);
    printk(KERN_INFO "%s: Driver released\n\n", DEV_NAME);
}

module_init(init_driver);
module_exit(exit_driver);

MODULE_AUTHOR("Rich Cahill rcahill@altera.com");
MODULE_DESCRIPTION("Arria 10 PCI Express Driver");
MODULE_LICENSE("GPL");
