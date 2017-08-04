#define DEV_NAME "arria10_pcie_fpga" 

#define ALTERA_DMA_BAR_NUM (6)
#define ALTERA_DMA_DESCRIPTOR_NUM 128

//IOCTL Control Variables
//Copy this struct to User Program
struct pcie_ioctl {
    int bar_select;
    unsigned long bar_start[ALTERA_DMA_BAR_NUM];
    unsigned long bar_end[ALTERA_DMA_BAR_NUM];
    unsigned long bar_length[ALTERA_DMA_BAR_NUM];       // PCI Bar length
    unsigned long bar_flags[ALTERA_DMA_BAR_NUM]; 
};

// Character device structure
struct pci_cdev_struct {
    struct device *device_driver;                       /* Centralized Device Driver model needed to model the system */
    struct cdev *char_device;         		            /* Standard Char device structure */
    struct pci_dev *pci_device;        		            /* PCI device structure handle */
    dev_t first;                    		            /* Major and minor numbers for char device */
    void * __iomem bar[ALTERA_DMA_BAR_NUM];             // PCI Bar User space address
    char name[16];                                      // Device Name
    int vendor_id;                                      // Vendor ID
    int device_id;                                      // Device ID
    char msi_enabled;
    u32 msi_cap[4];
    u16 msi_control_message;
    wait_queue_head_t my_wq;
    atomic_t intr_disabled;
    u8 irq_pin;
    u8 irq_line;
    int irq_count;
    u8 revision;

    struct pcie_ioctl pcie_control ;
    
};


//IOCTL Commands
#define IOCTL_BAR_GET _IOR('q', 1, struct pcie_ioctl *)
#define IOCTL_BAR_SET _IOW('q', 2, struct pcie_ioctl *)