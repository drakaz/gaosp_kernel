#include <linux/slab.h>
#include "phy_nand.h"
#include "../../../arch/arm/mach-msm/include/mach/dma.h"

typedef struct param {
	int into_charging_mode;
	int jtag_download_mode;
	int temp1;
	int temp2;
} PARAMETER;

static unsigned CFG0, CFG1;
static void *flash_spare;
static void *flash_data;
unsigned *flash_ptrlist;
dmov_s *flash_cmdlist;
static unsigned flash_maker = 0;
static unsigned flash_device = 0;
static read_config_with_ecc = 0;
unsigned nand_cfg0;
unsigned nand_cfg1;
 
#define CFG1_WIDE_FLASH (1U << 1)
#define paddr(n) ((unsigned) (n))

#define SRC_CRCI_NAND_CMD  CMD_SRC_CRCI(DMOV_NAND_CRCI_CMD)
#define DST_CRCI_NAND_CMD  CMD_DST_CRCI(DMOV_NAND_CRCI_CMD)
#define SRC_CRCI_NAND_DATA CMD_SRC_CRCI(DMOV_NAND_CRCI_DATA)
#define DST_CRCI_NAND_DATA CMD_DST_CRCI(DMOV_NAND_CRCI_DATA)

static int flash_init(void);
static int flash_read_config_to_ecc(dmov_s *cmdlist, unsigned *ptrlist);
static void flash_read_id(dmov_s *cmdlist, unsigned *ptrlist);
static int flash_erase_block(dmov_s *cmdlist, unsigned *ptrlist, unsigned page);
static int _flash_write_page_with_ecc(dmov_s *cmdlist, unsigned *ptrlist, unsigned page,
		                             const void *_addr, const void *_spareaddr);

struct data_flash_io {
	unsigned cmd;
	unsigned addr0;
	unsigned addr1;
	unsigned chipsel;
	unsigned cfg0;
	unsigned cfg1;
	unsigned exec;
	unsigned ecc_cfg;
	unsigned ecc_cfg_save;
	struct {
		unsigned flash_status;
		unsigned buffer_status;
	} result[4];
};

typedef struct dmov_ch dmov_ch;
struct dmov_ch
{
	volatile unsigned cmd;
	volatile unsigned result;
	volatile unsigned status;
	volatile unsigned config;
};

static void dmov_prep_ch(dmov_ch *ch, unsigned id)
{
	ch->cmd = DMOV_CMD_PTR(id);
	ch->result = DMOV_RSLT(id);
	ch->status = DMOV_STATUS(id);
	ch->config = DMOV_CONFIG(id);
}


static inline void DWB(void) /* drain write buffer */
{
	asm volatile (
			".word 0xf57ff04f\n"  /* dsb */
			".word 0xf57ff06f\n"  /* isb */
			);
}
static inline void writel(unsigned val, unsigned addr)
{
	DWB();
	(*(volatile unsigned *) (addr)) = (val);
	DWB();
}

static inline void writeb(unsigned val, unsigned addr)
{
	DWB();
	(*(volatile unsigned char *) (addr)) = (val);
	DWB();
}

static inline unsigned readl(unsigned addr)
{
	return (*(volatile unsigned *) (addr));
}

static int dmov_exec_cmdptr(unsigned id, unsigned *ptr)
{
	dmov_ch ch;
	unsigned n;

	dmov_prep_ch(&ch, id);

	writel(DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(paddr(ptr)), ch.cmd);

	while(!(readl(ch.status) & DMOV_STATUS_RSLT_VALID)) ;

	n = readl(ch.status);
	while(DMOV_STATUS_RSLT_COUNT(n)) {
		n = readl(ch.result);
		if(n != 0x80000002) {
#if VERBOSE
			printk("ERROR: result: %x\n", n);
			printk("ERROR:  flush: %x %x %x %x\n",
					readl(DMOV_FLUSH0(DMOV_NAND_CHAN)),
					readl(DMOV_FLUSH1(DMOV_NAND_CHAN)),
					readl(DMOV_FLUSH2(DMOV_NAND_CHAN)),
					readl(DMOV_FLUSH3(DMOV_NAND_CHAN)));
#endif
		}
		n = readl(ch.status);
	}

	return 0;
}

#define BLOCK_SIZE				2048
#define PARAMETER_FLASH_BASE	2046
#define PARAMETER_BLOCK_NUM		2
#define PARAMETER_FLASH_LENGTH	2 * (128 * 1024)

int mark_jtag_download_mode(void)
{
	int i, n;
	int page;
	unsigned *spare = (unsigned*)flash_spare;
	PARAMETER param;
	flash_init();

	param.into_charging_mode = 0;
	param.jtag_download_mode = 1;
	param.temp1 = 0;
	param.temp2 = 0;

	memcpy(flash_data, &param, BLOCK_SIZE);

	for(n = 0; n < 16; n++)
		spare[n] = 0xffffffff;
	
	page = PARAMETER_FLASH_BASE * 64;
	
	for(i = 0; i < PARAMETER_BLOCK_NUM; i++) {
		if(flash_erase_block(flash_cmdlist, flash_ptrlist, page)) {
			printk("flash_write_image: bad block @ %d\n", page >> 6);
			page += 64;
			continue;
		}
		break;
	}

	if(i == PARAMETER_BLOCK_NUM) {
		printk("All parameter block are bad block!\n");
		for(;;);
	}

	_flash_write_page_with_ecc(flash_cmdlist, flash_ptrlist, page, flash_data, flash_spare);

	kfree(flash_ptrlist);
	kfree(flash_cmdlist);
}
EXPORT_SYMBOL(mark_jtag_download_mode);

static int flash_init(void)
{
	flash_ptrlist = kmalloc(1024, __GFP_DMA);
	flash_cmdlist = kmalloc(1024, __GFP_DMA);
	flash_data = kmalloc(2048, GFP_KERNEL);
	flash_spare = kmalloc(64, GFP_KERNEL);

	if(flash_read_config_to_ecc(flash_cmdlist, flash_ptrlist)) {
		for(;;);
	}

	flash_read_id(flash_cmdlist, flash_ptrlist);
	return 0;
}

static void flash_read_id(dmov_s *cmdlist, unsigned *ptrlist)
{
    dmov_s *cmd = cmdlist;
    unsigned *ptr = ptrlist;
    unsigned *data = ptrlist + 4;

    data[0] = 0 | 4;
    data[1] = NAND_CMD_FETCH_ID;
    data[2] = 1;
    data[3] = 0;
    data[4] = 0;
    
    cmd[0].cmd = 0 | CMD_OCB;
    cmd[0].src = paddr(&data[0]);
    cmd[0].dst = NAND_FLASH_CHIP_SELECT;
    cmd[0].len = 4;

    cmd[1].cmd = DST_CRCI_NAND_CMD;
    cmd[1].src = paddr(&data[1]);
    cmd[1].dst = NAND_FLASH_CMD;
    cmd[1].len = 4;

    cmd[2].cmd = 0;
    cmd[2].src = paddr(&data[2]);
    cmd[2].dst = NAND_EXEC_CMD;
    cmd[2].len = 4;

    cmd[3].cmd = SRC_CRCI_NAND_DATA;
    cmd[3].src = NAND_FLASH_STATUS;
    cmd[3].dst = paddr(&data[3]);
    cmd[3].len = 4;

    cmd[4].cmd = CMD_OCU | CMD_LC;
    cmd[4].src = NAND_READ_ID;
    cmd[4].dst = paddr(&data[4]);
    cmd[4].len = 4;
    
    ptr[0] = (paddr(cmd) >> 3) | CMD_PTR_LP;

    dmov_exec_cmdptr(DMOV_NAND_CHAN, ptr);

#if VERBOSE
    cprintf("status: %x\n", data[3]);
#endif

#if PRINTINFO
    cprintf("nandid: %x maker %b device %b\n",
            data[4], data[4] & 0xff, (data[4] >> 8) & 0xff);
#endif
    flash_maker = data[4] & 0xff;
    flash_device = (data[4] >> 8) & 0xff;
}

static int flash_read_config_to_ecc(dmov_s *cmdlist, unsigned *ptrlist)
{
    cmdlist[0].cmd = CMD_OCB;
    cmdlist[0].src = NAND_DEV0_CFG0;
    cmdlist[0].dst = paddr(&CFG0);
    cmdlist[0].len = 4;

    cmdlist[1].cmd = CMD_OCU | CMD_LC;
    cmdlist[1].src = NAND_DEV0_CFG1;
    cmdlist[1].dst = paddr(&CFG1);
    cmdlist[1].len = 4;

    *ptrlist = (paddr(cmdlist) >> 3) | CMD_PTR_LP;

    dmov_exec_cmdptr(DMOV_NAND_CHAN, ptrlist);

    if((CFG0 == 0) || (CFG1 == 0)) {
        return -1;
    }
    
	CFG0 = (3 <<  6)  /* 4 codeword per page for 2k nand */
		|  (516 <<  9)  /* 516 user data bytes */
		|   (10 << 19)  /* 10 parity bytes */
		|    (5 << 27)  /* 5 address cycles */
		|    (1 << 30)  /* Read status before data */
		|    (1 << 31)  /* Send read cmd */
            /* 0 spare bytes for 16 bit nand or 1 spare bytes for 8 bit */
		| ((nand_cfg1 & CFG1_WIDE_FLASH) ? (0 << 23) : (1 << 23));
	CFG1 = (0 <<  0)  /* Enable ecc */
		|    (7 <<  2)  /* 8 recovery cycles */
		|    (0 <<  5)  /* Allow CS deassertion */
		|  (465 <<  6)  /* Bad block marker location */
		|    (0 << 16)  /* Bad block in user data area */
		|    (2 << 17)  /* 6 cycle tWB/tRB */
		| (nand_cfg1 & CFG1_WIDE_FLASH); /* preserve wide flash flag */

	read_config_with_ecc = 1;

    return 0;
}

static int flash_erase_block(dmov_s *cmdlist, unsigned *ptrlist, unsigned page)
{
	dmov_s *cmd = cmdlist;
	unsigned *ptr = ptrlist;
	unsigned *data = ptrlist + 4;

	/* only allow erasing on block boundaries */
	if(page & 63) return -1;

	data[0] = NAND_CMD_BLOCK_ERASE;
	data[1] = page;
	data[2] = 0;
	data[3] = 0 | 4;
	data[4] = 1;
	data[5] = 0xeeeeeeee;
	data[6] = CFG0 & (~(7 << 6));  /* CW_PER_PAGE = 0 */
	data[7] = CFG1;

	cmd[0].cmd = DST_CRCI_NAND_CMD | CMD_OCB;
	cmd[0].src = paddr(&data[0]);
	cmd[0].dst = NAND_FLASH_CMD;
	cmd[0].len = 16;

	cmd[1].cmd = 0;
	cmd[1].src = paddr(&data[6]);
	cmd[1].dst = NAND_DEV0_CFG0;
	cmd[1].len = 8;

	cmd[2].cmd = 0;
	cmd[2].src = paddr(&data[4]);
	cmd[2].dst = NAND_EXEC_CMD;
	cmd[2].len = 4;

	cmd[3].cmd = SRC_CRCI_NAND_DATA | CMD_OCU | CMD_LC;
	cmd[3].src = NAND_FLASH_STATUS;
	cmd[3].dst = paddr(&data[5]);
	cmd[3].len = 4;

	ptr[0] = (paddr(cmd) >> 3) | CMD_PTR_LP;

	dmov_exec_cmdptr(DMOV_NAND_CHAN, ptr);

	if(data[5] & 0x110) return -1;
	if(!(data[5] & 0x80)) return -1;

	return 0;
}

static int _flash_write_page_with_ecc(dmov_s *cmdlist, unsigned *ptrlist, unsigned page,
		                             const void *_addr, const void *_spareaddr)
{
	dmov_s *cmd = cmdlist;
	unsigned *ptr = ptrlist;
	struct data_flash_io *data = (void*) (ptrlist + 4);
	unsigned addr = (unsigned) _addr;
	unsigned spareaddr = (unsigned) _spareaddr;
	unsigned n;

	data->cmd = NAND_CMD_PRG_PAGE;
	data->addr0 = page << 16;
	data->addr1 = (page >> 16) & 0xff;
	data->chipsel = 0 | 4; /* flash0 + undoc bit */

	data->cfg0 = CFG0;
	data->cfg1 = CFG1;

	/* GO bit for the
	 * EXEC register */
	data->exec = 1;
	data->ecc_cfg = 0x203;

	/* save
	 * existing
	 * ecc
	 * config */
	cmd->cmd = CMD_OCB;
	cmd->src = NAND_EBI2_ECC_BUF_CFG;
	cmd->dst = paddr(&data->ecc_cfg_save);
	cmd->len = 4;
	cmd++;

	for(n = 0; n < 4; n++) {
		/* write CMD / ADDR0 / ADDR1 / CHIPSEL regs in a burst */
		cmd->cmd = DST_CRCI_NAND_CMD;
		cmd->src = paddr(&data->cmd);
		cmd->dst = NAND_FLASH_CMD;
		cmd->len = ((n == 0) ? 16 : 4);
		cmd++;

		if (n == 0) {
			/* set configuration */
			cmd->cmd = 0;
			cmd->src = paddr(&data->cfg0);
			cmd->dst = NAND_DEV0_CFG0;
			cmd->len = 8;
			cmd++;

			/* set our ecc config */
			cmd->cmd = 0;
			cmd->src = paddr(&data->ecc_cfg);
			cmd->dst = NAND_EBI2_ECC_BUF_CFG;
			cmd->len = 4;
			cmd++;
		}

		/* write data block */
		cmd->cmd = 0;
		cmd->src = addr + n * 516;
		cmd->dst = NAND_FLASH_BUFFER;
		cmd->len = ((n < 3) ? 516 : 510);
		cmd++;

		if (n == 3) {
			/* write extra data */
			cmd->cmd = 0;
			cmd->src = spareaddr;
			cmd->dst = NAND_FLASH_BUFFER + 500;
			cmd->len = 16;
			cmd++;
		}

		/* kick the execute register */
		cmd->cmd = 0;
		cmd->src = paddr(&data->exec);
		cmd->dst = NAND_EXEC_CMD;
		cmd->len = 4;
		cmd++;

		/* block on data ready, then read the status register */
		cmd->cmd = SRC_CRCI_NAND_DATA;
		cmd->src = NAND_FLASH_STATUS;
		cmd->dst = paddr(&data->result[n]);
		cmd->len = 8;
		cmd++;
	}

	/* restore saved ecc config */
	cmd->cmd = CMD_OCU | CMD_LC;
	cmd->src = paddr(&data->ecc_cfg_save);
	cmd->dst = NAND_EBI2_ECC_BUF_CFG;
	cmd->len = 4;

	ptr[0] = (paddr(cmdlist) >> 3) | CMD_PTR_LP;

	dmov_exec_cmdptr(DMOV_NAND_CHAN, ptr);

#if VERBOSE
	dprintf("write page %d: status: %x %x %x %x\n",
			page, data[5], data[6], data[7], data[8]);
#endif

	/* if any of the writes failed (0x10), or there was a
	 ** protection violation (0x100), or the program success
	 ** bit (0x80) is unset, we lose */
	for(n = 0; n < 4; n++) {
		if(data->result[n].flash_status & 0x110) return -1;
		if(!(data->result[n].flash_status & 0x80)) return -1;
	}

	return 0;
}
