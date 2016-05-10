#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <asm/irq.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <asm/arch/regs-gpio.h>
#include <asm/hardware.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/hardware/clock.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <asm/arch/regs-gpio.h>
#include <asm/hardware.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h> 
#include <linux/delay.h>
#include <asm/irq.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/unistd.h>
#include <linux/spinlock.h>
#include <asm/system.h>
#include <asm/uaccess.h>

#include "reg24l01.h"	//24L01

#define CLKCON		0X4C00000C
#define GPECON 		0X56000040
#define GPEDAT 		0X56000044
#define GPEUP 		0X56000048

#define GPFCON 		0X56000050
#define GPFDAT 		0X56000054
#define GPFUP 		0X56000058

#define GPGCON 		0X56000060
#define GPGDAT  	0X56000064
#define GPGUP   	0X56000068

#define GPBCON 		0X56000010
#define GPBDAT 		0X56000014
#define GPBUP  		0X56000018

#define SPI_TXRX_READY      (((inb(spi_spsta0))&0x1) == 0x01) 
 
#define CEH outw(inw(addrGDAT)|(1<<7),addrGDAT)      	//高电平
#define CEL outw(inw(addrGDAT)&(~(1<<7)),addrGDAT)   	//低电平

#define CSNH outw(inw(addrGDAT)|(1<<6),addrGDAT)		//高电平
#define CSNL outw(inw(addrGDAT)&(~(1<<6)),addrGDAT)  	//低电平

uchar  TX_ADDRESS[RX_ADR_WIDTH]= {0x01,0x3d,0x2c,0x1b,0x0a};	//接收地址0（与汇聚节点通信）
static struct clk	*spi_clock;
volatile unsigned long s3c2440_clkcon;  
volatile unsigned long spi_spcon0;    //SPI Part define  
volatile unsigned long spi_spsta0;  
volatile unsigned long spi_sppin0;  
volatile unsigned long spi_sppre0;  
volatile unsigned long spi_senddat0;  
volatile unsigned long spi_resvdat0;  
volatile unsigned long addr;
volatile unsigned long addrGDAT;
volatile unsigned long datb;

#define DEVICE_NAME	"nRF24L01"

#define LED_MAJOR 235

static DECLARE_WAIT_QUEUE_HEAD (nrf24l01_waitq);
static volatile int nrf24l01_resv = 0;
static void spiwrite(const char c)  
{  	
		outb(c,spi_senddat0); 
		while(!SPI_TXRX_READY); 		
}  

static char spiread(void)  
{  
		char ch ; 
		outb(0XFF,spi_senddat0); 
 		while(!SPI_TXRX_READY); 
		ch=inb(spi_resvdat0); 
		return ch;  
} 
uchar spiwrite_reg(uchar reg, uchar value)
{
/*功能：NRF24L01写寄存器函数*/	
	CSNL;                  
	spiwrite(reg);     
	spiwrite(value); 
	CSNH;  	
	return 0; 
}
uchar spiread_reg(uchar reg)
{
/*功能：NRF24L01读寄存器函数*/	
	uchar temp;
	CSNL;                  
	spiwrite(reg);     
	temp=spiread(); 
	CSNH;  	
	return temp; 
}
uchar spiread_buf(uchar reg, uchar *num, uchar count)
{
/*功能: 用于读数据，reg：为寄存器地址，pBuf：为待读出数据地址，uchars：读出数据的个数*/
	uchar i;	
	CSNL; 
	spiwrite(reg); 	
	for(i=0;i<count;i++)
	{
		*(num+i) = spiread();  
	}	
	CSNH;                           	
	return 0;  
}

uchar spiwrite_buf(uchar reg, uchar *num, uchar count)
{	
/*功能: 用于写数据：为寄存器地址，pBuf：为待写入数据地址，uchars：写入数据的个数*/
	uchar i;
	CSNL;         
	spiwrite(reg);   
	for(i=0; i<count; i++) 
	spiwrite(*num++);
	CSNH;           
	return 0;     
}
uchar resvtoram(uchar* num)
{
		/*功能：数据读取后放如num接收缓冲区中*/
		spiread_buf(RD_RX_PLOAD,num,TX_PLOAD_WIDTH);	// 从RX_FIFO 读取数据
		spiwrite_reg(WRITE_REG+STATUS,0X7F);   		//通过写1来清除中断标志。保存发送/接收模式不变
		return 0;						//
}
uchar ramtosend(uchar *num,uchar *address)
{
		/*功能：发送 num中数据*/
	//uchar sstatus;
	CEL;
	spiwrite_reg(WRITE_REG + CONFIG, 0x0e); 
	spiwrite_buf(WRITE_REG + RX_ADDR_P0, address, TX_ADR_WIDTH); 		//与发送地址 保持一致	
	spiwrite_buf(WRITE_REG + TX_ADDR, address, TX_ADR_WIDTH);    		//装载发送地址（汇聚节点）
	spiwrite_buf(WR_TX_PLOAD, num, TX_PLOAD_WIDTH); 			// 装载数据	
	CEH;									//CE上升沿触发发送
	return 0;

}
void nrf24l01_init(void)
{
/*NRF24L01初始化 ,接收模式（如要改为发送模式，再本程序之后调用sendmode()函数） */
 	CEL;    
 	CSNH; 
	spiwrite_reg(WRITE_REG + EN_AA, 0x01);      				//通道0为自动应答模式
	spiwrite_reg(WRITE_REG + EN_RXADDR, 0x1);  				//开启接收通道0
	spiwrite_reg(WRITE_REG + RF_CH, 0);        				//设置信道工作为2.4GHZ
	spiwrite_reg(WRITE_REG + RF_SETUP, 0x07);   				//设置发射速率为1MHZ，发射功率为0dB
	spiwrite_reg(WRITE_REG + SETUP_AW, 0x03); 				//5B address
	spiwrite_reg(WRITE_REG + SETUP_RETR,0x0f);				//250us,15次重发 
	spiwrite_reg(WRITE_REG + CONFIG, 0x0f);   				//清除中断，16位CRC默认接收，
	spiwrite_reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); 		 	//接收数据宽度~~P0
	spiwrite_buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    	//装载默认发送地址（汇聚节点）
	spiwrite_buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, RX_ADR_WIDTH); 	//装载接收P0地址~~ 要求RX_ADDRESS_P0=TX_ADDRESS
	spiwrite_reg(WRITE_REG+STATUS,0X7F);					//清除中断标志位
	spiwrite_reg(0xe1,0XFF);											//清空缓存；
	spiwrite_reg(0xe2,0XFF);
	CEH;									//start resv；
}


static int tyled_open(struct inode *inode, struct file *filp)
{

	s3c2440_clkcon   = (long)ioremap(CLKCON,4); 
	datb		=inl(s3c2440_clkcon) | (1<<18);
	outl(datb,s3c2440_clkcon);  //spi sck enabled;
	printk("CLKCON=%X\n",(int)inl(s3c2440_clkcon));

	addr		=(long)ioremap(GPECON,4);
	datb		=inl(addr);
	datb		= datb &(~((0X3<<26) | (0X3<<24) | (0X3<<22)) );
	datb		= datb | ( (0X2<<26) | (0X2<<24) | (0X2<<22) );//chang it now?????
	outl(datb,addr);	
	printk("datb=%X    GPECON=%X\n",(int)datb,(int)inl(addr));

	addr		=(long)ioremap(GPEUP,2);
	datb		=inw(addr);
	datb		&=0XC7FF;
	datb		|=0x2000;
	outl(datb,addr);
	printk("GPEUP=%X\n",(int)inl(addr));

	addr		=(long)ioremap(GPGCON,2);
	datb		=inw(addr);
	datb		=datb &~( (0x3<<10) | (0x3<<12) | (0x3<<14) );  
	datb		=datb  | ( (0x2<<10) | (0x1<<12) | (0x1<<14) );  //GPG5=EINT13,GPG6=OUT,GPG7=OUT
	outw(datb,addr);

	addr		=(long)ioremap(GPGUP,2);
	datb		=inw(addr);
	datb 		=datb & ~( (0x1<<5) | (0x1<<6) | (0x1<<7) );//all up !!
	outl(datb,addr);

	addrGDAT	= (long)ioremap(GPGDAT,2);
  	spi_spcon0 	= (long)ioremap(0x59000000,1);  
    	spi_spsta0  	= (long)ioremap(0x59000004,1);  
    	spi_sppin0 	= (long)ioremap(0x59000008,1);  
    	spi_sppre0 	= (long)ioremap(0x5900000c,1);  
    	spi_senddat0 	= (long)ioremap(0x59000010,1);  
    	spi_resvdat0 	= (long)ioremap(0x59000014,1); 
	
	
	outb((0<<6)|(0<<5)|(1<<4)|(1<<3)|(0<<2)|(0<<1)|(0<<0),spi_spcon0);//Tx auto garbage data mode
	outb(0x01,spi_spsta0);
	outb((0<<2)|(0<<1)|(0<<0),spi_sppin0);
	outb(0xfe,spi_sppre0);
	nrf24l01_init();
	printk("\n<<<<<<<<<<<open sussess>>>>>>>>>>>\n");
	return 0;		//must do it!!
}
static int tyled_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
switch(cmd) 
   {
	case 0:
		{	
			if (arg) 
			{
				spiwrite_reg(WRITE_REG+STATUS,0X7F);//清除中断标志位
				spiwrite_reg(0xe1,0XFF);											//清空缓存；
				spiwrite_reg(0xe2,0XFF);				
				spiwrite_reg(WRITE_REG + CONFIG, 0x0e);
				CEH;
				printk("spend mode !!\n");
				
			}
			else
			{
				spiwrite_reg(WRITE_REG+STATUS,0X7F);//清除中断标志位
				spiwrite_reg(0xe1,0XFF);											//清空缓存；
				spiwrite_reg(0xe2,0XFF);	
				spiwrite_reg(WRITE_REG + CONFIG, 0x0f);
				CEH;
				printk("resv mode !!\n");
			}
			break;
		}
		
	case 1:
		{	
			printk("negtive!!!\n");
			break;
		}
	default:
		{
			printk("negtive!!!\n");
		}

   }
return -EINVAL;
}

static ssize_t tyled_write(struct file * file,const char * buf,size_t count,loff_t* f_ops)
{	
	uchar temp[TX_PLOAD_WIDTH];
//	printk("enter write ok temp\n");
	copy_from_user(&temp,buf,TX_PLOAD_WIDTH);
	ramtosend(temp,TX_ADDRESS);
	return count;
}
static ssize_t tyled_read(struct file * file,char * buf,size_t count,loff_t* f_ops)
{	
	uchar temp[TX_PLOAD_WIDTH];
//	printk("read enter !!!\n");
	if(nrf24l01_resv)
		{
			nrf24l01_resv=0;
			resvtoram(temp);
		}
	else
		{
			wait_event_interruptible(nrf24l01_waitq, nrf24l01_resv);
			nrf24l01_resv=0;
			resvtoram(temp);
		}
	copy_to_user(buf,&temp,TX_PLOAD_WIDTH);
	return count;
}
static irqreturn_t nrf24l01_interrupt(int irq,void *dunmmy,struct pt_regs *fp)
{
	uchar sstatus;
	sstatus=spiread_reg(STATUS);
//	printk("STATUS=%X\n",sstatus);
	if(TX_DS(sstatus))
		{
			spiwrite_reg(WRITE_REG+STATUS,sstatus);
			//printk("nRF24L01 sussesed\n");		//send susses
		}
	else if(MAX_RT(sstatus)) 								//send fail
		{
			spiwrite_reg(0xe1,0XFF);			// clear TX buffer				//冲洗TX BUFF
			spiwrite_reg(WRITE_REG+STATUS,sstatus);	
			//printk("nRF24L01 timeout\n");		//send susses						
		}
	else if(RX_DR(sstatus))								//resv 
		{
			spiwrite_reg(WRITE_REG+STATUS,sstatus);
			nrf24l01_resv=1;							//resv susses!!
			wake_up_interruptible(&nrf24l01_waitq);   				/*唤醒休眠的进程 */
		}
	return 0;
}
static struct file_operations tyled_fops =
{
    .owner=	THIS_MODULE,
    .open=	tyled_open,
    .read=	tyled_read,
    .write=	tyled_write,
    .ioctl=	tyled_ioctl,
};

static int __init tyled_init(void)
{
	int ret;
	ret = register_chrdev(LED_MAJOR, DEVICE_NAME, &tyled_fops);
	if (ret < 0) 
	{
	  printk(DEVICE_NAME " can't register major number\n");
	  return ret;
	}
	devfs_mk_cdev(MKDEV(LED_MAJOR, 0), S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP, DEVICE_NAME);
	if(!request_mem_region(GPECON,4,"USE BY SPI"))
	{
		printk("error GPECON is not free ");
		return -EINVAL;
	}
	spi_clock = clk_get(NULL, "spi");//申请ADC时钟
	if (!spi_clock)
	{
		printk(KERN_ERR "failed to get adc clock source\n");
		return -ENOENT;
	}
	clk_use(spi_clock);
	clk_enable(spi_clock);//ENABLE ADCCLK
	ret=request_irq(IRQ_EINT13,nrf24l01_interrupt,0,"nrf24l01",NULL);
	set_irq_type(IRQ_EINT13, __IRQT_FALEDGE);
	if(!ret)
	{
		printk("IRQ fail ret=%X",ret);
	}
	printk("<<<<<<<spi initialized , ty works>>>>>>\n");
	return 0;
}


static void __exit tyled_exit(void)
{
	devfs_remove(DEVICE_NAME);
	unregister_chrdev(LED_MAJOR, DEVICE_NAME);
    	printk("<1>spi_exit!\n");
}


module_init(tyled_init);
module_exit(tyled_exit);
MODULE_LICENSE("GPL");

