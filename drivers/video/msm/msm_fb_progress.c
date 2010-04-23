#include <linux/module.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/vt_kern.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>

#include <linux/irq.h>
#include <asm/system.h>

#include "msm_fb.h"

#include "msm_fb_progress.h"

//#define _DEBUG
#ifdef _DEBUG
#define dprintk(s, args...) printk("[fb_progress] %s:%d - " s, __func__, __LINE__,  ##args)
#else
#define dprintk(s, args...)
#endif  /* _DEBUG */

static int progress = 0;

static int progress_flag = 0;

static struct timer_list progress_timer;

extern int msm_fb_resume_sw_refresher(struct msm_fb_data_type *mfd);

static int progress_state=PROGRESS_STOP;

void msm_fb_set_progress_state(int val)    //KSH_TEST
{
	progress_state = val;
}

int msm_fb_get_progress_state(void)
{
	return progress_state;
}

EXPORT_SYMBOL(msm_fb_get_progress_state);

static void progress_timer_handler(unsigned long data)
{
	unsigned short *src, *dst;	
	unsigned short buffer[320*112*2];	
	int	i, j, p, q;
	int pad_width = 42;
	int progress_laststep = 15;
	int progress_start=0, progress_end=252;
	int pad_size, pad_src, pad_dst;
	int extra_start, extra_end, extra_dst;
	int progress_step = 0;

	struct fb_info *info;
	struct msm_fb_data_type *mfd;
	
	info = registered_fb[0];
       mfd = (struct msm_fb_data_type *)info->par;
	   
	dprintk("progress:%d %d\n", progress, mfd->fb_imgType);

	dst = (unsigned short *)(info->screen_base + ((320 * 420) * 2)+60);
//	dst = (unsigned short *)(info->screen_base + ((320 * 167) * 2)+40);

       
#if 0
       src = progress_bar1;

       pad_size = (progress%3) *pad_width;

       if(progress >= progress_laststep)
	   	progress = 0;
	   
       pad_src = progress_end - pad_size;
	

       // copy pad~end to start(0)
	for (i = 0; i < 10; i++) {   //10*252
		for (j = pad_src, pad_dst=progress_start; j < progress_end; j++, pad_dst++) {
			p = ((320 * i) + pad_dst);
			q = (progress_end*i*2)+j;
			*(dst + p) = *(src + q);
		}
	}

      // extra is prgress-pad
	extra_start = progress_start + pad_size;
	extra_end = progress_end - pad_size;

	// copy start(0)~end-pad  to  start+padsize
	for (i = 0; i < 10; i++) {   //10*252
		for (j = progress_start, extra_dst=extra_start; j < extra_end; j++, extra_dst++) {
			p = ((320 * i) + extra_dst);
			q = (progress_end*i*2)+j;
			*(dst + p) = *(src + q);
		}
	}

	progress++;
#endif

      progress_step = progress/3;  //too fast, down 

      if(progress_step == 0)
      {
	  	src = progress_bar0;
      	}
      else if(progress_step == 1)
      {
	  	src = progress_bar1;       	  	
      }
      else if(progress_step == 2)
      {
	  	src = progress_bar2;       	  
      }
      else if(progress_step == 3)
      {
	  	src = progress_bar3;       	  
      	}
      else if(progress_step == 4)
      	{
	  	src = progress_bar4;       	  
      	}
      else if(progress_step == 5)
      	{
	  	src = progress_bar5;       	  
      	}
      else if(progress_step == 6)
      {
	  	src = progress_bar4;       	  	
      }
      else if(progress_step == 7)
      {
	  	src = progress_bar3;       	  
      }
      else if(progress_step == 8)
      {
	  	src = progress_bar2;       	  
      	}
      else if(progress_step == 9)
      	{
	  	src = progress_bar1;       	  
      	}
      else if(progress_step == 10)
      	{
	  	src = progress_bar0;       	  
		progress = 0;
      	}
	  
       for (i = 0; i < 8; i++) {   //10*252
            memcpy(dst+320*i, src+260*i*2,260*2);
       }


	progress++;	   

	progress_timer.expires = (get_jiffies_64() + (HZ/10)); 
	progress_timer.function = progress_timer_handler; 
	add_timer(&progress_timer);
	
}


void msm_fb_start_progress(void)
{
	int fd, err = 0;
	struct fb_info *info;
	struct msm_fb_data_type *mfd;
	
	info = registered_fb[0];
       mfd = (struct msm_fb_data_type *)info->par;
	   
       dprintk("msms_fb_start_progress\n");

	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}

	msm_fb_set_progress_state(PROGRESS_START);
	
	mfd->sw_refreshing_enable = TRUE;
	mfd->sw_currently_refreshing = TRUE;
	mdp_refresh_screen((unsigned long)mfd);

	init_timer(&progress_timer);
	progress_timer.expires = (get_jiffies_64() + (HZ/10)); 
	progress_timer.function = progress_timer_handler; 
	add_timer(&progress_timer);	   
	
}

void msm_fb_stop_progress(void)
{
        msm_fb_set_progress_state(PROGRESS_STOP);
        del_timer(&progress_timer);
}

