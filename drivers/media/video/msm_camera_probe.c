/*
 * Copyright (c) 2009 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/poll.h>

#include <mach/board.h>
#include <mach/camera.h>

#include <media/msm_camera.h>

static int msm_camera_remove(struct platform_device *pdev)
{
	return msm_camera_drv_remove(pdev);
}

static int __init msm_camera_probe(struct platform_device *dev)
{
	int rc;
	rc = msm_camera_drv_start(dev);
	return rc;
}

static struct platform_driver msm_camera_driver = {
	.probe = msm_camera_probe,
	.remove	 = msm_camera_remove,
	.driver = {
		.name = "msm_camera",
		.owner = THIS_MODULE,
	},
};

static int __init msm_camera_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

static void __exit msm_camera_exit(void)
{
	platform_driver_unregister(&msm_camera_driver);
}

module_init(msm_camera_init);
module_exit(msm_camera_exit);
