/*****************************************************************************
 * debugfs entries See Documentation/filesystems/debugfs.txt
 *****************************************************************************/

static void vip_regs_show_reg(struct seq_file *s, const char *nm, u32 reg)
{
	struct lf2000_vip_dev *pcdev = s->private;

	seq_printf(s, "%14s:  0x%04X\n", nm, readw(pcdev->base + reg));
}

static void vip_regs_show_reg_d(struct seq_file *s, const char *nm, u32 reg)
{
	struct lf2000_vip_dev *pcdev = s->private;
	u16 val = readw(pcdev->base + reg);

	seq_printf(s, "%14s:  0x%04X (%u)\n", nm, val, val);
}

static void vip_regs_show_regl(struct seq_file *s, const char *nm, u32 reg)
{
	struct lf2000_vip_dev *pcdev = s->private;

	seq_printf(s, "%14s:  0x%08X\n", nm, readl(pcdev->base + reg));
}

static int vip_deci_regs_show(struct seq_file *s, void *v)
{
	struct lf2000_vip_dev *pcdev = s->private;
	u16 val;
	s16 ext;

	seq_printf(s, "%9s:  %u\n", "CHANNEL", pcdev->pdev->id);
	seq_printf(s, "%9s:  0x%p\n", "ADDRESS", pcdev->base);
	seq_printf(s, "\n");

	vip_regs_show_reg_d(s, "DECI_TARGETW", VIP_DECI_TARGETW);
	vip_regs_show_reg_d(s, "DECI_TARGETH", VIP_DECI_TARGETH);
	vip_regs_show_reg_d(s, "DECI_DELTAW", VIP_DECI_DELTAW);
	vip_regs_show_reg_d(s, "DECI_DELTAH", VIP_DECI_DELTAH);
	/* Truncated 2's complement.  Need to sign-extend. */
	val = readw(pcdev->base + VIP_DECI_CLEARW);
	ext = val;
	if(val & 0x1000)
		ext |= 0xE000;
	seq_printf(s, "%14s:  0x%04X (%hd)\n", "DECI_CLEARW", val, ext);

	val = readw(pcdev->base + VIP_DECI_CLEARH);
	ext = val;
	if(val & 0x0800)
		ext |= 0xF000;
	seq_printf(s, "%14s:  0x%04X (%hd)\n", "DECI_CLEARH",  val, ext);

	vip_regs_show_reg(s, "DECI_LUSEG", VIP_DECI_LUSEG);
	vip_regs_show_reg(s, "DECI_CRSEG", VIP_DECI_CRSEG);
	vip_regs_show_reg(s, "DECI_CBSEG", VIP_DECI_CBSEG);
	vip_regs_show_reg(s, "DECI_FORMAT", VIP_DECI_FORMAT);	
	vip_regs_show_reg(s, "DECI_ROTFLIP", VIP_DECI_ROTFLIP);
	vip_regs_show_reg_d(s, "DECI_LULEFT", VIP_DECI_LULEFT);
	vip_regs_show_reg_d(s, "DECI_CRLEFT", VIP_DECI_CRLEFT);
	vip_regs_show_reg_d(s, "DECI_CBLEFT", VIP_DECI_CBLEFT);
	vip_regs_show_reg_d(s, "DECI_LURIGHT", VIP_DECI_LURIGHT);
	vip_regs_show_reg_d(s, "DECI_CRRIGHT", VIP_DECI_CRRIGHT);
	vip_regs_show_reg_d(s, "DECI_CBRIGHT", VIP_DECI_CBRIGHT);
	vip_regs_show_reg_d(s, "DECI_LUTOP", VIP_DECI_LUTOP);
	vip_regs_show_reg_d(s, "DECI_CRTOP", VIP_DECI_CRTOP);
	vip_regs_show_reg_d(s, "DECI_CBTOP", VIP_DECI_CBTOP);
	vip_regs_show_reg_d(s, "DECI_LUBOTTOM", VIP_DECI_LUBOTTOM);
	vip_regs_show_reg_d(s, "DECI_CRBOTTOM", VIP_DECI_CRBOTTOM);
	vip_regs_show_reg_d(s, "DECI_CBBOTTOM", VIP_DECI_CBBOTTOM);

	return 0;
}

static int vip_clip_regs_show(struct seq_file *s, void *v)
{
	struct lf2000_vip_dev *pcdev = s->private;
	u32 stride;

	seq_printf(s, "%9s:  %u\n", "CHANNEL", pcdev->pdev->id);
	seq_printf(s, "%9s:  0x%p\n", "ADDRESS", pcdev->base);
	seq_printf(s, "\n");
	vip_regs_show_reg_d(s, "CLIP_LEFT", VIP_CLIP_LEFT);
	vip_regs_show_reg_d(s, "CLIP_RIGHT", VIP_CLIP_RIGHT);
	vip_regs_show_reg_d(s, "CLIP_TOP", VIP_CLIP_TOP);
	vip_regs_show_reg_d(s, "CLIP_BOTTOM", VIP_CLIP_BOTTOM);

	vip_regs_show_reg(s, "CLIP_LUSEG", VIP_CLIP_LUSEG);
	vip_regs_show_reg(s, "CLIP_CRSEG", VIP_CLIP_CRSEG);
	vip_regs_show_reg(s, "CLIP_CBSEG", VIP_CLIP_CBSEG);
	vip_regs_show_reg(s, "CLIP_FORMAT", VIP_CLIP_FORMAT);	
	vip_regs_show_reg(s, "CLIP_ROTFLIP", VIP_CLIP_ROTFLIP);
	vip_regs_show_reg_d(s, "CLIP_LULEFT", VIP_CLIP_LULEFT);
	vip_regs_show_reg_d(s, "CLIP_CRLEFT", VIP_CLIP_CRLEFT);
	vip_regs_show_reg_d(s, "CLIP_CBLEFT", VIP_CLIP_CBLEFT);
	vip_regs_show_reg_d(s, "CLIP_LURIGHT", VIP_CLIP_LURIGHT);
	vip_regs_show_reg_d(s, "CLIP_CRRIGHT", VIP_CLIP_CRRIGHT);
	vip_regs_show_reg_d(s, "CLIP_CBRIGHT", VIP_CLIP_CBRIGHT);
	vip_regs_show_reg_d(s, "CLIP_LUTOP", VIP_CLIP_LUTOP);
	vip_regs_show_reg_d(s, "CLIP_CRTOP", VIP_CLIP_CRTOP);
	vip_regs_show_reg_d(s, "CLIP_CBTOP", VIP_CLIP_CBTOP);
	vip_regs_show_reg_d(s, "CLIP_LUBOTTOM", VIP_CLIP_LUBOTTOM);
	vip_regs_show_reg_d(s, "CLIP_CRBOTTOM", VIP_CLIP_CRBOTTOM);
	vip_regs_show_reg_d(s, "CLIP_CBBOTTOM", VIP_CLIP_CBBOTTOM);

	vip_regs_show_reg(s, "CLIP_YUYVENB", VIP_CLIP_YUYVENB);
	vip_regs_show_reg(s, "CLIP_BASEADDRH", VIP_CLIP_BASEADDRH);
	vip_regs_show_reg(s, "CLIP_BASEADDRL", VIP_CLIP_BASEADDRL);
	vip_regs_show_reg(s, "CLIP_STRIDEH", VIP_CLIP_STRIDEH);
	vip_regs_show_reg(s, "CLIP_STRIDEL", VIP_CLIP_STRIDEL);

	stride = readw(pcdev->base + VIP_CLIP_STRIDEH);
	stride <<= 16;
	stride |= readw(pcdev->base + VIP_CLIP_STRIDEL); 
	seq_printf(s, "%14s:  0x%08X (%u)\n", "YUYV stride", stride, stride);

	return 0;
}

static int vip_regs_show(struct seq_file *s, void *v)
{
	struct lf2000_vip_dev *pcdev = s->private;

	seq_printf(s, "%9s:  %u\n", "CHANNEL", pcdev->pdev->id);
	seq_printf(s, "%9s:  0x%p\n", "ADDRESS", pcdev->base);
	seq_printf(s, "\n");
	vip_regs_show_reg(s, "CONFIG", VIP_CONFIG);
	vip_regs_show_reg(s, "INTCTRL", VIP_INTCTRL);
	vip_regs_show_reg(s, "SYNCCTRL", VIP_SYNCCTRL);
	vip_regs_show_reg(s, "SYNCMON", VIP_SYNCMON);
	vip_regs_show_reg_d(s, "VBEGIN", VIP_VBEGIN);
	vip_regs_show_reg_d(s, "VEND", VIP_VEND);
	vip_regs_show_reg_d(s, "HBEGIN", VIP_HBEGIN);
	vip_regs_show_reg_d(s, "HEND", VIP_HEND);
	vip_regs_show_reg(s, "FIFOCTRL", VIP_FIFOCTRL);
	vip_regs_show_reg_d(s, "HCOUNT", VIP_HCOUNT);
	vip_regs_show_reg_d(s, "VCOUNT", VIP_VCOUNT);
	vip_regs_show_reg(s, "VCLKOUTSEL", VIP_VCLKOUTSEL);
	vip_regs_show_reg(s, "CDENB", VIP_CDENB);
	vip_regs_show_reg(s, "ODINT", VIP_ODINT);
	vip_regs_show_reg_d(s, "IMGWIDTH", VIP_IMGWIDTH);
	vip_regs_show_reg_d(s, "IMGHEIGHT", VIP_IMGHEIGHT);

	vip_regs_show_reg(s, "VIP1", VIP_VIP1);

	vip_regs_show_regl(s, "CLKENB", VIP_CLKENB);
	vip_regs_show_regl(s, "CLKGENL", VIP_CLKGENL);
	vip_regs_show_regl(s, "CLKGENH", VIP_CLKGENH);

	vip_regs_show_reg(s, "SCANMODE", VIP_SCANMODE);

	return 0;
}

static int vip_regs_open(struct inode *inode, struct file *file)
{
	struct lf2000_vip_dev *pcdev = inode->i_private;

	if(file->f_dentry == pcdev->debug_registers)
	{
		return single_open(file, vip_regs_show, inode->i_private);
	}
	if(file->f_dentry == pcdev->debug_clip_registers)
	{
		return single_open(file, vip_clip_regs_show, inode->i_private);
	}
	if(file->f_dentry == pcdev->debug_deci_registers)
	{
		return single_open(file, vip_deci_regs_show, inode->i_private);
	}

	return -ENOENT;
}

static const struct file_operations vip_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= vip_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void vip_init_debugfs(struct lf2000_vip_dev *pcdev)
{
	struct dentry *dir;
 
	dir = debugfs_create_dir(pcdev->name, NULL);
	if (!dir || IS_ERR(dir)) {
		pcdev->debug = NULL;
		return;
	}

	pcdev->debug_registers = debugfs_create_file("registers", S_IRUSR,
				dir, pcdev, &vip_regs_fops);

	pcdev->debug_clip_registers = debugfs_create_file("clipper", S_IRUSR,
				dir, pcdev, &vip_regs_fops);

	pcdev->debug_deci_registers = debugfs_create_file("decimator", S_IRUSR,
				dir, pcdev, &vip_regs_fops);

	pcdev->debug = dir;
}
