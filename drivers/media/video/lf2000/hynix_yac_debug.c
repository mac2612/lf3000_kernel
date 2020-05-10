/*****************************************************************************
 * debugfs entries See Documentation/filesystems/debugfs.txt
 *****************************************************************************/

struct hynix_debug {
	const enum register_page	page;
	struct hynix_priv		*priv;

	int (*const show)(struct seq_file *s, void *v);

	const char			*name;
};

static DEFINE_MUTEX(debugfs_mutex);

#define DUMP(reg) \
	hynix_read(client, debug->page, reg, &val); \
	seq_printf(s, "%12s:  0x%02X\n", #reg, val);

#if 0
// To convert hynix_regs.h to DUMP() list:
// # cut -f 2 -d ' ' | cut -f 1 | sed 's/^.*/\tDUMP\(&\);/g'
#endif

static int show_window_page(struct seq_file *s, void *v)
{
	struct hynix_debug *debug = s->private;
	struct hynix_priv *priv = v;
	struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);
	int ret;
	u8 val;

	if(0 != (ret = hynix_read(client, debug->page, PWRCTL, &val)))
		return ret;

	DUMP(PWRCTL);
	DUMP(DEVID);
	DUMP(PLLCTL1);
	DUMP(PLLCTL2);
	DUMP(VDOCTL1);
	DUMP(VDOCTL2);
	DUMP(SYNCCTL);
	if(priv->type == HI253)
	{
		DUMP(HREFCTL);
	}
	DUMP(WINROWH);
	DUMP(WINROWL);
	DUMP(WINCOLH);
	DUMP(WINCOLL);
	DUMP(WINHGTH);
	DUMP(WINHGTL);
	DUMP(WINWIDH);
	DUMP(WINWIDL);
	if(priv->type == SR300PC10)
	{ 	
		DUMP(BIN3WIDH);
		DUMP(BIN3WIDL);
	}
	DUMP(HBLANKH);
	DUMP(HBLANKL);
	DUMP(VSYNCH);
	DUMP(VSYNCL);
	DUMP(VSCLIP);
	DUMP(VSCTL1);
	DUMP(VSCTL2);
	DUMP(VSCTL3);
	if(priv->type == HI253)
	{
		DUMP(HREF1);
		DUMP(HREF2);
		DUMP(VSYNCDELAY1);
		DUMP(VSYNCDELAY2);
	}
	DUMP(BLCCTL);
	DUMP(BLCTIMETHON);
	DUMP(BLCTIMETHOFF);
	DUMP(BLCAGTHH);
	DUMP(BLCAGTHL);
	DUMP(BLCDGH);
	DUMP(BLCDGL);
	DUMP(BLCOFSDB);
	DUMP(BLCOFSDGB);
	DUMP(BLCOFSDR);
	DUMP(BLCOFSDGR);
	DUMP(STRCTL);
	DUMP(STRWID);
	DUMP(STRTIME);

	return 0;
}

static int show_format_page(struct seq_file *s, void *v)
{
	struct hynix_debug *debug = s->private;
	struct hynix_priv *priv = v;
	struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);
	int ret;
	u8 val;

	if(0 != (ret = hynix_read(client, debug->page, ISPCTL1, &val)))
		return ret;

	DUMP(ISPCTL1);
	DUMP(ISPCTL2);
	DUMP(ISPCTL3);
	DUMP(ISPCTL4);
	DUMP(ISPCTL5);
	DUMP(ITUCTL);
	DUMP(ITUSOF);
	DUMP(ITUSOL);
	DUMP(ITUEOF);
	DUMP(ITUEOL);
	DUMP(YOFS);
	DUMP(DYOFS);
	DUMP(UOFS);
	DUMP(VOFS);
	DUMP(UCON);
	DUMP(VCON);
	DUMP(SOLARI);
	DUMP(BINARY);
	DUMP(CONTRAST);
	DUMP(AGBRT);
	DUMP(SATCTL);
	DUMP(SATB);
	DUMP(SATR);
	DUMP(AGSAT);
	DUMP(SATTIMETH);
	DUMP(SATOUTDEL);
	DUMP(UPOSSAT);
	DUMP(UNEGSAT);
	DUMP(VPOSSAT);
	DUMP(VNEGSAT);
	DUMP(VNEGSAT);
	if(priv->type == HI253)
	{
		DUMP(LGRATION);
		DUMP(LGOFS);
	}

	return 0;
}

static int show_autoexp_page(struct seq_file *s, void *v)
{
	struct hynix_debug *debug = s->private;
	struct hynix_priv *priv = v;
	struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);
	int ret;
	u8 val;

	if(0 != (ret = hynix_read(client, debug->page, AECTL1, &val)))
		return ret;

	DUMP(AECTL1);
	DUMP(AECTL2);
	DUMP(AEFRAMECTL1);
	DUMP(AEFINECTL1);
	DUMP(AEFINECTL2);
	DUMP(AEFINECTL3);
	DUMP(AEFINECTL4);
	if(priv->type == HI253)
	{
		DUMP(AEFINECTL5);
		DUMP(AEFINECTL6);
		DUMP(AEFINECTL7);
		DUMP(AEFINECTL8);
	}
	DUMP(AEWGT1);
	DUMP(AEWGT2);
	DUMP(AEWGT3);
	DUMP(AEWGT4);
	DUMP(AEWGT5);
	DUMP(AEWGT6);
	DUMP(AEWGT7);
	DUMP(AEWGT8);
	DUMP(AEWGT9);
	DUMP(AEWGT10);
	DUMP(AEWGT11);
	DUMP(AEWGT12);
	DUMP(AEWGT13);
	DUMP(AEWGT14);
	DUMP(AEWGT15);
	DUMP(AEWGT16);
	DUMP(YLVL);
	DUMP(YTH1);
	DUMP(YTH2HI);
	DUMP(YTH2LOW);
	DUMP(EXPINTH);
	DUMP(EXPINTM);
	DUMP(EXPINTL);
	DUMP(EXPTIMEH);
	DUMP(EXPTIMEM);
	DUMP(EXPTIMEL);
	DUMP(EXPMINH);
	DUMP(EXPMINL);
	DUMP(EXPMAXH);
	DUMP(EXPMAXM);
	DUMP(EXPMAXL);
	DUMP(EXP100H);
	DUMP(EXP100L);
	DUMP(EXP120H);
	DUMP(EXP120L);
	DUMP(EXPFIXH);
	DUMP(EXPFIXM);
	DUMP(EXPFIXL);
	DUMP(EXPOUT1);
	DUMP(EXPOUT2);
	DUMP(EXPLMTH);
	DUMP(EXPLMTL);
	DUMP(EXPUNITH);
	DUMP(EXPUNITL);
	DUMP(AG);
	DUMP(AGMIN);
	DUMP(AGMAX);
	DUMP(AGLVLH);
	DUMP(AGTH1);
	DUMP(AGTH2);
	DUMP(AGBTH1);
	DUMP(AGBTH2);
	DUMP(AGBTH3);
	DUMP(AGBTH4);
	DUMP(AGBTH5);
	DUMP(AGBTH6);
	DUMP(AGBTH7);
	DUMP(AGBTH8);
	if(priv->type == SR300PC10)
	{
		DUMP(AGBTH9);
		DUMP(AGBTH10);
		DUMP(AGBTH11);
		DUMP(AGBTH12);
		DUMP(AGLVLL_SR300PC10);
		DUMP(AGSKY_SR300PC10);
	}
	if(priv->type == HI253)
	{
		DUMP(AGSKY_HI253);
		DUMP(AGLVLL_HI253);
		DUMP(AGTIMETH);
	}
	DUMP(DGMAX);
	DUMP(DGMIN);
	DUMP(YAVG);

	return 0;
}

static struct hynix_debug debug_pages[] = {
	{WINDOW_PAGE, NULL, show_window_page, "WINDOW_PAGE"},
	{FORMAT_PAGE, NULL, show_format_page, "FORMAT_PAGE"},
	{AUTOEXP_PAGE, NULL, show_autoexp_page, "AUTOEXP_PAGE"},
};

static int hynix_regs_show(struct seq_file *s, void *v)
{
	struct hynix_debug	*debug = s->private;
	struct hynix_priv	*priv = debug->priv;

	seq_printf(s, "Page 0x%02X (%9s)\n\n", reg_map[priv->type][debug->page], debug->name);

	return debug->show(s, priv);
}

static int hynix_regs_open(struct inode *inode, struct file *file)
{
	int ret;
	struct hynix_debug *debug = inode->i_private;

	if(0 != (ret = mutex_lock_interruptible(&debugfs_mutex)))
		return ret;

	/* Multiple instance hack.  Protected by debugfs_mutex. */
	/* TODO: get rid of debug->priv and look up priv in hynix_regs_show() */
	debug->priv = file->f_path.dentry->d_parent->d_inode->i_private;

	return single_open(file, hynix_regs_show, debug);
}

static int hynix_regs_release(struct inode *inode, struct file *file)
{
	int ret;

	ret = single_release(inode, file);

	mutex_unlock(&debugfs_mutex);

	return ret;
}

static const struct file_operations hynix_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= hynix_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= hynix_regs_release,
};

static void hynix_init_debugfs(struct hynix_priv *priv)
{
	struct dentry *dev_dir, *reg_dir;
	unsigned int i;

	dev_dir = debugfs_create_dir(priv->subdev.name, NULL);
	if (!dev_dir || IS_ERR(dev_dir)) {
		priv->debug = NULL;
		return;
	}

	/*
	 * Use debugfs_create_file() instead of debugfs_create_dir() so we can
	 * utilize private data pointer for multiple instance hack.
	 */
	reg_dir = debugfs_create_file("registers",
				   S_IFDIR | S_IRWXU | S_IRUGO | S_IXUGO,
				   dev_dir, priv, NULL);

	if (!reg_dir || IS_ERR(reg_dir)) {
		debugfs_remove_recursive(dev_dir);	
		priv->debug = NULL;
		return;
	}

	for(i = 0; i < ARRAY_SIZE(debug_pages); i++)
	{
		debugfs_create_file(debug_pages[i].name, S_IRUSR,
			reg_dir, &debug_pages[i], &hynix_regs_fops);
	}

	priv->debug = dev_dir;
}
