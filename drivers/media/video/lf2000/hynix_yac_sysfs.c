/*****************************************************************************
 * sysfs entries See Documentation/filesystems/sysfs.txt
 *****************************************************************************/

static int show_hblank(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	int ret;
	u16 val;
	struct soc_camera_device *icd = dev_get_drvdata(dev);
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	char seq[][3] = {
		{WINDOW_PAGE, HBLANKH, },
		{WINDOW_PAGE, HBLANKL, },
	};

	ret = fetch_seq(client, seq, ARRAY_SIZE(seq));
	if(ret)
		goto out;

	val = (seq[0][2] << 8) | (seq[1][2]);

	ret = scnprintf(buf, PAGE_SIZE, "%hu\n", val);
out:
	return ret;
}

static ssize_t sys_set_hblank(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
	int ret;
	u16 val;
	struct soc_camera_device *icd = dev_get_drvdata(dev);
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	char seq[][3] = {
		{WINDOW_PAGE, HBLANKH, },
		{WINDOW_PAGE, HBLANKL, },
	};

	if(sscanf(buf, "%hu\n", &val) != 1)
		return -EINVAL;

	seq[0][2] = val >> 8;
	seq[1][2] = val;

	ret = apply_seq(client, seq, ARRAY_SIZE(seq));
	if(!ret)
		ret = count;

	return ret;
}

static DEVICE_ATTR(hblank, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH, show_hblank, sys_set_hblank);

static struct attribute *hynix_yac_attributes[] = {
	&dev_attr_hblank.attr,
	NULL,
};

static struct attribute_group hynix_attr_group = {
	.attrs = hynix_yac_attributes
};

