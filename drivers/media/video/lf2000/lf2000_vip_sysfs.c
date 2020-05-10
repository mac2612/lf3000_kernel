/****************************************************************************
 * sysfs entries See Documentation/filesystems/sysfs.txt
 ****************************************************************************/

static ssize_t show_frames(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	unsigned int temp;
	unsigned long flags;
	struct soc_camera_host *ici = (struct soc_camera_host *)dev_get_drvdata(dev);
	struct lf2000_vip_dev	*pcdev = ici->priv;

	read_lock_irqsave(&pcdev->sysfs_lock, flags);
	temp = pcdev->frame_count;
	read_unlock_irqrestore(&pcdev->sysfs_lock, flags);

	return scnprintf(buf, PAGE_SIZE, "%u\n", temp);
}

static ssize_t set_frames(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
	unsigned int temp;
	unsigned long flags;
	struct soc_camera_host *ici = (struct soc_camera_host *)dev_get_drvdata(dev);
	struct lf2000_vip_dev	*pcdev = ici->priv;

	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;

	write_lock_irqsave(&pcdev->sysfs_lock, flags);
	pcdev->frame_count = temp;
	write_unlock_irqrestore(&pcdev->sysfs_lock, flags);

	return count;
}

static DEVICE_ATTR(frame_count, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH, show_frames, set_frames);

static ssize_t show_skip(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	unsigned int temp;
	struct soc_camera_host *ici = (struct soc_camera_host *)dev_get_drvdata(dev);
	struct lf2000_vip_dev	*pcdev = ici->priv;

	down_read(&pcdev->skip_mutex);
	temp = pcdev->skip_frames;
	up_read(&pcdev->skip_mutex);

	return scnprintf(buf, PAGE_SIZE, "%u\n", temp);
}

static ssize_t set_skip(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
	unsigned int temp;
	struct soc_camera_host *ici = (struct soc_camera_host *)dev_get_drvdata(dev);
	struct lf2000_vip_dev	*pcdev = ici->priv;

	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;

	if(!down_write_trylock(&pcdev->skip_mutex))
		return -EBUSY;
	pcdev->skip_frames = temp;
	up_write(&pcdev->skip_mutex);

	return count;
}

static DEVICE_ATTR(skip_frames, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH, show_skip, set_skip);

/* Monster macro for the 6 sync fields */
#define SYNC_SYSFS(field) \
static ssize_t show_##field(struct device *dev, struct device_attribute *attr,\
                        char *buf)\
{\
	unsigned short temp;\
	struct soc_camera_host *ici = (struct soc_camera_host *)dev_get_drvdata(dev);\
	struct lf2000_vip_dev	*pcdev = ici->priv;\
\
	temp = pcdev->pdata->field;\
\
	return scnprintf(buf, PAGE_SIZE, "%u\n", temp);\
}\
\
static ssize_t set_##field(struct device *dev, struct device_attribute *attr,\
                        const char *buf, size_t count)\
{\
	unsigned short temp;\
	unsigned long flags;\
	struct soc_camera_host *ici = (struct soc_camera_host *)dev_get_drvdata(dev);\
	struct lf2000_vip_dev	*pcdev = ici->priv;\
\
	if(sscanf(buf, "%hu", &temp) != 1)\
		return -EINVAL;\
\
	pcdev->pdata->field = temp;\
\
	spin_lock_irqsave(&pcdev->lock, flags);\
	update_sync(pcdev, pcdev->pdata->flags & LF2000_VIP_ITU656);\
	spin_unlock_irqrestore(&pcdev->lock, flags);\
\
	return count;\
}\
\
static DEVICE_ATTR(field, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR, show_##field, set_##field);

SYNC_SYSFS(hfp)
SYNC_SYSFS(hsw)
SYNC_SYSFS(hbp)
SYNC_SYSFS(vfp)
SYNC_SYSFS(vsw)
SYNC_SYSFS(vbp)

static ssize_t show_flip(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	unsigned int temp;
	struct soc_camera_host *ici = (struct soc_camera_host *)dev_get_drvdata(dev);
	struct lf2000_vip_dev	*pcdev = ici->priv;

	temp = pcdev->flip_mask;

	return scnprintf(buf, PAGE_SIZE, "%u\n", temp);
}

static ssize_t set_flip(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
	unsigned int temp;
	unsigned long flags;
	struct soc_camera_host *ici = (struct soc_camera_host *)dev_get_drvdata(dev);
	struct lf2000_vip_dev	*pcdev = ici->priv;

	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	pcdev->flip_mask = temp;

	spin_lock_irqsave(&pcdev->lock, flags);
	set_flip_rotate(pcdev, (pcdev->flip_mask) ? 0 : pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE], LF2000_VIP_PIPE_CAPTURE);
	set_flip_rotate(pcdev, (pcdev->flip_mask) ? 0 : pcdev->flip_flags[LF2000_VIP_PIPE_OVERLAY], LF2000_VIP_PIPE_OVERLAY);
	spin_unlock_irqrestore(&pcdev->lock, flags);

	return count;
}

static DEVICE_ATTR(flip_mask, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH, show_flip, set_flip);

static struct attribute *vip_attributes[] = {
	&dev_attr_frame_count.attr,
	&dev_attr_skip_frames.attr,
	&dev_attr_hfp.attr,
	&dev_attr_hsw.attr,
	&dev_attr_hbp.attr,
	&dev_attr_vfp.attr,
	&dev_attr_vsw.attr,
	&dev_attr_vbp.attr,
	&dev_attr_flip_mask.attr,
	NULL,
};

static struct attribute_group vip_attr_group = {
	.attrs = vip_attributes
};
