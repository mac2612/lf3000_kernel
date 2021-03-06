/*
 */
#ifndef __NXP_SPDIF_H__
#define __NXP_SPDIF_H__

#include "nxp-pcm.h"

#define SND_SOC_SPDIF_TX_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)
#define SND_SOC_SPDIF_RX_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

#define SND_SOC_SPDIF_RATES	SNDRV_PCM_RATE_32000 |	\
							SNDRV_PCM_RATE_44100 |	\
							SNDRV_PCM_RATE_48000 |	\
							SNDRV_PCM_RATE_96000 \


#endif /* __NXP_SPDIF_H__ */

