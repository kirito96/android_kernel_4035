
#ifndef __MMC_COMMON_INTER_H__
#define __MMC_COMMON_INTER_H__

#include "msdc_cfg.h"
#include "mmc_types.h"

#if defined(MMC_MSDC_DRV_PRELOADER)
extern u32 mmc_init_device (void);

#if CFG_LEGACY_USB_DOWNLOAD
extern u32 mmc_read_data (u8 * buf, u32 offset);
extern u32 mmc_write_data (u8 * buf, u32 offset);
extern bool mmc_erase_data (u32 offset, u32 offset_limit, u32 size);
extern void mmc_wait_ready (void);
extern u32 mmc_find_safe_block (u32 offset);

extern u32 mmc_chksum_per_file (u32 mmc_offset, u32 img_size);
extern u32 mmc_chksum_body (u32 chksm, char *buf, u32 pktsz);
extern u32 mmc_get_device_id(u8 *id, u32 len,u32 *fw_len);
#endif
#endif

#if defined(MMC_MSDC_DRV_LK)
int mmc_legacy_init(int verbose);
unsigned long mmc_wrap_bread(int dev_num, unsigned long blknr, u32 blkcnt, unsigned long *dst);
unsigned long mmc_wrap_bwrite(int dev_num, unsigned long blknr, u32 blkcnt, unsigned long *src);
#endif

extern int mmc_do_erase(int dev_num,u64 start_addr,u64 len);

#endif /* __MMC_COMMON_INTER_H__ */
