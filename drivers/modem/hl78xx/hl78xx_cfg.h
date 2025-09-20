/*
 * hl78xx_cfg.h
 *
 * Helper APIs for RAT, band and APN configuration extracted from hl78xx.c
 * to keep the state machine file smaller and easier to read.
 */
#ifndef ZEPHYR_DRIVERS_MODEM_HL78XX_HL78XX_CFG_H_
#define ZEPHYR_DRIVERS_MODEM_HL78XX_HL78XX_CFG_H_

#include <zephyr/types.h>
#include <stdbool.h>
#include "hl78xx.h"

int hl78xx_rat_cfg(struct hl78xx_data *data, bool *modem_require_restart,
                   enum hl78xx_cell_rat_mode *rat_request);

int hl78xx_band_cfg(struct hl78xx_data *data, bool *modem_require_restart,
                    enum hl78xx_cell_rat_mode rat_config_request);

int hl78xx_set_apn_internal(struct hl78xx_data *data, const char *apn, uint16_t size);

#endif /* ZEPHYR_DRIVERS_MODEM_HL78XX_HL78XX_CFG_H_ */
