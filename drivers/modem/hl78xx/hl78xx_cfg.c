/*
 * hl78xx_cfg.c
 *
 * Extracted helper implementations for RAT, band and APN configuration to
 * keep the main state-machine TU small and maintainable.
 */

#include "hl78xx.h"
#include "hl78xx_cfg.h"
#include "hl78xx_chat.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(hl78xx_dev);

int hl78xx_rat_cfg(struct hl78xx_data *data, bool *modem_require_restart,
		   enum hl78xx_cell_rat_mode *rat_request)
{
	int ret = 0;
	char const *cmd_ksrat_query = (const char *)KSRAT_QUERY;
	char const *cmd_kselq_disable = (const char *)DISABLE_RAT_AUTO;
	const char *cmd_set_rat = NULL;

#if defined(CONFIG_MODEM_HL78XX_AUTORAT)
	/* Check autorat status/configs */
	if (IS_ENABLED(CONFIG_MODEM_HL78XX_AUTORAT_OVER_WRITE_PRL) ||
	    (data->kselacq_data.rat1 == 0 && data->kselacq_data.rat2 == 0 &&
	     data->kselacq_data.rat3 == 0)) {
		char cmd_kselq[] = "AT+KSELACQ=0," CONFIG_MODEM_HL78XX_AUTORAT_PRL_PROFILES;
		ret = modem_dynamic_cmd_send(data, NULL, cmd_kselq, strlen(cmd_kselq),
					     hl78xx_get_ok_match(), 1, false);
		if (ret < 0) {
			goto error;
		} else {
			*modem_require_restart = true;
		}
	}

	*rat_request = HL78XX_RAT_MODE_AUTO;
#else
	if (data->kselacq_data.rat1 != 0 && data->kselacq_data.rat2 != 0 &&
	    data->kselacq_data.rat3 != 0) {
		ret = modem_dynamic_cmd_send(data, NULL, cmd_kselq_disable,
					     strlen(cmd_kselq_disable), hl78xx_get_ok_match(), 1,
					     false);
		if (ret < 0) {
			goto error;
		}
	}

	ret = modem_dynamic_cmd_send(data, NULL, cmd_ksrat_query, strlen(cmd_ksrat_query),
				     hl78xx_get_ksrat_match(), 1, false);
	if (ret < 0) {
		goto error;
	}

#if !defined(CONFIG_MODEM_HL78XX_RAT_M1) && !defined(CONFIG_MODEM_HL78XX_RAT_NB1) &&               \
	!defined(CONFIG_MODEM_HL78XX_RAT_GSM) && !defined(CONFIG_MODEM_HL78XX_RAT_NBNTN)
#error "No rat has been selected."
#endif

	if (IS_ENABLED(CONFIG_MODEM_HL78XX_RAT_M1)) {
		cmd_set_rat = (const char *)SET_RAT_M1_CMD_LEGACY;
		*rat_request = HL78XX_RAT_CAT_M1;
	} else if (IS_ENABLED(CONFIG_MODEM_HL78XX_RAT_NB1)) {
		cmd_set_rat = (const char *)SET_RAT_NB1_CMD_LEGACY;
		*rat_request = HL78XX_RAT_NB1;
	}
#ifdef CONFIG_MODEM_HL7812
	else if (IS_ENABLED(CONFIG_MODEM_HL78XX_RAT_GSM)) {
		cmd_set_rat = (const char *)SET_RAT_GSM_CMD_LEGACY;
		*rat_request = HL78XX_RAT_GSM;
	}
#ifdef CONFIG_MODEM_HL7812_FW_R6
	else if (IS_ENABLED(CONFIG_MODEM_HL78XX_RAT_NBNTN)) {
		cmd_set_rat = (const char *)SET_RAT_NBNTN_CMD_LEGACY;
		*rat_request = HL78XX_RAT_NBNTN;
	}
#endif
#endif

	if (cmd_set_rat == NULL || *rat_request == HL78XX_RAT_MODE_NONE) {
		ret = -EINVAL;
		goto error;
	}

	if (*rat_request != data->status.registration.rat_mode) {
		ret = modem_dynamic_cmd_send(data, NULL, cmd_set_rat, strlen(cmd_set_rat),
					     hl78xx_get_ok_match(), 1, false);
		if (ret < 0) {
			goto error;
		} else {
			*modem_require_restart = true;
		}
	}
#endif

error:
	return ret;
}

int hl78xx_band_cfg(struct hl78xx_data *data, bool *modem_require_restart,
		    enum hl78xx_cell_rat_mode rat_config_request)
{
	int ret = 0;
	char bnd_bitmap[MDM_BAND_HEX_STR_LEN] = {0};
	const char *modem_trimmed;
	const char *expected_trimmed;

	if (rat_config_request == HL78XX_RAT_MODE_NONE) {
		return -EINVAL;
	}
#ifdef CONFIG_MODEM_HL78XX_AUTORAT
	for (int rat = HL78XX_RAT_CAT_M1; rat <= HL78XX_RAT_NB1; rat++) {
		if (rat == HL78XX_RAT_GSM) {
			continue;
		}
#else
	int rat = rat_config_request;
#endif
		ret = hl78xx_get_band_default_config_for_rat(rat, bnd_bitmap,
							     ARRAY_SIZE(bnd_bitmap));
		if (ret) {
			LOG_ERR("%d %s error get band default config %d", __LINE__, __func__, ret);
			goto error;
		}
		modem_trimmed = hl78xx_trim_leading_zeros(data->status.kbndcfg[rat].bnd_bitmap);
		expected_trimmed = hl78xx_trim_leading_zeros(bnd_bitmap);

		if (strcmp(modem_trimmed, expected_trimmed) != 0) {
			char cmd_bnd[80] = {0};

			snprintf(cmd_bnd, sizeof(cmd_bnd), "AT+KBNDCFG=%d,%s", rat, bnd_bitmap);
			ret = modem_dynamic_cmd_send(data, NULL, cmd_bnd, strlen(cmd_bnd),
						     hl78xx_get_ok_match(), 1, false);
			if (ret < 0) {
				goto error;
			} else {
				*modem_require_restart |= true;
			}
		} else {
			LOG_DBG("The band configs (%s) matched with exist configs (%s) for rat: "
				"[%d]",
				modem_trimmed, expected_trimmed, rat);
		}
#ifdef CONFIG_MODEM_HL78XX_AUTORAT
	}
#endif
error:
	return ret;
}

int hl78xx_set_apn_internal(struct hl78xx_data *data, const char *apn, uint16_t size)
{
	int ret = 0;
	char cmd_string[sizeof("AT+KCNXCFG=,\"\",\"\"") + sizeof(uint8_t) +
			MODEM_HL78XX_ADDRESS_FAMILY_FORMAT_LEN + MDM_APN_MAX_LENGTH] = {0};
	int cmd_max_len = sizeof(cmd_string) - 1;
	int apn_size = strlen(apn);

	if (apn == NULL || size >= MDM_APN_MAX_LENGTH) {
		return -EINVAL;
	}

	k_mutex_lock(&data->api_lock, K_FOREVER);
	if (strncmp(data->identity.apn, apn, apn_size) != 0) {
		safe_strncpy(data->identity.apn, apn, sizeof(data->identity.apn));
	}
	k_mutex_unlock(&data->api_lock);

	snprintk(cmd_string, cmd_max_len, "AT+CGDCONT=1,\"%s\",\"%s\"", MODEM_HL78XX_ADDRESS_FAMILY,
		 apn);

	ret = modem_dynamic_cmd_send(data, NULL, cmd_string, strlen(cmd_string),
				     hl78xx_get_ok_match(), 1, false);
	if (ret < 0) {
		goto error;
	}
	snprintk(cmd_string, cmd_max_len,
		 "AT+KCNXCFG=1,\"GPRS\",\"%s\",,,\"" MODEM_HL78XX_ADDRESS_FAMILY "\"", apn);
	ret = modem_dynamic_cmd_send(data, NULL, cmd_string, strlen(cmd_string),
				     hl78xx_get_ok_match(), 1, false);
	if (ret < 0) {
		goto error;
	}
	data->status.apn.state = APN_STATE_CONFIGURED;
	return 0;
error:
	LOG_ERR("Set APN to %s, result: %d", apn, ret);
	return ret;
}
