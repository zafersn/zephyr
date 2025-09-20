/******************************************************************************
 * hl78xx_chat.c
 *
 * Centralized translation unit for MODEM_CHAT_* macro-generated objects and
 * chat scripts for the HL78xx driver. This file contains the MODEM_CHAT
 * matches and script definitions and exposes runtime wrapper functions
 * declared in hl78xx_chat.h.
 *
 * Contract:
 *  - Other translation units MUST NOT take addresses of the MODEM_CHAT_*
 *    symbols or use ARRAY_SIZE() on them at file scope. Use the getters
 *    (hl78xx_get_*) and runners (hl78xx_run_*_script[_async]) instead.
 ******************************************************************************/

#include "hl78xx.h"
#include "hl78xx_chat.h"
#include <zephyr/modem/chat.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(hl78xx_dev);

/* Forward declarations of handlers implemented in hl78xx.c (extern linkage) */
void hl78xx_on_cxreg(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_kstatev(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_socknotifydata(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_ktcpnotif(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
/*
 * Chat script and URC match definitions - extracted from hl78xx.c
 */
void hl78xx_on_udprcv(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_kbndcfg(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_csq(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_cesq(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_cfun(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_cops(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_ksup(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_imei(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_cgmm(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_imsi(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_cgmi(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_cgmr(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_iccid(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_ksrep(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_ksrat(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);
void hl78xx_on_kselacq(struct modem_chat *chat, char **argv, uint16_t argc, void *user_data);

MODEM_CHAT_MATCH_DEFINE(hl78xx_ok_match, "OK", "", NULL);
MODEM_CHAT_MATCHES_DEFINE(hl78xx_allow_match, MODEM_CHAT_MATCH("OK", "", NULL),
                          MODEM_CHAT_MATCH("+CME ERROR: ", "", NULL));

MODEM_CHAT_MATCHES_DEFINE(hl78xx_unsol_matches, MODEM_CHAT_MATCH("+CREG: ", ",", hl78xx_on_cxreg),
                          MODEM_CHAT_MATCH("+CEREG: ", ",", hl78xx_on_cxreg),
                          MODEM_CHAT_MATCH("+CGREG: ", ",", hl78xx_on_cxreg),
                          MODEM_CHAT_MATCH("+KSTATEV: ", ",", hl78xx_on_kstatev),
                          MODEM_CHAT_MATCH("+KUDP_DATA: ", ",", hl78xx_on_socknotifydata),
                          MODEM_CHAT_MATCH("+KTCP_DATA: ", ",", hl78xx_on_socknotifydata),
                          MODEM_CHAT_MATCH("+KTCP_NOTIF: ", ",", hl78xx_on_ktcpnotif),
                          MODEM_CHAT_MATCH("+KUDP_RCV: ", ",", hl78xx_on_udprcv),
                          MODEM_CHAT_MATCH("+KBNDCFG: ", ",", hl78xx_on_kbndcfg),
                          MODEM_CHAT_MATCH("+CSQ: ", ",", hl78xx_on_csq),
                          MODEM_CHAT_MATCH("+CESQ: ", ",", hl78xx_on_cesq),
                          MODEM_CHAT_MATCH("+CFUN: ", "", hl78xx_on_cfun),
                          MODEM_CHAT_MATCH("+COPS: ", ",", hl78xx_on_cops));

MODEM_CHAT_MATCHES_DEFINE(hl78xx_abort_matches, MODEM_CHAT_MATCH("+CME ERROR: ", "", NULL));
MODEM_CHAT_MATCH_DEFINE(hl78xx_at_ready_match, "+KSUP: ", "", hl78xx_on_ksup);
MODEM_CHAT_MATCH_DEFINE(hl78xx_imei_match, "", "", hl78xx_on_imei);
MODEM_CHAT_MATCH_DEFINE(hl78xx_cgmm_match, "", "", hl78xx_on_cgmm);
MODEM_CHAT_MATCH_DEFINE(hl78xx_cimi_match, "", "", hl78xx_on_imsi);
MODEM_CHAT_MATCH_DEFINE(hl78xx_cgmi_match, "", "", hl78xx_on_cgmi);
MODEM_CHAT_MATCH_DEFINE(hl78xx_cgmr_match, "", "", hl78xx_on_cgmr);
MODEM_CHAT_MATCH_DEFINE(hl78xx_iccid_match, "+CCID: ", "", hl78xx_on_iccid);
MODEM_CHAT_MATCH_DEFINE(hl78xx_ksrep_match, "+KSREP: ", ",", hl78xx_on_ksrep);
MODEM_CHAT_MATCH_DEFINE(hl78xx_ksrat_match, "+KSRAT: ", "", hl78xx_on_ksrat);
MODEM_CHAT_MATCH_DEFINE(hl78xx_kselacq_match, "+KSELACQ: ", ",", hl78xx_on_kselacq);

/* Chat script matches / definitions */
MODEM_CHAT_SCRIPT_CMDS_DEFINE(hl78xx_periodic_chat_script_cmds,
                              MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG?", hl78xx_ok_match));

MODEM_CHAT_SCRIPT_DEFINE(hl78xx_periodic_chat_script, hl78xx_periodic_chat_script_cmds,
                         hl78xx_abort_matches, hl78xx_chat_callback_handler, 4);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(
    hl78xx_init_chat_script_cmds, MODEM_CHAT_SCRIPT_CMD_RESP("", hl78xx_at_ready_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+KHWIOCFG=3,1,6", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("ATE0", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+CGACT=0", hl78xx_allow_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=4,0", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+KSLEEP=2", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CPSMS=0", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEDRXS=0", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+KPATTERN=\"--EOF--Pattern--\"", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CCID", hl78xx_iccid_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CMEE=1", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGSN", hl78xx_imei_match), MODEM_CHAT_SCRIPT_CMD_RESP("", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMM", hl78xx_cgmm_match), MODEM_CHAT_SCRIPT_CMD_RESP("", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMI", hl78xx_cgmi_match), MODEM_CHAT_SCRIPT_CMD_RESP("", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGMR", hl78xx_cgmr_match), MODEM_CHAT_SCRIPT_CMD_RESP("", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CIMI", hl78xx_cimi_match), MODEM_CHAT_SCRIPT_CMD_RESP("", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+KSTATEV=1", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGEREP=2", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+KSELACQ?", hl78xx_kselacq_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+KSRAT?", hl78xx_ksrat_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+KBNDCFG?", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CGACT?", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CREG=0", hl78xx_ok_match),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+CEREG=5", hl78xx_ok_match));

MODEM_CHAT_SCRIPT_DEFINE(hl78xx_init_chat_script, hl78xx_init_chat_script_cmds, hl78xx_abort_matches,
                         hl78xx_chat_callback_handler, 10);

/* Post-restart script (moved from hl78xx.c) */
MODEM_CHAT_SCRIPT_CMDS_DEFINE(hl78xx_post_restart_chat_script_cmds,
                              MODEM_CHAT_SCRIPT_CMD_RESP("", hl78xx_at_ready_match),
                              MODEM_CHAT_SCRIPT_CMD_RESP("AT+KSRAT?", hl78xx_ksrat_match),
                              MODEM_CHAT_SCRIPT_CMD_RESP("AT+KSTATEV=1", hl78xx_ok_match));

MODEM_CHAT_SCRIPT_DEFINE(hl78xx_post_restart_chat_script, hl78xx_post_restart_chat_script_cmds,
                         hl78xx_abort_matches, hl78xx_chat_callback_handler, 1000);

/* init_fail_script moved from hl78xx.c */
MODEM_CHAT_SCRIPT_CMDS_DEFINE(init_fail_script_cmds,
                              MODEM_CHAT_SCRIPT_CMD_RESP("AT+KSREP?", hl78xx_ksrep_match));

MODEM_CHAT_SCRIPT_DEFINE(init_fail_script, init_fail_script_cmds, hl78xx_abort_matches,
                         hl78xx_chat_callback_handler, 10);

MODEM_CHAT_SCRIPT_CMDS_DEFINE(hl78xx_enable_ksup_urc_cmds,
                              MODEM_CHAT_SCRIPT_CMD_RESP("AT+KSREP=1", hl78xx_ok_match),
                              MODEM_CHAT_SCRIPT_CMD_RESP("AT+KSREP?", hl78xx_ksrep_match));

MODEM_CHAT_SCRIPT_DEFINE(hl78xx_enable_ksup_urc_script, hl78xx_enable_ksup_urc_cmds, hl78xx_abort_matches,
                         hl78xx_chat_callback_handler, 4);

/* power-off script moved from hl78xx.c */
MODEM_CHAT_SCRIPT_CMDS_DEFINE(hl78xx_pwroff_cmds,
                              MODEM_CHAT_SCRIPT_CMD_RESP("AT+CFUN=0", hl78xx_ok_match),
                              MODEM_CHAT_SCRIPT_CMD_RESP("AT+CPWROFF", hl78xx_ok_match));

MODEM_CHAT_SCRIPT_DEFINE(hl78xx_pwroff_script, hl78xx_pwroff_cmds, hl78xx_abort_matches,
                         hl78xx_chat_callback_handler, 4);

/* modem_init_chat is implemented in hl78xx.c so it can construct the
 * modem_chat_config with device-local buffer sizes (argv_size) without
 * relying on ARRAY_SIZE at file scope inside this translation unit.
 */

/* Bridge function - modem_chat callback */
void hl78xx_chat_callback_handler(struct modem_chat *chat,
                                  enum modem_chat_script_result result,
                                  void *user_data)
{
    struct hl78xx_data *data = (struct hl78xx_data *)user_data;

    if (result == MODEM_CHAT_SCRIPT_RESULT_SUCCESS) {
        hl78xx_delegate_event(data, MODEM_HL78XX_EVENT_SCRIPT_SUCCESS);
    } else {
        hl78xx_delegate_event(data, MODEM_HL78XX_EVENT_SCRIPT_FAILED);
    }
}

/* --- Wrapper helpers -------------------------------------------------- */
const struct modem_chat_match *hl78xx_get_ok_match(void)
{
    return &hl78xx_ok_match;
}

const struct modem_chat_match *hl78xx_get_abort_matches(void)
{
    return hl78xx_abort_matches;
}

const struct modem_chat_match *hl78xx_get_unsol_matches(void)
{
    return hl78xx_unsol_matches;
}

size_t hl78xx_get_unsol_matches_size(void)
{
    /* Return size as a runtime value to avoid constant-expression errors
     * in translation units that include this header.
     */
    return (size_t)(sizeof(hl78xx_unsol_matches) / sizeof(hl78xx_unsol_matches[0]));
}

size_t hl78xx_get_abort_matches_size(void)
{
    return (size_t)(sizeof(hl78xx_abort_matches) / sizeof(hl78xx_abort_matches[0]));
}

const struct modem_chat_match *hl78xx_get_allow_match(void)
{
    return hl78xx_allow_match;
}

size_t hl78xx_get_allow_match_size(void)
{
    return (size_t)(sizeof(hl78xx_allow_match) / sizeof(hl78xx_allow_match[0]));
}

/* Run the predefined init script for the given device */
int hl78xx_run_init_script(struct hl78xx_data *data)
{
    if (!data) {
        return -EINVAL;
    }
    return modem_chat_run_script(&data->chat, &hl78xx_init_chat_script);
}

/* Run the periodic script */
int hl78xx_run_periodic_script(struct hl78xx_data *data)
{
    if (!data) {
        return -EINVAL;
    }
    return modem_chat_run_script(&data->chat, &hl78xx_periodic_chat_script);
}

int hl78xx_run_init_script_async(struct hl78xx_data *data)
{
    if (!data) {
        return -EINVAL;
    }
    return modem_chat_run_script_async(&data->chat, &hl78xx_init_chat_script);
}

int hl78xx_run_periodic_script_async(struct hl78xx_data *data)
{
    if (!data) {
        return -EINVAL;
    }
    return modem_chat_run_script_async(&data->chat, &hl78xx_periodic_chat_script);
}

const struct modem_chat_match *hl78xx_get_ksrat_match(void)
{
    return &hl78xx_ksrat_match;
}

int hl78xx_run_post_restart_script(struct hl78xx_data *data)
{
    if (!data) {
        return -EINVAL;
    }
    return modem_chat_run_script(&data->chat, &hl78xx_post_restart_chat_script);
}

int hl78xx_run_post_restart_script_async(struct hl78xx_data *data)
{
    if (!data) {
        return -EINVAL;
    }
    return modem_chat_run_script_async(&data->chat, &hl78xx_post_restart_chat_script);
}

int hl78xx_run_init_fail_script_async(struct hl78xx_data *data)
{
    if (!data) {
        return -EINVAL;
    }
    return modem_chat_run_script_async(&data->chat, &init_fail_script);
}

int hl78xx_run_enable_ksup_urc_script_async(struct hl78xx_data *data)
{
    if (!data) {
        return -EINVAL;
    }
    return modem_chat_run_script_async(&data->chat, &hl78xx_enable_ksup_urc_script);
}

int hl78xx_run_pwroff_script_async(struct hl78xx_data *data)
{
    if (!data) {
        return -EINVAL;
    }
    return modem_chat_run_script_async(&data->chat, &hl78xx_pwroff_script);
}
