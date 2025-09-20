#include <zephyr.h>
#include <ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/modem/hl78xx_apis.h>
#include "../../../../../drivers/modem/hl78xx/hl78xx.h"
#include <string.h>

/* Provide a fake modem_pipe_receive by weak symbol to feed test data */
int modem_pipe_receive(struct modem_pipe *pipe, uint8_t *buf, size_t size)
{
    ARG_UNUSED(pipe);
    static const char test_data[] = "\r\nCONNECT\r\nHelloWorld--EOF--Pattern--\r\nOK\r\n";
    static size_t idx;

    if (idx >= sizeof(test_data) - 1) {
        return 0; /* no more data */
    }
    size_t to_copy = MIN(size, sizeof(test_data) - 1 - idx);
    memcpy(buf, &test_data[idx], to_copy);
    idx += to_copy;
    return to_copy;
}

/* Minimal stubs to satisfy references in the sockets TU */
struct modem_pipe { int dummy; };

/* Minimal fake structures used by modem_process_handler */
static struct hl78xx_data fake_modem_data;
static struct hl78xx_socket_data fake_socket_data;

/* A small helper to initialize the fake objects */
static void setup_fake(void)
{
    memset(&fake_modem_data, 0, sizeof(fake_modem_data));
    memset(&fake_socket_data, 0, sizeof(fake_socket_data));
    /* Link the objects like the driver would */
    fake_socket_data.mdata_global = &fake_modem_data;
    fake_modem_data.uart_pipe = (struct modem_pipe *)&fake_modem_data; /* dummy */
    fake_modem_data.offload_dev = (const struct device *)&fake_modem_data; /* dummy */

    /* Expecting a length >0 so the handler will attempt reads */
    fake_socket_data.expected_buf_len = 64;
    k_sem_init(&fake_modem_data.script_stopped_sem_rx_int, 0, 1);
}

void test_modem_process_handler_receives_full_message(void)
{
    setup_fake();

    /* call the test wrapper which invokes the internal handler */
    extern int hl78xx_parser_test_invoke(struct hl78xx_data *data);
    int ret = hl78xx_parser_test_invoke(&fake_modem_data);
    zassert_equal(ret, 0, "handler returned non-zero");

    /* After processing the synthesized stream the handler should have
     * cleared expected_buf_len when the OK+EOF pattern arrives.
     */
    zassert_equal(fake_socket_data.expected_buf_len, 0, "expected_buf_len not cleared");
}

void test_main(void)
{
    ztest_test_suite(hl78xx_parser_harness,
                     ztest_unit_test(test_modem_process_handler_receives_full_message)
    );
    ztest_run_test_suite(hl78xx_parser_harness);
}
