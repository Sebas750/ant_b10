
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/modem/backend/uart.h>

#include <drivers/aoa/ant_b10/ant_b10.h>

#define DT_DRV_COMPAT u_blox_ant_b10
#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "Custom ANT-B10 driver enabled without any devices"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ant_b10);

/*  ##################################################### Callback functions ##################################################### */

/**
 * @brief Callback for handling match responses from the antenna.
 *
 * These functions LOG the recived information and stores it in the ant_b10_data structure.
 *
 * @param chat Pointer to the modem chat instance.
 * @param argv Array of received arguments.
 * @param argc Number of arguments.
 * @param user_data User data pointer (typically the ant_b10_data structure).
 */

static void ant_b10_on_strp(struct modem_chat *chat, char **argv, u_int16_t argc, void *user_data)
{
    LOG_DBG("Received +STARTUP");
}

static void ant_b10_on_ok(struct modem_chat *chat, char **argv, u_int16_t argc, void *user_data)
{
    LOG_DBG("Received OK");
}

static void ant_b10_on_echo(struct modem_chat *chat, char **argv, u_int16_t argc, void *user_data)
{
    LOG_DBG("Received ECHO");
}

static void ant_b10_on_gmi(struct modem_chat *chat, char **argv, u_int16_t argc, void *user_data)
{
    struct ant_b10_data *data = (struct ant_b10_data *)user_data;
    strcpy(data->manufacturer, argv[1]);                                                               // Save manufacturer name to data struct
    LOG_DBG("GMI received %s", data->manufacturer); 
}

static void ant_b10_on_gmm(struct modem_chat *chat, char **argv, u_int16_t argc, void *user_data)
{
    struct ant_b10_data *data = (struct ant_b10_data *)user_data;
    strcpy(data->model, argv[1]);                                                                      // Save model name to data struct
    LOG_DBG("GMM received %s", data->model);
}

static void ant_b10_on_gmr(struct modem_chat *chat, char **argv, u_int16_t argc, void *user_data)
{   
    struct ant_b10_data *data = (struct ant_b10_data *)user_data;
    strcpy(data->firmware_version, argv[1]);                                                          // Save firmware version to data struct
    LOG_DBG("GMR received %s", data->firmware_version);
}

static void ant_b10_on_umla(struct modem_chat *chat, char **argv, u_int16_t argc, void *user_data)
{
    struct ant_b10_data *data = (struct ant_b10_data *)user_data;
    strcpy(data->mac_address, argv[1]);                                                              // Save MAC address to data struct
    LOG_DBG("UMLA received %s", data->mac_address);
}

static void ant_b10_on_umrs(struct modem_chat *chat, char **argv, u_int16_t argc, void *user_data)
{
    struct ant_b10_data *data = (struct ant_b10_data *)user_data;
    int buffer;
    sscanf(argv[1], "%i,%i", &data->baud_rate, &buffer);                                            // Save baud rate and flow control to data struct
    data->flow_control = (bool)buffer;
    LOG_DBG("UMRS received %i and %i", data->baud_rate, data->flow_control);
}

static void ant_b10_on_udfcfg(struct modem_chat *chat, char **argv, u_int16_t argc, void *user_data)
{
    struct ant_b10_data *data = (struct ant_b10_data *)user_data;
    int buffer1, buffer2;
    char buffer3[32];

    int match = sscanf(argv[1], "%i,%i", &buffer1, &buffer2);                                       // Check if the received message has 2 integer arguments
    if (match == 2) {
        if (buffer1 != 2 && buffer1 != 4) {
            data->udfcfg_tags[buffer1 - 1] = buffer2;                                               // Save the received tag to the data udfcfg_tags array
            LOG_DBG("UDFCFG received %i saved to udfcfg_tags[%i]", data->udfcfg_tags[buffer1 - 1], buffer1 - 1);
        } else {
            LOG_DBG("UDFCFG received invalid tag %i", buffer1);
        }
    } else {
        match = sscanf(argv[1], "%i,%s", &buffer1, buffer3);                                        // Check if the received message has 1 integer and 1 string argument
        if (match == 2) {
            if (buffer1 == 2) {
                strcpy(data->udfcfg_tag_2, buffer3);                                                // Save the received tag to the data udfcfg_tag_2 array
                LOG_DBG("UDFCFG received %s", data->udfcfg_tag_2);
            } else if (buffer1 == 4) {
                strcpy(data->udfcfg_tag_4, buffer3);                                                // Save the received tag to the data udfcfg_tag_4 array
                LOG_DBG("UDFCFG received %s", data->udfcfg_tag_4);
            } else {
                LOG_ERR("UDFCFG received invalid tag %i", buffer1);
            }
        } else {
            LOG_ERR("UDFCFG received invalid arguments");
        }
    }
}

static void ant_b10_on_udfscancfg(struct modem_chat *chat, char **argv, u_int16_t argc, void *user_data)
{
    struct ant_b10_data *data = (struct ant_b10_data *)user_data;
    int buffer1, buffer2;
    sscanf(argv[1], "%i,%i", &buffer1, &buffer2);
    data->udfscan_tags[buffer1 - 1] = buffer2;                                                      // Save the received tag to the data udfscan_tags array
    LOG_DBG("UDFSCANCFG received %i saved to udfscan_tags[%i]", data->udfscan_tags[buffer1 - 1], buffer1 - 1);
}

static void ant_b10_on_uudf(struct modem_chat *chat, char **argv, u_int16_t argc, void *user_data)
{
    struct ant_b10_data *data = (struct ant_b10_data *)user_data;
    LOG_DBG("UUDF received %s", argv[1]);

    int temp1, temp2, temp3, temp4; // Temporary variables
    char str1[32], str2[32];        // Temporary strings
    
    // Check if the tag 13 is enabled to know the angle is received
    if (data->udfcfg_tags[13] == 1) { // Direct azimuthal angle
        sscanf(argv[1], "%x,%d,%d,%d,%d,%d,%s,%s,%d,%d", 
            &temp1, &data->rssi, &data->azimuth_angle, &data->elevation_angle,
            &temp2, &temp3, str1, str2, &data->time_stamp_last_scan, &temp4);
    } else { // Direct angle
        sscanf(argv[1], "%x,%d,%d,%d,%d,%d,%s,%s,%d,%d", 
            &temp1, &data->rssi, &data->direct_angle, &data->elevation_angle,
            &temp2, &temp3, str1, str2, &data->time_stamp_last_scan, &temp4);
    }
    LOG_DBG("RSSI: %d, Direct Angle: %d, Azimutal Angle: %d", 
       data->rssi, data->direct_angle, data->azimuth_angle);
}

/*  ##################################################### Chat MATCH ##################################################### */ 

/* ANT_B10 CHAT MATCH DEFINITIONS */
/* These macros define the matching patterns and associated callbacks for various AT commands responses. */

MODEM_CHAT_MATCH_DEFINE(ok_m, "OK", "", ant_b10_on_ok);
MODEM_CHAT_MATCHES_DEFINE(ate0_ms,
    MODEM_CHAT_MATCH("ATE0", "", ant_b10_on_echo),
    MODEM_CHAT_MATCH("OK", "", ant_b10_on_ok));
#if(CONFIG_ANT_B10_ECHO)
MODEM_CHAT_MATCHES_DEFINE(ate1_ms,
    MODEM_CHAT_MATCH("", "", ant_b10_on_echo),
    MODEM_CHAT_MATCH("OK", "", ant_b10_on_ok));
#endif
MODEM_CHAT_MATCHES_DEFINE(gmi_ms,
    MODEM_CHAT_MATCH("", "", ant_b10_on_gmi),
    MODEM_CHAT_MATCH("OK", "", ant_b10_on_ok));
MODEM_CHAT_MATCHES_DEFINE(gmm_ms, 
    MODEM_CHAT_MATCH("", "", ant_b10_on_gmm), 
    MODEM_CHAT_MATCH("OK", "", ant_b10_on_ok));
MODEM_CHAT_MATCHES_DEFINE(gmr_ms,
    MODEM_CHAT_MATCH("", "", ant_b10_on_gmr),
    MODEM_CHAT_MATCH("OK", "", ant_b10_on_ok));
MODEM_CHAT_MATCHES_DEFINE(umla_ms,
    MODEM_CHAT_MATCH("+UMLA:", "", ant_b10_on_umla),
    MODEM_CHAT_MATCH("OK", "", ant_b10_on_ok));
MODEM_CHAT_MATCHES_DEFINE(umrs_ms,
    MODEM_CHAT_MATCH("+UMRS:", "", ant_b10_on_umrs),
    MODEM_CHAT_MATCH("OK", "", ant_b10_on_ok));
MODEM_CHAT_MATCHES_DEFINE(udfcfg_ms,
    MODEM_CHAT_MATCH("+UDFCFG:", "", ant_b10_on_udfcfg),
    MODEM_CHAT_MATCH("OK", "", ant_b10_on_ok));
MODEM_CHAT_MATCHES_DEFINE(udfscancfg_ms,
    MODEM_CHAT_MATCH("+UDFSCANCFG:", "", ant_b10_on_udfscancfg),
    MODEM_CHAT_MATCH("OK", "", ant_b10_on_ok));

/* Unsolved MATCHES */
MODEM_CHAT_MATCHES_DEFINE(unsol_m, 
    MODEM_CHAT_MATCH("+UUDF:", "", ant_b10_on_uudf),  // For UDDF commands
    MODEM_CHAT_MATCH("+STARTUP", "", ant_b10_on_strp));

/* Abort MATCHES */
MODEM_CHAT_MATCHES_DEFINE(abort_m, MODEM_CHAT_MATCH("ERROR", "", NULL));

/*  ##################################################### Chat SCRIPTS ##################################################### */ 

/* ANT_B10 INIT SCRIPT */
/**
 * @brief Initialization script for the ANT-B10 device.
 *
 * This script disables echo, configures scanning, retrieves metadata,
 * sets LED configurations, saves configuration, and powers off the device.
 */
MODEM_CHAT_SCRIPT_CMDS_DEFINE(init_script_cmds,
    /* Disables echo */
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("ATE0", ate0_ms),
    /* Data scanning disable to a correct configuration */
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+UDFCFG=3,0", ok_m), 
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+UDFENABLE=0", ok_m),
    /* Metadata */
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+GMI", gmi_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+GMM", gmm_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+GMR", gmr_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UMLA=1", umla_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UMRS?", umrs_ms),
    /* Led config and turn off */
#if(CONFIG_ANT_B10_ECHO)
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("ATE1", ate1_ms),
#endif
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+USYSCFG=1,0", ok_m),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT&W", ok_m), 
    MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT+CPWROFF", CONFIG_ANT_B10_WAKEUP_TIME_MS),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+ULED=1,0", ok_m),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+ULED=2,0", ok_m),
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+ULED=3,0", ok_m),
    );

/* ANT_B10 START SCAN SCRIPT */
/**
 * @brief Script to start scanning for ANT-B10.
 *
 * This script enables scanning by sending the appropriate AT command.
 */
MODEM_CHAT_SCRIPT_CMDS_DEFINE(start_scan_script_cmds, 
    MODEM_CHAT_SCRIPT_CMD_RESP("AT+UDFENABLE=1", ok_m),
    );

/* ANT_B10 SLEEP SCRIPT */
/**
 * @brief Script to put the ANT-B10 device into sleep mode.
 *
 * This script disables scanning, saves configuration, and powers off the device.
 */
MODEM_CHAT_SCRIPT_CMDS_DEFINE(sleep_script_cmds,
    MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT+UDFENABLE=0", CONFIG_ANT_B10_SLEEP_TIME_MS),
    MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT&W", CONFIG_ANT_B10_SLEEP_TIME_MS), 
    MODEM_CHAT_SCRIPT_CMD_RESP_NONE("AT+UPWRSAVE=3", CONFIG_ANT_B10_SLEEP_TIME_MS),
    );

/* ANT_B10 UDFCFG GET SCRIPT */
/**
 * @brief Script to retrieve the user-defined configuration.
 *
 * This script sends a series of AT commands to get various configuration tags.
 */
MODEM_CHAT_SCRIPT_CMDS_DEFINE(udfcfg_script_cmds,
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=1", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=2", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=3", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=4", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=5", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=6", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=7", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=8", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=9", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=10", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=11", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=12", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=13", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=14", udfcfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFCFG=15", udfcfg_ms),
    );

/* ANT_B10 UDFSCANCFG GET SCRIPT */
/**
 * @brief Script to retrieve the user-defined scan configuration.
 *
 * This script sends a series of AT commands to get various scan configuration tags.
 */
MODEM_CHAT_SCRIPT_CMDS_DEFINE(udfscancfg_script_cmds,
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFSCANCFG=1", udfscancfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFSCANCFG=2", udfscancfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFSCANCFG=3", udfscancfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFSCANCFG=4", udfscancfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFSCANCFG=5", udfscancfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFSCANCFG=6", udfscancfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFSCANCFG=7", udfscancfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFSCANCFG=8", udfscancfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFSCANCFG=9", udfscancfg_ms),
    MODEM_CHAT_SCRIPT_CMD_RESP_MULT("AT+UDFSCANCFG=10", udfscancfg_ms),
    );

MODEM_CHAT_SCRIPT_DEFINE(init_script, init_script_cmds, abort_m, NULL, CONFIG_ANT_B10_SCRIPT_TIMEOUT_S);
MODEM_CHAT_SCRIPT_DEFINE(sleep_script, sleep_script_cmds, abort_m, NULL, CONFIG_ANT_B10_SCRIPT_TIMEOUT_S);
MODEM_CHAT_SCRIPT_DEFINE(start_scan_script, start_scan_script_cmds, abort_m, NULL, CONFIG_ANT_B10_SCRIPT_TIMEOUT_S);
MODEM_CHAT_SCRIPT_DEFINE(udfcfg_script, udfcfg_script_cmds, abort_m, NULL, CONFIG_ANT_B10_SCRIPT_TIMEOUT_S);
MODEM_CHAT_SCRIPT_DEFINE(udfscancfg_script, udfscancfg_script_cmds, abort_m, NULL, CONFIG_ANT_B10_SCRIPT_TIMEOUT_S);

/**
 * @brief Configures the enable GPIO
 *
 * Used to power up the ANT-B10 device.
 *
 * @param aoa_enable_pin Pointer to the enable GPIO device.
 * @return 0 on success, or a negative error code on failure.
 */
static int enable_gpio_setup(const struct gpio_dt_spec *aoa_enable_pin)
{
    if(!gpio_is_ready_dt(aoa_enable_pin)) {                            
        LOG_ERR("ENABLE GPIO device not ready");
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(aoa_enable_pin, GPIO_OUTPUT_LOW);  // Apagado de dispositivo
    if (err) {
        LOG_ERR("Failed to configure ENABLE GPIO");
        return err;
    }

    k_sleep(K_MSEC(CONFIG_ANT_B10_WAKE_GPIO_DOWN_TIME_MS));

    err = gpio_pin_configure_dt(aoa_enable_pin, GPIO_OUTPUT_HIGH);    // Encendido de dispositivo
    if (err) {
        LOG_ERR("Failed to configureENABLE GPIO");
        return err;
    }

    return 0;
}

/**
 * @brief Configures the status GPIO
 *
 * Used to wake up the ANT-B10 device.
 *
 * @param aoa_state_pin Pointer to the status GPIO device.
 * @return 0 on success, or a negative error code on failure.
 */
static int state_gpio_setup(const struct gpio_dt_spec *aoa_state_pin)
{
    if(!gpio_is_ready_dt(aoa_state_pin)) {                            
        LOG_ERR("STATE GPIO device not ready");
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(aoa_state_pin, GPIO_OUTPUT_HIGH);
    if (err) {
        LOG_ERR("Failed to configure STATE GPIO");
        return err;
    }

    return 0;
}

/**
 * @brief Runs a modem chat script.
 *
 * Opens the UART pipe, attaches the chat, runs the provided script, then releases the chat and closes the pipe.
 *
 * @param data Pointer to the ant_b10_data structure.
 * @param script Pointer to the modem chat script to run.
 * @return 0 on success, or a negative error code on failure.
 */
static int run_script(ant_b10_data_t *data, const struct modem_chat_script *script) {

    int err = modem_pipe_open(data->uart_pipe);
    if (err) {
        LOG_ERR("Failed to open UART pipe: %d", err);
        return err;
    }

    err = modem_chat_attach(&data->chat, data->uart_pipe);
    if (err) {
        LOG_ERR("Failed to attach chat: %d", err);
        return err;
    }
	err = modem_chat_run_script(&data->chat, script);
    if (err) {
        LOG_ERR("Failed to run script: %d", err);
        return err;
    }

    modem_chat_release(&data->chat);
    modem_pipe_close(data->uart_pipe);

    LOG_DBG("script ran");

    return 0;
}

/**
 * @brief Sends a formatted AT command to the ANT-B10 device.
 *
 * Constructs a dynamic script with the provided command format and runs it.
 *
 * @param dev Pointer to the device structure.
 * @param cmd_format Format string for the command.
 * @param ... Variable arguments for the format string.
 * @return 0 on success, or a negative error code on failure.
 */
int ant_b10_command(const struct device *dev, const char *cmd_format, ...)
{
    ant_b10_data_t *data = dev->data;
    va_list args;
    va_start(args, cmd_format);

    int ret;

    ret = vsnprintk(data->dynamic_request_buf, sizeof(data->dynamic_request_buf), cmd_format, args);
    if (ret < 0) {
        va_end(args);
        return -ENOMEM;
    }

    data->dynamic_script_chat.request_size = ret;
    LOG_DBG("Request: %s", data->dynamic_request_buf);

    ret = snprintk(data->dynamic_match_buf, sizeof(data->dynamic_match_buf), "OK"); // TODO: Implement custom matches
    if (ret < 0) {
        va_end(args);
        return -ENOMEM;
    }

    data->dynamic_match.match_size = ret;
    LOG_DBG("Request match: %s", data->dynamic_match_buf);

    va_end(args);

    ret = run_script(data, &data->dynamic_script);
    if (ret) {
        LOG_ERR("Failed to run dynamic script: %d", ret);
        return ret;
    }
    LOG_DBG("Dynamic script ran");

    return 0;
}

/**
 * @brief Sets a user-defined configuration tag.
 *
 * Configures a specified UDFCFG tag with the provided value. If save is true,
 * the configuration is saved and the antenna is reset.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @param tag The UDFCFG tag to set.
 * @param value Pointer to the value (string for tags 2 and 4, integer otherwise).
 * @param save If true, saves the configuration and resets the antenna.
 * @return 0 on success, or a negative error code on failure.
 * 
 * If you use several of this commands to configure various tags in one instance, you should use save=true 
 * in the last one to avoid reseting the antenna each time.
 * 
 * Example 
 *   
 * ant_b10_udfcfg_set(dev, 1, 1, false);
 * ant_b10_udfcfg_set(dev, 2, "hello", false);
 * ant_b10_udfcfg_set(dev, 3, 1, false);
 * ant_b10_udfcfg_set(dev, 4, "bye", false);
 * ant_b10_udfcfg_set(dev, 5, 1, true);  <-- Save the configuration and reset the antenna
 */
int ant_b10_udfcfg_set(const struct device *ant_b10_dev, ant_b10_udfcfg_tags_e tag, void *value, bool save)
{   
    int ret;

    if (tag == 2 || tag == 4) {
        ret = ant_b10_command(ant_b10_dev, "AT+UDFCFG=%i,\"%s\"", (int)tag, (char *)value); // String format for tag 2 and 4
    } else {
        ret = ant_b10_command(ant_b10_dev, "AT+UDFCFG=%i,%i", (int)tag, *(int *)value); // Integer format for the rest of the tags
    }
    if (save) {
        ret = ant_b10_command(ant_b10_dev, "AT&W");
        ret = ant_b10_command(ant_b10_dev, "AT+CPWROFF");
        k_sleep(K_MSEC(CONFIG_ANT_B10_WAKEUP_TIME_MS));
    }
    return ret;
}

/**
 * @brief Sets a user-defined scan configuration tag.
 *
 * Configures a specified UDFSCANCFG tag with the provided integer value.
 * If save is true, the configuration is saved and the antenna is reset.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @param tag The UDFSCANCFG tag to set.
 * @param value The integer value to set for the tag.
 * @param save If true, saves the configuration and resets the antenna.
 * @return 0 on success, or a negative error code on failure.
 * 
 * If you use several of this commands to configure various tags in one instance, you should use save=true 
 * in the last one to avoid reseting the antenna each time. 
 *
 * Example 
 *  
 * ant_b10_udfscancfg_set(dev, 1, 1, false);
 * ant_b10_udfscancfg_set(dev, 2, 2, false);
 * ant_b10_udfscancfg_set(dev, 3, 3, false);
 * ant_b10_udfscancfg_set(dev, 4, 4, false);
 * ant_b10_udfscancfg_set(dev, 5, 5, true);  <-- Save the configuration and reset the antenna
 */
int ant_b10_udfscancfg_set(const struct device *ant_b10_dev, ant_b10_udfscan_tags_e tag, int value, bool save)
{   
    int err = ant_b10_command(ant_b10_dev, "AT+UDFSCANCFG=%i,%i", (int)tag, value);
    if (save) {
        err = ant_b10_command(ant_b10_dev, "AT&W");
        err = ant_b10_command(ant_b10_dev, "AT+CPWROFF");
        k_sleep(K_MSEC(CONFIG_ANT_B10_WAKEUP_TIME_MS));
    }
    return err;
}

/**
 * @brief Performs the initial configuration of the ANT-B10 device.
 *
 * Sets up the initial user-defined configuration and scan configuration tags as defined in Kconfig.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int ant_b10_initial_config(const struct device *dev)
{   
    int buffer; 
    int ret = 0;
#if(CONFIG_ANT_B10_UUDFCFG)
    buffer = CONFIG_ANT_B10_UUDFCFG_MINIMUN_TAG_EVENT_TIME_MS;
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_1, &buffer, false);
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_2, CONFIG_ANT_B10_UUDFCFG_USER_DEFINED_STRING, false);
    buffer = CONFIG_ANT_B10_UUDCFG_ANGLE_CALCULATION_STARTUP;
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_3, &buffer, false);
    buffer = CONFIG_ANT_B10_UUDCFG_ANGLE_CALCULATION_BOTH;
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_5, &buffer, false);
    buffer = CONFIG_ANT_B10_UUDCFG_MEASUREMENT_FORMAT;
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_7, &buffer, false);
    buffer = CONFIG_ANT_B10_UUDCFG_ANGLE_POSTPROCESSING;
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_8, &buffer, false);
    buffer = CONFIG_ANT_B10_UUDCFG_EVENT_TIME_MS;
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_9, &buffer, false);
    buffer = CONFIG_ANT_B10_UUDCFG_ACCEPTED_LAG_FOR_ANGLE;
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_10, &buffer, false);
    buffer = CONFIG_ANT_B10_UUDCFG_FIXED_SMOOTHING_FACTOR; 
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_11, &buffer, false);
    buffer = CONFIG_ANT_B10_UUDCFG_STOP_SCANNING_DEFINED_BY_FILTER;
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_12, &buffer, false);
    buffer = CONFIG_ANT_B10_UUDCFG_ANGLE_TYPE;
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_13, &buffer, false);
    buffer = CONFIG_ANT_B10_UUDCFG_OUTPUT_ENABLED;
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_14, &buffer, false);
    buffer = CONFIG_ANT_B10_UUDCFG_USE_RFU_FIELD_FOR_ADVERTISING;
    ret = ant_b10_udfcfg_set(dev, ANT_B10_UDFCFG_TAG_15, &buffer, false);
#endif

#if(CONFIG_ANT_B10_UDFSCANCFG)
    buffer = CONFIG_ANT_B10_UDFSCANCFG_TAG_DROPPED_TIME_MS;
    ret = ant_b10_udfscancfg_set(dev, ANT_B10_UDFSCANCFG_TAG_1, buffer, false);
    buffer = CONFIG_ANT_B10_UDFSCANCFG_TAG_EVENTS_BEFORE_DROP;
    ret = ant_b10_udfscancfg_set(dev, ANT_B10_UDFSCANCFG_TAG_2, buffer, false);
    buffer = CONFIG_ANT_B10_UDFSCANCFG_TAG_FORGET_TIME_MS;
    ret = ant_b10_udfscancfg_set(dev, ANT_B10_UDFSCANCFG_TAG_3, buffer, false);
    buffer = CONFIG_ANT_B10_UDFSCANCFG_TAG_SYNC_ATTEMPTS;
    ret = ant_b10_udfscancfg_set(dev, ANT_B10_UDFSCANCFG_TAG_4, buffer, false);
    buffer = CONFIG_ANT_B10_UDFSCANCFG_TAG_SYNC_TIME_MS;
    ret = ant_b10_udfscancfg_set(dev, ANT_B10_UDFSCANCFG_TAG_5, buffer, false);
    buffer = CONFIG_ANT_B10_UDFSCANCFG_SIMULTANEOUS_TAGS;
    ret = ant_b10_udfscancfg_set(dev, ANT_B10_UDFSCANCFG_TAG_6, buffer, false);
    buffer = CONFIG_ANT_B10_UDFSCANCFG_SIMULTANEOUS_SYNC_TAGS;
    ret = ant_b10_udfscancfg_set(dev, ANT_B10_UDFSCANCFG_TAG_7, buffer, false);
    buffer = CONFIG_ANT_B10_UDFSCANCFG_SCAN_INTERVAL_MS;
    ret = ant_b10_udfscancfg_set(dev, ANT_B10_UDFSCANCFG_TAG_8, buffer, false);
    buffer = CONFIG_ANT_B10_UDFSCANCFG_SCAN_WINDOW_MS;
    ret = ant_b10_udfscancfg_set(dev, ANT_B10_UDFSCANCFG_TAG_9, buffer, false);
    buffer = CONFIG_ANT_B10_UDFSCANCFG_RSSI_THRESHOLD_DBM;
    ret = ant_b10_udfscancfg_set(dev, ANT_B10_UDFSCANCFG_TAG_10, buffer, true);
#endif
    return ret;
}

/**
 * @brief Sets the state of an LED on the ANT-B10 device.
 *
 * Sends an AT command to configure the specified LED with the given state.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @param led The LED identifier.
 * @param state The state to set for the LED.
 * @return 0 on success, or a negative error code on failure.
 */
int ant_b10_led_set(const struct device *ant_b10_dev, ant_b10_led_e led, ant_b10_led_state_e state)
{   
    int ret = ant_b10_command(ant_b10_dev, "AT+ULED=%d,%d", led, state); // Set LED state
    return ret;
}

/**
 * @brief Wakes up the ANT-B10 device.
 *
 * Toggles the status GPIO to wake up the device and waits for the device to be ready.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return 0 on success, or a negative error code on failure.
 */
int ant_b10_wake(const struct device *ant_b10_dev)
{
    LOG_INF("ANT-B10 waking up");
    const ant_b10_config_t *config = ant_b10_dev->config;
    int err = gpio_pin_configure_dt(&config->status_gpio, GPIO_OUTPUT_LOW);  // wake up the device
    if (err) {
        return err;
    }
    k_sleep(K_MSEC(CONFIG_ANT_B10_WAKE_GPIO_DOWN_TIME_MS));
    err = gpio_pin_configure_dt(&config->status_gpio, GPIO_OUTPUT_HIGH);     // Return pin high for next wake up instance
    if (err) {
        return err;
    }
    k_sleep(K_MSEC(CONFIG_ANT_B10_WAKEUP_TIME_MS));                          // Time for the device to wake up
    return 0;
}

/**
 * @brief Starts a scan operation on the ANT-B10 device.
 *
 * Opens the UART pipe, attaches the modem chat, and runs the start scan script.
 * The pipe and chat remain open for receiving unsolicited messages.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return 0 on success, or a negative error code on failure.
 */
int ant_b10_start_scan(const struct device *ant_b10_dev)
{   
    ant_b10_data_t *data = ant_b10_dev->data;
    const ant_b10_config_t *config = ant_b10_dev->config;
    LOG_INF("ANT-B10 starting scan");
    int err = modem_pipe_open(data->uart_pipe);
    if (err) {
        LOG_ERR("Failed to open UART pipe: %d", err);
        return err;
    }
    err = modem_chat_attach(&data->chat, data->uart_pipe);
    if (err) {
        LOG_ERR("Failed to attach chat: %d", err);
        return err;
    }
	err = modem_chat_run_script(&data->chat, config->start_scan_script);
    if (err) {
        LOG_ERR("Failed to run script: %d", err);
        return err;
    }

    // Keep chat and pipe open for unsolicited messages

    // modem_chat_release(&data->chat);
    // modem_pipe_close(data->uart_pipe);

    return 0;
}

/**
 * @brief Puts the ANT-B10 device into sleep mode.
 *
 * Runs the sleep script which disables scanning and resets the device.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return 0 on success, or a negative error code on failure.
 */
int ant_b10_sleep(const struct device *ant_b10_dev)
{   
    LOG_INF("ANT-B10 going to sleep");
    const ant_b10_config_t *config = ant_b10_dev->config;
    int err = run_script(ant_b10_dev->data, config->sleep_script); // This closes the chat and pipe
    return err;
}

/**
 * @brief Retrieves the current configuration from the ANT-B10 device.
 *
 * Runs scripts to get both UDFCFG and UDFSCANCFG configuration.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return 0 on success, or a negative error code on failure.
 */
int ant_b10_config_get(const struct device *ant_b10_dev)
{   
    const ant_b10_config_t *config = ant_b10_dev->config;

    LOG_DBG("ANT-B10 getting configuration");
    int err = run_script(ant_b10_dev->data, config->udfcfg_script);
    err = run_script(ant_b10_dev->data, config->udfscancfg_script);
    return err;
}

/**
 * @brief Gets the azimuth angle from the last scan.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return The azimuth angle.
 */
int ant_b10_azimutal_angle_get(const struct device *ant_b10_dev)
{   
    ant_b10_data_t *data = ant_b10_dev->data;
    return data->azimuth_angle;
}

/**
 * @brief Gets the elevation angle from the last scan.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return The elevation angle.
 */
int ant_b10_elevation_angle_get(const struct device *ant_b10_dev)
{
    ant_b10_data_t *data = ant_b10_dev->data;
    return data->elevation_angle;
}

/**
 * @brief Gets the direct angle from the last scan.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return The direct angle.
 */
int ant_b10_direct_angle_get(const struct device *ant_b10_dev)
{
    ant_b10_data_t *data = ant_b10_dev->data;
    return data->direct_angle;
}

/**
 * @brief Gets the RSSI from the last scan.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return The received signal strength indicator (RSSI).
 */
int ant_b10_rssi_get(const struct device *ant_b10_dev)
{
    ant_b10_data_t *data = ant_b10_dev->data;
    return data->rssi;
}

/**
 * @brief Performs a factory reset on the ANT-B10 device.
 *
 * Sends commands to reset the device to factory settings.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return 0 on success, or a negative error code on failure.
 */
int ant_b10_factory_reset(const struct device *ant_b10_dev)
{
    LOG_INF("ANT-B10 factory reset");
    int err = ant_b10_command(ant_b10_dev, "AT+UFACTORY");
    err = ant_b10_command(ant_b10_dev, "AT&W");
    err = ant_b10_command(ant_b10_dev, "AT+CPWROFF");
    k_sleep(K_MSEC(CONFIG_ANT_B10_WAKEUP_TIME_MS));
    return err;
}

/**
 * @brief Initializes the ANT-B10 device.
 *
 * Sets up GPIOs, UART, and modem chat configurations. It then runs the initialization
 * script, performs the initial configuration, and puts the device to sleep.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, or a negative error code on failure.
 */
static int ant_b10_init(const struct device *dev)
{   
    ant_b10_data_t *data = (ant_b10_data_t *)dev->data;
    const ant_b10_config_t *config = (const ant_b10_config_t *)dev->config;
    data->dev = dev;

    int err = enable_gpio_setup(&config->enable_gpio);               // Power up
    err = state_gpio_setup(&config->status_gpio);                    // Status pin
    
    /* Dynamic script config (for more examples see zephyr/drivers/gnss/gnss_quectel_lcx6g.c) */
	data->dynamic_match.match = data->dynamic_match_buf;
	data->dynamic_match.separators = data->dynamic_separators_buf;
	data->dynamic_match.separators_size = sizeof(data->dynamic_separators_buf);
	data->dynamic_match.wildcards = false;
	data->dynamic_match.partial = false;

	data->dynamic_script_chat.request = data->dynamic_request_buf;
	data->dynamic_script_chat.response_matches = &data->dynamic_match;
	data->dynamic_script_chat.response_matches_size = 1;
	data->dynamic_script_chat.timeout = 0;

	data->dynamic_script.name = "dynamic";
	data->dynamic_script.script_chats = &data->dynamic_script_chat;
	data->dynamic_script.script_chats_size = 1;
	data->dynamic_script.abort_matches = NULL;
	data->dynamic_script.abort_matches_size = 0;
	data->dynamic_script.callback = NULL;
	data->dynamic_script.timeout = CONFIG_ANT_B10_SCRIPT_TIMEOUT_S;

    /* UART config */
	{
		const struct modem_backend_uart_config uart_backend_config = {
			.uart = config->uart,
			.receive_buf = data->uart_backend_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->uart_backend_receive_buf),
			.transmit_buf = data->uart_backend_transmit_buf,
			.transmit_buf_size = ARRAY_SIZE(data->uart_backend_transmit_buf),
		};

		data->uart_pipe = modem_backend_uart_init(&data->uart_backend,
							  &uart_backend_config);
	}

    /* MODEM CHAT config */
	{
		const struct modem_chat_config chat_config = {
			.user_data = data,
			.receive_buf = data->chat_receive_buf,
			.receive_buf_size = ARRAY_SIZE(data->chat_receive_buf),
			.delimiter = data->chat_delimiter,
			.delimiter_size = strlen(data->chat_delimiter),
			.filter = data->chat_filter,
			.filter_size = data->chat_filter ? strlen(data->chat_filter) : 0,
			.argv = data->chat_argv,
			.argv_size = ARRAY_SIZE(data->chat_argv),
			.unsol_matches = unsol_m,
			.unsol_matches_size = ARRAY_SIZE(unsol_m),
		};

		modem_chat_init(&data->chat, &chat_config);
	}

    LOG_INF("ANT-B10 initialized!");
    k_sleep(K_MSEC(CONFIG_ANT_B10_WAKEUP_TIME_MS)); // Time for the device to wake up
    run_script(data, config->init_script);          // Init script
    ant_b10_initial_config(dev);                    // Initial configuration from Kconfig
    ant_b10_sleep(dev);                             // Sleep script
    
    ant_b10_wake(dev);                              // This is reduntant, but necesary to avoid the first scan to fail
    ant_b10_sleep(dev);                             // Sleep after the first scan
    LOG_INF("ANT-B10 configuration done!");
    return 0;
}

#define AOA_DEVICE_ANT_B10(inst)                                            \
static ant_b10_data_t ant_b10_data_##inst = {                               \
        .chat_delimiter = "\r",                                             \
        .chat_filter = "\n",                                                \
    };                                                                      \
                                                                            \
    static const ant_b10_config_t ant_b10_config_##inst = {                 \
        .uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                           \
        .enable_gpio = GPIO_DT_SPEC_INST_GET(inst, enable_gpios),           \
        .status_gpio = GPIO_DT_SPEC_INST_GET(inst, status_gpios),           \
        .init_script = &init_script,                                        \
        .start_scan_script = &start_scan_script,                            \
        .sleep_script = &sleep_script,                                      \
        .udfcfg_script = &udfcfg_script,                                    \
        .udfscancfg_script = &udfscancfg_script,                            \
    };                                                                      \
                                                                            \
    DEVICE_DT_INST_DEFINE(inst,                                             \
        ant_b10_init,                                                       \
        NULL,                                                               \
        &ant_b10_data_##inst,                                               \
        &ant_b10_config_##inst,                                             \
        POST_KERNEL, 99,                                                    \
        NULL);                                                              \
    
DT_INST_FOREACH_STATUS_OKAY(AOA_DEVICE_ANT_B10)
