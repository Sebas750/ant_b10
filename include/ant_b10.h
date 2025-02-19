#include <zephyr/modem/chat.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/modem/backend/uart.h>

#include <stdarg.h>
#include <sys/types.h>

#ifndef __ANT_B10_H__
#define __ANT_B10_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enum for the different LED colors on the ANT-B10 device
 */
typedef enum ant_b10_led {
	ANT_B10_RED_LED = 1,
	ANT_B10_GREEN_LED,
	ANT_B10_BLUE_LED
} ant_b10_led_e;

/**
 * @brief Enum for the state of the LEDs on the ANT-B10 device
 */
typedef enum ant_b10_led_state {
	ANT_B10_LED_OFF = 0,
	ANT_B10_LED_ON,
} ant_b10_led_state_e;

/**
 * @brief Enum for the user-defined configuration tags on the ANT-B10 device
 */
typedef enum ant_b10_udfcfg_tags {
	ANT_B10_UDFCFG_TAG_1 = 1,
	ANT_B10_UDFCFG_TAG_2,
	ANT_B10_UDFCFG_TAG_3,
	ANT_B10_UDFCFG_TAG_4,
	ANT_B10_UDFCFG_TAG_5,
	ANT_B10_UDFCFG_TAG_6,
	ANT_B10_UDFCFG_TAG_7,
	ANT_B10_UDFCFG_TAG_8,
	ANT_B10_UDFCFG_TAG_9,
	ANT_B10_UDFCFG_TAG_10,
	ANT_B10_UDFCFG_TAG_11,
	ANT_B10_UDFCFG_TAG_12,
	ANT_B10_UDFCFG_TAG_13,
	ANT_B10_UDFCFG_TAG_14,
	ANT_B10_UDFCFG_TAG_15
} ant_b10_udfcfg_tags_e;

/**
 * @brief Enum for the user-defined scan configuration tags on the ANT-B10 device
 */
typedef enum ant_b10_udfscan_tags {
	ANT_B10_UDFSCANCFG_TAG_1 = 1,
	ANT_B10_UDFSCANCFG_TAG_2,
	ANT_B10_UDFSCANCFG_TAG_3,
	ANT_B10_UDFSCANCFG_TAG_4,
	ANT_B10_UDFSCANCFG_TAG_5,
	ANT_B10_UDFSCANCFG_TAG_6,
	ANT_B10_UDFSCANCFG_TAG_7,
	ANT_B10_UDFSCANCFG_TAG_8,
	ANT_B10_UDFSCANCFG_TAG_9,
	ANT_B10_UDFSCANCFG_TAG_10
} ant_b10_udfscan_tags_e;

/**
 * @brief Structure representing the data and state of the ANT-B10 device
 */
typedef struct ant_b10_data {
	/* UART backend */
	struct modem_pipe *uart_pipe;
	struct modem_backend_uart uart_backend;
	uint8_t uart_backend_receive_buf[CONFIG_ANT_B10_UART_BUFFER_SIZES];
	uint8_t uart_backend_transmit_buf[CONFIG_ANT_B10_UART_BUFFER_SIZES];
  
	/* Modem chat */
	struct modem_chat chat;
	uint8_t chat_receive_buf[CONFIG_ANT_B10_CHAT_BUFFER_SIZES];
	uint8_t *chat_delimiter;
	uint8_t *chat_filter;
	uint8_t *chat_argv[32];

	/* Metadata */
	char manufacturer[CONFIG_ANT_B10_METADATA_SIZE];
	char model[CONFIG_ANT_B10_METADATA_SIZE];
	char firmware_version[CONFIG_ANT_B10_METADATA_SIZE];
	char mac_address[CONFIG_ANT_B10_METADATA_SIZE];
	int baud_rate;
	bool flow_control;

	/* Actual UDFCFG and ADFSCANCFG config */
	int udfcfg_tags[15]; /* Don't use 2 and 4 */
	char udfcfg_tag_2[CONFIG_ANT_B10_METADATA_SIZE];
	char udfcfg_tag_4[CONFIG_ANT_B10_METADATA_SIZE];
	int udfscan_tags[10];

	/* Read data */
	int rssi;
	int direct_angle;
	int azimuth_angle;
	int elevation_angle;
	int time_stamp_last_scan;

	/* Dynamic chat script */
	uint8_t dynamic_match_buf[32];
	uint8_t dynamic_separators_buf[2];
	uint8_t dynamic_request_buf[32];
	struct modem_chat_match dynamic_match;
	struct modem_chat_script_chat dynamic_script_chat;
	struct modem_chat_script dynamic_script;

	/* Device */
	const struct device *dev;

} ant_b10_data_t;

/**
 * @brief Structure for configuring the ANT-B10 device
 */
typedef struct ant_b10_config {
    /* UART device */
	const struct device *uart;

	/* GPIO device */
	const struct gpio_dt_spec enable_gpio;
	const struct gpio_dt_spec status_gpio;

	/* Static scripts */
	const struct modem_chat_script *init_script;
	const struct modem_chat_script *start_scan_script;
	const struct modem_chat_script *sleep_script;
	const struct modem_chat_script *udfcfg_script;
	const struct modem_chat_script *udfscancfg_script;

} ant_b10_config_t;

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
int ant_b10_command(const struct device *dev, const char *cmd_format, ...);

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
 */
int ant_b10_udfcfg_set(const struct device *dev, ant_b10_udfcfg_tags_e tag,  void *state, bool save);

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
 */
int ant_b10_udfscancfg_set(const struct device *ant_b10_dev, ant_b10_udfscan_tags_e tag, int value, bool save);

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
int ant_b10_led_set(const struct device *dev, ant_b10_led_e led, ant_b10_led_state_e state);

/**
 * @brief Wakes up the ANT-B10 device.
 *
 * Toggles the status GPIO to wake up the device and waits for the device to be ready.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return 0 on success, or a negative error code on failure.
 */
int ant_b10_wake(const struct device *dev);

/**
 * @brief Starts a scan operation on the ANT-B10 device.
 *
 * Opens the UART pipe, attaches the modem chat, and runs the start scan script.
 * The pipe and chat remain open for receiving unsolicited messages.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return 0 on success, or a negative error code on failure.
 */
int ant_b10_start_scan(const struct device *ant_b10_dev);

/**
 * @brief Puts the ANT-B10 device into sleep mode.
 *
 * Runs the sleep script which disables scanning and resets the device.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return 0 on success, or a negative error code on failure.
 */
int ant_b10_sleep(const struct device *dev);

/**
 * @brief Retrieves the current configuration from the ANT-B10 device.
 *
 * Runs scripts to get both UDFCFG and UDFSCANCFG configuration.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return 0 on success, or a negative error code on failure.
 */
int ant_b10_config_get(const struct device *dev);

/**
 * @brief Gets the azimuth angle from the last scan.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return The azimuth angle.
 */
int ant_b10_azimutal_angle_get(const struct device *dev);

/**
 * @brief Gets the elevation angle from the last scan.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return The elevation angle.
 */
int ant_b10_elevation_angle_get(const struct device *dev);

/**
 * @brief Gets the direct angle from the last scan.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return The direct angle.
 */
int ant_b10_direct_angle_get(const struct device *dev);

/**
 * @brief Gets the RSSI from the last scan.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return The received signal strength indicator (RSSI).
 */
int ant_b10_rssi_get(const struct device *dev);

/**
 * @brief Performs a factory reset on the ANT-B10 device.
 *
 * Sends commands to reset the device to factory settings.
 *
 * @param ant_b10_dev Pointer to the ANT-B10 device.
 * @return 0 on success, or a negative error code on failure.
 */
int ant_b10_factory_reset(const struct device *ant_b10_dev);

#ifdef __cplusplus
}
#endif

#endif /* __ANT_B10_H__ */