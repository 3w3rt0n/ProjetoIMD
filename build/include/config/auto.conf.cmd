deps_config := \
	/home/Desktop/esp-idf/components/app_trace/Kconfig \
	/home/Desktop/esp-idf/components/aws_iot/Kconfig \
	/home/Desktop/esp-idf/components/bt/Kconfig \
	/home/Desktop/esp-idf/components/esp32/Kconfig \
	/home/Desktop/esp-idf/components/ethernet/Kconfig \
	/home/Desktop/esp-idf/components/fatfs/Kconfig \
	/home/Desktop/esp-idf/components/freertos/Kconfig \
	/home/Desktop/esp-idf/components/heap/Kconfig \
	/home/Desktop/esp-idf/components/libsodium/Kconfig \
	/home/Desktop/esp-idf/components/log/Kconfig \
	/home/Desktop/esp-idf/components/lwip/Kconfig \
	/home/Desktop/esp-idf/components/mbedtls/Kconfig \
	/home/Desktop/esp-idf/components/openssl/Kconfig \
	/home/Desktop/esp-idf/components/pthread/Kconfig \
	/home/Desktop/esp-idf/components/spi_flash/Kconfig \
	/home/Desktop/esp-idf/components/spiffs/Kconfig \
	/home/Desktop/esp-idf/components/tcpip_adapter/Kconfig \
	/home/Desktop/esp-idf/components/wear_levelling/Kconfig \
	/home/Desktop/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/Desktop/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/Desktop/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/Desktop/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
