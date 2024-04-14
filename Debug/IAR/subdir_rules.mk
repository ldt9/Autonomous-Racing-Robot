################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
IAR/%.obj: ../IAR/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.6.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs1040/ccs/ccs_base/arm/include" --include_path="C:/ti/ccs1040/ccs/ccs_base/arm/include/CMSIS" --include_path="C:/Users/dicke/OneDrive - University of Pittsburgh/Desktop/MQTT_SJD/CC3100BOOST_MQTT-TwitterLED_MSP432P401R" --include_path="C:/Users/dicke/OneDrive - University of Pittsburgh/Desktop/MQTT_SJD/CC3100BOOST_MQTT-TwitterLED_MSP432P401R" --include_path="C:/Users/dicke/OneDrive - University of Pittsburgh/Desktop/MQTT_SJD/CC3100BOOST_MQTT-TwitterLED_MSP432P401R/simplelink/include" --include_path="C:/Users/dicke/OneDrive - University of Pittsburgh/Desktop/MQTT_SJD/CC3100BOOST_MQTT-TwitterLED_MSP432P401R/simplelink/source" --include_path="C:/Users/dicke/OneDrive - University of Pittsburgh/Desktop/MQTT_SJD/CC3100BOOST_MQTT-TwitterLED_MSP432P401R/board" --include_path="C:/Users/dicke/OneDrive - University of Pittsburgh/Desktop/MQTT_SJD/CC3100BOOST_MQTT-TwitterLED_MSP432P401R/cli_uart" --include_path="C:/Users/dicke/OneDrive - University of Pittsburgh/Desktop/MQTT_SJD/CC3100BOOST_MQTT-TwitterLED_MSP432P401R/driverlib/MSP432P4xx" --include_path="C:/Users/dicke/OneDrive - University of Pittsburgh/Desktop/MQTT_SJD/CC3100BOOST_MQTT-TwitterLED_MSP432P401R/spi_cc3100" --include_path="C:/Users/dicke/OneDrive - University of Pittsburgh/Desktop/MQTT_SJD/CC3100BOOST_MQTT-TwitterLED_MSP432P401R/uart_cc3100" --include_path="C:/Users/dicke/OneDrive - University of Pittsburgh/Desktop/MQTT_SJD/CC3100BOOST_MQTT-TwitterLED_MSP432P401R/mqtt" --include_path="C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.6.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs --define=_USE_CLI_ -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="IAR/$(basename $(<F)).d_raw" --obj_directory="IAR" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


