################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1120/ccs/tools/compiler/ti-cgt-arm_20.2.6.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me -O3 --include_path="C:/Users/joshu/Documents/SNHU School Related/CS-350/Week 7/Project_Kovacevich" --include_path="C:/Users/joshu/Documents/SNHU School Related/CS-350/Week 7/Project_Kovacevich/Debug" --include_path="C:/ti/simplelink_cc32xx_sdk_6_10_00_05/source" --include_path="C:/ti/simplelink_cc32xx_sdk_6_10_00_05/kernel/nortos" --include_path="C:/ti/simplelink_cc32xx_sdk_6_10_00_05/kernel/nortos/posix" --include_path="C:/ti/ccs1120/ccs/tools/compiler/ti-cgt-arm_20.2.6.LTS/include" --define=DeviceFamily_CC3220 --define=NORTOS_SUPPORT -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="C:/Users/joshu/Documents/SNHU School Related/CS-350/Week 7/Project_Kovacevich/Debug/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-230295121: ../gpiointerrupt.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs1120/ccs/utils/sysconfig_1.12.0/sysconfig_cli.bat" -s "C:/ti/simplelink_cc32xx_sdk_6_10_00_05/.metadata/product.json" --script "C:/Users/joshu/Documents/SNHU School Related/CS-350/Week 7/Project_Kovacevich/gpiointerrupt.syscfg" -o "syscfg" --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_drivers_config.c: build-230295121 ../gpiointerrupt.syscfg
syscfg/ti_drivers_config.h: build-230295121
syscfg/ti_utils_build_linker.cmd.genlibs: build-230295121
syscfg/syscfg_c.rov.xs: build-230295121
syscfg/ti_utils_runtime_model.gv: build-230295121
syscfg/ti_utils_runtime_Makefile: build-230295121
syscfg/: build-230295121

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1120/ccs/tools/compiler/ti-cgt-arm_20.2.6.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me -O3 --include_path="C:/Users/joshu/Documents/SNHU School Related/CS-350/Week 7/Project_Kovacevich" --include_path="C:/Users/joshu/Documents/SNHU School Related/CS-350/Week 7/Project_Kovacevich/Debug" --include_path="C:/ti/simplelink_cc32xx_sdk_6_10_00_05/source" --include_path="C:/ti/simplelink_cc32xx_sdk_6_10_00_05/kernel/nortos" --include_path="C:/ti/simplelink_cc32xx_sdk_6_10_00_05/kernel/nortos/posix" --include_path="C:/ti/ccs1120/ccs/tools/compiler/ti-cgt-arm_20.2.6.LTS/include" --define=DeviceFamily_CC3220 --define=NORTOS_SUPPORT -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="C:/Users/joshu/Documents/SNHU School Related/CS-350/Week 7/Project_Kovacevich/Debug/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1976928737: ../image.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs1120/ccs/utils/sysconfig_1.12.0/sysconfig_cli.bat" -s "C:/ti/simplelink_cc32xx_sdk_6_10_00_05/.metadata/product.json" --script "C:/Users/joshu/Documents/SNHU School Related/CS-350/Week 7/Project_Kovacevich/image.syscfg" -o "syscfg" --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_drivers_net_wifi_config.json: build-1976928737 ../image.syscfg
syscfg/: build-1976928737


