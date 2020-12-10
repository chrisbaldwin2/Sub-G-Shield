################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
smartrf_settings/%.obj: ../smartrf_settings/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.2.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/Chris/workspace_v10/spi_sdp_CC1310_LAUNCHXL_tirtos_ccs2" --include_path="C:/ti/simplelink_cc13x0_sdk_4_10_01_01/source/ti/posix/ccs" --include_path="C:/ti/ccs1011/ccs/tools/compiler/ti-cgt-arm_20.2.2.LTS/include" --define=DeviceFamily_CC13X0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="smartrf_settings/$(basename $(<F)).d_raw" --obj_directory="smartrf_settings" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


