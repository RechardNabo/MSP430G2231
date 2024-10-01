################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccs1271/ccs/tools/compiler/ti-cgt-msp430_21.6.1.LTS/bin/cl430" -vmsp --use_hw_mpy=none --include_path="C:/ti/ccs1271/ccs/ccs_base/msp430/include" --include_path="C:/Users/rechardn/Documents/workspace_v12/MSP430G2231_LIBRARY" --include_path="C:/ti/ccs1271/ccs/tools/compiler/ti-cgt-msp430_21.6.1.LTS/include" --advice:power="all" --define=__MSP430G2231__ -g --printf_support=full --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


