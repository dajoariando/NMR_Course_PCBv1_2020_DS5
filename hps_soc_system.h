#ifndef _ALTERA_HPS_SOC_SYSTEM_H_
#define _ALTERA_HPS_SOC_SYSTEM_H_

/*
 * This file was automatically generated by the swinfo2header utility.
 * 
 * Created from SOPC Builder system 'soc_system' in
 * file '../QUARTUS-SoC/CODE_qsys/soc_system.sopcinfo'.
 */

/*
 * This file contains macros for module 'hps_0' and devices
 * connected to the following masters:
 *   h2f_axi_master
 *   h2f_lw_axi_master
 * 
 * Do not include this header file and another header file created for a
 * different module or master group at the same time.
 * Doing so may result in duplicate macro names.
 * Instead, use the system header file which has macros with unique names.
 */

/*
 * Macros for device 'adc_fifo_mem_out', class 'altera_avalon_fifo'
 * The macros are prefixed with 'ADC_FIFO_MEM_OUT_'.
 * The prefix is the slave descriptor.
 */
#define ADC_FIFO_MEM_OUT_COMPONENT_TYPE altera_avalon_fifo
#define ADC_FIFO_MEM_OUT_COMPONENT_NAME adc_fifo_mem
#define ADC_FIFO_MEM_OUT_BASE 0x0
#define ADC_FIFO_MEM_OUT_SPAN 8
#define ADC_FIFO_MEM_OUT_END 0x7
#define ADC_FIFO_MEM_OUT_AVALONMM_AVALONMM_DATA_WIDTH 32
#define ADC_FIFO_MEM_OUT_AVALONMM_AVALONST_DATA_WIDTH 32
#define ADC_FIFO_MEM_OUT_BITS_PER_SYMBOL 16
#define ADC_FIFO_MEM_OUT_CHANNEL_WIDTH 0
#define ADC_FIFO_MEM_OUT_ERROR_WIDTH 0
#define ADC_FIFO_MEM_OUT_FIFO_DEPTH 1024
#define ADC_FIFO_MEM_OUT_SINGLE_CLOCK_MODE 1
#define ADC_FIFO_MEM_OUT_SYMBOLS_PER_BEAT 2
#define ADC_FIFO_MEM_OUT_USE_AVALONMM_READ_SLAVE 1
#define ADC_FIFO_MEM_OUT_USE_AVALONMM_WRITE_SLAVE 0
#define ADC_FIFO_MEM_OUT_USE_AVALONST_SINK 1
#define ADC_FIFO_MEM_OUT_USE_AVALONST_SOURCE 0
#define ADC_FIFO_MEM_OUT_USE_BACKPRESSURE 1
#define ADC_FIFO_MEM_OUT_USE_IRQ 0
#define ADC_FIFO_MEM_OUT_USE_PACKET 0
#define ADC_FIFO_MEM_OUT_USE_READ_CONTROL 0
#define ADC_FIFO_MEM_OUT_USE_REGISTER 0
#define ADC_FIFO_MEM_OUT_USE_WRITE_CONTROL 1

/*
 * Macros for device 'sysid_qsys', class 'altera_avalon_sysid_qsys'
 * The macros are prefixed with 'SYSID_QSYS_'.
 * The prefix is the slave descriptor.
 */
#define SYSID_QSYS_COMPONENT_TYPE altera_avalon_sysid_qsys
#define SYSID_QSYS_COMPONENT_NAME sysid_qsys
#define SYSID_QSYS_BASE 0x8
#define SYSID_QSYS_SPAN 8
#define SYSID_QSYS_END 0xf
#define SYSID_QSYS_ID 2899645186
#define SYSID_QSYS_TIMESTAMP 1594954625

/*
 * Macros for device 'nmr_parameters_samples_per_echo', class 'altera_avalon_pio'
 * The macros are prefixed with 'NMR_PARAMETERS_SAMPLES_PER_ECHO_'.
 * The prefix is the slave descriptor.
 */
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_COMPONENT_TYPE altera_avalon_pio
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_COMPONENT_NAME nmr_parameters_samples_per_echo
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_BASE 0x10
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_SPAN 16
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_END 0x1f
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_BIT_CLEARING_EDGE_REGISTER 0
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_BIT_MODIFYING_OUTPUT_REGISTER 0
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_CAPTURE 0
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_DATA_WIDTH 32
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_DO_TEST_BENCH_WIRING 0
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_DRIVEN_SIM_VALUE 0
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_EDGE_TYPE NONE
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_FREQ 50000000
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_HAS_IN 0
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_HAS_OUT 1
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_HAS_TRI 0
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_IRQ_TYPE NONE
#define NMR_PARAMETERS_SAMPLES_PER_ECHO_RESET_VALUE 255

/*
 * Macros for device 'dac_preamp', class 'altera_avalon_spi'
 * The macros are prefixed with 'DAC_PREAMP_'.
 * The prefix is the slave descriptor.
 */
#define DAC_PREAMP_COMPONENT_TYPE altera_avalon_spi
#define DAC_PREAMP_COMPONENT_NAME dac_preamp
#define DAC_PREAMP_BASE 0x20
#define DAC_PREAMP_SPAN 32
#define DAC_PREAMP_END 0x3f
#define DAC_PREAMP_IRQ 2
#define DAC_PREAMP_CLOCKMULT 1
#define DAC_PREAMP_CLOCKPHASE 0
#define DAC_PREAMP_CLOCKPOLARITY 1
#define DAC_PREAMP_CLOCKUNITS "Hz"
#define DAC_PREAMP_DATABITS 24
#define DAC_PREAMP_DATAWIDTH 32
#define DAC_PREAMP_DELAYMULT "1.0E-9"
#define DAC_PREAMP_DELAYUNITS "ns"
#define DAC_PREAMP_EXTRADELAY 0
#define DAC_PREAMP_INSERT_SYNC 0
#define DAC_PREAMP_ISMASTER 1
#define DAC_PREAMP_LSBFIRST 0
#define DAC_PREAMP_NUMSLAVES 1
#define DAC_PREAMP_PREFIX "spi_"
#define DAC_PREAMP_SYNC_REG_DEPTH 2
#define DAC_PREAMP_TARGETCLOCK 128000
#define DAC_PREAMP_TARGETSSDELAY "0.0"

/*
 * Macros for device 'i2c_ext', class 'altera_avalon_i2c'
 * The macros are prefixed with 'I2C_EXT_'.
 * The prefix is the slave descriptor.
 */
#define I2C_EXT_COMPONENT_TYPE altera_avalon_i2c
#define I2C_EXT_COMPONENT_NAME i2c_ext
#define I2C_EXT_BASE 0x40
#define I2C_EXT_SPAN 64
#define I2C_EXT_END 0x7f
#define I2C_EXT_IRQ 4
#define I2C_EXT_FIFO_DEPTH 32
#define I2C_EXT_FREQ 50000000
#define I2C_EXT_USE_AV_ST 0

/*
 * Macros for device 'nmr_sys_pll_reconfig', class 'altera_pll_reconfig'
 * The macros are prefixed with 'NMR_SYS_PLL_RECONFIG_'.
 * The prefix is the slave descriptor.
 */
#define NMR_SYS_PLL_RECONFIG_COMPONENT_TYPE altera_pll_reconfig
#define NMR_SYS_PLL_RECONFIG_COMPONENT_NAME nmr_sys_pll_reconfig
#define NMR_SYS_PLL_RECONFIG_BASE 0x200
#define NMR_SYS_PLL_RECONFIG_SPAN 256
#define NMR_SYS_PLL_RECONFIG_END 0x2ff

/*
 * Macros for device 'i2c_int', class 'altera_avalon_i2c'
 * The macros are prefixed with 'I2C_INT_'.
 * The prefix is the slave descriptor.
 */
#define I2C_INT_COMPONENT_TYPE altera_avalon_i2c
#define I2C_INT_COMPONENT_NAME i2c_int
#define I2C_INT_BASE 0x300
#define I2C_INT_SPAN 64
#define I2C_INT_END 0x33f
#define I2C_INT_IRQ 1
#define I2C_INT_FIFO_DEPTH 32
#define I2C_INT_FREQ 50000000
#define I2C_INT_USE_AV_ST 0

/*
 * Macros for device 'adc_fifo_mem_in_csr', class 'altera_avalon_fifo'
 * The macros are prefixed with 'ADC_FIFO_MEM_IN_CSR_'.
 * The prefix is the slave descriptor.
 */
#define ADC_FIFO_MEM_IN_CSR_COMPONENT_TYPE altera_avalon_fifo
#define ADC_FIFO_MEM_IN_CSR_COMPONENT_NAME adc_fifo_mem
#define ADC_FIFO_MEM_IN_CSR_BASE 0x340
#define ADC_FIFO_MEM_IN_CSR_SPAN 32
#define ADC_FIFO_MEM_IN_CSR_END 0x35f
#define ADC_FIFO_MEM_IN_CSR_AVALONMM_AVALONMM_DATA_WIDTH 32
#define ADC_FIFO_MEM_IN_CSR_AVALONMM_AVALONST_DATA_WIDTH 32
#define ADC_FIFO_MEM_IN_CSR_BITS_PER_SYMBOL 16
#define ADC_FIFO_MEM_IN_CSR_CHANNEL_WIDTH 0
#define ADC_FIFO_MEM_IN_CSR_ERROR_WIDTH 0
#define ADC_FIFO_MEM_IN_CSR_FIFO_DEPTH 1024
#define ADC_FIFO_MEM_IN_CSR_SINGLE_CLOCK_MODE 1
#define ADC_FIFO_MEM_IN_CSR_SYMBOLS_PER_BEAT 2
#define ADC_FIFO_MEM_IN_CSR_USE_AVALONMM_READ_SLAVE 1
#define ADC_FIFO_MEM_IN_CSR_USE_AVALONMM_WRITE_SLAVE 0
#define ADC_FIFO_MEM_IN_CSR_USE_AVALONST_SINK 1
#define ADC_FIFO_MEM_IN_CSR_USE_AVALONST_SOURCE 0
#define ADC_FIFO_MEM_IN_CSR_USE_BACKPRESSURE 1
#define ADC_FIFO_MEM_IN_CSR_USE_IRQ 0
#define ADC_FIFO_MEM_IN_CSR_USE_PACKET 0
#define ADC_FIFO_MEM_IN_CSR_USE_READ_CONTROL 0
#define ADC_FIFO_MEM_IN_CSR_USE_REGISTER 0
#define ADC_FIFO_MEM_IN_CSR_USE_WRITE_CONTROL 1

/*
 * Macros for device 'nmr_parameters_rx_delay', class 'altera_avalon_pio'
 * The macros are prefixed with 'NMR_PARAMETERS_RX_DELAY_'.
 * The prefix is the slave descriptor.
 */
#define NMR_PARAMETERS_RX_DELAY_COMPONENT_TYPE altera_avalon_pio
#define NMR_PARAMETERS_RX_DELAY_COMPONENT_NAME nmr_parameters_rx_delay
#define NMR_PARAMETERS_RX_DELAY_BASE 0x380
#define NMR_PARAMETERS_RX_DELAY_SPAN 16
#define NMR_PARAMETERS_RX_DELAY_END 0x38f
#define NMR_PARAMETERS_RX_DELAY_BIT_CLEARING_EDGE_REGISTER 0
#define NMR_PARAMETERS_RX_DELAY_BIT_MODIFYING_OUTPUT_REGISTER 0
#define NMR_PARAMETERS_RX_DELAY_CAPTURE 0
#define NMR_PARAMETERS_RX_DELAY_DATA_WIDTH 32
#define NMR_PARAMETERS_RX_DELAY_DO_TEST_BENCH_WIRING 0
#define NMR_PARAMETERS_RX_DELAY_DRIVEN_SIM_VALUE 0
#define NMR_PARAMETERS_RX_DELAY_EDGE_TYPE NONE
#define NMR_PARAMETERS_RX_DELAY_FREQ 50000000
#define NMR_PARAMETERS_RX_DELAY_HAS_IN 0
#define NMR_PARAMETERS_RX_DELAY_HAS_OUT 1
#define NMR_PARAMETERS_RX_DELAY_HAS_TRI 0
#define NMR_PARAMETERS_RX_DELAY_IRQ_TYPE NONE
#define NMR_PARAMETERS_RX_DELAY_RESET_VALUE 0

/*
 * Macros for device 'ctrl_in', class 'altera_avalon_pio'
 * The macros are prefixed with 'CTRL_IN_'.
 * The prefix is the slave descriptor.
 */
#define CTRL_IN_COMPONENT_TYPE altera_avalon_pio
#define CTRL_IN_COMPONENT_NAME ctrl_in
#define CTRL_IN_BASE 0x3a0
#define CTRL_IN_SPAN 16
#define CTRL_IN_END 0x3af
#define CTRL_IN_BIT_CLEARING_EDGE_REGISTER 0
#define CTRL_IN_BIT_MODIFYING_OUTPUT_REGISTER 0
#define CTRL_IN_CAPTURE 0
#define CTRL_IN_DATA_WIDTH 8
#define CTRL_IN_DO_TEST_BENCH_WIRING 0
#define CTRL_IN_DRIVEN_SIM_VALUE 0
#define CTRL_IN_EDGE_TYPE NONE
#define CTRL_IN_FREQ 50000000
#define CTRL_IN_HAS_IN 1
#define CTRL_IN_HAS_OUT 0
#define CTRL_IN_HAS_TRI 0
#define CTRL_IN_IRQ_TYPE NONE
#define CTRL_IN_RESET_VALUE 0

/*
 * Macros for device 'ctrl_out', class 'altera_avalon_pio'
 * The macros are prefixed with 'CTRL_OUT_'.
 * The prefix is the slave descriptor.
 */
#define CTRL_OUT_COMPONENT_TYPE altera_avalon_pio
#define CTRL_OUT_COMPONENT_NAME ctrl_out
#define CTRL_OUT_BASE 0x3b0
#define CTRL_OUT_SPAN 16
#define CTRL_OUT_END 0x3bf
#define CTRL_OUT_BIT_CLEARING_EDGE_REGISTER 0
#define CTRL_OUT_BIT_MODIFYING_OUTPUT_REGISTER 0
#define CTRL_OUT_CAPTURE 0
#define CTRL_OUT_DATA_WIDTH 32
#define CTRL_OUT_DO_TEST_BENCH_WIRING 0
#define CTRL_OUT_DRIVEN_SIM_VALUE 0
#define CTRL_OUT_EDGE_TYPE NONE
#define CTRL_OUT_FREQ 50000000
#define CTRL_OUT_HAS_IN 0
#define CTRL_OUT_HAS_OUT 1
#define CTRL_OUT_HAS_TRI 0
#define CTRL_OUT_IRQ_TYPE NONE
#define CTRL_OUT_RESET_VALUE 119

/*
 * Macros for device 'nmr_parameters_pulse_t1', class 'altera_avalon_pio'
 * The macros are prefixed with 'NMR_PARAMETERS_PULSE_T1_'.
 * The prefix is the slave descriptor.
 */
#define NMR_PARAMETERS_PULSE_T1_COMPONENT_TYPE altera_avalon_pio
#define NMR_PARAMETERS_PULSE_T1_COMPONENT_NAME nmr_parameters_pulse_t1
#define NMR_PARAMETERS_PULSE_T1_BASE 0x3c0
#define NMR_PARAMETERS_PULSE_T1_SPAN 16
#define NMR_PARAMETERS_PULSE_T1_END 0x3cf
#define NMR_PARAMETERS_PULSE_T1_BIT_CLEARING_EDGE_REGISTER 0
#define NMR_PARAMETERS_PULSE_T1_BIT_MODIFYING_OUTPUT_REGISTER 0
#define NMR_PARAMETERS_PULSE_T1_CAPTURE 0
#define NMR_PARAMETERS_PULSE_T1_DATA_WIDTH 32
#define NMR_PARAMETERS_PULSE_T1_DO_TEST_BENCH_WIRING 0
#define NMR_PARAMETERS_PULSE_T1_DRIVEN_SIM_VALUE 0
#define NMR_PARAMETERS_PULSE_T1_EDGE_TYPE NONE
#define NMR_PARAMETERS_PULSE_T1_FREQ 50000000
#define NMR_PARAMETERS_PULSE_T1_HAS_IN 0
#define NMR_PARAMETERS_PULSE_T1_HAS_OUT 1
#define NMR_PARAMETERS_PULSE_T1_HAS_TRI 0
#define NMR_PARAMETERS_PULSE_T1_IRQ_TYPE NONE
#define NMR_PARAMETERS_PULSE_T1_RESET_VALUE 0

/*
 * Macros for device 'nmr_parameters_pulse_90deg', class 'altera_avalon_pio'
 * The macros are prefixed with 'NMR_PARAMETERS_PULSE_90DEG_'.
 * The prefix is the slave descriptor.
 */
#define NMR_PARAMETERS_PULSE_90DEG_COMPONENT_TYPE altera_avalon_pio
#define NMR_PARAMETERS_PULSE_90DEG_COMPONENT_NAME nmr_parameters_pulse_90deg
#define NMR_PARAMETERS_PULSE_90DEG_BASE 0x3d0
#define NMR_PARAMETERS_PULSE_90DEG_SPAN 16
#define NMR_PARAMETERS_PULSE_90DEG_END 0x3df
#define NMR_PARAMETERS_PULSE_90DEG_BIT_CLEARING_EDGE_REGISTER 0
#define NMR_PARAMETERS_PULSE_90DEG_BIT_MODIFYING_OUTPUT_REGISTER 0
#define NMR_PARAMETERS_PULSE_90DEG_CAPTURE 0
#define NMR_PARAMETERS_PULSE_90DEG_DATA_WIDTH 32
#define NMR_PARAMETERS_PULSE_90DEG_DO_TEST_BENCH_WIRING 0
#define NMR_PARAMETERS_PULSE_90DEG_DRIVEN_SIM_VALUE 0
#define NMR_PARAMETERS_PULSE_90DEG_EDGE_TYPE NONE
#define NMR_PARAMETERS_PULSE_90DEG_FREQ 50000000
#define NMR_PARAMETERS_PULSE_90DEG_HAS_IN 0
#define NMR_PARAMETERS_PULSE_90DEG_HAS_OUT 1
#define NMR_PARAMETERS_PULSE_90DEG_HAS_TRI 0
#define NMR_PARAMETERS_PULSE_90DEG_IRQ_TYPE NONE
#define NMR_PARAMETERS_PULSE_90DEG_RESET_VALUE 16

/*
 * Macros for device 'nmr_parameters_pulse_180deg', class 'altera_avalon_pio'
 * The macros are prefixed with 'NMR_PARAMETERS_PULSE_180DEG_'.
 * The prefix is the slave descriptor.
 */
#define NMR_PARAMETERS_PULSE_180DEG_COMPONENT_TYPE altera_avalon_pio
#define NMR_PARAMETERS_PULSE_180DEG_COMPONENT_NAME nmr_parameters_pulse_180deg
#define NMR_PARAMETERS_PULSE_180DEG_BASE 0x3e0
#define NMR_PARAMETERS_PULSE_180DEG_SPAN 16
#define NMR_PARAMETERS_PULSE_180DEG_END 0x3ef
#define NMR_PARAMETERS_PULSE_180DEG_BIT_CLEARING_EDGE_REGISTER 0
#define NMR_PARAMETERS_PULSE_180DEG_BIT_MODIFYING_OUTPUT_REGISTER 0
#define NMR_PARAMETERS_PULSE_180DEG_CAPTURE 0
#define NMR_PARAMETERS_PULSE_180DEG_DATA_WIDTH 32
#define NMR_PARAMETERS_PULSE_180DEG_DO_TEST_BENCH_WIRING 0
#define NMR_PARAMETERS_PULSE_180DEG_DRIVEN_SIM_VALUE 0
#define NMR_PARAMETERS_PULSE_180DEG_EDGE_TYPE NONE
#define NMR_PARAMETERS_PULSE_180DEG_FREQ 50000000
#define NMR_PARAMETERS_PULSE_180DEG_HAS_IN 0
#define NMR_PARAMETERS_PULSE_180DEG_HAS_OUT 1
#define NMR_PARAMETERS_PULSE_180DEG_HAS_TRI 0
#define NMR_PARAMETERS_PULSE_180DEG_IRQ_TYPE NONE
#define NMR_PARAMETERS_PULSE_180DEG_RESET_VALUE 16

/*
 * Macros for device 'nmr_parameters_init_delay', class 'altera_avalon_pio'
 * The macros are prefixed with 'NMR_PARAMETERS_INIT_DELAY_'.
 * The prefix is the slave descriptor.
 */
#define NMR_PARAMETERS_INIT_DELAY_COMPONENT_TYPE altera_avalon_pio
#define NMR_PARAMETERS_INIT_DELAY_COMPONENT_NAME nmr_parameters_init_delay
#define NMR_PARAMETERS_INIT_DELAY_BASE 0x3f0
#define NMR_PARAMETERS_INIT_DELAY_SPAN 16
#define NMR_PARAMETERS_INIT_DELAY_END 0x3ff
#define NMR_PARAMETERS_INIT_DELAY_BIT_CLEARING_EDGE_REGISTER 0
#define NMR_PARAMETERS_INIT_DELAY_BIT_MODIFYING_OUTPUT_REGISTER 0
#define NMR_PARAMETERS_INIT_DELAY_CAPTURE 0
#define NMR_PARAMETERS_INIT_DELAY_DATA_WIDTH 32
#define NMR_PARAMETERS_INIT_DELAY_DO_TEST_BENCH_WIRING 0
#define NMR_PARAMETERS_INIT_DELAY_DRIVEN_SIM_VALUE 0
#define NMR_PARAMETERS_INIT_DELAY_EDGE_TYPE NONE
#define NMR_PARAMETERS_INIT_DELAY_FREQ 50000000
#define NMR_PARAMETERS_INIT_DELAY_HAS_IN 0
#define NMR_PARAMETERS_INIT_DELAY_HAS_OUT 1
#define NMR_PARAMETERS_INIT_DELAY_HAS_TRI 0
#define NMR_PARAMETERS_INIT_DELAY_IRQ_TYPE NONE
#define NMR_PARAMETERS_INIT_DELAY_RESET_VALUE 0

/*
 * Macros for device 'nmr_parameters_echoes_per_scan', class 'altera_avalon_pio'
 * The macros are prefixed with 'NMR_PARAMETERS_ECHOES_PER_SCAN_'.
 * The prefix is the slave descriptor.
 */
#define NMR_PARAMETERS_ECHOES_PER_SCAN_COMPONENT_TYPE altera_avalon_pio
#define NMR_PARAMETERS_ECHOES_PER_SCAN_COMPONENT_NAME nmr_parameters_echoes_per_scan
#define NMR_PARAMETERS_ECHOES_PER_SCAN_BASE 0x400
#define NMR_PARAMETERS_ECHOES_PER_SCAN_SPAN 16
#define NMR_PARAMETERS_ECHOES_PER_SCAN_END 0x40f
#define NMR_PARAMETERS_ECHOES_PER_SCAN_BIT_CLEARING_EDGE_REGISTER 0
#define NMR_PARAMETERS_ECHOES_PER_SCAN_BIT_MODIFYING_OUTPUT_REGISTER 0
#define NMR_PARAMETERS_ECHOES_PER_SCAN_CAPTURE 0
#define NMR_PARAMETERS_ECHOES_PER_SCAN_DATA_WIDTH 32
#define NMR_PARAMETERS_ECHOES_PER_SCAN_DO_TEST_BENCH_WIRING 0
#define NMR_PARAMETERS_ECHOES_PER_SCAN_DRIVEN_SIM_VALUE 0
#define NMR_PARAMETERS_ECHOES_PER_SCAN_EDGE_TYPE NONE
#define NMR_PARAMETERS_ECHOES_PER_SCAN_FREQ 50000000
#define NMR_PARAMETERS_ECHOES_PER_SCAN_HAS_IN 0
#define NMR_PARAMETERS_ECHOES_PER_SCAN_HAS_OUT 1
#define NMR_PARAMETERS_ECHOES_PER_SCAN_HAS_TRI 0
#define NMR_PARAMETERS_ECHOES_PER_SCAN_IRQ_TYPE NONE
#define NMR_PARAMETERS_ECHOES_PER_SCAN_RESET_VALUE 0

/*
 * Macros for device 'nmr_parameters_delay_t1', class 'altera_avalon_pio'
 * The macros are prefixed with 'NMR_PARAMETERS_DELAY_T1_'.
 * The prefix is the slave descriptor.
 */
#define NMR_PARAMETERS_DELAY_T1_COMPONENT_TYPE altera_avalon_pio
#define NMR_PARAMETERS_DELAY_T1_COMPONENT_NAME nmr_parameters_delay_t1
#define NMR_PARAMETERS_DELAY_T1_BASE 0x410
#define NMR_PARAMETERS_DELAY_T1_SPAN 16
#define NMR_PARAMETERS_DELAY_T1_END 0x41f
#define NMR_PARAMETERS_DELAY_T1_BIT_CLEARING_EDGE_REGISTER 0
#define NMR_PARAMETERS_DELAY_T1_BIT_MODIFYING_OUTPUT_REGISTER 0
#define NMR_PARAMETERS_DELAY_T1_CAPTURE 0
#define NMR_PARAMETERS_DELAY_T1_DATA_WIDTH 32
#define NMR_PARAMETERS_DELAY_T1_DO_TEST_BENCH_WIRING 0
#define NMR_PARAMETERS_DELAY_T1_DRIVEN_SIM_VALUE 0
#define NMR_PARAMETERS_DELAY_T1_EDGE_TYPE NONE
#define NMR_PARAMETERS_DELAY_T1_FREQ 50000000
#define NMR_PARAMETERS_DELAY_T1_HAS_IN 0
#define NMR_PARAMETERS_DELAY_T1_HAS_OUT 1
#define NMR_PARAMETERS_DELAY_T1_HAS_TRI 0
#define NMR_PARAMETERS_DELAY_T1_IRQ_TYPE NONE
#define NMR_PARAMETERS_DELAY_T1_RESET_VALUE 0

/*
 * Macros for device 'nmr_parameters_delay_sig', class 'altera_avalon_pio'
 * The macros are prefixed with 'NMR_PARAMETERS_DELAY_SIG_'.
 * The prefix is the slave descriptor.
 */
#define NMR_PARAMETERS_DELAY_SIG_COMPONENT_TYPE altera_avalon_pio
#define NMR_PARAMETERS_DELAY_SIG_COMPONENT_NAME nmr_parameters_delay_sig
#define NMR_PARAMETERS_DELAY_SIG_BASE 0x420
#define NMR_PARAMETERS_DELAY_SIG_SPAN 16
#define NMR_PARAMETERS_DELAY_SIG_END 0x42f
#define NMR_PARAMETERS_DELAY_SIG_BIT_CLEARING_EDGE_REGISTER 0
#define NMR_PARAMETERS_DELAY_SIG_BIT_MODIFYING_OUTPUT_REGISTER 0
#define NMR_PARAMETERS_DELAY_SIG_CAPTURE 0
#define NMR_PARAMETERS_DELAY_SIG_DATA_WIDTH 32
#define NMR_PARAMETERS_DELAY_SIG_DO_TEST_BENCH_WIRING 0
#define NMR_PARAMETERS_DELAY_SIG_DRIVEN_SIM_VALUE 0
#define NMR_PARAMETERS_DELAY_SIG_EDGE_TYPE NONE
#define NMR_PARAMETERS_DELAY_SIG_FREQ 50000000
#define NMR_PARAMETERS_DELAY_SIG_HAS_IN 0
#define NMR_PARAMETERS_DELAY_SIG_HAS_OUT 1
#define NMR_PARAMETERS_DELAY_SIG_HAS_TRI 0
#define NMR_PARAMETERS_DELAY_SIG_IRQ_TYPE NONE
#define NMR_PARAMETERS_DELAY_SIG_RESET_VALUE 16

/*
 * Macros for device 'nmr_parameters_delay_nosig', class 'altera_avalon_pio'
 * The macros are prefixed with 'NMR_PARAMETERS_DELAY_NOSIG_'.
 * The prefix is the slave descriptor.
 */
#define NMR_PARAMETERS_DELAY_NOSIG_COMPONENT_TYPE altera_avalon_pio
#define NMR_PARAMETERS_DELAY_NOSIG_COMPONENT_NAME nmr_parameters_delay_nosig
#define NMR_PARAMETERS_DELAY_NOSIG_BASE 0x430
#define NMR_PARAMETERS_DELAY_NOSIG_SPAN 16
#define NMR_PARAMETERS_DELAY_NOSIG_END 0x43f
#define NMR_PARAMETERS_DELAY_NOSIG_BIT_CLEARING_EDGE_REGISTER 0
#define NMR_PARAMETERS_DELAY_NOSIG_BIT_MODIFYING_OUTPUT_REGISTER 0
#define NMR_PARAMETERS_DELAY_NOSIG_CAPTURE 0
#define NMR_PARAMETERS_DELAY_NOSIG_DATA_WIDTH 32
#define NMR_PARAMETERS_DELAY_NOSIG_DO_TEST_BENCH_WIRING 0
#define NMR_PARAMETERS_DELAY_NOSIG_DRIVEN_SIM_VALUE 0
#define NMR_PARAMETERS_DELAY_NOSIG_EDGE_TYPE NONE
#define NMR_PARAMETERS_DELAY_NOSIG_FREQ 50000000
#define NMR_PARAMETERS_DELAY_NOSIG_HAS_IN 0
#define NMR_PARAMETERS_DELAY_NOSIG_HAS_OUT 1
#define NMR_PARAMETERS_DELAY_NOSIG_HAS_TRI 0
#define NMR_PARAMETERS_DELAY_NOSIG_IRQ_TYPE NONE
#define NMR_PARAMETERS_DELAY_NOSIG_RESET_VALUE 16

/*
 * Macros for device 'jtag_uart', class 'altera_avalon_jtag_uart'
 * The macros are prefixed with 'JTAG_UART_'.
 * The prefix is the slave descriptor.
 */
#define JTAG_UART_COMPONENT_TYPE altera_avalon_jtag_uart
#define JTAG_UART_COMPONENT_NAME jtag_uart
#define JTAG_UART_BASE 0x440
#define JTAG_UART_SPAN 8
#define JTAG_UART_END 0x447
#define JTAG_UART_IRQ 2
#define JTAG_UART_READ_DEPTH 64
#define JTAG_UART_READ_THRESHOLD 8
#define JTAG_UART_WRITE_DEPTH 64
#define JTAG_UART_WRITE_THRESHOLD 8


#endif /* _ALTERA_HPS_SOC_SYSTEM_H_ */
