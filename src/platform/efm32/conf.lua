-- Configuration file for the EFM32GG-DK3750 board

local sdk_path = "/SiliconLabs/SimplicityStudio/v2/developer/sdks/efm32/v2"
local cmsis_inc_path = sdk_path .. "/CMSIS/Include"
local emlib_src_path = sdk_path .. "/emlib/src"
local emlib_inc_path = sdk_path .. "/emlib/inc"
local device_src_path = sdk_path .. "/Device/SiliconLabs/EFM32GG/Source"
local device_inc_path = sdk_path .. "/Device/SiliconLabs/EFM32GG/Include"
local bsp_path = sdk_path .. "/kits/common/bsp"
local drivers_path = sdk_path .. "/kits/common/drivers"
local kitconfig_path = sdk_path .. "/kits/EFM32GG_DK3750/config"

-- include paths
addi( sf( 'src/platform/%s/', platform ) )

addi( sf( '%s', cmsis_inc_path ) )
addi( sf( '%s', device_inc_path ) )
addi( sf( '%s', emlib_inc_path ) )
addi( sf( '%s', bsp_path ) )
addi( sf( '%s', drivers_path ) )
addi( sf( '%s', kitconfig_path ) )


-- source files
local bsp_files = "bsp_dk_3201.c "
local device_files = "system_efm32gg.c "
local emlib_files = "em_assert.c em_cmu.c em_emu.c em_ebi.c em_gpio.c em_int.c em_system.c em_usart.c "
local source_files = "platform.c startup_efm32gg.s "

local ldscript = "efm32.ld"

bsp_files = utils.prepend_path( bsp_files, bsp_path )
device_files = utils.prepend_path( device_files, device_src_path )
emlib_files = utils.prepend_path( emlib_files, emlib_src_path )
source_files = utils.prepend_path( source_files, "src/platform/" .. platform )

specific_files = bsp_files .. device_files .. emlib_files .. source_files

-- Prepend with path
specific_files = specific_files .. " src/platform/cortex_utils.s src/platform/arm_cortex_interrupts.c"
ldscript = sf( "src/platform/%s/%s", platform, ldscript )

addm( { "FOR" .. cnorm( comp.cpu ), "FOR" .. cnorm( comp.board ), 'gcc', 'EFM32GG990F1024', 'CORTEX_M3' } )

-- Standard GCC Flags
addcf( { '-ffunction-sections', '-fdata-sections', '-fno-strict-aliasing', '-Wall' } )
addlf( { '-nostartfiles','-nostdlib', '-T', ldscript, '-Wl,--gc-sections', '-Wl,--allow-multiple-definition' } )
addaf( { '-x', 'assembler-with-cpp', '-c', '-Wall' } )
addlib( { 'c','gcc','m' } )

local target_flags = { '-mcpu=cortex-m3', '-mthumb' }

-- Configure general flags for target
addcf( { target_flags, '-mlittle-endian' } )
addlf( { target_flags, '-Wl,-e,Reset_Handler', '-Wl,-static' , ' -Xlinker -Map=efm32.map'} )
addaf( target_flags )

-- Toolset data
tools.efm32 = {}

-- Array of file names that will be checked against the 'prog' target; their absence will force a rebuild
tools.efm32.prog_flist = { output .. ".bin"}

-- We use 'gcc' as the assembler
toolset.asm = toolset.compile

