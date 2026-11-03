# Command-line build of the STM32U575 firmware with the GNU Arm toolchain shipped with STM32CubeIDE.
# Usage:  powershell -ExecutionPolicy Bypass -File build.ps1 [-Toolchain <path to arm-none-eabi bin>]
param(
    [string]$Toolchain = ""
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
if ($Toolchain -eq "") {
    $gcc = Get-ChildItem "C:\ST" -Recurse -Filter arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($null -eq $gcc) { throw "arm-none-eabi-gcc not found; pass -Toolchain" }
    $Toolchain = $gcc.DirectoryName
}
$cc = Join-Path $Toolchain "arm-none-eabi-gcc.exe"
$size = Join-Path $Toolchain "arm-none-eabi-size.exe"
$objcopy = Join-Path $Toolchain "arm-none-eabi-objcopy.exe"
$out = Join-Path $root "Build"
New-Item -ItemType Directory -Force $out | Out-Null

$common = @("-mcpu=cortex-m33","-mfpu=fpv5-sp-d16","-mfloat-abi=hard","-mthumb","-DUSE_HAL_DRIVER","-DSTM32U575xx",
            "-ffunction-sections","-fdata-sections","-O2","-g3","-Wall","-Wextra","-fstack-usage")
$inc = @("-I$root\Inc","-I$root\Drivers\CMSIS\Include","-I$root\Drivers\CMSIS\Device\ST\STM32U5xx\Include","-I$root\Drivers\STM32U5xx_HAL_Driver\Inc")
$halNames = "hal","hal_cortex","hal_rcc","hal_rcc_ex","hal_gpio","hal_pwr","hal_pwr_ex","hal_tim","hal_tim_ex",
            "hal_spi","hal_spi_ex","hal_adc","hal_adc_ex","hal_uart","hal_uart_ex","hal_iwdg","hal_icache",
            "hal_exti","hal_flash","hal_flash_ex","hal_dma","hal_dma_ex"
$sources = @()
$sources += Get-ChildItem "$root\Src\*.c" | ForEach-Object { $_.FullName }
$sources += $halNames | ForEach-Object { "$root\Drivers\STM32U5xx_HAL_Driver\Src\stm32u5xx_$_.c" }
$sources += "$root\STM32CubeIDE\Example\User\syscalls.c","$root\STM32CubeIDE\Example\User\sysmem.c"
$sources += "$root\STM32CubeIDE\Example\Startup\startup_stm32u575zitx.s"

$objects = @()
foreach ($src in $sources) {
    $obj = Join-Path $out (([IO.Path]::GetFileNameWithoutExtension($src)) + ".o")
    $isAsm = $src.EndsWith(".s")
    $args = @() + $common + $inc
    if ($isAsm) { $args += @("-x","assembler-with-cpp") }
    $args += @("-c",$src,"-o",$obj)
    & $cc @args
    if ($LASTEXITCODE -ne 0) { throw "compile failed: $src" }
    $objects += $obj
}
$elf = Join-Path $out "STM32U575_Volumetric_Firmware.elf"
$linkArgs = @() + $common + @("-T$root\STM32CubeIDE\STM32U575ZITX_FLASH.ld","--specs=nano.specs","-Wl,--gc-sections",
            "-Wl,-Map=$out\firmware.map","-static","-Wl,--start-group","-lc","-lm","-Wl,--end-group") + $objects + @("-o",$elf)
& $cc @linkArgs
if ($LASTEXITCODE -ne 0) { throw "link failed" }
& $size $elf
& $objcopy -O binary $elf (Join-Path $out "STM32U575_Volumetric_Firmware.bin")
Write-Host "Build OK: $elf"
