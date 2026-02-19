param(
    [string]$SourceDir = "",
    [string]$BuildDir = "",
    [switch]$SaveBaseline
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($SourceDir))
{
    $SourceDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
}
if ([string]::IsNullOrWhiteSpace($BuildDir))
{
    $BuildDir = Join-Path $SourceDir "build/stclang-foc"
}

$starmBundleRoot = "C:/Users/keruth/AppData/Local/stm32cube/bundles/st-arm-clang/19.1.6+st.10"
$starmBin = Join-Path $starmBundleRoot "bin"
$gnuBin = "C:/Users/keruth/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin"
$starmObjdump = Join-Path $starmBin "starm-objdump.exe"

if (-not (Test-Path $starmObjdump))
{
    throw "starm-objdump not found: $starmObjdump"
}

$env:PATH = "$starmBin;$gnuBin;$env:PATH"
$env:CLANG_GCC_CMSIS_COMPILER = $starmBundleRoot
$env:GCC_TOOLCHAIN_ROOT = $gnuBin

$toolchainFile = Join-Path $SourceDir "cmake/starm-clang.cmake"

cmake -S $SourceDir -B $BuildDir -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="$toolchainFile"
if ($LASTEXITCODE -ne 0)
{
    exit $LASTEXITCODE
}

cmake --build $BuildDir --target AxDr -- -j4
if ($LASTEXITCODE -ne 0)
{
    exit $LASTEXITCODE
}

$elfPath = Join-Path $BuildDir "AxDr.elf"
$disasmPath = Join-Path $BuildDir "AxDr.disasm.S"
& $starmObjdump -d -C $elfPath | Out-File -Encoding UTF8 $disasmPath

$symbolSpecs = @(
    @{ Name = "ADC1_2_IRQHandler"; Match = "ADC1_2_IRQHandler"; Exact = $true },
    @{ Name = "HAL_ADCEx_InjectedConvCpltCallback"; Match = "HAL_ADCEx_InjectedConvCpltCallback"; Exact = $true },
    @{ Name = "XRFOC_Bench_ControllerStep"; Match = "XRFOC_Bench_ControllerStep"; Exact = $true },
    @{ Name = "CoreStepImpl<false>"; Match = "CoreStepImpl<false>"; Exact = $false },
    @{ Name = "STM32Inverter::SetDuty"; Match = "STM32Inverter::SetDuty\("; Exact = $false }
)

function Get-InstructionCount
{
    param(
        [string[]]$Lines,
        [string]$Match,
        [bool]$Exact
    )

    $inFunction = $false
    $count = 0

    foreach ($line in $Lines)
    {
        if ($line -match "^([0-9a-fA-F]+) <(.+)>:$")
        {
            if ($Exact)
            {
                $inFunction = ($Matches[2] -eq $Match)
            }
            else
            {
                $inFunction = ($Matches[2] -match $Match)
            }
            continue
        }
        if ($inFunction -and $line -match "^\s+[0-9a-fA-F]+:\s+[0-9a-fA-F ]+\s+[a-zA-Z]")
        {
            $count++
        }
    }

    return $count
}

$disasmLines = Get-Content -Encoding UTF8 $disasmPath
$currentCounts = @{}
foreach ($spec in $symbolSpecs)
{
    $currentCounts[$spec.Name] = Get-InstructionCount -Lines $disasmLines -Match $spec.Match -Exact $spec.Exact
}

$currentJsonPath = Join-Path $BuildDir "foc_instr_current.json"
$currentCounts | ConvertTo-Json | Out-File -Encoding UTF8 $currentJsonPath

$baselineJsonPath = Join-Path $BuildDir "foc_instr_baseline.json"
if ($SaveBaseline)
{
    Copy-Item $currentJsonPath $baselineJsonPath -Force
    Write-Output "Baseline saved: $baselineJsonPath"
}

Write-Output ""
Write-Output "| Symbol | Baseline | Current | Delta | Delta % |"
Write-Output "| --- | ---: | ---: | ---: | ---: |"

if (Test-Path $baselineJsonPath)
{
    $baselineCounts = Get-Content -Raw -Encoding UTF8 $baselineJsonPath | ConvertFrom-Json
    foreach ($spec in $symbolSpecs)
    {
        $symbol = $spec.Name
        $baseline = $null
        if ($null -ne $baselineCounts.PSObject.Properties[$symbol])
        {
            $baseline = [int]$baselineCounts.$symbol
        }
        $current = [int]$currentCounts[$symbol]
        if ($null -eq $baseline)
        {
            Write-Output "| $symbol | - | $current | - | - |"
            continue
        }
        $delta = $current - $baseline
        $deltaPct = 0.0
        if ($baseline -ne 0)
        {
            $deltaPct = ($delta * 100.0) / $baseline
        }
        Write-Output "| $symbol | $baseline | $current | $delta | $("{0:N2}" -f $deltaPct)% |"
    }
}
else
{
    foreach ($spec in $symbolSpecs)
    {
        $symbol = $spec.Name
        $current = [int]$currentCounts[$symbol]
        Write-Output "| $symbol | - | $current | - | - |"
    }
}

Write-Output ""
Write-Output "Disassembly: $disasmPath"
Write-Output "Current counts: $currentJsonPath"
