param([Parameter(Mandatory = $true)][string]$Method)
$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..\..")).Path
$Unity = "C:\Program Files\Unity\Hub\Editor\6000.3.20f1\Editor\Unity.exe"
$args2 = @("-batchmode", "-quit", "-projectPath", (Join-Path $Repo "unity\AvatarCheck"),
           "-executeMethod", $Method, "-logFile", (Join-Path $Repo "unity\AvatarCheck\Logs\goblin_$Method.log"))
# Wait for Unity.exe only. Start-Process -Wait also waits for the child dotnet process Unity leaves behind (~+10 min).
$p = Start-Process -FilePath $Unity -ArgumentList $args2 -PassThru -NoNewWindow
$null = $p.Handle   # cache the handle so ExitCode is available after exit
$p.WaitForExit()
exit $p.ExitCode
