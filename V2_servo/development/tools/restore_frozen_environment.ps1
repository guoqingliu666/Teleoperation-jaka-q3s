param([string]$EnvironmentPath='D:\ChatGPT\Environments\v2_shadow_20261004',[switch]$VerifyOnly)
$ErrorActionPreference='Stop'
$v2Root=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$env:TEMP='D:\ChatGPT\Temp';$env:TMP=$env:TEMP;$env:CONDA_PKGS_DIRS='D:\ChatGPT\Cache\conda-v2-restore'
$env:PYTHONDONTWRITEBYTECODE='1'
$target=[IO.Path]::GetFullPath($EnvironmentPath)
if(-not $target.StartsWith('D:\',[StringComparison]::OrdinalIgnoreCase)){throw 'Restore environment must stay on D:'}
$manifest=Get-Content -LiteralPath (Join-Path $v2Root 'RELEASE_MANIFEST.json') -Raw | ConvertFrom-Json
foreach($entry in $manifest.files){
    $p=[IO.Path]::GetFullPath((Join-Path $v2Root $entry.path))
    if(-not $p.StartsWith($v2Root+'\',[StringComparison]::OrdinalIgnoreCase)){throw 'Invalid manifest path'}
    if(-not(Test-Path -LiteralPath $p)){throw "Missing $($entry.path)"}
    if((Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash.ToLowerInvariant() -ne $entry.sha256){throw "SHA mismatch $($entry.path)"}
}
Write-Output 'FROZEN_FILES_SHA256_OK; no SDK or motion'
if($VerifyOnly){exit 0}
if(Test-Path -LiteralPath $target){throw 'Environment exists. Activate and verify it, or choose a NEW D: path; never overwrite.'}
$conda=$env:CONDA_EXE
if(-not $conda){$conda=(Get-Command conda -CommandType Application -ErrorAction Stop).Source}
$packages=Get-Content -LiteralPath (Join-Path $v2Root 'reproduction/environment-packages.json') -Raw | ConvertFrom-Json
$lines=@('@EXPLICIT')
foreach($package in $packages){
    $p=[IO.Path]::GetFullPath((Join-Path $v2Root ('reproduction/conda-packages/'+$package.filename)))
    $lines+=([Uri]$p).AbsoluteUri+'#'+$package.sha256
}
$list=Join-Path $v2Root 'reproduction/local-install.explicit.txt'
$lines | Set-Content -LiteralPath $list -Encoding ascii
& $conda create --yes --offline --prefix $target --file $list
if($LASTEXITCODE -ne 0){throw 'Conda offline restore failed'}
$python=Join-Path $target 'python.exe'
& $python (Join-Path $v2Root 'run_v2_app.py') --check
if($LASTEXITCODE -ne 0){throw 'Restored runtime check failed'}
Write-Output "Restored. conda activate $target ; run START_V2.cmd from this release root."
