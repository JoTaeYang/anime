$ErrorActionPreference = "Stop"
$S = $PSScriptRoot
$Repo = (Resolve-Path (Join-Path $S "..\..\..\..")).Path
Set-Location $Repo
function BL([string]$Script, [string]$Blend = "", [string[]]$Rest = @()) {
    Write-Host ">>> $Script $Blend $Rest"
    $a = @("-NoProfile", "-File", "$S\bl.ps1", "-Script", $Script)
    if ($Blend) { $a += @("-Blend", $Blend) }
    if ($Rest.Count) { $a += $Rest }
    & powershell @a
    if ($LASTEXITCODE -ne 0) { throw "FAILED $Script (exit $LASTEXITCODE)" }
}
BL s00_source_prep.py
BL s01_pivots.py gob_r00_source.blend
BL check_g1_source.py gob_r00_source.blend
BL s02a_body.py gob_r00_source.blend
BL s02b_limbs.py work/r01a_body.blend
BL s02c_rigid.py work/r01b_limbs.blend
BL s02d_finish.py work/r01c_rigid.blend
BL s02e_mouth.py work/r01d_merged.blend
BL check_g2_static.py gob_r01_retopo.blend
BL check_g2_shape.py gob_r01_retopo.blend
BL s03_temprig.py gob_r01_retopo.blend
BL check_g3_deform.py gob_r01t_temprig.blend
BL s04_skeleton.py gob_r01_retopo.blend
BL check_g4_skeleton.py gob_r02_skeleton.blend
BL s05_skin.py gob_r02_skeleton.blend
BL check_g5_skin.py gob_r03_skinned.blend
BL s06a_ctrl_torso_head.py gob_r03_skinned.blend
BL s06b_ctrl_arms.py work/r04a.blend
BL s06c_ctrl_legs.py work/r04b.blend
BL s06d_ctrl_weapon.py work/r04c.blend
BL s06e_rig_ui.py work/r04d.blend
BL check_g6_ctrl.py gob_r04_ctrl.blend
BL s07a_rigtest.py gob_r04_ctrl.blend
BL check_g7_bake.py gob_r05_rigtest.blend
BL s08_export_fbx.py gob_r05_rigtest.blend
BL check_g8_fbx.py
New-Item -ItemType Directory -Force unity/AvatarCheck/Assets/Goblin | Out-Null
Copy-Item work/goblin_swing/rig/export/goblin.fbx, "work/goblin_swing/rig/export/goblin@rigtest.fbx", work/goblin_swing/rig/export/goblin_club.fbx unity/AvatarCheck/Assets/Goblin/ -Force
BL s09_blender_ref.py export/stage_rigtest.blend
& powershell -NoProfile -File "$S\unity_goblin.ps1" -Method GoblinRigCheck.Run
if ($LASTEXITCODE -ne 0) { throw "FAILED GoblinRigCheck (exit $LASTEXITCODE)" }
BL compare_g9.py
# G10.3: 기준선 대비 변경 경로
$base = Get-Content work/goblin_swing/rig/inspect/baseline_git_status.txt
$now = git status --porcelain -uall
$allowed = '^(\?\?|.M|M.|A.)\s+"?(work/goblin_swing/rig/|unity/AvatarCheck/Assets/Editor/Goblin|unity/AvatarCheck/Assets/Goblin)'
$new = Compare-Object $base $now | Where-Object SideIndicator -eq "=>" | ForEach-Object InputObject
$bad = $new | Where-Object { $_ -notmatch $allowed }
New-Item -ItemType Directory -Force work/goblin_swing/rig/inspect/G10 | Out-Null
@{ new_status_lines = @($new); out_of_scope = @($bad) } | ConvertTo-Json | Set-Content -Encoding utf8 work/goblin_swing/rig/inspect/G10/scope.json
Write-Host "OUT_OF_SCOPE_COUNT=$(@($bad).Count)"
