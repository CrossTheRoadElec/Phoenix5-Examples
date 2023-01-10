$Depth = 1
$Exclude_dirs = "HERO C#", ".github"
$Levels = '/*' * $Depth

$ErrorActionPreference = "Stop"

Get-ChildItem -Directory "./$Levels" -Exclude $Exclude_dirs |
    ForEach-Object {
        Push-Location $_.FullName
        echo "Building example $_"
        ./gradlew build
        if (-not $?) {
            throw "Example $_ failed to build"
        }
        Pop-Location
    }