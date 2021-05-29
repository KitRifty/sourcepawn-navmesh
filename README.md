# SourcePawn NavMesh
![](https://github.com/KitRifty/sourcepawn-navmesh/actions/workflows/compile.yml/badge.svg?branch=master)

This is basically a SourceMod plugin that can parse .NAV files and make data out of it that
SourceMod plugins can read from. This plugin by itself doesn't do anything other than read
data from the .NAV file. Other plugins have to utilize this plugin's features in order for
this to have any purpose.

Special thanks goes to Anthony Iacano (pimpinjuice) for the parser code, which can be found here:
https://github.com/AnthonyIacono/War3SourceV2/tree/master/Nav

## Requirements

- SourceMod 1.10+

## Game Compatibility

| Name                             |     Supported?     |
| -------------------------------- | :----------------: |
| Team Fortress 2                  | :heavy_check_mark: |
| Counter-Strike: Global Offensive | :heavy_check_mark: |
| Counter-Strike: Source           | :heavy_check_mark: |
| Left 4 Dead 2                    | :heavy_check_mark: |

If your game isn't listed here, there's a _slight_ chance it may still work, just that it's never been tested. You really have to pray to God that the game doesn't append any custom data to the NavMesh and/or its areas (which, realistically, is hardly ever the case). It'll get pretty obvious that it doesn't work if you see the script loading in an infinite amount of areas, or you get a memory overflow, or the script execution just times out... whatever comes first, really.

If your game doesn't natively support .NAV files, can you still use this plugin? [Yes.](../../wiki/Using-the-plugin-in-non-native-games)

Want to reverse a .NAV format for a game? [Read here.](../../wiki/Reversing-a-.NAV-File-Format) 

## Current Dev. Goals

- **Move away from using `ArrayStack`.** Transition functions to push to a given `ArrayList` rather than allocate an `ArrayStack`.
- **Replace NavMeshArea\_\* natives with CNavArea methodmap natives.**
