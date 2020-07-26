# SourcePawn NavMesh

This is basically a SourceMod plugin that can parse .NAV files and make data out of it that
SourceMod plugins can read from. This plugin by itself doesn't do anything other than read
data from the .NAV file. Other plugins have to utilize this plugin's features in order for 
this to have any purpose.

Special thanks goes to Anthony Iacano (pimpinjuice) for the parser code, which can be found here: 
https://github.com/AnthonyIacono/War3SourceV2/tree/master/Nav

## Game Compatibility

- **Team Fortress 2** - fully supported

If your game isn't listed here, there's a *slight* chance it may still work, just that it's never been tested. You really have to pray to God that the game doesn't append any custom data to the NavMesh and/or its areas (which, realistically, is hardly ever the case). It'll get pretty obvious that it doesn't work if you see the script loading in an infinite amount of areas, or you get a memory overflow, or the script execution just times out... whatever comes first, really.

I don't plan on working on support for other games, but feel free to open a pull request if you wish to do so. Make use of the `NavMeshLoadCustomDataPreArea`, `NavMeshLoadCustomData`, and `NavMeshLoadAreaCustomData` functions as much as possible.

## Current Dev. Goals

- **Move away from using `ArrayStack`.** Transition functions to push to a given `ArrayList` rather than allocate an `ArrayStack`. 
