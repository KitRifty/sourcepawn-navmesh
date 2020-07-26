# SourcePawn NavMesh

This is basically a SourceMod plugin that can parse .NAV files and make data out of it that
SourceMod plugins can read from. This plugin by itself doesn't do anything other than read
data from the .NAV file. Other plugins have to utilize this plugin's features in order for 
this to have any purpose.

Special thanks goes to Anthony Iacano (pimpinjuice) for the parser code, which can be found here: 
https://github.com/AnthonyIacono/War3SourceV2/tree/master/Nav

## Game Compatibility

- **Team Fortress 2** - fully supported

I don't plan on working on support for other games, but feel free to open a pull request if you wish to do so. Make use of the `NavMeshLoadCustomDataPreArea`, `NavMeshLoadCustomData`, and `NavMeshLoadAreaCustomData` functions as much as possible.
