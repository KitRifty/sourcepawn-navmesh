# SourcePawn NavMesh

This is basically a SourceMod plugin that can parse .NAV files and make data out of it that
SourceMod plugins can read from. This plugin by itself doesn't do anything other than read
data from the .NAV file. Other plugins have to utilize this plugin's features in order for 
this to have any purpose.

Special thanks goes to Anthony Iacano (pimpinjuice) for the parser code, which can be found here: 
https://github.com/AnthonyIacono/War3SourceV2/tree/master/Nav

## Requirements
- SourceMod 1.10+

## Game Compatibility

| Name | Supported? |
| ---  | :---: |
| Team Fortress 2 | :heavy_check_mark: |
| Counter-Strike: Global Offensive | :heavy_check_mark: |
| Counter-Strike: Source | :heavy_check_mark: |
| Left 4 Dead 2 | :x: |

If your game isn't listed here, there's a *slight* chance it may still work, just that it's never been tested. You really have to pray to God that the game doesn't append any custom data to the NavMesh and/or its areas (which, realistically, is hardly ever the case). It'll get pretty obvious that it doesn't work if you see the script loading in an infinite amount of areas, or you get a memory overflow, or the script execution just times out... whatever comes first, really.

I don't plan on working on support for other games, but feel free to open a pull request if you wish to do so. Make use of the `NavMeshLoadCustomDataPreArea`, `NavMeshLoadCustomData`, and `NavMeshLoadAreaCustomData` functions as much as possible.

## Reversing a game-specific .NAV format

As a start, you may use [VTable Dumper](https://asherkin.github.io/vtable/) and check for any subclasses of `CNavArea` or `CNavMesh`, and checking if the subclass overrides the following functions:

```c++
class CNavMesh
{
  virtual NavErrorType Load( void );
  virtual void LoadCustomData( CUtlBuffer &fileBuffer, unsigned int subVersion ) { }
  virtual void LoadCustomDataPreArea( CUtlBuffer &fileBuffer, unsigned int subVersion ) { }
}

class CNavArea
{
  virtual NavErrorType Load( CUtlBuffer &fileBuffer, unsigned int version, unsigned int subVersion );
}
```

If it's the case that some of those functions are being overridden, then it'll give you a good starting point on where to look when disassembling a Linux server binary. If you do stumble upon custom data being loaded in a function, take note of all calls to `CUtlBuffer::Get<Type>`. 

If there aren't any subclasses, then you might not have to do anything and there's a chance that the plugin will work as is.

If you want to disassemble a Windows server binary, then God help you.

## Current Dev. Goals

- **Move away from using `ArrayStack`.** Transition functions to push to a given `ArrayList` rather than allocate an `ArrayStack`. 
