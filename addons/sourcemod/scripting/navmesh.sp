// Huge huge HUGE props and credits to Anthony Iacono (pimpinjuice) and his Nav-file parser code,
// which can be found here: https://github.com/AnthonyIacono/War3SourceV2/tree/master/Nav

// KitRifty: The rest of the code is based off code in the source-sdk-2013 repository. I only
// attempted to recreate those functions to be used in SourcePawn.

#include <sourcemod>
#include <sdktools>

#pragma newdecls required
#include <navmesh>

#define PLUGIN_VERSION "1.0.4"

public Plugin myinfo = 
{
    name = "SourcePawn Navigation Mesh Parser",
    author	= "KitRifty, Benoist3012",
    description	= "A plugin that can read Valve's Navigation Mesh.",
    version = PLUGIN_VERSION,
    url = ""
}

#define UNSIGNED_INT_BYTE_SIZE 4
#define UNSIGNED_CHAR_BYTE_SIZE 1
#define UNSIGNED_SHORT_BYTE_SIZE 2
#define FLOAT_BYTE_SIZE 4

Handle g_hNavMeshPlaces;
Handle g_hNavMeshAreas;
Handle g_hNavMeshAreaConnections;
Handle g_hNavMeshAreaHidingSpots;
Handle g_hNavMeshAreaEncounterPaths;
Handle g_hNavMeshAreaEncounterSpots;
Handle g_hNavMeshAreaLadderConnections;
Handle g_hNavMeshAreaVisibleAreas;

Handle g_hNavMeshLadders;

int g_iNavMeshMagicNumber;
int g_iNavMeshVersion;
int g_iNavMeshSubVersion;
int g_iNavMeshSaveBSPSize;
bool g_bNavMeshAnalyzed;

Handle g_hNavMeshGrid;
Handle g_hNavMeshGridLists;

float g_flNavMeshGridCellSize = 300.0;
float g_flNavMeshMinX;
float g_flNavMeshMinY;
int g_iNavMeshGridSizeX;
int g_iNavMeshGridSizeY;


bool g_bNavMeshBuilt = false;

// For A* pathfinding.
int g_iNavMeshAreaOpenListIndex = -1;
int g_iNavMeshAreaOpenListTailIndex = -1;
int g_iNavMeshAreaMasterMarker = 0;


public APLRes AskPluginLoad2(Handle myself, bool late, char[] error,int err_max)
{
	RegPluginLibrary("navmesh");
	
	CreateNative("NavMesh_Exists", Native_NavMeshExists);
	CreateNative("NavMesh_GetMagicNumber", Native_NavMeshGetMagicNumber);
	CreateNative("NavMesh_GetVersion", Native_NavMeshGetVersion);
	CreateNative("NavMesh_GetSubVersion", Native_NavMeshGetSubVersion);
	CreateNative("NavMesh_GetSaveBSPSize", Native_NavMeshGetSaveBSPSize);
	CreateNative("NavMesh_IsAnalyzed", Native_NavMeshIsAnalyzed);
	CreateNative("NavMesh_GetPlaces", Native_NavMeshGetPlaces);
	CreateNative("NavMesh_GetAreas", Native_NavMeshGetAreas);
	CreateNative("NavMesh_GetLadders", Native_NavMeshGetLadders);
	
	CreateNative("NavMesh_CollectSurroundingAreas", Native_NavMeshCollectSurroundingAreas);
	CreateNative("NavMesh_BuildPath", Native_NavMeshBuildPath);
	
	CreateNative("NavMesh_GetArea", Native_NavMeshGetArea);
	CreateNative("NavMesh_GetNearestArea", Native_NavMeshGetNearestArea);
	
	CreateNative("NavMesh_WorldToGridX", Native_NavMeshWorldToGridX);
	CreateNative("NavMesh_WorldToGridY", Native_NavMeshWorldToGridY);
	CreateNative("NavMesh_GetAreasOnGrid", Native_NavMeshGridGetAreas);
	CreateNative("NavMesh_GetGridSizeX", Native_NavMeshGetGridSizeX);
	CreateNative("NavMesh_GetGridSizeY", Native_NavMeshGetGridSizeY);
	
	CreateNative("NavMesh_GetGroundHeight", Native_NavMeshGetGroundHeight);
	
	CreateNative("NavMeshArea_GetMasterMarker", Native_NavMeshAreaGetMasterMarker);
	CreateNative("NavMeshArea_ChangeMasterMarker", Native_NavMeshAreaChangeMasterMarker);
	
	CreateNative("NavMeshArea_IsConnected", Native_NavMeshAreaIsConnected);
	CreateNative("NavMeshArea_GetFlags", Native_NavMeshAreaGetFlags);
	CreateNative("NavMeshArea_GetCenter", Native_NavMeshAreaGetCenter);
	CreateNative("NavMeshArea_GetAdjacentList", Native_NavMeshAreaGetAdjacentList);
	CreateNative("NavMeshArea_GetLadderList", Native_NavMeshAreaGetLadderList);
	CreateNative("NavMeshArea_GetClosestPointOnArea", Native_NavMeshAreaGetClosestPointOnArea);
	CreateNative("NavMeshArea_GetTotalCost", Native_NavMeshAreaGetTotalCost);
	CreateNative("NavMeshArea_GetParent", Native_NavMeshAreaGetParent);
	CreateNative("NavMeshArea_GetParentHow", Native_NavMeshAreaGetParentHow);
	CreateNative("NavMeshArea_SetParent", Native_NavMeshAreaSetParent);
	CreateNative("NavMeshArea_SetParentHow", Native_NavMeshAreaSetParentHow);
	CreateNative("NavMeshArea_GetCostSoFar", Native_NavMeshAreaGetCostSoFar);
	CreateNative("NavMeshArea_GetExtentLow", Native_NavMeshAreaGetExtentLow);
	CreateNative("NavMeshArea_GetExtentHigh", Native_NavMeshAreaGetExtentHigh);
	CreateNative("NavMeshArea_IsOverlappingPoint", Native_NavMeshAreaIsOverlappingPoint);
	CreateNative("NavMeshArea_IsOverlappingArea", Native_NavMeshAreaIsOverlappingArea);
	CreateNative("NavMeshArea_GetNECornerZ", Native_NavMeshAreaGetNECornerZ);
	CreateNative("NavMeshArea_GetSWCornerZ", Native_NavMeshAreaGetSWCornerZ);
	CreateNative("NavMeshArea_GetZ", Native_NavMeshAreaGetZ);
	CreateNative("NavMeshArea_GetZFromXAndY", Native_NavMeshAreaGetZFromXAndY);
	CreateNative("NavMeshArea_Contains", Native_NavMeshAreaContains);
	CreateNative("NavMeshArea_ComputePortal", Native_NavMeshAreaComputePortal);
	CreateNative("NavMeshArea_ComputeClosestPointInPortal", Native_NavMeshAreaComputeClosestPointInPortal);
	CreateNative("NavMeshArea_ComputeDirection", Native_NavMeshAreaComputeDirection);
	CreateNative("NavMeshArea_GetLightIntensity", Native_NavMeshAreaGetLightIntensity);
	
	CreateNative("NavMeshLadder_GetLength", Native_NavMeshLadderGetLength);
}

public void OnPluginStart()
{
	g_hNavMeshPlaces = CreateArray(256);
	g_hNavMeshAreas = CreateArray(NavMeshArea_MaxStats);
	g_hNavMeshAreaConnections = CreateArray(NavMeshConnection_MaxStats);
	g_hNavMeshAreaHidingSpots = CreateArray(NavMeshHidingSpot_MaxStats);
	g_hNavMeshAreaEncounterPaths = CreateArray(NavMeshEncounterPath_MaxStats);
	g_hNavMeshAreaEncounterSpots = CreateArray(NavMeshEncounterSpot_MaxStats);
	g_hNavMeshAreaLadderConnections = CreateArray(NavMeshLadderConnection_MaxStats);
	g_hNavMeshAreaVisibleAreas = CreateArray(NavMeshVisibleArea_MaxStats);
	
	g_hNavMeshLadders = CreateArray(NavMeshLadder_MaxStats);
	
	g_hNavMeshGrid = CreateArray(NavMeshGrid_MaxStats);
	g_hNavMeshGridLists = CreateArray(NavMeshGridList_MaxStats);
	
	HookEvent("nav_blocked", Event_NavAreaBlocked);
}

public void OnMapStart()
{
	NavMeshDestroy();

	char sMap[256];
	GetCurrentMap(sMap, sizeof(sMap));
	
	g_bNavMeshBuilt = NavMeshLoad(sMap);
}

public Action Event_NavAreaBlocked(Handle event, const char[] name, bool dB)
{
	if (!g_bNavMeshBuilt) return;

	int iAreaID = GetEventInt(event, "area");
	int iAreaIndex = FindValueInArray(g_hNavMeshAreas, iAreaID);
	if (iAreaIndex != -1)
	{
		bool bBlocked = view_as<bool>(GetEventInt(event, "blocked"));
		SetArrayCell(g_hNavMeshAreas, iAreaIndex, bBlocked, NavMeshArea_Blocked);
	}
}

stock int OppositeDirection(int iNavDirection)
{
	switch (iNavDirection)
	{
		case NAV_DIR_NORTH: return NAV_DIR_SOUTH;
		case NAV_DIR_SOUTH: return NAV_DIR_NORTH;
		case NAV_DIR_EAST: return NAV_DIR_WEST;
		case NAV_DIR_WEST: return NAV_DIR_EAST;
	}
	
	return NAV_DIR_NORTH;
}

stock float NavMeshAreaComputeAdjacentConnectionHeightChange(int iAreaIndex,int iTargetAreaIndex)
{
	bool bFoundArea = false;
	int iNavDirection;
	
	for (iNavDirection = 0; iNavDirection < NAV_DIR_COUNT; iNavDirection++)
	{
		Handle hConnections = NavMeshAreaGetAdjacentList(iAreaIndex, iNavDirection);
		if (hConnections == INVALID_HANDLE) continue;
		
		while (!IsStackEmpty(hConnections))
		{
			int iTempAreaIndex = -1;
			PopStackCell(hConnections, iTempAreaIndex);
			
			if (iTempAreaIndex == iTargetAreaIndex)
			{
				bFoundArea = true;
				break;
			}
		}
		
		CloseHandle(hConnections);
		
		if (bFoundArea) break;
	}
	
	if (!bFoundArea) return 99999999.9;
	
	float flMyEdge[3];
	float flHalfWidth;
	NavMeshAreaComputePortal(iAreaIndex, iTargetAreaIndex, iNavDirection, flMyEdge, flHalfWidth);
	
	float flOtherEdge[3];
	NavMeshAreaComputePortal(iAreaIndex, iTargetAreaIndex, OppositeDirection(iNavDirection), flOtherEdge, flHalfWidth);
	
	return flOtherEdge[2] - flMyEdge[2];
}

Handle NavMeshCollectSurroundingAreas(int iStartAreaIndex,
	float flTravelDistanceLimit=1500.0,
	float flMaxStepUpLimit=StepHeight,
	float flMaxDropDownLimit=100.0)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not search surrounding areas because the nav mesh does not exist!");
		return INVALID_HANDLE;
	}
	
	if (iStartAreaIndex == -1)
	{
		LogError("Could not search surrounding areas because the starting area does not exist!");
		return INVALID_HANDLE;
	}
	
	Handle hNearAreasList = CreateStack();
	
	NavMeshAreaClearSearchLists();
	
	NavMeshAreaAddToOpenList(iStartAreaIndex);
	SetArrayCell(g_hNavMeshAreas, iStartAreaIndex, 0, NavMeshArea_TotalCost);
	SetArrayCell(g_hNavMeshAreas, iStartAreaIndex, 0, NavMeshArea_CostSoFar);
	SetArrayCell(g_hNavMeshAreas, iStartAreaIndex, -1, NavMeshArea_Parent);
	SetArrayCell(g_hNavMeshAreas, iStartAreaIndex, NUM_TRAVERSE_TYPES, NavMeshArea_ParentHow);
	NavMeshAreaMark(iStartAreaIndex);
	
	while (!NavMeshAreaIsOpenListEmpty())
	{
		int iAreaIndex = NavMeshAreaPopOpenList();
		if (flTravelDistanceLimit > 0.0 && 
			float(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_CostSoFar)) > flTravelDistanceLimit)
		{
			continue;
		}
		
		int iAreaParent = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Parent);
		if (iAreaParent != -1)
		{
			float flDeltaZ = NavMeshAreaComputeAdjacentConnectionHeightChange(iAreaParent, iAreaIndex);
			if (flDeltaZ > flMaxStepUpLimit) continue;
			if (flDeltaZ < -flMaxDropDownLimit) continue;
		}
		
		PushStackCell(hNearAreasList, iAreaIndex);
		
		NavMeshAreaMark(iAreaIndex);
		
		for (int iNavDir = 0; iNavDir < NAV_DIR_COUNT; iNavDir++)
		{
			Handle hConnections = NavMeshAreaGetAdjacentList(iAreaIndex, iNavDir);
			if (hConnections != INVALID_HANDLE)
			{
				while (!IsStackEmpty(hConnections))
				{
					int iAdjacentAreaIndex = -1;
					PopStackCell(hConnections, iAdjacentAreaIndex);
					
					if (view_as<bool>(GetArrayCell(g_hNavMeshAreas, iAdjacentAreaIndex, NavMeshArea_Blocked))) continue;
					
					if (!NavMeshAreaIsMarked(iAdjacentAreaIndex))
					{
						SetArrayCell(g_hNavMeshAreas, iAdjacentAreaIndex, 0, NavMeshArea_TotalCost);
						SetArrayCell(g_hNavMeshAreas, iAdjacentAreaIndex, iAreaIndex, NavMeshArea_Parent);
						SetArrayCell(g_hNavMeshAreas, iAdjacentAreaIndex, iNavDir, NavMeshArea_ParentHow);
						
						int iDistAlong = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_CostSoFar);
						
						float flAdjacentAreaCenter[3], flAreaCenter[3];
						NavMeshAreaGetCenter(iAreaIndex, flAreaCenter);
						NavMeshAreaGetCenter(iAdjacentAreaIndex, flAdjacentAreaCenter);
						
						iDistAlong += RoundToFloor(GetVectorDistance(flAdjacentAreaCenter, flAreaCenter));
						SetArrayCell(g_hNavMeshAreas, iAdjacentAreaIndex, iDistAlong, NavMeshArea_CostSoFar);
						NavMeshAreaAddToOpenList(iAdjacentAreaIndex);
					}
				}
				
				CloseHandle(hConnections);
			}
		}
	}
	
	return hNearAreasList;
}

bool NavMeshAreaIsConnected(int iAreaIndex1,int iAreaIndex2)
{
	Handle hAreas = NavMeshCollectSurroundingAreas(iAreaIndex1,100000.0,1000.0,1000.0);			
	while (!IsStackEmpty(hAreas))
	{
		int iAreaIndex = -1;
		PopStackCell(hAreas, iAreaIndex);
		if(iAreaIndex==iAreaIndex2)
		{
			CloseHandle(hAreas);
			return true;
		}
	}
	return false;
}

bool NavMeshBuildPath(int iStartAreaIndex,
	int iGoalAreaIndex,
	const float flGoalPos[3],
	Handle hCostFunctionPlugin,
	Function iCostFunction,
	any iCostData=INVALID_HANDLE,
	int &iClosestAreaIndex=-1,
	float flMaxPathLength=0.0,
	float flMaxAng=0.0)
{
	if (!g_bNavMeshBuilt) 
	{
		LogError("Could not build path because the nav mesh does not exist!");
		return false;
	}
	
	if (iClosestAreaIndex != -1) 
	{
		iClosestAreaIndex = iStartAreaIndex;
	}
	
	if (iStartAreaIndex == -1)
	{
		LogError("Could not build path because the starting area does not exist!");
		return false;
	}
	
	SetArrayCell(g_hNavMeshAreas, iStartAreaIndex, -1, NavMeshArea_Parent);
	SetArrayCell(g_hNavMeshAreas, iStartAreaIndex, NUM_TRAVERSE_TYPES, NavMeshArea_ParentHow);
	
	if (iGoalAreaIndex == -1)
	{
		LogError("Could not build path from area %d to area %d because the goal area does not exist!");
		return false;
	}
	
	if (iStartAreaIndex == iGoalAreaIndex) return true;
	
	// Start the search.
	NavMeshAreaClearSearchLists();
	
	// Compute estimate of path length.
	float flStartAreaCenter[3];
	NavMeshAreaGetCenter(iStartAreaIndex, flStartAreaCenter);
	
	int iStartTotalCost = RoundFloat(GetVectorDistance(flStartAreaCenter, flGoalPos));
	SetArrayCell(g_hNavMeshAreas, iStartAreaIndex, iStartTotalCost, NavMeshArea_TotalCost);
	
	int iInitCost;
	
	Call_StartFunction(hCostFunctionPlugin, iCostFunction);
	Call_PushCell(iStartAreaIndex);
	Call_PushCell(-1);
	Call_PushCell(-1);
	Call_PushCell(iCostData);
	Call_Finish(iInitCost);
	
	if (iInitCost < 0) return false;
	
	SetArrayCell(g_hNavMeshAreas, iStartAreaIndex, 0, NavMeshArea_CostSoFar);
	SetArrayCell(g_hNavMeshAreas, iStartAreaIndex, 0.0, NavMeshArea_PathLengthSoFar);
	NavMeshAreaAddToOpenList(iStartAreaIndex);
	
	int iClosestAreaDist = iStartTotalCost;
	
	bool bHaveMaxPathLength = view_as<bool>(flMaxPathLength != 0.0);
	
	// Perform A* search.
	while (!NavMeshAreaIsOpenListEmpty())
	{
		int iAreaIndex = NavMeshAreaPopOpenList();
		
		if (view_as<bool>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Blocked))) 
		{
			// Don't consider blocked areas.
			continue;
		}
		
		if (iAreaIndex == iGoalAreaIndex ||
			(iGoalAreaIndex == -1 && NavMeshAreaContains(iAreaIndex, flGoalPos)))
		{
			if (iClosestAreaIndex != -1)
			{
				iClosestAreaIndex = iGoalAreaIndex;
			}
			
			return true;
		}
		
		// No support for elevator areas yet.
		static int SEARCH_FLOOR = 0, SEARCH_LADDERS = 1;
		
		int iSearchWhere = SEARCH_FLOOR;
		int iSearchDir = NAV_DIR_NORTH;
		
		Handle hFloorList = NavMeshAreaGetAdjacentList(iAreaIndex, iSearchDir);
		
		bool bLadderUp = true;
		Handle hLadderList = INVALID_HANDLE;
		int iLadderTopDir = 0;
		
		for (;;)
		{
			int iNewAreaIndex = -1;
			int iNavTraverseHow = 0;
			int iLadderIndex = -1;
			
			if (iSearchWhere == SEARCH_FLOOR)
			{
				if (hFloorList == INVALID_HANDLE || IsStackEmpty(hFloorList))
				{
					iSearchDir++;
					if (hFloorList != INVALID_HANDLE) CloseHandle(hFloorList);
					
					if (iSearchDir == NAV_DIR_COUNT)
					{
						iSearchWhere = SEARCH_LADDERS;
						
						hLadderList = NavMeshAreaGetLadderList(iAreaIndex, NAV_LADDER_DIR_UP);
						iLadderTopDir = 0;
					}
					else
					{
						hFloorList = NavMeshAreaGetAdjacentList(iAreaIndex, iSearchDir);
					}
					continue;
				}
				
				PopStackCell(hFloorList, iNewAreaIndex);
				iNavTraverseHow = iSearchDir;
			}
			else if (iSearchWhere == SEARCH_LADDERS)
			{
				if (hLadderList == INVALID_HANDLE || IsStackEmpty(hLadderList))
				{
					if (hLadderList != INVALID_HANDLE) CloseHandle(hLadderList);
					
					if (!bLadderUp)
					{
						iLadderIndex = -1;
						break;
					}
					else
					{
						bLadderUp = false;
						hLadderList = NavMeshAreaGetLadderList(iAreaIndex, NAV_LADDER_DIR_DOWN);
					}
					
					continue;
				}
				
				PopStackCell(hLadderList, iLadderIndex);
				
				if (bLadderUp)
				{
					switch (iLadderTopDir)
					{
						case 0:
						{
							iNewAreaIndex = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_TopForwardAreaIndex);
						}
						case 1:
						{
							iNewAreaIndex = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_TopLeftAreaIndex);
						}
						case 2:
						{
							iNewAreaIndex = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_TopRightAreaIndex);
						}
						default:
						{
							iLadderTopDir = 0;
							continue;
						}
					}
					
					iNavTraverseHow = GO_LADDER_UP;
					iLadderTopDir++;
				}
				else
				{
					iNewAreaIndex = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_BottomAreaIndex);
					iNavTraverseHow = GO_LADDER_DOWN;
				}
				
				if (iNewAreaIndex == -1) continue;
			}
			
			if (GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Parent) == iNewAreaIndex) 
			{
				// Don't backtrack.
				continue;
			}
			
			if (iNewAreaIndex == iAreaIndex)
			{
				continue;
			}
			
			if (view_as<bool>(GetArrayCell(g_hNavMeshAreas, iNewAreaIndex, NavMeshArea_Blocked))) 
			{
				// Don't consider blocked areas.
				continue;
			}
			
			float flNewAreaCenter[3];
			NavMeshAreaGetCenter(iNewAreaIndex, flNewAreaCenter);
			
			if (flMaxAng != 0.0)
			{
				float flAreaCenter[3];
				NavMeshAreaGetCenter(iAreaIndex, flAreaCenter);
				//We don't have to look for the step size, if we go off the clip.
				if(flAreaCenter[2] <= flNewAreaCenter[2])
				{
					float flH = GetVectorDistance(flNewAreaCenter, flAreaCenter);
					float flFakePoint[3];
					flFakePoint[0] = flNewAreaCenter[0];
					flFakePoint[1] = flNewAreaCenter[1];
					flFakePoint[2] = flAreaCenter[2];
					float flA = GetVectorDistance(flFakePoint, flAreaCenter);
					if(flMaxAng < (180*(Cosine(flA/flH))/PI))
						continue;
				}
			}
			
			int iNewCostSoFar;
			
			Call_StartFunction(hCostFunctionPlugin, iCostFunction);
			Call_PushCell(iNewAreaIndex);
			Call_PushCell(iAreaIndex);
			Call_PushCell(iLadderIndex);
			Call_Finish(iNewCostSoFar);
			
			if (iNewCostSoFar < 0) continue;
			
			if (bHaveMaxPathLength)
			{
				float flAreaCenter[3];
				NavMeshAreaGetCenter(iAreaIndex, flAreaCenter);
				
				float flDeltaLength = GetVectorDistance(flNewAreaCenter, flAreaCenter);
				float flNewLengthSoFar = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_PathLengthSoFar)) + flDeltaLength;
				if (flNewLengthSoFar > flMaxPathLength)
				{
					continue;
				}
				
				SetArrayCell(g_hNavMeshAreas, iNewAreaIndex, flNewLengthSoFar, NavMeshArea_PathLengthSoFar);
			}
			
			if ((NavMeshAreaIsOpen(iNewAreaIndex) || NavMeshAreaIsClosed(iNewAreaIndex)) &&
				GetArrayCell(g_hNavMeshAreas, iNewAreaIndex, NavMeshArea_CostSoFar) <= iNewCostSoFar)
			{
				continue;
			}
			else
			{
				int iNewCostRemaining = RoundFloat(GetVectorDistance(flNewAreaCenter, flGoalPos));
				
				if (iClosestAreaIndex != -1 && iNewCostRemaining < iClosestAreaDist)
				{
					iClosestAreaIndex = iNewAreaIndex;
					iClosestAreaDist = iNewCostRemaining;
				}
				
				SetArrayCell(g_hNavMeshAreas, iNewAreaIndex, iNewCostSoFar, NavMeshArea_CostSoFar);
				SetArrayCell(g_hNavMeshAreas, iNewAreaIndex, iNewCostSoFar + iNewCostRemaining, NavMeshArea_TotalCost);
				
				/*
				if (NavMeshAreaIsClosed(iNewAreaIndex)) 
				{
					NavMeshAreaRemoveFromClosedList(iNewAreaIndex);
				}
				*/
				
				if (NavMeshAreaIsOpen(iNewAreaIndex))
				{
					NavMeshAreaUpdateOnOpenList(iNewAreaIndex);
				}
				else
				{
					NavMeshAreaAddToOpenList(iNewAreaIndex);
				}
				
				SetArrayCell(g_hNavMeshAreas, iNewAreaIndex, iAreaIndex, NavMeshArea_Parent);
				SetArrayCell(g_hNavMeshAreas, iNewAreaIndex, iNavTraverseHow, NavMeshArea_ParentHow);
			}
		}
		
		NavMeshAreaAddToClosedList(iAreaIndex);
	}
	
	return false;
}

void NavMeshAreaClearSearchLists()
{
	g_iNavMeshAreaMasterMarker++;
	g_iNavMeshAreaOpenListIndex = -1;
	g_iNavMeshAreaOpenListTailIndex = -1;
}

bool NavMeshAreaIsMarked(int iAreaIndex)
{
	return view_as<bool>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Marker) == g_iNavMeshAreaMasterMarker);
}

void NavMeshAreaMark(int iAreaIndex)
{
	SetArrayCell(g_hNavMeshAreas, iAreaIndex, g_iNavMeshAreaMasterMarker, NavMeshArea_Marker);
}

bool NavMeshAreaIsOpen(int iAreaIndex)
{
	return view_as<bool>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_OpenMarker) == g_iNavMeshAreaMasterMarker);
}

bool NavMeshAreaIsOpenListEmpty()
{
	return view_as<bool>(g_iNavMeshAreaOpenListIndex == -1);
}

void NavMeshAreaAddToOpenList(int iAreaIndex)
{
	if (NavMeshAreaIsOpen(iAreaIndex)) return;
	
	SetArrayCell(g_hNavMeshAreas, iAreaIndex, g_iNavMeshAreaMasterMarker, NavMeshArea_OpenMarker);
	
	if (g_iNavMeshAreaOpenListIndex == -1)
	{
		g_iNavMeshAreaOpenListIndex = iAreaIndex;
		g_iNavMeshAreaOpenListTailIndex = iAreaIndex;
		SetArrayCell(g_hNavMeshAreas, iAreaIndex, -1, NavMeshArea_PrevOpenIndex);
		SetArrayCell(g_hNavMeshAreas, iAreaIndex, -1, NavMeshArea_NextOpenIndex);
		return;
	}
	
	int iTotalCost = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_TotalCost);
	
	int iTempAreaIndex = -1, iLastAreaIndex = -1;
	for (iTempAreaIndex = g_iNavMeshAreaOpenListIndex; iTempAreaIndex != -1; iTempAreaIndex = GetArrayCell(g_hNavMeshAreas, iTempAreaIndex, NavMeshArea_NextOpenIndex))
	{
		if (iTotalCost < GetArrayCell(g_hNavMeshAreas, iTempAreaIndex, NavMeshArea_TotalCost)) break;
		iLastAreaIndex = iTempAreaIndex;
	}
	
	if (iTempAreaIndex != -1)
	{
		int iPrevOpenIndex = GetArrayCell(g_hNavMeshAreas, iTempAreaIndex, NavMeshArea_PrevOpenIndex);
		SetArrayCell(g_hNavMeshAreas, iAreaIndex, iPrevOpenIndex, NavMeshArea_PrevOpenIndex);
		
		if (iPrevOpenIndex != -1)
		{
			SetArrayCell(g_hNavMeshAreas, iPrevOpenIndex, iAreaIndex, NavMeshArea_NextOpenIndex);
		}
		else
		{
			g_iNavMeshAreaOpenListIndex = iAreaIndex;
		}
		
		SetArrayCell(g_hNavMeshAreas, iAreaIndex, iTempAreaIndex, NavMeshArea_NextOpenIndex);
		SetArrayCell(g_hNavMeshAreas, iTempAreaIndex, iAreaIndex, NavMeshArea_PrevOpenIndex);
	}
	else
	{
		SetArrayCell(g_hNavMeshAreas, iLastAreaIndex, iAreaIndex, NavMeshArea_NextOpenIndex);
		SetArrayCell(g_hNavMeshAreas, iAreaIndex, iLastAreaIndex, NavMeshArea_PrevOpenIndex);
		
		SetArrayCell(g_hNavMeshAreas, iAreaIndex, -1, NavMeshArea_NextOpenIndex);
		
		g_iNavMeshAreaOpenListTailIndex = iAreaIndex;
	}
}

stock void NavMeshAreaAddToOpenListTail(int iAreaIndex)
{
	if (NavMeshAreaIsOpen(iAreaIndex)) return;
	
	SetArrayCell(g_hNavMeshAreas, iAreaIndex, g_iNavMeshAreaMasterMarker, NavMeshArea_OpenMarker);
	
	if (g_iNavMeshAreaOpenListIndex == -1)
	{
		g_iNavMeshAreaOpenListIndex = iAreaIndex;
		g_iNavMeshAreaOpenListTailIndex = iAreaIndex;
		SetArrayCell(g_hNavMeshAreas, iAreaIndex, -1, NavMeshArea_PrevOpenIndex);
		SetArrayCell(g_hNavMeshAreas, iAreaIndex, -1, NavMeshArea_NextOpenIndex);
		return;
	}
	
	SetArrayCell(g_hNavMeshAreas, g_iNavMeshAreaOpenListTailIndex, iAreaIndex, NavMeshArea_NextOpenIndex);
	
	SetArrayCell(g_hNavMeshAreas, iAreaIndex, g_iNavMeshAreaOpenListTailIndex, NavMeshArea_PrevOpenIndex);
	SetArrayCell(g_hNavMeshAreas, iAreaIndex, -1, NavMeshArea_NextOpenIndex);
	
	g_iNavMeshAreaOpenListTailIndex = iAreaIndex;
}

void NavMeshAreaUpdateOnOpenList(int iAreaIndex)
{
	int iTotalCost = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_TotalCost);
	
	int iPrevIndex = -1;
	
	while ((iPrevIndex = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_PrevOpenIndex)) != -1 &&
		iTotalCost < (GetArrayCell(g_hNavMeshAreas, iPrevIndex, NavMeshArea_TotalCost)))
	{
		int iOtherIndex = iPrevIndex;
		int iBeforeIndex = GetArrayCell(g_hNavMeshAreas, iPrevIndex, NavMeshArea_PrevOpenIndex);
		int iAfterIndex = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_NextOpenIndex);
	
		SetArrayCell(g_hNavMeshAreas, iAreaIndex, iPrevIndex, NavMeshArea_NextOpenIndex);
		SetArrayCell(g_hNavMeshAreas, iAreaIndex, iBeforeIndex, NavMeshArea_PrevOpenIndex);
		
		SetArrayCell(g_hNavMeshAreas, iOtherIndex, iAreaIndex, NavMeshArea_PrevOpenIndex);
		SetArrayCell(g_hNavMeshAreas, iOtherIndex, iAfterIndex, NavMeshArea_NextOpenIndex);
		
		if (iBeforeIndex != -1)
		{
			SetArrayCell(g_hNavMeshAreas, iBeforeIndex, iAreaIndex, NavMeshArea_NextOpenIndex);
		}
		else
		{
			g_iNavMeshAreaOpenListIndex = iAreaIndex;
		}
		
		if (iAfterIndex != -1)
		{
			SetArrayCell(g_hNavMeshAreas, iAfterIndex, iOtherIndex, NavMeshArea_PrevOpenIndex);
		}
		else
		{
			g_iNavMeshAreaOpenListTailIndex = iAreaIndex;
		}
	}
}

void NavMeshAreaRemoveFromOpenList(int iAreaIndex)
{
	if (GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_OpenMarker) == 0) return;
	
	int iPrevOpenIndex = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_PrevOpenIndex);
	int iNextOpenIndex = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_NextOpenIndex);
	
	if (iPrevOpenIndex != -1)
	{
		SetArrayCell(g_hNavMeshAreas, iPrevOpenIndex, iNextOpenIndex, NavMeshArea_NextOpenIndex);
	}
	else
	{
		g_iNavMeshAreaOpenListIndex = iNextOpenIndex;
	}
	
	if (iNextOpenIndex != -1)
	{
		SetArrayCell(g_hNavMeshAreas, iNextOpenIndex, iPrevOpenIndex, NavMeshArea_PrevOpenIndex);
	}
	else
	{
		g_iNavMeshAreaOpenListTailIndex = iPrevOpenIndex;
	}
	
	SetArrayCell(g_hNavMeshAreas, iAreaIndex, 0, NavMeshArea_OpenMarker);
}

int NavMeshAreaPopOpenList()
{
	if (g_iNavMeshAreaOpenListIndex != -1)
	{
		int iOpenListIndex = g_iNavMeshAreaOpenListIndex;
	
		NavMeshAreaRemoveFromOpenList(iOpenListIndex);
		SetArrayCell(g_hNavMeshAreas, iOpenListIndex, -1, NavMeshArea_PrevOpenIndex);
		SetArrayCell(g_hNavMeshAreas, iOpenListIndex, -1, NavMeshArea_NextOpenIndex);
		
		return iOpenListIndex;
	}
	
	return -1;
}

bool NavMeshAreaIsClosed(int iAreaIndex)
{
	if (NavMeshAreaIsMarked(iAreaIndex) && !NavMeshAreaIsOpen(iAreaIndex)) return true;
	return false;
}

void NavMeshAreaAddToClosedList(int iAreaIndex)
{
	NavMeshAreaMark(iAreaIndex);
}

/*
static NavMeshAreaRemoveFromClosedList(iAreaIndex)
{
}
*/

bool NavMeshLoad(const char[] sMapName)
{
	char sNavFilePath[PLATFORM_MAX_PATH];
	Format(sNavFilePath, sizeof(sNavFilePath), "maps\\%s.nav", sMapName);
	
	Handle hFile = OpenFile(sNavFilePath, "rb");
	bool bVPK = false;
	if (hFile == INVALID_HANDLE)
	{
		hFile = OpenFile(sNavFilePath, "rb", true);
		bVPK = true;
		if (hFile == INVALID_HANDLE)
		{
			EngineVersion iEngineVersion;
			bool bFound = false;
			bVPK = false;
			
			if (GetFeatureStatus(FeatureType_Native, "GetEngineVersion") == FeatureStatus_Available)
			{
				iEngineVersion = GetEngineVersion();
				
				switch (iEngineVersion)
				{
					case Engine_CSGO:
					{
						// Search addon directories.
						Handle hDir = OpenDirectory("addons");
						if (hDir != INVALID_HANDLE)
						{
							LogMessage("Couldn't find .nav file in maps folder, checking addon folders...");
							
							char sFolderName[PLATFORM_MAX_PATH];
							FileType iFileType;
							while (ReadDirEntry(hDir, sFolderName, sizeof(sFolderName), iFileType))
							{
								if (iFileType == FileType_Directory)
								{
									Format(sNavFilePath, sizeof(sNavFilePath), "addons\\%s\\maps\\%s.nav", sFolderName, sMapName);
									hFile = OpenFile(sNavFilePath, "rb");
									if (hFile != INVALID_HANDLE)
									{
										bFound = true;
										break;
									}
								}
							}
							
							CloseHandle(hDir);
						}
					}
					case Engine_TF2:
					{
						// Search custom directories.
						Handle hDir = OpenDirectory("custom");
						if (hDir != INVALID_HANDLE)
						{
							LogMessage("Couldn't find .nav file in maps folder, checking custom folders...");
						
							char sFolderName[PLATFORM_MAX_PATH];
							FileType iFileType;
							while (ReadDirEntry(hDir, sFolderName, sizeof(sFolderName), iFileType))
							{
								if (iFileType == FileType_Directory)
								{
									Format(sNavFilePath, sizeof(sNavFilePath), "custom\\%s\\maps\\%s.nav", sFolderName, sMapName);
									hFile = OpenFile(sNavFilePath, "rb");
									if (hFile != INVALID_HANDLE)
									{
										bFound = true;
										break;
									}
								}
							}
							
							CloseHandle(hDir);
						}
					}
				}
			}
			
			if (!bFound)
			{
				LogMessage(".NAV file for %s could not be found", sMapName);
				return false;
			}
		}
	}
	
	LogMessage("Found .NAV file in %s %s", sNavFilePath, bVPK ? "(VPK)" : "");
	
	// Get magic number.
	int iNavMagicNumber;
	int iElementsRead = ReadFileCell(hFile, iNavMagicNumber, UNSIGNED_INT_BYTE_SIZE);
	
	if (iElementsRead != 1)
	{
		CloseHandle(hFile);
		LogError("Error reading magic number value from navigation mesh: %s", sNavFilePath);
		return false;
	}
	
	if (iNavMagicNumber != NAV_MAGIC_NUMBER)
	{
		CloseHandle(hFile);
		LogError("Invalid magic number value from navigation mesh: %s [%p]", sNavFilePath, iNavMagicNumber);
		return false;
	}
	
	// Get the version.
	int iNavVersion;
	iElementsRead = ReadFileCell(hFile, iNavVersion, UNSIGNED_INT_BYTE_SIZE);
	
	if (iElementsRead != 1)
	{
		CloseHandle(hFile);
		LogError("Error reading version number from navigation mesh: %s", sNavFilePath);
		return false;
	}
	
	if (iNavVersion < 6 || iNavVersion > 16)
	{
		CloseHandle(hFile);
		LogError("Invalid version number value from navigation mesh: %s [%d]", sNavFilePath, iNavVersion);
		return false;
	}
	
	// Get the sub version, if supported.
	int iNavSubVersion;
	if (iNavVersion >= 10)
	{
		ReadFileCell(hFile, iNavSubVersion, UNSIGNED_INT_BYTE_SIZE);
	}
	
	// Get the save bsp size.
	int iNavSaveBspSize;
	if (iNavVersion >= 4)
	{
		ReadFileCell(hFile, iNavSaveBspSize, UNSIGNED_INT_BYTE_SIZE);
	}
	
	// Check if the nav mesh was analyzed.
	int iNavMeshAnalyzed;
	if (iNavVersion >= 14)
	{
		ReadFileCell(hFile, iNavMeshAnalyzed, UNSIGNED_CHAR_BYTE_SIZE);
		LogMessage("Is mesh analyzed: %d", iNavMeshAnalyzed);
	}
	
	LogMessage("Nav version: %d; SubVersion: %d (v10+); BSPSize: %d; MagicNumber: %d", iNavVersion, iNavSubVersion, iNavSaveBspSize, iNavMagicNumber);
	
	int iPlaceCount;
	ReadFileCell(hFile, iPlaceCount, UNSIGNED_SHORT_BYTE_SIZE);
	LogMessage("Place count: %d", iPlaceCount);
	
	// Parse through places.
	// TF2 doesn't use places, but CS:S does.
	for (int iPlaceIndex = 0; iPlaceIndex < iPlaceCount; iPlaceIndex++) 
	{
		int iPlaceStringSize;
		ReadFileCell(hFile, iPlaceStringSize, UNSIGNED_SHORT_BYTE_SIZE);
		
		char sPlaceName[256];
		ReadFileString(hFile, sPlaceName, sizeof(sPlaceName), iPlaceStringSize);
		
		PushArrayString(g_hNavMeshPlaces, sPlaceName);
		
		//LogMessage("Parsed place \"%s\" [index: %d]", sPlaceName, iPlaceIndex);
	}
	
	// Get any unnamed areas.
	int iNavUnnamedAreas;
	if (iNavVersion > 11)
	{
		ReadFileCell(hFile, iNavUnnamedAreas, UNSIGNED_CHAR_BYTE_SIZE);
		LogMessage("Has unnamed areas: %s", iNavUnnamedAreas ? "true" : "false");
	}
	
	// Get area count.
	int iAreaCount;
	ReadFileCell(hFile, iAreaCount, UNSIGNED_INT_BYTE_SIZE);
	
	LogMessage("Area count: %d", iAreaCount);
	
	float flExtentLow[2] = { 99999999.9, 99999999.9 };
	bool bExtentLowX = false;
	bool bExtentLowY = false;
	float flExtentHigh[2] = { -99999999.9, -99999999.9 };
	bool bExtentHighX = false;
	bool bExtentHighY = false;
	
	if (iAreaCount > 0)
	{
		// The following are index values that will serve as starting and ending markers for areas
		// to determine what is theirs.
		
		// This is to avoid iteration of the whole area set to reduce lookup time.
		
		int iGlobalConnectionsStartIndex = 0;
		int iGlobalHidingSpotsStartIndex = 0;
		int iGlobalEncounterPathsStartIndex = 0;
		int iGlobalEncounterSpotsStartIndex = 0;
		int iGlobalLadderConnectionsStartIndex = 0;
		int iGlobalVisibleAreasStartIndex = 0;
		
		for (int iAreaIndex = 0; iAreaIndex < iAreaCount; iAreaIndex++)
		{
			int iAreaID;
			float x1, y1, z1, x2, y2, z2;
			int iAreaFlags;
			int iInheritVisibilityFrom;
			int iHidingSpotCount;
			int iVisibleAreaCount;
			float flEarliestOccupyTimeFirstTeam;
			float flEarliestOccupyTimeSecondTeam;
			float flNECornerZ;
			float flSWCornerZ;
			int iPlaceID;
			int unk01;
			
			ReadFileCell(hFile, iAreaID, UNSIGNED_INT_BYTE_SIZE);
			
			//LogMessage("Area ID: %d", iAreaID);
			
			if (iNavVersion <= 8) 
			{
				ReadFileCell(hFile, iAreaFlags, UNSIGNED_CHAR_BYTE_SIZE);
			}
			else if (iNavVersion < 13) 
			{
				ReadFileCell(hFile, iAreaFlags, UNSIGNED_SHORT_BYTE_SIZE);
			}
			else 
			{
				ReadFileCell(hFile, iAreaFlags, UNSIGNED_INT_BYTE_SIZE);
			}
			
			//LogMessage("Area Flags: %d", iAreaFlags);
			
			ReadFileCell(hFile, view_as<int>(x1), FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, view_as<int>(y1), FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, view_as<int>(z1), FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, view_as<int>(x2), FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, view_as<int>(y2), FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, view_as<int>(z2), FLOAT_BYTE_SIZE);
			
			//LogMessage("Area extent: (%f, %f, %f), (%f, %f, %f)", x1, y1, z1, x2, y2, z2);
			
			if (!bExtentLowX || x1 < flExtentLow[0]) 
			{
				bExtentLowX = true;
				flExtentLow[0] = x1;
			}
			
			if (!bExtentLowY || y1 < flExtentLow[1]) 
			{
				bExtentLowY = true;
				flExtentLow[1] = y1;
			}
			
			if (!bExtentHighX || x2 > flExtentHigh[0]) 
			{
				bExtentHighX = true;
				flExtentHigh[0] = x2;
			}
			
			if (!bExtentHighY || y2 > flExtentHigh[1]) 
			{
				bExtentHighY = true;
				flExtentHigh[1] = y2;
			}
			
			// Cache the center position.
			float flAreaCenter[3];
			flAreaCenter[0] = (x1 + x2) / 2.0;
			flAreaCenter[1] = (y1 + y2) / 2.0;
			flAreaCenter[2] = (z1 + z2) / 2.0;
			
			float flInvDxCorners = 0.0; 
			float flInvDyCorners = 0.0;
			
			if ((x2 - x1) > 0.0 && (y2 - y1) > 0.0)
			{
				flInvDxCorners = 1.0 / (x2 - x1);
				flInvDyCorners = 1.0 / (y2 - y1);
			}
			
			ReadFileCell(hFile, view_as<int>(flNECornerZ), FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, view_as<int>(flSWCornerZ), FLOAT_BYTE_SIZE);
			
			//LogMessage("Corners: NW(%f), SW(%f)", flNECornerZ, flSWCornerZ);
			
			int iConnectionsStartIndex = -1;
			int iConnectionsEndIndex = -1;
			
			// Find connections.
			for (int iDirection = 0; iDirection < NAV_DIR_COUNT; iDirection++)
			{
				int iConnectionCount;
				ReadFileCell(hFile, iConnectionCount, UNSIGNED_INT_BYTE_SIZE);
				
				//LogMessage("Connection count: %d", iConnectionCount);
				
				if (iConnectionCount > 0)
				{
					if (iConnectionsStartIndex == -1) iConnectionsStartIndex = iGlobalConnectionsStartIndex;
				
					for (int iConnectionIndex = 0; iConnectionIndex < iConnectionCount; iConnectionIndex++) 
					{
						iConnectionsEndIndex = iGlobalConnectionsStartIndex;
					
						int iConnectingAreaID;
						ReadFileCell(hFile, iConnectingAreaID, UNSIGNED_INT_BYTE_SIZE);
						
						int iIndex = PushArrayCell(g_hNavMeshAreaConnections, iConnectingAreaID);
						SetArrayCell(g_hNavMeshAreaConnections, iIndex, iDirection, NavMeshConnection_Direction);
						
						iGlobalConnectionsStartIndex++;
					}
				}
			}
			
			// Get hiding spots.
			ReadFileCell(hFile, iHidingSpotCount, UNSIGNED_CHAR_BYTE_SIZE);
			
			//LogMessage("Hiding spot count: %d", iHidingSpotCount);
			
			int iHidingSpotsStartIndex = -1;
			int iHidingSpotsEndIndex = -1;
			
			if (iHidingSpotCount > 0)
			{
				iHidingSpotsStartIndex = iGlobalHidingSpotsStartIndex;
				
				for (int iHidingSpotIndex = 0; iHidingSpotIndex < iHidingSpotCount; iHidingSpotIndex++)
				{
					iHidingSpotsEndIndex = iGlobalHidingSpotsStartIndex;
				
					int iHidingSpotID;
					ReadFileCell(hFile, iHidingSpotID, UNSIGNED_INT_BYTE_SIZE);
					
					float flHidingSpotX, flHidingSpotY, flHidingSpotZ;
					ReadFileCell(hFile, view_as<int>(flHidingSpotX), FLOAT_BYTE_SIZE);
					ReadFileCell(hFile, view_as<int>(flHidingSpotY), FLOAT_BYTE_SIZE);
					ReadFileCell(hFile, view_as<int>(flHidingSpotZ), FLOAT_BYTE_SIZE);
					
					int iHidingSpotFlags;
					ReadFileCell(hFile, iHidingSpotFlags, UNSIGNED_CHAR_BYTE_SIZE);
					
					int iIndex = PushArrayCell(g_hNavMeshAreaHidingSpots, iHidingSpotID);
					SetArrayCell(g_hNavMeshAreaHidingSpots, iIndex, flHidingSpotX, NavMeshHidingSpot_X);
					SetArrayCell(g_hNavMeshAreaHidingSpots, iIndex, flHidingSpotY, NavMeshHidingSpot_Y);
					SetArrayCell(g_hNavMeshAreaHidingSpots, iIndex, flHidingSpotZ, NavMeshHidingSpot_Z);
					SetArrayCell(g_hNavMeshAreaHidingSpots, iIndex, iHidingSpotFlags, NavMeshHidingSpot_Flags);
					
					iGlobalHidingSpotsStartIndex++;
					
					//LogMessage("Parsed hiding spot (%f, %f, %f) with ID [%d] and flags [%d]", flHidingSpotX, flHidingSpotY, flHidingSpotZ, iHidingSpotID, iHidingSpotFlags);
				}
			}
			
			// Get approach areas (old version, only used to read data)
			if (iNavVersion < 15)
			{
				int iApproachAreaCount;
				ReadFileCell(hFile, iApproachAreaCount, UNSIGNED_CHAR_BYTE_SIZE);
				
				for (int iApproachAreaIndex = 0; iApproachAreaIndex < iApproachAreaCount; iApproachAreaIndex++)
				{
					int iApproachHereID;
					ReadFileCell(hFile, iApproachHereID, UNSIGNED_INT_BYTE_SIZE);
					
					int iApproachPrevID;
					ReadFileCell(hFile, iApproachPrevID, UNSIGNED_INT_BYTE_SIZE);
					
					int iApproachType;
					ReadFileCell(hFile, iApproachType, UNSIGNED_CHAR_BYTE_SIZE);
					
					int iApproachNextID;
					ReadFileCell(hFile, iApproachNextID, UNSIGNED_INT_BYTE_SIZE);
					
					int iApproachHow;
					ReadFileCell(hFile, iApproachHow, UNSIGNED_CHAR_BYTE_SIZE);
				}
			}
			
			// Get encounter paths.
			int iEncounterPathCount;
			ReadFileCell(hFile, iEncounterPathCount, UNSIGNED_INT_BYTE_SIZE);
			
			//LogMessage("Encounter Path Count: %d", iEncounterPathCount);
			
			int iEncounterPathsStartIndex = -1;
			int iEncounterPathsEndIndex = -1;
			
			if (iEncounterPathCount > 0)
			{
				iEncounterPathsStartIndex = iGlobalEncounterPathsStartIndex;
			
				for (int iEncounterPathIndex = 0; iEncounterPathIndex < iEncounterPathCount; iEncounterPathIndex++)
				{
					iEncounterPathsEndIndex = iGlobalEncounterPathsStartIndex;
				
					int iEncounterFromID;
					ReadFileCell(hFile, iEncounterFromID, UNSIGNED_INT_BYTE_SIZE);
					
					int iEncounterFromDirection;
					ReadFileCell(hFile, iEncounterFromDirection, UNSIGNED_CHAR_BYTE_SIZE);
					
					int iEncounterToID;
					ReadFileCell(hFile, iEncounterToID, UNSIGNED_INT_BYTE_SIZE);
					
					int iEncounterToDirection;
					ReadFileCell(hFile, iEncounterToDirection, UNSIGNED_CHAR_BYTE_SIZE);
					
					int iEncounterSpotCount;
					ReadFileCell(hFile, iEncounterSpotCount, UNSIGNED_CHAR_BYTE_SIZE);
					
					//LogMessage("Encounter [from ID %d] [from dir %d] [to ID %d] [to dir %d] [spot count %d]", iEncounterFromID, iEncounterFromDirection, iEncounterToID, iEncounterToDirection, iEncounterSpotCount);
					
					int iEncounterSpotsStartIndex = -1;
					int iEncounterSpotsEndIndex = -1;
					
					if (iEncounterSpotCount > 0)
					{
						iEncounterSpotsStartIndex = iGlobalEncounterSpotsStartIndex;
					
						for (int iEncounterSpotIndex = 0; iEncounterSpotIndex < iEncounterSpotCount; iEncounterSpotIndex++)
						{
							iEncounterSpotsEndIndex = iGlobalEncounterSpotsStartIndex;
						
							int iEncounterSpotOrderID;
							ReadFileCell(hFile, iEncounterSpotOrderID, UNSIGNED_INT_BYTE_SIZE);
							
							int iEncounterSpotT;
							ReadFileCell(hFile, iEncounterSpotT, UNSIGNED_CHAR_BYTE_SIZE);
							
							float flEncounterSpotParametricDistance = float(iEncounterSpotT) / 255.0;
							
							int iIndex = PushArrayCell(g_hNavMeshAreaEncounterSpots, iEncounterSpotOrderID);
							SetArrayCell(g_hNavMeshAreaEncounterSpots, iIndex, flEncounterSpotParametricDistance, NavMeshEncounterSpot_ParametricDistance);
							
							iGlobalEncounterSpotsStartIndex++;
							
							//LogMessage("Encounter spot [order id %d] and [T %d]", iEncounterSpotOrderID, iEncounterSpotT);
						}
					}
					
					int iIndex = PushArrayCell(g_hNavMeshAreaEncounterPaths, iEncounterFromID);
					SetArrayCell(g_hNavMeshAreaEncounterPaths, iIndex, iEncounterFromDirection, NavMeshEncounterPath_FromDirection);
					SetArrayCell(g_hNavMeshAreaEncounterPaths, iIndex, iEncounterToID, NavMeshEncounterPath_ToID);
					SetArrayCell(g_hNavMeshAreaEncounterPaths, iIndex, iEncounterToDirection, NavMeshEncounterPath_ToDirection);
					SetArrayCell(g_hNavMeshAreaEncounterPaths, iIndex, iEncounterSpotsStartIndex, NavMeshEncounterPath_SpotsStartIndex);
					SetArrayCell(g_hNavMeshAreaEncounterPaths, iIndex, iEncounterSpotsEndIndex, NavMeshEncounterPath_SpotsEndIndex);
					
					iGlobalEncounterPathsStartIndex++;
				}
			}
			
			ReadFileCell(hFile, iPlaceID, UNSIGNED_SHORT_BYTE_SIZE);
			
			//LogMessage("Place ID: %d", iPlaceID);
			
			// Get ladder connections.
			
			int iLadderConnectionsStartIndex = -1;
			int iLadderConnectionsEndIndex = -1;
			
			for (int iLadderDirection = 0; iLadderDirection < NAV_LADDER_DIR_COUNT; iLadderDirection++)
			{
				int iLadderConnectionCount;
				ReadFileCell(hFile, iLadderConnectionCount, UNSIGNED_INT_BYTE_SIZE);
				
				//LogMessage("Ladder Connection Count: %d", iLadderConnectionCount);
				
				if (iLadderConnectionCount > 0)
				{
					iLadderConnectionsStartIndex = iGlobalLadderConnectionsStartIndex;
				
					for (int iLadderConnectionIndex = 0; iLadderConnectionIndex < iLadderConnectionCount; iLadderConnectionIndex++)
					{
						iLadderConnectionsEndIndex = iGlobalLadderConnectionsStartIndex;
					
						int iLadderConnectionID;
						ReadFileCell(hFile, iLadderConnectionID, UNSIGNED_INT_BYTE_SIZE);
						
						int iIndex = PushArrayCell(g_hNavMeshAreaLadderConnections, iLadderConnectionID);
						SetArrayCell(g_hNavMeshAreaLadderConnections, iIndex, iLadderDirection, NavMeshLadderConnection_Direction);
						
						iGlobalLadderConnectionsStartIndex++;
						
						//LogMessage("Parsed ladder connect [ID %d]\n", iLadderConnectionID);
					}
				}
			}
			
			ReadFileCell(hFile, view_as<int>(flEarliestOccupyTimeFirstTeam), FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, view_as<int>(flEarliestOccupyTimeSecondTeam), FLOAT_BYTE_SIZE);
			
			float flNavCornerLightIntensityNW;
			float flNavCornerLightIntensityNE;
			float flNavCornerLightIntensitySE;
			float flNavCornerLightIntensitySW;
			
			int iVisibleAreasStartIndex = -1;
			int iVisibleAreasEndIndex = -1;
			
			if (iNavVersion >= 11)
			{
				ReadFileCell(hFile, view_as<int>(flNavCornerLightIntensityNW), FLOAT_BYTE_SIZE);
				ReadFileCell(hFile, view_as<int>(flNavCornerLightIntensityNE), FLOAT_BYTE_SIZE);
				ReadFileCell(hFile, view_as<int>(flNavCornerLightIntensitySE), FLOAT_BYTE_SIZE);
				ReadFileCell(hFile, view_as<int>(flNavCornerLightIntensitySW), FLOAT_BYTE_SIZE);
				
				if (iNavVersion >= 16)
				{
					ReadFileCell(hFile, iVisibleAreaCount, UNSIGNED_INT_BYTE_SIZE);
					
					//LogMessage("Visible area count: %d", iVisibleAreaCount);
					
					if (iVisibleAreaCount > 0)
					{
						iVisibleAreasStartIndex = iGlobalVisibleAreasStartIndex;
					
						for (int iVisibleAreaIndex = 0; iVisibleAreaIndex < iVisibleAreaCount; iVisibleAreaIndex++)
						{
							iVisibleAreasEndIndex = iGlobalVisibleAreasStartIndex;
						
							int iVisibleAreaID;
							ReadFileCell(hFile, iVisibleAreaID, UNSIGNED_INT_BYTE_SIZE);
							
							int iVisibleAreaAttributes;
							ReadFileCell(hFile, iVisibleAreaAttributes, UNSIGNED_CHAR_BYTE_SIZE);
							
							int iIndex = PushArrayCell(g_hNavMeshAreaVisibleAreas, iVisibleAreaID);
							SetArrayCell(g_hNavMeshAreaVisibleAreas, iIndex, iVisibleAreaAttributes, NavMeshVisibleArea_Attributes);
							
							iGlobalVisibleAreasStartIndex++;
							
							//LogMessage("Parsed visible area [%d] with attr [%d]", iVisibleAreaID, iVisibleAreaAttributes);
						}
					}
					
					ReadFileCell(hFile, iInheritVisibilityFrom, UNSIGNED_INT_BYTE_SIZE);
					
					//LogMessage("Inherit visibilty from: %d", iInheritVisibilityFrom);
					
					ReadFileCell(hFile, unk01, UNSIGNED_INT_BYTE_SIZE);
				}
			}
			
			int iIndex = PushArrayCell(g_hNavMeshAreas, iAreaID);
			SetArrayCell(g_hNavMeshAreas, iIndex, iAreaFlags, NavMeshArea_Flags);
			SetArrayCell(g_hNavMeshAreas, iIndex, iPlaceID, NavMeshArea_PlaceID);
			SetArrayCell(g_hNavMeshAreas, iIndex, x1, NavMeshArea_X1);
			SetArrayCell(g_hNavMeshAreas, iIndex, y1, NavMeshArea_Y1);
			SetArrayCell(g_hNavMeshAreas, iIndex, z1, NavMeshArea_Z1);
			SetArrayCell(g_hNavMeshAreas, iIndex, x2, NavMeshArea_X2);
			SetArrayCell(g_hNavMeshAreas, iIndex, y2, NavMeshArea_Y2);
			SetArrayCell(g_hNavMeshAreas, iIndex, z2, NavMeshArea_Z2);
			SetArrayCell(g_hNavMeshAreas, iIndex, flAreaCenter[0], NavMeshArea_CenterX);
			SetArrayCell(g_hNavMeshAreas, iIndex, flAreaCenter[1], NavMeshArea_CenterY);
			SetArrayCell(g_hNavMeshAreas, iIndex, flAreaCenter[2], NavMeshArea_CenterZ);
			SetArrayCell(g_hNavMeshAreas, iIndex, flInvDxCorners, NavMeshArea_InvDxCorners);
			SetArrayCell(g_hNavMeshAreas, iIndex, flInvDyCorners, NavMeshArea_InvDyCorners);
			SetArrayCell(g_hNavMeshAreas, iIndex, flNECornerZ, NavMeshArea_NECornerZ);
			SetArrayCell(g_hNavMeshAreas, iIndex, flSWCornerZ, NavMeshArea_SWCornerZ);
			SetArrayCell(g_hNavMeshAreas, iIndex, iConnectionsStartIndex, NavMeshArea_ConnectionsStartIndex);
			SetArrayCell(g_hNavMeshAreas, iIndex, iConnectionsEndIndex, NavMeshArea_ConnectionsEndIndex);
			SetArrayCell(g_hNavMeshAreas, iIndex, iHidingSpotsStartIndex, NavMeshArea_HidingSpotsStartIndex);
			SetArrayCell(g_hNavMeshAreas, iIndex, iHidingSpotsEndIndex, NavMeshArea_HidingSpotsEndIndex);
			SetArrayCell(g_hNavMeshAreas, iIndex, iEncounterPathsStartIndex, NavMeshArea_EncounterPathsStartIndex);
			SetArrayCell(g_hNavMeshAreas, iIndex, iEncounterPathsEndIndex, NavMeshArea_EncounterPathsEndIndex);
			SetArrayCell(g_hNavMeshAreas, iIndex, iLadderConnectionsStartIndex, NavMeshArea_LadderConnectionsStartIndex);
			SetArrayCell(g_hNavMeshAreas, iIndex, iLadderConnectionsEndIndex, NavMeshArea_LadderConnectionsEndIndex);
			SetArrayCell(g_hNavMeshAreas, iIndex, flNavCornerLightIntensityNW, NavMeshArea_CornerLightIntensityNW);
			SetArrayCell(g_hNavMeshAreas, iIndex, flNavCornerLightIntensityNE, NavMeshArea_CornerLightIntensityNE);
			SetArrayCell(g_hNavMeshAreas, iIndex, flNavCornerLightIntensitySE, NavMeshArea_CornerLightIntensitySE);
			SetArrayCell(g_hNavMeshAreas, iIndex, flNavCornerLightIntensitySW, NavMeshArea_CornerLightIntensitySW);
			SetArrayCell(g_hNavMeshAreas, iIndex, iVisibleAreasStartIndex, NavMeshArea_VisibleAreasStartIndex);
			SetArrayCell(g_hNavMeshAreas, iIndex, iVisibleAreasEndIndex, NavMeshArea_VisibleAreasEndIndex);
			SetArrayCell(g_hNavMeshAreas, iIndex, iInheritVisibilityFrom, NavMeshArea_InheritVisibilityFrom);
			SetArrayCell(g_hNavMeshAreas, iIndex, flEarliestOccupyTimeFirstTeam, NavMeshArea_EarliestOccupyTimeFirstTeam);
			SetArrayCell(g_hNavMeshAreas, iIndex, flEarliestOccupyTimeSecondTeam, NavMeshArea_EarliestOccupyTimeSecondTeam);
			SetArrayCell(g_hNavMeshAreas, iIndex, unk01, NavMeshArea_unk01);
			SetArrayCell(g_hNavMeshAreas, iIndex, -1, NavMeshArea_Parent);
			SetArrayCell(g_hNavMeshAreas, iIndex, NUM_TRAVERSE_TYPES, NavMeshArea_ParentHow);
			SetArrayCell(g_hNavMeshAreas, iIndex, 0, NavMeshArea_TotalCost);
			SetArrayCell(g_hNavMeshAreas, iIndex, 0, NavMeshArea_CostSoFar);
			SetArrayCell(g_hNavMeshAreas, iIndex, -1, NavMeshArea_Marker);
			SetArrayCell(g_hNavMeshAreas, iIndex, -1, NavMeshArea_OpenMarker);
			SetArrayCell(g_hNavMeshAreas, iIndex, -1, NavMeshArea_PrevOpenIndex);
			SetArrayCell(g_hNavMeshAreas, iIndex, -1, NavMeshArea_NextOpenIndex);
			SetArrayCell(g_hNavMeshAreas, iIndex, 0.0, NavMeshArea_PathLengthSoFar);
			SetArrayCell(g_hNavMeshAreas, iIndex, false, NavMeshArea_Blocked);
			SetArrayCell(g_hNavMeshAreas, iIndex, -1, NavMeshArea_NearSearchMarker);
		}
	}
	
	// Set up the grid.
	NavMeshGridAllocate(flExtentLow[0], flExtentHigh[0], flExtentLow[1], flExtentHigh[1]);
	
	for (int i = 0; i < iAreaCount; i++)
	{
		NavMeshAddAreaToGrid(i);
	}
	
	NavMeshGridFinalize();
	
	int iLadderCount;
	ReadFileCell(hFile, iLadderCount, UNSIGNED_INT_BYTE_SIZE);
	
	if (iLadderCount > 0)
	{
		for (int iLadderIndex; iLadderIndex < iLadderCount; iLadderIndex++)
		{
			int iLadderID;
			ReadFileCell(hFile, iLadderID, UNSIGNED_INT_BYTE_SIZE);
			
			float flLadderWidth;
			ReadFileCell(hFile, view_as<int>(flLadderWidth), FLOAT_BYTE_SIZE);
			
			float flLadderTopX, flLadderTopY, flLadderTopZ, flLadderBottomX, flLadderBottomY, flLadderBottomZ;
			ReadFileCell(hFile, view_as<int>(flLadderTopX), FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, view_as<int>(flLadderTopY), FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, view_as<int>(flLadderTopZ), FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, view_as<int>(flLadderBottomX), FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, view_as<int>(flLadderBottomY), FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, view_as<int>(flLadderBottomZ), FLOAT_BYTE_SIZE);
			
			float flLadderLength;
			ReadFileCell(hFile, view_as<int>(flLadderLength), FLOAT_BYTE_SIZE);
			
			int iLadderDirection;
			ReadFileCell(hFile, iLadderDirection, UNSIGNED_INT_BYTE_SIZE);
			
			int iLadderTopForwardAreaID;
			ReadFileCell(hFile, iLadderTopForwardAreaID, UNSIGNED_INT_BYTE_SIZE);
			
			int iLadderTopLeftAreaID;
			ReadFileCell(hFile, iLadderTopLeftAreaID, UNSIGNED_INT_BYTE_SIZE);
			
			int iLadderTopRightAreaID;
			ReadFileCell(hFile, iLadderTopRightAreaID, UNSIGNED_INT_BYTE_SIZE);
			
			int iLadderTopBehindAreaID;
			ReadFileCell(hFile, iLadderTopBehindAreaID, UNSIGNED_INT_BYTE_SIZE);
			
			int iLadderBottomAreaID;
			ReadFileCell(hFile, iLadderBottomAreaID, UNSIGNED_INT_BYTE_SIZE);
			
			int iIndex = PushArrayCell(g_hNavMeshLadders, iLadderID);
			SetArrayCell(g_hNavMeshLadders, iIndex, flLadderWidth, NavMeshLadder_Width);
			SetArrayCell(g_hNavMeshLadders, iIndex, flLadderLength, NavMeshLadder_Length);
			SetArrayCell(g_hNavMeshLadders, iIndex, flLadderTopX, NavMeshLadder_TopX);
			SetArrayCell(g_hNavMeshLadders, iIndex, flLadderTopY, NavMeshLadder_TopY);
			SetArrayCell(g_hNavMeshLadders, iIndex, flLadderTopZ, NavMeshLadder_TopZ);
			SetArrayCell(g_hNavMeshLadders, iIndex, flLadderBottomX, NavMeshLadder_BottomX);
			SetArrayCell(g_hNavMeshLadders, iIndex, flLadderBottomY, NavMeshLadder_BottomY);
			SetArrayCell(g_hNavMeshLadders, iIndex, flLadderBottomZ, NavMeshLadder_BottomZ);
			SetArrayCell(g_hNavMeshLadders, iIndex, iLadderDirection, NavMeshLadder_Direction);
			SetArrayCell(g_hNavMeshLadders, iIndex, iLadderTopForwardAreaID, NavMeshLadder_TopForwardAreaIndex);
			SetArrayCell(g_hNavMeshLadders, iIndex, iLadderTopLeftAreaID, NavMeshLadder_TopLeftAreaIndex);
			SetArrayCell(g_hNavMeshLadders, iIndex, iLadderTopRightAreaID, NavMeshLadder_TopRightAreaIndex);
			SetArrayCell(g_hNavMeshLadders, iIndex, iLadderTopBehindAreaID, NavMeshLadder_TopBehindAreaIndex);
			SetArrayCell(g_hNavMeshLadders, iIndex, iLadderBottomAreaID, NavMeshLadder_BottomAreaIndex);
		}
	}
	
	g_iNavMeshMagicNumber = iNavMagicNumber;
	g_iNavMeshVersion = iNavVersion;
	g_iNavMeshSubVersion = iNavSubVersion;
	g_iNavMeshSaveBSPSize = iNavSaveBspSize;
	g_bNavMeshAnalyzed = view_as<bool>(iNavMeshAnalyzed);
	
	CloseHandle(hFile);
	
	// File parsing is all done. Convert IDs to array indexes for faster performance and 
	// lesser lookup time.
	
	if (GetArraySize(g_hNavMeshAreaConnections) > 0)
	{
		for (int iIndex = 0, iSize = GetArraySize(g_hNavMeshAreaConnections); iIndex < iSize; iIndex++)
		{
			int iConnectedAreaID = GetArrayCell(g_hNavMeshAreaConnections, iIndex, NavMeshConnection_AreaIndex);
			SetArrayCell(g_hNavMeshAreaConnections, iIndex, FindValueInArray(g_hNavMeshAreas, iConnectedAreaID), NavMeshConnection_AreaIndex);
		}
	}
	
	if (GetArraySize(g_hNavMeshAreaVisibleAreas) > 0)
	{
		for (int iIndex = 0, iSize = GetArraySize(g_hNavMeshAreaVisibleAreas); iIndex < iSize; iIndex++)
		{
			int iVisibleAreaID = GetArrayCell(g_hNavMeshAreaVisibleAreas, iIndex, NavMeshVisibleArea_Index);
			SetArrayCell(g_hNavMeshAreaVisibleAreas, iIndex, FindValueInArray(g_hNavMeshAreas, iVisibleAreaID), NavMeshVisibleArea_Index);
		}
	}
	
	if (GetArraySize(g_hNavMeshAreaLadderConnections) > 0)
	{
		for (int iIndex = 0, iSize = GetArraySize(g_hNavMeshAreaLadderConnections); iIndex < iSize; iIndex++)
		{
			int iLadderID = GetArrayCell(g_hNavMeshAreaLadderConnections, iIndex, NavMeshLadderConnection_LadderIndex);
			SetArrayCell(g_hNavMeshAreaLadderConnections, iIndex, FindValueInArray(g_hNavMeshLadders, iLadderID), NavMeshLadderConnection_LadderIndex);
		}
	}
	
	if (GetArraySize(g_hNavMeshLadders) > 0)
	{
		for (int iLadderIndex = 0; iLadderIndex < iLadderCount; iLadderIndex++)
		{
			int iTopForwardAreaID = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_TopForwardAreaIndex);
			SetArrayCell(g_hNavMeshLadders, iLadderIndex, FindValueInArray(g_hNavMeshAreas, iTopForwardAreaID), NavMeshLadder_TopForwardAreaIndex);
			
			int iTopLeftAreaID = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_TopLeftAreaIndex);
			SetArrayCell(g_hNavMeshLadders, iLadderIndex, FindValueInArray(g_hNavMeshAreas, iTopLeftAreaID), NavMeshLadder_TopLeftAreaIndex);
			
			int iTopRightAreaID = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_TopRightAreaIndex);
			SetArrayCell(g_hNavMeshLadders, iLadderIndex, FindValueInArray(g_hNavMeshAreas, iTopRightAreaID), NavMeshLadder_TopRightAreaIndex);
			
			int iTopBehindAreaID = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_TopBehindAreaIndex);
			SetArrayCell(g_hNavMeshLadders, iLadderIndex, FindValueInArray(g_hNavMeshAreas, iTopBehindAreaID), NavMeshLadder_TopBehindAreaIndex);
			
			int iBottomAreaID = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_BottomAreaIndex);
			SetArrayCell(g_hNavMeshLadders, iLadderIndex, FindValueInArray(g_hNavMeshAreas, iBottomAreaID), NavMeshLadder_BottomAreaIndex);
		}
	}
	
	return true;
}

void NavMeshDestroy()
{
	ClearArray(g_hNavMeshPlaces);
	ClearArray(g_hNavMeshAreas);
	ClearArray(g_hNavMeshAreaConnections);
	ClearArray(g_hNavMeshAreaHidingSpots);
	ClearArray(g_hNavMeshAreaEncounterPaths);
	ClearArray(g_hNavMeshAreaEncounterSpots);
	ClearArray(g_hNavMeshAreaLadderConnections);
	ClearArray(g_hNavMeshAreaVisibleAreas);
	ClearArray(g_hNavMeshLadders);
	
	ClearArray(g_hNavMeshGrid);
	ClearArray(g_hNavMeshGridLists);
	
	g_iNavMeshMagicNumber = 0;
	g_iNavMeshVersion = 0;
	g_iNavMeshSubVersion = 0;
	g_iNavMeshSaveBSPSize = 0;
	g_bNavMeshAnalyzed = false;
	
	g_bNavMeshBuilt = false;
	
	g_iNavMeshAreaOpenListIndex = -1;
	g_iNavMeshAreaOpenListTailIndex = -1;
	g_iNavMeshAreaMasterMarker = 0;
}

void NavMeshGridAllocate(float flMinX, float flMaxX, float flMinY, float flMaxY)
{
	ClearArray(g_hNavMeshGrid);
	ClearArray(g_hNavMeshGridLists);
	
	g_flNavMeshMinX = flMinX;
	g_flNavMeshMinY = flMinY;
	
	g_iNavMeshGridSizeX = IntCast((flMaxX - flMinX) / g_flNavMeshGridCellSize) + 1;
	g_iNavMeshGridSizeY = IntCast((flMaxY - flMinY) / g_flNavMeshGridCellSize) + 1;
	
	int iArraySize = g_iNavMeshGridSizeX * g_iNavMeshGridSizeY;
	ResizeArray(g_hNavMeshGrid, iArraySize);
	
	for (int iGridIndex = 0; iGridIndex < iArraySize; iGridIndex++)
	{
		SetArrayCell(g_hNavMeshGrid, iGridIndex, -1, NavMeshGrid_ListStartIndex);
		SetArrayCell(g_hNavMeshGrid, iGridIndex, -1, NavMeshGrid_ListEndIndex);
	}
}

void NavMeshGridFinalize()
{
	//int iAreaCount = GetArraySize(g_hNavMeshAreas);
	//bool[] bAreaInGrid = new bool[iAreaCount];
	//Apparently the lines above make the server goes crazy
	bool bAllIn = true;
	int iErrorAreaIndex = -1;
	
	SortADTArrayCustom(g_hNavMeshGridLists, SortNavMeshGridLists);
	
	for (int iGridIndex = 0, iSize = GetArraySize(g_hNavMeshGrid); iGridIndex < iSize; iGridIndex++)
	{
		int iStartIndex = -1;
		int iEndIndex = -1;
		NavMeshGridGetListBounds(iGridIndex, iStartIndex, iEndIndex);
		SetArrayCell(g_hNavMeshGrid, iGridIndex, iStartIndex, NavMeshGrid_ListStartIndex);
		SetArrayCell(g_hNavMeshGrid, iGridIndex, iEndIndex, NavMeshGrid_ListEndIndex);
		
		if (iStartIndex != -1)
		{
			for (int iListIndex = iStartIndex; iListIndex <= iEndIndex; iListIndex++)
			{
				int iAreaIndex = GetArrayCell(g_hNavMeshGridLists, iListIndex);
				if (iAreaIndex != -1)
				{
					
				}
				else
				{
					if(iErrorAreaIndex==-1)
						iErrorAreaIndex = iAreaIndex;
					bAllIn = false;
					LogError("Warning! Invalid nav area found in list of grid index %d!", iGridIndex);
				}
			}
		}
	}
	if (bAllIn)
	{
		LogMessage("All nav areas parsed into the grid!");
	}
	else
	{
		LogError("Warning! Not all nav areas were parsed into the grid! Please check your nav mesh!");
		LogError("First encountered nav area ID %d not in the grid!", GetArrayCell(g_hNavMeshAreas, iErrorAreaIndex));
	}
}

// The following functions should ONLY be called during NavMeshLoad(), due to displacement of
// array indexes!

// Some things to take into account: because we're adding things into the
// array, it's inevitable that the indexes will change over time. Therefore,
// we can't assign array indexes while this function is running, since it
// will shift preceding array indexes.

// The array indexes should be assigned afterwards using NavMeshGridFinalize().

public int SortNavMeshGridLists(int index1,int index2, Handle array, Handle hndl)
{
	int iGridIndex1 = GetArrayCell(array, index1, NavMeshGridList_Owner);
	int iGridIndex2 = GetArrayCell(array, index2, NavMeshGridList_Owner);
	
	if (iGridIndex1 < iGridIndex2) return -1;
	else if (iGridIndex1 > iGridIndex2) return 1;
	return 0;
}

void NavMeshGridAddAreaToList(int iGridIndex,int iAreaIndex)
{
	int iIndex = PushArrayCell(g_hNavMeshGridLists, iAreaIndex);
	
	if (iIndex != -1)
	{
		SetArrayCell(g_hNavMeshGridLists, iIndex, iGridIndex, NavMeshGridList_Owner);
	}
}

void NavMeshGridGetListBounds(int iGridIndex,int &iStartIndex,int &iEndIndex)
{
	iStartIndex = -1;
	iEndIndex = -1;
	
	for (int i = 0, iSize = GetArraySize(g_hNavMeshGridLists); i < iSize; i++)
	{
		if (GetArrayCell(g_hNavMeshGridLists, i, NavMeshGridList_Owner) == iGridIndex)
		{
			if (iStartIndex == -1) iStartIndex = i;
			iEndIndex = i;
		}
	}
}

void NavMeshAddAreaToGrid(int iAreaIndex)
{
	float flExtentLow[2], flExtentHigh[2];
//	NavMeshAreaGetExtentLow(iAreaIndex, flExtentLow);
//	NavMeshAreaGetExtentHigh(iAreaIndex, flExtentHigh);
	
	flExtentLow[0] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_X1));
	flExtentLow[1] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Y1));
	flExtentHigh[0] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_X2));
	flExtentHigh[1] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Y2));
	
	int loX = NavMeshWorldToGridX(flExtentLow[0]);
	int loY = NavMeshWorldToGridY(flExtentLow[1]);
	int hiX = NavMeshWorldToGridX(flExtentHigh[0]);
	int hiY = NavMeshWorldToGridY(flExtentHigh[1]);
	
	/*
	char path[PLATFORM_MAX_PATH];
	BuildPath(Path_SM,path,PLATFORM_MAX_PATH, "navtest.txt");
	Handle hFile = OpenFile(path, "a");
	
	WriteFileLine(hFile, "[%d]", GetArrayCell(g_hNavMeshAreas, iAreaIndex));
	WriteFileLine(hFile, "{");
	WriteFileLine(hFile, "\t---Extent: (%f, %f) - (%f, %f)", flExtentLow[0], flExtentLow[1], flExtentHigh[0], flExtentHigh[1]);
	*/
	
	for (int y = loY; y <= hiY; ++y)
	{
		//WriteFileLine(hFile, "\t--- y = %d", y);
	
		for (int x = loX; x <= hiX; ++x)
		{
			//WriteFileLine(hFile, "\t\t--- x = %d", x);
		
			int iGridIndex = x + y * g_iNavMeshGridSizeX;
			NavMeshGridAddAreaToList(iGridIndex, iAreaIndex);
			
			//WriteFileLine(hFile, "\t\t\t--- %d", iGridIndex);
		}
	}
	
	/*
	WriteFileLine(hFile, "}");
	CloseHandle(hFile);
	*/
}

// The following functions are stock functions associated with the navmesh grid. These
// are safe to use after the grid has been finalized using NavMeshGridFinalize(), and
// can be included in other stock functions as well.

stock int IntCast(float val)
{
	if (val < 0.0) return RoundToFloor(val);
	return RoundToCeil(val);
}

stock int NavMeshWorldToGridX(float flWX)
{
	int x = IntCast((flWX - g_flNavMeshMinX) / g_flNavMeshGridCellSize);
	
	if (x < 0) x = 0;
	else if (x >= g_iNavMeshGridSizeX) 
	{
	//	PrintToServer("NavMeshWorldToGridX: clamping x (%d) down to %d", x, g_iNavMeshGridSizeX - 1);
		x = g_iNavMeshGridSizeX - 1;
	}
	
	return x;
}

stock int NavMeshWorldToGridY(float flWY)
{
	int y = IntCast((flWY - g_flNavMeshMinY) / g_flNavMeshGridCellSize);
	
	if (y < 0) y = 0;
	else if (y >= g_iNavMeshGridSizeY) 
	{
	//	PrintToServer("NavMeshWorldToGridY: clamping y (%d) down to %d", y, g_iNavMeshGridSizeY - 1);
		y = g_iNavMeshGridSizeY - 1;
	}
	
	return y;
}

stock Handle NavMeshGridGetAreas(int x,int y)
{
	int iGridIndex = x + y * g_iNavMeshGridSizeX;
	int iListStartIndex = GetArrayCell(g_hNavMeshGrid, iGridIndex, NavMeshGrid_ListStartIndex);
	int iListEndIndex = GetArrayCell(g_hNavMeshGrid, iGridIndex, NavMeshGrid_ListEndIndex);
	
	if (iListStartIndex == -1) return INVALID_HANDLE;
	
	Handle hStack = CreateStack();
	
	for (int i = iListStartIndex; i <= iListEndIndex; i++)
	{
		PushStackCell(hStack, GetArrayCell(g_hNavMeshGridLists, i, NavMeshGridList_AreaIndex));
	}
	
	return hStack;
}

stock int NavMeshGetNearestArea(const float flPos[3], bool bAnyZ=false, float flMaxDist=10000.0, bool bCheckLOS=false, bool bCheckGround=true,int iTeam=-2)
{
	if (GetArraySize(g_hNavMeshGridLists) == 0) return -1;
	
	int iClosestAreaIndex = -1;
	float flClosestDistSq = flMaxDist * flMaxDist;
	
	if (!bCheckLOS && !bCheckGround)
	{
		iClosestAreaIndex = NavMeshGetArea(flPos);
		if (iClosestAreaIndex != -1) return iClosestAreaIndex;
	}
	
	float flSource[3];
	flSource[0] = flPos[0];
	flSource[1] = flPos[1];
	
	float flNormal[3];
	
	if (!NavMeshGetGroundHeight(flPos, flSource[2], flNormal))
	{
		if (!bCheckGround)
		{
			flSource[2] = flPos[2];
		}
		else
		{
			return -1;
		}
	}
	
	flSource[2] += HalfHumanHeight;
	
	static int iSearchMarker = -1;
	if (iSearchMarker == -1) iSearchMarker = GetRandomInt(0, 1024 * 1024);
	
	iSearchMarker++;
	if (iSearchMarker == 0) iSearchMarker++;
	
	int iOriginX = NavMeshWorldToGridX(flPos[0]);
	int iOriginY = NavMeshWorldToGridY(flPos[1]);
	
	int iShiftLimit = RoundToCeil(flMaxDist / g_flNavMeshGridCellSize);
	
	for (int iShift = 0; iShift <= iShiftLimit; ++iShift)
	{
		for (int x = (iOriginX - iShift); x <= (iOriginX + iShift); ++x)
		{
			if (x < 0 || x >= g_iNavMeshGridSizeX) continue;
			
			for (int y = (iOriginY - iShift); y <= (iOriginY + iShift); ++y)
			{
				if (y < 0 || y >= g_iNavMeshGridSizeY) continue;
				
				if (x > (iOriginX - iShift) &&
					x < (iOriginX + iShift) &&
					y > (iOriginY - iShift) &&
					y < (iOriginY + iShift))
				{
					continue;
				}
				
				Handle hAreas = NavMeshGridGetAreas(x, y);
				if (hAreas != INVALID_HANDLE)
				{
					while (!IsStackEmpty(hAreas))
					{
						int iAreaIndex = -1;
						PopStackCell(hAreas, iAreaIndex);
						
						int iAreaNearSearchMarker = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_NearSearchMarker);
						if (iAreaNearSearchMarker == iSearchMarker) continue;
						
						if (GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Blocked)) 
						{
							continue;
						}
						
						SetArrayCell(g_hNavMeshAreas, iAreaIndex, iSearchMarker, NavMeshArea_NearSearchMarker);
						
						float flAreaPos[3];
						NavMeshAreaGetClosestPointOnArea(iAreaIndex, flSource, flAreaPos);
						
						float flDistSq = Pow(GetVectorDistance(flPos, flAreaPos), 2.0);
						
						if (flDistSq >= flClosestDistSq) continue;
						
						if (bCheckLOS)
						{
							float flSafePos[3];
							float flStartPos[3];
							float flEndPos[3];
							flEndPos[0] = flPos[0];
							flEndPos[1] = flPos[1];
							flEndPos[2] = flPos[2] + StepHeight;
							
							Handle hTrace = TR_TraceRayFilterEx(flPos, flEndPos, MASK_NPCSOLID_BRUSHONLY, RayType_EndPoint, TraceRayIgnoreCustom);
							float flFraction = TR_GetFraction(hTrace);
							TR_GetEndPosition(flEndPos, hTrace);
							CloseHandle(hTrace);
							
							if (flFraction == 0.0)
							{
								flSafePos[0] = flEndPos[0];
								flSafePos[1] = flEndPos[1];
								flSafePos[2] = flEndPos[2] + 1.0;
							}
							else
							{
								flSafePos[0] = flPos[0];
								flSafePos[1] = flPos[1];
								flSafePos[2] = flPos[2];
							}
							
							float flHeightDelta = FloatAbs(flAreaPos[2] - flSafePos[2]);
							if (flHeightDelta > StepHeight)
							{
								flStartPos[0] = flAreaPos[0];
								flStartPos[1] = flAreaPos[1];
								flStartPos[2] = flAreaPos[2] + StepHeight;
								
								flEndPos[0] = flAreaPos[0];
								flEndPos[1] = flAreaPos[1];
								flEndPos[2] = flSafePos[2];
								
								hTrace = TR_TraceRayFilterEx(flStartPos, flEndPos, MASK_NPCSOLID_BRUSHONLY, RayType_EndPoint, TraceRayIgnoreCustom);
								flFraction = TR_GetFraction(hTrace);
								CloseHandle(hTrace);
								
								if (flFraction != 1.0)
								{
									continue;
								}
							}
							
							flEndPos[0] = flAreaPos[0];
							flEndPos[1] = flAreaPos[1];
							flEndPos[2] = flSafePos[2] + StepHeight;
							
							hTrace = TR_TraceRayFilterEx(flSafePos, flEndPos, MASK_NPCSOLID_BRUSHONLY, RayType_EndPoint, TraceRayIgnoreCustom);
							flFraction = TR_GetFraction(hTrace);
							CloseHandle(hTrace);
							
							if (flFraction != 1.0)
							{
								continue;
							}
						}
						
						flClosestDistSq = flDistSq;
						iClosestAreaIndex = iAreaIndex;
						
						iShiftLimit = iShift + 1;
					}
					
					CloseHandle(hAreas);
				}
			}
		}
	}
	
	return iClosestAreaIndex;
}

stock void NavMeshAreaGetClosestPointOnArea(int iAreaIndex, const float flPos[3], float flClose[3])
{
	float x, y, z;
	
	float flExtentLow[3], flExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flExtentHigh);
	
	x = fsel(flPos[0] - flExtentLow[0], flPos[0], flExtentLow[0]);
	x = fsel(x - flExtentHigh[0], flExtentHigh[0], x);
	
	y = fsel(flPos[1] - flExtentLow[1], flPos[1], flExtentLow[1]);
	y = fsel(y - flExtentHigh[1], flExtentHigh[1], y);
	
	z = NavMeshAreaGetZFromXAndY(iAreaIndex, x, y);
	
	flClose[0] = x;
	flClose[1] = y;
	flClose[2] = z;
}

stock float fsel(float a, float b, float c)
{
	return a >= 0.0 ? b : c;
}

stock int NavMeshAreaGetFlags(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return 0;
	
	return GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Flags);
}

stock bool NavMeshAreaGetCenter(int iAreaIndex, float flBuffer[3])
{
	if (!g_bNavMeshBuilt) return false;
	
	flBuffer[0] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_CenterX));
	flBuffer[1] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_CenterY));
	flBuffer[2] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_CenterZ));
	return true;
}

stock Handle NavMeshAreaGetAdjacentList(int iAreaIndex,int iNavDirection)
{
	if (!g_bNavMeshBuilt) return INVALID_HANDLE;
	
	int iConnectionsStartIndex = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_ConnectionsStartIndex);
	if (iConnectionsStartIndex == -1) return INVALID_HANDLE;
	
	int iConnectionsEndIndex = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_ConnectionsEndIndex);
	
	Handle hStack = CreateStack();
	
	for (int i = iConnectionsStartIndex; i <= iConnectionsEndIndex; i++)
	{
		if (GetArrayCell(g_hNavMeshAreaConnections, i, NavMeshConnection_Direction) == iNavDirection)
		{
			PushStackCell(hStack, GetArrayCell(g_hNavMeshAreaConnections, i, NavMeshConnection_AreaIndex));
		}
	}
	
	return hStack;
}

stock Handle NavMeshAreaGetLadderList(int iAreaIndex,int iLadderDir)
{
	if (!g_bNavMeshBuilt) return INVALID_HANDLE;
	
	int iLadderConnectionsStartIndex = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_LadderConnectionsStartIndex);
	if (iLadderConnectionsStartIndex == -1) return INVALID_HANDLE;
	
	int iLadderConnectionsEndIndex = GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_LadderConnectionsEndIndex);
	
	Handle hStack = CreateStack();
	
	for (int i = iLadderConnectionsStartIndex; i <= iLadderConnectionsEndIndex; i++)
	{
		if (GetArrayCell(g_hNavMeshAreaLadderConnections, i, NavMeshLadderConnection_Direction) == iLadderDir)
		{
			PushStackCell(hStack, GetArrayCell(g_hNavMeshAreaLadderConnections, i, NavMeshLadderConnection_LadderIndex));
		}
	}
	
	return hStack;
}

stock int NavMeshAreaGetTotalCost(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return 0;
	
	return GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_TotalCost);
}

stock int NavMeshAreaGetCostSoFar(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return 0;
	
	return GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_CostSoFar);
}

stock int NavMeshAreaGetParent(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return -1;
	
	return GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Parent);
}

stock int NavMeshAreaGetParentHow(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return NUM_TRAVERSE_TYPES;
	
	return GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_ParentHow);
}

stock void NavMeshAreaSetParent(int iAreaIndex,int iParentAreaIndex)
{
	if (!g_bNavMeshBuilt) return;
	
	SetArrayCell(g_hNavMeshAreas, iAreaIndex, iParentAreaIndex, NavMeshArea_Parent);
}

stock void NavMeshAreaSetParentHow(int iAreaIndex,int iParentHow)
{
	if (!g_bNavMeshBuilt) return;
	
	SetArrayCell(g_hNavMeshAreas, iAreaIndex, iParentHow, NavMeshArea_ParentHow);
}

stock bool NavMeshAreaGetExtentLow(int iAreaIndex, float flBuffer[3])
{
	if (!g_bNavMeshBuilt) return false;
	
	flBuffer[0] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_X1));
	flBuffer[1] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Y1));
	flBuffer[2] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Z1));
	return true;
}

stock bool NavMeshAreaGetExtentHigh(int iAreaIndex, float flBuffer[3])
{
	if (!g_bNavMeshBuilt) return false;
	
	flBuffer[0] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_X2));
	flBuffer[1] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Y2));
	flBuffer[2] = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_Z2));
	return true;
}

stock bool NavMeshAreaIsOverlappingPoint(int iAreaIndex, const float flPos[3], float flTolerance)
{
	if (!g_bNavMeshBuilt) return false;
	
	float flExtentLow[3], flExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flExtentHigh);
	
	if (flPos[0] + flTolerance >= flExtentLow[0] &&
		flPos[0] - flTolerance <= flExtentHigh[0] &&
		flPos[1] + flTolerance >= flExtentLow[1] &&
		flPos[1] - flTolerance <= flExtentHigh[1])
	{
		return true;
	}
	
	return false;
}

stock bool NavMeshAreaIsOverlappingArea(int iAreaIndex,int iTargetAreaIndex)
{
	if (!g_bNavMeshBuilt) return false;
	
	float flExtentLow[3], flExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flExtentHigh);
	
	float flTargetExtentLow[3], flTargetExtentHigh[3];
	NavMeshAreaGetExtentLow(iTargetAreaIndex, flTargetExtentLow);
	NavMeshAreaGetExtentHigh(iTargetAreaIndex, flTargetExtentHigh);
	
	if (flTargetExtentLow[0] < flExtentHigh[0] &&
		flTargetExtentHigh[0] > flExtentLow[0] &&
		flTargetExtentLow[1] < flExtentHigh[1] &&
		flTargetExtentHigh[1] > flExtentLow[1])
	{
		return true;
	}
	
	return false;
}

stock float NavMeshAreaGetNECornerZ(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return 0.0;
	
	return view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_NECornerZ));
}

stock float NavMeshAreaGetSWCornerZ(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return 0.0;
	
	return view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_SWCornerZ));
}

stock float NavMeshAreaGetZ(int iAreaIndex, const float flPos[3])
{
	if (!g_bNavMeshBuilt) return 0.0;
	
	float flExtentLow[3], flExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flExtentHigh);
	
	float dx = flExtentHigh[0] - flExtentLow[0];
	float dy = flExtentHigh[1] - flExtentLow[1];
	
	float flNEZ = NavMeshAreaGetNECornerZ(iAreaIndex);
	
	if (dx == 0.0 || dy == 0.0)
	{
		return flNEZ;
	}
	
	float u = (flPos[0] - flExtentLow[0]) / dx;
	float v = (flPos[1] - flExtentLow[1]) / dy;
	
	u = fsel(u, u, 0.0);
	u = fsel(u - 1.0, 1.0, u);
	
	v = fsel(v, v, 0.0);
	v = fsel(v - 1.0, 1.0, v);
	
	float flSWZ = NavMeshAreaGetSWCornerZ(iAreaIndex);
	
	float flNorthZ = flExtentLow[2] + u * (flNEZ - flExtentLow[2]);
	float flSouthZ = flSWZ + u * (flExtentHigh[2] - flSWZ);
	
	return flNorthZ + v * (flSouthZ - flNorthZ);
}

stock float NavMeshAreaGetZFromXAndY(int iAreaIndex, float x, float y)
{
	if (!g_bNavMeshBuilt) return 0.0;
	
	float flInvDxCorners = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_InvDxCorners));
	float flInvDyCorners = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_InvDyCorners));
	
	float flNECornerZ = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_NECornerZ));
	
	if (flInvDxCorners == 0.0 || flInvDyCorners == 0.0)
	{
		return flNECornerZ;
	}
	
	float flExtentLow[3], flExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flExtentHigh);

	float u = (x - flExtentLow[0]) * flInvDxCorners;
	float v = (y - flExtentLow[1]) * flInvDyCorners;
	
	u = FloatClamp(u, 0.0, 1.0);
	v = FloatClamp(v, 0.0, 1.0);
	
	float flSWCornerZ = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_SWCornerZ));
	
	float flNorthZ = flExtentLow[2] + u * (flNECornerZ - flExtentLow[2]);
	float flSouthZ = flSWCornerZ + u * (flExtentHigh[2] - flSWCornerZ);
	
	return flNorthZ + v * (flSouthZ - flNorthZ);
}

stock bool NavMeshAreaContains(int iAreaIndex, const float flPos[3])
{
	if (!g_bNavMeshBuilt) return false;
	
	if (!NavMeshAreaIsOverlappingPoint(iAreaIndex, flPos, 0.0)) return false;
	
	float flMyZ = NavMeshAreaGetZ(iAreaIndex, flPos);
	
	if ((flMyZ - StepHeight) > flPos[2]) return false;
	
	for (int i = 0, iSize = GetArraySize(g_hNavMeshAreas); i < iSize; i++)
	{
		if (i == iAreaIndex) continue;
		
		if (!NavMeshAreaIsOverlappingArea(iAreaIndex, i)) continue;
		
		float flTheirZ = NavMeshAreaGetZ(i, flPos);
		if ((flTheirZ - StepHeight) > flPos[2]) continue;
		
		if (flTheirZ > flMyZ)
		{
			return false;
		}
	}
	
	return true;
}

stock bool NavMeshAreaComputePortal(int iAreaIndex,int iAreaToIndex,int iNavDirection, float flCenter[3], float &flHalfWidth)
{
	if (!g_bNavMeshBuilt) return false;
	
	float flAreaExtentLow[3], flAreaExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flAreaExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flAreaExtentHigh);
	
	float flAreaToExtentLow[3], flAreaToExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaToIndex, flAreaToExtentLow);
	NavMeshAreaGetExtentHigh(iAreaToIndex, flAreaToExtentHigh);
	
	if (iNavDirection == NAV_DIR_NORTH || iNavDirection == NAV_DIR_SOUTH)
	{
		if (iNavDirection == NAV_DIR_NORTH)
		{
			flCenter[1] = flAreaExtentLow[1];
		}
		else
		{
			flCenter[1] = flAreaExtentHigh[1];
		}
		
		float flLeft = flAreaExtentLow[0] > flAreaToExtentLow[0] ? flAreaExtentLow[0] : flAreaToExtentLow[0];
		float flRight = flAreaExtentHigh[0] < flAreaToExtentHigh[0] ? flAreaExtentHigh[0] : flAreaToExtentHigh[0];
		
		if (flLeft < flAreaExtentLow[0]) flLeft = flAreaExtentLow[0];
		else if (flLeft > flAreaExtentHigh[0]) flLeft = flAreaExtentHigh[0];
		
		if (flRight < flAreaExtentLow[0]) flRight = flAreaExtentLow[0];
		else if (flRight > flAreaExtentHigh[0]) flRight = flAreaExtentHigh[0];
		
		flCenter[0] = (flLeft + flRight) / 2.0;
		flHalfWidth = (flRight - flLeft) / 2.0;
	}
	else
	{
		if (iNavDirection == NAV_DIR_WEST)
		{
			flCenter[0] = flAreaExtentLow[0];
		}
		else
		{
			flCenter[0] = flAreaExtentHigh[0];
		}
		
		float flTop = flAreaExtentLow[1] > flAreaToExtentLow[1] ? flAreaExtentLow[1] : flAreaToExtentLow[1];
		float flBottom = flAreaExtentHigh[1] < flAreaToExtentHigh[1] ? flAreaExtentHigh[1] : flAreaToExtentHigh[1];
		
		if (flTop < flAreaExtentLow[1]) flTop = flAreaExtentLow[1];
		else if (flTop > flAreaExtentHigh[1]) flTop = flAreaExtentHigh[1];
		
		if (flBottom < flAreaExtentLow[1]) flBottom = flAreaExtentLow[1];
		else if (flBottom > flAreaExtentHigh[1]) flBottom = flAreaExtentHigh[1];
		
		flCenter[1] = (flTop + flBottom) / 2.0;
		flHalfWidth = (flBottom - flTop) / 2.0;
	}
	
	flCenter[2] = NavMeshAreaGetZFromXAndY(iAreaIndex, flCenter[0], flCenter[1]);
	
	return true;
}

stock float FloatMin(float a, float b)
{
	if (a < b) return a;
	return b;
}

stock float FloatMax(float a, float b)
{
	if (a > b) return a;
	return b;
}

stock bool NavMeshAreaComputeClosestPointInPortal(int iAreaIndex,int iAreaToIndex,int iNavDirection, const float flFromPos[3], float flClosestPos[3])
{
	if (!g_bNavMeshBuilt) return false;
	
	static float flMargin = 25.0; // GenerationStepSize = 25.0;
	
	float flAreaExtentLow[3], flAreaExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flAreaExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flAreaExtentHigh);
	
	float flAreaToExtentLow[3], flAreaToExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaToIndex, flAreaToExtentLow);
	NavMeshAreaGetExtentHigh(iAreaToIndex, flAreaToExtentHigh);
	
	if (iNavDirection == NAV_DIR_NORTH || iNavDirection == NAV_DIR_SOUTH)
	{
		if (iNavDirection == NAV_DIR_NORTH)
		{
			flClosestPos[1] = flAreaExtentLow[1];
		}
		else
		{
			flClosestPos[1] = flAreaExtentHigh[1];
		}
		
		float flLeft = FloatMax(flAreaExtentLow[0], flAreaToExtentLow[0]);
		float flRight = FloatMin(flAreaExtentHigh[0], flAreaToExtentHigh[0]);
		
		float flLeftMargin = NavMeshAreaIsEdge(iAreaToIndex, NAV_DIR_WEST) ? (flLeft + flMargin) : flLeft;
		float flRightMargin = NavMeshAreaIsEdge(iAreaToIndex, NAV_DIR_EAST) ? (flRight - flMargin) : flRight;
		
		if (flLeftMargin > flRightMargin)
		{
			float flMid = (flLeft + flRight) / 2.0;
			flLeftMargin = flMid;
			flRightMargin = flMid;
		}
		
		if (flFromPos[0] < flLeftMargin)
		{
			flClosestPos[0] = flLeftMargin;
		}
		else if (flFromPos[0] > flRightMargin)
		{
			flClosestPos[0] = flRightMargin;
		}
		else
		{
			flClosestPos[0] = flFromPos[0];
		}
	}
	else
	{
		if (iNavDirection == NAV_DIR_WEST)
		{
			flClosestPos[0] = flAreaExtentLow[0];
		}
		else
		{
			flClosestPos[0] = flAreaExtentHigh[0];
		}
		
		float flTop = FloatMax(flAreaExtentLow[1], flAreaToExtentLow[1]);
		float flBottom = FloatMin(flAreaExtentHigh[1], flAreaToExtentHigh[1]);
		
		float flTopMargin = NavMeshAreaIsEdge(iAreaToIndex, NAV_DIR_NORTH) ? (flTop + flMargin) : flTop;
		float flBottomMargin = NavMeshAreaIsEdge(iAreaToIndex, NAV_DIR_SOUTH) ? (flBottom - flMargin) : flBottom;
		
		if (flTopMargin > flBottomMargin)
		{
			float flMid = (flTop + flBottom) / 2.0;
			flTopMargin = flMid;
			flBottomMargin = flMid;
		}
		
		if (flFromPos[1] < flTopMargin)
		{
			flClosestPos[1] = flTopMargin;
		}
		else if (flFromPos[1] > flBottomMargin)
		{
			flClosestPos[1] = flBottomMargin;
		}
		else
		{
			flClosestPos[1] = flFromPos[1];
		}
	}
	
	flClosestPos[2] = NavMeshAreaGetZFromXAndY(iAreaIndex, flClosestPos[0], flClosestPos[1]);
	
	return true;
}

stock int NavMeshAreaComputeDirection(int iAreaIndex, const float flPos[3])
{
	if (!g_bNavMeshBuilt) return NAV_DIR_COUNT;
	
	float flExtentLow[3], flExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flExtentHigh);
	
	if (flPos[0] >= flExtentLow[0] && flPos[0] <= flExtentHigh[0])
	{
		if (flPos[1] < flExtentLow[1])
		{
			return NAV_DIR_NORTH;
		}
		else if (flPos[1] > flExtentHigh[1])
		{
			return NAV_DIR_SOUTH;
		}
	}
	else if (flPos[1] >= flExtentLow[1] && flPos[1] <= flExtentHigh[1])
	{
		if (flPos[0] < flExtentLow[0])
		{
			return NAV_DIR_WEST;
		}
		else if (flPos[0] > flExtentHigh[0])
		{
			return NAV_DIR_EAST;
		}
	}
	
	float flCenter[3];
	NavMeshAreaGetCenter(iAreaIndex, flCenter);
	
	float flTo[3];
	SubtractVectors(flPos, flCenter, flTo);
	
	if (FloatAbs(flTo[0]) > FloatAbs(flTo[1]))
	{
		if (flTo[0] > 0.0) return NAV_DIR_EAST;
		
		return NAV_DIR_WEST;
	}
	else
	{
		if (flTo[1] > 0.0) return NAV_DIR_SOUTH;
		
		return NAV_DIR_NORTH;
	}
}

stock float NavMeshAreaGetLightIntensity(int iAreaIndex, const float flPos[3])
{
	if (!g_bNavMeshBuilt) return 0.0;
	
	float flExtentLow[3], flExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flExtentHigh);

	float flTestPos[3];
	flTestPos[0] = FloatClamp(flPos[0], flExtentLow[0], flExtentHigh[0]);
	flTestPos[1] = FloatClamp(flPos[1], flExtentLow[1], flExtentHigh[1]);
	flTestPos[2] = flPos[2];
	
	float dX = (flTestPos[0] - flExtentLow[0]) / (flExtentHigh[0] - flExtentLow[0]);
	float dY = (flTestPos[1] - flExtentLow[1]) / (flExtentHigh[1] - flExtentLow[1]);
	
	float flCornerLightIntensityNW = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_CornerLightIntensityNW));
	float flCornerLightIntensityNE = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_CornerLightIntensityNE));
	float flCornerLightIntensitySW = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_CornerLightIntensitySW));
	float flCornerLightIntensitySE = view_as<float>(GetArrayCell(g_hNavMeshAreas, iAreaIndex, NavMeshArea_CornerLightIntensitySE));
	
	float flNorthLight = flCornerLightIntensityNW * (1.0 - dX) + flCornerLightIntensityNE * dX;
	float flSouthLight = flCornerLightIntensitySW * (1.0 - dX) + flCornerLightIntensitySE * dX;
	
	return (flNorthLight * (1.0 - dY) + flSouthLight * dY);
}


stock float FloatClamp(float a, float min, float max)
{
	if (a < min) a = min;
	if (a > max) a = max;
	return a;
}

stock bool NavMeshAreaIsEdge(int iAreaIndex,int iNavDirection)
{
	Handle hConnections = NavMeshAreaGetAdjacentList(iAreaIndex, iNavDirection);
	if (hConnections == INVALID_HANDLE || IsStackEmpty(hConnections))
	{
		if (hConnections != INVALID_HANDLE) CloseHandle(hConnections);
		return true;
	}
	
	CloseHandle(hConnections);
	return false;
}

stock float NavMeshLadderGetLength(int iLadderIndex)
{
	if (!g_bNavMeshBuilt) return 0.0;
	
	return view_as<float>(GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_Length));
}

stock int NavMeshGetArea(const float flPos[3], float flBeneathLimit=120.0)
{
	if (!g_bNavMeshBuilt) return -1;
	
	int x = NavMeshWorldToGridX(flPos[0]);
	int y = NavMeshWorldToGridY(flPos[1]);
	
	Handle hAreas = NavMeshGridGetAreas(x, y);
	
	int iUseAreaIndex = -1;
	float flUseZ = -99999999.9;
	float flTestPos[3];
	flTestPos[0] = flPos[0];
	flTestPos[1] = flPos[1];
	flTestPos[2] = flPos[2] + 5.0;
	
	if (hAreas != INVALID_HANDLE)
	{
		while (IsStackEmpty(hAreas))
		{
			int iAreaIndex = -1;
			PopStackCell(hAreas, iAreaIndex);
			
			if (NavMeshAreaIsOverlappingPoint(iAreaIndex, flTestPos, 0.0))
			{
				float z = NavMeshAreaGetZ(iAreaIndex, flTestPos);
				
				if (z > flTestPos[2]) continue;
				
				if (z < flPos[2] - flBeneathLimit) continue;
				
				if (z > flUseZ)
				{
					iUseAreaIndex = iAreaIndex;
					flUseZ = z;
				}
			}
		}
		
		CloseHandle(hAreas);
	}
	
	return iUseAreaIndex;
}

stock bool NavMeshGetGroundHeight(const float flPos[3], float &flHeight, float flNormal[3])
{
	static float flMaxOffset = 100.0;
	
	float flTo[3], flFrom[3];
	flTo[0] = flPos[0];
	flTo[1] = flPos[1];
	flTo[2] = flPos[2] - 10000.0;
	
	flFrom[0] = flPos[0];
	flFrom[1] = flPos[1];
	flFrom[2] = flPos[2] + HalfHumanHeight + 0.001;
	
	while (flTo[2] - flPos[2] < flMaxOffset)
	{
		Handle hTrace = TR_TraceRayFilterEx(flFrom, flTo, MASK_NPCSOLID_BRUSHONLY, RayType_EndPoint, TraceRayIgnoreCustom);
		float flFraction = TR_GetFraction(hTrace);
		float flPlaneNormal[3];
		float flEndPos[3];
		TR_GetEndPosition(flEndPos, hTrace);
		TR_GetPlaneNormal(hTrace, flPlaneNormal);
		CloseHandle(hTrace);
		
		if (flFraction == 1.0 || ((flFrom[2] - flEndPos[2]) >= HalfHumanHeight))
		{
			flHeight = flEndPos[2];
			flNormal[0] = flPlaneNormal[0];
			flNormal[1] = flPlaneNormal[1];
			flNormal[2] = flPlaneNormal[2];
			return true;
		}
		
		flTo[2] = (flFraction == 0.0) ? flFrom[2] : flEndPos[2];
		flFrom[2] = flTo[2] + HalfHumanHeight + 0.001;
	}
	
	flHeight = 0.0;
	flNormal[0] = 0.0;
	flNormal[1] = 0.0;
	flNormal[2] = 1.0;
	
	return false;
}

//	==================================
//	API
//	==================================

public int Native_NavMeshExists(Handle plugin,int numParams)
{
	return g_bNavMeshBuilt;
}

public int Native_NavMeshGetMagicNumber(Handle plugin,int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve magic number because the nav mesh doesn't exist!");
		return -1;
	}
	
	return g_iNavMeshMagicNumber;
}

public int Native_NavMeshGetVersion(Handle plugin,int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve version because the nav mesh doesn't exist!");
		return -1;
	}
	
	return g_iNavMeshVersion;
}

public int Native_NavMeshGetSubVersion(Handle plugin,int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve subversion because the nav mesh doesn't exist!");
		return -1;
	}
	
	return g_iNavMeshSubVersion;
}

public int Native_NavMeshGetSaveBSPSize(Handle plugin,int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve save BSP size because the nav mesh doesn't exist!");
		return -1;
	}
	
	return g_iNavMeshSaveBSPSize;
}

public int Native_NavMeshIsAnalyzed(Handle plugin,int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve analysis state because the nav mesh doesn't exist!");
		return 0;
	}
	
	return g_bNavMeshAnalyzed;
}

public int Native_NavMeshGetPlaces(Handle plugin,int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve place list because the nav mesh doesn't exist!");
		return view_as<int>(INVALID_HANDLE);
	}
	
	return view_as<int>(g_hNavMeshPlaces);
}

public int Native_NavMeshGetAreas(Handle plugin,int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve area list because the nav mesh doesn't exist!");
		return view_as<int>(INVALID_HANDLE);
	}
	
	return view_as<int>(g_hNavMeshAreas);
}

public int Native_NavMeshGetLadders(Handle plugin,int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve ladder list because the nav mesh doesn't exist!");
		return view_as<int>(INVALID_HANDLE);
	}
	
	return view_as<int>(g_hNavMeshLadders);
}

public int Native_NavMeshAreaIsConnected(Handle plugin,int numParams)
{
	int iStartAreaIndex = GetNativeCell(1);
	int iEndAreaIndex = GetNativeCell(2);
	return NavMeshAreaIsConnected(iStartAreaIndex,iEndAreaIndex);
}

public int Native_NavMeshCollectSurroundingAreas(Handle plugin,int numParams)
{
	Handle hTarget = view_as<Handle>(GetNativeCell(1));
	Handle hDummy = NavMeshCollectSurroundingAreas(GetNativeCell(2), view_as<float>(GetNativeCell(3)), view_as<float>(GetNativeCell(4)), view_as<float>(GetNativeCell(5)));
	
	if (hDummy != INVALID_HANDLE)
	{
		while (!IsStackEmpty(hDummy))
		{
			int iAreaIndex = -1;
			PopStackCell(hDummy, iAreaIndex);
			PushStackCell(hTarget, iAreaIndex);
		}
		
		CloseHandle(hDummy);
	}
}

public int Native_NavMeshBuildPath(Handle plugin,int numParams)
{
	float flGoalPos[3];
	GetNativeArray(3, flGoalPos, 3);
	
	int iClosestIndex = GetNativeCellRef(6);
	
	bool bResult = NavMeshBuildPath(GetNativeCell(1), 
		GetNativeCell(2), 
		flGoalPos,
		plugin,
		view_as<Function>(GetNativeCell(4)),
		GetNativeCell(5),
		iClosestIndex,
		view_as<float>(GetNativeCell(7)),
		view_as<float>(GetNativeCell(8)));
		
	SetNativeCellRef(6, iClosestIndex);
	return bResult;
}

public int Native_NavMeshGetArea(Handle plugin,int numParams)
{
	float flPos[3];
	GetNativeArray(1, flPos, 3);

	return NavMeshGetArea(flPos, view_as<float>(GetNativeCell(2)));
}

public int Native_NavMeshGetNearestArea(Handle plugin,int numParams)
{
	float flPos[3];
	GetNativeArray(1, flPos, 3);
	
	return NavMeshGetNearestArea(flPos, view_as<bool>(GetNativeCell(2)), view_as<float>(GetNativeCell(3)), view_as<bool>(GetNativeCell(4)), view_as<bool>(GetNativeCell(5)), GetNativeCell(6));
}

public int Native_NavMeshWorldToGridX(Handle plugin,int numParams)
{
	return NavMeshWorldToGridX(view_as<float>(GetNativeCell(1)));
}

public int Native_NavMeshWorldToGridY(Handle plugin,int numParams)
{
	return NavMeshWorldToGridY(view_as<float>(GetNativeCell(1)));
}

public int Native_NavMeshGridGetAreas(Handle plugin,int numParams)
{
	Handle hTarget = view_as<Handle>(GetNativeCell(1));
	Handle hDummy = NavMeshGridGetAreas(GetNativeCell(2), GetNativeCell(3));
	
	if (hDummy != INVALID_HANDLE)
	{
		while (!IsStackEmpty(hDummy))
		{
			int iAreaIndex = -1;
			PopStackCell(hDummy, iAreaIndex);
			PushStackCell(hTarget, iAreaIndex);
		}
		
		CloseHandle(hDummy);
	}
}

public int Native_NavMeshGetGridSizeX(Handle plugin,int numParams)
{
	return g_iNavMeshGridSizeX;
}

public int Native_NavMeshGetGridSizeY(Handle plugin,int numParams)
{
	return g_iNavMeshGridSizeY;
}

public int Native_NavMeshAreaGetClosestPointOnArea(Handle plugin,int numParams)
{
	float flPos[3], flClose[3];
	GetNativeArray(2, flPos, 3);
	NavMeshAreaGetClosestPointOnArea(GetNativeCell(1), flPos, flClose);
	SetNativeArray(3, flClose, 3);
}

//stock bool:NavMeshGetGroundHeight(const float flPos[3], float &flHeight, float flNormal[3])
public int Native_NavMeshGetGroundHeight(Handle plugin,int numParams)
{
	float flPos[3], flNormal[3];
	GetNativeArray(1, flPos, 3);
	float flHeight = view_as<float>(GetNativeCellRef(2));
	bool bResult = NavMeshGetGroundHeight(flPos, flHeight, flNormal);
	SetNativeCellRef(2, flHeight);
	SetNativeArray(3, flNormal, 3);
	return bResult;
}

public int Native_NavMeshAreaGetMasterMarker(Handle plugin,int numParams)
{
	return g_iNavMeshAreaMasterMarker;
}

public int Native_NavMeshAreaChangeMasterMarker(Handle plugin,int numParams)
{
	g_iNavMeshAreaMasterMarker++;
}

public int Native_NavMeshAreaGetFlags(Handle plugin,int numParams)
{
	return NavMeshAreaGetFlags(GetNativeCell(1));
}

public int Native_NavMeshAreaGetCenter(Handle plugin,int numParams)
{
	float flResult[3];
	if (NavMeshAreaGetCenter(GetNativeCell(1), flResult))
	{
		SetNativeArray(2, flResult, 3);
		return true;
	}
	
	return false;
}

public int Native_NavMeshAreaGetAdjacentList(Handle plugin,int numParams)
{
	Handle hTarget = view_as<Handle>(GetNativeCell(1));
	Handle hDummy = NavMeshAreaGetAdjacentList(GetNativeCell(2), GetNativeCell(3));
	
	if (hDummy != INVALID_HANDLE)
	{
		while (!IsStackEmpty(hDummy))
		{
			int iAreaIndex = -1;
			PopStackCell(hDummy, iAreaIndex);
			PushStackCell(hTarget, iAreaIndex);
		}
		
		CloseHandle(hDummy);
	}
}

public int Native_NavMeshAreaGetLadderList(Handle plugin,int numParams)
{
	Handle hTarget = view_as<Handle>(GetNativeCell(1));

	Handle hDummy = NavMeshAreaGetLadderList(GetNativeCell(2), GetNativeCell(3));
	if (hDummy != INVALID_HANDLE)
	{
		while (!IsStackEmpty(hDummy))
		{
			int iAreaIndex = -1;
			PopStackCell(hDummy, iAreaIndex);
			PushStackCell(hTarget, iAreaIndex);
		}
		
		CloseHandle(hDummy);
	}
}

public int Native_NavMeshAreaGetTotalCost(Handle plugin,int numParams)
{
	return NavMeshAreaGetTotalCost(GetNativeCell(1));
}

public int Native_NavMeshAreaGetCostSoFar(Handle plugin,int numParams)
{
	return NavMeshAreaGetCostSoFar(GetNativeCell(1));
}

public int Native_NavMeshAreaGetParent(Handle plugin,int numParams)
{
	return NavMeshAreaGetParent(GetNativeCell(1));
}

public int Native_NavMeshAreaGetParentHow(Handle plugin,int numParams)
{
	return NavMeshAreaGetParentHow(GetNativeCell(1));
}

public int Native_NavMeshAreaSetParent(Handle plugin,int numParams)
{
	NavMeshAreaSetParent(GetNativeCell(1), GetNativeCell(2));
}

public int Native_NavMeshAreaSetParentHow(Handle plugin,int numParams)
{
	NavMeshAreaSetParentHow(GetNativeCell(1), GetNativeCell(2));
}

public int Native_NavMeshAreaGetExtentLow(Handle plugin,int numParams)
{
	float flExtent[3];
	if (NavMeshAreaGetExtentLow(GetNativeCell(1), flExtent))
	{
		SetNativeArray(2, flExtent, 3);
		return true;
	}
	
	return false;
}

public int Native_NavMeshAreaGetExtentHigh(Handle plugin,int numParams)
{
	float flExtent[3];
	if (NavMeshAreaGetExtentHigh(GetNativeCell(1), flExtent))
	{
		SetNativeArray(2, flExtent, 3);
		return true;
	}
	
	return false;
}

public int Native_NavMeshAreaIsOverlappingPoint(Handle plugin,int numParams)
{
	float flPos[3];
	GetNativeArray(2, flPos, 3);
	
	return NavMeshAreaIsOverlappingPoint(GetNativeCell(1), flPos, view_as<float>(GetNativeCell(3)));
}

public int Native_NavMeshAreaIsOverlappingArea(Handle plugin,int numParams)
{
	return NavMeshAreaIsOverlappingArea(GetNativeCell(1), GetNativeCell(2));
}

public int Native_NavMeshAreaGetNECornerZ(Handle plugin,int numParams)
{
	return view_as<int>(NavMeshAreaGetNECornerZ(GetNativeCell(1)));
}

public int Native_NavMeshAreaGetSWCornerZ(Handle plugin,int numParams)
{
	return view_as<int>(NavMeshAreaGetSWCornerZ(GetNativeCell(1)));
}

public int Native_NavMeshAreaGetZ(Handle plugin,int numParams)
{
	float flPos[3];
	GetNativeArray(2, flPos, 3);

	return view_as<int>(NavMeshAreaGetZ(GetNativeCell(1), flPos));
}

public int Native_NavMeshAreaGetZFromXAndY(Handle plugin,int numParams)
{
	return view_as<int>(NavMeshAreaGetZFromXAndY(GetNativeCell(1), view_as<float>(GetNativeCell(2)), view_as<float>(GetNativeCell(3))));
}

public int Native_NavMeshAreaContains(Handle plugin,int numParams)
{
	float flPos[3];
	GetNativeArray(2, flPos, 3);

	return NavMeshAreaContains(GetNativeCell(1), flPos);
}

public int Native_NavMeshAreaComputePortal(Handle plugin,int numParams)
{
	float flCenter[3];
	float flHalfWidth = GetNativeCellRef(5);
	
	bool bResult = NavMeshAreaComputePortal(GetNativeCell(1),
		GetNativeCell(2),
		GetNativeCell(3),
		flCenter,
		flHalfWidth);
		
	SetNativeArray(4, flCenter, 3);
	SetNativeCellRef(5, flHalfWidth);
	return bResult;
}

public int Native_NavMeshAreaComputeClosestPointInPortal(Handle plugin,int numParams)
{
	float flFromPos[3];
	GetNativeArray(4, flFromPos, 3);
	
	float flClosestPos[3];

	bool bResult = NavMeshAreaComputeClosestPointInPortal(GetNativeCell(1),
		GetNativeCell(2),
		GetNativeCell(3),
		flFromPos,
		flClosestPos);
		
	SetNativeArray(5, flClosestPos, 3);
	return bResult;
}

public int Native_NavMeshAreaComputeDirection(Handle plugin,int numParams)
{
	float flPos[3];
	GetNativeArray(2, flPos, 3);
	
	return NavMeshAreaComputeDirection(GetNativeCell(1), flPos);
}

public int Native_NavMeshAreaGetLightIntensity(Handle plugin,int numParams)
{
	float flPos[3];
	GetNativeArray(2, flPos, 3);
	
	return view_as<int>(NavMeshAreaGetLightIntensity(GetNativeCell(1), flPos));
}

public int Native_NavMeshLadderGetLength(Handle plugin,int numParams)
{
	return view_as<int>(NavMeshLadderGetLength(GetNativeCell(1)));
}
public bool TraceRayIgnoreCustom(int entity,int mask, any data)
{
	if (entity > 0 && entity <= MaxClients) return false;
	
	if (IsValidEdict(entity))
	{
		char sClass[64];
		GetEntityNetClass(entity, sClass, sizeof(sClass));
		if (StrEqual(sClass, "CFuncRespawnRoomVisualizer")) return false;
		else if (StrEqual(sClass, "CBaseDoor")) return false;
		else if (StrEqual(sClass, "CTFBaseBoss")) return false;
	}
	
	return true;
}