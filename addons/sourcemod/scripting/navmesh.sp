// Huge huge HUGE props and credits to Anthony Iacono (pimpinjuice) and his Nav-file parser code,
// which can be found here: https://github.com/AnthonyIacono/War3SourceV2/tree/master/Nav

#include <sourcemod>
#include <sdktools>
#include <navmesh>

#define PLUGIN_VERSION "1.0.5"

public Plugin myinfo = 
{
    name = "SourcePawn Navigation Mesh Parser",
    author	= "KitRifty, Benoist3012, (with modifications by sigsegv)",
    description	= "A plugin that can read Valve's Navigation Mesh.",
    version = PLUGIN_VERSION,
    url = ""
}

#define UNSIGNED_INT_BYTE_SIZE 4
#define UNSIGNED_CHAR_BYTE_SIZE 1
#define UNSIGNED_SHORT_BYTE_SIZE 2
#define FLOAT_BYTE_SIZE 4

enum
{
	NavMeshArea_ID = 0,
	NavMeshArea_Flags,
	NavMeshArea_PlaceID,
	NavMeshArea_X1,
	NavMeshArea_Y1,
	NavMeshArea_Z1,
	NavMeshArea_X2,
	NavMeshArea_Y2,
	NavMeshArea_Z2,
	NavMeshArea_CenterX,
	NavMeshArea_CenterY,
	NavMeshArea_CenterZ,
	NavMeshArea_InvDxCorners,
	NavMeshArea_InvDyCorners,
	NavMeshArea_NECornerZ,
	NavMeshArea_SWCornerZ,
	
//	NavMeshArea_Connections,
	NavMeshArea_ConnectionsStartIndex,
	NavMeshArea_ConnectionsEndIndex,
	
//	NavMeshArea_HidingSpots,
	NavMeshArea_HidingSpotsStartIndex,
	NavMeshArea_HidingSpotsEndIndex,
	
//	NavMeshArea_EncounterPaths,
	NavMeshArea_EncounterPathsStartIndex,
	NavMeshArea_EncounterPathsEndIndex,
	
//	NavMeshArea_LadderConnections,
	NavMeshArea_LadderConnectionsStartIndex,
	NavMeshArea_LadderConnectionsEndIndex,
	
	NavMeshArea_CornerLightIntensityNW,
	NavMeshArea_CornerLightIntensityNE,
	NavMeshArea_CornerLightIntensitySE,
	NavMeshArea_CornerLightIntensitySW,
	
//	NavMeshArea_VisibleAreas,
	NavMeshArea_VisibleAreasStartIndex,
	NavMeshArea_VisibleAreasEndIndex,
	
	NavMeshArea_InheritVisibilityFrom,
	NavMeshArea_EarliestOccupyTimeFirstTeam,
	NavMeshArea_EarliestOccupyTimeSecondTeam,
	NavMeshArea_unk01,
	NavMeshArea_Blocked,
	
// 	A* pathfinding
	NavMeshArea_Parent,
	NavMeshArea_ParentHow,
	NavMeshArea_CostSoFar,
	NavMeshArea_TotalCost,
	NavMeshArea_Marker,
	NavMeshArea_OpenMarker,
	NavMeshArea_PrevOpenIndex,
	NavMeshArea_NextOpenIndex,
	NavMeshArea_PathLengthSoFar,
	
	NavMeshArea_NearSearchMarker,
	
	NavMeshArea_MaxStats
};

enum
{
	NavMeshConnection_AreaIndex = 0,
	NavMeshConnection_Direction,
	NavMeshConnection_MaxStats
};

enum
{
	NavMeshHidingSpot_ID = 0,
	NavMeshHidingSpot_X,
	NavMeshHidingSpot_Y,
	NavMeshHidingSpot_Z,
	NavMeshHidingSpot_Flags,
	NavMeshHidingSpot_AreaIndex,
	NavMeshHidingSpot_MaxStats
};

enum
{
	NavMeshEncounterPath_FromAreaIndex = 0,
	NavMeshEncounterPath_FromDirection,
	NavMeshEncounterPath_ToAreaIndex,
	NavMeshEncounterPath_ToDirection,
	NavMeshEncounterPath_SpotsStartIndex,
	NavMeshEncounterPath_SpotsEndIndex,
	NavMeshEncounterPath_MaxStats
};

enum
{
	NavMeshEncounterSpot_HidingSpotIndex = 0,
	NavMeshEncounterSpot_ParametricDistance,
	NavMeshEncounterSpot_MaxStats
};

enum
{
	NavMeshLadderConnection_LadderIndex = 0,
	NavMeshLadderConnection_Direction,
	NavMeshLadderConnection_MaxStats
};

enum
{
	NavMeshVisibleArea_Index = 0,
	NavMeshVisibleArea_Attributes,
	NavMeshVisibleArea_MaxStats
};

enum
{
	NavMeshLadder_ID = 0,
	NavMeshLadder_Width,
	NavMeshLadder_Length,
	NavMeshLadder_TopX,
	NavMeshLadder_TopY,
	NavMeshLadder_TopZ,
	NavMeshLadder_BottomX,
	NavMeshLadder_BottomY,
	NavMeshLadder_BottomZ,
	NavMeshLadder_Direction,
	NavMeshLadder_TopForwardAreaIndex,
	NavMeshLadder_TopLeftAreaIndex,
	NavMeshLadder_TopRightAreaIndex,
	NavMeshLadder_TopBehindAreaIndex,
	NavMeshLadder_BottomAreaIndex,
	NavMeshLadder_MaxStats
};

enum
{
	NavMeshGrid_ListStartIndex = 0,
	NavMeshGrid_ListEndIndex,
	NavMeshGrid_MaxStats
};

enum
{
	NavMeshGridList_AreaIndex = 0,
	NavMeshGridList_Owner,
	NavMeshGridList_MaxStats
};

ArrayList g_hNavMeshPlaces;
ArrayList g_hNavMeshAreas;
ArrayList g_hNavMeshAreaConnections;
ArrayList g_hNavMeshAreaHidingSpots;
ArrayList g_hNavMeshAreaEncounterPaths;
ArrayList g_hNavMeshAreaEncounterSpots;
ArrayList g_hNavMeshAreaLadderConnections;
ArrayList g_hNavMeshAreaVisibleAreas;

ArrayList g_hNavMeshLadders;

int g_iNavMeshMagicNumber;
int g_iNavMeshVersion;
int g_iNavMeshSubVersion;
int g_iNavMeshSaveBSPSize;
bool g_bNavMeshAnalyzed;

ArrayList g_hNavMeshGrid;
ArrayList g_hNavMeshGridLists;

float g_flNavMeshGridCellSize = 300.0;
float g_flNavMeshMinX;
float g_flNavMeshMinY;
int g_iNavMeshGridSizeX;
int g_iNavMeshGridSizeY;


bool g_bNavMeshBuilt = false;

// For A* pathfinding.
static int g_iNavMeshAreaOpenListIndex = -1;
static int g_iNavMeshAreaOpenListTailIndex = -1;
static int g_iNavMeshAreaMasterMarker = 0;


public APLRes AskPluginLoad2(Handle myself, bool late, char[] error, int err_max)
{
	RegPluginLibrary("navmesh");
	
	CreateNative("NavMesh_Exists", Native_NavMeshExists);
	CreateNative("NavMesh_GetMagicNumber", Native_NavMeshGetMagicNumber);
	CreateNative("NavMesh_GetVersion", Native_NavMeshGetVersion);
	CreateNative("NavMesh_GetSubVersion", Native_NavMeshGetSubVersion);
	CreateNative("NavMesh_GetSaveBSPSize", Native_NavMeshGetSaveBSPSize);
	CreateNative("NavMesh_IsAnalyzed", Native_NavMeshIsAnalyzed);
	//CreateNative("NavMesh_GetPlaces", Native_NavMeshGetPlaces);
	//CreateNative("NavMesh_GetAreas", Native_NavMeshGetAreas);
	//CreateNative("NavMesh_GetLadders", Native_NavMeshGetLadders);
	
	CreateNative("NavMesh_CollectSurroundingAreas", Native_NavMeshCollectSurroundingAreas);
	CreateNative("NavMesh_BuildPath", Native_NavMeshBuildPath);
	
	CreateNative("NavMesh_FindAreaByID", Native_NavMeshFindAreaByID);
	CreateNative("NavMesh_GetArea", Native_NavMeshGetArea);
	CreateNative("NavMesh_GetNearestArea", Native_NavMeshGetNearestArea);
	
	CreateNative("NavMesh_FindHidingSpotByID", Native_NavMeshFindHidingSpotByID);
	CreateNative("NavMesh_GetRandomHidingSpot", Native_NavMeshGetRandomHidingSpot);
	
	CreateNative("NavMesh_WorldToGridX", Native_NavMeshWorldToGridX);
	CreateNative("NavMesh_WorldToGridY", Native_NavMeshWorldToGridY);
	CreateNative("NavMesh_GetAreasOnGrid", Native_NavMeshGridGetAreas);
	CreateNative("NavMesh_GetGridSizeX", Native_NavMeshGetGridSizeX);
	CreateNative("NavMesh_GetGridSizeY", Native_NavMeshGetGridSizeY);
	
	CreateNative("NavMesh_GetGroundHeight", Native_NavMeshGetGroundHeight);
	
	CreateNative("NavMeshArea_GetMasterMarker", Native_NavMeshAreaGetMasterMarker);
	CreateNative("NavMeshArea_ChangeMasterMarker", Native_NavMeshAreaChangeMasterMarker);
	
	CreateNative("NavMeshArea_GetID", Native_NavMeshAreaGetID);
	CreateNative("NavMeshArea_GetFlags", Native_NavMeshAreaGetFlags);
	CreateNative("NavMeshArea_GetPlace", Native_NavMeshAreaGetPlace);
	CreateNative("NavMeshArea_GetCenter", Native_NavMeshAreaGetCenter);
	CreateNative("NavMeshArea_GetAdjacentList", Native_NavMeshAreaGetAdjacentList);
	CreateNative("NavMeshArea_GetLadderList", Native_NavMeshAreaGetLadderList);
	CreateNative("NavMeshArea_GetHidingSpots", Native_NavMeshAreaGetHidingSpots);
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
	CreateNative("NavMeshArea_GetCorner", Native_NavMeshAreaGetCorner);
	CreateNative("NavMeshArea_GetZ", Native_NavMeshAreaGetZ);
	CreateNative("NavMeshArea_GetZFromXAndY", Native_NavMeshAreaGetZFromXAndY);
	CreateNative("NavMeshArea_IsEdge", Native_NavMeshAreaIsEdge);
	CreateNative("NavMeshArea_Contains", Native_NavMeshAreaContains);
	CreateNative("NavMeshArea_GetRandomPoint", Native_NavMeshAreaGetRandomPoint);
	CreateNative("NavMeshArea_IsConnected", Native_NavMeshAreaIsConnected);
	CreateNative("NavMeshArea_ComputePortal", Native_NavMeshAreaComputePortal);
	CreateNative("NavMeshArea_ComputeClosestPointInPortal", Native_NavMeshAreaComputeClosestPointInPortal);
	CreateNative("NavMeshArea_ComputeDirection", Native_NavMeshAreaComputeDirection);
	CreateNative("NavMeshArea_GetLightIntensity", Native_NavMeshAreaGetLightIntensity);
	
	CreateNative("NavHidingSpot_GetID", Native_NavHidingSpotGetID);
	CreateNative("NavHidingSpot_GetFlags", Native_NavHidingSpotGetFlags);
	CreateNative("NavHidingSpot_GetPosition", Native_NavHidingSpotGetPosition);
	CreateNative("NavHidingSpot_GetArea", Native_NavHidingSpotGetArea);
	
	CreateNative("NavMeshLadder_GetLength", Native_NavMeshLadderGetLength);
	CreateNative("NavMeshLadder_GetWidth", Native_NavMeshLadderGetWidth);
	CreateNative("NavMeshLadder_GetTopForwardArea", Native_NavMeshLadderGetTopForwardArea);
	CreateNative("NavMeshLadder_GetTopLeftArea", Native_NavMeshLadderGetTopLeftArea);
	CreateNative("NavMeshLadder_GetTopRightArea", Native_NavMeshLadderGetTopRightArea);
	CreateNative("NavMeshLadder_GetTopBehindArea", Native_NavMeshLadderGetTopBehindArea);
	CreateNative("NavMeshLadder_GetBottomArea", Native_NavMeshLadderGetBottomArea);
	CreateNative("NavMeshLadder_GetTop", Native_NavMeshLadderGetTop);
	CreateNative("NavMeshLadder_GetBottom", Native_NavMeshLadderGetBottom);
	
	CreateNative("NavSpotEncounter_GetFrom", Native_NavSpotEncounterGetFrom);
	CreateNative("NavSpotEncounter_GetFromDirection", Native_NavSpotEncounterGetFromDirection);
	CreateNative("NavSpotEncounter_GetTo", Native_NavSpotEncounterGetTo);
	CreateNative("NavSpotEncounter_GetToDirection", Native_NavSpotEncounterGetToDirection);
	CreateNative("NavSpotEncounter_GetSpots", Native_NavSpotEncounterGetSpots);
	
	CreateNative("NavSpotOrder_GetHidingSpot", Native_NavSpotOrderGetHidingSpot);
	CreateNative("NavSpotOrder_GetParametricDistance", Native_NavSpotOrderGetParametricDistance);
}

public void OnPluginStart()
{
	g_hNavMeshPlaces = new ArrayList(256);
	g_hNavMeshAreas = new ArrayList(NavMeshArea_MaxStats);
	g_hNavMeshAreaConnections = new ArrayList(NavMeshConnection_MaxStats);
	g_hNavMeshAreaHidingSpots = new ArrayList(NavMeshHidingSpot_MaxStats);
	g_hNavMeshAreaEncounterPaths = new ArrayList(NavMeshEncounterPath_MaxStats);
	g_hNavMeshAreaEncounterSpots = new ArrayList(NavMeshEncounterSpot_MaxStats);
	g_hNavMeshAreaLadderConnections = new ArrayList(NavMeshLadderConnection_MaxStats);
	g_hNavMeshAreaVisibleAreas = new ArrayList(NavMeshVisibleArea_MaxStats);
	
	g_hNavMeshLadders = new ArrayList(NavMeshLadder_MaxStats);
	
	g_hNavMeshGrid = new ArrayList(NavMeshGrid_MaxStats);
	g_hNavMeshGridLists = new ArrayList(NavMeshGridList_MaxStats);
	
	HookEvent("nav_blocked", Event_NavAreaBlocked);
}

public void OnMapStart()
{
	NavMeshDestroy();

	char sMap[256];
	GetCurrentMap(sMap, sizeof(sMap));
	
	g_bNavMeshBuilt = NavMeshLoad(sMap);
}

public void Event_NavAreaBlocked(Event event, const char[] name, bool dB)
{
	if (!g_bNavMeshBuilt) return;

	int iAreaID = event.GetInt("area");
	int iAreaIndex = g_hNavMeshAreas.FindValue(iAreaID); // Wow, this is terrible.
	if (iAreaIndex != -1)
	{
		bool bBlocked = view_as<bool>(event.GetInt("blocked"));
		g_hNavMeshAreas.Set(iAreaIndex, bBlocked, NavMeshArea_Blocked);
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

stock float NavMeshAreaComputeAdjacentConnectionHeightChange(int iAreaIndex, int iTargetAreaIndex)
{
	bool bFoundArea = false;
	int iNavDirection;
	
	for (iNavDirection = 0; iNavDirection < NAV_DIR_COUNT; iNavDirection++)
	{
		ArrayStack hConnections = NavMeshAreaGetAdjacentList(iAreaIndex, iNavDirection);
		if (hConnections == null) continue;
		
		while (!hConnections.Empty)
		{
			int iTempAreaIndex = -1;
			PopStackCell(hConnections, iTempAreaIndex);
			
			if (iTempAreaIndex == iTargetAreaIndex)
			{
				bFoundArea = true;
				break;
			}
		}
		
		delete hConnections;
		
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

ArrayStack NavMeshCollectSurroundingAreas(int iStartAreaIndex, float flTravelDistanceLimit=1500.0, float flMaxStepUpLimit=StepHeight, float flMaxDropDownLimit=100.0)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not search surrounding areas because the nav mesh does not exist!");
		return null;
	}
	
	if (iStartAreaIndex == -1)
	{
		LogError("Could not search surrounding areas because the starting area does not exist!");
		return null;
	}
	
	ArrayStack hNearAreasList = new ArrayStack();
	
	NavMeshAreaClearSearchLists();
	
	NavMeshAreaAddToOpenList(iStartAreaIndex);
	g_hNavMeshAreas.Set(iStartAreaIndex, 0, NavMeshArea_TotalCost);
	g_hNavMeshAreas.Set(iStartAreaIndex, 0, NavMeshArea_CostSoFar);
	g_hNavMeshAreas.Set(iStartAreaIndex, -1, NavMeshArea_Parent);
	g_hNavMeshAreas.Set(iStartAreaIndex, NUM_TRAVERSE_TYPES, NavMeshArea_ParentHow);
	NavMeshAreaMark(iStartAreaIndex);
	
	while (!NavMeshAreaIsOpenListEmpty())
	{
		int iAreaIndex = NavMeshAreaPopOpenList();
		if (flTravelDistanceLimit > 0.0 && 
			float(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_CostSoFar)) > flTravelDistanceLimit)
		{
			continue;
		}
		
		int iAreaParent = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Parent);
		if (iAreaParent != -1)
		{
			float flDeltaZ = NavMeshAreaComputeAdjacentConnectionHeightChange(iAreaParent, iAreaIndex);
			if (flDeltaZ > flMaxStepUpLimit) continue;
			if (flDeltaZ < -flMaxDropDownLimit) continue;
		}
		
		hNearAreasList.Push(iAreaIndex);
		
		NavMeshAreaMark(iAreaIndex);
		
		for (int iNavDir = 0; iNavDir < NAV_DIR_COUNT; iNavDir++)
		{
			ArrayStack hConnections = NavMeshAreaGetAdjacentList(iAreaIndex, iNavDir);
			if (hConnections != null)
			{
				while (!hConnections.Empty)
				{
					int iAdjacentAreaIndex = -1;
					PopStackCell(hConnections, iAdjacentAreaIndex);
					
					if (view_as<bool>(g_hNavMeshAreas.Get(iAdjacentAreaIndex, NavMeshArea_Blocked))) continue;
					
					if (!NavMeshAreaIsMarked(iAdjacentAreaIndex))
					{
						g_hNavMeshAreas.Set(iAdjacentAreaIndex, 0, NavMeshArea_TotalCost);
						g_hNavMeshAreas.Set(iAdjacentAreaIndex, iAreaIndex, NavMeshArea_Parent);
						g_hNavMeshAreas.Set(iAdjacentAreaIndex, iNavDir, NavMeshArea_ParentHow);
						
						int iDistAlong = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_CostSoFar);
						
						float flAdjacentAreaCenter[3]; 
						float flAreaCenter[3];
						NavMeshAreaGetCenter(iAreaIndex, flAreaCenter);
						NavMeshAreaGetCenter(iAdjacentAreaIndex, flAdjacentAreaCenter);
						
						iDistAlong += RoundToFloor(GetVectorDistance(flAdjacentAreaCenter, flAreaCenter));
						g_hNavMeshAreas.Set(iAdjacentAreaIndex, iDistAlong, NavMeshArea_CostSoFar);
						NavMeshAreaAddToOpenList(iAdjacentAreaIndex);
					}
				}
				
				delete hConnections;
			}
		}
	}
	
	return hNearAreasList;
}

bool NavMeshBuildPath(int iStartAreaIndex,
	int iGoalAreaIndex,
	const float flGoalPos[3],
	Handle hCostFunctionPlugin,
	NavPathCostFunctor iCostFunction,
	any iCostData=0,
	int &iClosestAreaIndex=-1,
	float flMaxPathLength=0.0,
	float flMaxStepSize=0.0)
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
	
	g_hNavMeshAreas.Set(iStartAreaIndex, -1, NavMeshArea_Parent);
	g_hNavMeshAreas.Set(iStartAreaIndex, NUM_TRAVERSE_TYPES, NavMeshArea_ParentHow);
	
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
	g_hNavMeshAreas.Set(iStartAreaIndex, iStartTotalCost, NavMeshArea_TotalCost);
	
	int iInitCost = 0;
	
	Call_StartFunction(hCostFunctionPlugin, iCostFunction);
	Call_PushCell(iStartAreaIndex);
	Call_PushCell(-1);
	Call_PushCell(-1);
	Call_PushCell(iCostData);
	Call_Finish(iInitCost);
	
	if (iInitCost < 0) return false;
	
	g_hNavMeshAreas.Set(iStartAreaIndex, 0, NavMeshArea_CostSoFar);
	g_hNavMeshAreas.Set(iStartAreaIndex, 0.0, NavMeshArea_PathLengthSoFar);
	NavMeshAreaAddToOpenList(iStartAreaIndex);
	
	int iClosestAreaDist = iStartTotalCost;
	
	bool bHaveMaxPathLength = (flMaxPathLength != 0.0);
	
	// Perform A* search.
	while (!NavMeshAreaIsOpenListEmpty())
	{
		int iAreaIndex = NavMeshAreaPopOpenList();
		
		if (view_as<bool>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Blocked)))
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
		static int SEARCH_FLOOR = 0; 
		static int SEARCH_LADDERS = 1;
		
		int iSearchWhere = SEARCH_FLOOR;
		int iSearchDir = NAV_DIR_NORTH;
		
		ArrayStack hFloorList = NavMeshAreaGetAdjacentList(iAreaIndex, iSearchDir);
		
		bool bLadderUp = true;
		ArrayStack hLadderList = null;
		int iLadderTopDir = 0;
		
		for (;;)
		{
			int iNewAreaIndex = -1;
			int iNavTraverseHow = 0;
			int iLadderIndex = -1;
			
			if (iSearchWhere == SEARCH_FLOOR)
			{
				if (hFloorList == null || hFloorList.Empty)
				{
					iSearchDir++;
					if (hFloorList != null) delete hFloorList;
					
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
				if (hLadderList == null || hLadderList.Empty)
				{
					if (hLadderList != null) delete hLadderList;
					
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
							iNewAreaIndex = g_hNavMeshLadders.Get(iLadderIndex, NavMeshLadder_TopForwardAreaIndex);
						}
						case 1:
						{
							iNewAreaIndex = g_hNavMeshLadders.Get(iLadderIndex, NavMeshLadder_TopLeftAreaIndex);
						}
						case 2:
						{
							iNewAreaIndex = g_hNavMeshLadders.Get(iLadderIndex, NavMeshLadder_TopRightAreaIndex);
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
					iNewAreaIndex = g_hNavMeshLadders.Get(iLadderIndex, NavMeshLadder_BottomAreaIndex);
					iNavTraverseHow = GO_LADDER_DOWN;
				}
				
				if (iNewAreaIndex == -1) continue;
			}
			
			if (g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Parent) == iNewAreaIndex) 
			{
				// Don't backtrack.
				continue;
			}
			
			if (iNewAreaIndex == iAreaIndex)
			{
				continue;
			}
			
			if (view_as<bool>(g_hNavMeshAreas.Get(iNewAreaIndex, NavMeshArea_Blocked))) 
			{
				// Don't consider blocked areas.
				continue;
			}
			
			int iNewCostSoFar = 0;
			
			Call_StartFunction(hCostFunctionPlugin, iCostFunction);
			Call_PushCell(iNewAreaIndex);
			Call_PushCell(iAreaIndex);
			Call_PushCell(iLadderIndex);
			Call_PushCell(iCostData);
			Call_Finish(iNewCostSoFar);
			
			if (iNewCostSoFar < 0) continue;
			
			if (flMaxStepSize > 0.0)
			{
				float flDeltaZ = NavMeshAreaComputeAdjacentConnectionHeightChange(iAreaIndex, iNewAreaIndex);
				if (flDeltaZ > flMaxStepSize) continue;
			}
			
			float flNewAreaCenter[3];
			NavMeshAreaGetCenter(iNewAreaIndex, flNewAreaCenter);
			
			if (bHaveMaxPathLength)
			{
				float flAreaCenter[3];
				NavMeshAreaGetCenter(iAreaIndex, flAreaCenter);
				
				float flDeltaLength = GetVectorDistance(flNewAreaCenter, flAreaCenter);
				float flNewLengthSoFar = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_PathLengthSoFar)) + flDeltaLength;
				if (flNewLengthSoFar > flMaxPathLength)
				{
					continue;
				}
				
				g_hNavMeshAreas.Set(iNewAreaIndex, flNewLengthSoFar, NavMeshArea_PathLengthSoFar);
			}
			
			if ((NavMeshAreaIsOpen(iNewAreaIndex) || NavMeshAreaIsClosed(iNewAreaIndex)) &&
				g_hNavMeshAreas.Get(iNewAreaIndex, NavMeshArea_CostSoFar) <= iNewCostSoFar)
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
				
				g_hNavMeshAreas.Set(iNewAreaIndex, iNewCostSoFar, NavMeshArea_CostSoFar);
				g_hNavMeshAreas.Set(iNewAreaIndex, iNewCostSoFar + iNewCostRemaining, NavMeshArea_TotalCost);
				
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
				
				g_hNavMeshAreas.Set(iNewAreaIndex, iAreaIndex, NavMeshArea_Parent);
				g_hNavMeshAreas.Set(iNewAreaIndex, iNavTraverseHow, NavMeshArea_ParentHow);
			}
		}
		
		NavMeshAreaAddToClosedList(iAreaIndex);
	}
	
	return false;
}

NavMeshAreaClearSearchLists()
{
	g_iNavMeshAreaMasterMarker++;
	g_iNavMeshAreaOpenListIndex = -1;
	g_iNavMeshAreaOpenListTailIndex = -1;
}

bool NavMeshAreaIsMarked(iAreaIndex)
{
	return view_as<bool>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Marker) == g_iNavMeshAreaMasterMarker);
}

void NavMeshAreaMark(iAreaIndex)
{
	g_hNavMeshAreas.Set(iAreaIndex, g_iNavMeshAreaMasterMarker, NavMeshArea_Marker);
}

bool NavMeshAreaIsOpen(iAreaIndex)
{
	return view_as<bool>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_OpenMarker) == g_iNavMeshAreaMasterMarker);
}

bool NavMeshAreaIsOpenListEmpty()
{
	return view_as<bool>(g_iNavMeshAreaOpenListIndex == -1);
}

void NavMeshAreaAddToOpenList(int iAreaIndex)
{
	if (NavMeshAreaIsOpen(iAreaIndex)) return;
	
	g_hNavMeshAreas.Set(iAreaIndex, g_iNavMeshAreaMasterMarker, NavMeshArea_OpenMarker);
	
	if (g_iNavMeshAreaOpenListIndex == -1)
	{
		g_iNavMeshAreaOpenListIndex = iAreaIndex;
		g_iNavMeshAreaOpenListTailIndex = iAreaIndex;
		g_hNavMeshAreas.Set(iAreaIndex, -1, NavMeshArea_PrevOpenIndex);
		g_hNavMeshAreas.Set(iAreaIndex, -1, NavMeshArea_NextOpenIndex);
		return;
	}
	
	int iTotalCost = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_TotalCost);
	
	int iTempAreaIndex = -1; 
	int iLastAreaIndex = -1;
	for (iTempAreaIndex = g_iNavMeshAreaOpenListIndex; iTempAreaIndex != -1; iTempAreaIndex = g_hNavMeshAreas.Get(iTempAreaIndex, NavMeshArea_NextOpenIndex))
	{
		if (iTotalCost < g_hNavMeshAreas.Get(iTempAreaIndex, NavMeshArea_TotalCost)) break;
		iLastAreaIndex = iTempAreaIndex;
	}
	
	if (iTempAreaIndex != -1)
	{
		int iPrevOpenIndex = g_hNavMeshAreas.Get(iTempAreaIndex, NavMeshArea_PrevOpenIndex);
		g_hNavMeshAreas.Set(iAreaIndex, iPrevOpenIndex, NavMeshArea_PrevOpenIndex);
		
		if (iPrevOpenIndex != -1)
		{
			g_hNavMeshAreas.Set(iPrevOpenIndex, iAreaIndex, NavMeshArea_NextOpenIndex);
		}
		else
		{
			g_iNavMeshAreaOpenListIndex = iAreaIndex;
		}
		
		g_hNavMeshAreas.Set(iAreaIndex, iTempAreaIndex, NavMeshArea_NextOpenIndex);
		g_hNavMeshAreas.Set(iTempAreaIndex, iAreaIndex, NavMeshArea_PrevOpenIndex);
	}
	else
	{
		g_hNavMeshAreas.Set(iLastAreaIndex, iAreaIndex, NavMeshArea_NextOpenIndex);
		g_hNavMeshAreas.Set(iAreaIndex, iLastAreaIndex, NavMeshArea_PrevOpenIndex);
		
		g_hNavMeshAreas.Set(iAreaIndex, -1, NavMeshArea_NextOpenIndex);
		
		g_iNavMeshAreaOpenListTailIndex = iAreaIndex;
	}
}

stock void NavMeshAreaAddToOpenListTail(int iAreaIndex)
{
	if (NavMeshAreaIsOpen(iAreaIndex)) return;
	
	g_hNavMeshAreas.Set(iAreaIndex, g_iNavMeshAreaMasterMarker, NavMeshArea_OpenMarker);
	
	if (g_iNavMeshAreaOpenListIndex == -1)
	{
		g_iNavMeshAreaOpenListIndex = iAreaIndex;
		g_iNavMeshAreaOpenListTailIndex = iAreaIndex;
		g_hNavMeshAreas.Set(iAreaIndex, -1, NavMeshArea_PrevOpenIndex);
		g_hNavMeshAreas.Set(iAreaIndex, -1, NavMeshArea_NextOpenIndex);
		return;
	}
	
	g_hNavMeshAreas.Set(g_iNavMeshAreaOpenListTailIndex, iAreaIndex, NavMeshArea_NextOpenIndex);
	
	g_hNavMeshAreas.Set(iAreaIndex, g_iNavMeshAreaOpenListTailIndex, NavMeshArea_PrevOpenIndex);
	g_hNavMeshAreas.Set(iAreaIndex, -1, NavMeshArea_NextOpenIndex);
	
	g_iNavMeshAreaOpenListTailIndex = iAreaIndex;
}

void NavMeshAreaUpdateOnOpenList(int iAreaIndex)
{
	int iTotalCost = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_TotalCost);
	
	int iPrevIndex = -1;
	
	while ((iPrevIndex = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_PrevOpenIndex)) != -1 &&
		iTotalCost < (g_hNavMeshAreas.Get(iPrevIndex, NavMeshArea_TotalCost)))
	{
		int iOtherIndex = iPrevIndex;
		int iBeforeIndex = g_hNavMeshAreas.Get(iPrevIndex, NavMeshArea_PrevOpenIndex);
		int iAfterIndex = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_NextOpenIndex);
	
		g_hNavMeshAreas.Set(iAreaIndex, iPrevIndex, NavMeshArea_NextOpenIndex);
		g_hNavMeshAreas.Set(iAreaIndex, iBeforeIndex, NavMeshArea_PrevOpenIndex);
		
		g_hNavMeshAreas.Set(iOtherIndex, iAreaIndex, NavMeshArea_PrevOpenIndex);
		g_hNavMeshAreas.Set(iOtherIndex, iAfterIndex, NavMeshArea_NextOpenIndex);
		
		if (iBeforeIndex != -1)
		{
			g_hNavMeshAreas.Set(iBeforeIndex, iAreaIndex, NavMeshArea_NextOpenIndex);
		}
		else
		{
			g_iNavMeshAreaOpenListIndex = iAreaIndex;
		}
		
		if (iAfterIndex != -1)
		{
			g_hNavMeshAreas.Set(iAfterIndex, iOtherIndex, NavMeshArea_PrevOpenIndex);
		}
		else
		{
			g_iNavMeshAreaOpenListTailIndex = iAreaIndex;
		}
	}
}

void NavMeshAreaRemoveFromOpenList(int iAreaIndex)
{
	if (g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_OpenMarker) == 0) return;
	
	int iPrevOpenIndex = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_PrevOpenIndex);
	int iNextOpenIndex = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_NextOpenIndex);
	
	if (iPrevOpenIndex != -1)
	{
		g_hNavMeshAreas.Set(iPrevOpenIndex, iNextOpenIndex, NavMeshArea_NextOpenIndex);
	}
	else
	{
		g_iNavMeshAreaOpenListIndex = iNextOpenIndex;
	}
	
	if (iNextOpenIndex != -1)
	{
		g_hNavMeshAreas.Set(iNextOpenIndex, iPrevOpenIndex, NavMeshArea_PrevOpenIndex);
	}
	else
	{
		g_iNavMeshAreaOpenListTailIndex = iPrevOpenIndex;
	}
	
	g_hNavMeshAreas.Set(iAreaIndex, 0, NavMeshArea_OpenMarker);
}

int NavMeshAreaPopOpenList()
{
	if (g_iNavMeshAreaOpenListIndex != -1)
	{
		int iOpenListIndex = g_iNavMeshAreaOpenListIndex;
	
		NavMeshAreaRemoveFromOpenList(iOpenListIndex);
		g_hNavMeshAreas.Set(iOpenListIndex, -1, NavMeshArea_PrevOpenIndex);
		g_hNavMeshAreas.Set(iOpenListIndex, -1, NavMeshArea_NextOpenIndex);
		
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
	
	File hFile = OpenFile(sNavFilePath, "rb");
	if (hFile == null)
	{
		// Try opening it from the Valve file system.
		hFile = OpenFile(sNavFilePath, "rb", true, NULL_STRING);
	}
	
	if (hFile == null)
	{
		// Try finding the file ourselves.
		new bool:bFound = false;
		
		if (GetFeatureStatus(FeatureType_Native, "GetEngineVersion") == FeatureStatus_Available)
		{
			EngineVersion iEngineVersion = GetEngineVersion();
			
			switch (iEngineVersion)
			{
				case Engine_CSGO:
				{
					// Search addon directories.
					DirectoryListing hDir = OpenDirectory("addons");
					if (hDir != null)
					{
						LogMessage("Couldn't find .nav file in maps folder, checking addon folders...");
						
						char sFolderName[PLATFORM_MAX_PATH];
						FileType iFileType;
						while (hDir.GetNext(sFolderName, sizeof(sFolderName), iFileType))
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
						
						delete hDir;
					}
				}
				case Engine_TF2:
				{
					// Search custom directories.
					DirectoryListing hDir = OpenDirectory("custom");
					if (hDir != INVALID_HANDLE)
					{
						LogMessage("Couldn't find .nav file in maps folder, checking custom folders...");
					
						char sFolderName[PLATFORM_MAX_PATH];
						FileType iFileType;
						while (hDir.GetNext(sFolderName, sizeof(sFolderName), iFileType))
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
						
						delete hDir;
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
	
	LogMessage("Found .NAV file in %s", sNavFilePath);
	
	// Get magic number.
	int iNavMagicNumber = 0;
	int iElementsRead = ReadFileCell(hFile, iNavMagicNumber, UNSIGNED_INT_BYTE_SIZE);
	
	if (iElementsRead != 1)
	{
		delete hFile;
		LogError("Error reading magic number value from navigation mesh: %s", sNavFilePath);
		return false;
	}
	
	if (iNavMagicNumber != NAV_MAGIC_NUMBER)
	{
		delete hFile;
		LogError("Invalid magic number value from navigation mesh: %s [%p]", sNavFilePath, iNavMagicNumber);
		return false;
	}
	
	// Get the version.
	int iNavVersion;
	iElementsRead = ReadFileCell(hFile, iNavVersion, UNSIGNED_INT_BYTE_SIZE);
	
	if (iElementsRead != 1)
	{
		delete hFile;
		LogError("Error reading version number from navigation mesh: %s", sNavFilePath);
		return false;
	}
	
	if (iNavVersion < 6 || iNavVersion > 16)
	{
		delete hFile;
		LogError("Invalid version number value from navigation mesh: %s [%d]", sNavFilePath, iNavVersion);
		return false;
	}
	
	// Get the sub version, if supported.
	int iNavSubVersion = 0;
	if (iNavVersion >= 10)
	{
		ReadFileCell(hFile, iNavSubVersion, UNSIGNED_INT_BYTE_SIZE);
	}
	
	// Get the save bsp size.
	int iNavSaveBspSize = 0;
	if (iNavVersion >= 4)
	{
		ReadFileCell(hFile, iNavSaveBspSize, UNSIGNED_INT_BYTE_SIZE);
	}
	
	// Check if the nav mesh was analyzed.
	int iNavMeshAnalyzed = 0;
	if (iNavVersion >= 14)
	{
		ReadFileCell(hFile, iNavMeshAnalyzed, UNSIGNED_CHAR_BYTE_SIZE);
		LogMessage("Is mesh analyzed: %d", iNavMeshAnalyzed);
	}
	
	LogMessage("Nav version: %d; SubVersion: %d (v10+); BSPSize: %d; MagicNumber: %d", iNavVersion, iNavSubVersion, iNavSaveBspSize, iNavMagicNumber);
	
	int iPlaceCount = 0;
	ReadFileCell(hFile, iPlaceCount, UNSIGNED_SHORT_BYTE_SIZE);
	LogMessage("Place count: %d", iPlaceCount);
	
	// Parse through places.
	// TF2 doesn't use places, but CS:S does.
	for (int iPlaceIndex = 0; iPlaceIndex < iPlaceCount; iPlaceIndex++) 
	{
		int iPlaceStringSize = 0;
		ReadFileCell(hFile, iPlaceStringSize, UNSIGNED_SHORT_BYTE_SIZE);
		
		char sPlaceName[256];
		ReadFileString(hFile, sPlaceName, sizeof(sPlaceName), iPlaceStringSize);
		
		PushArrayString(g_hNavMeshPlaces, sPlaceName);
		
		//LogMessage("Parsed place \"%s\" [index: %d]", sPlaceName, iPlaceIndex);
	}
	
	// Get any unnamed areas.
	int iNavUnnamedAreas = 0;
	if (iNavVersion > 11)
	{
		ReadFileCell(hFile, iNavUnnamedAreas, UNSIGNED_CHAR_BYTE_SIZE);
		LogMessage("Has unnamed areas: %s", iNavUnnamedAreas ? "true" : "false");
	}
	
	// Get area count.
	int iAreaCount = 0;
	ReadFileCell(hFile, iAreaCount, UNSIGNED_INT_BYTE_SIZE);
	
	LogMessage("Area count: %d", iAreaCount);
	
	static float flExtentLow[2] = { 99999999.9, 99999999.9 };
	static float flExtentHigh[2] = { -99999999.9, -99999999.9 };
	bool bExtentLowX = false;
	bool bExtentLowY = false;
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
			float x1;
			float y1;
			float z1; 
			float x2; 
			float y2; 
			float z2;
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
				int iConnectionCount = 0;
				ReadFileCell(hFile, iConnectionCount, UNSIGNED_INT_BYTE_SIZE);
				
				//LogMessage("Connection count: %d", iConnectionCount);
				
				if (iConnectionCount > 0)
				{
					if (iConnectionsStartIndex == -1) iConnectionsStartIndex = iGlobalConnectionsStartIndex;
				
					for (int iConnectionIndex = 0; iConnectionIndex < iConnectionCount; iConnectionIndex++) 
					{
						iConnectionsEndIndex = iGlobalConnectionsStartIndex;
						
						int iConnectingAreaID = 0;
						ReadFileCell(hFile, iConnectingAreaID, UNSIGNED_INT_BYTE_SIZE);
						
						int iIndex = g_hNavMeshAreaConnections.Push(iConnectingAreaID);
						g_hNavMeshAreaConnections.Set(iIndex, iDirection, NavMeshConnection_Direction);
						
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
					
					float flHidingSpotX; float flHidingSpotY; float flHidingSpotZ;
					ReadFileCell(hFile, view_as<int>(flHidingSpotX), FLOAT_BYTE_SIZE);
					ReadFileCell(hFile, view_as<int>(flHidingSpotY), FLOAT_BYTE_SIZE);
					ReadFileCell(hFile, view_as<int>(flHidingSpotZ), FLOAT_BYTE_SIZE);
					
					int iHidingSpotFlags;
					ReadFileCell(hFile, iHidingSpotFlags, UNSIGNED_CHAR_BYTE_SIZE);
					
					int iIndex = g_hNavMeshAreaHidingSpots.Push(iHidingSpotID);
					g_hNavMeshAreaHidingSpots.Set(iIndex, flHidingSpotX, NavMeshHidingSpot_X);
					g_hNavMeshAreaHidingSpots.Set(iIndex, flHidingSpotY, NavMeshHidingSpot_Y);
					g_hNavMeshAreaHidingSpots.Set(iIndex, flHidingSpotZ, NavMeshHidingSpot_Z);
					g_hNavMeshAreaHidingSpots.Set(iIndex, iHidingSpotFlags, NavMeshHidingSpot_Flags);
					g_hNavMeshAreaHidingSpots.Set(iIndex, iAreaIndex, NavMeshHidingSpot_AreaIndex);
					
					iGlobalHidingSpotsStartIndex++;
					
					//LogMessage("Parsed hiding spot (%f, %f, %f) with ID [%d] and flags [%d]", flHidingSpotX, flHidingSpotY, flHidingSpotZ, iHidingSpotID, iHidingSpotFlags);
				}
			}
			
			// Get approach areas (old version, only used to read data)
			if (iNavVersion < 15)
			{
				int iApproachAreaCount = 0;
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
							
							int iIndex = g_hNavMeshAreaEncounterSpots.Push(iEncounterSpotOrderID);
							g_hNavMeshAreaEncounterSpots.Set(iIndex, flEncounterSpotParametricDistance, NavMeshEncounterSpot_ParametricDistance);
							
							iGlobalEncounterSpotsStartIndex++;
							
							//LogMessage("Encounter spot [order id %d] and [T %d]", iEncounterSpotOrderID, iEncounterSpotT);
						}
					}
					
					int iIndex = g_hNavMeshAreaEncounterPaths.Push(iEncounterFromID);
					g_hNavMeshAreaEncounterPaths.Set(iIndex, iEncounterFromDirection, NavMeshEncounterPath_FromDirection);
					g_hNavMeshAreaEncounterPaths.Set(iIndex, iEncounterToID, NavMeshEncounterPath_ToAreaIndex);
					g_hNavMeshAreaEncounterPaths.Set(iIndex, iEncounterToDirection, NavMeshEncounterPath_ToDirection);
					g_hNavMeshAreaEncounterPaths.Set(iIndex, iEncounterSpotsStartIndex, NavMeshEncounterPath_SpotsStartIndex);
					g_hNavMeshAreaEncounterPaths.Set(iIndex, iEncounterSpotsEndIndex, NavMeshEncounterPath_SpotsEndIndex);
					
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
						
						int iIndex = g_hNavMeshAreaLadderConnections.Push(iLadderConnectionID);
						g_hNavMeshAreaLadderConnections.Set(iIndex, iLadderDirection, NavMeshLadderConnection_Direction);
						
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
							
							int iIndex = g_hNavMeshAreaVisibleAreas.Push(iVisibleAreaID);
							g_hNavMeshAreaVisibleAreas.Set(iIndex, iVisibleAreaAttributes, NavMeshVisibleArea_Attributes);
							
							iGlobalVisibleAreasStartIndex++;
							
							//LogMessage("Parsed visible area [%d] with attr [%d]", iVisibleAreaID, iVisibleAreaAttributes);
						}
					}
					
					ReadFileCell(hFile, iInheritVisibilityFrom, UNSIGNED_INT_BYTE_SIZE);
					
					//LogMessage("Inherit visibilty from: %d", iInheritVisibilityFrom);
					
					ReadFileCell(hFile, unk01, UNSIGNED_INT_BYTE_SIZE);
				}
			}
			
			int iIndex = g_hNavMeshAreas.Push(iAreaID);
			g_hNavMeshAreas.Set(iIndex, iAreaFlags, NavMeshArea_Flags);
			g_hNavMeshAreas.Set(iIndex, iPlaceID, NavMeshArea_PlaceID);
			g_hNavMeshAreas.Set(iIndex, x1, NavMeshArea_X1);
			g_hNavMeshAreas.Set(iIndex, y1, NavMeshArea_Y1);
			g_hNavMeshAreas.Set(iIndex, z1, NavMeshArea_Z1);
			g_hNavMeshAreas.Set(iIndex, x2, NavMeshArea_X2);
			g_hNavMeshAreas.Set(iIndex, y2, NavMeshArea_Y2);
			g_hNavMeshAreas.Set(iIndex, z2, NavMeshArea_Z2);
			g_hNavMeshAreas.Set(iIndex, flAreaCenter[0], NavMeshArea_CenterX);
			g_hNavMeshAreas.Set(iIndex, flAreaCenter[1], NavMeshArea_CenterY);
			g_hNavMeshAreas.Set(iIndex, flAreaCenter[2], NavMeshArea_CenterZ);
			g_hNavMeshAreas.Set(iIndex, flInvDxCorners, NavMeshArea_InvDxCorners);
			g_hNavMeshAreas.Set(iIndex, flInvDyCorners, NavMeshArea_InvDyCorners);
			g_hNavMeshAreas.Set(iIndex, flNECornerZ, NavMeshArea_NECornerZ);
			g_hNavMeshAreas.Set(iIndex, flSWCornerZ, NavMeshArea_SWCornerZ);
			g_hNavMeshAreas.Set(iIndex, iConnectionsStartIndex, NavMeshArea_ConnectionsStartIndex);
			g_hNavMeshAreas.Set(iIndex, iConnectionsEndIndex, NavMeshArea_ConnectionsEndIndex);
			g_hNavMeshAreas.Set(iIndex, iHidingSpotsStartIndex, NavMeshArea_HidingSpotsStartIndex);
			g_hNavMeshAreas.Set(iIndex, iHidingSpotsEndIndex, NavMeshArea_HidingSpotsEndIndex);
			g_hNavMeshAreas.Set(iIndex, iEncounterPathsStartIndex, NavMeshArea_EncounterPathsStartIndex);
			g_hNavMeshAreas.Set(iIndex, iEncounterPathsEndIndex, NavMeshArea_EncounterPathsEndIndex);
			g_hNavMeshAreas.Set(iIndex, iLadderConnectionsStartIndex, NavMeshArea_LadderConnectionsStartIndex);
			g_hNavMeshAreas.Set(iIndex, iLadderConnectionsEndIndex, NavMeshArea_LadderConnectionsEndIndex);
			g_hNavMeshAreas.Set(iIndex, flNavCornerLightIntensityNW, NavMeshArea_CornerLightIntensityNW);
			g_hNavMeshAreas.Set(iIndex, flNavCornerLightIntensityNE, NavMeshArea_CornerLightIntensityNE);
			g_hNavMeshAreas.Set(iIndex, flNavCornerLightIntensitySE, NavMeshArea_CornerLightIntensitySE);
			g_hNavMeshAreas.Set(iIndex, flNavCornerLightIntensitySW, NavMeshArea_CornerLightIntensitySW);
			g_hNavMeshAreas.Set(iIndex, iVisibleAreasStartIndex, NavMeshArea_VisibleAreasStartIndex);
			g_hNavMeshAreas.Set(iIndex, iVisibleAreasEndIndex, NavMeshArea_VisibleAreasEndIndex);
			g_hNavMeshAreas.Set(iIndex, iInheritVisibilityFrom, NavMeshArea_InheritVisibilityFrom);
			g_hNavMeshAreas.Set(iIndex, flEarliestOccupyTimeFirstTeam, NavMeshArea_EarliestOccupyTimeFirstTeam);
			g_hNavMeshAreas.Set(iIndex, flEarliestOccupyTimeSecondTeam, NavMeshArea_EarliestOccupyTimeSecondTeam);
			g_hNavMeshAreas.Set(iIndex, unk01, NavMeshArea_unk01);
			g_hNavMeshAreas.Set(iIndex, -1, NavMeshArea_Parent);
			g_hNavMeshAreas.Set(iIndex, NUM_TRAVERSE_TYPES, NavMeshArea_ParentHow);
			g_hNavMeshAreas.Set(iIndex, 0, NavMeshArea_TotalCost);
			g_hNavMeshAreas.Set(iIndex, 0, NavMeshArea_CostSoFar);
			g_hNavMeshAreas.Set(iIndex, -1, NavMeshArea_Marker);
			g_hNavMeshAreas.Set(iIndex, -1, NavMeshArea_OpenMarker);
			g_hNavMeshAreas.Set(iIndex, -1, NavMeshArea_PrevOpenIndex);
			g_hNavMeshAreas.Set(iIndex, -1, NavMeshArea_NextOpenIndex);
			g_hNavMeshAreas.Set(iIndex, 0.0, NavMeshArea_PathLengthSoFar);
			g_hNavMeshAreas.Set(iIndex, false, NavMeshArea_Blocked);
			g_hNavMeshAreas.Set(iIndex, -1, NavMeshArea_NearSearchMarker);
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
			
			float flLadderTopX; float flLadderTopY; float flLadderTopZ; float flLadderBottomX; float flLadderBottomY; float flLadderBottomZ;
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
			
			int iIndex = g_hNavMeshLadders.Push(iLadderID);
			g_hNavMeshLadders.Set(iIndex, flLadderWidth, NavMeshLadder_Width);
			g_hNavMeshLadders.Set(iIndex, flLadderLength, NavMeshLadder_Length);
			g_hNavMeshLadders.Set(iIndex, flLadderTopX, NavMeshLadder_TopX);
			g_hNavMeshLadders.Set(iIndex, flLadderTopY, NavMeshLadder_TopY);
			g_hNavMeshLadders.Set(iIndex, flLadderTopZ, NavMeshLadder_TopZ);
			g_hNavMeshLadders.Set(iIndex, flLadderBottomX, NavMeshLadder_BottomX);
			g_hNavMeshLadders.Set(iIndex, flLadderBottomY, NavMeshLadder_BottomY);
			g_hNavMeshLadders.Set(iIndex, flLadderBottomZ, NavMeshLadder_BottomZ);
			g_hNavMeshLadders.Set(iIndex, iLadderDirection, NavMeshLadder_Direction);
			g_hNavMeshLadders.Set(iIndex, iLadderTopForwardAreaID, NavMeshLadder_TopForwardAreaIndex);
			g_hNavMeshLadders.Set(iIndex, iLadderTopLeftAreaID, NavMeshLadder_TopLeftAreaIndex);
			g_hNavMeshLadders.Set(iIndex, iLadderTopRightAreaID, NavMeshLadder_TopRightAreaIndex);
			g_hNavMeshLadders.Set(iIndex, iLadderTopBehindAreaID, NavMeshLadder_TopBehindAreaIndex);
			g_hNavMeshLadders.Set(iIndex, iLadderBottomAreaID, NavMeshLadder_BottomAreaIndex);
		}
	}
	
	g_iNavMeshMagicNumber = iNavMagicNumber;
	g_iNavMeshVersion = iNavVersion;
	g_iNavMeshSubVersion = iNavSubVersion;
	g_iNavMeshSaveBSPSize = iNavSaveBspSize;
	g_bNavMeshAnalyzed = view_as<bool>(iNavMeshAnalyzed);
	
	delete hFile;
	
	// File parsing is all done. Convert referenced IDs to array indexes for faster performance and 
	// lesser lookup time in the future.
	
	if (g_hNavMeshAreaConnections.Length > 0)
	{
		for (int iIndex = 0, iSize = g_hNavMeshAreaConnections.Length; iIndex < iSize; iIndex++)
		{
			int id = g_hNavMeshAreaConnections.Get(iIndex, NavMeshConnection_AreaIndex);
			g_hNavMeshAreaConnections.Set(iIndex, g_hNavMeshAreas.FindValue(id), NavMeshConnection_AreaIndex);
		}
	}
	
	if (g_hNavMeshAreaVisibleAreas.Length > 0)
	{
		for (int iIndex = 0, iSize = g_hNavMeshAreaVisibleAreas.Length; iIndex < iSize; iIndex++)
		{
			int id = g_hNavMeshAreaVisibleAreas.Get(iIndex, NavMeshVisibleArea_Index);
			g_hNavMeshAreaVisibleAreas.Set(iIndex, g_hNavMeshAreas.FindValue(id), NavMeshVisibleArea_Index);
		}
	}
	
	if (g_hNavMeshAreaEncounterPaths.Length > 0)
	{
		for (int iIndex = 0, iSize = g_hNavMeshAreaEncounterPaths.Length; iIndex < iSize; iIndex++)
		{
			int id = g_hNavMeshAreaEncounterPaths.Get(iIndex, NavMeshEncounterPath_FromAreaIndex);
			g_hNavMeshAreaEncounterPaths.Set(iIndex, g_hNavMeshAreas.FindValue(id), NavMeshEncounterPath_FromAreaIndex);
			
			id = g_hNavMeshAreaEncounterPaths.Get(iIndex, NavMeshEncounterPath_ToAreaIndex);
			g_hNavMeshAreaEncounterPaths.Set(iIndex, g_hNavMeshAreas.FindValue(id), NavMeshEncounterPath_ToAreaIndex);
		}
	}
	
	if (g_hNavMeshAreaEncounterSpots.Length > 0)
	{
		for (int iIndex = 0, iSize = g_hNavMeshAreaEncounterPaths.Length; iIndex < iSize; iIndex++)
		{
			int id = g_hNavMeshAreaEncounterSpots.Get(iIndex, NavMeshEncounterSpot_HidingSpotIndex);
			g_hNavMeshAreaEncounterSpots.Set(iIndex, g_hNavMeshAreaHidingSpots.FindValue(id), NavMeshEncounterSpot_HidingSpotIndex);
		}
	}
	
	if (g_hNavMeshAreaLadderConnections.Length > 0)
	{
		for (int iIndex = 0, iSize = g_hNavMeshAreaLadderConnections.Length; iIndex < iSize; iIndex++)
		{
			int id = g_hNavMeshAreaLadderConnections.Get(iIndex, NavMeshLadderConnection_LadderIndex);
			g_hNavMeshAreaLadderConnections.Set(iIndex, g_hNavMeshLadders.FindValue(id), NavMeshLadderConnection_LadderIndex);
		}
	}
	
	if (g_hNavMeshLadders.Length > 0)
	{
		for (int iLadderIndex = 0; iLadderIndex < iLadderCount; iLadderIndex++)
		{
			int iTopForwardAreaID = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_TopForwardAreaIndex);
			g_hNavMeshLadders.Set(iLadderIndex, g_hNavMeshAreas.FindValue(iTopForwardAreaID), NavMeshLadder_TopForwardAreaIndex);
			
			int iTopLeftAreaID = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_TopLeftAreaIndex);
			g_hNavMeshLadders.Set(iLadderIndex, g_hNavMeshAreas.FindValue(iTopLeftAreaID), NavMeshLadder_TopLeftAreaIndex);
			
			int iTopRightAreaID = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_TopRightAreaIndex);
			g_hNavMeshLadders.Set(iLadderIndex, g_hNavMeshAreas.FindValue(iTopRightAreaID), NavMeshLadder_TopRightAreaIndex);
			
			int iTopBehindAreaID = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_TopBehindAreaIndex);
			g_hNavMeshLadders.Set(iLadderIndex, g_hNavMeshAreas.FindValue(iTopBehindAreaID), NavMeshLadder_TopBehindAreaIndex);
			
			int iBottomAreaID = GetArrayCell(g_hNavMeshLadders, iLadderIndex, NavMeshLadder_BottomAreaIndex);
			g_hNavMeshLadders.Set(iLadderIndex, g_hNavMeshAreas.FindValue(iBottomAreaID), NavMeshLadder_BottomAreaIndex);
		}
	}
	
	return true;
}

void NavMeshDestroy()
{
	g_hNavMeshPlaces.Clear();
	g_hNavMeshAreas.Clear();
	g_hNavMeshAreaConnections.Clear();
	g_hNavMeshAreaHidingSpots.Clear();
	g_hNavMeshAreaEncounterPaths.Clear();
	g_hNavMeshAreaEncounterSpots.Clear();
	g_hNavMeshAreaLadderConnections.Clear();
	g_hNavMeshAreaVisibleAreas.Clear();
	g_hNavMeshLadders.Clear();
	
	g_hNavMeshGrid.Clear();
	g_hNavMeshGridLists.Clear();
	
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
	g_hNavMeshGrid.Clear();
	g_hNavMeshGridLists.Clear();
	
	g_flNavMeshMinX = flMinX;
	g_flNavMeshMinY = flMinY;
	
	g_iNavMeshGridSizeX = IntCast((flMaxX - flMinX) / g_flNavMeshGridCellSize) + 1;
	g_iNavMeshGridSizeY = IntCast((flMaxY - flMinY) / g_flNavMeshGridCellSize) + 1;
	
	int iArraySize = g_iNavMeshGridSizeX * g_iNavMeshGridSizeY;
	g_hNavMeshGrid.Resize(iArraySize);
	
	for (int iGridIndex = 0; iGridIndex < iArraySize; iGridIndex++)
	{
		g_hNavMeshGrid.Set(iGridIndex, -1, NavMeshGrid_ListStartIndex);
		g_hNavMeshGrid.Set(iGridIndex, -1, NavMeshGrid_ListEndIndex);
	}
}

void NavMeshGridFinalize()
{
	bool bAllIn = true;
	
	SortADTArrayCustom(g_hNavMeshGridLists, SortNavMeshGridLists);
	
	for (int iGridIndex = 0, iSize = g_hNavMeshGrid.Length; iGridIndex < iSize; iGridIndex++)
	{
		int iStartIndex = -1;
		int iEndIndex = -1;
		NavMeshGridGetListBounds(iGridIndex, iStartIndex, iEndIndex);
		g_hNavMeshGrid.Set(iGridIndex, iStartIndex, NavMeshGrid_ListStartIndex);
		g_hNavMeshGrid.Set(iGridIndex, iEndIndex, NavMeshGrid_ListEndIndex);
		
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
					LogError("Warning! Invalid nav area found in list of grid index %d!", iGridIndex);
					bAllIn = false;
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
	}
}

// The following functions should ONLY be called during NavMeshLoad(), due to displacement of
// array indexes!

// Some things to take into account: because we're adding things into the
// array, it's inevitable that the indexes will change over time. Therefore,
// we can't assign array indexes while this function is running, since it
// will shift preceding array indexes.

// The array indexes should be assigned afterwards using NavMeshGridFinalize().

public int SortNavMeshGridLists(int index1, int index2, Handle ar, Handle hndl)
{
	ArrayList array = view_as<ArrayList>(ar);
	int iGridIndex1 = array.Get(index1, NavMeshGridList_Owner);
	int iGridIndex2 = array.Get(index2, NavMeshGridList_Owner);
	
	if (iGridIndex1 < iGridIndex2) return -1;
	else if (iGridIndex1 > iGridIndex2) return 1;
	return 0;
}

void NavMeshGridAddAreaToList(int iGridIndex, int iAreaIndex)
{
	int iIndex = g_hNavMeshGridLists.Push(iAreaIndex);
	
	if (iIndex != -1)
	{
		g_hNavMeshGridLists.Set(iIndex, iGridIndex, NavMeshGridList_Owner);
	}
}

void NavMeshGridGetListBounds(int iGridIndex, int &iStartIndex, int &iEndIndex)
{
	iStartIndex = -1;
	iEndIndex = -1;
	
	for (int i = 0, iSize = g_hNavMeshGridLists.Length; i < iSize; i++)
	{
		if (g_hNavMeshGridLists.Get(i, NavMeshGridList_Owner) == iGridIndex)
		{
			if (iStartIndex == -1) iStartIndex = i;
			iEndIndex = i;
		}
	}
}

void NavMeshAddAreaToGrid(iAreaIndex)
{
	float flExtentLow[2]; float flExtentHigh[2];
//	NavMeshAreaGetExtentLow(iAreaIndex, flExtentLow);
//	NavMeshAreaGetExtentHigh(iAreaIndex, flExtentHigh);
	
	flExtentLow[0] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_X1));
	flExtentLow[1] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Y1));
	flExtentHigh[0] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_X2));
	flExtentHigh[1] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Y2));
	
	int loX = NavMeshWorldToGridX(flExtentLow[0]);
	int loY = NavMeshWorldToGridY(flExtentLow[1]);
	int hiX = NavMeshWorldToGridX(flExtentHigh[0]);
	int hiY = NavMeshWorldToGridY(flExtentHigh[1]);
	
	/*
	decl String:path[PLATFORM_MAX_PATH];
	BuildPath(Path_SM,path,PLATFORM_MAX_PATH, "navtest.txt");
	new Handle:hFile = OpenFile(path, "a");
	
	WriteFileLine(hFile, "[%d]", g_hNavMeshAreas.Get(iAreaIndex));
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
	new y = IntCast((flWY - g_flNavMeshMinY) / g_flNavMeshGridCellSize);
	
	if (y < 0) y = 0;
	else if (y >= g_iNavMeshGridSizeY) 
	{
	//	PrintToServer("NavMeshWorldToGridY: clamping y (%d) down to %d", y, g_iNavMeshGridSizeY - 1);
		y = g_iNavMeshGridSizeY - 1;
	}
	
	return y;
}

stock ArrayStack NavMeshGridGetAreas(x, y)
{
	int iGridIndex = x + y * g_iNavMeshGridSizeX;
	int iListStartIndex = g_hNavMeshGrid.Get(iGridIndex, NavMeshGrid_ListStartIndex);
	int iListEndIndex = g_hNavMeshGrid.Get(iGridIndex, NavMeshGrid_ListEndIndex);
	
	if (iListStartIndex == -1) return null;
	
	ArrayStack hStack = new ArrayStack();
	
	for (int i = iListStartIndex; i <= iListEndIndex; i++)
	{
		hStack.Push(g_hNavMeshGridLists.Get(i, NavMeshGridList_AreaIndex));
	}
	
	return hStack;
}

stock int NavMeshGetNearestArea(float flPos[3], bool bAnyZ=false, float flMaxDist=10000.0, bool bCheckLOS=false, bool bCheckGround=true, int iTeam=-2)
{
	if (g_hNavMeshGridLists.Length == 0) return -1;
	
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
				
				ArrayStack hAreas = NavMeshGridGetAreas(x, y);
				if (hAreas != null)
				{
					while (!hAreas.Empty)
					{
						int iAreaIndex = -1;
						PopStackCell(hAreas, iAreaIndex);
						
						int iAreaNearSearchMarker = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_NearSearchMarker);
						if (iAreaNearSearchMarker == iSearchMarker) continue;
						
						if (g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Blocked)) 
						{
							continue;
						}
						
						g_hNavMeshAreas.Set(iAreaIndex, iSearchMarker, NavMeshArea_NearSearchMarker);
						
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
							delete hTrace;
							
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
								delete hTrace;
								
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
							delete hTrace;
							
							if (flFraction != 1.0)
							{
								continue;
							}
						}
						
						flClosestDistSq = flDistSq;
						iClosestAreaIndex = iAreaIndex;
						
						iShiftLimit = iShift + 1;
					}
					
					delete hAreas;
				}
			}
		}
	}
	
	return iClosestAreaIndex;
}

stock void NavMeshAreaGetClosestPointOnArea(int iAreaIndex, const float flPos[3], float flClose[3])
{
	float x; float y; float z;
	
	float flExtentLow[3]; float flExtentHigh[3];
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

stock int NavMeshAreaGetID(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return -1;
	
	return g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_ID);
}

stock int NavMeshFindAreaByID(int iAreaID)
{
	return g_hNavMeshAreas.FindValue(iAreaID);
}

stock int NavMeshAreaGetFlags(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return 0;
	
	return g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Flags);
}

stock void NavMeshAreaGetPlace(int iAreaIndex, char[] buffer, int maxlen)
{
	int placeIndex = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_PlaceID);

	if (placeIndex < 0 || placeIndex >= g_hNavMeshPlaces.Length)
	{
		strcopy(buffer, maxlen, "");
		return;
	}
	
	g_hNavMeshPlaces.GetString(placeIndex, buffer, maxlen);
}

stock bool NavMeshAreaGetCenter(int iAreaIndex, float flBuffer[3])
{
	if (!g_bNavMeshBuilt) return false;
	
	flBuffer[0] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_CenterX));
	flBuffer[1] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_CenterY));
	flBuffer[2] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_CenterZ));
	return true;
}

stock void NavMeshAreaGetCorner(int iAreaIndex, NavCornerType corner, float buffer[3])
{
	switch (corner)
	{
		case NAV_CORNER_NORTH_WEST:
		{
			NavMeshAreaGetExtentLow( iAreaIndex, buffer );
		}
		case NAV_CORNER_NORTH_EAST:
		{
			buffer[0] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_X2));
			buffer[1] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Y1));
			buffer[2] = NavMeshAreaGetNECornerZ(iAreaIndex);
		}
		case NAV_CORNER_SOUTH_WEST:
		{
			buffer[0] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_X1));
			buffer[1] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Y2));
			buffer[2] = NavMeshAreaGetSWCornerZ(iAreaIndex);
		}
		case NAV_CORNER_SOUTH_EAST:
		{
			NavMeshAreaGetExtentHigh( iAreaIndex, buffer );
		}
	}
}

stock void NavMeshAreaGetRandomPoint(int iAreaIndex, float buffer[3])
{
	float extentLow[3];
	float extentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, extentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, extentHigh);
	
	buffer[0] = GetRandomFloat(extentLow[0], extentHigh[0]);
	buffer[1] = GetRandomFloat(extentLow[1], extentHigh[1]);
	buffer[2] = NavMeshAreaGetZ(iAreaIndex, buffer);
}

stock ArrayStack NavMeshAreaGetAdjacentList(int iAreaIndex, int iNavDirection)
{
	if (!g_bNavMeshBuilt) return null;
	
	int iConnectionsStartIndex = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_ConnectionsStartIndex);
	if (iConnectionsStartIndex == -1) return null;
	
	int iConnectionsEndIndex = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_ConnectionsEndIndex);
	
	ArrayStack hStack = new ArrayStack();
	
	for (int i = iConnectionsStartIndex; i <= iConnectionsEndIndex; i++)
	{
		if (g_hNavMeshAreaConnections.Get(i, NavMeshConnection_Direction) == iNavDirection)
		{
			hStack.Push(g_hNavMeshAreaConnections.Get(i, NavMeshConnection_AreaIndex));
		}
	}
	
	return hStack;
}

stock ArrayStack NavMeshAreaGetLadderList(int iAreaIndex, int iLadderDir)
{
	if (!g_bNavMeshBuilt) return null;
	
	int iLadderConnectionsStartIndex = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_LadderConnectionsStartIndex);
	if (iLadderConnectionsStartIndex == -1) return null;
	
	int iLadderConnectionsEndIndex = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_LadderConnectionsEndIndex);
	
	ArrayStack hStack = new ArrayStack();
	
	for (int i = iLadderConnectionsStartIndex; i <= iLadderConnectionsEndIndex; i++)
	{
		if (g_hNavMeshAreaLadderConnections.Get(i, NavMeshLadderConnection_Direction) == iLadderDir)
		{
			hStack.Push(g_hNavMeshAreaLadderConnections.Get(i, NavMeshLadderConnection_LadderIndex));
		}
	}
	
	return hStack;
}

stock ArrayStack NavMeshAreaGetHidingSpots(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return null;
	
	int startIndex = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_HidingSpotsStartIndex);
	if (startIndex == -1) return null;
	
	int endIndex = g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_HidingSpotsEndIndex);
	
	ArrayStack hStack = new ArrayStack();
	for (int i = startIndex; i <= endIndex; i++) hStack.Push(i);
	
	return hStack;
}

stock int NavMeshAreaGetTotalCost(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return 0;
	
	return g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_TotalCost);
}

stock int NavMeshAreaGetCostSoFar(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return 0;
	
	return g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_CostSoFar);
}

stock int NavMeshAreaGetParent(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return -1;
	
	return g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Parent);
}

stock int NavMeshAreaGetParentHow(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return NUM_TRAVERSE_TYPES;
	
	return g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_ParentHow);
}

stock void NavMeshAreaSetParent(int iAreaIndex, int iParentAreaIndex)
{
	if (!g_bNavMeshBuilt) return;
	
	g_hNavMeshAreas.Set(iAreaIndex, iParentAreaIndex, NavMeshArea_Parent);
}

stock void NavMeshAreaSetParentHow(int iAreaIndex, int iParentHow)
{
	if (!g_bNavMeshBuilt) return;
	
	g_hNavMeshAreas.Set(iAreaIndex, iParentHow, NavMeshArea_ParentHow);
}

stock bool NavMeshAreaGetExtentLow(int iAreaIndex, float flBuffer[3])
{
	if (!g_bNavMeshBuilt) return false;
	
	flBuffer[0] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_X1));
	flBuffer[1] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Y1));
	flBuffer[2] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Z1));
	return true;
}

stock bool NavMeshAreaGetExtentHigh(int iAreaIndex, float flBuffer[3])
{
	if (!g_bNavMeshBuilt) return false;
	
	flBuffer[0] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_X2));
	flBuffer[1] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Y2));
	flBuffer[2] = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_Z2));
	return true;
}

stock bool NavMeshAreaIsOverlappingPoint(int iAreaIndex, const float flPos[3], float flTolerance)
{
	if (!g_bNavMeshBuilt) return false;
	
	float flExtentLow[3]; float flExtentHigh[3];
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

stock bool NavMeshAreaIsOverlappingArea(int iAreaIndex, int iTargetAreaIndex)
{
	if (!g_bNavMeshBuilt) return false;
	
	float flExtentLow[3]; float flExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flExtentHigh);
	
	float flTargetExtentLow[3]; float flTargetExtentHigh[3];
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
	return view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_NECornerZ));
}

stock float NavMeshAreaGetSWCornerZ(int iAreaIndex)
{
	if (!g_bNavMeshBuilt) return 0.0;
	return view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_SWCornerZ));
}

stock float NavMeshAreaGetZ(int iAreaIndex, const float flPos[3])
{
	if (!g_bNavMeshBuilt) return 0.0;
	
	float flExtentLow[3]; float flExtentHigh[3];
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
	
	float flInvDxCorners = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_InvDxCorners));
	float flInvDyCorners = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_InvDyCorners));
	
	float flNECornerZ = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_NECornerZ));
	
	if (flInvDxCorners == 0.0 || flInvDyCorners == 0.0)
	{
		return flNECornerZ;
	}
	
	float flExtentLow[3]; float flExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flExtentHigh);

	float u = (x - flExtentLow[0]) * flInvDxCorners;
	float v = (y - flExtentLow[1]) * flInvDyCorners;
	
	u = FloatClamp(u, 0.0, 1.0);
	v = FloatClamp(v, 0.0, 1.0);
	
	float flSWCornerZ = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_SWCornerZ));
	
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
	
	for (new i = 0, iSize = g_hNavMeshAreas.Length; i < iSize; i++)
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

stock bool NavMeshAreaComputePortal(int iAreaIndex, int iAreaToIndex, int iNavDirection, float flCenter[3], float &flHalfWidth)
{
	if (!g_bNavMeshBuilt) return false;
	
	float flAreaExtentLow[3]; float flAreaExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flAreaExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flAreaExtentHigh);
	
	float flAreaToExtentLow[3]; float flAreaToExtentHigh[3];
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

stock bool NavMeshAreaIsConnected(int iAreaIndex, int iTargetAreaIndex, int iNavDirection)
{
	if (iAreaIndex == iTargetAreaIndex) return true;
	
	if (iNavDirection == NAV_DIR_COUNT)
	{
		for (int dir = 0; dir < NAV_DIR_COUNT; dir++)
		{
			if (NavMeshAreaIsConnected(iAreaIndex, iTargetAreaIndex, dir))
			{
				return true;
			}
		}
		
		// TODO: Check ladder connections.
	}
	else
	{
		ArrayStack areas = NavMeshAreaGetAdjacentList(iAreaIndex, iNavDirection);
		if (areas != null)
		{
			if (!areas.Empty)
			{
				while (!areas.Empty)
				{
					int tempAreaIndex = -1;
					PopStackCell(areas, tempAreaIndex);
					
					if (tempAreaIndex == iTargetAreaIndex)
					{
						delete areas;
						return true;
					}
				}
			}
			
			delete areas;
		}
	}
	
	return false;
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

stock bool NavMeshAreaComputeClosestPointInPortal(int iAreaIndex, int iAreaToIndex, int iNavDirection, const float flFromPos[3], float flClosestPos[3])
{
	if (!g_bNavMeshBuilt) return false;
	
	static float flMargin = 25.0; // GenerationStepSize = 25.0;
	
	float flAreaExtentLow[3]; float flAreaExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flAreaExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flAreaExtentHigh);
	
	float flAreaToExtentLow[3]; float flAreaToExtentHigh[3];
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
	
	float flExtentLow[3]; float flExtentHigh[3];
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
	
	float flExtentLow[3]; float flExtentHigh[3];
	NavMeshAreaGetExtentLow(iAreaIndex, flExtentLow);
	NavMeshAreaGetExtentHigh(iAreaIndex, flExtentHigh);

	float flTestPos[3];
	flTestPos[0] = FloatClamp(flPos[0], flExtentLow[0], flExtentHigh[0]);
	flTestPos[1] = FloatClamp(flPos[1], flExtentLow[1], flExtentHigh[1]);
	flTestPos[2] = flPos[2];
	
	float dX = (flTestPos[0] - flExtentLow[0]) / (flExtentHigh[0] - flExtentLow[0]);
	float dY = (flTestPos[1] - flExtentLow[1]) / (flExtentHigh[1] - flExtentLow[1]);
	
	float flCornerLightIntensityNW = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_CornerLightIntensityNW));
	float flCornerLightIntensityNE = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_CornerLightIntensityNE));
	float flCornerLightIntensitySW = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_CornerLightIntensitySW));
	float flCornerLightIntensitySE = view_as<float>(g_hNavMeshAreas.Get(iAreaIndex, NavMeshArea_CornerLightIntensitySE));
	
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

stock bool NavMeshAreaIsEdge(int iAreaIndex, int iNavDirection)
{
	ArrayStack hConnections = NavMeshAreaGetAdjacentList(iAreaIndex, iNavDirection);
	if (hConnections == null || hConnections.Empty)
	{
		if (hConnections != null) delete hConnections;
		return true;
	}
	
	delete hConnections;
	return false;
}

stock float NavMeshLadderGetWidth(int iLadderIndex)
{
	if (!g_bNavMeshBuilt) return 0.0;
	
	return view_as<float>(g_hNavMeshLadders.Get(iLadderIndex, NavMeshLadder_Width));
}

stock float NavMeshLadderGetLength(int iLadderIndex)
{
	if (!g_bNavMeshBuilt) return 0.0;
	
	return view_as<float>(g_hNavMeshLadders.Get(iLadderIndex, NavMeshLadder_Length));
}

stock int NavMeshGetArea(const float flPos[3], float flBeneathLimit=120.0)
{
	if (!g_bNavMeshBuilt) return -1;
	
	int x = NavMeshWorldToGridX(flPos[0]);
	int y = NavMeshWorldToGridY(flPos[1]);
	
	ArrayStack hAreas = NavMeshGridGetAreas(x, y);
	
	int iUseAreaIndex = -1;
	float flUseZ = -99999999.9;
	float flTestPos[3];
	flTestPos[0] = flPos[0];
	flTestPos[1] = flPos[1];
	flTestPos[2] = flPos[2] + 5.0;
	
	if (hAreas != null)
	{
		while (!hAreas.Empty)
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
		
		delete hAreas;
	}
	
	return iUseAreaIndex;
}

stock bool NavMeshGetGroundHeight(const float flPos[3], float &flHeight, float flNormal[3])
{
	static float flMaxOffset = 100.0;
	
	float flTo[3]; float flFrom[3];
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
		delete hTrace;
		
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

stock int NavSpotEncounterGetFrom(int spotEncounterIndex)
{
	return g_hNavMeshAreaEncounterPaths.Get(spotEncounterIndex, NavMeshEncounterPath_FromAreaIndex);
}

stock int NavSpotEncounterGetFromDirection(int spotEncounterIndex)
{
	return g_hNavMeshAreaEncounterPaths.Get(spotEncounterIndex, NavMeshEncounterPath_FromDirection);
}

stock int NavSpotEncounterGetTo(int spotEncounterIndex)
{
	return g_hNavMeshAreaEncounterPaths.Get(spotEncounterIndex, NavMeshEncounterPath_ToAreaIndex);
}

stock int NavSpotEncounterGetToDirection(int spotEncounterIndex)
{
	return g_hNavMeshAreaEncounterPaths.Get(spotEncounterIndex, NavMeshEncounterPath_ToDirection);
}

stock ArrayStack NavSpotEncounterGetSpots(int spotEncounterIndex)
{
	int startIndex = g_hNavMeshAreaEncounterPaths.Get(spotEncounterIndex, NavMeshEncounterPath_SpotsStartIndex);
	if (startIndex == -1) return null;
	
	ArrayStack buffer = new ArrayStack();
	int endIndex = g_hNavMeshAreaEncounterPaths.Get(spotEncounterIndex, NavMeshEncounterPath_SpotsEndIndex);
	for (int i = startIndex; i <= endIndex; i++)
	{
		buffer.Push(i);
	}
	
	return buffer;
}

stock int NavSpotOrderGetHidingSpot(int spotOrderIndex)
{
	return g_hNavMeshAreaEncounterSpots.Get(spotOrderIndex, NavMeshEncounterSpot_HidingSpotIndex);
}

stock int NavSpotOrderGetParametricDistance(int spotOrderIndex)
{
	return g_hNavMeshAreaEncounterSpots.Get(spotOrderIndex, NavMeshEncounterSpot_ParametricDistance);
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
//	==================================
//	API
//	==================================

public int Native_NavMeshExists(Handle plugin, int numParams)
{
	return g_bNavMeshBuilt;
}

public int Native_NavMeshGetMagicNumber(Handle plugin, int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve magic number because the nav mesh doesn't exist!");
		return -1;
	}
	
	return g_iNavMeshMagicNumber;
}

public int Native_NavMeshGetVersion(Handle plugin, int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve version because the nav mesh doesn't exist!");
		return -1;
	}
	
	return g_iNavMeshVersion;
}

public int Native_NavMeshGetSubVersion(Handle plugin, int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve subversion because the nav mesh doesn't exist!");
		return -1;
	}
	
	return g_iNavMeshSubVersion;
}

public int Native_NavMeshGetSaveBSPSize(Handle plugin, int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve save BSP size because the nav mesh doesn't exist!");
		return -1;
	}
	
	return g_iNavMeshSaveBSPSize;
}

public int Native_NavMeshIsAnalyzed(Handle plugin, int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve analysis state because the nav mesh doesn't exist!");
		return 0;
	}
	
	return g_bNavMeshAnalyzed;
}

public int Native_NavMeshGetPlaces(Handle plugin, int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve place list because the nav mesh doesn't exist!");
		return 0;
	}
	
	return view_as<int>(g_hNavMeshPlaces);
}

public int Native_NavMeshGetAreas(Handle plugin, int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve area list because the nav mesh doesn't exist!");
		return 0;
	}
	
	return view_as<int>(g_hNavMeshAreas);
}

public int Native_NavMeshGetLadders(Handle plugin, int numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve ladder list because the nav mesh doesn't exist!");
		return 0;
	}
	
	return view_as<int>(g_hNavMeshLadders);
}

public int Native_NavMeshCollectSurroundingAreas(Handle plugin, int numParams)
{
	ArrayStack hTarget = view_as<ArrayStack>(GetNativeCell(1));
	ArrayStack hDummy = NavMeshCollectSurroundingAreas(view_as<int>(GetNativeCell(2)), view_as<float>(GetNativeCell(3)), view_as<float>(GetNativeCell(4)), view_as<float>(GetNativeCell(5)));
	
	if (hDummy != null)
	{
		while (!IsStackEmpty(hDummy))
		{
			int iAreaIndex = -1;
			PopStackCell(hDummy, iAreaIndex);
			hTarget.Push(iAreaIndex);
		}
		
		delete hDummy;
	}
}

public int Native_NavMeshBuildPath(Handle plugin, int numParams)
{
	float flGoalPos[3];
	GetNativeArray(3, flGoalPos, 3);
	
	int iClosestIndex = view_as<int>(GetNativeCellRef(6));
	
	bool bResult = NavMeshBuildPath(view_as<int>(GetNativeCell(1)), 
		view_as<int>(GetNativeCell(2)), 
		flGoalPos,
		plugin,
		view_as<NavPathCostFunctor>(GetNativeFunction(4)),
		GetNativeCell(5),
		iClosestIndex,
		view_as<float>(GetNativeCell(7)),
		view_as<float>(GetNativeCell(8)));
		
	SetNativeCellRef(6, iClosestIndex);
	return bResult;
}

public int Native_NavMeshGetArea(Handle plugin, int numParams)
{
	float flPos[3];
	GetNativeArray(1, flPos, 3);

	return NavMeshGetArea(flPos, view_as<float>(GetNativeCell(2)));
}

public int Native_NavMeshGetNearestArea(Handle plugin, int numParams)
{
	float flPos[3];
	GetNativeArray(1, flPos, 3);
	
	return NavMeshGetNearestArea(flPos, view_as<bool>(GetNativeCell(2)), view_as<float>(GetNativeCell(3)), view_as<bool>(GetNativeCell(4)), view_as<bool>(GetNativeCell(5)), GetNativeCell(6));
}

public int Native_NavMeshFindHidingSpotByID(Handle plugin, int numParams)
{
	return g_hNavMeshAreaHidingSpots.FindValue(GetNativeCell(1));
}

public int Native_NavMeshGetRandomHidingSpot(Handle plugin, int numParams)
{
	return GetRandomInt(0, g_hNavMeshAreaHidingSpots.Length - 1);
}

public int Native_NavMeshWorldToGridX(Handle plugin, int numParams)
{
	return NavMeshWorldToGridX(view_as<float>(GetNativeCell(1)));
}

public int Native_NavMeshWorldToGridY(Handle plugin, int numParams)
{
	return NavMeshWorldToGridY(view_as<float>(GetNativeCell(1)));
}

public int Native_NavMeshGridGetAreas(Handle plugin, int numParams)
{
	ArrayStack hTarget = view_as<ArrayStack>(GetNativeCell(1));
	ArrayStack hDummy = NavMeshGridGetAreas(GetNativeCell(2), GetNativeCell(3));
	
	if (hDummy != null)
	{
		while (!hDummy.Empty)
		{
			int iAreaIndex = -1;
			PopStackCell(hDummy, iAreaIndex);
			hTarget.Push(iAreaIndex);
		}
		
		delete hDummy;
	}
}

public int Native_NavMeshGetGridSizeX(Handle plugin, int numParams)
{
	return g_iNavMeshGridSizeX;
}

public int Native_NavMeshGetGridSizeY(Handle plugin, int numParams)
{
	return g_iNavMeshGridSizeY;
}

public int Native_NavMeshAreaGetClosestPointOnArea(Handle plugin, int numParams)
{
	float flPos[3]; float flClose[3];
	GetNativeArray(2, flPos, 3);
	NavMeshAreaGetClosestPointOnArea(GetNativeCell(1), flPos, flClose);
	SetNativeArray(3, flClose, 3);
}

public int Native_NavMeshGetGroundHeight(Handle plugin, int numParams)
{
	float flPos[3]; float flNormal[3];
	GetNativeArray(1, flPos, 3);
	float flHeight = view_as<float>(GetNativeCellRef(2));
	bool bResult = NavMeshGetGroundHeight(flPos, flHeight, flNormal);
	SetNativeCellRef(2, flHeight);
	SetNativeArray(3, flNormal, 3);
	return bResult;
}

public int Native_NavMeshFindAreaByID(Handle plugin, int numParams)
{
	return NavMeshFindAreaByID(GetNativeCell(1));
}

public int Native_NavMeshAreaGetMasterMarker(Handle plugin, int numParams)
{
	return g_iNavMeshAreaMasterMarker;
}

public int Native_NavMeshAreaChangeMasterMarker(Handle plugin, int numParams)
{
	g_iNavMeshAreaMasterMarker++;
}

public int Native_NavMeshAreaGetID(Handle plugin, int numParams)
{
	return NavMeshAreaGetID(GetNativeCell(1));
}

public int Native_NavMeshAreaGetFlags(Handle plugin, int numParams)
{
	return NavMeshAreaGetFlags(GetNativeCell(1));
}

public int Native_NavMeshAreaGetPlace(Handle plugin, int numParams)
{
	int maxlen = GetNativeCell(3);
	char[] buffer = new char[maxlen];
	GetNativeString(2, buffer, maxlen);
	NavMeshAreaGetPlace(GetNativeCell(1), buffer, maxlen);
	SetNativeString(2, buffer, maxlen);
}

public int Native_NavMeshAreaGetCenter(Handle plugin, int numParams)
{
	float flResult[3];
	if (NavMeshAreaGetCenter(GetNativeCell(1), flResult))
	{
		SetNativeArray(2, flResult, 3);
		return true;
	}
	
	return false;
}

public int Native_NavMeshAreaGetAdjacentList(Handle plugin, int numParams)
{
	ArrayStack hTarget = view_as<ArrayStack>(GetNativeCell(1));
	ArrayStack hDummy = NavMeshAreaGetAdjacentList(GetNativeCell(2), GetNativeCell(3));
	
	if (hDummy != null)
	{
		while (!hDummy.Empty)
		{
			new iAreaIndex = -1;
			PopStackCell(hDummy, iAreaIndex);
			hTarget.Push(iAreaIndex);
		}
		
		delete hDummy;
	}
}

public int Native_NavMeshAreaGetLadderList(Handle plugin, int numParams)
{
	ArrayStack hTarget = view_as<ArrayStack>(GetNativeCell(1));
	ArrayStack hDummy = NavMeshAreaGetLadderList(GetNativeCell(2), GetNativeCell(3));
	
	if (hDummy != null)
	{
		while (!IsStackEmpty(hDummy))
		{
			int iAreaIndex = -1;
			PopStackCell(hDummy, iAreaIndex);
			PushStackCell(hTarget, iAreaIndex);
		}
		
		delete hDummy;
	}
}

public int Native_NavMeshAreaGetHidingSpots(Handle plugin, int numParams)
{
	ArrayStack hTarget = view_as<ArrayStack>(GetNativeCell(1));
	ArrayStack hDummy = NavMeshAreaGetHidingSpots(GetNativeCell(2));
	
	if (hDummy != null)
	{
		while (!IsStackEmpty(hDummy))
		{
			int iAreaIndex = -1;
			PopStackCell(hDummy, iAreaIndex);
			PushStackCell(hTarget, iAreaIndex);
		}
		
		delete hDummy;
	}
}

public int Native_NavMeshAreaGetTotalCost(Handle plugin, int numParams)
{
	return NavMeshAreaGetTotalCost(GetNativeCell(1));
}

public int Native_NavMeshAreaGetCostSoFar(Handle plugin, int numParams)
{
	return NavMeshAreaGetCostSoFar(GetNativeCell(1));
}

public int Native_NavMeshAreaGetParent(Handle plugin, int numParams)
{
	return NavMeshAreaGetParent(GetNativeCell(1));
}

public int Native_NavMeshAreaGetParentHow(Handle plugin, int numParams)
{
	return NavMeshAreaGetParentHow(GetNativeCell(1));
}

public int Native_NavMeshAreaSetParent(Handle plugin, int numParams)
{
	NavMeshAreaSetParent(GetNativeCell(1), GetNativeCell(2));
}

public int Native_NavMeshAreaSetParentHow(Handle plugin, int numParams)
{
	NavMeshAreaSetParentHow(GetNativeCell(1), GetNativeCell(2));
}

public int Native_NavMeshAreaGetExtentLow(Handle plugin, int numParams)
{
	float flExtent[3];
	if (NavMeshAreaGetExtentLow(GetNativeCell(1), flExtent))
	{
		SetNativeArray(2, flExtent, 3);
		return true;
	}
	
	return false;
}

public int Native_NavMeshAreaGetExtentHigh(Handle plugin, int numParams)
{
	float flExtent[3];
	if (NavMeshAreaGetExtentHigh(GetNativeCell(1), flExtent))
	{
		SetNativeArray(2, flExtent, 3);
		return true;
	}
	
	return false;
}

public int Native_NavMeshAreaIsOverlappingPoint(Handle plugin, int numParams)
{
	float flPos[3];
	GetNativeArray(2, flPos, 3);
	
	return NavMeshAreaIsOverlappingPoint(GetNativeCell(1), flPos, view_as<float>(GetNativeCell(3)));
}

public int Native_NavMeshAreaIsOverlappingArea(Handle plugin, int numParams)
{
	return NavMeshAreaIsOverlappingArea(GetNativeCell(1), GetNativeCell(2));
}

public int Native_NavMeshAreaGetNECornerZ(Handle plugin, int numParams)
{
	return view_as<int>(NavMeshAreaGetNECornerZ(GetNativeCell(1)));
}

public int Native_NavMeshAreaGetSWCornerZ(Handle plugin, int numParams)
{
	return view_as<int>(NavMeshAreaGetSWCornerZ(GetNativeCell(1)));
}

public int Native_NavMeshAreaGetCorner(Handle plugin, int numParams)
{
	float buffer[3];
	GetNativeArray(3, buffer, 3);
	NavMeshAreaGetCorner(GetNativeCell(1), view_as<NavCornerType>(GetNativeCell(2)), buffer);
	SetNativeArray(3, buffer, 3);
}

public int Native_NavMeshAreaGetZ(Handle plugin, int numParams)
{
	float flPos[3];
	GetNativeArray(2, flPos, 3);

	return view_as<int>(NavMeshAreaGetZ(GetNativeCell(1), flPos));
}

public int Native_NavMeshAreaGetZFromXAndY(Handle plugin, int numParams)
{
	return view_as<int>(NavMeshAreaGetZFromXAndY(GetNativeCell(1), view_as<float>(GetNativeCell(2)), view_as<float>(GetNativeCell(3))));
}

public int Native_NavMeshAreaIsEdge(Handle plugin, int numParams)
{
	return NavMeshAreaIsEdge(GetNativeCell(1), GetNativeCell(2));
}

public int Native_NavMeshAreaContains(Handle plugin, int numParams)
{
	float flPos[3];
	GetNativeArray(2, flPos, 3);

	return NavMeshAreaContains(GetNativeCell(1), flPos);
}

public int Native_NavMeshAreaGetRandomPoint(Handle plugin, int numParams)
{
	float buffer[3];
	GetNativeArray(2, buffer, 3);
	NavMeshAreaGetRandomPoint(GetNativeCell(1), buffer);
	SetNativeArray(2, buffer, 3);
}

public int Native_NavMeshAreaComputePortal(Handle plugin, int numParams)
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

public int Native_NavMeshAreaIsConnected(Handle plugin, int numParams)
{
	return NavMeshAreaIsConnected(GetNativeCell(1), GetNativeCell(2), GetNativeCell(3));
}

public int Native_NavMeshAreaComputeClosestPointInPortal(Handle plugin, int numParams)
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

public int Native_NavMeshAreaComputeDirection(Handle plugin, int numParams)
{
	float flPos[3];
	GetNativeArray(2, flPos, 3);
	
	return NavMeshAreaComputeDirection(GetNativeCell(1), flPos);
}

public int Native_NavMeshAreaGetLightIntensity(Handle plugin, int numParams)
{
	float flPos[3];
	GetNativeArray(2, flPos, 3);
	
	return view_as<int>(NavMeshAreaGetLightIntensity(GetNativeCell(1), flPos));
}

public int Native_NavHidingSpotGetID(Handle plugin, int numParams)
{
	return g_hNavMeshAreaHidingSpots.Get(GetNativeCell(1), NavMeshHidingSpot_ID);
}

public int Native_NavHidingSpotGetFlags(Handle plugin, int numParams)
{
	return g_hNavMeshAreaHidingSpots.Get(GetNativeCell(1), NavMeshHidingSpot_Flags);
}

public int Native_NavHidingSpotGetPosition(Handle plugin, int numParams)
{
	float buffer[3];
	GetNativeArray(2, buffer, 3);
	
	int hidingSpotIndex = GetNativeCell(1);
	
	buffer[0] = view_as<float>(g_hNavMeshAreaHidingSpots.Get(hidingSpotIndex, NavMeshHidingSpot_X));
	buffer[1] = view_as<float>(g_hNavMeshAreaHidingSpots.Get(hidingSpotIndex, NavMeshHidingSpot_Y));
	buffer[2] = view_as<float>(g_hNavMeshAreaHidingSpots.Get(hidingSpotIndex, NavMeshHidingSpot_Z));
	
	SetNativeArray(2, buffer, 3);
}

public int Native_NavHidingSpotGetArea(Handle plugin, int numParams)
{
	return g_hNavMeshAreaHidingSpots.Get(GetNativeCell(1), NavMeshHidingSpot_AreaIndex);
}

public int Native_NavMeshLadderGetWidth(Handle plugin, int numParams)
{
	return view_as<int>(NavMeshLadderGetWidth(GetNativeCell(1)));
}

public int Native_NavMeshLadderGetLength(Handle plugin, int numParams)
{
	return view_as<int>(NavMeshLadderGetLength(GetNativeCell(1)));
}

public int Native_NavMeshLadderGetTopForwardArea(Handle plugin, int numParams)
{
	return g_hNavMeshLadders.Get(GetNativeCell(1), NavMeshLadder_TopForwardAreaIndex);
}

public int Native_NavMeshLadderGetTopLeftArea(Handle plugin, int numParams)
{
	return g_hNavMeshLadders.Get(GetNativeCell(1), NavMeshLadder_TopLeftAreaIndex);
}

public int Native_NavMeshLadderGetTopRightArea(Handle plugin, int numParams)
{
	return g_hNavMeshLadders.Get(GetNativeCell(1), NavMeshLadder_TopRightAreaIndex);
}

public int Native_NavMeshLadderGetTopBehindArea(Handle plugin, int numParams)
{
	return g_hNavMeshLadders.Get(GetNativeCell(1), NavMeshLadder_TopBehindAreaIndex);
}

public int Native_NavMeshLadderGetBottomArea(Handle plugin, int numParams)
{
	return g_hNavMeshLadders.Get(GetNativeCell(1), NavMeshLadder_BottomAreaIndex);
}

public int Native_NavMeshLadderGetTop(Handle plugin, int numParams)
{
	float buffer[3];
	GetNativeArray(2, buffer, 3);
	
	buffer[0] = view_as<float>(g_hNavMeshLadders.Get(GetNativeCell(1), NavMeshLadder_TopX));
	buffer[1] = view_as<float>(g_hNavMeshLadders.Get(GetNativeCell(1), NavMeshLadder_TopY));
	buffer[2] = view_as<float>(g_hNavMeshLadders.Get(GetNativeCell(1), NavMeshLadder_TopZ));
	
	SetNativeArray(2, buffer, 3);
}

public int Native_NavMeshLadderGetBottom(Handle plugin, int numParams)
{
	float buffer[3];
	GetNativeArray(2, buffer, 3);
	
	buffer[0] = view_as<float>(g_hNavMeshLadders.Get(GetNativeCell(1), NavMeshLadder_BottomX));
	buffer[1] = view_as<float>(g_hNavMeshLadders.Get(GetNativeCell(1), NavMeshLadder_BottomY));
	buffer[2] = view_as<float>(g_hNavMeshLadders.Get(GetNativeCell(1), NavMeshLadder_BottomZ));
	
	SetNativeArray(2, buffer, 3);
}

public int Native_NavSpotEncounterGetFrom(Handle plugin, int numParams)
{
	return NavSpotEncounterGetFrom(GetNativeCell(1));
}

public int Native_NavSpotEncounterGetFromDirection(Handle plugin, int numParams)
{
	return NavSpotEncounterGetFromDirection(GetNativeCell(1));
}

public int Native_NavSpotEncounterGetTo(Handle plugin, int numParams)
{
	return NavSpotEncounterGetTo(GetNativeCell(1));
}

public int Native_NavSpotEncounterGetToDirection(Handle plugin, int numParams)
{
	return NavSpotEncounterGetToDirection(GetNativeCell(1));
}

public int Native_NavSpotEncounterGetSpots(Handle plugin, int numParams)
{
	ArrayStack buffer = view_as<ArrayStack>(GetNativeCell(2));
	ArrayStack dummy = NavSpotEncounterGetSpots(GetNativeCell(1));
	if (dummy != null)
	{
		while (!dummy.Empty)
		{
			int val;
			PopStackCell(dummy, val);
			buffer.Push(val);
		}
		delete dummy;
	}
}

public int Native_NavSpotOrderGetHidingSpot(Handle plugin, int numParams)
{
	return NavSpotOrderGetHidingSpot(GetNativeCell(1));
}

public int Native_NavSpotOrderGetParametricDistance(Handle plugin, int numParams)
{
	return NavSpotOrderGetParametricDistance(GetNativeCell(1));
}

