#include <sourcemod>
#include <sdktools>
#include <navmesh>

#define PLUGIN_VERSION "1.0.0"

public Plugin:myinfo = 
{
    name = "SP-Readable Navigation Mesh",
    author	= "KitRifty (code based from pimpinjuice)",
    description	= "Heavily based off of pimpinjuice's .cpp code for War3Source, this analyzes the map's .nav file and creates the mesh based off the info.",
    version = PLUGIN_VERSION,
    url = ""
}

#define NAV_MAGIC_NUMBER 0xFEEDFACE

#define UNSIGNED_INT_BYTE_SIZE 4
#define UNSIGNED_CHAR_BYTE_SIZE 1
#define UNSIGNED_SHORT_BYTE_SIZE 2
#define FLOAT_BYTE_SIZE 4


new Handle:g_hNavMesh;
new bool:g_bNavMeshBuilt = false;


public APLRes:AskPluginLoad2(Handle:myself, bool:late, String:error[], err_max)
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
	
	CreateNative("NavMesh_BuildPath", Native_NavMeshBuildPath);
	
	CreateNative("NavMeshArea_GetFlags", Native_NavMeshAreaGetFlags);
	CreateNative("NavMeshArea_GetCenter", Native_NavMeshAreaGetCenter);
	CreateNative("NavMeshArea_GetAdjacentList", Native_NavMeshAreaGetAdjacentList);
	CreateNative("NavMeshArea_GetLadderList", Native_NavMeshAreaGetLadderList);
	CreateNative("NavMeshArea_GetTotalCost", Native_NavMeshAreaGetTotalCost);
	CreateNative("NavMeshArea_GetParent", Native_NavMeshAreaGetParent);
	CreateNative("NavMeshArea_GetParentHow", Native_NavMeshAreaGetParentHow);
	CreateNative("NavMeshArea_SetParent", Native_NavMeshAreaSetParent);
	CreateNative("NavMeshArea_SetParentHow", Native_NavMeshAreaSetParentHow);
	CreateNative("NavMeshArea_GetCostSoFar", Native_NavMeshAreaGetCostSoFar);
	
	CreateNative("NavMeshLadder_GetLength", Native_NavMeshLadderGetLength);
}

public OnPluginStart()
{
	g_hNavMesh = CreateArray(NavMesh_MaxStats);
	
	HookEvent("nav_blocked", Event_NavAreaBlocked);
}

public OnMapStart()
{
	NavMeshDestroy();

	decl String:sMap[256];
	GetCurrentMap(sMap, sizeof(sMap));
	
	g_bNavMeshBuilt = NavMeshLoad(sMap);
}

public Event_NavAreaBlocked(Handle:event, const String:name[], bool:dB)
{
	if (!g_bNavMeshBuilt) return;

	new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
	if (hAreas != INVALID_HANDLE)
	{
		new iAreaID = GetEventInt(event, "area");
		new iAreaIndex = FindValueInArray(hAreas, iAreaID);
		if (iAreaIndex != -1)
		{
			new bool:bBlocked = bool:GetEventInt(event, "blocked");
			SetArrayCell(hAreas, iAreaIndex, bBlocked, NavMeshArea_Blocked);
		}
	}
}

bool:NavMeshBuildPath(iStartAreaID,
	iGoalAreaID,
	const Float:flGoalPos[3],
	Function:iCostFunction,
	iClosestAreaID=-1)
{
	if (!g_bNavMeshBuilt) 
	{
		LogError("Could not build path from area %d to area %d because the nav mesh does not exist!", iStartAreaID, iGoalAreaID);
		return false;
	}

	new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
	if (hAreas == INVALID_HANDLE)
	{
		LogError("Could not build path from area %d to area %d because the nav mesh does not have any areas!", iStartAreaID, iGoalAreaID);
		return false;
	}
	
	new iStartAreaIndex = FindValueInArray(hAreas, iStartAreaID);
	if (iStartAreaIndex == -1)
	{
		LogError("Could not build path from area %d to area %d because the starting area does not exist!", iStartAreaID, iGoalAreaID);
		return false;
	}
	
	new iGoalAreaIndex = FindValueInArray(hAreas, iGoalAreaID);
	if (iGoalAreaIndex == -1)
	{
		LogError("Could not build path from area %d to area %d because the goal area does not exist!", iStartAreaID, iGoalAreaID);
		return false;
	}
	
	NavMeshAreaSetParent(iStartAreaID, -1);
	NavMeshAreaSetParentHow(iStartAreaID, NUM_TRAVERSE_TYPES);
	
	if (iStartAreaID == iGoalAreaID)
	{
		NavMeshAreaSetParent(iGoalAreaID, -1);
		NavMeshAreaSetParentHow(iGoalAreaID, NUM_TRAVERSE_TYPES);
		return true;
	}
	
	// Start the search.
	static Handle:hOpenList = INVALID_HANDLE;
	static Handle:hClosedList = INVALID_HANDLE;
	
	if (hOpenList == INVALID_HANDLE) hOpenList = CreateArray();
	if (hClosedList == INVALID_HANDLE) hClosedList = CreateArray();
	
	// Start the search!
	ClearArray(hOpenList);
	ClearArray(hClosedList);
	
	decl Float:flStartAreaCenter[3];
	NavMeshAreaGetCenter(iStartAreaID, flStartAreaCenter);
	
	SetArrayCell(hAreas, iStartAreaIndex, GetVectorDistance(flStartAreaCenter, flGoalPos), NavMeshArea_TotalCost);
	
	new Float:flInitCost;
	
	Call_StartFunction(INVALID_HANDLE, iCostFunction);
	Call_PushCell(iStartAreaID);
	Call_PushCell(-1);
	Call_PushCell(-1);
	Call_Finish(flInitCost);
	
	if (flInitCost < 0.0) return false;
	
	PushArrayCell(hOpenList, iStartAreaID);
	
	if (iClosestAreaID != -1) iClosestAreaID = iStartAreaID;
	
	new Float:flClosestAreaDist = Float:GetArrayCell(hAreas, iStartAreaIndex, NavMeshArea_TotalCost);
	
	new Handle:hLadders = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Ladders);
	
	// Perform A* search.
	while (GetArraySize(hOpenList))
	{
		new iAreaID = GetArrayCell(hOpenList, 0);
		RemoveFromArray(hOpenList, 0);
		
		new iAreaIndex = FindValueInArray(hAreas, iAreaID);
		if (bool:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_Blocked)) continue;
		
		if (iAreaID == iGoalAreaID)
		{
			if (iClosestAreaID != -1)
			{
				iClosestAreaID = iGoalAreaID;
			}
			
			return true;
		}
		
		new bool:bSearchFloor = true;
		new iFloorDir = NAV_DIR_NORTH;
		new Handle:hFloorList = NavMeshAreaGetAdjacentList(iAreaID, NAV_DIR_NORTH);
		new iFloorIter = 0;
		
		new bool:bLadderUp = true;
		new Handle:hLadderList = INVALID_HANDLE;
		new iLadderIter = 0;
		new iLadderTopDir = 0;
		
		for (;;)
		{
			new iNewAreaID = -1;
			new iNavTraverseHow = 0;
			new iLadderID = -1;
			
			if (bSearchFloor)
			{
				if (hFloorList == INVALID_HANDLE || iFloorIter >= GetArraySize(hFloorList))
				{
					iFloorDir++;
					
					if (iFloorDir == NAV_DIR_COUNT)
					{
						bSearchFloor = false;
						if (hFloorList != INVALID_HANDLE) CloseHandle(hFloorList);
						
						hLadderList = NavMeshAreaGetLadderList(iAreaID, NAV_LADDER_DIR_UP);
						iLadderIter = 0;
						iLadderTopDir = 0;
					}
					else
					{
						if (hFloorList != INVALID_HANDLE) CloseHandle(hFloorList);
						hFloorList = NavMeshAreaGetAdjacentList(iAreaID, iFloorDir);
						iFloorIter = 0;
					}
					
					continue;
				}
				
				iNewAreaID = GetArrayCell(hFloorList, iFloorIter);
				iNavTraverseHow = iFloorDir;
				iFloorIter++;
			}
			else
			{
				if (hLadderList == INVALID_HANDLE || iLadderIter >= GetArraySize(hLadderList))
				{
					if (!bLadderUp)
					{
						if (hLadderList != INVALID_HANDLE) CloseHandle(hLadderList);
						
						break;
					}
					else
					{
						bLadderUp = false;
						if (hLadderList != INVALID_HANDLE) CloseHandle(hLadderList);
						hLadderList = NavMeshAreaGetLadderList(iAreaID, NAV_LADDER_DIR_DOWN);
						iLadderIter = 0;
					}
					
					continue;
				}
				
				iLadderID = GetArrayCell(hLadderList, iLadderIter);
				new iLadderIndex = FindValueInArray(hLadders, iLadderID);
				
				if (bLadderUp)
				{
					switch (iLadderTopDir)
					{
						case 0:
						{
							iNewAreaID = GetArrayCell(hLadders, iLadderIndex, NavMeshLadder_TopForwardAreaID);
						}
						case 1:
						{
							iNewAreaID = GetArrayCell(hLadders, iLadderIndex, NavMeshLadder_TopLeftAreaID);
						}
						case 2:
						{
							iNewAreaID = GetArrayCell(hLadders, iLadderIndex, NavMeshLadder_TopRightAreaID);
						}
						default:
						{
							iLadderIter++;
							iLadderTopDir = 0;
							continue;
						}
					}
					
					iNavTraverseHow = GO_LADDER_UP;
					iLadderTopDir++;
				}
				else
				{
					iNewAreaID = GetArrayCell(hLadders, iLadderIndex, NavMeshLadder_BottomAreaID);
					iNavTraverseHow = GO_LADDER_DOWN;
					iLadderIter++;
				}
				
				if (iNewAreaID == -1) continue;
			}
			
			if (iNewAreaID == iAreaID) continue;
			
			new iNewAreaIndex = FindValueInArray(hAreas, iNewAreaID);
			
			if (bool:GetArrayCell(hAreas, iNewAreaIndex, NavMeshArea_Blocked)) continue;
			
			new Float:flNewCostSoFar;
			
			Call_StartFunction(INVALID_HANDLE, iCostFunction);
			Call_PushCell(iNewAreaID);
			Call_PushCell(iAreaID);
			Call_PushCell(iLadderID);
			Call_Finish(flNewCostSoFar);
			
			if (flNewCostSoFar < 0.0) continue;
			
			if (Float:GetArrayCell(hAreas, iNewAreaIndex, NavMeshArea_CostSoFar) <= flNewCostSoFar)
			{
				continue;
			}
			else
			{
				decl Float:flNewAreaCenter[3];
				NavMeshAreaGetCenter(iNewAreaID, flNewAreaCenter);
				
				new Float:flNewCostRemaining = GetVectorDistance(flNewAreaCenter, flGoalPos);
				
				if (iClosestAreaID != -1 && flNewCostRemaining < flClosestAreaDist)
				{
					iClosestAreaID = iNewAreaID;
					flClosestAreaDist = flNewCostRemaining;
				}
				
				NavMeshAreaSetParent(iNewAreaID, iAreaID);
				NavMeshAreaSetParentHow(iNewAreaID, iNavTraverseHow);
				
				SetArrayCell(hAreas, iNewAreaIndex, flNewCostSoFar + flNewCostRemaining, NavMeshArea_TotalCost);
				
				new iClosedListIndex = FindValueInArray(hClosedList, iNewAreaID);
				if (iClosedListIndex != -1) 
				{
					RemoveFromArray(hClosedList, iClosedListIndex);
					
					new iOpenListIndex = FindValueInArray(hOpenList, iNewAreaID);
					if (iOpenListIndex == -1)
					{
						PushArrayCell(hOpenList, iNewAreaID);
					}
					
					SortADTArrayCustom(hOpenList, SortNavMeshOpenList);
				}
			}
		}
		
		new iClosedListIndex = FindValueInArray(hClosedList, iAreaID);
		if (iClosedListIndex == -1) PushArrayCell(hClosedList, iAreaID);
	}
	
	return false;
}

public SortNavMeshOpenList(index1, index2, Handle:hOpenList, Handle:hndl)
{
	new iArea1ID = GetArrayCell(hOpenList, index1);
	new Float:iArea1TotalCost = NavMeshAreaGetTotalCost(iArea1ID);
	
	new iArea2ID = GetArrayCell(hOpenList, index2);
	new Float:iArea2TotalCost = NavMeshAreaGetTotalCost(iArea2ID);
	
	if (iArea1TotalCost < iArea2TotalCost)
	{
		return -1;
	}
	else if (iArea1TotalCost == iArea2TotalCost)
	{	
		return 0;
	}
	else
	{
		return 1;
	}
}

bool:NavMeshLoad(const String:sMapName[])
{
	decl String:sNavFilePath[PLATFORM_MAX_PATH];
	Format(sNavFilePath, sizeof(sNavFilePath), "maps\\%s.nav", sMapName);
	
	new Handle:hFile = OpenFile(sNavFilePath, "rb");
	if (hFile == INVALID_HANDLE)
	{
		LogError("Unable to find navigation mesh: %s", sNavFilePath);
		return false;
	}
	
	// Get magic number.
	new iNavMagicNumber;
	new iElementsRead = ReadFileCell(hFile, iNavMagicNumber, UNSIGNED_INT_BYTE_SIZE);
	
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
	new iNavVersion;
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
	new iNavSubVersion;
	if (iNavVersion >= 10)
	{
		ReadFileCell(hFile, iNavSubVersion, UNSIGNED_INT_BYTE_SIZE);
	}
	
	// Get the save bsp size.
	new iNavSaveBspSize;
	if (iNavVersion >= 4)
	{
		ReadFileCell(hFile, iNavSaveBspSize, UNSIGNED_INT_BYTE_SIZE);
	}
	
	// Check if the nav mesh was analyzed.
	new iNavMeshAnalyzed;
	if (iNavVersion >= 14)
	{
		ReadFileCell(hFile, iNavMeshAnalyzed, UNSIGNED_CHAR_BYTE_SIZE);
		LogMessage("Is mesh analyzed: %d", iNavMeshAnalyzed);
	}
	
	LogMessage("Nav version: %d; SubVersion: %d (v10+); BSPSize: %d; MagicNumber: %d", iNavVersion, iNavSubVersion, iNavSaveBspSize, iNavMagicNumber);
	
	new iPlaceCount;
	ReadFileCell(hFile, iPlaceCount, UNSIGNED_SHORT_BYTE_SIZE);
	LogMessage("Place count: %d", iPlaceCount);
	
	// Parse through places.
	// TF2 doesn't use places, but CS:S does.
	new Handle:hPlaces = CreateArray(256);
	
	for (new iPlaceIndex = 0; iPlaceIndex < iPlaceCount; iPlaceIndex++) 
	{
		new iPlaceSize;
		ReadFileCell(hFile, iPlaceSize, UNSIGNED_SHORT_BYTE_SIZE);
		
		new String:sPlaceName[256];
		ReadFileString(hFile, sPlaceName, sizeof(sPlaceName), iPlaceSize);
		
		PushArrayString(hPlaces, sPlaceName);
		
		//LogMessage("Parsed place! %s [%d]", sPlaceName, iPlaceIndex);
	}
	
	if (GetArraySize(hPlaces) <= 0)
	{
		CloseHandle(hPlaces);
		hPlaces = INVALID_HANDLE;
	}
	
	// Get any unnamed areas.
	new iNavUnnamedAreas;
	if (iNavVersion > 11)
	{
		ReadFileCell(hFile, iNavUnnamedAreas, UNSIGNED_CHAR_BYTE_SIZE);
		
		LogMessage("Has unnamed areas: %d", iNavUnnamedAreas);
	}
	
	// Get area count.
	new iAreaCount;
	ReadFileCell(hFile, iAreaCount, UNSIGNED_INT_BYTE_SIZE);
	
	LogMessage("Area count: %d", iAreaCount);
	
	// Parse through areas, if any.
	new Handle:hAreas = CreateArray(NavMeshArea_MaxStats);
	
	if (iAreaCount > 0)
	{
		for (new iAreaIndex = 0; iAreaIndex < iAreaCount; iAreaIndex++)
		{
			new iAreaID;
			new Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2;
			new iAreaFlags;
			new Handle:hConnections = CreateArray(NavMeshConnection_MaxStats);
			new Handle:hHidingSpots = CreateArray(NavMeshHidingSpot_MaxStats);
			new Handle:hEncounterPaths = CreateArray(NavMeshEncounterPath_MaxStats);
			new Handle:hLadderConnections = CreateArray(NavMeshLadderConnection_MaxStats);
			new Handle:hCornerLightIntensities = CreateArray(NavMeshCornerLightIntensity_MaxStats);
			new Handle:hVisibleAreas = CreateArray(NavMeshVisibleArea_MaxStats);
			new iInheritVisibilityFrom;
			new iHidingSpotCount;
			new iVisibleAreaCount;
			new Float:flEarliestOccupyTimeFirstTeam;
			new Float:flEarliestOccupyTimeSecondTeam;
			new Float:flNECornerZ;
			new Float:flSWCornerZ;
			new iPlaceID;
			new unk01;
			
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
			
			ReadFileCell(hFile, _:x1, FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, _:y1, FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, _:z1, FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, _:x2, FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, _:y2, FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, _:z2, FLOAT_BYTE_SIZE);
			
			//LogMessage("Area extent: (%f, %f, %f), (%f, %f, %f)", x1, y1, z1, x2, y2, z2);
			
			ReadFileCell(hFile, _:flNECornerZ, FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, _:flSWCornerZ, FLOAT_BYTE_SIZE);
			
			//LogMessage("Corners: NW(%f), SW(%f)", flNECornerZ, flSWCornerZ);
			
			// Find connections.
			for (new iDirection = 0; iDirection < NAV_DIR_COUNT; iDirection++)
			{
				new iConnectionCount;
				ReadFileCell(hFile, iConnectionCount, UNSIGNED_INT_BYTE_SIZE);
				
				//LogMessage("Connection count: %d", iConnectionCount);
				
				for (new iConnectionIndex = 0; iConnectionIndex < iConnectionCount; iConnectionIndex++) 
				{
					new iConnectingAreaID;
					ReadFileCell(hFile, iConnectingAreaID, UNSIGNED_INT_BYTE_SIZE);
					
					new iIndex = PushArrayCell(hConnections, iConnectingAreaID);
					SetArrayCell(hConnections, iIndex, iDirection, NavMeshConnection_Direction);
				}
			}
			
			if (GetArraySize(hConnections) <= 0)
			{
				CloseHandle(hConnections);
				hConnections = INVALID_HANDLE;
			}
			
			// Get hiding spots.
			ReadFileCell(hFile, iHidingSpotCount, UNSIGNED_CHAR_BYTE_SIZE);
			
			//LogMessage("Hiding spot count: %d", iHidingSpotCount);
			
			if (iHidingSpotCount > 0)
			{
				for (new iHidingSpotIndex = 0; iHidingSpotIndex < iHidingSpotCount; iHidingSpotIndex++)
				{
					new iHidingSpotID;
					ReadFileCell(hFile, iHidingSpotID, UNSIGNED_INT_BYTE_SIZE);
					
					new Float:flHidingSpotX, Float:flHidingSpotY, Float:flHidingSpotZ;
					ReadFileCell(hFile, _:flHidingSpotX, FLOAT_BYTE_SIZE);
					ReadFileCell(hFile, _:flHidingSpotY, FLOAT_BYTE_SIZE);
					ReadFileCell(hFile, _:flHidingSpotZ, FLOAT_BYTE_SIZE);
					
					new iHidingSpotFlags;
					ReadFileCell(hFile, iHidingSpotFlags, UNSIGNED_CHAR_BYTE_SIZE);
					
					new iIndex = PushArrayCell(hHidingSpots, iHidingSpotID);
					SetArrayCell(hHidingSpots, iIndex, flHidingSpotX, NavMeshHidingSpot_X);
					SetArrayCell(hHidingSpots, iIndex, flHidingSpotY, NavMeshHidingSpot_Y);
					SetArrayCell(hHidingSpots, iIndex, flHidingSpotZ, NavMeshHidingSpot_Z);
					SetArrayCell(hHidingSpots, iIndex, iHidingSpotFlags, NavMeshHidingSpot_Flags);
					
					//LogMessage("Parsed hiding spot (%f, %f, %f) with ID [%d] and flags [%d]", flHidingSpotX, flHidingSpotY, flHidingSpotZ, iHidingSpotID, iHidingSpotFlags);
				}
			}
			else
			{
				CloseHandle(hHidingSpots);
				hHidingSpots = INVALID_HANDLE;
			}
			
			// Get approach areas (old version, only used to read data)
			if (iNavVersion < 15)
			{
				new iApproachAreaCount;
				ReadFileCell(hFile, iApproachAreaCount, UNSIGNED_CHAR_BYTE_SIZE);
				
				for (new iApproachAreaIndex = 0; iApproachAreaIndex < iApproachAreaCount; iApproachAreaIndex++)
				{
					new iApproachHereID;
					ReadFileCell(hFile, iApproachHereID, UNSIGNED_INT_BYTE_SIZE);
					
					new iApproachPrevID;
					ReadFileCell(hFile, iApproachPrevID, UNSIGNED_INT_BYTE_SIZE);
					
					new iApproachType;
					ReadFileCell(hFile, iApproachType, UNSIGNED_CHAR_BYTE_SIZE);
					
					new iApproachNextID;
					ReadFileCell(hFile, iApproachNextID, UNSIGNED_INT_BYTE_SIZE);
					
					new iApproachHow;
					ReadFileCell(hFile, iApproachHow, UNSIGNED_CHAR_BYTE_SIZE);
				}
			}
			
			// Get encounter paths.
			new iEncounterPathCount;
			ReadFileCell(hFile, iEncounterPathCount, UNSIGNED_INT_BYTE_SIZE);
			
			//LogMessage("Encounter Path Count: %d", iEncounterPathCount);
			
			if (iEncounterPathCount > 0)
			{
				for (new iEncounterPathIndex = 0; iEncounterPathIndex < iEncounterPathCount; iEncounterPathIndex++)
				{
					new iEncounterFromID;
					ReadFileCell(hFile, iEncounterFromID, UNSIGNED_INT_BYTE_SIZE);
					
					new iEncounterFromDirection;
					ReadFileCell(hFile, iEncounterFromDirection, UNSIGNED_CHAR_BYTE_SIZE);
					
					new iEncounterToID;
					ReadFileCell(hFile, iEncounterToID, UNSIGNED_INT_BYTE_SIZE);
					
					new iEncounterToDirection;
					ReadFileCell(hFile, iEncounterToDirection, UNSIGNED_CHAR_BYTE_SIZE);
					
					new iEncounterSpotCount;
					ReadFileCell(hFile, iEncounterSpotCount, UNSIGNED_CHAR_BYTE_SIZE);
					
					//LogMessage("Encounter [from ID %d] [from dir %d] [to ID %d] [to dir %d] [spot count %d]", iEncounterFromID, iEncounterFromDirection, iEncounterToID, iEncounterToDirection, iEncounterSpotCount);
					
					new Handle:hEncounterSpots = CreateArray(NavMeshEncounterSpot_MaxStats);
					
					if (iEncounterSpotCount > 0)
					{
						for (new iEncounterSpotIndex = 0; iEncounterSpotIndex < iEncounterSpotCount; iEncounterSpotIndex++)
						{
							new iEncounterSpotOrderID;
							ReadFileCell(hFile, iEncounterSpotOrderID, UNSIGNED_INT_BYTE_SIZE);
							
							new iEncounterSpotT;
							ReadFileCell(hFile, iEncounterSpotT, UNSIGNED_CHAR_BYTE_SIZE);
							
							new Float:flEncounterSpotParametricDistance = float(iEncounterSpotT) / 255.0;
							
							new iIndex = PushArrayCell(hEncounterSpots, iEncounterSpotOrderID);
							SetArrayCell(hEncounterSpots, iIndex, flEncounterSpotParametricDistance, NavMeshEncounterSpot_ParametricDistance);
							
							//LogMessage("Encounter spot [order id %d] and [T %d]", iEncounterSpotOrderID, iEncounterSpotT);
						}
					}
					else
					{
						CloseHandle(hEncounterSpots);
						hEncounterSpots = INVALID_HANDLE;
					}
					
					new iIndex = PushArrayCell(hEncounterPaths, iEncounterFromID);
					SetArrayCell(hEncounterPaths, iIndex, iEncounterFromDirection, NavMeshEncounterPath_FromDirection);
					SetArrayCell(hEncounterPaths, iIndex, iEncounterToID, NavMeshEncounterPath_ToID);
					SetArrayCell(hEncounterPaths, iIndex, iEncounterToDirection, NavMeshEncounterPath_ToDirection);
					SetArrayCell(hEncounterPaths, iIndex, hEncounterSpots, NavMeshEncounterPath_Spots);
				}
			}
			else
			{
				CloseHandle(hEncounterPaths);
				hEncounterPaths = INVALID_HANDLE;
			}
			
			ReadFileCell(hFile, iPlaceID, UNSIGNED_SHORT_BYTE_SIZE);
			
			//LogMessage("Place ID: %d", iPlaceID);
			
			// Get ladder connections.
			for (new iLadderDirection = 0; iLadderDirection < NAV_LADDER_DIR_COUNT; iLadderDirection++)
			{
				new iLadderConnectionCount;
				ReadFileCell(hFile, iLadderConnectionCount, UNSIGNED_INT_BYTE_SIZE);
				
				//LogMessage("Ladder Connection Count: %d", iLadderConnectionCount);
				
				for (new iLadderConnectionIndex = 0; iLadderConnectionIndex < iLadderConnectionCount; iLadderConnectionIndex++)
				{
					new iLadderConnectionID;
					ReadFileCell(hFile, iLadderConnectionID, UNSIGNED_INT_BYTE_SIZE);
					
					new iIndex = PushArrayCell(hLadderConnections, iLadderConnectionID);
					SetArrayCell(hLadderConnections, iIndex, iLadderDirection, NavMeshLadderConnection_Direction);
					
					//LogMessage("Parsed ladder connect [ID %d]\n", iLadderConnectionID);
				}
			}
			
			if (GetArraySize(hLadderConnections) <= 0)
			{
				CloseHandle(hLadderConnections);
				hLadderConnections = INVALID_HANDLE;
			}
			
			ReadFileCell(hFile, _:flEarliestOccupyTimeFirstTeam, FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, _:flEarliestOccupyTimeSecondTeam, FLOAT_BYTE_SIZE);
			
			if (iNavVersion >= 11)
			{
				for (new iNavCornerIndex = 0; iNavCornerIndex < NAV_CORNER_COUNT; iNavCornerIndex++)
				{
					new Float:flNavCornerLightIntensity;
					ReadFileCell(hFile, _:flNavCornerLightIntensity, FLOAT_BYTE_SIZE);
					
					new iIndex = PushArrayCell(hCornerLightIntensities, iNavCornerIndex);
					SetArrayCell(hCornerLightIntensities, iIndex, flNavCornerLightIntensity, NavMeshCornerLightIntensity_Intensity);
					
					//LogMessage("Light intensity: [%f] [idx %d]", flNavCornerLightIntensity, iNavCornerIndex);
				}
				
				if (iNavVersion >= 16)
				{
					ReadFileCell(hFile, iVisibleAreaCount, UNSIGNED_INT_BYTE_SIZE);
					
					//LogMessage("Visible area count: %d", iVisibleAreaCount);
					
					if (iVisibleAreaCount > 0)
					{
						for (new iVisibleAreaIndex = 0; iVisibleAreaIndex < iVisibleAreaCount; iVisibleAreaIndex++)
						{
							new iVisibleAreaID;
							ReadFileCell(hFile, iVisibleAreaID, UNSIGNED_INT_BYTE_SIZE);
							
							new iVisibleAreaAttributes;
							ReadFileCell(hFile, iVisibleAreaAttributes, UNSIGNED_CHAR_BYTE_SIZE);
							
							new iIndex = PushArrayCell(hVisibleAreas, iVisibleAreaID);
							SetArrayCell(hVisibleAreas, iIndex, iVisibleAreaAttributes, NavMeshVisibleArea_Attributes);
							
							//LogMessage("Parsed visible area [%d] with attr [%d]", iVisibleAreaID, iVisibleAreaAttributes);
						}
					}
					else
					{
						CloseHandle(hVisibleAreas);
						hVisibleAreas = INVALID_HANDLE;
					}
					
					ReadFileCell(hFile, iInheritVisibilityFrom, UNSIGNED_INT_BYTE_SIZE);
					
					//LogMessage("Inherit visibilty from: %d", iInheritVisibilityFrom);
					
					ReadFileCell(hFile, unk01, UNSIGNED_INT_BYTE_SIZE);
				}
			}
			
			new iIndex = PushArrayCell(hAreas, iAreaID);
			SetArrayCell(hAreas, iIndex, iAreaFlags, NavMeshArea_Flags);
			SetArrayCell(hAreas, iIndex, iPlaceID, NavMeshArea_PlaceID);
			SetArrayCell(hAreas, iIndex, x1, NavMeshArea_X1);
			SetArrayCell(hAreas, iIndex, y1, NavMeshArea_Y1);
			SetArrayCell(hAreas, iIndex, z1, NavMeshArea_Z1);
			SetArrayCell(hAreas, iIndex, x2, NavMeshArea_X2);
			SetArrayCell(hAreas, iIndex, y2, NavMeshArea_Y2);
			SetArrayCell(hAreas, iIndex, z2, NavMeshArea_Z2);
			SetArrayCell(hAreas, iIndex, flNECornerZ, NavMeshArea_NECornerZ);
			SetArrayCell(hAreas, iIndex, flSWCornerZ, NavMeshArea_SWCornerZ);
			SetArrayCell(hAreas, iIndex, hConnections, NavMeshArea_Connections);
			SetArrayCell(hAreas, iIndex, hHidingSpots, NavMeshArea_HidingSpots);
			SetArrayCell(hAreas, iIndex, hEncounterPaths, NavMeshArea_EncounterPaths);
			SetArrayCell(hAreas, iIndex, hLadderConnections, NavMeshArea_LadderConnections);
			SetArrayCell(hAreas, iIndex, hCornerLightIntensities, NavMeshArea_CornerLightIntensities);
			SetArrayCell(hAreas, iIndex, hVisibleAreas, NavMeshArea_VisibleAreas);
			SetArrayCell(hAreas, iIndex, iInheritVisibilityFrom, NavMeshArea_InheritVisibilityFrom);
			SetArrayCell(hAreas, iIndex, flEarliestOccupyTimeFirstTeam, NavMeshArea_EarliestOccupyTimeFirstTeam);
			SetArrayCell(hAreas, iIndex, flEarliestOccupyTimeSecondTeam, NavMeshArea_EarliestOccupyTimeSecondTeam);
			SetArrayCell(hAreas, iIndex, unk01, NavMeshArea_unk01);
			SetArrayCell(hAreas, iIndex, -1, NavMeshArea_Parent);
			SetArrayCell(hAreas, iIndex, NUM_TRAVERSE_TYPES, NavMeshArea_ParentHow);
			SetArrayCell(hAreas, iIndex, 0.0, NavMeshArea_TotalCost);
			SetArrayCell(hAreas, iIndex, 0.0, NavMeshArea_CostSoFar);
			SetArrayCell(hAreas, iIndex, false, NavMeshArea_Blocked);
		}
	}
	else
	{
		CloseHandle(hAreas);
		hAreas = INVALID_HANDLE;
	}
	
	new iLadderCount;
	ReadFileCell(hFile, iLadderCount, UNSIGNED_INT_BYTE_SIZE);
	
	new Handle:hLadders = CreateArray(NavMeshLadder_MaxStats);
	
	if (iLadderCount > 0)
	{
		for (new iLadderIndex; iLadderIndex < iLadderCount; iLadderIndex++)
		{
			new iLadderID;
			ReadFileCell(hFile, iLadderID, UNSIGNED_INT_BYTE_SIZE);
			
			new Float:flLadderWidth;
			ReadFileCell(hFile, _:flLadderWidth, FLOAT_BYTE_SIZE);
			
			new Float:flLadderTopX, Float:flLadderTopY, Float:flLadderTopZ, Float:flLadderBottomX, Float:flLadderBottomY, Float:flLadderBottomZ;
			ReadFileCell(hFile, _:flLadderTopX, FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, _:flLadderTopY, FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, _:flLadderTopZ, FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, _:flLadderBottomX, FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, _:flLadderBottomY, FLOAT_BYTE_SIZE);
			ReadFileCell(hFile, _:flLadderBottomZ, FLOAT_BYTE_SIZE);
			
			new Float:flLadderLength;
			ReadFileCell(hFile, _:flLadderLength, FLOAT_BYTE_SIZE);
			
			new iLadderDirection;
			ReadFileCell(hFile, iLadderDirection, UNSIGNED_INT_BYTE_SIZE);
			
			new iLadderTopForwardAreaID;
			ReadFileCell(hFile, iLadderTopForwardAreaID, UNSIGNED_INT_BYTE_SIZE);
			
			new iLadderTopLeftAreaID;
			ReadFileCell(hFile, iLadderTopLeftAreaID, UNSIGNED_INT_BYTE_SIZE);
			
			new iLadderTopRightAreaID;
			ReadFileCell(hFile, iLadderTopRightAreaID, UNSIGNED_INT_BYTE_SIZE);
			
			new iLadderTopBehindAreaID;
			ReadFileCell(hFile, iLadderTopBehindAreaID, UNSIGNED_INT_BYTE_SIZE);
			
			new iLadderBottomAreaID;
			ReadFileCell(hFile, iLadderBottomAreaID, UNSIGNED_INT_BYTE_SIZE);
			
			new iIndex = PushArrayCell(hLadders, iLadderID);
			SetArrayCell(hLadders, iIndex, flLadderWidth, NavMeshLadder_Width);
			SetArrayCell(hLadders, iIndex, flLadderLength, NavMeshLadder_Length);
			SetArrayCell(hLadders, iIndex, flLadderTopX, NavMeshLadder_TopX);
			SetArrayCell(hLadders, iIndex, flLadderTopY, NavMeshLadder_TopY);
			SetArrayCell(hLadders, iIndex, flLadderTopZ, NavMeshLadder_TopZ);
			SetArrayCell(hLadders, iIndex, flLadderBottomX, NavMeshLadder_BottomX);
			SetArrayCell(hLadders, iIndex, flLadderBottomY, NavMeshLadder_BottomY);
			SetArrayCell(hLadders, iIndex, flLadderBottomZ, NavMeshLadder_BottomZ);
			SetArrayCell(hLadders, iIndex, iLadderDirection, NavMeshLadder_Direction);
			SetArrayCell(hLadders, iIndex, iLadderTopForwardAreaID, NavMeshLadder_TopForwardAreaID);
			SetArrayCell(hLadders, iIndex, iLadderTopLeftAreaID, NavMeshLadder_TopLeftAreaID);
			SetArrayCell(hLadders, iIndex, iLadderTopRightAreaID, NavMeshLadder_TopRightAreaID);
			SetArrayCell(hLadders, iIndex, iLadderTopBehindAreaID, NavMeshLadder_TopBehindAreaID);
			SetArrayCell(hLadders, iIndex, iLadderBottomAreaID, NavMeshLadder_BottomAreaID);
		}
	}
	else
	{
		CloseHandle(hLadders);
		hLadders = INVALID_HANDLE;
	}
	
	new iIndex = PushArrayCell(g_hNavMesh, iNavMagicNumber);
	SetArrayCell(g_hNavMesh, iIndex, iNavVersion, NavMesh_Version);
	SetArrayCell(g_hNavMesh, iIndex, iNavSubVersion, NavMesh_SubVersion);
	SetArrayCell(g_hNavMesh, iIndex, iNavSaveBspSize, NavMesh_SaveBSPSize);
	SetArrayCell(g_hNavMesh, iIndex, iNavMeshAnalyzed, NavMesh_IsMeshAnalyzed);
	SetArrayCell(g_hNavMesh, iIndex, hPlaces, NavMesh_Places);
	SetArrayCell(g_hNavMesh, iIndex, hAreas, NavMesh_Areas);
	SetArrayCell(g_hNavMesh, iIndex, hLadders, NavMesh_Ladders);
	
	CloseHandle(hFile);
	
	return true;
}

NavMeshDestroy()
{
	for (new i = 0, iSize = GetArraySize(g_hNavMesh); i < iSize; i++)
	{
		new Handle:hPlaces = Handle:GetArrayCell(g_hNavMesh, i, NavMesh_Places);
		if (hPlaces != INVALID_HANDLE) CloseHandle(hPlaces);
		
		new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, i, NavMesh_Areas);
		if (hAreas != INVALID_HANDLE)
		{
			for (new iAreaIndex = 0, iAreaCount = GetArraySize(hAreas); iAreaIndex < iAreaCount; iAreaIndex++)
			{
				new Handle:hAreaConnections = Handle:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_Connections);
				if (hAreaConnections != INVALID_HANDLE) CloseHandle(hAreaConnections);
				
				new Handle:hAreaHidingSpots = Handle:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_HidingSpots);
				if (hAreaHidingSpots != INVALID_HANDLE) CloseHandle(hAreaHidingSpots);
				
				new Handle:hAreaEncounterPaths = Handle:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_EncounterPaths);
				if (hAreaEncounterPaths != INVALID_HANDLE)
				{
					for (new iAreaEncounterPathIndex = 0, iAreaEncounterPathCount = GetArraySize(hAreaEncounterPaths); iAreaEncounterPathIndex < iAreaEncounterPathCount; iAreaEncounterPathIndex++)
					{
						new Handle:hAreaEncounterSpots = Handle:GetArrayCell(hAreaEncounterPaths, iAreaEncounterPathIndex, NavMeshEncounterPath_Spots);
						if (hAreaEncounterSpots != INVALID_HANDLE) CloseHandle(hAreaEncounterSpots);
					}
					
					CloseHandle(hAreaEncounterPaths);
				}
				
				new Handle:hAreaLadderConnections = Handle:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_LadderConnections);
				if (hAreaLadderConnections != INVALID_HANDLE) CloseHandle(hAreaLadderConnections);
				
				new Handle:hAreaCornerLightIntensities = Handle:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_CornerLightIntensities);
				if (hAreaCornerLightIntensities != INVALID_HANDLE) CloseHandle(hAreaCornerLightIntensities);
				
				new Handle:hAreaVisibleAreas = Handle:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_VisibleAreas);
				if (hAreaVisibleAreas != INVALID_HANDLE) CloseHandle(hAreaVisibleAreas);
			}
			
			CloseHandle(hAreas);
		}
	}
	
	ClearArray(g_hNavMesh);
	
	g_bNavMeshBuilt = false;
}

stock NavMeshAreaGetFlags(iAreaID)
{
	if (!g_bNavMeshBuilt) return 0;

	new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
	if (hAreas == INVALID_HANDLE) return 0;
	
	new iAreaIndex = FindValueInArray(hAreas, iAreaID);
	if (iAreaIndex == -1) return 0;
	
	return GetArrayCell(hAreas, iAreaIndex, NavMeshArea_Flags);
}

stock bool:NavMeshAreaGetCenter(iAreaID, Float:flBuffer[3])
{
	if (!g_bNavMeshBuilt) return false;

	new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
	if (hAreas == INVALID_HANDLE) return false;
	
	new iAreaIndex = FindValueInArray(hAreas, iAreaID);
	if (iAreaIndex == -1) return false;
	
	decl Float:flExtentLow[3], Float:flExtentHigh[3];
	flExtentLow[0] = Float:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_X1);
	flExtentLow[1] = Float:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_Y1);
	flExtentLow[2] = Float:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_Z1);
	flExtentHigh[0] = Float:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_X2);
	flExtentHigh[1] = Float:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_Y2);
	flExtentHigh[2] = Float:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_Z2);
	
	for (new i = 0; i < 3; i++) flBuffer[i] = (flExtentLow[i] + flExtentHigh[i]) / 2.0;
	
	return true;
}

stock Handle:NavMeshAreaGetAdjacentList(iAreaID, iNavDirection)
{
	if (!g_bNavMeshBuilt) return INVALID_HANDLE;

	new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
	if (hAreas == INVALID_HANDLE) return INVALID_HANDLE;
	
	new iAreaIndex = FindValueInArray(hAreas, iAreaID);
	if (iAreaIndex == -1) return INVALID_HANDLE;
	
	new Handle:hConnections = Handle:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_Connections);
	if (hConnections == INVALID_HANDLE) return INVALID_HANDLE;
	
	new Handle:hAdjacentList = CreateArray();
	
	for (new i = 0, iSize = GetArraySize(hConnections); i < iSize; i++)
	{
		if (GetArrayCell(hConnections, i, NavMeshConnection_Direction) == iNavDirection)
		{
			PushArrayCell(hAdjacentList, GetArrayCell(hConnections, i, NavMeshConnection_AreaID));
		}
	}
	
	return hAdjacentList;
}

stock Handle:NavMeshAreaGetLadderList(iAreaID, iLadderDir)
{
	if (!g_bNavMeshBuilt) return INVALID_HANDLE;

	new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
	if (hAreas == INVALID_HANDLE) return INVALID_HANDLE;
	
	new iAreaIndex = FindValueInArray(hAreas, iAreaID);
	if (iAreaIndex == -1) return INVALID_HANDLE;
	
	new Handle:hConnections = Handle:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_LadderConnections);
	if (hConnections == INVALID_HANDLE) return INVALID_HANDLE;
	
	new Handle:hLadderList = CreateArray();
	
	for (new i = 0, iSize = GetArraySize(hConnections); i < iSize; i++)
	{
		if (GetArrayCell(hConnections, i, NavMeshLadderConnection_Direction) == iLadderDir)
		{
			PushArrayCell(hLadderList, GetArrayCell(hConnections, i, NavMeshLadderConnection_ID));
		}
	}
	
	return hLadderList;
}

stock Float:NavMeshAreaGetTotalCost(iAreaID)
{
	if (!g_bNavMeshBuilt) return 0.0;

	new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
	if (hAreas == INVALID_HANDLE) return 0.0;
	
	new iAreaIndex = FindValueInArray(hAreas, iAreaID);
	if (iAreaIndex == -1) return 0.0;
	
	return Float:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_TotalCost);
}

stock Float:NavMeshAreaGetCostSoFar(iAreaID)
{
	if (!g_bNavMeshBuilt) return 0.0;

	new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
	if (hAreas == INVALID_HANDLE) return 0.0;
	
	new iAreaIndex = FindValueInArray(hAreas, iAreaID);
	if (iAreaIndex == -1) return 0.0;
	
	return Float:GetArrayCell(hAreas, iAreaIndex, NavMeshArea_CostSoFar);
}

stock NavMeshAreaGetParent(iAreaID)
{
	if (!g_bNavMeshBuilt) return -1;

	new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
	if (hAreas == INVALID_HANDLE) return -1;
	
	new iAreaIndex = FindValueInArray(hAreas, iAreaID);
	if (iAreaIndex == -1) return -1;
	
	return GetArrayCell(hAreas, iAreaIndex, NavMeshArea_Parent);
}

stock NavMeshAreaGetParentHow(iAreaID)
{
	if (!g_bNavMeshBuilt) return NUM_TRAVERSE_TYPES;

	new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
	if (hAreas == INVALID_HANDLE) return NUM_TRAVERSE_TYPES;
	
	new iAreaIndex = FindValueInArray(hAreas, iAreaID);
	if (iAreaIndex == -1) return NUM_TRAVERSE_TYPES;
	
	return GetArrayCell(hAreas, iAreaIndex, NavMeshArea_ParentHow);
}

stock NavMeshAreaSetParent(iAreaID, iParentAreaID)
{
	if (!g_bNavMeshBuilt) return;

	new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
	if (hAreas == INVALID_HANDLE) return;
	
	new iAreaIndex = FindValueInArray(hAreas, iAreaID);
	if (iAreaIndex == -1) return;
	
	SetArrayCell(hAreas, iAreaIndex, iParentAreaID, NavMeshArea_Parent);
}

stock NavMeshAreaSetParentHow(iAreaID, iParentHow)
{
	if (!g_bNavMeshBuilt) return;

	new Handle:hAreas = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
	if (hAreas == INVALID_HANDLE) return;
	
	new iAreaIndex = FindValueInArray(hAreas, iAreaID);
	if (iAreaIndex == -1) return;
	
	SetArrayCell(hAreas, iAreaIndex, iParentHow, NavMeshArea_ParentHow);
}

stock Float:NavMeshLadderGetLength(iLadderID)
{
	if (!g_bNavMeshBuilt) return 0.0;

	new Handle:hLadders = Handle:GetArrayCell(g_hNavMesh, 0, NavMesh_Ladders);
	if (hLadders == INVALID_HANDLE) return 0.0;
	
	new iLadderIndex = FindValueInArray(hLadders, iLadderID);
	if (iLadderIndex == -1) return 0.0;
	
	return Float:GetArrayCell(hLadders, iLadderIndex, NavMeshLadder_Length);
}

//	==================================
//	API
//	==================================

public Native_NavMeshExists(Handle:plugin, numParams)
{
	return g_bNavMeshBuilt;
}

public Native_NavMeshGetMagicNumber(Handle:plugin, numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve magic number because the nav mesh doesn't exist!");
		return -1;
	}
	
	return GetArrayCell(g_hNavMesh, 0, NavMesh_MagicNumber);
}

public Native_NavMeshGetVersion(Handle:plugin, numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve version because the nav mesh doesn't exist!");
		return -1;
	}
	
	return GetArrayCell(g_hNavMesh, 0, NavMesh_Version);
}

public Native_NavMeshGetSubVersion(Handle:plugin, numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve subversion because the nav mesh doesn't exist!");
		return -1;
	}
	
	return GetArrayCell(g_hNavMesh, 0, NavMesh_SubVersion);
}

public Native_NavMeshGetSaveBSPSize(Handle:plugin, numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve save BSP size because the nav mesh doesn't exist!");
		return -1;
	}
	
	return GetArrayCell(g_hNavMesh, 0, NavMesh_SaveBSPSize);
}

public Native_NavMeshIsAnalyzed(Handle:plugin, numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve analysis state because the nav mesh doesn't exist!");
		return 0;
	}
	
	return GetArrayCell(g_hNavMesh, 0, NavMesh_IsMeshAnalyzed);
}

public Native_NavMeshGetPlaces(Handle:plugin, numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve place list because the nav mesh doesn't exist!");
		return _:INVALID_HANDLE;
	}
	
	return GetArrayCell(g_hNavMesh, 0, NavMesh_Places);
}

public Native_NavMeshGetAreas(Handle:plugin, numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve area list because the nav mesh doesn't exist!");
		return _:INVALID_HANDLE;
	}
	
	return GetArrayCell(g_hNavMesh, 0, NavMesh_Areas);
}

public Native_NavMeshGetLadders(Handle:plugin, numParams)
{
	if (!g_bNavMeshBuilt)
	{
		LogError("Could not retrieve ladder list because the nav mesh doesn't exist!");
		return _:INVALID_HANDLE;
	}
	
	return GetArrayCell(g_hNavMesh, 0, NavMesh_Ladders);
}

public Native_NavMeshBuildPath(Handle:plugin, numParams)
{
	decl Float:flGoalPos[3];
	GetNativeArray(3, flGoalPos, 3);
	
	return NavMeshBuildPath(GetNativeCell(1), 
		GetNativeCell(2), 
		flGoalPos,
		Function:GetNativeCell(4),
		GetNativeCell(5));
}

public Native_NavMeshAreaGetFlags(Handle:plugin, numParams)
{
	return NavMeshAreaGetFlags(GetNativeCell(1));
}

public Native_NavMeshAreaGetCenter(Handle:plugin, numParams)
{
	decl Float:flResult[3];
	if (NavMeshAreaGetCenter(GetNativeCell(1), flResult))
	{
		SetNativeArray(2, flResult, 3);
		return true;
	}
	
	return false;
}

public Native_NavMeshAreaGetAdjacentList(Handle:plugin, numParams)
{
	return _:NavMeshAreaGetAdjacentList(GetNativeCell(1), GetNativeCell(2));
}

public Native_NavMeshAreaGetLadderList(Handle:plugin, numParams)
{
	return _:NavMeshAreaGetLadderList(GetNativeCell(1), GetNativeCell(2));
}

public Native_NavMeshAreaGetTotalCost(Handle:plugin, numParams)
{
	return _:NavMeshAreaGetTotalCost(GetNativeCell(1));
}

public Native_NavMeshAreaGetCostSoFar(Handle:plugin, numParams)
{
	return _:NavMeshAreaGetCostSoFar(GetNativeCell(1));
}

public Native_NavMeshAreaGetParent(Handle:plugin, numParams)
{
	return NavMeshAreaGetParent(GetNativeCell(1));
}

public Native_NavMeshAreaGetParentHow(Handle:plugin, numParams)
{
	return NavMeshAreaGetParentHow(GetNativeCell(1));
}

public Native_NavMeshAreaSetParent(Handle:plugin, numParams)
{
	NavMeshAreaSetParent(GetNativeCell(1), GetNativeCell(2));
}

public Native_NavMeshAreaSetParentHow(Handle:plugin, numParams)
{
	NavMeshAreaSetParentHow(GetNativeCell(1), GetNativeCell(2));
}

public Native_NavMeshLadderGetLength(Handle:plugin, numParams)
{
	return _:NavMeshLadderGetLength(GetNativeCell(1));
}