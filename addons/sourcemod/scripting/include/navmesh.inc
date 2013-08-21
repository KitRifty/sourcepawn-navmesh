#if defined _navmesh_included
 #endinput
#endif
#define _navmesh_included

enum
{
	NAV_DIR_NORTH = 0,
	NAV_DIR_EAST,
	NAV_DIR_SOUTH,
	NAV_DIR_WEST,
	NAV_DIR_COUNT
};

enum
{
	NAV_CORNER_NORTH_WEST = 0,
	NAV_CORNER_NORTH_EAST,
	NAV_CORNER_SOUTH_EAST,
	NAV_CORNER_SOUTH_WEST,
	NAV_CORNER_COUNT
};

enum
{
	NAV_LADDER_DIR_UP = 0,
	NAV_LADDER_DIR_DOWN,
	NAV_LADDER_DIR_COUNT
};

enum
{
	NAV_MESH_CROUCH = 0x0001, 
	NAV_MESH_JUMP = 0x0002, 
	NAV_MESH_PRECISE = 0x0004, 
	NAV_MESH_NO_JUMP = 0x0008, 
	NAV_MESH_STOP = 0x0010, 
	NAV_MESH_RUN = 0x0020, 
	NAV_MESH_WALK = 0x0040, 
	NAV_MESH_AVOID = 0x0080, 
	NAV_MESH_TRANSIENT = 0x0100, 
	NAV_MESH_DONT_HIDE = 0x0200, 
	NAV_MESH_STAND = 0x0400, 
	NAV_MESH_NO_HOSTAGES = 0x0800
};

enum
{
	GO_NORTH = 0,
	GO_EAST,
	GO_SOUTH,
	GO_WEST,
	GO_LADDER_UP,
	GO_LADDER_DOWN,
	GO_JUMP,
	NUM_TRAVERSE_TYPES
};

enum
{
	NavMesh_MagicNumber = 0,
	NavMesh_Version,
	NavMesh_SubVersion,
	NavMesh_SaveBSPSize,
	NavMesh_IsMeshAnalyzed,
	NavMesh_Places,
	NavMesh_Areas,
	NavMesh_Ladders,
	NavMesh_MaxStats
};

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
	NavMeshArea_NECornerZ,
	NavMeshArea_SWCornerZ,
	NavMeshArea_Connections,
	NavMeshArea_HidingSpots,
	NavMeshArea_EncounterPaths,
	NavMeshArea_LadderConnections,
	NavMeshArea_CornerLightIntensityNW,
	NavMeshArea_CornerLightIntensityNE,
	NavMeshArea_CornerLightIntensitySE,
	NavMeshArea_CornerLightIntensitySW,
	NavMeshArea_VisibleAreas,
	NavMeshArea_InheritVisibilityFrom,
	NavMeshArea_EarliestOccupyTimeFirstTeam,
	NavMeshArea_EarliestOccupyTimeSecondTeam,
	NavMeshArea_unk01,
	NavMeshArea_Blocked,
	NavMeshArea_Parent,
	NavMeshArea_ParentHow,
	NavMeshArea_CostSoFar,
	NavMeshArea_TotalCost,
	NavMeshArea_Marker,
	NavMeshArea_OpenMarker,
	NavMeshArea_PrevOpenIndex,
	NavMeshArea_NextOpenIndex,
	NavMeshArea_PathLengthSoFar,
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
	NavMeshHidingSpot_MaxStats
};

enum
{
	NavMeshEncounterPath_FromID = 0,
	NavMeshEncounterPath_FromDirection,
	NavMeshEncounterPath_ToID,
	NavMeshEncounterPath_ToDirection,
	NavMeshEncounterPath_Spots,
	NavMeshEncounterPath_MaxStats
};

enum
{
	NavMeshEncounterSpot_OrderID = 0,
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


native NavMesh_Exists();

native NavMesh_GetMagicNumber();

native NavMesh_GetVersion();

native NavMesh_GetSubVersion();

native NavMesh_GetSaveBSPSize();

native bool:NavMesh_IsAnalyzed();

native Handle:NavMesh_GetPlaces();

native Handle:NavMesh_GetAreas();

native Handle:NavMesh_GetLadders();


native bool:NavMesh_BuildPath(iStartAreaIndex, iGoalAreaIndex, Float:flGoalPos[3], Function:iCostFunctor, &iClosestAreaIndex=-1, Float:flMaxPathLength=0.0);


native NavMeshArea_GetFlags(iAreaIndex);

native bool:NavMeshArea_GetCenter(iAreaIndex, Float:flBuffer[3]);

native Handle:NavMeshArea_GetAdjacentList(iAreaIndex, iDirection);

native Handle:NavMeshArea_GetLadderList(iAreaIndex, iLadderDirection);

native NavMeshArea_GetTotalCost(iAreaIndex);

native NavMeshArea_GetCostSoFar(iAreaIndex);

native NavMeshArea_GetParent(iAreaIndex);

native NavMeshArea_GetParentHow(iAreaIndex);

native NavMeshArea_SetParent(iAreaIndex, iParentAreaID);

native NavMeshArea_SetParentHow(iAreaIndex, iParentHow);

native bool:NavMeshArea_GetExtentLow(iAreaIndex, Float:flBuffer[3]);

native bool:NavMeshArea_GetExtentHigh(iAreaIndex, Float:flBuffer[3]);

native bool:NavMeshArea_IsOverlappingPoint(iAreaIndex, const Float:flPos[3], Float:flTolerance);

native bool:NavMeshArea_IsOverlappingArea(iAreaIndex, iTargetAreaID);

native Float:NavMeshArea_GetNECornerZ(iAreaIndex);

native Float:NavMeshArea_GetSWCornerZ(iAreaIndex);

native Float:NavMeshArea_GetZ(iAreaIndex, const Float:flPos[3]);

native bool:NavMeshArea_Contains(iAreaIndex, const Float:flPos[3]);


native Float:NavMeshLadder_GetLength(iLadderID);


// Shortest-path cost functor for NavMeshBuildPath
public NavMeshShortestPathCost(iAreaIndex, iFromAreaIndex, iLadderIndex)
{
	if (iFromAreaIndex == -1)
	{
		return 0;
	}
	else
	{
		new iDist;
		
		if (iLadderIndex != -1)
		{
			iDist = RoundFloat(NavMeshLadder_GetLength(iLadderIndex));
		}
		else
		{
			decl Float:flAreaCenter[3], Float:flFromAreaCenter[3];
			NavMeshArea_GetCenter(iAreaIndex, flAreaCenter);
			NavMeshArea_GetCenter(iFromAreaIndex, flFromAreaCenter);
			
			iDist = RoundFloat(GetVectorDistance(flAreaCenter, flFromAreaCenter));
		}
		
		new iCost = iDist + NavMeshArea_GetCostSoFar(iFromAreaIndex);
		
		new iAreaFlags = NavMeshArea_GetFlags(iAreaIndex);
		if (iAreaFlags & NAV_MESH_CROUCH) iCost += 20;
		if (iAreaFlags & NAV_MESH_JUMP) iCost += (5 * iDist);
		
		return iCost;
	}
}


public SharedPlugin:__pl_navmesh = 
{
	name = "navmesh",
	file = "navmesh.smx",
#if defined REQUIRE_PLUGIN
	required = 1,
#else
	required = 0,
#endif
};