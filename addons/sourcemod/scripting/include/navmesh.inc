#if defined _navmesh_included
 #endinput
#endif
#define _navmesh_included

#define HalfHumanHeight 35.5

#define StepHeight 18.0

#define NAV_MAGIC_NUMBER 0xFEEDFACE

const float PI = 3.14159265358979323846264338327950288419716939937510;

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
	NavMeshHidingSpot_MaxStats
};

enum
{
	NavMeshEncounterPath_FromID = 0,
	NavMeshEncounterPath_FromDirection,
	NavMeshEncounterPath_ToID,
	NavMeshEncounterPath_ToDirection,
	NavMeshEncounterPath_SpotsStartIndex,
	NavMeshEncounterPath_SpotsEndIndex,
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


native bool NavMesh_Exists();

native int NavMesh_GetMagicNumber();

native int NavMesh_GetVersion();

native int NavMesh_GetSubVersion();

native int NavMesh_GetSaveBSPSize();

native bool NavMesh_IsAnalyzed();

native Handle NavMesh_GetPlaces();

native Handle NavMesh_GetAreas();

native Handle NavMesh_GetLadders();

native int NavMesh_CollectSurroundingAreas(Handle hStack,int iStartAreaIndex, float flTravelDistanceLimit=1500.0, float flMaxStepUpLimit=StepHeight, float flMaxDropDownLimit=100.0);

native bool NavMesh_BuildPath(int iStartAreaIndex,int iGoalAreaIndex, const float flGoalPos[3], Function iCostFunctor, any iCostData=INVALID_HANDLE,int &iClosestAreaIndex=-1, float flMaxPathLength=0.0, float flMaxAng=0.0, bool bIgnoreBlockedNav = false, float flMaxStepSize=0.0);


native int NavMesh_GetArea(const float flPos[3], float flBeneathLimit=120.0);

native int NavMesh_GetNearestArea(const float flPos[3], bool bAnyZ=false, float flMaxDist=10000.0, bool bCheckLOS=false, bool bCheckGround=true,int iTeam=-2);


native int NavMesh_WorldToGridX(float flWX);

native int NavMesh_WorldToGridY(float flWY);

native int NavMesh_GetAreasOnGrid(Handle hStack,int x,int y);

native int NavMesh_GetGridSizeX();

native int NavMesh_GetGridSizeY();


native bool NavMesh_GetGroundHeight(const float flPos[3], float &flHeight=-1.0, float flNormal[3]=NULL_VECTOR);


native bool NavMeshArea_IsConnected(int iAreaIndex1,int iAreaIndex2);

native int NavMeshArea_GetFlags(int iAreaIndex);

native bool NavMeshArea_GetCenter(int iAreaIndex, float flBuffer[3]);

native int NavMeshArea_GetAdjacentList(Handle hStack,int iAreaIndex,int iDirection);

native int NavMeshArea_GetLadderList(Handle hStack,int iAreaIndex,int iLadderDirection);

native int NavMeshArea_GetClosestPointOnArea(int iAreaIndex, const float flPos[3], float flClose[3]);

native int NavMeshArea_GetTotalCost(int iAreaIndex);

native int NavMeshArea_GetCostSoFar(int iAreaIndex);

native int NavMeshArea_GetParent(int iAreaIndex);

native int NavMeshArea_GetParentHow(int iAreaIndex);

native int NavMeshArea_SetParent(int iAreaIndex,int iParentAreaID);

native int NavMeshArea_SetParentHow(int iAreaIndex,int iParentHow);

native bool NavMeshArea_GetExtentLow(int iAreaIndex, float flBuffer[3]);

native bool NavMeshArea_GetExtentHigh(int iAreaIndex, float flBuffer[3]);

native bool NavMeshArea_IsOverlappingPoint(int iAreaIndex, const float flPos[3], float flTolerance);

native bool NavMeshArea_IsOverlappingArea(int iAreaIndex,int iTargetAreaID);

native float NavMeshArea_GetNECornerZ(int iAreaIndex);

native float NavMeshArea_GetSWCornerZ(int iAreaIndex);

native float NavMeshArea_GetZ(int iAreaIndex, const float flPos[3]);

native float NavMeshArea_GetZFromXAndY(int iAreaIndex, float flX, float flY);

native bool NavMeshArea_Contains(int iAreaIndex, const float flPos[3]);

native bool NavMeshArea_ComputePortal(int iAreaIndex,int iAreaToIndex,int iNavDirection, float flCenter[3], float &flHalfWidth);

native bool NavMeshArea_ComputeClosestPointInPortal(int iAreaIndex,int iAreaToIndex,int iNavDirection, const float flFromPos[3], float flClosestPos[3]);

native int NavMeshArea_ComputeDirection(int iAreaIndex, const float flPos[3]);

native float NavMeshArea_GetLightIntensity(int iAreaIndex, const float flPos[3]);

native float NavMeshLadder_GetLength(int iLadderID);


// Shortest-path cost function for NavMesh_BuildPath.
public int NavMeshShortestPathCost(int iAreaIndex,int iFromAreaIndex,int iLadderIndex, any data)
{
	if (iFromAreaIndex == -1)
	{
		return 0;
	}
	else
	{
		int iDist;
		
		if (iLadderIndex != -1)
		{
			iDist = RoundFloat(NavMeshLadder_GetLength(iLadderIndex));
		}
		else
		{
			float flAreaCenter[3], flFromAreaCenter[3];
			NavMeshArea_GetCenter(iAreaIndex, flAreaCenter);
			NavMeshArea_GetCenter(iFromAreaIndex, flFromAreaCenter);
			
			iDist = RoundFloat(GetVectorDistance(flAreaCenter, flFromAreaCenter));
		}
		
		int iCost = iDist + NavMeshArea_GetCostSoFar(iFromAreaIndex);
		
		int iAreaFlags = NavMeshArea_GetFlags(iAreaIndex);
		if (iAreaFlags & NAV_MESH_CROUCH) iCost += 20;
		if (iAreaFlags & NAV_MESH_JUMP) iCost += (5 * iDist);
		
		return iCost;
	}
}

#define WALK_THRU_PROP_DOORS		0x01
#define WALK_THRU_FUNC_DOORS		0x02
#define WALK_THRU_DOORS				(WALK_THRU_PROP_DOORS | WALK_THRU_FUNC_DOORS)
#define WALK_THRU_BREAKABLES		0x04
#define WALK_THRU_TOGGLE_BRUSHES	0x08
#define WALK_THRU_EVERYTHING		(WALK_THRU_DOORS | WALK_THRU_BREAKABLES | WALK_THRU_TOGGLE_BRUSHES)


stock bool IsEntityWalkable(int iEnt,int iFlags)
{
	char sClass[64];
	GetEntityClassname(iEnt, sClass, sizeof(sClass));
	
	if (!strcmp(sClass, "worldspawn")) return false;
	
	if (!strcmp(sClass, "player")) return false;
	
	if (!strcmp(sClass, "func_door*"))
	{
		return (iFlags & WALK_THRU_PROP_DOORS) ? true : false;
	}
	
	if (!strcmp(sClass, "prop_door*"))
	{
		return (iFlags & WALK_THRU_PROP_DOORS) ? true : false;
	}
	
	if (!strcmp(sClass, "func_brush"))
	{
		int iSolidity = GetEntProp(iEnt, Prop_Data, "m_iSolidity");
		
		switch (iSolidity)
		{
			case 2: // BRUSHSOLID_ALWAYS
			{
				return false;
			}
			case 1: // BRUSHSOLID_NEVER
			{
				return true;
			}
			case 0: // BRUSHSOLID_TOGGLE
			{
				return (iFlags & WALK_THRU_TOGGLE_BRUSHES) ? true : false;
			}
		}
		
		return (iFlags & WALK_THRU_PROP_DOORS) ? true : false;
	}
	
	if (!strcmp(sClass, "func_breakable") && GetEntProp(iEnt, Prop_Data, "m_iHealth") && GetEntProp(iEnt, Prop_Data, "m_takedamage") == 2) // DAMAGE_YES
	{
		return (iFlags & WALK_THRU_BREAKABLES) ? true : false;
	}
	
	if (!strcmp(sClass, "func_playerinfected_clip"))
	{
		return true;
	}
	
	return false;
}

stock bool IsWalkableTraceLineClear(const float flFrom[3], const float flTo[3],int iFlags)
{
	float flUseFrom[3];
	flUseFrom[0] = flFrom[0];
	flUseFrom[1] = flFrom[1];
	flUseFrom[2] = flFrom[2];
	
	Handle hTrace;
	
	float flFraction = 0.0;
	
	for (int t = 0; t < 50; t++)
	{
		hTrace = TR_TraceRayFilterEx(flUseFrom, flTo, MASK_NPCSOLID, RayType_EndPoint, TraceFilterWalkableEntities, iFlags);
		flFraction = TR_GetFraction(hTrace);
		int iResEnt = TR_GetEntityIndex(hTrace);
		float flResPos[3];
		TR_GetEndPosition(flResPos, hTrace);
		CloseHandle(hTrace);
		
		if (flFraction != 1.0 && IsEntityWalkable(iResEnt, iFlags))
		{
			float flDir[3];
			SubtractVectors(flTo, flFrom, flDir);
			NormalizeVector(flDir, flDir);
			ScaleVector(flDir, 5.0);
			AddVectors(flResPos, flDir, flUseFrom);
		}
		else
		{
			break;
		}
	}
	
	if (flFraction == 1.0) return true;
	
	return false;
}

public bool TraceFilterWalkableEntities(int entity,int mask, any iFlags)
{
	return !IsEntityWalkable(entity, iFlags);
}


public SharedPlugin __pl_navmesh = 
{
	name = "navmesh",
	file = "navmesh.smx",
#if defined REQUIRE_PLUGIN
	required = 1,
#else
	required = 0,
#endif
};