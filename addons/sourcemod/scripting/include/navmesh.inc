#if defined _navmesh_included
 #endinput
#endif
#define _navmesh_included

#define HalfHumanHeight 35.5
#define StepHeight 18.0

#define NAV_MAGIC_NUMBER 0xFEEDFACE

#define WALK_THRU_PROP_DOORS		0x01
#define WALK_THRU_FUNC_DOORS		0x02
#define WALK_THRU_DOORS 			(WALK_THRU_PROP_DOORS | WALK_THRU_FUNC_DOORS)
#define WALK_THRU_BREAKABLES		0x04
#define WALK_THRU_TOGGLE_BRUSHES	0x08
#define WALK_THRU_EVERYTHING		(WALK_THRU_DOORS | WALK_THRU_BREAKABLES | WALK_THRU_TOGGLE_BRUSHES)

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

enum NavCornerType
{
	NAV_CORNER_NORTH_WEST = 0,
	NAV_CORNER_NORTH_EAST,
	NAV_CORNER_SOUTH_EAST,
	NAV_CORNER_SOUTH_WEST,
	NAV_CORNER_COUNT
};

const CNavArea INVALID_NAV_AREA = view_as<CNavArea>(-1);
const CNavLadder INVALID_NAV_LADDER = view_as<CNavLadder>(-1);
const HidingSpot INVALID_NAV_HIDING_SPOT = view_as<HidingSpot>(-1);

typedef NavPathCostFunctor = function int(CNavArea area, CNavArea from, CNavLadder ladder, any data);


// NavMesh

native bool NavMesh_Exists();

native int NavMesh_GetMagicNumber();

native int NavMesh_GetVersion();

native int NavMesh_GetSubVersion();

native int NavMesh_GetSaveBSPSize();

native bool NavMesh_IsAnalyzed();

native bool NavMesh_GetGroundHeight(const float pos[3], float &height=-1.0, float normal[3]=NULL_VECTOR);

native CNavArea NavMesh_FindAreaByID( int areaID );

native CNavArea NavMesh_GetArea(const float pos[3], float beneathLimit=120.0);

native CNavArea NavMesh_GetNearestArea(const float pos[3], bool anyZ=false, float maxDist=10000.0, bool checkLOS=false, bool checkGround=true, int team=-2);

native HidingSpot NavMesh_FindHidingSpotByID(int hidingSpotID);

native HidingSpot NavMesh_GetRandomHidingSpot();

// These grid functions are more internal than anything.
native int NavMesh_WorldToGridX(float wx);

native int NavMesh_WorldToGridY(float wy);

native void NavMesh_GetAreasOnGrid(ArrayStack hStack, int x, int y);

native int NavMesh_GetGridSizeX();

native int NavMesh_GetGridSizeY();


native void NavMesh_CollectSurroundingAreas(ArrayStack hStack, CNavArea startArea, float travelDistLimit=1500.0, float maxStepUpLimit=StepHeight, float maxDropDownLimit=100.0);

native bool NavMesh_BuildPath(CNavArea startArea, CNavArea goalArea, const float goalPos[3], NavPathCostFunctor costFunctor, any costData=0, CNavArea &closestArea=INVALID_NAV_AREA, float maxPathLength=0.0, float flMaxStepSize=0.0);



// CNavArea

native int NavMeshArea_GetID(int areaIndex);

native int NavMeshArea_GetFlags(int areaIndex);

native void NavMeshArea_GetPlace(int areaIndex, char[] buffer, int maxlen);

native bool NavMeshArea_GetCenter(int areaIndex, float buffer[3]);

native void NavMeshArea_GetAdjacentList(ArrayStack buffer, int areaIndex, int dir);

native void NavMeshArea_GetLadderList(ArrayStack buffer, int areaIndex, int dir);

native void NavMeshArea_GetHidingSpots(ArrayStack buffer, int areaIndex);

native void NavMeshArea_GetClosestPointOnArea(int areaIndex, const float pos[3], float closePos[3]);

native int NavMeshArea_GetTotalCost(int areaIndex);

native int NavMeshArea_GetCostSoFar(int areaIndex);

native int NavMeshArea_GetParent(int areaIndex);

native int NavMeshArea_GetParentHow(int areaIndex);

native void NavMeshArea_SetParent(int areaIndex, int parentAreaIndex);

native void NavMeshArea_SetParentHow(int areaIndex, int parentHow);

native bool NavMeshArea_GetExtentLow(int areaIndex, float buffer[3]);

native bool NavMeshArea_GetExtentHigh(int areaIndex, float buffer[3]);

native bool NavMeshArea_IsOverlappingPoint(int areaIndex, const float pos[3], float tolerance);

native bool NavMeshArea_IsOverlappingArea(int areaIndex, int targetAreaIndex);

native float NavMeshArea_GetNECornerZ(int areaIndex);

native float NavMeshArea_GetSWCornerZ(int areaIndex);

native void NavMeshArea_GetCorner(int areaIndex, NavCornerType corner, float buffer[3]);

native float NavMeshArea_GetZ(int areaIndex, const float pos[3]);

native float NavMeshArea_GetZFromXAndY(int areaIndex, float x, float y);

native bool NavMeshArea_IsEdge(int areaIndex, int dir);

native bool NavMeshArea_Contains(int areaIndex, const float pos[3]);

native void NavMeshArea_GetRandomPoint(int areaIndex, float buffer[3]);

native bool NavMeshArea_IsConnected(int areaIndex, int targetAreaIndex, int dir);

native bool NavMeshArea_ComputePortal(int areaIndex, int areaToIndex, int dir, float center[3], float &halfWidth);

native bool NavMeshArea_ComputeClosestPointInPortal(int areaIndex, int areaToIndex, int dir, const float fromPos[3], float closePos[3]);

native int NavMeshArea_ComputeDirection(int areaIndex, const float pos[3]);

native float NavMeshArea_GetLightIntensity(int areaIndex, const float pos[3]);

methodmap CNavArea
{
	public CNavArea( int index )
	{
		return view_as<CNavArea>(index);
	}
	
	property int Index
	{
		public get() { return view_as<int>(this); }
	}
	
	property int ID
	{
		public get() { return NavMeshArea_GetID(this.Index); }
	}
	
	property int Attributes
	{
		public get() { return NavMeshArea_GetFlags(this.Index); }
	}
	
	property CNavArea Parent
	{
		public get() { return CNavArea(NavMeshArea_GetParent(this.Index)); }
		public set( CNavArea area ) { NavMeshArea_SetParent(this.Index, area.Index); }
	}
	
	property int ParentHow
	{
		public get() { return NavMeshArea_GetParentHow(this.Index); }
		public set( int how ) { NavMeshArea_SetParentHow(this.Index, how); }
	}
	
	property int TotalCost
	{
		public get() { return NavMeshArea_GetTotalCost(this.Index); }
	}
	
	property int CostSoFar
	{
		public get() { return NavMeshArea_GetCostSoFar(this.Index); }
	}
	
	property float NECornerZ
	{
		public get() { return NavMeshArea_GetNECornerZ(this.Index); }
	}
	
	property float SWCornerZ
	{
		public get() { return NavMeshArea_GetSWCornerZ(this.Index); }
	}
	
	public void GetPlace( char[] buffer, int maxlen )
	{
		NavMeshArea_GetPlace(this.Index, buffer, maxlen);
	}
	
	public bool GetCenter( float buffer[3] )
	{
		return NavMeshArea_GetCenter(this.Index, buffer);
	}
	
	public bool GetExtentLow( float buffer[3] )
	{
		return NavMeshArea_GetExtentLow(this.Index, buffer);
	}
	
	public bool GetExtentHigh( float buffer[3] )
	{
		return NavMeshArea_GetExtentHigh(this.Index, buffer);
	}
	
	public void GetCorner( NavCornerType corner, float buffer[3] )
	{
		NavMeshArea_GetCorner( this.Index, corner, buffer );
	}
	
	public float GetZ( const float point[3] )
	{
		return NavMeshArea_GetZ(this.Index, point);
	}
	
	public float GetZFromXAndY( float x, float y )
	{
		return NavMeshArea_GetZFromXAndY(this.Index, x, y);
	}
	
	public bool IsOverlappingPoint( const float point[3], float tolerance=0.0 )
	{
		return NavMeshArea_IsOverlappingPoint(this.Index, point, tolerance);
	}
	
	public bool IsOverlappingArea( CNavArea area )
	{
		return NavMeshArea_IsOverlappingArea(this.Index, area.Index);
	}
	
	public bool Contains( const float point[3] )
	{
		return NavMeshArea_Contains(this.Index, point);
	}
	
	public bool IsEdge( int dir )
	{
		return NavMeshArea_IsEdge(this.Index, dir);
	}
	
	public void GetRandomPoint( float buffer[3] )
	{
		NavMeshArea_GetRandomPoint(this.Index, buffer);
	}
	
	public bool IsConnected( CNavArea targetArea, int dir )
	{
		return NavMeshArea_IsConnected(this.Index, targetArea.Index, dir);
	}
	
	public void GetClosestPointOnArea( const float pos[3], float buffer[3] )
	{
		NavMeshArea_GetClosestPointOnArea(this.Index, pos, buffer);
	}
	
	public void GetAdjacentAreas( int dir, ArrayStack buffer )
	{
		NavMeshArea_GetAdjacentList( buffer, this.Index, dir );
	}
	
	public int GetAdjacentCount( int dir )
	{
		ArrayStack buffer = new ArrayStack();
		this.GetAdjacentAreas( dir, buffer );
		int count = 0;
		while (!buffer.Empty)
		{
			int value;
			PopStackCell(buffer, value);
			count++;
		}
		delete buffer;
		return count;
	}
	
	public CNavArea GetRandomAdjacentArea( int dir )
	{
		ArrayStack buffer = new ArrayStack();
		this.GetAdjacentAreas( dir, buffer );
		
		if (buffer.Empty)
		{
			delete buffer;
			return INVALID_NAV_AREA;
		}
		
		ArrayList list = new ArrayList();
		while (!buffer.Empty)
		{
			int value;
			PopStackCell(buffer, value);
			list.Push(value);
		}
		
		delete buffer;
		CNavArea area = CNavArea( list.Get( GetRandomInt(0, list.Length - 1) ) );
		delete list;
		return area;
	}
	
	public void GetLadders( int dir, ArrayStack buffer )
	{
		NavMeshArea_GetLadderList( buffer, this.Index, dir );
	}
	
	public void GetHidingSpots( ArrayStack buffer )
	{
		NavMeshArea_GetHidingSpots( buffer, this.Index );
	}
	
	public void ComputePortal( CNavArea to, int dir, float center[3], float &halfWidth )
	{
		NavMeshArea_ComputePortal(this.Index, to.Index, dir, center, halfWidth);
	}
	
	public void ComputeClosestPointInPortal( CNavArea to, int dir, const float fromPos[3], float closePos[3] )
	{
		NavMeshArea_ComputeClosestPointInPortal(this.Index, to.Index, dir, fromPos, closePos);
	}
	
	public int ComputeDirection( const float point[3] )
	{
		return NavMeshArea_ComputeDirection(this.Index, point);
	}
	
	public float GetLightIntensity( const float point[3] )
	{
		return NavMeshArea_GetLightIntensity(this.Index, point);
	}
}


// HidingSpot

enum
{
	NAV_HIDING_SPOT_IN_COVER = 0x01,
	NAV_HIDING_SPOT_GOOD_SNIPER_SPOT = 0x02,
	NAV_HIDING_SPOT_IDEAL_SNIPER_SPOT = 0x04,
	NAV_HIDING_SPOT_EXPOSED = 0x08
}

native int NavHidingSpot_GetID(int hidingSpotIndex);

native int NavHidingSpot_GetFlags(int hidingSpotIndex);

native void NavHidingSpot_GetPosition(int hidingSpotIndex, float buffer[3]);

native CNavArea NavHidingSpot_GetArea(int hidingSpotIndex);

methodmap HidingSpot
{
	public HidingSpot( int index )
	{
		return view_as<HidingSpot>(index);
	}
	
	property int Index
	{
		public get() { return view_as<int>(this); }
	}
	
	property int ID
	{
		public get() { return NavHidingSpot_GetID( this.Index ); }
	}
	
	property int Flags
	{
		public get() { return NavHidingSpot_GetFlags( this.Index ); }
	}
	
	public void GetPosition( float buffer[3] )
	{
		NavHidingSpot_GetPosition( this.Index, buffer );
	}
	
	public bool HasGoodCover()
	{
		return this.Flags & NAV_HIDING_SPOT_IN_COVER ? true : false;
	}
	
	public bool IsGoodSniperSpot()
	{
		return this.Flags & NAV_HIDING_SPOT_GOOD_SNIPER_SPOT ? true : false;
	}
	
	public bool IsIdealSniperSpot()
	{
		return this.Flags & NAV_HIDING_SPOT_IDEAL_SNIPER_SPOT ? true : false;
	}
	
	public bool IsExposed()
	{
		return this.Flags & NAV_HIDING_SPOT_EXPOSED ? true : false;
	}
	
	public CNavArea GetArea()
	{
		return NavHidingSpot_GetArea(this.Index);
	}
}


// CNavLadder

native float NavMeshLadder_GetLength(int ladderIndex);

native float NavMeshLadder_GetWidth(int ladderIndex);

native CNavArea NavMeshLadder_GetTopForwardArea(int ladderIndex);

native CNavArea NavMeshLadder_GetTopLeftArea(int ladderIndex);

native CNavArea NavMeshLadder_GetTopRightArea(int ladderIndex);

native CNavArea NavMeshLadder_GetTopBehindArea(int ladderIndex);

native CNavArea NavMeshLadder_GetBottomArea(int ladderIndex);

native void NavMeshLadder_GetTop(int ladderIndex, float buffer[3]);

native void NavMeshLadder_GetBottom(int ladderIndex, float buffer[3]);

methodmap CNavLadder
{
	public CNavLadder( int index )
	{
		return view_as<CNavLadder>(index);
	}
	
	property int Index
	{
		public get() { return view_as<int>(this); }
	}
	
	property float Length
	{
		public get() { return NavMeshLadder_GetLength(this.Index); }
	}
	
	property float Width
	{
		public get() { return NavMeshLadder_GetWidth(this.Index); }
	}
	
	property CNavArea TopForwardArea
	{
		public get() { return NavMeshLadder_GetTopForwardArea(this.Index); }
	}
	
	property CNavArea TopLeftArea
	{
		public get() { return NavMeshLadder_GetTopLeftArea(this.Index); }
	}
	
	property CNavArea TopRightArea
	{
		public get() { return NavMeshLadder_GetTopRightArea(this.Index); }
	}
	
	property CNavArea TopBehindArea
	{
		public get() { return NavMeshLadder_GetTopBehindArea(this.Index); }
	}
	
	property CNavArea BottomArea
	{
		public get() { return NavMeshLadder_GetBottomArea(this.Index); }
	}
	
	public void GetTop( float buffer[3] )
	{
		NavMeshLadder_GetTop(this.Index, buffer);
	}
	
	public void GetBottom( float buffer[3] )
	{
		NavMeshLadder_GetBottom(this.Index, buffer);
	}
}


// SpotEncounter

native CNavArea NavSpotEncounter_GetFrom(int spotEncounterIndex);

native int NavSpotEncounter_GetFromDirection(int spotEncounterIndex);

native CNavArea NavSpotEncounter_GetTo(int spotEncounterIndex);

native int NavSpotEncounter_GetToDirection(int spotEncounterIndex);

native void NavSpotEncounter_GetSpots(int spotEncounterIndex, ArrayStack buffer);

methodmap SpotEncounter
{
	public SpotEncounter( int index )
	{
		return view_as<SpotEncounter>(index);
	}
	
	property int Index
	{
		public get() { return view_as<int>(this); }
	}
	
	property CNavArea From
	{
		public get() { return NavSpotEncounter_GetFrom(this.Index); }
	}
	
	property int FromDirection
	{
		public get() { return NavSpotEncounter_GetFromDirection(this.Index); }
	}
	
	property CNavArea To
	{
		public get() { return NavSpotEncounter_GetTo(this.Index); }
	}
	
	property int ToDirection
	{
		public get() { return NavSpotEncounter_GetToDirection(this.Index); }
	}
	
	public void GetSpots( ArrayStack buffer )
	{
		NavSpotEncounter_GetSpots(this.Index, buffer);
	}
}


// SpotOrder

native HidingSpot NavSpotOrder_GetHidingSpot(int spotOrderIndex);

native int NavSpotOrder_GetParametricDistance(int spotOrderIndex);

methodmap SpotOrder
{
	public SpotOrder( int index )
	{
		return view_as<SpotOrder>(index);
	}
	
	property int Index
	{
		public get() { return view_as<int>(this); }
	}
	
	property HidingSpot HidingSpot
	{
		public get() { return NavSpotOrder_GetHidingSpot(this.Index); }
	}
	
	property int ParametricDistance
	{
		public get() { return NavSpotOrder_GetParametricDistance(this.Index); }
	}
}


// Shortest-path cost function for NavMesh_BuildPath.
public int NavMeshShortestPathCost(CNavArea area, CNavArea from, CNavLadder ladder, any data)
{
	if (from == INVALID_NAV_AREA)
	{
		return 0;
	}
	else
	{
		int iDist = 0;
		if (ladder != INVALID_NAV_LADDER)
		{
			iDist = RoundFloat(ladder.Length);
		}
		else
		{
			float flAreaCenter[3]; float flFromAreaCenter[3];
			area.GetCenter(flAreaCenter);
			from.GetCenter(flFromAreaCenter);
			
			iDist = RoundFloat(GetVectorDistance(flAreaCenter, flFromAreaCenter));
		}
		
		int iCost = iDist + from.CostSoFar;
		int iAreaFlags = area.Attributes;
		if (iAreaFlags & NAV_MESH_CROUCH) iCost += 20;
		if (iAreaFlags & NAV_MESH_JUMP) iCost += (5 * iDist);
		return iCost;
	}
}

// stocks

stock bool IsEntityWalkable(int iEnt, int iFlags)
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

stock bool IsWalkableTraceLineClear(const float flFrom[3], const float flTo[3], int iFlags)
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
		delete hTrace;
		
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

public bool TraceFilterWalkableEntities(int entity, int mask, any iFlags)
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