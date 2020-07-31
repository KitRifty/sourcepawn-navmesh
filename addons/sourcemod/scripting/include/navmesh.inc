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

typedef NavAreaFunctor = function bool(CNavArea area, any data);

typedef NavPathCostFunctor = function int(CNavArea area, CNavArea from, CNavLadder ladder, any data);

typedef NavSearchSurroundingAreasFunctor = function bool(CNavArea area, CNavArea from, CNavLadder ladder, any data);

// CNavMesh

enum GetNavAreaFlags_t
{
	GETNAVAREA_CHECK_LOS			= 0x1,
	GETNAVAREA_ALLOW_BLOCKED_AREAS	= 0x2,
	GETNAVAREA_CHECK_GROUND			= 0x4,
}

enum SearchSurroundingAreasFlags_t
{
	SEARCHSURROUNDAREAS_NONE = 0,
	SEARCHSURROUNDAREAS_INCLUDE_INCOMING_CONNECTIONS	= 0x1,
	SEARCHSURROUNDAREAS_INCLUDE_BLOCKED_AREAS = 0x2,
	SEARCHSURROUNDAREAS_EXCLUDE_OUTGOING_CONNECTIONS	= 0x4,
	SEARCHSURROUNDAREAS_EXCLUDE_ELEVATORS				= 0x8
}

/**
 * Called when the plugin finishes loading the navigation mesh, even if it fails. Called within the
 * OnMapStart() forward.
 * 
 * @param success		true if sucessful, false otherwise.
 */
forward void OnNavMeshLoaded(bool success);

methodmap CNavMesh
{
	public CNavMesh( int index ) { return view_as<CNavMesh>(index); }

	/**
	 *	Specifies if the plugin was successful in loading the nav mesh or not.
	 */
	property bool Exists
	{
		public native get();
	}

	/**
	 *	The magic number of the loaded nav mesh.
	 */
	property int MagicNumber
	{
		public native get();
	}

	/**
	 *	The version of the nav mesh.
	 */
	property int Version
	{
		public native get();
	}

	/**
	 *	The subversion of the nav mesh.
	 */
	property int Subversion
	{
		public native get();
	}

	/**
	 *	The size of the map that the nav mesh was built for.
	 */
	property int SaveBSPSize
	{
		public native get();
	}

	/**
	 *	Specifies if the nav mesh has been analyzed.
	 */
	property bool Analyzed
	{
		public native get();
	}

	/**
	 *	Given a position, returns the height of the point from the nav mesh.
	 *
	 *	@param	pos			The position to check.
	 *	@param	height		Height of the position from the nav mesh.
	 *	@param	normal		Optional. The normal vector of the ground beneath the point.
	 *	@return				True if successful; false otherwise.
	 */
	public native bool GetGroundHeight(const float pos[3], float &height=-1.0, float normal[3]=NULL_VECTOR);

	/**
	 *	Given an id, returns the area associated with that id.
	 *
	 *	@param	id	The id of the area to search for.
	 *	@return		A CNavArea, or INVALID_NAV_AREA if no area can be found.
	 */
	public native CNavArea GetNavAreaByID(int id);

	/**
	 *	Given a position, return the nav area that IsOverlapping() and is immediately beneath it.
	 *
	 *	@param	pos				The position to check.
	 *	@param	beneathLimit	How far down an area can be to still be considered overlapping.
	 *	@return					The CNavArea directly below pos, or INVALID_NAV_AREA if no area can be found.
	 */
	public native CNavArea GetNavArea(const float pos[3], float beneathLimit=120.0);

	/**
	 *	Given a position in the world, return the nav area that is closest and at the same height, or beneath it.
	 *	Used to find initial area if we start off of the mesh.
	 *	
	 *	@param	pos				The position to check.
	 *	@param	nGetAreaFlags	Flags that filter area by attribute.
	 *	@param	maxDist			Maximum distance to search from pos.
	 *	@param	team			
	 *	@return					A CNavArea, or INVALID_NAV_AREA if no area can be found.
	 */
	public native CNavArea GetNearestNavArea(const float pos[3], GetNavAreaFlags_t nGetAreaFlags=GETNAVAREA_CHECK_GROUND, float maxDist=10000.0, int team=-2);

	/**
	 *	Given an id, returns the hiding spot associated with that id.
	 *
	 *	@param	id	The id of the spot to search for.
	 *	@return		A HidingSpot, or INVALID_NAV_HIDING_SPOT if no spot can be found.
	 */
	public native HidingSpot GetHidingSpotByID(int id);

	/**
	 *	Returns a random hiding spot in the nav mesh.
	 *
	 *	@return		A HidingSpot, or INVALID_NAV_HIDING_SPOT if no spot can be found.
	 */
	public native HidingSpot GetRandomHidingSpot();

	/**
	 *	Iterate through areas surrounding startArea.
	 *
	 *	@param	startArea			The starting area.
	 *	@param	startPos			The starting position.
	 *	@param	filterFunc			The function that (after checking if area's blocked/cost/height change) determines if search should 
	 *								continue outward from the current area.
	 *	@param	filterData			Optional. User-defined data to pass to filterFunc.
	 *	@param	options				Optional. Flags that change behavior of this function. See SearchSurroundingAreasFlags_t.
	 *	@param	maxRange			Optional. Maximum travel distance to search.
	 *	@param	maxStepSize			Optional. Maximum position height change allowed to next area. If 0.0, then no limit.
	 *	@param	maxDropDownLimit	Optional. Maximum negative height change allowed to next area. If 0.0, then no limit.
	 *	@param	usePortal			Optional. If true, will calculate travel cost using closest point in portal.
	 *								By default, distance is calculated from center of area (which may be problematic for large areas.)
	 */
	public native void SearchSurroundingAreas(CNavArea startArea, const float startPos[3], NavSearchSurroundingAreasFunctor filterFunc, any filterData=0, SearchSurroundingAreasFlags_t options=SEARCHSURROUNDAREAS_NONE, float maxRange=1500.0, float maxStepSize=StepHeight, float maxDropDownLimit=100.0, bool usePortal=false);

	/**
	 *	Collect areas surrounding startArea. This uses SearchSurroundingAreas() internally.
	 *
	 *	@param	areas				Buffer that receives the areas.
	 *	@param	startArea			The starting area.
	 *	@param	startPos			The starting position.
	 *	@param	options				Optional. Flags that change behavior of this function. See SearchSurroundingAreasFlags_t.
	 *	@param	maxRange			Optional. Maximum travel distance to search.
	 *	@param	maxStepSize			Optional. Maximum position height change allowed to next area. If 0.0, then no limit.
	 *	@param	maxDropDownLimit	Optional. Maximum negative height change allowed to next area. If 0.0, then no limit.
	 *	@param	usePortal			Optional. If true, will calculate travel cost using closest point in portal.
	 *								By default, distance is calculated from center of area (which may be problematic for large areas.)
	 */
	public native void CollectSurroundingAreas(ArrayList areas, CNavArea startArea, const float startPos[3], SearchSurroundingAreasFlags_t options=SEARCHSURROUNDAREAS_NONE, float maxRange=1500.0, float maxStepSize=StepHeight, float maxDropDownLimit=100.0, bool usePortal=false);

	/**
	 *	Iterate areas located in a radius around the given position.
	 *
	 *	@param	pos					The position to calculate from.
	 *	@param	radius				Maximum distance of search from position.
	 *	@param	functor				Callback function that is called for every area searched. If the functor returns false,
	 *								search will stop.
	 *	@param	functorData			Optional. User-defined data for functor.
	 *	@param	useClosestPoint		Optional. If true, will calculate distance using the closest point to pos on area.
	 *								By default, distance is calculated from center of area (which may be problematic for large areas.)
	 */
	public native void ForAllAreasInRadius(const float pos[3], float radius, NavAreaFunctor functor, any functorData=0, bool useClosestPoint=false)

	/**
	 *	Collect areas located in a radius around the given position. This uses ForAllAreasInRadius() internally.
	 *
	 *	@param	pos					The position to calculate from.
	 *	@param	radius				Maximum distance of search from position.
	 *	@param	buffer				ArrayList that will receive the areas.
	 *	@param	useClosestPoint		Optional. If true, will calculate distance using the closest point to pos on area.
	 *								By default, distance is calculated from center of area (which may be problematic for large areas.)
	 */
	public native void CollectAreasInRadius(const float pos[3], float radius, ArrayList buffer, bool useClosestPoint=false);

	/**
	 *	Find path from startArea to goalArea via an A* search, using supplied cost heuristic.
	 *	If cost functor returns -1 for an area, that area is considered a dead end.
	 *	This doesn't actually build a path, but the path is defined by following the Parent area back from goalArea 
	 *	to startArea.
	 *	
	 *	@param	startArea		The starting area.
	 *	@param	goalArea		The goal area.
	 *	@param	goalPos			The goal position on the goal area.
	 *	@param	costFunctor		A callback function that calculates the total cost of the current area.
	 *							If cost functor returns -1, that area is considered a dead end.
	 *	@param	costData		Optional. User-defined data to pass to costFunctor.
	 *	@param	closestArea		Optional. If path fails, contains the closest area to goal; if path success, the
	 *							goal area.
	 *	@param	maxPathLength	Optional. Maximum cost so far allowed on the path.
	 * 	@param	maxStepSize		Optional. Maximum connection height change between areas on the path.
	 *	@return					True if the built path reaches goalArea; false if it can't reach goalArea.
	 */
	public native bool BuildPath(CNavArea startArea, CNavArea goalArea, const float goalPos[3], NavPathCostFunctor costFunctor, any costData=0, CNavArea &closestArea=INVALID_NAV_AREA, float maxPathLength=0.0, float maxStepSize=0.0);
}

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
	
	/**
	 *	The id of the area.
	 */
	property int ID
	{
		public native get();
	}
	
	/**
	 *	The base attributes of the area.
	 */
	property int Attributes
	{
		public native get();
	}
	
	/**
	 *	The parent of the area. Used in A* pathfiding.
	 */
	property CNavArea Parent
	{
		public native get();
		public native set( CNavArea area );
	}
	
	/**
	 *	Method to reach the parent area. Used in A* pathfiding.
	 */
	property int ParentHow
	{
		public native get();
		public native set( int how );
	}
	
	/**
	 *	The total cost; that is, cost so far + heuristic. Used in A* pathfinding.
	 */
	property int TotalCost
	{
		public native get();
	}
	
	/**
	 *	The travel cost so far, or current progress. Used in A* pathfinding.
	 */
	property int CostSoFar
	{
		public native get();
	}
	
	/**
	 *	The Z component of this area's northeast corner.
	 */
	property float NECornerZ
	{
		public native get();
	}
	
	/**
	 *	The Z component of this area's southwest corner.
	 */
	property float SWCornerZ
	{
		public native get();
	}
	
	property CNavArea InheritVisibilityFrom
	{
		public native get();
	}

	/**
	 *	The place name of the area.
	 *
	 *	@param	buffer	The buffer to store the place name.
	 *	@param	maxlen	The max length of the buffer.
	 */
	public native void GetPlace( char[] buffer, int maxlen );
	
	/**
	 *	The center point of the area.
	 *
	 *	@param	buffer	The center point.
	 *	@return			True if successful, false otherwise.
	 */
	public native bool GetCenter( float buffer[3] );
	
	/**
	 *	The low corner of the area; lowest x,y components.
	 *
	 *	@param	buffer	
	 *	@return			True if successful, false otherwise.
	 */
	public native bool GetExtentLow( float buffer[3] );
	
	/**
	 *	The high corner of the area; highest x,y components.
	 *
	 *	@param	buffer	
	 *	@return			True if successful, false otherwise.
	 */
	public native bool GetExtentHigh( float buffer[3] );
	
	/**
	 *	Retrieves the corner point of the given corner type.
	 *
	 *	@param	corner	The corner type.
	 *	@param	buffer
	 */
	public native void GetCorner( NavCornerType corner, float buffer[3] );
	
	/**
	 *	Calculates the Z component of the given point on the area.
	 *
	 *	@param 	point	The point to calculate from.
	 *	@return			The Z component of the point on area.
	 */
	public native float GetZ( const float point[3] );
	
	/**
	 *	Calculates the Z component of the given x and y components on the area.
	 *
	 *	@param 	x
	 *	@param	y
	 *	@return		The Z component of the point on area.
	 */
	public native float GetZFromXAndY( float x, float y );
	
	/**
	 * Returns whether or not the area overlaps the given point. The Z-axis is not checked.
	 *
	 * @param	point		The point to check.
	 * @param	tolerance	How many units the point can be away from the edge of the area.
	 * @return				true if overlapping, false otherwise.
	 */
	public native bool IsOverlappingPoint( const float point[3], float tolerance=0.0 );
	
	/**
	 * Returns whether or not the area overlaps the given area. The Z-axis is not checked.
	 *
	 * @param	area	The area to check.
	 * @return			true if overlapping, false otherwise.
	 */
	public native bool IsOverlappingArea( CNavArea area );
	
	/**
	 * Returns whether or not the given point is contained by the area.
	 *
	 * Note: If the point overlaps the area, the given point can be at most StepHeight units above the 
	 * area to still be considered "contained".
	 *
	 * @param	point	The point to check.
	 * @return			true if the point is contained by the area, false otherwise.
	 */
	public native bool Contains( const float point[3] );
	
	/**
	 * Returns whether or not if there are no bi-directional links in the given direction of the area.
	 * 
	 * Note: For the longest time this only checked for presence of an outgoing connection, regardless
	 * if the connection was two-way or not. This has been recently changed to properly reflect
	 * CNavArea::IsEdge( NavDirType dir )'s functionality.
	 *
	 * @param	dir		Direction to check.
	 * @return			true if there are no bi-directional connections; false otherwise.
	 */
	public native bool IsEdge( int dir );
	
	public native void GetRandomPoint( float buffer[3] );
	
	/**
	 * Returns whether or not there exists an outgoing link to targetArea in the provided direction.
	 *
	 * @param	targetArea	The area to check for.
	 * @param	dir			The direction to check. If NAV_DIR_COUNT is the direction, all directions will be checked.
	 * @return				true if there exists a link, false otherwise.
	 */
	public native bool IsConnected( CNavArea targetArea, int dir );
	
	public native void GetClosestPointOnArea( const float pos[3], float buffer[3] );
	
	/**
	 * Retrieves all bi-directional and one-way OUTGOING connections from this area and stores CNavArea's in the
	 * provided buffer.
	 *
	 * Note: This does NOT include one-way INCOMING connections. Use GetIncomingConnections() for that.
	 *
	 * @param	dir		Direction to check.
	 * @param	buffer	An ArrayList to insert the CNavArea's in.
	 */
	public native void GetAdjacentAreas( int dir, ArrayList buffer );

	public CNavArea GetRandomAdjacentArea( int dir )
	{
		ArrayList list = new ArrayList();
		this.GetAdjacentAreas(dir, list);
		CNavArea area = list.Length > 0 ? CNavArea( list.Get( GetRandomInt(0, list.Length - 1) ) ) : INVALID_NAV_AREA;
		delete list;
		return area;
	}

	/**
	 * Returns the amount of bi-directional and one-way OUTGOING connections from this area in the provided
	 * direction.
	 *
	 * Note: This does NOT count one-way INCOMING connections.
	 *
	 * @param	dir		Direction to check. If NAV_DIR_COUNT is the direction, all directions will be checked.
	 * @return			The amount of connections.
	 */
	public int GetAdjacentCount( int dir )
	{
		int count = 0;
		ArrayList list = new ArrayList();
		this.GetAdjacentAreas(dir, list);
		count = list.Length;
		delete list;
		return count;
	}

	/**
	 * Retrieves all one-way INCOMING connections to this area and stores CNavArea's in the provided
	 * buffer.
	 *
	 * @param	dir		Direction to check. If NAV_DIR_COUNT is the direction, all directions will be checked.
	 * @param	buffer	An ArrayList to insert the CNavArea's in.
	 */
	public native void GetIncomingConnections( int dir, ArrayList buffer );
	
	/**
	 *	Gets all ladders connected to the area.
	 *
	 *	@param	dir		Ladder direction to check.
	 *	@param	buffer	Buffer that will receive the ladders.
	 */
	public native void GetLadders( int dir, ArrayList buffer );
	
	/**
	 *	Gets all hiding spots associated with the area.
	 *
	 *	@param	buffer	Buffer that will receive the hiding spots.
	 */
	public native void GetHidingSpots( ArrayList buffer );
	
	/**
	 *	Computes the "entryway" to the specified area and direction from this area. The given area does
	 *	not have to be connected.
	 *	
	 *	.----------------------.
	 *	|   |   |              |
	 *	|   |<- | portal       |
	 *	|   |   |              |
	 *	'---|                  |
	 *	    '------------------'
	 *
	 *	@param	to			The area to calculate the entryway to.
	 *	@param	dir			The direction to calculate from.
	 *	@param	center		The center of the portal.
	 *	@param	halfWidth	The half width of the portal.
	 */
	public native void ComputePortal( CNavArea to, int dir, float center[3], float &halfWidth );
	
	/**
	 *	Computes the portal to the specified area, and calculates the closest point to the given point
	 *	within the portal.
	 *
	 *	@param	to			The area to calculate the entryway to.
	 *	@param	dir			The direction to calcucate from.
	 *	@param	fromPos		The point to calculate from.
	 *	@param	closePos	The closest point to fromPos within the portal.
	 */
	public native void ComputeClosestPointInPortal( CNavArea to, int dir, const float fromPos[3], float closePos[3] );
	
	public native int ComputeDirection( const float point[3] );
	
	public native float GetLightIntensity( const float point[3] );
}

// CTFNavArea
// Bitflags for CTFNavArea's AttributeFlags property.
enum
{
	TF_NAV_INVALID						= 0x00000000,

	TF_NAV_BLOCKED						= 0x00000001,
	TF_NAV_SPAWN_ROOM_RED				= 0x00000002,
	TF_NAV_SPAWN_ROOM_BLUE				= 0x00000004,
	TF_NAV_SPAWN_ROOM_EXIT				= 0x00000008,
	TF_NAV_HAS_AMMO						= 0x00000010,
	TF_NAV_HAS_HEALTH					= 0x00000020,
	TF_NAV_CONTROL_POINT				= 0x00000040,

	TF_NAV_BLUE_SENTRY_DANGER			= 0x00000080,			// sentry can potentially fire upon enemies in this area
	TF_NAV_RED_SENTRY_DANGER			= 0x00000100,

	TF_NAV_BLUE_SETUP_GATE				= 0x00000800,			// this area is blocked until the setup period is over
	TF_NAV_RED_SETUP_GATE				= 0x00001000,			// this area is blocked until the setup period is over
	TF_NAV_BLOCKED_AFTER_POINT_CAPTURE	= 0x00002000,			// this area becomes blocked after the first point is capped
	TF_NAV_BLOCKED_UNTIL_POINT_CAPTURE  = 0x00004000,			// this area is blocked until the first point is capped, then is unblocked
	TF_NAV_BLUE_ONE_WAY_DOOR			= 0x00008000,
	TF_NAV_RED_ONE_WAY_DOOR				= 0x00010000,

 	TF_NAV_WITH_SECOND_POINT			= 0x00020000,
 	TF_NAV_WITH_THIRD_POINT				= 0x00040000,
  	TF_NAV_WITH_FOURTH_POINT			= 0x00080000,
 	TF_NAV_WITH_FIFTH_POINT				= 0x00100000,

	TF_NAV_SNIPER_SPOT					= 0x00200000,			// this is a good place for a sniper to lurk
	TF_NAV_SENTRY_SPOT					= 0x00400000,			// this is a good place to build a sentry

	TF_NAV_ESCAPE_ROUTE					= 0x00800000,			// for Raid mode
	TF_NAV_ESCAPE_ROUTE_VISIBLE			= 0x01000000,			// all areas that have visibility to the escape route

	TF_NAV_NO_SPAWNING					= 0x02000000,			// don't spawn bots in this area

 	TF_NAV_RESCUE_CLOSET				= 0x04000000,			// for respawning friends in Raid mode

 	TF_NAV_BOMB_CAN_DROP_HERE			= 0x08000000,			// the bomb can be dropped here and reached by the invaders in MvM

	TF_NAV_DOOR_NEVER_BLOCKS			= 0x10000000,
	TF_NAV_DOOR_ALWAYS_BLOCKS			= 0x20000000,

	TF_NAV_UNBLOCKABLE					= 0x40000000,			// this area cannot be blocked

	// save/load these manually set flags, and don't clear them between rounds
	TF_NAV_PERSISTENT_ATTRIBUTES		= TF_NAV_SNIPER_SPOT | TF_NAV_SENTRY_SPOT | TF_NAV_NO_SPAWNING | TF_NAV_BLUE_SETUP_GATE | TF_NAV_RED_SETUP_GATE | TF_NAV_BLOCKED_AFTER_POINT_CAPTURE | TF_NAV_BLOCKED_UNTIL_POINT_CAPTURE | TF_NAV_BLUE_ONE_WAY_DOOR | TF_NAV_RED_ONE_WAY_DOOR | TF_NAV_DOOR_NEVER_BLOCKS | TF_NAV_DOOR_ALWAYS_BLOCKS | TF_NAV_UNBLOCKABLE | TF_NAV_WITH_SECOND_POINT | TF_NAV_WITH_THIRD_POINT | TF_NAV_WITH_FOURTH_POINT | TF_NAV_WITH_FIFTH_POINT | TF_NAV_RESCUE_CLOSET
};

methodmap CTFNavArea < CNavArea
{
	property int AttributeFlags
	{
		public native get();
	}
}

// CCSNavArea
enum struct CCSNavArea_ApproachInfo
{
	int HereArea;
	int PrevArea;
	int PrevToHereHow;
	int NextArea;
	int HereToNextHow;
}

methodmap CCSNavArea < CNavArea
{
	public native void GetApproachInfoList( ArrayList buffer );
}

// TerrorNavArea
// Bitflags for TerrorNavArea.SpawnAttributes
enum
{
	TERROR_NAV_EMPTY = 2,
	TERROR_NAV_STOP = 4,
	TERROR_NAV_FINALE = 0x40,
	TERROR_NAV_BATTLEFIELD = 0x100,
	TERROR_NAV_PLAYER_START = 0x80,
	TERROR_NAV_IGNORE_VISIBILITY = 0x200,
	TERROR_NAV_NOT_CLEARABLE = 0x400,
	TERROR_NAV_CHECKPOINT = 0x800,
	TERROR_NAV_OBSCURED = 0x1000,
	TERROR_NAV_NO_MOBS = 0x2000,
	TERROR_NAV_THREAT = 0x4000,
	TERROR_NAV_NOTHREAT = 0x80000,
	TERROR_NAV_LYINGDOWN = 0x100000,
	TERROR_NAV_RESCUE_CLOSET = 0x10000,
	TERROR_NAV_RESCUE_VEHICLE = 0x8000
}

methodmap TerrorNavArea < CNavArea
{
	/**
	 *	Attributes that define the Director's spawning behavior in this area.
	 */
	property int SpawnAttributes
	{
		public native get();
	}
}

methodmap TerrorNavMesh < CNavMesh
{
	property float NavMaxViewDistance
	{
		public native get();
	}

	/**
	 *	Returns the type of zombies to be spawned in the map; the default population string.
	 *	See here for more info: https://developer.valvesoftware.com/wiki/Left_4_Dead_Infected_Populations
	 *
	 *	@param	buffer		
	 * 	@param	maxSize		Size of buffer
	 */
	public native void GetZombiePopulation(char[] buffer, int maxSize);
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

#pragma deprecated Use CNavMesh.Exists instead.
native bool NavMesh_Exists();

#pragma deprecated Use CNavMesh.MagicNumber instead.
native int NavMesh_GetMagicNumber();

#pragma deprecated Use CNavMesh.Version instead.
native int NavMesh_GetVersion();

#pragma deprecated Use CNavMesh.Subversion instead.
native int NavMesh_GetSubVersion();

#pragma deprecated Use CNavMesh.SaveBSPSize instead.
native int NavMesh_GetSaveBSPSize();

#pragma deprecated Use CNavMesh.Analyzed instead.
native bool NavMesh_IsAnalyzed();

#pragma deprecated Use CNavMesh.GetGroundHeight() instead.
native bool NavMesh_GetGroundHeight(const float pos[3], float &height=-1.0, float normal[3]=NULL_VECTOR);

#pragma deprecated Use CNavMesh.GetNavAreaByID() instead.
native CNavArea NavMesh_FindAreaByID( int areaID );

#pragma deprecated Use CNavMesh.GetNavArea() instead.
native CNavArea NavMesh_GetArea(const float pos[3], float beneathLimit=120.0);

#pragma deprecated Use CNavMesh.GetNearestNavArea() instead.
native CNavArea NavMesh_GetNearestArea(const float pos[3], bool anyZ=false, float maxDist=10000.0, bool checkLOS=false, bool checkGround=true, int team=-2);

#pragma deprecated Use CNavMesh.GetHidingSpotByID() instead.
native HidingSpot NavMesh_FindHidingSpotByID(int hidingSpotID);

#pragma deprecated Use CNavMesh.GetRandomHidingSpot() instead.
native HidingSpot NavMesh_GetRandomHidingSpot();

#pragma deprecated This function will be removed in the future.
native int NavMesh_WorldToGridX(float wx);

#pragma deprecated This function will be removed in the future.
native int NavMesh_WorldToGridY(float wy);

#pragma deprecated This function will be removed in the future.
native void NavMesh_GetAreasOnGrid(ArrayStack hStack, int x, int y);

#pragma deprecated This function will be removed in the future.
native int NavMesh_GetGridSizeX();

#pragma deprecated This function will be removed in the future.
native int NavMesh_GetGridSizeY();

#pragma deprecated Use CNavMesh.CollectSurroundingAreas() instead.
native void NavMesh_CollectSurroundingAreas(ArrayStack hStack, CNavArea startArea, float travelDistLimit=1500.0, float maxStepUpLimit=StepHeight, float maxDropDownLimit=100.0);

#pragma deprecated Use CNavMesh.BuildPath() instead.
native bool NavMesh_BuildPath(CNavArea startArea, CNavArea goalArea, const float goalPos[3], NavPathCostFunctor costFunctor, any costData=0, CNavArea &closestArea=INVALID_NAV_AREA, float maxPathLength=0.0, float flMaxStepSize=0.0);

#pragma deprecated Use CNavArea.ID instead.
native int NavMeshArea_GetID(int areaIndex);

#pragma deprecated Use CNavArea.Attributes instead.
native int NavMeshArea_GetFlags(int areaIndex);

#pragma deprecated Use CNavArea.GetPlace() instead.
native void NavMeshArea_GetPlace(int areaIndex, char[] buffer, int maxlen);

#pragma deprecated Use CNavArea.GetCenter() instead.
native bool NavMeshArea_GetCenter(int areaIndex, float buffer[3]);

#pragma deprecated Use CNavArea.GetLadders() instead.
native void NavMeshArea_GetLadderList(ArrayStack buffer, int areaIndex, int dir);

#pragma deprecated Use CNavArea.GetHidingSpots() instead.
native void NavMeshArea_GetHidingSpots(ArrayStack buffer, int areaIndex);

#pragma deprecated Use CNavArea.GetClosestPointOnArea() instead.
native void NavMeshArea_GetClosestPointOnArea(int areaIndex, const float pos[3], float closePos[3]);

#pragma deprecated Use CNavArea.TotalCost instead.
native int NavMeshArea_GetTotalCost(int areaIndex);

#pragma deprecated Use CNavArea.CostSoFar instead.
native int NavMeshArea_GetCostSoFar(int areaIndex);

#pragma deprecated Use CNavArea.Parent instead.
native int NavMeshArea_GetParent(int areaIndex);

#pragma deprecated Use CNavArea.ParentHow instead.
native int NavMeshArea_GetParentHow(int areaIndex);

#pragma deprecated Use CNavArea.Parent instead.
native void NavMeshArea_SetParent(int areaIndex, int parentAreaIndex);

#pragma deprecated Use CNavArea.ParentHow instead.
native void NavMeshArea_SetParentHow(int areaIndex, int parentHow);

#pragma deprecated Use CNavArea.GetExtentLow() instead.
native bool NavMeshArea_GetExtentLow(int areaIndex, float buffer[3]);

#pragma deprecated Use CNavArea.GetExtentHigh() instead.
native bool NavMeshArea_GetExtentHigh(int areaIndex, float buffer[3]);

#pragma deprecated Use CNavArea.IsOverlappingPoint() instead.
native bool NavMeshArea_IsOverlappingPoint(int areaIndex, const float pos[3], float tolerance);

#pragma deprecated Use CNavArea.IsOverlappingArea() instead.
native bool NavMeshArea_IsOverlappingArea(int areaIndex, int targetAreaIndex);

#pragma deprecated Use CNavArea.NECornerZ instead.
native float NavMeshArea_GetNECornerZ(int areaIndex);

#pragma deprecated Use CNavArea.SWCornerZ instead.
native float NavMeshArea_GetSWCornerZ(int areaIndex);

#pragma deprecated Use CNavArea.GetCorner() instead.
native void NavMeshArea_GetCorner(int areaIndex, NavCornerType corner, float buffer[3]);

#pragma deprecated Use CNavArea.GetZ() instead.
native float NavMeshArea_GetZ(int areaIndex, const float pos[3]);

#pragma deprecated Use CNavArea.GetZFromXAndY() instead.
native float NavMeshArea_GetZFromXAndY(int areaIndex, float x, float y);

#pragma deprecated Use CNavArea.IsEdge() instead.
native bool NavMeshArea_IsEdge(int areaIndex, int dir);

#pragma deprecated Use CNavArea.Contains() instead.
native bool NavMeshArea_Contains(int areaIndex, const float pos[3]);

#pragma deprecated Use CNavArea.GetRandomPoint() instead.
native void NavMeshArea_GetRandomPoint(int areaIndex, float buffer[3]);

#pragma deprecated Use CNavArea.IsConnected() instead.
native bool NavMeshArea_IsConnected(int areaIndex, int targetAreaIndex, int dir);

#pragma deprecated Use CNavArea.ComputePortal() instead.
native bool NavMeshArea_ComputePortal(int areaIndex, int areaToIndex, int dir, float center[3], float &halfWidth);

#pragma deprecated Use CNavArea.ComputeClosestPointInPortal() instead.
native bool NavMeshArea_ComputeClosestPointInPortal(int areaIndex, int areaToIndex, int dir, const float fromPos[3], float closePos[3]);

#pragma deprecated Use CNavArea.ComputeDirection() instead.
native int NavMeshArea_ComputeDirection(int areaIndex, const float pos[3]);

#pragma deprecated Use CNavArea.GetLightIntensity() instead.
native float NavMeshArea_GetLightIntensity(int areaIndex, const float pos[3]);

#pragma deprecated Use CNavArea.GetAdjacentAreas() instead.
native void NavMeshArea_GetAdjacentList(ArrayStack buffer, int areaIndex, int dir);

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