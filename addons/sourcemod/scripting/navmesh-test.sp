#include <sourcemod>
#include <sdktools>
#include <profiler>

#pragma newdecls required
#include <navmesh>

#define PLUGIN_VERSION "1.0.6"

int g_iPathLaserModelIndex = -1;

public Plugin myinfo = 
{
    name = "SourcePawn Navigation Mesh Test",
    author	= "KitRifty, Benoist3012",
    description	= "Testing plugin of the Navigation Mesh plugin.",
    version = PLUGIN_VERSION,
    url = ""
}

float g_flTrackNavAreaThinkRate = 0.1;
float g_flTrackNavAreaNextThink = 0.0;

bool g_bPlayerTrackNavArea[MAXPLAYERS + 1] = { false, ... };
Handle g_hPlayerTrackNavAreaInfoHudSync = null;

public void OnPluginStart()
{
	RegAdminCmd("sm_navmesh_collectsurroundingareas", Command_NavMeshCollectSurroundingAreas, ADMFLAG_CHEATS, "");
	RegAdminCmd("sm_navmesh_buildpath", Command_NavMeshBuildPath, ADMFLAG_CHEATS, "");
	RegAdminCmd("sm_navmesh_getnearestarea", Command_GetNearestArea, ADMFLAG_CHEATS, "");
	RegAdminCmd("sm_navmesh_show", Command_Show, ADMFLAG_CHEATS, "");

	g_hPlayerTrackNavAreaInfoHudSync = CreateHudSynchronizer();
}

public void OnMapStart()
{
	g_iPathLaserModelIndex = PrecacheModel("materials/sprites/laserbeam.vmt");

	g_flTrackNavAreaNextThink = 0.0;
}

void DrawNavArea( int client, CNavArea area, const int color[4], float duration=0.15 ) 
{
	if ( !IsClientInGame(client) || area == INVALID_NAV_AREA )
		return;

	float from[3], to[3];
	area.GetCorner( NAV_CORNER_NORTH_WEST, from );
	area.GetCorner( NAV_CORNER_NORTH_EAST, to );
	from[2] += 2; to[2] += 2;

	TE_SetupBeamPoints(from, to, g_iPathLaserModelIndex, g_iPathLaserModelIndex, 0, 30, duration, 1.0, 1.0, 0, 0.0, color, 1);
	TE_SendToClient(client);

	area.GetCorner( NAV_CORNER_NORTH_EAST, from );
	area.GetCorner( NAV_CORNER_SOUTH_EAST, to );
	from[2] += 2; to[2] += 2;

	TE_SetupBeamPoints(from, to, g_iPathLaserModelIndex, g_iPathLaserModelIndex, 0, 30, duration, 1.0, 1.0, 0, 0.0, color, 1);
	TE_SendToClient(client);

	area.GetCorner( NAV_CORNER_SOUTH_EAST, from );
	area.GetCorner( NAV_CORNER_SOUTH_WEST, to );
	from[2] += 2; to[2] += 2;

	TE_SetupBeamPoints(from, to, g_iPathLaserModelIndex, g_iPathLaserModelIndex, 0, 30, duration, 1.0, 1.0, 0, 0.0, color, 1);
	TE_SendToClient(client);

	area.GetCorner( NAV_CORNER_SOUTH_WEST, from );
	area.GetCorner( NAV_CORNER_NORTH_WEST, to );
	from[2] += 2; to[2] += 2;

	TE_SetupBeamPoints(from, to, g_iPathLaserModelIndex, g_iPathLaserModelIndex, 0, 30, duration, 1.0, 1.0, 0, 0.0, color, 1);
	TE_SendToClient(client);
}

public void OnGameFrame()
{
	EngineVersion engineVersion = GetEngineVersion();

	if ( GetGameTime() >= g_flTrackNavAreaNextThink )
	{
		g_flTrackNavAreaNextThink = GetGameTime() + g_flTrackNavAreaThinkRate;

		for (int client = 1; client <= MaxClients; client++)
		{
			static const int DefaultAreaColor[] = { 255, 0, 0, 255 };
			static const int FocusedAreaColor[] = { 255, 255, 0, 255 };

			if (!IsClientInGame(client))
				continue;
			
			if ( g_bPlayerTrackNavArea[client] )
			{
				float flEyePos[3], flEyeDir[3], flEndPos[3];
				GetClientEyePosition(client, flEyePos);
				GetClientEyeAngles(client, flEyeDir);
				GetAngleVectors(flEyeDir, flEyeDir, NULL_VECTOR, NULL_VECTOR);
				NormalizeVector(flEyeDir, flEyeDir);
				ScaleVector(flEyeDir, 1000.0);
				AddVectors(flEyePos, flEyeDir, flEndPos);
				
				Handle hTrace = TR_TraceRayFilterEx(flEyePos,
					flEndPos,
					MASK_PLAYERSOLID_BRUSHONLY,
					RayType_EndPoint,
					TraceRayDontHitEntity,
					client);
				
				TR_GetEndPosition(flEndPos, hTrace);
				delete hTrace;

				CNavArea area = NavMesh_GetNearestArea(flEndPos);
				if (area == INVALID_NAV_AREA)
					continue;
				
				DrawNavArea( client, area, FocusedAreaColor );

				ArrayList connections = new ArrayList();
				area.GetAdjacentList(NAV_DIR_COUNT, connections);
				ArrayList incomingConnections = new ArrayList();
				area.GetIncomingConnections(NAV_DIR_COUNT, incomingConnections);

				for (int i = 0; i < connections.Length; i++)
				{
					DrawNavArea(client, connections.Get(i), DefaultAreaColor);	
				}

				for (int i = 0; i < incomingConnections.Length; i++)
				{
				}

				switch (engineVersion)
				{
					case Engine_Left4Dead2:
					{
						PrintHintText(client, "ID: %d, # Connections: %d, # Incoming: %d", area.ID, connections.Length, incomingConnections.Length);
					}
					default:
					{
						SetHudTextParams(-1.0, 0.75, 0.2, 255, 255, 0, 150, 0, 0.0, 0.0, 0.0);
						ShowSyncHudText(client, g_hPlayerTrackNavAreaInfoHudSync, "ID: %d\n# Connections: %d\n# Incoming: %d\n", area.ID, connections.Length, incomingConnections.Length);
					}
				}

				delete connections;
				delete incomingConnections;
			}
		}
	}
}

public void OnClientDisconnect(int client)
{
	g_bPlayerTrackNavArea[client] = false;
}

public Action Command_Show(int client,int args)
{
	if (!NavMesh_Exists()) return Plugin_Handled;

	if ( args < 1 ) 
	{
		ReplyToCommand(client, "Usage: sm_navmesh_show <0/1>");
		return Plugin_Handled;
	}

	char sArg[16];
	GetCmdArg(1, sArg, sizeof(sArg));
	g_bPlayerTrackNavArea[client] = (StringToInt(sArg) != 0);

	return Plugin_Handled;
}

public Action Command_GetNearestArea(int client,int args)
{
	if (!NavMesh_Exists()) return Plugin_Handled;
	float flEyePos[3], flEyeDir[3], flEndPos[3];
	GetClientEyePosition(client, flEyePos);
	GetClientEyeAngles(client, flEyeDir);
	GetAngleVectors(flEyeDir, flEyeDir, NULL_VECTOR, NULL_VECTOR);
	NormalizeVector(flEyeDir, flEyeDir);
	ScaleVector(flEyeDir, 1000.0);
	AddVectors(flEyePos, flEyeDir, flEndPos);
	
	Handle hTrace = TR_TraceRayFilterEx(flEyePos,
		flEndPos,
		MASK_PLAYERSOLID_BRUSHONLY,
		RayType_EndPoint,
		TraceRayDontHitEntity,
		client);
	
	TR_GetEndPosition(flEndPos, hTrace);
	CloseHandle(hTrace);
	
	CNavArea area = NavMesh_GetNearestArea(flEndPos);
	if (area != INVALID_NAV_AREA)
	{
		PrintToChat(client, "Nearest area ID: %d", area.ID);
	}
	else
	{
		PrintToChat(client, "Could not find an area.");
	}
	
	return Plugin_Handled;
}

public Action Command_NavMeshCollectSurroundingAreas(int client,int args)
{
	return Plugin_Handled;
}

public Action Command_NavMeshBuildPath(int client,int args)
{
	if (args < 2)
	{
		ReplyToCommand(client, "Usage: sm_navmesh_buildpath <start area ID> <goal area ID>");
		return Plugin_Handled;
	}
	
	if (!NavMesh_Exists()) return Plugin_Handled;
	
	char sStartAreaID[64], sGoalAreaID[64];
	GetCmdArg(1, sStartAreaID, sizeof(sStartAreaID));
	GetCmdArg(2, sGoalAreaID, sizeof(sGoalAreaID));
	
	CNavArea startArea =  NavMesh_FindAreaByID(StringToInt(sStartAreaID));
	CNavArea goalArea = NavMesh_FindAreaByID(StringToInt(sGoalAreaID));
	
	if (startArea == INVALID_NAV_AREA || goalArea == INVALID_NAV_AREA) return Plugin_Handled;
	
	float flGoalPos[3];
	goalArea.GetCenter(flGoalPos);

	int iColor[4] = { 0, 255, 0, 255 };
	
	float flMaxPathLength = 0.0;
	if (args > 2)
	{
		char sMaxPathLength[64];
		GetCmdArg(3, sMaxPathLength, sizeof(sMaxPathLength));
		flMaxPathLength = StringToFloat(sMaxPathLength);
		
		if (flMaxPathLength < 0.0) return Plugin_Handled;
	}
	
	float flMaxStepSize = 0.0;
	if (args > 3)
	{
		char sMaxStepSize[64];
		GetCmdArg(4, sMaxStepSize, sizeof(sMaxStepSize));
		flMaxStepSize = StringToFloat(sMaxStepSize);
	}
	
	CNavArea closestArea = INVALID_NAV_AREA;
	
	Profiler profiler = CreateProfiler();
	profiler.Start();
	
	bool bBuiltPath = NavMesh_BuildPath(startArea, goalArea, flGoalPos, NavMeshShortestPathCost, _, closestArea, flMaxPathLength, flMaxStepSize);
	
	profiler.Stop();
	float flProfileTime = profiler.Time;
	delete profiler;

	if (client > 0) 
	{
		PrintToChat(client, "Build path time: %f\nReached goal: %d", flProfileTime, bBuiltPath);
		
		CNavArea tempArea = closestArea;
		CNavArea parentArea = tempArea.Parent;

		int iNavDirection;
		float flHalfWidth;
		float flCenterPortal[3], flClosestPoint[3];
		
		ArrayList positions = new ArrayList(3);
		positions.PushArray(flGoalPos, 3);

		while (parentArea != INVALID_NAV_AREA)
		{
			float flTempAreaCenter[3], flParentAreaCenter[3];
			tempArea.GetCenter(flTempAreaCenter);
			parentArea.GetCenter(flParentAreaCenter);

			iNavDirection = tempArea.ComputeDirection(flParentAreaCenter);
			tempArea.ComputePortal(parentArea, iNavDirection, flCenterPortal, flHalfWidth);
			tempArea.ComputeClosestPointInPortal(parentArea, iNavDirection, flCenterPortal, flClosestPoint);

			flClosestPoint[2] = tempArea.GetZ(flClosestPoint);
			positions.PushArray(flClosestPoint, 3);

			tempArea = parentArea;
			parentArea = tempArea.Parent;
		}
		
		float flStartPos[3];
		startArea.GetCenter(flStartPos);
		positions.PushArray(flStartPos, 3);

		for (int i = positions.Length - 1; i > 0; i--)
		{
			float flFromPos[3], flToPos[3];
			positions.GetArray(i, flFromPos, 3);
			positions.GetArray(i - 1, flToPos, 3);

			TE_SetupBeamPoints(flFromPos, flToPos, g_iPathLaserModelIndex, g_iPathLaserModelIndex, 0, 30, 5.0, 5.0, 5.0, 5,  0.0, iColor, 30);
			TE_SendToClient(client);
		}
	}
	else 
	{
		PrintToServer("Path built!\nBuild path time: %f\nReached goal: %d", flProfileTime, bBuiltPath);
	}
	
	return Plugin_Handled;
}

public bool TraceRayDontHitEntity(int entity,int mask, any data)
{
	if (entity == data) return false;
	return true;
}