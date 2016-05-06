#include <sourcemod>
#include <sdktools>
#include <profiler>

#pragma newdecls required
#include <navmesh>

#define PLUGIN_VERSION "1.0.2"

int g_iPathLaserModelIndex = -1;

public Plugin myinfo = 
{
    name = "SP-Readable Navigation Mesh Test",
    author	= "KitRifty, Benoist3012",
    description	= "Testing plugin of the SP-Readable Navigation Mesh plugin.",
    version = PLUGIN_VERSION,
    url = ""
}

public void OnPluginStart()
{
	RegConsoleCmd("sm_navmesh_collectsurroundingareas", Command_NavMeshCollectSurroundingAreas);
	RegConsoleCmd("sm_navmesh_buildpath", Command_NavMeshBuildPath);
	RegConsoleCmd("sm_navmesh_worldtogridx", Command_NavMeshWorldToGridX);
	RegConsoleCmd("sm_navmesh_worldtogridy", Command_NavMeshWorldToGridY);
	RegConsoleCmd("sm_navmesh_getareasongrid", Command_GetNavAreasOnGrid);
	RegConsoleCmd("sm_navmesh_getareanavindex", Command_GetArea);
	RegConsoleCmd("sm_navmesh_getnearestarea", Command_GetNearestArea);
	RegConsoleCmd("sm_navmesh_getadjacentareas", Command_GetAdjacentNavAreas);
}

public void OnMapStart()
{
	g_iPathLaserModelIndex = PrecacheModel("materials/sprites/laserbeam.vmt");
}

public Action Command_GetArea(int client,int args)
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
	
	int iAreaID = NavMesh_GetNearestArea(flEndPos);
	
	PrintToChat(client, "Nearest area ID: %d", iAreaID);
	
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
	
	int x = NavMesh_WorldToGridX(flEndPos[0]);
	int y = NavMesh_WorldToGridY(flEndPos[1]);
	int iGridIndex = x + y * NavMesh_GetGridSizeX();
	
	int iAreaIndex = NavMesh_GetNearestArea(flEndPos);
	if (iAreaIndex != -1)
	{
		Handle hAreas = NavMesh_GetAreas();
		int iAreaID = GetArrayCell(hAreas, iAreaIndex, NavMeshArea_ID);
		
		PrintToChat(client, "Nearest area ID found from spiral out of %d: %d", iGridIndex, iAreaID);
	}
	else
	{
		PrintToChat(client, "Could not find nearest area in spiral out of %d!", iGridIndex);
	}
	
	return Plugin_Handled;
}

public Action Command_GetAdjacentNavAreas(int client,int args)
{
	if (!NavMesh_Exists()) return Plugin_Handled;
	
	if (args < 2)
	{
		ReplyToCommand(client, "Usage: sm_navmesh_getadjacentareas <area ID>");
		return Plugin_Handled;
	}
	
	Handle hAreas = NavMesh_GetAreas();
	if (hAreas == INVALID_HANDLE) return Plugin_Handled;
	
	char sAreaID[64];
	GetCmdArg(1, sAreaID, sizeof(sAreaID));
	
	int iAreaID = StringToInt(sAreaID);
	
	int iStartAreaIndex = FindValueInArray(hAreas, iAreaID);
	if (iStartAreaIndex == -1) return Plugin_Handled;
	
	char sNavDirection[64];
	GetCmdArg(2, sNavDirection, sizeof(sNavDirection));
	
	int iNavDirection = StringToInt(sNavDirection);
	if (iNavDirection >= NAV_DIR_COUNT)
	{
		ReplyToCommand(client, "Invalid direction! Direction cannot reach %d!", NAV_DIR_COUNT);
		return Plugin_Handled;
	}
	
	Handle hAdjacentAreas = CreateStack();
	NavMeshArea_GetAdjacentList(hAdjacentAreas, iStartAreaIndex, iNavDirection);
	
	if (!IsStackEmpty(hAdjacentAreas))
	{
		while (!IsStackEmpty(hAdjacentAreas))
		{
			int iAreaIndex = -1;
			PopStackCell(hAdjacentAreas, iAreaIndex);
			PrintToChat(client, "Found adjacent area (ID: %d) for area ID %d", GetArrayCell(hAreas, iAreaIndex), iAreaID);
		}
		
		CloseHandle(hAdjacentAreas);
	}
	else
	{
		PrintToChat(client, "Found no adjacent areas for area ID %d", iAreaID);
	}
	
	CloseHandle(hAdjacentAreas);
	
	return Plugin_Handled;
}

public Action Command_NavMeshCollectSurroundingAreas(int client,int args)
{
	if (args < 2)
	{
		ReplyToCommand(client, "Usage: sm_navmesh_collectsurroundingareas <area ID> <max dist>");
		return Plugin_Handled;
	}
	
	if (!NavMesh_Exists()) return Plugin_Handled;
	
	Handle hAreas = NavMesh_GetAreas();
	if (hAreas == INVALID_HANDLE) return Plugin_Handled;
	
	char sAreaID[64];
	GetCmdArg(1, sAreaID, sizeof(sAreaID));
	
	int iAreaIndex = FindValueInArray(hAreas, StringToInt(sAreaID));
	
	if (iAreaIndex == -1 || iAreaIndex == -1) return Plugin_Handled;
	
	char sMaxDist[64];
	GetCmdArg(2, sMaxDist, sizeof(sMaxDist));
	
	float flMaxDist = StringToFloat(sMaxDist);
	
	Handle hProfiler = CreateProfiler();
	StartProfiling(hProfiler);
	
	Handle hNearAreas = CreateStack();
	NavMesh_CollectSurroundingAreas(hNearAreas, iAreaIndex, flMaxDist);
	
	StopProfiling(hProfiler);
	
	float flProfileTime = GetProfilerTime(hProfiler);
	
	CloseHandle(hProfiler);
	
	if (!IsStackEmpty(hNearAreas))
	{
		int iAreaCount;
		while (!IsStackEmpty(hNearAreas))
		{
			int iSomething;
			PopStackCell(hNearAreas, iSomething);
			iAreaCount++;
		}
		
		if (client > 0) 
		{
			PrintToChat(client, "Collected %d areas in %f seconds.", iAreaCount, flProfileTime);
		}
		else
		{
			PrintToServer("Collected %d areas in %f seconds.", iAreaCount, flProfileTime);
		}
	}
	
	CloseHandle(hNearAreas);
	
	return Plugin_Handled;
}

public Action Command_NavMeshWorldToGridX(int client,int args)
{
	if (args < 1) return Plugin_Handled;
	
	char arg1[32];
	GetCmdArg(1, arg1, sizeof(arg1));
	
	float flpl = StringToFloat(arg1);
	
	ReplyToCommand(client, "Grid x: %d", NavMesh_WorldToGridX(flpl));
	
	return Plugin_Handled;
}

public Action Command_NavMeshWorldToGridY(int client,int args)
{
	if (args < 1) return Plugin_Handled;
	
	char arg1[32];
	GetCmdArg(1, arg1, sizeof(arg1));
	
	float flpl = StringToFloat(arg1);
	
	ReplyToCommand(client, "Grid y: %d", NavMesh_WorldToGridY(flpl));
	
	return Plugin_Handled;
}

public Action Command_GetNavAreasOnGrid(int client,int args)
{
	if (args < 2) return Plugin_Handled;
	
	char arg1[32];
	GetCmdArg(1, arg1, sizeof(arg1));
	
	int x = StringToInt(arg1);
	
	char arg2[32];
	GetCmdArg(2, arg2, sizeof(arg2));
	
	int y = StringToInt(arg2);
	
	Handle hAreas = CreateStack();
	NavMesh_GetAreasOnGrid(hAreas, x, y);
	
	if (!IsStackEmpty(hAreas))
	{
		while (!IsStackEmpty(hAreas))
		{
			int iAreaIndex = -1;
			PopStackCell(hAreas, iAreaIndex);
			
			ReplyToCommand(client, "%d", iAreaIndex);
		}
	}
	
	CloseHandle(hAreas);
	
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
	
	Handle hAreas = NavMesh_GetAreas();
	if (hAreas == INVALID_HANDLE) return Plugin_Handled;
	
	char sStartAreaID[64], sGoalAreaID[64];
	GetCmdArg(1, sStartAreaID, sizeof(sStartAreaID));
	GetCmdArg(2, sGoalAreaID, sizeof(sGoalAreaID));
	
	int iStartAreaIndex = FindValueInArray(hAreas, StringToInt(sStartAreaID));
	int iGoalAreaIndex = FindValueInArray(hAreas, StringToInt(sGoalAreaID));
	
	if (iStartAreaIndex == -1 || iGoalAreaIndex == -1) return Plugin_Handled;
	
	float flGoalPos[3];
	NavMeshArea_GetCenter(iGoalAreaIndex, flGoalPos);
	
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
	char sMaxStepSize[64];
	GetCmdArg(4, sMaxStepSize, sizeof(sMaxStepSize));
	flMaxStepSize = StringToFloat(sMaxStepSize);
	
	int iClosestAreaIndex = 0;
	
	Handle hProfiler = CreateProfiler();
	StartProfiling(hProfiler);
	
	bool bBuiltPath = NavMesh_BuildPath(iStartAreaIndex, 
		iGoalAreaIndex,
		flGoalPos,
		NavMeshShortestPathCost,
		_,
		iClosestAreaIndex,
		flMaxPathLength,
		flMaxStepSize);
	
	StopProfiling(hProfiler);
	
	float flProfileTime = GetProfilerTime(hProfiler);
	
	CloseHandle(hProfiler);
	
	if (client > 0) 
	{
		PrintToChat(client, "Path built!\nBuild path time: %f\nReached goal: %d", flProfileTime, bBuiltPath);
		
		int iTempAreaIndex = iClosestAreaIndex;
		int iParentAreaIndex = NavMeshArea_GetParent(iTempAreaIndex);
		int iNavDirection;
		float flHalfWidth;
		
		float flCenterPortal[3], flClosestPoint[3];
		
		Handle hPositions = CreateArray(3);
		
		PushArrayArray(hPositions, flGoalPos, 3);
		
		while (iParentAreaIndex != -1)
		{
			float flTempAreaCenter[3], flParentAreaCenter[3];
			NavMeshArea_GetCenter(iTempAreaIndex, flTempAreaCenter);
			NavMeshArea_GetCenter(iParentAreaIndex, flParentAreaCenter);
			
			iNavDirection = NavMeshArea_ComputeDirection(iTempAreaIndex, flParentAreaCenter);
			NavMeshArea_ComputePortal(iTempAreaIndex, iParentAreaIndex, iNavDirection, flCenterPortal, flHalfWidth);
			NavMeshArea_ComputeClosestPointInPortal(iTempAreaIndex, iParentAreaIndex, iNavDirection, flCenterPortal, flClosestPoint);
			
			flClosestPoint[2] = NavMeshArea_GetZ(iTempAreaIndex, flClosestPoint);
			
			PushArrayArray(hPositions, flClosestPoint, 3);
			
			iTempAreaIndex = iParentAreaIndex;
			iParentAreaIndex = NavMeshArea_GetParent(iTempAreaIndex);
		}
		
		float flStartPos[3];
		NavMeshArea_GetCenter(iStartAreaIndex, flStartPos);
		PushArrayArray(hPositions, flStartPos, 3);
		
		for (int i = GetArraySize(hPositions) - 1; i > 0; i--)
		{
			float flFromPos[3], flToPos[3];
			GetArrayArray(hPositions, i, flFromPos, 3);
			GetArrayArray(hPositions, i - 1, flToPos, 3);
			
			TE_SetupBeamPoints(flFromPos,
				flToPos,
				g_iPathLaserModelIndex,
				g_iPathLaserModelIndex,
				0,
				30,
				5.0,
				5.0,
				5.0,
				5, 
				0.0,
				iColor,
				30);
				
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