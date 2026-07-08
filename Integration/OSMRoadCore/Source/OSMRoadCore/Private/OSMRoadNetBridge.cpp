// OSMRoadNetBridge.cpp — OSM ways → RoadNet (Pipeline 4). See header for the
// one-directional dependency rule (OSMRoadCore -> RoadNet only).
#include "OSMRoadNetBridge.h"
#include "OSMRoadContract.h"
#include "RoadNetwork.h"
#include "RoadNetTypes.h"
#include "RoadNetActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"

DEFINE_LOG_CATEGORY_STATIC(LogOSMRoadNet, Log, All);

namespace
{
	ERoadNetClass MapClass(EOSMHighwayClass C)
	{
		switch (C)
		{
			case EOSMHighwayClass::Motorway:     return ERoadNetClass::Motorway;
			case EOSMHighwayClass::Trunk:        return ERoadNetClass::Trunk;
			case EOSMHighwayClass::Primary:      return ERoadNetClass::Primary;
			case EOSMHighwayClass::Secondary:    return ERoadNetClass::Secondary;
			case EOSMHighwayClass::Tertiary:     return ERoadNetClass::Tertiary;
			case EOSMHighwayClass::Residential:  return ERoadNetClass::Residential;
			case EOSMHighwayClass::Unclassified: return ERoadNetClass::Residential;
			case EOSMHighwayClass::Service:      return ERoadNetClass::Service;
			case EOSMHighwayClass::LivingStreet: return ERoadNetClass::Residential;
			case EOSMHighwayClass::Pedestrian:   return ERoadNetClass::Pedestrian;
			case EOSMHighwayClass::Footway:      return ERoadNetClass::Path;
			case EOSMHighwayClass::Cycleway:     return ERoadNetClass::Path;
			case EOSMHighwayClass::Path:         return ERoadNetClass::Path;
			case EOSMHighwayClass::Track:        return ERoadNetClass::Path;
			default:                             return ERoadNetClass::Unknown;
		}
	}

	FRoadDef MakeRoadDef(const FOSMRoadWay& Way)
	{
		FRoadDef R;
		R.Source = ERoadNetSource::OSM;
		R.Class  = MapClass(Way.Class);
		R.Ref    = Way.PointsCm;
		R.NodeIds = Way.NodeIds;
		R.Layer  = Way.Layer;
		R.bBridge = Way.bBridge;
		R.bTunnel = Way.bTunnel;
		R.Name   = Way.Name;

		// Lanes: preserve OSM directional data; derive a lane width that makes
		// FRoadNetLaneSpec::HalfWidthCm() match the shared OSM half-width so
		// RoadNet's carriageway matches every other pipeline.
		FRoadNetLaneSpec& L = R.Lanes;
		L.bOneway  = Way.bOneway;
		L.Forward  = FMath::Max(0, Way.LanesForward);
		L.Backward = FMath::Max(0, Way.LanesBackward);
		L.Total    = Way.Lanes > 0 ? Way.Lanes : (Way.bOneway ? 1 : 2);

		const int32 LaneCount = FMath::Max(1, L.EffectiveLaneCount());
		const float HalfCm    = FMath::Max(50.f, OSMRoadGeom::RoadHalfWidthCm(Way));
		L.LaneWidthDefault    = (2.f * HalfCm) / (float)LaneCount;

		bool bLeft = false, bRight = false;
		OSMRoadGeom::SidewalkSides(Way, bLeft, bRight);
		L.bSidewalkLeft  = bLeft;
		L.bSidewalkRight = bRight;
		L.SidewalkWidth  = FMath::Max(50.f, OSMRoadGeom::SidewalkWidthCm(Way));

		return R;
	}
}

namespace OSMRoadNetBridge
{
	bool BuildRoads(UWorld* World, const TArray<FOSMRoadWay>& Ways, int32& OutNumRoads, FString& OutError)
	{
		OutNumRoads = 0;
		if (!World)
		{
			OutError = TEXT("OSMRoadNetBridge: no world.");
			return false;
		}

		// Find (or spawn) the level-persistent RoadNet actor (§9.1). Both OSM and
		// hand-draw target the same URoadNetwork so they converge in one network.
		ARoadNetActor* NetActor = nullptr;
		for (TActorIterator<ARoadNetActor> It(World); It; ++It) { NetActor = *It; break; }
		if (!NetActor)
		{
			NetActor = World->SpawnActor<ARoadNetActor>();
			if (!NetActor)
			{
				OutError = TEXT("OSMRoadNetBridge: failed to spawn ARoadNetActor.");
				return false;
			}
#if WITH_EDITOR
			NetActor->SetActorLabel(TEXT("RoadNet"));
#endif
		}

		URoadNetwork* Network = NetActor->GetNetwork();
		if (!Network)
		{
			OutError = TEXT("OSMRoadNetBridge: RoadNet actor has no network.");
			return false;
		}
		Network->SetWorld(World);

		// Refresh ONLY the OSM-sourced roads; hand-drawn roads are preserved (§9.4).
		Network->RemoveRoadsBySource(ERoadNetSource::OSM);

		int32 Added = 0;
		for (const FOSMRoadWay& Way : Ways)
		{
			if (Way.PointsCm.Num() < 2) { continue; }
			Network->AddRoad(MakeRoadDef(Way));
			++Added;
		}

		OutNumRoads = Added;
		UE_LOG(LogOSMRoadNet, Log,
			TEXT("OSMRoadNetBridge: registered %d OSM roads from %d ways (network total %d); running RoadNet rebuild..."),
			Added, Ways.Num(), Network->NumRoads());

		Network->Rebuild();   // full rebuild (empty modified = all)
		return true;
	}
}
