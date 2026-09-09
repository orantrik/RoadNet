#include "RoadNetCurbs.h"
#include "RoadNetSurface.h"
#include "RoadNetwork.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetCurbCornerTest, "RoadNet.Curbs.ShortCornerPieces",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRoadNetCurbCornerTest::RunTest(const FString& Parameters)
{
	RoadNetMesh::FCenterlineHeightField Height;
	// The corner lies 10 cm after a standard stone, inside the old 25 cm
	// minimum-piece exclusion. Test rotations and both directions of travel.
	for (double Angle : {30.0, 60.0, 90.0, 120.0, 150.0})
	{
		for (double Rotation : {0.0, 37.0, 135.0})
		{
			for (double Tail : {10.0, 210.0})
			{
				const double A = FMath::DegreesToRadians(Rotation);
				const double B = FMath::DegreesToRadians(Rotation + Angle);
				const FVector Corner(110.0 * FMath::Cos(A), 110.0 * FMath::Sin(A), 0.0);
				TArray<FVector> Line = {FVector::ZeroVector, Corner,
					Corner + FVector(Tail * FMath::Cos(B), Tail * FMath::Sin(B), 0.0)};
				for (int32 Reverse = 0; Reverse < 2; ++Reverse)
				{
					if (Reverse) { Swap(Line[0], Line[2]); }
					TArray<RoadNetCurbs::FCurbInstance> Pieces;
					RoadNetCurbs::BuildCurbInstancesAlongLine(Line, Height, 100.0, 0.0, Pieces);
					double Length = 0.0;
					for (const auto& Piece : Pieces)
					{
						Length += Piece.LengthCm;
						const FVector2D Mid(Piece.Location.X, Piece.Location.Y);
						TestTrue(TEXT("Stone midpoint stays on the curb path"),
							RoadNetMath::ProjectToPolyline(Line, Mid).Distance < 0.01);
					}
					TestTrue(TEXT("Pieces cover the entire path without cutting a corner"),
						FMath::Abs(Length - (110.0 + Tail)) < 0.01);
				}
			}
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetJunctionCurbBoundaryTest, "RoadNet.Curbs.JunctionRoundingAngles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRoadNetJunctionCurbBoundaryTest::RunTest(const FString& Parameters)
{
	using namespace UE::Geometry;
	for (double Angle : {30.0, 60.0, 90.0, 120.0, 150.0})
	{
		for (double Rotation : {0.0, 37.0, 135.0})
		{
			// Two-point arms reproduce manual drawing, without OSM node IDs or
			// import conditioning. Densified versions must round identically.
			for (int32 Case = 0; Case < 6; ++Case)
			{
				const int32 Samples = Case % 2 == 0 ? 2 : 61;
				const int32 ThroughArms = Case / 2; // V, T, X
				FRoadCurves A, B;
				for (int32 Arm = 0; Arm < 2; ++Arm)
				{
					FRoadCurves& C = Arm == 0 ? A : B;
					const double Bearing = FMath::DegreesToRadians(Rotation + Arm * Angle);
					for (int32 i = 0; i < Samples; ++i)
					{
						const double Start = Arm < ThroughArms ? -6000.0 : 0.0;
						const double S = FMath::Lerp(Start, 6000.0, (double)i / (Samples - 1));
						C.Sampled.Emplace(S * FMath::Cos(Bearing), S * FMath::Sin(Bearing), 0.0);
					}
					RoadNetMath::OffsetPolyline(C.Sampled, 350.0, C.LeftEdge);
					RoadNetMath::OffsetPolyline(C.Sampled, -350.0, C.RightEdge);
				}
				TArray<const FRoadCurves*> Curves = {&A, &B};
				FGeneralPolygon2d Disc;
				RoadNetSurface::MakeDisc(FVector2D::ZeroVector, 350.0, 48, Disc);
				TArray<FGeneralPolygon2d> Discs = {Disc}, Local, Reference;
				RoadNetSurface::FJunctionClose J;
				J.FillRadiusCm = 350.0;
				J.CloseCm = 150.0;
				TArray<RoadNetSurface::FJunctionClose> Junctions = {J};
				RoadNetSurface::BuildMergedSurface(Curves, Local, 150.0, &Discs, &Junctions);
				RoadNetSurface::BuildMergedSurface(Curves, Reference, 150.0, &Discs);
				// Probe the circular fillet halfway between its two tangent points.
				// This point must be filled in both the local and unrestricted close.
				const double HalfAngle = FMath::DegreesToRadians(Angle * 0.5);
				const double Bisector = FMath::DegreesToRadians(Rotation + Angle * 0.5);
				const double Radius = 500.0 / FMath::Sin(HalfAngle) - 150.0 - 2.0;
				const FVector2d Probe(Radius * FMath::Cos(Bisector), Radius * FMath::Sin(Bisector));
				auto Contains = [&](const TArray<FGeneralPolygon2d>& Polys)
				{
					for (const auto& P : Polys) { if (P.Contains(Probe)) { return true; } }
					return false;
				};
				TestTrue(TEXT("Reference rounds the curb corner"), Contains(Reference));
				TestTrue(TEXT("Local rounding reaches the curb corner"), Contains(Local));
			}
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetCurbSidewalkRejectTest, "RoadNet.Curbs.SidewalkBoundsReject",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

// BuildCurbInstancesForZone rejects sidewalk polygons by bounding box before the
// point-in-polygon walk. A reject that is too eager silently drops kerbs, which
// is invisible until someone looks at the road. Pin the invariant: far-away
// sidewalks must change nothing, and the touching one must still be found.
bool FRoadNetCurbSidewalkRejectTest::RunTest(const FString& Parameters)
{
	using namespace UE::Geometry;

	auto Rect = [](double X0, double Y0, double X1, double Y1)
	{
		FPolygon2d Poly(TArray<FVector2d>{
			FVector2d(X0, Y0), FVector2d(X1, Y0), FVector2d(X1, Y1), FVector2d(X0, Y1)});
		if (Poly.IsClockwise()) { Poly.Reverse(); }
		return FGeneralPolygon2d(Poly);
	};

	// Carriageway 10 m square; footway band abutting its +X edge only.
	const TArray<FGeneralPolygon2d> Surface = {Rect(0.0, 0.0, 1000.0, 1000.0)};
	const TArray<FGeneralPolygon2d> Near    = {Rect(1000.0, 0.0, 1400.0, 1000.0)};

	RoadNetMesh::FCenterlineHeightField Height;
	TArray<RoadNetCurbs::FCurbInstance> Pieces;
	RoadNetCurbs::BuildCurbInstancesForZone(Surface, Near, Height, 100.0, 0.0, Pieces);

	TestTrue(TEXT("The abutting footway produces a kerb run"), Pieces.Num() > 0);
	double Length = 0.0;
	for (const RoadNetCurbs::FCurbInstance& P : Pieces)
	{
		Length += P.LengthCm;
		TestTrue(TEXT("Kerb pieces sit on the shared edge"), FMath::Abs(P.Location.X - 1000.0) < 60.0);
	}
	TestTrue(TEXT("The kerb run spans the shared edge"), FMath::Abs(Length - 1000.0) < 1.0);

	// Same footway plus one a kilometre away: the box reject must skip the distant
	// polygon without altering a single piece.
	TArray<FGeneralPolygon2d> WithFar = Near;
	WithFar.Add(Rect(100000.0, 100000.0, 101000.0, 101000.0));
	TArray<RoadNetCurbs::FCurbInstance> Pieces2;
	RoadNetCurbs::BuildCurbInstancesForZone(Surface, WithFar, Height, 100.0, 0.0, Pieces2);

	TestEqual(TEXT("A distant footway changes nothing"), Pieces2.Num(), Pieces.Num());
	for (int32 i = 0; i < FMath::Min(Pieces.Num(), Pieces2.Num()); ++i)
	{
		TestTrue(TEXT("Piece transforms are unchanged"),
			Pieces2[i].Location.Equals(Pieces[i].Location, 0.01));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetPlacedMarkTest, "RoadNet.Markings.PlacedMarks",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

// Hand-placed marks are authored data with no visual feedback until a rebuild
// finishes, so a bad radius or a flipped heading looks like "the click did
// nothing". Pin both: an arrow must face along the road it lands on, and Delete
// must only lift a mark that is actually under the cursor.
bool FRoadNetPlacedMarkTest::RunTest(const FString& Parameters)
{
	URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
	if (!Net) { AddError(TEXT("Could not create a URoadNetwork")); return false; }

	// One road running due north (+Y), so the expected heading is 90 degrees.
	FRoadDef R;
	R.Ref = { FVector(0, 0, 0), FVector(0, 10000, 0) };
	Net->AddRoad(R);

	float Yaw = -999.f;
	TestTrue(TEXT("A click beside the road takes its heading"),
		Net->HeadingOfNearestRoad(FVector(300, 5000, 0), 2500.0, Yaw));
	TestTrue(TEXT("Heading points along the road, not at the camera"),
		FMath::Abs(FMath::FindDeltaAngleDegrees(Yaw, 90.f)) < 0.01f);

	// Far enough away and the caller must be told to pick its own yaw rather
	// than being handed the heading of a road on the other side of town.
	float Untouched = 123.f;
	TestFalse(TEXT("A click far from any road reports no heading"),
		Net->HeadingOfNearestRoad(FVector(90000, 5000, 0), 2500.0, Untouched));
	TestEqual(TEXT("A failed heading lookup leaves the caller's yaw alone"), Untouched, 123.f);

	const FVector At(300, 5000, 20);
	Net->AddPlacedMark(At, Yaw, ERoadNetMarkKind::ThroughLeft);
	Net->AddPlacedMark(At + FVector(0, 4000, 0), Yaw, ERoadNetMarkKind::Right);
	TestEqual(TEXT("Both marks are stored"), Net->PlacedMarks.Num(), 2);
	TestEqual(TEXT("The mark keeps the kind it was placed with"),
		Net->PlacedMarks[0].Kind, ERoadNetMarkKind::ThroughLeft);

	TestEqual(TEXT("A miss removes nothing"),
		Net->RemovePlacedMarkNear(At + FVector(0, 1000, 0), 300.0), (int32)INDEX_NONE);
	TestEqual(TEXT("Both marks survive a miss"), Net->PlacedMarks.Num(), 2);

	TestEqual(TEXT("A hit removes the mark under the cursor"),
		Net->RemovePlacedMarkNear(At + FVector(50, 50, 0), 300.0), 0);
	TestEqual(TEXT("Only the one mark went"), Net->PlacedMarks.Num(), 1);
	TestEqual(TEXT("The surviving mark is the far one"),
		Net->PlacedMarks[0].Kind, ERoadNetMarkKind::Right);
	return true;
}

#endif
