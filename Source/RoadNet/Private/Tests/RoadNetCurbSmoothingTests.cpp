#include "RoadNetCurbs.h"
#include "RoadNetMath.h"
#include "RoadNetMesh.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "RoadNetSurface.h"
#include "RoadNetwork.h"
#include "Materials/Material.h"
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetCurbCornerArcTest, "RoadNet.Curbs.CornerRadiusUsesShortStones",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

// The junction close rounds a corner with a round join whose default density is
// relative to the polygon's bounds. Scaled by 1e-3 it gave a 90° corner a single
// segment: a 45° chamfer, and the kerb followed it in two long stones. The corner
// must be the arc, and the kerb on it short stones sitting on that arc.
bool FRoadNetCurbCornerArcTest::RunTest(const FString& Parameters)
{
	using namespace UE::Geometry;
	constexpr double Half = 350.0, Close = 150.0, Reach = 6000.0;

	FRoadCurves East, North;
	East.Sampled  = { FVector(0, 0, 0), FVector(Reach, 0, 0) };
	North.Sampled = { FVector(0, 0, 0), FVector(0, Reach, 0) };
	for (FRoadCurves* C : { &East, &North })
	{
		RoadNetMath::OffsetPolyline(C->Sampled, -Half, C->LeftEdge);
		RoadNetMath::OffsetPolyline(C->Sampled,  Half, C->RightEdge);
	}
	const TArray<const FRoadCurves*> Curves = { &East, &North };

	FGeneralPolygon2d Hull;
	RoadNetSurface::MakeHull({ FVector2D(Half, 0.0), FVector2D(-Half, 0.0),
		FVector2D(0.0, Half), FVector2D(0.0, -Half) }, Hull);
	RoadNetSurface::FJunctionClose J;
	J.FillRadiusCm = Half;
	J.CloseCm = Close;
	const TArray<RoadNetSurface::FJunctionClose> Junctions = { J };
	TArray<FGeneralPolygon2d> Fills = { Hull }, Surface;
	RoadNetSurface::BuildMergedSurface(Curves, Surface, Close, &Fills, &Junctions);

	auto Covered = [&Surface](double X, double Y)
	{
		for (const FGeneralPolygon2d& P : Surface) { if (P.Contains(FVector2d(X, Y))) { return true; } }
		return false;
	};
	// The fillet between the two kerbs (x = Half, y = Half) is centred here.
	const FVector2D Centre(Half + Close, Half + Close);
	TestTrue(TEXT("The corner is paved up to the arc"), Covered(380.0, 380.0));
	TestFalse(TEXT("No chamfer: the inside of the arc stays unpaved"), Covered(410.0, 410.0));

	TArray<FVector> Arc;
	for (const FGeneralPolygon2d& P : Surface)
	{
		for (const FVector2d& V : P.GetOuter().GetVertices())
		{
			if (V.X > Half - 1.0 && V.Y > Half - 1.0 && V.X < Centre.X + 1.0 && V.Y < Centre.Y + 1.0)
			{
				Arc.Emplace(V.X, V.Y, 0.0);
			}
		}
	}
	TestTrue(FString::Printf(TEXT("The 90° corner is an arc, not a chord (%d vertices)"), Arc.Num()), Arc.Num() >= 10);
	for (const FVector& V : Arc)
	{
		const double Off = FMath::Abs(FVector2D::Distance(FVector2D(V.X, V.Y), Centre) - Close);
		TestTrue(FString::Printf(TEXT("A corner vertex lies on the radius (%.1f cm off)"), Off), Off < 3.0);
	}

	auto Angle = [&Centre](const FVector& V)
	{
		const double A = FMath::Atan2(V.Y - Centre.Y, V.X - Centre.X);
		return A < 0.0 ? A + 2.0 * PI : A;
	};
	Arc.Sort([&Angle](const FVector& A, const FVector& B) { return Angle(A) < Angle(B); });
	TArray<FVector> Line = { FVector(Half, Centre.Y + 300.0, 0.0) };
	Line.Append(Arc);
	Line.Add(FVector(Centre.X + 300.0, Half, 0.0));

	RoadNetMesh::FCenterlineHeightField Height;
	TArray<RoadNetCurbs::FCurbInstance> Pieces;
	RoadNetCurbs::BuildCurbInstancesAlongLine(Line, Height, 100.0, 0.0, Pieces);
	int32 Short = 0;
	for (const RoadNetCurbs::FCurbInstance& P : Pieces)
	{
		if (P.Location.X < Half + 1.0 || P.Location.Y < Half + 1.0) { continue; } // the straight leads
		++Short;
		const double Off = FMath::Abs(FVector2D::Distance(FVector2D(P.Location.X, P.Location.Y), Centre) - Close);
		TestTrue(FString::Printf(TEXT("A corner stone is a scaled-down kerb (%.1f cm)"), P.LengthCm), P.LengthCm < 30.f);
		TestTrue(FString::Printf(TEXT("A corner stone sits on the radius (%.1f cm off)"), Off), Off < 3.0);
	}
	TestTrue(FString::Printf(TEXT("The corner takes more than a couple of stones (got %d)"), Short), Short >= 6);
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
					RoadNetMath::OffsetPolyline(C.Sampled, -350.0, C.LeftEdge);
					RoadNetMath::OffsetPolyline(C.Sampled, 350.0, C.RightEdge);
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetJunctionFillShapeTest, "RoadNet.Surface.JunctionFillFollowsArms",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

// The node fill used to be a disc of radius max(arm half-width), which is how a
// narrow arm meeting a wide one grew a bulge of asphalt belonging to neither of
// them — the wide arm's radius stamped omnidirectionally over the narrow one. A
// disc has no orientation, so it cannot be told which way an arm points or how
// wide that particular arm is.
//
// The fill is now the convex hull of the arms' own carriageway edge ends, so
// every vertex of it is a point one of the arms actually reaches. Pin the
// consequence: pavement may not appear in a quadrant no arm occupies.
bool FRoadNetJunctionFillShapeTest::RunTest(const FString& Parameters)
{
	using namespace UE::Geometry;

	// A 14 m wide road running east from the node, and a 7 m one running north
	// from it: the narrow-meets-wide corner the bulge showed up on.
	constexpr double WideHalf = 700.0, NarrowHalf = 350.0, Reach = 6000.0;

	FRoadCurves Wide, Narrow;
	Wide.Sampled   = { FVector(0, 0, 0), FVector(Reach, 0, 0) };
	Narrow.Sampled = { FVector(0, 0, 0), FVector(0, Reach, 0) };
	RoadNetMath::OffsetPolyline(Wide.Sampled,   -WideHalf,    Wide.LeftEdge);
	RoadNetMath::OffsetPolyline(Wide.Sampled,    WideHalf,    Wide.RightEdge);
	RoadNetMath::OffsetPolyline(Narrow.Sampled, -NarrowHalf,  Narrow.LeftEdge);
	RoadNetMath::OffsetPolyline(Narrow.Sampled,  NarrowHalf,  Narrow.RightEdge);
	const TArray<const FRoadCurves*> Curves = { &Wide, &Narrow };

	// The arms' edge ends at the node, which is what BuildSurfaceUnion hulls.
	FGeneralPolygon2d Hull;
	TestTrue(TEXT("The node hull builds from four edge ends"), RoadNetSurface::MakeHull(
		{ FVector2D(0.0,  WideHalf),  FVector2D(0.0, -WideHalf),
		  FVector2D(-NarrowHalf, 0.0), FVector2D(NarrowHalf, 0.0) }, Hull));

	RoadNetSurface::FJunctionClose J;
	J.FillRadiusCm = WideHalf;
	J.CloseCm = 150.0;
	const TArray<RoadNetSurface::FJunctionClose> Junctions = { J };

	TArray<FGeneralPolygon2d> Fills = { Hull }, Surface;
	RoadNetSurface::BuildMergedSurface(Curves, Surface, 150.0, &Fills, &Junctions);
	TestTrue(TEXT("The junction surface builds"), Surface.Num() > 0);

	auto Covered = [&Surface](double X, double Y)
	{
		for (const FGeneralPolygon2d& P : Surface) { if (P.Contains(FVector2d(X, Y))) { return true; } }
		return false;
	};

	// Both arms leave the node to the north and east, so the south-west quadrant
	// is behind both of them. A 7 m disc reached 5.6 m into it.
	TestFalse(TEXT("No asphalt behind the corner"), Covered(-400.0, -400.0));
	// Beside the narrow arm but past the wide arm's start — the other half of the
	// same bulge, and the side the kerb ran into and broke off on.
	TestFalse(TEXT("No asphalt outboard of the narrow arm"), Covered(-500.0, 900.0));

	// The fill must still do its job: the node itself and both arms stay paved,
	// or this trades a bulge for a hole.
	TestTrue(TEXT("The node is paved"),          Covered(0.0, 0.0));
	TestTrue(TEXT("The corner interior is paved"), Covered(200.0, 200.0));
	TestTrue(TEXT("The wide arm is paved"),      Covered(3000.0, 0.0));
	TestTrue(TEXT("The narrow arm is paved"),    Covered(0.0, 3000.0));

	// Two roads joined end to end at a 60 degree bend: the outside of the bend lost a
	// wedge of sidewalk (about 9 m2 here) because each road's side mask ends square at the node.
	auto Walk = [](double BendDeg)
	{
		const double Len = 10000.0;
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		const double B = FMath::DegreesToRadians(BendDeg);
		const FVector Ends[3] = { FVector(0, 0, 0), FVector(Len, 0, 0), FVector(Len + Len * FMath::Cos(B), Len * FMath::Sin(B), 0) };
		for (int32 i = 0; i < 2; ++i)
		{
			FRoadDef R;
			R.Ref = { Ends[i], Ends[i + 1] };
			R.NodeIds = { 1 + i * 9, 10 + i * 10 };
			R.Lanes.Total = 2;
			R.Lanes.bSidewalkLeft = R.Lanes.bSidewalkRight = true;
			R.Lanes.SidewalkWidth = 200.f;
			Net->AddRoad(R);
		}
		Net->Rebuild();
		return Net->LastSidewalkAreaCm2;
	};
	const double Straight = Walk(0.0), Bent = Walk(60.0);
	TestTrue(TEXT("the straight seam has sidewalk"), Straight > 0.0);
	AddInfo(FString::Printf(TEXT("sidewalk straight %.1f m2, bent %.1f m2"), Straight / 1.0e4, Bent / 1.0e4));
	TestTrue(TEXT("the outside of a bent seam keeps its sidewalk"), Bent > Straight - 50000.0);

	// Every corner of a node whose two facing sides have a walk is one piece of walk,
	// solid to the walk's full width along the corner's bisector: the outside of a bend
	// at a seam and at a T, a bend past a lane widening, acute kerb returns, a crossing.
	auto InWalk = [](const URoadNetwork* Net, const FVector2D& P)
	{
		bool bIn = false;
		for (const FRoadNetPlanEdge& E : Net->PlanSidewalkEdges)
		{
			const TArray<FVector>& V = E.Points;
			for (int32 i = 0, j = V.Num() - 1; i < V.Num(); j = i++)
			{
				if ((V[i].Y > P.Y) != (V[j].Y > P.Y)
					&& P.X < (V[j].X - V[i].X) * (P.Y - V[i].Y) / (V[j].Y - V[i].Y) + V[i].X) { bIn = !bIn; }
			}
		}
		return bIn;
	};
	struct FStarArm { double Deg; int32 Lanes; };
	// Roads radiating from a shared node at the origin, or (bCross) straight through it
	// with no shared node, so the two meet as a mid-span crossing.
	auto Star = [](const TArray<FStarArm>& Arms, bool bCross)
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		for (int32 i = 0; i < Arms.Num(); ++i)
		{
			const double B = FMath::DegreesToRadians(Arms[i].Deg);
			const FVector Far(10000.0 * FMath::Cos(B), 10000.0 * FMath::Sin(B), 0.0);
			FRoadDef R;
			R.Ref = { bCross ? -Far : FVector::ZeroVector, Far };
			R.NodeIds = { bCross ? 200 + i : 1, 100 + i };
			R.Lanes.Total = Arms[i].Lanes;
			R.Lanes.bSidewalkLeft = R.Lanes.bSidewalkRight = true;
			R.Lanes.SidewalkWidth = 200.f;
			Net->AddRoad(R);
		}
		Net->Rebuild();
		return Net;
	};
	auto CheckCorners = [&](const TCHAR* Name, const TArray<FStarArm>& Arms, bool bCross)
	{
		const URoadNetwork* Net = Star(Arms, bCross);
		TArray<double> Bearings;
		for (const FStarArm& A : Arms)
		{
			Bearings.Add(FMath::Fmod(A.Deg + 360.0, 360.0));
			if (bCross) { Bearings.Add(FMath::Fmod(A.Deg + 180.0, 360.0)); }
		}
		Bearings.Sort();
		for (int32 i = 0; i < Bearings.Num(); ++i)
		{
			const double Next = i + 1 < Bearings.Num() ? Bearings[i + 1] : Bearings[0] + 360.0;
			const double Mid = FMath::DegreesToRadians(0.5 * (Bearings[i] + Next));
			const FVector2D Dir(FMath::Cos(Mid), FMath::Sin(Mid));
			double Best = 0.0, Run = 0.0;
			for (double S = 0.0; S <= 6000.0; S += 10.0)
			{
				Run = InWalk(Net, Dir * S) ? Run + 10.0 : 0.0;
				Best = FMath::Max(Best, Run);
			}
			TestTrue(FString::Printf(TEXT("%s: the corner at %.0f deg is walked %.0f cm deep (walk 200 cm)"),
				Name, FMath::RadiansToDegrees(Mid), Best), Best >= 180.0);
		}
		int32 Pieces = 0;
		for (const FRoadNetPlanEdge& E : Net->PlanSidewalkEdges) { if (!E.bHole) { ++Pieces; } }
		TestEqual(FString::Printf(TEXT("%s: one walk piece per corner"), Name), Pieces, Bearings.Num());
	};
	CheckCorners(TEXT("seam bent 60 deg"), { { 0.0, 2 }, { 120.0, 2 } }, false);
	CheckCorners(TEXT("T whose through road bends 30 deg"), { { 0.0, 2 }, { 75.0, 2 }, { 150.0, 2 } }, false);
	CheckCorners(TEXT("bent seam gaining two lanes"), { { 0.0, 2 }, { 140.0, 4 } }, false);
	CheckCorners(TEXT("junction with 40 deg corners"), { { 0.0, 2 }, { 40.0, 2 }, { 180.0, 2 }, { 220.0, 2 } }, false);
	CheckCorners(TEXT("roads crossing at 60 deg"), { { 0.0, 2 }, { 60.0, 2 } }, true);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetCornerKerbRadiusTest, "RoadNet.Surface.CornerKerbRadius",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

// A street corner's kerb return is a 3 m arc, 5 m where a secondary or bigger road
// meets it. At a 90° corner between kerbs x = Hx and y = Hy the arc's midpoint is
// (Hx + d, Hy + d) with d = r (1 - 1/√2): the carriageway reaches just short of it
// and the walk starts just past it.
bool FRoadNetCornerKerbRadiusTest::RunTest(const FString& Parameters)
{
	auto InWalk = [](const URoadNetwork* Net, const FVector2D& P)
	{
		bool bIn = false;
		for (const FRoadNetPlanEdge& E : Net->PlanSidewalkEdges)
		{
			const TArray<FVector>& V = E.Points;
			for (int32 i = 0, j = V.Num() - 1; i < V.Num(); j = i++)
			{
				if ((V[i].Y > P.Y) != (V[j].Y > P.Y)
					&& P.X < (V[j].X - V[i].X) * (P.Y - V[i].Y) / (V[j].Y - V[i].Y) + V[i].X) { bIn = !bIn; }
			}
		}
		return bIn;
	};
	auto CheckT = [&](const TCHAR* Name, ERoadNetClass Through, double RadiusCm)
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		const FVector Ends[3] = { FVector(10000, 0, 0), FVector(-10000, 0, 0), FVector(0, 10000, 0) };
		for (int32 i = 0; i < 3; ++i)
		{
			FRoadDef R;
			R.Ref = { FVector::ZeroVector, Ends[i] };
			R.NodeIds = { 1, 10 + i };
			R.Class = i < 2 ? Through : ERoadNetClass::Residential;
			R.Lanes.Total = 2;
			R.Lanes.bSidewalkLeft = R.Lanes.bSidewalkRight = true;
			R.Lanes.SidewalkWidth = 200.f;
			Net->AddRoad(R);
		}
		Net->Rebuild();
		const double Hy = Net->GetRoads()[0].Lanes.HalfWidthCm();
		const double Hx = Net->GetRoads()[2].Lanes.HalfWidthCm();
		const double d = RadiusCm * (1.0 - FMath::Sqrt(0.5));
		for (const double Sx : { 1.0, -1.0 })
		{
			const FVector2D Paved(Sx * (Hx + d - 15.0), Hy + d - 15.0), Walk(Sx * (Hx + d + 20.0), Hy + d + 20.0);
			TestFalse(FString::Printf(TEXT("%s: the %.0f m corner arc is carriageway at (%.0f, %.0f)"),
				Name, RadiusCm / 100.0, Paved.X, Paved.Y), InWalk(Net, Paved));
			TestTrue(FString::Printf(TEXT("%s: the walk starts past the %.0f m arc at (%.0f, %.0f)"),
				Name, RadiusCm / 100.0, Walk.X, Walk.Y), InWalk(Net, Walk));
		}
	};
	CheckT(TEXT("residential T"), ERoadNetClass::Residential, 300.0);
	CheckT(TEXT("residential into secondary T"), ERoadNetClass::Secondary, 500.0);
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetGradeFollowsHillTest, "RoadNet.Grade.LongRoadFollowsHill",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

// A drawn road over a 20 m hill with no junction between its ends was one chord
// held within the 0.5 m budget, so it ran straight through the hill. A road up a
// slope steeper than the 12% grade cap was clamped to 12% from both ends and sank
// tens of metres into the slope.
bool FRoadNetGradeFollowsHillTest::RunTest(const FString& Parameters)
{
	// Worst |road - ground| and worst grade of a road drawn on the ground over [0, Len].
	auto Drive = [this](double LenCm, TFunction<double(double)> GroundZ, double& OutWorstGrade) -> double
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef R;
		for (double X = 0.0; X <= LenCm + 1.0; X += 400.0) { R.Ref.Add(FVector(X, 0.0, GroundZ(X))); }
		R.Lanes.Total = 2;
		Net->AddRoad(R);
		Net->Rebuild();

		OutWorstGrade = 0.0;
		const TArray<FRoadNetDeformCorridor>& Cors = Net->GetDeformCorridors();
		if (!TestEqual(TEXT("The road has one built centreline"), Cors.Num(), 1)) { return 1.0e9; }
		const TArray<FVector>& P = Cors[0].Points;
		double WorstOff = 0.0;
		for (int32 i = 0; i < P.Num(); ++i)
		{
			WorstOff = FMath::Max(WorstOff, FMath::Abs(P[i].Z - GroundZ(P[i].X)));
			if (i > 0)
			{
				OutWorstGrade = FMath::Max(OutWorstGrade,
					FMath::Abs(P[i].Z - P[i - 1].Z) / FMath::Max(1.0, FVector::Dist2D(P[i - 1], P[i])));
			}
		}
		return WorstOff;
	};

	double HillGrade = 0.0;
	const double HillOff = Drive(60000.0, [](double X) { return 2000.0 * FMath::Sin(PI * X / 60000.0); }, HillGrade);
	AddInfo(FString::Printf(TEXT("hill: worst %.0f cm off the ground, worst grade %.1f%%"), HillOff, HillGrade * 100.0));
	TestTrue(TEXT("The road stays within 1.5 m of the hill along its whole length"), HillOff <= 150.0);
	TestTrue(TEXT("Following a hill gentler than the cap keeps the grade under the 12% cap"), HillGrade <= 0.125);

	double SlopeGrade = 0.0;
	const double SlopeOff = Drive(20000.0, [](double X) { return 0.2 * X; }, SlopeGrade);
	AddInfo(FString::Printf(TEXT("20%% slope: worst %.0f cm off the ground, worst grade %.1f%%"), SlopeOff, SlopeGrade * 100.0));
	TestTrue(TEXT("A road up a 20% slope stays on the slope instead of sinking into it"), SlopeOff <= 150.0);

	// A 10 m cliff: the road may not climb it at more than the 15 deg road ceiling.
	double CliffGrade = 0.0;
	Drive(20000.0, [](double X) { return 1000.0 * FMath::Clamp((X - 10000.0) / 200.0, 0.0, 1.0); }, CliffGrade);
	AddInfo(FString::Printf(TEXT("10 m cliff: worst grade %.1f%%"), CliffGrade * 100.0));
	TestTrue(TEXT("A road over a cliff stays at or under the 15 deg ceiling"), CliffGrade <= 0.28);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetHeightBlendCliffTest, "RoadNet.Surface.HeightFieldHasNoBlendCliff",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

// The height field only blends roads within a fixed radius, and it resolves
// overlaps with a soft-max sharp enough to be a hard max. Together those two
// meant a high road held its own level flat out to the radius and then handed
// the full height difference back in one sample — a ring-shaped cliff around
// every junction, which is what shattered kerb chains and trenched the asphalt
// beside them. Walk a kerb-like line straight across that ring and require the
// field to stay continuous over it.
bool FRoadNetHeightBlendCliffTest::RunTest(const FString& Parameters)
{
	// A high through road, and a stem that ramps up 10% to meet it: the exact
	// arrangement the vertical pass produces at a tee.
	TArray<FVector> Through, Stem;
	for (int32 x = -5000; x <= 5000; x += 500) { Through.Emplace((double)x, 0.0, 500.0); }
	for (int32 y = 0; y <= 3000; y += 100)     { Stem.Emplace(0.0, (double)y, 500.0 - 0.1 * y); }

	RoadNetMesh::FCenterlineHeightField Field;
	Field.Build(TArray<const TArray<FVector>*>{ &Through, &Stem });

	// 4 m off the stem's centreline, i.e. where its kerb line runs.
	auto Z = [&Field](double y) { return Field.SampleHeight(400.0, y, 0.0); };

	double Worst = 0.0, WorstAt = 0.0, Prev = Z(100.0);
	for (double y = 105.0; y <= 2500.0; y += 5.0)
	{
		const double Here = Z(y);
		if (FMath::Abs(Here - Prev) > Worst) { Worst = FMath::Abs(Here - Prev); WorstAt = y; }
		Prev = Here;
	}
	TestTrue(FString::Printf(TEXT("The field has no step across the blend radius (worst %.1f cm at y=%.0f)"),
		Worst, WorstAt), Worst < 10.0);

	// And it is a ramp, not a flat line that trivially passes the step check:
	// pinned to the junction near the mouth, on the stem's own grade far out.
	TestTrue(TEXT("The field holds junction level at the junction mouth"), Z(100.0) > 480.0);
	TestTrue(TEXT("The field returns to the road's own grade well away from it"), Z(2500.0) < 270.0);

	// The stones laid along that kerb line stay within a street grade.
	TArray<RoadNetCurbs::FCurbInstance> Stones;
	RoadNetCurbs::BuildCurbInstancesAlongLine({ FVector(400.0, 100.0, 0.0), FVector(400.0, 2500.0, 0.0) }, Field, 100.0, 0.0, Stones);
	float Steepest = 0.f;
	for (const RoadNetCurbs::FCurbInstance& S : Stones) { Steepest = FMath::Max(Steepest, FMath::Abs(S.PitchDeg)); }
	TestTrue(FString::Printf(TEXT("No kerb stone tilts past a 20%% grade (steepest %.1f deg)"), Steepest),
		Stones.Num() > 0 && Steepest <= 11.4f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetCurbGradeCurvatureTest, "RoadNet.Curbs.StonesIgnoreGradeCurvature",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

// A kerb stone is a rigid precast block, so a bend in the GRADE must not shorten
// it — only a bend in PLAN may. The kerb line here is dead straight while the
// road it sits beside crests hard, and the crest is deliberately off a stone
// boundary so one full-length stone has to span it.
bool FRoadNetCurbGradeCurvatureTest::RunTest(const FString& Parameters)
{
	const TArray<FVector> Centre = {
		FVector(0.0, 0.0, 0.0), FVector(1550.0, 0.0, 310.0), FVector(3000.0, 0.0, 0.0) };
	RoadNetMesh::FCenterlineHeightField Field;
	Field.Build(TArray<const TArray<FVector>*>{ &Centre });

	const TArray<FVector> Line = { FVector(0.0, 0.0, 0.0), FVector(2000.0, 0.0, 0.0) };
	TArray<RoadNetCurbs::FCurbInstance> Pieces;
	RoadNetCurbs::BuildCurbInstancesAlongLine(Line, Field, 100.0, 0.0, Pieces);

	TestEqual(TEXT("A straight 20 m kerb line is 20 standard stones"), Pieces.Num(), 20);
	for (const RoadNetCurbs::FCurbInstance& P : Pieces)
	{
		TestTrue(FString::Printf(TEXT("The crest does not shorten a stone (got %.1f cm)"), P.LengthCm),
			FMath::Abs(P.LengthCm - 100.f) < 0.01f);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetFitCircleTest, "RoadNet.Math.FitCircleNoisyArc",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRoadNetFitCircleTest::RunTest(const FString& Parameters)
{
	const FVector2D TrueC(12500.0, -8400.0);
	const double TrueR = 1800.0;
	TArray<FVector2D> Arc;
	Arc.Reserve(24);
	for (int32 i = 0; i < 24; ++i)
	{
		const double A = 0.15 + (1.4 * PI * i) / 23.0;
		const double Noise = (double)((i * 17) % 11) - 5.0;
		Arc.Emplace(TrueC.X + (TrueR + Noise) * FMath::Cos(A),
		            TrueC.Y + (TrueR + Noise) * FMath::Sin(A));
	}
	FVector2D C; double R = 0.0;
	TestTrue(TEXT("FitCircle returns a circle"), RoadNetMath::FitCircle(Arc, C, R));
	TestTrue(FString::Printf(TEXT("Centre within 80 cm (got (%.1f,%.1f))"), C.X, C.Y),
		FVector2D::Distance(C, TrueC) < 80.0);
	TestTrue(FString::Printf(TEXT("Radius within 80 cm (got %.1f)"), R),
		FMath::Abs(R - TrueR) < 80.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetSurfaceMeshInsideTest, "RoadNet.Mesh.TrianglesStayInsidePolygon",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRoadNetSurfaceMeshInsideTest::RunTest(const FString& Parameters)
{
	using namespace UE::Geometry;
	// A 14 cm paint strip bent into a 100 m x 30 m U: thin and concave, so its convex hull
	// is almost all outside it.
	const FPolygon2d Strip(TArray<FVector2d>{
		{0, -7}, {10007, -7}, {10007, 3007}, {0, 3007},
		{0, 2993}, {9993, 2993}, {9993, 7}, {0, 7}});
	const TArray<FVector> Centre = {FVector(0, 0, 0), FVector(10000, 0, 0), FVector(10000, 3000, 0)};
	FDynamicMesh3 Mesh;
	RoadNetMesh::AppendSurfaceMesh({FGeneralPolygon2d(Strip)}, {&Centre}, 0.0, Mesh);

	const FGeneralPolygon2d GP(Strip);
	double Area = 0.0;
	for (const int32 Tid : Mesh.TriangleIndicesItr())
	{
		const FIndex3i T = Mesh.GetTriangle(Tid);
		const FVector2d A(Mesh.GetVertex(T.A)), B(Mesh.GetVertex(T.B)), C(Mesh.GetVertex(T.C));
		TestTrue(TEXT("Triangle centroid is inside the strip"), GP.Contains((A + B + C) / 3.0));
		Area += 0.5 * FMath::Abs((B.X - A.X) * (C.Y - A.Y) - (B.Y - A.Y) * (C.X - A.X));
	}
	TestTrue(FString::Printf(TEXT("Triangles cover the strip exactly (%.0f vs %.0f cm^2)"), Area, Strip.Area()),
		FMath::IsNearlyEqual(Area, Strip.Area(), 1.0));

	// A road straight in plan over a 1.8 m crest (a drawn road draped on a hill):
	// the slab must follow the crest the kerbs sit on, not span it as one chord.
	// Off-axis, the points put back on the deck edges are collinear only to
	// rounding: no triangle may come out flat in plan or wound against its normal.
	for (const double Deg : {0.0, 37.0})
	{
		constexpr double L = 14300.0, Crest = 180.0;
		const FVector2d Ax(FMath::Cos(FMath::DegreesToRadians(Deg)), FMath::Sin(FMath::DegreesToRadians(Deg)));
		const FVector2d Ay(-Ax.Y, Ax.X);
		const FVector2d O(51234.5, -20311.25);
		auto At = [&](double U, double V) { return O + Ax * U + Ay * V; };
		TArray<FVector> Road;
		for (double X = 0.0; X < L; X += 400.0)
		{
			const FVector2d P = At(X, 0.0);
			Road.Add(FVector(P.X, P.Y, Crest * FMath::Sin(PI * X / L)));
		}
		const FVector2d End = At(L, 0.0);
		Road.Add(FVector(End.X, End.Y, 0.0));
		const FPolygon2d Deck(TArray<FVector2d>{At(0, -600), At(L, -600), At(L, 600), At(0, 600)});
		FDynamicMesh3 CrestMesh;
		RoadNetMesh::AppendSurfaceMesh({FGeneralPolygon2d(Deck)}, {&Road}, 0.0, CrestMesh,
			nullptr, /*bComputeUVs*/true, 100.0, /*bGradientNormals*/true, /*bWorldUVs*/false, /*bSkirtToGround*/false);
		RoadNetMesh::FCenterlineHeightField Field;
		Field.Build({&Road});
		double Worst = 0.0, CrestArea = 0.0;
		int32 Flat = 0;
		for (const int32 Tid : CrestMesh.TriangleIndicesItr())
		{
			const FIndex3i T = CrestMesh.GetTriangle(Tid);
			const FVector3d A = CrestMesh.GetVertex(T.A), B = CrestMesh.GetVertex(T.B), C3 = CrestMesh.GetVertex(T.C);
			const FVector3d C = (A + B + C3) / 3.0;
			Worst = FMath::Max(Worst, FMath::Abs(C.Z - Field.SampleHeight(C.X, C.Y, 0.0)));
			const double TwiceArea = FMath::Abs((B.X - A.X) * (C3.Y - A.Y) - (B.Y - A.Y) * (C3.X - A.X));
			Flat += TwiceArea < 1.0 ? 1 : 0;
			CrestArea += 0.5 * TwiceArea;
		}
		TestTrue(FString::Printf(TEXT("Crest %.0f deg: slab within 5 cm of the road height everywhere (worst %.1f cm)"), Deg, Worst),
			CrestMesh.TriangleCount() > 0 && Worst < 5.0);
		TestEqual(FString::Printf(TEXT("Crest %.0f deg: triangles with no plan area"), Deg), Flat, 0);
		TestTrue(FString::Printf(TEXT("Crest %.0f deg: triangles cover the deck (%.0f vs %.0f cm^2)"), Deg, CrestArea, Deck.Area()),
			FMath::IsNearlyEqual(CrestArea, FMath::Abs(Deck.Area()), 10.0));
		TestEqual(FString::Printf(TEXT("Crest %.0f deg: triangles wound against their normals"), Deg),
			RoadNetMesh::RepairWinding(CrestMesh), 0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetSurfaceUVTest, "RoadNet.Mesh.UVsNonDegenerate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRoadNetSurfaceUVTest::RunTest(const FString& Parameters)
{
	using namespace UE::Geometry;
	const double Unit = 100.0, S = 1.0 / Unit;
	// Every triangle, top or wall, must map to UV space at close to its true size: a
	// collapsed or smeared one has no tangent basis and shades wrong once normal-mapped.
	auto Check = [&](const TCHAR* Label, const TArray<FGeneralPolygon2d>& Polys, const TArray<FVector>& Centre)
	{
		FDynamicMesh3 Mesh;
		RoadNetMesh::AppendSurfaceMesh(Polys, {&Centre}, 15.0, Mesh, nullptr, true, Unit, true, false, true);
		const FDynamicMeshUVOverlay* UVs = Mesh.Attributes()->PrimaryUV();
		const FDynamicMeshNormalOverlay* Ns = Mesh.Attributes()->PrimaryNormals();
		int32 Bad = 0, Checked = 0;
		for (const int32 Tid : Mesh.TriangleIndicesItr())
		{
			const FIndex3i T = Mesh.GetTriangle(Tid);
			const double A3 = 0.5 * (Mesh.GetVertex(T.B) - Mesh.GetVertex(T.A)).Cross(Mesh.GetVertex(T.C) - Mesh.GetVertex(T.A)).Length();
			if (A3 < 1.0) { continue; }
			FVector2f U[3]; FVector3f N[3];
			UVs->GetTriElements(Tid, U[0], U[1], U[2]);
			Ns->GetTriElements(Tid, N[0], N[1], N[2]);
			const double AUV = 0.5 * FMath::Abs((double)((U[1].X - U[0].X) * (U[2].Y - U[0].Y) - (U[1].Y - U[0].Y) * (U[2].X - U[0].X)));
			const double Q = AUV / (A3 * S * S);
			++Checked;
			if (Q < 0.25 || Q > 4.0 || !FMath::IsNearlyEqual(N[0].Length(), 1.f, 0.01f)) { ++Bad; }
		}
		TestTrue(FString::Printf(TEXT("%s: meshed (%d triangles)"), Label, Checked), Checked > 0);
		TestEqual(FString::Printf(TEXT("%s: triangles with degenerate UVs or normals"), Label), Bad, 0);
		// Front faces must match the authored normals, or a baked static mesh culls them.
		TestEqual(FString::Printf(TEXT("%s: triangles wound against their normals"), Label), RoadNetMesh::RepairWinding(Mesh), 0);
	};

	// A road slab running 10 m past both ends of its centreline: end caps where every
	// corner clamps to the same station.
	const FPolygon2d Slab(TArray<FVector2d>{{0, -500}, {10000, -500}, {10000, 500}, {0, 500}});
	Check(TEXT("End caps"), {FGeneralPolygon2d(Slab)}, {FVector(1000, 0, 0), FVector(9000, 0, 0)});

	// A roundabout ring: a closed centreline whose stations jump from L back to 0.
	TArray<FVector> Ring, Outer, Inner;
	RoadNetMath::SampleCircle(FVector2D::ZeroVector, 1650.0, 0.0, 64, Ring);
	RoadNetMath::SampleCircle(FVector2D::ZeroVector, 2000.0, 0.0, 64, Outer);
	RoadNetMath::SampleCircle(FVector2D::ZeroVector, 1300.0, 0.0, 64, Inner);
	auto ToPoly = [](const TArray<FVector>& P)
	{
		FPolygon2d Out;
		for (const FVector& V : P) { if (Out.VertexCount() == 0 || FVector2d(V) != Out.GetVertices().Last()) { Out.AppendVertex(FVector2d(V)); } }
		if (Out.VertexCount() > 1 && Out.GetVertices()[0] == Out.GetVertices().Last()) { Out.RemoveVertex(Out.VertexCount() - 1); }
		return Out;
	};
	FGeneralPolygon2d Annulus(ToPoly(Outer));
	FPolygon2d Hole = ToPoly(Inner);
	Hole.Reverse();
	Annulus.AddHole(Hole, false, false);
	Check(TEXT("Roundabout"), {Annulus}, Ring);
	return true;
}

// Merge Points: the second point snaps onto the first and they become one node -
// a junction on two roads, imported (node ids) or drawn (weld only).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetMergePointsTest, "RoadNet.Edit.MergePoints",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRoadNetMergePointsTest::RunTest(const FString&)
{
	auto Road = [](TArray<FVector> Ref, TArray<int64> Ids = {})
	{
		FRoadDef R;
		R.Ref = MoveTemp(Ref);
		R.NodeIds = MoveTemp(Ids);
		return R;
	};
	FString Msg;

	// A drawn road's end onto an imported road's interior node: a T junction.
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(Road({ {0, 0, 0}, {5000, 0, 0}, {10000, 0, 0} }, { 1, 2, 3 }));
		Net->AddRoad(Road({ {5000, 3000, 0}, {5000, 800, 0} }));
		TestTrue(TEXT("T: merges"), Net->MergeRoadPoints(FIntPoint(0, 1), FIntPoint(1, 1), Msg));
		const TArray<FRoadDef>& R = Net->GetRoads();
		if (TestEqual(TEXT("T: the interior point split its road"), R.Num(), 3))
		{
			TestTrue(TEXT("T: the drawn end snapped onto the kept point"), R[1].Ref.Last().Equals(FVector(5000, 0, 0)));
			TestEqual(TEXT("T: no doubled vertex at the cut"), R[0].Ref.Num(), 2);
			TestTrue(TEXT("T: the far half starts at the junction"), R[2].Ref[0].Equals(FVector(5000, 0, 0)));
			bool bStart = false, bEnd = false;
			Net->GetJunctionEnds(1, bStart, bEnd);
			TestTrue(TEXT("T: the drawn road now ends in a junction"), bEnd);
		}
	}

	// An imported end leaving its own junction for another: it takes the kept node id.
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(Road({ {0, 0, 0}, {1000, 0, 0} }, { 10, 11 }));
		FRoadDef Linked = Road({ {1000, 900, 0}, {3000, 900, 0} }, { 20, 21 });
		Linked.StartLinks.AddDefaulted();
		Net->AddRoad(Linked);
		Net->AddRoad(Road({ {1000, 900, 0}, {1000, 3000, 0} }, { 20, 30 }));
		Net->AddRoad(Road({ {1000, 0, 0}, {1000, -2000, 0} }, { 11, 40 }));
		TestTrue(TEXT("node: merges"), Net->MergeRoadPoints(FIntPoint(0, 1), FIntPoint(1, 0), Msg));
		const TArray<FRoadDef>& R = Net->GetRoads();
		TestEqual(TEXT("node: three ends at the node stay a junction"), R.Num(), 4);
		TestTrue(TEXT("node: the end moved"), R[1].Ref[0].Equals(FVector(1000, 0, 0)));
		TestEqual(TEXT("node: it adopts the kept node id"), R[1].NodeIds[0], (int64)11);
		TestEqual(TEXT("node: its links to the old junction are gone"), R[1].StartLinks.Num(), 0);
		TestTrue(TEXT("node: the old junction's other road is untouched"), R[2].Ref[0].Equals(FVector(1000, 900, 0)));
	}

	// One road: neighbours collapse, the two ends close a loop, interior pairs refuse.
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(Road({ {0, 0, 0}, {100, 0, 0}, {5000, 0, 0}, {9000, 0, 0}, {9000, 5000, 0} }));
		TestTrue(TEXT("one road: neighbours merge"), Net->MergeRoadPoints(FIntPoint(0, 1), FIntPoint(0, 0), Msg));
		TestEqual(TEXT("one road: one point fewer"), Net->GetRoads()[0].Ref.Num(), 4);
		TestFalse(TEXT("one road: two interior points apart are refused"),
			Net->MergeRoadPoints(FIntPoint(0, 1), FIntPoint(0, 3), Msg));
		TestTrue(TEXT("one road: the ends close a loop"), Net->MergeRoadPoints(FIntPoint(0, 0), FIntPoint(0, 3), Msg));
		TestTrue(TEXT("one road: loop end on its start"), Net->GetRoads()[0].Ref.Last().Equals(FVector(100, 0, 0)));
	}

	// Two ends meeting where nothing else does: one road through the merged point.
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(Road({ {0, 0, 0}, {1000, 0, 0} }));
		Net->AddRoad(Road({ {1000, 300, 0}, {3000, 300, 0} }));
		const FGuid KeptId = Net->GetRoads()[0].Id;
		TestTrue(TEXT("join: merges"), Net->MergeRoadPoints(FIntPoint(0, 1), FIntPoint(1, 0), Msg));
		const TArray<FRoadDef>& R = Net->GetRoads();
		if (TestEqual(TEXT("join: one road left"), R.Num(), 1))
		{
			TestEqual(TEXT("join: the merged point is a mid point"), R[0].Ref.Num(), 3);
			TestTrue(TEXT("join: through the kept point"), R[0].Ref[1].Equals(FVector(1000, 0, 0)));
			TestTrue(TEXT("join: to the other road's far end"), R[0].Ref.Last().Equals(FVector(3000, 300, 0)));
			TestTrue(TEXT("join: keeps the kept road's id"), R[0].Id == KeptId);
		}
	}

	// Head to head: the snapped road turns round, its traffic and arc data with it.
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef A = Road({ {0, 0, 0}, {1000, 0, 0} });
		A.Lanes.Forward = 1; A.Lanes.Backward = 2; A.Lanes.bSidewalkLeft = true;
		FRoadDef B = Road({ {3000, 0, 0}, {1000, 0, 0} });
		B.Lanes.Forward = 2; B.Lanes.Backward = 1; B.Lanes.bSidewalkRight = true;
		FRoadNetCrossingMark X; X.DistanceCm = 500.f;
		B.Crossings.Add(X);
		FRoadNetEdgeKnot K; K.Distance = 0.0; K.Offset = 400.0;
		B.Lanes.SidewalkKnotsRight.Add(K);
		Net->AddRoad(A);
		Net->AddRoad(B);
		TestTrue(TEXT("reverse: merges"), Net->MergeRoadPoints(FIntPoint(0, 1), FIntPoint(1, 1), Msg));
		const TArray<FRoadDef>& R = Net->GetRoads();
		if (TestEqual(TEXT("reverse: one road left"), R.Num(), 1))
		{
			const FRoadDef& J = R[0];
			TestTrue(TEXT("reverse: runs on to the far end"), J.Ref.Last().Equals(FVector(3000, 0, 0)));
			TestEqual(TEXT("reverse: forward lanes"), J.Lanes.Forward, 1);
			TestEqual(TEXT("reverse: backward lanes"), J.Lanes.Backward, 2);
			if (TestEqual(TEXT("reverse: the crossing is kept"), J.Crossings.Num(), 1))
			{
				TestEqual(TEXT("reverse: the crossing sits where it was"), (double)J.Crossings[0].DistanceCm, 2500.0, 0.5);
			}
			TestEqual(TEXT("reverse: the first part's walk keeps its width"), J.Lanes.SidewalkWidthAt(500.0, true), 200.0, 0.5);
			TestEqual(TEXT("reverse: the turned walk keeps its far-end width"), J.Lanes.SidewalkWidthAt(3000.0, true), 400.0, 0.5);
		}
	}

	// Keep at a road's start: the snapped road goes first.
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(Road({ {1000, 0, 0}, {2000, 0, 0} }));
		Net->AddRoad(Road({ {0, 0, 0}, {1000, 100, 0} }));
		TestTrue(TEXT("prepend: merges"), Net->MergeRoadPoints(FIntPoint(0, 0), FIntPoint(1, 1), Msg));
		const TArray<FRoadDef>& R = Net->GetRoads();
		if (TestEqual(TEXT("prepend: one road left"), R.Num(), 1))
		{
			TestTrue(TEXT("prepend: starts at the snapped road's start"), R[0].Ref[0].Equals(FVector(0, 0, 0)));
			TestTrue(TEXT("prepend: ends at the kept road's end"), R[0].Ref.Last().Equals(FVector(2000, 0, 0)));
		}
	}

	// Different cross-sections or one-ways facing each other stay two roads.
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		Net->AddRoad(Road({ {0, 0, 0}, {1000, 0, 0} }));
		FRoadDef Wide = Road({ {1000, 0, 0}, {3000, 0, 0} });
		Wide.Lanes.Total = 4;
		Net->AddRoad(Wide);
		TestTrue(TEXT("lanes differ: merges"), Net->MergeRoadPoints(FIntPoint(0, 1), FIntPoint(1, 0), Msg));
		TestEqual(TEXT("lanes differ: two roads"), Net->GetRoads().Num(), 2);
		TestTrue(TEXT("lanes differ: says why"), Msg.Contains(TEXT("lanes differ")));
	}
	{
		URoadNetwork* Net = NewObject<URoadNetwork>(GetTransientPackage());
		FRoadDef A = Road({ {0, 0, 0}, {1000, 0, 0} });
		A.Lanes.bOneway = true;
		FRoadDef B = Road({ {2000, 0, 0}, {1000, 0, 0} });
		B.Lanes.bOneway = true;
		Net->AddRoad(A);
		Net->AddRoad(B);
		TestTrue(TEXT("one-ways: merges"), Net->MergeRoadPoints(FIntPoint(0, 1), FIntPoint(1, 1), Msg));
		TestEqual(TEXT("one-ways: two roads"), Net->GetRoads().Num(), 2);
	}
	return true;
}

// A sidewalk outer edge or a kerb chain that comes back from a polygon boolean
// is packed every few centimetres and zigzags. The skirt and the kerb follow
// it, so a raised road grows a stepped wall. Fairing straightens a run whose
// turns cancel and leaves a real bend on its curve.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetBoundaryFairingTest, "RoadNet.Curbs.BoundaryFairing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRoadNetBoundaryFairingTest::RunTest(const FString&)
{
	TArray<FVector2D> Ring;
	for (double X = 0.0; X <= 4000.0; X += 12.0)
	{
		Ring.Emplace(X, (FMath::Fmod(X, 24.0) < 12.0) ? 10.0 : -10.0);
	}
	Ring.Emplace(4000.0, 400.0);
	Ring.Emplace(0.0, 400.0);
	const int32 Before = Ring.Num();
	TestTrue(TEXT("the packed edge loses vertices"), RoadNetMath::FairBoundaryPolyline(Ring, true) > 0);
	TestTrue(TEXT("the ring is shorter"), Ring.Num() < Before / 2);

	int32 Mid = 0;
	double Worst = 0.0;
	double Closest = TNumericLimits<double>::Max();
	for (int32 i = 0; i < Ring.Num(); ++i)
	{
		const FVector2D& P = Ring[i];
		if (P.X < 600.0 || P.X > 3400.0 || P.Y > 50.0) { continue; }
		++Mid;
		Worst = FMath::Max(Worst, FMath::Abs(P.Y));
		const FVector2D& Q = Ring[(i + 1) % Ring.Num()];
		if (Q.X > 600.0 && Q.X < 3400.0 && Q.Y < 50.0)
		{
			Closest = FMath::Min(Closest, FVector2D::Distance(P, Q));
		}
	}
	TestTrue(TEXT("the middle of the straight is still there"), Mid >= 4);
	TestTrue(FString::Printf(TEXT("the zigzag is gone (worst %.1f cm)"), Worst), Worst < 4.0);
	TestTrue(FString::Printf(TEXT("straight vertices are no longer packed (closest %.0f cm)"), Closest),
		Closest >= 70.0);

	// A quarter-circle fillet. Its turns accumulate, so it must stay on the arc
	// rather than being pulled onto the chord.
	TArray<FVector2D> Arc;
	constexpr double R = 200.0;
	const double ArcLen = 0.5 * PI * R;
	for (double S = 0.0; S <= ArcLen; S += 8.0)
	{
		const double A = S / R;
		Arc.Emplace(R * FMath::Cos(A), R * FMath::Sin(A));
	}
	RoadNetMath::FairBoundaryPolyline(Arc, false);
	double Best = TNumericLimits<double>::Max();
	const FVector2D Want(R * FMath::Cos(PI / 4.0), R * FMath::Sin(PI / 4.0));
	for (const FVector2D& P : Arc) { Best = FMath::Min(Best, FVector2D::Distance(P, Want)); }
	TestTrue(FString::Printf(TEXT("the fillet stays on its arc (%.1f cm off)"), Best), Best < 15.0);
	return true;
}

// A decal paints its whole box, so merged or curved marks must split until each
// piece nearly fills its box - else a curved edge line washes the lane white.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetPaintRectsTest, "RoadNet.Markings.DecalsHugPaint",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRoadNetPaintRectsTest::RunTest(const FString&)
{
	using namespace UE::Geometry;
	auto Poly = [](const TArray<FVector2d>& Pts)
	{
		FPolygon2d P;
		for (const FVector2d& Q : Pts) { P.AppendVertex(Q); }
		if (P.IsClockwise()) { P.Reverse(); }
		return FGeneralPolygon2d(P);
	};
	auto Check = [this](const TCHAR* What, const FGeneralPolygon2d& Mark, double MaxHalfWide)
	{
		TArray<RoadNetSurface::FPaintRect> Rects;
		RoadNetSurface::SplitToPaintRects(Mark, Rects);
		double Painted = 0.0, Widest = 0.0;
		for (const RoadNetSurface::FPaintRect& R : Rects)
		{
			Painted += 4.0 * R.HalfLen * R.HalfWide;
			Widest = FMath::Max(Widest, FMath::Min(R.HalfLen, R.HalfWide));
		}
		const double Area = FMath::Abs(Mark.SignedArea());
		TestTrue(FString::Printf(TEXT("%s: decals cover the paint (%.0f of %.0f cm2)"), What, Painted, Area), Painted >= 0.99 * Area);
		TestTrue(FString::Printf(TEXT("%s: overspray %.0f%%"), What, 100.0 * (Painted / Area - 1.0)), Painted <= 1.2 * Area);
		TestTrue(FString::Printf(TEXT("%s: widest decal half-width %.1f cm"), What, Widest), Widest <= MaxHalfWide);
		return Rects.Num();
	};

	TestEqual(TEXT("a straight dash is one decal"),
		Check(TEXT("dash"), Poly({ {0, 0}, {300, 0}, {300, 15}, {0, 15} }), 7.6), 1);

	TArray<FVector2d> Arc;
	for (int32 i = 0; i <= 64; ++i)
	{
		const double A = HALF_PI * i / 64.0;
		Arc.Add(FVector2d(FMath::Cos(A), FMath::Sin(A)) * 5007.5);
	}
	for (int32 i = 64; i >= 0; --i)
	{
		const double A = HALF_PI * i / 64.0;
		Arc.Add(FVector2d(FMath::Cos(A), FMath::Sin(A)) * 4992.5);
	}
	Check(TEXT("curved edge line"), Poly(Arc), 9.0);

	// A stop bar with five zebra bars fused to it.
	TArray<FVector2d> Comb = { {0, 0}, {450, 0}, {450, 40} };
	for (int32 b = 4; b >= 0; --b)
	{
		const double X = 50.0 + 80.0 * b;
		Comb.Append({ {X + 50, 40}, {X + 50, 340}, {X, 340}, {X, 40} });
	}
	Comb.Add({ 0, 40 });
	Check(TEXT("bars fused to a stop line"), Poly(Comb), 26.0);
	return true;
}

// Markings As Decals falls back to these; a decal component cannot draw anything else.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRoadNetPaintDecalTest, "RoadNet.Markings.PaintDecalsAreDecals",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRoadNetPaintDecalTest::RunTest(const FString&)
{
	for (const TCHAR* Path : { TEXT("/RoadNet/Materials/MI_RoadNetPaintWhite.MI_RoadNetPaintWhite"),
		TEXT("/RoadNet/Materials/MI_RoadNetPaintYellow.MI_RoadNetPaintYellow") })
	{
		const UMaterialInterface* M = LoadObject<UMaterialInterface>(nullptr, Path);
		const UMaterial* Base = M ? M->GetMaterial() : nullptr;
		TestTrue(FString::Printf(TEXT("%s loads"), Path), Base != nullptr);
		TestTrue(FString::Printf(TEXT("%s is a Deferred Decal"), Path), Base && Base->MaterialDomain == MD_DeferredDecal);
	}
	return true;
}

#endif
