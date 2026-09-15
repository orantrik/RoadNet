// ===========================================================================
// RoadNetGrade.cpp — vertical alignment stage.
//
// Standard road-design model (FAO §3.1.3): the road runs a TANGENT GRADE
// between vertical points of intersection, and each VPI is rounded by a
// vertical curve. Here the junctions are the VPIs, so between two junctions
// the centreline is a straight line in (arc length, Z) — no sag, no terrain
// following — and every junction sits on a flat plate with a smooth grade
// transition on each approach.
//
// Runs after BuildCurves/BuildCrossings and before the deform-corridor
// snapshot, so the terrain conform and the mesh see the SAME reconciled Z.
// ===========================================================================
#include "RoadNetwork.h"
#include "RoadNetActor.h"
#include "RoadNetMath.h"
#include "RoadNetStandards.h"
#include "RoadNetLog.h"
#include "EngineUtils.h"
#include "HAL/IConsoleManager.h"

// ---- tuning (mirrored on the OSM control panel) ---------------------------
static TAutoConsoleVariable<float> CVarRoadNetStraightSpanM(
	TEXT("roadnet.StraightSpanM"),
	150.0f,
	TEXT("Junction span length (m) at or below which the road is a DEAD STRAIGHT grade between junctions. Longer spans are allowed to bow toward the terrain, ramping up to roadnet.MaxChordDeviationM. Default 150."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarRoadNetMaxChordDeviationM(
	TEXT("roadnet.MaxChordDeviationM"),
	3.0f,
	TEXT("Maximum height (m) a long span may deviate from the straight chord between its two junctions, so a long link can still follow rolling ground. Reached at 3x roadnet.StraightSpanM. Default 3."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarRoadNetMaxJunctionRelaxCm(
	TEXT("roadnet.MaxJunctionRelaxCm"),
	300.0f,
	TEXT("How far (cm) a junction may be pulled BELOW its highest arriving road so straight chords do not turn one noisy drape sample into a network-wide berm. 0 disables the relaxation. Default 300."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarRoadNetVerticalCurveKM(
	TEXT("roadnet.VerticalCurveKM"),
	2.0f,
	TEXT("Vertical-curve rate: metres of grade transition per 1% of grade change at a junction, capped by osm.RoadJunctionLandingCm. Higher = longer, gentler crest/sag. Default 2."),
	ECVF_Default);

// Max SIDE slope between two road beds that share a corridor (dual
// carriageways, junction aprons). Enforced HERE, on the latent profiles —
// where a correction can raise AND lower and is then re-smoothed — never on
// mesh vertices (mesh-space raising builds plateaus; see git history).
static TAutoConsoleVariable<float> CVarRoadNetMaxSideSlopeDeg(
	TEXT("roadnet.MaxSideSlopeDeg"),
	2.0f,
	TEXT("Max lateral slope (degrees) between nearby parallel road beds, enforced on the vertical alignment before meshing. Default 2."),
	ECVF_Default);

// Longitudinal grades past this are not roads any more (the user cap is
// 2-15 degrees; 15 deg = tan 0.2679). osm.RoadGradeMaxSlope stays the working
// cap; this is the ceiling it can never exceed.
static constexpr double kMaxLongitudinalTan = 0.2679;

namespace
{
	// Cross-module CVar read (the junction/grade knobs live in OSMRoadCore, and
	// RoadNet must not depend on it). Falls back when that module is absent.
	double ReadCVarFloat(const TCHAR* Name, double Fallback)
	{
		const IConsoleVariable* CV = IConsoleManager::Get().FindConsoleVariable(Name);
		return CV ? (double)CV->GetFloat() : Fallback;
	}

	// Cubic smoothstep, 0 at t=0 and 1 at t=1 with zero slope at BOTH ends — the
	// blend that makes a grade transition C1 where it meets the flat junction
	// plate and again where it meets the tangent.
	FORCEINLINE double SmoothStep01(double t)
	{
		return t * t * (3.0 - 2.0 * t);
	}

	// Residual slope defects of the LAST vertical alignment, for the
	// RoadNet.SlopeSelfCheck console command. All zero is the latent solver's
	// contract: no cliff and no >cap side slope ever reaches the mesh.
	struct FRoadNetSlopeCheck
	{
		int32 UnfixableChords     = 0;
		int32 LateralResiduals    = 0;
		int32 RampsTooTight       = 0;
		int32 OverGradedJunctions = 0;
	};
	FRoadNetSlopeCheck GSlopeCheck;
}

void URoadNetwork::BuildVerticalAlignment(FRoadNetRebuildContext& Ctx) const
{
	if (Ctx.Curves.Num() == 0)
	{
		return;
	}

	const double StraightCm = FMath::Max(1.0,   (double)CVarRoadNetStraightSpanM.GetValueOnAnyThread() * 100.0);
	const double MaxDevCm   = FMath::Max(0.0,   (double)CVarRoadNetMaxChordDeviationM.GetValueOnAnyThread() * 100.0);
	const double RelaxCm    = FMath::Max(0.0,   (double)CVarRoadNetMaxJunctionRelaxCm.GetValueOnAnyThread());
	const double CurveKCm   = FMath::Max(0.0,   (double)CVarRoadNetVerticalCurveKM.GetValueOnAnyThread() * 100.0);
	const double FlatCm     = FMath::Max(0.0,   ReadCVarFloat(TEXT("osm.RoadJunctionFlatCm"),    300.0));
	const double LandingCm  = FMath::Max(1.0,   ReadCVarFloat(TEXT("osm.RoadJunctionLandingCm"), 800.0));
	const double MaxSlope   = FMath::Clamp(ReadCVarFloat(TEXT("osm.RoadGradeMaxSlope"), 0.12),
		0.005, kMaxLongitudinalTan);

	// -----------------------------------------------------------------------
	// 1. Junction elevations (כרך 2 §8.5).
	// -----------------------------------------------------------------------
	// This used to be "highest Z wins" over every arriving road, which is where
	// the spikes came from: one noisy drape sample on one minor arm lifted the
	// whole junction, and every chord radiating out of it became a berm.
	//
	// §8.5.3 says the opposite. The MAIN road holds its own level and crossfall
	// straight through the junction, and each secondary arm is warped up or
	// down into it. So the junction's elevation is the main axis's own
	// elevation, and a minor arm has no vote at all. Where no arm dominates
	// (a Y with no through road), §8.5.4's plane method applies instead: fit
	// the junction plate to all the arms together.
	// Z at one end of a road. Prefers the freshly built curve (this rebuild's
	// draped + grade-smoothed bed); roads outside a windowed rebuild fall back
	// to their persistent Ref/Elev, which is also what the NEXT full rebuild
	// will read — so repeated rebuilds are idempotent, not cumulative.
	auto ArmEndZ = [this, &Ctx](int32 RoadIdx, bool bStart) -> double
	{
		if (const FRoadCurves* C = Ctx.Curves.Find(RoadIdx))
		{
			if (C->Sampled.Num() > 0)
			{
				return bStart ? C->Sampled[0].Z : C->Sampled.Last().Z;
			}
		}
		if (Roads.IsValidIndex(RoadIdx) && Roads[RoadIdx].Ref.Num() > 0)
		{
			const FRoadDef& R = Roads[RoadIdx];
			const int32 i = bStart ? 0 : R.Ref.Num() - 1;
			return R.Elev.IsValidIndex(i) ? R.Elev[i] : R.Ref[i].Z;
		}
		return 0.0;
	};

	const int32 NumJoints = Ctx.Joints.Num();
	TArray<double> JointZ, JointBase;
	TBitArray<>    JointRelaxable(false, NumJoints);
	JointZ.SetNumZeroed(NumJoints);
	JointBase.SetNumZeroed(NumJoints);

	// Road end -> joint. Keyed by an encoded (road, end) so the map hashes ints.
	auto ArmKey = [](int32 RoadIdx, bool bStart) { return RoadIdx * 2 + (bStart ? 0 : 1); };
	TMap<int32, int32> ArmToJoint;
	for (int32 j = 0; j < NumJoints; ++j)
	{
		const FRoadNetJoint& J = Ctx.Joints[j];
		for (const FRoadNetJointArm& Arm : J.Arms)
		{
			ArmToJoint.Add(ArmKey(Arm.Road, Arm.bAtStart), j);
		}

		double Base = J.Z;
		if (J.Arms.IsValidIndex(J.MainA) && J.Arms.IsValidIndex(J.MainB))
		{
			// §8.5.3 crossfall preservation — the main road runs through at its
			// own level, so the node sits on the main road's profile.
			Base = 0.5 * (ArmEndZ(J.Arms[J.MainA].Road, J.Arms[J.MainA].bAtStart)
			            + ArmEndZ(J.Arms[J.MainB].Road, J.Arms[J.MainB].bAtStart));
		}
		else if (J.Arms.Num() > 0)
		{
			// §8.5.4 plane method. ponytail: the mean of the arm elevations,
			// not a least-squares plane evaluated at the node. The arm ends are
			// welded to within kEndpointWeldCm of each other, so the two answers
			// differ by less than the drape noise they are both smoothing. If
			// junction plates ever get big enough for the tilt to read, the
			// upgrade is a proper plane fit through (x, y, z) of the arm ends.
			double Sum = 0.0;
			for (const FRoadNetJointArm& Arm : J.Arms) { Sum += ArmEndZ(Arm.Road, Arm.bAtStart); }
			Base = Sum / (double)J.Arms.Num();
		}

		JointBase[j] = Base;
		JointZ[j]    = Base;
		// A dead end is not a junction: it stays pinned to its own ground level.
		JointRelaxable[j] = (J.Arms.Num() >= 2);
	}

	// -----------------------------------------------------------------------
	// 2. Relax junction elevations network-wide.
	// -----------------------------------------------------------------------
	// With straight chords, one junction's elevation drives cut/fill all the way
	// to its neighbours, so a single noisy drape sample would be amplified into a
	// berm. Treat the junctions as a graph and pull each toward the elevation
	// that puts its chords closest to the ground: for a link to neighbour k whose
	// mean terrain elevation is T, the chord midpoint lands on the ground when
	// Zj = 2*T - Zk. Short links pull harder (weight 1/Length). Every sweep is
	// clamped to within the budget of the §8.5 base elevation, in EITHER
	// direction — under "highest wins" the base was an upper bound so the clamp
	// was one-sided, but the main road's own level is a target to stay near,
	// not a ceiling.
	struct FGradeLink { int32 Ja = 0; int32 Jb = 0; double Weight = 0.0; double MeanZ = 0.0; };
	TArray<FGradeLink> Links;
	for (int32 r = 0; r < Roads.Num(); ++r)
	{
		if (!Roads[r].IsValid())
		{
			continue;
		}
		const int32* Pa = ArmToJoint.Find(ArmKey(r, true));
		const int32* Pb = ArmToJoint.Find(ArmKey(r, false));
		if (!Pa || !Pb || *Pa == *Pb)
		{
			continue;
		}
		double Len = 0.0, SumZ = 0.0;
		int32  Cnt = 0;
		if (const FRoadCurves* C = Ctx.Curves.Find(r))
		{
			Len = C->Length;
			for (const FVector& P : C->Sampled) { SumZ += P.Z; ++Cnt; }
		}
		else
		{
			Len = RoadNetMath::TotalLength(Roads[r].Ref);
			for (const FVector& P : Roads[r].Ref) { SumZ += P.Z; ++Cnt; }
		}
		if (Cnt == 0 || Len < 1.0)
		{
			continue;
		}
		Links.Add({ *Pa, *Pb, 1.0 / Len, SumZ / (double)Cnt });
	}

	if (RelaxCm > 0.0 && Links.Num() > 0)
	{
		// ponytail: fixed sweep count instead of a convergence test. 8 Jacobi
		// sweeps move ~8 junctions' worth of information; on a very long chain of
		// junctions the far end relaxes less. Upgrade path is a residual-based
		// loop if that ever shows up as a visible seam.
		constexpr int32 kSweeps = 8;
		TArray<double> Acc, W;
		for (int32 Sweep = 0; Sweep < kSweeps; ++Sweep)
		{
			Acc.Init(0.0, NumJoints);
			W.Init(0.0, NumJoints);
			for (const FGradeLink& L : Links)
			{
				Acc[L.Ja] += L.Weight * (2.0 * L.MeanZ - JointZ[L.Jb]); W[L.Ja] += L.Weight;
				Acc[L.Jb] += L.Weight * (2.0 * L.MeanZ - JointZ[L.Ja]); W[L.Jb] += L.Weight;
			}
			for (int32 j = 0; j < NumJoints; ++j)
			{
				if (!JointRelaxable[j] || W[j] <= 0.0)
				{
					continue;
				}
				JointZ[j] = FMath::Clamp(Acc[j] / W[j], JointBase[j] - RelaxCm, JointBase[j] + RelaxCm);
			}
		}
	}

	// -----------------------------------------------------------------------
	// 2b. Steep-chord negotiation — junctions give ground before the road does.
	// -----------------------------------------------------------------------
	// No profile beats a straight line between two FIXED junctions, so when the
	// chord itself is steeper than the cap the self-check below used to just
	// warn and the cliff sailed into the mesh. Fix the only thing that can be
	// fixed: the junction levels. Pull the two ends toward each other until the
	// chord is legal. This deliberately may exceed RelaxCm — a junction sitting
	// further from its §8.5 base level is a blemish, a cliff is a defect.
	int32 ChordViolations = 0, ChordUnfixable = 0;
	if (Links.Num() > 0)
	{
		for (const FGradeLink& L : Links)
		{
			const double Len = 1.0 / FMath::Max(L.Weight, 1e-9);
			if (FMath::Abs(JointZ[L.Jb] - JointZ[L.Ja]) > MaxSlope * Len + 1e-3)
			{
				++ChordViolations;
			}
		}
		// Gauss-Seidel: each fix may steepen a neighbouring chord, so sweep until
		// quiet. Moves shrink the total Z spread monotonically, so this converges.
		constexpr int32 kNegotiateSweeps = 24;
		for (int32 Sweep = 0; Sweep < kNegotiateSweeps && ChordViolations > 0; ++Sweep)
		{
			bool bAny = false;
			for (const FGradeLink& L : Links)
			{
				const double Len   = 1.0 / FMath::Max(L.Weight, 1e-9);
				const double MaxDz = MaxSlope * Len;
				const double dZ    = JointZ[L.Jb] - JointZ[L.Ja];   // + when Jb is higher
				if (FMath::Abs(dZ) <= MaxDz + 1e-3)
				{
					continue;
				}
				const bool bA = JointRelaxable[L.Ja];
				const bool bB = JointRelaxable[L.Jb];
				if (!bA && !bB)
				{
					continue;   // two pinned dead ends: reported as a steep chord below
				}
				const double Excess = FMath::Abs(dZ) - MaxDz;
				const double Sign   = (dZ > 0.0) ? 1.0 : -1.0;
				const double MoveA  = bA ? (bB ? 0.5 * Excess : Excess) : 0.0;
				const double MoveB  = bB ? (bA ? 0.5 * Excess : Excess) : 0.0;
				JointZ[L.Ja] += Sign * MoveA;
				JointZ[L.Jb] -= Sign * MoveB;
				bAny = true;
			}
			if (!bAny)
			{
				break;
			}
		}
		for (const FGradeLink& L : Links)
		{
			const double Len = 1.0 / FMath::Max(L.Weight, 1e-9);
			if (FMath::Abs(JointZ[L.Jb] - JointZ[L.Ja]) > MaxSlope * Len + 1e-3)
			{
				++ChordUnfixable;
			}
		}
	}

	// -----------------------------------------------------------------------
	// 3. Interior anchors — mid-span crossings are junctions too.
	// -----------------------------------------------------------------------
	TMap<int32, TArray<TPair<double, double>>> Interior;   // road -> (arc cm, Z)
	for (const FRoadNetCrossing& X : Ctx.Crossings)
	{
		if (!Roads.IsValidIndex(X.RoadA) || !Roads.IsValidIndex(X.RoadB))
		{
			continue;
		}
		const FRoadDef& A = Roads[X.RoadA];
		const FRoadDef& B = Roads[X.RoadB];
		// Grade-separated crossings must NOT share an elevation.
		if (A.Layer != B.Layer || A.bBridge || B.bBridge || A.bTunnel || B.bTunnel)
		{
			continue;
		}
		const double KingZ = FMath::Max(X.Za, X.Zb);
		for (const int32 r : { X.RoadA, X.RoadB })
		{
			const FRoadCurves* C = Ctx.Curves.Find(r);
			if (!C || C->Sampled.Num() < 2)
			{
				continue;
			}
			const RoadNetMath::FProjectResult PR = RoadNetMath::ProjectToPolyline(C->Sampled, X.Point);
			Interior.FindOrAdd(r).Emplace(PR.AlongDist, KingZ);
		}
	}

	// -----------------------------------------------------------------------
	// 4. Per-road profile: chord -> budget -> grade cap -> plate + curve.
	// -----------------------------------------------------------------------
	int32 RoadsAligned = 0, SteepChords = 0, BudgetBreaks = 0, RelaxEscapes = 0;
	// "A high junction meeting a low road": every anchor whose junction level sits
	// clear of the ground the road would otherwise lie on. Counted so the pass can
	// say how many it found and ramped, and warned about individually only when
	// the road ran out of length to swallow one at a legal grade.
	constexpr double kStepReportCm = 30.0;
	int32 StepsRamped = 0, StepsTooTight = 0, StepsReported = 0;

	for (int32 j = 0; j < NumJoints; ++j)
	{
		if (JointRelaxable[j] && FMath::Abs(JointZ[j] - JointBase[j]) > RelaxCm + 1e-3)
		{
			++RelaxEscapes;
		}
	}

	// Per-road guards for the lateral cross-tie pass (4b): which samples are a
	// junction plate / vertical curve / pinned anchor (untouchable), and the
	// arc-length table so the tie can re-run the grade clamp afterwards.
	TMap<int32, TBitArray<>>     TieProtected;
	TMap<int32, TArray<double>>  TieArcS;

	for (TPair<int32, FRoadCurves>& KV : Ctx.Curves)
	{
		const int32   RoadIdx = KV.Key;
		FRoadCurves&  C       = KV.Value;
		const int32   N       = C.Sampled.Num();
		if (N < 2)
		{
			continue;
		}

		TArray<double> S;
		RoadNetMath::CumulativeLength(C.Sampled, S);
		const double Len = S.Last();
		if (Len < 1.0)
		{
			continue;
		}

		// Nearest sample index to an arc position (S is sorted ascending).
		auto NearestIdx = [&S, N](double s) -> int32
		{
			int32 Lo = 0, Hi = N - 1;
			while (Lo < Hi)
			{
				const int32 Probe = (Lo + Hi) / 2;
				if (S[Probe] < s) { Lo = Probe + 1; } else { Hi = Probe; }
			}
			if (Lo > 0 && FMath::Abs(S[Lo - 1] - s) < FMath::Abs(S[Lo] - s))
			{
				return Lo - 1;
			}
			return Lo;
		};

		// ---- anchors: both ends plus any mid-span crossing -------------------
		// A crossing nearer an end than MinGap has no room for its own flat plate
		// and vertical curve, so it cannot be a MID-span anchor. It used to be
		// dropped, and that is what put a cliff at the mouth of every T.
		//
		// A minor road that tees into a major one ENDS on the major road's
		// centreline. There is no endpoint-to-endpoint weld there (the major road
		// runs straight past), so the joint pass never sees it and the end anchor
		// fell back to the road's own draped ground level. The major road, whose
		// crossing IS mid-span, kept its anchor and was lifted to KingZ. One side
		// lifted, the other left on the terrain, nothing in between: a vertical
		// step exactly at the junction mouth.
		//
		// A crossing at the end is not a mid-span anchor to discard — it is that
		// END's junction level. Promote it, and the plate-and-curve pass below
		// ramps the arm up onto the junction the way it already does at a weld.
		const double MinGap = FMath::Max(1.0, 2.0 * FlatCm);
		TArray<TPair<double, double>> MidAnchors;
		TOptional<double> StartCrossZ, EndCrossZ;
		if (const TArray<TPair<double, double>>* Ints = Interior.Find(RoadIdx))
		{
			for (const TPair<double, double>& It : *Ints)
			{
				if (It.Key <= MinGap)
				{
					// Several crossings can pile up at one mouth (a divided road
					// is two centrelines); the junction sits at the highest, which
					// is the same "king road wins" rule KingZ applies.
					StartCrossZ = StartCrossZ.IsSet() ? FMath::Max(*StartCrossZ, It.Value) : It.Value;
				}
				else if (It.Key >= Len - MinGap)
				{
					EndCrossZ = EndCrossZ.IsSet() ? FMath::Max(*EndCrossZ, It.Value) : It.Value;
				}
				else
				{
					MidAnchors.Add(It);
				}
			}
		}
		MidAnchors.Sort([](const TPair<double, double>& A, const TPair<double, double>& B) { return A.Key < B.Key; });
		for (int32 i = MidAnchors.Num() - 1; i > 0; --i)
		{
			if (MidAnchors[i].Key - MidAnchors[i - 1].Key < MinGap)
			{
				MidAnchors[i - 1].Value = FMath::Max(MidAnchors[i - 1].Value, MidAnchors[i].Value);
				MidAnchors.RemoveAt(i);
			}
		}

		const int32* JStart = ArmToJoint.Find(ArmKey(RoadIdx, true));
		const int32* JEnd   = ArmToJoint.Find(ArmKey(RoadIdx, false));

		// A joint of two or more arms outranks a crossing: the joint pass has
		// already balanced every arm meeting there, while a crossing only knows
		// about two roads.
		//
		// A ONE-arm joint does not outrank anything — it is the record of a dead
		// end, and it is why this bug survived. Every road end gets a joint, so a
		// tee's stem had one too, holding it at its own draped ground level. But a
		// dead end that a crossing lands on is not a dead end at all: it is a tee
		// whose stem never welded, because the road it runs into carries straight
		// past instead of ending there.
		auto EndAnchorZ = [&Ctx, &JointZ](const int32* Joint,
			const TOptional<double>& CrossZ, double DrapeZ)
		{
			const bool bWelded = Joint && Ctx.Joints.IsValidIndex(*Joint)
				&& Ctx.Joints[*Joint].Arms.Num() >= 2;
			if (bWelded)        { return JointZ[*Joint]; }
			if (CrossZ.IsSet()) { return *CrossZ; }
			return Joint ? JointZ[*Joint] : DrapeZ;
		};

		TArray<TPair<double, double>> Anchors;
		Anchors.Reserve(MidAnchors.Num() + 2);
		Anchors.Emplace(0.0, EndAnchorZ(JStart, StartCrossZ, C.Sampled[0].Z));
		Anchors.Append(MidAnchors);
		Anchors.Emplace(Len, EndAnchorZ(JEnd, EndCrossZ, C.Sampled.Last().Z));

		// ---- straight chord with a span-scaled deviation budget --------------
		// Budget(L) = MaxDev * clamp((L - Straight) / (2*Straight), 0, 1)
		// A span of Straight or less gets Budget 0, which IS the straight chord;
		// the budget ramps in continuously so nothing pops as a span grows.
		TArray<double> Z;
		Z.SetNumUninitialized(N);
		{
			int32 a = 0;
			for (int32 i = 0; i < N; ++i)
			{
				while (a + 2 < Anchors.Num() && S[i] > Anchors[a + 1].Key) { ++a; }
				const double Sa = Anchors[a].Key,     Za = Anchors[a].Value;
				const double Sb = Anchors[a + 1].Key, Zb = Anchors[a + 1].Value;
				const double L  = FMath::Max(1.0, Sb - Sa);
				const double Chord  = FMath::Lerp(Za, Zb, FMath::Clamp((S[i] - Sa) / L, 0.0, 1.0));
				const double Budget = MaxDevCm * FMath::Clamp((L - StraightCm) / (2.0 * StraightCm), 0.0, 1.0);
				Z[i] = FMath::Clamp(C.Sampled[i].Z, Chord - Budget, Chord + Budget);
			}
		}

		// ---- pin anchors, then cap the local grade ---------------------------
		TBitArray<> Pinned(false, N);
		for (const TPair<double, double>& A : Anchors)
		{
			const int32 Ai = NearestIdx(A.Key);
			Pinned[Ai] = true;
			Z[Ai]      = A.Value;
		}

		// Only bites on spans long enough to have kept some terrain shape; on a
		// short span the chord already satisfies the cap and this is a no-op.
		for (int32 Pass = 0; Pass < 3; ++Pass)
		{
			for (int32 i = 1; i < N; ++i)
			{
				if (Pinned[i]) { continue; }
				const double dS = FMath::Max(1.0, S[i] - S[i - 1]);
				Z[i] = FMath::Clamp(Z[i], Z[i - 1] - MaxSlope * dS, Z[i - 1] + MaxSlope * dS);
			}
			for (int32 i = N - 2; i >= 0; --i)
			{
				if (Pinned[i]) { continue; }
				const double dS = FMath::Max(1.0, S[i + 1] - S[i]);
				Z[i] = FMath::Clamp(Z[i], Z[i + 1] - MaxSlope * dS, Z[i + 1] + MaxSlope * dS);
			}
		}

		// ---- flat plate + vertical curve at every anchor ---------------------
		// The plate holds the junction surface level out to FlatCm. Beyond it the
		// profile eases from grade 0 back onto the tangent over a length that
		// scales with the grade change (K metres per 1%), smoothstep-blended so
		// grade is continuous at BOTH joins. The blend is strictly local, so the
		// tangent — and therefore the far anchor — is untouched.
		const TArray<double> Tangent = Z;
		auto TangentAt = [&](double s) -> double
		{
			const double sc = FMath::Clamp(s, 0.0, Len);
			const int32  i  = NearestIdx(sc);
			if (i > 0 && S[i] > sc)
			{
				const double d = FMath::Max(1.0, S[i] - S[i - 1]);
				return FMath::Lerp(Tangent[i - 1], Tangent[i], (sc - S[i - 1]) / d);
			}
			if (i < N - 1 && S[i] < sc)
			{
				const double d = FMath::Max(1.0, S[i + 1] - S[i]);
				return FMath::Lerp(Tangent[i], Tangent[i + 1], (sc - S[i]) / d);
			}
			return Tangent[i];
		};

		// Reach of each anchor's plate + vertical curve, so the budget self-check
		// below skips exactly the stretch this pass deliberately shaped.
		TArray<double> Reach;
		Reach.Init(FlatCm, Anchors.Num());

		for (int32 ai = 0; ai < Anchors.Num(); ++ai)
		{
			const double Sa = Anchors[ai].Key;
			const double Za = Anchors[ai].Value;

			// How far this junction rides above (or below) the ground the road
			// would otherwise lie on. C.Sampled still holds the raw drape here —
			// the reconciled profile is written back after this loop — so the two
			// levels are directly comparable.
			//
			// This is the quantity the whole "high junction, low road" complaint
			// is about, so it now sizes the ramp rather than being ignored. A fixed
			// landing put a 3 m step onto an 8 m run, which is 37% and reads as a
			// wall; the length a step needs to land at the legal grade is
			// Step/MaxSlope, so ask for that and let the neighbouring anchors be
			// the only thing allowed to shorten it.
			const int32  AnchorIdx = NearestIdx(Sa);
			const double StepCm    = FMath::Abs(Za - C.Sampled[AnchorIdx].Z);

			double MaxH = FMath::Max(LandingCm, StepCm / MaxSlope);
			if (ai > 0)                 { MaxH = FMath::Min(MaxH, 0.5 * (Sa - Anchors[ai - 1].Key)); }
			if (ai + 1 < Anchors.Num()) { MaxH = FMath::Min(MaxH, 0.5 * (Anchors[ai + 1].Key - Sa)); }
			MaxH = FMath::Max(0.0, MaxH - FlatCm);

			if (StepCm > kStepReportCm)
			{
				++StepsRamped;
				// The neighbours won: this anchor cannot get the length its step
				// needs, so what lands here is steeper than the cap. Worth naming,
				// because the fix is to move a junction, not to retune anything.
				if (MaxH * MaxSlope + 1.0 < StepCm)
				{
					++StepsTooTight;
					if (StepsReported < 5)
					{
						++StepsReported;
						UE_LOG(LogRoadNet, Warning,
							TEXT("[RoadNet][GRADE] junction at (%.0f, %.0f) sits %.2f m off road %d's own ground, but the next anchor is close enough that only %.0f m of ramp fits — %.0f m is needed at the %.0f%% cap. Move one of the two junctions apart, or lower this one."),
							C.Sampled[AnchorIdx].X, C.Sampled[AnchorIdx].Y, StepCm / 100.0, RoadIdx,
							MaxH / 100.0, (StepCm / MaxSlope) / 100.0, MaxSlope * 100.0);
					}
				}
			}

			const double Probe = FlatCm + MaxH;
			const double gIn   = (Probe > 1.0 && Sa - Probe >= 0.0) ? (Za - TangentAt(Sa - Probe)) / Probe : 0.0;
			const double gOut  = (Probe > 1.0 && Sa + Probe <= Len) ? (TangentAt(Sa + Probe) - Za) / Probe : 0.0;

			auto CurveLen = [CurveKCm, MaxH](double Grade)
			{
				return FMath::Clamp(CurveKCm * FMath::Abs(Grade) * 100.0, 0.0, MaxH);
			};
			const double hIn  = CurveLen(gIn);
			const double hOut = CurveLen(gOut);
			Reach[ai] = FlatCm + FMath::Max(hIn, hOut);

			for (int32 i = 0; i < N; ++i)
			{
				const double d  = S[i] - Sa;
				const double ad = FMath::Abs(d);
				if (ad <= FlatCm)
				{
					Z[i] = Za;
					continue;
				}
				const double h = (d < 0.0) ? hIn : hOut;
				if (h <= 1.0 || ad > FlatCm + h)
				{
					continue;
				}
				const double w = 1.0 - SmoothStep01((ad - FlatCm) / h);
				Z[i] = FMath::Lerp(Tangent[i], Za, w);
			}
		}

		// ---- write back; the edges are index-parallel to the centreline ------
		for (int32 i = 0; i < N; ++i) { C.Sampled[i].Z = Z[i]; }
		if (C.LeftEdge.Num()  == N) { for (int32 i = 0; i < N; ++i) { C.LeftEdge[i].Z  = Z[i]; } }
		if (C.RightEdge.Num() == N) { for (int32 i = 0; i < N; ++i) { C.RightEdge[i].Z = Z[i]; } }
		++RoadsAligned;

		// ---- record the untouchable stretch for the cross-tie pass -----------
		{
			TBitArray<> Prot(false, N);
			for (int32 i = 0; i < N; ++i)
			{
				if (Pinned[i]) { Prot[i] = true; continue; }
				for (int32 ai = 0; ai < Anchors.Num(); ++ai)
				{
					if (FMath::Abs(S[i] - Anchors[ai].Key) <= Reach[ai])
					{
						Prot[i] = true;
						break;
					}
				}
			}
			TieProtected.Add(RoadIdx, MoveTemp(Prot));
			TieArcS.Add(RoadIdx, S);
		}

		// ---- self-check: the profile must be its chord to within its budget ---
		for (int32 ai = 0; ai + 1 < Anchors.Num(); ++ai)
		{
			const double Sa = Anchors[ai].Key,     Za = Anchors[ai].Value;
			const double Sb = Anchors[ai + 1].Key, Zb = Anchors[ai + 1].Value;
			const double L  = FMath::Max(1.0, Sb - Sa);
			const double ChordGrade = FMath::Abs(Zb - Za) / L;
			if (ChordGrade > MaxSlope + 1e-4)
			{
				++SteepChords;
				UE_LOG(LogRoadNet, Warning,
					TEXT("[RoadNet][GRADE] road %d span %.0f-%.0f m: junctions differ by %.2f m over %.0f m = %.1f%%, past the %.1f%% cap. No profile beats a straight line between two fixed junctions — lower a junction instead."),
					RoadIdx, Sa / 100.0, Sb / 100.0, (Zb - Za) / 100.0, L / 100.0, ChordGrade * 100.0, MaxSlope * 100.0);
				continue;
			}
			const double Budget = MaxDevCm * FMath::Clamp((L - StraightCm) / (2.0 * StraightCm), 0.0, 1.0);
			double Worst = 0.0;
			for (int32 i = 0; i < N; ++i)
			{
				// Skip the plate + curve zone at either end: that is where the
				// profile is deliberately shaped away from the chord. The reach is
				// per anchor now that a big step buys a longer curve than LandingCm.
				if (S[i] <= Sa + Reach[ai] || S[i] >= Sb - Reach[ai + 1])
				{
					continue;
				}
				Worst = FMath::Max(Worst, FMath::Abs(Z[i] - FMath::Lerp(Za, Zb, (S[i] - Sa) / L)));
			}
			if (Worst > Budget + 1.0)
			{
				++BudgetBreaks;
				UE_LOG(LogRoadNet, Warning,
					TEXT("[RoadNet][GRADE] road %d span %.0f-%.0f m deviates %.0f cm from its chord; the budget for a %.0f m span is %.0f cm."),
					RoadIdx, Sa / 100.0, Sb / 100.0, Worst, L / 100.0, Budget);
			}
		}
	}

	// -----------------------------------------------------------------------
	// 4b. Lateral cross-tie: nearby beds may not disagree faster than 2°.
	// -----------------------------------------------------------------------
	// Two roads whose beds meet in one surface union (dual carriageways, the
	// apron where two arms overlap) must not disagree in Z faster than the
	// side-slope cap, or the union's Delaunay puts a wall between them — the
	// 90° cliff of the screenshots. Solved on the PROFILES: corrections can
	// raise and lower, are shared between the two roads, and are re-smoothed
	// longitudinally afterwards. Junction plates and vertical-curve zones are
	// protected — the plate levels were already negotiated above.
	const double TanLat = FMath::Tan(FMath::DegreesToRadians(FMath::Clamp(
		(double)CVarRoadNetMaxSideSlopeDeg.GetValueOnAnyThread(), 0.1, 45.0)));
	int32 TieViolations = 0, TieRemaining = 0;
	{
		// ponytail: ties reach at most one grid cell (26 m); beds further apart
		// than that are not a shared corridor. Upgrade path: derive the tie
		// radius from the zone unions instead of a constant.
		constexpr double kTieCellCm = 2600.0;
		constexpr double kTieGapCm  = 800.0;   // median/verge gap still counted as shared

		struct FTieEntry { int32 Road; int32 Idx; };
		TMultiMap<FIntPoint, FTieEntry> Grid;
		TMap<int32, double> HalfW;
		for (TPair<int32, FRoadCurves>& KV : Ctx.Curves)
		{
			const int32 r = KV.Key;
			if (!Roads.IsValidIndex(r) || !Roads[r].IsValid()
				|| Roads[r].bBridge || Roads[r].bTunnel)
			{
				continue;
			}
			HalfW.Add(r, FMath::Max(50.0, (double)Roads[r].Lanes.HalfWidthCm()));
			const TArray<FVector>& P = KV.Value.Sampled;
			for (int32 i = 0; i < P.Num(); ++i)
			{
				Grid.Add(FIntPoint(
					(int32)FMath::FloorToInt(P[i].X / kTieCellCm),
					(int32)FMath::FloorToInt(P[i].Y / kTieCellCm)), { r, i });
			}
		}

		auto IsProtected = [&TieProtected](int32 Road, int32 Idx) -> bool
		{
			const TBitArray<>* B = TieProtected.Find(Road);
			return B && B->IsValidIndex(Idx) && (*B)[Idx];
		};

		// One sweep: visit every close pair once (a < b by road index), pull the
		// two beds inside the allowed lateral wedge. Returns violations seen.
		TArray<FTieEntry> Bucket;
		auto Sweep = [&](bool bCorrect) -> int32
		{
			int32 Seen = 0;
			for (TPair<int32, FRoadCurves>& KV : Ctx.Curves)
			{
				const int32 r = KV.Key;
				const double* Hr = HalfW.Find(r);
				if (!Hr) { continue; }
				TArray<FVector>& P = KV.Value.Sampled;
				const int32 LayerR = Roads[r].Layer;
				for (int32 i = 0; i < P.Num(); ++i)
				{
					const int32 CX = (int32)FMath::FloorToInt(P[i].X / kTieCellCm);
					const int32 CY = (int32)FMath::FloorToInt(P[i].Y / kTieCellCm);
					for (int32 dx = -1; dx <= 1; ++dx)
					{
						for (int32 dy = -1; dy <= 1; ++dy)
						{
							Bucket.Reset();
							Grid.MultiFind(FIntPoint(CX + dx, CY + dy), Bucket);
							for (const FTieEntry& E : Bucket)
							{
								if (E.Road <= r) { continue; }   // each pair once
								const double* He = HalfW.Find(E.Road);
								if (!He || Roads[E.Road].Layer != LayerR) { continue; }
								FRoadCurves& CE = Ctx.Curves.FindChecked(E.Road);
								if (!CE.Sampled.IsValidIndex(E.Idx)) { continue; }
								FVector& Q = CE.Sampled[E.Idx];
								const double D = FVector::Dist2D(P[i], Q);
								if (D < 1.0 || D > *Hr + *He + kTieGapCm) { continue; }
								const double Allowed = TanLat * D + 1.0;
								const double dZ = P[i].Z - Q.Z;
								if (FMath::Abs(dZ) <= Allowed) { continue; }
								++Seen;
								if (!bCorrect) { continue; }
								const bool bProtP = IsProtected(r, i);
								const bool bProtQ = IsProtected(E.Road, E.Idx);
								if (bProtP && bProtQ) { continue; }   // two plates: joint pass's turf
								const double Excess = FMath::Abs(dZ) - Allowed;
								const double Sign   = (dZ > 0.0) ? 1.0 : -1.0;
								const double MoveP  = bProtP ? 0.0 : (bProtQ ? Excess : 0.5 * Excess);
								const double MoveQ  = bProtQ ? 0.0 : (bProtP ? Excess : 0.5 * Excess);
								P[i].Z -= Sign * MoveP;
								Q.Z    += Sign * MoveQ;
							}
						}
					}
				}
			}
			return Seen;
		};

		// A tie correction can break the longitudinal cap locally; re-clamp with
		// the protected stretch pinned, exactly like the main grade pass.
		auto ReclampLongitudinal = [&]()
		{
			for (TPair<int32, FRoadCurves>& KV : Ctx.Curves)
			{
				const TArray<double>* SPtr = TieArcS.Find(KV.Key);
				const TBitArray<>* Prot = TieProtected.Find(KV.Key);
				TArray<FVector>& P = KV.Value.Sampled;
				const int32 N = P.Num();
				if (!SPtr || !Prot || SPtr->Num() != N) { continue; }
				const TArray<double>& S = *SPtr;
				for (int32 i = 1; i < N; ++i)
				{
					if ((*Prot)[i]) { continue; }
					const double dS = FMath::Max(1.0, S[i] - S[i - 1]);
					P[i].Z = FMath::Clamp(P[i].Z, P[i - 1].Z - MaxSlope * dS, P[i - 1].Z + MaxSlope * dS);
				}
				for (int32 i = N - 2; i >= 0; --i)
				{
					if ((*Prot)[i]) { continue; }
					const double dS = FMath::Max(1.0, S[i + 1] - S[i]);
					P[i].Z = FMath::Clamp(P[i].Z, P[i + 1].Z - MaxSlope * dS, P[i + 1].Z + MaxSlope * dS);
				}
			}
		};

		TieViolations = Sweep(/*bCorrect*/true);
		if (TieViolations > 0)
		{
			constexpr int32 kTieIterations = 4;
			for (int32 It = 1; It < kTieIterations; ++It)
			{
				ReclampLongitudinal();
				if (Sweep(/*bCorrect*/true) == 0) { break; }
			}
			ReclampLongitudinal();
			TieRemaining = Sweep(/*bCorrect*/false);

			// The edges are index-parallel to the centreline: re-sync their Z.
			for (TPair<int32, FRoadCurves>& KV : Ctx.Curves)
			{
				FRoadCurves& C = KV.Value;
				const int32 N = C.Sampled.Num();
				if (C.LeftEdge.Num()  == N) { for (int32 i = 0; i < N; ++i) { C.LeftEdge[i].Z  = C.Sampled[i].Z; } }
				if (C.RightEdge.Num() == N) { for (int32 i = 0; i < N; ++i) { C.RightEdge[i].Z = C.Sampled[i].Z; } }
			}
		}
	}

	// -----------------------------------------------------------------------
	// 5. Junction grades (כרך 2 §8.2.3 and Table 8.1).
	// -----------------------------------------------------------------------
	// Two rules pointing in OPPOSITE directions, which is easy to get backwards:
	//
	//   Table 8.1 is the CEILING — an arm may not exceed 4% (6% below 80 km/h)
	//   through the junction, or turning traffic tips.
	//
	//   §8.2.3 is a FLOOR — «לא יפחת מ-1%», the junction's steepest direction
	//   must be at least 1% or water ponds on it. Its section is titled
	//   «שיפועים מזעריים» (minimum grades) and sits beside the 0.5% longitudinal
	//   minimum. The quantity is *named* "maximum resultant grade" because it is
	//   the grade in the steepest direction, not because it is a limit.
	const double MaxArmGrade = RoadNetStandards::MaxArmGradeAtJunction(
		/*worst case, the flattest allowance*/ 80);
	const double MinDrainage = RoadNetStandards::MinJunctionResultantGrade();

	int32 SteepJunctions = 0, PondingJunctions = 0, Reported = 0;

	for (const FRoadNetJoint& J : Ctx.Joints)
	{
		if (J.Arms.Num() < 3) { continue; }

		double AreaCm = 0.0;
		for (const FRoadNetJointArm& Arm : J.Arms) { AreaCm = FMath::Max(AreaCm, Arm.HalfWidthCm); }
		if (AreaCm < 1.0) { continue; }

		double Worst = 0.0;
		for (const FRoadNetJointArm& Arm : J.Arms)
		{
			const FRoadCurves* C = Ctx.Curves.Find(Arm.Road);
			if (!C || C->Sampled.Num() < 2) { continue; }

			const int32 Cnt  = C->Sampled.Num();
			const int32 From = Arm.bAtStart ? 0 : Cnt - 1;
			const int32 Step = Arm.bAtStart ? 1 : -1;

			double Walked = 0.0;
			for (int32 i = From; Walked < AreaCm; i += Step)
			{
				const int32 Nxt = i + Step;
				if (!C->Sampled.IsValidIndex(Nxt)) { break; }
				const double dS = FVector::Dist2D(C->Sampled[i], C->Sampled[Nxt]);
				if (dS < 1.0) { continue; }
				Walked += dS;

				const double Longitudinal = (C->Sampled[Nxt].Z - C->Sampled[i].Z) / dS;
				// Crossfall is zero in this model — the outer edges take the
				// centreline's Z — but the resultant is the quantity both rules
				// are written in, so it is spelled out for when crossfall lands.
				Worst = FMath::Max(Worst, RoadNetStandards::ResultantGrade(Longitudinal, 0.0));
			}
		}

		// The ceiling is the per-junction failure worth naming: it is caused by
		// terrain the alignment could not absorb, and it is fixable by moving or
		// re-levelling that junction.
		if (Worst > MaxArmGrade + 1e-4)
		{
			++SteepJunctions;
			if (Reported < 5)
			{
				++Reported;
				UE_LOG(LogRoadNet, Warning,
					TEXT("[RoadNet][GRADE] junction at (%.0f, %.0f) carries %.2f%% across its %.1f m area, past the %.0f%% arm limit (כרך 2 Table 8.1). Lower a neighbouring junction, or raise roadnet.MaxJunctionRelaxCm so this one can sink."),
					J.Location.X, J.Location.Y, Worst * 100.0, AreaCm / 100.0, MaxArmGrade * 100.0);
			}
		}
		if (Worst < MinDrainage) { ++PondingJunctions; }
	}

	// The drainage floor is reported as a COUNT, not per junction, because the
	// flat plate makes every junction violate it by construction — the warning
	// would be one line per junction saying the same thing about the design.
	//
	// ponytail: the plate is dead level. A junction that drains needs the plate
	// tilted to ~1% in a chosen direction (§8.2.3) with inlets on the upstream
	// kerb (§8.7). Until then this counter is a standing reminder, not a bug
	// report — and it is the reason a real drainage pass is still outstanding.
	if (PondingJunctions > 0)
	{
		UE_LOG(LogRoadNet, Log,
			TEXT("[RoadNet][GRADE] %d junction(s) are flatter than the %.0f%% drainage minimum (כרך 2 §8.2.3). Expected: the junction plate is level by design, so nothing sheds water off it yet."),
			PondingJunctions, MinDrainage * 100.0);
	}

	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet][GRADE] aligned %d road(s) over %d junction(s) | straight span %.0f m, chord budget %.1f m, relax %.0f cm, curve %.1f m/%% | %d steep chord(s), %d budget break(s), %d relax escape(s), %d over-graded junction(s) | %d junction(s) sit clear of their road's ground and were ramped, %d of those too tight to ramp fully | %d steep chord(s) negotiated (%d unfixable) | %d lateral tie(s) at %.1f%%, %d left after solve"),
		RoadsAligned, NumJoints, StraightCm / 100.0, MaxDevCm / 100.0, RelaxCm, CurveKCm / 100.0,
		SteepChords, BudgetBreaks, RelaxEscapes, SteepJunctions, StepsRamped, StepsTooTight,
		ChordViolations, ChordUnfixable, TieViolations, TanLat * 100.0, TieRemaining);

	// Snapshot for RoadNet.SlopeSelfCheck: the last rebuild's residual defects.
	GSlopeCheck = { ChordUnfixable, TieRemaining, StepsTooTight, SteepJunctions };
}

// ---------------------------------------------------------------------------
// RoadNet.SlopeSelfCheck — did the last vertical alignment leave any slope
// defect that will reach the mesh? Zero on every counter is the contract the
// latent solver makes: cliffs and >2° side slopes are fixed BEFORE meshing.
// ---------------------------------------------------------------------------
namespace
{
	FAutoConsoleCommand GSlopeSelfCheckCmd(
		TEXT("RoadNet.SlopeSelfCheck"),
		TEXT("Report the residual slope defects of the last RoadNet rebuild: steep chords the junction negotiation could not fix, lateral ties past the side-slope cap after the solve, junction ramps without room, and over-graded junction areas. All zero = PASS."),
		FConsoleCommandDelegate::CreateLambda([]()
		{
			const FRoadNetSlopeCheck& C = GSlopeCheck;
			const bool bOK = C.UnfixableChords == 0 && C.LateralResiduals == 0
				&& C.RampsTooTight == 0 && C.OverGradedJunctions == 0;
			UE_LOG(LogRoadNet, Display,
				TEXT("SlopeSelfCheck: %s | %d unfixable steep chord(s), %d lateral residual(s), %d ramp(s) without room, %d over-graded junction(s). Details are in the [RoadNet][GRADE] warnings of the last rebuild."),
				bOK ? TEXT("PASS") : TEXT("FAIL"),
				C.UnfixableChords, C.LateralResiduals, C.RampsTooTight, C.OverGradedJunctions);
		}));
}

// ---------------------------------------------------------------------------
// § street validation — the acceptance pass of the unified street plan.
// Validation, NOT mesh surgery: it walks what was actually committed and
// reports, with world positions, every triangle and edge that violates the
// contract the latent solver promised to enforce. With the solver healthy the
// three counters are zero; a non-zero counter is a regression with an address.
// ---------------------------------------------------------------------------
int32 URoadNetwork::ValidateStreet() const
{
	const TArray<FVector>& Verts = GetConformVerts();
	const TArray<int32>&   Tris  = GetConformTris();
	const TArray<FRoadNetDeformCorridor>& Corridors = GetDeformCorridors();
	if (Verts.Num() == 0 || Tris.Num() == 0 || Corridors.Num() == 0)
	{
		UE_LOG(LogRoadNet, Warning,
			TEXT("[RoadNet][VALIDATE] nothing to validate — rebuild the network first (the conform soup is transient)."));
		return 0;
	}

	const double LatTan = FMath::Tan(FMath::DegreesToRadians(FMath::Clamp(
		(double)CVarRoadNetMaxSideSlopeDeg.GetValueOnAnyThread(), 0.1, 45.0)));
	const double FlatCm = FMath::Max(0.0, ReadCVarFloat(TEXT("osm.RoadJunctionFlatCm"), 300.0));

	// ---- centreline segments in a grid, for "which way does the road run
	// here" — the lateral check needs a tangent to split the gradient against.
	struct FSeg { FVector2D A; FVector2D B; FVector2D Tan; };
	TArray<FSeg> Segs;
	constexpr double kCellCm = 3000.0;
	TMultiMap<FIntPoint, int32> Grid;
	for (const FRoadNetDeformCorridor& C : Corridors)
	{
		if (C.bBridge || C.bTunnel || C.Layer != 0) { continue; }
		for (int32 i = 0; i + 1 < C.Points.Num(); ++i)
		{
			const FVector2D A(C.Points[i].X, C.Points[i].Y);
			const FVector2D B(C.Points[i + 1].X, C.Points[i + 1].Y);
			const double L = FVector2D::Distance(A, B);
			if (L < 1.0) { continue; }
			const int32 Idx = Segs.Add({ A, B, (B - A) / L });
			Grid.Add(FIntPoint(
				(int32)FMath::FloorToInt(A.X / kCellCm),
				(int32)FMath::FloorToInt(A.Y / kCellCm)), Idx);
		}
	}
	TArray<int32> Bucket;
	auto NearestTangent = [&](const FVector2D& P, FVector2D& OutTan) -> bool
	{
		const int32 CX = (int32)FMath::FloorToInt(P.X / kCellCm);
		const int32 CY = (int32)FMath::FloorToInt(P.Y / kCellCm);
		double Best = 1e18;
		for (int32 dx = -1; dx <= 1; ++dx)
		{
			for (int32 dy = -1; dy <= 1; ++dy)
			{
				Bucket.Reset();
				Grid.MultiFind(FIntPoint(CX + dx, CY + dy), Bucket);
				for (const int32 Idx : Bucket)
				{
					const FSeg& S = Segs[Idx];
					const FVector2D AB = S.B - S.A;
					const double T = FMath::Clamp(
						FVector2D::DotProduct(P - S.A, AB) / FMath::Max(AB.SizeSquared(), 1.0), 0.0, 1.0);
					const double D = FVector2D::DistSquared(P, S.A + AB * T);
					if (D < Best) { Best = D; OutTan = S.Tan; }
				}
			}
		}
		return Best < 1e17;
	};

	// ---- junction plate centres: corridor endpoints welded, 3+ arms.
	// ponytail: O(n²) over endpoints — two per road, trivial counts.
	struct FCluster { FVector2D P; int32 Count = 0; };
	TArray<FCluster> Clusters;
	constexpr double kWeldCm = 600.0;
	for (const FRoadNetDeformCorridor& C : Corridors)
	{
		if (C.Points.Num() < 2 || C.Layer != 0) { continue; }
		for (const FVector& EndW : { C.Points[0], C.Points.Last() })
		{
			const FVector2D E(EndW.X, EndW.Y);
			bool bFound = false;
			for (FCluster& K : Clusters)
			{
				if (FVector2D::Distance(K.P, E) < kWeldCm)
				{
					K.P = (K.P * K.Count + E) / (K.Count + 1);
					++K.Count;
					bFound = true;
					break;
				}
			}
			if (!bFound) { Clusters.Add({ E, 1 }); }
		}
	}
	TArray<FVector2D> Plates;
	for (const FCluster& K : Clusters)
	{
		if (K.Count >= 3) { Plates.Add(K.P); }
	}

	// ---- walk the committed triangles.
	int32 LatBad = 0, PlateBad = 0, EdgeBad = 0, Reported = 0;
	auto Report = [&Reported](const TCHAR* What, const FVector& At, double Val)
	{
		if (Reported++ < 12)
		{
			UE_LOG(LogRoadNet, Warning, TEXT("[RoadNet][VALIDATE] %s at (%.0f, %.0f, %.0f): %.1f%%"),
				What, At.X, At.Y, At.Z, Val * 100.0);
		}
	};
	// A little headroom over the exact caps: the conform soup is a decimated
	// stand-in for the render mesh, so hairline overshoot is sampling, not sin.
	const double LatLimit   = LatTan * 1.25 + 0.005;
	const double PlateLimit = 0.02;   // a "flat" plate may carry 2% before it reads as tilted
	for (int32 t = 0; t + 2 < Tris.Num(); t += 3)
	{
		const FVector& A = Verts[Tris[t]];
		const FVector& B = Verts[Tris[t + 1]];
		const FVector& C = Verts[Tris[t + 2]];
		const FVector N = FVector::CrossProduct(B - A, C - A);
		if (N.SizeSquared() < 1.0) { continue; }
		const FVector Cen = (A + B + C) / 3.0;
		if (FMath::Abs(N.Z) < 1e-6)
		{
			++LatBad;   // a vertical face in a driving surface is always a defect
			Report(TEXT("vertical face"), Cen, 1.0);
			continue;
		}
		// Gradient of the triangle's plane: z rises by |G| per cm travelled.
		const FVector2D G(-N.X / N.Z, -N.Y / N.Z);
		const double Slope = G.Size();
		if (Slope < 0.0175) { continue; }   // < 1°: legal in every direction

		bool bOnPlate = false;
		for (const FVector2D& P : Plates)
		{
			if (FVector2D::Distance(P, FVector2D(Cen.X, Cen.Y)) < FlatCm + 200.0)
			{
				bOnPlate = true;
				break;
			}
		}
		if (bOnPlate)
		{
			if (Slope > PlateLimit)
			{
				++PlateBad;
				Report(TEXT("tilted junction plate"), Cen, Slope);
			}
			continue;
		}

		FVector2D Tan;
		if (!NearestTangent(FVector2D(Cen.X, Cen.Y), Tan)) { continue; }
		const double Along = FVector2D::DotProduct(G, Tan);
		const double Lat   = (G - Tan * Along).Size();
		if (Lat > LatLimit)
		{
			++LatBad;
			Report(TEXT("lateral slope"), Cen, Lat);
		}
	}

	// ---- kerb / skirt continuity: the plan sidewalk edge is the line a fence
	// or parcel meets; a vertical step along it is exactly the "skirt gap" the
	// screenshots showed. Legal Z change = the longitudinal ceiling over the
	// run, plus a kerb's worth of tolerance.
	for (const FRoadNetPlanEdge& E : PlanSidewalkEdges)
	{
		const int32 N = E.Points.Num();
		for (int32 i = 0; i < N; ++i)
		{
			const FVector& A = E.Points[i];
			const FVector& B = E.Points[(i + 1) % N];
			const double dXY = FMath::Max(1.0, FVector::Dist2D(A, B));
			const double dZ  = FMath::Abs(B.Z - A.Z);
			if (dZ > kMaxLongitudinalTan * dXY + 20.0)
			{
				++EdgeBad;
				Report(TEXT("sidewalk edge step"), A, dZ / dXY);
			}
		}
	}

	const int32 Total = LatBad + PlateBad + EdgeBad;
	UE_LOG(LogRoadNet, Display,
		TEXT("StreetValidate: %s | %d triangle(s) over the %.1f%% lateral cap, %d tilted plate triangle(s), %d sidewalk edge step(s) — over %d committed triangle(s), %d plate(s), %d plan edge ring(s).%s"),
		Total == 0 ? TEXT("PASS") : TEXT("FAIL"), LatBad, LatTan * 100.0, PlateBad, EdgeBad,
		Tris.Num() / 3, Plates.Num(), PlanSidewalkEdges.Num(),
		Reported > 12 ? TEXT(" (first 12 positions logged)") : TEXT(""));
	return Total;
}

// ---------------------------------------------------------------------------
// RoadNet.StreetValidate — run the street acceptance pass on every network in
// the world. The knob-tuning loop on a hero junction is: rebuild, run this,
// adjust osm.RoadJunctionFlatCm / LandingCm / roadnet.VerticalCurveKM /
// HeightBlendTauCm, repeat until PASS and the junction reads right.
// ---------------------------------------------------------------------------
namespace
{
	FAutoConsoleCommandWithWorld GStreetValidateCmd(
		TEXT("RoadNet.StreetValidate"),
		TEXT("Walk the committed road surface and the plan sidewalk edges of every RoadNet network and report slope-contract violations (lateral > roadnet.MaxSideSlopeDeg, tilted junction plates, sidewalk edge steps) with world positions. Zero everywhere = PASS."),
		FConsoleCommandWithWorldDelegate::CreateLambda([](UWorld* World)
		{
			if (!World) { return; }
			int32 Nets = 0;
			for (TActorIterator<ARoadNetActor> It(World); It; ++It)
			{
				if (URoadNetwork* Net = It->GetNetwork())
				{
					Net->ValidateStreet();
					++Nets;
				}
			}
			if (Nets == 0)
			{
				UE_LOG(LogRoadNet, Warning, TEXT("StreetValidate: no RoadNet actor in this world."));
			}
		}));
}
