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
#include "RoadNetMath.h"
#include "RoadNetLog.h"
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
	const double MaxSlope   = FMath::Max(0.005, ReadCVarFloat(TEXT("osm.RoadGradeMaxSlope"),     0.12));

	// -----------------------------------------------------------------------
	// 1. Junction elevations — "highest Z wins" over every arriving road.
	// -----------------------------------------------------------------------
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
	TArray<double> JointZ, JointKing;
	TBitArray<>    JointRelaxable(false, NumJoints);
	JointZ.SetNumZeroed(NumJoints);
	JointKing.SetNumZeroed(NumJoints);

	// Road end -> joint. Keyed by an encoded (road, end) so the map hashes ints.
	auto ArmKey = [](int32 RoadIdx, bool bStart) { return RoadIdx * 2 + (bStart ? 0 : 1); };
	TMap<int32, int32> ArmToJoint;
	for (int32 j = 0; j < NumJoints; ++j)
	{
		const FRoadNetJoint& J = Ctx.Joints[j];
		double King = -TNumericLimits<double>::Max();
		for (const TPair<int32, bool>& Arm : J.Arms)
		{
			King = FMath::Max(King, ArmEndZ(Arm.Key, Arm.Value));
			ArmToJoint.Add(ArmKey(Arm.Key, Arm.Value), j);
		}
		if (J.Arms.Num() == 0)
		{
			King = J.Z;
		}
		JointKing[j]      = King;
		JointZ[j]         = King;
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
	// clamped so a junction is never lifted above its king Z and never sinks more
	// than the budget below it.
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
				JointZ[j] = FMath::Clamp(Acc[j] / W[j], JointKing[j] - RelaxCm, JointKing[j]);
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

	for (int32 j = 0; j < NumJoints; ++j)
	{
		if (JointZ[j] > JointKing[j] + 1e-3 ||
			(JointRelaxable[j] && JointZ[j] < JointKing[j] - RelaxCm - 1e-3))
		{
			++RelaxEscapes;
		}
	}

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
		const double MinGap = FMath::Max(1.0, 2.0 * FlatCm);
		TArray<TPair<double, double>> MidAnchors;
		if (const TArray<TPair<double, double>>* Ints = Interior.Find(RoadIdx))
		{
			for (const TPair<double, double>& It : *Ints)
			{
				if (It.Key > MinGap && It.Key < Len - MinGap)
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

		TArray<TPair<double, double>> Anchors;
		Anchors.Reserve(MidAnchors.Num() + 2);
		Anchors.Emplace(0.0, JStart ? JointZ[*JStart] : C.Sampled[0].Z);
		Anchors.Append(MidAnchors);
		Anchors.Emplace(Len, JEnd ? JointZ[*JEnd] : C.Sampled.Last().Z);

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

		for (int32 ai = 0; ai < Anchors.Num(); ++ai)
		{
			const double Sa = Anchors[ai].Key;
			const double Za = Anchors[ai].Value;

			// Neighbouring anchors bound the reach so two curves never overlap.
			double MaxH = LandingCm;
			if (ai > 0)                 { MaxH = FMath::Min(MaxH, 0.5 * (Sa - Anchors[ai - 1].Key)); }
			if (ai + 1 < Anchors.Num()) { MaxH = FMath::Min(MaxH, 0.5 * (Anchors[ai + 1].Key - Sa)); }
			MaxH = FMath::Max(0.0, MaxH - FlatCm);

			const double Probe = FlatCm + MaxH;
			const double gIn   = (Probe > 1.0 && Sa - Probe >= 0.0) ? (Za - TangentAt(Sa - Probe)) / Probe : 0.0;
			const double gOut  = (Probe > 1.0 && Sa + Probe <= Len) ? (TangentAt(Sa + Probe) - Za) / Probe : 0.0;

			auto CurveLen = [CurveKCm, MaxH](double Grade)
			{
				return FMath::Clamp(CurveKCm * FMath::Abs(Grade) * 100.0, 0.0, MaxH);
			};
			const double hIn  = CurveLen(gIn);
			const double hOut = CurveLen(gOut);

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
				// profile is deliberately shaped away from the chord.
				if (S[i] <= Sa + FlatCm + LandingCm || S[i] >= Sb - FlatCm - LandingCm)
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

	UE_LOG(LogRoadNet, Log,
		TEXT("[RoadNet][GRADE] aligned %d road(s) over %d junction(s) | straight span %.0f m, chord budget %.1f m, relax %.0f cm, curve %.1f m/%% | %d steep chord(s), %d budget break(s), %d relax escape(s)"),
		RoadsAligned, NumJoints, StraightCm / 100.0, MaxDevCm / 100.0, RelaxCm, CurveKCm / 100.0,
		SteepChords, BudgetBreaks, RelaxEscapes);
}
