# LiDAR Place Recognition and Landmark Localization — Literature Survey

**Date:** 2026-08-16
**Scope:** Methods for fixing robot pose from LiDAR alone, for a Husky UGV with an Ouster
OS0-64, in GPS-denied/spoofed conditions, **without semantic classification**.

**Every paper below was retrieved and verified in this session.** Where a claim could not be
confirmed against a fetched source, it is marked as such.

---

## 0. Scope correction — this changed the conclusion

The original brief described a flat park (0.007 m relief, ground ≈ 83% of every scan) and
asked for per-object landmark recognition with ground removed as a preprocessing step.

**The corrected brief states the target environment has real terrain relief — hills, slopes,
banks, dips, shorelines — and the park is only one test map.** This inverts two premises:

1. **Ground removal is no longer a neutral prerequisite; it is destructive.** On terrain with
   relief the ground carries the most distinctive signal available. A ridge line or a lake
   shore is far more discriminative than the 24th identical tree.
2. **Terrain is not an object.** It is continuous, underlies everything, and extends past
   sensor range. It cannot be clustered into a blob, so any "segment discrete objects,
   describe each one" pipeline structurally cannot represent it.

The ranked recommendation in Section 7 reflects the corrected framing. Section 6 states the
central negative finding.

---

## 1. Global (whole-scan) descriptors

### 1.1 Scan Context — **top recommendation**

> G. Kim and A. Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition
> Within 3D Point Cloud Map," *IEEE/RSJ IROS 2018*, pp. 4802–4809.
> DOI: [10.1109/IROS.2018.8593953](https://doi.org/10.1109/IROS.2018.8593953)

*Citation verified from two independent fetched sources (the ScanContext++ / DSC reference
lists). The IEEE Xplore page itself is paywalled and would not render.*

**Core idea.** Partition the scan into a polar grid of `Ns` azimuthal sectors × `Nr` radial
rings around the sensor. Each bin stores **the maximum height of the points falling in it**.
The result is a 2D matrix (an image) that is a direct, non-histogram record of the visible 3D
structure. No training, no prior learning.

**Terrain encoding — decisive for this project.** Because the bin value *is* max height as a
function of (bearing, range), Scan Context encodes elevation structure natively. A rising bank
shows up as a monotonic height ramp along the radial axis; a dip shows as a trough; a shoreline
shows as a height discontinuity. **Terrain shape is signal, not noise, and no ground/object
decision is ever made.** This is exactly the property the corrected brief asks for.

**Rotation invariance — how, precisely.** Robot yaw maps to a pure **circular column shift** of
the matrix. Two mechanisms exploit this:
- **Ring key** — a rotation-invariant vector (one value per ring, invariant to column order)
  used for fast kNN retrieval via KD-tree.
- **Column-shift alignment** — for each candidate, all column shifts are tried; the best score
  gives both the similarity and, as a bonus, **an estimate of the relative yaw**.

This is genuine yaw invariance by construction, not an approximation. That directly answers
requirement 2 of the brief (re-recognition from a different heading).

**Data requirements.** Range + height only. **No intensity** (safe: ours is constant zero). No
training data. No initial pose guess.

**Known failure modes.**
- **Lateral translation sensitivity.** The descriptor is egocentric — viewing the same place
  from a laterally displaced position changes the polar binning and degrades the match.
  Addressed by Scan Context++ (below).
- **Requires height variation.** DSC (Sec. 1.4) states Scan Context "is based on the hypothesis
  of structured scenes and may fail if there is no significant variation in height." *On the
  flat park this is a real risk; on the terrain-rich target maps the relief supplies exactly
  the variation it needs.* The scope correction makes Scan Context stronger, not weaker.
- Max-height-only discards structure below the top surface.

### 1.2 Scan Context++

> G. Kim, S. Choi, and A. Kim, "Scan Context++: Structural Place Recognition Robust to
> Rotation and Lateral Variations in Urban Environments," *IEEE Transactions on Robotics*, 2022.
> [Semantic Scholar](https://www.semanticscholar.org/paper/18d8175e7b6edaa1b5488431498f84efe162face)

Adds two subdescriptors (polar and Cartesian context) to gain robustness to **lateral
translation** as well as rotation, and provides 1-DoF semi-metric localization — bridging
topological retrieval and metric pose. Directly patches Scan Context's main weakness.

*Note: the search-result summary of the abstract is consistent across sources, but I could not
fetch a full text (no open arXiv version found under the IDs tried). Treat the internal detail
as less firmly verified than Scan Context itself.*

### 1.3 RING++ — **strong second recommendation**

> X. Xu, S. Lu, J. Wu, H. Lu, Q. Zhu, Y. Liao, R. Xiong, Y. Wang, "RING++: Roto-translation
> Invariant Gram for Global Localization on a Sparse Scan Map," *IEEE T-RO*, vol. 39, 2023.
> arXiv: [2210.05984](https://arxiv.org/abs/2210.05984) · DOI: 10.1109/TRO.2023.3303035

**Core idea.** Build a BEV representation, apply a **Radon transform** to get a sinogram, then a
row-wise 1D **Fourier magnitude** to get a translation-invariant "TING", and use circular
cross-correlation for rotation. Yields a representation invariant to **both** rotation and
translation, with theoretical guarantees.

**Why it matters here.** It solves *all* subtasks — place recognition **and full 3-DoF metric
pose** (1-DoF rotation + 2-DoF planar translation, optionally ICP-refined to 6-DoF) — and it is
explicitly **"the first learning-free framework"** to do so. That satisfies requirement 3
(metric pose fix) without a training set.

**Terrain encoding.** Operates on multi-channel BEV features (occupancy plus geometric features
such as change-of-curvature, omnivariance, eigenvalue-entropy). Height/occupancy structure is
retained, so terrain relief contributes. **No intensity required** — verified explicitly.

**Failure mode (stated).** "Due to the finite scan range occlusion, the invariance of RING
cannot be guaranteed when translation is significant with respect to the scan range occlusion."
Degrades when displacement approaches sensor range.

### 1.4 M2DP — reportable but not recommended

> L. He, X. Wang, and H. Zhang, "M2DP: A Novel 3D Point Cloud Descriptor and Its Application
> in Loop Closure Detection," *IEEE/RSJ IROS 2016*, pp. 231–237.
> DOI: [10.1109/IROS.2016.7759060](https://doi.org/10.1109/IROS.2016.7759060)
> *Citation verified from the VNI-Net reference list (fetched).*

Projects the cloud onto multiple 2D planes at varying azimuth/elevation, builds a density
signature per plane (concentric circles + bins), stacks them, and takes the **first left and
right singular vectors from an SVD** as a compact descriptor.

**Rotation invariance is the weak point.** It is obtained by shifting to the centroid and
defining a reference frame from the **two dominant PCA directions**. PCA axes are unstable when
the scene's dominant directions are ambiguous or when occlusion changes which structure is
visible — precisely the "different viewpoint" case we care about. This is a fragile,
data-dependent frame rather than Scan Context's structural column shift.

**Also:** density signatures compress away height detail, so terrain relief is represented far
less directly than in Scan Context. No intensity needed, no training.

### 1.5 ESF / VFH-style global descriptors — ruled out

Classical PCL global shape descriptors were designed for **isolated, fully-observed, densely
sampled objects** (tabletop CAD-scale recognition), not for partially observed outdoor scans.
They assume the object is already segmented, which reintroduces exactly the object/ground
decision the corrected brief forbids. Note the ensemble-of-shape-functions idea does reappear
inside SegMatch as a *segment* descriptor (Sec. 2.1).

---

## 2. Segment-based and local keypoint pipelines

### 2.1 SegMatch — technically excellent, but structurally wrong for terrain

> R. Dubé, D. Dugas, E. Stumm, J. Nieto, R. Siegwart, C. Cadena, "SegMatch: Segment Based
> Place Recognition in 3D Point Clouds," *IEEE ICRA 2017*.
> arXiv: [1609.07720](https://arxiv.org/abs/1609.07720) · DOI: 10.1109/ICRA.2017.7989618

**Verified pipeline** (from the fetched full text):
1. **Ground plane removal**, then **Euclidean clustering** ("Cluster-All", 0.2 m max voxel
   distance). Segments kept at **100–15,000 points**.
2. **Descriptors:** 7-D eigenvalue features (linearity, planarity, scattering, omnivariance,
   anisotropy, eigenentropy, curvature change) plus a 640-D ensemble of shape histograms
   (D2/D3/A3 shape functions).
3. **Matching:** a **random forest** (25 trees) classifies segment pairs.
4. **Geometric verification:** **RANSAC on segment centroids** → 6-DoF transform.

**What it gets right for the original brief.** No intensity. **No semantic labels.** It
explicitly does not assume "perfect segmentation" or "the existence of 'objects'". The final
pose comes from the *spatial arrangement of centroids*, not from any single segment's identity —
which is the correct way to beat perceptual aliasing (see Sec. 6).

**Why the scope correction demotes it.** The pipeline **begins with ground removal and Euclidean
clustering**. Both steps are fatal on terrain-rich maps:
- A hillside, bank, or shoreline **cannot be clustered into a segment** — it is continuous and
  runs past the sensor horizon. There is no blob to extract.
- Removing the ground *deletes the most distinctive structure in the map* before matching begins.
- What survives clustering in a park is trees, benches, lamps — i.e. **exactly the mutually
  aliased repeated objects** (23 trees, 16 benches, 15 lamps).

So SegMatch would discard the good signal and retain the ambiguous signal. **The segment-based
family fundamentally excludes terrain features.** This answers the coordinator's central
question directly: yes, and it rules the family out as the primary method for terrain-rich
environments.

### 2.2 SegMap

> R. Dubé, A. Cramariuc, D. Dugas, H. Sommer, M. Dymczyk, J. Nieto, R. Siegwart, C. Cadena,
> "SegMap: Segment-based mapping and localization using data-driven descriptors,"
> *IJRR* 2020 / RSS 2018. arXiv: [1804.09557](https://arxiv.org/abs/1804.09557),
> [1909.12837](https://arxiv.org/abs/1909.12837)

Replaces SegMatch's handcrafted descriptor with a learned one: **incremental region growing**
segmentation, then a CNN over a **32×32×16 binary voxel grid** (three 3D conv layers + two FC)
producing a **64-D** descriptor; kNN retrieval + centroid geometric consistency.

**Requires training data:** KITTI sequences 05–06, ~3,300 segments, correspondences labelled
automatically **using GPS ground truth**. We have neither a labelled dataset nor usable GPS
(that is the threat model). Rotation handling is only partial — 2D PCA alignment to gravity plus
**rotation data augmentation**, i.e. learned approximate invariance, not structural invariance.

Inherits SegMatch's terrain blindness *plus* adds a training requirement. **Ruled out.**

### 2.3 SHOT, FPFH, 3DSC, NARF, ISS — ruled out on point density

> F. Tombari, S. Salti, L. Di Stefano, "Unique Signatures of Histograms for Local Surface
> Description," *ECCV 2010*, pp. 356–369.
> R. B. Rusu, N. Blodow, M. Beetz, "Fast Point Feature Histograms (FPFH) for 3D Registration,"
> *IEEE ICRA 2009*, pp. 3212–3217.

These describe a small neighbourhood around a keypoint using a local reference frame (SHOT) or
surface-normal angle histograms (FPFH). Rotation invariance comes from the local reference
frame — which is itself estimated from the local points and becomes unstable when those points
are few or noisy.

**Verified failure mode:** the literature is explicit that these degrade badly on sparse data —
"they do not perform well at sparse space locations," and "due to the sparsity of the LiDAR
point cloud, the statistical methods may fail to generate effective feature representation when
there are not enough points." FPFH, SHOT and RoPS are also reported "sensitive to Gaussian
noise," with performance "greatly affected" by reduced surface resolution.

**Our data:** a structure at 20–40 m returns only **dozens of points**. Normal estimation and
local reference frames are unreliable there. These were designed for dense CAD/RGB-D scans.
**Ruled out.**

### 2.4 STD (Stable Triangle Descriptor) — ruled out on vegetation

> C. Yuan, J. Lin, Z. Zou, X. Hong, F. Zhang, "STD: Stable Triangle Descriptor for 3D Place
> Recognition," *IEEE ICRA 2023*. arXiv: [2209.12435](https://arxiv.org/abs/2209.12435)

Elegant idea: extract keypoints, form triangles from keypoint triples, describe each by its
three side lengths + normal dot products. **A triangle's shape is exactly invariant to rigid
transformation**, so this is true rotation invariance, training-free, and intensity-free.

**But its keypoints come from voxel-based plane detection and plane-boundary extraction.** The
paper's own stated limitation: "Our method performs poorly only when the structure or planes of
the scene are particularly sparse because the key points extracted in such scenes will be
scarce," with acknowledged failure in vegetation-heavy environments and degraded performance on
park datasets with dense trees. A park/natural-terrain map has almost no planes. **Ruled out**,
though its *triangle-invariant* principle is worth borrowing (Sec. 6).

---

## 3. Learned descriptors — all ruled out (no training data)

| Method | Citation | Why ruled out |
|---|---|---|
| **PointNetVLAD** | M. A. Uy, G. H. Lee, *CVPR 2018*. arXiv [1804.03492](https://arxiv.org/abs/1804.03492) | Needs a labelled place-retrieval benchmark; makes **no rotation-invariance claim** |
| **OverlapNet** | X. Chen, T. Läbe, A. Milioto, T. Röhling, O. Vysotska, A. Haag, J. Behley, C. Stachniss, *RSS 2020*. arXiv [2105.11344](https://arxiv.org/abs/2105.11344) | **Requires intensity and semantics** for its good results |
| **LoGG3D-Net** | K. Vidanapathirana, M. Ramezani, P. Moghadam, S. Sridharan, C. Fookes, *ICRA 2022*, pp. 2215–2221. arXiv [2109.08336](https://arxiv.org/abs/2109.08336) | Sparse-conv U-Net trained on KITTI/MulRan; no rotation-invariance claim |
| **LPD-Net** | graph-based neighbourhood modelling (per survey) | Training required |

**OverlapNet deserves a specific note**, because its yaw mechanism is otherwise ideal: a range
image is a cyclic projection, so **yaw rotation = horizontal column shift**, and a
translation-equivariant FCN plus circular correlation recovers yaw at 1° resolution. The same
insight as Scan Context, applied to range images.

However, the verified ablation is disqualifying for us: OverlapNet takes **four channels —
range, normals, intensity/remission, and semantic class probabilities** (from RangeNet++).
Depth alone gives "reasonable overlap prediction but **poor yaw estimation**"; semantics drop
mean yaw error from 2.53° to 1.13°. We have **no intensity** (constant zero) and **semantics are
forbidden by the brief**. That strips OverlapNet to its weakest configuration and still leaves
a training requirement.

**General caution, verified from the global-localization survey:** despite >95% Recall@1 on
benchmarks, "there still remain several challenges… e.g., generalization ability"; learned
descriptors suffer "generalization failures" under domain shift. A network trained on KITTI
urban driving has no reason to transfer to a Gazebo park with a simulated OS0-64.

---

## 4. Registration-based methods — the metric back-end

These do not *recognize* a place; they *align* clouds and return a pose. They are the natural
second stage after a descriptor narrows the candidates.

### 4.1 NDT

> P. Biber and W. Straßer, "The Normal Distributions Transform: A New Approach to Laser Scan
> Matching," *IEEE/RSJ IROS 2003*. DOI: 10.1109/IROS.2003.1249285
>
> M. Magnusson, "The Three-Dimensional Normal-Distributions Transform — an Efficient
> Representation for Registration, Surface Analysis, and Loop Detection," PhD thesis,
> Örebro University, 2009.

Subdivides space into cells and fits a **normal distribution per cell**, giving a smooth,
piecewise-continuous probability field. Because the objective is smooth and analytically
differentiable, standard optimizers (Newton's method) apply — **no explicit point
correspondences needed**, unlike ICP.

**Terrain handling — a genuine advantage.** NDT models *surfaces as distributions*, so a sloped
bank or ridge is represented directly as an oriented Gaussian. It makes **no object/ground
distinction whatsoever**, which fits the corrected brief perfectly.

**Requirement:** a reasonable **initial guess**. It is a local optimizer with a limited
convergence basin.

### 4.2 ICP variants

Iteratively pairs nearest points and minimizes distance. **Verified limitation from the
survey:** "ICP and its variants might fall into local minima, making it inapplicable for global
registration." Go-ICP achieves global optimality via branch-and-bound but is "time-consuming on
resource-constrained platforms." ICP with partial overlap needs careful outlier rejection.

### 4.3 NDT-MCL

> J. Saarinen, H. Andreasson, T. Stoyanov, A. J. Lilienthal, "Normal Distributions Transform
> Monte-Carlo Localization (NDT-MCL)," *IEEE/RSJ IROS 2013*, pp. 382–389.

Uses the NDT representation as the **observation model inside a particle filter**. Reported to
outperform standard grid-based MCL and to approach commercial infrastructure-based positioning
in accuracy/repeatability for industrial AGVs.

**Why this is the right back-end architecture here.** A particle filter over an NDT map:
- fuses a *sequence* of scans, so a single ambiguous scan cannot cause a wrong fix — the
  direct antidote to perceptual aliasing (Sec. 6);
- makes no object/ground decision, so terrain contributes fully;
- needs no training and no intensity;
- naturally consumes the robot's existing wheel/IMU odometry as the motion model.

Its weakness — needing initialization — is exactly what Scan Context or RING++ supplies.

---

## 5. Ground segmentation — **reframed: use terrain, don't delete it**

Per the scope correction, these are reported **not** as a preprocessing step to adopt, but so
the trade-off is explicit.

> M. Himmelsbach, F. v. Hundelshausen, H.-J. Wuensche, "Fast Segmentation of 3D Point Clouds
> for Ground Vehicles," *IEEE Intelligent Vehicles Symposium (IV) 2010*, pp. 560–565.

Splits segmentation into **local ground plane estimation** followed by **2D connected-components
labelling**. Fast, line-fit based, no intensity. This is essentially the front-end SegMatch uses.

> H. Lim, M. Oh, H. Myung, "Patchwork: Concentric Zone-based Region-wise Ground Segmentation
> with Ground Likelihood Estimation Using a 3D LiDAR Sensor," *IEEE RA-L*, vol. 6, no. 4,
> pp. 6458–6465, 2021. arXiv: [2108.05560](https://arxiv.org/abs/2108.05560)

Concentric Zone Model + region-wise plane fitting + Ground Likelihood Estimation. **>40 Hz.**
Handles slopes and bumpy roads by fitting *per region* rather than globally.

> S. Lee, H. Lim, H. Myung, "Patchwork++: Fast and Robust Ground Segmentation Solving Partial
> Under-Segmentation Using 3D Point Cloud," *IEEE/RSJ IROS 2022*.
> arXiv: [2207.11919](https://arxiv.org/abs/2207.11919)

Adds A-GLE, Temporal Ground Revert, Region-wise Vertical Plane Fitting, and Reflected Noise
Removal. **54.85 Hz** on an i7-7700K.

**Two caveats that matter for us:**
1. **Patchwork++'s RNR step requires intensity** — verified: it filters points with "lower
   intensity than the noise removal intensity threshold, I_noise" (0.2). **Our intensity is
   constant zero, so RNR is inoperable** and would misbehave if left enabled (every point looks
   like noise by the intensity test). Patchwork (v1) or Himmelsbach avoid this dependency.
2. **Using any of these to *delete* ground throws away the localization signal** on terrain-rich
   maps.

**Recommended reframing.** If ground points are separated at all, do it to **label** them, not
to discard them — then feed the terrain surface into a 2.5D height map (Sec. 5.1) or into NDT
cells as oriented Gaussians. Patchwork's region-wise plane fits are themselves a compact
description of local terrain slope and are usable *as features*.

**Methods that depend on ground removal to function at all** — and are therefore penalized under
the corrected brief: **SegMatch, SegMap**, and any Euclidean-clustering object pipeline. Scan
Context, RING++, NDT and NDT-MCL require **no** ground removal.

### 5.1 Terrain / elevation-map matching — the family the original brief missed

> M. Werner, D. Čapek, T. Musil, O. Franěk, T. Báča, M. Saska, "Kilometer-Scale GNSS-Denied UAV
> Navigation via Heightmap Gradients: A Winning System from the SPRIN-D Challenge," arXiv, 2025.
> arXiv: [2510.01348](https://arxiv.org/abs/2510.01348)

This is a **directly analogous GNSS-denied problem** solved by terrain matching, and it is the
closest published match to the corrected brief.

**Method (verified):** build a local heightmap from LiDAR by taking **max height in 1 m bins**
(the same primitive as Scan Context) over a 40×40 m local window; match it against a prior
heightmap by **gradient-based template matching**; feed the resulting similarity map into a
**clustered particle filter** alongside odometry and compass.

**Why gradients, not raw heights** — the stated rationale transfers exactly: absolute height is
unreliable because barometry is noisy and "the ground might be sloped, making it impossible to
obtain absolute height." **Gradient matching gives bias/offset invariance** — it compares terrain
*shape*, not absolute elevation. They further keep only gradients above a threshold to emphasize
"tall, stable structures such as buildings and trees, while discarding small or transient
objects."

**Heading is handled outside the matcher:** a **compass** north-aligns the local heightmap before
correlation. **We have `/compass/data`, an absolute non-drifting yaw source** (per CLAUDE.md), so
this decomposition is directly available to us and removes the need for rotation search.

**Verified failure mode — important and honest:** performance was "better… in urban environment,
where there were more features," whereas "in open field areas, the system mostly relied on the
odometry." **Flat, featureless terrain starves the method.** This is the mirror image of the park
problem: terrain matching is strong exactly where the corrected brief says the target maps are
(relief) and weak exactly where the park test map is (0.007 m).

Related background confirming the family: terrain-aided navigation builds a live DEM and matches
it to a stored reference DEM to correct accumulated error.

---

## 6. The central negative finding

**Viewpoint-invariant recognition of a *single object* from geometry alone, without semantics,
is not how this problem is solved in the literature — and for terrain-rich maps it is the wrong
frame entirely.** Three independent lines of evidence, all from fetched sources:

**(a) The field works at whole-scan level, not per-object level.** The global-localization
survey's taxonomy is built entirely around whole-scan/submap descriptors and full-cloud
registration. Its assessment: LiDAR gives "textureless" data in an "irregular format" requiring
*holistic* compression. Even the segmentation-based methods "rely on full-scan segmentation,
showing the field views objects as context within whole scenes, not independent cues."

**(b) The information simply is not there in one object.** A single scan sees one side of a
non-symmetric object; at 20–40 m that side is dozens of points. Local descriptors are documented
to fail at exactly this density (Sec. 2.3). And with 23 identical trees, 16 identical benches and
15 identical lamps, **even a perfect descriptor cannot disambiguate instances** — they are
geometrically identical by construction. Perceptual aliasing here is not a tuning problem; it is
an identifiability limit. The survey lists repetitive-structure aliasing as an **open problem**,
noting keyframe maps "may not be suitable… in certain environments, such as indoor or forested
areas where many local environments are similar."

**(c) What actually carries the identity is the *arrangement*, not the object.** Every successful
"landmark-ish" method resolves ambiguity with **inter-landmark geometry**, which is what is
genuinely viewpoint invariant:
- SegMatch: RANSAC over **segment centroids** — individual segment descriptors only propose;
  the *constellation* disposes.
- STD: **triangle side lengths**, exactly invariant under rigid motion.
- GLARE: "inter-word distances being **viewpoint invariant**," encoding geometric relations
  between landmarks into a pose-invariant histogram.

> **Conclusion.** Do not try to build a viewpoint-invariant descriptor for one tree. Individual
> objects in this scene are *deliberately non-identifiable*. Identity lives in the whole-scan
> structure — terrain shape plus the spatial arrangement of everything visible. Under the
> corrected brief this is doubly true, because the single most distinctive structure in a
> terrain-rich map (the ground surface itself) **cannot be expressed as an object at all**.
> Per-object landmark recognition is the wrong frame.

---

## 7. Ranked recommendation

### #1 — Scan Context (or Scan Context++) for recognition, NDT for the metric fix

**Why it wins under the corrected brief.**
- Its bin value is **max height per (bearing, range) cell** — the same primitive the SPRIN-D
  terrain system uses. **Terrain relief is encoded natively.** Ridges, banks, dips and shorelines
  become descriptor structure.
- **No ground/object decision is ever made** — nothing to segment, nothing to delete.
- **Yaw invariance is structural** (ring key + column shift) and returns a **yaw estimate** as a
  by-product.
- No intensity, no training, no initial guess. Every hard constraint satisfied.
- The one documented weakness — "may fail if there is no significant variation in height" — is
  **removed by the scope correction**. Relief is what it wants.

**Caveat to plan for:** it is sensitive to lateral displacement, so map entries must be sampled
densely enough along the route, or use Scan Context++.

**Pipeline:** Scan Context retrieval → top-k candidates → **NDT** alignment to a stored submap
for the 6-DoF metric fix (Scan Context's column shift supplies NDT's initial yaw).

### #2 — RING++, if a single self-contained method is preferred

Learning-free, intensity-free, **roto-translation invariant with theoretical guarantees**, and it
outputs a **full 3-DoF metric pose** by itself — recognition and pose fix in one framework, with
optional ICP refinement. Strictly more capable than Scan Context on the translation axis. It is
#2 only because it is a heavier implementation (Radon + Fourier + multi-channel BEV) and less
widely field-replicated. Watch its stated limit: invariance degrades when displacement is large
relative to scan range.

### #3 — Terrain/heightmap-gradient matching in an NDT-MCL-style particle filter

The most direct answer to "use terrain as the localization signal," and a proven GNSS-denied
architecture.
- Build a local max-height heightmap; match **gradients** (bias/offset invariant) against a
  prior heightmap extracted from the world model.
- **North-align with `/compass/data`** exactly as the SPRIN-D system uses its compass, removing
  rotation search entirely.
- Run it as the observation model in a **particle filter** (NDT-MCL pattern), fusing odometry.
  A filter over a *sequence* is the structural fix for perceptual aliasing — no single ambiguous
  scan can force a wrong fix.

**Honest limitation:** this degrades toward odometry-only in flat, featureless areas — verified
in the source. **On the 0.007 m park it will contribute almost nothing; on the terrain-rich
target maps it should be the strongest single cue.** Recommend it as a fused input, never as the
sole source.

### Explicitly ruled OUT

| Method | Reason |
|---|---|
| **SegMatch / SegMap** | Depend on **ground removal + Euclidean clustering**; terrain cannot be clustered, so they delete the best signal and keep the aliased objects. SegMap additionally needs training data labelled with GPS ground truth. |
| **SHOT, FPFH, 3DSC, NARF, ISS** | Documented failure on sparse clouds; dozens of points at 20–40 m cannot support stable normals/local reference frames. |
| **STD** | Keypoints require planar structure; author-stated poor performance in vegetation and park scenes. |
| **PointNetVLAD, LPD-Net, LoGG3D-Net** | No training data, no labelled dataset; documented generalization failure under domain shift. |
| **OverlapNet** | Best results require **intensity** (ours is constant zero) **and semantics** (forbidden); plus training. |
| **ESF / VFH** | Assume isolated, densely-sampled, pre-segmented objects. |
| **Patchwork++ RNR step** | Its noise removal thresholds on **intensity**; inoperable with constant-zero intensity. Use Patchwork v1 or Himmelsbach if ground *labelling* is needed. |
| **Bare ICP for global fix** | Local minima make it "inapplicable for global registration" without an initial guess. Fine as a refinement stage only. |

---

## 8. Practical notes for this codebase

- **Keep `/compass/data` in the loop.** Both the #1 and #3 recommendations benefit: it supplies
  an absolute, non-drifting yaw, letting the descriptor stage spend its budget on position.
- **Do not remove the ground** in the localization path. If ground points are separated, label
  them and keep them as terrain surface.
- **Sample the map densely along the route.** Scan Context's lateral sensitivity is the main
  practical failure mode; map-entry spacing is the mitigation.
- **The park map is a weak test for terrain methods.** With 0.007 m relief, recommendation #3
  will look broken there while being correct for the target environment. Judge it on a
  terrain-rich map, or the test will mislead.
- **Ring field is populated** → range-image reconstruction is available, which is what the
  Scan Context / OverlapNet cyclic-shift yaw trick relies on.

---

## References

1. Kim, G., Kim, A. "Scan Context: Egocentric Spatial Descriptor for Place Recognition Within 3D Point Cloud Map." IROS 2018, 4802–4809. https://doi.org/10.1109/IROS.2018.8593953
2. Kim, G., Choi, S., Kim, A. "Scan Context++: Structural Place Recognition Robust to Rotation and Lateral Variations in Urban Environments." IEEE T-RO, 2022. https://www.semanticscholar.org/paper/18d8175e7b6edaa1b5488431498f84efe162face
3. Xu, X., Lu, S., Wu, J., Lu, H., Zhu, Q., Liao, Y., Xiong, R., Wang, Y. "RING++: Roto-translation Invariant Gram for Global Localization on a Sparse Scan Map." IEEE T-RO 39, 2023. https://arxiv.org/abs/2210.05984
4. He, L., Wang, X., Zhang, H. "M2DP: A Novel 3D Point Cloud Descriptor and Its Application in Loop Closure Detection." IROS 2016, 231–237. https://doi.org/10.1109/IROS.2016.7759060
5. Dubé, R., Dugas, D., Stumm, E., Nieto, J., Siegwart, R., Cadena, C. "SegMatch: Segment Based Place Recognition in 3D Point Clouds." ICRA 2017. https://arxiv.org/abs/1609.07720
6. Dubé, R., Cramariuc, A., Dugas, D., Sommer, H., Dymczyk, M., Nieto, J., Siegwart, R., Cadena, C. "SegMap: Segment-based mapping and localization using data-driven descriptors." IJRR 2020 / RSS 2018. https://arxiv.org/abs/1804.09557 · https://arxiv.org/abs/1909.12837
7. Tombari, F., Salti, S., Di Stefano, L. "Unique Signatures of Histograms for Local Surface Description." ECCV 2010, 356–369.
8. Rusu, R. B., Blodow, N., Beetz, M. "Fast Point Feature Histograms (FPFH) for 3D Registration." ICRA 2009, 3212–3217.
9. Yuan, C., Lin, J., Zou, Z., Hong, X., Zhang, F. "STD: Stable Triangle Descriptor for 3D Place Recognition." ICRA 2023. https://arxiv.org/abs/2209.12435
10. Uy, M. A., Lee, G. H. "PointNetVLAD: Deep Point Cloud Based Retrieval for Large-Scale Place Recognition." CVPR 2018. https://arxiv.org/abs/1804.03492
11. Chen, X., Läbe, T., Milioto, A., Röhling, T., Vysotska, O., Haag, A., Behley, J., Stachniss, C. "OverlapNet: Loop Closing for LiDAR-based SLAM." RSS 2020. https://arxiv.org/abs/2105.11344
12. Vidanapathirana, K., Ramezani, M., Moghadam, P., Sridharan, S., Fookes, C. "LoGG3D-Net: Locally Guided Global Descriptor Learning for 3D Place Recognition." ICRA 2022, 2215–2221. https://arxiv.org/abs/2109.08336
13. Biber, P., Straßer, W. "The Normal Distributions Transform: A New Approach to Laser Scan Matching." IROS 2003. https://doi.org/10.1109/IROS.2003.1249285
14. Magnusson, M. "The Three-Dimensional Normal-Distributions Transform — an Efficient Representation for Registration, Surface Analysis, and Loop Detection." PhD thesis, Örebro University, 2009.
15. Saarinen, J., Andreasson, H., Stoyanov, T., Lilienthal, A. J. "Normal Distributions Transform Monte-Carlo Localization (NDT-MCL)." IROS 2013, 382–389.
16. Himmelsbach, M., von Hundelshausen, F., Wuensche, H.-J. "Fast Segmentation of 3D Point Clouds for Ground Vehicles." IEEE Intelligent Vehicles Symposium 2010, 560–565.
17. Lim, H., Oh, M., Myung, H. "Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor." IEEE RA-L 6(4), 6458–6465, 2021. https://arxiv.org/abs/2108.05560
18. Lee, S., Lim, H., Myung, H. "Patchwork++: Fast and Robust Ground Segmentation Solving Partial Under-Segmentation Using 3D Point Cloud." IROS 2022. https://arxiv.org/abs/2207.11919
19. Werner, M., Čapek, D., Musil, T., Franěk, O., Báča, T., Saska, M. "Kilometer-Scale GNSS-Denied UAV Navigation via Heightmap Gradients: A Winning System from the SPRIN-D Challenge." arXiv 2025. https://arxiv.org/abs/2510.01348
20. Yin, H., et al. "A Survey on Global LiDAR Localization: Challenges, Advances and Open Problems." arXiv 2302.07433. https://arxiv.org/abs/2302.07433
21. Luo, K., Yu, H., Chen, X., Yang, Z., Wang, J., Cheng, P., Mian, A. "3D point cloud-based place recognition: a survey." Artificial Intelligence Review, 2024. https://doi.org/10.1007/s10462-024-10713-6

### Unverified — could not confirm full text in this session
- **Scan Context++** — citation details consistent across multiple search sources and confirmed
  present in other papers' reference lists, but no open full text was retrievable; internal
  method detail is from abstracts/summaries only.
- **Himmelsbach et al. (IV 2010)**, **Tombari et al. (ECCV 2010)**, **Rusu et al. (ICRA 2009)**,
  **Biber & Straßer (IROS 2003)**, **Magnusson (2009)**, **Saarinen et al. (IROS 2013)** —
  bibliographic details confirmed from indexed catalogue records; full texts were not fetchable
  (paywalled or server unreachable). Method descriptions rest on secondary sources.
- **Luo et al. survey (AI Review 2024)** — Springer redirected to an auth wall; cited for
  existence and bibliographic detail only, not relied on for any claim above.
- **LPD-Net** — described only via the survey's characterization; primary paper not fetched, so
  no independent citation is asserted.
