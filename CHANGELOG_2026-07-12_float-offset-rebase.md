# 2026-07-12 -- Point-cloud float+offset rebasing for memory/bandwidth savings

## Motivation

The Master previously held the entire LAS point cloud in memory as `double` (24 bytes/point)
during ICP processing (`processIcpJob`) and BIM-distance processing (`loadBimPcChunks`) -- for a
plant-scale scan (hundreds of millions of points), this is a significant memory cost, and the
per-element chunks sent to Slaves over the network carried the same double precision.

Coordinates in this domain (site/plant surveying) are typically large absolute values (tens to
hundreds of meters, sometimes UTM-scale), which would lose sub-mm precision if stored directly as
`float`. The standard fix (used in CAD/GIS/game engines) is to rebase: subtract a representative
offset near the data so stored values stay close to zero, where `float`'s ~7 significant digits
are enough for sub-mm precision.

## What changed

- `IcpJob` (`include/IcpTypes.h`) gained `rebaseOffsetX/Y/Z` (double), computed once per job as
  the bounding-box center of the BIM elements (`icp::computeBimRebaseOffset`, new function in
  `include/bimtree/PseudoPointGenerator.hpp`). This is distinct from the pre-existing, user-supplied
  `offsetX/Y/Z` (a different, manual pre-shift feature).
- `processIcpJob()` (`MasterApp/MasterApplication_ICP.cpp`) now loads the BIM/GLTF data *before*
  the LAS file (previously the reverse), so the rebase offset can be computed before the point
  cloud is loaded.
- The Master's bulk, whole-point-cloud arrays (`sourcePoints` in `processIcpJob`, `pc` in
  `loadBimPcChunks`) are now stored as `std::vector<std::array<float,3>>`, shifted by the rebase
  offset -- roughly half the memory of the previous `double` bulk arrays.
- **`IcpChunk`, `BimPcChunk`, `IcpCore.hpp`'s ICP algorithm (`runIcp`), and
  `TaskManager.h`'s `processBimPc()` are completely unchanged** -- every individual chunk built
  from the bulk float array is widened back to `double` and un-shifted (offset added back) to
  absolute coordinates before being handed to the (already double, already network-serialized as
  double) chunk types. The actual ICP computation and BIM-distance computation never see shifted
  or float data; this keeps the delicate, freshly-stabilized ICP correspondence/SVD code (fixed
  earlier in this session's `IcpSerialization.hpp` work) completely untouched.
- Because every chunk is built in absolute coordinates, `job->coarseTransform` and
  `job->finalTransform` are naturally correct in the absolute frame -- no post-hoc correction is
  needed on the values reported via REST (`/api/icp/jobs/{id}/result`).
- A new helper `icp::toShiftedFrame(absoluteTransform, offsetX, offsetY, offsetZ)`
  (`include/IcpTypes.h`) converts an absolute-frame rigid transform into the equivalent transform
  for already-shifted points (`t_shifted = t_absolute - (I - R) * offset`; rotation is unchanged
  by a common translation). This is used at exactly the two points where an already-computed
  transform must be applied directly to the (float, shifted) bulk array without widening the
  whole array to double first:
  - Phase 5 (`alignedSource = transformPointCloudCopy(sourcePoints, coarseTransformShifted)`)
  - Phase 7, final aligned-LAS output (`alignedFullResSource = transformPointCloudCopy(sourcePoints, finalTransformShifted)`,
    with the offset added back per point when converting to `las::PointData` for writing -- a
    widen-to-double step that's required regardless, since the LAS writer needs double).
- New float-aware overloads/helpers added to `include/bimtree/IcpCore.hpp`:
  `transformPointCloudCopy(vector<array<float,3>>&, Transform4x4)`,
  `downsampleUniform(vector<array<float,3>>&, ratio)`, `widenToDouble()`, `narrowToFloat()`.
- `MasterApp/MasterApplication.h`/`.cpp`'s `loadIcpElementChunks()` signature changed to accept
  the float bulk array plus the offset (instead of a double array); `loadBimPcChunks()` similarly
  computes the offset and works from a float bulk array internally. Both still produce absolute,
  double `IcpChunk`/`BimPcChunk` output exactly as before.

## Verification

- Full solution rebuild (Release, x64, v145 toolset override): 0 errors.
- REST `/api/icp/start` on the icp_test sample (371,643 points, 1 BIM element, distributed to 1
  Slave): `coarse_rmse=0.0134833`, `final_rmse=0.0135168` -- identical to the pre-change baseline
  recorded earlier this session. Rebase offset computed as `(39.8225, -1.6635, -29.22)` (BIM bbox
  center). Output LAS bounds (`..._icp_aligned.las`) match the original input's bounds to within
  the expected ~1-2cm correction, confirming the output is in correct absolute coordinates (not
  left shifted).
- CLI `bimpc` distance calculation on the same sample: 371,643 points / 26,250 faces processed,
  distance stats (`min=0.0, max=0.1, avg=0.0`) and within/outside classification counts
  (`within=366460, outside=5183`) match the expected pre-change behavior.
- All 4 existing unit test suites pass (`TestLasImport`, `TestLaslibWrapper` x12,
  `TestGltfImport` x2, `TestTaskManager` x5) -- no regressions.
- Not yet re-verified at large scale (samsung.las, 3.4GB) in this pass -- the small-scale
  numeric match plus unchanged chunk/algorithm code gives high confidence, but actual peak-memory
  reduction on a large dataset has not been measured.
