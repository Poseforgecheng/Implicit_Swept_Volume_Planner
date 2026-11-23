# Three-level planner publication checks

Checked on 2026-11-23 in a new, isolated catkin workspace. The source repository
was symlinked into that workspace to test a non-fixed source-directory layout.
The existing physical-robot workspace was not rebuilt or launched.

## Completed checks

- Fresh Release build of all five catkin packages with GCC 9, ROS Noetic,
  system Python 3.8, and the supplied Linux x86-64 dependency binaries.
- C++ planning contracts: 13 tests, including forced R² failure semantics,
  forced-HIGH recovery, geometric versus solver labels, partial search chains,
  dense footprint checks, stitched tangents, and corner smoothing.
- C++ experiment/validation/serialization contracts: 20 tests.
- Python experiment helpers: 8 tests, including two added tests for executable
  lookup through sourced catkin prefixes instead of guessed parent directories.
- IR-SIM map/command conversion helpers: 5 tests.
- Passive recording and release-default contracts: 22 tests, including a real
  ROS message/rosbag serialization round trip without a master.
- ROS parameter expansion for standalone demo and the IR-SIM launch; confirmed
  default three-level allocation and disabled standalone local command output.
- Actual isolated planning smoke run: one empty-map task with each of
  `current_full`, `full_r2`, and `full_strict`. All three returned trajectories
  passed the independent executable gate. The Full-SVSDF run made a full-backend
  call; the R²-only run made none.
- Integrity checking of those three runs: zero errors and zero warnings.
- Full postprocessing and table export of smoke JSONL completed. Analysis ran
  in a Python environment containing the optional pandas/SciPy dependencies;
  those packages are listed in `planner_experiments/requirements.txt`.
- Staged whitespace check and review for build files, bags, secrets, paper
  files, and historical run records before publication.

These smoke results are functionality checks, not formal comparisons or new
paper results. Small test counts must not be used to claim statistical gains.

## Limits of this validation

- No new physical-robot motion or hardware drivers were started.
- The complete four-method IR-SIM execution benchmark and the large formal
  experiment matrix were not rerun for this source publication.
- ARM binaries, other ROS distributions, install-space deployment, and other
  operating systems were not built. The checked workflow uses a sourced devel
  space; bundled third-party binaries are x86-64.
- Existing PCL/VTK configuration and upstream libigl warnings are not proof of
  cross-platform compatibility. The Release build itself completed.
- Original workspaces, historical experiment data, bags, papers, and credential
  files are not included in the publication commit.
