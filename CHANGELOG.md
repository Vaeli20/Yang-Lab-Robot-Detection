# Changelog
<!--
All notable changes to this project are documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versioning follows [Semantic Versioning](https://semver.org/).

## Contributor - mm/dd/yyyy
 
Here we write upgrading notes to keep track of changes made (especially ones that are hard to pin point)
 
### Added
- MINOR title goes here.
 
### Changed
 
### Fixed

EX: 
---
## Jaelynn - 2017/03/15 

### Changed
- PATCH Drupal.org is now used for composer.
 
### Fixed
- PATCH Add logic to runsheet teaser delete to delete corresponding
  schedule cards.
-->
## Jaelynn - 11/6/2026 
- Tested Trjectory Camera Recorder. Files from the recordings were added. 

### Changed
- Added a set to 0 into `main.py` so that robot can be reset to original position.

---
## Jaelynn - 11/2/2026
### Changed
- Modified `robot_worker.py` function `_drain_commands()` exception line, ```python except Exception as e:```, to revert back to display `connect` if port doesn't connect
- Implemented camera recording in continous trajectory from `main.py`
- for both `main.py` and `camera.py` the camera section was changed to allow changing of camera in the python application and not <br>solified in the code. 

---
## Jaelynn - 10/30/2026
### Added
- `Trajectory Points` folder had trejectory points added to it from todays testing of point trajectories camera references. <br>**NOTE** worked as intended.
### Changed
- added minor changes to `main.py` to ensure point trajectory recorded images correctly and stored them inside the correct file location.<br>Along with adding minor spelling changes to port connection
- changed line 51 in `camera.py` to work for computer camera.  **NOTE** Will need to change to 1 for web camera.

### Removed
- removed `robot_worker 09-25-2026.py` file as it was a duplicate of current `robot_worker.py` file

---
## Jaelynn - 10/26/2026
### Changed
- changed `main.py` to tack screenshots and record while taking trajectory points.
- removed unneed extra lines in files
  - `robot_arm_gripper_Code`
    - `ArmDriver.py`
    - `camera.py`
    - `DM_CAN.py`
    - `filters.py`
    - `Gripper_test 09-25-2926.py`
    - `original_main.py`
    - `point_io.py`
    - `robot_worker.py`
    - `robot_worker 09-25-2026.py`
    - `trajectory_io.py`
  - `.gitignore`

### Fixed
- Fixed `main.py` trajectory point table to have proper header spacing and autosize when screen size is changed.

---
## Jaelynn - 10/23/2026
### Added
- `original_main.py` Initial main.py file from [Gitee Robot Arm](https://gitee.com/majormj/robot-arm-4340)
### Changed
- `main.py` modified python window size to have set min size and max size. Avoiding it going above what the screen can hold. <br> Along with fixing point header column size and having text from notes to wrap. All implemented in en or zh versions.

---
## Jaelynn - 10/22/2026
### Added
- `CHANGELOG.md` file created for initial set up for future documentation of changes made
- `brainstorm.md` created to keep track of ideas for repo/lab
### Changed
- `readme.md` modified to describe purpose of repository a bit more accurately
