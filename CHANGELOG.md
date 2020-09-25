## Aikido 0 (prerelease)

### 0.5.0 (2020-XX-XX)

* Planner

  * Fixed key bugs in CRRTs: [#574](https://github.com/personalrobotics/aikido/pull/574)
  * Make planner_dart component and build all DART planners as planner_dart component: [#567](https://github.com/personalrobotics/aikido/pull/567)

### 0.4.0 (2020-08-27)

* Planner

  * Defined post-processor parameter structs in AIKIDO: [#579](https://github.com/personalrobotics/aikido/pull/579)
  * Deleted old post-processor interface: [#581](https://github.com/personalrobotics/aikido/pull/581)

### 0.3.0 (2020-05-22)

* Common

  * Fixed bug in StepSequence::getMaxSteps(): [#305](https://github.com/personalrobotics/aikido/pull/305)
  * Fixed bug in StepSequence iterator: [#320](https://github.com/personalrobotics/aikido/pull/320)
  * Cleaned up doxygen errors: [#357](https://github.com/personalrobotics/aikido/pull/357)
  * Fixed bug in compiling with Boost 1.58 on Kinetic + Xenial: [#490](https://github.com/personalrobotics/aikido/pull/490)
  * Fixed bug in Interpolated::addWaypoint(): [#483](https://github.com/personalrobotics/aikido/pull/483)
  * Fixed bug in VanDerCorput sequence generator to handle non-unit span: [#552](https://github.com/personalrobotics/aikido/pull/552)
  * Updated to C++14 Standard: [#570](https://github.com/personalrobotics/aikido/pull/570)

* Distance

  * Added methods to rank configurations based on specified metric: [#423](https://github.com/personalrobotics/aikido/pull/423)
  * Added weights as optional parameter to rankers: [#484](https://github.com/personalrobotics/aikido/pull/484)
  * Updated constness of return type of SE2::getStateSpace(): [#536](https://github.com/personalrobotics/aikido/pull/536)

* State Space

  * Refactored JointStateSpace and MetaSkeletonStateSpace: [#278](https://github.com/personalrobotics/aikido/pull/278)
  * Added methods for checking compatibility between DART objects and state spaces: [#315](https://github.com/personalrobotics/aikido/pull/315)
  * Added flags to MetaSkeletonStateSaver to specify what to save: [#339](https://github.com/personalrobotics/aikido/pull/339)
  * Fixed bug in the representation of SO(2) statespace: [#408](https://github.com/personalrobotics/aikido/pull/408)
  * Fixed const correctness of StateHandle::getState(): [#419](https://github.com/personalrobotics/aikido/pull/419)
  * Fixed hidden compose function (in-place version): [#421](https://github.com/personalrobotics/aikido/pull/421)
  * Added clone functionality to StateSpace: [#422](https://github.com/personalrobotics/aikido/pull/422)
  * Used const StateSpaces everywhere: [#429](https://github.com/personalrobotics/aikido/pull/429)
  * Changed the SE(2) space representation to [x,y,theta]: [#458](https://github.com/personalrobotics/aikido/pull/458)

* Constraint

  * Added SequentialSampleable: [#393](https://github.com/personalrobotics/aikido/pull/393)
  * Changed deprecated DART call in InverseKinematics: [#524](https://github.com/personalrobotics/aikido/pull/524)

* Control

  * Fixed CollisionGroup bugs in Hand executors: [#299](https://github.com/personalrobotics/aikido/pull/299)
  * Rewrote executors for faster-than-realtime simulation: [#316](https://github.com/personalrobotics/aikido/pull/316), [#450](https://github.com/personalrobotics/aikido/pull/450)
  * Introduced uniform and dart namespaces: [#342](https://github.com/personalrobotics/aikido/pull/342)
  * Removed Barrett-specific hand executors: [#380](https://github.com/personalrobotics/aikido/pull/380)
  * Supported canceling in-progress trajectories: [#400](https://github.com/personalrobotics/aikido/pull/400)

* Perception

  * Added integrated PoseEstimatorModule: [#336](https://github.com/personalrobotics/aikido/pull/336)
  * Added voxel grid perception module: [#448](https://github.com/personalrobotics/aikido/pull/448)
  * Added YAML communication between PoseEstimatorModule and third-party perception algorithms: [#497](https://github.com/personalrobotics/aikido/pull/497)
  * Made AssetDatabase key required for Marker text: [#508](https://github.com/personalrobotics/aikido/pull/508)

* Trajectory

  * Added B-spline trajectory: [#453](https://github.com/personalrobotics/aikido/pull/453)
  * Added trajectory utility functions: [#462](https://github.com/personalrobotics/aikido/pull/462)
  * Fixed toR1JointTrajectory to copy Waypoints with their time information: [#510](https://github.com/personalrobotics/aikido/pull/510)
  * Removed incorrect Spline to Interpolated conversions: [#511](https://github.com/personalrobotics/aikido/pull/511)
  * Updated findTimeOfClosestStateTrajectory to use StateSpace Distance Metric: [#543](https://github.com/personalrobotics/aikido/pull/543)
  * Add SO2 handling in spline conversions: [#546](https://github.com/personalrobotics/aikido/pull/546)

* Planner

  * Check that all planning problems only hold ScopedStates: [#569](https://github.com/personalrobotics/aikido/pull/569)
  * Make all DART planners take MetaSkeleton, and add adapter for turning planners into DART planners: [#437](https://github.com/personalrobotics/aikido/pull/437)
  * Added parabolic timing for linear spline [#302](https://github.com/personalrobotics/aikido/pull/302), [#324](https://github.com/personalrobotics/aikido/pull/324)
  * Fixed step sequence iteration in VFP: [#303](https://github.com/personalrobotics/aikido/pull/303)
  * Refactored planning API: [#314](https://github.com/personalrobotics/aikido/pull/314), [#414](https://github.com/personalrobotics/aikido/pull/414), [#426](https://github.com/personalrobotics/aikido/pull/426), [#432](https://github.com/personalrobotics/aikido/pull/432)
  * Added flags to WorldStateSaver to specify what to save: [#339](https://github.com/personalrobotics/aikido/pull/339)
  * Changed interface for TrajectoryPostProcessor: [#341](https://github.com/personalrobotics/aikido/pull/341)
  * Planning calls with InverseKinematicsSampleable constraints explicitly set MetaSkeleton to solve IK with: [#379](https://github.com/personalrobotics/aikido/pull/379)
  * Added a kinodynamic timer that generates a time-optimal smooth trajectory without completely stopping at each waypoint: [#443](https://github.com/personalrobotics/aikido/pull/443)
  * Fixed segmentation fault on 32-bit machines in vector-field planner: [#459](https://github.com/personalrobotics/aikido/pull/459)
  * Updated interface to OMPL planners to follow the style of the new refactored planning API: [#466](https://github.com/personalrobotics/aikido/pull/466)
  * Added convenience function for converting SO(2) trajectories to R1 trajectories, support for postprocessing SO(2) trajectories: [#496](https://github.com/personalrobotics/aikido/pull/496)
  * Used ConfigurationRanker in TSR planners: [#503](https://github.com/personalrobotics/aikido/pull/503)
  * Returned Interpolated trajectories from VFP: [#513](https://github.com/personalrobotics/aikido/pull/513)
  * Added support for taking state snapshots with list of skeleton names: [#523](https://github.com/personalrobotics/aikido/pull/523)

* Robot

  * Added Robot, Manipulator, Hand interfaces, and ConcreteRobot, ConcreteManipulator classes: [#325](https://github.com/personalrobotics/aikido/pull/325), [#392](https://github.com/personalrobotics/aikido/pull/392)
  * Added Kunz timer to Robot class: [#505](https://github.com/personalrobotics/aikido/pull/505)
  * Made ConcreteRobot limit functions public: [#556](https://github.com/personalrobotics/aikido/pull/556)

* RViz

  * Fixed bug of not joining Viewer threads when stopping auto-update: [#463](https://github.com/personalrobotics/aikido/pull/463)
  * Fixed bug of not passing full file path to RViz when MeshShape is used: [#518](https://github.com/personalrobotics/aikido/pull/518)
  * Merged WorldInteractiveMarkerViewer into InteractiveMarkerViewer, removing the former: [#520](https://github.com/personalrobotics/aikido/pull/520)
  * Fixed bug of not removing SkeletonMarkers of skeletons removed from the associated World: [#560](https://github.com/personalrobotics/aikido/pull/560)

* IO

  * Added loadSkeletonFromURDF convenience function: [#388](https://github.com/personalrobotics/aikido/pull/388), [#401](https://github.com/personalrobotics/aikido/pull/401)
  * Added methods for caching and retrieving trajectories: [#541](https://github.com/personalrobotics/aikido/pull/541)

* Build & Testing & ETC

  * Fixed Eigen memory alignment issues on 32bit Ubuntu: [#368](https://github.com/personalrobotics/aikido/pull/368)
  * Defined optional dependencies: [#376](https://github.com/personalrobotics/aikido/pull/376)
  * Fixed compilation bug with Eigen 3.3.5: [#452](https://github.com/personalrobotics/aikido/pull/452)
  * Updated gtest version to 1.8.1: [#478](https://github.com/personalrobotics/aikido/pull/478)
  * Added DART 6.7 support: [#480](https://github.com/personalrobotics/aikido/pull/480)
  * Fixed use of dart::common::make_unique for C++14 enabled compilers: [#481](https://github.com/personalrobotics/aikido/pull/481)
  * Changed to use own build script for GoogleTest: [#485](https://github.com/personalrobotics/aikido/pull/485)
  * Added aikido::common::make_unique wrapper: [#532](https://github.com/personalrobotics/aikido/pull/532)
  * Manually generated (minimal) Python bindings: [#499](https://github.com/personalrobotics/aikido/pull/499)

### 0.2.0 (2018-01-09)

* State Space

  * Moved MetaSkeletonStateSpaceSaver implementation to src: [#273](https://github.com/personalrobotics/aikido/pull/273)

* Constraint

  * Added methods for removing groups from NonColliding constraints: [#247](https://github.com/personalrobotics/aikido/pull/247)
  * Renamed NonColliding to CollisionFree: [#256](https://github.com/personalrobotics/aikido/pull/256)
  * Added TestableOutcome class: [#266](https://github.com/personalrobotics/aikido/pull/266)

* Control

  * Added Instantaneous and Queued TrajectoryExecutors: [#259](https://github.com/personalrobotics/aikido/pull/259)

* Perception

  * Added RcnnPoseModule: [#264](https://github.com/personalrobotics/aikido/pull/264)

* Planner

  * Added World class: [#243](https://github.com/personalrobotics/aikido/pull/243), [#252](https://github.com/personalrobotics/aikido/pull/252), [#265](https://github.com/personalrobotics/aikido/pull/265)
  * Added vector field planner: [#246](https://github.com/personalrobotics/aikido/pull/246), [#262](https://github.com/personalrobotics/aikido/pull/262), [#268](https://github.com/personalrobotics/aikido/pull/268)

* RViz

  * Added WorldInteractiveMarkerViewer: [#242](https://github.com/personalrobotics/aikido/pull/242)
  * Added TrajectoryMarker: [#288](https://github.com/personalrobotics/aikido/pull/288)

* IO

  * Added support for new ErrorStr API in tinyxml2 6.0.0: [#290](https://github.com/personalrobotics/aikido/pull/290), [#295](https://github.com/personalrobotics/aikido/pull/295)

* Build & Testing & ETC

  * Changed to use size_t over std::size_t: [#230](https://github.com/personalrobotics/aikido/pull/230)
  * Included test code to formatting code list: [#239](https://github.com/personalrobotics/aikido/pull/239)
  * Fixed RViz component dependencies: [#253](https://github.com/personalrobotics/aikido/pull/253)

### [0.1.0 (2017-10-02)](https://github.com/personalrobotics/aikido/milestone/4?closed=1)

* Common

  * Added VanDerCorput::getLength(): [#223](https://github.com/personalrobotics/aikido/pull/223)
  * Added ExecutorThread and ExecutorMultiplexer: [#151](https://github.com/personalrobotics/aikido/pull/151)
  * Changed splitEngine() to set default numOutputs to 1: [#214](https://github.com/personalrobotics/aikido/pull/214)

* State Space

  * Added MetaSkeletonStateSpaceSaver: [#184](https://github.com/personalrobotics/aikido/pull/184), [#219](https://github.com/personalrobotics/aikido/pull/219)
  * Added WeldJoint: [#146](https://github.com/personalrobotics/aikido/pull/146)
  * Improved real vector space classes to take advantage of Eigen's fixed-size objects: [#159](https://github.com/personalrobotics/aikido/pull/159)

* Constraint

  * Added SE2BoxConstraint: [#135](https://github.com/personalrobotics/aikido/pull/135)
  * Added satisfiable tolerance parameter to TSR constructor: [#180](https://github.com/personalrobotics/aikido/pull/180)
  * Added ConstantSampler: [#146](https://github.com/personalrobotics/aikido/pull/146)
  * Renamed SO2Sampleable to SO2UniformSampler: [#224](https://github.com/personalrobotics/aikido/pull/224)

* Perception

  * Added simulation world env to ApriltagsModule: [#156](https://github.com/personalrobotics/aikido/pull/156)

* Planner

  * Added parabolic smoother: [#206](https://github.com/personalrobotics/aikido/pull/206)
  * Added wrapper for OMPL path simplifier: [#164](https://github.com/personalrobotics/aikido/pull/164)
  * Added support OMPL 1.2.0 and above: [#139](https://github.com/personalrobotics/aikido/pull/139)
  * Changed to SnapePlanner to return nullptr when planning fails: [#201](https://github.com/personalrobotics/aikido/pull/201)
  * Refactored parabolic timer: [#206](https://github.com/personalrobotics/aikido/pull/206)

* IO

  * Added KinBody parser: [#102](https://github.com/personalrobotics/aikido/pull/102)
  * Improved yaml extension: [#175](https://github.com/personalrobotics/aikido/pull/175), [#200](https://github.com/personalrobotics/aikido/pull/200)

* RViz

  * Added TSR visualizer: [#136](https://github.com/personalrobotics/aikido/pull/136)
  * Made FrameId a variable: [#183](https://github.com/personalrobotics/aikido/pull/183)
  * Fixed inconsistent function signature: [#222](https://github.com/personalrobotics/aikido/pull/222)

* Control

  * Added toSplineJointTrajectory() that can controls only specifying joints: [#217](https://github.com/personalrobotics/aikido/pull/217)
  * Added RosPositionCommandExecutor: [#173](https://github.com/personalrobotics/aikido/pull/173)
  * Added BarretthandPositionCommandExecutor: [#166](https://github.com/personalrobotics/aikido/pull/166)
  * Added RosJointStateClient: [#150](https://github.com/personalrobotics/aikido/pull/150)
  * Added trajectory conversions between Aikido's and ROS's: [#147](https://github.com/personalrobotics/aikido/pull/147), [#149](https://github.com/personalrobotics/aikido/pull/149)
  * Improved RosTrajectoryExecutor to use helper function for waiting for action server: [#192](https://github.com/personalrobotics/aikido/pull/192)
  * Improved (~)Executor::step() not to lock: [#182](https://github.com/personalrobotics/aikido/pull/182)
  * Fixed renaming sping to step: [#189](https://github.com/personalrobotics/aikido/pull/189)
  * Fixed to work on real hardware: [#216](https://github.com/personalrobotics/aikido/pull/216)
  * Removed StateSpace from RosTrajectoryExecutor: [#190](https://github.com/personalrobotics/aikido/pull/190)

* Build & Testing & ETC

  * Added Findinteractive_markers.cmake: [#198](https://github.com/personalrobotics/aikido/pull/198)
  * Improved to build without warnings: [#114](https://github.com/personalrobotics/aikido/pull/114), [#143](https://github.com/personalrobotics/aikido/pull/143)
  * Improved to enforce to follow code formatting: [#153](https://github.com/personalrobotics/aikido/pull/153), [#167](https://github.com/personalrobotics/aikido/pull/167), [#172](https://github.com/personalrobotics/aikido/pull/172), [#168](https://github.com/personalrobotics/aikido/pull/168), [#170](https://github.com/personalrobotics/aikido/pull/170), [#171](https://github.com/personalrobotics/aikido/pull/171), [#191](https://github.com/personalrobotics/aikido/pull/191), [#207](https://github.com/personalrobotics/aikido/pull/207), [#211](https://github.com/personalrobotics/aikido/pull/211)
  * Switched to codecov for online code coverage reporting: [#208](https://github.com/personalrobotics/aikido/pull/208)
  * Increased the required minimum version of DART to 6.2: [#210](https://github.com/personalrobotics/aikido/pull/210)
  * Changed to test Aikido for both of release and debug modes, and on macOS: [#195](https://github.com/personalrobotics/aikido/pull/195)
  * Splitted util namespace into common and io: [#225](https://github.com/personalrobotics/aikido/pull/225)
  * Fixed build issues on macOS: [#138](https://github.com/personalrobotics/aikido/pull/138)

### 0.0.1 (2017-03-10)

  * Initial release
