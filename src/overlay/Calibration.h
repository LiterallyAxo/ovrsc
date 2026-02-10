#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Windows.h>
#include <openvr.h>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <deque>
#include <cstdint>

#include "Protocol.h"

enum class CalibrationState
{
	None,
	Begin,
	Rotation,
	Translation,
	Editing,
	CaptureExtrinsic,
	Continuous,
	ContinuousStandby,
};

struct StandbyDevice {
	std::string trackingSystem;
	std::string model, serial;
};

struct TrackingWorldPair
{
	Eigen::AffineCompact3d T_world_ref = Eigen::AffineCompact3d::Identity();
	Eigen::AffineCompact3d T_world_target = Eigen::AffineCompact3d::Identity();
};

struct RigidMountExtrinsic
{
	Eigen::AffineCompact3d T_ref_tracker_mount = Eigen::AffineCompact3d::Identity();
	double rotationConfidence = 0.0;
	double translationConfidence = 0.0;
	double timestamp = 0.0;
	uint32_t sampleCount = 0;
	bool calibrated = false;
};

struct CalibrationContext
{
	static constexpr int ProfileSchemaVersion = 2;

	enum class RuntimeMode {
		Current,
		Legacy,
	};

	CalibrationState state = CalibrationState::None;
	int32_t referenceID = -1, targetID = -1;

	static const size_t MAX_CONTROLLERS = 8;
	int32_t controllerIDs[MAX_CONTROLLERS];

	StandbyDevice targetStandby, referenceStandby;

	Eigen::Vector3d calibratedRotation;
	Eigen::Vector3d calibratedTranslation;
	double calibratedScale;

	std::string referenceTrackingSystem;
	std::string targetTrackingSystem;

	bool enabled = false;
	bool validProfile = false;
	bool clearOnLog = false;
	bool quashTargetInContinuous = false;
	double timeLastTick = 0, timeLastScan = 0, timeLastAssign = 0, timeLastAlignment = 0;
	bool ignoreOutliers = false;
	double wantedUpdateInterval = 1.0;
	float jitterThreshold = 3.0f;
	uint64_t continuousFrameCounter = 0;
	uint64_t lastAlignmentFrame = 0;
	uint32_t alignmentPeriodFrames = 300;

	bool requireTriggerPressToApply = false;
	bool wasWaitingForTriggers = false;
	bool hasAppliedCalibrationResult = false;

	float xprev, yprev, zprev;

	float continuousCalibrationThreshold;
	float maxRelativeErrorThreshold = 0.005f;
	Eigen::Vector3d continuousCalibrationOffset;

	protocol::AlignmentSpeedParams alignmentSpeedParams;
	bool enableStaticRecalibration;
	bool lockRelativePosition = false;
	bool useLegacyDynamicSolver = false;
	bool lockedExtrinsic = false;
	float lockedExtrinsicQuality = 0.0f;
	int alignmentPeriodFrames = 0;
	RuntimeMode runtimeMode = RuntimeMode::Current;
	bool lockedExtrinsicNeedsRecapture = false;
	double extrinsicCaptureRms = 0.0;
	double extrinsicCaptureVariance = 0.0;
	int extrinsicCaptureSampleCount = 0;
	std::string extrinsicCaptureDate;
	bool enableLockedExtrinsicPeriodicPath = false;

	RigidMountExtrinsic rigidMountExtrinsic;
	int extrinsicSampleTarget = 500;
	float extrinsicMinMotionDiversity = 0.03f;
	float extrinsicMinAxisExcitationDeg = 20.0f;
	float extrinsicMaxRmsResidual = 0.02f;
	float extrinsicMinRotationalSpreadDeg = 20.0f;
	float extrinsicMaxTranslationVariance = 0.0004f;

	Eigen::AffineCompact3d refToTargetPose = Eigen::AffineCompact3d::Identity();
	bool relativePosCalibrated = false;

	enum Speed
	{
		FAST = 0,
		SLOW = 1,
		VERY_SLOW = 2
	};
	Speed calibrationSpeed = FAST;

	vr::DriverPose_t devicePoses[vr::k_unMaxTrackedDeviceCount];

	CalibrationContext() {
		calibratedScale = 1.0;
		memset(devicePoses, 0, sizeof(devicePoses));
		ResetConfig();
	}

	void ResetConfig() {
		alignmentSpeedParams.thr_rot_tiny = 0.49f * (EIGEN_PI / 180.0f);
		alignmentSpeedParams.thr_rot_small = 0.5f * (EIGEN_PI / 180.0f);
		alignmentSpeedParams.thr_rot_large = 5.0f * (EIGEN_PI / 180.0f);

		alignmentSpeedParams.thr_trans_tiny = 0.98f / 1000.0; // mm
		alignmentSpeedParams.thr_trans_small = 1.0f / 1000.0; // mm
		alignmentSpeedParams.thr_trans_large = 20.0f / 1000.0; // mm

		alignmentSpeedParams.align_speed_tiny = 1.0f;
		alignmentSpeedParams.align_speed_small = 1.0f;
		alignmentSpeedParams.align_speed_large = 2.0f;

		continuousCalibrationThreshold = 1.5f;
		maxRelativeErrorThreshold = 0.005f;
		jitterThreshold = 3.0f;

		continuousCalibrationOffset = Eigen::Vector3d::Zero();

		enableStaticRecalibration = false;

		extrinsicSampleTarget = 500;
		extrinsicMinMotionDiversity = 0.03f;
		extrinsicMinAxisExcitationDeg = 20.0f;
		extrinsicMaxRmsResidual = 0.02f;
		extrinsicMinRotationalSpreadDeg = 20.0f;
		extrinsicMaxTranslationVariance = 0.0004f;
		useLegacyDynamicSolver = false;
		alignmentPeriodFrames = 300;
		continuousFrameCounter = 0;
		lastAlignmentFrame = 0;
		timeLastAlignment = 0;
		enableLockedExtrinsicPeriodicPath = false;
	}

	struct Chaperone
	{
		bool valid = false;
		bool autoApply = true;
		std::vector<vr::HmdQuad_t> geometry;
		vr::HmdMatrix34_t standingCenter = {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
		};
		vr::HmdVector2_t playSpaceSize = { 0.0f, 0.0f };
	} chaperone;

	void ClearLogOnMessage() {
		clearOnLog = true;
	}

	void Clear()
	{
		chaperone.geometry.clear();
		chaperone.standingCenter = vr::HmdMatrix34_t();
		chaperone.playSpaceSize = vr::HmdVector2_t();
		chaperone.valid = false;

		calibratedRotation = Eigen::Vector3d();
		calibratedTranslation = Eigen::Vector3d();
		calibratedScale = 1.0;
		referenceTrackingSystem = "";
		targetTrackingSystem = "";
		enabled = false;
		validProfile = false;
		rigidMountExtrinsic = RigidMountExtrinsic();
		refToTargetPose = Eigen::AffineCompact3d::Identity();
		relativePosCalibrated = true;
		lockedExtrinsic = false;
		lockedExtrinsicQuality = 0.0f;
		runtimeMode = RuntimeMode::Current;
		lockedExtrinsicNeedsRecapture = false;
		extrinsicCaptureRms = 0.0;
		extrinsicCaptureVariance = 0.0;
		extrinsicCaptureSampleCount = 0;
		extrinsicCaptureDate = "";
	}

	size_t SampleCount()
	{
		if (state == CalibrationState::CaptureExtrinsic) {
			return std::max<size_t>(500, static_cast<size_t>(extrinsicSampleTarget));
		}

		switch (calibrationSpeed)
		{
		case FAST:
			return 100;
		case SLOW:
			return 250;
		case VERY_SLOW:
			return 500;
		}
		return 100;
	}

	struct Message
	{
		enum Type
		{
			String,
			Progress
		} type = String;

		Message(Type type) : type(type), progress(0), target(0) { }

		std::string str;
		int progress, target;
	};

	std::deque<Message> messages;

	void Log(const std::string &msg)
	{
		if (clearOnLog) {
			messages.clear();
			clearOnLog = false;
		}

		if (messages.empty() || messages.back().type == Message::Progress)
			messages.push_back(Message(Message::String));

		OutputDebugStringA(msg.c_str());

		messages.back().str += msg;
		std::cerr << msg;

		while (messages.size() > 15) messages.pop_front();
	}

	void Progress(int current, int target)
	{
		if (messages.empty() || messages.back().type == Message::String)
			messages.push_back(Message(Message::Progress));

		messages.back().progress = current;
		messages.back().target = target;
	}

	bool TargetPoseIsValidSimple() const {
		return targetID >= 0 && targetID <= vr::k_unMaxTrackedDeviceCount
			&& devicePoses[targetID].poseIsValid && devicePoses[targetID].result == vr::ETrackingResult::TrackingResult_Running_OK;
	}

	bool ReferencePoseIsValidSimple() const {
		return referenceID >= 0 && referenceID <= vr::k_unMaxTrackedDeviceCount
			&& devicePoses[referenceID].poseIsValid && devicePoses[referenceID].result == vr::ETrackingResult::TrackingResult_Running_OK;
	}
};

extern CalibrationContext CalCtx;

void InitCalibrator();
void CalibrationTick(double time);
void StartCalibration();
void StartExtrinsicCalibration();
void StartContinuousCalibration();
void EndContinuousCalibration();
void LoadChaperoneBounds();
void ApplyChaperoneBounds();

void PushCalibrationApplyTime();
void ShowCalibrationDebug(int r, int c);
void DebugApplyRandomOffset();
