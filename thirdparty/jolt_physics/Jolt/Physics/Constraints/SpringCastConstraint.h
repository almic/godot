// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Core/NonCopyable.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/Constraints/ConstraintPart/AngularFrictionConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/AxisConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/ContactConstraintPart.h>
#include <Jolt/Physics/Constraints/ContactConstraintManager.h>
#include <Jolt/Physics/PhysicsStepListener.h>


JPH_NAMESPACE_BEGIN

class SpringCastConstraint;
class BroadPhaseLayerFilter;
class ObjectLayerFilter;
class BodyFilter;

/// Settings for a Spring Cast constraint
class JPH_EXPORT SpringCastConstraintSettings : public ConstraintSettings
{
	JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, SpringCastConstraintSettings)

public:
	/// Saves the contents in binary form to inStream.
	virtual void			SaveBinaryState(StreamOut &inStream) const;

	/// Restores the contents in binary form to inStream.
	virtual void			RestoreBinaryState(StreamIn &inStream);

	Vec3					mPosition { 0, 0, 0 };			///< Attachment point of the spring in local space of the body
	Vec3					mForcePoint { 0, 0, 0 };		///< Where spring forces (and friction) are applied, in local space of the body. Only recommended for kinematic-like bodies that use central forces for movement. See mEnableForcePoint.
	Vec3					mDirection { 0, -1, 0 };		///< Direction of the spring in local space of the body, should point down
	bool					mEnableForcePoint = false;		///< Enables mForcePoint, if disabled, the forces are applied at the collision contact point. This leads to a more accurate simulation when interacting with dynamic objects. When setting this to true, all forces will be applied to a fixed point on the body.
	float					mRadius = 0.3f;					///< Radius of the sphere cast (m)
	float					mMaxAngle = DegreesToRadians(85.0f); ///< Maximum angle for collisions
	float					mMinLength = 0.3f;				///< How long the spring is at max compression relative to the attachment point (m)
	float					mMaxLength = 0.5f;				///< How long the spring is at max extension relative to the attachment point (m)
	uint					mMaxBodyCount = 4;				///< Limit for the number of bodies the spring can collide with at the same time, this determines how many collision constraints will be simulated
	SpringSettings			mSpringSettings { ESpringMode::FrequencyAndDamping, 1.5f, 0.5f }; ///< Settings for the spring
};

class JPH_EXPORT SpringCastCollisionTester : public RefTarget<SpringCastCollisionTester>, public NonCopyable
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Constructors
									SpringCastCollisionTester() = default;
	explicit						SpringCastCollisionTester(ObjectLayer inObjectLayer) : mObjectLayer(inObjectLayer) { }

	/// Object layer to use for collision detection, this is used when the filters are not overridden
	ObjectLayer						GetObjectLayer() const												{ return mObjectLayer; }
	void							SetObjectLayer(ObjectLayer inObjectLayer)							{ mObjectLayer = inObjectLayer; }

	/// Access to the broad phase layer filter, when set this overrides the object layer supplied in the constructor
	void							SetBroadPhaseLayerFilter(const BroadPhaseLayerFilter *inFilter)		{ mBroadPhaseLayerFilter = inFilter; }
	const BroadPhaseLayerFilter *	GetBroadPhaseLayerFilter() const									{ return mBroadPhaseLayerFilter; }

	/// Access to the object layer filter, when set this overrides the object layer supplied in the constructor
	void							SetObjectLayerFilter(const ObjectLayerFilter *inFilter)				{ mObjectLayerFilter = inFilter; }
	const ObjectLayerFilter *		GetObjectLayerFilter() const										{ return mObjectLayerFilter; }

	/// Access to the body filter, when set this overrides the default filter that filters out the spring body
	void							SetBodyFilter(const BodyFilter *inFilter)							{ mBodyFilter = inFilter; }
	const BodyFilter *				GetBodyFilter() const												{ return mBodyFilter; }

	/// Do a collision test with the world
	/// @param inPhysicsSystem The physics system that should be tested against
	/// @param inOrigin Origin for the test, corresponds to the world space position for the spring attachment point
	/// @param inDirection Direction for the test (unit vector, world space)
	/// @param ioSpringCastConstraint The spring cast constraint
	/// @return True when collision found, false if not
	virtual bool					Collide(PhysicsSystem &inPhysicsSystem, RVec3Arg inOrigin, Vec3Arg inDirection, SpringCastConstraint &ioSpringCastConstraint) const = 0;

	/// Do a cheap contact properties prediction based on the contact properties from the last collision test (provided as input parameters)
	/// @param inPhysicsSystem The physics system that should be tested against
	/// @param inOrigin Origin for the test, corresponds to the world space position for the spring attachment point
	/// @param inDirection Direction for the test (unit vector, world space)
	/// @param ioSpringCastConstraint The spring cast constraint
	virtual void					PredictContactProperties(PhysicsSystem &inPhysicsSystem, RVec3Arg inOrigin, Vec3Arg inDirection, SpringCastConstraint &ioSpringCastConstraint) const = 0;

protected:
	const BroadPhaseLayerFilter	*	mBroadPhaseLayerFilter = nullptr;
	const ObjectLayerFilter *		mObjectLayerFilter = nullptr;
	const BodyFilter *				mBodyFilter = nullptr;
	ObjectLayer						mObjectLayer = cObjectLayerInvalid;
};

class JPH_EXPORT SpringCastCollisionTesterCastSphere : public SpringCastCollisionTester
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Constructors
									SpringCastCollisionTesterCastSphere() : SpringCastCollisionTester() {}
	/// @param inObjectLayer Object layer to test collision with
									SpringCastCollisionTesterCastSphere(ObjectLayer inObjectLayer) : SpringCastCollisionTester(inObjectLayer) { }

	virtual bool					Collide(PhysicsSystem &inPhysicsSystem, RVec3Arg inOrigin, Vec3Arg inDirection, SpringCastConstraint &ioSpringCastConstraint) const override;
	virtual void					PredictContactProperties(PhysicsSystem &inPhysicsSystem, RVec3Arg inOrigin, Vec3Arg inDirection, SpringCastConstraint &ioSpringCastConstraint) const override;
};

/// Base class for runtime data for a spring constraint
class JPH_EXPORT SpringCastConstraint final : public Constraint, public PhysicsStepListener
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Constructor
								SpringCastConstraint(Body &inBody, const SpringCastConstraintSettings &inSettings);
	virtual						~SpringCastConstraint() = default;

	// Generic interface of a constraint
	virtual bool				IsActive() const override					{ return mIsActive && mBody->IsInBroadPhase() && Constraint::IsActive(); }
	virtual EConstraintSubType	GetSubType() const 							{ return EConstraintSubType::User1; }
	virtual void				NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM) override { /* Do nothing */ }
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
	virtual void				BuildIslands(uint32 inConstraintIndex, IslandBuilder &ioBuilder, BodyManager &inBodyManager) override;
	virtual uint				BuildIslandSplits(LargeIslandSplitter &ioSplitter) const override;
#ifdef JPH_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer *inRenderer) const override;
#endif // JPH_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder &inStream) const override;
	virtual void				RestoreState(StateRecorder &inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	/// Get settings for the spring cast
	const SpringCastConstraintSettings *GetSettings() const					{ return mSettings; }
	void						SetSettings(SpringCastConstraintSettings *inSettings) { mSettings = inSettings; }

	const SpringCastCollisionTester *GetCollisionTester() const				{ return mCollisionTester; }
	/// Set the interface that tests collision between wheel and ground
	void						SetCollisionTester(const SpringCastCollisionTester *inTester) { mCollisionTester = inTester; }

	/// Get the main body
	Body *						GetBody() const								{ return mBody; }

	/// Returns true if the spring is touching an object
	inline bool					HasContact() const							{ return mNumContactBody > 0; }

	/// Returns the body that this spring is touching
	Body *						GetContactBody(uint inBodyIndex) const 		{ CheckBodyIndex(inBodyIndex); return mContactBody[inBodyIndex]; }

	/// Returns the body ID of the body that this spring is touching
	BodyID						GetContactBodyID(uint inBodyIndex) const 	{ CheckBodyIndex(inBodyIndex); return mContactBodyID[inBodyIndex]; }

	/// Returns the number of contact bodies
	uint						GetNumContactBody() const					{ return mNumContactBody; }

	/// Returns the sub shape ID where we're contacting the body
	SubShapeID					GetContactSubShapeID(uint inBodyIndex, uint inSubShapeIndex) const { uint sub_shape_index = CheckSubShapeIndex(inBodyIndex, inSubShapeIndex); return mContactSubShapeID[sub_shape_index]; }

	/// Returns the number of sub shapes for a body
	uint						GetNumSubShape(uint inBodyIndex) const
	{
		CheckBodyIndex(inBodyIndex);
		const uint offset = mContactSubShapeOffset[inBodyIndex];
		const uint next = (inBodyIndex + 1) < mContactSubShapeOffset.size()? mContactSubShapeOffset[inBodyIndex + 1] : mContactSubShapeID.size();
		return next - offset;
	}

	/// Returns the contact position in world space of the body
	RVec3						GetContactPoint(uint inBodyIndex, uint inContactPointIndex) const { uint contact_point_index = CheckContactPointIndex(inBodyIndex, inContactPointIndex); return mContactPoint[contact_point_index]; }

	/// Returns the contact position in world space for a specific shape of the body
	RVec3						GetContactPoint(uint inBodyIndex, uint inSubShapeIndex, uint inContactPointIndex) const { uint contact_point_index = CheckContactPointIndex(inBodyIndex, inSubShapeIndex, inContactPointIndex); return mContactPoint[contact_point_index]; }

	/// Returns the total number of contact points for a body
	uint						GetNumContactPoint(uint inBodyIndex) const
	{
		CheckBodyIndex(inBodyIndex);
		const uint first_point = mContactPointOffset[mContactSubShapeOffset[inBodyIndex]];

		uint last_point;
		if ((inBodyIndex + 1) < mContactSubShapeOffset.size())
			last_point = mContactPointOffset[mContactSubShapeOffset[inBodyIndex + 1]];
		else
			last_point = mContactPoint.size();

		return last_point - first_point;
	}

	/// Returns the number of contact points for a body's shape
	uint						GetNumContactPoint(uint inBodyIndex, uint inSubShapeIndex) const
	{
		uint sub_shape_index = CheckSubShapeIndex(inBodyIndex, inSubShapeIndex);
		uint contact_point_offset = mContactPointOffset[sub_shape_index];
		uint size = (sub_shape_index + 1) < mContactPointOffset.size()? mContactPointOffset[sub_shape_index + 1] : mContactPoint.size();
		return size - contact_point_offset;
	}

	/// Returns the average contact point of a body
	Vec3						GetContactPointAverage(uint inBodyIndex) const { CheckBodyIndex(inBodyIndex); return mContactPointAverage[inBodyIndex]; }

	/// Returns the contact normal in world space
	Vec3						GetContactNormal(uint inBodyIndex) const	{ CheckBodyIndex(inBodyIndex); return mContactNormal[inBodyIndex]; }

	/// Get the length of the spring (m)
	float						GetSpringLength() const						{ return mSpringLength; }

	/// Check if the spring hit its compression limit
	bool						HasHitHardPoint() const						{ return mHasHitHardPoint; }

	/// Get the total impulse (N s) that was applied by the spring
	float						GetSpringLambda() const
	{
		if (mSpringLambdaDirty) {
			mTotalSpringLambda = 0.0;
			for (const AxisConstraintPart &spring_part : mSpringPart)
				mTotalSpringLambda += spring_part.GetTotalLambda();
			mSpringLambdaDirty = false;
		}
		return mTotalSpringLambda;
	}

	/// Get total impulse (N s) applied as planar friction
	float						GetFrictionLambda() const
	{
		if (mFrictionLambdaDirty) {
			mFrictionLinearLambda = 0.0;
			for (const FrictionPart &friction_part : mFrictionConstraint)
				mFrictionLinearLambda += friction_part.GetTotalLambda();
			mFrictionLambdaDirty = false;
		}
		return mFrictionLinearLambda;
	}

	/// Get total impulse (N m s) applied as angular friction
	float						GetFrictionAngularLambda() const
	{
		if (mFrictionAngularLambdaDirty) {
			mFrictionAngularLambda = 0.0;
			for (const AngularPart &angular_part : mAngularConstraint)
				mFrictionAngularLambda += angular_part.GetTotalLambda();
			mFrictionAngularLambdaDirty = false;
		}
		return mFrictionAngularLambda;
	}

	using PickCollisionsFunction = Array<BodyID> (*)(void *inContext, const Array<Body *> &inBodyList, const Array<float> &inSpringLength, const Array<RVec3> &inAverageContactPoint, const Array<Vec3> &inContactNormal);

	PickCollisionsFunction		GetPickCollisionsFunction() const { return mPickCollisionsFunction; }
	void						SetPickCollisionsFunction(PickCollisionsFunction inPickCollisionsFunction, void *inContext) { mPickCollisionsFunction = inPickCollisionsFunction; mPickCollisionsContext = inContext; }

	/// Number of simulation steps between spring collision tests when the body is active. Default is 1. 0 = never, 1 = every step, 2 = every other step, etc.
	/// If there are multiple spring cast constraints, the tests will be spread out based on the BodyID .
	/// If you set this to test less than every step, you may see simulation artifacts. This setting can be used to reduce the cost of simulating spring constraints in the distance.
	uint						GetNumStepsBetweenCollisionTestActive() const { return mNumStepsBetweenCollisionTestActive; }
	void						SetNumStepsBetweenCollisionTestActive(uint inSteps) { mNumStepsBetweenCollisionTestActive = inSteps; }

	/// Number of simulation steps between spring collision tests when the body is inactive. Default is 1. 0 = never, 1 = every step, 2 = every other step, etc.
	/// If there are multiple spring cast constraints, the tests will be spread out based on the BodyID.
	/// This number can be lower than the number of steps when the body is active as the only purpose of this test is
	/// to allow the body to wake up in response to bodies moving into the spring cast but not touching the body.
	uint						GetNumStepsBetweenCollisionTestInactive() const { return mNumStepsBetweenCollisionTestInactive; }
	void						SetNumStepsBetweenCollisionTestInactive(uint inSteps) { mNumStepsBetweenCollisionTestInactive = inSteps; }

private:
	// See: PhysicsStepListener
	virtual void				OnStep(const PhysicsStepListenerContext &inContext) override;

	JPH_INLINE void				CheckBodyIndex(uint inBodyIndex) const		{ JPH_ASSERT(inBodyIndex < mNumContactBody); }
	JPH_INLINE uint				CheckSubShapeIndex(uint inBodyIndex, uint inSubShapeIndex) const
	{
		CheckBodyIndex(inBodyIndex);
		const uint sub_shape_offset = mContactSubShapeOffset[inBodyIndex];
		const uint sub_shape_end = (inBodyIndex + 1) < mContactSubShapeOffset.size()? mContactSubShapeOffset[inBodyIndex + 1] : mContactSubShapeID.size();
		JPH_ASSERT(sub_shape_offset + inSubShapeIndex < sub_shape_end);
		return sub_shape_offset + inSubShapeIndex;
	}
	JPH_INLINE uint				CheckContactPointIndex(uint inBodyIndex, uint inContactPointIndex) const
	{
		CheckBodyIndex(inBodyIndex);
		const uint first_point = mContactPointOffset[mContactSubShapeOffset[inBodyIndex]];
		uint end_point;
		if ((inBodyIndex + 1) < mContactSubShapeOffset.size())
			end_point = mContactPointOffset[mContactSubShapeOffset[inBodyIndex + 1]];
		else
			end_point = mContactPoint.size();
		JPH_ASSERT(first_point + inContactPointIndex < end_point);
		return first_point + inContactPointIndex;
	}
	JPH_INLINE uint				CheckContactPointIndex(uint inBodyIndex, uint inSubShapeIndex, uint inContactPointIndex) const
	{
		uint sub_shape_index = CheckSubShapeIndex(inBodyIndex, inSubShapeIndex);
		uint contact_point_offset = mContactPointOffset[sub_shape_index];
		uint size = (sub_shape_index + 1) < mContactPointOffset.size()? mContactPointOffset[sub_shape_index + 1] : mContactPoint.size();
		JPH_ASSERT(inContactPointIndex < size - contact_point_offset);
		return contact_point_offset + inContactPointIndex;
	}

	void						CalculateSpringForcePoint(size_t inContactIndex, Vec3 &outR1PlusU, Vec3 &outR2) const;

	// Simulation information
	Body *						mBody;								///< Body of the spring constraint
	RefConst<SpringCastConstraintSettings> mSettings;				///< Configuration settings for this spring cast
	RefConst<SpringCastCollisionTester> mCollisionTester;			///< Class that performs testing of collision for the spring

protected:
	// Data writable by spring cast tester
	friend class SpringCastCollisionTester;
	friend class SpringCastCollisionTesterCastSphere;

	PickCollisionsFunction		mPickCollisionsFunction;			///< Function to exclude certain body contacts after initial collision detection
	void *						mPickCollisionsContext;				///< Pointer to context data for the pick collisions function
	uint						mNumContactBody = 0;				///< Current number of contact bodies
	Array<Body *>				mContactBody;						///< Body objects in contact with the spring
	Array<BodyID>				mContactBodyID;						///< Body IDs in contact with the spring, used to retain constraint forces across steps
	Array<uint>					mContactBodyIDSorted;				///< Sorted Body IDs, ascending
	Array<uint>					mContactSubShapeOffset;				///< Sub shape ID offset list, indexed by BodyID index
	Array<SubShapeID>			mContactSubShapeID;					///< Sub shape ID list
	Array<uint>					mContactPointOffset;				///< Contact point offset list, indexed by SubShapeID index
	Array<RVec3>				mContactPoint;						///< Contact point list
	Array<RVec3>				mContactPointAverage;				///< Average contact point between the spring and contact body
	Array<float>				mContactFrictionDistance;			///< Average distance from the average friction point (which is just the point average)
	Array<Vec3>					mContactNormal;						///< Average contact world normal between the spring body and contact body, generally points away from the body
	Array<Real>					mPlaneConstant;						///< Constant for the contact plane of the spring, defined as ContactNormal . (WorldSpaceSpringPoint + SpringLength * WorldSpaceSpringDirection)
	float						mSpringLength;						///< Current length of the spring

	using SpringPartList = Array<AxisConstraintPart>;
	using FrictionPart = ContactConstraintPart<EMotionType::Dynamic, EMotionType::Dynamic>;
	using FrictionPartList = Array<FrictionPart>;
	using AngularPart = AngularFrictionConstraintPart<EMotionType::Dynamic, EMotionType::Dynamic>;
	using AngularPartList = Array<AngularPart>;

	SpringPartList				mSpringPart;						///< Controls movement up/down along the contact normal, comes in sets of 2 per body, with the second being for hard limits when reaching the minimal compression length
	FrictionPartList			mFrictionConstraint;				///< Friction with colliding bodies, comes in sets of 2 per body for planar friction
	AngularPartList				mAngularConstraint;					///< Angular friction with colliding bodies that have multiple contact points

private:
	using CombineFunction = ContactConstraintManager::CombineFunction;

	CombineFunction				mCombineFriction;					///< Cached combine friction function, set each step
	mutable float				mTotalSpringLambda;					///< Combined spring and max up constraint lambda
	mutable float				mFrictionLinearLambda;				///< Combined planar friction constraint lambda
	mutable float				mFrictionAngularLambda;				///< Combined angular friction constraint lambda
	mutable bool				mSpringLambdaDirty = true;			///< Switch for constraint lambdas being modified during solving/ warm-start phase
	mutable bool				mFrictionLambdaDirty = true;		///< Switch for constraint lambdas being modified during solving/ warm-start phase
	mutable bool				mFrictionAngularLambdaDirty = true;	///< Switch for constraint lambdas being modified during solving/ warm-start phase
	bool						mHasHitHardPoint = false;			///< Cache value for spring max up part activation
	bool						mIsActive = false;					///< If this constraint is active
	uint						mNumStepsBetweenCollisionTestActive = 1; ///< Number of simulation steps between spring collision tests when active
	uint						mNumStepsBetweenCollisionTestInactive = 1; ///< Number of simulation steps between spring collision tests when inactive
	uint						mCurrentStep = 0;					///< Current step number, used to determine when to test the spring
};

JPH_NAMESPACE_END
