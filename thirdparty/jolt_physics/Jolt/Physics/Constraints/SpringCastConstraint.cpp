// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Constraints/SpringCastConstraint.h>

#include <Jolt/Core/QuickSort.h>
#include <Jolt/Core/StreamIn.h>
#include <Jolt/Core/StreamOut.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/PhysicsSystem.h>

JPH_NAMESPACE_BEGIN

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(SpringCastConstraintSettings)
{
	JPH_ADD_BASE_CLASS(SpringCastConstraintSettings, ConstraintSettings)

	JPH_ADD_ATTRIBUTE(SpringCastConstraintSettings, mPosition)
	JPH_ADD_ATTRIBUTE(SpringCastConstraintSettings, mForcePoint)
	JPH_ADD_ATTRIBUTE(SpringCastConstraintSettings, mDirection)
	JPH_ADD_ATTRIBUTE(SpringCastConstraintSettings, mMinLength)
	JPH_ADD_ATTRIBUTE(SpringCastConstraintSettings, mMaxLength)
	JPH_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(SpringCastConstraintSettings, mSpringSettings.mMode, "mSpringMode")
	JPH_ADD_ATTRIBUTE_WITH_ALIAS(SpringCastConstraintSettings, mSpringSettings.mFrequency, "mFrequency")
	JPH_ADD_ATTRIBUTE_WITH_ALIAS(SpringCastConstraintSettings, mSpringSettings.mDamping, "mDamping")
	JPH_ADD_ATTRIBUTE(SpringCastConstraintSettings, mRadius)
	JPH_ADD_ATTRIBUTE(SpringCastConstraintSettings, mEnableForcePoint)
}

void SpringCastConstraintSettings::SaveBinaryState(StreamOut &inStream) const
{
	ConstraintSettings::SaveBinaryState(inStream);

	inStream.Write(mPosition);
	inStream.Write(mForcePoint);
	inStream.Write(mDirection);
	inStream.Write(mMinLength);
	inStream.Write(mMaxLength);
	mSpringSettings.SaveBinaryState(inStream);
	inStream.Write(mRadius);
	inStream.Write(mEnableForcePoint);
}

void SpringCastConstraintSettings::RestoreBinaryState(StreamIn &inStream)
{
	ConstraintSettings::RestoreBinaryState(inStream);

	inStream.Read(mPosition);
	inStream.Read(mForcePoint);
	inStream.Read(mDirection);
	inStream.Read(mMinLength);
	inStream.Read(mMaxLength);
	mSpringSettings.RestoreBinaryState(inStream);
	inStream.Read(mRadius);
	inStream.Read(mEnableForcePoint);
}

class SphereCastCollector : public CastShapeCollector
{
public:
						SphereCastCollector(PhysicsSystem &inPhysicsSystem, const RShapeCast &inShapeCast, Vec3Arg inUpDirection, float inCosMaxAngle, uint inMaxBodies, SpringCastConstraint::PickCollisionsFunction inPickCollisionsFunction, void *inPickCollisionsContext) :
		mPhysicsSystem(inPhysicsSystem),
		mPhysicsSettings(inPhysicsSystem.GetPhysicsSettings()),
		mShapeCast(inShapeCast),
		// NOTE: flip up direction to avoid many normal direction flips during hit detection
		mUpDirection(-inUpDirection),
		mCosMaxAngle(inCosMaxAngle),
		mMaxBodies(inMaxBodies),
		mPickCollisionsFunction(inPickCollisionsFunction),
		mPickCollisionsContext(inPickCollisionsContext)
	{
	}

	virtual void		OnBody(const Body &inBody) override
	{
		// Ignore sensors
		if (inBody.IsSensor())
		{
			ForceEarlyOut();
			return;
		}

		mCurrentBody = &inBody;
	}

	virtual void		AddHit(const ShapeCastResult &inResult) override
	{
		const float fraction = inResult.mFraction;
		// TODO: Shouldn't happen, this check should be removed
		if (fraction > GetEarlyOutFraction())
			return;

		// Test that we're not hitting a vertical wall
		const Vec3 normal = inResult.mPenetrationAxis.Normalized();
		if (normal.Dot(mUpDirection) < mCosMaxAngle)
			return;

		const float early_out = fraction + mPhysicsSettings.mSpeculativeContactDistance;

		const BodyID body_id = inResult.mBodyID2;
		const SubShapeID sub_shape_id = inResult.mSubShapeID2;
		const RVec3 contact_point = mShapeCast.mCenterOfMassStart.GetTranslation() + inResult.mContactPointOn2;

		// Find body id index
		uint body_index = 0;
		for (uint size = mContactBodyID.size(); body_index < size; ++body_index)
		{
			if (mContactBodyID[body_index] == body_id)
				break;
		}

		if (body_index == mContactBodyID.size())
		{
			mFractionList.push_back(fraction);
			mContactBody.push_back(const_cast<Body *>(mCurrentBody));
			mContactBodyID.push_back(body_id);
			mContactSubShapeOffset.push_back(mContactSubShapeID.size());
		}

		if (early_out < GetEarlyOutFraction())
		{
			mFractionList[body_index] = fraction;
			UpdateEarlyOutFraction(early_out);
		}

		// Find sub shape id index
		uint shape_index = 0;
		const uint sub_shape_offset = mContactSubShapeOffset[body_index];
		const uint16 sub_shape_count = ((body_index + 1) < mContactSubShapeOffset.size() ? mContactSubShapeOffset[body_index + 1] : mContactSubShapeID.size()) - sub_shape_offset;
		bool found = false;
		for (; shape_index < sub_shape_count; ++shape_index)
		{
			if (mContactSubShapeID[sub_shape_offset + shape_index] == sub_shape_id)
			{
				found = true;
				break;
			}
		}

		shape_index += sub_shape_offset;

		if (!found)
		{
			// Increment offset of next body shape keys
			for (uint next_body_index = body_index + 1; next_body_index < mContactSubShapeOffset.size(); ++next_body_index)
				mContactSubShapeOffset[next_body_index] += 1;

			mContactSubShapeID.insert(mContactSubShapeID.cbegin() + shape_index, sub_shape_id);

			// Take offset of current location
			uint current_point_offset = 0;
			if (shape_index + 1 < mContactPointOffset.size())
				current_point_offset = mContactPointOffset[shape_index + 1];
			// Offset increment happens later, when the point is inserted
			mContactPointOffset.insert(mContactSubShapeOffset.cbegin() + shape_index, current_point_offset);
		}

		// Insert point at last position for this shape offset
		uint point_offset_index = shape_index + 1;
		if (point_offset_index < mContactPointOffset.size())
		{
			const uint offset = mContactPointOffset[point_offset_index];
			mContactPoint.insert(mContactPoint.cbegin() + offset, contact_point);
			mContactNormal.insert(mContactNormal.cbegin() + offset, normal);
		}
		else
		{
			mContactPoint.push_back(contact_point);
			mContactNormal.push_back(normal);
		}

		// Update offsets
		for (; point_offset_index < mContactPointOffset.size(); ++point_offset_index)
			mContactPointOffset[point_offset_index] += 1;
	}

	virtual void		OnBodyEnd() override
	{
		// NOTE: we collect for every body, and later trim to the closest set of hits
		Reset();
	}

	/// Removes contact points, shapes, and bodies that are further than the speculative contact distance from the current fraction,
	/// and calculates average points, normals, and velocity.
	void				FinalizeContacts()
	{
		// Generally a waste of time, but I'd like to be safe
		mContactPointAverage.clear();
		mContactFrictionDistance.clear();

		// Early out if no bodies matched
		if (mContactBody.size() == 0)
			return;

		// Since this is a sphere cast, we need to do extra math to check distances from the final sphere center
		// But removal is simply dropping points and updating offsets as we test
		const float slop_distance = mPhysicsSettings.mSpeculativeContactDistance;
		const float radius_plus_slop_sq = Square(static_cast<const SphereShape *>(mShapeCast.mShape)->GetRadius() + slop_distance);
		const RVec3 sphere_start = mShapeCast.mCenterOfMassStart.GetTranslation();

		// Using -1 body/ shape indexes so that the first point will initialize everything correctly
		uint body_index = -1;
		uint body_point_count = 0;
		uint next_body_at_shape = 0;

		uint shape_index = -1;
		uint shape_keep_index = 0;
		uint shape_point_count = 0;
		uint next_shape_at_point = 0;

		uint point_index = 0;
		uint point_keep_index = 0;

		RVec3 average_contact_point = RVec3::sZero();
		Vec3 average_contact_normal = Vec3::sZero();
		RVec3 sphere_point = RVec3::sZero();
		for (; point_index <= mContactPoint.size(); ++point_index)
		{
			if (point_index == next_shape_at_point)
			{
				if (shape_point_count > 0)
				{
					mContactSubShapeID[shape_keep_index] = mContactSubShapeID[shape_index];
					++shape_keep_index;
					if (shape_keep_index < mContactPointOffset.size())
						mContactPointOffset[shape_keep_index] = point_keep_index;
					shape_point_count = 0;
				}

				++shape_index;
				if (shape_index + 1 < mContactPointOffset.size())
					next_shape_at_point = mContactPointOffset[shape_index + 1];
				else
					next_shape_at_point = mContactPoint.size();

				if (shape_index == next_body_at_shape)
				{
					if (body_point_count > 0)
					{
						mContactPointAverage.push_back(average_contact_point / body_point_count);
						average_contact_point = RVec3::sZero();

						// Normals are per body now, we have already read in the normal at this location
						mContactNormal[body_index] = average_contact_normal.NormalizedOr(mUpDirection);
						average_contact_normal = Vec3::sZero();

						body_point_count = 0;
					}

					++body_index;
					if (body_index > mContactBody.size())
						break;

					mContactSubShapeOffset[body_index] = shape_keep_index;

					if (body_index + 1 < mContactSubShapeOffset.size())
						next_body_at_shape = mContactSubShapeOffset[body_index + 1];
					else
						next_body_at_shape = mContactSubShapeID.size();

					sphere_point = sphere_start + (mShapeCast.mDirection * mFractionList[body_index]);
				}
			}

			const RVec3 point = mContactPoint[point_index]; // above code should stop this from running when point_index == size

			if (point.IsClose(sphere_point, radius_plus_slop_sq)) {
				// Put into actual index
				++body_point_count;
				++shape_point_count;
				mContactPoint[point_keep_index] = point;
				++point_keep_index;

				average_contact_point += point;
				average_contact_normal += mContactNormal[point_index];
			}
		}

		// Initial resize for anything that was removed
		mContactNormal.resize(body_index);

		mContactSubShapeID.resize(shape_keep_index);
		mContactPointOffset.resize(shape_keep_index);

		mContactPoint.resize(point_keep_index);

		// Find closest fraction of all kept bodies
		Array<BodyID> kept_body_list;
		if (mPickCollisionsFunction != nullptr)
			kept_body_list = mPickCollisionsFunction(mPickCollisionsContext, mContactBody, mFractionList, mContactPointAverage, mContactNormal);
		else
			kept_body_list = mContactBodyID;

		mClosestFraction = 1.0f;
		for (const BodyID& body_id : kept_body_list)
		{
			for (body_index = 0; body_index < mContactBodyID.size(); ++body_index)
				if (mContactBodyID[body_index] == body_id)
					break;

			if (body_index == mContactBodyID.size())
				continue;

			const float fraction = mFractionList[body_index];
			if (fraction < mClosestFraction)
				mClosestFraction = fraction;
		}

		const RVec3 sphere_center = sphere_start + (mShapeCast.mDirection * mClosestFraction);

		body_index = -1;
		uint body_keep_index = 0;
		body_point_count = 0;
		next_body_at_shape = 0;

		shape_index = -1;
		shape_keep_index = 0;
		shape_point_count = 0;
		next_shape_at_point = 0;

		point_keep_index = 0;

		average_contact_point = RVec3::sZero();
		average_contact_normal = Vec3::sZero();

		for (point_index = 0; point_index <= mContactPoint.size(); ++point_index)
		{
			if (point_index == next_shape_at_point)
			{
				if (shape_point_count > 0)
				{
					mContactSubShapeID[shape_keep_index] = mContactSubShapeID[shape_index];
					++shape_keep_index;
					if (shape_keep_index < mContactPointOffset.size())
						mContactPointOffset[shape_keep_index] = point_keep_index;
					shape_point_count = 0;
				}

				++shape_index;
				if (shape_index + 1 < mContactPointOffset.size())
					next_shape_at_point = mContactPointOffset[shape_index + 1];
				else
					next_shape_at_point = mContactPoint.size();

				if (shape_index == next_body_at_shape)
				{
					if (body_point_count > 0)
					{
						mContactBody[body_keep_index] = mContactBody[body_index];
						mContactBodyID[body_keep_index] = mContactBodyID[body_index];
						mFractionList[body_keep_index] = mFractionList[body_index];

						average_contact_point /= body_point_count;
						mContactPointAverage[body_keep_index] = average_contact_point;

						average_contact_normal = average_contact_normal.NormalizedOr(mUpDirection);
						mContactNormal[body_keep_index] = average_contact_normal;

						// Calculate average friction point distance for angular friction
						float average_friction_distance = 0.0f;
						for (size_t p_index = point_keep_index - body_point_count; p_index < point_keep_index; ++p_index)
						{
							const RVec3 point = mContactPoint[p_index];
							const Vec3 delta = Vec3(point - average_contact_point);
							average_friction_distance += (delta - delta.Dot(average_contact_normal) * average_contact_normal).Length();
						}

						mContactFrictionDistance.push_back(average_friction_distance / body_point_count);

						++body_keep_index;
						if (body_keep_index >= mMaxBodies)
							break;
						if (body_keep_index < mContactSubShapeOffset.size())
							mContactSubShapeOffset[body_keep_index] = shape_keep_index;

						average_contact_normal = Vec3::sZero();
						average_contact_point = RVec3::sZero();
						body_point_count = 0;
					}

					bool found = false;
					do {
						++body_index;
						if (body_index >= mContactBodyID.size())
							break;

						const BodyID search_id = mContactBodyID[body_index];
						for (const BodyID& body_id : kept_body_list)
						{
							if (body_id != search_id)
								continue;

							found = true;
							break;
						}
					} while (!found);

					// All kept bodies have been processed
					if (body_index >= mContactBody.size())
						break;

					if (body_index + 1 < mContactSubShapeOffset.size())
						next_body_at_shape = mContactSubShapeOffset[body_index + 1];
					else
						next_body_at_shape = mContactSubShapeID.size();
				}
			}

			const RVec3 point = mContactPoint[point_index]; // above code should stop this from running when point_index == size
			if (point.IsClose(sphere_center, radius_plus_slop_sq)) {
				// Put into actual index
				++body_point_count;
				++shape_point_count;
				mContactPoint[point_keep_index] = point;
				++point_keep_index;

				average_contact_point += point;
				average_contact_normal += mContactNormal[point_index];
			}
		}

		// Final resize
		mContactBody.resize(body_keep_index);
		mContactBodyID.resize(body_keep_index);
		mContactSubShapeOffset.resize(body_keep_index);

		mContactSubShapeID.resize(shape_keep_index);
		mContactPointOffset.resize(shape_keep_index);

		mContactPoint.resize(point_keep_index);
		mContactPointAverage.resize(body_keep_index);

		mContactNormal.resize(body_index);
	}

	// Configuration
	PhysicsSystem &		mPhysicsSystem;
	const PhysicsSettings &mPhysicsSettings;
	const RShapeCast &	mShapeCast;
	Vec3				mUpDirection;			///< Determines direction to test max contact angle with
	float				mCosMaxAngle;			///< Cached cosine of the max contact angle
	uint				mMaxBodies;				///< Maximum bodies to collide with, only applied on finalize
	SpringCastConstraint::PickCollisionsFunction mPickCollisionsFunction;
	void *				mPickCollisionsContext;

	float				mClosestFraction;		///< Closest hit fraction
	Array<float>		mFractionList;			///< List of closest hit fractions per colliding body

	const Body *		mCurrentBody;

	Array<Body *>		mContactBody;
	Array<BodyID>		mContactBodyID;
	Array<uint>			mContactSubShapeOffset;
	Array<SubShapeID>	mContactSubShapeID;
	Array<uint>			mContactPointOffset;
	Array<RVec3>		mContactPoint;			///< Before finalize: list of contact points, not guaranteed to be the closest group. After finalize: all contacts within radius + mSpeculativeContactDistance of the closest hit fraction
	Array<RVec3>		mContactPointAverage;	///< After finalize: List of average contact point on each body
	Array<float>		mContactFrictionDistance; ///< After finalize: List of average contact point distance from friction center
	Array<Vec3>			mContactNormal;			///< Before finalize: list of contact normals for each contact point, pointing away from the spring. After finalize: list of average contact normal for each body
};

bool SpringCastCollisionTesterCastSphere::Collide(PhysicsSystem &inPhysicsSystem, RVec3Arg inOrigin, Vec3Arg inDirection, SpringCastConstraint &ioSpringCastConstraint) const
{
	const DefaultBroadPhaseLayerFilter default_broadphase_layer_filter = inPhysicsSystem.GetDefaultBroadPhaseLayerFilter(mObjectLayer);
	const BroadPhaseLayerFilter &broadphase_layer_filter = mBroadPhaseLayerFilter != nullptr? *mBroadPhaseLayerFilter : default_broadphase_layer_filter;

	const DefaultObjectLayerFilter default_object_layer_filter = inPhysicsSystem.GetDefaultLayerFilter(mObjectLayer);
	const ObjectLayerFilter &object_layer_filter = mObjectLayerFilter != nullptr? *mObjectLayerFilter : default_object_layer_filter;

	const BodyFilter default_body_filter;
	const BodyFilter &body_filter = mBodyFilter != nullptr ? *mBodyFilter : default_body_filter;
	const IgnoreSingleBodyFilterChained body_filter_chained(ioSpringCastConstraint.GetBody()->GetID(), body_filter);

	const SpringCastConstraintSettings *cast_settings = ioSpringCastConstraint.GetSettings();

	SphereShape sphere(cast_settings->mRadius);
	sphere.SetEmbedded();

	const float cos_max_angle = Cos(cast_settings->mMaxAngle);
	const float max_length = cast_settings->mMaxLength;

	RShapeCast shape_cast(&sphere, Vec3::sOne(), RMat44::sTranslation(inOrigin), inDirection * max_length);
	ShapeCastSettings settings;
	settings.mUseShrunkenShapeAndConvexRadius = true;

	// TODO: UP is assumed to be opposite to cast direction, though this should probably be a setting on the constraint
	SphereCastCollector collector(inPhysicsSystem, shape_cast, -inDirection, cos_max_angle, cast_settings->mMaxBodyCount, ioSpringCastConstraint.GetPickCollisionsFunction(), ioSpringCastConstraint.mPickCollisionsContext);
	inPhysicsSystem.GetNarrowPhaseQueryNoLock().CastShape(shape_cast, settings, shape_cast.mCenterOfMassStart.GetTranslation(), collector, broadphase_layer_filter, object_layer_filter, body_filter_chained);
	collector.FinalizeContacts();

	const size_t num_bodies = collector.mContactBody.size();
	const size_t old_num_bodies = ioSpringCastConstraint.mNumContactBody;
	ioSpringCastConstraint.mNumContactBody = num_bodies;
	ioSpringCastConstraint.mContactBody.assign(collector.mContactBody.begin(), collector.mContactBody.end());

	// Retain active parts by matching up the body IDs and copying lambdas
	SpringCastConstraint::SpringPartList old_spring_part;
	SpringCastConstraint::FrictionPartList old_friction_part;
	SpringCastConstraint::AngularPartList old_angular_part;
	old_spring_part.resize(num_bodies * 2);
	old_friction_part.resize(num_bodies * 2);
	old_angular_part.resize(num_bodies);

	ioSpringCastConstraint.mSpringPart.swap(old_spring_part);
	ioSpringCastConstraint.mFrictionConstraint.swap(old_friction_part);
	ioSpringCastConstraint.mAngularConstraint.swap(old_angular_part);

	for (size_t body_index = 0; body_index < num_bodies; ++body_index)
	{
		const BodyID body_id = collector.mContactBodyID[body_index];
		for (size_t other_index = 0; other_index < old_num_bodies; ++other_index)
		{
			if (body_id != ioSpringCastConstraint.mContactBodyID[other_index])
				continue;

			ioSpringCastConstraint.mSpringPart[body_index * 2].SetTotalLambda(old_spring_part[other_index * 2].GetTotalLambda());
			ioSpringCastConstraint.mSpringPart[body_index * 2 + 1].SetTotalLambda(old_spring_part[other_index * 2 + 1].GetTotalLambda());
			ioSpringCastConstraint.mFrictionConstraint[body_index * 2].SetTotalLambda(old_friction_part[other_index * 2].GetTotalLambda());
			ioSpringCastConstraint.mFrictionConstraint[body_index * 2 + 1].SetTotalLambda(old_friction_part[other_index * 2 + 1].GetTotalLambda());
			ioSpringCastConstraint.mAngularConstraint[body_index].SetTotalLambda(old_angular_part[other_index].GetTotalLambda());

			break;
		}
	}

	ioSpringCastConstraint.mContactBodyID.assign(collector.mContactBodyID.begin(), collector.mContactBodyID.end());

	// Generate sorted BodyID list in ascending order
	ioSpringCastConstraint.mContactBodyIDSorted.resize(num_bodies);
	for (uint i = 0; i < num_bodies; ++i)
		ioSpringCastConstraint.mContactBodyIDSorted[i] = i;

	QuickSort(
		ioSpringCastConstraint.mContactBodyIDSorted.begin(),
		ioSpringCastConstraint.mContactBodyIDSorted.end(),
		[&ioSpringCastConstraint](const uint &a, const uint &b) {
			BodyID id_a = ioSpringCastConstraint.mContactBodyID[a];
			BodyID id_b = ioSpringCastConstraint.mContactBodyID[b];
			return id_a < id_b;
		}
	);

	ioSpringCastConstraint.mContactSubShapeOffset.assign(collector.mContactSubShapeOffset.begin(), collector.mContactSubShapeOffset.end());
	ioSpringCastConstraint.mContactSubShapeID.assign(collector.mContactSubShapeID.begin(), collector.mContactSubShapeID.end());
	ioSpringCastConstraint.mContactPointOffset.assign(collector.mContactPointOffset.begin(), collector.mContactPointOffset.end());
	ioSpringCastConstraint.mContactPoint.assign(collector.mContactPoint.begin(), collector.mContactPoint.end());
	ioSpringCastConstraint.mContactPointAverage.assign(collector.mContactPointAverage.begin(), collector.mContactPointAverage.end());
	ioSpringCastConstraint.mContactFrictionDistance.assign(collector.mContactFrictionDistance.begin(), collector.mContactFrictionDistance.end());
	ioSpringCastConstraint.mContactNormal.assign(collector.mContactNormal.begin(), collector.mContactNormal.end());
	ioSpringCastConstraint.mSpringLength = max(0.0f, max_length * collector.mClosestFraction);

	return ioSpringCastConstraint.mNumContactBody > 0;
}

void SpringCastCollisionTesterCastSphere::PredictContactProperties(PhysicsSystem &inPhysicsSystem, RVec3Arg inOrigin, Vec3Arg inDirection, SpringCastConstraint &ioSpringCastConstraint) const
{
	const SpringCastConstraintSettings *settings = ioSpringCastConstraint.GetSettings();
	const float max_length = settings->mMaxLength;

	size_t body_count = ioSpringCastConstraint.GetNumContactBody();
	if (body_count == 0)
	{
		ioSpringCastConstraint.mSpringLength = max_length;
		return;
	}

	const float radius = settings->mRadius;
	const float neg_cos_max_angle = -Cos(settings->mMaxAngle);

	// NOTE: because two points at the edges of the sphere could average to a point inside the
	// sphere, giving a much smaller fraction than it should be, I calculate a fraction per point,
	// average the new points per body for the velocity calculation, and average the fractions per
	// per body.
	float average_fraction = 0.0f;

	for (uint body_index = 0, num_body = body_count; body_index < num_body; ++body_index)
	{
		// Body could be invalid
		const BodyID body_id = ioSpringCastConstraint.mContactBodyID[body_index];
		if (body_id.IsInvalid())
		{
			--body_count;
			continue;
		}

		const Body *body = inPhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody(body_id);
		ioSpringCastConstraint.mContactBody[body_index] = const_cast<Body *>(body);

		if (body == nullptr)
		{
			--body_count;
			ioSpringCastConstraint.mContactBodyID[body_index] = BodyID();
			continue;
		}

		const Vec3 normal = ioSpringCastConstraint.mContactNormal[body_index];
		const Vec3 neg_normal = -normal;
		const float d_dot_n = inDirection.Dot(neg_normal);

		// NOTE: because these are negatives, the comparison flips
		if (d_dot_n > neg_cos_max_angle)
		{
			--body_count;
			// Drop pointer to disable forces for this step
			ioSpringCastConstraint.mContactBody[body_index] = nullptr;
			continue;
		}

		const uint32 shape_offset = ioSpringCastConstraint.mContactSubShapeOffset[body_index];
		const size_t first_point = ioSpringCastConstraint.mContactPointOffset[shape_offset];
		const size_t last_point = (body_index + 1) < ioSpringCastConstraint.mContactSubShapeOffset.size() ? ioSpringCastConstraint.mContactPointOffset[ioSpringCastConstraint.mContactSubShapeOffset[body_index + 1]] : ioSpringCastConstraint.mContactPoint.size();
		const size_t point_count = last_point - first_point;
		if (point_count == 0)
		{
			--body_count;
			// Drop pointer to disable forces for this step
			ioSpringCastConstraint.mContactBody[body_index] = nullptr;
			continue;
		}

		float average_body_fraction = 0.0f;
		RVec3 average_contact_point = RVec3::sZero();
		for (size_t point_index = first_point; point_index < last_point; ++point_index)
		{
			const RVec3 point = ioSpringCastConstraint.mContactPoint[point_index];
			const float oc_dot_n = Vec3(point - inOrigin).Dot(neg_normal);
			const float fraction = (radius + oc_dot_n) / d_dot_n;
			const RVec3 new_point = inOrigin + fraction * inDirection - radius * neg_normal;
			ioSpringCastConstraint.mContactPoint[point_index] = new_point;
			average_body_fraction += Clamp(fraction, 0.0f, max_length);
			average_contact_point += new_point;
		}

		average_contact_point /= point_count;
		ioSpringCastConstraint.mContactPointAverage[body_index] = average_contact_point;
		average_fraction += average_body_fraction / point_count;

		float average_friction_distance = 0.0f;
		for (size_t point_index = first_point; point_index < last_point; ++point_index)
		{
			const RVec3 point = ioSpringCastConstraint.mContactPoint[point_index];
			const Vec3 delta = Vec3(point - average_contact_point);
			average_friction_distance += (delta - delta.Dot(normal) * normal).Length();
		}
		average_friction_distance /= point_count;

		ioSpringCastConstraint.mContactFrictionDistance[body_index] = average_friction_distance;
	}

	if (body_count == 0)
		ioSpringCastConstraint.mSpringLength = max_length;
	else
		ioSpringCastConstraint.mSpringLength = average_fraction / body_count;
}

SpringCastConstraint::SpringCastConstraint(Body &inBody, const SpringCastConstraintSettings &inSettings) :
	Constraint(inSettings),
	mBody(&inBody),
	mSettings(&inSettings)
{
	mCurrentStep = uint32(Hash64(inBody.GetID().GetIndex()));
}

void SpringCastConstraint::OnStep(const PhysicsStepListenerContext &inContext)
{
	// Step only if we're in the broadphase
	if (!mBody->IsInBroadPhase())
		return;

	mCombineFriction = inContext.mPhysicsSystem->GetCombineFriction();

	mIsActive = mBody->IsActive();
	const uint num_steps_between_collisions = mIsActive? mNumStepsBetweenCollisionTestActive : mNumStepsBetweenCollisionTestInactive;

	const RMat44 body_transform = mBody->GetWorldTransform();
	const RVec3 origin = body_transform * mSettings->mPosition;
	const Vec3 direction = body_transform.Multiply3x3(mSettings->mDirection);

	if (mCurrentStep % num_steps_between_collisions != 0)
		// Prediction using existing points
		mCollisionTester->PredictContactProperties(*inContext.mPhysicsSystem, origin, direction, *this);
	else
		// Full Collision
		mCollisionTester->Collide(*inContext.mPhysicsSystem, origin, direction, *this);

	for (uint body_index = 0; body_index < mNumContactBody; ++body_index)
		mPlaneConstant[body_index] = RVec3(-mContactNormal[body_index]).Dot(origin + mSpringLength * direction);

	++mCurrentStep;
}

void SpringCastConstraint::BuildIslands(uint32 inConstraintIndex, IslandBuilder &ioBuilder, BodyManager &inBodyManager)
{
	// Find dynamic bodies that the spring is touching
	BodyID *body_ids = (BodyID *)JPH_STACK_ALLOC((mContactBody.size() + 1) * sizeof(BodyID));
	int num_bodies = 0;
	bool needs_to_activate = false;
	for (const Body *body : mContactBody)
	{
		if (body == nullptr)
			continue;

		if (body->IsDynamic())
		{
			body_ids[num_bodies] = body->GetID();
			++num_bodies;
			needs_to_activate |= !body->IsActive();
		}
	}

	// Activate bodies, note that if we get here we have already told the system that we're active so that means our main body needs to be active too
	if (!mBody->IsActive())
	{
		// Our main body is not active, activate it too
		body_ids[num_bodies] = mBody->GetID();
		inBodyManager.ActivateBodies(body_ids, num_bodies + 1);
	}
	else if (needs_to_activate)
	{
		// Only activate bodies the wheels are touching
		inBodyManager.ActivateBodies(body_ids, num_bodies);
	}

	// Link dynamic bodies into the same island as the vehicle
	for (int i = 0; i < num_bodies; ++i)
	{
		const Body &body = inBodyManager.GetBody(body_ids[i]);
		if (body.IsDynamic())
			ioBuilder.LinkBodies(mBody->GetIndexInActiveBodiesInternal(), body.GetIndexInActiveBodiesInternal());
	}

	// Link the constraint to the body
	ioBuilder.LinkConstraint(inConstraintIndex, mBody->GetIndexInActiveBodiesInternal());
}

uint SpringCastConstraint::BuildIslandSplits(LargeIslandSplitter &ioSplitter) const
{
	ioSplitter.AssignToNonParallelSplit(mBody);
}

void SpringCastConstraint::CalculateSpringForcePoint(size_t inContactIndex, Vec3 &outR1PlusU, Vec3 &outR2) const
{
	RVec3 force_point;
	if (mSettings->mEnableForcePoint)
		force_point = mBody->GetWorldTransform() * mSettings->mForcePoint;
	else
		force_point = mContactPointAverage[inContactIndex];

	outR1PlusU = Vec3(force_point - mBody->GetCenterOfMassPosition());
	outR2 = Vec3(force_point - mContactBody[inContactIndex]->GetCenterOfMassPosition());
}

void SpringCastConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	// TODO: memory leak: must occasionally do shrink_to_fit() on contact arrays. For now I just call it every 500 steps. In the future it could be done less often with some heuristic.
	if (mCurrentStep % 500 == 0)
	{
		mContactBody.shrink_to_fit();
		mContactBodyID.shrink_to_fit();
		mContactBodyIDSorted.shrink_to_fit();
		mContactSubShapeOffset.shrink_to_fit();
		mContactSubShapeID.shrink_to_fit();
		mContactPointOffset.shrink_to_fit();
		mContactPoint.shrink_to_fit();
		mContactPointAverage.shrink_to_fit();
		mContactFrictionDistance.shrink_to_fit();
		mContactNormal.shrink_to_fit();

		mSpringPart.shrink_to_fit();
		mFrictionConstraint.shrink_to_fit();
		mAngularConstraint.shrink_to_fit();
	}

	// This will be the same for all contact bodies, so calculate early
	RMat44 body_transform = mBody->GetWorldTransform();
	const Vec3 spring_ws_direction = body_transform.Multiply3x3(mSettings->mDirection);
	const float spring_c = mSpringLength - mSettings->mMaxLength;
	const bool spring_valid = mSettings->mMaxLength > mSettings->mMinLength;
	const bool spring_hard = mSpringLength < mSettings->mMinLength;
	float stiffness, damping;
	if (spring_valid)
	{
		ESpringMode mode = mSettings->mSpringSettings.mMode;
		if (mode == ESpringMode::FrequencyAndDamping || mode == ESpringMode::MassNormalizedStiffnessAndDamping)
		{
			Vec3 force_point;
			if (mSettings->mEnableForcePoint)
				force_point = mSettings->mForcePoint;
			else
				// NOTE: Force point for effective mass is half of the current spring length. This
				// is different from VehicleConstraint because I think this makes more sense. If
				// this actually gives worse results then it should be changed to "average" spring
				// length: position + 0.5 * (min + max) * direction
				force_point = mSettings->mPosition + 0.5f * mSpringLength * mSettings->mDirection;
			// TODO: this should be a setting, but for now it is fine because basically all local up is the Y axis
			Vec3 force_point_x_neg_up = force_point.Cross(-Vec3::sAxisY());
			const MotionProperties *mp = mBody->GetMotionProperties();
			float effective_mass = 1.0f / (mp->GetInverseMass() + force_point_x_neg_up.Dot(mp->GetLocalSpaceInverseInertia().Multiply3x3(force_point_x_neg_up)));

			if (mode == ESpringMode::FrequencyAndDamping)
			{
				float omega = 2.0f * JPH_PI * mSettings->mSpringSettings.mFrequency;
				stiffness = effective_mass * Square(omega);
				damping = 2.0f * effective_mass * mSettings->mSpringSettings.mDamping * omega;
			}
			else
			{
				stiffness = effective_mass * mSettings->mSpringSettings.mStiffness;
				damping = effective_mass * mSettings->mSpringSettings.mDamping;
			}
		}
		else
		{
			stiffness = mSettings->mSpringSettings.mStiffness;
			damping = mSettings->mSpringSettings.mDamping;
		}
	}

	const EMotionType motion_type1 = mBody->GetMotionType();
	const MotionProperties *motion_properties1 = mBody->GetMotionPropertiesUnchecked();
	float inv_m1;
	Mat44 inv_i1;
	if (motion_type1 == EMotionType::Dynamic)
	{
		inv_m1 = motion_properties1->GetInverseMass();
		inv_i1 = mBody->GetInverseInertia();
	}
	else
	{
		inv_m1 = 0.0f;
		inv_i1 = Mat44::sZero();
	}

	for (uint body_index = 0; body_index < mContactBody.size(); ++body_index)
	{
		// Some bodies could be gone or prediction has them disabled, so check for that
		const Body *contact_body = mContactBody[body_index];
		AxisConstraintPart &spring_part = mSpringPart[body_index * 2];
		AxisConstraintPart &spring_hard_part = mSpringPart[body_index * 2 + 1];
		FrictionPart &friction1 = mFrictionConstraint[body_index * 2];
		FrictionPart &friction2 = mFrictionConstraint[body_index * 2 + 1];
		AngularPart &angular_friction = mAngularConstraint[body_index];

		if (contact_body == nullptr)
		{
			spring_part.Deactivate();
			spring_hard_part.Deactivate();
			friction1.Deactivate();
			friction2.Deactivate();
			angular_friction.Deactivate();
			continue;
		}

		const Vec3 contact_normal = mContactNormal[body_index];
		Vec3 r1_plus_u, r2;
		CalculateSpringForcePoint(body_index, r1_plus_u, r2);

		if (spring_valid)
		{
			const float cos_angle = max(0.1f, spring_ws_direction.Dot(contact_normal));
			spring_part.CalculateConstraintPropertiesWithStiffnessAndDamping(inDeltaTime, *mBody, r1_plus_u, *contact_body, r2, contact_normal, 0.0f, spring_c, stiffness / cos_angle, damping / cos_angle);
		}
		else
			spring_part.Deactivate();

		if (spring_hard)
			spring_hard_part.CalculateConstraintProperties(*mBody, r1_plus_u, *contact_body, r2, contact_normal);
		else
			spring_hard_part.Deactivate();

		const Vec3 t1 = contact_normal.GetNormalizedPerpendicular();
		const Vec3 t2 = contact_normal.Cross(t1);
		const float combined_friction = mCombineFriction(*mBody, SubShapeID(), *contact_body, SubShapeID());

		if (combined_friction > 0.0f)
		{
			const EMotionType motion_type2 = contact_body->GetMotionType();
			const MotionProperties *motion_properties2 = contact_body->GetMotionPropertiesUnchecked();
			float inv_m2;
			Mat44 inv_i2;
			if (motion_type2 == EMotionType::Dynamic)
			{
				inv_m2 = motion_properties2->GetInverseMass();
				inv_i2 = contact_body->GetInverseInertia();
			}
			else
			{
				inv_m2 = 0.0f;
				inv_i2 = Mat44::sZero();
			}

			friction1.CalculateConstraintProperties(inv_m1, inv_i1, r1_plus_u, inv_m2, inv_i2, r2, t1);
			friction2.CalculateConstraintProperties(inv_m1, inv_i1, r1_plus_u, inv_m2, inv_i2, r2, t2);

			if (GetNumContactPoint(body_index) > 1)
				angular_friction.CalculateConstraintProperties(inv_i1, inv_i2, contact_normal);
			else
				angular_friction.Deactivate();
		}
		else
		{
			friction1.Deactivate();
			friction2.Deactivate();
			angular_friction.Deactivate();
		}
	}
}

void SpringCastConstraint::ResetWarmStart()
{
	for (AxisConstraintPart &spring_part : mSpringPart)
		spring_part.Deactivate();

	for (FrictionPart &friction_part : mFrictionConstraint)
		friction_part.Deactivate();

	for (AngularPart &angular_part : mAngularConstraint)
		angular_part.Deactivate();
}

void SpringCastConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	const EMotionType motion_type1 = mBody->GetMotionType();
	const bool has_body_velocity = motion_type1 != EMotionType::Static;
	MotionProperties *motion_properties1 = mBody->GetMotionPropertiesUnchecked();
	Vec3 linear_velocity1, angular_velocity1;
	if (has_body_velocity)
	{
		linear_velocity1 = motion_properties1->GetLinearVelocity();
		angular_velocity1 = motion_properties1->GetAngularVelocity();
	}
	else
	{
		linear_velocity1 = Vec3::sZero();
		angular_velocity1 = Vec3::sZero();
	}

	float inv_m1;
	if (motion_type1 == EMotionType::Dynamic)
		inv_m1 = motion_properties1->GetInverseMass();
	else
		inv_m1 = 0.0f;

	for (uint body_index = 0; body_index < mContactBody.size(); ++body_index)
	{
		Body *contact_body = mContactBody[body_index];
		if (contact_body == nullptr)
			continue;

		const Vec3 contact_normal = mContactNormal[body_index];
		AxisConstraintPart &spring_part = mSpringPart[body_index * 2];
		AxisConstraintPart &spring_hard_part = mSpringPart[body_index * 2 + 1];

		if (spring_part.IsActive())
		{
			spring_part.WarmStart(*mBody, *contact_body, contact_normal, inWarmStartImpulseRatio);
			if (has_body_velocity)
			{
				linear_velocity1 = motion_properties1->GetLinearVelocity();
				angular_velocity1 = motion_properties1->GetAngularVelocity();
			}
		}
		if (spring_hard_part.IsActive())
		{
			spring_hard_part.WarmStart(*mBody, *contact_body, contact_normal, inWarmStartImpulseRatio);
			if (has_body_velocity)
			{
				linear_velocity1 = motion_properties1->GetLinearVelocity();
				angular_velocity1 = motion_properties1->GetAngularVelocity();
			}
		}

		const Vec3 t1 = contact_normal.GetNormalizedPerpendicular();
		const Vec3 t2 = contact_normal.Cross(t1);

		bool any_impulse_applied = false;
		const EMotionType motion_type2 = contact_body->GetMotionType();
		MotionProperties *motion_properties2 = contact_body->GetMotionPropertiesUnchecked();

		Vec3 linear_velocity2, angular_velocity2;
		if (motion_type2 != EMotionType::Static)
		{
			linear_velocity2 = motion_properties2->GetLinearVelocity();
			angular_velocity2 = motion_properties2->GetAngularVelocity();
		}
		else
		{
			linear_velocity2 = Vec3::sZero();
			angular_velocity2 = Vec3::sZero();
		}

		float inv_m2;
		if (motion_type2 == EMotionType::Dynamic)
			inv_m2 = motion_properties2->GetInverseMass();
		else
			inv_m2 = 0.0f;

		FrictionPart &friction1 = mFrictionConstraint[body_index * 2];
		FrictionPart &friction2 = mFrictionConstraint[body_index * 2 + 1];
		AngularPart &angular_friction = mAngularConstraint[body_index];

		if (friction1.IsActive() && friction1.WarmStart(linear_velocity1, angular_velocity1, linear_velocity2, angular_velocity2, inv_m1, inv_m2, t1, inWarmStartImpulseRatio))
			any_impulse_applied = true;
		if (friction2.IsActive() && friction2.WarmStart(linear_velocity1, angular_velocity1, linear_velocity2, angular_velocity2, inv_m1, inv_m2, t2, inWarmStartImpulseRatio))
			any_impulse_applied = true;
		if (angular_friction.IsActive() && angular_friction.WarmStart(angular_velocity1, angular_velocity2, inWarmStartImpulseRatio))
			any_impulse_applied = true;

		if (any_impulse_applied)
		{
			if (motion_type1 == EMotionType::Dynamic)
			{
				motion_properties1->ApplyLinearVelocityStep(linear_velocity1);
				motion_properties1->ApplyAngularVelocityStep(angular_velocity1);
			}

			if (motion_type2 == EMotionType::Dynamic)
			{
				motion_properties2->ApplyLinearVelocityStep(linear_velocity2);
				motion_properties2->ApplyAngularVelocityStep(angular_velocity2);
			}
		}
	}
}

bool SpringCastConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	const EMotionType motion_type1 = mBody->GetMotionType();
	const bool has_body_velocity = motion_type1 != EMotionType::Static;
	MotionProperties *motion_properties1 = mBody->GetMotionPropertiesUnchecked();
	Vec3 linear_velocity1, angular_velocity1;
	if (has_body_velocity)
	{
		linear_velocity1 = motion_properties1->GetLinearVelocity();
		angular_velocity1 = motion_properties1->GetAngularVelocity();
	}
	else
	{
		linear_velocity1 = Vec3::sZero();
		angular_velocity1 = Vec3::sZero();
	}

	float inv_m1;
	if (motion_type1 == EMotionType::Dynamic)
		inv_m1 = motion_properties1->GetInverseMass();
	else
		inv_m1 = 0.0f;

	bool impulse = false;
	for (uint sorted_index = 0; sorted_index < mContactBodyIDSorted.size(); ++sorted_index)
	{
		const uint body_index = mContactBodyIDSorted[sorted_index];
		Body *contact_body = mContactBody[body_index];
		if (contact_body == nullptr)
			continue;

		const Vec3 contact_normal = mContactNormal[body_index];
		const Vec3 t1 = contact_normal.GetNormalizedPerpendicular();
		const Vec3 t2 = contact_normal.Cross(t1);

		const EMotionType motion_type2 = contact_body->GetMotionType();
		MotionProperties *motion_properties2 = contact_body->GetMotionPropertiesUnchecked();

		Vec3 linear_velocity2, angular_velocity2;
		if (motion_type2 != EMotionType::Static)
		{
			linear_velocity2 = motion_properties2->GetLinearVelocity();
			angular_velocity2 = motion_properties2->GetAngularVelocity();
		}
		else
		{
			linear_velocity2 = Vec3::sZero();
			angular_velocity2 = Vec3::sZero();
		}

		float inv_m2;
		if (motion_type2 == EMotionType::Dynamic)
			inv_m2 = motion_properties2->GetInverseMass();
		else
			inv_m2 = 0.0f;

		AxisConstraintPart &spring_part = mSpringPart[body_index * 2];
		AxisConstraintPart &spring_hard_part = mSpringPart[body_index * 2 + 1];

		FrictionPart &friction1 = mFrictionConstraint[body_index * 2];
		FrictionPart &friction2 = mFrictionConstraint[body_index * 2 + 1];
		AngularPart &angular_friction = mAngularConstraint[body_index];

		const bool linear_friction_active = friction1.IsActive() || friction2.IsActive();
		const bool angular_friction_active = angular_friction.IsActive();

		const float combined_friction = mCombineFriction(*mBody, SubShapeID(), *contact_body, SubShapeID());
		float max_linear_lambda, max_angular_lambda;
		if (linear_friction_active || angular_friction_active)
		{
			float lambda = spring_part.GetTotalLambda() + spring_hard_part.GetTotalLambda();
			max_linear_lambda = lambda * combined_friction;
			max_angular_lambda = mContactFrictionDistance[body_index] * lambda * combined_friction;
		}
		else
		{
			max_linear_lambda = 0.0f;
			max_angular_lambda = 0.0f;
		}

		bool any_impulse_applied = false;

		if (linear_friction_active)
		{
			float lambda1 = friction1.SolveVelocityConstraintGetTotalLambda(linear_velocity1, angular_velocity1, linear_velocity2, angular_velocity2, t1);
			float lambda2 = friction2.SolveVelocityConstraintGetTotalLambda(linear_velocity1, angular_velocity1, linear_velocity2, angular_velocity2, t2);

			float total_lambda_sq = Square(lambda1) + Square(lambda2);
			if (total_lambda_sq > Square(max_linear_lambda))
			{
				float scale = max_linear_lambda / Sqrt(total_lambda_sq);
				lambda1 *= scale;
				lambda2 *= scale;
			}

			if (friction1.SolveVelocityConstraintApplyLambda(linear_velocity1, angular_velocity1, linear_velocity2, angular_velocity2, inv_m1, inv_m2, t1, lambda1))
			{
				mFrictionLambdaDirty = true;
				any_impulse_applied = true;
			}
			if (friction2.SolveVelocityConstraintApplyLambda(linear_velocity1, angular_velocity1, linear_velocity2, angular_velocity2, inv_m1, inv_m2, t2, lambda2))
			{
				mFrictionLambdaDirty = true;
				any_impulse_applied = true;
			}
		}

		if (angular_friction_active && angular_friction.SolveVelocityConstraint(angular_velocity1, angular_velocity2, contact_normal, -max_angular_lambda, max_angular_lambda))
		{
			mFrictionAngularLambdaDirty = true;
			any_impulse_applied = true;
		}

		if (any_impulse_applied)
		{
			if (motion_type1 == EMotionType::Dynamic)
			{
				motion_properties1->ApplyLinearVelocityStep(linear_velocity1);
				motion_properties1->ApplyAngularVelocityStep(angular_velocity1);
			}

			if (motion_type2 == EMotionType::Dynamic)
			{
				motion_properties2->ApplyLinearVelocityStep(linear_velocity2);
				motion_properties2->ApplyAngularVelocityStep(angular_velocity2);
			}
		}

		if (spring_part.IsActive() && spring_part.SolveVelocityConstraint(*mBody, *contact_body, contact_normal, 0.0f, FLT_MAX))
		{
			mSpringLambdaDirty = true;
			any_impulse_applied = true;
		}
		if (spring_hard_part.IsActive() && spring_hard_part.SolveVelocityConstraint(*mBody, *contact_body, contact_normal, 0.0f, FLT_MAX))
		{
			mSpringLambdaDirty = true;
			any_impulse_applied = true;
		}

		impulse |= any_impulse_applied;
	}

	return impulse;
}

bool SpringCastConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	bool impulse = false;

	const RMat44 body_transform = mBody->GetWorldTransform();
	const Vec3 spring_ws_direction = body_transform.Multiply3x3(mSettings->mDirection);
	const RVec3 spring_ws_position = body_transform * mSettings->mPosition;
	const RVec3 spring_min_pos = spring_ws_position + mSettings->mMinLength * spring_ws_direction;

	for (uint sorted_index = 0; sorted_index < mContactBodyIDSorted.size(); ++sorted_index)
	{
		const uint body_index = mContactBodyIDSorted[sorted_index];
		Body *contact_body = mContactBody[body_index];
		if (contact_body == nullptr)
			continue;

		const Vec3 contact_normal = mContactNormal[body_index];
		AxisConstraintPart &spring_hard_part = mSpringPart[body_index * 2 + 1];

		float max_up_error = float(RVec3(-contact_normal).Dot(spring_min_pos) - mPlaneConstant[body_index]);
		if (max_up_error < 0.0f)
		{
			Vec3 r1_plus_u, r2;
			CalculateSpringForcePoint(body_index, r1_plus_u, r2);
			spring_hard_part.CalculateConstraintProperties(*mBody, r1_plus_u, *contact_body, r2, contact_normal);
			impulse |= spring_hard_part.SolvePositionConstraint(*mBody, *contact_body, contact_normal, max_up_error, inBaumgarte);
		}
	}

	return impulse;
}

#ifdef JPH_DEBUG_RENDERER
void SpringCastConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{

}
#endif // JPH_DEBUG_RENDERER

void SpringCastConstraint::SaveState(StateRecorder &inStream) const
{
	Constraint::SaveState(inStream);

	for (const AxisConstraintPart &spring_part : mSpringPart)
		spring_part.SaveState(inStream);

	for (const FrictionPart &friction_part : mFrictionConstraint)
		friction_part.SaveState(inStream);

	for (const AngularPart &angular_part : mAngularConstraint)
		angular_part.SaveState(inStream);

	inStream.Write(mCurrentStep);
}

void SpringCastConstraint::RestoreState(StateRecorder &inStream)
{
	Constraint::RestoreState(inStream);

	for (AxisConstraintPart &spring_part : mSpringPart)
		spring_part.RestoreState(inStream);

	for (FrictionPart &friction_part : mFrictionConstraint)
		friction_part.RestoreState(inStream);

	for (AngularPart &angular_part : mAngularConstraint)
		angular_part.RestoreState(inStream);

	inStream.Read(mCurrentStep);
}

Ref<ConstraintSettings> SpringCastConstraint::GetConstraintSettings() const
{
	SpringCastConstraintSettings *settings = new SpringCastConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mPosition = mSettings->mPosition;
	settings->mForcePoint = mSettings->mForcePoint;
	settings->mDirection = mSettings->mDirection;
	settings->mMinLength = mSettings->mMinLength;
	settings->mMaxLength = mSettings->mMaxLength;
	settings->mSpringSettings = mSettings->mSpringSettings;
	settings->mRadius = mSettings->mRadius;
	settings->mEnableForcePoint = mSettings->mEnableForcePoint;
	return settings;
}

JPH_NAMESPACE_END
