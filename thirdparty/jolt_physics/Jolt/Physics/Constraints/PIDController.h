// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Jolt/Physics/StateRecorder.h>

JPH_NAMESPACE_BEGIN

// A PID Controller (Proportional-Integral-Derivative) used to improve response
class JPH_EXPORT PIDController
{

public:

	/// Update controller with a measured value and target value, returns the new output value
	/// @param inMeasure Measured value
	/// @param inTarget Target value
	/// @param inDelta Time step
	inline float				Update(float inMeasure, float inTarget, float inDelta)
	{
		float error = inTarget - inMeasure;
		mErrorSum += error * inDelta;

		float derivative = error - mLastError;
		mLastError = error;

		mOutput = (error * mProportional + mErrorSum * mIntegral + derivative * mDerivative);

		return mOutput;
	}

	/// Reset the internal state of this controller
	inline void					Reset()
	{
		mOutput = 0.0;
		mLastError = 0.0;
		mErrorSum = 0.0;
	}

	/// Get the current output value
	inline float				GetOutput() const 							{ return mOutput; }

	inline void					SetParameters(float inProportional, float inIntegral, float inDerivative)
	{
		mProportional = inProportional;
		mIntegral = inIntegral;
		mDerivative = inDerivative;
	}

	/// Save state of this PID Controller
	inline void					SaveState(StateRecorder &inStream) const
	{
		inStream.Write(mOutput);
		inStream.Write(mLastError);
		inStream.Write(mErrorSum);
	}

	/// Restore state of this PID Controller
	inline void					RestoreState(StateRecorder &inStream)
	{
		inStream.Read(mOutput);
		inStream.Read(mLastError);
		inStream.Read(mErrorSum);
	}

	// PID parameters
	float						mProportional;		/// Proportional multiplier of the error added to the output
	float						mIntegral;			/// Integral multiplier the error sum added to the output
	float						mDerivative;		/// Derivative multiplier of the change in the error added to the output

private:
	float						mOutput = 0.0f;		/// Current controller output
	float						mLastError = 0.0f;	/// The last calculated error, used for derivative calculation
	float						mErrorSum = 0.0f;	/// Running sum of errors, used for integral calculation
};

JPH_NAMESPACE_END