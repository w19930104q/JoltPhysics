// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include "UnitTestFramework.h"
#include "PhysicsTestContext.h"
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include "Layers.h"

TEST_SUITE("WheeledVehicleTests")
{
	enum
	{
		FL_WHEEL,
		FR_WHEEL,
		BL_WHEEL,
		BR_WHEEL
	};

	// Simplified vehicle settings
	struct VehicleSettings
	{
		RVec3		mPosition { 0, 2, 0 };
		bool		mUseCastSphere = true;
		float		mWheelRadius = 0.3f;
		float		mWheelWidth = 0.1f;
		float		mHalfVehicleLength = 2.0f;
		float		mHalfVehicleWidth = 0.9f;
		float		mHalfVehicleHeight = 0.2f;
		float		mWheelOffsetHorizontal = 1.4f;
		float		mWheelOffsetVertical = 0.18f;
		float		mSuspensionMinLength = 0.3f;
		float		mSuspensionMaxLength = 0.5f;
		float		mMaxSteeringAngle = DegreesToRadians(30);
		bool		mFourWheelDrive = false;
		float		mFrontBackLimitedSlipRatio = 1.4f;
		float		mLeftRightLimitedSlipRatio = 1.4f;
		bool		mAntiRollbar = true;
	};

	// Helper function to create a vehicle
	static VehicleConstraint *AddVehicle(PhysicsTestContext &inContext, VehicleSettings &inSettings)
	{
		// Create vehicle body
		RefConst<Shape> car_shape = OffsetCenterOfMassShapeSettings(Vec3(0, -inSettings.mHalfVehicleHeight, 0), new BoxShape(Vec3(inSettings.mHalfVehicleWidth, inSettings.mHalfVehicleHeight, inSettings.mHalfVehicleLength))).Create().Get();
		BodyCreationSettings car_body_settings(car_shape, inSettings.mPosition, Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
		car_body_settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
		car_body_settings.mMassPropertiesOverride.mMass = 1500.0f;
		Body *car_body = inContext.GetBodyInterface().CreateBody(car_body_settings);
		inContext.GetBodyInterface().AddBody(car_body->GetID(), EActivation::Activate);

		// Create vehicle constraint
		VehicleConstraintSettings vehicle;
		vehicle.mDrawConstraintSize = 0.1f;
		vehicle.mMaxPitchRollAngle = DegreesToRadians(60.0f);

		// Wheels
		WheelSettingsWV *fl = new WheelSettingsWV;
		fl->mPosition = Vec3(inSettings.mHalfVehicleWidth, -inSettings.mWheelOffsetVertical, inSettings.mWheelOffsetHorizontal);
		fl->mMaxSteerAngle = inSettings.mMaxSteeringAngle;
		fl->mMaxHandBrakeTorque = 0.0f; // Front wheel doesn't have hand brake

		WheelSettingsWV *fr = new WheelSettingsWV;
		fr->mPosition = Vec3(-inSettings.mHalfVehicleWidth, -inSettings.mWheelOffsetVertical, inSettings.mWheelOffsetHorizontal);
		fr->mMaxSteerAngle = inSettings.mMaxSteeringAngle;
		fr->mMaxHandBrakeTorque = 0.0f; // Front wheel doesn't have hand brake

		WheelSettingsWV *bl = new WheelSettingsWV;
		bl->mPosition = Vec3(inSettings.mHalfVehicleWidth, -inSettings.mWheelOffsetVertical, -inSettings.mWheelOffsetHorizontal);
		bl->mMaxSteerAngle = 0.0f;

		WheelSettingsWV *br = new WheelSettingsWV;
		br->mPosition = Vec3(-inSettings.mHalfVehicleWidth, -inSettings.mWheelOffsetVertical, -inSettings.mWheelOffsetHorizontal);
		br->mMaxSteerAngle = 0.0f;

		vehicle.mWheels.resize(4);
		vehicle.mWheels[FL_WHEEL] = fl;
		vehicle.mWheels[FR_WHEEL] = fr;
		vehicle.mWheels[BL_WHEEL] = bl;
		vehicle.mWheels[BR_WHEEL] = br;

		for (WheelSettings *w : vehicle.mWheels)
		{
			w->mRadius = inSettings.mWheelRadius;
			w->mWidth = inSettings.mWheelWidth;
			w->mSuspensionMinLength = inSettings.mSuspensionMinLength;
			w->mSuspensionMaxLength = inSettings.mSuspensionMaxLength;
		}

		WheeledVehicleControllerSettings *controller = new WheeledVehicleControllerSettings;
		vehicle.mController = controller;

		// Differential
		controller->mDifferentials.resize(inSettings.mFourWheelDrive? 2 : 1);
		controller->mDifferentials[0].mLeftWheel = FL_WHEEL;
		controller->mDifferentials[0].mRightWheel = FR_WHEEL;
		controller->mDifferentials[0].mLimitedSlipRatio = inSettings.mLeftRightLimitedSlipRatio;
		controller->mDifferentialLimitedSlipRatio = inSettings.mFrontBackLimitedSlipRatio;
		if (inSettings.mFourWheelDrive)
		{
			controller->mDifferentials[1].mLeftWheel = BL_WHEEL;
			controller->mDifferentials[1].mRightWheel = BR_WHEEL;
			controller->mDifferentials[1].mLimitedSlipRatio = inSettings.mLeftRightLimitedSlipRatio;

			// Split engine torque
			controller->mDifferentials[0].mEngineTorqueRatio = controller->mDifferentials[1].mEngineTorqueRatio = 0.5f;
		}

		// Anti rollbars
		if (inSettings.mAntiRollbar)
		{
			vehicle.mAntiRollBars.resize(2);
			vehicle.mAntiRollBars[0].mLeftWheel = FL_WHEEL;
			vehicle.mAntiRollBars[0].mRightWheel = FR_WHEEL;
			vehicle.mAntiRollBars[1].mLeftWheel = BL_WHEEL;
			vehicle.mAntiRollBars[1].mRightWheel = BR_WHEEL;
		}

		// Create the constraint
		VehicleConstraint *constraint = new VehicleConstraint(*car_body, vehicle);

		// Create collision tester
		RefConst<VehicleCollisionTester> tester;
		if (inSettings.mUseCastSphere)
			tester = new VehicleCollisionTesterCastSphere(Layers::MOVING, 0.5f * inSettings.mWheelWidth);
		else
			tester = new VehicleCollisionTesterRay(Layers::MOVING);
		constraint->SetVehicleCollisionTester(tester);

		// Add to the world
		inContext.GetSystem()->AddConstraint(constraint);
		inContext.GetSystem()->AddStepListener(constraint);
		return constraint;
	}

	static void CheckOnGround(VehicleConstraint *inConstraint, const VehicleSettings &inSettings, const BodyID &inGroundID)
	{
		// Between min and max suspension length
		RVec3 pos = inConstraint->GetVehicleBody()->GetPosition();
		CHECK(pos.GetY() > inSettings.mSuspensionMinLength + inSettings.mWheelOffsetVertical + inSettings.mHalfVehicleHeight);
		CHECK(pos.GetY() < inSettings.mSuspensionMaxLength + inSettings.mWheelOffsetVertical + inSettings.mHalfVehicleHeight);

		// Wheels touching ground
		for (const Wheel *w : inConstraint->GetWheels())
			CHECK(w->GetContactBodyID() == inGroundID);
	}

	TEST_CASE("TestBasicWheeledVehicle")
	{
		PhysicsTestContext c;
		BodyID floor_id = c.CreateFloor().GetID();

		VehicleSettings settings;
		VehicleConstraint *constraint = AddVehicle(c, settings);
		Body *body = constraint->GetVehicleBody();
		WheeledVehicleController *controller = static_cast<WheeledVehicleController *>(constraint->GetController());

		// Should start at specified position
		CHECK_APPROX_EQUAL(body->GetPosition(), settings.mPosition);

		// After 1 step we should not be at ground yet
		c.SimulateSingleStep();
		for (const Wheel *w : constraint->GetWheels())
			CHECK(w->GetContactBodyID().IsInvalid());
		CHECK(controller->GetTransmission().GetCurrentGear() == 0);

		// After 1 second we should be on ground but not moving horizontally
		c.Simulate(1.0f);
		CheckOnGround(constraint, settings, floor_id);
		RVec3 pos1 = body->GetPosition();
		CHECK_APPROX_EQUAL(pos1.GetX(), 0); // Not moving horizontally
		CHECK_APPROX_EQUAL(pos1.GetZ(), 0);
		CHECK(controller->GetTransmission().GetCurrentGear() == 0);

		// Start driving forward
		controller->SetDriverInput(1.0f, 0.0f, 0.0f, 0.0f);
		c.GetBodyInterface().ActivateBody(body->GetID());
		c.Simulate(2.0f);
		CheckOnGround(constraint, settings, floor_id);
		RVec3 pos2 = body->GetPosition();
		CHECK_APPROX_EQUAL(pos2.GetX(), 0, 1.0e-2_r); // Not moving left/right
		CHECK(pos2.GetZ() > pos1.GetZ() + 1.0f); // Moving in Z direction
		Vec3 vel = body->GetLinearVelocity();
		CHECK_APPROX_EQUAL(vel.GetX(), 0, 2.0e-2f); // Not moving left/right
		CHECK(vel.GetZ() > 1.0f); // Moving in Z direction
		CHECK(controller->GetTransmission().GetCurrentGear() > 0);

		// Brake
		controller->SetDriverInput(0.0f, 0.0f, 1.0f, 0.0f);
		c.GetBodyInterface().ActivateBody(body->GetID());
		c.Simulate(5.0f);
		CheckOnGround(constraint, settings, floor_id);
		CHECK(!body->IsActive()); // Car should have gone to sleep
		RVec3 pos3 = body->GetPosition();
		CHECK_APPROX_EQUAL(pos3.GetX(), 0, 2.0e-2_r); // Not moving left/right
		CHECK(pos3.GetZ() > pos2.GetZ() + 1.0f); // Moving in Z direction while braking
		vel = body->GetLinearVelocity();
		CHECK_APPROX_EQUAL(vel, Vec3::sZero(), 1.0e-3f); // Not moving

		// Start driving backwards
		controller->SetDriverInput(-1.0f, 0.0f, 0.0f, 0.0f);
		c.GetBodyInterface().ActivateBody(body->GetID());
		c.Simulate(2.0f);
		CheckOnGround(constraint, settings, floor_id);
		RVec3 pos4 = body->GetPosition();
		CHECK_APPROX_EQUAL(pos4.GetX(), 0, 3.0e-2_r); // Not moving left/right
		CHECK(pos4.GetZ() < pos3.GetZ() - 1.0f); // Moving in -Z direction
		vel = body->GetLinearVelocity();
		CHECK_APPROX_EQUAL(vel.GetX(), 0, 5.0e-2f); // Not moving left/right
		CHECK(vel.GetZ() < -1.0f); // Moving in -Z direction
		CHECK(controller->GetTransmission().GetCurrentGear() < 0);

		// Brake
		controller->SetDriverInput(0.0f, 0.0f, 1.0f, 0.0f);
		c.GetBodyInterface().ActivateBody(body->GetID());
		c.Simulate(5.0f);
		CheckOnGround(constraint, settings, floor_id);
		CHECK(!body->IsActive()); // Car should have gone to sleep
		RVec3 pos5 = body->GetPosition();
		CHECK_APPROX_EQUAL(pos5.GetX(), 0, 7.0e-2_r); // Not moving left/right
		CHECK(pos5.GetZ() < pos4.GetZ() - 1.0f); // Moving in -Z direction while braking
		vel = body->GetLinearVelocity();
		CHECK_APPROX_EQUAL(vel, Vec3::sZero(), 1.0e-3f); // Not moving

		// Turn right
		controller->SetDriverInput(1.0f, 1.0f, 0.0f, 0.0f);
		c.GetBodyInterface().ActivateBody(body->GetID());
		c.Simulate(2.0f);
		CheckOnGround(constraint, settings, floor_id);
		Vec3 omega = body->GetAngularVelocity();
		CHECK(omega.GetY() < -0.4f); // Rotating right
		CHECK(controller->GetTransmission().GetCurrentGear() > 0);

		// Hand brake
		controller->SetDriverInput(0.0f, 0.0f, 0.0f, 1.0f);
		c.GetBodyInterface().ActivateBody(body->GetID());
		c.Simulate(7.0f);
		CheckOnGround(constraint, settings, floor_id);
		CHECK(!body->IsActive()); // Car should have gone to sleep
		vel = body->GetLinearVelocity();
		CHECK_APPROX_EQUAL(vel, Vec3::sZero(), 1.0e-3f); // Not moving

		// Turn left
		controller->SetDriverInput(1.0f, -1.0f, 0.0f, 0.0f);
		c.GetBodyInterface().ActivateBody(body->GetID());
		c.Simulate(2.0f);
		CheckOnGround(constraint, settings, floor_id);
		omega = body->GetAngularVelocity();
		CHECK(omega.GetY() > 0.4f); // Rotating left
		CHECK(controller->GetTransmission().GetCurrentGear() > 0);
	}

	TEST_CASE("TestLSDifferential")
	{
		struct Test
		{
			RVec3	mBlockPosition;		// Location of the box under the vehicle
			bool	mFourWheelDrive;	// 4WD or not
			float	mFBLSRatio;			// Limited slip ratio front-back
			float	mLRLSRatio;			// Limited slip ratio left-right
			bool	mFLHasContactPre;	// Which wheels should be in contact with the ground prior to the test
			bool	mFRHasContactPre;
			bool	mBLHasContactPre;
			bool	mBRHasContactPre;
			bool	mShouldMove;		// If the vehicle should be able to drive off the block
		};

		Test tests[] = {
			// Block Position,			4WD,	FBSlip,		LRSlip		FLPre,	FRPre,	BLPre,	BRPre,	ShouldMove
			{ RVec3(1, 0.5f, 0),		true,	FLT_MAX,	FLT_MAX,	false,	true,	false,	true,	false	},		// Block left, no limited slip -> vehicle can't move
			{ RVec3(1, 0.5f, 0),		true,	1.4f,		FLT_MAX,	false,	true,	false,	true,	false	},		// Block left, only FB limited slip -> vehicle can't move
			{ RVec3(1, 0.5f, 0),		true,	1.4f,		1.4f,		false,	true,	false,	true,	true	},		// Block left, limited slip -> vehicle drives off
			{ RVec3(-1, 0.5f, 0),		true,	FLT_MAX,	FLT_MAX,	true,	false,	true,	false,	false	},		// Block right, no limited slip -> vehicle can't move
			{ RVec3(-1, 0.5f, 0),		true,	1.4f,		FLT_MAX,	true,	false,	true,	false,	false	},		// Block right, only FB limited slip -> vehicle can't move
			{ RVec3(-1, 0.5f, 0),		true,	1.4f,		1.4f,		true,	false,	true,	false,	true	},		// Block right, limited slip -> vehicle drives off
			{ RVec3(0, 0.5f, 1.5f),		true,	FLT_MAX,	FLT_MAX,	false,	false,	true,	true,	false	},		// Block front, no limited slip -> vehicle can't move
			{ RVec3(0, 0.5f, 1.5f),		true,	1.4f,		FLT_MAX,	false,	false,	true,	true,	true	},		// Block front, only FB limited slip -> vehicle drives off
			{ RVec3(0, 0.5f, 1.5f),		true,	1.4f,		1.4f,		false,	false,	true,	true,	true	},		// Block front, limited slip -> vehicle drives off
			{ RVec3(0, 0.5f, 1.5f),		false,	1.4f,		1.4f,		false,	false,	true,	true,	false	},		// Block front, limited slip, 2WD -> vehicle can't move
			{ RVec3(0, 0.5f, -1.5f),	true,	FLT_MAX,	FLT_MAX,	true,	true,	false,	false,	false	},		// Block back, no limited slip -> vehicle can't move
			{ RVec3(0, 0.5f, -1.5f),	true,	1.4f,		FLT_MAX,	true,	true,	false,	false,	true	},		// Block back, only FB limited slip -> vehicle drives off
			{ RVec3(0, 0.5f, -1.5f),	true,	1.4f,		1.4f,		true,	true,	false,	false,	true	},		// Block back, limited slip -> vehicle drives off
			{ RVec3(0, 0.5f, -1.5f),	false,	1.4f,		1.4f,		true,	true,	false,	false,	true	},		// Block back, limited slip, 2WD -> vehicle drives off
		};

		for (Test &t : tests)
		{
			PhysicsTestContext c;
			BodyID floor_id = c.CreateFloor().GetID();

			// Box under left side of the vehicle, left wheels won't be touching the ground
			Body &box = c.CreateBox(t.mBlockPosition, Quat::sIdentity(), EMotionType::Static, EMotionQuality::Discrete, Layers::NON_MOVING, Vec3::sReplicate(0.5f));
			box.SetFriction(1.0f);

			// Create vehicle
			VehicleSettings settings;
			settings.mFourWheelDrive = t.mFourWheelDrive;
			settings.mFrontBackLimitedSlipRatio = t.mFBLSRatio;
			settings.mLeftRightLimitedSlipRatio = t.mLRLSRatio;

			VehicleConstraint *constraint = AddVehicle(c, settings);
			Body *body = constraint->GetVehicleBody();
			WheeledVehicleController *controller = static_cast<WheeledVehicleController *>(constraint->GetController());

			// Give the wheels extra grip
			controller->SetTireMaxImpulseCallback(
				[](uint, float &outLongitudinalImpulse, float &outLateralImpulse, float inSuspensionImpulse, float inLongitudinalFriction, float inLateralFriction, float, float, float)
				{
					outLongitudinalImpulse = 10.0f * inLongitudinalFriction * inSuspensionImpulse;
					outLateralImpulse = inLateralFriction * inSuspensionImpulse;
				});

			// Simulate till vehicle rests on block
			bool vehicle_on_floor = false;
			for (float time = 0; time < 2.0f; time += c.GetDeltaTime())
			{
				c.SimulateSingleStep();

				// Check pre condition
				if ((constraint->GetWheel(FL_WHEEL)->GetContactBodyID() == (t.mFLHasContactPre? floor_id : BodyID()))
					&& (constraint->GetWheel(FR_WHEEL)->GetContactBodyID() == (t.mFRHasContactPre? floor_id : BodyID()))
					&& (constraint->GetWheel(BL_WHEEL)->GetContactBodyID() == (t.mBLHasContactPre? floor_id : BodyID()))
					&& (constraint->GetWheel(BR_WHEEL)->GetContactBodyID() == (t.mBRHasContactPre? floor_id : BodyID())))
				{
					vehicle_on_floor = true;
					break;
				}
			}
			CHECK(vehicle_on_floor);
			CHECK_APPROX_EQUAL(body->GetPosition().GetZ(), 0, 0.03_r);

			// Start driving
			controller->SetDriverInput(1.0f, 0, 0, 0);
			c.GetBodyInterface().ActivateBody(body->GetID());
			c.Simulate(2.0f);

			// Check if vehicle had traction
			if (t.mShouldMove)
				CHECK(body->GetPosition().GetZ() > 0.5f);
			else
				CHECK_APPROX_EQUAL(body->GetPosition().GetZ(), 0, 0.06_r);
		}
	}

	// Test that the runtime vehicle state (steering pose, wheel rotation, engine/transmission state,
	// wheel collision test cadence and gravity override) survives a save/restore cycle and that
	// a corrupt snapshot is rejected without affecting the live vehicle.
	TEST_CASE("TestVehicleSaveRestoreState")
	{
		// Runtime state as recorded by the "vehicle-v6" snapshot
		constexpr float cWheelAngle = 0.75f;					// Wheel rotation angle (rad)
		constexpr float cWheelAngularVelocity = 9.0f;		// Wheel rotation speed (rad/s)
		constexpr float cSteerAngle = DegreesToRadians(17.0f); // Steering angle (rad)
		constexpr int cGear = 3;							// Third gear
		constexpr float cClutchFriction = 0.5f;				// Half engaged clutch
		constexpr uint cCollisionTestInterval = 3;			// Full wheel collision test every 3 steps
		const Vec3 cGravityOverride(1.5f, -4.0f, 2.5f); // Custom gravity for the vehicle
		constexpr float cGravityFactor = 0.0f;				// Overridden gravity drives the body factor to 0

		PhysicsTestContext c;
		c.CreateFloor();

		VehicleSettings settings;
		VehicleConstraint *constraint = AddVehicle(c, settings);
		Body *body = constraint->GetVehicleBody();
		WheeledVehicleController *controller = static_cast<WheeledVehicleController *>(constraint->GetController());

		// Let the vehicle settle on its wheels so that wheel contacts exist
		c.Simulate(1.0f);
		for (const Wheel *w : constraint->GetWheels())
			CHECK(w->HasContact());

		// Enable the gravity override and step once so OnStep drives the body's gravity factor to 0
		constraint->OverrideGravity(cGravityOverride);
		c.SimulateSingleStep();
		CHECK(constraint->IsGravityOverridden());
		CHECK_APPROX_EQUAL(constraint->GetGravityOverride(), cGravityOverride);
		CHECK_APPROX_EQUAL(body->GetMotionProperties()->GetGravityFactor(), cGravityFactor);

		// Put the vehicle in the recorded runtime state and take the snapshot immediately:
		// WheeledVehicleController::PreCollide recomputes the steering angle from the driver input every step,
		// so these values must not be followed by another simulation step before saving.
		for (Wheel *w : constraint->GetWheels())
		{
			w->SetSteerAngle(cSteerAngle);
			w->SetRotationAngle(cWheelAngle);
			w->SetAngularVelocity(cWheelAngularVelocity);
		}
		controller->GetTransmission().Set(cGear, cClutchFriction);
		constraint->SetNumStepsBetweenCollisionTestActive(cCollisionTestInterval);
		constraint->SetNumStepsBetweenCollisionTestInactive(cCollisionTestInterval);
		CHECK(constraint->GetNumStepsBetweenCollisionTestActive() == cCollisionTestInterval);

		// Save the runtime state ("vehicle-v6" snapshot)
		StateRecorderImpl snapshot;
		c.GetSystem()->SaveState(snapshot);
		const std::string snapshot_data = snapshot.GetData();

		// Disturb the live state: reset steering/wheel spin, cadence back to every step and gravity back to normal
		for (Wheel *w : constraint->GetWheels())
		{
			w->SetSteerAngle(0.0f);
			w->SetRotationAngle(0.0f);
			w->SetAngularVelocity(0.0f);
		}
		controller->GetTransmission().Set(0, 1.0f);
		constraint->SetNumStepsBetweenCollisionTestActive(1);
		constraint->SetNumStepsBetweenCollisionTestInactive(1);
		constraint->ResetGravityOverride();
		CHECK(!constraint->IsGravityOverridden());
		CHECK_APPROX_EQUAL(body->GetMotionProperties()->GetGravityFactor(), 1.0f);
		c.SimulateSingleStep();

		// Restore the snapshot: all runtime state must come back
		StateRecorderImpl restored;
		restored.WriteBytes(snapshot_data.data(), snapshot_data.size());
		CHECK(c.GetSystem()->RestoreState(restored));

		// Steering pose, wheel rotation angle and speed survive
		for (const Wheel *w : constraint->GetWheels())
		{
			CHECK_APPROX_EQUAL(w->GetSteerAngle(), cSteerAngle);
			CHECK_APPROX_EQUAL(w->GetRotationAngle(), cWheelAngle);
			CHECK_APPROX_EQUAL(w->GetAngularVelocity(), cWheelAngularVelocity);
		}

		// Gear and half clutch survive
		CHECK(controller->GetTransmission().GetCurrentGear() == cGear);
		CHECK_APPROX_EQUAL(controller->GetTransmission().GetClutchFriction(), cClutchFriction);

		// Contact cadence survives (this used to silently fall back to every-step testing)
		CHECK(constraint->GetNumStepsBetweenCollisionTestActive() == cCollisionTestInterval);
		CHECK(constraint->GetNumStepsBetweenCollisionTestInactive() == cCollisionTestInterval);

		// Gravity override survives together with the body's gravity factor
		CHECK(constraint->IsGravityOverridden());
		CHECK_APPROX_EQUAL(constraint->GetGravityOverride(), cGravityOverride);
		CHECK_APPROX_EQUAL(body->GetMotionProperties()->GetGravityFactor(), cGravityFactor);

		// Capturing the restored state again must produce an identical stream
		StateRecorderImpl verify;
		c.GetSystem()->SaveState(verify);
		CHECK(snapshot.IsEqual(verify));

		// After another step the restored vehicle still behaves under the overridden gravity
		c.SimulateSingleStep();
		CHECK(constraint->IsGravityOverridden());
		CHECK_APPROX_EQUAL(body->GetMotionProperties()->GetGravityFactor(), cGravityFactor);
	}

	// Test that a truncated (corrupt) vehicle snapshot is rejected without changing the live vehicle
	TEST_CASE("TestVehicleRestoreTruncatedState")
	{
		constexpr float cSteerAngle = DegreesToRadians(17.0f);
		constexpr uint cCollisionTestInterval = 3;
		const Vec3 cGravityOverride(1.5f, -4.0f, 2.5f);

		PhysicsTestContext c;
		c.CreateFloor();

		VehicleSettings settings;
		VehicleConstraint *constraint = AddVehicle(c, settings);
		Body *body = constraint->GetVehicleBody();
		WheeledVehicleController *controller = static_cast<WheeledVehicleController *>(constraint->GetController());

		c.Simulate(1.0f);

		// Enable the gravity override and step once so the body's gravity factor is driven to 0
		constraint->OverrideGravity(cGravityOverride);
		c.SimulateSingleStep();

		// Put the vehicle in a non-default runtime state and take the keyframe immediately
		// (the controller recomputes steering and the transmission every simulation step)
		for (Wheel *w : constraint->GetWheels())
		{
			w->SetSteerAngle(cSteerAngle);
			w->SetRotationAngle(0.75f);
			w->SetAngularVelocity(9.0f);
		}
		controller->GetTransmission().Set(3, 0.5f);
		constraint->SetNumStepsBetweenCollisionTestActive(cCollisionTestInterval);

		StateRecorderImpl keyframe;
		c.GetSystem()->SaveState(keyframe);
		const std::string keyframe_data = keyframe.GetData();

		// Move the live world into a different state
		c.Simulate(1.0f);
		for (Wheel *w : constraint->GetWheels())
			w->SetSteerAngle(0.0f);
		controller->GetTransmission().Set(0, 1.0f);
		constraint->SetNumStepsBetweenCollisionTestActive(1);
		constraint->ResetGravityOverride();
		c.SimulateSingleStep();

		// Remember the live state that must remain untouched by a bad snapshot
		const RVec3 live_position = body->GetPosition();
		const Quat live_rotation = body->GetRotation();
		const Vec3 live_velocity = body->GetLinearVelocity();
		const float live_steer = constraint->GetWheel(FL_WHEEL)->GetSteerAngle();
		const int live_gear = controller->GetTransmission().GetCurrentGear();
		const uint live_cadence = constraint->GetNumStepsBetweenCollisionTestActive();
		const bool live_gravity_overridden = constraint->IsGravityOverridden();
		const float live_gravity_factor = body->GetMotionProperties()->GetGravityFactor();

		// Truncated snapshots at various cut points must all be rejected ...
		for (float fraction : { 0.0f, 0.1f, 0.5f, 0.9f, 0.99f })
		{
			StateRecorderImpl truncated;
			truncated.WriteBytes(keyframe_data.data(), size_t(fraction * keyframe_data.size()));

			CHECK(!c.GetSystem()->RestoreState(truncated));

			// ... without moving any object or changing any runtime vehicle state
			CHECK(body->GetPosition() == live_position);
			CHECK(body->GetRotation() == live_rotation);
			CHECK(body->GetLinearVelocity() == live_velocity);
			CHECK_APPROX_EQUAL(constraint->GetWheel(FL_WHEEL)->GetSteerAngle(), live_steer);
			CHECK(controller->GetTransmission().GetCurrentGear() == live_gear);
			CHECK(constraint->GetNumStepsBetweenCollisionTestActive() == live_cadence);
			CHECK(constraint->IsGravityOverridden() == live_gravity_overridden);
			CHECK_APPROX_EQUAL(body->GetMotionProperties()->GetGravityFactor(), live_gravity_factor);
		}

		// The vehicle must still be in a consistent state and be able to simulate
		c.Simulate(0.5f);
		CHECK(body->IsInBroadPhase());

		// The complete keyframe can still be restored and brings all runtime state back
		StateRecorderImpl complete;
		complete.WriteBytes(keyframe_data.data(), keyframe_data.size());
		CHECK(c.GetSystem()->RestoreState(complete));
		CHECK_APPROX_EQUAL(constraint->GetWheel(FL_WHEEL)->GetSteerAngle(), cSteerAngle);
		CHECK(controller->GetTransmission().GetCurrentGear() == 3);
		CHECK_APPROX_EQUAL(controller->GetTransmission().GetClutchFriction(), 0.5f);
		CHECK(constraint->GetNumStepsBetweenCollisionTestActive() == cCollisionTestInterval);
		CHECK(constraint->IsGravityOverridden());
		CHECK_APPROX_EQUAL(constraint->GetGravityOverride(), cGravityOverride);
	}
}
