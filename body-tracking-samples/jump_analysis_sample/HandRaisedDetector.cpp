// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "HandRaisedDetector.h"
#include <iostream>
using namespace std;
using namespace std::chrono;

// 손 감지 됐을 때 저장
int rightDetected = 0;

void HandRaisedDetector::UpdateData(k4abt_body_t selectedBody, uint64_t currentTimestampUsec)
{   
    rightDetected++;
    cout << rightDetected << endl;
    k4a_float3_t leftWristJoint = selectedBody.skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position;
    k4a_float3_t rightWristJoint = selectedBody.skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position;
    k4a_float3_t headJoint = selectedBody.skeleton.joints[K4ABT_JOINT_HEAD].position;
    k4a_quaternion_t rightWristJointQ = selectedBody.skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].orientation;

    // Notice: y direction is pointing towards the ground! So jointA.y < jointB.y means jointA is higher than jointB
    bool bothHandsAreRaised = true;

    cout << rightWristJoint.xyz.x << endl;
    cout<< rightWristJoint.xyz.y << endl;
    cout << rightWristJoint.xyz.z << endl;
    cout << rightWristJointQ.v[0] << endl;
    cout << rightWristJointQ.v[1] << endl;
    cout << rightWristJointQ.v[2] << endl;
    cout << rightWristJointQ.v[3] << endl;

    microseconds currentTimestamp(currentTimestampUsec);
    if (m_previousTimestamp == microseconds::zero())
    {
        m_previousTimestamp = currentTimestamp;
        m_handRaisedTimeSpan = microseconds::zero();
    }

    if (!m_bothHandsAreRaised && bothHandsAreRaised)
    {
        // Start accumulating the hand raising time
        m_handRaisedTimeSpan += currentTimestamp - m_previousTimestamp;
        if (m_handRaisedTimeSpan > m_stableTime)
        {
            m_bothHandsAreRaised = bothHandsAreRaised;
        }
    }
    else if (!bothHandsAreRaised)
    {
        // Stop the time accumulation immediately when hands are put down
        m_bothHandsAreRaised = false;
        m_previousTimestamp = microseconds::zero();
        m_handRaisedTimeSpan = microseconds::zero();
    }
}
